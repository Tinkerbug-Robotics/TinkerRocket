#include <TR_I2C_Interface.h>
#include <CRC.h>
#include <cstring>
#include <esp_log.h>
#include <esp_idf_version.h>

// This component targets the ESP-IDF V2 I2C slave driver (on_receive /
// on_request + i2c_slave_write). Pre-6.0 it is opt-in, and a *missing* flag
// (Kconfig "not set" => undefined macro, not defined-to-0) silently exposes
// the V1 API — yielding a wall of confusing "no member 'on_receive'" /
// "i2c_slave_write not declared" errors. Key the guard off the IDF version so
// it fires on absent-OR-disabled, not just disabled. (On 6.0+ V2 is the only
// slave driver and the flag no longer exists.)
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
#  if !defined(CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2) || !CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2
#    error "TR_I2C_Interface requires the V2 I2C slave driver. Set CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2=y in sdkconfig.defaults, then regenerate sdkconfig (idf.py set-target <chip>)."
#  endif
#endif

static const char *TAG = "I2C_IF";

// I2C slave TX service task (services the V2 on_request callback by writing
// the staged response). Pinned to the slave ISR's core and run at high
// priority so it clears the V2 address-match SCL stretch well within the
// hardware stretch-protect window (I2C_LL_STRETCH_PROTECT_TIME ~= 2.55 ms at
// 400 kHz) — otherwise the master sees a held bus and wedges on clear_bus.
static constexpr uint32_t    SLAVE_TX_TASK_STACK       = 3072;
static constexpr UBaseType_t SLAVE_TX_TASK_PRIO        = 20;
static constexpr int         SLAVE_TX_WRITE_TIMEOUT_MS = 20;

TR_I2C_Interface::TR_I2C_Interface(uint8_t device_address_7bit)
    : device_address(device_address_7bit),
      _master_bus(nullptr),
      _master_dev(nullptr),
      _slave_dev(nullptr),
      _rx_queue(nullptr),
      _tx_req_queue(nullptr),
      _tx_mux(nullptr),
      _tx_task(nullptr),
      _tx_len(0)
{
}

// ---------------------------------------------------------------------------
//  Master init
// ---------------------------------------------------------------------------
esp_err_t TR_I2C_Interface::beginMaster(int sda_pin,
                                        int scl_pin,
                                        uint32_t clock_hz,
                                        bool enable_internal_pullups)
{
    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = static_cast<gpio_num_t>(sda_pin);
    bus_cfg.scl_io_num = static_cast<gpio_num_t>(scl_pin);
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = enable_internal_pullups;

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &_master_bus);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = device_address;
    dev_cfg.scl_speed_hz    = clock_hz;
    // The OC slave runs the ESP-IDF V2 driver, which stretches SCL on
    // address-match until its on_request TX task calls i2c_slave_write
    // (bounded by the ~2.56 ms hardware stretch-protect timer @ 400 kHz). The
    // master's default SCL-wait (I2C_LL_SCL_WAIT_US_VAL_DEFAULT = 2000 us) is
    // shorter than that window, so reads abort mid-stretch (#88 bench:
    // "[I2C] read FAIL ... dur~2400us", query ok/fail=0/N). Wait past the
    // protect timer so the master tolerates the V2 slave's stretch.
    dev_cfg.scl_wait_us     = 5000;

    err = i2c_master_bus_add_device(_master_bus, &dev_cfg, &_master_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
    }
    return err;
}

// ---------------------------------------------------------------------------
//  Slave init
// ---------------------------------------------------------------------------
esp_err_t TR_I2C_Interface::beginSlave(int sda_pin,
                                       int scl_pin,
                                       uint32_t clock_hz,
                                       size_t rx_buffer_len,
                                       size_t tx_buffer_len,
                                       bool enable_internal_pullups)
{
    (void)clock_hz;  // slave clock is driven by master

    // ISR -> task signals: _rx_queue carries each received frame's byte count;
    // _tx_req_queue carries one token per master-read request. _tx_mux guards
    // the staged TX response shared between writeToSlave() and slaveTxTask().
    _rx_queue     = xQueueCreate(4, sizeof(size_t));
    _tx_req_queue = xQueueCreate(4, sizeof(uint8_t));
    _tx_mux       = xSemaphoreCreateMutex();
    if (_rx_queue == nullptr || _tx_req_queue == nullptr || _tx_mux == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create slave queues/mutex");
        return ESP_ERR_NO_MEM;
    }
    _tx_len = 0;

    i2c_slave_config_t slave_cfg = {};
    slave_cfg.i2c_port = I2C_NUM_0;
    slave_cfg.sda_io_num = static_cast<gpio_num_t>(sda_pin);
    slave_cfg.scl_io_num = static_cast<gpio_num_t>(scl_pin);
    slave_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    slave_cfg.send_buf_depth = tx_buffer_len;     // TX ringbuffer depth
    slave_cfg.receive_buf_depth = rx_buffer_len;  // driver-owned RX ring
    slave_cfg.slave_addr = device_address;
    slave_cfg.addr_bit_len = I2C_ADDR_BIT_LEN_7;
    slave_cfg.intr_priority = 0;
    slave_cfg.flags.enable_internal_pullup = enable_internal_pullups;

    esp_err_t err = i2c_new_slave_device(&slave_cfg, &_slave_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_new_slave_device failed: %s", esp_err_to_name(err));
        return err;
    }

    memset(_slave_rx_buf, 0, SLAVE_RX_BUF_SIZE);

    // Start the TX service task before registering callbacks so the consumer
    // is ready the instant on_request can fire. Pin it to the core this runs
    // on — the slave ISR is allocated on the same core, so on_request -> task
    // wakeups stay on-core and clear the SCL stretch with minimal latency.
    BaseType_t task_ok = xTaskCreatePinnedToCore(slaveTxTask, "i2c_slv_tx",
                                                 SLAVE_TX_TASK_STACK, this,
                                                 SLAVE_TX_TASK_PRIO, &_tx_task,
                                                 xPortGetCoreID());
    if (task_ok != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create I2C slave TX task");
        return ESP_ERR_NO_MEM;
    }

    // Register both halves of the V2 slave protocol: on_receive (master writes
    // land in our RX ring) and on_request (master reads -> wake the TX task).
    // The V2 driver auto-receives, so there is no i2c_slave_receive() arming
    // call as in V1.
    i2c_slave_event_callbacks_t cbs = {};
    cbs.on_receive = slaveReceiveISR;
    cbs.on_request = slaveRequestISR;
    err = i2c_slave_register_event_callbacks(_slave_dev, &cbs, this);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_slave_register_event_callbacks failed: %s", esp_err_to_name(err));
        vTaskDelete(_tx_task);
        _tx_task = nullptr;
        return err;
    }

    return ESP_OK;
}

// ---------------------------------------------------------------------------
//  Slave on_receive ISR (V2): master finished a write transaction.
//  edata->buffer is driver-owned and reused on the next receive, so copy the
//  payload into our class-scoped _slave_rx_buf and post the byte count to
//  _rx_queue. Single-buffer: a second master write before readFromSlave()
//  consumes the first overwrites it — acceptable at the FC->OC poll rate.
// ---------------------------------------------------------------------------
bool IRAM_ATTR TR_I2C_Interface::slaveReceiveISR(
    i2c_slave_dev_handle_t channel,
    const i2c_slave_rx_done_event_data_t *edata,
    void *user_data)
{
    (void)channel;
    auto *self = static_cast<TR_I2C_Interface *>(user_data);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    size_t copy_len = edata->length;
    if (copy_len > SLAVE_RX_BUF_SIZE)
    {
        copy_len = SLAVE_RX_BUF_SIZE;
    }
    memcpy(self->_slave_rx_buf, edata->buffer, copy_len);
    xQueueSendFromISR(self->_rx_queue, &copy_len, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken == pdTRUE;
}

// ---------------------------------------------------------------------------
//  Slave on_request ISR (V2): master is addressing us for a read. We cannot
//  call i2c_slave_write() here (it takes a mutex), so post a token and let the
//  TX service task perform the write — that feeds the master and releases the
//  address-match SCL stretch.
// ---------------------------------------------------------------------------
bool IRAM_ATTR TR_I2C_Interface::slaveRequestISR(
    i2c_slave_dev_handle_t channel,
    const i2c_slave_request_event_data_t *edata,
    void *user_data)
{
    (void)channel;
    (void)edata;
    auto *self = static_cast<TR_I2C_Interface *>(user_data);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    const uint8_t token = 1;
    xQueueSendFromISR(self->_tx_req_queue, &token, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken == pdTRUE;
}

// ---------------------------------------------------------------------------
//  Slave TX service task: blocks on _tx_req_queue, then writes the staged
//  response with i2c_slave_write() (which also clears the SCL stretch). If
//  nothing has been staged yet, it writes a single zero byte so the master's
//  read still completes and the bus is released promptly instead of hanging
//  until the hardware stretch-protect timer.
//
//  Wire-alignment note: i2c_slave_write() pushes exactly _tx_len bytes; the FC
//  reads a fixed size and resyncs on the SOF marker, so a small size mismatch
//  is tolerated. Watch for cumulative drift on the bench — if the staged size
//  and the master read size differ, leftover TX bytes can accumulate in the
//  ringbuffer across polls.
// ---------------------------------------------------------------------------
void TR_I2C_Interface::slaveTxTask(void *arg)
{
    auto *self = static_cast<TR_I2C_Interface *>(arg);
    uint8_t token = 0;

    for (;;)
    {
        if (xQueueReceive(self->_tx_req_queue, &token, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        // Snapshot the staged response under the mutex, then write outside it.
        uint8_t local[SLAVE_TX_BUF_SIZE];
        size_t len = 0;
        if (xSemaphoreTake(self->_tx_mux, portMAX_DELAY) == pdTRUE)
        {
            len = self->_tx_len;
            if (len > SLAVE_TX_BUF_SIZE)
            {
                len = SLAVE_TX_BUF_SIZE;
            }
            if (len > 0)
            {
                memcpy(local, self->_tx_buf, len);
            }
            xSemaphoreGive(self->_tx_mux);
        }

        if (len == 0)
        {
            local[0] = 0x00;
            len = 1;
        }

        uint32_t written = 0;
        esp_err_t werr = i2c_slave_write(self->_slave_dev, local,
                                         static_cast<uint32_t>(len), &written,
                                         SLAVE_TX_WRITE_TIMEOUT_MS);
        // #280: surface a chronically failing serve. The status was discarded,
        // so a TX that timed out every poll looked identical to a healthy one.
        self->_tx_writes++;
        if (werr != ESP_OK || written != len)
        {
            const uint32_t fails = ++self->_tx_write_fails;
            // Rate-limit on the per-poll TX path: shout on the first failure,
            // then every 256th, so a stuck serve stays visible without flooding.
            if (fails == 1 || (fails % 256) == 0)
            {
                ESP_LOGW(TAG, "slave TX write failed: %s, wrote %lu/%lu B "
                              "(fails=%lu of %lu serves)",
                         esp_err_to_name(werr), (unsigned long)written,
                         (unsigned long)len, (unsigned long)fails,
                         (unsigned long)self->_tx_writes);
            }
        }
    }
}

// ---------------------------------------------------------------------------
//  Master: send a framed message
// ---------------------------------------------------------------------------
esp_err_t TR_I2C_Interface::sendMessage(uint8_t type,
                                        const uint8_t *payload,
                                        size_t len,
                                        uint32_t timeout_ms) const
{
    uint8_t frame[MAX_FRAME];
    size_t frame_len = 0;
    if (!packMessage(type,
                     payload,
                     len,
                     frame,
                     sizeof(frame),
                     frame_len))
    {
        return ESP_ERR_INVALID_SIZE;
    }

    return i2c_master_transmit(_master_dev,
                               frame,
                               frame_len,
                               pdMS_TO_TICKS(timeout_ms));
}

esp_err_t TR_I2C_Interface::sendStartLoggingMsg(uint32_t timeout_ms) const
{
    return sendMessage(START_LOGGING,
                       nullptr,
                       0,
                       timeout_ms);
}

esp_err_t TR_I2C_Interface::sendEndOfFlightMsg(uint32_t timeout_ms) const
{
    return sendMessage(END_FLIGHT,
                       nullptr,
                       0,
                       timeout_ms);
}

bool TR_I2C_Interface::getOutReady(uint8_t* out_command,
                                   uint32_t timeout_ms) const
{
    OutStatusQueryData empty = {};
    empty.format_version = 1;
    return getOutReady(empty, out_command, timeout_ms);
}

// ---------------------------------------------------------------------------
//  Slave: read data that was received from master
//  The new API is transaction-based: each master write results in one
//  callback. The protocol uses framed messages (SOF + len), so we parse
//  the frame header to determine valid data length.
// ---------------------------------------------------------------------------
int TR_I2C_Interface::readFromSlave(uint8_t* out_buf,
                                    size_t out_buf_capacity,
                                    uint32_t timeout_ms)
{
    if (out_buf == nullptr || out_buf_capacity == 0 || _rx_queue == nullptr)
    {
        return -1;
    }

    // The on_receive ISR already copied the master-write payload into
    // _slave_rx_buf and posted its byte count. The V2 driver auto-re-arms via
    // its internal RX ring, so there is no explicit re-arming here.
    size_t rx_len = 0;
    if (xQueueReceive(_rx_queue, &rx_len, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return 0; // no data available
    }

    // Determine valid data length from frame protocol:
    // SOF(4) + type(1) + len(1) + payload(len) + CRC(2)
    size_t valid_len = 0;
    if (rx_len >= 6 &&
        _slave_rx_buf[0] == SOF0 && _slave_rx_buf[1] == SOF1 &&
        _slave_rx_buf[2] == SOF2 && _slave_rx_buf[3] == SOF3)
    {
        size_t payload_len = _slave_rx_buf[5];
        valid_len = 4 + 1 + 1 + payload_len + 2; // SOF + type + len + payload + CRC
        if (valid_len > rx_len)
        {
            valid_len = rx_len; // clamp to actually-received bytes
        }
    }
    else
    {
        valid_len = rx_len;
    }

    if (valid_len == 0)
    {
        return 0;
    }

    size_t copy_len = (valid_len < out_buf_capacity) ? valid_len : out_buf_capacity;
    memcpy(out_buf, _slave_rx_buf, copy_len);
    return static_cast<int>(copy_len);
}

// ---------------------------------------------------------------------------
//  Slave: write data for master to read
// ---------------------------------------------------------------------------
int TR_I2C_Interface::writeToSlave(const uint8_t* data,
                                   size_t len,
                                   uint32_t timeout_ms)
{
    if (data == nullptr || len == 0 || _slave_dev == nullptr || _tx_mux == nullptr)
    {
        return -1;
    }
    if (len > SLAVE_TX_BUF_SIZE)
    {
        len = SLAVE_TX_BUF_SIZE; // clamp to staging capacity
    }

    // Stage the latest response; slaveTxTask() serves it when on_request
    // fires. Non-blocking when timeout_ms == 0: if the task momentarily holds
    // the mutex, drop this update (the next poll stages a fresh one) — the
    // same drop-on-contention behaviour the V1 path had on a full TX FIFO.
    if (xSemaphoreTake(_tx_mux, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return -1;
    }
    memcpy(_tx_buf, data, len);
    _tx_len = len;
    xSemaphoreGive(_tx_mux);

    return static_cast<int>(len);
}

// ---------------------------------------------------------------------------
//  Master: raw read
// ---------------------------------------------------------------------------
esp_err_t TR_I2C_Interface::masterRead(uint8_t* out_buf,
                                       size_t len,
                                       uint32_t timeout_ms) const
{
    if (out_buf == nullptr || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_receive(_master_dev,
                              out_buf,
                              len,
                              pdMS_TO_TICKS(timeout_ms));
}

// ---------------------------------------------------------------------------
//  Master: getOutReady
// ---------------------------------------------------------------------------
bool TR_I2C_Interface::getOutReady(const OutStatusQueryData& query_data,
                                   uint8_t* out_command,
                                   uint32_t timeout_ms) const
{
    (void)query_data;   // query is now sent by the caller
    (void)timeout_ms;   // only the short read timeout is used

    uint8_t rx_frame[10] = {};
    bool got_response = false;
    static constexpr uint32_t READ_TIMEOUT_MS = 2;

    esp_err_t read_err = masterRead(rx_frame, sizeof(rx_frame), READ_TIMEOUT_MS);
    if (read_err == ESP_OK)
    {
        uint8_t rx_type = 0;
        uint8_t payload[2] = {0, 0};
        size_t payload_len = 0;
        if (unpackMessage(rx_frame,
                          sizeof(rx_frame),
                          rx_type,
                          payload,
                          sizeof(payload),
                          payload_len,
                          true))
        {
            if ((rx_type == OUT_STATUS_RESPONSE) && (payload_len >= 1))
            {
                if (out_command != nullptr)
                {
                    *out_command = (payload_len >= 2) ? payload[1] : 0U;
                }
                got_response = (payload[0] != 0U);
            }
        }
    }

    return got_response;
}

// ---------------------------------------------------------------------------
//  Frame pack/unpack (static, unchanged)
// ---------------------------------------------------------------------------
bool TR_I2C_Interface::packMessage(uint8_t type,
                                   const uint8_t *payload,
                                   size_t len,
                                   uint8_t *frame_out,
                                   size_t frame_out_capacity,
                                   size_t &frame_len_out)
{
    frame_len_out = 0;

    if (frame_out == nullptr)
    {
        return false;
    }
    if ((len > 0) && (payload == nullptr))
    {
        return false;
    }
    if (len > MAX_PAYLOAD)
    {
        return false;
    }
    if (len > 0xFF)
    {
        return false;
    }

    const size_t frame_len = 4 + 1 + 1 + len + 2;
    if (frame_out_capacity < frame_len)
    {
        return false;
    }

    size_t idx = 0;
    frame_out[idx++] = SOF0;
    frame_out[idx++] = SOF1;
    frame_out[idx++] = SOF2;
    frame_out[idx++] = SOF3;

    frame_out[idx++] = type;
    frame_out[idx++] = static_cast<uint8_t>(len);

    if (len > 0)
    {
        memcpy(frame_out + idx,
               payload,
               len);
        idx += len;
    }

    const uint16_t crc = calcCRC16(frame_out + 4,
                                   static_cast<int>(1 + 1 + len));

    frame_out[idx++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    frame_out[idx++] = static_cast<uint8_t>(crc & 0xFF);

    frame_len_out = idx;
    return true;
}

bool TR_I2C_Interface::unpackMessage(const uint8_t *frame,
                                     size_t frame_len,
                                     uint8_t &type_out,
                                     uint8_t *payload_out,
                                     size_t payload_out_capacity,
                                     size_t &payload_len_out,
                                     bool verify_crc)
{
    payload_len_out = 0;

    if (frame == nullptr)
    {
        return false;
    }
    if (frame_len < (4 + 1 + 1 + 2))
    {
        return false;
    }

    if ((frame[0] != SOF0) ||
        (frame[1] != SOF1) ||
        (frame[2] != SOF2) ||
        (frame[3] != SOF3))
    {
        return false;
    }

    const uint8_t type = frame[4];
    const size_t payload_len = frame[5];
    const size_t expected_len = 4 + 1 + 1 + payload_len + 2;

    if (frame_len != expected_len)
    {
        return false;
    }
    if (payload_len > MAX_PAYLOAD)
    {
        return false;
    }
    if ((payload_len > 0) && (payload_out == nullptr))
    {
        return false;
    }
    if (payload_len > payload_out_capacity)
    {
        return false;
    }

    if (verify_crc)
    {
        const uint16_t crc_expected = static_cast<uint16_t>((frame[expected_len - 2] << 8) |
                                                             frame[expected_len - 1]);
        const uint16_t crc_actual = calcCRC16(frame + 4,
                                              static_cast<int>(1 + 1 + payload_len));
        if (crc_actual != crc_expected)
        {
            return false;
        }
    }

    if (payload_len > 0)
    {
        memcpy(payload_out,
               frame + 6,
               payload_len);
    }

    type_out = type;
    payload_len_out = payload_len;
    return true;
}
