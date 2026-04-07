#include <TR_I2C_Interface.h>
#include <CRC.h>
#include <cstring>
#include <esp_log.h>

static const char *TAG = "I2C_IF";

TR_I2C_Interface::TR_I2C_Interface(uint8_t device_address_7bit)
    : device_address(device_address_7bit),
      _master_bus(nullptr),
      _master_dev(nullptr),
      _slave_dev(nullptr),
      _rx_queue(nullptr)
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
    (void)clock_hz;       // slave clock is driven by master
    (void)rx_buffer_len;  // new API uses per-transaction buffers

    // Queue for ISR → task notification of completed receives
    _rx_queue = xQueueCreate(4, sizeof(uint8_t *));
    if (_rx_queue == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create slave RX queue");
        return ESP_ERR_NO_MEM;
    }

    i2c_slave_config_t slave_cfg = {};
    slave_cfg.i2c_port = I2C_NUM_0;
    slave_cfg.sda_io_num = static_cast<gpio_num_t>(sda_pin);
    slave_cfg.scl_io_num = static_cast<gpio_num_t>(scl_pin);
    slave_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    slave_cfg.send_buf_depth = tx_buffer_len;
    slave_cfg.slave_addr = device_address;
    slave_cfg.addr_bit_len = I2C_ADDR_BIT_LEN_7;
    slave_cfg.intr_priority = 0;

    esp_err_t err = i2c_new_slave_device(&slave_cfg, &_slave_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_new_slave_device failed: %s", esp_err_to_name(err));
        return err;
    }

    // Register receive-done callback
    i2c_slave_event_callbacks_t cbs = {};
    cbs.on_recv_done = slaveRxDoneISR;
    err = i2c_slave_register_event_callbacks(_slave_dev, &cbs, this);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_slave_register_event_callbacks failed: %s", esp_err_to_name(err));
        return err;
    }

    // Zero the buffer and arm the first receive
    memset(_slave_rx_buf, 0, SLAVE_RX_BUF_SIZE);
    err = i2c_slave_receive(_slave_dev, _slave_rx_buf, SLAVE_RX_BUF_SIZE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_slave_receive (initial arm) failed: %s", esp_err_to_name(err));
    }
    return err;
}

// ---------------------------------------------------------------------------
//  Slave RX done ISR callback
//  The new API fires this when the master completes a write transaction
//  (STOP condition). edata->buffer points to our _slave_rx_buf.
//  We post the buffer pointer to the queue so readFromSlave() can pick it up.
// ---------------------------------------------------------------------------
bool IRAM_ATTR TR_I2C_Interface::slaveRxDoneISR(
    i2c_slave_dev_handle_t channel,
    const i2c_slave_rx_done_event_data_t *edata,
    void *user_data)
{
    (void)channel;
    auto *self = static_cast<TR_I2C_Interface *>(user_data);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Post the buffer pointer (for identification, not ownership transfer)
    uint8_t *buf = edata->buffer;
    xQueueSendFromISR(self->_rx_queue, &buf, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken == pdTRUE;
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

    uint8_t *buf_ptr = nullptr;
    if (xQueueReceive(_rx_queue, &buf_ptr, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return 0; // no data available
    }

    // Determine valid data length from frame protocol:
    // SOF(4) + type(1) + len(1) + payload(len) + CRC(2)
    size_t valid_len = 0;
    if (buf_ptr[0] == SOF0 && buf_ptr[1] == SOF1 &&
        buf_ptr[2] == SOF2 && buf_ptr[3] == SOF3)
    {
        size_t payload_len = buf_ptr[5];
        valid_len = 4 + 1 + 1 + payload_len + 2; // SOF + type + len + payload + CRC
        if (valid_len > SLAVE_RX_BUF_SIZE)
        {
            valid_len = SLAVE_RX_BUF_SIZE; // safety clamp
        }
    }
    else
    {
        // Non-framed data or corrupted — return whatever we can
        // Scan backwards for last non-zero byte
        valid_len = SLAVE_RX_BUF_SIZE;
        while (valid_len > 0 && buf_ptr[valid_len - 1] == 0)
        {
            valid_len--;
        }
    }

    if (valid_len == 0)
    {
        // Re-arm and return no data
        memset(_slave_rx_buf, 0, SLAVE_RX_BUF_SIZE);
        i2c_slave_receive(_slave_dev, _slave_rx_buf, SLAVE_RX_BUF_SIZE);
        return 0;
    }

    size_t copy_len = (valid_len < out_buf_capacity) ? valid_len : out_buf_capacity;
    memcpy(out_buf, buf_ptr, copy_len);

    // Zero the buffer and re-arm for next receive
    memset(_slave_rx_buf, 0, SLAVE_RX_BUF_SIZE);
    i2c_slave_receive(_slave_dev, _slave_rx_buf, SLAVE_RX_BUF_SIZE);

    return static_cast<int>(copy_len);
}

// ---------------------------------------------------------------------------
//  Slave: write data for master to read
// ---------------------------------------------------------------------------
int TR_I2C_Interface::writeToSlave(const uint8_t* data,
                                   size_t len,
                                   uint32_t timeout_ms)
{
    if (data == nullptr || len == 0 || _slave_dev == nullptr)
    {
        return -1;
    }
    esp_err_t err = i2c_slave_transmit(_slave_dev,
                                       data,
                                       static_cast<int>(len),
                                       static_cast<int>(timeout_ms));
    return (err == ESP_OK) ? static_cast<int>(len) : -1;
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
