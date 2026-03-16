#include <TR_I2C_Interface.h>
#include <CRC.h>
#include <cstring>

TR_I2C_Interface::TR_I2C_Interface(i2c_port_t port, uint8_t device_address_7bit)
    : port(port),
      device_address(device_address_7bit)
{
}

esp_err_t TR_I2C_Interface::beginMaster(int sda_pin,
                                        int scl_pin,
                                        uint32_t clock_hz,
                                        bool enable_internal_pullups,
                                        bool delete_existing_driver)
{
    (void)delete_existing_driver;

    i2c_config_t config = {};
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = static_cast<gpio_num_t>(sda_pin);
    config.scl_io_num = static_cast<gpio_num_t>(scl_pin);
    config.sda_pullup_en = enable_internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    config.scl_pullup_en = enable_internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    config.master.clk_speed = clock_hz;
    config.clk_flags = 0;

    esp_err_t err = i2c_param_config(port, &config);
    if (err != ESP_OK)
    {
        return err;
    }

    return i2c_driver_install(port,
                              config.mode,
                              0,
                              0,
                              0);
}

esp_err_t TR_I2C_Interface::beginSlave(int sda_pin,
                                       int scl_pin,
                                       uint32_t clock_hz,
                                       size_t rx_buffer_len,
                                       size_t tx_buffer_len,
                                       bool enable_internal_pullups,
                                       bool delete_existing_driver)
{
    if (delete_existing_driver)
    {
        (void)i2c_driver_delete(port);
    }

    i2c_config_t config = {};
    config.mode = I2C_MODE_SLAVE;
    config.sda_io_num = static_cast<gpio_num_t>(sda_pin);
    config.scl_io_num = static_cast<gpio_num_t>(scl_pin);
    config.sda_pullup_en = enable_internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    config.scl_pullup_en = enable_internal_pullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    config.slave.addr_10bit_en = 0;
    config.slave.slave_addr = device_address;
    (void)clock_hz; // not used by the old slave driver, kept for API symmetry

    esp_err_t err = i2c_param_config(port, &config);
    if (err != ESP_OK)
    {
        return err;
    }

    return i2c_driver_install(port,
                              config.mode,
                              static_cast<int>(rx_buffer_len),
                              static_cast<int>(tx_buffer_len),
                              0);
}

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

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
                          static_cast<uint8_t>((device_address << 1) | I2C_MASTER_WRITE),
                          true);
    i2c_master_write(cmd,
                     frame,
                     frame_len,
                     true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(port,
                                         cmd,
                                         pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);
    return err;
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

int TR_I2C_Interface::readFromSlave(uint8_t* out_buf,
                                    size_t out_buf_capacity,
                                    uint32_t timeout_ms) const
{
    if (out_buf == nullptr || out_buf_capacity == 0)
    {
        return -1;
    }
    return i2c_slave_read_buffer(port,
                                 out_buf,
                                 static_cast<size_t>(out_buf_capacity),
                                 pdMS_TO_TICKS(timeout_ms));
}

int TR_I2C_Interface::writeToSlave(const uint8_t* data,
                                   size_t len,
                                   uint32_t timeout_ms) const
{
    if (data == nullptr || len == 0)
    {
        return -1;
    }
    return i2c_slave_write_buffer(port,
                                  const_cast<uint8_t*>(data),
                                  static_cast<size_t>(len),
                                  pdMS_TO_TICKS(timeout_ms));
}

esp_err_t TR_I2C_Interface::masterRead(uint8_t* out_buf,
                                       size_t len,
                                       uint32_t timeout_ms) const
{
    if (out_buf == nullptr || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
                          static_cast<uint8_t>((device_address << 1) | I2C_MASTER_READ),
                          true);
    if (len > 1)
    {
        i2c_master_read(cmd,
                        out_buf,
                        len - 1,
                        I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd,
                         &out_buf[len - 1],
                         I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(port,
                                         cmd,
                                         pdMS_TO_TICKS(timeout_ms));
    i2c_cmd_link_delete(cmd);
    return err;
}

// Read-only: reads the previous OUT_STATUS_RESPONSE from the slave TX
// buffer.  The caller is responsible for sending the next OUT_STATUS_QUERY
// (e.g. via the I2C sender task queue) to avoid blocking the main loop
// with I2C port-mutex contention.
bool TR_I2C_Interface::getOutReady(const OutStatusQueryData& query_data,
                                   uint8_t* out_command,
                                   uint32_t timeout_ms) const
{
    (void)query_data;   // query is now sent by the caller
    (void)timeout_ms;   // only the short read timeout is used

    // Use a short timeout: the response from the previous query has had
    // ~250 ms to land in the slave TX buffer.  If it's not there within
    // 2 ms it was lost — no point blocking the main loop any longer.
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
