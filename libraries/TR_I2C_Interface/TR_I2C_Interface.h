#ifndef TR_I2C_INTERFACE_H
#define TR_I2C_INTERFACE_H

#include <Arduino.h>
#include <driver/i2c.h>
#include <RocketComputerTypes.h>

class TR_I2C_Interface
{
public:
    TR_I2C_Interface(i2c_port_t port, uint8_t device_address_7bit);

    esp_err_t beginMaster(int sda_pin,
                          int scl_pin,
                          uint32_t clock_hz,
                          bool enable_internal_pullups = false,
                          bool delete_existing_driver = true);
    esp_err_t beginSlave(int sda_pin,
                         int scl_pin,
                         uint32_t clock_hz,
                         size_t rx_buffer_len = 2048,
                         size_t tx_buffer_len = 256,
                         bool enable_internal_pullups = false,
                         bool delete_existing_driver = true);

    esp_err_t sendMessage(uint8_t type,
                          const uint8_t *payload,
                          size_t len,
                          uint32_t timeout_ms = 1000) const;
    
    esp_err_t sendStartLoggingMsg(uint32_t timeout_ms = 1000) const;
    esp_err_t sendEndOfFlightMsg(uint32_t timeout_ms = 1000) const;
    bool getOutReady(uint8_t* out_command = nullptr,
                     uint32_t timeout_ms = 100) const;
    bool getOutReady(const OutStatusQueryData& query_data,
                     uint8_t* out_command = nullptr,
                     uint32_t timeout_ms = 100) const;
    esp_err_t masterRead(uint8_t* out_buf,
                         size_t len,
                         uint32_t timeout_ms = 10) const;
    int readFromSlave(uint8_t* out_buf,
                      size_t out_buf_capacity,
                      uint32_t timeout_ms = 10) const;
    int writeToSlave(const uint8_t* data,
                     size_t len,
                     uint32_t timeout_ms = 10) const;

    static bool packMessage(uint8_t type,
                            const uint8_t *payload,
                            size_t len,
                            uint8_t *frame_out,
                            size_t frame_out_capacity,
                            size_t &frame_len_out);

    static bool unpackMessage(const uint8_t *frame,
                              size_t frame_len,
                              uint8_t &type_out,
                              uint8_t *payload_out,
                              size_t payload_out_capacity,
                              size_t &payload_len_out,
                              bool verify_crc = true);

private:
    static constexpr uint8_t SOF0 = 0xAA;
    static constexpr uint8_t SOF1 = 0x55;
    static constexpr uint8_t SOF2 = 0xAA;
    static constexpr uint8_t SOF3 = 0x55;

    i2c_port_t port;
    uint8_t device_address;
};

#endif
