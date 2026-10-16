#pragma once

// Host stand-in for ESP-IDF's driver/i2c_master.h: the types and the three
// calls the I2C drivers make, with the IDF v6 signatures.  Declarations
// only.  A host test that compiles a driver against it defines the calls
// itself, backed by a fake of the part (test_qmc5883p_driver.cpp).

#include <stddef.h>
#include <stdint.h>

// esp_err_t / ESP_OK, guarded the same way driver/ledc.h's shim does.
#ifndef ESP_OK
typedef int esp_err_t;
#define ESP_OK 0
static inline const char* esp_err_to_name(esp_err_t) { return "ESP_OK"; }
#endif
#ifndef ESP_FAIL
#define ESP_FAIL -1
#endif

typedef struct i2c_master_bus_t* i2c_master_bus_handle_t;
typedef struct i2c_master_dev_t* i2c_master_dev_handle_t;

typedef enum {
    I2C_ADDR_BIT_LEN_7  = 0,
    I2C_ADDR_BIT_LEN_10 = 1,
} i2c_addr_bit_len_t;

typedef struct {
    i2c_addr_bit_len_t dev_addr_length;
    uint16_t device_address;
    uint32_t scl_speed_hz;
    uint32_t scl_wait_us;
    struct {
        uint32_t disable_ack_check : 1;
    } flags;
} i2c_device_config_t;

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus_handle,
                                    const i2c_device_config_t* dev_config,
                                    i2c_master_dev_handle_t* ret_handle);
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t i2c_dev,
                              const uint8_t* write_buffer, size_t write_size,
                              int xfer_timeout_ms);
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t i2c_dev,
                                      const uint8_t* write_buffer, size_t write_size,
                                      uint8_t* read_buffer, size_t read_size,
                                      int xfer_timeout_ms);
