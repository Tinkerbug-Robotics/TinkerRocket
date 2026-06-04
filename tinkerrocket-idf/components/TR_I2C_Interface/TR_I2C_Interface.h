#ifndef TR_I2C_INTERFACE_H
#define TR_I2C_INTERFACE_H

#include <compat.h>
#include <driver/i2c_master.h>
#include <driver/i2c_slave.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <RocketComputerTypes.h>

class TR_I2C_Interface
{
public:
    TR_I2C_Interface(uint8_t device_address_7bit);

    esp_err_t beginMaster(int sda_pin,
                          int scl_pin,
                          uint32_t clock_hz,
                          bool enable_internal_pullups = false);
    esp_err_t beginSlave(int sda_pin,
                         int scl_pin,
                         uint32_t clock_hz,
                         size_t rx_buffer_len = 2048,
                         size_t tx_buffer_len = 256,
                         bool enable_internal_pullups = false);

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
                      uint32_t timeout_ms = 10);
    int writeToSlave(const uint8_t* data,
                     size_t len,
                     uint32_t timeout_ms = 10);

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

    // --- Slave callbacks (ESP-IDF V2 slave driver; run in ISR context) ----
    // on_receive: master finished writing to us. The driver-owned buffer is
    // reused on the next receive, so we memcpy it into _slave_rx_buf and post
    // the byte count to _rx_queue for readFromSlave() to pick up.
    static bool IRAM_ATTR slaveReceiveISR(i2c_slave_dev_handle_t channel,
                                          const i2c_slave_rx_done_event_data_t *edata,
                                          void *user_data);
    // on_request: master is addressing us for a read. The V2 driver stretches
    // SCL on address-match and only releases it when i2c_slave_write() runs —
    // but that call takes a mutex and is illegal in ISR context. So we just
    // wake slaveTxTask(), which performs the write (feeding the master and
    // clearing the stretch). This is the fix for the #88 V2 bus-wedge: the
    // earlier attempt left on_request unregistered, so reads never got served
    // and the bus hung until the hardware stretch-protect timer.
    static bool IRAM_ATTR slaveRequestISR(i2c_slave_dev_handle_t channel,
                                          const i2c_slave_request_event_data_t *edata,
                                          void *user_data);
    // TX service task: blocks on _tx_req_queue, writes the staged response.
    static void slaveTxTask(void *arg);

    uint8_t device_address;

    // Master mode handles
    i2c_master_bus_handle_t _master_bus;
    i2c_master_dev_handle_t _master_dev;

    // Slave mode handles
    i2c_slave_dev_handle_t _slave_dev;

    // Slave RX: on_receive ISR copies the received frame into _slave_rx_buf
    // and posts its byte count to _rx_queue; readFromSlave() dequeues + parses
    // the framed message (SOF + len) from it.
    QueueHandle_t _rx_queue;
    static constexpr size_t SLAVE_RX_BUF_SIZE = 256;
    uint8_t _slave_rx_buf[SLAVE_RX_BUF_SIZE];

    // Slave TX: writeToSlave() stages the latest response into _tx_buf under
    // _tx_mux. on_request posts a token to _tx_req_queue; slaveTxTask() wakes,
    // snapshots the staged bytes, and calls i2c_slave_write().
    QueueHandle_t _tx_req_queue;
    SemaphoreHandle_t _tx_mux;
    TaskHandle_t _tx_task;
    static constexpr size_t SLAVE_TX_BUF_SIZE = 256;
    uint8_t _tx_buf[SLAVE_TX_BUF_SIZE];
    volatile size_t _tx_len;
};

#endif
