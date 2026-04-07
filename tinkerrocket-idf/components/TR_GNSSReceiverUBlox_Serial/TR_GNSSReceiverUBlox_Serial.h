// TR_GNSSReceiverUBlox.h (Serial version — ESP-IDF native UART)
#ifndef TRGNSSRECEIVERUBLOX_H
#define TRGNSSRECEIVERUBLOX_H

#include <compat.h>
#include <driver/uart.h>
#include "TR_SparkFun_u-blox_GNSS_v3.h"
#include <sys/time.h>
#include <string>

#include <RocketComputerTypes.h>

class TR_GNSSReceiverUBloxSerial
{
    public:

        // Constructor — takes the ESP-IDF UART port to use (e.g. UART_NUM_1)
        TR_GNSSReceiverUBloxSerial(uart_port_t uart_port = UART_NUM_1);

        // Initialize GNSS over UART
        bool begin(uint8_t update_rate_hz,
                   uint8_t GNSS_RX,
                   uint8_t GNSS_TX,
                   int8_t reset_n_pin = -1,
                   int8_t safeboot_n_pin = -1);

        // Update a copy of GNSSData (legacy timer-based path)
        void getGNSSData(GNSSData &data);

        /// Non-blocking poll: parse any pending serial bytes and, if a
        /// new NAV-PVT message has arrived, fill `data` and return true.
        /// Call this at >=2x the navigation rate for reliable capture.
        bool pollNewPVT(GNSSData &data);

    private:

        SFE_UBLOX_GNSS_SERIAL gnss;

        uart_port_t _uartPort;

        uint8_t update_rate_hz;

        // Helper: install/reconfigure the UART driver at a given baud rate and pins.
        // Tears down any existing driver first.
        void uartBegin(uint32_t baud, uint8_t rx_pin, uint8_t tx_pin);

        // Helper: tear down the UART driver.
        void uartEnd();

        // Helper: return how many bytes are buffered in the UART RX FIFO.
        size_t uartAvailable();

        // Helper: blocking read of one byte. Returns byte or -1 on timeout.
        int uartRead();

        // Staleness detection: if GNSS second+millisecond are unchanged for
        // STALE_THRESHOLD consecutive getGNSSData() calls, the serial link
        // has likely lost sync and the library is returning cached data.
        static constexpr uint16_t STALE_THRESHOLD = 5;  // ~500 ms at 10 Hz
        uint8_t  prev_second      = 0xFF;   // impossible initial value
        uint16_t prev_milli_second = 0xFFFF;
        uint16_t stale_count       = 0;

        void enuToEcefVelocityAndPosition(float v_east,
                                          float v_north,
                                          float v_up,
                                          double lat_rad,
                                          double lon_rad,
                                          double height,
                                          int16_t& ecef_vX,
                                          int16_t& ecef_vY,
                                          int16_t& ecef_vZ,
                                          int32_t& ecef_X,
                                          int32_t& ecef_Y,
                                          int32_t& ecef_Z);

};

#endif
