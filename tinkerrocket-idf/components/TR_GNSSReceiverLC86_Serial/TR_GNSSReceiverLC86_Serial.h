// TR_GNSSReceiverLC86_Serial.h — Quectel LC86G (LA) driver, ESP-IDF native UART
//
// Drop-in for TR_GNSSReceiverUBloxSerial: identical public surface, identical
// GNSSData output, selected in TR_Sensor_Collector by TR_GNSS_DRIVER_LC86.
// Speaks NMEA + Quectel PAIR/PQTM instead of UBX; the load-bearing sentence is
// $PQTMPVT — the only one this module emits with vertical velocity (VelD).
// All sentence parsing lives in Lc86Parser (pure, host-tested); this class
// owns only the UART plumbing and the begin()-time command/ack choreography.
#ifndef TRGNSSRECEIVERLC86_H
#define TRGNSSRECEIVERLC86_H

#include <cstdint>
#include <driver/uart.h>

#include <RocketComputerTypes.h>
#include "Lc86Parser.h"

class TR_GNSSReceiverLC86Serial
{
    public:

        // Constructor — takes the ESP-IDF UART port to use (e.g. UART_NUM_1)
        TR_GNSSReceiverLC86Serial(uart_port_t uart_port = UART_NUM_1);

        // Initialize GNSS over UART. reset_n_pin/safeboot_n_pin exist only
        // for signature compatibility with the u-blox driver (the collector
        // passes both unconditionally) and are IGNORED: the LC86G has no
        // SAFEBOOT concept and the mini wires no RESET_N — the module
        // self-starts when its rail rises. Returns false (GNSS-absent
        // degraded mode, #557) on a dead/deaf module or when the load-bearing
        // $PQTMPVT stream cannot be enabled.
        /** No OTP high-perf-clock concept on this part — it is not an M10.
         *  Present so callers need not branch on the driver (#837 item 6). */
        uint8_t otpState() const { return gnss_otp::NOT_M10; }

        bool begin(uint8_t update_rate_hz,
                   uint8_t GNSS_RX,
                   uint8_t GNSS_TX,
                   int8_t reset_n_pin = -1,
                   int8_t safeboot_n_pin = -1);

        // Copy the already-parsed cache. Never touches the UART (#572
        // contract carried over from the u-blox driver) — only meaningful
        // immediately after a successful pollNewPVT().
        void getGNSSData(GNSSData &data);

        /// Non-blocking poll: parse any pending serial bytes and, if a
        /// new $PQTMPVT epoch has arrived, fill `data` and return true.
        /// Call this at >=2x the navigation rate for reliable capture.
        /// Bounded work per call (the collector counts >1 ms polls).
        bool pollNewPVT(GNSSData &data);

    private:

        // What awaitEvent() is pumping the RX stream for (begin()-time only;
        // the poll path never blocks).
        enum class AwaitKind : uint8_t
        {
            ANY_VALID,   // any checksummed sentence (liveness)
            PAIR_ACK,    // $PAIR001 with a matching CommandID
            QTM_RESULT,  // $PQTMCFGMSGRATE,OK / ,ERROR
            PVT,         // a parsed $PQTMPVT epoch
        };

        // Helper: install/reconfigure the UART driver at a given baud rate
        // and pins. Tears down any existing driver first.
        void uartBegin(uint32_t baud, uint8_t rx_pin, uint8_t tx_pin);

        // Helper: tear down the UART driver.
        void uartEnd();

        // Frame `body` with '$'/checksum/CRLF and write it, waiting for the
        // wire so a follow-on uartBegin() can never truncate it mid-frame.
        void sendSentence(const char* body);

        // Pump RX bytes through the parser until the wanted event or timeout.
        // PVT epochs seen along the way always refresh the cache.
        bool awaitEvent(AwaitKind kind, uint32_t timeout_ms,
                        uint16_t pair_id = 0,
                        uint8_t* pair_result = nullptr,
                        bool* qtm_ok = nullptr);

        // Send + $PAIR001 ack wait with retries; true only on Result 0.
        bool sendPairCommand(const char* body, uint16_t ack_id);

        // Send + $PQTMCFGMSGRATE,OK/ERROR wait with retries; true on OK.
        bool sendQtmMsgRate(const char* body);

        // Copy the parser's PVT into the cache, stamp MCU time, apply the
        // no-PVT-stream insurance latch.
        void publishFromParser();

        lc86::Lc86Parser parser_;

        uart_port_t _uartPort;

        uint8_t update_rate_hz = 0;

        // Last published epoch — what getGNSSData() returns.
        GNSSData data_ = {};

        // begin() got $PQTMCFGMSGRATE,OK for PQTMPVT. When false, nothing may
        // present as a usable fix (see the vel_d rule in the .cpp).
        bool pvt_stream_ok_ = false;

        uint32_t pvt_epochs_ = 0;
};

#endif
