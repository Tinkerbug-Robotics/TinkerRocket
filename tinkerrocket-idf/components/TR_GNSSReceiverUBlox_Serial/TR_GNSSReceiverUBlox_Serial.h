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
        /** GNSS high-perf-clock OTP state at boot; a gnss_otp::* constant. */
        uint8_t otpState() const { return otp_state_; }

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

#if defined(TR_GNSS_COCOM_DIAG) && TR_GNSS_COCOM_DIAG
        /// Ask the receiver for per-satellite reports (UBX-NAV-SAT). Call once
        /// after begin(). Off in normal builds.
        ///
        /// The flight path never needs this: it polls NAV-PVT and takes the
        /// satellite count from getSIV(). But #491 turns on telling a withheld
        /// position (satellites still tracked at healthy C/N0) from a lost
        /// signal (satellites gone), and that distinction is only visible
        /// per-satellite. Without NAV-SAT the two are indistinguishable and the
        /// measurement cannot be made at all.
        bool enableSatDiag();

        /// Emit one line of fix state and one of per-satellite C/N0, in a form
        /// cocom_fcdiag.py converts into the same UBX capture format the
        /// conducted rig already analyses. Non-blocking; call about 1 Hz.
        void logSatDiag();
#endif

    private:

        SFE_UBLOX_GNSS_SERIAL gnss;

        uart_port_t _uartPort;

        uint8_t update_rate_hz;

        // SAM-M10Q "high performance navigation update rate" (integration
        // manual UBX-22020019 §2.1.5): verify the high-CPU-clock OTP
        // configuration is present; program it (permanent, one-time) if not.
        // Returns false ONLY when the OTP was just programmed and the
        // receiver needs a hardware reset + reconnect for it to apply.
        bool ensureHighPerformanceClock();
        bool otp_program_attempted_ = false;  // one write attempt per boot
        bool otp_reset_done_ = false;         // one post-write reset/reconnect per boot
        // #837 item 6: RESET_N is not wired on ANY board revision to date
        // (v7/v8/v9 all declare GNSS_RESET_N = -1) — the module's ~RESET goes
        // only to its own pull-up and is never brought out to the host
        // connector. Warn once per boot rather than on every retry.
        bool reset_pin_absent_logged_ = false;

        // What ensureHighPerformanceClock() concluded, as a gnss_otp::*
        // constant.  Previously determined at every boot and logged only to
        // serial — so no flight record could say whether the receiver ran the
        // high clock (#837 item 6).  Now carried into FlightSettingsData v7.
        uint8_t otp_state_ = gnss_otp::UNKNOWN;

        // Helper: install/reconfigure the UART driver at a given baud rate and pins.
        // Tears down any existing driver first.
        void uartBegin(uint32_t baud, uint8_t rx_pin, uint8_t tx_pin);

        // Helper: tear down the UART driver.
        void uartEnd();

        // Helper: return how many bytes are buffered in the UART RX FIFO.
        size_t uartAvailable();

        // Helper: blocking read of one byte. Returns byte or -1 on timeout.
        int uartRead();

        // (#572: the old getGNSSData() staleness counter was removed — it was
        // dead code, only reachable after getPVT(0) already proved a fresh
        // frame. Liveness is owned by the downstream collector/EKF time_us
        // freshness gates.)

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
