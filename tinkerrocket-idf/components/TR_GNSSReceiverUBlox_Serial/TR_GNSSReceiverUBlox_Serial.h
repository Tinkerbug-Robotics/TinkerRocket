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

        /// Non-blocking poll: parse any pending serial bytes and, if a new
        /// UBX-NAV-SAT report has arrived since the last call, pack it into
        /// `out` (tracked satellites first — gnssSatSelect) and return true.
        /// begin() enables NAV-SAT at the navigation rate, so one arrives per
        /// epoch alongside NAV-PVT; pair the two records by itow_ms.  Returns
        /// false forever if the receiver refused the enable at begin().
        bool pollNewSat(GNSSSatData &out);

#if defined(TR_GNSS_COCOM_DIAG) && TR_GNSS_COCOM_DIAG
        /// Emit one line of fix state and one of per-satellite C/N0 over the
        /// console, in the form cocom_fcdiag.py converts into the rig's UBX
        /// capture format.  Reads the NAV-PVT and NAV-SAT caches; the flight
        /// path (pollNewSat) owns the freshness flag.  Call about 1 Hz.
        void logSatDiag();
#endif

    private:

        SFE_UBLOX_GNSS_SERIAL gnss;

        uart_port_t _uartPort;

        uint8_t update_rate_hz;

        // NAV-SAT accepted by the receiver at begin().  False leaves
        // pollNewSat() a no-op rather than failing GNSS bring-up: the fix is
        // flight-critical, the per-satellite record is not.
        bool sat_reports_enabled_ = false;

        // Staging for pollNewSat(): every block the receiver reported this
        // epoch, converted, before gnssSatSelect() picks the ones that fit.
        // A member, not a stack array — 255 * 6 B on the poll task's stack
        // is not worth the risk.
        GNSSSatBlock sat_scratch_[UBX_NAV_SAT_MAX_BLOCKS];

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
