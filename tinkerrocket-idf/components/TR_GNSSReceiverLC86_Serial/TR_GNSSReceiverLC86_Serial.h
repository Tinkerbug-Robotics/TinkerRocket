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
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>

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

        /// Per-satellite report for the latest satellites-in-view burst
        /// (#1032).  Harvests only — the bytes were already drained and
        /// parsed by pollNewPVT(), which the collector calls first on the
        /// same pass; this never touches the UART.
        ///
        /// This part speaks NMEA plus Quectel's PAIR/PQTM, not UBX, so there
        /// is no NAV-SAT and GSV is the only C/N0 source.  begin() enables it
        /// at roughly 1 Hz (a divisor of the fix rate; exactly 1 Hz as
        /// measured on the Beetle), not at the fix rate: the poll task's
        /// latency budget is why, and a coexistence or interference question
        /// is answered by C/N0 trends, not by 18 Hz of them.  So records
        /// arrive ~1/s while GNSSData arrives at the fix rate — expect far
        /// fewer GNSS_SAT_MSG records in a mini log than in a V9 one, and see
        /// Lc86Parser::takeSat for the fields GSV cannot fill (flags is
        /// always 0).  The first calls after begin() also log, once, how many
        /// GSV bursts arrived (checkGsvRate).
        bool pollNewSat(GNSSSatData &out);

        /// Raw RTCM3 frames (GNSS_RTCM_MSG): the oldest frame not yet taken,
        /// verbatim from the 0xD3 through its CRC-24Q, and the MCU time its
        /// last byte was parsed. False when none is waiting. Every frame the
        /// parser accepts goes into a ring as it arrives — an MSM7 epoch is
        /// ~5 frames back to back, which a one-slot handoff would lose all but
        /// one of — and this drains it. Safe from another task than the one
        /// calling pollNewPVT(). `cap` must be GNSS_RTCM_MAX_FRAME; a frame
        /// that cannot fit is dropped and counted.
        bool pollRtcmFrame(uint8_t* out, size_t cap, size_t& len, uint32_t& time_us);

        /// Frames lost because the ring was full (nobody draining it) or
        /// `out` was too small. Monotonic.
        uint32_t rtcmFramesDropped() const { return rtcm_dropped_; }

    private:

        // What awaitEvent() is pumping the RX stream for (begin()-time only;
        // the poll path never blocks).
        enum class AwaitKind : uint8_t
        {
            ANY_VALID,   // any checksummed sentence (liveness)
            PAIR_ACK,    // $PAIR001 with a matching CommandID
            QTM_RESULT,  // $PQTMCFGMSGRATE,OK / ,ERROR
            PVT,         // a parsed $PQTMPVT epoch
            NAV_MODE,    // a $PAIR081,<NavMode> answer
            RTCM_MODE,   // a $PAIR433,<Mode> answer
            RTCM_FRAME,  // any RTCM3 frame whose CRC checks
            RTCM_EPH_MODE, // a $PAIR437,<Enable> answer
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

        // Put the frame behind the parser's last RTCM / MSM_EPOCH event into
        // the ring (see pollRtcmFrame).
        void stashRtcmFrame();

        // One boot-log line on how many GSV bursts arrived in the first few
        // seconds of fixes after begin() (see the .cpp).
        void checkGsvRate();

        // The same for the RTCM3 MSM7 epochs, and one more line at the
        // first epoch that has satellites measured (see the .cpp).
        void checkMsmRate();
        void reportFirstMsmEpoch();

        lc86::Lc86Parser parser_;

        uart_port_t _uartPort;

        uint8_t update_rate_hz = 0;

        // Last published epoch — what getGNSSData() returns.
        GNSSData data_ = {};

        // begin() got $PQTMCFGMSGRATE,OK for PQTMPVT. When false, nothing may
        // present as a usable fix (see the vel_d rule in the .cpp).
        bool pvt_stream_ok_ = false;

        // begin() got $PAIR001 Result 0 for the GSV enable. False means the
        // module declined and pollNewSat() stays silent for the session
        // rather than reporting a table nothing refreshes.
        bool sat_stream_ok_ = false;

        uint32_t pvt_epochs_ = 0;

        // Raw-frame ring (pollRtcmFrame). 4 kB holds ~6 s of open-sky MSM7
        // (~600 B per epoch with item headers): the main loop drains it every
        // pass, so it only has to ride out the boot, before the loop runs.
        // Static storage, created in begin() — never on a task stack.
        static constexpr size_t kRtcmRingBytes = 4096;
        uint8_t  rtcm_ring_mem_[kRtcmRingBytes] = {};
        StaticRingbuffer_t rtcm_ring_struct_ = {};
        RingbufHandle_t rtcm_ring_ = nullptr;
        // One item being built: time_us, then the frame.
        uint8_t  rtcm_item_[4 + rtcm3::kMaxPayload + rtcm3::kFrameOverhead] = {};
        uint32_t rtcm_dropped_ = 0;

        // checkGsvRate(): the PVT epoch and GSV burst counts when begin()
        // finished, and whether the line is done (true until a begin() that
        // enabled GSV arms it).
        uint32_t gsv_check_epochs0_     = 0;
        uint32_t gsv_check_bursts0_     = 0;
        uint32_t gsv_check_sat_bursts0_ = 0;
        bool     gsv_checked_           = true;

        // begin() got $PAIR001 Result 0 for $PAIR432,1 (RTCM3 MSM7 on).
        bool msm_stream_ok_ = false;

        // begin() got $PAIR001 Result 0 for $PAIR436,1 (RTCM3 ephemeris on).
        bool eph_stream_ok_ = false;


        // checkMsmRate() / reportFirstMsmEpoch(): the counts when begin()
        // finished, and whether each line is done.
        uint32_t msm_check_epochs0_  = 0;
        uint32_t msm_check_msm0_     = 0;
        uint32_t msm_check_msgs0_    = 0;
        uint32_t msm_check_crc0_     = 0;
        uint32_t msm_check_pre0_     = 0;
        uint32_t msm_check_frames0_  = 0;
        uint32_t msm_check_hdr0_     = 0;
        uint32_t msm_check_eph0_     = 0;
        bool     msm_rate_checked_   = true;
        bool     msm_first_reported_ = true;
};

#endif
