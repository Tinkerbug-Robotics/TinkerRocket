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
#include <freertos/queue.h>
#include <freertos/ringbuf.h>

#include <RocketComputerTypes.h>
#include "GnssTimeSync.h"
#include "Lc86Parser.h"
#include "RawMeas.h"

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

        // The MCU clock against GPS time (GnssTimeSync.h), fed by every
        // $PQTMPVT with a fix: what a raw-measurement filter places each
        // epoch by (#1600). Stamps are when the sentence's last byte arrived,
        // backdated from the read by the bytes that followed it.
        const lc86::GnssTimeSync& timeSync() const { return time_sync_; }
        // The last MSM7 epoch: its GPS time of week, and when its final
        // frame arrived (MCU us, backdated the same way). False before one.
        bool lastMsmEpochTime(uint32_t& gps_tow_ms, int64_t& end_us) const;

        // Raw measurements for a raw-GNSS filter (#1600), off until
        // enableRawOutputs() creates their queues (false without the memory).
        // takeRawEpoch: each MSM7 epoch's GPS, Galileo and BeiDou cells
        // (Lc86RawEpoch.h) with its GPS instant on the MCU clock -- mcu_us is
        // 0 until the time sync has 10 s of fixes, and late by its
        // uncalibrated latency_s after that. takeEphemeris: the payload of
        // each GPS, Galileo or BeiDou broadcast ephemeris message (1019,
        // 1046, 1042). Both are safe from another task than the one polling;
        // a full queue drops the newest and counts it.
        bool enableRawOutputs();
        bool takeRawEpoch(gnss_raw::TimedRawEpoch& out);
        bool takeEphemeris(uint8_t* out, size_t cap, size_t& len);
        uint32_t rawEpochsDropped() const { return raw_dropped_; }
        uint32_t ephemerisDropped() const { return eph_dropped_; }
        static constexpr size_t kEphMaxPayload = 72;   // 1042, the largest, is 64

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
        void stashRtcmFrame(int64_t end_us);

        // One boot-log line on how many GSV bursts arrived in the first few
        // seconds of fixes after begin() (see the .cpp).
        void checkGsvRate();

        // The same for the RTCM3 MSM7 epochs, and one more line at the
        // first epoch that has satellites measured (see the .cpp).
        void checkMsmRate();
        void reportFirstMsmEpoch();
        // GnssTimeSync feed and its periodic boot-console line.
        void noteTimeSync(int64_t end_us);
        void logTimeSync();
        // The raw outputs (enableRawOutputs): the epoch behind the parser's
        // last MSM_EPOCH event, the ephemeris behind its last RTCM event.
        void publishRawEpoch(int64_t end_us);
        void publishEphemeris();

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

        // One byte's time on the wire at the configured baud, us (8N1: 10
        // bits). Set by uartBegin().
        double   byte_us_ = 1.0e7 / 115200.0;
        lc86::GnssTimeSync time_sync_;
        int64_t  time_log_us_ = 0;
        uint32_t msm_epoch_tow_ms_ = 0;
        int64_t  msm_epoch_end_us_ = 0;

        // Raw outputs (#1600): created by enableRawOutputs(), never on a
        // task stack (an epoch is ~1.6 kB; the poll task has 4 kB).
        struct EphItem
        {
            uint8_t len;
            uint8_t bytes[kEphMaxPayload];
        };
        QueueHandle_t raw_epoch_q_ = nullptr;
        QueueHandle_t eph_q_ = nullptr;
        gnss_raw::TimedRawEpoch raw_scratch_ = {};
        uint32_t raw_dropped_ = 0;
        uint32_t eph_dropped_ = 0;

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
