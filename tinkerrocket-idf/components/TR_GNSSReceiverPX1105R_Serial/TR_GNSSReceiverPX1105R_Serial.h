// TR_GNSSReceiverPX1105R_Serial.h — SkyTraq PX1105R driver, ESP-IDF native UART
//
// Drop-in for the other GNSS drivers (the same begin / pollNewPVT /
// getGNSSData surface and the same GNSSData), selected in TR_Sensor_Collector
// by TR_GNSS_DRIVER_PX1105R: the V9 flying the PX1105R carrier
// (hardware/gnss-px1105r-18mm-hv, its J3 on the V9's J1) in place of the
// mosaic-G5, with the same raw-GNSS navigation filter. It speaks SkyTraq
// binary (SkyTraqBinary.h).
//
// What it does:
//   - finds and sets up the receiver (SkyTraqLink.h): 921600 baud, RTK base
//     mode with the kinematic function (the PX1105R sends raw measurements
//     only as an RTK base: AN0037), airborne navigation mode, power normal, a
//     5 deg elevation mask, 0xE5 raw measurements, 0xDF navigation state, 0xE7
//     channel status and the GPS / Galileo / BeiDou navigation subframes at
//     20 Hz; all in the receiver's SRAM, so a power cycle undoes it and the
//     next begin() sets it up again;
//   - GNSSData from 0xDF (its ECEF fix; the height is above the ellipsoid:
//     the receiver sends no geoid in binary, geoidSeparationM is false);
//   - every checksum-valid frame, verbatim (A0 A1 through 0D 0A), into a ring
//     the flight loop logs as GNSS_RTCM_MSG (0x95), as the LC86G's RTCM3 and
//     the mosaic's SBF;
//   - the MCU clock against GPS time from when each 0xE5 arrives, without a
//     1PPS (the carrier's J3 pin 5 is the receiver's RXD2, an input, which
//     this driver never touches): the LC86G's GnssTimeSync, logged as
//     GNSS_CLOCK_MSG (0x9D);
//   - with enableRawOutputs() (the PX1105R image, whose navigation filter is
//     the raw-GNSS filter): each 0xE5 on the filter's fused grid as a
//     TimedRawEpoch (Px1105rRawEpoch.h), its Dopplers in pairs with the epoch
//     between when asked (setRawDopplerPairing: MosaicRawEpoch.h's rule, the
//     same as the mosaic's), each ephemeris as decoded from the subframes
//     (SkyTraqNav.h), and the latest GPS ionosphere (subframe 4 page 18).
// All parsing is in SkyTraqBinary / SkyTraqLink / SkyTraqNav /
// Px1105rRawEpoch (pure, host-tested); this class owns only the UART and the
// hand-offs between tasks.
#ifndef TRGNSSRECEIVERPX1105R_H
#define TRGNSSRECEIVERPX1105R_H

#include <atomic>
#include <cstdint>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>

#include <RocketComputerTypes.h>
#include <GnssTimeSync.h>      // TR_GNSSReceiverLC86_Serial: the clock line, shared
#include <MosaicRawEpoch.h>    // TR_GNSSReceiverMosaic_Serial: the Doppler pairs' rule, shared
#include <RawMeas.h>           // TR_GNSSRaw: gnss_raw::TimedRawEpoch
#include "gnss/eph.h"
#include "Px1105rRawEpoch.h"
#include "SkyTraqBinary.h"
#include "SkyTraqLink.h"
#include "SkyTraqNav.h"

class TR_GNSSReceiverPX1105RSerial : private skytraq::FrameSink
{
    public:

        TR_GNSSReceiverPX1105RSerial(uart_port_t uart_port = UART_NUM_1);

        /** No OTP high-performance clock on this part; present so callers
         *  need not branch on the driver (#837 item 6). */
        uint8_t otpState() const { return gnss_otp::NOT_M10; }

        // No 1PPS reaches the FC on this carrier (J3 pin 5 is the receiver's
        // RXD2): accepted for the callers' sake and ignored. The pin is never
        // touched.
        void setPpsPin(int8_t pin) { (void)pin; }

        // How long begin() looks for the receiver: 30 s unless set.
        void setBeginTimeoutMs(uint32_t ms) { begin_timeout_ms_ = ms; }

        // Whether begin() waits for the receiver (the default) or only starts
        // looking and returns true, the poll task then finding and setting it
        // up whenever it answers (a reboot in flight), as the mosaic's.
        void setBeginWait(bool wait) { begin_wait_ = wait; }

        // Find the receiver and set it up. Waits up to the begin timeout;
        // false on a dead or deaf receiver (#557 GNSS-absent mode).
        // reset_n_pin / safeboot_n_pin: signature compatibility only.
        bool begin(uint8_t update_rate_hz,
                   uint8_t GNSS_RX,
                   uint8_t GNSS_TX,
                   int8_t reset_n_pin = -1,
                   int8_t safeboot_n_pin = -1);

        // The last published fix. Never touches the UART (#572 contract).
        void getGNSSData(GNSSData &data);

        /// Non-blocking: parse what the UART holds (bounded) and keep the link
        /// set up; true with `data` filled when a new 0xDF arrived. The GNSS
        /// poll task's only call into the UART.
        bool pollNewPVT(GNSSData &data);

        /// GNSS_SAT_MSG has no source here: the per-satellite C/N0 is in the
        /// logged 0xE5 frames. Always false.
        bool pollNewSat(GNSSSatData &out) { (void)out; return false; }

        /// The oldest frame not yet taken, verbatim from A0 A1 through 0D 0A,
        /// and the MCU time its last byte arrived. Safe from another task than
        /// the poll. `cap` should be GNSS_SBF_MAX_FRAME.
        bool pollRawFrame(uint8_t* out, size_t cap, size_t& len, uint32_t& time_us);
        uint32_t rawFramesDropped() const { return ring_dropped_ + too_long_ + short_buffer_; }

        // The MCU clock against GPS time from the 0xE5 arrivals (GnssTimeSync,
        // latency the receiver profile's). takeClockLine: the line each time
        // it moves (about once a second with a fix), for GNSS_CLOCK_MSG.
        const lc86::GnssTimeSync& timeSync() const { return time_sync_; }
        bool takeClockLine(GnssClockData& out);
        uint32_t clockLinesDropped() const { return clock_dropped_; }

        // The receiver's delay from an epoch to the last byte of its 0xE5, s
        // (GnssTimeSync::Config::latency_s): the receiver profile's
        // epoch_latency_s. Call before begin().
        void setEpochLatencyS(double s);

        // No geoid in the receiver's binary output: always false.
        bool geoidSeparationM(float& sep_m) const { sep_m = 0.0f; return false; }

        // The subframes repeat (each satellite's ephemeris every 30 s), so
        // there is nothing to ask for again: present for the callers.
        void requestNavResend() {}

        // The receiver's own fixes (0xDF with a 2-D or 3-D fix), with their
        // GPS instant on the MCU clock once the line is valid, for the raw
        // filter's check of each fix against the IMU (#1508). Off until
        // enableRawOutputs(); a full queue drops the newest.
        bool takeFix(gnss_raw::TimedFix& out);

        // ── Raw outputs for the raw-GNSS navigation filter ──
        // Off until enableRawOutputs(), called before begin(): it makes the
        // queues and the decode's state (false without the memory, and then
        // nothing is kept). Every take is safe from another task than the
        // poll; a full queue drops the newest and counts it.
        //
        // takeRawEpoch: each 0xE5 on the fused grid (setRawEpochPeriodMs,
        // GPS-time ms; 0, every epoch, until it is set) with at least one GPS
        // L1 C/A, Galileo E1 or BeiDou B1I cell. Its GPS instant on the MCU
        // clock, mcu_us, is its time of week less the receiver's clock bias
        // (the last 0xDF's, carried by its drift), through the arrival line;
        // 0, untimed, until the line is valid and the receiver has a fix.
        //
        // takeEphemeris: each ephemeris the subframes complete (SkyTraqNav.h),
        // decoded, in gps_eph_t's conventions (as RtcmEphemeris.h and
        // SbfNav.h fill it), for RawGnssMeasStage::addEphemeris.
        //
        // broadcastIono: the last GPS ionosphere (subframe 4 page 18) as the 8
        // broadcast bytes and the GPS day it came; false before one.
        bool enableRawOutputs();
        void setRawEpochPeriodMs(uint32_t period_ms);
        // The Dopplers in pairs: each 0xE5 off the grid read for its
        // Dopplers, and each grid epoch's averaged with them by
        // MosaicRawEpoch.h's rule (averageDopplerPairs) before it is handed
        // over. Call once, after enableRawOutputs(); false without the memory.
        bool setRawDopplerPairing(const mosaic::MosaicDopplerPairConfig& c);
        bool takeRawEpoch(gnss_raw::TimedRawEpoch& out);
        bool takeEphemeris(gps_eph_t& out);
        bool broadcastIono(int8_t raw[8], uint16_t& gps_day) const;

        // The raw outputs' counts since boot (the console lines). Read from
        // any task: a counter may be one behind another.
        struct RawStats
        {
            bool     enabled = false;
            uint32_t period_ms = 0;          // the fused grid
            // 0xE5
            uint32_t epochs = 0;             // decoded
            uint32_t published = 0;          // handed over (timed or not)
            uint32_t untimed = 0;            //   with mcu_us 0
            uint32_t decimated = 0;          // off the grid: nothing handed over (their Dopplers paired)
            uint32_t empty = 0;              // on the grid, with no cell the filter takes
            uint32_t dropped = 0;            // the queue was full
            uint32_t slips = 0;              // cells with the cycle-slip flag
            // the receiver's clock (0xDF)
            uint32_t clock_jumps = 0;        // whole-millisecond steps of its bias
            uint32_t clock_restarts = 0;     // its time going back or skipping
            float    bias_ms = 0.0f;         // the last bias, ms (receiver minus GPS)
            float    drift_ppm = 0.0f;       //   and its drift, ppm
            // the arrival line
            bool     line_valid = false;
            float    line_rms_ms = 0.0f;
            float    late90_ms = 0.0f;
            uint32_t line_restarts = 0;
            // navigation data, by GPS, Galileo, BeiDou
            uint32_t eph_queued[3] = {};
            uint32_t eph_dropped[3] = {};    // the queue was full
            uint32_t nav_frames[3] = {};     // subframes / pages read
            uint32_t nav_refused = 0;        // refused by the decoder (parity, CRC, cutover, GEO, ...)
            uint32_t ions = 0;               // ionospheres decoded
            // fixes
            uint32_t fixes = 0;              // 0xDF with a fix
            uint32_t fixes_dropped = 0;
        };
        void rawStats(RawStats& out) const;
        // The Dopplers in pairs, once on (as the mosaic driver's).
        struct RawPairStats
        {
            mosaic::MosaicDopplerPairStats pair;
            uint32_t reads = 0;
            uint32_t read_us_max = 0;
            uint64_t read_us_sum = 0;
            uint32_t adapts = 0;
            uint32_t adapt_us_max = 0;
            uint64_t adapt_us_sum = 0;
            uint32_t avg_us_max = 0;
        };
        bool rawPairStats(RawPairStats& out) const;

        // The link's state for the console and the bench.
        const skytraq::SkyTraqLink& link() const { return link_; }

    private:

        // FrameSink
        void onFrame(const uint8_t* body, size_t len, size_t index) override;

        bool uartBegin(uint32_t baud);
        size_t pump(uint32_t wait_ms, size_t max_bytes);
        void runLink();
        void drainUartEvents();
        void logStatus();
        void logChannels();
        void logSetup();
        int64_t arrivalUs(size_t index) const;
        void publishFix(const skytraq::RcvState& s, int64_t end_us);
        // the raw outputs, all in the poll task
        void rawE5(const uint8_t* body, size_t len, int64_t end_us);
        void rawDf(const skytraq::RcvState& s, int64_t end_us);
        void rawNav(const uint8_t* body, size_t len);

        uart_port_t _uartPort;
        uint8_t     rx_pin_ = 0;
        uint8_t     tx_pin_ = 0;
        uint8_t     update_rate_hz_ = 20;
        uint32_t    begin_timeout_ms_;
        bool        begin_wait_ = true;

        skytraq::FrameParser parser_;
        skytraq::SkyTraqLink link_;

        // The read the parser is working through, to tell when each byte arrived.
        uint8_t     rx_buf_[512] = {};
        size_t      rd_n_ = 0;
        size_t      rd_behind_ = 0;
        int64_t     rd_us_ = 0;
        double      byte_us_ = 1.0e7 / 115200.0;
        QueueHandle_t uart_events_ = nullptr;
        uint32_t    fifo_overflows_ = 0;
        uint32_t    uart_baud_ = 0;

        // Published fix
        GNSSData    data_ = {};
        bool        new_pvt_ = false;
        skytraq::ChannelSummary channels_ = {};
        bool        have_channels_ = false;
        // the last 0xE7 as it came, for the status log's C/N0 line (poll task only)
        uint8_t     last_e7_[4 + 7 * 80] = {};
        size_t      last_e7_len_ = 0;

        // Frames for the flight log (as the mosaic's ring: aligned storage)
        static constexpr size_t kRingBytes = 24576;
        alignas(8) uint8_t ring_mem_[kRingBytes] = {};
        StaticRingbuffer_t ring_struct_ = {};
        RingbufHandle_t   ring_ = nullptr;
        uint8_t           item_[4 + GNSS_SBF_MAX_FRAME] = {};
        uint32_t          ring_dropped_ = 0;
        uint32_t          too_long_ = 0;
        std::atomic<bool> draining_{false};
        uint32_t          short_buffer_ = 0;

        lc86::GnssTimeSync time_sync_;
        static constexpr int kClockQDepth = 4;
        StaticQueue_t     clock_q_struct_ = {};
        alignas(8) uint8_t clock_q_mem_[kClockQDepth * sizeof(GnssClockData)] = {};
        QueueHandle_t     clock_q_ = nullptr;
        uint32_t          clock_seq_seen_ = 0;
        uint32_t          clock_dropped_ = 0;

        // The receiver's clock from 0xDF (also without the raw outputs: the
        // time sync needs the bias)
        px1105r::Px1105rClock clk_ = {};

        // The raw outputs: everything they keep is in one block allocated by
        // enableRawOutputs().
        struct RawState
        {
            QueueHandle_t epoch_q = nullptr;     // TimedRawEpoch, internal RAM
            QueueHandle_t eph_q = nullptr;       // gps_eph_t
            QueueHandle_t fix_q = nullptr;       // gnss_raw::TimedFix
            std::atomic<uint32_t> period_ms{0};
            std::atomic<bool> taking_epochs{false};
            std::atomic<bool> taking_eph{false};
            std::atomic<bool> taking_fix{false};
            skytraq::RawE5 e5;                   // the 0xE5 being adapted
            gnss_raw::TimedRawEpoch scratch;
            px1105r::Px1105rLockState lock;
            skytraq::NavAssembler* nav = nullptr;   // its own block, PSRAM where there is some
            int8_t        iono_raw[8] = {};
            bool          iono_have = false;     // (under raw_mux_)
            uint16_t      iono_day = 0;
            RawStats      st;
            struct Pairing
            {
                mosaic::MosaicDopplerPairConfig cfg;
                mosaic::MosaicDopplerSet prev;   // the last off-grid epoch's Dopplers
                RawPairStats st;
            };
            std::atomic<Pairing*> pairing{nullptr};
        };
        RawState*         raw_ = nullptr;
        mutable portMUX_TYPE raw_mux_ = portMUX_INITIALIZER_UNLOCKED;

        // The 30 s status line
        int64_t           status_us_ = 0;
        uint32_t          n_e5_ = 0, n_df_ = 0, n_nav_ = 0, n_frames_ = 0, n_fix_ = 0;
        uint32_t          setups_logged_ = 0;
};

#endif
