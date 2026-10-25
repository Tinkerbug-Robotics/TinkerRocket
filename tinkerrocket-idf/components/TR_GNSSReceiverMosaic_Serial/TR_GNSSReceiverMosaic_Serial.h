// TR_GNSSReceiverMosaic_Serial.h — Septentrio mosaic-G5 driver, ESP-IDF native UART
//
// Drop-in for TR_GNSSReceiverUBloxSerial / TR_GNSSReceiverLC86Serial: the
// same begin / pollNewPVT / getGNSSData surface and the same GNSSData,
// selected in TR_Sensor_Collector by TR_GNSS_DRIVER_MOSAIC (the V9 flying a
// mosaic-go G5 P3 on J1). It speaks SBF (Septentrio Binary Format): the
// mosaic-G5 has no RTCM3 output at all (its setDataInOut offers SBF and
// NMEA), so its raw measurements are SBF blocks.
//
// What it does:
//   - finds and sets up the receiver over COM1 (MosaicLink): 2 Mbaud; 20 Hz
//     MeasEpoch, MeasExtra and PVTGeodetic; 1 Hz status; navigation data as
//     it changes; 1PPS on GPS time; the same blocks logged on the receiver's
//     own microSD card; all of it saved into the receiver's boot
//     configuration, so a receiver that resets comes back streaming;
//   - GNSSData from PVTGeodetic (SbfParser.h: gnssDataFromPvt);
//   - every CRC-valid block, verbatim, into a ring the flight loop logs as
//     GNSS_RTCM_MSG (0x95), the record that carries the LC86G's RTCM3 (all
//     of them, or with the REDUCED GNSS log the raw measurements once a
//     second: gnss_log_policy.h);
//   - the MCU clock against GPS time from the 1PPS on J1 pin 5 (GPIO2 on the
//     V9), logged as GNSS_CLOCK_MSG (0x9D) with GCL_PPS;
//   - the navigation data again on request (requestNavResend(), at launch),
//     because the log starts at launch and the receiver sent it long before;
//   - with enableRawOutputs() (the mosaic-only image, whose navigation filter
//     is the raw-GNSS filter): each MeasEpoch on the filter's fused grid as a
//     TimedRawEpoch with its GPS instant on the MCU clock (MosaicRawEpoch.h),
//     each GPS, Galileo I/NAV and BeiDou MEO/IGSO ephemeris block as it came,
//     and the latest GPS ionosphere (SbfNav.h). Compiled into that image
//     alone (TR_GNSS_DRIVER_MOSAIC): the auto image, which carries this
//     driver too, keeps it exactly as it was.
// All parsing is in SbfParser / MosaicLink / SbfMeas / SbfNav / MosaicRawEpoch
// (pure, host-tested); this class owns only the UART, the PPS interrupt and
// the hand-offs between tasks.
#ifndef TRGNSSRECEIVERMOSAIC_H
#define TRGNSSRECEIVERMOSAIC_H

#include <atomic>
#include <cstdint>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>

#include <RocketComputerTypes.h>
#include <GnssTimeSync.h>      // TR_GNSSReceiverLC86_Serial: the clock line, shared
#include "MosaicLink.h"
#include "PpsPairer.h"
#include "SbfParser.h"
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
#include <RawMeas.h>           // TR_GNSSRaw: gnss_raw::TimedRawEpoch
#include "MosaicMultiband.h"
#include "MosaicRawEpoch.h"
#include "MosaicTimeTag.h"
#include "SbfNav.h"
#endif

class TR_GNSSReceiverMosaicSerial : private mosaic::SbfSink
{
    public:

        TR_GNSSReceiverMosaicSerial(uart_port_t uart_port = UART_NUM_1);

        /** No OTP high-performance clock on this part; present so callers
         *  need not branch on the driver (#837 item 6). */
        uint8_t otpState() const { return gnss_otp::NOT_M10; }

        // The GPIO the receiver's 1PPS reaches (J1 pin 5), or -1 for none.
        // Call before begin(). The pin is only ever read.
        void setPpsPin(int8_t pin) { pps_pin_ = pin; }

        // How long begin() looks for the receiver: 30 s unless set. The V9's
        // auto image (TR_GNSSAuto) shortens it when the u-blox driver's own
        // 60 s search ran first, to keep the boot inside its setup watchdog.
        void setBeginTimeoutMs(uint32_t ms) { begin_timeout_ms_ = ms; }

        // Whether begin() waits for the receiver (the default) or only starts
        // looking and returns true: the poll task (pollNewPVT) then finds and
        // sets it up whenever it answers, as it does after a receiver reset in
        // flight. The mosaic-only image's reboot in flight, so the flight
        // resumes without waiting on the receiver's boot.
        void setBeginWait(bool wait) { begin_wait_ = wait; }

        // Find the receiver and set it up (MosaicLink.h). Waits up to
        // the begin timeout for it, so a receiver that powers up with the FC
        // is found; false on a dead or deaf receiver (#557 GNSS-absent mode).
        // reset_n_pin / safeboot_n_pin: signature compatibility only.
        bool begin(uint8_t update_rate_hz,
                   uint8_t GNSS_RX,
                   uint8_t GNSS_TX,
                   int8_t reset_n_pin = -1,
                   int8_t safeboot_n_pin = -1);

        // The last published fix. Never touches the UART (#572 contract).
        void getGNSSData(GNSSData &data);

        /// Non-blocking: parse what the UART holds (bounded), keep the link
        /// set up, pair PPS edges; true with `data` filled when a new
        /// PVTGeodetic arrived. The GNSS poll task's only call into the UART.
        bool pollNewPVT(GNSSData &data);

        /// GNSS_SAT_MSG has no source here yet: the per-satellite C/N0 is in
        /// the logged MeasEpoch blocks. Always false.
        bool pollNewSat(GNSSSatData &out) { (void)out; return false; }

        /// The oldest SBF block not yet taken, verbatim from "$@" through its
        /// last byte, and the MCU time that byte arrived. Safe from another
        /// task than the poll. `cap` must be GNSS_SBF_MAX_FRAME.
        bool pollRawFrame(uint8_t* out, size_t cap, size_t& len, uint32_t& time_us);
        uint32_t rawFramesDropped() const { return ring_dropped_ + too_long_ + short_buffer_; }

        // The MCU clock against GPS time from the PPS edges (GnssTimeSync.h,
        // latency 0: an edge is the GPS second). takeClockLine: the line each
        // time it moves (about once a second with PPS), for GNSS_CLOCK_MSG.
        const lc86::GnssTimeSync& timeSync() const { return time_sync_; }
        bool takeClockLine(GnssClockData& out);
        uint32_t clockLinesDropped() const { return clock_dropped_; }

        // The geoid above the ellipsoid at the last fix, m. False before one.
        bool geoidSeparationM(float& sep_m) const
        {
            sep_m = geoid_sep_m_.load(std::memory_order_relaxed);
            return geoid_sep_valid_.load(std::memory_order_relaxed);
        }

        // Ask the receiver to send every navigation block it holds once more
        // (exeSBFOnce). Any task; the poll task sends it. With the raw outputs
        // on it is also asked after every link setup: the filter needs the
        // ephemerides the receiver decoded before this boot (OnChange sent
        // them then).
        void requestNavResend() { nav_resend_.store(true, std::memory_order_relaxed); }

#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
        // ── Raw outputs for the raw-GNSS navigation filter (#1600 on the mosaic) ──
        // The mosaic-only image's alone (see the top of this file).
        // Off until enableRawOutputs(), called before begin(): it makes the
        // queues and the decode's scratch (false without the memory, and
        // then nothing is kept). Every take is safe from another task than
        // the poll; a full queue drops the newest and counts it, once
        // something has started taking from it.
        //
        // takeRawEpoch: each MeasEpoch on the fused grid (setRawEpochPeriodMs,
        // GPS-time milliseconds; 0, every epoch, until it is set) with at
        // least one GPS L1 C/A, Galileo E1 or BeiDou B1I cell
        // (MosaicRawEpoch.h). Its GPS instant on the MCU clock, mcu_us, is
        // TOW less the receiver's clock bias then (the last PVTGeodetic whose
        // own MeasEpoch was seen, carried by its drift and the clock jumps
        // since, for up to 120 s), through the 1PPS line -- and 0, untimed,
        // unless it is right: the line valid, the receiver's time FINETIME,
        // an edge paired with its second within 120 s, a bias to carry, the
        // last edge on the line and where the PVTs' own account of GPS time
        // puts it (the chain), and the bias's chain confirmed by the 1PPS
        // after any break (a receiver restart, a step in its time against its
        // clock, a failed PPS check, a restart of the line). MosaicTimeTag.h
        // says why.
        //
        // takeEphemeris: each GPSNav, GALNav or BDSNav block as it came
        // (CRC checked), for SbfNav's decodeSbfEphemeris. Galileo F/NAV and
        // BeiDou GEO, which the decoder refuses, are counted and never
        // queued. A block longer than `cap` is dropped and counted.
        //
        // broadcastIono: the last GPSIon's 8 Klobuchar bytes as broadcast and
        // the GPS day it came (decodeGpsIon); false before one. Not taken:
        // the caller compares it with what it has.
        bool enableRawOutputs();
        void setRawEpochPeriodMs(uint32_t period_ms);
        // The Dopplers in pairs (MosaicRawEpoch.h; the receiver profile's
        // rr_pair_avg, off unless set): each MeasEpoch off the grid read for
        // its Dopplers, and each grid epoch's averaged with them by the rule
        // there before it is handed over. Call once, after
        // enableRawOutputs(), from any task; false without the memory for
        // its state (~1.3 kB, internal RAM), and then nothing changes. Never
        // called, every epoch is handed over exactly as before.
        bool setRawDopplerPairing(const mosaic::MosaicDopplerPairConfig& c);
        // Each satellite's other bands in its row's Doppler (MosaicMultiband.h;
        // config.h TR_RAW_MULTIBAND, on in the mosaic image): every MeasEpoch's
        // anchor (GPS L1 C/A, Galileo E1, BeiDou B1I) checked against the same
        // satellite's other bands -- voted out by two that agree, dropped by the
        // drift guard, or averaged with them -- before the epoch is paired or
        // handed over. windup_k is the antenna's carrier wind-up, cycles a turn
        // (the profile's; 0.99 on the Goblin's helix), for the gyro's check on
        // the combiner's own wind-up (noteRollRate). Call once, after
        // enableRawOutputs() and setRawDopplerPairing(), from any task; false
        // without the memory for its state (~27 kB, internal RAM, else PSRAM),
        // and then nothing changes. Never called, every epoch is the L1 band's.
        bool setRawMultiband(const mosaic::MosaicMultibandConfig& c, float windup_k);
        // The IMU path's roll rate about the nose (rad/s, body x), each
        // flight-filter tick, from one task: the combiner's check on its own
        // wind-up estimate, taken when no older than kMbGyroMaxAgeUs. Never
        // called, the combiner runs on its estimate alone.
        void noteRollRate(float p_rps);
        static constexpr int64_t kMbGyroMaxAgeUs = 20000;
        bool takeRawEpoch(gnss_raw::TimedRawEpoch& out);
        bool takeEphemeris(uint8_t* out, size_t cap, size_t& len);
        bool broadcastIono(int8_t raw[8], uint16_t& gps_day) const;

        // The raw outputs' counts since boot (the 30 s status line, the
        // [RAWGNSS] console line). Read from any task: a counter may be one
        // behind another.
        struct RawStats
        {
            bool     enabled = false;
            bool     nav_in_psram = false;   // the navigation queue: 64 deep in PSRAM, or 24 in internal RAM
            uint16_t nav_depth = 0;
            uint32_t period_ms = 0;          // the fused grid
            // MeasEpoch
            uint32_t published = 0;          // handed over (timed or not)
            uint32_t untimed = 0;            //   with mcu_us 0 (why: tag.why)
            uint32_t decimated = 0;          // off the grid: their clock jumps noted, nothing handed over
            uint32_t empty = 0;              // on the grid, refused or with no cell: not handed over
            uint32_t dropped = 0;            // the queue was full
            uint32_t clock_jumps = 0;        // the receiver's 1 ms clock jumps seen
            uint32_t clock_restarts = 0;     //   and its jump count starting over (a reset, a gap, a big jump)
            uint32_t clock_resets = 0;       //   of which the receiver's clock was lost (MosaicJumpState::resets)
            uint32_t clock_resolved = 0;     //   jumps the arrivals placed past the count's 256 ms
            uint32_t line_steps = 0;         // the PPS line moved onto a step of its edges (GnssTimeSync)
            double   line_last_step_ms = 0.0;
            uint32_t line_later_refused = 0; // edges set aside as later than the line with no PVT step
            uint32_t line_restarts = 0;      // the PPS line's restarts
            uint32_t pairs_refused_late = 0; // 1PPS pairings refused while the link was backed up
            // navigation blocks, by GPS, Galileo, BeiDou
            uint32_t nav_queued[3] = {};
            uint32_t nav_filtered[3] = {};   // F/NAV, GEO or too short: what the decoder would refuse
            uint32_t nav_dropped[3] = {};    // the queue was full
            uint32_t nav_too_long = 0;       // longer than kSbfNavMaxLen
            uint32_t nav_short_buffer = 0;   // longer than the taker's buffer
            uint32_t ions = 0;               // GPSIon blocks decoded
            uint32_t ions_bad = 0;           //   and refused
            // the time tag (MosaicTimeTag.h): the untimed by why, the
            // chain's breaks and confirmations, and the PPS tag check: each
            // whole-second epoch's tag against its 1PPS edge (ppsResidualUs,
            // with that second's own RxClkBias), or where the edge was due
            // when the interrupt stamped it late, judged on the seconds handed
            // over timed once the line has the crystal's drift (notePpsTag);
            // the bar is kPpsTagBarUs
            mosaic::MosaicTimeTag::Stats tag;
        };
        void rawStats(RawStats& out) const;
        // The Dopplers in pairs (setRawDopplerPairing), once on: the epochs
        // in pairs and as they came, each Doppler averaged, left or dropped
        // (MosaicDopplerPairStats), and what the poll task spent on them --
        // each off-grid epoch's read, and each grid epoch's adapting
        // (rawEpochFromMeasEpoch) and averaging, us. Kept with the pairs'
        // state, apart from RawStats, which tasks copy onto their stacks.
        // False, out zeroed, with the pairs off. As rawStats: a counter may be
        // one behind another.
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
        // The bands combined (setRawMultiband), once on: the combiner's counts
        // (MosaicMultibandStats: combined, anchor-only and why, votes, the drift
        // guard), the gyro's wind-up given or not, and what the poll task spent
        // on each combination (the grid epoch's and the epoch between's), us.
        // False, out zeroed, with it off. A counter may be one behind another.
        struct RawMbStats
        {
            mosaic::MosaicMultibandStats mb;
            uint32_t runs = 0;
            uint32_t us_max = 0;
            uint64_t us_sum = 0;
            uint32_t gyro_given = 0;
            uint32_t gyro_stale = 0;
        };
        bool rawMbStats(RawMbStats& out) const;
        static constexpr int32_t kPpsTagBarUs = 200;   // a right tag leaves ~0.03 ms rms
        // An edge this far off the line, or a whole second's tag this far
        // off its edge, holds the tag back (MosaicTimeTag.h), and the line
        // follows a step of its edges this big (GnssTimeSync exact_step_s).
        static constexpr int32_t kPpsOnLineUs = 100;
        // No 1PPS edge pairs while the last MeasEpoch came this much later
        // than the earliest lately (MosaicJumpState::late_us): a block a
        // second behind would pair the next second's edge.
        static constexpr int64_t kPairMaxLateUs = 300000;
#endif

    private:

        // SbfSink
        void onBlock(const uint8_t* block, size_t len, size_t index) override;
        void onLine(const char* line, size_t index) override;
        void onPrompt(const char* port, size_t index) override;

        bool uartBegin(uint32_t baud);   // false: the port would not come up at this baud
        void uartEnd();
        // Read once (up to `wait_ms` for the first byte) and feed the parser.
        size_t pump(uint32_t wait_ms, size_t max_bytes);
        // Carry out what the link asks for until it asks nothing.
        void runLink();
        void setupPps();
        static void ppsIsr(void* arg);
        void pairPps(double gps_s, int64_t arrival_us);
        void noteDelay(uint16_t number, uint32_t tow_ms, uint16_t wnc, int64_t end_us);
        void drainUartEvents();
        void logStatus();
        void logSetup();
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
        // the raw outputs (enableRawOutputs), all in the poll task
        void rawMeasEpoch(const uint8_t* block, size_t len, uint32_t tow_ms, int64_t end_us);
        void rawPvt(const mosaic::PvtGeodetic& p);
        void rawNav(const uint8_t* block, size_t len, uint16_t number);
        void rawIon(const uint8_t* block, size_t len);
        void logRawStatus();
#if defined(TR_BENCH_MOSAIC_TAG_TRACE) && TR_BENCH_MOSAIC_TAG_TRACE
        void benchTagTrace(const mosaic::PvtGeodetic& p);
#endif
#endif

        uart_port_t _uartPort;
        uint8_t     rx_pin_ = 0;
        uint8_t     tx_pin_ = 0;
        int8_t      pps_pin_ = -1;
        uint8_t     update_rate_hz_ = 20;
        uint32_t    begin_timeout_ms_;
        bool        begin_wait_ = true;

        mosaic::SbfParser parser_;
        mosaic::MosaicLink link_;
        mosaic::SetupCommand cmds_[12] = {};
        size_t      n_cmds_ = 0;
        char        fast_com_[96] = {};   // the 20 Hz stream commands, built from the rate
        char        fast_dsk_[96] = {};

        // The read the parser is working through, to tell when each byte arrived.
        uint8_t     rx_buf_[512] = {};
        size_t      rd_n_ = 0;
        size_t      rd_behind_ = 0;
        int64_t     rd_us_ = 0;
        double      byte_us_ = 1.0e7 / 115200.0;
        QueueHandle_t uart_events_ = nullptr;
        uint32_t    fifo_overflows_ = 0;   // the UART's FIFO or ring ran over: bytes lost
        uint32_t    uart_baud_ = 0;

        // Published fix
        GNSSData    data_ = {};
        bool        new_pvt_ = false;
        int         leap_s_ = -128;        // GPS - UTC from ReceiverTime; -128 until known
        uint16_t    pdop_x100_ = 0;
        std::atomic<float> geoid_sep_m_{0.0f};
        std::atomic<bool>  geoid_sep_valid_{false};

        // SBF blocks for the flight log. 24 kB: ~0.35 s at the full set,
        // drained by the flight loop every pass; it only rides out the boot.
        // alignas: IDF's ring buffer asserts its storage is 32-bit aligned on
        // every send (ringbuf.c rbCHECK_ALIGNED), and as a plain byte array
        // behind a bool it sat at an odd offset: the first block the receiver
        // sent crashed the FC.
        static constexpr size_t kRingBytes = 24576;
        alignas(8) uint8_t ring_mem_[kRingBytes] = {};
        StaticRingbuffer_t ring_struct_ = {};
        RingbufHandle_t   ring_ = nullptr;
        uint8_t           item_[4 + GNSS_SBF_MAX_FRAME] = {};
        uint32_t          ring_dropped_ = 0;     // full ring while the flight loop drains it
        uint32_t          too_long_ = 0;         // longer than GNSS_SBF_MAX_FRAME
        std::atomic<bool> draining_{false};      // the flight loop has started taking blocks
        uint32_t          short_buffer_ = 0;     // written by the taking task only

        // PPS: edges from the interrupt, paired with GPS seconds here.
        static constexpr uint32_t kPpsRing = 8;
        volatile int64_t  pps_edges_[kPpsRing] = {};
        std::atomic<uint32_t> pps_head_{0};      // written by the ISR only
        uint32_t          pps_tail_ = 0;         // read by the poll task only
        volatile int64_t  pps_last_isr_us_ = 0;
        volatile uint32_t pps_glitches_ = 0;     // edges too soon after the last
        volatile uint32_t pps_burst_ = 0;
        volatile bool     pps_muted_ = false;    // interrupt off after a burst of glitches
        int64_t           pps_muted_at_us_ = 0;
        mosaic::PpsPairer pairer_;
        uint32_t          pps_edges_seen_ = 0;

        lc86::GnssTimeSync time_sync_;
        static constexpr int kClockQDepth = 4;
        StaticQueue_t     clock_q_struct_ = {};
        alignas(8) uint8_t clock_q_mem_[kClockQDepth * sizeof(GnssClockData)] = {};
        QueueHandle_t     clock_q_ = nullptr;
        uint32_t          clock_seq_seen_ = 0;
        uint32_t          clock_dropped_ = 0;
#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
        uint32_t          pairs_refused_late_ = 0;   // pairings refused while the link was backed up
#endif
#if defined(TR_BENCH_MOSAIC_TAG_TRACE) && TR_BENCH_MOSAIC_TAG_TRACE
        int64_t           xpps_s_ = 0;               // the last xPPSOffset's GPS second
        int               xpps_sync_age_ = -1;       //   and its SyncAge, s
#endif

        std::atomic<bool> nav_resend_{false};
        uint32_t          nav_resends_ = 0;

#if defined(TR_GNSS_DRIVER_MOSAIC) && TR_GNSS_DRIVER_MOSAIC
        // The raw outputs: everything they keep is in one block allocated by
        // enableRawOutputs(), so a mosaic image that never asks for them (the
        // -DTR_RAW_GNSS_SHADOW=0 one) carries only this pointer and the lock.
        struct NavItem
        {
            uint8_t len;
            uint8_t bytes[mosaic::kSbfNavMaxLen];
        };
        static_assert(mosaic::kSbfNavMaxLen <= 255, "NavItem::len is a byte");
        struct RawState
        {
            QueueHandle_t epoch_q = nullptr;     // TimedRawEpoch, internal RAM
            QueueHandle_t nav_q = nullptr;       // NavItem, PSRAM where there is some
            bool          nav_caps = false;      //   made by xQueueCreateWithCaps (deleted with its own call)
            std::atomic<uint32_t> period_ms{0};  // the fused grid (setRawEpochPeriodMs)
            std::atomic<bool> taking_epochs{false};
            std::atomic<bool> taking_nav{false};
            gnss_raw::TimedRawEpoch scratch;     // the MeasEpoch being adapted
            NavItem       nav_item = {};         // the block being queued
            mosaic::MosaicJumpState jump;        // every MeasEpoch's clock jumps
            // the tag: FINETIME, the anchoring PVT and its chain, the last
            // edge paired, the whole second's check (MosaicTimeTag.h)
            mosaic::MosaicTimeTag tag;
            // the last whole-second epoch's tag as made (a bench trace's)
            int64_t       sec_rx_ms = 0;
            double        sec_dtr_ms = 0.0;
            bool          sec_placed = false;
            mosaic::MosaicTimeTag::Why sec_why = mosaic::MosaicTimeTag::Why::NO_LINE;
            // the last GPSIon (under raw_mux_)
            bool          iono_have = false;
            int8_t        iono_raw[8] = {};
            uint16_t      iono_day = 0;
            RawStats      st;
            // the Dopplers in pairs: made by setRawDopplerPairing, null (off)
            // until then; the poll task alone writes what it points to (and
            // reads all but st, which rawPairStats copies for any task)
            struct Pairing
            {
                mosaic::MosaicDopplerPairConfig cfg;
                mosaic::MosaicDopplerSet prev;   // the last off-grid epoch's Dopplers
                RawPairStats st;
            };
            std::atomic<Pairing*> pairing{nullptr};
            // the bands combined: made by setRawMultiband, null (off) until
            // then; the poll task alone writes what it points to
            struct Multiband
            {
                mosaic::MosaicMultibandConfig cfg;
                float windup_k = 0.99f;
                mosaic::MosaicMbScratch scr;
                mosaic::MosaicMbOut out;        // the grid epoch's side outputs
                mosaic::MosaicMbOut out_prev;   //   and the epoch between's
                RawMbStats st;
            };
            std::atomic<Multiband*> multiband{nullptr};
            // the IMU path's last roll rate (noteRollRate), a seqlock: one
            // writer, the poll task reads
            std::atomic<uint32_t> roll_seq{0};
            float   roll_p = 0.0f;
            int64_t roll_us = 0;
        };
        RawState*         raw_ = nullptr;
        // the gyro's wind-up for the MeasEpoch being decoded, and a
        // combination's time, counted (setRawMultiband)
        static mosaic::MosaicMbWindupIn mbGyroWindup(RawState& r, RawState::Multiband& m);
        static void mbTimed(RawMbStats& st, int64_t t0_us);
        mutable portMUX_TYPE raw_mux_ = portMUX_INITIALIZER_UNLOCKED;
#endif

        // The 30 s status line
        int64_t           status_us_ = 0;
        uint32_t          n_meas_ = 0, n_extra_ = 0, n_pvt_ = 0, n_blocks_ = 0;
        uint32_t          n_fix_ = 0;
        struct DelayStats { uint32_t n = 0; double sum_ms = 0.0; double max_ms = 0.0; };
        DelayStats        delay_meas_, delay_pvt_;
        mosaic::ReceiverStatus rx_status_ = {};
        mosaic::DiskStatus disk_ = {};
        bool              have_rx_status_ = false;
        bool              setup_logged_ = true;
        uint32_t          setups_logged_ = 0;
};

#endif
