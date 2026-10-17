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
//     GNSS_RTCM_MSG (0x95), the record that carries the LC86G's RTCM3;
//   - the MCU clock against GPS time from the 1PPS on J1 pin 5 (GPIO2 on the
//     V9), logged as GNSS_CLOCK_MSG (0x9D) with GCL_PPS;
//   - the navigation data again on request (requestNavResend(), at launch),
//     because the log starts at launch and the receiver sent it long before.
// All parsing is in SbfParser / MosaicLink (pure, host-tested); this class
// owns only the UART, the PPS interrupt and the hand-offs between tasks.
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
        // (exeSBFOnce). Any task; the poll task sends it.
        void requestNavResend() { nav_resend_.store(true, std::memory_order_relaxed); }

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

        uart_port_t _uartPort;
        uint8_t     rx_pin_ = 0;
        uint8_t     tx_pin_ = 0;
        int8_t      pps_pin_ = -1;
        uint8_t     update_rate_hz_ = 20;
        uint32_t    begin_timeout_ms_;

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

        std::atomic<bool> nav_resend_{false};
        uint32_t          nav_resends_ = 0;

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
