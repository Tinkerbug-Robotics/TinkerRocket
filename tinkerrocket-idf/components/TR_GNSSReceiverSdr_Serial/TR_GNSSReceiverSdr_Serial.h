// TR_GNSSReceiverSdr_Serial.h — TinkerRocket's gnss-sdr-dev board on J1, ESP-IDF native UART
//
// The board (gnss repo, firmware/gnss-sdr-dev) is TinkerRocket's own GNSS
// receiver. On a V9 or V10's J1 it sends the flight computer RTCM 3 at
// 2 Mbaud (SdrRtcm.h has what): about 400 frames and 40-55 kB a second at its
// measurement mode's 100 Hz. It takes no commands, only TinkerRocket's 4002
// frames. This driver listens, and sends what the flight task hands it:
//   - every frame whose CRC-24Q checks, verbatim, into a ring the flight
//     loop logs as GNSS_RTCM_MSG (0x95), the record that carries the
//     LC86G's RTCM3 and the mosaic-G5's SBF (Data_Analysis/
//     extract_gnss_rtcm.py turns a log into a .rtcm3 file);
//   - no GNSSData: the navigation filter does not take the board's
//     measurements or fixes yet, so the FC flies GNSS-absent with it
//     (gnssTypeGivesFix) and logs its stream beside the IMU;
//   - its counts on the console every 30 s (frames, epochs and their gaps,
//     fixes, CRC fails, drops);
//   - the MCU clock against GPS time, fitted to the board's epochs
//     (GnssTimeSync, each epoch at its latch, those the board marks late
//     left out), handed to the flight loop each time it moves
//     (takeClockLine, logged as GNSS_CLOCK_MSG): what every 4002 frame is
//     stamped by;
//   - sendAidFrame, the flight task's way to send the board TinkerRocket's
//     message 4002 (SdrAid.h) without ever blocking. A flight image sends
//     the real flight phase with it (sub-type 3: projects/flight_computer/
//     main/sdr_phase_policy.h), which picks the board's boost loops, and
//     its filter's attitude and its rests (sub-types 1 and 5:
//     sdr_aid_policy.h, placed at the board's own fix, sub-type 2); a
//     bench image (TR_BENCH_SDR_AID) sends the P1 script's frames instead,
//     and prints a second console line: what went out, and what the board
//     says it took (the aiding block of its epochs).
// begin() needs nothing from the board: before its first fix the board sends
// only its status, so after a cold power-up its stream starts long after the
// FC has booted, and the ring fills from then on. Called by TR_GNSSAuto once
// the probes found the board, or remembered it from last boot.
// The framing is SdrRtcm and the 4002 frames SdrAid (both pure, host-tested);
// this class owns only the UART and the hand-off between tasks.
#ifndef TRGNSSRECEIVERSDR_H
#define TRGNSSRECEIVERSDR_H

#include <atomic>
#include <cstdint>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>

#include <RocketComputerTypes.h>
#include "SdrAid.h"
#include "SdrRtcm.h"
#include <GnssTimeSync.h>      // TR_GNSSReceiverLC86_Serial: the clock line, shared

class TR_GNSSReceiverSdrSerial : private sdr_rtcm::Rtcm3Framer::Sink
{
    public:

        TR_GNSSReceiverSdrSerial(uart_port_t uart_port = UART_NUM_1);

        uint8_t otpState() const { return gnss_otp::NOT_M10; }

        // Open the UART at the board's 2 Mbaud and listen. False only when
        // the UART will not come up. reset_n_pin / safeboot_n_pin and the
        // rate: signature compatibility (the board's rate is its own).
        bool begin(uint8_t update_rate_hz,
                   uint8_t GNSS_RX,
                   uint8_t GNSS_TX,
                   int8_t reset_n_pin = -1,
                   int8_t safeboot_n_pin = -1);

        // Never a fix (see the top of this file).
        void getGNSSData(GNSSData &data) { data = GNSSData{}; }

        /// Non-blocking: frame what the UART holds (bounded) into the log
        /// ring. Always false: no PVT. The GNSS poll task's only call into
        /// the UART.
        bool pollNewPVT(GNSSData &data);

        /// The oldest frame not yet taken, from its 0xD3 through its CRC, and
        /// the MCU time its last byte arrived. Safe from another task than the
        /// poll. `cap` at least sdr_rtcm::kMaxFrame (GNSS_RTCM_MAX_FRAME).
        bool pollRawFrame(uint8_t* out, size_t cap, size_t& len, uint32_t& time_us);
        uint32_t rawFramesDropped() const { return ring_dropped_ + short_buffer_; }

        // Whether a 4001 frame has come yet this boot (the board is streaming).
        bool heardBoard() const { return tr_frames_.load(std::memory_order_relaxed) > 0; }

        /// One whole 4002 frame (SdrAid.h) to the board, from the flight
        /// task: the UART's one writer once the board is found. False at
        /// once, and counted, with no driver, no room for it in the TX ring
        /// or a frame of the wrong size. Never blocks: the ring's room is
        /// checked before the write, and nothing waits for the line.
        bool sendAidFrame(const uint8_t* frame, size_t len);

        /// The clock line each time it moves (about once a second while the
        /// board's epochs come), oldest first, for GNSS_CLOCK_MSG and the
        /// 4002 stamps (the flight image's phase, the bench's script). Safe
        /// from another task than the poll.
        bool takeClockLine(GnssClockData& out);

    private:
        void onFrame(const uint8_t* frame, size_t len, size_t after) override;
        void noteEpoch(const sdr_rtcm::TrEpoch& e);
        void pump(size_t max_bytes);
        void drainUartEvents();
        void logStatus();
        void noteLink(const uint8_t* frame, size_t len, const sdr_rtcm::TrEpoch& e, int64_t end_us);
#if defined(TR_BENCH_SDR_AID) && TR_BENCH_SDR_AID
        void logLink();
#endif

        uart_port_t _uartPort;
        uint8_t     rx_pin_ = 0;
        uint8_t     tx_pin_ = 0;

        sdr_rtcm::Rtcm3Framer framer_;

        // The read the framer is working through, to tell when each byte arrived.
        uint8_t     rx_buf_[1024] = {};
        size_t      rd_behind_ = 0;
        int64_t     rd_us_ = 0;
        QueueHandle_t uart_events_ = nullptr;
        uint32_t    fifo_overflows_ = 0;   // the UART's FIFO or ring ran over: bytes lost

        // Frames for the flight log. 32 kB: ~0.6 s of the stream with the
        // ring's own headers, drained by the flight loop every pass; it rides
        // out the boot and a long pass. alignas: IDF's ring buffer asserts its
        // storage is 32-bit aligned on every send (ringbuf.c rbCHECK_ALIGNED;
        // the mosaic driver's first block crashed the FC without it).
        static constexpr size_t kRingBytes = 32768;
        alignas(8) uint8_t ring_mem_[kRingBytes] = {};
        StaticRingbuffer_t ring_struct_ = {};
        RingbufHandle_t   ring_ = nullptr;
        alignas(4) uint8_t item_[4 + sdr_rtcm::kMaxFrame] = {};
        uint32_t          ring_dropped_ = 0;     // full ring while the flight loop drains it
        std::atomic<bool> draining_{false};      // the flight loop has started taking frames
        uint32_t          short_buffer_ = 0;     // written by the taking task only

        // What the stream held, for the console.
        std::atomic<uint32_t> tr_frames_{0};     // 4001 frames, any sub-type
        uint32_t    n_msm_ = 0, n_epochs_ = 0, n_fixes_ = 0, n_fix_valid_ = 0, n_eph_ = 0, n_other_ = 0;
        uint32_t    epoch_gaps_ = 0;             // epochs missing between two that came
        uint32_t    epoch_backsteps_ = 0;        // an epoch at or before the last one
        uint32_t    last_tow_ms_ = 0;
        uint8_t     last_mode_ = 0xFF;
        uint32_t    step_ms_[4] = {};            // each mode's epoch step, the shortest seen
        uint8_t     last_nobs_ = 0;
        uint8_t     last_fix_nsat_ = 0;
        bool        last_fix_valid_ = false;
        uint32_t    n_status_ = 0;               // sub-type 3: the board up, no GPS time yet
        sdr_rtcm::TrStatus last_status_;
        int64_t     status_us_ = 0;
        uint32_t    status_frames_ = 0, status_epochs_ = 0;

        // The 4002 frames sendAidFrame sent, by sub-type (the low four bits of
        // the frame's fifth byte), and those it did not: written by the flight
        // task, read by the console's.
        std::atomic<uint32_t> aid_sent_[16] = {};
        std::atomic<uint32_t> aid_tx_full_{0};
        std::atomic<uint32_t> aid_no_uart_{0};
        std::atomic<uint32_t> aid_bad_{0};       // a frame of the wrong size, or a write cut short

        // The MCU clock against GPS time from the board's epochs (this task
        // alone touches time_sync_), to the flight loop through a queue. On
        // every image with this driver (about 3.6 kB, most of it the line's
        // history), so a flight image can stamp its phase frames.
        lc86::GnssTimeSync time_sync_;
        static constexpr int kClockQDepth = 4;
        StaticQueue_t     clock_q_struct_ = {};
        alignas(8) uint8_t clock_q_mem_[kClockQDepth * sizeof(GnssClockData)] = {};
        QueueHandle_t     clock_q_ = nullptr;
        uint32_t          clock_seq_seen_ = 0;
        uint32_t          clock_dropped_ = 0;
        uint32_t          late_left_out_ = 0;    // epochs the board marked late, not on the line

#if defined(TR_BENCH_SDR_AID) && TR_BENCH_SDR_AID
        // What the board says it does with them (the aiding block), and the
        // 4002 frames it took, from its 8-bit count against what went out.
        sdr_aid::EpochAid last_aid_;
        uint32_t          n_aid_ = 0;            // epochs that carried the block
        sdr_aid::EchoCount echo_;
        // The counts at the console's last link line.
        uint32_t          line_sent_[sdr_aid::kGeometry + 1] = {};
        uint32_t          line_took_ = 0;
        uint32_t          line_of_ = 0;
        uint32_t          line_aside_ = 0;
#endif
};

#endif
