// TR_GNSSReceiverSdr_Serial.h — TinkerRocket's gnss-sdr-dev board on J1, ESP-IDF native UART
//
// The board (gnss repo, firmware/gnss-sdr-dev) is TinkerRocket's own GNSS
// receiver. On a V9 or V10's J1 it sends the flight computer RTCM 3 at
// 2 Mbaud (SdrRtcm.h has what): about 400 frames and 40-55 kB a second at its
// measurement mode's 100 Hz. It takes no commands, so this driver only
// listens:
//   - every frame whose CRC-24Q checks, verbatim, into a ring the flight
//     loop logs as GNSS_RTCM_MSG (0x95), the record that carries the
//     LC86G's RTCM3 and the mosaic-G5's SBF (Data_Analysis/
//     extract_gnss_rtcm.py turns a log into a .rtcm3 file);
//   - no GNSSData: the navigation filter does not take the board's
//     measurements or fixes yet, so the FC flies GNSS-absent with it
//     (gnssTypeGivesFix) and logs its stream beside the IMU;
//   - its counts on the console every 30 s (frames, epochs and their gaps,
//     fixes, CRC fails, drops).
// begin() needs nothing from the board: the board sends nothing before its
// first fix, so after a cold power-up it starts its stream long after the FC
// has booted, and the ring fills from then on. Called by TR_GNSSAuto once the
// probes found the board, or remembered it from last boot.
// The framing is SdrRtcm (pure, host-tested); this class owns only the UART
// and the hand-off between tasks.
#ifndef TRGNSSRECEIVERSDR_H
#define TRGNSSRECEIVERSDR_H

#include <atomic>
#include <cstdint>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>

#include <RocketComputerTypes.h>
#include "SdrRtcm.h"

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

    private:
        void onFrame(const uint8_t* frame, size_t len, size_t after) override;
        void noteEpoch(const sdr_rtcm::TrEpoch& e);
        void pump(size_t max_bytes);
        void drainUartEvents();
        void logStatus();

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
};

#endif
