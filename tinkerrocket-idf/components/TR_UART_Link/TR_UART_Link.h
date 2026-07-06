#pragma once

#include <compat.h>
#include <driver/uart.h>

#include "TR_MsgCodec.h"

// Framed UART transport for the radio-modem link (#409): tr_msg codec over
// an ESP-IDF UART port. Used on both ends — the daughterboard's host port
// and the OC/BS side of the link — so the two directions can never drift.
//
// Threading: begin() once, then sendFrame()/poll() from a single task (the
// main loop on both current hosts and the modem). No internal locking.
class TR_UART_Link
{
public:
    struct Config
    {
        uart_port_t port = UART_NUM_1;
        int tx_pin = -1;
        int rx_pin = -1;
        int baud = 921'600;
        // Driver ring buffers. RX sized for a burst of full frames arriving
        // while the loop is busy elsewhere; TX sized so sendFrame never
        // blocks in practice at link rates (LoRa airtime dominates).
        size_t rx_ring = 4096;
        size_t tx_ring = 4096;
    };

    struct Stats
    {
        uint32_t tx_frames = 0;
        uint32_t tx_fails = 0;      // pack error or ring-buffer write short
        uint32_t rx_frames = 0;
        uint32_t rx_crc_fails = 0;
        uint32_t rx_resync_bytes = 0;
    };

    esp_err_t begin(const Config& cfg);

    // Pack and queue one frame. Returns false on pack failure or if the
    // driver TX ring cannot take the whole frame within timeout_ms.
    bool sendFrame(uint8_t type, const uint8_t* payload, size_t len,
                   uint32_t timeout_ms = 20);

    // Drain all pending RX bytes through the deframer, invoking handler for
    // each complete valid frame. Non-blocking; call every loop iteration.
    using FrameHandler = void (*)(void* ctx, uint8_t type,
                                  const uint8_t* payload, size_t len);
    void poll(FrameHandler handler, void* ctx);

    void getStats(Stats& out) const;

private:
    Config cfg_{};
    bool began_ = false;
    tr_msg::MsgDeframer deframer_;
    Stats stats_{};
    uint8_t tx_frame_[tr_msg::MAX_FRAME] = {};
    uint8_t rx_chunk_[256] = {};
};
