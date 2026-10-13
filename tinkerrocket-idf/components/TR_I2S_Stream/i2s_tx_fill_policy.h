#pragma once

// What the FC's I2S sender writes next: keep the TX DMA ring full.
//
// The IDF TX driver hands i2s_channel_write() the DMA buffer the hardware
// released longest ago, and the hardware keeps cycling the ring whether or not
// anything new was written. The old sender wrote a frame when one was queued
// and 64 B of zeros once a tick otherwise, far below the link's 352.8 B/ms, so
// the ring ran nearly empty and a write could land in a buffer the DMA was
// about to read, or was already reading: the GDMA reads memory ahead of the
// EOF that tells the driver a buffer is done. Bytes copied in behind the read
// point go out as zeros. A frame that loses its SOF that way vanishes cleanly:
// the OC skips the rest as idle fill, and nothing on either side counts it.
// On the Beetle (2026-10-12) that cost ~0.9 IMU batches a second at 7680 Hz,
// almost always the first frame of a FIFO burst, written after the sender had
// been idle.
//
// With the ring full, every buffer the sender fills is the one the DMA has
// just released, which it reads again only after the rest of the ring
// (~20 ms at the 88200 link, against a read-ahead well under a millisecond):
//   - nothing queued: zeros, blocking while the ring is full. The link
//     carried zeros between frames anyway; now they are written ahead.
//   - away for more than a quarter of the ring (preempted: the GNSS task
//     alone runs 5-8 ms calls above the sender; or a new channel): refill it
//     with zeros, without blocking, before the next frame, so the frame goes
//     in behind them instead of nearer the read point. A blocking write
//     returns within one buffer (an eighth of the ring), so a quarter is
//     never reached while the sender is running.
//   - no channel (#1122): wait a tick per attempt rather than spin.
// A frame waits up to one ring in the DMA before it is sent (~23 ms).
//
// Header-only and IDF-free so the host tests pin it. Times are the 32-bit
// microsecond clock and may wrap; the comparison is wrap-safe.

#include <stdint.h>

class I2sTxFillPolicy
{
public:
    enum class Step : uint8_t
    {
        Refill,     // zeros into every free DMA buffer, without blocking
        SendFrame,  // write the queued frame
        IdleFill,   // nothing queued: IDLE_FILL_BYTES of zeros, blocking while the ring is full
        Backoff,    // no channel: wait a tick, call retry(), then a Refill tries it again
    };

    // Zeros per idle write: small enough that a frame queued behind it waits
    // ~0.7 ms of link at most, beyond the wait for a buffer to free.
    static constexpr uint32_t IDLE_FILL_BYTES = 256;

    // ring_us: how long the DMA takes to send the whole TX ring.
    explicit I2sTxFillPolicy(uint32_t ring_us) : ring_us_(ring_us) {}

    // frame_ready: a frame is waiting. now_us: the clock.
    Step next(bool frame_ready, uint32_t now_us) const
    {
        if (broken_) return Step::Backoff;
        if (!primed_ || (uint32_t)(now_us - last_write_us_) > ring_us_ / 4u) return Step::Refill;
        return frame_ready ? Step::SendFrame : Step::IdleFill;
    }

    // After every write. channel_ok: there was a channel to write to; a
    // timeout or a short write still counts (the ring was full, or the
    // write's own deadline passed), only ESP_ERR_INVALID_STATE does not.
    void wrote(bool channel_ok, uint32_t now_us)
    {
        if (!channel_ok)
        {
            broken_ = true;
            primed_ = false;
            return;
        }
        primed_ = true;
        last_write_us_ = now_us;
    }

    // The Backoff tick has passed: the next step tries the channel again.
    void retry() { broken_ = false; }

    // The ring was torn down or rebuilt (the OTA flip): start with a refill.
    void reset()
    {
        primed_ = false;
        broken_ = false;
    }

    uint32_t ringUs() const { return ring_us_; }

private:
    uint32_t ring_us_;
    uint32_t last_write_us_ = 0;
    bool primed_ = false;
    bool broken_ = false;
};
