#pragma once

// #1485: timestamps for IMU samples drained from the FIFO in bursts.
//
// A sample read one per DRDY edge was stamped with the time it was read, so
// each stamp carried the read latency (0-400 us on the mini). Samples drained
// from the FIFO arrive several at a time, so there is no read time per sample
// to use at all. What the drain does give is a count: the chip produced these
// n samples one period apart, and the newest of them within one period before
// the FIFO level was read.
//
// So the clock hands out evenly spaced times and steers them gently toward
// that anchor:
//   - the newest sample of a burst is taken to be half a period before the
//     level read (the true value is anywhere in the period before it);
//   - the phase moves a fraction of the error per burst, and the period
//     follows the error more slowly, so the steady-state period matches the
//     chip's actual rate (its oscillator is a few percent off nominal);
//   - an error beyond RESYNC_PERIODS periods (a FIFO overflow, a long stall)
//     re-anchors at once instead of slewing;
//   - times never run backwards, and consecutive samples are never closer
//     than half a period.
//
// Header-only and IDF-free, so the host tests pin it. Times are the 32-bit
// microsecond clock and may wrap; all comparisons are wrap-safe.

#include <stdint.h>

class ImuSampleClock
{
public:
    static constexpr float PHASE_GAIN = 0.03f;      // share of the phase error fixed per burst
    static constexpr float FREQ_GAIN = 0.0005f;      // share folded into the period, per sample
    static constexpr float PERIOD_LIMIT = 0.05f;    // the period may drift +/-5 % from nominal
    static constexpr float RESYNC_PERIODS = 8.0f;   // beyond this the clock re-anchors

    uint32_t resyncs = 0;  // hard re-anchors since reset()

    // period_us: the nominal sample period, 1e6 / ODR.
    void reset(float period_us)
    {
        nominal_ = period_us;
        period_ = period_us;
        primed_ = false;
        pending_ = 0;
        resyncs = 0;
    }

    float period() const { return period_; }

    // Call once per drained burst, before next(). t_read_us: the clock when the
    // FIFO level was read. n: complete samples in this burst.
    void beginBurst(uint32_t t_read_us, uint32_t n)
    {
        pending_ = n;
        if (n == 0) return;

        // Where the newest sample of this burst really is: half a period
        // before the level read, give or take half a period.
        const float span = (float)(n - 1) * period_;
        if (!primed_)
        {
            anchorFirst(t_read_us, span);
            primed_ = true;
            return;
        }

        // Where the clock would put it, relative to the level read.
        const float predicted = offsetFrom(t_read_us, next_base_) + next_frac_ + span;
        const float err = -0.5f * period_ - predicted;

        if (err > RESYNC_PERIODS * period_ || err < -RESYNC_PERIODS * period_)
        {
            resyncs++;
            anchorFirst(t_read_us, span);
            // Never earlier than the last sample handed out.
            clampToLast();
            return;
        }

        // Frequency: spread over the burst, so a long burst after a late read
        // weighs no more than the samples it holds.
        period_ += FREQ_GAIN * err / (float)n;
        const float lo = nominal_ * (1.0f - PERIOD_LIMIT);
        const float hi = nominal_ * (1.0f + PERIOD_LIMIT);
        if (period_ < lo) period_ = lo;
        if (period_ > hi) period_ = hi;

        // Phase: move the whole burst a fraction of the way.
        advance(PHASE_GAIN * err);
        clampToLast();
    }

    // Time of the next sample of the current burst.
    uint32_t next()
    {
        const uint32_t t = next_base_ + (uint32_t)(int32_t)next_frac_;
        last_ = t;
        have_last_ = true;
        advance(period_);
        if (pending_ > 0) pending_--;
        return t;
    }

private:
    float nominal_ = 260.4f;
    float period_ = 260.4f;
    bool primed_ = false;
    uint32_t pending_ = 0;

    // The next sample's time, kept as an integer base plus a small float part
    // so the float never has to hold a full 32-bit microsecond count.
    uint32_t next_base_ = 0;
    float next_frac_ = 0.0f;

    uint32_t last_ = 0;
    bool have_last_ = false;

    // Signed microseconds from `from` to `to`, wrap-safe.
    static float offsetFrom(uint32_t to, uint32_t from)
    {
        return (float)(int32_t)(from - to);
    }

    void advance(float us)
    {
        next_frac_ += us;
        const int32_t whole = (int32_t)next_frac_;
        next_base_ += (uint32_t)whole;
        next_frac_ -= (float)whole;
    }

    void anchorFirst(uint32_t t_read_us, float span)
    {
        next_base_ = t_read_us;
        next_frac_ = 0.0f;
        advance(-0.5f * period_ - span);
    }

    // Keep time moving forward: the next sample at least half a period after
    // the last one handed out.
    void clampToLast()
    {
        if (!have_last_) return;
        const float min_gap = 0.5f * period_;
        const float gap = offsetFrom(last_, next_base_) + next_frac_;
        if (gap < min_gap) advance(min_gap - gap);
    }
};
