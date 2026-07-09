#pragma once

#include <cstdint>

// #468: decide whether a far-backstepped frame timestamp means the FC
// rebooted or a stale I2S TX descriptor got replayed.
//
// The old heuristic reset every dedup baseline the moment one frame's
// time_us fell more than REBOOT_BACKSTEP_US behind the global high-water
// mark. On the bench that fired at 21.5 Hz for seconds at a time: an FC
// sender underrun leaves one descriptor of old frames in the TX DMA ring,
// and the same 10+ s-old frame re-arrives once per ring revolution
// (46.4 ms at 44100 Hz) interleaved with perfectly fresh traffic. Each
// false "reboot" wiped the baselines — briefly disabling replay
// protection for every frame type — and flooded the log.
//
// The physical discriminator: after a REAL reboot (or a uint32 time_us
// wrap) every subsequent frame is backstepped, because the FC's clock
// restarted; a replay burst is always followed within a couple of
// milliseconds by fresh frames. So a reboot verdict requires the
// backstep to PERSIST for confirm_ms with no fresh frame in between —
// default 100 ms, a bit over two TX DMA ring revolutions, so a replayed
// descriptor recurring every revolution still cannot confirm as long as
// live frames keep flowing around it.
//
// Pure header (no ESP-IDF deps) so the host suite can exercise it; the
// caller owns time (millis()) and the actual baseline reset.
class DedupRebootPolicy
{
public:
    enum class Action : uint8_t
    {
        PROCEED,         // not backstepped — frame flows to the normal dedup checks
        DROP_REPLAY,     // backstepped but unconfirmed — drop it, keep all baselines
        RESET_BASELINE,  // backstep persisted — genuine reboot, reset dedup state
    };

    explicit DedupRebootPolicy(uint32_t confirm_ms = 100)
        : confirm_ms_(confirm_ms)
    {
    }

    Action onFrame(bool far_backstepped, uint32_t now_ms)
    {
        if (!far_backstepped)
        {
            // Fresh traffic still flowing: any pending suspicion was a replay.
            suspect_active_ = false;
            return Action::PROCEED;
        }
        if (!suspect_active_)
        {
            suspect_active_ = true;
            suspect_since_ms_ = now_ms;
            return Action::DROP_REPLAY;
        }
        if ((uint32_t)(now_ms - suspect_since_ms_) >= confirm_ms_)
        {
            suspect_active_ = false;
            return Action::RESET_BASELINE;
        }
        return Action::DROP_REPLAY;
    }

    bool suspectActive() const { return suspect_active_; }

private:
    uint32_t confirm_ms_;
    uint32_t suspect_since_ms_ = 0;
    bool suspect_active_ = false;
};
