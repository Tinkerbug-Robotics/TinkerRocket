#pragma once
// ==========================================================================
// #732 item 3b — the piezo square-wave state machine, as a pure decision
// table so the DC-latch invariant is a test rather than a comment.
//
// LS1 (MLT-8530) is a PASSIVE magnetic transducer: it makes no sound on its
// own, firmware bit-bangs the carrier. The hardware review's I51 warns that
// leaving the pin high puts DC through the ~16 ohm coil — ~200 mA / 0.7 W —
// which cooks the transducer and loads the sensor rail.
//
// The failure this header exists to make impossible: piezoStop() runs on the
// main task while piezoToggleCb() is already past its `active` check on the
// esp_timer task (a different core on the P4). The stopper drives the pin
// low and cancels the timer; the callback then completes its toggle and
// drives the pin HIGH. Nothing is left running to toggle it back, so the
// coil sits energised until the next beep or a reboot — not for a starvation
// interval, but indefinitely.
//
// Two things fix it, and both are needed:
//   1. main.cpp serialises every transition below under a spinlock, so a
//      tick can no longer straddle a stop.
//   2. This table has NO path that returns DriveHigh while `active` is
//      false, so even a tick that arrives after a stop is inert.
// The tests exercise (2) directly by interleaving the calls by hand.
// ==========================================================================
#include <stdint.h>

namespace PiezoWavePolicy
{

// What the caller must do to PIEZO_PIN after this transition.
enum class PinAction : uint8_t
{
    None = 0,   // leave the pin exactly as it is
    DriveLow,
    DriveHigh,
};

struct State
{
    bool    active   = false;
    bool    pin_high = false;
    int64_t end_us   = 0;   // full 64-bit; see #382 for why it is not 32
};

// One periodic-timer fire.
//
// `stop_timer` is an OUT parameter rather than part of the return value
// because esp_timer_stop() takes a lock and so must be called by the caller
// OUTSIDE the critical section this runs in.
inline PinAction onTick(State &s, int64_t now_us, bool &stop_timer)
{
    stop_timer = false;

    if (!s.active)
    {
        // Lost the race with onStop() (or a spurious late fire). The stopper
        // already drove the pin low and owns the last write — touching it
        // here is what latched the coil on.
        return PinAction::None;
    }

    if (now_us >= s.end_us)
    {
        s.active   = false;
        s.pin_high = false;
        stop_timer = true;
        return PinAction::DriveLow;
    }

    s.pin_high = !s.pin_high;
    return s.pin_high ? PinAction::DriveHigh : PinAction::DriveLow;
}

// Stop the wave. Always ends low: this is the only write that matters for
// the coil, so it is unconditional rather than guarded on `active` — a stop
// against an already-stopped wave must still assert the safe level.
inline PinAction onStop(State &s)
{
    s.active   = false;
    s.pin_high = false;
    return PinAction::DriveLow;
}

// Begin a new wave. Starts from a known-low pin so the first onTick() edge
// is always low->high and the duty cycle is symmetric.
inline PinAction onStart(State &s, int64_t now_us, uint32_t duration_ms)
{
    s.end_us   = now_us + (int64_t)duration_ms * 1000LL;
    s.pin_high = false;
    s.active   = true;
    return PinAction::DriveLow;
}

}   // namespace PiezoWavePolicy
