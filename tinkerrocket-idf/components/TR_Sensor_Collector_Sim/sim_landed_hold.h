#pragma once
// #971: when the SIM_LANDED hold may end.
//
// The rule used to be a bare `elapsed_ms >= 9000`, chosen from a code comment
// that budgeted "5 consecutive 1-second checks (5s) plus a 2-second
// state-machine debounce = 7s minimum" and assumed 2 s of margin on top.
// That budget was wrong in a way only a sim can expose, and the error was
// invisible because nothing compared the two numbers.  Kept here as a pure
// rule so a test pins it instead.

#include <cstdint>

namespace sim_landed
{

// How long alt_landed_flag actually takes to latch after a sim touchdown.
// Measured on flight_20260827_122854: 7.0 s.  A sim never produces an impact
// (accel sits at a flat 1 g, never the 15 g LANDING_IMPACT_G spike), so the
// flag can only come from the SLOW vote, whose sub-flags run a leaky counter
// at 1 Hz — far slower than the 5 s the old comment assumed.
inline constexpr uint32_t MEASURED_FLAG_LATCH_MS = 7000;

// INFLIGHT -> LANDED needs the flag held this long.  Since #1137 item 8 the
// dwell lives in landing_transition_policy.h as kDwellMs and the comparison
// is `>=`, not the old strictly-greater `> 2000U`; the duration is unchanged,
// and this mirror is kept so the arithmetic below stays readable.  (It is a
// mirror, not the source: the policy header owns the number.)
inline constexpr uint32_t FC_LANDED_DEBOUNCE_MS = 2000;

// The old fixed hold: exactly MEASURED_FLAG_LATCH_MS + FC_LANDED_DEBOUNCE_MS.
// Against the strictly-greater comparison of the day that was a margin of
// ZERO, which is why no sim flight ever reached LANDED.
inline constexpr uint32_t LEGACY_HOLD_MS = 9000;

// Backstop only.  The hold normally ends the moment the FC reports LANDED;
// this caps the wait when landing detection never fires, so a broken detector
// cannot hang the sim.  Generous on purpose — the cost of waiting is bench
// time, and the cost of being too short is this bug.
//
// Raised 30 s -> 90 s (#574 bench, 2026-09-12).  30 s was generous for the
// HEALTHY path (slow vote latches ~7 s + 2 s debounce) but it was exactly the
// DEAD-IMU path's dwell: with the IMU stale from burnout the only route to
// LANDED is the baro-only backstop, whose alt_landed dwell is ~30 s.  Measured
// on a V9 with TR_SIM_DEAD_IMU: the flag latched 30.8 s after touchdown and
// LANDED followed 2 s later — i.e. the old backstop expired within a second of
// the latch, twice, and the sim gave up first.  #574's dead-IMU half was
// therefore unobservable in a sim by a margin of zero, the same failure shape
// the 9 s -> 30 s raise fixed for the healthy path.  Sized off the slowest
// documented path, not the fastest.
inline constexpr uint32_t HOLD_MAX_MS = 90000;

enum class Exit : unsigned char
{
    Hold,      // keep feeding stationary data; the FC has not landed yet
    FcLanded,  // the FC reached LANDED — the hold did its job
    GaveUp,    // backstop expired; landing detection did not fire
};

/// Pure decision.  `landed_state` is RocketState::LANDED, passed in so this
/// header stays free of the firmware type.
constexpr Exit decide(uint8_t fc_state, uint8_t landed_state, uint32_t elapsed_ms)
{
    if (fc_state == landed_state) return Exit::FcLanded;
    if (elapsed_ms >= HOLD_MAX_MS) return Exit::GaveUp;
    return Exit::Hold;
}

}  // namespace sim_landed
