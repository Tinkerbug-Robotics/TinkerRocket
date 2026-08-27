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

// flight_computer/main.cpp: INFLIGHT -> LANDED needs the flag held for
// STRICTLY MORE than this (`now_ms - landed_candidate_start_millis > 2000U`).
inline constexpr uint32_t FC_LANDED_DEBOUNCE_MS = 2000;

// The old fixed hold: exactly MEASURED_FLAG_LATCH_MS + FC_LANDED_DEBOUNCE_MS.
// Against a strictly-greater comparison that is a margin of ZERO, which is why
// no sim flight ever reached LANDED.
inline constexpr uint32_t LEGACY_HOLD_MS = 9000;

// Backstop only.  The hold normally ends the moment the FC reports LANDED;
// this caps the wait when landing detection never fires, so a broken detector
// cannot hang the sim.  Generous on purpose — the cost of waiting is bench
// time, and the cost of being too short is this bug.
inline constexpr uint32_t HOLD_MAX_MS = 30000;

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
