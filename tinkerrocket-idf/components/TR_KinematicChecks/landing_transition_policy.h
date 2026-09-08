#pragma once

// #1137 item 8 — the INFLIGHT -> LANDED debounce, extracted so it can be
// tested on the host.
//
// The rule this replaces read:
//
//     const bool landing_conditions =
//         kinematics.alt_landed_flag && (fabsf(roll_rate_dps) < 30.0f);
//     if (landing_conditions) {
//         if (!landed_candidate_active) { start = now; active = true; }
//         if (now - start > 2000U) rocket_state = LANDED;
//     } else if (active && (now - start > 2500U)) {
//         active = false;
//     }
//
// and had two defects.
//
// 1. It was never a dwell.  The 2 s test is evaluated against the candidate
//    START, and the reset arm is *also* measured from the start, so every
//    false sample between t0 and t0+2.5 s is discarded.  One qualifying
//    sample at t0 plus one qualifying sample any time after t0+2 s is a
//    complete landing; the 2500 ms arm can only ever fire after the 2000 ms
//    one has already had its chance.  The comment claimed noise immunity, but
//    what it bought was noise *blindness*.  Here the dwell is real (the
//    condition must hold for kDwellMs) and the reset is what the comment
//    always said it was: a run of false samples lasting kFalseResetMs.
//
// 2. The gyro veto trusts a number that can be frozen.  roll_rate_dps is only
//    refreshed under `if (have_ism6_si)`, and have_ism6_si latches on the
//    first successful read and never clears -- so once the IMU stops
//    answering, roll_rate_dps holds its last value forever.  Frozen above
//    30 dps it vetoes LANDED for the rest of the flight, and LANDED is what
//    arms post_flight_lockout (#317), safes the pyro rail and closes the log.
//    A rocket lying in a field with a wedged IMU therefore keeps live squibs
//    on the ground until the 10-minute MAX_FLIGHT_TIME_MS backstop in the
//    flight loop safes them for it.  That backstop is what makes this a
//    bounded hazard rather than an unbounded one -- it is not a reason to
//    spend ten minutes reaching it, since it is also the last line of defence
//    for genuine landing-detection failure.
//
//    So the veto now needs live evidence to fire: a stale IMU cannot veto at
//    all, and even a fresh one gives up kGyroVetoBoundMs after the kinematics
//    vote first said "landed".  Two minutes is far longer than any real
//    canopy-drag spin-down and comfortably inside the backstop, so a stuck
//    gyro costs ~2 minutes on the ground instead of ~10.  The companion
//    change (#1137 item 7) closes the other half: a stale IMU no longer casts
//    the gyro-quiet vote that sets alt_landed_flag in the first place.

#include <math.h>
#include <stdint.h>

namespace landing_transition
{

// The condition must hold this long before the state actually changes.
inline constexpr uint32_t kDwellMs = 2000;

// ...and must FAIL continuously this long to restart that dwell.  Shorter
// blips are ignored, which is what the original comment intended.
inline constexpr uint32_t kFalseResetMs = 500;

// How long a fresh gyro may hold LANDED off after alt_landed_flag latched.
inline constexpr uint32_t kGyroVetoBoundMs = 120000;

// Roll rate below which the airframe counts as no longer spinning.
inline constexpr float kGyroQuietDps = 30.0f;

struct State
{
    uint32_t vote_since_ms      = 0;  // when alt_landed_flag first latched
    uint32_t candidate_start_ms = 0;  // when the current dwell began
    uint32_t false_since_ms     = 0;  // when the current false run began
    bool     vote_seen          = false;
    bool     candidate_active   = false;
    bool     false_pending      = false;
};

enum class Action : uint8_t
{
    Hold,
    Land,
};

// `imu_fresh` is the same signal the kinematics vote is given as
// imu_healthy: the IMU answered within the staleness timeout.  It is NOT
// have_ism6_si, which latches forever.
inline Action step(State &s,
                   bool     alt_landed_flag,
                   bool     imu_fresh,
                   float    roll_rate_dps,
                   uint32_t now_ms)
{
    if (!alt_landed_flag)
    {
        // alt_landed_flag latches once true, so in practice this only runs
        // before the vote -- but a reset must leave no half-built dwell.
        s = State{};
        return Action::Hold;
    }

    if (!s.vote_seen)
    {
        s.vote_seen      = true;
        s.vote_since_ms  = now_ms;
    }

    const bool veto_expired =
        (uint32_t)(now_ms - s.vote_since_ms) >= kGyroVetoBoundMs;

    // A veto needs live evidence and a deadline.  Without either, the only
    // remaining question is whether the kinematics vote said landed -- and it
    // did, or we would have returned above.
    const bool gyro_vetoes = imu_fresh && !veto_expired &&
                             (fabsf(roll_rate_dps) >= kGyroQuietDps);

    if (!gyro_vetoes)
    {
        s.false_pending = false;
        if (!s.candidate_active)
        {
            s.candidate_start_ms = now_ms;
            s.candidate_active   = true;
        }
        if ((uint32_t)(now_ms - s.candidate_start_ms) >= kDwellMs)
        {
            return Action::Land;
        }
    }
    else if (s.candidate_active)
    {
        if (!s.false_pending)
        {
            s.false_pending  = true;
            s.false_since_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - s.false_since_ms) >= kFalseResetMs)
        {
            s.candidate_active = false;
            s.false_pending    = false;
        }
    }

    return Action::Hold;
}

}  // namespace landing_transition
