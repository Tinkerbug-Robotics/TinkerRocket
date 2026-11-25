#pragma once

// The INFLIGHT safety timeout: how long a flight may stay INFLIGHT before the
// FC safes every channel and declares LANDED on its own (landing detection
// failed, stuck at altitude, a diverged filter).
//
// The base is 600 s. #1706 (V10 pre-flight review, 2026-11-18) let it run to
// the 1800 s ceiling while a deployment channel is still owed after apogee, so
// that a main due at its altitude under a slow drogue from 10 km or more can
// still fire (10.5 km under a 15 m/s drogue reaches 300 m at T+592 s).
//
// #1753: that extension did not hold. "Owed" counted only channels in Idle.
// On the pass where a late main's trigger is met, servicePyroChannels() moves
// it Idle -> ArmSettle, and the main cannot reach Firing until the 10 ms
// settle has passed. The timeout at the end of the same pass then saw nothing
// owed, fell back to 600 s, safed the main and declared LANDED. A main due
// after T+600 s never fired, the exact case #1706 was written for.
//
// Two rules fix it:
// - A channel is owed until its pulse is done: enabled and Idle, or in
//   ArmSettle or Firing. ArmSettle and Firing count whether or not the channel
//   is still enabled, so the base never cuts a pulse that has begun. The
//   ceiling is absolute: a pulse begun in its last 210 ms is still cut there.
// - After the last pulse of the flight ends, the flight stays open for
//   kAfterFireMs, never less than the base and never more than the ceiling.
//   A main that fires past T+600 s then gets its descent before the timeout,
//   rather than LANDED (log end, snapshot clear, power-hold release) on the
//   pass after the pulse while it is still under the canopy. Before #1706 the
//   timeout also ended such a flight in the air.
//
// A normal flight is unchanged: with every channel done well before T+300 s,
// the limit stays at the 600 s base, and landing detection ends the flight
// long before either bound.
//
// Pure so the table is host-testable (tests_cpp/test_flight_timeout_policy.cpp),
// in the style of test_mode_gate_policy.h.
#include <cstdint>

namespace FlightTimeoutPolicy {

inline constexpr uint32_t kBaseMs      = 600000U;   // 10 min
// = RecoveryArmGate::Config::max_flight_ms and the OC's
// InflightRefusalPolicy::kMaxFlightTimeMs / kSnapshotServeTtlMs; main.cpp
// asserts the first.
inline constexpr uint32_t kCeilingMs   = 1800000U;  // 30 min
// After the last pulse: a main at 1500 m under a 5 m/s canopy lands in 300 s.
inline constexpr uint32_t kAfterFireMs = 300000U;   // 5 min

static_assert(kBaseMs < kCeilingMs, "the extension must extend");

// A channel as the timeout sees it.
enum class Channel : uint8_t {
    Idle,     // not yet triggered
    Pulsing,  // ArmSettle or Firing: the pulse has begun and must finish
    Done,     // fired; latched for the flight
};

// Is this channel a charge still to come?
inline constexpr bool owed(bool enabled, Channel state)
{
    return state == Channel::Pulsing || (enabled && state == Channel::Idle);
}

struct In {
    bool     apogee_detected;       // pyro_apogee_detected
    bool     channel_owed;          // owed() for any of the four channels
    bool     fired_this_flight;     // a pulse ended in this flight (or a restore found one Done)
    uint32_t last_fire_elapsed_ms;  // when the last pulse ended, ms after launch
};

// The bound on time since launch for this pass.
inline constexpr uint32_t limitMs(const In& in)
{
    if (!in.apogee_detected) return kBaseMs;
    if (in.channel_owed) return kCeilingMs;
    if (in.fired_this_flight) {
        // 64-bit so a late fire cannot wrap past the ceiling.
        const uint64_t after = static_cast<uint64_t>(in.last_fire_elapsed_ms) + kAfterFireMs;
        if (after < kBaseMs) return kBaseMs;
        if (after > kCeilingMs) return kCeilingMs;
        return static_cast<uint32_t>(after);
    }
    return kBaseMs;
}

// Should the timeout safe every channel and declare LANDED on this pass?
inline constexpr bool expired(uint32_t elapsed_ms, const In& in)
{
    return elapsed_ms > limitMs(in);
}

} // namespace FlightTimeoutPolicy
