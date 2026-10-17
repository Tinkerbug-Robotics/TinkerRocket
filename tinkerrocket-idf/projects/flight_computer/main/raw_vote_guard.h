#pragma once

// Raw vote guard: may the raw-GNSS filter still cast its two apogee votes?
// Mosaic-only build (V8 + mosaic-G5, TR_RAW_NAV_MOSAIC); the plan's critic
// item F5, "fresh but wrong filter".
//
// WHY IT EXISTS.  On the mosaic-only build the raw-GNSS filter is the ONLY
// navigation filter.  Its vertical velocity and its pitch are two of the four
// apogee voters (TR_KinematicChecks), and the GPS voter reads the same
// receiver's PVT, so one receiver stands behind three of the four.  With
// GpsInsEKF gone there is no second filter to cross-check it against, and
// "healthy" (RawNavAdapter::isHealthy) only asks that the filter runs and its
// state is finite: a filter dragged off by a coherent receiver fault is fresh,
// finite and wrong, and still votes.  Near apogee the barometer's climb rate
// (the altitude filter's rate, kinematics.d_alt_est_) is the one independent
// witness of the vertical velocity left, so the guard compares the two:
//
//     | filter vz  -  barometer climb rate |  >  10 m/s, held for 1 s
//
// after burnout latches, and the filter's velocity and pitch votes are
// withheld for the rest of the flight.  The apogee call then rests on the
// barometer and the GPS voter (both must agree: the quorum's floor is two
// concurring voters) and the Layer-2 barometer backstop.
//
//   * 10 m/s is far above what the barometer's rate filter does on its own.
//     In a model of updateAltKF (Qv 5, R 1, samples at 487 Hz) its rate lags
//     a 12 m/s^2 coast deceleration by 2.7 m/s, and a burnout bay-pressure
//     snap-back like 7/05 V2 F1's (15 m in 0.25 s while climbing at 46 m/s)
//     holds it more than 10 m/s off for about 0.5 s, inside the 1 s hold.
//   * The numbers are tuned on Grid B, which must show zero false latches:
//     a false latch removes two of the four voters.
//
// WHAT IT NEVER DOES.
//   * Never unlatches in flight.  A filter that has been 10 m/s wrong for a
//     second has shown it can be; letting it back in would let a fault that
//     comes and goes vote in its good half.  Only reset() clears it: the sim
//     edge (resetFlightStateForSim), the sim's equivalent of a reboot.
//   * Never acts before burnout.  Under thrust the barometer lags the climb
//     and the motor's flow pulls on the bay pressure; the apogee vote does
//     not run before burnout anyway.
//   * Never acts while mach-locked, with an unhealthy barometer (stale or
//     outside 25-125 kPa) or on a non-finite climb rate.  Then the barometer
//     is not a witness, and taking the filter's votes on its word would
//     remove them exactly when they are most of the voters left.
//   * Never acts while the filter is unhealthy: the vote already excludes
//     both of its voters then.
//   * Never judges at a filter |vz| of 60 m/s or more.  The barometer's rate
//     filter lags the deceleration and the bay pressure moves with dynamic
//     pressure at speed; the votes that matter are cast in the late coast.
//   * Never touches anything but the two votes.  Control, GnssAscentGate,
//     telemetry and the landing checks keep the filter.
//   * State is RAM only: an in-flight reboot starts it unlatched.
//
// Pure and host-testable (tests_cpp/test_raw_vote_guard.cpp).  Shaped like
// GnssAscentGate: one step() per kinematics pass, then the caller passes
// `ekf_healthy && !state.latched` as kinematicChecks()'s ekf_healthy only.

#include <cmath>
#include <cstdint>

namespace RawVoteGuard {

struct Config {
    float    diff_mps   = 10.0f;   // |filter vz - barometer rate| above this
    uint32_t hold_ms    = 1000;    // held this long, continuously, latches
    float    vz_max_mps = 60.0f;   // judged only below this filter |vz|
};

struct State {
    bool     latched     = false;  // votes withheld for the rest of the flight
    bool     disagreeing = false;  // a disagreement run is in progress
    uint32_t since_ms    = 0;      // when it began (valid while disagreeing)
};

// Each maps onto one signal in main.cpp's kinematics block.  The defaults
// cannot judge, so a field left unset never latches the guard.
struct Inputs {
    uint32_t now_ms           = 0;      // now_ms (millis domain)
    bool     after_burnout    = false;  // burnout_detected
    bool     baro_healthy     = false;  // baro_healthy (fresh, 25-125 kPa)
    bool     mach_locked      = false;  // mach_locked_out (transonic lockout)
    bool     filter_healthy   = false;  // ekf_healthy (initialized && isHealthy())
    float    filter_vz_up_mps = 0.0f;   // imu_vel[2]: ENU up, the velocity voter's
    float    baro_rate_mps    = 0.0f;   // kinematics.d_alt_est_, up-positive
};

// Boot and the sim edge: unlatched, no run in progress.
inline void reset(State& st)
{
    st = State{};
}

// Once per kinematics pass.  Returns whether the filter's votes are withheld,
// i.e. st.latched, not the latching edge: a caller that ANDs the return into
// the vote can never let the filter back in.
inline bool step(State& st, const Inputs& in, const Config& cfg = Config{})
{
    if (st.latched)
    {
        return true;
    }

    // baro_healthy judges the pressure sample, not the rate filter behind
    // d_alt_est_: a non-finite rate is a dead witness, and must not take the
    // votes away.  A non-finite filter vz fails the speed bound (and the
    // filter is unhealthy then anyway).
    const bool judging = in.after_burnout && in.baro_healthy && !in.mach_locked &&
                         in.filter_healthy &&
                         std::isfinite(in.baro_rate_mps) &&
                         fabsf(in.filter_vz_up_mps) < cfg.vz_max_mps;
    const bool disagree = judging &&
                          fabsf(in.filter_vz_up_mps - in.baro_rate_mps) > cfg.diff_mps;

    // A pass that cannot judge breaks the run too: the hold is a second of
    // disagreement the barometer witnessed, not a second with some in it.
    if (!disagree)
    {
        st.disagreeing = false;
        return false;
    }
    if (!st.disagreeing)
    {
        st.disagreeing = true;
        st.since_ms    = in.now_ms;
    }
    // Unsigned difference: the ~49.7-day millis() rollover is a non-event.
    if ((uint32_t)(in.now_ms - st.since_ms) >= cfg.hold_ms)
    {
        st.latched = true;
    }
    return st.latched;
}

}  // namespace RawVoteGuard
