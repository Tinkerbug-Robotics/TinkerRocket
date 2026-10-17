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
//     filter vz  <  barometer climb rate - 10 m/s, held for 1 s
//
// after burnout and before apogee latches, and the filter's velocity and
// pitch votes are withheld for the rest of the flight.  The apogee call then
// rests on the barometer and the GPS voter (both must agree: the quorum's
// floor is two concurring voters) and the Layer-2 barometer backstop.
//
// ONE-SIDED, on purpose.  Only a filter that reads BELOW the barometer is
// judged: that is the direction that advances the call (the velocity vote
// passes on vz < 0).  A filter that reads high only holds back its own
// velocity vote, which the quorum rides out, so withholding it gains nothing.
// And the barometer's own failures read low.  A sealed or taped static port
// reads fresh, in range and flat (GnssAscentGate's stuck case), so a correct
// filter reads far above it all coast; a two-sided guard latched 1 s after
// the filter came under 60 m/s, the barometer never passed and the GPS voter
// alone is short of the two-voter floor: no apogee, no drogue, no main.  The
// burnout bay-pressure snap-back (7/05 V2 F1: 15 m in 0.25 s while climbing
// at 46 m/s) pulls the rate down too, so it is not judged either.
//
//   * 10 m/s is far above what the barometer's rate filter does on its own.
//     In a model of updateAltKF (Qv 5, R 1, samples at 487 Hz) its rate lags
//     a 12 m/s^2 coast deceleration by 2.7 m/s, reading high: toward a latch,
//     leaving 7.3 m/s.  A slow static port adds its own lag on top (its time
//     constant times the deceleration); one slower than about 0.6 s is this
//     guard's false latch, and the case Grid B must look for.
//   * The numbers are tuned on Grid B, which must show zero false latches:
//     a false latch removes two of the four voters.
//
// WHAT IT NEVER DOES.
//   * Never unlatches in flight.  A filter that has been 10 m/s wrong for a
//     second has shown it can be; letting it back in would let a fault that
//     comes and goes vote in its good half.  Only reset() clears it: at
//     launch (enterInflight) and on the sim edge (resetFlightStateForSim), so
//     a latch never carries into the next flight of the same boot.
//   * Never acts before burnout or after apogee.  Under thrust the barometer
//     lags the climb and the motor's flow pulls on the bay pressure, and the
//     apogee vote does not run before burnout anyway.  After apogee the call
//     is made (kinematicChecks reads ekf_healthy only before it), and the
//     ejection charge, the deployment and the canopy swing all move the bay
//     pressure: a latch there would only be a false alarm.
//   * Never acts while mach-locked, with a barometer that cannot vouch (stale,
//     outside 25-125 kPa, or judged stuck: the caller passes
//     GnssAscentGate::baroFusable()) or on a non-finite rate.  Then the
//     barometer is not a witness, and taking the filter's votes on its word
//     would remove them exactly when they are most of the voters left.  The
//     transonic lockout is driven by this same filter's speed, so a filter
//     reading over 260 m/s silences the barometer voter and this guard alike;
//     the guard does not cover that.
//   * Never acts on a flat barometer: |rate| under 1 m/s, the stuck bar of
//     GnssAscentGate and MainDeployGate's Layer 1.  A sealed port reads flat
//     all flight, and GnssAscentGate's stuck verdict needs 2 s and a GNSS fix
//     to take it out of baro_healthy.  One-sided alone, a correct filter
//     falling through -10 m/s after apogee would still latch against it
//     while a slow call (pitch late, GPS not admitted) was pending.  A
//     working barometer is that flat only for about 0.2 s around apogee,
//     too late for a call to be early.
//   * Never judges while the barometer's |rate| is 60 m/s or more.  The bound
//     is on the barometer, not the filter, so the filter under test cannot
//     switch its own check off (one reading -65 m/s while the rocket climbs
//     is judged).  At speed the barometer's rate filter lags harder and the
//     bay pressure moves with dynamic pressure; the votes that matter are cast
//     in the late coast.
//   * Never touches anything but the two votes.  Control, GnssAscentGate,
//     telemetry and the landing checks keep the filter.
//   * State is RAM only: an in-flight reboot starts it unlatched.
//
// Pure and host-testable (tests_cpp/test_raw_vote_guard.cpp).  Shaped like
// GnssAscentGate: one step() per kinematics pass, after GnssAscentGate's, then
// the caller passes `ekf_healthy && !state.latched` as kinematicChecks()'s
// ekf_healthy only.

#include <cmath>
#include <cstdint>

namespace RawVoteGuard {

struct Config {
    float    diff_mps      = 10.0f;   // filter vz more than this below the barometer
    uint32_t hold_ms       = 1000;    // held this long, continuously, latches
    float    rate_max_mps  = 60.0f;   // judged only below this barometer |rate|
    float    rate_flat_mps = 1.0f;    // and not below this one (flat: a sealed port)
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
    bool     apogee           = false;  // kinematics.apogee_flag || pyro_apogee_detected
    bool     baro_healthy     = false;  // GnssAscentGate::baroFusable(gnss_admit_state, baro_healthy)
    bool     mach_locked      = false;  // mach_locked_out (transonic lockout)
    bool     filter_healthy   = false;  // ekf_healthy (initialized && isHealthy())
    float    filter_vz_up_mps = 0.0f;   // imu_vel[2]: ENU up, the velocity voter's
    float    baro_rate_mps    = 0.0f;   // kinematics.d_alt_est_, up-positive
};

// Boot, launch and the sim edge: unlatched, no run in progress.
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
    // votes away.  The speed bounds read the barometer's rate only, so the
    // filter cannot step out of its own check by reading fast.
    const float baro_speed = fabsf(in.baro_rate_mps);
    const bool  judging = in.after_burnout && !in.apogee && in.baro_healthy &&
                          !in.mach_locked && in.filter_healthy &&
                          std::isfinite(in.filter_vz_up_mps) &&
                          std::isfinite(in.baro_rate_mps) &&
                          baro_speed >= cfg.rate_flat_mps &&
                          baro_speed < cfg.rate_max_mps;
    // One-sided: only a filter reading below the barometer is judged.
    const bool disagree = judging &&
                          in.filter_vz_up_mps < in.baro_rate_mps - cfg.diff_mps;

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
