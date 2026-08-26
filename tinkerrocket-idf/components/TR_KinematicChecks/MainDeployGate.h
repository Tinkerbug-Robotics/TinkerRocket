#pragma once

// #834 item 4: source-of-truth gate for the ALTITUDE_ON_DESCENT pyro trigger.
//
// THE DEFECT.  servicePyroChannels() fired the main on
//     pressure_alt_m <= val && pressure_alt_rate_mps < 0.0f
// where both values come from TR_KinematicChecks::updateAltKF() via
// kinematics.alt_est / d_alt_est_.  That KF has no notion of "I have no
// input": with new_measurement == false it takes the predict-only branch and
// sets alt_est = alt_est + d_alt_est_*dt with d_alt_est_ UNCHANGED, i.e. it
// free-runs at whatever vertical rate happened to be in the filter when the
// barometer stopped answering.  Two consequences, both bad:
//
//   * d_alt_est_ < 0 latches true forever once negative, so the rate half of
//     the trigger stops being a condition at all.
//   * alt_est walks down through ANY threshold on a schedule set by an
//     accident of timing.  Measured (1500 m apogee, 200 m main): a rate
//     frozen at the drogue-deployment transient (-40 m/s) instead of the
//     settled -20 m/s crosses the threshold with the vehicle really at
//     850 m AGL — a main deployed 650 m high, at 4x its design opening
//     load.  The filter's own covariance at that moment is P00 = 58267 m^2
//     (1-sigma = 241 m): it already "knows" it does not know, and nothing
//     was reading that.
//
// THE OTHER DEFECT, found while fixing the first, and the more likely one.
// A blocked or taped static port (wadding, a tape-over-the-port checklist
// miss) leaves the sensor answering perfectly: pressure stays FRESH and
// IN-RANGE, so the #257 baro_healthy predicate is TRUE forever, while
// alt_est converges on a constant and d_alt_est_ -> 0.  Then
// "alt <= val && rate < 0" can never be satisfied and the channel sits Idle
// until LANDED forecloses it.  That is a silent, guaranteed no-main-deploy,
// and a health test built only from timestamp+range cannot see it by
// construction.  Catching it needs a sensor that does not share the port.
//
// DESIGN.
//   Layer 0  Baro is authoritative while it is TRUSTED, and on that path the
//            predicate is bit-identical to the legacy one — a healthy flight
//            behaves exactly as it does today (locked down by an exhaustive
//            sweep test).  "Trusted" adds three things to baro_healthy:
//              - not stuck (see Layer 1),
//              - a re-acquire dwell after any unhealthy period, because a
//                free-run inflates P00 until K0 -> 1 and the filter adopts
//                the very first returning sample WHOLESALE.  One in-range
//                but corrupt sample would otherwise become the state and
//                fire the charge.  Sized from the filter: worst measured
//                re-convergence (|rate err| < 1 m/s and |alt err| < 2 m)
//                across a sweep of free-run durations x frozen-rate errors
//                is 453 ms, so the dwell is 600 ms.
//              - a post-reboot settle, because reboot recovery restores
//                pyro_apogee_time_ms and resumes INFLIGHT with the sensor
//                stack cold (#846/#104).
//   Layer 1  Stuck-baro cross-check.  Baro rate flat while GNSS Doppler says
//            we are descending => the port is blocked; demote to Layer 2.
//            The two thresholds are separated so the conjunction is close to
//            self-contradictory in steady state, and it is a LEAKY
//            ACCUMULATOR, not a latch: a false positive costs a segment of
//            the descent, never the whole flight, and a baro that starts
//            agreeing again is taken back.
//   Layer 2  GNSS backstop.  Pad-referenced AGL, Doppler-propagated to now so
//            fix latency does not enter the error budget.  The margin is
//            LATE-BIASED on purpose — fire at (agl <= val + margin), never
//            (agl + margin <= val).  A main released a little high costs
//            drift; a main released below the operator's number does not
//            inflate.  This bounds the true fire altitude to [val, val+2*margin]
//            rather than pushing it below val, which matters most at the
//            apps' shipped default of 100 m.
//
// NOT DONE, deliberately: GNSS never pre-empts a trusted barometer.  A
// GNSS-vs-baro disagreement demotes the PAIR (Layer 1); it never authorises
// acting on the weaker member.  Every fire path is inhibited by landing
// evidence.  There is no time-only "fire on a clock with no altitude" path:
// with both altitude sources gone, a clock-fired main at an unknown altitude
// is a range-safety problem, not a rescue.
//
// Pure and host-testable; the FC and the mini share it (both already REQUIRE
// TR_KinematicChecks).  Split into a mutating step() called once per tick and
// a const evaluate() called per channel, so the four-channel loop cannot
// advance a timer four times.

#include <math.h>
#include <stdint.h>

namespace MainDeployGate {

enum class Source : uint8_t {
    None = 0,   // do not fire
    Baro = 1,   // fired on the trusted barometric estimate (nominal)
    Gnss = 2,   // fired on the GNSS backstop (barometer not trusted)
};

struct Config {
    // Baro must be continuously healthy this long before it is trusted again.
    // 453 ms measured worst-case KF re-convergence + margin.
    uint32_t reacquire_ms    = 600;
    // Sensor-stack settle after a mid-flight reboot resumes INFLIGHT.
    uint32_t resume_dwell_ms = 2000;
    // Stuck-port cross-check.  |baro rate| below this AND GNSS descending
    // faster than this, sustained for stuck_dwell_ms, demotes the baro.
    float    stuck_baro_mps  = 1.0f;
    float    stuck_gnss_mps  = 1.0f;
    // Longest real post-apogee coincidence of these two conditions across the
    // committed example flights is 0.19 s; 2 s is a 10x margin.  The same
    // value drains the accumulator, so recovery costs one dwell.
    uint32_t stuck_dwell_ms  = 2000;
    // Late-biased GNSS margin (see Layer 2).  Covers receiver vertical error
    // plus pad-datum error once ref_pos_count has converged.
    float    gnss_margin_m   = 25.0f;
    // The GNSS path must see a genuine descent.  "< 0" would admit receiver
    // noise on a landed vehicle, which matters because impact_flag no longer
    // inhibits this path (see Inputs) and quiescent_flag can take ~30 s.
    float    gnss_min_descent_mps = 1.0f;
};

struct State {
    bool     seeded          = false;
    bool     baro_ok_prev    = false;
    uint32_t baro_ok_since_ms = 0;
    int32_t  stuck_ms        = 0;      // leaky accumulator
    bool     baro_stuck      = false;
    uint32_t resume_ms       = 0;
    bool     after_reboot    = false;
    uint32_t last_step_ms    = 0;
};

struct Inputs {
    uint32_t now_ms          = 0;
    bool     apogee          = false;  // pyro_apogee_detected
    // Landing evidence, split by the sensor it rests on.  impact_flag's only
    // altitude gate is BAROMETRIC (pressure_altitude < LANDING_IMPACT_ALT_M),
    // so a static port blocked on the pad pins barometric altitude near zero
    // for the whole flight and the drogue charge's own >15 g shock latches it
    // — on exactly the flight the GNSS backstop exists to save.  So it may
    // only inhibit while the barometer is trusted.  quiescent_flag (#824) is
    // baro-independent and always inhibits.
    bool     impact_flag     = false;
    bool     quiescent_flag  = false;
    // Barometric estimate (TR_KinematicChecks)
    bool     baro_healthy    = false;  // #257: fresh + in-range.  NOTE this is
                                       // NOT "the KF is updating" — see
                                       // MAX_CONSEC_BARO_REJECTS, which can
                                       // hold off updates ~20 ms on fresh,
                                       // in-range samples.
    float    baro_alt_m      = 0.0f;   // kinematics.alt_est
    float    baro_rate_mps   = 0.0f;   // kinematics.d_alt_est_
    // GNSS backstop
    bool     gnss_ok         = false;  // 3D + sats + fresh + converged datum
    float    gnss_agl_m      = 0.0f;   // (alt - ref_alt_m), Doppler-propagated
    float    gnss_vel_u_mps  = 0.0f;   // up-positive; descent is negative
};

// Call at enterInflight() and on the reboot-recovery restore.  after_reboot
// charges the full re-acquire dwell instead of crediting the pad phase.
inline void reset(State& st, uint32_t now_ms, bool after_reboot)
{
    st = State{};
    st.resume_ms    = now_ms;
    st.after_reboot = after_reboot;
}

// Advance timers and the stuck-port accumulator.  Once per tick, before
// evaluate().  Idempotent only across a single now_ms — do not call from
// inside the per-channel loop.
inline void step(State& st, const Inputs& in, const Config& cfg = Config{})
{
    if (!st.seeded)
    {
        // First observation of the flight.  On a normal launch the barometer
        // has been alive through the whole pad phase, so the dwell is already
        // satisfied; after a mid-flight reboot it has not, so charge it.
        st.baro_ok_since_ms = st.after_reboot ? in.now_ms
                                              : in.now_ms - cfg.reacquire_ms;
        st.baro_ok_prev = in.baro_healthy;
        st.last_step_ms = in.now_ms;
        st.seeded       = true;
        return;
    }

    const uint32_t dt_ms = (uint32_t)(in.now_ms - st.last_step_ms);
    st.last_step_ms = in.now_ms;

    // Re-acquire dwell: restart the clock on every unhealthy -> healthy edge.
    if (in.baro_healthy && !st.baro_ok_prev) st.baro_ok_since_ms = in.now_ms;
    st.baro_ok_prev = in.baro_healthy;

    // Stuck-port cross-check.  Only meaningful when both sensors are answering
    // and we are past apogee (on the pad both rates are legitimately ~0, and
    // under boost the baro is not flat).
    const bool disagree = in.apogee && in.baro_healthy && in.gnss_ok &&
                          fabsf(in.baro_rate_mps) < cfg.stuck_baro_mps &&
                          in.gnss_vel_u_mps < -cfg.stuck_gnss_mps;
    if (disagree)
    {
        st.stuck_ms += (int32_t)dt_ms;
        if (st.stuck_ms >= (int32_t)cfg.stuck_dwell_ms)
        {
            st.stuck_ms   = (int32_t)cfg.stuck_dwell_ms;
            st.baro_stuck = true;
        }
    }
    else
    {
        st.stuck_ms -= (int32_t)dt_ms;
        if (st.stuck_ms <= 0)
        {
            st.stuck_ms   = 0;
            st.baro_stuck = false;   // leaky, not latched
        }
    }
}

// True when the barometric estimate may authorise a deploy on its own.
inline bool baroTrusted(const State& st, const Inputs& in,
                        const Config& cfg = Config{})
{
    if (!in.baro_healthy || st.baro_stuck) return false;
    if ((uint32_t)(in.now_ms - st.baro_ok_since_ms) < cfg.reacquire_ms)
        return false;
    if (st.after_reboot &&
        (uint32_t)(in.now_ms - st.resume_ms) < cfg.resume_dwell_ms)
        return false;
    return true;
}

// Per-channel decision.  Const — call after step().
inline Source evaluate(const State& st, const Inputs& in, float threshold_m,
                       const Config& cfg = Config{})
{
    if (!in.apogee) return Source::None;

    const bool trusted = baroTrusted(st, in, cfg);

    // quiescent_flag is baro-independent and inhibits unconditionally.
    // impact_flag is only believable while the barometer that gates it is.
    if (in.quiescent_flag || (in.impact_flag && trusted)) return Source::None;

    if (trusted)
    {
        // Bit-identical to the legacy predicate.  Do not "improve" this line:
        // a healthy-baro flight must behave exactly as the shipped firmware.
        return (in.baro_alt_m <= threshold_m && in.baro_rate_mps < 0.0f)
                   ? Source::Baro : Source::None;
    }

    // Barometer not trusted — GNSS backstop.
    if (!in.gnss_ok) return Source::None;
    if (st.after_reboot &&
        (uint32_t)(in.now_ms - st.resume_ms) < cfg.resume_dwell_ms)
        return Source::None;
    if (in.gnss_vel_u_mps > -cfg.gnss_min_descent_mps)
        return Source::None;                          // must really be descending
    return (in.gnss_agl_m <= threshold_m + cfg.gnss_margin_m)
               ? Source::Gnss : Source::None;
}

}  // namespace MainDeployGate
