#pragma once
#include <cstdint>

// Single source of truth for the recovery-DEPLOYMENT detector: the moment the
// ejection charge fires and the recovery device (drogue on a dual-deploy, main
// on a single) comes out.  Pure, dependency-free logic — the host unit test
// (tests_cpp/test_deployment_detector.cpp) exercises the *real* firmware code
// rather than a hand-copied mirror, same contract as BurnoutDetector.h.
//
// This OBSERVES deployment; it never commands it.  Pyro firing stays on the
// apogee voter in TR_KinematicChecks — a detector that fired the charges it is
// trying to detect would be a circular trigger.  Current consumer is the
// dynamic logging rate (IMU_RATE_DYNAMIC): log at 3840 Hz from the pad through
// boost and coast, step down to 960 Hz once the airframe is under canopy and
// nothing fast is left to capture.  The latch is exported on the wire as
// NSF2_DEPLOYED so later features (recovery-phase telemetry cadence, landing
// prediction hand-off, post-flight event marking) can share one definition.
//
// Two independent paths latch it, because they fail in different places:
//
//   FAST  (shock + baro step, coincident)
//         The charge is a near-simultaneous pair: a structural shock on the
//         accelerometer and an abrupt step in indicated altitude as ejection
//         gas vents past the static port and the airframe separates.  Fires
//         within milliseconds, which is what the logging step-down wants.
//         Requires BOTH — either alone is too easy to fake.  Baro noise is
//         ~0.1-0.3 m RMS at the BMP585's 500 Hz, and 100 m/s of real motion
//         moves indicated altitude only ~0.2 m between samples, so a
//         several-metre step between consecutive samples is not something
//         flight can produce.
//
//   SLOW  (descent-rate collapse)
//         Descent rate having been ballistic and then holding well below that
//         for a sustained window is the unambiguous signature of a canopy,
//         and it survives a charge whose shock or pressure signature the FC
//         missed.  Costs ~1-2 s of canopy inflation before it fires.
//
// Known limits, deliberately not papered over:
//   * On a single-deploy flight that goes straight to main at apogee, descent
//     may never reach the ballistic threshold, so the SLOW path never arms and
//     detection rests on the FAST path.
//   * On a ballistic (no-deploy) flight the SLOW path fires at ground impact,
//     where descent rate genuinely does collapse.  Harmless for the logging
//     step-down; consumers needing a strict "recovery deployed" reading should
//     stop stepping the detector at LANDED, as the flight loop does.

namespace tr {

// Which path latched, as a bitmask — both may be set if they fire on the same
// step.  Logged and printed so a post-flight read says WHY, not just when.
inline constexpr uint8_t kDeployReasonShockBaro       = (1u << 0);
inline constexpr uint8_t kDeployReasonDescentCollapse = (1u << 1);

// Tunables.  Owned by config::DEPLOY_* on the FC; passed in rather than baked
// as constants so the host test can sweep them — a suite pinned to one
// parameter set cannot tell a threshold-dependent invariant from a constant.
struct DeploymentConfig
{
    // FAST path
    float    shock_ms2;         // |specific force| at or above this = shock
    uint16_t shock_count;       // consecutive loop iterations required
    float    baro_step_m;       // |Δ indicated altitude| between consecutive
                                // baro samples that flight cannot produce
    uint32_t coincidence_ms;    // shock and baro step must both be this recent

    // SLOW path
    float    ballistic_mps;     // descent rate that arms the collapse test
    float    canopy_mps;        // descent rate that counts as "under canopy"
    uint16_t canopy_count;      // consecutive loop iterations required

    // Both paths
    uint32_t launch_lockout_ms; // no detection this soon after launch
};

// Caller-owned state; persists across calls for one flight.  Reset it for a
// new flight (deploymentReset) — the latch is per-flight, like burnout.
struct DeploymentState
{
    bool     detected    = false;
    uint32_t detected_ms = 0;    // t_since_launch_ms at the latch
    uint8_t  reason      = 0;    // kDeployReason* bitmask

    // FAST path
    uint16_t shock_count  = 0;
    bool     shock_seen   = false;
    uint32_t shock_ms     = 0;
    bool     baro_step_seen = false;
    uint32_t baro_step_ms = 0;

    // Baro tracking runs even while locked out, so the first armed sample
    // compares against a fresh altitude instead of a stale one.
    bool     have_palt   = false;
    float    last_palt_m = 0.0f;

    // SLOW path
    bool     ballistic    = false;
    uint16_t canopy_count = 0;
};

inline void deploymentReset(DeploymentState& st) { st = DeploymentState{}; }

// Advance the detector by one flight-loop iteration.
//
//   st, cfg           : caller-owned state and tunables.
//   t_since_launch_ms : milliseconds since launch was latched.
//   acc_mag_ms2       : |specific force| (m/s^2) — the flight loop's
//                       accel_norm, which auto-switches to the high-g channel
//                       near low-g saturation, so an ejection shock is
//                       measured rather than clipped.
//   palt_m, new_baro  : RAW pressure altitude and whether this iteration
//                       carries a fresh baro sample.  Raw on purpose: the
//                       rate gate inside TR_KinematicChecks exists precisely
//                       to swallow ejection spikes, which is the signal here.
//   palt_rate_mps     : filtered altitude rate (negative = descending).
//   burnout_detected  : motor cutoff latched.  Gates both paths — under thrust
//                       the airframe is already shaking and climbing, and
//                       nothing has deployed.
//
// Counts are in flight-loop iterations (~1 kHz), not IMU samples, matching
// LANDING_IMPACT_COUNT in TR_KinematicChecks — the loop calls this once per
// pass no matter what ODR the IMU is running.
//
// Returns true exactly once, on the rising edge that latches deployment, so
// the caller can stamp the time and act.  A no-op returning false thereafter.
inline bool deploymentDetectStep(DeploymentState& st, const DeploymentConfig& cfg,
                                 uint32_t t_since_launch_ms,
                                 float acc_mag_ms2,
                                 float palt_m, bool new_baro,
                                 float palt_rate_mps,
                                 bool burnout_detected)
{
    if (st.detected) return false;

    // Baro history tracks unconditionally; only the verdict is gated.
    float baro_step = 0.0f;
    bool  have_step = false;
    if (new_baro)
    {
        if (st.have_palt)
        {
            baro_step = palt_m - st.last_palt_m;
            have_step = true;
        }
        st.last_palt_m = palt_m;
        st.have_palt   = true;
    }

    if (!burnout_detected || t_since_launch_ms <= cfg.launch_lockout_ms)
    {
        // Under thrust / just off the pad: hold the transient counters at zero
        // so boost vibration cannot carry a partial count into the live window.
        st.shock_count = 0;
        return false;
    }

    // --- FAST path feature 1: structural shock ---
    if (acc_mag_ms2 >= cfg.shock_ms2)
    {
        if (++st.shock_count >= cfg.shock_count)
        {
            st.shock_seen = true;
            st.shock_ms   = t_since_launch_ms;
        }
    }
    else
    {
        st.shock_count = 0;
    }

    // --- FAST path feature 2: baro step ---
    if (have_step)
    {
        const float mag = (baro_step < 0.0f) ? -baro_step : baro_step;
        if (mag >= cfg.baro_step_m)
        {
            st.baro_step_seen = true;
            st.baro_step_ms   = t_since_launch_ms;
        }
    }

    // --- SLOW path: descent-rate collapse ---
    if (palt_rate_mps <= -cfg.ballistic_mps)
    {
        st.ballistic = true;
    }
    if (st.ballistic)
    {
        if (palt_rate_mps > -cfg.canopy_mps)
        {
            if (st.canopy_count < 0xFFFFu) st.canopy_count++;
        }
        else
        {
            st.canopy_count = 0;
        }
    }

    uint8_t reason = 0;
    if (st.shock_seen && st.baro_step_seen &&
        (t_since_launch_ms - st.shock_ms)    <= cfg.coincidence_ms &&
        (t_since_launch_ms - st.baro_step_ms) <= cfg.coincidence_ms)
    {
        reason |= kDeployReasonShockBaro;
    }
    // canopy_count > 0 is required independently of the configured threshold:
    // a threshold of 0 must still mean "one slow sample", never "fires the
    // instant the ballistic flag arms, while still falling at full speed".
    if (st.ballistic && st.canopy_count > 0 && st.canopy_count >= cfg.canopy_count)
    {
        reason |= kDeployReasonDescentCollapse;
    }

    if (reason != 0)
    {
        st.detected    = true;
        st.detected_ms = t_since_launch_ms;
        st.reason      = reason;
        return true;
    }
    return false;
}

} // namespace tr
