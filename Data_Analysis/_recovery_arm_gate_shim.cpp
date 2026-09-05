// C shim so replay_recovery_arm_gate.py can drive the REAL arming interlock
// over a recorded flight instead of a Python port of it.
//
// Two firmware sources are pulled in directly, and that is the whole point of
// this file:
//
//   RecoveryArmGate.h     — the interlock itself, the same header the flight
//                           computer compiles.
//   TR_KinematicChecks.cpp — the real altitude filter and quiescence detector.
//
// The second one is not optional. Two of the gate's inputs — `quiescent_flag`
// and the barometric rate `d_alt_est_` — are computed inside the kinematics
// module and NEVER REACH THE FLIGHT LOG, so a replay cannot read them back.
// Re-deriving them with the shipped code is the only way the replay can agree
// with the vehicle.
//
// THE COLD-FILTER POINT, which is what makes this replay mean anything.
// A recovery boot restarts the altitude filter from nothing: it has no history,
// and it free-runs until it has real samples. So the replay constructs a FRESH
// TR_KinematicChecks at the simulated reboot instant and feeds it only samples
// from that point on. Replaying against the log's own (warm) baro_alt_rate
// would be optimistic and would never exercise the gate's settle window at all.
//
// Built on demand by the Python driver — see _build_shim() there. Not a
// standalone program.

#include <cstdint>
#include <cstring>

#include "RecoveryArmGate.h"
#include "TR_KinematicChecks.h"

extern "C" {

// The shipped thresholds, as RecoveryArmGate defines them. Split into a float
// and an unsigned bank so ctypes never has to know a struct layout.
//   f = [descent_mps, boost_ms2, gnss_mps, freefall_ms2, spin_dps]
//   u = [descent_hold, boost_hold, gnss_hold, freefall_hold, spin_hold,
//        baro_settle, refute_hold, max_flight, apogee_arm]
void tr_rag_shipped_cfg(float* f, unsigned* u)
{
    const RecoveryArmGate::Config c;
    f[0] = c.descent_mps;   f[1] = c.boost_ms2;   f[2] = c.gnss_mps;
    f[3] = c.freefall_ms2;  f[4] = c.spin_dps;
    u[0] = c.descent_hold_ms;  u[1] = c.boost_hold_ms;
    u[2] = c.gnss_hold_ms;     u[3] = c.freefall_hold_ms;
    u[4] = c.spin_hold_ms;     u[5] = c.baro_settle_ms;
    u[6] = c.refute_hold_ms;   u[7] = c.max_flight_ms;
    u[8] = c.apogee_arm_ms;
}

// Replay one tick stream through a cold kinematics filter and the interlock.
//
// `reboot_at_ms` selects the simulated reboot: ticks before it are skipped
// entirely, and both the filter and the gate start fresh at the first tick
// at or after it. `flight_elapsed_at_reboot_ms` is what the restored snapshot
// would have said, i.e. how far into the flight the reboot happened.
//
// Returns the number of ticks actually stepped. Outputs are -1 / 0 when the
// corresponding event never happened.
int tr_rag_replay(
    int n,
    const unsigned* t_ms,          // tick time, launch-relative (may be negative-biased by the caller)
    const float* palt_m,           // pressure altitude, firmware formula
    const unsigned char* baro_new,
    const unsigned char* baro_healthy,
    const float* accel_norm,       // m/s^2, firmware low/high-g selection
    const float* roll_rate_dps,
    const unsigned char* imu_fresh,
    const float* gnss_alt_m,
    const float* gnss_vel_u_mps,
    const unsigned char* gnss_new,
    const unsigned char* gnss_ok,
    unsigned reboot_at_ms,
    unsigned flight_elapsed_at_reboot_ms,
    unsigned gnss_cold_ms,         // GNSS suppressed this long after the reboot
    const float* cfg_f,            // NULL => shipped defaults
    const unsigned* cfg_u,
    long* out_open_ms,             // tick at which the gate Opened, else -1
    unsigned char* out_arm,        // which arm carried it
    long* out_apogee_armed_ms,     // tick at which the restored apogee was released, else -1
    long* out_refute_ms,           // tick at which it Refuted, else -1
    unsigned char* out_trace)      // per-tick verdict, may be NULL
{
    RecoveryArmGate::Config cfg;
    if (cfg_f && cfg_u)
    {
        cfg.descent_mps = cfg_f[0]; cfg.boost_ms2 = cfg_f[1];
        cfg.gnss_mps    = cfg_f[2]; cfg.freefall_ms2 = cfg_f[3];
        cfg.spin_dps    = cfg_f[4];
        cfg.descent_hold_ms  = cfg_u[0]; cfg.boost_hold_ms    = cfg_u[1];
        cfg.gnss_hold_ms     = cfg_u[2]; cfg.freefall_hold_ms = cfg_u[3];
        cfg.spin_hold_ms     = cfg_u[4]; cfg.baro_settle_ms   = cfg_u[5];
        cfg.refute_hold_ms   = cfg_u[6]; cfg.max_flight_ms    = cfg_u[7];
        cfg.apogee_arm_ms    = cfg_u[8];
    }

    *out_open_ms = -1; *out_refute_ms = -1; *out_apogee_armed_ms = -1;
    *out_arm = 0;

    // Cold on purpose — see the header note.
    TR_KinematicChecks kin;
    RecoveryArmGate::State st;
    bool seeded = false;
    int stepped = 0;

    for (int i = 0; i < n; ++i)
    {
        if (t_ms[i] < reboot_at_ms) continue;
        if (!seeded) { RecoveryArmGate::reset(st, t_ms[i]); seeded = true; }

        // Run the shipped detector so the rate and the quiescence flag are the
        // ones the vehicle would compute, not an approximation of them. The
        // EKF-derived arguments only feed votes the interlock does not consume,
        // so zeros here cannot bias the gate's inputs.
        float pos[3] = {0.0f, 0.0f, 0.0f};
        float vel[3] = {0.0f, 0.0f, 0.0f};
        kin.kinematicChecks(palt_m[i],
                            imu_fresh[i] ? accel_norm[i] : 0.0f,
                            pos, vel,
                            roll_rate_dps[i],
                            baro_new[i] != 0,
                            gnss_alt_m[i],
                            gnss_new[i] != 0,
                            1.57f,                 // pitch: unused by these paths
                            false,                 // burnout
                            false,                 // baro locked out
                            gnss_vel_u_mps[i],
                            true,                  // ekf_healthy
                            baro_healthy[i] != 0);

        RecoveryArmGate::Inputs in;
        in.now_ms            = t_ms[i];
        in.flight_elapsed_ms = flight_elapsed_at_reboot_ms + (t_ms[i] - reboot_at_ms);
        in.baro_healthy      = baro_healthy[i] != 0;
        in.baro_rate_mps     = kin.d_alt_est_;
        in.imu_fresh         = imu_fresh[i] != 0;
        in.accel_norm_ms2    = imu_fresh[i] ? accel_norm[i] : 0.0f;
        in.gyro_norm_dps     = imu_fresh[i] ? (roll_rate_dps[i] < 0.0f ? -roll_rate_dps[i]
                                                                      : roll_rate_dps[i])
                                            : 0.0f;
        // A recovery boot power-cycles the receiver, and V_BCKP sits on the
        // switched rail, so it COLD STARTS: no fix for roughly 30 s. The log
        // was recorded with a receiver that had been locked for minutes, so
        // replaying its fix straight through would hand the gate an input the
        // vehicle would not have. Suppress it for the acquisition window.
        const bool gnss_live = (gnss_ok[i] != 0) &&
                               ((unsigned)(t_ms[i] - reboot_at_ms) >= gnss_cold_ms);
        in.gnss_ok           = gnss_live;
        in.gnss_vel_u_mps    = gnss_live ? gnss_vel_u_mps[i] : 0.0f;
        in.quiescent_flag    = kin.quiescent_flag;

        const RecoveryArmGate::Verdict before = st.verdict;
        RecoveryArmGate::step(st, in, cfg);

        if (before != st.verdict)
        {
            if (st.verdict == RecoveryArmGate::Verdict::Open)
            {
                *out_open_ms = (long)t_ms[i];
                *out_arm     = (unsigned char)st.opened_by;
            }
            else if (st.verdict == RecoveryArmGate::Verdict::Refuted)
            {
                *out_refute_ms = (long)t_ms[i];
            }
        }
        if (*out_apogee_armed_ms < 0 &&
            RecoveryArmGate::apogeeArmed(st, t_ms[i], cfg))
        {
            *out_apogee_armed_ms = (long)t_ms[i];
        }
        if (out_trace) out_trace[i] = (unsigned char)st.verdict;
        ++stepped;
    }
    return stepped;
}

}  // extern "C"
