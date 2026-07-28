// C shim so replay_deployment_detector.py can drive the REAL deployment
// detector over a recorded flight instead of a Python port of it.
//
// Two firmware sources are pulled in directly, and that is the whole point of
// this file:
//
//   DeploymentDetector.h  — the detector itself, the same translation unit the
//                           flight computer compiles.
//   flight_computer/main/config.h — the shipped config::DEPLOY_* tunables.
//                           The FC's config.h has no ESP-IDF dependency (pins
//                           live in board/board_v*.h, which is plain constants
//                           over <stdint.h>), so it compiles host-side as-is.
//
// So a replay can never disagree with the vehicle because someone edited a
// threshold and forgot the analysis tool: the driver rebuilds this shim
// whenever either header changes, and reports the values it actually got.
//
// Built on demand by the Python driver — see _build_shim() there. Not a
// standalone program.

#include <cstdint>
#include "DeploymentDetector.h"
#include "config.h"

extern "C" {

// The shipped tunables, as the flight computer defines them.  Split into a
// float and an unsigned bank so ctypes never has to know a struct layout.
//   f = [shock_ms2, baro_step_m, ballistic_mps, canopy_mps]
//   u = [shock_count, coincidence_ms, canopy_count, launch_lockout_ms,
//        flight_loop_update_rate_hz]
void tr_deploy_shipped_cfg(float* f, unsigned* u)
{
    f[0] = config::DEPLOY_SHOCK_MS2;
    f[1] = config::DEPLOY_BARO_STEP_M;
    f[2] = config::DEPLOY_BALLISTIC_MPS;
    f[3] = config::DEPLOY_CANOPY_MPS;
    u[0] = config::DEPLOY_SHOCK_COUNT;
    u[1] = config::DEPLOY_COINCIDENCE_MS;
    u[2] = config::DEPLOY_CANOPY_COUNT;
    u[3] = config::DEPLOY_LAUNCH_LOCKOUT_MS;
    u[4] = config::FLIGHT_LOOP_UPDATE_RATE;
}

// Run one reconstructed flight-loop stream through the detector.
//
// The caller supplies one entry per loop TICK (not per IMU sample) — the
// detector's counts are in loop iterations, so a replay that stepped once per
// logged IMU sample would make every count threshold fire in less wall time
// than it does on the vehicle.
//
// Returns 1 if the detector latched, 0 if it never did.  On a latch, *out_t is
// the tick time and *out_reason the tr::kDeployReason* bitmask.  *out_ballistic
// reports whether the descent-collapse path ever armed, which is how a replay
// tells "the slow path voted no" from "the slow path was never in play".
int tr_deploy_run(float shock_ms2, unsigned shock_count,
                  float baro_step_m, unsigned coincidence_ms,
                  float ballistic_mps, float canopy_mps, unsigned canopy_count,
                  unsigned launch_lockout_ms,
                  const unsigned* t_ms, const float* acc_ms2, const float* palt_m,
                  const unsigned char* new_baro, const float* rate_mps,
                  const unsigned char* burnout, const unsigned char* stop,
                  int n,
                  unsigned* out_t, unsigned char* out_reason,
                  unsigned char* out_ballistic)
{
    tr::DeploymentConfig cfg{};
    cfg.shock_ms2         = shock_ms2;
    cfg.shock_count       = (uint16_t)shock_count;
    cfg.baro_step_m       = baro_step_m;
    cfg.coincidence_ms    = coincidence_ms;
    cfg.ballistic_mps     = ballistic_mps;
    cfg.canopy_mps        = canopy_mps;
    cfg.canopy_count      = (uint16_t)canopy_count;
    cfg.launch_lockout_ms = launch_lockout_ms;

    tr::DeploymentState st;
    int latched = 0;
    for (int i = 0; i < n; ++i)
    {
        // Mirrors the flight loop's own gate: it stops stepping the detector
        // once the airframe is down, so a replay must too or it scores the
        // landing impact as a deployment.
        if (stop[i]) break;
        if (tr::deploymentDetectStep(st, cfg, t_ms[i], acc_ms2[i], palt_m[i],
                                     new_baro[i] != 0, rate_mps[i],
                                     burnout[i] != 0))
        {
            *out_t      = st.detected_ms;
            *out_reason = st.reason;
            latched     = 1;
            break;
        }
    }
    *out_ballistic = st.ballistic ? 1 : 0;
    return latched;
}

}  // extern "C"
