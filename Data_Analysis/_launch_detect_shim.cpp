// C shim so analyze_launch_fallback.py can replay the REAL launch detector
// (TR_KinematicChecks.cpp) and the REAL ground-datum freeze (GroundRefFreeze.h)
// over a recorded flight, instead of a Python port of either.  Same pattern as
// _recovery_arm_gate_shim.cpp: the driver rebuilds this whenever a firmware
// source it pulls in changes, so the analysis can never quietly disagree with
// the vehicle.
//
// Built on demand by the Python driver -- see _build_shim() there.  Not a
// standalone program.

#include <cstdint>
#include <cmath>
#include "TR_KinematicChecks.h"
#include "GroundRefFreeze.h"

extern "C" {

// Replay one tick stream through a cold detector.
//
//   n, t_ms      ticks (one per flight-loop iteration) and their wall clock
//   t_us, p_pa   the barometer sample current at each tick (raw pressure)
//   baro_new     1 on ticks where that sample is new (the FC's bmp_new_for_kf)
//   accel_norm   |a| the FC would compute at each tick
//   imu_fresh    1 if the IMU is fresh at that tick; 0 feeds the detector 0 m/s2
//                and imu_healthy = false, exactly as main.cpp does
//   baro_healthy the #257 predicate at each tick
//   p_ref        the pad datum
//   datum_mode   0 = reference frozen at p_ref from the first tick (PRELAUNCH)
//                1 = reference re-seeds from the sample every tick, through
//                    GroundRefFreeze, as INITIALIZATION / READY do (#1108)
//
// Returns the tick index at which launch_flag latched, or -1.  *out_path is
// the TR_KinematicChecks::LaunchPath value; *out_freeze_idx is the first tick
// GroundRefFreeze held the datum (or -1); *out_ref_pa is the datum in force
// when launch latched (or at the end).
int tr_launch_replay(int n,
                     const unsigned* t_ms, const unsigned* t_us,
                     const float* p_pa, const unsigned char* baro_new,
                     const float* accel_norm, const unsigned char* imu_fresh,
                     const unsigned char* baro_healthy,
                     float p_ref, int datum_mode,
                     unsigned char* out_path, int* out_freeze_idx, float* out_ref_pa)
{
    TR_KinematicChecks kin;
    GroundRefFreeze::State gr;
    float ref = p_ref;
    *out_path = 0;
    *out_freeze_idx = -1;

    float pos[3] = {0.0f, 0.0f, 0.0f};
    float vel[3] = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < n; ++i)
    {
        setMockMillis(t_ms[i]);

        if (datum_mode == 1)
        {
            // Mirrors the FC's pre-PRELAUNCH re-seed block, which runs before
            // the kinematics call in the same pass.
            switch (GroundRefFreeze::step(gr, t_us[i], p_pa[i]))
            {
                case GroundRefFreeze::Verdict::Track:      ref = p_pa[i]; break;
                case GroundRefFreeze::Verdict::FreezeEdge: ref = gr.rollback_pa;
                                                           if (*out_freeze_idx < 0) *out_freeze_idx = i;
                                                           break;
                case GroundRefFreeze::Verdict::Frozen:     break;
            }
        }
        const float palt = 44330.0f * (1.0f - powf(p_pa[i] / ref, 1.0f / 5.255f));

        kin.kinematicChecks(palt,
                            imu_fresh[i] ? accel_norm[i] : 0.0f,
                            pos, vel,
                            0.0f,                  // roll rate: unused by launch
                            baro_new[i] != 0,
                            0.0f, false,           // gnss alt / new: unused by launch
                            1.57f,                 // pitch: unused
                            false,                 // burnout
                            false,                 // baro locked out
                            0.0f,                  // gnss vel_u
                            true,                  // ekf_healthy
                            baro_healthy[i] != 0,
                            imu_fresh[i] != 0);    // imu_healthy

        if (kin.launch_flag)
        {
            *out_path = (unsigned char)kin.launch_path;
            *out_ref_pa = ref;
            return i;
        }
    }
    *out_ref_pa = ref;
    return -1;
}

}  // extern "C"
