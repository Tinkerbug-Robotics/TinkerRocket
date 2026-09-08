#pragma once

#include <stdint.h>

// #1137 item 6: which roll-control law runs this tick.
//
// The three-way choice used to be spread across three `if` conditions in the
// flight loop, and they did not agree about the EKF. The quorum predicate asks
// `ekf_initialized && ekf.isHealthy()`; the CONTROL gate asked only
// `ekf.isHealthy()`.
//
// That matters because isHealthy() answers "has the filter diverged recently",
// not "has it ever run". On a launch where the EKF never initialised — no GNSS
// fix, so no origin — it returns true, so control took the GAIN SCHEDULE
// branch and scaled its gains on an EKF speed that was never computed. With
// speed pinned at 0 the schedule sits at its low-speed end, which is its
// MAXIMUM gain (3x per the #253 schedule), so a no-GNSS launch flew the whole
// flight at the gain cap instead of dropping to the pure-gyro rate-null
// fallback that exists for exactly this case.
//
// Angle mode already carried the missing term (`use_angle_control &&
// ekf_initialized && ekf_ctrl_healthy`), which is why this only ever bit the
// rate-null path.
//
// Pure so the table is host-testable, in the style of inflight_refusal_policy.h
// and rail_restore_policy.h.
namespace RollControlModePolicy {

enum class Mode : uint8_t {
    Angle,          // quaternion roll extraction + controlAngle()
    GainSchedule,   // controlWithGainSchedule(): scales on EKF speed
    PureGyro,       // control(): raw gyro rate-null, no EKF dependency
};

// `ekf_healthy` is GpsInsEKF::isHealthy() — finite and not freshly diverged.
// `ekf_initialized` is whether the filter ever ran at all.
// `angle_segment` is true when the roll profile selects an ANGLE segment for
// this instant (only meaningful when angle mode is otherwise available).
inline Mode select(bool use_angle_control,
                   bool ekf_initialized,
                   bool ekf_healthy,
                   bool gain_sched_enabled,
                   bool angle_segment)
{
    // The EKF is usable for control only if it both ran and is not diverged.
    const bool ekf_usable = ekf_initialized && ekf_healthy;

    if (use_angle_control && ekf_usable && angle_segment) return Mode::Angle;
    if (gain_sched_enabled && ekf_usable)                 return Mode::GainSchedule;
    return Mode::PureGyro;
}

}  // namespace RollControlModePolicy
