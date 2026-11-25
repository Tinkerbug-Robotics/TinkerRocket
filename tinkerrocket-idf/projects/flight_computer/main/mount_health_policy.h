#pragma once
// #1755: the apps' "Mounting" health dot, before launch.
//
// On AUTO board-to-rocket mounting the frame flown is the one the pad
// orientation estimator's last completed still window confirmed
// (handleOrientationEstimate in main.cpp).  A board snapped on a prep table,
// then racked on a rail swaying too much to finish a window, flies the
// table's frame.  The launch log gives the confirmation's age and the boost
// thrust-axis check tests the frame (thrust_axis_policy.h), but both come too
// late for the operator.  This is the warning on the pad: a sensor-health
// state the flight computer sends in FC_STATUS_MSG's flags (FCS_MOUNT_*), the
// out computer passes to the apps ("mo"), and a base station relays from the
// LoRa SLOW frame's status bits 2-3.
//
//   - SH_NA: a manual mounting code (the owner's statement of the mounting,
//     which no window changes), or off the pad: the estimator runs only in
//     READY and PRELAUNCH, and launch latches the frame.
//   - SH_OK: AUTO on the pad, and a still window completed in the last
//     kFreshMs.
//   - SH_DEGRADED: AUTO on the pad, and no window this boot, or none in the
//     last kFreshMs.
//
// SH_BAD is not used: an unconfirmed frame is most often still the right one,
// and the thrust-axis check stands guard in the boost, so the apps hold
// "Ready to fly" to amber, never red.
//
// Pure; host-tested in tests_cpp/test_mount_health_policy.cpp.

#include <cstdint>

#include <RocketComputerTypes.h>   // SensorHealthState

namespace MountHealthPolicy {

// A window is 2 s of still blocks, so a rocket left still on the rail
// completes one every 2 s; 30 s is many missed in a row.
inline constexpr uint32_t kFreshMs = 30000;

// manual_mounting: b2r_active_mode == ORIENT_MODE_MANUAL.  on_pad: rocket_state
// READY or PRELAUNCH.  last_window_ms: time_ms() when the last still window
// completed, 0 for none this boot.  now_ms: time_ms() now.  The age is taken
// in unsigned arithmetic, so it holds across the 32-bit millisecond wrap.
inline SensorHealthState state(bool manual_mounting, bool on_pad,
                               uint32_t last_window_ms, uint32_t now_ms)
{
    if (manual_mounting || !on_pad) return SH_NA;
    if (last_window_ms == 0) return SH_DEGRADED;
    return (uint32_t)(now_ms - last_window_ms) <= kFreshMs ? SH_OK : SH_DEGRADED;
}

} // namespace MountHealthPolicy
