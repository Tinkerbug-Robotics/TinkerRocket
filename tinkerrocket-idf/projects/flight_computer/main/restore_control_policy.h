#pragma once

#include <stdint.h>

// #1769: roll control after an FC reboot in flight.
//
// The #1176 restore restarts the navigation filter from the OC's flight
// snapshot: the velocity and attitude it had just before the reset. The time
// the reboot took was flown with no filter running, so after a reset in the
// boost the restored speed is low by everything the motor added in the gap
// (80-120 m/s on the review's runs), and nothing corrects it quickly: GNSS
// comes back late or not at all, the raw filter holds the barometer out while
// its own climb rate is high, and GpsInsEKF weighs it down by the port error
// at that same stale speed. The filter says so -- the raw filter's restore
// declares its velocity unsure by 20 m/s (RAW_RESTORE_VEL_SIGMA_MPS),
// GpsInsEKF's by 5 m/s on top of the snapshot's own -- but roll control never
// read that. Once the 500 ms settle ended and the filter answered healthy, the
// gain schedule ran on the stale speed: up to 26x the gain at 100-200 m/s.
// The pure-gyro fallback is no answer either: its base gains are themselves
// 4-16x the scheduled gain at those speeds.
//
// So after a restore the fins hold neutral until the filter's velocity is
// trusted again, by the velocity half of the EKF row's "OK": healthy, and
// every velocity axis's variance under EKF_VEL_VAR_OK (2 m/s, 1 sigma). The
// restore leaves it well over that (400 (m/s)^2 on the raw filter, 25 or more
// on GpsInsEKF), and only a measurement brings it down. On the barometer
// alone the horizontal axes stay unsure, so the fins hold until GNSS's
// velocity is back; if it never is, they hold neutral for the rest of the
// flight. That is the vehicle's passive flight, as when the speed gate never
// opens.
//
// A restore that found no filter (the snapshot's ekf_initialized false: the
// launch came before any start) has no speed to be stale. It flies the
// pure-gyro rate-null after the settle, as it did before the reset, and for
// the rest of the flight: a filter that starts in flight after it (a
// receiver absent this boot starts one degraded, at zero velocity and with
// a start variance well under EKF_VEL_VAR_OK) is never one roll control runs
// on.
//
// Angle mode stays off for the rest of a restored flight; its segments fly as
// the rate-null. The restored roll is the snapshot's plus however far the
// vehicle rolled in the gap, and there is no test for when it is right
// again: the restore's own attitude sigma (15 deg) already passes the EKF
// row's attitude test, neither filter fuses the magnetometer in flight before
// the canopy, GpsInsEKF takes no heading from GNSS, and the raw filter only
// as far as the flight's lateral acceleration shows it (none on a straight
// climb).
//
// Pure so it is host-testable (tests_cpp/test_restore_control_policy.cpp;
// a reboot in the boost through the raw filter in
// tests_cpp/test_raw_gnss_shadow.cpp).
namespace RestoreControlPolicy {

// The filter's velocity is one the gain schedule may run on: it answers
// healthy (isHealthy: the raw filter's running, fresh and finite; GpsInsEKF's
// finite and not freshly diverged) and each velocity axis's variance
// (getCovVel) is under var_ok. A NaN or negative variance is no variance at
// all, and is not trusted.
inline bool velocityTrusted(bool ekf_healthy, const float (&var_vel)[3], float var_ok)
{
    if (!ekf_healthy)
        return false;
    for (int i = 0; i < 3; ++i)
        if (!(var_vel[i] >= 0.0f && var_vel[i] < var_ok))
            return false;
    return true;
}

// After a restore (the hold armed in setup_fc): do the fins still hold
// neutral? Until the settle's end (signed compare: wrap-safe), and past it
// while the filter the restore brought back (restored_with_filter: the
// snapshot's ekf_initialized) has no trusted velocity. main.cpp latches the
// release: once the hold ends it is not taken up again.
inline bool holdFins(uint32_t now_ms, uint32_t settle_end_ms, bool restored_with_filter, bool velocity_trusted)
{
    if ((int32_t)(now_ms - settle_end_ms) < 0)
        return true;
    return restored_with_filter && !velocity_trusted;
}

// The filter is one roll control may run on (the gain schedule's speed, and
// angle mode below): it has started and answers healthy, and on a restored
// flight (reboot_recovery_telem: from the restore to the landing) it is the
// filter the restore brought back, not one started in flight after a restore
// that found none.
inline bool filterUsableForControl(bool ekf_initialized, bool ekf_healthy, bool restored_flight,
                                   bool restored_with_filter)
{
    return ekf_initialized && ekf_healthy && (!restored_flight || restored_with_filter);
}

// Angle mode this pass: opted in, the filter usable for control, and not a
// restored flight.
inline bool angleModeAllowed(bool use_angle_control, bool ekf_ctrl_healthy, bool restored_flight)
{
    return use_angle_control && ekf_ctrl_healthy && !restored_flight;
}

}  // namespace RestoreControlPolicy
