#pragma once
#include <cstdint>

// Single source of truth for the post-launch burnout (motor-cutoff) detector
// (#197).  Pure, dependency-free logic so the host unit test
// (tests_cpp/test_burnout_detector.cpp) exercises the *real* firmware code
// instead of a hand-copied mirror that can silently diverge.
//
// #256: burnout detection MUST run independently of roll/servo control.
// Apogee detection (TR_KinematicChecks) is gated on burnout_detected, and
// drogue/main deployment is gated on apogee — so if this step is ever skipped
// (e.g. trapped behind `if (servo_enabled)`), recovery silently never fires.
// Callers must invoke it from the flight loop unconditionally.

namespace tr {

// Post-launch lockout: ignore the first 200 ms so boost vibration / pad
// transients can't trip the detector.  Strictly-greater comparison, i.e.
// t == 200 ms is still locked out, t == 201 ms is live.
inline constexpr uint32_t kBurnoutLaunchLockoutMs = 200;

// The axial specific force the detector reads (#1755).  Rocket +X is only the
// thrust axis when the board-to-rocket frame is right, and on AUTO mounting a
// frame snapped on a prep table, or a board mounted tail-forward that never
// confirmed its frame on the rail, flies a lateral or reversed +X: burnout then
// latched under thrust (thrust reads negative) or at a random time, or never.
// The thrust-axis check (TR_Orientation's ThrustAxisCheck) measures the real
// thrust direction over the first 150 ms of the flight; the flight computer's
// thrust_axis_policy.h decides what to do with it.  This step only applies it:
//   - axis_measuring: no input (returns false), so burnout cannot latch on an
//     axis not yet known (the 200 ms lockout normally covers it);
//   - thrust_axis (unit, rocket frame): the specific force along it, once the
//     measured thrust axis has disproved an AUTO frame;
//   - nullptr: rocket +X, as before (a right or manual frame, a restored
//     flight that never ran the check, no verdict).
inline bool burnoutAxialInput(const float f_rocket[3], bool axis_measuring,
                              const float* thrust_axis, float& axial_mps2)
{
    if (axis_measuring) return false;
    axial_mps2 = thrust_axis
        ? f_rocket[0] * thrust_axis[0] + f_rocket[1] * thrust_axis[1] + f_rocket[2] * thrust_axis[2]
        : f_rocket[0];
    return true;
}

// Advance the burnout detector by one IMU sample.
//
//   detected, neg_count : caller-owned latching state (persist across calls).
//   body_ax_mps2        : forward-axis (FRD body-X) specific force; goes
//                         negative once the motor stops thrusting.
//   t_since_launch_ms   : milliseconds since launch was latched.
//   neg_hysteresis      : consecutive negative samples required to latch
//                         (config::BURNOUT_NEG_HYSTERESIS).
//
// Returns true exactly once — on the rising edge that latches burnout — so the
// caller can stamp the burnout time and log.  Once `detected` is true the
// function is a no-op and returns false.
inline bool burnoutDetectStep(bool& detected, uint16_t& neg_count,
                              float body_ax_mps2, uint32_t t_since_launch_ms,
                              uint16_t neg_hysteresis)
{
    if (detected) return false;
    if (t_since_launch_ms <= kBurnoutLaunchLockoutMs) return false;
    if (body_ax_mps2 < 0.0f) {
        if (++neg_count >= neg_hysteresis) {
            detected = true;
            return true;
        }
    } else {
        neg_count = 0;
    }
    return false;
}

} // namespace tr
