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
