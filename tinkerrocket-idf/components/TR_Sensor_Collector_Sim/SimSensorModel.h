#pragma once
//
//  SimSensorModel.h — the firmware sim's synthetic sensor models (#512).
//
//  These three functions define what the simulated vehicle's accelerometer,
//  gyroscope and magnetometer read at a given pitch. They MUST describe one
//  common rigid-body motion — and before #512 they did not: the accelerometer's
//  up-vector rotated exactly 180° opposite to what the gyroscope reported, while
//  the magnetometer agreed with the (wrong-signed) gyro. The estimator was being
//  handed contradictory data, so the firmware sim could never validate the EKF,
//  attitude, heading or guidance.
//
//  ── Frame convention ────────────────────────────────────────────────────────
//  Rocket body frame is FLU:  X = nose (forward),  Y = left,  Z = up.
//  `pitch` is the nose-up angle from horizontal: 90° = vertical on the rail,
//  0° = horizontal. World frame is NED (North, East, Down).
//
//  Body axes expressed in NED at pitch θ:
//      X_body = ( cosθ, 0, -sinθ )      nose
//      Y_body = ( 0,   -1,  0     )      left  (facing north, left is west)
//      Z_body = (-sinθ, 0, -cosθ )      up    (= X × Y)
//
//  ── The invariant ───────────────────────────────────────────────────────────
//  A world-fixed direction, viewed in the body frame, obeys
//
//      du/dt = -omega x u
//
//  Both `upInBody` (gravity) and `fieldInBody` (Earth's field) are world-fixed
//  directions, so BOTH must satisfy this with `gyroInBody`. That identity is the
//  whole contract of this header, and it is what test_sim_sensor_model.cpp pins.
//  Any future edit to one of these three functions must keep the other two in
//  step, or the sim silently goes back to emitting impossible data.
//
//  Pure math, no ESP/Arduino dependencies, so it is host-testable.
//

#include <math.h>

namespace sim_sensor_model {

// World "up" expressed in the body frame. A stationary accelerometer reads the
// specific force, which is +g along this direction.
//   θ = 90° (vertical) → (1,0,0) = +X, the nose.   An upright rocket reads +g on its nose. ✓
//   θ =  0° (horizontal) → (0,0,1) = +Z, up.        A level rocket reads +g on its up axis. ✓
inline void upInBody(float pitch_rad, float out[3])
{
    out[0] = sinf(pitch_rad);
    out[1] = 0.0f;
    out[2] = cosf(pitch_rad);
}

// Body angular velocity for a given pitch RATE.
//
// The sign is the whole bug (#512). In FLU, a positive rotation about +Y (left)
// carries +Z toward +X — i.e. it pitches the nose DOWN. But `pitch` measures
// nose-UP from horizontal, so a rising nose (θ̇ > 0) is a NEGATIVE rotation
// about +Y. Hence omega_y = -pitch_rate, not +pitch_rate.
//
// Concretely: during coast the nose pitches over (θ̇ < 0), which must produce a
// POSITIVE omega_y. The old model produced a negative one.
inline void gyroInBody(float pitch_rate, float out[3])
{
    out[0] = 0.0f;
    out[1] = -pitch_rate;
    out[2] = 0.0f;
}

// Earth's magnetic field (NED components, µT) expressed in the body frame:
// project the NED field onto each body axis.
//   B_x = B · X_body =  N·cosθ - D·sinθ
//   B_y = B · Y_body = -E
//   B_z = B · Z_body = -N·sinθ - D·cosθ
// Check at θ=0 (level, nose north): (N, -E, -D) — north along the nose, the
// field's east component along "left" is -E, and "down" along "up" is -D. ✓
inline void fieldInBody(float pitch_rad,
                        float b_north, float b_east, float b_down,
                        float out[3])
{
    const float sp = sinf(pitch_rad);
    const float cp = cosf(pitch_rad);
    out[0] =  b_north * cp - b_down * sp;
    out[1] = -b_east;
    out[2] = -b_north * sp - b_down * cp;
}

}  // namespace sim_sensor_model
