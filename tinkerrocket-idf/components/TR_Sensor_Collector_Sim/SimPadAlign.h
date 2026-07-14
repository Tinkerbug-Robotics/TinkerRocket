#pragma once
//
//  SimPadAlign.h — pad-frame alignment for the firmware sim (#508).
//
//  The sim's physics assume the rocket is standing on the rail
//  (pitch = LAUNCH_ANGLE_RAD). On a bench the board is usually lying flat, so at
//  the instant synthetic data replaces real data the rocket-frame gravity vector
//  jumps by ~90°. The EKF still has accel/mag updates live on the pad, and the
//  only state that can explain "large attitude correction, no measured rate" is
//  the gyro bias — which slams to a physically impossible value (-190 dps was
//  observed) and is then FROZEN IN at launch, because accel/mag updates are
//  gated off for the whole flight. The attitude solution never recovers.
//
//  Rather than clamp the symptom, remove the discontinuity: solve the fixed
//  rotation that carries the sim's assumed pad attitude onto the board's ACTUAL
//  resting attitude, and rotate every synthetic body-frame vector (accel, gyro,
//  mag) by it. The handover is then continuous by construction.
//
//  Why this is safe for the trajectory: rotating the body-frame vectors AND the
//  implied attitude by the same R leaves world-frame acceleration unchanged
//    R_wb(q·Rᵀ)·(R·a_body) == R_wb(q)·a_body
//  so GNSS/baro need no adjustment — the simulated flight path is identical, only
//  the body frame is re-expressed.
//
//  Solved with TRIAD from two vector pairs (gravity + field). TRIAD makes the
//  gravity direction and the field's projection PERPENDICULAR to gravity exact.
//  That is precisely what the two EKF updates consume — the accel update levels
//  attitude, and the mag update is heading-only — so both see a zero step. Any
//  residual is confined to magnetic dip (the sim's canonical field vs the bench's
//  real one), which a heading-only update never looks at.
//
//  Pure math, no ESP/Arduino dependencies, so it is host-testable
//  (tests_cpp/test_sim_pad_align.cpp).
//

#include "SimSensorModel.h"

#include <math.h>

namespace sim_pad_align {

inline bool normalize3(float v[3])
{
    const float n = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (!(n > 1e-6f)) return false;
    v[0] /= n; v[1] /= n; v[2] /= n;
    return true;
}

inline void cross3(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

// Orthonormal triad from a primary vector p and a secondary reference s:
//   t1 = p̂ ; t2 = (t1 × s)̂ ; t3 = t1 × t2
// Stored row-major with t1,t2,t3 as COLUMNS. False if p and s are parallel.
inline bool triad(const float p[3], const float s[3], float M[9])
{
    float t1[3] = {p[0], p[1], p[2]};
    if (!normalize3(t1)) return false;
    float t2[3];
    cross3(t1, s, t2);
    if (!normalize3(t2)) return false;   // degenerate: p ∥ s
    float t3[3];
    cross3(t1, t2, t3);
    M[0] = t1[0]; M[1] = t2[0]; M[2] = t3[0];
    M[3] = t1[1]; M[4] = t2[1]; M[5] = t3[1];
    M[6] = t1[2]; M[7] = t2[2]; M[8] = t3[2];
    return true;
}

// R = A * Bᵀ (row-major 3x3).
inline void mulABt(const float A[9], const float B[9], float R[9])
{
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            R[r * 3 + c] = A[r * 3 + 0] * B[c * 3 + 0]
                         + A[r * 3 + 1] * B[c * 3 + 1]
                         + A[r * 3 + 2] * B[c * 3 + 2];
}

inline void setIdentity(float R[9])
{
    for (int i = 0; i < 9; ++i) R[i] = (i % 4 == 0) ? 1.0f : 0.0f;
}

// Minimal rotation carrying unit â onto unit b̂ (Rodrigues), handling the
// parallel and antiparallel degeneracies.
inline void rotationBetween(const float a[3], const float b[3], float R[9])
{
    float v[3];
    cross3(a, b, v);
    const float c  = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    const float s2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];

    if (s2 < 1e-12f)
    {
        if (c > 0.0f) { setIdentity(R); return; }        // already aligned
        // Antiparallel: 180° about any axis ⊥ a.
        float axis[3] = {1.0f, 0.0f, 0.0f};
        if (fabsf(a[0]) > 0.9f) { axis[0] = 0.0f; axis[1] = 1.0f; }
        float p[3];
        cross3(a, axis, p);
        normalize3(p);
        const float x = p[0], y = p[1], z = p[2];
        R[0] = 2*x*x-1; R[1] = 2*x*y;   R[2] = 2*x*z;
        R[3] = 2*x*y;   R[4] = 2*y*y-1; R[5] = 2*y*z;
        R[6] = 2*x*z;   R[7] = 2*y*z;   R[8] = 2*z*z-1;
        return;
    }

    const float k = (1.0f - c) / s2;    // R = I + [v]x + [v]x² (1-c)/s²
    const float vx = v[0], vy = v[1], vz = v[2];
    R[0] = 1.0f + k * (-vz*vz - vy*vy);  R[1] = -vz + k * (vx*vy);            R[2] =  vy + k * (vx*vz);
    R[3] =  vz  + k * (vx*vy);           R[4] = 1.0f + k * (-vz*vz - vx*vx);  R[5] = -vx + k * (vy*vz);
    R[6] = -vy  + k * (vx*vz);           R[7] =  vx + k * (vy*vz);            R[8] = 1.0f + k * (-vy*vy - vx*vx);
}

// The sim's own pad attitude, in ROCKET frame, at `launch_angle_rad`. Delegates
// to the shared sensor model so this alignment reference cannot drift from what
// the encoders actually emit — which is exactly how #512 hid for so long.
inline void simPadReference(float launch_angle_rad,
                            float b_north, float b_east, float b_down,
                            float up_out[3], float mag_out[3])
{
    sim_sensor_model::upInBody(launch_angle_rad, up_out);
    sim_sensor_model::fieldInBody(launch_angle_rad, b_north, b_east, b_down, mag_out);
}

// True when |m| looks like an Earth field (µT). Guards against a dead/saturated
// magnetometer poisoning the alignment.
inline bool magIsPlausible(const float m[3])
{
    const float m2 = m[0]*m[0] + m[1]*m[1] + m[2]*m[2];
    return m2 >= (15.0f * 15.0f) && m2 <= (80.0f * 80.0f);
}

// Solve the rotation carrying the sim's pad frame onto the measured one.
// `mag_sim` / `mag_meas` may be null (or implausible) → falls back to aligning
// gravity alone, which still removes the attitude step; only the heading step
// survives. Returns false (R = identity) if gravity itself is unusable.
inline bool solvePadAlignment(const float up_sim[3], const float mag_sim[3],
                              const float up_meas[3], const float mag_meas[3],
                              float R[9], bool* used_mag_out = nullptr)
{
    setIdentity(R);
    if (used_mag_out) *used_mag_out = false;
    if (up_sim == nullptr || up_meas == nullptr) return false;

    float us[3] = {up_sim[0],  up_sim[1],  up_sim[2]};
    float um[3] = {up_meas[0], up_meas[1], up_meas[2]};
    if (!normalize3(us) || !normalize3(um)) return false;

    if (mag_sim != nullptr && mag_meas != nullptr && magIsPlausible(mag_meas))
    {
        float Ms[9], Mm[9];
        if (triad(us, mag_sim, Ms) && triad(um, mag_meas, Mm))
        {
            mulABt(Mm, Ms, R);                       // maps sim → measured
            if (used_mag_out) *used_mag_out = true;
            return true;
        }
    }

    rotationBetween(us, um, R);                      // gravity-only fallback
    return true;
}

// Rotation angle of R in degrees (0 for identity).
inline float rotationAngleDeg(const float R[9])
{
    float c = (R[0] + R[4] + R[8] - 1.0f) * 0.5f;
    if (c >  1.0f) c =  1.0f;
    if (c < -1.0f) c = -1.0f;
    return acosf(c) * (180.0f / 3.14159265f);
}

inline void applyRotation(const float R[9], float& x, float& y, float& z)
{
    const float rx = R[0] * x + R[1] * y + R[2] * z;
    const float ry = R[3] * x + R[4] * y + R[5] * z;
    const float rz = R[6] * x + R[7] * y + R[8] * z;
    x = rx; y = ry; z = rz;
}

}  // namespace sim_pad_align
