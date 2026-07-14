#include <gtest/gtest.h>

#include "SimPadAlign.h"

#include <cmath>

using namespace sim_pad_align;

namespace {

// The sim's on-rail pad attitude (matches SensorCollectorSim's constants).
constexpr float LAUNCH_ANGLE_RAD = 85.0f * 3.14159265f / 180.0f;
constexpr float B_NORTH = 22.0f;
constexpr float B_EAST  =  5.0f;
constexpr float B_DOWN  = 42.0f;

void simRef(float up[3], float mag[3])
{
    simPadReference(LAUNCH_ANGLE_RAD, B_NORTH, B_EAST, B_DOWN, up, mag);
}

float dot3(const float a[3], const float b[3])
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

float angleBetweenDeg(const float a[3], const float b[3])
{
    float u[3] = {a[0], a[1], a[2]};
    float v[3] = {b[0], b[1], b[2]};
    normalize3(u);
    normalize3(v);
    float c = dot3(u, v);
    if (c >  1.0f) c =  1.0f;
    if (c < -1.0f) c = -1.0f;
    return std::acos(c) * (180.0f / 3.14159265f);
}

// Component of m perpendicular to up — the only part a HEADING-ONLY mag update
// consumes. This is what must be continuous across the sim handover.
void horizontal(const float m[3], const float up[3], float out[3])
{
    float u[3] = {up[0], up[1], up[2]};
    normalize3(u);
    const float d = dot3(m, u);
    out[0] = m[0] - d * u[0];
    out[1] = m[1] - d * u[1];
    out[2] = m[2] - d * u[2];
    normalize3(out);
}

void rotate(const float R[9], const float v[3], float out[3])
{
    out[0] = v[0]; out[1] = v[1]; out[2] = v[2];
    applyRotation(R, out[0], out[1], out[2]);
}

// The bench case that produced the -190 dps excursion: board lying flat, so
// gravity sits ~86° off the configured +X nose axis.
void flatBenchPad(float up[3], float mag[3])
{
    const float off = 86.1f * 3.14159265f / 180.0f;
    up[0] = std::cos(off); up[1] = 0.0f; up[2] = std::sin(off);
    // A plausible bench field with a DIFFERENT dip than the sim's canonical one.
    mag[0] = 28.0f; mag[1] = -6.0f; mag[2] = 33.0f;
    const float n = std::sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
    mag[0] *= 44.3f / n; mag[1] *= 44.3f / n; mag[2] *= 44.3f / n;
}

}  // namespace

// ================================================================
// The reference the alignment maps FROM must match what the encoders emit.
// ================================================================

TEST(SimPadAlign, PadReferenceMatchesEncoderOutput) {
    float up[3], mag[3];
    simRef(up, mag);
    // Gravity along the rail at 85° nose-up.
    EXPECT_NEAR(up[0], std::sin(LAUNCH_ANGLE_RAD), 1e-5);
    EXPECT_NEAR(up[2], std::cos(LAUNCH_ANGLE_RAD), 1e-5);

    // Field at the 85° rail. UPDATED for #512: the old expectation here was
    // (25.6, 5.0, 39.9) — the value the pre-#512 encoders emitted and that the
    // bench logs recorded. That field was rotating in the WRONG sense (opposite
    // to the accelerometer), so pinning it here was pinning the bug. The correct
    // projection of the NED field onto the body axes is (N·cosθ - D·sinθ, -E,
    // -N·sinθ - D·cosθ). Magnitude is unchanged, as a rotation demands.
    EXPECT_NEAR(mag[0], -39.9f, 0.2f);
    EXPECT_NEAR(mag[1],  -5.0f, 0.1f);
    EXPECT_NEAR(mag[2], -25.6f, 0.2f);
    const float b = std::sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
    EXPECT_NEAR(b, 47.68f, 0.05f);
}

// ================================================================
// The two invariants that matter: the EKF's accel update and its (heading-only)
// mag update must both see ZERO step across the real→sim handover.
// ================================================================

TEST(SimPadAlign, FlatBenchGravityStepGoesToZero) {
    float up_sim[3], mag_sim[3], up_meas[3], mag_meas[3];
    simRef(up_sim, mag_sim);
    flatBenchPad(up_meas, mag_meas);

    // Before alignment there is a large gravity step. Note it is NOT the 86.1°
    // the [ORIENT] check prints: that is measured off the NOSE (+X), while the
    // sim's rail sits LAUNCH_ANGLE=85° from it — so the step the EKF actually
    // eats at handover is 86.1 − 5 = 81.1°.
    EXPECT_NEAR(angleBetweenDeg(up_sim, up_meas), 81.1f, 0.5f);

    float R[9];
    bool used_mag = false;
    ASSERT_TRUE(solvePadAlignment(up_sim, mag_sim, up_meas, mag_meas, R, &used_mag));
    EXPECT_TRUE(used_mag);

    float up_aligned[3];
    rotate(R, up_sim, up_aligned);
    EXPECT_LT(angleBetweenDeg(up_aligned, up_meas), 0.01f);   // was 86.1°
}

TEST(SimPadAlign, FlatBenchHeadingStepGoesToZero) {
    float up_sim[3], mag_sim[3], up_meas[3], mag_meas[3];
    simRef(up_sim, mag_sim);
    flatBenchPad(up_meas, mag_meas);

    float R[9];
    ASSERT_TRUE(solvePadAlignment(up_sim, mag_sim, up_meas, mag_meas, R));

    float mag_aligned[3];
    rotate(R, mag_sim, mag_aligned);

    // The mag update is heading-only, so what must match is the projection
    // perpendicular to gravity. TRIAD makes this exact.
    float h_sim[3], h_meas[3];
    horizontal(mag_aligned, up_meas, h_sim);
    horizontal(mag_meas,    up_meas, h_meas);
    EXPECT_LT(angleBetweenDeg(h_sim, h_meas), 0.01f);
}

TEST(SimPadAlign, ResidualIsDipOnlyWhichHeadingUpdateNeverSees) {
    // The sim's canonical field and a real bench field generally have different
    // magnetic dip. TRIAD cannot (and need not) reconcile that — it survives as a
    // dip mismatch, which a heading-only update does not consume. Assert it is
    // confined there: the FULL vector still differs, while heading does not.
    float up_sim[3], mag_sim[3], up_meas[3], mag_meas[3];
    simRef(up_sim, mag_sim);
    flatBenchPad(up_meas, mag_meas);

    float R[9];
    ASSERT_TRUE(solvePadAlignment(up_sim, mag_sim, up_meas, mag_meas, R));
    float mag_aligned[3];
    rotate(R, mag_sim, mag_aligned);

    EXPECT_GT(angleBetweenDeg(mag_aligned, mag_meas), 1.0f);   // dip differs
    float h_sim[3], h_meas[3];
    horizontal(mag_aligned, up_meas, h_sim);
    horizontal(mag_meas,    up_meas, h_meas);
    EXPECT_LT(angleBetweenDeg(h_sim, h_meas), 0.01f);          // heading does not
}

// ================================================================
// Must not change behaviour when there is nothing to fix.
// ================================================================

TEST(SimPadAlign, BoardAlreadyOnRailYieldsIdentity) {
    float up_sim[3], mag_sim[3];
    simRef(up_sim, mag_sim);

    float R[9];
    ASSERT_TRUE(solvePadAlignment(up_sim, mag_sim, up_sim, mag_sim, R));
    EXPECT_LT(rotationAngleDeg(R), 0.01f);
    for (int i = 0; i < 9; ++i) {
        EXPECT_NEAR(R[i], (i % 4 == 0) ? 1.0f : 0.0f, 1e-4) << "R[" << i << "]";
    }
}

// ================================================================
// Degradation: a missing / dead magnetometer must still kill the gravity step.
// ================================================================

TEST(SimPadAlign, NullMagFallsBackToGravityOnly) {
    float up_sim[3], mag_sim[3], up_meas[3], mag_meas[3];
    simRef(up_sim, mag_sim);
    flatBenchPad(up_meas, mag_meas);

    float R[9];
    bool used_mag = true;
    ASSERT_TRUE(solvePadAlignment(up_sim, mag_sim, up_meas, nullptr, R, &used_mag));
    EXPECT_FALSE(used_mag);

    float up_aligned[3];
    rotate(R, up_sim, up_aligned);
    EXPECT_LT(angleBetweenDeg(up_aligned, up_meas), 0.01f);   // gravity still exact
}

TEST(SimPadAlign, ImplausibleMagFallsBackToGravityOnly) {
    float up_sim[3], mag_sim[3], up_meas[3], mag_meas[3];
    simRef(up_sim, mag_sim);
    flatBenchPad(up_meas, mag_meas);

    float dead[3] = {0.0f, 0.0f, 0.0f};          // dead magnetometer
    float huge[3] = {500.0f, 0.0f, 0.0f};        // saturated / hard-iron blown
    EXPECT_FALSE(magIsPlausible(dead));
    EXPECT_FALSE(magIsPlausible(huge));

    for (float* m : {dead, huge}) {
        float R[9];
        bool used_mag = true;
        ASSERT_TRUE(solvePadAlignment(up_sim, mag_sim, up_meas, m, R, &used_mag));
        EXPECT_FALSE(used_mag);
        float up_aligned[3];
        rotate(R, up_sim, up_aligned);
        EXPECT_LT(angleBetweenDeg(up_aligned, up_meas), 0.01f);
    }
}

TEST(SimPadAlign, DegenerateGravityIsRejected) {
    float up_sim[3], mag_sim[3];
    simRef(up_sim, mag_sim);
    float zero[3] = {0.0f, 0.0f, 0.0f};
    float R[9];
    EXPECT_FALSE(solvePadAlignment(up_sim, mag_sim, zero, nullptr, R));
    // R must be left as identity so an un-solvable alignment is a no-op, not a
    // garbage rotation applied to every synthetic sample.
    for (int i = 0; i < 9; ++i) EXPECT_NEAR(R[i], (i % 4 == 0) ? 1.0f : 0.0f, 1e-6);
}

// ================================================================
// R must be a proper rotation, or it would shear every synthetic vector.
// ================================================================

TEST(SimPadAlign, SolvedRotationIsOrthonormalWithUnitDeterminant) {
    float up_sim[3], mag_sim[3], up_meas[3], mag_meas[3];
    simRef(up_sim, mag_sim);
    flatBenchPad(up_meas, mag_meas);

    float R[9];
    ASSERT_TRUE(solvePadAlignment(up_sim, mag_sim, up_meas, mag_meas, R));

    // R·Rᵀ == I
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            const float d = R[r*3+0]*R[c*3+0] + R[r*3+1]*R[c*3+1] + R[r*3+2]*R[c*3+2];
            EXPECT_NEAR(d, (r == c) ? 1.0f : 0.0f, 1e-4);
        }
    }
    const float det = R[0]*(R[4]*R[8] - R[5]*R[7])
                    - R[1]*(R[3]*R[8] - R[5]*R[6])
                    + R[2]*(R[3]*R[7] - R[4]*R[6]);
    EXPECT_NEAR(det, 1.0f, 1e-4);   // +1, not a reflection
}

TEST(SimPadAlign, AntiparallelGravityIsHandled) {
    // Board upside-down relative to the rail: gravity exactly opposite. The
    // Rodrigues fallback must still produce a valid 180° rotation, not NaN.
    float up_sim[3], mag_sim[3];
    simRef(up_sim, mag_sim);
    float up_meas[3] = {-up_sim[0], -up_sim[1], -up_sim[2]};

    float R[9];
    ASSERT_TRUE(solvePadAlignment(up_sim, mag_sim, up_meas, nullptr, R));
    for (int i = 0; i < 9; ++i) EXPECT_FALSE(std::isnan(R[i]));

    float up_aligned[3];
    rotate(R, up_sim, up_aligned);
    EXPECT_LT(angleBetweenDeg(up_aligned, up_meas), 0.01f);
    EXPECT_NEAR(rotationAngleDeg(R), 180.0f, 0.5f);
}
