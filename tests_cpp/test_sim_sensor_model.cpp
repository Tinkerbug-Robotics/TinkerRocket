#include <gtest/gtest.h>

#include "SimSensorModel.h"

#include <cmath>

using namespace sim_sensor_model;

// ================================================================
// #512 — the sim's three synthetic sensors must describe ONE rigid-body motion.
//
// Before this, they did not: the accelerometer's gravity vector rotated exactly
// 180° opposite to what the sim's own gyroscope reported, while the magnetometer
// agreed with the (wrong-signed) gyro. The sim was emitting a motion no rigid
// body can perform, so the EKF could never fuse it — and the firmware sim could
// never validate attitude, heading, or guidance.
//
// THE contract of the sensor model is the rigid-body kinematic identity: a
// world-fixed direction, viewed from the body frame, obeys
//
//     du/dt = -omega x u
//
// Both gravity ("up") and the Earth field are world-fixed directions, so BOTH
// must satisfy it against the same gyro. These tests pin exactly that. A single
// sign flip in any one of the three functions breaks them.
// ================================================================

namespace {

constexpr float B_NORTH = 22.0f;
constexpr float B_EAST  =  5.0f;
constexpr float B_DOWN  = 42.0f;

void cross(const float a[3], const float b[3], float o[3])
{
    o[0] = a[1]*b[2] - a[2]*b[1];
    o[1] = a[2]*b[0] - a[0]*b[2];
    o[2] = a[0]*b[1] - a[1]*b[0];
}

float angleBetweenDeg(const float a[3], const float b[3])
{
    const float na = std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    const float nb = std::sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]);
    if (na < 1e-9f || nb < 1e-9f) return 0.0f;
    float c = (a[0]*b[0] + a[1]*b[1] + a[2]*b[2]) / (na * nb);
    if (c >  1.0f) c =  1.0f;
    if (c < -1.0f) c = -1.0f;
    return std::acos(c) * 180.0f / (float)M_PI;
}

// d/dt of a body-frame vector as the sim actually produces it, via du/dt =
// (du/dθ)·θ̇. The pitch step is FIXED (not scaled by the rate) so the central
// difference stays well-conditioned in float32 — perturbing by rate·h makes the
// step ~1e-6 rad at low rates and the difference is then pure rounding noise.
template <typename F>
void numericalDerivative(F f, float pitch, float pitch_rate, float out[3])
{
    constexpr float DTH = 1e-3f;   // rad
    float a[3], b[3];
    f(pitch - DTH, a);
    f(pitch + DTH, b);
    for (int i = 0; i < 3; ++i)
        out[i] = ((b[i] - a[i]) / (2.0f * DTH)) * pitch_rate;
}

// What rigid-body kinematics REQUIRES: du/dt = -omega x u.
template <typename F>
void kinematicDerivative(F f, float pitch, float pitch_rate, float out[3])
{
    float u[3], w[3], wxu[3];
    f(pitch, u);
    gyroInBody(pitch_rate, w);
    cross(w, u, wxu);
    for (int i = 0; i < 3; ++i) out[i] = -wxu[i];
}

auto UP    = [](float p, float o[3]) { upInBody(p, o); };
auto FIELD = [](float p, float o[3]) { fieldInBody(p, B_NORTH, B_EAST, B_DOWN, o); };

}  // namespace

// ================================================================
// The invariant — across the whole flight envelope.
// ================================================================

TEST(SimSensorModel, GravityObeysRigidBodyKinematics) {
    for (int pd = -20; pd <= 90; pd += 5) {
        for (float rd : {-30.0f, -14.0f, -1.0f, 1.0f, 14.0f, 30.0f}) {
            const float p = pd * (float)M_PI / 180.0f;
            const float r = rd * (float)M_PI / 180.0f;
            float num[3], kin[3];
            numericalDerivative(UP, p, r, num);
            kinematicDerivative(UP, p, r, kin);
            EXPECT_LT(angleBetweenDeg(num, kin), 0.05f)
                << "gravity violates du/dt = -w x u at pitch=" << pd
                << "° rate=" << rd << "°/s";
        }
    }
}

TEST(SimSensorModel, MagneticFieldObeysRigidBodyKinematics) {
    for (int pd = -20; pd <= 90; pd += 5) {
        for (float rd : {-30.0f, -14.0f, -1.0f, 1.0f, 14.0f, 30.0f}) {
            const float p = pd * (float)M_PI / 180.0f;
            const float r = rd * (float)M_PI / 180.0f;
            float num[3], kin[3];
            numericalDerivative(FIELD, p, r, num);
            kinematicDerivative(FIELD, p, r, kin);
            EXPECT_LT(angleBetweenDeg(num, kin), 0.05f)
                << "field violates du/dt = -w x u at pitch=" << pd
                << "° rate=" << rd << "°/s";
        }
    }
}

// The regression that actually happened: gravity and the field rotated in
// OPPOSITE senses. Pin that they now rotate together.
TEST(SimSensorModel, GravityAndFieldRotateInTheSameSense) {
    const float p = 60.0f * (float)M_PI / 180.0f;
    const float r = -14.0f * (float)M_PI / 180.0f;   // coast: nose pitching over

    float up_num[3], up_kin[3], f_num[3], f_kin[3];
    numericalDerivative(UP,    p, r, up_num);
    kinematicDerivative(UP,    p, r, up_kin);
    numericalDerivative(FIELD, p, r, f_num);
    kinematicDerivative(FIELD, p, r, f_kin);

    // Pre-#512 this was 180.0° for gravity and 0.0° for the field.
    EXPECT_LT(angleBetweenDeg(up_num, up_kin), 0.05f) << "gravity is inverted again";
    EXPECT_LT(angleBetweenDeg(f_num,  f_kin),  0.05f) << "field is inverted again";
}

// ================================================================
// Physical sanity — the model must describe the rocket we actually fly.
// ================================================================

TEST(SimSensorModel, UprightRocketReadsGravityOnItsNose) {
    float up[3];
    upInBody(90.0f * (float)M_PI / 180.0f, up);   // vertical on the rail
    EXPECT_NEAR(up[0], 1.0f, 1e-4);               // +X = nose
    EXPECT_NEAR(up[1], 0.0f, 1e-4);
    EXPECT_NEAR(up[2], 0.0f, 1e-4);
}

TEST(SimSensorModel, LevelRocketReadsGravityOnItsUpAxis) {
    float up[3];
    upInBody(0.0f, up);                            // horizontal
    EXPECT_NEAR(up[0], 0.0f, 1e-4);
    EXPECT_NEAR(up[1], 0.0f, 1e-4);
    EXPECT_NEAR(up[2], 1.0f, 1e-4);                // +Z = up
}

TEST(SimSensorModel, NoseDownIsAPositiveRotationAboutY) {
    // THE sign bug. In FLU, +Y is left and a positive rotation about it pitches
    // the nose DOWN. During coast the nose pitches over, i.e. pitch_rate < 0 —
    // which must yield a POSITIVE omega_y. The old model produced a negative one.
    float w[3];
    gyroInBody(-14.0f, w);                         // nose pitching over
    EXPECT_GT(w[1], 0.0f) << "nose-down must be +omega_y in FLU";
    EXPECT_NEAR(w[0], 0.0f, 1e-6);
    EXPECT_NEAR(w[2], 0.0f, 1e-6);

    gyroInBody(+14.0f, w);                         // nose rising
    EXPECT_LT(w[1], 0.0f) << "nose-up must be -omega_y in FLU";
}

TEST(SimSensorModel, LevelNorthPointingRocketSeesTheFieldOnTheRightAxes) {
    float b[3];
    fieldInBody(0.0f, B_NORTH, B_EAST, B_DOWN, b);
    // Nose = north → the field's north component lies along +X.
    // Left = west   → its east component reads -E on +Y.
    // Up            → its down component reads -D on +Z.
    EXPECT_NEAR(b[0],  B_NORTH, 1e-3);
    EXPECT_NEAR(b[1], -B_EAST,  1e-3);
    EXPECT_NEAR(b[2], -B_DOWN,  1e-3);
}

TEST(SimSensorModel, FieldMagnitudeIsInvariantUnderPitch) {
    // A rotation cannot change |B|. If it does, the projection is wrong.
    const float want = std::sqrt(B_NORTH*B_NORTH + B_EAST*B_EAST + B_DOWN*B_DOWN);
    for (int pd = -20; pd <= 90; pd += 5) {
        float b[3];
        fieldInBody(pd * (float)M_PI / 180.0f, B_NORTH, B_EAST, B_DOWN, b);
        const float got = std::sqrt(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);
        EXPECT_NEAR(got, want, 1e-3) << "at pitch " << pd << "°";
    }
    EXPECT_NEAR(want, 47.68f, 0.02f);   // the 47.69 µT seen in every bench log
}
