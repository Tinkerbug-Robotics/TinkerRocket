// Tests for TR_Orientation — the board→rocket mounting orientation
// utilities behind the arbitrary-mounting feature.  Covers the 24-code
// discrete rotation set, quaternion round-trips, the gravity-vector
// nose snap (pad auto-detect building block), and the pad attitude
// quaternion previously inlined in flight_computer main.cpp.

#include <gtest/gtest.h>
#include "TR_Orientation.h"
#include <cmath>
#include <cstring>
#include <set>
#include <array>

namespace {

constexpr float kG = 9.80665f;

void matMul(const float A[9], const float B[9], float out[9])
{
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
        {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) s += A[r * 3 + k] * B[k * 3 + c];
            out[r * 3 + c] = s;
        }
}

void matVec(const float R[9], const float v[3], float out[3])
{
    for (int r = 0; r < 3; r++)
        out[r] = R[r * 3 + 0] * v[0] + R[r * 3 + 1] * v[1] + R[r * 3 + 2] * v[2];
}

float det3(const float R[9])
{
    return R[0] * (R[4] * R[8] - R[5] * R[7])
         - R[1] * (R[3] * R[8] - R[5] * R[6])
         + R[2] * (R[3] * R[7] - R[4] * R[6]);
}

}  // namespace

// ---------- Discrete code set ----------

TEST(Orientation, Code0_IsIdentity) {
    float R[9];
    orientCodeToMatrix(ORIENT_CODE_IDENTITY, R);
    for (int i = 0; i < 9; i++)
    {
        EXPECT_NEAR(R[i], (i % 4 == 0) ? 1.0f : 0.0f, 1e-6f) << "entry " << i;
    }
}

TEST(Orientation, All24_ProperRotations_AndDistinct) {
    std::set<std::array<int, 9>> seen;
    for (uint8_t code = 0; code < ORIENT_CODE_COUNT; code++)
    {
        float R[9];
        orientCodeToMatrix(code, R);

        // Orthonormal: R * R^T == I
        float Rt[9] = { R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8] };
        float I[9];
        matMul(R, Rt, I);
        for (int i = 0; i < 9; i++)
        {
            EXPECT_NEAR(I[i], (i % 4 == 0) ? 1.0f : 0.0f, 1e-5f)
                << "code " << (int)code << " entry " << i;
        }

        // Proper (det +1), and entries are exact 0/±1
        EXPECT_NEAR(det3(R), 1.0f, 1e-5f) << "code " << (int)code;
        std::array<int, 9> key{};
        for (int i = 0; i < 9; i++)
        {
            EXPECT_NEAR(R[i], roundf(R[i]), 1e-6f) << "code " << (int)code;
            key[i] = (int)lroundf(R[i]);
        }
        EXPECT_TRUE(seen.insert(key).second) << "duplicate matrix for code " << (int)code;
    }
}

TEST(Orientation, NoseAxis_MapsToRocketX_AllClocks) {
    // nose_sel order: +X, -X, +Y, -Y, +Z, -Z
    const float axes[6][3] = {
        { 1, 0, 0 }, { -1, 0, 0 },
        { 0, 1, 0 }, { 0, -1, 0 },
        { 0, 0, 1 }, { 0, 0, -1 },
    };
    for (int nose = 0; nose < 6; nose++)
    {
        for (int clock = 0; clock < 4; clock++)
        {
            float R[9], out[3];
            orientCodeToMatrix((uint8_t)(nose * 4 + clock), R);
            matVec(R, axes[nose], out);
            EXPECT_NEAR(out[0], 1.0f, 1e-6f) << "nose " << nose << " clock " << clock;
            EXPECT_NEAR(out[1], 0.0f, 1e-6f) << "nose " << nose << " clock " << clock;
            EXPECT_NEAR(out[2], 0.0f, 1e-6f) << "nose " << nose << " clock " << clock;
        }
    }
}

TEST(Orientation, Clock_IsQuarterTurnAboutRocketX) {
    // Rx(90°) about rocket +X, CCW (right-hand rule)
    const float Rx90[9] = { 1, 0, 0,   0, 0, -1,   0, 1, 0 };
    for (int nose = 0; nose < 6; nose++)
    {
        float prev[9];
        orientCodeToMatrix((uint8_t)(nose * 4), prev);
        for (int clock = 1; clock < 4; clock++)
        {
            float expected[9], actual[9];
            matMul(Rx90, prev, expected);
            orientCodeToMatrix((uint8_t)(nose * 4 + clock), actual);
            for (int i = 0; i < 9; i++)
            {
                EXPECT_NEAR(actual[i], expected[i], 1e-5f)
                    << "nose " << nose << " clock " << clock << " entry " << i;
            }
            memcpy(prev, actual, sizeof(prev));
        }
    }
}

TEST(Orientation, MatrixToCode_RoundTrip) {
    for (uint8_t code = 0; code < ORIENT_CODE_COUNT; code++)
    {
        float R[9];
        orientCodeToMatrix(code, R);
        uint8_t back = 0xFF;
        ASSERT_TRUE(orientMatrixToCode(R, back)) << "code " << (int)code;
        EXPECT_EQ(back, code);
    }
    // A non-discrete rotation must NOT match (10° about Z)
    const float c = cosf(10.0f * (float)M_PI / 180.0f);
    const float s = sinf(10.0f * (float)M_PI / 180.0f);
    const float R10[9] = { c, -s, 0,   s, c, 0,   0, 0, 1 };
    uint8_t code = 0;
    EXPECT_FALSE(orientMatrixToCode(R10, code));
}

TEST(Orientation, InvalidCode_FallsBackToIdentity) {
    float R[9];
    orientCodeToMatrix(24, R);
    for (int i = 0; i < 9; i++)
    {
        EXPECT_NEAR(R[i], (i % 4 == 0) ? 1.0f : 0.0f, 1e-6f);
    }
    EXPECT_STREQ(orientCodeName(24), "?");
}

// ---------- Quaternion round-trips ----------

TEST(Orientation, Quat_RoundTrips_AllCodes) {
    for (uint8_t code = 0; code < ORIENT_CODE_COUNT; code++)
    {
        float R[9], q[4], R2[9];
        orientCodeToMatrix(code, R);
        orientCodeToQuat(code, q);

        // Unit norm, canonical sign
        const float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        EXPECT_NEAR(n, 1.0f, 1e-5f) << "code " << (int)code;
        EXPECT_GE(q[0], -1e-6f) << "code " << (int)code;

        orientQuatToMatrix(q, R2);
        for (int i = 0; i < 9; i++)
        {
            EXPECT_NEAR(R2[i], R[i], 1e-5f) << "code " << (int)code << " entry " << i;
        }
    }
}

TEST(Orientation, Quat_SurvivesWireQuantization) {
    // Round-trip every code through the int16 ×10000 wire encoding the
    // OC/flight-settings use; the recovered matrix must still match.
    for (uint8_t code = 0; code < ORIENT_CODE_COUNT; code++)
    {
        float q[4], R[9], R2[9];
        orientCodeToQuat(code, q);
        float qw[4];
        for (int i = 0; i < 4; i++)
        {
            const int16_t wire = (int16_t)lroundf(q[i] * ORIENT_QUAT_WIRE_SCALE);
            qw[i] = (float)wire / ORIENT_QUAT_WIRE_SCALE;
        }
        orientCodeToMatrix(code, R);
        orientQuatToMatrix(qw, R2);
        for (int i = 0; i < 9; i++)
        {
            EXPECT_NEAR(R2[i], R[i], 1e-3f) << "code " << (int)code << " entry " << i;
        }
    }
}

// ---------- Gravity-vector nose snap ----------

TEST(Orientation, NearestNose_CleanAxes) {
    const float vecs[6][3] = {
        { kG, 0, 0 }, { -kG, 0, 0 },
        { 0, kG, 0 }, { 0, -kG, 0 },
        { 0, 0, kG }, { 0, 0, -kG },
    };
    for (int nose = 0; nose < 6; nose++)
    {
        float residual = -1.0f;
        const uint8_t code = orientNearestNoseCode(vecs[nose], &residual);
        EXPECT_EQ(code, (uint8_t)(nose * 4)) << "nose " << nose;
        EXPECT_NEAR(residual, 0.0f, 1e-3f) << "nose " << nose;
    }
}

TEST(Orientation, NearestNose_TiltedVector_ResidualAngle) {
    // 10° off +Z toward +X — still snaps to +Z with ~10° residual
    const float tilt = 10.0f * (float)M_PI / 180.0f;
    const float v[3] = { kG * sinf(tilt), 0.0f, kG * cosf(tilt) };
    float residual = -1.0f;
    const uint8_t code = orientNearestNoseCode(v, &residual);
    EXPECT_EQ(code, (uint8_t)(4 * 4));  // +Z nose
    EXPECT_NEAR(residual, 10.0f, 0.1f);
}

TEST(Orientation, NearestNose_SnappedCodeMapsVectorNearRocketX) {
    // The snapped code's matrix must take the measured vector to within
    // the residual of rocket +X — the property the pad auto-detect needs.
    const float v[3] = { 2.0f, 1.0f, 9.5f };  // mostly +Z, messy
    float residual = -1.0f;
    const uint8_t code = orientNearestNoseCode(v, &residual);
    float R[9], out[3];
    orientCodeToMatrix(code, R);
    matVec(R, v, out);
    const float n = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    const float angle = acosf(out[0] / n) * 180.0f / (float)M_PI;
    EXPECT_NEAR(angle, residual, 1e-3f);
    EXPECT_LT(angle, 45.0f);
}

TEST(Orientation, NearestNose_ZeroVector_Identity) {
    const float v[3] = { 0, 0, 0 };
    float residual = -1.0f;
    EXPECT_EQ(orientNearestNoseCode(v, &residual), ORIENT_CODE_IDENTITY);
    EXPECT_EQ(residual, 0.0f);
}

TEST(Orientation, Quat_PinnedValue_Code17) {
    // +Z r90 is the x→y→z cyclic permutation: 120° about (1,1,1), so the
    // quaternion is exactly (0.5, 0.5, 0.5, 0.5).  Pinned because the iOS
    // decode test (MessageParserTests.testFlightSettingsDecode) embeds the
    // same value — keep the two in sync.
    float q[4];
    orientCodeToQuat(17, q);
    EXPECT_NEAR(q[0], 0.5f, 1e-5f);
    EXPECT_NEAR(q[1], 0.5f, 1e-5f);
    EXPECT_NEAR(q[2], 0.5f, 1e-5f);
    EXPECT_NEAR(q[3], 0.5f, 1e-5f);
}

TEST(Orientation, CodeNames) {
    EXPECT_STREQ(orientCodeName(0), "+X");
    EXPECT_STREQ(orientCodeName(4), "-X");
    EXPECT_STREQ(orientCodeName(16), "+Z");
    EXPECT_STREQ(orientCodeName(17), "+Z r90");
    EXPECT_STREQ(orientCodeName(23), "-Z r270");
}

// ---------- Pad attitude quaternion (EKF init) ----------

namespace {
// Body-to-NED rotate via quaternion (scalar-first)
void quatRotate(const float q[4], const float v[3], float out[3])
{
    float R[9];
    orientQuatToMatrix(q, R);
    matVec(R, v, out);
}
}

TEST(Orientation, PadQuat_NoseUp_MatchesLegacyInit) {
    // Stationary nose-up FRD: specific force +g along body X.  The legacy
    // EKF init hardcoded (0.7071, 0, 0.7071, 0) — gravity init must agree.
    float q[4];
    quatFromAccelHeading(kG, 0.0f, 0.0f, 0.0f, q);
    EXPECT_NEAR(q[0], 0.70711f, 1e-3f);
    EXPECT_NEAR(q[1], 0.0f, 1e-3f);
    EXPECT_NEAR(q[2], 0.70711f, 1e-3f);
    EXPECT_NEAR(q[3], 0.0f, 1e-3f);
}

TEST(Orientation, PadQuat_GravityMapsToNedUp) {
    // For any stationary attitude, the init quaternion must take the
    // measured specific force to NED "up" (0, 0, -g).
    const float cases[][3] = {
        { kG, 0, 0 },           // nose up
        { -kG, 0, 0 },          // nose down
        { 0, 0, -kG },          // level (body Z down)
        { kG * 0.7071f, 0, -kG * 0.7071f },   // 45° pitch
        { kG * 0.5f, kG * 0.5f, -kG * 0.7071f },  // pitched + rolled
    };
    for (const auto& acc : cases)
    {
        float q[4], up[3];
        quatFromAccelHeading(acc[0], acc[1], acc[2], 0.0f, q);
        quatRotate(q, acc, up);
        EXPECT_NEAR(up[0], 0.0f, 0.02f) << acc[0] << "," << acc[1] << "," << acc[2];
        EXPECT_NEAR(up[1], 0.0f, 0.02f) << acc[0] << "," << acc[1] << "," << acc[2];
        EXPECT_NEAR(up[2], -kG, 0.02f) << acc[0] << "," << acc[1] << "," << acc[2];
    }
}

TEST(Orientation, PadQuat_HeadingOnlyYaw_WhenLevel) {
    // Level body with 90° heading: yaw quaternion about NED Z.
    float q[4];
    quatFromAccelHeading(0.0f, 0.0f, -kG, (float)M_PI / 2.0f, q);
    EXPECT_NEAR(q[0], 0.70711f, 1e-3f);
    EXPECT_NEAR(q[1], 0.0f, 1e-3f);
    EXPECT_NEAR(q[2], 0.0f, 1e-3f);
    EXPECT_NEAR(q[3], 0.70711f, 1e-3f);
}

TEST(Orientation, PadQuat_DegenerateInput_NoNaN) {
    float q[4];
    quatFromAccelHeading(0.0f, 0.0f, 0.0f, 0.0f, q);
    for (int i = 0; i < 4; i++) EXPECT_FALSE(std::isnan(q[i]));
}
