#include <gtest/gtest.h>
#include "TR_ControlMixer.h"
#include <cmath>

class ControlMixerTest : public ::testing::Test {
protected:
    TR_ControlMixer mixer;
    static constexpr float DT = 0.002f; // 500 Hz
    static constexpr float MAX_FIN = 15.0f;

    void SetUp() override {
        mixer.configure(0.04f, 0.001f, 0.0003f,  // pitch P/I/D
                        0.04f, 0.001f, 0.0003f,  // yaw P/I/D
                        MAX_FIN, 95.0f, 30.0f);  // max_fin, v_ref, v_min
    }

    // Helper: run one update and get deflections
    void updateAndGet(float pitch_cmd, float yaw_cmd,
                      float pitch_actual, float yaw_actual,
                      float pitch_rate, float yaw_rate,
                      float roll_cmd, float speed,
                      float defl[4]) {
        mixer.update(pitch_cmd, yaw_cmd, pitch_actual, yaw_actual,
                     pitch_rate, yaw_rate, roll_cmd, speed,
                     1.0f, 1.0f, DT); // kp_angle = 1.0
        mixer.getFinDeflections(defl);
    }
};

TEST_F(ControlMixerTest, ZeroCommand_ZeroDeflections) {
    float d[4];
    // First call (init)
    updateAndGet(0, 0, 0, 0, 0, 0, 0, 95.0f, d);
    // Second call
    updateAndGet(0, 0, 0, 0, 0, 0, 0, 95.0f, d);

    for (int i = 0; i < 4; i++) {
        EXPECT_NEAR(d[i], 0.0f, 0.01f);
    }
}

TEST_F(ControlMixerTest, PurePitch_CorrectMixing) {
    // Pitch error -> pitch rate cmd -> PID -> fin deflection
    // PITCH_MIX = [+1, 0, -1, 0]
    float d[4];
    // Init call
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 95.0f, d);
    // Second call with pitch error
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 95.0f, d);

    // Top and bottom fins should have opposite signs
    // Right and left fins should be near zero (yaw + roll both 0)
    EXPECT_GT(std::abs(d[0]), 0.0f);  // top
    EXPECT_NEAR(d[1], 0.0f, 0.01f);  // right (yaw only)
    EXPECT_NEAR(d[0], -d[2], 0.01f); // top = -bottom
    EXPECT_NEAR(d[3], 0.0f, 0.01f);  // left (yaw only)
}

TEST_F(ControlMixerTest, PureYaw_CorrectMixing) {
    // YAW_MIX = [0, +1, 0, -1]
    float d[4];
    updateAndGet(0, 10.0f, 0, 0, 0, 0, 0, 95.0f, d);
    updateAndGet(0, 10.0f, 0, 0, 0, 0, 0, 95.0f, d);

    EXPECT_NEAR(d[0], 0.0f, 0.01f);  // top (pitch only)
    EXPECT_GT(std::abs(d[1]), 0.0f);  // right
    EXPECT_NEAR(d[2], 0.0f, 0.01f);  // bottom (pitch only)
    EXPECT_NEAR(d[1], -d[3], 0.01f); // right = -left
}

TEST_F(ControlMixerTest, PureRoll_AllSameSign) {
    // ROLL_MIX = [+1, +1, +1, +1]
    float d[4];
    // Roll command passed directly (not through PID)
    updateAndGet(0, 0, 0, 0, 0, 0, 5.0f, 95.0f, d);

    // All fins should deflect same direction for roll
    for (int i = 0; i < 4; i++) {
        EXPECT_NEAR(d[i], 5.0f, 0.01f);
    }
}

TEST_F(ControlMixerTest, FinDeflection_Clamped) {
    float d[4];
    // Large roll command that would exceed max_fin_deg
    updateAndGet(0, 0, 0, 0, 0, 0, 100.0f, 95.0f, d);

    for (int i = 0; i < 4; i++) {
        EXPECT_LE(d[i], MAX_FIN);
        EXPECT_GE(d[i], -MAX_FIN);
    }
}

TEST_F(ControlMixerTest, GainSchedule_HighSpeed) {
    mixer.enableGainSchedule(95.0f, 30.0f);

    float d_ref[4], d_fast[4];
    // At reference speed
    mixer.reset();
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 95.0f, d_ref);
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 95.0f, d_ref);

    // At 2x reference speed -> gains should be ~1/4
    mixer.reset();
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 190.0f, d_fast);
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 190.0f, d_fast);

    // Fast speed -> smaller deflection
    EXPECT_LT(std::abs(d_fast[0]), std::abs(d_ref[0]));
}

TEST_F(ControlMixerTest, GainSchedule_LowSpeed) {
    mixer.enableGainSchedule(95.0f, 30.0f);

    float d_ref[4], d_slow[4];
    // At reference speed
    mixer.reset();
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 95.0f, d_ref);
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 95.0f, d_ref);

    // At half reference speed -> gains should be ~4x (capped at 3x)
    mixer.reset();
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 47.5f, d_slow);
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 47.5f, d_slow);

    // Slow speed -> larger deflection
    EXPECT_GT(std::abs(d_slow[0]), std::abs(d_ref[0]));
}

TEST_F(ControlMixerTest, GainSchedule_BelowVmin) {
    mixer.enableGainSchedule(95.0f, 30.0f);

    float d_vmin[4], d_zero[4];
    // At v_min
    mixer.reset();
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 30.0f, d_vmin);
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 30.0f, d_vmin);

    // At 0 speed -> should use v_min floor, same result
    mixer.reset();
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 0.0f, d_zero);
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 0.0f, d_zero);

    EXPECT_NEAR(d_vmin[0], d_zero[0], 0.01f);
}

TEST_F(ControlMixerTest, DisableGainSchedule_RestoresBase) {
    mixer.enableGainSchedule(95.0f, 30.0f);

    // Run with low speed (gains boosted)
    float d_boosted[4];
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 30.0f, d_boosted);
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 30.0f, d_boosted);

    // Disable gain scheduling
    mixer.disableGainSchedule();

    // Run same input at same speed - now gains should be base (not boosted)
    mixer.reset();
    float d_base[4];
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 30.0f, d_base);
    updateAndGet(10.0f, 0, 0, 0, 0, 0, 0, 30.0f, d_base);

    // Base gains at 30 m/s = smaller deflection than boosted
    EXPECT_LT(std::abs(d_base[0]), std::abs(d_boosted[0]));
}

TEST_F(ControlMixerTest, Reset_ClearsAllState) {
    // Run some updates to accumulate PID state
    float d[4];
    for (int i = 0; i < 100; i++) {
        updateAndGet(10.0f, 5.0f, 0, 0, 0, 0, 3.0f, 95.0f, d);
    }

    mixer.reset();
    mixer.getFinDeflections(d);

    for (int i = 0; i < 4; i++) {
        EXPECT_FLOAT_EQ(d[i], 0.0f);
    }
    EXPECT_FLOAT_EQ(mixer.getPitchFinCmd(), 0.0f);
    EXPECT_FLOAT_EQ(mixer.getYawFinCmd(), 0.0f);
}

// ─── Configurable fin layout (servo→azimuth + reverse) ───────────────────────

// REGRESSION: with no setFinLayout call (default {0,90,180,270} + mask 0),
// mixToFins must reproduce the legacy hardcoded "+" mix exactly:
//   d0=+pitch+roll  d1=+yaw+roll  d2=-pitch+roll  d3=-yaw+roll
TEST_F(ControlMixerTest, MixToFins_DefaultReproducesHardcodedPlus) {
    float d[4];
    mixer.mixToFins(0.0f, 2.0f, 0.0f, MAX_FIN, d);          // pure pitch
    EXPECT_NEAR(d[0],  2.0f, 1e-4f);
    EXPECT_NEAR(d[1],  0.0f, 1e-4f);
    EXPECT_NEAR(d[2], -2.0f, 1e-4f);
    EXPECT_NEAR(d[3],  0.0f, 1e-4f);

    mixer.mixToFins(0.0f, 0.0f, 3.0f, MAX_FIN, d);          // pure yaw
    EXPECT_NEAR(d[0],  0.0f, 1e-4f);
    EXPECT_NEAR(d[1],  3.0f, 1e-4f);
    EXPECT_NEAR(d[2],  0.0f, 1e-4f);
    EXPECT_NEAR(d[3], -3.0f, 1e-4f);

    mixer.mixToFins(4.0f, 0.0f, 0.0f, MAX_FIN, d);          // pure roll → common
    for (int i = 0; i < 4; i++) EXPECT_NEAR(d[i], 4.0f, 1e-4f);

    mixer.mixToFins(1.0f, 2.0f, 3.0f, MAX_FIN, d);          // combined
    EXPECT_NEAR(d[0],  2.0f + 1.0f, 1e-4f);
    EXPECT_NEAR(d[1],  3.0f + 1.0f, 1e-4f);
    EXPECT_NEAR(d[2], -2.0f + 1.0f, 1e-4f);
    EXPECT_NEAR(d[3], -3.0f + 1.0f, 1e-4f);
}

TEST_F(ControlMixerTest, SetFinLayout_DefaultAzimuthsMatchHardcode) {
    const float az[4] = {0.0f, 90.0f, 180.0f, 270.0f};
    mixer.setFinLayout(az, 0);
    float d[4];
    mixer.mixToFins(1.0f, 2.0f, 3.0f, MAX_FIN, d);
    EXPECT_NEAR(d[0],  2.0f + 1.0f, 1e-4f);
    EXPECT_NEAR(d[1],  3.0f + 1.0f, 1e-4f);
    EXPECT_NEAR(d[2], -2.0f + 1.0f, 1e-4f);
    EXPECT_NEAR(d[3], -3.0f + 1.0f, 1e-4f);
}

// "×" layout: every fin shares pitch AND yaw at cos/sin(45°)=±0.707.
TEST_F(ControlMixerTest, SetFinLayout_CrossSharesPitchAndYaw) {
    const float az[4] = {45.0f, 135.0f, 225.0f, 315.0f};
    mixer.setFinLayout(az, 0);
    const float k = 0.70710678f;
    float d[4];
    mixer.mixToFins(0.0f, 1.0f, 0.0f, MAX_FIN, d);          // pitch → cos(az)
    EXPECT_NEAR(d[0],  k, 1e-4f);
    EXPECT_NEAR(d[1], -k, 1e-4f);
    EXPECT_NEAR(d[2], -k, 1e-4f);
    EXPECT_NEAR(d[3],  k, 1e-4f);

    mixer.mixToFins(0.0f, 0.0f, 1.0f, MAX_FIN, d);          // yaw → sin(az)
    EXPECT_NEAR(d[0],  k, 1e-4f);
    EXPECT_NEAR(d[1],  k, 1e-4f);
    EXPECT_NEAR(d[2], -k, 1e-4f);
    EXPECT_NEAR(d[3], -k, 1e-4f);

    mixer.mixToFins(2.0f, 0.0f, 0.0f, MAX_FIN, d);          // roll still common
    for (int i = 0; i < 4; i++) EXPECT_NEAR(d[i], 2.0f, 1e-4f);
}

// A reverse bit negates that servo's whole deflection (mirrored mount).
TEST_F(ControlMixerTest, SetFinLayout_ReverseNegatesFin) {
    const float az[4] = {0.0f, 90.0f, 180.0f, 270.0f};
    mixer.setFinLayout(az, 0x02);                            // reverse servo 1
    float d[4];
    mixer.mixToFins(1.0f, 2.0f, 3.0f, MAX_FIN, d);
    EXPECT_NEAR(d[0],   2.0f + 1.0f,  1e-4f);                // unchanged
    EXPECT_NEAR(d[1], -(3.0f + 1.0f), 1e-4f);                // fully negated
    EXPECT_NEAR(d[2],  -2.0f + 1.0f,  1e-4f);                // unchanged
    EXPECT_NEAR(d[3],  -3.0f + 1.0f,  1e-4f);                // unchanged
    EXPECT_EQ(mixer.getFinReverseMask(), 0x02);
    EXPECT_NEAR(mixer.getFinAzimuthDeg(1), 90.0f, 1e-4f);
}

TEST_F(ControlMixerTest, MixToFins_Clamps) {
    float d[4];
    mixer.mixToFins(100.0f, 100.0f, 100.0f, MAX_FIN, d);
    for (int i = 0; i < 4; i++) {
        EXPECT_LE(d[i],  MAX_FIN);
        EXPECT_GE(d[i], -MAX_FIN);
    }
}
