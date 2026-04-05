#include <gtest/gtest.h>
#include "TR_GuidancePN.h"
#include <cmath>

class GuidancePNTest : public ::testing::Test {
protected:
    TR_GuidancePN guid;
    static constexpr float TARGET_ALT = 600.0f;
    static constexpr float DT = 0.002f;

    void SetUp() override {
        guid.configure(3.0f, 20.0f, TARGET_ALT);
    }
};

TEST_F(GuidancePNTest, DirectlyBelowTarget_NearZeroLateral) {
    // Rocket at (0, 0, 300) moving straight up toward target at (0, 0, 600)
    float pos_enu[3] = {0.0f, 0.0f, 300.0f};
    // vel_ned: [vN, vE, vD] - moving up means vD < 0
    float vel_ned[3] = {0.0f, 0.0f, -50.0f}; // 50 m/s upward

    guid.update(pos_enu, vel_ned, DT);

    EXPECT_TRUE(guid.isActive());
    // Lateral commands should be near zero when directly below target
    EXPECT_NEAR(guid.getAccelEastCmd(), 0.0f, 0.5f);
    EXPECT_NEAR(guid.getAccelNorthCmd(), 0.0f, 0.5f);
    EXPECT_NEAR(guid.getLateralOffset(), 0.0f, 0.1f);
}

TEST_F(GuidancePNTest, OffsetFromPad_LateralCorrection) {
    // Rocket 50m east of pad, moving up
    float pos_enu[3] = {50.0f, 0.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, -50.0f}; // moving up

    guid.update(pos_enu, vel_ned, DT);

    EXPECT_TRUE(guid.isActive());
    // East accel should be negative (pushing back toward pad)
    EXPECT_LT(guid.getAccelEastCmd(), 0.0f);
    EXPECT_NEAR(guid.getLateralOffset(), 50.0f, 0.1f);
}

TEST_F(GuidancePNTest, AccelClamping) {
    // Large offset -> should clamp to max_accel_mps2 (20 m/s^2)
    float pos_enu[3] = {200.0f, 200.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, -50.0f};

    guid.update(pos_enu, vel_ned, DT);

    float a_mag = std::sqrt(guid.getAccelEastCmd() * guid.getAccelEastCmd() +
                            guid.getAccelNorthCmd() * guid.getAccelNorthCmd() +
                            guid.getAccelUpCmd() * guid.getAccelUpCmd());
    EXPECT_LE(a_mag, 20.0f + 0.01f);
}

TEST_F(GuidancePNTest, CloseRange_Deactivates) {
    // Rocket very close to target -> should deactivate (range < 1m)
    float pos_enu[3] = {0.0f, 0.0f, TARGET_ALT - 0.5f};
    float vel_ned[3] = {0.0f, 0.0f, -1.0f};

    bool active = guid.update(pos_enu, vel_ned, DT);

    EXPECT_FALSE(active);
    EXPECT_FALSE(guid.isActive());
    EXPECT_FLOAT_EQ(guid.getAccelEastCmd(), 0.0f);
    EXPECT_FLOAT_EQ(guid.getAccelNorthCmd(), 0.0f);
    EXPECT_FLOAT_EQ(guid.getAccelUpCmd(), 0.0f);
}

TEST_F(GuidancePNTest, ClosingVelocity_Approaching) {
    // Moving toward target -> positive closing velocity
    float pos_enu[3] = {0.0f, 0.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, -50.0f}; // up toward target

    guid.update(pos_enu, vel_ned, DT);

    EXPECT_GT(guid.getClosingVelocity(), 0.0f);
    EXPECT_FALSE(guid.isCpaReached());
}

TEST_F(GuidancePNTest, CPA_FlagsWhenReceding) {
    // Moving away from target -> negative closing velocity -> CPA reached
    float pos_enu[3] = {0.0f, 0.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, 50.0f}; // downward, away from target

    guid.update(pos_enu, vel_ned, DT);

    EXPECT_LE(guid.getClosingVelocity(), 0.0f);
    EXPECT_TRUE(guid.isCpaReached());
}

TEST_F(GuidancePNTest, Range_Computed) {
    float pos_enu[3] = {30.0f, 40.0f, 100.0f};
    float vel_ned[3] = {0.0f, 0.0f, -10.0f};

    guid.update(pos_enu, vel_ned, DT);

    // Range to target at (0,0,600): sqrt(30^2 + 40^2 + 500^2) = sqrt(900+1600+250000) ~ 502.5
    float expected_range = std::sqrt(30.0f*30.0f + 40.0f*40.0f + 500.0f*500.0f);
    EXPECT_NEAR(guid.getRange(), expected_range, 0.1f);
}

TEST_F(GuidancePNTest, Reset_ClearsAll) {
    float pos_enu[3] = {50.0f, 50.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, -50.0f};
    guid.update(pos_enu, vel_ned, DT);

    guid.reset();

    EXPECT_FLOAT_EQ(guid.getAccelEastCmd(), 0.0f);
    EXPECT_FLOAT_EQ(guid.getAccelNorthCmd(), 0.0f);
    EXPECT_FLOAT_EQ(guid.getAccelUpCmd(), 0.0f);
    EXPECT_FLOAT_EQ(guid.getRange(), 0.0f);
    EXPECT_FLOAT_EQ(guid.getLateralOffset(), 0.0f);
    EXPECT_FALSE(guid.isActive());
    EXPECT_FALSE(guid.isCpaReached());
}

TEST_F(GuidancePNTest, Configure_UpdatesParams) {
    guid.configure(5.0f, 30.0f, 800.0f);

    // Verify new target altitude is used
    float pos_enu[3] = {0.0f, 0.0f, 799.5f};
    float vel_ned[3] = {0.0f, 0.0f, -1.0f};

    // Should deactivate (range < 1m from new 800m target)
    bool active = guid.update(pos_enu, vel_ned, DT);
    EXPECT_FALSE(active);
}
