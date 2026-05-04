#include <gtest/gtest.h>
#include "TR_KinematicChecks.h"

// TR_KinematicChecks depends on millis() via the host shim.
// Tests must call setMockMillis() to advance time.

class KinematicChecksTest : public ::testing::Test {
protected:
    TR_KinematicChecks kc;

    void SetUp() override {
        setMockMillis(0);
        kc.reset();
    }

    // Helper: call kinematicChecks with typical stationary data
    void callStationary(float alt = 0.0f, float acc_mag = 9.81f, bool new_baro = true) {
        float pos[3] = {0, 0, alt};
        float vel[3] = {0, 0, 0};
        kc.kinematicChecks(alt, acc_mag, pos, vel, 0.0f, new_baro);
    }

    // Helper: call with flight-like data
    void callFlight(float alt, float acc_mag, float vel_u, float roll_rate = 0.0f,
                    float gps_alt = 0.0f, bool new_gps = false,
                    float pitch_rad = 1.57f, bool burnout = false, bool baro_lockout = false) {
        float pos[3] = {0, 0, alt};
        float vel[3] = {0, 0, vel_u};
        kc.kinematicChecks(alt, acc_mag, pos, vel, roll_rate, true, gps_alt, new_gps,
                           pitch_rad, burnout, baro_lockout);
    }
};

TEST_F(KinematicChecksTest, NoLaunch_BelowThreshold) {
    for (int i = 0; i < 200; i++) {
        setMockMillis(i);
        callStationary(0.0f, 15.0f); // below 20 m/s^2 threshold
    }
    EXPECT_FALSE(kc.launch_flag);
}

TEST_F(KinematicChecksTest, Launch_SustainedAccel) {
    // First, let the altitude KF see some upward motion
    // Feed altitude increasing + high accel for 60+ calls
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2); // 2ms steps
        float alt = 0.5f * i; // altitude climbing
        callFlight(alt, 25.0f, 10.0f); // high accel, positive velocity
    }
    EXPECT_TRUE(kc.launch_flag);
}

TEST_F(KinematicChecksTest, Launch_BriefSpike_NoTrigger) {
    // Only 10 samples of high accel -> should NOT trigger launch
    for (int i = 0; i < 10; i++) {
        setMockMillis(i * 2);
        callFlight(0.0f, 25.0f, 0.0f);
    }
    // Drop back below threshold
    for (int i = 0; i < 200; i++) {
        setMockMillis(20 + i * 2);
        callStationary(0.0f, 5.0f);
    }
    EXPECT_FALSE(kc.launch_flag);
}

TEST_F(KinematicChecksTest, MaxAltitude_SpikeRejection) {
    // Establish a max altitude of 100m
    setMockMillis(0);
    callFlight(100.0f, 5.0f, 0.0f);
    EXPECT_NEAR(kc.max_altitude, 100.0f, 1.0f);

    // Single spike to 500m (>50m from current max) -> should be rejected
    setMockMillis(2);
    callFlight(500.0f, 5.0f, 0.0f);
    EXPECT_LT(kc.max_altitude, 200.0f); // spike rejected

    // Normal increase within 50m window
    setMockMillis(4);
    callFlight(120.0f, 5.0f, 0.0f);
    EXPECT_NEAR(kc.max_altitude, 120.0f, 1.0f);
}

TEST_F(KinematicChecksTest, Apogee_GatedOnBurnout) {
    // Force launch
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);

    // Now descending, but burnout NOT detected
    for (int i = 0; i < 50; i++) {
        setMockMillis(160 + i * 2);
        callFlight(100.0f - i, 5.0f, -10.0f, 0.0f, 0.0f, false, -0.2f, false);
    }
    EXPECT_FALSE(kc.apogee_flag); // gated on burnout
}

TEST_F(KinematicChecksTest, Apogee_WithBurnout_DetectsApogee) {
    // Force launch
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);

    // Ascending phase
    for (int i = 0; i < 50; i++) {
        setMockMillis(160 + i * 2);
        callFlight(80.0f + i, 5.0f, 10.0f, 0.0f, 0.0f, false, 1.0f, true);
    }

    // Descending with burnout detected
    for (int i = 0; i < 50; i++) {
        setMockMillis(260 + i * 2);
        float alt = 130.0f - i * 2;
        // EKF velocity negative, altitude decreasing, pitch below horizontal
        callFlight(alt, 5.0f, -10.0f, 0.0f, alt, true, -0.2f, true);
    }
    EXPECT_TRUE(kc.apogee_flag);
}

TEST_F(KinematicChecksTest, Landing_StableAlt) {
    // Force launch and establish max_altitude > 15m
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    ASSERT_GT(kc.max_altitude, 15.0f);

    // Now simulate landed: alt < 50, stable, low roll rate
    // Landing needs 5 consecutive 1-second checks
    for (int second = 0; second < 7; second++) {
        uint32_t base = 1000 + second * 1000;
        // Call many times within each second (landing_check_dt = 1000ms)
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(5.0f, 9.81f, 0.0f, 0.1f); // stable at 5m, low roll rate
        }
    }
    EXPECT_TRUE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_NotPremature) {
    // If max_altitude was never > 15m, landing should NOT trigger
    for (int second = 0; second < 10; second++) {
        uint32_t base = second * 1000;
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(5.0f, 9.81f, 0.0f, 0.1f);
        }
    }
    EXPECT_FALSE(kc.alt_landed_flag); // max_altitude < 15
}

TEST_F(KinematicChecksTest, AltKF_ConvergesToMeasurement) {
    // Feed constant altitude measurements
    for (int i = 0; i < 500; i++) {
        setMockMillis(i * 2);
        callStationary(100.0f, 9.81f);
    }
    EXPECT_NEAR(kc.alt_est, 100.0f, 1.0f);
}

TEST_F(KinematicChecksTest, AltKF_TracksRamp) {
    // Feed linearly increasing altitude
    for (int i = 0; i < 500; i++) {
        setMockMillis(i * 2);
        float alt = float(i) * 0.1f; // 50 m/s altitude rate
        callStationary(alt, 9.81f);
    }
    // The filtered rate should be positive
    EXPECT_GT(kc.d_alt_est_, 0.0f);
}

TEST_F(KinematicChecksTest, Reset_ClearsAll) {
    // Force launch
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);

    kc.reset();

    EXPECT_FALSE(kc.launch_flag);
    EXPECT_FALSE(kc.alt_landed_flag);
    EXPECT_FALSE(kc.alt_apogee_flag);
    EXPECT_FALSE(kc.vel_u_apogee_flag);
    EXPECT_FALSE(kc.gps_apogee_flag);
    EXPECT_FALSE(kc.pitch_apogee_flag);
    EXPECT_FALSE(kc.apogee_flag);
    EXPECT_FLOAT_EQ(kc.max_altitude, 0.0f);
    EXPECT_FLOAT_EQ(kc.max_speed, 0.0f);
}

// ── Tests for issue #113: relaxed gyro threshold + impact fast path ──

TEST_F(KinematicChecksTest, Landing_RollRate15dps_StillPasses) {
    // The relaxed 20 dps threshold accepts steady 15 dps wobble (e.g. wind
    // on a landed rocket). Old 2 dps threshold would fail this case.
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    ASSERT_GT(kc.max_altitude, 15.0f);

    for (int second = 0; second < 7; second++) {
        uint32_t base = 1000 + second * 1000;
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(5.0f, 9.81f, 0.0f, 15.0f);
        }
    }
    EXPECT_TRUE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_RollRate25dps_DoesNotTrigger) {
    // 25 dps exceeds the 20 dps threshold -- still rejected by slow path.
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);

    for (int second = 0; second < 7; second++) {
        uint32_t base = 1000 + second * 1000;
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(5.0f, 9.81f, 0.0f, 25.0f);
        }
    }
    EXPECT_FALSE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_FastPath_ImpactTriggers) {
    // apogee + low altitude + >15g for 5 consecutive samples -> landed
    kc.apogee_flag = true;
    for (int i = 0; i < 10; i++) {
        setMockMillis(1000 + i);
        callFlight(5.0f, 200.0f, -10.0f, 0.0f);  // ~20g, 5m alt
    }
    EXPECT_TRUE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_FastPath_GatedOnApogee) {
    // Same impact-magnitude accel pre-apogee -> NO trigger (e.g. boost spike)
    for (int i = 0; i < 50; i++) {
        setMockMillis(1000 + i);
        callFlight(5.0f, 200.0f, 10.0f, 0.0f);
    }
    EXPECT_FALSE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_FastPath_GatedOnAltitude) {
    // Apogee + impact-magnitude accel at altitude (e.g. ejection at apogee)
    // -> NO trigger because pressure_altitude > 20m
    kc.apogee_flag = true;
    for (int i = 0; i < 50; i++) {
        setMockMillis(1000 + i);
        callFlight(100.0f, 200.0f, -10.0f, 0.0f);
    }
    EXPECT_FALSE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_FastPath_BelowG_NoTrigger) {
    // Apogee + low altitude but accel below 15 g threshold -> NO trigger
    kc.apogee_flag = true;
    for (int i = 0; i < 50; i++) {
        setMockMillis(1000 + i);
        callFlight(5.0f, 100.0f, -10.0f, 0.0f);  // ~10g, below threshold
    }
    EXPECT_FALSE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_FastPath_BriefSpike_CounterResets) {
    // Single high-g sample then back to quiet -> count resets, no trigger.
    // Verifies the noise-rejection behavior of the consecutive-sample gate.
    kc.apogee_flag = true;
    setMockMillis(1000);
    callFlight(5.0f, 200.0f, -10.0f, 0.0f);  // 1 sample at impact magnitude
    for (int i = 0; i < 100; i++) {
        setMockMillis(1001 + i);
        callFlight(5.0f, 9.81f, 0.0f, 0.0f);  // back to gravity floor
    }
    EXPECT_FALSE(kc.alt_landed_flag);
}
