// Tests for TR_ServoControl (ledc_mult) — the roll-rate / roll-angle
// controller that drives the fin servos.
//
// The core regression here is #372: controlAngle() used to persist its
// outer-loop rate command into pid_setpoint, so after an ANGLE tick the
// rate-null paths (control / controlWithGainSchedule) held the residual
// rate command instead of nulling to the configured setpoint.  Post-v4
// roll profiles are always ANGLE once past the first waypoint, so the
// transition back to rate-null happens exactly on the #265 EKF-health
// fallback — the safety path — which made the stale setpoint worse.
//
// The LEDC calls are no-ops on the host (host_shim/driver/ledc.h); the
// controller math runs exactly as on the rocket.  micros() is the shim's
// mock clock, advanced explicitly per tick.

#include <gtest/gtest.h>
#include <Arduino.h>  // host shim: setMockMicros()
#include "TR_ServoControl_ledc_mult.h"

namespace {

constexpr float KP      = 1.0f;   // P-only inner loop: output == rate error
constexpr float KI      = 0.0f;
constexpr float KD      = 0.0f;
constexpr float MIN_CMD = -60.0f;
constexpr float MAX_CMD = 60.0f;

constexpr uint32_t TICK_US = 2000;  // 500 Hz control loop

class ServoControlTest : public ::testing::Test {
protected:
    TR_ServoControl servo{1, 2, 3, 4,      // pins
                          0, 0, 0, 0,      // bias us
                          50, 1000, 2000,  // hz, min/max us
                          KP, KI, KD,
                          MIN_CMD, MAX_CMD};

    uint32_t now_us_ = 0;

    void SetUp() override {
        setMockMicros(0);
        servo.begin();
        // TR_PID returns 0 on its first call (dt bootstrap) — burn it so
        // every test asserts on steady-state ticks.
        tick();
        servo.control(0.0f);
    }

    // Advance the mock clock one control period.
    void tick() {
        now_us_ += TICK_US;
        setMockMicros(now_us_);
    }
};

// ---------- controlAngle cascade behavior (unchanged by #372 fix) ----------

TEST_F(ServoControlTest, ControlAngleTracksProportionalRateCommand) {
    // Outer loop: rate_cmd = kp_angle * (target - actual) = 2 * 10 = 20 dps.
    // Inner loop (P-only, measured rate 0): output = -rate_cmd - 0 = -20.
    tick();
    servo.controlAngle(10.0f, 0.0f, 0.0f, 50.0f,
                       /*kp_angle=*/2.0f, /*rate_cap_dps=*/60.0f);
    EXPECT_NEAR(servo.getRollCmdDeg(), -20.0f, 1e-4f);
}

TEST_F(ServoControlTest, ControlAngleCapsOuterLoopRateCommand) {
    // 180 deg error at kp_angle=4 would demand 720 dps; the cap holds it
    // at 60, so the inner loop sees setpoint -60.
    tick();
    servo.controlAngle(180.0f, 0.0f, 0.0f, 50.0f, 4.0f, 60.0f);
    EXPECT_NEAR(servo.getRollCmdDeg(), -60.0f, 1e-4f);
}

TEST_F(ServoControlTest, ControlAngleWrapsAngleError) {
    // target -170, actual +170: raw error -340 wraps to +20 -> rate_cmd
    // +40 at kp_angle=2 -> inner-loop output -40.
    tick();
    servo.controlAngle(-170.0f, 170.0f, 0.0f, 50.0f, 2.0f, 60.0f);
    EXPECT_NEAR(servo.getRollCmdDeg(), -40.0f, 1e-4f);
}

// ---------- #372 regression: no stale setpoint after an ANGLE tick ----------

TEST_F(ServoControlTest, RateNullAfterAngleTickNullsToConfiguredSetpoint) {
    servo.setSetpoint(0.0f);

    // Saturating ANGLE tick: outer loop commands the full ±60 dps cap.
    tick();
    servo.controlAngle(180.0f, 0.0f, 0.0f, 50.0f, 4.0f, 60.0f);
    ASSERT_NEAR(servo.getRollCmdDeg(), -60.0f, 1e-4f);

    // Next tick drops to rate-null (profile segment change / #265 EKF-health
    // fallback).  Measured rate is already 0, so a correct null commands 0.
    // Pre-#372-fix this held the stale -60 dps setpoint -> -60 output.
    tick();
    servo.control(0.0f);
    EXPECT_NEAR(servo.getRollCmdDeg(), 0.0f, 1e-4f);
}

TEST_F(ServoControlTest, GainScheduledRateNullAfterAngleTickAlsoNulls) {
    servo.setSetpoint(0.0f);

    tick();
    servo.controlAngle(180.0f, 0.0f, 0.0f, 50.0f, 4.0f, 60.0f);
    ASSERT_NEAR(servo.getRollCmdDeg(), -60.0f, 1e-4f);

    // The FC's gain-scheduled rate-null path (controlWithGainSchedule) must
    // null to the configured setpoint too — schedule disabled, gains as-is.
    tick();
    servo.controlWithGainSchedule(0.0f, 40.0f);
    EXPECT_NEAR(servo.getRollCmdDeg(), 0.0f, 1e-4f);
}

TEST_F(ServoControlTest, SetSetpointStillGovernsRateNull) {
    // The one legitimate way to bias the rate loop: setSetpoint().  An
    // intervening ANGLE tick must not clobber it.
    servo.setSetpoint(5.0f);

    tick();
    servo.controlAngle(10.0f, 0.0f, 0.0f, 50.0f, 2.0f, 60.0f);

    tick();
    servo.control(0.0f);
    // P-only: output = setpoint - measured = 5.
    EXPECT_NEAR(servo.getRollCmdDeg(), 5.0f, 1e-4f);
}

}  // namespace
