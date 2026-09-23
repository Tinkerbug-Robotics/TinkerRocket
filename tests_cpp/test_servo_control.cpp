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
// The LEDC calls drive nothing on the host but are recorded
// (host_shim/driver/ledc.h: hostLedcLog); the controller math runs exactly as
// on the rocket.  micros() is the shim's mock clock, advanced explicitly per
// tick.

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
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

// ---------- roll-reverse mask on the roll-only drive ----------
//
// "Reverse roll" (FinConfigData::roll_reverse_mask) used to reach only
// TR_ControlMixer, which mixes the guided and ground-test paths.  Roll-only
// control drives every fin from THIS class's single PID output, so the toggle
// had no path to it and a rocket that rolled the wrong way in roll-only flight
// could not be corrected from the app.  These lock the mask into that drive.

// Pulse for a commanded fin angle under the fixture's calibration: fin cal
// defaults to the command clamp (±60 deg) mapped onto 1000..2000 us, biases 0.
static int expectedPulseUs(float fin_deg) {
    return 1000 + static_cast<int>(((fin_deg + 60.0f) / 120.0f) * 1000.0f);
}

TEST_F(ServoControlTest, RollReverseMaskNegatesOnlyTheMaskedServos) {
    servo.setRollReverseMask(0b1010);  // servos 2 and 4 linked the other way

    tick();
    servo.control(-20.0f);             // P-only: cmd = -(-20) = +20 deg
    ASSERT_NEAR(servo.getRollCmdDeg(), 20.0f, 1e-4f);

    EXPECT_EQ(servo.getServoPulseUs(0), expectedPulseUs( 20.0f));
    EXPECT_EQ(servo.getServoPulseUs(1), expectedPulseUs(-20.0f));
    EXPECT_EQ(servo.getServoPulseUs(2), expectedPulseUs( 20.0f));
    EXPECT_EQ(servo.getServoPulseUs(3), expectedPulseUs(-20.0f));
    EXPECT_EQ(servo.getRollReverseMask(), 0b1010);
}

TEST_F(ServoControlTest, RollReverseMaskAllBitsReversesRollGlobally) {
    // The usual airframe case: a mirrored horn/fin convention flips all four
    // together, so setting every bit is what reverses roll direction outright.
    tick();
    servo.control(-20.0f);
    const int forward[4] = { servo.getServoPulseUs(0), servo.getServoPulseUs(1),
                             servo.getServoPulseUs(2), servo.getServoPulseUs(3) };

    servo.setRollReverseMask(0b1111);
    tick();
    servo.control(-20.0f);
    for (int i = 0; i < 4; ++i) {
        // Same magnitude of deflection, opposite side of centre.
        EXPECT_EQ(servo.getServoPulseUs(i), expectedPulseUs(-20.0f));
        EXPECT_NE(servo.getServoPulseUs(i), forward[i]);
    }
}

TEST_F(ServoControlTest, RollReverseMaskZeroDrivesAllServosAlike) {
    // Regression guard: the drive was a single broadcast pulse before the mask
    // existed.  An unconfigured airframe (mask 0) must be bit-identical to it,
    // including the legacy servo-1 telemetry reading.
    tick();
    servo.control(-20.0f);

    const int expected = expectedPulseUs(20.0f);
    for (int i = 0; i < 4; ++i) EXPECT_EQ(servo.getServoPulseUs(i), expected);
    EXPECT_EQ(servo.getRollCmdUs(), expected);
    EXPECT_EQ(servo.getRollReverseMask(), 0);
}



// ---------- #1115: the angle-error wrap must be TOTAL ----------
//
// controlAngle()'s wrap used to be two unbounded while-loops.  `a -= 360.0f`
// makes no progress once |a| >= 2^33 (ulp exceeds 720, so the subtraction
// rounds straight back), and none at all for ±inf.  This runs on the FC's
// highest-priority ~1 kHz flight task under a 5 s panic-on-expiry task WDT,
// so a single bad input wedged the vehicle and rebooted it mid-flight —
// then again after every snapshot recovery, for the rest of the flight.
//
// BOTH arguments can supply one.  target_roll_deg came from an unvalidated
// roll-profile waypoint (fixed at its own acceptance points, RollProfileGate.h);
// actual_roll_deg is an atan2f over the EKF quaternion, which no gate here
// owns.  So this component must be safe on its own, and these tests pin that
// independently of the profile gate.
//
// Termination is pinned the only way gtest can: a regression does not fail
// these tests, it hangs the suite.

TEST_F(ServoControlTest, ControlAngleTerminatesOnAHugeTarget) {
    // Every value made zero progress in the old loop (1e8 merely took ~278k
    // iterations inside a 1 ms tick budget).
    const float huge[] = {1e8f, 8.59e9f, 1e10f, 1e30f, 3.4e38f,
                          -1e8f, -8.59e9f, -1e10f, -1e30f, -3.4e38f};
    for (float t : huge) {
        tick();
        servo.controlAngle(t, 0.0f, 0.0f, 50.0f, 2.0f, 60.0f);
        const float cmd = servo.getRollCmdDeg();
        EXPECT_TRUE(std::isfinite(cmd)) << t;
        EXPECT_GE(cmd, MIN_CMD) << t;
        EXPECT_LE(cmd, MAX_CMD) << t;
    }
}

TEST_F(ServoControlTest, ControlAngleNonFiniteTargetCommandsNullRate) {
    // A non-finite error degrades to zero error -> zero rate command, which is
    // exactly what the controller does when the angle loop is not engaged.
    // The alternative — the old NaN path, which never hung — drove the inner
    // PID with a NaN setpoint for the whole flight.
    const float inf = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (float t : {inf, -inf, nan}) {
        tick();
        servo.controlAngle(t, 0.0f, /*roll_rate_dps=*/0.0f, 50.0f, 2.0f, 60.0f);
        EXPECT_TRUE(std::isfinite(servo.getRollCmdDeg()));
        EXPECT_NEAR(servo.getRollCmdDeg(), 0.0f, 1e-4f);
    }
}

TEST_F(ServoControlTest, ControlAngleNonFiniteMeasuredRollCommandsNullRate) {
    // The other input: actual_roll_deg is atan2f over the EKF quaternion, so a
    // NaN attitude reaches this function with a perfectly valid target.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    tick();
    servo.controlAngle(45.0f, nan, 0.0f, 50.0f, 2.0f, 60.0f);
    EXPECT_TRUE(std::isfinite(servo.getRollCmdDeg()));
    EXPECT_NEAR(servo.getRollCmdDeg(), 0.0f, 1e-4f);
}

TEST_F(ServoControlTest, ControlAngleStillWrapsLargeButOrdinaryErrors) {
    // The wrap has to keep MEANING, not just terminate: a target the profile
    // gate accepts at its bound (±720) must still fly the shortest arc.
    // 710 - 0 = 710 -> -10, so rate_cmd = 2 * -10 = -20 -> output +20.
    tick();
    servo.controlAngle(710.0f, 0.0f, 0.0f, 50.0f, 2.0f, 60.0f);
    EXPECT_NEAR(servo.getRollCmdDeg(), 20.0f, 1e-3f);
}

}  // namespace

// ── #1137 item 2: a degenerate fin calibration must never be stored ──
//
// usFromFinDeg() has a `span_deg == 0` escape hatch that returns the raw pulse
// midpoint, and setServoAngles() clamps every command into [fin_min, fin_max].
// Store min == max and the two together freeze all four fins at centre for the
// whole flight while the controller happily computes commands nobody obeys.
// The wire path used to guard against it and the NVS boot restore did not, so
// the guard now lives in the setter where both paths pass through.

TEST_F(ServoControlTest, FinCalibration_AcceptsARealAirframe) {
    EXPECT_TRUE(servo.setFinCalibration(-45.0f, 45.0f));
    EXPECT_FLOAT_EQ(servo.getFinMinDeg(), -45.0f);
    EXPECT_FLOAT_EQ(servo.getFinMaxDeg(),  45.0f);
}

TEST_F(ServoControlTest, FinCalibration_RejectsZeroSpanAndKeepsThePrevious) {
    ASSERT_TRUE(servo.setFinCalibration(-30.0f, 30.0f));
    EXPECT_FALSE(servo.setFinCalibration(12.0f, 12.0f));
    // The previous calibration must survive -- falling back to the last good
    // value is the whole point; zeroing it would be the failure being fixed.
    EXPECT_FLOAT_EQ(servo.getFinMinDeg(), -30.0f);
    EXPECT_FLOAT_EQ(servo.getFinMaxDeg(),  30.0f);
}

TEST_F(ServoControlTest, FinCalibration_RejectsAnInvertedSpan) {
    ASSERT_TRUE(servo.setFinCalibration(-30.0f, 30.0f));
    EXPECT_FALSE(servo.setFinCalibration(30.0f, -30.0f));
    EXPECT_FLOAT_EQ(servo.getFinMinDeg(), -30.0f);
}

TEST_F(ServoControlTest, FinCalibration_RejectsNonFiniteValues) {
    ASSERT_TRUE(servo.setFinCalibration(-30.0f, 30.0f));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(servo.setFinCalibration(nan, 30.0f));
    EXPECT_FALSE(servo.setFinCalibration(-30.0f, nan));
    EXPECT_FALSE(servo.setFinCalibration(-inf, inf));
    EXPECT_FLOAT_EQ(servo.getFinMinDeg(), -30.0f);
    EXPECT_FLOAT_EQ(servo.getFinMaxDeg(),  30.0f);
}

TEST_F(ServoControlTest, FinCalibration_RejectsATooNarrowSpan) {
    // Just under the floor is refused, exactly the floor is accepted.  A span
    // this small is not a real airframe: a 1 deg command already saturates the
    // servo, which looks the same from outside as the frozen-fin failure.
    ASSERT_TRUE(servo.setFinCalibration(-30.0f, 30.0f));
    EXPECT_FALSE(servo.setFinCalibration(
        0.0f, TR_ServoControl::kMinFinSpanDeg - 0.1f));
    EXPECT_FLOAT_EQ(servo.getFinMinDeg(), -30.0f);
    EXPECT_TRUE(servo.setFinCalibration(0.0f, TR_ServoControl::kMinFinSpanDeg));
    EXPECT_FLOAT_EQ(servo.getFinMaxDeg(), TR_ServoControl::kMinFinSpanDeg);
}

TEST_F(ServoControlTest, FinCalibration_PredicateMatchesTheSetter) {
    // finCalibrationValid() is what the SERVO_CONFIG handler consults to decide
    // whether to PERSIST, without applying first (it must not apply INFLIGHT).
    // If the two ever disagree, a value could be saved that the setter refuses
    // -- which is precisely the bug this closes.
    const float cases[][2] = {
        {-60.0f, 60.0f}, {0.0f, 2.0f}, {12.0f, 12.0f}, {30.0f, -30.0f},
        {0.0f, 1.9f},    {-1.0f, 1.0f}, {0.0f, 0.0f},
    };
    for (const auto &c : cases) {
        TR_ServoControl fresh{1, 2, 3, 4, 0, 0, 0, 0, 50, 1000, 2000,
                              KP, KI, KD, MIN_CMD, MAX_CMD};
        const bool predicate = TR_ServoControl::finCalibrationValid(c[0], c[1]);
        const bool applied   = fresh.setFinCalibration(c[0], c[1]);
        EXPECT_EQ(predicate, applied) << "min=" << c[0] << " max=" << c[1];
    }
}

TEST_F(ServoControlTest, FinCalibration_DegenerateSpanWouldHaveFrozenTheFins) {
    // Demonstrates the consequence the guard prevents, by driving the mapping
    // directly through the accepted path and then showing that the refused
    // pair leaves the good mapping intact.
    const float angles[4] = {30.0f, 30.0f, 30.0f, 30.0f};
    ASSERT_TRUE(servo.setFinCalibration(-60.0f, 60.0f));
    servo.setServoAngles(angles);
    const int commanded = servo.getServoPulseUs(0);

    ASSERT_FALSE(servo.setFinCalibration(5.0f, 5.0f));
    servo.setServoAngles(angles);
    EXPECT_EQ(servo.getServoPulseUs(0), commanded)
        << "the refused calibration changed the fin mapping anyway";
}

// ── #1141: four defects in the servo layer ──

TEST_F(ServoControlTest, StowParksAtTheCalibratedFinZeroNotTheRawMidpoint) {
    // #1141 item 1. stowControl() used to call setPulse(0), which writes
    // servo_mid_us_ = (min+max)/2 + bias -- a number that knows nothing about
    // the #267 fin calibration. usFromFinDeg(0) equals that midpoint only when
    // the calibration is symmetric. With 1000/2000 us and a [-10,+30] cal,
    // fin-zero is 1250 us against a 1500 us midpoint: a standing 25%-of-travel
    // deflection on all four fins, held for the whole descent, on the one path
    // whose entire job is to stow them.
    ASSERT_TRUE(servo.setFinCalibration(-10.0f, 30.0f));

    const float zeros[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    servo.setServoAngles(zeros);
    int commanded[4];
    for (int i = 0; i < 4; ++i) commanded[i] = servo.getServoPulseUs(i);

    servo.stowControl();
    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(servo.getServoPulseUs(i), commanded[i])
            << "stow disagreed with a commanded 0 deg on servo " << i;
    }
    EXPECT_NE(commanded[0], (1000 + 2000) / 2)
        << "this cal must be asymmetric or the test proves nothing";
}

TEST_F(ServoControlTest, StowMatchesTheMidpointForASymmetricCalibration) {
    // The unchanged case: with a symmetric cal the two paths agree, so this
    // fix is a no-op on an airframe whose travel is centred.
    ASSERT_TRUE(servo.setFinCalibration(-45.0f, 45.0f));
    servo.stowControl();
    EXPECT_EQ(servo.getServoPulseUs(0), (1000 + 2000) / 2);
}

TEST_F(ServoControlTest, ANeutralSettleCompletesOnceServiced) {
    // #1141 item 2. The component half was always correct: given a service
    // call past the hold, the settle finishes. The defect was that the two
    // service call sites lived inside `case READY` and `case PRELAUNCH` while
    // beginNeutralSettle() is also reached from the SERVO_CONFIG trim preview
    // and SERVO_TEST_STOP -- so a settle begun in another state was never
    // serviced and the fins stayed 6 deg past neutral, energised, for the rest
    // of the session. That half is fixed in main.cpp (the service call is now
    // unconditional); this pins the contract it relies on.
    ASSERT_TRUE(servo.setFinCalibration(-45.0f, 45.0f));
    servo.beginNeutralSettle(0);
    ASSERT_TRUE(servo.isNeutralSettling());
    const int overshoot_us = servo.getServoPulseUs(0);

    servo.serviceNeutralSettle(TR_ServoControl::kNeutralSettleHoldMs - 1);
    EXPECT_TRUE(servo.isNeutralSettling()) << "settled before the hold elapsed";
    EXPECT_EQ(servo.getServoPulseUs(0), overshoot_us);

    servo.serviceNeutralSettle(TR_ServoControl::kNeutralSettleHoldMs);
    EXPECT_FALSE(servo.isNeutralSettling());
    EXPECT_EQ(servo.getServoPulseUs(0), (1000 + 2000) / 2);
}

TEST_F(ServoControlTest, IdleAbandonsAnInProgressSettle) {
    // Relaxing the rail supersedes the settle; leaving the flag set would let
    // a later service tick re-energise fins that were deliberately relaxed.
    servo.beginNeutralSettle(0);
    ASSERT_TRUE(servo.isNeutralSettling());
    servo.idle();
    EXPECT_FALSE(servo.isNeutralSettling());
}

TEST_F(ServoControlTest, StowAbandonsAnInProgressSettle) {
    servo.beginNeutralSettle(0);
    ASSERT_TRUE(servo.isNeutralSettling());
    servo.stowControl();
    EXPECT_FALSE(servo.isNeutralSettling());
}

TEST_F(ServoControlTest, ServoTimingValid_RejectsWhatIsNotATiming) {
    // #1141 item 3. The duty math is pulse_us * servo_hz * max_duty / 1e6, so
    // hz == 0 puts every channel at 0% duty -- no rising edge, no pulse train,
    // which is exactly what idle() does. A negative hz casts to ~4.29e9 and
    // wraps the 32-bit product to an arbitrary duty. Both arrived off the wire
    // and were applied without a word.
    EXPECT_FALSE(TR_ServoControl::servoTimingValid(0, 1000, 2000));
    EXPECT_FALSE(TR_ServoControl::servoTimingValid(-50, 1000, 2000));
    EXPECT_FALSE(TR_ServoControl::servoTimingValid(50, 2000, 1000));   // inverted
    EXPECT_FALSE(TR_ServoControl::servoTimingValid(50, 1000, 1050));   // span too small
    EXPECT_FALSE(TR_ServoControl::servoTimingValid(5000, 1000, 2000)); // absurd rate
    EXPECT_TRUE(TR_ServoControl::servoTimingValid(50, 1000, 2000));
    EXPECT_TRUE(TR_ServoControl::servoTimingValid(333, 900, 2100));
}

TEST_F(ServoControlTest, RejectedServoTimingKeepsThePrevious) {
    ASSERT_TRUE(servo.setServoTiming(50, 1000, 2000));
    ASSERT_TRUE(servo.setFinCalibration(-45.0f, 45.0f));
    servo.stowControl();
    const int before = servo.getServoPulseUs(0);

    EXPECT_FALSE(servo.setServoTiming(0, 1000, 2000));
    servo.stowControl();
    EXPECT_EQ(servo.getServoPulseUs(0), before)
        << "a refused timing changed the pulse anyway";
    EXPECT_EQ(servo.getServoMinUs(), 1000);
    EXPECT_EQ(servo.getServoMaxUs(), 2000);
}

// ── setServoTiming(): no counter reset, no motion ──
//
// Every SERVO_CONFIG from the app lands in setServoTiming(), rate changed or
// not.  It used to re-run ledc_timer_config() on all four timers -- on target
// that ends in ledc_timer_rst(), restarting the counter mid-frame, so a pulse
// that was high at the time came out stretched by however long it had already
// been high -- and then setPulse(0), which powered all four fins to the raw
// midpoint, including from setup_fc when the boot restored a non-default NVS
// timing.  The shim records peripheral calls, so these pin what the path may
// ask for, not just the values it stores.

TEST_F(ServoControlTest, UnchangedFrameRateTouchesNoTimerAndMovesNothing) {
    // SetUp left the fixture at 50 Hz with all four channels driven.
    int held[4];
    for (int i = 0; i < 4; ++i) held[i] = servo.getServoPulseUs(i);
    hostLedcLogReset();

    ASSERT_TRUE(servo.setServoTiming(50, 900, 2100));   // a trim resend: same rate

    const HostLedcLog& log = hostLedcLog();
    EXPECT_EQ(log.timer_config_calls, 0)
        << "a timer was reconfigured -- and so reset -- for an unchanged rate";
    EXPECT_EQ(log.set_freq_calls, 0);
    EXPECT_EQ(log.set_duty_calls, 0) << "setServoTiming() drove a fin";
    for (int i = 0; i < 4; ++i) EXPECT_EQ(servo.getServoPulseUs(i), held[i]);
    EXPECT_EQ(servo.getServoMinUs(), 900) << "the new timing must still be stored";
    EXPECT_EQ(servo.getServoMaxUs(), 2100);
}

TEST_F(ServoControlTest, ChangedFrameRateRetimesWithoutAResetAndHoldsEveryPulse) {
    int held[4];
    for (int i = 0; i < 4; ++i) held[i] = servo.getServoPulseUs(i);
    hostLedcLogReset();

    ASSERT_TRUE(servo.setServoTiming(333, 1000, 2000));

    const HostLedcLog& log = hostLedcLog();
    EXPECT_EQ(log.timer_config_calls, 0)
        << "ledc_timer_config() resets the counter mid-frame; use ledc_set_freq()";
    EXPECT_EQ(log.set_freq_calls, 4);
    for (int t = 0; t < 4; ++t) EXPECT_EQ(log.freq_hz[t], 333u);
    // Duty is a count of the period, so each held width must be re-expressed
    // at the new rate or the pulse on the pin changes by 333/50.
    constexpr uint32_t kMaxDuty = (1u << LEDC_TIMER_12_BIT) - 1;   // the driver's resolution
    EXPECT_EQ(log.set_duty_calls, 4);
    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(servo.getServoPulseUs(i), held[i]) << "servo " << i << " moved";
        EXPECT_EQ(log.duty[i], static_cast<uint32_t>(held[i]) * 333u * kMaxDuty / 1000000u)
            << "servo " << i << " holds a different width at the new rate";
    }
}

TEST_F(ServoControlTest, ATimingChangeNeverWakesARelaxedServo) {
    // The boot path: setup_fc restores a non-default NVS timing right after
    // begin(), before it has asked whether this boot is resuming a flight.
    TR_ServoControl fresh{1, 2, 3, 4, 0, 0, 0, 0, 50, 1000, 2000,
                          KP, KI, KD, MIN_CMD, MAX_CMD};
    fresh.begin();
    hostLedcLogReset();

    ASSERT_TRUE(fresh.setServoTiming(333, 900, 2100));

    EXPECT_TRUE(fresh.isIdle());
    EXPECT_EQ(hostLedcLog().set_duty_calls, 0) << "setup_fc moved a fin";
    for (int i = 0; i < 4; ++i) EXPECT_EQ(fresh.getServoPulseUs(i), 0);

    // The pad relax: the same, after the pulse train was deliberately cut.
    servo.idle();
    hostLedcLogReset();
    ASSERT_TRUE(servo.setServoTiming(333, 1000, 2000));
    EXPECT_TRUE(servo.isIdle());
    EXPECT_EQ(hostLedcLog().set_duty_calls, 0) << "a relaxed servo was re-energised";
}

TEST_F(ServoControlTest, ATimingChangeReexpressesOnlyTheChannelsBeingDriven) {
    // The boot wiggle wakes one channel at a time, so "not idle" is not "all
    // four driven" -- and last_pulse_us_ keeps its value through idle() as a
    // diagnostic, so it cannot be the test either.  Re-expressing a channel
    // that is relaxed would energise it.
    servo.idle();
    servo.beginWiggle(0);
    servo.serviceWiggle(0);            // servo 1 to its min; 2-4 stay relaxed
    ASSERT_FALSE(servo.isIdle());
    hostLedcLogReset();

    ASSERT_TRUE(servo.setServoTiming(333, 1000, 2000));

    EXPECT_EQ(hostLedcLog().set_duty_calls, 1) << "a relaxed channel was driven";
    EXPECT_NE(hostLedcLog().duty[0], 0u);
}

TEST_F(ServoControlTest, ATimingSetBeforeBeginIsAppliedByBegin) {
    TR_ServoControl fresh{1, 2, 3, 4, 0, 0, 0, 0, 50, 1000, 2000,
                          KP, KI, KD, MIN_CMD, MAX_CMD};
    hostLedcLogReset();

    ASSERT_TRUE(fresh.setServoTiming(333, 1000, 2000));
    EXPECT_EQ(hostLedcLog().set_freq_calls, 0) << "no timers exist before begin()";

    fresh.begin();
    EXPECT_EQ(hostLedcLog().timer_config_calls, 4);
    for (int t = 0; t < 4; ++t) EXPECT_EQ(hostLedcLog().freq_hz[t], 333u);
}

TEST_F(ServoControlTest, GainScheduleDoesNotLeakIntoTheRateNullFallback) {
    // #1141 item 4. applyGainSchedule() mutates the live PID gains in place and
    // nothing on the unscheduled path put them back, so on the
    // ekf_ctrl_healthy FALLING edge the flight loop switched to control() --
    // the pure-gyro fallback that exists BECAUSE the EKF is untrustworthy --
    // and ran it on gains still scaled by the last healthy tick, up to the 3x
    // GAIN_SCHEDULE_SCALE_CAP.
    //
    // P-only fixture (KP=1, KI=KD=0), so the output is the scaled rate error.
    servo.enableGainSchedule(/*v_ref=*/95.0f, /*v_min=*/30.0f);

    tick();
    servo.controlWithGainSchedule(10.0f, /*velocity_ms=*/30.0f);   // scale -> cap
    const float scheduled = servo.getRollCmdDeg();

    tick();
    servo.control(10.0f);                                          // the fallback
    const float fallback = servo.getRollCmdDeg();

    EXPECT_LT(std::fabs(fallback), std::fabs(scheduled))
        << "the fallback inherited the schedule's gains";
    EXPECT_NEAR(std::fabs(fallback), 10.0f, 0.5f)
        << "the fallback should run at the unscaled 1x gain";
}

TEST_F(ServoControlTest, ResetPidAlsoRestoresBaseGains) {
    servo.enableGainSchedule(95.0f, 30.0f);
    tick();
    servo.controlWithGainSchedule(10.0f, 30.0f);
    servo.resetPID();
    // pid.reset() re-arms the dt bootstrap, so the next call returns 0 by
    // design (see SetUp) — burn it, then assert on a steady-state tick.
    tick();
    servo.control(10.0f);
    tick();
    servo.control(10.0f);
    EXPECT_NEAR(std::fabs(servo.getRollCmdDeg()), 10.0f, 0.5f);
}
