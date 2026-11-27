// #1770: the fin servos after a servo test, with servo control on or off
// (flight_computer/main/servo_pad_policy.h and main.cpp's servo-test stop).
//
// With servo control off the servos stay live, and the servo test drives them,
// but every path that brought them back -- the neutral settle's service, the
// pad relax, the #363 launch failsafe's stow, LANDED's relax -- was gated on
// servo_enabled. A SERVO_TEST_STOP (sent whenever the app's Servo Test screen
// closes) started the anti-backlash settle, and the fins stayed 6 deg past
// neutral, powered, through the flight.
//
// Two parts:
//   - the policy's tables;
//   - a pad session and flight with the real TR_ServoControl (its LEDC calls
//     go to the host shim, which records the duty each channel was given),
//     each pass written as main.cpp runs it: the command, the #363 failsafe,
//     the test-mode chain, the settle's service, the state's relax.

#include <gtest/gtest.h>
#include <Arduino.h>   // host shim: setMockMicros()
#include <driver/ledc.h>
#include <array>
#include <cstdint>
#include <vector>

#include "servo_pad_policy.h"
#include "TR_ServoControl_ledc_mult.h"

namespace {

// ---------------------------------------------------------------------------
// The policy's tables
// ---------------------------------------------------------------------------

TEST(ServoPadPolicy, ServoControlOnRelaxesOnlyWithThePadRelax)
{
    EXPECT_TRUE(ServoPadPolicy::relaxOnPad(true, true, false, false, 100, 100));
    EXPECT_FALSE(ServoPadPolicy::relaxOnPad(true, false, false, false, 100, 100));
}

TEST(ServoPadPolicy, ServoControlOffAlwaysRelaxes)
{
    EXPECT_TRUE(ServoPadPolicy::relaxOnPad(false, true, false, false, 100, 100));
    EXPECT_TRUE(ServoPadPolicy::relaxOnPad(false, false, false, false, 100, 100));
}

TEST(ServoPadPolicy, NeverWhileTheSweepOrASettleDrivesThemOrInTheWakeWindow)
{
    for (int on = 0; on < 2; ++on)
    {
        EXPECT_FALSE(ServoPadPolicy::relaxOnPad(on != 0, true, true, false, 100, 100)) << on;
        EXPECT_FALSE(ServoPadPolicy::relaxOnPad(on != 0, true, false, true, 100, 100)) << on;
        EXPECT_FALSE(ServoPadPolicy::relaxOnPad(on != 0, true, false, false, 99, 100)) << on;
        EXPECT_TRUE(ServoPadPolicy::relaxOnPad(on != 0, true, false, false, 100, 100)) << on;
    }
}

TEST(ServoPadPolicy, TheWakeWindowIsWrapSafe)
{
    const uint32_t wake = 0x00000010u;   // armed just past the wrap
    EXPECT_FALSE(ServoPadPolicy::relaxOnPad(false, true, false, false, 0xFFFFFFF0u, wake));
    EXPECT_TRUE(ServoPadPolicy::relaxOnPad(false, true, false, false, wake, wake));
}

TEST(ServoPadPolicy, LandedRelaxesWithServoControlOffThePadRelaxOrACut)
{
    EXPECT_TRUE(ServoPadPolicy::relaxAtLanding(false, false, false));
    EXPECT_TRUE(ServoPadPolicy::relaxAtLanding(true, true, false));
    EXPECT_TRUE(ServoPadPolicy::relaxAtLanding(true, false, true));
    EXPECT_FALSE(ServoPadPolicy::relaxAtLanding(true, false, false));
}

TEST(ServoPadPolicy, AStopSettlesWhileATestRunsOrOnThePad)
{
    EXPECT_TRUE(ServoPadPolicy::stopSettles(true, false));
    EXPECT_TRUE(ServoPadPolicy::stopSettles(false, true));
    EXPECT_TRUE(ServoPadPolicy::stopSettles(true, true));
    EXPECT_FALSE(ServoPadPolicy::stopSettles(false, false));
}

TEST(ServoPadPolicy, ACancelledModeIsStowedWhenSomethingDroveTheFins)
{
    for (int t = 0; t < 2; ++t)
        for (int r = 0; r < 2; ++r)
            EXPECT_TRUE(ServoPadPolicy::stowOnCancel(true, t != 0, r != 0)) << "servo control on";
    EXPECT_TRUE(ServoPadPolicy::stowOnCancel(false, true, false));
    EXPECT_TRUE(ServoPadPolicy::stowOnCancel(false, false, true));
    EXPECT_FALSE(ServoPadPolicy::stowOnCancel(false, false, false)) << "a ground test drives nothing with it off";
}

// ---------------------------------------------------------------------------
// A pad session and flight with the real servo controller
// ---------------------------------------------------------------------------

constexpr uint32_t kTickMs = 2;
constexpr uint32_t kWakeMs = 6000;          // config::SERVO_PAD_TRIM_WAKE_MS
constexpr uint32_t kTestAt = 1000;          // the Servo Test screen's angles
constexpr uint32_t kStopAt = 3000;          // the screen closes
constexpr uint32_t kPrelaunchAt = 10000;
constexpr uint32_t kLaunchAt = 30000;
constexpr uint32_t kLandAt = 90000;
constexpr uint32_t kEndAt = 95000;
constexpr float    kTestDeg = 10.0f;

enum class St { Ready, Prelaunch, Inflight, Landed };

class ServoSession : public ::testing::Test
{
protected:
    // A 1000-2000 us, +/-20 deg calibration: neutral 1500 us, the settle's
    // 6 deg overshoot 1650 us, the test's 10 deg 1750 us.
    TR_ServoControl servo{1, 2, 3, 4, 0, 0, 0, 0, 50, 1000, 2000, 0.12f, 0.01f, 0.0f, -20.0f, 20.0f};

    // What the session does: main.cpp after #1770 (fixed) or before it.
    bool fixed = true;
    bool servo_enabled = false;
    bool relax_on_pad = true;           // config::SERVO_RELAX_ON_PAD
    bool send_test = true;              // the screen drives the fins before it closes
    bool stop_after_launch = false;     // the test still running at launch (no stop: INFLIGHT polls none)
    bool ground_test_at_launch = false; // a ground test running at launch instead
    bool reboot_mid_test = false;       // the FC resets while the test holds its angles
    bool land = false;
    uint32_t stray_stop_at = 0;         // a stop with no test running (0: none)

    St state = St::Ready;
    bool servo_test_active = false;
    bool servo_replay_active = false;
    bool ground_test_active = false;
    bool launch_flag = false;
    bool landed_actions_done = false;
    uint32_t wake_until_ms = 0;

    void SetUp() override
    {
        hostLedcLogReset();
        setMockMicros(0);
        ASSERT_TRUE(servo.setFinCalibration(-20.0f, 20.0f));
        servo.begin();   // every channel at 0% duty: relaxed, as setup_fc leaves them
    }

    static bool pulsesCut()
    {
        for (int i = 0; i < 4; ++i)
            if (hostLedcLog().duty[i] != 0) return false;
        return true;
    }
    bool allAt(int us) const
    {
        for (int i = 0; i < 4; ++i)
            if (servo.getServoPulseUs(i) != us) return false;
        return true;
    }

    // The command handlers.
    void servoTestAngles()
    {
        const float a[4] = {kTestDeg, kTestDeg, kTestDeg, kTestDeg};
        servo_test_active = true;
        servo.setServoAngles(a);
    }
    void servoTestStop(uint32_t now_ms)
    {
        if (fixed)
        {
            const bool on_pad = state == St::Ready || state == St::Prelaunch;
            if (ServoPadPolicy::stopSettles(servo_test_active, on_pad))
            {
                servo.beginNeutralSettle(now_ms);
                wake_until_ms = now_ms + kWakeMs;
            }
            servo_test_active = false;
        }
        else
        {
            servo_test_active = false;
            servo.beginNeutralSettle(now_ms);
            wake_until_ms = now_ms + kWakeMs;
        }
    }

    void servoReplayAngles(float deg)
    {
        const float a[4] = {deg, deg, deg, deg};
        servo_replay_active = true;
        servo.setServoAngles(a);
    }
    void servoReplayStop(uint32_t now_ms)
    {
        servo_replay_active = false;
        servo.stowControl();
        servo.resetPID();
        if (fixed) wake_until_ms = now_ms + kWakeMs;
    }

    // One pass after the command poll.
    void pass(uint32_t now_ms)
    {
        // #363: a test left running at launch is cancelled and the fins
        // stowed; the wake window keeps the same pass's pad relax from cutting
        // the stow before the fins get there.
        if (launch_flag && (servo_test_active || servo_replay_active || ground_test_active))
        {
            const bool stow = fixed
                ? ServoPadPolicy::stowOnCancel(servo_enabled, servo_test_active, servo_replay_active)
                : servo_enabled;
            servo_test_active = false;
            servo_replay_active = false;
            ground_test_active = false;
            if (stow)
            {
                servo.stowControl();
                if (fixed) wake_until_ms = now_ms + kWakeMs;
            }
        }
        // The test-mode chain skips the state machine; a ground test drives
        // nothing with servo control off.
        if (servo_test_active || servo_replay_active || ground_test_active) return;

        if (fixed || servo_enabled) servo.serviceNeutralSettle(now_ms);

        switch (state)
        {
            case St::Ready:
            case St::Prelaunch:
            {
                const bool relax = fixed
                    ? ServoPadPolicy::relaxOnPad(servo_enabled, relax_on_pad, servo.isWiggling(),
                                                 servo.isNeutralSettling(), now_ms, wake_until_ms)
                    : (servo_enabled && relax_on_pad && !servo.isWiggling() &&
                       !servo.isNeutralSettling() && (int32_t)(now_ms - wake_until_ms) >= 0);
                if (relax) servo.idle();
                if (state == St::Prelaunch && launch_flag) state = St::Inflight;
                break;
            }
            case St::Inflight:
                // Servo control off: the servo block does not run.  (On, the
                // roll laws drive the fins; not modeled here.)
                break;
            case St::Landed:
                if (!landed_actions_done)
                {
                    landed_actions_done = true;
                    if (fixed)
                    {
                        if (ServoPadPolicy::relaxAtLanding(servo_enabled, relax_on_pad, false))
                            servo.idle();
                        else
                            servo.stowControl();
                    }
                    else if (servo_enabled)
                    {
                        if (relax_on_pad) servo.idle(); else servo.stowControl();
                    }
                }
                break;
        }
    }

    struct Result
    {
        bool settling_at_launch = false;
        bool relaxed_at_launch = false;
        int  pulse_at_launch = 0;
        int  inflight_passes_driven = 0;   // a pulse train on the servos in flight
        int  inflight_passes_off_neutral = 0;
        bool settling_at_end = false;
        bool relaxed_at_end = false;
        int  pulse_at_end = 0;
    };

    Result run()
    {
        Result r;
        for (uint32_t t = 0; t <= kEndAt; t += kTickMs)
        {
            setMockMicros((uint64_t)t * 1000u);
            if (send_test && t == kTestAt) servoTestAngles();
            if (ground_test_at_launch && t == kTestAt) ground_test_active = true;
            if (reboot_mid_test && t == kTestAt + 500)
            {
                // A reset: the flag is gone, begin() leaves every channel at
                // 0% (relaxed), and the fins stay where the test put them.
                servo_test_active = false;
                servo.idle();
            }
            if (!stop_after_launch && !ground_test_at_launch && t == kStopAt) servoTestStop(t);
            if (stray_stop_at != 0 && t == stray_stop_at) servoTestStop(t);
            if (t == kPrelaunchAt) state = St::Prelaunch;
            if (t == kLaunchAt)
            {
                launch_flag = true;
                r.settling_at_launch = servo.isNeutralSettling();
                r.relaxed_at_launch = servo.isIdle();
                r.pulse_at_launch = servo.getServoPulseUs(0);
            }
            if (land && t == kLandAt) state = St::Landed;
            pass(t);
            if (state == St::Inflight)
            {
                if (!servo.isIdle()) ++r.inflight_passes_driven;
                if (!servo.isIdle() && !allAt(1500)) ++r.inflight_passes_off_neutral;
            }
        }
        r.settling_at_end = servo.isNeutralSettling();
        r.relaxed_at_end = servo.isIdle();
        r.pulse_at_end = servo.getServoPulseUs(0);
        return r;
    }
};

// The issue's case: servo control off, a test, the screen closes, the rocket
// flies without a power cycle. Now: the settle finishes at neutral, the wake
// window shows it for 6 s, the servos relax, and they fly relaxed at neutral.
TEST_F(ServoSession, ServoControlOffAStopSettlesToNeutralThenRelaxes)
{
    const Result r = run();
    EXPECT_FALSE(r.settling_at_launch);
    EXPECT_TRUE(r.relaxed_at_launch);
    EXPECT_EQ(r.pulse_at_launch, 1500);
    EXPECT_EQ(r.inflight_passes_driven, 0);
    EXPECT_FALSE(r.settling_at_end);
    EXPECT_TRUE(r.relaxed_at_end);
    EXPECT_TRUE(pulsesCut());
}

TEST_F(ServoSession, ServoControlOffTheStowIsShownThroughTheWakeWindow)
{
    for (uint32_t t = 0; t <= kStopAt + kWakeMs + 100; t += kTickMs)
    {
        setMockMicros((uint64_t)t * 1000u);
        if (t == kTestAt) servoTestAngles();
        if (t == kStopAt) servoTestStop(t);
        pass(t);
        if (t == kStopAt) EXPECT_TRUE(allAt(1650));   // the overshoot
        if (t == kStopAt + TR_ServoControl::kNeutralSettleHoldMs + 2)
        {
            EXPECT_TRUE(allAt(1500));
            EXPECT_FALSE(servo.isIdle());
        }
        if (t == kStopAt + kWakeMs - 2) EXPECT_FALSE(servo.isIdle()) << "relaxed inside the wake window";
        if (t == kStopAt + kWakeMs) EXPECT_TRUE(servo.isIdle());
    }
}

// Before #1770 the same session left the settle unserviced: 1650 us on all
// four, powered, through the flight -- the issue's harness result.
TEST_F(ServoSession, BeforeTheFixTheOvershootFlewPowered)
{
    fixed = false;
    const Result r = run();
    EXPECT_TRUE(r.settling_at_launch);
    EXPECT_EQ(r.pulse_at_launch, 1650);
    EXPECT_GT(r.inflight_passes_off_neutral, 0);
    EXPECT_TRUE(r.settling_at_end);
    EXPECT_EQ(r.pulse_at_end, 1650);
    EXPECT_FALSE(pulsesCut());
}

// A reboot mid-test leaves the fins relaxed at the test's angles (with servo
// control off there is no boot sweep). The app's stop on the pad, with no
// test running any more, still stows them: neutral, then relaxed.
TEST_F(ServoSession, ServoControlOffAStopAfterARebootMidTestStillStows)
{
    reboot_mid_test = true;
    const Result r = run();
    EXPECT_EQ(r.pulse_at_launch, 1500);
    EXPECT_TRUE(r.relaxed_at_launch);
    EXPECT_FALSE(r.settling_at_launch);
    EXPECT_EQ(r.inflight_passes_driven, 0);
}

// Off the pad a stop with no test running drives nothing: LANDED never
// relaxes the servos again, so a settle there would hold them powered.
TEST_F(ServoSession, AStrayStopAfterLandingDrivesNothing)
{
    for (int on = 0; on < 2; ++on)
    {
        SCOPED_TRACE(on ? "servo control on" : "servo control off");
        hostLedcLogReset();
        servo = TR_ServoControl{1, 2, 3, 4, 0, 0, 0, 0, 50, 1000, 2000, 0.12f, 0.01f, 0.0f, -20.0f, 20.0f};
        ASSERT_TRUE(servo.setFinCalibration(-20.0f, 20.0f));
        servo.begin();
        servo_enabled = on != 0;
        state = St::Landed;
        landed_actions_done = true;
        servo.idle();
        const int duty_calls = hostLedcLog().set_duty_calls;
        servoTestStop(100);
        for (uint32_t t = 100; t < 10000; t += kTickMs) { setMockMicros((uint64_t)t * 1000u); pass(t); }
        EXPECT_EQ(hostLedcLog().set_duty_calls, duty_calls);
        EXPECT_TRUE(servo.isIdle());
    }
}

// A ground test running at launch with servo control off: it never drove the
// fins, so the failsafe leaves them relaxed rather than powering them.
TEST_F(ServoSession, ServoControlOffAGroundTestAtLaunchLeavesTheFinsRelaxed)
{
    send_test = false;
    ground_test_at_launch = true;
    const int duty_calls = hostLedcLog().set_duty_calls;
    const Result r = run();
    EXPECT_EQ(r.inflight_passes_driven, 0);
    EXPECT_EQ(hostLedcLog().set_duty_calls, duty_calls);
}

// A test still running at launch with servo control off: the #363 failsafe
// cancels it and now stows, so the flight holds neutral, not the test's
// angles.
TEST_F(ServoSession, ServoControlOffATestRunningAtLaunchIsStowed)
{
    stop_after_launch = true;
    const Result r = run();
    EXPECT_GT(r.inflight_passes_driven, 0);
    EXPECT_EQ(r.inflight_passes_off_neutral, 0);
    EXPECT_EQ(r.pulse_at_end, 1500);
}

TEST_F(ServoSession, BeforeTheFixATestRunningAtLaunchFlewItsAngles)
{
    fixed = false;
    stop_after_launch = true;
    const Result r = run();
    EXPECT_GT(r.inflight_passes_off_neutral, 0);
    EXPECT_EQ(r.pulse_at_end, 1750);   // the test's 10 deg, to the end
}

// A servo replay's stop stows; with servo control off the pad now relaxes, so
// the stop arms the wake window: the fins are driven to neutral and shown
// there before the relax, not relaxed wherever the replay left them.
TEST_F(ServoSession, ServoControlOffAReplayStopStowsThenRelaxes)
{
    send_test = false;
    for (uint32_t t = 0; t <= kStopAt + kWakeMs + 100; t += kTickMs)
    {
        setMockMicros((uint64_t)t * 1000u);
        if (t == kTestAt) servoReplayAngles(12.0f);
        if (t == kStopAt) servoReplayStop(t);
        pass(t);
        if (t == kStopAt) EXPECT_TRUE(allAt(1500));
        if (t == kStopAt) EXPECT_FALSE(servo.isIdle()) << "relaxed on the stop's own pass";
        if (t == kStopAt + kWakeMs - 2) EXPECT_FALSE(servo.isIdle());
        if (t == kStopAt + kWakeMs) EXPECT_TRUE(servo.isIdle());
    }
    EXPECT_EQ(servo.getServoPulseUs(0), 1500);
}

// The failsafe's stow, then LANDED: relaxed with servo control off, with the
// pad relax on or off.
TEST_F(ServoSession, ServoControlOffLandedRelaxes)
{
    stop_after_launch = true;
    land = true;
    const Result r = run();
    EXPECT_TRUE(r.relaxed_at_end);
    EXPECT_TRUE(pulsesCut());
}

TEST_F(ServoSession, ServoControlOffLandedRelaxesWithoutThePadRelaxToo)
{
    stop_after_launch = true;
    land = true;
    relax_on_pad = false;
    const Result r = run();
    EXPECT_GT(r.inflight_passes_driven, 0);   // the failsafe's stow, held in flight
    EXPECT_TRUE(r.relaxed_at_end);
    EXPECT_TRUE(pulsesCut());
}

// Servo control on, on the pad: pass for pass the same pulses, duty and relax
// as main.cpp before #1770 -- a test and its stop, then a stop with no test
// running -- with the pad relax on and off.
TEST_F(ServoSession, ServoControlOnThePadIsPassForPassUnchanged)
{
    using Row = std::array<int, 10>;
    auto trace = [&](bool fixed_code, bool relax) {
        hostLedcLogReset();
        servo = TR_ServoControl{1, 2, 3, 4, 0, 0, 0, 0, 50, 1000, 2000, 0.12f, 0.01f, 0.0f, -20.0f, 20.0f};
        EXPECT_TRUE(servo.setFinCalibration(-20.0f, 20.0f));
        servo.begin();
        fixed = fixed_code;
        servo_enabled = true;
        relax_on_pad = relax;
        state = St::Ready;
        servo_test_active = false;
        wake_until_ms = 0;
        std::vector<Row> rows;
        for (uint32_t t = 0; t < kLaunchAt; t += kTickMs)
        {
            setMockMicros((uint64_t)t * 1000u);
            if (t == kTestAt) servoTestAngles();
            if (t == kStopAt) servoTestStop(t);
            if (t == 15000) servoTestStop(t);   // a stray stop on the pad
            if (t == kPrelaunchAt) state = St::Prelaunch;
            pass(t);
            Row row{};
            row[0] = servo.isIdle();
            row[1] = servo.isNeutralSettling();
            for (int i = 0; i < 4; ++i) row[2 + i] = servo.getServoPulseUs(i);
            for (int i = 0; i < 4; ++i) row[6 + i] = (int)hostLedcLog().duty[i];
            rows.push_back(row);
        }
        return rows;
    };
    for (int relax = 0; relax < 2; ++relax)
    {
        const std::vector<Row> before = trace(false, relax != 0);
        const std::vector<Row> after = trace(true, relax != 0);
        ASSERT_EQ(before.size(), after.size());
        for (size_t k = 0; k < before.size(); ++k)
            ASSERT_EQ(before[k], after[k]) << "relax=" << relax << " pass " << k;
    }
}

// Servo control on, as before: with the pad relax the stop settles, shows the
// stow, then relaxes; without it the fins hold the stowed neutral.
TEST_F(ServoSession, ServoControlOnWithThePadRelaxIsUnchanged)
{
    servo_enabled = true;
    for (uint32_t t = 0; t < kLaunchAt; t += kTickMs)
    {
        setMockMicros((uint64_t)t * 1000u);
        if (t == kTestAt) servoTestAngles();
        if (t == kStopAt) servoTestStop(t);
        pass(t);
        if (t == kStopAt + kWakeMs - 2) EXPECT_FALSE(servo.isIdle());
        if (t == kStopAt + kWakeMs) EXPECT_TRUE(servo.isIdle());
    }
    EXPECT_EQ(servo.getServoPulseUs(0), 1500);
    EXPECT_FALSE(servo.isNeutralSettling());
}

TEST_F(ServoSession, ServoControlOnWithoutThePadRelaxHoldsNeutral)
{
    servo_enabled = true;
    relax_on_pad = false;
    for (uint32_t t = 0; t < kLaunchAt; t += kTickMs)
    {
        setMockMicros((uint64_t)t * 1000u);
        if (t == kTestAt) servoTestAngles();
        if (t == kStopAt) servoTestStop(t);
        pass(t);
    }
    EXPECT_FALSE(servo.isIdle());
    EXPECT_TRUE(allAt(1500));
    EXPECT_FALSE(servo.isNeutralSettling());
}

}  // namespace
