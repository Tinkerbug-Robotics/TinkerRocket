// #1768: the fins' power once the eject has occurred
// (flight_computer/main/fin_eject_policy.h).
//
// The FC's INFLIGHT servo block had no deployment term: a roll law kept
// driving the fins, at the gain schedule's 3x cap under the chute, and the
// servo rail stayed up, from the eject to LANDED -- on a nominal flight with
// the servos already disconnected at the drogue's eject. Now roll control and
// guidance run as before until the eject, and the eject cuts the pulse train,
// then the rail, for the rest of the flight.
//
// Two parts:
//   - the policy's table;
//   - a flight with the real TR_ServoControl (its LEDC calls go to the host
//     shim, which records the duty each channel was given), the pass written
//     as main.cpp's INFLIGHT case runs it: control past apogee up to the
//     drogue's eject, then the pulses cut and locked, the rail off 50 ms
//     later and the logged roll command 0, to the end of a ten-minute
//     descent, whatever commands arrive.

#include <gtest/gtest.h>
#include <Arduino.h>   // host shim: setMockMicros()
#include <driver/ledc.h>
#include <cmath>
#include <cstdint>

#include "fin_eject_policy.h"
#include "TR_ServoControl_ledc_mult.h"

using FinEjectPolicy::Fins;

namespace {

// ---------------------------------------------------------------------------
// The policy's table
// ---------------------------------------------------------------------------

constexpr uint32_t kRailDelayMs = 50;   // config::SERVO_RAIL_CUT_DELAY_MS

TEST(FinEjectPolicy, BeforeTheEjectTheFinsArePowered)
{
    FinEjectPolicy::State s;
    for (uint32_t t = 0; t < 2000; t += 2) ASSERT_EQ(FinEjectPolicy::step(s, false, t, kRailDelayMs), Fins::Powered);
    EXPECT_FALSE(FinEjectPolicy::cut(s));
}

TEST(FinEjectPolicy, TheEjectCutsThePulsesThenTheRailAfterTheDelay)
{
    FinEjectPolicy::State s;
    EXPECT_EQ(FinEjectPolicy::step(s, false, 9998, kRailDelayMs), Fins::Powered);
    EXPECT_EQ(FinEjectPolicy::step(s, true, 10000, kRailDelayMs), Fins::CutBegin);
    EXPECT_TRUE(FinEjectPolicy::cut(s));
    EXPECT_EQ(FinEjectPolicy::step(s, true, 10002, kRailDelayMs), Fins::Cutting);
    EXPECT_EQ(FinEjectPolicy::step(s, true, 10049, kRailDelayMs), Fins::Cutting);
    EXPECT_EQ(FinEjectPolicy::step(s, true, 10050, kRailDelayMs), Fins::RailOff);
    for (uint32_t t = 10052; t < 12000; t += 2) ASSERT_EQ(FinEjectPolicy::step(s, true, t, kRailDelayMs), Fins::Cut);
}

TEST(FinEjectPolicy, TheCutHoldsForTheFlight)
{
    FinEjectPolicy::State s;
    FinEjectPolicy::step(s, true, 0, kRailDelayMs);
    // Nothing un-cuts it short of a reset: not an input that drops, in either phase.
    EXPECT_EQ(FinEjectPolicy::step(s, false, 10, kRailDelayMs), Fins::Cutting);
    EXPECT_EQ(FinEjectPolicy::step(s, false, 60, kRailDelayMs), Fins::RailOff);
    EXPECT_EQ(FinEjectPolicy::step(s, false, 70, kRailDelayMs), Fins::Cut);
    EXPECT_TRUE(FinEjectPolicy::cut(s));
}

TEST(FinEjectPolicy, TheRailDelaySurvivesTheClockWrap)
{
    FinEjectPolicy::State s;
    const uint32_t t0 = 0xFFFFFFF0u;
    EXPECT_EQ(FinEjectPolicy::step(s, true, t0, kRailDelayMs), Fins::CutBegin);
    EXPECT_EQ(FinEjectPolicy::step(s, true, t0 + 10u, kRailDelayMs), Fins::Cutting);   // not yet wrapped
    EXPECT_EQ(FinEjectPolicy::step(s, true, t0 + 30u, kRailDelayMs), Fins::Cutting);   // wrapped
    EXPECT_EQ(FinEjectPolicy::step(s, true, t0 + kRailDelayMs, kRailDelayMs), Fins::RailOff);
}

TEST(FinEjectPolicy, AResetStartsANewFlight)
{
    FinEjectPolicy::State s;
    FinEjectPolicy::step(s, true, 0, kRailDelayMs);
    FinEjectPolicy::step(s, true, 100, kRailDelayMs);
    FinEjectPolicy::reset(s);
    EXPECT_FALSE(FinEjectPolicy::cut(s));
    EXPECT_EQ(FinEjectPolicy::step(s, false, 200, kRailDelayMs), Fins::Powered);
    EXPECT_EQ(FinEjectPolicy::step(s, true, 300, kRailDelayMs), Fins::CutBegin);
    EXPECT_EQ(FinEjectPolicy::step(s, true, 310, kRailDelayMs), Fins::Cutting);
}

// ---------------------------------------------------------------------------
// The drive, and a flight, with the real servo controller
// ---------------------------------------------------------------------------

// config.h's defaults: KP/KI/KD, the +/-20 deg authority, the gain schedule's
// 50 m/s reference and 25 m/s floor (3x at and below it), 1000-2000 us, 50 Hz.
constexpr float kKp = 0.12f, kKi = 0.01f, kKd = 0.0f;
constexpr float kMinCmd = -20.0f, kMaxCmd = 20.0f;
constexpr float kVRef = 50.0f, kVMin = 25.0f;
constexpr uint32_t kTickMs = 2;            // a 500 Hz pass, for speed
constexpr uint32_t kApogeeMs = 12000;      // the apogee call
constexpr uint32_t kEjectMs = 13000;       // the drogue's charge, 1 s after it
constexpr uint32_t kMainMs = 60000;        // the main opens
constexpr uint32_t kLandMs = 600000;       // ten minutes of descent

struct Rail
{
    bool on = true;
    int  offs = 0;          // set_rail(false) calls
    int  ons = 0;
    bool idle_when_cut = false;     // the signal lines low when the rail went
    bool locked_when_cut = false;   // and held low
    bool locked_when_raised = false;
    uint32_t cut_at_ms = 0;
    TR_ServoControl* servo = nullptr;
    void operator()(bool v)
    {
        if (!v)
        {
            ++offs;
            idle_when_cut = servo && servo->isIdle();
            locked_when_cut = servo && servo->outputsLocked();
            cut_at_ms = (uint32_t)(micros() / 1000u);
        }
        else
        {
            ++ons;
            locked_when_raised = servo && servo->outputsLocked();
        }
        on = v;
    }
};

class FinsAtTheEject : public ::testing::Test
{
protected:
    TR_ServoControl servo{1, 2, 3, 4, 0, 0, 0, 0, 50, 1000, 2000,
                          kKp, kKi, kKd, kMinCmd, kMaxCmd};
    Rail rail;

    void SetUp() override
    {
        hostLedcLogReset();
        setMockMicros(0);
        servo.begin();
        servo.enableGainSchedule(kVRef, kVMin);
        rail.servo = &servo;
    }

    // The speed the gain schedule sees, and a section that keeps spinning at
    // 20 deg/s however the fins answer.
    static float speedAt(uint32_t t)
    {
        if (t < 3000) return 50.0f + 80.0f * (float)t / 3000.0f;     // boost
        if (t < kApogeeMs) return 130.0f * (float)(kApogeeMs - t) / (float)(kApogeeMs - 3000) + 2.0f;
        if (t < kEjectMs) return 2.0f + 9.8f * (float)(t - kApogeeMs) / 1000.0f;   // falling, no chute yet
        return t < kMainMs ? 25.0f : 6.0f;
    }
    static constexpr float kRollRateDps = 20.0f;

    static bool pulsesCut()
    {
        for (int i = 0; i < 4; ++i)
            if (hostLedcLog().duty[i] != 0) return false;
        return true;
    }

    struct Flight
    {
        bool eject_wired = true;      // false: main.cpp before #1768 (no eject term)
        bool reboot = false;          // a reboot under the drogue, after the eject
        uint32_t reboot_at_ms = 90000;
        bool landed_commands = false; // commands after the cut: a trim preview, a test stop
    };

    struct Result
    {
        int  control_passes_after_apogee_before_eject = 0;
        int  control_passes_after_eject = 0;
        int  driven_after_eject = 0;          // passes past the eject with a pulse train
        int  roll_cmd_logged_after_eject = 0; // passes past the eject whose roll_cmd is not 0
        int  set_duty_calls_after_eject = 0;
        bool rail_on_at_end = true;
        float max_fin_cmd_deg_after_eject = 0.0f;
    };

    // One flight, the pass as main.cpp's INFLIGHT case runs it: the policy's
    // step and drive, then the servo block (the cut branch first, then the
    // pre-control hold, then the roll law).
    Result fly(const Flight& f)
    {
        Result r;
        FinEjectPolicy::State fins;
        bool reboot_recovery = false;
        uint32_t settle_end_ms = 0;
        int duty_calls_at_eject = -1;
        for (uint32_t t = 0; t <= kLandMs; t += kTickMs)
        {
            setMockMicros((uint64_t)t * 1000u);
            if (f.reboot && t == f.reboot_at_ms)
            {
                // A fresh boot: the policy's static, an unlocked servo
                // controller with its channels at 0% (begin()), the rail
                // raised; the restore reads the cut from the snapshot.
                fins = FinEjectPolicy::State{};
                servo.lockOutputs(false);
                servo.idle();
                servo.resetPID();
                rail(true);
                reboot_recovery = true;
                settle_end_ms = t + 500;
            }
            if (reboot_recovery && t >= settle_end_ms) reboot_recovery = false;
            if (t == kEjectMs) duty_calls_at_eject = hostLedcLog().set_duty_calls;

            // The charge fires at kEjectMs; after a reboot, the snapshot's cut.
            const bool ejected = f.eject_wired && t >= kEjectMs;
            const Fins pass = FinEjectPolicy::step(fins, ejected, t, kRailDelayMs);
            FinEjectPolicy::drive(pass, servo, rail);

            if (FinEjectPolicy::cut(fins))
            {
                // guidance held; nothing commands the fins
            }
            else if (reboot_recovery)
            {
                servo.stowControl();
                servo.resetPID();
            }
            else
            {
                servo.controlWithGainSchedule(-kRollRateDps, speedAt(t));
                if (t >= kApogeeMs && t < kEjectMs) ++r.control_passes_after_apogee_before_eject;
                if (t >= kEjectMs)
                {
                    ++r.control_passes_after_eject;
                    const float cmd = std::fabs(servo.getRollCmdDeg());
                    if (cmd > r.max_fin_cmd_deg_after_eject) r.max_fin_cmd_deg_after_eject = cmd;
                }
            }

            // The command handlers poll on every pass, in INFLIGHT and LANDED
            // alike, and none checks the cut: a trim preview's settle (served
            // at the top of each pass), a servo-test or ground-test stop.
            if (f.landed_commands && t >= kEjectMs + 1000 && (t % 10000) == 0)
            {
                servo.beginNeutralSettle(t);
                servo.stowControl();
                const float trim[4] = {3.0f, 3.0f, 3.0f, 3.0f};
                servo.setServoAngles(trim);
                servo.beginWiggle(t);
            }
            servo.serviceNeutralSettle(t);
            servo.serviceWiggle(t);

            if (t >= kEjectMs)
            {
                if (!servo.isIdle()) ++r.driven_after_eject;
                if (servo.getRollCmdDeg() != 0.0f) ++r.roll_cmd_logged_after_eject;
                if (servo.isIdle()) EXPECT_TRUE(pulsesCut()) << "t=" << t;
            }
        }
        r.set_duty_calls_after_eject = hostLedcLog().set_duty_calls - duty_calls_at_eject;
        r.rail_on_at_end = rail.on;
        return r;
    }
};

TEST_F(FinsAtTheEject, CutBeginCutsAndLocksThePulsesAndLeavesTheRail)
{
    servo.controlWithGainSchedule(-kRollRateDps, 50.0f);
    ASSERT_FALSE(servo.isIdle());
    FinEjectPolicy::drive(Fins::CutBegin, servo, rail);
    EXPECT_TRUE(servo.isIdle());
    EXPECT_TRUE(servo.outputsLocked());
    EXPECT_TRUE(pulsesCut());
    EXPECT_TRUE(rail.on) << "the rail waits out the pulse under way";
    EXPECT_EQ(rail.offs, 0);
}

TEST_F(FinsAtTheEject, RailOffCutsTheRailWithTheLinesLowAndHeld)
{
    FinEjectPolicy::drive(Fins::CutBegin, servo, rail);
    FinEjectPolicy::drive(Fins::Cutting, servo, rail);
    EXPECT_TRUE(rail.on);
    FinEjectPolicy::drive(Fins::RailOff, servo, rail);
    EXPECT_FALSE(rail.on);
    EXPECT_EQ(rail.offs, 1);
    EXPECT_TRUE(rail.idle_when_cut);
    EXPECT_TRUE(rail.locked_when_cut);
    FinEjectPolicy::drive(Fins::Cut, servo, rail);
    EXPECT_EQ(rail.offs, 1);
}

TEST_F(FinsAtTheEject, PoweredDrivesNothing)
{
    servo.controlWithGainSchedule(-kRollRateDps, 50.0f);
    const int duty_calls = hostLedcLog().set_duty_calls;
    FinEjectPolicy::drive(Fins::Powered, servo, rail);
    EXPECT_FALSE(servo.isIdle());
    EXPECT_FALSE(servo.outputsLocked());
    EXPECT_EQ(hostLedcLog().set_duty_calls, duty_calls);
    EXPECT_TRUE(rail.on);
    EXPECT_EQ(rail.offs + rail.ons, 0);
}

// Control runs through apogee up to the eject, then nothing drives the fins:
// the pulses cut on the eject's pass (one write per channel, and none after),
// the rail off SERVO_RAIL_CUT_DELAY_MS later, to the end.
TEST_F(FinsAtTheEject, ControlRunsToTheEjectThenThePowerIsCutForTheFlight)
{
    const Result r = fly(Flight{});
    EXPECT_EQ(r.control_passes_after_apogee_before_eject, (int)((kEjectMs - kApogeeMs) / kTickMs));
    EXPECT_EQ(r.control_passes_after_eject, 0);
    EXPECT_EQ(r.driven_after_eject, 0);
    EXPECT_EQ(r.roll_cmd_logged_after_eject, 0);
    EXPECT_EQ(r.set_duty_calls_after_eject, 4);
    EXPECT_FALSE(r.rail_on_at_end);
    EXPECT_EQ(rail.offs, 1);
    EXPECT_EQ(rail.cut_at_ms, kEjectMs + kRailDelayMs);
    EXPECT_TRUE(rail.idle_when_cut);
    EXPECT_TRUE(rail.locked_when_cut);
}

// The command handlers can still drive the fins after the cut (LANDED polls
// them, and none checks the cut): with the outputs locked they write nothing,
// so no pulse reaches the unpowered rail.
TEST_F(FinsAtTheEject, CommandsAfterTheCutDriveNothing)
{
    Flight f;
    f.landed_commands = true;
    const Result r = fly(f);
    EXPECT_EQ(r.driven_after_eject, 0);
    EXPECT_EQ(r.set_duty_calls_after_eject, 4);
    EXPECT_FALSE(r.rail_on_at_end);
    EXPECT_TRUE(servo.isIdle());
    EXPECT_TRUE(pulsesCut());
}

// The harness discriminates: main.cpp before #1768 kept the gain schedule on
// the fins to the end, at its 3x cap under both chutes (7.2 deg for 20 deg/s,
// against 2.4 at the base gain), with the rail up.
TEST_F(FinsAtTheEject, BeforeTheFixTheGainScheduleKeptDrivingTheFins)
{
    Flight f;
    f.eject_wired = false;
    const Result r = fly(f);
    EXPECT_EQ(r.control_passes_after_eject, (int)((kLandMs - kEjectMs) / kTickMs) + 1);
    EXPECT_GT(r.driven_after_eject, 0);
    EXPECT_GT(r.roll_cmd_logged_after_eject, 0);
    EXPECT_GE(r.max_fin_cmd_deg_after_eject, 7.2f - 0.01f);
    EXPECT_TRUE(r.rail_on_at_end);
}

// A reboot under the drogue: the boot raises the rail and unlocks a fresh
// controller, the restore's cut is the eject, and the first pass cuts again --
// during the reboot settle, and the settle's end brings no control back.
TEST_F(FinsAtTheEject, ARebootAfterTheEjectCutsAgainOnItsFirstPass)
{
    Flight f;
    f.reboot = true;
    const Result r = fly(f);
    EXPECT_EQ(r.control_passes_after_eject, 0);
    EXPECT_EQ(r.driven_after_eject, 0);
    EXPECT_EQ(rail.offs, 2);   // the eject's, and the reboot's
    EXPECT_EQ(rail.ons, 1);    // the boot
    EXPECT_EQ(rail.cut_at_ms, f.reboot_at_ms + kRailDelayMs);
    EXPECT_FALSE(r.rail_on_at_end);
    EXPECT_TRUE(servo.outputsLocked());
    EXPECT_TRUE(pulsesCut());
}

// The next flight (a launch, or a sim's reset) raises the rail first and only
// then frees the outputs, so its first drive goes into a powered rail.
TEST_F(FinsAtTheEject, TheNextFlightRaisesTheRailThenFreesTheOutputs)
{
    FinEjectPolicy::State fins;
    FinEjectPolicy::drive(FinEjectPolicy::step(fins, true, 0, kRailDelayMs), servo, rail);
    FinEjectPolicy::drive(FinEjectPolicy::step(fins, true, kRailDelayMs, kRailDelayMs), servo, rail);
    ASSERT_FALSE(rail.on);
    // main.cpp's enterInflight / resetFlightStateForSim, in their order.
    FinEjectPolicy::reset(fins);
    rail(true);
    servo.lockOutputs(false);
    EXPECT_TRUE(rail.locked_when_raised);
    EXPECT_EQ(FinEjectPolicy::step(fins, false, 1000, kRailDelayMs), Fins::Powered);
    setMockMicros(1000000);
    servo.controlWithGainSchedule(-kRollRateDps, 50.0f);
    setMockMicros(1002000);
    servo.controlWithGainSchedule(-kRollRateDps, 50.0f);
    EXPECT_FALSE(servo.isIdle());
    EXPECT_FALSE(pulsesCut());
}

// The cut clears the rate loop's integrator that a 20 deg/s spin wound up,
// so the next flight's (or a sim's) first rate-null pass starts from zero.
TEST_F(FinsAtTheEject, TheCutClearsTheRateIntegrator)
{
    uint32_t t = 0;
    for (; t < kEjectMs; t += kTickMs)
    {
        setMockMicros((uint64_t)t * 1000u);
        servo.controlWithGainSchedule(-kRollRateDps, speedAt(t));
    }
    TR_ServoControl probe = servo;
    setMockMicros((uint64_t)t * 1000u);
    probe.controlWithGainSchedule(0.0f, 50.0f);
    ASSERT_GT(std::fabs(probe.getRollCmdDeg()), 0.5f);   // wound up

    FinEjectPolicy::drive(Fins::CutBegin, servo, rail);
    servo.lockOutputs(false);   // the next flight's
    t += kTickMs;
    setMockMicros((uint64_t)t * 1000u);
    servo.controlWithGainSchedule(0.0f, 50.0f);   // first pass after a reset: dt bootstrap
    t += kTickMs;
    setMockMicros((uint64_t)t * 1000u);
    servo.controlWithGainSchedule(0.0f, 50.0f);
    EXPECT_NEAR(servo.getRollCmdDeg(), 0.0f, 1e-3f);
}

}  // namespace
