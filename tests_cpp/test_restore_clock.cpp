// #1777: the flight clock a restore in flight resumes on
// (restore_clock_policy.h).
//
// The FC resets in flight and restores from the OC's last snapshot, whose
// times are elapsed from launch. The restore rebases them on this boot's
// start, so the setup before the restore (the receiver's bring-up, the asks to
// the OC) stays in the flight's clock. The clock then misses only the
// snapshot's age at the reset and the reset itself: late by those, never early.
//
// The flight tests restore past apogee with a time-after-apogee channel owed,
// step the real recovery arm gate and its apogee hand-over at the flight
// loop's 1 ms, and fire on the flight loop's own trigger test. Times are kept
// in real time on the previous boot's clock, which runs on through the reset.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>

#include "RecoveryArmGate.h"
#include "flight_timeout_policy.h"
#include "pwr_hold_release_policy.h"
#include "restore_clock_policy.h"

using RestoreClock::eventMs;
using RestoreClock::launchMs;
using RestoreClock::timeAfterApogeeDue;

namespace {

constexpr float kG = 9.80665f;

// ---------------------------------------------------------------------------
// The rebase
// ---------------------------------------------------------------------------

TEST(RestoreClock, TheLaunchIsCountedFromThisBootsStart)
{
    const uint32_t elapsed = 30000;   // the snapshot: 30 s since launch
    const uint32_t launch = launchMs(elapsed);
    // the restore 1.5 s into this boot: those 1.5 s are flight time too
    EXPECT_EQ(1500u - launch, 31500u);
    EXPECT_EQ(0u - launch, elapsed) << "at this boot's start the snapshot's own time";
    // a flight older than the boot wraps below 0, as before #1777
    EXPECT_GT(launch, 1500u);
}

TEST(RestoreClock, ApogeeAndBurnoutKeepTheirPlaceFromLaunch)
{
    const uint32_t launch = launchMs(42000);
    const uint32_t apogee = eventMs(launch, 31000);
    const uint32_t burnout = eventMs(launch, 2800);
    EXPECT_EQ(apogee - launch, 31000u);
    EXPECT_EQ(burnout - launch, 2800u);
    // 2 s into this boot: 11 s past apogee by the snapshot, plus this boot
    EXPECT_EQ(2000u - apogee, 13000u);
    EXPECT_EQ(2000u - burnout, 41200u);
}

TEST(RestoreClock, TheTimeAfterApogeeTestAtItsEdge)
{
    EXPECT_TRUE(timeAfterApogeeDue(25000, 5000, 20.0f));
    EXPECT_FALSE(timeAfterApogeeDue(24999, 5000, 20.0f));
    EXPECT_TRUE(timeAfterApogeeDue(6500, 5000, 1.5f));
    EXPECT_FALSE(timeAfterApogeeDue(6499, 5000, 1.5f));
    // a restored apogee before this boot's start (wrapped)
    const uint32_t apogee = 0u - 3000u;
    EXPECT_TRUE(timeAfterApogeeDue(17000, apogee, 20.0f));
    EXPECT_FALSE(timeAfterApogeeDue(16999, apogee, 20.0f));
    EXPECT_TRUE(timeAfterApogeeDue(0, 0, 0.0f)) << "no delay: due at apogee";
}

// ---------------------------------------------------------------------------
// A restore past apogee with a time-after-apogee channel owed
// ---------------------------------------------------------------------------

// How the restore puts the snapshot's launch on this boot's clock.
using Rebase = uint32_t (*)(uint32_t restore_ms, uint32_t flight_elapsed_ms);
uint32_t rebaseOnBootStart(uint32_t, uint32_t e) { return launchMs(e); }
uint32_t rebaseOnRestore(uint32_t restore_ms, uint32_t e) { return restore_ms - e; }   // before #1777

struct Case
{
    float    delay_s;                 // the channel: time after apogee
    uint32_t reset_after_apogee_ms;   // the FC alone resets this long after apogee
    uint32_t snap_age_ms;             // the last snapshot's age at the reset: under 100 while
                                      // the loop runs, seconds after a watchdog's reset
    uint32_t down_ms;                 // the reset and the bootloader: before this boot's clock
    uint32_t boot_ms;                 // this boot's setup, to the restore
};

struct Outcome
{
    bool    fired = false;
    int64_t fire_ms = 0;       // real time after the real apogee
    int64_t handover_ms = -1;  // real time after the real apogee: the gate hands the apogee over
    int64_t correct_ms = 0;    // the earliest it may fire: max(delay, hand-over)
    int64_t late_ms = 0;       // fire_ms - correct_ms
};

RecoveryArmGate::Inputs underTheDrogue(uint32_t now_ms, uint32_t launch_ms)
{
    RecoveryArmGate::Inputs in;
    in.now_ms            = now_ms;
    in.flight_elapsed_ms = now_ms - launch_ms;   // main.cpp: now_ms - launch_time_millis
    in.baro_healthy      = true;
    in.baro_rate_mps     = -20.0f;
    in.imu_fresh         = true;
    in.accel_norm_ms2    = kG;
    in.gyro_norm_dps     = 10.0f;
    return in;
}

Outcome fly(const Case& c, Rebase rebase = rebaseOnBootStart)
{
    // The previous boot, on its own clock.
    const uint32_t L = 4000;                           // launch
    const uint32_t A = L + 31000;                      // apogee
    const uint32_t R = A + c.reset_after_apogee_ms;    // the reset
    const uint32_t S = R - c.snap_age_ms;              // the last snapshot sent
    // buildFlightSnapshot's two times
    const uint32_t flight_elapsed = S - L;
    const uint32_t apogee_elapsed = A - L;
    // This boot's clock reads 0 down_ms after the reset.
    const auto real = [&](uint32_t t) { return (int64_t)R + c.down_ms + t; };

    const uint32_t restore = c.boot_ms;
    const uint32_t launch = rebase(restore, flight_elapsed);
    const uint32_t apogee = eventMs(launch, apogee_elapsed);
    RecoveryArmGate::State gate;
    RecoveryArmGate::reset(gate, restore);
    bool handed = false;   // recovery_apogee_armed

    Outcome o;
    for (uint32_t t = restore; t < restore + 120000; ++t)
    {
        RecoveryArmGate::step(gate, underTheDrogue(t, launch));
        if (!handed && RecoveryArmGate::apogeeArmed(gate, t))
        {
            handed = true;
            o.handover_ms = real(t) - A;
        }
        // servicePyroChannels: a channel leaves Idle only through the open
        // gate, and the restored apogee counts only once it is handed over
        if (RecoveryArmGate::armingPermitted(gate) && handed &&
            timeAfterApogeeDue(t, apogee, c.delay_s))
        {
            o.fired = true;
            o.fire_ms = real(t) - A;
            break;
        }
    }
    o.correct_ms = std::max<int64_t>((int64_t)(c.delay_s * 1000.0f), o.handover_ms);
    o.late_ms = o.fire_ms - o.correct_ms;
    return o;
}

// The issue's table (a snapshot 80 ms old, no time for the reset), plus a
// reset and bootloader of 300 ms, a 70 s boot, a hung loop the task watchdog
// reset (its snapshot 5 s old), and the never-early edge: a fresh snapshot, no
// time lost to the reset, and the delay deciding.
class RestoredTimerChannel : public ::testing::TestWithParam<Case> {};

TEST_P(RestoredTimerChannel, FiresNeverEarlyAndLateOnlyByTheSnapshotAndTheReset)
{
    const Case c = GetParam();
    const Outcome o = fly(c);
    ASSERT_TRUE(o.fired);
    ASSERT_GE(o.handover_ms, 0);
    EXPECT_GE(o.fire_ms, (int64_t)(c.delay_s * 1000.0f)) << "never before apogee + its delay";
    EXPECT_GE(o.fire_ms, o.handover_ms) << "never before the gate hands the apogee over";
    EXPECT_LE(o.late_ms, (int64_t)(c.snap_age_ms + c.down_ms) + 1)
        << "this boot's " << c.boot_ms << " ms of setup are not lost";
}

INSTANTIATE_TEST_SUITE_P(
    RestoreClock, RestoredTimerChannel,
    ::testing::Values(
        Case{15.0f, 8000, 80, 0, 1000},
        Case{15.0f, 8000, 80, 0, 4000},
        Case{15.0f, 8000, 80, 0, 15000},
        Case{20.0f, 5000, 80, 0, 15000},
        Case{1.0f, 600, 80, 0, 1500},
        Case{15.0f, 8000, 80, 300, 1000},
        Case{20.0f, 5000, 99, 300, 15000},
        Case{40.0f, 2000, 0, 0, 70000},
        Case{30.0f, 8000, 5000, 300, 1500},
        Case{30.0f, 5000, 0, 0, 1500}));

TEST(RestoreClock, WhenTheDelayStillOwedDecidesItTheBootChangesNothing)
{
    // a 30 s channel, 5 s past apogee: the hand-over comes well before the
    // delay is up, whatever this boot took
    for (uint32_t boot : {500u, 1500u, 4000u, 15000u})
    {
        const Outcome o = fly(Case{30.0f, 5000, 60, 250, boot});
        ASSERT_TRUE(o.fired) << boot;
        EXPECT_EQ(o.correct_ms, 30000) << boot;
        EXPECT_EQ(o.fire_ms, 30000 + 60 + 250) << "late by the snapshot and the reset only, boot " << boot;
    }
}

TEST(RestoreClock, RebasedOnTheRestoreItLostThisBootsSetup)
{
    // The clock before #1777, on the issue's first row: the channel fired a
    // whole boot late, so the cases above tell the two apart.
    const Case c{15.0f, 8000, 80, 0, 1000};
    const Outcome before = fly(c, rebaseOnRestore);
    ASSERT_TRUE(before.fired);
    EXPECT_EQ(before.fire_ms, 16080);
    EXPECT_EQ(before.late_ms, 1080);
    const Case slow{20.0f, 5000, 80, 0, 15000};
    EXPECT_EQ(fly(slow, rebaseOnRestore).fire_ms, 35080) << "the issue's 12.4 s late";
    EXPECT_EQ(fly(slow).fire_ms, fly(slow).handover_ms) << "now at the hand-over";
}

// ---------------------------------------------------------------------------
// A second reset, and the other clocks counted from launch
// ---------------------------------------------------------------------------

TEST(RestoreClock, ASecondResetLosesOnlyItsOwnSnapshotAndReset)
{
    // Real flight time on the previous boot's clock, from launch.
    const uint32_t A = 31000;   // apogee
    // first reset: the last snapshot 70 ms old, 200 ms down, 1.2 s of setup
    uint32_t fe = 40000 - 70;
    uint32_t real_at_boot_start = 40000 + 200;
    uint32_t launch = launchMs(fe);
    uint32_t apogee = eventMs(launch, A);
    // the restored flight sends snapshots on this boot's clock
    // (buildFlightSnapshot): 8 s into this boot, one 40 ms before the next reset
    const uint32_t t_snap = 8000 - 40;
    const uint32_t fe2 = t_snap - launch;
    const uint32_t ae2 = apogee - launch;
    EXPECT_EQ(ae2, A) << "the apogee round-trips through this boot's snapshots";
    EXPECT_EQ(fe2, real_at_boot_start + t_snap - 70 - 200);
    // second reset at 8 s into this boot, 150 ms down
    real_at_boot_start += 8000 + 150;
    launch = launchMs(fe2);
    apogee = eventMs(launch, ae2);
    for (uint32_t t : {900u, 2500u, 20000u})
    {
        EXPECT_EQ(real_at_boot_start + t - (t - launch), 70u + 200u + 40u + 150u)
            << "late by both snapshots' ages and both resets, at " << t;
        EXPECT_EQ((real_at_boot_start + t - A) - (t - apogee), 70u + 200u + 40u + 150u);
    }
}

TEST(RestoreClock, TheTimeoutCountsThisBootsSetup)
{
    // No channel owed, the base bound: 600 s since launch
    const FlightTimeoutPolicy::In in{false, false, false, 0};
    const uint32_t launch = launchMs(598500);
    EXPECT_TRUE(FlightTimeoutPolicy::expired(2000u - launch, in))
        << "600.5 s since launch at the restore, 2 s into this boot";
    EXPECT_FALSE(FlightTimeoutPolicy::expired(1500u - launch, in));
    EXPECT_FALSE(FlightTimeoutPolicy::expired(2000u - rebaseOnRestore(2000, 598500), in))
        << "before #1777: 2 s short";
}

TEST(RestoreClock, TheGatesFlightAgeCountsThisBootsSetup)
{
    // A flight past the FC's own ceiling is over: the gate refuses it on its
    // first step, counting the setup before the restore as flight time.
    const RecoveryArmGate::Config cfg;
    const uint32_t restore = 2000;
    const uint32_t launch = launchMs(cfg.max_flight_ms - 1500);
    RecoveryArmGate::State gate;
    RecoveryArmGate::reset(gate, restore);
    RecoveryArmGate::step(gate, underTheDrogue(restore, launch));
    EXPECT_TRUE(RecoveryArmGate::refuted(gate));
}

TEST(RestoreClock, AtLandingARestoredFlightCountsAnOcAnswerOnlyFromTheRestore)
{
    // The FC resets 0.3 s after launch; this boot asks the OC 0.5 s in and
    // restores 1.5 s in; the status poll stays off until LANDED.
    const uint32_t restore = 1500;
    for (uint32_t elapsed : {300u, 900000u})
    {
        const uint32_t launch = launchMs(elapsed);
        const uint32_t from = pwr_hold::answerFromMs(launch, true, restore);
        EXPECT_FALSE(pwr_hold::ocAliveSinceLaunch(true, 500, from))
            << "the setup's ask is no evidence at landing, flight " << elapsed;
        EXPECT_FALSE(pwr_hold::ocAliveSinceLaunch(true, restore, from)) << elapsed;
        EXPECT_TRUE(pwr_hold::ocAliveSinceLaunch(true, restore + 1, from))
            << "a poll after the restore is, flight " << elapsed;
        EXPECT_FALSE(pwr_hold::ocAliveSinceLaunch(false, restore + 1, from));
        // from the launch, the ask reads as from after it
        EXPECT_TRUE(pwr_hold::ocAliveSinceLaunch(true, 500, launch)) << elapsed;
    }
    // a flight this boot launched: from its launch, as before
    EXPECT_EQ(pwr_hold::answerFromMs(42000, false, 1500), 42000u);
    EXPECT_FALSE(pwr_hold::ocAliveSinceLaunch(true, 41000, pwr_hold::answerFromMs(42000, false, 0)));
    EXPECT_TRUE(pwr_hold::ocAliveSinceLaunch(true, 42001, pwr_hold::answerFromMs(42000, false, 0)));
}

}  // namespace
