// The mosaic-only image's rules for its one navigation filter
// (flight_computer/main/raw_nav_policy.h): the crash-loop rule, the transonic
// lockout's window, the navigation-block resends, the restart in flight and
// the restored flight's wait for the barometer.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "fc_rescue_policy.h"
#include "raw_nav_policy.h"

using namespace RawNavPolicy;

// ---------------------------------------------------------------- crash loop

TEST(RawNavCrashLoop, TheThirdBootAfterTwoCrashesSkipsTheFilter)
{
    // FcRescuePolicy counts each boot at boot: a power-on reads 1, the boot
    // after one crash 2, after two crashes 3
    using namespace FcRescuePolicy;
    const uint32_t image = 0x10000;
    BootDecision d = atBoot(Record{}, ResetClass::FreshStart, image);
    EXPECT_EQ(d.record.unproven_boots, 1);
    EXPECT_FALSE(crashLoop(false, d.record.unproven_boots));
    d = atBoot(d.record, ResetClass::Abnormal, image);   // crash 1
    EXPECT_EQ(d.mode, BootMode::Normal);
    EXPECT_FALSE(crashLoop(true, d.record.unproven_boots));
    d = atBoot(d.record, ResetClass::Abnormal, image);   // crash 2
    EXPECT_EQ(d.mode, BootMode::Normal);
    EXPECT_EQ(d.record.unproven_boots, 3);
    EXPECT_TRUE(crashLoop(true, d.record.unproven_boots));
    // a boot that ran its loop a minute clears the count: the next crash
    // boot starts the filter again
    const Record proven = onProven(d.record);
    d = atBoot(proven, ResetClass::Abnormal, image);
    EXPECT_FALSE(crashLoop(true, d.record.unproven_boots));
}

TEST(RawNavCrashLoop, APowerCycleAlwaysBringsTheFilterBack)
{
    EXPECT_FALSE(crashLoop(false, 3));
    EXPECT_FALSE(crashLoop(false, 255));
    using namespace FcRescuePolicy;
    Record r = freshRecord(7);
    r.unproven_boots = 5;
    const BootDecision d = atBoot(sealed(r), ResetClass::FreshStart, 7);
    EXPECT_EQ(d.record.unproven_boots, 1);
    EXPECT_FALSE(crashLoop(false, d.record.unproven_boots));
}

TEST(RawNavCrashLoop, ARescueRefusedForAFlightStillSkipsIt)
{
    // the fourth boot is a rescue candidate; with a flight on the OC it boots
    // normally, counted again, and the filter stays out
    using namespace FcRescuePolicy;
    Record r = freshRecord(9);
    r.unproven_boots = kMaxUnprovenBoots;
    const BootDecision d = atBoot(sealed(r), ResetClass::Abnormal, 9);
    ASSERT_EQ(d.mode, BootMode::RescueCandidate);
    const Record refused = onRescueRefused(d.record);
    EXPECT_TRUE(crashLoop(true, refused.unproven_boots));
}

// ---------------------------------------------------------- transonic lockout

namespace
{
MachInputs boost(float speed)
{
    MachInputs in;
    in.filter_has_run = true;
    in.speed_mps = speed;
    in.in_flight = true;
    return in;
}
MachInputs coast(float speed, uint32_t since_burnout_ms)
{
    MachInputs in = boost(speed);
    in.burnout = true;
    in.since_burnout_ms = since_burnout_ms;
    return in;
}
}  // namespace

TEST(RawNavMachLockout, ARealTransonicBoostLocksAndHoldsThroughTheCoast)
{
    bool locked = false;
    for (float v = 0.0f; v <= 300.0f; v += 10.0f)
        locked = machLockout(locked, boost(v));
    EXPECT_TRUE(locked);
    // burnout at 320; the coast slows through 260 and 245 still locked
    locked = machLockout(locked, coast(320.0f, 0));
    for (uint32_t t = 0; t < 4000; t += 100)
    {
        locked = machLockout(locked, coast(320.0f - 0.02f * (float)t, t));
        EXPECT_TRUE(locked) << t;
    }
    EXPECT_TRUE(machLockout(true, coast(240.0f, 4000)));   // not under 240 yet
    EXPECT_FALSE(machLockout(true, coast(239.9f, 4100)));
}

TEST(RawNavMachLockout, AFalseSpeedLateInTheCoastNeverLocksTheBarometerOut)
{
    // the single-source case: the filter reads 300 m/s near apogee
    EXPECT_FALSE(machLockout(false, coast(300.0f, 1000)));
    EXPECT_FALSE(machLockout(false, coast(1.0e4f, 8000)));
    // inside the burnout's second it may (the detector's latency, a tail-off)
    EXPECT_TRUE(machLockout(false, coast(261.0f, 999)));
}

TEST(RawNavMachLockout, AFilterThatHasNotRunHasNoSpeed)
{
    // a restore's start velocity, or the crash-loop boot's: unlocked, and a
    // lock carried over is cleared
    MachInputs in = boost(300.0f);
    in.filter_has_run = false;
    EXPECT_FALSE(machLockout(false, in));
    EXPECT_FALSE(machLockout(true, in));
}

TEST(RawNavMachLockout, OnTheGroundItIsOff)
{
    MachInputs in = boost(300.0f);
    in.in_flight = false;
    EXPECT_FALSE(machLockout(false, in));
    EXPECT_FALSE(machLockout(true, in));
}

TEST(RawNavMachLockout, ANonFiniteSpeedNeitherLocksNorUnlocks)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(machLockout(false, boost(nan)));
    EXPECT_TRUE(machLockout(true, boost(nan)));   // as GpsInsEKF's hysteresis held it
    EXPECT_TRUE(machLockout(true, boost(std::numeric_limits<float>::infinity())));
}

TEST(RawNavMachLockout, TheConfigsThresholdsAreUsed)
{
    MachConfig c;
    c.on_mps = 100.0f;
    c.off_mps = 90.0f;
    c.engage_after_burnout_ms = 0;
    EXPECT_TRUE(machLockout(false, boost(101.0f), c));
    EXPECT_FALSE(machLockout(false, coast(101.0f, 0), c));
    EXPECT_TRUE(machLockout(true, coast(95.0f, 5000), c));
    EXPECT_FALSE(machLockout(true, coast(89.0f, 5000), c));
}

// ------------------------------------------------- navigation blocks again

TEST(RawNavResend, OnThePadEveryThirtySecondsWhileASatelliteHasNoEphemeris)
{
    PadResend st;
    EXPECT_FALSE(padResendDue(st, 1000, true, 3));    // the boot's link setup asked just now
    EXPECT_FALSE(padResendDue(st, 30999, true, 3));
    EXPECT_TRUE(padResendDue(st, 31000, true, 3));
    EXPECT_FALSE(padResendDue(st, 31005, true, 3));
    EXPECT_FALSE(padResendDue(st, 60999, true, 3));
    EXPECT_TRUE(padResendDue(st, 61000, true, 1));
    EXPECT_EQ(st.sent, 2u);
}

TEST(RawNavResend, NotInFlightNorWithEveryEphemeris)
{
    PadResend st;
    (void)padResendDue(st, 0, true, 0);
    EXPECT_FALSE(padResendDue(st, 40000, false, 5));   // in flight: the launch's ask is the one
    EXPECT_FALSE(padResendDue(st, 50000, true, 0));
    EXPECT_FALSE(padResendDue(st, 50000, true, -1));
    EXPECT_TRUE(padResendDue(st, 50001, true, 1));     // back on the pad, still short of one
}

TEST(RawNavResend, TheClockWraps)
{
    PadResend st;
    (void)padResendDue(st, 0xFFFFF000u, true, 1);
    EXPECT_FALSE(padResendDue(st, 0x00001000u, true, 1));
    EXPECT_TRUE(padResendDue(st, 0xFFFFF000u + 30000u, true, 1));
}

TEST(RawNavResend, OnceAtPrelaunchAndOnceFiveSecondsIntoLanded)
{
    ResendEdges st;
    EXPECT_FALSE(edgeResendDue(st, false, false, 0));     // READY
    EXPECT_TRUE(edgeResendDue(st, true, false, 100));     // PRELAUNCH entry
    EXPECT_FALSE(edgeResendDue(st, true, false, 200));
    EXPECT_FALSE(edgeResendDue(st, false, false, 300));   // INFLIGHT
    EXPECT_FALSE(edgeResendDue(st, false, true, 10000));  // LANDED
    EXPECT_FALSE(edgeResendDue(st, false, true, 14999));
    EXPECT_TRUE(edgeResendDue(st, false, true, 15000));
    EXPECT_FALSE(edgeResendDue(st, false, true, 20000));
    EXPECT_FALSE(edgeResendDue(st, false, true, 90000));
}

TEST(RawNavResend, ALandedLeftBeforeFiveSecondsAsksNothing)
{
    // a sim's reset out of LANDED: the pad's own rule takes over
    ResendEdges st;
    EXPECT_FALSE(edgeResendDue(st, false, true, 0));
    EXPECT_FALSE(edgeResendDue(st, false, false, 2000));
    EXPECT_FALSE(edgeResendDue(st, false, false, 9000));
    EXPECT_TRUE(edgeResendDue(st, true, false, 9100));    // and PRELAUNCH again asks again
}

// ------------------------------------------------------- restart in flight

TEST(RawNavRestart, AtMostThreeAFlightOncePerStop)
{
    EXPECT_TRUE(restartDue(true, true, false, 0));
    EXPECT_TRUE(restartDue(true, true, false, 2));
    EXPECT_FALSE(restartDue(true, true, false, 3));
    EXPECT_FALSE(restartDue(true, true, true, 0));    // asked: waiting for it to run
    EXPECT_FALSE(restartDue(false, true, false, 0));  // not in flight
    EXPECT_FALSE(restartDue(true, false, false, 0));  // running
    EXPECT_TRUE(restartDue(true, true, false, 4, 5));
}

// ------------------------------------------------ a restored flight's start

TEST(RawNavRestoreHold, ItWaitsForTheBarometerThenStartsAtItsHeight)
{
    RestoreHold h;
    armRestoreHold(h, 1000);
    EXPECT_EQ(restoreSeedStep(h, true, false, 1000), RestoreSeed::Wait);
    EXPECT_EQ(restoreSeedStep(h, true, false, 3999), RestoreSeed::Wait);
    EXPECT_EQ(restoreSeedStep(h, true, true, 4000), RestoreSeed::Rebase);
    // decided once: the seeds after go as they are
    EXPECT_FALSE(h.armed);
    EXPECT_EQ(restoreSeedStep(h, true, true, 4100), RestoreSeed::Publish);
}

TEST(RawNavRestoreHold, WithoutABarometerInThreeSecondsTheSnapshotsHeight)
{
    RestoreHold h;
    armRestoreHold(h, 0xFFFFFF00u);   // across the clock's wrap
    EXPECT_EQ(restoreSeedStep(h, true, false, 0x00000200u), RestoreSeed::Wait);
    EXPECT_EQ(restoreSeedStep(h, true, false, 0xFFFFFF00u + 3000u), RestoreSeed::SnapshotHeight);
    EXPECT_EQ(restoreSeedStep(h, true, false, 0xFFFFFF00u + 3001u), RestoreSeed::Publish);
}

TEST(RawNavRestoreHold, NothingIsHeldWithoutARebootsRestore)
{
    RestoreHold h;
    EXPECT_EQ(restoreSeedStep(h, true, false, 0), RestoreSeed::Publish);   // a restart in flight: not armed
    EXPECT_EQ(restoreSeedStep(h, false, false, 0), RestoreSeed::Publish);
    armRestoreHold(h, 0);
    EXPECT_EQ(restoreSeedStep(h, false, false, 10), RestoreSeed::Publish); // no restore pending
    EXPECT_FALSE(h.armed);
}
