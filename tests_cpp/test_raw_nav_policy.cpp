// The mosaic-only image's rules for its one navigation filter
// (flight_computer/main/raw_nav_policy.h): the crash-loop rule, the transonic
// lockout's window, its restore and its release on an unhealthy filter, the
// navigation-block resends, the restart in flight and the restored flight's
// wait for the barometer.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "fc_rescue_policy.h"
#include "raw_nav_policy.h"

using namespace RawNavPolicy;

// ---------------------------------------------------------------- crash loop

namespace
{
constexpr uint32_t kImage = 0x10000;

// One boot as main.cpp counts it: FcRescuePolicy's record and the filter's
// own crash count, both at boot (setup_fc: fcRescueAtBoot, crashAtBoot)
struct Boards
{
    FcRescuePolicy::Record rescue{};
    CrashRecord crash{};
    FcRescuePolicy::BootDecision boot(bool crash_reset, uint32_t image = kImage)
    {
        using namespace FcRescuePolicy;
        const ResetClass rc = crash_reset ? ResetClass::Abnormal : ResetClass::FreshStart;
        BootDecision d = atBoot(rescue, rc, image);
        rescue = d.record;
        crash = crashAtBoot(crash, crash_reset, image);
        return d;
    }
    void runAMinute()   // fcMaybeMarkBootProven
    {
        rescue = FcRescuePolicy::onProven(rescue);
        crash = crashOnProven(crash);
    }
};
}  // namespace

TEST(RawNavCrashLoop, InFlightTheThirdBootAfterTwoCrashesSkipsTheFilter)
{
    // the pad's boot always runs its minute: the first crash ends a proven
    // boot, and still counts
    Boards b;
    b.boot(false);                     // power-on on the pad
    EXPECT_FALSE(crashLoop(b.crash));
    b.runAMinute();                    // minutes on the pad
    b.boot(true);                      // crash 1, in flight: the restore boot
    EXPECT_EQ(b.crash.crashes, 1);
    EXPECT_FALSE(crashLoop(b.crash));  // the filter starts again
    const FcRescuePolicy::BootDecision d = b.boot(true);   // crash 2, 5 s into it
    EXPECT_EQ(d.mode, FcRescuePolicy::BootMode::Normal);
    EXPECT_EQ(b.crash.crashes, 2);
    EXPECT_TRUE(crashLoop(b.crash));   // the third boot: no filter
    // FcRescuePolicy's count reads 2 here, not 3: it cannot tell this from
    // the bench's case below on its own
    EXPECT_EQ(b.rescue.unproven_boots, 2);
}

TEST(RawNavCrashLoop, OnTheBenchTheThirdBootToo)
{
    // the power-on's boot crashes inside its first minute (a sim started at
    // once): the same third boot, though FcRescuePolicy's count reads 3
    Boards b;
    b.boot(false);
    b.boot(true);
    EXPECT_FALSE(crashLoop(b.crash));
    b.boot(true);
    EXPECT_TRUE(crashLoop(b.crash));
    EXPECT_EQ(b.rescue.unproven_boots, 3);
}

TEST(RawNavCrashLoop, AMinuteOfLoopEndsTheRun)
{
    Boards b;
    b.boot(false);
    b.boot(true);
    b.runAMinute();                    // the filter flew a minute this time
    b.boot(true);
    EXPECT_EQ(b.crash.crashes, 1);
    EXPECT_FALSE(crashLoop(b.crash));
}

TEST(RawNavCrashLoop, APowerCycleAlwaysBringsTheFilterBack)
{
    Boards b;
    b.boot(false);
    b.boot(true);
    b.boot(true);
    b.boot(true);
    ASSERT_TRUE(crashLoop(b.crash));
    b.boot(false);                     // power cycle, or a brownout
    EXPECT_EQ(b.crash.crashes, 0);
    EXPECT_FALSE(crashLoop(b.crash));
    // and garbage in RTC memory after a cold boot is never read
    CrashRecord junk;
    std::memset(&junk, 0xA5, sizeof(junk));
    EXPECT_FALSE(crashValid(junk));
    EXPECT_FALSE(crashLoop(junk));
    EXPECT_EQ(crashAtBoot(junk, true, kImage).crashes, 0);
    EXPECT_EQ(crashAtBoot(junk, false, kImage).crashes, 0);
}

TEST(RawNavCrashLoop, AnotherImageStartsOver)
{
    // an update's restart, or a rollback: another partition, another count
    Boards b;
    b.boot(false);
    b.boot(true);
    b.boot(true, kImage + 1);
    EXPECT_EQ(b.crash.crashes, 0);
    EXPECT_FALSE(crashLoop(b.crash));
}

TEST(RawNavCrashLoop, ARescueRefusedForAFlightStillSkipsIt)
{
    // a crash loop long enough for rescue mode: with a flight on the OC the
    // candidate boots normally, and the filter stays out
    using namespace FcRescuePolicy;
    Boards b;
    b.boot(false);
    b.boot(true);
    b.boot(true);   // the filter's crash loop skips it; something else crashes
    ASSERT_TRUE(crashLoop(b.crash));
    const BootDecision d = b.boot(true);
    ASSERT_EQ(d.mode, BootMode::RescueCandidate);
    b.rescue = onRescueRefused(d.record);
    EXPECT_EQ(b.crash.crashes, 3);
    EXPECT_TRUE(crashLoop(b.crash));
}

// ---------------------------------------------------------- transonic lockout

namespace
{
MachInputs boost(float speed)
{
    MachInputs in;
    in.filter_has_run = true;
    in.filter_healthy = true;
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
    // the crash-loop boot's start velocity, or a restore's under the
    // lockout's speed: unlocked, and a lock carried over is cleared
    MachInputs in = boost(300.0f);
    in.filter_has_run = false;
    in.filter_healthy = false;
    EXPECT_FALSE(machLockout(false, in));
    EXPECT_FALSE(machLockout(true, in));
}

TEST(RawNavMachLockout, ARestoreInTheTransonicCoastLocksAgain)
{
    // a reboot 3 s past burnout at 280 m/s: the snapshot over 260 locks it
    // from the restore (main.cpp sets it there), the restored filter's own
    // speed holds it, and under 240 lets it go
    EXPECT_TRUE(restoreLocks(true, 280.0f));
    EXPECT_FALSE(restoreLocks(true, 250.0f));
    EXPECT_FALSE(restoreLocks(false, 280.0f));   // no filter this boot: nothing would release it

    MachInputs in = coast(280.0f, 3000);
    in.filter_has_run = false;
    in.filter_healthy = false;
    in.restored_at_speed = true;
    in.since_restore_ms = 0;
    bool locked = true;   // as the restore leaves it
    for (uint32_t t = 0; t <= 400; t += 100)
    {
        in.since_restore_ms = t;   // waiting for its first running state
        locked = machLockout(locked, in);
        EXPECT_TRUE(locked) << t;
    }
    // the restored filter runs, still fast, 3.5 s past burnout
    in.filter_has_run = true;
    in.filter_healthy = true;
    in.since_burnout_ms = 3500;
    for (float v = 279.0f; v >= 241.0f; v -= 2.0f)
    {
        in.speed_mps = v;
        locked = machLockout(locked, in);
        EXPECT_TRUE(locked) << v;
    }
    in.speed_mps = 239.0f;
    EXPECT_FALSE(machLockout(locked, in));
    // the same reboot without the restore's lock never locks (the window is shut)
    MachInputs late = coast(280.0f, 3500);
    EXPECT_FALSE(machLockout(false, late));
}

TEST(RawNavMachLockout, ARestoresLockEndsIfTheFilterNeverRuns)
{
    MachInputs in = coast(280.0f, 3000);
    in.filter_has_run = false;
    in.filter_healthy = false;
    in.restored_at_speed = true;
    in.since_restore_ms = 4999;
    EXPECT_TRUE(machLockout(true, in));
    in.since_restore_ms = 5000;
    EXPECT_FALSE(machLockout(true, in));
    // and a sim's reset or a landing (not in flight) ends it too
    in.since_restore_ms = 0;
    in.in_flight = false;
    EXPECT_FALSE(machLockout(true, in));
}

TEST(RawNavMachLockout, AStoppedFiltersHeldSpeedLetsTheLockGoAfterASecond)
{
    // locked in the burn; the filter then stops and its restarts are spent:
    // the held state's speed (a free INS, here still over 240) holds the
    // lock no more than a second past its last healthy read
    MachInputs in = coast(300.0f, 2000);
    in.filter_healthy = false;
    in.unhealthy_ms = 0;
    EXPECT_TRUE(machLockout(true, in));
    in.unhealthy_ms = 999;
    EXPECT_TRUE(machLockout(true, in));
    in.unhealthy_ms = 1000;
    EXPECT_FALSE(machLockout(true, in));
    // nor does a held state's speed begin one, even in the burn
    MachInputs burn = boost(300.0f);
    burn.filter_healthy = false;
    EXPECT_FALSE(machLockout(false, burn));
}

TEST(RawNavMachLockout, ANonFiniteHeldSpeedLetsTheLockGoToo)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    MachInputs in = coast(nan, 2000);
    in.filter_healthy = false;   // a NaN state is never healthy
    in.unhealthy_ms = 500;
    EXPECT_TRUE(machLockout(true, in));
    in.unhealthy_ms = 1500;
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
    c.unhealthy_release_ms = 200;
    MachInputs sick = coast(95.0f, 5000);
    sick.filter_healthy = false;
    sick.unhealthy_ms = 199;
    EXPECT_TRUE(machLockout(true, sick, c));
    sick.unhealthy_ms = 200;
    EXPECT_FALSE(machLockout(true, sick, c));
    c.restore_lock_max_ms = 300;
    MachInputs restored = coast(95.0f, 5000);
    restored.filter_has_run = false;
    restored.restored_at_speed = true;
    restored.since_restore_ms = 299;
    EXPECT_TRUE(machLockout(true, restored, c));
    restored.since_restore_ms = 300;
    EXPECT_FALSE(machLockout(true, restored, c));
    EXPECT_TRUE(restoreLocks(true, 101.0f, c));
    EXPECT_FALSE(restoreLocks(true, 99.0f, c));
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

TEST(RawNavRestoreHold, ALockedOutBarometerStartsItAtTheSnapshotsHeightAtOnce)
{
    // a restore in the transonic coast: the barometer cannot vouch for a
    // height, so neither a wait nor a rebase on it
    RestoreHold h;
    armRestoreHold(h, 1000);
    EXPECT_EQ(restoreSeedStep(h, true, true, 1000, true), RestoreSeed::Locked);
    EXPECT_FALSE(h.armed);
    EXPECT_EQ(restoreSeedStep(h, true, true, 1001, true), RestoreSeed::Publish);
    // unlocked, the same pass waits or rebases as before
    armRestoreHold(h, 1000);
    EXPECT_EQ(restoreSeedStep(h, true, false, 1000, false), RestoreSeed::Wait);
    EXPECT_EQ(restoreSeedStep(h, true, true, 1100, false), RestoreSeed::Rebase);
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
