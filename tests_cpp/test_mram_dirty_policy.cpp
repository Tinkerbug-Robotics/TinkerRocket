// #417: MramDirtyPolicy — the sink-mode "dirty" marker must mean "logging was
// activated" (real launch, or a deliberate manual cmd-23 start), NOT merely "a
// log file was pre-created at PRELAUNCH". The old behavior set the marker at
// openLogSession()/pre-create and only cleared it on a clean close, so a bench
// power-cut from PRELAUNCH (never launched, never cleanly closed) left the
// marker set and spawned a bogus flight_mram_recovered_*.bin on nearly every
// power cycle. These tests pin the fixed decision table.
//
// Convention mirrored from TR_LogToFlash's lifecycle:
//   onPreCreate()        -> PRELAUNCH pre-create (must NOT dirty)
//   onActivate()         -> launch detect / manual cmd-23 start (dirties)
//   onCloseClean()       -> end-of-flight drain / manual stop (clears)
//   onRecoveryFinished() -> boot recovery drained (clears)
// A power-cut is modeled by simply *stopping* — whatever markerShouldBeSet()
// returns at that point is what survives in non-volatile MRAM to the next boot,
// which shouldRecoverOnBoot() then gates on.

#include <gtest/gtest.h>

#include "mram_dirty_policy.h"

TEST(MramDirtyPolicy, DefaultsClean)
{
    MramDirtyPolicy p;
    EXPECT_FALSE(p.markerShouldBeSet());
    EXPECT_FALSE(MramDirtyPolicy::shouldRecoverOnBoot(p.markerShouldBeSet()));
}

// Bench case (a): power-cycle from READY, no session ever opened.
TEST(MramDirtyPolicy, ReadyPowerCycle_NoRecovery)
{
    MramDirtyPolicy p;
    // (power cut) — nothing happened.
    EXPECT_FALSE(p.markerShouldBeSet());
    EXPECT_FALSE(MramDirtyPolicy::shouldRecoverOnBoot(p.markerShouldBeSet()));
}

// Bench case (b) — the #417 regression: PRELAUNCH pre-creates the file, then
// power is cut before any launch. The marker must NOT survive set.
TEST(MramDirtyPolicy, PreCreateThenPowerCut_NoRecovery)
{
    MramDirtyPolicy p;
    p.onPreCreate();
    // (power cut — never launched, never cleanly closed)
    EXPECT_FALSE(p.markerShouldBeSet());
    EXPECT_FALSE(MramDirtyPolicy::shouldRecoverOnBoot(p.markerShouldBeSet()));
}

// The genuine #274 case: launch detected (activate), then an in-flight
// brownout. The surviving marker must gate recovery ON.
TEST(MramDirtyPolicy, ActivateThenBrownout_Recovers)
{
    MramDirtyPolicy p;
    p.onPreCreate();
    p.onActivate();
    // (in-flight brownout)
    EXPECT_TRUE(p.markerShouldBeSet());
    EXPECT_TRUE(MramDirtyPolicy::shouldRecoverOnBoot(p.markerShouldBeSet()));
}

// Bench case (c): a clean flight — pre-create, activate, clean close. The
// marker is cleared; a later power-cut leaves nothing to recover.
TEST(MramDirtyPolicy, CleanFlight_NoRecovery)
{
    MramDirtyPolicy p;
    p.onPreCreate();
    p.onActivate();
    EXPECT_TRUE(p.markerShouldBeSet());   // marker live during the flight
    p.onCloseClean();
    // (power cut, later)
    EXPECT_FALSE(p.markerShouldBeSet());
    EXPECT_FALSE(MramDirtyPolicy::shouldRecoverOnBoot(p.markerShouldBeSet()));
}

// Manual cmd-23 start routes through activate without necessarily a PRELAUNCH
// pre-create; a power-cut mid manual-logging still has real ring data.
TEST(MramDirtyPolicy, ManualStartThenPowerCut_Recovers)
{
    MramDirtyPolicy p;
    p.onActivate();   // deliberate manual start
    // (power cut mid-logging)
    EXPECT_TRUE(p.markerShouldBeSet());
    EXPECT_TRUE(MramDirtyPolicy::shouldRecoverOnBoot(p.markerShouldBeSet()));
}

// Manual start followed by a manual stop clears the marker like a clean flight.
TEST(MramDirtyPolicy, ManualStartThenManualStop_NoRecovery)
{
    MramDirtyPolicy p;
    p.onActivate();
    p.onCloseClean();
    EXPECT_FALSE(p.markerShouldBeSet());
    EXPECT_FALSE(MramDirtyPolicy::shouldRecoverOnBoot(p.markerShouldBeSet()));
}

// After a genuine recovery drains at boot, finishing it clears the marker so
// recovery does not re-fire on the following boot.
TEST(MramDirtyPolicy, RecoveryFinishedClearsMarker)
{
    MramDirtyPolicy p;
    p.onActivate();
    EXPECT_TRUE(p.markerShouldBeSet());   // survived a brownout into this boot
    p.onRecoveryFinished();
    EXPECT_FALSE(p.markerShouldBeSet());
    EXPECT_FALSE(MramDirtyPolicy::shouldRecoverOnBoot(p.markerShouldBeSet()));
}

// A second, aborted arm cycle after a clean flight must not resurrect the
// marker on the pre-create step (guards the #417 mechanism across re-arms).
TEST(MramDirtyPolicy, RearmPreCreateAfterCleanFlight_StaysClean)
{
    MramDirtyPolicy p;
    p.onPreCreate();
    p.onActivate();
    p.onCloseClean();
    // Re-arm: PRELAUNCH again, but power is cut before the next launch.
    p.onPreCreate();
    EXPECT_FALSE(p.markerShouldBeSet());
    EXPECT_FALSE(MramDirtyPolicy::shouldRecoverOnBoot(p.markerShouldBeSet()));
}
