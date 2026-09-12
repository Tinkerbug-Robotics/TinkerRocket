// #825: should an OC boot re-assert the FC power rail? The deliberate_off
// row is load-bearing in the DANGEROUS direction: the power-off command is
// IMPLEMENTED as a reboot, so restoring the rail on that reboot would undo
// the operator's command — while failing to restore on a fault reset is the
// original mid-flight ballistic failure.

#include <gtest/gtest.h>

#include "rail_restore_policy.h"

using RailRestorePolicy::shouldRestore;

TEST(RailRestorePolicy, FaultResetWithRailOnRestores) {
    // Panic/WDT/brownout mid-session: the #825 case.
    EXPECT_TRUE(shouldRestore(false, true, true, false, 0));
}

TEST(RailRestorePolicy, SelfOtaRestartRestores) {
    // The OTA esp_restart is a SW reset with rail_on retained — same row.
    EXPECT_TRUE(shouldRestore(false, true, true, false, 0));
}

TEST(RailRestorePolicy, DeliberatePowerOffStandsDown) {
    // The reboot that IMPLEMENTS power-off must boot rail-LOW.
    EXPECT_FALSE(shouldRestore(false, true, true, true, 0));
    // ...even if rail_on were stale-true alongside it.
    EXPECT_FALSE(shouldRestore(false, true, false, true, 0));
}

TEST(RailRestorePolicy, ColdPowerOnNeverRestores) {
    EXPECT_FALSE(shouldRestore(true, true, true, false, 0));
    EXPECT_FALSE(shouldRestore(true, false, false, false, 0));
}

TEST(RailRestorePolicy, GarbageRtcNeverRestores) {
    EXPECT_FALSE(shouldRestore(false, false, true, false, 0));
}

TEST(RailRestorePolicy, RailWasOffStaysOff) {
    EXPECT_FALSE(shouldRestore(false, true, false, false, 0));
}

TEST(RailRestorePolicy, RetryBudgetBoundsTheBrownoutLoop) {
    // A pack sagging under the restored load browns the OC out repeatedly;
    // after kMaxRestoreAttempts consecutive restore boots the policy stands
    // down to the stable rail-off idle (the pre-#825 endpoint).
    for (uint8_t a = 0; a < RailRestorePolicy::kMaxRestoreAttempts; ++a) {
        EXPECT_TRUE(shouldRestore(false, true, true, false, a)) << int(a);
    }
    EXPECT_FALSE(shouldRestore(false, true, true, false,
                               RailRestorePolicy::kMaxRestoreAttempts));
    EXPECT_FALSE(shouldRestore(false, true, true, false, 255));
}

// ── #1129: the budget is cleared by proof, not by the OC's own init ─────────
//
// restore_attempts was zeroed on the first loop_oc pass, right after
// initPeripherals() — seconds BEFORE the FC applies the load the budget
// exists to bound (GPS_ACT, then servo_control.wiggle(), which the FC's own
// comment puts at 4.2 s). So every brownout restarted from zero, shouldRestore
// kept returning true, and the stand-down branch was unreachable: a sagging
// pack cycled the FC rail forever instead of settling into the stable rail-off
// idle pre-#825 firmware reached after one brownout.

TEST(RailRestore, ABootThatDidNotRestoreHasNothingToProve) {
    EXPECT_FALSE(RailRestorePolicy::restoreProven(false, true, 60000));
    EXPECT_FALSE(RailRestorePolicy::restoreProven(false, false, 60000));
}

TEST(RailRestore, TheOcsOwnInitIsNotProof) {
    // The bug, stated directly: at the moment the old code cleared the budget
    // the FC had sent nothing and the uptime was under a second.
    EXPECT_FALSE(RailRestorePolicy::restoreProven(true, false, 800));
}

TEST(RailRestore, AnEarlyFcFrameIsNotProofEither) {
    // A warm FC that never actually dropped can be sending frames well before
    // it has re-applied the load. Requiring the uptime too is what makes this
    // "survived the load" rather than "is alive".
    EXPECT_FALSE(RailRestorePolicy::restoreProven(true, true, 1000));
    EXPECT_FALSE(RailRestorePolicy::restoreProven(
        true, true, RailRestorePolicy::kRestoreProvenMs - 1));
}

TEST(RailRestore, UptimeAloneIsNotProofWithASilentFc) {
    // The rail being on is the premise, not the proof — an FC that is
    // brownout-looping never sends a frame, and that is exactly the case the
    // budget must keep counting.
    EXPECT_FALSE(RailRestorePolicy::restoreProven(true, false, 60000));
    EXPECT_FALSE(RailRestorePolicy::restoreProven(true, false, 600000));
}

TEST(RailRestore, AFcHeardFromUnderLoadClearsTheBudget) {
    EXPECT_TRUE(RailRestorePolicy::restoreProven(
        true, true, RailRestorePolicy::kRestoreProvenMs));
    EXPECT_TRUE(RailRestorePolicy::restoreProven(true, true, 60000));
}

TEST(RailRestore, TheProofWindowClearsTheFcsServoWiggle) {
    // The FC's own comment: "servo wiggle alone can be 4.2 s", and it starts
    // after GPS_ACT. If this ever drops below that, the budget is being
    // cleared before the load again.
    EXPECT_GT(RailRestorePolicy::kRestoreProvenMs, 4200u);
}

