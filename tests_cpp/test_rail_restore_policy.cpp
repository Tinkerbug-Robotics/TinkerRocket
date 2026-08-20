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
