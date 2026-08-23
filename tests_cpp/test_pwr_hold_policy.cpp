// #848: boot reconciliation for the P4_EN_HOLD power latch. The full truth
// table — this decides whether a rebooting FC keeps holding its own power
// rail, and the KeepHold row is the difference between a recoverable downed
// rocket (GNSS tracker alive) and a dead one.

#include <gtest/gtest.h>

#include "pwr_hold_policy.h"

using PwrHoldPolicy::BootAction;
using PwrHoldPolicy::bootAction;

TEST(PwrHoldPolicy, ColdBootDoesNothing) {
    EXPECT_EQ(bootAction(false, false, false), BootAction::None);
    EXPECT_EQ(bootAction(false, false, true),  BootAction::None);
}

TEST(PwrHoldPolicy, RecoveredFlightAsserts) {
    // Recovery restored INFLIGHT — re-latch regardless of how boot started.
    EXPECT_EQ(bootAction(true,  true, true),  BootAction::Assert);
    EXPECT_EQ(bootAction(true,  true, false), BootAction::Assert);
    // Even without a latched flag the flight is on — assert. Today this row
    // is forward-looking: reaching state_inflight needs the #104 snapshot
    // restore, which V9/V10 lack until #846 lands (and V7/V8 have no hold
    // pin), but the policy must already be right for it.
    EXPECT_EQ(bootAction(false, true, true),  BootAction::Assert);
    EXPECT_EQ(bootAction(false, true, false), BootAction::Assert);
}

TEST(PwrHoldPolicy, OrphanedHoldWithOcAliveReleases) {
    // The flight did not resume but the OC answers: it owns the rail again.
    EXPECT_EQ(bootAction(true, false, true), BootAction::Release);
}

TEST(PwrHoldPolicy, OrphanedHoldWithOcDeadKeepsHolding) {
    // Downed-rocket tracker mode: the hold is the only thing keeping the
    // GNSS downlink alive. Battery pull is the power-off.
    EXPECT_EQ(bootAction(true, false, false), BootAction::KeepHold);
}
