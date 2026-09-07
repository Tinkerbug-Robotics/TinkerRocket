// #1188 (#1176 decision 4): the boot LED cue decision table. The restore
// block's "both LEDs solid" covered only the boots that actually restored a
// flight; a token boot the FC declined looked like an operator power-on. This
// pins the full table, including the transitions a single boot walks through.

#include <gtest/gtest.h>

#include "boot_cue_policy.h"

using BootCuePolicy::Cue;
using BootCuePolicy::cueFor;

TEST(BootCuePolicy, NormalBootShowsNothing) {
    EXPECT_EQ(cueFor(false, false, true),  Cue::None);   // on the pad
    EXPECT_EQ(cueFor(false, false, false), Cue::None);   // in the air, or still initialising
}

TEST(BootCuePolicy, RestoredFlightWinsRegardless) {
    EXPECT_EQ(cueFor(true, false, false), Cue::RestoredFlight);
    EXPECT_EQ(cueFor(true, true,  false), Cue::RestoredFlight);
    // on_ground cannot be true while a restored flight is in progress (INFLIGHT
    // is not a ground state), but the table must not depend on that.
    EXPECT_EQ(cueFor(true, false, true),  Cue::RestoredFlight);
    EXPECT_EQ(cueFor(true, true,  true),  Cue::RestoredFlight);
}

TEST(BootCuePolicy, TokenBootTheFcDeclinedBlinksOnTheGround) {
    // The half that was missing: the OC raised the rail from a token, the FC
    // refused the snapshot (a simulated frame, a LANDED clear, no answer) and
    // sits in READY/PRELAUNCH looking exactly like an operator power-on.
    EXPECT_EQ(cueFor(false, true, true), Cue::SelfPoweredIdle);
}

TEST(BootCuePolicy, RefutedRestoreDropsFromSolidToBlink) {
    // A restored flight the interlock refuted to LANDED: solid while the board
    // still believed it was flying, blink once it decided it is on the ground.
    EXPECT_EQ(cueFor(true,  true, false), Cue::RestoredFlight);
    EXPECT_EQ(cueFor(false, true, true),  Cue::SelfPoweredIdle);
}

TEST(BootCuePolicy, ALaunchInThisSessionEndsTheCue) {
    // enterInflight clears the gate and leaves the ground: the launch is its
    // own evidence, and the heartbeat owns the blue LED in the air. A sim
    // takes the same path.
    EXPECT_EQ(cueFor(false, true, false), Cue::None);
}

TEST(BootCuePolicy, TierOneRestoreWithoutSelfPowerNeverBlinks) {
    // An FC panic or OC fault reset mid-flight restores a flight without the
    // OC ever raising the rail from a token: solid in the air, nothing after.
    EXPECT_EQ(cueFor(true,  false, false), Cue::RestoredFlight);
    EXPECT_EQ(cueFor(false, false, true),  Cue::None);
}
