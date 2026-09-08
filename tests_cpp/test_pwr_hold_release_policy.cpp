// #1137 item 9 — the LANDED P4_EN_HOLD release test.
//
// Releasing the hold on a dead OC drops the FC's rail in ~0.8 s and takes the
// downed rocket's GNSS downlink with it.  The old gate (`out_ready` alone)
// could not tell a live OC from one that died at burnout, because the poll
// that sets that latch is suppressed for the whole INFLIGHT phase.

#include <gtest/gtest.h>

#include "pwr_hold_release_policy.h"

using pwr_hold::ocAliveSinceLaunch;

TEST(PwrHoldReleasePolicy, AnswerFromAfterLaunchReleases) {
    EXPECT_TRUE(ocAliveSinceLaunch(true, /*last=*/120000, /*launch=*/100000));
}

TEST(PwrHoldReleasePolicy, OnlyAPadAnswerKeepsTheHold) {
    // The whole finding: the OC answered at T-50 s and died at burnout.  The
    // latch is still true at landing, and used to be the entire test.
    EXPECT_FALSE(ocAliveSinceLaunch(true, /*last=*/50000, /*launch=*/100000));
}

TEST(PwrHoldReleasePolicy, AnswerExactlyAtLaunchIsNotEvidence) {
    // Same millisecond as the launch stamp is the pad answer, not a post-launch
    // one -- the poll had not yet been suppressed. Strictly-after is the test.
    EXPECT_FALSE(ocAliveSinceLaunch(true, /*last=*/100000, /*launch=*/100000));
}

TEST(PwrHoldReleasePolicy, NeverAnsweredKeepsTheHold) {
    // out_ready false is the #848 case that already worked; it must keep
    // working, since a rocket that landed after a both-MCU brownout is exactly
    // the one that needs to stay findable.
    EXPECT_FALSE(ocAliveSinceLaunch(false, /*last=*/120000, /*launch=*/100000));
    EXPECT_FALSE(ocAliveSinceLaunch(false, /*last=*/0, /*launch=*/0));
}

TEST(PwrHoldReleasePolicy, AFlightThatNeverLaunchedReleasesNormally) {
    // launch_time_millis is 0 when no launch was detected (a bench LANDED, the
    // #1176 refuted-restore path).  There was no window during which the poll
    // was suppressed, so an ordinary answer is current and the rail goes back.
    EXPECT_TRUE(ocAliveSinceLaunch(true, /*last=*/8000, /*launch=*/0));
}

TEST(PwrHoldReleasePolicy, SurvivesMillisWraparound) {
    // Launch just before the ~49.7-day rollover, OC answers just after it.
    const uint32_t launch = 0xFFFFF000u;
    const uint32_t answer = launch + 20000u;   // wraps
    ASSERT_LT(answer, launch) << "this test is meaningless without the wrap";
    EXPECT_TRUE(ocAliveSinceLaunch(true, answer, launch));

    // ...and the pad-answer case still reads as stale across the same wrap.
    const uint32_t pad = launch - 50000u;
    EXPECT_FALSE(ocAliveSinceLaunch(true, pad, launch));
}

TEST(PwrHoldReleasePolicy, TheDefaultDirectionIsToKeepTheHold) {
    // Stated as a property: with no positive evidence of a post-launch answer,
    // every combination keeps the hold.  Keeping it costs a battery pull;
    // releasing it wrongly costs the rocket.
    for (uint32_t last : {0u, 1u, 99999u, 100000u}) {
        EXPECT_FALSE(ocAliveSinceLaunch(true, last, 100000u))
            << "released on last=" << last;
        EXPECT_FALSE(ocAliveSinceLaunch(false, last, 100000u))
            << "released with out_ready false, last=" << last;
    }
}
