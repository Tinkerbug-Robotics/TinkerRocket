// #1137 items 7-8 — the INFLIGHT -> LANDED debounce.
//
// Two things are pinned here.  The dwell has to be a real dwell (the code this
// replaced measured its reset arm from the candidate START, which made every
// false sample inside the window invisible), and the gyro veto has to be
// survivable: it may not hold a landed rocket in INFLIGHT on a frozen reading
// until the 10-minute MAX_FLIGHT_TIME_MS backstop bails it out, because LANDED
// is what arms post_flight_lockout, safes the pyro rail and closes the log.

#include <gtest/gtest.h>

#include "landing_transition_policy.h"

using landing_transition::Action;
using landing_transition::State;
using landing_transition::step;
using landing_transition::kDwellMs;
using landing_transition::kFalseResetMs;
using landing_transition::kGyroVetoBoundMs;

namespace {

// Run the policy from t0 to t0+span at 10 ms and report the first Land.
// Returns 0 when it never lands.
uint32_t runUntilLand(State &s, bool vote, bool imu_fresh, float roll,
                      uint32_t t0, uint32_t span)
{
    for (uint32_t t = t0; t <= t0 + span; t += 10)
    {
        if (step(s, vote, imu_fresh, roll, t) == Action::Land) return t;
    }
    return 0;
}

}  // namespace

TEST(LandingTransitionPolicy, NoVoteNeverLands)
{
    State s;
    EXPECT_EQ(runUntilLand(s, false, true, 0.0f, 1000, 600000), 0u);
}

TEST(LandingTransitionPolicy, QuietGyroLandsAfterTheDwell)
{
    State s;
    const uint32_t at = runUntilLand(s, true, true, 1.0f, 1000, 10000);
    ASSERT_NE(at, 0u);
    EXPECT_EQ(at, 1000u + kDwellMs);
}

TEST(LandingTransitionPolicy, DwellIsNotSatisfiedByTwoLuckySamples)
{
    // The defect this closes: the old rule armed at the first qualifying
    // sample and then only asked whether the condition happened to hold again
    // once 2 s had passed.  Everything in between was discarded, so a rocket
    // spinning hard for the whole window still landed on two lucky frames.
    State s;
    uint32_t landed_at = 0;
    for (uint32_t t = 1000; t <= 6000; t += 10)
    {
        // Quiet on exactly two samples: the one that armed the candidate and
        // the first one the old `now - start > 2000` test would have accepted.
        const bool quiet = (t == 1000) || (t == 1000 + kDwellMs + 10);
        const float roll = quiet ? 1.0f : 200.0f;
        if (step(s, true, true, roll, t) == Action::Land) { landed_at = t; break; }
    }
    EXPECT_EQ(landed_at, 0u);
}

TEST(LandingTransitionPolicy, BriefSpinBlipDoesNotRestartTheDwell)
{
    // The noise immunity the old comment promised, now actually implemented:
    // a false run shorter than kFalseResetMs leaves the dwell running.
    State s;
    uint32_t landed_at = 0;
    for (uint32_t t = 1000; t <= 8000; t += 10)
    {
        // One 200 ms burst of spin in the middle of the dwell.
        const bool blip = (t >= 1500 && t < 1700);
        const float roll = blip ? 200.0f : 1.0f;
        if (step(s, true, true, roll, t) == Action::Land) { landed_at = t; break; }
    }
    ASSERT_NE(landed_at, 0u);
    EXPECT_EQ(landed_at, 1000u + kDwellMs);
}

TEST(LandingTransitionPolicy, SustainedSpinRestartsTheDwell)
{
    State s;
    uint32_t landed_at = 0;
    // Spin from 1500 for a full second -- longer than kFalseResetMs -- then
    // go quiet.  The dwell must start over from the end of the spin.
    for (uint32_t t = 1000; t <= 12000; t += 10)
    {
        const bool spin = (t >= 1500 && t < 2500);
        const float roll = spin ? 200.0f : 1.0f;
        if (step(s, true, true, roll, t) == Action::Land) { landed_at = t; break; }
    }
    ASSERT_NE(landed_at, 0u);
    EXPECT_EQ(landed_at, 2500u + kDwellMs);
}

TEST(LandingTransitionPolicy, StaleImuCannotVeto)
{
    // roll_rate_dps is frozen at 200 dps by a wedged IMU.  With no live
    // evidence the veto must not fire at all -- the kinematics vote already
    // said landed, and it no longer counts a stale gyro toward that vote.
    State s;
    const uint32_t at = runUntilLand(s, true, /*imu_fresh=*/false, 200.0f,
                                     1000, 10000);
    ASSERT_NE(at, 0u);
    EXPECT_EQ(at, 1000u + kDwellMs);
}

TEST(LandingTransitionPolicy, FreshSpinVetoesButOnlyUpToTheBound)
{
    // A genuinely spinning airframe holds LANDED off -- but not past
    // kGyroVetoBoundMs, or a stuck-but-fresh gyro would leave the squibs live
    // on the ground for the full 10 minutes it takes the flight-time backstop
    // to safe them.
    State s;
    ASSERT_EQ(runUntilLand(s, true, true, 200.0f, 1000, kGyroVetoBoundMs - 5000),
              0u);

    const uint32_t at = runUntilLand(s, true, true, 200.0f,
                                     1000 + kGyroVetoBoundMs - 5000, 20000);
    ASSERT_NE(at, 0u);
    // The bound is measured from when the vote first latched (t = 1000), and
    // the dwell only starts once the veto lifts.
    EXPECT_EQ(at, 1000u + kGyroVetoBoundMs + kDwellMs);
}

TEST(LandingTransitionPolicy, VetoBoundRunsFromTheVoteNotFromBoot)
{
    // Ten minutes of flight before the vote must not consume the allowance.
    State s;
    for (uint32_t t = 0; t < 600000; t += 1000)
    {
        ASSERT_EQ(step(s, false, true, 200.0f, t), Action::Hold);
    }
    EXPECT_EQ(runUntilLand(s, true, true, 200.0f, 600000, kGyroVetoBoundMs - 5000),
              0u);
}

TEST(LandingTransitionPolicy, ThresholdIsExclusiveAtTheBoundary)
{
    State s;
    // Exactly kGyroQuietDps vetoes; just under it does not.
    EXPECT_EQ(runUntilLand(s, true, true, landing_transition::kGyroQuietDps,
                           1000, 10000), 0u);
    State s2;
    EXPECT_NE(runUntilLand(s2, true, true,
                           landing_transition::kGyroQuietDps - 0.1f,
                           1000, 10000), 0u);
}

TEST(LandingTransitionPolicy, NegativeRollRateIsMagnitudeTested)
{
    State s;
    EXPECT_EQ(runUntilLand(s, true, true, -200.0f, 1000, 10000), 0u);
}

TEST(LandingTransitionPolicy, SurvivesMillisWraparound)
{
    // now_ms is a 32-bit millisecond counter; every comparison is an unsigned
    // difference so the ~49.7-day rollover is a non-event.
    State s;
    const uint32_t t0 = 0xFFFFF000u;
    uint32_t landed_at = 0;
    for (uint32_t i = 0; i <= 1000; i++)
    {
        const uint32_t t = t0 + i * 10u;   // wraps partway through
        if (step(s, true, true, 1.0f, t) == Action::Land) { landed_at = t; break; }
    }
    EXPECT_EQ(landed_at, (uint32_t)(t0 + kDwellMs));
}

TEST(LandingTransitionPolicy, ResetClearsAHalfBuiltDwell)
{
    State s;
    step(s, true, true, 1.0f, 1000);
    ASSERT_TRUE(s.candidate_active);
    s = State{};
    EXPECT_FALSE(s.candidate_active);
    EXPECT_FALSE(s.vote_seen);
    // ...and the dwell then runs in full from the new start.
    const uint32_t at = runUntilLand(s, true, true, 1.0f, 50000, 10000);
    EXPECT_EQ(at, 50000u + kDwellMs);
}
