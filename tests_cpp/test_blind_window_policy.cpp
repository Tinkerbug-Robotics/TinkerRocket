// #1271/#917 — what a phone-IO blind window cost, and specifically when it is
// severe enough to latch a warning the operator must acknowledge.
//
// The two rules under test both exist because a persistent false warning is
// worse than none:
//   * a SHORT window clips the front of a log that still exists, because the
//     OC's launch edge in loop_oc is delayed by the pause rather than missed;
//   * a SIM sets NSF_LAUNCH exactly as a real flight does, and the bench
//     procedure for this feature is itself a sim start.
#include <gtest/gtest.h>
#include "blind_window_policy.h"

using BlindWindowPolicy::Verdict;
using BlindWindowPolicy::classify;
using BlindWindowPolicy::kLatchFloorMs;
using BlindWindowPolicy::shouldLatch;

namespace {
constexpr bool kLaunched = true, kNotLaunched = false;
constexpr bool kPreLatched = true, kNotPreLatched = false;
constexpr bool kSim = true, kReal = false;
}

TEST(BlindWindowPolicy, LongWindowSwallowingALaunchLatches)
{
    const Verdict v = classify(kLaunched, kNotPreLatched, kReal, 240000);
    EXPECT_EQ(v, Verdict::LaunchLost);
    EXPECT_TRUE(shouldLatch(v)) << "a four-minute download that ate a flight must be reported";
}

// The regression the whole duration floor exists for.
TEST(BlindWindowPolicy, ShortWindowDoesNotLatchBecauseTheLogStillExists)
{
    // A cmd-2 file list or a cmd-3 delete's block erases land here. Neither is
    // INFLIGHT-gated in a way that excludes the pad, so without the floor an
    // operator tidying storage while the rocket lights would latch a permanent
    // acknowledgement-required warning about a flight whose log is complete.
    for (uint32_t ms : {0u, 1u, 50u, 200u, 500u, 999u})
    {
        const Verdict v = classify(kLaunched, kNotPreLatched, kReal, ms);
        EXPECT_EQ(v, Verdict::LaunchClipped) << "blind_ms=" << ms;
        EXPECT_FALSE(shouldLatch(v)) << "blind_ms=" << ms;
    }
}

TEST(BlindWindowPolicy, TheFloorIsInclusiveAtItsBoundary)
{
    EXPECT_EQ(classify(kLaunched, kNotPreLatched, kReal, kLatchFloorMs - 1),
              Verdict::LaunchClipped);
    EXPECT_EQ(classify(kLaunched, kNotPreLatched, kReal, kLatchFloorMs),
              Verdict::LaunchLost);
}

// The other spurious-latch path: a sim launch is indistinguishable from a real
// one at the verdict site except for this flag.
TEST(BlindWindowPolicy, ASimulatedLaunchIsNeverLatchedHoweverLongTheWindow)
{
    for (uint32_t ms : {0u, 1000u, 60000u, 600000u})
    {
        const Verdict v = classify(kLaunched, kNotPreLatched, kSim, ms);
        EXPECT_EQ(v, Verdict::LaunchSim) << "blind_ms=" << ms;
        EXPECT_FALSE(shouldLatch(v))
            << "a sim start during a download is this feature's own bench test — "
               "latching it would leave a permanent warning after every run";
    }
}

// If the vehicle was ALREADY flying when the pause began, the window did not
// cost us a launch we would otherwise have caught.
TEST(BlindWindowPolicy, AlreadyFlyingBeforeTheWindowIsNotANewLoss)
{
    // It still falls through to the long-window remark — a four-minute pause
    // mid-flight is worth a line — but it must never latch, because the launch
    // was already known before the window opened.
    EXPECT_EQ(classify(kLaunched, kPreLatched, kReal, 240000), Verdict::NoLaunchLong);
    EXPECT_FALSE(shouldLatch(classify(kLaunched, kPreLatched, kReal, 240000)));
    EXPECT_EQ(classify(kLaunched, kPreLatched, kReal, 200), Verdict::Quiet);
    EXPECT_FALSE(shouldLatch(classify(kLaunched, kPreLatched, kReal, 200)));
}

TEST(BlindWindowPolicy, NoLaunchAcrossTheWindowPreservesThePriorReportingRule)
{
    // Unchanged #917 behaviour: remark on a long quiet window, stay silent on
    // a short one. Neither latches.
    EXPECT_EQ(classify(kNotLaunched, kNotPreLatched, kReal, 5001), Verdict::NoLaunchLong);
    EXPECT_EQ(classify(kNotLaunched, kNotPreLatched, kReal, 5000), Verdict::Quiet);
    EXPECT_EQ(classify(kNotLaunched, kNotPreLatched, kReal, 0),    Verdict::Quiet);
    EXPECT_FALSE(shouldLatch(classify(kNotLaunched, kNotPreLatched, kReal, 5001)));
}

TEST(BlindWindowPolicy, OnlyLaunchLostEverLatches)
{
    // Exhaustive over the input space that matters, so a future verdict added
    // to the enum cannot silently start latching.
    for (bool launched : {false, true})
      for (bool pre : {false, true})
        for (bool sim : {false, true})
          for (uint32_t ms : {0u, 999u, 1000u, 5001u, 600000u})
          {
              const Verdict v = classify(launched, pre, sim, ms);
              const bool expect_latch = launched && !pre && !sim && ms >= kLatchFloorMs;
              EXPECT_EQ(shouldLatch(v), expect_latch)
                  << "launched=" << launched << " pre=" << pre
                  << " sim=" << sim << " ms=" << ms;
          }
}

TEST(BlindWindowPolicy, TheFloorIsOverridableForTest)
{
    EXPECT_EQ(classify(kLaunched, kNotPreLatched, kReal, 100, 50), Verdict::LaunchLost);
    EXPECT_EQ(classify(kLaunched, kNotPreLatched, kReal, 100, 5000), Verdict::LaunchClipped);
}
