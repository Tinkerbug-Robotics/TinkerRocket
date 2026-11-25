// #1755: the apps' "Mounting" health dot (flight_computer
// main/mount_health_policy.h). On AUTO mounting, a still pad window in the
// last 30 s is OK and none is DEGRADED (amber in the apps, never red); a
// manual code, or any state but READY and PRELAUNCH, is N/A.

#include <gtest/gtest.h>

#include <cstdint>

#include "mount_health_policy.h"

using MountHealthPolicy::kFreshMs;
using MountHealthPolicy::state;

namespace {
constexpr bool kAuto = false, kManual = true;
constexpr bool kOnPad = true, kOffPad = false;
}  // namespace

TEST(MountHealthPolicy, FreshIsThirtySeconds) {
    EXPECT_EQ(kFreshMs, 30000u);
}

// A window just now, and one exactly kFreshMs ago, confirm the frame.
TEST(MountHealthPolicy, AutoOnThePadWithARecentWindowIsOk) {
    EXPECT_EQ(state(kAuto, kOnPad, 100000, 100000), SH_OK);
    EXPECT_EQ(state(kAuto, kOnPad, 100000, 112345), SH_OK);
    EXPECT_EQ(state(kAuto, kOnPad, 100000, 100000 + kFreshMs), SH_OK);
}

// One millisecond past kFreshMs, and long past, it is DEGRADED.
TEST(MountHealthPolicy, AutoOnThePadWithAnOldWindowIsDegraded) {
    EXPECT_EQ(state(kAuto, kOnPad, 100000, 100000 + kFreshMs + 1), SH_DEGRADED);
    EXPECT_EQ(state(kAuto, kOnPad, 100000, 100000 + 10u * kFreshMs), SH_DEGRADED);
}

// No window this boot (0): DEGRADED from the first frame, however early.
TEST(MountHealthPolicy, AutoOnThePadWithNoWindowIsDegraded) {
    EXPECT_EQ(state(kAuto, kOnPad, 0, 0), SH_DEGRADED);
    EXPECT_EQ(state(kAuto, kOnPad, 0, 1500), SH_DEGRADED);
    EXPECT_EQ(state(kAuto, kOnPad, 0, kFreshMs), SH_DEGRADED);
    EXPECT_EQ(state(kAuto, kOnPad, 0, 0xFFFFFFFFu), SH_DEGRADED);
}

// time_ms() wraps at 2^32 ms (~49.7 days): a window just before the wrap is
// still aged right just after it.
TEST(MountHealthPolicy, TheAgeHoldsAcrossTheMillisecondWrap) {
    const uint32_t before = 0xFFFFFFFFu - 999u;   // 1 s before the wrap
    EXPECT_EQ(state(kAuto, kOnPad, before, 500u), SH_OK);                     // 1.5 s later
    EXPECT_EQ(state(kAuto, kOnPad, before, kFreshMs - 1000u), SH_OK);         // exactly kFreshMs
    EXPECT_EQ(state(kAuto, kOnPad, before, kFreshMs - 1000u + 1u), SH_DEGRADED);
}

// A manual code is the owner's statement of the mounting: N/A whatever the
// windows say, on the pad or off it.
TEST(MountHealthPolicy, AManualCodeIsNotApplicable) {
    EXPECT_EQ(state(kManual, kOnPad, 100000, 100000), SH_NA);
    EXPECT_EQ(state(kManual, kOnPad, 0, 100000), SH_NA);
    EXPECT_EQ(state(kManual, kOnPad, 100000, 100000 + 10u * kFreshMs), SH_NA);
    EXPECT_EQ(state(kManual, kOffPad, 0, 100000), SH_NA);
}

// Off the pad (in flight, landed, initializing) the estimator does not run and
// the frame is latched: N/A, never a stale amber or green.
TEST(MountHealthPolicy, OffThePadIsNotApplicable) {
    EXPECT_EQ(state(kAuto, kOffPad, 100000, 100000), SH_NA);
    EXPECT_EQ(state(kAuto, kOffPad, 0, 100000), SH_NA);
    EXPECT_EQ(state(kAuto, kOffPad, 100000, 100000 + 10u * kFreshMs), SH_NA);
}

// SH_BAD is never produced: the apps hold "Ready to fly" to amber at worst.
TEST(MountHealthPolicy, NeverBad) {
    for (bool manual : {false, true})
        for (bool pad : {false, true})
            for (uint32_t last : {0u, 1u, 100000u, 0xFFFFFFF0u})
                for (uint32_t now : {0u, 1u, 100000u, 100000u + kFreshMs, 100000u + kFreshMs + 1u, 0xFFFFFFFFu})
                    EXPECT_NE(state(manual, pad, last, now), SH_BAD)
                        << manual << pad << " " << last << " " << now;
}
