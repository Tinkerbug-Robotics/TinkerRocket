// #468: DedupRebootPolicy — a reboot verdict requires the timestamp
// backstep to persist; replayed stale TX descriptors interleaved with
// fresh traffic must be dropped without resetting the dedup baselines.

#include <gtest/gtest.h>

#include "dedup_reboot_policy.h"

using Action = DedupRebootPolicy::Action;

TEST(DedupRebootPolicy, FreshTrafficAlwaysProceeds)
{
    DedupRebootPolicy p;
    for (uint32_t t = 0; t < 1000; t += 1)
    {
        EXPECT_EQ(p.onFrame(false, t), Action::PROCEED);
    }
    EXPECT_FALSE(p.suspectActive());
}

TEST(DedupRebootPolicy, LoneReplayDroppedWithoutReset)
{
    DedupRebootPolicy p;
    EXPECT_EQ(p.onFrame(false, 10), Action::PROCEED);
    EXPECT_EQ(p.onFrame(true, 11), Action::DROP_REPLAY);
    EXPECT_TRUE(p.suspectActive());
    // Fresh frame right after: the backstep was a replay, suspicion clears.
    EXPECT_EQ(p.onFrame(false, 12), Action::PROCEED);
    EXPECT_FALSE(p.suspectActive());
}

// The bench signature that motivated #468: one stale descriptor re-arrives
// every TX DMA ring revolution (~46 ms), ~15 frames per burst, with fresh
// traffic flowing in between. The old heuristic reset baselines 21.5x/s;
// the policy must never confirm a reboot.
TEST(DedupRebootPolicy, PerRevolutionReplayBurstsNeverConfirm)
{
    DedupRebootPolicy p;
    uint32_t resets = 0;
    for (uint32_t rev = 0; rev < 220; rev++)  // ~10 s of revolutions
    {
        const uint32_t t0 = rev * 46;
        for (int i = 0; i < 15; i++)  // stale burst (all within ~3 ms)
        {
            if (p.onFrame(true, t0 + (i * 3) / 15) == Action::RESET_BASELINE)
            {
                resets++;
            }
        }
        for (int i = 0; i < 40; i++)  // fresh traffic for the rest of the rev
        {
            EXPECT_EQ(p.onFrame(false, t0 + 3 + i), Action::PROCEED);
        }
    }
    EXPECT_EQ(resets, 0u);
}

TEST(DedupRebootPolicy, SustainedBackstepConfirmsOnceAfterWindow)
{
    DedupRebootPolicy p(100);
    uint32_t confirms = 0;
    uint32_t confirm_time = 0;
    // Genuine reboot: every frame from t=500 on is backstepped (2 ms cadence).
    for (uint32_t t = 500; t < 700; t += 2)
    {
        const Action a = p.onFrame(true, t);
        if (a == Action::RESET_BASELINE)
        {
            confirms++;
            if (confirms == 1) confirm_time = t;
            break;  // caller resets max_time_us, so later frames aren't backstepped
        }
        EXPECT_EQ(a, Action::DROP_REPLAY);
    }
    EXPECT_EQ(confirms, 1u);
    EXPECT_EQ(confirm_time, 600u);  // first frame at >= suspect_since + 100
}

TEST(DedupRebootPolicy, FreshFrameRestartsConfirmWindow)
{
    DedupRebootPolicy p(100);
    // 60 ms of backstep...
    for (uint32_t t = 0; t <= 60; t += 2)
    {
        EXPECT_EQ(p.onFrame(true, t), Action::DROP_REPLAY);
    }
    // ...interrupted by one fresh frame: window must restart.
    EXPECT_EQ(p.onFrame(false, 61), Action::PROCEED);
    for (uint32_t t = 62; t < 161; t += 2)
    {
        EXPECT_EQ(p.onFrame(true, t), Action::DROP_REPLAY);
    }
    // 62 + 100 = 162: first frame at/after that confirms.
    EXPECT_EQ(p.onFrame(true, 162), Action::RESET_BASELINE);
}

TEST(DedupRebootPolicy, ExactBoundaryConfirms)
{
    DedupRebootPolicy p(100);
    EXPECT_EQ(p.onFrame(true, 1000), Action::DROP_REPLAY);
    EXPECT_EQ(p.onFrame(true, 1099), Action::DROP_REPLAY);
    EXPECT_EQ(p.onFrame(true, 1100), Action::RESET_BASELINE);
}

TEST(DedupRebootPolicy, MillisWrapDuringWindow)
{
    DedupRebootPolicy p(100);
    const uint32_t near_wrap = 0xFFFFFFF0u;
    EXPECT_EQ(p.onFrame(true, near_wrap), Action::DROP_REPLAY);
    // 16 ms later millis() has wrapped to 0; 84 ms into the window.
    EXPECT_EQ(p.onFrame(true, 0x00000044u), Action::DROP_REPLAY);  // +84 ms
    EXPECT_EQ(p.onFrame(true, 0x00000054u), Action::RESET_BASELINE);  // +100 ms
}

TEST(DedupRebootPolicy, ReusableAfterConfirm)
{
    DedupRebootPolicy p(100);
    EXPECT_EQ(p.onFrame(true, 0), Action::DROP_REPLAY);
    EXPECT_EQ(p.onFrame(true, 100), Action::RESET_BASELINE);
    // New session runs fresh...
    EXPECT_EQ(p.onFrame(false, 200), Action::PROCEED);
    // ...and a later genuine reboot confirms again.
    EXPECT_EQ(p.onFrame(true, 300), Action::DROP_REPLAY);
    EXPECT_EQ(p.onFrame(true, 400), Action::RESET_BASELINE);
}
