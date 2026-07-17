// #510: FlushIterLedger — per-iteration accounting for the OC log-flush task.
// The 7/14 bench flight's 588 ms activation "STALL" was 98% invisible because
// every existing accumulator is a per-op MAXIMUM; the activation iteration
// drains the whole ~97 KB prelaunch ring as ~24 sub-threshold page writes.
// These tests pin the ledger arithmetic and the WARN/INFO classification:
// a long iteration the section timers explain is LongAccounted (INFO, e.g.
// the designed arm-time 80-block erase), while a genuine blind spot stays
// LongUnaccounted (WARN).

#include <gtest/gtest.h>

#include "flush_iter_ledger.h"

using Verdict = FlushIterLedger::Verdict;

static constexpr uint32_t THRESH = 100'000;  // mirrors STALL_THRESHOLD_US

TEST(FlushIterLedger, DefaultsAndResetZeroEverything)
{
    FlushIterLedger l;
    EXPECT_EQ(l.accountedUs(), 0u);
    EXPECT_EQ(l.drain_pages, 0u);

    l.staging_us = 1; l.open_us = 2; l.hook_us = 3; l.activate_us = 4;
    l.drain_us = 5; l.endflight_us = 6;
    l.drain_pages = 7; l.drain_bytes = 8; l.pop_us = 9; l.write_us = 10;
    l.reset();
    EXPECT_EQ(l.accountedUs(), 0u);
    EXPECT_EQ(l.drain_pages, 0u);
    EXPECT_EQ(l.drain_bytes, 0u);
    EXPECT_EQ(l.pop_us, 0u);
    EXPECT_EQ(l.write_us, 0u);
}

TEST(FlushIterLedger, AccountedSumsTheSixSections_NotSubDetails)
{
    FlushIterLedger l;
    l.staging_us = 10; l.open_us = 20; l.hook_us = 30;
    l.activate_us = 40; l.drain_us = 50; l.endflight_us = 60;
    l.pop_us = 1000; l.write_us = 2000;  // sub-details of drain, not added
    EXPECT_EQ(l.accountedUs(), 210u);
}

TEST(FlushIterLedger, UnaccountedClampsAtZero)
{
    FlushIterLedger l;
    l.drain_us = 500;
    EXPECT_EQ(l.unaccountedUs(1000), 500u);
    // Section brackets can sum a hair past the iteration bracket (extra
    // esp_timer reads between them) — must clamp, not wrap.
    EXPECT_EQ(l.unaccountedUs(499), 0u);
}

TEST(FlushIterLedger, DrainOtherIsolatesNonPopWriteTime_AndClamps)
{
    FlushIterLedger l;
    l.drain_us = 581'000; l.pop_us = 210'000; l.write_us = 350'000;
    EXPECT_EQ(l.drainOtherUs(), 21'000u);  // FL prints / loop overhead

    // End-of-flight iterations book pops under endflight_us, so drain_us
    // can be smaller than the sub-details — clamp to 0, don't wrap.
    FlushIterLedger e;
    e.endflight_us = 400'000; e.pop_us = 150'000; e.write_us = 200'000;
    EXPECT_EQ(e.drainOtherUs(), 0u);
}

TEST(FlushIterLedger, ClassifyQuietAtOrUnderThreshold)
{
    EXPECT_EQ(FlushIterLedger::classify(THRESH, 0, THRESH), Verdict::Quiet);
    EXPECT_EQ(FlushIterLedger::classify(THRESH - 1, 0, THRESH), Verdict::Quiet);
    // Matches the pre-#510 trigger: strictly-greater fires.
    EXPECT_NE(FlushIterLedger::classify(THRESH + 1, 0, THRESH), Verdict::Quiet);
}

// The real 7/14 case as it SHOULD read with the ledger in place: 588 ms
// iteration, ~583 ms of it in activate+drain sections → INFO, not WARN.
TEST(FlushIterLedger, Bench588ms_MostlyAccounted_IsInfo)
{
    EXPECT_EQ(FlushIterLedger::classify(588'005, 583'000, THRESH),
              Verdict::LongAccounted);
}

// The same case as it read on 7/14 (only ~11 ms instrumented): a genuine
// blind spot → stays WARN.
TEST(FlushIterLedger, Bench588ms_AsReported11msAccounted_IsWarn)
{
    EXPECT_EQ(FlushIterLedger::classify(588'005, 11'000, THRESH),
              Verdict::LongUnaccounted);
}

// The designed 342 ms arm-time erase, fully booked under hook_us → INFO
// (#510 item 3: the designed cost must stop reading as a fault).
TEST(FlushIterLedger, ArmErase342ms_FullyAccounted_IsInfo)
{
    EXPECT_EQ(FlushIterLedger::classify(342'099, 342'000, THRESH),
              Verdict::LongAccounted);
}

TEST(FlushIterLedger, ClassifyBoundary_UnaccountedExactlyThresholdIsInfo)
{
    // unacc == threshold → still LongAccounted (WARN needs strictly greater,
    // mirroring the iteration trigger).
    EXPECT_EQ(FlushIterLedger::classify(300'000, 200'000, THRESH),
              Verdict::LongAccounted);
    EXPECT_EQ(FlushIterLedger::classify(300'001, 200'000, THRESH),
              Verdict::LongUnaccounted);
    // accounted > iter (clamp path) → unacc 0 → INFO.
    EXPECT_EQ(FlushIterLedger::classify(300'000, 300'500, THRESH),
              Verdict::LongAccounted);
}
