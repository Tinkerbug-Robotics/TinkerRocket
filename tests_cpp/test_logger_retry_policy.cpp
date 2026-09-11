// #1228: the OC's schedule for retrying a flight logger whose begin() failed.
//
// The schedule is the whole policy. It is bounded because a failed begin()
// against a dead NAND stalls loop_oc for seconds (the LittleFS format fallback
// waits out nandWaitReady's two-second timeout per operation), so an unbounded
// retry would gap the downlink every minute for a chip the scorecard already
// reports as BAD. It defers — without spending an attempt — while a flight is
// in progress, after LANDED (the #317 lockout means a recovered logger could
// log nothing this boot) and during an FC OTA relay.

#include <gtest/gtest.h>

#include "logger_retry_policy.h"

using LoggerRetryPolicy::Verdict;
using LoggerRetryPolicy::decide;
using LoggerRetryPolicy::delayBeforeRetryMs;
using LoggerRetryPolicy::kMaxRetries;
using LoggerRetryPolicy::kRetryDelayMs;

namespace {
constexpr uint32_t kSec = 1000U;
}

TEST(LoggerRetryPolicy, ThreeRetriesSpreadOverAboutAHundredSeconds) {
    EXPECT_EQ(kMaxRetries, 3U);
    EXPECT_EQ(delayBeforeRetryMs(0), 10U * kSec);
    EXPECT_EQ(delayBeforeRetryMs(1), 30U * kSec);
    EXPECT_EQ(delayBeforeRetryMs(2), 60U * kSec);
    // The budget is spent after the third retry: no fourth delay exists.
    EXPECT_EQ(delayBeforeRetryMs(3), 0U);
    EXPECT_EQ(delayBeforeRetryMs(200), 0U);
    uint32_t total = 0;
    for (uint32_t d : kRetryDelayMs) total += d;
    EXPECT_EQ(total, 100U * kSec);
}

TEST(LoggerRetryPolicy, LiveLoggerIsIdleWhateverTheStampsSay) {
    EXPECT_EQ(decide(true, 0, 0, 0, false, false), Verdict::Idle);
    EXPECT_EQ(decide(true, kMaxRetries, 5000, 0, true, true), Verdict::Idle);
}

TEST(LoggerRetryPolicy, WaitsUntilTheDueStamp) {
    const uint32_t due = 60U * kSec;
    EXPECT_EQ(decide(false, 0, due - 1, due, false, false), Verdict::Wait);
    EXPECT_EQ(decide(false, 0, due,     due, false, false), Verdict::Retry);
    EXPECT_EQ(decide(false, 0, due + 1, due, false, false), Verdict::Retry);
}

TEST(LoggerRetryPolicy, DueComparisonSurvivesTheMillisWrap) {
    // A failure 5 s before the 49-day wrap schedules a retry 5 s after it.
    const uint32_t due = 5U * kSec;                 // wrapped
    const uint32_t before_wrap = 0xFFFFFFFFu - 2U * kSec;
    EXPECT_EQ(decide(false, 0, before_wrap, due, false, false), Verdict::Wait);
    EXPECT_EQ(decide(false, 0, due + 10, due, false, false), Verdict::Retry);
}

TEST(LoggerRetryPolicy, HoldsDeferWithoutSpendingTheBudget) {
    // Deferred is a verdict on THIS pass only; the OC leaves retries_spent
    // alone, so the same inputs with the hold lifted are a Retry.
    EXPECT_EQ(decide(false, 0, 0, 0, true,  false), Verdict::Deferred);   // INFLIGHT / LANDED
    EXPECT_EQ(decide(false, 0, 0, 0, false, true),  Verdict::Deferred);   // FC OTA relay
    EXPECT_EQ(decide(false, 0, 0, 0, true,  true),  Verdict::Deferred);
    EXPECT_EQ(decide(false, 0, 0, 0, false, false), Verdict::Retry);
    // A hold on a retry that is not yet due is still just Wait.
    EXPECT_EQ(decide(false, 0, 0, 1, true, false), Verdict::Wait);
}

TEST(LoggerRetryPolicy, ExhaustedOnceTheBudgetIsSpent) {
    EXPECT_EQ(decide(false, kMaxRetries, 0, 0, false, false), Verdict::Exhausted);
    // Exhausted beats every other verdict: nothing is scheduled any more.
    EXPECT_EQ(decide(false, kMaxRetries, 0, 0, true, true), Verdict::Exhausted);
    EXPECT_EQ(decide(false, kMaxRetries, 0, 1000, false, false), Verdict::Exhausted);
    EXPECT_EQ(decide(false, kMaxRetries - 1, 0, 0, false, false), Verdict::Retry);
}

// The OC's bookkeeping, end to end: a failure stamps due = now + delay(spent);
// a Retry increments spent before the attempt; another failure re-stamps.
TEST(LoggerRetryPolicy, ScheduleWalkAgainstAChipThatNeverComesBack) {
    uint32_t now = 100U * kSec;          // the boot's cmd-8 power-on
    uint8_t  spent = 0;
    uint32_t due = now + delayBeforeRetryMs(spent);   // initial failure

    uint32_t expected_delay[] = { 10U * kSec, 30U * kSec, 60U * kSec };
    for (uint32_t d : expected_delay)
    {
        EXPECT_EQ(decide(false, spent, now, due, false, false), Verdict::Wait);
        now = due - 1;
        EXPECT_EQ(decide(false, spent, now, due, false, false), Verdict::Wait);
        now = due;
        EXPECT_EQ(due - (now - d), d) << "the gap before this retry";
        ASSERT_EQ(decide(false, spent, now, due, false, false), Verdict::Retry);
        spent++;                                          // the attempt runs and fails
        due = now + delayBeforeRetryMs(spent);
    }
    // Three retries in: 10 + 30 + 60 s after the first failure, then nothing.
    EXPECT_EQ(now, 100U * kSec + 100U * kSec);
    EXPECT_EQ(spent, kMaxRetries);
    EXPECT_EQ(delayBeforeRetryMs(spent), 0U);
    EXPECT_EQ(decide(false, spent, now + 3600U * kSec, due, false, false), Verdict::Exhausted);
}

// A flight that starts while a retry is pending pushes the retry past the
// landing without consuming it — and LANDED keeps deferring it, because the
// #317 lockout would make a recovered logger useless until the next boot.
TEST(LoggerRetryPolicy, AFlightPushesAPendingRetryOutWithoutSpendingIt) {
    const uint32_t due = 10U * kSec;
    uint8_t spent = 0;
    for (uint32_t t = due; t < due + 600U * kSec; t += 15U * kSec)
    {
        EXPECT_EQ(decide(false, spent, t, due, /*flight_hold*/ true, false), Verdict::Deferred);
    }
    EXPECT_EQ(spent, 0U);
    // A sim re-arm that leaves LANDED lifts the hold: the same retry runs.
    EXPECT_EQ(decide(false, spent, due + 700U * kSec, due, false, false), Verdict::Retry);
}
