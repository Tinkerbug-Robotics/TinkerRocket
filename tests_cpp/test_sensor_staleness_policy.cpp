// #1137 items 10 and 12 — the scorecard's staleness debounce.
//
// The verdict this drives is the operator's pre-launch go/no-go, so it has two
// jobs that pull against each other: it must actually fire (the whole finding
// is that a wedged sensor read amber forever), and it must not chatter (a red
// light that blinks is a red light that gets ignored).  Both are pinned here.

#include <gtest/gtest.h>

#include "sensor_staleness_policy.h"

using sensor_staleness::State;
using sensor_staleness::step;
using sensor_staleness::kAssertMs;
using sensor_staleness::kHoldMs;

TEST(SensorStalenessPolicy, FreshSensorIsNeverStale)
{
    State s;
    for (uint32_t t = 0; t < 600000; t += 10)
    {
        ASSERT_FALSE(step(s, true, t)) << "at t=" << t;
    }
}

TEST(SensorStalenessPolicy, AssertsOnlyAfterTheFullPersistence)
{
    State s;
    uint32_t first = 0;
    for (uint32_t t = 1000; t <= 1000 + kAssertMs + 100; t += 10)
    {
        if (step(s, false, t)) { first = t; break; }
    }
    ASSERT_NE(first, 0u);
    EXPECT_EQ(first, 1000u + kAssertMs);
}

TEST(SensorStalenessPolicy, OneMissedPollDoesNotAssert)
{
    // A single dropped sample at a 10 ms tick is nowhere near kAssertMs.
    State s;
    EXPECT_FALSE(step(s, true,  1000));
    EXPECT_FALSE(step(s, false, 1010));
    EXPECT_FALSE(step(s, true,  1020));
    for (uint32_t t = 1030; t < 20000; t += 10)
    {
        ASSERT_FALSE(step(s, true, t));
    }
}

TEST(SensorStalenessPolicy, IntermittentDropoutsNeverAccumulate)
{
    // Stale for 1 s, fresh for 100 ms, repeatedly.  Each stale run is short of
    // kAssertMs and the counter restarts, so this must stay green -- the
    // policy asserts on CONTINUOUS staleness, not on a duty cycle.
    State s;
    for (uint32_t t = 0; t < 60000; t += 10)
    {
        const bool fresh = (t % 1100) >= 1000;
        ASSERT_FALSE(step(s, fresh, t)) << "at t=" << t;
    }
}

TEST(SensorStalenessPolicy, HoldsAssertedAcrossTheFullRecovery)
{
    State s;
    // Assert.
    for (uint32_t t = 0; t <= kAssertMs; t += 10) step(s, false, t);
    ASSERT_TRUE(s.asserted);

    // Now fresh: must stay asserted for the whole hold, then clear.
    const uint32_t rec = kAssertMs + 10;
    uint32_t cleared = 0;
    for (uint32_t t = rec; t <= rec + kHoldMs + 1000; t += 10)
    {
        if (!step(s, true, t)) { cleared = t; break; }
    }
    ASSERT_NE(cleared, 0u);
    EXPECT_EQ(cleared, rec + kHoldMs);
}

TEST(SensorStalenessPolicy, AFlappingSensorReadsBadContinuously)
{
    // The reason the hold is longer than the assert.  A sensor answering only
    // every few seconds must not strobe the operator's light: once asserted,
    // each new dropout arrives well inside the hold and keeps it red.
    State s;
    for (uint32_t t = 0; t <= kAssertMs; t += 10) step(s, false, t);
    ASSERT_TRUE(s.asserted);

    for (uint32_t t = kAssertMs + 10; t < 120000; t += 10)
    {
        // 500 ms of answers, then 3 s of silence -- forever.
        const bool fresh = (t % 3500) < 500;
        ASSERT_TRUE(step(s, fresh, t)) << "flickered green at t=" << t;
    }
}

TEST(SensorStalenessPolicy, PartialRecoveryDoesNotClear)
{
    State s;
    for (uint32_t t = 0; t <= kAssertMs; t += 10) step(s, false, t);
    ASSERT_TRUE(s.asserted);

    // Fresh for just under the hold, then stale again.
    const uint32_t rec = kAssertMs + 10;
    for (uint32_t t = rec; t < rec + kHoldMs - 100; t += 10)
    {
        ASSERT_TRUE(step(s, true, t));
    }
    EXPECT_TRUE(step(s, false, rec + kHoldMs - 100));
    // ...and the hold restarts from the NEXT recovery, not the earlier one.
    const uint32_t rec2 = rec + kHoldMs;
    uint32_t cleared = 0;
    for (uint32_t t = rec2; t <= rec2 + kHoldMs + 1000; t += 10)
    {
        if (!step(s, true, t)) { cleared = t; break; }
    }
    EXPECT_EQ(cleared, rec2 + kHoldMs);
}

TEST(SensorStalenessPolicy, ReassertsAfterAFullRecovery)
{
    State s;
    for (uint32_t t = 0; t <= kAssertMs; t += 10) step(s, false, t);
    ASSERT_TRUE(s.asserted);
    for (uint32_t t = kAssertMs + 10; t <= kAssertMs + 10 + kHoldMs; t += 10)
        step(s, true, t);
    ASSERT_FALSE(s.asserted);

    const uint32_t t0 = kAssertMs + 10 + kHoldMs + 10;
    uint32_t again = 0;
    for (uint32_t t = t0; t <= t0 + kAssertMs + 100; t += 10)
    {
        if (step(s, false, t)) { again = t; break; }
    }
    ASSERT_NE(again, 0u);
    EXPECT_EQ(again, t0 + kAssertMs);
}

TEST(SensorStalenessPolicy, SurvivesMillisWraparound)
{
    State s;
    const uint32_t t0 = 0xFFFFF000u;
    uint32_t first = 0;
    for (uint32_t i = 0; i <= 1000; i++)
    {
        const uint32_t t = t0 + i * 10u;   // wraps partway through
        if (step(s, false, t)) { first = t; break; }
    }
    EXPECT_EQ(first, (uint32_t)(t0 + kAssertMs));
}

TEST(SensorStalenessPolicy, TheHoldIsLongerThanTheAssert)
{
    // Not arbitrary: the asymmetry is what makes a repeatedly-dropping sensor
    // read bad continuously rather than blinking.  If these are ever equalised
    // the AFlappingSensor test above is the one that will notice, but state
    // the intent here too.
    static_assert(kHoldMs > kAssertMs,
                  "the recovery hold must outlast the assert persistence");
    SUCCEED();
}
