#include <gtest/gtest.h>
#include "iis2mdc_poll_gate.h"

// The IIS2MDC attempt gate (#1111).  Pure schedule logic — the I2C transfers
// it paces are bench-only.  Times are the 32-bit microsecond clock.

static constexpr uint32_t MS = 1000u;
static constexpr uint32_t S  = 1000000u;

// Drive a fresh gate into the stalled state.  `t` is advanced past the
// failed attempts so the caller continues on the same clock.
static Iis2mdcPollGate stalledGate(uint32_t& t)
{
    Iis2mdcPollGate g;
    g.reset(t);
    for (uint32_t i = 0; i < Iis2mdcPollGate::STALL_FAILS; i++)
    {
        EXPECT_TRUE(g.due(t));
        g.markAttempt(t);
        g.onResult(false);
        t += 10 * MS;
    }
    EXPECT_TRUE(g.stalled);
    return g;
}

TEST(Iis2mdcPollGate, HealthyCadenceIsTheOdr)
{
    Iis2mdcPollGate g;
    g.reset(5 * S);
    EXPECT_TRUE(g.due(5 * S));                       // first attempt immediately
    g.markAttempt(5 * S);
    EXPECT_EQ(g.onResult(true), Iis2mdcPollGate::EV_NONE);
    EXPECT_FALSE(g.due(5 * S + 9999));
    EXPECT_TRUE(g.due(5 * S + 10 * MS));
    EXPECT_EQ(g.read_ok, 1u);
    EXPECT_EQ(g.read_fail, 0u);
    EXPECT_FALSE(g.stalled);
}

TEST(Iis2mdcPollGate, FailedAttemptStillAdvancesTheGate)
{
    // #1111: the gate this replaces advanced only on success, so a failed
    // read was retried on the very next DRDY wake (~0.26 ms at 3840 Hz).
    Iis2mdcPollGate g;
    g.reset(1 * S);
    g.markAttempt(1 * S);
    EXPECT_EQ(g.onResult(false), Iis2mdcPollGate::EV_NONE);
    EXPECT_FALSE(g.due(1 * S + 260));                // the next DRDY
    EXPECT_FALSE(g.due(1 * S + 9999));
    EXPECT_TRUE(g.due(1 * S + 10 * MS));             // still the ODR, never faster
    EXPECT_EQ(g.read_fail, 1u);
    EXPECT_FALSE(g.stalled);
}

TEST(Iis2mdcPollGate, StallsAfterFiveConsecutiveFailures)
{
    Iis2mdcPollGate g;
    g.reset(0);
    uint32_t t = 0;
    for (uint32_t i = 1; i < Iis2mdcPollGate::STALL_FAILS; i++)
    {
        ASSERT_TRUE(g.due(t));
        g.markAttempt(t);
        EXPECT_EQ(g.onResult(false), Iis2mdcPollGate::EV_NONE) << "failure " << i;
        EXPECT_FALSE(g.stalled);
        t += 10 * MS;
    }
    ASSERT_TRUE(g.due(t));
    g.markAttempt(t);
    EXPECT_EQ(g.onResult(false), Iis2mdcPollGate::EV_STALLED);
    EXPECT_TRUE(g.stalled);
    EXPECT_EQ(g.stall_events, 1u);
    EXPECT_EQ(g.consec_fails, Iis2mdcPollGate::STALL_FAILS);
    // Now a 1 s probe cadence, not 10 ms.
    EXPECT_EQ(g.periodUs(), Iis2mdcPollGate::STALL_RETRY_MIN_US);
    EXPECT_FALSE(g.due(t + 10 * MS));
    EXPECT_FALSE(g.due(t + 999 * MS));
    EXPECT_TRUE(g.due(t + 1 * S));
}

TEST(Iis2mdcPollGate, SuccessResetsTheFailureStreak)
{
    Iis2mdcPollGate g;
    g.reset(0);
    uint32_t t = 0;
    auto attempt = [&](bool ok) {
        g.markAttempt(t);
        const Iis2mdcPollGate::Event ev = g.onResult(ok);
        t += 10 * MS;
        return ev;
    };
    for (int i = 0; i < 4; i++) EXPECT_EQ(attempt(false), Iis2mdcPollGate::EV_NONE);
    EXPECT_EQ(attempt(true), Iis2mdcPollGate::EV_NONE);
    EXPECT_EQ(g.consec_fails, 0u);
    for (int i = 0; i < 4; i++) EXPECT_EQ(attempt(false), Iis2mdcPollGate::EV_NONE);
    EXPECT_FALSE(g.stalled);
    EXPECT_EQ(g.stall_events, 0u);
    EXPECT_EQ(g.read_fail, 8u);
    EXPECT_EQ(g.read_ok, 1u);
}

TEST(Iis2mdcPollGate, StallProbesBackOffToThirtyTwoSeconds)
{
    uint32_t t = 100 * S;
    Iis2mdcPollGate g = stalledGate(t);
    const uint32_t expect_period_s[] = {1, 2, 4, 8, 16, 32, 32, 32};
    for (uint32_t period_s : expect_period_s)
    {
        EXPECT_EQ(g.periodUs(), period_s * S);
        const uint32_t last = g.last_attempt_us;
        EXPECT_FALSE(g.due(last + period_s * S - 1));
        EXPECT_TRUE(g.due(last + period_s * S));
        g.markAttempt(last + period_s * S);
        EXPECT_EQ(g.onResult(false), Iis2mdcPollGate::EV_NONE);   // still stalled, no new event
    }
    EXPECT_TRUE(g.stalled);
    EXPECT_EQ(g.stall_events, 1u);                   // one stall, however long it lasts
}

TEST(Iis2mdcPollGate, ProbeSuccessRecoversAndResumesOdr)
{
    uint32_t t = 10 * S;
    Iis2mdcPollGate g = stalledGate(t);
    // Two failed probes, then the chip answers.
    for (int i = 0; i < 2; i++)
    {
        t = g.last_attempt_us + g.periodUs();
        g.markAttempt(t);
        EXPECT_EQ(g.onResult(false), Iis2mdcPollGate::EV_NONE);
    }
    t = g.last_attempt_us + g.periodUs();
    ASSERT_TRUE(g.due(t));
    g.markAttempt(t);
    EXPECT_EQ(g.onResult(true), Iis2mdcPollGate::EV_RECOVERED);
    EXPECT_FALSE(g.stalled);
    EXPECT_EQ(g.recoveries, 1u);
    EXPECT_EQ(g.consec_fails, 0u);
    EXPECT_EQ(g.periodUs(), Iis2mdcPollGate::PERIOD_US);
    EXPECT_FALSE(g.due(t + 9999));
    EXPECT_TRUE(g.due(t + 10 * MS));

    // A second stall starts the back-off from 1 s again.
    for (uint32_t i = 0; i < Iis2mdcPollGate::STALL_FAILS; i++)
    {
        t += 10 * MS;
        g.markAttempt(t);
        g.onResult(false);
    }
    EXPECT_TRUE(g.stalled);
    EXPECT_EQ(g.stall_events, 2u);
    EXPECT_EQ(g.periodUs(), Iis2mdcPollGate::STALL_RETRY_MIN_US);
}

TEST(Iis2mdcPollGate, ResetSeedsFromNowSoALateBeginPollsImmediately)
{
    // The mini brings the sensor rail up on command, so begin() can run long
    // after boot.  A gate seeded with 0 (what the code before #1111 did)
    // reads (now - 0) as a negative signed difference once the microsecond
    // clock passes 2^31 (35.8 min): a rail-up at boot+40 min left the mag
    // silent until the clock wrapped at boot+71.6 min.
    const uint32_t boot_plus_40min = 40u * 60u * S;          // 2.4e9, past 2^31
    Iis2mdcPollGate legacy;
    legacy.last_attempt_us = 0;
    EXPECT_FALSE(legacy.due(boot_plus_40min));               // the old behaviour ...
    EXPECT_FALSE(legacy.due(boot_plus_40min + 30u * 60u * S)); // ... half an hour later still

    Iis2mdcPollGate g;
    g.reset(boot_plus_40min);
    EXPECT_TRUE(g.due(boot_plus_40min));
    g.markAttempt(boot_plus_40min);
    g.onResult(true);
    EXPECT_FALSE(g.due(boot_plus_40min + 9999));
    EXPECT_TRUE(g.due(boot_plus_40min + 10 * MS));
}

TEST(Iis2mdcPollGate, SurvivesClockWrapMidStall)
{
    uint32_t t = 0xFFFFFFFFu - 500 * MS;                     // 0.5 s before the 32-bit wrap
    Iis2mdcPollGate g = stalledGate(t);                      // attempts straddle the wrap
    const uint32_t last = g.last_attempt_us;
    EXPECT_FALSE(g.due(last + 999 * MS));                    // wraps to a small number
    EXPECT_TRUE(g.due(last + 1 * S));
}
