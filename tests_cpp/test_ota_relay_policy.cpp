// Host tests for OtaRelayPolicy (#834 items 6 and 7).
//
// The imperative half of the fix (which IDF calls, under which mutex) is not
// testable off-target. What is testable — and what these lock down — is the
// timing that decides WHEN the OC gives up on a stalled relay and HOW OFTEN it
// retries a slave-RX channel it failed to open. Both guard the same outcome:
// the OC must never be left permanently without FC telemetry.

#include <gtest/gtest.h>
#include "ota_relay_policy.h"

using namespace OtaRelayPolicy;

// ---------------------------------------------------------------------------
// shouldRetryRx
// ---------------------------------------------------------------------------
TEST(OtaRelayPolicy, HealthyRxIsNeverRetried)
{
    EXPECT_FALSE(shouldRetryRx(/*rx_broken=*/false, 1000000, 0));
}

TEST(OtaRelayPolicy, BrokenRxRetriesOnTheInterval)
{
    const uint32_t last = 5000;
    EXPECT_FALSE(shouldRetryRx(true, last, last));
    EXPECT_FALSE(shouldRetryRx(true, last + kRxRetryIntervalMs - 1, last));
    EXPECT_TRUE (shouldRetryRx(true, last + kRxRetryIntervalMs, last));
    EXPECT_TRUE (shouldRetryRx(true, last + 60000, last));
}

TEST(OtaRelayPolicy, RetrySurvivesMillisWraparound)
{
    // esp_timer ms wraps every ~49.7 days; unsigned subtraction must carry it.
    const uint32_t last = 0xFFFFFF00u;
    const uint32_t now  = last + kRxRetryIntervalMs;   // wraps past zero
    EXPECT_TRUE(shouldRetryRx(true, now, last));
    EXPECT_FALSE(shouldRetryRx(true, (uint32_t)(last + 10), last));
}

// ---------------------------------------------------------------------------
// relayStalled
// ---------------------------------------------------------------------------
TEST(OtaRelayPolicy, NotStalledWhenNotFlipped)
{
    // Normal telemetry operation: no relay, so nothing to abandon.
    EXPECT_FALSE(relayStalled(/*tx_mode=*/false, 1000, 1000 + 10 * kRelayStallTimeoutMs));
}

TEST(OtaRelayPolicy, NotArmedBeforeTheAppIsToldReady)
{
    // last_chunk_ms == 0 means "ready" has not been released yet. The warmup
    // window deliberately idle-fills so the FC's slave RX can lock onto BCLK;
    // timing that out would abort every single OTA.
    EXPECT_FALSE(relayStalled(true, 0, 10 * kRelayStallTimeoutMs));
}

TEST(OtaRelayPolicy, StallsOnlyAfterTheFullTimeout)
{
    const uint32_t last = 250000;
    EXPECT_FALSE(relayStalled(true, last, last));
    EXPECT_FALSE(relayStalled(true, last, last + kRelayStallTimeoutMs - 1));
    EXPECT_TRUE (relayStalled(true, last, last + kRelayStallTimeoutMs));
}

TEST(OtaRelayPolicy, ASteadilyPumpingAppNeverStalls)
{
    // Chunks arriving well inside the window keep pushing the deadline out.
    // NOTE the loop bound is a CONSTANT computed up front: an earlier version
    // wrote `t < last + 120000` while the body assigned `last = t`, so the
    // bound chased the cursor and the loop ran 21,473,738 times (terminating
    // only on uint32 wraparound) instead of the 600 it claimed to.
    const uint32_t start = 100000;
    const uint32_t end   = start + 120000;         // 120 s of steady pumping
    uint32_t last = start;
    uint32_t iterations = 0;
    for (uint32_t t = start; t < end; t += 200)
    {
        ASSERT_FALSE(relayStalled(true, last, t)) << "t=" << t;
        last = t;                                  // chunk arrived
        ++iterations;
    }
    EXPECT_EQ(iterations, 600u);                   // pins the loop actually ran
}

TEST(OtaRelayPolicy, AnAppThatStopsMidTransferDoesStall)
{
    // The mirror of the test above: pump for a while, then go silent.
    const uint32_t start = 100000;
    uint32_t last = start;
    for (uint32_t t = start; t < start + 20000; t += 200) last = t;
    ASSERT_FALSE(relayStalled(true, last, last));
    EXPECT_TRUE(relayStalled(true, last, last + kRelayStallTimeoutMs));
}

TEST(OtaRelayPolicy, StallSurvivesMillisWraparound)
{
    const uint32_t last = 0xFFFFF000u;
    EXPECT_TRUE (relayStalled(true, last, (uint32_t)(last + kRelayStallTimeoutMs)));
    EXPECT_FALSE(relayStalled(true, last, (uint32_t)(last + 100)));
}

TEST(OtaRelayPolicy, StallTimeoutClearsABleSupervisionTimeout)
{
    // The window must be comfortably longer than a dropped link's own detection
    // (~2-6 s) so a normal disconnect is handled by onDisconnect's abort path,
    // and this watchdog only catches the case with no disconnect event at all.
    EXPECT_GT(kRelayStallTimeoutMs, 6000u);
}
