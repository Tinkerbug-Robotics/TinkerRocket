// #1151: when the OC re-attempts a slave device it failed to create.
//
// The bug this guards is not in the timing — it was that there was no retry at
// all. resetSlaveTx() nulls the device handle before re-creating it, and its
// entry guard used to be "does a device exist", so one failed create latched
// the interface off: every later call, including the next I2C_TX_RESYNC, was
// refused. The FC<->OC command path then stayed dead for the whole power cycle
// while telemetry, LoRa and BLE kept working, so nothing else showed it.
//
// The guard is now keyed off "a device is WANTED". These pin the cadence that
// spends the retry, and the one property that matters: the latch is the input,
// never the device's presence.

#include <gtest/gtest.h>

#include "i2c_slave_recovery_policy.h"

namespace P = I2cSlaveRecoveryPolicy;

TEST(I2cSlaveRecovery, AHealthySlaveIsNeverRetried) {
    // The common case: nothing broken, so no work and no log, at any time.
    EXPECT_FALSE(P::shouldRetry(false, 0, 0));
    EXPECT_FALSE(P::shouldRetry(false, 1000000, 0));
    EXPECT_FALSE(P::shouldComplain(false, 1000000, 0));
}

TEST(I2cSlaveRecovery, ABrokenSlaveIsRetriedOnceTheIntervalHasPassed) {
    const uint32_t t0 = 500000;
    EXPECT_FALSE(P::shouldRetry(true, t0, t0));
    EXPECT_FALSE(P::shouldRetry(true, t0 + P::kRetryIntervalMs - 1, t0));
    EXPECT_TRUE(P::shouldRetry(true, t0 + P::kRetryIntervalMs, t0));
    EXPECT_TRUE(P::shouldRetry(true, t0 + 10 * P::kRetryIntervalMs, t0));
}

TEST(I2cSlaveRecovery, TheRetryKeepsComingForeverRatherThanGivingUp) {
    // There is no attempt budget on purpose. A rocket that accepts no commands
    // is not in a state where "stop trying" is ever the better answer, and the
    // cost of trying is one allocation per second. This is the property the
    // original code got wrong by omission, so state it.
    const uint32_t t0 = 1000;
    for (uint32_t hours = 1; hours <= 6; ++hours) {
        const uint32_t now = t0 + hours * 3600u * 1000u;
        EXPECT_TRUE(P::shouldRetry(true, now, now - P::kRetryIntervalMs))
            << "the retry stopped after " << hours << " h";
    }
}

TEST(I2cSlaveRecovery, TheLogIsRarerThanTheRetry) {
    // Once a second is the right cadence to recover on and the wrong one to
    // log at — the still-broken line has to stay readable on the bench.
    EXPECT_GT(P::kComplainIntervalMs, P::kRetryIntervalMs);
    const uint32_t t0 = 77000;
    EXPECT_FALSE(P::shouldComplain(true, t0 + P::kRetryIntervalMs, t0));
    EXPECT_FALSE(P::shouldComplain(true, t0 + P::kComplainIntervalMs - 1, t0));
    EXPECT_TRUE(P::shouldComplain(true, t0 + P::kComplainIntervalMs, t0));
}

TEST(I2cSlaveRecovery, TheMillisecondCounterMayWrapWithoutStallingTheRetry) {
    // time_ms() is a uint32 that wraps every ~49.7 days. The subtraction is
    // done in unsigned arithmetic so the difference stays right across the
    // wrap; a naive signed compare would park a broken slave until reboot,
    // which is the exact failure mode this issue is about.
    const uint32_t before_wrap = 0xFFFFFF00u;
    const uint32_t after_wrap  = before_wrap + P::kRetryIntervalMs;   // wraps
    EXPECT_LT(after_wrap, before_wrap) << "the fixture does not actually wrap";
    EXPECT_TRUE(P::shouldRetry(true, after_wrap, before_wrap));
    EXPECT_TRUE(P::shouldComplain(true, before_wrap + P::kComplainIntervalMs,
                                  before_wrap));
}
