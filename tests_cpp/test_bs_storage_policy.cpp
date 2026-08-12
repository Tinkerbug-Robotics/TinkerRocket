#include <gtest/gtest.h>

#include "bs_storage_policy.h"
#include "config.h"

using namespace bs_storage_policy;

// #761: mountExternalFlashFat() was single-shot — one transient error on any of
// its four steps demoted the whole boot from a 512 MB NAND to a ~1.9 MB SPIFFS
// partition, and a flight logged there truncates. These pin the decisions the
// retry loop makes, because two of them are destructive if they are off by one:
// formatting on the wrong attempt wipes every stored flight, and a fencepost on
// `is_last` silently drops the reduced-clock retry.

namespace {
constexpr uint32_t kFast = 20 * 1000 * 1000;
constexpr uint32_t kSlow = 10 * 1000 * 1000;
constexpr uint8_t  kAttempts = 3;

Attempt plan(uint8_t n) { return planAttempt(n, kAttempts, kFast, kSlow); }
}  // namespace

// ---- Formatting is the destructive one ----

// The whole reason allow_format is gated. esp_vfs_fat_nand_mount's
// format_if_mount_failed runs f_mkfs, which erases the volume — so a mount
// failure that attempt 2 would have recovered must never reach it on attempt 1.
TEST(BsStoragePolicy, OnlyTheFinalAttemptMayFormat) {
    EXPECT_FALSE(plan(1).allow_format);
    EXPECT_FALSE(plan(2).allow_format);
    EXPECT_TRUE(plan(3).allow_format);
}

// A genuinely blank NAND still has to get formatted, or a fresh board never
// comes up at all. It just happens one attempt later than it used to.
TEST(BsStoragePolicy, ABlankChipIsStillFormattedEventually) {
    bool ever = false;
    for (uint8_t n = 1; n <= kAttempts; ++n) ever = ever || plan(n).allow_format;
    EXPECT_TRUE(ever);
}

// ---- Clock step-down ----

TEST(BsStoragePolicy, RetriesHoldTheNormalClockUntilTheLastAttempt) {
    EXPECT_EQ(plan(1).clock_hz, kFast);
    EXPECT_EQ(plan(2).clock_hz, kFast);
    EXPECT_EQ(plan(3).clock_hz, kSlow);
}

// ---- Fenceposts ----

TEST(BsStoragePolicy, IsLastMarksExactlyTheFinalAttempt) {
    EXPECT_FALSE(plan(1).is_last);
    EXPECT_FALSE(plan(2).is_last);
    EXPECT_TRUE(plan(3).is_last);
}

// A single-attempt configuration must still format and still be last —
// otherwise setting NAND_MOUNT_ATTEMPTS=1 to disable retries would also
// silently disable formatting, and a blank chip would never mount.
TEST(BsStoragePolicy, SingleAttemptStillFormats) {
    const Attempt only = planAttempt(1, 1, kFast, kSlow);
    EXPECT_TRUE(only.is_last);
    EXPECT_TRUE(only.allow_format);
    EXPECT_EQ(only.clock_hz, kSlow);
}

// Defensive: an attempt counter past the configured limit must not read as a
// non-final attempt (which would withhold the format forever).
TEST(BsStoragePolicy, OverrunCountsAsFinal) {
    EXPECT_TRUE(planAttempt(4, kAttempts, kFast, kSlow).is_last);
    EXPECT_TRUE(planAttempt(4, kAttempts, kFast, kSlow).allow_format);
}

// ---- Demotion classification ----

// The bench case that opened this: BaseStation V4 has a NAND, and came up on
// internal flash anyway.
TEST(BsStoragePolicy, PrimaryStorageFittedButOnSpiffsIsADemotion) {
    EXPECT_TRUE(demoted(/*has_primary_storage=*/true, /*using_internal_flash=*/true));
}

TEST(BsStoragePolicy, PrimaryStorageUpIsNotADemotion) {
    EXPECT_FALSE(demoted(true, false));
}

// A board with no NAND and no SD slot logs to SPIFFS by design. Flagging that
// as a fallback would cry wolf on every boot.
TEST(BsStoragePolicy, NoPrimaryStorageIsAConfigurationNotAFailure) {
    EXPECT_FALSE(demoted(/*has_primary_storage=*/false, /*using_internal_flash=*/true));
}

// ---- Recovered-after-retry ----

TEST(BsStoragePolicy, FirstAttemptSuccessIsNotARecovery) {
    EXPECT_FALSE(recovered(/*demoted_now=*/false, /*attempts_used=*/1));
}

TEST(BsStoragePolicy, SucceedingOnALaterAttemptIsARecovery) {
    EXPECT_TRUE(recovered(false, 2));
    EXPECT_TRUE(recovered(false, 3));
}

// The two states are mutually exclusive on the wire: a demoted boot exhausted
// every attempt, so reporting "recovered after retries" alongside it would be
// contradictory.
TEST(BsStoragePolicy, ADemotedBootIsNeverAlsoARecovery) {
    EXPECT_FALSE(recovered(/*demoted_now=*/true, /*attempts_used=*/3));
}

// ---- The shipped configuration ----

// config.h is what actually flies. Pin the properties the retry depends on so a
// future edit to those constants trips here rather than at the pad.
TEST(BsStoragePolicy, ShippedConfigRetriesAndStepsTheClockDown) {
    EXPECT_GE(config::NAND_MOUNT_ATTEMPTS, 2);
    EXPECT_LT(config::NAND_CLOCK_FALLBACK_HZ, config::NAND_CLOCK_HZ);

    // Worst case is every attempt failing. Boot delay must stay small enough
    // that a demoted boot is still a *fast* boot — the operator should not be
    // waiting on storage that is not coming up.
    const uint32_t worst_ms =
        config::NAND_MOUNT_ATTEMPTS *
        (config::NAND_SETTLE_MS + config::NAND_READY_TIMEOUT_MS + config::NAND_MOUNT_RETRY_DELAY_MS);
    EXPECT_LT(worst_ms, 500u);
}
