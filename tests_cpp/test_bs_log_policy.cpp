// test_bs_log_policy.cpp
// Host-side gtest for the BS LoRa-logging policy helpers extracted from
// projects/base_station/main/bs_log_policy.h (#137).
//
// Currently covers parseSequentialFilename() — the sscanf bug that
// produced bogus `lora_9886.csv` filenames was the most concrete
// regression risk from the 5/9/26 test flights.  Other policy decisions
// (silence-timeout threshold, auto-start gating) live as simple inline
// conditionals in main.cpp and would need an integration / bench test to
// exercise meaningfully — that's what tests/bench/test_lora_log_capture.py
// is for.

#include <gtest/gtest.h>

#include "bs_log_policy.h"

// ---------------------------------------------------------------------------
// parseSequentialFilename()
// ---------------------------------------------------------------------------

TEST(BSLogPolicyParseFilename, AcceptsValidSequentialNames)
{
    uint16_t n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_001.csv", n));
    EXPECT_EQ(n, 1u);

    n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_42.csv", n));
    EXPECT_EQ(n, 42u);

    n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_9886.csv", n));
    EXPECT_EQ(n, 9886u);

    n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_65535.csv", n));
    EXPECT_EQ(n, 65535u);

    n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_0.csv", n));
    EXPECT_EQ(n, 0u);
}

TEST(BSLogPolicyParseFilename, RejectsTimestampedNames)
{
    // Regression for the core #137 sscanf bug.  Pre-fix:
    //   sscanf("lora_20260509_164143.csv", "lora_%hu.csv", &n) == 1, n=9885
    // This made findNextFileNumber inflate max+1 to 9886, producing
    // `lora_9886.csv` on the next no-time-sync boot.
    uint16_t n = 0xFFFF;
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_20260509_164143.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_20260509_144144.csv", n));
    // _2 collision-suffix variant should also reject (it's still timestamped)
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_20260509_164143_2.csv", n));
}

TEST(BSLogPolicyParseFilename, RejectsUnrelatedNames)
{
    uint16_t n = 0xFFFF;
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "flight_20260509_144128.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora.csv", n));               // no number
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_001.txt", n));           // wrong extension
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_001", n));               // missing extension
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_001.csv.bak", n));       // trailing junk
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        ".write_test", n));            // BS boot probe file
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(nullptr, n));
}

TEST(BSLogPolicyParseFilename, RejectsNegativeSignAndNonDigits)
{
    // The parser walks digits explicitly (rather than letting sscanf %hu
    // accept signs / whitespace and wrap), so any non-digit between
    // "lora_" and ".csv" disqualifies the match.
    uint16_t n = 0xFFFF;
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_-1.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_+1.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_ 1.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_1a.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_abc.csv", n));
}

TEST(BSLogPolicyParseFilename, RejectsOversized)
{
    // 6-digit run can't fit uint16 (max=65535) — and even values <=65535
    // that happen to have 6 digits ("099999" hypothetically) are rejected
    // because they'd have come from a sequence we never produce.  Real-
    // world BS code writes %03u, so anything >5 digits is malformed.
    uint16_t n = 0xFFFF;
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_99999.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_123456.csv", n));
    // Boundary: 65535 is the highest uint16 (5 digits) — accepted.
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_65535.csv", n));
}
