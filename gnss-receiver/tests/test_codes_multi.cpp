// Galileo E1 and BeiDou B1C spreading codes against their ICDs (milestone 6).

#include "b1c_icd_chips.h"

extern "C" {
#include "gnss/sig.h"
}

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace {

// The first or last 24 chips as the ICDs print them: first chip in the most significant bit.
uint32_t chips24(const uint8_t *c)
{
    uint32_t v = 0;
    for (int i = 0; i < 24; i++) {
        v = (v << 1) | c[i];
    }
    return v;
}

int ones(const std::vector<uint8_t> &c)
{
    int n = 0;
    for (uint8_t b : c) {
        n += b;
    }
    return n;
}

}  // namespace

TEST(CodesB1C, EveryCodeMatchesTheIcdsFirstAndLast24Chips)
{
    std::vector<uint8_t> c(BDS_B1C_LEN), s(BDS_B1C_SEC_LEN);
    for (int prn = 1; prn <= BDS_MAX_PRN; prn++) {
        ASSERT_EQ(bds_b1c_code(prn, GNSS_SIG_BDS_B1CD, c.data()), 0);
        EXPECT_EQ(chips24(c.data()), b1c_icd::data[prn - 1][0]) << "data PRN " << prn;
        EXPECT_EQ(chips24(c.data() + BDS_B1C_LEN - 24), b1c_icd::data[prn - 1][1]) << "data PRN " << prn;
        ASSERT_EQ(bds_b1c_code(prn, GNSS_SIG_BDS_B1CP, c.data()), 0);
        EXPECT_EQ(chips24(c.data()), b1c_icd::pilot[prn - 1][0]) << "pilot PRN " << prn;
        EXPECT_EQ(chips24(c.data() + BDS_B1C_LEN - 24), b1c_icd::pilot[prn - 1][1]) << "pilot PRN " << prn;
        ASSERT_EQ(bds_b1c_secondary(prn, s.data()), 0);
        EXPECT_EQ(chips24(s.data()), b1c_icd::secondary[prn - 1][0]) << "secondary PRN " << prn;
        EXPECT_EQ(chips24(s.data() + BDS_B1C_SEC_LEN - 24), b1c_icd::secondary[prn - 1][1]) << "secondary PRN " << prn;
    }
    EXPECT_EQ(bds_b1c_code(0, GNSS_SIG_BDS_B1CD, c.data()), -1);
    EXPECT_EQ(bds_b1c_code(64, GNSS_SIG_BDS_B1CP, c.data()), -1);
    EXPECT_EQ(bds_b1c_code(1, GNSS_SIG_GAL_E1B, c.data()), -1);
}

TEST(CodesE1, MemoryCodesAndTheSecondaryCode)
{
    std::vector<uint8_t> c(GAL_E1_LEN);
    // The first 32 chips of PRN 1 as the ICD's Annex C prints them (hex F5D71013, B39340CA).
    ASSERT_EQ(gal_e1_code(1, GNSS_SIG_GAL_E1B, c.data()), 0);
    EXPECT_EQ((chips24(c.data()) << 8) | (c[24] << 7 | c[25] << 6 | c[26] << 5 | c[27] << 4 | c[28] << 3 |
                                            c[29] << 2 | c[30] << 1 | c[31]),
              0xF5D71013u);
    ASSERT_EQ(gal_e1_code(1, GNSS_SIG_GAL_E1C, c.data()), 0);
    EXPECT_EQ(chips24(c.data()), 0xB39340u);
    // Every code is close to balanced, and B and C differ.
    std::vector<uint8_t> b(GAL_E1_LEN);
    for (int prn = 1; prn <= GAL_MAX_PRN; prn++) {
        ASSERT_EQ(gal_e1_code(prn, GNSS_SIG_GAL_E1B, b.data()), 0);
        ASSERT_EQ(gal_e1_code(prn, GNSS_SIG_GAL_E1C, c.data()), 0);
        EXPECT_NEAR(ones(b), GAL_E1_LEN / 2, 64) << prn;
        EXPECT_NEAR(ones(c), GAL_E1_LEN / 2, 64) << prn;
        EXPECT_NE(b, c) << prn;
    }
    EXPECT_EQ(gal_e1_code(51, GNSS_SIG_GAL_E1B, c.data()), -1);
    // CS25_1, OS SIS ICD 3.4.2: 0 0 1 1 1 0 0 0 0 0 0 0 1 0 1 0 1 1 0 1 1 0 0 1 0.
    const uint8_t want[GAL_E1C_SEC_LEN] = {0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0};
    uint8_t cs[GAL_E1C_SEC_LEN];
    gal_e1c_secondary(cs);
    for (int i = 0; i < GAL_E1C_SEC_LEN; i++) {
        EXPECT_EQ(cs[i], want[i]) << i;
    }
}

TEST(CodesMulti, SignalDefinitions)
{
    const gnss_sigdef_t *e1b = gnss_sigdef(GNSS_SIG_GAL_E1B), *b1cp = gnss_sigdef(GNSS_SIG_BDS_B1CP);
    ASSERT_NE(e1b, nullptr);
    ASSERT_NE(b1cp, nullptr);
    EXPECT_EQ(e1b->code_len, GAL_E1_LEN);
    EXPECT_EQ(e1b->symbol_ms, 4);
    EXPECT_EQ(e1b->boc11, 1);
    EXPECT_EQ(b1cp->code_len, BDS_B1C_LEN);
    EXPECT_EQ(b1cp->symbol_ms, 0);
    EXPECT_EQ(gnss_sigdef(GNSS_SIG_BDS_B1CD)->symbol_ms, 10);
}
