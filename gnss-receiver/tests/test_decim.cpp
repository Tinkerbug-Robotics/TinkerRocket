#include "decim.h"
#include "fe_format.h"

#include <gtest/gtest.h>

TEST(Decim, SubsampleKeepsOnePhase)
{
    uint8_t in[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    uint8_t out[4];
    decim_t d;
    decim_init(&d, DECIM_SUBSAMPLE, 2);
    size_t n = decim_process(&d, in, 5, out);
    n += decim_process(&d, in + 5, 7, out + n);
    ASSERT_EQ(n, 3u);
    EXPECT_EQ(out[0], 2);
    EXPECT_EQ(out[1], 6);
    EXPECT_EQ(out[2], 10);
}

TEST(Decim, Sum4RequantizesTheWeightedSum)
{
    const uint8_t pos_large = FE_CODE_I_MAG | FE_CODE_Q_MAG;   // I +3, Q +3
    const uint8_t neg_small = FE_CODE_I_SIGN | FE_CODE_Q_SIGN;  // I -1, Q -1
    // I: +3 +3 -1 -1 = +4 -> +large; Q: +3 -1 -1 -1 = 0 -> +small.
    uint8_t in[4] = {pos_large, (uint8_t)(FE_CODE_I_MAG | FE_CODE_Q_SIGN), neg_small, neg_small};
    uint8_t out[1];
    decim_t d;
    decim_init(&d, DECIM_SUM4, 0);
    ASSERT_EQ(decim_process(&d, in, 4, out), 1u);
    EXPECT_EQ(out[0], FE_CODE_I_MAG);
}
