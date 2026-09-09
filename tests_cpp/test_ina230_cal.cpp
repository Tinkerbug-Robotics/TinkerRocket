// #1155 item 17: an out-of-range CAL must clamp, not wrap into a plausible value.
#include <gtest/gtest.h>
#include <cmath>
#include <TR_INA230_Cal.h>

using tr_ina230::computeCalibration;

TEST(Ina230Cal, NominalShuntProgramsTheDatasheetValue)
{
    uint16_t cal = 0;
    ASSERT_TRUE(computeCalibration(0.002f, 0.001f, cal));   // 2 mOhm, 1 mA/bit
    EXPECT_EQ(cal, 2560u);                                   // 0.00512 / (1e-3 * 2e-3)
}

TEST(Ina230Cal, OutOfRangeClampsInsteadOfWrapping)
{
    uint16_t cal = 0;
    // 0.00512 / (1e-6 * 2e-3) = 2,560,000. (uint16_t)2560000 == 4096, which the
    // old code accepted after its too-late clamp.
    ASSERT_TRUE(computeCalibration(0.002f, 0.000001f, cal));
    EXPECT_EQ(cal, 0x7FFFu);
    EXPECT_NE(cal, 4096u);
    // Just past 65535 wrapped to a two-digit number.
    ASSERT_TRUE(computeCalibration(0.000078f, 0.001f, cal));   // ~65641
    EXPECT_EQ(cal, 0x7FFFu);
}

TEST(Ina230Cal, ValueThatRoundsToZeroIsRefused)
{
    uint16_t cal = 77;
    EXPECT_FALSE(computeCalibration(1.0f, 10.0f, cal));   // 0.00512 -> CAL 0
    EXPECT_EQ(cal, 77u);                                  // untouched on refusal
}

TEST(Ina230Cal, NonPositiveAndNanAreRefused)
{
    uint16_t cal = 0;
    EXPECT_FALSE(computeCalibration(0.0f, 0.001f, cal));
    EXPECT_FALSE(computeCalibration(0.002f, -0.001f, cal));
    EXPECT_FALSE(computeCalibration(NAN, 0.001f, cal));
    EXPECT_FALSE(computeCalibration(0.002f, NAN, cal));
    EXPECT_FALSE(computeCalibration(INFINITY, 0.001f, cal));   // 0.00512/inf = 0 -> refused
}
