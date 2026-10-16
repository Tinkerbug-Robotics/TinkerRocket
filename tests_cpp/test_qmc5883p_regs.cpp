#include <gtest/gtest.h>
#include "qmc5883p_regs.h"

// The QMC5883P register packing (#1312), pinned against the datasheet's own
// worked examples (QMC5883P Datasheet Rev. C, section 7) so the bytes the
// driver writes are the vendor's and not a transcription.  Header-only and
// IDF-free; the I2C traffic that carries them is bench-only.

using namespace qmc5883p;

TEST(Qmc5883pRegs, NormalModeSetupExample) {
    // 7.1: "Write Register 0BH by 0x08 (Define Set/Reset mode, with Set/Reset
    // On, Field Range 8Guass)" then "Write Register 0AH by 0xCD (set normal
    // mode, set ODR=200Hz)".
    EXPECT_EQ(ctrl2(Range::G8, SetReset::SET_AND_RESET_ON), 0x08);
    EXPECT_EQ(ctrl1(Mode::NORMAL, Odr::HZ_200, Osr1::X8, Osr2::X8), 0xCD);
}

TEST(Qmc5883pRegs, ContinuousModeSetupExample) {
    // 7.2: "Write Register 0AH by 0xC3 (set continuous mode)" — OSR2=8,
    // OSR1=8, the ODR bits at their reset value, mode 11.
    EXPECT_EQ(ctrl1(Mode::CONTINUOUS, Odr::HZ_10, Osr1::X8, Osr2::X8), 0xC3);
}

TEST(Qmc5883pRegs, SelfTestAndSoftResetExamples) {
    // 7.3: "Write Register 0AH by 0x03 (set continuous mode)" and "Write
    // Register 0BH by 0x40 (enter self-test function)"; 7.6: "Write Register
    // 0BH by 0x80" (soft reset).
    EXPECT_EQ(ctrl1(Mode::CONTINUOUS, Odr::HZ_10, Osr1::X8, Osr2::X1), 0x03);
    EXPECT_EQ(CTRL2_SELF_TEST, 0x40);
    EXPECT_EQ(CTRL2_SOFT_RST,  0x80);
}

TEST(Qmc5883pRegs, TheDriverDefaultIsNormalMode100HzMostFiltering) {
    // TR_QMC5883P::configure() with no arguments writes 0xC9: OSR2=8,
    // OSR1=8, ODR=100 Hz, normal mode — the IIS2MDC's cadence, and the
    // 10 ms the collector's poll gate assumes.
    EXPECT_EQ(ctrl1(Mode::NORMAL, Odr::HZ_100, Osr1::X8, Osr2::X8), 0xC9);
    EXPECT_EQ(odrPeriodUs(Odr::HZ_100), 10000u);
    // The suspend word configure() parks the part with keeps the OSR/ODR
    // bits and clears only the mode.
    EXPECT_EQ(ctrl1(Mode::SUSPEND, Odr::HZ_100, Osr1::X8, Osr2::X8), 0xC8);
}

TEST(Qmc5883pRegs, BitFieldsDoNotOverlap) {
    EXPECT_EQ(ctrl1(Mode::CONTINUOUS, Odr::HZ_10,  Osr1::X8, Osr2::X1), 0x03);
    EXPECT_EQ(ctrl1(Mode::SUSPEND,    Odr::HZ_200, Osr1::X8, Osr2::X1), 0x0C);
    EXPECT_EQ(ctrl1(Mode::SUSPEND,    Odr::HZ_10,  Osr1::X1, Osr2::X1), 0x30);
    EXPECT_EQ(ctrl1(Mode::SUSPEND,    Odr::HZ_10,  Osr1::X8, Osr2::X8), 0xC0);
    EXPECT_EQ(ctrl2(Range::G30, SetReset::SET_AND_RESET_ON), 0x00);
    EXPECT_EQ(ctrl2(Range::G2,  SetReset::SET_AND_RESET_ON), 0x0C);
    EXPECT_EQ(ctrl2(Range::G30, SetReset::OFF),              0x02);
}

TEST(Qmc5883pRegs, SensitivityTableAndTheLoggedScale) {
    // Table 2.
    EXPECT_EQ(lsbPerGauss(Range::G30), 1000);
    EXPECT_EQ(lsbPerGauss(Range::G12), 2500);
    EXPECT_EQ(lsbPerGauss(Range::G8),  3750);
    EXPECT_EQ(lsbPerGauss(Range::G2),  15000);
    // The ±8 G scale is what MAG_TYPE_QMC5883P promises every log reader.
    EXPECT_FLOAT_EQ(uTPerLsb(Range::G8), 100.0f / 3750.0f);
    EXPECT_NEAR(uTPerLsb(Range::G8), 0.02667f, 1e-5f);
}

TEST(Qmc5883pRegs, FixedAddressChipIdAndMap) {
    EXPECT_EQ(I2C_ADDR, 0x2C);          // datasheet 5.4
    EXPECT_EQ(CHIP_ID, 0x80);           // 9.2.1
    EXPECT_EQ(REG_CHIP_ID, 0x00);
    EXPECT_EQ(REG_XOUT_L, 0x01);
    EXPECT_EQ(REG_ZOUT_H, 0x06);
    EXPECT_EQ(REG_STATUS, 0x09);
    EXPECT_EQ(REG_CTRL1, 0x0A);
    EXPECT_EQ(REG_CTRL2, 0x0B);
    EXPECT_EQ(REG_AXIS_SIGN, 0x29);     // section 7, every example
    EXPECT_EQ(STATUS_DRDY, 0x01);
    EXPECT_EQ(STATUS_OVFL, 0x02);
}

TEST(Qmc5883pRegs, TheSignWordIsTheResetValueNotTheDatasheetExample) {
    // Section 7 writes 0x06.  The driver writes 0x00: every QMC5883P log so
    // far, and the chip signs #1590 measured from one, were taken at 0x00,
    // because the 0x06 write never took (#1590 scale).  0x06 would most
    // likely turn Y and Z, and with them the Beetle's pad heading.
    EXPECT_EQ(AXIS_SIGN_VALUE, 0x00);
    EXPECT_EQ(AXIS_SIGN_DATASHEET_EXAMPLE, 0x06);
}

TEST(Qmc5883pRegs, ReadbackComparesCtrl1WholeAndCtrl2UnderItsMask) {
    const uint8_t c1 = ctrl1(Mode::NORMAL, Odr::HZ_100, Osr1::X8, Osr2::X8);   // 0xC9
    const uint8_t c2 = ctrl2(Range::G8, SetReset::SET_AND_RESET_ON);          // 0x08
    EXPECT_TRUE(configTook({0xC9, 0x08, 0x00}, c1, c2, AXIS_SIGN_VALUE));
    // The Beetle before the fix: CTRL1 took, CTRL2 and the sign word did not.
    EXPECT_FALSE(configTook({0xC9, 0x00, 0x00}, c1, c2, AXIS_SIGN_VALUE));
    EXPECT_FALSE(configTook({0xC9, 0x08, 0x06}, c1, c2, AXIS_SIGN_VALUE));
    EXPECT_FALSE(configTook({0xC8, 0x08, 0x00}, c1, c2, AXIS_SIGN_VALUE));   // still suspended
    // SOFT_RST or SELF_TEST still set is a miss; the RFU bits 5:4 are not.
    EXPECT_FALSE(configTook({0xC9, 0x88, 0x00}, c1, c2, AXIS_SIGN_VALUE));
    EXPECT_FALSE(configTook({0xC9, 0x48, 0x00}, c1, c2, AXIS_SIGN_VALUE));
    EXPECT_TRUE(configTook({0xC9, 0x38, 0x00}, c1, c2, AXIS_SIGN_VALUE));
    EXPECT_EQ(CTRL2_READBACK_MASK, 0xCF);
}

TEST(Qmc5883pRegs, RangeOfCtrl2) {
    EXPECT_EQ(rangeOfCtrl2(0x00), Range::G30);   // the Beetle's readback before the fix
    EXPECT_EQ(rangeOfCtrl2(0x04), Range::G12);
    EXPECT_EQ(rangeOfCtrl2(0x08), Range::G8);
    EXPECT_EQ(rangeOfCtrl2(0x0C), Range::G2);
    EXPECT_EQ(rangeOfCtrl2(0xB7), Range::G12);   // only bits 3:2 count
    EXPECT_EQ(rangeOfCtrl2(0xF3), Range::G30);
    EXPECT_EQ(fullScaleGauss(Range::G30), 30);
    EXPECT_EQ(fullScaleGauss(Range::G12), 12);
    EXPECT_EQ(fullScaleGauss(Range::G8), 8);
    EXPECT_EQ(fullScaleGauss(Range::G2), 2);
}

TEST(Qmc5883pRegs, CountsConvertToTheEightGaussScale) {
    EXPECT_EQ(CONTRACT_RANGE, Range::G8);
    // ±8 G is the identity, end to end.
    EXPECT_EQ(toContractCounts(0, Range::G8), 0);
    EXPECT_EQ(toContractCounts(1875, Range::G8), 1875);
    EXPECT_EQ(toContractCounts(-32768, Range::G8), -32768);
    EXPECT_EQ(toContractCounts(32767, Range::G8), 32767);
    // ±30 G: x3.75.  The Beetle's 13.27 uT at rest (498 counts at the logged
    // scale, which were 498 counts at 1000 LSB/G) is 49.8 uT.
    EXPECT_EQ(toContractCounts(498, Range::G30), 1868);
    EXPECT_NEAR(toContractCounts(498, Range::G30) * uTPerLsb(Range::G8), 49.8f, 0.05f);
    EXPECT_EQ(toContractCounts(1, Range::G30), 4);      // 3.75, rounded
    EXPECT_EQ(toContractCounts(-1, Range::G30), -4);
    EXPECT_EQ(toContractCounts(2, Range::G30), 8);      // 7.5, half away from zero
    EXPECT_EQ(toContractCounts(-2, Range::G30), -8);
    EXPECT_EQ(toContractCounts(8737, Range::G30), 32764);
    EXPECT_EQ(toContractCounts(8738, Range::G30), 32767);    // 32767.5 saturates
    EXPECT_EQ(toContractCounts(30000, Range::G30), 32767);   // the OVFL threshold
    EXPECT_EQ(toContractCounts(-8739, Range::G30), -32768);
    // ±12 G is x1.5, ±2 G x0.25.
    EXPECT_EQ(toContractCounts(1000, Range::G12), 1500);
    EXPECT_EQ(toContractCounts(15000, Range::G2), 3750);
    EXPECT_EQ(toContractCounts(1, Range::G2), 0);
    EXPECT_EQ(toContractCounts(2, Range::G2), 1);
}

TEST(Qmc5883pRegs, DecodeIsLittleEndianTwosComplement) {
    const uint8_t minus_one[2] = {0xFF, 0xFF};
    const uint8_t floor_[2]    = {0x00, 0x80};   // -32768, the saturation floor
    const uint8_t one_gauss[2] = {0xA6, 0x0E};   // 0x0EA6 = 3750
    EXPECT_EQ(decodeAxis(minus_one), -1);
    EXPECT_EQ(decodeAxis(floor_), -32768);
    EXPECT_EQ(decodeAxis(one_gauss), 3750);
}

TEST(Qmc5883pRegs, OffsetSubtractionSaturatesLikeTheSTPart) {
    EXPECT_EQ(subtractSaturating(100, 30), 70);
    EXPECT_EQ(subtractSaturating(-100, 30), -130);
    EXPECT_EQ(subtractSaturating(0, 0), 0);
    EXPECT_EQ(subtractSaturating(32000, -2000), 32767);
    EXPECT_EQ(subtractSaturating(-32000, 2000), -32768);
    EXPECT_EQ(subtractSaturating(-32768, 32767), -32768);
    // The Rolly Polly V hard iron (#1303: ~-209 µT on one axis) is -7838 QMC
    // counts — well inside the int16 the offset rides in.  A 50 µT field
    // (1875 counts) on top of it comes back as exactly the field.
    EXPECT_EQ(subtractSaturating(-7838 + 1875, -7838), 1875);
}

TEST(Qmc5883pRegs, OneBurstReachesStatusAtIndexEight)
{
    // #1485: readRawXYZ reads 01H through 09H in one transaction — the axes,
    // the two unmapped bytes, then STATUS — instead of a separate status read.
    EXPECT_EQ(BURST_WITH_STATUS_LEN, 9);
    EXPECT_EQ(BURST_STATUS_INDEX, 8);
    EXPECT_EQ(REG_XOUT_L + BURST_STATUS_INDEX, REG_STATUS);
}
