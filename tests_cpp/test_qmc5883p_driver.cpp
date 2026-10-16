#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <utility>
#include <vector>

#include "TR_QMC5883P.h"

// TR_QMC5883P against a fake of the QMC5883P's register file (#1590 scale).
//
// The Beetle's boot readback was CTRL1=0xC9 CTRL2=0x00 AXIS_SIGN=0x00: the
// ±8 G range and the sign word never took, only the final CTRL1 write did.
// The fake's default reset model reproduces that from the old driver's
// writes, and these tests hold the driver to leaving ±8 G and sign word 0x00
// in that part.  A second model, a reset that clears itself, is the
// datasheet's text read naively.  The bench readback rules it out, but the
// driver has to work on it too.  The I2C calls the driver makes resolve to
// the fake below (host_shim/driver/i2c_master.h declares them).

using namespace qmc5883p;

namespace {

enum class ResetModel {
    // The bench: SOFT_RST stays set until written back to 0.  While it is
    // set, the part keeps every register at its default and no write sticks.
    // The write that clears it does not stick either.
    HELD_UNTIL_CLEARED,
    // The reset completes at once and the bit reads 0.
    SELF_CLEARING,
};

struct FakeQmc5883p {
    ResetModel model = ResetModel::HELD_UNTIL_CLEARED;
    uint8_t regs[0x30] = {};
    bool in_reset = false;

    // The field at the part, in gauss, in the frame sign word 0x00 reads.
    double field_g[3] = {0.20, -0.30, 0.40};

    // Fault injection.
    int  drop_ctrl2_writes = 0;          // the next N CTRL2 writes are ACKed and lost
    bool range_never_takes = false;      // CTRL2's RNG bits never change
    int  nack_from_transfer = -1;        // every transfer from this index on fails

    int transfers = 0;
    std::vector<std::pair<uint8_t, uint8_t>> writes;   // every (reg, value) written

    FakeQmc5883p() { powerOn(); }

    void powerOn()
    {
        std::memset(regs, 0, sizeof(regs));
        regs[REG_CHIP_ID] = CHIP_ID;
        in_reset = false;
    }

    void restoreDefaults()
    {
        std::memset(regs, 0, sizeof(regs));
        regs[REG_CHIP_ID] = CHIP_ID;
    }

    void write(uint8_t reg, uint8_t v)
    {
        writes.emplace_back(reg, v);
        if (reg == REG_CTRL2 && (v & CTRL2_SOFT_RST))
        {
            restoreDefaults();
            if (model == ResetModel::HELD_UNTIL_CLEARED)
            {
                regs[REG_CTRL2] = CTRL2_SOFT_RST;
                in_reset = true;
            }
            return;
        }
        if (in_reset)
        {
            // Held: only the write that clears SOFT_RST does anything, and it
            // releases the part with every register at its default.
            if (reg == REG_CTRL2)
            {
                restoreDefaults();
                in_reset = false;
            }
            return;
        }
        if (reg == REG_CTRL2)
        {
            if (drop_ctrl2_writes > 0) { --drop_ctrl2_writes; return; }
            if (range_never_takes)
                v = (uint8_t)((v & ~0x0Cu) | (regs[REG_CTRL2] & 0x0Cu));
            regs[REG_CTRL2] = v;
            return;
        }
        if (reg == REG_CTRL1 || reg == REG_AXIS_SIGN) regs[reg] = v;
        // Everything else is read-only.
    }

    int16_t axisCounts(int axis) const
    {
        const Range r = rangeOfCtrl2(regs[REG_CTRL2]);
        const double c = std::lround(field_g[axis] * lsbPerGauss(r));
        return (int16_t)(c > 32767 ? 32767 : c < -32768 ? -32768 : c);
    }

    uint8_t read(uint8_t reg) const
    {
        if (reg >= REG_XOUT_L && reg <= REG_ZOUT_H)
        {
            if ((regs[REG_CTRL1] & 0x3u) == 0) return 0;   // suspend: no measurement
            const int axis = (reg - REG_XOUT_L) / 2;
            const uint16_t c = (uint16_t)axisCounts(axis);
            return ((reg - REG_XOUT_L) % 2 == 0) ? (uint8_t)(c & 0xFF) : (uint8_t)(c >> 8);
        }
        if (reg == REG_STATUS) return 0;   // no refresh lands mid-burst here
        return reg < sizeof(regs) ? regs[reg] : 0;
    }

    bool nack() { return nack_from_transfer >= 0 && transfers++ >= nack_from_transfer; }

    ConfigReadback readback() const
    {
        return {regs[REG_CTRL1], regs[REG_CTRL2], regs[REG_AXIS_SIGN]};
    }
};

FakeQmc5883p* g_fake = nullptr;

}  // namespace

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t, const i2c_device_config_t* cfg,
                                    i2c_master_dev_handle_t* ret_handle)
{
    if (cfg->device_address != I2C_ADDR) return ESP_FAIL;
    *ret_handle = reinterpret_cast<i2c_master_dev_handle_t>(g_fake);
    return ESP_OK;
}

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t, const uint8_t* buf, size_t len, int)
{
    if (g_fake->nack() || len != 2) return ESP_FAIL;
    g_fake->write(buf[0], buf[1]);
    return ESP_OK;
}

esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t, const uint8_t* wbuf, size_t wlen,
                                      uint8_t* rbuf, size_t rlen, int)
{
    if (g_fake->nack() || wlen != 1) return ESP_FAIL;
    for (size_t i = 0; i < rlen; ++i) rbuf[i] = g_fake->read((uint8_t)(wbuf[0] + i));
    return ESP_OK;
}

namespace {

i2c_master_bus_handle_t fakeBus() { return reinterpret_cast<i2c_master_bus_handle_t>(0x1); }

// The bytes the driver used to write from begin() through configure().
void replayPreFixSequence(FakeQmc5883p& f)
{
    f.write(REG_CTRL2, 0x80);   // softReset(), then 5 ms
    f.write(REG_AXIS_SIGN, 0x06);
    f.write(REG_CTRL1, 0xC8);
    f.write(REG_CTRL2, 0x08);
    f.write(REG_CTRL1, 0xC9);
}

class Qmc5883pDriver : public ::testing::Test {
protected:
    FakeQmc5883p fake;
    TR_QMC5883P  drv;
    void SetUp() override { g_fake = &fake; }
    void TearDown() override { g_fake = nullptr; }
};

}  // namespace

TEST_F(Qmc5883pDriver, TheHeldResetModelReproducesTheBeetlesBootReadback)
{
    // dc37cf07 on the Beetle, 2026-10-15: "QMC5883P CTRL1=0xC9 CTRL2=0x00
    // (RNG=0) AXIS_SIGN=0x00".
    replayPreFixSequence(fake);
    const ConfigReadback rb = fake.readback();
    EXPECT_EQ(rb.ctrl1, 0xC9);
    EXPECT_EQ(rb.ctrl2, 0x00);
    EXPECT_EQ(rb.axis_sign, 0x00);
    EXPECT_EQ(rangeOfCtrl2(rb.ctrl2), Range::G30);

    // A reset that clears itself would have kept every write -- not what the
    // part reported, so that is not how it behaves.
    FakeQmc5883p naive;
    naive.model = ResetModel::SELF_CLEARING;
    replayPreFixSequence(naive);
    EXPECT_EQ(naive.readback().ctrl2, 0x08);
    EXPECT_EQ(naive.readback().axis_sign, 0x06);
}

TEST_F(Qmc5883pDriver, BeginAndConfigureLeaveEightGaussAndSignWordZeroInThePart)
{
    ASSERT_EQ(drv.begin(fakeBus()), TR_QMC5883P_OK);
    EXPECT_FALSE(fake.in_reset) << "softReset() must release SOFT_RST itself";
    ASSERT_EQ(drv.configure(), TR_QMC5883P_OK);

    EXPECT_TRUE(drv.configVerified());
    EXPECT_EQ(drv.configAttempts(), 1);
    EXPECT_EQ(drv.range(), Range::G8);
    const ConfigReadback rb = fake.readback();
    EXPECT_EQ(rb.ctrl1, 0xC9);       // OSR2=8 OSR1=8, 100 Hz, normal
    EXPECT_EQ(rb.ctrl2, 0x08);       // ±8 G, set/reset on
    EXPECT_EQ(rb.axis_sign, 0x00);   // the frame #1590 measured
    EXPECT_EQ(drv.configReadback().ctrl2, 0x08);
}

TEST_F(Qmc5883pDriver, TheSelfClearingModelConfiguresInOnePassToo)
{
    fake.model = ResetModel::SELF_CLEARING;
    ASSERT_EQ(drv.begin(fakeBus()), TR_QMC5883P_OK);
    ASSERT_EQ(drv.configure(), TR_QMC5883P_OK);
    EXPECT_TRUE(drv.configVerified());
    EXPECT_EQ(drv.configAttempts(), 1);
    EXPECT_EQ(fake.readback().ctrl2, 0x08);
}

TEST_F(Qmc5883pDriver, SoftResetIsReleasedBeforeAnyOtherWrite)
{
    ASSERT_EQ(drv.begin(fakeBus()), TR_QMC5883P_OK);
    ASSERT_GE(fake.writes.size(), 2u);
    EXPECT_EQ(fake.writes[0], std::make_pair(REG_CTRL2, (uint8_t)0x80));
    EXPECT_EQ(fake.writes[1], std::make_pair(REG_CTRL2, (uint8_t)0x00));
}

TEST_F(Qmc5883pDriver, TheDatasheetSignWordIsNeverWritten)
{
    // 0x06 would most likely turn Y and Z, and the Beetle's pad heading with
    // them (qmc5883p_regs.h).  Every sign-word write is 0x00.
    ASSERT_EQ(drv.begin(fakeBus()), TR_QMC5883P_OK);
    fake.drop_ctrl2_writes = 1;   // force a second pass: it must not differ
    ASSERT_EQ(drv.configure(), TR_QMC5883P_OK);
    int sign_writes = 0;
    for (const auto& w : fake.writes)
    {
        if (w.first != REG_AXIS_SIGN) continue;
        ++sign_writes;
        EXPECT_EQ(w.second, 0x00);
    }
    EXPECT_EQ(sign_writes, 2);
}

TEST_F(Qmc5883pDriver, ALostRangeWriteIsWrittenAgain)
{
    ASSERT_EQ(drv.begin(fakeBus()), TR_QMC5883P_OK);
    fake.drop_ctrl2_writes = 1;
    ASSERT_EQ(drv.configure(), TR_QMC5883P_OK);
    EXPECT_TRUE(drv.configVerified());
    EXPECT_EQ(drv.configAttempts(), 2);
    EXPECT_EQ(fake.readback().ctrl2, 0x08);
    EXPECT_EQ(drv.range(), Range::G8);
}

TEST_F(Qmc5883pDriver, ConfigureRecoversAPartLeftInResetWithoutABegin)
{
    // The stall-revive path calls configure() without begin()'s reset.  A
    // part held in reset loses the first pass, whose CTRL2 write releases
    // it, and keeps the second.
    ASSERT_EQ(drv.begin(fakeBus()), TR_QMC5883P_OK);
    fake.write(REG_CTRL2, CTRL2_SOFT_RST);
    ASSERT_TRUE(fake.in_reset);
    ASSERT_EQ(drv.configure(), TR_QMC5883P_OK);
    EXPECT_TRUE(drv.configVerified());
    EXPECT_EQ(drv.configAttempts(), 2);
    EXPECT_EQ(fake.readback().ctrl2, 0x08);
}

TEST_F(Qmc5883pDriver, TheFieldReadsAtTheLoggedScale)
{
    ASSERT_EQ(drv.begin(fakeBus()), TR_QMC5883P_OK);
    ASSERT_EQ(drv.configure(), TR_QMC5883P_OK);

    QMC5883P_RawData raw{};
    ASSERT_EQ(drv.readRawXYZ(&raw), TR_QMC5883P_OK);
    EXPECT_EQ(raw.x, 750);      // 0.20 G at 3750 LSB/G
    EXPECT_EQ(raw.y, -1125);
    EXPECT_EQ(raw.z, 1500);

    float x = 0, y = 0, z = 0;
    ASSERT_EQ(drv.readFieldsXYZ_uT(&x, &y, &z), TR_QMC5883P_OK);
    EXPECT_NEAR(x, 20.0f, 1e-3f);
    EXPECT_NEAR(y, -30.0f, 1e-3f);
    EXPECT_NEAR(z, 40.0f, 1e-3f);
    EXPECT_FLOAT_EQ(drv.lsbToUt(), QMC5883P_LSB_TO_uT);
}

TEST_F(Qmc5883pDriver, ARangeThatNeverTakesStillHandsOutEightGaussCounts)
{
    // The defence behind the fix: if the part answers every write and still
    // sits at ±30 G, its counts are converted, not passed on 3.75x small.
    ASSERT_EQ(drv.begin(fakeBus()), TR_QMC5883P_OK);
    fake.range_never_takes = true;
    ASSERT_EQ(drv.configure(), TR_QMC5883P_OK) << "never a reason to halt the boot";
    EXPECT_FALSE(drv.configVerified());
    EXPECT_EQ(drv.configAttempts(), TR_QMC5883P::CONFIGURE_ATTEMPTS);
    EXPECT_EQ(drv.range(), Range::G30);
    EXPECT_EQ(rangeOfCtrl2(drv.configReadback().ctrl2), Range::G30);

    QMC5883P_RawData raw{};
    ASSERT_EQ(drv.readRawXYZ(&raw), TR_QMC5883P_OK);
    EXPECT_EQ(fake.axisCounts(0), 200);   // what the part hands over at 1000 LSB/G
    EXPECT_EQ(raw.x, 750);
    EXPECT_EQ(raw.y, -1125);
    EXPECT_EQ(raw.z, 1500);

    float x = 0, y = 0, z = 0;
    ASSERT_EQ(drv.readFieldsXYZ_uT(&x, &y, &z), TR_QMC5883P_OK);
    EXPECT_NEAR(std::sqrt(x * x + y * y + z * z), std::sqrt(20.0 * 20 + 30 * 30 + 40 * 40), 1e-3);
}

TEST_F(Qmc5883pDriver, HardIronIsSubtractedInEightGaussCounts)
{
    // The offsets the calibrator stores are ±8 G counts; they come off after
    // any range conversion, never before.
    ASSERT_EQ(drv.begin(fakeBus()), TR_QMC5883P_OK);
    fake.range_never_takes = true;
    ASSERT_EQ(drv.configure(), TR_QMC5883P_OK);
    ASSERT_EQ(drv.setHardIronOffset(100, -125, 0), TR_QMC5883P_OK);
    QMC5883P_RawData raw{};
    ASSERT_EQ(drv.readRawXYZ(&raw), TR_QMC5883P_OK);
    EXPECT_EQ(raw.x, 650);
    EXPECT_EQ(raw.y, -1000);
    EXPECT_EQ(raw.z, 1500);
}

TEST_F(Qmc5883pDriver, AFailedTransferIsAnError)
{
    ASSERT_EQ(drv.begin(fakeBus()), TR_QMC5883P_OK);
    fake.transfers = 0;
    fake.nack_from_transfer = 2;   // the third transfer of configure() fails
    EXPECT_EQ(drv.configure(), TR_QMC5883P_ERROR);
    EXPECT_FALSE(drv.configVerified());

    FakeQmc5883p readback_fails;
    g_fake = &readback_fails;
    TR_QMC5883P drv2;
    ASSERT_EQ(drv2.begin(fakeBus()), TR_QMC5883P_OK);
    readback_fails.nack_from_transfer = 4;   // the four writes pass, the readback fails
    EXPECT_EQ(drv2.configure(), TR_QMC5883P_ERROR);
    EXPECT_FALSE(drv2.configVerified());
}

TEST_F(Qmc5883pDriver, SoftResetZeroesTheOffsetAndBeginRejectsAnotherChip)
{
    ASSERT_EQ(drv.begin(fakeBus()), TR_QMC5883P_OK);
    ASSERT_EQ(drv.setHardIronOffset(1, 2, 3), TR_QMC5883P_OK);
    ASSERT_EQ(drv.softReset(), TR_QMC5883P_OK);
    int16_t ox = -1, oy = -1, oz = -1;
    drv.getHardIronOffset(&ox, &oy, &oz);
    EXPECT_EQ(ox, 0);
    EXPECT_EQ(oy, 0);
    EXPECT_EQ(oz, 0);
    EXPECT_FALSE(fake.in_reset);

    FakeQmc5883p other;
    other.regs[REG_CHIP_ID] = 0x40;   // e.g. an IIS2MDC's WHO_AM_I
    g_fake = &other;
    TR_QMC5883P drv2;
    EXPECT_EQ(drv2.begin(fakeBus()), TR_QMC5883P_ERROR);
}
