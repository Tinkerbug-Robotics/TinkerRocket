// test_sim_high_g_bias.cpp — the rig replay's high-g counts carry the flight
// computer's high-g bias, so the converter returns the flight's own specific
// force where the low-g rail hands over to the high-g channel (#1600 on the V9
// mosaic image; SimHighGBias.h).
//
// Before this, the replay wrote the force itself into the high-g counts and
// the converter took the V9's NVS bias (about 1.3 m/s^2) off every railed
// sample: 48 ms at 18.8 g in rp5_0829, 0.03-0.06 m/s by burnout. These run
// the counts through SensorConverter itself, at the sensor rotations the
// boards use and the bias the Goblin's V9 holds.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "SimHighGBias.h"
#include "TR_Sensor_Data_Converter.h"

namespace
{

constexpr float kG = 9.80665f;
constexpr float kHighLsb = (256.0f * 1000.0f / 32768.0f) * 1.0e-3f * kG;   // the sim's ACC_HIGH_MS2_PER_LSB

SensorConverter converter(float rot_z_deg, float bx, float by, float bz)
{
    SensorConverter c;
    c.configureISM6HG256FullScale(ISM6LowGFullScale::FS_16G, ISM6HighGFullScale::FS_256G,
                                  ISM6GyroFullScale::DPS_4000);
    c.configureISM6HG256RotationZ(rot_z_deg);
    c.setHighGBias(bx, by, bz);
    return c;
}

// The high-g reading the converter makes of counts the replay packed for a
// board-frame force f with the bias it was handed
void roundTrip(const SensorConverter& conv_const, float rot_z_deg, const float f[3], const float bias[3],
               double out[3])
{
    SensorConverter conv = conv_const;
    const float rad = rot_z_deg * static_cast<float>(M_PI / 180.0);
    ISM6HG256Data raw{};
    int16_t hg[3];
    sim_high_g::countsCarryingBias(f, bias, cosf(rad), sinf(rad), kHighLsb, hg);
    raw.acc_high_raw.x = hg[0];
    raw.acc_high_raw.y = hg[1];
    raw.acc_high_raw.z = hg[2];
    ISM6HG256DataSI si{};
    conv.convertISM6HG256Data(raw, si);
    out[0] = si.high_g_acc_x;
    out[1] = si.high_g_acc_y;
    out[2] = si.high_g_acc_z;
}

}  // namespace

TEST(SimHighGBias, TheConverterReturnsTheFlightsForceWithTheBiasCarried)
{
    // the boost's railed samples: 18.8 g along the nose, a little across
    const float f[3] = {18.8f * kG, 0.6f, -1.1f};
    const float bias[3] = {1.31f, -0.42f, 0.27f};   // about the Goblin's V9
    for (float rot : {0.0f, 90.0f, -90.0f, 30.0f, 180.0f})
    {
        const SensorConverter conv = converter(rot, bias[0], bias[1], bias[2]);
        double got[3];
        roundTrip(conv, rot, f, bias, got);
        for (int i = 0; i < 3; ++i)
            EXPECT_NEAR(got[i], f[i], 0.75 * kHighLsb) << "rotation " << rot << " axis " << i;
    }
}

TEST(SimHighGBias, WithoutTheBiasEveryRailedSampleCameOutItShort)
{
    // what the replay did before: the force alone in the counts
    const float f[3] = {18.8f * kG, 0.0f, 0.0f};
    const float bias[3] = {1.31f, 0.0f, 0.0f};
    const float none[3] = {0.0f, 0.0f, 0.0f};
    const SensorConverter conv = converter(0.0f, bias[0], bias[1], bias[2]);
    double got[3];
    roundTrip(conv, 0.0f, f, none, got);
    EXPECT_NEAR(got[0], f[0] - bias[0], 0.75 * kHighLsb);
    // 48 ms of it: the velocity the replay used to lose by burnout
    EXPECT_NEAR(bias[0] * 0.048, 0.063, 0.001);
}

TEST(SimHighGBias, NoBiasIsTheOldCounts)
{
    // a board whose NVS holds no bias: the counts are what they always were
    const float f[3] = {12.0f * kG, -3.0f, 2.5f};
    const float none[3] = {0.0f, 0.0f, 0.0f};
    const float rad = 30.0f * static_cast<float>(M_PI / 180.0);
    const float c = cosf(rad), s = sinf(rad);
    int16_t hg[3];
    sim_high_g::countsCarryingBias(f, none, c, s, kHighLsb, hg);
    EXPECT_EQ(hg[0], static_cast<int16_t>(lroundf((f[0] * c + f[1] * s) / kHighLsb)));
    EXPECT_EQ(hg[1], static_cast<int16_t>(lroundf((-f[0] * s + f[1] * c) / kHighLsb)));
    EXPECT_EQ(hg[2], static_cast<int16_t>(lroundf(f[2] / kHighLsb)));
}

TEST(SimHighGBias, TheCountsClampAtTheRange)
{
    const float f[3] = {400.0f * kG, -400.0f * kG, 0.0f};
    const float none[3] = {0.0f, 0.0f, 0.0f};
    int16_t hg[3];
    sim_high_g::countsCarryingBias(f, none, 1.0f, 0.0f, kHighLsb, hg);
    EXPECT_EQ(hg[0], 32767);
    EXPECT_EQ(hg[1], -32768);
    EXPECT_EQ(hg[2], 0);
}
