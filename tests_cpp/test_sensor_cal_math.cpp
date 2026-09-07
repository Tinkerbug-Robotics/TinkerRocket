// Pad-calibration arithmetic (#1110): the IMU poll task sums every ISM6
// sample it reads during the window; sensor_cal_math.h turns the sums into
// the gyro zero-rate offsets, the high-g cross-calibration bias and the
// gravity plausibility verdict.  The poll-task handshake around it needs a
// rocket on the bench; the math does not.
#include <gtest/gtest.h>
#include "sensor_cal_math.h"
#include <cmath>
#include <limits>

namespace
{
constexpr float kG = sensor_cal::kG;

// m/s² per LSB at the shipping full scales (config.h: 16 g low, 256 g high),
// mirroring SensorConverter::configureISM6HG256FullScale.
constexpr float kLgMs2PerLsb = (16.0f  * 1000.0f / 32768.0f) * 1e-3f * kG;
constexpr float kHgMs2PerLsb = (256.0f * 1000.0f / 32768.0f) * 1e-3f * kG;

// 1 g in raw counts on each channel at those full scales.
constexpr int16_t kLgOneG = 32768 / 16;    // 2048
constexpr int16_t kHgOneG = 32768 / 256;   // 128

// A window of n identical samples.
SensorCalSums windowOf(uint32_t n,
                       int16_t gx, int16_t gy, int16_t gz,
                       int16_t lx, int16_t ly, int16_t lz,
                       int16_t hx, int16_t hy, int16_t hz)
{
    SensorCalSums s;
    s.count = n;
    s.g[0]  = (int64_t)gx * n; s.g[1]  = (int64_t)gy * n; s.g[2]  = (int64_t)gz * n;
    s.lg[0] = (int64_t)lx * n; s.lg[1] = (int64_t)ly * n; s.lg[2] = (int64_t)lz * n;
    s.hg[0] = (int64_t)hx * n; s.hg[1] = (int64_t)hy * n; s.hg[2] = (int64_t)hz * n;
    return s;
}

bool computeDefault(const SensorCalSums& s, float rot_deg, SensorCalResult& r)
{
    return sensor_cal::compute(s, rot_deg, 16, 256, r);
}
} // namespace

// ---------- nothing to average ----------

TEST(SensorCalMath, EmptyWindowIsRefused)
{
    SensorCalSums s;   // count 0
    SensorCalResult r;
    EXPECT_FALSE(computeDefault(s, 0.0f, r));
}

// ---------- gyro zero-rate offset ----------

TEST(SensorCalMath, GyroOffsetIsTheMeanRawCount)
{
    const SensorCalSums s = windowOf(1000, 10, -20, 30, 0, 0, kLgOneG, 0, 0, kHgOneG);
    SensorCalResult r;
    ASSERT_TRUE(computeDefault(s, 0.0f, r));
    EXPECT_EQ(r.gyro_offset[0], 10);
    EXPECT_EQ(r.gyro_offset[1], -20);
    EXPECT_EQ(r.gyro_offset[2], 30);
}

TEST(SensorCalMath, GyroOffsetTruncatesTowardZeroLikeTheOldLoop)
{
    // The 1000-sample loop did (int16_t)(sum / count) — integer division,
    // toward zero on both signs.  Keep that exact behaviour.
    SensorCalSums s = windowOf(1000, 0, 0, 0, 0, 0, kLgOneG, 0, 0, kHgOneG);
    s.g[0] = 10500;    // mean 10.5  -> 10
    s.g[1] = -10500;   // mean -10.5 -> -10
    s.g[2] = 999;      // mean 0.999 -> 0
    SensorCalResult r;
    ASSERT_TRUE(computeDefault(s, 0.0f, r));
    EXPECT_EQ(r.gyro_offset[0], 10);
    EXPECT_EQ(r.gyro_offset[1], -10);
    EXPECT_EQ(r.gyro_offset[2], 0);
}

// ---------- accel cross-calibration ----------

TEST(SensorCalMath, OneGOnZGivesZeroBiasAndGravity)
{
    const SensorCalSums s = windowOf(19200, 0, 0, 0, 0, 0, kLgOneG, 0, 0, kHgOneG);
    SensorCalResult r;
    ASSERT_TRUE(computeDefault(s, 0.0f, r));
    EXPECT_NEAR(r.lg_body[2], kG, 1e-3f);
    EXPECT_NEAR(r.hg_body[2], kG, 1e-3f);
    EXPECT_NEAR(r.hg_bias[0], 0.0f, 1e-4f);
    EXPECT_NEAR(r.hg_bias[1], 0.0f, 1e-4f);
    EXPECT_NEAR(r.hg_bias[2], 0.0f, 1e-3f);
    EXPECT_NEAR(r.gravity_mag, kG, 1e-3f);
    EXPECT_TRUE(sensor_cal::gravityPlausible(r.gravity_mag));
}

TEST(SensorCalMath, HighGBiasIsHighMinusLowInBodyFrame)
{
    // High-g reads 13 LSB high on Z (~0.1 g at 256 g FS) and 4 LSB on X.
    const SensorCalSums s = windowOf(5000, 0, 0, 0,
                                     0, 0, kLgOneG,
                                     4, 0, (int16_t)(kHgOneG + 13));
    SensorCalResult r;
    ASSERT_TRUE(computeDefault(s, 0.0f, r));
    EXPECT_NEAR(r.hg_bias[0], 4.0f * kHgMs2PerLsb, 1e-3f);
    EXPECT_NEAR(r.hg_bias[1], 0.0f, 1e-4f);
    EXPECT_NEAR(r.hg_bias[2], 13.0f * kHgMs2PerLsb, 1e-3f);
    // Gravity comes from the low-g channel alone.
    EXPECT_NEAR(r.gravity_mag, kG, 1e-3f);
}

TEST(SensorCalMath, RotationZMapsSensorXOntoBodyY)
{
    // Gravity along sensor +X; a 90 deg mount rotation puts it on body +Y
    // (lg_by = x*sin + y*cos), the same formula the old loop applied.
    const SensorCalSums s = windowOf(1000, 0, 0, 0, kLgOneG, 0, 0, kHgOneG, 0, 0);
    SensorCalResult r;
    ASSERT_TRUE(computeDefault(s, 90.0f, r));
    EXPECT_NEAR(r.lg_body[0], 0.0f, 1e-3f);
    EXPECT_NEAR(r.lg_body[1], kG, 1e-3f);
    EXPECT_NEAR(r.lg_body[2], 0.0f, 1e-3f);
    // Both channels rotate together, so an aligned pair has no bias...
    EXPECT_NEAR(r.hg_bias[0], 0.0f, 1e-3f);
    EXPECT_NEAR(r.hg_bias[1], 0.0f, 1e-3f);
    // ...and the magnitude is rotation-invariant.
    EXPECT_NEAR(r.gravity_mag, kG, 1e-3f);
}

TEST(SensorCalMath, ConfiguredFullScaleSetsThePerLsbScale)
{
    // #572: the same raw counts at half the full scale are half the m/s².
    const SensorCalSums s = windowOf(1000, 0, 0, 0, 0, 0, kLgOneG, 0, 0, kHgOneG);
    SensorCalResult r16, r8;
    ASSERT_TRUE(sensor_cal::compute(s, 0.0f, 16, 256, r16));
    ASSERT_TRUE(sensor_cal::compute(s, 0.0f, 8, 128, r8));
    EXPECT_NEAR(r8.lg_body[2], 0.5f * r16.lg_body[2], 1e-4f);
    EXPECT_NEAR(r8.hg_body[2], 0.5f * r16.hg_body[2], 1e-4f);
    EXPECT_NEAR(r8.gravity_mag, 0.5f * kG, 1e-3f);
    // ...which the gate then refuses: a full-scale mismatch is one of the
    // failure modes it exists for.
    EXPECT_FALSE(sensor_cal::gravityPlausible(r8.gravity_mag));
}

// ---------- 64-bit sums ----------

TEST(SensorCalMath, TenSecondsAtMaxOdrDoesNotOverflow)
{
    // 7680 Hz x 10 s = 76,800 samples of full-scale counts: the per-axis sum
    // is ~2.5e9, past int32.  The mean must come back exact.
    const uint32_t n = 76800;
    const SensorCalSums s = windowOf(n, 32767, -32768, 32767,
                                     32767, 32767, 32767,
                                     32767, 32767, 32767);
    ASSERT_GT(s.g[0], (int64_t)std::numeric_limits<int32_t>::max());
    SensorCalResult r;
    ASSERT_TRUE(computeDefault(s, 0.0f, r));
    EXPECT_EQ(r.gyro_offset[0], 32767);
    EXPECT_EQ(r.gyro_offset[1], -32768);
    EXPECT_EQ(r.gyro_offset[2], 32767);
    EXPECT_NEAR(r.lg_body[2], 32767.0f * kLgMs2PerLsb, 1e-2f);
}

// ---------- gravity plausibility gate ----------

TEST(SensorCalMath, GravityBandAcceptsAStationaryPad)
{
    EXPECT_TRUE(sensor_cal::gravityPlausible(kG));
    EXPECT_TRUE(sensor_cal::gravityPlausible(0.97f * kG));   // sensitivity + offset tolerance
    EXPECT_TRUE(sensor_cal::gravityPlausible(1.03f * kG));
    EXPECT_TRUE(sensor_cal::gravityPlausible(sensor_cal::kGravityMinMs2));
    EXPECT_TRUE(sensor_cal::gravityPlausible(sensor_cal::kGravityMaxMs2));
}

TEST(SensorCalMath, GravityBandRejectsCorruptOrMovingResults)
{
    EXPECT_FALSE(sensor_cal::gravityPlausible(0.0f));
    EXPECT_FALSE(sensor_cal::gravityPlausible(sensor_cal::kGravityMinMs2 - 0.01f));
    EXPECT_FALSE(sensor_cal::gravityPlausible(sensor_cal::kGravityMaxMs2 + 0.01f));
    EXPECT_FALSE(sensor_cal::gravityPlausible(2.0f * kG));
    EXPECT_FALSE(sensor_cal::gravityPlausible(30.0f));
    EXPECT_FALSE(sensor_cal::gravityPlausible(-kG));
    EXPECT_FALSE(sensor_cal::gravityPlausible(std::numeric_limits<float>::quiet_NaN()));
    EXPECT_FALSE(sensor_cal::gravityPlausible(std::numeric_limits<float>::infinity()));
}

TEST(SensorCalMath, MisoContentionGarbageIsRejected)
{
    // Two slaves driving MISO: the bus reads as all-ones (every axis -1 LSB,
    // magnitude ~0) or as full-scale positives (16 g on every axis).  Both
    // compute fine and both must fail the gate — the old loop accepted them.
    const SensorCalSums ones = windowOf(1000, -1, -1, -1, -1, -1, -1, -1, -1, -1);
    SensorCalResult r;
    ASSERT_TRUE(computeDefault(ones, 0.0f, r));
    EXPECT_LT(r.gravity_mag, 0.1f);
    EXPECT_FALSE(sensor_cal::gravityPlausible(r.gravity_mag));

    const SensorCalSums rail = windowOf(1000, 0, 0, 0,
                                        32767, 32767, 32767,
                                        32767, 32767, 32767);
    ASSERT_TRUE(computeDefault(rail, 0.0f, r));
    EXPECT_GT(r.gravity_mag, 20.0f * kG);
    EXPECT_FALSE(sensor_cal::gravityPlausible(r.gravity_mag));
}
