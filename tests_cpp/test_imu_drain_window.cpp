// Drain-window statistics (#1191): the flight loop pops every queued IMU
// sample each pass; imu_drain_window.h turns the pass's samples into the one
// sample the EKF, the roll controller and the kinematics consume.  It used to
// be the freshest sample — a 1-in-N decimation with no anti-alias step — and
// is now the window mean, with the low-g near-rail switch fed from the
// window's worst raw sample instead.
#include <gtest/gtest.h>
#include "imu_drain_window.h"
#include <cmath>
#include <vector>

namespace
{
constexpr int32_t kNearRail16g = imu_drain::nearRailLsb(16.0f, 0.5f);   // 31744

ISM6HG256Data sample(uint32_t t_us,
                     int16_t lx, int16_t ly, int16_t lz,
                     int16_t hx, int16_t hy, int16_t hz,
                     int16_t gx, int16_t gy, int16_t gz)
{
    ISM6HG256Data s{};
    s.time_us = t_us;
    s.acc_low_raw  = {lx, ly, lz};
    s.acc_high_raw = {hx, hy, hz};
    s.gyro_raw     = {gx, gy, gz};
    return s;
}

// A sample with the same value on every axis of every channel.
ISM6HG256Data flat(uint32_t t_us, int16_t v)
{
    return sample(t_us, v, v, v, v, v, v, v, v, v);
}

// Run a raw stream through the firmware's drain the way loop_fc does: one
// window per tick, `sizes[k]` samples in tick k.  Returns the value the old
// path consumed (the last sample of each window) and the new one (the mean),
// both on low-g X.
struct Decimated { std::vector<double> freshest, mean; };

Decimated decimate(const std::vector<int16_t>& x, const std::vector<int>& sizes)
{
    Decimated d;
    size_t i = 0;
    for (int n : sizes)
    {
        ImuDrainWindow w;
        int16_t last = 0;
        for (int k = 0; k < n && i < x.size(); ++k, ++i)
        {
            last = x[i];
            w.add(sample((uint32_t)i, x[i], 0, 0, 0, 0, 0, 0, 0, 0));
        }
        ISM6HG256Data m{};
        if (!w.mean(m)) break;
        d.freshest.push_back(last);
        d.mean.push_back(m.acc_low_raw.x);
    }
    return d;
}

double rms(const std::vector<double>& v)
{
    double s = 0.0;
    for (double x : v) s += x * x;
    return std::sqrt(s / (double)v.size());
}

std::vector<int16_t> tone(double f_hz, double fs_hz, double amp, size_t n)
{
    std::vector<int16_t> x(n);
    for (size_t i = 0; i < n; ++i)
        x[i] = (int16_t)std::lround(amp * std::sin(2.0 * M_PI * f_hz * (double)i / fs_hz));
    return x;
}
} // namespace

// ---------- nothing drained ----------

TEST(ImuDrainWindow, EmptyWindowIsRefusedAndLeavesOutputAlone)
{
    ImuDrainWindow w;
    ISM6HG256Data out = flat(77, 5);
    EXPECT_FALSE(w.mean(out));
    EXPECT_EQ(out.time_us, 77u);
    EXPECT_EQ(out.acc_low_raw.x, 5);
    EXPECT_FALSE(w.lowGNearRail(kNearRail16g));
}

TEST(ImuDrainWindow, ResetForgetsEverything)
{
    ImuDrainWindow w;
    w.add(flat(10, 32000));
    w.reset();
    ISM6HG256Data out{};
    EXPECT_FALSE(w.mean(out));
    EXPECT_EQ(w.n, 0u);
    EXPECT_EQ(w.max_lg[0], 0);
}

// ---------- the mean ----------

TEST(ImuDrainWindow, SingleSampleComesBackUnchanged)
{
    ImuDrainWindow w;
    w.add(sample(1000, 1, -2, 3, 40, -50, 60, -700, 800, -900));
    ISM6HG256Data out{};
    ASSERT_TRUE(w.mean(out));
    EXPECT_EQ(out.time_us, 1000u);
    EXPECT_EQ(out.acc_low_raw.x, 1);   EXPECT_EQ(out.acc_low_raw.y, -2);  EXPECT_EQ(out.acc_low_raw.z, 3);
    EXPECT_EQ(out.acc_high_raw.x, 40); EXPECT_EQ(out.acc_high_raw.y, -50); EXPECT_EQ(out.acc_high_raw.z, 60);
    EXPECT_EQ(out.gyro_raw.x, -700);   EXPECT_EQ(out.gyro_raw.y, 800);    EXPECT_EQ(out.gyro_raw.z, -900);
}

TEST(ImuDrainWindow, KnownSamplesInMeanOut)
{
    // Eight samples, each channel/axis its own ramp so a mixed-up axis or
    // channel shows as a wrong number rather than a coincidence.
    ImuDrainWindow w;
    for (int k = 0; k < 8; ++k)
    {
        w.add(sample(1000 + 260 * k,
                     (int16_t)(100 + k), (int16_t)(-200 - 2 * k), (int16_t)(300 + 3 * k),
                     (int16_t)(1000 + 10 * k), (int16_t)(-2000 - 20 * k), (int16_t)(3000 + 30 * k),
                     (int16_t)(-40 * k), (int16_t)(50 * k), (int16_t)(-60 * k)));
    }
    ASSERT_EQ(w.n, 8u);
    ISM6HG256Data out{};
    ASSERT_TRUE(w.mean(out));
    // Ramps of 0..7 average 3.5: half-away-from-zero rounding lands on 4
    // going up and -4 going down, times the step.
    EXPECT_EQ(out.acc_low_raw.x,  104);     // 103.5
    EXPECT_EQ(out.acc_low_raw.y,  -207);    // -207.0
    EXPECT_EQ(out.acc_low_raw.z,  311);     // 310.5
    EXPECT_EQ(out.acc_high_raw.x, 1035);    // 1035.0
    EXPECT_EQ(out.acc_high_raw.y, -2070);   // -2070.0
    EXPECT_EQ(out.acc_high_raw.z, 3105);    // 3105.0
    EXPECT_EQ(out.gyro_raw.x,     -140);    // -140.0
    EXPECT_EQ(out.gyro_raw.y,     175);     // 175.0
    EXPECT_EQ(out.gyro_raw.z,     -210);    // -210.0
}

TEST(ImuDrainWindow, RoundsHalfAwayFromZero)
{
    EXPECT_EQ(imu_drain::roundedMean(10, 3), 3);     // 3.33
    EXPECT_EQ(imu_drain::roundedMean(11, 3), 4);     // 3.67
    EXPECT_EQ(imu_drain::roundedMean(-10, 3), -3);
    EXPECT_EQ(imu_drain::roundedMean(-11, 3), -4);
    EXPECT_EQ(imu_drain::roundedMean(7, 2), 4);      // 3.5
    EXPECT_EQ(imu_drain::roundedMean(-7, 2), -4);
    EXPECT_EQ(imu_drain::roundedMean(0, 5), 0);
    EXPECT_EQ(imu_drain::roundedMean(32767 * 4, 4), 32767);
    EXPECT_EQ(imu_drain::roundedMean(-32768 * 4, 4), -32768);
}

TEST(ImuDrainWindow, SumsHoldAFullQueueAtRail)
{
    // ISM6_QUEUE_DEPTH is 256: a stalled loop drains that many in one pass.
    ImuDrainWindow hi, lo;
    for (int k = 0; k < 256; ++k)
    {
        hi.add(flat((uint32_t)k, 32767));
        lo.add(flat((uint32_t)k, -32768));
    }
    ISM6HG256Data out{};
    ASSERT_TRUE(hi.mean(out));
    EXPECT_EQ(out.acc_low_raw.x, 32767);
    EXPECT_EQ(out.gyro_raw.z, 32767);
    ASSERT_TRUE(lo.mean(out));
    EXPECT_EQ(out.acc_high_raw.y, -32768);
    EXPECT_EQ(lo.max_lg[0], 32768);   // |INT16_MIN| does not wrap
}

// ---------- the stamp ----------

TEST(ImuDrainWindow, StampIsTheCentreOfTheWindow)
{
    ImuDrainWindow w;
    for (int k = 0; k < 8; ++k) w.add(flat(1000 + 260 * k, 0));
    ISM6HG256Data out{};
    ASSERT_TRUE(w.mean(out));
    EXPECT_EQ(out.time_us, 1000u + (7u * 260u) / 2u);   // 1910
}

TEST(ImuDrainWindow, StampSurvivesTheMicrosWrap)
{
    ImuDrainWindow w;
    w.add(flat(0xFFFFFF00u, 0));
    w.add(flat(0x00000100u, 0));
    ISM6HG256Data out{};
    ASSERT_TRUE(w.mean(out));
    EXPECT_EQ(out.time_us, 0u);   // 0xFFFFFF00 + 0x200/2
}

// ---------- max |raw| ----------

TEST(ImuDrainWindow, MaxAbsIsPerAxisAndPerChannel)
{
    ImuDrainWindow w;
    w.add(sample(0,  100, -900,  5,   7,  8, -9,  1,  2,  3));
    w.add(sample(1, -300,  200, -5,  70, -8,  9, -1, 20, -3));
    w.add(sample(2,   50,   10,  0,   7,  8, 90,  1,  2, 30));
    EXPECT_EQ(w.max_lg[0], 300); EXPECT_EQ(w.max_lg[1], 900); EXPECT_EQ(w.max_lg[2], 5);
    EXPECT_EQ(w.max_hg[0], 70);  EXPECT_EQ(w.max_hg[1], 8);   EXPECT_EQ(w.max_hg[2], 90);
    EXPECT_EQ(w.max_gy[0], 1);   EXPECT_EQ(w.max_gy[1], 20);  EXPECT_EQ(w.max_gy[2], 30);
}

// ---------- the near-rail verdict ----------

TEST(ImuDrainWindow, NearRailThresholdMatchesTheOldBodyFrameBar)
{
    // 15.5 g of a 16 g full scale, in LSB.
    EXPECT_EQ(kNearRail16g, 31744);
    EXPECT_EQ(imu_drain::nearRailLsb(8.0f, 0.5f), 30720);
}

TEST(ImuDrainWindow, NearRailFiresOnOneClippedSampleTheMeanWouldHide)
{
    // Seven quiet samples and one at the rail: the mean sits at ~5.7k LSB,
    // a third of the bar, but the switch to the high-g channel must fire.
    ImuDrainWindow w;
    for (int k = 0; k < 7; ++k) w.add(sample((uint32_t)k, 2000, 0, 0, 0, 0, 0, 0, 0, 0));
    w.add(sample(7, 32000, 0, 0, 0, 0, 0, 0, 0, 0));
    ISM6HG256Data out{};
    ASSERT_TRUE(w.mean(out));
    EXPECT_LT(out.acc_low_raw.x, kNearRail16g);
    EXPECT_TRUE(w.lowGNearRail(kNearRail16g));
}

TEST(ImuDrainWindow, NearRailSeesEveryAxisAndBothSigns)
{
    ImuDrainWindow x, y, z, none;
    x.add(sample(0, -31745, 0, 0, 0, 0, 0, 0, 0, 0));
    y.add(sample(0, 0, 31745, 0, 0, 0, 0, 0, 0, 0));
    z.add(sample(0, 0, 0, -32768, 0, 0, 0, 0, 0, 0));
    none.add(sample(0, 31744, -31744, 31744, 32767, 32767, 32767, 32767, 32767, 32767));
    EXPECT_TRUE(x.lowGNearRail(kNearRail16g));
    EXPECT_TRUE(y.lowGNearRail(kNearRail16g));
    EXPECT_TRUE(z.lowGNearRail(kNearRail16g));
    // Exactly at the bar is not above it, and the high-g / gyro channels are
    // not the low-g rail.
    EXPECT_FALSE(none.lowGNearRail(kNearRail16g));
}

TEST(ImuDrainWindow, NearRailIsJudgedInSensorAxes)
{
    // One sensor axis at the rail, the other at zero.  Through the -45 deg
    // mount this reads 11.3 g on BOTH body axes, under the old 15.5 g body-
    // frame bar; the raw per-axis test sees the rail regardless of mounting.
    ImuDrainWindow w;
    w.add(sample(0, 32767, 0, 0, 0, 0, 0, 0, 0, 0));
    const double body = 32767.0 * std::cos(M_PI / 4.0);   // 23170 LSB = 11.3 g
    EXPECT_LT(body, (double)kNearRail16g);
    EXPECT_TRUE(w.lowGNearRail(kNearRail16g));
}

// ---------- the anti-alias effect ----------

TEST(ImuDrainWindow, BoxcarKnocksTheBoostToneDown)
{
    // 800 Hz at the 3840 Hz boost ODR, eight samples per 480 Hz tick.  The
    // freshest-sample pick keeps the tone at full amplitude (folded to
    // 160 Hz); the mean is an 8-tap boxcar, |H(800)| = 0.178, -15 dB.
    const double fs = 3840.0, amp = 10000.0;
    const auto x = tone(800.0, fs, amp, 3840);
    const Decimated d = decimate(x, std::vector<int>(480, 8));
    ASSERT_EQ(d.mean.size(), 480u);
    const double r_fresh = rms(d.freshest), r_mean = rms(d.mean);
    EXPECT_NEAR(r_fresh, amp / std::sqrt(2.0), 0.02 * amp);   // still the whole tone
    EXPECT_LT(r_mean / r_fresh, 0.20);                        // >= 14 dB down
}

TEST(ImuDrainWindow, VariableWindowsStillAttenuateTheTone)
{
    // The loop is not phase-locked to the ODR: ~7.6 samples per tick on the
    // 2026-08-29 log.  Alternate 7 and 8; the nulls move, the tone still
    // loses better than 12 dB.
    const double fs = 3840.0, amp = 10000.0;
    const auto x = tone(800.0, fs, amp, 3840 * 2);
    std::vector<int> sizes;
    for (int k = 0; k < 1000; ++k) sizes.push_back((k & 1) ? 8 : 7);
    const Decimated d = decimate(x, sizes);
    ASSERT_GT(d.mean.size(), 900u);
    EXPECT_LT(rms(d.mean) / rms(d.freshest), 0.25);
}

TEST(ImuDrainWindow, BoxcarPassesAirframeBendingUntouched)
{
    // 30 Hz at 3840 Hz through the same 8-tap boxcar: |H(30)| = 0.9936.  One
    // second is 30 whole cycles at 16 ticks per cycle, so the RMS ratio is
    // the filter gain and nothing else.
    const double fs = 3840.0, amp = 10000.0;
    const auto x = tone(30.0, fs, amp, 3840);
    const Decimated d = decimate(x, std::vector<int>(480, 8));
    const double ratio = rms(d.mean) / rms(d.freshest);
    EXPECT_GT(ratio, 0.99);
    EXPECT_LE(ratio, 1.0 + 1e-9);
}
