// test_sim_replay_epochs.cpp — the rig replay's clock from the mosaic-G5's
// timed raw epochs, and the carried flight at the V9's IMU rate (#1600 on the
// V9 mosaic image).
//
// The mosaic-G5's driver hands no timed fix (getGNSSFix is the LC86G's), so a
// replay build of the mosaic-only image sets its clock from the raw epochs the
// driver tags on the 1PPS line: ten a second, each with its GPS week, the
// receiver's time of week and that instant on the MCU clock
// (TR_Sensor_Collector_Sim's getGNSSRawEpoch, ReplayClock::onEpoch). These
// pin that the epochs arm and freeze the clock as the LC86G's 1 Hz fixes do,
// and that the V9's 3840 Hz IMU, reading the carried 500 Hz flight at each of
// its own instants, integrates to the flight's own velocity.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "SimReplay.h"

using sim_replay::Header;
using sim_replay::ReplayClock;
using sim_replay::Sample;
using sim_replay::SimReplay;

namespace
{

// The rig file's ignition: GPS week 2433, an hour into its Saturday
constexpr uint16_t kWeek = 2433;
constexpr double kIgnitionTow = 573786.0;
constexpr double kIgnitionGps = kWeek * 604800.0 + kIgnitionTow;

Header rigHeader(double ignition_gps = kIgnitionGps)
{
    Header h;
    h.ignition_gps_s = ignition_gps;
    h.scen_start_s = -180.0;   // the file's 180 s pad (p180)
    h.scen_end_s = 76.7;
    return h;
}

// The receiver's epoch at replay time t, its instant on the MCU at mcu_s
void epochAt(ReplayClock& c, double t, double mcu_s, double ignition_gps = kIgnitionGps)
{
    const double gps = ignition_gps + t;
    const double week = std::floor(gps / 604800.0);
    c.onEpoch(static_cast<uint16_t>(week), gps - week * 604800.0,
              static_cast<int64_t>(std::llround(mcu_s * 1e6)));
}

}  // namespace

// ---- the clock from the epochs

TEST(ReplayClockEpochs, TenASecondFollowThePadAndFreezeBeforeIgnition)
{
    ReplayClock c;
    c.reset(rigHeader());
    // the first timed epoch at t = -140 s (file second 40), the MCU 55 s up
    const double mcu0 = 55.0;
    uint32_t n = 0;
    double t = -140.0;
    for (; t < -5.05; t += 0.1, ++n)
    {
        epochAt(c, t, mcu0 + (t + 140.0));
        ASSERT_EQ(c.state(), ReplayClock::kFollowing) << "t " << t;
    }
    EXPECT_EQ(c.restarts(), 0u);
    double tr = 0.0;
    ASSERT_TRUE(c.tAt(static_cast<int64_t>((mcu0 + 100.0) * 1e6), tr));
    EXPECT_NEAR(tr, -40.0, 1e-6);
    // the first epoch at or after arm_by (-5 s) freezes it
    epochAt(c, -5.0, mcu0 + 135.0);
    ++n;
    EXPECT_EQ(c.state(), ReplayClock::kFrozen);
    EXPECT_EQ(c.fixes(), n);   // each epoch counts where the console says fixes=
    // from here the MCU's crystal: epochs 2 ms off it move nothing
    epochAt(c, 20.0, mcu0 + 160.002);
    ASSERT_TRUE(c.tAt(static_cast<int64_t>((mcu0 + 160.0) * 1e6), tr));
    EXPECT_NEAR(tr, 20.0, 1e-6);
    EXPECT_NEAR(c.lastFixT(), 20.0, 1e-6);
}

TEST(ReplayClockEpochs, AnEpochWithoutItsTimeIsNotCounted)
{
    ReplayClock c;
    c.reset(rigHeader());
    // the driver hands epochs before the 1PPS line is valid (mcu_us 0) and,
    // after a boot, before any names its week (week 0)
    c.onEpoch(kWeek, kIgnitionTow - 150.0, 0);
    c.onEpoch(0, kIgnitionTow - 150.0, 40000000);
    // and nothing outside a week is a time of week
    c.onEpoch(kWeek, -0.1, 40000000);
    c.onEpoch(kWeek, 604800.0, 40000000);
    c.onEpoch(kWeek, std::nan(""), 40000000);
    EXPECT_EQ(c.state(), ReplayClock::kWaiting);
    EXPECT_EQ(c.fixes(), 0u);
    EXPECT_EQ(c.outside(), 0u);
    double t = 0.0;
    EXPECT_FALSE(c.tAt(41000000, t));
}

TEST(ReplayClockEpochs, TheRealSkysEpochsAreNotTheRigs)
{
    ReplayClock c;
    c.reset(rigHeader());
    // a receiver that tracked the real sky before the rig's (49 days on: the
    // V9's October against the rig's August), or a stale clock
    c.onEpoch(kWeek + 7, 100000.0, 30000000);
    c.onEpoch(kWeek, kIgnitionTow - 400.0, 30100000);
    c.onEpoch(kWeek, kIgnitionTow + 200.0, 30200000);
    EXPECT_EQ(c.state(), ReplayClock::kWaiting);
    EXPECT_EQ(c.outside(), 3u);
    EXPECT_EQ(c.fixes(), 0u);
    // the rig's own then starts it
    epochAt(c, -120.0, 70.0);
    EXPECT_EQ(c.state(), ReplayClock::kFollowing);
}

TEST(ReplayClockEpochs, TheFirstTimedEpochAfterTheArmingInstantNeverStartsTheFlight)
{
    ReplayClock c;
    c.reset(rigHeader());
    // the epochs stayed untimed all through the pad (no 1PPS pairing): the
    // first timed one comes at -4.9 s -- the flight would begin mid-boost
    c.onEpoch(kWeek, kIgnitionTow - 30.0, 0);
    epochAt(c, -4.9, 200.0);
    EXPECT_EQ(c.state(), ReplayClock::kMissed);
    for (double t = -4.8; t < 10.0; t += 0.1)
        epochAt(c, t, 200.0 + (t + 4.9));
    EXPECT_EQ(c.state(), ReplayClock::kMissed);
    double tr = 0.0;
    EXPECT_FALSE(c.tAt(210000000, tr));   // the pad, looped, for good
}

TEST(ReplayClockEpochs, AGapAcrossTheArmingInstantKeepsTheClockGoing)
{
    ReplayClock c;
    c.reset(rigHeader());
    for (double t = -60.0; t < -20.0; t += 0.1)
        epochAt(c, t, 100.0 + (t + 60.0));
    ASSERT_EQ(c.state(), ReplayClock::kFollowing);
    // no timed epoch from -20 s to +3 s (the line lost its pairing, say): the
    // flight starts on the last base all the same, on the MCU's crystal
    double tr = 0.0;
    ASSERT_TRUE(c.tAt(static_cast<int64_t>((100.0 + 60.0) * 1e6), tr));
    EXPECT_NEAR(tr, 0.0, 1e-6);
    // and the first epoch after it freezes the clock where it stands
    epochAt(c, 3.0, 100.0 + 63.0 + 0.0004);
    EXPECT_EQ(c.state(), ReplayClock::kFrozen);
    EXPECT_EQ(c.restarts(), 0u);
    ASSERT_TRUE(c.tAt(static_cast<int64_t>((100.0 + 63.0) * 1e6), tr));
    EXPECT_NEAR(tr, 3.0, 1e-3);
}

TEST(ReplayClockEpochs, AFlightAcrossTheWeeksEndKeepsOneClock)
{
    // ignition 30 s before the end of a GPS week: the pad's epochs in one
    // week, the flight's in the next
    const double ign = (kWeek + 1) * 604800.0 - 30.0;
    ReplayClock c;
    c.reset(rigHeader(ign));
    epochAt(c, -60.0, 10.0, ign);
    epochAt(c, -10.0, 60.0, ign);
    EXPECT_EQ(c.state(), ReplayClock::kFollowing);
    epochAt(c, 40.0, 110.0, ign);   // week kWeek + 1, time of week 10 s
    EXPECT_EQ(c.state(), ReplayClock::kFrozen);
    EXPECT_EQ(c.outside(), 0u);
    EXPECT_EQ(c.restarts(), 0u);
    double tr = 0.0;
    ASSERT_TRUE(c.tAt(110000000, tr));
    EXPECT_NEAR(tr, 40.0, 1e-6);
}

// ---- the carried flight at the V9's IMU rate

namespace
{

template <typename T>
void put(std::vector<uint8_t>& b, size_t off, T v)
{
    std::memcpy(b.data() + off, &v, sizeof(T));
}

// A 500 Hz flight in the writer's layout (tools/sim_replay_blob.py): 2 s of
// pad, a 1.6 s burn shaped like a G motor's (a 15 g spike over 40 ms, then
// 9 g tapering), a coast, and the ground; specific force along the nose
// (FLU x), the attitude still. f_true holds each sample's value in m/s^2.
std::vector<uint8_t> burnBlob(std::vector<double>& f_true, double& acc_lsb)
{
    const uint16_t rate = 500;
    const uint32_t n = 3500;   // t = -2 .. +5 s
    const double t0 = -2.0;
    acc_lsb = 0.01;
    std::vector<uint8_t> b(SimReplay::kHeaderBytes + n * SimReplay::kSampleBytes, 0);
    std::memcpy(b.data(), "TRRP", 4);
    put<uint16_t>(b, 4, 1);
    put<uint16_t>(b, 6, 128);
    put<uint16_t>(b, 8, 24);
    put<uint16_t>(b, 10, rate);
    put<uint32_t>(b, 12, n);
    put<double>(b, 16, t0);
    put<double>(b, 24, kIgnitionGps);
    put<double>(b, 32, 0.0);   // the pad: no place of anyone's
    put<double>(b, 40, 0.0);
    put<double>(b, 48, 0.0);
    put<double>(b, 56, -180.0);
    put<double>(b, 64, 76.7);
    put<float>(b, 72, static_cast<float>(acc_lsb));
    put<float>(b, 76, 0.001f);
    put<float>(b, 80, 100000.0f);
    put<float>(b, 84, 0.25f);
    put<float>(b, 88, 1.0f);
    put<float>(b, 92, 0.5f);
    put<float>(b, 96, 2026.6575f);
    std::memcpy(b.data() + 100, "burn", 4);
    f_true.assign(n, 0.0);
    for (uint32_t i = 0; i < n; ++i)
    {
        const double t = t0 + i / static_cast<double>(rate);
        double f = 9.80665;   // on the pad and on the ground: 1 g along the nose
        uint8_t phase = 0;
        if (t >= 0.0 && t < 1.6)
        {
            f = (t < 0.04 ? 15.0 : 9.0 - 3.0 * (t - 0.04)) * 9.80665;
            phase = 2;
        }
        else if (t >= 1.6 && t < 4.4)
        {
            f = -0.3 * 9.80665;   // the drag, decelerating it
            phase = 3;
        }
        else if (t >= 4.4)
            phase = 6;
        const int16_t counts = static_cast<int16_t>(std::lround(f / acc_lsb));
        f_true[i] = counts * acc_lsb;
        const size_t o = SimReplay::kHeaderBytes + i * SimReplay::kSampleBytes;
        put<int16_t>(b, o + 0, counts);
        put<int16_t>(b, o + 12, 32767);   // q = 1: still
        b[o + 22] = phase;
    }
    return b;
}

}  // namespace

TEST(SimReplayAtImuRate, TheV9sImuIntegratesTheCarriedFlightToItsOwnVelocity)
{
    std::vector<double> f_true;
    double lsb = 0.0;
    const std::vector<uint8_t> b = burnBlob(f_true, lsb);
    SimReplay r;
    ASSERT_TRUE(r.load(b.data(), b.size())) << r.why();
    // The flight's own velocity change over [ta, tb]: each 500 Hz pair mean
    // held over its own 2 ms, as the CSV's 1 kHz integrates
    auto carried_dv = [&](double ta, double tb) {
        double v = 0.0;
        for (size_t i = 0; i < f_true.size(); ++i)
        {
            const double c = -2.0 + i / 500.0;
            const double lo = std::max(ta, c - 0.001), hi = std::min(tb, c + 0.001);
            if (hi > lo) v += (f_true[i] - 9.80665) * (hi - lo);
        }
        return v;
    };
    // The V9's ISM6 FIFO runs at 3780 Hz on this part (3840 nominal; 7680 on
    // a reduced GNSS log), each sample read at its own instant
    // (replaySampleAt), the filter integrating each over its own interval.
    // Windows from the pad to a flat stretch: the middle of the spike, the
    // taper, the coast, so every edge the interpolation spreads is inside.
    for (const double rate_hz : {3780.0, 3840.0, 7680.0})
    {
        for (const double tb : {0.02, 0.5, 2.5, 4.0})
        {
            const double ta = -0.5;
            const double dt = 1.0 / rate_hz;
            double v = 0.0;
            Sample s;
            int k = 0;
            for (; ta + (k + 0.5) * dt < tb; ++k)
            {
                r.at(ta + (k + 0.5) * dt, s);
                v += (s.f[0] - 9.80665) * dt;
            }
            v -= (s.f[0] - 9.80665) * (k * dt - (tb - ta));   // the last step's part past tb (flat there)
            EXPECT_NEAR(v, carried_dv(ta, tb), 0.005) << rate_hz << " Hz to t " << tb;
        }
    }
}
