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

TEST(ReplayClockEpochs, TheClockIsTheTagsGpsInstantNotTheReceiversTimeOfWeek)
{
    // The mosaic-G5's epochs carry the receiver's own time of week (its clock
    // within the 0.5 ms jump threshold of GPS); the driver's tag places the
    // GPS instant TOW - dtr on the MCU (TimedRawEpoch::tag_dtr_ms). The clock
    // must take that instant, or the replayed IMU sits dtr off the sky's GPS
    // time: 0.4 ms is 0.07 m/s at 18 g.
    const double dtr_s = 0.4e-3;   // receiver minus GPS
    ReplayClock c;
    c.reset(rigHeader());
    for (double t = -140.0; t < -5.05; t += 0.1)
    {
        // the receiver's epoch on its own 100 ms grid at t; its GPS instant is
        // dtr earlier, and the MCU stands at 55 s + (that instant + 140 s)
        const double gps = kIgnitionGps + t;
        const double mcu_s = 55.0 + (t - dtr_s + 140.0);
        c.onEpoch(kWeek, gps - kWeek * 604800.0, static_cast<int64_t>(std::llround(mcu_s * 1e6)), dtr_s);
    }
    ASSERT_EQ(c.state(), ReplayClock::kFollowing);
    // the replay's time at an MCU instant is that instant's GPS time
    double tr = 0.0;
    ASSERT_TRUE(c.tAt(static_cast<int64_t>((55.0 + 100.0) * 1e6), tr));
    EXPECT_NEAR(tr, -40.0, 2e-6);
    // without the bias it came out dtr late on the sky: the old pairing
    ReplayClock old;
    old.reset(rigHeader());
    for (double t = -140.0; t < -5.05; t += 0.1)
    {
        const double gps = kIgnitionGps + t;
        old.onEpoch(kWeek, gps - kWeek * 604800.0,
                    static_cast<int64_t>(std::llround((55.0 + (t - dtr_s + 140.0)) * 1e6)));
    }
    ASSERT_TRUE(old.tAt(static_cast<int64_t>((55.0 + 100.0) * 1e6), tr));
    EXPECT_NEAR(tr, -40.0 + dtr_s, 2e-6);
    // a bias that is not a number is no epoch's
    ReplayClock nan_c;
    nan_c.reset(rigHeader());
    nan_c.onEpoch(kWeek, kIgnitionTow - 100.0, 40000000, std::nan(""));
    EXPECT_EQ(nan_c.state(), ReplayClock::kWaiting);
    EXPECT_EQ(nan_c.fixes(), 0u);
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

// ---- the freeze against the 1PPS edge (ReplayClock::onTagCheck)
//
// The driver's tags come from its clock line, which follows a sky that came
// late (a transmitter underrun) only after about a minute; the 1PPS edges
// follow it at once, and its check says how far each whole second's tag sat
// from its edge. The 10/17 pad check on the rig: -6855 us for 27 s and more.

namespace
{

constexpr double kMcu0 = 55.0;   // the MCU at t = -140 s, running at GPS rate

// The MCU instant the line tags the GPS instant t with (it never moved)
double tagged(double t) { return kMcu0 + (t + 140.0); }

// Ten epochs a second from `from` to before `to`, as the line tags them; the
// sky slipped `late_s` late from `slip_t` on (the edges with it), and each
// whole second's 1PPS check follows its epoch with the residual the driver
// would find: the tag less the edge
void feed(ReplayClock& c, double from, double to, double late_s = 0.0, double slip_t = 1e9, bool checks = true)
{
    for (int k = 0; from + 0.1 * k < to - 1e-9; ++k)
    {
        const double t = from + 0.1 * k;
        epochAt(c, t, tagged(t));
        const double whole = std::round(t);
        if (checks && std::fabs(t - whole) < 1e-6)
        {
            const double edge_late = t >= slip_t ? late_s : 0.0;
            c.onTagCheck(kIgnitionGps + whole, static_cast<int32_t>(std::lround(-edge_late * 1e6)));
        }
    }
}

int64_t us(double s) { return static_cast<int64_t>(std::llround(s * 1e6)); }

}  // namespace

TEST(ReplayClockPpsCheck, AFreezeTheEdgeAgreesWithIsKept)
{
    ReplayClock c;
    c.reset(rigHeader());
    feed(c, -140.0, -1.0, 30e-6, -1e9);   // every tag 30 us early: the line's own noise
    ASSERT_EQ(c.state(), ReplayClock::kFrozen);
    EXPECT_EQ(c.check(), ReplayClock::kKept);
    EXPECT_NEAR(c.checkT(), -5.0, 1e-6);   // the freeze's own second
    EXPECT_EQ(c.checkResidualUs(), -30);
    EXPECT_NEAR(c.checkMoveUs(), -30.0, 1.0);
    double tr = 0.0;
    ASSERT_TRUE(c.tAt(us(tagged(3.0)), tr));
    EXPECT_NEAR(tr, 3.0, 1e-6);   // the tags' clock, unmoved
}

TEST(ReplayClockPpsCheck, AFreezeOnALineThatHasNotFollowedALateSkyMovesOntoTheEdge)
{
    // the sky slips 6.856 ms late at t = -40 s; the line still tags with the
    // old mapping at the freeze, so the tags' clock is the sky's less 6.9 ms
    const double late = 6.856e-3;
    ReplayClock c;
    c.reset(rigHeader());
    feed(c, -140.0, -5.05, late, -40.0);
    ASSERT_EQ(c.state(), ReplayClock::kFollowing);
    double tr = 0.0;
    ASSERT_TRUE(c.tAt(us(tagged(-10.0) + late), tr));
    EXPECT_NEAR(tr, -10.0 + late, 1e-6);   // following the tags: the IMU 6.9 ms ahead of the sky
    feed(c, -5.0, -1.0, late, -40.0);
    ASSERT_EQ(c.state(), ReplayClock::kFrozen);
    ASSERT_EQ(c.check(), ReplayClock::kMoved);
    EXPECT_NEAR(c.checkT(), -5.0, 1e-6);
    EXPECT_EQ(c.checkResidualUs(), -6856);
    EXPECT_NEAR(c.checkMoveUs(), -6856.0, 1.0);
    // the flight on the sky's time: the instant the sky shows t is t
    for (double t : {0.0, 2.5, 11.7, 60.0})
    {
        ASSERT_TRUE(c.tAt(us(tagged(t) + late), tr));
        EXPECT_NEAR(tr, t, 2e-6) << t;
    }
    // and a check after check_by_s never moves it again (the line catching up)
    feed(c, -1.0, 5.0, 0.0, -1e9);
    EXPECT_EQ(c.check(), ReplayClock::kMoved);
    ASSERT_TRUE(c.tAt(us(tagged(2.5) + late), tr));
    EXPECT_NEAR(tr, 2.5, 2e-6);
}

TEST(ReplayClockPpsCheck, ASlipInTheLastSecondBeforeTheFreezeIsCaughtByItsOwnSecond)
{
    // the slip at -5.5 s: the check of -6 s says nothing of it, the check of
    // the freeze's own second (-5 s) does
    const double late = 3.4e-3;
    ReplayClock c;
    c.reset(rigHeader());
    feed(c, -140.0, -1.0, late, -5.5);
    ASSERT_EQ(c.check(), ReplayClock::kMoved);
    EXPECT_NEAR(c.checkT(), -5.0, 1e-6);
    double tr = 0.0;
    ASSERT_TRUE(c.tAt(us(tagged(1.0) + late), tr));
    EXPECT_NEAR(tr, 1.0, 2e-6);
}

TEST(ReplayClockPpsCheck, NoCheckByTheDeadlineLeavesTheFreezeUnchecked)
{
    ReplayClock c;
    c.reset(rigHeader());
    feed(c, -140.0, -5.05);
    feed(c, -5.0, 3.0, 5e-3, -1e9, false);   // no PVT, no check, through the deadline
    EXPECT_EQ(c.check(), ReplayClock::kUnchecked);
    // a check that comes later moves nothing: the replayed rocket is flying
    feed(c, 3.0, 6.0, 5e-3, -1e9);
    EXPECT_EQ(c.check(), ReplayClock::kUnchecked);
    double tr = 0.0;
    ASSERT_TRUE(c.tAt(us(tagged(4.0)), tr));
    EXPECT_NEAR(tr, 4.0, 1e-6);
}

TEST(ReplayClockPpsCheck, OnlyACountedSecondIsPairedAndOnlyAfterTheFreeze)
{
    ReplayClock c;
    c.reset(rigHeader());
    // a check of a second no epoch gave (the real sky's, or not drained yet)
    c.onTagCheck(kIgnitionGps - 60.0, -7000);
    EXPECT_EQ(c.tagChecks(), 0u);
    // while following, a bad check moves nothing (the pad: the freeze decides)
    feed(c, -140.0, -20.0, 7e-3, -30.0);
    EXPECT_GT(c.tagChecks(), 100u);
    EXPECT_EQ(c.check(), ReplayClock::kNotChecked);
    double tr = 0.0;
    ASSERT_TRUE(c.tAt(us(tagged(-25.0)), tr));
    EXPECT_NEAR(tr, -25.0, 1e-6);
    // a check of a second two back is not kept (only the last two whole seconds)
    feed(c, -20.0, -17.0);
    const uint32_t n = c.tagChecks();
    c.onTagCheck(kIgnitionGps - 20.0, 0);
    EXPECT_EQ(c.tagChecks(), n);
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
