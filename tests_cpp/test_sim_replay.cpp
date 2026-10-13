// test_sim_replay.cpp — the rig replay's carried flight and its clock (#1600).
//
// SimReplay reads the file tools/sim_replay_blob.py writes (the writer's own
// test, tests/unit/test_sim_replay_blob.py, keeps fixtures/sim_replay_tiny.bin
// in step with it); ReplayClock places the MCU's instants on the flight's
// timeline from the receiver's fixes, following them on the pad and freezing
// before ignition.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <iterator>
#include <vector>

#include "SimReplay.h"

using sim_replay::Header;
using sim_replay::ReplayClock;
using sim_replay::Sample;
using sim_replay::SimReplay;

namespace
{

constexpr double kIgnitionGps = 2433 * 604800.0 + 573786.0;

struct Spec
{
    uint16_t rate = 100;
    uint32_t n = 400;
    double   t0 = -2.0;
    double   scen0 = -180.0, scen1 = 76.7;
    float    acc_lsb = 0.01f, gyro_lsb = 0.001f, p_ref = 100000.0f, p_lsb = 0.25f;
    float    pad_loop = 1.0f, land_loop = 0.5f;
};

struct Raw
{
    int16_t f[3] = {}, w[3] = {}, q[4] = {32767, 0, 0, 0}, p = 0;
    uint8_t phase = 0, flags = 0;
};

template <typename T>
void put(std::vector<uint8_t>& b, size_t off, T v)
{
    std::memcpy(b.data() + off, &v, sizeof(T));
}

// A file in the writer's layout: t = t0 + i / rate; the first second is the
// pad and the last half second the ground unless fill says otherwise.
std::vector<uint8_t> makeBlob(const Spec& s, const std::function<void(uint32_t, double, Raw&)>& fill)
{
    std::vector<uint8_t> b(SimReplay::kHeaderBytes + s.n * SimReplay::kSampleBytes, 0);
    std::memcpy(b.data(), "TRRP", 4);
    put<uint16_t>(b, 4, 1);
    put<uint16_t>(b, 6, 128);
    put<uint16_t>(b, 8, 24);
    put<uint16_t>(b, 10, s.rate);
    put<uint32_t>(b, 12, s.n);
    put<double>(b, 16, s.t0);
    put<double>(b, 24, kIgnitionGps);
    put<double>(b, 32, 39.4683565);
    put<double>(b, 40, -75.29286495);
    put<double>(b, 48, -0.712);
    put<double>(b, 56, s.scen0);
    put<double>(b, 64, s.scen1);
    put<float>(b, 72, s.acc_lsb);
    put<float>(b, 76, s.gyro_lsb);
    put<float>(b, 80, s.p_ref);
    put<float>(b, 84, s.p_lsb);
    put<float>(b, 88, s.pad_loop);
    put<float>(b, 92, s.land_loop);
    put<float>(b, 96, 2026.6575f);
    std::memcpy(b.data() + 100, "test", 4);
    const double t_last = s.t0 + (s.n - 1) / static_cast<double>(s.rate);
    for (uint32_t i = 0; i < s.n; ++i)
    {
        const double t = s.t0 + i / static_cast<double>(s.rate);
        Raw r;
        r.phase = t <= s.t0 + s.pad_loop + 1e-9 ? 0 : (t >= t_last - s.land_loop - 1e-9 ? 6 : 3);
        if (fill)
            fill(i, t, r);
        const size_t o = SimReplay::kHeaderBytes + i * SimReplay::kSampleBytes;
        for (int k = 0; k < 3; ++k)
        {
            put<int16_t>(b, o + 2 * k, r.f[k]);
            put<int16_t>(b, o + 6 + 2 * k, r.w[k]);
        }
        for (int k = 0; k < 4; ++k)
            put<int16_t>(b, o + 12 + 2 * k, r.q[k]);
        put<int16_t>(b, o + 20, r.p);
        b[o + 22] = r.phase;
        b[o + 23] = r.flags;
    }
    return b;
}

// a ramp in every channel: f[0] counts = i, so f[0] = i * acc_lsb
std::vector<uint8_t> rampBlob(const Spec& s = Spec())
{
    return makeBlob(s, [](uint32_t i, double, Raw& r) {
        r.f[0] = static_cast<int16_t>(i);
        r.f[1] = -static_cast<int16_t>(2 * i);
        r.f[2] = 1000;
        r.w[2] = static_cast<int16_t>(3 * i);
        r.p = static_cast<int16_t>(-4 * static_cast<int>(i));
    });
}

}  // namespace

// ---- the carried flight

TEST(SimReplay, ReadsTheWritersFile)
{
    // tests/unit/test_sim_replay_blob.py's made-up flight, written by the
    // writer: 100 Hz in pairs (50 Hz), from t = -2 s, FRD turned to FLU
    std::ifstream in(TR_SIM_REPLAY_FIXTURE, std::ios::binary);
    ASSERT_TRUE(in.good()) << TR_SIM_REPLAY_FIXTURE;
    const std::vector<uint8_t> blob((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    SimReplay r;
    ASSERT_TRUE(r.load(blob.data(), blob.size())) << r.why();
    const Header& h = r.header();
    EXPECT_EQ(h.rate_hz, 50);
    EXPECT_EQ(h.samples, 200u);
    EXPECT_NEAR(h.t0_s, -1.995, 1e-9);
    EXPECT_EQ(h.ignition_gps_s, 2433 * 604800.0 + 573609.0);   // the file's first instant + its 3 s prologue
    EXPECT_EQ(h.scen_start_s, -3.0);
    EXPECT_EQ(h.scen_end_s, 2.0);
    EXPECT_STREQ(h.name, "tiny");
    EXPECT_FLOAT_EQ(h.pad_loop_s, 1.0f);
    EXPECT_FLOAT_EQ(h.land_loop_s, 0.5f);
    EXPECT_NEAR(r.tLast(), 1.985, 1e-9);

    Sample s;
    r.raw(0, s);   // the mean of t = -2.00 and -1.99: the CSV's (10 + t, 0.5, -1 + t/4) FRD, (0.1 t, 0.2, -0.3)
    const double t = -1.995;
    EXPECT_NEAR(s.f[0], 10.0 + t, h.acc_lsb);
    EXPECT_NEAR(s.f[1], -0.5, h.acc_lsb);
    EXPECT_NEAR(s.f[2], 1.0 - 0.25 * t, h.acc_lsb);
    EXPECT_NEAR(s.w[0], 0.1 * t, h.gyro_lsb);
    EXPECT_NEAR(s.w[1], -0.2, h.gyro_lsb);
    EXPECT_NEAR(s.w[2], 0.3, h.gyro_lsb);
    EXPECT_NEAR(s.p_pa, 100000.0, h.p_lsb_pa);
    EXPECT_EQ(s.phase, 0);
    // the attitude: the nose (FLU +x) points up the rail, 83.6 deg up at 44.7 deg
    const float* q = s.q;
    const double nose_n = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
    const double nose_e = 2 * (q[1] * q[2] + q[0] * q[3]);
    const double nose_d = 2 * (q[1] * q[3] - q[0] * q[2]);
    EXPECT_NEAR(std::asin(-nose_d) * 180.0 / M_PI, 83.56, 0.05);
    EXPECT_NEAR(std::atan2(nose_e, nose_n) * 180.0 / M_PI, 44.69, 0.1);
    // and the rocket's z, square to the nose, lies within 6.4 deg of level
    const double z_d = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    EXPECT_LT(std::fabs(z_d), std::sin(6.5 * M_PI / 180.0));

    r.raw(127, s);   // t = 0.545: the high-g channel (0.50 .. 0.59 s), boost
    EXPECT_TRUE(s.high_g);
    EXPECT_EQ(s.phase, 2);
    r.raw(180, s);   // t = 1.605: 80 m up, the port reading 2 m low
    EXPECT_EQ(s.phase, 6);
    EXPECT_LT(s.p_pa, 100000.0f - 900.0f);
}

TEST(SimReplay, RefusesWhatIsNotOne)
{
    SimReplay r;
    std::vector<uint8_t> b = rampBlob();
    EXPECT_FALSE(r.load(b.data(), 100));
    EXPECT_FALSE(r.loaded());
    EXPECT_FALSE(r.load(b.data(), b.size() - 1));   // a sample short
    std::vector<uint8_t> bad = b;
    bad[0] = 'X';
    EXPECT_FALSE(r.load(bad.data(), bad.size()));
    bad = b;
    put<uint16_t>(bad, 4, 2);   // another version
    EXPECT_FALSE(r.load(bad.data(), bad.size()));
    bad = b;
    put<float>(bad, 72, 0.0f);   // no accelerometer scale
    EXPECT_FALSE(r.load(bad.data(), bad.size()));
    bad = b;
    put<float>(bad, 88, 3.0f);   // loops longer than the flight
    EXPECT_FALSE(r.load(bad.data(), bad.size()));
    // a pad loop that is not the pad would jump on the pad
    Spec s;
    const std::vector<uint8_t> moving = makeBlob(s, [](uint32_t i, double, Raw& r) {
        if (i == 50) r.phase = 2;
    });
    EXPECT_FALSE(r.load(moving.data(), moving.size()));
    EXPECT_STREQ(r.why(), "its pad loop is not all pad");
    const std::vector<uint8_t> unlanded = makeBlob(s, [](uint32_t i, double, Raw& r) {
        if (i == 399) r.phase = 5;
    });
    EXPECT_FALSE(r.load(unlanded.data(), unlanded.size()));
    EXPECT_TRUE(r.load(b.data(), b.size())) << r.why();
}

TEST(SimReplay, InterpolatesBetweenSamples)
{
    const std::vector<uint8_t> b = rampBlob();
    SimReplay r;
    ASSERT_TRUE(r.load(b.data(), b.size()));
    Sample s;
    // t = t0 + 123.25 samples
    r.at(-2.0 + 123.25 / 100.0, s);
    EXPECT_NEAR(s.f[0], 1.2325f, 1e-5f);
    EXPECT_NEAR(s.f[1], -2.465f, 1e-5f);
    EXPECT_NEAR(s.f[2], 10.0f, 1e-5f);
    EXPECT_NEAR(s.w[2], 0.36975f, 1e-6f);
    EXPECT_NEAR(s.p_pa, 100000.0f - 123.25f, 1e-2f);
    EXPECT_NEAR(s.q[0], 1.0f, 1e-6f);
    EXPECT_EQ(s.phase, 3);
    // exactly on a sample
    r.at(-2.0 + 200 / 100.0, s);
    EXPECT_NEAR(s.f[0], 2.0f, 1e-5f);
}

TEST(SimReplay, AttitudeIsNormalisedAndTakesTheShortWay)
{
    // q and -q are one attitude: a sign flip between samples must not blend
    // through zero
    Spec spec;
    const float c = std::cos(0.025f), sn = std::sin(0.025f);   // half of 0.05 rad
    const std::vector<uint8_t> b = makeBlob(spec, [&](uint32_t i, double, Raw& r) {
        const float sign = (i % 2 == 0) ? 1.0f : -1.0f;
        const float ang = (i % 2 == 0) ? 0.0f : 0.1f;
        const float qw = std::cos(ang / 2), qz = std::sin(ang / 2);
        r.q[0] = static_cast<int16_t>(std::lround(sign * qw * 32767));
        r.q[3] = static_cast<int16_t>(std::lround(sign * qz * 32767));
    });
    SimReplay r;
    ASSERT_TRUE(r.load(b.data(), b.size()));
    Sample s;
    r.at(-2.0 + 200.5 / 100.0, s);   // halfway from 0 to 0.1 rad about down: 0.05 rad
    const float n = std::sqrt(s.q[0] * s.q[0] + s.q[1] * s.q[1] + s.q[2] * s.q[2] + s.q[3] * s.q[3]);
    EXPECT_NEAR(n, 1.0f, 1e-5f);
    EXPECT_NEAR(std::fabs(s.q[0]), c, 1e-3f);
    EXPECT_NEAR(std::fabs(s.q[3]), sn, 1e-3f);
}

TEST(SimReplay, ThePadLoopsBeforeAndTheGroundAfter)
{
    const std::vector<uint8_t> b = rampBlob();
    SimReplay r;
    ASSERT_TRUE(r.load(b.data(), b.size()));
    Sample a, s;
    // before the first sample: the first second, in a loop
    r.at(-2.0 + 0.4, a);
    r.at(-2.0 + 0.4 - 1.0, s);
    EXPECT_NEAR(s.f[0], a.f[0], 1e-5f);
    r.at(-2.0 + 0.4 - 7.0, s);
    EXPECT_NEAR(s.f[0], a.f[0], 1e-5f);
    r.at(-150.0, s);
    EXPECT_EQ(s.phase, 0);
    EXPECT_LE(s.f[0], 100 * 0.01f + 1e-5f);   // within the first second's samples
    // the free-running pad, before the clock
    r.padAt(1234.4, s);
    EXPECT_NEAR(s.f[0], a.f[0], 1e-4f);
    r.padAt(-0.6, s);
    EXPECT_NEAR(s.f[0], a.f[0], 1e-4f);
    // after the last: the last half second, in a loop
    const double t_last = r.tLast();
    EXPECT_NEAR(t_last, 1.99, 1e-9);
    r.at(t_last - 0.5 + 0.2, a);
    r.at(t_last + 0.2, s);
    EXPECT_NEAR(s.f[0], a.f[0], 1e-4f);
    r.at(t_last + 100.2, s);
    EXPECT_NEAR(s.f[0], a.f[0], 1e-3f);
    EXPECT_EQ(s.phase, 6);
}

// ---- the clock

namespace
{

Header rigHeader()
{
    Header h;
    h.ignition_gps_s = kIgnitionGps;
    h.scen_start_s = -180.0;
    h.scen_end_s = 76.7;
    return h;
}

// a fix at replay time t, received when the MCU clock read mcu_s (GPS seconds
// near 1.47e9 hold 0.24 us: the times below agree to a microsecond)
void fixAt(ReplayClock& c, double t, double mcu_s)
{
    c.onFix(kIgnitionGps + t, static_cast<int64_t>(std::llround(mcu_s * 1e6)));
}

}  // namespace

TEST(ReplayClock, WaitsForTheRigsFirstFix)
{
    ReplayClock c;
    c.reset(rigHeader());
    double t = 0.0;
    EXPECT_FALSE(c.tAt(1000000, t));
    EXPECT_EQ(c.state(), ReplayClock::kWaiting);
    // a fix without the MCU instant (the time sync not ready) or a GPS time
    c.onFix(kIgnitionGps - 150.0, 0);
    c.onFix(0.0, 5000000);
    EXPECT_EQ(c.state(), ReplayClock::kWaiting);
    // a fix from another time (the receiver's own clock before the rig): not the rig's
    c.onFix(kIgnitionGps + 86400.0 * 45, 6000000);
    c.onFix(kIgnitionGps - 300.0, 6100000);
    EXPECT_EQ(c.state(), ReplayClock::kWaiting);
    EXPECT_EQ(c.outside(), 2u);
    // the rig: TOW 573786 - 140 at MCU 50 s
    fixAt(c, -140.0, 50.0);
    EXPECT_EQ(c.state(), ReplayClock::kFollowing);
    EXPECT_EQ(c.fixes(), 1u);
    ASSERT_TRUE(c.tAt(60000000, t));
    EXPECT_NEAR(t, -130.0, 1e-6);
}

TEST(ReplayClock, FollowsOnThePadAndFreezesBeforeIgnition)
{
    ReplayClock c;
    c.reset(rigHeader());
    fixAt(c, -140.0, 50.0);
    // the time sync moves the fixes' instants by a millisecond: followed
    fixAt(c, -100.0, 90.001);
    double t = 0.0;
    ASSERT_TRUE(c.tAt(90001000, t));
    EXPECT_NEAR(t, -100.0, 1e-6);
    EXPECT_EQ(c.restarts(), 0u);
    // the last fix before arm_by (-5 s) still moves it; the first after freezes it
    fixAt(c, -5.1, 184.9);
    EXPECT_EQ(c.state(), ReplayClock::kFollowing);
    fixAt(c, -4.9, 185.102);
    EXPECT_EQ(c.state(), ReplayClock::kFrozen);
    ASSERT_TRUE(c.tAt(185102000, t));
    EXPECT_NEAR(t, -4.9, 1e-6);
    // from here the MCU's crystal: a fix 3 ms off moves nothing
    fixAt(c, 10.0, 200.005);
    ASSERT_TRUE(c.tAt(200005000, t));
    EXPECT_NEAR(t, 10.003, 1e-6);
    EXPECT_NEAR(c.lastFixT(), 10.0, 1e-6);
}

TEST(ReplayClock, ARestartOfTheFileOnThePadIsFollowed)
{
    ReplayClock c;
    c.reset(rigHeader());
    fixAt(c, -140.0, 50.0);
    // the rig restarted its file: the receiver's time steps back 60 s
    fixAt(c, -150.0, 100.0);
    EXPECT_EQ(c.restarts(), 1u);
    double t = 0.0;
    ASSERT_TRUE(c.tAt(110000000, t));
    EXPECT_NEAR(t, -140.0, 1e-6);
}

TEST(ReplayClock, AClockAfterIgnitionNeverStartsTheFlight)
{
    ReplayClock c;
    c.reset(rigHeader());
    fixAt(c, 3.0, 100.0);   // the receiver's first fix came mid-boost
    EXPECT_EQ(c.state(), ReplayClock::kMissed);
    double t = 0.0;
    EXPECT_FALSE(c.tAt(110000000, t));
    fixAt(c, 4.0, 101.0);
    EXPECT_EQ(c.state(), ReplayClock::kMissed);
    // a reset starts again
    c.reset(rigHeader());
    EXPECT_EQ(c.state(), ReplayClock::kWaiting);
    EXPECT_EQ(c.fixes(), 0u);
}
