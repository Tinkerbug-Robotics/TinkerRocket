// Host tests for the raw-GNSS shadow filter (#1600 phase 4): the cross-core
// handoffs (ShadowLink.h) and RawGnssShadow, the part of the flight
// computer's core-0 task that is not FreeRTOS -- when it starts and stops,
// when a raw epoch is applied and how late, the pad resync, its 10 Hz log
// record -- and, end to end, two real MSM7 epochs the Beetle's LC86G sent
// (fixtures/raw_meas_golden.h) through the flight parser into a running
// shadow filter.
#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <thread>
#include <vector>

#include "Lc86Parser.h"
#include "Lc86RawEpoch.h"
#include "RawGnssShadow.h"
#include "Rtcm3Msm.h"
#include "ShadowLink.h"
#include "fixtures/raw_meas_golden.h"
#include "gnss/atmo.h"

using gnss_raw::RawGnssShadow;
using gnss_raw::ShadowImu;
using gnss_raw::ShadowSeed;
using gnss_raw::TimedRawEpoch;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int64_t kT0 = 5000000000LL;      // 5000 s of MCU time: past a 32-bit wrap
constexpr double kG = 9.807;

ShadowSeed padSeed(int64_t t_us, double lat = 32.99 * kPi / 180.0, double lon = -106.97 * kPi / 180.0,
                   double h = 1400.0)
{
    ShadowSeed s{};
    s.time_us = static_cast<uint32_t>(t_us);
    s.lla[0] = lat;
    s.lla[1] = lon;
    s.lla[2] = h;
    s.quat[0] = 1.0f;          // body = NED: level
    s.on_pad = 1;
    return s;
}

ShadowImu still(int64_t t_us, uint8_t flags = gnss_raw::kShadowLevel)
{
    ShadowImu s{};
    s.time_us = static_cast<uint32_t>(t_us);
    s.acc[2] = static_cast<float>(-kG);
    s.flags = flags;
    return s;
}

// A filter that steps on every tick, for the tests that time things to one.
RawGnssShadow::Config everyTick()
{
    RawGnssShadow::Config c;
    c.min_step_s = 0.0;
    return c;
}

// IMU ticks at 200 Hz from t0 (inclusive) to t1 (exclusive), us.
void feedStill(RawGnssShadow& sh, int64_t t0, int64_t t1, uint8_t flags = gnss_raw::kShadowLevel)
{
    for (int64_t t = t0; t < t1; t += 5000)
        sh.onImu(still(t, flags), t);
}

std::vector<uint8_t> frame(const GoldenPayload& p)
{
    std::vector<uint8_t> f(static_cast<size_t>(p.len) + rtcm3::kFrameOverhead);
    f[0] = rtcm3::kPreamble;
    f[1] = static_cast<uint8_t>((p.len >> 8) & 0x03);
    f[2] = static_cast<uint8_t>(p.len & 0xFF);
    for (int i = 0; i < p.len; ++i)
        f[3 + i] = p.bytes[i];
    const uint32_t crc = rtcm3::crc24q(f.data(), static_cast<size_t>(p.len) + 3);
    f[p.len + 3] = static_cast<uint8_t>(crc >> 16);
    f[p.len + 4] = static_cast<uint8_t>(crc >> 8);
    f[p.len + 5] = static_cast<uint8_t>(crc);
    return f;
}

}  // namespace

// ---------------------------------------------------------------- ShadowLink

TEST(ShadowLink, RingKeepsOrderAndCountsWhatItDrops)
{
    gnss_raw::SpscRing<int, 4> r;
    int v = 0;
    EXPECT_FALSE(r.pop(v));
    for (int i = 0; i < 4; ++i)
        EXPECT_TRUE(r.push(i));
    EXPECT_FALSE(r.push(99));                 // full: dropped, counted
    EXPECT_EQ(r.dropped(), 1u);
    for (int i = 0; i < 4; ++i)
    {
        ASSERT_TRUE(r.pop(v));
        EXPECT_EQ(v, i);
    }
    EXPECT_FALSE(r.pop(v));
    for (int i = 0; i < 10; ++i)              // around the end of the buffer
    {
        EXPECT_TRUE(r.push(100 + i));
        ASSERT_TRUE(r.pop(v));
        EXPECT_EQ(v, 100 + i);
    }
}

TEST(ShadowLink, RingCarriesEveryTickBetweenTwoThreads)
{
    gnss_raw::SpscRing<ShadowImu, 64> r;
    constexpr uint32_t kN = 200000;
    std::thread producer([&] {
        for (uint32_t i = 0; i < kN; ++i)
        {
            ShadowImu s{};
            s.time_us = i;
            s.acc[0] = static_cast<float>(i);
            while (!r.push(s))
                std::this_thread::yield();
        }
    });
    uint32_t next = 0;
    bool in_order = true;
    while (next < kN)
    {
        ShadowImu s;
        if (!r.pop(s))
        {
            std::this_thread::yield();
            continue;
        }
        in_order = in_order && s.time_us == next && s.acc[0] == static_cast<float>(next);
        next++;
    }
    producer.join();
    EXPECT_TRUE(in_order);
}

TEST(ShadowLink, SeqLockReadsWholeWritesOnly)
{
    gnss_raw::SeqLock<ShadowSeed> lock;
    ShadowSeed s{};
    uint32_t seq = 0;
    EXPECT_FALSE(lock.read(s, seq));          // nothing written yet
    std::atomic<bool> done{false};
    std::thread writer([&] {
        for (uint32_t i = 1; i <= 50000; ++i)
        {
            ShadowSeed w{};
            w.time_us = i;
            for (int k = 0; k < 3; ++k)
                w.lla[k] = i;                 // a torn read would mix two writes
            lock.write(w);
        }
        done = true;
    });
    bool whole = true;
    uint32_t reads = 0, last_seq = 0;
    while (!done)
    {
        ShadowSeed r;
        if (lock.read(r, seq))
        {
            reads++;
            whole = whole && r.lla[0] == r.time_us && r.lla[1] == r.time_us && r.lla[2] == r.time_us;
            whole = whole && seq >= last_seq;
            last_seq = seq;
        }
    }
    writer.join();
    EXPECT_TRUE(whole);
    EXPECT_GT(reads, 0u);
    ShadowSeed r;
    ASSERT_TRUE(lock.read(r, seq));
    EXPECT_EQ(r.time_us, 50000u);
}

TEST(ShadowLink, UnwrapsThe32BitClock)
{
    const int64_t ref = (1LL << 32) + 1000;   // just past a wrap
    EXPECT_EQ(RawGnssShadow::unwrap32(500, ref), (1LL << 32) + 500);
    EXPECT_EQ(RawGnssShadow::unwrap32(0xFFFFFF00u, ref), (1LL << 32) - 256);   // from before it
    EXPECT_EQ(RawGnssShadow::unwrap32(2000, ref), (1LL << 32) + 2000);         // a little ahead
}

// ------------------------------------------------------------- RawGnssShadow

TEST(RawGnssShadow, StartsOnlyFromAFreshPadSeed)
{
    RawGnssShadow sh(everyTick());
    feedStill(sh, kT0, kT0 + 100000);
    EXPECT_FALSE(sh.running());               // no seed

    ShadowSeed off_pad = padSeed(kT0 + 100000);
    off_pad.on_pad = 0;
    sh.onSeed(off_pad);
    feedStill(sh, kT0 + 100000, kT0 + 200000);
    EXPECT_FALSE(sh.running());               // not from a rocket that left the pad

    sh.onSeed(padSeed(kT0 - 3000000));        // 3 s old
    feedStill(sh, kT0 + 200000, kT0 + 300000);
    EXPECT_FALSE(sh.running());

    const ShadowSeed seed = padSeed(kT0 + 300000);
    sh.onSeed(seed);
    feedStill(sh, kT0 + 300000, kT0 + 1300000);
    ASSERT_TRUE(sh.running());
    EXPECT_EQ(sh.stats().starts, 1u);
    EXPECT_NEAR(sh.filter().lla()[0], seed.lla[0], 1e-9);
    EXPECT_NEAR(sh.filter().lla()[2], seed.lla[2], 1e-3);
    // the filter adds up its float steps: within a microsecond of the IMU's clock
    EXPECT_NEAR(sh.filter().time(), (kT0 + 1295000) * 1e-6, 1e-6);
}

TEST(RawGnssShadow, AStepNotTakenStopsIt)
{
    RawGnssShadow sh;
    sh.onSeed(padSeed(kT0));
    feedStill(sh, kT0, kT0 + 500000);
    ASSERT_TRUE(sh.running());
    sh.onImu(still(kT0 + 495000), kT0 + 495000);   // a repeated stamp: skipped
    EXPECT_TRUE(sh.running());
    EXPECT_EQ(sh.stats().imu_repeats, 1u);
    feedStill(sh, kT0 + 1200000, kT0 + 1210000);    // a 0.7 s hole
    EXPECT_EQ(sh.stats().last_stop, RawGnssShadow::kStopGap);
    feedStill(sh, kT0 + 1210000, kT0 + 1300000);    // and the next fresh pad seed? none: still old
    sh.onSeed(padSeed(kT0 + 1300000));
    feedStill(sh, kT0 + 1300000, kT0 + 1400000);
    EXPECT_TRUE(sh.running());
    sh.onImu(still(kT0 + 1400000, gnss_raw::kShadowSim), kT0 + 1400000);   // a sensor sim
    EXPECT_FALSE(sh.running());
    EXPECT_EQ(sh.stats().last_stop, RawGnssShadow::kStopSim);
}

TEST(RawGnssShadow, LogsTenRecordsASecondWhileRunning)
{
    RawGnssShadow sh;
    RawGnssEkfData rec;
    sh.onSeed(padSeed(kT0));
    int n = 0;
    for (int64_t t = kT0; t < kT0 + 2000000; t += 5000)
    {
        sh.onImu(still(t), t);
        while (sh.takeLog(rec))
        {
            n++;
            EXPECT_EQ(rec.time_us, static_cast<uint32_t>(t));
        }
    }
    EXPECT_NEAR(n, 20, 1);
    EXPECT_EQ(rec.lat_e7, static_cast<int32_t>(std::lround(32.99e7)));
    EXPECT_EQ(rec.lon_e7, static_cast<int32_t>(std::lround(-106.97e7)));
    EXPECT_NEAR(rec.alt_mm, 1400000, 2);
    EXPECT_TRUE(rec.flags & RGE_RUNNING);
    EXPECT_TRUE(rec.flags & RGE_LEVEL);
    EXPECT_FALSE(rec.flags & RGE_CLOCK);      // no raw epoch yet
    EXPECT_FALSE(rec.flags & RGE_IN_FLIGHT);
    EXPECT_FLOAT_EQ(rec.quat[0], 1.0f);
}

TEST(RawGnssShadow, HoldsAnEpochUntilTheImuPassesItThenAppliesItWithItsAge)
{
    RawGnssShadow sh(everyTick());
    sh.onSeed(padSeed(kT0));
    feedStill(sh, kT0, kT0 + 1000000);
    TimedRawEpoch e;
    e.epoch.tow_s = 100.0;
    e.epoch.n = 0;                            // nothing to build: only its timing is tested here
    e.mcu_us = kT0 + 1200000;                 // ahead of the IMU
    sh.onEpoch(e);
    feedStill(sh, kT0 + 1000000, kT0 + 1200000);
    EXPECT_EQ(sh.stats().epochs_applied, 0u);   // the IMU is just short of it
    feedStill(sh, kT0 + 1200000, kT0 + 1260000);
    EXPECT_EQ(sh.stats().epochs_applied, 1u);
    EXPECT_NEAR(sh.stats().last_age_s, 0.0, 1e-6);   // the first tick at it

    e.mcu_us = kT0 + 1100000;                 // 0.16 s behind the IMU when it arrives
    e.epoch.tow_s = 101.0;
    sh.onEpoch(e);
    feedStill(sh, kT0 + 1260000, kT0 + 1265000);
    EXPECT_EQ(sh.stats().epochs_applied, 2u);
    EXPECT_NEAR(sh.stats().last_age_s, 0.16, 1e-6);
    EXPECT_EQ(sh.stats().last_tow_ms, 101000u);

    e.mcu_us = kT0 + 500000;                  // 0.77 s behind: past the history
    sh.onEpoch(e);
    feedStill(sh, kT0 + 1265000, kT0 + 1270000);
    EXPECT_EQ(sh.stats().epochs_applied, 2u);
    EXPECT_EQ(sh.stats().epochs_stale, 1u);

    e.mcu_us = 0;                             // no MCU time yet
    sh.onEpoch(e);
    EXPECT_EQ(sh.stats().epochs_untimed, 1u);
}

TEST(RawGnssShadow, TakesTheWholeStateFromThePadUntilItHasAFixOfItsOwn)
{
    RawGnssShadow sh;
    ShadowSeed seed = padSeed(kT0);
    sh.onSeed(seed);
    feedStill(sh, kT0, kT0 + 500000);
    // the flight filter moves its pad position (its own GNSS settling)
    seed.lla[2] = 1410.0;
    seed.time_us = static_cast<uint32_t>(kT0 + 500000);
    seed.quat[0] = 0.0f;                       // and its attitude: nose up
    seed.quat[0] = 0.70710678f;
    seed.quat[2] = 0.70710678f;
    sh.onSeed(seed);
    feedStill(sh, kT0 + 500000, kT0 + 1100000);
    EXPECT_GE(sh.stats().resyncs, 1u);
    // taken whole (no fix of its own yet): 1,410 m, not the 1,400 it started at
    // (the level IMU this test feeds disagrees with the nose-up attitude
    // taken with it, so the height wanders a few cm after the resync)
    EXPECT_NEAR(sh.filter().lla()[2], 1410.0, 0.5);
    EXPECT_NEAR(sh.filter().quat()[2], 0.70710678f, 1e-3f);
    // in flight nothing is taken
    seed.lla[2] = 1500.0;
    seed.time_us = static_cast<uint32_t>(kT0 + 1100000);
    sh.onSeed(seed);
    feedStill(sh, kT0 + 1100000, kT0 + 2300000, gnss_raw::kShadowInFlight);
    EXPECT_LT(sh.filter().lla()[2], 1420.0);
}

TEST(RawGnssShadow, StepsAtTwentyFiveHertzOnTheMeanOfTheTicks)
{
    // 200 Hz ticks of 1 m/s^2 north, alternately 0.5 and 1.5 so only their
    // mean is the truth: one step in eight, on that mean
    RawGnssShadow sh;                         // min_step_s 0.04
    RawGnssShadow every(everyTick());
    sh.onSeed(padSeed(kT0));
    every.onSeed(padSeed(kT0));
    for (int64_t t = kT0; t < kT0 + 1000000; t += 5000)
    {
        ShadowImu s = still(t, 0);
        s.acc[0] = ((t - kT0) / 5000) % 2 ? 1.5f : 0.5f;
        sh.onImu(s, t);
        every.onImu(s, t);
    }
    EXPECT_NEAR(static_cast<double>(sh.stats().step_n), 25.0, 1.0);
    EXPECT_NEAR(static_cast<double>(every.stats().step_n), 199.0, 1.0);
    // each has the velocity of the time it has integrated, at 1 m/s^2
    const double t0 = static_cast<double>(kT0) * 1e-6;
    EXPECT_NEAR(sh.filter().velNed()[0], sh.filter().time() - t0, 0.01);
    EXPECT_NEAR(every.filter().velNed()[0], every.filter().time() - t0, 0.01);
}

TEST(RawGnssShadow, FlightFromTheBeetlesOwnRtcm)
{
    // Every ephemeris and two MSM7 epochs the LC86G sent on the rig, through
    // the flight parser, into a shadow filter seeded at the prototype's fix.
    lc86::Lc86Parser parser;
    std::vector<TimedRawEpoch> epochs;
    for (int i = 0; i < kGoldenRmMsmN; ++i)
        for (uint8_t b : frame(kGoldenRmMsm[i]))
            if (parser.feed(b) == lc86::Event::MSM_EPOCH)
            {
                TimedRawEpoch e;
                ASSERT_TRUE(lc86::rawEpochFromMsm(parser.msmEpoch(), e.epoch));
                epochs.push_back(e);
            }
    ASSERT_EQ(epochs.size(), 2u);
    double lat, lon, h;
    ecef_to_geo(kGoldenRmRx, &lat, &lon, &h);

    RawGnssShadow sh(everyTick());
    for (int i = 0; i < kGoldenRmEphN; ++i)
        sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
    EXPECT_EQ(sh.stats().eph_added, static_cast<uint32_t>(kGoldenRmEphN));
    EXPECT_EQ(sh.ephemerisSatellites(), kGoldenRmEphN);

    sh.onSeed(padSeed(kT0, lat, lon, h));
    feedStill(sh, kT0, kT0 + 1000000);
    ASSERT_TRUE(sh.running());
    // the epochs on the MCU clock: a second apart, each arriving 0.2 s late
    for (size_t k = 0; k < epochs.size(); ++k)
    {
        const int64_t te = kT0 + 1000000 + static_cast<int64_t>(k) * 1000000;
        feedStill(sh, kT0 + 1000000 + static_cast<int64_t>(k) * 1000000, te + 200000);
        epochs[k].mcu_us = te;
        sh.onEpoch(epochs[k]);
        feedStill(sh, te + 200000, te + 205000);
        EXPECT_EQ(sh.stats().epochs_applied, k + 1);
        EXPECT_NEAR(sh.stats().last_age_s, 0.2, 1e-6);
        // the first epoch has 8 satellites, the second 4 more (the fixture's 12)
        EXPECT_GE(sh.stats().last.used_pr, k == 0 ? 7 : kGoldenRmMeasN - 1);
        EXPECT_EQ(sh.stats().last.rejected_pr, 0);
    }
    EXPECT_TRUE(sh.filter().clockReady());
    // it stays at the fix the same pseudoranges gave the prototype (the
    // flight computer has no ionosphere yet: the night-time model only)
    double r[3], v[3];
    sh.filter().receiverEcef(r, v);
    const double d = std::sqrt((r[0] - kGoldenRmRx[0]) * (r[0] - kGoldenRmRx[0])
                             + (r[1] - kGoldenRmRx[1]) * (r[1] - kGoldenRmRx[1])
                             + (r[2] - kGoldenRmRx[2]) * (r[2] - kGoldenRmRx[2]));
    EXPECT_LT(d, 10.0);
}
