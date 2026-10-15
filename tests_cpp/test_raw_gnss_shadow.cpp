// Host tests for the raw-GNSS shadow filter (#1600 phase 4): the cross-core
// handoffs (ShadowLink.h) and RawGnssShadow, the part of the flight
// computer's core-0 task that is not FreeRTOS -- when it starts and stops,
// when a raw epoch is applied and how late, the pad resync, its 10 Hz log
// record -- and, end to end, two real MSM7 epochs the Beetle's LC86G sent
// (fixtures/raw_meas_golden.h) through the flight parser into a running
// shadow filter.
#include <gtest/gtest.h>

#include <cstdlib>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <random>
#include <thread>
#include <vector>

#include "Lc86Parser.h"
#include "Lc86RawEpoch.h"
#include "RawGnssShadow.h"
#include "Rtcm3Msm.h"
#include "RtcmEphemeris.h"
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

TEST(ShadowLink, SlotsAreFilledAndReadInPlaceInOrder)
{
    // The GNSS task's built epochs (~6 kB) are not copied through the ring:
    // the task fills a slot where it lies (PSRAM on the flight computer), the
    // filter's task reads it there; two wait at most.
    gnss_raw::SpscSlots<int, 2> slots;
    EXPECT_EQ(slots.claim(), nullptr);        // nothing attached
    int items[2] = {};
    slots.attach(items);
    EXPECT_EQ(slots.peek(), nullptr);
    for (int v = 1; v <= 2; ++v)
    {
        int* s = slots.claim();
        ASSERT_NE(s, nullptr);
        *s = v;
        EXPECT_EQ(slots.peek() == nullptr, v == 1);   // not the consumer's until published
        slots.publish();
    }
    EXPECT_EQ(slots.claim(), nullptr);        // both waiting: full
    ASSERT_NE(slots.peek(), nullptr);
    EXPECT_EQ(*slots.peek(), 1);
    slots.release();
    int* s = slots.claim();                   // the freed one, filled again
    ASSERT_NE(s, nullptr);
    *s = 3;
    slots.publish();
    for (int want : {2, 3})
    {
        ASSERT_NE(slots.peek(), nullptr);
        EXPECT_EQ(*slots.peek(), want);
        slots.release();
    }
    EXPECT_EQ(slots.peek(), nullptr);
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

// ------------------------------------------------- the ionosphere (phase 5)

namespace
{

// The fixture's Klobuchar (BRDC_2026241, kGoldenRmIon) in the bytes GPS
// broadcasts, dated by GPS day (2026-08-29 is 17037: week 2433, Saturday).
constexpr uint16_t kGoldenDay = 17037;
constexpr uint16_t kGoldenWeek = 2433;

gnss_raw::ShadowIono goldenIono(uint16_t day)
{
    gnss_raw::ShadowIono io{};
    const int8_t b[8] = {14, 3, -1, -2, 51, 6, -2, -5};
    for (int k = 0; k < 8; ++k)
        io.raw[k] = b[k];
    io.gps_day = day;
    return io;
}

// The fixture's two MSM7 epochs through the flight parser, on the MCU clock
// a second apart from kT0 + 1 s.
std::vector<TimedRawEpoch> goldenEpochs()
{
    lc86::Lc86Parser parser;
    std::vector<TimedRawEpoch> epochs;
    for (int i = 0; i < kGoldenRmMsmN; ++i)
        for (uint8_t b : frame(kGoldenRmMsm[i]))
            if (parser.feed(b) == lc86::Event::MSM_EPOCH)
            {
                TimedRawEpoch e;
                EXPECT_TRUE(lc86::rawEpochFromMsm(parser.msmEpoch(), e.epoch));
                e.mcu_us = kT0 + 1000000 + static_cast<int64_t>(epochs.size()) * 1000000;
                e.gps_week = kGoldenWeek;
                epochs.push_back(e);
            }
    return epochs;
}

ShadowSeed goldenSeed(int64_t t_us, bool pad_valid)
{
    double lat, lon, h;
    ecef_to_geo(kGoldenRmRx, &lat, &lon, &h);
    ShadowSeed s = padSeed(t_us, lat, lon, h);
    s.pad_valid = pad_valid ? 1 : 0;
    s.pad_sigma_m = 2.5f;
    for (int a = 0; a < 3; ++a)
        s.pad_lla[a] = s.lla[a];
    return s;
}

// Each golden epoch handed over 0.2 s late, and the IMU carried past it.
void runGolden(RawGnssShadow& sh, std::vector<TimedRawEpoch>& epochs, size_t k, uint8_t flags = gnss_raw::kShadowLevel)
{
    const int64_t te = epochs[k].mcu_us;
    feedStill(sh, te - 1000000 + 5000, te + 200000, flags);
    sh.onEpoch(epochs[k]);
    feedStill(sh, te + 200000, te + 205000, flags);
}

// Each look at the clock a millisecond on: every unit of an epoch's work
// costs 1 ms, so the slicing does not hang on the host's speed.
int64_t g_fake_us = 0;
int64_t fakeClock() { return g_fake_us += 1000; }

}  // namespace

TEST(RawGnssShadow, TheNavigationFilterFusesAnEpochInSlicesAndKeepsStepping)
{
    // As the navigation filter it works an epoch a slice a step
    // (epoch_slice_us), so the state it publishes keeps up with the IMU while
    // it builds and fuses a sky: on the rig (r7-r9) one pass over 14
    // satellites held it 40 ms. The golden epochs, each handed over 20 ms
    // late, at once and in 2.5 ms slices: the sliced run steps through its
    // epochs, fuses every one, and ends where the other does.
    std::vector<TimedRawEpoch> epochs = goldenEpochs();
    ASSERT_EQ(epochs.size(), 2u);
    double r_end[2][3];
    uint32_t slices[2] = {};
    uint32_t steps_during[2] = {};
    for (int sliced = 0; sliced < 2; ++sliced)
    {
        RawGnssShadow::Config cfg;            // the navigation filter's (primaryCfg, below)
        cfg.primary = true;
        cfg.min_step_s = 0.01;
        if (sliced)
            cfg.epoch_slice_us = 2500.0;
        RawGnssShadow sh(cfg);
        g_fake_us = 0;
        sh.setClock(fakeClock);               // both runs on it: only the slicing differs
        for (int i = 0; i < kGoldenRmEphN; ++i)
            sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
        sh.onSeed(goldenSeed(kT0, false));
        int64_t t = kT0;
        feedStill(sh, t, kT0 + 1000000);
        t = kT0 + 1000000;
        ASSERT_TRUE(sh.running());
        for (size_t k = 0; k < epochs.size(); ++k)
        {
            const int64_t te = epochs[k].mcu_us;
            feedStill(sh, t, te + 20000);
            t = te + 20000;
            sh.onEpoch(epochs[k]);
            const uint32_t applied = sh.stats().epochs_applied;
            const uint32_t steps0 = sh.stats().step_n;
            while (sh.stats().epochs_applied == applied && t < te + 400000)
            {
                feedStill(sh, t, t + 5000);
                t += 5000;
            }
            ASSERT_EQ(sh.stats().epochs_applied, applied + 1) << "epoch " << k;
            steps_during[sliced] = std::max(steps_during[sliced], sh.stats().step_n - steps0);
            EXPECT_GE(sh.stats().last.used_pr, k == 0 ? 7 : kGoldenRmMeasN - 1);
            EXPECT_LT(sh.stats().last_age_s, 0.3);
            slices[sliced] = std::max(slices[sliced], sh.stats().epoch_slices);
        }
        double v[3];
        sh.filter().receiverEcef(r_end[sliced], v);
    }
    EXPECT_EQ(slices[0], 1u);
    EXPECT_GE(slices[1], 5u);
    EXPECT_LE(steps_during[0], 1u);
    EXPECT_GE(steps_during[1], 5u);           // it stepped while it fused
    const double d = std::sqrt((r_end[0][0] - r_end[1][0]) * (r_end[0][0] - r_end[1][0])
                             + (r_end[0][1] - r_end[1][1]) * (r_end[0][1] - r_end[1][1])
                             + (r_end[0][2] - r_end[1][2]) * (r_end[0][2] - r_end[1][2]));
    EXPECT_LT(d, 1.0);
}

TEST(RawGnssShadow, TheBroadcastBytesAreTheDaysKlobuchar)
{
    const gps_iono_t io = gnss_raw::RawMeasBuilder::ionoFromBroadcast(goldenIono(kGoldenDay).raw);
    EXPECT_TRUE(io.valid);
    for (int k = 0; k < 4; ++k)
    {
        // the RINEX header prints five digits
        EXPECT_NEAR(io.alpha[k], kGoldenRmIon[k], 1e-4 * std::fabs(kGoldenRmIon[k])) << k;
        EXPECT_NEAR(io.beta[k], kGoldenRmIon[4 + k], 1e-4 * std::fabs(kGoldenRmIon[4 + k])) << k;
    }
}

TEST(RawGnssShadow, UsesTheBroadcastIonosphereWhileItIsCurrent)
{
    std::vector<TimedRawEpoch> epochs = goldenEpochs();
    ASSERT_EQ(epochs.size(), 2u);
    RawGnssShadow sh(everyTick());
    for (int i = 0; i < kGoldenRmEphN; ++i)
        sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
    sh.onSeed(goldenSeed(kT0, false));
    feedStill(sh, kT0, kT0 + 5000);
    ASSERT_TRUE(sh.running());
    int age = 0;
    EXPECT_FALSE(sh.haveIono());
    EXPECT_FALSE(sh.ionoAgeDays(age));
    EXPECT_EQ(sh.ionoSource(), RawGnssShadow::kIonoNone);

    sh.setIono(goldenIono(kGoldenDay - 3));
    EXPECT_FALSE(sh.ionoAgeDays(age));        // the receiver's date is not known yet
    runGolden(sh, epochs, 0);
    ASSERT_TRUE(sh.ionoAgeDays(age));
    EXPECT_EQ(age, 3);
    EXPECT_EQ(sh.ionoSource(), RawGnssShadow::kIonoBroadcast);
    EXPECT_DOUBLE_EQ(sh.builder().iono().alpha[0], 14 * 9.313225746154785e-10);
    EXPECT_EQ(sh.stats().last_pad_corrected, 0);

    // a set 31 days old gives way: no pad corrections yet, so nothing
    sh.setIono(goldenIono(kGoldenDay - 31));
    runGolden(sh, epochs, 1);
    ASSERT_TRUE(sh.ionoAgeDays(age));
    EXPECT_EQ(age, 31);
    EXPECT_EQ(sh.ionoSource(), RawGnssShadow::kIonoNone);
    EXPECT_FALSE(sh.builder().iono().valid);
    EXPECT_EQ(sh.builder().iono().alpha[0], 0.0);
    EXPECT_EQ(sh.stats().iono_sets, 2u);
}

TEST(RawGnssShadow, KeepsItsIonosphereThroughTheFlight)
{
    std::vector<TimedRawEpoch> epochs = goldenEpochs();
    RawGnssShadow sh(everyTick());
    for (int i = 0; i < kGoldenRmEphN; ++i)
        sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
    sh.setIono(goldenIono(kGoldenDay));
    sh.onSeed(goldenSeed(kT0, false));
    feedStill(sh, kT0, kT0 + 5000);
    runGolden(sh, epochs, 0);
    ASSERT_EQ(sh.ionoSource(), RawGnssShadow::kIonoBroadcast);
    // in flight, parameters that would not be used on the pad change nothing
    sh.setIono(goldenIono(kGoldenDay - 90));
    runGolden(sh, epochs, 1, gnss_raw::kShadowInFlight);
    EXPECT_EQ(sh.ionoSource(), RawGnssShadow::kIonoBroadcast);
    EXPECT_TRUE(sh.builder().iono().valid);
}

TEST(RawGnssShadow, SurveysThePadOnlyOnThePadWithALaunchSite)
{
    std::vector<TimedRawEpoch> epochs = goldenEpochs();
    {
        RawGnssShadow sh(everyTick());
        for (int i = 0; i < kGoldenRmEphN; ++i)
            sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
        sh.onSeed(goldenSeed(kT0, false));    // no launch site yet
        feedStill(sh, kT0, kT0 + 5000);
        runGolden(sh, epochs, 0);
        EXPECT_EQ(sh.stats().survey_epochs, 0u);
        sh.onSeed(goldenSeed(kT0 + 1300000, true));
        runGolden(sh, epochs, 1);
        EXPECT_EQ(sh.stats().survey_epochs, 1u);
        // each satellite's lock is the shadow's own: two epochs is still settling
        EXPECT_EQ(sh.survey().summary().tracked, sh.stats().last_meas);
        EXPECT_EQ(sh.survey().summary().settling, sh.stats().last_meas);
        EXPECT_EQ(sh.survey().summary().corrected, 0);
    }
    {
        RawGnssShadow sh(everyTick());
        for (int i = 0; i < kGoldenRmEphN; ++i)
            sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
        sh.onSeed(goldenSeed(kT0, true));
        feedStill(sh, kT0, kT0 + 5000);
        runGolden(sh, epochs, 0, gnss_raw::kShadowInFlight);
        EXPECT_EQ(sh.stats().survey_epochs, 0u);   // not in flight
    }
}

TEST(RawGnssShadow, CountsTheTrackedSatellitesWithNoEphemeris)
{
    std::vector<TimedRawEpoch> epochs = goldenEpochs();
    RawGnssShadow sh(everyTick());
    sh.onEpoch(epochs[1]);                    // not running: still counted
    const int tracked = sh.stats().tracked;
    EXPECT_GE(tracked, kGoldenRmMeasN);
    EXPECT_EQ(sh.stats().tracked_no_eph, tracked);
    for (int i = 0; i < kGoldenRmEphN; ++i)
        sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
    sh.onEpoch(epochs[1]);
    EXPECT_EQ(sh.stats().tracked, tracked);
    // the capture carried an ephemeris for every satellite the epoch has but
    // the ones the builder leaves for want of one
    EXPECT_EQ(sh.stats().tracked_no_eph, tracked - kGoldenRmUnmaskedN);
}

// ------------------------------------------------ the physics checks (#1508)

namespace
{

// A receiver fix at the golden pad, offset north by n_m, its GPS instant at t_us.
gnss_raw::TimedFix goldenFix(int64_t t_us, double n_m = 0.0, uint8_t fix_mode = 3)
{
    double lat, lon, h;
    ecef_to_geo(kGoldenRmRx, &lat, &lon, &h);
    gnss_raw::TimedFix f;
    f.lat_rad = lat + n_m / 6.37e6;
    f.lon_rad = lon;
    f.alt_m = h;
    f.fix_mode = fix_mode;
    f.num_sats = 12;
    f.mcu_us = t_us;
    f.arrival_us = t_us + 120000;
    return f;
}

}  // namespace

TEST(RawGnssShadow, JudgesEachReceiverFixOnceTheImuHasPassedIt)
{
    RawGnssShadow sh;                         // 25 Hz steps: the flight computer's history
    sh.onSeed(goldenSeed(kT0, false));
    feedStill(sh, kT0, kT0 + 5000);
    ASSERT_TRUE(sh.running());
    GnssCheckData rec;
    int pass = 0, flagged = 0, unjudged = 0;
    for (int k = 0; k < 20; ++k)
    {
        const int64_t tf = kT0 + 100000 + k * 100000;
        // the fix arrives before the IMU reaches its instant: held
        sh.onFix(goldenFix(tf, k == 12 ? 300.0 : 0.0));
        feedStill(sh, tf - 95000, tf - 5000);
        EXPECT_FALSE(sh.takeCheckLog(rec));
        feedStill(sh, tf - 5000, tf + 45000);  // a 25 Hz step past it
        ASSERT_TRUE(sh.takeCheckLog(rec)) << k;
        EXPECT_EQ(rec.time_us, static_cast<uint32_t>(tf));
        if (rec.verdict == GCK_PASS) pass++;
        if (rec.verdict == GCK_FLAGGED) flagged++;
        if (rec.verdict == GCK_UNJUDGED) unjudged++;
        if (k == 12)
        {
            EXPECT_EQ(rec.verdict, GCK_FLAGGED);
            EXPECT_NEAR(rec.pos_err_m, 300.0, 3.0);
        }
    }
    // Unjudged: the first, and the one or two after each pad resync. With no
    // raw epoch of its own yet the shadow retakes its whole state from the
    // flight filter once a second, which starts the IMU history over.
    EXPECT_GE(unjudged, 1);
    EXPECT_LE(unjudged, 5);
    EXPECT_EQ(flagged, 1);                    // the jump, and the track passes again after it
    EXPECT_EQ(pass + flagged + unjudged, 20);
    EXPECT_EQ(sh.fixCheck().stats().flagged, 1u);
    // a fix with no MCU time, or no 3-D fix, is not judged
    sh.onFix(goldenFix(0));
    sh.onFix(goldenFix(kT0 + 2200000, 0.0, 2));
    feedStill(sh, kT0 + 2100000, kT0 + 2400000);
    EXPECT_FALSE(sh.takeCheckLog(rec));
    EXPECT_EQ(sh.stats().fixes_untimed, 1u);
    gnss_raw::ShadowPadStatus ps;
    sh.padStatus(ps);
    EXPECT_EQ(ps.fix_suspect, 0);
}

TEST(RawGnssShadow, TakesASatelliteWhoseCodeJumpedAsRelocked)
{
    // The rig's two epochs were taken under way (the builder's g_peak is 2 g)
    // and this IMU stands still, so the check already sees a satellite or two
    // move as the receiver did; a 40 m jump put on one it did not flag is
    // flagged as well, and taken as relocked.
    auto run = [](int jump_index, RawGnssShadow& sh) {
        std::vector<TimedRawEpoch> epochs = goldenEpochs();
        for (int i = 0; i < kGoldenRmEphN; ++i)
            sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
        sh.onSeed(goldenSeed(kT0, false));
        feedStill(sh, kT0, kT0 + 5000);
        runGolden(sh, epochs, 0);
        EXPECT_EQ(sh.satCheck().summary().judged, 0);
        if (jump_index >= 0)
            epochs[1].epoch.obs[jump_index].pr_m += 40.0;
        runGolden(sh, epochs, 1);
        return epochs;
    };
    RawGnssShadow ref(everyTick());
    const std::vector<TimedRawEpoch> epochs = run(-1, ref);
    const int base = ref.stats().last_sats_jumped;
    EXPECT_GE(ref.satCheck().summary().judged, 5);
    // a satellite of both epochs the check left alone
    int target = -1;
    for (int i = 0; i < epochs[1].epoch.n && target < 0; ++i)
    {
        const gnss_raw::RawObs& o = epochs[1].epoch.obs[i];
        if (!o.has_pr || o.sys > GNSS_SYS_BDS)
            continue;
        bool in_first = false;
        for (int j = 0; j < epochs[0].epoch.n; ++j)
            in_first |= epochs[0].epoch.obs[j].sys == o.sys && epochs[0].epoch.obs[j].prn == o.prn;
        if (in_first && ref.builder().relockAge(o.sys, o.prn, kGoldenRmTowB) > 0.5)
            target = i;
    }
    ASSERT_GE(target, 0);
    RawGnssShadow sh(everyTick());
    run(target, sh);
    const gnss_raw::RawObs& o = epochs[1].epoch.obs[target];
    EXPECT_EQ(sh.stats().last_sats_jumped, base + 1);
    EXPECT_EQ(sh.builder().relockAge(o.sys, o.prn, kGoldenRmTowB), 0.0);
    EXPECT_EQ(sh.stats().sats_jumped, static_cast<uint32_t>(base + 1));
}

TEST(RawGnssShadow, LevelsOnTheGroundButInFlightOnlyOnceSteady)
{
    // The flight filter levels after apogee, and as a drogue opens that is
    // wrong (LevelGate.h, the rig replay of 2026-10-13). With no raw epochs
    // there is nothing to call the descent steady from: in flight the shadow
    // holds off, and its records say it did not level.
    RawGnssShadow sh;
    RawGnssEkfData rec;
    sh.onSeed(padSeed(kT0));
    feedStill(sh, kT0, kT0 + 1000000);           // the pad
    ASSERT_TRUE(sh.running());
    const uint32_t pad_levels = sh.stats().levels;
    EXPECT_GT(pad_levels, 20u);                  // 25 steps a second
    EXPECT_EQ(sh.stats().levels_held, 0u);
    while (sh.takeLog(rec)) {}
    EXPECT_TRUE(rec.flags & RGE_LEVEL);

    const uint8_t descent = gnss_raw::kShadowLevel | gnss_raw::kShadowInFlight;   // after apogee
    feedStill(sh, kT0 + 1000000, kT0 + 2000000, descent);
    ASSERT_TRUE(sh.running());
    EXPECT_EQ(sh.stats().levels, pad_levels);
    EXPECT_GT(sh.stats().levels_held, 20u);
    while (sh.takeLog(rec)) {}
    EXPECT_FALSE(rec.flags & RGE_LEVEL);
    EXPECT_TRUE(rec.flags & RGE_IN_FLIGHT);

    feedStill(sh, kT0 + 2000000, kT0 + 2500000);   // landed: the ground again
    EXPECT_GT(sh.stats().levels, pad_levels);
}

// ------------------------------------------- the barometer and the landing

namespace
{

// A shadow with a fix of its own: the golden ephemerides and two MSM7 epochs
// (FlightFromTheBeetlesOwnRtcm), on the pad at the prototype's fix. Returns
// the MCU time it has reached.
int64_t ownFixShadow(RawGnssShadow& sh, double& h0)
{
    lc86::Lc86Parser parser;
    std::vector<TimedRawEpoch> epochs;
    for (int i = 0; i < kGoldenRmMsmN; ++i)
        for (uint8_t b : frame(kGoldenRmMsm[i]))
            if (parser.feed(b) == lc86::Event::MSM_EPOCH)
            {
                TimedRawEpoch e;
                lc86::rawEpochFromMsm(parser.msmEpoch(), e.epoch);
                epochs.push_back(e);
            }
    double lat, lon;
    ecef_to_geo(kGoldenRmRx, &lat, &lon, &h0);
    for (int i = 0; i < kGoldenRmEphN; ++i)
        sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
    sh.onSeed(padSeed(kT0, lat, lon, h0));
    feedStill(sh, kT0, kT0 + 1000000);
    for (size_t k = 0; k < epochs.size(); ++k)
    {
        const int64_t te = kT0 + 1000000 + static_cast<int64_t>(k) * 1000000;
        feedStill(sh, te, te + 200000);
        epochs[k].mcu_us = te;
        sh.onEpoch(epochs[k]);
        feedStill(sh, te + 200000, te + 1000000);
    }
    return kT0 + 1000000 + static_cast<int64_t>(epochs.size()) * 1000000;
}

// Ticks at 200 Hz with the vertical specific force a_up too strong (m/s^2),
// and a barometer sample every 0.1 s at height_m if baro.
int64_t feedFlight(RawGnssShadow& sh, int64_t t0, double secs, float a_up, uint8_t flags, bool baro,
                   float height_m = 0.0f)
{
    const int64_t t1 = t0 + static_cast<int64_t>(secs * 1e6);
    for (int64_t t = t0; t < t1; t += 5000)
    {
        ShadowImu s = still(t, flags);
        s.acc[2] -= a_up;
        if (baro && (t - t0) % 100000 == 0)
            sh.onBaro(gnss_raw::ShadowBaro{static_cast<uint32_t>(t), height_m});
        sh.onImu(s, t);
    }
    return t1;
}

}  // namespace

TEST(RawGnssShadow, TheBarometerHoldsTheHeightWithNoSatellites)
{
    // On the rig the receiver lost the sky under the main and the shadow,
    // with nothing else, drifted 70 m in 20 s. With the barometer's offset
    // learned on the pad and frozen at launch, 15 s of an IMU 0.3 m/s^2
    // strong in the vertical and no satellites stay at the barometer's height.
    double drift[2] = {};
    for (int with_baro = 0; with_baro < 2; ++with_baro)
    {
        RawGnssShadow sh;
        double h0 = 0.0;
        int64_t t = ownFixShadow(sh, h0);
        ASSERT_TRUE(sh.running());
        // 12 s on the pad with the barometer at the pad (0 m)
        t = feedFlight(sh, t, 12.0, 0.0f, gnss_raw::kShadowLevel, with_baro != 0);
        const double h_pad = sh.filter().lla()[2];
        if (with_baro)
        {
            EXPECT_GE(sh.stats().baro_fused, 10u);
            EXPECT_NEAR(sh.stats().baro_offset_m, h_pad, 1.0);   // the filter's height less 0
            EXPECT_EQ(sh.stats().baro_offset_frozen, 0);
        }
        t = feedFlight(sh, t, 15.0, 0.3f, gnss_raw::kShadowInFlight, with_baro != 0);
        ASSERT_TRUE(sh.running());
        drift[with_baro] = sh.filter().lla()[2] - h_pad;
        if (with_baro)
            EXPECT_EQ(sh.stats().baro_offset_frozen, 1);
    }
    EXPECT_GT(drift[0], 25.0);                   // 0.3 m/s^2 for 15 s: 34 m
    EXPECT_LT(std::fabs(drift[1]), 3.0);
}

TEST(RawGnssShadow, NoOffsetNoBarometer)
{
    // without a fix of its own on the pad there is nothing to refer the
    // barometer to: it is counted, not fused
    RawGnssShadow sh;
    sh.onSeed(padSeed(kT0));
    int64_t t = feedFlight(sh, kT0, 12.0, 0.0f, gnss_raw::kShadowLevel, true);
    EXPECT_GE(sh.stats().baro, 100u);
    EXPECT_EQ(sh.stats().baro_fused, 0u);
    (void)t;
}

TEST(RawGnssShadow, LandedIsZeroVelocityTenTimesASecond)
{
    RawGnssShadow sh;
    sh.onSeed(padSeed(kT0));
    feedStill(sh, kT0, kT0 + 1000000);
    // 3 s in flight with the IMU 0.5 m/s^2 strong: about 1.5 m/s up
    int64_t t = feedFlight(sh, kT0 + 1000000, 3.0, 0.5f, gnss_raw::kShadowInFlight, false);
    EXPECT_LT(sh.filter().velNed()[2], -1.0f);
    // landed: 2 s still
    t = feedFlight(sh, t, 2.0, 0.0f, gnss_raw::kShadowLanded | gnss_raw::kShadowLevel, false);
    EXPECT_NEAR(sh.stats().zupts, 20u, 1u);
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(sh.filter().velNed()[i], 0.0f, 0.15f);
}

// ------------------------------------- the navigation filter (Config::primary)

namespace
{

RawGnssShadow::Config primaryCfg()
{
    RawGnssShadow::Config c;
    c.primary = true;
    c.min_step_s = 0.01;
    return c;
}

// body FRD -> NED, yaw then pitch, scalar first (Quat2DCM of it is NED -> body)
void quatYawPitch(double yaw, double pitch, float q[4])
{
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2), cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    q[0] = float(cy * cp);
    q[1] = float(-sy * sp);
    q[2] = float(cy * sp);
    q[3] = float(sy * cp);
}

void nedToBody(const float q[4], const double v[3], float out[3])
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double M[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)},
                            {2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)},
                            {2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)}};
    for (int i = 0; i < 3; ++i)
        out[i] = float(M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2]);
}

double attErrDeg(const float a[4], const float b[4])
{
    const double d = std::fabs(double(a[0]) * b[0] + double(a[1]) * b[1] + double(a[2]) * b[2] + double(a[3]) * b[3]);
    return 2.0 * std::acos(std::min(1.0, d)) * 180.0 / kPi;
}

}  // namespace

TEST(RawGnssShadow, TheNavigationFilterSetsItsOwnAttitudeOnThePad)
{
    // rp5_0829 on its rail: 83.6 deg up, heading 44.7 deg; the seed only knows
    // the tilt (the flight computer's guess at the heading: north). Levelling
    // and the magnetometer, never a resync, bring it to the truth.
    float q_true[4], q_seed[4];
    quatYawPitch(44.7 * kPi / 180.0, 83.6 * kPi / 180.0, q_true);
    quatYawPitch(0.0, 83.6 * kPi / 180.0, q_seed);
    const double b_ned[3] = {21.0, -4.3, 45.5};
    const double up[3] = {0.0, 0.0, -kG};
    float acc[3], mag[3];
    nedToBody(q_true, up, acc);
    nedToBody(q_true, b_ned, mag);

    RawGnssShadow sh(primaryCfg());
    ShadowSeed seed = padSeed(kT0);
    for (int i = 0; i < 4; ++i)
        seed.quat[i] = q_seed[i];
    seed.declination_rad = float(std::atan2(b_ned[1], b_ned[0]));
    seed.field_ut = 50.3f;
    seed.have_declination = 1;
    sh.onSeed(seed);
    for (int64_t t = kT0; t < kT0 + 5000000; t += 5000)
    {
        ShadowImu s{};
        s.time_us = static_cast<uint32_t>(t);
        for (int i = 0; i < 3; ++i)
            s.acc[i] = acc[i];
        s.flags = gnss_raw::kShadowLevel;
        if ((t - kT0) % 20000 == 0)                // a 50 Hz magnetometer
        {
            gnss_raw::ShadowMag m{};
            m.time_us = static_cast<uint32_t>(t);
            for (int i = 0; i < 3; ++i)
                m.mag[i] = mag[i];
            sh.onMag(m);
        }
        if ((t - kT0) % 1000000 == 0)              // and the seed, once a second as main.cpp sends it
        {
            seed.time_us = static_cast<uint32_t>(t);
            sh.onSeed(seed);
        }
        sh.onImu(s, t);
    }
    ASSERT_TRUE(sh.running());
    EXPECT_EQ(sh.stats().resyncs, 0u);
    EXPECT_GT(sh.stats().mag_fused, 200u);
    EXPECT_LT(attErrDeg(sh.filter().quat(), q_true), 1.0);

    gnss_raw::ShadowNav nav;
    sh.navState(nav);
    EXPECT_EQ(nav.running, 1);
    for (int i = 0; i < 4; ++i)
        EXPECT_FLOAT_EQ(nav.quat[i], sh.filter().quat()[i]);
    EXPECT_NEAR(nav.lla[2], 1400.0, 1.0);
    EXPECT_EQ(nav.gnss_aided, 0);                 // no raw measurements in this test
    EXPECT_GT(nav.var[8], 0.0f);
}

TEST(RawGnssShadow, EachRecordCarriesTheAttitudeSigmaAndTheBiasesBesideIt)
{
    // RAW_GNSS_ATT_MSG, beside every RAW_GNSS_EKF_MSG record at the same tick.
    // The navigation filter up rp5_0829's rail: levelling learns the tilt, so
    // north and east are known long before down -- the heading -- is; the
    // magnetometer, from 3 s, brings the heading in, and the records show when.
    float q[4];
    quatYawPitch(44.7 * kPi / 180.0, 83.6 * kPi / 180.0, q);
    const double b_ned[3] = {21.0, -4.3, 45.5};
    const double up[3] = {0.0, 0.0, -kG};
    float acc[3], mag[3];
    nedToBody(q, up, acc);
    nedToBody(q, b_ned, mag);

    RawGnssShadow sh(primaryCfg());
    ShadowSeed seed = padSeed(kT0);
    for (int i = 0; i < 4; ++i)
        seed.quat[i] = q[i];
    seed.declination_rad = float(std::atan2(b_ned[1], b_ned[0]));
    seed.field_ut = 50.3f;
    seed.have_declination = 1;
    sh.onSeed(seed);
    RawGnssEkfData rec;
    RawGnssAttData att, before_mag{}, after_mag{};
    int n = 0;
    for (int64_t t = kT0; t < kT0 + 6000000; t += 5000)
    {
        ShadowImu s{};
        s.time_us = static_cast<uint32_t>(t);
        for (int i = 0; i < 3; ++i)
            s.acc[i] = acc[i];
        s.flags = gnss_raw::kShadowLevel;
        if (t >= kT0 + 3000000 && (t - kT0) % 20000 == 0)   // a 50 Hz magnetometer, from 3 s
        {
            gnss_raw::ShadowMag m{};
            m.time_us = static_cast<uint32_t>(t);
            for (int i = 0; i < 3; ++i)
                m.mag[i] = mag[i];
            sh.onMag(m);
        }
        if ((t - kT0) % 1000000 == 0)              // the seed, once a second as main.cpp sends it
        {
            seed.time_us = static_cast<uint32_t>(t);
            sh.onSeed(seed);
        }
        sh.onImu(s, t);
        while (sh.takeLog(rec))
        {
            ASSERT_TRUE(sh.takeAttLog(att));
            EXPECT_EQ(att.time_us, rec.time_us);
            n++;
            (t < kT0 + 3000000 ? before_mag : after_mag) = att;
        }
        EXPECT_FALSE(sh.takeAttLog(att));          // never one without its record
    }
    EXPECT_GT(n, 50);
    EXPECT_LT(before_mag.att_sigma_cdeg[0], before_mag.att_sigma_cdeg[2] / 4);
    EXPECT_LT(before_mag.att_sigma_cdeg[1], before_mag.att_sigma_cdeg[2] / 4);
    EXPECT_LT(after_mag.att_sigma_cdeg[2], before_mag.att_sigma_cdeg[2] / 4);
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_TRUE(std::isfinite(after_mag.acc_bias_mps2[i]));
        EXPECT_TRUE(std::isfinite(after_mag.gyro_bias_dps[i]));
        EXPECT_GT(after_mag.acc_bias_sigma_mmps2[i], 0);
        EXPECT_GT(after_mag.gyro_bias_sigma_mdps[i], 0);
    }
}

TEST(RawGnssShadow, TheNavigationFilterStartsInFlightFromARestoredFlight)
{
    RawGnssShadow sh(primaryCfg());
    ShadowSeed seed = padSeed(kT0, 39.47 * kPi / 180.0, -75.29 * kPi / 180.0, 600.0);
    seed.on_pad = 0;
    seed.vel_ned[2] = 15.0f;                      // descending under the drogue
    seed.restore = 1;
    sh.onSeed(seed);
    int64_t t = feedFlight(sh, kT0, 0.5, 0.0f, gnss_raw::kShadowInFlight, false);
    ASSERT_TRUE(sh.running());
    EXPECT_EQ(sh.stats().restores, 1u);
    EXPECT_NEAR(sh.filter().velNed()[2], 15.0f, 0.5f);
    // its first barometer sample sets the offset from the restored height,
    // frozen: no pad to learn it on
    t = feedFlight(sh, t, 5.0, 0.0f, gnss_raw::kShadowInFlight, true, 0.0f);
    EXPECT_EQ(sh.stats().baro_offset_frozen, 1);
    EXPECT_GE(sh.stats().baro_fused, 40u);
    // a shadow (not the navigation filter) would not start from it
    RawGnssShadow shadow;
    shadow.onSeed(seed);
    feedFlight(shadow, kT0, 0.5, 0.0f, gnss_raw::kShadowInFlight, false);
    EXPECT_FALSE(shadow.running());
}

TEST(RawGnssShadow, TheNavigationFilterFliesASimAndStartsOverOnANewStart)
{
    RawGnssShadow sh(primaryCfg());
    ShadowSeed seed = padSeed(kT0);
    seed.start_gen = 1;
    sh.onSeed(seed);
    feedStill(sh, kT0, kT0 + 500000, gnss_raw::kShadowLevel | gnss_raw::kShadowSim);
    EXPECT_TRUE(sh.running());                    // the sim's IMU is all it has
    EXPECT_EQ(sh.stats().starts, 1u);
    seed.start_gen = 2;                           // the flight computer started over on the pad
    seed.time_us = static_cast<uint32_t>(kT0 + 500000);
    sh.onSeed(seed);
    feedStill(sh, kT0 + 500000, kT0 + 600000);
    EXPECT_TRUE(sh.running());
    EXPECT_EQ(sh.stats().starts, 2u);
}

TEST(RawGnssShadow, TheNavigationFilterStepsOverAHoleTheShadowStopsAt)
{
    for (bool primary : {false, true})
    {
        RawGnssShadow::Config c;
        c.primary = primary;
        RawGnssShadow sh(c);
        sh.onSeed(padSeed(kT0));
        feedStill(sh, kT0, kT0 + 500000);
        ASSERT_TRUE(sh.running());
        feedStill(sh, kT0 + 1500000, kT0 + 2000000, gnss_raw::kShadowInFlight);   // a 1 s hole, in flight
        EXPECT_EQ(sh.stats().stops, primary ? 0u : 1u);
        if (primary)
        {
            EXPECT_TRUE(sh.running());
            EXPECT_EQ(sh.stats().gaps_bridged, 1u);
            EXPECT_NEAR(sh.filter().time(), (kT0 + 1995000) * 1e-6, 0.05);   // still on the IMU's clock
        }
        else
        {
            EXPECT_EQ(sh.stats().last_stop, RawGnssShadow::kStopGap);
        }
    }
}

TEST(RawGnssShadow, AShortPadStillGivesTheFlightItsBarometer)
{
    // A sim's pad is 5 s, under the offset's 10 s: launch freezes what was
    // learned, so the flight still has its vertical
    RawGnssShadow sh(primaryCfg());
    sh.onSeed(padSeed(kT0));
    int64_t t = feedFlight(sh, kT0, 3.0, 0.0f, gnss_raw::kShadowLevel, true);
    EXPECT_EQ(sh.stats().baro_fused, 0u);         // too short to fuse on the pad
    const double h_pad = sh.filter().lla()[2];
    t = feedFlight(sh, t, 15.0, 0.3f, gnss_raw::kShadowInFlight, true);
    EXPECT_EQ(sh.stats().baro_offset_frozen, 1);
    EXPECT_GE(sh.stats().baro_fused, 140u);
    EXPECT_LT(std::fabs(sh.filter().lla()[2] - h_pad), 3.0);
}

TEST(RawGnssShadow, InFlightTheBarometerIsHeldOutAtSpeed)
{
    // #1579's static-port error is a bias, low at speed: on the rig replay
    // (r3) the barometer read 1.4 times the modelled term low through boost
    // and coast, and fused fifty times a second it took the raw filter 21 m
    // low at the top of the coast. Here that barometer, no satellites, and
    // 1 s at 60 m/s^2 then 3 s at 60 m/s (210 m): held out from 21 m/s up,
    // the climb the IMU feels stands.
    double err[2] = {};
    for (int hold = 0; hold < 2; ++hold)
    {
        RawGnssShadow::Config cfg = primaryCfg();
        if (!hold)
            cfg.baro_hold_port_m = 1e9;           // at any speed, as before
        RawGnssShadow sh(cfg);
        sh.onSeed(padSeed(kT0));
        int64_t t = feedFlight(sh, kT0, 12.0, 0.0f, gnss_raw::kShadowLevel, true);
        const double h_pad = sh.filter().lla()[2];
        double h = 0.0, v = 0.0;                  // the truth, up
        for (int64_t k = t; k < t + 4000000; k += 5000)
        {
            const float a_up = k - t < 1000000 ? 60.0f : 0.0f;
            ShadowImu s = still(k, gnss_raw::kShadowInFlight);
            s.acc[2] -= a_up;
            if ((k - t) % 100000 == 0)
            {
                const double q = 0.5 * 1.225 * v * v;
                sh.onBaro(gnss_raw::ShadowBaro{static_cast<uint32_t>(k),
                                               static_cast<float>(h - 1.4 * cfg.baro_port_k * q)});
            }
            sh.onImu(s, k);
            h += v * 0.005 + 0.5 * a_up * 0.005 * 0.005;
            v += a_up * 0.005;
        }
        t += 4000000;
        err[hold] = sh.filter().lla()[2] - h_pad - h;
        if (!hold)
        {
            EXPECT_EQ(sh.stats().baro_held, 0u);
            continue;
        }
        EXPECT_GE(sh.stats().baro_held, 35u);
        // and for no longer than 30 s after launch: a wrong speed cannot
        // keep it out for good
        const uint32_t fused = sh.stats().baro_fused;
        t = feedFlight(sh, t, 27.0, 0.0f, gnss_raw::kShadowInFlight, true);
        EXPECT_GE(sh.stats().baro_fused - fused, 5u);
    }
    std::printf("height at 210 m up, 60 m/s: %+.1f m with the barometer held out, %+.1f m without\n",
                err[1], err[0]);
    EXPECT_LT(std::fabs(err[1]), 2.0);
    // pulled toward the port's reading: modestly here, the IMU perfect and the
    // filter's vertical tight off the pad (with the rig's IMU, 21 m in r3)
    EXPECT_LT(err[0], -3.0);
}

TEST(RawGnssShadow, DriftingWithTheWindKeepsTheBarometer)
{
    // Under a chute the rocket drifts with the wind, which moves no air past
    // the port: the vertical speed decides, not the ground speed
    RawGnssShadow sh(primaryCfg());
    sh.onSeed(padSeed(kT0));
    int64_t t = feedFlight(sh, kT0, 12.0, 0.0f, gnss_raw::kShadowLevel, true);
    const uint32_t fused = sh.stats().baro_fused;
    // 3 s at 10 m/s^2 sideways: 30 m/s over the ground, level
    for (int64_t k = t; k < t + 3000000; k += 5000)
    {
        ShadowImu s = still(k, gnss_raw::kShadowInFlight);
        s.acc[0] += 10.0f;
        if ((k - t) % 100000 == 0)
            sh.onBaro(gnss_raw::ShadowBaro{static_cast<uint32_t>(k), 0.0f});
        sh.onImu(s, k);
    }
    const float* v = sh.filter().velNed();
    EXPECT_GT(std::hypot(v[0], v[1]), 25.0f);
    EXPECT_EQ(sh.stats().baro_held, 0u);
    EXPECT_GE(sh.stats().baro_fused - fused, 25u);
}

TEST(RawGnssShadow, OnThePadWithNoSatellitesZeroVelocityHoldsItStill)
{
    // On the rig (r4) the receiver lost the sky 87 s before launch and the
    // navigation filter, with nothing to hold it, drifted 150 m on the pad.
    // Here a filter that has come to think it moves 1 m/s sideways, then 60 s
    // still on the pad with no satellites.
    double drift[2] = {}, speed[2] = {};
    for (int zupt = 0; zupt < 2; ++zupt)
    {
        RawGnssShadow::Config cfg = primaryCfg();
        cfg.pad_zupt = zupt != 0;
        RawGnssShadow sh(cfg);
        sh.onSeed(padSeed(kT0));
        int64_t t = feedFlight(sh, kT0, 5.0, 0.0f, gnss_raw::kShadowLevel, false);
        // 0.5 s at 2 m/s^2 sideways (no levelling on it): 1 m/s
        for (int64_t k = t; k < t + 500000; k += 5000)
        {
            ShadowImu s = still(k, 0);
            s.acc[0] += 2.0f;
            sh.onImu(s, k);
        }
        t += 500000;
        const double lat0 = sh.filter().lla()[0], lon0 = sh.filter().lla()[1];
        t = feedFlight(sh, t, 60.0, 0.0f, gnss_raw::kShadowLevel, false);
        const double dn = (sh.filter().lla()[0] - lat0) * 6.371e6;
        const double de = (sh.filter().lla()[1] - lon0) * 6.371e6 * std::cos(lat0);
        drift[zupt] = std::hypot(dn, de);
        speed[zupt] = std::hypot(sh.filter().velNed()[0], sh.filter().velNed()[1]);
        if (zupt)
            EXPECT_GE(sh.stats().pad_zupts, 600u);
        else
            EXPECT_EQ(sh.stats().pad_zupts, 0u);
        EXPECT_EQ(sh.stats().zupts, 0u);          // the landed count is the landing's
    }
    EXPECT_GT(drift[0], 40.0);
    EXPECT_GT(speed[0], 0.5);
    EXPECT_LT(drift[1], 2.0);
    EXPECT_LT(speed[1], 0.05);
}

TEST(RawGnssShadow, ThrustBeforeTheLaunchVerdictIsNotLevelledOn)
{
    // Between ignition and the launch detector's verdict the vehicle is "on
    // the pad" (no kShadowInFlight) with 4 g of thrust: levelling on that
    // would take the thrust for gravity (a Beetle sim, 2026-10-13)
    RawGnssShadow sh(primaryCfg());
    sh.onSeed(padSeed(kT0));
    feedStill(sh, kT0, kT0 + 1000000);
    const uint32_t lv = sh.stats().levels;
    const float ab0 = sh.filter().accelBias()[2];
    feedFlight(sh, kT0 + 1000000, 0.3, 3.0f * float(kG), gnss_raw::kShadowLevel, false);   // 4 g, still "on the pad"
    EXPECT_EQ(sh.stats().levels, lv);
    EXPECT_GE(sh.stats().levels_held, 25u);
    EXPECT_NEAR(sh.filter().accelBias()[2], ab0, 1e-3f);
    // straight up, as the thrust pushes it: no horizontal velocity from it
    EXPECT_NEAR(sh.filter().velNed()[0], 0.0f, 0.05f);
    EXPECT_NEAR(sh.filter().velNed()[1], 0.0f, 0.05f);
    EXPECT_LT(sh.filter().velNed()[2], -8.0f);
}

// ------------------- the firmware's own sim through the navigation filter

namespace
{

// TR_Sensor_Collector_Sim's physics and sensors (its defaults: 0.5 kg, 20 N
// for 2 s, 5 m/s under the chute; 85 degrees on the rail, a gravity turn in
// coast, a pendulum under the chute), in its FLU body frame, heading north.
struct FwSim
{
    double alt = 0, vel = 0, acc = 0, pitch = 85.0 * kPi / 180.0, rate = 0;
    int phase = 0;                 // 0 pad, 1 powered, 2 coasting, 3 descent, 4 landed
    double t_phase = 0;
    static constexpr double kM = 0.5, kThrust = 20.0, kBurn = 2.0, kDescent = 5.0, kG0 = 9.80665;
    static double rho(double h) { const double x = 1.0 - 2.2558e-5 * h; return x > 0 ? 1.225 * std::pow(x, 4.2559) : 0; }
    double drag() const
    {
        const double v = std::fabs(vel), f = 0.5 * 0.5 * 0.0019635 * rho(alt) * v * v;
        return (vel > 0 ? f : -f) / kM;
    }
    void step(double dt)
    {
        t_phase += dt;
        switch (phase)
        {
        case 0:
            if (t_phase >= 5.0) { phase = 1; t_phase = 0; }
            break;
        case 1:
        {
            const double net = kThrust / kM - kG0 - drag();
            vel += net * dt; alt += vel * dt; acc = net;
            if (t_phase >= kBurn) { phase = 2; t_phase = 0; }
            break;
        }
        case 2:
        {
            const double net = -kG0 - drag();
            vel += net * dt; alt += vel * dt; acc = net;
            const double m_grav = kM * kG0 * std::sin(pitch) * (-0.075);
            double m_damp = 0;
            if (std::fabs(vel) > 1.0)
            {
                const double q = 0.5 * rho(alt) * vel * vel;
                m_damp = -20.0 * q * 0.0019635 / std::fabs(vel) * rate;
            }
            rate += (m_grav + m_damp) / 0.02 * dt;
            pitch += rate * dt;
            if (vel <= 0) { vel = 0; phase = 3; t_phase = 0; }
            break;
        }
        case 3:
        {
            vel = -kDescent; alt += vel * dt; acc = 0;
            const double m_pend = -kM * kG0 * std::sin(pitch) * 0.1, m_damp = -0.5 * rate;
            rate += (m_pend + m_damp) / 0.02 * dt;
            pitch += rate * dt;
            if (alt <= 0) { alt = 0; vel = 0; phase = 4; t_phase = 0; }
            break;
        }
        default:
            vel = 0; acc = 0; rate = 0;
            break;
        }
    }
    // FLU -> FRD
    void imu(float f_frd[3], float w_frd[3]) const
    {
        const double sf = acc + kG0;
        const double f[3] = {sf * std::sin(pitch), 0.0, sf * std::cos(pitch)};   // upInBody
        const double w[3] = {0.0, -rate, 0.0};                                  // gyroInBody
        f_frd[0] = float(f[0]); f_frd[1] = float(-f[1]); f_frd[2] = float(-f[2]);
        w_frd[0] = float(w[0]); w_frd[1] = float(-w[1]); w_frd[2] = float(-w[2]);
    }
    void mag(float m_frd[3]) const
    {
        const double sp = std::sin(pitch), cp = std::cos(pitch);           // fieldInBody (22, 5, 42 uT NED)
        const double m[3] = {22.0 * cp - 42.0 * sp, -5.0, -22.0 * sp - 42.0 * cp};
        m_frd[0] = float(m[0]); m_frd[1] = float(-m[1]); m_frd[2] = float(-m[2]);
    }
};

}  // namespace

namespace
{

struct SimScore
{
    double max_h_err = 0, max_horiz = 0, t_land = -1;
    float  v_end[3] = {};
    bool   running = false;
};

// A sim on the Beetle drops the receiver's raw output: the navigation filter
// flies it on the IMU, the barometer and the magnetometer alone. The sim's
// world is vertical -- no horizontal motion at all -- so what the filter puts
// there is its own error. gyro_bias_dps: on every axis of the IMU it sees.
SimScore flyFirmwareSim(double gyro_bias_dps)
{
    SimScore sc;
    RawGnssShadow sh(primaryCfg());
    FwSim sim;
    ShadowSeed seed = padSeed(kT0, 38.0 * kPi / 180.0, -122.0 * kPi / 180.0, 0.0);
    {
        float f[3], w[3];
        sim.imu(f, w);
        // main.cpp's start: the accelerometer's tilt, a heading of north (a guess)
        const double pitch = std::atan2(-double(f[0]), -double(f[2])) ;   // nose up: f along -z body
        (void)pitch;
        quatYawPitch(0.0, 85.0 * kPi / 180.0, seed.quat);
    }
    seed.declination_rad = float(std::atan2(5.0, 22.0));
    seed.field_ut = 47.7f;
    seed.have_declination = 1;
    const double h0 = 0.0;
    double max_h_err = 0, max_horiz = 0, t_land = -1;
    bool launched = false, apogee = false;
    int64_t t = kT0;
    double acc_f[3] = {}, acc_w[3] = {};
    int n_acc = 0;
    for (int k = 0; k < 140000; ++k)            // 140 s at 1 kHz
    {
        sim.step(0.001);
        float f[3], w[3];
        sim.imu(f, w);
        for (int i = 0; i < 3; ++i)
            w[i] += float(gyro_bias_dps * kPi / 180.0);
        for (int i = 0; i < 3; ++i) { acc_f[i] += f[i]; acc_w[i] += w[i]; }
        n_acc++;
        t += 1000;
        if (sim.phase >= 1 && sim.t_phase > 0.2) launched = launched || sim.phase >= 1;
        if (sim.phase == 3) apogee = true;
        if (sim.phase == 4 && t_land < 0) t_land = (t - kT0) * 1e-6;
        const bool landed = t_land > 0 && (t - kT0) * 1e-6 > t_land + 1.0;
        const bool in_flight = launched && !(t_land > 0 && (t - kT0) * 1e-6 > t_land + 2.0);
        if (n_acc == 4)                          // the flight filter's ticks, 250 Hz
        {
            ShadowImu s{};
            s.time_us = static_cast<uint32_t>(t);
            for (int i = 0; i < 3; ++i) { s.acc[i] = float(acc_f[i] / 4); s.gyro[i] = float(acc_w[i] / 4); acc_f[i] = acc_w[i] = 0; }
            n_acc = 0;
            s.flags = static_cast<uint8_t>((!in_flight || apogee ? gnss_raw::kShadowLevel : 0)
                                           | (in_flight ? gnss_raw::kShadowInFlight : 0)
                                           | (landed ? gnss_raw::kShadowLanded : 0));
            if ((t - kT0) % 20000 == 0)          // barometer and magnetometer at 50 Hz
            {
                sh.onBaro(gnss_raw::ShadowBaro{static_cast<uint32_t>(t), float(sim.alt)});
                gnss_raw::ShadowMag m{};
                m.time_us = static_cast<uint32_t>(t);
                sim.mag(m.mag);
                sh.onMag(m);
            }
            if (!in_flight && (t - kT0) % 1000000 == 0)
            {
                seed.time_us = static_cast<uint32_t>(t);
                seed.on_pad = launched ? 0 : 1;
                sh.onSeed(seed);
            }
            sh.onImu(s, t);
            if (std::getenv("SIM_TRACE") && sh.running() && (t - kT0) % 500000 == 0)
            {
                const float* q = sh.filter().quat();
                const float* v = sh.filter().velNed();
                const double nose_d = 2 * (q[1] * q[3] - q[0] * q[2]);
                const double nose_e = 2 * (q[1] * q[2] + q[0] * q[3]);
                const double rz_e = 2 * (q[2] * q[3] - q[0] * q[1]);       // body FRD z in NED, east part
                std::printf("t %6.2f ph %d sim pitch %6.1f filt pitch %6.1f nose_e %+.3f z_e %+.3f v %+6.2f %+6.2f %+6.2f "
                            "ab %+.3f %+.3f %+.3f wb %+.4f %+.4f %+.4f\n",
                            (t - kT0) * 1e-6, sim.phase, sim.pitch * 180 / kPi, std::asin(-nose_d) * 180 / kPi, nose_e, rz_e,
                            v[0], v[1], v[2], sh.filter().accelBias()[0], sh.filter().accelBias()[1],
                            sh.filter().accelBias()[2], sh.filter().gyroBias()[0], sh.filter().gyroBias()[1],
                            sh.filter().gyroBias()[2]);
            }
            if (sh.running() && launched)
            {
                const double h_err = (sh.filter().lla()[2] - h0) - sim.alt;
                const double dn = (sh.filter().lla()[0] - seed.lla[0]) * 6.371e6;
                const double de = (sh.filter().lla()[1] - seed.lla[1]) * 6.371e6 * std::cos(seed.lla[0]);
                max_h_err = std::max(max_h_err, std::fabs(h_err));
                max_horiz = std::max(max_horiz, std::hypot(dn, de));
            }
        }
    }
    sc.running = sh.running();
    sc.t_land = t_land;
    sc.max_h_err = max_h_err;
    sc.max_horiz = max_horiz;
    for (int i = 0; i < 3; ++i)
        sc.v_end[i] = sh.filter().velNed()[i];
    std::printf("sim flight (gyro bias %.2f dps): landed %.1f s; max height error %.1f m, max horizontal %.1f m; "
                "baro fused %u (held at speed %u), levelled %u (held %u), mag %u\n",
                gyro_bias_dps, t_land, max_h_err, max_horiz, sh.stats().baro_fused, sh.stats().baro_held,
                sh.stats().levels, sh.stats().levels_held, sh.stats().mag_fused);
    return sc;
}

}  // namespace

TEST(RawGnssShadow, TheNavigationFilterFliesTheFirmwaresSim)
{
    const SimScore sc = flyFirmwareSim(0.0);
    ASSERT_TRUE(sc.running);
    ASSERT_GT(sc.t_land, 0.0);
    EXPECT_LT(sc.max_h_err, 5.0);
    EXPECT_LT(sc.max_horiz, 5.0);
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(sc.v_end[i], 0.0f, 0.3f);
}

TEST(RawGnssShadow, UnderAChuteWithNoSatellitesLevellingHoldsAGyroBias)
{
    // Half a degree a second on every axis -- a sim's synthetic counts less
    // the board's stored gyro calibration did that on the Beetle, and put the
    // navigation filter 1.5 km east in 40 s with nothing to level on
    const SimScore sc = flyFirmwareSim(0.4);
    ASSERT_TRUE(sc.running);
    EXPECT_LT(sc.max_h_err, 10.0);
    EXPECT_LT(sc.max_horiz, 150.0);
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(sc.v_end[i], 0.0f, 0.5f);
}

// ------------------------------------------- the pad context (PAD_CONTEXT_MSG)

namespace
{

// A broadcast ephemeris payload for msg and sat, len bytes, filled with fill.
std::vector<uint8_t> ephPayload(uint16_t msg, uint8_t sat, size_t len = 61, uint8_t fill = 0x5A)
{
    std::vector<uint8_t> p(len, fill);
    p[0] = static_cast<uint8_t>(msg >> 4);
    p[1] = static_cast<uint8_t>(((msg & 0x0F) << 4) | (sat >> 2));
    p[2] = static_cast<uint8_t>(((sat & 0x03) << 6) | (fill & 0x3F));
    return p;
}

// The message and satellite of a framed ephemeris (6-bit satellite IDs).
std::pair<uint16_t, uint8_t> ephKey(const uint8_t* frame)
{
    const uint16_t msg = static_cast<uint16_t>((frame[3] << 4) | (frame[4] >> 4));
    return {msg, static_cast<uint8_t>(((frame[4] & 0x0F) << 2) | (frame[5] >> 6))};
}

struct PadContextRec
{
    PadContextHeader h;
    std::vector<uint8_t> body;
};

// Every record the shadow writes at now_us, until it has none.
std::vector<PadContextRec> drainPadContext(RawGnssShadow& sh, uint32_t now_us)
{
    std::vector<PadContextRec> out;
    uint8_t buf[MAX_PAYLOAD];
    size_t len = 0;
    while (sh.takePadContext(buf, sizeof(buf), len, now_us))
    {
        PadContextRec r;
        EXPECT_GE(len, sizeof(PadContextHeader));
        EXPECT_LE(len, sizeof(buf));
        std::memcpy(&r.h, buf, sizeof(r.h));
        r.body.assign(buf + sizeof(r.h), buf + len);
        out.push_back(r);
        if (out.size() > 200)
            break;
    }
    return out;
}

}  // namespace

TEST(EphemerisFrames, EachComesBackAsTheFrameTheReceiverSent)
{
    // The Beetle's own ephemerides (GPS, Galileo, BeiDou): framed again they
    // are the receiver's bytes, and the flight parser takes every one.
    gnss_raw::EphemerisFrames ef;
    for (int i = 0; i < kGoldenRmEphN; ++i)
        ASSERT_TRUE(ef.add(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len)));
    ASSERT_EQ(ef.count(), kGoldenRmEphN);
    lc86::Lc86Parser parser;
    int rtcm = 0, bad = 0;
    for (int i = 0; i < kGoldenRmEphN; ++i)
    {
        uint8_t out[PCX_EPH_MAX_FRAME];
        const size_t n = ef.frame(i, out, sizeof(out));
        const std::vector<uint8_t> want = frame(kGoldenRmEph[i]);
        ASSERT_EQ(n, want.size());
        EXPECT_TRUE(std::equal(want.begin(), want.end(), out)) << "ephemeris " << i;
        EXPECT_EQ(gnss_raw::EphemerisFrames::crc24q(out, n - 3), rtcm3::crc24q(out, n - 3));
        for (size_t k = 0; k < n; ++k)
        {
            const lc86::Event e = parser.feed(out[k]);
            rtcm += e == lc86::Event::RTCM;
            bad += e == lc86::Event::BAD;
        }
        EXPECT_EQ(ef.frame(i, out, n - 1), 0u);     // one byte short: nothing
    }
    EXPECT_EQ(rtcm, kGoldenRmEphN);
    EXPECT_EQ(bad, 0);
    EXPECT_EQ(parser.rtcmEphFrames(), static_cast<uint32_t>(kGoldenRmEphN));
}

TEST(EphemerisFrames, KeepsEachSatellitesLatestAndDropsTheOldestWhenFull)
{
    gnss_raw::EphemerisFrames ef;
    uint8_t out[PCX_EPH_MAX_FRAME];
    // a new one for a satellite replaces its last
    auto a = ephPayload(1019, 5, 61, 0x11), b = ephPayload(1019, 5, 61, 0x22);
    ASSERT_TRUE(ef.add(a.data(), a.size()));
    ASSERT_TRUE(ef.add(b.data(), b.size()));
    ASSERT_EQ(ef.count(), 1);
    ASSERT_EQ(ef.frame(0, out, sizeof(out)), 67u);
    EXPECT_EQ(out[3 + 10], 0x22);
    // the same satellite in another message is another
    auto c = ephPayload(1046, 5, 63);
    ASSERT_TRUE(ef.add(c.data(), c.size()));
    EXPECT_EQ(ef.count(), 2);
    // full: the one added longest ago goes, not the one refreshed
    for (uint8_t sat = 6; ef.count() < gnss_raw::EphemerisFrames::kMax; ++sat)
    {
        auto p = ephPayload(1019, sat);
        ASSERT_TRUE(ef.add(p.data(), p.size()));
    }
    ASSERT_TRUE(ef.add(b.data(), b.size()));           // 1019 sat 5 again: now the newest
    auto d = ephPayload(1042, 1, 64);
    ASSERT_TRUE(ef.add(d.data(), d.size()));
    EXPECT_EQ(ef.count(), gnss_raw::EphemerisFrames::kMax);
    bool has_1019_5 = false, has_1046_5 = false, has_1042_1 = false;
    for (int i = 0; i < ef.count(); ++i)
    {
        ASSERT_GT(ef.frame(i, out, sizeof(out)), 0u);
        const auto k = ephKey(out);
        has_1019_5 |= k == std::make_pair<uint16_t, uint8_t>(1019, 5);
        has_1046_5 |= k == std::make_pair<uint16_t, uint8_t>(1046, 5);
        has_1042_1 |= k == std::make_pair<uint16_t, uint8_t>(1042, 1);
    }
    EXPECT_TRUE(has_1019_5);
    EXPECT_FALSE(has_1046_5);                          // the oldest
    EXPECT_TRUE(has_1042_1);
    // too short or too long to be one: refused
    EXPECT_FALSE(ef.add(a.data(), 2));
    auto big = ephPayload(1042, 2, gnss_raw::EphemerisFrames::kMaxPayload + 1);
    EXPECT_FALSE(ef.add(big.data(), big.size()));
    EXPECT_FALSE(ef.add(nullptr, 61));
}

TEST(RawGnssShadow, WritesThePadContextOnceFromSixSecondsAfterLaunch)
{
    // The Beetle's ephemerides and two epochs on a surveyed pad with a
    // broadcast ionosphere; then the launch. Everything the filter had at the
    // launch tick comes back, as it was, from 6 s later: its state, its
    // covariance in four parts, the pad survey, then each ephemeris framed.
    // (The survey takes each satellite at once here: two epochs are all the
    // fixture has.)
    RawGnssShadow::Config cfg = primaryCfg();
    cfg.survey.settle_s = 0.0;
    cfg.survey.min_span_s = 0.0;
    RawGnssShadow sh(cfg);
    for (int i = 0; i < kGoldenRmEphN; ++i)
        sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
    sh.setIono(goldenIono(kGoldenDay));
    auto epochs = goldenEpochs();
    ShadowSeed seed = goldenSeed(kT0, true);
    seed.declination_rad = 0.15f;
    seed.field_ut = 48.0f;
    seed.have_declination = 1;
    sh.onSeed(seed);
    for (size_t k = 0; k < epochs.size(); ++k)
        runGolden(sh, epochs, k);
    ASSERT_TRUE(sh.running());
    const int64_t t_edge = epochs.back().mcu_us + 205000;
    feedStill(sh, epochs.back().mcu_us + 205000, t_edge);
    uint8_t buf[MAX_PAYLOAD];
    size_t len = 0;
    EXPECT_FALSE(sh.takePadContext(buf, sizeof(buf), len, static_cast<uint32_t>(t_edge + 10000000)));
    EXPECT_EQ(sh.stats().pad_contexts, 0u);

    // the filter as it is at the launch tick, before the tick steps it
    const auto& f = sh.filter();
    const double lla[3] = {f.lla()[0], f.lla()[1], f.lla()[2]};
    float q[4];
    for (int i = 0; i < 4; ++i)
        q[i] = f.quat()[i];
    std::vector<float> tri;
    for (int i = 0; i < PCX_COV_N; ++i)
        for (int j = i; j < PCX_COV_N; ++j)
            tri.push_back(f.P(i, j));
    std::vector<PadSurveyEntry> corrections;
    for (int sys = 0; sys <= 2; ++sys)
        for (int prn = 1; prn < 64; ++prn)
        {
            double c = 0.0;
            if (sh.survey().correction(sys, prn, c))
                corrections.push_back({static_cast<uint8_t>(sys), static_cast<uint8_t>(prn), static_cast<float>(c)});
        }
    ASSERT_GE(corrections.size(), 8u);
    const uint8_t tracked = sh.survey().summary().tracked;

    int64_t t = feedFlight(sh, t_edge, 2.0, 20.0f, gnss_raw::kShadowInFlight, false);
    EXPECT_EQ(sh.stats().pad_contexts, 1u);
    // an ephemeris for a new satellite in flight is in the flight's own log:
    // not one of these
    const auto late = ephPayload(1019, 31);
    sh.onEphemeris(late.data(), late.size());
    // nothing until 6 s after the launch tick
    EXPECT_FALSE(sh.takePadContext(buf, sizeof(buf), len, static_cast<uint32_t>(t_edge + 5999000)));
    t = feedFlight(sh, t, 4.5, 0.0f, gnss_raw::kShadowInFlight, false);
    const uint32_t now = static_cast<uint32_t>(t);
    // a buffer too small for the next record takes nothing from it
    EXPECT_FALSE(sh.takePadContext(buf, sizeof(PadContextHeader) + 10, len, now));

    const std::vector<PadContextRec> recs = drainPadContext(sh, now);
    ASSERT_EQ(recs.size(), 1u + 4u + 1u + static_cast<size_t>(kGoldenRmEphN));
    for (const auto& r : recs)
        EXPECT_EQ(r.h.time_us, now);
    // the state
    ASSERT_EQ(recs[0].h.kind, PCX_NAV);
    EXPECT_EQ(recs[0].h.parts, 1);
    ASSERT_EQ(recs[0].body.size(), sizeof(PadContextNav));
    PadContextNav nav;
    std::memcpy(&nav, recs[0].body.data(), sizeof(nav));
    EXPECT_EQ(nav.filter_time_us, static_cast<uint32_t>(t_edge));
    for (int i = 0; i < 3; ++i)
        EXPECT_EQ(nav.lla[i], lla[i]);
    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(nav.quat[i], q[i]);
    EXPECT_EQ(nav.flags, PCN_RUNNING | PCN_CLOCK | PCN_IONO | PCN_DECLINATION | PCN_PRIMARY);
    EXPECT_EQ(nav.iono_gps_day, kGoldenDay);
    EXPECT_EQ(nav.iono[0], 14);
    EXPECT_EQ(nav.iono[7], -5);
    EXPECT_EQ(nav.iono_source, sh.ionoSource());
    EXPECT_FLOAT_EQ(nav.declination_rad, 0.15f);
    EXPECT_FLOAT_EQ(nav.field_ut, 48.0f);
    // the covariance's triangle, in four parts
    std::vector<float> got;
    for (int p = 0; p < 4; ++p)
    {
        const PadContextRec& r = recs[1 + p];
        ASSERT_EQ(r.h.kind, PCX_COV);
        EXPECT_EQ(r.h.part, p);
        EXPECT_EQ(r.h.parts, 4);
        PadContextCov c{};
        ASSERT_LE(r.body.size(), sizeof(c));
        std::memcpy(&c, r.body.data(), r.body.size());
        EXPECT_EQ(c.first, p * PCX_COV_PER_PART);
        EXPECT_EQ(c.n, p < 3 ? 48 : 46);
        EXPECT_EQ(r.body.size(), padContextCovWireSize(c.n));
        for (int k = 0; k < c.n; ++k)
            got.push_back(c.p[k]);
    }
    EXPECT_EQ(got, tri);
    // the pad survey
    ASSERT_EQ(recs[5].h.kind, PCX_SURVEY);
    EXPECT_EQ(recs[5].h.parts, 1);
    PadContextSurvey sv{};
    ASSERT_LE(recs[5].body.size(), sizeof(sv));
    std::memcpy(&sv, recs[5].body.data(), recs[5].body.size());
    EXPECT_EQ(sv.tracked, tracked);
    EXPECT_EQ(recs[5].body.size(), padContextSurveyWireSize(sv.n));
    ASSERT_EQ(sv.n, corrections.size());
    for (size_t i = 0; i < corrections.size(); ++i)
    {
        EXPECT_EQ(sv.e[i].sys, corrections[i].sys);
        EXPECT_EQ(sv.e[i].prn, corrections[i].prn);
        EXPECT_EQ(sv.e[i].corr_m, corrections[i].corr_m);
    }
    // each ephemeris, as the receiver framed it
    for (int i = 0; i < kGoldenRmEphN; ++i)
    {
        const PadContextRec& r = recs[6 + i];
        ASSERT_EQ(r.h.kind, PCX_EPH);
        EXPECT_EQ(r.h.part, i);
        EXPECT_EQ(r.h.parts, kGoldenRmEphN);
        EXPECT_EQ(r.body, frame(kGoldenRmEph[i]));
    }
    // and once: nothing more this flight
    t = feedFlight(sh, t, 1.0, 0.0f, gnss_raw::kShadowInFlight, false);
    EXPECT_FALSE(sh.takePadContext(buf, sizeof(buf), len, static_cast<uint32_t>(t)));
    EXPECT_EQ(sh.stats().pad_contexts, 1u);
    // a flight after this one, back on the ground, is another
    t = feedFlight(sh, t, 1.0, 0.0f, gnss_raw::kShadowLevel, false);
    t = feedFlight(sh, t, 1.0, 20.0f, gnss_raw::kShadowInFlight, false);
    EXPECT_EQ(sh.stats().pad_contexts, 2u);
    EXPECT_FALSE(drainPadContext(sh, static_cast<uint32_t>(t + 6000000)).empty());
}

TEST(RawGnssShadow, ARestoredFlightWritesNoPadContext)
{
    // Rebooted in the air: it never saw the pad, so it has none to write.
    RawGnssShadow sh(primaryCfg());
    ShadowSeed seed = padSeed(kT0);
    seed.on_pad = 0;
    seed.restore = 1;
    sh.onSeed(seed);
    const int64_t t = feedFlight(sh, kT0, 8.0, 0.0f, gnss_raw::kShadowInFlight, false);
    ASSERT_TRUE(sh.running());
    EXPECT_EQ(sh.stats().pad_contexts, 0u);
    EXPECT_TRUE(drainPadContext(sh, static_cast<uint32_t>(t)).empty());
}

// ------------------------------------------- the GNSS task's stage (#1626)

namespace
{

// The GNSS task, as main.cpp plays it beside the filter's: each epoch seen by
// the stage and built from the filter's published state, then handed to the
// filter built; the satellites the filter found jumped go back to the stage
// first, and its counts come with each epoch.
void offerBuilt(RawGnssShadow& sh, gnss_raw::RawGnssMeasStage& stage, const TimedRawEpoch& e,
                gnss_raw::BuiltEpoch& built)
{
    gnss_raw::ShadowRelock r;
    while (sh.takeRelock(r))
        stage.markRelock(r.sys, r.prn, r.tow_s);
    gnss_raw::ShadowNav nav;
    sh.navState(nav);
    if (stage.offer(e, nav, built))
        sh.onBuilt(built);
    sh.onMeasStats(stage.stats());
}

// runGolden, the epoch built in the GNSS task.
void runGoldenBuilt(RawGnssShadow& sh, gnss_raw::RawGnssMeasStage& stage, std::vector<TimedRawEpoch>& epochs,
                    size_t k, gnss_raw::BuiltEpoch& built)
{
    const int64_t te = epochs[k].mcu_us;
    feedStill(sh, te - 1000000 + 5000, te + 200000);
    offerBuilt(sh, stage, epochs[k], built);
    feedStill(sh, te + 200000, te + 205000);
}

gnss_raw::ShadowNav goldenNav(bool running)
{
    gnss_raw::ShadowNav nav{};
    ecef_to_geo(kGoldenRmRx, &nav.lla[0], &nav.lla[1], &nav.lla[2]);
    nav.running = running ? 1 : 0;
    return nav;
}

}  // namespace

TEST(RawGnssMeasStage, BuildsATimedEpochForARunningFilterAsTheBuilderDoes)
{
    // The GNSS task's turn for each epoch: an epoch with no MCU time yet, or
    // for a filter not running, keeps its satellites' lock ages and is not
    // built; one for a running filter is built at the filter's position --
    // what RawMeasBuilder makes of it there, with no stress on the Doppler
    // (the filter adds that), each measurement with its satellite's lock age.
    gnss_raw::RawGnssMeasStage stage;
    for (int i = 0; i < kGoldenRmEphN; ++i)
        ASSERT_TRUE(stage.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len)));
    EXPECT_GT(stage.ephemerisSatellites(), 0);
    std::vector<TimedRawEpoch> epochs = goldenEpochs();
    gnss_raw::BuiltEpoch b;
    TimedRawEpoch untimed = epochs[0];
    untimed.mcu_us = 0;
    EXPECT_FALSE(stage.offer(untimed, goldenNav(true), b));
    EXPECT_EQ(stage.stats().untimed, 1u);
    EXPECT_EQ(stage.stats().last_timed, 0);
    EXPECT_FALSE(stage.offer(epochs[0], goldenNav(false), b));
    EXPECT_EQ(stage.stats().idle, 1u);
    stage.skip(epochs[0]);                    // the handoff full: seen, not built
    EXPECT_EQ(stage.stats().dropped, 1u);
    ASSERT_TRUE(stage.offer(epochs[0], goldenNav(true), b));
    EXPECT_EQ(stage.stats().epochs, 4u);
    EXPECT_EQ(stage.stats().built, 1u);
    EXPECT_EQ(stage.stats().last_timed, 1);
    EXPECT_GT(stage.stats().tracked, 0);
    EXPECT_EQ(b.mcu_us, epochs[0].mcu_us);
    EXPECT_EQ(b.tow_s, epochs[0].epoch.tow_s);
    EXPECT_EQ(b.gps_week, kGoldenWeek);
    EXPECT_EQ(b.iono_changed, 1);             // the first epoch on the pad chooses the ionosphere
    EXPECT_EQ(b.use_broadcast, 0);            //   none brought: the pad survey's
    ASSERT_GE(b.n, 7);

    gnss_raw::RawMeasBuilder ref{gnss_raw::RawMeasBuilder::Config{}};
    ref.setIono(gps_iono_t{});
    gnss_raw::RawMeas want[gnss_raw::RawEpoch::kMaxObs];
    double r[3];
    geo_to_ecef(goldenNav(true).lla[0], goldenNav(true).lla[1], goldenNav(true).lla[2], r);
    const int n = ref.build(epochs[0].epoch, stage.ephemerides(), r, 0.0, want, gnss_raw::RawEpoch::kMaxObs);
    ASSERT_EQ(b.n, n);
    for (int i = 0; i < n; ++i)
    {
        EXPECT_EQ(b.meas[i].sys, want[i].sys);
        EXPECT_EQ(b.meas[i].prn, want[i].prn);
        EXPECT_EQ(b.meas[i].pr, want[i].pr);
        EXPECT_EQ(b.meas[i].rr, want[i].rr);
        EXPECT_EQ(b.meas[i].sat_pos[0], want[i].sat_pos[0]);
        EXPECT_EQ(b.meas[i].sigma_rr, want[i].sigma_rr);
        EXPECT_EQ(b.meas[i].sigma_pr_corr, want[i].sigma_pr_corr);
        EXPECT_EQ(b.lock_age_s[i], stage.builder().relockAge(b.meas[i].sys, b.meas[i].prn, b.tow_s));
    }
}

namespace
{

struct GoldenRun
{
    double   r_end[3] = {};
    uint32_t slices = 0;           // over both epochs
    uint32_t used_pr = 0;
    uint32_t min_steps_during = 0; // the fewest steps the filter took while an epoch was worked
};

// The golden epochs through the navigation filter, each handed over 20 ms
// late: built here or in the GNSS task, at once or a 2.5 ms slice a step.
GoldenRun runGoldenNav(bool split, bool sliced)
{
    std::vector<TimedRawEpoch> epochs = goldenEpochs();
    GoldenRun out;
    out.min_steps_during = 1000000;
    RawGnssShadow::Config cfg;
    cfg.primary = true;
    cfg.min_step_s = 0.01;
    cfg.epoch_slice_us = sliced ? 2500.0 : 0.0;
    cfg.external_build = split;
    RawGnssShadow sh(cfg);
    gnss_raw::RawGnssMeasStage stage(RawGnssShadow::stageConfig(cfg));
    gnss_raw::BuiltEpoch built;
    g_fake_us = 0;
    sh.setClock(fakeClock);
    for (int i = 0; i < kGoldenRmEphN; ++i)
    {
        sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
        stage.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
    }
    sh.onSeed(goldenSeed(kT0, false));
    int64_t t = kT0;
    feedStill(sh, t, kT0 + 1000000);
    t = kT0 + 1000000;
    EXPECT_TRUE(sh.running());
    for (size_t k = 0; k < epochs.size(); ++k)
    {
        const int64_t te = epochs[k].mcu_us;
        feedStill(sh, t, te + 20000);
        t = te + 20000;
        if (split)
            offerBuilt(sh, stage, epochs[k], built);
        else
            sh.onEpoch(epochs[k]);
        const uint32_t applied = sh.stats().epochs_applied;
        const uint32_t steps0 = sh.stats().step_n;
        while (sh.stats().epochs_applied == applied && t < te + 400000)
        {
            feedStill(sh, t, t + 5000);
            t += 5000;
        }
        EXPECT_EQ(sh.stats().epochs_applied, applied + 1) << "split " << split << ", epoch " << k;
        out.min_steps_during = std::min(out.min_steps_during, sh.stats().step_n - steps0);
        out.slices += sh.stats().epoch_slices;
        out.used_pr += static_cast<uint32_t>(sh.stats().last.used_pr);
    }
    if (split)
    {
        EXPECT_EQ(sh.stats().epochs, 2u);           // the GNSS task's counts
        EXPECT_EQ(sh.stats().epochs_stale, 0u);
        EXPECT_EQ(sh.ephemerisSatellites(), stage.ephemerisSatellites());
    }
    double v[3];
    sh.filter().receiverEcef(out.r_end, v);
    return out;
}

double dist3(const double a[3], const double b[3])
{
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

}  // namespace

TEST(RawGnssShadow, BuiltInAGnssTaskTheGoldenEpochsLandWhereTheyDoBuiltHere)
{
    // The navigation filter with its epochs built in the GNSS task
    // (external_build, main.cpp) fuses what the one that builds its own does.
    // At once, the same epochs fused at the same steps: the same place, the
    // build's position (the filter's published state, a tick earlier) all
    // that differs. A 2.5 ms slice a step: the split one works each epoch in
    // fewer slices -- the build is no longer among them -- stepping all the
    // while, and ends near the other (it fuses earlier, between other steps'
    // updates, as the sliced and the at-once runs differ).
    const GoldenRun here = runGoldenNav(false, false);
    const GoldenRun task = runGoldenNav(true, false);
    EXPECT_EQ(task.used_pr, here.used_pr);
    EXPECT_GE(here.used_pr, 7u + kGoldenRmMeasN - 1);
    EXPECT_LT(dist3(here.r_end, task.r_end), 1e-3);
    EXPECT_EQ(here.slices, 2u);
    EXPECT_EQ(task.slices, 2u);

    const GoldenRun here_sliced = runGoldenNav(false, true);
    const GoldenRun task_sliced = runGoldenNav(true, true);
    EXPECT_EQ(task_sliced.used_pr, here.used_pr);
    EXPECT_LT(task_sliced.slices, here_sliced.slices);
    EXPECT_GE(task_sliced.min_steps_during, 2u);   // it stepped while it fused
    EXPECT_LT(dist3(task_sliced.r_end, here.r_end), 1.0);
    EXPECT_LT(dist3(task_sliced.r_end, here_sliced.r_end), 0.2);
}

TEST(RawGnssShadow, BuiltInAGnssTaskAJumpedSatelliteGoesBackToItsStage)
{
    // TakesASatelliteWhoseCodeJumpedAsRelocked, with the epochs built in the
    // GNSS task: the satellite the filter's check finds jumped comes back to
    // that task's stage (takeRelock), which takes it as relocked for the
    // epochs after; the filter widens it at once as before.
    std::vector<TimedRawEpoch> epochs = goldenEpochs();
    RawGnssShadow::Config cfg = everyTick();
    cfg.external_build = true;
    auto run = [&](int jump_index, RawGnssShadow& sh, gnss_raw::RawGnssMeasStage& stage) {
        std::vector<TimedRawEpoch> es = goldenEpochs();
        gnss_raw::BuiltEpoch built;
        for (int i = 0; i < kGoldenRmEphN; ++i)
        {
            sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
            stage.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
        }
        sh.onSeed(goldenSeed(kT0, false));
        feedStill(sh, kT0, kT0 + 5000);
        runGoldenBuilt(sh, stage, es, 0, built);
        if (jump_index >= 0)
            es[1].epoch.obs[jump_index].pr_m += 40.0;
        runGoldenBuilt(sh, stage, es, 1, built);
    };
    RawGnssShadow ref(cfg);
    gnss_raw::RawGnssMeasStage ref_stage(RawGnssShadow::stageConfig(cfg));
    run(-1, ref, ref_stage);
    const int base = ref.stats().last_sats_jumped;
    gnss_raw::ShadowRelock r;
    int base_marks = 0;
    while (ref.takeRelock(r))
        base_marks++;
    EXPECT_EQ(base_marks, base);
    int target = -1;
    for (int i = 0; i < epochs[1].epoch.n && target < 0; ++i)
    {
        const gnss_raw::RawObs& o = epochs[1].epoch.obs[i];
        if (!o.has_pr || o.sys > GNSS_SYS_BDS)
            continue;
        bool in_first = false;
        for (int j = 0; j < epochs[0].epoch.n; ++j)
            in_first |= epochs[0].epoch.obs[j].sys == o.sys && epochs[0].epoch.obs[j].prn == o.prn;
        if (in_first && ref_stage.builder().relockAge(o.sys, o.prn, kGoldenRmTowB) > 0.5)
            target = i;
    }
    ASSERT_GE(target, 0);
    RawGnssShadow sh(cfg);
    gnss_raw::RawGnssMeasStage stage(RawGnssShadow::stageConfig(cfg));
    run(target, sh, stage);
    const gnss_raw::RawObs& o = epochs[1].epoch.obs[target];
    EXPECT_EQ(sh.stats().last_sats_jumped, base + 1);
    EXPECT_GT(stage.builder().relockAge(o.sys, o.prn, kGoldenRmTowB), 0.5);   // not yet told
    bool marked = false;
    while (sh.takeRelock(r))
    {
        marked |= r.sys == o.sys && r.prn == o.prn && r.tow_s == kGoldenRmTowB;
        stage.markRelock(r.sys, r.prn, r.tow_s);
    }
    EXPECT_TRUE(marked);
    EXPECT_EQ(stage.builder().relockAge(o.sys, o.prn, kGoldenRmTowB), 0.0);
    EXPECT_EQ(sh.builder().relockAge(o.sys, o.prn, kGoldenRmTowB), 0.0)
        << "the shadow's own stage builds nothing here";
}

TEST(RawGnssShadow, ABuiltEpochIsHeldAndDroppedAsARawEpochIs)
{
    // Built in the GNSS task, an epoch waits for the IMU to pass its instant
    // as a raw one does: one for a filter not running, one the IMU passed
    // more than max_epoch_age_s ago, and the oldest of three waiting are
    // dropped as stale. And each path takes only its own kind of epoch.
    RawGnssShadow::Config cfg = everyTick();
    cfg.external_build = true;
    RawGnssShadow sh(cfg);
    gnss_raw::BuiltEpoch b;                   // no satellites: a job of a check, the clock and the close
    b.mcu_us = kT0 + 100000;
    sh.onBuilt(b);
    EXPECT_EQ(sh.stats().epochs_stale, 1u);   // not running
    sh.onSeed(padSeed(kT0));
    feedStill(sh, kT0, kT0 + 50000);
    ASSERT_TRUE(sh.running());
    TimedRawEpoch raw;
    raw.mcu_us = kT0 + 60000;
    sh.onEpoch(raw);                          // a raw epoch: not this path's
    EXPECT_EQ(sh.stats().epochs, 0u);
    // the IMU is at kT0 + 45 ms: an epoch a second old is stale when taken
    b.mcu_us = kT0 + 45000 - 1000000;
    sh.onBuilt(b);
    feedStill(sh, kT0 + 50000, kT0 + 55000);
    EXPECT_EQ(sh.stats().epochs_stale, 2u);
    EXPECT_EQ(sh.stats().epochs_applied, 0u);
    // three ahead of the IMU: the oldest goes, the two after are applied
    for (int k = 0; k < 3; ++k)
    {
        b.mcu_us = kT0 + 100000 + k * 10000;
        sh.onBuilt(b);
    }
    EXPECT_EQ(sh.stats().epochs_stale, 3u);
    feedStill(sh, kT0 + 55000, kT0 + 200000);
    EXPECT_EQ(sh.stats().epochs_applied, 2u);
    EXPECT_EQ(sh.stats().last_tow_ms, 0u);

    RawGnssShadow inline_sh(everyTick());     // builds its own: takes no built epoch
    inline_sh.onSeed(padSeed(kT0));
    feedStill(inline_sh, kT0, kT0 + 50000);
    b.mcu_us = kT0 + 60000;
    inline_sh.onBuilt(b);
    feedStill(inline_sh, kT0 + 50000, kT0 + 100000);
    EXPECT_EQ(inline_sh.stats().epochs_applied, 0u);
    EXPECT_EQ(inline_sh.stats().epochs_stale, 0u);
}

TEST(RawGnssShadow, BuiltInAGnssTaskItKeepsTheEphemeridesForThePadContextOnly)
{
    // The GNSS task's stage decodes the ephemerides; the filter still takes
    // each as the receiver sent it, for the pad context, and decodes none.
    RawGnssShadow::Config cfg;
    cfg.external_build = true;
    RawGnssShadow sh(cfg);
    for (int i = 0; i < kGoldenRmEphN; ++i)
        sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
    EXPECT_EQ(sh.ephemerisFrames().count(), kGoldenRmEphN);
    EXPECT_EQ(sh.ephemerides().satellites(GNSS_SYS_GPS), 0);
    EXPECT_EQ(sh.ephemerisSatellites(), 0);
    gnss_raw::ShadowMeasStats ms{};
    ms.eph_sats = 9;
    ms.tracked = 12;
    ms.tracked_no_eph = 3;
    ms.iono_age_known = 1;
    ms.iono_age_days = -45;
    sh.onMeasStats(ms);
    EXPECT_EQ(sh.ephemerisSatellites(), 9);
    EXPECT_EQ(sh.stats().tracked_no_eph, 3);
    int age = 0;
    EXPECT_TRUE(sh.ionoAgeDays(age));
    EXPECT_EQ(age, -45);
    gnss_raw::ShadowPadStatus ps;
    sh.padStatus(ps);
    EXPECT_EQ(ps.tracked, 12);
    EXPECT_EQ(ps.no_eph, 3);
}

TEST(RawGnssShadow, ItsNavStateSaysWhetherItIsInFlight)
{
    // The GNSS task keeps its ionosphere through the flight on this flag.
    RawGnssShadow sh(everyTick());
    sh.onSeed(padSeed(kT0));
    feedStill(sh, kT0, kT0 + 20000);
    gnss_raw::ShadowNav nav;
    sh.navState(nav);
    EXPECT_EQ(nav.running, 1);
    EXPECT_EQ(nav.in_flight, 0);
    feedStill(sh, kT0 + 20000, kT0 + 40000, gnss_raw::kShadowLevel | gnss_raw::kShadowInFlight);
    sh.navState(nav);
    EXPECT_EQ(nav.in_flight, 1);
}

// ------------------------------------------- the heading from thrust (HeadingFit)

namespace
{

struct Mat3
{
    double m[3][3];
};

Mat3 mat3Mul(const Mat3& a, const Mat3& b)
{
    Mat3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
}

void mat3Vec(const Mat3& a, const double v[3], double o[3])
{
    for (int i = 0; i < 3; ++i)
        o[i] = a.m[i][0] * v[0] + a.m[i][1] * v[1] + a.m[i][2] * v[2];
}

Mat3 rotAbout(const double axis[3], double a)
{
    const double n = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    const double k[3] = {axis[0] / n, axis[1] / n, axis[2] / n};
    const double K[3][3] = {{0, -k[2], k[1]}, {k[2], 0, -k[0]}, {-k[1], k[0], 0}};
    Mat3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            double kk = 0.0;
            for (int l = 0; l < 3; ++l)
                kk += K[i][l] * K[l][j];
            r.m[i][j] = (i == j ? 1.0 : 0.0) + std::sin(a) * K[i][j] + (1.0 - std::cos(a)) * kk;
        }
    return r;
}

// A nose-up body leaning lean_deg toward azimuth toward_deg, rolled about
// its nose: body -> NED
Mat3 leaningBody(double lean_deg, double toward_deg, double roll_deg)
{
    const double z[3] = {0, 0, 1}, y[3] = {0, 1, 0}, up[3] = {0, 0, -1.0};
    double nose[3];
    mat3Vec(mat3Mul(rotAbout(z, toward_deg * kPi / 180.0), rotAbout(y, lean_deg * kPi / 180.0)), up, nose);
    Mat3 C{};                                    // body x up
    C.m[0][2] = 1;
    C.m[1][1] = 1;
    C.m[2][0] = -1;
    const double cx[3] = {C.m[0][0], C.m[1][0], C.m[2][0]};
    const double ax[3] = {cx[1] * nose[2] - cx[2] * nose[1], cx[2] * nose[0] - cx[0] * nose[2],
                          cx[0] * nose[1] - cx[1] * nose[0]};
    const double an = std::sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]);
    if (an > 1e-12)
        C = mat3Mul(rotAbout(ax, std::asin(an)), C);
    return mat3Mul(rotAbout(nose, roll_deg * kPi / 180.0), C);
}

// body -> NED as a scalar-first quaternion (Quat2DCM of it is NED -> body)
void quatOf(const Mat3& Cm, float q[4])
{
    const double (&R)[3][3] = Cm.m;
    const double t = R[0][0] + R[1][1] + R[2][2];
    double w, x, y, z;
    if (t > 0.0)
    {
        w = 0.5 * std::sqrt(1.0 + t);
        x = (R[2][1] - R[1][2]) / (4 * w);
        y = (R[0][2] - R[2][0]) / (4 * w);
        z = (R[1][0] - R[0][1]) / (4 * w);
    }
    else if (R[0][0] >= R[1][1] && R[0][0] >= R[2][2])
    {
        x = 0.5 * std::sqrt(1.0 + 2 * R[0][0] - t);
        w = (R[2][1] - R[1][2]) / (4 * x);
        y = (R[0][1] + R[1][0]) / (4 * x);
        z = (R[0][2] + R[2][0]) / (4 * x);
    }
    else if (R[1][1] >= R[2][2])
    {
        y = 0.5 * std::sqrt(1.0 + 2 * R[1][1] - t);
        w = (R[0][2] - R[2][0]) / (4 * y);
        x = (R[0][1] + R[1][0]) / (4 * y);
        z = (R[1][2] + R[2][1]) / (4 * y);
    }
    else
    {
        z = 0.5 * std::sqrt(1.0 + 2 * R[2][2] - t);
        w = (R[1][0] - R[0][1]) / (4 * z);
        x = (R[0][2] + R[2][0]) / (4 * z);
        y = (R[1][2] + R[2][1]) / (4 * z);
    }
    const double sgn = w < 0 ? -1.0 : 1.0;
    q[0] = float(sgn * w);
    q[1] = float(sgn * x);
    q[2] = float(sgn * y);
    q[3] = float(sgn * z);
}

// The sky the Beetle's own ephemerides give at the pad of its rig: each
// satellite's pseudorange and range rate as the LC86G reports them, for a
// receiver whose truth the test sets. A twin of the flight's builder finds
// each satellite's transmit state, so the shadow's builder takes them back to
// that truth (its troposphere off; the night-time ionosphere it removes, a
// few metres, the clock and position absorb). The range rate is the receiver's
// rr_lag before the tag, as the LC86G's.
struct SynthSky
{
    gnss_raw::EphemerisStore store;
    gnss_raw::RawMeasBuilder builder;
    std::mt19937 rng{11};
    double clk_m = 3000.0;        // the receiver's clock, m
    double drift_mps = 0.0;
    double sigma_pr = 1.0, sigma_rr = 0.05;

    explicit SynthSky(const gnss_raw::RawMeasBuilder::Config& c) : builder(c)
    {
        for (int i = 0; i < kGoldenRmEphN; ++i)
        {
            gps_eph_t e;
            if (gnss_raw::decodeEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len),
                                          kGoldenWeek, e))
                store.add(e);
        }
    }

    // r: the receiver at the tag (ECEF), v_lag: its velocity rr_lag before (ECEF)
    gnss_raw::RawEpoch epoch(double tow, const double r[3], const double v_lag[3])
    {
        gnss_raw::RawEpoch ep;
        ep.tow_s = tow;
        for (int i = 0; i < kGoldenRmEphN; ++i)
        {
            gps_eph_t e;
            if (!gnss_raw::decodeEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len),
                                           kGoldenWeek, e))
                continue;
            gnss_raw::RawObs& o = ep.obs[ep.n++];
            o.sys = static_cast<uint8_t>(e.sys);
            o.prn = static_cast<uint8_t>(e.prn);
            o.has_pr = o.has_rate = true;
            o.cn0_dbhz = 42.0f;
            o.pr_m = 2.2e7;
            o.rate_mps = 0.0;
            o.lock_ms = 600000;
        }
        // twice: the transmit time follows the pseudorange
        gnss_raw::RawMeas m[gnss_raw::RawEpoch::kMaxObs];
        for (int it = 0; it < 3; ++it)
        {
            gnss_raw::RawEpoch probe = ep;
            const int n = builder.build(probe, store, r, 0.0, m, gnss_raw::RawEpoch::kMaxObs);
            for (int k = 0; k < n; ++k)
                for (int j = 0; j < ep.n; ++j)
                {
                    gnss_raw::RawObs& o = ep.obs[j];
                    if (o.sys != m[k].sys || o.prn != m[k].prn)
                        continue;
                    const double corr = m[k].pr - o.pr_m;            // the builder's clock, less the ionosphere
                    o.pr_m = gnss_raw::sagnacRange(m[k], r) + clk_m - corr;
                    double u[3];
                    const double still = gnss_raw::stillRangeRate(m[k], r, u);
                    const double clk_rate = m[k].rr - o.rate_mps;     // the satellite clock's rate, m/s
                    o.rate_mps = still - (u[0] * v_lag[0] + u[1] * v_lag[1] + u[2] * v_lag[2]) + drift_mps
                               - clk_rate;
                }
        }
        // the builder drops what it cannot place (below its mask): keep what it takes
        gnss_raw::RawEpoch out;
        out.tow_s = tow;
        const int n = builder.build(ep, store, r, 0.0, m, gnss_raw::RawEpoch::kMaxObs);
        std::normal_distribution<double> npr(0.0, sigma_pr), nrr(0.0, sigma_rr);
        for (int k = 0; k < n; ++k)
            for (int j = 0; j < ep.n; ++j)
                if (ep.obs[j].sys == m[k].sys && ep.obs[j].prn == m[k].prn)
                {
                    gnss_raw::RawObs o = ep.obs[j];
                    o.pr_m += npr(rng);
                    o.rate_mps += nrr(rng);
                    out.obs[out.n++] = o;
                }
        return out;
    }
};

struct BoostRun
{
    std::vector<HeadingFitData> recs;
    float q_true[4] = {};
    float q_end[4] = {};       // the filter's attitude at the end
    RawGnssShadow::Stats st;
    int handed_at = -1;        // the record that handed over
};

// The Beetle's pad: still for 6 s, then a boost (10 g for 1.5 s) and drag to
// T+10 s on a body leaning 5 deg toward azimuth 70, rolled 25 deg; raw epochs
// once a GPS second, each delivered 0.2 s late. The navigation filter starts
// from a seed whose heading is seed_err_deg off (no magnetometer: it keeps
// it); the launch detector calls it at T+0.1 s. external: each epoch built by
// the GNSS task's stage and handed over built, as on the Beetle (#1626).
// apart, min_epochs, sure_deg: heading_handoff_apart, _min_epochs and
// _sure_deg, when not negative (else the defaults).
BoostRun flyBoost(uint8_t mode, double seed_err_deg, double bias_y = 0.0, double bias_z = 0.0, bool external = false,
                  double apart = -1.0, int min_epochs = -1, double sure_deg = -1.0)
{
    double lat, lon, h;
    ecef_to_geo(kGoldenRmRx, &lat, &lon, &h);
    double r0[3];
    geo_to_ecef(lat, lon, h, r0);
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    auto nedToEcefVec = [&](const double n[3], double o[3]) {
        const double e[3] = {n[1], n[0], -n[2]};
        o[0] = -so * e[0] - sl * co * e[1] + cl * co * e[2];
        o[1] = co * e[0] - sl * so * e[1] + cl * so * e[2];
        o[2] = cl * e[1] + sl * e[2];
    };

    RawGnssShadow::Config cfg = primaryCfg();
    cfg.heading_mode = mode;
    if (apart >= 0.0)
        cfg.heading_handoff_apart = apart;
    if (min_epochs >= 0)
        cfg.heading_handoff_min_epochs = min_epochs;
    if (sure_deg >= 0.0)
        cfg.heading_handoff_sure_deg = sure_deg;
    cfg.meas.troposphere = false;
    cfg.external_build = external;
    RawGnssShadow sh(cfg);
    gnss_raw::RawGnssMeasStage stage(RawGnssShadow::stageConfig(cfg));
    gnss_raw::BuiltEpoch built;
    for (int i = 0; i < kGoldenRmEphN; ++i)
    {
        sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
        stage.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
    }
    SynthSky sky(cfg.meas);
    sky.drift_mps = 0.0;

    const Mat3 Ct = leaningBody(5.0, 70.0, 25.0);
    const double zax[3] = {0, 0, 1};
    BoostRun run;
    quatOf(Ct, run.q_true);
    float q_seed[4];
    quatOf(mat3Mul(rotAbout(zax, seed_err_deg * kPi / 180.0), Ct), q_seed);
    ShadowSeed seed = padSeed(kT0, lat, lon, h);
    for (int i = 0; i < 4; ++i)
        seed.quat[i] = q_seed[i];
    sh.onSeed(seed);

    const double tow0 = goldenEpochs()[0].epoch.tow_s;    // the ephemerides' day
    const double t_ign = 6.0, t_det = 6.1, t_end = 16.0, dt = 0.005, rr_lag = cfg.ekf.rr_lag_s;
    double p_ned[3] = {}, v_ned[3] = {};
    std::vector<std::pair<double, double>> hist_v;            // (t, a step's start) for the lagged velocity
    std::vector<std::array<double, 3>> vel_hist;
    int next_epoch = 1;
    std::vector<TimedRawEpoch> due;                           // built at their tag, handed over 0.2 s later
    for (double t = dt; t <= t_end + 1e-9; t += dt)
    {
        // the truth: piecewise-constant specific force, the body's attitude fixed
        double f_b[3] = {0.0, 0.0, 0.0};
        if (t > t_ign)
            f_b[0] = t <= t_ign + 1.5 ? 100.0 : -3.0;
        double f_n[3], a_n[3];
        if (t > t_ign)
        {
            mat3Vec(Ct, f_b, f_n);
            for (int i = 0; i < 3; ++i)
                a_n[i] = f_n[i] + (i == 2 ? kG : 0.0);
        }
        else
        {
            a_n[0] = a_n[1] = a_n[2] = 0.0;
            const double up[3] = {0, 0, -kG};             // on the pad: gravity's reaction, in the body
            Mat3 CtT{};
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    CtT.m[i][j] = Ct.m[j][i];
            mat3Vec(CtT, up, f_b);
        }
        for (int i = 0; i < 3; ++i)
        {
            p_ned[i] += v_ned[i] * dt + 0.5 * a_n[i] * dt * dt;
            v_ned[i] += a_n[i] * dt;
        }
        vel_hist.push_back({v_ned[0], v_ned[1], v_ned[2]});
        ShadowImu s{};
        s.time_us = static_cast<uint32_t>(kT0 + static_cast<int64_t>(std::llround(t * 1e6)));
        s.acc[0] = float(f_b[0]);
        s.acc[1] = float(f_b[1] + bias_y);
        s.acc[2] = float(f_b[2] + bias_z);
        s.flags = t < t_ign ? gnss_raw::kShadowLevel : (t >= t_det ? gnss_raw::kShadowInFlight : 0);
        const int64_t t64 = kT0 + static_cast<int64_t>(std::llround(t * 1e6));
        sh.onImu(s, t64);
        // an epoch at each whole GPS second, built where the receiver truly is
        if (t + 1e-9 >= next_epoch)
        {
            const int lag_steps = static_cast<int>(std::lround(rr_lag / dt));
            const auto& vl = vel_hist[vel_hist.size() > static_cast<size_t>(lag_steps)
                                          ? vel_hist.size() - 1 - lag_steps : 0];
            double dr[3], r[3], vlag_e[3];
            nedToEcefVec(p_ned, dr);
            nedToEcefVec(vl.data(), vlag_e);
            for (int i = 0; i < 3; ++i)
                r[i] = r0[i] + dr[i];
            TimedRawEpoch e;
            e.epoch = sky.epoch(tow0 + next_epoch, r, vlag_e);
            e.mcu_us = t64;
            e.gps_week = kGoldenWeek;
            due.push_back(e);
            next_epoch++;
        }
        // handed over 0.2 s after its tag
        while (!due.empty() && t64 - due.front().mcu_us >= 200000)
        {
            if (external)
                offerBuilt(sh, stage, due.front(), built);
            else
                sh.onEpoch(due.front());
            due.erase(due.begin());
        }
        HeadingFitData d;
        while (sh.takeHeadingLog(d))
        {
            if (d.flags & HFT_HANDED_NOW)
                run.handed_at = static_cast<int>(run.recs.size());
            run.recs.push_back(d);
        }
    }
    for (int i = 0; i < 4; ++i)
        run.q_end[i] = sh.filter().quat()[i];
    run.st = sh.stats();
    return run;
}

}  // namespace

TEST(RawGnssShadow, TheHeadingFitObservesWithoutActing)
{
    // Observe: the fit finds the pad's 40 deg error in the boost's first
    // epochs and logs it, the filter flies on as before (cos4 keeps a
    // vertical heading where it was)
    const BoostRun run = flyBoost(RawGnssShadow::kHeadingObserve, 40.0);
    EXPECT_EQ(run.st.heading_flights, 1u);
    EXPECT_GT(run.st.heading_seeds, 100u);
    ASSERT_GE(run.recs.size(), 5u);
    EXPECT_EQ(run.st.heading_handoffs, 0u);
    EXPECT_EQ(run.handed_at, -1);
    const HeadingFitData& last = run.recs.back();
    EXPECT_TRUE(last.flags & HFT_SOLVED);
    EXPECT_FALSE(last.flags & (HFT_ACT | HFT_HANDED | HFT_HANDED_NOW));
    EXPECT_NEAR(last.psi_cdeg / 100.0, -40.0, 1.0);          // the turn back
    ASSERT_NE(last.sigma_cdeg, HFT_NO_SIGMA);
    EXPECT_LT(last.sigma_cdeg / 100.0, 3.0);
    EXPECT_NEAR(last.filt_cdeg / 100.0, 40.0, 3.0);          // the filter still 40 off
    EXPECT_LT(last.seed_ms, 120);                            // seeded at the last still step
    EXPECT_GE(last.sats, 5);
    EXPECT_NEAR(gnss_raw::headingBetween(run.q_end, run.q_true) * 180.0 / kPi, 40.0, 5.0);
}

TEST(RawGnssShadow, TheHeadingFitHandsItsHeadingOverOnce)
{
    // Act: the same flight, the fit's heading handed to the navigation filter
    // once it is sure -- once; the filter flies the truth's heading to the end
    // with its tilt untouched. With the accelerometer biased sideways, and
    // with the epochs built in the GNSS task, as the Beetle builds them. The
    // filter's GNSS turns 16 of the 40 deg out but its own 1-sigma stays
    // wide, so by the default rule the fit waits until it is sure to 3 deg
    // (the 9th epoch) and turns the last 24 deg.
    for (int external = 0; external < 2; ++external)
    for (double by : {0.0, 0.08})
    {
        const BoostRun run = flyBoost(RawGnssShadow::kHeadingAct, 40.0, by, -0.05, external != 0);
        SCOPED_TRACE(external ? "built in the GNSS task" : "built here");
        ASSERT_GE(run.recs.size(), 5u) << by;
        EXPECT_EQ(run.st.heading_handoffs, 1u) << by;
        ASSERT_GE(run.handed_at, 0) << by;
        const HeadingFitData& h = run.recs[run.handed_at];
        EXPECT_TRUE(h.flags & HFT_ACT);
        ASSERT_NE(h.sigma_cdeg, HFT_NO_SIGMA);
        EXPECT_LE(h.sigma_cdeg, 1000);
        EXPECT_NEAR(h.turn_cdeg, -h.filt_cdeg, 1);
        for (size_t i = run.handed_at + 1; i < run.recs.size(); ++i)
        {
            EXPECT_TRUE(run.recs[i].flags & HFT_HANDED);
            EXPECT_FALSE(run.recs[i].flags & HFT_HANDED_NOW);
        }
        for (const HeadingFitData& d : run.recs)             // thrust along the nose: one valley
            EXPECT_FALSE(d.flags & HFT_RIVAL);
        EXPECT_LE(h.sigma_cdeg, 300);
        // the whole attitude: the hand-off's heading is as good as the fit's
        // 1-sigma then, and the coast's gentle push teaches the filter little
        // after it
        EXPECT_LT(attErrDeg(run.q_end, run.q_true), 3.0) << by;
        // turned about the vertical only: the filter's tilt is the truth's
        float q_lev[4];
        const float* qe = run.q_end;
        const double hd = gnss_raw::headingBetween(qe, run.q_true);
        const float qz[4] = {float(std::cos(-0.5 * hd)), 0.0f, 0.0f, float(std::sin(-0.5 * hd))};
        q_lev[0] = qz[0] * qe[0] - qz[3] * qe[3];
        q_lev[1] = qz[0] * qe[1] - qz[3] * qe[2];
        q_lev[2] = qz[0] * qe[2] + qz[3] * qe[1];
        q_lev[3] = qz[0] * qe[3] + qz[3] * qe[0];
        EXPECT_LT(attErrDeg(q_lev, run.q_true), 2.0) << by;
    }
}

TEST(RawGnssShadow, TheHeadingFitLeavesAFilterThatIsNotLost)
{
    // heading_handoff_apart: the hand-off waits until the filter's heading is
    // further from the fit's than that many of their combined 1-sigmas (the
    // rig's r18 handed a two-epoch fit 9.8 deg off to a filter 2.5 deg off) --
    // or, with heading_handoff_sure_deg, until the fit is that sure.
    // Seeded right, the filter agrees with the fit and keeps its own heading.
    const BoostRun right = flyBoost(RawGnssShadow::kHeadingAct, 0.0, 0.0, 0.0, false, 2.0, 0, 0.0);
    ASSERT_GE(right.recs.size(), 5u);
    EXPECT_TRUE(right.recs.back().flags & HFT_SOLVED);
    EXPECT_EQ(right.st.heading_handoffs, 0u);
    EXPECT_EQ(right.handed_at, -1);
    EXPECT_LT(std::fabs(gnss_raw::headingBetween(right.q_end, right.q_true)) * 180.0 / kPi, 2.0);

    // Seeded 40 deg off, its GNSS turns it part of the way, its own 1-sigma
    // still wide (30 deg at launch): never twice their 1-sigmas apart, it
    // stalls 22 deg off. Sure to 3 deg, the fit hands over, late but whole.
    const BoostRun stalled = flyBoost(RawGnssShadow::kHeadingAct, 40.0, 0.0, 0.0, false, 2.0, 0, 0.0);
    EXPECT_EQ(stalled.st.heading_handoffs, 0u);
    EXPECT_GT(std::fabs(gnss_raw::headingBetween(stalled.q_end, stalled.q_true)) * 180.0 / kPi, 8.0);
    const BoostRun sure = flyBoost(RawGnssShadow::kHeadingAct, 40.0, 0.0, 0.0, false, 2.0, 0, 3.0);
    EXPECT_EQ(sure.st.heading_handoffs, 1u);
    ASSERT_GE(sure.handed_at, 0);
    EXPECT_LE(sure.recs[sure.handed_at].sigma_cdeg, 300);
    EXPECT_GT(std::fabs(sure.recs[sure.handed_at].filt_cdeg / 100.0), 8.0);
    EXPECT_LT(attErrDeg(sure.q_end, sure.q_true), 3.0);
    // and a looser apart takes the lost filter early
    const BoostRun lost = flyBoost(RawGnssShadow::kHeadingAct, 40.0, 0.0, 0.0, false, 1.0, 0, 0.0);
    EXPECT_EQ(lost.st.heading_handoffs, 1u);
    ASSERT_GE(lost.handed_at, 0);
    EXPECT_LE(lost.recs[lost.handed_at].n, 4);
    EXPECT_LT(attErrDeg(lost.q_end, lost.q_true), 3.0);

    // heading_handoff_min_epochs: held to that many epochs (unasked, the
    // third's 6 deg 1-sigma hands over)
    const BoostRun later = flyBoost(RawGnssShadow::kHeadingAct, 40.0, 0.0, 0.0, false, 0.0, 5, 0.0);
    EXPECT_EQ(later.st.heading_handoffs, 1u);
    ASSERT_GE(later.handed_at, 0);
    EXPECT_EQ(later.recs[later.handed_at].n, 5);
    EXPECT_LT(attErrDeg(later.q_end, later.q_true), 3.0);
}

TEST(RawGnssShadow, TheHeadingFitIsOffByDefault)
{
    const BoostRun run = flyBoost(RawGnssShadow::kHeadingOff, 40.0);
    EXPECT_TRUE(run.recs.empty());
    EXPECT_EQ(run.st.heading_seeds, 0u);
    EXPECT_EQ(run.st.heading_epochs, 0u);
}
