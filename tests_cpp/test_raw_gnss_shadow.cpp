// Host tests for the raw-GNSS shadow filter (#1600 phase 4): the cross-core
// handoffs (ShadowLink.h) and RawGnssShadow, the part of the flight
// computer's core-0 task that is not FreeRTOS -- when it starts and stops,
// when a raw epoch is applied and how late, the pad resync, its 10 Hz log
// record -- and, end to end, two real MSM7 epochs the Beetle's LC86G sent
// (fixtures/raw_meas_golden.h) through the flight parser into a running
// shadow filter.
#include <gtest/gtest.h>

#include <cstdlib>

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

}  // namespace

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
                "baro fused %u, levelled %u (held %u), mag %u\n",
                gyro_bias_dps, t_land, max_h_err, max_horiz, sh.stats().baro_fused, sh.stats().levels,
                sh.stats().levels_held, sh.stats().mag_fused);
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
