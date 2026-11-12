// Host tests for the gnss-sdr-dev board's stream as the V10's SDR raw-nav
// image hands it to the raw-GNSS filter, on the driver's path
// (TR_GNSSReceiverSdr_Serial: SdrRawEpoch.h's assembler and Doppler means,
// SdrRawFeed.h's clock-line sample, placement and fix):
//   - the Doppler means: a whole window of the board's epochs since the last
//     fused one averaged, the window's breaks (a lost epoch, a relock, a step,
//     a satellite missing from one) and the half rule;
//   - the assembler with the means on the recorded stream;
//   - the driver's order on a long stream (the recorded epochs re-timed over
//     20 s): every 4001 on the clock line before its epoch is placed, the
//     epochs untimed until the line is valid and then at their tags, the
//     board's clock steps put back, a record the board marks late left off
//     the line;
//   - the board's fix as GNSSData.
// On the board's recorded stream (fixtures/sdr_rtcm_stream.h).
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

#include "Rtcm3Msm.h"
#include "SdrRawFeed.h"
#include "fixtures/sdr_rtcm_stream.h"

using namespace sdr_rtcm;

namespace {

std::vector<std::vector<uint8_t>> frames()
{
    std::vector<std::vector<uint8_t>> out;
    size_t i = 0;
    while (i + 6 <= sizeof(kSdrStream))
    {
        const size_t len = 6 + (((kSdrStream[i + 1] & 3) << 8) | kSdrStream[i + 2]);
        out.emplace_back(kSdrStream + i, kSdrStream + i + len);
        i += len;
    }
    return out;
}

void putBits(std::vector<uint8_t>& m, size_t& pos, uint64_t v, int n)
{
    for (int i = n - 1; i >= 0; i--, pos++)
        if ((v >> i) & 1U) m[pos / 8] |= static_cast<uint8_t>(0x80 >> (pos % 8));
}

std::vector<uint8_t> frameOf(std::vector<uint8_t> m)
{
    std::vector<uint8_t> f = {0xD3, static_cast<uint8_t>(m.size() >> 8), static_cast<uint8_t>(m.size())};
    f.insert(f.end(), m.begin(), m.end());
    const uint32_t crc = rtcm3::crc24q(f.data(), f.size());
    f.push_back(static_cast<uint8_t>(crc >> 16));
    f.push_back(static_cast<uint8_t>(crc >> 8));
    f.push_back(static_cast<uint8_t>(crc));
    return f;
}

// A 4001 sub-type 1 written to its layout (rtcm_tr_epoch_encode), with its
// tail (no marks) and, when asked, the aiding block with its late bit.
std::vector<uint8_t> record(const TrEpoch& e, bool aid_block, bool late)
{
    std::vector<uint8_t> m(80, 0);
    size_t pos = 0;
    putBits(m, pos, kTrMessage, 12);
    putBits(m, pos, kTrEpoch, 4);
    putBits(m, pos, e.week, 16);
    putBits(m, pos, e.tow_ms, 30);
    putBits(m, pos, static_cast<uint64_t>(e.shift_ns) & 0xFFFFF, 20);
    putBits(m, pos, e.clk_steps, 16);
    putBits(m, pos, static_cast<uint64_t>(std::llround(e.clk_step_m * 1000.0)) & 0xFFFFFFFFFFFFull, 48);
    putBits(m, pos, e.mode, 2);
    putBits(m, pos, e.nobs, 6);
    putBits(m, pos, e.sample & 0xFFFFFFFFFFFFull, 48);
    if (aid_block)
    {
        putBits(m, pos, 0, 6);    // ranges left out
        putBits(m, pos, 0, 6);    // marks
        putBits(m, pos, 1, 4);    // the block's version
        putBits(m, pos, 0, 14);   // src .. phase
        putBits(m, pos, late ? 1 : 0, 1);
        putBits(m, pos, 0, 5);
        putBits(m, pos, 0, 8);    // fc_frames
    }
    m.resize((pos + 7) / 8);
    return frameOf(m);
}

// The MSM7 frame re-timed: its 30-bit epoch time (DF004/DF248/DF427, bits
// 24-53 of the payload) moved on by dt_ms, the CRC made again.
std::vector<uint8_t> retimeMsm(const std::vector<uint8_t>& f, uint32_t dt_ms)
{
    std::vector<uint8_t> m(f.begin() + 3, f.end() - 3);
    uint32_t t = 0;
    for (size_t b = 24; b < 54; b++) t = (t << 1) | ((m[b / 8] >> (7 - b % 8)) & 1U);
    t = (t + dt_ms) % 604800000U;
    for (size_t b = 24; b < 54; b++)
    {
        const uint8_t bit = static_cast<uint8_t>(0x80 >> (b % 8));
        if ((t >> (53 - b)) & 1U) m[b / 8] |= bit;
        else m[b / 8] &= static_cast<uint8_t>(~bit);
    }
    return frameOf(m);
}

// A RawEpoch of n satellites, each Doppler rate[i] (GPS PRNs 1..n).
gnss_raw::RawEpoch epochOf(const std::vector<double>& rate, uint32_t lock_ms = 10000)
{
    gnss_raw::RawEpoch e;
    e.n = static_cast<int>(rate.size());
    for (int i = 0; i < e.n; i++)
    {
        gnss_raw::RawObs& o = e.obs[i];
        o.sys = GNSS_SYS_GPS;
        o.prn = static_cast<uint8_t>(i + 1);
        o.has_pr = true;
        o.pr_m = 2.0e7;
        o.has_rate = true;
        o.rate_mps = rate[i];
        o.cn0_dbhz = 40.0f;
        o.lock_ms = lock_ms;
    }
    return e;
}

SdrDopplerMeanConfig tenAt10()
{
    SdrDopplerMeanConfig c;
    c.n = 10;
    c.period_ms = 10;
    c.max_step_mps = 5.0;
    return c;
}

}  // namespace

// ── The Doppler means ───────────────────────────────────────────────────

TEST(SdrDopplerMean, AWholeWindowIsAveragedAtItsMiddle)
{
    // two satellites, one ramping 1 m/s an epoch (100 m/s^2), one still
    SdrDopplerMean m;
    m.setConfig(tenAt10());
    SdrDopplerMeanStats st;
    gnss_raw::RawEpoch e;
    for (int k = 0; k < 10; k++)
    {
        e = epochOf({100.0 + k, -50.0});
        m.note(e, 1000 + 10 * k);
    }
    EXPECT_EQ(m.close(e, st), 2);
    EXPECT_DOUBLE_EQ(e.obs[0].rate_mps, 104.5);   // the ramp at its window's middle, 45 ms before the last
    EXPECT_DOUBLE_EQ(e.obs[1].rate_mps, -50.0);
    EXPECT_TRUE(e.obs[0].has_rate);
    EXPECT_EQ(st.epochs, 1u);
    EXPECT_EQ(st.means, 2u);
    EXPECT_EQ(st.dopplers, 2u);
    EXPECT_FLOAT_EQ(st.step_max_mps, 1.0f);
    EXPECT_FLOAT_EQ(st.spread_max_mps, 9.0f);
    // the window starts over: the next grid epoch alone is not a window of ten
    gnss_raw::RawEpoch next = epochOf({110.0, -50.0});
    m.note(next, 1100);
    EXPECT_EQ(m.close(next, st), 0);
    EXPECT_DOUBLE_EQ(next.obs[0].rate_mps, 110.0);   // as it came
    EXPECT_EQ(st.epochs_short, 1u);
    EXPECT_EQ(st.alone_epoch, 2u);
}

TEST(SdrDopplerMean, ALostEpochLeavesTheWholeEpochAsItCame)
{
    SdrDopplerMean m;
    m.setConfig(tenAt10());
    SdrDopplerMeanStats st;
    gnss_raw::RawEpoch e;
    for (int k = 0; k < 11; k++)
    {
        if (k == 4) continue;   // lost: the board's 10 ms grid broken
        e = epochOf({100.0 + k});
        m.note(e, 1000 + 10 * k);
    }
    EXPECT_EQ(m.close(e, st), 0);
    EXPECT_DOUBLE_EQ(e.obs[0].rate_mps, 110.0);
    EXPECT_EQ(st.epochs_short, 1u);
}

TEST(SdrDopplerMean, ARelockAStepOrAGapDropsThatSatellitesDoppler)
{
    // four satellites; one relocks, one steps 6 m/s in 10 ms, one misses an
    // epoch, one is clean: three of four do not average, so under half do --
    // the epoch goes as it came. With a fifth and sixth clean, three of six
    // average and the other three are dropped from the epoch of means.
    for (int clean : {1, 3})
    {
        SCOPED_TRACE(clean);
        SdrDopplerMean m;
        m.setConfig(tenAt10());
        SdrDopplerMeanStats st;
        gnss_raw::RawEpoch e;
        for (int k = 0; k < 10; k++)
        {
            std::vector<double> r = {10.0, 20.0 + (k >= 5 ? 6.0 : 0.0), 30.0};
            for (int c = 0; c < clean; c++) r.push_back(40.0 + c);
            e = epochOf(r);
            e.obs[0].lock_ms = k == 3 ? 0 : 10000 + 10 * k;   // relocked at the fourth epoch
            if (k == 7) e.obs[2].has_rate = false;            // missing from one
            m.note(e, 2000 + 10 * k);
        }
        const int n = m.close(e, st);
        if (clean == 1)
        {
            EXPECT_EQ(n, 0);
            EXPECT_EQ(st.epochs_few, 1u);
            EXPECT_TRUE(e.obs[0].has_rate && e.obs[1].has_rate && e.obs[2].has_rate);
            EXPECT_DOUBLE_EQ(e.obs[1].rate_mps, 26.0);   // as it came
        }
        else
        {
            EXPECT_EQ(n, 3);
            EXPECT_FALSE(e.obs[0].has_rate);
            EXPECT_FALSE(e.obs[1].has_rate);
            EXPECT_FALSE(e.obs[2].has_rate);
            EXPECT_TRUE(e.obs[0].has_pr);                // the code stays
            EXPECT_EQ(st.alone_lock, 1u);
            EXPECT_EQ(st.alone_step, 1u);
            EXPECT_EQ(st.alone_missing, 1u);
            EXPECT_EQ(st.means, 3u);
            for (int c = 0; c < 3; c++) EXPECT_DOUBLE_EQ(e.obs[3 + c].rate_mps, 40.0 + c);
        }
    }
}

TEST(SdrDopplerMean, OffItTouchesNothing)
{
    SdrDopplerMean m;   // n 0
    SdrDopplerMeanStats st;
    gnss_raw::RawEpoch e = epochOf({5.0});
    for (int k = 0; k < 10; k++) m.note(e, 10 * k);
    EXPECT_EQ(m.close(e, st), 0);
    EXPECT_EQ(st.epochs, 0u);
    EXPECT_DOUBLE_EQ(e.obs[0].rate_mps, 5.0);
}

TEST(SdrDopplerMean, TheAssemblerAveragesTheRecordedEpochs)
{
    // The fixture's ten epochs (330 .. 420) on a 20 ms grid with means of two:
    // 330 and 340 open their periods with no whole window behind them, then
    // 360, 380, 400 and 420 each average with the epoch before
    SdrEpochAssembler::Config c;
    c.fuse_period_ms = 20;
    c.mean.n = 2;
    c.mean.period_ms = 10;
    c.mean.max_step_mps = 5.0;
    SdrEpochAssembler with(c);
    SdrEpochAssembler::Config plain_c = c;
    plain_c.fuse_period_ms = 0;
    plain_c.mean = SdrDopplerMeanConfig{};
    SdrEpochAssembler plain(plain_c);
    std::vector<SdrEpoch> all, grid;
    int64_t t = 0;
    for (const auto& f : frames())
    {
        std::unique_ptr<SdrEpoch> e(new SdrEpoch());
        t += 100;
        if (plain.onFrame(f.data(), f.size(), t, *e)) all.push_back(*e);
        if (with.onFrame(f.data(), f.size(), t, *e)) grid.push_back(*e);
    }
    ASSERT_EQ(all.size(), kSdrStreamEpochs);
    ASSERT_EQ(grid.size(), 6u);
    EXPECT_EQ(grid[0].rr_mean, 0);
    EXPECT_EQ(grid[1].rr_mean, 0);
    for (size_t g = 2; g < grid.size(); g++)
    {
        EXPECT_EQ(grid[g].rr_mean, 1);
        const size_t k = (grid[g].tow_ms - kSdrStreamFirstEpochTow) / 10;   // its index in `all`
        for (int i = 0; i < grid[g].epoch.n; i++)
        {
            const gnss_raw::RawObs& o = grid[g].epoch.obs[i];
            if (!o.has_rate) continue;
            EXPECT_NEAR(o.rate_mps, 0.5 * (all[k].epoch.obs[i].rate_mps + all[k - 1].epoch.obs[i].rate_mps), 1e-9);
            EXPECT_DOUBLE_EQ(o.pr_m, all[k].epoch.obs[i].pr_m);   // only the Doppler moves
        }
    }
    EXPECT_EQ(with.meanStats().epochs, 6u);
    EXPECT_EQ(with.meanStats().epochs_short, 2u);
    EXPECT_EQ(with.meanStats().means, with.meanStats().dopplers - with.meanStats().alone_epoch
                                          - with.meanStats().alone_missing - with.meanStats().alone_lock
                                          - with.meanStats().alone_step);
}

// ── The driver's path on a long stream ──────────────────────────────────

namespace {

struct FeedRun
{
    std::vector<gnss_raw::TimedRawEpoch> epochs;
    std::vector<uint32_t> tow;      // each one's tag
    int samples = 0, late = 0;
};

// The recorded ten epochs re-timed into `n` consecutive ones from the first's
// tag, each arriving `delay_us` after its tag on an MCU clock that runs with
// GPS time; every `late_every`th record marked late by the board. Fed as the
// driver feeds them: each 4001 to the line first (lineSampleOf), then every
// frame to the assembler, each epoch handed over placed (timedEpochOf).
FeedRun driverPath(int n, int64_t delay_us, const SdrEpochAssembler::Config& cfg, int late_every = 0,
               int32_t shift_ns = 0)
{
    const auto fs = frames();
    lc86::GnssTimeSync line;
    SdrEpochAssembler a(cfg);
    std::unique_ptr<SdrEpoch> e(new SdrEpoch());
    FeedRun r;
    const int64_t t0 = 5000000;
    for (int k = 0; k < n; k++)
    {
        const int src = k % 10;   // the recorded epoch it repeats
        const uint32_t dt = static_cast<uint32_t>(10 * (k - src));
        const uint32_t tow = kSdrStreamFirstEpochTow + 10U * static_cast<uint32_t>(k);
        const int64_t arrive = t0 + static_cast<int64_t>(10 * k) * 1000 + delay_us;
        std::vector<std::vector<uint8_t>> epoch_frames;
        for (int j = 0; j < 3; j++) epoch_frames.push_back(retimeMsm(fs[5 + 4 * src + j], dt));
        TrEpoch rec;
        EXPECT_TRUE(decodeTrEpoch(fs[8 + 4 * src].data(), fs[8 + 4 * src].size(), rec));
        rec.tow_ms = tow;
        rec.shift_ns = shift_ns;
        const bool late = late_every > 0 && k % late_every == late_every - 1;
        epoch_frames.push_back(record(rec, true, late));
        for (const auto& f : epoch_frames)
        {
            TrEpoch r1;
            if (decodeTrEpoch(f.data(), f.size(), r1))
            {
                double g = 0.0;
                if (lineSampleOf(f.data(), f.size(), r1, g))
                {
                    line.addSample(g, arrive);
                    r.samples++;
                }
                else
                {
                    r.late++;
                }
            }
            if (a.onFrame(f.data(), f.size(), arrive, *e))
            {
                r.epochs.emplace_back();
                timedEpochOf(*e, line, r.epochs.back());
                r.tow.push_back(e->tow_ms);
            }
        }
    }
    return r;
}

}  // namespace

TEST(SdrRawFeed, EpochsAreUntimedUntilTheLineThenAtTheirTags)
{
    // 20 s of the board at 100 Hz, fused at 10 Hz with the means, each record
    // 3 ms after its tag: the line answers after its 10 s, and from then every
    // epoch's GPS instant (its tag) sits on the MCU clock 3 ms late -- the
    // board's delay, uncalibrated (latency_s 0)
    SdrEpochAssembler::Config c;
    c.fuse_period_ms = 100;
    c.mean = SdrDopplerMeanConfig{10, 10, 5.0, {}};
    const FeedRun r = driverPath(2000, 3000, c);
    ASSERT_EQ(r.epochs.size(), 201u);   // tags 330 .. 20320: the first of each 100 ms, 300 .. 20300
    EXPECT_EQ(r.samples, 2000);
    int untimed = 0, timed = 0, with_means = 0;
    for (size_t i = 0; i < r.epochs.size(); i++)
    {
        const gnss_raw::TimedRawEpoch& t = r.epochs[i];
        EXPECT_EQ(t.gps_week, 2443);
        with_means += t.rr_paired;
        if (t.mcu_us == 0)
        {
            untimed++;
            EXPECT_EQ(t.tag_dtr_ms, 0.0f);
            continue;
        }
        timed++;
        const int64_t truth = 5000000 + static_cast<int64_t>(r.tow[i] - kSdrStreamFirstEpochTow) * 1000;
        EXPECT_NEAR(static_cast<double>(t.mcu_us - truth), 3000.0, 50.0) << i;
        // its time of week the board's unstepped clock: the tag plus the step sum's
        EXPECT_NEAR(t.epoch.tow_s - t.tag_dtr_ms * 1.0e-3, r.tow[i] * 1.0e-3, 1e-6);
        EXPECT_NEAR(t.tag_dtr_ms, 2591783.819 / kC * 1.0e3, 1e-4);
    }
    EXPECT_GT(untimed, 90);   // the line's first 10 s
    EXPECT_LT(untimed, 110);
    EXPECT_GT(timed, 90);
    EXPECT_EQ(with_means, 199);   // all but the first two: 330 has no window behind it, 400 seven (340 .. 400)
}

TEST(SdrRawFeed, TheLineIsFittedToTheLatchNotTheTag)
{
    // a shift of 0.4 ms (the tag after the latch; the field holds +-0.52 ms):
    // the line follows the latch, so the tag lands 0.4 ms later on the MCU
    // clock than the latch does -- where the measurements are
    SdrEpochAssembler::Config c;
    c.fuse_period_ms = 100;
    const FeedRun a = driverPath(1500, 3000, c, 0, 0);
    const FeedRun b = driverPath(1500, 3000, c, 0, 400000);
    ASSERT_EQ(a.epochs.size(), b.epochs.size());
    const gnss_raw::TimedRawEpoch& ea = a.epochs.back();
    const gnss_raw::TimedRawEpoch& eb = b.epochs.back();
    ASSERT_NE(ea.mcu_us, 0);
    ASSERT_NE(eb.mcu_us, 0);
    EXPECT_NEAR(static_cast<double>(eb.mcu_us - ea.mcu_us), 400.0, 20.0);
}

TEST(SdrRawFeed, ARecordTheBoardMarksLateIsLeftOffTheLine)
{
    SdrEpochAssembler::Config c;
    c.fuse_period_ms = 100;
    const FeedRun r = driverPath(1500, 3000, c, 7);
    EXPECT_EQ(r.late, 1500 / 7);
    EXPECT_EQ(r.samples, 1500 - 1500 / 7);
    // its epoch is still made and placed: only the line leaves it out
    EXPECT_EQ(r.epochs.size(), 151u);
    EXPECT_NE(r.epochs.back().mcu_us, 0);
}

TEST(SdrRawFeed, LineSampleOfIsTheDriversRule)
{
    // the driver's noteLink: decodeEpochAid then latchGps
    const auto fs = frames();
    TrEpoch rec;
    ASSERT_TRUE(decodeTrEpoch(fs[8].data(), fs[8].size(), rec));
    double g = 0.0, want = 0.0;
    sdr_aid::EpochAid aid;
    ASSERT_TRUE(sdr_aid::decodeEpochAid(fs[8].data(), fs[8].size(), aid));
    ASSERT_TRUE(sdr_aid::latchGps(rec, aid, want));
    ASSERT_TRUE(lineSampleOf(fs[8].data(), fs[8].size(), rec, g));
    EXPECT_DOUBLE_EQ(g, want);
    EXPECT_NEAR(g, rec.week * 604800.0 + rec.tow_ms * 1e-3 - rec.shift_ns * 1e-9, 1e-9);
    // an MSM is no record
    EXPECT_FALSE(lineSampleOf(fs[5].data(), fs[5].size(), rec, g));
}

// ── The board's fix as GNSSData ─────────────────────────────────────────

TEST(SdrRawFeed, TheBoardsFixIsTheFlightComputersGnssData)
{
    const auto fs = frames();
    TrFix f;
    ASSERT_TRUE(decodeTrFix(fs[0].data(), fs[0].size(), f));
    GNSSData d;
    ASSERT_TRUE(gnssDataFromTrFix(f, 1234567u, d));
    EXPECT_EQ(d.time_us, 1234567u);
    EXPECT_EQ(d.fix_mode, 3);
    EXPECT_EQ(d.num_sats, 18);
    EXPECT_EQ(d.pdop_x10, 13);
    EXPECT_EQ(d.h_acc_m, 255);   // not given
    // the bench antenna, 39.97 N 74.94 W, its height above the ellipsoid
    EXPECT_NEAR(d.lat_e7 * 1e-7, 39.9716209, 1e-7);
    EXPECT_NEAR(d.lon_e7 * 1e-7, -74.9362967, 1e-7);
    EXPECT_NEAR(d.alt_mm * 1e-3, -11.528, 0.002);
    // its ECEF velocity turned into north, east and up (mm/s)
    EXPECT_EQ(d.vel_n_mmps, 5);
    EXPECT_EQ(d.vel_e_mmps, -3);
    EXPECT_EQ(d.vel_u_mmps, -2);
    // GPS week 2443, 156619.277 s: 2026-11-02 19:30:19.277 GPS, 18 s ahead of UTC
    EXPECT_EQ(d.year, 2026);
    EXPECT_EQ(d.month, 11);
    EXPECT_EQ(d.day, 2);
    EXPECT_EQ(d.hour, 19);
    EXPECT_EQ(d.minute, 30);
    EXPECT_EQ(d.second, 1);
    EXPECT_EQ(d.milli_second, 277);
    // a fix without its velocity, or not valid, is no fix: the time alone
    TrFix nv = f;
    nv.vel_valid = false;
    EXPECT_FALSE(gnssDataFromTrFix(nv, 1u, d));
    EXPECT_EQ(d.fix_mode, 0);
    EXPECT_EQ(d.lat_e7, 0);
    EXPECT_EQ(d.year, 2026);
    TrFix bad = f;
    bad.valid = false;
    EXPECT_FALSE(gnssDataFromTrFix(bad, 1u, d));
    TrFix zero = f;
    zero.pos[0] = zero.pos[1] = zero.pos[2] = 0.0;
    EXPECT_FALSE(gnssDataFromTrFix(zero, 1u, d));
}

TEST(SdrRawFeed, UtcAcrossAMonthAndAYear)
{
    GNSSData d;
    // GPS week 2399 (2025-12-28 .. 2026-01-03), Wednesday 2025-12-31 23:59:59 UTC + 18 s
    gpsToUtcFields(2399, (3 * 86400 + 86399 + 18) * 1000U + 500U, 18, d);
    EXPECT_EQ(d.year, 2025);
    EXPECT_EQ(d.month, 12);
    EXPECT_EQ(d.day, 31);
    EXPECT_EQ(d.hour, 23);
    EXPECT_EQ(d.minute, 59);
    EXPECT_EQ(d.second, 59);
    EXPECT_EQ(d.milli_second, 500);
    gpsToUtcFields(2399, (4 * 86400 + 18) * 1000U, 18, d);
    EXPECT_EQ(d.year, 2026);
    EXPECT_EQ(d.month, 1);
    EXPECT_EQ(d.day, 1);
    EXPECT_EQ(d.hour, 0);
}
