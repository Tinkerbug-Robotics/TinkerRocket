// Host tests for the replay harness's mosaic-G5 feed (tools/raw_replay/
// mosaic_feed.h): what the V8/V9 driver and GNSS task do with each SBF block,
// run on the firmware's own decoders. MeasEpochs are handed over on the
// fused grid only, the others' clock jumps still followed; each epoch's GPS
// time is its receiver time less the bias of the last PVTGeodetic that
// anchors the jump count, carried by its drift and the jumps since; a PVT
// whose own MeasEpoch was lost anchors nothing; an empty epoch is not handed
// over; navigation blocks are kept as the driver keeps them; GPSIon goes
// through the one ionosphere rule; PVTGeodetic is the fix the flight loop's
// GPS voter gets; and an epoch is timed only where the driver would time it
// (FINETIME, a 1PPS edge paired within 120 s, the clock line valid).
// Synthetic blocks only (sbf_build.h, sbf_meas_build.h).
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "MosaicRawEpoch.h"
#include "SbfNav.h"
#include "mosaic_feed.h"
#include "sbf_build.h"
#include "sbf_meas_build.h"

using rr::MosaicFeed;

namespace {

constexpr uint16_t kWnc = 2440;
constexpr uint32_t kTow0 = 507606000;   // a whole second

// A block body written at the guide's absolute offsets (the body starts at 14).
struct Body
{
    std::vector<uint8_t> v;
    explicit Body(size_t block_len) : v(block_len - 14, 0) {}
    void put(size_t at, const void* p, size_t n) { std::memcpy(v.data() + (at - 14), p, n); }
    void u8(size_t at, uint8_t x) { put(at, &x, 1); }
    void u16(size_t at, uint16_t x) { put(at, &x, 2); }   // little-endian host
    void f32(size_t at, float x) { put(at, &x, 4); }
    void f64(size_t at, double x) { put(at, &x, 8); }
};

// PVTGeodetic (4007) rev 2 with a solution: mode 1, the clock's bias and drift.
std::vector<uint8_t> pvt(uint32_t tow, double bias_ms, float drift_ppm, double height_m = 120.0,
                         float undulation_m = -33.0f, float vu = 0.25f, uint8_t nr_sv = 21)
{
    Body b(96);
    b.u8(14, 1);
    b.u8(15, 0);
    b.f64(16, 0.5);
    b.f64(24, 2.0);
    b.f64(32, height_m);
    b.f32(40, undulation_m);
    b.f32(44, 0.5f);
    b.f32(48, -0.5f);
    b.f32(52, vu);
    b.f64(60, bias_ms);
    b.f32(68, drift_ppm);
    b.u8(72, 0);
    b.u8(74, nr_sv);
    b.u16(88, 20);
    b.u16(90, 300);
    b.u16(92, 500);
    return sbftest::block(mosaic::SBF_PVT_GEODETIC, 2, tow, kWnc, b.v);
}

// A MeasEpoch with GPS L1 C/A and Galileo E1 satellites.
std::vector<uint8_t> meas(uint32_t tow, uint8_t cum, bool empty = false)
{
    sbftest::MeasEpochSpec s;
    s.tow = tow;
    s.wnc = kWnc;
    s.cum = cum;
    if (empty)
    {
        sbftest::MeasT1 glo;
        glo.sig(8).svid = 40;   // GLONASS L1: not a signal the filter takes
        glo.prM(2.0e7).dopplerHz(100.0);
        s.t1.push_back(glo);
    }
    else
    {
        for (int k = 0; k < 4; ++k)
        {
            sbftest::MeasT1 t;
            t.sig(0).svid = static_cast<uint8_t>(3 + k);   // GPS
            t.prM(2.1e7 + 1.0e5 * k).dopplerHz(-1500.0 + 300.0 * k);
            t.rx_channel = static_cast<uint8_t>(1 + k);
            s.t1.push_back(t);
        }
        sbftest::MeasT1 e;
        e.sig(17).svid = 73;   // Galileo E03
        e.prM(2.4e7).dopplerHz(800.0);
        e.rx_channel = 9;
        s.t1.push_back(e);
    }
    return sbftest::measEpoch(s);
}

MosaicFeed::Kind feed(MosaicFeed& f, const std::vector<uint8_t>& b, gnss_raw::TimedRawEpoch& te, int64_t t = 1000)
{
    return f.onBlock(b.data(), b.size(), t, te);
}

std::vector<uint8_t> gpsIon(const int8_t raw[8], uint32_t tow, uint16_t wnc)
{
    static constexpr int kExp[8] = {-30, -27, -24, -24, 11, 14, 16, 16};
    Body b(48);
    b.u8(14, 12);
    for (int k = 0; k < 8; k++)
        b.f32(16 + 4 * k, static_cast<float>(std::ldexp(static_cast<double>(raw[k]), kExp[k])));
    return sbftest::block(mosaic::kSbfGpsIon, 0, tow, wnc, b.v);
}

// GALNav (4002): Source 2 (I/NAV) or 16 (F/NAV), every float a plain number.
std::vector<uint8_t> galNav(uint8_t svid, uint8_t source)
{
    Body b(152);
    b.u8(14, svid);
    b.u8(15, source);
    b.f64(16, 5440.6);
    b.u16(124, kWnc % 4096);
    b.u16(126, kWnc % 4096);
    b.u16(128, 77);
    b.u16(130, 0x01);   // E1-B status received, healthy
    b.u8(134, 107);
    return sbftest::block(mosaic::kSbfGalNav, 0, kTow0, kWnc, b.v);
}

}  // namespace

TEST(RawReplayMosaic, EpochsOnTheFusedGridOnly)
{
    MosaicFeed f;   // 100 ms grid
    gnss_raw::TimedRawEpoch te;
    int handed = 0;
    for (int k = 0; k < 20; ++k)
    {
        const uint32_t tow = kTow0 + 50 * k;
        const MosaicFeed::Kind kind = feed(f, meas(tow, 10), te, 1000 + 50000 * k);
        if (tow % 100 == 0)
        {
            ASSERT_EQ(kind, MosaicFeed::Kind::EPOCH) << tow;
            handed++;
            EXPECT_EQ(te.gps_week, kWnc);
            EXPECT_EQ(te.arrival_us, 1000 + 50000 * k);
            EXPECT_DOUBLE_EQ(te.epoch.tow_s, tow * 1e-3);
            EXPECT_EQ(te.epoch.n, 5);
            EXPECT_EQ(te.mcu_us, 0);   // the caller places it on the clock line
        }
        else
        {
            EXPECT_EQ(kind, MosaicFeed::Kind::EPOCH_SKIPPED) << tow;
        }
    }
    EXPECT_EQ(handed, 10);
    EXPECT_EQ(f.counts().epochs, 10u);
    EXPECT_EQ(f.counts().off_grid, 10u);

    // the bench ladder's 5 Hz: every fourth
    MosaicFeed::Config c;
    c.fuse_period_ms = 200;
    MosaicFeed f5(c);
    handed = 0;
    for (int k = 0; k < 20; ++k)
        handed += feed(f5, meas(kTow0 + 50 * k, 10), te) == MosaicFeed::Kind::EPOCH ? 1 : 0;
    EXPECT_EQ(handed, 5);
}

TEST(RawReplayMosaic, TheTagCarriesTheLastAnchoringPvt)
{
    MosaicFeed f;
    gnss_raw::TimedRawEpoch te;
    // no PVT yet: the bias is taken as 0
    ASSERT_EQ(feed(f, meas(kTow0, 10), te), MosaicFeed::Kind::EPOCH);
    EXPECT_FALSE(f.anchored());
    EXPECT_EQ(f.dtrMs(), 0.0);
    EXPECT_DOUBLE_EQ(f.gpsS(), kWnc * 604800.0 + kTow0 * 1e-3);
    EXPECT_EQ(f.counts().dtr_zero, 1u);
    // Stream7's order: each PVT just after its MeasEpoch
    EXPECT_EQ(feed(f, pvt(kTow0, 0.30, 0.25f), te), MosaicFeed::Kind::PVT);
    EXPECT_EQ(feed(f, meas(kTow0 + 50, 10), te), MosaicFeed::Kind::EPOCH_SKIPPED);
    EXPECT_EQ(feed(f, pvt(kTow0 + 50, 0.31, 0.25f), te), MosaicFeed::Kind::PVT);
    ASSERT_EQ(feed(f, meas(kTow0 + 100, 10), te), MosaicFeed::Kind::EPOCH);
    ASSERT_TRUE(f.anchored());
    // the PVT of +50 ms, carried 50 ms by its drift
    const double dtr = mosaic::dtrMs(0.31, 0.25f, 0.05, 0);
    EXPECT_DOUBLE_EQ(f.dtrMs(), dtr);
    EXPECT_DOUBLE_EQ(f.gpsS(), mosaic::epochGpsS(kWnc, kTow0 + 100, dtr));
    // a double holds a GPS time of 1.5e9 s to 0.24 us
    EXPECT_NEAR((f.gpsS() - (kWnc * 604800.0 + (kTow0 + 100) * 1e-3)) * 1e3, -0.31, 5e-4);
    EXPECT_EQ(f.counts().pvt_anchor, 2u);
}

TEST(RawReplayMosaic, AClockJumpMovesTheTag)
{
    MosaicFeed f;
    gnss_raw::TimedRawEpoch te;
    feed(f, meas(kTow0, 10), te);
    feed(f, pvt(kTow0, 0.45, 0.25f), te);
    // the clock steps -1 ms at the next epoch (CumClkJumps 10 -> 9): the receiver's time a millisecond less ahead
    ASSERT_EQ(feed(f, meas(kTow0 + 50, 9), te), MosaicFeed::Kind::EPOCH_SKIPPED);
    ASSERT_EQ(feed(f, meas(kTow0 + 100, 9), te), MosaicFeed::Kind::EPOCH);
    // the PVT of +50 never came: the anchor is the first, 100 ms back, with the jump
    EXPECT_TRUE(f.anchored());
    EXPECT_DOUBLE_EQ(f.dtrMs(), mosaic::dtrMs(0.45, 0.25f, 0.1, -1));
    EXPECT_NEAR(f.dtrMs(), 0.45 + 0.25e-3 * 0.1 - 1.0, 1e-9);
    EXPECT_EQ(f.counts().jumps, 1u);
    EXPECT_EQ(f.info().j_ms, -1);
    // every cell's lock counts from the jump
    for (int i = 0; i < te.epoch.n; ++i)
        EXPECT_LE(te.epoch.obs[i].lock_ms, 50u);
}

TEST(RawReplayMosaic, APvtWhoseMeasEpochWasLostAnchorsNothing)
{
    MosaicFeed f;
    gnss_raw::TimedRawEpoch te;
    feed(f, meas(kTow0, 10), te);
    feed(f, pvt(kTow0, 0.20, 0.25f), te);
    // the next MeasEpoch lost on the link, its PVT not
    feed(f, pvt(kTow0 + 50, 0.90, 0.25f), te);
    EXPECT_EQ(f.counts().pvt_anchor, 1u);
    ASSERT_EQ(feed(f, meas(kTow0 + 100, 10), te), MosaicFeed::Kind::EPOCH);
    EXPECT_DOUBLE_EQ(f.dtrMs(), mosaic::dtrMs(0.20, 0.25f, 0.1, 0));
}

TEST(RawReplayMosaic, APvtOlderThanASecondIsNotCarried)
{
    MosaicFeed f;
    gnss_raw::TimedRawEpoch te;
    feed(f, meas(kTow0, 10), te);
    feed(f, pvt(kTow0, 0.20, 0.25f), te);
    ASSERT_EQ(feed(f, meas(kTow0 + 1000, 10), te), MosaicFeed::Kind::EPOCH);
    EXPECT_TRUE(f.anchored());
    ASSERT_EQ(feed(f, meas(kTow0 + 1100, 10), te), MosaicFeed::Kind::EPOCH);
    EXPECT_FALSE(f.anchored());
    EXPECT_EQ(f.dtrMs(), 0.0);
    EXPECT_EQ(f.counts().dtr_zero, 2u);   // and the first, before any PVT
}

TEST(RawReplayMosaic, AnEmptyEpochIsNotHandedOver)
{
    MosaicFeed f;
    gnss_raw::TimedRawEpoch te;
    EXPECT_EQ(feed(f, meas(kTow0, 10, true), te), MosaicFeed::Kind::EPOCH_SKIPPED);
    EXPECT_EQ(f.counts().empty, 1u);
    EXPECT_EQ(f.counts().epochs, 0u);
    // a block that does not check out at all
    std::vector<uint8_t> bad = meas(kTow0 + 100, 10);
    bad.resize(20);
    EXPECT_EQ(f.onBlock(bad.data(), bad.size(), 0, te), MosaicFeed::Kind::OTHER);
}

TEST(RawReplayMosaic, NavigationBlocksAsTheDriverKeepsThem)
{
    MosaicFeed f;
    gnss_raw::TimedRawEpoch te;
    feed(f, meas(kTow0, 10), te);
    ASSERT_EQ(feed(f, galNav(73, 2), te), MosaicFeed::Kind::NAV);
    EXPECT_EQ(f.eph().sys, GNSS_SYS_GAL);
    EXPECT_EQ(f.eph().prn, 3);
    EXPECT_EQ(f.eph().iode, 77);
    EXPECT_EQ(f.eph().week, kWnc);
    // F/NAV: the driver never queues it
    EXPECT_EQ(feed(f, galNav(73, 16), te), MosaicFeed::Kind::NAV_SKIPPED);
    EXPECT_EQ(f.counts().nav, 2u);
    EXPECT_EQ(f.counts().nav_ok, 1u);
    EXPECT_EQ(f.counts().nav_dropped, 1u);
}

TEST(RawReplayMosaic, TheOneIonosphereRule)
{
    MosaicFeed f;
    gnss_raw::TimedRawEpoch te;
    const int8_t a[8] = {12, 4, -5, -1, 76, 0, -2, 1};
    const int8_t b[8] = {13, 4, -5, -1, 76, 0, -2, 1};
    const uint32_t day_ms = 86400000;
    // the first set, then the same bytes again: applied once
    ASSERT_EQ(feed(f, gpsIon(a, 2 * day_ms, kWnc), te), MosaicFeed::Kind::ION);
    EXPECT_EQ(f.iono().gps_day, kWnc * 7 + 2);
    EXPECT_EQ(std::memcmp(f.iono().raw, a, 8), 0);
    EXPECT_EQ(feed(f, gpsIon(a, 2 * day_ms + 5000, kWnc), te), MosaicFeed::Kind::OTHER);
    // an older day's: not applied; a later day's new bytes: applied
    EXPECT_EQ(feed(f, gpsIon(b, 1 * day_ms, kWnc), te), MosaicFeed::Kind::OTHER);
    EXPECT_EQ(feed(f, gpsIon(b, 3 * day_ms, kWnc), te), MosaicFeed::Kind::ION);
    EXPECT_EQ(f.counts().ion, 4u);
    EXPECT_EQ(f.counts().ion_applied, 2u);

    // the app's against the receiver's on the same day: the receiver's wins either way round
    rr::IonoRule r;
    gnss_raw::ShadowIono app{}, rx{};
    std::memcpy(app.raw, a, 8);
    std::memcpy(rx.raw, b, 8);
    app.gps_day = rx.gps_day = 17000;
    EXPECT_TRUE(r.offer(app, false));
    EXPECT_TRUE(r.offer(rx, true));
    EXPECT_FALSE(r.offer(app, false));
    rr::IonoRule r2;
    EXPECT_TRUE(r2.offer(rx, true));
    EXPECT_FALSE(r2.offer(app, false));
    app.gps_day = 17001;   // but the app's later set is taken
    EXPECT_TRUE(r2.offer(app, false));
}

TEST(RawReplayMosaic, ThePvtIsTheLoopsFix)
{
    MosaicFeed f;
    gnss_raw::TimedRawEpoch te;
    ASSERT_EQ(f.onBlock(pvt(kTow0, 0.2, 0.25f, 150.0, -33.5f, -4.25f, 23).data(), 96, 123456789, te),
              MosaicFeed::Kind::PVT);
    const GNSSData& g = f.gnss();
    EXPECT_EQ(g.fix_mode, 3);
    EXPECT_EQ(g.num_sats, 23);
    EXPECT_EQ(g.alt_mm, 183500);        // above the geoid: the height less the undulation
    EXPECT_EQ(g.vel_u_mmps, -4250);
    EXPECT_EQ(g.vel_n_mmps, 500);
    EXPECT_EQ(g.time_us, 123456789u);   // stamped on arrival
    EXPECT_TRUE(f.pvt().has_solution);
    EXPECT_EQ(f.counts().pvt_fix, 1u);
}

// ReceiverTime (5914): the leap seconds and SyncLevel (bit 2 FINETIME).
std::vector<uint8_t> receiverTime(uint32_t tow, uint8_t sync_level)
{
    Body b(24);
    b.u8(20, 18);
    b.u8(21, sync_level);
    return sbftest::block(mosaic::SBF_RECEIVER_TIME, 0, tow, kWnc, b.v);
}

TEST(RawReplayMosaic, TheTimeTagWhereTheDriverWouldGiveIt)
{
    // rawTimeTagOk: the clock line valid, FINETIME, and a 1PPS edge paired within 120 s
    using Why = rr::TimeTagGate::Why;
    const int64_t t = 500000000;
    rr::TimeTagGate g;
    g.pair_us = t - 1000000;
    // a log that begins mid-session: its first ReceiverTime (FINETIME) stands for the ones before it
    g.first_rt = 1;
    EXPECT_EQ(g.judge(false, false, true, t), Why::TIMED);
    // as after a boot: nothing until a ReceiverTime says FINETIME
    g.from_boot = true;
    EXPECT_EQ(g.judge(false, false, true, t), Why::NOT_FINETIME);
    EXPECT_EQ(g.judge(true, true, true, t), Why::TIMED);
    g.from_boot = false;
    // once the receiver has said, its word: not FINETIME is untimed
    EXPECT_EQ(g.judge(true, false, true, t), Why::NOT_FINETIME);
    g.first_rt = 0;
    EXPECT_EQ(g.judge(false, false, true, t), Why::NOT_FINETIME);
    // a log with no ReceiverTime at all: untimed, as the driver would leave it, unless assumed
    g.first_rt = -1;
    EXPECT_EQ(g.judge(false, false, true, t), Why::NOT_FINETIME);
    g.assume_finetime = true;
    EXPECT_EQ(g.judge(false, false, true, t), Why::TIMED);
    g.assume_finetime = false;
    g.first_rt = 1;
    // the 1PPS pairing: under 120 s old (the log's first K line, later than the epoch, stands for the earlier)
    g.pair_us = t - 119999999;
    EXPECT_EQ(g.judge(true, true, true, t), Why::TIMED);
    g.pair_us = t - 120000000;
    EXPECT_EQ(g.judge(true, true, true, t), Why::PPS_STALE);
    g.pair_us = INT64_MIN;
    EXPECT_EQ(g.judge(true, true, true, t), Why::PPS_STALE);
    g.pair_us = t + 5000000;
    EXPECT_EQ(g.judge(true, true, true, t), Why::TIMED);
    // the clock line
    EXPECT_EQ(g.judge(true, true, false, t), Why::NO_LINE);

    // with the feed: a ReceiverTime without FINETIME, then one with it
    MosaicFeed f;
    gnss_raw::TimedRawEpoch te;
    EXPECT_FALSE(f.haveTime());
    const std::vector<uint8_t> coarse = receiverTime(kTow0, 0x03);
    f.onBlock(coarse.data(), coarse.size(), 1000, te);
    ASSERT_TRUE(f.haveTime());
    EXPECT_EQ(g.judge(f.haveTime(), f.finetime(), true, t), Why::NOT_FINETIME);
    const std::vector<uint8_t> fine = receiverTime(kTow0 + 1000, 0x07);
    f.onBlock(fine.data(), fine.size(), 2000, te);
    EXPECT_EQ(g.judge(f.haveTime(), f.finetime(), true, t), Why::TIMED);
}
