// Host tests for the mosaic-G5's Dopplers in pairs
// (TR_GNSSReceiverMosaic_Serial/MosaicRawEpoch.h, exp/v9-dop-avg): the
// off-grid epoch read for its Dopplers alone gives the very cells the full
// adapter would (the same picks, duplicates, units, C/N0 and lock time) and
// follows the clock jumps as noteMeasEpochClock does; a grid epoch's Doppler
// becomes the mean of the pair only when the epoch before is exactly 50 ms
// earlier and read whole, no clock jump came between, the satellite's
// Doppler is valid and above the floor at both ends, its lock did not go
// down, and the two are within the bound -- each refusal counted once, by the
// first rule it failed, and its Doppler left exactly as the receiver gave it;
// and the harness's feed (tools/raw_replay/mosaic_feed.h) pairs as the
// driver does, and with the pairs off hands over what it always did.
// Synthetic blocks only (sbf_meas_build.h).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "MosaicRawEpoch.h"
#include "mosaic_feed.h"
#include "sbf_meas_build.h"

using namespace mosaic;
using gnss_raw::RawEpoch;
using gnss_raw::RawObs;
using sbftest::MeasEpochSpec;
using sbftest::MeasT1;
using sbftest::MeasT2;

namespace {

constexpr double kC = 299792458.0;
constexpr double kLamL1 = kC / 1575.42e6;
constexpr uint16_t kWnc = 2440;
constexpr uint32_t kTow0 = 507606000;   // a whole second: on the 100 ms grid
constexpr int32_t kDopDnu = -2147483647 - 1;

int64_t rxMs(uint32_t tow) { return static_cast<int64_t>(kWnc) * 604800000LL + tow; }

// A GPS L1 C/A master: its Doppler (Hz), C/N0 (raw: 0.25 dB-Hz + 10) and lock (s).
MeasT1 gps(uint8_t svid, double dop_hz, uint8_t cn0 = 128, uint16_t lock = 600)
{
    MeasT1 t;
    t.rx_channel = svid;
    t.svid = svid;
    t.sig(0);
    t.prM(2.1e7 + 1.0e4 * svid).dopplerHz(dop_hz);
    t.carrier(-1000 - svid);
    t.cn0 = cn0;
    t.lock = lock;
    return t;
}

MeasT1 gal(uint8_t prn, double dop_hz, uint16_t lock = 600)
{
    MeasT1 t = gps(static_cast<uint8_t>(70 + prn), dop_hz, 128, lock);
    t.sig(17);
    return t;
}

std::vector<uint8_t> epoch(uint32_t tow, const std::vector<MeasT1>& sats, uint8_t cum = 0)
{
    MeasEpochSpec s;
    s.tow = tow;
    s.wnc = kWnc;
    s.cum = cum;
    s.t1 = sats;
    return sbftest::measEpoch(s);
}

const RawObs* cell(const RawEpoch& ep, int sys, int prn)
{
    for (int i = 0; i < ep.n; ++i)
        if (ep.obs[i].sys == sys && ep.obs[i].prn == prn) return &ep.obs[i];
    return nullptr;
}

MosaicDopplerPairConfig pairConfig()
{
    MosaicDopplerPairConfig c;
    for (int k = 0; k < GNSS_SYS_COUNT; ++k)
        c.cn0_min[k] = 25.0f;   // the mosaic profile's cn0_min_rr
    return c;
}

// Every field of the jump state (its padding is not compared)
void expectSameJumps(const MosaicJumpState& a, const MosaicJumpState& b)
{
    EXPECT_EQ(a.have, b.have);
    EXPECT_EQ(a.last_cum, b.last_cum);
    EXPECT_EQ(a.last_ms, b.last_ms);
    EXPECT_EQ(a.j_ms, b.j_ms);
    EXPECT_EQ(a.jumps, b.jumps);
    EXPECT_EQ(a.rebases, b.rebases);
    EXPECT_EQ(a.cut, b.cut);
    EXPECT_EQ(a.cut_ms, b.cut_ms);
    EXPECT_EQ(a.jsum_ms, b.jsum_ms);
    EXPECT_EQ(a.resets, b.resets);
    EXPECT_EQ(a.resolved, b.resolved);
    EXPECT_EQ(a.arr_have, b.arr_have);
    EXPECT_EQ(a.arr_hi_us, b.arr_hi_us);
    EXPECT_EQ(a.late_us, b.late_us);
    EXPECT_EQ(a.last_arr_us, b.last_arr_us);
}

uint32_t accounted(const MosaicDopplerPairStats& st)
{
    return st.pairs + st.alone_epoch + st.alone_mate + st.alone_cn0 + st.alone_lock + st.alone_diff;
}

}  // namespace

TEST(MosaicDopplerPair, TheOffGridReadIsTheAdaptersCells)
{
    // the picks of every kind: masters and Type2s, Galileo, BeiDou MEO and
    // GEO, GLONASS, an aux antenna, a satellite on two channels (the second
    // with the code the first lacks), a lost lock and a lost Doppler
    MeasEpochSpec s;
    s.tow = kTow0 + 50;
    s.wnc = kWnc;
    s.t1.push_back(gps(3, -1234.5678));
    MeasT1 g8 = gps(8, 0.0);
    g8.sig(4);                                         // L5 master, L1 C/A as its Type2
    MeasT2 l1;
    l1.sig(0);
    l1.codeOffsetMm(1234);
    l1.dopplerOffset(-567);
    l1.lock = 17;
    l1.cn0 = 100;
    g8.t2 = {l1};
    g8.dopplerHz(1111.25);
    s.t1.push_back(g8);
    s.t1.push_back(gal(11, 2345.125));
    MeasT1 c19 = gps(140 + 19, -333.0);
    c19.sig(28);                                       // BeiDou C19 B1I
    s.t1.push_back(c19);
    MeasT1 c03 = gps(140 + 3, 10.0);
    c03.sig(28);                                       // GEO: left out
    s.t1.push_back(c03);
    MeasT1 r02 = gps(40, 50.0);
    r02.sig(8);                                        // GLONASS: not taken
    s.t1.push_back(r02);
    MeasT1 aux = gps(12, 70.0);
    aux.sig(0, 1);                                     // Aux1: not taken
    s.t1.push_back(aux);
    MeasT1 g20a = gps(20, -50.0);
    g20a.codeMm(0);                                    // no code: a second copy replaces it
    s.t1.push_back(g20a);
    MeasT1 g20b = gps(20, -51.0);
    g20b.rx_channel = 33;
    s.t1.push_back(g20b);
    s.t1.push_back(gps(21, 75.0, 128, 65535));         // no lock time
    MeasT1 g22 = gps(22, 0.0);
    g22.doppler = kDopDnu;                             // no Doppler
    s.t1.push_back(g22);
    s.t1.push_back(gps(23, 99.0, 255));                // C/N0 do-not-use: no measurement at all
    const std::vector<uint8_t> blk = sbftest::measEpoch(s);

    MosaicJumpState js_a, js_b;
    MosaicEpochInfo info_a, info_b;
    RawEpoch ep;
    ASSERT_TRUE(rawEpochFromMeasEpoch(blk.data(), blk.size(), ep, js_a, info_a, 12345));
    MosaicDopplerSet set;
    ASSERT_TRUE(noteMeasEpochDopplers(blk.data(), blk.size(), js_b, info_b, set, 12345));
    ASSERT_TRUE(set.valid);
    EXPECT_EQ(set.rx_ms, rxMs(kTow0 + 50));
    ASSERT_EQ(set.n, ep.n);
    EXPECT_GE(ep.n, 8);
    for (int i = 0; i < ep.n; ++i)
    {
        SCOPED_TRACE(testing::Message() << "cell " << i);
        const RawObs& o = ep.obs[i];
        const MosaicDopplerCell& c = set.c[i];
        EXPECT_EQ(c.sys, o.sys);
        EXPECT_EQ(c.prn, o.prn);
        EXPECT_EQ(c.has_pr, o.has_pr);
        EXPECT_EQ(c.has_rate, o.has_rate);
        EXPECT_EQ(c.cn0_dbhz, o.cn0_dbhz);
        EXPECT_EQ(c.lock_ms, o.lock_ms);
        EXPECT_EQ(c.rate_mps, o.rate_mps);   // bit for bit: the same expression
    }
    EXPECT_EQ(cell(ep, GNSS_SYS_GPS, 20)->rate_mps, -kLamL1 * -51.0);   // the copy with the code
    EXPECT_FALSE(cell(ep, GNSS_SYS_GPS, 22)->has_rate);
    // the header's work is the same: the jump state and the epoch's info
    EXPECT_EQ(info_a.status, info_b.status);
    EXPECT_EQ(info_a.tow_ms, info_b.tow_ms);
    EXPECT_EQ(info_a.lock_cap_ms, info_b.lock_cap_ms);
    expectSameJumps(js_a, js_b);
}

TEST(MosaicDopplerPair, TheOffGridReadFollowsTheClockAsNoteMeasEpochClock)
{
    // a run of epochs with a -1 ms jump, a +2, a big step (a start-over) and
    // time going back (a reset): the two readers leave the same state
    const uint8_t cums[] = {10, 10, 9, 9, 11, 11, 40, 40, 40};
    const int32_t dt[] = {0, 50, 100, 150, 200, 250, 300, 350, 100};
    MosaicJumpState a, b;
    for (size_t k = 0; k < sizeof(cums); ++k)
    {
        SCOPED_TRACE(testing::Message() << "epoch " << k);
        const std::vector<uint8_t> blk = epoch(kTow0 + dt[k], {gps(3, -100.0), gps(5, 200.0)}, cums[k]);
        MosaicEpochInfo ia, ib;
        MosaicDopplerSet set;
        const int64_t arr = 1000000 + 50000 * static_cast<int64_t>(k);
        EXPECT_TRUE(noteMeasEpochClock(blk.data(), blk.size(), a, ia, arr));
        EXPECT_TRUE(noteMeasEpochDopplers(blk.data(), blk.size(), b, ib, set, arr));
        expectSameJumps(a, b);
        EXPECT_EQ(ia.jumped, ib.jumped);
        EXPECT_EQ(ia.rebased, ib.rebased);
        EXPECT_EQ(ia.reset, ib.reset);
        EXPECT_EQ(ia.j_ms, ib.j_ms);
    }
    // a block the reader refuses: no set, and no state moved
    MosaicJumpState c = b;
    std::vector<uint8_t> bad = epoch(kTow0 + 400, {gps(3, 1.0)});
    bad.resize(bad.size() - 4);
    MosaicEpochInfo ic;
    MosaicDopplerSet set;
    set.valid = true;
    EXPECT_FALSE(noteMeasEpochDopplers(bad.data(), bad.size(), c, ic, set));
    EXPECT_FALSE(set.valid);
    expectSameJumps(c, b);
}

TEST(MosaicDopplerPair, APairIsTheMeanAndEveryOtherDopplerIsLeftAsItCame)
{
    // the epoch before (T - 50 ms) and the grid epoch (T): one satellite for each rule
    const double h = 50.0;   // Hz between the two: 9.5 m/s
    std::vector<MeasT1> before = {
        gps(3, -1500.0),                        // a pair
        gps(5, -900.0).dopplerHz(-900.0),       // its Doppler lost before: alone (mate)
        gps(6, 400.0, 59),                      // 24.75 dB-Hz before: alone (C/N0)
        gps(7, 250.0, 128, 30),                 // lock 30 s before, 0 now: a relock, alone (lock)
        gps(8, 100.0),                          // 85 Hz apart: 16 m/s, past the bound, alone (diff)
        gps(9, 300.0),                          // under the floor now: not a usable Doppler at all
        gps(10, -700.0 - h),                    // a pair 9.5 m/s apart: within the bound
        gal(11, 2000.0),                        // a pair, Galileo
    };
    before[1].doppler = kDopDnu;
    std::vector<MeasT1> grid = {
        gps(3, -1501.0),
        gps(4, 50.0),                           // not in the epoch before: alone (mate)
        gps(5, -901.0),
        gps(6, 401.0),
        gps(7, 251.0, 128, 0),
        gps(8, 185.0),
        gps(9, 301.0, 56),                      // 24 dB-Hz
        gps(10, -700.0),
        gal(11, 2000.5),
    };
    const std::vector<uint8_t> b0 = epoch(kTow0 - 50, before);
    const std::vector<uint8_t> b1 = epoch(kTow0, grid);
    MosaicJumpState js;
    MosaicEpochInfo info;
    MosaicDopplerSet prev;
    ASSERT_TRUE(noteMeasEpochDopplers(b0.data(), b0.size(), js, info, prev));
    RawEpoch ep;
    ASSERT_TRUE(rawEpochFromMeasEpoch(b1.data(), b1.size(), ep, js, info));
    ASSERT_FALSE(info.jumped || info.rebased);
    const RawEpoch own = ep;

    MosaicDopplerPairStats st;
    uint64_t mask = 0;
    const int n = averageDopplerPairs(ep, rxMs(kTow0), false, prev, pairConfig(), st, &mask);
    EXPECT_EQ(n, 3);
    EXPECT_EQ(st.epochs, 1u);
    EXPECT_EQ(st.dopplers, 8u);   // all but G09
    EXPECT_EQ(st.pairs, 3u);
    EXPECT_EQ(st.alone_mate, 2u);
    EXPECT_EQ(st.alone_cn0, 1u);
    EXPECT_EQ(st.alone_lock, 1u);
    EXPECT_EQ(st.alone_diff, 1u);
    EXPECT_EQ(st.alone_epoch, 0u);
    EXPECT_EQ(accounted(st), st.dopplers);
    EXPECT_NEAR(st.diff_max_mps, kLamL1 * h, 1e-4);
    for (int i = 0; i < ep.n; ++i)
    {
        SCOPED_TRACE(testing::Message() << "prn " << int(ep.obs[i].prn) << " sys " << int(ep.obs[i].sys));
        const bool paired = (mask >> i) & 1u;
        const bool want = (ep.obs[i].sys == GNSS_SYS_GPS && (ep.obs[i].prn == 3 || ep.obs[i].prn == 10)) ||
                          ep.obs[i].sys == GNSS_SYS_GAL;
        EXPECT_EQ(paired, want);
        if (!paired)
        {
            EXPECT_EQ(ep.obs[i].rate_mps, own.obs[i].rate_mps);   // exactly the receiver's
            continue;
        }
        const MosaicDopplerCell* m = nullptr;
        for (int j = 0; j < prev.n; ++j)
            if (prev.c[j].sys == ep.obs[i].sys && prev.c[j].prn == ep.obs[i].prn) m = &prev.c[j];
        ASSERT_NE(m, nullptr);
        EXPECT_EQ(ep.obs[i].rate_mps, 0.5 * (own.obs[i].rate_mps + m->rate_mps));
        // everything else the cell carries is the grid epoch's own
        EXPECT_EQ(ep.obs[i].cn0_dbhz, own.obs[i].cn0_dbhz);
        EXPECT_EQ(ep.obs[i].lock_ms, own.obs[i].lock_ms);
        EXPECT_EQ(ep.obs[i].pr_m, own.obs[i].pr_m);
        EXPECT_EQ(ep.obs[i].phase_m, own.obs[i].phase_m);
    }
    // G03: the mean of -1500 and -1501 Hz, as a range rate
    EXPECT_NEAR(cell(ep, GNSS_SYS_GPS, 3)->rate_mps, -kLamL1 * -1500.5, 1e-9);
}

TEST(MosaicDopplerPair, NoPairWithoutTheEpochJustBeforeOrAcrossAClockJump)
{
    const std::vector<MeasT1> sats = {gps(3, -1500.0), gps(4, 600.0)};
    MosaicDopplerPairConfig c = pairConfig();
    auto run = [&](uint32_t tow_prev, bool read_prev, bool cut, MosaicDopplerPairStats& st) {
        const std::vector<uint8_t> b0 = epoch(tow_prev, sats);
        const std::vector<uint8_t> b1 = epoch(kTow0, {gps(3, -1501.0), gps(4, 601.0)});
        MosaicJumpState js;
        MosaicEpochInfo info;
        MosaicDopplerSet prev;
        if (read_prev)
            EXPECT_TRUE(noteMeasEpochDopplers(b0.data(), b0.size(), js, info, prev));
        RawEpoch ep;
        EXPECT_TRUE(rawEpochFromMeasEpoch(b1.data(), b1.size(), ep, js, info));
        const RawEpoch own = ep;
        const int n = averageDopplerPairs(ep, rxMs(kTow0), cut, prev, c, st);
        for (int i = 0; i < ep.n; ++i)
            EXPECT_EQ(ep.obs[i].rate_mps, own.obs[i].rate_mps);
        return n;
    };
    MosaicDopplerPairStats st;
    EXPECT_EQ(run(kTow0 - 100, true, false, st), 0);   // the epoch before lost: the one before that is no pair
    EXPECT_EQ(run(kTow0 - 50, false, false, st), 0);   // none read (refused, or the pairs just turned on)
    EXPECT_EQ(run(kTow0 - 50, true, true, st), 0);     // a clock jump between
    EXPECT_EQ(st.epochs, 3u);
    EXPECT_EQ(st.epochs_no_prev, 2u);
    EXPECT_EQ(st.epochs_jump, 1u);
    EXPECT_EQ(st.alone_epoch, 6u);
    EXPECT_EQ(st.pairs, 0u);
    EXPECT_EQ(accounted(st), st.dopplers);

    // the jump as the adapter sees it: CumClkJumps one down at the grid epoch
    const std::vector<uint8_t> b0 = epoch(kTow0 - 50, sats, 7);
    const std::vector<uint8_t> b1 = epoch(kTow0, {gps(3, -1501.0), gps(4, 601.0)}, 6);
    MosaicJumpState js;
    MosaicEpochInfo info;
    MosaicDopplerSet prev;
    ASSERT_TRUE(noteMeasEpochDopplers(b0.data(), b0.size(), js, info, prev));
    RawEpoch ep;
    ASSERT_TRUE(rawEpochFromMeasEpoch(b1.data(), b1.size(), ep, js, info));
    EXPECT_TRUE(info.jumped);
    MosaicDopplerPairStats sj;
    EXPECT_EQ(averageDopplerPairs(ep, rxMs(kTow0), info.jumped || info.rebased, prev, c, sj), 0);
    EXPECT_EQ(sj.epochs_jump, 1u);
}

TEST(MosaicDopplerPair, TheBoundIsInclusiveAndConfigurable)
{
    // 15 m/s apart exactly pairs; just past it does not; a tighter bound refuses it
    RawEpoch ep;
    ep.n = 1;
    ep.obs[0].sys = GNSS_SYS_GAL;
    ep.obs[0].prn = 4;
    ep.obs[0].has_rate = true;
    ep.obs[0].cn0_dbhz = 40.0f;
    ep.obs[0].rate_mps = 115.0;
    ep.obs[0].lock_ms = 5000;
    RawEpoch before = ep;
    before.obs[0].rate_mps = 100.0;
    MosaicDopplerSet prev;
    dopplerSetFromRawEpoch(before, rxMs(kTow0 - 50), prev);
    MosaicDopplerPairStats st;
    RawEpoch e1 = ep;
    EXPECT_EQ(averageDopplerPairs(e1, rxMs(kTow0), false, prev, pairConfig(), st), 1);
    EXPECT_EQ(e1.obs[0].rate_mps, 107.5);
    RawEpoch e2 = ep;
    e2.obs[0].rate_mps = 115.001;
    EXPECT_EQ(averageDopplerPairs(e2, rxMs(kTow0), false, prev, pairConfig(), st), 0);
    EXPECT_EQ(e2.obs[0].rate_mps, 115.001);
    MosaicDopplerPairConfig tight = pairConfig();
    tight.max_diff_mps = 5.0;
    RawEpoch e3 = ep;
    EXPECT_EQ(averageDopplerPairs(e3, rxMs(kTow0), false, prev, tight, st), 0);
    EXPECT_EQ(st.alone_diff, 2u);
    // the floor per system: Galileo's raised past this C/N0, none paired; the grid's own under it, none counted
    MosaicDopplerPairConfig gal_floor = pairConfig();
    gal_floor.cn0_min[GNSS_SYS_GAL] = 45.0f;
    MosaicDopplerPairStats sf;
    RawEpoch e4 = ep;
    EXPECT_EQ(averageDopplerPairs(e4, rxMs(kTow0), false, prev, gal_floor, sf), 0);
    EXPECT_EQ(sf.dopplers, 0u);
    EXPECT_EQ(sf.epochs, 0u);
    EXPECT_EQ(e4.obs[0].rate_mps, 115.0);
}

TEST(MosaicDopplerPair, TheHarnessFeedPairsAsTheDriver)
{
    // the feed with the pairs on: the off-grid epoch read, the grid one averaged; off, the grid epoch as always
    auto blk = [](uint32_t tow, double d3, double d4) { return epoch(tow, {gps(3, d3), gps(4, d4)}); };
    const std::vector<uint8_t> b0 = blk(kTow0 - 50, -1500.0, 600.0);
    const std::vector<uint8_t> b1 = blk(kTow0, -1502.0, 604.0);
    auto arrival = [](uint32_t tow) { return 1000000 + (static_cast<int64_t>(tow) - kTow0) * 1000 + 13000; };

    rr::MosaicFeed off;   // 100 ms grid
    gnss_raw::TimedRawEpoch te_off;
    EXPECT_EQ(off.onBlock(b0.data(), b0.size(), arrival(kTow0 - 50), te_off), rr::MosaicFeed::Kind::EPOCH_SKIPPED);
    ASSERT_EQ(off.onBlock(b1.data(), b1.size(), arrival(kTow0), te_off), rr::MosaicFeed::Kind::EPOCH);
    EXPECT_EQ(off.lastPaired(), 0u);
    EXPECT_EQ(off.pairStats().epochs, 0u);

    rr::MosaicFeed::Config c;
    c.pair = true;
    c.pair_cfg = pairConfig();
    rr::MosaicFeed on(c);
    gnss_raw::TimedRawEpoch te_on;
    EXPECT_EQ(on.onBlock(b0.data(), b0.size(), arrival(kTow0 - 50), te_on), rr::MosaicFeed::Kind::EPOCH_SKIPPED);
    ASSERT_EQ(on.onBlock(b1.data(), b1.size(), arrival(kTow0), te_on), rr::MosaicFeed::Kind::EPOCH);
    EXPECT_EQ(on.lastPaired(), 3u);
    EXPECT_EQ(on.pairStats().pairs, 2u);
    EXPECT_EQ(on.counts().off_grid, off.counts().off_grid);
    ASSERT_EQ(te_on.epoch.n, te_off.epoch.n);
    for (int i = 0; i < te_on.epoch.n; ++i)
    {
        EXPECT_NE(te_on.epoch.obs[i].rate_mps, te_off.epoch.obs[i].rate_mps);
        EXPECT_EQ(te_on.epoch.obs[i].pr_m, te_off.epoch.obs[i].pr_m);
    }
    EXPECT_NEAR(cell(te_on.epoch, GNSS_SYS_GPS, 3)->rate_mps, -kLamL1 * -1501.0, 1e-9);
    EXPECT_NEAR(cell(te_on.epoch, GNSS_SYS_GPS, 4)->rate_mps, -kLamL1 * 602.0, 1e-9);
    // the next grid epoch has no mate: the off-grid one between was lost
    const std::vector<uint8_t> b3 = blk(kTow0 + 100, -1504.0, 608.0);
    ASSERT_EQ(on.onBlock(b3.data(), b3.size(), arrival(kTow0 + 100), te_on), rr::MosaicFeed::Kind::EPOCH);
    EXPECT_EQ(on.lastPaired(), 0u);
    EXPECT_EQ(on.pairStats().epochs_no_prev, 1u);
    EXPECT_NEAR(cell(te_on.epoch, GNSS_SYS_GPS, 3)->rate_mps, -kLamL1 * -1504.0, 1e-9);
}
