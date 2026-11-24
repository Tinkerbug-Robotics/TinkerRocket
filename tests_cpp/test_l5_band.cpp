// Host tests for the L5 band from the PX1105R (TR_RAW_L5, 2026-11-24): its
// 0xE5 cells as the filter's band-1 cells (Px1105rRawEpoch.h l5), the L5-band
// codes moved onto the L1 band's (CodeBridge.h), the Dopplers in pairs kept to
// one band (MosaicRawEpoch.h), and the PX1105R profile's L5-band signals
// (ReceiverProfile.h). The fixture 0xE5 (fixtures/skytraq_frames.h) is the
// 09-26 open-sky capture's: 18 L1-band GPS, Galileo and BeiDou MEO/IGSO cells
// and 17 L5-band ones, G10 on the L5 band alone.
#include <gtest/gtest.h>

#include <cmath>

#include "CodeBridge.h"
#include "MosaicRawEpoch.h"
#include "Px1105rRawEpoch.h"
#include "RawMeas.h"
#include "ReceiverProfile.h"
#include "SkyTraqBinary.h"
#include "fixtures/skytraq_frames.h"
#include "gnss/types.h"

using gnss_raw::CodeBridge;
using gnss_raw::CodeBridgeConfig;
using gnss_raw::kBandL1;
using gnss_raw::kBandL5;
using gnss_raw::RawEpoch;
using gnss_raw::RawObs;

namespace
{

constexpr double kC = 299792458.0;

skytraq::RawE5 fixtureE5()
{
    skytraq::RawE5 e;
    EXPECT_TRUE(skytraq::decodeRawE5(kE5, sizeof kE5, e));
    return e;
}

RawObs cell(uint8_t sys, uint8_t prn, uint8_t band, double pr)
{
    RawObs o{};
    o.sys = sys;
    o.prn = prn;
    o.band = band;
    o.has_pr = true;
    o.has_rate = true;
    o.cn0_dbhz = 40.0f;
    o.pr_m = pr;
    return o;
}

}  // namespace

TEST(Px1105rRawEpochL5, WithoutItTheEpochIsTheL1BandsAsBefore)
{
    const skytraq::RawE5 e = fixtureE5();
    px1105r::Px1105rLockState lk;
    RawEpoch out;
    EXPECT_EQ(px1105r::rawEpochFromE5(e, lk, out), 18);
    for (int i = 0; i < out.n; ++i)
    {
        EXPECT_EQ(out.obs[i].band, kBandL1);
        EXPECT_FALSE(out.obs[i].pr_l1);
    }
}

TEST(Px1105rRawEpochL5, TheL5BandFollowsEveryL1BandCell)
{
    const skytraq::RawE5 e = fixtureE5();
    px1105r::Px1105rLockState lk1, lk5;
    RawEpoch l1, both;
    ASSERT_EQ(px1105r::rawEpochFromE5(e, lk1, l1), 18);
    ASSERT_EQ(px1105r::rawEpochFromE5(e, lk5, both, false, true), 35);
    // the first 18 are the L1-band epoch's, field for field
    for (int i = 0; i < l1.n; ++i)
    {
        SCOPED_TRACE(i);
        const RawObs &a = l1.obs[i], &b = both.obs[i];
        EXPECT_EQ(a.sys, b.sys);
        EXPECT_EQ(a.prn, b.prn);
        EXPECT_EQ(b.band, kBandL1);
        EXPECT_EQ(a.pr_m, b.pr_m);
        EXPECT_EQ(a.rate_mps, b.rate_mps);
        EXPECT_EQ(a.lock_ms, b.lock_ms);
        EXPECT_EQ(a.has_phase, b.has_phase);
        EXPECT_EQ(a.phase_m, b.phase_m);
    }
    // then the L5 band's: GPS, Galileo, BeiDou MEO/IGSO, each on 1176.45 MHz, no carrier phase
    int sys_n[GNSS_SYS_COUNT] = {};
    bool g10 = false;
    for (int i = l1.n; i < both.n; ++i)
    {
        const RawObs& o = both.obs[i];
        EXPECT_EQ(o.band, kBandL5);
        EXPECT_FALSE(o.has_phase);
        EXPECT_FALSE(o.pr_l1);
        ASSERT_LT(o.sys, GNSS_SYS_COUNT);
        sys_n[o.sys]++;
        g10 = g10 || (o.sys == GNSS_SYS_GPS && o.prn == 10);
        // its own cell's Doppler on its own wavelength
        for (int k = 0; k < e.n; ++k)
        {
            const skytraq::RawCell& c = e.cell[k];
            if (c.sig == skytraq::kSigL5 && c.svid == o.prn
                && ((o.sys == GNSS_SYS_GPS && c.gnss == skytraq::kGnssGps)
                    || (o.sys == GNSS_SYS_GAL && c.gnss == skytraq::kGnssGal)
                    || (o.sys == GNSS_SYS_BDS && c.gnss == skytraq::kGnssBds)))
            {
                EXPECT_NEAR(o.rate_mps, -(kC / GNSS_FREQ_L5_HZ) * skytraq::dopplerHz(c.doppler_hz), 1e-9);
                EXPECT_EQ(o.pr_m, c.pr_m);
                EXPECT_EQ(o.cn0_dbhz, static_cast<float>(c.cn0));
            }
        }
    }
    EXPECT_EQ(sys_n[GNSS_SYS_GPS], 5);
    EXPECT_EQ(sys_n[GNSS_SYS_GAL], 6);
    EXPECT_EQ(sys_n[GNSS_SYS_BDS], 6);
    EXPECT_TRUE(g10);                        // a satellite on the L5 band alone is still handed over
}

TEST(Px1105rRawEpochL5, AFullEpochKeepsItsL1BandCells)
{
    // more cells than RawEpoch holds: the L5 band's are the ones left out
    skytraq::RawE5 e = fixtureE5();
    const int n0 = e.n;
    while (e.n < skytraq::RawE5::kMaxCells && e.n < 2 * n0)
    {
        e.cell[e.n] = e.cell[e.n - n0];
        e.n++;
    }
    px1105r::Px1105rLockState lk;
    RawEpoch out;
    ASSERT_EQ(px1105r::rawEpochFromE5(e, lk, out, false, true), RawEpoch::kMaxObs);
    int l1 = 0;
    for (int i = 0; i < out.n; ++i)
        l1 += out.obs[i].band == kBandL1 ? 1 : 0;
    EXPECT_EQ(l1, 36);                       // every L1-band cell (each twice), then 12 of the L5 band's
    for (int i = 0; i < 36; ++i)
        EXPECT_EQ(out.obs[i].band, kBandL1);
}

TEST(Px1105rRawEpochL5, AnL5BandSlipIsItsOwn)
{
    skytraq::RawE5 e = fixtureE5();
    int k5 = -1;
    for (int k = 0; k < e.n && k5 < 0; ++k)
        if (e.cell[k].sig == skytraq::kSigL5 && e.cell[k].gnss == skytraq::kGnssGps && e.cell[k].svid == 23)
            k5 = k;
    ASSERT_GE(k5, 0);
    px1105r::Px1105rLockState lk0;
    RawEpoch out;
    px1105r::rawEpochFromE5(e, lk0, out, false, true);   // the capture's own flags
    e.cell[k5].ind |= skytraq::kIndSlip;
    px1105r::Px1105rLockState lk;
    px1105r::rawEpochFromE5(e, lk, out, false, true);
    EXPECT_EQ(lk.slips, lk0.slips + 1);
    EXPECT_NE(lk.slip_l5_ms[GNSS_SYS_GPS][23], 0);
    EXPECT_EQ(lk.slip_ms[GNSS_SYS_GPS][23], 0);
    for (int i = 0; i < out.n; ++i)
        if (out.obs[i].sys == GNSS_SYS_GPS && out.obs[i].prn == 23)
        {
            if (out.obs[i].band == kBandL5)
                EXPECT_EQ(out.obs[i].lock_ms, 0u);
            else
                EXPECT_EQ(out.obs[i].lock_ms, 204800u);   // the L1 band's lock untouched
        }
}

TEST(CodeBridge, MovesAnL5CodeOntoItsSatellitesL1CodeOnceTheL1CodeIsGone)
{
    CodeBridge br;
    RawEpoch e;
    double tow = 1000.0;
    // 30 epochs of both codes, the L5 band's 27.5 m long (+-0.4 m)
    for (int k = 0; k < 30; ++k, tow += 0.05)
    {
        e.tow_s = tow;
        e.n = 0;
        const double pr1 = 2.2e7 + 100.0 * k;
        e.obs[e.n++] = cell(GNSS_SYS_GPS, 23, kBandL1, pr1);
        e.obs[e.n++] = cell(GNSS_SYS_GPS, 23, kBandL5, pr1 + 27.5 + (k % 2 ? 0.4 : -0.4));
        EXPECT_EQ(br.apply(e), 0);
        EXPECT_FALSE(e.obs[1].pr_l1);        // its satellite's L1-band code is there: left as it came
        EXPECT_EQ(e.obs[1].pr_m, pr1 + 27.5 + (k % 2 ? 0.4 : -0.4));
    }
    double off = 0.0;
    uint32_t n = 0;
    ASSERT_TRUE(br.offsetOf(GNSS_SYS_GPS, 23, off, n));
    EXPECT_NEAR(off, 27.5, 0.1);
    EXPECT_EQ(n, 30u);
    // the L1 band's code gone (its cell there with a Doppler alone): the L5 band's moved
    e.tow_s = tow;
    e.n = 0;
    RawObs d = cell(GNSS_SYS_GPS, 23, kBandL1, 0.0);
    d.has_pr = false;
    e.obs[e.n++] = d;
    e.obs[e.n++] = cell(GNSS_SYS_GPS, 23, kBandL5, 2.3e7);
    EXPECT_EQ(br.apply(e), 1);
    EXPECT_TRUE(e.obs[1].pr_l1);
    EXPECT_NEAR(e.obs[1].pr_m, 2.3e7 - off, 1e-6);
    EXPECT_EQ(br.stats().moved, 1u);
}

TEST(CodeBridge, LeavesACodeWhoseSatellitesDifferenceIsNewOldOrNone)
{
    CodeBridgeConfig cfg;
    CodeBridge br(cfg);
    RawEpoch e;
    // a satellite never seen with both codes
    e.tow_s = 100.0;
    e.n = 0;
    e.obs[e.n++] = cell(GNSS_SYS_GAL, 11, kBandL5, 2.4e7);
    EXPECT_EQ(br.apply(e), 0);
    EXPECT_FALSE(e.obs[0].pr_l1);
    EXPECT_EQ(br.stats().none, 1u);
    // five epochs of both: under min_n
    for (int k = 0; k < 5; ++k)
    {
        e.tow_s = 101.0 + 0.05 * k;
        e.n = 0;
        e.obs[e.n++] = cell(GNSS_SYS_GAL, 11, kBandL1, 2.4e7);
        e.obs[e.n++] = cell(GNSS_SYS_GAL, 11, kBandL5, 2.4e7 + 25.0);
        br.apply(e);
    }
    e.tow_s = 102.0;
    e.n = 0;
    e.obs[e.n++] = cell(GNSS_SYS_GAL, 11, kBandL5, 2.4e7);
    EXPECT_EQ(br.apply(e), 0);
    EXPECT_EQ(br.stats().too_new, 1u);
    // enough of them, then max_age_s and more without: too old
    for (int k = 0; k < 30; ++k)
    {
        e.tow_s = 103.0 + 0.05 * k;
        e.n = 0;
        e.obs[e.n++] = cell(GNSS_SYS_GAL, 11, kBandL1, 2.4e7);
        e.obs[e.n++] = cell(GNSS_SYS_GAL, 11, kBandL5, 2.4e7 + 25.0);
        br.apply(e);
    }
    e.tow_s = 103.0 + 1.45 + cfg.max_age_s - 0.1;
    e.n = 0;
    e.obs[e.n++] = cell(GNSS_SYS_GAL, 11, kBandL5, 2.4e7);
    EXPECT_EQ(br.apply(e), 1);               // just within max_age_s
    e.tow_s = 103.0 + 1.45 + cfg.max_age_s + 0.1;
    e.obs[0] = cell(GNSS_SYS_GAL, 11, kBandL5, 2.4e7);
    EXPECT_EQ(br.apply(e), 0);
    EXPECT_EQ(br.stats().too_old, 1u);
}

TEST(CodeBridge, AStepInTheDifferenceStartsItOver)
{
    CodeBridge br;
    RawEpoch e;
    for (int k = 0; k < 40; ++k)
    {
        e.tow_s = 500.0 + 0.05 * k;
        e.n = 0;
        const double step = k >= 25 ? 29.3 : 0.0;   // an L5 code chip slipped
        e.obs[e.n++] = cell(GNSS_SYS_BDS, 27, kBandL1, 3.0e7);
        e.obs[e.n++] = cell(GNSS_SYS_BDS, 27, kBandL5, 3.0e7 + 30.0 + step);
        br.apply(e);
    }
    EXPECT_EQ(br.stats().restarts, 1u);
    double off = 0.0;
    uint32_t n = 0;
    ASSERT_TRUE(br.offsetOf(GNSS_SYS_BDS, 27, off, n));
    EXPECT_NEAR(off, 59.3, 1e-6);
    EXPECT_EQ(n, 15u);
    // GLONASS and QZSS are not its systems
    EXPECT_FALSE(br.offsetOf(GNSS_SYS_GLO, 3, off, n));
}

TEST(MosaicDopplerPairsL5, APairIsOneBandsAndEachBandItsOwnFloor)
{
    // the grid epoch: G23 on both bands; the epoch before: G23 on the L1 band
    // alone, and G24 on the L5 band at 24 dB-Hz, under the L5 band's 26 floor
    mosaic::MosaicDopplerSet prev;
    prev.valid = true;
    prev.rx_ms = 1000;
    prev.n = 2;
    prev.c[0].sys = GNSS_SYS_GPS;
    prev.c[0].prn = 23;
    prev.c[0].band = kBandL1;
    prev.c[0].has_rate = true;
    prev.c[0].cn0_dbhz = 40.0f;
    prev.c[0].lock_ms = 5000;
    prev.c[0].rate_mps = 100.0;
    prev.c[1] = prev.c[0];
    prev.c[1].prn = 24;
    prev.c[1].band = kBandL5;
    prev.c[1].cn0_dbhz = 24.0f;
    RawEpoch e;
    e.n = 0;
    RawObs a = cell(GNSS_SYS_GPS, 23, kBandL1, 2.2e7);
    a.rate_mps = 101.0;
    a.lock_ms = 5050;
    e.obs[e.n++] = a;
    RawObs b = a;
    b.band = kBandL5;
    b.rate_mps = 102.0;
    e.obs[e.n++] = b;
    RawObs c = a;
    c.prn = 24;
    c.band = kBandL5;
    c.rate_mps = 50.0;
    e.obs[e.n++] = c;
    mosaic::MosaicDopplerPairConfig cfg;
    cfg.dt_ms = 50;
    for (int k = 0; k < GNSS_SYS_COUNT; ++k)
    {
        cfg.cn0_min[k] = 25.0f;
        cfg.cn0_min_l5[k] = 26.0f;
    }
    mosaic::MosaicDopplerPairStats st;
    uint64_t paired = 0;
    // three usable, one pair: under half in pairs, the epoch as it came
    EXPECT_EQ(mosaic::averageDopplerPairs(e, 1050, false, prev, cfg, st, &paired), 0);
    EXPECT_EQ(st.epochs_few, 1u);
    // G24's L5 cell gone: two usable, one pair -- the L1 band's G23 averaged with the L1 band's alone
    e.n = 2;
    EXPECT_EQ(mosaic::averageDopplerPairs(e, 1050, false, prev, cfg, st, &paired), 1);
    EXPECT_EQ(paired, 1u);
    EXPECT_DOUBLE_EQ(e.obs[0].rate_mps, 100.5);
    EXPECT_EQ(st.alone_mate, 1u);            // G23's L5 cell: no L5 mate in the epoch before
    // the L5 band's floor is its own: G24's 24 dB-Hz end refused as under it
    // (beside G23's L1-band pair, so the epoch goes in pairs)
    RawEpoch f;
    f.n = 0;
    f.obs[f.n++] = a;
    f.obs[f.n++] = c;
    mosaic::MosaicDopplerPairStats st2;
    mosaic::averageDopplerPairs(f, 1050, false, prev, cfg, st2);
    EXPECT_EQ(st2.alone_cn0, 1u);
}

TEST(ReceiverProfileL5, ThePx1105rTakesTheL5BandsDopplersAndNoProfileItsCode)
{
    const gnss_raw::GnssReceiverProfile p = gnss_raw::px1105rProfile();
    for (int k = GNSS_SYS_GPS; k <= GNSS_SYS_BDS; ++k)
    {
        SCOPED_TRACE(k);
        const gnss_raw::GnssSignalProfile& s = p.sig_l5[k];
        EXPECT_EQ(s.carrier_hz, GNSS_FREQ_L5_HZ);
        EXPECT_EQ(s.cn0_min_pr, 25.0f);
        EXPECT_EQ(s.cn0_min_rr, 25.0f);
        EXPECT_FALSE(s.pr_fused);            // its code only moved onto the L1 band's (CodeBridge.h)
        EXPECT_FALSE(s.half_cycle_unflagged);
        EXPECT_NEAR(s.rr_sigma_scale, GNSS_FREQ_L1_HZ / GNSS_FREQ_L5_HZ, 1e-6);
    }
    EXPECT_EQ(p.rr_lag_l5_s, 0.040);
    // every other profile's L5 band: the default, its code never fused, no floors of its own
    for (const gnss_raw::GnssReceiverProfile& q :
         {gnss_raw::lc86gProfile(), gnss_raw::mosaicG5Profile(), gnss_raw::sdrDevProfile()})
    {
        SCOPED_TRACE(q.name);
        EXPECT_EQ(q.rr_lag_l5_s, 0.0);
        for (int k = 0; k < gnss_raw::kProfileSys; ++k)
        {
            EXPECT_EQ(q.sig_l5[k].carrier_hz, GNSS_FREQ_L5_HZ);
            EXPECT_FALSE(q.sig_l5[k].pr_fused);
            EXPECT_EQ(q.sig_l5[k].cn0_min_rr, 0.0f);
        }
        // the L1 band's Doppler model unscaled
        for (int k = 0; k < gnss_raw::kProfileSys; ++k)
            EXPECT_EQ(q.sig[k].rr_sigma_scale, 1.0f);
    }
}

TEST(RawMeasL5, TheBandFieldsSitInPaddingSoTheStructsKeepTheirSize)
{
    EXPECT_EQ(sizeof(RawObs), 40u);
    EXPECT_EQ(sizeof(gnss_raw::RawMeas), 160u);
    RawObs o{GNSS_SYS_GPS, 7, true, true, 40.0f, 2.2e7, 100.0, 0};
    EXPECT_EQ(o.band, kBandL1);
    EXPECT_FALSE(o.pr_l1);
    gnss_raw::RawMeas m{};
    EXPECT_EQ(m.band, kBandL1);
}
