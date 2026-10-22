// Host tests for the multi-band combiner (TR_GNSSReceiverMosaic_Serial/
// MosaicMultiband.h; the harness's --rr-multiband, no firmware includes it):
//   - off, the cells and the Doppler sets are rawEpochFromMeasEpoch's and
//     noteMeasEpochDopplers' to the bit; on with nothing to join, too; on
//     with bands, every anchor field but the range rate is theirs;
//   - the scaling: each band's range rate from its own wavelength, wherever
//     the anchor sits (a Type1, or a Type2 under an L5 master), so a known
//     range rate and wind-up come back as the anchor's;
//   - the weights: inverse variance from each band's own C/N0;
//   - the wind-up: referred to the anchor's wavelength (the row carries its
//     anchor's), or left at the weighted wavelength the side outputs give;
//     too few values and the epoch goes anchor-only;
//   - the screen: a slipped band out, an anchor voted out by two agreeing
//     bands, a majority off and the epoch anchor-only;
//   - the join rule: the lock (none, 0 s, 1 s), the floor's hysteresis, the
//     half-cycle flag gating nothing, the anchor's channel only (a satellite on
//     two channels), at most max_bands, and composition changes counted;
//   - the pairs: both ends combined, the pair averaged on the combined rates
//     and a voted-out end dropping its pair; the paired row's sigma ratio;
//   - the effective C/N0 round trip and its clamps.
// Synthetic blocks (sbf_meas_build.h); behind TR_SBF_PRIVATE_CAPTURE, real
// captures for the identities (statistics only, never a position).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

#include "MosaicMultiband.h"
#include "MosaicRawEpoch.h"
#include "mosaic_feed.h"
#include "sbf_meas_build.h"
#include "sbf_private_capture.h"

using namespace mosaic;
using gnss_raw::RawEpoch;
using gnss_raw::RawObs;
using sbftest::MeasEpochSpec;
using sbftest::MeasT1;
using sbftest::MeasT2;

namespace {

constexpr double kC = 299792458.0;
constexpr uint16_t kWnc = 2440;
constexpr uint32_t kTow0 = 507606000;   // a whole second: on the 100 ms grid

double lam(uint8_t sig) { return kC / sbfCarrierHz(sig); }

// One band of a satellite: its signal, its range rate (m/s), C/N0 (raw: 0.25 dB-Hz + 10), lock (s) and ObsInfo.
struct Band
{
    uint8_t sig;
    double  rr;
    uint8_t cn0 = 140;   // 45 dB-Hz
    uint8_t lock = 30;
    uint8_t obs = 0;
};

// A satellite: its master band (a Type1), the others as its Type2s, each Doppler -rr / lambda.
MeasT1 sat(uint8_t svid, uint8_t ch, const Band& m, const std::vector<Band>& t2s)
{
    MeasT1 t;
    t.rx_channel = ch;
    t.svid = svid;
    t.sig(m.sig);
    t.prM(2.1e7 + 1.0e4 * svid);
    t.dopplerHz(-m.rr / lam(m.sig));
    t.carrier(-1000 - svid);
    t.cn0 = m.cn0;
    t.lock = m.lock;
    t.obs = m.obs;
    const double dm_q = t.doppler * 1.0e-4;
    for (const Band& b : t2s)
    {
        MeasT2 u;
        u.sig(b.sig);
        u.lock = b.lock;
        u.cn0 = b.cn0;
        u.obs = b.obs;
        u.codeOffsetMm(123);
        u.carrier(456);
        const double off = -b.rr / lam(b.sig) - dm_q * sbfCarrierHz(b.sig) / sbfCarrierHz(m.sig);
        u.dopplerOffset(static_cast<int>(std::lround(off * 1.0e4)));
        t.t2.push_back(u);
    }
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

// The truth on band b: R + W lambda_b + noise.
double rrOf(double R, double W, uint8_t sig, double n = 0.0) { return R + W * lam(sig) + n; }

// GPS (L5 master, L1 C/A and L2C as Type2s), Galileo (E1 master, E5a, E5b, E6) and BeiDou C20 (B1I master, B2a,
// B3I, B1C): range rates R, wind-up W Hz, with noise n[sig] where given.
std::vector<MeasT1> sky(double W, const double* n = nullptr, double R0 = -350.0)
{
    auto nz = [&](uint8_t sig) { return n != nullptr ? n[sig] : 0.0; };
    const double Rg = R0, Re = R0 + 812.5, Rb = R0 - 233.25;
    std::vector<MeasT1> v;
    v.push_back(sat(5, 5, {4, rrOf(Rg, W, 4, nz(4))}, {{0, rrOf(Rg, W, 0, nz(0))}, {3, rrOf(Rg, W, 3, nz(3)), 128}}));
    v.push_back(sat(70 + 11, 11, {17, rrOf(Re, W, 17, nz(17))},
                    {{20, rrOf(Re, W, 20, nz(20)), 132}, {21, rrOf(Re, W, 21, nz(21)), 136},
                     {19, rrOf(Re, W, 19, nz(19)), 124}}));
    v.push_back(sat(140 + 20, 20, {28, rrOf(Rb, W, 28, nz(28))},
                    {{14, rrOf(Rb, W, 14, nz(14)), 128}, {30, rrOf(Rb, W, 30, nz(30)), 136},
                     {13, rrOf(Rb, W, 13, nz(13)), 138}}));
    return v;
}

MosaicMultibandConfig measured()
{
    MosaicMultibandConfig c;
    mosaicMbMeasured(c);
    c.on = true;
    return c;
}

const RawObs* cell(const RawEpoch& ep, int sys, int prn)
{
    for (int i = 0; i < ep.n; ++i)
        if (ep.obs[i].sys == sys && ep.obs[i].prn == prn) return &ep.obs[i];
    return nullptr;
}

int cellIndex(const RawEpoch& ep, int sys, int prn)
{
    for (int i = 0; i < ep.n; ++i)
        if (ep.obs[i].sys == sys && ep.obs[i].prn == prn) return i;
    return -1;
}

// Every field of two cells; with rate false, all but the range rate and has_rate.
void expectSameCell(const RawObs& a, const RawObs& b, bool rate = true)
{
    EXPECT_EQ(a.sys, b.sys);
    EXPECT_EQ(a.prn, b.prn);
    EXPECT_EQ(a.has_pr, b.has_pr);
    EXPECT_EQ(a.cn0_dbhz, b.cn0_dbhz);
    EXPECT_EQ(a.pr_m, b.pr_m);
    EXPECT_EQ(a.lock_ms, b.lock_ms);
    EXPECT_EQ(a.has_phase, b.has_phase);
    EXPECT_EQ(a.halfcyc, b.halfcyc);
    EXPECT_EQ(a.phase_m, b.phase_m);
    if (rate)
    {
        EXPECT_EQ(a.has_rate, b.has_rate);
        EXPECT_EQ(a.rate_mps, b.rate_mps);
    }
}

void expectSameInfo(const MosaicEpochInfo& a, const MosaicEpochInfo& b)
{
    EXPECT_EQ(a.status, b.status);
    EXPECT_EQ(a.tow_ms, b.tow_ms);
    EXPECT_EQ(a.wnc, b.wnc);
    EXPECT_EQ(a.common_flags, b.common_flags);
    EXPECT_EQ(a.jumped, b.jumped);
    EXPECT_EQ(a.rebased, b.rebased);
    EXPECT_EQ(a.reset, b.reset);
    EXPECT_EQ(a.jump_ms, b.jump_ms);
    EXPECT_EQ(a.jump_full_ms, b.jump_full_ms);
    EXPECT_EQ(a.lock_cap_ms, b.lock_cap_ms);
    EXPECT_EQ(a.j_ms, b.j_ms);
    EXPECT_EQ(a.n_gps, b.n_gps);
    EXPECT_EQ(a.n_gal, b.n_gal);
    EXPECT_EQ(a.n_bds, b.n_bds);
    EXPECT_EQ(a.geo, b.geo);
    EXPECT_EQ(a.dupes, b.dupes);
    EXPECT_EQ(a.over, b.over);
}

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

// The combiner's state, owned as the driver would own it: off the stack.
struct Mb
{
    MosaicMultibandConfig cfg = measured();
    std::unique_ptr<MosaicMbScratch> scr{new MosaicMbScratch()};
    MosaicMultibandStats st;
    MosaicMbOut out;
    MosaicJumpState js;
    MosaicEpochInfo info;

    bool grid(const std::vector<uint8_t>& b, RawEpoch& ep, int64_t arr = kMosaicNoArrival)
    {
        return rawEpochFromMeasEpochMb(b.data(), b.size(), ep, js, info, cfg, *scr, st, out, arr);
    }
    bool off(const std::vector<uint8_t>& b, MosaicDopplerSet& set, int64_t arr = kMosaicNoArrival)
    {
        return noteMeasEpochDopplersMb(b.data(), b.size(), js, info, set, cfg, *scr, st, out, arr);
    }
};

// The picks of every kind (test_mosaic_doppler_pair's), with extra bands on most.
std::vector<uint8_t> everyKind(uint32_t tow)
{
    MeasEpochSpec s;
    s.tow = tow;
    s.wnc = kWnc;
    s.t1.push_back(sat(3, 3, {0, -1234.5678 * lam(0)}, {{3, -1234.5678 * lam(0) + 0.01}, {4, -1234.5678 * lam(0)}}));
    MeasT1 g8 = sat(8, 8, {4, 300.0}, {{0, 300.02, 100, 17}, {3, 299.97}});   // L1 C/A under an L5 master
    s.t1.push_back(g8);
    s.t1.push_back(sat(70 + 11, 11, {17, 800.0}, {{20, 800.01}, {21, 799.99}, {19, 800.0}}));
    s.t1.push_back(sat(140 + 19, 19, {28, -333.0}, {{14, -333.02}, {30, -332.98}, {13, -333.0}}));
    s.t1.push_back(sat(140 + 3, 3, {28, 10.0}, {{14, 10.0}}));                 // GEO: left out, bands too
    MeasT1 r02 = sat(40, 40, {8, 50.0}, {});                                   // GLONASS: not taken
    s.t1.push_back(r02);
    MeasT1 aux = sat(12, 12, {0, 70.0}, {{4, 70.0}});
    aux.sig(0, 1);                                                             // Aux1: not taken
    s.t1.push_back(aux);
    MeasT1 g20a = sat(20, 20, {0, -50.0}, {{4, -49.0}});
    g20a.codeMm(0);                                                            // no code: replaced by the next
    s.t1.push_back(g20a);
    s.t1.push_back(sat(20, 33, {0, -51.0}, {{4, -51.01}, {3, -50.99}}));
    MeasT1 g21 = sat(21, 21, {0, 75.0}, {{4, 75.0}});
    g21.lock = 65535;                                                          // no lock time
    s.t1.push_back(g21);
    MeasT1 g22 = sat(22, 22, {0, 0.0}, {{4, 0.0}});
    g22.doppler = -2147483647 - 1;                                             // no Doppler (its Type2s none)
    s.t1.push_back(g22);
    s.t1.push_back(sat(23, 23, {0, 99.0, 255}, {{4, 99.0}}));                  // C/N0 do-not-use
    return sbftest::measEpoch(s);
}

}  // namespace

// ── Off, and on with nothing to join: the image's cells ────────────────

TEST(MosaicMultiband, OffIsTheL1AdapterBitForBit)
{
    const std::vector<uint8_t> blk = everyKind(kTow0);
    MosaicJumpState js;
    MosaicEpochInfo info;
    RawEpoch ref;
    ASSERT_TRUE(rawEpochFromMeasEpoch(blk.data(), blk.size(), ref, js, info, 777));
    MosaicDopplerSet set_ref;
    MosaicJumpState js2;
    MosaicEpochInfo info2;
    ASSERT_TRUE(noteMeasEpochDopplers(blk.data(), blk.size(), js2, info2, set_ref, 777));

    for (int mode = 0; mode < 2; ++mode)
    {
        SCOPED_TRACE(mode == 0 ? "off" : "on, nothing in the join set");
        Mb m;
        m.cfg.on = mode == 1;
        if (mode == 1)
            for (int s = 0; s < kMbSigs; ++s)
                m.cfg.band[s].join = false;
        RawEpoch ep;
        ASSERT_TRUE(m.grid(blk, ep, 777));
        ASSERT_EQ(ep.n, ref.n);
        EXPECT_EQ(ep.tow_s, ref.tow_s);
        for (int i = 0; i < ep.n; ++i)
            expectSameCell(ep.obs[i], ref.obs[i]);
        expectSameInfo(m.info, info);
        expectSameJumps(m.js, js);

        Mb m2;
        m2.cfg = m.cfg;
        MosaicDopplerSet set;
        ASSERT_TRUE(m2.off(blk, set, 777));
        ASSERT_EQ(set.n, set_ref.n);
        EXPECT_EQ(set.rx_ms, set_ref.rx_ms);
        EXPECT_TRUE(set.valid);
        for (int i = 0; i < set.n; ++i)
        {
            EXPECT_EQ(set.c[i].sys, set_ref.c[i].sys);
            EXPECT_EQ(set.c[i].prn, set_ref.c[i].prn);
            EXPECT_EQ(set.c[i].has_pr, set_ref.c[i].has_pr);
            EXPECT_EQ(set.c[i].has_rate, set_ref.c[i].has_rate);
            EXPECT_EQ(set.c[i].cn0_dbhz, set_ref.c[i].cn0_dbhz);
            EXPECT_EQ(set.c[i].lock_ms, set_ref.c[i].lock_ms);
            EXPECT_EQ(set.c[i].rate_mps, set_ref.c[i].rate_mps);
        }
        expectSameJumps(m2.js, js2);
        if (mode == 1)
        {
            EXPECT_EQ(m.st.sats_combined, 0u);
            EXPECT_EQ(m.out.epoch, 1);   // no wind-up value at all: anchor-only
        }
    }
    // a block the reader refuses: false and nothing out, as the adapter
    std::vector<uint8_t> bad = blk;
    bad.resize(bad.size() - 8);
    Mb m;
    RawEpoch ep;
    EXPECT_FALSE(m.grid(bad, ep));
    EXPECT_EQ(ep.n, 0);
    EXPECT_EQ(m.st.epochs, 0u);
}

TEST(MosaicMultiband, OnEveryAnchorFieldButTheRateIsTheAdapters)
{
    const std::vector<uint8_t> blk = everyKind(kTow0);
    MosaicJumpState js;
    MosaicEpochInfo info;
    RawEpoch ref;
    ASSERT_TRUE(rawEpochFromMeasEpoch(blk.data(), blk.size(), ref, js, info));
    Mb m;
    RawEpoch ep;
    ASSERT_TRUE(m.grid(blk, ep));
    ASSERT_EQ(ep.n, ref.n);
    for (int i = 0; i < ep.n; ++i)
        expectSameCell(ep.obs[i], ref.obs[i], false);
    expectSameInfo(m.info, info);
    EXPECT_GT(m.st.sats_combined, 0u);
    // the satellite on two channels: the second copy's anchor and its own channel's bands (-51.01, -50.99)
    const int i20 = cellIndex(ep, GNSS_SYS_GPS, 20);
    ASSERT_GE(i20, 0);
    EXPECT_EQ(m.out.row[i20], MosaicMbRow::COMBINED);
    EXPECT_EQ(m.out.bands[i20], 2);
    EXPECT_NEAR(ep.obs[i20].rate_mps, -51.0, 0.02);   // not pulled toward the first channel's -49
    // GEO, GLONASS and the aux antenna's bands never captured; no lock time on the anchor is no gate on its row
    for (int i = 0; i < m.scr->n; ++i)
        EXPECT_FALSE(m.scr->sat[i].sys == GNSS_SYS_BDS && m.scr->sat[i].prn == 3);
    // the anchor without a Doppler: no row (its Type2s have none either), left as it came
    const int i22 = cellIndex(ep, GNSS_SYS_GPS, 22);
    ASSERT_GE(i22, 0);
    EXPECT_FALSE(ep.obs[i22].has_rate);
    EXPECT_EQ(m.out.row[i22], MosaicMbRow::NO_RATE);
}

// ── Scaling, wind-up and weights ───────────────────────────────────────

TEST(MosaicMultiband, AKnownRangeRateAndWindUpComeBackAtTheAnchorsWavelength)
{
    for (double W : {0.0, 0.99 * 1.0, -0.99 * 3.0})   // Hz: no roll, 1 rev/s, -3 rev/s
    {
        SCOPED_TRACE(testing::Message() << "W " << W);
        Mb m;
        RawEpoch ep;
        ASSERT_TRUE(m.grid(epoch(kTow0, sky(W)), ep));
        EXPECT_EQ(m.out.epoch, 0);
        EXPECT_NEAR(m.out.w_hat, W, 2e-3);
        const double R0 = -350.0;
        const struct { int sys, prn; double R; uint8_t asig; } want[] = {
            {GNSS_SYS_GPS, 5, R0, 0}, {GNSS_SYS_GAL, 11, R0 + 812.5, 17}, {GNSS_SYS_BDS, 20, R0 - 233.25, 28}};
        for (const auto& w : want)
        {
            const int i = cellIndex(ep, w.sys, w.prn);
            ASSERT_GE(i, 0);
            EXPECT_EQ(m.out.row[i], MosaicMbRow::COMBINED);
            // the row carries its anchor's wind-up exactly (REF): R + W lambda_a
            EXPECT_NEAR(ep.obs[i].rate_mps, w.R + W * lam(w.asig), 2e-4);
            EXPECT_DOUBLE_EQ(m.out.lam_wu[i], lam(w.asig));
        }
        EXPECT_EQ(m.out.bands[cellIndex(ep, GNSS_SYS_GPS, 5)], 2);   // L5 and L2C (L1 C/A, its anchor, a Type2)
        EXPECT_EQ(m.out.bands[cellIndex(ep, GNSS_SYS_GAL, 11)], 3);
        EXPECT_EQ(m.out.bands[cellIndex(ep, GNSS_SYS_BDS, 20)], 3);
    }
}

TEST(MosaicMultiband, WithoutReferralTheRowCarriesItsWeightedWavelength)
{
    // LAM: no referral; the row's wind-up at sum(w lambda) / sum(w), told in lam_wu
    const double W = 0.99 * 2.0;
    for (MosaicMbWindup mode : {MosaicMbWindup::LAM, MosaicMbWindup::NONE})
    {
        Mb m;
        m.cfg.windup = mode;
        RawEpoch ep;
        ASSERT_TRUE(m.grid(epoch(kTow0, sky(W)), ep));
        const int i = cellIndex(ep, GNSS_SYS_GAL, 11);
        ASSERT_GE(i, 0);
        // the weights by hand: E1 at 45, E5a 43, E5b 44, E6 41 dB-Hz
        const struct { uint8_t sig; double cn0; } b[] = {{17, 45.0}, {20, 43.0}, {21, 44.0}, {19, 41.0}};
        double ws = 0.0, wl = 0.0;
        for (const auto& x : b)
        {
            const double s = mbSigma(m.cfg.band[x.sig], x.cn0);
            ws += 1.0 / (s * s);
            wl += lam(x.sig) / (s * s);
        }
        EXPECT_NEAR(m.out.lam_wu[i], wl / ws, 1e-12);
        EXPECT_GT(m.out.lam_wu[i], lam(17) + 0.02);
        EXPECT_NEAR(ep.obs[i].rate_mps, -350.0 + 812.5 + W * wl / ws, 2e-4);
    }
}

TEST(MosaicMultiband, TheWeightsAreInverseVarianceFromEachBandsOwnCn0)
{
    // no wind-up, noise on each band (m/s): the combination is the inverse-variance mean, the anchor's included
    double n[kMbSigs] = {};
    n[17] = 0.030;
    n[20] = -0.050;
    n[21] = 0.020;
    n[19] = 0.045;
    n[0] = -0.01;
    n[4] = 0.02;
    n[3] = 0.0;
    n[28] = 0.015;
    n[14] = -0.02;
    n[30] = 0.01;
    n[13] = -0.005;
    Mb m;
    m.cfg.windup = MosaicMbWindup::NONE;   // the screen alone takes W_hat: the weights are seen bare
    RawEpoch ep;
    ASSERT_TRUE(m.grid(epoch(kTow0, sky(0.0, n)), ep));
    const int i = cellIndex(ep, GNSS_SYS_GAL, 11);
    ASSERT_GE(i, 0);
    ASSERT_EQ(m.out.row[i], MosaicMbRow::COMBINED);
    const struct { uint8_t sig; double cn0; } b[] = {{17, 45.0}, {20, 43.0}, {21, 44.0}, {19, 41.0}};
    double ws = 0.0, wn = 0.0;
    for (const auto& x : b)
    {
        const double s = mbSigma(m.cfg.band[x.sig], x.cn0);
        ws += 1.0 / (s * s);
        wn += n[x.sig] / (s * s);
    }
    EXPECT_NEAR(ep.obs[i].rate_mps, -350.0 + 812.5 + wn / ws, 2e-4);
    // its sigma: (sum w)^(-1/2), below the anchor's; the shares sum to 1 with the anchor's
    EXPECT_NEAR(m.out.sigma_c[i], 1.0 / std::sqrt(ws), 1e-6);
    EXPECT_LT(m.out.sigma_c[i], m.out.sigma_a[i]);
    double share = 0.0;
    for (int q = 0; q < m.out.ncand[i]; ++q)
        share += m.out.det[i][q].w;
    const double sa = mbSigma(m.cfg.band[17], 45.0);
    EXPECT_NEAR(share + 1.0 / (sa * sa) / ws, 1.0, 1e-5);
    // the measured model: a band's sigma in Hz the same, so in m/s with its wavelength
    EXPECT_NEAR(mbSigma(m.cfg.band[20], 35.0) / lam(20), mbSigma(m.cfg.band[17], 35.0) / lam(17), 0.02);
    EXPECT_NEAR(mbSigma(m.cfg.band[17], 45.0), 0.105 * std::pow(10.0, -0.5), 1e-6);
}

TEST(MosaicMultiband, FewerThanThreeWindUpValuesLeaveTheEpochAnchorOnly)
{
    // one GPS satellite with L5 and L2C: two values
    Mb m;
    RawEpoch ep;
    const double W = 1.5;
    ASSERT_TRUE(m.grid(epoch(kTow0, {sat(5, 5, {0, rrOf(-100.0, W, 0)},
                                         {{4, rrOf(-100.0, W, 4)}, {3, rrOf(-100.0, W, 3)}})}), ep));
    EXPECT_EQ(m.out.epoch, 1);
    EXPECT_EQ(m.out.w_n, 2);
    EXPECT_EQ(m.st.epochs_few_w, 1u);
    EXPECT_EQ(m.out.row[0], MosaicMbRow::EPOCH);
    EXPECT_NEAR(ep.obs[0].rate_mps, -100.0 + W * lam(0), 2e-5);   // its own, as the receiver gave it
    RawEpoch ref;
    MosaicJumpState js;
    MosaicEpochInfo info;
    const std::vector<uint8_t> blk = epoch(kTow0, {sat(5, 5, {0, rrOf(-100.0, W, 0)},
                                                      {{4, rrOf(-100.0, W, 4)}, {3, rrOf(-100.0, W, 3)}})});
    ASSERT_TRUE(rawEpochFromMeasEpoch(blk.data(), blk.size(), ref, js, info));
    EXPECT_EQ(ep.obs[0].rate_mps, ref.obs[0].rate_mps);
    // w_min_values 2 lets it through
    Mb m2;
    m2.cfg.w_min_values = 2;
    RawEpoch ep2;
    ASSERT_TRUE(m2.grid(blk, ep2));
    EXPECT_EQ(m2.out.row[0], MosaicMbRow::COMBINED);
}

// ── The screen ─────────────────────────────────────────────────────────

TEST(MosaicMultiband, ASlippedBandLeavesForTheEpoch)
{
    double n[kMbSigs] = {};
    n[20] = 0.8;   // Galileo E5a slipped 0.8 m/s
    Mb m;
    RawEpoch ep;
    ASSERT_TRUE(m.grid(epoch(kTow0, sky(0.5, n)), ep));
    const int i = cellIndex(ep, GNSS_SYS_GAL, 11);
    ASSERT_EQ(m.out.row[i], MosaicMbRow::COMBINED);
    EXPECT_EQ(m.out.bands[i], 2);
    EXPECT_EQ((m.out.mask[i] >> 20) & 1u, 0u);
    EXPECT_EQ((m.out.mask[i] >> 21) & 1u, 1u);
    EXPECT_EQ(m.st.screened, 1u);
    EXPECT_EQ(m.st.voted, 0u);
    EXPECT_NEAR(ep.obs[i].rate_mps, -350.0 + 812.5 + 0.5 * lam(17), 0.02);   // the slip kept out
}

TEST(MosaicMultiband, AnAnchorOffAgainstTwoAgreeingBandsIsVotedOut)
{
    double n[kMbSigs] = {};
    n[17] = 0.8;   // Galileo E1 off; E5a, E5b, E6 agree
    Mb m;
    RawEpoch ep;
    ASSERT_TRUE(m.grid(epoch(kTow0, sky(0.0, n)), ep));
    const int i = cellIndex(ep, GNSS_SYS_GAL, 11);
    EXPECT_EQ(m.out.row[i], MosaicMbRow::VOTED);
    EXPECT_FALSE(ep.obs[i].has_rate);
    EXPECT_TRUE(ep.obs[i].has_pr);   // its code kept
    EXPECT_EQ(m.st.voted, 1u);
    // the others combined as before
    EXPECT_EQ(m.out.row[cellIndex(ep, GNSS_SYS_GPS, 5)], MosaicMbRow::COMBINED);
    // without the vote: its flagged bands leave, the anchor alone
    Mb m2;
    m2.cfg.anchor_vote = false;
    RawEpoch ep2;
    ASSERT_TRUE(m2.grid(epoch(kTow0, sky(0.0, n)), ep2));
    EXPECT_EQ(m2.out.row[i], MosaicMbRow::ALONE);
    EXPECT_TRUE(ep2.obs[i].has_rate);
    EXPECT_EQ(m2.st.screened, 3u);
}

TEST(MosaicMultiband, AMajorityOffLeavesTheEpochAnchorOnly)
{
    // six bands, four slipped both ways (the median stays at 0): anchor-only, nothing voted
    Mb m;
    const std::vector<MeasT1> s = {
        sat(5, 5, {0, -100.0}, {{4, -100.0 + 1.0}, {3, -100.0 - 1.0}}),
        sat(6, 6, {0, 200.0}, {{4, 200.0 + 1.0}, {3, 200.0 - 1.0}}),
        sat(7, 7, {0, 300.0}, {{4, 300.0}, {3, 300.0}}),
    };
    RawEpoch ep;
    ASSERT_TRUE(m.grid(epoch(kTow0, s), ep));
    EXPECT_EQ(m.out.epoch, 2);
    EXPECT_EQ(m.st.epochs_screen, 1u);
    EXPECT_EQ(m.st.voted, 0u);
    for (int i = 0; i < ep.n; ++i)
        EXPECT_EQ(m.out.row[i], MosaicMbRow::EPOCH);
    EXPECT_NEAR(ep.obs[0].rate_mps, -100.0, 2e-5);
}

// ── The join rule ──────────────────────────────────────────────────────

TEST(MosaicMultiband, ABandJoinsOnlyWithASecondOfLock)
{
    for (uint8_t lock : {static_cast<uint8_t>(0), static_cast<uint8_t>(1), static_cast<uint8_t>(255)})
    {
        SCOPED_TRACE(testing::Message() << "E5a lock " << int(lock));
        std::vector<MeasT1> s = sky(0.3);
        s[1].t2[0].lock = lock;   // Galileo E5a
        Mb m;
        RawEpoch ep;
        ASSERT_TRUE(m.grid(epoch(kTow0, s), ep));
        const int i = cellIndex(ep, GNSS_SYS_GAL, 11);
        EXPECT_EQ((m.out.mask[i] >> 20) & 1u, lock == 1 ? 1u : 0u);
        EXPECT_EQ(m.out.bands[i], lock == 1 ? 3 : 2);
    }
    // the anchor's own lock (a lockCapMs or a relock) is no gate on its row
    std::vector<MeasT1> s = sky(0.3);
    s[1].lock = 0;
    Mb m;
    RawEpoch ep;
    ASSERT_TRUE(m.grid(epoch(kTow0, s), ep));
    EXPECT_EQ(m.out.row[cellIndex(ep, GNSS_SYS_GAL, 11)], MosaicMbRow::COMBINED);
}

TEST(MosaicMultiband, TheFloorHasHysteresisAndTheHalfCycleFlagGatesNothing)
{
    // Galileo E6 at 26 dB-Hz: under 25 + 2, so out; at 28, in; back at 26, stays; at 24, out; at 26, still out
    Mb m;
    const uint8_t cn0s[] = {64, 72, 64, 56, 64};   // raw: 26, 28, 26, 24, 26 dB-Hz
    const uint32_t joined[] = {0, 1, 1, 0, 0};
    for (int k = 0; k < 5; ++k)
    {
        SCOPED_TRACE(testing::Message() << "epoch " << k);
        std::vector<MeasT1> s = sky(0.1);
        s[1].t2[2].cn0 = cn0s[k];   // E6
        s[1].t2[1].obs = 0x04;      // E5b with a half-cycle ambiguity: joins all the same
        RawEpoch ep;
        ASSERT_TRUE(m.grid(epoch(kTow0 + 100 * k, s), ep));
        const int i = cellIndex(ep, GNSS_SYS_GAL, 11);
        EXPECT_EQ((m.out.mask[i] >> 19) & 1u, joined[k]);
        EXPECT_EQ((m.out.mask[i] >> 21) & 1u, 1u);
    }
    // composition changes: E6 in at 1, out at 3
    EXPECT_EQ(m.st.comp_changes, 2u);
    // the epoch before more than kMbPrevMaxMs back: the join threshold again
    std::vector<MeasT1> s = sky(0.1);
    s[1].t2[2].cn0 = 72;
    RawEpoch ep;
    ASSERT_TRUE(m.grid(epoch(kTow0 + 500, s), ep));
    s[1].t2[2].cn0 = 64;
    ASSERT_TRUE(m.grid(epoch(kTow0 + 1000, s), ep));
    EXPECT_EQ((m.out.mask[cellIndex(ep, GNSS_SYS_GAL, 11)] >> 19) & 1u, 0u);
}

TEST(MosaicMultiband, AtMostMaxBandsTheStrongestFirst)
{
    Mb m;
    m.cfg.max_bands = 1;
    m.cfg.w_min_values = 2;   // one band a satellite: GPS's L5 and Galileo's E5b give the values (B1C's is none)
    RawEpoch ep;
    ASSERT_TRUE(m.grid(epoch(kTow0, sky(0.2)), ep));
    const int i = cellIndex(ep, GNSS_SYS_GAL, 11);
    EXPECT_EQ(m.out.bands[i], 1);
    EXPECT_EQ(m.out.mask[i], uint64_t{1} << 21);   // E5b, at 44 dB-Hz the strongest
}

TEST(MosaicMultiband, ABandsLagFarFromItsAnchorsKeepsItOut)
{
    Mb m;
    m.cfg.band[20].lag_ms = 2.0f;   // E5a 2 ms late
    RawEpoch ep;
    ASSERT_TRUE(m.grid(epoch(kTow0, sky(0.2)), ep));
    EXPECT_EQ((m.out.mask[cellIndex(ep, GNSS_SYS_GAL, 11)] >> 20) & 1u, 0u);
    m.cfg.band[17].lag_ms = 1.5f;   // and its anchor 1.5: within 1 ms
    ASSERT_TRUE(m.grid(epoch(kTow0 + 100, sky(0.2)), ep));
    EXPECT_EQ((m.out.mask[cellIndex(ep, GNSS_SYS_GAL, 11)] >> 20) & 1u, 1u);
}

// ── The pairs ──────────────────────────────────────────────────────────

TEST(MosaicMultiband, BothEndsCombinedThenAveragedAsThePairsAre)
{
    const double W = 0.99 * 0.5;
    rr::MosaicFeed::Config c;
    c.pair = true;
    for (int k = 0; k < GNSS_SYS_COUNT; ++k)
        c.pair_cfg.cn0_min[k] = 25.0f;
    c.mb = measured();
    rr::MosaicFeed f(c);
    auto arrival = [](uint32_t tow) { return 1000000 + (static_cast<int64_t>(tow) - kTow0) * 1000 + 13000; };
    gnss_raw::TimedRawEpoch te;
    const std::vector<uint8_t> b0 = epoch(kTow0 - 50, sky(W, nullptr, -351.0));
    const std::vector<uint8_t> b1 = epoch(kTow0, sky(W, nullptr, -350.0));
    ASSERT_EQ(f.onBlock(b0.data(), b0.size(), arrival(kTow0 - 50), te), rr::MosaicFeed::Kind::EPOCH_SKIPPED);
    ASSERT_TRUE(f.mbTake());
    EXPECT_FALSE(f.mbLastGrid());
    const MosaicMbOut o0 = f.mbLast();
    ASSERT_EQ(f.onBlock(b1.data(), b1.size(), arrival(kTow0), te), rr::MosaicFeed::Kind::EPOCH);
    ASSERT_TRUE(f.mbTake());
    EXPECT_TRUE(f.mbLastGrid());
    EXPECT_EQ(te.rr_paired, 1);
    EXPECT_EQ(f.lastPaired(), 7u);
    const int i = cellIndex(te.epoch, GNSS_SYS_GAL, 11);
    ASSERT_GE(i, 0);
    // the mean of the two combined ends: (R0 + R1) / 2 + W lambda_E1
    EXPECT_NEAR(te.epoch.obs[i].rate_mps, -350.5 + 812.5 + W * lam(17), 2e-4);
    // the paired row's sigma ratio from both ends
    const MosaicMbOut& g = f.mbGrid();
    int j = -1;
    for (int k = 0; k < o0.n; ++k)
        if (o0.sys[k] == GNSS_SYS_GAL && o0.prn[k] == 11) j = k;
    ASSERT_GE(j, 0);
    const double sci = g.sigma_c[i], scj = o0.sigma_c[j], sai = g.sigma_a[i], saj = o0.sigma_a[j];
    const double want = std::sqrt((sci * sci + scj * scj) / (sai * sai + saj * saj));
    EXPECT_NEAR(f.mbRowRatio(i), want, 1e-9);
    EXPECT_LT(f.mbRowRatio(i), 0.8);
    EXPECT_DOUBLE_EQ(f.mbRowLam(i), lam(17));
    EXPECT_EQ(f.mbStats().epochs, 2u);

    // the same pair with the combination off: the L1 pair alone, the same as the image
    rr::MosaicFeed::Config c0 = c;
    c0.mb.on = false;
    rr::MosaicFeed f0(c0);
    gnss_raw::TimedRawEpoch te0;
    f0.onBlock(b0.data(), b0.size(), arrival(kTow0 - 50), te0);
    ASSERT_EQ(f0.onBlock(b1.data(), b1.size(), arrival(kTow0), te0), rr::MosaicFeed::Kind::EPOCH);
    EXPECT_FALSE(f0.mbTake());
    EXPECT_NEAR(te0.epoch.obs[i].rate_mps, -350.5 + 812.5 + W * lam(17), 2e-4);   // no noise: the same value
    EXPECT_EQ(f0.mbStats().epochs, 0u);
}

TEST(MosaicMultiband, TheFeedWithNothingToJoinHandsOverTheImagesEpochs)
{
    // a run of 20 Hz epochs, pairs on: the feed with the combination on and an empty join set against it off
    rr::MosaicFeed::Config c;
    c.pair = true;
    for (int k = 0; k < GNSS_SYS_COUNT; ++k)
        c.pair_cfg.cn0_min[k] = 25.0f;
    rr::MosaicFeed off(c);
    c.mb = measured();
    for (int s = 0; s < kMbSigs; ++s)
        c.mb.band[s].join = false;
    rr::MosaicFeed on(c);
    double n[kMbSigs] = {};
    for (int k = 0; k < 12; ++k)
    {
        SCOPED_TRACE(testing::Message() << "epoch " << k);
        n[0] = 0.01 * ((k * 7) % 5 - 2);
        n[17] = -0.02 * ((k * 3) % 4 - 1.5);
        const uint32_t tow = kTow0 - 50 + 50 * k;
        const std::vector<uint8_t> b = epoch(tow, sky(0.3 * k, n, -350.0 + 0.4 * k));
        gnss_raw::TimedRawEpoch ta, tb;
        const int64_t arr = 1000000 + 50000 * k + 13000;
        const rr::MosaicFeed::Kind ka = off.onBlock(b.data(), b.size(), arr, ta);
        const rr::MosaicFeed::Kind kb = on.onBlock(b.data(), b.size(), arr, tb);
        ASSERT_EQ(ka, kb);
        if (ka != rr::MosaicFeed::Kind::EPOCH)
            continue;
        EXPECT_EQ(ta.rr_paired, tb.rr_paired);
        ASSERT_EQ(ta.epoch.n, tb.epoch.n);
        for (int i = 0; i < ta.epoch.n; ++i)
            expectSameCell(ta.epoch.obs[i], tb.epoch.obs[i]);
        EXPECT_EQ(off.lastPaired(), on.lastPaired());
    }
    EXPECT_GT(on.pairStats().pairs, 0u);
    EXPECT_EQ(on.pairStats().pairs, off.pairStats().pairs);
    EXPECT_EQ(on.mbStats().sats_combined, 0u);
}

TEST(MosaicMultiband, AnEndWhoseAnchorIsVotedOutDropsItsPair)
{
    rr::MosaicFeed::Config c;
    c.pair = true;
    for (int k = 0; k < GNSS_SYS_COUNT; ++k)
        c.pair_cfg.cn0_min[k] = 25.0f;
    c.mb = measured();
    rr::MosaicFeed f(c);
    gnss_raw::TimedRawEpoch te;
    double n[kMbSigs] = {};
    n[17] = 1.0;   // E1 off at the epoch between alone
    const std::vector<uint8_t> b0 = epoch(kTow0 - 50, sky(0.0, n));
    const std::vector<uint8_t> b1 = epoch(kTow0, sky(0.0));
    f.onBlock(b0.data(), b0.size(), 1000000, te);
    ASSERT_EQ(f.onBlock(b1.data(), b1.size(), 1050000, te), rr::MosaicFeed::Kind::EPOCH);
    EXPECT_EQ(f.mbStats().voted, 1u);
    EXPECT_EQ(te.rr_paired, 1);   // two of three in pairs
    const RawObs* e1 = cell(te.epoch, GNSS_SYS_GAL, 11);
    ASSERT_NE(e1, nullptr);
    EXPECT_FALSE(e1->has_rate);   // its instant is not the pairs': dropped (the pairs' rule, unchanged)
    EXPECT_EQ(f.pairStats().alone_mate, 1u);
}

// ── The effective C/N0 ─────────────────────────────────────────────────

TEST(MosaicMultiband, TheEffectiveCn0RoundTrips)
{
    const double a = 0.01, b = 0.075;
    auto sig = [&](double cn0) { return std::sqrt(a * a + b * b * std::pow(10.0, -(cn0 - 35.0) / 10.0)); };
    for (double cn0 : {28.0, 35.0, 41.25, 46.0})
        for (double ratio : {0.5, 0.7, 0.85})
        {
            SCOPED_TRACE(testing::Message() << cn0 << " dB-Hz, ratio " << ratio);
            const uint8_t q = mbEffCn0Q(cn0, ratio, a, b);
            if (q == 254)
                continue;   // under the floor a
            ASSERT_GT(q, 0);
            const double got = sig(mbCn0OfQ(q)) / sig(cn0);
            EXPECT_NEAR(got, ratio, 0.025);   // a quarter dB-Hz's step
            EXPECT_GT(mbCn0OfQ(q), cn0);
        }
    EXPECT_EQ(mbEffCn0Q(50.0, 0.2, a, b), 254);   // at or under a: the top code
    EXPECT_EQ(mbEffCn0Q(40.0, 1.0, a, b), 0);     // no gain: its own
    EXPECT_EQ(mbEffCn0Q(40.0, 0.999, a, b), 0);   //   nor under a quarter dB-Hz
    EXPECT_EQ(mbEffCn0Q(40.0, 0.7, a, 0.0), 0);
    // express_sigma fills it on a combined row
    Mb m;
    m.cfg.express_sigma = true;
    RawEpoch ep;
    ASSERT_TRUE(m.grid(epoch(kTow0, sky(0.0)), ep));
    const int i = cellIndex(ep, GNSS_SYS_GAL, 11);
    EXPECT_GT(m.out.rr_cn0_q[i], 0);
    EXPECT_GT(mbCn0OfQ(m.out.rr_cn0_q[i]), 45.0);
}

TEST(MosaicMultiband, TheWavelengthTableIsTheAdaptersDouble)
{
    for (uint8_t s : {0, 3, 4, 13, 14, 17, 19, 20, 21, 28, 30})
        EXPECT_EQ(kMbLambda.v[s], kMosaicC / sbfCarrierHz(s));
    EXPECT_EQ(kMbLambda.v[8], 0.0);   // GLONASS FDMA: never joins
    EXPECT_EQ(mbSigSys(4), GNSS_SYS_GPS);
    EXPECT_EQ(mbSigSys(20), GNSS_SYS_GAL);
    EXPECT_EQ(mbSigSys(13), GNSS_SYS_BDS);
    EXPECT_EQ(mbSigSys(8), 0xFF);
    float v[] = {3.0f, -1.0f, 2.0f, 10.0f};
    EXPECT_FLOAT_EQ(mbMedian(v, 4), 2.5f);
    float u[] = {3.0f, -1.0f, 2.0f};
    EXPECT_FLOAT_EQ(mbMedian(u, 3), 2.0f);
}

// ── Real captures (private; TR_SBF_PRIVATE_CAPTURE) ─────────────────────

TEST(MosaicMultibandPrivateCapture, AnchorsAreTheAdaptersAndTheBandsCombine)
{
    const auto paths = sbftest::privateCapturePaths();
    if (paths.empty()) GTEST_SKIP() << "TR_SBF_PRIVATE_CAPTURE not set";
    for (const std::string& path : paths)
    {
        SCOPED_TRACE(sbftest::captureName(path));
        Mb on, empty;
        for (int s = 0; s < kMbSigs; ++s)
            empty.cfg.band[s].join = false;
        MosaicJumpState js;
        MosaicEpochInfo info;
        auto ref = std::make_unique<RawEpoch>();
        auto a = std::make_unique<RawEpoch>();
        auto b = std::make_unique<RawEpoch>();
        uint32_t epochs = 0, diff_empty = 0, diff_anchor = 0, rows = 0, combined = 0;
        double sum_ratio = 0.0;
        sbftest::CaptureScan scan;
        const bool read = sbftest::scanCapture(path, [&](const uint8_t* blk, size_t len) {
            SbfHeader h;
            if (!sbfHeader(blk, len, h) || h.number != SBF_MEAS_EPOCH)
                return;
            const bool r0 = rawEpochFromMeasEpoch(blk, len, *ref, js, info);
            const bool r1 = rawEpochFromMeasEpochMb(blk, len, *a, empty.js, empty.info, empty.cfg, *empty.scr,
                                                    empty.st, empty.out);
            const bool r2 = rawEpochFromMeasEpochMb(blk, len, *b, on.js, on.info, on.cfg, *on.scr, on.st, on.out);
            if (r0 != r1 || r0 != r2 || ref->n != a->n || ref->n != b->n)
            {
                diff_empty++;
                return;
            }
            epochs++;
            for (int i = 0; i < ref->n; ++i)
            {
                const RawObs& o = ref->obs[i];
                const RawObs& x = a->obs[i];
                const RawObs& y = b->obs[i];
                if (o.has_rate != x.has_rate || o.rate_mps != x.rate_mps || o.pr_m != x.pr_m || o.lock_ms != x.lock_ms ||
                    o.cn0_dbhz != x.cn0_dbhz || o.phase_m != x.phase_m || o.has_phase != x.has_phase ||
                    o.halfcyc != x.halfcyc || o.has_pr != x.has_pr)
                    diff_empty++;
                if (o.sys != y.sys || o.prn != y.prn || o.pr_m != y.pr_m || o.lock_ms != y.lock_ms ||
                    o.cn0_dbhz != y.cn0_dbhz || o.phase_m != y.phase_m || o.has_phase != y.has_phase ||
                    o.halfcyc != y.halfcyc || o.has_pr != y.has_pr)
                    diff_anchor++;
                if (on.out.row[i] != MosaicMbRow::NO_RATE)
                    rows++;
                if (on.out.row[i] == MosaicMbRow::COMBINED)
                {
                    combined++;
                    sum_ratio += on.out.sigma_c[i] / on.out.sigma_a[i];
                }
            }
        }, scan);
        ASSERT_TRUE(read);
        EXPECT_EQ(diff_empty, 0u);
        EXPECT_EQ(diff_anchor, 0u);
        EXPECT_GT(epochs, 0u);
        const MosaicMultibandStats& st = on.st;
        std::printf("  %s: %u MeasEpochs; usable rows %u, combined %u (%.1f %%), mean sigma ratio %.3f; epochs "
                    "anchor-only %u (few W) %u (screen); W_hat rms %.3f Hz, max %.2f; screened %u, voted %u, "
                    "composition changes %u\n",
                    sbftest::captureName(path).c_str(), epochs, rows, combined, rows ? 100.0 * combined / rows : 0.0,
                    combined ? sum_ratio / combined : 0.0, st.epochs_few_w, st.epochs_screen,
                    st.w_epochs ? std::sqrt(st.w_sum2 / st.w_epochs) : 0.0, static_cast<double>(st.w_max_abs),
                    st.screened, st.voted, st.comp_changes);
    }
}
