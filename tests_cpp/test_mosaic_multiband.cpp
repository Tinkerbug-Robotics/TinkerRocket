// Host tests for the multi-band combiner (TR_GNSSReceiverMosaic_Serial/
// MosaicMultiband.h: the mosaic image's driver, config.h TR_RAW_MULTIBAND, and the harness's --rr-multiband):
//   - off, the cells and the Doppler sets are rawEpochFromMeasEpoch's and
//     noteMeasEpochDopplers' to the bit; on with nothing to join, too; on
//     with bands, every anchor field but the range rate is theirs;
//   - the scaling: each band's range rate from its own wavelength, wherever
//     the anchor sits (a Type1, or a Type2 under an L5 master), so a known
//     range rate and wind-up come back as the anchor's;
//   - the weights: inverse variance from each band's own C/N0, with its own
//     signal's offset (GPS L1P and L2P carry no +10);
//   - the wind-up: referred to the anchor's wavelength (the row carries its
//     anchor's), or left at the weighted wavelength the side outputs give;
//     too few values and the epoch goes anchor-only; the estimate one value
//     a satellite (its bands regressed on their wavelengths), the weights
//     capped, a capped median then a clipped capped mean, smoothed in time
//     with each satellite gated (one bad epoch or one satellite held out, a
//     step taken only on the satellites' or the gyro's agreement), the
//     defaults on a sky of eight satellites, and the gyro's standing in
//     where asked or where the estimate is short;
//   - composition changes: a band joining or leaving steps the row by its
//     weight times its bias against the others, by noise alone without one;
//   - faults on the anchor alone: a step over the screen voted out (or the
//     row left its anchor's whole), one under it reaching the row at the
//     anchor's share until the drift guard drops it, a ramp dropped by the
//     guard before the screen would vote it out, the guard letting go and
//     starting over, and neither it nor the new rules tripping on noise;
//   - the review's three defects of a08bdcde, each a test that failed there:
//     one strong satellite's anchor fault capturing the estimate, the
//     estimate checked against the gyro's, the guard's hold through a
//     screened-out last band, through anchor-only epochs and under
//     acceleration, a fault common to every anchor going anchor-only, and
//     one common to a system's anchors never thinned to its single-band
//     satellites;
//   - the screen: a slipped band out, an anchor voted out by two agreeing
//     bands, a majority off and the epoch anchor-only;
//   - the join rule: the lock (none, 0 s, 1 s), the floor's hysteresis, the
//     half-cycle flag gating nothing, the anchor's channel only (a satellite on
//     two channels), at most max_bands, and composition changes counted;
//   - the pairs: both ends combined, the pair averaged on the combined rates
//     and a voted-out end dropping its pair; the paired row's sigma ratio;
//   - the effective C/N0 round trip and its clamps;
//   - what the image flies (mosaicMbFlight) and the gyro's wind-up as the driver reads it (mbWindupFromGyro).
// Synthetic blocks (sbf_meas_build.h); behind TR_SBF_PRIVATE_CAPTURE, real
// captures for the identities (statistics only, never a position).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <tuple>
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
    // the three-satellite sky below gives seven wind-up values from three satellites with a 1-sigma about 0.4-0.7
    // Hz, and two once the screen's tests put one off: under the defaults' floors (six values, three satellites,
    // 0.75 Hz), which the tests of the estimate itself take as they are
    c.w_min_values = 3;
    c.w_min_sats = 2;
    c.w_max_sigma_hz = 10.0f;
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
    MosaicMbWindupIn wext;   // the gyro's wind-up, as the driver would give it (w_src GYRO, AUTO)

    bool grid(const std::vector<uint8_t>& b, RawEpoch& ep, int64_t arr = kMosaicNoArrival)
    {
        return rawEpochFromMeasEpochMb(b.data(), b.size(), ep, js, info, cfg, *scr, st, out, arr, wext);
    }
    bool off(const std::vector<uint8_t>& b, MosaicDopplerSet& set, int64_t arr = kMosaicNoArrival)
    {
        return noteMeasEpochDopplersMb(b.data(), b.size(), js, info, set, cfg, *scr, st, out, arr, wext);
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
    // w_min_values 2 and w_min_sats 1 let it through
    Mb m2;
    m2.cfg.w_min_values = 2;
    m2.cfg.w_min_sats = 1;
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
    // six bands, four slipped both ways: anchor-only, nothing voted. The estimate's clip would leave the two
    // unslipped values alone (too few): the gyro's 0 refers them, so the screen sees all six
    Mb m;
    m.cfg.w_src = MosaicMbWSrc::GYRO;
    m.wext = {true, 0.0f};
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

// ── Each band's own C/N0 offset ────────────────────────────────────────

TEST(MosaicMultiband, AnExtraBandsCn0IsItsOwnSignalsGpsL1pAndL2pWithoutTheTen)
{
    // GPS L1 C/A with L2 P(Y) (signal 2) and L1P (signal 1) beside it, both let join through the configuration (a
    // model file could): their raw C/N0 is 0.25 dB-Hz steps with no +10 (SbfMeasSignal::cn0Dbhz), unlike L2C's
    const double R = -120.0, W = 0.3;
    for (uint8_t raw : {static_cast<uint8_t>(104), static_cast<uint8_t>(120)})
    {
        SCOPED_TRACE(testing::Message() << "L1P and L2P raw C/N0 " << int(raw));
        const double want = raw * 0.25;   // 26 or 30 dB-Hz: not 36 or 40
        Mb k;
        k.cfg.band[1] = MosaicMbBand{true, 0.0f, 0.104f, 1.0f, 25.0f, 0.0f};
        k.cfg.band[2] = MosaicMbBand{true, 0.0f, 0.129f, 1.0f, 25.0f, 0.0f};
        k.cfg.w_min_values = 1;
        k.cfg.w_min_sats = 1;
        const MosaicMultibandConfig& mc = k.cfg;
        RawEpoch ep;
        ASSERT_TRUE(k.grid(epoch(kTow0, {sat(5, 5, {0, rrOf(R, W, 0)},
                                             {{1, rrOf(R, W, 1), raw}, {2, rrOf(R, W, 2), raw}, {3, rrOf(R, W, 3), raw}})}),
                           ep));
        ASSERT_EQ(ep.n, 1);
        // L2C (signal 3) at the same raw value reads 10 dB-Hz higher, and joins
        bool l2c = false;
        for (int q = 0; q < k.out.ncand[0]; ++q)
        {
            const MosaicMbBandOut& d = k.out.det[0][q];
            if (d.sig == 3)
            {
                l2c = true;
                EXPECT_FLOAT_EQ(d.cn0, static_cast<float>(want + 10.0));
            }
            if (d.sig == 1 || d.sig == 2)
                EXPECT_FLOAT_EQ(d.cn0, static_cast<float>(want));
        }
        EXPECT_TRUE(l2c);
        // at 26 dB-Hz both are under 25 + 2 and stay out; at 30 both join, each weighted from its own 30 dB-Hz
        int pn = 0;
        for (int q = 0; q < k.out.ncand[0]; ++q)
            pn += k.out.det[0][q].sig == 1 || k.out.det[0][q].sig == 2;
        EXPECT_EQ(pn, raw == 104 ? 0 : 2);
        if (raw == 120)
        {
            ASSERT_EQ(k.out.row[0], MosaicMbRow::COMBINED);
            double ws = 1.0 / std::pow(mbSigma(mc.band[0], 45.0), 2);
            for (const auto& x : {std::pair<uint8_t, double>{1, want}, {2, want}, {3, want + 10.0}})
                ws += 1.0 / std::pow(mbSigma(mc.band[x.first], x.second), 2);
            for (int q = 0; q < k.out.ncand[0]; ++q)
            {
                const MosaicMbBandOut& d = k.out.det[0][q];
                const double cn0 = d.sig == 3 ? want + 10.0 : want;
                EXPECT_NEAR(d.w, 1.0 / std::pow(mbSigma(mc.band[d.sig], cn0), 2) / ws, 1e-6);
            }
        }
    }
}

// ── The wind-up estimate ───────────────────────────────────────────────

TEST(MosaicMultiband, TheWindUpEstimateIsOneValueASatelliteCappedThenMedianAndClip)
{
    MosaicMbWork wk;
    {
        // capped weights: none more than the share of their sum (water-filling); too few for the share: all equal
        const float I[] = {10.0f, 1.0f, 1.0f, 1.0f};
        float u[4] = {};
        mbCapWeights(I, u, 4, 0.35f, wk.idx);
        const float tau = 0.35f * 3.0f / 0.65f;
        EXPECT_NEAR(u[0], tau, 1e-5);
        EXPECT_FLOAT_EQ(u[1], 1.0f);
        EXPECT_NEAR(u[0] / (u[0] + 3.0f), 0.35f, 1e-5);
        const float J[] = {9.0f, 8.0f, 1.0f, 1.0f, 1.0f, 1.0f};   // two capped
        float v[6] = {};
        mbCapWeights(J, v, 6, 0.35f, wk.idx);
        const float tau2 = 0.35f * 4.0f / (1.0f - 0.7f);
        EXPECT_NEAR(v[0], tau2, 1e-4);
        EXPECT_NEAR(v[1], tau2, 1e-4);
        EXPECT_FLOAT_EQ(v[2], 1.0f);
        const float K[] = {5.0f, 1.0f};
        float w[2] = {};
        mbCapWeights(K, w, 2, 0.35f, wk.idx);
        EXPECT_FLOAT_EQ(w[0], w[1]);
        const float L[] = {1.0f, 1.1f, 1.05f};   // none over the share: their own
        float x[3] = {};
        mbCapWeights(L, x, 3, 0.35f, wk.idx);
        EXPECT_FLOAT_EQ(x[1], 1.1f);
    }
    float hz = 0.0f, sig = 0.0f;
    int ks = 0, kv = 0;
    auto sat_ = [](float v, float s, int nval = 2) {
        MosaicMbWSat x;
        x.v = v;
        x.sig = s;
        x.w = 1.0f / (s * s);
        x.nval = static_cast<uint8_t>(nval);
        return x;
    };
    {
        // one strong satellite 5 Hz off (0.96 of the weight uncapped): capped to 0.35 it is not the median, and the
        // clip leaves it out; uncapped it was the median and the estimate
        MosaicMbWSat s[] = {sat_(5.0f, 0.1f), sat_(0.1f, 1.0f), sat_(-0.1f, 1.0f), sat_(0.0f, 1.0f), sat_(0.05f, 1.0f)};
        ASSERT_TRUE(mbWindupEstimate(s, 5, 3, 6, 100.0f, 3.0f, 0.35f, wk, hz, sig, ks, kv));
        EXPECT_EQ(ks, 4);
        EXPECT_EQ(kv, 8);
        EXPECT_NEAR(hz, 0.0125f, 1e-6);
        EXPECT_NEAR(sig, 0.5f, 1e-6);   // sqrt(sum u^2 sigma^2) / sum u, equal weights
        MosaicMbWSat t[] = {sat_(5.0f, 0.1f), sat_(0.1f, 1.0f), sat_(-0.1f, 1.0f), sat_(0.0f, 1.0f), sat_(0.05f, 1.0f)};
        ASSERT_FALSE(mbWindupEstimate(t, 5, 3, 6, 100.0f, 3.0f, 1.0f, wk, hz, sig, ks, kv));   // no cap
        EXPECT_EQ(ks, 1);
        EXPECT_FLOAT_EQ(hz, 5.0f);
    }
    {
        // the floors: satellites, values and the 1-sigma (hz, sig set either way); a capped weight's honest sigma
        MosaicMbWSat s[] = {sat_(1.0f, 1.0f, 1), sat_(1.2f, 1.0f, 1), sat_(0.8f, 1.0f, 1)};
        EXPECT_FALSE(mbWindupEstimate(s, 3, 4, 1, 100.0f, 3.0f, 0.35f, wk, hz, sig, ks, kv));
        EXPECT_EQ(ks, 3);
        EXPECT_NEAR(hz, 1.0f, 1e-6);
        EXPECT_FALSE(mbWindupEstimate(s, 3, 3, 4, 100.0f, 3.0f, 0.35f, wk, hz, sig, ks, kv));   // 3 values
        EXPECT_TRUE(mbWindupEstimate(s, 3, 3, 3, 0.6f, 3.0f, 0.35f, wk, hz, sig, ks, kv));
        EXPECT_NEAR(sig, 1.0f / std::sqrt(3.0f), 1e-6);
        EXPECT_FALSE(mbWindupEstimate(s, 3, 3, 3, 0.5f, 3.0f, 0.35f, wk, hz, sig, ks, kv));
        EXPECT_FALSE(mbWindupEstimate(s, 0, 0, 0, 100.0f, 3.0f, 0.35f, wk, hz, sig, ks, kv));
        EXPECT_EQ(ks, 0);
        MosaicMbWSat t[] = {sat_(1.0f, 0.1f), sat_(1.0f, 1.0f), sat_(1.0f, 1.0f), sat_(1.0f, 1.0f)};
        ASSERT_TRUE(mbWindupEstimate(t, 4, 3, 1, 100.0f, 3.0f, 0.35f, wk, hz, sig, ks, kv));
        const float tau = 0.35f * 3.0f / 0.65f;   // the strong one capped: its sigma counts at its capped weight
        EXPECT_NEAR(sig, std::sqrt(tau * tau * 0.01f + 3.0f) / (tau + 3.0f), 1e-6);
    }
    {
        // one value a satellite: its anchor's and bands' range rates regressed on their wavelengths, the anchor's
        // error counted once. Galileo 11 alone with E5a, E5b, E6, its anchor off F: the slope by hand (normal
        // equations of r = R + W lambda), and with one band: (r_b - r_a) / (lambda_b - lambda_a)
        Mb m;
        mosaicMbMeasured(m.cfg);
        m.cfg.on = true;
        m.cfg.w_min_sats = 1;
        m.cfg.w_min_values = 1;
        m.cfg.w_max_sigma_hz = 100.0f;
        const double W = 0.7, F = 0.05, R = 300.0;
        RawEpoch ep;
        ASSERT_TRUE(m.grid(epoch(kTow0, {sat(70 + 11, 11, {17, rrOf(R, W, 17) + F},
                                             {{20, rrOf(R, W, 20), 132}, {21, rrOf(R, W, 21), 136},
                                              {19, rrOf(R, W, 19), 124}})}),
                           ep));
        ASSERT_EQ(m.scr->nws, 1);
        double S0 = 0, Sx = 0, Sy = 0, Sxx = 0, Sxy = 0;
        for (const auto& b : {std::tuple<uint8_t, double, double>{17, 45.0, F}, {20, 43.0, 0.0}, {21, 44.0, 0.0},
                              {19, 41.0, 0.0}})
        {
            const double w = 1.0 / std::pow(mbSigma(m.cfg.band[std::get<0>(b)], std::get<1>(b)), 2);
            const double x = lam(std::get<0>(b)), y = rrOf(R, W, std::get<0>(b)) + std::get<2>(b);
            S0 += w;
            Sx += w * x;
            Sy += w * y;
            Sxx += w * x * x;
            Sxy += w * x * y;
        }
        const double I = Sxx - Sx * Sx / S0;
        EXPECT_NEAR(m.scr->wsat[0].v, (Sxy - Sx * Sy / S0) / I, 2e-3);
        EXPECT_NEAR(m.scr->wsat[0].sig, 1.0 / std::sqrt(I), 1e-4);
        EXPECT_EQ(m.scr->wsat[0].nval, 3);
        Mb o;
        o.cfg = m.cfg;
        ASSERT_TRUE(o.grid(epoch(kTow0, {sat(5, 5, {0, rrOf(R, W, 0) + F}, {{4, rrOf(R, W, 4), 132}})}), ep));
        ASSERT_EQ(o.scr->nws, 1);
        const double dl = lam(4) - lam(0);
        EXPECT_NEAR(o.scr->wsat[0].v, (-F) / dl + W, 2e-3);
        EXPECT_NEAR(o.scr->wsat[0].sig,
                    std::sqrt(std::pow(mbSigma(o.cfg.band[0], 45.0), 2) + std::pow(mbSigma(o.cfg.band[4], 43.0), 2)) /
                        std::fabs(dl),
                    1e-4);
    }
}

namespace {

// A sky of eight satellites: GPS L1 C/A with L2C and L5, Galileo E1 with E5a, E5b and E6, BeiDou B1I with B2a, B3I
// and B1C (each anchor the Type1 master at 45 dB-Hz, its bands Type2s at 43), range rates R_s; err(s, sig) is band
// sig's error on satellite s (m/s), cn0(s, sig) a raw C/N0 in place of the default.
struct SatSpec
{
    uint8_t svid, ch, master;
    std::vector<uint8_t> others;
    double R;
};

const std::vector<SatSpec>& bigSky()
{
    static const std::vector<SatSpec> s = {
        {5, 5, 0, {3, 4}, -350.0},           {6, 6, 0, {3, 4}, 125.25},          {7, 7, 0, {3, 4}, 610.5},
        {70 + 11, 11, 17, {20, 21, 19}, 462.5}, {70 + 12, 12, 17, {20, 21, 19}, -95.0},
        {70 + 13, 13, 17, {20, 21, 19}, 240.0}, {140 + 20, 20, 28, {14, 30, 13}, -583.25},
        {140 + 21, 21, 28, {14, 30, 13}, 77.0},
    };
    return s;
}

using ErrFn = std::function<double(int, uint8_t)>;
using Cn0Fn = std::function<int(int, uint8_t)>;

std::vector<MeasT1> buildSky(double W, const ErrFn& err = nullptr, const Cn0Fn& cn0 = nullptr, double dR = 0.0)
{
    std::vector<MeasT1> v;
    const std::vector<SatSpec>& ss = bigSky();
    for (int s = 0; s < static_cast<int>(ss.size()); ++s)
    {
        auto rr = [&](uint8_t sig) { return rrOf(ss[s].R + dR, W, sig, err ? err(s, sig) : 0.0); };
        auto c = [&](uint8_t sig, int dflt) {
            const int q = cn0 ? cn0(s, sig) : -1;
            return static_cast<uint8_t>(q >= 0 ? q : dflt);
        };
        std::vector<Band> t2;
        for (uint8_t sig : ss[s].others)
            t2.push_back({sig, rr(sig), c(sig, 132)});
        v.push_back(sat(ss[s].svid, ss[s].ch, {ss[s].master, rr(ss[s].master), c(ss[s].master, 140)}, t2));
    }
    return v;
}

// White noise, reproducible: xorshift and Box-Muller
struct Gauss
{
    uint64_t x;
    explicit Gauss(uint64_t seed) : x(seed * 2654435761ULL + 1) {}
    double u()
    {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        return (static_cast<double>(x >> 11) + 0.5) / 9007199254740992.0;
    }
    double operator()() { return std::sqrt(-2.0 * std::log(u())) * std::cos(6.283185307179586 * u()); }
};

// Each band's noise at its model sigma (anchors at 45 dB-Hz, bands at 43), drawn afresh each epoch
struct SkyNoise
{
    Gauss g;
    double n[8][kMbSigs] = {};
    explicit SkyNoise(uint64_t seed) : g(seed) {}
    void draw(const MosaicMultibandConfig& c)
    {
        for (int s = 0; s < 8; ++s)
            for (int sig = 0; sig < kMbSigs; ++sig)
            {
                const bool anchor = sig == 0 || sig == 17 || sig == 28;
                n[s][sig] = c.band[sig].b > 0.0f ? g() * mbSigma(c.band[sig], anchor ? 45.0 : 43.0) : 0.0;
            }
    }
};

MosaicMultibandConfig defaults()
{
    MosaicMultibandConfig c;
    mosaicMbMeasured(c);
    c.on = true;
    return c;
}

int satIndex(const RawEpoch& ep, int s)
{
    const SatSpec& x = bigSky()[s];
    uint8_t prn = 0;
    const SbfSys sys = sbfSvid(x.svid, prn);
    const int gs = sys == SbfSys::GPS ? GNSS_SYS_GPS : sys == SbfSys::GAL ? GNSS_SYS_GAL : GNSS_SYS_BDS;
    return cellIndex(ep, gs, prn);
}

}  // namespace

TEST(MosaicMultiband, TheDefaultsReferABigSkyWithTheWeightedEstimate)
{
    // eight satellites, every band's own noise: the defaults (six values, a 1-sigma of 0.75 Hz, smoothed) refer every
    // epoch; the estimate stays within a few of its sigmas of the truth, and the rows on their anchors' wind-up
    Mb m;
    m.cfg = defaults();
    SkyNoise nz(7);
    const double W = 0.99 * 0.5;
    double worst = 0.0;
    for (int k = 0; k < 200; ++k)
    {
        nz.draw(m.cfg);
        RawEpoch ep;
        ASSERT_TRUE(m.grid(epoch(kTow0 + 50 * k, buildSky(W, [&](int s, uint8_t sig) { return nz.n[s][sig]; })), ep));
        ASSERT_EQ(m.out.w_src, 1);
        EXPECT_GE(m.out.w_kept, 6);
        worst = std::max(worst, std::fabs(m.out.w_hat - W) / std::max(1e-3f, m.out.w_sig));
    }
    EXPECT_EQ(m.st.epochs_w_est, 200u);
    EXPECT_EQ(m.st.epochs_few_w, 0u);
    EXPECT_LT(worst, 4.5);
    EXPECT_LT(m.out.w_sig, m.out.w_est_sig);   // smoothed: below one epoch's own
    EXPECT_EQ(m.st.guard_trips, 0u);           // noise alone never trips the drift guard
}

TEST(MosaicMultiband, TheSmoothedEstimateGatesEachSatelliteAndStepsOnlyWithAgreement)
{
    MosaicMultibandConfig c = defaults();
    MosaicMbScratch scr;
    MosaicMultibandStats st;
    MosaicMbOut out;
    MosaicMbWindupIn gyro;
    // eight satellites, each at its own value v[s] Hz with a 1 Hz sigma (two values each): an epoch k
    MosaicMbWSat ws[8];
    auto epochAt = [&](int k, std::function<float(int)> v, int n = 8) {
        for (int s = 0; s < n; ++s)
        {
            ws[s] = MosaicMbWSat{};
            ws[s].v = v(s) + 0.02f * (s - 4);
            ws[s].sig = 1.0f;
            ws[s].w = 1.0f;
            ws[s].nval = 2;
            ws[s].cell = static_cast<int16_t>(s);
            ws[s].prn = static_cast<uint8_t>(s + 1);
        }
        return mbWindupPick(ws, n, 2 * n, kTow0 + 50LL * k, c, gyro, scr, st, out);
    };
    auto all = [](float x) { return [x](int) { return x; }; };
    int k = 0;
    // a steady 1 Hz: the smoothed 1-sigma under the epoch's own (0.35)
    for (; k < 40; ++k)
        ASSERT_TRUE(epochAt(k, all(1.0f)));
    EXPECT_NEAR(out.w_hat, 1.0f, 1e-2);
    EXPECT_LT(out.w_sig, 0.3f);
    // one satellite 6 Hz off and staying there: gated out every epoch, the estimate unmoved
    for (int j = 0; j < 20; ++j, ++k)
    {
        ASSERT_TRUE(epochAt(k, [](int s) { return s == 0 ? 7.0f : 1.0f; }));
        EXPECT_NEAR(out.w_hat, 1.0f, 2e-2);
        EXPECT_EQ(out.w_gated, 1);
    }
    // one bad epoch (every satellite at 4 Hz): held back, then dropped
    ASSERT_TRUE(epochAt(k++, all(4.0f)));
    EXPECT_NEAR(out.w_hat, 1.0f, 2e-2);
    EXPECT_NEAR(out.w_est, 4.0f, 2e-2);   // the epoch's own, told
    ASSERT_TRUE(epochAt(k++, all(1.0f)));
    EXPECT_NEAR(out.w_hat, 1.0f, 2e-2);
    // a step every satellite makes and holds: the second epoch confirms it, the filter starts over there
    const uint32_t steps0 = st.w_steps;
    ASSERT_TRUE(epochAt(k++, all(5.0f)));
    EXPECT_NEAR(out.w_hat, 1.0f, 2e-2);
    ASSERT_TRUE(epochAt(k++, all(5.0f)));
    EXPECT_NEAR(out.w_hat, 5.0f, 2e-2);
    EXPECT_EQ(st.w_steps, steps0 + 1);
    // three of eight jump to -3 Hz and stay: never a step (the epoch's own estimate clips them; each is gated
    // against the prediction), the estimate unmoved
    for (int j = 0; j < 20; ++j, ++k)
    {
        ASSERT_TRUE(epochAt(k, [](int s) { return s < 3 ? -3.0f : 5.0f; }));
        EXPECT_NEAR(out.w_hat, 5.0f, 5e-2);
        EXPECT_EQ(out.w_gated, 3);
    }
    EXPECT_EQ(st.w_steps, steps0 + 1);
    // the same three at 2 Hz, and the others' noise: within their sigmas, so they pull the estimate as noise would
    // and the epoch's own moves past the gate now and then; never a step (those that stayed outweigh those that moved)
    for (int j = 0; j < 20; ++j, ++k)
        ASSERT_TRUE(epochAt(k, [&](int s) { return s < 3 ? 2.0f : 5.0f + 0.3f * ((j * 7 + s * 3) % 5 - 2); }));
    EXPECT_EQ(st.w_steps, steps0 + 1);
    // too few within the gate (one): the filter coasts; past the gap, started over at the epoch's own
    const float before = out.w_hat;
    ASSERT_TRUE(epochAt(k++, [](int s) { return s == 0 ? 5.0f : -2.0f; }, 1));
    EXPECT_FLOAT_EQ(out.w_hat, before);
    k += 10;
    ASSERT_TRUE(epochAt(k++, all(-2.0f)));
    EXPECT_NEAR(out.w_hat, -2.0f, 5e-2);
    // with the gyro's at hand it says whether the roll rate moved: a step it does not make is refused (the
    // estimate coasts, then the check against it sends the epochs anchor-only); one it makes is taken
    gyro = {true, -2.0f};
    for (int j = 0; j < 5; ++j)
        ASSERT_TRUE(epochAt(k++, all(-2.0f)));
    const uint32_t refused0 = st.w_steps_refused;
    ASSERT_TRUE(epochAt(k++, all(3.0f)));
    ASSERT_TRUE(epochAt(k++, all(3.0f)));
    EXPECT_NEAR(out.w_hat, -2.0f, 2e-2);
    EXPECT_EQ(st.w_steps_refused, refused0 + 2);
    bool alone = false;
    for (int j = 0; j < 20 && !alone; ++j)
        alone = !epochAt(k++, all(3.0f));
    EXPECT_TRUE(alone);
    EXPECT_GE(out.x_state, 4);
    gyro = {true, 3.0f};
    bool back = false;
    for (int j = 0; j < 20 && !back; ++j)
        back = epochAt(k++, all(3.0f));
    EXPECT_TRUE(back);
    EXPECT_NEAR(out.w_hat, 3.0f, 5e-2);
    gyro = {true, 8.0f};
    ASSERT_TRUE(epochAt(k++, all(8.0f)));
    ASSERT_TRUE(epochAt(k++, all(8.0f)));
    EXPECT_NEAR(out.w_hat, 8.0f, 5e-2);
    // a restart with too little weight is not used until the filter has gathered enough
    gyro = MosaicMbWindupIn{};
    c.w_max_sigma_hz = 0.3f;
    k += 20;
    EXPECT_FALSE(epochAt(k++, all(0.5f)));   // 0.35 Hz its own
    bool used = false;
    for (int j = 0; j < 10 && !used; ++j)
        used = epochAt(k++, all(0.5f));
    EXPECT_TRUE(used);
}

TEST(MosaicMultiband, TheGyrosWindUpStandsInWhereAskedOrWhereTheEstimateIsShort)
{
    const double W = 0.99 * 1.0;
    const std::vector<uint8_t> blk = epoch(kTow0, buildSky(W));
    const std::vector<uint8_t> thin = epoch(kTow0, {sat(5, 5, {0, rrOf(-100.0, W, 0)}, {{4, rrOf(-100.0, W, 4)}})});
    struct Case
    {
        MosaicMbWSrc src;
        bool gyro_ok;
        float gyro;
        bool big;
        uint8_t want_src;   // 0 anchor-only, 1 the estimate, 2 the gyro
    } cases[] = {
        {MosaicMbWSrc::EST, true, 0.0f, true, 1},  {MosaicMbWSrc::EST, true, 0.0f, false, 0},
        {MosaicMbWSrc::GYRO, true, 0.99f, true, 2}, {MosaicMbWSrc::GYRO, false, 0.0f, true, 0},
        {MosaicMbWSrc::AUTO, true, 0.0f, true, 1},  {MosaicMbWSrc::AUTO, true, 0.99f, false, 2},
        {MosaicMbWSrc::AUTO, false, 0.0f, false, 0},
    };
    for (const Case& k : cases)
    {
        SCOPED_TRACE(testing::Message() << "source " << int(k.src) << " gyro " << k.gyro_ok << " big " << k.big);
        Mb m;
        m.cfg = defaults();
        m.cfg.w_src = k.src;
        m.wext = {k.gyro_ok, k.gyro};
        RawEpoch ep;
        ASSERT_TRUE(m.grid(k.big ? blk : thin, ep));
        EXPECT_EQ(m.out.w_src, k.want_src);
        EXPECT_EQ(m.out.epoch, k.want_src == 0 ? 1 : 0);
        if (k.want_src == 0)
            continue;
        EXPECT_NEAR(m.out.w_hat, k.want_src == 2 ? k.gyro : W, 2e-3);
        // referred with the gyro's exact W the row is its anchor's range rate and wind-up
        const int i = k.big ? satIndex(ep, 3) : 0;
        ASSERT_EQ(m.out.row[i], MosaicMbRow::COMBINED);
        const double R = k.big ? bigSky()[3].R : -100.0;
        EXPECT_NEAR(ep.obs[i].rate_mps, R + W * lam(k.big ? 17 : 0), 2e-4);
    }
    // a gyro off by 1 Hz moves the row by that times sum_b w_b (lambda_b - lambda_a): the reason the estimate leads
    Mb m;
    m.cfg = defaults();
    m.cfg.w_src = MosaicMbWSrc::GYRO;
    m.wext = {true, static_cast<float>(W - 1.0)};
    RawEpoch ep;
    ASSERT_TRUE(m.grid(blk, ep));
    const int i = satIndex(ep, 3);
    double lev = 0.0;
    for (int q = 0; q < m.out.ncand[i]; ++q)
        lev += m.out.det[i][q].w * (lam(m.out.det[i][q].sig) - lam(17));
    EXPECT_NEAR(ep.obs[i].rate_mps - (bigSky()[3].R + W * lam(17)), lev, 2e-4);
    // the estimate against the gyro's, counted where both are
    Mb a;
    a.cfg = defaults();
    a.cfg.w_src = MosaicMbWSrc::AUTO;
    a.wext = {true, static_cast<float>(W)};
    ASSERT_TRUE(a.grid(blk, ep));
    EXPECT_EQ(a.st.wx_n, 1u);
    EXPECT_NEAR(a.st.wx_sum, 0.0, 2e-3);
}

// ── Composition changes ────────────────────────────────────────────────

TEST(MosaicMultiband, ABandJoiningOrLeavingStepsTheRowByItsWeightTimesItsBias)
{
    // Galileo E1's row (satellite 3); its E6 is X off the others (an inter-band bias), at 26 dB-Hz (under its 25 + 2)
    // the first epoch, 28 the second (it joins), and its lock 0 the third (it leaves). Referred with the exact W
    // (the gyro's), the row steps by w_E6 X as E6 joins and back as it leaves; with X 0 it does not move
    for (int src = 0; src < 2; ++src)
        for (double X : {0.0, 0.2, -0.15})
        {
            SCOPED_TRACE(testing::Message() << (src == 0 ? "the gyro's W" : "the estimate") << ", E6 off by " << X);
            Mb m;
            m.cfg = defaults();
            if (src == 0)
            {
                m.cfg.w_src = MosaicMbWSrc::GYRO;
                m.wext = {true, 0.99f * 0.5f};
            }
            const double W = 0.99 * 0.5;
            auto err = [&](int s, uint8_t sig) { return s == 3 && sig == 19 ? X : 0.0; };
            const uint8_t e6_cn0[] = {64, 72, 72};   // 26, 28, 28 dB-Hz
            double rate[3] = {}, w_e6 = 0.0;
            for (int k = 0; k < 3; ++k)
            {
                std::vector<MeasT1> sky = buildSky(W, err, [&](int s, uint8_t sig) {
                    return s == 3 && sig == 19 ? e6_cn0[k] : -1;
                });
                if (k == 2)
                    sky[3].t2[2].lock = 0;   // E6 relocked: out for a second
                RawEpoch ep;
                ASSERT_TRUE(m.grid(epoch(kTow0 + 50 * k, sky), ep));
                const int i = satIndex(ep, 3);
                ASSERT_EQ(m.out.row[i], MosaicMbRow::COMBINED);
                EXPECT_EQ((m.out.mask[i] >> 19) & 1u, k == 1 ? 1u : 0u);
                rate[k] = ep.obs[i].rate_mps;
                for (int q = 0; q < m.out.ncand[i]; ++q)
                    if (k == 1 && m.out.det[i][q].sig == 19)
                        w_e6 = m.out.det[i][q].w;
            }
            ASSERT_GT(w_e6, 0.0);
            // its share by hand: E6 at 28 dB-Hz against E1 at 45 and E5a, E5b at 43
            double ws = 0.0;
            for (const auto& b : {std::pair<uint8_t, double>{17, 45.0}, {20, 43.0}, {21, 43.0}, {19, 28.0}})
                ws += 1.0 / std::pow(mbSigma(m.cfg.band[b.first], b.second), 2);
            EXPECT_NEAR(w_e6, 1.0 / std::pow(mbSigma(m.cfg.band[19], 28.0), 2) / ws, 1e-6);
            const double tol = src == 0 ? 2e-4 : 2e-4 + 0.1 * std::fabs(w_e6 * X);
            EXPECT_NEAR(rate[1] - rate[0], w_e6 * X, tol);
            EXPECT_NEAR(rate[2] - rate[1], -w_e6 * X, tol);
            EXPECT_EQ(m.st.comp_changes, 2u);
        }
}

TEST(MosaicMultiband, WithoutABiasACompositionChangeMovesTheRowByNoiseAlone)
{
    // every band's own error, fixed over the three epochs (no bias between them beyond that noise): the row's step
    // as E6 joins is its weighted mean with E6's error less the one without it, exactly
    Mb m;
    m.cfg = defaults();
    m.cfg.w_src = MosaicMbWSrc::GYRO;
    const double W = 0.3;
    m.wext = {true, static_cast<float>(W)};
    const double n17 = 0.02, n20 = -0.05, n21 = 0.03, n19 = 0.09;
    auto err = [&](int s, uint8_t sig) {
        if (s != 3)
            return 0.0;
        return sig == 17 ? n17 : sig == 20 ? n20 : sig == 21 ? n21 : sig == 19 ? n19 : 0.0;
    };
    double rate[2] = {};
    for (int k = 0; k < 2; ++k)
    {
        RawEpoch ep;
        ASSERT_TRUE(m.grid(epoch(kTow0 + 50 * k, buildSky(W, err, [&](int s, uint8_t sig) {
                                     return s == 3 && sig == 19 ? (k == 0 ? 64 : 72) : -1;
                                 })),
                           ep));
        rate[k] = ep.obs[satIndex(ep, 3)].rate_mps;
    }
    auto mean = [&](bool with_e6) {
        double ws = 0.0, wn = 0.0;
        for (const auto& b : {std::tuple<uint8_t, double, double>{17, 45.0, n17}, {20, 43.0, n20}, {21, 43.0, n21},
                              {19, 28.0, n19}})
        {
            if (std::get<0>(b) == 19 && !with_e6)
                continue;
            const double w = 1.0 / std::pow(mbSigma(m.cfg.band[std::get<0>(b)], std::get<1>(b)), 2);
            ws += w;
            wn += w * std::get<2>(b);
        }
        return wn / ws;
    };
    EXPECT_NEAR(rate[1] - rate[0], mean(true) - mean(false), 2e-4);
}

// ── Faults on the anchor alone ─────────────────────────────────────────

namespace {

// Galileo satellite 3's E1 (its anchor) off by f(k) m/s at epoch k, every band's own noise from nz (or none), the
// wind-up the gyro's (exact): each epoch's row less the truth, what the row became, and the guard's mean
struct AnchorRun
{
    std::vector<double> off;
    std::vector<MosaicMbRow> row;
    std::vector<float> g_m;
};

AnchorRun anchorFault(Mb& m, int epochs, const std::function<double(int)>& f, SkyNoise* nz = nullptr,
                      bool gyro = true)
{
    const double W = 0.99 * 0.25;
    if (gyro)
    {
        m.cfg.w_src = MosaicMbWSrc::GYRO;
        m.wext = {true, static_cast<float>(W)};
    }
    AnchorRun r;
    for (int k = 0; k < epochs; ++k)
    {
        if (nz != nullptr)
            nz->draw(m.cfg);
        RawEpoch ep;
        EXPECT_TRUE(m.grid(epoch(kTow0 + 50 * k, buildSky(W, [&](int s, uint8_t sig) {
                                     return (nz != nullptr ? nz->n[s][sig] : 0.0) + (s == 3 && sig == 17 ? f(k) : 0.0);
                                 })),
                           ep));
        const int i = satIndex(ep, 3);
        r.off.push_back(ep.obs[i].has_rate ? ep.obs[i].rate_mps - (bigSky()[3].R + W * lam(17)) : NAN);
        r.row.push_back(m.out.row[i]);
        r.g_m.push_back(m.out.g_m[i]);
    }
    return r;
}

// the anchor's share of its row: E1 at 45 dB-Hz with E5a, E5b and E6 at 43
double anchorShare(const MosaicMultibandConfig& c)
{
    double ws = 0.0;
    for (uint8_t sig : {17, 20, 21, 19})
        ws += 1.0 / std::pow(mbSigma(c.band[sig], sig == 17 ? 45.0 : 43.0), 2);
    return 1.0 / std::pow(mbSigma(c.band[17], 45.0), 2) / ws;
}

}  // namespace

TEST(MosaicMultiband, AStepOnTheAnchorAloneOverTheScreenIsVotedOutOrLeftAlone)
{
    // 0.8 m/s on E1 with three agreeing bands: voted out, no Doppler; on GPS L1 C/A with L5 alone (one band to
    // judge by), the band leaves and the row is its anchor's with the whole step, as the image's L1 row would be
    Mb m;
    m.cfg = defaults();
    m.cfg.guard = false;
    AnchorRun r = anchorFault(m, 1, [](int) { return 0.8; });
    EXPECT_EQ(r.row[0], MosaicMbRow::VOTED);
    EXPECT_TRUE(std::isnan(r.off[0]));
    Mb g;
    g.cfg = defaults();
    g.cfg.w_src = MosaicMbWSrc::GYRO;
    const double W = 0.25;
    g.wext = {true, static_cast<float>(W)};
    std::vector<MeasT1> sky = buildSky(W, [](int s, uint8_t sig) { return s == 0 && sig == 0 ? 0.8 : 0.0; });
    sky[0].t2.erase(sky[0].t2.begin());   // GPS 5: L5 alone beside L1 C/A
    RawEpoch ep;
    ASSERT_TRUE(g.grid(epoch(kTow0, sky), ep));
    const int i = satIndex(ep, 0);
    EXPECT_EQ(g.out.row[i], MosaicMbRow::ALONE);
    EXPECT_NEAR(ep.obs[i].rate_mps - (bigSky()[0].R + W * lam(0)), 0.8, 2e-4);
}

TEST(MosaicMultiband, AStepUnderTheScreenReachesTheRowDilutedUntilTheDriftGuardDropsIt)
{
    // 0.2 m/s on E1 alone, under every band's screen (5 sigma of the difference: 0.29-0.36 m/s here): without the
    // guard it stays in the row at the anchor's share, about 0.47 of it, for good. The guard drops the Doppler once
    // its running mean's sigma (one epoch's over the root of their number) is an eighth of the step: the fourth
    // epoch; noise-free here
    const double wa = anchorShare(defaults());
    {
        Mb m;
        m.cfg = defaults();
        m.cfg.guard = false;
        AnchorRun r = anchorFault(m, 100, [](int) { return 0.2; });
        for (size_t k = 0; k < r.off.size(); ++k)
        {
            ASSERT_EQ(r.row[k], MosaicMbRow::COMBINED);
            EXPECT_NEAR(r.off[k], wa * 0.2, 2e-4);
        }
        EXPECT_GT(wa, 0.3);
        EXPECT_LT(wa, 0.6);
    }
    Mb m;
    m.cfg = defaults();
    AnchorRun r = anchorFault(m, 100, [](int) { return 0.2; });
    int first = -1;
    for (size_t k = 0; k < r.row.size(); ++k)
    {
        if (first < 0 && r.row[k] == MosaicMbRow::DRIFT)
            first = static_cast<int>(k);
        if (first < 0)
        {
            EXPECT_EQ(r.row[k], MosaicMbRow::COMBINED);
            EXPECT_NEAR(r.off[k], wa * 0.2, 2e-4);   // what reaches the row before: the anchor's share
        }
        else
        {
            EXPECT_EQ(r.row[k], MosaicMbRow::DRIFT) << "epoch " << k;
        }
    }
    EXPECT_GT(first, 0);
    EXPECT_LE(first, 4);
    EXPECT_NEAR(r.g_m[first], 0.2, 1e-3);
    EXPECT_EQ(m.st.guard_trips, 1u);
    EXPECT_EQ(m.st.voted, 0u);
}

TEST(MosaicMultiband, ARampOnTheAnchorAloneIsVotedOutAtTheScreenOrDroppedEarlierByTheGuard)
{
    // E1 ramping 0.1 m/s a second (5 mm/s an epoch at 20 Hz), every band's own noise: without the guard the row
    // carries the anchor's share of the ramp until two bands' screens flag it and they vote it out; with the guard
    // the leaky mean of E1 less the others crosses its threshold well before
    const double wa = anchorShare(defaults());
    auto ramp = [](int k) { return 0.005 * k; };
    int voted_at = -1, dropped_at = -1;
    double reached_vote = 0.0, reached_guard = 0.0;
    {
        Mb m;
        m.cfg = defaults();
        m.cfg.guard = false;
        SkyNoise nz(11);
        AnchorRun r = anchorFault(m, 200, ramp, &nz);
        for (size_t k = 0; k < r.row.size() && voted_at < 0; ++k)
            if (r.row[k] == MosaicMbRow::VOTED)
                voted_at = static_cast<int>(k);
            else
                reached_vote = std::max(reached_vote, wa * ramp(static_cast<int>(k)));
    }
    {
        Mb m;
        m.cfg = defaults();
        SkyNoise nz(11);
        AnchorRun r = anchorFault(m, 200, ramp, &nz);
        for (size_t k = 0; k < r.row.size() && dropped_at < 0; ++k)
            if (r.row[k] == MosaicMbRow::DRIFT)
                dropped_at = static_cast<int>(k);
            else
                reached_guard = std::max(reached_guard, wa * ramp(static_cast<int>(k)));
        // once dropped, it stays dropped while the ramp goes on
        for (size_t k = static_cast<size_t>(dropped_at); k < r.row.size(); ++k)
            EXPECT_TRUE(r.row[k] == MosaicMbRow::DRIFT || r.row[k] == MosaicMbRow::VOTED) << "epoch " << k;
    }
    std::printf("  ramp 0.1 m/s/s on E1 alone: voted out at %.2f m/s (epoch %d), the row carrying up to %.3f m/s of "
                "it; the drift guard drops it at %.2f m/s (epoch %d), the row carrying up to %.3f m/s\n",
                ramp(voted_at), voted_at, reached_vote, ramp(dropped_at), dropped_at, reached_guard);
    ASSERT_GT(voted_at, 0);
    ASSERT_GT(dropped_at, 0);
    EXPECT_GT(ramp(voted_at), 0.25);
    EXPECT_LT(ramp(dropped_at), 0.16);
    EXPECT_LT(reached_guard, 0.6 * reached_vote);
}

TEST(MosaicMultiband, TheDriftGuardLetsGoWhenTheAnchorComesBackAndStartsOverAfterAGap)
{
    // 0.3 m/s on E1 for 2 s, then none: dropped within a few epochs, back once its mean is within half the threshold
    Mb m;
    m.cfg = defaults();
    m.cfg.screen_k = 50.0f;   // the screen out of the way: the guard alone
    AnchorRun r = anchorFault(m, 400, [](int k) { return k < 40 ? 0.3 : 0.0; });
    EXPECT_EQ(r.row[0], MosaicMbRow::COMBINED);   // one epoch's difference: its sigma alone, 8 of them over 0.3
    EXPECT_EQ(r.row[3], MosaicMbRow::DRIFT);
    EXPECT_EQ(r.row[39], MosaicMbRow::DRIFT);
    int back = -1;
    for (size_t k = 40; k < r.row.size() && back < 0; ++k)
        if (r.row[k] == MosaicMbRow::COMBINED)
            back = static_cast<int>(k);
    ASSERT_GT(back, 40);
    for (size_t k = static_cast<size_t>(back); k < r.row.size(); ++k)
        EXPECT_EQ(r.row[k], MosaicMbRow::COMBINED) << "epoch " << k;
    EXPECT_NEAR(r.off.back(), 0.0, 2e-4);
    // its own values stay out of the estimate while it is held
    Mb e;
    e.cfg = defaults();
    e.cfg.screen_k = 50.0f;
    AnchorRun s = anchorFault(e, 10, [](int) { return 0.3; }, nullptr, false);
    EXPECT_EQ(s.row.back(), MosaicMbRow::DRIFT);
    EXPECT_EQ(e.out.w_n, 3 * 2 + 3 * 3 + 2 * 2 - 3);   // GPS two values, Galileo three, BeiDou two (B1C's lambda is
                                                        // B1I's), less the held satellite's three
    EXPECT_NEAR(e.out.w_hat, 0.99f * 0.25f, 2e-3);
    // a held satellite not tracked for more than guard_gap_s (here no epoch at all: a new track) starts over: its
    // Doppler back, alone. Held with nothing to compare while it is tracked, it stays held
    // (TheDriftGuardHoldsThroughAnchorOnlyEpochs)
    Mb h;
    h.cfg = defaults();
    h.cfg.screen_k = 50.0f;
    anchorFault(h, 5, [](int) { return 0.3; });
    RawEpoch ep;
    std::vector<MeasT1> sky = buildSky(0.99 * 0.25, [](int s, uint8_t sig) { return s == 3 && sig == 17 ? 0.3 : 0.0; });
    sky[3].t2.clear();   // E1 alone
    ASSERT_TRUE(h.grid(epoch(kTow0 + 50 * 5, sky), ep));
    EXPECT_EQ(h.out.row[satIndex(ep, 3)], MosaicMbRow::DRIFT);   // within the gap: still held
    ASSERT_TRUE(h.grid(epoch(kTow0 + 50 * 5 + 600, sky), ep));
    EXPECT_EQ(h.out.row[satIndex(ep, 3)], MosaicMbRow::ALONE);
    EXPECT_TRUE(ep.obs[satIndex(ep, 3)].has_rate);
}

TEST(MosaicMultiband, NoiseAloneNeverTripsTheDriftGuardOrTheNewRules)
{
    // 20 minutes of 20 Hz epochs, every band at its model sigma, the estimate referring with the gyro's (exact) given
    // as the driver would give it: no guard trip (its largest mean within a few of its sigmas), no step, no epoch
    // anchor-only by the check against the gyro's or by the common offset, no system rule
    Mb m;
    m.cfg = defaults();
    m.wext = {true, 0.2f};
    SkyNoise nz(3);
    for (int k = 0; k < 24000; ++k)
    {
        nz.draw(m.cfg);
        RawEpoch ep;
        ASSERT_TRUE(m.grid(epoch(kTow0 + 50 * k, buildSky(0.2, [&](int s, uint8_t sig) { return nz.n[s][sig]; })),
                           ep));
    }
    EXPECT_EQ(m.st.guard_trips, 0u);
    EXPECT_EQ(m.st.guard_rows, 0u);
    EXPECT_LT(m.st.guard_max_z, 5.0f);
    EXPECT_GT(m.st.guard_looks, 100000u);
    EXPECT_EQ(m.st.epochs_w_est, 24000u);
    EXPECT_EQ(m.st.w_steps, 0u);
    EXPECT_EQ(m.st.x_trips, 0u);
    EXPECT_EQ(m.st.epochs_xcheck, 0u);
    EXPECT_LT(m.st.x_max_r, 0.6f);
    EXPECT_EQ(m.st.common_trips, 0u);
    EXPECT_EQ(m.st.epochs_common, 0u);
    EXPECT_EQ(m.st.sys_rows, 0u);
    EXPECT_EQ(m.st.sys_instant, 0u);
    EXPECT_LT(m.st.mon_max_r, 0.5f);
    std::printf("  noise, 24000 epochs: largest guard |mean| %.1f sigma, the check %.2f of its threshold, the common "
                "offset %.2f of its threshold\n",
                static_cast<double>(m.st.guard_max_z), static_cast<double>(m.st.x_max_r),
                static_cast<double>(m.st.mon_max_r));
}

// ── The review's three defects (each test failed on a08bdcde) ──────────
// The review's synthetic cases (gnss-attitude-study/v9-mosaic-filter-plan/evidence/multiband/scripts/prep/adv/
// break_guard.cpp), as tests: one satellite capturing the wind-up estimate, the drift guard letting a held satellite
// go, and a fault common to every anchor (or to one system's) that the combination turned into satellite-specific
// errors. Written against the API a08bdcde had, so each one compiles there and fails.

namespace {

// bigSky()'s satellites, changeable (a satellite cut to fewer bands)
std::vector<SatSpec> skyOf() { return bigSky(); }

bool isAnchorSig(uint8_t sig) { return sig == 0 || sig == 17 || sig == 28; }

// ss's satellites at R_s + dR_s + W lambda + err(s, sig), raw C/N0 cn0(s, sig) (-1: 140 an anchor, 132 a band)
std::vector<MeasT1> buildSkyOf(const std::vector<SatSpec>& ss, double W, const ErrFn& err, const Cn0Fn& cn0 = nullptr,
                               const std::vector<double>* dR = nullptr)
{
    std::vector<MeasT1> v;
    for (int s = 0; s < static_cast<int>(ss.size()); ++s)
    {
        const double R = ss[s].R + (dR != nullptr ? (*dR)[s] : 0.0);
        auto rr = [&](uint8_t sig) { return rrOf(R, W, sig, err ? err(s, sig) : 0.0); };
        auto c = [&](uint8_t sig, int dflt) {
            const int q = cn0 ? cn0(s, sig) : -1;
            return static_cast<uint8_t>(q >= 0 ? q : dflt);
        };
        std::vector<Band> t2;
        for (uint8_t sig : ss[s].others)
            t2.push_back({sig, rr(sig), c(sig, 132)});
        v.push_back(sat(ss[s].svid, ss[s].ch, {ss[s].master, rr(ss[s].master), c(ss[s].master, 140)}, t2));
    }
    return v;
}

int cellOfSat(const RawEpoch& ep, const SatSpec& x)
{
    uint8_t prn = 0;
    const SbfSys sys = sbfSvid(x.svid, prn);
    const int gs = sys == SbfSys::GPS ? GNSS_SYS_GPS : sys == SbfSys::GAL ? GNSS_SYS_GAL : GNSS_SYS_BDS;
    return cellIndex(ep, gs, prn);
}

// A band's noise at its model sigma for a raw C/N0
double noiseAt(Gauss& g, const MosaicMultibandConfig& c, uint8_t sig, int raw)
{
    return c.band[sig].b > 0.0f ? g() * mbSigma(c.band[sig], raw * 0.25 + 10.0) : 0.0;
}

}  // namespace

TEST(MosaicMultiband, OneStrongSatellitesAnchorFaultDoesNotCaptureTheWindUpEstimate)
{
    // The review's capture (break_guard.cpp E): Galileo satellite 3 strong (E1 at raw qa, its E5a, E5b and E6 8 under),
    // the other seven at 30 / 28 dB-Hz, so it held 0.5-0.9 of a08bdcde's estimate's weight; its E1 alone +F from 2 s;
    // every band's own noise; the estimate referring, no gyro. There W_hat moved 1.4-5 Hz with the fault, the faulted
    // row kept 0.6-1.0 of it and the guard dropped up to 63 % of the healthy rows. Scored over 4-8 s: W_hat within 1 Hz
    // of the truth on average, the faulted satellite carrying under a tenth of its fault on average (a dropped
    // Doppler carries none), no healthy row dropped
    const double W = 0.99 * 0.25;
    for (double F : {0.2, 0.3, 0.6})
        for (int qa : {120, 148})
            for (uint64_t seed : {21u, 22u})
            {
                SCOPED_TRACE(testing::Message() << "F " << F << " m/s, anchor " << qa * 0.25 + 10 << " dB-Hz, seed "
                                                << seed);
                Mb m;
                m.cfg = defaults();
                Gauss g(seed);
                const std::vector<SatSpec> ss = skyOf();
                double werr = 0.0, carried = 0.0;
                int nw = 0, n_faulted = 0, healthy_dropped = 0, healthy = 0;
                for (int k = 0; k < 160; ++k)
                {
                    const double t = 0.05 * k;
                    auto raw = [&](int s, uint8_t sig) {
                        return s == 3 ? (isAnchorSig(sig) ? qa : qa - 8) : (isAnchorSig(sig) ? 80 : 72);
                    };
                    auto err = [&](int s, uint8_t sig) {
                        return noiseAt(g, m.cfg, sig, raw(s, sig)) + (s == 3 && sig == 17 && t >= 2.0 ? F : 0.0);
                    };
                    RawEpoch ep;
                    ASSERT_TRUE(m.grid(epoch(kTow0 + 50 * k, buildSkyOf(ss, W, err, raw)), ep));
                    if (t < 4.0)
                        continue;
                    if (m.out.w_src != 0)
                    {
                        werr += std::fabs(m.out.w_hat - W);
                        nw++;
                    }
                    for (int s = 0; s < 8; ++s)
                    {
                        const int i = cellOfSat(ep, ss[s]);
                        if (s == 3)
                        {
                            n_faulted++;
                            if (ep.obs[i].has_rate)
                                carried += ep.obs[i].rate_mps - (ss[s].R + W * lam(17));
                        }
                        else
                        {
                            healthy++;
                            healthy_dropped += ep.obs[i].has_rate ? 0 : 1;
                        }
                    }
                }
                ASSERT_GT(nw, 0);
                EXPECT_LT(werr / nw, 1.0) << "mean |W_hat - W|, Hz";
                EXPECT_LT(std::fabs(carried / n_faulted), 0.1 * F) << "the fault the faulted row carries, mean";
                EXPECT_EQ(healthy_dropped, 0) << "of " << healthy << " healthy rows";
            }
}

TEST(MosaicMultiband, TheEstimateAgainstTheGyrosWindUpSendsAPersistentDisagreementAnchorOnly)
{
    // The estimate refers; the gyro's wind-up is given each epoch as the driver would give it. Where the two disagree
    // for more than a moment (here the gyro's 3 Hz off from 2 s to 4 s: either one could be the wrong one), the epoch
    // goes anchor-only, the image's rows; once they agree again it refers again (here by 6.5 s: the check lets go
    // after 0.5 s of agreement, and the anchors' common offset, which the gyro's error looked like at its 3 Hz times
    // the bands' wavelengths, once its satellites' means are back). a08bdcde never compared them
    const double W = 0.99 * 0.25;
    Mb m;
    m.cfg = defaults();
    SkyNoise nz(5);
    int alone_mid = 0, n_mid = 0, combined_after = 0, n_after = 0, combined_before = 0, n_before = 0;
    for (int k = 0; k < 180; ++k)
    {
        const double t = 0.05 * k;
        m.wext = {true, static_cast<float>(W + (t >= 2.0 && t < 4.0 ? 3.0 : 0.0))};
        nz.draw(m.cfg);
        RawEpoch ep;
        ASSERT_TRUE(m.grid(epoch(kTow0 + 50 * k, buildSky(W, [&](int s, uint8_t sig) { return nz.n[s][sig]; })), ep));
        const int i = satIndex(ep, 3);
        if (t >= 1.0 && t < 2.0)
        {
            n_before++;
            combined_before += m.out.row[i] == MosaicMbRow::COMBINED;
        }
        if (t >= 2.5 && t < 4.0)
        {
            n_mid++;
            alone_mid += m.out.epoch != 0 && m.out.row[i] == MosaicMbRow::EPOCH;
        }
        if (t >= 6.5)
        {
            n_after++;
            combined_after += m.out.row[i] == MosaicMbRow::COMBINED;
        }
    }
    EXPECT_EQ(combined_before, n_before);
    EXPECT_EQ(alone_mid, n_mid);
    EXPECT_EQ(combined_after, n_after);
}

TEST(MosaicMultiband, TheDriftGuardKeepsAHeldSatelliteOutWhenItsOnlyBandIsScreenedOut)
{
    // The review's case A: GPS 5's L1 C/A with L5 alone beside it, a ramp on L1 C/A, every band's own noise. The guard
    // drops its Doppler early in the ramp; a08bdcde then let it go once the fault grew past L5's screen (the guard not
    // looked at, its hold expired after guard_gap_s) and the row went back ALONE with the whole fault. Once dropped,
    // the Doppler must stay out while the ramp goes on: with the gyro's wind-up or the estimate, 0.1 and 0.03 m/s a second
    struct Case
    {
        bool gyro;
        double rate, secs;
    } cases[] = {{true, 0.1, 15.0}, {false, 0.1, 15.0}, {true, 0.03, 30.0}};
    for (const Case& c : cases)
    {
        SCOPED_TRACE(testing::Message() << (c.gyro ? "the gyro's W" : "the estimate") << ", ramp " << c.rate);
        std::vector<SatSpec> ss = skyOf();
        ss[0].others = {4};
        Mb m;
        m.cfg = defaults();
        const double W = 0.99 * 0.25;
        if (c.gyro)
        {
            m.cfg.w_src = MosaicMbWSrc::GYRO;
            m.wext = {true, static_cast<float>(W)};
        }
        Gauss g(5);
        int first = -1, back = 0;
        double worst = 0.0;
        const int n = static_cast<int>(c.secs * 20);
        for (int k = 0; k < n; ++k)
        {
            const double f = c.rate * k * 0.05;
            auto err = [&](int s, uint8_t sig) {
                return noiseAt(g, m.cfg, sig, isAnchorSig(sig) ? 140 : 132) + (s == 0 && sig == 0 ? f : 0.0);
            };
            RawEpoch ep;
            ASSERT_TRUE(m.grid(epoch(kTow0 + 50 * k, buildSkyOf(ss, W, err)), ep));
            const int i = cellOfSat(ep, ss[0]);
            if (first < 0 && m.out.row[i] == MosaicMbRow::DRIFT)
                first = k;
            if (first >= 0 && ep.obs[i].has_rate)
            {
                back++;
                worst = std::max(worst, std::fabs(ep.obs[i].rate_mps - (ss[0].R + W * lam(0))));
            }
        }
        ASSERT_GE(first, 0) << "never dropped";
        EXPECT_LT(c.rate * first * 0.05, 0.2) << "dropped at";
        EXPECT_EQ(back, 0) << "epochs its Doppler came back after the drop; the largest error carried " << worst;
    }
}

TEST(MosaicMultiband, AnAnchorThatOutgrowsItsOnlyBandAtOnceIsHeldToo)
{
    // The evaluation's one-band GPS satellite with a 1 m/s step on its L1 C/A (the cone-off rig: 1.3 m/s of velocity
    // error on a08bdcde, 1.7 on the image): its only band leaves at the screen in the first epoch, so the guard,
    // fed only by combinations, never saw it and the anchor stayed in alone with the whole step. An epoch that
    // screens out every band feeds the guard too, as a vote does: held within a few epochs and for good while the
    // step lasts; a single such epoch (a glitch) holds nothing
    for (int len : {1, 200})
    {
        SCOPED_TRACE(testing::Message() << "a step of " << len << " epochs");
        std::vector<SatSpec> ss = skyOf();
        ss[0].others = {4};
        Mb m;
        m.cfg = defaults();
        const double W = 0.99 * 0.25;
        m.wext = {true, static_cast<float>(W)};
        Gauss g(31);
        int first = -1, alone = 0, back = 0;
        for (int k = 0; k < 40 + 200; ++k)
        {
            const bool on = k >= 40 && k < 40 + len;
            auto err = [&](int s, uint8_t sig) {
                return noiseAt(g, m.cfg, sig, isAnchorSig(sig) ? 140 : 132) + (s == 0 && sig == 0 && on ? 1.0 : 0.0);
            };
            RawEpoch ep;
            ASSERT_TRUE(m.grid(epoch(kTow0 + 50 * k, buildSkyOf(ss, W, err)), ep));
            const int i = cellOfSat(ep, ss[0]);
            if (k < 40)
                continue;
            if (first < 0 && m.out.row[i] == MosaicMbRow::DRIFT)
                first = k - 40;
            if (on && ep.obs[i].has_rate)
                alone++;
            if (!on && ep.obs[i].has_rate)
                back++;
        }
        if (len == 1)
        {
            EXPECT_EQ(first, -1);
            EXPECT_EQ(back, 199);   // the glitch's one epoch alone, as the image's row; nothing held after it
        }
        else
        {
            ASSERT_GE(first, 0);
            EXPECT_LE(first, 8);
            EXPECT_LE(alone, 8) << "epochs the row went in with the step";
        }
    }
}

TEST(MosaicMultiband, TheDriftGuardHoldsThroughAnchorOnlyEpochs)
{
    // The review's case C: Galileo 11's E1 0.2 m/s off (under the screen) from the start, held by the guard; from 3 s
    // every other satellite's bands are gone for 0.8 s (an obstruction: too few wind-up values, every epoch
    // anchor-only), then back. a08bdcde let the hold expire after guard_gap_s: the anchor went in with its 0.2 m/s
    // and, once the bands came back, combined again for a few epochs. The hold must last through the gap and after
    const double W = 0.99 * 0.25;
    Mb m;
    m.cfg = defaults();
    Gauss g(13);
    int first = -1, back = 0, alone_epochs = 0;
    for (int k = 0; k < 200; ++k)
    {
        const double t = 0.05 * k;
        std::vector<SatSpec> cur = skyOf();
        if (t >= 3.0 && t < 3.8)
            for (size_t s = 0; s < cur.size(); ++s)
                if (s != 3)
                    cur[s].others.clear();
        auto err = [&](int s, uint8_t sig) {
            return noiseAt(g, m.cfg, sig, isAnchorSig(sig) ? 140 : 132) + (s == 3 && sig == 17 ? 0.2 : 0.0);
        };
        RawEpoch ep;
        ASSERT_TRUE(m.grid(epoch(kTow0 + 50 * k, buildSkyOf(cur, W, err)), ep));
        const int i = cellOfSat(ep, cur[3]);
        alone_epochs += m.out.epoch != 0;
        if (first < 0 && m.out.row[i] == MosaicMbRow::DRIFT)
            first = k;
        if (first >= 0 && ep.obs[i].has_rate)
            back++;
    }
    ASSERT_GE(first, 0);
    EXPECT_LT(first, 20);
    EXPECT_GE(alone_epochs, 8);   // the gap's epochs went anchor-only, once the smoother's 0.25 s coast ran out
    EXPECT_EQ(back, 0) << "epochs its Doppler came back after the drop";
}

TEST(MosaicMultiband, TheDriftGuardAllowsForAcceleration)
{
    // The review's case B (boost): every extra band 1.0 ms behind its anchor (inside max_lag_ms), so under a
    // line-of-sight acceleration a it reads a x 1 ms less; each satellite sees a share (0.4-0.95) of 150 m/s^2 for
    // 2 s, then -12 (coast); the gyro's wind-up (0). The screen allows max_lag 3 ms x |a_hat| and flags nothing;
    // a08bdcde's guard had no such allowance and dropped 15-20 of the 80 rows every 0.5 s from 1.5 s into the boost to
    // 2 s into coast. With the screen's allowance it drops none
    const std::vector<SatSpec> ss = skyOf();
    Mb m;
    m.cfg = defaults();
    m.cfg.w_src = MosaicMbWSrc::GYRO;
    m.wext = {true, 0.0f};
    const double share[8] = {0.95, 0.80, 0.55, 0.90, 0.70, 0.40, 0.85, 0.60};
    Gauss g(9);
    std::vector<double> dR(ss.size(), 0.0);
    const int pad = 200;   // 10 s on the pad first
    int drift = 0, rows = 0;
    for (int k = 0; k < pad + 160; ++k)
    {
        const double t = (k - pad) * 0.05;
        const double a = t < 0.0 ? 0.0 : (t < 2.0 ? 150.0 : -12.0);
        for (size_t s = 0; s < ss.size(); ++s)
            dR[s] += a * share[s] * 0.05;
        auto err = [&](int s, uint8_t sig) {
            return noiseAt(g, m.cfg, sig, isAnchorSig(sig) ? 140 : 132) -
                   (isAnchorSig(sig) ? 0.0 : a * share[s] * 1.0e-3);
        };
        RawEpoch ep;
        ASSERT_TRUE(m.grid(epoch(kTow0 + 50 * k, buildSkyOf(ss, 0.0, err, nullptr, &dR)), ep));
        if (k < pad)
            continue;
        for (size_t s = 0; s < ss.size(); ++s)
        {
            rows++;
            drift += m.out.row[cellOfSat(ep, ss[s])] == MosaicMbRow::DRIFT;
        }
    }
    EXPECT_EQ(drift, 0) << "rows the guard dropped of " << rows;
}

TEST(MosaicMultiband, AFaultCommonToEveryAnchorGoesAnchorOnlyAsTheImagesRowsWould)
{
    // The review's third defect: every satellite's anchor +0.3 m/s from 2 s (a fault common to the L1 band). The
    // image's rows carry it whole on every satellite and the receiver clock's drift takes it: error 0. On a08bdcde
    // the estimate read it as wind-up and handed a different share back to each row; with the gyro's wind-up the
    // drift guard dropped nearly every satellite's Doppler. Over 3-8 s each epoch must go anchor-only (the image's
    // rows: every Doppler kept, each its anchor's own), with the estimate referring (the gyro's given as the driver
    // would give it) and with the gyro's referring
    const double W = 0.99 * 0.25;
    for (int src = 0; src < 2; ++src)
    {
        SCOPED_TRACE(src == 0 ? "the estimate, the gyro's given" : "the gyro's");
        Mb m;
        m.cfg = defaults();
        if (src == 1)
            m.cfg.w_src = MosaicMbWSrc::GYRO;
        m.wext = {true, static_cast<float>(W)};
        SkyNoise nz(17);
        int alone = 0, n = 0, dropped = 0, not_own = 0;
        for (int k = 0; k < 160; ++k)
        {
            const double t = 0.05 * k;
            nz.draw(m.cfg);
            const std::vector<uint8_t> blk = epoch(kTow0 + 50 * k, buildSky(W, [&](int s, uint8_t sig) {
                return nz.n[s][sig] + (isAnchorSig(sig) && t >= 2.0 ? 0.3 : 0.0);
            }));
            RawEpoch ep, ref;
            ASSERT_TRUE(m.grid(blk, ep));
            MosaicJumpState js;
            MosaicEpochInfo info;
            ASSERT_TRUE(rawEpochFromMeasEpoch(blk.data(), blk.size(), ref, js, info));
            if (t < 3.0)
                continue;
            n++;
            alone += m.out.epoch != 0;
            for (int i = 0; i < ep.n; ++i)
            {
                dropped += ep.obs[i].has_rate ? 0 : 1;
                not_own += ep.obs[i].has_rate && ep.obs[i].rate_mps != ref.obs[i].rate_mps;
            }
        }
        EXPECT_EQ(alone, n) << "epochs anchor-only";
        EXPECT_EQ(dropped, 0) << "Dopplers dropped";
        EXPECT_EQ(not_own, 0) << "rows not their anchor's own";
    }
}

TEST(MosaicMultiband, AFaultCommonToOneSystemsAnchorsIsNotThinnedToItsSingleBandSatellites)
{
    // The evaluation's GPS-wide 1 m/s step: every GPS satellite's L1 C/A +1 m/s; GPS 5 has L5 alone beside it. On
    // a08bdcde the vote dropped GPS 6 and 7 (two agreeing bands each) and GPS 5's L5 left at the screen, so GPS 5 went
    // in alone with the whole step: one faulted row in place of three, under the filter's epoch test (on the rigs the
    // velocity ran to 1 m/s against the image's 29-241 mm/s). The system's rows must be treated together -- every GPS
    // row with its anchor's Doppler, or none with a Doppler -- and the other systems' rows left combined
    const double W = 0.99 * 0.25;
    for (int src = 0; src < 2; ++src)
    {
        SCOPED_TRACE(src == 0 ? "the estimate" : "the gyro's");
        std::vector<SatSpec> ss = skyOf();
        ss[0].others = {4};
        Mb m;
        m.cfg = defaults();
        if (src == 1)
            m.cfg.w_src = MosaicMbWSrc::GYRO;
        m.wext = {true, static_cast<float>(W)};
        Gauss g(23);
        int split = 0, n = 0, others_not_combined = 0, with_gps = 0;
        for (int k = 0; k < 120; ++k)
        {
            const double t = 0.05 * k;
            auto err = [&](int s, uint8_t sig) {
                return noiseAt(g, m.cfg, sig, isAnchorSig(sig) ? 140 : 132) + (s < 3 && sig == 0 && t >= 2.0 ? 1.0 : 0.0);
            };
            RawEpoch ep;
            ASSERT_TRUE(m.grid(epoch(kTow0 + 50 * k, buildSkyOf(ss, W, err)), ep));
            if (t < 2.0)
                continue;
            n++;
            int with = 0;
            for (int s = 0; s < 3; ++s)
                with += ep.obs[cellOfSat(ep, ss[s])].has_rate ? 1 : 0;
            split += with != 0 && with != 3;
            with_gps += with;
            for (int s = 3; s < 8; ++s)
                others_not_combined += m.out.row[cellOfSat(ep, ss[s])] != MosaicMbRow::COMBINED;
        }
        EXPECT_EQ(split, 0) << "epochs with the GPS rows split, of " << n;
        EXPECT_EQ(others_not_combined, 0);
        EXPECT_EQ(with_gps, 0);   // the system's rule, sys_action DROP: none of them with a Doppler
    }
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

// What the mosaic image flies (main.cpp rawMbStart) and --receiver mosaic replays: the measured model, on, the
// profile's Doppler floors on the anchors, the builder's white model, the rest the defaults.
TEST(MosaicMultiband, TheFlightConfigIsTheMeasuredModelOnWithTheProfilesFloors)
{
    MosaicMultibandConfig c;
    const float floors[GNSS_SYS_COUNT] = {25.0f, 27.0f, 29.0f, 31.0f};
    mosaicMbFlight(c, floors, 0.01, 0.075);
    MosaicMultibandConfig m;
    mosaicMbMeasured(m);
    EXPECT_TRUE(c.on);
    for (int k = 0; k < GNSS_SYS_COUNT; ++k)
        EXPECT_FLOAT_EQ(c.anchor_cn0_min[k], floors[k]);
    EXPECT_FLOAT_EQ(c.rr_a, 0.01f);
    EXPECT_FLOAT_EQ(c.rr_b, 0.075f);
    int joins = 0;
    for (int i = 0; i < kMbSigs; ++i)
    {
        EXPECT_EQ(c.band[i].join, m.band[i].join) << "signal " << i;
        EXPECT_FLOAT_EQ(c.band[i].b, m.band[i].b) << "signal " << i;
        EXPECT_FLOAT_EQ(c.band[i].lag_ms, 0.0f) << "signal " << i;
        joins += c.band[i].join ? 1 : 0;
    }
    EXPECT_EQ(joins, 8);   // GPS L2C, L5; Galileo E6, E5a, E5b; BeiDou B1C, B2a, B3I
    EXPECT_FALSE(c.band[SBF_SIG_GPS_L1CA].join);
    EXPECT_FALSE(c.band[SBF_SIG_GAL_E1].join);
    EXPECT_FALSE(c.band[SBF_SIG_BDS_B1I].join);
    EXPECT_EQ(c.windup, MosaicMbWindup::REF);
    EXPECT_EQ(c.w_src, MosaicMbWSrc::EST);
    EXPECT_TRUE(c.w_check);
    EXPECT_TRUE(c.guard);
    EXPECT_TRUE(c.common);
    EXPECT_TRUE(c.anchor_vote);
    EXPECT_FALSE(c.express_sigma);
}

// The gyro's wind-up as the driver reads it: W = -k p / 2pi from a fresh sample under the rail, none otherwise.
TEST(MosaicMultiband, TheGyrosWindupIsFreshAndUnderTheRailOrNone)
{
    const double two_pi = 2.0 * 3.14159265358979323846;
    MosaicMbWindupIn w = mbWindupFromGyro(true, two_pi, 5000, 0.99);
    EXPECT_TRUE(w.ok);
    EXPECT_NEAR(w.hz, -0.99, 1e-6);                       // one turn a second, 0.99 cycles a turn
    w = mbWindupFromGyro(true, -0.5 * two_pi, 0, 0.99);
    EXPECT_TRUE(w.ok);
    EXPECT_NEAR(w.hz, 0.495, 1e-6);
    EXPECT_TRUE(mbWindupFromGyro(true, 1.0, 20000, 0.99).ok);    // at the age bound
    EXPECT_FALSE(mbWindupFromGyro(true, 1.0, 20001, 0.99).ok);   // past it
    EXPECT_FALSE(mbWindupFromGyro(true, 1.0, -1, 0.99).ok);      // no age
    EXPECT_FALSE(mbWindupFromGyro(false, 1.0, 0, 0.99).ok);      // no sample yet
    const double rail = 4000.0 * 3.14159265358979323846 / 180.0;
    EXPECT_FALSE(mbWindupFromGyro(true, rail, 0, 0.99).ok);      // at the gyro's rail
    EXPECT_TRUE(mbWindupFromGyro(true, 0.99 * rail, 0, 0.99).ok);
    EXPECT_FLOAT_EQ(mbWindupFromGyro(true, rail, 0, 0.99).hz, 0.0f);
}

TEST(MosaicMultibandPrivateCapture, AnchorsAreTheAdaptersAndTheBandsCombine)
{
    const auto paths = sbftest::privateCapturePaths();
    if (paths.empty()) GTEST_SKIP() << "TR_SBF_PRIVATE_CAPTURE not set";
    for (const std::string& path : paths)
    {
        SCOPED_TRACE(sbftest::captureName(path));
        Mb on, empty;
        on.cfg = defaults();   // the estimate, the smoothing and the guard as the harness runs them
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
