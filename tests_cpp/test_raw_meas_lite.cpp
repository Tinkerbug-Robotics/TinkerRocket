// Host tests for the raw-GNSS builder's cheaper build (RawMeasBuilder.h,
// Config::lite; the receiver profile's build_lite): the Beetle's two MSM7
// epochs on the COCOM rig (fixtures/raw_meas_golden.h) stepped at 10 Hz,
// built with and without it. Off, beginWith/nextWith are begin/next. On, the
// epoch the atmosphere is worked out on gives the full build's corrections,
// the period's other epochs keep them (the Doppler and the satellites as the
// full build's), each satellite's 5 s orbit solve falls on its own epoch, the
// noise is the full model's, and the GNSS task's stage builds the same picks.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "EphemerisStore.h"
#include "Lc86Parser.h"
#include "Lc86RawEpoch.h"
#include "Rtcm3Msm.h"
#include "RtcmEphemeris.h"
#include "fixtures/raw_meas_golden.h"
#include "gnss/atmo.h"
#include "gnss/types.h"

// the builder's orbit cache, read to count its solves
#define private public
#include "RawMeasBuilder.h"
#include "RawGnssMeasStage.h"
#undef private

using gnss_raw::EphemerisStore;
using gnss_raw::RawEpoch;
using gnss_raw::RawMeas;
using gnss_raw::RawMeasBuilder;

namespace
{

constexpr int kNearWeek = 2440;

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

std::vector<RawEpoch> parseEpochs()
{
    lc86::Lc86Parser parser;
    std::vector<RawEpoch> out;
    for (int i = 0; i < kGoldenRmMsmN; ++i)
        for (uint8_t b : frame(kGoldenRmMsm[i]))
            if (parser.feed(b) == lc86::Event::MSM_EPOCH)
            {
                rtcm3::MsmEpoch me;
                EXPECT_TRUE(parser.takeMsmEpoch(me));
                RawEpoch ep;
                EXPECT_TRUE(lc86::rawEpochFromMsm(me, ep));
                out.push_back(ep);
            }
    return out;
}

EphemerisStore storeAll()
{
    EphemerisStore s;
    for (int i = 0; i < kGoldenRmEphN; ++i)
    {
        gps_eph_t e;
        EXPECT_TRUE(gnss_raw::decodeEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len),
                                              kNearWeek, e));
        s.add(e);
    }
    return s;
}

gps_iono_t goldenIono()
{
    gps_iono_t io{};
    io.valid = 1;
    for (int k = 0; k < 4; ++k)
    {
        io.alpha[k] = kGoldenRmIon[k];
        io.beta[k] = kGoldenRmIon[4 + k];
    }
    return io;
}

// The second golden epoch k tenths of a second on (the receiver's code and
// Doppler as they were: only the geometry and the times move).
RawEpoch epochAt(const RawEpoch& base, int k)
{
    RawEpoch e = base;
    e.tow_s = base.tow_s + 0.1 * k;
    return e;
}

// The receiver k tenths of a second on, climbing at vz m/s along the radius.
void receiverAt(int k, double vz, double r[3], double lla[3])
{
    const double R = std::sqrt(kGoldenRmRx[0] * kGoldenRmRx[0] + kGoldenRmRx[1] * kGoldenRmRx[1] +
                               kGoldenRmRx[2] * kGoldenRmRx[2]);
    for (int j = 0; j < 3; ++j)
        r[j] = kGoldenRmRx[j] * (1.0 + vz * 0.1 * k / R);
    ecef_to_geo(r, &lla[0], &lla[1], &lla[2]);
}

// The mosaic's build: no carrier, its noise model, the atmosphere once a second.
RawMeasBuilder::Config mosaicLike(bool lite)
{
    RawMeasBuilder::Config c;
    c.carrier = false;
    c.pr_white_a = 0.10;
    c.pr_white_b = 0.30;
    c.rr_white_a = 0.01;
    c.rr_white_b = 0.075;
    c.rr_scale = 1.5;
    c.relock_gap_s = 0.5;
    c.epoch_period_s = 0.1;
    c.lite = lite;
    c.atmo_period_s = lite ? 1.0 : 0.0;
    return c;
}

int build(RawMeasBuilder& b, const RawEpoch& ep, const EphemerisStore& store, const double r[3], const double lla[3],
          RawMeas* out)
{
    std::memset(out, 0, sizeof(RawMeas) * RawEpoch::kMaxObs);
    b.beginWith(ep, r, lla, 0.0, false);
    return b.nextWith(ep, store, nullptr, out, 0, RawEpoch::kMaxObs, RawEpoch::kMaxObs);
}

void expectSameBut(const RawMeas& a, const RawMeas& b, bool pr_too, bool sigmas_exact)
{
    EXPECT_EQ(a.sys, b.sys);
    EXPECT_EQ(a.prn, b.prn);
    EXPECT_EQ(a.has_pr, b.has_pr);
    EXPECT_EQ(a.has_rr, b.has_rr);
    for (int k = 0; k < 3; ++k)
    {
        EXPECT_EQ(a.sat_pos[k], b.sat_pos[k]);
        EXPECT_EQ(a.sat_vel[k], b.sat_vel[k]);
        EXPECT_EQ(a.sat_acc[k], b.sat_acc[k]);
    }
    if (pr_too)
        EXPECT_EQ(a.pr, b.pr);
    EXPECT_EQ(a.rr, b.rr);
    if (sigmas_exact)
    {
        EXPECT_EQ(a.sigma_pr, b.sigma_pr);
        EXPECT_EQ(a.sigma_pr_corr, b.sigma_pr_corr);
        EXPECT_EQ(a.sigma_rr, b.sigma_rr);
    }
    else
    {
        EXPECT_FLOAT_EQ(a.sigma_pr, b.sigma_pr);
        EXPECT_FLOAT_EQ(a.sigma_pr_corr, b.sigma_pr_corr);
        EXPECT_FLOAT_EQ(a.sigma_rr, b.sigma_rr);
    }
    EXPECT_EQ(a.tau_pr, b.tau_pr);
    EXPECT_EQ(a.tau_rr, b.tau_rr);
    EXPECT_EQ(a.has_cr, b.has_cr);
    if (pr_too)
        EXPECT_EQ(a.cr_dphi_m, b.cr_dphi_m);
    else
        EXPECT_NEAR(a.cr_dphi_m, b.cr_dphi_m, 1e-6);
}

}  // namespace

TEST(RawMeasLite, OffBeginWithAndNextWithAreBeginAndNext)
{
    const std::vector<RawEpoch> eps = parseEpochs();
    ASSERT_EQ(eps.size(), 2u);
    const EphemerisStore store = storeAll();
    for (bool carrier : {true, false})
    {
        RawMeasBuilder::Config c;             // the LC86G's, lite off
        c.carrier = carrier;
        RawMeasBuilder a(c), b(c);
        a.setIono(goldenIono());
        b.setIono(goldenIono());
        RawMeas ma[RawEpoch::kMaxObs], mb[RawEpoch::kMaxObs];
        for (int k = 0; k < 30; ++k)
        {
            const RawEpoch ep = epochAt(eps[1], k);
            double r[3], lla[3];
            receiverAt(k, 300.0, r, lla);
            std::memset(ma, 0, sizeof(ma));
            const int na = a.build(ep, store, r, 2.0, ma, RawEpoch::kMaxObs);
            std::memset(mb, 0, sizeof(mb));
            b.beginWith(ep, r, lla, 2.0, true);   // lla and noted ignored with lite off
            const int nb = b.nextWith(ep, store, nullptr, mb, 0, RawEpoch::kMaxObs, RawEpoch::kMaxObs);
            ASSERT_EQ(na, nb);
            for (int i = 0; i < na; ++i)
                expectSameBut(ma[i], mb[i], true, true);
        }
    }
}

TEST(RawMeasLite, TheSigmasAreTheFullModels)
{
    // the full build's noise (RawMeasBuilder::next), on the mosaic's model and the LC86G's
    for (const RawMeasBuilder::Config& c : {mosaicLike(true), RawMeasBuilder::Config()})
        for (double cn0 = 15.0; cn0 <= 56.0; cn0 += 0.25)
            for (double age : {0.0, 0.3, 2.0, 9.9, 60.0, 399.0, 401.0, 3600.0})
                for (double g : {0.0, 0.5, 3.0})
                {
                    auto sig = [](double a, double b, double x) {
                        return std::sqrt(a * a + b * b * std::pow(10.0, -(x - 35.0) / 10.0));
                    };
                    const double s_rr = std::hypot(sig(c.rr_white_a, c.rr_white_b, cn0), 0.0);
                    double s_corr = sig(c.pr_corr_a, c.pr_corr_b, cn0) * c.pr_corr_scale;
                    const double fresh = std::exp(-age / c.relock_tau_s);
                    s_corr = std::hypot(s_corr, c.relock_sigma_m * fresh);
                    const double s_dyn = c.rr_dyn_sigma * g, s_rel = c.rr_relock_sigma * fresh;
                    const double s_rr2 = s_rr * c.rr_scale;
                    float pr, corr, rr;
                    RawMeasBuilder::liteSigmas(c, cn0, age, GNSS_FREQ_L1_HZ, g, pr, corr, rr);
                    SCOPED_TRACE(testing::Message() << "cn0 " << cn0 << " age " << age << " g " << g);
                    EXPECT_FLOAT_EQ(pr, static_cast<float>(sig(c.pr_white_a, c.pr_white_b, cn0)));
                    EXPECT_FLOAT_EQ(corr, static_cast<float>(s_corr));
                    EXPECT_FLOAT_EQ(rr, static_cast<float>(std::sqrt(s_rr2 * s_rr2 + s_dyn * s_dyn + s_rel * s_rel)));
                }
}

TEST(RawMeasLite, TheAtmosphereEpochIsTheFullBuilds)
{
    // each second's first epoch: the full build's code to the bit, given the
    // same geodetic position (here the full build's own, from ECEF)
    const std::vector<RawEpoch> eps = parseEpochs();
    const EphemerisStore store = storeAll();
    RawMeasBuilder full(mosaicLike(false)), lite(mosaicLike(true));
    full.setIono(goldenIono());
    lite.setIono(goldenIono());
    RawMeas mf[RawEpoch::kMaxObs], ml[RawEpoch::kMaxObs];
    for (int k = 0; k < 30; ++k)
    {
        const RawEpoch ep = epochAt(eps[1], k);
        double r[3], lla[3];
        receiverAt(k, 0.0, r, lla);
        std::memset(mf, 0, sizeof(mf));
        const int nf = full.build(ep, store, r, 0.0, mf, RawEpoch::kMaxObs);
        const int nl = build(lite, ep, store, r, lla, ml);
        ASSERT_EQ(nf, nl);
        ASSERT_GT(nf, 8);
        if (k % 10 != 0)
            continue;
        SCOPED_TRACE(testing::Message() << "epoch " << k);
        for (int i = 0; i < nf; ++i)
        {
            // the orbit: both solved at the first epoch, and the 5 s phases
            // differ after it -- the satellite within millimetres
            if (k == 0)
                expectSameBut(mf[i], ml[i], false, false);   // the code: its ionosphere to parts in 10^16
            for (int j = 0; j < 3; ++j)
                EXPECT_NEAR(mf[i].sat_pos[j], ml[i].sat_pos[j], 0.01);
            EXPECT_NEAR(mf[i].pr, ml[i].pr, k == 0 ? 1e-6 : 0.01);
        }
    }
}

TEST(RawMeasLite, ThePeriodsOtherEpochsKeepItsAtmosphere)
{
    // A receiver climbing at 300 m/s through three seconds of 10 Hz epochs,
    // the orbits solved at every epoch in both builds: the Doppler, the
    // satellites and the noise are the full build's on every epoch; the code
    // the full build's on each second's first epoch, and on the others off
    // it only by what the satellites' own motion changes in a second (the
    // troposphere is the climbing receiver's: its mapping is kept)
    const std::vector<RawEpoch> eps = parseEpochs();
    const EphemerisStore store = storeAll();
    RawMeasBuilder::Config cf = mosaicLike(false), cl = mosaicLike(true);
    cf.sat_refresh_s = cl.sat_refresh_s = 0.0;
    RawMeasBuilder full(cf), lite(cl);
    full.setIono(goldenIono());
    lite.setIono(goldenIono());
    RawMeas mf[RawEpoch::kMaxObs], ml[RawEpoch::kMaxObs];
    double worst = 0.0;
    for (int k = 0; k < 30; ++k)
    {
        const RawEpoch ep = epochAt(eps[1], k);
        double r[3], lla[3];
        receiverAt(k, 300.0, r, lla);
        std::memset(mf, 0, sizeof(mf));
        const int nf = full.build(ep, store, r, 0.0, mf, RawEpoch::kMaxObs);
        const int nl = build(lite, ep, store, r, lla, ml);
        ASSERT_EQ(nf, nl);
        SCOPED_TRACE(testing::Message() << "epoch " << k);
        for (int i = 0; i < nf; ++i)
        {
            expectSameBut(mf[i], ml[i], false, false);
            const double d = std::fabs(ml[i].pr - mf[i].pr);
            if (k % 10 == 0)
                EXPECT_LT(d, 1e-6);   // each second's first epoch: the full build's code
            worst = d > worst ? d : worst;
            EXPECT_LT(d, 0.05);
        }
    }
    EXPECT_GT(worst, 0.0);                // it does keep it
    std::printf("the code's worst difference off a second's first epoch, climbing at 300 m/s: %.3f m\n", worst);
}

TEST(RawMeasLite, ANewIonosphereIsTakenAtOnce)
{
    const std::vector<RawEpoch> eps = parseEpochs();
    const EphemerisStore store = storeAll();
    RawMeasBuilder::Config cf = mosaicLike(false), cl = mosaicLike(true);
    cf.sat_refresh_s = cl.sat_refresh_s = 0.0;
    RawMeasBuilder full(cf), lite(cl);
    RawMeas mf[RawEpoch::kMaxObs], ml[RawEpoch::kMaxObs];
    double r[3], lla[3];
    receiverAt(0, 0.0, r, lla);
    for (int k = 0; k < 5; ++k)
    {
        if (k == 3)
        {
            full.setIono(goldenIono());       // mid-period: the pad's choice
            lite.setIono(goldenIono());
        }
        const RawEpoch ep = epochAt(eps[1], k);
        std::memset(mf, 0, sizeof(mf));
        const int nf = full.build(ep, store, r, 0.0, mf, RawEpoch::kMaxObs);
        ASSERT_EQ(build(lite, ep, store, r, lla, ml), nf);
        if (k == 3)
            for (int i = 0; i < nf; ++i)
                EXPECT_NEAR(ml[i].pr, mf[i].pr, 1e-6);
    }
}

TEST(RawMeasLite, ACarrierPhaseTakesItsAtmosphereEveryEpoch)
{
    // the LC86G's carrier on: a satellite with a phase is worked out every
    // epoch, so its carrier rates are the full build's
    const std::vector<RawEpoch> eps = parseEpochs();
    const EphemerisStore store = storeAll();
    RawMeasBuilder::Config cf, cl;
    cf.sat_refresh_s = cl.sat_refresh_s = 0.0;
    cf.epoch_period_s = cl.epoch_period_s = 0.1;
    cl.lite = true;
    cl.atmo_period_s = 1.0;
    RawMeasBuilder full(cf), lite(cl);
    full.setIono(goldenIono());
    lite.setIono(goldenIono());
    RawMeas mf[RawEpoch::kMaxObs], ml[RawEpoch::kMaxObs];
    int rates = 0;
    for (int k = 0; k < 12; ++k)
    {
        const RawEpoch ep = epochAt(eps[1], k);
        double r[3], lla[3];
        receiverAt(k, 300.0, r, lla);
        std::memset(mf, 0, sizeof(mf));
        const int nf = full.build(ep, store, r, 0.0, mf, RawEpoch::kMaxObs);
        ASSERT_EQ(build(lite, ep, store, r, lla, ml), nf);
        for (int i = 0; i < nf; ++i)
        {
            SCOPED_TRACE(testing::Message() << "epoch " << k << " meas " << i);
            bool phase = false;
            for (int j = 0; j < ep.n; ++j)
                if (ep.obs[j].sys == mf[i].sys && ep.obs[j].prn == mf[i].prn)
                    phase = ep.obs[j].has_phase;
            expectSameBut(mf[i], ml[i], false, false);
            if (phase || k % 10 == 0)
                EXPECT_NEAR(mf[i].pr, ml[i].pr, 1e-6);
            EXPECT_NEAR(mf[i].cr_dphi_m, ml[i].cr_dphi_m, 1e-6);
            EXPECT_EQ(mf[i].sigma_cr, ml[i].sigma_cr);
            EXPECT_EQ(mf[i].cr_dt, ml[i].cr_dt);
            rates += ml[i].has_cr ? 1 : 0;
        }
    }
    EXPECT_GT(rates, 0);
}

TEST(RawMeasLite, EachSatellitesOrbitIsSolvedOnItsOwnEpoch)
{
    // 20 s of 10 Hz epochs: the full build solves every satellite again on
    // one epoch each 5 s; the cheaper build, after the first epoch's (a cold
    // start's), at most two on any epoch, and each still at least every 5 s
    const std::vector<RawEpoch> eps = parseEpochs();
    const EphemerisStore store = storeAll();
    int worst[2] = {0, 0};
    double oldest[2] = {0.0, 0.0};
    for (int lite = 0; lite < 2; ++lite)
    {
        RawMeasBuilder b(mosaicLike(lite != 0));
        b.setIono(goldenIono());
        RawMeas m[RawEpoch::kMaxObs];
        double t_ref[RawMeasBuilder::kSatCache];
        for (int k = 0; k < 200; ++k)
        {
            for (int s = 0; s < RawMeasBuilder::kSatCache; ++s)
                t_ref[s] = b.cache_[s].valid ? b.cache_[s].t_ref : -1.0;
            const RawEpoch ep = epochAt(eps[1], k);
            double r[3], lla[3];
            receiverAt(0, 0.0, r, lla);
            const int n = build(b, ep, store, r, lla, m);
            ASSERT_GT(n, 8);
            int solved = 0;
            for (int s = 0; s < RawMeasBuilder::kSatCache; ++s)
            {
                if (!b.cache_[s].valid)
                    continue;
                solved += b.cache_[s].t_ref != t_ref[s] ? 1 : 0;
                const double age = ep.tow_s - b.cache_[s].t_ref;
                oldest[lite] = age > oldest[lite] ? age : oldest[lite];
            }
            if (k > 0 && solved > worst[lite])
                worst[lite] = solved;
        }
    }
    std::printf("the most satellites solved on one epoch after the first: full %d, cheaper %d; oldest %.2f / %.2f s\n",
                worst[0], worst[1], oldest[0], oldest[1]);
    EXPECT_GE(worst[0], 5);               // the sky's, on two neighbouring epochs (their transmit times)
    EXPECT_LE(worst[1], 2);
    EXPECT_LT(oldest[1], 5.2);            // the age at its use (the transmit time ~0.07 s back)
}

TEST(RawMeasLite, TheStageBuildsTheSamePicks)
{
    // The GNSS task's stage, the cap on and off: the cheaper build builds the
    // same satellites from the same picks in the same order, at the same
    // counts, with the code within millimetres (each second's first epoch)
    const std::vector<RawEpoch> eps = parseEpochs();
    for (int cap : {0, 6})
    {
        gnss_raw::RawGnssMeasStage::Config cf, cl;
        cf.meas = mosaicLike(false);
        cl.meas = mosaicLike(true);
        cf.sat_cap = cl.sat_cap = cap;
        gnss_raw::RawGnssMeasStage full(cf), lite(cl);
        for (int i = 0; i < kGoldenRmEphN; ++i)
        {
            ASSERT_TRUE(full.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len)));
            ASSERT_TRUE(lite.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len)));
        }
        gnss_raw::ShadowNav nav = {};
        nav.running = 1;
        double r[3];
        receiverAt(0, 0.0, r, nav.lla);
        static gnss_raw::BuiltEpoch bf, bl;
        for (int k = 0; k < 40; ++k)
        {
            gnss_raw::TimedRawEpoch te;
            te.epoch = epochAt(eps[1], k);
            te.mcu_us = 1000000 + 100000 * k;
            te.gps_week = kNearWeek;
            // a satellite drops out for a while: the cap's picks move
            if (k >= 12 && k < 20 && te.epoch.n > 2)
                te.epoch.obs[1].has_pr = false;
            ASSERT_TRUE(full.offer(te, nav, bf));
            ASSERT_TRUE(lite.offer(te, nav, bl));
            SCOPED_TRACE(testing::Message() << "cap " << cap << " epoch " << k);
            ASSERT_EQ(bf.n, bl.n);
            ASSERT_GT(bf.n, cap > 0 ? cap - 1 : 8);
            for (int i = 0; i < bf.n; ++i)
            {
                EXPECT_EQ(bf.meas[i].sys, bl.meas[i].sys);
                EXPECT_EQ(bf.meas[i].prn, bl.meas[i].prn);
                EXPECT_EQ(bf.lock_age_s[i], bl.lock_age_s[i]);
                EXPECT_NEAR(bf.meas[i].rr, bl.meas[i].rr, 1e-4);   // the clock's rate, solved at other epochs
                if (k % 10 == 0)
                    EXPECT_NEAR(bf.meas[i].pr, bl.meas[i].pr, 0.005);
            }
            EXPECT_EQ(full.stats().tracked, lite.stats().tracked);
            EXPECT_EQ(full.stats().tracked_no_eph, lite.stats().tracked_no_eph);
            EXPECT_EQ(full.stats().capped_out, lite.stats().capped_out);
        }
    }
}
