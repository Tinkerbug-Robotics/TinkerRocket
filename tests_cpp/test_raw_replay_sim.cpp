// Host tests for a sim's epochs through the GNSS task's stage
// (tools/raw_replay/sim_epoch.h; registered in v9raw/sim.cmake): raw_replay's
// 'X' events, Grid B's input. A sim hands its satellites already placed, so
// the stage's ephemerides and builder cannot place them; simOffer runs the
// stage's own see(), lock ages, cap and ionosphere choice on stand-in
// ephemerides, and mirrors the builder's floors, mask and noise model. The
// check that matters: on the same sky, the sim path picks the satellites the
// stage's own offer() builds, with the same sigmas and lock ages, epoch after
// epoch -- the cap's hold, a relock after a gap, the C/N0 floor.
//
// The skies are synthetic: Keplerian MEO shells over a receiver on the
// equator at the prime meridian, no real place.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <map>
#include <utility>
#include <vector>

// sim_epoch.h runs the stage's private parts (capEpoch, capBuilt, its counts), as raw_replay.cpp does
#define private public
#define protected public
#include "RawGnssMeasStage.h"
#undef private
#undef protected

#include "ReceiverProfileApply.h"
#include "gnss/atmo.h"
#include "gnss/types.h"
#include "sim_epoch.h"

using gnss_raw::BuiltEpoch;
using gnss_raw::RawGnssMeasStage;
using gnss_raw::RawObs;
using gnss_raw::TimedRawEpoch;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int64_t kT0 = 5000000000LL;   // MCU time of the first epoch, us
constexpr double kTow0 = 302400.0;      // midweek
constexpr uint16_t kWeek = 2440;
const double kRxLla[3] = {0.0, 0.0, 0.0};

// A satellite of a MEO shell (GPS's 6 planes, Galileo's 3, BeiDou's 3).
gps_eph_t orbit(int sys, int prn)
{
    gps_eph_t e;
    std::memset(&e, 0, sizeof(e));
    e.valid = 1;
    e.sys = sys;
    e.prn = prn;
    e.week = kWeek;
    e.iode = 7;
    e.toe = e.toc = kTow0 + (sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0);
    e.sqrt_a = sys == GNSS_SYS_GPS ? 5153.7 : (sys == GNSS_SYS_GAL ? 5440.6 : 5282.6);
    e.e = 0.005;
    e.i0 = (sys == GNSS_SYS_GAL ? 56.0 : 55.0) * kPi / 180.0;
    const int planes = sys == GNSS_SYS_GPS ? 6 : 3;
    e.omega0 = (prn % planes) * 2.0 * kPi / planes;
    e.m0 = (prn / planes) * 2.0 * kPi / (sys == GNSS_SYS_GPS ? 5.4 : 9.0) + (prn % planes) * 0.7;
    return e;
}

// The mosaic-G5's stage: its profile v0 (floors, noise model, relock gap) and the cap, as the shadow's
// stageConfig gives it.
RawGnssMeasStage::Config mosaicStage()
{
    RawGnssMeasStage::Config c;
    const gnss_raw::GnssReceiverProfile p = gnss_raw::mosaicG5Profile();
    gnss_raw::applyReceiverProfile(p, c.meas);
    c.sat_cap = p.sat_cap;
    return c;
}

gnss_raw::ShadowNav navHere()
{
    gnss_raw::ShadowNav nav{};
    for (int k = 0; k < 3; ++k)
        nav.lla[k] = kRxLla[k];
    nav.running = 1;
    return nav;
}

// One epoch of the sky at tow, both ways: the receiver's raw epoch (for the
// stage's own offer, its ephemerides filed) and the sim's (each satellite
// placed from the same orbit). Every satellite of the three shells above
// min_el_deg, at C/N0 cn0(sys, prn); skip(sys, prn) leaves one out.
struct Pair
{
    TimedRawEpoch raw;
    rr::SimEpoch sim;
};

template <class Cn0, class Skip>
Pair epochAt(double tow, Cn0 cn0, Skip skip, double min_el_deg = -5.0)
{
    double rr_[3];
    geo_to_ecef(kRxLla[0], kRxLla[1], kRxLla[2], rr_);
    Pair p;
    p.raw.epoch.tow_s = tow;
    p.raw.mcu_us = kT0 + static_cast<int64_t>(std::llround((tow - kTow0) * 1.0e6));
    p.raw.gps_week = kWeek;
    p.sim.tow_s = tow;
    p.sim.mcu_us = p.raw.mcu_us;
    p.sim.week = kWeek;
    const int sys_order[3] = {GNSS_SYS_GPS, GNSS_SYS_GAL, GNSS_SYS_BDS};
    for (int sys : sys_order)
    {
        const int prn0 = sys == GNSS_SYS_BDS ? 19 : 1;
        const int prn1 = sys == GNSS_SYS_GPS ? 32 : (sys == GNSS_SYS_GAL ? 36 : 46);
        for (int prn = prn0; prn <= prn1 && p.raw.epoch.n < gnss_raw::RawEpoch::kMaxObs; ++prn)
        {
            if (skip(sys, prn))
                continue;
            const gps_eph_t e = orbit(sys, prn);
            double rs[3], vs[3], clk;
            gps_sat_pos(&e, tow + (sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0), rs, vs, &clk);
            const double d[3] = {rs[0] - rr_[0], rs[1] - rr_[1], rs[2] - rr_[2]};
            double az, el;
            gnss_az_el(kRxLla[0], kRxLla[1], d, &az, &el);
            if (el < min_el_deg * kPi / 180.0)
                continue;
            RawObs& o = p.raw.epoch.obs[p.raw.epoch.n++];
            o = RawObs{};
            o.sys = static_cast<uint8_t>(sys);
            o.prn = static_cast<uint8_t>(prn);
            o.has_pr = o.has_rate = true;
            o.cn0_dbhz = cn0(sys, prn);
            o.pr_m = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            o.rate_mps = 100.0;
            o.lock_ms = 60000;
            rr::SimMeas& m = p.sim.meas[p.sim.n++];
            m.sys = o.sys;
            m.prn = o.prn;
            m.has_pr = m.has_rr = true;
            for (int k = 0; k < 3; ++k)
            {
                m.sat_pos[k] = rs[k];
                m.sat_vel[k] = vs[k];
            }
            m.pr = o.pr_m;
            m.rr = o.rate_mps;
            m.cn0_dbhz = o.cn0_dbhz;
        }
    }
    return p;
}

double elevationDeg(int sys, int prn, double tow = kTow0)
{
    double rr_[3], rs[3], clk, az, el;
    geo_to_ecef(kRxLla[0], kRxLla[1], kRxLla[2], rr_);
    const gps_eph_t e = orbit(sys, prn);
    gps_sat_pos(&e, tow + (sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0), rs, nullptr, &clk);
    const double d[3] = {rs[0] - rr_[0], rs[1] - rr_[1], rs[2] - rr_[2]};
    gnss_az_el(kRxLla[0], kRxLla[1], d, &az, &el);
    return el * 180.0 / kPi;
}

void fileOrbits(RawGnssMeasStage& st, const Pair& p)
{
    for (int i = 0; i < p.raw.epoch.n; ++i)
        st.addEphemeris(orbit(p.raw.epoch.obs[i].sys, p.raw.epoch.obs[i].prn));
}

std::map<std::pair<int, int>, int> index(const BuiltEpoch& b)
{
    std::map<std::pair<int, int>, int> m;
    for (int i = 0; i < b.n; ++i)
        m[{b.meas[i].sys, b.meas[i].prn}] = i;
    return m;
}

// The two builds of an epoch agree: the same satellites, sigmas and lock ages.
void expectSameBuild(const BuiltEpoch& a, const BuiltEpoch& b, int epoch)
{
    ASSERT_EQ(a.n, b.n) << "epoch " << epoch;
    const auto ia = index(a), ib = index(b);
    for (const auto& kv : ia)
    {
        auto it = ib.find(kv.first);
        ASSERT_NE(it, ib.end()) << "epoch " << epoch << " sys " << kv.first.first << " prn " << kv.first.second;
        const gnss_raw::RawMeas& x = a.meas[kv.second];
        const gnss_raw::RawMeas& y = b.meas[it->second];
        EXPECT_EQ(x.sigma_pr, y.sigma_pr);
        EXPECT_EQ(x.sigma_pr_corr, y.sigma_pr_corr);
        EXPECT_EQ(x.tau_pr, y.tau_pr);
        EXPECT_EQ(x.sigma_rr, y.sigma_rr);
        EXPECT_EQ(x.tau_rr, y.tau_rr);
        EXPECT_EQ(x.has_rr, y.has_rr);
        EXPECT_FALSE(y.has_cr);
        EXPECT_EQ(a.lock_age_s[kv.second], b.lock_age_s[it->second]);
    }
    EXPECT_EQ(a.mcu_us, b.mcu_us);
    EXPECT_EQ(a.tow_s, b.tow_s);
    EXPECT_EQ(a.gps_week, b.gps_week);
    EXPECT_EQ(a.use_broadcast, b.use_broadcast);
    EXPECT_EQ(a.iono_changed, b.iono_changed);
}

}  // namespace

// On a sky of 30-odd satellites at 10 Hz for 3 s -- C/N0 shuffled every
// second (the cap's 3 dB hold at work), one Galileo satellite gone for 0.7 s
// (a relock past the profile's 0.5 s gap), one BeiDou satellite under the
// mosaic's 25 dB-Hz floor -- the sim path builds, epoch by epoch, what the
// stage's own offer builds from the receiver's raw epoch and ephemerides:
// the same 16 satellites, sigmas, lock ages and ionosphere choice, and the
// same counts.
TEST(RawReplaySim, TheSimPathBuildsWhatTheStagesOfferBuilds)
{
    RawGnssMeasStage real(mosaicStage()), sim(mosaicStage());
    gnss_raw::ShadowIono io{};
    io.gps_day = static_cast<uint16_t>(kWeek * 7 + 3);
    real.setIono(io);
    sim.setIono(io);
    rr::SimCounts c;
    int built = 0;
    for (int k = 0; k < 30; ++k)
    {
        const double tow = kTow0 + 0.1 * k;
        const int second = k / 10;
        auto cn0 = [&](int sys, int prn) {
            if (sys == GNSS_SYS_BDS && prn == 23)
                return 24.0f;                                    // under the floor
            return 35.0f + static_cast<float>((prn * 7 + sys * 5 + second * 3) % 13);
        };
        auto skip = [&](int sys, int prn) { return sys == GNSS_SYS_GAL && prn == 5 && k >= 10 && k < 17; };
        Pair p = epochAt(tow, cn0, skip);
        if (k == 0)
            fileOrbits(real, p);
        ASSERT_GT(p.raw.epoch.n, 20);
        BuiltEpoch a, b;
        ASSERT_TRUE(real.offer(p.raw, navHere(), a));
        ASSERT_TRUE(rr::simOffer(sim, p.sim, navHere(), b, c));
        expectSameBuild(a, b, k);
        // the cap's 16, less a pick under the mask (the cap cannot tell; picked last from then on)
        EXPECT_LE(b.n, 16);
        EXPECT_GE(b.n, 15);
        built++;
    }
    EXPECT_EQ(real.stats().capped_out, sim.stats().capped_out);
    EXPECT_EQ(real.stats().built, sim.stats().built);
    EXPECT_EQ(real.stats().tracked, sim.stats().tracked);
    EXPECT_EQ(sim.stats().tracked_no_eph, 0);
    EXPECT_EQ(c.built, static_cast<uint32_t>(built));
    EXPECT_GT(c.eph_standins, 20u);
}

// The relock: a satellite missing longer than the profile's relock_gap_s
// comes back with its lock age 0 -- its code carrying the relock sigma, its
// Doppler the relock term -- and one back within the gap keeps its age.
TEST(RawReplaySim, ASatelliteGoneLongerThanTheGapComesBackRelocked)
{
    RawGnssMeasStage st(mosaicStage());
    st.cfg_.sat_cap = 0;
    rr::SimCounts c;
    auto cn0 = [](int, int) { return 45.0f; };
    // two GPS satellites well up
    int up[2] = {0, 0}, nup = 0;
    for (int prn = 1; prn <= 32 && nup < 2; ++prn)
        if (elevationDeg(GNSS_SYS_GPS, prn) > 20.0)
            up[nup++] = prn;
    ASSERT_EQ(nup, 2);
    BuiltEpoch b;
    double age_short = -1.0, age_long = -1.0;
    float sig_short = 0.0f, sig_long = 0.0f;
    for (int k = 0; k < 40; ++k)
    {
        auto skip = [&](int sys, int prn) {
            return (sys == GNSS_SYS_GPS && prn == up[0] && k >= 10 && k < 14) ||   // 0.4 s: within the gap
                   (sys == GNSS_SYS_GPS && prn == up[1] && k >= 10 && k < 17);     // 0.7 s: past it
        };
        Pair p = epochAt(kTow0 + 0.1 * k, cn0, skip);
        ASSERT_TRUE(rr::simOffer(st, p.sim, navHere(), b, c));
        if (k == 17)
            for (int i = 0; i < b.n; ++i)
            {
                if (b.meas[i].sys == GNSS_SYS_GPS && b.meas[i].prn == up[0])
                {
                    age_short = b.lock_age_s[i];
                    sig_short = b.meas[i].sigma_pr_corr;
                }
                if (b.meas[i].sys == GNSS_SYS_GPS && b.meas[i].prn == up[1])
                {
                    age_long = b.lock_age_s[i];
                    sig_long = b.meas[i].sigma_pr_corr;
                }
            }
    }
    EXPECT_NEAR(age_short, 1.7, 1e-6);
    EXPECT_NEAR(age_long, 0.0, 1e-6);
    EXPECT_GT(sig_long, 39.0f);           // the profile's 40 m relock sigma, fresh
    EXPECT_LT(sig_short, sig_long);
}

// The builder's floors and mask: a satellite under the code's C/N0 floor or
// the elevation mask is not built and is counted; a Doppler's extra sigma
// (a stale satellite's) is added in quadrature to the builder's.
TEST(RawReplaySim, TheFloorTheMaskAndAnExtraDopplerSigma)
{
    RawGnssMeasStage st(mosaicStage());
    st.cfg_.sat_cap = 0;
    rr::SimCounts c;
    // the sky down to 8 deg under the horizon: every one under the 3 deg mask counted, none built
    int weak_prn = 0, n_low = 0;
    for (int prn = 1; prn <= 36; ++prn)
        if (weak_prn == 0 && elevationDeg(GNSS_SYS_GAL, prn) > 10.0)
            weak_prn = prn;
    ASSERT_NE(weak_prn, 0);
    auto cn0 = [&](int sys, int prn) { return sys == GNSS_SYS_GAL && prn == weak_prn ? 20.0f : 44.0f; };
    Pair p = epochAt(kTow0, cn0, [](int, int) { return false; }, -8.0);
    ASSERT_LT(p.sim.n, gnss_raw::RawEpoch::kMaxObs);
    for (int i = 0; i < p.sim.n; ++i)
        n_low += elevationDeg(p.sim.meas[i].sys, p.sim.meas[i].prn) < 3.0 ? 1 : 0;
    ASSERT_GT(n_low, 0);
    int stale = -1;
    for (int i = 0; i < p.sim.n; ++i)
        if (p.sim.meas[i].sys == GNSS_SYS_BDS && stale < 0 &&
            elevationDeg(p.sim.meas[i].sys, p.sim.meas[i].prn) > 10.0)
        {
            stale = i;
            p.sim.meas[i].rr_sig_add = 4.0f;
        }
    ASSERT_GE(stale, 0);
    BuiltEpoch b;
    ASSERT_TRUE(rr::simOffer(st, p.sim, navHere(), b, c));
    bool low_built = false, weak_built = false;
    float s_stale = 0.0f, s_other = 0.0f;
    for (int i = 0; i < b.n; ++i)
    {
        low_built |= elevationDeg(b.meas[i].sys, b.meas[i].prn) < 3.0;
        weak_built |= b.meas[i].sys == GNSS_SYS_GAL && b.meas[i].prn == weak_prn;
        if (b.meas[i].sys == p.sim.meas[stale].sys && b.meas[i].prn == p.sim.meas[stale].prn)
            s_stale = b.meas[i].sigma_rr;
        else if (b.meas[i].sys == GNSS_SYS_BDS)
            s_other = b.meas[i].sigma_rr;
    }
    EXPECT_FALSE(low_built);
    EXPECT_FALSE(weak_built);
    EXPECT_EQ(c.below_mask, static_cast<uint32_t>(n_low));
    EXPECT_EQ(c.below_cn0, 1u);
    ASSERT_GT(s_other, 0.0f);
    EXPECT_NEAR(s_stale, std::hypot(s_other, 4.0f), 1e-5);
}

// An epoch with no MCU time is seen, not built (untimed); one for a filter
// that is not running is seen, not built (idle); both keep the lock ages.
TEST(RawReplaySim, UntimedAndIdleEpochsAreSeenNotBuilt)
{
    RawGnssMeasStage st(mosaicStage());
    rr::SimCounts c;
    auto cn0 = [](int, int) { return 44.0f; };
    Pair p = epochAt(kTow0, cn0, [](int, int) { return false; });
    BuiltEpoch b;
    rr::SimEpoch untimed = p.sim;
    untimed.mcu_us = 0;
    EXPECT_FALSE(rr::simOffer(st, untimed, navHere(), b, c));
    gnss_raw::ShadowNav off = navHere();
    off.running = 0;
    EXPECT_FALSE(rr::simOffer(st, p.sim, off, b, c));
    EXPECT_EQ(st.stats().untimed, 1u);
    EXPECT_EQ(st.stats().idle, 1u);
    EXPECT_EQ(st.stats().epochs, 2u);
    EXPECT_EQ(st.stats().built, 0u);
    EXPECT_EQ(c.built, 0u);
    EXPECT_GT(st.stats().tracked, 20);
}
