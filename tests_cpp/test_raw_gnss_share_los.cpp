// Host tests for TR_RawGnssEKF's shared epoch geometry (RawGnssEKFParams
// share_los, rr_float): each satellite's line of sight taken once an epoch
// and shared by the epoch's units, the receiver's frame kept while the
// position stands, inject's radii kept, and the Doppler's prediction in
// float. On a synthetic sky of 16 satellites over a receiver climbing at
// 150 m/s: the units before any update give the very bits they give unshared;
// an epoch worked as the shadow works it (units between IMU steps) stays
// within micrometres of the unshared filter and uses and refuses the same
// measurements; the ranges follow the receiver to second order and are taken
// again past kLosRetakeM, for another epoch, or for a measurement that is not
// the one taken; the boost screen's copy takes the epoch's geometry along;
// both switches are off by default, in every profile, and the profile puts
// them into the filter.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <random>
#include <vector>

#include "RawMeas.h"
#include "ReceiverProfile.h"
#include "ReceiverProfileApply.h"
#include "SatJumpCheck.h"
#include "TR_RawGnssEKF.h"
#include "gnss/atmo.h"
#include "gnss/types.h"

using gnss_raw::DopplerChangeImu;
using gnss_raw::DopplerRef;
using gnss_raw::RawEpoch;
using gnss_raw::RawGnssEKF;
using gnss_raw::RawGnssEKFParams;
using gnss_raw::RawGnssStats;
using gnss_raw::RawMeas;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kC = 299792458.0;
constexpr double kOmegaE = 7.2921151467e-5;
constexpr double kG = 9.807;
constexpr double kStep = 0.01;     // the flight computer's 100 steps a second
constexpr int kSats = 16;

struct Sat
{
    int sys, prn;
    double pos[3], vel[3], acc[3];
};

void enuToEcef(double lat, double lon, const double enu[3], double out[3])
{
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    out[0] = -so * enu[0] - sl * co * enu[1] + cl * co * enu[2];
    out[1] = co * enu[0] - sl * so * enu[1] + cl * so * enu[2];
    out[2] = cl * enu[1] + sl * enu[2];
}

// a synthetic receiver, no real place: a fifth and a tenth of pi, 1 km up
const double kLla0[3] = {kPi / 5.0, kPi / 10.0, 1000.0};

// kSats satellites in GPS, Galileo and BeiDou, 10-80 deg up all around
std::vector<Sat> makeSky()
{
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    std::vector<Sat> sky;
    for (int i = 0; i < kSats; ++i)
    {
        const double az = (22.5 * i + 7.0) * kPi / 180.0, el = (10.0 + 70.0 * ((i * 7) % kSats) / kSats) * kPi / 180.0;
        const double enu[3] = {std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el)};
        double u[3];
        enuToEcef(kLla0[0], kLla0[1], enu, u);
        const double R = 26.56e6, ru = r[0] * u[0] + r[1] * u[1] + r[2] * u[2];
        const double rr = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
        const double rho = -ru + std::sqrt(ru * ru - rr + R * R);
        Sat s{i % 3, 2 + i, {}, {}, {}};
        for (int k = 0; k < 3; ++k)
            s.pos[k] = r[k] + rho * u[k];
        const double z[3] = {0.3, -0.2, 1.0};
        double v[3] = {s.pos[1] * z[2] - s.pos[2] * z[1], s.pos[2] * z[0] - s.pos[0] * z[2],
                       s.pos[0] * z[1] - s.pos[1] * z[0]};
        const double vn = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        for (int k = 0; k < 3; ++k)
        {
            s.vel[k] = 3870.0 * v[k] / vn;
            s.acc[k] = -3.986004418e14 * s.pos[k] / (R * R * R);
        }
        sky.push_back(s);
    }
    return sky;
}

void rangeAndRate(const double rs[3], const double vs[3], const double r[3], const double v[3], double& rho,
                  double& rate)
{
    rho = std::sqrt((rs[0] - r[0]) * (rs[0] - r[0]) + (rs[1] - r[1]) * (rs[1] - r[1]) + (rs[2] - r[2]) * (rs[2] - r[2]));
    double c = 1.0, s = 0.0, x[3] = {rs[0], rs[1], rs[2]};
    for (int it = 0; it < 2; ++it)
    {
        const double th = kOmegaE * rho / kC;
        c = std::cos(th);
        s = std::sin(th);
        x[0] = c * rs[0] + s * rs[1];
        x[1] = -s * rs[0] + c * rs[1];
        rho = std::sqrt((x[0] - r[0]) * (x[0] - r[0]) + (x[1] - r[1]) * (x[1] - r[1]) + (x[2] - r[2]) * (x[2] - r[2]));
    }
    const double vr[3] = {c * vs[0] + s * vs[1], -s * vs[0] + c * vs[1], vs[2]};
    rate = 0.0;
    for (int k = 0; k < 3; ++k)
        rate += (x[k] - r[k]) / rho * (vr[k] - v[k]);
}

// The sky moved on by t seconds (each satellite along its velocity), measured from r, v (ECEF) with noise.
std::vector<RawMeas> measure(const std::vector<Sat>& sky, double t, const double r[3], const double v[3],
                             std::mt19937& g)
{
    std::normal_distribution<double> n01(0.0, 1.0);
    std::vector<RawMeas> out;
    for (const Sat& s : sky)
    {
        RawMeas m{};
        m.sys = static_cast<uint8_t>(s.sys);
        m.prn = static_cast<uint8_t>(s.prn);
        for (int k = 0; k < 3; ++k)
        {
            m.sat_pos[k] = s.pos[k] + s.vel[k] * t;
            m.sat_vel[k] = s.vel[k];
            m.sat_acc[k] = s.acc[k];
        }
        double rho, rate;
        rangeAndRate(m.sat_pos, m.sat_vel, r, v, rho, rate);
        const double isb = s.sys == GNSS_SYS_GAL ? 12.0 : (s.sys == GNSS_SYS_BDS ? -25.0 : 0.0);
        m.has_pr = m.has_rr = true;
        m.pr = rho + 1234.5 - 2.5 * t + isb + 0.8 * n01(g);
        m.rr = rate - 2.5 + 0.05 * n01(g);
        m.sigma_pr = 1.0f;
        m.sigma_pr_corr = 0.5f;
        m.tau_pr = 15.0f;
        m.sigma_rr = 0.05f;
        m.tau_rr = 0.0f;
        out.push_back(m);
    }
    return out;
}

RawGnssEKFParams params(bool share, bool rrf = false)
{
    RawGnssEKFParams p;
    p.rr_lag_s = 0.0;               // the mosaic-G5's Doppler: on time
    p.cr_mode = gnss_raw::kCarrierOff;
    p.share_los = share;
    p.rr_float = rrf;
    return p;
}

// A level body climbing: 30 m/s^2 up from 150 m/s (the truth the sky is measured from, ECEF)
struct Truth
{
    double t = 0.0;
    double r[3], v[3];
    double up[3];
    Truth()
    {
        geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
        const double enu[3] = {0.0, 0.0, 1.0};
        enuToEcef(kLla0[0], kLla0[1], enu, up);
        for (int k = 0; k < 3; ++k)
            v[k] = 150.0 * up[k];
    }
    void step(double dt)
    {
        for (int k = 0; k < 3; ++k)
        {
            r[k] += v[k] * dt + 0.5 * 30.0 * up[k] * dt * dt;
            v[k] += 30.0 * up[k] * dt;
        }
        t += dt;
    }
};

template <typename T>
void imuStep(RawGnssEKF<T>& f)
{
    const T acc[3] = {T(0), T(0), T(-kG - 30.0)};   // level, FRD: up is -z
    const T gyro[3] = {T(0), T(0), T(0)};
    f.propagate(acc, gyro, T(kStep));
}

// A filter on the truth's track, its clock set and settled by n epochs worked whole.
RawGnssEKF<float> settled(bool share, Truth& tr, std::mt19937& g, const std::vector<Sat>& sky, int n = 20)
{
    RawGnssEKF<float> f(params(share));
    const float v0[3] = {0.0f, 0.0f, -150.0f};
    const float q0[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    double lla[3] = {kLla0[0] + 3.0 / 6.37e6, kLla0[1], kLla0[2] + 4.0};
    f.init(lla, v0, q0, 0.0);
    f.setQuaternion(q0);
    for (int e = 0; e < n; ++e)
    {
        for (int s = 0; s < 10; ++s)
        {
            imuStep(f);
            tr.step(kStep);
        }
        const auto m = measure(sky, tr.t, tr.r, tr.v, g);
        f.setFusePr(e % 10 == 0);
        f.updateGnssRaw(m.data(), kSats, 0.0);
    }
    return f;
}

double dist(const double a[3], const double b[3])
{
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

// One epoch as RawGnssShadow works it, a unit a step after the first three
// (a slice each): the change check, the satellite check, the clock, the
// screen, each satellite's update, the close. age: the epoch's age at the
// first unit; each step adds kStep to it. Returns the stats.
template <typename T>
RawGnssStats workEpoch(RawGnssEKF<T>& f, const RawMeas* m, double age, DopplerRef& ref, gnss_raw::SatJumpCheck& sc,
                       double tow, bool pr, bool sliced)
{
    std::vector<gnss_raw::dvc::Row> rows(RawEpoch::kMaxObs);
    DopplerChangeImu imu;
    DopplerRef next;
    RawGnssStats st;
    bool flagged[RawEpoch::kMaxObs];
    auto step = [&]() {
        if (!sliced)
            return;
        imuStep(f);
        age += kStep;
    };
    (void)f.dopplerChange(m, kSats, age, ref, next, rows.data(), imu);
    ref = next;
    double r[3], v[3];
    f.receiverEcefAt(age, r, v);
    sc.check(m, kSats, tow, r, flagged, f.sharedRanges(m, kSats, age, r));
    step();
    f.setFusePr(pr);
    if (!f.beginGnssRaw(m, kSats, age, st))
        return st;
    f.screenRangeRates(m, kSats, age, st);
    step();
    for (int i = 0; i < kSats; ++i)
    {
        f.updateGnssRawPart(m, i, i + 1, age, st);
        if (i % 4 == 3)
            step();
    }
    f.receiverEcefAt(age, r, v);
    sc.commit(m, kSats, tow, r, f.sharedRanges(m, kSats, age, r));
    return st;
}

}  // namespace

TEST(RawGnssShareLos, BothSwitchesAreOffByDefaultAndInEveryProfile)
{
    const RawGnssEKFParams p;
    EXPECT_FALSE(p.share_los);
    EXPECT_FALSE(p.rr_float);
    for (const auto& rx : {gnss_raw::lc86gProfile(), gnss_raw::mosaicG5Profile(), gnss_raw::genericProfile(),
                           gnss_raw::neoM8tProfile(), gnss_raw::px1105rProfile()})
    {
        EXPECT_FALSE(rx.share_los) << rx.name;
        EXPECT_FALSE(rx.rr_float) << rx.name;
    }
    // the profile puts them into the filter (the bench's TR_BENCH_RAW_SHARE_LOS sets the profile's)
    gnss_raw::GnssReceiverProfile rx = gnss_raw::mosaicG5Profile();
    rx.share_los = true;
    rx.rr_float = true;
    RawGnssEKFParams e;
    gnss_raw::applyReceiverProfile(rx, e);
    EXPECT_TRUE(e.share_los);
    EXPECT_TRUE(e.rr_float);
    // off, the filter has no shared ranges to give
    RawGnssEKF<float> f(params(false));
    const double r[3] = {1.0, 2.0, 3.0};
    RawMeas m{};
    EXPECT_EQ(f.sharedRanges(&m, 1, 0.0, r), nullptr);
}

TEST(RawGnssShareLos, TheUnitsBeforeAnyUpdateGiveTheSameBits)
{
    const auto sky = makeSky();
    Truth tr;
    std::mt19937 g(11);
    RawGnssEKF<float> off = settled(false, tr, g, sky);
    RawGnssEKF<float> on = off;
    on.setParams(params(true));
    DopplerRef ref_off, ref_on;
    // the change check's reference: the epoch before, each at its own
    std::vector<gnss_raw::dvc::Row> rows_off(RawEpoch::kMaxObs), rows_on(RawEpoch::kMaxObs);
    DopplerChangeImu imu_off, imu_on;
    {
        DopplerRef tmp;
        const auto m0 = measure(sky, tr.t, tr.r, tr.v, g);
        (void)off.dopplerChange(m0.data(), kSats, 0.0, tmp, ref_off, rows_off.data(), imu_off);
        ref_on = ref_off;
    }
    for (int s = 0; s < 10; ++s)
    {
        imuStep(off);
        imuStep(on);
        tr.step(kStep);
    }
    const auto m = measure(sky, tr.t, tr.r, tr.v, g);
    const double age = 0.03;
    DopplerRef out_off, out_on;
    const int n_off = off.dopplerChange(m.data(), kSats, age, ref_off, out_off, rows_off.data(), imu_off);
    const int n_on = on.dopplerChange(m.data(), kSats, age, ref_on, out_on, rows_on.data(), imu_on);
    ASSERT_EQ(n_off, kSats);
    ASSERT_EQ(n_on, n_off);
    for (int i = 0; i < kSats; ++i)
    {
        EXPECT_EQ(out_on.y[i], out_off.y[i]) << i;
        EXPECT_EQ(rows_on[i].z, rows_off[i].z) << i;
        for (int a = 0; a < 3; ++a)
            EXPECT_EQ(rows_on[i].u[a], rows_off[i].u[a]);
    }
    // the satellite check's receiver and ranges
    double r_off[3], v_off[3], r_on[3], v_on[3];
    off.receiverEcefAt(age, r_off, v_off);
    on.receiverEcefAt(age, r_on, v_on);
    for (int a = 0; a < 3; ++a)
    {
        EXPECT_EQ(r_on[a], r_off[a]);
        EXPECT_EQ(v_on[a], v_off[a]);
    }
    const double* rng = on.sharedRanges(m.data(), kSats, age, r_on);
    ASSERT_NE(rng, nullptr);
    for (int i = 0; i < kSats; ++i)
        EXPECT_EQ(rng[i], gnss_raw::sagnacRange(m[i], r_off)) << i;
    // the clock and the screen
    RawGnssStats st_off, st_on;
    ASSERT_TRUE(off.beginGnssRaw(m.data(), kSats, age, st_off));
    ASSERT_TRUE(on.beginGnssRaw(m.data(), kSats, age, st_on));
    EXPECT_EQ(on.clockBias(), off.clockBias());
    EXPECT_EQ(on.clockDrift(), off.clockDrift());
    off.screenRangeRates(m.data(), kSats, age, st_off);
    on.screenRangeRates(m.data(), kSats, age, st_on);
    EXPECT_EQ(st_on.screen_rr, st_off.screen_rr);
    EXPECT_EQ(st_on.screen_refused, st_off.screen_refused);
    EXPECT_EQ(st_on.screen_fit_n, st_off.screen_fit_n);
    EXPECT_EQ(st_on.screen_dv_down_mps, st_off.screen_dv_down_mps);
    // and the epoch's first satellite's update, before any other moved the receiver
    on.updateGnssRawPart(m.data(), 0, 1, age, st_on);
    off.updateGnssRawPart(m.data(), 0, 1, age, st_off);
    EXPECT_EQ(on.lla()[0], off.lla()[0]);
    EXPECT_EQ(on.lla()[2], off.lla()[2]);
    for (int a = 0; a < 3; ++a)
        EXPECT_EQ(on.velNed()[a], off.velNed()[a]);
    EXPECT_EQ(st_on.used_rr, st_off.used_rr);
}

TEST(RawGnssShareLos, AnEpochWorkedInSlicesStaysWithinMicrometresOfItsOwn)
{
    for (const bool sliced : {false, true})
    {
        const auto sky = makeSky();
        Truth tr;
        std::mt19937 g(5);
        RawGnssEKF<float> off = settled(false, tr, g, sky);
        RawGnssEKF<float> on = off;
        on.setParams(params(true));
        DopplerRef ref_off, ref_on;
        gnss_raw::SatJumpCheck sc_off, sc_on;
        double worst_r = 0.0, worst_v = 0.0;
        int used_off = 0, used_on = 0, rej_off = 0, rej_on = 0;
        for (int e = 0; e < 40; ++e)
        {
            for (int s = 0; s < 10; ++s)
                tr.step(kStep);
            const auto m = measure(sky, tr.t, tr.r, tr.v, g);
            // sliced, the epoch is worked from 3 steps after its time, a step between slices; whole, at its time
            const double age = sliced ? 0.03 : 0.0;
            while (off.time() < tr.t + age - 1.0e-6)
            {
                imuStep(off);
                imuStep(on);
            }
            const RawGnssStats so = workEpoch(off, m.data(), age, ref_off, sc_off, 100.0 + 0.1 * e, e % 10 == 0,
                                              sliced);
            const RawGnssStats sn = workEpoch(on, m.data(), age, ref_on, sc_on, 100.0 + 0.1 * e, e % 10 == 0,
                                              sliced);
            used_off += so.used_pr + so.used_rr;
            used_on += sn.used_pr + sn.used_rr;
            rej_off += so.rejected_pr + so.rejected_rr;
            rej_on += sn.rejected_pr + sn.rejected_rr;
            double ro[3], vo[3], rn[3], vn[3];
            off.receiverEcef(ro, vo);
            on.receiverEcef(rn, vn);
            worst_r = std::fmax(worst_r, dist(ro, rn));
            worst_v = std::fmax(worst_v, dist(vo, vn));
            EXPECT_EQ(sn.screen_rr, so.screen_rr);
        }
        EXPECT_EQ(used_on, used_off) << "sliced " << sliced;
        EXPECT_EQ(rej_on, rej_off) << "sliced " << sliced;
        EXPECT_GT(used_on, 40 * kSats);
        EXPECT_LT(worst_r, 1.0e-4) << "sliced " << sliced;   // m
        EXPECT_LT(worst_v, 1.0e-4) << "sliced " << sliced;   // m/s
        // and the satellite check judged the same
        EXPECT_EQ(sc_on.summary().judged, sc_off.summary().judged);
        EXPECT_NEAR(sc_on.summary().clock_m, sc_off.summary().clock_m, 1.0e-4);
    }
}

TEST(RawGnssShareLos, TheRangesFollowTheReceiverAndAreTakenAgainPastTheLimit)
{
    const auto sky = makeSky();
    Truth tr;
    std::mt19937 g(3);
    RawGnssEKF<float> f = settled(true, tr, g, sky, 5);
    imuStep(f);   // the next epoch, a step on
    tr.step(kStep);
    const auto m = measure(sky, tr.t, tr.r, tr.v, g);
    double r0[3], v0[3];
    f.receiverEcefAt(0.0, r0, v0);
    const double* rng = f.sharedRanges(m.data(), kSats, 0.0, r0);   // the epoch's first unit: taken at r0
    ASSERT_NE(rng, nullptr);
    for (int i = 0; i < kSats; ++i)
        EXPECT_EQ(rng[i], gnss_raw::sagnacRange(m[i], r0));
    // 3 m away (a correction's worth): to second order, and the Sagnac turn's 6.5e-6 of the range's change
    const double d3[3] = {1.7, -2.0, 1.4};
    double r1[3];
    for (int a = 0; a < 3; ++a)
        r1[a] = r0[a] + d3[a];
    rng = f.sharedRanges(m.data(), kSats, 0.0, r1);
    double worst = 0.0;
    for (int i = 0; i < kSats; ++i)
    {
        const double e = std::fabs(rng[i] - gnss_raw::sagnacRange(m[i], r1));
        worst = std::fmax(worst, e);
        EXPECT_NE(rng[i], gnss_raw::sagnacRange(m[i], r0));
    }
    EXPECT_LT(worst, 3.0e-5);   // m
    EXPECT_GT(worst, 0.0);      // shared, not taken again
    // 30 m away, past kLosRetakeM: taken again there, its own bits
    double r2[3];
    for (int a = 0; a < 3; ++a)
        r2[a] = r0[a] + 10.0 * d3[a];
    rng = f.sharedRanges(m.data(), kSats, 0.0, r2);
    for (int i = 0; i < kSats; ++i)
        EXPECT_EQ(rng[i], gnss_raw::sagnacRange(m[i], r2));
    // another epoch (an age 50 ms on): taken afresh at its first unit
    rng = f.sharedRanges(m.data(), kSats, 0.05, r1);
    for (int i = 0; i < kSats; ++i)
        EXPECT_EQ(rng[i], gnss_raw::sagnacRange(m[i], r1));
    // the same epoch, the measurements in another order: none is the one taken at its place, so each its own
    // (3 m from where they were taken, so a shared one would not be these bits)
    std::vector<RawMeas> rev(m.rbegin(), m.rend());
    rng = f.sharedRanges(rev.data(), kSats, 0.05, r0);
    for (int i = 0; i < kSats; ++i)
        EXPECT_EQ(rng[i], gnss_raw::sagnacRange(rev[i], r0));
}

TEST(RawGnssShareLos, TheFrameKeptIsReceiverStatesOwnAndCarriedToSecondOrder)
{
    const auto sky = makeSky();
    Truth tr;
    std::mt19937 g(9);
    RawGnssEKF<float> f = settled(true, tr, g, sky, 5);
    double r[3], v[3], r_own[3], v_own[3];
    imuStep(f);   // (the settle's last update left a carried frame)
    for (int k = 0; k < 3; ++k)
    {
        // kept: the same bits as receiverState's (receiverEcef takes it every time)
        f.receiverEcefAt(0.0, r, v);
        f.receiverEcef(r_own, v_own);
        for (int a = 0; a < 3; ++a)
        {
            EXPECT_EQ(r[a], r_own[a]);
            EXPECT_EQ(v[a], v_own[a]);
        }
        imuStep(f);   // the position moves: taken again
    }
    // an epoch's updates carry the frame: the position within a micrometre of its own, the velocity turned by the
    // axes left as they were (1.6e-7 rad a metre carried, at 150-200 m/s)
    for (int s = 0; s < 9; ++s)
        tr.step(kStep);
    tr.step(kStep);
    const auto m = measure(sky, tr.t, tr.r, tr.v, g);
    RawGnssStats st;
    f.setFusePr(true);
    ASSERT_TRUE(f.beginGnssRaw(m.data(), kSats, 0.0, st));
    for (int i = 0; i < kSats; ++i)
        f.updateGnssRawPart(m.data(), i, i + 1, 0.0, st);
    EXPECT_GT(st.used_pr, kSats / 2);
    f.receiverEcefAt(0.0, r, v);
    f.receiverEcef(r_own, v_own);
    EXPECT_LT(dist(r, r_own), 1.0e-6);
    EXPECT_LT(dist(v, v_own), 1.0e-4);
}

TEST(RawGnssShareLos, TheBoostScreensCopyTakesTheEpochsGeometryAlong)
{
    const auto sky = makeSky();
    Truth tr;
    std::mt19937 g(21);
    RawGnssEKF<float> f = settled(true, tr, g, sky, 5);
    for (int s = 0; s < 10; ++s)
    {
        imuStep(f);
        tr.step(kStep);
    }
    const auto m = measure(sky, tr.t, tr.r, tr.v, g);
    RawGnssStats st;
    ASSERT_TRUE(f.beginGnssRaw(m.data(), kSats, 0.02, st));
    f.screenRangeRates(m.data(), kSats, 0.02, st);
    RawGnssEKF<float> dry = f;   // RawGnssShadow's dry_ = ekf_
    RawGnssStats a, b;
    for (int i = 0; i < kSats; ++i)
    {
        f.updateGnssRawPart(m.data(), i, i + 1, 0.02, a);
        dry.updateGnssRawPart(m.data(), i, i + 1, 0.02, b);
    }
    EXPECT_EQ(a.used_rr, b.used_rr);
    EXPECT_EQ(std::memcmp(f.lla(), dry.lla(), sizeof(double) * 3), 0);
    for (int i = 0; i < RawGnssEKF<float>::N; ++i)
        EXPECT_EQ(f.P(i, i), dry.P(i, i));
}

TEST(RawGnssShareLos, TheDopplerInFloatIsWithinATenthOfAMillimetrePerSecond)
{
    const auto sky = makeSky();
    Truth tr;
    std::mt19937 g(13);
    RawGnssEKF<float> d = settled(true, tr, g, sky);
    RawGnssEKF<float> fl = d;
    fl.setParams(params(true, true));
    DopplerRef tmp, ref;
    std::vector<gnss_raw::dvc::Row> rows(RawEpoch::kMaxObs), rows_f(RawEpoch::kMaxObs);
    DopplerChangeImu imu;
    {
        const auto m0 = measure(sky, tr.t, tr.r, tr.v, g);
        (void)d.dopplerChange(m0.data(), kSats, 0.0, tmp, ref, rows.data(), imu);
    }
    for (int s = 0; s < 10; ++s)
    {
        imuStep(d);
        imuStep(fl);
        tr.step(kStep);
    }
    const auto m = measure(sky, tr.t, tr.r, tr.v, g);
    DopplerRef out_d, out_f;
    ASSERT_EQ(d.dopplerChange(m.data(), kSats, 0.02, ref, out_d, rows.data(), imu), kSats);
    ASSERT_EQ(fl.dopplerChange(m.data(), kSats, 0.02, ref, out_f, rows_f.data(), imu), kSats);
    double worst_y = 0.0, worst_z = 0.0;
    for (int i = 0; i < kSats; ++i)
    {
        worst_y = std::fmax(worst_y, std::fabs(out_d.y[i] - out_f.y[i]));
        worst_z = std::fmax(worst_z, std::fabs(rows[i].z - rows_f[i].z));
    }
    EXPECT_LT(worst_y, 1.0e-4);   // m/s
    EXPECT_LT(worst_z, 1.0e-4);
    EXPECT_GT(worst_y, 0.0);      // in float: not the double's bits
    // the screen's fit and the update agree with the double prediction to the same
    RawGnssStats sd, sf;
    ASSERT_TRUE(d.beginGnssRaw(m.data(), kSats, 0.02, sd));
    ASSERT_TRUE(fl.beginGnssRaw(m.data(), kSats, 0.02, sf));
    d.screenRangeRates(m.data(), kSats, 0.02, sd);
    fl.screenRangeRates(m.data(), kSats, 0.02, sf);
    EXPECT_EQ(sf.screen_refused, sd.screen_refused);
    EXPECT_NEAR(sf.screen_dv_down_mps, sd.screen_dv_down_mps, 1.0e-4f);
    for (int i = 0; i < kSats; ++i)
    {
        d.updateGnssRawPart(m.data(), i, i + 1, 0.02, sd);
        fl.updateGnssRawPart(m.data(), i, i + 1, 0.02, sf);
    }
    EXPECT_EQ(sf.used_rr, sd.used_rr);
    for (int a = 0; a < 3; ++a)
        EXPECT_NEAR(fl.velNed()[a], d.velNed()[a], 1.0e-4f);
}

// The shared geometry with the Dopplers in pairs (exp/v9-dop-avg; the bench
// integration exp/v9-moredata-bench): an epoch in pairs takes its Dopplers
// rr_pair_lag_s before the tag, and each satellite's own range acceleration
// once for all its consumers (pairAccel). With share_los those consumers read
// the epoch's lines of sight: the shared filter stays within micrometres of
// the unshared one and uses and refuses the same. rr_float steps aside where
// the Doppler has a lag, so on an epoch in pairs share_los 2 gives share_los
// 1's very bits; on one handed over as it came (no lag on the mosaic) it is
// float again, other bits.
TEST(RawGnssShareLos, AnEpochInPairsTakesTheSharedGeometryAndTheDoublePrediction)
{
    const auto sky = makeSky();
    Truth tr;
    std::mt19937 g(21);
    auto paired = [](RawGnssEKFParams p) {
        p.rr_pair_lag_s = 0.025;          // the mosaic's 50 ms pairs: their middle
        return p;
    };
    RawGnssEKF<float> off = settled(false, tr, g, sky);
    off.setParams(paired(params(false)));
    RawGnssEKF<float> on = off;
    on.setParams(paired(params(true)));
    DopplerRef ref_off, ref_on;
    gnss_raw::SatJumpCheck sc_off, sc_on;
    double worst_r = 0.0, worst_v = 0.0, worst_vt = 0.0;
    int used_off = 0, used_on = 0, rej_off = 0, rej_on = 0, float_differs = 0, n_paired = 0, n_alone = 0;
    for (int e = 0; e < 40; ++e)
    {
        for (int s = 0; s < 10; ++s)
            tr.step(kStep);
        // three epochs in four in pairs: their Dopplers the range rates 25 ms before the tag
        const bool in_pairs = e % 4 != 3;
        double v_d[3];
        for (int k = 0; k < 3; ++k)
            v_d[k] = tr.v[k] - (in_pairs ? 30.0 * 0.025 * tr.up[k] : 0.0);
        const auto m = measure(sky, tr.t, tr.r, v_d, g);
        const double age = 0.03;
        while (off.time() < tr.t + age - 1.0e-6)
        {
            imuStep(off);
            imuStep(on);
        }
        off.setRrPaired(in_pairs);
        on.setRrPaired(in_pairs);
        ASSERT_EQ(on.rrPaired(), in_pairs);
        EXPECT_DOUBLE_EQ(on.rrLag(), in_pairs ? 0.025 : 0.0);
        // rr_float on a copy of the shared filter: the same epoch, the same work
        RawGnssEKF<float> fl = on;
        fl.setParams(paired(params(true, true)));
        DopplerRef ref_fl = ref_on;
        gnss_raw::SatJumpCheck sc_fl = sc_on;
        const RawGnssStats so = workEpoch(off, m.data(), age, ref_off, sc_off, 100.0 + 0.1 * e, e % 10 == 0, true);
        const RawGnssStats sn = workEpoch(on, m.data(), age, ref_on, sc_on, 100.0 + 0.1 * e, e % 10 == 0, true);
        const RawGnssStats sf = workEpoch(fl, m.data(), age, ref_fl, sc_fl, 100.0 + 0.1 * e, e % 10 == 0, true);
        used_off += so.used_pr + so.used_rr;
        used_on += sn.used_pr + sn.used_rr;
        rej_off += so.rejected_pr + so.rejected_rr;
        rej_on += sn.rejected_pr + sn.rejected_rr;
        double ro[3], vo[3], rn[3], vn[3];
        off.receiverEcef(ro, vo);
        on.receiverEcef(rn, vn);
        worst_r = std::fmax(worst_r, dist(ro, rn));
        worst_v = std::fmax(worst_v, dist(vo, vn));
        if (e >= 20)
        {
            double vt[3];                 // the truth's velocity at the filter's time
            for (int k = 0; k < 3; ++k)
                vt[k] = tr.v[k] + 30.0 * (on.time() - tr.t) * tr.up[k];
            worst_vt = std::fmax(worst_vt, dist(vn, vt));
        }
        const bool same = std::memcmp(fl.velNed(), on.velNed(), 3 * sizeof(float)) == 0 &&
                          std::memcmp(fl.lla(), on.lla(), 3 * sizeof(double)) == 0 &&
                          std::memcmp(fl.quat(), on.quat(), 4 * sizeof(float)) == 0 &&
                          fl.clockDrift() == on.clockDrift();
        if (in_pairs)
        {
            n_paired++;
            EXPECT_TRUE(same) << "epoch " << e;
            EXPECT_EQ(sf.used_rr, sn.used_rr);
            EXPECT_EQ(sf.screen_rr, sn.screen_rr);
        }
        else
        {
            n_alone++;
            float_differs += same ? 0 : 1;
        }
    }
    EXPECT_EQ(n_paired, 30);
    EXPECT_EQ(n_alone, 10);
    EXPECT_GT(float_differs, 0);          // rr_float is live on the epochs handed over as they came
    EXPECT_EQ(used_on, used_off);
    EXPECT_EQ(rej_on, rej_off);
    EXPECT_GT(used_on, 40 * kSats);
    // each epoch's Dopplers taken at their own instant: taken at the tag, the pairs' would put the climb's
    // change over 25 ms (0.75 m/s) into the velocity
    EXPECT_LT(worst_vt, 0.15);            // m/s
    EXPECT_LT(worst_r, 1.0e-4);           // m
    EXPECT_LT(worst_v, 1.0e-4);           // m/s
}
