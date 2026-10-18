// Host tests for the Doppler change check (exp/v9-dvcheck): its arithmetic
// (TR_RawGnssEKF/DopplerChange.h), the EKF's rows (RawGnssEKF::dopplerChange)
// and the shadow's use of them (RawGnssShadow's dv_check) -- each satellite's
// Doppler change between epochs against the IMU's velocity change along its
// line of sight, the receiver's whole change fitted out; the epoch's change
// against the IMU's with its allowances.
//
// The sky is test_raw_gnss_ekf.cpp's: ten satellites fixed in place with an
// orbit's speed, their range rates made with the Earth-rotation model the
// filter predicts with, and a receiver clock whose drift can step.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "DopplerChange.h"
#include "RawGnssShadow.h"
#include "RawMeas.h"
#include "ReceiverProfileApply.h"
#include "TR_RawGnssEKF.h"
#include "gnss/atmo.h"
#include "gnss/types.h"

using gnss_raw::DopplerChangeImu;
using gnss_raw::DopplerRef;
using gnss_raw::RawGnssEKF;
using gnss_raw::RawGnssEKFParams;
using gnss_raw::RawGnssShadow;
using gnss_raw::RawMeas;
using gnss_raw::RawEpoch;
namespace dvc = gnss_raw::dvc;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kC = 299792458.0;
constexpr double kOmegaE = 7.2921151467e-5;
constexpr double kG = 9.807;

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

void nedToEcef(double lat, double lon, const double ned[3], double out[3])
{
    const double enu[3] = {ned[1], ned[0], -ned[2]};
    enuToEcef(lat, lon, enu, out);
}

std::vector<Sat> makeSky(const double lla[3])
{
    struct AzEl { int sys, prn; double az, el; };
    const AzEl dirs[] = {{GNSS_SYS_GPS, 3, 0, 70},   {GNSS_SYS_GPS, 8, 60, 35},   {GNSS_SYS_GPS, 14, 130, 20},
                         {GNSS_SYS_GPS, 22, 200, 50}, {GNSS_SYS_GPS, 31, 290, 25}, {GNSS_SYS_GAL, 4, 30, 15},
                         {GNSS_SYS_GAL, 11, 170, 60}, {GNSS_SYS_GAL, 26, 250, 40}, {GNSS_SYS_BDS, 23, 100, 45},
                         {GNSS_SYS_BDS, 37, 320, 55}};
    double r[3];
    geo_to_ecef(lla[0], lla[1], lla[2], r);
    std::vector<Sat> sky;
    for (const AzEl& d : dirs)
    {
        const double az = d.az * kPi / 180.0, el = d.el * kPi / 180.0;
        const double enu[3] = {std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el)};
        double u[3];
        enuToEcef(lla[0], lla[1], enu, u);
        const double R = 26.56e6, ru = r[0] * u[0] + r[1] * u[1] + r[2] * u[2];
        const double rr = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
        const double rho = -ru + std::sqrt(ru * ru - rr + R * R);
        Sat s{d.sys, d.prn, {}, {}, {}};
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
    rho = std::sqrt((rs[0] - r[0]) * (rs[0] - r[0]) + (rs[1] - r[1]) * (rs[1] - r[1])
                    + (rs[2] - r[2]) * (rs[2] - r[2]));
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

// The receiver at r (ECEF) measures the sky. Each satellite's range rate is
// at v_rr[k] (ECEF; a stale satellite's own), or v; the clock's bias and drift.
std::vector<RawMeas> measure(const std::vector<Sat>& sky, const double r[3], const double v[3], double bias,
                             double drift, const std::vector<const double*>& v_rr = {})
{
    std::vector<RawMeas> out;
    for (size_t i = 0; i < sky.size(); ++i)
    {
        const Sat& s = sky[i];
        RawMeas m{};
        m.sys = static_cast<uint8_t>(s.sys);
        m.prn = static_cast<uint8_t>(s.prn);
        for (int k = 0; k < 3; ++k)
        {
            m.sat_pos[k] = s.pos[k];
            m.sat_vel[k] = s.vel[k];
            m.sat_acc[k] = s.acc[k];
        }
        double rho, rate;
        rangeAndRate(s.pos, s.vel, r, i < v_rr.size() && v_rr[i] ? v_rr[i] : v, rho, rate);
        m.has_pr = m.has_rr = true;
        m.pr = rho + bias;
        m.rr = rate + drift;
        m.sigma_pr = 1.0f;
        m.tau_pr = 15.0f;
        m.sigma_rr = 0.05f;
        out.push_back(m);
    }
    return out;
}

const double kLla0[3] = {32.99 * kPi / 180.0, -106.97 * kPi / 180.0, 1400.0};

// ------------------------------------------------------------------ the arithmetic

// unit vectors spread over the sky (NED, receiver to satellite: up is -down)
std::vector<std::array<double, 3>> losSet(int n)
{
    std::vector<std::array<double, 3>> u;
    for (int i = 0; i < n; ++i)
    {
        const double az = (137.5 * i + 17.0) * kPi / 180.0, el = (15.0 + 15.0 * (i % 5)) * kPi / 180.0;
        u.push_back({std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), -std::sin(el)});
    }
    return u;
}

std::vector<dvc::Row> rowsFor(const double w[3], double c, int n, double s = 0.05)
{
    std::vector<dvc::Row> r(static_cast<size_t>(n));
    const auto u = losSet(n);
    for (int i = 0; i < n; ++i)
    {
        dvc::Row& x = r[static_cast<size_t>(i)];
        for (int a = 0; a < 3; ++a)
            x.u[a] = u[static_cast<size_t>(i)][a];
        x.z = -(x.u[0] * w[0] + x.u[1] * w[1] + x.u[2] * w[2]) + c;
        x.s = s;
        x.im = x.iref = x.iout = i;
    }
    return r;
}

}  // namespace

TEST(DopplerChange, TheFitFindsTheReceiversChangeAndTheClocks)
{
    const double w[3] = {0.3, -0.2, 1.1}, c = 0.7;
    auto r = rowsFor(w, c, 10);
    const dvc::Fit f = dvc::fit(r.data(), 10, 5.0);
    ASSERT_TRUE(f.ok);
    EXPECT_EQ(f.nin, 10);
    EXPECT_EQ(f.nout, 0);
    for (int a = 0; a < 3; ++a)
        EXPECT_NEAR(f.x[a], w[a], 1e-9);
    EXPECT_NEAR(f.x[3], c, 1e-9);
    EXPECT_NEAR(f.chi2, 0.0, 1e-12);
    for (const dvc::Row& x : r)
        EXPECT_NEAR(x.e, 0.0, 1e-9);
    // its covariance: symmetric, and the vertical (sky on one side) worse than the horizontal
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            EXPECT_NEAR(f.C[i][j], f.C[j][i], 1e-15);
    EXPECT_GT(f.C[2][2], f.C[0][0]);
    EXPECT_GT(f.C[0][0], 0.0);
}

TEST(DopplerChange, OneSatelliteOnItsOwnIsLeftOutAndFails)
{
    const double w[3] = {0.0, 0.0, 0.0};
    auto r = rowsFor(w, 0.2, 10);
    r[4].z += 1.0;                        // 20 of its sigmas
    const dvc::Fit f = dvc::fit(r.data(), 10, 5.0);
    ASSERT_TRUE(f.ok);
    EXPECT_EQ(f.nout, 1);
    EXPECT_FALSE(r[4].in);
    EXPECT_GT(std::fabs(r[4].e), 15.0);
    for (int i = 0; i < 10; ++i)
        if (i != 4)
            EXPECT_LT(std::fabs(r[static_cast<size_t>(i)].e), 1e-6);
    // the receiver's change is the rest's
    for (int a = 0; a < 3; ++a)
        EXPECT_NEAR(f.x[a], 0.0, 1e-9);
    // a common change on every satellite is the clock's, not a fault
    auto r2 = rowsFor(w, 3.0, 10);
    const dvc::Fit f2 = dvc::fit(r2.data(), 10, 5.0);
    EXPECT_EQ(f2.nout, 0);
    EXPECT_NEAR(f2.x[3], 3.0, 1e-9);
}

TEST(DopplerChange, SixStayInTheFit)
{
    // three of seven off on their own: the fit stops leaving out at six
    const double w[3] = {0.0, 0.0, 0.0};
    auto r = rowsFor(w, 0.0, 7);
    r[0].z += 2.0;
    r[3].z -= 3.0;
    r[5].z += 4.0;
    const dvc::Fit f = dvc::fit(r.data(), 7, 5.0);
    ASSERT_TRUE(f.ok);
    EXPECT_EQ(f.nin, 6);
    EXPECT_EQ(f.nout, 1);
}

TEST(DopplerChange, ASatelliteAlreadyOutStaysOutOfTheFit)
{
    // a satellite on hold drifting 0.3 m/s an epoch (6 sigmas): kept out, the receiver's change is the rest's
    const double w[3] = {0.0, 0.0, 0.0};
    auto r = rowsFor(w, 0.0, 10);
    for (dvc::Row& x : r)
        x.in = true;
    r[2].z += 0.3;
    r[2].in = false;
    const dvc::Fit f = dvc::fit(r.data(), 10, 5.0, 6, true);
    ASSERT_TRUE(f.ok);
    EXPECT_FALSE(r[2].in);
    EXPECT_EQ(f.nin, 9);
    for (int a = 0; a < 3; ++a)
        EXPECT_NEAR(f.x[a], 0.0, 1e-9);
    EXPECT_NEAR(r[2].e, 6.0, 1e-6);      // judged against the rest
    // taken back in, it pulls the fit and its own residual shrinks
    auto r2 = rowsFor(w, 0.0, 10);
    r2[2].z += 0.3;
    const dvc::Fit f2 = dvc::fit(r2.data(), 10, 5.0);
    EXPECT_TRUE(r2[2].in);
    EXPECT_LT(std::fabs(r2[2].e), 6.0);
    // with under five left in, every row goes back in
    auto r3 = rowsFor(w, 0.0, 6);
    for (dvc::Row& x : r3)
        x.in = false;
    r3[0].in = r3[1].in = r3[2].in = r3[3].in = true;
    const dvc::Fit f3 = dvc::fit(r3.data(), 6, 5.0, 6, true);
    EXPECT_EQ(f3.nin, 6);
}

TEST(DopplerChange, UnderFourRowsOrNoVelocityIsNoFit)
{
    const double w[3] = {0.1, 0.2, 0.3};
    auto r = rowsFor(w, 0.0, 3);
    EXPECT_FALSE(dvc::fit(r.data(), 3, 5.0).ok);
    // every line of sight the same: no velocity in the geometry
    auto same = rowsFor(w, 0.0, 6);
    for (dvc::Row& x : same)
    {
        x.u[0] = 0.0;
        x.u[1] = 0.6;
        x.u[2] = -0.8;
    }
    EXPECT_FALSE(dvc::fit(same.data(), 6, 5.0).ok);
}

TEST(DopplerChange, TheAttitudeAllowanceLeavesTheChangeAlongTheSpecificForce)
{
    // a boost epoch's specific force: 10 m/s up in 0.1 s; the fit's covariance 5 cm/s each way
    const double sf[3] = {0.0, 0.0, -10.0};
    double Cw[3][3] = {};
    for (int i = 0; i < 3; ++i)
        Cw[i][i] = 0.05 * 0.05;
    dvc::Allow a;
    a.att_rad = 30.0 * kPi / 180.0;
    // a 5 deg attitude error turns the change 0.87 m/s across it: well inside the allowance
    const double across[3] = {0.87, 0.0, 0.0};
    EXPECT_LT(dvc::epochChi2(across, Cw, sf, a), 1.0);
    // a receiver reporting 10 % less of the change: 1 m/s along it
    const double along[3] = {0.0, 0.0, 1.0};
    EXPECT_GT(dvc::epochChi2(along, Cw, sf, a), 300.0);
    EXPECT_NEAR(dvc::alongSf(along, sf), -1.0, 1e-12);
    // with no specific force (free fall) the attitude allows nothing: the whole change is judged
    const double none[3] = {0.0, 0.0, 0.0};
    EXPECT_NEAR(dvc::epochChi2(across, Cw, none, a), 0.87 * 0.87 / (0.05 * 0.05), 1e-6);
    EXPECT_EQ(dvc::alongSf(along, none), 0.0);
}

TEST(DopplerChange, WithNoAllowanceTheChiSquareIsThePlainOne)
{
    double Cw[3][3] = {{0.04, 0.01, 0.0}, {0.01, 0.09, 0.0}, {0.0, 0.0, 0.25}};
    const double sf[3] = {0.0, 0.0, 0.0};
    const double w[3] = {0.2, -0.3, 0.5};
    // w' C^-1 w by hand
    const double det2 = 0.04 * 0.09 - 0.01 * 0.01;
    const double q2 = (0.09 * 0.2 * 0.2 - 2 * 0.01 * 0.2 * -0.3 + 0.04 * 0.3 * 0.3) / det2;
    EXPECT_NEAR(dvc::epochChi2(w, Cw, sf, dvc::Allow()), q2 + 0.5 * 0.5 / 0.25, 1e-9);
    // the bias and the lever arm add on every axis
    dvc::Allow b;
    b.bias_mps = 0.3;
    b.lever_mps = 0.4;
    double S[3][3];
    dvc::allowCov(Cw, sf, b, S);
    EXPECT_NEAR(S[2][2], 0.25 + 0.09 + 0.16, 1e-12);
    EXPECT_NEAR(S[0][1], 0.01, 1e-12);
    // a singular covariance says so
    double Z[3][3] = {};
    EXPECT_LT(dvc::epochChi2(w, Z, sf, dvc::Allow()), 0.0);
}

TEST(DopplerChange, TheHeadingAllowanceTakesAnyTurnAboutTheVertical)
{
    // a coast epoch's change on a tilted trajectory, the drag 0.7 m/s across the ground and 0.9 m/s up; the
    // fit's covariance 5 cm/s each way
    const double sf[3] = {0.6, 0.35, -0.9};
    double Cw[3][3] = {};
    for (int i = 0; i < 3; ++i)
        Cw[i][i] = 0.05 * 0.05;
    dvc::Allow a;
    a.att_rad = 10.0 * kPi / 180.0;
    a.scale = 0.01;
    // the IMU's change in a frame whose heading is psi off: w = (R(psi) - I) sf, horizontal only
    auto turned = [&](double psi_deg, double w[3]) {
        const double c = std::cos(psi_deg * kPi / 180.0), s = std::sin(psi_deg * kPi / 180.0);
        w[0] = c * sf[0] - s * sf[1] - sf[0];
        w[1] = s * sf[0] + c * sf[1] - sf[1];
        w[2] = 0.0;
    };
    EXPECT_DOUBLE_EQ(dvc::headingChord(kPi), 2.0);
    EXPECT_NEAR(dvc::headingChord(kPi / 3.0), 1.0, 1e-12);
    dvc::Allow any = a;
    any.hdg_rad = kPi;
    for (double psi : {30.0, 90.0, 120.0, -150.0, 180.0})
    {
        double w[3];
        turned(psi, w);
        // the attitude's small-angle allowance takes none of a heading far off: the epoch would fail
        if (std::fabs(psi) >= 90.0)
            EXPECT_GT(dvc::epochChi2(w, Cw, sf, a), 16.0) << psi;
        // any heading: well inside 4 sigmas, whatever the turn
        EXPECT_LT(dvc::epochChi2(w, Cw, sf, any), 2.0) << psi;
    }
    // In a boost near vertical (0.4 m/s across the ground, 3 m/s up) the vertical is judged as before: a
    // receiver reporting a sixth less of the climb still fails. (On the tilted coast above the attitude
    // allowance tilts the change, so with the heading free the vertical alone is judged more loosely: the part
    // along the change no longer ties it to the horizontal.)
    const double boost[3] = {0.35, 0.2, -3.0};
    const double lag[3] = {0.0, 0.0, 0.5};
    EXPECT_GT(dvc::epochChi2(lag, Cw, boost, any), 16.0);
    double S0[3][3], S1[3][3];
    dvc::allowCov(Cw, sf, a, S0);
    dvc::allowCov(Cw, sf, any, S1);
    EXPECT_EQ(S1[2][2], S0[2][2]);
    EXPECT_EQ(S1[0][2], S0[0][2]);
    EXPECT_NEAR(S1[0][0] - S0[0][0], 4.0 * (0.6 * 0.6 + 0.35 * 0.35), 1e-12);
    EXPECT_NEAR(S1[1][1] - S0[1][1], 4.0 * (0.6 * 0.6 + 0.35 * 0.35), 1e-12);
    EXPECT_EQ(S1[0][1], S0[0][1]);
    // and 0 is no heading allowance at all, bit for bit
    dvc::Allow zero = a;
    zero.hdg_rad = 0.0;
    double Sz[3][3];
    dvc::allowCov(Cw, sf, zero, Sz);
    EXPECT_EQ(0, std::memcmp(Sz, S0, sizeof(Sz)));
}

// ------------------------------------------------------------------ the EKF's rows

TEST(DopplerChange, TheLevelTestTakesTheFiltersOwnVelocityUncertainty)
{
    // a receiver 1 m/s off the filter's velocity, down: against a tight fit alone it is far off; against a
    // filter 1 m/s unsure it is one sigma; the floor adds the same way
    const double off[3] = {0.0, 0.0, 1.0};
    double Cw[3][3] = {}, P0[3][3] = {}, P1[3][3] = {};
    for (int i = 0; i < 3; ++i)
    {
        Cw[i][i] = 0.02 * 0.02;
        P1[i][i] = 1.0;
    }
    EXPECT_NEAR(dvc::levelChi2(off, Cw, P0, 0.0), 1.0 / (0.02 * 0.02), 1e-6);
    EXPECT_NEAR(dvc::levelChi2(off, Cw, P1, 0.0), 1.0 / (1.0 + 0.02 * 0.02), 1e-9);
    EXPECT_NEAR(dvc::levelChi2(off, Cw, P0, 0.25), 1.0 / (0.25 + 0.02 * 0.02), 1e-9);
    // only the down variance matters for a down offset
    double Ph[3][3] = {};
    Ph[0][0] = Ph[1][1] = 100.0;
    EXPECT_NEAR(dvc::levelChi2(off, Cw, Ph, 0.0), 1.0 / (0.02 * 0.02), 1e-6);
    // singular: no answer
    const double Z[3][3] = {};
    EXPECT_LT(dvc::levelChi2(off, Z, Z, 0.0), 0.0);
}

namespace
{

struct TwoEpochs
{
    int nr = 0;
    DopplerChangeImu imu;
    dvc::Fit fit;
    std::vector<dvc::Row> rows;
    DopplerRef ref0, ref1;
};

// A level body (body = NED) whose filter starts v_err off the truth, still;
// epoch 0, then 0.1 s of a_ned (s of IMU steps, 5 ms), then epoch 1 -- its
// Dopplers at v_rep (NED; the truth's if null) and its clock drift stepped
// by d_drift. Bad: that satellite's Doppler moved off by this much more.
TwoEpochs twoEpochs(const double a_ned[3], const double v_err[3], const double* v_rep, double d_drift,
                    int bad = -1, double bad_mps = 0.0)
{
    RawGnssEKFParams p;
    p.rr_lag_s = 0.0;
    RawGnssEKF<double> f(p);
    const double q[4] = {1, 0, 0, 0};
    f.init(kLla0, v_err, q, 0.0);
    f.setQuaternion(q);
    const auto sky = makeSky(kLla0);
    double r0[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r0);
    const double v0[3] = {0.0, 0.0, 0.0};
    TwoEpochs out;
    auto m0 = measure(sky, r0, v0, 100.0, -3.0);
    // a short pad, so the history covers the interval
    const double still[3] = {0.0, 0.0, -kG}, w0[3] = {0, 0, 0};
    for (int i = 0; i < 40; ++i)
        f.propagate(still, w0, 0.005);
    std::vector<dvc::Row> rows(RawEpoch::kMaxObs);
    DopplerRef none;
    EXPECT_EQ(f.dopplerChange(m0.data(), static_cast<int>(m0.size()), 0.0, none, out.ref0, rows.data(), out.imu), 0);
    EXPECT_FALSE(out.imu.ok);
    EXPECT_TRUE(out.ref0.valid);
    EXPECT_EQ(out.ref0.n, 10);
    const double acc[3] = {a_ned[0], a_ned[1], a_ned[2] - kG};
    for (int i = 0; i < 20; ++i)
        f.propagate(acc, w0, 0.005);
    double v1n[3], v1[3], p1n[3], dr[3], r1[3], vrep[3];
    for (int k = 0; k < 3; ++k)
    {
        v1n[k] = a_ned[k] * 0.1;
        p1n[k] = 0.5 * a_ned[k] * 0.01;
    }
    nedToEcef(kLla0[0], kLla0[1], v1n, v1);
    nedToEcef(kLla0[0], kLla0[1], p1n, dr);
    for (int k = 0; k < 3; ++k)
        r1[k] = r0[k] + dr[k];
    if (v_rep)
        nedToEcef(kLla0[0], kLla0[1], v_rep, vrep);
    auto m1 = measure(sky, r1, v_rep ? vrep : v1, 100.0 - 0.3, -3.0 + d_drift);
    if (bad >= 0)
        m1[static_cast<size_t>(bad)].rr += bad_mps;
    out.nr = f.dopplerChange(m1.data(), static_cast<int>(m1.size()), 0.0, out.ref0, out.ref1, rows.data(), out.imu);
    rows.resize(static_cast<size_t>(out.nr));
    out.fit = dvc::fit(rows.data(), out.nr, 5.0);
    out.rows = rows;
    return out;
}

}  // namespace

TEST(DopplerChange, AnEpochThatChangedAsTheImuSaysLeavesOnlyTheClock)
{
    // 3 g up and a little sideways for 0.1 s; the filter's own velocity 3.7 m/s wrong: it drops out
    const double a[3] = {1.0, 2.0, -30.0}, v_err[3] = {2.0, -1.0, 3.0};
    const TwoEpochs e = twoEpochs(a, v_err, nullptr, 0.4);
    ASSERT_EQ(e.nr, 10);
    ASSERT_TRUE(e.imu.ok);
    EXPECT_NEAR(e.imu.dt, 0.1, 1e-9);
    for (int k = 0; k < 3; ++k)
        EXPECT_NEAR(e.imu.dv[k], a[k] * 0.1, 1e-4);
    EXPECT_NEAR(e.imu.dv_sf[2], (a[2] - kG) * 0.1, 1e-4);
    ASSERT_TRUE(e.fit.ok);
    for (int k = 0; k < 3; ++k)
        EXPECT_NEAR(e.fit.x[k], 0.0, 2e-3) << k;
    EXPECT_NEAR(e.fit.x[3], 0.4, 2e-3);       // the clock drift's step, every satellite's
    EXPECT_EQ(e.fit.nout, 0);
    for (const dvc::Row& x : e.rows)
    {
        EXPECT_LT(std::fabs(x.e), 0.1);
        EXPECT_NEAR(x.s, std::sqrt(2.0) * 0.05, 1e-6);   // both ends' sigmas
    }
}

TEST(DopplerChange, AStaleReceiverReportsLessOfTheChange)
{
    // the receiver's velocity moves 60 % of the way: its change 40 % short, along the specific force
    const double a[3] = {1.0, 2.0, -30.0}, v_err[3] = {0.0, 0.0, 0.0};
    const double v_rep[3] = {0.6 * a[0] * 0.1, 0.6 * a[1] * 0.1, 0.6 * a[2] * 0.1};
    const TwoEpochs e = twoEpochs(a, v_err, v_rep, 0.0);
    ASSERT_TRUE(e.fit.ok);
    for (int k = 0; k < 3; ++k)
        EXPECT_NEAR(e.fit.x[k], -0.4 * a[k] * 0.1, 3e-3) << k;
    const double w[3] = {e.fit.x[0], e.fit.x[1], e.fit.x[2]};
    const double sf[3] = {e.imu.dv_sf[0], e.imu.dv_sf[1], e.imu.dv_sf[2]};
    EXPECT_LT(dvc::alongSf(w, sf), -1.0);
    double Cw[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            Cw[i][j] = e.fit.C[i][j];
    dvc::Allow al;
    al.att_rad = 30.0 * kPi / 180.0;
    al.scale = 0.01;
    al.bias_mps = 0.01;
    EXPECT_GT(dvc::epochChi2(w, Cw, sf, al), 25.0);
    // no satellite on its own: the change is the receiver's
    EXPECT_EQ(e.fit.nout, 0);
}

TEST(DopplerChange, OneSatellitesStaleDopplerIsItsAlone)
{
    const double a[3] = {0.0, 0.0, -30.0}, v_err[3] = {0.0, 0.0, 0.0};
    const TwoEpochs e = twoEpochs(a, v_err, nullptr, 0.0, 6, -0.8);
    ASSERT_TRUE(e.fit.ok);
    EXPECT_EQ(e.fit.nout, 1);
    for (const dvc::Row& x : e.rows)
    {
        if (x.im == 6)
            EXPECT_GT(std::fabs(x.e), 5.0);
        else
            EXPECT_LT(std::fabs(x.e), 0.2);
    }
    for (int k = 0; k < 3; ++k)
        EXPECT_NEAR(e.fit.x[k], 0.0, 3e-3);
}

TEST(DopplerChange, NoReferenceOrNoHistoryAcrossGivesNoRows)
{
    RawGnssEKFParams p;
    p.rr_lag_s = 0.0;
    RawGnssEKF<double> f(p);
    const double q[4] = {1, 0, 0, 0}, v0[3] = {0, 0, 0};
    f.init(kLla0, v0, q, 0.0);
    const double still[3] = {0.0, 0.0, -kG}, w0[3] = {0, 0, 0};
    for (int i = 0; i < 400; ++i)
        f.propagate(still, w0, 0.005);
    const auto sky = makeSky(kLla0);
    double r0[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r0);
    auto m = measure(sky, r0, v0, 0.0, 0.0);
    std::vector<dvc::Row> rows(RawEpoch::kMaxObs);
    DopplerRef ref, out;
    DopplerChangeImu imu;
    ref.valid = true;
    ref.n = 1;
    ref.sys[0] = GNSS_SYS_GPS;
    ref.prn[0] = 3;
    ref.t = 0.5;                          // 1.5 s back: the history (128 x 5 ms) does not reach it
    EXPECT_EQ(f.dopplerChange(m.data(), static_cast<int>(m.size()), 0.0, ref, out, rows.data(), imu), 0);
    EXPECT_FALSE(imu.ok);
    EXPECT_EQ(out.n, 10);                 // this epoch is still the next one's start
}

// ------------------------------------------------------------------ the shadow

namespace
{

// A level body (its nose is no matter here) on a pad 3 s, then 1.5 s at
// 30 m/s^2 up and 2 north, then 6.5 s decelerating at 1 m/s^2 past gravity:
// raw epochs at 10 Hz from the truth, built (external_build) and handed over
// 30 ms after their instant. Faults after the boost (the sim's stale
// patterns, sim_events.py): the satellites in stale (indices) report the
// velocity v - L v, L = exp(-(t - burnout) / stale_tau) -- 1 through the
// boost; drift_mps2: satellite drift_sat's Doppler moves this much a second
// more from burnout on.
struct DvFlight
{
    RawGnssShadow::Config cfg;
    std::vector<int> stale;
    double stale_tau = 1.0;
    int drift_sat = -1;
    double drift_mps2 = 0.0;
    // a coherent fault (sim_events.py's coherent and slow patterns): every Doppler at the receiver's velocity
    // plus coh_rate m/s a second down from coh_from s, held at coh_max (coh_from < 0: none)
    double coh_from = -1.0;
    double coh_rate = 0.0;
    double coh_max = 0.0;
    // the pad's heading guess this far off the truth, about the vertical (no magnetometer here: the start's)
    double seed_hdg_deg = 0.0;
};

struct DvRun
{
    RawGnssShadow::Stats st;
    gnss_raw::ShadowNav nav{};
    double v_err_coast_max = 0.0;         // |filter - truth| velocity, the largest from 1 s after burnout
    double v_err_end = 0.0;               //   at the end
    uint32_t held_last = 0;               // Dopplers held in the last second
    double hdg_share_end = 1.0;           // the last epoch's heading share (hdg_obs_mode; 1 with it off)
};

RawGnssShadow::Config dvCfg(uint8_t mode)
{
    RawGnssShadow::Config c;
    c.primary = true;
    c.min_step_s = 0.01;
    c.external_build = true;
    c.ekf.rr_lag_s = 0.0;                 // this sky's Dopplers are on time (a lag the filter assumes and the
                                          //   receiver has not shows at a thrust step: the boost's end here)
    c.dv_check = mode;
    return c;
}

DvRun flyDv(const DvFlight& o)
{
    RawGnssShadow sh(o.cfg);
    const auto sky = makeSky(kLla0);
    double r0[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r0);
    const int64_t t0 = 50000000;
    gnss_raw::ShadowSeed seed{};
    seed.time_us = static_cast<uint32_t>(t0);
    seed.lla[0] = kLla0[0];
    seed.lla[1] = kLla0[1];
    seed.lla[2] = kLla0[2];
    seed.quat[0] = static_cast<float>(std::cos(0.5 * o.seed_hdg_deg * kPi / 180.0));
    seed.quat[3] = static_cast<float>(std::sin(0.5 * o.seed_hdg_deg * kPi / 180.0));
    seed.on_pad = 1;
    sh.onSeed(seed);
    const double t_ign = 3.0, t_burn = 4.5, t_end = 11.0, dt = 0.001;
    double p[3] = {}, v[3] = {};
    std::vector<gnss_raw::BuiltEpoch> due;
    int next_ms = 100;
    DvRun run;
    uint32_t held_at_last_s = 0;
    for (int k = 1; k <= static_cast<int>(t_end / dt); ++k)
    {
        const double t = k * dt;
        double a[3] = {0.0, 0.0, 0.0};
        if (t > t_ign && t <= t_burn)
        {
            a[0] = 2.0;
            a[2] = -30.0;
        }
        else if (t > t_burn)
        {
            a[2] = kG + 1.0;
        }
        for (int i = 0; i < 3; ++i)
        {
            p[i] += v[i] * dt + 0.5 * a[i] * dt * dt;
            v[i] += a[i] * dt;
        }
        const int64_t t64 = t0 + static_cast<int64_t>(std::llround(t * 1e6));
        if (k % 4 == 0)                   // the flight filter's ticks, 250 Hz
        {
            gnss_raw::ShadowImu s{};
            s.time_us = static_cast<uint32_t>(t64);
            s.acc[0] = float(a[0]);
            s.acc[1] = float(a[1]);
            s.acc[2] = float(a[2] - kG);
            s.flags = t < t_ign ? gnss_raw::kShadowLevel : gnss_raw::kShadowInFlight;
            const uint32_t ef0 = sh.stats().dv_epoch_fails, sf0 = sh.stats().dv_sat_fails;
            const uint32_t le0 = sh.stats().dv_latch_epochs;
            sh.onImu(s, t64);
            // DV_TRACE=1: each check that failed something; 2: every tick
            const char* trace = std::getenv("DV_TRACE");
            if (trace && (sh.stats().dv_epoch_fails != ef0 || sh.stats().dv_sat_fails != sf0 || std::atoi(trace) > 1
                          || sh.stats().dv_latch_epochs != le0))
                std::printf("t %.3f checks %u epoch fails %u sat fails %u chi2 %.1f along %.3f max_e %.1f latch %u/%u "
                            "level %.1f vz %.3f\n",
                            t, sh.stats().dv_checks, sh.stats().dv_epoch_fails, sh.stats().dv_sat_fails,
                            static_cast<double>(sh.stats().dv_last_chi2),
                            static_cast<double>(sh.stats().dv_last_along), static_cast<double>(sh.stats().dv_max_e),
                            sh.stats().dv_latches, sh.stats().dv_latch_releases,
                            static_cast<double>(sh.stats().dv_last_level_chi2),
                            sh.running() ? static_cast<double>(sh.filter().velNed()[2]) - v[2] : 0.0);
            while (!due.empty() && t64 >= due.front().mcu_us + 30000)
            {
                sh.onBuilt(due.front());
                due.erase(due.begin());
            }
            if (t > t_burn + 1.0 && sh.running())
            {
                const float* fv = sh.filter().velNed();
                const double e = std::sqrt((fv[0] - v[0]) * (fv[0] - v[0]) + (fv[1] - v[1]) * (fv[1] - v[1])
                                           + (fv[2] - v[2]) * (fv[2] - v[2]));
                run.v_err_coast_max = std::max(run.v_err_coast_max, e);
                run.v_err_end = e;
            }
            if (std::fabs(t - (t_end - 1.0)) < 0.5 * dt * 4)
                held_at_last_s = sh.stats().dv_held_rr;
        }
        if (k == next_ms)
        {
            next_ms += 100;
            double dr[3], r[3], ve[3];
            nedToEcef(kLla0[0], kLla0[1], p, dr);
            nedToEcef(kLla0[0], kLla0[1], v, ve);
            for (int i = 0; i < 3; ++i)
                r[i] = r0[i] + dr[i];
            const double L = t <= t_ign ? 0.0 : (t <= t_burn ? 1.0 : std::exp(-(t - t_burn) / o.stale_tau));
            double vs_n[3] = {(1.0 - L) * v[0], (1.0 - L) * v[1], (1.0 - L) * v[2]}, vs[3];
            nedToEcef(kLla0[0], kLla0[1], vs_n, vs);
            std::vector<const double*> vrr(sky.size(), nullptr);
            double vb[3];
            if (o.coh_from >= 0.0 && t > o.coh_from)
            {
                const double b = std::min(o.coh_rate * (t - o.coh_from), o.coh_max);
                const double vb_n[3] = {v[0], v[1], v[2] + b};
                nedToEcef(kLla0[0], kLla0[1], vb_n, vb);
                for (size_t i = 0; i < sky.size(); ++i)
                    vrr[i] = vb;
            }
            for (int i : o.stale)
                vrr[static_cast<size_t>(i)] = vs;
            auto m = measure(sky, r, ve, 1000.0 + 2.0 * t, 2.0, vrr);
            if (o.drift_sat >= 0 && t > t_burn)
                m[static_cast<size_t>(o.drift_sat)].rr += o.drift_mps2 * (t - t_burn);
            gnss_raw::BuiltEpoch b;
            b.mcu_us = t64;
            b.tow_s = 300000.0 + t;
            b.gps_week = 2441;
            b.n = static_cast<int>(m.size());
            for (int i = 0; i < b.n; ++i)
            {
                b.meas[i] = m[static_cast<size_t>(i)];
                b.lock_age_s[i] = 100.0;
            }
            for (int i = 0; i < 3; ++i)
                b.r_approx[i] = r[i];
            due.push_back(b);
        }
    }
    run.st = sh.stats();
    run.held_last = run.st.dv_held_rr - held_at_last_s;
    run.hdg_share_end = static_cast<double>(sh.filter().hdgObsScale());
    sh.navState(run.nav);
    return run;
}

}  // namespace

TEST(DopplerChange, TheChecksDefaultIsOff)
{
    const RawGnssShadow::Config c;
    EXPECT_EQ(c.dv_check, RawGnssShadow::kDvOff);
    EXPECT_EQ(c.dv_window_s, 0.0);
    EXPECT_EQ(c.dv_noise_tau_s, 0.0);
    EXPECT_FALSE(c.dv_deweight);
    EXPECT_FALSE(c.dv_ground);
    EXPECT_EQ(c.dv_latch_k, 0.0);
}

TEST(DopplerChange, ObserveLeavesTheFilterAsOff)
{
    DvFlight o;
    o.stale = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};   // the whole receiver
    o.cfg = dvCfg(RawGnssShadow::kDvOff);
    const DvRun off = flyDv(o);
    o.cfg = dvCfg(RawGnssShadow::kDvObserve);
    o.cfg.dv_window_s = 1.0;
    const DvRun obs = flyDv(o);
    EXPECT_EQ(off.st.dv_checks, 0u);
    EXPECT_GT(obs.st.dv_checks, 50u);
    EXPECT_GT(obs.st.dv_epoch_fails, 0u);
    EXPECT_EQ(obs.st.dv_held_rr, 0u);
    // bit for bit
    EXPECT_EQ(0, std::memcmp(off.nav.lla, obs.nav.lla, sizeof(off.nav.lla)));
    EXPECT_EQ(0, std::memcmp(off.nav.vel_ned, obs.nav.vel_ned, sizeof(off.nav.vel_ned)));
    EXPECT_EQ(0, std::memcmp(off.nav.quat, obs.nav.quat, sizeof(off.nav.quat)));
    EXPECT_EQ(off.st.epochs_applied, obs.st.epochs_applied);
}

TEST(DopplerChange, WithNoFaultActingHoldsNothingInFlight)
{
    DvFlight o;
    o.cfg = dvCfg(RawGnssShadow::kDvOff);
    const DvRun off = flyDv(o);
    o.cfg = dvCfg(RawGnssShadow::kDvAct);
    o.cfg.dv_window_s = 1.0;
    const DvRun act = flyDv(o);
    EXPECT_GT(act.st.dv_checks, 50u);
    EXPECT_EQ(act.st.dv_sat_fails, 0u);
    EXPECT_EQ(act.st.dv_epoch_fails, 0u);
    EXPECT_EQ(act.st.dv_held_rr, 0u);
    EXPECT_EQ(0, std::memcmp(off.nav.vel_ned, act.nav.vel_ned, sizeof(off.nav.vel_ned)));
    EXPECT_EQ(0, std::memcmp(off.nav.quat, act.nav.quat, sizeof(off.nav.quat)));
}

TEST(DopplerChange, ActingHoldsOutAStaleReceiversClosingTail)
{
    // the whole receiver at the pad's velocity through the boost, closing with tau 1 s: the vertical check
    // drops the far epochs and its latch the tail to 5 m/s; the tail under that is fused -- or, acting,
    // held out while its change disagrees with the IMU's
    DvFlight o;
    o.stale = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    o.cfg = dvCfg(RawGnssShadow::kDvOff);
    const DvRun off = flyDv(o);
    o.cfg = dvCfg(RawGnssShadow::kDvAct);
    const DvRun act = flyDv(o);
    EXPECT_GT(act.st.dv_epoch_fails, 0u);
    EXPECT_GT(act.st.dv_held_epochs, 0u);
    EXPECT_GT(off.v_err_coast_max, 0.5);
    EXPECT_LT(act.v_err_coast_max, 0.5 * off.v_err_coast_max)
        << "off " << off.v_err_coast_max << ", act " << act.v_err_coast_max;
}

TEST(DopplerChange, ActingHoldsOutAStaleSatellite)
{
    DvFlight o;
    o.stale = {3};
    o.stale_tau = 2.0;
    o.cfg = dvCfg(RawGnssShadow::kDvAct);
    const DvRun act = flyDv(o);
    EXPECT_GT(act.st.dv_sat_fails, 0u);
    EXPECT_GT(act.st.dv_held_rr, 0u);
    EXPECT_EQ(act.st.dv_epoch_fails, 0u);   // the rest of the sky agrees with the IMU
    EXPECT_LT(act.v_err_coast_max, 0.3);
}

TEST(DopplerChange, TheWindowSeesASlowDriftNoEpochDoes)
{
    // 0.6 m/s a second on one Doppler: 6 cm/s an epoch, under one epoch's noise (7.7 cm/s), 60 cm/s a second
    DvFlight o;
    o.drift_sat = 7;
    o.drift_mps2 = 0.6;
    o.cfg = dvCfg(RawGnssShadow::kDvObserve);
    const DvRun epoch = flyDv(o);
    EXPECT_EQ(epoch.st.dv_sat_fails, 0u);
    o.cfg.dv_window_s = 1.0;
    const DvRun win = flyDv(o);
    EXPECT_GT(win.st.dv_sat_win_fails, 0u);
    EXPECT_EQ(win.st.dv_sat_fails, win.st.dv_sat_win_fails);
}

TEST(DopplerChange, MostOfAnEpochHeldHoldsAllOfIt)
{
    // six of ten satellites stale: each holds out on its own, and then the four left are held too
    DvFlight o;
    o.stale = {0, 2, 4, 6, 8, 9};
    o.cfg = dvCfg(RawGnssShadow::kDvAct);
    const DvRun all = flyDv(o);
    EXPECT_GT(all.st.dv_all_held + all.st.dv_epoch_fails, 0u);
    o.cfg.dv_hold_all_frac = 1.0;
    const DvRun some = flyDv(o);
    EXPECT_EQ(some.st.dv_all_held, 0u);
}

TEST(DopplerChange, TheLatchKeepsOutAReceiverThatRampedAwayAndStays)
{
    // every Doppler 3 m/s a second further down from just after burnout, held at 6 m/s: the window holds them
    // while the ramp runs; once it stops, their change agrees with the IMU's again and without the latch they
    // come back 6 m/s off. With it they stay out: their level never comes back to the filter's velocity.
    DvFlight o;
    o.coh_from = 4.7;
    o.coh_rate = 3.0;
    o.coh_max = 6.0;
    o.cfg = dvCfg(RawGnssShadow::kDvAct);
    o.cfg.dv_window_s = 1.0;
    const DvRun win = flyDv(o);
    o.cfg.dv_latch_k = 5.0;
    const DvRun latch = flyDv(o);
    EXPECT_GT(win.st.dv_epoch_fails, 0u);
    EXPECT_EQ(win.held_last, 0u) << "the window lets the biased Dopplers back";
    EXPECT_GT(win.v_err_end, 2.0);
    EXPECT_GE(latch.st.dv_latches, 1u);
    EXPECT_EQ(latch.st.dv_latch_releases, 0u);
    EXPECT_GT(latch.held_last, 50u);
    EXPECT_LT(latch.v_err_end, 1.0) << "window " << win.v_err_end << ", latch " << latch.v_err_end;
}

TEST(DopplerChange, TheLatchLetsGoWhenTheReceiverIsBackWithTheFilter)
{
    // the whole receiver at the pad's velocity through the boost, closing with tau 1 s: the latch holds from
    // the boost's first epoch and lets go once the tail is within what the IMU can have carried the filter off
    // by since; the last second is fused
    DvFlight o;
    o.stale = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    o.cfg = dvCfg(RawGnssShadow::kDvAct);
    o.cfg.dv_latch_k = 5.0;
    const DvRun latch = flyDv(o);
    EXPECT_GE(latch.st.dv_latches, 1u);
    EXPECT_GE(latch.st.dv_latch_releases, 1u);
    EXPECT_GT(latch.st.dv_latch_epochs, 30u);
    EXPECT_EQ(latch.held_last, 0u);
    EXPECT_LT(latch.v_err_end, 0.3);
}

TEST(DopplerChange, ObservedTheLatchIsCountedAndNothingHeld)
{
    DvFlight o;
    o.coh_from = 4.7;
    o.coh_rate = 3.0;
    o.coh_max = 6.0;
    o.cfg = dvCfg(RawGnssShadow::kDvOff);
    const DvRun off = flyDv(o);
    o.cfg = dvCfg(RawGnssShadow::kDvObserve);
    o.cfg.dv_window_s = 1.0;
    o.cfg.dv_latch_k = 5.0;
    const DvRun obs = flyDv(o);
    EXPECT_GE(obs.st.dv_latches, 1u);
    EXPECT_EQ(obs.st.dv_held_rr, 0u);
    EXPECT_EQ(0, std::memcmp(off.nav.vel_ned, obs.nav.vel_ned, sizeof(off.nav.vel_ned)));
    EXPECT_EQ(0, std::memcmp(off.nav.quat, obs.nav.quat, sizeof(off.nav.quat)));
}

// ------------------------------------------------------- the mosaic image's flight safeguards

TEST(DopplerChange, TheMosaicFlightSafeguardsHoldNothingWithNoFault)
{
    // gnss_raw::applyMosaicFlightSafeguards as the mosaic-only image flies
    // them (ReceiverProfileApply.h): a clean flight is checked every epoch in
    // flight, and nothing is held or latched
    DvFlight o;
    o.cfg = dvCfg(RawGnssShadow::kDvOff);
    const DvRun off = flyDv(o);
    gnss_raw::applyMosaicFlightSafeguards(o.cfg);
    const DvRun fl = flyDv(o);
    EXPECT_GT(fl.st.dv_checks, 50u);
    EXPECT_EQ(fl.st.dv_sat_fails, 0u);
    EXPECT_EQ(fl.st.dv_epoch_fails, 0u);
    EXPECT_EQ(fl.st.dv_held_rr, 0u);
    EXPECT_EQ(fl.st.dv_latches, 0u);
    EXPECT_LT(fl.v_err_coast_max, off.v_err_coast_max + 0.05)
        << "off " << off.v_err_coast_max << ", safeguards " << fl.v_err_coast_max;
    // the coast pushes nothing sideways: GNSS's heading gain is closed
    EXPECT_EQ(off.hdg_share_end, 1.0);
    EXPECT_LT(fl.hdg_share_end, 0.05);
    // and a body that never turns takes no rotation allowance
    EXPECT_EQ(fl.st.rot_max_sigma_mps, 0.0f);
}

TEST(DopplerChange, TheMosaicFlightSafeguardsHoldOutAStaleReceiverAndLetItBack)
{
    // the whole receiver at the pad's velocity through the boost, closing
    // with tau 1 s (Grid B's stale receiver): held out while its change
    // disagrees with the IMU's, and let back once it agrees with the filter
    DvFlight o;
    o.stale = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    o.cfg = dvCfg(RawGnssShadow::kDvOff);
    const DvRun off = flyDv(o);
    gnss_raw::applyMosaicFlightSafeguards(o.cfg);
    const DvRun fl = flyDv(o);
    EXPECT_GT(fl.st.dv_epoch_fails, 0u);
    EXPECT_GT(fl.st.dv_held_epochs, 0u);
    EXPECT_GT(off.v_err_coast_max, 0.5);
    EXPECT_LT(fl.v_err_coast_max, 0.5 * off.v_err_coast_max)
        << "off " << off.v_err_coast_max << ", safeguards " << fl.v_err_coast_max;
    EXPECT_EQ(fl.st.dv_latches, fl.st.dv_latch_releases);
    EXPECT_EQ(fl.held_last, 0u);
}

TEST(DopplerChange, TheMosaicFlightSafeguardsTakeAnyHeadingWithNoMagnetometerHeading)
{
    // No magnetometer on the pad (the V9's today): the heading is the start's
    // guess, any distance off. With only the 10 deg attitude allowance, a
    // guess 120-180 deg off fails the boost's epochs -- the IMU's push across
    // the ground turned the wrong way -- and the latch holds the Dopplers out.
    // With any heading allowed about the vertical nothing is held, and the
    // filter is the one without the check, bit for bit.
    for (double hdg : {120.0, -150.0, 180.0})
    {
        SCOPED_TRACE(hdg);
        DvFlight o;
        o.seed_hdg_deg = hdg;
        o.cfg = dvCfg(RawGnssShadow::kDvOff);
        gnss_raw::applyMosaicFlightSafeguards(o.cfg);
        RawGnssShadow::Config fl_cfg = o.cfg;
        o.cfg.dv_check = RawGnssShadow::kDvOff;
        const DvRun off = flyDv(o);
        o.cfg = fl_cfg;
        const DvRun fl = flyDv(o);
        EXPECT_EQ(fl.st.dv_hdg_unset, 1);
        EXPECT_GT(fl.st.dv_checks, 50u);
        EXPECT_EQ(fl.st.dv_epoch_fails, 0u);
        EXPECT_EQ(fl.st.dv_latches, 0u);
        EXPECT_EQ(fl.st.dv_held_rr, 0u);
        EXPECT_EQ(0, std::memcmp(off.nav.vel_ned, fl.nav.vel_ned, sizeof(off.nav.vel_ned)));
        EXPECT_EQ(0, std::memcmp(off.nav.quat, fl.nav.quat, sizeof(off.nav.quat)));
        // without it (RC1, 357e7021) the boost's epochs fail and the latch holds the Dopplers out
        o.cfg.dv_hdg_unset_deg = 0.0;
        const DvRun rc1 = flyDv(o);
        EXPECT_EQ(rc1.st.dv_hdg_unset, 0);
        EXPECT_GT(rc1.st.dv_epoch_fails, 0u);
        EXPECT_GE(rc1.st.dv_latches, 1u);
        EXPECT_GT(rc1.st.dv_held_rr, 0u);
    }
    // a stale receiver is still held out with the heading that far off: its
    // closing tail is a change along the climb, the vertical's
    DvFlight o;
    o.seed_hdg_deg = 150.0;
    o.stale = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    o.cfg = dvCfg(RawGnssShadow::kDvOff);
    gnss_raw::applyMosaicFlightSafeguards(o.cfg);
    const RawGnssShadow::Config fl_cfg = o.cfg;
    o.cfg.dv_check = RawGnssShadow::kDvOff;
    const DvRun off = flyDv(o);
    o.cfg = fl_cfg;
    const DvRun fl = flyDv(o);
    EXPECT_GT(fl.st.dv_epoch_fails, 0u);
    EXPECT_GT(fl.st.dv_held_epochs, 0u);
    EXPECT_LT(fl.v_err_coast_max, 0.5 * off.v_err_coast_max)
        << "off " << off.v_err_coast_max << ", safeguards " << fl.v_err_coast_max;
}

namespace
{

// q (scalar first, body -> NED) turned by the body rate w (rad/s) over dt
void turnQuat(double q[4], const double w[3], double dt)
{
    const double wn = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    if (!(wn > 0.0))
        return;
    const double s = std::sin(0.5 * wn * dt) / wn;
    const double d[4] = {std::cos(0.5 * wn * dt), s * w[0], s * w[1], s * w[2]};
    const double r[4] = {q[0] * d[0] - q[1] * d[1] - q[2] * d[2] - q[3] * d[3],
                         q[0] * d[1] + q[1] * d[0] + q[2] * d[3] - q[3] * d[2],
                         q[0] * d[2] - q[1] * d[3] + q[2] * d[0] + q[3] * d[1],
                         q[0] * d[3] + q[1] * d[2] - q[2] * d[1] + q[3] * d[0]};
    for (int i = 0; i < 4; ++i)
        q[i] = r[i];
}

}  // namespace

TEST(DopplerChange, TheNosesTurnInFloatIsTheDoublesWithinTenPpm)
{
    // dv_lever_mode 1 tracks the nose's own turn in float (NoseTurn<float>, as the shadow flies it). Against the
    // same in double, from the same float attitude and rate, 100 steps a second for 3 s: a coast weathercocking
    // (a 2 Hz sine of 0.6 rad/s about body y) under a roll of 0, 1, 5 and 15 rev/s. Each 0.1 s check's allowance,
    // the arm (0.3 m) times |dn'| + 2 age |n''|max (age 60 ms), within 1e-5 of the double's. A pure roll has none
    // in either.
    for (double rev : {0.0, 1.0, 5.0, 15.0})
    {
        SCOPED_TRACE(rev);
        for (bool weathercock : {true, false})
        {
            dvc::NoseTurn<float> nf;
            dvc::NoseTurn<double> nd;
            double q[4] = {0.69, 0.08, -0.71, 0.12};      // the nose a few degrees off vertical
            const double qn = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            for (double& c : q)
                c /= qn;
            double worst = 0.0, smallest = 1e9;
            int checks = 0;
            for (int k = 1; k <= 300; ++k)
            {
                const double t = 0.01 * k;
                const double w[3] = {rev * 2.0 * kPi, weathercock ? 0.6 * std::sin(2.0 * kPi * 2.0 * t) : 0.0, 0.0};
                turnQuat(q, w, 0.01);
                const float qf[4] = {float(q[0]), float(q[1]), float(q[2]), float(q[3])};
                const float wf[3] = {float(w[0]), float(w[1]), float(w[2])};
                nf.step(qf, wf, 0.01f);
                nd.step(qf, wf, 0.01);
                if (k % 10 != 0)
                    continue;
                const double af = 0.3 * (double(nf.change()) + 2.0 * 0.06 * double(nf.ndd_peak));
                const double ad = 0.3 * (nd.change() + 2.0 * 0.06 * nd.ndd_peak);
                if (!weathercock)
                {
                    EXPECT_EQ(af, 0.0) << t;
                    EXPECT_EQ(ad, 0.0) << t;
                }
                else if (k > 10)                          // the first check's interval starts before the first step
                {
                    worst = std::max(worst, std::fabs(af - ad) / ad);
                    smallest = std::min(smallest, ad);
                    checks++;
                }
                nf.rebase();
                nd.rebase();
            }
            if (weathercock)
            {
                EXPECT_EQ(checks, 29);
                EXPECT_GT(smallest, 0.01);                 // every check's allowance is a real one
                EXPECT_LT(worst, 1e-5);
            }
        }
    }
}
