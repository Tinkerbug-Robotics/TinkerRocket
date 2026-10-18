// Host tests for TR_RawGnssEKF's hdg_obs_mode (exp/v9-observ): the GNSS rows'
// gain on the heading -- the attitude about the vertical and the gyro bias
// along it -- scaled by how well the moment observes it, the boost left open,
// and RawGnssShadow's heading_obs_scope. On the synthetic sky of
// test_raw_gnss_ekf.cpp (its helpers copied here): a level body, the truth's
// heading 0, its filter 20 deg off with act's 30 deg prior, pushed north for
// 4 s with GNSS so the Dopplers tie the heading to the east velocity. After
// the push a horizontal velocity error from the receiver (a loop settling,
// a stale satellite) is what the full gate would turn the heading by.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include "RawGnssShadow.h"
#include "RawMeas.h"
#include "TR_RawGnssEKF.h"
#include "gnss/atmo.h"
#include "gnss/types.h"

using gnss_raw::RawGnssEKF;
using gnss_raw::RawGnssEKFParams;
using gnss_raw::RawGnssShadow;
using gnss_raw::RawMeas;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kC = 299792458.0;
constexpr double kOmegaE = 7.2921151467e-5;
constexpr double kG = 9.807;
constexpr double kDt = 0.005;     // a 200 Hz IMU
constexpr double kD2R = kPi / 180.0;

struct Sat
{
    int sys, prn;
    double pos[3], vel[3], acc[3];
};

struct Clock
{
    double bias = 1234.5, drift = -2.5, isb_gal = 12.0, isb_bds = -25.0;
    void tick(double dt) { bias += drift * dt; }
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
        const double az = d.az * kD2R, el = d.el * kD2R;
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

std::vector<RawMeas> measure(const std::vector<Sat>& sky, const double r[3], const double v[3], const Clock& clk)
{
    std::vector<RawMeas> out;
    for (const Sat& s : sky)
    {
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
        rangeAndRate(s.pos, s.vel, r, v, rho, rate);
        const double isb = s.sys == GNSS_SYS_GAL ? clk.isb_gal : (s.sys == GNSS_SYS_BDS ? clk.isb_bds : 0.0);
        m.has_pr = m.has_rr = true;
        m.pr = rho + clk.bias + isb;
        m.rr = rate + clk.drift;
        m.sigma_pr = 1.0f;
        m.sigma_pr_corr = 0.0f;
        m.tau_pr = 15.0f;
        m.sigma_rr = 0.05f;
        m.tau_rr = 0.1f;
        out.push_back(m);
    }
    return out;
}

const double kLla0[3] = {32.99 * kPi / 180.0, -106.97 * kPi / 180.0, 1400.0};

RawGnssEKFParams quietParams()
{
    RawGnssEKFParams p;
    p.rr_lag_s = 0.0;
    p.clk_drift_psd = 0.036;
    return p;
}

void quatYaw(double yaw, double q[4])
{
    q[0] = std::cos(yaw / 2);
    q[1] = q[2] = 0.0;
    q[3] = std::sin(yaw / 2);
}

// The rotation about NED down from attitude b to a (deg).
double headingDiffDeg(const double a[4], const double b[4])
{
    const double b0 = b[0], b1 = -b[1], b2 = -b[2], b3 = -b[3];
    const double r0 = a[0] * b0 - a[1] * b1 - a[2] * b2 - a[3] * b3;
    const double r3 = a[0] * b3 + a[1] * b2 - a[2] * b1 + a[3] * b0;
    const double h = 2.0 * std::atan2(r3, r0) / kD2R;
    return std::fmod(h + 540.0, 360.0) - 180.0;
}

// NED down in the body frame at attitude q
void downInBody(const double q[4], double e[3])
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    e[0] = 2 * (x * z - w * y);
    e[1] = 2 * (y * z + w * x);
    e[2] = 1 - 2 * (x * x + y * y);
}

double alongVertical(const RawGnssEKF<double>& f, const double* v3)
{
    double e[3];
    downInBody(f.quat(), e);
    return e[0] * v3[0] + e[1] * v3[1] + e[2] * v3[2];
}

// A level body's flight: each second, the IMU at a[s] (NED, m/s^2) and then
// the sky measured from the receiver's velocity, which reads v_err (NED, m/s)
// off the truth's. The truth (p, v NED from the pad) is carried across calls.
struct Truth
{
    double p[3] = {}, v[3] = {};
};

void fly(RawGnssEKF<double>& f, const std::vector<std::array<double, 3>>& a, Clock& clk, Truth& tr,
         const double v_err[3] = nullptr)
{
    const auto sky = makeSky(kLla0);
    double r0[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r0);
    for (const auto& as : a)
    {
        const double acc[3] = {as[0], as[1], as[2] - kG};
        const double gyro[3] = {0.0, 0.0, 0.0};
        for (int i = 0; i < 200; ++i)
            f.propagate(acc, gyro, kDt);
        clk.tick(1.0);
        for (int k = 0; k < 3; ++k)
        {
            tr.p[k] += tr.v[k] + 0.5 * as[k];
            tr.v[k] += as[k];
        }
        double vm[3];
        for (int k = 0; k < 3; ++k)
            vm[k] = tr.v[k] + (v_err ? v_err[k] : 0.0);
        double dp[3], dv[3], r[3];
        nedToEcef(kLla0[0], kLla0[1], tr.p, dp);
        nedToEcef(kLla0[0], kLla0[1], vm, dv);
        for (int k = 0; k < 3; ++k)
            r[k] = r0[k] + dp[k];
        const auto m = measure(sky, r, dv, clk);
        f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    }
}

// The same IMU seconds as fly, with no epoch: what the heading does on the
// gyro alone (its bias estimate turns it), for a run to be set against.
void flyNoGnss(RawGnssEKF<double>& f, const std::vector<std::array<double, 3>>& a)
{
    for (const auto& as : a)
    {
        const double acc[3] = {as[0], as[1], as[2] - kG};
        const double gyro[3] = {0.0, 0.0, 0.0};
        for (int i = 0; i < 200; ++i)
            f.propagate(acc, gyro, kDt);
    }
}

// Act's launch on a levelled pad, its heading 20 deg off with the 30 deg
// prior, then 4 s pushed north at 3 m/s^2 with GNSS from the truth.
RawGnssEKF<double> pushed(const RawGnssEKFParams& p, Clock& clk, Truth& tr)
{
    RawGnssEKF<double> f(p);
    double q0[4];
    quatYaw(20.0 * kD2R, q0);
    const double v0[3] = {0.0, 0.0, 0.0};
    f.init(kLla0, v0, q0, 0.0);
    f.setQuaternion(q0);
    const double still[3] = {0.0, 0.0, -kG}, no_rate[3] = {0.0, 0.0, 0.0};
    for (int k = 0; k < 300; ++k)                       // the pad: 6 s at 50 Hz
    {
        f.propagate(still, no_rate, 0.02);
        f.updateAccelLevel(still);
    }
    f.setHeadingSigma(30.0 * kD2R);
    f.setTiltSigmaFloor(1.0 * kD2R);
    fly(f, std::vector<std::array<double, 3>>(4, std::array<double, 3>{3.0, 0.0, 0.0}), clk, tr);
    return f;
}

RawGnssEKFParams forceParams()
{
    RawGnssEKFParams p = quietParams();
    p.hdg_obs_mode = gnss_raw::kHdgObsForce;
    p.hdg_obs_tau_s = 0.05;       // the push's mean within a step or two of the push
    p.hdg_obs_fh_mps2 = 2.0;
    return p;
}

void expectSameState(const RawGnssEKF<double>& a, const RawGnssEKF<double>& b, const char* what)
{
    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(a.quat()[i], b.quat()[i]) << what;
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_EQ(a.velNed()[i], b.velNed()[i]) << what;
        EXPECT_EQ(a.lla()[i], b.lla()[i]) << what;
        EXPECT_EQ(a.gyroBias()[i], b.gyroBias()[i]) << what;
        EXPECT_EQ(a.accelBias()[i], b.accelBias()[i]) << what;
    }
    for (int i = 0; i < RawGnssEKF<double>::N; ++i)
        for (int j = 0; j < RawGnssEKF<double>::N; ++j)
            EXPECT_EQ(a.P(i, j), b.P(i, j)) << what << " P " << i << "," << j;
}

bool positiveDefinite(const RawGnssEKF<double>& f)
{
    constexpr int N = RawGnssEKF<double>::N;
    double L[N][N] = {};
    for (int i = 0; i < N; ++i)
        for (int j = 0; j <= i; ++j)
        {
            double s = f.P(i, j);
            for (int k = 0; k < j; ++k)
                s -= L[i][k] * L[j][k];
            if (i == j)
            {
                if (!(s > 0.0))
                    return false;
                L[i][i] = std::sqrt(s);
            }
            else
            {
                L[i][j] = s / L[j][j];
            }
        }
    return true;
}

}  // namespace

TEST(RawGnssHdgObs, OffItIsTodaysFilterWhateverItsOtherSettings)
{
    // Mode off: the other hdg_obs settings change nothing, the force is not
    // tracked, and every epoch's share is 1.
    RawGnssEKFParams q = quietParams();
    q.hdg_obs_fh_mps2 = 0.3;
    q.hdg_obs_tau_s = 0.01;
    q.hdg_obs_window_s = 50.0;
    q.hdg_obs_open_g = 100.0;
    q.hdg_obs_floor = 0.2;
    Clock c0, c1;
    Truth t0, t1;
    RawGnssEKF<double> a = pushed(quietParams(), c0, t0);
    RawGnssEKF<double> b = pushed(q, c1, t1);
    const double err[3] = {0.0, 1.5, 0.0};
    const std::vector<std::array<double, 3>> coast(3, std::array<double, 3>{0.0, 0.0, 0.0});
    fly(a, coast, c0, t0, err);
    fly(b, coast, c1, t1, err);
    expectSameState(a, b, "off");
    EXPECT_EQ(b.hdgObsScale(), 1.0);
    EXPECT_EQ(b.hdgObsScaleNow(0.0), 1.0);
    EXPECT_EQ(b.hdgObsForce()[0], 0.0);
    EXPECT_EQ(b.hdgObsOpenT(), -1.0e9);
}

TEST(RawGnssHdgObs, APushThatObservesTheHeadingKeepsTheFullGate)
{
    // Pushed at 3 m/s^2 against a 2 m/s^2 threshold the share is 1 (capped)
    // at every epoch: the filter is today's, bit for bit.
    Clock c0, c1;
    Truth t0, t1;
    RawGnssEKF<double> a = pushed(quietParams(), c0, t0);
    RawGnssEKF<double> b = pushed(forceParams(), c1, t1);
    expectSameState(a, b, "push");
    EXPECT_EQ(b.hdgObsScale(), 1.0);
    EXPECT_NEAR(b.hdgObsForce()[0], 3.0, 0.05);
    EXPECT_NEAR(b.hdgObsForce()[1], 0.0, 0.05);
}

TEST(RawGnssHdgObs, ClosedAVelocityErrorTurnsNoHeading)
{
    // After the push a 1 s vertical boost (3 g: open), then 3 s of coast
    // with no horizontal force and the receiver 1.5 m/s east of the truth (a
    // loop settling), the window closing the heading for 5 s from the boost.
    // Through the push and the boost the filter is today's, bit for bit. In
    // the coast each run is set against its own copy flown on the IMU alone:
    // the full gate turns the heading by degrees through the correlation the
    // push left; closed, the
    // updates leave the heading, the gyro bias along the vertical and the
    // heading's sigma as the IMU alone does (to the second order of the tilt
    // corrections), and the velocity is still the receiver's.
    RawGnssEKFParams pw = quietParams();
    pw.hdg_obs_mode = gnss_raw::kHdgObsWindow;
    pw.hdg_obs_window_s = 5.0;
    Clock c0, c1;
    Truth t0, t1;
    RawGnssEKF<double> a = pushed(quietParams(), c0, t0);
    RawGnssEKF<double> b = pushed(pw, c1, t1);
    const std::vector<std::array<double, 3>> boost(1, std::array<double, 3>{0.0, 0.0, -2.0 * kG});
    fly(a, boost, c0, t0);
    fly(b, boost, c1, t1);
    expectSameState(a, b, "push and boost");
    RawGnssEKF<double> a_imu = a, b_imu = b;
    double e0[3];
    downInBody(b.quat(), e0);
    const double err[3] = {0.0, 1.5, 0.0};
    const std::vector<std::array<double, 3>> coast(3, std::array<double, 3>{0.0, 0.0, 0.0});
    fly(a, coast, c0, t0, err);
    fly(b, coast, c1, t1, err);
    flyNoGnss(a_imu, coast);
    flyNoGnss(b_imu, coast);
    EXPECT_EQ(b.hdgObsScale(), 0.0);
    const double turned_full = headingDiffDeg(a.quat(), a_imu.quat());
    const double turned_obs = headingDiffDeg(b.quat(), b_imu.quat());
    EXPECT_GT(std::fabs(turned_full), 1.0) << "the full gate's turn";
    EXPECT_LT(std::fabs(turned_obs), 0.01) << "closed: " << turned_obs << " against " << turned_full;
    auto along = [&](const RawGnssEKF<double>& f) {
        return e0[0] * f.gyroBias()[0] + e0[1] * f.gyroBias()[1] + e0[2] * f.gyroBias()[2];
    };
    const double dwb_full = along(a) - along(a_imu), dwb_obs = along(b) - along(b_imu);
    // (each update's vertical is its own prior's: the tilt corrections move it
    // a little from e0, a percent or two of the full gate's)
    EXPECT_LT(std::fabs(dwb_obs), 0.03 * std::fabs(dwb_full)) << dwb_obs << " against " << dwb_full;
    double sb[3], sbi[3];
    b.attSigmaNed(sb);
    b_imu.attSigmaNed(sbi);
    EXPECT_NEAR(sb[2], sbi[2], 1.0e-3 * sbi[2]) << "closed: the heading's sigma the IMU's alone";
    EXPECT_NEAR(b.velNed()[1], t1.v[1] + err[1], 0.3);
    EXPECT_TRUE(positiveDefinite(b));
}

TEST(RawGnssHdgObs, TheForcesShareIsTheFiltersOwnHorizontalForce)
{
    // The same coast by the force: the share is the filter's own horizontal
    // specific force -- its tilt and accelerometer bias as it has them, not
    // the truth's zero -- over the threshold, and the heading turns less than
    // the full gate's.
    Clock c0, c1;
    Truth t0, t1;
    RawGnssEKF<double> a = pushed(quietParams(), c0, t0);
    RawGnssEKF<double> b = pushed(forceParams(), c1, t1);
    RawGnssEKF<double> a_imu = a, b_imu = b;
    const double err[3] = {0.0, 1.5, 0.0};
    const std::vector<std::array<double, 3>> coast(3, std::array<double, 3>{0.0, 0.0, 0.0});
    fly(a, coast, c0, t0, err);
    fly(b, coast, c1, t1, err);
    flyNoGnss(a_imu, coast);
    flyNoGnss(b_imu, coast);
    const double fh = std::hypot(b.hdgObsForce()[0], b.hdgObsForce()[1]);
    EXPECT_LT(fh, 1.0);
    // the last epoch's share was taken a step's force before this one's
    EXPECT_NEAR(b.hdgObsScale(), fh / 2.0, 0.01);
    const double turned_full = headingDiffDeg(a.quat(), a_imu.quat());
    const double turned_obs = headingDiffDeg(b.quat(), b_imu.quat());
    EXPECT_LT(std::fabs(turned_obs), 0.6 * std::fabs(turned_full)) << turned_obs << " against " << turned_full;
    EXPECT_TRUE(positiveDefinite(b));
}

TEST(RawGnssHdgObs, AShareScalesTheHeadingsCorrectionAndNothingElse)
{
    // One Doppler, one scalar update, at a share of 0.5 against 1: the gyro
    // bias's correction along the vertical is half, across it and on the
    // velocity, position and clock it is the same; P stays positive definite.
    RawGnssEKFParams p = forceParams();
    p.hdg_obs_tau_s = 2.0;                      // the push's force still in the mean in the coast
    Clock clk;
    Truth tr;
    RawGnssEKF<double> f = pushed(p, clk, tr);
    const std::vector<std::array<double, 3>> coast(1, std::array<double, 3>{0.0, 0.0, 0.0});
    fly(f, coast, clk, tr);
    // a tenth of a second on, then one satellite's Doppler, 0.1 m/s off
    const double still[3] = {0.0, 0.0, -kG}, no_rate[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 20; ++i)
        f.propagate(still, no_rate, kDt);
    const double fh = std::hypot(f.hdgObsForce()[0], f.hdgObsForce()[1]);
    ASSERT_GT(fh, 0.5);
    RawGnssEKF<double> full = f, half = f;
    RawGnssEKFParams pf = f.params(), ph = f.params();
    pf.hdg_obs_mode = gnss_raw::kHdgObsOff;
    ph.hdg_obs_fh_mps2 = 2.0 * fh;
    full.setParams(pf);
    half.setParams(ph);
    const auto sky = makeSky(kLla0);
    double r0[3], dp[3], dv[3], r[3], p_now[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r0);
    for (int k = 0; k < 3; ++k)
        p_now[k] = tr.p[k] + tr.v[k] * 0.1;
    nedToEcef(kLla0[0], kLla0[1], p_now, dp);
    nedToEcef(kLla0[0], kLla0[1], tr.v, dv);
    for (int k = 0; k < 3; ++k)
        r[k] = r0[k] + dp[k];
    Clock c1 = clk;
    c1.tick(0.1);
    std::vector<RawMeas> m = measure(std::vector<Sat>{sky[1]}, r, dv, c1);
    m[0].has_pr = false;
    m[0].rr += 0.1;
    // the vertical the update scales along: the prior's
    double e0[3];
    downInBody(full.quat(), e0);
    double wb0[3], v0[3];
    for (int i = 0; i < 3; ++i)
    {
        wb0[i] = full.gyroBias()[i];
        v0[i] = full.velNed()[i];
    }
    const auto sf = full.updateGnssRaw(m.data(), 1);
    const auto sh = half.updateGnssRaw(m.data(), 1);
    ASSERT_EQ(sf.used_rr, 1);
    ASSERT_EQ(sh.used_rr, 1);
    EXPECT_NEAR(half.hdgObsScale(), 0.5, 1.0e-9);
    double dw_f[3], dw_h[3], vf = 0.0, vh = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        dw_f[i] = full.gyroBias()[i] - wb0[i];
        dw_h[i] = half.gyroBias()[i] - wb0[i];
        vf += e0[i] * dw_f[i];
        vh += e0[i] * dw_h[i];
    }
    ASSERT_GT(std::fabs(vf), 1.0e-9);
    EXPECT_NEAR(vh / vf, 0.5, 1.0e-9);
    for (int i = 0; i < 3; ++i)
    {
        // across the vertical: the same correction
        EXPECT_NEAR(dw_h[i] - e0[i] * vh, dw_f[i] - e0[i] * vf, 1.0e-9 * std::fabs(vf)) << i;
        EXPECT_EQ(half.velNed()[i] - v0[i], full.velNed()[i] - v0[i]) << i;
        EXPECT_EQ(half.lla()[i], full.lla()[i]) << i;
    }
    EXPECT_EQ(half.clockDrift(), full.clockDrift());
    // the heading turned half as far (the quaternion's own nonlinearity aside)
    double qf0[4];
    for (int i = 0; i < 4; ++i)
        qf0[i] = f.quat()[i];
    const double hf = headingDiffDeg(full.quat(), qf0), hh = headingDiffDeg(half.quat(), qf0);
    ASSERT_GT(std::fabs(hf), 1.0e-6);
    EXPECT_NEAR(hh / hf, 0.5, 1.0e-3);
    EXPECT_TRUE(positiveDefinite(half));
}

TEST(RawGnssHdgObs, TheBoostKeepsTheFullGate)
{
    // After the push, a vertical boost (level body, 2 g up: 3 g on the
    // accelerometer) with the receiver 1.5 m/s east: no horizontal force, but
    // past hdg_obs_open_g -- open, today's filter bit for bit. With the open
    // threshold out of reach the heading is held instead.
    Clock c0, c1, c2;
    Truth t0, t1, t2;
    RawGnssEKF<double> a = pushed(quietParams(), c0, t0);
    RawGnssEKF<double> b = pushed(forceParams(), c1, t1);
    // never open, and a threshold far past any force here: closed throughout
    RawGnssEKFParams pc = forceParams();
    pc.hdg_obs_open_g = 100.0;
    pc.hdg_obs_fh_mps2 = 1.0e4;
    RawGnssEKF<double> c = pushed(pc, c2, t2);
    RawGnssEKF<double> a_imu = a, c_imu = c;
    const double err[3] = {0.0, 1.5, 0.0};
    const std::vector<std::array<double, 3>> boost(2, std::array<double, 3>{0.0, 0.0, -2.0 * kG});
    fly(a, boost, c0, t0, err);
    fly(b, boost, c1, t1, err);
    fly(c, boost, c2, t2, err);
    flyNoGnss(a_imu, boost);
    flyNoGnss(c_imu, boost);
    expectSameState(a, b, "boost");
    EXPECT_EQ(b.hdgObsScale(), 1.0);
    EXPECT_GT(std::fabs(headingDiffDeg(a.quat(), a_imu.quat())), 0.5);
    EXPECT_LT(std::fabs(headingDiffDeg(c.quat(), c_imu.quat())), 0.05);
    EXPECT_LT(c.hdgObsScale(), 1.0e-2);
}

TEST(RawGnssHdgObs, EachModesShare)
{
    // The shares as TR_RawGnssEKF.h gives them, on a level body pushed north
    // at 1 m/s^2 with the force's mean settled (tau 0.1 s, 2 s).
    RawGnssEKFParams p = quietParams();
    p.hdg_obs_tau_s = 0.1;
    const double v0[3] = {0.0, 0.0, 0.0};
    const double q0[4] = {1.0, 0.0, 0.0, 0.0};
    const double push[3] = {1.0, 0.0, -kG}, no_rate[3] = {0.0, 0.0, 0.0};
    auto settled = [&](const RawGnssEKFParams& pp) {
        RawGnssEKF<double> f(pp);
        f.init(kLla0, v0, q0, 0.0);
        f.setQuaternion(q0);
        f.setHeadingSigma(10.0 * kD2R);
        for (int i = 0; i < 400; ++i)
            f.propagate(push, no_rate, kDt);
        return f;
    };
    // the force: |f_h| over the threshold, at most 1, at least the floor
    p.hdg_obs_mode = gnss_raw::kHdgObsForce;
    p.hdg_obs_fh_mps2 = 2.0;
    EXPECT_NEAR(settled(p).hdgObsScaleNow(0.0), 0.5, 1.0e-6);
    p.hdg_obs_fh_mps2 = 0.5;
    EXPECT_EQ(settled(p).hdgObsScaleNow(0.0), 1.0);
    p.hdg_obs_fh_mps2 = 10.0;
    EXPECT_NEAR(settled(p).hdgObsScaleNow(0.0), 0.1, 1.0e-6);
    p.hdg_obs_floor = 0.3;
    EXPECT_EQ(settled(p).hdgObsScaleNow(0.0), 0.3);
    p.hdg_obs_floor = 0.0;
    // not applied (the shadow's scope): open
    {
        RawGnssEKF<double> f = settled(p);
        f.setHdgObsOn(false);
        EXPECT_EQ(f.hdgObsScaleNow(0.0), 1.0);
    }
    // the information: x = |f_h| tau sigma_psi / sigma_v, s = x^2 / (1 + x^2)
    p.hdg_obs_mode = gnss_raw::kHdgObsInfo;
    p.hdg_obs_tau_s = 0.1;
    p.hdg_obs_sigma_v = 0.02;
    {
        RawGnssEKF<double> f = settled(p);
        double sig[3];
        f.attSigmaNed(sig);
        const double fh = std::hypot(f.hdgObsForce()[0], f.hdgObsForce()[1]);
        EXPECT_NEAR(fh, 1.0, 1.0e-6);
        const double x = fh * 0.1 * sig[2] / 0.02;
        EXPECT_NEAR(f.hdgObsScaleNow(0.0), x * x / (1.0 + x * x), 1.0e-12);
        EXPECT_GT(x, 0.5);
        EXPECT_LT(x, 2.0);
        // a heading twice as unsure takes more of it
        f.setHeadingSigma(2.0 * sig[2]);
        const double x2 = 2.0 * x;
        EXPECT_NEAR(f.hdgObsScaleNow(0.0), x2 * x2 / (1.0 + x2 * x2), 1.0e-9);
    }
    // the window: open on the pad, closed for its length after a boost (past
    // the hold), open after
    p.hdg_obs_mode = gnss_raw::kHdgObsWindow;
    p.hdg_obs_window_s = 3.0;
    p.hdg_obs_open_hold_s = 0.2;
    {
        RawGnssEKF<double> f = settled(p);
        EXPECT_EQ(f.hdgObsScaleNow(0.0), 1.0);           // no boost yet
        const double boost[3] = {0.0, 0.0, -3.0 * kG};
        for (int i = 0; i < 200; ++i)
            f.propagate(boost, no_rate, kDt);
        const double t_b = f.hdgObsOpenT();
        EXPECT_NEAR(t_b, f.time(), 1.0e-9);
        EXPECT_EQ(f.hdgObsScaleNow(0.0), 1.0);           // in it
        const double coast[3] = {0.0, 0.0, -0.3 * kG};
        for (int i = 0; i < 60; ++i)                      // 0.3 s on
            f.propagate(coast, no_rate, kDt);
        EXPECT_EQ(f.hdgObsScaleNow(0.0), 0.0);
        EXPECT_EQ(f.hdgObsScaleNow(0.2), 1.0);           // an epoch from within the hold
        for (int i = 0; i < 560; ++i)                     // 3.1 s on
            f.propagate(coast, no_rate, kDt);
        EXPECT_EQ(f.hdgObsScaleNow(0.0), 0.0);
        for (int i = 0; i < 40; ++i)                      // 3.3 s on
            f.propagate(coast, no_rate, kDt);
        EXPECT_EQ(f.hdgObsScaleNow(0.0), 1.0);
    }
}

TEST(RawGnssHdgObs, TheUnaidedTwinIsTheSameWithOrWithout)
{
    // att_gnss_off masks the attitude and the gyro bias from every GNSS row:
    // the mode has nothing left to scale, so the twin the acceptance compares
    // against is the same filter, bit for bit.
    RawGnssEKFParams p0 = quietParams(), p1 = forceParams();
    p0.att_gnss_off = p1.att_gnss_off = true;
    Clock c0, c1;
    Truth t0, t1;
    RawGnssEKF<double> a = pushed(p0, c0, t0);
    RawGnssEKF<double> b = pushed(p1, c1, t1);
    const double err[3] = {0.0, 1.5, 0.0};
    const std::vector<std::array<double, 3>> coast(3, std::array<double, 3>{0.0, 0.0, 0.0});
    fly(a, coast, c0, t0, err);
    fly(b, coast, c1, t1, err);
    expectSameState(a, b, "unaided");
}

TEST(RawGnssHdgObs, TheShadowAppliesItWhereItsScopeSays)
{
    // A navigation filter on a level, still pad (no horizontal force: the
    // force's share 0 wherever it applies). Everywhere: on the pad too. In
    // flight: not on the pad, from the launch edge, back off on the ground.
    // Launch to the apogee call: off again from the call.
    using gnss_raw::kShadowApogee;
    using gnss_raw::kShadowInFlight;
    using gnss_raw::kShadowLevel;
    for (uint8_t scope : {RawGnssShadow::kObsEverywhere, RawGnssShadow::kObsFlight, RawGnssShadow::kObsAscent})
    {
        RawGnssShadow::Config c;
        c.primary = true;
        c.min_step_s = 0.01;
        c.ekf.hdg_obs_mode = gnss_raw::kHdgObsForce;
        c.heading_obs_scope = scope;
        RawGnssShadow sh(c);
        const int64_t t0 = 5000000000LL;
        gnss_raw::ShadowSeed s{};
        s.time_us = static_cast<uint32_t>(t0);
        s.lla[0] = kLla0[0];
        s.lla[1] = kLla0[1];
        s.lla[2] = kLla0[2];
        s.quat[0] = 1.0f;
        s.on_pad = 1;
        sh.onSeed(s);
        int64_t t = t0;
        auto feed = [&](double secs, uint8_t flags) {
            for (const int64_t t1 = t + static_cast<int64_t>(secs * 1e6); t < t1; t += 5000)
            {
                gnss_raw::ShadowImu m{};
                m.time_us = static_cast<uint32_t>(t);
                m.acc[2] = static_cast<float>(-kG);
                m.flags = flags;
                sh.onImu(m, t);
            }
        };
        feed(2.0, kShadowLevel);
        ASSERT_TRUE(sh.running());
        const bool pad_on = scope == RawGnssShadow::kObsEverywhere;
        EXPECT_EQ(sh.filter().hdgObsScaleNow(0.0), pad_on ? 0.0f : 1.0f) << "pad, scope " << int(scope);
        feed(1.0, kShadowInFlight);
        EXPECT_EQ(sh.filter().hdgObsScaleNow(0.0), 0.0f) << "flight, scope " << int(scope);
        feed(1.0, kShadowInFlight | kShadowApogee);
        EXPECT_EQ(sh.filter().hdgObsScaleNow(0.0), scope == RawGnssShadow::kObsAscent ? 1.0f : 0.0f)
            << "after the call, scope " << int(scope);
        feed(1.0, kShadowLevel);
        EXPECT_EQ(sh.filter().hdgObsScaleNow(0.0), pad_on ? 0.0f : 1.0f) << "ground, scope " << int(scope);
    }
}
