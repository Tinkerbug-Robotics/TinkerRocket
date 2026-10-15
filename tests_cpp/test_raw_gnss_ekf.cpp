// Host tests for TR_RawGnssEKF, the tightly coupled GNSS/INS filter (#1600),
// on a synthetic sky: ten satellites (five GPS, three Galileo, two BeiDou)
// around a receiver near Spaceport America, their pseudoranges and range
// rates made with the same Earth-rotation model the filter predicts with, a
// receiver clock with a bias, a drift and Galileo and BeiDou offsets. (That
// the port reproduces the bench prototype on the bench flights is shown by the
// gnss repo's tc_ekf_cocom.py, mode ins-cpp, through tinkerrocket_sim._rawgnss.)
#include <gtest/gtest.h>

#include <array>

#include <cmath>
#include <vector>

#include "RawMeas.h"
#include "TR_RawGnssEKF.h"
#include "gnss/atmo.h"
#include "gnss/types.h"

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
constexpr double kDt = 0.005;     // a 200 Hz IMU

struct Sat
{
    int sys, prn;
    double pos[3], vel[3], acc[3];
};

// The receiver clock: its bias runs at its drift.
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
        // a circular orbit's speed, at right angles to the radius
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

// Range and range rate, the satellite turned by the Earth's rotation over the travel time.
void rangeAndRate(const double rs[3], const double vs[3], const double r[3], const double v[3],
                  double& rho, double& rate)
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

// The receiver at r, v (ECEF) measures the sky. rr_lag: its range rates are
// those of rr_lag seconds before (the receiver and the satellites then).
std::vector<RawMeas> measure(const std::vector<Sat>& sky, const double r[3], const double v[3], const Clock& clk,
                             double a_ecef[3] = nullptr, double rr_lag = 0.0)
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
        if (rr_lag > 0.0)
        {
            const double T = rr_lag;
            double rs[3], vs[3], rl[3], vl[3];
            for (int k = 0; k < 3; ++k)
            {
                rs[k] = s.pos[k] - s.vel[k] * T + 0.5 * s.acc[k] * T * T;
                vs[k] = s.vel[k] - s.acc[k] * T;
                const double a = a_ecef ? a_ecef[k] : 0.0;
                rl[k] = r[k] - v[k] * T + 0.5 * a * T * T;
                vl[k] = v[k] - a * T;
            }
            double rho_l;
            rangeAndRate(rs, vs, rl, vl, rho_l, rate);
        }
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
    p.rr_lag_s = 0.0;          // this sky reports the range rate on time
    p.clk_drift_psd = 0.036;   // a free-running TCXO, not the LC86G's steered clock
    return p;
}

template <typename T>
void initLevel(RawGnssEKF<T>& f, const double lla[3], const double v_ned[3])
{
    const T v[3] = {T(v_ned[0]), T(v_ned[1]), T(v_ned[2])};
    const T q[4] = {T(1), T(0), T(0), T(0)};          // body = NED
    f.init(lla, v, q, 0.0);
    f.setQuaternion(q);
}

// IMU steps up to the next whole second: a level body that feels only its acceleration (NED) and gravity.
template <typename T>
void imuSecond(RawGnssEKF<T>& f, const double a_ned[3] = nullptr, int steps = 200)
{
    const T acc[3] = {T(a_ned ? a_ned[0] : 0.0), T(a_ned ? a_ned[1] : 0.0), T((a_ned ? a_ned[2] : 0.0) - kG)};
    const T gyro[3] = {T(0), T(0), T(0)};
    for (int i = 0; i < steps; ++i)
        f.propagate(acc, gyro, T(kDt));
}

template <typename T>
void ecefOf(const RawGnssEKF<T>& f, double r[3])
{
    double v[3];
    f.receiverEcef(r, v);
}

double dist(const double a[3], const double b[3])
{
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

template <typename T>
void expectCovarianceHealthy(const RawGnssEKF<T>& f)
{
    for (int i = 0; i < RawGnssEKF<T>::N; ++i)
    {
        EXPECT_TRUE(std::isfinite(f.P(i, i)));
        EXPECT_GT(f.P(i, i), T(0));
        for (int j = 0; j < i; ++j)
        {
            EXPECT_EQ(f.P(i, j), f.P(j, i));
            EXPECT_LE(f.P(i, j) * f.P(i, j), T(1.0001) * f.P(i, i) * f.P(j, j));
        }
    }
}

// A filter started off the truth (static receiver) and given n epochs.
template <typename T>
RawGnssEKF<T> converged(int epochs, Clock& clk, RawGnssEKFParams p = quietParams())
{
    p.p0_pos = 30.0;
    RawGnssEKF<T> f(p);
    double lla[3] = {kLla0[0] + 20.0 / 6.37e6, kLla0[1] - 10.0 / 5.35e6, kLla0[2] - 15.0};
    const double v0[3] = {0.0, 0.0, 0.0};
    initLevel(f, lla, v0);
    const auto sky = makeSky(kLla0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    const double v[3] = {0.0, 0.0, 0.0};
    for (int e = 0; e < epochs; ++e)
    {
        imuSecond(f);
        clk.tick(1.0);
        const auto m = measure(sky, r, v, clk);
        f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    }
    return f;
}

}  // namespace

TEST(RawGnssEKF, StaysPutOnThePad)
{
    RawGnssEKF<float> f(quietParams());
    const double v0[3] = {0.0, 0.0, 0.0};
    initLevel(f, kLla0, v0);
    const float acc[3] = {0.0f, 0.0f, float(-kG)};
    const float gyro[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 2000; ++i)          // 10 s, levelled every step as the flight computer does
    {
        f.propagate(acc, gyro, float(kDt));
        f.updateAccelLevel(acc);
    }
    EXPECT_NEAR(f.time(), 10.0, 1e-6);
    EXPECT_NEAR(f.lla()[0], kLla0[0], 1e-12);
    EXPECT_NEAR(f.lla()[2], kLla0[2], 1e-6);
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(f.velNed()[i], 0.0f, 1e-6f);
    EXPECT_NEAR(f.quat()[0], 1.0f, 1e-6f);
    EXPECT_GT(f.sigma(0), 10.0f);           // nothing has seen the position
    expectCovarianceHealthy(f);
}

TEST(RawGnssEKF, TheFirstEpochSetsTheClockAndTheSystemOffsets)
{
    RawGnssEKF<double> f(quietParams());
    const double v0[3] = {0.0, 0.0, 0.0};
    initLevel(f, kLla0, v0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    const Clock clk;
    const auto m = measure(makeSky(kLla0), r, v0, clk);
    EXPECT_FALSE(f.clockReady());
    ASSERT_TRUE(f.initClockFrom(m.data(), static_cast<int>(m.size())));
    EXPECT_TRUE(f.clockReady());
    EXPECT_NEAR(f.clockBias(), clk.bias, 1e-3);
    EXPECT_NEAR(f.clockDrift(), clk.drift, 1e-6);
    EXPECT_NEAR(f.isb(GNSS_SYS_GAL), clk.isb_gal, 1e-3);
    EXPECT_NEAR(f.isb(GNSS_SYS_BDS), clk.isb_bds, 1e-3);
    EXPECT_NEAR(f.sigma(RawGnssEKF<double>::kClkBias), std::sqrt(30.0 * 30.0 + 10.0 * 10.0), 1e-9);
    // no pseudorange at all: nothing to start from
    std::vector<RawMeas> none = m;
    for (auto& x : none)
        x.has_pr = false;
    RawGnssEKF<double> g(quietParams());
    initLevel(g, kLla0, v0);
    EXPECT_FALSE(g.initClockFrom(none.data(), static_cast<int>(none.size())));
}

TEST(RawGnssEKF, PullsAWrongStartToTheSatellites)
{
    Clock clk;
    RawGnssEKF<double> f = converged<double>(20, clk);
    double r[3], r0[3];
    ecefOf(f, r);
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r0);
    EXPECT_LT(dist(r, r0), 0.3);
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(f.velNed()[i], 0.0, 0.02);
    EXPECT_NEAR(f.clockBias(), clk.bias, 0.5);
    EXPECT_NEAR(f.isb(GNSS_SYS_GAL), clk.isb_gal, 0.5);
    EXPECT_NEAR(f.isb(GNSS_SYS_BDS), clk.isb_bds, 0.5);
    EXPECT_LT(f.sigma(0), 2.0);
    expectCovarianceHealthy(f);
}

TEST(RawGnssEKF, AnUpdateLeavesTheKalmanCovariance)
{
    // One pseudorange, h = [-u_ned, 0..., 1 (clock bias), 0...]: the Joseph
    // form with the optimal gain must leave P - (P h)(P h)' / (h' P h + R).
    Clock clk;
    RawGnssEKF<double> f = converged<double>(5, clk);
    constexpr int N = RawGnssEKF<double>::N;
    imuSecond(f);
    clk.tick(1.0);
    double P0[N][N];
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            P0[i][j] = f.P(i, j);
    double r[3], v[3], r0[3];
    f.receiverEcef(r, v);
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r0);
    const double still[3] = {0.0, 0.0, 0.0};
    auto m = measure(makeSky(kLla0), r0, still, clk);
    m.resize(1);                              // GPS PRN 3 alone
    m[0].has_rr = false;
    // its line of sight from where the filter is, turned by the Earth's rotation
    const double* rs = m[0].sat_pos;
    double rho = dist(rs, r), x[3] = {rs[0], rs[1], rs[2]};
    for (int it = 0; it < 2; ++it)
    {
        const double th = kOmegaE * rho / kC;
        x[0] = std::cos(th) * rs[0] + std::sin(th) * rs[1];
        x[1] = -std::sin(th) * rs[0] + std::cos(th) * rs[1];
        rho = dist(x, r);
    }
    const double lat = f.lla()[0], lon = f.lla()[1];
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    const double u[3] = {(x[0] - r[0]) / rho, (x[1] - r[1]) / rho, (x[2] - r[2]) / rho};
    double h[N] = {};
    h[0] = -(-sl * co * u[0] - sl * so * u[1] + cl * u[2]);
    h[1] = -(-so * u[0] + co * u[1]);
    h[2] = -(-cl * co * u[0] - cl * so * u[1] - sl * u[2]);
    h[RawGnssEKF<double>::kClkBias] = 1.0;
    const RawGnssStats st = f.updateGnssRaw(m.data(), 1);
    ASSERT_EQ(st.used_pr, 1);
    double Ph[N], hPh = 0.0;
    for (int i = 0; i < N; ++i)
    {
        Ph[i] = 0.0;
        for (int j = 0; j < N; ++j)
            Ph[i] += P0[i][j] * h[j];
        hPh += h[i] * Ph[i];
    }
    const double S = hPh + 1.0;               // R = sigma_pr^2
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
        {
            const double expect = P0[i][j] - Ph[i] * Ph[j] / S;
            EXPECT_NEAR(f.P(i, j), expect, 1e-9 * (std::fabs(P0[i][i] * P0[j][j]) > 0 ? std::sqrt(P0[i][i] * P0[j][j]) : 1.0))
                << "P(" << i << ", " << j << ")";
        }
}

TEST(RawGnssEKF, FloatTracksDouble)
{
    Clock cd, cs;
    RawGnssEKF<double> d = converged<double>(20, cd);
    RawGnssEKF<float> s = converged<float>(20, cs);
    double rd[3], rs[3];
    ecefOf(d, rd);
    ecefOf(s, rs);
    EXPECT_LT(dist(rd, rs), 0.01);
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(double(s.velNed()[i]), d.velNed()[i], 1e-3);
    EXPECT_NEAR(s.clockBias(), d.clockBias(), 0.01);
    EXPECT_NEAR(double(s.clockDrift()), d.clockDrift(), 1e-3);
    expectCovarianceHealthy(s);
}

TEST(RawGnssEKF, TakesAClockStepEverySatelliteAgreesOn)
{
    Clock clk;
    RawGnssEKF<double> f = converged<double>(10, clk);
    const auto sky = makeSky(kLla0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    const double v[3] = {0.0, 0.0, 0.0};

    imuSecond(f);
    clk.tick(1.0);
    clk.bias += 5000.0;                       // the LC86G steering its clock to its first fix
    auto m = measure(sky, r, v, clk);
    RawGnssStats st = f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    EXPECT_NEAR(st.clock_step_m, 5000.0, 1.0);
    EXPECT_EQ(st.clock_jump_ms, 0);
    EXPECT_EQ(st.used_pr, 10);
    EXPECT_EQ(st.rejected_pr, 0);

    imuSecond(f);
    clk.tick(1.0);
    clk.bias += 2.0 * kC * 1e-3;              // a receiver that steps whole milliseconds
    clk.drift += 30.0;                        // and its frequency
    m = measure(sky, r, v, clk);
    st = f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    EXPECT_EQ(st.clock_jump_ms, 2);
    EXPECT_NEAR(st.drift_step_mps, 30.0, 0.2);
    EXPECT_EQ(st.used_pr + st.used_rr, 20);
    EXPECT_NEAR(f.clockBias(), clk.bias, 1.0);
    EXPECT_NEAR(f.clockDrift(), clk.drift, 0.05);
}

TEST(RawGnssEKF, RefusesOneBadSatelliteThenLetsItBackIn)
{
    Clock clk;
    RawGnssEKF<double> f = converged<double>(10, clk);
    const auto sky = makeSky(kLla0);
    double r[3], r0[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    const double v[3] = {0.0, 0.0, 0.0};
    for (int e = 0; e < 4; ++e)
    {
        imuSecond(f);
        clk.tick(1.0);
        auto m = measure(sky, r, v, clk);
        m[3].pr += 200.0;                     // multipath gone wild on one satellite
        const RawGnssStats st = f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
        if (e < 3)
        {
            EXPECT_EQ(st.rejected_pr, 1) << "epoch " << e;
            EXPECT_EQ(st.used_pr, 9);
        }
        else
        {
            // refused three epochs running: let in, its variance inflated
            // until it would have just passed the gate on R alone
            EXPECT_EQ(st.rejected_pr, 0);
            EXPECT_EQ(st.used_pr, 10);
            EXPECT_GE(st.nis_max, 10.83f);
            EXPECT_LT(st.nis_max, 20.0f);
        }
        EXPECT_EQ(st.used_rr, 10);
    }
    ecefOf(f, r0);
    EXPECT_LT(dist(r0, r), 5.0);              // let in once, inflated: metres, not 200
}

TEST(RawGnssEKF, AppliesALateEpochAtItsOwnTime)
{
    // A receiver climbing at constant velocity; its epochs reach the filter
    // 0.2 s late. Truth comes from a filter given perfect IMU and no updates.
    const double vel[3] = {30.0, -20.0, -50.0};
    RawGnssEKFParams p = quietParams();
    RawGnssEKF<double> truth(p), on_time(p), late(p), naive(p);
    initLevel(truth, kLla0, vel);
    const double v_err[3] = {vel[0] + 0.5, vel[1] - 0.5, vel[2] + 0.5};
    double lla_err[3] = {kLla0[0] + 8.0 / 6.37e6, kLla0[1], kLla0[2] + 5.0};
    initLevel(on_time, lla_err, v_err);
    initLevel(late, lla_err, v_err);
    initLevel(naive, lla_err, v_err);
    const auto sky = makeSky(kLla0);
    Clock clk;
    imuSecond(truth);
    imuSecond(on_time);
    imuSecond(late);
    imuSecond(naive);
    clk.tick(1.0);
    for (int e = 0; e < 12; ++e)
    {
        double r[3], v[3];
        truth.receiverEcef(r, v);
        const auto m = measure(sky, r, v, clk);
        on_time.updateGnssRaw(m.data(), static_cast<int>(m.size()));
        imuSecond(late, nullptr, 40);         // 0.2 s on
        late.updateGnssRaw(m.data(), static_cast<int>(m.size()), 0.2);
        imuSecond(naive, nullptr, 40);
        naive.updateGnssRaw(m.data(), static_cast<int>(m.size()));   // as if it were now
        imuSecond(truth, nullptr, 40);
        imuSecond(on_time, nullptr, 40);
        imuSecond(truth, nullptr, 160);       // to the next whole second
        imuSecond(on_time, nullptr, 160);
        imuSecond(late, nullptr, 160);
        imuSecond(naive, nullptr, 160);
        clk.tick(1.0);
    }
    double rt[3], ro[3], rl[3], rn[3];
    ecefOf(truth, rt);
    ecefOf(on_time, ro);
    ecefOf(late, rl);
    ecefOf(naive, rn);
    EXPECT_LT(dist(ro, rt), 0.5);
    EXPECT_LT(dist(rl, rt), 0.5);
    EXPECT_LT(dist(rl, ro), 0.3);
    EXPECT_GT(dist(rn, rt), 3.0);             // taking it as now costs v * 0.2 s
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(late.velNed()[i], truth.velNed()[i], 0.05);
}

TEST(RawGnssEKF, AnEpochInPartsAtOneAgeIsTheEpochAtOnce)
{
    // beginGnssRaw, then updateGnssRawPart over the satellites in two parts,
    // all at one age: updateGnssRaw, to rounding (each part takes the epoch
    // back with the velocity the parts before it left, and gates the attitude
    // with cos^4 pitch at the attitude they left: a velocity error now tilts
    // the attitude, 0.6 deg here, and the gate moves at its square, so the
    // velocity differs by ~1e-8 m/s)
    const double vel[3] = {30.0, -20.0, -50.0};
    RawGnssEKFParams p = quietParams();
    RawGnssEKF<double> truth(p), whole(p), parts(p);
    initLevel(truth, kLla0, vel);
    const double v_err[3] = {vel[0] + 0.5, vel[1] - 0.5, vel[2] + 0.5};
    double lla_err[3] = {kLla0[0] + 8.0 / 6.37e6, kLla0[1], kLla0[2] + 5.0};
    initLevel(whole, lla_err, v_err);
    initLevel(parts, lla_err, v_err);
    const auto sky = makeSky(kLla0);
    Clock clk;
    imuSecond(truth);
    imuSecond(whole);
    imuSecond(parts);
    clk.tick(1.0);
    for (int e = 0; e < 4; ++e)
    {
        double r[3], v[3];
        truth.receiverEcef(r, v);
        const auto m = measure(sky, r, v, clk);
        const int n = static_cast<int>(m.size());
        const RawGnssStats sw = whole.updateGnssRaw(m.data(), n, 0.1);
        RawGnssStats sp;
        ASSERT_TRUE(parts.beginGnssRaw(m.data(), n, 0.1, sp));
        parts.updateGnssRawPart(m.data(), 0, n / 2, 0.1, sp);
        parts.updateGnssRawPart(m.data(), n / 2, n, 0.1, sp);
        EXPECT_EQ(sp.used_pr, sw.used_pr);
        EXPECT_EQ(sp.used_rr, sw.used_rr);
        double rp[3], rw[3];
        ecefOf(parts, rp);
        ecefOf(whole, rw);
        EXPECT_LT(dist(rp, rw), 1e-6);
        for (int i = 0; i < 3; ++i)
            EXPECT_NEAR(parts.velNed()[i], whole.velNed()[i], 1e-7);
        imuSecond(truth);
        imuSecond(whole);
        imuSecond(parts);
        clk.tick(1.0);
    }
}

TEST(RawGnssEKF, AnEpochFusedInPartsBetweenStepsLandsAtItsOwnTime)
{
    // The navigation filter fuses a sky a few satellites at a time between its
    // steps (RawGnssShadow's epoch job), each part at the age the epoch has by
    // then. On the climbing receiver above it lands at the truth, and where
    // the epoch fused at once lands.
    const double vel[3] = {30.0, -20.0, -50.0};
    RawGnssEKFParams p = quietParams();
    RawGnssEKF<double> truth(p), whole(p), parts(p);
    initLevel(truth, kLla0, vel);
    const double v_err[3] = {vel[0] + 0.5, vel[1] - 0.5, vel[2] + 0.5};
    double lla_err[3] = {kLla0[0] + 8.0 / 6.37e6, kLla0[1], kLla0[2] + 5.0};
    initLevel(whole, lla_err, v_err);
    initLevel(parts, lla_err, v_err);
    const auto sky = makeSky(kLla0);
    Clock clk;
    imuSecond(truth);
    imuSecond(whole);
    imuSecond(parts);
    clk.tick(1.0);
    for (int e = 0; e < 12; ++e)
    {
        double r[3], v[3];
        truth.receiverEcef(r, v);
        const auto m = measure(sky, r, v, clk);
        const int n = static_cast<int>(m.size());
        imuSecond(whole, nullptr, 4);         // 20 ms on
        whole.updateGnssRaw(m.data(), n, 0.02);
        imuSecond(parts, nullptr, 4);
        double age = 0.02;
        RawGnssStats st;
        ASSERT_TRUE(parts.beginGnssRaw(m.data(), n, age, st));
        int samples = 4;
        for (int i = 0; i < n; i += 2)
        {
            imuSecond(parts, nullptr, 2);     // a step (10 ms) between parts
            samples += 2;
            age += 0.01;
            parts.updateGnssRawPart(m.data(), i, std::min(i + 2, n), age, st);
        }
        EXPECT_GE(st.used_pr, n - 1);
        imuSecond(truth, nullptr, 200);       // to the next whole second
        imuSecond(whole, nullptr, 196);
        imuSecond(parts, nullptr, 200 - samples);
        clk.tick(1.0);
    }
    double rt[3], rw[3], rp[3];
    ecefOf(truth, rt);
    ecefOf(whole, rw);
    ecefOf(parts, rp);
    EXPECT_LT(dist(rw, rt), 0.5);
    EXPECT_LT(dist(rp, rt), 0.5);
    EXPECT_LT(dist(rp, rw), 0.3);
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(parts.velNed()[i], truth.velNed()[i], 0.05);
}

TEST(RawGnssEKF, PredictsADopplerThatLagsItsTag)
{
    // 2 g of upward acceleration; the receiver reports the range rate 55.5 ms
    // before each epoch's tag, as the LC86G does.
    const double a_ned[3] = {0.0, 0.0, -20.0};
    const double lag = 0.0555;
    RawGnssEKFParams with_lag = quietParams(), without = quietParams();
    with_lag.rr_lag_s = lag;
    RawGnssEKF<double> truth(with_lag), f(with_lag), g(without);
    const double v0[3] = {0.0, 0.0, 0.0};
    initLevel(truth, kLla0, v0);
    initLevel(f, kLla0, v0);
    initLevel(g, kLla0, v0);
    const auto sky = makeSky(kLla0);
    Clock clk;
    double worst_f = 0.0, worst_g = 0.0;
    const double still[3] = {0.0, 0.0, 0.0};
    for (int e = 0; e < 8; ++e)
    {
        const double* acc = e == 0 ? still : a_ned;   // the first epoch at rest: the clock starts exact
        imuSecond(truth, acc);
        imuSecond(f, acc);
        imuSecond(g, acc);
        clk.tick(1.0);
        double r[3], v[3], a[3];
        truth.receiverEcef(r, v);
        nedToEcef(truth.lla()[0], truth.lla()[1], acc, a);
        const auto m = measure(sky, r, v, clk, a, lag);
        f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
        g.updateGnssRaw(m.data(), static_cast<int>(m.size()));
        if (e >= 2)
        {
            worst_f = std::fmax(worst_f, std::fabs(f.velNed()[2] - truth.velNed()[2]));
            worst_g = std::fmax(worst_g, std::fabs(g.velNed()[2] - truth.velNed()[2]));
        }
    }
    EXPECT_LT(worst_f, 0.05);                 // the lag modelled
    EXPECT_GT(worst_g, 0.3);                  // ignored: the Doppler drags the velocity back 20 * 0.0555 m/s
}

// ---- the barometer and zero velocity (#1600: the shadow's aiding without satellites)

TEST(RawGnssEKF, AHeightHoldsTheVerticalAnIMUWouldLose)
{
    // An accelerometer 0.3 m/s^2 strong in the vertical for 20 s: alone it
    // climbs 60 m; a height at the true one every 0.1 s (2 m sigma) holds it
    // within a couple of metres
    for (bool with_height : {false, true})
    {
        RawGnssEKF<float> f(quietParams());
        const double v0[3] = {0.0, 0.0, 0.0};
        initLevel(f, kLla0, v0);
        const double a_up[3] = {0.0, 0.0, -0.3};   // NED: up
        for (int s = 0; s < 200; ++s)
        {
            imuSecond(f, a_up, 20);                 // 0.1 s
            if (with_height)
                f.updateHeight(kLla0[2], 4.0f);
        }
        if (with_height)
        {
            EXPECT_NEAR(f.lla()[2], kLla0[2], 3.0);
            EXPECT_NEAR(f.velNed()[2], 0.0f, 0.5f);
        }
        else
        {
            EXPECT_GT(f.lla()[2] - kLla0[2], 55.0);
        }
        expectCovarianceHealthy(f);
    }
}

TEST(RawGnssEKF, ZeroVelocityMovesOnlyPositionAndVelocity)
{
    RawGnssEKF<float> f(quietParams());
    const double v0[3] = {3.0, -2.0, 1.0};
    initLevel(f, kLla0, v0);
    imuSecond(f);                                    // 1 s at that velocity: the attitude and biases correlate
    const float q0[4] = {f.quat()[0], f.quat()[1], f.quat()[2], f.quat()[3]};
    const float ab0[3] = {f.accelBias()[0], f.accelBias()[1], f.accelBias()[2]};
    const float wb0[3] = {f.gyroBias()[0], f.gyroBias()[1], f.gyroBias()[2]};
    for (int k = 0; k < 30; ++k)                     // 3 s at 10 Hz, still
    {
        imuSecond(f, nullptr, 20);
        f.updateZeroVelocity(0.5f);
    }
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(f.velNed()[i], 0.0f, 0.1f);
        EXPECT_FLOAT_EQ(f.accelBias()[i], ab0[i]);
        EXPECT_FLOAT_EQ(f.gyroBias()[i], wb0[i]);
    }
    for (int i = 0; i < 4; ++i)
        EXPECT_FLOAT_EQ(f.quat()[i], q0[i]);
    expectCovarianceHealthy(f);
}

// ---- a tilt's velocity: the specific force's Jacobian (#1626 E1)

namespace
{

// A level filter at rest whose tilt has the 1-sigma s (the half-angle state:
// a rotation of 2 s) about north and east, nothing shared with the rest.
RawGnssEKF<double> tiltSigma(double s)
{
    RawGnssEKFParams p = quietParams();
    p.p0_att = s;
    RawGnssEKF<double> f(p);
    const double v0[3] = {0.0, 0.0, 0.0}, q[4] = {1.0, 0.0, 0.0, 0.0};
    f.init(kLla0, v0, q, 0.0);
    return f;
}

void stepBoth(RawGnssEKF<double>& a, RawGnssEKF<double>& b, const double acc[3], int steps)
{
    const double gyro[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < steps; ++i)
    {
        a.propagate(acc, gyro, kDt);
        b.propagate(acc, gyro, kDt);
    }
}

}  // namespace

TEST(RawGnssEKF, OnThePadATiltDrivesTheHorizontalVelocity)
{
    // Still and level, the accelerometer reads g up: a tilt theta turns g theta
    // of it into a horizontal acceleration, v' = Tb2n f + g. So against a filter
    // whose tilt is known, a tilt sigma s gives the velocity a cross-covariance
    // F_va s^2 t with it and F_va s^2 F_va' t^2 more variance, F_va = -2 Tb2n
    // [f x]: -2 g between the turn about y and north, 2 g between the turn
    // about x and east, nothing to the vertical. (Built from the kinematic
    // acceleration, 0 here, the filter saw none of it.)
    using F = RawGnssEKF<double>;
    const double s = 0.02;                               // 2.3 deg of tilt
    F known = tiltSigma(1.0e-3), tilted = tiltSigma(s);
    const double acc[3] = {0.0, 0.0, -kG};
    stepBoth(known, tilted, acc, 200);                   // 1 s
    const double t = 1.0, ds2 = s * s - 1.0e-6, g2 = 2.0 * kG;
    auto d = [&](int i, int j) { return tilted.P(i, j) - known.P(i, j); };
    EXPECT_NEAR(d(F::kVel, F::kAtt + 1), -g2 * ds2 * t, 1.0e-9);
    EXPECT_NEAR(d(F::kVel + 1, F::kAtt), g2 * ds2 * t, 1.0e-9);
    EXPECT_NEAR(d(F::kVel, F::kAtt), 0.0, 1.0e-12);
    EXPECT_NEAR(d(F::kVel + 1, F::kAtt + 1), 0.0, 1.0e-12);
    for (int k = 0; k < 3; ++k)
        EXPECT_NEAR(d(F::kVel + 2, F::kAtt + k), 0.0, 1.0e-12) << k;
    EXPECT_NEAR(d(F::kVel, F::kVel), g2 * g2 * ds2 * t * t, 1.0e-9);     // 0.15 (m/s)^2
    EXPECT_NEAR(d(F::kVel + 1, F::kVel + 1), g2 * g2 * ds2 * t * t, 1.0e-9);
    EXPECT_NEAR(d(F::kVel + 2, F::kVel + 2), 0.0, 1.0e-12);
    EXPECT_GT(d(F::kVel, F::kVel), 0.1);
    expectCovarianceHealthy(tilted);
}

TEST(RawGnssEKF, InFreeFallATiltDrivesNoVelocity)
{
    // Falling with nothing pushing (a drag-free coast), the accelerometer reads
    // 0: whatever the tilt, the velocity is gravity's alone. A tilt sigma builds
    // no velocity-attitude cross-covariance, and the velocity's variance grows
    // as a known tilt's does. (The kinematic acceleration's block, -2 Tb2n
    // [(f + Tn2b g) x], built 2 g of it here.)
    using F = RawGnssEKF<double>;
    F known = tiltSigma(1.0e-3), tilted = tiltSigma(0.02);
    const double acc[3] = {0.0, 0.0, 0.0};
    stepBoth(known, tilted, acc, 200);
    for (int i = 0; i < 3; ++i)
    {
        for (int k = 0; k < 3; ++k)
            EXPECT_NEAR(tilted.P(F::kVel + i, F::kAtt + k), 0.0, 1.0e-15) << i << k;
        EXPECT_NEAR(tilted.P(F::kVel + i, F::kVel + i), known.P(F::kVel + i, F::kVel + i), 1.0e-12) << i;
    }
    EXPECT_NEAR(tilted.velNed()[2], kG, 1.0e-9);         // and the state falls as before
    expectCovarianceHealthy(tilted);
}

TEST(RawGnssEKF, TheTiltToVelocityIsWhatATiltDoesToTheState)
{
    // At any attitude and specific force, the covariance's tilt-to-velocity
    // block is what a tilt does to the state's own velocity over a step: the
    // propagation from q (x) [1, +-eps e_k] (the error state's own injection),
    // differenced. That pins F_va's frame and sign, -2 Tb2n [f x].
    using F = RawGnssEKF<double>;
    const double yaw = 30.0 * kPi / 180.0, pitch = 60.0 * kPi / 180.0, roll = 20.0 * kPi / 180.0;
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2), cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    const double qyp[4] = {cy * cp, -sy * sp, cy * sp, sy * cp};
    const double qr[4] = {std::cos(roll / 2), std::sin(roll / 2), 0.0, 0.0};
    auto mul = [](const double a[4], const double b[4], double o[4]) {
        o[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
        o[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
        o[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
        o[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    };
    double q[4];
    mul(qyp, qr, q);
    const double acc[3] = {4.0, -1.5, -11.0}, gyro[3] = {0.0, 0.0, 0.0}, v0[3] = {0.0, 0.0, 0.0};
    const double dt = 1.0e-3, s = 0.01, eps = 1.0e-6;
    RawGnssEKFParams p = quietParams();
    p.p0_att = p.p0_hdg = s;                             // the attitude's variance s^2 on every axis
    F f(p);
    f.init(kLla0, v0, q, 0.0);
    f.propagate(acc, gyro, dt);
    for (int k = 0; k < 3; ++k)
    {
        double vk[2][3];
        for (int sgn = 0; sgn < 2; ++sgn)
        {
            double d[4] = {1.0, 0.0, 0.0, 0.0}, qp[4];
            d[1 + k] = sgn == 0 ? eps : -eps;
            mul(q, d, qp);
            F g(p);
            g.init(kLla0, v0, qp, 0.0);
            g.propagate(acc, gyro, dt);
            for (int i = 0; i < 3; ++i)
                vk[sgn][i] = g.velNed()[i];
        }
        for (int i = 0; i < 3; ++i)
        {
            const double fd = (vk[0][i] - vk[1][i]) / (2.0 * eps * dt);      // F_va[i][k], from the state
            const double cov = f.P(F::kVel + i, F::kAtt + k) / (s * s * dt);  // and from the covariance
            EXPECT_NEAR(cov, fd, 1.0e-3) << "velocity " << i << ", attitude " << k;   // of up to 2 |f|, 24
        }
    }
}

// ---- the magnetometer as a heading (#1578's measurement, for a raw filter that starts itself)

namespace
{

// yaw then pitch (no roll), body -> NED, scalar first: Quat2DCM of it is NED -> body
void quatYawPitch(double yaw, double pitch, float q[4])
{
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2), cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    q[0] = float(cy * cp);
    q[1] = float(-sy * sp);
    q[2] = float(cy * sp);
    q[3] = float(sy * cp);
}

void nedToBody(const float q[4], const double v[3], float out[3])
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double M[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)},
                            {2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)},
                            {2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)}};
    for (int i = 0; i < 3; ++i)
        out[i] = float(M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2]);
}

// the angle between two attitudes, deg
double attitudeErrorDeg(const float a[4], const float b[4])
{
    const double dot = std::fabs(double(a[0]) * b[0] + double(a[1]) * b[1] + double(a[2]) * b[2] + double(a[3]) * b[3]);
    return 2.0 * std::acos(std::min(1.0, dot)) * 180.0 / kPi;
}

}  // namespace

TEST(RawGnssEKF, TheMagnetometerSetsTheHeadingLevelOrOnAVerticalRail)
{
    // the pad of rp5_0829: its field (TR_GeoMag), magnetic north 11.6 deg west
    const double b_ned[3] = {21.0, -4.3, 45.5};
    const float decl = float(std::atan2(b_ned[1], b_ned[0]));
    for (double pitch_deg : {0.0, 83.6, 90.0})
    {
        float q_true[4], q_wrong[4];
        quatYawPitch(44.7 * kPi / 180.0, pitch_deg * kPi / 180.0, q_true);
        quatYawPitch((44.7 + 30.0) * kPi / 180.0, pitch_deg * kPi / 180.0, q_wrong);   // 30 deg off
        RawGnssEKF<float> f(quietParams());
        const float v0[3] = {0, 0, 0};
        f.init(kLla0, v0, q_wrong, 0.0);
        const double g_ned[3] = {0.0, 0.0, -kG};    // the specific force at rest: up
        float acc[3], mag[3];
        nedToBody(q_true, g_ned, acc);
        nedToBody(q_true, b_ned, mag);
        const float gyro[3] = {0, 0, 0};
        int fused = 0;
        for (int k = 0; k < 100; ++k)                // 2 s at 50 Hz
        {
            f.propagate(acc, gyro, 0.02f);
            f.updateAccelLevel(acc);
            fused += f.updateMagHeading(acc, mag, true, decl, 0.0f) ? 1 : 0;
        }
        EXPECT_EQ(fused, 100) << pitch_deg;
        EXPECT_LT(attitudeErrorDeg(f.quat(), q_true), 1.0) << "pitch " << pitch_deg;
        expectCovarianceHealthy(f);
    }
}

TEST(RawGnssEKF, AFieldWithNoHorizontalPartSaysNothing)
{
    RawGnssEKF<float> f(quietParams());
    const double v0[3] = {0, 0, 0};
    initLevel(f, kLla0, v0);
    const float acc[3] = {0, 0, float(-kG)};
    const float down_only[3] = {0.5f, 0.0f, 48.0f};  // 0.5 uT across: at a magnetic pole
    EXPECT_FALSE(f.updateMagHeading(acc, down_only, true, 0.0f, -1.0f));
    EXPECT_FLOAT_EQ(f.quat()[0], 1.0f);
}

// ---- the attitude's 1-sigma about north, east and down (RAW_GNSS_ATT_MSG)

TEST(RawGnssEKF, TheAttitudeSigmaIsAboutNorthEastAndDown)
{
    // A known attitude (setQuaternion: 1e-6 on each half-angle) is 2 mrad of
    // angle about every axis, at any attitude.
    {
        float q[4];
        quatYawPitch(30.0 * kPi / 180.0, 90.0 * kPi / 180.0, q);
        RawGnssEKF<float> f(quietParams());
        const float v0[3] = {0, 0, 0};
        f.init(kLla0, v0, q, 0.0);
        f.setQuaternion(q);
        float s[3];
        f.attSigmaNed(s);
        for (int i = 0; i < 3; ++i)
            EXPECT_NEAR(s[i], 2.0e-3f, 1.0e-5f) << i;
    }
    // Levelling on gravity learns the tilt and never the heading: at any pitch,
    // up the rail included, north and east lose their sigma and down keeps it.
    // From the default start: its heading sigma is about the vertical at any
    // attitude (#1626 N10). (It was body z's, which tipped over shared down
    // with the tilt, and this took an isotropic start, p0_att = p0_hdg = 0.5.)
    for (double pitch_deg : {0.0, 45.0, 90.0})
    {
        float q[4];
        quatYawPitch(30.0 * kPi / 180.0, pitch_deg * kPi / 180.0, q);
        RawGnssEKF<float> f(quietParams());
        const float v0[3] = {0, 0, 0};
        f.init(kLla0, v0, q, 0.0);
        float s0[3];
        f.attSigmaNed(s0);
        const double g_ned[3] = {0.0, 0.0, -kG};    // the specific force at rest: up
        float acc[3];
        nedToBody(q, g_ned, acc);
        const float gyro[3] = {0, 0, 0};
        for (int k = 0; k < 100; ++k)                // 2 s at 50 Hz
        {
            f.propagate(acc, gyro, 0.02f);
            f.updateAccelLevel(acc);
        }
        float s[3];
        f.attSigmaNed(s);
        EXPECT_LT(s[0], 0.2f * s0[0]) << "north, pitch " << pitch_deg;
        EXPECT_LT(s[1], 0.2f * s0[1]) << "east, pitch " << pitch_deg;
        EXPECT_GT(s[2], 0.8f * s0[2]) << "down (the heading), pitch " << pitch_deg;
    }
}

// ---- the start's heading sigma about the vertical (#1626 N10)

TEST(RawGnssEKF, TheStartLeavesTheHeadingFreeAboutTheVertical)
{
    // init's attitude covariance: p0_hdg about NED down in the body frame and
    // p0_att about north and east, at any attitude -- up a nose-up rocket's
    // rail the heading's is about the nose, not across it -- and nothing shared
    // between the vertical and the tilt. (The state is the half-angle: a
    // rotation's 1-sigma is twice it, as attSigmaNed reports.)
    const RawGnssEKFParams p = quietParams();
    const double a2 = p.p0_att * p.p0_att, h2 = p.p0_hdg * p.p0_hdg;
    ASSERT_GT(p.p0_hdg, 1.4 * p.p0_att);
    using F = RawGnssEKF<double>;
    for (double pitch_deg : {0.0, 45.0, 83.6, 90.0})
    {
        float qf[4];
        quatYawPitch(30.0 * kPi / 180.0, pitch_deg * kPi / 180.0, qf);
        const double q[4] = {qf[0], qf[1], qf[2], qf[3]};
        F f(p);
        const double v0[3] = {0.0, 0.0, 0.0};
        f.init(kLla0, v0, q, 0.0);
        double s[3];
        f.attSigmaNed(s);
        EXPECT_NEAR(s[0], 2.0 * p.p0_att, 1.0e-9) << pitch_deg;
        EXPECT_NEAR(s[1], 2.0 * p.p0_att, 1.0e-9) << pitch_deg;
        EXPECT_NEAR(s[2], 2.0 * p.p0_hdg, 1.0e-9) << pitch_deg;
        // NED down in the body: an eigenvector of the block, with p0_hdg^2
        const double* qn = f.quat();
        const double w = qn[0], x = qn[1], y = qn[2], z = qn[3];
        const double e[3] = {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)};
        for (int a = 0; a < 3; ++a)
        {
            double pe = 0.0;
            for (int b = 0; b < 3; ++b)
                pe += f.P(F::kAtt + a, F::kAtt + b) * e[b];
            EXPECT_NEAR(pe, h2 * e[a], 1.0e-9) << "pitch " << pitch_deg << ", axis " << a;
        }
    }
    // nose up, heading north: the vertical is body -x, so the heading's
    // variance is on x, and z (east, a tilt) has the tilt's
    {
        float qf[4];
        quatYawPitch(0.0, 90.0 * kPi / 180.0, qf);
        const double q[4] = {qf[0], qf[1], qf[2], qf[3]};
        F f(p);
        const double v0[3] = {0.0, 0.0, 0.0};
        f.init(kLla0, v0, q, 0.0);
        EXPECT_NEAR(f.P(F::kAtt, F::kAtt), h2, 1.0e-6);
        EXPECT_NEAR(f.P(F::kAtt + 1, F::kAtt + 1), a2, 1.0e-12);
        EXPECT_NEAR(f.P(F::kAtt + 2, F::kAtt + 2), a2, 1.0e-6);
    }
    // level (q = 1): the diagonal it always was
    {
        F f(p);
        const double v0[3] = {0.0, 0.0, 0.0}, q[4] = {1.0, 0.0, 0.0, 0.0};
        f.init(kLla0, v0, q, 0.0);
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b)
                EXPECT_EQ(f.P(F::kAtt + a, F::kAtt + b), a == b ? (a == 2 ? h2 : a2) : 0.0) << a << b;
    }
}

TEST(RawGnssEKF, UpTheRailTheLevelledStartKeepsItsHeadingFreeInFloat)
{
    // The navigation filter's pad (RawGnssShadow::start): started from the
    // accelerometer's tilt on rp5_0829's rail, and levelled 3 minutes at
    // 100 Hz with no magnetometer. North and east are learned; the heading is
    // not, and keeps p0_hdg's 57 deg (more, as the vertical gyro bias walks)
    // -- in float, as the flight computer runs it, within 1% of double. With
    // p0_hdg at pi, float lost it after 60 s here: the attitude block holding
    // pi^2 about a tilted axis beside what the levelling pins (the tilt with
    // the accelerometer bias, to 4e-6) is past float's precision; it went
    // indefinite, and its rows restarted, the heading at 0.2 deg. A seed
    // degrees off the tilt gives some of the heading's variance to the
    // levelling, as the linearization moves (3 deg off: 41 deg after 10 s).
    float q[4];
    quatYawPitch(44.7 * kPi / 180.0, 83.6 * kPi / 180.0, q);
    const double qd[4] = {q[0], q[1], q[2], q[3]};
    RawGnssEKF<float> f(quietParams());
    RawGnssEKF<double> d(quietParams());
    const float v0[3] = {0, 0, 0};
    const double v0d[3] = {0, 0, 0};
    f.init(kLla0, v0, q, 0.0);
    d.init(kLla0, v0d, qd, 0.0);
    float s0[3];
    f.attSigmaNed(s0);
    const double g_ned[3] = {0.0, 0.0, -kG};
    float acc[3];
    nedToBody(q, g_ned, acc);
    const double accd[3] = {acc[0], acc[1], acc[2]};
    const float gyro[3] = {0, 0, 0};
    const double gyrod[3] = {0, 0, 0};
    double worst = 0.0;
    for (int k = 1; k <= 18000; ++k)
    {
        f.propagate(acc, gyro, 0.01f);
        f.updateAccelLevel(acc);
        d.propagate(accd, gyrod, 0.01);
        d.updateAccelLevel(accd);
        if (k % 100 == 0)
        {
            float s[3];
            double sd[3];
            f.attSigmaNed(s);
            d.attSigmaNed(sd);
            for (int i = 0; i < 3; ++i)
                worst = std::max(worst, std::fabs(double(s[i]) / sd[i] - 1.0));
            ASSERT_GT(s[2], 0.95f * s0[2]) << "t " << k * 0.01 << " s";
        }
    }
    float s[3];
    f.attSigmaNed(s);
    EXPECT_NEAR(s0[2], 2.0f * float(quietParams().p0_hdg), 1.0e-5f);
    EXPECT_LT(s[0], 0.05f * s0[0]);
    EXPECT_LT(s[1], 0.05f * s0[1]);
    EXPECT_LT(worst, 0.01);
    expectCovarianceHealthy(f);
}

// ---- learning the heading in flight (the heading fit's hand-off, RawGnssShadow)

namespace
{

// The rotation about NED down that takes attitude b to attitude a (a = Rz b
// when they differ only in heading), deg, and the tilt left over, deg.
template <typename T>
void headingTilt(const T a[4], const T b[4], double& hdg_deg, double& tilt_deg)
{
    const double a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3];
    const double b0 = b[0], b1 = -b[1], b2 = -b[2], b3 = -b[3];   // conj(b)
    const double r0 = a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3;
    const double r3 = a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0;
    const double n = std::sqrt(r0 * r0 + r3 * r3);
    hdg_deg = 2.0 * std::atan2(r3, r0) * 180.0 / kPi;
    hdg_deg = std::fmod(hdg_deg + 540.0, 360.0) - 180.0;
    tilt_deg = 2.0 * std::acos(std::min(1.0, n)) * 180.0 / kPi;    // the swing's angle
}

// A level body from rest, accelerating each second at a[s] (NED, m/s^2):
// the filter on its IMU and, each second, the sky measured where the body
// truly is. A heading error is told from an accelerometer bias only once the
// push turns.
template <typename T>
void flyLevel(RawGnssEKF<T>& f, const std::vector<std::array<double, 3>>& a, Clock& clk)
{
    const auto sky = makeSky(kLla0);
    double r0[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r0);
    double p_ned[3] = {}, v_ned[3] = {};
    for (const auto& as : a)
    {
        imuSecond(f, as.data());
        clk.tick(1.0);
        double dp[3], dv[3], r[3], v[3];
        for (int k = 0; k < 3; ++k)
        {
            p_ned[k] += v_ned[k] + 0.5 * as[k];
            v_ned[k] += as[k];
        }
        nedToEcef(kLla0[0], kLla0[1], p_ned, dp);
        nedToEcef(kLla0[0], kLla0[1], v_ned, dv);
        for (int k = 0; k < 3; ++k)
        {
            r[k] = r0[k] + dp[k];
            v[k] = dv[k];
        }
        const auto m = measure(sky, r, v, clk);
        f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    }
}

}  // namespace

TEST(RawGnssEKF, TheVerticalGateLearnsTheHeadingAndLeavesTheTilt)
{
    // A level body pushed north-east, its filter 20 deg off in heading and
    // handed that heading with a 30 deg sigma: GNSS turns the heading in and,
    // through the vertical gate, nothing else of the attitude -- the tilt the
    // pad levelled stays exactly, and the gyro bias moves only along the vertical.
    // The pad levels for 6 s first, as flyBoost's (test_raw_gnss_shadow.cpp):
    // that learns the gyro bias across the vertical (1.7e-3 rad/s left of the
    // start's 1.7e-2), which GNSS through the gate cannot. Without it the
    // tilt's sigma grows to 11 deg in the 12 s, and with a tilt's velocity
    // modelled (#1626 E1) the heading is learned only to 2.3 deg (its own
    // sigma 12).
    for (bool gate : {true, false})
    {
        RawGnssEKFParams p = quietParams();
        p.att_gate_vertical = gate;
        RawGnssEKF<double> f(p);
        float qf[4];
        quatYawPitch(20.0 * kPi / 180.0, 0.0, qf);
        const double q[4] = {qf[0], qf[1], qf[2], qf[3]};
        const double v0[3] = {0.0, 0.0, 0.0};
        f.init(kLla0, v0, q, 0.0);
        f.setQuaternion(q);
        const double still[3] = {0.0, 0.0, -kG}, no_rate[3] = {0.0, 0.0, 0.0};
        for (int k = 0; k < 300; ++k)                       // 6 s at 50 Hz
        {
            f.propagate(still, no_rate, 0.02);
            f.updateAccelLevel(still);
        }
        f.setHeadingSigma(30.0 * kPi / 180.0);
        Clock clk;
        std::vector<std::array<double, 3>> a(12, std::array<double, 3>{2.0, 0.0, 0.0});
        for (int s = 4; s < 8; ++s)
            a[s] = {0.0, 2.0, 0.0};
        for (int s = 8; s < 12; ++s)
            a[s] = {-2.0, 1.0, 0.0};
        flyLevel(f, a, clk);
        // in double: a float copy is a unit quaternion to 6e-8 only, which
        // reads as up to 0.03 deg of tilt
        const double level[4] = {1.0, 0.0, 0.0, 0.0};
        double hdg, tilt;
        headingTilt(f.quat(), level, hdg, tilt);
        EXPECT_LT(std::fabs(hdg), 1.0) << "gate " << gate;
        if (gate)
        {
            EXPECT_LT(tilt, 1.0e-4);
            // a level body's vertical is its z: the gyro bias moved along it alone
            EXPECT_LT(std::fabs(f.gyroBias()[0]), 1.0e-9);
            EXPECT_LT(std::fabs(f.gyroBias()[1]), 1.0e-9);
        }
        expectCovarianceHealthy(f);
    }
}

TEST(RawGnssEKF, AHeadingSigmaFreesTheHeadingAndKeepsTheTilt)
{
    // Up a rail or level: after setQuaternion (the pad: a degree's tenth on
    // every axis) a heading sigma frees the rotation about the vertical alone,
    // and drops what it shared with every other state.
    for (double pitch_deg : {0.0, 60.0, 85.0, 90.0})
    {
        Clock clk;
        RawGnssEKF<double> f = converged<double>(5, clk);   // correlated states
        float qf[4];
        quatYawPitch(30.0 * kPi / 180.0, pitch_deg * kPi / 180.0, qf);
        const double q[4] = {qf[0], qf[1], qf[2], qf[3]};
        f.setQuaternion(q);
        imuSecond(f);                                       // the attitude correlates again
        double s0[3];
        f.attSigmaNed(s0);
        f.setHeadingSigma(30.0 * kPi / 180.0);
        double s[3];
        f.attSigmaNed(s);
        EXPECT_NEAR(s[0], s0[0], 1.0e-9 * s0[0]) << pitch_deg;   // the tilt as it was
        EXPECT_NEAR(s[1], s0[1], 1.0e-9 * s0[1]) << pitch_deg;
        EXPECT_NEAR(s[2], 30.0 * kPi / 180.0, 1.0e-6) << pitch_deg;
        // the vertical's part of the attitude shares nothing with the rest
        double M[3][3];
        const double* qn = f.quat();
        const double w = qn[0], x = qn[1], y = qn[2], z = qn[3];
        M[0][2] = 2 * (x * z - w * y);
        M[1][2] = 2 * (y * z + w * x);
        M[2][2] = 1 - 2 * (x * x + y * y);
        for (int j = 0; j < RawGnssEKF<double>::N; ++j)
        {
            if (j >= RawGnssEKF<double>::kAtt && j < RawGnssEKF<double>::kAtt + 3)
                continue;
            double c = 0.0;
            for (int a = 0; a < 3; ++a)
                c += M[a][2] * f.P(RawGnssEKF<double>::kAtt + a, j);
            EXPECT_NEAR(c, 0.0, 1.0e-12 * (1.0 + std::sqrt(f.P(j, j)))) << "state " << j << ", pitch " << pitch_deg;
        }
        expectCovarianceHealthy(f);
    }
}

TEST(RawGnssEKF, TurningTheHeadingKeepsTheTilt)
{
    // yaw then pitch, body -> NED (quatYawPitch, in double)
    auto yawPitch = [](double yaw, double pitch, double q[4]) {
        const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2), cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
        q[0] = cy * cp;
        q[1] = -sy * sp;
        q[2] = cy * sp;
        q[3] = sy * cp;
    };
    double q0[4], want[4];
    yawPitch(10.0 * kPi / 180.0, 80.0 * kPi / 180.0, q0);
    yawPitch(35.0 * kPi / 180.0, 80.0 * kPi / 180.0, want);
    RawGnssEKF<double> f(quietParams());
    const double v0[3] = {0, 0, 0};
    f.init(kLla0, v0, q0, 0.0);
    f.turnHeading(25.0 * kPi / 180.0);
    const double sgn = f.quat()[0] * want[0] >= 0.0 ? 1.0 : -1.0;
    for (int i = 0; i < 4; ++i)
        EXPECT_NEAR(sgn * f.quat()[i], want[i], 1.0e-12) << i;
}

TEST(RawGnssEKF, AHandOffPutsTheBiasesBackAndLetsTheVelocityGo)
{
    Clock clk;
    RawGnssEKF<double> f = converged<double>(8, clk);
    const double ab[3] = {0.02, -0.03, 0.05}, wb[3] = {1e-4, -2e-4, 3e-4};
    double Pb[6][6] = {};
    for (int i = 0; i < 6; ++i)
        Pb[i][i] = (i < 3 ? 0.01 : 1e-6) * (1 + i);
    Pb[0][1] = Pb[1][0] = 1e-3;
    const double v_before[3] = {f.velNed()[0], f.velNed()[1], f.velNed()[2]};
    f.setBiases(ab, wb, Pb);
    f.releaseVelocity(3.0);
    double got[6][6];
    f.biasCovariance(got);
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_EQ(f.accelBias()[i], ab[i]);
        EXPECT_EQ(f.gyroBias()[i], wb[i]);
        EXPECT_EQ(f.velNed()[i], v_before[i]);
        EXPECT_GE(f.P(RawGnssEKF<double>::kVel + i, RawGnssEKF<double>::kVel + i), 9.0);
    }
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            EXPECT_EQ(got[i][j], Pb[i][j]);
    for (int j = 0; j < RawGnssEKF<double>::N; ++j)
    {
        const bool bias = j >= RawGnssEKF<double>::kAccBias && j < RawGnssEKF<double>::kAccBias + 6;
        const bool vel = j >= RawGnssEKF<double>::kVel && j < RawGnssEKF<double>::kVel + 3;
        for (int i = 0; i < 3; ++i)
        {
            if (!bias)
                EXPECT_EQ(f.P(RawGnssEKF<double>::kAccBias + i, j), 0.0) << j;
            if (!vel)
                EXPECT_EQ(f.P(RawGnssEKF<double>::kVel + i, j), 0.0) << j;
        }
    }
    expectCovarianceHealthy(f);
}

TEST(RawGnssEKF, ClearingTheRefusalsForgetsThem)
{
    // Refused twice, forgotten, then refused anew: the third refusal in a row
    // since the clear lets it in, not the third overall.
    Clock clk;
    RawGnssEKF<double> f = converged<double>(10, clk);
    const auto sky = makeSky(kLla0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    const double v[3] = {0.0, 0.0, 0.0};
    for (int e = 0; e < 6; ++e)
    {
        if (e == 2)
            f.clearRejects();
        imuSecond(f);
        clk.tick(1.0);
        auto m = measure(sky, r, v, clk);
        m[3].pr += 200.0;
        const RawGnssStats st = f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
        EXPECT_EQ(st.rejected_pr, e < 5 ? 1 : 0) << "epoch " << e;
    }
}

TEST(RawGnssEKF, AStillReceiversRangeRateIsTheSkysDoppler)
{
    const auto sky = makeSky(kLla0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    const double v[3] = {0.0, 0.0, 0.0};
    Clock clk;
    clk.drift = 0.0;
    const auto m = measure(sky, r, v, clk);
    for (const RawMeas& mi : m)
    {
        double u[3];
        const double rate = stillRangeRate(mi, r, u);
        EXPECT_NEAR(rate, mi.rr, 1.0e-9);
        EXPECT_NEAR(u[0] * u[0] + u[1] * u[1] + u[2] * u[2], 1.0, 1.0e-12);
        // toward the satellite
        EXPECT_GT(u[0] * (mi.sat_pos[0] - r[0]) + u[1] * (mi.sat_pos[1] - r[1]) + u[2] * (mi.sat_pos[2] - r[2]), 0.0);
    }
}
