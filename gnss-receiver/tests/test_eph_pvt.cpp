extern "C" {
#include "gnss/eph.h"
#include "gnss/pvt.h"
#include "gnss/sig.h"
}

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

// GPS PRN 5, toe 208800 (2026-08-18), as broadcast (BRDC_2026230_MN.rnx).
gps_eph_t prn5()
{
    gps_eph_t e{};
    e.valid = 1;
    e.prn = 5;
    e.week = 2432;
    e.iodc = e.iode = 38;
    e.toe = e.toc = 208800.0;
    e.sqrt_a = 5153.553930283;
    e.e = 0.005468975286931;
    e.i0 = 0.9815418526551;
    e.omega0 = 1.058898734125;
    e.omega = 1.493738749586;
    e.m0 = -1.882253619327;
    e.delta_n = 3.918734659599e-09;
    e.idot = 7.714607058635e-11;
    e.omega_dot = -7.686391597634e-09;
    e.cuc = -9.126961231232e-07;
    e.cus = 8.98540019989e-06;
    e.crc = 217.3125;
    e.crs = -17.65625;
    e.cic = 7.078051567078e-08;
    e.cis = 8.940696716309e-08;
    e.af0 = -0.0002360786311328;
    e.af1 = -3.410605131648e-13;
    e.af2 = 0.0;
    e.tgd = -1.024454832077e-08;
    return e;
}

}  // namespace

TEST(Eph, MatchesAnIndependentImplementation)
{
    // Reference from py/gnssrx/rinex.py (a separate Python implementation of IS-GPS-200).
    gps_eph_t e = prn5();
    double p[3], clk;
    gps_sat_pos(&e, 210630.0, p, nullptr, &clk);
    EXPECT_NEAR(p[0], -6206618.9784, 1e-3);
    EXPECT_NEAR(p[1], -25666069.7405, 1e-3);
    EXPECT_NEAR(p[2], -2917945.9111, 1e-3);
    EXPECT_NEAR(clk, -2.360565044605619e-04, 1e-15);
    EXPECT_NEAR(gps_time_diff(10.0, 604790.0), 20.0, 1e-9);
    EXPECT_NEAR(gps_time_diff(604790.0, 10.0), -20.0, 1e-9);
}

TEST(Pvt, RecoversAKnownPositionAndClock)
{
    // Satellites: PRN 5's orbit rotated in node and anomaly, kept where the receiver sees them.
    double lat = 0.0, lon = -119.0 * M_PI / 180.0, h = 1200.0, rx[3];
    geo_to_ecef(lat, lon, h, rx);
    const double t_rx = 210630.0, bias = 1234.5, vel_rx[3] = {0.0, 0.0, 0.0};
    (void)vel_rx;
    static gps_eph_t eph[GNSS_SYS_COUNT][GNSS_MAX_PRN + 1];
    std::vector<pvt_meas_t> m;
    int prn = 1;
    for (int k = 0; k < 24 && prn <= 10; k++) {
        gps_eph_t e = prn5();
        e.prn = prn;
        e.omega0 += k * 2.0 * M_PI / 6.0;
        e.m0 += k * 0.9;
        // Transmit time by light-time iteration, with the Earth's rotation during flight.
        double tau = 0.07, sp[3] = {0, 0, 0}, clk = 0.0;
        for (int it = 0; it < 5; it++) {
            double p[3];
            gps_sat_pos(&e, t_rx - tau, p, nullptr, &clk);
            double a = GPS_OMEGA_E * tau;
            sp[0] = std::cos(a) * p[0] + std::sin(a) * p[1];
            sp[1] = -std::sin(a) * p[0] + std::cos(a) * p[1];
            sp[2] = p[2];
            tau = std::sqrt(std::pow(sp[0] - rx[0], 2) + std::pow(sp[1] - rx[1], 2) + std::pow(sp[2] - rx[2], 2)) /
                  GNSS_C;
        }
        // Visible?
        double up = (std::cos(lat) * std::cos(lon) * (sp[0] - rx[0]) + std::cos(lat) * std::sin(lon) * (sp[1] - rx[1]) +
                     std::sin(lat) * (sp[2] - rx[2])) / (tau * GNSS_C);
        if (up < std::sin(10.0 * M_PI / 180.0)) {
            continue;
        }
        eph[GNSS_SYS_GPS][prn] = e;
        pvt_meas_t q{};
        q.sys = GNSS_SYS_GPS;
        q.prn = prn;
        q.pr = tau * GNSS_C + bias - GNSS_C * clk;
        q.t_sv = t_rx - tau + clk;  // the satellite's clock reading at transmission
        q.cn0 = 45.0f;
        m.push_back(q);
        prn++;
    }
    ASSERT_GE(m.size(), 5u);
    pvt_opt_t opt;
    pvt_default_opt(&opt);
    opt.use_iono = opt.use_tropo = 0;
    pvt_sol_t sol;
    ASSERT_EQ(pvt_solve(m.data(), int(m.size()), eph, nullptr, &opt, nullptr, &sol), 0);
    EXPECT_NEAR(sol.pos[0], rx[0], 1e-3);
    EXPECT_NEAR(sol.pos[1], rx[1], 1e-3);
    EXPECT_NEAR(sol.pos[2], rx[2], 1e-3);
    EXPECT_NEAR(sol.clk_bias, bias, 1e-3);
    EXPECT_NEAR(sol.h, h, 1e-3);
    EXPECT_LT(sol.resid_rms, 1e-3);
}

TEST(Pvt, GeodeticRoundTrip)
{
    for (double la : {-60.0, 0.0, 33.3, 89.9}) {
        double x[3], lat, lon, h;
        geo_to_ecef(la * M_PI / 180.0, 2.0, 1500.0, x);
        ecef_to_geo(x, &lat, &lon, &h);
        EXPECT_NEAR(lat * 180.0 / M_PI, la, 1e-9);
        EXPECT_NEAR(lon, 2.0, 1e-12);
        EXPECT_NEAR(h, 1500.0, 1e-6);
    }
}

// The troposphere above a climbing rocket: the delay carries on through 10 km (0.6 m at the zenith
// there) and fades out by 40 km; and the velocity solution follows the delay's fall as the receiver
// climbs (at 1 km/s through 5 km, over 0.4 m/s of range rate at 20 deg).
TEST(Pvt, TroposphereAboveAClimbingReceiver)
{
    const double z30 = M_PI / 6.0;  // 30 deg elevation
    EXPECT_NEAR(tropo_saastamoinen(0.0, 9999.0, z30), tropo_saastamoinen(0.0, 10001.0, z30), 1e-3);
    EXPECT_GT(tropo_saastamoinen(0.0, 10000.0, M_PI / 2.0), 0.55);
    EXPECT_GT(tropo_saastamoinen(0.0, 30000.0, M_PI / 2.0), 0.0);
    EXPECT_LT(tropo_saastamoinen(0.0, 30000.0, M_PI / 2.0), 0.02);
    EXPECT_EQ(tropo_saastamoinen(0.0, 41000.0, M_PI / 2.0), 0.0);
    // Finite and thinning all the way up: the wet term once divided by zero at 38.4 km.
    for (const double el : {M_PI / 2.0, z30, 5.0 * M_PI / 180.0}) {
        double last = tropo_saastamoinen(0.0, 0.0, el);
        for (double hh = 10.0; hh <= 40000.0; hh += 10.0) {
            const double d = tropo_saastamoinen(0.0, hh, el);
            ASSERT_TRUE(std::isfinite(d)) << hh;
            ASSERT_LE(d, last + 1e-12) << hh;
            last = d;
        }
    }

    double lat = 0.0, lon = -119.0 * M_PI / 180.0, h = 5000.0, rx[3];
    geo_to_ecef(lat, lon, h, rx);
    const double upv[3] = {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
    const double t_rx = 210630.0, bias = 1234.5, drift = 0.5, v_up = 1000.0;
    const double v_rx[3] = {v_up * upv[0], v_up * upv[1], v_up * upv[2]};
    const double lambda = GNSS_C / GNSS_FREQ_L1_HZ;
    static gps_eph_t eph[GNSS_SYS_COUNT][GNSS_MAX_PRN + 1];
    std::vector<pvt_meas_t> m;
    int prn = 1;
    for (int k = 0; k < 24 && prn <= 10; k++) {
        gps_eph_t e = prn5();
        e.prn = prn;
        e.omega0 += k * 2.0 * M_PI / 6.0;
        e.m0 += k * 0.9;
        double tau = 0.07, sp[3] = {0, 0, 0}, clk = 0.0, p[3], vs[3];
        for (int it = 0; it < 5; it++) {
            gps_sat_pos(&e, t_rx - tau, p, vs, &clk);
            double a = GPS_OMEGA_E * tau;
            sp[0] = std::cos(a) * p[0] + std::sin(a) * p[1];
            sp[1] = -std::sin(a) * p[0] + std::cos(a) * p[1];
            sp[2] = p[2];
            tau = std::sqrt(std::pow(sp[0] - rx[0], 2) + std::pow(sp[1] - rx[1], 2) + std::pow(sp[2] - rx[2], 2)) /
                  GNSS_C;
        }
        double rho = tau * GNSS_C;
        double el = std::asin(((sp[0] - rx[0]) * upv[0] + (sp[1] - rx[1]) * upv[1] + (sp[2] - rx[2]) * upv[2]) / rho);
        if (el < 10.0 * M_PI / 180.0) {
            continue;
        }
        eph[GNSS_SYS_GPS][prn] = e;
        // Doppler as the PVT models it: the satellite's velocity along the unrotated line of sight,
        // its clock's rate, and the troposphere's change with height.
        double d[3] = {p[0] - rx[0], p[1] - rx[1], p[2] - rx[2]};
        double r = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        double c1, p1[3];
        gps_sat_pos(&e, t_rx - tau + 1.0, p1, nullptr, &c1);
        double rr = 0.0;
        for (int j = 0; j < 3; j++) {
            rr += (vs[j] - v_rx[j]) * d[j] / r;
        }
        const double g = 0.5 * (tropo_saastamoinen(lat, h + 1.0, el) - tropo_saastamoinen(lat, h - 1.0, el));
        rr += g * v_up + drift - GNSS_C * (c1 - clk);
        pvt_meas_t q{};
        q.sys = GNSS_SYS_GPS;
        q.prn = prn;
        q.pr = rho + bias - GNSS_C * clk + tropo_saastamoinen(lat, h, el);
        q.t_sv = t_rx - tau + clk;
        q.dop = -rr / lambda;
        q.cn0 = 45.0f;
        m.push_back(q);
        prn++;
    }
    ASSERT_GE(m.size(), 5u);
    pvt_opt_t opt;
    pvt_default_opt(&opt);
    opt.use_iono = 0;
    pvt_sol_t sol;
    ASSERT_EQ(pvt_solve(m.data(), int(m.size()), eph, nullptr, &opt, nullptr, &sol), 0);
    EXPECT_NEAR(sol.h, h, 0.01);
    for (int j = 0; j < 3; j++) {
        EXPECT_NEAR(sol.vel[j], v_rx[j], 0.01) << j;
    }
    EXPECT_NEAR(sol.clk_drift, drift, 0.01);
}

namespace {

// A static receiver and its satellites (PRN 5's orbit rotated), with exact pseudoranges.
struct StaticSky {
    double lat = 0.0, lon = -119.0 * M_PI / 180.0, h = 1200.0, rx[3];
    const double t_rx = 210630.0, bias = 1234.5;
    gps_eph_t eph[GNSS_SYS_COUNT][GNSS_MAX_PRN + 1] = {};
    std::vector<pvt_meas_t> m;

    StaticSky()
    {
        geo_to_ecef(lat, lon, h, rx);
        int prn = 1;
        for (int k = 0; k < 96 && prn <= 12; k++) {
            gps_eph_t e = prn5();
            e.prn = prn;
            e.omega0 += k * 2.0 * M_PI / 6.0;
            e.m0 += k * 0.37;
            double tau = 0.07, sp[3] = {0, 0, 0}, clk = 0.0;
            for (int it = 0; it < 5; it++) {
                double p[3];
                gps_sat_pos(&e, t_rx - tau, p, nullptr, &clk);
                double a = GPS_OMEGA_E * tau;
                sp[0] = std::cos(a) * p[0] + std::sin(a) * p[1];
                sp[1] = -std::sin(a) * p[0] + std::cos(a) * p[1];
                sp[2] = p[2];
                tau = std::sqrt(std::pow(sp[0] - rx[0], 2) + std::pow(sp[1] - rx[1], 2) +
                                std::pow(sp[2] - rx[2], 2)) / GNSS_C;
            }
            double up = (std::cos(lat) * std::cos(lon) * (sp[0] - rx[0]) +
                         std::cos(lat) * std::sin(lon) * (sp[1] - rx[1]) + std::sin(lat) * (sp[2] - rx[2])) /
                        (tau * GNSS_C);
            if (up < std::sin(10.0 * M_PI / 180.0)) {
                continue;
            }
            eph[GNSS_SYS_GPS][prn] = e;
            pvt_meas_t q{};
            q.sys = GNSS_SYS_GPS;
            q.prn = prn;
            q.pr = tau * GNSS_C + bias - GNSS_C * clk;
            q.t_sv = t_rx - tau + clk;
            q.cn0 = 45.0f;
            q.sigma = 0.5f;
            m.push_back(q);
            prn++;
        }
    }

    int solve(pvt_sol_t *sol) const
    {
        pvt_opt_t opt;
        pvt_default_opt(&opt);
        opt.use_iono = opt.use_tropo = 0;
        return pvt_solve(m.data(), int(m.size()), eph, nullptr, &opt, nullptr, sol);
    }

    double err(const pvt_sol_t &sol) const
    {
        return std::sqrt(std::pow(sol.pos[0] - rx[0], 2) + std::pow(sol.pos[1] - rx[1], 2) +
                         std::pow(sol.pos[2] - rx[2], 2));
    }
};

}  // namespace

// The residual test (milestone 7): one bad range is found and left out.
TEST(Pvt, LeavesOutAFaultyRange)
{
    StaticSky s;
    ASSERT_GE(s.m.size(), 8u);
    s.m[3].pr += 30.0;
    pvt_sol_t sol;
    ASSERT_EQ(s.solve(&sol), 0);
    EXPECT_EQ(sol.nexcl, 1);
    EXPECT_EQ(sol.excluded[3] & 1, 1);
    EXPECT_EQ(sol.used[3], 0);
    EXPECT_LT(s.err(sol), 0.01);
    EXPECT_LE(sol.chi2, sol.chi2_lim);
}

// Three bad ranges are more than it may leave out (two): the fix is withheld, not reported wrong.
TEST(Pvt, WithholdsAFixItCannotMend)
{
    StaticSky s;
    ASSERT_GE(s.m.size(), 8u);
    s.m[1].pr += 40.0;
    s.m[4].pr -= 35.0;
    s.m[6].pr += 50.0;
    pvt_sol_t sol;
    EXPECT_EQ(s.solve(&sol), -2);
    EXPECT_EQ(sol.valid, 0);
}

// Weights follow each measurement's sigma: a weak satellite's 5 m error barely moves the fix,
// where with equal weights it would move it metres.
TEST(Pvt, WeighsAWeakSatelliteLightly)
{
    StaticSky s;
    ASSERT_GE(s.m.size(), 8u);
    s.m[2].pr += 5.0;
    s.m[2].sigma = pvt_sigma_pr(30.0f, 0.0f);  // raw, at 30 dB-Hz: ~3 m
    pvt_sol_t sol;
    ASSERT_EQ(s.solve(&sol), 0);
    EXPECT_EQ(sol.nexcl, 0);
    EXPECT_LT(s.err(sol), 0.3);
    EXPECT_GT(pvt_sigma_pr(30.0f, 0.0f), 2.5);
    EXPECT_LT(pvt_sigma_pr(45.0f, 100.0f), 0.25);
    // Range rates: wider loops and pulling-in channels count for less.
    const double d10 = pvt_sigma_dop(43.0f, 1, 10.0f);
    EXPECT_NEAR(d10, 0.063, 0.005);
    EXPECT_NEAR(pvt_sigma_dop(43.0f, 1, 50.0f) / d10, std::pow(5.0, 0.75), 1e-9);
    EXPECT_NEAR(pvt_sigma_dop(43.0f, 0, 10.0f) / d10, 5.0, 1e-9);
    EXPECT_GT(pvt_sigma_dop(31.0f, 1, 10.0f), 3.5 * d10);
}

// Coarse-time navigation (milestone 7, the seed): every transmit time 0.3 s late, as when the
// receiver's milliseconds come from a seed whose clock is that far out (PSAS's TeleMetrum time
// was 0.6 s out). The plain solve puts the satellites where they will be 0.3 s on (hundreds of
// metres); the fifth unknown finds the 0.3 s and the position.
TEST(Pvt, CoarseTimeSolvesLateTransmitTimes)
{
    /* A seed clock 0.61 s out (PSAS's TeleMetrum time) leaves every transmit time that late. */
    for (const double tau : {0.0, 0.005, 0.610}) {
        StaticSky s;
        ASSERT_GE(s.m.size(), 8u);
        for (auto &q : s.m) {
            q.t_sv += tau;
        }
        pvt_opt_t opt;
        pvt_default_opt(&opt);
        opt.use_iono = opt.use_tropo = 0;
        pvt_sol_t plain;
        const int r = pvt_solve(s.m.data(), int(s.m.size()), s.eph, nullptr, &opt, nullptr, &plain);
        if (tau > 0.1) {
            EXPECT_TRUE(r == -2 || s.err(plain) > 10.0) << "the plain solve should not survive " << tau << " s";
        }
        opt.coarse_time = 1;
        pvt_sol_t sol;
        ASSERT_EQ(pvt_solve(s.m.data(), int(s.m.size()), s.eph, nullptr, &opt, nullptr, &sol), 0) << tau;
        EXPECT_NEAR(sol.time_offset, tau, 1e-6) << tau;
        EXPECT_LT(s.err(sol), 0.01) << tau;
        /* Half-metre ranges against a few hundred m/s of range-rate spread: a millisecond or so. */
        EXPECT_GT(sol.time_sigma, 1e-4) << tau;
        EXPECT_LT(sol.time_sigma, 5e-3) << tau;
        EXPECT_EQ(sol.nexcl, 0) << tau;
    }
}
