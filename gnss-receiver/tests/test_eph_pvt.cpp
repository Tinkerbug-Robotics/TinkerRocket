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
