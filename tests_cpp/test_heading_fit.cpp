// Host tests for the heading from thrust against GNSS velocity
// (TR_RawGnssShadow/HeadingFit.h): the batch fit on a boosted, leaning body
// from any seed heading, its lateral biases, its silence on a vertical
// flight, the robust cut and the shared weight of close epochs, the fast
// profile against the prototype's full grid (its second valley too), and the
// Doppler velocity that feeds it, in float on a fast flight. Synthetic
// throughout, as the prototype's own tests (gnss repo
// sim/tests/test_heading_fit.py).
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "HeadingFit.h"
#include "TR_RawGnssEKF.h"
#include "gnss/atmo.h"
#include "gnss/types.h"

using gnss_raw::HeadingFit;
using gnss_raw::RawMeas;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.807;
constexpr double kDeg = kPi / 180.0;

struct M3
{
    double m[3][3];
};

M3 eye()
{
    M3 r{};
    for (int i = 0; i < 3; ++i)
        r.m[i][i] = 1.0;
    return r;
}

M3 mul(const M3& a, const M3& b)
{
    M3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
}

void mulv(const M3& a, const double v[3], double o[3])
{
    for (int i = 0; i < 3; ++i)
        o[i] = a.m[i][0] * v[0] + a.m[i][1] * v[1] + a.m[i][2] * v[2];
}

M3 transpose(const M3& a)
{
    M3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.m[i][j] = a.m[j][i];
    return r;
}

// Rodrigues: a rotation by angle a about axis
M3 rot(const double axis[3], double a)
{
    const double n = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    const double k[3] = {axis[0] / n, axis[1] / n, axis[2] / n};
    const double K[3][3] = {{0, -k[2], k[1]}, {k[2], 0, -k[0]}, {-k[1], k[0], 0}};
    M3 r = eye();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            double kk = 0.0;
            for (int l = 0; l < 3; ++l)
                kk += K[i][l] * K[l][j];
            r.m[i][j] += std::sin(a) * K[i][j] + (1.0 - std::cos(a)) * kk;
        }
    return r;
}

void cross(const double a[3], const double b[3], double o[3])
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}

double norm(const double v[3])
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// The quaternion (scalar first) of an active rotation R (v' = R v), Shepperd.
void dcm2quat(const M3& Rm, double q[4])
{
    const double (&R)[3][3] = Rm.m;
    const double t = R[0][0] + R[1][1] + R[2][2];
    const double c[4] = {t, R[0][0], R[1][1], R[2][2]};
    int k = 0;
    for (int i = 1; i < 4; ++i)
        if (c[i] > c[k])
            k = i;
    if (k == 0)
    {
        const double w = 0.5 * std::sqrt(1.0 + t);
        q[0] = w; q[1] = (R[2][1] - R[1][2]) / (4 * w); q[2] = (R[0][2] - R[2][0]) / (4 * w); q[3] = (R[1][0] - R[0][1]) / (4 * w);
    }
    else if (k == 1)
    {
        const double x = 0.5 * std::sqrt(1.0 + 2 * R[0][0] - t);
        q[0] = (R[2][1] - R[1][2]) / (4 * x); q[1] = x; q[2] = (R[0][1] + R[1][0]) / (4 * x); q[3] = (R[0][2] + R[2][0]) / (4 * x);
    }
    else if (k == 2)
    {
        const double y = 0.5 * std::sqrt(1.0 + 2 * R[1][1] - t);
        q[0] = (R[0][2] - R[2][0]) / (4 * y); q[1] = (R[0][1] + R[1][0]) / (4 * y); q[2] = y; q[3] = (R[1][2] + R[2][1]) / (4 * y);
    }
    else
    {
        const double z = 0.5 * std::sqrt(1.0 + 2 * R[2][2] - t);
        q[0] = (R[1][0] - R[0][1]) / (4 * z); q[1] = (R[0][2] + R[2][0]) / (4 * z); q[2] = (R[1][2] + R[2][1]) / (4 * z); q[3] = z;
    }
    if (q[0] < 0)
        for (int i = 0; i < 4; ++i)
            q[i] = -q[i];
}

// The rotation about NED down taking attitude b to a, deg (a = Rz b when they
// differ only in heading).
double headingDiffDeg(const float a[4], const double b_in[4])
{
    const double a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3];
    const double b0 = b_in[0], b1 = -b_in[1], b2 = -b_in[2], b3 = -b_in[3];
    const double r0 = a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3;
    const double r3 = a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0;
    double h = 2.0 * std::atan2(r3, r0) / kDeg;
    return std::fmod(h + 540.0, 360.0) - 180.0;
}

// Body-to-NED of a nose-up body leaning lean_deg toward azimuth toward_deg,
// rolled about its nose (the prototype's _leaning_body).
M3 leaningBody(double lean_deg = 5.0, double toward_deg = 70.0, double roll_deg = 25.0)
{
    const double z[3] = {0, 0, 1}, y[3] = {0, 1, 0}, up[3] = {0, 0, -1.0};
    const M3 Rn = mul(rot(z, toward_deg * kDeg), rot(y, lean_deg * kDeg));
    double nose[3];
    mulv(Rn, up, nose);
    M3 C{};                                     // body x up
    C.m[0][2] = 1; C.m[1][1] = 1; C.m[2][0] = -1;
    const double cx[3] = {C.m[0][0], C.m[1][0], C.m[2][0]};
    double ax[3];
    cross(cx, nose, ax);
    if (norm(ax) > 1e-12)
        C = mul(rot(ax, std::asin(norm(ax))), C);
    return mul(rot(nose, roll_deg * kDeg), C);
}

struct Boost
{
    HeadingFit fit;
    double seed_err_deg = 0.0;   // the seed's heading error, measured
    double q_true[4] = {};
};

// A boost (10 g for 1.5 s), then drag to 8 s, on a leaning body; the fit
// seeded from the biased pad specific force and turned seed_err_deg about
// vertical; GNSS velocity once a second (or every gnss_every steps of 1 ms).
// One epoch can be thrown off by bad_mps.
void boost(Boost& out, double seed_err_deg, double by = 0.06, double bz = -0.09, double noise = 0.1,
           unsigned seed = 1, int gnss_every = 1000, int bad_epoch = -1, double bad_mps = 0.0)
{
    std::mt19937 rng(seed);
    std::normal_distribution<double> nd(0.0, noise);
    const M3 Ct = leaningBody();
    const double b[3] = {0.0, by, bz};
    const double g_up[3] = {0, 0, -kG};
    double a_pad[3];
    mulv(transpose(Ct), g_up, a_pad);
    for (int i = 0; i < 3; ++i)
        a_pad[i] += b[i];
    double v[3];
    mulv(Ct, a_pad, v);
    const double down_neg[3] = {0, 0, -1.0};
    double ax[3];
    cross(v, down_neg, ax);
    const M3 Rfix = norm(ax) > 0 ? rot(ax, std::asin(norm(ax) / norm(v))) : eye();
    const double zax[3] = {0, 0, 1};
    const M3 Cs = mul(mul(rot(zax, seed_err_deg * kDeg), Rfix), Ct);
    double q_seed[4];
    dcm2quat(Ct, out.q_true);
    dcm2quat(Cs, q_seed);
    const float qs[4] = {float(q_seed[0]), float(q_seed[1]), float(q_seed[2]), float(q_seed[3])};
    const float wb[3] = {0, 0, 0};
    out.fit.start(qs, wb, 0.0);
    out.seed_err_deg = headingDiffDeg(qs, out.q_true);
    double t = 0.0, vt[3] = {0, 0, 0};
    const double dt = 0.001;
    int epoch = 0;
    for (int k = 1; k <= 8000; ++k)
    {
        const double fb[3] = {t < 1.5 ? 100.0 : -3.0, 0.0, 0.0};
        const float acc[3] = {float(fb[0] + b[0]), float(fb[1] + b[1]), float(fb[2] + b[2])};
        const float gyro[3] = {0, 0, 0};
        out.fit.imu(acc, gyro, float(dt));
        double fn[3];
        mulv(Ct, fb, fn);
        for (int i = 0; i < 3; ++i)
            vt[i] += (fn[i] + (i == 2 ? kG : 0.0)) * dt;
        t += dt;
        if (k % gnss_every == 0)
        {
            double vm[2] = {vt[0] + nd(rng), vt[1] + nd(rng)};
            nd(rng);                                  // the vertical's draw, as the prototype
            if (epoch == bad_epoch)
                vm[0] += bad_mps;
            out.fit.gnss(out.fit.time(), vm, noise * noise);
            epoch++;
        }
    }
}

}  // namespace

TEST(HeadingFit, FindsAnySeedHeadingAndTheLateralBiases)
{
    for (double err : {-60.0, 20.0, 150.0})
    {
        Boost bo;
        boost(bo, err);
        const HeadingFit::Result r = bo.fit.solve();
        ASSERT_TRUE(r.ok);
        EXPECT_NEAR(bo.seed_err_deg, err, 1e-3);
        EXPECT_NEAR(r.psi / kDeg, -bo.seed_err_deg, 0.5) << err;    // psi is the turn back
        ASSERT_GT(r.sigma, 0.0);
        EXPECT_LT(r.sigma / kDeg, 3.0);
        EXPECT_NEAR(r.bias[0], 0.06, 0.01);
        EXPECT_NEAR(r.bias[1], -0.09, 0.01);
        EXPECT_EQ(r.n, 8);
        EXPECT_EQ(r.n_cut, 0);
        // turned by psi, the gyro-only attitude is the truth's heading
        float q[4];
        bo.fit.attitude(r.psi, q);
        EXPECT_LT(std::fabs(headingDiffDeg(q, bo.q_true)), 0.5);
        EXPECT_NEAR(headingDiffDeg(q, std::vector<double>(bo.fit.gyroOnlyQuat(), bo.fit.gyroOnlyQuat() + 4).data()),
                    r.psi / kDeg, 1e-3);
    }
}

TEST(HeadingFit, TheFastProfileIsThePrototypesGrid)
{
    // solve() walks a coarse pass and the grid near its best; solveGrid()
    // evaluates all 1,440 points, as the prototype: the same answer, for far
    // fewer points.
    for (unsigned seed = 1; seed <= 12; ++seed)
    {
        Boost bo;
        boost(bo, -170.0 + 29.0 * seed, 0.03 * std::sin(seed), -0.05 * std::cos(seed), 0.05 + 0.05 * (seed % 4), seed);
        const HeadingFit::Result f = bo.fit.solve(), g = bo.fit.solveGrid();
        ASSERT_TRUE(f.ok && g.ok);
        EXPECT_NEAR(f.psi, g.psi, 1e-9) << seed;
        EXPECT_EQ(f.sigma, g.sigma) << seed;
        EXPECT_NEAR(f.bias[0], g.bias[0], 1e-9);
        EXPECT_NEAR(f.bias[1], g.bias[1], 1e-9);
        EXPECT_EQ(f.n_cut, g.n_cut);
        EXPECT_LT(f.evals * 4, g.evals) << seed;
        // thrust along the nose, biases across it: one valley (the biases take
        // up a turn the same whichever way it goes, or not at all)
        EXPECT_LT(g.rival, 0.0) << seed;
        EXPECT_LT(f.rival, 0.0) << seed;
    }
}

TEST(HeadingFit, BiasesThatCanTakeUpAHalfTurnLeaveARival)
{
    // Only an IMU pushed along an axis whose bias the fit solves for gives the
    // profile a second valley: here a level body thrust out its right side
    // (body y, east) while climbing at 1 g, its y bias free to 100 m/s^2. The
    // bias takes up the push backwards as well as forwards, so a half-turn
    // leaves only the bias's prior to tell them apart. Both searches find the
    // same valley, and the same rival.
    for (double err : {-120.0, 30.0, 170.0})
    {
        HeadingFit::Config c;
        c.bias_sigma_mps2 = 100.0;
        HeadingFit fit(c);
        const float q[4] = {float(std::cos(0.5 * err * kDeg)), 0.0f, 0.0f, float(std::sin(0.5 * err * kDeg))};
        const float wb[3] = {0, 0, 0};
        fit.start(q, wb, 0.0);
        std::mt19937 rng(7);
        std::normal_distribution<double> nd(0.0, 0.1);
        const float acc[3] = {0.0f, 100.0f, float(-2.0 * kG)}, gyro[3] = {0, 0, 0};
        for (int k = 1; k <= 3000; ++k)
        {
            fit.imu(acc, gyro, 0.001f);
            if (k % 250 == 0)
            {
                const double v[2] = {nd(rng), 100.0 * fit.time() + nd(rng)};
                fit.gnss(fit.time(), v, 0.01);
            }
        }
        const HeadingFit::Result f = fit.solve(), g = fit.solveGrid();
        ASSERT_TRUE(f.ok && g.ok);
        EXPECT_NEAR(f.psi, g.psi, 1e-9) << err;
        EXPECT_EQ(f.sigma, g.sigma) << err;
        EXPECT_NEAR(std::remainder(g.psi + err * kDeg, 2.0 * kPi), 0.0, 0.5 * kDeg) << err;
        ASSERT_GE(g.rival, 0.0) << err;
        EXPECT_NEAR(f.rival, g.rival, 1e-6 * g.rival) << err;
        EXPECT_GT(g.rival, 1.0) << err;
    }
}

TEST(HeadingFit, SaysNothingOnAVerticalFlight)
{
    // Straight up: no horizontal speed, no heading to give -- even at 20 Hz,
    // where the IMU's drift alone would read as one
    HeadingFit fit;
    const float q[4] = {float(std::cos(kPi / 4)), 0.0f, float(std::sin(kPi / 4)), 0.0f};   // nose up
    const float wb[3] = {0, 0, 0};
    fit.start(q, wb, 0.0);
    std::mt19937 rng(3);
    std::normal_distribution<double> n05(0.0, 0.05), n1(0.0, 0.1);
    double t = 0.0, vt = 0.0;
    for (int k = 1; k <= 8000; ++k)
    {
        const double f = t < 1.5 ? 100.0 : -3.0;
        const float acc[3] = {float(f + n05(rng)), float(n05(rng)), float(n05(rng))};
        const float gyro[3] = {0, 0, 0};
        fit.imu(acc, gyro, 0.001f);
        vt += (f - kG) * 0.001;
        t += 0.001;
        if (k % 50 == 0)
        {
            const double v[2] = {n1(rng), n1(rng)};
            EXPECT_FALSE(fit.gnss(fit.time(), v, 0.01));
        }
    }
    EXPECT_FALSE(fit.solve().ok);
}

TEST(HeadingFit, CutsAnEpochThatDisagrees)
{
    Boost bo;
    boost(bo, 35.0, 0.06, -0.09, 0.1, 1, 1000, 5, 20.0);   // the 6th epoch 20 m/s off
    const HeadingFit::Result r = bo.fit.solve();
    ASSERT_TRUE(r.ok);
    EXPECT_EQ(r.n_cut, 1);
    EXPECT_EQ(r.n, 7);
    EXPECT_NEAR(r.psi / kDeg, -bo.seed_err_deg, 0.5);
}

TEST(HeadingFit, CloseEpochsShareOneEpochsWeight)
{
    // A 20 Hz receiver has twenty epochs a second whose errors the IMU's
    // drift makes one: shared, its sigma is about a 1 Hz receiver's, not a
    // twentieth of it (sqrt(20) narrower)
    Boost slow, fast;
    boost(slow, 25.0, 0.06, -0.09, 0.3, 7, 1000);
    boost(fast, 25.0, 0.06, -0.09, 0.3, 7, 50);
    const HeadingFit::Result rs = slow.fit.solve(), rf = fast.fit.solve();
    ASSERT_TRUE(rs.ok && rf.ok);
    ASSERT_GT(rs.sigma, 0.0);
    ASSERT_GT(rf.sigma, 0.0);
    EXPECT_GT(rf.sigma, 0.4 * rs.sigma);
}

TEST(HeadingFit, TakesOnlyWhatItsHistoryCovers)
{
    HeadingFit fit;
    const float q[4] = {1, 0, 0, 0};
    const float wb[3] = {0, 0, 0};
    const double v[2] = {5.0, 1.0};
    EXPECT_FALSE(fit.gnss(0.5, v, 0.01));                    // not started
    fit.start(q, wb, 10.0);
    const float acc[3] = {0, 0, float(-kG)}, gyro[3] = {0, 0, 0};
    for (int k = 0; k < 400; ++k)                            // 4 s at 100 Hz
        fit.imu(acc, gyro, 0.01f);
    EXPECT_FALSE(fit.gnss(14.5, v, 0.01));                   // after the last step
    EXPECT_FALSE(fit.gnss(10.5, v, 0.01));                   // older than the 2.56 s kept
    EXPECT_TRUE(fit.gnss(13.0, v, 0.01));
    const double slow[2] = {0.5, 0.5};
    EXPECT_FALSE(fit.gnss(13.5, slow, 0.01));                // under 1 m/s
    EXPECT_EQ(fit.epochs(), 1);
}

// ---- the Doppler velocity

namespace
{

// Satellites 20,000 km up at fixed azimuths and elevations, in orbit-like motion.
std::vector<RawMeas> skyMeasured(const double lla[3], const double v_ned[3], double drift, double sigma_rr,
                                 unsigned seed)
{
    struct AzEl { int sys, prn; double az, el; };
    const AzEl dirs[] = {{GNSS_SYS_GPS, 3, 0, 70},   {GNSS_SYS_GPS, 8, 60, 35},   {GNSS_SYS_GPS, 14, 130, 20},
                         {GNSS_SYS_GPS, 22, 200, 50}, {GNSS_SYS_GPS, 31, 290, 25}, {GNSS_SYS_GAL, 4, 30, 15},
                         {GNSS_SYS_GAL, 11, 170, 60}, {GNSS_SYS_GAL, 26, 250, 40}, {GNSS_SYS_BDS, 23, 100, 45},
                         {GNSS_SYS_BDS, 37, 320, 55}};
    double r[3];
    geo_to_ecef(lla[0], lla[1], lla[2], r);
    const double sl = std::sin(lla[0]), cl = std::cos(lla[0]), so = std::sin(lla[1]), co = std::cos(lla[1]);
    auto enu2ecef = [&](const double e[3], double o[3]) {
        o[0] = -so * e[0] - sl * co * e[1] + cl * co * e[2];
        o[1] = co * e[0] - sl * so * e[1] + cl * so * e[2];
        o[2] = cl * e[1] + sl * e[2];
    };
    const double v_enu[3] = {v_ned[1], v_ned[0], -v_ned[2]};
    double v[3];
    enu2ecef(v_enu, v);
    std::mt19937 rng(seed);
    std::normal_distribution<double> nd(0.0, sigma_rr);
    std::vector<RawMeas> out;
    for (const AzEl& d : dirs)
    {
        const double az = d.az * kDeg, el = d.el * kDeg;
        const double enu[3] = {std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el)};
        double u[3];
        enu2ecef(enu, u);
        const double R = 26.56e6, ru = r[0] * u[0] + r[1] * u[1] + r[2] * u[2];
        const double rr = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
        const double rho = -ru + std::sqrt(ru * ru - rr + R * R);
        RawMeas m{};
        m.sys = static_cast<uint8_t>(d.sys);
        m.prn = static_cast<uint8_t>(d.prn);
        for (int k = 0; k < 3; ++k)
            m.sat_pos[k] = r[k] + rho * u[k];
        const double z[3] = {0.3, -0.2, 1.0};
        double sv[3];
        cross(m.sat_pos, z, sv);
        const double n = norm(sv);
        for (int k = 0; k < 3; ++k)
            m.sat_vel[k] = 3870.0 * sv[k] / n;
        double uu[3];
        const double still = gnss_raw::stillRangeRate(m, r, uu);
        m.has_rr = true;
        m.rr = still - (uu[0] * v[0] + uu[1] * v[1] + uu[2] * v[2]) + drift + nd(rng);
        m.sigma_rr = static_cast<float>(sigma_rr);
        out.push_back(m);
    }
    return out;
}

}  // namespace

TEST(HeadingFit, TheDopplerVelocityDropsABadSatellite)
{
    const double lla[3] = {38.0 * kDeg, -122.0 * kDeg, 100.0};
    const double v_true[3] = {3.0, -2.0, -40.0};
    auto m = skyMeasured(lla, v_true, 12.5, 0.05, 5);
    double r[3];
    geo_to_ecef(lla[0], lla[1], lla[2], r);
    double v_rest[3], cov[3][3];
    int used = 0;
    ASSERT_TRUE(gnss_raw::dopplerVelocity(m.data() + 1, static_cast<int>(m.size()) - 1, r, v_rest, cov, used));
    m[0].rr += 25.0;                                           // a relocked satellite's Doppler
    double v_ned[3], var_h = 0.0;
    ASSERT_TRUE(gnss_raw::dopplerVelocityNed(m.data(), static_cast<int>(m.size()), r, lla[0], lla[1], v_ned,
                                             var_h, used));
    EXPECT_EQ(used, static_cast<int>(m.size()) - 1);
    double v[3];
    ASSERT_TRUE(gnss_raw::dopplerVelocity(m.data(), static_cast<int>(m.size()), r, v, cov, used));
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(v[i], v_rest[i], 1e-9);
        EXPECT_NEAR(v_ned[i], v_true[i], 0.3) << i;           // a few sigma of 0.05 m/s through this geometry
    }
    EXPECT_GT(var_h, 0.0);
    EXPECT_LT(var_h, 0.05);
    // too few left: nothing
    EXPECT_FALSE(gnss_raw::dopplerVelocity(m.data(), 4, r, v, cov, used));
}


TEST(HeadingFit, TheFloatDopplerVelocityHoldsAFastFlightToACentimetre)
{
    // Float after one double difference, the solve in float: on a receiver
    // at 340 m/s whose clock drifts 300 m/s -- the largest numbers the solve
    // sees -- its rounding stays under the Doppler's own millimetres
    const double lla[3] = {33.0 * kDeg, -107.0 * kDeg, 9000.0};
    const double v_true[3] = {150.0, -80.0, -300.0};
    const auto m = skyMeasured(lla, v_true, 300.0, 0.001, 9);
    double r[3];
    geo_to_ecef(lla[0], lla[1], lla[2], r);
    double v_ned[3], var_h = 0.0;
    int used = 0;
    ASSERT_TRUE(gnss_raw::dopplerVelocityNed(m.data(), static_cast<int>(m.size()), r, lla[0], lla[1], v_ned, var_h,
                                             used));
    EXPECT_EQ(used, static_cast<int>(m.size()));
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(v_ned[i], v_true[i], 0.01) << i;
}
