// Host tests for the physics checks on the GNSS (#1508): the raw filter's
// IMU-only motion between two times (RawGnssEKF::imuMotion), each satellite's
// pseudorange against the IMU's propagation epoch to epoch (SatJumpCheck), and
// the receiver's own fix against the IMU (FixCheck) -- the jump, the frozen
// fix on a falling vehicle and the slow climb through a 3 g boost that #1508
// caught the LC86G publishing.
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "FixCheck.h"
#include "SatJumpCheck.h"
#include "TR_RawGnssEKF.h"
#include "gnss/atmo.h"
#include "gnss/types.h"

using gnss_raw::FixCheck;
using gnss_raw::RawGnssEKF;
using gnss_raw::RawEpoch;
using gnss_raw::RawGnssEKFParams;
using gnss_raw::RawMeas;
using gnss_raw::SatJumpCheck;
using gnss_raw::TimedFix;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kG = 9.807;
const double kLla[3] = {39.97 * kDeg, -74.94 * kDeg, 20.0};

// ---- the IMU's motion

RawGnssEKF<double> filterAt(double t0, double spacing_s)
{
    RawGnssEKFParams p;
    p.hist_spacing_s = spacing_s;
    RawGnssEKF<double> f(p);
    const double v[3] = {0.0, 0.0, 0.0};
    const double q[4] = {1.0, 0.0, 0.0, 0.0};   // body = NED
    f.init(kLla, v, q, t0);
    return f;
}

// ---- satellites

void basis(const double r[3], double e[3], double n[3], double u[3])
{
    const double rn = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    for (int a = 0; a < 3; ++a)
        u[a] = r[a] / rn;
    const double h = std::sqrt(r[0] * r[0] + r[1] * r[1]);
    e[0] = -r[1] / h;
    e[1] = r[0] / h;
    e[2] = 0.0;
    n[0] = u[1] * e[2] - u[2] * e[1];
    n[1] = u[2] * e[0] - u[0] * e[2];
    n[2] = u[0] * e[1] - u[1] * e[0];
}

struct Sv
{
    uint8_t sys, prn;
    double az, el;
};

std::vector<Sv> sky()
{
    return {{GNSS_SYS_GPS, 5, 40, 75}, {GNSS_SYS_GPS, 10, 130, 35}, {GNSS_SYS_GPS, 18, 220, 20},
            {GNSS_SYS_GPS, 20, 310, 50}, {GNSS_SYS_GAL, 4, 80, 60}, {GNSS_SYS_GAL, 9, 190, 40},
            {GNSS_SYS_BDS, 22, 150, 65}, {GNSS_SYS_BDS, 37, 240, 33}};
}

// Satellites placed from the pad (they do not move: the receiver's motion is
// what the check is about), each pseudorange its range from r plus clock.
std::vector<RawMeas> epochAt(const double pad[3], const double r[3], double clock_m)
{
    double e[3], n[3], u[3];
    basis(pad, e, n, u);
    std::vector<RawMeas> out;
    for (const Sv& s : sky())
    {
        RawMeas m{};
        m.sys = s.sys;
        m.prn = s.prn;
        m.has_pr = true;
        const double az = s.az * kDeg, el = s.el * kDeg;
        for (int a = 0; a < 3; ++a)
            m.sat_pos[a] = pad[a] + 2.2e7 * (std::cos(el) * std::sin(az) * e[a] + std::cos(el) * std::cos(az) * n[a]
                                             + std::sin(el) * u[a]);
        m.pr = gnss_raw::sagnacRange(m, r) + clock_m;
        out.push_back(m);
    }
    return out;
}

// ---- fixes

TimedFix fixAt(double n_m, double e_m, double d_m, double vn, double ve, double vd)
{
    const double s = std::sin(kLla[0]);
    const double w = std::sqrt(1.0 - 0.0066943799901 * s * s);
    const double rn = 6378137.0 * (1.0 - 0.0066943799901) / (w * w * w) + kLla[2];
    const double re = 6378137.0 / w + kLla[2];
    TimedFix f;
    f.lat_rad = kLla[0] + n_m / rn;
    f.lon_rad = kLla[1] + e_m / (re * std::cos(kLla[0]));
    f.alt_m = kLla[2] - d_m;
    f.vel_ned[0] = static_cast<float>(vn);
    f.vel_ned[1] = static_cast<float>(ve);
    f.vel_ned[2] = static_cast<float>(vd);
    f.fix_mode = 3;
    f.num_sats = 12;
    return f;
}

// A vehicle accelerating straight up at a (m/s^2) from rest at t = 0: the
// fix and the IMU's motion between two times.
TimedFix boostFix(double t, double a)
{
    return fixAt(0.0, 0.0, -0.5 * a * t * t, 0.0, 0.0, -a * t);
}

void boostImu(double t1, double t2, double a, float dv[3], float dp[3])
{
    // velocity change, and the displacement that change adds: a (t2 - t1)^2 / 2
    const double T = t2 - t1;
    dv[0] = dv[1] = 0.0f;
    dp[0] = dp[1] = 0.0f;
    dv[2] = static_cast<float>(-a * T);
    dp[2] = static_cast<float>(-0.5 * a * T * T);
}

}  // namespace

// ------------------------------------------------------------- imuMotion

TEST(ImuMotion, IsTheIntegralOfTheImusOwnAcceleration)
{
    // 2 m/s^2 north for 2 s, level: the velocity change and the displacement
    // it adds over any window are the textbook ones
    RawGnssEKF<double> f = filterAt(100.0, 0.02);
    const double acc[3] = {2.0, 0.0, -kG}, gyro[3] = {0.0, 0.0, 0.0};
    for (int k = 0; k < 400; ++k)
        f.propagate(acc, gyro, 0.005);
    double dv[3], dp[3];
    ASSERT_TRUE(f.imuMotion(100.5, 101.5, dv, dp));
    EXPECT_NEAR(dv[0], 2.0, 2e-3);
    EXPECT_NEAR(dp[0], 1.0, 2e-3);             // (2 / 2) * 1 s^2
    EXPECT_NEAR(dv[2], 0.0, 1e-3);             // gravity is out of it
    ASSERT_TRUE(f.imuMotion(101.2, 101.2, dv, dp));
    EXPECT_EQ(dv[0], 0.0);
    // a window within one entry is that entry's share
    ASSERT_TRUE(f.imuMotion(101.903, 101.913, dv, dp));
    EXPECT_NEAR(dv[0], 0.02, 1e-6);
}

TEST(ImuMotion, IsRefusedOutsideTheHistory)
{
    RawGnssEKF<double> f = filterAt(100.0, 0.004);    // 128 entries of 5 ms: 0.64 s
    const double acc[3] = {0.0, 0.0, -kG}, gyro[3] = {0.0, 0.0, 0.0};
    for (int k = 0; k < 400; ++k)
        f.propagate(acc, gyro, 0.005);
    double dv[3], dp[3];
    EXPECT_FALSE(f.imuMotion(100.5, 101.9, dv, dp));  // before the history
    EXPECT_FALSE(f.imuMotion(101.8, 102.5, dv, dp));  // after the filter's time
    EXPECT_FALSE(f.imuMotion(101.9, 101.8, dv, dp));  // backwards
    EXPECT_TRUE(f.imuMotion(101.5, 102.0, dv, dp));
}

// ----------------------------------------------------------- SatJumpCheck

TEST(SatJumpCheck, AClockStepAndAConsistentPositionErrorAreNotJumps)
{
    double pad[3];
    geo_to_ecef(kLla[0], kLla[1], kLla[2], pad);
    SatJumpCheck c;
    bool flagged[RawEpoch::kMaxObs];
    // the receiver moving 40 m/s east; the filter 50 m off, the same at both epochs
    double e[3], n[3], u[3];
    basis(pad, e, n, u);
    double r0[3], r1[3], f0[3], f1[3];
    for (int a = 0; a < 3; ++a)
    {
        r0[a] = pad[a];
        r1[a] = pad[a] + 40.0 * e[a];
        f0[a] = r0[a] + 50.0 * n[a];
        f1[a] = r1[a] + 50.0 * n[a];
    }
    auto m0 = epochAt(pad, r0, 1.0e5);
    EXPECT_EQ(c.check(m0.data(), static_cast<int>(m0.size()), 1000.0, f0, flagged), 0);
    EXPECT_EQ(c.summary().judged, 0);         // nothing to compare with yet
    c.commit(m0.data(), static_cast<int>(m0.size()), 1000.0, f0);
    // the receiver clock steps by a millisecond and a bit
    auto m1 = epochAt(pad, r1, 1.0e5 + 299792.458 + 17.0);
    EXPECT_EQ(c.check(m1.data(), static_cast<int>(m1.size()), 1001.0, f1, flagged), 0);
    EXPECT_EQ(c.summary().judged, static_cast<int>(m1.size()));
    EXPECT_NEAR(c.summary().clock_m, 299792.458 + 17.0, 0.05);
    EXPECT_LT(c.summary().worst_m, 0.1);
}

TEST(SatJumpCheck, ACodeJumpIsFlaggedAndTheNextEpochIsCleanAgain)
{
    double pad[3];
    geo_to_ecef(kLla[0], kLla[1], kLla[2], pad);
    SatJumpCheck c;
    bool flagged[RawEpoch::kMaxObs];
    auto m0 = epochAt(pad, pad, 0.0);
    c.check(m0.data(), static_cast<int>(m0.size()), 1000.0, pad, flagged);
    c.commit(m0.data(), static_cast<int>(m0.size()), 1000.0, pad);
    auto m1 = epochAt(pad, pad, 3.0);
    m1[2].pr += 30.0;                          // G18 jumps 30 m
    EXPECT_EQ(c.check(m1.data(), static_cast<int>(m1.size()), 1001.0, pad, flagged), 1);
    EXPECT_TRUE(flagged[2]);
    EXPECT_NEAR(c.summary().worst_m, 30.0, 0.01);
    // the pad survey takes its correction out between check and commit: the
    // next epoch is judged from the pseudoranges as built
    for (RawMeas& x : m1)
        x.pr -= 7.0;
    c.commit(m1.data(), static_cast<int>(m1.size()), 1001.0, pad);
    auto m2 = epochAt(pad, pad, 6.0);
    m2[2].pr += 30.0;                          // at its new level
    EXPECT_EQ(c.check(m2.data(), static_cast<int>(m2.size()), 1002.0, pad, flagged), 0);
}

TEST(SatJumpCheck, JudgesNothingWithTooFewOrAGap)
{
    double pad[3];
    geo_to_ecef(kLla[0], kLla[1], kLla[2], pad);
    SatJumpCheck c;
    bool flagged[RawEpoch::kMaxObs];
    auto m0 = epochAt(pad, pad, 0.0);
    c.check(m0.data(), static_cast<int>(m0.size()), 1000.0, pad, flagged);
    c.commit(m0.data(), static_cast<int>(m0.size()), 1000.0, pad);
    auto m1 = epochAt(pad, pad, 0.0);
    m1[0].pr += 100.0;
    EXPECT_EQ(c.check(m1.data(), 4, 1001.0, pad, flagged), 0);   // four in both: too few
    EXPECT_EQ(c.check(m1.data(), static_cast<int>(m1.size()), 1003.0, pad, flagged), 0);   // 3 s on
    EXPECT_EQ(c.summary().judged, 0);
}

// --------------------------------------------------------------- FixCheck

TEST(FixCheck, ATrueBoostPassesAndTheFirstFixIsUnjudged)
{
    FixCheck c;
    const double a = 3.0 * kG;
    const float up[3] = {0.0f, 0.0f, static_cast<float>(-a)};
    EXPECT_EQ(c.onFix(boostFix(0.0, a), 0.0, nullptr, nullptr, up).verdict, FixCheck::kUnjudged);
    for (int k = 1; k <= 30; ++k)
    {
        const double t = 0.1 * k;
        double ta;
        ASSERT_TRUE(c.anchorTime(ta));
        float dv[3], dp[3];
        boostImu(ta, t, a, dv, dp);
        const auto r = c.onFix(boostFix(t, a), t, dv, dp, up);
        EXPECT_EQ(r.verdict, FixCheck::kPass) << t << " " << r.pos_err_m << "/" << r.pos_tol_m << " "
                                              << r.vel_err_mps << "/" << r.vel_tol_mps;
    }
    EXPECT_EQ(c.stats().passed, 30u);
}

TEST(FixCheck, ASlowClimbThroughA3gBoostIsFlagged)
{
    // #1508 at 3 g: the receiver climbing 1-7 m/s while the vehicle went from
    // 78 to 323 m/s. Here: it keeps saying 4 m/s up from a good fix at rest.
    FixCheck c;
    const double a = 3.0 * kG;
    const float up[3] = {0.0f, 0.0f, static_cast<float>(-a)};
    c.onFix(boostFix(0.0, a), 0.0, nullptr, nullptr, up);
    int flagged = 0;
    for (int k = 1; k <= 10; ++k)
    {
        const double t = 0.1 * k;
        double ta;
        c.anchorTime(ta);
        float dv[3], dp[3];
        boostImu(ta, t, a, dv, dp);
        const auto r = c.onFix(fixAt(0.0, 0.0, -4.0 * t, 0.0, 0.0, -4.0), t, dv, dp, up);
        if (r.verdict == FixCheck::kFlagged)
            flagged++;
    }
    EXPECT_GE(flagged, 9);    // the first is within the noise; every one after, not
}

TEST(FixCheck, AJumpIsFlaggedAndTheTrackPassesAgainAfterIt)
{
    FixCheck c;
    const double v = 50.0;                     // m/s east, steady
    auto at = [&](double t, double extra_d) { return fixAt(0.0, v * t, extra_d, 0.0, v, 0.0); };
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    c.onFix(at(0.0, 0.0), 0.0, nullptr, nullptr, zero);
    for (int k = 1; k <= 5; ++k)
        EXPECT_EQ(c.onFix(at(0.1 * k, 0.0), 0.1 * k, zero, zero, zero).verdict, FixCheck::kPass);
    const auto jump = c.onFix(at(0.6, 300.0), 0.6, zero, zero, zero);   // 300 m down, one fix
    EXPECT_EQ(jump.verdict, FixCheck::kFlagged);
    EXPECT_NEAR(jump.pos_err_m, 300.0, 1.0);
    const auto back = c.onFix(at(0.7, 0.0), 0.7, zero, zero, zero);      // judged from 0.5 s
    EXPECT_EQ(back.verdict, FixCheck::kPass);
    EXPECT_NEAR(back.span_s, 0.2, 1e-6);
}

TEST(FixCheck, AFixFrozenOnAFallingVehicleIsFlaggedUntilItLapses)
{
    // #1508 near apogee: the fix stood still near the pad while the vehicle
    // fell (specific force zero: the IMU's velocity change is gravity's)
    FixCheck c;
    const float fall[3] = {0.0f, 0.0f, static_cast<float>(kG)};
    c.onFix(fixAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0, nullptr, nullptr, fall);
    int flagged = 0;
    for (int k = 1; k <= 50; ++k)
    {
        const double t = 0.1 * k;
        double ta;
        c.anchorTime(ta);
        const double T = t - ta;
        const float dv[3] = {0.0f, 0.0f, static_cast<float>(kG * T)};
        const float dp[3] = {0.0f, 0.0f, static_cast<float>(0.5 * kG * T * T)};
        const auto r = c.onFix(fixAt(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), t, dv, dp, fall);
        if (r.verdict == FixCheck::kFlagged)
            flagged++;
    }
    EXPECT_GE(flagged, 45);
    EXPECT_GE(c.stats().lapsed, 1u);           // the good fix aged past the history once
}

TEST(FixCheck, NoImuHistoryMeansUnjudgedAndANewAnchor)
{
    FixCheck c;
    c.onFix(fixAt(0, 0, 0, 0, 0, 0), 0.0, nullptr, nullptr, nullptr);
    const auto r = c.onFix(fixAt(500, 0, 0, 0, 0, 0), 0.1, nullptr, nullptr, nullptr);
    EXPECT_EQ(r.verdict, FixCheck::kUnjudged);
    double ta;
    ASSERT_TRUE(c.anchorTime(ta));
    EXPECT_EQ(ta, 0.1);
    const float zero[3] = {0.0f, 0.0f, 0.0f};
    EXPECT_EQ(c.onFix(fixAt(500, 0, 0, 0, 0, 0), 4.5, zero, zero, zero).verdict, FixCheck::kUnjudged);   // 4.4 s on
}
