// Host tests for the raw-GNSS filter's allowances for a turning body with an
// antenna off the IMU by an offset nobody measured (RawGnssShadow.h, "The
// turning body"): the rate the filter keeps for them (RawGnssEKF
// setTrackRate/takeRatePeak), the hold that keeps GNSS off the attitude and
// the gyro bias for a while (setAttHold), and the bound on how fast such an
// antenna can move (RawGnssShadow::rotLeverSigma). Off, each one is the
// Beetle's filter exactly. The sky is test_raw_gnss_ekf_v9.cpp's: ten
// satellites around a still receiver, with a receiver clock that has a bias,
// a drift and system offsets. The shadow's side -- the allowance on each
// Doppler, the epochs held -- is in test_raw_gnss_shadow_v9.cpp.
#include <gtest/gtest.h>

#include <algorithm>
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

void rangeAndRate(const double rs[3], const double vs[3], const double r[3], double& rho, double& rate)
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
        rate += (x[k] - r[k]) / rho * vr[k];
}

std::vector<RawMeas> measure(const std::vector<Sat>& sky, const double r[3], const Clock& clk)
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
        rangeAndRate(s.pos, s.vel, r, rho, rate);
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
    p.cr_mode = gnss_raw::kCarrierOff;
    return p;
}

void imuStill(RawGnssEKF<float>& f, double dt_s, const float gyro[3] = nullptr)
{
    const float acc[3] = {0.0f, 0.0f, float(-kG)};
    const float w0[3] = {0.0f, 0.0f, 0.0f};
    const int steps = static_cast<int>(std::lround(dt_s / kDt));
    for (int i = 0; i < steps; ++i)
        f.propagate(acc, gyro ? gyro : w0, float(kDt));
}

// A level filter started off the truth and given n epochs a second apart.
RawGnssEKF<float> converged(int epochs, Clock& clk, RawGnssEKFParams p = quietParams())
{
    p.p0_pos = 30.0;
    RawGnssEKF<float> f(p);
    const double lla[3] = {kLla0[0] + 20.0 / 6.37e6, kLla0[1] - 10.0 / 5.35e6, kLla0[2] - 15.0};
    const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float v[3] = {0.0f, 0.0f, 0.0f};
    f.init(lla, v, q, 0.0);
    f.setQuaternion(q);
    const auto sky = makeSky(kLla0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    for (int e = 0; e < epochs; ++e)
    {
        imuStill(f, 1.0);
        clk.tick(1.0);
        const auto m = measure(sky, r, clk);
        f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    }
    return f;
}

bool sameState(const RawGnssEKF<float>& a, const RawGnssEKF<float>& b)
{
    for (int i = 0; i < 3; ++i)
        if (a.lla()[i] != b.lla()[i] || a.velNed()[i] != b.velNed()[i] || a.accelBias()[i] != b.accelBias()[i]
            || a.gyroBias()[i] != b.gyroBias()[i])
            return false;
    for (int i = 0; i < 4; ++i)
        if (a.quat()[i] != b.quat()[i])
            return false;
    if (a.clockBias() != b.clockBias() || a.clockDrift() != b.clockDrift())
        return false;
    for (int i = 0; i < RawGnssEKF<float>::N; ++i)
        for (int j = 0; j < RawGnssEKF<float>::N; ++j)
            if (a.P(i, j) != b.P(i, j))
                return false;
    return true;
}

}  // namespace

// ---- the rate the filter keeps (setTrackRate, takeRatePeak)

TEST(RawGnssRot, TheRateIsKeptOnlyWhenAskedAndThenChangesNothingElse)
{
    // Off (the Beetle's), the take reads 0 whatever the gyro did. On, it
    // reads the largest rate since the last take -- the gyro less the
    // filter's bias -- in whole and across the nose (body y and z), each its
    // own largest, and starts again from 0. Keeping it moves nothing of the
    // filter: the two step bit for bit alike.
    Clock clk;
    RawGnssEKF<float> off = converged(3, clk);
    RawGnssEKF<float> on = off;
    on.setTrackRate(true);
    const float bias[3] = {0.01f, -0.02f, 0.005f};
    off.seedGyroBias(bias);
    on.seedGyroBias(bias);
    const float g1[3] = {0.51f, 1.98f, -0.995f};      // less the bias: 0.5, 2.0, -1.0
    const float g2[3] = {3.01f, -0.02f, 0.005f};      //   3.0, 0, 0: a roll, nothing across
    const float g3[3] = {0.01f, 0.48f, 0.005f};       //   0, 0.5, 0
    for (RawGnssEKF<float>* f : {&off, &on})
    {
        imuStill(*f, 0.05, g1);
        imuStill(*f, 0.05, g2);
    }
    EXPECT_TRUE(sameState(off, on));
    float all = -1.0f, across = -1.0f;
    off.takeRatePeak(all, across);
    EXPECT_EQ(all, 0.0f);
    EXPECT_EQ(across, 0.0f);
    on.takeRatePeak(all, across);
    EXPECT_NEAR(all, 3.0f, 1e-5f);                      // the roll's, the larger whole
    EXPECT_NEAR(across, std::sqrt(5.0f), 1e-5f);        // g1's across: sqrt(4 + 1), though g2's whole was larger
    on.takeRatePeak(all, across);
    EXPECT_EQ(all, 0.0f);                               // taken: none since
    EXPECT_EQ(across, 0.0f);
    imuStill(on, 0.05, g3);
    on.takeRatePeak(all, across);
    EXPECT_NEAR(all, 0.5f, 1e-5f);                      // a pitch alone: all of it across
    EXPECT_NEAR(across, 0.5f, 1e-5f);
    // init starts it again
    imuStill(on, 0.05, g1);
    const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float v[3] = {0.0f, 0.0f, 0.0f};
    on.init(kLla0, v, q, 0.0);
    on.takeRatePeak(all, across);
    EXPECT_EQ(all, 0.0f);
    EXPECT_EQ(across, 0.0f);
}

// ---- the hold (setAttHold)

TEST(RawGnssRot, HeldAnEpochIsTheUnaidedTwinsAndLetGoTheFiltersOwn)
{
    // Held, an epoch's pseudoranges and Dopplers reach the attitude and the
    // gyro bias no more than the unaided twin's do (att_gnss_off): the same
    // state after it, bit for bit. Let go, the next epoch is the default
    // filter's update again; and a hold never touches a filter that is not
    // held. init lets it go.
    Clock clk;
    RawGnssEKF<float> base = converged(5, clk);
    imuStill(base, 1.0);
    clk.tick(1.0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    auto m = measure(makeSky(kLla0), r, clk);
    for (auto& x : m)
        x.rr += 0.03;                                   // a little for the update to do
    RawGnssEKFParams pu = quietParams();
    pu.p0_pos = 30.0;
    pu.att_gnss_off = true;
    RawGnssEKF<float> held = base, twin = base, aided = base;
    twin.setParams(pu);
    held.setAttHold(true);
    EXPECT_TRUE(held.attHold());
    EXPECT_FALSE(aided.attHold());
    const RawGnssStats sh = held.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    twin.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    aided.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    EXPECT_EQ(sh.used_pr, 10);
    EXPECT_EQ(sh.used_rr, 10);
    EXPECT_TRUE(sameState(held, twin));
    for (int i = 0; i < 3; ++i)
        EXPECT_EQ(held.gyroBias()[i], base.gyroBias()[i]);
    constexpr int kAtt = RawGnssEKF<float>::kAtt, kGb = RawGnssEKF<float>::kGyroBias;
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
        {
            EXPECT_EQ(held.P(kAtt + a, kAtt + b), base.P(kAtt + a, kAtt + b));
            EXPECT_EQ(held.P(kGb + a, kGb + b), base.P(kGb + a, kGb + b));
        }
    // the aided filter's update did move them
    bool moved = false;
    for (int a = 0; a < 3; ++a)
        moved = moved || aided.P(kAtt + a, kAtt + a) != base.P(kAtt + a, kAtt + a);
    EXPECT_TRUE(moved);
    // let go, the next epoch reaches the attitude again
    RawGnssEKF<float> let_go = held;
    let_go.setAttHold(false);
    imuStill(let_go, 0.1);
    clk.tick(0.1);
    auto m2 = measure(makeSky(kLla0), r, clk);
    for (auto& x : m2)
        x.rr -= 0.03;
    const RawGnssEKF<float> before = let_go;
    let_go.updateGnssRaw(m2.data(), static_cast<int>(m2.size()));
    bool turned = false;
    for (int a = 0; a < 3; ++a)
        turned = turned || let_go.P(kAtt + a, kAtt + a) != before.P(kAtt + a, kAtt + a);
    EXPECT_TRUE(turned);
    // a copy takes the hold along; init lets it go
    RawGnssEKF<float> copy = held;
    EXPECT_TRUE(copy.attHold());
    const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    const float v[3] = {0.0f, 0.0f, 0.0f};
    copy.init(kLla0, v, q, 0.0);
    EXPECT_FALSE(copy.attHold());
}

// ---- the bound (RawGnssShadow::rotLeverSigma)

TEST(RawGnssRot, TheBoundIsTheAxialArmOnTheCrossRatePlusTheRadialArmOnTheWholeRate)
{
    // Off at 0, 0 whatever the rate
    EXPECT_EQ(RawGnssShadow::rotLeverSigma(0.0, 0.0, 5.0, 5.0), 0.0);
    // 0.3 m up the nose: a roll does not move it, a pitch does
    EXPECT_DOUBLE_EQ(RawGnssShadow::rotLeverSigma(0.3, 0.0, 10.0, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(RawGnssShadow::rotLeverSigma(0.3, 0.0, 5.0, 5.0), 1.5);
    // 0.05 m across it: any rate does, up to its whole size
    EXPECT_DOUBLE_EQ(RawGnssShadow::rotLeverSigma(0.0, 0.05, 10.0, 0.0), 0.5);
    EXPECT_DOUBLE_EQ(RawGnssShadow::rotLeverSigma(0.5, 0.05, 4.0, 3.0), 0.5 * 3.0 + 0.05 * 4.0);
    // an isotropic bound (all of it across): |omega| r
    EXPECT_DOUBLE_EQ(RawGnssShadow::rotLeverSigma(0.0, 0.4, 2.5, 1.0), 1.0);
    // negatives count as 0
    EXPECT_EQ(RawGnssShadow::rotLeverSigma(-0.3, -0.1, 5.0, 5.0), 0.0);
    EXPECT_EQ(RawGnssShadow::rotLeverSigma(0.3, 0.1, -5.0, -5.0), 0.0);
    // it is a bound: for any offset within the box and any rate, |omega x r| is no larger
    const double axial = 0.4, radial = 0.06;
    for (int i = 0; i < 2000; ++i)
    {
        const double a = std::sin(1.3 * i) * axial;                  // along the nose, either side
        const double ph = 0.7 * i, rr = radial * std::fabs(std::cos(0.37 * i));
        const double r[3] = {a, rr * std::cos(ph), rr * std::sin(ph)};
        const double w[3] = {6.0 * std::sin(0.11 * i), 4.0 * std::cos(0.23 * i), 3.0 * std::sin(0.53 * i + 1.0)};
        const double c[3] = {w[1] * r[2] - w[2] * r[1], w[2] * r[0] - w[0] * r[2], w[0] * r[1] - w[1] * r[0]};
        const double v = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
        const double all = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]), across = std::hypot(w[1], w[2]);
        EXPECT_LE(v, RawGnssShadow::rotLeverSigma(axial, radial, all, across) + 1e-12) << i;
    }
}
