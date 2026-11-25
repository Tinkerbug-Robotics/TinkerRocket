// Host tests for the antenna's offset where the rocket's owner measured it
// (TR_RawGnssEKF.h lever_on, RawGnssShadow.h "The antenna's offset"): the
// antenna's offset and velocity the filter keeps beside its history and
// takes at each measurement's own instant (leverAt), every range rate and
// pseudorange predicted at the antenna, the Doppler change check's changes
// the antenna's (and what an attitude error does to the lever's predicted
// change: knownLeverArm), the offset's roll terms observed or fused
// (lever_att_mode, the gyro bias's at the Doppler's instant),
// and the owner's setting -- the board's frame, mm -- in the filter's
// (antennaOffsetInBody, applyAntennaOffset). Off, the filter is the one the
// Beetle flies, bit for bit. The sky is test_raw_gnss_rot.cpp's: ten
// satellites around the pad, a receiver clock with a bias, a drift and
// system offsets, here measured at an antenna that moves.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "RawGnssShadow.h"
#include "RawMeas.h"
#include "ReceiverProfileApply.h"
#include "TR_RawGnssEKF.h"
#include "gnss/atmo.h"
#include "gnss/types.h"

using gnss_raw::AntennaOffset;
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

// The Sagnac-turned range from r and the range rate a receiver moving at ve
// (ECEF) sees
void rangeAndRate(const double rs[3], const double vs[3], const double r[3], const double ve[3], double& rho,
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
        rate += (x[k] - r[k]) / rho * (vr[k] - ve[k]);
}

// Every satellite measured by a receiver at r moving at ve (ECEF)
std::vector<RawMeas> measure(const std::vector<Sat>& sky, const double r[3], const double ve[3], const Clock& clk)
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
        rangeAndRate(s.pos, s.vel, r, ve, rho, rate);
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
    p.p0_pos = 30.0;
    return p;
}

// body -> NED of a rotation by a about a unit axis (body frame), after C0
struct M3
{
    double m[3][3];
};

M3 mul(const M3& a, const M3& b)
{
    M3 c{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                c.m[i][j] += a.m[i][k] * b.m[k][j];
    return c;
}

M3 rot(const double ax[3], double a)
{
    const double c = std::cos(a), s = std::sin(a), t = 1.0 - c;
    const double x = ax[0], y = ax[1], z = ax[2];
    return M3{{{t * x * x + c, t * x * y - s * z, t * x * z + s * y},
               {t * x * y + s * z, t * y * y + c, t * y * z - s * x},
               {t * x * z - s * y, t * y * z + s * x, t * z * z + c}}};
}

void apply(const M3& a, const double v[3], double o[3])
{
    for (int i = 0; i < 3; ++i)
        o[i] = a.m[i][0] * v[0] + a.m[i][1] * v[1] + a.m[i][2] * v[2];
}

void quatOf(const M3& C, float q[4])
{
    // body -> NED, scalar first (Shepperd)
    const double tr = C.m[0][0] + C.m[1][1] + C.m[2][2];
    double w, x, y, z;
    if (tr > 0.0)
    {
        const double s = std::sqrt(tr + 1.0) * 2.0;
        w = 0.25 * s;
        x = (C.m[2][1] - C.m[1][2]) / s;
        y = (C.m[0][2] - C.m[2][0]) / s;
        z = (C.m[1][0] - C.m[0][1]) / s;
    }
    else if (C.m[0][0] > C.m[1][1] && C.m[0][0] > C.m[2][2])
    {
        const double s = std::sqrt(1.0 + C.m[0][0] - C.m[1][1] - C.m[2][2]) * 2.0;
        w = (C.m[2][1] - C.m[1][2]) / s;
        x = 0.25 * s;
        y = (C.m[0][1] + C.m[1][0]) / s;
        z = (C.m[0][2] + C.m[2][0]) / s;
    }
    else if (C.m[1][1] > C.m[2][2])
    {
        const double s = std::sqrt(1.0 + C.m[1][1] - C.m[0][0] - C.m[2][2]) * 2.0;
        w = (C.m[0][2] - C.m[2][0]) / s;
        x = (C.m[0][1] + C.m[1][0]) / s;
        y = 0.25 * s;
        z = (C.m[1][2] + C.m[2][1]) / s;
    }
    else
    {
        const double s = std::sqrt(1.0 + C.m[2][2] - C.m[0][0] - C.m[1][1]) * 2.0;
        w = (C.m[1][0] - C.m[0][1]) / s;
        x = (C.m[0][2] + C.m[2][0]) / s;
        y = (C.m[1][2] + C.m[2][1]) / s;
        z = 0.25 * s;
    }
    q[0] = float(w);
    q[1] = float(x);
    q[2] = float(y);
    q[3] = float(z);
}

// The rig: the IMU stands still on the pad (it sits on the axis the body
// turns about), the body at C0 turning about body axis ax at w rad/s from
// t_turn on; the antenna r_b from the IMU (body FRD). A filter that knows
// the attitude to start, converged on the still pad, steps through it on a
// perfect IMU; the truth's attitude at any time is C(t).
struct Rig
{
    M3 C0{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    double ax[3] = {0.0, 1.0, 0.0};
    double w = 0.0;
    double t_turn = 1e9;
    double r_b[3] = {0.0, 0.0, 0.0};
    Clock clk;
    std::vector<Sat> sky = makeSky(kLla0);
    double t = 0.0;

    M3 C(double at) const { return mul(C0, rot(ax, at > t_turn ? w * (at - t_turn) : 0.0)); }
    // one IMU step ending at t + kDt: the specific force (gravity's reaction in the body) and the rate
    void step(RawGnssEKF<float>& f)
    {
        const double t1 = t + kDt;
        const double wb = t1 > t_turn + 1e-12 ? w : 0.0;
        const M3 Cm = C(t + 0.5 * kDt);
        const double up[3] = {0.0, 0.0, -kG};
        double fb[3];
        for (int i = 0; i < 3; ++i)
            fb[i] = Cm.m[0][i] * up[0] + Cm.m[1][i] * up[1] + Cm.m[2][i] * up[2];
        const float acc[3] = {float(fb[0]), float(fb[1]), float(fb[2])};
        const float gyro[3] = {float(ax[0] * wb), float(ax[1] * wb), float(ax[2] * wb)};
        f.propagate(acc, gyro, float(kDt));
        t = t1;
        clk.tick(kDt);
    }
    void run(RawGnssEKF<float>& f, double dt_s)
    {
        const int n = static_cast<int>(std::lround(dt_s / kDt));
        for (int i = 0; i < n; ++i)
            step(f);
    }
    // the antenna now: offset and velocity, NED
    void antenna(double p_ned[3], double v_ned[3]) const
    {
        const M3 Cm = C(t);
        const double wb = t > t_turn + 1e-12 ? w : 0.0;
        const double wv[3] = {ax[0] * wb, ax[1] * wb, ax[2] * wb};
        const double wxr[3] = {wv[1] * r_b[2] - wv[2] * r_b[1], wv[2] * r_b[0] - wv[0] * r_b[2],
                               wv[0] * r_b[1] - wv[1] * r_b[0]};
        apply(Cm, r_b, p_ned);
        apply(Cm, wxr, v_ned);
    }
    // an epoch measured at the antenna now (the IMU at the pad)
    std::vector<RawMeas> epoch() const
    {
        double p_ned[3], v_ned[3], r[3], dr[3], ve[3];
        antenna(p_ned, v_ned);
        geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
        nedToEcef(kLla0[0], kLla0[1], p_ned, dr);
        nedToEcef(kLla0[0], kLla0[1], v_ned, ve);
        for (int k = 0; k < 3; ++k)
            r[k] += dr[k];
        return measure(sky, r, ve, clk);
    }
    // a filter at the pad, its attitude the truth's, given n epochs a second apart
    RawGnssEKF<float> filter(const RawGnssEKFParams& p, int epochs = 5)
    {
        RawGnssEKF<float> f(p);
        const double lla[3] = {kLla0[0] + 20.0 / 6.37e6, kLla0[1] - 10.0 / 5.35e6, kLla0[2] - 15.0};
        float q[4];
        quatOf(C(t), q);
        const float v[3] = {0.0f, 0.0f, 0.0f};
        f.init(lla, v, q, t);
        f.setQuaternion(q);
        for (int e = 0; e < epochs; ++e)
        {
            run(f, 1.0);
            const auto m = epoch();
            f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
        }
        return f;
    }
};

RawGnssEKFParams leverParams(const double r_b[3], uint8_t att_mode = gnss_raw::kLeverAttOff)
{
    RawGnssEKFParams p = quietParams();
    p.lever_on = true;
    for (int i = 0; i < 3; ++i)
        p.lever_b_m[i] = r_b[i];
    p.lever_att_mode = att_mode;
    return p;
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

// the smallest rotation from one attitude to the other, deg
double attErrDeg(const float a[4], const float b[4])
{
    double d = 0.0;
    for (int i = 0; i < 4; ++i)
        d += double(a[i]) * b[i];
    return 2.0 * std::acos(std::min(1.0, std::fabs(d))) * 180.0 / kPi;
}

}  // namespace

// ---- off: the Beetle's filter

TEST(RawGnssAntenna, OffTheFilterStepsAndUpdatesAsItAlwaysHas)
{
    // The lever's history is written only with it on, and every use of it
    // sits behind lever_on: a filter with the offset's fields set but off,
    // and one with setLever(false), step and fuse bit for bit as the default
    // filter, through a turn with Dopplers off by an antenna's swing.
    Rig rig;
    rig.w = 4.0;
    rig.t_turn = 5.2;
    rig.r_b[0] = 0.3;
    RawGnssEKFParams p = quietParams(), q = quietParams();
    q.lever_b_m[0] = 0.3;                        // set, but off
    q.lever_att_mode = gnss_raw::kLeverAttFuse;
    Rig ra = rig, rb = rig, rc = rig;
    RawGnssEKF<float> a = ra.filter(p), b = rb.filter(q), c = rc.filter(p);
    const double off[3] = {0.3, 0.0, 0.0};
    c.setLever(false, off, gnss_raw::kLeverAttFuse);
    for (int e = 0; e < 6; ++e)
    {
        ra.run(a, 0.1);
        rb.run(b, 0.1);
        rc.run(c, 0.1);
        const auto m = ra.epoch();
        const RawGnssStats sa = a.updateGnssRaw(m.data(), static_cast<int>(m.size()), 0.03);
        const RawGnssStats sb = b.updateGnssRaw(m.data(), static_cast<int>(m.size()), 0.03);
        c.updateGnssRaw(m.data(), static_cast<int>(m.size()), 0.03);
        EXPECT_EQ(sa.used_rr, sb.used_rr);
    }
    EXPECT_TRUE(sameState(a, b));
    EXPECT_TRUE(sameState(a, c));
    float pl[3], vl[3];
    a.leverAt(0.0, pl, vl);
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_EQ(pl[i], 0.0f);
        EXPECT_EQ(vl[i], 0.0f);
    }
}

// ---- the antenna's offset and velocity, at their own instant

TEST(RawGnssAntenna, TheAntennasVelocityIsTheBodysRateCrossTheOffsetTurnedIntoNed)
{
    // A nose-up body (body x up) pitching at 5 rad/s about body y from 1.0 s,
    // the antenna 0.3 m up the nose and 0.05 m along body y: C r and C (w x
    // r) at the step's end, and before the turn the offset alone, still.
    Rig rig;
    const double y[3] = {0.0, 1.0, 0.0};
    rig.C0 = rot(y, kPi / 2.0);                  // nose up: body x along NED -z
    rig.w = 5.0;
    rig.t_turn = 6.0;
    rig.r_b[0] = 0.3;
    rig.r_b[1] = 0.05;
    RawGnssEKF<float> f = rig.filter(leverParams(rig.r_b), 5);
    double p0[3], v0[3];
    rig.antenna(p0, v0);
    float pl[3], vl[3];
    f.leverAt(0.0, pl, vl);
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(pl[i], p0[i], 1e-4) << i;    // the filter's attitude and gyro bias, 1e-4 off after the pad
        EXPECT_NEAR(vl[i], 0.0, 1e-4) << i;
    }
    EXPECT_NEAR(pl[2], -0.3, 1e-4);              // up the nose: up
    rig.run(f, 1.1);                             // into the turn: 0.1 s of it
    double p1[3], v1[3];
    rig.antenna(p1, v1);
    f.leverAt(0.0, pl, vl);
    const double vmag = std::sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
    EXPECT_NEAR(vmag, 1.5, 1e-6);                // 5 rad/s x 0.3 m: the arm across the rate (the 0.05 is along it)
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(pl[i], p1[i], 2e-3) << i;    // the step's end: the filter's attitude, a few mrad of steps
        EXPECT_NEAR(vl[i], v1[i], 1e-2) << i;
    }
    // 0.15 s back is before the turn: the offset where it stood, not moving;
    // 0.05 s back, the turn's 0.05 s point
    f.leverAt(0.15, pl, vl);
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(pl[i], p0[i], 1e-4) << i;
        EXPECT_NEAR(vl[i], 0.0, 1e-4) << i;
    }
    f.leverAt(0.05, pl, vl);
    const M3 Cm = rig.C(rig.t - 0.05);
    const double wxr[3] = {5.0 * rig.r_b[2], 0.0, -5.0 * rig.r_b[0]};
    double vm[3];
    apply(Cm, wxr, vm);
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(vl[i], vm[i], 1e-2) << i;
}

// ---- the predictions at the antenna

TEST(RawGnssAntenna, KnownTheTurnsDopplersMatchTheFilterStill)
{
    // The IMU still at the pad, the body pitching at 5 rad/s with the antenna
    // 0.1 m up the nose: every Doppler carries the antenna's 0.5 m/s. Taken at
    // the IMU (off), the epoch's fitted velocity -- the vertical check's,
    // down -- is that swing against the still filter; known, nothing. Each
    // epoch is handed over 30 ms after its instant: the swing taken then, not
    // at the update.
    Rig rig;
    rig.w = 5.0;
    rig.t_turn = 5.0;
    rig.r_b[0] = 0.1;
    Rig r_off = rig, r_on = rig;
    RawGnssEKF<float> off = r_off.filter(quietParams()), on = r_on.filter(leverParams(rig.r_b));
    r_off.run(off, 0.1);
    r_on.run(on, 0.1);
    const auto m = r_on.epoch();
    double p_ned[3], v_ned[3];
    r_on.antenna(p_ned, v_ned);
    ASSERT_GT(std::fabs(v_ned[2]), 0.05);        // the swing has a vertical part to see
    r_off.run(off, 0.03);
    r_on.run(on, 0.03);
    RawGnssStats so, sn;
    off.screenRangeRates(m.data(), static_cast<int>(m.size()), 0.03, so);
    on.screenRangeRates(m.data(), static_cast<int>(m.size()), 0.03, sn);
    ASSERT_EQ(so.screen_fit_n, 10);
    ASSERT_EQ(sn.screen_fit_n, 10);
    EXPECT_NEAR(so.screen_dv_down_mps, v_ned[2], 0.02);
    EXPECT_NEAR(sn.screen_dv_down_mps, 0.0, 0.01);
    // and the update: the still filter's velocity pulled by the swing off,
    // left still known
    on.updateGnssRaw(m.data(), static_cast<int>(m.size()), 0.03);
    off.updateGnssRaw(m.data(), static_cast<int>(m.size()), 0.03);
    double von = 0.0, voff = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        von += double(on.velNed()[i]) * on.velNed()[i];
        voff += double(off.velNed()[i]) * off.velNed()[i];
    }
    EXPECT_LT(std::sqrt(von), 0.01);
    EXPECT_GT(std::sqrt(voff), 0.05);
}

TEST(RawGnssAntenna, KnownThePseudorangesPutTheImuWhereItIs)
{
    // A still pad with the antenna 2 m above the IMU (2 m up the nose of a
    // nose-up body): taken at the IMU, the filter's height converges 2 m
    // high; known, on the IMU's
    Rig rig;
    const double y[3] = {0.0, 1.0, 0.0};
    rig.C0 = rot(y, kPi / 2.0);
    rig.r_b[0] = 2.0;
    Rig r_off = rig, r_on = rig;
    RawGnssEKF<float> off = r_off.filter(quietParams(), 30), on = r_on.filter(leverParams(rig.r_b), 30);
    EXPECT_NEAR(off.lla()[2], kLla0[2] + 2.0, 0.2);
    EXPECT_NEAR(on.lla()[2], kLla0[2], 0.2);
}

TEST(RawGnssAntenna, KnownTheChangeChecksChangesAreTheAntennas)
{
    // The Doppler change check's rows from a still epoch to one 0.1 s into a
    // 5 rad/s pitch, the antenna 0.3 m up the nose: taken at the IMU, each
    // row is the swing along its line of sight (up to 1.5 m/s); known, the
    // swing is in the prediction at this end and every row is 0
    Rig rig;
    rig.w = 5.0;
    rig.t_turn = 5.0;
    rig.r_b[0] = 0.3;
    Rig r_off = rig, r_on = rig;
    RawGnssEKF<float> off = r_off.filter(quietParams()), on = r_on.filter(leverParams(rig.r_b));
    gnss_raw::DopplerRef ref_off, ref_on, out;
    gnss_raw::dvc::Row rows[gnss_raw::RawEpoch::kMaxObs];
    gnss_raw::DopplerChangeImu imu;
    const auto m0 = r_on.epoch();
    off.dopplerChange(m0.data(), static_cast<int>(m0.size()), 0.0, gnss_raw::DopplerRef(), ref_off, rows, imu);
    on.dopplerChange(m0.data(), static_cast<int>(m0.size()), 0.0, gnss_raw::DopplerRef(), ref_on, rows, imu);
    r_off.run(off, 0.1);
    r_on.run(on, 0.1);
    const auto m1 = r_on.epoch();
    const int n_off = off.dopplerChange(m1.data(), static_cast<int>(m1.size()), 0.0, ref_off, out, rows, imu);
    ASSERT_EQ(n_off, 10);
    double worst_off = 0.0;
    for (int i = 0; i < n_off; ++i)
        worst_off = std::max(worst_off, std::fabs(rows[i].z));
    EXPECT_GT(worst_off, 0.8);
    const int n_on = on.dopplerChange(m1.data(), static_cast<int>(m1.size()), 0.0, ref_on, out, rows, imu);
    ASSERT_EQ(n_on, 10);
    for (int i = 0; i < n_on; ++i)
        EXPECT_NEAR(rows[i].z, 0.0, 0.02) << i;
}

TEST(RawGnssAntenna, TheChangeChecksLeverComesBackOutOfItsFit)
{
    // Where the heading is unset, RawGnssShadow's change check takes the
    // lever's predicted change back out of the fitted change: w + (v_lev now
    // - v_lev then), each end's lever velocity as dopplerChange took it
    // (imu.v_lev). That is the unknown filter's fit: each end's prediction is
    // linear in the velocity it takes, and the fit (a velocity change and a
    // clock rate) moves by a velocity common to every row of an end, less
    // only that velocity times the line of sight's turn between the ends.
    // Two filters, the lever known and not, through a 5 rad/s pitch with the
    // antenna 0.3 m up the nose, epochs 0.05 and 0.15 s into it: the fits
    // differ by the lever's change (0.74 m/s), and taken back out by under
    // 1 mm/s; with and without a Doppler's lag.
    for (double lag : {0.0, 0.03})
    {
        SCOPED_TRACE(lag);
        Rig rig;
        rig.w = 5.0;
        rig.t_turn = 5.0;
        rig.r_b[0] = 0.3;
        RawGnssEKFParams p_off = quietParams(), p_on = leverParams(rig.r_b);
        p_off.rr_lag_s = p_on.rr_lag_s = lag;
        Rig r_off = rig, r_on = rig;
        RawGnssEKF<float> off = r_off.filter(p_off), on = r_on.filter(p_on);
        r_off.run(off, 0.05);
        r_on.run(on, 0.05);
        gnss_raw::DopplerRef ref_off, ref_on, out;
        gnss_raw::dvc::Row rows_off[gnss_raw::RawEpoch::kMaxObs], rows_on[gnss_raw::RawEpoch::kMaxObs];
        gnss_raw::DopplerChangeImu imu_off, imu_ref, imu_on;
        const auto m0 = r_on.epoch();
        off.dopplerChange(m0.data(), static_cast<int>(m0.size()), 0.0, gnss_raw::DopplerRef(), ref_off, rows_off,
                          imu_off);
        on.dopplerChange(m0.data(), static_cast<int>(m0.size()), 0.0, gnss_raw::DopplerRef(), ref_on, rows_on,
                         imu_ref);
        EXPECT_EQ(imu_off.v_lev[0], 0.0f);           // unknown: none
        r_off.run(off, 0.1);
        r_on.run(on, 0.1);
        const auto m1 = r_on.epoch();
        const int n_off =
            off.dopplerChange(m1.data(), static_cast<int>(m1.size()), 0.0, ref_off, out, rows_off, imu_off);
        const int n_on = on.dopplerChange(m1.data(), static_cast<int>(m1.size()), 0.0, ref_on, out, rows_on, imu_on);
        ASSERT_EQ(n_off, 10);
        ASSERT_EQ(n_on, 10);
        const gnss_raw::dvc::Fit f_off = gnss_raw::dvc::fit(rows_off, n_off, 5.0);
        const gnss_raw::dvc::Fit f_on = gnss_raw::dvc::fit(rows_on, n_on, 5.0);
        ASSERT_TRUE(f_off.ok);
        ASSERT_TRUE(f_on.ok);
        double dv_lev[3], wu[3];
        for (int i = 0; i < 3; ++i)
            dv_lev[i] = double(imu_on.v_lev[i]) - double(imu_ref.v_lev[i]);
        const double arm = gnss_raw::dvc::knownLeverArm(f_on.x, dv_lev, 0.3, 0.05, false, 0.17, wu);
        EXPECT_DOUBLE_EQ(arm, 0.35);                 // the whole offset and its doubt
        double moved = 0.0, back = 0.0;
        for (int i = 0; i < 3; ++i)
        {
            moved += (f_on.x[i] - f_off.x[i]) * (f_on.x[i] - f_off.x[i]);
            back = std::max(back, std::fabs(wu[i] - f_off.x[i]));
        }
        EXPECT_NEAR(std::sqrt(moved), 0.74, 0.02);   // 2 x 1.5 m/s x sin(0.25): the lever's change between the ends
        EXPECT_LT(back, 1e-3);
        EXPECT_NEAR(f_on.x[3], f_off.x[3], 1e-3);    // the clock's rate untouched
    }
}

TEST(RawGnssAntenna, TheLatchsLevelPutsTheLeverBackAsTheChangeDoes)
{
    // The change check's latch lets go on one end's Dopplers as they stand
    // (RawGnssShadow::dvLevelChi2): each less its prediction at the filter's
    // velocity -- known, at the antenna, the lever's velocity in it -- fitted
    // for one velocity offset and a clock rate. Where the heading is unset the
    // shadow puts that end's lever velocity back on the fitted offset, x +
    // v_lev (imu.v_lev, as dopplerChange took it), the sign the change's wu
    // takes: the unknown filter's level, as each row moves by the same
    // velocity along its own line of sight. 0.1 s into a 5 rad/s pitch with
    // the antenna 0.3 m up the nose: the fits differ by the lever's 1.5 m/s,
    // and put back by under 1 mm/s; with and without a Doppler's lag.
    for (double lag : {0.0, 0.03})
    {
        SCOPED_TRACE(lag);
        Rig rig;
        rig.w = 5.0;
        rig.t_turn = 5.0;
        rig.r_b[0] = 0.3;
        RawGnssEKFParams p_off = quietParams(), p_on = leverParams(rig.r_b);
        p_off.rr_lag_s = p_on.rr_lag_s = lag;
        Rig r_off = rig, r_on = rig;
        RawGnssEKF<float> off = r_off.filter(p_off), on = r_on.filter(p_on);
        r_off.run(off, 0.1);
        r_on.run(on, 0.1);
        const auto m = r_on.epoch();
        gnss_raw::DopplerRef y_off, y_on;
        gnss_raw::dvc::Row rows_off[gnss_raw::RawEpoch::kMaxObs], rows_on[gnss_raw::RawEpoch::kMaxObs];
        gnss_raw::DopplerChangeImu imu_off, imu_on;
        off.dopplerChange(m.data(), static_cast<int>(m.size()), 0.0, gnss_raw::DopplerRef(), y_off, rows_off,
                          imu_off);
        on.dopplerChange(m.data(), static_cast<int>(m.size()), 0.0, gnss_raw::DopplerRef(), y_on, rows_on, imu_on);
        ASSERT_EQ(y_off.n, 10);
        ASSERT_EQ(y_on.n, 10);
        // the level's rows, as dvLevelChi2 builds them
        auto levelFit = [](const gnss_raw::DopplerRef& y, gnss_raw::dvc::Row* r) {
            for (int i = 0; i < y.n; ++i)
            {
                for (int a = 0; a < 3; ++a)
                    r[i].u[a] = static_cast<double>(y.u[i][a]);
                r[i].z = y.y[i];
                r[i].s = static_cast<double>(y.s[i]);
                r[i].im = r[i].iref = r[i].iout = -1;
                r[i].in = true;
                r[i].e = 0.0;
            }
            return gnss_raw::dvc::fit(r, y.n, 5.0, 6, true);
        };
        const gnss_raw::dvc::Fit f_off = levelFit(y_off, rows_off), f_on = levelFit(y_on, rows_on);
        ASSERT_TRUE(f_off.ok);
        ASSERT_TRUE(f_on.ok);
        double moved = 0.0, back = 0.0;
        for (int i = 0; i < 3; ++i)
        {
            moved += (f_on.x[i] - f_off.x[i]) * (f_on.x[i] - f_off.x[i]);
            back = std::max(back, std::fabs(f_on.x[i] + double(imu_on.v_lev[i]) - f_off.x[i]));
        }
        EXPECT_NEAR(std::sqrt(moved), 1.5, 0.02);    // 5 rad/s x 0.3 m
        EXPECT_LT(back, 1e-3);
        EXPECT_NEAR(f_on.x[3], f_off.x[3], 1e-3);    // the clock's rate untouched
    }
}

namespace
{

// The Doppler change check's epoch test (RawGnssShadow::jobDvCheck,
// DopplerChange.h) on one interval set here: the antenna r_b on a body going
// from C1 to C2 while its rate goes from w1 to w2 (its largest wp), over dt,
// the IMU's specific-force change sf (NED); the filter's attitude the truth's
// turned by E at both ends, so its predictions -- the IMU's change and the
// lever's -- are turned by E. The fit's covariance 3 cm/s a side; the mosaic
// image's allowances (10 deg, 1 %, 0.1 m/s^2; the heading's any where unset).
struct LeverStep
{
    M3 C1, C2;
    double w1[3], w2[3];
    double wp, dt;
    double sf[3];
};

enum LeverDesign
{
    kUnknown,         // the offset unknown: the bound, 0.3 m
    kKnownAsItWas,    // known, the prediction kept and only its doubt allowed (01b9a163)
    kKnown,           // known: knownLeverArm
};

double leverStepChi2(const LeverStep& s, const double r_b[3], const M3& E, LeverDesign d, bool hdg_known)
{
    auto cross = [](const double a[3], const double b[3], double o[3]) {
        o[0] = a[1] * b[2] - a[2] * b[1];
        o[1] = a[2] * b[0] - a[0] * b[2];
        o[2] = a[0] * b[1] - a[1] * b[0];
    };
    double wr1[3], wr2[3], vt1[3], vt2[3], vp1[3], vp2[3], sf_f[3];
    cross(s.w1, r_b, wr1);
    cross(s.w2, r_b, wr2);
    apply(s.C1, wr1, vt1);
    apply(s.C2, wr2, vt2);
    apply(E, vt1, vp1);
    apply(E, vt2, vp2);
    apply(E, s.sf, sf_f);
    double w_unk[3], w_kn[3], dv_lev[3], dw2 = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        dv_lev[i] = vp2[i] - vp1[i];
        w_unk[i] = (s.sf[i] - sf_f[i]) + (vt2[i] - vt1[i]);
        w_kn[i] = w_unk[i] - dv_lev[i];
        dw2 += (s.w2[i] - s.w1[i]) * (s.w2[i] - s.w1[i]);
    }
    const double rates = std::sqrt(dw2) + s.wp * s.wp * s.dt;
    double Cw[3][3] = {};
    for (int i = 0; i < 3; ++i)
        Cw[i][i] = 0.03 * 0.03;
    gnss_raw::dvc::Allow a;
    a.att_rad = 10.0 * kPi / 180.0;
    a.scale = 0.01;
    a.bias_mps = 0.1 * s.dt;
    if (!hdg_known)
        a.hdg_rad = kPi;
    if (d == kUnknown)
    {
        a.lever_mps = 0.3 * rates;
        return gnss_raw::dvc::epochChi2(w_unk, Cw, sf_f, a);
    }
    if (d == kKnownAsItWas)
    {
        a.lever_mps = 0.05 * rates;
        return gnss_raw::dvc::epochChi2(w_kn, Cw, sf_f, a);
    }
    double wu[3];
    const double r = std::sqrt(r_b[0] * r_b[0] + r_b[1] * r_b[1] + r_b[2] * r_b[2]);
    a.lever_mps = gnss_raw::dvc::knownLeverArm(w_kn, dv_lev, r, 0.05, hdg_known, a.att_rad, wu) * rates;
    return gnss_raw::dvc::epochChi2(wu, Cw, sf_f, a);
}

}  // namespace

TEST(RawGnssAntenna, KnownTheChangeCheckAllowsWhatTheAttitudeDoesToThePredictedLever)
{
    // Known, each end's Dopplers are predicted at the antenna, its velocity
    // C (omega x r) at the filter's attitude: an attitude error turns the
    // predicted change. With the heading unset (no magnetometer: the V9's
    // launch) any heading can be off, up to twice the change. Allowed only
    // the offset's 5 cm of doubt (as 01b9a163 had it), a weathercocking coast
    // (the rate swinging 0.75 rad/s in 0.1 s, at most 0.6) 60 deg off failed
    // the 4-sigma test that unknown passed (0.3 m: 19.9 against 0.89), and so
    // did a drogue's snatch on a coast near vertical (3 to 5 rad/s in 0.1 s,
    // 1 g along the tail) 180 deg off (0.5 m: 47 against 0.86). Now the
    // prediction is taken back out and the whole offset allowed: each passes
    // at 0.3 and 0.5 m, as unknown does. With the heading known, an attitude
    // error within the 10 deg allowance turns the prediction by at most its
    // chord: allowed, it passes too.
    const double kE = 16.0;                      // dv_epoch_k 4, squared
    const double y[3] = {0.0, 1.0, 0.0}, down[3] = {0.0, 0.0, 1.0};
    const M3 up = rot(y, kPi / 2.0);             // nose up: body x along NED -z
    LeverStep wc{up, up, {0.0, 0.375, 0.0}, {0.0, -0.375, 0.0}, 0.6, 0.1, {0, 0, 0}};
    LeverStep dr{mul(up, rot(y, -0.2)), mul(up, rot(y, 0.2)), {0.0, 3.0, 0.0}, {0.0, 5.0, 0.0}, 5.0, 0.1, {0, 0, 0}};
    const double snatch_b[3] = {-kG * 0.1, 0.0, 0.0};
    apply(up, snatch_b, dr.sf);                  // the chute's pull along the tail, at the interval's middle
    struct Case
    {
        const char* name;
        const LeverStep* s;
        double hdg_deg, r;
    };
    const Case cases[] = {{"weathercock 0.3 m, 60 deg", &wc, 60.0, 0.3},
                          {"weathercock 0.5 m, 60 deg", &wc, 60.0, 0.5},
                          {"drogue 0.3 m, 180 deg", &dr, 180.0, 0.3},
                          {"drogue 0.5 m, 180 deg", &dr, 180.0, 0.5}};
    for (const Case& c : cases)
    {
        SCOPED_TRACE(c.name);
        const double r_b[3] = {c.r, 0.0, 0.0};
        const M3 E = rot(down, c.hdg_deg * kPi / 180.0);
        const double unk = leverStepChi2(*c.s, r_b, E, kUnknown, false);
        const double was = leverStepChi2(*c.s, r_b, E, kKnownAsItWas, false);
        const double now = leverStepChi2(*c.s, r_b, E, kKnown, false);
        EXPECT_LT(unk, kE);
        EXPECT_GT(was, kE);
        EXPECT_LT(now, kE);
        EXPECT_LE(now, unk);                     // the measured size, not the bound
    }
    // the heading known, the attitude 9 deg off about a tilted axis
    const double ax[3] = {1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0)};
    const M3 E9 = rot(ax, 9.0 * kPi / 180.0);
    for (const LeverStep* s : {&wc, &dr})
        for (double r : {0.3, 0.5})
        {
            const double r_b[3] = {r, 0.0, 0.0};
            EXPECT_LT(leverStepChi2(*s, r_b, E9, kKnown, true), 4.0) << r;
        }
}

// ---- the offset's roll terms fused (lever_att_mode fuse)

TEST(RawGnssAntenna, FusedTheLeversRollTermsTakeOutARollAboutAVerticalNose)
{
    // The rig: a nose-up body rolling about its nose, the IMU on the axis and
    // the antenna off it. Nothing else sees a turn about the vertical: the
    // filter, its heading 5 deg off, keeps the error epoch after epoch unless
    // the antenna's swing is read for it. In one epoch a roll looks just like
    // a velocity error; as the swing turns, the filter's own velocity tells
    // them apart. Fused, the terms take the error out: at 5 rad/s and 0.3 m
    // (1.5 m/s) to under 1 deg in 2 s, at 3 rad/s and 0.1 m (0.3 m/s) by
    // half; off and observe (whose counting is the change check's) keep
    // most of it. The attitude gate is open, as the shadow's act opens it in flight
    // (cos^4 of a vertical nose would hold every correction off).
    const double y[3] = {0.0, 1.0, 0.0};
    struct Case
    {
        double w, r, fused_below, fused_above;
    };
    for (const Case& c : {Case{5.0, 0.3, 1.0, 0.0}, Case{3.0, 0.1, 3.0, 1.0}})
    {
        Rig rig;
        rig.C0 = rot(y, kPi / 2.0);
        rig.ax[0] = 1.0;
        rig.ax[1] = 0.0;
        rig.w = c.w;
        rig.t_turn = 5.0;
        rig.r_b[1] = c.r;
        auto make = [&](uint8_t mode, Rig& r) {
            RawGnssEKFParams lp = leverParams(rig.r_b, mode);
            lp.att_gate_cos4 = false;
            RawGnssEKF<float> f = r.filter(lp);
            f.turnHeading(float(5.0 * kPi / 180.0));
            f.setHeadingSigma(float(10.0 * kPi / 180.0));
            return f;
        };
        Rig r_off = rig, r_obs = rig, r_fuse = rig;
        RawGnssEKF<float> off = make(gnss_raw::kLeverAttOff, r_off), obs = make(gnss_raw::kLeverAttObserve, r_obs),
                          fuse = make(gnss_raw::kLeverAttFuse, r_fuse);
        for (int e = 0; e < 20; ++e)
        {
            r_off.run(off, 0.1);
            r_obs.run(obs, 0.1);
            r_fuse.run(fuse, 0.1);
            const auto m = r_obs.epoch();
            off.updateGnssRaw(m.data(), static_cast<int>(m.size()));
            obs.updateGnssRaw(m.data(), static_cast<int>(m.size()));
            fuse.updateGnssRaw(m.data(), static_cast<int>(m.size()));
        }
        // the error is about the vertical alone: the whole of it
        auto err = [&](const RawGnssEKF<float>& f, const Rig& r) {
            float qt[4];
            quatOf(r.C(r.t), qt);
            return attErrDeg(f.quat(), qt);
        };
        EXPECT_TRUE(sameState(off, obs)) << c.w;
        EXPECT_GT(err(off, r_off), 3.5) << c.w;
        EXPECT_LT(err(fuse, r_fuse), c.fused_below) << c.w;
        EXPECT_GT(err(fuse, r_fuse), c.fused_above) << c.w;
    }
}

TEST(RawGnssAntenna, FusedTheGyroBiasTermIsTheBodysAtTheDopplersInstant)
{
    // A Doppler's lever term on the gyro bias (leverRows): a bias db turns
    // the rate the body had at the Doppler's instant, in the body as it stood
    // then -- C_l (-db x r) -- not as it stands when the epoch is fused. A
    // nose-up body rolling at 10 rad/s, the antenna 0.3 m across the nose, an
    // epoch fused 0.155 s after its instant: the body has turned 89 deg
    // since. The term, as the update fuses it, read from its gain: with the
    // biases' covariance set apart from the rest (setBiases), one Doppler
    // fused alone moves the gyro bias along P h' = sigma^2 h. Against the
    // finite difference of the antenna's range rate in the gyro bias at the
    // truth's attitude then: within a degree (the filter's own attitude, a
    // step's interpolation). The term taken with the attitude now (as
    // 01b9a163 had it, which this matched to 0.01 deg) is 82 deg off: the
    // body's turn about the nose, less the term's part along it.
    const double y[3] = {0.0, 1.0, 0.0};
    Rig rig;
    rig.C0 = rot(y, kPi / 2.0);
    rig.ax[0] = 1.0;
    rig.ax[1] = 0.0;
    rig.w = 10.0;
    rig.t_turn = 5.0;
    rig.r_b[1] = 0.3;
    RawGnssEKFParams lp = leverParams(rig.r_b, gnss_raw::kLeverAttFuse);
    lp.att_gate_cos4 = false;
    RawGnssEKF<float> f = rig.filter(lp);
    rig.run(f, 0.3);                             // into the roll
    const auto m = rig.epoch();
    const M3 C_l = rig.C(rig.t);                 // the Doppler's instant (no lag here)
    const double age = 0.155;
    rig.run(f, age);
    const M3 C_now = rig.C(rig.t);
    // GPS 3, 70 deg up: mostly along the nose, so its term is across it, where the roll turns it
    RawMeas one = m[0];
    ASSERT_EQ(one.prn, 3);
    one.has_pr = false;
    one.rr += 0.05;                              // an innovation to move the bias by
    const float ab[3] = {f.accelBias()[0], f.accelBias()[1], f.accelBias()[2]};
    const float wb[3] = {f.gyroBias()[0], f.gyroBias()[1], f.gyroBias()[2]};
    float Pb[6][6] = {};
    for (int i = 3; i < 6; ++i)
        Pb[i][i] = 0.05f * 0.05f;
    f.setBiases(ab, wb, Pb);
    RawGnssStats st;
    f.updateGnssRawPart(&one, 0, 1, age, st);
    ASSERT_EQ(st.used_rr, 1);
    double db[3];
    for (int i = 0; i < 3; ++i)
        db[i] = double(f.gyroBias()[i]) - double(wb[i]);
    // the antenna's part of the range rate, -u . C ((w - db) x r), u the line of sight (NED)
    const double el = 70.0 * kPi / 180.0;
    const double u[3] = {std::cos(el), 0.0, -std::sin(el)};
    auto rate = [&](const M3& C, const double d[3]) {
        const double wv[3] = {rig.w - d[0], -d[1], -d[2]};
        const double* r = rig.r_b;
        const double wxr[3] = {wv[1] * r[2] - wv[2] * r[1], wv[2] * r[0] - wv[0] * r[2], wv[0] * r[1] - wv[1] * r[0]};
        double v[3];
        apply(C, wxr, v);
        return -(u[0] * v[0] + u[1] * v[1] + u[2] * v[2]);
    };
    double h_l[3];
    const double eps = 1e-6;
    for (int j = 0; j < 3; ++j)
    {
        double dp[3] = {0.0, 0.0, 0.0}, dm[3] = {0.0, 0.0, 0.0};
        dp[j] = eps;
        dm[j] = -eps;
        h_l[j] = (rate(C_l, dp) - rate(C_l, dm)) / (2.0 * eps);
    }
    // the term as it was taken, -(u x p)' C with C now: the same NED vector, C_l h_l, read in the body now
    double g[3], h_now[3];
    apply(C_l, h_l, g);
    for (int j = 0; j < 3; ++j)
        h_now[j] = C_now.m[0][j] * g[0] + C_now.m[1][j] * g[1] + C_now.m[2][j] * g[2];
    auto angleDeg = [](const double a[3], const double b[3]) {
        const double ab = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        const double na = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        const double nb = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
        return std::acos(std::max(-1.0, std::min(1.0, ab / (na * nb)))) * 180.0 / kPi;
    };
    // the arm times the line of sight's part across it: at least its part along the nose, sin 70
    const double hn = std::sqrt(h_l[0] * h_l[0] + h_l[1] * h_l[1] + h_l[2] * h_l[2]);
    EXPECT_GT(hn, 0.3 * std::sin(el));
    EXPECT_LT(hn, 0.3);
    EXPECT_LT(angleDeg(db, h_l), 1.0);
    EXPECT_GT(angleDeg(db, h_now), 75.0);
}

// ---- the owner's setting into the filter's frame

TEST(RawGnssAntenna, TheBoardsSettingIsTurnedByTheMountingIntoTheFiltersFrame)
{
    // Board +X to the nose (identity): x along the nose, the board's FLU y
    // and z become FRD's -y and -z; mm to m
    const float I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    AntennaOffset a = gnss_raw::antennaOffsetInBody(AntennaOffsetConfigData{1, 312, -25, 4}, I);
    EXPECT_TRUE(a.known);
    EXPECT_DOUBLE_EQ(a.b_m[0], 0.312);
    EXPECT_DOUBLE_EQ(a.b_m[1], 0.025);
    EXPECT_DOUBLE_EQ(a.b_m[2], -0.004);
    // board +Z to the nose (a board standing on its end, component side up
    // the rocket): v_rocket = R v_board with R taking board z to rocket x
    const float Rz[9] = {0, 0, 1, 0, 1, 0, -1, 0, 0};
    a = gnss_raw::antennaOffsetInBody(AntennaOffsetConfigData{1, 10, 20, 150}, Rz);
    EXPECT_TRUE(a.known);
    EXPECT_NEAR(a.b_m[0], 0.150, 1e-9);
    EXPECT_NEAR(a.b_m[1], -0.020, 1e-9);
    EXPECT_NEAR(a.b_m[2], 0.010, 1e-9);
    // unknown, and a setting that is no setting, are unknown
    EXPECT_FALSE(gnss_raw::antennaOffsetInBody(AntennaOffsetConfigData{0, 0, 0, 0}, I).known);
    EXPECT_FALSE(gnss_raw::antennaOffsetInBody(AntennaOffsetConfigData{1, 2001, 0, 0}, I).known);
    EXPECT_FALSE(gnss_raw::antennaOffsetInBody(AntennaOffsetConfigData{2, 10, 0, 0}, I).known);
}

TEST(RawGnssAntenna, TheSettingsWireFormAndWhatTheBoardsTake)
{
    EXPECT_EQ(sizeof(AntennaOffsetConfigData), 7u);
    EXPECT_TRUE(antennaOffsetValid(AntennaOffsetConfigData{0, 0, 0, 0}));    // unknown: a fresh NVS, all zero
    EXPECT_FALSE(antennaOffsetValid(AntennaOffsetConfigData{0, 5, 0, 0}));   // unknown with a value
    EXPECT_TRUE(antennaOffsetValid(AntennaOffsetConfigData{1, 0, 0, 0}));    // measured: on the IMU
    EXPECT_TRUE(antennaOffsetValid(AntennaOffsetConfigData{1, 2000, -2000, 2000}));
    EXPECT_FALSE(antennaOffsetValid(AntennaOffsetConfigData{1, 2001, 0, 0}));
    EXPECT_FALSE(antennaOffsetValid(AntennaOffsetConfigData{1, 0, -2001, 0}));
    EXPECT_FALSE(antennaOffsetValid(AntennaOffsetConfigData{1, 0, 0, 32767}));
    EXPECT_FALSE(antennaOffsetValid(AntennaOffsetConfigData{2, 0, 0, 0}));
    // little-endian, packed: the apps' cmd 81 frame
    const AntennaOffsetConfigData a{1, 312, -25, 4};
    const uint8_t* b = reinterpret_cast<const uint8_t*>(&a);
    const uint8_t want[7] = {0x01, 0x38, 0x01, 0xE7, 0xFF, 0x04, 0x00};
    for (int i = 0; i < 7; ++i)
        EXPECT_EQ(b[i], want[i]) << i;
}

TEST(RawGnssAntenna, UnknownTheImagesConfigIsUntouchedAndKnownOnlyTheOffsetsFieldsMove)
{
    RawGnssShadow::Config base;
    gnss_raw::applyMosaicFlightSafeguards(base);
    RawGnssShadow::Config unk = base;
    gnss_raw::applyAntennaOffset(AntennaOffset(), unk);
    EXPECT_FALSE(unk.ekf.lever_on);
    EXPECT_EQ(unk.ekf.lever_att_mode, base.ekf.lever_att_mode);
    for (int i = 0; i < 3; ++i)
        EXPECT_EQ(unk.ekf.lever_b_m[i], 0.0);
    EXPECT_EQ(unk.rot_lever_axial_m, base.rot_lever_axial_m);
    EXPECT_EQ(unk.rot_lever_radial_m, base.rot_lever_radial_m);
    EXPECT_EQ(unk.rot_lever_min_rps, base.rot_lever_min_rps);
    EXPECT_EQ(unk.dv_lever_allow_m, base.dv_lever_allow_m);
    AntennaOffset k;
    k.known = true;
    k.b_m[0] = 0.10;
    k.b_m[2] = -0.02;
    RawGnssShadow::Config kn = base;
    gnss_raw::applyAntennaOffset(k, kn);
    EXPECT_TRUE(kn.ekf.lever_on);
    EXPECT_EQ(kn.ekf.lever_b_m[0], 0.10);
    EXPECT_EQ(kn.ekf.lever_b_m[1], 0.0);
    EXPECT_EQ(kn.ekf.lever_b_m[2], -0.02);
    EXPECT_EQ(kn.ekf.lever_att_mode, gnss_raw::kLeverAttObserve);
    // the bounds stay in the config (an unknown setting later takes them back); the sigmas replace them in use
    EXPECT_EQ(kn.rot_lever_axial_m, base.rot_lever_axial_m);
    EXPECT_EQ(kn.dv_lever_allow_m, base.dv_lever_allow_m);
    EXPECT_EQ(kn.antenna_sigma_axial_m, 0.05);
    EXPECT_EQ(kn.antenna_sigma_radial_m, 0.02);
    EXPECT_EQ(kn.antenna_min_rps, 0.0);
}
