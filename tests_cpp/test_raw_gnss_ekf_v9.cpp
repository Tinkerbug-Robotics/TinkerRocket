// Host tests for TR_RawGnssEKF's switches for the mosaic-G5's raw filter
// (fc/v9-raw-primary): the unaided twin (att_gnss_off), the Doppler's own
// refusal count (reject_persist_rr), the pseudorange off (setFusePr), a
// restored flight's sigmas and accelerometer bias. At its default each one is
// the Beetle's filter exactly. The sky is test_raw_gnss_ekf.cpp's: ten
// satellites (five GPS, three Galileo, two BeiDou) around a still receiver,
// their pseudoranges and range rates made with the Earth-rotation model the
// filter predicts with, a receiver clock with a bias, a drift and system
// offsets.
#include <gtest/gtest.h>

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

// The still receiver at r (ECEF) measures the sky.
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
    p.rr_lag_s = 0.0;          // this sky reports the range rate on time
    p.clk_drift_psd = 0.036;   // a free-running TCXO, not the LC86G's steered clock
    p.cr_mode = gnss_raw::kCarrierOff;
    return p;
}

template <typename T>
void initAt(RawGnssEKF<T>& f, const double lla[3], const T q[4])
{
    const T v[3] = {T(0), T(0), T(0)};
    f.init(lla, v, q, 0.0);
    f.setQuaternion(q);
}

// IMU steps over dt_s: a still body, level, that feels only gravity.
template <typename T>
void imuStill(RawGnssEKF<T>& f, double dt_s, const T gyro[3] = nullptr)
{
    const T acc[3] = {T(0), T(0), T(-kG)};
    const T w0[3] = {T(0), T(0), T(0)};
    const int steps = static_cast<int>(std::lround(dt_s / kDt));
    for (int i = 0; i < steps; ++i)
        f.propagate(acc, gyro ? gyro : w0, T(kDt));
}

// A level filter started off the truth and given n epochs a second apart.
template <typename T>
RawGnssEKF<T> converged(int epochs, Clock& clk, RawGnssEKFParams p = quietParams())
{
    p.p0_pos = 30.0;
    RawGnssEKF<T> f(p);
    const double lla[3] = {kLla0[0] + 20.0 / 6.37e6, kLla0[1] - 10.0 / 5.35e6, kLla0[2] - 15.0};
    const T q[4] = {T(1), T(0), T(0), T(0)};
    initAt(f, lla, q);
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

double attErrDeg(const double a[4], const double b[4])
{
    const double d = std::fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
    return 2.0 * std::acos(std::min(1.0, d)) * 180.0 / kPi;
}

// body -> NED: yaw about down, then a tilt about the body's x (roll), scalar first
template <typename T>
void quatYawRoll(double yaw, double roll, T q[4])
{
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2), cr = std::cos(roll / 2), sr = std::sin(roll / 2);
    q[0] = T(cy * cr);
    q[1] = T(cy * sr);
    q[2] = T(sy * sr);
    q[3] = T(sy * cr);
}

template <typename T>
void nedToBody(const T q[4], const double v[3], T out[3])
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double M[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)},
                            {2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)},
                            {2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)}};
    for (int i = 0; i < 3; ++i)
        out[i] = T(M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2]);
}

}  // namespace

// ---- att_gnss_off: the unaided twin

TEST(RawGnssEKFV9, TheUnaidedTwinsAttitudeIsTheImusAlone)
{
    // A still receiver, the filter started 3 deg off in roll with a gyro bias
    // it does not know: GNSS sees the tilt through the velocity it drives, and
    // the aided filter rights it. With att_gnss_off the same updates fuse the
    // position, velocity and clock (and the accelerometer bias takes the
    // tilt's pull) but never the attitude or the gyro bias: their estimates
    // and covariance stay what the IMU alone made of them, a filter given no
    // satellites at all.
    const auto sky = makeSky(kLla0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    double q0[4];
    quatYawRoll(0.0, 3.0 * kPi / 180.0, q0);
    const double truth[4] = {1.0, 0.0, 0.0, 0.0};
    const double gyro[3] = {0.0005, -0.0003, 0.0002};    // rad/s, the IMU's own bias
    // the tilt the start's p0_att leaves free, an accelerometer bias the
    // pad's long levelling has pinned (so GNSS can tell the two apart)
    RawGnssEKFParams pa = quietParams();
    pa.p0_abias = 0.02;
    RawGnssEKFParams pu = pa;
    pu.att_gnss_off = true;
    RawGnssEKF<double> unaided(pu), aided(pa), imu_only(pu);
    const double v0[3] = {0.0, 0.0, 0.0};
    for (RawGnssEKF<double>* f : {&unaided, &aided, &imu_only})
        f->init(kLla0, v0, q0, 0.0);
    Clock clk;
    int used = 0;
    for (int e = 0; e < 20; ++e)
    {
        for (RawGnssEKF<double>* f : {&unaided, &aided, &imu_only})
            imuStill(*f, 1.0, gyro);
        clk.tick(1.0);
        const auto m = measure(sky, r, clk);
        const RawGnssStats st = unaided.updateGnssRaw(m.data(), static_cast<int>(m.size()));
        used += st.used_pr + st.used_rr;
        aided.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    }
    EXPECT_GT(used, 300);                                 // GNSS was fused, every satellite most epochs
    // the attitude: the IMU's alone (the inject's renormalization aside), and
    // the gyro bias and the attitude and gyro-bias covariance exactly so
    EXPECT_LT(attErrDeg(unaided.quat(), imu_only.quat()), 1e-9);
    for (int i = 0; i < 3; ++i)
        EXPECT_EQ(unaided.gyroBias()[i], imu_only.gyroBias()[i]);
    constexpr int kAtt = RawGnssEKF<double>::kAtt, kGb = RawGnssEKF<double>::kGyroBias;
    const int rows[6] = {kAtt, kAtt + 1, kAtt + 2, kGb, kGb + 1, kGb + 2};
    for (int a : rows)
        for (int b : rows)
            EXPECT_EQ(unaided.P(a, b), imu_only.P(a, b)) << a << ", " << b;
    // the aided filter took the tilt out; the unaided one is still off by it
    EXPECT_LT(attErrDeg(aided.quat(), truth), 1.0);
    EXPECT_GT(attErrDeg(unaided.quat(), truth), 2.5);
    // and the unaided one's position still the satellites'
    double ru[3], vu[3];
    unaided.receiverEcef(ru, vu);
    const double d = std::sqrt((ru[0] - r[0]) * (ru[0] - r[0]) + (ru[1] - r[1]) * (ru[1] - r[1])
                               + (ru[2] - r[2]) * (ru[2] - r[2]));
    EXPECT_LT(d, 1.0);
}

TEST(RawGnssEKFV9, TheUnaidedTwinsGnssLeavesTheAttitudeBitForBitInFloat)
{
    // the flight's scalar: an epoch's updates leave the gyro bias and the
    // attitude's and gyro bias's covariance exactly as they were
    Clock clk;
    RawGnssEKFParams p = quietParams();
    p.att_gnss_off = true;
    RawGnssEKF<float> f = converged<float>(5, clk, p);
    imuStill(f, 1.0);
    clk.tick(1.0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    auto m = measure(makeSky(kLla0), r, clk);
    for (auto& x : m)
        x.rr += 0.03;                                   // a little for the update to do
    const RawGnssEKF<float> before = f;
    const RawGnssStats st = f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    EXPECT_EQ(st.used_pr, 10);
    EXPECT_EQ(st.used_rr, 10);
    for (int i = 0; i < 3; ++i)
        EXPECT_EQ(f.gyroBias()[i], before.gyroBias()[i]);
    for (int i = 0; i < 4; ++i)
        EXPECT_NEAR(f.quat()[i], before.quat()[i], 1e-6f);
    constexpr int kAtt = RawGnssEKF<float>::kAtt, kGb = RawGnssEKF<float>::kGyroBias;
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
        {
            EXPECT_EQ(f.P(kAtt + a, kAtt + b), before.P(kAtt + a, kAtt + b));
            EXPECT_EQ(f.P(kGb + a, kGb + b), before.P(kGb + a, kGb + b));
            EXPECT_EQ(f.P(kAtt + a, kGb + b), before.P(kAtt + a, kGb + b));
        }
    // the velocity, though, took the satellites' word
    bool moved = false;
    for (int i = 0; i < 3; ++i)
        moved = moved || f.velNed()[i] != before.velNed()[i];
    EXPECT_TRUE(moved);
}

TEST(RawGnssEKFV9, TheUnaidedTwinStillLevelsAndTakesTheMagnetometer)
{
    // GNSS kept off the attitude only at the GNSS update: the levelling and
    // the magnetometer (scalarUpdate's other callers) still right a start 30
    // deg off in heading and 4 deg in roll, with epochs fused in between
    const double b_ned[3] = {21.0, -4.3, 45.5};
    const float decl = float(std::atan2(b_ned[1], b_ned[0]));
    float q_true[4], q_wrong[4];
    quatYawRoll(44.7 * kPi / 180.0, 0.0, q_true);
    quatYawRoll((44.7 + 30.0) * kPi / 180.0, 4.0 * kPi / 180.0, q_wrong);
    RawGnssEKFParams p = quietParams();
    p.att_gnss_off = true;
    RawGnssEKF<float> f(p);
    const float v0[3] = {0, 0, 0};
    f.init(kLla0, v0, q_wrong, 0.0);
    const double g_ned[3] = {0.0, 0.0, -kG};
    float acc[3], mag[3];
    nedToBody(q_true, g_ned, acc);
    nedToBody(q_true, b_ned, mag);
    const float gyro[3] = {0, 0, 0};
    const auto sky = makeSky(kLla0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    Clock clk;
    int fused = 0, used = 0;
    for (int k = 1; k <= 250; ++k)                       // 5 s at 50 Hz
    {
        f.propagate(acc, gyro, 0.02f);
        f.updateAccelLevel(acc);
        fused += f.updateMagHeading(acc, mag, true, decl, 0.0f) ? 1 : 0;
        if (k % 50 == 0)                                 // and an epoch a second
        {
            clk.tick(1.0);
            const auto m = measure(sky, r, clk);
            const RawGnssStats st = f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
            used += st.used_pr + st.used_rr;
        }
    }
    EXPECT_EQ(fused, 250);
    EXPECT_GT(used, 0);
    double qa[4], qb[4];
    for (int i = 0; i < 4; ++i)
    {
        qa[i] = f.quat()[i];
        qb[i] = q_true[i];
    }
    EXPECT_LT(attErrDeg(qa, qb), 1.0);
}

// ---- reject_persist_rr: the Doppler's let-in in time at 10 Hz

namespace
{

// Ten epochs a second on a converged filter, satellite 9's Doppler (fused
// last) 5 m/s off and satellite 2's pseudorange 200 m off. Each epoch: the
// boost screen's gate first (screenRangeRates), then the update. Returns the
// epochs (from 1) at which each is first let in, and the screen's verdict on
// the Doppler then.
struct LetIn
{
    int rr_epoch = -1, pr_epoch = -1;
    int screen_refused_at_rr = -1;      // the screen at the Doppler's let-in epoch: refused (1) or let in (0)
    int screen_refused_before = 0;      // the screen's refusals of it before that
};

LetIn tenHertz(RawGnssEKFParams p, int epochs = 40)
{
    Clock clk;
    RawGnssEKF<double> f = converged<double>(10, clk, p);
    const auto sky = makeSky(kLla0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    LetIn out;
    for (int e = 1; e <= epochs && (out.rr_epoch < 0 || out.pr_epoch < 0); ++e)
    {
        imuStill(f, 0.1);
        clk.tick(0.1);
        auto m = measure(sky, r, clk);
        m[9].rr += 5.0;
        m[2].pr += 200.0;
        const int n = static_cast<int>(m.size());
        RawGnssStats st;
        if (!f.beginGnssRaw(m.data(), n, 0.0, st))
            continue;
        RawGnssStats sc;
        f.screenRangeRates(&m[9], 1, 0.0, sc);
        f.updateGnssRawPart(m.data(), 0, n, 0.0, st);
        if (out.rr_epoch < 0)
        {
            if (st.rejected_rr == 0)
            {
                out.rr_epoch = e;
                out.screen_refused_at_rr = sc.screen_refused;
            }
            else
            {
                out.screen_refused_before += sc.screen_refused;
            }
        }
        if (out.pr_epoch < 0 && st.rejected_pr == 0)
            out.pr_epoch = e;
    }
    return out;
}

}  // namespace

TEST(RawGnssEKFV9, AtTenHertzTheDopplersOwnCountLetsARefusalInAfterThreeSeconds)
{
    // The Beetle's reject_persist (3 epochs: 3 s at its 1 Hz) at 10 Hz would
    // let a refused Doppler in after 0.3 s. reject_persist_rr = 30: after 30
    // refusals, 3.0 s -- and the boost screen's gate lets it in at that very
    // epoch, as the update does. The pseudorange keeps reject_persist's 3.
    RawGnssEKFParams p = quietParams();
    p.reject_persist_rr = 30;
    const LetIn a = tenHertz(p);
    EXPECT_EQ(a.rr_epoch, 31);                           // 30 epochs refused: 3.0 s
    EXPECT_EQ(a.screen_refused_before, 30);
    EXPECT_EQ(a.screen_refused_at_rr, 0);
    EXPECT_EQ(a.pr_epoch, 4);

    // the default: the Doppler on reject_persist's count too, 0.3 s
    const LetIn d = tenHertz(quietParams());
    EXPECT_EQ(d.rr_epoch, 4);
    EXPECT_EQ(d.screen_refused_before, 3);
    EXPECT_EQ(d.screen_refused_at_rr, 0);
    EXPECT_EQ(d.pr_epoch, 4);
}

TEST(RawGnssEKFV9, BelowZeroTheDopplerFollowsRejectPersist)
{
    // -1 follows reject_persist wherever it is set; the Beetle's 3 given
    // explicitly flies the same filter bit for bit
    RawGnssEKFParams p = quietParams();
    p.reject_persist = 5;
    const LetIn five = tenHertz(p);
    EXPECT_EQ(five.rr_epoch, 6);
    EXPECT_EQ(five.pr_epoch, 6);

    RawGnssEKFParams q3 = quietParams();
    q3.reject_persist_rr = 3;
    Clock ca, cb;
    RawGnssEKF<double> a = converged<double>(10, ca, quietParams()), b = converged<double>(10, cb, q3);
    const auto sky = makeSky(kLla0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    for (int e = 0; e < 12; ++e)
    {
        imuStill(a, 0.1);
        imuStill(b, 0.1);
        ca.tick(0.1);
        auto m = measure(sky, r, ca);
        m[9].rr += 5.0;
        a.updateGnssRaw(m.data(), static_cast<int>(m.size()));
        b.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    }
    for (int i = 0; i < RawGnssEKF<double>::N; ++i)
        for (int j = 0; j < RawGnssEKF<double>::N; ++j)
            ASSERT_EQ(a.P(i, j), b.P(i, j));
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_EQ(a.lla()[i], b.lla()[i]);
        EXPECT_EQ(a.velNed()[i], b.velNed()[i]);
    }
}

// ---- setFusePr: the pseudoranges once a second from 10 Hz epochs

TEST(RawGnssEKFV9, WithThePseudorangeOffAnEpochLeavesTheCodeAsIfItNeverCame)
{
    // Pseudoranges alone at 10 Hz, fused only on the whole seconds, one
    // satellite 200 m off: the filter is the one that saw only the
    // whole-second epochs, bit for bit -- each satellite's code refusals and
    // last update stood through the epochs between (the bad one let in at the
    // fourth whole second, not after three tenths), and its correlated
    // weighting is the time since its last fused update, a second.
    RawGnssEKFParams p = quietParams();
    Clock ca;
    RawGnssEKF<double> tenth = converged<double>(3, ca, p);
    RawGnssEKF<double> whole = tenth;
    const auto sky = makeSky(kLla0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    int let_in_s = -1;
    for (int k = 1; k <= 60; ++k)                        // 6 s at 10 Hz
    {
        imuStill(tenth, 0.1);
        imuStill(whole, 0.1);
        ca.tick(0.1);
        auto m = measure(sky, r, ca);
        for (auto& x : m)
        {
            x.has_rr = false;
            x.sigma_pr_corr = 1.0f;                      // a correlated part for the time since to weigh
        }
        m[2].pr += 200.0;
        const int n = static_cast<int>(m.size());
        const bool on_second = k % 10 == 0;
        tenth.setFusePr(on_second);
        const RawGnssStats st = tenth.updateGnssRaw(m.data(), n);
        if (on_second)
        {
            const RawGnssStats sw = whole.updateGnssRaw(m.data(), n);
            EXPECT_EQ(st.used_pr, sw.used_pr) << k;
            EXPECT_EQ(st.rejected_pr, sw.rejected_pr) << k;
            if (let_in_s < 0 && st.rejected_pr == 0)
                let_in_s = k / 10;
        }
        else
        {
            EXPECT_EQ(st.used_pr + st.rejected_pr, 0) << k;
        }
    }
    EXPECT_EQ(let_in_s, 4);
    tenth.setFusePr(true);
    for (int i = 0; i < RawGnssEKF<double>::N; ++i)
        for (int j = 0; j < RawGnssEKF<double>::N; ++j)
            ASSERT_EQ(tenth.P(i, j), whole.P(i, j)) << i << ", " << j;
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_EQ(tenth.lla()[i], whole.lla()[i]);
        EXPECT_EQ(tenth.velNed()[i], whole.velNed()[i]);
    }
    EXPECT_EQ(tenth.clockBias(), whole.clockBias());
}

TEST(RawGnssEKFV9, WithThePseudorangeOffTheDopplersFuseAndTheClockStillStepsAMillisecond)
{
    Clock clk;
    RawGnssEKF<double> f = converged<double>(10, clk);
    const auto sky = makeSky(kLla0);
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    imuStill(f, 0.1);
    clk.tick(0.1);
    clk.bias -= kC * 1e-3;                               // the mosaic's -1 ms clock jump
    auto m = measure(sky, r, clk);
    f.setFusePr(false);
    EXPECT_FALSE(f.fusePr());
    RawGnssStats st = f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    EXPECT_EQ(st.clock_jump_ms, -1);                     // the epoch's pseudoranges still step the clock
    EXPECT_EQ(st.used_pr + st.rejected_pr, 0);
    EXPECT_EQ(st.used_rr, 10);
    EXPECT_NEAR(f.clockBias(), clk.bias, 1.0);
    // the next fused epoch finds every pseudorange where it predicts it
    imuStill(f, 0.1);
    clk.tick(0.1);
    m = measure(sky, r, clk);
    f.setFusePr(true);
    st = f.updateGnssRaw(m.data(), static_cast<int>(m.size()));
    EXPECT_EQ(st.clock_jump_ms, 0);
    EXPECT_EQ(st.used_pr, 10);
    EXPECT_EQ(st.rejected_pr, 0);
    EXPECT_NEAR(f.clockBias(), clk.bias, 0.5);
}

TEST(RawGnssEKFV9, ACopyTakesThePseudorangeSwitchAlong)
{
    // the boost screen works an epoch through on a copy (dry_ = ekf_)
    RawGnssEKF<float> f(quietParams());
    f.setFusePr(false);
    const RawGnssEKF<float> copy = f;
    EXPECT_FALSE(copy.fusePr());
    EXPECT_TRUE(RawGnssEKF<float>(quietParams()).fusePr());
}

// ---- a restored flight's start

namespace
{

template <typename T>
void checkRestoreSigmas()
{
    Clock clk;
    RawGnssEKF<T> f = converged<T>(10, clk);
    imuStill(f, 0.5);                                    // correlations everywhere
    const RawGnssEKF<T> before = f;
    constexpr int N = RawGnssEKF<T>::N, kPos = RawGnssEKF<T>::kPos, kVel = RawGnssEKF<T>::kVel,
                  kAtt = RawGnssEKF<T>::kAtt;
    const double att = 15.0 * kPi / 180.0;
    f.inflateRestoreSigmas(T(100), T(20), T(att));
    auto in = [](int i, int base) { return i >= base && i < base + 3; };
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
        {
            const bool ri = in(i, kPos) || in(i, kVel) || in(i, kAtt);
            const bool rj = in(j, kPos) || in(j, kVel) || in(j, kAtt);
            if (ri && i == j)
            {
                const double want = in(i, kPos) ? 1.0e4 : (in(i, kVel) ? 400.0 : 0.25 * att * att);
                EXPECT_NEAR(double(f.P(i, i)), want, 1e-6 * want) << i;
            }
            else if (ri || rj)
            {
                EXPECT_EQ(f.P(i, j), T(0)) << i << ", " << j;
            }
            else
            {
                EXPECT_EQ(f.P(i, j), before.P(i, j)) << i << ", " << j;
            }
        }
    // 15 deg about north, east and down
    T s[3];
    f.attSigmaNed(s);
    for (int a = 0; a < 3; ++a)
        EXPECT_NEAR(double(s[a]) * 180.0 / kPi, 15.0, 1e-3);
    // a block whose sigma is 0 is left as it was
    RawGnssEKF<T> g = before;
    g.inflateRestoreSigmas(T(0), T(20), T(-1));
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            if (!in(i, kVel) && !in(j, kVel))
                EXPECT_EQ(g.P(i, j), before.P(i, j)) << i << ", " << j;
    EXPECT_NEAR(double(g.P(kVel, kVel)), 400.0, 1e-3);
    EXPECT_EQ(g.P(kVel, kPos), T(0));
    // the state itself untouched
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_EQ(f.lla()[i], before.lla()[i]);
        EXPECT_EQ(f.velNed()[i], before.velNed()[i]);
    }
}

}  // namespace

TEST(RawGnssEKFV9, ARestoredStartsSigmasAreSetAndTheirCrossTermsDropped)
{
    checkRestoreSigmas<float>();
    checkRestoreSigmas<double>();
}

TEST(RawGnssEKFV9, ARestoredStartTakesTheSnapshotsAccelerometerBias)
{
    Clock clk;
    RawGnssEKF<float> f = converged<float>(5, clk);
    const RawGnssEKF<float> before = f;
    const float ab[3] = {0.12f, -0.07f, 0.3f};
    f.setAccelBias(ab);
    for (int i = 0; i < 3; ++i)
        EXPECT_EQ(f.accelBias()[i], ab[i]);
    for (int i = 0; i < RawGnssEKF<float>::N; ++i)
        for (int j = 0; j < RawGnssEKF<float>::N; ++j)
            ASSERT_EQ(f.P(i, j), before.P(i, j));
    // the IMU's specific force less it from then on: a still body that reads
    // the bias on top of gravity stays still
    const float acc[3] = {ab[0], ab[1], float(-kG) + ab[2]};
    const float gyro[3] = {0, 0, 0};
    for (int k = 0; k < 200; ++k)
        f.propagate(acc, gyro, float(kDt));
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(f.velNed()[i] - before.velNed()[i], 0.0f, 0.01f);
}
