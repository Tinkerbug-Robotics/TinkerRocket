// Host tests for the carrier range rate in TR_RawGnssEKF (#1600): a
// synthetic sky over a receiver near Spaceport America whose carrier phases
// are exact ranges plus the receiver clock (a drift and white frequency
// noise), a common carrier wind-up, a per-satellite integer and phase noise --
// and its pseudoranges and Dopplers beside them, noisier, as the LC86G's are.
// Observe never moves the state; fusing beats the Doppler alone, at constant
// velocity and through manoeuvres; the screen takes a half-cycle step out or
// repairs it; a correction between epochs (a barometer, zero velocity,
// levelling) does not bias the next epoch's rates; float tracks double; the
// carrier goes through the GNSS updates' attitude gate. (The builder's side --
// the gates, a lock reset, a gap -- is test_raw_gnss_meas.cpp's.)
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

#include "RawMeas.h"
#include "ReceiverProfile.h"
#include "TR_RawGnssEKF.h"
#include "gnss/atmo.h"
#include "gnss/types.h"

using gnss_raw::CarrierInfo;
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
constexpr double kDt = 0.005;                      // a 200 Hz IMU
constexpr double kL1 = kC / GNSS_FREQ_L1_HZ;       // 0.19 m
const double kLla0[3] = {32.99 * kPi / 180.0, -106.97 * kPi / 180.0, 1400.0};

// Normal draws the same on every platform: Box-Muller on mt19937's own words
// (test_heading_fit.cpp's).
class PortableNormal
{
public:
    explicit PortableNormal(double sigma) : sigma_(sigma) {}
    double operator()(std::mt19937& rng)
    {
        if (have_next_)
        {
            have_next_ = false;
            return sigma_ * next_;
        }
        const double u1 = (static_cast<double>(rng()) + 0.5) / 4294967296.0;
        const double u2 = (static_cast<double>(rng()) + 0.5) / 4294967296.0;
        const double r = std::sqrt(-2.0 * std::log(u1));
        next_ = r * std::sin(2.0 * kPi * u2);
        have_next_ = true;
        return sigma_ * r * std::cos(2.0 * kPi * u2);
    }

private:
    double sigma_;
    double next_ = 0.0;
    bool   have_next_ = false;
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

void ecefToNed(double lat, double lon, const double e[3], double ned[3])
{
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    ned[0] = -sl * co * e[0] - sl * so * e[1] + cl * e[2];
    ned[1] = -so * e[0] + co * e[1];
    ned[2] = -cl * co * e[0] - cl * so * e[1] - sl * e[2];
}

// A satellite on a straight line through the sky at its orbit's speed (and
// the pull toward the Earth): the range it gives changes by hundreds of
// metres a second, as a real one's.
struct Sat
{
    int sys, prn;
    double pos0[3], vel[3], acc[3];
    void at(double t, double p[3], double v[3]) const
    {
        for (int k = 0; k < 3; ++k)
        {
            p[k] = pos0[k] + vel[k] * t + 0.5 * acc[k] * t * t;
            v[k] = vel[k] + acc[k] * t;
        }
    }
};

std::vector<Sat> makeSky()
{
    struct AzEl { int sys, prn; double az, el; };
    const AzEl dirs[] = {{GNSS_SYS_GPS, 3, 0, 70},   {GNSS_SYS_GPS, 8, 60, 35},   {GNSS_SYS_GPS, 14, 130, 20},
                         {GNSS_SYS_GPS, 22, 200, 50}, {GNSS_SYS_GPS, 31, 290, 25}, {GNSS_SYS_GAL, 4, 30, 15},
                         {GNSS_SYS_GAL, 11, 170, 60}, {GNSS_SYS_GAL, 26, 250, 40}, {GNSS_SYS_BDS, 23, 100, 45},
                         {GNSS_SYS_BDS, 37, 320, 55}};
    double r[3];
    geo_to_ecef(kLla0[0], kLla0[1], kLla0[2], r);
    std::vector<Sat> sky;
    for (const AzEl& d : dirs)
    {
        const double az = d.az * kPi / 180.0, el = d.el * kPi / 180.0;
        const double enu[3] = {std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el)};
        double u[3];
        enuToEcef(kLla0[0], kLla0[1], enu, u);
        const double R = 26.56e6, ru = r[0] * u[0] + r[1] * u[1] + r[2] * u[2];
        const double rr = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
        const double rho = -ru + std::sqrt(ru * ru - rr + R * R);
        Sat s{d.sys, d.prn, {}, {}, {}};
        for (int k = 0; k < 3; ++k)
            s.pos0[k] = r[k] + rho * u[k];
        const double z[3] = {0.3, -0.2, 1.0};
        double v[3] = {s.pos0[1] * z[2] - s.pos0[2] * z[1], s.pos0[2] * z[0] - s.pos0[0] * z[2],
                       s.pos0[0] * z[1] - s.pos0[1] * z[0]};
        const double vn = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        for (int k = 0; k < 3; ++k)
        {
            s.vel[k] = 3870.0 * v[k] / vn;
            s.acc[k] = -3.986004418e14 * s.pos0[k] / (R * R * R);
        }
        sky.push_back(s);
    }
    return sky;
}

// The range with the Earth's rotation over the travel time (exact turn), and
// its rate for a receiver at r moving at v.
void rangeAndRate(const double rs[3], const double vs[3], const double r[3], const double v[3], double& rho,
                  double& rate)
{
    rho = std::sqrt((rs[0] - r[0]) * (rs[0] - r[0]) + (rs[1] - r[1]) * (rs[1] - r[1]) + (rs[2] - r[2]) * (rs[2] - r[2]));
    double c = 1.0, s = 0.0, x[3] = {rs[0], rs[1], rs[2]};
    for (int it = 0; it < 3; ++it)
    {
        const double th = kOmegaE * rho / kC;
        c = std::cos(th);
        s = std::sin(th);
        x[0] = c * rs[0] + s * rs[1];
        x[1] = -s * rs[0] + c * rs[1];
        x[2] = rs[2];
        rho = std::sqrt((x[0] - r[0]) * (x[0] - r[0]) + (x[1] - r[1]) * (x[1] - r[1]) + (x[2] - r[2]) * (x[2] - r[2]));
    }
    const double vr[3] = {c * vs[0] + s * vs[1], -s * vs[0] + c * vs[1], vs[2]};
    rate = 0.0;
    for (int k = 0; k < 3; ++k)
        rate += (x[k] - r[k]) / rho * (vr[k] - v[k]);
}

// The receiver's path: from kLla0 at v0 (NED), accelerating at a0 plus a
// swing of amplitude swing (m/s^2) at 0.2 Hz across the first two axes.
struct Path
{
    double v0[3] = {0.0, 0.0, 0.0};
    double a0[3] = {0.0, 0.0, 0.0};
    double swing = 0.0;
    void acc(double t, double a[3]) const
    {
        const double w = 2.0 * kPi * 0.2;
        a[0] = a0[0] + swing * std::cos(w * t);
        a[1] = a0[1] + swing * std::sin(w * t);
        a[2] = a0[2];
    }
};

// The truth, stepped with the IMU: the filter's own mechanization (NED on
// the ellipsoid, the body's attitude held in it, constant gravity) taken
// exactly -- each step's mean velocity moving the position, where the filter
// steps it on the step's end velocity. A flat-earth truth would turn away
// from the filter's local frame by a kilometre's 1.6e-4 rad, enough for the
// carrier to see.
struct Truth
{
    double lla[3] = {kLla0[0], kLla0[1], kLla0[2]};
    double v[3] = {0.0, 0.0, 0.0};
    double t = 0.0;
    void step(const double a[3], double dt)
    {
        double vm[3];
        for (int i = 0; i < 3; ++i)
        {
            vm[i] = v[i] + 0.5 * a[i] * dt;
            v[i] += a[i] * dt;
        }
        const double s = std::sin(lla[0]);
        const double d = std::fabs(1.0 - 0.0066943799901 * s * s);
        const double rew = 6378137.0 / std::sqrt(d), rns = 6378137.0 * (1.0 - 0.0066943799901) / (d * std::sqrt(d));
        lla[0] += dt * vm[0] / (rns + lla[2]);
        lla[1] += dt * vm[1] / ((rew + lla[2]) * std::cos(lla[0]));
        lla[2] -= dt * vm[2];
        t += dt;
    }
    double height() const { return lla[2]; }
    void ecef(double r[3], double ve[3]) const
    {
        geo_to_ecef(lla[0], lla[1], lla[2], r);
        nedToEcef(lla[0], lla[1], v, ve);
    }
};

// The receiver's clock and carrier: a bias walking at its drift plus white
// frequency noise (a step of fm_sigma each second), a wind-up common to every
// satellite, and each satellite's integer.
struct Receiver
{
    double clk_m = 1234.5, drift_mps = -2.5;
    double fm_sigma_m = 0.1;          // the clock's white FM over a second, m
    double wind_cyc_s = 0.5;          // the common wind-up, cycles a second
    double wind_cyc = 0.0;
    double phase_sigma_m = 0.004;     // each phase's noise
    double pr_sigma_m = 1.0, rr_sigma_mps = 0.1;
    // an epoch a second on
    void tick(std::mt19937& rng, PortableNormal& fm)
    {
        clk_m += drift_mps + fm(rng) * (fm_sigma_m > 0.0 ? 1.0 : 0.0);
        wind_cyc += wind_cyc_s;
    }
    // the rate the carrier's common part runs at over the last second, given the clock's step
    double commonRate(double dclk) const { return dclk + wind_cyc_s * kL1; }
};

// One epoch's measurements of the sky at time t by the truth (r, v ECEF),
// with the carrier range rate since the epoch before (phase_prev, sat_prev:
// each satellite's phase and position then; none on the first).
struct EpochGen
{
    std::vector<Sat> sky = makeSky();
    Receiver rx;
    std::mt19937 rng{20261015};
    PortableNormal n1{1.0};
    PortableNormal fm{0.1};
    std::vector<double> phase_prev;
    std::vector<std::array<double, 3>> sat_prev;
    bool have_prev = false;

    std::vector<RawMeas> epoch(double t, const double r[3], const double v[3])
    {
        std::vector<RawMeas> out;
        std::vector<double> phase(sky.size());
        std::vector<std::array<double, 3>> satp(sky.size());
        for (size_t i = 0; i < sky.size(); ++i)
        {
            const Sat& s = sky[i];
            RawMeas m{};
            m.sys = static_cast<uint8_t>(s.sys);
            m.prn = static_cast<uint8_t>(s.prn);
            double sp[3], sv[3];
            s.at(t, sp, sv);
            for (int k = 0; k < 3; ++k)
            {
                m.sat_pos[k] = sp[k];
                m.sat_vel[k] = sv[k];
                m.sat_acc[k] = s.acc[k];
                satp[i][k] = sp[k];
            }
            double rho, rate;
            rangeAndRate(sp, sv, r, v, rho, rate);
            const double isb = s.sys == GNSS_SYS_GAL ? 12.0 : (s.sys == GNSS_SYS_BDS ? -25.0 : 0.0);
            m.has_pr = m.has_rr = true;
            m.pr = rho + rx.clk_m + isb + rx.pr_sigma_m * n1(rng);
            m.rr = rate + rx.drift_mps + rx.rr_sigma_mps * n1(rng);
            m.sigma_pr = static_cast<float>(rx.pr_sigma_m);
            m.sigma_pr_corr = 0.0f;
            m.tau_pr = 15.0f;
            m.sigma_rr = static_cast<float>(rx.rr_sigma_mps);
            m.tau_rr = 0.1f;
            const double lam = s.sys == GNSS_SYS_BDS ? kC / 1561.098e6 : kL1;
            phase[i] = rho + rx.clk_m + lam * (37.0 + s.prn + rx.wind_cyc) + rx.phase_sigma_m * n1(rng);
            if (have_prev)
            {
                m.has_cr = true;
                m.cr_dt = 1.0f;
                m.cr_dphi_m = phase[i] - phase_prev[i];
                for (int k = 0; k < 3; ++k)
                    m.sat_pos_prev[k] = sat_prev[i][k];
                m.sigma_cr = 0.008f;
            }
            out.push_back(m);
        }
        phase_prev = phase;
        sat_prev = satp;
        have_prev = true;
        return out;
    }
};

RawGnssEKFParams params(uint8_t mode)
{
    RawGnssEKFParams p;          // the LC86G's
    p.rr_lag_s = 0.0;            // this sky's Doppler is on time
    p.cr_mode = mode;
    // the IMU's biases as minutes on the pad leave them (the levelling, the
    // still gyro's mean), not the start's 1 m/s^2 and 1 deg/s: with those,
    // a level body's attitude wanders a degree or two under the Dopplers and
    // a manoeuvre turns that into a velocity error that grows through each
    // second -- which the carrier's velocity-only H takes as steady
    p.p0_abias = 0.05;
    p.p0_wbias = 0.0005;
    return p;
}

// A body at the fixed attitude q (body FRD -> NED, scalar first; level: 1)
// feels a - g: its specific force in the body.
void specificForce(const double q[4], const double a_ned[3], double f_b[3])
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double M[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)},
                            {2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)},
                            {2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)}};   // NED -> body
    const double fn[3] = {a_ned[0], a_ned[1], a_ned[2] - kG};
    for (int i = 0; i < 3; ++i)
        f_b[i] = M[i][0] * fn[0] + M[i][1] * fn[1] + M[i][2] * fn[2];
}

// A filter on the path: its start a little off the truth, an epoch each
// second applied age_s late. Returns its velocity error at each epoch (after
// the update, against the truth at the filter's time) and the epochs' stats.
template <typename T>
struct Flight
{
    RawGnssEKF<T> f;
    Truth truth;
    EpochGen gen;
    Path path;
    double q[4] = {1.0, 0.0, 0.0, 0.0};
    double age_s = 0.0;
    std::vector<double> verr;
    std::vector<RawGnssStats> stats;

    Flight(const RawGnssEKFParams& p, const Path& pa, const double* q_body = nullptr) : f(p), path(pa)
    {
        if (q_body)
            for (int i = 0; i < 4; ++i)
                q[i] = q_body[i];
        for (int i = 0; i < 3; ++i)
            truth.v[i] = pa.v0[i];
        const double lla[3] = {kLla0[0] + 3.0 / 6.37e6, kLla0[1] - 2.0 / 5.35e6, kLla0[2] + 4.0};
        const T v0[3] = {T(pa.v0[0] + 0.3), T(pa.v0[1] - 0.2), T(pa.v0[2] + 0.2)};
        const T qq[4] = {T(q[0]), T(q[1]), T(q[2]), T(q[3])};
        f.init(lla, v0, qq, 0.0);
        f.setQuaternion(qq);
    }

    void steps(int n)
    {
        for (int i = 0; i < n; ++i)
        {
            double a[3], fb[3];
            path.acc(truth.t, a);
            specificForce(q, a, fb);
            const T acc[3] = {T(fb[0]), T(fb[1]), T(fb[2])};
            const T gyro[3] = {T(0), T(0), T(0)};
            f.propagate(acc, gyro, T(kDt));
            truth.step(a, kDt);
        }
    }

    double velErr() const
    {
        double e = 0.0;
        for (int i = 0; i < 3; ++i)
        {
            const double d = static_cast<double>(f.velNed()[i]) - truth.v[i];
            e += d * d;
        }
        return std::sqrt(e);
    }

    // To the next whole second, the epoch measured there, then age_s on and applied.
    std::vector<RawMeas> nextEpoch()
    {
        steps(static_cast<int>(std::lround((1.0 - age_s) / kDt)));
        const double before = gen.rx.clk_m;
        gen.rx.tick(gen.rng, gen.fm);
        (void)before;
        double r[3], v[3];
        truth.ecef(r, v);
        std::vector<RawMeas> m = gen.epoch(truth.t, r, v);
        steps(static_cast<int>(std::lround(age_s / kDt)));
        return m;
    }

    void apply(const std::vector<RawMeas>& m)
    {
        stats.push_back(f.updateGnssRaw(m.data(), static_cast<int>(m.size()), age_s));
        verr.push_back(velErr());
    }

    void fly(int epochs)
    {
        for (int e = 0; e < epochs; ++e)
            apply(nextEpoch());
    }
};

double rms(const std::vector<double>& v, size_t from)
{
    double s = 0.0;
    for (size_t i = from; i < v.size(); ++i)
        s += v[i] * v[i];
    return std::sqrt(s / static_cast<double>(v.size() - from));
}

template <typename T>
bool sameState(const RawGnssEKF<T>& a, const RawGnssEKF<T>& b)
{
    bool same = std::memcmp(a.lla(), b.lla(), 3 * sizeof(double)) == 0;
    same = same && std::memcmp(a.velNed(), b.velNed(), 3 * sizeof(T)) == 0;
    same = same && std::memcmp(a.quat(), b.quat(), 4 * sizeof(T)) == 0;
    same = same && std::memcmp(a.accelBias(), b.accelBias(), 3 * sizeof(T)) == 0;
    same = same && std::memcmp(a.gyroBias(), b.gyroBias(), 3 * sizeof(T)) == 0;
    const double ca = a.clockBias(), cb = b.clockBias();
    const T da = a.clockDrift(), db = b.clockDrift();
    same = same && std::memcmp(&ca, &cb, sizeof(double)) == 0 && std::memcmp(&da, &db, sizeof(T)) == 0;
    for (int i = 0; i < RawGnssEKF<T>::N; ++i)
        for (int j = 0; j < RawGnssEKF<T>::N; ++j)
        {
            const T pa = a.P(i, j), pb = b.P(i, j);
            same = same && std::memcmp(&pa, &pb, sizeof(T)) == 0;
        }
    return same;
}

}  // namespace

TEST(RawGnssCarrier, ObserveLeavesTheStateBitwiseAsOff)
{
    // The same flight, the carrier off and observed, a barometer and zero
    // velocity between epochs besides: every bit of the state alike at every
    // epoch -- and observe has screened every rate and found the common rate
    Path path;
    path.v0[0] = 20.0;
    path.v0[2] = -30.0;
    path.a0[2] = 3.0;
    path.swing = 2.0;
    for (int late = 0; late < 2; ++late)
    {
        Flight<float> off(params(gnss_raw::kCarrierOff), path), obs(params(gnss_raw::kCarrierObserve), path);
        off.age_s = obs.age_s = late ? 0.2 : 0.0;
        for (int e = 0; e < 12; ++e)
        {
            const std::vector<RawMeas> m = off.nextEpoch();
            (void)obs.nextEpoch();
            off.apply(m);
            obs.apply(m);
            off.f.updateHeight(off.truth.height(), 4.0f);
            obs.f.updateHeight(obs.truth.height(), 4.0f);
            if (e == 5)
            {
                off.f.updateZeroVelocity(5.0f);
                obs.f.updateZeroVelocity(5.0f);
            }
            ASSERT_TRUE(sameState(off.f, obs.f)) << "epoch " << e << (late ? " late" : "");
        }
        const RawGnssStats& so = off.stats.back();
        EXPECT_EQ(so.cr_built, 0);                 // off: not even looked at
        EXPECT_EQ(so.used_cr, 0);
        const RawGnssStats& s = obs.stats.back();
        EXPECT_EQ(s.cr_built, 10);
        EXPECT_EQ(s.cr_in, 10);
        EXPECT_EQ(s.used_cr, 0);
        EXPECT_EQ(s.cr_unmatched + s.cr_half + s.cr_other + s.cr_repaired, 0);
        // the common rate: the clock's step over the second and the wind-up
        // (0.5 cycles a second, 9.5 cm/s)
        EXPECT_NEAR(s.cr_common_mps, obs.gen.rx.drift_mps + 0.5 * kL1, 0.4);
        // the rates agree among themselves to their noise once the velocity
        // the Dopplers left is taken out, and that velocity correction is the
        // filter's own error (the Dopplers': decimetres a second)
        EXPECT_LT(s.cr_fit_rms_mps, 0.015f);
        EXPECT_GT(s.cr_rms_mps, s.cr_fit_rms_mps);
        EXPECT_NEAR(s.cr_dv_mps, obs.verr.back(), 0.03);
    }
}

TEST(RawGnssCarrier, FusingBeatsTheDopplerAloneAtConstantVelocityAndThroughManoeuvres)
{
    // The Doppler at 0.1 m/s, the carrier's phase at 4 mm; the clock's white
    // FM 0.1 m and a wind-up of half a cycle a second that the common rate
    // takes, not the velocity
    Path cruise;
    cruise.v0[0] = 30.0;
    cruise.v0[1] = -20.0;
    cruise.v0[2] = -50.0;
    Path manoeuvre = cruise;
    manoeuvre.a0[2] = 4.0;
    manoeuvre.swing = 8.0;                         // 0.8 g, turning at 0.2 Hz
    for (const Path& path : {cruise, manoeuvre})
    {
        Flight<float> dop(params(gnss_raw::kCarrierObserve), path), car(params(gnss_raw::kCarrierFuse), path);
        dop.age_s = car.age_s = 0.15;
        dop.fly(40);
        car.fly(40);
        const double e_dop = rms(dop.verr, 10), e_car = rms(car.verr, 10);
        EXPECT_LT(e_car, 0.2 * e_dop) << "Doppler " << e_dop << ", carrier " << e_car;
        EXPECT_LT(e_car, 0.02);
        const RawGnssStats& s = car.stats.back();
        EXPECT_EQ(s.used_cr, 10);
        EXPECT_EQ(s.cr_in, 10);
        // the common rate estimated beside the state: the clock's step and the wind-up
        const double dclk = s.cr_common_mps - 0.5 * kL1;
        EXPECT_NEAR(dclk, car.gen.rx.drift_mps, 0.4);
        EXPECT_LT(s.cr_fit_rms_mps, 0.015f);
    }
}

TEST(RawGnssCarrier, AHalfCycleStepIsRejectedOrRepaired)
{
    double y_rejected = 0.0;
    for (int repair = 0; repair < 2; ++repair)
    {
        RawGnssEKFParams p = params(gnss_raw::kCarrierFuse);
        p.cr_half_cycle = repair ? gnss_raw::kHalfCycleRepair : gnss_raw::kHalfCycleReject;
        Path path;
        path.v0[1] = 15.0;
        Flight<double> fl(p, path);
        fl.fly(8);
        std::vector<RawMeas> m = fl.nextEpoch();
        // GPS 8: half a cycle too much; Galileo 4 the same (its pilot cannot
        // step half a cycle: anything there is something else); GPS 22 a whole
        // cycle less (a slip, not a half)
        m[1].cr_dphi_m += 0.5 * kL1;
        m[5].cr_dphi_m += 0.5 * kL1;
        m[3].cr_dphi_m -= kL1;
        fl.apply(m);
        const RawGnssStats& s = fl.stats.back();
        int n = 0;
        const CarrierInfo* ci = fl.f.carrierInfo(n);
        ASSERT_EQ(n, 10);
        EXPECT_EQ(ci[1].half, 1);
        EXPECT_EQ(ci[1].verdict, repair ? CarrierInfo::kRepaired : CarrierInfo::kHalfCycle);
        EXPECT_EQ(ci[5].verdict, CarrierInfo::kOther);
        EXPECT_EQ(ci[3].verdict, CarrierInfo::kOther);
        EXPECT_EQ(s.cr_half, repair ? 0 : 1);
        EXPECT_EQ(s.cr_repaired, repair ? 1 : 0);
        EXPECT_EQ(s.cr_other, 2);
        EXPECT_EQ(s.cr_in, repair ? 8 : 7);
        EXPECT_EQ(s.used_cr, s.cr_in);
        if (!repair)
            y_rejected = ci[1].y;              // as measured
        else
        {
            // repaired: half a cycle less, and with it the rates agree to
            // their noise again (the step left in would be 95 mm/s)
            EXPECT_NEAR(ci[1].y, y_rejected - 0.5 * kL1, 1e-9);
            EXPECT_LT(s.cr_fit_rms_mps, 0.015f);
        }
        // and the velocity is none the worse for it
        EXPECT_LT(fl.verr.back(), 0.02);
        fl.fly(2);
        EXPECT_EQ(fl.stats.back().cr_in, 10);
    }
}

TEST(RawGnssCarrier, FewerThanFourRatesAreNotTakenAndAMissedEpochIsNotDifferencedAcross)
{
    Path path;
    path.v0[0] = 10.0;
    Flight<float> fuse(params(gnss_raw::kCarrierFuse), path), none(params(gnss_raw::kCarrierFuse), path);
    fuse.fly(5);
    none.fly(5);
    // three rates: nothing to screen them against, none fused -- the state
    // as if there had been no rates at all
    std::vector<RawMeas> m = fuse.nextEpoch();
    (void)none.nextEpoch();
    std::vector<RawMeas> m0 = m;
    for (size_t i = 0; i < m.size(); ++i)
    {
        m[i].has_cr = i < 3;
        m0[i].has_cr = false;
    }
    fuse.apply(m);
    none.apply(m0);
    EXPECT_TRUE(fuse.stats.back().cr_too_few);
    EXPECT_EQ(fuse.stats.back().cr_built, 3);
    EXPECT_EQ(fuse.stats.back().used_cr, 0);
    int n = 0;
    const CarrierInfo* ci = fuse.f.carrierInfo(n);
    ASSERT_EQ(n, 3);
    for (int j = 0; j < n; ++j)
        EXPECT_EQ(ci[j].verdict, CarrierInfo::kTooFew);
    EXPECT_TRUE(sameState(fuse.f, none.f)) << "fused under four";
    // an epoch the filter never took: the next epoch's rates span it -- not
    // the filter's last epoch -- and go unmatched
    (void)fuse.nextEpoch();
    fuse.apply(fuse.nextEpoch());
    EXPECT_EQ(fuse.stats.back().cr_unmatched, 10);
    EXPECT_EQ(fuse.stats.back().used_cr, 0);
    // and the epoch after is matched again
    fuse.apply(fuse.nextEpoch());
    EXPECT_EQ(fuse.stats.back().cr_unmatched, 0);
    EXPECT_EQ(fuse.stats.back().used_cr, 10);
}

TEST(RawGnssCarrier, ACorrectionBetweenEpochsMovesTheNextRatesOnlyByItsVelocity)
{
    // Two filters alike to an epoch; then one has its height corrected (a
    // barometer, 3 m off and sure of it), zero velocity, or levelling before
    // the next epoch. That epoch's carrier rates, screened at once (no
    // pseudoranges or Dopplers between): the correction's position jump is
    // the whole trajectory's, not a velocity -- each rate moves by exactly
    // the line of sight's share of the correction's velocity change. (Taken
    // naively, 3 m in a second is 3 m/s on the down component.)
    Path path;
    path.v0[0] = 25.0;
    path.v0[2] = -10.0;
    for (int kind = 0; kind < 3; ++kind)
    {
        Flight<double> a(params(gnss_raw::kCarrierObserve), path), b(params(gnss_raw::kCarrierObserve), path);
        a.fly(6);
        b.fly(6);
        a.steps(100);
        b.steps(100);
        if (kind == 0)
            b.f.updateHeight(b.truth.height() + 3.0, 1e-4);
        else if (kind == 1)
            b.f.updateZeroVelocity(0.5);
        else
        {
            const double fb_d[3] = {0.0, 0.0, -kG};
            const double fb[3] = {fb_d[0] + 0.3, fb_d[1], fb_d[2]};   // a levelling that tilts it a little
            b.f.updateAccelLevel(fb);
        }
        a.steps(100);
        b.steps(100);
        a.gen.rx.tick(a.gen.rng, a.gen.fm);
        b.gen.rx.tick(b.gen.rng, b.gen.fm);
        double r[3], v[3];
        a.truth.ecef(r, v);
        const std::vector<RawMeas> ma = a.gen.epoch(a.truth.t, r, v);
        const std::vector<RawMeas> mb = b.gen.epoch(b.truth.t, r, v);
        RawGnssStats sa, sb;
        ASSERT_TRUE(a.f.beginGnssRaw(ma.data(), 10, 0.0, sa));
        ASSERT_TRUE(b.f.beginGnssRaw(mb.data(), 10, 0.0, sb));
        a.f.beginCarrier(ma.data(), 10, 0.0, sa);
        b.f.beginCarrier(mb.data(), 10, 0.0, sb);
        int na = 0, nb = 0;
        const CarrierInfo* ca = a.f.carrierInfo(na);
        const CarrierInfo* cb = b.f.carrierInfo(nb);
        ASSERT_EQ(na, 10);
        ASSERT_EQ(nb, 10);
        double dv[3], r_b[3], v_b[3], dpos = 0.0;
        for (int i = 0; i < 3; ++i)
            dv[i] = b.f.velNed()[i] - a.f.velNed()[i];
        b.f.receiverEcef(r_b, v_b);
        {
            double r_a[3], v_a[3];
            a.f.receiverEcef(r_a, v_a);
            for (int k = 0; k < 3; ++k)
                dpos += (r_b[k] - r_a[k]) * (r_b[k] - r_a[k]);
        }
        dpos = std::sqrt(dpos);
        if (kind == 0)
            EXPECT_GT(dpos, 1.0);                  // the jump was metres
        double worst = 0.0, naive = 0.0;
        for (int j = 0; j < 10; ++j)
        {
            // the line of sight, NED
            const RawMeas& mj = mb[ca[j].idx];
            double los[3], ned[3], rn = 0.0;
            for (int k = 0; k < 3; ++k)
            {
                los[k] = mj.sat_pos[k] - r_b[k];
                rn += los[k] * los[k];
            }
            for (int k = 0; k < 3; ++k)
                los[k] /= std::sqrt(rn);
            ecefToNed(b.f.lla()[0], b.f.lla()[1], los, ned);
            const double expect = ned[0] * dv[0] + ned[1] * dv[1] + ned[2] * dv[2];
            worst = std::fmax(worst, std::fabs((cb[j].y - ca[j].y) - expect));
            if (kind == 0)
                naive = std::fmax(naive, std::fabs(ned[2]) * 3.0);   // the jump read as a velocity over the second
        }
        // the lines of sight turn ~2e-4 rad over the second: a jump of
        // metres is a millimetre a second at most at second order
        EXPECT_LT(worst, 2e-3) << "kind " << kind << ", |dv| " << std::sqrt(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]);
        if (kind == 0)
            EXPECT_GT(naive, 1.0);
    }
}

TEST(RawGnssCarrier, FloatTracksDouble)
{
    Path path;
    path.v0[0] = 30.0;
    path.v0[2] = -50.0;
    path.a0[2] = 3.0;
    path.swing = 4.0;
    Flight<double> d(params(gnss_raw::kCarrierFuse), path);
    Flight<float> s(params(gnss_raw::kCarrierFuse), path);
    d.age_s = s.age_s = 0.1;
    d.fly(30);
    s.fly(30);
    double rd[3], rs[3], vd[3], vs[3], dist = 0.0;
    d.f.receiverEcef(rd, vd);
    s.f.receiverEcef(rs, vs);
    for (int k = 0; k < 3; ++k)
        dist += (rd[k] - rs[k]) * (rd[k] - rs[k]);
    EXPECT_LT(std::sqrt(dist), 0.05);
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(double(s.f.velNed()[i]), d.f.velNed()[i], 1e-3);
    EXPECT_EQ(s.stats.back().used_cr, d.stats.back().used_cr);
    EXPECT_NEAR(s.stats.back().cr_common_mps, d.stats.back().cr_common_mps, 2e-3);
    for (int i = 0; i < RawGnssEKF<float>::N; ++i)
    {
        EXPECT_TRUE(std::isfinite(s.f.P(i, i)));
        EXPECT_GT(s.f.P(i, i), 0.0f);
    }
}

TEST(RawGnssCarrier, TheCarrierGoesThroughTheGnssUpdatesAttitudeGate)
{
    // A body pitched 30 deg and accelerating, so the velocity and the tilt
    // share covariance. The carrier rates fused at once from one state, with
    // the full gate, cos^4 pitch, and about the vertical only: the full gate
    // tilts it, cos^4's by (cos 30)^4 as much, the vertical gate not at all
    const double pitch = 30.0 * kPi / 180.0;
    const double q_body[4] = {std::cos(pitch / 2), 0.0, std::sin(pitch / 2), 0.0};
    Path path;
    path.v0[0] = 40.0;
    path.a0[0] = 6.0;
    path.swing = 3.0;
    double tilt[3] = {}, turn[3] = {};
    for (int gate = 0; gate < 3; ++gate)
    {
        // every run the same to the epoch (the full gate), then the gate set
        RawGnssEKFParams p = params(gnss_raw::kCarrierFuse);
        p.att_gate_cos4 = false;
        Flight<double> fl(p, path, q_body);
        fl.f.setTiltSigmaFloor(2.0 * kPi / 180.0);   // a tilt to learn
        fl.fly(4);
        fl.f.setAttGateCos4(gate == 1);
        fl.f.setAttGateVertical(gate == 2);
        const std::vector<RawMeas> m = fl.nextEpoch();
        RawGnssStats st;
        ASSERT_TRUE(fl.f.beginGnssRaw(m.data(), 10, 0.0, st));
        ASSERT_EQ(fl.f.beginCarrier(m.data(), 10, 0.0, st), 10);
        double q0[4];
        for (int i = 0; i < 4; ++i)
            q0[i] = fl.f.quat()[i];
        fl.f.fuseCarrier(m.data(), st);
        fl.f.endCarrier(st);
        EXPECT_EQ(st.used_cr, 10);
        // the turn the carrier gave, as a rotation vector in NED
        const double* q1 = fl.f.quat();
        const double dq[4] = {q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3],
                              q0[0] * q1[1] - q0[1] * q1[0] - q0[2] * q1[3] + q0[3] * q1[2],
                              q0[0] * q1[2] + q0[1] * q1[3] - q0[2] * q1[0] - q0[3] * q1[1],
                              q0[0] * q1[3] - q0[1] * q1[2] + q0[2] * q1[1] - q0[3] * q1[0]};   // q0^-1 q1, body
        // body -> NED: the rotation vector in NED is q0's rotation of the body one
        const double bx = 2.0 * dq[1], by = 2.0 * dq[2], bz = 2.0 * dq[3];
        const double w = q0[0], x = q0[1], y = q0[2], z = q0[3];
        const double R[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
                                {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
                                {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};
        const double n[3] = {R[0][0] * bx + R[0][1] * by + R[0][2] * bz, R[1][0] * bx + R[1][1] * by + R[1][2] * bz,
                             R[2][0] * bx + R[2][1] * by + R[2][2] * bz};
        tilt[gate] = std::hypot(n[0], n[1]);
        turn[gate] = std::fabs(n[2]);
    }
    EXPECT_GT(tilt[0], 1e-6);                                     // the full gate: the carrier tilts it
    EXPECT_NEAR(tilt[1] / tilt[0], std::pow(std::cos(pitch), 4.0), 0.1);   // cos^4 pitch of that
    EXPECT_LT(tilt[2], 1e-3 * tilt[0]);                           // the vertical gate: none
}
