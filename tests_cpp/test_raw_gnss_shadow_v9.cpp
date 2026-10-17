// Host tests for RawGnssShadow's and LevelGate's switches for a receiver fused
// at 10 Hz (the mosaic-G5 on the V8/V9, fc/v9-raw-primary): the pseudorange
// once a second (pr_period_s), the Doppler's stress held for a window
// (rr_dyn_window_s), the heading fit an epoch a second
// (heading_fit_period_s), a restored flight's sigmas and accelerometer bias,
// levelling only on epochs that fused something (level_on_fused_epochs), and
// LevelGate's steady test on epochs a second apart (min_dt_s). Each at its
// default is the Beetle's filter exactly (the existing tests, unchanged).
//
// The flights are test_raw_gnss_shadow.cpp's synthetic sky: the Beetle's own
// ephemerides (fixtures/raw_meas_golden.h) giving each satellite's
// pseudorange and range rate for a receiver whose truth the test sets, here
// an epoch every 0.1 s on the GPS-time grid, through the mosaic-G5's profile.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <vector>

#include "LevelGate.h"
#include "RawGnssShadow.h"
#include "ReceiverProfileApply.h"
#include "RtcmEphemeris.h"
#include "ShadowLink.h"
#include "fixtures/raw_meas_golden.h"
#include "gnss/atmo.h"

using gnss_raw::LevelGate;
using gnss_raw::RawGnssShadow;
using gnss_raw::ShadowImu;
using gnss_raw::ShadowSeed;
using gnss_raw::TimedRawEpoch;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int64_t kT0 = 5000000000LL;      // 5000 s of MCU time: past a 32-bit wrap
constexpr double kG = 9.807;
constexpr uint16_t kGoldenWeek = 2433;     // the fixture's (2026-08-29)

ShadowSeed padSeed(int64_t t_us, double lat, double lon, double h)
{
    ShadowSeed s{};
    s.time_us = static_cast<uint32_t>(t_us);
    s.lla[0] = lat;
    s.lla[1] = lon;
    s.lla[2] = h;
    s.quat[0] = 1.0f;          // body = NED: level
    s.on_pad = 1;
    return s;
}

RawGnssShadow::Config primaryCfg()
{
    RawGnssShadow::Config c;
    c.primary = true;
    c.min_step_s = 0.01;
    return c;
}

// The mosaic-G5's: its profile on the navigation filter's config
RawGnssShadow::Config mosaicCfg()
{
    RawGnssShadow::Config c = primaryCfg();
    gnss_raw::applyReceiverProfile(gnss_raw::mosaicG5Profile(), c);
    c.meas.troposphere = false;            // the synthetic sky has none
    return c;
}

double attErrDeg(const float a[4], const float b[4])
{
    const double d = std::fabs(double(a[0]) * b[0] + double(a[1]) * b[1] + double(a[2]) * b[2] + double(a[3]) * b[3]);
    return 2.0 * std::acos(std::min(1.0, d)) * 180.0 / kPi;
}

struct Mat3
{
    double m[3][3];
};

Mat3 mat3Mul(const Mat3& a, const Mat3& b)
{
    Mat3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
}

void mat3Vec(const Mat3& a, const double v[3], double o[3])
{
    for (int i = 0; i < 3; ++i)
        o[i] = a.m[i][0] * v[0] + a.m[i][1] * v[1] + a.m[i][2] * v[2];
}

Mat3 rotAbout(const double axis[3], double a)
{
    const double n = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    const double k[3] = {axis[0] / n, axis[1] / n, axis[2] / n};
    const double K[3][3] = {{0, -k[2], k[1]}, {k[2], 0, -k[0]}, {-k[1], k[0], 0}};
    Mat3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            double kk = 0.0;
            for (int l = 0; l < 3; ++l)
                kk += K[i][l] * K[l][j];
            r.m[i][j] = (i == j ? 1.0 : 0.0) + std::sin(a) * K[i][j] + (1.0 - std::cos(a)) * kk;
        }
    return r;
}

// A nose-up body leaning lean_deg toward azimuth toward_deg: body -> NED
Mat3 leaningBody(double lean_deg, double toward_deg)
{
    const double z[3] = {0, 0, 1}, y[3] = {0, 1, 0};
    Mat3 up{};                                   // body x up
    up.m[0][2] = 1;
    up.m[1][1] = 1;
    up.m[2][0] = -1;
    return mat3Mul(rotAbout(z, toward_deg * kPi / 180.0), mat3Mul(rotAbout(y, lean_deg * kPi / 180.0), up));
}

// body -> NED as a scalar-first quaternion (Quat2DCM of it is NED -> body)
void quatOf(const Mat3& Cm, float q[4])
{
    const double (&R)[3][3] = Cm.m;
    const double t = R[0][0] + R[1][1] + R[2][2];
    double w, x, y, z;
    if (t > 0.0)
    {
        w = 0.5 * std::sqrt(1.0 + t);
        x = (R[2][1] - R[1][2]) / (4 * w);
        y = (R[0][2] - R[2][0]) / (4 * w);
        z = (R[1][0] - R[0][1]) / (4 * w);
    }
    else if (R[0][0] >= R[1][1] && R[0][0] >= R[2][2])
    {
        x = 0.5 * std::sqrt(1.0 + 2 * R[0][0] - t);
        w = (R[2][1] - R[1][2]) / (4 * x);
        y = (R[0][1] + R[1][0]) / (4 * x);
        z = (R[0][2] + R[2][0]) / (4 * x);
    }
    else if (R[1][1] >= R[2][2])
    {
        y = 0.5 * std::sqrt(1.0 + 2 * R[1][1] - t);
        w = (R[0][2] - R[2][0]) / (4 * y);
        x = (R[0][1] + R[1][0]) / (4 * y);
        z = (R[1][2] + R[2][1]) / (4 * y);
    }
    else
    {
        z = 0.5 * std::sqrt(1.0 + 2 * R[2][2] - t);
        w = (R[1][0] - R[0][1]) / (4 * z);
        x = (R[0][2] + R[2][0]) / (4 * z);
        y = (R[1][2] + R[2][1]) / (4 * z);
    }
    const double sgn = w < 0 ? -1.0 : 1.0;
    q[0] = float(sgn * w);
    q[1] = float(sgn * x);
    q[2] = float(sgn * y);
    q[3] = float(sgn * z);
}

// The sky the Beetle's own ephemerides give at the pad of its rig: each
// satellite's pseudorange and range rate for a receiver whose truth the test
// sets (test_raw_gnss_shadow.cpp's SynthSky). A twin of the flight's builder
// finds each satellite's transmit state, so the shadow's builder takes them
// back to that truth.
struct SynthSky
{
    gnss_raw::EphemerisStore store;
    gnss_raw::RawMeasBuilder builder;
    std::mt19937 rng{11};
    double clk_m = 3000.0;        // the receiver's clock, m
    double sigma_pr = 1.0, sigma_rr = 0.05;

    explicit SynthSky(const gnss_raw::RawMeasBuilder::Config& c) : builder(c)
    {
        for (int i = 0; i < kGoldenRmEphN; ++i)
        {
            gps_eph_t e;
            if (gnss_raw::decodeEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len),
                                          kGoldenWeek, e))
                store.add(e);
        }
    }

    // r: the receiver at the tag (ECEF), v: its velocity (ECEF)
    gnss_raw::RawEpoch epoch(double tow, const double r[3], const double v[3])
    {
        gnss_raw::RawEpoch ep;
        ep.tow_s = tow;
        for (int i = 0; i < kGoldenRmEphN; ++i)
        {
            gps_eph_t e;
            if (!gnss_raw::decodeEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len),
                                           kGoldenWeek, e))
                continue;
            gnss_raw::RawObs& o = ep.obs[ep.n++];
            o.sys = static_cast<uint8_t>(e.sys);
            o.prn = static_cast<uint8_t>(e.prn);
            o.has_pr = o.has_rate = true;
            o.cn0_dbhz = 42.0f;
            o.pr_m = 2.2e7;
            o.rate_mps = 0.0;
            o.lock_ms = 600000;
        }
        // three times: the transmit time follows the pseudorange
        gnss_raw::RawMeas m[gnss_raw::RawEpoch::kMaxObs];
        for (int it = 0; it < 3; ++it)
        {
            gnss_raw::RawEpoch probe = ep;
            const int n = builder.build(probe, store, r, 0.0, m, gnss_raw::RawEpoch::kMaxObs);
            for (int k = 0; k < n; ++k)
                for (int j = 0; j < ep.n; ++j)
                {
                    gnss_raw::RawObs& o = ep.obs[j];
                    if (o.sys != m[k].sys || o.prn != m[k].prn)
                        continue;
                    const double corr = m[k].pr - o.pr_m;            // the builder's clock, less the ionosphere
                    o.pr_m = gnss_raw::sagnacRange(m[k], r) + clk_m - corr;
                    double u[3];
                    const double still = gnss_raw::stillRangeRate(m[k], r, u);
                    const double clk_rate = m[k].rr - o.rate_mps;     // the satellite clock's rate, m/s
                    o.rate_mps = still - (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]) - clk_rate;
                }
        }
        gnss_raw::RawEpoch out;
        out.tow_s = tow;
        const int n = builder.build(ep, store, r, 0.0, m, gnss_raw::RawEpoch::kMaxObs);
        std::normal_distribution<double> npr(0.0, sigma_pr), nrr(0.0, sigma_rr);
        for (int k = 0; k < n; ++k)
            for (int j = 0; j < ep.n; ++j)
                if (ep.obs[j].sys == m[k].sys && ep.obs[j].prn == m[k].prn)
                {
                    gnss_raw::RawObs o = ep.obs[j];
                    o.pr_m += npr(rng);
                    o.rate_mps += nrr(rng);
                    out.obs[out.n++] = o;
                }
        return out;
    }
};

uint64_t stateHash(const gnss_raw::RawGnssEKF<float>& f)
{
    uint64_t h = 1469598103934665603ULL;
    auto add = [&](const void* p, size_t n) {
        const uint8_t* b = static_cast<const uint8_t*>(p);
        for (size_t i = 0; i < n; ++i)
        {
            h ^= b[i];
            h *= 1099511628211ULL;
        }
    };
    add(f.lla(), 3 * sizeof(double));
    add(f.velNed(), 3 * sizeof(float));
    add(f.quat(), 4 * sizeof(float));
    add(f.accelBias(), 3 * sizeof(float));
    add(f.gyroBias(), 3 * sizeof(float));
    const double cb = f.clockBias();
    const float cd = f.clockDrift();
    add(&cb, sizeof(cb));
    add(&cd, sizeof(cd));
    for (int i = 0; i < gnss_raw::RawGnssEKF<float>::N; ++i)
        for (int j = 0; j < gnss_raw::RawGnssEKF<float>::N; ++j)
        {
            const float p = f.P(i, j);
            add(&p, sizeof(p));
        }
    return h;
}

// One epoch as it closed: its time (s from the first epoch's GPS second), its
// counts, and its Dopplers' largest and smallest 1-sigma as the filter took
// them (the stress added).
struct ClosedEpoch
{
    double t = 0.0;
    gnss_raw::RawGnssStats st;
    float sigma_rr_max = 0.0f, sigma_rr_min = 0.0f;
};

struct TenRun
{
    std::vector<ClosedEpoch> eps;
    std::vector<uint64_t> state;          // stateHash every 0.25 s
    std::vector<HeadingFitData> fits;
    double first_aided_t = -1.0;          // flight time gnss_aided first came up
    int unaided_ticks_after = 0;          //   ticks without it after that
    RawGnssShadow::Stats st;
};

// The Goblin-like flight at 10 Hz: the pad 4 s, a boost of 3 g along the nose
// for 2 s (burnout at 6 s), then a coast to t_end, on a nose-up body leaning
// lean_deg toward azimuth 70. An epoch every 0.1 s of GPS time from the sky,
// each handed over 50 ms after its tag (the mosaic's 20 ms and the task's);
// the launch detector calls it at 4.1 s.
constexpr double kIgn = 4.0, kBurnout = 6.0;

TenRun flyTen(const RawGnssShadow::Config& cfg, double lean_deg = 0.0, double t_end = 10.0)
{
    double lat, lon, h;
    ecef_to_geo(kGoldenRmRx, &lat, &lon, &h);
    double r0[3];
    geo_to_ecef(lat, lon, h, r0);
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    auto nedToEcefVec = [&](const double n[3], double o[3]) {
        const double e[3] = {n[1], n[0], -n[2]};
        o[0] = -so * e[0] - sl * co * e[1] + cl * co * e[2];
        o[1] = co * e[0] - sl * so * e[1] + cl * so * e[2];
        o[2] = cl * e[1] + sl * e[2];
    };
    RawGnssShadow sh(cfg);
    for (int i = 0; i < kGoldenRmEphN; ++i)
        sh.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
    SynthSky sky(cfg.meas);
    const Mat3 Ct = leaningBody(lean_deg, 70.0);
    ShadowSeed seed = padSeed(kT0, lat, lon, h);
    quatOf(Ct, seed.quat);
    sh.onSeed(seed);
    const double tow0 = kGoldenRmTowA, dt = 0.005;
    double p_ned[3] = {}, v_ned[3] = {};
    std::vector<TimedRawEpoch> due;
    int next_epoch = 1;                                       // tenths of a second
    uint32_t last_tow_ms = 0;
    TenRun run;
    for (int step = 1; step * dt <= t_end + 1e-9; ++step)
    {
        const double t = step * dt;
        double f_b[3] = {0.0, 0.0, 0.0}, a_n[3] = {0.0, 0.0, 0.0};
        if (t > kIgn)
        {
            f_b[0] = t <= kBurnout ? 3.0 * kG : 0.0;          // then coasting, weightless
            double f_n[3];
            mat3Vec(Ct, f_b, f_n);
            for (int i = 0; i < 3; ++i)
                a_n[i] = f_n[i] + (i == 2 ? kG : 0.0);
        }
        else
        {
            const double up[3] = {0, 0, -kG};                 // on the pad: gravity's reaction, in the body
            Mat3 CtT{};
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    CtT.m[i][j] = Ct.m[j][i];
            mat3Vec(CtT, up, f_b);
        }
        for (int i = 0; i < 3; ++i)
        {
            p_ned[i] += v_ned[i] * dt + 0.5 * a_n[i] * dt * dt;
            v_ned[i] += a_n[i] * dt;
        }
        const int64_t t64 = kT0 + static_cast<int64_t>(std::llround(t * 1e6));
        ShadowImu s{};
        s.time_us = static_cast<uint32_t>(t64);
        for (int i = 0; i < 3; ++i)
            s.acc[i] = float(f_b[i]);
        s.flags = t < kIgn ? gnss_raw::kShadowLevel : (t >= kIgn + 0.1 ? gnss_raw::kShadowInFlight : 0);
        sh.onImu(s, t64);
        // each epoch as it closes
        if (sh.stats().last_tow_ms != last_tow_ms)
        {
            last_tow_ms = sh.stats().last_tow_ms;
            ClosedEpoch c;
            c.t = (static_cast<double>(last_tow_ms) - tow0 * 1000.0) / 1000.0;
            c.st = sh.stats().last;
            int n = 0;
            const gnss_raw::RawMeas* m = sh.lastMeasurements(n);
            for (int i = 0; i < n; ++i)
            {
                c.sigma_rr_max = std::max(c.sigma_rr_max, m[i].sigma_rr);
                c.sigma_rr_min = i == 0 ? m[i].sigma_rr : std::min(c.sigma_rr_min, m[i].sigma_rr);
            }
            run.eps.push_back(c);
        }
        gnss_raw::ShadowNav nav;
        sh.navState(nav);
        if (nav.gnss_aided && run.first_aided_t < 0.0)
            run.first_aided_t = t;
        else if (!nav.gnss_aided && run.first_aided_t >= 0.0)
            run.unaided_ticks_after++;
        if (step % 50 == 0)
            run.state.push_back(stateHash(sh.filter()));
        // an epoch at each tenth of a GPS second, built where the receiver truly is
        if (step % 20 == 0)
        {
            double dr[3], r[3], ve[3];
            nedToEcefVec(p_ned, dr);
            nedToEcefVec(v_ned, ve);
            for (int i = 0; i < 3; ++i)
                r[i] = r0[i] + dr[i];
            TimedRawEpoch e;
            e.epoch = sky.epoch(tow0 + next_epoch / 10.0, r, ve);
            e.mcu_us = t64;
            e.gps_week = kGoldenWeek;
            due.push_back(e);
            next_epoch++;
        }
        while (!due.empty() && t64 - due.front().mcu_us >= 50000)
        {
            sh.onEpoch(due.front());
            due.erase(due.begin());
        }
        HeadingFitData d;
        while (sh.takeHeadingLog(d))
            run.fits.push_back(d);
    }
    run.st = sh.stats();
    return run;
}

}  // namespace

// ---- pr_period_s: the code once a second

TEST(RawGnssShadowV9, AtTenHertzThePseudorangeIsFusedOnceASecond)
{
    // The mosaic's profile: the Dopplers every epoch, the pseudoranges only
    // on the whole GPS seconds -- and gnss_aided (a pseudorange within 2 s)
    // holds from the first one to the end, through the boost and the coast
    const TenRun run = flyTen(mosaicCfg());
    ASSERT_GT(run.eps.size(), 80u);
    int with_pr = 0, whole = 0;
    for (const ClosedEpoch& c : run.eps)
    {
        const long ms = std::lround(c.t * 1000.0);
        const bool on_second = ms % 1000 == 0;
        whole += on_second ? 1 : 0;
        if (c.st.screen_dropped)
            continue;
        EXPECT_EQ(c.st.used_pr > 0, on_second) << "t " << c.t;
        EXPECT_GT(c.st.used_rr, 4) << "t " << c.t;
        with_pr += c.st.used_pr > 0 ? 1 : 0;
    }
    EXPECT_GE(with_pr, whole - 1);
    EXPECT_GE(run.first_aided_t, 0.0);
    EXPECT_LT(run.first_aided_t, 1.5);
    EXPECT_EQ(run.unaided_ticks_after, 0);
    EXPECT_EQ(run.st.stops, 0u);

    // at 0 (the Beetle's), every epoch's
    RawGnssShadow::Config every = mosaicCfg();
    every.pr_period_s = 0.0;
    const TenRun all = flyTen(every);
    for (const ClosedEpoch& c : all.eps)
        if (!c.st.screen_dropped)
            EXPECT_GT(c.st.used_pr, 0) << "t " << c.t;
}

// ---- rr_dyn_window_s: the Doppler's stress held after burnout

TEST(RawGnssShadowV9, TheDopplersStressIsHeldForItsWindowAfterBurnout)
{
    // 3 g to burnout at 6 s: the Dopplers take rr_dyn_sigma (0.5 m/s a g
    // above 1) on the IMU's 2 g excess, 1 m/s in quadrature. The LC86G's epoch
    // a second carried that into the second after burnout; at 10 Hz the peak
    // since the epoch before is gone by the second epoch after -- with a 1 s
    // window it is held to the second's end. (Every satellite is new at the
    // start here, its sigma the relock's 3 m/s decaying: the two flights' are
    // the same but for the stress.)
    const TenRun held = flyTen(mosaicCfg());
    RawGnssShadow::Config c0 = mosaicCfg();
    c0.rr_dyn_window_s = 0.0;
    const TenRun bare = flyTen(c0);
    auto at = [](const TenRun& r, double t) -> const ClosedEpoch* {
        for (const ClosedEpoch& c : r.eps)
            if (std::fabs(c.t - t) < 0.01)
                return &c;
        return nullptr;
    };
    int held_n = 0;
    for (int k = 30; k <= 90; ++k)                 // 3 s to 9 s
    {
        const double t = k / 10.0;
        const ClosedEpoch* h = at(held, t);
        const ClosedEpoch* b = at(bare, t);
        ASSERT_NE(h, nullptr) << t;
        ASSERT_NE(b, nullptr) << t;
        const double h2 = double(h->sigma_rr_min) * h->sigma_rr_min, b2 = double(b->sigma_rr_min) * b->sigma_rr_min;
        if (t > kBurnout + 0.01 && t <= kBurnout + 1.0 + 0.01)
        {
            // the second after burnout: the boost's stress held
            EXPECT_NEAR(h2 - b2, 1.0, 0.01) << t;
            held_n++;
        }
        else
        {
            // the pad, the boost and the epoch that closes it, and the coast after
            // the window: the peak since the epoch before, either way
            EXPECT_NEAR(h2, b2, 1e-4 * b2) << t;
        }
    }
    EXPECT_EQ(held_n, 10);
    // and in the boost the stress is there in both
    const ClosedEpoch* pad = at(bare, 3.9);
    const ClosedEpoch* boost = at(bare, 4.5);
    ASSERT_NE(pad, nullptr);
    ASSERT_NE(boost, nullptr);
    EXPECT_GT(double(boost->sigma_rr_min) * boost->sigma_rr_min - double(pad->sigma_rr_min) * pad->sigma_rr_min, 0.5);
}

// ---- heading_fit_period_s: the fit an epoch a second

TEST(RawGnssShadowV9, TheHeadingFitTakesAnEpochASecond)
{
    // Observe on a body leaning 5 deg: sideways fast enough for the fit from
    // ~0.4 s after ignition. Every epoch at 10 Hz fills its 64 in 6.4 s;
    // with the period, one a second, each a second after the last
    RawGnssShadow::Config cfg = mosaicCfg();
    cfg.heading_mode = RawGnssShadow::kHeadingObserve;
    const TenRun sec = flyTen(cfg, 5.0, 12.0);
    ASSERT_GE(sec.fits.size(), 5u);
    EXPECT_LE(sec.fits.size(), 9u);                         // 4.5 s to 12 s
    EXPECT_EQ(sec.st.heading_epochs, static_cast<uint32_t>(sec.fits.size()));
    for (size_t i = 1; i < sec.fits.size(); ++i)
    {
        const double gap = static_cast<double>(static_cast<int32_t>(sec.fits[i].time_us - sec.fits[i - 1].time_us)) * 1e-6;
        EXPECT_GE(gap, 0.975) << i;
        EXPECT_LT(gap, 1.15) << i;
    }
    cfg.heading_fit_period_s = 0.0;
    const TenRun every = flyTen(cfg, 5.0, 12.0);
    EXPECT_GT(every.st.heading_epochs, 40u);
}

// ---- a restored flight's start

namespace
{

const double kPadLat = 32.99 * kPi / 180.0, kPadLon = -106.97 * kPi / 180.0;

// A seed and the IMU tick that starts the filter from it (and no step yet):
// a flight restored under the drogue, or the pad.
void startFrom(RawGnssShadow& sh, bool restore)
{
    ShadowSeed seed = padSeed(kT0, kPadLat, kPadLon, 1400.0);
    seed.on_pad = restore ? 0 : 1;
    seed.restore = restore ? 1 : 0;
    seed.vel_ned[2] = 15.0f;                      // descending under the drogue
    seed.has_accel_bias = 1;
    seed.accel_bias[0] = 0.12f;
    seed.accel_bias[1] = -0.21f;
    seed.accel_bias[2] = 0.05f;
    sh.onSeed(seed);
    ShadowImu s{};
    s.time_us = static_cast<uint32_t>(kT0);
    s.acc[2] = static_cast<float>(-kG);
    s.flags = restore ? gnss_raw::kShadowInFlight : gnss_raw::kShadowLevel;
    sh.onImu(s, kT0);                             // the start, before any step
}

}  // namespace

TEST(RawGnssShadowV9, ARestoredStartDeclaresItsSigmasAndTakesTheSnapshotsBias)
{
    RawGnssShadow::Config cfg = primaryCfg();
    cfg.restore_pos_sigma_m = 100.0;
    cfg.restore_vel_sigma_mps = 20.0;
    cfg.restore_att_sigma_deg = 15.0;
    cfg.restore_seed_bias = true;
    RawGnssShadow sh(cfg);
    startFrom(sh, true);
    ASSERT_TRUE(sh.running());
    ASSERT_EQ(sh.stats().restores, 1u);
    const auto& f = sh.filter();
    using F = gnss_raw::RawGnssEKF<float>;
    const float att = static_cast<float>(15.0 * kPi / 180.0);
    for (int a = 0; a < 3; ++a)
    {
        EXPECT_FLOAT_EQ(f.P(F::kPos + a, F::kPos + a), 1.0e4f);
        EXPECT_FLOAT_EQ(f.P(F::kVel + a, F::kVel + a), 400.0f);
        EXPECT_FLOAT_EQ(f.P(F::kAtt + a, F::kAtt + a), 0.25f * att * att);
    }
    EXPECT_EQ(f.P(F::kPos, F::kVel + 2), 0.0f);
    EXPECT_EQ(f.P(F::kAtt, F::kGyroBias), 0.0f);
    EXPECT_FLOAT_EQ(f.accelBias()[0], 0.12f);
    EXPECT_FLOAT_EQ(f.accelBias()[1], -0.21f);
    EXPECT_FLOAT_EQ(f.accelBias()[2], 0.05f);
    EXPECT_FLOAT_EQ(f.velNed()[2], 15.0f);

    // the defaults: today's restored start (the attitude known, the bias zero)
    RawGnssShadow today(primaryCfg());
    startFrom(today, true);
    ASSERT_TRUE(today.running());
    const auto& g = today.filter();
    for (int a = 0; a < 3; ++a)
    {
        EXPECT_FLOAT_EQ(g.P(F::kAtt + a, F::kAtt + a), 1e-6f);
        EXPECT_FLOAT_EQ(g.P(F::kPos + a, F::kPos + a), 100.0f);   // p0_pos
        EXPECT_EQ(g.accelBias()[a], 0.0f);
    }
    // and a start on the pad takes none of it, whatever the config says
    RawGnssShadow pad(cfg);
    startFrom(pad, false);
    ASSERT_TRUE(pad.running());
    EXPECT_EQ(pad.stats().restores, 0u);
    for (int a = 0; a < 3; ++a)
    {
        EXPECT_FLOAT_EQ(pad.filter().P(F::kPos + a, F::kPos + a), 100.0f);
        EXPECT_EQ(pad.filter().accelBias()[a], 0.0f);
    }
}

// ---- level_on_fused_epochs

namespace
{

// The navigation filter under a steady 1 g with the sky gone: the receiver
// still sends an epoch every 0.1 s, but with nothing in it. The pad (2 s,
// never levelled, zero velocity) leaves the attitude 5 deg off in roll; in
// flight a 0.3 s jolt (a drogue's snatch) and then a steady 1 g, levelling
// allowed. Returns the filter's tilt error at t_end, and its levels in flight.
struct EmptySky
{
    double tilt_deg = 0.0;
    uint32_t levels = 0;
    double first_level_t = -1.0;
};

EmptySky flyEmptySky(bool level_on_fused, double t_end = 15.0)
{
    RawGnssShadow::Config cfg = primaryCfg();
    cfg.level_on_fused_epochs = level_on_fused;
    RawGnssShadow sh(cfg);
    ShadowSeed seed = padSeed(kT0, kPadLat, kPadLon, 1400.0);
    const double r5 = 5.0 * kPi / 180.0;
    seed.quat[0] = float(std::cos(r5 / 2));
    seed.quat[1] = float(std::sin(r5 / 2));
    sh.onSeed(seed);
    const double dt = 0.005, t_flight = 2.0;
    int next_epoch = 1;
    std::vector<TimedRawEpoch> due;
    EmptySky out;
    uint32_t levels0 = 0;
    for (int step = 1; step * dt <= t_end + 1e-9; ++step)
    {
        const double t = step * dt;
        const int64_t t64 = kT0 + static_cast<int64_t>(std::llround(t * 1e6));
        ShadowImu s{};
        s.time_us = static_cast<uint32_t>(t64);
        const bool jolt = t > t_flight && t <= t_flight + 0.3;
        s.acc[2] = static_cast<float>(jolt ? -2.0 * kG : -kG);
        s.flags = t <= t_flight ? 0 : (gnss_raw::kShadowInFlight | gnss_raw::kShadowLevel);
        if (std::fabs(t - t_flight) < 0.5 * dt)
            levels0 = sh.stats().levels;
        sh.onImu(s, t64);
        if (t > t_flight && out.first_level_t < 0.0 && sh.stats().levels > levels0)
            out.first_level_t = t;
        if (step % 20 == 0)
        {
            TimedRawEpoch e;
            e.epoch.tow_s = kGoldenRmTowA + next_epoch / 10.0;   // empty: no satellite in it
            e.mcu_us = t64;
            e.gps_week = kGoldenWeek;
            due.push_back(e);
            next_epoch++;
        }
        while (!due.empty() && t64 - due.front().mcu_us >= 50000)
        {
            sh.onEpoch(due.front());
            due.erase(due.begin());
        }
    }
    EXPECT_TRUE(sh.running());
    EXPECT_GT(sh.stats().epochs_applied, 100u);
    // the tilt alone: the heading is free and nothing here sets it
    const float* q = sh.filter().quat();
    const double down_x = 2.0 * (double(q[1]) * q[3] - double(q[0]) * q[2]);
    const double down_y = 2.0 * (double(q[2]) * q[3] + double(q[0]) * q[1]);
    out.tilt_deg = std::asin(std::min(1.0, std::sqrt(down_x * down_x + down_y * down_y))) * 180.0 / kPi;
    out.levels = sh.stats().levels - levels0;
    return out;
}

}  // namespace

TEST(RawGnssShadowV9, EmptyEpochsNoLongerKeepTheLevellingFromItsOutage)
{
    // Every empty epoch counted as a levelling epoch: the velocity the 5 deg
    // tilt drives (0.85 m/s^2) is never steady, and the epochs keep the outage
    // from coming -- the filter never levels. Counted only when they fuse
    // something: no epoch, an outage, and it levels once the force has been
    // g's size for g_hold_s (5 s) after the jolt.
    const EmptySky on = flyEmptySky(true);
    EXPECT_GT(on.levels, 100u);
    EXPECT_NEAR(on.first_level_t, 2.3 + 5.0, 0.05);
    EXPECT_LT(on.tilt_deg, 1.0);
    const EmptySky off = flyEmptySky(false);
    EXPECT_EQ(off.levels, 0u);
    EXPECT_GT(off.tilt_deg, 4.0);
}

TEST(RawGnssShadowV9, WithSatellitesLevellingOnFusedEpochsChangesNothing)
{
    // every epoch of a flight with a sky fuses something: the same filter, bit
    // for bit
    RawGnssShadow::Config on = mosaicCfg();
    on.level_on_fused_epochs = true;
    RawGnssShadow::Config off = mosaicCfg();
    off.level_on_fused_epochs = false;
    const TenRun a = flyTen(on), b = flyTen(off);
    ASSERT_EQ(a.state.size(), b.state.size());
    for (size_t i = 0; i < a.state.size(); ++i)
        ASSERT_EQ(a.state[i], b.state[i]) << i;
    EXPECT_GT(a.st.levels, 0u);
}

// ---- LevelGate::Config::min_dt_s

namespace
{

const float kStill[3] = {0.0f, 0.0f, -9.81f};   // a steady descent: the drag is g, straight up

// A drogue's swing: 1.2 m/s sideways about a 15 m/s descent, a 2.4 s
// pendulum, at t (s). Steady over each tenth of a second either side of a
// turning point, never over a second: the drag swings 18 deg either side of
// vertical.
void swing(double t, float v[3])
{
    v[0] = 1.0f;
    v[1] = float(1.2 * std::sin(2.0 * kPi * t / 2.4));
    v[2] = 15.0f;
}

}  // namespace

TEST(LevelGateV9, TenHertzEpochsASecondApartGiveTheOneHertzVerdicts)
{
    LevelGate one;                                // the Beetle's: an epoch a second
    LevelGate::Config c;
    c.min_dt_s = 0.95;
    LevelGate ten(c);                             // the mosaic's: ten a second, taken a second apart
    LevelGate raw;                                // ten a second, every one taken
    int raw_steady = 0, one_steady = 0;
    for (int k = 0; k <= 300; ++k)                // 30 s from t = 10
    {
        const double t = 10.0 + k / 10.0;
        float v[3];
        swing(t, v);
        if (k % 10 == 0)
        {
            one.onEpoch(t, v);
            one.onStep(t, kStill);
            one_steady += one.steady() ? 1 : 0;
        }
        ten.onEpoch(t, v);
        ten.onStep(t, kStill);
        raw.onEpoch(t, v);
        raw.onStep(t, kStill);
        raw_steady += raw.steady() ? 1 : 0;
        // between the seconds the last second's verdict stands
        EXPECT_EQ(ten.steady(), one.steady()) << t;
        EXPECT_EQ(ten.allow(t + 0.01, kStill), one.allow(t + 0.01, kStill)) << t;
    }
    // every tenth taken, the turning points pass as steady
    EXPECT_GT(raw_steady, 0);
    EXPECT_GT(raw_steady, one_steady);
}

TEST(LevelGateV9, TheOutageCountsFromTheLastEpochSeen)
{
    // Ten a second until 12.4 s, taken at 10, 11 and 12 (a jolt each second:
    // never steady), the force g's size all along: no outage 2 s after the
    // last epoch the steady test took, one 2 s after the last that came.
    LevelGate::Config c;
    c.min_dt_s = 0.95;
    LevelGate g(c);
    g.onStep(0.0, kStill);
    for (int k = 0; k <= 24; ++k)
    {
        const double t = 10.0 + k / 10.0;
        const float v[3] = {0.0f, 0.0f, (k / 10) % 2 == 0 ? 5.0f : 12.0f};
        g.onEpoch(t, v);
        g.onStep(t, kStill);
    }
    EXPECT_FALSE(g.steady());
    EXPECT_FALSE(g.allow(14.2, kStill));          // 2.2 s after the last taken, 1.8 after the last seen
    EXPECT_TRUE(g.allow(14.45, kStill));          // 2.05 s after the last seen
    // a reset forgets both: no epoch since, an outage
    g.reset();
    g.onStep(14.5, kStill);
    EXPECT_FALSE(g.allow(19.4, kStill));          // g's size 4.9 s
    EXPECT_TRUE(g.allow(19.5, kStill));
}

TEST(LevelGateV9, AtZeroEveryEpochIsTaken)
{
    // the Beetle's: two steady changes in a row, however close
    LevelGate g;
    const float v[3] = {0.0f, 0.0f, 15.0f};
    g.onEpoch(20.0, v);
    g.onEpoch(20.1, v);
    EXPECT_FALSE(g.steady());
    g.onEpoch(20.2, v);
    EXPECT_TRUE(g.steady());
    // with the mosaic's 0.95 s it takes two seconds
    LevelGate::Config c;
    c.min_dt_s = 0.95;
    LevelGate s(c);
    for (int k = 0; k <= 19; ++k)
    {
        s.onEpoch(20.0 + k / 10.0, v);
        EXPECT_FALSE(s.steady()) << k;
    }
    s.onEpoch(22.0, v);
    EXPECT_TRUE(s.steady());
}
