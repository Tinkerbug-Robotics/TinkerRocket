// Host tests for RawGnssShadow's and LevelGate's switches for a receiver fused
// at 10 Hz (the mosaic-G5 on the V8/V9, fc/v9-raw-primary): the pseudorange
// once a second (pr_period_s), the Doppler's stress held for a window
// (rr_dyn_window_s), the heading fit an epoch a second
// (heading_fit_period_s), a restored flight's sigmas and accelerometer bias,
// levelling only on epochs that fused something (level_on_fused_epochs),
// LevelGate's steady test on epochs a second apart (min_dt_s), and the
// profile's satellite cap on its way to the GNSS task's stage. Each at its
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
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <random>
#include <vector>

#include "LevelGate.h"
#include "RawGnssMeasStage.h"
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

// R's rotation vector (R = exp([v x]), |v| below pi)
void rotVec(const Mat3& R, double v[3])
{
    const double c = std::max(-1.0, std::min(1.0, 0.5 * (R.m[0][0] + R.m[1][1] + R.m[2][2] - 1.0)));
    const double th = std::acos(c);
    const double f = th > 1e-9 ? 0.5 * th / std::sin(th) : 0.5;
    v[0] = f * (R.m[2][1] - R.m[1][2]);
    v[1] = f * (R.m[0][2] - R.m[2][0]);
    v[2] = f * (R.m[1][0] - R.m[0][1]);
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
    bool held = false;                    // GNSS held off the attitude for it (att_hold_rate_rps)
    bool skipped = false;                 // nothing of it fused while the body turned (rot_skip_rate_rps)
    uint32_t dv_epoch_fails = 0;          // the Doppler change check's failed epochs so far
    uint32_t dv_latch_releases = 0;       //   its latches let go so far (dv_latch_k)
    float dv_level_chi2 = 0.0f;           //   and the last level test's chi-square
    float dv_lever_mps = 0.0f;            //   the lever allowance its last epoch test took
    float dv_lever_win_mps = 0.0f;        //     and its window's
    uint32_t dv_epoch_win_fails = 0;      //   its failed epochs on the window's sum alone so far
};

struct TenRun
{
    std::vector<ClosedEpoch> eps;
    std::vector<uint64_t> state;          // stateHash every 0.25 s
    std::vector<double> att_err_deg;      //   and the attitude's error then against the truth, deg
    double vel_err_max_mps = 0.0;         // the velocity's largest error against the truth's from ignition, m/s
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

struct TenFlight
{
    double lean_deg = 0.0;
    double t_end = 10.0;
    double thrust_g = 3.0;               // along the nose at ignition,
    double burnout_thrust_g = 3.0;       //   at burnout, and straight between
    int    epoch_ms = 100;               // an epoch this often (a multiple of 5 ms)
    bool   lose_whole_seconds = false;   // the epochs on whole GPS seconds never come
    double bad_vertical_t = -1.0;        // the epoch at this time: its Dopplers 40 m/s off vertically
    // a turn in flight: from turn_t (after ignition) the body turns about
    // its turn_axis (0 the nose, 1 body y) at turn_rps for turn_s, then back
    // as fast for as long, the thrust turning with it; the antenna is lever_m
    // up the nose from the IMU (the filter takes it at the IMU)
    double turn_t = -1.0;
    double turn_s = 0.5;
    double turn_rps = 0.0;
    int    turn_axis = 1;
    double lever_m = 0.0;
    double lever_y_m = 0.0;              // and this far along body y (across the nose)
    double lever_z_m = 0.0;              //   and body z
    double apogee_t = -1.0;              // the flight computer calls apogee from here (kShadowApogee)
    double seed_heading_err_deg = 0.0;   // the pad seed's heading this far off the body's (about the down axis)
    // the whole receiver stale (test_doppler_change.cpp's pattern, sim_events.py's): its Dopplers at the pad's
    // velocity through the boost, closing on the truth with this time constant from burnout (0: none)
    double stale_tau = 0.0;
    // a coherent ramp (test_doppler_change.cpp's, sim_events.py's slow pattern): every Doppler at the receiver's
    // velocity plus coh_rate m/s a second down from coh_from s, held at coh_max (coh_from < 0: none)
    double coh_from = -1.0;
    double coh_rate = 0.0;
    double coh_max = 0.0;
    // a weathercocking body: from wc_t for wc_s, its rate about the turn's axis a sine of wc_rps at wc_hz
    // (on top of any turn above)
    double wc_t = -1.0;
    double wc_s = 1.0;
    double wc_rps = 0.0;
    double wc_hz = 2.0;
    // a roll about the nose at roll_rps (rad/s) from roll_t, on top of the turn and the weathercock (theirs then
    // about their axis as it stood before the roll, fixed in NED: the nose turns as it would without the roll), the
    // gyro each step's whole turn (0: none)
    double roll_t = -1.0;
    double roll_rps = 0.0;
    // a calibrated magnetometer on the pad (the truth's field, 50 Hz, to ignition): the launch's heading is
    // the magnetometer's (the Doppler change check's dv_hdg_unset_deg judges it known)
    bool   mag_on_pad = false;
    // each epoch handed over in pairs (TimedRawEpoch::rr_paired; the config's ekf.rr_pair_lag_s to match): its
    // Dopplers the antenna's velocity this long before the epoch's tag, the pair's middle (a multiple of 5 ms; 0:
    // none, the epoch's own instant)
    int    pair_lag_ms = 0;
};

TenRun flyTen(const RawGnssShadow::Config& cfg, const TenFlight& fl)
{
    const double lean_deg = fl.lean_deg, t_end = fl.t_end;
    const int epoch_steps = fl.epoch_ms / 5;
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
    const double down[3] = {0.0, 0.0, 1.0};
    quatOf(mat3Mul(rotAbout(down, fl.seed_heading_err_deg * 0.017453292519943295), Ct), seed.quat);
    const double b_ned[3] = {21.0, -4.3, 45.5};              // the field, uT (test_raw_gnss_shadow.cpp's RailPad)
    float b_body[3] = {};
    if (fl.mag_on_pad)
    {
        seed.declination_rad = float(std::atan2(b_ned[1], b_ned[0]));
        seed.field_ut = float(std::sqrt(b_ned[0] * b_ned[0] + b_ned[1] * b_ned[1] + b_ned[2] * b_ned[2]));
        seed.have_declination = 1;
        seed.dip_rad = float(std::atan2(b_ned[2], std::hypot(b_ned[0], b_ned[1])));
        seed.have_dip = 1;
        for (int i = 0; i < 3; ++i)
            b_body[i] = float(Ct.m[0][i] * b_ned[0] + Ct.m[1][i] * b_ned[1] + Ct.m[2][i] * b_ned[2]);
    }
    sh.onSeed(seed);
    const double tow0 = kGoldenRmTowA, dt = 0.005;
    double p_ned[3] = {}, v_ned[3] = {};
    std::vector<TimedRawEpoch> due;
    int next_epoch = 1;                                       // tenths of a second
    uint32_t last_tow_ms = 0;
    TenRun run;
    const size_t pair_steps = static_cast<size_t>(fl.pair_lag_ms / 5);
    std::deque<std::array<double, 3>> ve_back;                // in pairs: the antenna's ECEF velocity, the last steps
    Mat3 C_prev = Ct;                                         // the roll's: the attitude a step before
    for (int step = 1; step * dt <= t_end + 1e-9; ++step)
    {
        const double t = step * dt;
        // the turn (in flight only: the pad stands still): its rate over the step ending at t, and the angle at t
        double w_turn = 0.0, ang = 0.0;
        if (fl.turn_t >= 0.0 && t > fl.turn_t + 1e-9 && t <= fl.turn_t + 2.0 * fl.turn_s + 1e-9)
        {
            const bool out = t <= fl.turn_t + fl.turn_s + 1e-9;
            w_turn = out ? fl.turn_rps : -fl.turn_rps;
            ang = fl.turn_rps * (out ? t - fl.turn_t : fl.turn_t + 2.0 * fl.turn_s - t);
        }
        if (fl.wc_rps > 0.0 && t > fl.wc_t + 1e-9)
        {
            // the angle at t and the step's mean rate (the gyro's), the sine's integral
            const double k = 2.0 * kPi * fl.wc_hz;
            auto wcAng = [&](double at) {
                const double s = std::min(at, fl.wc_t + fl.wc_s) - fl.wc_t;
                return s > 0.0 ? fl.wc_rps / k * (1.0 - std::cos(k * s)) : 0.0;
            };
            ang += wcAng(t);
            w_turn += (wcAng(t) - wcAng(t - dt)) / dt;
        }
        const double ax[3] = {fl.turn_axis == 0 ? 1.0 : 0.0, fl.turn_axis == 1 ? 1.0 : 0.0, 0.0};
        const Mat3 Cturn = mat3Mul(Ct, rotAbout(ax, ang));   // body -> NED, turned
        const double nose[3] = {1.0, 0.0, 0.0};
        const double roll = fl.roll_rps != 0.0 && t > fl.roll_t + 1e-9 ? fl.roll_rps * (t - fl.roll_t) : 0.0;
        const Mat3 Croll = rotAbout(nose, roll);
        const Mat3 Cn = fl.roll_rps != 0.0 ? mat3Mul(Cturn, Croll) : Cturn;   //   and rolled
        double f_b[3] = {0.0, 0.0, 0.0}, a_n[3] = {0.0, 0.0, 0.0};
        if (t > kIgn)
        {
            const double thrust = fl.thrust_g + (fl.burnout_thrust_g - fl.thrust_g) * (t - kIgn) / (kBurnout - kIgn);
            f_b[0] = t <= kBurnout ? thrust * kG : 0.0;       // then coasting, weightless
            double f_n[3];
            mat3Vec(Cn, f_b, f_n);
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
        if (fl.roll_rps != 0.0)
        {
            // the step's whole turn (the roll and the turn do not commute): C_prev' Cn's rotation vector over dt
            Mat3 CpT{};
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    CpT.m[i][j] = C_prev.m[j][i];
            double rv[3];
            rotVec(mat3Mul(CpT, Cn), rv);
            for (int i = 0; i < 3; ++i)
                s.gyro[i] = float(rv[i] / dt);
            C_prev = Cn;
        }
        else
        {
            s.gyro[fl.turn_axis] = float(w_turn);
        }
        s.flags = t < kIgn ? gnss_raw::kShadowLevel : (t >= kIgn + 0.1 ? gnss_raw::kShadowInFlight : 0);
        if (fl.apogee_t >= 0.0 && t >= fl.apogee_t)
            s.flags |= gnss_raw::kShadowApogee;
        if (fl.mag_on_pad && t < kIgn && step % 4 == 0)
        {
            gnss_raw::ShadowMag mg{};
            mg.time_us = s.time_us;
            for (int i = 0; i < 3; ++i)
                mg.mag[i] = b_body[i];
            mg.flags = gnss_raw::kShadowMagSim;
            sh.onMag(mg);
        }
        sh.onImu(s, t64);
        if (t > kIgn)
        {
            const float* vf = sh.filter().velNed();
            double e2 = 0.0;
            for (int i = 0; i < 3; ++i)
                e2 += (double(vf[i]) - v_ned[i]) * (double(vf[i]) - v_ned[i]);
            run.vel_err_max_mps = std::max(run.vel_err_max_mps, std::sqrt(e2));
        }
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
            c.held = sh.stats().att_last_held;
            c.skipped = sh.stats().rot_last_skipped;
            c.dv_epoch_fails = sh.stats().dv_epoch_fails;
            c.dv_latch_releases = sh.stats().dv_latch_releases;
            c.dv_level_chi2 = sh.stats().dv_last_level_chi2;
            c.dv_lever_mps = sh.stats().dv_last_lever_mps;
            c.dv_lever_win_mps = sh.stats().dv_last_lever_win_mps;
            c.dv_epoch_win_fails = sh.stats().dv_epoch_win_fails;
            run.eps.push_back(c);
        }
        gnss_raw::ShadowNav nav;
        sh.navState(nav);
        if (nav.gnss_aided && run.first_aided_t < 0.0)
            run.first_aided_t = t;
        else if (!nav.gnss_aided && run.first_aided_t >= 0.0)
            run.unaided_ticks_after++;
        if (step % 50 == 0)
        {
            run.state.push_back(stateHash(sh.filter()));
            float qt[4];
            quatOf(Cn, qt);
            const float* qf = sh.filter().quat();
            double d = 0.0;
            for (int i = 0; i < 4; ++i)
                d += double(qf[i]) * qt[i];
            run.att_err_deg.push_back(2.0 * std::acos(std::min(1.0, std::fabs(d))) * 57.29577951308232);
        }
        // in pairs: the antenna's velocity each step (no faults), the epoch's Dopplers taken pair_steps back
        if (pair_steps > 0)
        {
            const double r_b[3] = {fl.lever_m, fl.lever_y_m, fl.lever_z_m};
            double w_b[3] = {ax[0] * w_turn, ax[1] * w_turn, 0.0};
            if (fl.roll_rps != 0.0)
            {
                const double wt[3] = {w_b[0], w_b[1], w_b[2]};
                for (int i = 0; i < 3; ++i)
                    w_b[i] = Croll.m[0][i] * wt[0] + Croll.m[1][i] * wt[1] + Croll.m[2][i] * wt[2];
                if (roll != 0.0)
                    w_b[0] += fl.roll_rps;
            }
            const double wxr[3] = {w_b[1] * r_b[2] - w_b[2] * r_b[1], w_b[2] * r_b[0] - w_b[0] * r_b[2],
                                   w_b[0] * r_b[1] - w_b[1] * r_b[0]};
            double wxr_n[3], ve_s[3];
            mat3Vec(Cn, wxr, wxr_n);
            const double v_rx[3] = {v_ned[0] + wxr_n[0], v_ned[1] + wxr_n[1], v_ned[2] + wxr_n[2]};
            nedToEcefVec(v_rx, ve_s);
            ve_back.push_back({ve_s[0], ve_s[1], ve_s[2]});
            if (ve_back.size() > pair_steps + 1)
                ve_back.pop_front();
        }
        // an epoch every epoch_ms of GPS time (a tenth of a second), built
        // where the receiver truly is
        if (step % epoch_steps == 0)
        {
            const int ms = next_epoch * fl.epoch_ms;
            if (!fl.lose_whole_seconds || ms % 1000 != 0)
            {
                // the antenna: lever_m up the nose, moving omega x r with the turn
                const double r_b[3] = {fl.lever_m, fl.lever_y_m, fl.lever_z_m};
                double w_b[3] = {ax[0] * w_turn, ax[1] * w_turn, 0.0};
                if (fl.roll_rps != 0.0)
                {
                    // rolled: the turn's rate in the rolled body, and the roll
                    const double wt[3] = {w_b[0], w_b[1], w_b[2]};
                    for (int i = 0; i < 3; ++i)
                        w_b[i] = Croll.m[0][i] * wt[0] + Croll.m[1][i] * wt[1] + Croll.m[2][i] * wt[2];
                    if (roll != 0.0)
                        w_b[0] += fl.roll_rps;
                }
                const double wxr[3] = {w_b[1] * r_b[2] - w_b[2] * r_b[1], w_b[2] * r_b[0] - w_b[0] * r_b[2],
                                       w_b[0] * r_b[1] - w_b[1] * r_b[0]};
                double r_n[3], wxr_n[3], p_ant[3];
                mat3Vec(Cn, r_b, r_n);
                mat3Vec(Cn, wxr, wxr_n);
                for (int i = 0; i < 3; ++i)
                    p_ant[i] = p_ned[i] + r_n[i];
                double dr[3], r[3], ve[3];
                nedToEcefVec(p_ant, dr);
                double v_rx[3] = {v_ned[0] + wxr_n[0], v_ned[1] + wxr_n[1], v_ned[2] + wxr_n[2]};
                if (std::fabs(ms / 1000.0 - fl.bad_vertical_t) < 1e-6)
                    v_rx[2] += 40.0;
                if (fl.stale_tau > 0.0 && t > kIgn)
                {
                    const double L = t <= kBurnout ? 1.0 : std::exp(-(t - kBurnout) / fl.stale_tau);
                    for (int i = 0; i < 3; ++i)
                        v_rx[i] -= L * v_ned[i];
                }
                if (fl.coh_from >= 0.0 && t > fl.coh_from)
                    v_rx[2] += std::min(fl.coh_rate * (t - fl.coh_from), fl.coh_max);
                nedToEcefVec(v_rx, ve);
                for (int i = 0; i < 3; ++i)
                    r[i] = r0[i] + dr[i];
                if (pair_steps > 0 && ve_back.size() == pair_steps + 1)
                    for (int i = 0; i < 3; ++i)
                        ve[i] = ve_back.front()[i];
                TimedRawEpoch e;
                e.epoch = sky.epoch(tow0 + ms / 1000.0, r, ve);
                e.mcu_us = t64;
                e.gps_week = kGoldenWeek;
                if (pair_steps > 0)
                    e.rr_paired = 1;
                due.push_back(e);
            }
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

TenRun flyTen(const RawGnssShadow::Config& cfg, double lean_deg = 0.0, double t_end = 10.0)
{
    TenFlight fl;
    fl.lean_deg = lean_deg;
    fl.t_end = t_end;
    return flyTen(cfg, fl);
}

// The epochs that fused the code are the first of each GPS second that was
// fused (not dropped): one a second, whichever epochs came
void expectCodeOnceASecond(const TenRun& run)
{
    long last_s = -1;
    for (const ClosedEpoch& c : run.eps)
    {
        if (c.st.screen_dropped)
            continue;
        const long s = std::lround(c.t * 1000.0) / 1000;
        const bool first = s != last_s;
        EXPECT_EQ(c.st.used_pr > 0, first) << "t " << c.t;
        EXPECT_GT(c.st.used_rr, 4) << "t " << c.t;
        if (first)
            last_s = s;
    }
}

}  // namespace

// ---- pr_period_s: the code once a second

TEST(RawGnssShadowV9, AtTenHertzThePseudorangeIsFusedOnceASecond)
{
    // The mosaic's profile: the Dopplers every epoch, the pseudoranges on the
    // first epoch of each GPS second -- the start's first, then the whole
    // seconds -- and gnss_aided (a pseudorange within 2 s) holds from the
    // first one to the end, through the boost and the coast
    const TenRun run = flyTen(mosaicCfg());
    ASSERT_GT(run.eps.size(), 80u);
    expectCodeOnceASecond(run);
    int with_pr = 0, whole = 0;
    for (const ClosedEpoch& c : run.eps)
        if (c.st.used_pr > 0)
        {
            with_pr++;
            whole += std::lround(c.t * 1000.0) % 1000 == 0 ? 1 : 0;
        }
    EXPECT_GE(whole, 9);                          // 1 s to 9 s
    EXPECT_EQ(with_pr, whole + 1);                // and the start's first, at 0.1 s
    EXPECT_GE(run.first_aided_t, 0.0);
    EXPECT_LT(run.first_aided_t, 0.5);
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

TEST(RawGnssShadowV9, AWholeSecondsEpochLostMovesTheCodeToTheNext)
{
    // Every whole second's epoch lost (stale, or never built): the code goes
    // on the next epoch, 0.1 s late, not a second late -- and gnss_aided
    // never lapses. (On the whole seconds only, the code came every 2 s.)
    TenFlight fl;
    fl.lose_whole_seconds = true;
    const TenRun run = flyTen(mosaicCfg(), fl);
    ASSERT_GT(run.eps.size(), 70u);
    expectCodeOnceASecond(run);
    int with_pr = 0;
    double last = -1.0;
    for (const ClosedEpoch& c : run.eps)
        if (c.st.used_pr > 0)
        {
            with_pr++;
            if (last >= 0.0)
                EXPECT_NEAR(c.t - last, 1.0, 0.15) << "t " << c.t;
            last = c.t;
        }
    EXPECT_GE(with_pr, 9);
    EXPECT_GE(run.first_aided_t, 0.0);
    EXPECT_EQ(run.unaided_ticks_after, 0);
    EXPECT_EQ(run.st.stops, 0u);
}

TEST(RawGnssShadowV9, AWholeSecondsEpochDroppedMovesTheCodeToTheNext)
{
    // The vertical check drops the epoch at 3 s (its Dopplers 40 m/s off
    // vertically): none of it fused, so the second's code goes on the next
    // epoch fused, at 3.1 s
    TenFlight fl;
    fl.bad_vertical_t = 3.0;
    const TenRun run = flyTen(mosaicCfg(), fl);
    const ClosedEpoch* dropped = nullptr;
    const ClosedEpoch* next = nullptr;
    for (const ClosedEpoch& c : run.eps)
    {
        if (std::fabs(c.t - 3.0) < 0.01)
            dropped = &c;
        if (std::fabs(c.t - 3.1) < 0.01)
            next = &c;
    }
    ASSERT_NE(dropped, nullptr);
    ASSERT_NE(next, nullptr);
    EXPECT_TRUE(dropped->st.screen_dropped);
    EXPECT_FALSE(next->st.screen_dropped);
    EXPECT_GT(next->st.used_pr, 0);
    expectCodeOnceASecond(run);
    EXPECT_EQ(run.unaided_ticks_after, 0);
}

TEST(RawGnssShadowV9, EpochsOffTheWholeSecondStillFuseTheCodeEachSecond)
{
    // A bench's 150 ms epochs: one falls on a whole second only every 3 s.
    // The code still goes on the first epoch of each second, 0.9-1.05 s apart
    TenFlight fl;
    fl.epoch_ms = 150;
    const TenRun run = flyTen(mosaicCfg(), fl);
    ASSERT_GT(run.eps.size(), 50u);
    expectCodeOnceASecond(run);
    int with_pr = 0;
    double last = -1.0;
    for (const ClosedEpoch& c : run.eps)
        if (c.st.used_pr > 0)
        {
            with_pr++;
            if (last >= 0.0)
            {
                EXPECT_GE(c.t - last, 0.85) << "t " << c.t;
                EXPECT_LE(c.t - last, 1.1) << "t " << c.t;
            }
            last = c.t;
        }
    EXPECT_GE(with_pr, 9);
    EXPECT_EQ(run.unaided_ticks_after, 0);
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

TEST(RawGnssShadowV9, TheStressHeldIsTheWindowsPeakUnderAFallingThrust)
{
    // A boost falling from 5 g to 2 g at burnout: each epoch's own peak is
    // smaller than the one before it. Held, an epoch's stress is the largest
    // any epoch within the second before it carried: the second after
    // burnout keeps at least the last boost epoch's, and none is left once
    // the window has passed it. (A hold of one peak, replaced by the epoch
    // of the moment when it aged out, let the boost's go 0.1 s after
    // burnout.) Each epoch's stress^2 is its Doppler sigma's square less a
    // flight's with none.
    TenFlight fl;
    fl.thrust_g = 5.0;
    fl.burnout_thrust_g = 2.0;
    RawGnssShadow::Config c_none = mosaicCfg();
    c_none.meas.rr_dyn_sigma = 0.0;
    RawGnssShadow::Config c_bare = mosaicCfg();
    c_bare.rr_dyn_window_s = 0.0;
    const TenRun none = flyTen(c_none, fl), bare = flyTen(c_bare, fl), held = flyTen(mosaicCfg(), fl);
    auto stress2 = [&](const TenRun& r, double t) -> double {
        const ClosedEpoch* a = nullptr;
        const ClosedEpoch* z = nullptr;
        for (const ClosedEpoch& c : r.eps)
            if (std::fabs(c.t - t) < 0.01)
                a = &c;
        for (const ClosedEpoch& c : none.eps)
            if (std::fabs(c.t - t) < 0.01)
                z = &c;
        EXPECT_NE(a, nullptr) << t;
        EXPECT_NE(z, nullptr) << t;
        if (a == nullptr || z == nullptr)
            return 0.0;
        return double(a->sigma_rr_min) * a->sigma_rr_min - double(z->sigma_rr_min) * z->sigma_rr_min;
    };
    // the epochs' own stress (window 0): the last boost epoch's
    double last_boost_t = -1.0, last_boost = 0.0;
    for (int k = 30; k <= 90; ++k)
    {
        const double b = stress2(bare, k / 10.0);
        if (b > 0.01)
        {
            last_boost_t = k / 10.0;
            last_boost = b;
        }
    }
    ASSERT_NEAR(last_boost_t, kBurnout, 0.15);
    EXPECT_GT(last_boost, 0.2);                   // 2 g at burnout: 1 g over, 0.5 m/s
    for (int k = 30; k <= 90; ++k)
    {
        const double t = k / 10.0;
        // the window's peak: of the epochs well inside it at least, of those
        // on its edge too at most
        double lo = 0.0, hi = 0.0;
        for (int j = k - 10; j <= k; ++j)
        {
            const double b = stress2(bare, j / 10.0);
            hi = std::max(hi, b);
            if (j > k - 10)
                lo = std::max(lo, b);
        }
        const double h = stress2(held, t);
        EXPECT_GE(h, lo - 0.01) << t;
        EXPECT_LE(h, hi + 0.01) << t;
        if (t > last_boost_t + 0.01 && t < last_boost_t + 0.95)
            EXPECT_GE(h, last_boost - 0.01) << t;
        if (t > last_boost_t + 1.05)
            EXPECT_LT(h, 0.01) << t;
    }
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

// ---- sat_cap: the profile's cap reaches the GNSS task's stage

namespace
{

// RawGnssMeasStage::Config::sat_cap, where the stage has it (its own change,
// the mosaic's cap); -1 where it has none yet
template <class S>
auto stageSatCap(const S& s, int) -> decltype(static_cast<int>(s.sat_cap))
{
    return static_cast<int>(s.sat_cap);
}

template <class S>
int stageSatCap(const S&, long)
{
    return -1;
}

}  // namespace

TEST(RawGnssShadowV9, TheProfilesSatelliteCapReachesTheGnssTasksStage)
{
    // The GNSS task builds with RawGnssShadow::stageConfig(cfg): the mosaic's
    // 16 must arrive there, or the V8 builds every satellite of a 28-strong
    // sky ten times a second; the LC86G's none
    RawGnssShadow::Config m = primaryCfg();
    gnss_raw::applyReceiverProfile(gnss_raw::mosaicG5Profile(), m);
    ASSERT_EQ(m.sat_cap, 16);
    RawGnssShadow::Config b = primaryCfg();
    gnss_raw::applyReceiverProfile(gnss_raw::lc86gProfile(), b);
    ASSERT_EQ(b.sat_cap, 0);
    const int got = stageSatCap(RawGnssShadow::stageConfig(m), 0);
    if (got < 0)
        GTEST_SKIP() << "RawGnssMeasStage::Config has no sat_cap yet (the stage's change)";
    EXPECT_EQ(got, 16);
    EXPECT_EQ(stageSatCap(RawGnssShadow::stageConfig(b), 0), 0);
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

TEST(RawGnssShadowV9, EachEpochIsTakenInPairsOrAsItCameByItsMark)
{
    // The mosaic's Dopplers in pairs: each epoch's job takes the epoch's mark
    // (TimedRawEpoch / BuiltEpoch rr_paired) to the filter, so an epoch in
    // pairs has its Dopplers 25 ms further back and the next, as it came, at
    // its own instant -- built here or by the GNSS task. With the pairs off
    // (no pair lag: every profile as it is) a mark changes nothing.
    const uint8_t marks[] = {1, 1, 0, 1, 0, 0, 1, 0};
    for (bool pairs_on : {true, false})
        for (bool external : {false, true})
        {
            SCOPED_TRACE(testing::Message() << (pairs_on ? "pairs on" : "pairs off")
                                            << (external ? ", built by the GNSS task" : ", built here"));
            gnss_raw::GnssReceiverProfile p = gnss_raw::mosaicG5Profile();
            p.rr_pair_avg = pairs_on;
            RawGnssShadow::Config cfg = primaryCfg();
            gnss_raw::applyReceiverProfile(p, cfg);
            cfg.external_build = external;
            EXPECT_EQ(cfg.ekf.rr_lag_s, 0.0);
            EXPECT_EQ(cfg.ekf.rr_pair_lag_s, pairs_on ? 0.025 : 0.0);
            RawGnssShadow sh(cfg);
            sh.onSeed(padSeed(kT0, kPadLat, kPadLon, 1400.0));
            const double dt = 0.005;
            std::vector<TimedRawEpoch> due;
            int next_epoch = 1, checked = 0, pending = -1;
            for (int step = 1; step <= 600; ++step)
            {
                const int64_t t64 = kT0 + static_cast<int64_t>(std::llround(step * dt * 1e6));
                ShadowImu s{};
                s.time_us = static_cast<uint32_t>(t64);
                s.acc[2] = static_cast<float>(-kG);
                sh.onImu(s, t64);
                if (pending >= 0)
                {
                    // the epoch handed over before this step: its job ran in it
                    const bool want = pairs_on && pending == 1;
                    EXPECT_EQ(sh.filter().rrPaired(), want) << "epoch " << checked;
                    EXPECT_EQ(sh.filter().rrLag(), want ? 0.025 : 0.0);
                    checked++;
                    pending = -1;
                }
                if (step % 20 == 0)
                {
                    TimedRawEpoch e;
                    e.epoch.tow_s = kGoldenRmTowA + next_epoch / 10.0;   // empty: the mark alone matters here
                    e.mcu_us = t64;
                    e.gps_week = kGoldenWeek;
                    e.rr_paired = marks[next_epoch % 8];
                    due.push_back(e);
                    next_epoch++;
                }
                while (!due.empty() && t64 - due.front().mcu_us >= 50000)
                {
                    const TimedRawEpoch& e = due.front();
                    if (sh.running())
                    {
                        if (external)
                        {
                            gnss_raw::BuiltEpoch b;
                            b.mcu_us = e.mcu_us;
                            b.tow_s = e.epoch.tow_s;
                            b.gps_week = e.gps_week;
                            b.rr_paired = e.rr_paired;
                            sh.onBuilt(b);
                        }
                        else
                        {
                            sh.onEpoch(e);
                        }
                        pending = e.rr_paired;
                    }
                    due.erase(due.begin());
                }
            }
            EXPECT_GT(checked, 20);
        }
}

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

// ---- the turning body: rot_lever_axial_m and rot_lever_radial_m, att_hold_rate_rps and att_hold_s

namespace
{

const ClosedEpoch* epochAt(const TenRun& r, double t)
{
    for (const ClosedEpoch& c : r.eps)
        if (std::fabs(c.t - t) < 0.01)
            return &c;
    return nullptr;
}

// out and back over 1 s from 7 s, in the coast
TenFlight turningCoast(double rps, int axis, double lever_m)
{
    TenFlight fl;
    fl.turn_t = 7.0;
    fl.turn_s = 0.5;
    fl.turn_rps = rps;
    fl.turn_axis = axis;
    fl.lever_m = lever_m;
    return fl;
}

}  // namespace

TEST(RawGnssShadowV9, TheTurningBodysAllowanceIsOnTheDopplersOfTheEpochsItTurnedIn)
{
    // 2 rad/s about body y: with 0.3 m along the nose and 0.05 m across it
    // allowed, every Doppler of an epoch whose interval (since the epoch
    // before) saw the turn takes 0.3 x 2 + 0.05 x 2 = 0.7 m/s in quadrature
    // -- the epochs tagged 7.0 to 8.0 s, each handed over 50 ms after its tag.
    // Every other epoch is as without the allowance, and so is the filter
    // until the turn.
    const TenFlight fl = turningCoast(2.0, 1, 0.0);
    RawGnssShadow::Config c = mosaicCfg();
    c.rot_lever_axial_m = 0.3;
    c.rot_lever_radial_m = 0.05;
    const TenRun on = flyTen(c, fl), bare = flyTen(mosaicCfg(), fl);
    int turned = 0;
    for (int k = 30; k <= 95; ++k)
    {
        const double t = k / 10.0;
        const ClosedEpoch* a = epochAt(on, t);
        const ClosedEpoch* b = epochAt(bare, t);
        ASSERT_NE(a, nullptr) << t;
        ASSERT_NE(b, nullptr) << t;
        const double a2 = double(a->sigma_rr_min) * a->sigma_rr_min, b2 = double(b->sigma_rr_min) * b->sigma_rr_min;
        if (t > 7.0 - 1e-6 && t < 8.0 + 1e-6)
        {
            EXPECT_NEAR(a2 - b2, 0.49, 0.02) << t;
            turned++;
        }
        else
        {
            EXPECT_NEAR(a2, b2, 1e-4 * b2) << t;
        }
        EXPECT_FALSE(a->held) << t;
    }
    EXPECT_EQ(turned, 11);
    EXPECT_NEAR(on.st.rot_max_sigma_mps, 0.7f, 0.02f);
    EXPECT_EQ(on.st.att_held, 0u);
    // the filter itself: the same as without it until the turn
    ASSERT_EQ(on.state.size(), bare.state.size());
    int same = 0;
    for (size_t i = 0; i < on.state.size() && 0.25 * double(i + 1) < 7.0; ++i)
    {
        EXPECT_EQ(on.state[i], bare.state[i]) << i;
        same++;
    }
    EXPECT_EQ(same, 27);
    // a roll -- the nose's own axis -- moves an antenna on it not at all: only
    // the radial 0.05 m takes it, 0.05 x 5
    const TenRun roll = flyTen(c, turningCoast(5.0, 0, 0.0));
    EXPECT_NEAR(roll.st.rot_max_sigma_mps, 0.25f, 0.01f);
    // only after the apogee call (rot_after_apogee): called at 7.5 s, the
    // epochs of the turn handed over before it take nothing
    c.rot_after_apogee = true;
    TenFlight late = fl;
    late.apogee_t = 7.5;
    const TenRun after = flyTen(c, late), after_bare = flyTen(mosaicCfg(), late);
    int allowed = 0;
    for (int k = 30; k <= 95; ++k)
    {
        const double t = k / 10.0;
        const ClosedEpoch* a = epochAt(after, t);
        const ClosedEpoch* b = epochAt(after_bare, t);
        ASSERT_NE(a, nullptr) << t;
        ASSERT_NE(b, nullptr) << t;
        const double a2 = double(a->sigma_rr_min) * a->sigma_rr_min, b2 = double(b->sigma_rr_min) * b->sigma_rr_min;
        if (t > 7.5 - 1e-6 && t < 8.0 + 1e-6)
        {
            EXPECT_NEAR(a2 - b2, 0.49, 0.02) << t;
            allowed++;
        }
        else
        {
            EXPECT_NEAR(a2, b2, 1e-4 * b2) << t;
        }
    }
    EXPECT_EQ(allowed, 6);
    // with a floor on the rate across the nose: a 2 rad/s turn under a 2.5
    // floor takes nothing, over a 1.5 floor the whole bound
    RawGnssShadow::Config cm = mosaicCfg();
    cm.rot_lever_axial_m = 0.3;
    cm.rot_lever_radial_m = 0.05;
    cm.rot_lever_min_rps = 2.5;
    EXPECT_EQ(flyTen(cm, fl).st.rot_max_sigma_mps, 0.0f);
    cm.rot_lever_min_rps = 1.5;
    EXPECT_NEAR(flyTen(cm, fl).st.rot_max_sigma_mps, 0.7f, 0.02f);
}

TEST(RawGnssShadowV9, TheFlightSafeguardsAllowASnatchNotAWeathercockOrARoll)
{
    // The mosaic image's allowance (applyMosaicFlightSafeguards: 0.3 m along
    // the nose, 0.03 m across, from 2 rad/s across the nose): a 3 rad/s turn
    // about body y takes 0.3 x 3 + 0.03 x 3 m/s; a 1.5 rad/s one (a boost's
    // weathercocking) and a 5 rad/s roll (nothing across the nose) take none
    RawGnssShadow::Config c = mosaicCfg();
    gnss_raw::applyMosaicFlightSafeguards(c);
    EXPECT_NEAR(flyTen(c, turningCoast(3.0, 1, 0.0)).st.rot_max_sigma_mps, 0.99f, 0.03f);
    EXPECT_EQ(flyTen(c, turningCoast(1.5, 1, 0.0)).st.rot_max_sigma_mps, 0.0f);
    EXPECT_EQ(flyTen(c, turningCoast(5.0, 0, 0.0)).st.rot_max_sigma_mps, 0.0f);
    EXPECT_EQ(flyTen(c, turningCoast(3.0, 1, 0.0)).st.att_held, 0u);
}

TEST(RawGnssShadowV9, TheHoldTakesTheEpochsThatTurnedAcrossTheNoseAndTheTailAfter)
{
    // Held from 1.5 rad/s across the nose and for 0.25 s after: a 2 rad/s
    // pitch holds the 11 epochs that saw it and the 2 within 0.25 s after the
    // last; a 5 rad/s roll holds none. No allowance on the Dopplers was asked
    // for: their sigmas are the bare flight's.
    RawGnssShadow::Config c = mosaicCfg();
    c.att_hold_rate_rps = 1.5;
    c.att_hold_s = 0.25;
    const TenRun pitch = flyTen(c, turningCoast(2.0, 1, 0.0)), bare = flyTen(mosaicCfg(), turningCoast(2.0, 1, 0.0));
    int held = 0;
    for (const ClosedEpoch& e : pitch.eps)
    {
        const bool want = e.t > 7.0 - 1e-6 && e.t < 8.2 + 1e-6;
        EXPECT_EQ(e.held, want) << e.t;
        held += e.held ? 1 : 0;
        const ClosedEpoch* b = epochAt(bare, e.t);
        ASSERT_NE(b, nullptr) << e.t;
        EXPECT_EQ(e.sigma_rr_min, b->sigma_rr_min) << e.t;
    }
    EXPECT_EQ(held, 13);
    EXPECT_EQ(pitch.st.att_held, 13u);
    EXPECT_EQ(pitch.st.rot_max_sigma_mps, 0.0f);
    const TenRun roll = flyTen(c, turningCoast(5.0, 0, 0.0));
    EXPECT_EQ(roll.st.att_held, 0u);
    // under the threshold, nothing is held: 1.4 rad/s
    const TenRun slow = flyTen(c, turningCoast(1.4, 1, 0.0));
    EXPECT_EQ(slow.st.att_held, 0u);
    // only after the apogee call: called at 7.5 s, the epochs of the turn
    // handed over before it are not held, those after it and the tail are
    c.rot_after_apogee = true;
    TenFlight fl = turningCoast(2.0, 1, 0.0);
    fl.apogee_t = 7.5;
    const TenRun late = flyTen(c, fl);
    for (const ClosedEpoch& e : late.eps)
    {
        const bool want = e.t > 7.5 - 1e-6 && e.t < 8.2 + 1e-6;
        EXPECT_EQ(e.held, want) << e.t;
    }
    EXPECT_EQ(late.st.att_held, 8u);
    // and with no call at all, none
    const TenRun never = flyTen(c, turningCoast(2.0, 1, 0.0));
    EXPECT_EQ(never.st.att_held, 0u);
}

TEST(RawGnssShadowV9, KeepingTheRateForTheAllowancesChangesNothingUntilOneActs)
{
    // A hold that never triggers (100 rad/s) keeps the filter's rate, and
    // nothing else: through the turn, every 0.25 s, the filter is the
    // default's bit for bit
    RawGnssShadow::Config c = mosaicCfg();
    c.att_hold_rate_rps = 100.0;
    const TenFlight fl = turningCoast(3.0, 1, 0.3);
    const TenRun kept = flyTen(c, fl), bare = flyTen(mosaicCfg(), fl);
    ASSERT_EQ(kept.state.size(), bare.state.size());
    for (size_t i = 0; i < kept.state.size(); ++i)
        EXPECT_EQ(kept.state[i], bare.state[i]) << i;
    EXPECT_EQ(kept.st.att_held, 0u);
    // and so does a skip that never triggers
    RawGnssShadow::Config k2 = mosaicCfg();
    k2.rot_skip_rate_rps = 100.0;
    k2.rot_skip_s = 1.0;
    const TenRun kept2 = flyTen(k2, fl);
    ASSERT_EQ(kept2.state.size(), bare.state.size());
    for (size_t i = 0; i < kept2.state.size(); ++i)
        EXPECT_EQ(kept2.state[i], bare.state[i]) << i;
    EXPECT_EQ(kept2.st.rot_skipped, 0u);
}

TEST(RawGnssShadowV9, TheSkipFusesNothingOfTheEpochsThatTurnedAcrossTheNoseAndTheTailAfter)
{
    // Skipped from 1.5 rad/s across the nose and for 0.25 s after: a 2 rad/s
    // pitch drops the 11 epochs that saw it and the 2 within 0.25 s after the
    // last -- not a Doppler or a code of them fused, none held, their sigmas
    // the bare flight's -- and every other epoch fuses as the bare flight's
    // does. A 5 rad/s roll skips none, nor does 1.4 rad/s.
    RawGnssShadow::Config c = mosaicCfg();
    c.rot_skip_rate_rps = 1.5;
    c.rot_skip_s = 0.25;
    const TenFlight fl = turningCoast(2.0, 1, 0.0);
    const TenRun pitch = flyTen(c, fl), bare = flyTen(mosaicCfg(), fl);
    int skipped = 0;
    for (const ClosedEpoch& e : pitch.eps)
    {
        const bool want = e.t > 7.0 - 1e-6 && e.t < 8.2 + 1e-6;
        EXPECT_EQ(e.skipped, want) << e.t;
        EXPECT_FALSE(e.held) << e.t;
        const ClosedEpoch* b = epochAt(bare, e.t);
        ASSERT_NE(b, nullptr) << e.t;
        EXPECT_EQ(e.sigma_rr_min, b->sigma_rr_min) << e.t;
        if (want)
        {
            EXPECT_EQ(e.st.used_rr + e.st.used_pr, 0) << e.t;
            EXPECT_EQ(e.st.rejected_rr + e.st.rejected_pr, 0) << e.t;
            skipped++;
        }
        else if (e.t < 7.0)
        {
            EXPECT_EQ(e.st.used_rr, b->st.used_rr) << e.t;
            EXPECT_EQ(e.st.used_pr, b->st.used_pr) << e.t;
        }
    }
    EXPECT_EQ(skipped, 13);
    EXPECT_EQ(pitch.st.rot_skipped, 13u);
    EXPECT_EQ(pitch.st.att_held, 0u);
    EXPECT_EQ(pitch.st.rot_max_sigma_mps, 0.0f);
    // the epochs skipped are not dropped by a check: the vertical check and
    // the boost screen counted none of them
    EXPECT_EQ(pitch.st.vert_dropped, bare.st.vert_dropped);
    EXPECT_EQ(flyTen(c, turningCoast(5.0, 0, 0.0)).st.rot_skipped, 0u);
    EXPECT_EQ(flyTen(c, turningCoast(1.4, 1, 0.0)).st.rot_skipped, 0u);
    // only after the apogee call: called at 7.5 s, the epochs of the turn
    // handed over before it are fused, those after it and the tail skipped
    c.rot_after_apogee = true;
    TenFlight late = fl;
    late.apogee_t = 7.5;
    const TenRun after = flyTen(c, late);
    for (const ClosedEpoch& e : after.eps)
    {
        const bool want = e.t > 7.5 - 1e-6 && e.t < 8.2 + 1e-6;
        EXPECT_EQ(e.skipped, want) << e.t;
    }
    EXPECT_EQ(after.st.rot_skipped, 8u);
    // and with no call at all, none
    EXPECT_EQ(flyTen(c, fl).st.rot_skipped, 0u);
}

namespace
{

// The flight's own: act, the whole attitude GNSS's from launch (the test's
// mosaicCfg keeps the ground's gate). A 5 rad/s pitch out and back over
// 0.5 s from 5.0 s, in the boost (3 g along the nose turning with it). These
// flights launch with no magnetometer heading unless mag_on_pad, and were
// written before such a launch had its own prior and hand-off (2026-10-26):
// they keep the fallback prior and the config's own hand-off.
RawGnssShadow::Config actCfg()
{
    RawGnssShadow::Config c = mosaicCfg();
    c.heading_mode = RawGnssShadow::kHeadingAct;
    c.heading_pad_sigma_unset_deg = c.heading_pad_sigma_fallback_deg;
    c.heading_handoff_unset_deg = c.heading_handoff_deg;
    return c;
}

TenFlight boostTurn(double lever_m, double heading_err_deg)
{
    TenFlight fl;
    fl.turn_t = 5.0;
    fl.turn_s = 0.25;
    fl.turn_rps = 5.0;
    fl.turn_axis = 1;
    fl.lever_m = lever_m;
    fl.seed_heading_err_deg = heading_err_deg;
    fl.lean_deg = heading_err_deg != 0.0 ? 5.0 : 0.0;
    fl.t_end = 9.0;
    return fl;
}

// att_err_deg's sample at t (every 0.25 s from 0.25)
double errAt(const TenRun& r, double t)
{
    return r.att_err_deg.at(static_cast<size_t>(std::lround(t / 0.25)) - 1);
}

}  // namespace

TEST(RawGnssShadowV9, ASkippedTurnKeepsAnAntennaUpTheNoseOffTheAttitude)
{
    // The antenna 0.3 m up the nose swings 1.5 m/s against the IMU through
    // the turn, and the filter takes it at the IMU. The bare filter turns its
    // attitude by it: 0.5-0.8 deg off from 6.5 s on. Skipped from 1.5 rad/s
    // (0.1 s after), the turn's epochs fuse nothing and the IMU's attitude
    // stands: under 0.15 deg. Held instead, the velocity takes the swing
    // while the attitude is held, and the epochs after it turn the attitude
    // by what is left: about 2 deg and more (1.86 at the lowest on CI's Linux
    // runner, whose math library rounds differently from macOS's), ten times
    // the skip's.
    RawGnssShadow::Config skip = actCfg(), hold = actCfg();
    skip.rot_skip_rate_rps = 1.5;
    skip.rot_skip_s = 0.1;
    hold.att_hold_rate_rps = 1.5;
    hold.att_hold_s = 0.1;
    const TenFlight fl = boostTurn(0.3, 0.0);
    const TenRun bare = flyTen(actCfg(), fl), sk = flyTen(skip, fl), hd = flyTen(hold, fl);
    EXPECT_GT(sk.st.rot_skipped, 0u);
    for (double t = 6.5; t <= 8.5 + 1e-9; t += 0.25)
    {
        EXPECT_GT(errAt(bare, t), 0.4) << t;
        EXPECT_LT(errAt(sk, t), 0.15) << t;
        EXPECT_GT(errAt(hd, t), 1.5) << t;
    }
}

TEST(RawGnssShadowV9, ASkippedTurnLeavesItsHeadingErrorForTheEpochsAfter)
{
    // The pad seed's heading 10 deg off, the antenna on the IMU. The turn is
    // the flight's best look at the heading: the bare filter takes it there
    // (under 1 deg by 5.75 s). Skipped, nothing of the turn is fused (still
    // over 6.5 deg at 5.75 s), but the error the wrong heading built into the
    // velocity through the turn is there for the first epoch after it, which
    // reads it through the covariance the turn built: under 1 deg by 6.25 s,
    // 0.5 from 7.25 s. Held, the velocity took those epochs, and with them
    // the evidence: still over 5 deg to the end.
    RawGnssShadow::Config skip = actCfg(), hold = actCfg();
    skip.rot_skip_rate_rps = 1.5;
    skip.rot_skip_s = 0.1;
    hold.att_hold_rate_rps = 1.5;
    hold.att_hold_s = 0.1;
    const TenFlight fl = boostTurn(0.0, 10.0);
    const TenRun bare = flyTen(actCfg(), fl), sk = flyTen(skip, fl), hd = flyTen(hold, fl);
    EXPECT_GT(errAt(bare, 4.75), 6.5);
    EXPECT_LT(errAt(bare, 5.75), 1.0);
    EXPECT_GT(errAt(sk, 5.75), 6.5);
    EXPECT_LT(errAt(sk, 6.25), 1.0);
    for (double t = 7.25; t <= 8.75 + 1e-9; t += 0.25)
    {
        EXPECT_LT(errAt(sk, t), 0.5) << t;
        EXPECT_GT(errAt(hd, t), 5.0) << t;
    }
}

// ---- the antenna's offset, known (RawGnssShadow.h; applyAntennaOffset, setAntenna)

namespace
{

RawGnssShadow::Config knownCfg(double axial_m, uint8_t att_mode = gnss_raw::kLeverAttObserve)
{
    RawGnssShadow::Config c = actCfg();
    gnss_raw::applyMosaicFlightSafeguards(c);
    c.antenna_att_mode = att_mode;
    gnss_raw::AntennaOffset a;
    a.known = true;
    a.b_m[0] = axial_m;
    gnss_raw::applyAntennaOffset(a, c);
    return c;
}

RawGnssShadow::Config safeguardsCfg()
{
    RawGnssShadow::Config c = actCfg();
    gnss_raw::applyMosaicFlightSafeguards(c);
    return c;
}

}  // namespace

TEST(RawGnssShadowV9, AKnownAntennaTakesTheTurnsSwingOutOfTheAttitude)
{
    // The turning boost (boostTurn: 5 rad/s about body y for 0.25 s and back)
    // with the antenna 0.3 m up the nose. Unknown, the image's allowance
    // takes the swing's Dopplers at 1.65 m/s and what is left still turns the
    // attitude: over 0.45 deg from 6 s on (about 0.50 to 0.66 with the
    // spinning body's settings, over 0.5 before them). Known, the swing is
    // predicted and, with the launch's heading the magnetometer's, the
    // Dopplers allow only what the offset could be off by (0.05 m along the
    // nose and 0.02 across: 0.35 m/s); with no magnetometer, also what any
    // heading does to the predicted swing (leverUnsetM: twice the 0.3 m,
    // 3.35 m/s). Either: under 0.25 deg. The offset wrong by 5 cm either way
    // is still better than leaving it out.
    const TenFlight fl = boostTurn(0.3, 0.0);
    TenFlight fm = fl;
    fm.mag_on_pad = true;
    const TenRun unk = flyTen(safeguardsCfg(), fl), kn = flyTen(knownCfg(0.3), fl), km = flyTen(knownCfg(0.3), fm),
                 lo = flyTen(knownCfg(0.25), fl), hi = flyTen(knownCfg(0.35), fl);
    EXPECT_NEAR(unk.st.rot_max_sigma_mps, 1.65f, 0.02f);
    EXPECT_EQ(km.st.dv_hdg_unset, 0);
    EXPECT_NEAR(km.st.rot_max_sigma_mps, 0.35f, 0.01f);
    EXPECT_EQ(kn.st.dv_hdg_unset, 1);
    EXPECT_NEAR(kn.st.rot_max_sigma_mps, 3.35f, 0.02f);
    for (double t = 6.0; t <= 8.5 + 1e-9; t += 0.25)
    {
        EXPECT_GT(errAt(unk, t), 0.45) << t;
        EXPECT_LT(errAt(kn, t), 0.25) << t;
        EXPECT_LT(errAt(km, t), 0.25) << t;
        EXPECT_LT(errAt(lo, t), errAt(unk, t)) << t;
        EXPECT_LT(errAt(hi, t), errAt(unk, t)) << t;
    }
}

TEST(RawGnssShadowV9, AKnownAntennasAllowanceIsWhatItCouldBeOffByAtAnyRate)
{
    // Known, each Doppler takes rotLeverSigma at the offset's sigmas (0.05 m
    // along the nose, 0.02 across) from any rate: a 2 rad/s pitch 0.14 m/s, a
    // 1.5 rad/s one (under the unknown's 2 rad/s floor) 0.105, a 5 rad/s roll
    // the radial 0.10; the epochs before the turn are as the unknown's
    EXPECT_NEAR(flyTen(knownCfg(0.0), turningCoast(2.0, 1, 0.0)).st.rot_max_sigma_mps, 0.14f, 0.005f);
    EXPECT_NEAR(flyTen(knownCfg(0.0), turningCoast(1.5, 1, 0.0)).st.rot_max_sigma_mps, 0.105f, 0.005f);
    EXPECT_EQ(flyTen(safeguardsCfg(), turningCoast(1.5, 1, 0.0)).st.rot_max_sigma_mps, 0.0f);
    EXPECT_NEAR(flyTen(knownCfg(0.0), turningCoast(5.0, 0, 0.0)).st.rot_max_sigma_mps, 0.10f, 0.005f);
    // still, nothing moves: the known filter on a flight with no turn is the unknown one's
    const TenRun a = flyTen(safeguardsCfg(), 0.0, 8.0), b = flyTen(knownCfg(0.0), 0.0, 8.0);
    ASSERT_EQ(a.att_err_deg.size(), b.att_err_deg.size());
    for (size_t i = 0; i < a.att_err_deg.size(); ++i)
        EXPECT_NEAR(a.att_err_deg[i], b.att_err_deg[i], 1e-3) << i;
}

TEST(RawGnssShadowV9, TheLeverRollReadsARollAboutTheNoseFromTheChangeCheck)
{
    // A nose-up coast rolling at 5 rad/s about the nose for 0.5 s and back,
    // the antenna 0.1 m across it: the swing turns with the body, and the
    // pad's heading 10 deg off (about the down axis: -10 about the nose that
    // points up) turns the predicted swing with it. The change check's lever
    // roll reads the +10 deg correction within two of its 1-sigmas (10.6
    // +- 2.0 deg here); unknown, or with the terms fused, it reads nothing
    // (no epochs fitted for it).
    TenFlight fl;
    fl.turn_t = 7.0;
    fl.turn_s = 0.5;
    fl.turn_rps = 5.0;
    fl.turn_axis = 0;
    fl.lever_y_m = 0.1;
    fl.seed_heading_err_deg = 10.0;
    RawGnssShadow::Config c = safeguardsCfg();
    // settled tracking, as minutes on a pad leave it: every satellite of this
    // synthetic sky starts locked at the flight's start, and the builder's
    // relock term (3 m/s on each Doppler, decaying over 10 s) would leave the
    // change check's fit a 1 m/s 1-sigma through the roll
    c.meas.rr_relock_sigma = 0.0;
    gnss_raw::AntennaOffset a;
    a.known = true;
    a.b_m[1] = 0.1;
    gnss_raw::applyAntennaOffset(a, c);
    const TenRun r = flyTen(c, fl);
    ASSERT_GT(r.st.lever_roll_epochs, 0u);
    ASSERT_GT(r.st.lever_roll_info, 0.0);
    const double roll = r.st.lever_roll_hy / r.st.lever_roll_info * 57.29577951308232;
    const double sig = 57.29577951308232 / std::sqrt(r.st.lever_roll_info);
    EXPECT_LT(sig, 3.0);
    EXPECT_NEAR(roll, 10.0, std::max(2.0 * sig, 1.0));
    EXPECT_EQ(flyTen(safeguardsCfg(), fl).st.lever_roll_epochs, 0u);
    RawGnssShadow::Config cf = c;
    cf.antenna_att_mode = gnss_raw::kLeverAttFuse;
    cf.ekf.lever_att_mode = gnss_raw::kLeverAttFuse;
    EXPECT_EQ(flyTen(cf, fl).st.lever_roll_epochs, 0u);
}

TEST(RawGnssShadowV9, AnAntennaSetInFlightWaitsForTheGround)
{
    // setAntenna on the ground: taken at once; in flight: held until the
    // first ground tick, then taken; unknown again: the bounds, the offset
    // gone from the filter
    RawGnssShadow sh(safeguardsCfg());
    const double b[3] = {0.2, 0.0, -0.01};
    ShadowImu s{};
    s.acc[2] = float(-kG);
    s.time_us = static_cast<uint32_t>(kT0);
    s.flags = gnss_raw::kShadowLevel;
    sh.onImu(s, kT0);
    sh.setAntenna(true, b);
    EXPECT_TRUE(sh.filter().params().lever_on);
    EXPECT_EQ(sh.filter().params().lever_b_m[0], 0.2);
    EXPECT_EQ(sh.filter().params().lever_att_mode, gnss_raw::kLeverAttObserve);
    EXPECT_EQ(sh.stats().antenna_sets, 1u);
    s.time_us += 10000;
    s.flags = gnss_raw::kShadowInFlight;
    sh.onImu(s, kT0 + 10000);
    sh.setAntenna(false, b);
    EXPECT_EQ(sh.stats().antenna_pending, 1);
    EXPECT_TRUE(sh.filter().params().lever_on);          // still the flight's
    s.time_us += 10000;
    sh.onImu(s, kT0 + 20000);
    EXPECT_TRUE(sh.filter().params().lever_on);
    s.time_us += 10000;
    s.flags = gnss_raw::kShadowLevel;                     // landed, on the ground
    sh.onImu(s, kT0 + 30000);
    EXPECT_EQ(sh.stats().antenna_pending, 0);
    EXPECT_FALSE(sh.filter().params().lever_on);
    for (int i = 0; i < 3; ++i)
        EXPECT_EQ(sh.filter().params().lever_b_m[i], 0.0);
    EXPECT_EQ(sh.filter().params().lever_att_mode, gnss_raw::kLeverAttOff);
    EXPECT_EQ(sh.stats().antenna_sets, 2u);
}

namespace
{

// The image's flight with no magnetometer heading at launch (the V9's), the
// offset known this far up the nose (0: unknown), and settled tracking, as
// minutes on a pad leave it (TheLeverRollReadsARollAboutTheNoseFromTheChangeCheck:
// the builder's relock term would leave the change check's fit a 1 m/s 1-sigma)
RawGnssShadow::Config unsetCfg(double axial_m)
{
    RawGnssShadow::Config c = safeguardsCfg();
    c.meas.rr_relock_sigma = 0.0;
    if (axial_m > 0.0)
    {
        gnss_raw::AntennaOffset a;
        a.known = true;
        a.b_m[0] = axial_m;
        gnss_raw::applyAntennaOffset(a, c);
    }
    return c;
}

// A swing from `from` for secs: the rate about body y a 2 Hz sine of rps at its peak (0.6 rad/s, a coast
// weathercocking: up to 0.75 rad/s in an epoch's 0.1 s; 5, a drogue-like swing); the antenna r up the nose, the
// pad's heading hdg off
TenFlight swing(double rps, double from, double secs, double r, double hdg)
{
    TenFlight fl;
    fl.wc_t = from;
    fl.wc_s = secs;
    fl.wc_rps = rps;
    fl.lever_m = r;
    fl.seed_heading_err_deg = hdg;
    return fl;
}

// the change check's failed epochs closed from t0 up to t1
uint32_t dvFailsBetween(const TenRun& r, double t0, double t1)
{
    uint32_t a = 0, b = 0;
    for (const ClosedEpoch& c : r.eps)
    {
        if (c.t < t0 - 1e-6)
            a = c.dv_epoch_fails;
        if (c.t < t1 + 1e-6)
            b = c.dv_epoch_fails;
    }
    return b - a;
}

}  // namespace

TEST(RawGnssShadowV9, AKnownAntennaWithTheHeadingUnsetPassesTheChangeCheckThroughASwing)
{
    // Known, each end's Dopplers are predicted at the antenna at the
    // filter's attitude, and with no magnetometer heading the filter's
    // heading is any distance off: the predicted lever's change turns with
    // it. 01b9a163 allowed it only the offset's 5 cm of doubt: the
    // weathercock failed 2 (0.3 m, 60 deg) to 8 (0.5 m, 180 deg) epochs and
    // the swing 2 to 6 (chi-square up to 505), each latching the Dopplers
    // out. Now the prediction comes back out of the change and the whole
    // offset is allowed (DopplerChange.h knownLeverArm): none fails, as none
    // does unknown.
    for (double hdg : {60.0, 180.0})
        for (double r : {0.3, 0.5})
        {
            SCOPED_TRACE(testing::Message() << r << " m, " << hdg << " deg");
            const TenRun wc = flyTen(unsetCfg(r), swing(0.6, 6.5, 1.0, r, hdg));
            const TenRun dr = flyTen(unsetCfg(r), swing(5.0, 7.0, 0.5, r, hdg));
            EXPECT_EQ(wc.st.dv_hdg_unset, 1);
            EXPECT_GT(wc.st.dv_checks, 50u);
            EXPECT_EQ(wc.st.dv_epoch_fails, 0u);
            EXPECT_EQ(wc.st.dv_latches, 0u);
            EXPECT_GT(dr.st.dv_checks, 50u);
            EXPECT_EQ(dr.st.dv_epoch_fails, 0u);
            EXPECT_EQ(dr.st.dv_latches, 0u);
            if (hdg == 60.0)
            {
                EXPECT_EQ(flyTen(unsetCfg(0.0), swing(0.6, 6.5, 1.0, r, hdg)).st.dv_epoch_fails, 0u);
                EXPECT_EQ(flyTen(unsetCfg(0.0), swing(5.0, 7.0, 0.5, r, hdg)).st.dv_epoch_fails, 0u);
            }
        }
}

TEST(RawGnssShadowV9, AKnownAntennaWithTheHeadingUnsetStillCatchesAStaleOrRampingReceiver)
{
    // The same allowance does not hide a receiver gone wrong through the
    // swing. The whole receiver stale through the boost and closing on the
    // truth from burnout (tau 1 s) while the coast weathercocks from burnout:
    // every epoch of the weathercock and the half second after fails, as
    // unknown. Every Doppler ramping 2 m/s a second down from 6.6 s, held at
    // 2 m/s, through the weathercock from 6.5 s: unknown fails 6 (0.3 m) and
    // 7 (0.5 m) epochs from 6.5 to 8 s, known 6 and 4 (the larger offset's
    // allowance the wider). The 0.5 m case's count sits near the 4-sigma
    // line: CI's Linux runner, whose math library rounds differently from
    // macOS's, fails 2 of them; what the test holds it to is that the ramp
    // fails at least one epoch and its Dopplers are held.
    struct Case
    {
        double r, hdg;
        uint32_t ramp_min;
    };
    for (const Case& c : {Case{0.3, 60.0, 5u}, Case{0.5, 180.0, 1u}})
    {
        SCOPED_TRACE(testing::Message() << c.r << " m, " << c.hdg << " deg");
        TenFlight st = swing(0.6, kBurnout, 1.0, c.r, c.hdg);
        st.stale_tau = 1.0;
        const TenRun kn = flyTen(unsetCfg(c.r), st), unk = flyTen(unsetCfg(0.0), st);
        EXPECT_GE(dvFailsBetween(kn, kBurnout, kBurnout + 1.5), 15u);
        EXPECT_EQ(dvFailsBetween(kn, kBurnout, kBurnout + 1.5), dvFailsBetween(unk, kBurnout, kBurnout + 1.5));
        EXPECT_GT(kn.st.dv_held_epochs, 0u);
        TenFlight rp = swing(0.6, 6.5, 1.0, c.r, c.hdg);
        rp.coh_from = 6.6;
        rp.coh_rate = 2.0;
        rp.coh_max = 2.0;
        const TenRun kr = flyTen(unsetCfg(c.r), rp), ur = flyTen(unsetCfg(0.0), rp);
        EXPECT_GE(dvFailsBetween(kr, 6.5, 8.0), c.ramp_min);
        EXPECT_GE(dvFailsBetween(ur, 6.5, 8.0), 5u);
        EXPECT_GT(kr.st.dv_held_epochs, 0u);
    }
}

namespace
{

// the epoch that let the change check's first latch go (none: nullptr)
const ClosedEpoch* firstRelease(const TenRun& r)
{
    for (const ClosedEpoch& c : r.eps)
        if (c.dv_latch_releases > 0)
            return &c;
    return nullptr;
}

}  // namespace

TEST(RawGnssShadowV9, AKnownAntennaWithTheHeadingUnsetLetsTheLatchGoAtUnknownsLevel)
{
    // The latch's level test fits the epoch's Dopplers as they stand, each
    // less its prediction: known, at the antenna, the lever's velocity at the
    // filter's attitude in it. With no magnetometer heading that attitude's
    // heading is any distance off, and a swing's predicted lever velocity
    // turns with it: 180 deg off, the level is the swing across the nose
    // twice over where unknown's is once. Now the prediction goes back on the
    // fitted offset, as on the epoch test's change (knownLeverArm), and the
    // latch lets go where unknown's does. An epoch 40 m/s off vertically
    // latches at 7.0 s (its change in, and out at 7.1); the coast pitches at
    // 3 rad/s from 7.15 s, 0.3 s out and back, the antenna 0.3 m up the nose
    // (0.9 m/s), the pad's heading 180 deg off. Unknown lets go at 7.2 s, its
    // level's chi-square 11.2 against dv_latch_k's 25; known now at the same
    // epoch and level. Before (a0ca93cf), known's was 42.6 there and 29.9 at
    // 7.3 s, and the latch held to 7.4.
    TenFlight fl;
    fl.bad_vertical_t = 7.0;
    fl.turn_t = 7.15;
    fl.turn_s = 0.3;
    fl.turn_rps = 3.0;
    fl.turn_axis = 1;
    fl.lever_m = 0.3;
    fl.seed_heading_err_deg = 180.0;
    fl.t_end = 9.0;
    const TenRun unk = flyTen(unsetCfg(0.0), fl), kn = flyTen(unsetCfg(0.3), fl);
    EXPECT_EQ(kn.st.dv_hdg_unset, 1);
    ASSERT_EQ(unk.st.dv_latches, 1u);
    ASSERT_EQ(kn.st.dv_latches, 1u);
    const ClosedEpoch* ru = firstRelease(unk);
    const ClosedEpoch* rk = firstRelease(kn);
    ASSERT_NE(ru, nullptr);
    ASSERT_NE(rk, nullptr);
    EXPECT_GT(ru->t, fl.turn_t);                          // let go in the swing
    EXPECT_LT(ru->t, fl.turn_t + 2.0 * fl.turn_s);
    EXPECT_GT(ru->dv_level_chi2, 4.0f);                   //   the swing in its level
    EXPECT_NEAR(rk->t, ru->t, 1e-6);
    EXPECT_EQ(kn.st.dv_latch_epochs, unk.st.dv_latch_epochs);
    EXPECT_NEAR(rk->dv_level_chi2, ru->dv_level_chi2, 0.02f * ru->dv_level_chi2);
}

TEST(RawGnssShadowV9, AKnownAntennaWithTheHeadingUnsetAllowsTheSwingsDopplersWhatAnyHeadingDoes)
{
    // The latch lets go at unknown's level (above), so the swing's Dopplers
    // reach the update, each still predicted with the lever's velocity at the
    // filter's heading: 180 deg off, the swing across the nose the wrong way,
    // 3 m/s off at 5 rad/s with 0.3 m, where the offset's doubt allowed 0.35.
    // An epoch 40 m/s off vertically latches at 7.0 s; the coast pitches at
    // 5 rad/s from 7.1 s, 0.3 s out and back. Before (697808d3) the update
    // took the swing's Dopplers and the velocity went 5.5 m/s off by 12 s,
    // and the latch, latched again, held the good Dopplers out against it (49
    // epochs); unknown's stayed within 0.31 m/s (8 epochs). Now each Doppler
    // also allows what any heading does to the predicted swing (leverUnsetM:
    // twice the offset along the nose, 3.35 m/s here): 0.16 m/s and 8 epochs.
    // That allowance starts where the bounds do (rot_lever_min_rps, 2 rad/s
    // across the nose): a 1.5 rad/s turn takes the doubt's 0.105 m/s only, a
    // 2.5 rad/s one 0.175 and 1.5 for the heading.
    TenFlight fl;
    fl.bad_vertical_t = 7.0;
    fl.turn_t = 7.1;
    fl.turn_s = 0.3;
    fl.turn_rps = 5.0;
    fl.turn_axis = 1;
    fl.lever_m = 0.3;
    fl.seed_heading_err_deg = 180.0;
    fl.t_end = 12.0;
    const TenRun unk = flyTen(unsetCfg(0.0), fl), kn = flyTen(unsetCfg(0.3), fl);
    EXPECT_EQ(kn.st.dv_hdg_unset, 1);
    EXPECT_NEAR(kn.st.rot_max_sigma_mps, 3.35f, 0.02f);
    EXPECT_GT(unk.st.dv_latch_epochs, 0u);
    EXPECT_LE(kn.vel_err_max_mps, unk.vel_err_max_mps + 0.5);
    EXPECT_LE(kn.st.dv_latch_epochs, unk.st.dv_latch_epochs + 3u);
    EXPECT_NEAR(flyTen(unsetCfg(0.3), turningCoast(1.5, 1, 0.3)).st.rot_max_sigma_mps, 0.105f, 0.005f);
    EXPECT_NEAR(flyTen(unsetCfg(0.3), turningCoast(2.5, 1, 0.3)).st.rot_max_sigma_mps, 1.675f, 0.01f);
}

TEST(RawGnssShadowV9, AKnownAntennaWithTheMagnetometersHeadingKeepsTheLatchsLevelAsFitted)
{
    // With the launch's heading the magnetometer's, the lever's velocity
    // predicted at the filter's attitude is the antenna's own: the latch's
    // level is the fit's offset as it stands (no v_lev put back), and the
    // Dopplers allow only the offset's doubt (no leverUnsetM). So where a
    // swing holds unknown's latch on (its level the swing's 1.5 m/s), known's
    // lets go. An epoch 40 m/s off vertically latches at 7.0 s; the coast
    // pitches at 5 rad/s from 7.15 s, 0.3 s out and back, the antenna 0.3 m
    // up the nose, a calibrated magnetometer on the pad. Unknown's level is
    // 26.7 to 31.1 against dv_latch_k's 25 to 7.5 s, and it lets go at 7.6
    // (19.5); known at 7.3 (0.1). With v_lev put back here too, known held to
    // 7.6 at unknown's level.
    TenFlight fl;
    fl.bad_vertical_t = 7.0;
    fl.turn_t = 7.15;
    fl.turn_s = 0.3;
    fl.turn_rps = 5.0;
    fl.turn_axis = 1;
    fl.lever_m = 0.3;
    fl.mag_on_pad = true;
    fl.t_end = 9.0;
    const TenRun unk = flyTen(unsetCfg(0.0), fl), kn = flyTen(unsetCfg(0.3), fl);
    EXPECT_EQ(unk.st.dv_hdg_unset, 0);
    EXPECT_EQ(kn.st.dv_hdg_unset, 0);
    EXPECT_NEAR(kn.st.rot_max_sigma_mps, 0.35f, 0.01f);
    const ClosedEpoch* ru = firstRelease(unk);
    const ClosedEpoch* rk = firstRelease(kn);
    ASSERT_NE(ru, nullptr);
    ASSERT_NE(rk, nullptr);
    EXPECT_GT(ru->dv_level_chi2, 4.0f);
    EXPECT_LT(rk->dv_level_chi2, 1.0f);
    EXPECT_LT(rk->t, ru->t - 0.15);
}

// ---- the change check's lever allowance on the nose's own turn (dv_lever_mode 1; the high-roll port)

namespace
{

// The Goblin's antenna from its IMU (the V9 holder as measured; body FRD, m): 47 mm up the nose and 12.6 mm across it
constexpr double kGoblinAnt[3] = {0.04696, -0.01138, 0.00548};

// A flight rolling about the nose at rev_s from half a second into the boost to the end (the rate a step, then
// steady), the truth's antenna at r (body FRD, m), the pad's magnetometer calibrated or not
TenFlight rolling(double rev_s, const double r[3], bool mag)
{
    TenFlight fl;
    fl.turn_t = kIgn + 0.5;
    fl.turn_s = 100.0;                   // never back
    fl.turn_rps = rev_s * 2.0 * kPi;
    fl.turn_axis = 0;
    fl.lever_m = r[0];
    fl.lever_y_m = r[1];
    fl.lever_z_m = r[2];
    fl.mag_on_pad = mag;
    return fl;
}

// The image's safeguards with the exact attitude transition (a roll's tilt variance turned, not stretched), the
// change check's lever mode, settled tracking (unsetCfg's), and the offset known at r (nullptr: unknown). The
// wind-up stays off: the synthetic sky's Dopplers carry none.
RawGnssShadow::Config leverCfg(uint8_t mode, const double* r)
{
    RawGnssShadow::Config c = safeguardsCfg();
    c.meas.rr_relock_sigma = 0.0;
    c.ekf.att_exact = true;
    c.windup_k = 0.0;
    c.dv_lever_mode = mode;
    c.dv_lever_radial_m = 0.0;
    if (r != nullptr)
    {
        gnss_raw::AntennaOffset a;
        a.known = true;
        for (int i = 0; i < 3; ++i)
            a.b_m[i] = r[i];
        gnss_raw::applyAntennaOffset(a, c);
    }
    return c;
}

// every Doppler ramping 2 m/s a second down from 6.6 s, held at 4 m/s (sim_events.py's slow12)
TenFlight ramped(TenFlight fl)
{
    fl.coh_from = 6.6;
    fl.coh_rate = 2.0;
    fl.coh_max = 4.0;
    return fl;
}

}  // namespace

TEST(RawGnssShadowV9, TheNoseLeverTakesNothingFromARollAboutTheNose)
{
    // The Goblin's antenna known, the coast rolling at a steady 5 rev/s about a nose that points straight up: the
    // nose does not turn, so on its own turn (mode 1) the epoch's lever allowance is the arm times nothing (under
    // 0.01 m/s), where on the whole rate (mode 0) it is the arm times |w|^2 dt: 5.8 m/s with the magnetometer's
    // heading (0.05 + chord(10 deg) x 0.0486 m) and 9.7 with the heading unset (0.0486 + 0.05 m). With the heading
    // the magnetometer's, the roll's lever velocity is predicted and no epoch fails on the nose's turn either.
    for (bool mag : {true, false})
    {
        SCOPED_TRACE(mag ? "the magnetometer's heading" : "the heading unset");
        const TenRun nose = flyTen(leverCfg(1, kGoblinAnt), rolling(5.0, kGoblinAnt, mag));
        const TenRun whole = flyTen(leverCfg(0, kGoblinAnt), rolling(5.0, kGoblinAnt, mag));
        EXPECT_EQ(nose.st.dv_hdg_unset, mag ? 0 : 1);
        EXPECT_GT(nose.st.dv_checks, 50u);
        EXPECT_LT(nose.st.dv_last_lever_mps, 0.01f);
        EXPECT_NEAR(whole.st.dv_last_lever_mps, mag ? 5.77f : 9.73f, 0.1f);
        if (mag)
        {
            EXPECT_EQ(nose.st.dv_epoch_fails, 0u);
            EXPECT_EQ(nose.st.dv_held_epochs, 0u);
        }
    }
}

TEST(RawGnssShadowV9, TheNoseLeverCatchesASlowRampThatTheWholeRateMisses)
{
    // Every Doppler ramping 2 m/s a second (slow12) through a coast rolling at 1 rev/s, the Goblin's antenna in the
    // truth: on the whole rate (mode 0) the lever allowance takes the roll (0.39 m/s an epoch with the offset known
    // and the heading unset, 1.2 unknown) and no epoch fails; on the nose's own turn (mode 1) the ramp fails epoch
    // after epoch (20 or so from 6.6 to 9 s) and its Dopplers are held -- the offset unknown, or known with the
    // heading unset or the magnetometer's. Without the ramp none fails either way: 12.6 mm across the nose at
    // 1 rev/s changes an epoch's Dopplers by 0.05 m/s. At 5 rev/s with the magnetometer's heading (the one case
    // whose own roll passes there: below), the same.
    struct Case
    {
        const double* r;
        bool mag;
        const char* what;
    };
    for (const Case& c : {Case{nullptr, false, "unknown"}, Case{kGoblinAnt, false, "known, the heading unset"},
                          Case{kGoblinAnt, true, "known, the magnetometer's heading"}})
    {
        SCOPED_TRACE(c.what);
        const TenFlight fl = rolling(1.0, kGoblinAnt, c.mag);
        const TenRun nose = flyTen(leverCfg(1, c.r), ramped(fl)), whole = flyTen(leverCfg(0, c.r), ramped(fl));
        EXPECT_GE(dvFailsBetween(nose, 6.6, 9.0), 10u);
        EXPECT_GT(nose.st.dv_held_epochs, 10u);
        EXPECT_EQ(whole.st.dv_epoch_fails, 0u);
        EXPECT_EQ(flyTen(leverCfg(1, c.r), fl).st.dv_epoch_fails, 0u);
        EXPECT_EQ(flyTen(leverCfg(0, c.r), fl).st.dv_epoch_fails, 0u);
    }
    const TenFlight fast = rolling(5.0, kGoblinAnt, true);
    EXPECT_GE(dvFailsBetween(flyTen(leverCfg(1, kGoblinAnt), ramped(fast)), 6.6, 9.0), 10u);
    EXPECT_EQ(flyTen(leverCfg(0, kGoblinAnt), ramped(fast)).st.dv_epoch_fails, 0u);
}

TEST(RawGnssShadowV9, AnAntennaOffTheRollAxisUnpredictedHoldsTheDopplersAtFiveRevS)
{
    // With no radial bound (dv_lever_radial_m 0) the nose's own turn allows nothing for an antenna across the roll
    // axis that the filter does not predict: the offset unknown, or known with the heading unset (the prediction
    // taken back out of the change). At 5 rev/s the Goblin's 12.6 mm turns an epoch's Dopplers by 0.79 m/s, and the
    // coast's epochs fail and hold their Dopplers -- towards the unaided filter, the documented price. On the axis
    // nothing fails.
    const TenRun unk = flyTen(leverCfg(1, nullptr), rolling(5.0, kGoblinAnt, false));
    const TenRun unset = flyTen(leverCfg(1, kGoblinAnt), rolling(5.0, kGoblinAnt, false));
    EXPECT_GE(unk.st.dv_epoch_fails, 20u);
    EXPECT_GT(unk.st.dv_held_epochs, 20u);
    EXPECT_GE(unset.st.dv_epoch_fails, 20u);
    EXPECT_GT(unset.st.dv_held_epochs, 20u);
    const double axis[3] = {kGoblinAnt[0], 0.0, 0.0};
    EXPECT_EQ(flyTen(leverCfg(1, nullptr), rolling(5.0, axis, false)).st.dv_epoch_fails, 0u);
}

namespace
{

// rolling() at rev_s with a pitch-over too: from 6.5 s the nose turns at 1 rad/s about body y as it stood before
// the roll (fixed in NED) for 1 s, then back as fast for as long
TenFlight pitchingOver(double rev_s, const double r[3], bool mag)
{
    TenFlight fl = rolling(rev_s, r, mag);
    fl.roll_t = fl.turn_t;
    fl.roll_rps = fl.turn_rps;
    fl.turn_axis = 1;
    fl.turn_t = 6.5;
    fl.turn_s = 1.0;
    fl.turn_rps = 1.0;
    return fl;
}

}  // namespace

TEST(RawGnssShadowV9, UnderARollTheNoseLeverIsTheCasesArmTimesThePitchOversTurn)
{
    // A pitch-over at a steady 1 rad/s under a 1 rev/s roll (pitchingOver), the Goblin's antenna in the truth, on the
    // nose's own turn (mode 1). Between two checks 0.1 s apart the nose's rate n' turns by 2 sin(0.05) =
    // 0.09996 rad/s; each epoch is checked 55 ms after its tag (handed over at 50 ms, the next step), so the pad adds
    // 2 x 0.055 s x 1 rad/s^2: the epoch's allowance is the arm times 0.20996 rad/s, the arm the offset's case's
    // (knownLeverArm) -- unknown, dv_lever_allow_m 0.3 m; known with the magnetometer's heading, 0.05 m + chord(10
    // deg) x 0.0486 m = 0.0585 m; known with the heading unset, 0.0486 + 0.05 = 0.0986 m. Taken in the body, or
    // turned the wrong way, n' would turn with the roll as well. The window's allowance is capped at the arm times
    // the rate across the nose now and its largest: 2 x arm x 1 rad/s. Once the turn is over, nothing is left
    // (under 0.01 m/s): each check starts the nose's change and its |n''| over. No epoch fails in any case.
    // Within 5 %: |n''| is the peak of the nose's turn tracked in float over a check's steps, and the libm under it
    // moves that peak -- this Mac's to within 2 %, CI's Linux runner's up to 3.1 % on two of the 18 epochs. A wrong
    // arm, a body-frame or conjugate n', a pad on half the age or on the age without the pair lag all miss by 19 %
    // or more.
    const double rn = std::sqrt(kGoblinAnt[0] * kGoblinAnt[0] + kGoblinAnt[1] * kGoblinAnt[1]
                                + kGoblinAnt[2] * kGoblinAnt[2]);
    const double turn = 2.0 * std::sin(0.05) + 2.0 * 0.055;
    struct Case
    {
        const double* r;
        bool mag;
        double arm;
        const char* what;
    };
    for (const Case& c : {Case{nullptr, false, 0.3, "unknown"},
                          Case{kGoblinAnt, true, 0.05 + 2.0 * std::sin(5.0 * kPi / 180.0) * rn,
                               "known, the magnetometer's heading"},
                          Case{kGoblinAnt, false, rn + 0.05, "known, the heading unset"}})
    {
        SCOPED_TRACE(c.what);
        const TenRun run = flyTen(leverCfg(1, c.r), pitchingOver(1.0, kGoblinAnt, c.mag));
        EXPECT_EQ(run.st.dv_hdg_unset, c.r != nullptr && c.mag ? 0 : 1);
        EXPECT_EQ(run.st.dv_epoch_fails, 0u);
        int steady = 0, after = 0;
        for (const ClosedEpoch& e : run.eps)
        {
            // the turn steady across the whole interval: out 6.6-7.4 s, back 7.6-8.4 s
            if ((e.t > 6.55 && e.t < 7.45) || (e.t > 7.55 && e.t < 8.45))
            {
                EXPECT_NEAR(e.dv_lever_mps, c.arm * turn, 0.05 * c.arm * turn) << e.t;
                EXPECT_NEAR(e.dv_lever_win_mps, 2.0 * c.arm, 0.01 * c.arm) << e.t;
                steady++;
            }
            else if (e.t > 8.55)
            {
                EXPECT_LT(e.dv_lever_mps, 0.01f) << e.t;
                after++;
            }
        }
        EXPECT_EQ(steady, 18);
        EXPECT_EQ(after, 14);
    }
}

TEST(RawGnssShadowV9, InPairsTheNoseLeversPadReachesBackToThePairsMiddle)
{
    // The pitch-over above with every epoch handed over in pairs (the bench's Doppler pairs): each epoch's Dopplers
    // are the antenna's 25 ms before its tag, and the filter takes them there (rr_pair_lag_s). The pad covers what
    // the nose's turn adds between the check's step and the Dopplers' instant, now 55 + 25 ms back: the allowance
    // is the arm times 2 sin(0.05) + 2 x 0.080 x 1 rad/s = 0.25996 rad/s (the epoch's own age alone, 0.055 s,
    // would leave it at 0.20996: 19 % short). The window's is unchanged, and no epoch fails in any case. Within 5 %,
    // as the test above says why.
    const double rn = std::sqrt(kGoblinAnt[0] * kGoblinAnt[0] + kGoblinAnt[1] * kGoblinAnt[1]
                                + kGoblinAnt[2] * kGoblinAnt[2]);
    const double turn = 2.0 * std::sin(0.05) + 2.0 * (0.055 + 0.025);
    struct Case
    {
        const double* r;
        bool mag;
        double arm;
        const char* what;
    };
    for (const Case& c : {Case{nullptr, false, 0.3, "unknown"},
                          Case{kGoblinAnt, true, 0.05 + 2.0 * std::sin(5.0 * kPi / 180.0) * rn,
                               "known, the magnetometer's heading"},
                          Case{kGoblinAnt, false, rn + 0.05, "known, the heading unset"}})
    {
        SCOPED_TRACE(c.what);
        RawGnssShadow::Config cfg = leverCfg(1, c.r);
        cfg.ekf.rr_pair_lag_s = 0.025;    // the profile's rr_pair_avg, applied
        TenFlight fl = pitchingOver(1.0, kGoblinAnt, c.mag);
        fl.pair_lag_ms = 25;
        const TenRun run = flyTen(cfg, fl);
        EXPECT_EQ(run.st.dv_epoch_fails, 0u);
        int steady = 0;
        for (const ClosedEpoch& e : run.eps)
            if ((e.t > 6.55 && e.t < 7.45) || (e.t > 7.55 && e.t < 8.45))
            {
                EXPECT_NEAR(e.dv_lever_mps, c.arm * turn, 0.05 * c.arm * turn) << e.t;
                EXPECT_NEAR(e.dv_lever_win_mps, 2.0 * c.arm, 0.01 * c.arm) << e.t;
                steady++;
            }
        EXPECT_EQ(steady, 18);
    }
}

TEST(RawGnssShadowV9, TheNoseLeversWindowIsCappedAtTheArmTimesTheRateAcrossTheNose)
{
    // A coast rolling at 1 rev/s and weathercocking (0.3 rad/s at 2 Hz from 6 s, about body y as it stood before the
    // roll), the offset unknown, the Goblin's antenna in the truth, on the nose's own turn. The nose's rate changes
    // fast: each epoch's lever allowance is 0.13-0.23 m/s, which summed over the 1 s window would allow about
    // 1.8 m/s. But the antenna's velocity change over the window telescopes, so the window's allowance is capped at
    // the arm times the rate across the nose now and its largest: 0.3 m x 0.6 rad/s = 0.18 m/s (on the whole rate,
    // mode 0, 3.8 m/s with the roll in it). So slow12 through it fails on the window's sum alone, from about 7.2 s,
    // where the epoch's own test passes it and the whole rate fails nothing; without the ramp nothing fails either
    // way.
    TenFlight fl = pitchingOver(1.0, kGoblinAnt, false);
    fl.turn_t = -1.0;                    // no pitch-over
    fl.wc_t = 6.0;
    fl.wc_s = 4.0;
    fl.wc_rps = 0.3;
    const TenRun quiet = flyTen(leverCfg(1, nullptr), fl), whole = flyTen(leverCfg(0, nullptr), fl);
    EXPECT_EQ(quiet.st.dv_epoch_fails, 0u);
    EXPECT_EQ(whole.st.dv_epoch_fails, 0u);
    double sum = 0.0;
    int n = 0;
    for (const ClosedEpoch& e : quiet.eps)
        if (e.t > 6.45)
        {
            EXPECT_LE(e.dv_lever_win_mps, 0.3 * 0.6 + 1e-3) << e.t;
            sum += e.dv_lever_mps;
            n++;
        }
    ASSERT_EQ(n, 35);
    EXPECT_GT(sum / n, 0.15);
    for (const ClosedEpoch& e : whole.eps)
        if (e.t > 6.45)
            EXPECT_GT(e.dv_lever_win_mps, 3.5f) << e.t;
    const TenRun nose = flyTen(leverCfg(1, nullptr), ramped(fl));
    EXPECT_GE(nose.st.dv_epoch_win_fails, 10u);
    EXPECT_EQ(nose.st.dv_epoch_fails, nose.st.dv_epoch_win_fails);
    EXPECT_GT(nose.st.dv_held_epochs, 10u);
    EXPECT_EQ(flyTen(leverCfg(0, nullptr), ramped(fl)).st.dv_epoch_fails, 0u);
}

// ---- the satellite cap's governor (CapGovernor.h) in the filter's loop

namespace
{

// The pad, and a launch if asked (3 g along the nose of an upright body for
// 2 s, then a weightless coast), at 10 Hz through a GNSS task's stage as the
// flight computer runs them (external_build): each epoch offered to the stage
// at the filter's navigation state when it is handed over, delay_ms(t) after
// its tag, and the built epoch handed to the filter; lose(t): an epoch never
// handed over (the driver's queue full). The sky is the golden fixture's 15,
// so the governor's cap is set to bind on it: 8 to 12.
struct GovFlight
{
    double t_end = 24.0;
    double launch_t = -1.0;
    std::function<double(double)> delay_ms = [](double) { return 50.0; };
    std::function<bool(double)> lose = [](double) { return false; };
};

struct GovRun
{
    std::vector<double> t;        // each epoch handed over: its tag (s of the flight)
    std::vector<int> nav_cap;     //   the cap the navigation state carried then
    std::vector<int> built;       //   the satellites the stage built of it
    std::vector<uint64_t> state;  // stateHash every step
    std::vector<uint64_t> state2; //   and of the second filter, where there is one
    gnss_raw::CapGovernor::Stats gov;
};

RawGnssShadow::Config govCfg(int floor = 8, int ceiling = 12)
{
    RawGnssShadow::Config c = mosaicCfg();
    c.external_build = true;
    c.sat_cap = floor;
    c.sat_cap_max = ceiling;
    return c;
}

// Each epoch handed over goes to every shadow in shs, built once by the
// stage at shs[0]'s navigation state (the others see the same epochs)
GovRun flyGov(const std::vector<RawGnssShadow*>& shs, const GovFlight& fl)
{
    RawGnssShadow& sh = *shs[0];
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
    gnss_raw::RawGnssMeasStage stage(RawGnssShadow::stageConfig(sh.config()));
    for (int i = 0; i < kGoldenRmEphN; ++i)
        stage.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
    SynthSky sky(sh.config().meas);
    const Mat3 Ct = leaningBody(0.0, 70.0);
    ShadowSeed seed = padSeed(kT0, lat, lon, h);
    quatOf(Ct, seed.quat);
    for (RawGnssShadow* s : shs)
        s->onSeed(seed);
    const double tow0 = kGoldenRmTowA, dt = 0.005;
    const double burnout_t = fl.launch_t + 2.0;
    double p_ned[3] = {}, v_ned[3] = {};
    struct Due
    {
        TimedRawEpoch e;
        int64_t hand_us;
        double t;
    };
    std::vector<Due> due;
    GovRun run;
    gnss_raw::BuiltEpoch built;
    for (int step = 1; step * dt <= fl.t_end + 1e-9; ++step)
    {
        const double t = step * dt;
        const bool boost = fl.launch_t >= 0.0 && t > fl.launch_t && t <= burnout_t;
        const bool flying = fl.launch_t >= 0.0 && t > fl.launch_t;
        double f_b[3] = {0.0, 0.0, 0.0}, a_n[3] = {0.0, 0.0, 0.0};
        if (flying)
        {
            f_b[0] = boost ? 3.0 * kG : 0.0;
            double f_n[3];
            mat3Vec(Ct, f_b, f_n);
            for (int i = 0; i < 3; ++i)
                a_n[i] = f_n[i] + (i == 2 ? kG : 0.0);
        }
        else
        {
            const double up[3] = {0, 0, -kG};
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
        s.flags = !flying ? gnss_raw::kShadowLevel : (t >= fl.launch_t + 0.1 ? gnss_raw::kShadowInFlight : 0);
        for (RawGnssShadow* x : shs)
            x->onImu(s, t64);
        run.state.push_back(stateHash(sh.filter()));
        if (shs.size() > 1)
            run.state2.push_back(stateHash(shs[1]->filter()));
        if (step % 20 == 0)
        {
            double dr[3], r[3], ve[3];
            nedToEcefVec(p_ned, dr);
            nedToEcefVec(v_ned, ve);
            for (int i = 0; i < 3; ++i)
                r[i] = r0[i] + dr[i];
            Due d;
            d.e.epoch = sky.epoch(tow0 + std::round(t * 10.0) / 10.0, r, ve);
            d.e.mcu_us = t64;
            d.e.gps_week = kGoldenWeek;
            d.hand_us = t64 + static_cast<int64_t>(std::llround(fl.delay_ms(t) * 1000.0));
            d.t = t;
            if (!fl.lose(t))
                due.push_back(d);
        }
        for (size_t k = 0; k < due.size();)
        {
            if (t64 < due[k].hand_us)
            {
                ++k;
                continue;
            }
            gnss_raw::ShadowNav nav;
            sh.navState(nav);
            if (stage.offer(due[k].e, nav, built))
            {
                for (RawGnssShadow* x : shs)
                    x->onBuilt(built);
                run.t.push_back(due[k].t);
                run.nav_cap.push_back(nav.sat_cap);
                run.built.push_back(built.n);
            }
            for (RawGnssShadow* x : shs)
                x->onMeasStats(stage.stats());
            due.erase(due.begin() + static_cast<long>(k));
        }
    }
    run.gov = sh.capGovernor().stats();
    return run;
}

GovRun flyGov(const RawGnssShadow::Config& cfg, const GovFlight& fl)
{
    RawGnssShadow sh(cfg);
    return flyGov({&sh}, fl);
}

// The cap the navigation state carried at the last epoch handed over at or before t
int capAt(const GovRun& r, double t)
{
    int c = -1;
    for (size_t i = 0; i < r.t.size() && r.t[i] <= t + 1e-9; ++i)
        c = r.nav_cap[i];
    return c;
}

}  // namespace

TEST(RawGnssShadowV9, TheCapRisesOnThePadHoldsTheFloorThroughTheBoostAndClimbsAfter)
{
    // Epochs handed over 50 ms after their tags: every one closes well under
    // the raise age. On the pad the cap climbs from 8 to 12, a satellite every
    // 1.2 s; from the launch to burnout + the screen's 1.5 s window + the 2 s
    // hold it is 8; then it climbs again. The stage builds what it is told.
    GovFlight fl;
    fl.launch_t = 12.0;
    fl.t_end = 24.0;
    const GovRun r = flyGov(govCfg(), fl);
    ASSERT_GT(r.t.size(), 200u);
    EXPECT_EQ(capAt(r, 2.0) >= 8 && capAt(r, 2.0) <= 9, true) << capAt(r, 2.0);
    EXPECT_EQ(capAt(r, 9.0), 12);
    EXPECT_EQ(capAt(r, 11.9), 12);
    for (size_t i = 0; i < r.t.size(); ++i)
    {
        if (r.t[i] > 12.0 + 0.01 && r.t[i] < 14.0 + 1.5 + 2.0 - 0.05)
            EXPECT_EQ(r.nav_cap[i], 8) << "t " << r.t[i];
        if (r.nav_cap[i] > 0)
            EXPECT_EQ(r.built[i], std::min(r.nav_cap[i], 15)) << "t " << r.t[i];
        EXPECT_GE(r.nav_cap[i], 8);
        EXPECT_LE(r.nav_cap[i], 12);
    }
    EXPECT_GT(capAt(r, 19.0), 8);
    EXPECT_EQ(capAt(r, 23.9), 12);
    EXPECT_EQ(r.gov.cuts + r.gov.floors, 0u);
    EXPECT_EQ(r.gov.boosts, 1u);
    EXPECT_EQ(r.gov.raises, 8u);
}

TEST(RawGnssShadowV9, TheCapIsCutWhenTheEpochsComeLateAndFloorsOnOneTooOldOrLost)
{
    // Epochs handed over 170 ms late from 10 s (a lag that stays, past the
    // 160 ms cut): cut two satellites at a time to the floor
    {
        GovFlight fl;
        fl.t_end = 13.0;
        fl.delay_ms = [](double t) { return t >= 10.0 ? 170.0 : 50.0; };
        const GovRun r = flyGov(govCfg(), fl);
        EXPECT_EQ(capAt(r, 9.9), 12);
        EXPECT_EQ(capAt(r, 12.9), 8);
        EXPECT_EQ(r.gov.cuts, 2u);
        EXPECT_EQ(r.gov.floors, 0u);
    }
    // one handed over 205 ms late: its fusion passes the at-once point, the floor
    {
        GovFlight fl;
        fl.t_end = 11.0;
        fl.delay_ms = [](double t) { return std::fabs(t - 10.0) < 0.01 ? 205.0 : 50.0; };
        const GovRun r = flyGov(govCfg(), fl);
        EXPECT_EQ(capAt(r, 9.95), 12);
        EXPECT_EQ(capAt(r, 10.95), 8);
        EXPECT_EQ(r.gov.floors, 1u);
        EXPECT_EQ(r.gov.late, 1u);
    }
    // one never handed over: lost on the way, the floor
    {
        GovFlight fl;
        fl.t_end = 11.0;
        fl.lose = [](double t) { return std::fabs(t - 10.0) < 0.01; };
        const GovRun r = flyGov(govCfg(), fl);
        EXPECT_EQ(capAt(r, 9.95), 12);
        EXPECT_EQ(capAt(r, 10.95), 8);
        EXPECT_EQ(r.gov.lost, 1u);
        EXPECT_EQ(r.gov.floors, 1u);
    }
    // one handed over too late to fuse (past max_epoch_age_s): dropped stale, the floor
    {
        GovFlight fl;
        fl.t_end = 11.0;
        fl.delay_ms = [](double t) { return std::fabs(t - 10.0) < 0.01 ? 320.0 : 50.0; };
        RawGnssShadow sh(govCfg());
        const GovRun r = flyGov({&sh}, fl);
        EXPECT_EQ(capAt(r, 9.95), 12);
        EXPECT_EQ(capAt(r, 10.95), 8);
        EXPECT_GE(sh.stats().epochs_stale, 1u);
        EXPECT_GE(r.gov.lost, 1u);
    }
}

TEST(RawGnssShadowV9, TheBoostScreenOnEveryEpochKeepsTheCapAtTheFloor)
{
    // The CPU ladder's bench forces the screen on every epoch before the
    // apogee call (boost_screen_g 0): the governor reads each as the boost
    // and never raises -- that bench measures the floor's cost
    RawGnssShadow::Config c = govCfg();
    c.boost_screen = RawGnssShadow::kScreenOn;
    c.boost_screen_g = 0.0;
    GovFlight fl;
    fl.t_end = 12.0;
    const GovRun r = flyGov(c, fl);
    ASSERT_GT(r.t.size(), 100u);
    for (size_t i = 0; i < r.t.size(); ++i)
        EXPECT_EQ(r.nav_cap[i], 8) << "t " << r.t[i];
    EXPECT_EQ(r.gov.raises, 0u);
}

TEST(RawGnssShadowV9, TheGovernorNeverTouchesTheFilter)
{
    // Two filters fed the same built epochs, one with the governor (whose
    // cap the stage builds with) and one without: their states are the same
    // at every step -- the governor only tells the stage how many to build --
    // through the pad's climb, a late stretch, a lost epoch, the boost and
    // the coast; so is all of their navigation state but the cap. (Its one
    // other act, cutting an epoch built above the floor whose job starts in
    // the boost to the floor's, has nothing to cut here: each epoch is built
    // as it is handed over. The test after this one has the pipeline.)
    GovFlight fl;
    fl.launch_t = 12.0;
    fl.t_end = 20.0;
    fl.delay_ms = [](double t) { return t >= 6.0 && t < 7.0 ? 170.0 : 50.0; };
    fl.lose = [](double t) { return std::fabs(t - 9.0) < 0.01; };
    RawGnssShadow on(govCfg()), off(govCfg(8, 0));
    const GovRun r = flyGov({&on, &off}, fl);
    ASSERT_GT(r.t.size(), 150u);
    ASSERT_EQ(r.state.size(), r.state2.size());
    EXPECT_EQ(r.state, r.state2);
    EXPECT_GT(r.gov.raises, 0u);
    EXPECT_GT(r.gov.cuts + r.gov.floors, 0u);
    EXPECT_EQ(r.gov.trims, 0u);
    EXPECT_EQ(on.stats().epochs_applied, off.stats().epochs_applied);
    EXPECT_EQ(on.stats().epochs_stale, off.stats().epochs_stale);
    gnss_raw::ShadowNav na, nb;
    on.navState(na);
    off.navState(nb);
    EXPECT_EQ(nb.sat_cap, 0);
    EXPECT_GE(na.sat_cap, 8);
    EXPECT_EQ(na.time_us, nb.time_us);
    EXPECT_EQ(std::memcmp(na.lla, nb.lla, sizeof(na.lla)), 0);
    EXPECT_EQ(std::memcmp(na.vel_ned, nb.vel_ned, sizeof(na.vel_ned)), 0);
    EXPECT_EQ(std::memcmp(na.quat, nb.quat, sizeof(na.quat)), 0);
    EXPECT_EQ(std::memcmp(na.accel_bias, nb.accel_bias, sizeof(na.accel_bias)), 0);
    EXPECT_EQ(std::memcmp(na.gyro_bias, nb.gyro_bias, sizeof(na.gyro_bias)), 0);
    EXPECT_EQ(std::memcmp(na.var, nb.var, sizeof(na.var)), 0);
    EXPECT_EQ(na.running, nb.running);
    EXPECT_EQ(na.gnss_aided, nb.gnss_aided);
    EXPECT_EQ(na.baro_aided, nb.baro_aided);
    EXPECT_EQ(na.in_flight, nb.in_flight);
    EXPECT_EQ(na.epochs, nb.epochs);
}

namespace
{

// The launch pipeline (the review of 4aa08dd7): the GNSS task builds an
// epoch some 20 ms after its tag at the navigation state's cap, and the
// filter starts its job after the build and the epoch before it close (here
// 80 ms after the tag). An ignition at 10.05 s falls between the build and
// the job of the epoch tagged 10.0. prelaunch_t: the ticks carry
// kShadowPrelaunch on the pad from then (< 0: never, a launch from READY).
struct PipeClose
{
    uint32_t tow_ms;
    int meas;
    bool screened;
};

struct PipeRun
{
    int probe_cap = -1, probe_n = -1;      // the epoch tagged 10.0: the cap it was built with, its satellites
    int cap_at_4_9 = -1;                   // the navigation state's cap at 4.9 s
    int gov_at_probe_hand = -1;            // the governor's cap when the probe was handed over
    uint32_t probe_tow_ms = 0;
    std::vector<PipeClose> closed;         // each job closed (one a step here)
    gnss_raw::CapGovernor::Stats gov;
    uint32_t epochs_applied = 0;
    float vel_end[3] = {};
};

// mod: the config's other switches (the bench integration's); paired: every epoch handed over in pairs
PipeRun flyPipe(double prelaunch_t, const std::function<void(RawGnssShadow::Config&)>& mod = nullptr,
                bool paired = false)
{
    RawGnssShadow::Config cfg = govCfg(8, 12);
    cfg.boost_screen = RawGnssShadow::kScreenOn;
    cfg.boost_screen_g = 2.0;
    if (mod)
        mod(cfg);
    std::unique_ptr<RawGnssShadow> shp(new RawGnssShadow(cfg));
    RawGnssShadow& sh = *shp;
    double lat, lon, h;
    ecef_to_geo(kGoldenRmRx, &lat, &lon, &h);
    double r0[3];
    geo_to_ecef(lat, lon, h, r0);
    gnss_raw::RawGnssMeasStage stage(RawGnssShadow::stageConfig(sh.config()));
    for (int i = 0; i < kGoldenRmEphN; ++i)
        stage.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
    SynthSky sky(sh.config().meas);
    const Mat3 Ct = leaningBody(0.0, 70.0);
    ShadowSeed seed = padSeed(kT0, lat, lon, h);
    quatOf(Ct, seed.quat);
    sh.onSeed(seed);
    const double tow0 = kGoldenRmTowA, dt = 0.005;
    const double launch_t = 10.05;
    struct Pend
    {
        TimedRawEpoch e;
        int64_t build_us = 0, hand_us = 0;
        double tag = 0.0;
        bool built = false;
        int nav_cap = 0;
        gnss_raw::BuiltEpoch b;
    };
    std::vector<std::unique_ptr<Pend>> q;
    PipeRun run;
    run.probe_tow_ms = static_cast<uint32_t>(std::llround(std::fmod(tow0 + 10.0, 604800.0) * 1000.0));
    uint32_t last_tow_ms = 0, screened = 0;
    for (int step = 1; step * dt <= 12.0 + 1e-9; ++step)
    {
        const double t = step * dt;
        const bool flying = t > launch_t;
        double f_b[3] = {0.0, 0.0, 0.0};
        if (flying)
            f_b[0] = 3.0 * kG;
        else
        {
            const double up[3] = {0, 0, -kG};
            Mat3 CtT{};
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    CtT.m[i][j] = Ct.m[j][i];
            mat3Vec(CtT, up, f_b);
        }
        const int64_t t64 = kT0 + static_cast<int64_t>(std::llround(t * 1e6));
        ShadowImu s{};
        s.time_us = static_cast<uint32_t>(t64);
        for (int i = 0; i < 3; ++i)
            s.acc[i] = float(f_b[i]);
        s.flags = !flying ? gnss_raw::kShadowLevel : (t >= launch_t + 0.1 ? gnss_raw::kShadowInFlight : 0);
        if (prelaunch_t >= 0.0 && t >= prelaunch_t && !(s.flags & gnss_raw::kShadowInFlight))
            s.flags |= gnss_raw::kShadowPrelaunch;
        sh.onImu(s, t64);
        if (sh.stats().last_tow_ms != last_tow_ms)
        {
            last_tow_ms = sh.stats().last_tow_ms;
            run.closed.push_back({last_tow_ms, sh.stats().last_meas, sh.stats().boost_screened != screened});
            screened = sh.stats().boost_screened;
        }
        if (step % 20 == 0)
        {
            const double ve[3] = {0, 0, 0};
            std::unique_ptr<Pend> p(new Pend());
            p->e.epoch = sky.epoch(tow0 + std::round(t * 10.0) / 10.0, r0, ve);
            p->e.mcu_us = t64;
            p->e.gps_week = kGoldenWeek;
            p->e.rr_paired = paired ? 1 : 0;
            p->build_us = t64 + 20000;
            p->hand_us = t64 + 80000;
            p->tag = t;
            q.push_back(std::move(p));
        }
        for (size_t k = 0; k < q.size();)
        {
            Pend& p = *q[k];
            if (!p.built && t64 >= p.build_us)
            {
                gnss_raw::ShadowNav nav;
                sh.navState(nav);
                p.built = stage.offer(p.e, nav, p.b);
                p.nav_cap = nav.sat_cap;
                sh.onMeasStats(stage.stats());
                if (std::fabs(p.tag - 4.9) < 1e-6)
                    run.cap_at_4_9 = nav.sat_cap;
            }
            if (p.built && t64 >= p.hand_us)
            {
                if (std::fabs(p.tag - 10.0) < 1e-6)
                {
                    run.probe_cap = p.nav_cap;
                    run.probe_n = p.b.n;
                    run.gov_at_probe_hand = sh.capGovernor().cap();
                }
                sh.onBuilt(p.b);
                q.erase(q.begin() + static_cast<long>(k));
                continue;
            }
            ++k;
        }
    }
    run.gov = sh.capGovernor().stats();
    run.epochs_applied = sh.stats().epochs_applied;
    for (int i = 0; i < 3; ++i)
        run.vel_end[i] = sh.filter().velNed()[i];
    return run;
}

}  // namespace

// A launch from READY (no PRELAUNCH: #382's gates unmet): the epoch tagged
// 10.0 is built at the pad's cap (12) and its job starts after the IMU passed
// boost_screen_g; the boost screen judges it -- the scratch copy and every
// satellite twice, at the cap the boost is held at the floor to keep off. The
// job keeps only the floor's (8) satellites, and no screened job has more.
TEST(RawGnssShadowV9, AnEpochBuiltAtThePadsCapWhoseJobStartsInTheBoostIsWorkedAsTheFloors)
{
    const PipeRun r = flyPipe(-1.0);
    EXPECT_EQ(r.probe_cap, 12);               // built at the pad's cap
    EXPECT_EQ(r.probe_n, 12);
    EXPECT_EQ(r.gov_at_probe_hand, 8);        // the IMU's boost sign floored the governor already
    bool found = false;
    size_t n_screened = 0;
    for (const PipeClose& c : r.closed)
    {
        if (c.tow_ms == r.probe_tow_ms)
        {
            found = true;
            EXPECT_TRUE(c.screened);
            EXPECT_LE(c.meas, 8);             // the screened job: the floor's satellites
            EXPECT_GE(c.meas, 4);
        }
        if (c.screened)
        {
            n_screened++;
            EXPECT_LE(c.meas, 8) << "tow_ms " << c.tow_ms;   // no screened job above the floor
        }
    }
    EXPECT_TRUE(found);
    EXPECT_GE(n_screened, 5u);
    EXPECT_EQ(r.gov.trims, 1u);
    EXPECT_EQ(r.gov.trimmed, 4u);
}

// The same launch from PRELAUNCH (from 5 s): the cap climbed in READY, and
// the pad holds the floor from PRELAUNCH, so the epochs built for the
// launch's first jobs are the floor's -- nothing to cut
TEST(RawGnssShadowV9, PrelaunchHoldsTheFloorSoTheLaunchsFirstEpochsAreBuiltAtIt)
{
    const PipeRun r = flyPipe(5.0);
    EXPECT_GT(r.cap_at_4_9, 8);               // READY: climbing
    EXPECT_EQ(r.probe_cap, 8);                // PRELAUNCH: the floor
    EXPECT_EQ(r.probe_n, 8);
    for (const PipeClose& c : r.closed)
        if (c.screened)
            EXPECT_LE(c.meas, 8) << "tow_ms " << c.tow_ms;
    EXPECT_EQ(r.gov.trims, 0u);
    EXPECT_EQ(r.gov.boosts, 1u);              // one hold, from PRELAUNCH through the boost
}

// The same launch pipeline with the bench integration's other three
// switches on too (exp/v9-moredata-bench): the cheaper build in the GNSS
// task, the shared geometry with the Doppler in float, and every epoch in
// pairs. The epoch built at the pad's cap is cut to the floor's at its job's
// start just the same, before any unit takes the epoch's lines of sight or a
// satellite's range acceleration (neither is taken for a satellite cut), and
// every job closes with the satellites the governor alone gives it, no
// screened one above the floor. (Which epochs the screen drops can differ:
// the synthetic sky stands still through the boost, and the pairs take its
// Dopplers 25 ms earlier.)
TEST(RawGnssShadowV9, TheCutToTheFloorComposesWithTheCheaperBuildTheSharedGeometryAndThePairs)
{
    const PipeRun gov = flyPipe(-1.0);
    const PipeRun all = flyPipe(
        -1.0,
        [](RawGnssShadow::Config& c) {
            c.meas.lite = true;               // the profile's build_lite, applied
            c.meas.atmo_period_s = 1.0;
            c.ekf.share_los = true;           // TR_BENCH_RAW_SHARE_LOS 2
            c.ekf.rr_float = true;
            c.ekf.rr_pair_lag_s = 0.025;      // the profile's rr_pair_avg, applied
        },
        true);
    EXPECT_EQ(all.probe_cap, 12);
    EXPECT_EQ(all.probe_n, 12);
    EXPECT_EQ(all.gov_at_probe_hand, 8);
    EXPECT_EQ(all.gov.trims, gov.gov.trims);
    EXPECT_EQ(all.gov.trimmed, gov.gov.trimmed);
    EXPECT_EQ(all.gov.trims, 1u);
    EXPECT_EQ(all.gov.trimmed, 4u);
    EXPECT_EQ(all.gov.boosts, gov.gov.boosts);
    EXPECT_EQ(all.gov.raises, gov.gov.raises);
    ASSERT_EQ(all.closed.size(), gov.closed.size());
    size_t n_screened = 0;
    for (size_t i = 0; i < all.closed.size(); ++i)
    {
        EXPECT_EQ(all.closed[i].tow_ms, gov.closed[i].tow_ms) << i;
        EXPECT_EQ(all.closed[i].meas, gov.closed[i].meas) << "tow_ms " << all.closed[i].tow_ms;
        if (all.closed[i].screened)
        {
            n_screened++;
            EXPECT_LE(all.closed[i].meas, 8) << "tow_ms " << all.closed[i].tow_ms;
        }
        if (all.closed[i].tow_ms == all.probe_tow_ms)
            EXPECT_LE(all.closed[i].meas, 8);
    }
    EXPECT_GE(n_screened, 5u);
    EXPECT_GE(all.epochs_applied + 2, gov.epochs_applied);
    for (int i = 0; i < 3; ++i)
        EXPECT_TRUE(std::isfinite(all.vel_end[i])) << i;
}

TEST(RawGnssShadowV9, OffTheNavigationStateCarriesNoCap)
{
    // No ceiling (every profile's), or no GNSS task to cap for, or the
    // Beetle's: the state's cap is 0 and the stage builds with its own
    GovFlight fl;
    fl.t_end = 6.0;
    for (int k = 0; k < 3; ++k)
    {
        RawGnssShadow::Config c = govCfg(8, 0);
        if (k == 1)
        {
            c = mosaicCfg();                  // the flight image's: 16, no ceiling
            c.external_build = true;
        }
        if (k == 2)
        {
            c = govCfg(8, 12);
            c.external_build = false;         // the shadow builds its own: no cap to govern
        }
        RawGnssShadow sh(c);
        EXPECT_FALSE(sh.capGovernor().on()) << k;
        if (k == 2)
            continue;
        const GovRun r = flyGov({&sh}, fl);
        ASSERT_GT(r.t.size(), 30u);
        for (size_t i = 0; i < r.t.size(); ++i)
            EXPECT_EQ(r.nav_cap[i], 0) << k;
    }
    RawGnssShadow::Config b = primaryCfg();
    gnss_raw::applyReceiverProfile(gnss_raw::lc86gProfile(), b);
    b.external_build = true;
    EXPECT_EQ(b.sat_cap_max, 0);
    RawGnssShadow beetle(b);
    EXPECT_FALSE(beetle.capGovernor().on());
}
