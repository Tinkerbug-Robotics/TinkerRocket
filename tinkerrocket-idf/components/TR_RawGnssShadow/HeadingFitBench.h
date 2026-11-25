// HeadingFitBench.h — what the heading from thrust costs a CPU: the fit's
// step, an epoch's Doppler velocity by the number of satellites, and the fit
// by the number of epochs it holds (its own profile against the prototype's
// full grid), each timed on whatever clock the caller hands it. Synthetic: a
// boosted, leaning body at the navigation filter's 100 Hz step, an epoch a
// second, and a sky of satellites 20,000 km up.
//
// The same code on the host (tests_cpp/bench_heading_fit.cpp, std::chrono)
// and on the flight computer (TR_HEADING_FIT_BENCH: at boot, on the raw-GNSS
// task's core, esp_timer), so the two can be set side by side. Header-only,
// bench only: nothing in flight calls it.
#ifndef HEADING_FIT_BENCH_H
#define HEADING_FIT_BENCH_H

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "HeadingFit.h"
#include "TR_RawGnssEKF.h"   // stillRangeRate

namespace gnss_raw
{
namespace bench
{

using Clock = int64_t (*)();                      // any tick: run() is told how many make a microsecond
using Print = void (*)(const char* line);

// A sky of n satellites above (lat, lon, h): fixed azimuths and elevations,
// orbit-like motion, the receiver moving at v_ned; each range rate as the
// LC86G's (with a clock drift) and one 25 m/s off when bad.
inline int sky(RawMeas* out, int n, const double r[3], double lat, double lon, const double v_ned[3], bool bad)
{
    constexpr double kPi = 3.14159265358979323846;
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    auto enu2ecef = [&](const double e[3], double o[3]) {
        o[0] = -so * e[0] - sl * co * e[1] + cl * co * e[2];
        o[1] = co * e[0] - sl * so * e[1] + cl * so * e[2];
        o[2] = cl * e[1] + sl * e[2];
    };
    const double v_enu[3] = {v_ned[1], v_ned[0], -v_ned[2]};
    double v[3];
    enu2ecef(v_enu, v);
    if (n > RawEpoch::kMaxObs)
        n = RawEpoch::kMaxObs;
    for (int i = 0; i < n; ++i)
    {
        const double az = (37.0 * i + 11.0 * (i / 8)) * kPi / 180.0;
        const double el = (10.0 + std::fmod(23.0 * i, 75.0)) * kPi / 180.0;
        const double enu[3] = {std::cos(el) * std::sin(az), std::cos(el) * std::cos(az), std::sin(el)};
        double u[3];
        enu2ecef(enu, u);
        const double R = 26.56e6, ru = r[0] * u[0] + r[1] * u[1] + r[2] * u[2];
        const double rr = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
        const double rho = -ru + std::sqrt(ru * ru - rr + R * R);
        RawMeas& m = out[i];
        m = RawMeas{};
        m.sys = static_cast<uint8_t>(i % 3);
        m.prn = static_cast<uint8_t>(1 + i);
        for (int k = 0; k < 3; ++k)
            m.sat_pos[k] = r[k] + rho * u[k];
        const double z[3] = {0.3, -0.2, 1.0};
        const double sv[3] = {m.sat_pos[1] * z[2] - m.sat_pos[2] * z[1], m.sat_pos[2] * z[0] - m.sat_pos[0] * z[2],
                              m.sat_pos[0] * z[1] - m.sat_pos[1] * z[0]};
        const double nsv = std::sqrt(sv[0] * sv[0] + sv[1] * sv[1] + sv[2] * sv[2]);
        for (int k = 0; k < 3; ++k)
            m.sat_vel[k] = 3870.0 * sv[k] / nsv;
        double uu[3];
        const double still = stillRangeRate(m, r, uu);
        m.has_pr = m.has_rr = true;
        m.pr = sagnacRange(m, r) + 1000.0 + 2.0 * std::cos(2.3 * i);   // a clock, and metres of error
        m.sigma_pr = 3.0f;
        m.sigma_pr_corr = 2.0f;
        m.tau_pr = 15.0f;
        m.tau_rr = 0.1f;
        m.rr = still - (uu[0] * v[0] + uu[1] * v[1] + uu[2] * v[2]) + 7.5 + 0.03 * std::sin(1.7 * i);
        m.sigma_rr = 0.2f;
        if (bad && i == n / 2)
            m.rr += 25.0;
    }
    return n;
}

// The fit fed a boosted, leaning flight at 100 Hz from a seed 30 deg off;
// an epoch a second until it holds epochs of them (the store's 64 at most).
// Returns the fit's step time in clock ticks (the mean of the steps).
inline double fly(HeadingFit& fit, int epochs, Clock clock)
{
    constexpr double kPi = 3.14159265358979323846;
    // nose up, leaning 5 deg toward azimuth 70: body x along nose, a seed 30 deg off in heading
    const double lean = 5.0 * kPi / 180.0, az = 70.0 * kPi / 180.0, err = 30.0 * kPi / 180.0;
    const double nose[3] = {std::sin(lean) * std::cos(az), std::sin(lean) * std::sin(az), -std::cos(lean)};
    // the seed: pitched up 90 - 5 deg, its heading the lean's plus the error (yaw then pitch, body -> NED)
    const double yaw = az + err, pitch = kPi / 2 - lean;
    const float q[4] = {static_cast<float>(std::cos(yaw / 2) * std::cos(pitch / 2)),
                        static_cast<float>(-std::sin(yaw / 2) * std::sin(pitch / 2)),
                        static_cast<float>(std::cos(yaw / 2) * std::sin(pitch / 2)),
                        static_cast<float>(std::sin(yaw / 2) * std::cos(pitch / 2))};
    const float wb[3] = {0.0f, 0.0f, 0.0f};
    fit.start(q, wb, 0.0);
    double v[3] = {0.0, 0.0, 0.0};
    int64_t step_us = 0;
    int steps = 0;
    const double dt = 0.01;
    for (int k = 1; k <= 100 * epochs + 100; ++k)
    {
        const double t = k * dt;
        const float f = t < 1.5 ? 100.0f : -3.0f;
        const float acc[3] = {f, 0.01f, -0.02f}, gyro[3] = {0.001f, 0.0f, 0.0f};
        const int64_t c0 = clock();
        fit.imu(acc, gyro, static_cast<float>(dt));
        step_us += clock() - c0;
        steps++;
        for (int i = 0; i < 3; ++i)
            v[i] += (f * nose[i] + (i == 2 ? 9.807 : 0.0)) * dt;
        if (k % 100 == 0 && fit.epochs() < epochs)
        {
            const double vh[2] = {v[0] + 0.05 * std::sin(0.3 * k), v[1] + 0.05 * std::cos(0.7 * k)};
            fit.gnss(fit.time(), vh, 0.04);
        }
    }
    return static_cast<double>(step_us) / steps;
}

// The bench: one line a measurement through print, in microseconds (the
// clock's ticks over ticks_per_us). reps: how often each is timed (the mean
// is printed, and the worst). fit and m (RawEpoch::kMaxObs of them) are its
// working storage, 19 kB, more than a task's stack: the flight computer
// lends it PSRAM, where the shadow's own fit lives, and takes it back after.
inline void run(HeadingFit& fit, RawMeas* m, Clock clock, Print print, int reps = 20, double ticks_per_us = 1.0)
{
    char line[200];
    const double k_us = 1.0 / ticks_per_us;
    // the fit's step
    {
        fit.reset();
        const double ticks = fly(fit, 8, clock);
        std::snprintf(line, sizeof(line), "heading fit bench: step (imu) %.2f us mean", ticks * k_us);
        print(line);
    }
    // an epoch's Doppler velocity by satellites, clean and with one dropped
    {
        constexpr double kPi = 3.14159265358979323846;
        const double lat = 32.99 * kPi / 180.0, lon = -106.97 * kPi / 180.0;
        double r[3];
        {
            const double a = 6378137.0, e2 = 0.0066943799901, s = std::sin(lat);
            const double N = a / std::sqrt(1.0 - e2 * s * s), h = 1400.0;
            r[0] = (N + h) * std::cos(lat) * std::cos(lon);
            r[1] = (N + h) * std::cos(lat) * std::sin(lon);
            r[2] = (N * (1.0 - e2) + h) * s;
        }
        const double v_ned[3] = {4.0, 11.0, -120.0};
        const int sats[] = {6, 10, 15, 20, 30, 40};
        for (int n : sats)
            for (int bad = 0; bad < 2; ++bad)
            {
                const int nm = sky(m, n, r, lat, lon, v_ned, bad != 0);
                int64_t sum = 0, worst = 0;
                int used = 0;
                double vn[3], var = 0.0;
                for (int k = 0; k < reps; ++k)
                {
                    const int64_t c0 = clock();
                    dopplerVelocityNed(m, nm, r, lat, lon, vn, var, used);
                    const int64_t d = clock() - c0;
                    sum += d;
                    worst = d > worst ? d : worst;
                }
                std::snprintf(line, sizeof(line),
                              "heading fit bench: velocity, %2d satellites%s: %8.1f us mean, %8.1f max (used %d)", n,
                              bad ? " (one dropped)" : "              ", static_cast<double>(sum) / reps * k_us,
                              static_cast<double>(worst) * k_us, used);
                print(line);
            }
    }
    // the fit by epochs held: its own profile, and the prototype's full grid
    const int eps[] = {2, 4, 8, 16, 32, 64};
    for (int ep : eps)
    {
        fit.reset();
        fly(fit, ep, clock);
        for (int grid = 0; grid < 2; ++grid)
        {
            int64_t sum = 0, worst = 0;
            HeadingFit::Result r;
            for (int k = 0; k < reps; ++k)
            {
                const int64_t c0 = clock();
                r = grid ? fit.solveGrid() : fit.solve();
                const int64_t d = clock() - c0;
                sum += d;
                worst = d > worst ? d : worst;
            }
            std::snprintf(line, sizeof(line),
                          "heading fit bench: fit, %2d epochs, %s: %8.1f us mean, %8.1f max (%d points, sigma %.2f deg)",
                          fit.epochs(), grid ? "full grid " : "own search", static_cast<double>(sum) / reps * k_us,
                          static_cast<double>(worst) * k_us, r.evals, r.sigma * 57.29578);
            print(line);
        }
    }
}

}  // namespace bench
}  // namespace gnss_raw

#endif  // HEADING_FIT_BENCH_H
