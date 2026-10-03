// Carrier smoothing and its slip check (rx_smooth): a slip the lock detector missed restarts that
// channel's smoothing; what every channel shares, and a code that only wanders, leave it alone.

extern "C" {
#include "gnss/rx.h"
}

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace {

const double LAMBDA = GNSS_C / GNSS_FREQ_L1_HZ;

// Eight GPS channels at 45 dB-Hz, locked, 0.1 s epochs. Each satellite's range changes at its own rate;
// its code carries the raw code's noise (a fixed pseudo-random sequence), its carrier none.
struct Sky {
    std::unique_ptr<rx_t> rx{new rx_t};
    rx_obs_t obs[8];
    float smooth[8];
    double slip_m[8] = {};      // carrier phase moved by this much (a slip), from the epoch it is set
    double shared_mps = 0.0;    // carrier drifts from the code on every channel (the emulator's oscillator)
    double wander_m = 0.0;      // channel 0's code wanders this much, at wander_hz
    double wander_hz = 0.2;
    uint32_t seed = 1;
    int epoch = 0;

    explicit Sky(float slip_k = 4.0f)
    {
        rx_cfg_t c;
        rx_default_cfg(&c, 6.75e6, 0.0);
        c.slip_k = slip_k;
        rx_init(rx.get(), &c);
        for (int k = 0; k < 8; k++) {
            obs[k] = rx_obs_t{};
            obs[k].sys = GNSS_SYS_GPS;
            obs[k].prn = k + 1;
            obs[k].ch = k;
            obs[k].cn0 = 45.0f;
            obs[k].lock_s = 10.0f;
        }
    }
    double range(int k, double t) const { return 2.1e7 + 1e5 * k + (300.0 * k - 1000.0) * t; }
    double noise()
    {
        seed = seed * 1664525u + 1013904223u;
        return ((seed >> 8) / 16777216.0 - 0.5) * 1.2;  // uniform, 0.35 m rms
    }
    // One epoch; returns channel k's smoothed error, m.
    double step(int k_out)
    {
        const double t = 0.1 * epoch;
        const uint64_t ts = (uint64_t)llround(t * rx->cfg.fs) + 1000;
        for (int k = 0; k < 8; k++) {
            const double wander = k == 0 ? wander_m * std::sin(2.0 * M_PI * wander_hz * t) : 0.0;
            obs[k].pr_raw = range(k, t) + noise() + wander;
            obs[k].adr = (range(k, t) + slip_m[k] + shared_mps * t) / LAMBDA;
        }
        rx_smooth(rx.get(), ts, obs, 8, smooth);
        epoch++;
        return obs[k_out].pr - range(k_out, t);
    }
    double run(double s, int k_out)
    {
        double e = 0.0;
        for (int n = 0; n < (int)llround(s / 0.1); n++) {
            e = step(k_out);
        }
        return e;
    }
};

}  // namespace

TEST(Smooth, AveragesTheCodeNoise)
{
    Sky s;
    double worst = 0.0;
    s.run(30.0, 0);
    for (int n = 0; n < 300; n++) {
        worst = std::fmax(worst, std::fabs(s.step(0)));
    }
    EXPECT_LT(worst, 0.2);  // 0.35 m rms raw
    EXPECT_EQ(s.rx->n_slip, 0u);
}

TEST(Smooth, ASlipTheLockMissedRestartsThatChannel)
{
    Sky s;
    s.run(30.0, 3);
    s.slip_m[3] = 30.0 * LAMBDA;  // 30 whole cycles, lock kept: the smoothing would carry 5.7 m
    double e = 0.0;
    for (int n = 0; n < 50; n++) {
        e = s.step(3);
    }
    EXPECT_EQ(s.rx->n_slip, 1u);  // within 5 s, and on that channel alone
    EXPECT_LT(std::fabs(e), 1.0);
    Sky u(0.0f);  // the check off: the step drags
    u.run(30.0, 3);
    u.slip_m[3] = 30.0 * LAMBDA;
    EXPECT_GT(std::fabs(u.run(5.0, 3)), 4.0);
    EXPECT_EQ(u.rx->n_slip, 0u);
}

TEST(Smooth, WhatEveryChannelSharesIsLeftAlone)
{
    Sky s;
    s.shared_mps = 3.0;  // every carrier walks from its code at 3 m/s: the fix takes it as clock
    s.run(60.0, 0);
    EXPECT_EQ(s.rx->n_slip, 0u);
}

TEST(Smooth, ACodeThatOnlyWandersIsLeftAlone)
{
    Sky s;
    s.wander_m = 1.0;  // a metre each way every 5 s, as a code that beats with the sample phase
    s.run(120.0, 0);
    EXPECT_EQ(s.rx->n_slip, 0u);
}
