// The board's IMU model (host/imu_ism6.h): the channels, the pad calibration, and the sample-and-hold delay.

#include "imu_ism6.h"

#include <gtest/gtest.h>

#include <cmath>

namespace {

const double G = 9.80665;

struct Load {
    double g0 = 1.0, g1 = 1.0, t_step = 1e9;  // the axial force: g0 before t_step, g1 after
};

double force(void *ctx, double t)
{
    const Load *l = static_cast<const Load *>(ctx);
    return (t < l->t_step ? l->g0 : l->g1) * G;
}

// The mean of the P4's estimate over [t0, t1), m/s^2.
double mean(ism6_t *m, Load *l, double t0, double t1)
{
    double s = 0.0;
    int n = 0;
    for (double t = t0; t < t1; t += 1e-3, n++) {
        s += ism6_read(m, t, force, l);
    }
    return s / n;
}

}  // namespace

TEST(ImuIsm6, PadCalibratedReadsOneG)
{
    ism6_cfg_t c;
    ism6_cfg_default(&c, 1);  // the worst part
    ism6_t m;
    ism6_init(&m, &c, 0.0, 1);
    Load l;
    EXPECT_NEAR(mean(&m, &l, 0.01, 2.0), G, 0.003);  // the pad, offsets taken out
    c.pad_cal = 0;
    ism6_init(&m, &c, 0.0, 1);
    // Raw, the low-g offsets of both axes add along the thrust: 2 x 65 mg x cos 45, and 1 % of 1 g.
    EXPECT_NEAR(mean(&m, &l, 0.01, 2.0), G * (1.01 + 2 * 0.065 * std::cos(M_PI / 4)), 0.003);
}

TEST(ImuIsm6, ThirtyGOnTheHighGChannel)
{
    ism6_cfg_t c;
    ism6_cfg_default(&c, 0);
    ism6_t m;
    ism6_init(&m, &c, 0.0, 2);
    Load l;
    l.g1 = 30.0;
    l.t_step = 1.0;
    mean(&m, &l, 0.0, 1.0);
    const double hi = mean(&m, &l, 1.1, 2.1);
    // 21.2 g on each axis: the high-g channel. Left after the pad calibration: the sensitivity error on the 29 g
    // above the pad's, and the nonlinearity's change (2 %FS of 64 g, quadratic).
    const double f = 30.0 * std::cos(M_PI / 4), f0 = std::cos(M_PI / 4);
    const double nl = 0.02 * 64.0 * ((f / 64.0) * (f / 64.0) - (f0 / 64.0) * (f0 / 64.0));
    const double expect = 30.0 + (c.sf_hg * (f - f0) + nl) * 2 * std::cos(M_PI / 4);
    EXPECT_NEAR(hi / G, expect, 0.01);
    EXPECT_GT(m.n_high, 0u);
    EXPECT_EQ(m.n_rail, 0u);
    // The traveler's 19 g stays on the low-g channel at 45 deg: 13.4 g a side.
    ism6_init(&m, &c, 0.0, 3);
    l.g1 = 19.0;
    mean(&m, &l, 0.0, 2.1);
    EXPECT_EQ(m.n_high, 0u);
}

TEST(ImuIsm6, RawHighGAddsItsOffsetAtTheSwitch)
{
    ism6_cfg_t c;
    ism6_cfg_default(&c, 0);
    c.pad_cal = 0;
    ism6_t m;
    ism6_init(&m, &c, 0.0, 4);
    Load l;
    l.g1 = 30.0;
    l.t_step = 1.0;
    const double pad = mean(&m, &l, 0.0, 1.0) / G;
    const double hi = mean(&m, &l, 1.1, 2.1) / G;
    // Uncalibrated, the high-g channel's 250 mg a side lands on the axis at the switch: 354 mg, against the low-g's 14.
    EXPECT_NEAR(pad - 1.0, 2 * 0.010 * std::cos(M_PI / 4) + c.sf_lg, 0.002);
    EXPECT_GT(hi - 30.0, 0.30);
}

TEST(ImuIsm6, SampledAndHeldWithAboutOneSampleOfDelay)
{
    ism6_cfg_t c;
    ism6_cfg_default(&c, 0);
    c.nd_lg = c.nd_hg = 0.0;  // no noise: watch the step arrive
    ism6_t m;
    ism6_init(&m, &c, 0.0, 5);
    Load l;
    l.g1 = 10.0;
    l.t_step = 1.0;
    double t_seen = -1.0;
    for (double t = 0.0; t < 1.02; t += 1e-5) {
        if (ism6_read(&m, t, force, &l) > 5.0 * G && t_seen < 0.0) {
            t_seen = t;
        }
    }
    // The step at 1.0 s shows once a sample taken after it (less LPF1's one sample) reaches the P4.
    const double dt = 1.0 / c.odr_hz;
    EXPECT_GT(t_seen - 1.0, dt + c.transport_s - 1e-4);
    EXPECT_LT(t_seen - 1.0, 2.0 * dt + c.transport_s + 1e-4);
    EXPECT_NEAR(ism6_mean_delay(&c), 1.5 * dt + c.transport_s, 1e-12);
}
