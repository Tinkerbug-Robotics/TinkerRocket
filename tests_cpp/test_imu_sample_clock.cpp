#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "imu_sample_clock.h"

// The FIFO sample clock (#1485). A simulated IMU produces samples at its own
// true rate; the task drains the FIFO at irregular times; the clock must hand
// out times that track the true sample times, never run backwards, and stay
// evenly spaced.

namespace {

// Fixed LCG: std:: distributions differ between standard libraries, and these
// tests must give the same answer everywhere.
struct Lcg
{
    uint32_t s;
    explicit Lcg(uint32_t seed) : s(seed) {}
    uint32_t next() { s = s * 1664525u + 1013904223u; return s; }
    // Uniform in [0, 1).
    double unit() { return (next() >> 8) * (1.0 / 16777216.0); }
};

constexpr double NOMINAL_US = 1e6 / 3840.0;  // 260.4167

struct SimResult
{
    double max_abs_err_us = 0;   // after convergence
    double min_gap_us = 1e9;
    double max_gap_us = 0;
    bool backwards = false;
    uint32_t samples = 0;
    float period = 0;
    uint32_t resyncs = 0;
};

// Simulate `duration_us` of an IMU whose true period is true_period_us, first
// sample at t0. The task reads every read_every_us, plus up to jitter_us late.
// Errors are measured once `settle_us` has passed.
SimResult simulate(double true_period_us, uint32_t t0, double duration_us,
             double read_every_us, double jitter_us, double settle_us, uint32_t seed,
             double stall_at_us = -1, double stall_us = 0, uint32_t overflow_keep = 0)
{
    ImuSampleClock clk;
    clk.reset((float)NOMINAL_US);
    Lcg rng(seed);
    SimResult r;

    uint64_t produced_idx = 0;  // index of the next sample the chip will produce
    uint64_t read_idx = 0;      // index of the next sample the task will read
    double t_rel = 0;           // time since t0
    bool have_prev = false;
    uint32_t prev = 0;

    while (t_rel < duration_us)
    {
        double next_read = t_rel + read_every_us + rng.unit() * jitter_us;
        if (stall_at_us >= 0 && t_rel < stall_at_us && next_read >= stall_at_us)
            next_read += stall_us;
        t_rel = next_read;

        // Samples the chip has produced by now.
        while ((double)produced_idx * true_period_us <= t_rel) produced_idx++;
        uint64_t n = produced_idx - read_idx;
        // A FIFO overflow keeps only the newest overflow_keep samples.
        if (overflow_keep && n > overflow_keep)
        {
            read_idx = produced_idx - overflow_keep;
            n = overflow_keep;
        }
        if (n == 0) continue;

        const uint32_t t_read = t0 + (uint32_t)llround(t_rel);
        clk.beginBurst(t_read, (uint32_t)n);
        for (uint64_t k = 0; k < n; ++k)
        {
            const uint32_t got = clk.next();
            const double truth = (double)(read_idx + k) * true_period_us;
            const double err = (double)(int32_t)(got - (t0 + (uint32_t)llround(truth)));
            if (t_rel > settle_us)
                r.max_abs_err_us = std::max(r.max_abs_err_us, std::fabs(err));
            if (have_prev)
            {
                const double gap = (double)(int32_t)(got - prev);
                if (gap < 0) r.backwards = true;
                if (t_rel > settle_us)
                {
                    r.min_gap_us = std::min(r.min_gap_us, gap);
                    r.max_gap_us = std::max(r.max_gap_us, gap);
                }
            }
            prev = got;
            have_prev = true;
            r.samples++;
        }
        read_idx += n;
    }
    r.period = clk.period();
    r.resyncs = clk.resyncs;
    return r;
}

}  // namespace

TEST(ImuSampleClock, TheFirstBurstPutsItsNewestSampleHalfAPeriodBeforeTheRead)
{
    ImuSampleClock clk;
    clk.reset(260.0f);
    clk.beginBurst(100000u, 4);
    const uint32_t a = clk.next();
    const uint32_t b = clk.next();
    clk.next();
    const uint32_t d = clk.next();
    EXPECT_EQ(d, 100000u - 130u);
    EXPECT_EQ(a, 100000u - 130u - 3u * 260u);
    EXPECT_EQ(b - a, 260u);
}

TEST(ImuSampleClock, SteadyReadsTrackTheTrueSampleTimes)
{
    const SimResult r = simulate(NOMINAL_US, 5000000u, 20e6, 2000.0, 400.0, 2e6, 1u);
    EXPECT_FALSE(r.backwards);
    EXPECT_EQ(r.resyncs, 0u);
    EXPECT_LT(r.max_abs_err_us, 60.0);
    // Evenly spaced: every gap within a few microseconds of the period.
    EXPECT_GT(r.min_gap_us, NOMINAL_US - 6.0);
    EXPECT_LT(r.max_gap_us, NOMINAL_US + 6.0);
}

TEST(ImuSampleClock, AChipRunningFastIsFollowed)
{
    // The ISM6's oscillator is a few percent off nominal.
    const double true_p = NOMINAL_US * 0.97;
    const SimResult r = simulate(true_p, 1000u, 40e6, 2000.0, 400.0, 10e6, 7u);
    EXPECT_FALSE(r.backwards);
    EXPECT_NEAR(r.period, true_p, true_p * 0.002);
    EXPECT_LT(r.max_abs_err_us, 60.0);
}

TEST(ImuSampleClock, AChipRunningSlowIsFollowed)
{
    const double true_p = NOMINAL_US * 1.03;
    const SimResult r = simulate(true_p, 1000u, 40e6, 2000.0, 400.0, 10e6, 9u);
    EXPECT_FALSE(r.backwards);
    EXPECT_NEAR(r.period, true_p, true_p * 0.002);
    EXPECT_LT(r.max_abs_err_us, 60.0);
}

TEST(ImuSampleClock, AStallThatOverflowsTheFifoReanchorsInsteadOfSlewing)
{
    // 60 ms stall with a FIFO that keeps only the newest 170 samples.
    const SimResult r = simulate(NOMINAL_US, 1000u, 10e6, 2000.0, 400.0, 6e6, 3u,
                           /*stall_at_us=*/5e6, /*stall_us=*/60000.0, /*overflow_keep=*/170);
    EXPECT_FALSE(r.backwards);
    EXPECT_GE(r.resyncs, 1u);
    EXPECT_LT(r.max_abs_err_us, 60.0);
}

TEST(ImuSampleClock, TimeNeverRunsBackwardsUnderWildReadJitter)
{
    const SimResult r = simulate(NOMINAL_US, 1000u, 10e6, 500.0, 6000.0, 0.0, 11u);
    EXPECT_FALSE(r.backwards);
}

TEST(ImuSampleClock, TheMicrosecondClockWrapIsHarmless)
{
    // Start 3 s before the 32-bit microsecond clock wraps.
    const SimResult r = simulate(NOMINAL_US, 0xFFFFFFFFu - 3000000u, 10e6, 2000.0, 400.0, 2e6, 5u);
    EXPECT_FALSE(r.backwards);
    EXPECT_EQ(r.resyncs, 0u);
    EXPECT_LT(r.max_abs_err_us, 60.0);
}

TEST(ImuSampleClock, SevenKilohertzWorksToo)
{
    // The same clock at 7,680 Hz: a 130 us period.
    ImuSampleClock clk;
    clk.reset((float)(1e6 / 7680.0));
    uint32_t t = 1000000u;
    uint32_t prev = 0;
    bool have_prev = false;
    for (int burst = 0; burst < 2000; ++burst)
    {
        t += 2083u;  // 16 samples a burst
        clk.beginBurst(t, 16);
        for (int k = 0; k < 16; ++k)
        {
            const uint32_t got = clk.next();
            if (have_prev) ASSERT_GT((int32_t)(got - prev), 0);
            prev = got;
            have_prev = true;
        }
    }
    EXPECT_NEAR(clk.period(), 1e6 / 7680.0, 1.0);
}
