// Host tests for BaroGatePolicy (#450) — the EKF-side barometer admission gate.
//
// The centerpiece is a replay of the exact cadence mismatch from the
// 2026-07-09 bench sim flight: BMP samples at 487 Hz beating against a 959 Hz
// flight loop with EKF_DECIMATION=2 (EKF ticks at 479.5 Hz).  The old
// bmp_new_for_kf gate dropped every sample that arrived on an EKF-off loop
// iteration — ~50% of all samples, in alternating ~66 ms dead/live windows —
// and its rate-blind 5 m spike test then rejected the first sample after each
// dead window once climb rate exceeded ~75 m/s.  The policy must accept
// virtually every sample and flag zero spikes on that same replay.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "baro_gate_policy.h"

namespace
{
constexpr float kSpikeThreshM  = 5.0f;
constexpr float kSpikeRateMps  = 500.0f;
constexpr uint32_t kFuseIntervalUs = 20000u;  // config::BARO_FUSE_MIN_INTERVAL_US

// Pre-throttle tests evaluate with the throttle DISABLED (0) so they keep
// exercising freshness/spike behavior on every sample; throttle tests pass
// kFuseIntervalUs explicitly.
BaroGatePolicy::Decision eval(BaroGatePolicy& p, uint32_t t_us, float alt_m,
                              bool locked = false,
                              uint32_t fuse_interval_us = 0u)
{
    return p.evaluate(t_us, alt_m, locked, kSpikeThreshM, kSpikeRateMps,
                      fuse_interval_us);
}
}  // namespace

// ---------------------------------------------------------------------------
// Flight replay: 487 Hz BMP vs 959 Hz loop, EKF_DECIMATION=2, 100 m/s climb.
// Models the real consumption pattern: the loop reads samples into a
// "latest" slot every iteration; only every 2nd iteration presents that slot
// to the gate.  Samples overwritten in the slot between two EKF ticks are
// invisible to the gate by construction (~1.5% at these rates); everything
// else must be consumed exactly once, with no spikes.
// ---------------------------------------------------------------------------
TEST(BaroGatePolicy, BeatReplayConsumesEverySampleWithoutSpikes)
{
    constexpr double kLoopPeriodUs = 1e6 / 959.0;   // 1042.75 us
    constexpr double kBmpPeriodUs  = 1e6 / 487.0;   // 2053.39 us
    constexpr float  kClimbRateMps = 100.0f;        // fast boost, worst case
    constexpr int    kEkfDecimation = 2;
    constexpr double kDurationS   = 10.0;

    BaroGatePolicy gate;

    double   next_bmp_us   = 1000.0;
    uint32_t slot_time_us  = 0;      // "bmp_latest_si" equivalent
    float    slot_alt_m    = 0.0f;
    bool     slot_valid    = false;
    uint32_t slot_writes   = 0;      // samples produced into the slot
    uint32_t overwritten   = 0;      // produced but replaced before an EKF tick
    bool     slot_seen_by_ekf = true;

    uint32_t consumed = 0;
    uint32_t spikes   = 0;
    int      decim    = 0;

    const long total_iters = (long)(kDurationS * 1e6 / kLoopPeriodUs);
    for (long i = 0; i < total_iters; ++i)
    {
        const double now_us = i * kLoopPeriodUs;

        // Sensor read phase (every iteration, like main.cpp)
        if (now_us >= next_bmp_us)
        {
            if (!slot_seen_by_ekf) overwritten++;
            slot_time_us = (uint32_t)next_bmp_us;
            slot_alt_m   = kClimbRateMps * (float)(next_bmp_us * 1e-6);
            slot_valid   = true;
            slot_seen_by_ekf = false;
            slot_writes++;
            next_bmp_us += kBmpPeriodUs;
        }

        // EKF phase (every EKF_DECIMATION-th iteration)
        if (++decim >= kEkfDecimation)
        {
            decim = 0;
            if (slot_valid)
            {
                const auto d = eval(gate, slot_time_us, slot_alt_m);
                if (d.fresh)
                {
                    consumed++;
                    slot_seen_by_ekf = true;
                    if (d.spike) spikes++;
                }
            }
        }
    }

    // Every produced sample is either consumed or was overwritten in the slot;
    // nothing silently vanishes, and overwrites stay a rounding error.
    EXPECT_EQ(consumed + overwritten, slot_writes);
    EXPECT_LT((double)overwritten / slot_writes, 0.03);
    EXPECT_GT(consumed, (uint32_t)(0.97 * slot_writes));

    // The whole point of #450: a clean 100 m/s climb produces ZERO spikes.
    EXPECT_EQ(spikes, 0u);
}

// Consume-on-use: the EKF sees the same "latest sample" slot on every tick;
// a sample must be consumed exactly once, and re-presentations are no-ops
// rather than duplicate measurement updates.
TEST(BaroGatePolicy, RepeatedPresentationIsConsumedExactlyOnce)
{
    BaroGatePolicy gate;

    auto d1 = eval(gate, 1000u, 0.10f);
    EXPECT_TRUE(d1.fresh);
    EXPECT_TRUE(d1.accept);

    // Same sample presented on the next EKF ticks (no new BMP data yet):
    // consume-on-use makes the repeats no-ops instead of duplicate updates.
    auto d2 = eval(gate, 1000u, 0.10f);
    EXPECT_FALSE(d2.fresh);
    EXPECT_FALSE(d2.accept);
    auto d3 = eval(gate, 1000u, 0.10f);
    EXPECT_FALSE(d3.fresh);

    // Next real sample is fresh again.
    auto d4 = eval(gate, 3053u, 0.31f);
    EXPECT_TRUE(d4.fresh);
    EXPECT_TRUE(d4.accept);
}

// A legitimate ~66 ms sampling gap at 100 m/s (the exact #450 false-positive:
// delta ~6.6 m > 5 m threshold) must be ACCEPTED — the apparent rate is just
// the true climb rate, nowhere near the glitch regime.
TEST(BaroGatePolicy, LegitimateGapAtSpeedIsNotASpike)
{
    BaroGatePolicy gate;
    eval(gate, 0u, 0.0f);

    const auto d = eval(gate, 66000u, 6.6f);  // 66 ms later, +6.6 m
    EXPECT_TRUE(d.fresh);
    EXPECT_FALSE(d.spike);
    EXPECT_TRUE(d.accept);
    EXPECT_NEAR(d.apparent_rate_mps, 100.0f, 1.0f);
}

// An in-band glitch at the normal ~2 ms ODR (e.g. SPI-corrupted low bytes,
// ejection-charge transient) shows a km/s-scale apparent rate and is rejected;
// the reference still advances so the stream recovers immediately after.
TEST(BaroGatePolicy, GlitchAtOdrIsRejectedAndRecovers)
{
    BaroGatePolicy gate;
    eval(gate, 0u, 100.0f);
    eval(gate, 2053u, 100.2f);

    // Glitch: +8 m in one 2 ms sample -> ~3900 m/s apparent.
    const auto g = eval(gate, 4106u, 108.2f);
    EXPECT_TRUE(g.spike);
    EXPECT_FALSE(g.accept);

    // Return to trend: also ~8 m/2 ms relative to the (advanced) reference,
    // so this recovery sample is rejected too — matching the pre-#450
    // "always update reference" behavior that bounds rejection to two samples.
    const auto r1 = eval(gate, 6159u, 100.6f);
    EXPECT_TRUE(r1.spike);

    // Fully recovered on the next sample.
    const auto r2 = eval(gate, 8212u, 100.8f);
    EXPECT_FALSE(r2.spike);
    EXPECT_TRUE(r2.accept);
}

// Small deltas never spike regardless of timestamp pathology (duplicate/near-
// duplicate timestamps hit the dt floor, not a divide-by-zero).
TEST(BaroGatePolicy, DeadbandAndDtFloor)
{
    BaroGatePolicy gate;
    eval(gate, 1000u, 50.0f);

    // 4.9 m in 1 us: below the 5 m deadband -> not a spike even at an
    // absurd apparent rate.
    const auto d = eval(gate, 1001u, 54.9f);
    EXPECT_FALSE(d.spike);
    EXPECT_TRUE(d.accept);
    // dt floored at 100 us -> finite rate.
    EXPECT_LT(d.apparent_rate_mps, 4.9f / 100e-6f + 1.0f);
}

// Transonic lockout: fresh clean samples are not accepted while locked, but
// they DO advance the reference, so the first sample after unlock does not
// read as a spike (pre-#450 behavior, preserved).
TEST(BaroGatePolicy, LockoutBlocksAcceptButTracksReference)
{
    BaroGatePolicy gate;
    eval(gate, 0u, 0.0f);

    // 100 m/s climb, locked for 50 samples (~103 ms of flight).
    uint32_t t = 2053u;
    float    alt = 0.2053f;
    for (int i = 0; i < 50; ++i)
    {
        const auto d = eval(gate, t, alt, /*locked=*/true);
        EXPECT_TRUE(d.fresh);
        EXPECT_FALSE(d.spike);
        EXPECT_FALSE(d.accept);
        t += 2053u;
        alt += 0.2053f;
    }

    // First sample after unlock: normal delta vs the tracked reference.
    const auto d = eval(gate, t, alt);
    EXPECT_TRUE(d.fresh);
    EXPECT_FALSE(d.spike);
    EXPECT_TRUE(d.accept);
}

// Timestamp wrap (micros() rolls over at ~71.6 min): unsigned subtraction
// keeps the pair spacing correct across the wrap.
TEST(BaroGatePolicy, TimestampWrapIsHandled)
{
    BaroGatePolicy gate;
    const uint32_t near_wrap = 0xFFFFFC00u;  // 1024 us before rollover
    eval(gate, near_wrap, 10.0f);

    const uint32_t after_wrap = near_wrap + 2053u;  // wraps past 0
    ASSERT_LT(after_wrap, near_wrap);
    const auto d = eval(gate, after_wrap, 10.2f);
    EXPECT_TRUE(d.fresh);
    EXPECT_FALSE(d.spike);
    EXPECT_TRUE(d.accept);
    EXPECT_NEAR(d.apparent_rate_mps, 0.2f / 2053e-6f, 5.0f);
}

// ---------------------------------------------------------------------------
// Fusion throttle (post-#450 CPU fix): fusing baro at the full 487 Hz ODR
// tripled per-tick EKF cost (554 -> ~1650 us, loop 959 -> ~715/s) because each
// fusion pays the 15x15 covariance update.  The throttle rate-limits ACCEPTED
// samples to the configured interval while spike/freshness tracking still
// inspects every sample.
// ---------------------------------------------------------------------------
TEST(BaroGatePolicy, ThrottleLimitsFusionRateOnFullOdrStream)
{
    BaroGatePolicy gate;
    constexpr double kBmpPeriodUs = 1e6 / 487.0;
    constexpr double kDurationS   = 10.0;

    uint32_t fresh = 0, accepted = 0, spikes = 0;
    const long n = (long)(kDurationS * 1e6 / kBmpPeriodUs);
    for (long i = 0; i < n; ++i)
    {
        const uint32_t t = (uint32_t)(i * kBmpPeriodUs);
        const float alt = 100.0f * (float)(t * 1e-6);  // 100 m/s climb
        const auto d = eval(gate, t, alt, false, kFuseIntervalUs);
        if (d.fresh) fresh++;
        if (d.accept) accepted++;
        if (d.spike) spikes++;
    }

    EXPECT_EQ(fresh, (uint32_t)n);          // every sample still inspected
    EXPECT_EQ(spikes, 0u);
    // 20 ms interval -> 50 Hz.  Sample grid quantizes to the next sample at
    // or beyond each interval boundary, so allow a small band.
    EXPECT_GE(accepted, 480u);
    EXPECT_LE(accepted, 510u);
}

// Spike tracking must remain a consecutive-sample test while throttled: a
// glitch landing between fusions is still measured against its immediate
// predecessor (not the last fused sample), and a glitch on a fusion tick is
// still rejected.
TEST(BaroGatePolicy, ThrottleDoesNotBreakSpikeTracking)
{
    BaroGatePolicy gate;
    // Establish stream at 2 ms cadence; fuse interval 20 ms.
    uint32_t t = 0;
    for (int i = 0; i < 11; ++i)
    {
        eval(gate, t, 100.0f + 0.2f * i, false, kFuseIntervalUs);
        t += 2000u;
    }
    // Next sample is a +8 m glitch mid-throttle-window: apparent rate vs the
    // IMMEDIATE predecessor is ~4000 m/s -> flagged spike even though it
    // would not have been fused anyway.
    const auto g = eval(gate, t, 110.2f, false, kFuseIntervalUs);
    EXPECT_TRUE(g.fresh);
    EXPECT_TRUE(g.spike);
    EXPECT_FALSE(g.accept);
}

// Throttle interval 0 disables the throttle entirely (every clean fresh
// sample fuses) — guards the config escape hatch.
TEST(BaroGatePolicy, ZeroIntervalDisablesThrottle)
{
    BaroGatePolicy gate;
    uint32_t accepted = 0;
    for (int i = 0; i < 100; ++i)
    {
        const auto d = eval(gate, (uint32_t)(i * 2053u), 0.05f * i, false, 0u);
        if (d.accept) accepted++;
    }
    EXPECT_EQ(accepted, 100u);
}

// The throttle clock restarts from the last FUSED sample, so a spike-rejected
// or locked sample does not push the next fusion further out.
TEST(BaroGatePolicy, RejectedSamplesDoNotResetThrottleClock)
{
    BaroGatePolicy gate;
    eval(gate, 0u, 0.0f, false, kFuseIntervalUs);             // fused @ t=0

    // Locked sample at t=10ms: fresh, tracked, not fused.
    const auto locked = eval(gate, 10000u, 1.0f, true, kFuseIntervalUs);
    EXPECT_TRUE(locked.fresh);
    EXPECT_FALSE(locked.accept);

    // Clean sample at t=20ms: 20 ms since last FUSION -> fuses.
    const auto d = eval(gate, 20000u, 2.0f, false, kFuseIntervalUs);
    EXPECT_TRUE(d.accept);
}
