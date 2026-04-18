#include <gtest/gtest.h>
#include "TR_PID.h"

// ---------- Helpers ----------

static constexpr float KP   = 1.0f;
static constexpr float KI   = 0.5f;
static constexpr float KD   = 0.1f;
static constexpr float MAX  = 10.0f;
static constexpr float MIN  = -10.0f;
static constexpr float DT   = 0.01f; // 100 Hz

class PIDTest : public ::testing::Test {
protected:
    TR_PID pid{KP, KI, KD, MAX, MIN};
};

// ---------- Tests ----------

TEST_F(PIDTest, ZeroError_ZeroOutput) {
    pid.computePID(5.0f, 5.0f, DT); // first call always returns 0
    float out = pid.computePID(5.0f, 5.0f, DT);
    EXPECT_NEAR(out, 0.0f, 1e-6f);
}

TEST_F(PIDTest, FirstCall_ReturnsZero) {
    float out = pid.computePID(10.0f, 0.0f, DT);
    EXPECT_EQ(out, 0.0f);
}

TEST_F(PIDTest, ProportionalOnly) {
    TR_PID p_only(2.0f, 0.0f, 0.0f, MAX, MIN);
    p_only.computePID(0.0f, 0.0f, DT); // init
    float out = p_only.computePID(3.0f, 0.0f, DT);
    // P = Kp * error = 2.0 * 3.0 = 6.0
    // D-on-measurement: measurement didn't change -> D = 0
    EXPECT_NEAR(out, 6.0f, 1e-5f);
}

TEST_F(PIDTest, IntegralAccumulation) {
    TR_PID i_only(0.0f, 1.0f, 0.0f, MAX, MIN);
    i_only.computePID(0.0f, 0.0f, DT); // init

    // Accumulate error of 5.0 over 10 steps at DT=0.01
    float out = 0.0f;
    for (int i = 0; i < 10; i++) {
        out = i_only.computePID(5.0f, 0.0f, DT);
    }
    // cumulative_error = 5.0 * 0.01 * 10 = 0.5
    // I = Ki * cumulative_error = 1.0 * 0.5 = 0.5
    EXPECT_NEAR(out, 0.5f, 1e-4f);
}

TEST_F(PIDTest, IntegralAntiWindup) {
    TR_PID i_windup(0.0f, 100.0f, 0.0f, MAX, MIN);
    i_windup.computePID(0.0f, 0.0f, DT); // init

    // Large error over many steps -> integral should saturate at MAX
    for (int i = 0; i < 1000; i++) {
        i_windup.computePID(100.0f, 0.0f, DT);
    }
    float out = i_windup.computePID(100.0f, 0.0f, DT);
    EXPECT_LE(out, MAX);
    EXPECT_GE(out, MIN);
}

TEST_F(PIDTest, DerivativeOnMeasurement) {
    // D-on-measurement: a step change in setpoint should NOT cause a D kick.
    // Only changes in measurement cause D response.
    TR_PID d_only(0.0f, 0.0f, 1.0f, MAX, MIN);
    d_only.computePID(0.0f, 0.0f, DT); // init

    // Step setpoint from 0 to 100, measurement stays at 0
    float out = d_only.computePID(100.0f, 0.0f, DT);
    // D = -Kd * (actual - last_measurement) / dt = -1.0 * (0 - 0) / 0.01 = 0
    EXPECT_NEAR(out, 0.0f, 1e-6f);

    // Now measurement changes: D should respond
    out = d_only.computePID(100.0f, 5.0f, DT);
    // D = -1.0 * (5.0 - 0.0) / 0.01 = -500.0, clamped to MIN = -10
    EXPECT_NEAR(out, MIN, 1e-5f);
}

TEST_F(PIDTest, OutputClamping) {
    TR_PID clamped(100.0f, 0.0f, 0.0f, 5.0f, -5.0f);
    clamped.computePID(0.0f, 0.0f, DT); // init

    float out = clamped.computePID(100.0f, 0.0f, DT);
    EXPECT_NEAR(out, 5.0f, 1e-5f);

    out = clamped.computePID(-100.0f, 0.0f, DT);
    EXPECT_NEAR(out, -5.0f, 1e-5f);
}

TEST_F(PIDTest, Reset_ClearsState) {
    pid.computePID(10.0f, 0.0f, DT); // init
    pid.computePID(10.0f, 0.0f, DT); // accumulate
    pid.reset();

    // After reset, next call should behave like first call (return 0)
    float out = pid.computePID(10.0f, 0.0f, DT);
    EXPECT_EQ(out, 0.0f);
}

TEST_F(PIDTest, ResetIntegral_PreservesD) {
    pid.computePID(0.0f, 0.0f, DT); // init

    // Accumulate some integral
    for (int i = 0; i < 100; i++) {
        pid.computePID(5.0f, 0.0f, DT);
    }

    pid.resetIntegral();

    // After resetIntegral, integral term is zero but D-term still works
    // Measurement jumps from 0 to 5 -> D responds
    float out = pid.computePID(5.0f, 5.0f, DT);
    // P = 0, I = 0, D = -Kd * (5 - 0) / 0.01 -> clamped
    EXPECT_NE(out, 0.0f); // D-term should produce non-zero output
}

TEST_F(PIDTest, NegativeDt_ReturnsZero) {
    pid.computePID(10.0f, 0.0f, DT);  // init (returns 0)
    pid.computePID(10.0f, 0.0f, DT);  // normal call

    // Negative dt should return 0 and not corrupt state
    float out = pid.computePID(10.0f, 0.0f, -0.01f);
    EXPECT_EQ(out, 0.0f);

    // Subsequent normal call should still work
    out = pid.computePID(10.0f, 0.0f, DT);
    EXPECT_NE(out, 0.0f);
}

TEST_F(PIDTest, GainSetters) {
    TR_PID p(1.0f, 0.0f, 0.0f, MAX, MIN);
    p.computePID(0.0f, 0.0f, DT); // init

    float out1 = p.computePID(5.0f, 0.0f, DT);
    // P = 1.0 * 5.0 = 5.0
    EXPECT_NEAR(out1, 5.0f, 1e-4f);

    p.setKp(2.0f);
    float out2 = p.computePID(5.0f, 0.0f, DT);
    // P = 2.0 * 5.0 = 10.0, but measurement unchanged so D ~ 0
    EXPECT_NEAR(out2, MAX, 1e-4f); // clamped at 10
}

TEST_F(PIDTest, DFilter_DisabledByDefault_MatchesRawDerivative) {
    // With filter off (default), D term equals raw backward-difference.
    // Impulse measurement: step from 0 to 1.0 in one sample should give
    // D = -Kd * 1.0 / DT = -0.1 * 1.0 / 0.01 = -10 (clamped to MIN).
    TR_PID d_only(0.0f, 0.0f, 0.1f, MAX, MIN);
    d_only.computePID(0.0f, 0.0f, DT); // init, last_measurement = 0
    float out = d_only.computePID(0.0f, 1.0f, DT);
    EXPECT_NEAR(out, MIN, 1e-4f); // clamps
}

TEST_F(PIDTest, DFilter_Enabled_AttenuatesSingleSampleSpike) {
    // Same impulse as above, with LPF at 10 Hz and dt=0.01 (100 Hz).
    // alpha = dt / (dt + 1/(2*pi*10)) = 0.01 / (0.01 + 0.01592) = ~0.386
    // d_filtered after one step = alpha * D_raw = 0.386 * -10 = -3.86 approx
    // (unclamped math — output then clamped to [MIN, MAX]).
    TR_PID d_only(0.0f, 0.0f, 0.1f, MAX, MIN);
    d_only.setDerivativeFilterCutoffHz(10.0f);
    d_only.computePID(0.0f, 0.0f, DT); // init
    float out = d_only.computePID(0.0f, 1.0f, DT);
    // Filter attenuates the spike - should NOT reach the MIN clamp anymore
    EXPECT_GT(out, MIN + 1.0f);
    EXPECT_LT(out, 0.0f);         // still negative (derivative is negative)
    EXPECT_NEAR(out, -3.86f, 0.1f); // matches first-order IIR math
}

TEST_F(PIDTest, DFilter_WhiteNoise_ReducesStd) {
    // Feed both filters (off vs on) the same pseudo-random measurement
    // series; the filtered output should have much smaller std.
    auto run = [](bool filtered) {
        TR_PID p(0.0f, 0.0f, 0.1f, 1e9f, -1e9f); // unclamped
        if (filtered) p.setDerivativeFilterCutoffHz(10.0f);
        p.computePID(0.0f, 0.0f, DT);
        unsigned rng = 0xC0FFEEu;
        float sum = 0, sumsq = 0; int n = 0;
        for (int i = 0; i < 500; ++i) {
            rng = rng * 1103515245u + 12345u;
            float noise = (int(rng >> 8) % 1000) / 1000.0f - 0.5f; // ±0.5
            float out = p.computePID(0.0f, noise, DT);
            sum += out; sumsq += out*out; n++;
        }
        float mean = sum/n;
        float var = sumsq/n - mean*mean;
        return var;
    };
    float var_raw = run(false);
    float var_filt = run(true);
    // Filter should cut variance by at least 4x at 10 Hz vs 100 Hz sampling.
    EXPECT_LT(var_filt * 4.0f, var_raw);
}

TEST_F(PIDTest, DFilter_Reset_ClearsFilterState) {
    // After reset, filter state should be cleared so the first post-reset
    // call doesn't leak state from before.
    TR_PID p(0.0f, 0.0f, 0.1f, MAX, MIN);
    p.setDerivativeFilterCutoffHz(10.0f);
    p.computePID(0.0f, 0.0f, DT);
    for (int i = 0; i < 20; i++) {
        p.computePID(0.0f, 1.0f, DT); // drive filter up
    }
    p.reset();
    // First call after reset returns 0 (the first_call path) — verify.
    float out = p.computePID(0.0f, 0.0f, DT);
    EXPECT_EQ(out, 0.0f);
    // Second call: measurement unchanged, so D should be 0 too.
    out = p.computePID(0.0f, 0.0f, DT);
    EXPECT_NEAR(out, 0.0f, 1e-6f);
}
