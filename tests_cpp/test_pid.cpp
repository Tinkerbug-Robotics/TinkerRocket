#include <gtest/gtest.h>
#include <cmath>
#include <limits>
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
    // I accumulates Ki * error * dt = 1.0 * 5.0 * 0.01, ten times = 0.5
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

// ---------- #386: integrator ACCUMULATOR clamp ----------
//
// The I output was always clamped, but the accumulator grew unbounded during
// a long saturated stretch; after the error reversed, all that surplus had to
// integrate back down before the command moved at all — fins held hard-over
// long past reversal.  The accumulator is now bounded at the value that
// exactly saturates the output — [MIN, MAX] itself, since it holds the I term
// in output units — so recovery begins on the first post-reversal sample.

TEST_F(PIDTest, IntegratorRecoversPromptlyAfterLongSaturation) {
    // I-only controller: Ki=0.5, output cap ±10 -> I-term cap ±10.
    TR_PID i_pid(0.0f, KI, 0.0f, MAX, MIN);
    i_pid.computePID(0.0f, 0.0f, DT);  // init

    // 60 simulated seconds of hard +10 error: unclamped the I term would
    // reach 3000; clamped it stops at MAX.
    for (int i = 0; i < 6000; i++) {
        i_pid.computePID(10.0f, 0.0f, DT);
    }

    // Error reverses to -10.  Pre-#386, unwinding the surplus took ~59800
    // steps (~10 minutes of flight) with the output pinned at MAX the whole
    // time.  Now the output must leave the +MAX rail at once and cross zero
    // in the steps it takes Ki*|error|*dt = 0.05 per step to walk the I term
    // down from +10: ~200.
    int steps_to_leave_rail = -1, steps_to_negative = -1;
    for (int i = 0; i < 1000; i++) {
        float out = i_pid.computePID(-10.0f, 0.0f, DT);
        if (steps_to_leave_rail < 0 && out < MAX - 1e-4f) steps_to_leave_rail = i;
        if (steps_to_negative < 0 && out < 0.0f) { steps_to_negative = i; break; }
    }
    ASSERT_GE(steps_to_leave_rail, 0) << "output never left the +MAX rail";
    EXPECT_LE(steps_to_leave_rail, 3);
    ASSERT_GE(steps_to_negative, 0) << "output never crossed zero";
    EXPECT_LE(steps_to_negative, 450);
}

TEST_F(PIDTest, AccumulatorClampPreservesSteadyState) {
    // Below saturation the clamp must be inert: a small steady error
    // integrates exactly as before.
    TR_PID i_pid(0.0f, KI, 0.0f, MAX, MIN);
    i_pid.computePID(0.0f, 0.0f, DT);
    float out = 0.0f;
    for (int i = 0; i < 100; i++) out = i_pid.computePID(1.0f, 0.0f, DT);
    // 100 steps of error 1.0 at dt 0.01 -> accumulator 1.0 -> I = 0.5.
    EXPECT_NEAR(out, 0.5f, 1e-4f);
}

TEST_F(PIDTest, KiZeroAccumulatesNothingSoEnablingKiDoesNotBump) {
    // Ki = 0 integrates nothing — the increment is Ki * error * dt — so a
    // minute of hard error with the I term off leaves nothing behind for a
    // later setKi() to release.  (When the accumulator held the bare error
    // integral, enabling Ki put a minute's worth of it on the fins at once,
    // clamped only because #386 bounded it.)
    TR_PID pid_rt(0.0f, 0.0f, 0.0f, MAX, MIN);
    pid_rt.computePID(0.0f, 0.0f, DT);
    for (int i = 0; i < 6000; i++) pid_rt.computePID(10.0f, 0.0f, DT);

    pid_rt.setKi(KI);
    EXPECT_NEAR(pid_rt.computePID(0.0f, 0.0f, DT), 0.0f, 1e-6f);
    // And it integrates from zero: one reversed step is already negative.
    EXPECT_LT(pid_rt.computePID(-10.0f, 0.0f, DT), 0.0f);
}

// ---------- The I term is continuous when Ki changes ----------
//
// The accumulator holds Ki * error * dt summed — the I term in output units —
// not the bare error integral multiplied by whatever Ki is current.  With a
// fixed Ki those are the same controller; the roll V² gain schedule changes Ki
// on every tick, and under Ki * sum(e*dt) a trim offset the integrator had
// learned grew with Ki as the rocket slowed (up to 3x), which the loop then
// had to unwind all through coast: a standing roll error of ~10 dps on the
// 67 mm testbed's F67 flights, ~26 on a G80.

namespace {
// Drive an I-only PID at zero error for n steps while Ki ramps linearly from
// ki0 to ki1 — the gain schedule on a decelerating rocket whose loop is
// already holding its trim — and return the last output.
float rampKiAtZeroError(TR_PID &pid, float ki0, float ki1, int n) {
    float out = 0.0f;
    for (int i = 1; i <= n; i++) {
        pid.setKi(ki0 + (ki1 - ki0) * (float)i / (float)n);
        out = pid.computePID(0.0f, 0.0f, DT);
    }
    return out;
}
}  // namespace

TEST_F(PIDTest, KiRampHoldsALearnedTrimInsteadOfScalingIt) {
    TR_PID i_pid(0.0f, 0.06f, 0.0f, 20.0f, -20.0f);
    i_pid.computePID(0.0f, 0.0f, DT);
    // Learn a 3.6 deg trim: 0.06 * 60 dps * 0.01 s * 100 steps.
    float trim = 0.0f;
    for (int i = 0; i < 100; i++) trim = i_pid.computePID(60.0f, 0.0f, DT);
    ASSERT_NEAR(trim, 3.6f, 1e-3f);

    // Coast: the schedule takes Ki from 1x to its 3x cap.  At zero error the
    // I term must not move — Ki * sum(e*dt) would have tripled it to 10.8.
    EXPECT_NEAR(rampKiAtZeroError(i_pid, 0.06f, 0.18f, 500), trim, 1e-4f);
}

TEST_F(PIDTest, KiChangeSetsOnlyTheRateFromThereOn) {
    TR_PID i_pid(0.0f, KI, 0.0f, MAX, MIN);
    i_pid.computePID(0.0f, 0.0f, DT);
    for (int i = 0; i < 100; i++) i_pid.computePID(1.0f, 0.0f, DT);   // I = 0.5

    i_pid.setKi(3.0f * KI);
    EXPECT_NEAR(i_pid.computePID(0.0f, 0.0f, DT), 0.5f, 1e-5f)
        << "the I term jumped when Ki changed";
    // Ten more steps of error 1.0 now add 1.5 * 1.0 * 0.01 each.
    float out = 0.0f;
    for (int i = 0; i < 10; i++) out = i_pid.computePID(1.0f, 0.0f, DT);
    EXPECT_NEAR(out, 0.5f + 0.15f, 1e-4f);
}

TEST_F(PIDTest, FixedKiIsUnchangedFromKiTimesTheErrorIntegral) {
    // Every PID that does not schedule its gains — the ground-test roll PID,
    // roll control with the schedule off — must behave exactly as before.
    // Reference: Ki * sum(e*dt) in double, with the old clamp and the
    // separation gate, over an error sequence that saturates and reverses.
    const float ki = 0.3f, sep = 25.0f;
    TR_PID i_pid(0.0f, ki, 0.0f, MAX, MIN);
    i_pid.setIntegralSeparationThreshold(sep);
    i_pid.computePID(0.0f, 0.0f, DT);
    double acc = 0.0;
    for (int i = 0; i < 3000; i++) {
        const float e = 30.0f * std::sin(0.013f * (float)i) + 4.0f;   // gated above 25
        const float out = i_pid.computePID(e, 0.0f, DT);
        if (std::fabs(e) <= sep) acc += (double)e * DT;
        acc = std::fmin(std::fmax(acc, MIN / ki), MAX / ki);
        ASSERT_NEAR(out, (float)(ki * acc), 2e-4f) << "step " << i;
    }
}

TEST_F(PIDTest, SetKiZeroClearsTheIntegralTerm) {
    // Ki = 0 is "no integral action": the I term goes to zero at once, as it
    // did when it was Ki * sum(e*dt), rather than freezing at its last value.
    TR_PID i_pid(0.0f, KI, 0.0f, MAX, MIN);
    i_pid.computePID(0.0f, 0.0f, DT);
    for (int i = 0; i < 100; i++) i_pid.computePID(1.0f, 0.0f, DT);   // I = 0.5
    i_pid.setKi(0.0f);
    EXPECT_NEAR(i_pid.computePID(0.0f, 0.0f, DT), 0.0f, 1e-6f);
}

TEST_F(PIDTest, OutputLimitChangeClampsTheIntegralTerm) {
    TR_PID i_pid(0.0f, KI, 0.0f, MAX, MIN);
    i_pid.computePID(0.0f, 0.0f, DT);
    for (int i = 0; i < 1600; i++) i_pid.computePID(1.0f, 0.0f, DT);  // I = 8
    i_pid.setMaxCmd(5.0f);
    EXPECT_NEAR(i_pid.computePID(0.0f, 0.0f, DT), 5.0f, 1e-5f);
    // Clamped, not merely hidden: one reversed step comes straight off 5.
    EXPECT_LT(i_pid.computePID(-1.0f, 0.0f, DT), 5.0f);
}

TEST_F(PIDTest, NonFiniteGainOrErrorDoesNotPoisonTheIntegralTerm) {
    // Ki now lives inside the state, so a NaN Ki or error folded in once
    // would stay until the next reset.  The increment is skipped instead.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    TR_PID i_pid(0.0f, KI, 0.0f, MAX, MIN);
    i_pid.computePID(0.0f, 0.0f, DT);
    for (int i = 0; i < 100; i++) i_pid.computePID(1.0f, 0.0f, DT);   // I = 0.5

    i_pid.setKi(nan);
    i_pid.computePID(1.0f, 0.0f, DT);
    i_pid.setKi(KI);
    EXPECT_NEAR(i_pid.computePID(0.0f, 0.0f, DT), 0.5f, 1e-5f);

    i_pid.computePID(nan, 0.0f, DT);          // this tick's output is NaN (P)
    EXPECT_NEAR(i_pid.computePID(0.0f, 0.0f, DT), 0.5f, 1e-5f);

    i_pid.setKi(std::numeric_limits<float>::infinity());
    i_pid.computePID(0.0f, 0.0f, DT);         // inf * 0 = NaN
    i_pid.setKi(KI);
    EXPECT_NEAR(i_pid.computePID(0.0f, 0.0f, DT), 0.5f, 1e-5f);
}
