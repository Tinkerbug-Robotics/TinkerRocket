// Tests for the recovery-deployment detector, exercising the REAL firmware
// logic in tinkerrocket-idf/components/TR_KinematicChecks/DeploymentDetector.h
// (no hand-copied mirror).  The flight loop calls the same
// tr::deploymentDetectStep() once per iteration; these tests are the single
// source of truth for its behavior.
//
// The detector is threshold-driven, so a suite pinned to one config could not
// tell a real threshold dependency from a hardcoded constant.  Every behavioral
// case below either sweeps the tunable it is about, or asserts a relationship
// that must hold across configs (see the *_Swept and *_Param tests).

#include <gtest/gtest.h>
#include <cstdint>
#include <vector>
#include "DeploymentDetector.h"

namespace {

constexpr float G = 9.80665f;

// Mirror of config::DEPLOY_* in
// tinkerrocket-idf/projects/flight_computer/main/config.h.
tr::DeploymentConfig baseConfig()
{
    tr::DeploymentConfig c{};
    c.shock_ms2         = 8.0f * G;
    c.shock_count       = 3;
    c.baro_step_m       = 3.0f;
    c.coincidence_ms    = 250;
    c.ballistic_mps     = 18.0f;
    c.canopy_mps        = 10.0f;
    c.canopy_count      = 200;
    c.launch_lockout_ms = 1000;
    return c;
}

// One flight-loop iteration's worth of inputs.
struct Sample {
    float acc_ms2      = 1.0f * G;
    float palt_m       = 0.0f;
    bool  new_baro     = true;
    float palt_rate    = 0.0f;
    bool  burnout      = true;
};

// Drives the detector at 1 kHz (1 ms per iteration) from t_start_ms, returning
// the t_since_launch_ms of the latch or 0 if it never fired.
uint32_t run(tr::DeploymentState& st, const tr::DeploymentConfig& cfg,
             uint32_t t_start_ms, const std::vector<Sample>& samples)
{
    uint32_t fired_at = 0;
    uint32_t t = t_start_ms;
    for (const Sample& s : samples) {
        if (tr::deploymentDetectStep(st, cfg, t, s.acc_ms2, s.palt_m,
                                     s.new_baro, s.palt_rate, s.burnout)
            && fired_at == 0) {
            fired_at = t;
        }
        ++t;
    }
    return fired_at;
}

// Coasting: 1 g of drag decel, descending gently, altitude tracking the rate.
std::vector<Sample> coast(size_t n, float palt_start, float rate_mps)
{
    std::vector<Sample> out;
    out.reserve(n);
    float palt = palt_start;
    for (size_t i = 0; i < n; ++i) {
        Sample s;
        s.acc_ms2   = 1.0f * G;
        s.palt_m    = palt;
        s.palt_rate = rate_mps;
        out.push_back(s);
        palt += rate_mps * 0.001f;   // 1 ms per iteration
    }
    return out;
}

// --------------------------------------------------------------------------
// FAST path: shock AND baro step, coincident
// --------------------------------------------------------------------------

TEST(DeploymentDetector, FastPath_ShockPlusBaroStep_Latches)
{
    const auto cfg = baseConfig();
    tr::DeploymentState st;

    auto s = coast(50, 300.0f, -5.0f);
    // Ejection: a 40 g shock and a 12 m indicated-altitude step, together.
    for (int i = 0; i < 5; ++i) {
        Sample e;
        e.acc_ms2   = 40.0f * G;
        e.palt_m    = 300.0f + 12.0f;
        e.palt_rate = -5.0f;
        s.push_back(e);
    }
    const uint32_t fired = run(st, cfg, 5000, s);
    EXPECT_NE(fired, 0u);
    EXPECT_TRUE(st.detected);
    EXPECT_EQ(st.reason & tr::kDeployReasonShockBaro, tr::kDeployReasonShockBaro);
    EXPECT_EQ(st.detected_ms, fired);
}

TEST(DeploymentDetector, FastPath_ShockAlone_DoesNotLatch)
{
    const auto cfg = baseConfig();
    tr::DeploymentState st;

    auto s = coast(50, 300.0f, -5.0f);
    for (int i = 0; i < 500; ++i) {          // sustained hard shock, no baro step
        Sample e;
        e.acc_ms2   = 60.0f * G;
        e.palt_m    = 300.0f;
        e.palt_rate = -5.0f;
        s.push_back(e);
    }
    EXPECT_EQ(run(st, cfg, 5000, s), 0u);
    EXPECT_FALSE(st.detected);
}

TEST(DeploymentDetector, FastPath_BaroStepAlone_DoesNotLatch)
{
    const auto cfg = baseConfig();
    tr::DeploymentState st;

    auto s = coast(50, 300.0f, -5.0f);
    for (int i = 0; i < 500; ++i) {          // baro sawtooth, no shock
        Sample e;
        e.acc_ms2   = 1.0f * G;
        e.palt_m    = (i % 2 == 0) ? 320.0f : 300.0f;
        e.palt_rate = -5.0f;
        s.push_back(e);
    }
    EXPECT_EQ(run(st, cfg, 5000, s), 0u);
    EXPECT_FALSE(st.detected);
}

TEST(DeploymentDetector, FastPath_NonCoincidentFeatures_DoNotLatch)
{
    auto cfg = baseConfig();
    cfg.coincidence_ms = 100;
    tr::DeploymentState st;

    auto s = coast(50, 300.0f, -5.0f);
    for (int i = 0; i < 5; ++i) {            // baro step first
        Sample e; e.acc_ms2 = 1.0f * G; e.palt_m = 320.0f; e.palt_rate = -5.0f;
        s.push_back(e);
    }
    auto gap = coast(400, 320.0f, -5.0f);    // 400 ms of quiet, well past the window
    s.insert(s.end(), gap.begin(), gap.end());
    for (int i = 0; i < 5; ++i) {            // shock much later
        Sample e; e.acc_ms2 = 40.0f * G; e.palt_m = 318.0f; e.palt_rate = -5.0f;
        s.push_back(e);
    }
    EXPECT_EQ(run(st, cfg, 5000, s), 0u);
    EXPECT_FALSE(st.detected);
}

// The coincidence window is a real parameter, not a constant: the same
// separated pair must latch when the window is wide enough to span it.
TEST(DeploymentDetector, FastPath_CoincidenceWindow_Swept)
{
    for (uint32_t window : {50u, 150u, 300u, 600u}) {
        auto cfg = baseConfig();
        cfg.coincidence_ms = window;
        tr::DeploymentState st;

        constexpr uint32_t kSeparationMs = 200;
        auto s = coast(50, 300.0f, -5.0f);
        for (int i = 0; i < 5; ++i) {
            Sample e; e.acc_ms2 = 1.0f * G; e.palt_m = 320.0f; e.palt_rate = -5.0f;
            s.push_back(e);
        }
        auto gap = coast(kSeparationMs, 320.0f, -5.0f);
        s.insert(s.end(), gap.begin(), gap.end());
        for (int i = 0; i < 5; ++i) {
            Sample e; e.acc_ms2 = 40.0f * G; e.palt_m = 320.0f; e.palt_rate = -5.0f;
            s.push_back(e);
        }
        const bool expect_fire = (window > kSeparationMs);
        EXPECT_EQ(st.detected, false) << "window=" << window;
        run(st, cfg, 5000, s);
        EXPECT_EQ(st.detected, expect_fire)
            << "coincidence_ms=" << window << " separation=" << kSeparationMs;
    }
}

// The shock threshold must actually gate: the SAME acceleration is a shock
// under a low threshold and is not under a high one.
TEST(DeploymentDetector, ShockThreshold_Swept)
{
    constexpr float kShockG = 20.0f;
    for (float thresh_g : {5.0f, 10.0f, 25.0f, 50.0f}) {
        auto cfg = baseConfig();
        cfg.shock_ms2 = thresh_g * G;
        tr::DeploymentState st;

        auto s = coast(50, 300.0f, -5.0f);
        for (int i = 0; i < 10; ++i) {
            Sample e;
            e.acc_ms2   = kShockG * G;
            e.palt_m    = 315.0f;
            e.palt_rate = -5.0f;
            s.push_back(e);
        }
        run(st, cfg, 5000, s);
        EXPECT_EQ(st.detected, thresh_g <= kShockG)
            << "shock threshold " << thresh_g << " g vs " << kShockG << " g event";
    }
}

// Likewise the baro-step threshold.
TEST(DeploymentDetector, BaroStepThreshold_Swept)
{
    constexpr float kStepM = 6.0f;
    for (float thresh_m : {1.0f, 5.0f, 8.0f, 20.0f}) {
        auto cfg = baseConfig();
        cfg.baro_step_m = thresh_m;
        tr::DeploymentState st;

        auto s = coast(50, 300.0f, -5.0f);
        for (int i = 0; i < 10; ++i) {
            Sample e;
            e.acc_ms2   = 40.0f * G;
            e.palt_m    = 300.0f + kStepM;
            e.palt_rate = -5.0f;
            s.push_back(e);
        }
        run(st, cfg, 5000, s);
        EXPECT_EQ(st.detected, thresh_m <= kStepM)
            << "baro step threshold " << thresh_m << " m vs " << kStepM << " m event";
    }
}

TEST(DeploymentDetector, ShockCount_RequiresConsecutiveSamples)
{
    auto cfg = baseConfig();
    cfg.shock_count = 5;
    tr::DeploymentState st;

    auto s = coast(50, 300.0f, -5.0f);
    for (int burst = 0; burst < 20; ++burst) {   // 4-on / 1-off, never 5 in a row
        for (int i = 0; i < 4; ++i) {
            Sample e; e.acc_ms2 = 40.0f * G; e.palt_m = 320.0f; e.palt_rate = -5.0f;
            s.push_back(e);
        }
        Sample q; q.acc_ms2 = 1.0f * G; q.palt_m = 320.0f; q.palt_rate = -5.0f;
        s.push_back(q);
    }
    EXPECT_EQ(run(st, cfg, 5000, s), 0u);
    EXPECT_FALSE(st.detected);
}

// --------------------------------------------------------------------------
// SLOW path: descent-rate collapse
// --------------------------------------------------------------------------

TEST(DeploymentDetector, SlowPath_DescentCollapse_Latches)
{
    const auto cfg = baseConfig();
    tr::DeploymentState st;

    auto s = coast(2000, 400.0f, -30.0f);              // ballistic drogue-less fall
    auto under = coast(cfg.canopy_count + 50, 340.0f, -5.0f);   // canopy grabs
    s.insert(s.end(), under.begin(), under.end());

    const uint32_t fired = run(st, cfg, 5000, s);
    EXPECT_NE(fired, 0u);
    EXPECT_EQ(st.reason & tr::kDeployReasonDescentCollapse,
              tr::kDeployReasonDescentCollapse);
}

TEST(DeploymentDetector, SlowPath_NeverBallistic_DoesNotLatch)
{
    const auto cfg = baseConfig();
    tr::DeploymentState st;

    // Gentle descent that never reaches ballistic_mps — the collapse test must
    // stay disarmed even though the rate is below canopy_mps the whole time.
    auto s = coast(5000, 400.0f, -6.0f);
    EXPECT_EQ(run(st, cfg, 5000, s), 0u);
    EXPECT_FALSE(st.ballistic);
    EXPECT_FALSE(st.detected);
}

TEST(DeploymentDetector, SlowPath_BallisticOnly_DoesNotLatch)
{
    const auto cfg = baseConfig();
    tr::DeploymentState st;

    auto s = coast(5000, 900.0f, -45.0f);   // still falling fast: no canopy
    EXPECT_EQ(run(st, cfg, 5000, s), 0u);
    EXPECT_TRUE(st.ballistic);
    EXPECT_FALSE(st.detected);
}

TEST(DeploymentDetector, SlowPath_CanopyCount_RequiresSustained)
{
    auto cfg = baseConfig();
    cfg.canopy_count = 300;
    tr::DeploymentState st;

    auto s = coast(2000, 900.0f, -40.0f);
    for (int burst = 0; burst < 20; ++burst) {      // 100 ms slow, then fast again
        auto slow = coast(100, 500.0f, -4.0f);
        auto fast = coast(50, 500.0f, -40.0f);
        s.insert(s.end(), slow.begin(), slow.end());
        s.insert(s.end(), fast.begin(), fast.end());
    }
    EXPECT_EQ(run(st, cfg, 5000, s), 0u);
    EXPECT_FALSE(st.detected);
}

// canopy_count is a real dwell requirement: the same 250 ms of canopy-rate
// descent latches under a short dwell and does not under a long one.
TEST(DeploymentDetector, SlowPath_CanopyCount_Swept)
{
    constexpr size_t kSlowMs = 250;
    for (uint16_t dwell : {uint16_t(50), uint16_t(200), uint16_t(400), uint16_t(1000)}) {
        auto cfg = baseConfig();
        cfg.canopy_count = dwell;
        tr::DeploymentState st;

        auto s = coast(1500, 900.0f, -40.0f);
        auto slow = coast(kSlowMs, 800.0f, -4.0f);
        s.insert(s.end(), slow.begin(), slow.end());

        run(st, cfg, 5000, s);
        EXPECT_EQ(st.detected, dwell <= kSlowMs) << "canopy_count=" << dwell;
    }
}

// A canopy_count of 0 must still mean "one slow sample", never "fire the
// instant the ballistic flag arms while still falling at full speed".
TEST(DeploymentDetector, SlowPath_ZeroCanopyCount_StillNeedsASlowSample)
{
    auto cfg = baseConfig();
    cfg.canopy_count = 0;
    tr::DeploymentState st;

    auto s = coast(2000, 900.0f, -40.0f);      // ballistic arms, never slows
    EXPECT_EQ(run(st, cfg, 5000, s), 0u);
    EXPECT_TRUE(st.ballistic);
    EXPECT_FALSE(st.detected);
}

// --------------------------------------------------------------------------
// Gating
// --------------------------------------------------------------------------

TEST(DeploymentDetector, UnderThrust_NoDetection)
{
    const auto cfg = baseConfig();
    tr::DeploymentState st;

    // Boost: sustained high g and a fast-climbing (noisy) baro, but burnout has
    // not latched — nothing may fire.
    std::vector<Sample> s;
    for (int i = 0; i < 3000; ++i) {
        Sample e;
        e.acc_ms2   = 15.0f * G;
        e.palt_m    = (i % 2 == 0) ? 100.0f + i * 0.2f : 100.0f + i * 0.2f + 10.0f;
        e.palt_rate = 200.0f;
        e.burnout   = false;
        s.push_back(e);
    }
    EXPECT_EQ(run(st, cfg, 0, s), 0u);
    EXPECT_FALSE(st.detected);
}

TEST(DeploymentDetector, LaunchLockout_Swept)
{
    constexpr uint32_t kEventAtMs = 1500;
    for (uint32_t lockout : {500u, 1000u, 2000u, 4000u}) {
        auto cfg = baseConfig();
        cfg.launch_lockout_ms = lockout;
        tr::DeploymentState st;

        // Quiet up to kEventAtMs, then an unambiguous ejection signature.
        auto s = coast(kEventAtMs, 300.0f, -5.0f);
        for (int i = 0; i < 20; ++i) {
            Sample e; e.acc_ms2 = 40.0f * G; e.palt_m = 320.0f; e.palt_rate = -5.0f;
            s.push_back(e);
        }
        run(st, cfg, 0, s);
        EXPECT_EQ(st.detected, lockout < kEventAtMs) << "launch_lockout_ms=" << lockout;
    }
}

// Boost-phase shock must not carry a partial count across the gate: the very
// first armed iteration cannot inherit credit earned under thrust.
TEST(DeploymentDetector, ShockCountDoesNotCarryAcrossTheGate)
{
    auto cfg = baseConfig();
    cfg.shock_count       = 50;
    cfg.launch_lockout_ms = 100;
    tr::DeploymentState st;

    std::vector<Sample> s;
    for (int i = 0; i < 200; ++i) {          // hard shaking, still under thrust
        Sample e;
        e.acc_ms2 = 40.0f * G; e.palt_m = 100.0f; e.palt_rate = 100.0f;
        e.burnout = false;
        s.push_back(e);
    }
    for (int i = 0; i < 49; ++i) {           // burnout: 49 shocks, one short of 50
        Sample e;
        e.acc_ms2 = 40.0f * G; e.palt_m = 120.0f; e.palt_rate = -5.0f;
        s.push_back(e);
    }
    EXPECT_EQ(run(st, cfg, 0, s), 0u);
    EXPECT_FALSE(st.shock_seen);
    EXPECT_FALSE(st.detected);
}

// --------------------------------------------------------------------------
// Latch semantics
// --------------------------------------------------------------------------

TEST(DeploymentDetector, RisingEdgeFiresExactlyOnce)
{
    const auto cfg = baseConfig();
    tr::DeploymentState st;

    auto s = coast(50, 300.0f, -5.0f);
    for (int i = 0; i < 2000; ++i) {         // ejection signature held forever
        Sample e; e.acc_ms2 = 40.0f * G; e.palt_m = 320.0f; e.palt_rate = -5.0f;
        s.push_back(e);
    }

    int edges = 0;
    uint32_t t = 5000;
    for (const Sample& x : s) {
        if (tr::deploymentDetectStep(st, cfg, t, x.acc_ms2, x.palt_m,
                                     x.new_baro, x.palt_rate, x.burnout)) {
            ++edges;
        }
        ++t;
    }
    EXPECT_EQ(edges, 1);
    EXPECT_TRUE(st.detected);
}

TEST(DeploymentDetector, ResetClearsTheLatchForANewFlight)
{
    const auto cfg = baseConfig();
    tr::DeploymentState st;

    auto s = coast(50, 300.0f, -5.0f);
    for (int i = 0; i < 20; ++i) {
        Sample e; e.acc_ms2 = 40.0f * G; e.palt_m = 320.0f; e.palt_rate = -5.0f;
        s.push_back(e);
    }
    EXPECT_NE(run(st, cfg, 5000, s), 0u);

    tr::deploymentReset(st);
    EXPECT_FALSE(st.detected);
    EXPECT_EQ(st.reason, 0);
    EXPECT_EQ(st.detected_ms, 0u);
    EXPECT_FALSE(st.ballistic);
    EXPECT_FALSE(st.shock_seen);
    EXPECT_FALSE(st.baro_step_seen);

    // And it detects again on the next flight.
    EXPECT_NE(run(st, cfg, 5000, s), 0u);
}

// A stale baro reading (no fresh sample) must not manufacture a step, and the
// altitude carried across the lockout must be fresh — otherwise the first
// armed sample compares against a pad-era altitude and always "steps".
TEST(DeploymentDetector, NoFreshBaro_NoStep)
{
    const auto cfg = baseConfig();
    tr::DeploymentState st;

    std::vector<Sample> s;
    for (int i = 0; i < 2000; ++i) {
        Sample e;
        e.acc_ms2   = 40.0f * G;
        e.palt_m    = 300.0f + i;   // would be a huge step if it counted
        e.new_baro  = false;        // ...but no fresh baro sample ever arrives
        e.palt_rate = -5.0f;
        s.push_back(e);
    }
    EXPECT_EQ(run(st, cfg, 5000, s), 0u);
    EXPECT_FALSE(st.baro_step_seen);
}

TEST(DeploymentDetector, AltitudeDriftDuringLockoutIsNotAStep)
{
    auto cfg = baseConfig();
    cfg.launch_lockout_ms = 1000;
    tr::DeploymentState st;

    // 0-1000 ms: boost climbs 500 m while locked out. If baro history were only
    // tracked once armed, the first armed sample would look like a 500 m step.
    std::vector<Sample> s;
    float palt = 0.0f;
    for (int i = 0; i < 1200; ++i) {
        Sample e;
        e.acc_ms2   = 40.0f * G;      // shock feature satisfied throughout
        e.palt_m    = palt;
        e.palt_rate = 150.0f;
        e.burnout   = (i >= 900);
        s.push_back(e);
        palt += 0.15f;                // 150 m/s, smooth — no real step
    }
    EXPECT_EQ(run(st, cfg, 0, s), 0u);
    EXPECT_FALSE(st.baro_step_seen);
    EXPECT_FALSE(st.detected);
}

}  // namespace
