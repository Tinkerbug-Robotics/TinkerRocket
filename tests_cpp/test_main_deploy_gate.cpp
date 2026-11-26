// Host tests for MainDeployGate (#834 item 4).
//
// The load-bearing test is HealthyBaroPathIsIdenticalToLegacyPredicate: a
// healthy-barometer flight is ~99% of the population and must behave exactly
// as the shipped firmware does.  Everything else tests a path that only opens
// when the barometer has already stopped being trustworthy.

#include <gtest/gtest.h>
#include <cmath>
#include <functional>
#include "MainDeployGate.h"

using namespace MainDeployGate;

namespace {

// A flight past apogee with a perfectly healthy barometer.
Inputs healthy(uint32_t now_ms, float alt, float rate)
{
    Inputs in;
    in.now_ms        = now_ms;
    in.apogee        = true;
    in.baro_healthy  = true;
    in.baro_alt_m    = alt;
    in.baro_rate_mps = rate;
    return in;
}

// Run step() forward at 1 kHz so dwells elapse, holding the inputs constant.
// A new GNSS fix arrives every 100 ms (#1761: the backstop needs a run of
// them), each reading the same propagated altitude.
void advance(State& st, Inputs& in, uint32_t ms, const Config& cfg = Config{})
{
    for (uint32_t i = 0; i < ms; ++i)
    {
        in.now_ms += 1;
        if (in.now_ms % 100 == 0) { ++in.gnss_fix_id; in.gnss_fix_age_ms = 0; }
        else                      { ++in.gnss_fix_age_ms; }
        step(st, in, cfg);
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// No-regression: the trusted-baro path IS the legacy predicate.
// ---------------------------------------------------------------------------
TEST(MainDeployGate, HealthyBaroPathIsIdenticalToLegacyPredicate)
{
    const Config cfg;
    size_t checked = 0;
    for (float thr : {40.0f, 100.0f, 150.0f, 200.0f, 300.0f})
    for (float alt = -50.0f; alt <= 500.0f; alt += 7.0f)
    for (float rate : {-40.0f, -20.0f, -5.0f, -0.001f, 0.0f, 0.001f, 5.0f, 30.0f})
    {
        State st;
        Inputs in = healthy(1000, alt, rate);
        // Randomise every backstop input; none of it may matter here.
        in.gnss_ok        = (checked % 2) == 0;
        in.gnss_agl_m     = alt + 250.0f;      // would fire on its own
        in.gnss_vel_u_mps = -25.0f;
        step(st, in, cfg);
        advance(st, in, cfg.reacquire_ms + 10, cfg);

        const bool legacy = (alt <= thr && rate < 0.0f);
        const Source got  = evaluate(st, in, thr, cfg);
        EXPECT_EQ(got, legacy ? Source::Baro : Source::None)
            << "alt=" << alt << " rate=" << rate << " thr=" << thr;
        ++checked;
    }
    EXPECT_GT(checked, 3000u);
}

TEST(MainDeployGate, NeverFiresBeforeApogee)
{
    const Config cfg;
    State st;
    Inputs in = healthy(1000, 10.0f, -20.0f);   // well below any threshold
    in.apogee = false;
    step(st, in, cfg);
    advance(st, in, 5000, cfg);
    EXPECT_EQ(evaluate(st, in, 200.0f, cfg), Source::None);
}

// ---------------------------------------------------------------------------
// The defect itself: a free-running estimate must not authorise a fire.
// ---------------------------------------------------------------------------
TEST(MainDeployGate, FreeRunningEstimateCannotFireOnBaro)
{
    const Config cfg;
    State st;
    Inputs in = healthy(1000, 1500.0f, -40.0f);
    step(st, in, cfg);
    advance(st, in, 1000, cfg);

    // Baro dies; the KF keeps coasting down through the threshold.
    in.baro_healthy = false;
    advance(st, in, 5000, cfg);
    in.baro_alt_m = 150.0f;          // "below" a 200 m main, per the free-run
    EXPECT_FALSE(baroTrusted(st, in, cfg));
    EXPECT_EQ(evaluate(st, in, 200.0f, cfg), Source::None)
        << "a dead barometer's coasted estimate must never fire the main";
}

TEST(MainDeployGate, ReacquireDwellBlocksTheFirstSampleAfterAFreeRun)
{
    const Config cfg;
    State st;
    Inputs in = healthy(1000, 1500.0f, -20.0f);
    step(st, in, cfg);
    advance(st, in, 1000, cfg);

    in.baro_healthy = false;                 // free-run
    advance(st, in, 5000, cfg);

    // Baro returns.  After a long free-run K0 -> 1, so the filter would adopt
    // one corrupt sample wholesale; the dwell must hold the trigger off.
    in.baro_healthy = true;
    in.baro_alt_m   = 50.0f;                 // a "sample" deep below the main
    in.baro_rate_mps = -30.0f;
    advance(st, in, cfg.reacquire_ms - 50, cfg);
    EXPECT_EQ(evaluate(st, in, 200.0f, cfg), Source::None);

    advance(st, in, 100, cfg);               // dwell now satisfied
    EXPECT_EQ(evaluate(st, in, 200.0f, cfg), Source::Baro);
}

// ---------------------------------------------------------------------------
// GNSS backstop: it must actually deploy, and never below the operator's value.
// ---------------------------------------------------------------------------
TEST(MainDeployGate, GnssBackstopFiresAtTheAppDefaultThreshold)
{
    const Config cfg;
    State st;
    Inputs in = healthy(1000, 1500.0f, -20.0f);
    step(st, in, cfg);
    advance(st, in, 1000, cfg);

    in.baro_healthy   = false;
    in.gnss_ok        = true;
    in.gnss_vel_u_mps = -20.0f;
    in.gnss_agl_m     = 400.0f;
    advance(st, in, 2000, cfg);
    EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::None);

    // Descend to the shipped default 100 m main.  It MUST fire — a gate that
    // silently declines here is a no-deploy, which is its own catastrophe.
    in.gnss_agl_m = 100.0f;
    EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::Gnss);
}

TEST(MainDeployGate, GnssMarginIsLateBiasedNeverBelowTheOperatorValue)
{
    const Config cfg;
    for (float thr : {40.0f, 100.0f, 200.0f})
    {
        State st;
        Inputs in = healthy(1000, 1000.0f, -20.0f);
        step(st, in, cfg);
        advance(st, in, 1000, cfg);
        in.baro_healthy   = false;
        in.gnss_ok        = true;
        in.gnss_vel_u_mps = -20.0f;
        advance(st, in, 2000, cfg);

        // Highest altitude that still fires is thr + margin, never below thr.
        in.gnss_agl_m = thr + cfg.gnss_margin_m + 0.1f;
        EXPECT_EQ(evaluate(st, in, thr, cfg), Source::None);
        in.gnss_agl_m = thr + cfg.gnss_margin_m - 0.1f;
        EXPECT_EQ(evaluate(st, in, thr, cfg), Source::Gnss);
        // Firing is possible at or above thr — never forced below it.
        in.gnss_agl_m = thr;
        EXPECT_EQ(evaluate(st, in, thr, cfg), Source::Gnss);
    }
}

TEST(MainDeployGate, GnssBackstopRequiresDescent)
{
    const Config cfg;
    State st;
    Inputs in = healthy(1000, 1000.0f, -20.0f);
    step(st, in, cfg);
    advance(st, in, 1000, cfg);
    in.baro_healthy   = false;
    in.gnss_ok        = true;
    in.gnss_agl_m     = 50.0f;
    in.gnss_vel_u_mps = 0.0f;          // sitting still / stale fix
    advance(st, in, 2000, cfg);
    EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::None);
}

TEST(MainDeployGate, NoGnssAndNoBaroDeclinesRatherThanFiringOnAClock)
{
    const Config cfg;
    State st;
    Inputs in = healthy(1000, 1000.0f, -20.0f);
    step(st, in, cfg);
    in.baro_healthy = false;
    in.gnss_ok      = false;
    advance(st, in, 60000, cfg);
    EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::None);
}

// ---------------------------------------------------------------------------
// Stuck static port: the silent no-deploy this change also fixes.
// ---------------------------------------------------------------------------
TEST(MainDeployGate, StuckStaticPortDemotesToGnssAndDeploys)
{
    const Config cfg;
    State st;
    Inputs in = healthy(1000, 1500.0f, -20.0f);
    step(st, in, cfg);
    advance(st, in, 1000, cfg);

    // Port blocks at apogee: pressure stays fresh and in-range, so
    // baro_healthy is TRUE, but alt_est pins and the rate decays to zero.
    in.baro_alt_m     = 1500.0f;
    in.baro_rate_mps  = 0.0f;
    in.gnss_ok        = true;
    in.gnss_vel_u_mps = -20.0f;
    in.gnss_agl_m     = 1500.0f;

    advance(st, in, cfg.stuck_dwell_ms - 100, cfg);
    EXPECT_FALSE(st.baro_stuck);
    advance(st, in, 200, cfg);
    EXPECT_TRUE(st.baro_stuck) << "flat baro + descending Doppler must demote";
    EXPECT_FALSE(baroTrusted(st, in, cfg));

    // Now the GNSS backstop carries the deploy the frozen baro never could.
    in.gnss_agl_m = 100.0f;
    EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::Gnss);
}

TEST(MainDeployGate, StuckDetectionIsLeakyNotLatched)
{
    const Config cfg;
    State st;
    Inputs in = healthy(1000, 1500.0f, 0.0f);
    step(st, in, cfg);
    in.gnss_ok        = true;
    in.gnss_vel_u_mps = -20.0f;
    advance(st, in, cfg.stuck_dwell_ms + 100, cfg);
    ASSERT_TRUE(st.baro_stuck);

    // The barometer starts agreeing again — it must be taken back, so a false
    // positive costs a segment of the descent rather than the whole flight.
    in.baro_rate_mps = -20.0f;
    advance(st, in, cfg.stuck_dwell_ms + 100, cfg);
    EXPECT_FALSE(st.baro_stuck);
    EXPECT_TRUE(baroTrusted(st, in, cfg));
}

TEST(MainDeployGate, SteadyDescentWithAHealthyBaroNeverLooksStuck)
{
    const Config cfg;
    State st;
    Inputs in = healthy(1000, 1500.0f, -20.0f);
    step(st, in, cfg);
    in.gnss_ok        = true;
    in.gnss_vel_u_mps = -20.0f;
    advance(st, in, 120000, cfg);      // two minutes under canopy
    EXPECT_FALSE(st.baro_stuck);
    EXPECT_EQ(st.stuck_ms, 0);
}

// ---------------------------------------------------------------------------
// Inhibits.
// ---------------------------------------------------------------------------
TEST(MainDeployGate, QuiescentFlagInhibitsEveryPath)
{
    const Config cfg;
    for (bool baro_alive : {true, false})
    {
        State st;
        Inputs in = healthy(1000, 10.0f, -20.0f);
        step(st, in, cfg);
        advance(st, in, cfg.reacquire_ms + 10, cfg);
        in.baro_healthy   = baro_alive;
        in.gnss_ok        = true;
        in.gnss_agl_m     = 5.0f;
        in.gnss_vel_u_mps = -3.0f;
        advance(st, in, cfg.gnss_reacquire_ms + 100, cfg);   // #1761: a run of fixes
        ASSERT_NE(evaluate(st, in, 100.0f, cfg), Source::None);

        in.quiescent_flag = true;   // baro-independent: always inhibits
        EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::None)
            << "baro_alive=" << baro_alive;
    }
}

TEST(MainDeployGate, ImpactFlagInhibitsOnlyWhileTheBaroIsTrusted)
{
    const Config cfg;

    // Healthy barometer: impact_flag is meaningful, so it must inhibit.
    {
        State st;
        Inputs in = healthy(1000, 10.0f, -20.0f);
        step(st, in, cfg);
        advance(st, in, cfg.reacquire_ms + 10, cfg);
        ASSERT_EQ(evaluate(st, in, 100.0f, cfg), Source::Baro);
        in.impact_flag = true;
        EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::None);
    }

    // Static port blocked ON THE PAD: barometric altitude reads ~0 AGL all
    // flight, so the drogue charge's own shock latches impact_flag while the
    // vehicle is still at altitude. That must NOT disable the GNSS backstop —
    // this is precisely the flight the backstop exists for.
    {
        State st;
        Inputs in = healthy(1000, 0.0f, 0.0f);   // pinned baro, zero rate
        step(st, in, cfg);
        in.gnss_ok        = true;
        in.gnss_vel_u_mps = -20.0f;
        in.gnss_agl_m     = 800.0f;
        advance(st, in, cfg.stuck_dwell_ms + 100, cfg);
        ASSERT_TRUE(st.baro_stuck);

        in.impact_flag = true;                   // drogue shock, still at altitude
        in.gnss_agl_m  = 100.0f;
        EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::Gnss)
            << "a baro-gated impact latch must not veto the GNSS backstop";
    }
}

TEST(MainDeployGate, GnssPathIgnoresGroundNoiseDescentRates)
{
    const Config cfg;
    State st;
    Inputs in = healthy(1000, 1000.0f, -20.0f);
    step(st, in, cfg);
    advance(st, in, 1000, cfg);
    in.baro_healthy = false;
    in.gnss_ok      = true;
    in.gnss_agl_m   = 2.0f;                      // sitting on the ground
    advance(st, in, 2000, cfg);

    in.gnss_vel_u_mps = -0.4f;                   // receiver noise, not a descent
    EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::None);
    in.gnss_vel_u_mps = -5.0f;                   // a real descent still fires
    EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::Gnss);
}

TEST(MainDeployGate, MidFlightRebootDoesNotFireOnTheFirstTick)
{
    const Config cfg;
    State st;
    reset(st, 50000, /*after_reboot=*/true);

    // Recovery restored apogee from the snapshot; the sensor stack is cold.
    Inputs in;
    in.now_ms          = 50000;
    in.apogee          = true;
    in.baro_healthy    = false;
    in.gnss_ok         = true;
    in.gnss_agl_m      = 20.0f;
    in.gnss_vel_u_mps  = -20.0f;
    step(st, in, cfg);
    EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::None);

    advance(st, in, cfg.resume_dwell_ms - 100, cfg);
    EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::None);
    advance(st, in, 200, cfg);
    EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::Gnss);
}

TEST(MainDeployGate, NormalLaunchCreditsThePadPhaseSoThereIsNoDeadWindow)
{
    const Config cfg;
    State st;
    reset(st, 10000, /*after_reboot=*/false);
    Inputs in = healthy(10000, 50.0f, -20.0f);
    step(st, in, cfg);
    // First tick after apogee on a normal flight fires immediately — the
    // dwell must not open a hole the legacy code did not have.
    EXPECT_EQ(evaluate(st, in, 100.0f, cfg), Source::Baro);
}


// ---------------------------------------------------------------------------
// #1761: the GNSS backstop needs a run of consistent fixes.
// ---------------------------------------------------------------------------
namespace {
constexpr float kDescent = -20.0f;   // m/s under the drogue

// A descent with the barometer untrusted (dead, or a stuck port reading a flat
// 1200 m), as main.cpp feeds the gate. Fix records arrive every fix_period_ms
// while `up`; a record `invalid` says "no solution" (gnss_ok false until the
// next record). A good record carries fix_agl(t, truth) and the velocity at
// that moment. gnss_ok is main.cpp's: a good record under 2 s old. Steps and
// evaluates every 1 ms, as the FC does; returns the true AGL at which the
// main fired, NAN if it never did.
struct Descent
{
    float    start_agl     = 1500.0f;
    uint32_t fix_period_ms = 100;
    float    thr           = 150.0f;
    bool     stuck         = false;
    std::function<float(uint32_t)>        vel     = [](uint32_t) { return kDescent; };
    std::function<bool(uint32_t)>         up      = [](uint32_t) { return true; };
    std::function<bool(uint32_t)>         invalid = [](uint32_t) { return false; };
    std::function<float(uint32_t, float)> fix_agl = [](uint32_t, float truth) { return truth; };
};

float fly(const Descent& d, const Config& cfg = Config{})
{
    State st;
    reset(st, 1000, /*after_reboot=*/false);
    Inputs in;
    in.apogee        = true;
    in.baro_healthy  = d.stuck;
    in.baro_alt_m    = 1200.0f;
    in.baro_rate_mps = 0.0f;
    uint32_t last_rec_ms = 0;
    bool     have_rec = false, rec_valid = false;
    float    fix_value = 0.0f, fix_vel = 0.0f;
    float    truth = d.start_agl;
    for (uint32_t t = 0; t < 300000; ++t)
    {
        in.now_ms = 1000 + t;
        truth += d.vel(t) * 1e-3f;
        if (truth <= 0.0f) break;
        if (d.up(t) && t % d.fix_period_ms == 0)
        {
            last_rec_ms = in.now_ms;
            ++in.gnss_fix_id;
            have_rec  = true;
            rec_valid = !d.invalid(t);
            if (rec_valid) { fix_value = d.fix_agl(t, truth); fix_vel = d.vel(t); }
        }
        in.gnss_fix_age_ms = in.now_ms - last_rec_ms;
        in.gnss_ok         = have_rec && rec_valid && in.gnss_fix_age_ms < 2000;
        in.gnss_vel_u_mps  = fix_vel;
        in.gnss_agl_m      = fix_value + fix_vel * (float)in.gnss_fix_age_ms * 1e-3f;
        step(st, in, cfg);
        if (evaluate(st, in, d.thr, cfg) != Source::None) return truth;
    }
    return NAN;
}

// The constant-descent form the first #1761 tests use.
template <class FixAgl, class Up>
float flyBackstop(FixAgl fix_agl, Up up, float thr = 150.0f, bool stuck = false,
                  const Config& cfg = Config{}, uint32_t fix_period_ms = 100)
{
    Descent d;
    d.fix_agl       = fix_agl;
    d.up            = up;
    d.thr           = thr;
    d.stuck         = stuck;
    d.fix_period_ms = fix_period_ms;
    return fly(d, cfg);
}

const auto kTruth  = [](uint32_t, float truth) { return truth; };
const auto kAlways = [](uint32_t) { return true; };
}  // namespace

TEST(MainDeployGate, AGoodStreamStillFiresInsideTheLateBiasedBand)
{
    const Config cfg;
    const float at = flyBackstop(kTruth, kAlways);
    EXPECT_GE(at, 150.0f);
    EXPECT_LE(at, 150.0f + 2.0f * cfg.gnss_margin_m);
}

TEST(MainDeployGate, ALowFirstFixAfterAGapDoesNotFireTheMainHigh)
{
    // #1761's harness: silent 15-20 s after apogee (a receiver restart, or
    // lost lock under the drogue), and the first fix back reads 1 km low: 100 m
    // against a true 1100 m. That fix used to fire the main there.
    const Config cfg;
    const auto gap = [](uint32_t t) { return t < 15000 || t >= 20000; };
    const float at = flyBackstop([](uint32_t t, float truth) { return t == 20000 ? truth - 1000.0f : truth; }, gap);
    EXPECT_GE(at, 150.0f - 25.0f);
    EXPECT_LE(at, 150.0f + 2.0f * cfg.gnss_margin_m);
    // the first three fixes back low: the same
    const float at3 = flyBackstop([](uint32_t t, float truth) { return t >= 20000 && t < 20300 ? truth - 1000.0f : truth; }, gap);
    EXPECT_GE(at3, 150.0f - 25.0f);
    EXPECT_LE(at3, 150.0f + 2.0f * cfg.gnss_margin_m);
}

TEST(MainDeployGate, ASingleGlitchMidStreamDoesNotFireTheMainHigh)
{
    // No gap at all, so gnss_ok never drops: only the fix-to-fix check sees it.
    const Config cfg;
    const float at = flyBackstop([](uint32_t t, float truth) { return t == 20000 ? truth - 1000.0f : truth; }, kAlways);
    EXPECT_GE(at, 150.0f - 25.0f);
    EXPECT_LE(at, 150.0f + 2.0f * cfg.gnss_margin_m);
}

TEST(MainDeployGate, AStuckPortWithAShortGapAndALowFirstFixWaitsForTheSetting)
{
    // A stuck port: the barometer answers, flat, and is demoted by the
    // cross-check; a 3 s gap keeps it demoted. The first fix back reads 1 km low.
    const Config cfg;
    const auto gap = [](uint32_t t) { return t < 17000 || t >= 20000; };
    const float at = flyBackstop([](uint32_t t, float truth) { return t == 20000 ? truth - 1000.0f : truth; },
                                 gap, 150.0f, /*stuck=*/true);
    EXPECT_GE(at, 150.0f - 25.0f);
    EXPECT_LE(at, 150.0f + 2.0f * cfg.gnss_margin_m);
}

TEST(MainDeployGate, AGlitchJustAboveTheSettingCostsAtMostOneDwell)
{
    // A glitch that restarts the run right where the main is due delays it by
    // the re-acquire dwell: ~20 m at the drogue's 20 m/s, never more.
    const Config cfg;
    for (uint32_t glitch = 66000; glitch <= 68200; glitch += 100)
    {
        const float at = flyBackstop([glitch](uint32_t t, float truth) { return t == glitch ? truth + 400.0f : truth; },
                                     kAlways);
        EXPECT_GE(at, 150.0f + cfg.gnss_margin_m + kDescent * (cfg.gnss_reacquire_ms + 200) * 1e-3f)
            << "glitch at t=" << glitch;
    }
}

TEST(MainDeployGate, ReceiverNoiseNeverStarvesTheBackstop)
{
    // Fix-to-fix noise of up to +-15 m (well past a healthy receiver's) must
    // not keep restarting the run: that would be a no-deploy with the
    // barometer dead, the flight this path exists for.
    const Config cfg;
    uint32_t seed = 1761;
    const float at = flyBackstop([&seed](uint32_t, float truth) {
        seed = seed * 1103515245u + 12345u;
        return truth + (float)((int)((seed >> 8) % 31u) - 15);
    }, kAlways);
    ASSERT_FALSE(std::isnan(at)) << "the main never fired";
    EXPECT_GE(at, 150.0f - 15.0f);
    EXPECT_LE(at, 150.0f + 2.0f * cfg.gnss_margin_m + 15.0f);
}

TEST(MainDeployGate, AGnssStreamThatNeverSettlesNeverFires)
{
    // Every fix contradicts the one before it: no run ever forms.
    bool hi = false;
    const float at = flyBackstop([&hi](uint32_t, float truth) { hi = !hi; return hi ? truth + 400.0f : truth - 400.0f; },
                                 kAlways);
    EXPECT_TRUE(std::isnan(at));
}

TEST(MainDeployGate, AFastReceiversBurstOfLowFixesWaitsTheDwell)
{
    // A 20 Hz receiver: its first three fixes back after a gap agree with each
    // other and all read 1 km low, inside 100 ms. The fix count alone would
    // pass them; the dwell holds until the good fixes have taken over.
    const Config cfg;
    const auto gap = [](uint32_t t) { return t < 15000 || t >= 20000; };
    const float at = flyBackstop([](uint32_t t, float truth) { return t >= 20000 && t < 20150 ? truth - 1000.0f : truth; },
                                 gap, 150.0f, false, cfg, /*fix_period_ms=*/50);
    EXPECT_GE(at, 150.0f - 25.0f);
    EXPECT_LE(at, 150.0f + 2.0f * cfg.gnss_margin_m);
}

TEST(MainDeployGate, ASlowReceiversOneLowFixWaitsForMoreFixes)
{
    // Fixes 1.5 s apart: one low fix back after a gap is the whole run for
    // longer than the dwell. The dwell alone would pass it; the fix count
    // holds until more fixes say otherwise.
    const Config cfg;
    const auto gap = [](uint32_t t) { return t < 13500 || t >= 19500; };
    const float at = flyBackstop([](uint32_t t, float truth) { return t == 19500 ? truth - 1000.0f : truth; },
                                 gap, 150.0f, false, cfg, /*fix_period_ms=*/1500);
    EXPECT_GE(at, 150.0f - 40.0f);
    EXPECT_LE(at, 150.0f + 2.0f * cfg.gnss_margin_m + 30.0f);
}

TEST(MainDeployGate, ALongGapEndsTheRunWhateverTheToleranceWouldAllow)
{
    // After 55 s of silence the fix-to-fix tolerance has grown to 325 m, so a
    // fix back 200 m low (100 m against a true 300 m) would pass as consistent
    // with the one before the gap. The gap itself must end the run.
    const Config cfg;
    const auto gap = [](uint32_t t) { return t < 5000 || t >= 60000; };
    const float at = flyBackstop([](uint32_t t, float truth) { return t == 60000 ? truth - 200.0f : truth; }, gap);
    EXPECT_GE(at, 150.0f - 25.0f);
    EXPECT_LE(at, 150.0f + 2.0f * cfg.gnss_margin_m);
}

// ---- #1761, from the review: what the persistence must not cost ----

TEST(MainDeployGate, AnInvalidRecordEverySecondStillDeploys)
{
    // A 20 Hz receiver that drops a solution once a second (the 10-24 Goblin
    // log has them through its descent). Ending the run on each one restarted
    // it for ever and the main never fired.
    const Config cfg;
    Descent d;
    d.fix_period_ms = 50;
    d.invalid       = [](uint32_t t) { return t % 1000 == 500; };
    const float at = fly(d, cfg);
    ASSERT_FALSE(std::isnan(at)) << "the main never fired";
    EXPECT_GE(at, 150.0f);
    EXPECT_LE(at, 150.0f + 2.0f * cfg.gnss_margin_m);
}

TEST(MainDeployGate, ADroppedFixAtOneHertzKeepsTheRun)
{
    // Fixes 1.001 s apart (the gnss-sdr-dev board's own rate), one of them
    // lost: a 2.002 s hole, gnss_ok false for 2 ms. That must not end the run.
    const Config cfg;
    Descent d;
    d.fix_period_ms = 1001;
    const float clean = fly(d, cfg);
    for (uint32_t lost = 60060; lost <= 67067; lost += 1001)
    {
        d.up = [lost](uint32_t t) { return t != lost; };
        const float at = fly(d, cfg);
        EXPECT_NEAR(at, clean, 1.0f) << "fix at t=" << lost << " lost";
    }
}

TEST(MainDeployGate, AFastOneHertzDescentStillDeploys)
{
    // 60 m/s between fixes 1 s apart: the step between fixes is past the 50 m
    // tolerance, and only the Doppler prediction keeps the run going.
    const Config cfg;
    Descent d;
    d.start_agl     = 3000.0f;
    d.fix_period_ms = 1000;
    d.vel           = [](uint32_t) { return -60.0f; };
    const float at = fly(d, cfg);
    ASSERT_FALSE(std::isnan(at)) << "the main never fired";
    EXPECT_GE(at, 150.0f - 60.0f);
    EXPECT_LE(at, 150.0f + 2.0f * cfg.gnss_margin_m);
}

TEST(MainDeployGate, ADrogueOpeningDoesNotRestartTheRun)
{
    // 80 m/s slowing to 20 m/s over 2 s, right where a high main is due: a
    // restart would cost the dwell, ~20-80 m.
    const Config cfg;
    Descent d;
    d.start_agl = 1600.0f;
    d.thr       = 1200.0f;
    d.vel       = [](uint32_t t) { return t < 4000 ? -80.0f : t < 6000 ? -80.0f + 30.0f * (float)(t - 4000) * 1e-3f : -20.0f; };
    const float at = fly(d, cfg);
    ASSERT_FALSE(std::isnan(at));
    EXPECT_GE(at, 1200.0f + cfg.gnss_margin_m - 10.0f) << "the run restarted";
}

TEST(MainDeployGate, ThirtyFiveMetreStepsAsThePx100FlewDoNotRestartTheRun)
{
    // The 65 mm rocket's PX100 on 11-21: 35 m steps between 20 Hz fixes at 6
    // satellites. A tolerance tight enough to trip on them never deploys.
    const Config cfg;
    Descent d;
    d.fix_period_ms = 50;
    bool hi = false;
    d.fix_agl = [&hi](uint32_t, float truth) { hi = !hi; return truth + (hi ? 17.5f : -17.5f); };
    const float at = fly(d, cfg);
    ASSERT_FALSE(std::isnan(at)) << "the main never fired";
    EXPECT_GE(at, 150.0f - 20.0f);
    EXPECT_LE(at, 150.0f + 2.0f * cfg.gnss_margin_m + 20.0f);
}

TEST(MainDeployGate, TheToleranceGrowsWithTheTimeBetweenFixes)
{
    // After a 2.2 s dropout (short of a gap) the first fix back may step
    // 55 m from the prediction: 50 m + 5 m/s x 2.2 s allows it.
    const Config cfg;
    State st;
    reset(st, 1000, false);
    Inputs in;
    in.now_ms = 1000;
    in.apogee = true;
    float agl = 1500.0f;
    for (uint32_t t = 0; t <= 3000; ++t)
    {
        in.now_ms = 1000 + t;
        agl += kDescent * 1e-3f;
        if (t % 100 == 0) { ++in.gnss_fix_id; in.gnss_fix_age_ms = 0; in.gnss_agl_m = agl; }
        else              { ++in.gnss_fix_age_ms; in.gnss_agl_m += kDescent * 1e-3f; }
        in.gnss_ok        = true;
        in.gnss_vel_u_mps = kDescent;
        step(st, in, cfg);
    }
    ASSERT_TRUE(gnssSettled(st, in, cfg));
    for (uint32_t t = 3001; t <= 5200; ++t)
    {
        in.now_ms = 1000 + t;
        agl += kDescent * 1e-3f;
        ++in.gnss_fix_age_ms;
        in.gnss_ok = in.gnss_fix_age_ms < 2000;
        step(st, in, cfg);
    }
    ++in.gnss_fix_id;
    in.gnss_fix_age_ms = 0;
    in.gnss_agl_m      = agl + 55.0f;
    in.gnss_ok         = true;
    in.now_ms         += 1;
    step(st, in, cfg);
    EXPECT_TRUE(gnssSettled(st, in, cfg)) << "a 55 m step after 2.2 s restarted the run";
}
