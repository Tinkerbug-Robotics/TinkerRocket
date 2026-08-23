// Host tests for MainDeployGate (#834 item 4).
//
// The load-bearing test is HealthyBaroPathIsIdenticalToLegacyPredicate: a
// healthy-barometer flight is ~99% of the population and must behave exactly
// as the shipped firmware does.  Everything else tests a path that only opens
// when the barometer has already stopped being trustworthy.

#include <gtest/gtest.h>
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
void advance(State& st, Inputs& in, uint32_t ms, const Config& cfg = Config{})
{
    for (uint32_t i = 0; i < ms; ++i) { in.now_ms += 1; step(st, in, cfg); }
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
