// #1166: the out computer's verdict on the hold-up supercap — charged, still
// charging, or never charged — from a stream of V_SCAP readings and uptime.
// The mini's TPS61094 charges the 5 F cap at 100 mA to 2.5 V in about two
// minutes; a cap that silently never charges looks exactly like one that did,
// so this is the one thing on the board that says which.
#include <gtest/gtest.h>
#include "holdup_policy.h"
#include <SPI.h>       // host shim: SPI_MODE0, which the OC config.h names
#include "config.h"    // the SHIPPED HOLDUP_* constants (TR_BOARD_M1 from CMake)
#include <cmath>
#include <vector>

using namespace holdup_policy;

namespace
{
constexpr float    kBarV      = 2.2f;
constexpr uint32_t kAdvisory  = 180000u;   // 3 min
constexpr uint32_t kSecond    = 1000u;

// The measured ramp shape: 100 mA into 5 F is 20 mV/s, so 0 → 2.5 V takes
// ~125 s, then the converter holds 2.5 V.
float rampVolts(uint32_t t_ms)
{
    const float v = 0.02f * (float)(t_ms / kSecond);
    return v < 2.5f ? v : 2.5f;
}
}

TEST(HoldupPolicy1166, ColdStartRampIsChargingThenChargedNeverAnAdvisory)
{
    Tracker t;
    uint8_t worst = NONE;
    uint32_t first_charged_ms = 0;
    for (uint32_t ms = kSecond; ms <= 600u * kSecond; ms += kSecond)
    {
        const uint8_t s = t.update(rampVolts(ms), ms, kBarV, kAdvisory);
        EXPECT_NE(s, NOT_CHARGING) << "at +" << ms / kSecond << " s";
        if (s == CHARGED && first_charged_ms == 0) first_charged_ms = ms;
        if (s > worst) worst = s;
    }
    EXPECT_EQ(worst, CHARGED);
    // 2.2 V at 20 mV/s is 110 s.
    EXPECT_EQ(first_charged_ms, 110u * kSecond);
    EXPECT_EQ(t.state, CHARGED);
}

TEST(HoldupPolicy1166, CapThatNeverChargesTripsExactlyAtTheGraceWindow)
{
    Tracker t;
    // Reads 0.3 V forever (the #999 corner: bypass never enters buck_on).
    for (uint32_t ms = kSecond; ms < kAdvisory; ms += kSecond)
        EXPECT_EQ(t.update(0.3f, ms, kBarV, kAdvisory), CHARGING) << ms;
    EXPECT_EQ(t.update(0.3f, kAdvisory, kBarV, kAdvisory), NOT_CHARGING);
    EXPECT_EQ(t.update(0.3f, kAdvisory + 60u * kSecond, kBarV, kAdvisory), NOT_CHARGING);
}

TEST(HoldupPolicy1166, ADrainedCapGetsItsRechargeTimeFromTheLastChargedReading)
{
    Tracker t;
    // Charged for ten minutes, then a hold-up event drains it to 0.9 V.
    for (uint32_t ms = kSecond; ms <= 600u * kSecond; ms += kSecond)
        t.update(2.5f, ms, kBarV, kAdvisory);
    ASSERT_EQ(t.state, CHARGED);
    const uint32_t drain_ms = 601u * kSecond;
    // Uptime is far past 3 min, but the window runs from the last CHARGED
    // reading (600 s), so the first 3 min after the drain are CHARGING.
    EXPECT_EQ(t.update(0.9f, drain_ms, kBarV, kAdvisory), CHARGING);
    EXPECT_EQ(t.update(1.5f, 600u * kSecond + kAdvisory - kSecond, kBarV, kAdvisory), CHARGING);
    // Recharged in time: never an advisory.
    EXPECT_EQ(t.update(2.3f, 600u * kSecond + kAdvisory, kBarV, kAdvisory), CHARGED);

    // The same drain that does NOT recharge: advisory at 3 min after the drain.
    Tracker u;
    for (uint32_t ms = kSecond; ms <= 600u * kSecond; ms += kSecond)
        u.update(2.5f, ms, kBarV, kAdvisory);
    EXPECT_EQ(u.update(0.9f, 600u * kSecond + kAdvisory - kSecond, kBarV, kAdvisory), CHARGING);
    EXPECT_EQ(u.update(0.9f, 600u * kSecond + kAdvisory, kBarV, kAdvisory), NOT_CHARGING);
}

TEST(HoldupPolicy1166, TheBarIsInclusiveAndAReadingFailureIsItsOwnVerdict)
{
    Tracker t;
    EXPECT_EQ(t.update(2.19f, kSecond, kBarV, kAdvisory), CHARGING);
    EXPECT_EQ(t.update(2.20f, 2 * kSecond, kBarV, kAdvisory), CHARGED);
    // The ADC did not answer: NO_READING, an advisory of its own — a dead
    // sense is not silence — and the grace clock is untouched.
    EXPECT_EQ(t.update(NAN, 3 * kSecond, kBarV, kAdvisory), NO_READING);
    // ...and the failed read did not restart or advance the grace clock:
    // 3 min after the last charged reading it is an advisory, not sooner.
    EXPECT_EQ(t.update(1.0f, 2 * kSecond + kAdvisory - kSecond, kBarV, kAdvisory), CHARGING);
    EXPECT_EQ(t.update(1.0f, 2 * kSecond + kAdvisory, kBarV, kAdvisory), NOT_CHARGING);
}

TEST(HoldupPolicy1166, UptimeWrapDoesNotResetTheWindow)
{
    Tracker t;
    const uint32_t near_wrap = 0xFFFFFFFFu - 30u * kSecond;
    EXPECT_EQ(t.update(2.5f, near_wrap, kBarV, kAdvisory), CHARGED);
    // 3 min later, across the wrap, still under the bar → advisory; the
    // unsigned subtraction makes the elapsed time right.
    const uint32_t after = near_wrap + kAdvisory;   // wraps
    EXPECT_LT(after, near_wrap);
    EXPECT_EQ(t.update(1.0f, after - kSecond, kBarV, kAdvisory), CHARGING);
    EXPECT_EQ(t.update(1.0f, after, kBarV, kAdvisory), NOT_CHARGING);
}

TEST(HoldupPolicy1166, TraceFollowsTheVoltageAndNeverGoesQuietForLong)
{
    const float    dv = 0.05f;
    const uint32_t period = 60u * kSecond;
    // The first sample always prints.
    EXPECT_TRUE(traceDue(false, false, 0.0f, NAN, 0, 0, dv, period));
    // Steady: nothing until the period.
    EXPECT_FALSE(traceDue(false, true, 2.50f, 2.50f, 30u * kSecond, 0, dv, period));
    EXPECT_TRUE(traceDue(false, true, 2.50f, 2.50f, period, 0, dv, period));
    // Moving: the ramp at 20 mV/s earns a line every 2.5 s, either direction.
    EXPECT_FALSE(traceDue(false, true, 0.84f, 0.80f, 2u * kSecond, 0, dv, period));
    EXPECT_TRUE(traceDue(false, true, 0.85f, 0.80f, 2500u, 0, dv, period));
    EXPECT_TRUE(traceDue(false, true, 0.75f, 0.80f, 2500u, 0, dv, period));
    // A change of verdict always prints.
    EXPECT_TRUE(traceDue(true, true, 2.21f, 2.20f, 5u * kSecond, 0, dv, period));
    // NaN-to-NaN is steady (a dead sense does not spam); NaN-to-value and
    // value-to-NaN are transitions and print.
    EXPECT_FALSE(traceDue(false, true, NAN, NAN, 5u * kSecond, 0, dv, period));
    EXPECT_TRUE(traceDue(false, true, NAN, NAN, period, 0, dv, period));
    EXPECT_TRUE(traceDue(false, true, 1.0f, NAN, 5u * kSecond, 0, dv, period));
    EXPECT_TRUE(traceDue(false, true, NAN, 1.0f, 5u * kSecond, 0, dv, period));
    EXPECT_EQ(stateName(NOT_CHARGING), std::string("NOT CHARGING"));
    EXPECT_EQ(stateName(NO_READING), std::string("NO READING"));
    EXPECT_EQ(stateName(CHARGED), std::string("charged"));
}

TEST(HoldupPolicy1166, ADischargeRidingTheCapLogsEverySecond)
{
    // A hold-up event: the rail rides the cap and V_SCAP falls ~84 mV/s at
    // 190 mA.  Every one-second sample moves more than 50 mV, so every one
    // prints — that curve is the hold time.
    float v = 2.5f, last = 2.5f;
    uint32_t last_ms = 0;
    int lines = 0;
    for (uint32_t ms = kSecond; ms <= 10u * kSecond; ms += kSecond)
    {
        v -= 0.084f;
        if (traceDue(false, true, v, last, ms, last_ms, 0.05f, 60u * kSecond)) { ++lines; last = v; last_ms = ms; }
    }
    EXPECT_EQ(lines, 10);
}

// The local kBarV / kAdvisory above are what the cases are written against;
// this ties them to the values the M1 build actually ships, so a config.h
// edit cannot leave the suite proving the wrong window.
TEST(HoldupPolicy1166, TheSuiteMirrorsTheShippedConstants)
{
    EXPECT_FLOAT_EQ(kBarV, config::HOLDUP_CHARGED_V);
    EXPECT_EQ(kAdvisory, config::HOLDUP_LOW_ADVISORY_MS);
    EXPECT_FLOAT_EQ(0.05f, config::HOLDUP_TRACE_DELTA_V);
    EXPECT_EQ(60u * kSecond, config::HOLDUP_TRACE_PERIOD_MS);
    EXPECT_GE(config::SCAP_ADC_PIN, 0);   // the mini reads it (config.h asserts this too)
}

TEST(HoldupPolicy1166, TheGraceWindowOutlastsTheSlowCornerOfTheRamp)
{
    // A healthy board with a fat cap on a weak charge current must never see
    // a false advisory before it crosses the bar.  EDLC capacitance runs to
    // +30 %, and the ICHG code about -10 %: 6.5 F at 90 mA is the slow corner.
    const double c_max_f  = 5.0 * 1.30;
    const double i_min_a  = 0.100 * 0.90;
    const double t_bar_s  = c_max_f * config::HOLDUP_CHARGED_V / i_min_a;   // ~159 s
    EXPECT_GE(config::HOLDUP_LOW_ADVISORY_MS, (uint32_t)(t_bar_s * 1000.0) + 15u * kSecond)
        << "window " << config::HOLDUP_LOW_ADVISORY_MS << " ms vs the slow corner crossing at "
        << t_bar_s << " s";

    // And the tracker driven through exactly that ramp from a cold start:
    // CHARGING all the way to the bar, then CHARGED, never NOT_CHARGING.
    Tracker t;
    const float slope_v_per_s = (float)(i_min_a / c_max_f);   // ~13.8 mV/s
    bool charged = false;
    uint32_t first_charged_ms = 0;
    for (uint32_t ms = kSecond; ms <= 400u * kSecond; ms += kSecond)
    {
        float v = slope_v_per_s * (float)(ms / kSecond);
        if (v > 2.5f) v = 2.5f;
        const uint8_t s = t.update(v, ms, config::HOLDUP_CHARGED_V, config::HOLDUP_LOW_ADVISORY_MS);
        EXPECT_NE(s, NOT_CHARGING) << "false advisory at +" << ms / kSecond << " s on the slow corner";
        if (s == CHARGED && !charged) { charged = true; first_charged_ms = ms; }
    }
    EXPECT_TRUE(charged);
    EXPECT_LT(first_charged_ms, config::HOLDUP_LOW_ADVISORY_MS);

    // The nominal ramp (100 mA into 5 F, 20 mV/s) sits well inside the window.
    EXPECT_LT(5.0 * config::HOLDUP_CHARGED_V / 0.100 * 1000.0, config::HOLDUP_LOW_ADVISORY_MS * 0.7);
}

TEST(HoldupPolicy1166, ThePinCeilingSitsBetweenTheWorstCaseCapAndTheRangeEnd)
{
    // Clean charge terminates no higher than VIN - 800 mV = 2.665 V at the
    // 3.465 V rail, so the pad never legitimately exceeds that over the
    // divider; the 6 dB range is characterised to ~1750 mV on the S3.  The
    // over-range warning must fire only above the first and never wait for
    // the second.
    const int worst_pad_mv = (int)(2.665 / (double)config::SCAP_DIVIDER_RATIO * 1000.0 + 0.5);
    EXPECT_GT(config::HOLDUP_PIN_CEILING_MV, worst_pad_mv);
    EXPECT_LE(config::HOLDUP_PIN_CEILING_MV, 1750);
    // ...and a charged cap at termination is comfortably readable.
    EXPECT_LT(2.5 / (double)config::SCAP_DIVIDER_RATIO * 1000.0, config::HOLDUP_PIN_CEILING_MV);
}

TEST(HoldupPolicy1166, TracePeriodSurvivesUptimeWrap)
{
    // The window wrap is covered above; the trace period uses the same
    // unsigned arithmetic and must not go quiet, or chatter, across it.
    const uint32_t period = 60u * kSecond;
    const uint32_t last   = 0xFFFFFFFFu - 10u * kSecond;
    EXPECT_FALSE(traceDue(false, true, 2.5f, 2.5f, last + 30u * kSecond, last, 0.05f, period));  // wrapped, 30 s
    EXPECT_TRUE(traceDue(false, true, 2.5f, 2.5f, last + period, last, 0.05f, period));           // wrapped, 60 s
}

// ============================================================================
// #1485: is a capacitor on the sense node at all? Every sequence below is a
// 1 Hz trace measured on the mini's out computer on 2026-09-24.
// ============================================================================
namespace
{
Tracker fitted()
{
    return Tracker::withFitCheck(config::HOLDUP_JUMP_V, config::HOLDUP_JUMPS_NOT_FITTED,
                                 config::HOLDUP_CALM_TO_CLEAR);
}

// Feed a 1 Hz trace starting at `t0_s`; return the verdict after each reading.
std::vector<uint8_t> feed(Tracker& t, const std::vector<float>& volts, uint32_t t0_s = 10)
{
    std::vector<uint8_t> out;
    for (size_t i = 0; i < volts.size(); ++i)
        out.push_back(t.update(volts[i], (t0_s + (uint32_t)i) * kSecond,
                               config::HOLDUP_CHARGED_V, config::HOLDUP_LOW_ADVISORY_MS));
    return out;
}
}

TEST(HoldupPolicy1485, AnEmptyFootprintReadsNotFittedNotCharged)
{
    // First mini, built without C130: the node swung like this, and the
    // voltage rule alone called 18 of 21 readings CHARGED.
    const std::vector<float> no_cap = {2.54f, 3.09f, 2.31f, 2.77f, 2.12f,
                                       2.51f, 3.05f, 2.29f, 2.73f, 2.10f};
    Tracker plain;   // the rule as it shipped
    const auto before = feed(plain, no_cap);
    EXPECT_EQ(before[1], CHARGED);

    Tracker t = fitted();
    const auto v = feed(t, no_cap);
    // Three jumps take four readings; from then on it stays NOT_FITTED.
    for (size_t i = 3; i < v.size(); ++i) EXPECT_EQ(v[i], NOT_FITTED) << i;
}

TEST(HoldupPolicy1485, TheMeasuredRechargeRampNeverReadsNotFitted)
{
    // 21 mV/s from 0.81 V after a hold-up drained the cap (4.7 F at 100 mA).
    std::vector<float> ramp;
    for (float v = 0.81f; v < 2.40f; v += 0.021f) ramp.push_back(v);
    for (int i = 0; i < 20; ++i) ramp.push_back(2.40f);
    Tracker t = fitted();
    const auto v = feed(t, ramp);
    for (size_t i = 0; i < v.size(); ++i) EXPECT_NE(v[i], NOT_FITTED) << i;
    EXPECT_EQ(v.back(), CHARGED);
}

TEST(HoldupPolicy1485, AFullLoadHoldUpNeverReadsNotFitted)
{
    // USB pulled with both processors running: the fall steepens to
    // ~0.15 V/s at the end, the fastest a real cap was seen to move.
    const std::vector<float> hold = {2.40f, 2.35f, 2.29f, 2.23f, 2.18f, 2.11f, 2.05f, 1.99f,
                                     1.92f, 1.85f, 1.78f, 1.70f, 1.63f, 1.55f, 1.46f, 1.37f,
                                     1.27f, 1.16f, 1.03f, 0.88f};
    Tracker t = fitted();
    const auto v = feed(t, hold);
    for (size_t i = 0; i < v.size(); ++i) EXPECT_NE(v[i], NOT_FITTED) << i;
}

TEST(HoldupPolicy1485, OneGlitchIsNotAMissingCap)
{
    // A single bad sample is two jumps (out and back), one short of the three.
    Tracker t = fitted();
    const auto v = feed(t, {2.40f, 2.40f, 2.90f, 2.40f, 2.40f, 2.40f, 2.40f});
    for (size_t i = 0; i < v.size(); ++i) EXPECT_EQ(v[i], CHARGED) << i;
}

TEST(HoldupPolicy1485, CalmReadingsWithdrawTheVerdict)
{
    Tracker t = fitted();
    feed(t, {2.54f, 3.09f, 2.31f, 2.77f});
    EXPECT_EQ(t.state, NOT_FITTED);
    // 2.77 -> 2.40 is itself a 0.37 V jump; the calm steps start after it.
    std::vector<float> settle(1 + config::HOLDUP_CALM_TO_CLEAR - 1, 2.40f);
    feed(t, settle, 20);
    EXPECT_EQ(t.state, NOT_FITTED);   // one calm step short
    EXPECT_EQ(t.update(2.40f, 40u * kSecond, config::HOLDUP_CHARGED_V,
                       config::HOLDUP_LOW_ADVISORY_MS), CHARGED);
}

TEST(HoldupPolicy1485, AFailedReadNeitherJumpsNorCalms)
{
    Tracker t = fitted();
    EXPECT_EQ(t.update(2.40f, 1u * kSecond, kBarV, kAdvisory), CHARGED);
    EXPECT_EQ(t.update(NAN, 2u * kSecond, kBarV, kAdvisory), NO_READING);
    // The step is measured from the next good reading, not across the gap.
    EXPECT_EQ(t.update(0.90f, 3u * kSecond, kBarV, kAdvisory), CHARGING);
    EXPECT_EQ(t.jump_history, 0u);
}

TEST(HoldupPolicy1485, ThePlainTrackerKeepsTheVoltageRuleAlone)
{
    Tracker t;
    const auto v = feed(t, {2.54f, 3.09f, 2.31f, 2.77f, 2.12f, 2.51f});
    for (uint8_t s : v) EXPECT_NE(s, NOT_FITTED);
}

TEST(HoldupPolicy1485, TheThresholdSitsBetweenARealCapAndAnEmptyNode)
{
    // Twice the steepest real change (0.15 V/s) and under the smallest empty-
    // node swing (0.39 V) seen: the margin the constant is chosen for.
    EXPECT_GT(config::HOLDUP_JUMP_V, 2.0f * 0.15f - 0.001f);
    EXPECT_LT(config::HOLDUP_JUMP_V, 0.39f);
    EXPECT_LE(config::HOLDUP_JUMPS_NOT_FITTED, FIT_WINDOW);
    EXPECT_GT(config::HOLDUP_CALM_TO_CLEAR, FIT_WINDOW);   // the window empties before it clears
}
