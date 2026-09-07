// #1166: the hold-up capacitor verdict. The dangerous row is a converter
// that never charges the cap while everything else on the board looks fine —
// the policy must turn "flat after the window" into LOW, must NOT raise it
// during a normal charge ramp, and must clear itself the moment the cap
// actually crosses the threshold, however late.

#include <gtest/gtest.h>

#include <cmath>

#include "scap_holdup_policy.h"

using namespace ScapHoldupPolicy;

static constexpr uint32_t S = 1000;  // ms per second

TEST(ScapHoldupPolicy, WireValueZeroIsReserved) {
    // A zero-initialised TelemetryData must emit nothing (the #850 lesson).
    EXPECT_EQ(HU_NOT_REPORTED, 0);
    EXPECT_NE(HU_CHARGING, 0);
    EXPECT_NE(HU_CHARGED, 0);
    EXPECT_NE(HU_LOW, 0);
    EXPECT_NE(HU_NO_READING, 0);
}

TEST(ScapHoldupPolicy, ColdStartRampIsChargingThenCharged) {
    // 100 mA into 5 F: ~20 mV/s. Nothing is raised on the way up.
    HoldupState s = HU_NOT_REPORTED;
    s = next(s, 0.02f, 0 * S);    EXPECT_EQ(s, HU_CHARGING);
    s = next(s, 0.60f, 30 * S);   EXPECT_EQ(s, HU_CHARGING);
    s = next(s, 1.20f, 60 * S);   EXPECT_EQ(s, HU_CHARGING);
    s = next(s, 1.80f, 90 * S);   EXPECT_EQ(s, HU_CHARGING);
    s = next(s, 2.19f, 109 * S);  EXPECT_EQ(s, HU_CHARGING);
    s = next(s, 2.21f, 111 * S);  EXPECT_EQ(s, HU_CHARGED);
    s = next(s, 2.50f, 125 * S);  EXPECT_EQ(s, HU_CHARGED);
}

TEST(ScapHoldupPolicy, NeverChargesBecomesLowExactlyAtTheWindow) {
    // The #999 shape: bypass FET as an LDO, cap flat at a few hundred mV.
    HoldupState s = HU_NOT_REPORTED;
    s = next(s, 0.30f, 0);                 EXPECT_EQ(s, HU_CHARGING);
    s = next(s, 0.30f, kWindowMs - 1);     EXPECT_EQ(s, HU_CHARGING);
    s = next(s, 0.30f, kWindowMs);         EXPECT_EQ(s, HU_LOW);
    s = next(s, 0.30f, 20 * 60 * S);       EXPECT_EQ(s, HU_LOW);
}

TEST(ScapHoldupPolicy, LateChargeClearsLow) {
    // A slow or fat cap that crosses after the window: the advisory lifts on
    // the crossing sample, not on some later timer.
    HoldupState s = HU_LOW;
    s = next(s, 2.15f, 200 * S);  EXPECT_EQ(s, HU_LOW);      // inside the band is not a crossing
    s = next(s, 2.20f, 205 * S);  EXPECT_EQ(s, HU_CHARGED);
}

TEST(ScapHoldupPolicy, ChargedHoldsThroughTheHysteresisBand) {
    HoldupState s = HU_CHARGED;
    s = next(s, 2.15f, 300 * S);  EXPECT_EQ(s, HU_CHARGED);  // >= kLowV holds
    s = next(s, 2.10f, 301 * S);  EXPECT_EQ(s, HU_CHARGED);  // exactly kLowV holds
    s = next(s, 2.09f, 302 * S);  EXPECT_EQ(s, HU_LOW);      // below it: a real drop
}

TEST(ScapHoldupPolicy, ChargedThatFallsIsLowEvenInsideTheWindow) {
    // An OC reboot (self-OTA, fault) restarts the window with a charged cap;
    // if the cap then drains — pack bounce, bridge in use — that is LOW now,
    // not "charging".
    HoldupState s = HU_NOT_REPORTED;
    s = next(s, 2.45f, 2 * S);    EXPECT_EQ(s, HU_CHARGED);
    s = next(s, 1.50f, 10 * S);   EXPECT_EQ(s, HU_LOW);
    s = next(s, 2.25f, 50 * S);   EXPECT_EQ(s, HU_CHARGED);  // recharged: clears
}

TEST(ScapHoldupPolicy, NaNIsNoReadingAndRecoversIntoTheWindowRule) {
    HoldupState s = HU_CHARGING;
    s = next(s, NAN, 30 * S);              EXPECT_EQ(s, HU_NO_READING);
    s = next(s, NAN, 400 * S);             EXPECT_EQ(s, HU_NO_READING);
    // ADC back inside the window: charging; outside it: low.
    EXPECT_EQ(next(HU_NO_READING, 1.0f, 30 * S), HU_CHARGING);
    EXPECT_EQ(next(HU_NO_READING, 1.0f, kWindowMs), HU_LOW);
    EXPECT_EQ(next(HU_NO_READING, 2.4f, kWindowMs), HU_CHARGED);
    // A charged cap whose ADC blinks is NO_READING for that sample and
    // CHARGED again on the next good one.
    EXPECT_EQ(next(HU_CHARGED, NAN, 500 * S), HU_NO_READING);
    EXPECT_EQ(next(HU_NO_READING, 2.4f, 501 * S), HU_CHARGED);
}

TEST(ScapHoldupPolicy, WindowCoversTheSlowCornerOfTheChargeRamp) {
    // The window must outlast the slowest honest ramp, or a healthy board
    // with a fat cap raises a false LOW before it crosses. EDLC tolerance is
    // -10/+30 %, the ICHG code about -10 %: 6.5 F at 90 mA to 2.2 V.
    const double c_max_f   = 5.0 * 1.30;
    const double i_min_a   = 0.100 * 0.90;
    const double t_cross_s = c_max_f * kChargedV / i_min_a;   // ~159 s
    EXPECT_GE(kWindowMs, (uint32_t)(t_cross_s * 1000.0 + 15000.0))
        << "window " << kWindowMs << " ms vs worst-case crossing " << t_cross_s << " s";
    // ...and the nominal ramp is well inside it.
    const double t_nominal_s = 5.0 * kChargedV / 0.100;        // 110 s
    EXPECT_LT(t_nominal_s * 1000.0, kWindowMs / 2.0 + 30000.0);
    EXPECT_LT(kLowV, kChargedV);
}

TEST(ScapHoldupPolicy, LogCadenceFollowsTheRamp) {
    // First sample always logs.
    EXPECT_TRUE(shouldLog(false, false, 0.02f, NAN, 0, 0));
    // Steady inside the delta and the period: quiet. (Values sit clear of
    // the 50 mV boundary — single-precision 2.500 - 2.450 lands a hair
    // under 0.05, and the ramp never needs the exact edge anyway.)
    EXPECT_FALSE(shouldLog(false, true, 2.500f, 2.480f, 10 * S, 0));
    EXPECT_FALSE(shouldLog(false, true, 2.500f, 2.460f, 10 * S, 0));
    // Moved by the delta: a line, in either direction.
    EXPECT_TRUE(shouldLog(false, true, 2.500f, 2.440f, 10 * S, 0));
    EXPECT_TRUE(shouldLog(false, true, 2.390f, 2.450f, 10 * S, 0));
    // Steady but the period elapsed: a line.
    EXPECT_TRUE(shouldLog(false, true, 2.500f, 2.500f, kLogPeriodMs, 0));
    EXPECT_FALSE(shouldLog(false, true, 2.500f, 2.500f, kLogPeriodMs - 1, 0));
    // A state transition always logs, however small the move.
    EXPECT_TRUE(shouldLog(true, true, 2.200f, 2.199f, 1 * S, 0));
}

TEST(ScapHoldupPolicy, DeadAdcLogsOnThePeriodNotEverySecond) {
    // NaN to NaN is "steady": no line until the period, so a broken sense
    // does not fill the console.
    EXPECT_FALSE(shouldLog(false, true, NAN, NAN, 5 * S, 0));
    EXPECT_TRUE(shouldLog(false, true, NAN, NAN, kLogPeriodMs, 0));
    // The edges into and out of NaN are worth a line each.
    EXPECT_TRUE(shouldLog(false, true, NAN, 2.4f, 1 * S, 0));
    EXPECT_TRUE(shouldLog(false, true, 2.4f, NAN, 1 * S, 0));
}

TEST(ScapHoldupPolicy, LogPeriodSurvivesMillisWrap) {
    // now < last after a uint32 wrap: unsigned subtraction still measures
    // the true elapsed time.
    const uint32_t last = 0xFFFFFF00u;
    EXPECT_FALSE(shouldLog(false, true, 2.5f, 2.5f, last + 1000u, last));
    EXPECT_TRUE(shouldLog(false, true, 2.5f, 2.5f, last + kLogPeriodMs, last));
}

TEST(ScapHoldupPolicy, NamesAreDistinct) {
    EXPECT_STRNE(name(HU_CHARGING), name(HU_CHARGED));
    EXPECT_STRNE(name(HU_CHARGED), name(HU_LOW));
    EXPECT_STRNE(name(HU_LOW), name(HU_NO_READING));
    EXPECT_STREQ(name(HU_NOT_REPORTED), "n/a");
}
