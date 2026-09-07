// #1166: the out computer's verdict on the hold-up supercap — charged, still
// charging, or never charged — from a stream of V_SCAP readings and uptime.
// The mini's TPS61094 charges the 5 F cap at 100 mA to 2.5 V in about two
// minutes; a cap that silently never charges looks exactly like one that did,
// so this is the one thing on the board that says which.
#include <gtest/gtest.h>
#include "holdup_policy.h"
#include <cmath>

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
