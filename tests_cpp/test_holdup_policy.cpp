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

TEST(HoldupPolicy1166, TheBarIsInclusiveAndAReadingFailureIsNone)
{
    Tracker t;
    EXPECT_EQ(t.update(2.19f, kSecond, kBarV, kAdvisory), CHARGING);
    EXPECT_EQ(t.update(2.20f, 2 * kSecond, kBarV, kAdvisory), CHARGED);
    EXPECT_EQ(t.update(NAN, 3 * kSecond, kBarV, kAdvisory), NONE);   // ADC failed this time
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

TEST(HoldupPolicy1166, TraceIsDenseThroughTheRampThenSparse)
{
    const uint32_t ramp = 300u * kSecond, fast = 5u * kSecond, slow = 60u * kSecond;
    EXPECT_TRUE(traceDue(0, 0, false, ramp, fast, slow));                        // first ever
    EXPECT_FALSE(traceDue(4u * kSecond, 0, true, ramp, fast, slow));
    EXPECT_TRUE(traceDue(5u * kSecond, 0, true, ramp, fast, slow));
    EXPECT_FALSE(traceDue(ramp + 30u * kSecond, ramp, true, ramp, fast, slow));   // sparse after the ramp
    EXPECT_TRUE(traceDue(ramp + 60u * kSecond, ramp, true, ramp, fast, slow));
    EXPECT_EQ(stateName(NOT_CHARGING), std::string("NOT CHARGING"));
    EXPECT_EQ(stateName(CHARGED), std::string("charged"));
}
