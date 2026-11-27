// #1775: the pyro channel config's acceptance check (PyroConfigGate.h), which
// the FC's apply path and boot load, its pad health bits (pyroChannelHealth),
// and the OC's command 34 and boot load all use.
//
// Before, an enabled channel could be applied and saved with a mode other than
// 0 or 1, a NaN value, or an altitude of 0 m or less, and each of those leaves
// it unable to fire when it is needed. The last section here drives the FC's
// own main-deploy gate to show what the refused altitudes would have done.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>

#include "MainDeployGate.h"
#include "PyroConfigGate.h"
#include "RocketComputerTypes.h"

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr uint8_t kTime = PYRO_TRIGGER_TIME_AFTER_APOGEE;
constexpr uint8_t kAlt  = PYRO_TRIGGER_ALTITUDE_ON_DESCENT;

void setCh(PyroConfigData& c, int ch, uint8_t en, uint8_t mode, float v)
{
    switch (ch)
    {
        case 1: c.ch1_enabled = en; c.ch1_trigger_mode = mode; c.ch1_trigger_value = v; break;
        case 2: c.ch2_enabled = en; c.ch2_trigger_mode = mode; c.ch2_trigger_value = v; break;
        case 3: c.ch3_enabled = en; c.ch3_trigger_mode = mode; c.ch3_trigger_value = v; break;
        case 4: c.ch4_enabled = en; c.ch4_trigger_mode = mode; c.ch4_trigger_value = v; break;
    }
}

// A drogue at apogee and a main at 150 m: the usual pair.
PyroConfigData goodPair()
{
    PyroConfigData c = {};
    setCh(c, 1, 1, kTime, 0.0f);
    setCh(c, 2, 1, kAlt, 150.0f);
    return c;
}

}  // namespace

// ---------------------------------------------------------------------------
// One channel's trigger
// ---------------------------------------------------------------------------

TEST(PyroTriggerRc, ADelayFromZeroTo120SecondsIsAccepted)
{
    for (float v : {0.0f, -0.0f, 0.5f, 1.0f, 1.5f, 100.0f, PYRO_TRIGGER_MAX_DELAY_S})
        EXPECT_EQ(pyroTriggerRc(kTime, v), PYRO_CFG_OK) << v;
}

TEST(PyroTriggerRc, ANegativeOrLongerDelayIsRefused)
{
    for (float v : {-0.001f, -1.0f, std::nextafter(PYRO_TRIGGER_MAX_DELAY_S, 1e9f), 121.0f,
                    1500.0f /* 1.5 s sent in ms */, 1e30f})
        EXPECT_EQ(pyroTriggerRc(kTime, v), PYRO_CFG_REJ_DELAY) << v;
}

TEST(PyroTriggerRc, AnAltitudeFrom30MetersUpIsAccepted)
{
    for (float v : {PYRO_TRIGGER_MIN_ALTITUDE_M, 30.48f /* 100 ft */, 150.0f, 300.0f, 3000.0f,
                    1e6f})
        EXPECT_EQ(pyroTriggerRc(kAlt, v), PYRO_CFG_OK) << v;
}

TEST(PyroTriggerRc, AnAltitudeBelow30MetersIsRefused)
{
    for (float v : {std::nextafter(PYRO_TRIGGER_MIN_ALTITUDE_M, 0.0f), 29.87f /* 98 ft */, 10.0f,
                    0.5f, 0.0f, -0.0f, -5.0f, -1e30f})
        EXPECT_EQ(pyroTriggerRc(kAlt, v), PYRO_CFG_REJ_ALTITUDE) << v;
}

TEST(PyroTriggerRc, ANonFiniteValueIsRefusedInEitherMode)
{
    for (uint8_t mode : {kTime, kAlt})
        for (float v : {kNaN, -kNaN, kInf, -kInf})
            EXPECT_EQ(pyroTriggerRc(mode, v), PYRO_CFG_REJ_VALUE) << int(mode) << " " << v;
}

TEST(PyroTriggerRc, AModeOtherThanZeroOrOneIsRefusedWhateverTheValue)
{
    for (int mode = 2; mode <= 255; ++mode)
        for (float v : {0.0f, 1.0f, 150.0f, kNaN})
            ASSERT_EQ(pyroTriggerRc((uint8_t)mode, v), PYRO_CFG_REJ_MODE) << mode << " " << v;
}

// ---------------------------------------------------------------------------
// A whole config
// ---------------------------------------------------------------------------

TEST(PyroConfigCheck, AnAllDisabledConfigIsAccepted)
{
    const PyroConfigData zero = {};
    const PyroConfigVerdict v = pyroConfigCheck(zero);
    EXPECT_EQ(v.rc, PYRO_CFG_OK);
    EXPECT_EQ(v.channel, 0);
}

TEST(PyroConfigCheck, ADisabledChannelIsNeverChecked)
{
    // The trigger loop, the flight timeout and the health bits skip it, and
    // the apps send whatever the profile last held for it.
    PyroConfigData c = goodPair();
    setCh(c, 3, 0, 7, kNaN);
    setCh(c, 4, 0, kAlt, 0.0f);
    EXPECT_EQ(pyroConfigCheck(c).rc, PYRO_CFG_OK);
}

TEST(PyroConfigCheck, AnyNonzeroEnabledByteIsEnabled)
{
    // As the FC's pyroChEnabled() and the OC's cache read it.
    for (int en = 1; en <= 255; ++en)
    {
        PyroConfigData c = goodPair();
        setCh(c, 3, (uint8_t)en, kAlt, 0.0f);
        const PyroConfigVerdict v = pyroConfigCheck(c);
        ASSERT_EQ(v.rc, PYRO_CFG_REJ_ALTITUDE) << en;
        ASSERT_EQ(v.channel, 3) << en;
    }
}

TEST(PyroConfigCheck, EachChannelIsChecked)
{
    for (int ch = 1; ch <= 4; ++ch)
    {
        PyroConfigData c = {};
        setCh(c, ch, 1, kTime, kNaN);
        const PyroConfigVerdict v = pyroConfigCheck(c);
        EXPECT_EQ(v.rc, PYRO_CFG_REJ_VALUE) << ch;
        EXPECT_EQ(v.channel, ch);
    }
}

TEST(PyroConfigCheck, TheFirstFailingEnabledChannelIsNamed)
{
    PyroConfigData c = goodPair();
    setCh(c, 4, 1, 2, 1.0f);
    setCh(c, 3, 1, kTime, 500.0f);
    const PyroConfigVerdict v = pyroConfigCheck(c);
    EXPECT_EQ(v.rc, PYRO_CFG_REJ_DELAY);
    EXPECT_EQ(v.channel, 3);
}

TEST(PyroConfigCheck, TheIssuesConfigsAreRefused)
{
    struct Case { const char* what; int ch; uint8_t mode; float v; PyroConfigRc rc; };
    const Case cases[] = {
        // the dashboard sheet's cleared altitude field, sent as 0
        { "main at 0 m", 2, kAlt, 0.0f, PYRO_CFG_REJ_ALTITUDE },
        // channel 3 enabled with its time-mode 0.0 and the picker moved to altitude
        { "channel 3 switched to altitude", 3, kAlt, 0.0f, PYRO_CFG_REJ_ALTITUDE },
        { "main below the pad", 2, kAlt, -5.0f, PYRO_CFG_REJ_ALTITUDE },
        // pasted on iOS, or Android's toFloatOrNull("NaN")
        { "NaN delay", 1, kTime, kNaN, PYRO_CFG_REJ_VALUE },
        { "NaN altitude", 2, kAlt, kNaN, PYRO_CFG_REJ_VALUE },
        // a corrupted record or another client
        { "mode 2", 2, 2, 150.0f, PYRO_CFG_REJ_MODE },
    };
    for (const Case& k : cases)
    {
        PyroConfigData c = goodPair();
        setCh(c, k.ch, 1, k.mode, k.v);
        const PyroConfigVerdict v = pyroConfigCheck(c);
        EXPECT_EQ(v.rc, k.rc) << k.what;
        EXPECT_EQ(v.channel, k.ch) << k.what;
    }
}

TEST(PyroConfigCheck, TheAppsDefaultsPassEnabled)
{
    // RocketProfile / RocketConfig defaults: every channel time mode, at 1.0,
    // 100.0, 0.0 and 0.0 s. Enabling one as it stands is never refused.
    PyroConfigData c = {};
    setCh(c, 1, 1, kTime, 1.0f);
    setCh(c, 2, 1, kTime, 100.0f);
    setCh(c, 3, 1, kTime, 0.0f);
    setCh(c, 4, 1, kTime, 0.0f);
    EXPECT_EQ(pyroConfigCheck(c).rc, PYRO_CFG_OK);
}

TEST(PyroConfigCheck, TheRuleNamesStateTheBounds)
{
    char delay[32], alt[32];
    std::snprintf(delay, sizeof(delay), "0-%.0f s", (double)PYRO_TRIGGER_MAX_DELAY_S);
    std::snprintf(alt, sizeof(alt), "below %.0f m", (double)PYRO_TRIGGER_MIN_ALTITUDE_M);
    EXPECT_NE(std::string(pyroConfigRcName(PYRO_CFG_REJ_DELAY)).find(delay), std::string::npos);
    EXPECT_NE(std::string(pyroConfigRcName(PYRO_CFG_REJ_ALTITUDE)).find(alt), std::string::npos);
    EXPECT_STREQ(pyroConfigRcName(PYRO_CFG_OK), "ok");
    for (PyroConfigRc rc : {PYRO_CFG_REJ_MODE, PYRO_CFG_REJ_VALUE})
        EXPECT_STRNE(pyroConfigRcName(rc), "unknown");
}

TEST(PyroConfigCheck, TheChannelAccessorReadsEachField)
{
    PyroConfigData c = {};
    for (int ch = 1; ch <= 4; ++ch) setCh(c, ch, (uint8_t)(ch % 2), (uint8_t)(ch + 10), 1.5f * ch);
    for (int i = 0; i < 4; ++i)
    {
        const PyroConfigChannel k = pyroConfigChannel(c, i);
        EXPECT_EQ(k.enabled, ((i + 1) % 2) != 0) << i;
        EXPECT_EQ(k.mode, i + 11) << i;
        EXPECT_FLOAT_EQ(k.value, 1.5f * (i + 1)) << i;
    }
    EXPECT_FALSE(pyroConfigChannel(c, 4).enabled);
}

// ---------------------------------------------------------------------------
// The pad health bit (SH_PYRO)
// ---------------------------------------------------------------------------

TEST(PyroChannelHealth, ADisabledChannelIsLeftOut)
{
    // NA is excluded from the go/no-go, whatever the channel holds.
    for (bool known : {false, true})
        for (bool cont : {false, true})
        {
            EXPECT_EQ(pyroChannelHealth(false, kAlt, 0.0f, known, cont), SH_NA);
            EXPECT_EQ(pyroChannelHealth(false, 9, kNaN, known, cont), SH_NA);
        }
}

TEST(PyroChannelHealth, AGoodTriggerIsDecidedByContinuity)
{
    EXPECT_EQ(pyroChannelHealth(true, kAlt, 150.0f, false, false), SH_DEGRADED);
    EXPECT_EQ(pyroChannelHealth(true, kAlt, 150.0f, true, true), SH_OK);
    EXPECT_EQ(pyroChannelHealth(true, kAlt, 150.0f, true, false), SH_BAD);
    EXPECT_EQ(pyroChannelHealth(true, kTime, 0.0f, true, true), SH_OK);
}

TEST(PyroChannelHealth, ARefusedTriggerIsBadWhateverItsContinuity)
{
    // The issue's pad: continuity present, card green, main at 0 m.
    for (bool known : {false, true})
        for (bool cont : {false, true})
        {
            EXPECT_EQ(pyroChannelHealth(true, kAlt, 0.0f, known, cont), SH_BAD);
            EXPECT_EQ(pyroChannelHealth(true, kTime, kNaN, known, cont), SH_BAD);
            EXPECT_EQ(pyroChannelHealth(true, 2, 150.0f, known, cont), SH_BAD);
            EXPECT_EQ(pyroChannelHealth(true, kTime, 500.0f, known, cont), SH_BAD);
        }
}

// ---------------------------------------------------------------------------
// What the refused altitudes do in flight: the FC's own deploy gate
// ---------------------------------------------------------------------------

namespace {

// A descent from 1000 m at 20 m/s, 50 Hz, on a trusted barometer, after
// apogee. Returns the altitude the gate first fires at, or NaN if never.
float fireAltitude(float threshold_m)
{
    MainDeployGate::State  st;
    MainDeployGate::Inputs in;
    in.apogee        = true;
    in.baro_healthy  = true;
    in.baro_rate_mps = -20.0f;
    bool on_ground = false;
    for (int k = 0; k <= 50 * 60; ++k)
    {
        in.now_ms     = 100000u + 20u * (uint32_t)k;
        in.baro_alt_m = std::max(1000.0f - 0.4f * (float)k, 0.0f);   // 20 m/s at 50 Hz
        // The touchdown sample still reads descending (the estimated rate lags
        // the stop); from the next one the rocket is at rest.
        if (on_ground) in.baro_rate_mps = 0.0f;
        on_ground = in.baro_alt_m <= 0.0f;
        MainDeployGate::step(st, in);
        if (MainDeployGate::evaluate(st, in, threshold_m) != MainDeployGate::Source::None)
            return in.baro_alt_m;
    }
    return kNaN;
}

}  // namespace

TEST(PyroConfigInFlight, TheLowestAcceptedAltitudeFires30MetersUp)
{
    const float at = fireAltitude(PYRO_TRIGGER_MIN_ALTITUDE_M);
    ASSERT_FALSE(std::isnan(at));
    EXPECT_LE(at, PYRO_TRIGGER_MIN_ALTITUDE_M);
    EXPECT_GT(at, PYRO_TRIGGER_MIN_ALTITUDE_M - 0.5f);   // one 20 ms step
}

TEST(PyroConfigInFlight, TheRefusedAltitudesFireAtTouchdownOrNever)
{
    // 0 m: the touchdown sample, the one that reaches the pad's altitude still
    // descending (#1775's harness: 50.0 s into this descent).
    const float at0 = fireAltitude(0.0f);
    ASSERT_FALSE(std::isnan(at0));
    EXPECT_GT(at0, -0.5f);
    EXPECT_LT(at0, 0.5f);
    // Below the pad, or NaN: never.
    EXPECT_TRUE(std::isnan(fireAltitude(-5.0f)));
    EXPECT_TRUE(std::isnan(fireAltitude(kNaN)));
    EXPECT_NE(pyroTriggerRc(kAlt, 0.0f), PYRO_CFG_OK);
    EXPECT_NE(pyroTriggerRc(kAlt, -5.0f), PYRO_CFG_OK);
    EXPECT_NE(pyroTriggerRc(kAlt, kNaN), PYRO_CFG_OK);
}
