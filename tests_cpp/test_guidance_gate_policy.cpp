// Coast-guidance gate, launch origin and heading trust (guidance_gate_policy.h).
//
// GTV Blue and GTV White, 2026-11-01: the guidance origin was the power-up
// datum 43 m from the rail, Blue's heading was ~90 deg off on an amber
// magnetometer, and White's tilt latch tripped on the first control tick with
// nothing in the log to say so.  These pin each of those.

#include <gtest/gtest.h>

#include <cmath>

#include "guidance_gate_policy.h"

using namespace GuidanceGatePolicy;

namespace {

Inputs clear()
{
    Inputs in;
    in.ekf_initialized = true;
    in.ekf_healthy     = true;
    in.have_ref_pos    = true;
    in.origin_valid    = true;
    in.heading_trusted = true;
    in.tilt_latched    = false;
    in.speed_mps       = 50.0f;
    in.min_speed_mps   = 15.0f;
    return in;
}

constexpr double DEG = 3.14159265358979323846 / 180.0;

}  // namespace

TEST(GuidanceGate, AllClearSteers)
{
    EXPECT_EQ(inhibitFlags(clear()), 0u);
}

TEST(GuidanceGate, EachReasonHasItsOwnBit)
{
    Inputs in = clear(); in.tilt_latched = true;
    EXPECT_EQ(inhibitFlags(in), GF_TILT);
    in = clear(); in.speed_mps = 15.0f;               // at the floor is not above it
    EXPECT_EQ(inhibitFlags(in), GF_SLOW);
    in = clear(); in.ekf_healthy = false;
    EXPECT_EQ(inhibitFlags(in), GF_EKF);
    in = clear(); in.ekf_initialized = false;
    EXPECT_EQ(inhibitFlags(in), GF_EKF);
    in = clear(); in.have_ref_pos = false;
    EXPECT_EQ(inhibitFlags(in), GF_NO_ORIGIN);
    in = clear(); in.origin_valid = false;            // a reboot in flight
    EXPECT_EQ(inhibitFlags(in), GF_NO_ORIGIN);
    in = clear(); in.heading_trusted = false;         // GTV Blue
    EXPECT_EQ(inhibitFlags(in), GF_NO_HEADING);
}

TEST(GuidanceGate, WhiteFirstControlTick)
{
    // T+0.600 s: tilt 35.7 deg had latched the 15 deg boost limit, 38.5 m/s.
    Inputs in = clear();
    in.tilt_latched = true;
    in.speed_mps = 38.5f;
    EXPECT_EQ(inhibitFlags(in), GF_TILT);
}

TEST(GuidanceGate, NanSpeedIsSlow)
{
    Inputs in = clear(); in.speed_mps = NAN;
    EXPECT_EQ(inhibitFlags(in) & GF_SLOW, GF_SLOW);
}

TEST(GuidanceGate, ReasonBitsLeaveTheLegacyBitsAlone)
{
    EXPECT_EQ(GF_REASONS & (GF_ACTIVE | GF_BURNOUT), 0u);
    EXPECT_EQ(GF_REASONS | GF_ACTIVE | GF_BURNOUT, 0xFFu);
}

TEST(GuidanceOrigin, PowerUpAwayFromTheRail)
{
    // Datum at the prep table; the rail 42.4 m south and 6.7 m east of it,
    // as GTV White's filter read on the rail.
    const double ref_lat = 40.1637712 * DEG, ref_lon = -74.0809640 * DEG, ref_alt = 25.0;
    constexpr double R = 6378137.0;
    const double lat = ref_lat + (-42.4) / (R + ref_alt);
    const double lon = ref_lon + 6.7 / ((R + ref_alt) * std::cos(ref_lat));
    const Origin o = launchOrigin(lat, lon, ref_lat, ref_lon, ref_alt, true, true);
    ASSERT_TRUE(o.valid);
    EXPECT_NEAR(o.e_m, 6.7f, 1e-3f);
    EXPECT_NEAR(o.n_m, -42.4f, 1e-3f);
    // On the rail, guidance's position is (0,0) -- not the 43 m it was handed.
    EXPECT_NEAR(6.7f - o.e_m, 0.0f, 1e-3f);
    EXPECT_NEAR(-42.4f - o.n_m, 0.0f, 1e-3f);
}

TEST(GuidanceOrigin, NoDatumOrNoFilterIsNoOrigin)
{
    EXPECT_FALSE(launchOrigin(0.7, -1.3, 0.7, -1.3, 0.0, false, true).valid);
    EXPECT_FALSE(launchOrigin(0.7, -1.3, 0.7, -1.3, 0.0, true, false).valid);
    EXPECT_FALSE(launchOrigin(NAN, -1.3, 0.7, -1.3, 0.0, true, true).valid);
}

TEST(GuidanceHeading, OnlyAHealthyMagnetometerIsTrusted)
{
    constexpr uint8_t NA = 0, OK = 1, DEGRADED = 2, BAD = 3;   // SensorHealthState
    EXPECT_TRUE(launchHeadingTrusted(OK, OK));
    EXPECT_FALSE(launchHeadingTrusted(DEGRADED, OK));   // GTV Blue: 86 uT vs 52
    EXPECT_FALSE(launchHeadingTrusted(BAD, OK));
    EXPECT_FALSE(launchHeadingTrusted(NA, OK));         // no magnetometer
}
