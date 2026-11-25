// InertialVertical: the IMU's own vertical velocity, height and pitch for the
// apogee vote's accelerometer and gyro voters (owner's design, 2026-11-19).
//
// A planar flight model makes the samples: a rail at a set angle, a motor of
// a set acceleration and burn, drag on a standard atmosphere, and a nose that
// follows the flight path above 30 m/s.  The body-frame specific force is the
// true acceleration plus g, projected on the true attitude, and the gyro is
// the true pitch rate, so a perfect IMU must reproduce the truth up to the
// integration's own rounding.

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <vector>

#include "InertialVertical.h"

namespace {

constexpr float kG = 9.80665f;
constexpr double kPi = 3.14159265358979;

float rho(float h)
{
    if (h < 11000.0f)
        return 1.225f * std::pow((288.15f - 0.0065f * h) / 288.15f, 4.2559f);
    return 0.3639f * std::exp(-(h - 11000.0f) / 6341.6f);
}

struct Sample
{
    uint32_t t_us;
    float f[3];
    float w[3];
    // truth
    float vz, h, pitch;
    bool  burning;
};

struct Flight
{
    float rail_deg = 85.0f;     // from horizontal
    float acc_g = 35.0f;        // motor, over weight
    float burn_s = 2.3f;
    float k = 1.16e-4f;         // A Cd / (2 m), m^2/kg
    float pad_s = 3.0f;
    float rate_hz = 1000.0f;
    float end_s = 60.0f;        // after ignition
};

// Planar: nav x east, z up; body x out the nose, body y = nav y (left facing
// east), so a nose-up rate is about -y.
std::vector<Sample> fly(const Flight& fl)
{
    std::vector<Sample> out;
    const double dt = 1.0 / fl.rate_hz;
    double t = 0, x = 0, z = 0, vx = 0, vz = 0;
    double th = fl.rail_deg * kPi / 180.0, th_prev = th;
    const double t_ign = fl.pad_s;
    for (int n = 0; t < fl.pad_s + fl.end_s; ++n, t += dt)
    {
        const bool burning = t >= t_ign && t < t_ign + fl.burn_s;
        const double v = std::sqrt(vx * vx + vz * vz);
        const double T = burning ? fl.acc_g * kG : 0.0;
        const double D = rho((float)z) * v * v * fl.k;
        double ax = T * std::cos(th) - (v > 0 ? D * vx / v : 0);
        const double r = 6371000.0 / (6371000.0 + (z > 0 ? z : 0));
        const double g = kG * r * r;   // gravity falls off with height, as the integrator's
        double az = T * std::sin(th) - (v > 0 ? D * vz / v : 0) - g;
        const bool on_pad = t < t_ign || (z <= 0 && az < 0);
        if (on_pad)
        {
            ax = az = 0;   // the rail holds it
            vx = vz = 0;
        }
        // specific force = a + g up, projected on the body
        const double fx_n = ax, fz_n = az + g;
        Sample s{};
        s.t_us = (uint32_t)std::llround(t * 1e6);
        s.f[0] = (float)(fx_n * std::cos(th) + fz_n * std::sin(th));
        s.f[1] = 0.0f;
        s.f[2] = (float)(-fx_n * std::sin(th) + fz_n * std::cos(th));
        s.w[0] = 0.0f;
        s.w[1] = (float)(-(th - th_prev) / dt * 180.0 / kPi);
        s.w[2] = 0.0f;
        s.burning = burning;
        // advance (semi-implicit, as the integrator)
        if (!on_pad)
        {
            vx += ax * dt;
            vz += az * dt;
            x += vx * dt;
            z += vz * dt;
        }
        s.vz = (float)vz;
        s.h = (float)z;
        s.pitch = (float)th;
        out.push_back(s);
        th_prev = th;
        const double vn = std::sqrt(vx * vx + vz * vz);
        if (!on_pad && vn > 30.0)
            th = std::atan2(vz, vx);
    }
    return out;
}

struct Run
{
    InertialVertical iv;
    size_t i_launch = 0;
};

// Feeds every sample, calling launch() `latency_s` after ignition.  `mutate`
// may alter a sample before it is fed (sensor errors).
void feed(InertialVertical& iv, const std::vector<Sample>& s, const Flight& fl, float latency_s,
          const std::function<void(Sample&)>& mutate = nullptr, bool railed = false)
{
    const uint32_t t_launch = (uint32_t)std::llround((fl.pad_s + latency_s) * 1e6);
    bool launched = false;
    for (Sample x : s)
    {
        if (mutate)
            mutate(x);
        iv.update(x.t_us, x.f, x.w, railed);
        if (!launched && x.t_us >= t_launch)
        {
            iv.launch(x.t_us);
            launched = true;
        }
    }
}

size_t apogeeIndex(const std::vector<Sample>& s, const Flight& fl)
{
    for (size_t i = (size_t)(fl.pad_s * fl.rate_hz) + 100; i < s.size(); ++i)
        if (s[i].vz <= 0.0f)
            return i;
    return s.size() - 1;
}

}  // namespace

TEST(InertialVertical, LevelsOnThePad)
{
    Flight fl;
    fl.end_s = 0.0f;
    const auto s = fly(fl);
    InertialVertical iv;
    for (const auto& x : s)
        iv.update(x.t_us, x.f, x.w, false);
    EXPECT_EQ(iv.state(), InertialVertical::State::Pad);
    EXPECT_FALSE(iv.healthy());   // healthy only in flight
    EXPECT_NEAR(iv.pitchRad() * 180.0 / kPi, 85.0, 0.05);
    EXPECT_NEAR(iv.vzUp(), 0.0f, 1e-3f);
    EXPECT_NEAR(iv.gravity(), kG, 1e-3f);
}

TEST(InertialVertical, UpOf_IsTheUpComponentOfAnyBodyVector)
{
    // #1755: the flight computer asks whether the measured thrust axis points
    // up (thrust) or down (coast drag), in whatever frame the board flies.
    Flight fl;
    fl.end_s = 0.0f;
    const auto s = fly(fl);
    InertialVertical iv;
    for (const auto& x : s)
        iv.update(x.t_us, x.f, x.w, false);
    const float x_body[3] = {1.0f, 0.0f, 0.0f};
    EXPECT_NEAR(iv.upOf(x_body), std::sin(iv.pitchRad()), 1e-5f);
    // The pad's specific force points straight up; its reverse straight down.
    const auto& f = s.back().f;
    const float n = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    const float up[3] = {f[0] / n, f[1] / n, f[2] / n};
    const float down[3] = {-up[0], -up[1], -up[2]};
    EXPECT_NEAR(iv.upOf(up), 1.0f, 1e-3f);
    EXPECT_NEAR(iv.upOf(down), -1.0f, 1e-3f);
    // A body axis square to it is level.
    const float side[3] = {0.0f, 1.0f, 0.0f};
    EXPECT_NEAR(iv.upOf(side), up[1], 1e-3f);

    // A pad tilted on all three body axes, so every term of R's up row has a
    // sign to get right: levelled on it, upOf(v) is v's component along the
    // pad's measured up.
    const float d[3] = {0.3f, -0.5f, 0.81f};
    const float dn = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    const float u3[3] = {d[0] / dn, d[1] / dn, d[2] / dn};
    InertialVertical iv3;
    const float w0[3] = {0.0f, 0.0f, 0.0f};
    const float f3[3] = {kG * u3[0], kG * u3[1], kG * u3[2]};
    for (uint32_t i = 1; i <= 1000; i++)
        iv3.update(i * 1000u, f3, w0, false);
    ASSERT_EQ(iv3.state(), InertialVertical::State::Pad);
    const float ex[3] = {1.0f, 0.0f, 0.0f}, ey[3] = {0.0f, 1.0f, 0.0f}, ez[3] = {0.0f, 0.0f, 1.0f};
    EXPECT_NEAR(iv3.upOf(ex), u3[0], 1e-3f);
    EXPECT_NEAR(iv3.upOf(ey), u3[1], 1e-3f);
    EXPECT_NEAR(iv3.upOf(ez), u3[2], 1e-3f);
    EXPECT_NEAR(iv3.upOf(u3), 1.0f, 1e-3f);
}

TEST(InertialVertical, PerfectImu_TracksAHighFlightToApogee)
{
    Flight fl;   // 35 g for 2.3 s off a 89 deg rail: above the barometer's range
    fl.rail_deg = 89.0f;
    fl.k = 8.0e-5f;
    const auto s = fly(fl);
    InertialVertical iv;
    // follow it sample by sample to compare mid-flight
    const uint32_t t_launch = (uint32_t)std::llround((fl.pad_s + 0.05f) * 1e6);
    const size_t ia = apogeeIndex(s, fl);
    float worst_v = 0, worst_p = 0;
    for (size_t i = 0; i < s.size() && i <= ia; ++i)
    {
        iv.update(s[i].t_us, s[i].f, s[i].w, false);
        if (s[i].t_us >= t_launch && iv.state() == InertialVertical::State::Pad)
            iv.launch(s[i].t_us);
        if (iv.healthy())
        {
            worst_v = std::fmax(worst_v, std::fabs(iv.vzUp() - s[i].vz));
            worst_p = std::fmax(worst_p, std::fabs(iv.pitchRad() - s[i].pitch));
        }
    }
    ASSERT_TRUE(iv.healthy());
    EXPECT_GT(s[ia].h, 10400.0f);   // above the barometer's 25 kPa floor
    EXPECT_LT(worst_v, 0.5f);
    EXPECT_LT(worst_p * 180.0 / kPi, 0.5);
    EXPECT_NEAR(iv.heightM(), s[ia].h, 5.0f);
}

TEST(InertialVertical, LateLaunchDetection_LosesNothing)
{
    // It integrates from the last rest block, so a launch detected 250 ms
    // into the boost (the accel fallback's latency) has the whole boost.
    Flight fl;
    fl.end_s = 10.0f;
    const auto s = fly(fl);
    InertialVertical iv;
    feed(iv, s, fl, 0.25f);
    ASSERT_TRUE(iv.healthy());
    EXPECT_NEAR(iv.vzUp(), s.back().vz, 0.5f);
}

TEST(InertialVertical, ScaleAndBiasErrors_StayWithinTheVotersAllowance)
{
    // 1 % scale on the high-g channel through the boost, 0.01 g bias on
    // every axis and a 0.05 deg/s gyro bias that the pad did not see.  The
    // error at true apogee stays near 10 m/s on the 12.8 km flight: what the
    // single-sensor fallback's 15 m/s margin allows for.
    Flight fl;
    const auto s = fly(fl);
    const size_t ia = apogeeIndex(s, fl);
    InertialVertical iv;
    const uint32_t t_ign = (uint32_t)std::llround(fl.pad_s * 1e6);
    size_t n = 0;
    const uint32_t t_launch = t_ign + 50000;
    for (Sample x : s)
    {
        if (n++ > ia)
            break;
        if (x.burning)
            for (float& c : x.f)
                c *= 1.01f;
        for (float& c : x.f)
            c += 0.01f * kG;
        if (x.t_us >= t_ign)
            x.w[1] += 0.05f;
        iv.update(x.t_us, x.f, x.w, false);
        if (x.t_us >= t_launch && iv.state() == InertialVertical::State::Pad)
            iv.launch(x.t_us);
    }
    ASSERT_TRUE(iv.healthy());
    EXPECT_LT(std::fabs(iv.vzUp()), 12.0f) << "vz at true apogee " << iv.vzUp();
}

TEST(InertialVertical, ASmallFlight_IsTight)
{
    Flight fl;
    fl.acc_g = 10.0f;
    fl.burn_s = 1.6f;
    fl.k = 2.0e-3f;
    fl.end_s = 15.0f;
    const auto s = fly(fl);
    const size_t ia = apogeeIndex(s, fl);
    InertialVertical iv;
    size_t n = 0;
    const uint32_t t_launch = (uint32_t)std::llround((fl.pad_s + 0.05f) * 1e6);
    for (Sample x : s)
    {
        if (n++ > ia)
            break;
        for (float& c : x.f)
            c += 0.01f * kG;   // a 0.01 g bias the pad's g absorbs only on its vertical axis
        iv.update(x.t_us, x.f, x.w, false);
        if (x.t_us >= t_launch && iv.state() == InertialVertical::State::Pad)
            iv.launch(x.t_us);
    }
    ASSERT_TRUE(iv.healthy());
    EXPECT_LT(std::fabs(iv.vzUp()), 3.0f);
}

TEST(InertialVertical, NoRestBeforeLaunch_Fails)
{
    // A restored flight (FC reboot in the air) never saw the pad.
    InertialVertical iv;
    const float f[3] = {0.0f, 0.0f, 2.0f}, w[3] = {0, 0, 0};
    iv.update(1000, f, w, false);
    iv.launch(2000);
    EXPECT_EQ(iv.state(), InertialVertical::State::Failed);
    EXPECT_FALSE(iv.healthy());
}

TEST(InertialVertical, LastRestTooOldAtLaunch_Fails)
{
    // Still for 1 s, then 10 s of handling (never still), then launch.
    InertialVertical iv;
    uint32_t t = 0;
    const float w0[3] = {0, 0, 0};
    for (int i = 0; i < 1000; ++i, t += 1000)
    {
        const float f[3] = {kG, 0, 0};
        iv.update(t, f, w0, false);
    }
    ASSERT_EQ(iv.state(), InertialVertical::State::Pad);
    for (int i = 0; i < 10000; ++i, t += 1000)
    {
        const float f[3] = {kG + ((i & 1) ? 4.0f : -4.0f), 0, 0};
        iv.update(t, f, w0, false);
    }
    iv.launch(t);
    EXPECT_EQ(iv.state(), InertialVertical::State::Failed);
}

TEST(InertialVertical, ThrustInsideABlock_IsNotTakenForRest)
{
    // Ignition at every phase of a rest block: the block holding it never
    // re-levels (which would zero the boost's velocity).
    for (float phase : {0.01f, 0.1f, 0.2f, 0.249f})
    {
        Flight fl;
        fl.pad_s = 2.0f + phase;
        fl.end_s = 1.0f;
        const auto s = fly(fl);
        InertialVertical iv;
        feed(iv, s, fl, 0.3f);
        ASSERT_TRUE(iv.healthy()) << phase;
        EXPECT_NEAR(iv.vzUp(), s.back().vz, 0.5f) << phase;
    }
}

TEST(InertialVertical, GapInFlight_Fails_GapOnThePad_Recovers)
{
    Flight fl;
    fl.end_s = 5.0f;
    auto s = fly(fl);
    {
        InertialVertical iv;
        // drop 150 ms on the pad
        std::vector<Sample> cut;
        for (const auto& x : s)
            if (!(x.t_us > 1000000 && x.t_us < 1150000))
                cut.push_back(x);
        feed(iv, cut, fl, 0.05f);
        EXPECT_TRUE(iv.healthy());
    }
    {
        InertialVertical iv;
        std::vector<Sample> cut;
        const uint32_t t0 = (uint32_t)((fl.pad_s + 2.0f) * 1e6);
        for (const auto& x : s)
            if (!(x.t_us > t0 && x.t_us < t0 + 150000))
                cut.push_back(x);
        feed(iv, cut, fl, 0.05f);
        EXPECT_EQ(iv.state(), InertialVertical::State::Failed);
        EXPECT_STREQ(iv.failReason(), "an IMU gap in flight");
    }
}

TEST(InertialVertical, RailedGyro_HoldsTheAttitude_ThenFails)
{
    Flight fl;
    fl.end_s = 5.0f;
    const auto s = fly(fl);
    const uint32_t t_ign = (uint32_t)(fl.pad_s * 1e6);
    // 200 ms railed from ignition: the attitude is held, still healthy
    {
        InertialVertical iv;
        bool launched = false;
        for (const auto& x : s)
        {
            const bool r = x.t_us >= t_ign && x.t_us < t_ign + 200000;
            Sample y = x;
            if (r)
                y.w[1] = 4000.0f;   // garbage the integrator must not take
            iv.update(y.t_us, y.f, y.w, r);
            if (!launched && x.t_us >= t_ign + 50000)
            {
                iv.launch(x.t_us);
                launched = true;
            }
        }
        EXPECT_TRUE(iv.healthy());
    }
    // 600 ms railed: failed
    {
        InertialVertical iv;
        bool launched = false;
        for (const auto& x : s)
        {
            const bool r = x.t_us >= t_ign && x.t_us < t_ign + 600000;
            iv.update(x.t_us, x.f, x.w, r);
            if (!launched && x.t_us >= t_ign + 50000)
            {
                iv.launch(x.t_us);
                launched = true;
            }
        }
        EXPECT_EQ(iv.state(), InertialVertical::State::Failed);
    }
}

TEST(InertialVertical, NonFiniteSampleInFlight_Fails)
{
    Flight fl;
    fl.end_s = 2.0f;
    const auto s = fly(fl);
    InertialVertical iv;
    const uint32_t t_bad = (uint32_t)((fl.pad_s + 1.0f) * 1e6);
    feed(iv, s, fl, 0.05f, [&](Sample& x) {
        if (x.t_us == t_bad)
            x.f[0] = NAN;
    });
    EXPECT_EQ(iv.state(), InertialVertical::State::Failed);
}

TEST(InertialVertical, ResetForgetsTheFlight)
{
    Flight fl;
    fl.end_s = 2.0f;
    const auto s = fly(fl);
    InertialVertical iv;
    feed(iv, s, fl, 0.05f);
    ASSERT_TRUE(iv.healthy());
    iv.reset();
    EXPECT_EQ(iv.state(), InertialVertical::State::NoRest);
    EXPECT_FALSE(iv.healthy());
}
