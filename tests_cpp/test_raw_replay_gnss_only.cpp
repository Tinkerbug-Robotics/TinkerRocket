// The replay harness's GNSS-only arm (tools/raw_replay/gnss_only_arm.h):
// from its time on the ground no pad ZUPT and no levelling, and act's launch
// edge on the attitude -- the flight gate, the heading prior, the tilt floor
// -- at the switch and again after every restart, as in flight. The first E4
// run (10/17) kept the ground's cos^4 gate, about 1e-7 on the spin rig's
// nose-up payload: its GNSS-only tilt was the gyro's alone.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

// the arm reaches into RawGnssShadow, as raw_replay.cpp does
#define private public
#define protected public
#include "RawGnssShadow.h"
#undef private
#undef protected

#include "gnss_only_arm.h"

using gnss_raw::RawGnssShadow;
using gnss_raw::ShadowImu;
using gnss_raw::ShadowSeed;
using rr::GnssOnlyArm;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kD2R = kPi / 180.0;
constexpr int64_t kT0 = 5000000000LL;      // 5000 s of MCU time
constexpr double kG = 9.807;
constexpr double kNoseUpDeg = 89.0;        // the 10/16 V9 logs' cold-start pitch: 88.7-89.1 deg

// body FRD -> NED, yaw then pitch, scalar first
void quatYawPitch(double yaw, double pitch, float q[4])
{
    const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2), cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    q[0] = float(cy * cp);
    q[1] = float(-sy * sp);
    q[2] = float(cy * sp);
    q[3] = float(sy * cp);
}

void nedToBody(const float q[4], const double v[3], float out[3])
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double M[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)},
                            {2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)},
                            {2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)}};
    for (int i = 0; i < 3; ++i)
        out[i] = float(M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2]);
}

RawGnssShadow::Config cfgOf(uint8_t mode, uint8_t gate)
{
    RawGnssShadow::Config c;
    c.primary = true;
    c.min_step_s = 0.01;
    c.heading_mode = mode;
    c.heading_gate = gate;
    return c;
}

// A nose-up payload still on the pad, the flight loop's tick flags (levelling)
// through the arm, at 200 Hz; the magnetometer too if b_ned is given (a
// heading fused on the pad). Returns the next tick's time.
struct Pad
{
    RawGnssShadow sh;
    float q[4];
    float acc[3];
    float mag[3];
    double b_ned[3] = {21.0, -4.3, 45.5};
    ShadowSeed seed{};
    uint8_t last_flags = 0;

    explicit Pad(const RawGnssShadow::Config& c, bool with_mag = false) : sh(c)
    {
        quatYawPitch(0.0, kNoseUpDeg * kD2R, q);
        const double up[3] = {0.0, 0.0, -kG};
        nedToBody(q, up, acc);
        nedToBody(q, b_ned, mag);
        seed.time_us = static_cast<uint32_t>(kT0);
        seed.lla[0] = 32.99 * kD2R;
        seed.lla[1] = -106.97 * kD2R;
        seed.lla[2] = 1400.0;
        for (int i = 0; i < 4; ++i)
            seed.quat[i] = q[i];
        seed.on_pad = 1;
        if (with_mag)
        {
            seed.declination_rad = float(std::atan2(b_ned[1], b_ned[0]));
            seed.field_ut = 50.3f;
            seed.have_declination = 1;
        }
        sh.onSeed(seed);
    }

    int64_t feed(GnssOnlyArm& arm, int64_t t0, double secs, bool with_mag = false)
    {
        const int64_t t1 = t0 + static_cast<int64_t>(secs * 1e6);
        for (int64_t t = t0; t < t1; t += 5000)
        {
            ShadowImu s{};
            s.time_us = static_cast<uint32_t>(t);
            for (int i = 0; i < 3; ++i)
                s.acc[i] = acc[i];
            s.flags = arm.tick(sh, (t - kT0) * 1e-6, t * 1e-6, gnss_raw::kShadowLevel);
            last_flags = s.flags;
            if (with_mag && (t - kT0) % 20000 == 0)
            {
                gnss_raw::ShadowMag m{};
                m.time_us = static_cast<uint32_t>(t);
                for (int i = 0; i < 3; ++i)
                    m.mag[i] = mag[i];
                m.flags = gnss_raw::kShadowMagCalibrated;
                sh.onMag(m);
            }
            if ((t - kT0) % 1000000 == 0)
            {
                seed.time_us = static_cast<uint32_t>(t);
                sh.onSeed(seed);
            }
            sh.onImu(s, t);
        }
        return t1;
    }
};

// 'h' the heading's gate (att_gate_vertical), 'f' the whole attitude's,
// 'g' the EKF config's (the ground's: cos^4 pitch)
char gateOf(const RawGnssShadow& sh)
{
    const gnss_raw::RawGnssEKFParams& p = sh.filter().params();
    if (p.att_gate_vertical)
        return 'h';
    return p.att_gate_cos4 ? 'g' : 'f';
}

double headingSigmaDeg(const RawGnssShadow& sh)
{
    float sig[3];
    sh.filter().attSigmaNed(sig);
    return sig[2] / kD2R;
}

}  // namespace

TEST(RawReplayGnssOnly, ActsLaunchEdgeOpensTheGateANoseUpPadHoldsShut)
{
    // act with the full gate from launch, the firmware's defaults: before the
    // arm the ground's cos^4 passes about 1e-7 of a GNSS correction to the
    // attitude of a payload 89 deg nose up; from the arm the whole of it, the
    // heading's 1-sigma is the launch prior (no magnetometer: the fallback,
    // 30 deg) and the tilt's at least the floor
    Pad p(cfgOf(RawGnssShadow::kHeadingAct, RawGnssShadow::kGateFullLaunch));
    GnssOnlyArm arm(2.0, GnssOnlyArm::kGateLaunch);
    int64_t t = p.feed(arm, kT0, 1.9);
    ASSERT_TRUE(p.sh.running());
    EXPECT_FALSE(arm.on());
    EXPECT_EQ(gateOf(p.sh), 'g');
    EXPECT_LT(p.sh.ekf_.attGate(), 1.0e-6f);
    EXPECT_TRUE(p.sh.cfg_.pad_zupt);
    EXPECT_TRUE(p.last_flags & gnss_raw::kShadowLevel);
    t = p.feed(arm, t, 1.0);
    ASSERT_TRUE(p.sh.running());
    EXPECT_TRUE(arm.on());
    EXPECT_EQ(arm.launches(), 1);
    EXPECT_EQ(gateOf(p.sh), 'f');
    EXPECT_FLOAT_EQ(p.sh.ekf_.attGate(), 1.0f);
    EXPECT_FALSE(p.sh.cfg_.pad_zupt);
    EXPECT_FALSE(p.last_flags & gnss_raw::kShadowLevel);
    EXPECT_DOUBLE_EQ(arm.priorDeg(), 30.0);
    EXPECT_NEAR(headingSigmaDeg(p.sh), 30.0, 1.0);
    float sig[3];
    p.sh.filter().attSigmaNed(sig);
    EXPECT_GE(sig[0] / kD2R, 0.6 - 1e-3);
    EXPECT_GE(sig[1] / kD2R, 0.6 - 1e-3);
}

TEST(RawReplayGnssOnly, TheDescentArmKeepsTheLevelling)
{
    // keep_level (--gnss-only-keep-level): the same launch edge and no ZUPT,
    // but the tick keeps its levelling flag -- the descent under a chute as
    // flown -- and the shadow levels on it
    Pad p(cfgOf(RawGnssShadow::kHeadingAct, RawGnssShadow::kGateFullLaunch));
    GnssOnlyArm arm(1.0, GnssOnlyArm::kGateLaunch, true);
    EXPECT_TRUE(arm.keepLevel());
    int64_t t = p.feed(arm, kT0, 0.9);
    const uint32_t levels0 = p.sh.stats().levels;
    p.feed(arm, t, 1.0);
    EXPECT_TRUE(arm.on());
    EXPECT_EQ(arm.launches(), 1);
    EXPECT_EQ(gateOf(p.sh), 'f');
    EXPECT_FALSE(p.sh.cfg_.pad_zupt);
    EXPECT_TRUE(p.last_flags & gnss_raw::kShadowLevel);
    EXPECT_GE(p.sh.stats().levels, levels0 + 95u);       // every step of the second (100 a second)
    EXPECT_NEAR(headingSigmaDeg(p.sh), 30.0, 1.0);
}

TEST(RawReplayGnssOnly, TheGroundGateKeptOnlyWhenAsked)
{
    // kGateGround: the first E4 run's arm -- no ZUPT, no levelling, the
    // ground's gate and the pad's heading sigma as they were
    Pad p(cfgOf(RawGnssShadow::kHeadingAct, RawGnssShadow::kGateFullLaunch));
    GnssOnlyArm arm(1.0, GnssOnlyArm::kGateGround);
    int64_t t = p.feed(arm, kT0, 0.9);
    const double sig_before = headingSigmaDeg(p.sh);
    p.feed(arm, t, 1.0);
    EXPECT_TRUE(arm.on());
    EXPECT_EQ(arm.launches(), 0);
    EXPECT_EQ(gateOf(p.sh), 'g');
    EXPECT_FALSE(p.sh.cfg_.pad_zupt);
    EXPECT_FALSE(p.last_flags & gnss_raw::kShadowLevel);
    EXPECT_GT(headingSigmaDeg(p.sh), sig_before - 1.0);   // not the 30 deg prior
    EXPECT_GT(headingSigmaDeg(p.sh), 45.0);
}

TEST(RawReplayGnssOnly, EachGateAsItsLaunchEdgeSetsIt)
{
    // kGateHeading and kGateFullApogee gate GNSS to the heading from launch
    // (the apogee call opens the latter in flight only); observe leaves the
    // config's gate in flight, and so does the arm
    struct Case
    {
        uint8_t mode, gate;
        char want;
    } cases[] = {{RawGnssShadow::kHeadingAct, RawGnssShadow::kGateFullLaunch, 'f'},
                 {RawGnssShadow::kHeadingAct, RawGnssShadow::kGateHeading, 'h'},
                 {RawGnssShadow::kHeadingAct, RawGnssShadow::kGateFullApogee, 'h'},
                 {RawGnssShadow::kHeadingObserve, RawGnssShadow::kGateFullLaunch, 'g'}};
    for (const Case& c : cases)
    {
        Pad p(cfgOf(c.mode, c.gate));
        GnssOnlyArm arm(1.0, GnssOnlyArm::kGateLaunch);
        p.feed(arm, kT0, 1.5);
        ASSERT_TRUE(p.sh.running());
        EXPECT_EQ(gateOf(p.sh), c.want) << int(c.mode) << " " << int(c.gate);
        EXPECT_EQ(arm.priorDeg() < 0.0, c.mode != RawGnssShadow::kHeadingAct) << int(c.mode);
    }
}

TEST(RawReplayGnssOnly, AMagnetometerHeadingOnThePadGivesThePadPrior)
{
    // a heading fused within heading_mag_recent_s of the edge: the prior is
    // the pad's (10 deg), as act's launch edge takes it
    Pad p(cfgOf(RawGnssShadow::kHeadingAct, RawGnssShadow::kGateFullLaunch), true);
    GnssOnlyArm arm(4.0, GnssOnlyArm::kGateLaunch);
    p.feed(arm, kT0, 4.5, true);
    ASSERT_TRUE(p.sh.running());
    ASSERT_GT(p.sh.stats().mag_fused, 0u);
    EXPECT_EQ(arm.launches(), 1);
    EXPECT_DOUBLE_EQ(arm.priorDeg(), 10.0);
}

TEST(RawReplayGnssOnly, ARestartOnThePadTakesTheLaunchEdgeAgain)
{
    // a stop on the pad (far from the seed, say) and the restart put the
    // ground's gate back: the arm opens it again at the next tick
    Pad p(cfgOf(RawGnssShadow::kHeadingAct, RawGnssShadow::kGateFullLaunch));
    GnssOnlyArm arm(1.0, GnssOnlyArm::kGateLaunch);
    int64_t t = p.feed(arm, kT0, 1.5);
    ASSERT_EQ(arm.launches(), 1);
    ASSERT_EQ(gateOf(p.sh), 'f');
    const uint32_t starts = p.sh.stats().starts;
    p.sh.stop(RawGnssShadow::kStopFar);
    p.sh.start(t * 1e-6);
    ASSERT_TRUE(p.sh.running());
    ASSERT_EQ(p.sh.stats().starts, starts + 1);
    EXPECT_EQ(gateOf(p.sh), 'g');
    p.feed(arm, t, 0.1);
    EXPECT_EQ(arm.launches(), 2);
    EXPECT_EQ(gateOf(p.sh), 'f');
}

TEST(RawReplayGnssOnly, InFlightTheArmLeavesTheTickAlone)
{
    // the launch edge in flight is the shadow's own: the arm neither switches
    // on nor touches the flags
    RawGnssShadow sh(cfgOf(RawGnssShadow::kHeadingAct, RawGnssShadow::kGateFullLaunch));
    GnssOnlyArm arm(0.0, GnssOnlyArm::kGateLaunch);
    const uint8_t f = gnss_raw::kShadowInFlight | gnss_raw::kShadowLevel;
    EXPECT_EQ(arm.tick(sh, 5.0, kT0 * 1e-6, f), f);
    EXPECT_FALSE(arm.on());
    // and an arm never asked does nothing at all
    GnssOnlyArm none(1.0e9, GnssOnlyArm::kGateLaunch);
    EXPECT_FALSE(none.asked());
    EXPECT_EQ(none.tick(sh, 5.0, kT0 * 1e-6, gnss_raw::kShadowLevel), gnss_raw::kShadowLevel);
    EXPECT_TRUE(sh.cfg_.pad_zupt);
}
