// #1755 (and #1734): an AUTO-mounted board must confirm its frame on a swaying
// rail, and a frame that is wrong anyway must not decide burnout.
//
// On AUTO the board-to-rocket frame is whatever the pad orientation estimator's
// last still window snapped it to.  The estimator used to need every gyro axis
// under 3 dps on every sample for 2 s, so ordinary rail sway (or a little gyro
// bias) kept a racked rocket from ever completing a window: a board powered on a
// prep table flew the table's frame, a lateral +X, and a board mounted
// tail-forward flew the boot identity, a reversed +X.  Burnout read rocket +X,
// so it latched under thrust (reversed) or at a random time or never (lateral),
// and with it the apogee vote and the transonic lockout window.
//
// The first block feeds the real OrientationEstimator 1 kHz pad motion.  The
// second covers the thrust-axis check's window start (#1734).  The third tests
// the flight computer's ThrustAxisPolicy and flies the real ThrustAxisCheck,
// ThrustAxisPolicy, tr::burnoutAxialInput and tr::burnoutDetectStep, wired as
// main.cpp wires them, through a boost and coast in a right, a reversed and a
// lateral frame.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "BurnoutDetector.h"
#include "TR_Orientation.h"
#include "thrust_axis_policy.h"

namespace {

constexpr float kG   = 9.80665f;
constexpr float kPi  = 3.14159265358979f;
constexpr float kD2R = kPi / 180.0f;

// Rodrigues: v rotated by angle a (rad) about unit axis k.
void rotate(const float v[3], const float k[3], float a, float out[3])
{
    const float c = cosf(a), s = sinf(a);
    const float kv = k[0] * v[0] + k[1] * v[1] + k[2] * v[2];
    const float kx[3] = {k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0]};
    for (int i = 0; i < 3; i++) out[i] = v[i] * c + kx[i] * s + k[i] * kv * (1.0f - c);
}

float angleDeg(const float a[3], const float b[3])
{
    float d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    if (d > 1.0f) d = 1.0f;
    if (d < -1.0f) d = -1.0f;
    return acosf(d) / kD2R;
}

struct Fed
{
    int   windows = 0;
    bool  have = false;
    float est[3] = {0.0f, 0.0f, 0.0f};
};

// One 1 kHz pad sample: the specific force is g along `up` turned by the sway
// angle about `k`; the gyro reads the sway rate about `k` plus a bias.
void feedOne(OrientationEstimator& est, uint32_t t_us, const float up[3], const float k[3],
             float angle_rad, float rate_dps, const float bias_dps[3], Fed& out)
{
    float f[3];
    rotate(up, k, angle_rad, f);
    if (est.update(t_us, kG * f[0], kG * f[1], kG * f[2],
                   k[0] * rate_dps + bias_dps[0], k[1] * rate_dps + bias_dps[1],
                   k[2] * rate_dps + bias_dps[2]))
    {
        out.windows++;
        out.have = est.takeEstimate(out.est);
    }
}

// `secs` of sinusoidal sway at `freq_hz` whose rate peaks at `peak_dps`.
Fed sway(OrientationEstimator& est, uint32_t& t_us, float secs, const float up[3], const float k[3],
         float peak_dps, float freq_hz, const float bias_dps[3])
{
    Fed out;
    const float w = 2.0f * kPi * freq_hz;
    const float amp_rad = (peak_dps * kD2R) / w;
    const int n = (int)(secs * 1000.0f);
    for (int i = 0; i < n; i++)
    {
        t_us += 1000;
        const float t = (float)i / 1000.0f;
        feedOne(est, t_us, up, k, amp_rad * sinf(w * t), peak_dps * cosf(w * t), bias_dps, out);
    }
    return out;
}

const float kNoBias[3] = {0.0f, 0.0f, 0.0f};
const float kNose[3]   = {1.0f, 0.0f, 0.0f};   // a right frame on a vertical rail
const float kAboutY[3] = {0.0f, 1.0f, 0.0f};   // sway axes perpendicular to the nose
const float kAboutZ[3] = {0.0f, 0.0f, 1.0f};

} // namespace

// ---- The estimator on a swaying rail ----------------------------------------

TEST(OrientationSway, SwayOverTheOldThreeDpsLimitConfirmsTheFrame)
{
    // The issue's table: 3.05-4.0 dps peaks at 2 Hz completed no window in 600 s.
    // A 250 ms block is half a 2 Hz period, so its mean rate reaches (2/pi) of
    // the peak: every block passes the 5 dps mean up to a 7.8 dps peak.
    for (float peak : {3.05f, 4.0f, 6.0f, 7.5f})
    {
        OrientationEstimator est;
        uint32_t t = 1000;
        const Fed f = sway(est, t, 20.0f, kNose, kAboutY, peak, 2.0f, kNoBias);
        EXPECT_GE(f.windows, 9) << peak << " dps";
        ASSERT_TRUE(f.have) << peak << " dps";
        EXPECT_LT(angleDeg(f.est, kNose), 1.0f) << peak << " dps";
    }
    // Past that, blocks whose phase centres on a rate peak fail, but windows
    // still complete between them.
    OrientationEstimator est;
    uint32_t t = 1000;
    const Fed f = sway(est, t, 20.0f, kNose, kAboutY, 12.0f, 2.0f, kNoBias);
    EXPECT_GE(f.windows, 3);
    ASSERT_TRUE(f.have);
    EXPECT_LT(angleDeg(f.est, kNose), 1.0f);
}

TEST(OrientationSway, SlowerSwayConfirmsTheFrameToo)
{
    for (float hz : {0.5f, 1.0f, 4.0f})
    {
        OrientationEstimator est;
        uint32_t t = 1000;
        const Fed f = sway(est, t, 30.0f, kNose, kAboutZ, 4.0f, hz, kNoBias);
        EXPECT_GE(f.windows, 8) << hz << " Hz";
        ASSERT_TRUE(f.have);
        EXPECT_LT(angleDeg(f.est, kNose), 1.0f) << hz << " Hz";
    }
}

TEST(OrientationSway, GyroBiasUnderSwayConfirmsTheFrame)
{
    // The issue's table: 2.0 dps sway plus 1.5 dps bias completed no window.
    const float bias[3] = {1.5f, -1.5f, 1.5f};
    OrientationEstimator est;
    uint32_t t = 1000;
    const Fed f = sway(est, t, 20.0f, kNose, kAboutY, 2.0f, 2.0f, bias);
    EXPECT_GE(f.windows, 8);
    ASSERT_TRUE(f.have);
    EXPECT_LT(angleDeg(f.est, kNose), 1.0f);
}

TEST(OrientationSway, RandomTwoAxisSwayConfirmsTheFrame)
{
    // The issue's table: a lightly damped 2 Hz rail (damping 0.05) driven by
    // noise on two axes at 2.5 dps RMS completed no window (longest quiet run
    // 1.87 s).  Deterministic noise; the response is scaled to the RMS wanted.
    const int n = 120000;   // 120 s at 1 kHz
    const float w = 2.0f * kPi * 2.0f, zeta = 0.05f, dt = 0.001f;
    std::vector<float> ay(n), az(n), ry(n), rz(n);
    uint32_t lcg = 12345u;
    auto noise = [&lcg]() {
        lcg = lcg * 1664525u + 1013904223u;
        return ((float)(lcg >> 8) / 16777216.0f) - 0.5f;
    };
    float th[2] = {0.0f, 0.0f}, om[2] = {0.0f, 0.0f};
    double ss = 0.0;
    for (int i = 0; i < n; i++)
    {
        for (int a = 0; a < 2; a++)
        {
            const float acc = -w * w * th[a] - 2.0f * zeta * w * om[a] + 400.0f * noise();
            om[a] += acc * dt;
            th[a] += om[a] * dt;
        }
        ay[i] = th[0]; az[i] = th[1]; ry[i] = om[0]; rz[i] = om[1];
        ss += (double)om[0] * om[0] + (double)om[1] * om[1];
    }
    const float rms_raw = (float)sqrt(ss / (2.0 * n));   // rad/s per axis
    for (float rms_dps : {2.5f, 5.0f})
    {
        const float scale = (rms_dps * kD2R) / rms_raw;
        OrientationEstimator est;
        Fed out;
        uint32_t t = 1000;
        for (int i = 0; i < n; i++)
        {
            t += 1000;
            // Two small rotations about +Y then +Z (angles of a few tenths of a degree)
            float f1[3], f2[3];
            rotate(kNose, kAboutY, ay[i] * scale, f1);
            rotate(f1, kAboutZ, az[i] * scale, f2);
            if (est.update(t, kG * f2[0], kG * f2[1], kG * f2[2],
                           0.0f, ry[i] * scale / kD2R, rz[i] * scale / kD2R))
            {
                out.windows++;
                out.have = est.takeEstimate(out.est);
            }
        }
        EXPECT_GE(out.windows, 20) << rms_dps << " dps RMS";
        ASSERT_TRUE(out.have);
        EXPECT_LT(angleDeg(out.est, kNose), 1.0f) << rms_dps << " dps RMS";
    }
}

TEST(OrientationSway, PrepTableThenASwayingRailFliesTheRailsFrame)
{
    // The issue's first failure: still on a prep table (the board's +Y up in
    // the identity frame), then racked on a rail swaying at 4 dps.  The last
    // window must be the rail's.
    OrientationEstimator est;
    uint32_t t = 1000;
    const float table_up[3] = {0.0f, 1.0f, 0.0f};
    const Fed table = sway(est, t, 10.0f, table_up, kAboutZ, 0.0f, 2.0f, kNoBias);
    ASSERT_TRUE(table.have);
    EXPECT_LT(angleDeg(table.est, table_up), 1.0f);
    // Carried to the pad: a big turn the estimator must not average in.
    Fed carry;
    for (int i = 0; i < 3000; i++)
    {
        t += 1000;
        const float a = (90.0f * kD2R) * (float)i / 3000.0f;
        feedOne(est, t, table_up, kAboutZ, -a, -30.0f, kNoBias, carry);
    }
    EXPECT_EQ(carry.windows, 0);
    const Fed rail = sway(est, t, 20.0f, kNose, kAboutY, 4.0f, 2.0f, kNoBias);
    EXPECT_GE(rail.windows, 8);
    ASSERT_TRUE(rail.have);
    EXPECT_LT(angleDeg(rail.est, kNose), 1.0f);
}

// ---- What must still not confirm a frame ------------------------------------

TEST(OrientationSway, ASlowTurnNeverConfirms)
{
    // A rail being raised, or a rocket turned by hand, at 4 dps: the rate's
    // mean passes, the direction does not hold for 2 s.
    OrientationEstimator est;
    uint32_t t = 1000;
    Fed f;
    for (int i = 0; i < 20000; i++)
    {
        t += 1000;
        const float a = (4.0f * kD2R) * (float)i / 1000.0f;
        feedOne(est, t, kNose, kAboutY, a, 4.0f, kNoBias, f);
    }
    EXPECT_EQ(f.windows, 0);
}

TEST(OrientationSway, HandlingNeverConfirms)
{
    // A rocket carried or shaken: 40 dps rate peaks at 2 Hz.
    OrientationEstimator est;
    uint32_t t = 1000;
    const Fed f = sway(est, t, 20.0f, kNose, kAboutY, 40.0f, 2.0f, kNoBias);
    EXPECT_EQ(f.windows, 0);
    // And knocks on a still rocket: 0.5 g jolts four times a second.
    OrientationEstimator est2;
    t = 1000;
    int windows = 0;
    for (int i = 0; i < 20000; i++)
    {
        t += 1000;
        const float jolt = (i % 250 == 0) ? 0.5f * kG : 0.0f;
        windows += est2.update(t, kG + jolt, 0.0f, 0.0f, 0.1f, 0.1f, 0.1f) ? 1 : 0;
    }
    EXPECT_EQ(windows, 0);
}

TEST(OrientationSway, ASteadyTurnAboutTheNoseFailsTheMeanRate)
{
    // Gravity alone cannot see a turn about itself; the rate's mean can.
    const float bias[3] = {6.0f, 0.0f, 0.0f};
    OrientationEstimator est;
    uint32_t t = 1000;
    const Fed f = sway(est, t, 20.0f, kNose, kAboutY, 0.0f, 2.0f, bias);
    EXPECT_EQ(f.windows, 0);
}

TEST(OrientationSway, AFeedGapOnABlockBoundaryRestartsTheWindowToo)
{
    // 7 still blocks, then a long gap that starts right after a block closed,
    // then the rocket 4 deg away: the window must start again, not complete on
    // the next block with the old ones.
    OrientationEstimator est;
    uint32_t t = 1000;
    Fed f = sway(est, t, 1.757f, kNose, kAboutY, 0.0f, 2.0f, kNoBias);   // 7 blocks of 251 samples
    EXPECT_EQ(f.windows, 0);
    t += 60000000;   // 60 s with no sample
    const float moved[3] = {cosf(4.0f * kD2R), sinf(4.0f * kD2R), 0.0f};
    f = sway(est, t, 1.9f, moved, kAboutZ, 0.0f, 2.0f, kNoBias);
    EXPECT_EQ(f.windows, 0);
    f = sway(est, t, 0.5f, moved, kAboutZ, 0.0f, 2.0f, kNoBias);
    EXPECT_EQ(f.windows, 1);
    ASSERT_TRUE(f.have);
    EXPECT_LT(angleDeg(f.est, moved), 0.1f);
}

TEST(OrientationSway, AFeedGapRestartsTheWindow)
{
    OrientationEstimator est;
    uint32_t t = 1000;
    Fed f = sway(est, t, 1.9f, kNose, kAboutY, 0.0f, 2.0f, kNoBias);
    EXPECT_EQ(f.windows, 0);
    t += 150000;   // 150 ms with no sample
    f = sway(est, t, 1.9f, kNose, kAboutY, 0.0f, 2.0f, kNoBias);
    EXPECT_EQ(f.windows, 0);   // 3.8 s fed, but no 2 s of it unbroken
    f = sway(est, t, 0.5f, kNose, kAboutY, 0.0f, 2.0f, kNoBias);
    EXPECT_EQ(f.windows, 1);
}

// ---- The thrust-axis check's window (#1734) ---------------------------------

TEST(ThrustAxisWindow, ASampleStampedBeforeStartStillMeasures)
{
    // enterInflight() calls start() late in a pass; the next sample is an IMU
    // window's centre, stamped before it.  The check used to close on that
    // sample, invalid, on every launch.
    ThrustAxisCheck chk;
    chk.start(10000000);
    EXPECT_TRUE(chk.running());
    uint32_t t = 9999500;
    bool done = false;
    for (int i = 0; i < 400 && !done; i++, t += 1000) done = chk.update(t, 8.0f * kG, 1.0f, -1.0f);
    ASSERT_TRUE(done);
    EXPECT_FALSE(chk.running());
    ASSERT_TRUE(chk.valid());
    EXPECT_LT(chk.angleFromNoseDeg(), 5.0f);
    float u[3];
    ASSERT_TRUE(chk.axis(u));
    EXPECT_NEAR(u[0], 1.0f, 0.01f);
}

TEST(ThrustAxisWindow, AReversedFrameMeasuresMinusX)
{
    ThrustAxisCheck chk;
    chk.start(5000);
    uint32_t t = 5000;
    bool done = false;
    for (int i = 0; i < 400 && !done; i++, t += 1000) done = chk.update(t, -8.0f * kG, 0.5f, 0.0f);
    ASSERT_TRUE(done && chk.valid());
    EXPECT_GT(chk.angleFromNoseDeg(), 170.0f);
    float u[3];
    ASSERT_TRUE(chk.axis(u));
    EXPECT_NEAR(u[0], -1.0f, 0.01f);
}

TEST(ThrustAxisWindow, NotStartedIsNeitherRunningNorDone)
{
    // A restored flight never calls start(): burnout then reads rocket +X.
    ThrustAxisCheck chk;
    for (int i = 0; i < 500; i++) chk.update(1000 + i * 1000, 8.0f * kG, 0.0f, 0.0f);
    EXPECT_FALSE(chk.running());
    EXPECT_FALSE(chk.done());
    float u[3];
    EXPECT_FALSE(chk.axis(u));
}

// ---- The policy --------------------------------------------------------------

TEST(ThrustAxisPolicy, OnlyUpwardThrustOnAutoDisprovesTheFrame)
{
    using ThrustAxisPolicy::judge;
    // Upward thrust 90 deg off +X on AUTO: the frame is wrong.
    auto v = judge(true, 90.0f, true, 0.98f, /*manual=*/false);
    EXPECT_TRUE(v.thrust && v.mismatch && v.disproved);
    // The same on a manual code: flagged, flown as set.
    v = judge(true, 90.0f, true, 0.98f, true);
    EXPECT_TRUE(v.thrust && v.mismatch);
    EXPECT_FALSE(v.disproved);
    // Within 25 deg: no mismatch.
    v = judge(true, 20.0f, true, 0.98f, false);
    EXPECT_TRUE(v.thrust);
    EXPECT_FALSE(v.mismatch || v.disproved);
    // Pointing down (coast drag), sideways, or no levelled attitude: not thrust.
    for (float up : {-1.0f, -0.2f, 0.3f, 0.49f})
    {
        v = judge(true, 179.0f, true, up, false);
        EXPECT_FALSE(v.thrust || v.mismatch || v.disproved) << up;
    }
    v = judge(true, 179.0f, /*inertial_healthy=*/false, 0.98f, false);
    EXPECT_FALSE(v.thrust || v.disproved);
    v = judge(true, 179.0f, true, NAN, false);
    EXPECT_FALSE(v.thrust || v.disproved);
    // No verdict from the check.
    v = judge(false, 179.0f, true, 0.98f, false);
    EXPECT_FALSE(v.thrust || v.disproved);
}

TEST(ThrustAxisPolicy, BurnoutIsHeldOnlyOnAutoWhileMeasuringAndNeverPastTheWait)
{
    using ThrustAxisPolicy::holdBurnout;
    using ThrustAxisPolicy::kWaitMs;
    EXPECT_TRUE(holdBurnout(false, true, 150));
    EXPECT_TRUE(holdBurnout(false, true, kWaitMs - 1));
    EXPECT_FALSE(holdBurnout(false, true, kWaitMs));    // an IMU stalled at launch
    EXPECT_FALSE(holdBurnout(false, false, 150));       // closed, or never started
    EXPECT_FALSE(holdBurnout(true, true, 150));         // a manual code: as before
}

// ---- Burnout in a wrong frame ------------------------------------------------

namespace {

enum class Frame { Right, Reversed, Lateral };

// World up in the frame the board flies, for the IMU's levelled attitude.
void upInFrame(Frame fr, float up[3])
{
    up[0] = fr == Frame::Right ? 1.0f : fr == Frame::Reversed ? -1.0f : 0.0f;
    up[1] = fr == Frame::Lateral ? 1.0f : 0.0f;
    up[2] = 0.0f;
}

// The rocket's axial specific force: 11 g of thrust with 2 m/s² vibration for
// 3.0 s, then drag of 1.2 g falling off over the coast; small lateral terms.
// Returned in the frame the board flies: right (+X), reversed (tail-forward,
// 180 deg about Z) or lateral (snapped on a prep table: the axis is rocket +Y).
// t_ms is time since ignition.
void specificForce(Frame fr, uint32_t t_ms, float drag_g, float f[3])
{
    const float vib = 2.0f * sinf(0.37f * (float)t_ms) * cosf(0.11f * (float)t_ms);
    const float axial = t_ms < 3000 ? 11.0f * kG + vib
                                    : -drag_g * kG * expf(-(float)(t_ms - 3000) / 4000.0f) + 0.3f * vib;
    const float lat1 = 0.4f * sinf(0.05f * (float)t_ms), lat2 = 0.3f * cosf(0.07f * (float)t_ms);
    switch (fr)
    {
        case Frame::Right:    f[0] = axial;  f[1] = lat1;  f[2] = lat2; break;
        case Frame::Reversed: f[0] = -axial; f[1] = -lat1; f[2] = lat2; break;
        case Frame::Lateral:  f[0] = lat1;   f[1] = axial; f[2] = lat2; break;
    }
}

struct FlyOpts
{
    bool     with_check = true;     // false: a restored flight (no start())
    bool     manual = false;
    bool     levelled = true;       // InertialVertical flying at launch
    uint32_t launch_ms = 0;         // ignition-to-INFLIGHT (a late launch detect)
    float    drag_g = 1.2f;
};

// Fly 10 s at 1 kHz from INFLIGHT entry, wired as main.cpp wires it: the check
// from launch, judged by ThrustAxisPolicy at its close, then burnout on the
// policy's input.  Returns the latch time in ms after INFLIGHT, 0 for none.
uint32_t flyBurnout(Frame fr, const FlyOpts& o = FlyOpts())
{
    ThrustAxisCheck chk;
    if (o.with_check) chk.start(0);
    bool disproved = false;
    float axis[3] = {1.0f, 0.0f, 0.0f};
    float up[3];
    upInFrame(fr, up);
    bool detected = false;
    uint16_t neg = 0;
    for (uint32_t t = 1; t <= 10000; t++)
    {
        float f[3];
        specificForce(fr, o.launch_ms + t, o.drag_g, f);
        if (chk.update(t * 1000u, f[0], f[1], f[2]))
        {
            float u[3] = {1.0f, 0.0f, 0.0f};
            const bool have = chk.axis(u);
            const float axis_up = u[0] * up[0] + u[1] * up[1] + u[2] * up[2];
            const auto v = ThrustAxisPolicy::judge(have, chk.angleFromNoseDeg(), o.levelled, axis_up, o.manual);
            if (v.disproved)
            {
                disproved = true;
                for (int i = 0; i < 3; i++) axis[i] = u[i];
            }
        }
        float axial = 0.0f;
        if (tr::burnoutAxialInput(f, ThrustAxisPolicy::holdBurnout(o.manual, chk.running(), t),
                                  disproved ? axis : nullptr, axial) &&
            tr::burnoutDetectStep(detected, neg, axial, t, 50))
            return t;
    }
    return 0;
}

} // namespace

TEST(BurnoutAxis, OnAutoEveryFrameLatchesAtTheRealCutoff)
{
    for (Frame fr : {Frame::Right, Frame::Reversed, Frame::Lateral})
    {
        const uint32_t at = flyBurnout(fr);
        EXPECT_GE(at, 3000u) << (int)fr;
        EXPECT_LE(at, 3100u) << (int)fr;
    }
}

TEST(BurnoutAxis, OnRocketXAWrongFrameLatchedUnderThrustOrNever)
{
    // What the flight computer did before #1755, and still does with no check
    // (a restored flight): right only in the right frame.
    FlyOpts restored;
    restored.with_check = false;
    const uint32_t right = flyBurnout(Frame::Right, restored);
    EXPECT_GE(right, 3000u);
    EXPECT_LE(right, 3100u);
    EXPECT_LT(flyBurnout(Frame::Reversed, restored), 300u);   // under thrust
    const uint32_t lateral = flyBurnout(Frame::Lateral, restored);
    EXPECT_TRUE(lateral == 0u || lateral < 3000u);            // never, or in the boost
}

TEST(BurnoutAxis, AManualCodeIsFlownAsSet)
{
    // The owner's mounting code is trusted: rocket +X, as before #1755.  A right
    // manual frame is unchanged; a wrong one is flagged, not overridden.
    FlyOpts manual;
    manual.manual = true;
    const uint32_t right = flyBurnout(Frame::Right, manual);
    EXPECT_GE(right, 3000u);
    EXPECT_LE(right, 3100u);
    EXPECT_LT(flyBurnout(Frame::Reversed, manual), 300u);
}

TEST(BurnoutAxis, AFlightEnteredAfterBurnoutMeasuresDragAndKeepsRocketX)
{
    // The review's case: INFLIGHT entered at burnout (a late accel-only launch
    // detect, a short hot motor), so the check measures 2.5-4 g of coast drag,
    // which points down.  Read as the thrust axis it would hold burnout off for
    // good; it is not thrust, so burnout stays on rocket +X and latches after
    // the 200 ms lockout, as before.
    for (float drag : {2.5f, 4.0f})
    {
        FlyOpts late;
        late.launch_ms = 3000;
        late.drag_g = drag;
        const uint32_t at = flyBurnout(Frame::Right, late);
        EXPECT_GT(at, 200u) << drag;
        EXPECT_LT(at, 400u) << drag;
    }
}

TEST(BurnoutAxis, NoLevelledAttitudeGivesNoVerdict)
{
    // InertialVertical not flying: up and down cannot be told apart, so the
    // frame is not disproved and burnout reads rocket +X.
    FlyOpts o;
    o.levelled = false;
    const uint32_t right = flyBurnout(Frame::Right, o);
    EXPECT_GE(right, 3000u);
    EXPECT_LE(right, 3100u);
    EXPECT_LT(flyBurnout(Frame::Reversed, o), 300u);
}

TEST(BurnoutAxis, NoInputWhileTheAxisIsBeingMeasured)
{
    const float f[3] = {-5.0f, 0.0f, 0.0f};
    const float u[3] = {1.0f, 0.0f, 0.0f};
    float axial = 123.0f;
    EXPECT_FALSE(tr::burnoutAxialInput(f, /*measuring=*/true, u, axial));
    EXPECT_EQ(axial, 123.0f);
    ASSERT_TRUE(tr::burnoutAxialInput(f, false, nullptr, axial));
    EXPECT_EQ(axial, -5.0f);
    const float g[3] = {1.0f, -2.0f, 3.0f};
    const float v[3] = {0.0f, -1.0f, 0.0f};
    ASSERT_TRUE(tr::burnoutAxialInput(g, false, v, axial));
    EXPECT_FLOAT_EQ(axial, 2.0f);
}
