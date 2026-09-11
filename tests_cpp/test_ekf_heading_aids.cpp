// #1135 — the two GNSS-derived heading aids. Neither had any host coverage.
//
// Both ship OFF (GnssHeadingAids::Off), so nothing here changes a flight today.
// They are tested because they stay in the tree to be MEASURED: #1281's plan is
// to switch them on in replay against a sim with a realistic GNSS noise model
// and decide whether they earn their place. That measurement is only meaningful
// if the models are right.
//
// This file covers BOTH items of #1135, but they resolved differently:
//
//   item 2 (accelMatchHeadingUpdate) was a model error and is FIXED here;
//   item 1 (velCourseHeadingUpdate) is real but every proposed remedy was
//          measured and refused — see the characterisation test at the bottom,
//          which exists so the next person does not rebuild a rejected fix.

#include <gtest/gtest.h>

#include <cmath>

#include "TR_GpsInsEKF.h"

namespace {

constexpr float kG = 9.80665f;
constexpr float kDeg = (float)M_PI / 180.0f;

// Reaches the two aids, which are protected for exactly this purpose.
class AidProbe : public GpsInsEKF {
public:
    using GpsInsEKF::accelMatchHeadingUpdate;
    using GpsInsEKF::velCourseHeadingUpdate;

    // Attitude is the only state these two read, so set it directly and read
    // back what the update did to it.
    void setAttitudeDeg(float roll, float pitch, float yaw) {
        const float cr = std::cos(roll * kDeg * 0.5f), sr = std::sin(roll * kDeg * 0.5f);
        const float cp = std::cos(pitch * kDeg * 0.5f), sp = std::sin(pitch * kDeg * 0.5f);
        const float cy = std::cos(yaw * kDeg * 0.5f), sy = std::sin(yaw * kDeg * 0.5f);
        setQuaternion(cr * cp * cy + sr * sp * sy,
                      sr * cp * cy - cr * sp * sy,
                      cr * sp * cy + sr * cp * sy,
                      cr * cp * sy - sr * sp * cy);
        // setQuaternion() deliberately collapses the attitude covariance — it
        // exists to INJECT a known attitude. With P[6..8] at ~0 the Kalman gain
        // is ~0 and no measurement can move anything, so every test here would
        // pass for the wrong reason. Give the filter back a realistic attitude
        // uncertainty so "did this update do something" is a real question.
        for (int i = 6; i < 9; ++i) inflateCovDiag(i, 0.05f);
    }

    void quat(float (&out)[4]) const { getQuaternion(out); }
};

// Angle between two quaternions, degrees — "how far did this update move the
// attitude", which is the only thing these tests care about.
float attitudeDeltaDeg(const float a[4], const float b[4]) {
    float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    dot = std::fabs(dot);
    if (dot > 1.0f) dot = 1.0f;
    return 2.0f * std::acos(dot) / kDeg;
}

// Body specific force for a vehicle at REST at a given pitch, FRD body frame.
// At rest the accelerometer reads the reaction to gravity and nothing else.
void restingSpecificForce(float pitch_deg, float out[3]) {
    const float p = pitch_deg * kDeg;
    // Nose up by `pitch`: gravity's reaction projects onto the nose axis as
    // sin(pitch) and onto body-down as cos(pitch).
    out[0] = kG * std::sin(p);
    out[1] = 0.0f;
    out[2] = -kG * std::cos(p);
}

}  // namespace

// ── #1135 item 2 — FIXED: rotate the full specific force ────────────────────

TEST(EkfHeadingAids, StationaryOnATiltedRailPredictsNoHorizontalAcceleration) {
    // The demonstration from the issue. A rocket on a 5°-off-vertical rail is
    // not accelerating, so the true world horizontal acceleration is exactly
    // zero, and any nonzero prediction is the model talking to itself.
    //
    // The old prediction rotated only (0, ay, az) into NED and produced
    // g*sin(5°) = 0.85 m/s² — clear of its own 0.5 m/s² floor, so it was fused
    // against a differentiated-GNSS measurement that is pure noise at rest.
    // With the FULL specific force the axial and lateral contributions cancel
    // exactly, as they physically must, and the floor now refuses the sample.
    AidProbe ekf;
    ekf.setAttitudeDeg(0.0f, 85.0f, 0.0f);   // 5° off vertical

    float f[3];
    restingSpecificForce(85.0f, f);

    float before[4], after[4];
    ekf.quat(before);
    // A measurement of pure noise, well clear of the floor — the old code would
    // have fused against it.
    const float noisyWorldHoriz[2] = {2.0f, 2.0f};
    ekf.accelMatchHeadingUpdate(f, noisyWorldHoriz);
    ekf.quat(after);

    EXPECT_LT(attitudeDeltaDeg(before, after), 1e-3f)
        << "a stationary rocket moved its own attitude from a GNSS-noise azimuth";
}

TEST(EkfHeadingAids, ABoostAzimuthIsNoLongerNinetyDegreesFromItsOwnMeasurement) {
    // The systematic half of item 2, and the reason it mattered: during ascent
    // accelMeasUpdate and magMeasUpdate are both gated off, so this aid and the
    // course aid are the ONLY attitude-correcting updates in powered flight.
    //
    // Under boost the two sides modelled different quantities. The measurement
    // (d/dt of GNSS NED velocity) carries the horizontal projection of the
    // axial thrust — 9.4-18.4 m/s² on the four 2026-08-29 flights — while the
    // old prediction dropped fx entirely and kept gravity's lateral projection,
    // which the measurement does not have (35-100% of the old pred_h). So the
    // measurement azimuth tracked the NOSE and the prediction azimuth tracked
    // the lateral force: a near-90° systematic innovation fused at 26° sigma.
    //
    // Rotating the whole vector makes both sides the same physical quantity, so
    // a correct attitude now yields ~zero innovation. Feed the aid exactly the
    // measurement a world-frame observer would compute for this state and it
    // must recognise it as agreement.
    AidProbe ekf;
    ekf.setAttitudeDeg(0.0f, 85.0f, 0.0f);   // 5° off vertical, nose toward north

    float f[3];
    f[0] = 100.0f;   // axial: thrust
    f[1] = 2.0f;     // lateral aero
    f[2] = -1.0f;

    // The true horizontal kinematic acceleration for this attitude and force,
    // rotated here independently of the filter so this is a real check and not
    // the aid agreeing with itself. Gravity is purely vertical in NED, so the
    // horizontal kinematic acceleration IS the horizontal part of the rotated
    // specific force — the identity the fix rests on.
    //
    // 3-2-1 aerospace DCM at yaw = roll = 0, pitch θ nose-up; row 0 of
    // T_NED2B is the nose axis in NED, so the N column is
    //   (cos θ, 0, sin θ) · (fx, fy, fz)  and the E column is (0, 1, 0) · f.
    // The convention is pinned by the resting case above, where this
    // expression must cancel to exactly zero — and does.
    const float p = 85.0f * kDeg;
    const float truth[2] = {std::cos(p) * f[0] + std::sin(p) * f[2], f[1]};

    float before[4], after[4];
    ekf.quat(before);
    ekf.accelMatchHeadingUpdate(f, truth);
    ekf.quat(after);

    EXPECT_LT(attitudeDeltaDeg(before, after), 0.05f)
        << "the aid disagreed with a measurement generated from its own attitude";
}

TEST(EkfHeadingAids, AGenuineLateralForceStillObservesRoll) {
    // The fix must not gate the aid into uselessness: this is the DOF it exists
    // for. With the attitude wrong about roll, a disagreeing world azimuth must
    // still move it.
    AidProbe ekf;
    ekf.setAttitudeDeg(0.0f, 89.0f, 0.0f);   // 1° off vertical

    float f[3];
    f[0] = 3.0f;     // small axial (coasting drag)
    f[1] = 6.0f;     // clear lateral force
    f[2] = -2.0f;

    float before[4], after[4];
    ekf.quat(before);
    // Point the measured world acceleration well away from the prediction.
    const float worldHoriz[2] = {-6.0f, 3.0f};
    ekf.accelMatchHeadingUpdate(f, worldHoriz);
    ekf.quat(after);

    EXPECT_GT(attitudeDeltaDeg(before, after), 1e-3f)
        << "the aid no longer observes roll even when the lateral force dominates";
}

// ── #1135 item 1 — characterisation, NOT a fix ──────────────────────────────

TEST(EkfHeadingAids, TheCourseAidDeliberatelyHasNoTiltGateOrTiltScaledR) {
    // READ THIS BEFORE "FIXING" WHAT IT PINS.
    //
    // #1135 item 1 is real: psi_pred = atan2(nose_E, nose_N) degenerates near
    // vertical, H = 2*d has |d| = 1 at every attitude, and R_course is a flat
    // 0.05 rad² — so the filter believes a near-vertical course exactly as much
    // as a level one. The issue proposes a tilt gate and/or R ∝ 1/sin²(tilt).
    //
    // All of it was built and measured, and all of it was refused:
    //
    //   tilt gate (~15°)   — 90.2° MEAN ascent attitude change on Rolly Polly V
    //                        across the four 2026-08-29 flights; fails sim
    //                        scenario D (saturated gyro) and the backlash test.
    //   R ∝ 1/cos²(tilt)   — cannot bound the damage (157°/121°/132° synthetic
    //                        drift at R ceilings of 1/10/100); inflating R does
    //                        not fix a bias, it just repeats it. Fails lash.
    //   innovation gate    — rewrites rpv ascent by 68.1° mean. On fast ascent
    //                        those innovations are a CONSISTENT ~14° crosswind
    //                        offset (circular R 0.74), not noise, and a
    //                        belief-based bound cannot separate the two.
    //
    // The mechanism behind every one of those failures: near vertical, rotation
    // about NED-down IS the vehicle's roll, so this aid is the filter's main
    // constraint on that degenerate DOF — exactly where a gate would remove it,
    // and exactly what a saturated gyro leaves nothing else to hold.
    //
    // What DID ship is the aids' own precondition rather than a belief test:
    // both aids stop once the caller reports the vehicle is no longer flying
    // nose-first (PR #1283), which removed the 72-90% of fusions that were
    // canopy drift at ~90° mean innovation. That gate is covered by
    // EKFHeadingAidGate in test_ekf.cpp.
    //
    // So: near vertical this aid still fuses, at full strength, on purpose.
    // This test states that, so a future reader finds the measured verdict
    // instead of re-deriving the same refuted fix from the same source reading.
    AidProbe ekf;
    ekf.setAttitudeDeg(0.0f, 89.9f, 0.0f);   // ~0.1° off vertical

    const float v[3] = {60.0f, 60.0f, -10.0f};   // course 45° off the nose

    float before[4], after[4];
    ekf.quat(before);
    ekf.velCourseHeadingUpdate(v);
    ekf.quat(after);

    EXPECT_GT(attitudeDeltaDeg(before, after), 1e-3f)
        << "the course aid acquired a near-vertical guard; see this test's comment "
           "for the four-flight measurements that refused one";
}
