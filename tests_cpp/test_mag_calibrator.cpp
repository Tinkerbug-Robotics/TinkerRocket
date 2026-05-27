// Host-side tests for TR_MagCalibrator — focused on the #206 post-accept
// verification state machine.  The sphere-fit math is exercised indirectly
// by feeding hand-constructed samples through SAMPLING → REVIEW → accept().
//
// Pre-existing SAMPLING / REVIEW behaviour is covered lightly here; the
// emphasis is on the new VERIFYING state, the gates in evaluateVerify(),
// and the regress-to-REVIEW path with reject_code = VERIFY_FAILED.

#include <gtest/gtest.h>
#include <TR_MagCalibrator.h>
#include <RocketComputerTypes.h>
#include <math.h>

namespace {

constexpr float UT_PER_LSB = 0.15f;

// Convenience: drive a known good sphere-fit by feeding many samples per
// canonical direction at R=50 µT (≈ 333 LSB) centered at (0,0,0).  Across
// the 26 hand-picked directions below we light up at least
// MAG_CAL_MIN_COVERAGE_BINS (22 of 32 truncated-icosahedron cells) and
// give a tiny residual.  Real captures give many more samples per wedge;
// the linear-LSQ solver only needs enough non-coplanar samples for the
// 4×4 to be well conditioned.
void driveCleanFit(MagCalibrator& cal) {
    cal.start();
    // The accel-wedge bucket logic requires a fresh accel reading per
    // sample (otherwise samples drop).  We just need *some* per-wedge
    // diversity — fake it by walking a small set of accel orientations.
    struct Dir { int16_t x, y, z; };
    // Cover 26 unit-vector directions roughly: ±X / ±Y / ±Z, the 12
    // edge midpoints, and the 8 corners.  Mapped through #148's
    // truncated-icosahedron Voronoi these land in 22+ distinct cells —
    // enough to pass MAG_CAL_MIN_COVERAGE_BINS = 22.  Magnitude ~1000
    // LSB so the wedge logic sees a clean unit-vector direction.
    Dir dirs[] = {
        {1000, 0, 0}, {-1000, 0, 0}, {0, 1000, 0}, {0, -1000, 0},
        {0, 0, 1000}, {0, 0, -1000},
        {707, 707, 0}, {-707, 707, 0}, {707, -707, 0}, {-707, -707, 0},
        {707, 0, 707}, {-707, 0, 707}, {707, 0, -707}, {-707, 0, -707},
        {0, 707, 707}, {0, -707, 707}, {0, 707, -707}, {0, -707, -707},
        {577, 577, 577}, {-577, 577, 577}, {577, -577, 577}, {577, 577, -577},
        {-577, -577, 577}, {-577, 577, -577}, {577, -577, -577}, {-577, -577, -577},
    };
    const int N = sizeof(dirs) / sizeof(dirs[0]);
    // Many samples per direction to clear the MAG_CAL_MIN_SAMPLES floor.
    const int per_dir = 40;  // 26 × 40 = 1040 > 500
    for (int rep = 0; rep < per_dir; rep++) {
        for (int i = 0; i < N; i++) {
            // Accel parallel to body direction.
            cal.setLiveAccel(dirs[i].x, dirs[i].y, dirs[i].z);
            // Mag at R=50 µT along the same direction → sphere of radius
            // ~333 LSB centered at origin.  Scale dir to unit, multiply
            // by R in LSB.
            const double L = sqrt((double)dirs[i].x * dirs[i].x +
                                  (double)dirs[i].y * dirs[i].y +
                                  (double)dirs[i].z * dirs[i].z);
            const double R_lsb = 50.0 / UT_PER_LSB;  // 333.33
            const int16_t mx = (int16_t)((double)dirs[i].x / L * R_lsb);
            const int16_t my = (int16_t)((double)dirs[i].y / L * R_lsb);
            const int16_t mz = (int16_t)((double)dirs[i].z / L * R_lsb);
            cal.addSample(mx, my, mz);
        }
    }
    ASSERT_TRUE(cal.computeFit());
    ASSERT_EQ((int)cal.getState(), (int)MagCalibrator::State::REVIEW);
}

// Feed verify samples by direction; each sample's |B| is at the requested
// value, oriented along that direction.  Drives both the accel wedge
// (for verify_coverage_mask_) and the mag value (for verify_min/max).
void feedVerifySamples(MagCalibrator& cal, float magnitude_uT, int n_per_dir) {
    int16_t dirs[][3] = {
        {1000, 0, 0}, {-1000, 0, 0}, {0, 1000, 0}, {0, -1000, 0},
        {0, 0, 1000}, {0, 0, -1000},
        {707, 707, 0}, {-707, -707, 0},
    };
    const int N = sizeof(dirs) / sizeof(dirs[0]);
    const double R_lsb = magnitude_uT / UT_PER_LSB;
    for (int rep = 0; rep < n_per_dir; rep++) {
        for (int i = 0; i < N; i++) {
            cal.setLiveAccel(dirs[i][0], dirs[i][1], dirs[i][2]);
            const double L = sqrt((double)dirs[i][0] * dirs[i][0] +
                                  (double)dirs[i][1] * dirs[i][1] +
                                  (double)dirs[i][2] * dirs[i][2]);
            const int16_t mx = (int16_t)((double)dirs[i][0] / L * R_lsb);
            const int16_t my = (int16_t)((double)dirs[i][1] / L * R_lsb);
            const int16_t mz = (int16_t)((double)dirs[i][2] / L * R_lsb);
            cal.addSample(mx, my, mz);
        }
    }
}

}  // namespace


// --- tessellation geometry (issue #148) ---

// directionWedge is private; we can't call it directly, but we can verify
// the same cells are hit by feeding samples in known directions through
// addSample (which routes through directionWedge internally for accel
// bucketing).  Confirms the 32 wedges all reachable from a fresh tumble.
TEST(MagCalibratorTessellation, AllCellsReachableFromIcosaCorners) {
    MagCalibrator cal;
    cal.start();
    // Feed a sample in each of the 32 cell center directions.  Each
    // sample should land in its own cell (Voronoi assignment), so the
    // coverage_mask should hit all 32 bits.
    struct V { float x, y, z; };
    const V centers[32] = {
        { 0.00f,-0.526f,-0.851f}, {-0.526f,-0.851f, 0.00f}, {-0.851f, 0.00f,-0.526f},
        { 0.00f,-0.526f, 0.851f}, {-0.526f, 0.851f, 0.00f}, {-0.851f, 0.00f, 0.526f},
        { 0.00f, 0.526f,-0.851f}, { 0.526f,-0.851f, 0.00f}, { 0.851f, 0.00f,-0.526f},
        { 0.00f, 0.526f, 0.851f}, { 0.526f, 0.851f, 0.00f}, { 0.851f, 0.00f, 0.526f},
        {-0.577f,-0.577f,-0.577f}, { 0.00f,-0.934f,-0.357f}, {-0.357f, 0.00f,-0.934f},
        { 0.357f, 0.00f,-0.934f}, { 0.577f,-0.577f,-0.577f}, {-0.934f,-0.357f, 0.00f},
        {-0.577f,-0.577f, 0.577f}, { 0.00f,-0.934f, 0.357f}, {-0.934f, 0.357f, 0.00f},
        {-0.577f, 0.577f,-0.577f}, {-0.357f, 0.00f, 0.934f}, { 0.577f,-0.577f, 0.577f},
        { 0.357f, 0.00f, 0.934f}, {-0.577f, 0.577f, 0.577f}, { 0.00f, 0.934f,-0.357f},
        { 0.00f, 0.934f, 0.357f}, { 0.577f, 0.577f,-0.577f}, { 0.934f,-0.357f, 0.00f},
        { 0.934f, 0.357f, 0.00f}, { 0.577f, 0.577f, 0.577f},
    };
    const int LSB = 1000;
    for (int i = 0; i < 32; i++) {
        const int16_t ax = (int16_t)(centers[i].x * LSB);
        const int16_t ay = (int16_t)(centers[i].y * LSB);
        const int16_t az = (int16_t)(centers[i].z * LSB);
        cal.setLiveAccel(ax, ay, az);
        cal.addSample(ax, ay, az);
    }
    uint16_t n; uint8_t cov; float B;
    cal.getProgress(n, cov, B);
    // Each cell center hits a distinct cell → all 32 bits lit.
    EXPECT_EQ(cov, 32) << "Expected all 32 wedges populated by their own center "
                         "directions; got " << (int)cov << " — Voronoi assignment "
                         "may have a tie between centers or the cell-center table "
                         "is out of sync with directionWedge's geometry.";
}


// --- baseline plumbing ---

TEST(MagCalibratorVerify, AcceptEntersVerifyingNotApplied) {
    MagCalibrator cal;
    driveCleanFit(cal);
    ASSERT_TRUE(cal.accept());
    EXPECT_EQ((int)cal.getState(), (int)MagCalibrator::State::VERIFYING);
    // Previously accept() would land in APPLIED — this test is the
    // anchor that protects #206 from regressing into the old behaviour.
}

TEST(MagCalibratorVerify, EvaluateOutsideVerifyingNoOps) {
    MagCalibrator cal;
    float worst = -1.0f;
    EXPECT_FALSE(cal.evaluateVerify(worst));
    EXPECT_EQ((int)cal.getState(), (int)MagCalibrator::State::IDLE);
}


// --- pass path ---

TEST(MagCalibratorVerify, PassesWhenCorrectedFieldIsClean) {
    MagCalibrator cal;
    driveCleanFit(cal);
    ASSERT_TRUE(cal.accept());

    // 50 µT, plenty of samples, full rotation — should pass.
    feedVerifySamples(cal, /*uT*/50.0f, /*per_dir*/30);

    float worst = -1.0f;
    EXPECT_TRUE(cal.evaluateVerify(worst));
    EXPECT_EQ((int)cal.getState(), (int)MagCalibrator::State::APPLIED);
}


// --- fail paths ---

TEST(MagCalibratorVerify, FailsWhenFieldTooHigh) {
    MagCalibrator cal;
    driveCleanFit(cal);
    ASSERT_TRUE(cal.accept());

    // 80 µT — above MAG_CAL_VERIFY_MAX_UT (70).  Simulates the Eagle
    // Claw scenario: fit passed R∈[20,80] gate but the rotated |B|
    // exceeds the EKF input gate's upper edge.
    feedVerifySamples(cal, 80.0f, 30);

    float worst = -1.0f;
    EXPECT_FALSE(cal.evaluateVerify(worst));
    EXPECT_EQ((int)cal.getState(), (int)MagCalibrator::State::REVIEW);

    int16_t cx, cy, cz; float R, res; uint8_t reject;
    cal.getResult(cx, cy, cz, R, res, reject);
    EXPECT_EQ(reject, MAG_CAL_REJECT_VERIFY_TOO_HIGH);
    EXPECT_NEAR(worst, 80.0f, 1.0f);  // worst observed |B|
}

TEST(MagCalibratorVerify, FailsWhenFieldTooLow) {
    MagCalibrator cal;
    driveCleanFit(cal);
    ASSERT_TRUE(cal.accept());

    feedVerifySamples(cal, 15.0f, 30);  // under VERIFY_MIN_UT (20)

    float worst = -1.0f;
    EXPECT_FALSE(cal.evaluateVerify(worst));
    EXPECT_EQ((int)cal.getState(), (int)MagCalibrator::State::REVIEW);
    int16_t cx, cy, cz; float R, res; uint8_t reject;
    cal.getResult(cx, cy, cz, R, res, reject);
    EXPECT_EQ(reject, MAG_CAL_REJECT_VERIFY_TOO_LOW);
    EXPECT_NEAR(worst, 15.0f, 1.0f);
}

TEST(MagCalibratorVerify, FailsWhenRangeTooWide) {
    MagCalibrator cal;
    driveCleanFit(cal);
    ASSERT_TRUE(cal.accept());

    // Mix 35 µT and 65 µT — both in band but spread is 30 µT > 25.
    feedVerifySamples(cal, 35.0f, 15);
    feedVerifySamples(cal, 65.0f, 15);

    float worst = -1.0f;
    EXPECT_FALSE(cal.evaluateVerify(worst));
    EXPECT_EQ((int)cal.getState(), (int)MagCalibrator::State::REVIEW);
    int16_t cx, cy, cz; float R, res; uint8_t reject;
    cal.getResult(cx, cy, cz, R, res, reject);
    EXPECT_EQ(reject, MAG_CAL_REJECT_VERIFY_RANGE_WIDE);
    // Worst is whichever extreme is further from R (~50 µT) — both are
    // equidistant here, so either 35 or 65 is acceptable.  Round-trip
    // through int16 LSB drops a small fraction of a µT, so test with
    // some slop.
    EXPECT_TRUE(fabsf(worst - 35.0f) < 1.0f || fabsf(worst - 65.0f) < 1.0f)
        << "worst=" << worst;
}

TEST(MagCalibratorVerify, FailsWhenTooFewSamples) {
    MagCalibrator cal;
    driveCleanFit(cal);
    ASSERT_TRUE(cal.accept());

    // Only a handful — well under MAG_CAL_VERIFY_MIN_SAMPLES (100).
    feedVerifySamples(cal, 50.0f, /*per_dir*/2);  // 8 dirs × 2 = 16

    float worst = -1.0f;
    EXPECT_FALSE(cal.evaluateVerify(worst));
    EXPECT_EQ((int)cal.getState(), (int)MagCalibrator::State::REVIEW);
    int16_t cx, cy, cz; float R, res; uint8_t reject;
    cal.getResult(cx, cy, cz, R, res, reject);
    EXPECT_EQ(reject, MAG_CAL_REJECT_VERIFY_FEW_SAMPLES);
}

TEST(MagCalibratorVerify, FailsWhenStationary) {
    MagCalibrator cal;
    driveCleanFit(cal);
    ASSERT_TRUE(cal.accept());

    // Hold the rocket still — same accel direction every sample.
    cal.setLiveAccel(0, 0, 1000);
    const double R_lsb = 50.0 / UT_PER_LSB;
    for (int i = 0; i < 200; i++) {
        cal.addSample(0, 0, (int16_t)R_lsb);
    }

    float worst = -1.0f;
    EXPECT_FALSE(cal.evaluateVerify(worst));
    EXPECT_EQ((int)cal.getState(), (int)MagCalibrator::State::REVIEW);
    int16_t cx, cy, cz; float R, res; uint8_t reject;
    cal.getResult(cx, cy, cz, R, res, reject);
    EXPECT_EQ(reject, MAG_CAL_REJECT_VERIFY_LOW_COVERAGE);
    // Single wedge gets one coverage bit — below MAG_CAL_VERIFY_MIN_COVERAGE_BINS.
}


// --- regression: retry after verify-fail recovers cleanly ---

TEST(MagCalibratorVerify, RetryFromReviewAfterVerifyFailWorks) {
    MagCalibrator cal;
    driveCleanFit(cal);
    ASSERT_TRUE(cal.accept());
    feedVerifySamples(cal, 80.0f, 30);  // force fail
    float worst = -1.0f;
    EXPECT_FALSE(cal.evaluateVerify(worst));
    ASSERT_EQ((int)cal.getState(), (int)MagCalibrator::State::REVIEW);

    // User taps Retry → start fresh.
    cal.retry();
    EXPECT_EQ((int)cal.getState(), (int)MagCalibrator::State::SAMPLING);
    // Sphere-fit buffer was cleared.
    uint16_t n; uint8_t cov; float B;
    cal.getProgress(n, cov, B);
    EXPECT_EQ(n, 0u);
}


// --- status-frame plumbing ---

TEST(MagCalibratorVerify, StatusFrameReportsVerifyingSubType) {
    MagCalibrator cal;
    driveCleanFit(cal);
    ASSERT_TRUE(cal.accept());

    MagCalStatusData frame;
    cal.buildStatusFrame(/*time_us*/12345, frame);
    EXPECT_EQ(frame.sub_type, MAG_CAL_SUB_VERIFYING);
}

TEST(MagCalibratorVerify, StatusFrameOnVerifyFailReportsReviewWithRejectCode) {
    MagCalibrator cal;
    driveCleanFit(cal);
    ASSERT_TRUE(cal.accept());
    feedVerifySamples(cal, 80.0f, 30);
    float worst = -1.0f;
    ASSERT_FALSE(cal.evaluateVerify(worst));

    MagCalStatusData frame;
    cal.buildStatusFrame(67890, frame);
    EXPECT_EQ(frame.sub_type, MAG_CAL_SUB_REVIEW);
    // 80 µT verify samples trip the TOO_HIGH gate.
    EXPECT_EQ(frame.reject_code, MAG_CAL_REJECT_VERIFY_TOO_HIGH);
}
