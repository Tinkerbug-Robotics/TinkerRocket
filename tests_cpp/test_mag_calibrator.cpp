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

// 32 canonical cell-centre directions used by both tessellation
// reachability tests below.  Each direction lies inside its own
// truncated-icosahedron Voronoi cell.
namespace {
struct CellCenter { float x, y, z; };
const CellCenter MAG_CELL_CENTERS_TEST[32] = {
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
constexpr int LSB_DRIVE = 1000;
} // anonymous namespace

// directionWedge is private; we can't call it directly, but we can verify
// the same cells are hit by feeding samples in known directions through
// addSample.  Per #148, coverage_mask is now only set once a wedge
// reaches MAG_CAL_MIN_SAMPLES_PER_WEDGE samples — so feed enough to
// promote every cell out of partial into captured.
TEST(MagCalibratorTessellation, AllCellsReachableFromIcosaCorners) {
    MagCalibrator cal;
    cal.start();
    for (int rep = 0; rep < (int)MAG_CAL_MIN_SAMPLES_PER_WEDGE; rep++) {
        for (int i = 0; i < 32; i++) {
            const int16_t ax = (int16_t)(MAG_CELL_CENTERS_TEST[i].x * LSB_DRIVE);
            const int16_t ay = (int16_t)(MAG_CELL_CENTERS_TEST[i].y * LSB_DRIVE);
            const int16_t az = (int16_t)(MAG_CELL_CENTERS_TEST[i].z * LSB_DRIVE);
            cal.setLiveAccel(ax, ay, az);
            cal.addSample(ax, ay, az);
        }
    }
    uint16_t n; uint8_t cov; float B;
    cal.getProgress(n, cov, B);
    EXPECT_EQ(cov, 32) << "Expected all 32 wedges captured after the per-wedge "
                         "sample threshold; got " << (int)cov;
}

// #148: a wedge with at least 1 sample but fewer than the threshold
// should sit in partial_mask, not coverage_mask.
TEST(MagCalibratorTessellation, PartialBeforeThreshold) {
    MagCalibrator cal;
    cal.start();
    // Feed just under threshold samples in 32 distinct cell directions.
    const int per_cell = (int)MAG_CAL_MIN_SAMPLES_PER_WEDGE - 1;
    for (int rep = 0; rep < per_cell; rep++) {
        for (int i = 0; i < 32; i++) {
            const int16_t ax = (int16_t)(MAG_CELL_CENTERS_TEST[i].x * LSB_DRIVE);
            const int16_t ay = (int16_t)(MAG_CELL_CENTERS_TEST[i].y * LSB_DRIVE);
            const int16_t az = (int16_t)(MAG_CELL_CENTERS_TEST[i].z * LSB_DRIVE);
            cal.setLiveAccel(ax, ay, az);
            cal.addSample(ax, ay, az);
        }
    }
    MagCalStatusData frame;
    cal.buildStatusFrame(0, frame);
    EXPECT_EQ(__builtin_popcount(frame.coverage_mask), 0)
        << "No wedges should be captured before the threshold";
    EXPECT_EQ(__builtin_popcount(frame.partial_mask), 32)
        << "All 32 wedges should be partial (in-progress)";
    EXPECT_EQ(frame.coverage_mask & frame.partial_mask, 0u)
        << "coverage_mask and partial_mask must be disjoint";

    // One more sample in each wedge → all cross the threshold.
    for (int i = 0; i < 32; i++) {
        const int16_t ax = (int16_t)(MAG_CELL_CENTERS_TEST[i].x * LSB_DRIVE);
        const int16_t ay = (int16_t)(MAG_CELL_CENTERS_TEST[i].y * LSB_DRIVE);
        const int16_t az = (int16_t)(MAG_CELL_CENTERS_TEST[i].z * LSB_DRIVE);
        cal.setLiveAccel(ax, ay, az);
        cal.addSample(ax, ay, az);
    }
    cal.buildStatusFrame(0, frame);
    EXPECT_EQ(__builtin_popcount(frame.coverage_mask), 32);
    EXPECT_EQ(frame.partial_mask, 0u)
        << "All wedges should now be captured; partial mask should be empty";
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


// #1138 item 1: feed the same sample set driveCleanFit() uses, but stop just
// before computeFit() and capture the mask the SAMPLING path built.
static void driveCleanFitSamplesOnly(MagCalibrator& cal, MagCalStatusData& out) {
    struct Dir { int16_t x, y, z; };
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
    for (int rep = 0; rep < 40; rep++) {
        for (int i = 0; i < N; i++) {
            cal.setLiveAccel(dirs[i].x, dirs[i].y, dirs[i].z);
            const double L = sqrt((double)dirs[i].x * dirs[i].x +
                                  (double)dirs[i].y * dirs[i].y +
                                  (double)dirs[i].z * dirs[i].z);
            const double R_lsb = 50.0 / UT_PER_LSB;
            cal.addSample((int16_t)((double)dirs[i].x / L * R_lsb),
                          (int16_t)((double)dirs[i].y / L * R_lsb),
                          (int16_t)((double)dirs[i].z / L * R_lsb));
        }
    }
    cal.buildStatusFrame(0, out);
}


// ── #1138 item 1: runFit() must not redefine what the coverage mask means ──
//
// Everywhere else, the mask is the #148 32-cell truncated-icosahedron Voronoi
// index produced by directionWedge(). runFit() recounted coverage with a
// legacy 3x3x3 cube binning (27 wedges, a 0.4-per-axis threshold) and assigned
// the result straight to coverage_mask_. So from the fit onwards the published
// mask meant something different from the one accumulated during sampling —
// while coverage_bins (its popcount), the wire field and the iOS REVIEW
// coverage row all kept reading it as 32-cell. The gate the mask feeds is
// MAG_CAL_MIN_COVERAGE_BINS = 22 of 32.

TEST(MagCalibratorCoverage, PostFitMaskUsesTheSame32CellSchemeAsSampling) {
    MagCalibrator cal;
    driveCleanFit(cal);

    MagCalStatusData frame{};
    cal.buildStatusFrame(0, frame);

    // Every set bit must be a legal 32-cell index. The cube scheme could set
    // bit 26 for a direction the Voronoi scheme would never assign there, but
    // the decisive property is that the two agree at all: recompute the mask
    // independently from the same directions through the public wedge API.
    EXPECT_LE(__builtin_popcount(frame.coverage_mask), 32);
    EXPECT_GE(__builtin_popcount(frame.coverage_mask),
              (int)MAG_CAL_MIN_COVERAGE_BINS)
        << "the fit must still clear the coverage gate it feeds";
}

TEST(MagCalibratorCoverage, PostFitMasksStayDisjoint) {
    // The documented invariant: a cell is either covered or partially filled,
    // never both. runFit() overwrote coverage_mask_ with a differently-indexed
    // value and left partial_mask_ alone, so the two could overlap and mean
    // different things at the same time. The buffer is frozen from REVIEW on,
    // so after the fit there is no "partially filled" state left to report.
    MagCalibrator cal;
    driveCleanFit(cal);

    MagCalStatusData frame{};
    cal.buildStatusFrame(0, frame);
    EXPECT_EQ(frame.coverage_mask & frame.partial_mask, 0u)
        << "coverage and partial masks overlap after the fit";
}

TEST(MagCalibratorCoverage, PostFitMaskMatchesTheSamplingMask) {
    // The decisive property. During SAMPLING the mask is built by
    // directionWedge() (32-cell Voronoi); runFit() then RECOUNTS it from the
    // fit-centred samples. Those two counts must describe the same cells --
    // the defect was that the recount used a different indexing scheme
    // entirely, so the mask silently changed meaning at the REVIEW boundary.
    //
    // driveCleanFit() feeds mag samples along the SAME directions as the accel
    // wedges and centres the sphere at the origin, so the fit-centred unit
    // vectors are those same directions and the recount must reproduce the
    // sampling mask exactly.
    MagCalibrator cal;
    MagCalStatusData before{};
    MagCalStatusData after{};

    cal.start();
    driveCleanFitSamplesOnly(cal, before);   // captures the mask pre-fit
    ASSERT_TRUE(cal.computeFit());
    cal.buildStatusFrame(0, after);

    EXPECT_EQ(after.coverage_mask, before.coverage_mask)
        << "runFit() recounted coverage into a different cell scheme";
}

// ===========================================================================
// The fit itself. Everything above exercises the state machine and the
// coverage tessellation; none of it checks that the sphere fit RECOVERS A
// KNOWN OFFSET, which is the arithmetic the whole feature rests on, or pins
// the sign convention that decides whether programming the result into the
// chip's OFFSET registers removes the hard iron or doubles it.
//
// The real case that motivated this: the Rolly Polly V nosecone board flew
// 2026-08-29 with a 210 µT hard iron — about four times Earth's field, stable
// to 1.6 µT across the flight, almost entirely on one body axis. Sphere-fitting
// that flight's own samples recovers the site field to 0.4 µT and its dip to
// 1.7°, so the offset is fully correctable. These tests pin that the
// calibrator would in fact have recovered it.
// ===========================================================================
namespace fitmath {

// Drive a fit over a sphere of radius R_uT CENTRED AT (cx, cy, cz) raw LSB —
// i.e. a magnetometer with that hard iron on it. Same direction set and sample
// counts as driveCleanFit, which is centred at the origin.
void driveOffsetFit(MagCalibrator& cal, int cx, int cy, int cz, float R_uT) {
    cal.start();
    struct Dir { int16_t x, y, z; };
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
    const double R_lsb = R_uT / UT_PER_LSB;
    for (int rep = 0; rep < 40; rep++) {
        for (int i = 0; i < N; i++) {
            cal.setLiveAccel(dirs[i].x, dirs[i].y, dirs[i].z);
            const double L = sqrt((double)dirs[i].x * dirs[i].x +
                                  (double)dirs[i].y * dirs[i].y +
                                  (double)dirs[i].z * dirs[i].z);
            // Earth field along this direction, PLUS the hard iron.
            const int16_t mx = (int16_t)(cx + (double)dirs[i].x / L * R_lsb);
            const int16_t my = (int16_t)(cy + (double)dirs[i].y / L * R_lsb);
            const int16_t mz = (int16_t)(cz + (double)dirs[i].z / L * R_lsb);
            cal.addSample(mx, my, mz);
        }
    }
    ASSERT_TRUE(cal.computeFit());
}

}  // namespace fitmath
using namespace fitmath;

TEST(MagCalibratorFit, RecoversAKnownHardIron) {
    // 20 µT of hard iron, arbitrary direction, on a 50 µT field.
    const int cx = 133, cy = -67, cz = 40;      // ~20/10/6 µT at 0.15 µT/LSB
    MagCalibrator cal;
    driveOffsetFit(cal, cx, cy, cz, 50.0f);
    int16_t fx, fy, fz; float R, res; uint8_t reject;
    cal.getResult(fx, fy, fz, R, res, reject);
    EXPECT_EQ((int)reject, (int)MAG_CAL_OK);
    EXPECT_NEAR((int)fx, cx, 3);
    EXPECT_NEAR((int)fy, cy, 3);
    EXPECT_NEAR((int)fz, cz, 3);
    EXPECT_NEAR(R, 50.0f, 1.0f);
    EXPECT_LT(res, 1.0f);
}

TEST(MagCalibratorFit, TheSignIsSubtractNotAdd) {
    // THE test that matters for the chip. The IIS2MDC OFFSET registers are
    // SUBTRACTED from the raw output, so the fitted centre must be programmed
    // as-is. If the convention were inverted, taking the fit at face value
    // would DOUBLE the hard iron rather than remove it — and on the pad that
    // reads as a plausible-looking number, not an obvious failure.
    const int cx = 133, cy = -67, cz = 40;
    MagCalibrator cal;
    driveOffsetFit(cal, cx, cy, cz, 50.0f);
    int16_t fx, fy, fz; float R, res; uint8_t reject;
    cal.getResult(fx, fy, fz, R, res, reject);

    // A raw sample: Earth field along +X plus the hard iron, as the chip sees it.
    const double R_lsb = 50.0 / UT_PER_LSB;
    const double raw[3] = { cx + R_lsb, (double)cy, (double)cz };

    auto mag_uT = [](double x, double y, double z) {
        return sqrt(x*x + y*y + z*z) * UT_PER_LSB;
    };
    // Uncorrected it is far from the field magnitude.
    EXPECT_GT(mag_uT(raw[0], raw[1], raw[2]), 60.0);
    // SUBTRACTING the fit — what the chip does — lands on the field.
    EXPECT_NEAR(mag_uT(raw[0] - fx, raw[1] - fy, raw[2] - fz), 50.0, 1.0);
    // ADDING it does not. This is the assertion that fails if the sign flips.
    EXPECT_GT(fabs(mag_uT(raw[0] + fx, raw[1] + fy, raw[2] + fz) - 50.0), 5.0);
}

TEST(MagCalibratorFit, RecoversTheRollyPollyVMagnitude) {
    // 210 µT on one axis — the real 2026-08-29 nosecone offset, four times
    // Earth's field. 210 / 0.15 = 1400 LSB, comfortably inside the int16 the
    // OFFSET registers hold, so this is correctable rather than merely large.
    const int cy = -1394;                       // ~209 µT on -Y, as measured
    MagCalibrator cal;
    driveOffsetFit(cal, 0, cy, 145, 50.3f);
    int16_t fx, fy, fz; float R, res; uint8_t reject;
    cal.getResult(fx, fy, fz, R, res, reject);
    EXPECT_EQ((int)reject, (int)MAG_CAL_OK)
        << "a 210 uT offset must not be rejected — the fit's R and residual "
           "gates are about field quality, not offset size";
    EXPECT_NEAR((int)fy, cy, 5);
    EXPECT_NEAR(R, 50.3f, 1.0f);
    EXPECT_LT(res, 1.5f);
}

TEST(MagCalibratorFit, AnOffsetBeyondTheRegisterRangeStillFitsAsInt16) {
    // The OFFSET registers are int16 in raw LSB, so the largest correctable
    // hard iron is 32767 LSB = 4915 uT. Well past anything a rocket carries,
    // but the fit must not silently wrap on the way there.
    const int cx = 20000;                       // 3000 uT
    MagCalibrator cal;
    driveOffsetFit(cal, cx, 0, 0, 50.0f);
    int16_t fx, fy, fz; float R, res; uint8_t reject;
    cal.getResult(fx, fy, fz, R, res, reject);
    EXPECT_NEAR((int)fx, cx, 10);
    EXPECT_GT((int)fx, 0) << "sign wrapped through int16";
    EXPECT_NEAR(R, 50.0f, 1.5f);
}


// --- #1312: the count scale is the chip's, not a constant ---
//
// The mini's QMC5883P counts are 100/3750 µT/LSB; read at the IIS2MDC's 0.15
// the same sphere is 5.6x too big and every fit is R_TOO_HIGH.  The fit and
// the offsets stay in counts; only the µT-side gates and telemetry scale.

namespace {

constexpr float QMC_UT_PER_LSB = 100.0f / 3750.0f;

// driveCleanFit()'s 26-direction tumble at 50 µT, in a given count scale.
void driveSphereAtScale(MagCalibrator& cal, float uT_per_lsb) {
    cal.start();
    struct Dir { int16_t x, y, z; };
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
    const double R_lsb = 50.0 / uT_per_lsb;
    for (int rep = 0; rep < 40; rep++) {
        for (int i = 0; i < N; i++) {
            cal.setLiveAccel(dirs[i].x, dirs[i].y, dirs[i].z);
            const double L = sqrt((double)dirs[i].x * dirs[i].x +
                                  (double)dirs[i].y * dirs[i].y +
                                  (double)dirs[i].z * dirs[i].z);
            cal.addSample((int16_t)((double)dirs[i].x / L * R_lsb),
                          (int16_t)((double)dirs[i].y / L * R_lsb),
                          (int16_t)((double)dirs[i].z / L * R_lsb));
        }
    }
    ASSERT_TRUE(cal.computeFit());
}

// feedVerifySamples() in a given count scale.
void feedVerifyAtScale(MagCalibrator& cal, float magnitude_uT, int n_per_dir,
                       float uT_per_lsb) {
    int16_t dirs[][3] = {
        {1000, 0, 0}, {-1000, 0, 0}, {0, 1000, 0}, {0, -1000, 0},
        {0, 0, 1000}, {0, 0, -1000},
        {707, 707, 0}, {-707, -707, 0},
    };
    const int N = sizeof(dirs) / sizeof(dirs[0]);
    const double R_lsb = magnitude_uT / uT_per_lsb;
    for (int rep = 0; rep < n_per_dir; rep++) {
        for (int i = 0; i < N; i++) {
            cal.setLiveAccel(dirs[i][0], dirs[i][1], dirs[i][2]);
            const double L = sqrt((double)dirs[i][0] * dirs[i][0] +
                                  (double)dirs[i][1] * dirs[i][1] +
                                  (double)dirs[i][2] * dirs[i][2]);
            cal.addSample((int16_t)((double)dirs[i][0] / L * R_lsb),
                          (int16_t)((double)dirs[i][1] / L * R_lsb),
                          (int16_t)((double)dirs[i][2] / L * R_lsb));
        }
    }
}

}  // namespace

TEST(MagCalibratorScale, DefaultIsTheIIS2MDCAndNonsenseIsIgnored) {
    MagCalibrator cal;
    EXPECT_FLOAT_EQ(cal.countScale(), 0.15f);
    cal.setCountScale(0.0f);
    EXPECT_FLOAT_EQ(cal.countScale(), 0.15f);
    cal.setCountScale(-1.0f);
    EXPECT_FLOAT_EQ(cal.countScale(), 0.15f);
    cal.setCountScale(QMC_UT_PER_LSB);
    EXPECT_FLOAT_EQ(cal.countScale(), QMC_UT_PER_LSB);
}

TEST(MagCalibratorScale, QmcCountsAreRejectedAtTheIIS2MDCScale) {
    // 50 µT is 1875 QMC counts; at 0.15 µT/LSB that is a 281 µT sphere.  This
    // is what the mini would have done with the calibrator as it was: no cal
    // could ever be accepted.
    MagCalibrator cal;
    driveSphereAtScale(cal, QMC_UT_PER_LSB);
    int16_t cx, cy, cz; float R, res; uint8_t reject;
    cal.getResult(cx, cy, cz, R, res, reject);
    EXPECT_EQ((int)reject, (int)MAG_CAL_REJECT_R_TOO_HIGH);
    EXPECT_NEAR(R, 281.25f, 3.0f);
}

TEST(MagCalibratorScale, QmcCountsFitAtTheQmcScale) {
    MagCalibrator cal;
    cal.setCountScale(QMC_UT_PER_LSB);
    driveSphereAtScale(cal, QMC_UT_PER_LSB);
    int16_t cx, cy, cz; float R, res; uint8_t reject;
    cal.getResult(cx, cy, cz, R, res, reject);
    EXPECT_EQ((int)reject, (int)MAG_CAL_OK);
    EXPECT_NEAR(R, 50.0f, 1.0f);
    EXPECT_LT(res, 1.0f);
    // The offsets are still counts — nothing about them scaled.
    EXPECT_NEAR((int)cx, 0, 20);
    EXPECT_NEAR((int)cy, 0, 20);
    EXPECT_NEAR((int)cz, 0, 20);
}

TEST(MagCalibratorScale, VerifyAndTheStatusFrameScaleTheSameWay) {
    MagCalibrator cal;
    cal.setCountScale(QMC_UT_PER_LSB);
    driveSphereAtScale(cal, QMC_UT_PER_LSB);
    ASSERT_TRUE(cal.accept());

    // A clean 50 µT verify pass in QMC counts: inside the 20-70 µT band only
    // if the verify accumulators use the same scale as the fit gate.
    feedVerifyAtScale(cal, 50.0f, 30, QMC_UT_PER_LSB);
    MagCalStatusData frame{};
    cal.buildStatusFrame(0, frame);
    EXPECT_NEAR(frame.inst_field_uT_x10 / 10.0f, 50.0f, 1.0f);
    EXPECT_NEAR(frame.field_R_uT_x10 / 10.0f, 50.0f, 1.0f);

    float worst = -1.0f;
    EXPECT_TRUE(cal.evaluateVerify(worst));
    EXPECT_EQ((int)cal.getState(), (int)MagCalibrator::State::APPLIED);
}

TEST(MagCalibratorScale, TheRollyPollyVHardIronInQmcCounts) {
    // #1303's 210 µT offset is 7875 QMC counts — the int16 offset the QMC
    // driver subtracts holds it with room to spare, and the fit recovers it.
    const int cy = -7838;                       // -209 µT on -Y in QMC counts
    MagCalibrator cal;
    cal.setCountScale(QMC_UT_PER_LSB);
    cal.start();
    struct Dir { int16_t x, y, z; };
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
    const double R_lsb = 50.3 / QMC_UT_PER_LSB;
    for (int rep = 0; rep < 40; rep++) {
        for (int i = 0; i < N; i++) {
            cal.setLiveAccel(dirs[i].x, dirs[i].y, dirs[i].z);
            const double L = sqrt((double)dirs[i].x * dirs[i].x +
                                  (double)dirs[i].y * dirs[i].y +
                                  (double)dirs[i].z * dirs[i].z);
            cal.addSample((int16_t)((double)dirs[i].x / L * R_lsb),
                          (int16_t)((double)dirs[i].y / L * R_lsb + cy),
                          (int16_t)((double)dirs[i].z / L * R_lsb + 145));
        }
    }
    ASSERT_TRUE(cal.computeFit());
    int16_t fx, fy, fz; float R, res; uint8_t reject;
    cal.getResult(fx, fy, fz, R, res, reject);
    EXPECT_EQ((int)reject, (int)MAG_CAL_OK);
    EXPECT_NEAR((int)fy, cy, 5);
    EXPECT_NEAR((int)fz, 145, 5);
    EXPECT_NEAR(R, 50.3f, 1.0f);
}


// --- The MMC5983MA in the calibration (mag_cal_mmc.h) ---
//
// An MMC5983MA board (the Tinker-Beetle V2) calibrates in the IIS2MDC's
// 0.15 µT counts: the FC restates each 18-bit MMC count for the calibrator and
// turns the fit back into MMC counts for the converter's software offset.

#include <mag_cal_mmc.h>

namespace {

constexpr uint32_t kMmcZero = 131072;   // 2^17, the MMC's zero field

uint32_t mmcRaw(int32_t centered) { return (uint32_t)(centered + (int32_t)kMmcZero); }

}  // namespace

TEST(MagCalMmcUnits, OneIis2mdcCountIs24576ThousandthsOfAnMmcCount) {
    EXPECT_EQ(mag_cal_mmc::toMmcCounts(1000), 24576);
    EXPECT_EQ(mag_cal_mmc::toMmcCounts(-1000), -24576);
    EXPECT_EQ(mag_cal_mmc::toMmcCounts(0), 0);
    EXPECT_EQ(mag_cal_mmc::toCalCounts(mmcRaw(24576), 0), 1000);
    EXPECT_EQ(mag_cal_mmc::toCalCounts(mmcRaw(-24576), 0), -1000);
    // 50 µT in either count reads 50 µT, to the half MMC count the rounding costs
    EXPECT_NEAR(mag_cal_mmc::toMmcCounts(333) * MAG_UT_PER_COUNT_MMC5983MA, 333 * 0.15,
                MAG_UT_PER_COUNT_MMC5983MA / 2);
}

TEST(MagCalMmcUnits, RoundsHalfAwayFromZeroBothWays) {
    // 12 MMC counts = 0.488 of a calibrator count -> 0; 13 = 0.529 -> 1
    EXPECT_EQ(mag_cal_mmc::toCalCounts(mmcRaw(12), 0), 0);
    EXPECT_EQ(mag_cal_mmc::toCalCounts(mmcRaw(13), 0), 1);
    EXPECT_EQ(mag_cal_mmc::toCalCounts(mmcRaw(-12), 0), 0);
    EXPECT_EQ(mag_cal_mmc::toCalCounts(mmcRaw(-13), 0), -1);
    // 1 calibrator count = 24.576 MMC counts -> 25
    EXPECT_EQ(mag_cal_mmc::toMmcCounts(1), 25);
    EXPECT_EQ(mag_cal_mmc::toMmcCounts(-1), -25);
}

TEST(MagCalMmcUnits, TheOffsetComesOffBeforeTheRestating) {
    // The calibrator sees what the converter turns into µT: centered less the offset.
    EXPECT_EQ(mag_cal_mmc::toCalCounts(mmcRaw(24576 + 4915), 4915), 1000);
    EXPECT_EQ(mag_cal_mmc::toCalCounts(mmcRaw(4915), 4915), 0);
    // Only 18 bits count
    EXPECT_EQ(mag_cal_mmc::toCalCounts(mmcRaw(24576) | 0xFFFC0000u, 0), 1000);
}

TEST(MagCalMmcUnits, TheWholeRangeWithAnyOffsetFitsInt16) {
    // Rail to rail, less a rail-sized offset the other way: 2^18 counts.
    EXPECT_EQ(mag_cal_mmc::toCalCounts(0u, 131072), -10667);
    EXPECT_EQ(mag_cal_mmc::toCalCounts(0x3FFFFu, -131072), 10667);
}

// A tumble of an MMC5983MA board with 210 µT of hard iron on it -- the
// Rolly-Polly V nosecone's, 34406 MMC counts, past what an int16 of MMC counts
// holds -- through the FC's restating: the fit, made MMC counts, is the hard
// iron; with it on, the verify pass sees the Earth field and passes.
TEST(MagCalibratorMmc, FitsAnMmcHardIronAndTheOffsetTakesItOut) {
    struct D { int16_t x, y, z; };
    const D dirs[] = {
        {1000, 0, 0}, {-1000, 0, 0}, {0, 1000, 0}, {0, -1000, 0},
        {0, 0, 1000}, {0, 0, -1000},
        {707, 707, 0}, {-707, 707, 0}, {707, -707, 0}, {-707, -707, 0},
        {707, 0, 707}, {-707, 0, 707}, {707, 0, -707}, {-707, 0, -707},
        {0, 707, 707}, {0, -707, 707}, {0, 707, -707}, {0, -707, -707},
        {577, 577, 577}, {-577, 577, 577}, {577, -577, 577}, {577, 577, -577},
        {-577, -577, 577}, {-577, 577, -577}, {577, -577, -577}, {-577, -577, -577},
    };
    const int N = sizeof(dirs) / sizeof(dirs[0]);
    const double R_counts = 50.3 / MAG_UT_PER_COUNT_MMC5983MA;          // 8241
    const int32_t hard[3] = {1229, -34406, 2376};                        // 7.5, -210, 14.5 µT

    // The MMC's raw count along a direction, hard iron on.
    auto raw = [&](const D& d, int axis) {
        const double L = sqrt((double)d.x * d.x + (double)d.y * d.y + (double)d.z * d.z);
        const double u = (axis == 0 ? d.x : axis == 1 ? d.y : d.z) / L;
        return mmcRaw((int32_t)lround(u * R_counts) + hard[axis]);
    };

    MagCalibrator cal;   // default scale, 0.15 µT: the FC's on an MMC board
    cal.start();
    for (int rep = 0; rep < 40; rep++)
        for (int i = 0; i < N; i++) {
            cal.setLiveAccel(dirs[i].x, dirs[i].y, dirs[i].z);
            cal.addSample(mag_cal_mmc::toCalCounts(raw(dirs[i], 0), 0),    // session: offset zeroed
                          mag_cal_mmc::toCalCounts(raw(dirs[i], 1), 0),
                          mag_cal_mmc::toCalCounts(raw(dirs[i], 2), 0));
        }
    ASSERT_TRUE(cal.computeFit());
    int16_t fx, fy, fz; float R, res; uint8_t reject;
    cal.getResult(fx, fy, fz, R, res, reject);
    EXPECT_EQ((int)reject, (int)MAG_CAL_OK);
    EXPECT_NEAR(R, 50.3f, 0.5f);
    EXPECT_LT(res, 0.5f);

    // The fit, as the converter's offset: within one calibrator count (0.15 µT).
    const int32_t off[3] = {mag_cal_mmc::toMmcCounts(fx), mag_cal_mmc::toMmcCounts(fy),
                            mag_cal_mmc::toMmcCounts(fz)};
    for (int a = 0; a < 3; a++)
        EXPECT_NEAR(off[a], hard[a], 25) << "axis " << a;

    // Verify: samples now come less the candidate offset, as the FC feeds them.
    ASSERT_TRUE(cal.accept());
    for (int rep = 0; rep < 30; rep++)
        for (int i = 0; i < N; i++) {
            cal.setLiveAccel(dirs[i].x, dirs[i].y, dirs[i].z);
            cal.addSample(mag_cal_mmc::toCalCounts(raw(dirs[i], 0), off[0]),
                          mag_cal_mmc::toCalCounts(raw(dirs[i], 1), off[1]),
                          mag_cal_mmc::toCalCounts(raw(dirs[i], 2), off[2]));
        }
    float worst = -1.0f;
    EXPECT_TRUE(cal.evaluateVerify(worst)) << "worst |B| " << worst << " uT";

    // And the converter's own sum: centered less the offset, at its scale.
    const D up = {0, 0, 1000};
    double b2 = 0.0;
    for (int a = 0; a < 3; a++) {
        const double ut = (mag_cal_mmc::centered(raw(up, a)) - off[a]) * MAG_UT_PER_COUNT_MMC5983MA;
        b2 += ut * ut;
    }
    EXPECT_NEAR(sqrt(b2), 50.3, 0.5);
}
