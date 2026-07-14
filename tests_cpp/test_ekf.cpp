#include <gtest/gtest.h>
#include "TR_GpsInsEKF.h"
#include <cmath>

// ---------- Helpers ----------

static constexpr double LAT_DEG = 33.7;   // Approximate launch site
static constexpr double LON_DEG = -118.4;
static constexpr double ALT_M   = 100.0;
static constexpr double LAT_RAD = LAT_DEG * M_PI / 180.0;
static constexpr double LON_RAD = LON_DEG * M_PI / 180.0;

// Build stationary IMU data: gravity in FRD body frame pointing nose-up (+Z = down = +9.81)
static EkfIMUData makeStationaryIMU(uint32_t time_us) {
    EkfIMUData imu;
    imu.time_us = time_us;
    imu.acc_x = 0.0; imu.acc_y = 0.0; imu.acc_z = 9.807;
    imu.gyro_x = 0.0; imu.gyro_y = 0.0; imu.gyro_z = 0.0;
    return imu;
}

static EkfGNSSDataLLA makeStationaryGNSS(uint32_t time_us) {
    EkfGNSSDataLLA gnss;
    gnss.time_us = time_us;
    gnss.lat_rad = LAT_RAD;
    gnss.lon_rad = LON_RAD;
    gnss.alt_m = ALT_M;
    gnss.vel_n_mps = 0; gnss.vel_e_mps = 0; gnss.vel_d_mps = 0;
    return gnss;
}

static EkfMagData makeStationaryMag(uint32_t time_us) {
    EkfMagData mag;
    mag.time_us = time_us;
    // Approximate LA mag field in FRD body frame (nose up, N facing north)
    mag.mag_x = 22.0; mag.mag_y = 0.0; mag.mag_z = 42.0;
    return mag;
}

// Nose-up (vertical) fixtures physically consistent with the EKF's init
// attitude (quat = [0.707,0,0.707,0] → body-X = up).  Specific force points
// up along the nose (+X).  FRD body mag for an Earth field of (22 N, 0 E,
// 42 D) µT (magnetic north) rotated into the nose-up body frame → heading ≈ 0.
static EkfIMUData makeNoseUpIMU(uint32_t time_us) {
    EkfIMUData imu;
    imu.time_us = time_us;
    imu.acc_x = 9.807; imu.acc_y = 0.0; imu.acc_z = 0.0;   // specific force up = +X (nose)
    imu.gyro_x = 0.0; imu.gyro_y = 0.0; imu.gyro_z = 0.0;
    return imu;
}
static EkfMagData makeNoseUpMag(uint32_t time_us) {
    EkfMagData mag;
    mag.time_us = time_us;
    mag.mag_x = -42.0; mag.mag_y = 0.0; mag.mag_z = 22.0;
    return mag;
}

// Hamilton quaternion product (scalar-first) and geodesic angle (deg) —
// singularity-safe heading comparison (Euler yaw is gimbal-locked at vertical).
static void tqMul(const float a[4], const float b[4], float o[4]) {
    o[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    o[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    o[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    o[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}
static float tqGeodesicDeg(const float a[4], const float b[4]) {
    float d = std::fabs(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]);
    if (d > 1.0f) d = 1.0f;
    return 2.0f * std::acos(d) * 180.0f / (float)M_PI;
}

class EKFTest : public ::testing::Test {
protected:
    GpsInsEKF ekf;
};

// ---------- Tests ----------

TEST_F(EKFTest, Init_LLA_SetsPosition) {
    uint32_t t = 0;
    ekf.init(makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));

    double pos[3];
    ekf.getPosEst(pos);
    EXPECT_NEAR(pos[0], LAT_RAD, 1e-4); // lat
    EXPECT_NEAR(pos[1], LON_RAD, 1e-4); // lon
    EXPECT_NEAR(pos[2], ALT_M,   1.0);  // alt
}

TEST_F(EKFTest, Init_ECEF_SetsPosition) {
    // Convert LLA to ECEF for init
    double cos_lat = std::cos(LAT_RAD), sin_lat = std::sin(LAT_RAD);
    double cos_lon = std::cos(LON_RAD), sin_lon = std::sin(LON_RAD);
    constexpr double a = 6378137.0;
    constexpr double e2 = 6.69437999014e-3;
    double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);

    EkfGNSSData gnss;
    gnss.time_us = 0;
    gnss.ecef_x = (N + ALT_M) * cos_lat * cos_lon;
    gnss.ecef_y = (N + ALT_M) * cos_lat * sin_lon;
    gnss.ecef_z = (N * (1 - e2) + ALT_M) * sin_lat;
    gnss.ecef_vx = 0; gnss.ecef_vy = 0; gnss.ecef_vz = 0;

    ekf.init(makeStationaryIMU(0), gnss, makeStationaryMag(0));

    double pos[3];
    ekf.getPosEst(pos);
    EXPECT_NEAR(pos[0], LAT_RAD, 1e-3);
    EXPECT_NEAR(pos[1], LON_RAD, 1e-3);
    EXPECT_NEAR(pos[2], ALT_M,   5.0);
}

TEST_F(EKFTest, QuatNormPreserved) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));

    uint32_t t = 2000; // 2ms step
    for (int i = 0; i < 1000; i++) {
        t += 2000;
        ekf.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    }

    float q[4];
    ekf.getQuaternion(q);
    float norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    EXPECT_NEAR(norm, 1.0f, 0.01f);
}

TEST_F(EKFTest, StationaryConvergence_PositionCovariance) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));

    float cov_initial[3];
    ekf.getCovPos(cov_initial);

    uint32_t t = 0;
    for (int i = 0; i < 5000; i++) { // ~10 seconds at 500 Hz
        t += 2000;
        ekf.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    }

    float cov_final[3];
    ekf.getCovPos(cov_final);

    // Position covariance should decrease after convergence
    for (int i = 0; i < 3; i++) {
        EXPECT_LT(cov_final[i], cov_initial[i]);
    }
}

TEST_F(EKFTest, StationaryVelocityAccuracy) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));

    uint32_t t = 0;
    for (int i = 0; i < 15000; i++) { // 30 seconds at 500 Hz
        t += 2000;
        ekf.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    }

    float vel[3];
    ekf.getVelEst(vel);
    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(vel[i], 0.0f, 0.5f); // < 0.5 m/s for stationary
    }
}

// Rail-tilt fixtures (85° elevation, roll 0, heading 0) — a realistic pad.
// #480: exact-vertical is both numerically degenerate (psi_pred =
// atan2(~0, ~0)) and physically heading-weak from accel+mag (the accel-
// derived roll in the tilt-comp is singular); the old constant 3° R masked
// both by overpowering the artifacts every tick. The guarantees these tests
// exist for — pad heading converges and holds before launch — are encoded
// at the rail angle where the physics supports them; exact vertical gets a
// bounded-behavior guard below.
// down-in-body d = (−sinθ, 0, cosθ); acc = −g·d; mag = R_NED2B·(22,0,42).
static EkfIMUData makeRail85IMU(uint32_t time_us) {
    EkfIMUData imu;
    imu.time_us = time_us;
    imu.acc_x = 9.7697; imu.acc_y = 0.0; imu.acc_z = -0.8548;
    imu.gyro_x = 0.0; imu.gyro_y = 0.0; imu.gyro_z = 0.0;
    return imu;
}
static EkfMagData makeRail85Mag(uint32_t time_us) {
    EkfMagData mag;
    mag.time_us = time_us;
    mag.mag_x = -39.92; mag.mag_y = 0.0; mag.mag_z = 25.58;
    return mag;
}

// Heading fusion must CONVERGE on the rail — the case the old Mahony
// correction (cos²(pitch) → 0 near vertical) could not handle.
TEST_F(EKFTest, MagHeadingConvergesOnRail) {
    ekf.init(makeRail85IMU(0), makeStationaryGNSS(0), makeRail85Mag(0));
    uint32_t t = 0;
    for (int i = 0; i < 2000; i++) { t += 2000;
        ekf.update(true, makeRail85IMU(t), makeStationaryGNSS(t), makeRail85Mag(t)); }
    float q_ref[4]; ekf.getQuaternion(q_ref);

    // Inject a 50° heading error (rotation about NED-down), then reopen the
    // attitude covariance so the filter has room to correct it.
    const float a = 50.0f * (float)M_PI / 180.0f;
    const float qyaw[4] = { std::cos(a/2), 0.0f, 0.0f, std::sin(a/2) };  // about NED +Z (down)
    float q_err[4]; tqMul(qyaw, q_ref, q_err);
    ekf.setQuaternion(q_err[0], q_err[1], q_err[2], q_err[3]);
    for (int i = 6; i < 9; i++) ekf.inflateCovDiag(i, 1.0f);
    EXPECT_GT(tqGeodesicDeg(q_err, q_ref), 30.0f);     // error really was injected

    for (int i = 0; i < 6000; i++) { t += 2000;
        ekf.update(true, makeRail85IMU(t), makeStationaryGNSS(t), makeRail85Mag(t)); }
    float q_fin[4]; ekf.getQuaternion(q_fin);
    EXPECT_LT(tqGeodesicDeg(q_fin, q_ref), 8.0f);      // heading converged back
}

// Heading fusion must be STABLE (no drift) while stationary on the rail —
// the symptom we observed (±180° roll/yaw cycling) must not recur.
TEST_F(EKFTest, MagHeadingStableOnRail) {
    ekf.init(makeRail85IMU(0), makeStationaryGNSS(0), makeRail85Mag(0));
    uint32_t t = 0;
    for (int i = 0; i < 2000; i++) { t += 2000;
        ekf.update(true, makeRail85IMU(t), makeStationaryGNSS(t), makeRail85Mag(t)); }
    float q_a[4]; ekf.getQuaternion(q_a);
    for (int i = 0; i < 4000; i++) { t += 2000;
        ekf.update(true, makeRail85IMU(t), makeStationaryGNSS(t), makeRail85Mag(t)); }
    float q_b[4]; ekf.getQuaternion(q_b);
    EXPECT_LT(tqGeodesicDeg(q_a, q_b), 2.0f);          // no heading drift over 8 s
    float n = std::sqrt(q_b[0]*q_b[0] + q_b[1]*q_b[1] + q_b[2]*q_b[2] + q_b[3]*q_b[3]);
    EXPECT_NEAR(n, 1.0f, 0.01f);
}

// At EXACT vertical the mag heading update is gated off entirely (#480):
// the accel-derived roll is atan2(noise, noise) and psi_pred is atan2(~0,~0)
// — float garbage that differs by platform (CI x86 dragged heading ±175°
// where the bench arm held). With the gate, the filter simply HOLDS heading
// on the gyro: an injected error is neither corrected nor worsened, and the
// quaternion stays sane (the old ±180° cycling symptom cannot recur).
TEST_F(EKFTest, MagHeadingHeldAtExactVertical) {
    ekf.init(makeNoseUpIMU(0), makeStationaryGNSS(0), makeNoseUpMag(0));
    uint32_t t = 0;
    for (int i = 0; i < 2000; i++) { t += 2000;
        ekf.update(true, makeNoseUpIMU(t), makeStationaryGNSS(t), makeNoseUpMag(t)); }
    float q_ref[4]; ekf.getQuaternion(q_ref);

    const float a = 50.0f * (float)M_PI / 180.0f;
    const float qyaw[4] = { std::cos(a/2), 0.0f, 0.0f, std::sin(a/2) };
    float q_err[4]; tqMul(qyaw, q_ref, q_err);
    ekf.setQuaternion(q_err[0], q_err[1], q_err[2], q_err[3]);
    for (int i = 6; i < 9; i++) ekf.inflateCovDiag(i, 1.0f);

    for (int i = 0; i < 6000; i++) { t += 2000;
        ekf.update(true, makeNoseUpIMU(t), makeStationaryGNSS(t), makeNoseUpMag(t)); }
    float q_fin[4]; ekf.getQuaternion(q_fin);
    EXPECT_NEAR(tqGeodesicDeg(q_fin, q_ref), 50.0f, 5.0f);  // held: no fusion, no runaway
    float n = std::sqrt(q_fin[0]*q_fin[0] + q_fin[1]*q_fin[1] + q_fin[2]*q_fin[2] + q_fin[3]*q_fin[3]);
    EXPECT_NEAR(n, 1.0f, 0.01f);
}

TEST_F(EKFTest, SetQuaternion_ResetsAttCovariance) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));

    float cov_before[3];
    ekf.getCovOrient(cov_before);

    ekf.setQuaternion(1.0f, 0.0f, 0.0f, 0.0f);

    float cov_after[3];
    ekf.getCovOrient(cov_after);
    for (int i = 0; i < 3; i++) {
        EXPECT_LT(cov_after[i], cov_before[i]);
    }
}

TEST_F(EKFTest, GpsNoiseScale_InflatesR) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));
    ekf.setGpsNoiseScale(5.0f);
    EXPECT_FLOAT_EQ(ekf.getGpsNoiseScale(), 5.0f);

    // Run a few updates - with higher noise scale, position covariance should
    // stay higher than with scale = 1.0
    uint32_t t = 0;
    for (int i = 0; i < 500; i++) {
        t += 2000;
        ekf.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    }
    float cov_scaled[3];
    ekf.getCovPos(cov_scaled);

    // Reset and do same without scaling
    GpsInsEKF ekf2;
    ekf2.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));
    t = 0;
    for (int i = 0; i < 500; i++) {
        t += 2000;
        ekf2.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    }
    float cov_nominal[3];
    ekf2.getCovPos(cov_nominal);

    // Scaled covariance should be larger (GPS measurements weighted less)
    EXPECT_GT(cov_scaled[0], cov_nominal[0]);
}

TEST_F(EKFTest, BaroUpdateReducesAltCovariance) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));

    // Run a few time updates without baro to let alt covariance grow
    uint32_t t = 0;
    for (int i = 0; i < 100; i++) {
        t += 2000;
        ekf.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    }

    // EKF is 15-state (baro offset was dropped); baro directly updates the
    // altitude (down) component of position. Read cov[2] before/after.
    float cov_pos[3];
    ekf.getCovPos(cov_pos);
    float cov_alt_before = cov_pos[2];

    EkfBaroData baro;
    baro.time_us = t;
    baro.altitude_m = ALT_M;
    ekf.baroMeasUpdate(baro);

    ekf.getCovPos(cov_pos);
    float cov_alt_after = cov_pos[2];
    EXPECT_LE(cov_alt_after, cov_alt_before);
}

TEST_F(EKFTest, InvertMatrix6x6_Identity) {
    // Create identity matrix
    float I[36] = {};
    for (int i = 0; i < 6; i++) I[i*6+i] = 1.0f;

    // The invertMatrix6x6 method is private, but we can test it indirectly
    // by running the EKF with GPS measurements (which use 6x6 inversion).
    // Instead, test that the EKF doesn't produce NaN after init+update.
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));
    ekf.update(true, makeStationaryIMU(2000), makeStationaryGNSS(2000), makeStationaryMag(2000));

    float vel[3];
    ekf.getVelEst(vel);
    for (int i = 0; i < 3; i++) {
        EXPECT_FALSE(std::isnan(vel[i]));
        EXPECT_FALSE(std::isinf(vel[i]));
    }
}

TEST_F(EKFTest, Quat2Euler_KnownAngles) {
    // Identity quaternion [1,0,0,0] should give [0,0,0] euler
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));
    ekf.setQuaternion(1.0f, 0.0f, 0.0f, 0.0f);

    // Run one update to trigger Euler computation
    ekf.update(true, makeStationaryIMU(2000), makeStationaryGNSS(2000), makeStationaryMag(2000));

    float orient[3];
    ekf.getOrientEst(orient);
    // With identity quaternion injected, roll and pitch should be near zero
    // (yaw depends on mag correction, so we don't check it tightly)
    EXPECT_NEAR(orient[0], 0.0f, 0.2f); // roll
    EXPECT_NEAR(orient[1], 0.0f, 0.2f); // pitch
}

// ── #257/#265: EKF health signal (isHealthy) ──
TEST_F(EKFTest, IsHealthy_TrueWhenConverged) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));
    for (uint32_t i = 1; i <= 50; ++i) {
        uint32_t t = i * 2000;  // 2 ms steps
        ekf.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    }
    EXPECT_TRUE(ekf.isHealthy());
}

// A non-finite IMU sample drives the quaternion to NaN; isHealthy() must report
// the filter unhealthy so #257 excludes the EKF voters (and #265 stows the fins).
TEST_F(EKFTest, IsHealthy_FalseOnNonFiniteInput) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));
    ekf.update(true, makeStationaryIMU(2000), makeStationaryGNSS(2000), makeStationaryMag(2000));
    ASSERT_TRUE(ekf.isHealthy());   // healthy before the fault

    EkfIMUData bad = makeStationaryIMU(4000);
    bad.gyro_x = NAN;               // non-finite gyro → quaternion propagates to NaN
    ekf.update(true, bad, makeStationaryGNSS(4000), makeStationaryMag(4000));
    EXPECT_FALSE(ekf.isHealthy()) << "EKF must report unhealthy after a non-finite update";
}

TEST_F(EKFTest, LargeTimestepStability) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));

    // dt = 0.1s (10 Hz) - much slower than normal 500 Hz
    uint32_t t = 0;
    for (int i = 0; i < 100; i++) {
        t += 100000; // 100ms
        ekf.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    }

    float vel[3];
    ekf.getVelEst(vel);
    float q[4];
    ekf.getQuaternion(q);
    double pos[3];
    ekf.getPosEst(pos);

    for (int i = 0; i < 3; i++) {
        EXPECT_FALSE(std::isnan(vel[i]));
        EXPECT_FALSE(std::isinf(vel[i]));
        EXPECT_FALSE(std::isnan(pos[i]));
        EXPECT_FALSE(std::isinf(pos[i]));
    }
    for (int i = 0; i < 4; i++) {
        EXPECT_FALSE(std::isnan(q[i]));
    }
}

TEST_F(EKFTest, PCovarianceDiagClamping) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));

    // Run 10000 steps WITHOUT GPS to stress covariance growth
    // Create GNSS with old timestamp so it's never applied
    uint32_t t = 0;
    for (int i = 0; i < 10000; i++) {
        t += 2000;
        EkfIMUData imu = makeStationaryIMU(t);
        EkfGNSSDataLLA gnss = makeStationaryGNSS(0); // stale timestamp
        ekf.update(true, imu, gnss, makeStationaryMag(t));
    }

    // Position covariance should be clamped (P_MAX_POS = 1e8)
    float cov_pos[3];
    ekf.getCovPos(cov_pos);
    for (int i = 0; i < 3; i++) {
        EXPECT_LE(cov_pos[i], 1e9f); // generous bound
        EXPECT_FALSE(std::isinf(cov_pos[i]));
    }

    // Velocity covariance should be clamped (P_MAX_VEL = 1e4)
    float cov_vel[3];
    ekf.getCovVel(cov_vel);
    for (int i = 0; i < 3; i++) {
        EXPECT_FALSE(std::isinf(cov_vel[i]));
    }
}

TEST_F(EKFTest, SetPosition_UpdatesState) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));

    double new_lat = 34.0 * M_PI / 180.0;
    double new_lon = -117.0 * M_PI / 180.0;
    double new_alt = 500.0;
    ekf.setPosition(new_lat, new_lon, new_alt);

    double pos[3];
    ekf.getPosEst(pos);
    EXPECT_NEAR(pos[0], new_lat, 1e-6);
    EXPECT_NEAR(pos[1], new_lon, 1e-6);
    EXPECT_NEAR(pos[2], new_alt, 1e-6);
}

TEST_F(EKFTest, SetVelocity_UpdatesState) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));

    ekf.setVelocity(10.0f, -5.0f, 2.0f);

    float vel[3];
    ekf.getVelEst(vel);
    EXPECT_NEAR(vel[0], 10.0f, 1e-5f);
    EXPECT_NEAR(vel[1], -5.0f, 1e-5f);
    EXPECT_NEAR(vel[2], 2.0f,  1e-5f);
}

TEST_F(EKFTest, NoNaN_AfterManyUpdates) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));

    uint32_t t = 0;
    for (int i = 0; i < 30000; i++) { // 60 seconds at 500 Hz
        t += 2000;
        ekf.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));

        if (i % 10 == 0) {
            EkfBaroData baro;
            baro.time_us = t;
            baro.altitude_m = ALT_M;
            ekf.baroMeasUpdate(baro);
        }
    }

    float vel[3], orient[3], q[4], ab[3], gb[3];
    double pos[3];
    ekf.getVelEst(vel);
    ekf.getPosEst(pos);
    ekf.getOrientEst(orient);
    ekf.getQuaternion(q);
    ekf.getAccelBias(ab);
    ekf.getRotRateBias(gb);

    for (int i = 0; i < 3; i++) {
        EXPECT_FALSE(std::isnan(vel[i])) << "vel[" << i << "] is NaN";
        EXPECT_FALSE(std::isnan(pos[i])) << "pos[" << i << "] is NaN";
        EXPECT_FALSE(std::isnan(orient[i])) << "orient[" << i << "] is NaN";
        EXPECT_FALSE(std::isnan(ab[i])) << "accel_bias[" << i << "] is NaN";
        EXPECT_FALSE(std::isnan(gb[i])) << "gyro_bias[" << i << "] is NaN";
    }
    for (int i = 0; i < 4; i++) {
        EXPECT_FALSE(std::isnan(q[i])) << "q[" << i << "] is NaN";
    }
}

// ---------- #440: frozen IMU timestamp must hold, not drift ----------
//
// The dt floor used to rewrite a repeated timestamp's dt=0 to 2 ms, so a
// stalled/wedged/replayed IMU stream re-integrated the same sample as if
// time were passing — velocity and attitude drifted while isHealthy()
// stayed true. The filter must HOLD its state through a stall and resume
// cleanly when timestamps advance again.

TEST_F(EKFTest, FrozenTimestamp_HoldsAttitudeAndVelocity) {
    uint32_t t = 1000;
    ekf.init(makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    for (int i = 0; i < 200; i++) {
        t += 2000;
        ekf.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    }

    float q_before[4], v_before[3];
    ekf.getQuaternion(q_before);
    ekf.getVelEst(v_before);

    // IMU stalls at timestamp t while reporting a hard roll + a lateral
    // specific-force error — the worst case for fabricated integration.
    // 1000 frozen frames × the old fabricated 2 ms = 2 s of phantom time
    // (≈ 200° of phantom roll pre-fix).
    EkfIMUData frozen = makeStationaryIMU(t);
    frozen.gyro_x = 100.0;   // dps
    frozen.acc_y  = 3.0;     // m/s² lateral error
    for (int i = 0; i < 1000; i++) {
        ekf.update(true, frozen, makeStationaryGNSS(t), makeStationaryMag(t));
    }

    float q_after[4], v_after[3];
    ekf.getQuaternion(q_after);
    ekf.getVelEst(v_after);

    EXPECT_EQ(ekf.frozenDtSkips(), 1000u);
    // Exact equality: a skipped tick touches NOTHING, so the state must be
    // bit-identical. (A geodesic comparison is unusable here — the stored
    // quaternion's norm sits one float-ulp under 1.0, so acos() reports
    // ~0.04° even between identical values.)
    for (int i = 0; i < 4; i++) {
        EXPECT_EQ(q_after[i], q_before[i])
            << "attitude must hold bit-exact through a stall (q[" << i << "])";
    }
    for (int i = 0; i < 3; i++) {
        EXPECT_EQ(v_after[i], v_before[i])
            << "velocity must hold bit-exact through a stall (axis " << i << ")";
    }
}

TEST_F(EKFTest, FrozenTimestamp_ResumesCleanly) {
    uint32_t t = 1000;
    ekf.init(makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    for (int i = 0; i < 100; i++) {
        t += 2000;
        ekf.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    }

    // Brief stall, then the stream resumes with advancing timestamps.
    for (int i = 0; i < 50; i++) {
        ekf.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    }
    float q_stall[4]; ekf.getQuaternion(q_stall);

    for (int i = 0; i < 200; i++) {
        t += 2000;
        ekf.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    }
    float q_resumed[4]; ekf.getQuaternion(q_resumed);

    // Stationary data before, during, and after: the resumed filter should
    // stay converged at the stationary attitude, not jump (a resume glitch
    // would show as a large geodesic step).
    EXPECT_LT(tqGeodesicDeg(q_stall, q_resumed), 1.0f)
        << "filter must resume smoothly after a stall";
    EXPECT_EQ(ekf.frozenDtSkips(), 50u);
}

TEST_F(EKFTest, AdvancingTimestamps_NeverTripTheSkip) {
    // A healthy stream (the normal 500 Hz cadence every other test uses)
    // must never hit the frozen-timestamp path — the fix is provably inert
    // outside the anomaly it targets.
    uint32_t t = 1000;
    ekf.init(makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    for (int i = 0; i < 500; i++) {
        t += 2000;
        ekf.update(true, makeStationaryIMU(t), makeStationaryGNSS(t), makeStationaryMag(t));
    }
    EXPECT_EQ(ekf.frozenDtSkips(), 0u);
}

// ---------- Rank-1 Joseph baro covariance update (perf follow-up to #450) ----------
//
// baroMeasUpdate's Joseph-form covariance update was implemented with dense
// 15x15 matrix products (2 x 15^3 MACs) even though the baro measurement is
// scalar with a single nonzero H entry (H[2] = -1) — making K*H rank-1 and the
// whole update expressible in three outer products (~700 MACs).  On target the
// dense form cost ~1.1 ms per fusion; with #450 restoring every-sample fusion
// that tripled per-tick EKF cost (554 -> ~1650 us) and cut the loop 959 -> 715/s.
//
// The rewrite reassociates float arithmetic, so validation is tolerance-based:
// (1) formula equivalence — dense vs rank-1 on randomized symmetric-PSD
//     matrices, elementwise;
// (2) an end-to-end golden trajectory captured from the dense build.

namespace joseph_ref {

// Dense Joseph form exactly as previously implemented in baroMeasUpdate.
static void dense(float P[15][15], const float K[15], float R)
{
    float I_KH[15][15];
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++)
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * (j == 2 ? -1.0f : 0.0f);

    float P_new[15][15] = {};
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++)
            for (int k = 0; k < 15; k++)
                P_new[i][j] += I_KH[i][k] * P[k][j];

    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++) {
            float sum = 0;
            for (int k = 0; k < 15; k++) sum += P_new[i][k] * I_KH[j][k];
            P[i][j] = sum + K[i] * R * K[j];
        }
}

// Rank-1 form as now implemented in baroMeasUpdate (H[2] = -1):
//   A  = (I-KH)P      -> A[i][j] = P[i][j] + K[i]*P[2][j]
//   B  = A(I-KH)^T    -> B[i][j] = A[i][j] + A[i][2]*K[j]
//   P' = B + R*K*K^T
static void rank1(float P[15][15], const float K[15], float R)
{
    float Prow2[15];
    for (int j = 0; j < 15; j++) Prow2[j] = P[2][j];
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++)
            P[i][j] += K[i] * Prow2[j];

    float Acol2[15];
    for (int i = 0; i < 15; i++) Acol2[i] = P[i][2];
    for (int i = 0; i < 15; i++) {
        const float KR_i = K[i] * R;
        for (int j = 0; j < 15; j++)
            P[i][j] += Acol2[i] * K[j] + KR_i * K[j];
    }
}

// Deterministic LCG so the matrices are identical on every run/platform.
static uint32_t lcg_state = 0x1234567u;
static float frand()
{
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return (float)(lcg_state >> 8) / (float)(1u << 24);  // [0,1)
}

}  // namespace joseph_ref

TEST_F(EKFTest, Rank1JosephMatchesDenseJoseph) {
    using namespace joseph_ref;

    float max_rel = 0.0f;
    for (int trial = 0; trial < 200; ++trial) {
        // Random symmetric PSD P = M*M^T with entries scaled to spread
        // magnitudes across the state kinds (pos/vel/att/bias covariances
        // live at very different scales in flight).
        float M[15][15];
        for (int i = 0; i < 15; i++) {
            const float scale = (i < 3) ? 3.0f : (i < 6) ? 1.0f
                              : (i < 9) ? 0.3f : 0.03f;
            for (int j = 0; j < 15; j++)
                M[i][j] = scale * (frand() - 0.5f);
        }
        float P[15][15] = {};
        for (int i = 0; i < 15; i++)
            for (int j = 0; j < 15; j++)
                for (int k = 0; k < 15; k++)
                    P[i][j] += M[i][k] * M[j][k];

        // Optimal-gain K for a random measurement noise (plus a few trials
        // with deliberately suboptimal K — Joseph must tolerate any K).
        const float R = 0.01f + 10.0f * frand();
        const float S = P[2][2] + R;
        float K[15];
        for (int i = 0; i < 15; i++) {
            K[i] = -P[i][2] / S;
            if (trial % 5 == 4) K[i] *= 1.3f;  // suboptimal-gain trials
        }

        float Pd[15][15], Pr[15][15];
        memcpy(Pd, P, sizeof(P));
        memcpy(Pr, P, sizeof(P));
        dense(Pd, K, R);
        rank1(Pr, K, R);

        for (int i = 0; i < 15; i++)
            for (int j = 0; j < 15; j++) {
                const float denom = std::max(1e-6f, std::fabs(Pd[i][j]));
                const float rel = std::fabs(Pd[i][j] - Pr[i][j]) / denom;
                if (rel > max_rel) max_rel = rel;
            }
    }
    // Same algebra, different association: agreement to float roundoff.
    EXPECT_LT(max_rel, 5e-4f) << "dense vs rank-1 Joseph diverged";
}

// End-to-end golden: a deterministic 20 s dynamic scenario (sinusoidal thrust
// wobble, slow roll, oscillating baro altitude) captured on the dense-Joseph
// build.  The rank-1 build must reproduce the trajectory within float-
// reassociation tolerance.  Guards the integration (K computation, state
// correction, symmetrize/stabilize interplay), not just the formula.
TEST_F(EKFTest, BaroJosephGoldenTrajectory) {
    ekf.init(makeStationaryIMU(0), makeStationaryGNSS(0), makeStationaryMag(0));

    uint32_t t = 0;
    for (int i = 1; i <= 10000; i++) {   // 20 s at 500 Hz
        t += 2000;
        EkfIMUData imu = makeStationaryIMU(t);
        imu.acc_x = 0.3f * std::sin(0.004 * i);
        imu.acc_y = 0.2f * std::cos(0.003 * i);
        imu.acc_z = 9.807 + 0.5 * std::sin(0.002 * i);
        imu.gyro_z = 3.0f * std::sin(0.001 * i);   // slow yaw wobble, dps
        EkfGNSSDataLLA gnss = makeStationaryGNSS(t);
        gnss.alt_m = ALT_M + 1.5 * std::sin(0.0005 * i);
        ekf.update(true, imu, gnss, makeStationaryMag(t));

        if (i % 10 == 0) {               // 50 Hz baro (post-throttle rate)
            EkfBaroData baro;
            baro.time_us = t;
            baro.altitude_m = ALT_M + 2.0 * std::sin(0.0004 * i);
            ekf.baroMeasUpdate(baro);
        }
    }

    double pos[3]; float vel[3], q[4], cov[15];
    ekf.getPosEst(pos);
    ekf.getVelEst(vel);
    ekf.getQuaternion(q);
    ekf.getCovDiag(cov);

    // Goldens captured from the dense-Joseph build (pre-rank-1, commit
    // 6764977).  Tolerances cover float reassociation across 1000 fusions:
    // ~1e-8 rad lat/lon (~6 cm), mm-scale altitude, mm/s velocity.
    EXPECT_NEAR(pos[0],  0.588175958, 1e-8);
    EXPECT_NEAR(pos[1], -2.066469834, 1e-8);
    EXPECT_NEAR(pos[2], 100.287896,   5e-3);
    EXPECT_NEAR(vel[0], 0.024522f, 2e-3f);
    EXPECT_NEAR(vel[1], 0.000624f, 2e-3f);
    EXPECT_NEAR(vel[2], 0.009548f, 2e-3f);
    // Attitude golden RE-BASELINED for #508 (was {0.9247040, 0.0403636,
    // 0.3780217, -0.0198218}). This fixture is magnetically INCONSISTENT: it
    // yaws the gyro at 3 dps while feeding a CONSTANT magnetometer, so the field
    // does not follow the rotation. That is exactly the inconsistency #508's
    // gyro-bias observability gate exists to catch, and it fires 18 times over
    // the 20 s run — correctly refusing to learn gyro bias from a magnetometer
    // that is lying about the vehicle's rotation.
    //
    // This is an improvement, not a regression: the fixture's TRUE gyro bias is
    // zero, and the estimate went from 0.4985 dps (pre-#508) to 0.3870 dps. The
    // position, velocity and covariance goldens above are unchanged and still
    // pass — only attitude moved, because we stopped laundering the bad heading
    // into the bias. The test keeps its original job: guarding the rank-1 Joseph
    // integration against float-reassociation drift.
    const float q_gold[4] = {0.9235595f, 0.0412485f, 0.3807520f, -0.0190893f};
    for (int i = 0; i < 4; i++)
        EXPECT_NEAR(q[i], q_gold[i], 2e-4f) << "q[" << i << "]";
    // The gyro-bias block (12-14) is RE-BASELINED for #508 and is now slightly
    // LARGER (was 1.867444e-07, 2.240813e-07, 1.721429e-07). That is the gate
    // working as designed, not drift: zeroing rows 12-14 of K means the Joseph
    // update does not shrink the bias covariance on a gated sample, so the filter
    // correctly reports that it learned less about the bias from a magnetometer
    // that wasn't following the vehicle's yaw. Every other block (pos/vel/att/
    // accel-bias) is unchanged within the existing 2% tolerance.
    const float cov_gold[15] = {
        1.866526e-03f, 1.866530e-03f, 2.964469e-03f,
        2.649618e-03f, 2.650011e-03f, 2.643721e-03f,
        3.068467e-06f, 4.648058e-06f, 2.927017e-06f,
        7.511995e-04f, 1.435934e-03f, 6.463774e-04f,
        1.947543e-07f, 2.267617e-07f, 1.819631e-07f};
    for (int i = 0; i < 15; i++)
        EXPECT_NEAR(cov[i], cov_gold[i], 0.02f * cov_gold[i] + 1e-9f)
            << "cov[" << i << "]";
}

// #459: each unique mag sample (time_us) is fused exactly once. Re-presenting
// the held sample on subsequent EKF ticks — which is what the FC does between
// real ~98 Hz IIS2MDC samples while updateCore runs at ~480 Hz — must be a
// bitwise no-op, identical to running those ticks with no mag at all. Before
// the freshness gate, the held sample was re-fused every tick, contracting
// yaw covariance ~5x faster than the sensor's real information rate.
TEST_F(EKFTest, MagSampleFusedOncePerUniqueTimestamp) {
    GpsInsEKF a, b;
    a.init(makeNoseUpIMU(0), makeStationaryGNSS(0), makeNoseUpMag(0));
    b.init(makeNoseUpIMU(0), makeStationaryGNSS(0), makeNoseUpMag(0));

    const EkfMagData no_mag = {};   // zero field → mag_valid=false, fusion skipped

    uint32_t last_fresh_t = 0;
    for (int k = 1; k <= 500; k++) {
        const uint32_t t = 2000u * k;               // 500 Hz ticks
        EkfMagData mag_a, mag_b;
        if ((k - 1) % 5 == 0) {                     // fresh sample at ~100 Hz
            last_fresh_t = t;
            mag_a = makeNoseUpMag(t);
            mag_b = makeNoseUpMag(t);
        } else {
            mag_a = makeNoseUpMag(last_fresh_t);    // held sample, stale time_us
            mag_b = no_mag;                         // nothing new arrived
        }
        a.update(true, makeNoseUpIMU(t), makeStationaryGNSS(t), mag_a);
        b.update(true, makeNoseUpIMU(t), makeStationaryGNSS(t), mag_b);
    }

    float qa[4], qb[4], ca[3], cb[3];
    a.getQuaternion(qa); b.getQuaternion(qb);
    a.getCovOrient(ca);  b.getCovOrient(cb);
    for (int i = 0; i < 4; i++) EXPECT_EQ(qa[i], qb[i]) << "quat[" << i << "]";
    for (int i = 0; i < 3; i++) EXPECT_EQ(ca[i], cb[i]) << "covOrient[" << i << "]";
}

// ====================================================================
// #508 — gyro-bias observability gate + physical bound
//
// The bench failure: a flat board makes rocket-frame gravity jump ~81° the
// instant sim data replaces real data. accel/mag had no consistency test, so
// the Kalman gain laundered that inconsistency into gyro bias via the
// attitude↔bias cross-covariance — -190 dps, ~100x beyond anything physical.
// Launch then gates accel/mag off, freezing the bias in for the whole ascent.
//
// The same laundering happens on a REAL pad from a bump, wind sway, a steel
// rail deflecting the mag, or a stale hard-iron cal — the sim just made it
// extreme enough to see.
// ====================================================================

// Nose-up fixture with gravity rotated by `off_deg` away from the nose — i.e.
// the board is not resting where the filter thinks it is.
static EkfIMUData makeTiltedIMU(uint32_t time_us, float off_deg) {
    EkfIMUData imu;
    imu.time_us = time_us;
    const float r = off_deg * (float)M_PI / 180.0f;
    imu.acc_x = 9.807 * std::cos(r);      // along nose
    imu.acc_y = 0.0;
    imu.acc_z = 9.807 * std::sin(r);      // leaked perpendicular
    imu.gyro_x = 0.0; imu.gyro_y = 0.0; imu.gyro_z = 0.0;
    return imu;
}

static void settleNoseUp(GpsInsEKF& ekf, uint32_t& t) {
    ekf.init(makeNoseUpIMU(t), makeStationaryGNSS(t), makeNoseUpMag(t));
    for (int i = 0; i < 2000; i++) {
        t += 2500;   // 400 Hz
        ekf.update(true, makeNoseUpIMU(t), makeStationaryGNSS(t), makeNoseUpMag(t));
    }
}

TEST(EkfGyroBias508, ConvergedPadBiasIsSmallAndHealthy) {
    // Baseline: a consistent stationary pad must still converge to ~zero bias,
    // and must report healthy. If this regresses, the gate is too tight.
    GpsInsEKF ekf;
    uint32_t t = 0;
    settleNoseUp(ekf, t);

    EXPECT_LT(ekf.gyroBiasMaxDps(), 1.0f);
    EXPECT_EQ(ekf.gyroBiasClipCount(), 0u);
    EXPECT_TRUE(ekf.gyroBiasHealthy());
}

TEST(EkfGyroBias508, FrameStepNoLongerDivergesTheBias) {
    // Reproduce the bench event: converge on a consistent pad, then hand the
    // filter a gravity vector 81° away with the gyro still reading zero — the
    // real→sim handover. Before #508 this drove the bias to -190 dps.
    GpsInsEKF ekf;
    uint32_t t = 0;
    settleNoseUp(ekf, t);
    ASSERT_LT(ekf.gyroBiasMaxDps(), 1.0f);

    for (int i = 0; i < 2000; i++) {           // 5 s of the inconsistent frame
        t += 2500;
        ekf.update(true, makeTiltedIMU(t, 81.0f), makeStationaryGNSS(t), makeNoseUpMag(t));
    }

    // The gate must have fired, and the bias must stay essentially uncorrupted.
    // Unfixed, this scenario drove it to -190 dps; the LATCHING gate holds the
    // coupling cut through the whole attitude slew, so almost nothing leaks in.
    EXPECT_GT(ekf.gyroBiasGateTrips(), 0u);
    EXPECT_LT(ekf.gyroBiasMaxDps(), 1.0f) << "inconsistency still laundered into the bias";
}

TEST(EkfGyroBias508, GateTripsAreVisibleToThePreLaunchCheck) {
    // Even when the bias survives intact, a tripping gate means the filter is
    // being fed accel/mag data inconsistent with its state — a bad mounting, a
    // steel rail deflecting the mag, a stale hard-iron cal. The scorecard must be
    // able to see that, because launch freezes whatever the bias holds.
    GpsInsEKF ekf;
    uint32_t t = 0;
    settleNoseUp(ekf, t);
    ASSERT_EQ(ekf.gyroBiasGateTrips(), 0u);    // clean pad trips nothing

    for (int i = 0; i < 2000; i++) {
        t += 2500;
        ekf.update(true, makeTiltedIMU(t, 81.0f), makeStationaryGNSS(t), makeNoseUpMag(t));
    }
    EXPECT_GT(ekf.gyroBiasGateTrips(), 0u);
}

TEST(EkfGyroBias508, GenuineBiasIsStillLearnedAndOutOfSpecIsFlagged) {
    // The gate must not blind the filter to a REAL gyro bias — that would be a
    // bad trade. A gyro with a true 8 dps offset must still be learned exactly
    // (so attitude is compensated), with no gate trips, and reported out-of-spec
    // so a suspect sensor is caught before flight.
    GpsInsEKF ekf;
    uint32_t t = 0;
    EkfIMUData imu = makeNoseUpIMU(t);
    imu.gyro_x = 8.0;                          // real, consistent 8 dps offset
    ekf.init(imu, makeStationaryGNSS(t), makeNoseUpMag(t));
    for (int i = 0; i < 4000; i++) {
        t += 2500;
        EkfIMUData u = makeNoseUpIMU(t);
        u.gyro_x = 8.0;
        ekf.update(true, u, makeStationaryGNSS(t), makeNoseUpMag(t));
    }

    EXPECT_NEAR(ekf.gyroBiasMaxDps(), 8.0f, 0.5f);   // learned, not suppressed
    EXPECT_EQ(ekf.gyroBiasGateTrips(), 0u);          // consistent data — no trips
    EXPECT_FALSE(ekf.gyroBiasHealthy());             // but out of spec → no-go
}

TEST(EkfGyroBias508, AttitudeStillCorrectsThroughTheGate) {
    // The gate must cut the gyro-bias coupling ONLY — attitude has to keep
    // correcting, or genuinely re-orienting the rocket on the rail would leave
    // the filter stuck on a stale attitude. This is the anti-regression test.
    GpsInsEKF ekf;
    uint32_t t = 0;
    settleNoseUp(ekf, t);

    // Physically re-orient by 30° and hold. The filter must follow the gravity
    // vector to the new attitude.
    for (int i = 0; i < 4000; i++) {
        t += 2500;
        ekf.update(true, makeTiltedIMU(t, 30.0f), makeStationaryGNSS(t), makeNoseUpMag(t));
    }

    // Body-frame gravity should now line up with the measurement: the estimated
    // gravity direction must have swung ~30° off the nose.
    float q[4];
    ekf.getQuaternion(q);
    // Rotate NED-down into body: g_body ∝ third column of T_NED2B.
    const float gx = 2.0f * (q[1]*q[3] - q[0]*q[2]);
    const float gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    const float off = std::atan2(std::fabs(gz), std::fabs(gx)) * 180.0f / (float)M_PI;
    EXPECT_NEAR(off, 30.0f, 8.0f) << "attitude failed to track the re-orientation";
}

TEST(EkfGyroBias508, BoundIsABackstopOnAnyPath) {
    // Even init is a path: initCore seeds the bias from the raw gyro reading
    // ("assume stationary"), so initialising while the vehicle is being handled
    // bakes that rate straight in. It must be clamped, not trusted.
    GpsInsEKF ekf;
    EkfIMUData imu = makeNoseUpIMU(0);
    imu.gyro_x = 250.0;   // deg/s — being waved around at init
    imu.gyro_y = -300.0;
    ekf.init(imu, makeStationaryGNSS(0), makeNoseUpMag(0));

    EXPECT_LE(ekf.gyroBiasMaxDps(), 10.1f);    // clamped to the envelope
    EXPECT_GT(ekf.gyroBiasClipCount(), 0u);    // and it says the bound bit
    EXPECT_FALSE(ekf.gyroBiasHealthy());       // so this is a no-go
}

TEST(EkfGyroBias508, GateDoesNotFireOnNormalPadNoise) {
    // A small, legitimate tilt (5°) must pass the gate untouched — otherwise the
    // filter would never learn a real bias. NIS ≈ 3 here vs the gate at 25.
    GpsInsEKF ekf;
    uint32_t t = 0;
    settleNoseUp(ekf, t);
    const uint32_t trips_before = ekf.gyroBiasGateTrips();

    for (int i = 0; i < 400; i++) {
        t += 2500;
        ekf.update(true, makeTiltedIMU(t, 5.0f), makeStationaryGNSS(t), makeNoseUpMag(t));
    }
    EXPECT_EQ(ekf.gyroBiasGateTrips(), trips_before) << "gate is too tight";
}
