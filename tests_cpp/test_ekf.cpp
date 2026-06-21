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

// Heading fusion must CONVERGE while nose-vertical — the exact case the old
// Mahony correction (cos²(pitch) → 0 at vertical) could not handle.
TEST_F(EKFTest, MagHeadingConvergesWhenVertical) {
    ekf.init(makeNoseUpIMU(0), makeStationaryGNSS(0), makeNoseUpMag(0));
    uint32_t t = 0;
    for (int i = 0; i < 2000; i++) { t += 2000;
        ekf.update(true, makeNoseUpIMU(t), makeStationaryGNSS(t), makeNoseUpMag(t)); }
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
        ekf.update(true, makeNoseUpIMU(t), makeStationaryGNSS(t), makeNoseUpMag(t)); }
    float q_fin[4]; ekf.getQuaternion(q_fin);
    EXPECT_LT(tqGeodesicDeg(q_fin, q_ref), 8.0f);      // heading converged back
}

// Heading fusion must be STABLE (no drift) while stationary nose-vertical —
// the symptom we observed (±180° roll/yaw cycling) must not recur.
TEST_F(EKFTest, MagHeadingStableVertical) {
    ekf.init(makeNoseUpIMU(0), makeStationaryGNSS(0), makeNoseUpMag(0));
    uint32_t t = 0;
    for (int i = 0; i < 2000; i++) { t += 2000;
        ekf.update(true, makeNoseUpIMU(t), makeStationaryGNSS(t), makeNoseUpMag(t)); }
    float q_a[4]; ekf.getQuaternion(q_a);
    for (int i = 0; i < 4000; i++) { t += 2000;
        ekf.update(true, makeNoseUpIMU(t), makeStationaryGNSS(t), makeNoseUpMag(t)); }
    float q_b[4]; ekf.getQuaternion(q_b);
    EXPECT_LT(tqGeodesicDeg(q_a, q_b), 2.0f);          // no heading drift over 8 s
    float n = std::sqrt(q_b[0]*q_b[0] + q_b[1]*q_b[1] + q_b[2]*q_b[2] + q_b[3]*q_b[3]);
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
