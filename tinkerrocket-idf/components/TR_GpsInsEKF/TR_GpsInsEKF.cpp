// GPS/INS Fusion EKF — shared library for flight computer and simulation.
//
// Refactored: Mahony AHRS removed. EKF handles quaternion propagation,
// accelerometer gravity reference, and magnetometer heading correction.

#include "TR_GpsInsEKF.h"

GpsInsEKF::GpsInsEKF() {
    // (The GNSS observation matrix H = [I6 | 0] is exploited structurally in
    // measUpdate rather than stored and multiplied densely.)

    // Process noise Rw — 12x12
    std::memset(Rw_, 0, sizeof(Rw_));
    Rw_[0][0]=aNoiseSigma_mps2*aNoiseSigma_mps2;
    Rw_[1][1]=aNoiseSigma_mps2*aNoiseSigma_mps2;
    Rw_[2][2]=aNoiseSigma_mps2*aNoiseSigma_mps2;
    Rw_[3][3]=wNoiseSigma_rps*wNoiseSigma_rps;
    Rw_[4][4]=wNoiseSigma_rps*wNoiseSigma_rps;
    Rw_[5][5]=wNoiseSigma_rps*wNoiseSigma_rps;
    Rw_[6][6]=2.0f*(aMarkovSigma_mps2*aMarkovSigma_mps2)/aMarkovTau_s;
    Rw_[7][7]=2.0f*(aMarkovSigma_mps2*aMarkovSigma_mps2)/aMarkovTau_s;
    Rw_[8][8]=2.0f*(aMarkovSigma_mps2*aMarkovSigma_mps2)/aMarkovTau_s;
    Rw_[9][9]=2.0f*(wMarkovSigma_rps*wMarkovSigma_rps)/wMarkovTau_s;
    Rw_[10][10]=2.0f*(wMarkovSigma_rps*wMarkovSigma_rps)/wMarkovTau_s;
    Rw_[11][11]=2.0f*(wMarkovSigma_rps*wMarkovSigma_rps)/wMarkovTau_s;

    // Observation noise R — 6x6
    std::memset(R_, 0, sizeof(R_));
    R_[0][0]=pNoiseSigma_NE_m*pNoiseSigma_NE_m;
    R_[1][1]=pNoiseSigma_NE_m*pNoiseSigma_NE_m;
    R_[2][2]=pNoiseSigma_D_m*pNoiseSigma_D_m;
    R_[3][3]=vNoiseSigma_NE_mps*vNoiseSigma_NE_mps;
    R_[4][4]=vNoiseSigma_NE_mps*vNoiseSigma_NE_mps;
    R_[5][5]=vNoiseSigma_D_mps*vNoiseSigma_D_mps;

    // Initial covariance P — 15x15
    std::memset(P_, 0, sizeof(P_));
    P_[0][0]=pErrSigma_Init_m*pErrSigma_Init_m;
    P_[1][1]=pErrSigma_Init_m*pErrSigma_Init_m;
    P_[2][2]=pErrSigma_Init_m*pErrSigma_Init_m;
    P_[3][3]=vErrSigma_Init_mps*vErrSigma_Init_mps;
    P_[4][4]=vErrSigma_Init_mps*vErrSigma_Init_mps;
    P_[5][5]=vErrSigma_Init_mps*vErrSigma_Init_mps;
    P_[6][6]=attErrSigma_Init_rad*attErrSigma_Init_rad;
    P_[7][7]=attErrSigma_Init_rad*attErrSigma_Init_rad;
    P_[8][8]=hdgErrSigma_Init_rad*hdgErrSigma_Init_rad;
    P_[9][9]=aBiasSigma_Init_mps2*aBiasSigma_Init_mps2;
    P_[10][10]=aBiasSigma_Init_mps2*aBiasSigma_Init_mps2;
    P_[11][11]=aBiasSigma_Init_mps2*aBiasSigma_Init_mps2;
    P_[12][12]=wBiasSigma_Init_rps*wBiasSigma_Init_rps;
    P_[13][13]=wBiasSigma_Init_rps*wBiasSigma_Init_rps;
    P_[14][14]=wBiasSigma_Init_rps*wBiasSigma_Init_rps;

    // Gs — 15x12
    std::memset(Gs_, 0, sizeof(Gs_));
    Gs_[6][3]=-0.5f; Gs_[7][4]=-0.5f; Gs_[8][5]=-0.5f;
    Gs_[9][6]=1; Gs_[10][7]=1; Gs_[11][8]=1;
    Gs_[12][9]=1; Gs_[13][10]=1; Gs_[14][11]=1;

    // Fs — 15x15
    std::memset(Fs_, 0, sizeof(Fs_));
    Fs_[0][3]=1; Fs_[1][4]=1; Fs_[2][5]=1;
    Fs_[5][2]=-2.0f*G/EARTH_RADIUS;
    Fs_[6][12]=-0.5f; Fs_[7][13]=-0.5f; Fs_[8][14]=-0.5f;
    Fs_[9][9]=-1.0f/aMarkovTau_s;
    Fs_[10][10]=-1.0f/aMarkovTau_s;
    Fs_[11][11]=-1.0f/aMarkovTau_s;
    Fs_[12][12]=-1.0f/wMarkovTau_s;
    Fs_[13][13]=-1.0f/wMarkovTau_s;
    Fs_[14][14]=-1.0f/wMarkovTau_s;

    // Zero state vectors
    std::memset(aBias_mps2_, 0, sizeof(aBias_mps2_));
    std::memset(wBias_rps_, 0, sizeof(wBias_rps_));
    pEst_D_rrm_[0]=0; pEst_D_rrm_[1]=0; pEst_D_rrm_[2]=0;
    std::memset(vEst_NED_mps_, 0, sizeof(vEst_NED_mps_));
    std::memset(aEst_B_mps2_, 0, sizeof(aEst_B_mps2_));
    std::memset(wEst_B_rps_, 0, sizeof(wEst_B_rps_));
    std::memset(quat_BL_, 0, sizeof(quat_BL_));
    std::memset(x, 0, sizeof(x));

    // Initial quaternion: nose-up vertical rocket (FRD body frame)
    // Body X (nose) → NED Up (-D), Body Y (right) → East, Body Z (down) → North.
    // +90° rotation about Y: q = [cos(45°), 0, sin(45°), 0]
    quat_BL_[0] = 0.707107f; quat_BL_[1] = 0.0f;
    quat_BL_[2] = 0.707107f; quat_BL_[3] = 0.0f;
}

// ─── Init: shared core ──────────────────────────────────────────────

void GpsInsEKF::initCore(EkfIMUData imu_data,
                         EkfMagData mag_data,
                         double pMeas_D_rrm[3],
                         float vMeas_NED[3],
                         uint32_t gnss_time_us) {
    tPrev_us_ = imu_data.time_us;
    timeWeekPrev_ = gnss_time_us;

    float wMeas[3] = {(float)(imu_data.gyro_x*DEG2RAD),
                      (float)(imu_data.gyro_y*DEG2RAD),
                      (float)(imu_data.gyro_z*DEG2RAD)};

    pEst_D_rrm_[0]=pMeas_D_rrm[0]; pEst_D_rrm_[1]=pMeas_D_rrm[1]; pEst_D_rrm_[2]=pMeas_D_rrm[2];
    vEst_NED_mps_[0]=vMeas_NED[0]; vEst_NED_mps_[1]=vMeas_NED[1]; vEst_NED_mps_[2]=vMeas_NED[2];

    // Coarse "assume stationary" seed: whatever the gyro reads at init IS the
    // bias. #508: that is unbounded — init while the vehicle is being handled or
    // rotated and the rate is baked straight into the bias state. Clamp it into
    // the physical envelope, and start the health counters clean.
    gbias_gate_trips_ = 0;
    gbias_clip_count_ = 0;
    gbias_gate_hold_until_us_ = 0;
    wBias_rps_[0]=wMeas[0]; wBias_rps_[1]=wMeas[1]; wBias_rps_[2]=wMeas[2];
    clampGyroBias();

    // Initial quaternion: nose-up vertical (same as constructor)
    quat_BL_[0] = 0.707107f; quat_BL_[1] = 0.0f;
    quat_BL_[2] = 0.707107f; quat_BL_[3] = 0.0f;

    float T_NED2B[3][3];
    Quat2DCM(T_NED2B, quat_BL_);
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) T_B2NED[j][i]=T_NED2B[i][j];
}

// ─── Init: ECEF path (simulation) ──────────────────────────────────

void GpsInsEKF::init(EkfIMUData imu_data, EkfGNSSData gnss_data, EkfMagData mag_data) {
    double pMeas_D_rrm[3];
    ecefToLla(gnss_data.ecef_x, gnss_data.ecef_y, gnss_data.ecef_z,
              pMeas_D_rrm[0], pMeas_D_rrm[1], pMeas_D_rrm[2]);
    pMeas_D_rrm[0] *= DEG2RAD;
    pMeas_D_rrm[1] *= DEG2RAD;

    double ve, vn, vu;
    ecefVelToEnu(gnss_data.ecef_vx, gnss_data.ecef_vy, gnss_data.ecef_vz,
                 pMeas_D_rrm[0]*RAD2DEG, pMeas_D_rrm[1]*RAD2DEG, ve, vn, vu);
    float vMeas_NED[3] = {(float)vn, (float)ve, (float)-vu};

    initCore(imu_data, mag_data, pMeas_D_rrm, vMeas_NED, gnss_data.time_us);
}

// ─── Init: LLA path (flight computer) ──────────────────────────────

void GpsInsEKF::init(EkfIMUData imu_data, EkfGNSSDataLLA gnss_data, EkfMagData mag_data) {
    double pMeas_D_rrm[3] = {gnss_data.lat_rad, gnss_data.lon_rad, gnss_data.alt_m};
    float vMeas_NED[3] = {gnss_data.vel_n_mps, gnss_data.vel_e_mps, gnss_data.vel_d_mps};

    initCore(imu_data, mag_data, pMeas_D_rrm, vMeas_NED, gnss_data.time_us);
}

// ─── Update: shared core ────────────────────────────────────────────

void GpsInsEKF::updateCore(bool use_ahrs_acc,
                           EkfIMUData imu_data,
                           EkfMagData mag_data,
                           double pMeas_D_rrm[3],
                           float vMeas_NED[3],
                           uint32_t gnss_time_us) {
    // 1. Compute dt
    uint32_t t_us = imu_data.time_us;
    // #440: a repeated/frozen IMU timestamp means no new time has provably
    // elapsed — a stalled sensor, a wedged I2C read, or a replayed frame.
    // The floor below used to rewrite dt=0 to 2 ms, re-integrating the SAME
    // sample as if time had passed: velocity/attitude drifted while
    // isHealthy() stayed true, feeding control a confidently wrong state.
    // Skip the tick entirely — the filter HOLDS its state through a stall
    // (the caller's raw-gyro fallbacks own control when the IMU is stale,
    // #262/#270). The floor below still handles small-but-real dts.
    if (t_us == tPrev_us_) {
        ++frozen_dt_skips_;
        return;
    }
    dt_s_ = ((float)(t_us - tPrev_us_)) / 1e6f;
    tPrev_us_ = t_us;
    if (dt_s_ > 0.1f) dt_s_ = 0.1f;
    if (dt_s_ < 0.0005f) dt_s_ = 0.002f;  // Floor at 0.5ms; use 2ms default (500 Hz)

    // 2. Convert sensor data
    float wMeas[3] = {(float)(imu_data.gyro_x*DEG2RAD),
                      (float)(imu_data.gyro_y*DEG2RAD),
                      (float)(imu_data.gyro_z*DEG2RAD)};
    float aMeas[3] = {(float)imu_data.acc_x, (float)imu_data.acc_y, (float)imu_data.acc_z};
    float magMeas[3] = {(float)mag_data.mag_x, (float)mag_data.mag_y, (float)mag_data.mag_z};

    // 3. Magnetometer sanity check (15–80 µT range)
    bool mag_valid = true;
    {
        float mag_sq = magMeas[0]*magMeas[0] + magMeas[1]*magMeas[1] + magMeas[2]*magMeas[2];
        if (mag_sq < 225.0f || mag_sq > 6400.0f) {
            mag_valid = false;
        }
    }

    // 4. Accelerometer validity check (0.5g–1.5g)
    bool accel_valid = use_ahrs_acc;
    if (accel_valid) {
        float sf_sq = aMeas[0]*aMeas[0] + aMeas[1]*aMeas[1] + aMeas[2]*aMeas[2];
        float g = 9.807f;
        if (sf_sq < (0.5f*g)*(0.5f*g) || sf_sq > (1.5f*g)*(1.5f*g)) {
            accel_valid = false;
        }
    }

    // 5. Bias-corrected gyro rate (needed for quaternion propagation in timeUpdate)
    for (int i=0;i<3;i++) wEst_B_rps_[i] = wMeas[i] - wBias_rps_[i];

    // 6. Compute gravity in body frame and accel estimate using current quaternion
    {
        float T_NED2B_local[3][3];
        Quat2DCM(T_NED2B_local, quat_BL_);
        float aGrav_B[3] = {T_NED2B_local[0][2]*G, T_NED2B_local[1][2]*G, T_NED2B_local[2][2]*G};
        for (int i=0;i<3;i++) aEst_B_mps2_[i] = aMeas[i] + aGrav_B[i] - aBias_mps2_[i];
    }

    // 7. Time update (quaternion propagation + velocity/position/covariance prediction)
    timeUpdate();

    // 8. Accelerometer gravity reference update
    if (accel_valid) accelMeasUpdate(aMeas);

    // 9. Magnetometer heading update — scalar Kalman, yaw-only (needs valid
    //    accel for the gravity/level reference; accel owns roll/pitch, mag owns
    //    heading, so the two are complementary, not competing).
    //    #459: fuse each mag sample exactly once (same time_us dedup as
    //    baroMeasUpdate) — the mag delivers ~98 Hz while this runs every EKF
    //    tick (~480 Hz), and re-fusing one sample N times contracts yaw
    //    covariance N-fold faster than the sensor's real information rate.
    if (mag_valid && accel_valid && mag_data.time_us != magTimePrev_) {
        magTimePrev_ = mag_data.time_us;
        magMeasUpdate(aMeas, magMeas);
    }

    // 10. GNSS measurement update
    if ((gnss_time_us - timeWeekPrev_) > 0) {
        timeWeekPrev_ = gnss_time_us;
        measUpdate(pMeas_D_rrm, vMeas_NED);
        // 10b. GNSS course-over-ground heading aiding — only with enough
        //      horizontal speed for a well-defined course (else it's noise).
        const float vh_sq = vMeas_NED[0]*vMeas_NED[0] + vMeas_NED[1]*vMeas_NED[1];
        if (vh_sq > (3.0f*3.0f)) velCourseHeadingUpdate(vMeas_NED);

        // 10c. Accel-match heading aiding — differentiate the GNSS velocity to
        //      a world horizontal acceleration (low-passed), then match it to
        //      the body lateral specific force to observe the roll DOF.
        if (haveGnssAccel_ && prevGnssSampleUs_ != 0) {
            const float dtg = (float)(t_us - prevGnssSampleUs_) * 1e-6f;
            if (dtg > 0.005f && dtg < 0.5f) {
                const float aN_raw = (vMeas_NED[0] - prevGnssVel_NED_[0]) / dtg;
                const float aE_raw = (vMeas_NED[1] - prevGnssVel_NED_[1]) / dtg;
                const float lp = 0.3f;
                gnssAccelLP_NE_[0] += lp * (aN_raw - gnssAccelLP_NE_[0]);
                gnssAccelLP_NE_[1] += lp * (aE_raw - gnssAccelLP_NE_[1]);
                accelMatchHeadingUpdate(aMeas, gnssAccelLP_NE_);
            }
        }
        prevGnssVel_NED_[0] = vMeas_NED[0];
        prevGnssVel_NED_[1] = vMeas_NED[1];
        prevGnssVel_NED_[2] = vMeas_NED[2];
        prevGnssSampleUs_   = t_us;
        haveGnssAccel_      = true;
    }

    // 11. Recompute estimates with post-update quaternion and biases
    {
        float T_NED2B_local[3][3];
        Quat2DCM(T_NED2B_local, quat_BL_);
        for (int i=0;i<3;i++) for (int j=0;j<3;j++) T_B2NED[j][i] = T_NED2B_local[i][j];
        float aGrav_B[3] = {T_NED2B_local[0][2]*G, T_NED2B_local[1][2]*G, T_NED2B_local[2][2]*G};
        for (int i=0;i<3;i++) {
            aEst_B_mps2_[i] = aMeas[i] + aGrav_B[i] - aBias_mps2_[i];
            wEst_B_rps_[i] = wMeas[i] - wBias_rps_[i];
        }
    }

    // 12. Update Euler angles
    Quat2Euler(quat_BL_, euler_BL_rad_);
}

// ─── Update: ECEF path (simulation) ────────────────────────────────

void GpsInsEKF::update(bool use_ahrs_acc,
                       EkfIMUData imu_data,
                       EkfGNSSData gnss_data,
                       EkfMagData mag_data) {
    double pMeas_D_rrm[3];
    ecefToLla(gnss_data.ecef_x, gnss_data.ecef_y, gnss_data.ecef_z,
              pMeas_D_rrm[0], pMeas_D_rrm[1], pMeas_D_rrm[2]);
    pMeas_D_rrm[0] *= DEG2RAD;
    pMeas_D_rrm[1] *= DEG2RAD;

    double ve, vn, vu;
    ecefVelToEnu(gnss_data.ecef_vx, gnss_data.ecef_vy, gnss_data.ecef_vz,
                 pMeas_D_rrm[0]*RAD2DEG, pMeas_D_rrm[1]*RAD2DEG, ve, vn, vu);
    float vMeas_NED[3] = {(float)vn, (float)ve, (float)-vu};

    updateCore(use_ahrs_acc, imu_data, mag_data, pMeas_D_rrm, vMeas_NED, gnss_data.time_us);
}

// ─── Update: LLA path (flight computer) ────────────────────────────

void GpsInsEKF::update(bool use_ahrs_acc,
                       EkfIMUData imu_data,
                       EkfGNSSDataLLA gnss_data,
                       EkfMagData mag_data) {
    double pMeas_D_rrm[3] = {gnss_data.lat_rad, gnss_data.lon_rad, gnss_data.alt_m};
    float vMeas_NED[3] = {gnss_data.vel_n_mps, gnss_data.vel_e_mps, gnss_data.vel_d_mps};

    updateCore(use_ahrs_acc, imu_data, mag_data, pMeas_D_rrm, vMeas_NED, gnss_data.time_us);
}

// ─── Time Update (prediction) ───────────────────────────────────────

void GpsInsEKF::timeUpdate() {
    // #257/#265: age out the post-divergence unhealthy window one sample/cycle.
    if (unhealthy_cooldown_ > 0) unhealthy_cooldown_--;

    // Compute DCM from current (pre-propagation) quaternion
    float T_NED2B[3][3];
    Quat2DCM(T_NED2B, quat_BL_);
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) T_B2NED[j][i]=T_NED2B[i][j];

    // Propagate quaternion using bias-corrected gyro (rotation vector / exponential map).
    // Exact for constant angular velocity within the time step — much more accurate
    // than first-order Euler at high rotation rates (e.g. 500 deg/s roll).
    {
        float theta[3] = {dt_s_ * wEst_B_rps_[0],
                          dt_s_ * wEst_B_rps_[1],
                          dt_s_ * wEst_B_rps_[2]};
        float angle = std::sqrt(theta[0]*theta[0] + theta[1]*theta[1] + theta[2]*theta[2]);
        float dq[4];
        if (angle > 1e-8f) {
            float ha = 0.5f * angle;
            float s = std::sin(ha) / angle;
            dq[0] = std::cos(ha);
            dq[1] = s * theta[0];
            dq[2] = s * theta[1];
            dq[3] = s * theta[2];
        } else {
            dq[0] = 1.0f;
            dq[1] = 0.5f * theta[0];
            dq[2] = 0.5f * theta[1];
            dq[3] = 0.5f * theta[2];
        }
        float qTemp[4];
        multiplyQuaternions(quat_BL_, dq, qTemp);
        for (int i = 0; i < 4; i++) quat_BL_[i] = qTemp[i];
        normalizeQuaternion(quat_BL_, quat_BL_);
    }

    // Velocity update (using pre-propagation DCM)
    float aEst_NED[3] = {0,0,0};
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) aEst_NED[i]+=T_B2NED[i][j]*aEst_B_mps2_[j];
    for (int i=0;i<3;i++) vEst_NED_mps_[i]+=aEst_NED[i]*dt_s_;

    // Position update
    double pDot_D[3];
    NED2D_Rate(pDot_D, vEst_NED_mps_, pEst_D_rrm_);
    for (int i=0;i<3;i++) pEst_D_rrm_[i]+=(double)dt_s_*pDot_D[i];

    // ── Assemble Jacobian (dynamic blocks of Fs_) ──
    float skew_temp[3][3], Fs_sub[3][3];
    Skew(skew_temp, aEst_B_mps2_);
    multiplyMatrix3x3(T_B2NED, skew_temp, Fs_sub);
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Fs_[i+3][j+6]=-2.0f*Fs_sub[i][j];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Fs_[i+3][j+9]=-T_B2NED[i][j];
    float Fs_w[3][3];
    Skew(Fs_w, wEst_B_rps_);
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Fs_[i+6][j+6]=-Fs_w[i][j];

    // Update Gs dynamic portion (rows 3-5, cols 0-2)
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Gs_[i+3][j]=-T_B2NED[i][j];

    // ── Sparse Q computation and P propagation ────────────────────────
    // Exploits known zero structure of Fs, Gs, Rw to reduce ~14,500
    // FLOPs (7 dense 15×15 multiplies) to ~1,700 FLOPs.
    // Verified against dense reference: verify_sparse_ekf.py

    const float dt2 = dt_s_ * dt_s_;

    // ── Step 1: B[15][12] = (I + Fs*dt)*dt * Gs * diag(Rw) ─────────
    // Gs non-zero blocks:
    //   rows 3-5, cols 0-2:   Gs_[i+3][j] (dynamic, = -T_B2NED)
    //   rows 6-8, cols 3-5:   -0.5 (fixed)
    //   rows 9-11, cols 6-8:  1.0 (fixed)
    //   rows 12-14, cols 9-11: 1.0 (fixed)
    float B[15][12];
    std::memset(B, 0, sizeof(B));

    // Rw diagonal values (pre-scaled)
    const float rw[12] = {
        Rw_[0][0], Rw_[1][1], Rw_[2][2],
        Rw_[3][3], Rw_[4][4], Rw_[5][5],
        Rw_[6][6], Rw_[7][7], Rw_[8][8],
        Rw_[9][9], Rw_[10][10], Rw_[11][11]
    };

    // Cols 0-2: Gs rows 3-5 have a 3×3 block
    for (int i = 0; i < 15; i++) {
        for (int c = 0; c < 3; c++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                float pdt = dt2 * Fs_[i][k+3];
                if (i == k+3) pdt += dt_s_;
                s += pdt * Gs_[k+3][c];
            }
            B[i][c] = s * rw[c];
        }
    }

    // Cols 3-11: each column has a single non-zero Gs row
    static const int src_row[9] = {6, 7, 8, 9, 10, 11, 12, 13, 14};
    static const float src_val[9] = {-0.5f, -0.5f, -0.5f,
                                      1.0f, 1.0f, 1.0f,
                                      1.0f, 1.0f, 1.0f};
    for (int i = 0; i < 15; i++) {
        for (int cc = 0; cc < 9; cc++) {
            const int c = cc + 3;
            const int k = src_row[cc];
            float pdt = dt2 * Fs_[i][k];
            if (i == k) pdt += dt_s_;
            B[i][c] = pdt * src_val[cc] * rw[c];
        }
    }

    // ── Step 2: Q[15][15] = B * Gs^T ───────────────────────────────
    // Gs^T non-zero blocks (transposed from Gs):
    //   rows 0-2, cols 3-5:   Gs_[j][c] for j=3..5 (dynamic)
    //   row 3→col 6, row 4→col 7, row 5→col 8: -0.5
    //   row 6→col 9, ..., row 8→col 11: 1.0
    //   row 9→col 12, ..., row 11→col 14: 1.0
    float Q[15][15];
    std::memset(Q, 0, sizeof(Q));

    for (int i = 0; i < 15; i++) {
        // j = 3..5: Gs^T[c][j] = Gs_[j][c] for c=0..2
        for (int j = 3; j < 6; j++) {
            float s = 0.0f;
            for (int c = 0; c < 3; c++)
                s += B[i][c] * Gs_[j][c];
            Q[i][j] = s;
        }
        // j = 6..8: single non-zero per column (Gs^T[3..5][6..8] = -0.5)
        Q[i][6] = B[i][3] * (-0.5f);
        Q[i][7] = B[i][4] * (-0.5f);
        Q[i][8] = B[i][5] * (-0.5f);
        // j = 9..14: single non-zero per column (Gs^T[6..11][9..14] = 1.0)
        for (int j = 9; j < 15; j++)
            Q[i][j] = B[i][j - 3];
    }

    // Symmetrize Q
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < i; j++) {
            float s = 0.5f * (Q[i][j] + Q[j][i]);
            Q[i][j] = s; Q[j][i] = s;
        }

    // ── Step 3: P propagation — PHI*P (row pass) ───────────────────
    // PHI = I + Fs*dt.  Instead of full 15×15×15 multiply, add
    // dt * Fs[row] contributions per row, touching only non-zero Fs entries.
    float PHI_P[15][15];
    std::memcpy(PHI_P, P_, sizeof(PHI_P));

    // Rows 0-2: Fs[i][i+3] = 1
    for (int j = 0; j < 15; j++) PHI_P[0][j] += dt_s_ * P_[3][j];
    for (int j = 0; j < 15; j++) PHI_P[1][j] += dt_s_ * P_[4][j];
    for (int j = 0; j < 15; j++) PHI_P[2][j] += dt_s_ * P_[5][j];

    // Rows 3-5: Fs[i+3][6..8] and Fs[i+3][9..11]
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 15; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                s += Fs_[i+3][k+6]  * P_[k+6][j];
                s += Fs_[i+3][k+9]  * P_[k+9][j];
            }
            PHI_P[i+3][j] += dt_s_ * s;
        }
    }

    // Row 5 extra: Fs[5][2]
    {
        const float f52_dt = dt_s_ * Fs_[5][2];
        for (int j = 0; j < 15; j++) PHI_P[5][j] += f52_dt * P_[2][j];
    }

    // Rows 6-8: Fs[i+6][6..8] (skew) and Fs[i+6][i+12] (-0.5)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 15; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += Fs_[i+6][k+6] * P_[k+6][j];
            s += Fs_[i+6][i+12] * P_[i+12][j];
            PHI_P[i+6][j] += dt_s_ * s;
        }
    }

    // Rows 9-14: diagonal only
    for (int i = 9; i < 15; i++) {
        const float f_dt = dt_s_ * Fs_[i][i];
        for (int j = 0; j < 15; j++) PHI_P[i][j] += f_dt * P_[i][j];
    }

    // ── Step 4: Column pass — PHI_P * PHI^T + Q → P ────────────────
    // Same logic transposed: add dt * Fs[col] contributions per column.
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++)
            P_[i][j] = PHI_P[i][j] + Q[i][j];

    // Cols 0-2: Fs[i][i+3] = 1
    for (int i = 0; i < 15; i++) P_[i][0] += dt_s_ * PHI_P[i][3];
    for (int i = 0; i < 15; i++) P_[i][1] += dt_s_ * PHI_P[i][4];
    for (int i = 0; i < 15; i++) P_[i][2] += dt_s_ * PHI_P[i][5];

    // Cols 3-5: Fs[j+3][6..8] and Fs[j+3][9..11]
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 15; i++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                s += PHI_P[i][k+6]  * Fs_[j+3][k+6];
                s += PHI_P[i][k+9]  * Fs_[j+3][k+9];
            }
            P_[i][j+3] += dt_s_ * s;
        }
    }

    // Col 5 extra: Fs[5][2]
    {
        const float f52_dt = dt_s_ * Fs_[5][2];
        for (int i = 0; i < 15; i++) P_[i][5] += f52_dt * PHI_P[i][2];
    }

    // Cols 6-8: Fs[j+6][6..8] (skew) and Fs[j+6][j+12] (-0.5)
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 15; i++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++)
                s += PHI_P[i][k+6] * Fs_[j+6][k+6];
            s += PHI_P[i][j+12] * Fs_[j+6][j+12];
            P_[i][j+6] += dt_s_ * s;
        }
    }

    // Cols 9-14: diagonal only
    for (int j = 9; j < 15; j++) {
        const float f_dt = dt_s_ * Fs_[j][j];
        for (int i = 0; i < 15; i++) P_[i][j] += f_dt * PHI_P[i][j];
    }

    // Symmetrize P
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < i; j++) {
            float s = 0.5f * (P_[i][j] + P_[j][i]);
            P_[i][j] = s; P_[j][i] = s;
        }

    stabilizeP();
}

void GpsInsEKF::stabilizeP() {
    // Floor diagonals at a tiny positive value so float32 drift can't drive
    // them negative or NaN — downstream sqrt(P)/1/P would then propagate NaN
    // through the whole filter. Well below any legitimate filter state, so
    // it only fires as a numerical safety net.
    constexpr float P_MIN_DIAG = 1e-12f;
    static constexpr float pmax[15] = {
        P_MAX_POS, P_MAX_POS, P_MAX_POS,
        P_MAX_VEL, P_MAX_VEL, P_MAX_VEL,
        P_MAX_ATT, P_MAX_ATT, P_MAX_ATT,
        P_MAX_ABIAS, P_MAX_ABIAS, P_MAX_ABIAS,
        P_MAX_GBIAS, P_MAX_GBIAS, P_MAX_GBIAS
    };
    // Cooldown (in timeUpdate cycles) for which isHealthy() reports the filter
    // unhealthy after a genuine divergence, giving it time to re-converge before
    // consumers (#257 apogee voters, #265 servo gate) trust it again.
    constexpr uint16_t EKF_UNHEALTHY_COOLDOWN = 500;
    for (int i = 0; i < 15; i++) {
        if (!std::isfinite(P_[i][i]) || P_[i][i] < P_MIN_DIAG) {
            // A non-finite covariance is a genuine divergence (vs a benign
            // underflow below P_MIN_DIAG) — flag the filter unhealthy (#257/#265).
            if (!std::isfinite(P_[i][i])) unhealthy_cooldown_ = EKF_UNHEALTHY_COOLDOWN;
            // Zero the whole row and column so correlation terms can't carry
            // NaN forward even if this diagonal was non-finite.
            for (int j = 0; j < 15; j++) { P_[i][j] = 0.0f; P_[j][i] = 0.0f; }
            P_[i][i] = P_MIN_DIAG;
            continue;
        }
        if (P_[i][i] > pmax[i]) {
            float scale = std::sqrt(pmax[i] / P_[i][i]);
            for (int j = 0; j < 15; j++) {
                P_[i][j] *= scale;
                P_[j][i] *= scale;
            }
            // Diagonal was scaled twice; correct it
            P_[i][i] = pmax[i];
        }
    }
}

// ─── Accelerometer Gravity Reference Update ──────────────────────────

// ─── Shared scalar measurement update ────────────────────────────────
// Generalized scalar Kalman update for a 1-row H with hn nonzero entries
// (hval[m] at column hidx[m]).  Computes the gain, applies the full state
// correction, and updates P in Joseph form exploiting the RANK-1 structure of
// K*H: three outer-product passes (~700 MACs) instead of the dense 15x15
// products (~6750 MACs) each caller previously carried as a private copy of
// this exact sequence.  The dense form dominated the EKF budget whenever the
// AHRS updates ran (mag heading fused at every ~480 Hz tick).
//
//   A  = (I-KH)*P      -> A[i][j] = P[i][j] - K[i]*HP[j]
//   B  = A*(I-KH)^T    -> B[i][j] = A[i][j] - (A[i][:] . H) * K[j]
//   P' = B + R*K*K^T
//
// P is exactly symmetric on entry (every update ends with an explicit
// symmetrize), so K = P*H^T*S^-1 can be read from HP^T bit-identically.
void GpsInsEKF::applyScalarMeasUpdate(const int* hidx, const float* hval,
                                      int hn, float y, float R, float s_min,
                                      bool gate_gyro_bias)
{
    // HP = H*P (1x15), snapshotted before P is modified
    float HP[15] = {};
    for (int m = 0; m < hn; m++) {
        const float h = hval[m];
        const float* Prow = P_[hidx[m]];
        for (int j = 0; j < 15; j++) HP[j] += h * Prow[j];
    }

    // S = H*P*H^T + R  (negated form of !(S > s_min) is deliberate: it also
    // rejects a NaN S instead of proceeding with a poisoned gain)
    float S = R;
    for (int m = 0; m < hn; m++) S += HP[hidx[m]] * hval[m];
    if (!(S > s_min)) return;
    const float S_inv = 1.0f / S;

    // K = P*H^T * S^-1  (= HP^T * S^-1 by symmetry of P)
    float K[15];
    for (int i = 0; i < 15; i++) K[i] = HP[i] * S_inv;

    // ── #508: gyro-bias observability gate (scalar form; see accelMeasUpdate) ──
    // NIS = y²/S. Cut the gyro-bias coupling when the measurement is
    // inconsistent with the state, so an off heading (steel rail, stale
    // hard-iron cal) can't be laundered into the bias. Every other state still
    // corrects normally.
    if (gate_gyro_bias) {
        const float nis = (y * y) * S_inv;
        if (!(nis <= GBIAS_GATE_NIS_SCALAR)) tripGyroBiasGate();  // negated: catches NaN
    }
    if (gate_gyro_bias && gyroBiasGateHeld()) {
        K[12] = 0.0f; K[13] = 0.0f; K[14] = 0.0f;
    }

    // State correction xk = K*y — position via curvature radii, velocity,
    // biases, attitude via left-multiplied error quaternion (the shared
    // convention across all measurement updates).
    float xk[15];
    for (int i = 0; i < 15; i++) xk[i] = K[i] * y;

    double Rew, Rns;
    EarthRad(pEst_D_rrm_[0], &Rew, &Rns);
    pEst_D_rrm_[2] -= xk[2];
    pEst_D_rrm_[0] += xk[0] / (Rns + pEst_D_rrm_[2]);
    pEst_D_rrm_[1] += xk[1] / ((Rew + pEst_D_rrm_[2]) * std::cos(pEst_D_rrm_[0]));
    for (int i = 0; i < 3; i++) {
        vEst_NED_mps_[i] += xk[i + 3];
        aBias_mps2_[i]   += xk[i + 9];
        wBias_rps_[i]    += xk[i + 12];
    }
    clampGyroBias();   // #508 backstop: no path may leave a non-physical bias

    float quatDelta[4] = {1.0f, xk[6], xk[7], xk[8]};
    normalizeQuaternion(quatDelta, quatDelta);
    float qTemp[4];
    multiplyQuaternions(quat_BL_, quatDelta, qTemp);
    for (int i = 0; i < 4; i++) quat_BL_[i] = qTemp[i];

    // Rank-1 Joseph covariance update (in place)
    for (int i = 0; i < 15; i++) {
        const float Ki = K[i];
        for (int j = 0; j < 15; j++)
            P_[i][j] -= Ki * HP[j];
    }
    float Acol[15];
    for (int i = 0; i < 15; i++) {
        float s = 0.0f;
        for (int m = 0; m < hn; m++) s += P_[i][hidx[m]] * hval[m];
        Acol[i] = s;
    }
    for (int i = 0; i < 15; i++) {
        const float KR_i = K[i] * R;
        for (int j = 0; j < 15; j++)
            P_[i][j] += -Acol[i] * K[j] + KR_i * K[j];
    }

    // Symmetrize P
    for (int i = 0; i < 15; i++)
        for (int j = 0; j <= i; j++) {
            float s = 0.5f * (P_[i][j] + P_[j][i]);
            P_[i][j] = s; P_[j][i] = s;
        }

    stabilizeP();
}

void GpsInsEKF::accelMeasUpdate(const float aMeas[3]) {
    // Compute gravity in body frame from current quaternion
    float T_NED2B[3][3];
    Quat2DCM(T_NED2B, quat_BL_);
    float aGrav_B[3] = {T_NED2B[0][2]*G, T_NED2B[1][2]*G, T_NED2B[2][2]*G};

    // Innovation: bias-corrected specific force + estimated gravity ≈ 0 when stationary
    float y[3];
    for (int i = 0; i < 3; i++)
        y[i] = (aMeas[i] - aBias_mps2_[i]) + aGrav_B[i];

    // H matrix (3×15): attitude columns (6–8) and accel bias columns (9–11)
    // ∂y/∂δθ = -2 * skew(aGrav_B),  ∂y/∂δb_accel = +I
    float H[3][15] = {};
    H[0][6] =  0.0f;           H[0][7] =  2*aGrav_B[2];  H[0][8] = -2*aGrav_B[1];
    H[1][6] = -2*aGrav_B[2];   H[1][7] =  0.0f;          H[1][8] =  2*aGrav_B[0];
    H[2][6] =  2*aGrav_B[1];   H[2][7] = -2*aGrav_B[0];  H[2][8] =  0.0f;
    H[0][9]  = 1.0f;
    H[1][10] = 1.0f;
    H[2][11] = 1.0f;

    // S = H * P * H^T + R  (3×3)
    float HP[3][15] = {};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 15; j++)
            for (int k = 6; k <= 11; k++)
                HP[i][j] += H[i][k] * P_[k][j];

    float S[9] = {};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            for (int k = 6; k <= 11; k++)
                S[i*3+j] += HP[i][k] * H[j][k];
            if (i == j) S[i*3+j] += R_accel_;
        }

    float S_inv[9];
    if (!invertMatrix3x3(S, S_inv)) return;

    // K = P * H^T * S^-1  (15×3)
    float PHt[15][3] = {};
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 6; k <= 11; k++)
                PHt[i][j] += P_[i][k] * H[j][k];

    float K[15][3] = {};
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                K[i][j] += PHt[i][k] * S_inv[k*3+j];

    // ── #508: gyro-bias observability gate ──
    // NIS = yᵀ·S⁻¹·y. A gravity innovation this far from what P predicts is not
    // an attitude error the bias can explain — it's a bad measurement (pad bump,
    // wind sway, a frame change). Gyro bias is observable only through slow,
    // CONSISTENT drift, so cut its coupling for this update: zero rows 12-14 of
    // K. Attitude (and accel bias) still correct in full, so a genuine
    // re-orientation on the rail converges exactly as before. Joseph form below
    // is valid for any gain, so P stays consistent — and with these rows zeroed
    // the bias covariance is NOT reduced, so the filter keeps reporting the bias
    // as un-learned. That is what the pre-launch scorecard reads.
    float nis = 0.0f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            nis += y[i] * S_inv[i*3+j] * y[j];
    if (!(nis <= GBIAS_GATE_NIS_VEC)) tripGyroBiasGate();   // negated: catches NaN too
    if (gyroBiasGateHeld()) {
        for (int j = 0; j < 3; j++) {
            K[12][j] = 0.0f; K[13][j] = 0.0f; K[14][j] = 0.0f;
        }
    }

    // State correction: xk = K * y
    float xk[15] = {};
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 3; j++)
            xk[i] += K[i][j] * y[j];

    // Apply corrections
    double Rew, Rns;
    EarthRad(pEst_D_rrm_[0], &Rew, &Rns);
    pEst_D_rrm_[2] -= xk[2];
    pEst_D_rrm_[0] += xk[0] / (Rns + pEst_D_rrm_[2]);
    pEst_D_rrm_[1] += xk[1] / ((Rew + pEst_D_rrm_[2]) * std::cos(pEst_D_rrm_[0]));

    for (int i = 0; i < 3; i++) {
        vEst_NED_mps_[i] += xk[i + 3];
        aBias_mps2_[i] += xk[i + 9];
        wBias_rps_[i] += xk[i + 12];
    }
    clampGyroBias();   // #508 backstop: no path may leave a non-physical bias

    // Quaternion correction
    float quatDelta[4] = {1.0f, xk[6], xk[7], xk[8]};
    normalizeQuaternion(quatDelta, quatDelta);
    float qTemp[4];
    multiplyQuaternions(quat_BL_, quatDelta, qTemp);
    for (int i = 0; i < 4; i++) quat_BL_[i] = qTemp[i];

    // Sparse Joseph form: P = (I-KH)*P*(I-KH)^T + K*R*K^T
    // Exploits H sparsity (non-zero only in cols 6-11) to avoid building
    // full KH and I_KH matrices.  Uses already-computed HP and K.

    // Step 1: I_KH_P[i][j] = P[i][j] - K[i][:] * HP[:][j]
    float I_KH_P[15][15];
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++) {
            float v = P_[i][j];
            for (int m = 0; m < 3; m++)
                v -= K[i][m] * HP[m][j];
            I_KH_P[i][j] = v;
        }

    // Step 2: AHt[i][m] = sum_{k=6}^{11} I_KH_P[i][k] * H[m][k]
    float AHt[15][3];
    for (int i = 0; i < 15; i++)
        for (int m = 0; m < 3; m++) {
            float s = 0.0f;
            for (int k = 6; k <= 11; k++)
                s += I_KH_P[i][k] * H[m][k];
            AHt[i][m] = s;
        }

    // Step 3: P = I_KH_P - AHt * K^T + K * R * K^T
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++) {
            float v = I_KH_P[i][j];
            for (int m = 0; m < 3; m++) {
                v -= AHt[i][m] * K[j][m];
                v += K[i][m] * R_accel_ * K[j][m];
            }
            P_[i][j] = v;
        }

    // Symmetrize P
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < i; j++) {
            float s = 0.5f * (P_[i][j] + P_[j][i]);
            P_[i][j] = s; P_[j][i] = s;
        }

    stabilizeP();
}

// ─── Magnetometer Heading Update (heading-only scalar Kalman) ────────
//
// Fuses a tilt-compensated magnetic-heading measurement into the 15-state
// error covariance, correcting ONLY yaw (rotation about NED-down).  Unlike
// the retired Mahony correction, the measurement — a tilt-compensated atan2
// heading — stays well-defined at every attitude including nose-vertical, so
// there is no cos²(pitch) blind spot.  The accelerometer owns roll/pitch and
// the magnetometer owns heading; their measurement Jacobians are orthogonal in
// the attitude block, so the two updates are complementary, not competing.

void GpsInsEKF::magMeasUpdate(const float aMeas[3], const float magMeas[3]) {
    // Body-down unit vector from the current attitude (= aGrav_B / G); this is
    // both the leveling reference and the heading-error axis (H below).
    float T_NED2B[3][3];
    Quat2DCM(T_NED2B, quat_BL_);
    const float d[3] = { T_NED2B[0][2], T_NED2B[1][2], T_NED2B[2][2] };

    // ── Measured magnetic heading: tilt-compensate the body mag with the
    //    accelerometer (independent of the quaternion's yaw → no circularity).
    const float aN = std::sqrt(aMeas[0]*aMeas[0] + aMeas[1]*aMeas[1] + aMeas[2]*aMeas[2]);
    if (aN < 0.01f) return;
    // Down direction in body = -accel/|accel| (specific force at rest opposes
    // modeled gravity-in-body; same sign convention as accelMeasUpdate).
    const float dmx = -aMeas[0]/aN, dmy = -aMeas[1]/aN, dmz = -aMeas[2]/aN;
    // Roll/pitch from the measured down vector (FRD): d = (-sθ, sφcθ, cφcθ).
    const float roll  = std::atan2(dmy, dmz);
    const float pitch = std::atan2(-dmx, std::sqrt(dmy*dmy + dmz*dmz));
    const float sr = std::sin(roll),  cr = std::cos(roll);
    const float sp = std::sin(pitch), cp = std::cos(pitch);
    // ── #480: within ~2° of exact vertical, heading is unobservable from
    //    accel+mag — the accel-derived roll above is atan2(noise, noise) and
    //    psi_pred below degenerates to atan2(~0, ~0), whose float garbage is
    //    platform-dependent (CI x86 vs bench arm dragged heading ±180° in
    //    opposite ways). Skip outright, like the |B_h| gate: the filter holds
    //    heading on the gyro until the rocket tips a couple of degrees. The
    //    adaptive R below owns the near-vertical band outside this gate.
    if (cp < 0.03f) return;
    // Tilt-compensated horizontal field (level frame), standard e-compass.
    const float mx = magMeas[0], my = magMeas[1], mz = magMeas[2];
    const float Xh = mx*cp + my*sr*sp + mz*cr*sp;   // north-ish
    const float Yh = my*cr - mz*sr;                 // east-ish
    const float Bh2 = Xh*Xh + Yh*Yh;
    if (Bh2 < 9.0f) return;                         // |B_horiz| < 3 µT: heading
                                                    // unobservable (polar/vertical field)
    const float psi_meas = std::atan2(-Yh, Xh) + declination_rad_;   // TRUE heading

    // ── Per-sample measurement noise (#480). The tilt-compensated heading's
    //    noise is strongly attitude-dependent, and a constant R over-trusts
    //    the mag exactly where a rocket lives: near vertical the accel-derived
    //    ROLL is nearly singular (dmy,dmz ~ cos(pitch)·1g + accel noise), and
    //    a roll error δφ leaks the VERTICAL field into the horizontal
    //    components, i.e. δψ ≈ δφ·B_v/|B_h| (Bv/Bh ≈ 1.9 at mid-latitudes).
    //    On the pad at ~85° tilt that is ~6–7° of real per-sample heading
    //    noise against the 3° constant — the over-weighted samples dragged
    //    the heading estimate and pumped the gyro-z bias state (SIL pad NEES
    //    ~390). First-order propagation, all terms already in scope:
    //      σψ² ≈ 2·σ_m²/|B_h|²                (raw field noise on Xh,Yh)
    //           + (σ_roll · B_v/|B_h|)²      (accel tilt-comp roll noise)
    //      σ_roll ≈ (σ_a/|a|)/max(cp, ε)     (roll singular as pitch → ±90°)
    //    Floored at R_mag_ so level attitude keeps the tuned base (which also
    //    covers calibration residuals the propagation can't see), and capped
    //    so S stays finite at exact vertical (the update is then effectively
    //    a no-op — heading is genuinely unobservable from accel+mag there).
    //    KNOWN, ACCEPTED LIMIT (#483): the accel MARKOV BIAS also enters the
    //    tilt-comp roll and hence psi_meas — on the rail that is ~1.3–2° (1σ)
    //    of heading error CORRELATED over τ≈100 s, which no white-R value can
    //    represent. The filter tracks that slow component confidently (SIL
    //    pad heading NEES O(100) vs truth), while its fast-noise honesty is
    //    correct (0.10° observed vs 0.175° claimed). Accepted because the
    //    error is bounded (±~2° at ±2σ bias) and inside guidance-init
    //    tolerance. If sub-degree pad heading is ever needed, model the
    //    coupling instead: extend this update's H to the accel-bias states
    //    with the same geometry as the propagation above (δψ ≈
    //    (Bv/|Bh|)·δb_lat/(g·cosθ)) — design notes in #483.
    const float Bv   = -mx*sp + my*sr*cp + mz*cr*cp;         // vertical field, level frame
    const float sroll = (sigma_accel_mps2_ / aN) / std::max(cp, 0.02f);
    const float r_eff_raw = 2.0f*sigma_mag_uT_*sigma_mag_uT_/Bh2
                          + sroll*sroll*(Bv*Bv)/Bh2;
    const float R_eff = std::min(std::max(r_eff_raw, R_mag_), R_mag_ceiling_);

    // ── Predicted heading (yaw) from the state quaternion (true-NED frame).
    //    Computed from T_NED2B directly (euler_BL_rad_ is a cycle stale here).
    const float psi_pred = std::atan2(T_NED2B[0][1], T_NED2B[0][0]);

    // ── Scalar innovation, wrapped to (-π, π] (≤2 iters: inputs are bounded).
    float y = psi_meas - psi_pred;
    while (y >  (float)M_PI) y -= 2.0f * (float)M_PI;
    while (y < -(float)M_PI) y += 2.0f * (float)M_PI;

    // ── H (1×15): ∂ψ_pred/∂(δθ/2) in the body-frame error state.  ∂ψ/∂δθ is
    //    the body-down axis d (yaw is rotation about NED-down); the factor 2
    //    accounts for the half-angle attitude error xk[6..8] = δθ/2 (matching
    //    accelMeasUpdate's -2·skew).  Nonzero only in cols 6,7,8.
    //    |H|=2 ⇒ S ≥ R_mag_ at any attitude, so the s_min gate is NaN-safety.
    const int   hidx[3] = {6, 7, 8};
    const float hval[3] = {2.0f*d[0], 2.0f*d[1], 2.0f*d[2]};
    applyScalarMeasUpdate(hidx, hval, 3, y, R_eff, 1e-9f, /*gate_gyro_bias=*/true);
}

// ─── Heading update from GNSS velocity course ───────────────────────
// Treats the horizontal velocity direction (course over ground) as a heading
// measurement: at low angle of attack the velocity vector ≈ the nose direction.
// Same scalar-heading Kalman as magMeasUpdate (H ∝ body-down d), but the
// measurement comes from the GNSS velocity — independent of the mag and of the
// attitude tilt-comp, so it has none of the in-flight circularity that
// destabilized the mag update. NOTE: it observes the NOSE direction, so near
// vertical it sees only ~sin(tilt) of an azimuth/roll error — weak when vertical.
void GpsInsEKF::velCourseHeadingUpdate(const float vMeas_NED[3]) {
    float T_NED2B[3][3];
    Quat2DCM(T_NED2B, quat_BL_);
    const float d[3] = { T_NED2B[0][2], T_NED2B[1][2], T_NED2B[2][2] };

    const float psi_meas = std::atan2(vMeas_NED[1], vMeas_NED[0]);     // course (E,N)
    const float psi_pred = std::atan2(T_NED2B[0][1], T_NED2B[0][0]);   // nose azimuth

    float y = psi_meas - psi_pred;
    while (y >  (float)M_PI) y -= 2.0f * (float)M_PI;
    while (y < -(float)M_PI) y += 2.0f * (float)M_PI;

    const float R_course = 0.05f;   // ~13° course noise (rad²)
    const int   hidx[3] = {6, 7, 8};
    const float hval[3] = {2.0f*d[0], 2.0f*d[1], 2.0f*d[2]};
    applyScalarMeasUpdate(hidx, hval, 3, y, R_course, 1e-9f, /*gate_gyro_bias=*/true);
}

// ─── Heading update by body↔world lateral-acceleration matching ─────
// The aero side force (AoA + fins) is measured in the BODY frame (accelerometer
// lateral components) and in the WORLD frame (GNSS-derived horizontal accel).
// They are the same force, so the azimuth between them IS the roll/heading —
// the one DOF the velocity course cannot see near vertical (rolling the body
// does not move the nose, but it does rotate the lateral force in the world).
// Only the LATERAL body force is used (axial/nose component zeroed) so boost
// thrust doesn't dominate; caller gates on a clear lateral force in both frames.
void GpsInsEKF::accelMatchHeadingUpdate(const float aMeas[3], const float aWorldHoriz[2]) {
    float T_NED2B[3][3];
    Quat2DCM(T_NED2B, quat_BL_);
    const float d[3] = { T_NED2B[0][2], T_NED2B[1][2], T_NED2B[2][2] };

    // Rotate the body LATERAL specific force (0, a_y, a_z) into NED.
    const float ay = aMeas[1], az = aMeas[2];
    const float a_pred_N = T_NED2B[1][0]*ay + T_NED2B[2][0]*az;
    const float a_pred_E = T_NED2B[1][1]*ay + T_NED2B[2][1]*az;
    const float pred_h = std::sqrt(a_pred_N*a_pred_N + a_pred_E*a_pred_E);
    const float meas_h = std::sqrt(aWorldHoriz[0]*aWorldHoriz[0] + aWorldHoriz[1]*aWorldHoriz[1]);
    if (pred_h < 0.5f || meas_h < 0.5f) return;   // need a clear lateral force

    const float psi_pred = std::atan2(a_pred_E, a_pred_N);
    const float psi_meas = std::atan2(aWorldHoriz[1], aWorldHoriz[0]);
    float y = psi_meas - psi_pred;
    while (y >  (float)M_PI) y -= 2.0f * (float)M_PI;
    while (y < -(float)M_PI) y += 2.0f * (float)M_PI;

    const float R_am = 0.20f;   // ~26° (rad²) — GNSS-derived accel is noisy
    const int   hidx[3] = {6, 7, 8};
    const float hval[3] = {2.0f*d[0], 2.0f*d[1], 2.0f*d[2]};
    applyScalarMeasUpdate(hidx, hval, 3, y, R_am, 1e-9f, /*gate_gyro_bias=*/true);
}

// ─── Measurement Update (GNSS correction) ───────────────────────────

void GpsInsEKF::measUpdate(double pMeas_D_rrm[3], float vMeas_NED_mps[3]) {
    // Position error in NED
    float T_E2NED[3][3];
    TransE2NED(T_E2NED, pEst_D_rrm_);

    double pMeas_ecef[3], pEst_ecef[3], pDelta[3];
    D2E(pMeas_ecef, pMeas_D_rrm);
    D2E(pEst_ecef, pEst_D_rrm_);
    for (int i=0;i<3;i++) pDelta[i]=pMeas_ecef[i]-pEst_ecef[i];

    float pErr_NED[3];
    for (int i=0;i<3;i++) { pErr_NED[i]=0; for (int j=0;j<3;j++) pErr_NED[i]+=T_E2NED[i][j]*(float)pDelta[j]; }

    float vErr_NED[3];
    for (int i=0;i<3;i++) vErr_NED[i]=vMeas_NED_mps[i]-vEst_NED_mps_[i];

    float y[6];
    for (int i=0;i<3;i++) { y[i]=pErr_NED[i]; y[i+3]=vErr_NED[i]; }

    // Scaled measurement noise.
    // Position noise scales as σ*scale (GNSS h_acc directly inflated),
    // so variance scales as scale².  Velocity noise scales as σ*√scale
    // (Doppler less affected than code phase), so variance scales as scale.
    float pos_scale2 = gpsNoiseScale_ * gpsNoiseScale_;
    float vel_scale  = gpsNoiseScale_;
    float R_scaled[6][6]={};
    for (int i=0;i<3;i++) { R_scaled[i][i]     = R_[i][i]     * pos_scale2;
                            R_scaled[i+3][i+3] = R_[i+3][i+3] * vel_scale; }

    // S = H * P * H^T + R_scaled.  H = [I6 | 0] (position rows read states
    // 0-2, velocity rows states 3-5), so H*P is just the first six ROWS of P
    // and H*P*H^T its top-left 6x6 block — no dense products needed.
    float S[36];
    for (int i=0;i<6;i++) for (int j=0;j<6;j++)
        S[i*6+j] = P_[i][j] + R_scaled[i][j];

    float S_inv[36];
    if (!invertMatrix6x6(S, S_inv)) return;  // singular S — skip update

    // ── Gate 2: split position / velocity innovation tests (#174) ──
    // A single 6-DOF NIS gate let a large innovation in one block — e.g.
    // vertical velocity lagging during high-g boost — reject the ENTIRE
    // pos+vel update, discarding the good horizontal-velocity correction so
    // horizontal velocity dead-reckons and runs away.  Test the position
    // (3-DOF) and velocity blocks independently; on a trip de-weight only that
    // block (inflate its R) rather than dropping the whole measurement.  The
    // velocity block is split further into horizontal (2-DOF) and vertical
    // (1-DOF): the boost vertical-velocity lag is exactly the inconsistency
    // that must not be allowed to de-weight the GOOD horizontal velocity — else
    // horizontal velocity dead-reckons and snaps to GNSS position each fix (the
    // sawtooth seen in guidance).  Marginal precision of each block is the
    // inverse of that block of S.  chi²(0.001): 3-DOF≈16.27, 2≈13.82, 1≈10.83.
    {
        static constexpr float CHI2_GATE_3DOF = 16.27f;  // χ²(3, 0.001) — position
        static constexpr float CHI2_GATE_2DOF = 13.82f;  // χ²(2, 0.001) — horiz velocity
        static constexpr float CHI2_GATE_1DOF = 10.83f;  // χ²(1, 0.001) — vert  velocity
        float Spp[9] = { S[0],  S[1],  S[2],  S[6],  S[7],  S[8],  S[12], S[13], S[14] };
        float Sinv3[9];
        // Huber-style soft de-weight: when a block's NIS exceeds the gate,
        // inflate that block's R by (NIS/gate) instead of rejecting it.  An
        // inconsistent measurement is down-weighted in proportion to how far
        // out it is (so a true outlier has near-zero influence) but never
        // fully discarded — so the filter can still recover after it has
        // drifted (e.g. a large reacquired-GNSS velocity innovation following
        // a GNSS gap keeps pulling the estimate back).
        float infl_pos = 1.0f, infl_vh = 1.0f, infl_vd = 1.0f;
        if (invertMatrix3x3(Spp, Sinv3)) {
            float nis = 0.0f;
            for (int i=0;i<3;i++){ float r=0; for(int j=0;j<3;j++) r+=Sinv3[i*3+j]*y[j]; nis+=y[i]*r; }
            if (nis > CHI2_GATE_3DOF) infl_pos = nis / CHI2_GATE_3DOF;
        }
        // Horizontal velocity (vN, vE) — 2×2 innovation block, indices 3,4.
        {
            const float a = S[21], b = S[22], c = S[27], d = S[28];  // [[vN-vN,vN-vE],[vE-vN,vE-vE]]
            const float det = a*d - b*c;
            if (fabsf(det) > 1e-12f) {
                const float yN = y[3], yE = y[4];
                // NIS = yᵀ Sₕ⁻¹ y, with Sₕ⁻¹ = (1/det)[[d,-b],[-c,a]].
                const float nis = (yN*(d*yN - b*yE) + yE*(a*yE - c*yN)) / det;
                if (nis > CHI2_GATE_2DOF) infl_vh = nis / CHI2_GATE_2DOF;
            }
        }
        // Vertical velocity (vD) — 1×1 block, index 5. This is the term that
        // lags in high-g boost; isolating it keeps that lag from de-weighting
        // the horizontal velocity above.
        {
            const float Svd = S[35];
            if (Svd > 1e-12f) {
                const float nis = (y[5]*y[5]) / Svd;
                if (nis > CHI2_GATE_1DOF) infl_vd = nis / CHI2_GATE_1DOF;
            }
        }
        if (infl_pos > 1.0f || infl_vh > 1.0f || infl_vd > 1.0f) {
            R_scaled[0][0]*=infl_pos; R_scaled[1][1]*=infl_pos; R_scaled[2][2]*=infl_pos;
            R_scaled[3][3]*=infl_vh;  R_scaled[4][4]*=infl_vh;  R_scaled[5][5]*=infl_vd;
            for (int i=0;i<6;i++) for (int j=0;j<6;j++)
                S[i*6+j] = P_[i][j] + R_scaled[i][j];
            if (!invertMatrix6x6(S, S_inv)) return;
        }
    }

    // K = P * H^T * S^-1.  P*H^T is the first six COLUMNS of P.
    float K[15][6]={};
    for (int i=0;i<15;i++) for (int j=0;j<6;j++) for (int k=0;k<6;k++) K[i][j]+=P_[i][k]*S_inv[k*6+j];

    // Gate attitude rows of K by cos⁴(pitch) to suppress GNSS attitude
    // corrections near vertical, where position/velocity innovations have
    // negligible attitude observability.  cos⁴ is much more aggressive
    // than cos² at high pitch (5.8e-5 vs 7.6e-3 at 85°), preventing
    // accumulated corruption from many GNSS updates during vertical flight.
    // Applied before Joseph form so covariance is consistent with the
    // actual correction.
    {
        float sp = std::sin(euler_BL_rad_[1]);
        float cos2p = 1.0f - sp * sp;
        float cos4p = cos2p * cos2p;
        for (int j = 0; j < 6; j++) {
            K[6][j] *= cos4p;
            K[7][j] *= cos4p;
            K[8][j] *= cos4p;
        }
    }

    // Joseph form: P = (I-KH)*P*(I-KH)^T + K*R_scaled*K^T.  With H = [I6|0],
    // K*H = [K | 0] — a rank-6 block — so each product is one 15x15x6 pass
    // against the first six rows/columns instead of a dense 15^3 multiply
    // (~4.6k MACs total vs ~13.8k before, and no 15x15 intermediates for
    // KH/I_KH).  R_scaled is diagonal, so K*R*K^T contracts over one index.
    //
    //   A = (I-KH)*P    -> A[i][j] = P[i][j] - sum_k K[i][k]*P[k][j]   (k<6)
    //   B = A*(I-KH)^T  -> B[i][j] = A[i][j] - sum_k A[i][k]*K[j][k]   (k<6)
    //   P' = B + K*R*K^T
    float A[15][15];
    for (int i=0;i<15;i++) for (int j=0;j<15;j++) {
        float s = P_[i][j];
        for (int k=0;k<6;k++) s -= K[i][k]*P_[k][j];
        A[i][j] = s;
    }
    const float Rdiag[6] = {R_scaled[0][0], R_scaled[1][1], R_scaled[2][2],
                            R_scaled[3][3], R_scaled[4][4], R_scaled[5][5]};
    for (int i=0;i<15;i++) {
        float KRrow[6];
        for (int k=0;k<6;k++) KRrow[k]=K[i][k]*Rdiag[k];
        for (int j=0;j<15;j++) {
            float s = A[i][j];
            for (int k=0;k<6;k++) s += -A[i][k]*K[j][k] + KRrow[k]*K[j][k];
            P_[i][j] = s;
        }
    }

    // Symmetrize P
    for (int i=0;i<15;i++) for (int j=0;j<=i;j++) {
        float s=0.5f*(P_[i][j]+P_[j][i]); P_[i][j]=s; P_[j][i]=s;
    }

    stabilizeP();

    // State update: x = K * y
    float xk[15]={};
    for (int i=0;i<15;i++) for (int j=0;j<6;j++) xk[i]+=K[i][j]*y[j];

    // Apply corrections
    double Rew, Rns;
    EarthRad(pEst_D_rrm_[0], &Rew, &Rns);
    pEst_D_rrm_[2]-=xk[2];
    pEst_D_rrm_[0]+=xk[0]/(Rns+pEst_D_rrm_[2]);
    pEst_D_rrm_[1]+=xk[1]/((Rew+pEst_D_rrm_[2])*std::cos(pEst_D_rrm_[0]));

    for (int i=0;i<3;i++) {
        vEst_NED_mps_[i]+=xk[i+3];
        aBias_mps2_[i]+=xk[i+9];
        wBias_rps_[i]+=xk[i+12];
    }
    clampGyroBias();   // #508 backstop: no path may leave a non-physical bias

    // Attitude correction (K rows 6-8 already gated by cos²(pitch) above)
    float quatDelta[4]={1.0f, xk[6], xk[7], xk[8]};
    normalizeQuaternion(quatDelta, quatDelta);
    float qTemp[4];
    multiplyQuaternions(quat_BL_, quatDelta, qTemp);
    quat_BL_[0]=qTemp[0]; quat_BL_[1]=qTemp[1]; quat_BL_[2]=qTemp[2]; quat_BL_[3]=qTemp[3];
}

// ─── Barometer Measurement Update ───────────────────────────────────

void GpsInsEKF::baroMeasUpdate(EkfBaroData baro_data) {
    if (baro_data.time_us == baroTimePrev_) return;
    baroTimePrev_ = baro_data.time_us;

    // pEst_D_rrm_[2] stores altitude (positive-up), but the EKF state
    // vector index 2 is "Down" in NED.  H_baro = [0..0, -1, 0..0] at
    // index 2 maps the Down state to altitude: z_baro = -x_down = alt.
    //
    // Innovation: y = z - H*x = baro_alt - (-1)*(-down) = baro_alt - alt
    //           = baro_alt - pEst_D_rrm_[2]
    float y = (float)(baro_data.altitude_m - pEst_D_rrm_[2]);

    // H_baro is a single -1 at the Down state; the shared helper does the
    // gain, state correction, and rank-1 Joseph covariance update.
    const int   hidx[1] = {2};
    const float hval[1] = {-1.0f};
    applyScalarMeasUpdate(hidx, hval, 1, y, R_baro_, 1e-10f);
}

// ─── Set Quaternion ─────────────────────────────────────────────────

void GpsInsEKF::setQuaternion(float q0_in, float q1_in, float q2_in, float q3_in) {
    float q_in[4] = {q0_in, q1_in, q2_in, q3_in};
    normalizeQuaternion(q_in, quat_BL_);

    // Update DCM
    float T_NED2B[3][3];
    Quat2DCM(T_NED2B, quat_BL_);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            T_B2NED[j][i] = T_NED2B[i][j];

    // Update Euler angles
    Quat2Euler(quat_BL_, euler_BL_rad_);

    // Reset attitude covariance to near-zero (perfect orientation knowledge)
    // and zero all cross-covariances with attitude states
    float att_var = 1e-6f;  // (0.001 rad)^2 ~ 0.06 deg
    for (int i = 6; i <= 8; i++) {
        for (int j = 0; j < 15; j++) {
            P_[i][j] = 0.0f;
            P_[j][i] = 0.0f;
        }
        P_[i][i] = att_var;
    }
}

// ─── Quaternion to Euler ────────────────────────────────────────────

void GpsInsEKF::Quat2Euler(float q[4], float euler[3]) {
    euler[0] = std::atan2(2.0f*(q[0]*q[1]+q[2]*q[3]),
                          1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]));
    float sinp = 2.0f*(q[0]*q[2]-q[3]*q[1]);
    if (std::fabs(sinp)>=1)
        euler[1] = std::copysign((float)M_PI/2.0f, sinp);
    else
        euler[1] = std::asin(sinp);
    euler[2] = std::atan2(2.0f*(q[0]*q[3]+q[1]*q[2]),
                          1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]));
}
