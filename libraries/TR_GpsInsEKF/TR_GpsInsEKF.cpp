// GPS/INS Fusion EKF — shared library for flight computer and simulation.
//
// Refactored: Mahony AHRS removed. EKF handles quaternion propagation,
// accelerometer gravity reference, and magnetometer heading correction.

#include "TR_GpsInsEKF.h"

GpsInsEKF::GpsInsEKF() {
    // Observation matrix (H) — 6x15
    std::memset(H_, 0, sizeof(H_));
    H_[0][0]=1; H_[1][1]=1; H_[2][2]=1; H_[3][3]=1; H_[4][4]=1; H_[5][5]=1;

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
    wBias_rps_[0]=wMeas[0]; wBias_rps_[1]=wMeas[1]; wBias_rps_[2]=wMeas[2];

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
    dt_s_ = ((float)(t_us - tPrev_us_)) / 1e6f;
    tPrev_us_ = t_us;
    if (dt_s_ > 0.1f) dt_s_ = 0.1f;

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

    // 9. Magnetometer heading update (needs valid accel for gravity direction)
    if (mag_valid && accel_valid) magHeadingUpdate(aMeas, magMeas);

    // 10. GNSS measurement update
    if ((gnss_time_us - timeWeekPrev_) > 0) {
        timeWeekPrev_ = gnss_time_us;
        measUpdate(pMeas_D_rrm, vMeas_NED);
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

    // Assemble Jacobian
    float skew_temp[3][3], Fs_sub[3][3];
    Skew(skew_temp, aEst_B_mps2_);
    multiplyMatrix3x3(T_B2NED, skew_temp, Fs_sub);
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Fs_[i+3][j+6]=-2.0f*Fs_sub[i][j];
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Fs_[i+3][j+9]=-T_B2NED[i][j];
    float Fs_w[3][3];
    Skew(Fs_w, wEst_B_rps_);
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Fs_[i+6][j+6]=-Fs_w[i][j];

    float PHI[15][15], Q[15][15], PHI_T[15][15];
    float PHI_P[15][15], PHI_P_PHIt[15][15];
    float phi_dt[15][15];
    float Gs_T[12][15];
    float PHI_dt_Gs[15][12], PHI_dt_Gs_Rw[15][12];

    // PHI = I + Fs * dt
    for (int i=0;i<15;i++) for (int j=0;j<15;j++)
        PHI[i][j] = (i==j?1.0f:0.0f) + Fs_[i][j]*dt_s_;

    // Update Gs dynamic portion
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) Gs_[i+3][j]=-T_B2NED[i][j];

    // phi_dt = PHI * dt
    for (int i=0;i<15;i++) for (int j=0;j<15;j++) phi_dt[i][j]=PHI[i][j]*dt_s_;

    // PHI_dt_Gs = phi_dt * Gs
    for (int i=0;i<15;i++) for (int j=0;j<12;j++) {
        PHI_dt_Gs[i][j]=0;
        for (int k=0;k<15;k++) PHI_dt_Gs[i][j]+=phi_dt[i][k]*Gs_[k][j];
    }

    // PHI_dt_Gs_Rw = PHI_dt_Gs * Rw
    for (int i=0;i<15;i++) for (int j=0;j<12;j++) {
        PHI_dt_Gs_Rw[i][j]=0;
        for (int k=0;k<12;k++) PHI_dt_Gs_Rw[i][j]+=PHI_dt_Gs[i][k]*Rw_[k][j];
    }

    // Gs transpose
    for (int i=0;i<15;i++) for (int j=0;j<12;j++) Gs_T[j][i]=Gs_[i][j];

    // Q = PHI_dt_Gs_Rw * Gs^T
    for (int i=0;i<15;i++) for (int j=0;j<15;j++) {
        Q[i][j]=0;
        for (int k=0;k<12;k++) Q[i][j]+=PHI_dt_Gs_Rw[i][k]*Gs_T[k][j];
    }

    // Symmetrize Q
    for (int i=0;i<15;i++) for (int j=0;j<=i;j++) {
        float s=0.5f*(Q[i][j]+Q[j][i]); Q[i][j]=s; Q[j][i]=s;
    }

    // PHI transpose
    for (int i=0;i<15;i++) for (int j=0;j<15;j++) PHI_T[i][j]=PHI[j][i];

    // PHI * P
    for (int i=0;i<15;i++) for (int j=0;j<15;j++) {
        PHI_P[i][j]=0;
        for (int k=0;k<15;k++) PHI_P[i][j]+=PHI[i][k]*P_[k][j];
    }

    // P = PHI * P * PHI^T + Q
    for (int i=0;i<15;i++) for (int j=0;j<15;j++) {
        PHI_P_PHIt[i][j]=0;
        for (int k=0;k<15;k++) PHI_P_PHIt[i][j]+=PHI_P[i][k]*PHI_T[k][j];
        P_[i][j]=PHI_P_PHIt[i][j]+Q[i][j];
    }

    // Symmetrize P
    for (int i=0;i<15;i++) for (int j=0;j<=i;j++) {
        float s=0.5f*(P_[i][j]+P_[j][i]); P_[i][j]=s; P_[j][i]=s;
    }
}

// ─── Accelerometer Gravity Reference Update ──────────────────────────

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

    // Quaternion correction
    float quatDelta[4] = {1.0f, xk[6], xk[7], xk[8]};
    normalizeQuaternion(quatDelta, quatDelta);
    float qTemp[4];
    multiplyQuaternions(quat_BL_, quatDelta, qTemp);
    for (int i = 0; i < 4; i++) quat_BL_[i] = qTemp[i];

    // Joseph form covariance update: P = (I-KH)*P*(I-KH)^T + K*R*K^T
    float KH[15][15] = {};
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++)
            for (int k = 0; k < 3; k++)
                KH[i][j] += K[i][k] * H[k][j];

    float I_KH[15][15];
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++)
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];

    float I_KH_P[15][15] = {};
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++)
            for (int k = 0; k < 15; k++)
                I_KH_P[i][j] += I_KH[i][k] * P_[k][j];

    float P_new[15][15] = {};
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++) {
            for (int k = 0; k < 15; k++)
                P_new[i][j] += I_KH_P[i][k] * I_KH[j][k];
            for (int k = 0; k < 3; k++)
                P_new[i][j] += K[i][k] * R_accel_ * K[j][k];
        }

    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++)
            P_[i][j] = P_new[i][j];

    // Symmetrize P
    for (int i = 0; i < 15; i++)
        for (int j = 0; j <= i; j++) {
            float s = 0.5f * (P_[i][j] + P_[j][i]);
            P_[i][j] = s; P_[j][i] = s;
        }
}

// ─── Magnetometer Heading Update ─────────────────────────────────────

void GpsInsEKF::magHeadingUpdate(const float aMeas[3], const float magMeas[3]) {
    // Proportional heading correction ported from Mahony mag logic.
    // No covariance update — simple, stateless correction.

    float q0l = quat_BL_[0], q1l = quat_BL_[1], q2l = quat_BL_[2], q3l = quat_BL_[3];

    // Normalize accel (gravity direction)
    float ax = aMeas[0], ay = aMeas[1], az = aMeas[2];
    float aNorm = std::sqrt(ax*ax + ay*ay + az*az);
    if (aNorm < 0.01f) return;
    float inv_aNorm = 1.0f / aNorm;
    ax *= inv_aNorm; ay *= inv_aNorm; az *= inv_aNorm;

    // Normalize mag
    float mx = magMeas[0], my = magMeas[1], mz = magMeas[2];
    float mNorm = std::sqrt(mx*mx + my*my + mz*mz);
    if (mNorm < 0.01f) return;
    float inv_mNorm = 1.0f / mNorm;
    mx *= inv_mNorm; my *= inv_mNorm; mz *= inv_mNorm;

    // Pre-compute quaternion products
    float q0q0=q0l*q0l, q0q1=q0l*q1l, q0q2=q0l*q2l, q0q3=q0l*q3l;
    float q1q1=q1l*q1l, q1q2=q1l*q2l, q1q3=q1l*q3l;
    float q2q2=q2l*q2l, q2q3=q2l*q3l;

    // Estimated gravity direction in body frame (half-vectors, Mahony convention)
    float halfvx = q1q3 - q0q2;
    float halfvy = q0q1 + q2q3;
    float halfvz = q0q0 - 0.5f + q3l*q3l;

    // Rotate mag to NED frame to find reference field direction
    float hx = 2*(mx*(0.5f-q2q2-q3l*q3l) + my*(q1q2-q0q3) + mz*(q1q3+q0q2));
    float hy = 2*(mx*(q1q2+q0q3) + my*(0.5f-q1q1-q3l*q3l) + mz*(q2q3-q0q1));
    float bx = std::sqrt(hx*hx + hy*hy);
    float bz = 2*(mx*(q1q3-q0q2) + my*(q2q3+q0q1) + mz*(0.5f-q1q1-q2q2));

    // Expected mag field direction in body frame (from reference [bx, 0, bz])
    float halfwx = bx*(0.5f-q2q2-q3l*q3l) + bz*(q1q3-q0q2);
    float halfwy = bx*(q1q2-q0q3) + bz*(q0q1+q2q3);
    float halfwz = bx*(q0q2+q1q3) + bz*(0.5f-q1q1-q2q2);

    // Mag error: cross product of measured vs expected
    float halfex = my*halfwz - mz*halfwy;
    float halfey = mz*halfwx - mx*halfwz;
    float halfez = mx*halfwy - my*halfwx;

    // Project onto gravity axis (heading-only correction)
    float grav_mag2 = halfvx*halfvx + halfvy*halfvy + halfvz*halfvz;
    if (grav_mag2 > 0.01f) {
        float proj = (halfex*halfvx + halfey*halfvy + halfez*halfvz) / grav_mag2;
        halfex = proj * halfvx;
        halfey = proj * halfvy;
        halfez = proj * halfvz;
    }

    // Scale by cos²(pitch) to avoid gimbal-lock corruption near vertical
    {
        float cross_sq = halfvy*halfvy + halfvz*halfvz;
        float cos2_pitch = (grav_mag2 > 0.01f) ? (cross_sq / grav_mag2) : 1.0f;
        halfex *= cos2_pitch;
        halfey *= cos2_pitch;
        halfez *= cos2_pitch;
    }

    // Apply proportional correction to quaternion via first-order kinematics.
    // Correction rate: omega = magKp_ * [halfex, halfey, halfez]
    // Quaternion update: q += 0.5 * dt * Omega(omega) * q
    float hw[3] = {0.5f * magKp_ * halfex * dt_s_,
                    0.5f * magKp_ * halfey * dt_s_,
                    0.5f * magKp_ * halfez * dt_s_};
    float qw = quat_BL_[0], qx = quat_BL_[1], qy = quat_BL_[2], qz = quat_BL_[3];
    quat_BL_[0] = qw - qx*hw[0] - qy*hw[1] - qz*hw[2];
    quat_BL_[1] = qx + qw*hw[0] + qy*hw[2] - qz*hw[1];
    quat_BL_[2] = qy + qw*hw[1] - qx*hw[2] + qz*hw[0];
    quat_BL_[3] = qz + qw*hw[2] + qx*hw[1] - qy*hw[0];
    normalizeQuaternion(quat_BL_, quat_BL_);
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

    // S = H * P * H^T + R_scaled
    float H_T[15][6];
    for (int i=0;i<6;i++) for (int j=0;j<15;j++) H_T[j][i]=H_[i][j];

    float H_P[6][15]={};
    for (int i=0;i<6;i++) for (int j=0;j<15;j++) for (int k=0;k<15;k++) H_P[i][j]+=H_[i][k]*P_[k][j];

    float S[36]={};
    for (int i=0;i<6;i++) for (int j=0;j<6;j++) {
        for (int k=0;k<15;k++) S[i*6+j]+=H_P[i][k]*H_T[k][j];
        S[i*6+j]+=R_scaled[i][j];
    }

    float S_inv[36];
    if (!invertMatrix6x6(S, S_inv)) return;  // singular S — skip update

    float P_Ht[15][6]={};
    for (int i=0;i<15;i++) for (int j=0;j<6;j++) for (int k=0;k<15;k++) P_Ht[i][j]+=P_[i][k]*H_T[k][j];

    float K[15][6]={};
    for (int i=0;i<15;i++) for (int j=0;j<6;j++) for (int k=0;k<6;k++) K[i][j]+=P_Ht[i][k]*S_inv[k*6+j];

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

    // Joseph form: P = (I-KH)*P*(I-KH)^T + K*R_scaled*K^T
    float KH[15][15]={};
    for (int i=0;i<15;i++) for (int j=0;j<15;j++) for (int k=0;k<6;k++) KH[i][j]+=K[i][k]*H_[k][j];

    float I_KH[15][15];
    for (int i=0;i<15;i++) for (int j=0;j<15;j++) I_KH[i][j]=(i==j?1.0f:0.0f)-KH[i][j];

    float I_KH_P[15][15]={};
    for (int i=0;i<15;i++) for (int j=0;j<15;j++) for (int k=0;k<15;k++) I_KH_P[i][j]+=I_KH[i][k]*P_[k][j];

    float I_KH_P_I_KHt[15][15]={};
    for (int i=0;i<15;i++) for (int j=0;j<15;j++) for (int k=0;k<15;k++) I_KH_P_I_KHt[i][j]+=I_KH_P[i][k]*I_KH[j][k];

    float KR[15][6]={};
    for (int i=0;i<15;i++) for (int j=0;j<6;j++) for (int k=0;k<6;k++) KR[i][j]+=K[i][k]*R_scaled[k][j];

    float KR_Kt[15][15]={};
    for (int i=0;i<15;i++) for (int j=0;j<15;j++) for (int k=0;k<6;k++) KR_Kt[i][j]+=KR[i][k]*K[j][k];

    for (int i=0;i<15;i++) for (int j=0;j<15;j++) P_[i][j]=I_KH_P_I_KHt[i][j]+KR_Kt[i][j];

    // Symmetrize P
    for (int i=0;i<15;i++) for (int j=0;j<=i;j++) {
        float s=0.5f*(P_[i][j]+P_[j][i]); P_[i][j]=s; P_[j][i]=s;
    }

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

    // S = H*P*H^T + R.  With H[2]=-1: S = (-1)*P[2][2]*(-1) + R = P[2][2] + R
    float S = P_[2][2] + R_baro_;
    if (S < 1e-10f) return;
    float S_inv = 1.0f / S;

    // K = P * H^T * S^-1.  H^T has -1 at row 2, so K[i] = P[i][2]*(-1)*S_inv
    float K[15];
    for (int i = 0; i < 15; i++) K[i] = -P_[i][2] * S_inv;

    // State correction: x = K * y  (x is in NED, so xk[2] is Down)
    float xk[15];
    for (int i = 0; i < 15; i++) xk[i] = K[i] * y;

    // Apply corrections — same convention as GPS measUpdate:
    // altitude (up) subtracts the Down correction
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

    // Quaternion correction
    float quatDelta[4] = {1.0f, xk[6], xk[7], xk[8]};
    normalizeQuaternion(quatDelta, quatDelta);
    float qTemp[4];
    multiplyQuaternions(quat_BL_, quatDelta, qTemp);
    for (int i = 0; i < 4; i++) quat_BL_[i] = qTemp[i];

    // Joseph form covariance: P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    // H[2] = -1, so K*H row i col j = K[i] * (-1 if j==2, else 0)
    float I_KH[15][15];
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++)
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * (j == 2 ? -1.0f : 0.0f);

    float P_new[15][15] = {};
    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++)
            for (int k = 0; k < 15; k++)
                P_new[i][j] += I_KH[i][k] * P_[k][j];

    for (int i = 0; i < 15; i++)
        for (int j = 0; j < 15; j++) {
            float sum = 0;
            for (int k = 0; k < 15; k++) sum += P_new[i][k] * I_KH[j][k];
            P_[i][j] = sum + K[i] * R_baro_ * K[j];
        }

    // Symmetrize P
    for (int i = 0; i < 15; i++)
        for (int j = 0; j <= i; j++) {
            float s = 0.5f * (P_[i][j] + P_[j][i]);
            P_[i][j] = s; P_[j][i] = s;
        }
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
