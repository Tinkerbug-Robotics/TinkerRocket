#pragma once
// GPS/INS Fusion EKF — shared library for flight computer and simulation.
//
// Fuses 6-DOF IMU, GNSS, magnetometer, and barometer into a 15-state
// navigation solution: position (LLA), velocity (NED), attitude (quaternion),
// accelerometer bias, gyroscope bias.
//
// Unified EKF architecture (Mahony AHRS removed):
//   - Quaternion propagation via gyro in timeUpdate()
//   - Accelerometer gravity-reference measurement update (accelMeasUpdate)
//   - Magnetometer proportional heading correction (magHeadingUpdate)
//   - cos²(pitch) gating on magnetometer and GNSS attitude corrections
//   - Specific force magnitude gating (0.5g–1.5g) during thrust/free-fall
//   - Consistent FRD body frame / NED world frame convention

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include <cstdint>
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ─── EKF Input Data Structures ──────────────────────────────────────
// Prefixed with "Ekf" to avoid collision with RocketComputerTypes.h

struct EkfIMUData {
    uint32_t time_us = 0;
    double acc_x = 0, acc_y = 0, acc_z = 0;    // m/s^2  (FRD body frame)
    double gyro_x = 0, gyro_y = 0, gyro_z = 0; // deg/s  (FRD body frame)
};

/// GNSS input in ECEF coordinates (used by simulation)
struct EkfGNSSData {
    uint32_t time_us = 0;
    double ecef_x = 0, ecef_y = 0, ecef_z = 0;    // m
    double ecef_vx = 0, ecef_vy = 0, ecef_vz = 0;  // m/s
};

/// GNSS input in LLA + NED velocity (used by flight computer)
struct EkfGNSSDataLLA {
    uint32_t time_us = 0;
    double lat_rad = 0, lon_rad = 0, alt_m = 0; // radians, radians, meters
    float  vel_n_mps = 0, vel_e_mps = 0, vel_d_mps = 0; // m/s NED
};

struct EkfMagData {
    uint32_t time_us = 0;
    double mag_x = 0, mag_y = 0, mag_z = 0; // uT  (FRD body frame)
};

struct EkfBaroData {
    uint32_t time_us = 0;
    double altitude_m = 0;  // pressure altitude from ISA inversion
};


class GpsInsEKF {
public:
    GpsInsEKF();

    // ─── Initialization (call once at startup) ──────────────────────
    /// Initialize from ECEF GNSS data (simulation path)
    void init(EkfIMUData imu_data, EkfGNSSData gnss_data, EkfMagData mag_data);
    /// Initialize from LLA GNSS data (flight computer path)
    void init(EkfIMUData imu_data, EkfGNSSDataLLA gnss_data, EkfMagData mag_data);

    // ─── Main filter update (call every IMU sample) ─────────────────
    /// Update from ECEF GNSS data (simulation path)
    void update(bool use_ahrs_acc,
                EkfIMUData imu_data,
                EkfGNSSData gnss_data,
                EkfMagData mag_data);
    /// Update from LLA GNSS data (flight computer path)
    void update(bool use_ahrs_acc,
                EkfIMUData imu_data,
                EkfGNSSDataLLA gnss_data,
                EkfMagData mag_data);

    /// Barometer-only measurement update
    void baroMeasUpdate(EkfBaroData baro_data);

    // ─── State injection ────────────────────────────────────────────
    /// Inject a known quaternion (e.g. truth orientation at ignition).
    /// Also resets attitude covariance to near-zero.
    void setQuaternion(float q0, float q1, float q2, float q3);

    /// Set GPS measurement noise scale factor (1.0 = nominal).
    /// Values >1 inflate R_ during measUpdate, useful for de-weighting
    /// GNSS fixes immediately after reacquisition when h_acc is high.
    void setGpsNoiseScale(float scale) { gpsNoiseScale_ = scale; }
    float getGpsNoiseScale() const { return gpsNoiseScale_; }

    /// Inject known position (lat_rad, lon_rad, alt_m) and reset pos covariance.
    void setPosition(double lat_rad, double lon_rad, double alt_m) {
        pEst_D_rrm_[0] = lat_rad; pEst_D_rrm_[1] = lon_rad; pEst_D_rrm_[2] = alt_m;
        P_[0][0] = 1.0f; P_[1][1] = 1.0f; P_[2][2] = 1.0f;
    }

    /// Inject known NED velocity and reset vel covariance.
    void setVelocity(float vn, float ve, float vd) {
        vEst_NED_mps_[0] = vn; vEst_NED_mps_[1] = ve; vEst_NED_mps_[2] = vd;
        P_[3][3] = 0.1f; P_[4][4] = 0.1f; P_[5][5] = 0.1f;
    }

    // ─── Getters ────────────────────────────────────────────────────
    void getAccelEst(float (&r)[3]) const { r[0]=aEst_B_mps2_[0]; r[1]=aEst_B_mps2_[1]; r[2]=aEst_B_mps2_[2]; }
    void getAccelBias(float (&r)[3]) const { r[0]=aBias_mps2_[0]; r[1]=aBias_mps2_[1]; r[2]=aBias_mps2_[2]; }
    void getRotRateEst(float (&r)[3]) const { r[0]=wEst_B_rps_[0]; r[1]=wEst_B_rps_[1]; r[2]=wEst_B_rps_[2]; }
    void getRotRateBias(float (&r)[3]) const { r[0]=wBias_rps_[0]; r[1]=wBias_rps_[1]; r[2]=wBias_rps_[2]; }
    void getOrientEst(float (&r)[3]) const { r[0]=euler_BL_rad_[0]; r[1]=euler_BL_rad_[1]; r[2]=euler_BL_rad_[2]; }
    void getPosEst(double (&r)[3]) const { r[0]=pEst_D_rrm_[0]; r[1]=pEst_D_rrm_[1]; r[2]=pEst_D_rrm_[2]; }
    void getVelEst(float (&r)[3]) const { r[0]=vEst_NED_mps_[0]; r[1]=vEst_NED_mps_[1]; r[2]=vEst_NED_mps_[2]; }
    void getQuaternion(float (&r)[4]) const { r[0]=quat_BL_[0]; r[1]=quat_BL_[1]; r[2]=quat_BL_[2]; r[3]=quat_BL_[3]; }

    void getCovPos(float (&r)[3]) const { r[0]=P_[0][0]; r[1]=P_[1][1]; r[2]=P_[2][2]; }
    void getCovVel(float (&r)[3]) const { r[0]=P_[3][3]; r[1]=P_[4][4]; r[2]=P_[5][5]; }
    void getCovOrient(float (&r)[3]) const { r[0]=P_[6][6]; r[1]=P_[7][7]; r[2]=P_[8][8]; }
    void getCovAccelBias(float (&r)[3]) const { r[0]=P_[9][9]; r[1]=P_[10][10]; r[2]=P_[11][11]; }
    void getCovRotRateBias(float (&r)[3]) const { r[0]=P_[12][12]; r[1]=P_[13][13]; r[2]=P_[14][14]; }

private:
    void timeUpdate();
    void measUpdate(double pMeas_D_rrm[3], float vMeas_NED_mps[3]);
    void accelMeasUpdate(const float aMeas[3]);
    void magHeadingUpdate(const float aMeas[3], const float magMeas[3]);

    // Shared core of update() — called after converting GNSS to LLA + NED
    void updateCore(bool use_ahrs_acc,
                    EkfIMUData imu_data,
                    EkfMagData mag_data,
                    double pMeas_D_rrm[3],
                    float vMeas_NED[3],
                    uint32_t gnss_time_us);

    // Shared core of init() — called after converting GNSS to LLA + NED
    void initCore(EkfIMUData imu_data,
                  EkfMagData mag_data,
                  double pMeas_D_rrm[3],
                  float vMeas_NED[3],
                  uint32_t gnss_time_us);

    // Sensor noise parameters
    float aNoiseSigma_mps2 = 0.20f;
    float aMarkovSigma_mps2 = 0.01f;
    float aMarkovTau_s = 100.0f;
    float wNoiseSigma_rps = 0.00175f;
    float wMarkovSigma_rps = 0.00025f;
    float wMarkovTau_s = 50.0f;
    float pNoiseSigma_NE_m = 3.0f;
    float pNoiseSigma_D_m = 6.0f;
    float vNoiseSigma_NE_mps = 0.5f;
    float vNoiseSigma_D_mps = 1.0f;

    // Initial covariance
    static constexpr float pErrSigma_Init_m = 10.0f;
    static constexpr float vErrSigma_Init_mps = 1.0f;
    static constexpr float attErrSigma_Init_rad = 0.34906f;
    static constexpr float hdgErrSigma_Init_rad = 3.14159f;
    static constexpr float aBiasSigma_Init_mps2 = 0.9810f;
    static constexpr float wBiasSigma_Init_rps = 0.01745f;

    // State variables
    float aBias_mps2_[3];
    float wBias_rps_[3];
    double pEst_D_rrm_[3];
    float vEst_NED_mps_[3];
    float aEst_B_mps2_[3];
    float wEst_B_rps_[3];
    uint32_t tPrev_us_;
    float dt_s_;
    uint32_t timeWeekPrev_;
    uint32_t baroTimePrev_ = 0;
    float euler_BL_rad_[3];

    // Kalman matrices
    float quat_BL_[4];
    float T_B2NED[3][3];
    float x[15];
    float P_[15][15];
    float H_[6][15];
    float Rw_[12][12];
    float R_[6][6];
    float Gs_[15][12];
    float Fs_[15][15];

    // Constants
    static constexpr float G = 9.807f;
    static constexpr double ECC2 = 0.0066943799901;
    static constexpr float EARTH_RADIUS = 6378137.0f;
    static constexpr float RAD2DEG = 180.0f / M_PI;
    static constexpr float DEG2RAD = M_PI / 180.0f;

    // Baro measurement noise variance (2m sigma)^2
    float R_baro_ = 4.0f;

    // GPS measurement noise scale (1.0 = nominal, >1 during recovery)
    float gpsNoiseScale_ = 1.0f;

    // Accel gravity-reference measurement noise variance (sigma m/s²)^2
    float R_accel_ = 0.25f;

    // Mag heading proportional gain
    float magKp_ = 1.0f;

    // Helper methods
    void Quat2Euler(float q[4], float euler[3]);

    inline void ecefToLla(double x, double y, double z,
                          double &lat_deg, double &lon_deg, double &alt_m) {
        static constexpr double a = 6378137.0;
        static constexpr double e2 = 6.69437999014e-3;
        double lon = std::atan2(y, x);
        double p = std::sqrt(x*x + y*y);
        double b = a * std::sqrt(1.0 - e2);
        double ep2 = (a*a - b*b) / (b*b);
        double theta = std::atan2(z * a, p * b);
        double sin_th = std::sin(theta), cos_th = std::cos(theta);
        double lat = std::atan2(z + ep2*b*sin_th*sin_th*sin_th,
                                p - e2*a*cos_th*cos_th*cos_th);
        double N = a / std::sqrt(1.0 - e2*std::sin(lat)*std::sin(lat));
        double alt = p / std::cos(lat) - N;
        lat_deg = lat * 180.0 / M_PI;
        lon_deg = lon * 180.0 / M_PI;
        alt_m = alt;
    }

    inline void ecefVelToEnu(double vx, double vy, double vz,
                             double lat_deg, double lon_deg,
                             double& ve, double& vn, double& vu) {
        double phi = lat_deg * M_PI / 180.0;
        double lam = lon_deg * M_PI / 180.0;
        double sphi = std::sin(phi), cphi = std::cos(phi);
        double slam = std::sin(lam), clam = std::cos(lam);
        ve = -slam*vx + clam*vy;
        vn = -sphi*clam*vx - sphi*slam*vy + cphi*vz;
        vu = cphi*clam*vx + cphi*slam*vy + sphi*vz;
    }

    // Quat2DCM: produces the NED-to-body DCM from a body-to-NED quaternion.
    inline void Quat2DCM(float DCM[3][3], float q[4]) {
        DCM[0][0] = 1.0f - 2.0f*(q[2]*q[2]+q[3]*q[3]);
        DCM[0][1] = 2.0f*(q[1]*q[2]+q[0]*q[3]);
        DCM[0][2] = 2.0f*(q[1]*q[3]-q[0]*q[2]);
        DCM[1][0] = 2.0f*(q[1]*q[2]-q[0]*q[3]);
        DCM[1][1] = 1.0f - 2.0f*(q[1]*q[1]+q[3]*q[3]);
        DCM[1][2] = 2.0f*(q[2]*q[3]+q[0]*q[1]);
        DCM[2][0] = 2.0f*(q[1]*q[3]+q[0]*q[2]);
        DCM[2][1] = 2.0f*(q[2]*q[3]-q[0]*q[1]);
        DCM[2][2] = 1.0f - 2.0f*(q[1]*q[1]+q[2]*q[2]);
    }

    inline void EarthRad(double lat, double *Rew, double *Rns) {
        double denom = std::fabs(1.0 - ECC2*std::sin(lat)*std::sin(lat));
        double sqrt_denom = std::sqrt(denom);
        *Rew = EARTH_RADIUS / sqrt_denom;
        *Rns = EARTH_RADIUS * (1 - ECC2) / (denom * sqrt_denom);
    }

    inline void NED2D_Rate(double pDot_D[3], float v_NED[3], double pRef_D[3]) {
        double Rew, Rns;
        EarthRad(pRef_D[0], &Rew, &Rns);
        pDot_D[0] = v_NED[0] / (Rns + pRef_D[2]);
        pDot_D[1] = v_NED[1] / ((Rew + pRef_D[2]) * std::cos(pRef_D[0]));
        pDot_D[2] = -v_NED[2];
    }

    inline void Skew(float C[3][3], float w[3]) {
        C[0][0]=0;     C[0][1]=-w[2]; C[0][2]=w[1];
        C[1][0]=w[2];  C[1][1]=0;     C[1][2]=-w[0];
        C[2][0]=-w[1]; C[2][1]=w[0];  C[2][2]=0;
    }

    inline void multiplyMatrix3x3(const float A[3][3], const float B[3][3], float R[3][3]) {
        for (int i=0;i<3;i++)
            for (int j=0;j<3;j++) {
                R[i][j]=0;
                for (int k=0;k<3;k++) R[i][j]+=A[i][k]*B[k][j];
            }
    }

    inline void TransE2NED(float T[3][3], double pRef_D[3]) {
        T[0][0]=-std::sin(pRef_D[0])*std::cos(pRef_D[1]);
        T[0][1]=-std::sin(pRef_D[0])*std::sin(pRef_D[1]);
        T[0][2]=std::cos(pRef_D[0]);
        T[1][0]=-std::sin(pRef_D[1]);
        T[1][1]=std::cos(pRef_D[1]);
        T[1][2]=0.0f;
        T[2][0]=-std::cos(pRef_D[0])*std::cos(pRef_D[1]);
        T[2][1]=-std::cos(pRef_D[0])*std::sin(pRef_D[1]);
        T[2][2]=-std::sin(pRef_D[0]);
    }

    inline void D2E(double p_E[3], double p_D[3]) {
        double sinlat=std::sin(p_D[0]), coslat=std::cos(p_D[0]);
        double coslon=std::cos(p_D[1]), sinlon=std::sin(p_D[1]);
        double alt=p_D[2];
        double denom=std::fabs(1.0-ECC2*sinlat*sinlat);
        double Rew=EARTH_RADIUS/std::sqrt(denom);
        p_E[0]=(Rew+alt)*coslat*coslon;
        p_E[1]=(Rew+alt)*coslat*sinlon;
        p_E[2]=(Rew*(1.0-ECC2)+alt)*sinlat;
    }

    inline void multiplyQuaternions(const float q1[4], const float q2[4], float q3[4]) {
        q3[0]=q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
        q3[1]=q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2];
        q3[2]=q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1];
        q3[3]=q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0];
    }

    inline void normalizeQuaternion(const float qi[4], float qo[4]) {
        float m=std::sqrt(qi[0]*qi[0]+qi[1]*qi[1]+qi[2]*qi[2]+qi[3]*qi[3]);
        if (m > 1e-10f) {
            float inv_m = 1.0f / m;
            qo[0]=qi[0]*inv_m; qo[1]=qi[1]*inv_m; qo[2]=qi[2]*inv_m; qo[3]=qi[3]*inv_m;
        } else {
            // Degenerate — reset to identity
            qo[0]=1.0f; qo[1]=0.0f; qo[2]=0.0f; qo[3]=0.0f;
        }
    }

    inline bool invertMatrix6x6(const float* A, float* result) {
        float temp[6][12]={};
        for (int i=0;i<6;++i)
            for (int j=0;j<6;++j) {
                temp[i][j]=A[i*6+j];
                temp[i][j+6]=(i==j)?1.0f:0.0f;
            }
        for (int i=0;i<6;++i) {
            // Partial pivoting: find the row with the largest absolute pivot
            int max_row = i;
            float max_val = std::fabs(temp[i][i]);
            for (int r=i+1;r<6;++r) {
                float v = std::fabs(temp[r][i]);
                if (v > max_val) { max_val = v; max_row = r; }
            }
            if (max_val < 1e-20f) return false;  // singular
            // Swap rows if needed
            if (max_row != i) {
                for (int j=0;j<12;++j) {
                    float t = temp[i][j];
                    temp[i][j] = temp[max_row][j];
                    temp[max_row][j] = t;
                }
            }
            float pivot=temp[i][i];
            for (int j=0;j<12;++j) temp[i][j]/=pivot;
            for (int k=0;k<6;++k) {
                if (k!=i) {
                    float factor=temp[k][i];
                    for (int j=0;j<12;++j) temp[k][j]-=factor*temp[i][j];
                }
            }
        }
        for (int i=0;i<6;++i)
            for (int j=0;j<6;++j) result[i*6+j]=temp[i][j+6];
        return true;
    }

    inline bool invertMatrix3x3(const float A[9], float result[9]) {
        float a=A[0], b=A[1], c=A[2];
        float d=A[3], e=A[4], f=A[5];
        float g=A[6], h=A[7], i=A[8];
        float det = a*(e*i-f*h) - b*(d*i-f*g) + c*(d*h-e*g);
        if (std::fabs(det) < 1e-20f) return false;
        float inv_det = 1.0f / det;
        result[0] = (e*i-f*h)*inv_det;
        result[1] = (c*h-b*i)*inv_det;
        result[2] = (b*f-c*e)*inv_det;
        result[3] = (f*g-d*i)*inv_det;
        result[4] = (a*i-c*g)*inv_det;
        result[5] = (c*d-a*f)*inv_det;
        result[6] = (d*h-e*g)*inv_det;
        result[7] = (b*g-a*h)*inv_det;
        result[8] = (a*e-b*d)*inv_det;
        return true;
    }
};
