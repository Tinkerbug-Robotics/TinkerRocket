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
//   - Magnetometer heading-only scalar Kalman update (magMeasUpdate):
//     tilt-compensated, true-north via magnetic declination, well-conditioned
//     at any attitude including vertical (no cos²(pitch) blind spot)
//   - cos⁴(pitch) gating on GNSS velocity-derived attitude correction only
//   - Specific force magnitude gating (0.5g–1.5g) during thrust/free-fall
//   - Consistent FRD body frame / NED world frame convention

#include <compat.h>

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


// Bulk snapshot of all EKF internal state for persistence across reboots.
struct EkfStateSnapshot {
    double pos_rrm[3];        // LLA position (rad, rad, m)
    float  vel_ned_mps[3];    // NED velocity (m/s)
    float  quat[4];           // body-to-NED quaternion (scalar-first)
    float  accel_bias[3];     // accelerometer bias (m/s²)
    float  gyro_bias[3];      // gyroscope bias (rad/s)
    float  P[15][15];         // error covariance matrix
    uint32_t t_prev_us;       // last time-update timestamp (µs)
    float  euler[3];          // cached Euler angles (rad)
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

    /// Magnetic declination (radians, EAST-positive) added to the measured
    /// magnetic heading so the filter tracks TRUE north.  0 = magnetic north.
    /// Set once from GPS lat/lon/date via the World Magnetic Model.
    void setDeclination(float dec_rad) { declination_rad_ = dec_rad; }
    float getDeclination() const { return declination_rad_; }

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
    // #440 diagnostics: updates skipped on a frozen IMU timestamp.
    uint32_t frozenDtSkips() const { return frozen_dt_skips_; }

    void getOrientEst(float (&r)[3]) const { r[0]=euler_BL_rad_[0]; r[1]=euler_BL_rad_[1]; r[2]=euler_BL_rad_[2]; }
    void getPosEst(double (&r)[3]) const { r[0]=pEst_D_rrm_[0]; r[1]=pEst_D_rrm_[1]; r[2]=pEst_D_rrm_[2]; }
    void getVelEst(float (&r)[3]) const { r[0]=vEst_NED_mps_[0]; r[1]=vEst_NED_mps_[1]; r[2]=vEst_NED_mps_[2]; }
    void getQuaternion(float (&r)[4]) const { r[0]=quat_BL_[0]; r[1]=quat_BL_[1]; r[2]=quat_BL_[2]; r[3]=quat_BL_[3]; }

    /// EKF health for downstream consumers (#257 apogee voter exclusion, #265
    /// servo-stow gate).  v1 signal: the quaternion + velocity + position
    /// estimates are finite AND stabilizeP() has not had to repair a
    /// non-finite covariance within the last divergence-cooldown window.
    /// #572: position was missing — a NaN reaching only pEst_D_rrm_ fed NaN
    /// into guidance range/bearing and the landing estimate while isHealthy()
    /// still said true, so the voter and servo gate kept trusting it.
    /// Conservative — covariance-magnitude thresholds are deferred until
    /// tuned against flight logs.  (Bias states are deliberately not checked:
    /// a non-finite bias corrupts velocity/attitude within one predict step,
    /// so these checks already catch it a tick later.)
    bool isHealthy() const {
        for (int i = 0; i < 4; ++i) if (!std::isfinite(quat_BL_[i]))      return false;
        for (int i = 0; i < 3; ++i) if (!std::isfinite(vEst_NED_mps_[i])) return false;
        for (int i = 0; i < 3; ++i) if (!std::isfinite(pEst_D_rrm_[i]))   return false;  // #572
        return unhealthy_cooldown_ == 0;
    }

    void getCovPos(float (&r)[3]) const { r[0]=P_[0][0]; r[1]=P_[1][1]; r[2]=P_[2][2]; }
    void getCovVel(float (&r)[3]) const { r[0]=P_[3][3]; r[1]=P_[4][4]; r[2]=P_[5][5]; }
    void getCovOrient(float (&r)[3]) const { r[0]=P_[6][6]; r[1]=P_[7][7]; r[2]=P_[8][8]; }
    void getCovAccelBias(float (&r)[3]) const { r[0]=P_[9][9]; r[1]=P_[10][10]; r[2]=P_[11][11]; }
    void getCovRotRateBias(float (&r)[3]) const { r[0]=P_[12][12]; r[1]=P_[13][13]; r[2]=P_[14][14]; }

    // ─── #508: gyro-bias health, for the pre-launch go/no-go ───────
    //
    // These exist because accel/mag updates are gated OFF for the whole flight:
    // whatever the gyro bias holds at launch is FROZEN IN and integrates into
    // attitude at that rate for the entire ascent (a 5 dps error → 5°/s → ~50°
    // by apogee on a 10 s climb), and guidance keys off attitude. So a bias that
    // is large, still un-learned, or has been clipped is a genuine no-go — and
    // before this there was nothing that could tell you.

    // Largest |gyro-bias| component, deg/s. A healthy ISM6 sits well under 2.
    float gyroBiasMaxDps() const {
        float m = 0.0f;
        for (int i = 0; i < 3; ++i) {
            const float a = std::fabs(wBias_rps_[i]);
            if (a > m) m = a;
        }
        return m * RAD2DEG;
    }

    // Largest gyro-bias 1σ, deg/s — how well the bias has actually been learned.
    // Stays high precisely when the observability gate has been cutting the
    // bias coupling, which is the case we must not launch on.
    float gyroBiasSigmaMaxDps() const {
        float m = 0.0f;
        for (int i = 12; i <= 14; ++i) {
            const float v = P_[i][i];
            if (v > m) m = v;
        }
        return std::sqrt(m) * RAD2DEG;
    }

    // True when the bias estimate is physically plausible and the bound never had
    // to bite. Note the sigma bound is a guard against a genuinely diverged
    // covariance (it can reach ~5.7 dps at the P cap), NOT a convergence test:
    // gyro bias is only weakly observable on a stationary, consistent pad — with
    // no rotation there is little information to separate bias from attitude — so
    // sigma sits near ~0.9 dps even when fully settled. That is expected.
    bool gyroBiasHealthy(float max_dps = 3.0f, float max_sigma_dps = 2.0f) const {
        return gbias_clip_count_ == 0
            && gyroBiasMaxDps()      <= max_dps
            && gyroBiasSigmaMaxDps() <= max_sigma_dps;
    }

    uint32_t gyroBiasGateTrips() const { return gbias_gate_trips_; }
    uint32_t gyroBiasClipCount() const { return gbias_clip_count_; }

    // ─── Bulk state save/restore (reboot recovery) ─────────────────
    void getState(EkfStateSnapshot& s) const {
        std::memcpy(s.pos_rrm,     pEst_D_rrm_,   sizeof(s.pos_rrm));
        std::memcpy(s.vel_ned_mps, vEst_NED_mps_,  sizeof(s.vel_ned_mps));
        std::memcpy(s.quat,        quat_BL_,       sizeof(s.quat));
        std::memcpy(s.accel_bias,  aBias_mps2_,    sizeof(s.accel_bias));
        std::memcpy(s.gyro_bias,   wBias_rps_,     sizeof(s.gyro_bias));
        std::memcpy(s.P,           P_,             sizeof(s.P));
        s.t_prev_us = tPrev_us_;
        std::memcpy(s.euler,       euler_BL_rad_,  sizeof(s.euler));
    }

    void setState(const EkfStateSnapshot& s) {
        std::memcpy(pEst_D_rrm_,   s.pos_rrm,     sizeof(pEst_D_rrm_));
        std::memcpy(vEst_NED_mps_,  s.vel_ned_mps, sizeof(vEst_NED_mps_));
        std::memcpy(quat_BL_,       s.quat,        sizeof(quat_BL_));
        std::memcpy(aBias_mps2_,    s.accel_bias,  sizeof(aBias_mps2_));
        std::memcpy(wBias_rps_,     s.gyro_bias,   sizeof(wBias_rps_));
        std::memcpy(P_,             s.P,           sizeof(P_));
        tPrev_us_ = s.t_prev_us;
        std::memcpy(euler_BL_rad_,  s.euler,       sizeof(euler_BL_rad_));
        // Rebuild DCM from restored quaternion.  Quat2DCM produces T_NED2B;
        // transpose into T_B2NED (#386: this used to store the inverse —
        // latent because timeUpdate() recomputes the DCM every tick, but
        // wrong for any consumer in the window between a reboot-recovery
        // restore and the first EKF tick).  Same pattern as setQuaternion().
        float T_NED2B[3][3];
        Quat2DCM(T_NED2B, quat_BL_);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                T_B2NED[j][i] = T_NED2B[i][j];
    }

    /// Add offset to covariance diagonal (inflate uncertainty after reboot)
    void inflateCovDiag(int idx, float delta) {
        if (idx >= 0 && idx < 15) P_[idx][idx] += delta;
    }

    /// Extract the 15 diagonal elements of P into a flat array.
    /// Used to fit the EKF covariance into a wire-format snapshot
    /// (see FlightSnapshotData) without serializing the full 225-element matrix.
    void getCovDiag(float (&out)[15]) const {
        for (int i = 0; i < 15; i++) out[i] = P_[i][i];
    }

    /// Set the diagonal of P from a flat array; zero all off-diagonal terms.
    /// Recovery path: cross-correlations are lost but the per-state
    /// uncertainty is preserved exactly; the filter rebuilds off-diagonals
    /// over the next ~0.5-1 s of measurement updates.
    void setCovFromDiag(const float (&in)[15]) {
        for (int i = 0; i < 15; i++) {
            for (int j = 0; j < 15; j++) {
                P_[i][j] = (i == j) ? in[i] : 0.0f;
            }
        }
    }

private:
    void timeUpdate();
    void measUpdate(double pMeas_D_rrm[3], float vMeas_NED_mps[3]);
    void accelMeasUpdate(const float aMeas[3]);
    void magMeasUpdate(const float aMeas[3], const float magMeas[3]);
    // Heading update from the GNSS velocity course (direction of travel ≈ nose
    // heading at low AoA). Independent of mag/attitude tilt-comp (no
    // circularity). Caller gates on sufficient horizontal speed.
    void velCourseHeadingUpdate(const float vMeas_NED[3]);
    // Heading update by matching the body-frame lateral specific force (aero
    // side force from AoA/fins) against the GNSS-derived world horizontal
    // acceleration. The rotation between the two frames IS the roll, so unlike
    // the velocity course this observes the roll DOF directly — strongest near
    // vertical with an active lateral force. aWorldHoriz = [aN, aE] (m/s²).
    void accelMatchHeadingUpdate(const float aMeas[3], const float aWorldHoriz[2]);

    // Numerical safety net: floor P diagonals at a tiny positive value and
    // cap them at the per-state P_MAX_* values. Call after every path that
    // mutates P so float32 drift can't leave a negative or NaN on the diagonal
    // to propagate through sqrt(P) / 1/P downstream.
    void stabilizeP();

    // Shared core of update() — called after converting GNSS to LLA + NED
    void updateCore(bool use_ahrs_acc,
                    EkfIMUData imu_data,
                    EkfMagData mag_data,
                    double pMeas_D_rrm[3],
                    float vMeas_NED[3],
                    uint32_t gnss_time_us);

    // #834 item 5: restore P_ and the run-scoped state so init() is a real
    // from-scratch init. Called by the constructor and by initCore().
    void resetFilterState();

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
    // GNSS Doppler velocity is better than the old values implied. Measured
    // sample noise in benign coast: horizontal ~0.2 m/s, vertical ~0.28 m/s.
    // The old 0.5 / 1.0 under-trusted it ~6×/13× in variance, so the filter
    // dead-reckoned instead of using a good measurement. Tighten both to ~0.3
    // (matched to the clean-case noise). The boost failure mode is NOT a uniform
    // 1 m/s degradation and NOT loss of fix (stays 3D, 13 sats) — it is a large
    // *systematic* corruption (~6 m/s vertical). That is exactly what the
    // per-axis NIS gate in measUpdate() rejects (horizontal 2-DOF, vertical
    // 1-DOF independently), so tight nominal R + the gate is the right model:
    // trust the clean measurement, reject the corrupted one. Vertical position
    // is also baro-anchored, so over-trusting vertical velocity is low risk.
    float vNoiseSigma_NE_mps = 0.3f;
    float vNoiseSigma_D_mps  = 0.3f;

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
    // #386: default-initialized — the FC path always runs init()/
    // setQuaternion() first, but the sim/ECEF path could reach measUpdate()'s
    // cos^4(pitch) gate with garbage euler_BL_rad_ (and dt math with garbage
    // tPrev_us_/timeWeekPrev_) on its very first update.
    // #440: updates skipped because the IMU timestamp didn't advance.
    // A nonzero value in flight means the IMU stream stalled or replayed.
    uint32_t frozen_dt_skips_ = 0;
    uint32_t tPrev_us_ = 0;
    float dt_s_ = 0.0f;
    uint32_t timeWeekPrev_ = 0;
    uint32_t baroTimePrev_ = 0;
    // #459: last fused mag sample time — fuse each unique sample exactly once
    // (the mag delivers ~98 Hz; updateCore ticks ~480 Hz).
    uint32_t magTimePrev_ = 0;
    float euler_BL_rad_[3] = {0.0f, 0.0f, 0.0f};

    // GNSS-acceleration estimate for accelMatchHeadingUpdate: previous GNSS
    // velocity + sample time to difference, and a low-pass on the result
    // (differencing 25 Hz velocity is noisy).
    float    prevGnssVel_NED_[3] = {0,0,0};
    uint32_t prevGnssSampleUs_   = 0;
    bool     haveGnssAccel_      = false;
    float    gnssAccelLP_NE_[2]  = {0,0};   // low-passed world horizontal accel

    // #257/#265 health: counts down (one per timeUpdate) after stabilizeP()
    // repairs a non-finite covariance — a genuine divergence — keeping
    // isHealthy() false while the filter re-converges.
    uint16_t unhealthy_cooldown_ = 0;

    // Shared scalar measurement update: 1-row H with hn nonzero entries
    // (hval[m] at state index hidx[m]).  Computes the gain, applies the full
    // state correction (pos/vel/bias/quat), and updates P in Joseph form
    // exploiting the rank-1 structure of K*H.  Used by the baro-altitude and
    // the three heading updates (mag, GNSS course, accel-match).
    // `gate_gyro_bias` (#508) arms the gyro-bias observability gate. Set it ONLY
    // for measurements that carry ATTITUDE information (the three heading
    // updates) — those are the ones whose inconsistency can be laundered into
    // the bias. Baro measures altitude: its gyro-bias coupling is incidental, and
    // a large baro innovation during a climb is the filter lagging, not a bad
    // measurement, so gating there would fire constantly for no reason.
    void applyScalarMeasUpdate(const int* hidx, const float* hval, int hn,
                               float y, float R, float s_min,
                               bool gate_gyro_bias = false);

    // Kalman matrices
    float quat_BL_[4];
    float T_B2NED[3][3];
    float x[15];
    float P_[15][15];
    // (GNSS H = [I6 | 0] is exploited structurally in measUpdate — no stored H.)
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

    // Mag heading-only Kalman measurement noise variance FLOOR (rad²; ~3°
    // sigma) — the level-attitude base, which also absorbs calibration
    // residuals. #480: the effective R is computed per sample in
    // magMeasUpdate from the tilt geometry (accel roll noise leaking the
    // vertical field into the heading), floored here and capped below.
    float R_mag_ = 0.0027f;
    // Cap on the per-sample mag R (rad²; ~14° sigma). Near exact vertical the
    // propagated noise diverges (roll unobservable from accel); the cap keeps
    // the pad heading usable — converging over seconds instead of never —
    // while staying ~5x more honest there than the old 3° constant. The SIL
    // honesty evidence (#480) constrains attitudes up to the ~85° rail, where
    // the propagation (~6.7°) rules and this cap is far from binding.
    float R_mag_ceiling_ = 0.06f;
    // Physical 1σ inputs for the #480 propagation. Mag: IIS2MDC RMS noise
    // (~0.4 µT) plus margin. Accel: ISM6 low-g noise density integrated over
    // the AHRS bandwidth plus pad vibration margin.
    float sigma_mag_uT_ = 0.5f;
    float sigma_accel_mps2_ = 0.05f;

    // Magnetic declination (rad, EAST-positive); added to the measured magnetic
    // heading to yield a true-north heading.  Set from GPS+WMM at fix; 0 until.
    float declination_rad_ = 0.0f;

    // P diagonal clamping — prevents float32 overflow when states are
    // unobservable (e.g. no GPS fix on bench).  Caps are well above
    // initial P so they never fire during normal GNSS-aided operation.
    static constexpr float P_MAX_POS   = 1e8f;   // ~10 km sigma
    static constexpr float P_MAX_VEL   = 1e4f;   // ~100 m/s sigma
    static constexpr float P_MAX_ATT   = 10.0f;   // ~180 deg sigma
    static constexpr float P_MAX_ABIAS = 10.0f;   // ~3 m/s² sigma
    static constexpr float P_MAX_GBIAS = 0.01f;   // ~6 deg/s sigma

    // ── #508: gyro-bias observability gate + physical bound ──────────────
    //
    // Gyro bias is a PHYSICAL sensor parameter: the ISM6's zero-rate offset is
    // ~±1 dps typical, a few dps over temperature. It is observable ONLY through
    // slow, CONSISTENT attitude drift — a large transient innovation carries no
    // gyro-bias information at all; it means the attitude is off, or the
    // measurement is bad (pad bump, wind sway, a steel rail deflecting the mag,
    // a stale hard-iron cal, a wrong mounting setting).
    //
    // But accel/mag are the only updates that touch attitude, and K = P·Hᵀ·S⁻¹
    // is 15×N — so the correction reaches gyro bias through the attitude↔bias
    // cross-covariance whether or not H names those columns. Unlike the GNSS
    // update (NIS-gated since #174), accel/mag had NO consistency test, so an
    // inconsistent measurement was laundered straight into the bias state.
    // Observed on the bench: -190 dps, ~100x beyond anything physical. Because
    // accel/mag updates are gated off in flight, whatever the bias holds at
    // launch is FROZEN IN and integrates into attitude for the whole ascent.
    //
    // Fix: when the innovation is inconsistent with the state (NIS above the
    // gate), zero ONLY the gyro-bias rows of K. Attitude still corrects in full
    // — so genuinely re-orienting the rocket on the rail still works — but the
    // bias cannot absorb the inconsistency. Joseph form is valid for ANY gain,
    // not just the optimal one, so P stays consistent and PSD; and since those
    // rows are zeroed, the bias covariance is NOT reduced, so the filter
    // correctly reports that it has not learned the bias.
    //
    // Thresholds are deliberately generous — they must not fire in normal
    // operation, only on the wild innovations that indicate a bad measurement.
    // Reference: R_accel_ = 0.25 (σ ≈ 0.5 m/s²). A legitimate 5° pad tilt gives
    // NIS ≈ 3 (passes); the 81° frame step gives NIS ≈ 380 (trips).
    // Chi-squared 99.9% thresholds for the innovation's DOF — a textbook NIS
    // gate, not a tuned number. A legitimate 5° pad tilt sits at NIS ≈ 3 (passes);
    // the 81° frame step is ≈ 380 (trips).
    static constexpr float GBIAS_GATE_NIS_VEC    = 16.27f;  // chi2(3, 0.999)
    static constexpr float GBIAS_GATE_NIS_SCALAR = 10.83f;  // chi2(1, 0.999)

    // The gate LATCHES. Cutting the coupling only while NIS is above threshold
    // is not enough: as the attitude slews toward a genuinely new gravity vector
    // the innovation shrinks back under the gate, and the filter then absorbs the
    // remaining "attitude moved but the gyro reported nothing" discrepancy into
    // the bias — which is precisely the laundering we are trying to stop. So once
    // a trip occurs, hold the bias coupling cut until the measurements have been
    // consistent for a dwell, i.e. until the slew has finished. Sized to comfortably
    // outlast the accel update's attitude time constant.
    static constexpr uint32_t GBIAS_GATE_HOLD_US = 2000000u;   // 2 s
    // #572: store the TRIP time and compare elapsed, not an absolute
    // "hold-until" deadline. The old `tPrev_us_ < hold_until` broke across
    // the uint32 microsecond wrap (~71.6 min) twice over: a trip within 2 s
    // of the wrap computed a small hold_until so the hold dropped instantly,
    // and long after any trip the wrapped tPrev_us_ dipped BELOW the stale
    // nonzero hold_until, spuriously re-holding the coupling cut. Elapsed
    // arithmetic is wrap-safe, and the armed flag disarms on expiry so a
    // stale trip time can't re-trigger 71 minutes later.
    uint32_t gbias_gate_trip_us_ = 0;
    bool     gbias_gate_armed_   = false;

    // True while the gyro-bias coupling is held cut (gate tripped recently).
    inline bool gyroBiasGateHeld() {
        if (!gbias_gate_armed_) return false;
        if ((uint32_t)(tPrev_us_ - gbias_gate_trip_us_) >= GBIAS_GATE_HOLD_US) {
            gbias_gate_armed_ = false;   // expired — disarm so wrap can't re-hold
            return false;
        }
        return true;
    }
    // Arm/refresh the hold and count the trip.
    inline void tripGyroBiasGate() {
        ++gbias_gate_trips_;
        gbias_gate_armed_   = true;
        gbias_gate_trip_us_ = tPrev_us_;
    }

    // Hard physical bound on the gyro-bias state (rad/s). A backstop: no
    // estimate outside a real gyro's envelope is a bias, whatever path produced
    // it. ±10 dps is ~5-10x the ISM6 spec — generous enough never to clip a
    // genuine bias, tight enough that no path can reach -190 dps.
    static constexpr float GYRO_BIAS_MAX_RPS = 10.0f * (float)(M_PI / 180.0);

    // Clamp the gyro-bias state into its physical envelope. Called after every
    // state correction. Counts clips so the pre-launch scorecard can see that
    // the estimator has been driven somewhere impossible (#508).
    inline void clampGyroBias() {
        for (int i = 0; i < 3; ++i) {
            if (wBias_rps_[i] >  GYRO_BIAS_MAX_RPS) {
                wBias_rps_[i] =  GYRO_BIAS_MAX_RPS; ++gbias_clip_count_;
            } else if (wBias_rps_[i] < -GYRO_BIAS_MAX_RPS) {
                wBias_rps_[i] = -GYRO_BIAS_MAX_RPS; ++gbias_clip_count_;
            }
        }
    }

    uint32_t gbias_gate_trips_ = 0;   // updates whose gyro-bias coupling was cut
    uint32_t gbias_clip_count_ = 0;   // times the bound actually bit

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
