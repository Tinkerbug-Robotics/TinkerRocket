#ifndef TRKINEMATICCHECKS_H
#define TRKINEMATICCHECKS_H

#include <compat.h>

class TR_KinematicChecks
{
public:

    // Constructor
    TR_KinematicChecks();

    // Reset all state (for sim restart without reboot)
    void reset();

    // The apogee vote by sensor (owner's design, 2026-11-19).  The vote
    // counts sensors, not filters: the navigation filter's velocity carries
    // GNSS and the barometer, so beside the GPS and barometer voters it is the
    // same receiver or port voting twice.  With in_use, the velocity voter is
    // the accelerometer's own vertical velocity and the pitch voter the
    // gyro's own pitch (InertialVertical), and the filter casts no vote.
    // Pitch votes only while the rocket is solidly in the atmosphere, where
    // the air keeps its nose on its path (pitch_may_vote: below the
    // barometer's range).  When fewer than two sensors are left, the one that
    // is may call apogee alone once it reads a clear descent
    // (SINGLE_SENSOR_DESCENT_MPS).  in_use false (the default) keeps the vote
    // by filter, for every caller that predates it.
    struct InertialVote
    {
        bool  in_use = false;
        bool  healthy = false;          // InertialVertical::healthy()
        float vz_up_mps = 0.0f;         // + up
        float height_m = 0.0f;          // above the pad
        float pitch_rad = 1.57f;        // + nose up
        bool  pitch_may_vote = false;   // below the barometer's range
    };

    void kinematicChecks(float pressure_altitude,
                         float acc_mag,
                         float position[3],
                         float velocity[3],
                         float roll_rate,
                         bool  new_baro = true,
                         float gps_altitude = 0.0f,
                         bool  new_gps = false,
                         float pitch_rad = 1.57f,
                         bool  burnout_detected = false,
                         bool  baro_locked_out = false,
                         float gps_vel_u = 0.0f,
                         bool  ekf_healthy = true,
                         bool  baro_healthy = true,
                         bool  imu_healthy = true,
                         // GnssAscentGate: GNSS may cast an apogee vote only
                         // once it has been admitted after burnout.  While
                         // false the GPS voter neither counts toward the
                         // quorum nor accumulates.  Default keeps every
                         // caller that predates the gate unchanged.
                         bool  gnss_may_vote = true,
                         // A fresh barometer reading in or out of its range:
                         // the extended-quiescence landing path keeps its
                         // altitude test while the barometer still reads,
                         // even above its 25 kPa floor (V10 pre-flight
                         // review).  False keeps the old rule (the test only
                         // while baro_healthy).
                         bool  baro_reading_fresh = false,
                         // The vote by sensor (InertialVote above); null
                         // keeps the vote by filter.
                         const InertialVote* inertial = nullptr);

    // How old a GNSS fix may be and still count the GPS apogee voter as
    // available: 500 ms by default, longer for a receiver whose fix comes
    // less often (the gnss-sdr-dev board's once a second).
    void setGpsApogeeFreshMs(uint32_t ms) { gps_apogee_fresh_ms_ = ms; }
    // How many descending fixes the GPS voter needs: 4 by default (about
    // 0.2 s at the u-blox's 18 Hz), fewer for a receiver whose fix comes once
    // a second, where 4 would put the call 3-4 s after apogee: past the
    // owner's bar of 2 s, and above the barometer's range GNSS is one of the
    // only two sensors left (the vote by sensor).
    void setGpsApogeeCountHi(uint8_t n) { gps_apogee_count_hi_ = n ? n : 1; }

    bool launch_flag;
    // #1102: which detector latched launch_flag, and what the barometer claimed
    // on that tick -- read at INFLIGHT entry for the log.  An AccelOnly latch
    // with launch_baro_healthy == true is the blocked-static-port signature
    // (sensor fresh and in range, altitude never moved).  Stays None when the
    // caller sets launch_flag directly (reboot-recovery restore).  BaroOnly
    // (#1108) is the IMU-stale path: a sustained barometric climb alone.
    enum class LaunchPath : uint8_t { None = 0, BaroClimb = 1, AccelOnly = 2, BaroOnly = 3 };
    LaunchPath launch_path;
    bool       launch_baro_healthy;
    bool alt_landed_flag;       // Voted master landed
    bool alt_apogee_flag;       // Test 2: baro altitude decreasing
    bool vel_u_apogee_flag;     // Test 1: EKF velocity negative
    bool gps_apogee_flag;       // Test 3: GPS altitude decreasing
    bool pitch_apogee_flag;     // Test 4: pitch below horizontal
    bool apogee_flag;           // Combined voted result
    bool apogee_backstop_flag;  // Layer-2 baro descent backstop fired (#257)
    // The vote by sensor: one sensor called apogee alone, with no second
    // sensor available to agree or disagree (InertialVote).
    bool apogee_single_sensor_flag;
    // Landing sub-detector flags (#166) — exposed for diagnostics/logging.
    bool impact_flag;           // High-G ground impact (one-shot latch)
    bool baro_stable_flag;      // palt low and stable
    bool gyro_quiet_flag;       // roll-rate quiescent
    bool gps_stationary_flag;   // EKF speed near zero (GPS fresh)
    bool accel_1g_flag;         // acc_mag near 1 g
    bool quiescent_flag;        // #824 sustained stillness; baro-independent
    bool baro_landed_flag;      // #574 IMU-independent; baro-only, IMU-dead only
    float max_altitude;
    float max_speed;
    float alt_est;    // Filtered altitude (m)
    float d_alt_est_; // Filtered altitude change rate (m/s)

private:

    uint16_t launch_count;
    uint16_t launch_count_hi;   // #258 sustained-high-G counter for the accel-only launch fallback
    uint16_t launch_count_baro; // #1108 sustained-climb counter for the baro-only launch fallback
    float    launch_baro_start_alt_;  // rate-gated raw altitude when the current baro-only run began
    uint32_t landing_check_time;
    float landing_look_back_alt;
    uint32_t landing_check_dt;
    uint8_t apogee_count;          // baro apogee leaky counter
    uint8_t vel_apogee_count_;     // velocity apogee leaky counter
    uint8_t pitch_apogee_count_;   // pitch apogee leaky counter
    uint8_t backstop_descent_count_; // #257 Layer-2 descent backstop leaky counter
    uint8_t single_accel_count_;     // the vote by sensor: the accelerometer's clear descent
    uint8_t single_gps_count_;       // the vote by sensor: GNSS's clear descent
    uint16_t impact_seen_count;
    // Landing sub-detector leaky counters (#166).
    uint8_t baro_stable_count_;
    uint8_t gyro_quiet_count_;
    uint8_t gps_stationary_count_;
    uint8_t accel_1g_count_;
    uint8_t quiescent_count_;
    uint8_t baro_landed_count_;   // #574

    // GPS apogee test state
    float max_gps_altitude_;
    uint8_t gps_apogee_count_;
    bool gps_available_;
    uint32_t last_gps_time_ms_;
    uint32_t gps_apogee_fresh_ms_ = 500;   // = GPS_APOGEE_FRESH_MS (the .cpp); setGpsApogeeFreshMs
    uint8_t  gps_apogee_count_hi_ = 4;     // = GPS_APOGEE_COUNT_HI (the .cpp); setGpsApogeeCountHi

    // Baro settle window after burnout (7/05 V2 F1): thrust tail-off can snap
    // the bay pressure back from its boost-suction offset, dropping indicated
    // altitude >10 m in ~0.25 s while still climbing — which satisfies the
    // baro apogee test the instant its burnout gate opens.  The baro voter is
    // ignored until BARO_BURNOUT_SETTLE_MS after burnout is first seen
    // (apogee physically cannot be at burnout).
    bool     burnout_seen_;
    uint32_t burnout_seen_ms_;

    // Baro rate-gate state (rejects ejection / sensor spikes before they
    // reach the KF or the landing fast path).
    bool     baro_gate_init_;
    float    palt_accepted_;
    uint32_t last_baro_gate_ms_;
    uint8_t  consec_baro_rejects_;
    
    // 1D CV Kalman filter for altitude & rate
    bool kf_init_;

    // 2x2 covariance (row-major): [P00 P01; P10 P11]
    float P00_, P01_, P10_, P11_;

    // Tunables
    float Qz_;    // process noise (altitude random walk), m^2/s
    float Qv_;    // process noise (acceleration), (m/s)^2
    float R_;     // measurement noise variance (altitude), m^2

    uint32_t last_ms_; // to compute dt

    float updateAltKF(float z_meas, bool new_measurement);

};

#endif