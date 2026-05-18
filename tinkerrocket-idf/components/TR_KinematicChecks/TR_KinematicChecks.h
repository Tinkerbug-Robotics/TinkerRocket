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
                         bool  baro_locked_out = false);

    bool launch_flag;
    bool alt_landed_flag;       // Voted master landed
    bool alt_apogee_flag;       // Test 2: baro altitude decreasing
    bool vel_u_apogee_flag;     // Test 1: EKF velocity negative
    bool gps_apogee_flag;       // Test 3: GPS altitude decreasing
    bool pitch_apogee_flag;     // Test 4: pitch below horizontal
    bool apogee_flag;           // Combined voted result
    // Landing sub-detector flags (#166) — exposed for diagnostics/logging.
    bool impact_flag;           // High-G ground impact (one-shot latch)
    bool baro_stable_flag;      // palt low and stable
    bool gyro_quiet_flag;       // roll-rate quiescent
    bool gps_stationary_flag;   // EKF speed near zero (GPS fresh)
    bool accel_1g_flag;         // acc_mag near 1 g
    float max_altitude;
    float max_speed;
    float alt_est;    // Filtered altitude (m)
    float d_alt_est_; // Filtered altitude change rate (m/s)

private:

    uint16_t launch_count;
    uint32_t landing_check_time;
    float landing_look_back_alt;
    uint32_t landing_check_dt;
    uint8_t apogee_count;          // baro apogee leaky counter
    uint8_t vel_apogee_count_;     // velocity apogee leaky counter
    uint8_t pitch_apogee_count_;   // pitch apogee leaky counter
    uint16_t impact_seen_count;
    // Landing sub-detector leaky counters (#166).
    uint8_t baro_stable_count_;
    uint8_t gyro_quiet_count_;
    uint8_t gps_stationary_count_;
    uint8_t accel_1g_count_;

    // GPS apogee test state
    float max_gps_altitude_;
    uint8_t gps_apogee_count_;
    bool gps_available_;
    uint32_t last_gps_time_ms_;

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