#ifndef TRKINEMATICCHECKS_H
#define TRKINEMATICCHECKS_H

#include <Arduino.h>

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
                         bool new_baro = true);
    
    bool launch_flag;
    bool alt_landed_flag;
    bool alt_apogee_flag;
    bool vel_u_apogee_flag;
    float max_altitude;
    float max_speed;
    float alt_est;    // Filtered altitude (m)
    float d_alt_est_; // Filtered altitude change rate (m/s)

private:
    
    uint16_t launch_count;
    uint32_t landing_check_time;
    float landing_look_back_alt;
    uint32_t landing_check_dt;
    uint8_t apogee_count;
    uint16_t landing_checks;
    
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