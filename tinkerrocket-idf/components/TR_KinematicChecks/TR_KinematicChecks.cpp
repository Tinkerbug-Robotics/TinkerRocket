#include <TR_KinematicChecks.h>
#include <algorithm>

namespace {
// Impact-detected landing fast path (see issue #113).
// Descent tumbling never exceeds ~12 g; impact on RolyPoly 5/3/26 peaked at
// 218 g. 15 g cleanly separates them with no false positives in descent.
// Gated on apogee + low altitude so launch (~7 g) and ejection (~22 g at
// apogee) cannot trigger.
constexpr float    LANDING_IMPACT_G       = 15.0f;
constexpr float    LANDING_IMPACT_ALT_M   = 20.0f;
constexpr uint16_t LANDING_IMPACT_COUNT   = 5;       // ~5 ms at 1 kHz
constexpr float    G_MS2                  = 9.80665f;
}

TR_KinematicChecks::TR_KinematicChecks()
{
    launch_flag = false;
    alt_landed_flag = false;
    alt_apogee_flag = false;
    vel_u_apogee_flag = false;
    gps_apogee_flag = false;
    pitch_apogee_flag = false;
    apogee_flag = false;
    launch_count = 0;
    max_altitude = 0.0f;
    max_speed = 0.0f;
    landing_check_time = 0;
    landing_look_back_alt = 0.0;
    landing_check_dt = 1000;
    apogee_count = 0;
    landing_checks = 0;
    impact_seen_count = 0;
    max_gps_altitude_ = 0.0f;
    gps_apogee_count_ = 0;
    gps_available_ = false;
    last_gps_time_ms_ = 0;

    // KF init
    kf_init_ = false;
    alt_est = 0.0f;
    d_alt_est_ = 0.0f;
    P00_ = 10.0f;  P01_ = 0.0f;
    P10_ = 0.0f;   P11_ = 10.0f;

    // Tunable altitude filter parameters
    // R_ ~ altitude measurement variance; BMP585 noise is ~0.17m, but vibration
    // and barometric formula error add noise, so we use a conservative 1 m^2.
    R_  = 1.0f; // m^2
    // Qz_: slow random drift in altitude (bias/random walk)
    Qz_ = 0.05f; // m^2/s
    // Qv_: allows acceleration to change vertical speed; larger → more responsive rate.
    // Rockets experience high accelerations (>20g), so this needs to be large enough
    // to let the velocity estimate track rapid changes.
    Qv_ = 5.0f; // (m/s)^2
}

void TR_KinematicChecks::reset()
{
    launch_flag = false;
    alt_landed_flag = false;
    alt_apogee_flag = false;
    vel_u_apogee_flag = false;
    gps_apogee_flag = false;
    pitch_apogee_flag = false;
    apogee_flag = false;
    launch_count = 0;
    max_altitude = 0.0f;
    max_speed = 0.0f;
    landing_check_time = 0;
    landing_look_back_alt = 0.0;
    apogee_count = 0;
    landing_checks = 0;
    impact_seen_count = 0;
    max_gps_altitude_ = 0.0f;
    gps_apogee_count_ = 0;
    gps_available_ = false;
    last_gps_time_ms_ = 0;
    kf_init_ = false;
    alt_est = 0.0f;
    d_alt_est_ = 0.0f;
    P00_ = 10.0f;  P01_ = 0.0f;
    P10_ = 0.0f;   P11_ = 10.0f;
}

void TR_KinematicChecks::kinematicChecks(float pressure_altitude,
                                         float acc_mag,
                                         float position[3],
                                         float velocity[3],
                                         float roll_rate,
                                         bool  new_baro,
                                         float gps_altitude,
                                         bool  new_gps,
                                         float pitch_rad,
                                         bool  burnout_detected,
                                         bool  baro_locked_out)
{

    // ### Altitude ###

    // Kalman-filter altitude and altitude rate.
    // Predict step runs every call; update step only when new_baro is true
    // to avoid double-counting stale measurements.
    // Runs before launch check so d_alt_est_ is current for altitude rate confirmation.
    updateAltKF(pressure_altitude, new_baro);

    // ### Check for Launch Conditions ###

    // 50 consecutive readings of >20 m/s^2 (~42ms at 1200 Hz) AND
    // Kalman-filtered altitude rate > 1.0 m/s (confirms actual upward motion).
    // This rejects handling bumps which produce short accel spikes but no altitude change.
    if (launch_flag == false)
    {
        if (acc_mag > 20.0)
        {
            launch_count++;
            if (launch_count > 50 && d_alt_est_ > 1.0)
            {
                launch_flag = true;
            }
        }
        else
        {
            launch_count = 0;
        }
    }

    // Maximum achieved altitude (use raw measurement — not affected by filter lag).
    // Spike rejection: only accept readings within 50 m of current max to prevent
    // single-sample noise from ratcheting up the value.  First reading always accepted.
    if (max_altitude == 0.0f || fabs(pressure_altitude - max_altitude) < 50.0f) {
        max_altitude = std::max(max_altitude, pressure_altitude);
    }

    // ### Check for Landing ###

    if (millis() > landing_check_time + landing_check_dt)
    {
        float landing_altitude_change = fabs(pressure_altitude - landing_look_back_alt);
        landing_look_back_alt = pressure_altitude;
        landing_check_time = millis();

        // Checking landing declaration criteria.
        // roll_rate threshold: 20 dps tolerates wind-induced body roll on a
        // landed rocket (RolyPoly 5/3/26 saw 5-22 dps wobbles post-touchdown);
        // anything below 50 dps is clearly not in flight.
        if (pressure_altitude < 50.0      // Low current altitude
         && landing_altitude_change < 2.0 // Less than 2 m change
         && max_altitude > 15.0           // Max altitude was more than 15 m
         && fabs(roll_rate) < 20.0)       // Gyro is stable (no longer in flight)
        {
            landing_checks++;
        }
        else
        {
            landing_checks = 0;
        }
        // Once the criteria is true for multiple times in a row
        // indicate we have landed (5 consecutive 1-second checks = 5s)
        if (landing_checks > 4)
        {
            alt_landed_flag = true;
        }
    }

    // ### Impact-detected fast path ###
    // High-energy ground impact is unambiguous: descent tumbling tops out
    // around 12 g, and a hard landing is several hundred g for ~100 ms.
    // This runs every call (gyro rate, ~1 kHz) so we don't have to wait
    // 5 s for the slow path to accumulate when impact already gave us a
    // definitive ground signal. See issue #113.
    if (apogee_flag
     && pressure_altitude < LANDING_IMPACT_ALT_M
     && acc_mag > LANDING_IMPACT_G * G_MS2)
    {
        impact_seen_count++;
        if (impact_seen_count >= LANDING_IMPACT_COUNT)
        {
            alt_landed_flag = true;
        }
    }
    else
    {
        impact_seen_count = 0;
    }

    // ### GPS altitude tracking ###
    // Track max GPS altitude from launch onwards (raw MSL — relative
    // tracking means absolute value doesn't matter).
    if (new_gps && launch_flag)
    {
        gps_available_ = true;
        last_gps_time_ms_ = millis();

        // Spike rejection: 100m window (wider than baro's 50m for GPS noise)
        if (max_gps_altitude_ == 0.0f ||
            fabs(gps_altitude - max_gps_altitude_) < 100.0f) {
            max_gps_altitude_ = fmax(max_gps_altitude_, gps_altitude);
        }
    }

    // ================================================================
    // ### Apogee Detection (gated on burnout) ###
    // Four independent tests with N-1 of N voting.
    // No apogee flag can latch before motor burnout is confirmed.
    // ================================================================
    if (burnout_detected)
    {
        // --- Test 1: Velocity (EKF vertical velocity negative) ---
        if (launch_flag && position[2] > 15.0f && velocity[2] < 0.0f)
        {
            vel_u_apogee_flag = true;
        }

        // --- Test 2: Baro altitude (pressure altitude decreasing) ---
        if (pressure_altitude > 15.0f &&
            pressure_altitude < max_altitude - 5.0f)
        {
            apogee_count++;
        }
        else
        {
            apogee_count = 0;
        }
        if (apogee_count > 5)
        {
            alt_apogee_flag = true;
        }

        // --- Test 3: GPS altitude (GPS altitude decreasing) ---
        if (new_gps && gps_available_ && gps_altitude > 15.0f)
        {
            if (gps_altitude < max_gps_altitude_ - 10.0f)
            {
                gps_apogee_count_++;
            }
            else
            {
                gps_apogee_count_ = 0;
            }
            if (gps_apogee_count_ > 3)
            {
                gps_apogee_flag = true;
            }
        }

        // --- Test 4: Pitch below horizontal ---
        // NED convention: +90° = nose up, 0° = horizontal, negative = past horizontal.
        // -5° threshold avoids false trigger from coast oscillation.
        if (pitch_rad < -0.087f)
        {
            pitch_apogee_flag = true;
        }

        // --- Dynamic N-1 of N voting ---
        if (!apogee_flag)
        {
            uint8_t available = 0;
            uint8_t passed = 0;

            // Velocity — always available
            available++;
            if (vel_u_apogee_flag) passed++;

            // Baro — excluded during mach lockout
            if (!baro_locked_out)
            {
                available++;
                if (alt_apogee_flag) passed++;
            }

            // GPS — excluded if stale (>5s since last fix)
            if (gps_available_ &&
                (millis() - last_gps_time_ms_ < 5000))
            {
                available++;
                if (gps_apogee_flag) passed++;
            }

            // Pitch — always available
            available++;
            if (pitch_apogee_flag) passed++;

            // N-1 of N with minimum 2 confirmations required.
            // 4 available → need 3.  3 available → need 2.  2 available → need 2.
            if (available >= 2 && passed >= 2 && passed >= (available - 1))
            {
                apogee_flag = true;
            }
        }
    } // end burnout gate

    // ### Pre-Apogee Max Speed ###
    float speed = sqrt(velocity[0]*velocity[0]+
                       velocity[1]*velocity[1]+
                       velocity[2]*velocity[2]);
    if (!apogee_flag && speed > max_speed)
    {
        max_speed = speed;
    }

}

// ======================================================================
// 1D Constant-Velocity Kalman Filter
//
// State: x = [ z, vz ]^T
// Model: z_k+1 = z_k + vz_k*dt + w_z
//        vz_k+1 = vz_k + w_v
// Process noise: Q = [[Qz*dt, 0],[0, Qv*dt]]
// Measurement: y = z + v,  R = R_
//
// The predict step runs every call to propagate the state forward.
// The update step only runs when new_measurement is true, so that
// stale (repeated) baro readings are not double-counted.
//
// Returns filtered altitude; updates d_alt_est_ (altitude rate).
// ======================================================================
float TR_KinematicChecks::updateAltKF(float z_meas, bool new_measurement)
{
    uint32_t now = millis();
    float dt = 0.02f; // default 20 ms if first call
    if (kf_init_) {
        uint32_t dms = now - last_ms_;
        // clamp dt between 0.5 ms and 200 ms to avoid numeric extremes
        dt = constrain(dms * 0.001f, 0.0005f, 0.200f);
    }
    last_ms_ = now;

    if (!kf_init_)
    {
        // Initialize state to first measurement
        alt_est = z_meas;
        d_alt_est_ = 0.0f;
        // Covariance already set in ctor
        kf_init_ = true;
        return alt_est;
    }

    // ---- Predict ----
    // x = F x
    // F = [[1, dt],
    //      [0, 1]]
    float z_pred  = alt_est + d_alt_est_ * dt;
    float vz_pred = d_alt_est_;

    // P = F P F^T + Q
    const float F00 = 1.0f, F01 = dt;
    const float F10 = 0.0f, F11 = 1.0f;

    float P00p = F00*P00_ + F01*P10_;
    float P01p = F00*P01_ + F01*P11_;
    float P10p = F10*P00_ + F11*P10_;
    float P11p = F10*P01_ + F11*P11_;

    float P00pp = P00p*F00 + P01p*F01;
    float P01pp = P00p*F10 + P01p*F11;
    float P10pp = P10p*F00 + P11p*F01;
    float P11pp = P10p*F10 + P11p*F11;

    // Q (scaled by dt): allow slow drift in z and change in vz
    float Q00 = Qz_ * dt;
    float Q11 = Qv_ * dt;

    P00pp += Q00;
    P11pp += Q11;

    if (new_measurement)
    {
        // ---- Update (measure z) ----
        // y = H x + v, H = [1 0]
        // S = H P H^T + R = P00pp + R
        float S  = P00pp + R_;
        float K0 = P00pp / S;   // gain for z
        float K1 = P10pp / S;   // gain for vz

        float innov = z_meas - z_pred;

        // x = x + K * innov
        alt_est    = z_pred  + K0 * innov;
        d_alt_est_ = vz_pred + K1 * innov;

        // P = (I - K H) P
        // (I - K H) = [[1-K0,   0],
        //              [-K1 ,   1]]
        float P00n = (1.0f - K0) * P00pp;
        float P01n = (1.0f - K0) * P01pp;
        float P10n = P10pp - K1 * P00pp;
        float P11n = P11pp - K1 * P01pp;

        P00_ = P00n;
        P11_ = P11n;
        // Enforce covariance symmetry to prevent float drift over many iterations
        P01_ = 0.5f * (P01n + P10n);
        P10_ = P01_;
    }
    else
    {
        // No new measurement — keep predicted state and covariance
        alt_est    = z_pred;
        d_alt_est_ = vz_pred;

        P00_ = P00pp;
        P11_ = P11pp;
        P01_ = 0.5f * (P01pp + P10pp);
        P10_ = P01_;
    }

    return alt_est;
}

