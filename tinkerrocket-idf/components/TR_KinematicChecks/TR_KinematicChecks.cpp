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

// Baro rate-gate (#166): rejects |Δpalt|/dt above any plausible rocket
// dynamic. Self-scales with dt so legitimate large changes after a sensor
// dropout still pass.  Ejection spikes are O(10-100 km/s), max plausible
// rocket dynamic is ~200 m/s, so 300 m/s gives wide headroom both ways.
constexpr float    MAX_BARO_RATE_MS         = 300.0f;
// After this many consecutive rejects the gate accepts the next sample
// unconditionally and reseeds, preventing a permanently wedged sensor
// from blinding the detector for the rest of the flight.
constexpr uint8_t  MAX_CONSEC_BARO_REJECTS  = 10;

// Apogee sub-detector leaky-counter thresholds (#166).
// Each test runs +1 on pass / -1 on miss (clamped) so a single false
// positive can un-vote once conditions reverse, instead of latching for
// the rest of the flight. HI thresholds preserve original first-fire
// timing: ~6 ms at 1 kHz for baro/pitch/vel, ~800 ms at 5 Hz for GPS.
constexpr uint8_t  APOGEE_COUNT_MAX         = 10;
constexpr uint8_t  APOGEE_COUNT_HI          = 6;
constexpr uint8_t  GPS_APOGEE_COUNT_MAX     = 8;
constexpr uint8_t  GPS_APOGEE_COUNT_HI      = 4;
// GPS apogee fires on sustained Doppler descent faster than this (m/s).  GPS
// *velocity* is real-time; GPS *position* lags ~4 s and dives during boost
// (#237/#242), so the old altitude-below-peak test fired wildly off.
constexpr float    GPS_VEL_APOGEE_DESCENT_MPS = 1.0f;

// Landing slow-detector hysteresis (#166). Slow detectors evaluate inside
// the 1 Hz landing_check_dt gate, so HI=4 → 4 s to fire (matches the
// original ``landing_checks > 4`` first-fire timing).
constexpr uint8_t  LANDING_SLOW_COUNT_MAX   = 8;
constexpr uint8_t  LANDING_SLOW_COUNT_HI    = 4;

// Landing sub-detector thresholds.
// Lower bound is -50 m, not 0, because ground-pressure drift across a
// 60-90 s flight can leave a settled palt several m below the pre-launch
// reference (Roly Poly GTV 5/9 settled at palt ≈ -8 m). The rate-gate
// upstream filters deep spikes, so this is just a defensive backstop.
constexpr float    BARO_STABLE_PALT_MIN     = -50.0f;
constexpr float    BARO_STABLE_PALT_MAX     = 50.0f;
constexpr float    BARO_STABLE_DELTA_MAX    = 2.0f;     // |Δpalt| over 1 s
constexpr float    BARO_STABLE_MAX_ALT_MIN  = 15.0f;    // rocket must have flown
constexpr float    GYRO_QUIET_DPS           = 20.0f;
constexpr float    GPS_STATIONARY_SPEED_MS  = 1.0f;     // EKF speed proxy
constexpr float    ACCEL_1G_TOLERANCE_MS2   = 2.94f;    // ≈ 0.3 g
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
    vel_apogee_count_ = 0;
    pitch_apogee_count_ = 0;
    impact_seen_count = 0;
    impact_flag = false;
    baro_stable_flag = false;
    gyro_quiet_flag = false;
    gps_stationary_flag = false;
    accel_1g_flag = false;
    baro_stable_count_ = 0;
    gyro_quiet_count_ = 0;
    gps_stationary_count_ = 0;
    accel_1g_count_ = 0;
    max_gps_altitude_ = 0.0f;
    gps_apogee_count_ = 0;
    gps_available_ = false;
    last_gps_time_ms_ = 0;
    baro_gate_init_ = false;
    palt_accepted_ = 0.0f;
    last_baro_gate_ms_ = 0;
    consec_baro_rejects_ = 0;

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
    vel_apogee_count_ = 0;
    pitch_apogee_count_ = 0;
    impact_seen_count = 0;
    impact_flag = false;
    baro_stable_flag = false;
    gyro_quiet_flag = false;
    gps_stationary_flag = false;
    accel_1g_flag = false;
    baro_stable_count_ = 0;
    gyro_quiet_count_ = 0;
    gps_stationary_count_ = 0;
    accel_1g_count_ = 0;
    max_gps_altitude_ = 0.0f;
    gps_apogee_count_ = 0;
    gps_available_ = false;
    last_gps_time_ms_ = 0;
    baro_gate_init_ = false;
    palt_accepted_ = 0.0f;
    last_baro_gate_ms_ = 0;
    consec_baro_rejects_ = 0;
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
                                         bool  baro_locked_out,
                                         float gps_vel_u)
{
    // Snapshot apogee_flag so the rising-edge reset below sees the
    // state *before* this tick's apogee voting fires (#192).
    const bool apogee_was_set = apogee_flag;

    // ### Baro rate-gate (#166) ###
    // Reject obvious spikes (ejection charges, sensor glitches) before they
    // poison the KF or trip the landing fast path.  The rate test self-scales
    // with dt so a single missed sample doesn't make the next legitimate
    // reading look like a spike.  After MAX_CONSEC_BARO_REJECTS in a row the
    // next sample is accepted unconditionally so a wedged sensor reseeds.
    if (new_baro)
    {
        if (baro_gate_init_)
        {
            uint32_t dms = millis() - last_baro_gate_ms_;
            float dt = (dms < 1u) ? 0.001f : (dms * 0.001f);
            float rate = fabs(pressure_altitude - palt_accepted_) / dt;
            if (rate > MAX_BARO_RATE_MS &&
                consec_baro_rejects_ < MAX_CONSEC_BARO_REJECTS)
            {
                consec_baro_rejects_++;
                new_baro = false;
                pressure_altitude = palt_accepted_;
            }
            else
            {
                consec_baro_rejects_ = 0;
                palt_accepted_ = pressure_altitude;
                last_baro_gate_ms_ = millis();
            }
        }
        else
        {
            palt_accepted_ = pressure_altitude;
            last_baro_gate_ms_ = millis();
            baro_gate_init_ = true;
        }
    }
    else if (baro_gate_init_)
    {
        pressure_altitude = palt_accepted_;
    }

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

    // Maximum achieved altitude (KF-smoothed — raw P_alt during boost saw
    // ±15 m sample-to-sample spikes on GTV 67mm 2026-05-09 that the previous
    // 50 m spike-reject window let through, ratcheting max_altitude above the
    // true climb and tripping the baro apogee detector during ascent — #142).
    // First reading always accepted.
    if (max_altitude == 0.0f || fabs(alt_est - max_altitude) < 50.0f) {
        max_altitude = std::max(max_altitude, alt_est);
    }

    // ================================================================
    // ### Landing Detection: 5 sub-detectors + N-1 voting (#166) ###
    // Fast path (impact) fires master immediately on unambiguous high-G
    // ground contact.  Slow path requires 3 of the 4 settle-state
    // detectors (Baro-stable, Gyro-quiet, GPS-stationary, Accel-1g) to
    // agree — independent signals so a single noise source can't drive
    // a false landed.  Master ``alt_landed_flag`` latches once true.
    // ================================================================

    // --- Fast path: high-G impact, gated on apogee + ground proximity.
    // The baro rate-gate upstream already filters glitch spikes, but the
    // defensive ``palt > -10`` lower bound mirrors the gate's intent so
    // this path is robust even if the gate is bypassed in future.
    // One-shot latch — impact_flag stays true once a real impact fires.
    if (apogee_flag
     && pressure_altitude > -10.0f
     && pressure_altitude < LANDING_IMPACT_ALT_M
     && acc_mag > LANDING_IMPACT_G * G_MS2)
    {
        impact_seen_count++;
        if (impact_seen_count >= LANDING_IMPACT_COUNT)
        {
            impact_flag = true;
        }
    }
    else
    {
        impact_seen_count = 0;
    }

    // --- Slow detectors: evaluated at 1 Hz inside the existing time gate.
    if (millis() > landing_check_time + landing_check_dt)
    {
        float landing_altitude_change = fabs(pressure_altitude - landing_look_back_alt);
        landing_look_back_alt = pressure_altitude;
        landing_check_time = millis();

        // Sub 1: Baro-stable (was the original slow path; +lower bound)
        const bool baro_stable_pass = (pressure_altitude >= BARO_STABLE_PALT_MIN
                                       && pressure_altitude <= BARO_STABLE_PALT_MAX
                                       && landing_altitude_change < BARO_STABLE_DELTA_MAX
                                       && max_altitude > BARO_STABLE_MAX_ALT_MIN);
        if (baro_stable_pass) {
            if (baro_stable_count_ < LANDING_SLOW_COUNT_MAX) baro_stable_count_++;
        } else {
            if (baro_stable_count_ > 0) baro_stable_count_--;
        }
        if (baro_stable_count_ >= LANDING_SLOW_COUNT_HI) baro_stable_flag = true;
        else if (baro_stable_count_ == 0)                baro_stable_flag = false;

        // Sub 2: Gyro-quiet — roll_rate (future: generalize to all axes).
        const bool gyro_quiet_pass = (fabs(roll_rate) < GYRO_QUIET_DPS);
        if (gyro_quiet_pass) {
            if (gyro_quiet_count_ < LANDING_SLOW_COUNT_MAX) gyro_quiet_count_++;
        } else {
            if (gyro_quiet_count_ > 0) gyro_quiet_count_--;
        }
        if (gyro_quiet_count_ >= LANDING_SLOW_COUNT_HI) gyro_quiet_flag = true;
        else if (gyro_quiet_count_ == 0)                gyro_quiet_flag = false;

        // Sub 3: GPS-stationary — EKF speed proxy; excluded if GPS stale.
        if (gps_available_ && (millis() - last_gps_time_ms_) < 5000)
        {
            const float speed = sqrt(velocity[0]*velocity[0] +
                                     velocity[1]*velocity[1] +
                                     velocity[2]*velocity[2]);
            const bool gps_stationary_pass = (speed < GPS_STATIONARY_SPEED_MS);
            if (gps_stationary_pass) {
                if (gps_stationary_count_ < LANDING_SLOW_COUNT_MAX) gps_stationary_count_++;
            } else {
                if (gps_stationary_count_ > 0) gps_stationary_count_--;
            }
            if (gps_stationary_count_ >= LANDING_SLOW_COUNT_HI) gps_stationary_flag = true;
            else if (gps_stationary_count_ == 0)                gps_stationary_flag = false;
        }

        // Sub 4: Accel-1g floor.
        const bool accel_1g_pass = (fabs(acc_mag - G_MS2) < ACCEL_1G_TOLERANCE_MS2);
        if (accel_1g_pass) {
            if (accel_1g_count_ < LANDING_SLOW_COUNT_MAX) accel_1g_count_++;
        } else {
            if (accel_1g_count_ > 0) accel_1g_count_--;
        }
        if (accel_1g_count_ >= LANDING_SLOW_COUNT_HI) accel_1g_flag = true;
        else if (accel_1g_count_ == 0)                accel_1g_flag = false;
    }

    // --- Voting: impact alone fires; otherwise N-1 of N over slow flags.
    // Gated on apogee_flag — a static rocket on the pad trips gyro_quiet,
    // gps_stationary, and accel_1g (3 of 4 votes), which would otherwise
    // fire landed pre-flight (caught on bench 2026-05-18).
    if (!alt_landed_flag && apogee_flag)
    {
        if (impact_flag)
        {
            alt_landed_flag = true;
        }
        else
        {
            uint8_t available = 0;
            uint8_t passed = 0;

            available++; if (baro_stable_flag) passed++;
            available++; if (gyro_quiet_flag)  passed++;
            if (gps_available_ && (millis() - last_gps_time_ms_) < 5000)
            {
                available++; if (gps_stationary_flag) passed++;
            }
            available++; if (accel_1g_flag) passed++;

            if (available >= 2 && passed >= 2 && passed >= (available - 1))
            {
                alt_landed_flag = true;
            }
        }
    }

    // ### GPS freshness tracking ###
    // Mark GPS available + stamp the last fix for the voting staleness gate.
    // max_gps_altitude_ is still tracked (cheap, kept for diagnostics) but the
    // GPS apogee test no longer uses it — it fires on Doppler velocity now
    // (#237/#242), which is real-time, unlike the ~4 s-lagging GPS position.
    if (new_gps && launch_flag)
    {
        gps_available_ = true;
        last_gps_time_ms_ = millis();
        max_gps_altitude_ = fmax(max_gps_altitude_, gps_altitude);
    }

    // ================================================================
    // ### Apogee Detection (gated on launch + burnout) ###
    // Four independent tests, each a leaky counter with hysteresis so a
    // single bad sample (EKF wobble, baro glitch, momentary attitude
    // excursion) doesn't latch a vote for the rest of the flight (#166).
    // N-1 of N voting on the sub-flags; master apogee_flag still latches
    // once true.
    // ================================================================
    if (launch_flag && burnout_detected)
    {
        // --- Test 1: Velocity (EKF vertical velocity negative) ---
        const bool vel_pass = (position[2] > 15.0f && velocity[2] < 0.0f);
        if (vel_pass) {
            if (vel_apogee_count_ < APOGEE_COUNT_MAX) vel_apogee_count_++;
        } else {
            if (vel_apogee_count_ > 0) vel_apogee_count_--;
        }
        if (vel_apogee_count_ >= APOGEE_COUNT_HI)      vel_u_apogee_flag = true;
        else if (vel_apogee_count_ == 0)               vel_u_apogee_flag = false;

        // --- Test 2: Baro altitude (filtered altitude decreasing) ---
        // Uses KF-smoothed alt_est rather than raw pressure_altitude — boost
        // noise on the raw signal was previously tripping this test during
        // ascent (#142). Velocity gate (d_alt_est_ < 20 m/s) is preserved.
        const bool baro_pass = (alt_est > 15.0f &&
                                alt_est < max_altitude - 5.0f &&
                                d_alt_est_ < 20.0f);
        if (baro_pass) {
            if (apogee_count < APOGEE_COUNT_MAX) apogee_count++;
        } else {
            if (apogee_count > 0) apogee_count--;
        }
        if (apogee_count >= APOGEE_COUNT_HI)            alt_apogee_flag = true;
        else if (apogee_count == 0)                     alt_apogee_flag = false;

        // --- Test 3: GPS Doppler vertical velocity (descending) ---
        // Was "GPS altitude below running max", but GPS *position* lags ~4 s
        // and dives during boost (#242) — firing this wildly off (III at
        // T+3.6 s, II at T+9.9 s vs true apogee #237).  GPS *velocity*
        // (Doppler) is real-time, so a sustained descent tracks true apogee.
        // Gated only on a fresh fix — NO gps-altitude gate, since that lag is
        // the bug.  Post-burnout only, so an ascent excursion can't fire it.
        if (new_gps && gps_available_)
        {
            const bool gps_pass = (gps_vel_u < -GPS_VEL_APOGEE_DESCENT_MPS);
            if (gps_pass) {
                if (gps_apogee_count_ < GPS_APOGEE_COUNT_MAX) gps_apogee_count_++;
            } else {
                if (gps_apogee_count_ > 0) gps_apogee_count_--;
            }
            if (gps_apogee_count_ >= GPS_APOGEE_COUNT_HI) gps_apogee_flag = true;
            else if (gps_apogee_count_ == 0)              gps_apogee_flag = false;
        }

        // --- Test 4: Pitch below horizontal ---
        // NED convention: +90° = nose up, 0° = horizontal, negative = past horizontal.
        // -5° threshold avoids false trigger from coast oscillation.
        const bool pitch_pass = (pitch_rad < -0.087f);
        if (pitch_pass) {
            if (pitch_apogee_count_ < APOGEE_COUNT_MAX) pitch_apogee_count_++;
        } else {
            if (pitch_apogee_count_ > 0) pitch_apogee_count_--;
        }
        if (pitch_apogee_count_ >= APOGEE_COUNT_HI)     pitch_apogee_flag = true;
        else if (pitch_apogee_count_ == 0)              pitch_apogee_flag = false;

        // --- Dynamic N-1 of N voting (vel + baro + pitch; GPS excluded #237) ---
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

            // GPS — NOT a voter (#237/#242).  GPS apogee is unreliable during
            // boost on this hardware (RP III: position AND Doppler velocity
            // corrupted — vel_u noise to -38 m/s while climbing — at fix=3 with
            // good reported accuracy), so it false-fires.  gps_apogee_flag is
            // still computed + logged as a diagnostic, just excluded from the
            // vote so it can't move pyro timing.

            // Pitch — always available
            available++;
            if (pitch_apogee_flag) passed++;

            // N-1 of N with minimum 2 confirmations required.
            // 3 available → need 2.  2 available (mach lockout) → need 2.
            if (available >= 2 && passed >= 2 && passed >= (available - 1))
            {
                apogee_flag = true;
            }
        }
    } // end burnout gate

    // ### Apogee rising edge: reset landing sub-flag counters (#192) ###
    // The 1 Hz sub-detectors above accumulate evidence regardless of
    // flight state, so a rocket flying straight pre-apogee (low roll
    // rate, ~0 m/s in EKF frame, ~1g during coast) latches gyro_quiet /
    // gps_stationary / accel_1g before apogee.  The apogee gate on the
    // master vote prevents a false LANDED, but the latched sub-flags
    // would carry pre-apogee history into the post-apogee vote and erode
    // the 3-of-4 margin.  Zero counters + flags at the transition so
    // post-apogee voting reflects only post-apogee evidence.
    if (apogee_flag && !apogee_was_set)
    {
        baro_stable_count_    = 0;
        gyro_quiet_count_     = 0;
        gps_stationary_count_ = 0;
        accel_1g_count_       = 0;
        baro_stable_flag    = false;
        gyro_quiet_flag     = false;
        gps_stationary_flag = false;
        accel_1g_flag       = false;
    }

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

