//
//  TR_Sensor_Collector_Sim.cpp
//  TinkerRocket FlightComputer
//
//  Drop-in wrapper around SensorCollector.  When sim is active, consumes
//  real sensor data (maintaining hardware timing) and replaces output
//  values with simulated 1D flight data.
//
//  Physics model adapted from OutComputer/FlightSimulator.h.
//

#include "TR_Sensor_Collector_Sim.h"
#include "SimPadAlign.h"
#include "SimSensorModel.h"
#include <cmath>
// SensorCollector.h no longer transitively pulls in compat.h, but this
// file still uses the Arduino-shim millis()/micros() heavily.  Pulled
// in explicitly here until item #5 of issue #21 slims the sim too.
#include <compat.h>
// The `Serial` object this file prints through is the SparkFun shim in the
// u-blox driver (sfe_bus.h). It used to arrive transitively through
// SensorCollector.h; on boards where the collector selects the LC86 GNSS
// driver instead (TR_GNSS_DRIVER_LC86) that path is gone, so include it
// explicitly. The shim is compiled unconditionally, so this is
// declaration-only on every board.
#include <sfe_bus.h>

// ============================================================================
// Construction / Delegation
// ============================================================================

SensorCollectorSim::SensorCollectorSim(SensorCollector& real)
    : real_(real) {}

void SensorCollectorSim::begin(uint8_t imu_execution_core)
{
    real_.begin(imu_execution_core);
}

// ============================================================================
// Debug passthrough
// ============================================================================

void SensorCollectorSim::getISM6HG256DebugSnapshot(ISM6HG256DebugSnapshot& snapshot_out) const
{
    real_.getISM6HG256DebugSnapshot(snapshot_out);
}

void SensorCollectorSim::getMMC5983MADebugSnapshot(MMC5983MADebugSnapshot& snapshot_out) const
{
    real_.getMMC5983MADebugSnapshot(snapshot_out);
}

void SensorCollectorSim::getPollTimingSnapshot(PollTimingSnapshot& snapshot_out) const
{
    real_.getPollTimingSnapshot(snapshot_out);
}

void SensorCollectorSim::resetPollTimingSnapshot()
{
    real_.resetPollTimingSnapshot();
}

// ============================================================================
// Calibration (no-op — sim data is synthetic, no real bias to correct)
// ============================================================================

void SensorCollectorSim::calibrateGyro(float rotation_z_deg)
{
    if (isSimActive())
    {
        Serial.println("[SIM] calibrateGyro called — no-op in sim mode");
        return;
    }
    // Sim not active — delegate to real hardware
    real_.calibrateGyro(rotation_z_deg);
    // Copy results so FlightComputer can read them from the wrapper
    hg_bias_x       = real_.hg_bias_x;
    hg_bias_y       = real_.hg_bias_y;
    hg_bias_z       = real_.hg_bias_z;
    cal_gravity_mag  = real_.cal_gravity_mag;
}

bool SensorCollectorSim::setIIS2MDCHardIronOffset(int16_t cx, int16_t cy, int16_t cz)
{
    if (isSimActive())
    {
        // No physical IIS2MDC interaction in sim mode; cal flow is gated on
        // READY and we don't enter sim from there, so this branch is mostly
        // defensive.
        return false;
    }
    return real_.setIIS2MDCHardIronOffset(cx, cy, cz);
}

// ============================================================================
// Sim Control
// ============================================================================

void SensorCollectorSim::configureSim(const SimConfigData& cfg)
{
    cfg_ = cfg;
    configured_ = true;
    // mass_kg outside any plausible vehicle (>50 kg / <10 g) almost certainly
    // means a raw cmd-5 BLE payload (grams) reached us without the OC's /1000
    // conversion — the sim will sit on the pad (or go ballistic) silently.
    if (cfg_.mass_kg > 50.0f || cfg_.mass_kg < 0.010f)
    {
        ESP_LOGE("SIM", "Implausible mass_kg=%.4f — grams sent where kg expected? "
                 "(BLE cmd-5 mass is grams; OC converts)", (double)cfg_.mass_kg);
    }
    ESP_LOGI("SIM", "Config: mass=%.3fkg thrust=%.1fN burn=%.1fs descent=%.1fm/s",
             (double)cfg_.mass_kg, (double)cfg_.thrust_n,
             (double)cfg_.burn_time_s, (double)cfg_.descent_rate_mps);
}

void SensorCollectorSim::configureSimRotation(float ism6_rot_z_deg)
{
    float rad = ism6_rot_z_deg * (M_PI / 180.0f);
    ism6_inv_c_ = cosf(rad);
    ism6_inv_s_ = sinf(rad);
    ESP_LOGI("SIM", "ISM6 rotation: %.1f deg (inv_c=%.4f, inv_s=%.4f)",
             (double)ism6_rot_z_deg, (double)ism6_inv_c_, (double)ism6_inv_s_);
}

void SensorCollectorSim::configureSimIis2mdcRotation(float rot_z_deg)
{
    const float rad = rot_z_deg * (float)M_PI / 180.0f;
    iis_inv_c_ = cosf(rad);
    iis_inv_s_ = sinf(rad);
    ESP_LOGI("SIM", "IIS2MDC rotation: %.1f deg", (double)rot_z_deg);
}

void SensorCollectorSim::configureSimBoardToRocket(const float R[9])
{
    // Store the transpose (rocket→board); flag identity to skip the
    // multiply in the hot encode path.
    bool identity = true;
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            const float v = R[c * 3 + r];  // transpose
            b2r_inv_[r * 3 + c] = v;
            const float ref = (r == c) ? 1.0f : 0.0f;
            if (fabsf(v - ref) > 1e-6f) identity = false;
        }
    }
    b2r_identity_ = identity;
}

void SensorCollectorSim::simPadReference(float up_out[3], float mag_out[3]) const
{
    sim_pad_align::simPadReference(LAUNCH_ANGLE_RAD, B_NORTH, B_EAST, B_DOWN,
                                   up_out, mag_out);
}

void SensorCollectorSim::configureSimPadAlignment(const float up_rocket[3],
                                                  const float mag_rocket[3])
{
    float up_sim[3], mag_sim[3];
    simPadReference(up_sim, mag_sim);

    bool used_mag = false;
    const bool ok = sim_pad_align::solvePadAlignment(
        up_sim, mag_sim, up_rocket, mag_rocket, pad_align_, &used_mag);

    if (!ok)
    {
        pad_align_identity_ = true;
        ESP_LOGW("SIM", "pad align: measured gravity unusable — sim starts "
                        "UNALIGNED (expect a gyro-bias excursion, #508)");
        return;
    }

    pad_align_identity_ = false;
    ESP_LOGI("SIM", "pad align: %.1f deg (%s) — sim frame carried onto the "
                    "board's resting attitude; no handover step",
             (double)padAlignmentAngleDeg(),
             used_mag ? "gravity+field" : "gravity only");
}

float SensorCollectorSim::padAlignmentAngleDeg() const
{
    if (pad_align_identity_) return 0.0f;
    return sim_pad_align::rotationAngleDeg(pad_align_);
}

void SensorCollectorSim::startSim(float ground_pressure_pa)
{
    if (!configured_)
    {
        Serial.println("[SIM] ERROR: startSim called but not configured (missing config cmd?)");
        return;
    }

    altitude_m_      = 0.0f;
    velocity_mps_    = 0.0f;
    accel_mps2_      = 0.0f;
    sim_time_s_      = 0.0f;
    pitch_rad_       = LAUNCH_ANGLE_RAD;
    pitch_rate_rps_  = 0.0f;

    ground_pressure_pa_ = (ground_pressure_pa > 50000.0f)
        ? ground_pressure_pa : 101325.0f;

    phase_ = SIM_PRELAUNCH;
    phase_start_ms_ = millis();
    last_step_us_   = micros();
    gnss_timer_us_  = micros();
    last_gnss_real_us_ = micros();

    Serial.println("[SIM] Started");
}

void SensorCollectorSim::stopSim()
{
    phase_ = SIM_IDLE;
    // Drop the pad alignment: it belongs to the attitude the board was resting
    // at for THAT run, and a stale one would skew the next sim start.
    pad_align_identity_ = true;
    for (int i = 0; i < 9; ++i) pad_align_[i] = (i % 4 == 0) ? 1.0f : 0.0f;
    Serial.println("[SIM] Stopped");
}

bool SensorCollectorSim::isSimActive() const
{
    return phase_ != SIM_IDLE;
}

// ============================================================================
// Data Getters
// ============================================================================

bool SensorCollectorSim::getISM6HG256Data(ISM6HG256Data& data_out)
{
    bool have_data = real_.getISM6HG256Data(data_out);

    if (phase_ == SIM_IDLE) return have_data;

    if (!have_data) return false;

    // Real IMU data arrived at the hardware rate — step physics and replace values
    const uint32_t now_us = micros();
    const float dt = (float)(now_us - last_step_us_) * 1.0e-6f;
    last_step_us_ = now_us;
    const float clamped_dt = (dt > 0.1f) ? 0.1f : dt;
    sim_time_s_ += clamped_dt;

    stepPhysics(clamped_dt);

#if defined(TR_SIM_DEAD_IMU) && TR_SIM_DEAD_IMU
    // #556 bench validation: emulate an IMU that dies at burnout.  The flight
    // physics are stepped above, so baro/GNSS keep flying the FULL trajectory
    // (ascent -> apogee -> descent); but from the boost->coast boundary on we
    // stop DELIVERING fresh IMU samples.  The main loop derives IMU freshness
    // from this getter returning true, so ism6 goes stale, burnout_detected
    // never latches, and the burnout-gated apogee vote never runs — leaving the
    // baro-only Layer-2 backstop as the ONLY path to apogee.  That is exactly
    // the dead-IMU condition #556 must survive.  The IMU stays live through
    // PRELAUNCH/POWERED so launch still latches normally (launch detection
    // requires acc_mag > 20).  Compile-guarded AND sim-only: it can never
    // affect a real flight or a normal sim run.
    if (phase_ >= SIM_COASTING)
    {
        static bool announced = false;
        if (!announced) { announced = true; Serial.println("[SIM] DEAD-IMU: withholding IMU from coast (#556)"); }
        return false;
    }
#endif

    encodeISM6(now_us, data_out);
    return true;
}

bool SensorCollectorSim::getBMP585Data(BMP585Data& data_out)
{
    bool have_data = real_.getBMP585Data(data_out);

    if (phase_ == SIM_IDLE) return have_data;

    if (!have_data) return false;

    // Replace with simulated pressure/temperature
    encodeBMP585(micros(), data_out);
    return true;
}

bool SensorCollectorSim::getMMC5983MAData(MMC5983MAData& data_out)
{
    bool have_data = real_.getMMC5983MAData(data_out);

    if (phase_ == SIM_IDLE) return have_data;

    if (!have_data) return false;

    // Replace with simulated magnetometer
    encodeMMC5983MA(micros(), data_out);
    return true;
}

bool SensorCollectorSim::getIIS2MDCData(IIS2MDCData& data_out)
{
    bool have_data = real_.getIIS2MDCData(data_out);

    if (phase_ == SIM_IDLE) return have_data;

    if (!have_data) return false;

    // Replace with simulated magnetometer (#463). This path used to pass the
    // REAL bench field through during a sim: on IIS2MDC boards (MMC not
    // populated) the EKF then fused the bench's fixed heading against the
    // sim's flying attitude every sample — a sustained innovation the
    // heading-axis gyro bias absorbed (~±20–200 dps at sim start depending
    // on bench orientation, decaying all flight).
    encodeIIS2MDC(micros(), data_out);
    return true;
}

bool SensorCollectorSim::getGNSSData(GNSSData& data_out)
{
    bool have_data = real_.getGNSSData(data_out);

    if (phase_ == SIM_IDLE) return have_data;

    if (have_data)
    {
        // Real GNSS data arrived — consume it and replace with simulated values
        last_gnss_real_us_ = micros();
        encodeGNSS(micros(), data_out);
        return true;
    }

    // Fallback: if no real GNSS data for >100ms (indoor testing), generate
    // simulated GNSS on a timer at 25 Hz so the state machine can see a
    // GPS lock and transition READY → PRELAUNCH.
    const uint32_t now_us = micros();
    if ((now_us - last_gnss_real_us_) > GNSS_FALLBACK_TIMEOUT_US &&
        (now_us - gnss_timer_us_) >= GNSS_FALLBACK_INTERVAL_US)
    {
        gnss_timer_us_ = now_us;
        encodeGNSS(now_us, data_out);
        return true;
    }

    return false;
}

// ============================================================================
// Physics
// ============================================================================

void SensorCollectorSim::stepPhysics(float dt)
{
    const uint32_t elapsed_ms = millis() - phase_start_ms_;

    switch (phase_)
    {
    case SIM_PRELAUNCH:
        stepPrelaunch(elapsed_ms);
        break;
    case SIM_POWERED:
        stepPowered(dt);
        break;
    case SIM_COASTING:
        stepCoasting(dt);
        break;
    case SIM_DESCENT:
        stepDescent(dt);
        break;
    case SIM_LANDED:
    {
        velocity_mps_   = 0.0f;
        accel_mps2_     = 0.0f;
        pitch_rad_      = 0.0f;
        pitch_rate_rps_ = 0.0f;
        // Hold until the FC's own state machine reaches LANDED (#971).
        //
        // This used to be a fixed 9000 ms, chosen from a comment that budgeted
        // "5 consecutive 1-second checks (5s) plus a 2-second state-machine
        // debounce = 7s minimum".  That budget was wrong in a way that only
        // shows on a sim: a sim never produces an impact (accel is a flat 1 g,
        // never the 15 g LANDING_IMPACT_G spike), so alt_landed_flag can only
        // come from the SLOW vote, whose sub-flags run a leaky counter at 1 Hz.
        // Measured on flight_20260827_122854: the flag took 7.0 s, and the FC
        // then wants `> 2000` ms MORE of it — i.e. >9.0 s against a 9.0 s hold.
        // Every sim flight missed LANDED by a margin of zero, leaving the FC in
        // INFLIGHT with the log open until the 10-minute backstop.
        //
        // Asking the FC directly means this cannot drift out of sync again if
        // the detector's timing changes.  The timeout is only a backstop so a
        // detector that never fires cannot hang the sim forever.
        const sim_landed::Exit decision =
            sim_landed::decide(fc_rocket_state_, (uint8_t)LANDED, elapsed_ms);
        if (decision == sim_landed::Exit::FcLanded)
        {
            phase_ = SIM_IDLE;
            ESP_LOGI("SIM", "Complete (FC reached LANDED after %.1f s hold)",
                     (double)(elapsed_ms / 1000.0f));
        }
        else if (decision == sim_landed::Exit::GaveUp)
        {
            phase_ = SIM_IDLE;
            ESP_LOGW("SIM", "Complete (gave up after %.1f s — FC never left "
                     "state %u; landing detection did not fire)",
                     (double)(elapsed_ms / 1000.0f), (unsigned)fc_rocket_state_);
        }
        break;
    }
    default:
        break;
    }
}

void SensorCollectorSim::stepPrelaunch(uint32_t elapsed_ms)
{
    altitude_m_     = 0.0f;
    velocity_mps_   = 0.0f;
    accel_mps2_     = 0.0f;
    pitch_rad_      = LAUNCH_ANGLE_RAD;
    pitch_rate_rps_ = 0.0f;

    if (elapsed_ms >= (uint32_t)PRELAUNCH_DURATION_MS)
    {
        phase_ = SIM_POWERED;
        phase_start_ms_ = millis();
        Serial.println("[SIM] Phase: POWERED");
    }
}

void SensorCollectorSim::stepPowered(float dt)
{
    const float thrust_accel = cfg_.thrust_n / cfg_.mass_kg;
    const float drag = dragAccel(velocity_mps_, altitude_m_);
    const float net_accel = thrust_accel - GRAVITY - drag;

    velocity_mps_ += net_accel * dt;
    altitude_m_   += velocity_mps_ * dt;
    if (altitude_m_ < 0.0f) altitude_m_ = 0.0f;

    accel_mps2_ = net_accel;

    const float burn_elapsed = (float)(millis() - phase_start_ms_) * 0.001f;
    if (burn_elapsed >= cfg_.burn_time_s)
    {
        phase_ = SIM_COASTING;
        phase_start_ms_ = millis();
        Serial.println("[SIM] Phase: COASTING");
    }
}

void SensorCollectorSim::stepCoasting(float dt)
{
    const float drag = dragAccel(velocity_mps_, altitude_m_);
    const float net_accel = -GRAVITY - drag;

    velocity_mps_ += net_accel * dt;
    altitude_m_   += velocity_mps_ * dt;
    if (altitude_m_ < 0.0f) altitude_m_ = 0.0f;

    accel_mps2_ = net_accel;

    // --- Pitch dynamics: gravity turn ---
    // Gravity acting on a stable rocket (CP aft of CG) creates a
    // restoring moment that tips the nose toward the velocity vector.
    // M_grav = m * g * sin(θ) * -(CP-CG distance)
    // Sign: when pitch > 0 (nose up), moment is negative (tips nose down).
    const float M_grav = cfg_.mass_kg * GRAVITY * sinf(pitch_rad_) * (-CP_CG_DIST_M);

    // Aerodynamic pitch damping (opposes rotation, proportional to q_dyn)
    float M_damp = 0.0f;
    const float v_abs = fabsf(velocity_mps_);
    if (v_abs > 1.0f)
    {
        const float q_dyn = 0.5f * airDensity(altitude_m_) * v_abs * v_abs;
        M_damp = -C_DAMP * q_dyn * REFERENCE_AREA / v_abs * pitch_rate_rps_;
    }

    const float pitch_accel = (M_grav + M_damp) / I_TRANSVERSE;
    pitch_rate_rps_ += pitch_accel * dt;
    pitch_rad_      += pitch_rate_rps_ * dt;

    if (velocity_mps_ <= 0.0f)
    {
        velocity_mps_ = 0.0f;
        phase_ = SIM_DESCENT;
        phase_start_ms_ = millis();
        ESP_LOGI("SIM", "Phase: DESCENT (apogee ~%.0fm, pitch=%.1f deg)",
                 (double)altitude_m_, (double)(pitch_rad_ * 180.0f / 3.14159265f));
    }
}

void SensorCollectorSim::stepDescent(float dt)
{
    const float rate = (cfg_.descent_rate_mps > 0.0f)
        ? cfg_.descent_rate_mps : 5.0f;
    velocity_mps_ = -rate;
    altitude_m_  += velocity_mps_ * dt;
    accel_mps2_   = 0.0f;

    // Under parachute: pendulum relaxes pitch toward 0° (horizontal)
    const float M_pend = -cfg_.mass_kg * GRAVITY * sinf(pitch_rad_) * 0.1f;
    const float M_damp_chute = -0.5f * pitch_rate_rps_;
    pitch_rate_rps_ += (M_pend + M_damp_chute) / I_TRANSVERSE * dt;
    pitch_rad_      += pitch_rate_rps_ * dt;

    if (altitude_m_ <= 0.0f)
    {
        altitude_m_   = 0.0f;
        velocity_mps_ = 0.0f;
        phase_ = SIM_LANDED;
        phase_start_ms_ = millis();
        Serial.println("[SIM] Phase: LANDED");
    }
}

// ============================================================================
// Physics helpers
// ============================================================================

float SensorCollectorSim::airDensity(float alt_m) const
{
    const float x = 1.0f - 2.2558e-5f * alt_m;
    return (x > 0.0f) ? 1.225f * powf(x, 4.2559f) : 0.0f;
}

float SensorCollectorSim::pressureAtAltitude(float alt_m) const
{
    const float x = 1.0f - alt_m / 44330.0f;
    return (x > 0.0f) ? ground_pressure_pa_ * powf(x, 5.255f) : 0.0f;
}

float SensorCollectorSim::dragAccel(float velocity, float alt) const
{
    if (cfg_.mass_kg <= 0.0f) return 0.0f;
    const float v = fabsf(velocity);
    const float rho = airDensity(alt);
    const float f_drag = 0.5f * CD * REFERENCE_AREA * rho * v * v;
    const float a_drag = f_drag / cfg_.mass_kg;
    return (velocity > 0.0f) ? a_drag : -a_drag;
}

// ============================================================================
// Sensor encoding (inverse of SensorConverter formulas)
// ============================================================================

void SensorCollectorSim::encodeISM6(uint32_t time_us, ISM6HG256Data& out)
{
    memset(&out, 0, sizeof(out));
    out.time_us = time_us;

    // Specific force in body frame.
    // Body X = along rocket axis (forward), Z = perpendicular (down in body).
    //
    // The sim tracks vertical motion + pitch angle.  Gravity is [0, 0, -g]
    // in ENU.  Projected into body frame by pitch angle:
    //   body_ax = accel_vertical * sin(pitch) + g * sin(pitch)
    //           = (accel_mps2_ + GRAVITY) * sin(pitch)
    //   body_az = (accel_mps2_ + GRAVITY) * cos(pitch)
    //
    // Simplification: accel_mps2_ is the net vertical acceleration (excl gravity).
    // specific_force_vertical = accel_mps2_ + GRAVITY (what a vertical accel would read).
    // Project onto body axes via pitch:
    // #512: the specific-force DIRECTION is the world "up" seen in the body
    // frame; the magnitude is the vertical specific force. This was already
    // correct — it is the gyro and the field that were inconsistent with it.
    const float sf_vert = accel_mps2_ + GRAVITY;  // specific force if perfectly vertical
    float up_b[3];
    sim_sensor_model::upInBody(pitch_rad_, up_b);
    float body_ax = sf_vert * up_b[0];   // axial (along rocket)
    float body_ay = sf_vert * up_b[1];
    float body_az = sf_vert * up_b[2];   // perpendicular

    // Carry the sim's pad frame onto the board's actual resting attitude (#508)
    // so the real→sim handover has no gravity/attitude step.  Identity unless a
    // pad alignment was solved.
    applyPadAlign(body_ax, body_ay, body_az);

    // Rocket frame → board frame (inverse of the mounting orientation;
    // identity for the standard +X-nose mounting).
    rocketToBoard(body_ax, body_ay, body_az);

    // Inverse rotation: board-frame → sensor-frame (ISM6 board rotation about Z)
    // Only rotates X/Y; Z passes through
    const float sensor_ax =  body_ax * ism6_inv_c_ + body_ay * ism6_inv_s_;
    const float sensor_ay = -body_ax * ism6_inv_s_ + body_ay * ism6_inv_c_;
    const float sensor_az =  body_az;

    // Low-G accelerometer (±16g range)
    out.acc_low_raw.x = (int16_t)constrain(
        lroundf(sensor_ax / ACC_LOW_MS2_PER_LSB), -32768, 32767);
    out.acc_low_raw.y = (int16_t)constrain(
        lroundf(sensor_ay / ACC_LOW_MS2_PER_LSB), -32768, 32767);
    out.acc_low_raw.z = (int16_t)constrain(
        lroundf(sensor_az / ACC_LOW_MS2_PER_LSB), -32768, 32767);

    // High-G accelerometer (±256g range)
    out.acc_high_raw.x = (int16_t)constrain(
        lroundf(sensor_ax / ACC_HIGH_MS2_PER_LSB), -32768, 32767);
    out.acc_high_raw.y = (int16_t)constrain(
        lroundf(sensor_ay / ACC_HIGH_MS2_PER_LSB), -32768, 32767);
    out.acc_high_raw.z = (int16_t)constrain(
        lroundf(sensor_az / ACC_HIGH_MS2_PER_LSB), -32768, 32767);

    // Gyro: body rate for the current pitch rate, through the same inverse chain
    // as accel (rocket → board → sensor).
    //
    // #512: this used to be +pitch_rate on Y, which is the WRONG SIGN. In FLU a
    // positive rotation about +Y pitches the nose DOWN, while `pitch` measures
    // nose-UP — so a rising nose is a NEGATIVE omega_y. The old sign made the
    // accelerometer's gravity vector rotate 180° opposite to the gyro, i.e. the
    // sim emitted a motion no rigid body can perform, and the EKF could never
    // fuse it. gyroInBody() owns the sign now, alongside upInBody/fieldInBody, so
    // the three cannot drift apart again.
    float gyro_b[3];
    sim_sensor_model::gyroInBody(pitch_rate_rps_ * (180.0f / 3.14159265f), gyro_b);
    float gyro_body_x_dps = gyro_b[0];
    float gyro_body_y_dps = gyro_b[1];
    float gyro_body_z_dps = gyro_b[2];
    // Same rigid rotation as the accel: the body rates must be expressed in the
    // same re-oriented frame or the attitude the EKF integrates won't match the
    // gravity vector it sees (#508).
    applyPadAlign(gyro_body_x_dps, gyro_body_y_dps, gyro_body_z_dps);
    rocketToBoard(gyro_body_x_dps, gyro_body_y_dps, gyro_body_z_dps);

    const float sensor_gx =  gyro_body_x_dps * ism6_inv_c_ + gyro_body_y_dps * ism6_inv_s_;
    const float sensor_gy = -gyro_body_x_dps * ism6_inv_s_ + gyro_body_y_dps * ism6_inv_c_;
    const float sensor_gz =  gyro_body_z_dps;
    out.gyro_raw.x = (int16_t)constrain(
        lroundf(sensor_gx / GYRO_DPS_PER_LSB), -32768, 32767);
    out.gyro_raw.y = (int16_t)constrain(
        lroundf(sensor_gy / GYRO_DPS_PER_LSB), -32768, 32767);
    out.gyro_raw.z = (int16_t)constrain(
        lroundf(sensor_gz / GYRO_DPS_PER_LSB), -32768, 32767);
}

void SensorCollectorSim::encodeBMP585(uint32_t time_us, BMP585Data& out)
{
    memset(&out, 0, sizeof(out));
    out.time_us  = time_us;
    out.press_q6 = (uint32_t)(pressureAtAltitude(altitude_m_) * 64.0f);
    out.temp_q16 = (int32_t)(20.0f * 65536.0f);  // 20 deg C
}

void SensorCollectorSim::encodeMMC5983MA(uint32_t time_us, MMC5983MAData& out)
{
    memset(&out, 0, sizeof(out));
    out.time_us = time_us;

    // Earth magnetic field at ~38N latitude (NED frame):
    //   North ~ 22 µT, East ~ 5 µT, Down ~ 42 µT
    // Rotate into body frame by pitch angle.
    // Body X = along rocket axis, Y = lateral, Z = perpendicular (body-down)
    // Pitch rotation: nose-up angle from horizontal.
    //   body_x =  North * sin(pitch) + Down * cos(pitch)
    //   body_y =  East
    //   body_z = -North * cos(pitch) + Down * sin(pitch)
    static constexpr float COUNTS_PER_UT = 131072.0f / 800.0f;

    // #512: was (N·sp + D·cp, E, -N·cp + D·sp) — a rotation whose implied "up"
    // sat 90°+ away from the accelerometer's. fieldInBody() projects the NED
    // field onto the actual body axes, so it now rotates in lockstep with
    // upInBody/gyroInBody.
    float body[3];
    sim_sensor_model::fieldInBody(pitch_rad_, B_NORTH, B_EAST, B_DOWN, body);
    float body_x = body[0];
    float body_y = body[1];
    float body_z = body[2];

    // Pad-frame alignment (#508) — keeps the field continuous across the
    // real→sim handover, so no heading step winds the gyro bias either.
    applyPadAlign(body_x, body_y, body_z);

    // Rocket frame → board frame (inverse mounting orientation), matching
    // the forward board→rocket rotation the converter now applies to mag.
    rocketToBoard(body_x, body_y, body_z);

    out.mag_x = (uint32_t)(lroundf(body_x * COUNTS_PER_UT) + 131072);
    out.mag_y = (uint32_t)(lroundf(body_y * COUNTS_PER_UT) + 131072);
    out.mag_z = (uint32_t)(lroundf(body_z * COUNTS_PER_UT) + 131072);
}

void SensorCollectorSim::encodeIIS2MDC(uint32_t time_us, IIS2MDCData& out)
{
    memset(&out, 0, sizeof(out));
    out.time_us = time_us;

    // Same Earth field + body projection as encodeMMC5983MA (#463, corrected #512).
    static constexpr float COUNTS_PER_UT = 1.0f / 0.15f;   // datasheet 0.15 µT/LSB

    float body[3];
    sim_sensor_model::fieldInBody(pitch_rad_, B_NORTH, B_EAST, B_DOWN, body);
    float body_x = body[0];
    float body_y = body[1];
    float body_z = body[2];

    // Pad-frame alignment (#508) — see encodeISM6.
    applyPadAlign(body_x, body_y, body_z);

    // Rocket frame → board frame (inverse mounting), then board → sensor
    // frame (inverse of the converter's sensor→board +Z rotation), so the
    // forward conversion chain reproduces the simulated field exactly.
    rocketToBoard(body_x, body_y, body_z);
    const float sensor_x =  body_x * iis_inv_c_ + body_y * iis_inv_s_;
    const float sensor_y = -body_x * iis_inv_s_ + body_y * iis_inv_c_;

    out.mag_x = (int16_t)lroundf(sensor_x * COUNTS_PER_UT);
    out.mag_y = (int16_t)lroundf(sensor_y * COUNTS_PER_UT);
    out.mag_z = (int16_t)lroundf(body_z   * COUNTS_PER_UT);
}

void SensorCollectorSim::encodeGNSS(uint32_t time_us, GNSSData& out)
{
    memset(&out, 0, sizeof(out));
    out.time_us = time_us;

    // Fixed simulated location
    out.lat_e7 = (int32_t)(38.0 * 1e7);
    out.lon_e7 = (int32_t)(-122.0 * 1e7);

    out.alt_mm     = (int32_t)(altitude_m_ * 1000.0f);
    out.vel_u_mmps = (int32_t)(velocity_mps_ * 1000.0f);
    out.vel_e_mmps = 0;
    out.vel_n_mmps = 0;

    // Simulate good GPS fix
    out.fix_mode = 3;   // 3D fix
    out.num_sats = 12;
    out.pdop_x10 = 15;  // PDOP 1.5
    out.h_acc_m  = 2;
    out.v_acc_m  = 3;

    // Timestamp
    out.year   = 2025;
    out.month  = 1;
    out.day    = 1;
    out.hour   = 12;
    out.minute = 0;
    out.second = (uint8_t)((uint32_t)sim_time_s_ % 60);
    out.milli_second = (uint16_t)(fmodf(sim_time_s_, 1.0f) * 1000.0f);
}
