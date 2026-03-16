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
#include <cmath>

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

// ============================================================================
// Sim Control
// ============================================================================

void SensorCollectorSim::configureSim(const SimConfigData& cfg)
{
    cfg_ = cfg;
    configured_ = true;
    Serial.printf("[SIM] Config: mass=%.3fkg thrust=%.1fN burn=%.1fs descent=%.1fm/s\n",
                  (double)cfg_.mass_kg, (double)cfg_.thrust_n,
                  (double)cfg_.burn_time_s, (double)cfg_.descent_rate_mps);
}

void SensorCollectorSim::configureSimRotation(float ism6_rot_z_deg)
{
    float rad = ism6_rot_z_deg * (M_PI / 180.0f);
    ism6_inv_c_ = cosf(rad);
    ism6_inv_s_ = sinf(rad);
    Serial.printf("[SIM] ISM6 rotation: %.1f deg (inv_c=%.4f, inv_s=%.4f)\n",
                  (double)ism6_rot_z_deg, (double)ism6_inv_c_, (double)ism6_inv_s_);
}

void SensorCollectorSim::startSim(float ground_pressure_pa)
{
    if (!configured_)
    {
        Serial.println("[SIM] ERROR: startSim called but not configured (missing config cmd?)");
        return;
    }

    altitude_m_   = 0.0f;
    velocity_mps_ = 0.0f;
    accel_mps2_   = 0.0f;
    sim_time_s_   = 0.0f;

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
        velocity_mps_ = 0.0f;
        accel_mps2_   = 0.0f;
        // Hold long enough for the kinematic landing checks to fire.
        // Landing detection needs 5 consecutive 1-second checks (5s)
        // plus a 2-second state-machine debounce = 7s minimum.
        if (elapsed_ms >= 9000)
        {
            phase_ = SIM_IDLE;
            Serial.println("[SIM] Complete (auto-stop after LANDED hold)");
        }
        break;
    default:
        break;
    }
}

void SensorCollectorSim::stepPrelaunch(uint32_t elapsed_ms)
{
    altitude_m_   = 0.0f;
    velocity_mps_ = 0.0f;
    accel_mps2_   = 0.0f;

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

    if (velocity_mps_ <= 0.0f)
    {
        velocity_mps_ = 0.0f;
        phase_ = SIM_DESCENT;
        phase_start_ms_ = millis();
        Serial.printf("[SIM] Phase: DESCENT (apogee ~%.0fm)\n", (double)altitude_m_);
    }
}

void SensorCollectorSim::stepDescent(float dt)
{
    const float rate = (cfg_.descent_rate_mps > 0.0f)
        ? cfg_.descent_rate_mps : 5.0f;
    velocity_mps_ = -rate;
    altitude_m_  += velocity_mps_ * dt;
    accel_mps2_   = 0.0f;

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

    // Accelerometer reads specific force = net_accel + g
    //   Pad:     0 + g = g       (1g at rest)
    //   Thrust:  (T-g-D) + g = T-D  (reads thrust minus drag)
    //   Coast:   (-g-D) + g = -D    (near 0g in freefall)
    //   Descent: 0 + g = g       (1g under parachute)
    // Body-frame: accel on X (forward / long axis), zero on Y/Z
    const float body_ax = accel_mps2_ + GRAVITY;
    // body_ay = 0

    // Inverse rotation: body-frame → sensor-frame
    // sensor = R(θ)^T * body  where θ is the ISM6 config angle
    //        = [ cos θ   sin θ ] [ body_ax ]
    //          [-sin θ   cos θ ] [    0    ]
    const float sensor_ax =  body_ax * ism6_inv_c_;
    const float sensor_ay = -body_ax * ism6_inv_s_;

    // Low-G accelerometer (+-16g range)
    out.acc_low_raw.x = (int16_t)constrain(
        lroundf(sensor_ax / ACC_LOW_MS2_PER_LSB), -32768, 32767);
    out.acc_low_raw.y = (int16_t)constrain(
        lroundf(sensor_ay / ACC_LOW_MS2_PER_LSB), -32768, 32767);
    // z stays 0 (memset above)

    // High-G accelerometer (+-256g range)
    out.acc_high_raw.x = (int16_t)constrain(
        lroundf(sensor_ax / ACC_HIGH_MS2_PER_LSB), -32768, 32767);
    out.acc_high_raw.y = (int16_t)constrain(
        lroundf(sensor_ay / ACC_HIGH_MS2_PER_LSB), -32768, 32767);

    // Gyro (no rotation in 1D sim)
    // x, y, z already zeroed by memset
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

    // Simulated Earth magnetic field at ~38N latitude
    // X(north) ~ 22 uT, Y(east) ~ 5 uT, Z(down) ~ 42 uT
    static constexpr float COUNTS_PER_UT = 131072.0f / 800.0f;

    out.mag_x = (uint32_t)(lroundf(22.0f * COUNTS_PER_UT) + 131072);
    out.mag_y = (uint32_t)(lroundf(5.0f  * COUNTS_PER_UT) + 131072);
    out.mag_z = (uint32_t)(lroundf(42.0f * COUNTS_PER_UT) + 131072);
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
