#ifndef FLIGHT_SIMULATOR_H
#define FLIGHT_SIMULATOR_H

//
//  FlightSimulator.h
//  TinkerRocket OutComputer
//
//  1D physics-based rocket flight simulator.
//  Generates synthetic sensor data (ISM6, BMP585, GNSS, POWER, NonSensor, MMC5983MA)
//  that feeds directly into the OutComputer pipeline for end-to-end testing.
//
//  Header-only for easy Arduino sketch compilation.
//

#include <Arduino.h>
#include <RocketComputerTypes.h>
#include <cmath>

class FlightSimulator
{
public:
    // ========================================================================
    // Types
    // ========================================================================

    enum Phase : uint8_t
    {
        SIM_IDLE = 0,
        SIM_PRELAUNCH,
        SIM_POWERED,
        SIM_COASTING,
        SIM_DESCENT,
        SIM_LANDED
    };

    struct Config
    {
        float mass_kg;            // Rocket mass in kg
        float thrust_n;           // Average motor thrust in Newtons
        float burn_time_s;        // Motor burn time in seconds
        float descent_rate_mps;   // Parachute descent rate in m/s (positive = downward)
    };

    // ========================================================================
    // Public Interface
    // ========================================================================

    void configure(const Config& cfg)
    {
        cfg_ = cfg;
        configured_ = true;
    }

    /// Pre-compute inverse rotation for body-frame → sensor-frame encoding.
    /// Pass the ISM6 config rotation angle (sensor → board, degrees).
    void configureRotation(float ism6_rot_z_deg)
    {
        float rad = ism6_rot_z_deg * (M_PI / 180.0f);
        ism6_inv_c_ = cosf(rad);
        ism6_inv_s_ = sinf(rad);
    }

    /// Start the sim. Pass the current real ground pressure so simulated
    /// barometer output matches the actual environment (avoids a pressure
    /// step function when switching from real to synthetic sensor data).
    void start(float current_ground_pressure_pa = 0.0f)
    {
        if (!configured_) return;

        // Reset physics state
        altitude_m_ = 0.0f;
        velocity_mps_ = 0.0f;
        accel_mps2_ = 0.0f;
        max_alt_m_ = 0.0f;
        max_speed_mps_ = 0.0f;
        sim_time_s_ = 0.0f;
        flags_ = 0;

        phase_ = SIM_PRELAUNCH;
        phase_start_ms_ = millis();
        last_step_us_ = micros();

        // Use real ground pressure if available, otherwise fall back to standard atmosphere
        ground_pressure_pa_ = (current_ground_pressure_pa > 50000.0f)
            ? current_ground_pressure_pa
            : 101325.0f;
    }

    void stop()
    {
        phase_ = SIM_IDLE;
    }

    bool isActive() const
    {
        return phase_ != SIM_IDLE;
    }

    bool isConfigured() const
    {
        return configured_;
    }

    Phase getPhase() const
    {
        return phase_;
    }

    // ========================================================================
    // Step — advance physics and encode sensor data
    // ========================================================================

    void step(ISM6HG256Data& ism6_out,
              BMP585Data& bmp_out,
              GNSSData& gnss_out,
              POWERData& power_out,
              NonSensorData& ns_out,
              MMC5983MAData& mmc_out)
    {
        if (phase_ == SIM_IDLE) return;

        // Calculate real elapsed time
        const uint32_t now_us = micros();
        const float dt = (float)(now_us - last_step_us_) * 1.0e-6f;
        last_step_us_ = now_us;

        // Clamp dt to avoid huge jumps (e.g., breakpoint or slow loop)
        const float clamped_dt = (dt > 0.1f) ? 0.1f : dt;
        sim_time_s_ += clamped_dt;

        const uint32_t elapsed_ms = millis() - phase_start_ms_;

        // ---- Phase state machine ----
        switch (phase_)
        {
        case SIM_PRELAUNCH:
            stepPrelaunch(elapsed_ms);
            break;
        case SIM_POWERED:
            stepPowered(clamped_dt);
            break;
        case SIM_COASTING:
            stepCoasting(clamped_dt);
            break;
        case SIM_DESCENT:
            stepDescent(clamped_dt);
            break;
        case SIM_LANDED:
            // Hold landed state for 2 seconds so BLE telemetry (1s interval)
            // sends at least 1-2 updates with LANDED state and flags.
            velocity_mps_ = 0.0f;
            accel_mps2_ = 0.0f;
            if (elapsed_ms >= 2000)
            {
                phase_ = SIM_IDLE;  // isActive() returns false on next loop
            }
            break;
        default:
            break;
        }

        // Track maximums
        if (altitude_m_ > max_alt_m_) max_alt_m_ = altitude_m_;
        const float speed = fabsf(velocity_mps_);
        if (speed > max_speed_mps_) max_speed_mps_ = speed;

        // ---- Encode synthetic sensor data ----
        const uint32_t time_us = now_us;
        const uint8_t rocket_state = rocketStateForPhase();

        encodeBMP585(pressureAtAltitude(altitude_m_), 20.0f, time_us, bmp_out);
        encodeISM6(accel_mps2_, time_us, ism6_out);
        encodeGNSS(altitude_m_, velocity_mps_, time_us, gnss_out);
        encodePOWER(time_us, power_out);
        encodeNonSensor(altitude_m_, velocity_mps_, rocket_state, flags_,
                        time_us, ns_out);
        encodeMMC5983MA(time_us, mmc_out);
    }

private:
    // ========================================================================
    // Configuration & State
    // ========================================================================

    Config cfg_ = {};
    bool configured_ = false;
    volatile Phase phase_ = SIM_IDLE;  // volatile: read from Core 0 sim task, written from Core 1
    uint32_t phase_start_ms_ = 0;
    uint32_t last_step_us_ = 0;

    // Physics state (SI)
    float altitude_m_ = 0.0f;
    float velocity_mps_ = 0.0f;   // positive = up
    float accel_mps2_ = 0.0f;     // net acceleration (body X = forward/long axis)
    float max_alt_m_ = 0.0f;
    float max_speed_mps_ = 0.0f;
    float sim_time_s_ = 0.0f;
    float ground_pressure_pa_ = 101325.0f;
    uint8_t flags_ = 0;

    // ========================================================================
    // Constants
    // ========================================================================

    static constexpr float GRAVITY = 9.80665f;
    static constexpr float CD = 0.5f;                 // Average rocket drag coefficient
    static constexpr float REFERENCE_AREA = 0.0019635f; // pi * 0.025^2  (50mm diameter)
    static constexpr float PARACHUTE_DESCENT_RATE = 5.0f; // m/s downward
    static constexpr float PRELAUNCH_DURATION_MS = 5000.0f;

    // ISM6 sensitivity constants (for ±16g low-G, ±256g high-G, ±4000 dps gyro)
    static constexpr float ACC_LOW_MS2_PER_LSB = (16.0f * 1000.0f / 32768.0f) * 1.0e-3f * GRAVITY;
    static constexpr float ACC_HIGH_MS2_PER_LSB = (256.0f * 1000.0f / 32768.0f) * 1.0e-3f * GRAVITY;

    // Inverse rotation (body frame → sensor frame) for ISM6 encoding
    float ism6_inv_c_ = 1.0f;   // cos(config_angle)
    float ism6_inv_s_ = 0.0f;   // sin(config_angle)

    // ========================================================================
    // Phase Steppers
    // ========================================================================

    void stepPrelaunch(uint32_t elapsed_ms)
    {
        // Sitting on pad — gravity only, no motion
        altitude_m_ = 0.0f;
        velocity_mps_ = 0.0f;
        accel_mps2_ = 0.0f;  // accelerometer reads ~1g (gravity), but net accel is 0
        flags_ = 0;

        if (elapsed_ms >= (uint32_t)PRELAUNCH_DURATION_MS)
        {
            phase_ = SIM_POWERED;
            phase_start_ms_ = millis();
            flags_ = NSF_LAUNCH;
        }
    }

    void stepPowered(float dt)
    {
        // Thrust + gravity + drag
        const float thrust_accel = cfg_.thrust_n / cfg_.mass_kg;
        const float drag = dragAccel(velocity_mps_, altitude_m_);
        const float net_accel = thrust_accel - GRAVITY - drag;

        velocity_mps_ += net_accel * dt;
        altitude_m_ += velocity_mps_ * dt;
        if (altitude_m_ < 0.0f) altitude_m_ = 0.0f;

        // Net acceleration (velocity derivative). encodeISM6() adds +GRAVITY
        // to convert to specific force (what a real accelerometer reads).
        accel_mps2_ = net_accel;

        flags_ = NSF_LAUNCH;

        // Check burn time elapsed
        const float burn_elapsed = (float)(millis() - phase_start_ms_) * 0.001f;
        if (burn_elapsed >= cfg_.burn_time_s)
        {
            phase_ = SIM_COASTING;
            phase_start_ms_ = millis();
        }
    }

    void stepCoasting(float dt)
    {
        // Gravity + drag (no thrust)
        const float drag = dragAccel(velocity_mps_, altitude_m_);
        const float net_accel = -GRAVITY - drag;  // drag opposes upward velocity

        velocity_mps_ += net_accel * dt;
        altitude_m_ += velocity_mps_ * dt;
        if (altitude_m_ < 0.0f) altitude_m_ = 0.0f;

        // Net acceleration (velocity derivative). encodeISM6() adds +GRAVITY
        // to convert to specific force (what a real accelerometer reads).
        accel_mps2_ = net_accel;

        flags_ = NSF_LAUNCH;

        // Check for apogee (velocity crosses zero)
        if (velocity_mps_ <= 0.0f)
        {
            velocity_mps_ = 0.0f;
            phase_ = SIM_DESCENT;
            phase_start_ms_ = millis();
            flags_ |= NSF_VEL_APOGEE | NSF_ALT_APOGEE;
        }
    }

    void stepDescent(float dt)
    {
        // Constant descent rate under parachute (from config, falls back to default)
        const float rate = (cfg_.descent_rate_mps > 0.0f)
            ? cfg_.descent_rate_mps : PARACHUTE_DESCENT_RATE;
        velocity_mps_ = -rate;
        altitude_m_ += velocity_mps_ * dt;
        accel_mps2_ = 0.0f;  // steady descent, ~1g on IMU

        flags_ = NSF_LAUNCH | NSF_VEL_APOGEE | NSF_ALT_APOGEE;

        // Check for landing
        if (altitude_m_ <= 0.0f)
        {
            altitude_m_ = 0.0f;
            velocity_mps_ = 0.0f;
            phase_ = SIM_LANDED;
            phase_start_ms_ = millis();
            flags_ |= NSF_ALT_LANDED;
        }
    }

    // ========================================================================
    // Physics Helpers
    // ========================================================================

    /// Standard atmosphere air density at altitude (kg/m³)
    float airDensity(float alt_m) const
    {
        // ISA troposphere model
        // rho = 1.225 * (1 - 2.2558e-5 * alt)^4.2559
        const float x = 1.0f - 2.2558e-5f * alt_m;
        return (x > 0.0f) ? 1.225f * powf(x, 4.2559f) : 0.0f;
    }

    /// Barometric pressure at altitude (Pa)
    float pressureAtAltitude(float alt_m) const
    {
        // Inverse of: alt = 44330 * (1 - (P/P0)^(1/5.255))
        // P = P0 * (1 - alt/44330)^5.255
        const float x = 1.0f - alt_m / 44330.0f;
        return (x > 0.0f) ? ground_pressure_pa_ * powf(x, 5.255f) : 0.0f;
    }

    /// Drag acceleration with sign matching velocity direction.
    /// Positive when going up (callers subtract it to decelerate),
    /// negative when going down (callers subtract it, which decelerates
    /// downward motion by making net_accel less negative).
    float dragAccel(float velocity, float alt) const
    {
        if (cfg_.mass_kg <= 0.0f) return 0.0f;
        const float v = fabsf(velocity);
        const float rho = airDensity(alt);
        const float f_drag = 0.5f * CD * REFERENCE_AREA * rho * v * v;
        const float a_drag = f_drag / cfg_.mass_kg;
        return (velocity > 0.0f) ? a_drag : -a_drag;
    }

    /// Map sim phase to RocketState enum value
    uint8_t rocketStateForPhase() const
    {
        switch (phase_)
        {
        case SIM_PRELAUNCH: return (uint8_t)PRELAUNCH;
        case SIM_POWERED:
        case SIM_COASTING:
        case SIM_DESCENT:   return (uint8_t)INFLIGHT;
        case SIM_LANDED:    return (uint8_t)LANDED;
        default:            return (uint8_t)READY;
        }
    }

    // ========================================================================
    // Sensor Encoding (inverse of SensorConverter formulas)
    // ========================================================================

    void encodeBMP585(float pressure_pa, float temp_c, uint32_t time_us, BMP585Data& out)
    {
        memset(&out, 0, sizeof(out));
        out.time_us = time_us;
        out.press_q6 = (uint32_t)(pressure_pa * 64.0f);
        out.temp_q16 = (int32_t)(temp_c * 65536.0f);
    }

    void encodeISM6(float accel_mps2, uint32_t time_us, ISM6HG256Data& out)
    {
        memset(&out, 0, sizeof(out));
        out.time_us = time_us;

        // accel_mps2 is the net acceleration (d(velocity)/dt).
        // A real accelerometer reads specific force = net_accel + g
        // (gravity is invisible to the sensor, so it adds +g offset).
        //   Pad:     0 + g = g       (1g at rest)
        //   Thrust:  (T-g-D) + g = T-D  (reads thrust minus drag)
        //   Coast:   (-g-D) + g = -D    (near 0g in freefall)
        //   Descent: 0 + g = g       (1g under parachute)
        // Body-frame: accel on X (forward / long axis), zero on Y/Z.
        const float body_ax = accel_mps2 + GRAVITY;
        // body_ay = 0

        // Inverse rotation: body-frame → sensor-frame
        // sensor = R(θ)^T * body  where θ is the ISM6 config angle
        //        = [ cos θ   sin θ ] [ body_ax ]
        //          [-sin θ   cos θ ] [    0    ]
        const float sensor_ax =  body_ax * ism6_inv_c_;
        const float sensor_ay = -body_ax * ism6_inv_s_;

        // Low-G accelerometer (±16g range)
        out.acc_low_raw.x = (int16_t)constrain(
            lroundf(sensor_ax / ACC_LOW_MS2_PER_LSB), -32768, 32767);
        out.acc_low_raw.y = (int16_t)constrain(
            lroundf(sensor_ay / ACC_LOW_MS2_PER_LSB), -32768, 32767);
        // z stays 0 (memset above)

        // High-G accelerometer (±256g range)
        out.acc_high_raw.x = (int16_t)constrain(
            lroundf(sensor_ax / ACC_HIGH_MS2_PER_LSB), -32768, 32767);
        out.acc_high_raw.y = (int16_t)constrain(
            lroundf(sensor_ay / ACC_HIGH_MS2_PER_LSB), -32768, 32767);

        // Gyro (no rotation in 1D sim)
        // x, y, z already zeroed by memset
    }

    void encodeGNSS(float alt_m, float vel_u_mps, uint32_t time_us, GNSSData& out)
    {
        memset(&out, 0, sizeof(out));
        out.time_us = time_us;

        // Fixed simulated location (somewhere generic)
        out.lat_e7 = (int32_t)(38.0 * 1e7);
        out.lon_e7 = (int32_t)(-122.0 * 1e7);

        out.alt_mm = (int32_t)(alt_m * 1000.0f);
        out.vel_u_mmps = (int32_t)(vel_u_mps * 1000.0f);
        out.vel_e_mmps = 0;
        out.vel_n_mmps = 0;

        // Simulate good GPS fix
        out.fix_mode = 3;  // 3D fix
        out.num_sats = 12;
        out.pdop_x10 = 15;  // PDOP 1.5
        out.h_acc_m = 2;
        out.v_acc_m = 3;

        // Timestamp
        out.year = 2025;
        out.month = 1;
        out.day = 1;
        out.hour = 12;
        out.minute = 0;
        out.second = (uint8_t)((uint32_t)sim_time_s_ % 60);
        out.milli_second = (uint16_t)(fmodf(sim_time_s_, 1.0f) * 1000.0f);
    }

    void encodePOWER(uint32_t time_us, POWERData& out)
    {
        memset(&out, 0, sizeof(out));
        out.time_us = time_us;

        // Simulate steady 3.7V, 200mA, 80% SOC battery
        const float voltage_v = 3.7f;
        const float current_ma = 200.0f;
        const float soc_pct = 80.0f;

        out.voltage_raw = (uint16_t)lroundf((voltage_v / 10.0f) * 65535.0f);
        out.current_raw = (int16_t)lroundf((current_ma / 10000.0f) * 32767.0f);
        out.soc_raw = (int16_t)lroundf((soc_pct + 25.0f) * (32767.0f / 150.0f));
    }

    void encodeNonSensor(float alt_m, float vel_u_mps, uint8_t state,
                         uint8_t flags, uint32_t time_us, NonSensorData& out)
    {
        memset(&out, 0, sizeof(out));
        out.time_us = time_us;

        // Attitude (vertical = 90° pitch)
        out.roll = 0;
        out.pitch = (int16_t)(90 * 100);  // 90.00 degrees (centidegrees)
        out.yaw = 0;
        out.roll_cmd = 0;

        // Position (1D: only U component)
        out.e_pos = 0;
        out.n_pos = 0;
        out.u_pos = (int32_t)(alt_m * 100.0f);  // cm

        // Velocity (1D: only U component)
        out.e_vel = 0;
        out.n_vel = 0;
        out.u_vel = (int32_t)(vel_u_mps * 100.0f);  // cm/s

        out.flags = flags;
        out.rocket_state = state;

        // The sim's vertical velocity is the altitude rate
        out.baro_alt_rate_dmps = (int16_t)(vel_u_mps * 10.0f);
    }

    void encodeMMC5983MA(uint32_t time_us, MMC5983MAData& out)
    {
        memset(&out, 0, sizeof(out));
        out.time_us = time_us;

        // Simulated Earth magnetic field at ~38°N latitude (San Francisco area)
        // Approximate components: X(north) ≈ 22 µT, Y(east) ≈ 5 µT, Z(down) ≈ 42 µT
        // Encoding: raw = round(µT / UT_PER_COUNT) + 131072
        // where UT_PER_COUNT = (8 * 100) / 131072 ≈ 0.006104
        static constexpr float COUNTS_PER_UT = 131072.0f / 800.0f;

        out.mag_x = (uint32_t)(lroundf(22.0f * COUNTS_PER_UT) + 131072);
        out.mag_y = (uint32_t)(lroundf(5.0f  * COUNTS_PER_UT) + 131072);
        out.mag_z = (uint32_t)(lroundf(42.0f * COUNTS_PER_UT) + 131072);
    }
};

#endif // FLIGHT_SIMULATOR_H
