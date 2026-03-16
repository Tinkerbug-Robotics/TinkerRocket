#ifndef SENSOR_COLLECTOR_SIM_H
#define SENSOR_COLLECTOR_SIM_H

//
//  TR_Sensor_Collector_Sim.h
//  TinkerRocket FlightComputer
//
//  Drop-in wrapper around SensorCollector that can replace real sensor
//  data with simulated flight data.  When sim is inactive, all getters
//  pass through to the real collector unchanged.  When sim is active,
//  real sensors are still read (to maintain hardware timing/rates) but
//  the output values are replaced with 1D physics simulation data.
//
//  Everything downstream of SensorCollector — state machine, kinematics,
//  IMU integration, servo control, I2C telemetry, logging — sees the
//  simulated data as if it were real sensor data.
//

#include <TR_Sensor_Collector.h>
#include <RocketComputerTypes.h>

class SensorCollectorSim
{
public:
    SensorCollectorSim(SensorCollector& real);

    // Delegates to real collector
    void begin(uint8_t imu_execution_core);

    // Same interface as SensorCollector.
    // When sim inactive: passthrough.
    // When sim active:   consume real data (for timing), return simulated values.
    bool getISM6HG256Data(ISM6HG256Data& data_out);
    bool getBMP585Data(BMP585Data& data_out);
    bool getMMC5983MAData(MMC5983MAData& data_out);
    bool getGNSSData(GNSSData& data_out);

    // Debug passthrough
    void getISM6HG256DebugSnapshot(ISM6HG256DebugSnapshot& snapshot_out) const;
    void getMMC5983MADebugSnapshot(MMC5983MADebugSnapshot& snapshot_out) const;
    void getPollTimingSnapshot(PollTimingSnapshot& snapshot_out) const;
    void resetPollTimingSnapshot();

    // Calibration (no-op in sim — synthetic data has no bias)
    void calibrateGyro(float rotation_z_deg = 0.0f);
    float hg_bias_x = 0.0f, hg_bias_y = 0.0f, hg_bias_z = 0.0f;
    float cal_gravity_mag = 0.0f;

    // ---- Sim control ----
    void configureSim(const SimConfigData& cfg);
    void configureSimRotation(float ism6_rot_z_deg);
    void startSim(float ground_pressure_pa);
    void stopSim();
    bool isSimActive() const;
    bool isSimConfigured() const { return configured_; }

private:
    // ========================================================================
    // Sim phases
    // ========================================================================
    enum SimPhase : uint8_t
    {
        SIM_IDLE = 0,
        SIM_PRELAUNCH,
        SIM_POWERED,
        SIM_COASTING,
        SIM_DESCENT,
        SIM_LANDED
    };

    // ========================================================================
    // Members
    // ========================================================================
    SensorCollector& real_;

    SimConfigData cfg_         = {};
    bool          configured_  = false;
    SimPhase      phase_       = SIM_IDLE;
    uint32_t      phase_start_ms_ = 0;
    uint32_t      last_step_us_   = 0;

    // Physics state (SI)
    float altitude_m_       = 0.0f;
    float velocity_mps_     = 0.0f;   // positive = up
    float accel_mps2_       = 0.0f;   // net acceleration (body X = forward/long axis)
    float sim_time_s_       = 0.0f;
    float ground_pressure_pa_ = 101325.0f;

    // Inverse rotation (body frame → sensor frame) for ISM6 encoding
    float ism6_inv_c_ = 1.0f;   // cos(config_angle)
    float ism6_inv_s_ = 0.0f;   // sin(config_angle)

    // GNSS fallback timer (for indoor testing without GPS fix)
    uint32_t last_gnss_real_us_ = 0;
    uint32_t gnss_timer_us_     = 0;
    static constexpr uint32_t GNSS_FALLBACK_INTERVAL_US = 40000;  // 25 Hz
    static constexpr uint32_t GNSS_FALLBACK_TIMEOUT_US  = 100000; // 100 ms

    // ========================================================================
    // Physics constants
    // ========================================================================
    static constexpr float GRAVITY        = 9.80665f;
    static constexpr float CD             = 0.5f;
    static constexpr float REFERENCE_AREA = 0.0019635f; // pi * 0.025^2 (50mm dia)
    static constexpr float PRELAUNCH_DURATION_MS = 5000.0f;

    // ISM6 sensitivity constants (for ±16g low-G, ±256g high-G)
    static constexpr float ACC_LOW_MS2_PER_LSB  = (16.0f * 1000.0f / 32768.0f) * 1.0e-3f * GRAVITY;
    static constexpr float ACC_HIGH_MS2_PER_LSB = (256.0f * 1000.0f / 32768.0f) * 1.0e-3f * GRAVITY;

    // ========================================================================
    // Physics methods
    // ========================================================================
    void stepPhysics(float dt);
    void stepPrelaunch(uint32_t elapsed_ms);
    void stepPowered(float dt);
    void stepCoasting(float dt);
    void stepDescent(float dt);

    float airDensity(float alt_m) const;
    float pressureAtAltitude(float alt_m) const;
    float dragAccel(float velocity, float alt) const;

    // ========================================================================
    // Sensor encoding (inverse of SensorConverter formulas)
    // ========================================================================
    void encodeISM6(uint32_t time_us, ISM6HG256Data& out);
    void encodeBMP585(uint32_t time_us, BMP585Data& out);
    void encodeMMC5983MA(uint32_t time_us, MMC5983MAData& out);
    void encodeGNSS(uint32_t time_us, GNSSData& out);
};

#endif // SENSOR_COLLECTOR_SIM_H
