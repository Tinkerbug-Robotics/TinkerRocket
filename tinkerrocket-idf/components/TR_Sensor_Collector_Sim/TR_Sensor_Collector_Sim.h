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
    bool getIIS2MDCData(IIS2MDCData& data_out);
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

    // Mag hard-iron offset passthrough (issue #96).  In sim, the IIS2MDC
    // path is offline so the setter is a no-op; isIIS2MDCActive() returns
    // the real-collector's state regardless of sim mode so the boot apply
    // path still works after reverting from sim to live.
    bool isIIS2MDCActive() const { return real_.isIIS2MDCActive(); }
    bool setIIS2MDCHardIronOffset(int16_t cx, int16_t cy, int16_t cz);

    // ---- Sim control ----
    void configureSim(const SimConfigData& cfg);
    void configureSimRotation(float ism6_rot_z_deg);
    // #463: sensor→board rotation of the IIS2MDC, so the synthetic mag comes
    // out geodetically correct after the converter re-applies the forward
    // rotation (mirrors configureSimRotation for the ISM6).
    void configureSimIis2mdcRotation(float rot_z_deg);
    // Board→rocket mounting rotation (row-major, v_rocket = R * v_board).
    // The sim's physics are rocket-frame; encoders apply the INVERSE so the
    // forward conversion (chip rotZ, then board→rocket) reproduces the
    // simulated values regardless of the configured mounting.
    void configureSimBoardToRocket(const float R[9]);

    // Pad-frame alignment (#508). The sim's physics assume the rocket stands on
    // the rail (pitch = LAUNCH_ANGLE_RAD), but on a bench the board is usually
    // lying flat — so at the instant sim data replaces real data the rocket-frame
    // gravity vector jumps by ~90°. The EKF has accel/mag updates live on the pad
    // and can only explain a large attitude correction with no measured rate by
    // blaming the gyro bias, which then slams to a physically impossible value and
    // is frozen in at launch (accel/mag are gated off in flight) — wrecking
    // attitude for the whole run.
    //
    // Fix the discontinuity at the source: pass the board's ACTUAL resting
    // attitude (rocket-frame "up" from the orientation estimator, plus the
    // rocket-frame mag) and we solve for the fixed rotation that carries the
    // sim's pad attitude onto it. Every synthetic body-frame vector (accel, gyro,
    // mag) is then rotated by it, so the handover is continuous by construction.
    //
    // Rotating the body-frame vectors AND the implied attitude by the same R
    // leaves world-frame acceleration unchanged, so GNSS/baro need no adjustment
    // — the simulated trajectory is identical, only the body frame is re-expressed.
    //
    // `up_rocket` is the unit specific-force ("up") direction in rocket frame.
    // `mag_rocket` is the rocket-frame field in µT; pass nullptr (or a
    // non-physical magnitude) to align on gravity alone.
    void configureSimPadAlignment(const float up_rocket[3], const float mag_rocket[3]);

    // True once a pad alignment has been solved (diagnostics / tests).
    bool  hasPadAlignment() const { return !pad_align_identity_; }
    // Angle of the pad-alignment rotation, degrees (0 when identity).
    float padAlignmentAngleDeg() const;

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
    float accel_mps2_       = 0.0f;   // net acceleration (vertical, excludes gravity)
    float sim_time_s_       = 0.0f;
    float ground_pressure_pa_ = 101325.0f;

    // Attitude state (pitch-plane only, 0 = horizontal, π/2 = vertical nose-up)
    float pitch_rad_        = 0.0f;
    float pitch_rate_rps_   = 0.0f;

    // Inverse rotation (body frame → sensor frame) for ISM6 encoding
    float ism6_inv_c_ = 1.0f;   // cos(config_angle)
    float ism6_inv_s_ = 0.0f;   // sin(config_angle)

    // Inverse rotation (board frame → sensor frame) for IIS2MDC encoding (#463)
    float iis_inv_c_ = 1.0f;
    float iis_inv_s_ = 0.0f;

    // Inverse board→rocket rotation (rocket frame → board frame), i.e. the
    // transpose of the configured mounting matrix.  Identity by default.
    float b2r_inv_[9] = {1.0f, 0.0f, 0.0f,
                         0.0f, 1.0f, 0.0f,
                         0.0f, 0.0f, 1.0f};
    bool  b2r_identity_ = true;

    inline void rocketToBoard(float &x, float &y, float &z) const
    {
        if (b2r_identity_) return;
        const float bx = b2r_inv_[0] * x + b2r_inv_[1] * y + b2r_inv_[2] * z;
        const float by = b2r_inv_[3] * x + b2r_inv_[4] * y + b2r_inv_[5] * z;
        const float bz = b2r_inv_[6] * x + b2r_inv_[7] * y + b2r_inv_[8] * z;
        x = bx; y = by; z = bz;
    }

    // Pad-frame alignment (#508), applied to every synthetic ROCKET-frame vector
    // before rocketToBoard(). Identity until configureSimPadAlignment() runs, so
    // an un-aligned sim behaves exactly as before.
    float pad_align_[9] = {1.0f, 0.0f, 0.0f,
                           0.0f, 1.0f, 0.0f,
                           0.0f, 0.0f, 1.0f};
    bool  pad_align_identity_ = true;

    inline void applyPadAlign(float &x, float &y, float &z) const
    {
        if (pad_align_identity_) return;
        const float rx = pad_align_[0] * x + pad_align_[1] * y + pad_align_[2] * z;
        const float ry = pad_align_[3] * x + pad_align_[4] * y + pad_align_[5] * z;
        const float rz = pad_align_[6] * x + pad_align_[7] * y + pad_align_[8] * z;
        x = rx; y = ry; z = rz;
    }

    // The sim's own pad-attitude reference vectors in rocket frame, at
    // pitch = LAUNCH_ANGLE_RAD. These are what configureSimPadAlignment maps
    // ONTO the measured pad vectors.
    void simPadReference(float up_out[3], float mag_out[3]) const;

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

    // Synthetic Earth field at ~38N (NED, µT). Shared by both mag encoders and
    // by simPadReference() so the alignment reference cannot drift from what the
    // encoders actually emit.
    static constexpr float B_NORTH = 22.0f;
    static constexpr float B_EAST  =  5.0f;
    static constexpr float B_DOWN  = 42.0f;

    // Attitude dynamics defaults (typical model rocket)
    static constexpr float LAUNCH_ANGLE_RAD = 85.0f * 3.14159265f / 180.0f;  // 5° from vertical
    static constexpr float CP_CG_DIST_M    = 0.075f;   // 1.5 cal stability margin (50mm body)
    static constexpr float I_TRANSVERSE     = 0.02f;    // kg⋅m² (500g, 0.6m rocket)
    static constexpr float C_DAMP           = 20.0f;    // pitch damping coefficient

    // ISM6 sensitivity constants (for ±16g low-G, ±256g high-G, ±4000dps gyro)
    static constexpr float ACC_LOW_MS2_PER_LSB  = (16.0f * 1000.0f / 32768.0f) * 1.0e-3f * GRAVITY;
    static constexpr float ACC_HIGH_MS2_PER_LSB = (256.0f * 1000.0f / 32768.0f) * 1.0e-3f * GRAVITY;
    static constexpr float GYRO_DPS_PER_LSB     = (4000.0f * 1000.0f / 32768.0f) * 1.0e-3f;

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
    void encodeIIS2MDC(uint32_t time_us, IIS2MDCData& out);
    void encodeGNSS(uint32_t time_us, GNSSData& out);
};

#endif // SENSOR_COLLECTOR_SIM_H
