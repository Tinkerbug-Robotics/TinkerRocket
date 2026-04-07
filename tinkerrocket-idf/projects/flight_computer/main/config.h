#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

struct config 
{

    // ### SPI Pins ###
    static constexpr uint8_t SPI_SCK = 9;
    static constexpr uint8_t SPI_SDO = 10;
    static constexpr uint8_t SPI_SDI = 8;
    static constexpr uint32_t SPI_SPEED = 10'000'000; // 10 MHz

    // ### GNSS Serial Pins ###
    static constexpr uint8_t GNSS_RX = 3;  // 3
    static constexpr uint8_t GNSS_TX = 4;
    // Optional GNSS control pins. Set to -1 if not wired.
    static constexpr int8_t GNSS_RESET_N = -1;
    static constexpr int8_t GNSS_SAFEBOOT_N = -1;

    // ### Sensor Pins ###
    static constexpr uint8_t MMC5983MA_CS = 13;
    static constexpr uint8_t BMP585_CS = 6;
    static constexpr uint8_t ISM6HG256_CS = 7;

    // ### Sensors to use ###
    static constexpr bool USE_BMP585 = true;
    static constexpr bool USE_MMC5983MA = true;
    static constexpr bool USE_GNSS = true;
    static constexpr bool USE_ISM6HG256 = true;

    // ### Sensor Interrupt Pins ###
    static constexpr uint8_t ISM6HG256_INT = 21;
    static constexpr uint8_t BMP585_INT = 11;
    static constexpr uint8_t MMC5983MA_INT = 12;

    // ### Data Update Rates (Hz) ###
    static constexpr uint16_t FLIGHT_LOOP_UPDATE_RATE = 1000;
    // Requested > 10 Hz so the M10 runs at its 4-constellation ceiling
    // (~10 Hz).  The actual rate depends on satellite count; asking for
    // 18 Hz ensures the receiver doesn't self-limit below 10 Hz and the
    // poll task checks frequently enough (every 28 ms) to catch every fix.
    static constexpr uint16_t GNSS_UPDATE_RATE = 18;
    static constexpr uint16_t BMP585_UPDATE_RATE = 500;
    // MMC5983MA hardware supports 1/10/20/50/100/200/1000 Hz only.
    // 200 Hz is the highest step that fits within I2C budget.
    static constexpr uint16_t MMC5983MA_UPDATE_RATE = 200;
    static constexpr uint16_t ISM6HG256_UPDATE_RATE = 960;
    static constexpr uint16_t NON_SENSOR_UPDATE_RATE = 500;
    // ISM6 full-scale configuration used by SensorConverter (and shared to OUT).
    static constexpr uint8_t ISM6_LOW_G_FS_G = 16;
    static constexpr uint16_t ISM6_HIGH_G_FS_G = 256;
    static constexpr uint16_t ISM6_GYRO_FS_DPS = 4000;

    // ### Sensor Frame Rotations ###
    // ISM6 frame -> board frame rotation (deg), CCW positive about +Z.
    // Note: board frame: +X forward, +Y left, +Z up.
    static constexpr float ISM6HG256_ROT_Z_DEG = -45.0f;
    // MMC5983MA -> board frame (deg), CCW positive about +Z.
    // Note: board frame: +X forward, +Y left, +Z up.
    static constexpr float MMC5983MA_ROT_Z_DEG = 180.0f;

    // TODO Auto dection of orientation and rotation of aribtrary board position
    // to body frame

    // ### Camera Controls ###
    // Camera type: 0 = none, 1 = GoPro (GPIO pulse), 2 = RunCam (UART command)
    static constexpr uint8_t CAMERA_TYPE = 2;  // 0=none, 1=GoPro, 2=RunCam
    static constexpr bool USE_GOPRO = (CAMERA_TYPE == 1);
    static constexpr bool USE_RUNCAM = (CAMERA_TYPE == 2);

    // GoPro pins & timing
    static constexpr int8_t CAM_PWR_PIN = 30;        // powers on camera
    static constexpr int8_t CAM_SHUTTER_PIN = 31;    // GoPro shutter pulse
    static constexpr uint16_t GOPRO_PULSE_MS = 120;

    // RunCam UART pins & settings
    static constexpr int8_t RUNCAM_RX_PIN = 31;      // FC receives from RunCam
    static constexpr int8_t RUNCAM_TX_PIN = 32;      // FC sends to RunCam
    static constexpr int8_t RUNCAM_PWR_PIN = 30;     // powers on RunCam
    static constexpr uint32_t RUNCAM_BAUD = 115200;

    // ### Pyro Channel Pins ###
    static constexpr uint8_t PYRO1_ARM_PIN  = 14;
    static constexpr uint8_t PYRO1_FIRE_PIN = 15;
    static constexpr uint8_t PYRO1_CONT_PIN = 16;
    static constexpr uint8_t PYRO2_ARM_PIN  = 22;
    static constexpr uint8_t PYRO2_FIRE_PIN = 18;
    static constexpr uint8_t PYRO2_CONT_PIN = 19;
    static constexpr uint32_t PYRO_FIRE_DURATION_MS = 500;

    // ### Servo Controls (placeholder pins) ###
    // Set these to valid GPIOs for your board.
    static constexpr uint8_t SERVO_PIN_1 = 43;
    static constexpr uint8_t SERVO_PIN_2 = 44;
    static constexpr uint8_t SERVO_PIN_3 = 45;
    static constexpr uint8_t SERVO_PIN_4 = 46;

    static constexpr int SERVO_BIAS_1 = 0;
    static constexpr int SERVO_BIAS_2 = 0;
    static constexpr int SERVO_BIAS_3 = 0;
    static constexpr int SERVO_BIAS_4 = 0;

    static constexpr int SERVO_HZ = 333;
    static constexpr int SERVO_MIN_US = 1000;
    static constexpr int SERVO_MAX_US = 2000;

    static constexpr float ROLL_RATE_SET_POINT = 0.0f;
    // Tuned from Rolly Poly IV flight 2026-03-08 (Kp=0.04, Ki=0.001, Kd=0.0001)
    // K_plant ≈ 345 deg/s² per deg at V≈38 m/s. PM ≈ 64° at 2.2 Hz crossover.
    static constexpr float KP = 0.04f;
    static constexpr float KI = 0.001f;
    static constexpr float KD = 0.0003f;
    static constexpr float MIN_CMD = -10.0f;
    static constexpr float MAX_CMD = 10.0f;

    static constexpr bool USE_SERVO_CONTROL = true;
    static constexpr bool SERVO_WIGGLE_ON_BOOT = true;

    // EKF pad heading: compass heading of board Z+ (deg, 0=North, 90=East)
    static constexpr float PAD_HEADING_DEG = 0.0f;

    // GNSS quality gates — layered rejection of bad fixes
    // Note: h_acc is uint8_t (max 255). Many receivers report 255 = "unknown".
    // Set thresholds to 0 to disable h_acc gating when receiver doesn't populate it.
    static constexpr uint8_t  GNSS_MIN_SATS        = 4;      // minimum satellite count
    static constexpr float    GNSS_MAX_HACC_M       = 0.0f;  // max h_acc for EKF update (0 = disable)
    static constexpr float    GNSS_MAX_HACC_INIT_M  = 0.0f;  // max h_acc for EKF init  (0 = disable)
    static constexpr float    GNSS_MAX_VEL_INIT_MPS = 2.0f;  // max velocity magnitude for EKF init
    static constexpr uint8_t  GNSS_MIN_SATS_INIT    = 5;     // minimum sats for EKF init (stricter)

    // Barometer spike rejection: max altitude change (m) between consecutive
    // BMP585 samples before the reading is rejected.  At 500 Hz a 5 m jump
    // implies >2500 m/s — far beyond any model rocket — so real flight data
    // passes easily while ejection-charge pressure transients are rejected.
    static constexpr float BARO_SPIKE_THRESH_M = 5.0f;

    // Transonic barometer lockout: shock waves near Mach 1 cause large static
    // pressure errors (the "transonic jump").  Lock out baro updates when EKF
    // speed estimate exceeds LOCK_ON and keep them locked until speed drops
    // below LOCK_OFF (hysteresis prevents chatter at the boundary).
    // Set BARO_MACH_LOCKOUT_ON to 0 to disable.
    static constexpr float BARO_MACH_LOCKOUT_ON  = 260.0f;  // m/s (~Mach 0.76)
    static constexpr float BARO_MACH_LOCKOUT_OFF = 240.0f;  // m/s (~Mach 0.70)

    // EKF decimation: run EKF predict+update every Nth flight-loop iteration.
    // Navigation at 400–600 Hz is more than sufficient for rocket flight.
    // Setting this to 2 halves EKF compute load; the sensor read/log path
    // still runs every iteration so no IMU samples are lost.
    static constexpr uint8_t EKF_DECIMATION = 2;

    // Cascaded angle control: outer loop P-gain (rad/s per rad)
    static constexpr float KP_ANGLE = 4.0f;
    static constexpr bool USE_ANGLE_CONTROL = false;  // default; overridden at runtime via app

    // Roll control activation delay after launch (ms).  Keeps fins neutral
    // for this many ms after launch detection before engaging any roll control.
    static constexpr uint16_t ROLL_CONTROL_DELAY_MS = 0;  // default; overridden at runtime via app

    // Gain scheduling — V_ref is the speed where base gains apply.
    // Gains scale as (V_ref/V)², capped at 3.0. At low speed, gains
    // increase to compensate for reduced aerodynamic effectiveness.
    static constexpr bool  GAIN_SCHEDULE_ENABLED = true;
    static constexpr float GAIN_SCHEDULE_V_REF = 50.0f;
    static constexpr float GAIN_SCHEDULE_V_MIN = 25.0f;

    // ### Guidance (Proportional Navigation — coast only) ###
    // Textbook 3D PN (Yanushevsky Ch.2, eq 1.11 / 2.23)
    // Master enable (default off; overridden at runtime via NVS "guid_en")
    static constexpr bool GUIDANCE_ENABLED = false;
    // PN navigation constant (dimensionless, 3-5 typical)
    static constexpr float PN_NAV_GAIN = 5.0f;
    // Max lateral acceleration command (m/s^2)
    static constexpr float PN_MAX_ACCEL_MPS2 = 20.0f;
    // Target altitude above pad (m) — set above expected apogee
    static constexpr float PN_TARGET_ALT_M = 600.0f;
    // Outer-loop angle P gains (pitch and yaw)
    static constexpr float PN_KP_PITCH_ANGLE = 4.0f;
    static constexpr float PN_KP_YAW_ANGLE = 4.0f;
    // Pitch/yaw inner-loop rate PID gains (start same as roll)
    static constexpr float PN_PITCH_KP = 0.04f;
    static constexpr float PN_PITCH_KI = 0.001f;
    static constexpr float PN_PITCH_KD = 0.0003f;
    static constexpr float PN_YAW_KP = 0.04f;
    static constexpr float PN_YAW_KI = 0.001f;
    static constexpr float PN_YAW_KD = 0.0003f;
    // Max individual fin deflection in guided mode (deg)
    static constexpr float PN_MAX_FIN_DEG = 15.0f;
    // Minimum airspeed for guidance (below this, fins have no authority)
    static constexpr float PN_MIN_SPEED_MPS = 15.0f;
    // Delay after burnout before engaging guidance (ms)
    static constexpr uint16_t PN_COAST_DELAY_MS = 0;

    // ### Ground Test ###
    // Attitude-hold gain: virtual accel command per unit tilt (m/s² per rad)
    // Higher = more aggressive correction.  Only affects direction test on ground.
    static constexpr float GROUND_TEST_TILT_GAIN = 10.0f;
    // Accel-to-fin gain used on the ground (same as flight accel_to_fin_deg)
    static constexpr float GROUND_TEST_ACCEL_TO_FIN = 4.0f;

    // ### Indicators (Piezo and LED) ###
    static constexpr bool ENABLE_SOUNDS = false;
    static constexpr uint8_t PIEZO_PIN = 53;
    static constexpr uint8_t RED_LED_PIN = 2;
    static constexpr uint8_t BLUE_LED_PIN = 29;
    static constexpr uint16_t BLUE_LED_FLASH_MS = 40;
    static constexpr uint32_t HEARTBEAT_BEEP_INTERVAL_MS = 1000;
    static constexpr uint16_t HEARTBEAT_BEEP_FREQ_HZ = 2200;
    static constexpr uint16_t HEARTBEAT_BEEP_DURATION_MS = 50;
    // Set true if you also want periodic beeps during INFLIGHT.
    static constexpr bool HEARTBEAT_BEEP_IN_FLIGHT = false;

    // ### I2C Parameters (command/config channel only) ###
    static constexpr uint8_t ESP_I2C_ADR = 0x42;
    static constexpr uint8_t ESP_SDA_PIN = 41;
    static constexpr uint8_t ESP_SCL_PIN = 42;
    static constexpr uint32_t ESP_I2C_FREQ_HZ = 400'000; // Reduced — only commands now
    static constexpr uint16_t I2S_TX_QUEUE_LEN = 512;    // Must handle ~2160 frames/sec burst rate

    // ### I2S Parameters (high-frequency telemetry FC→OC) ###
    static constexpr int I2S_BCLK_PIN  = 27;
    static constexpr int I2S_WS_PIN    = 28;
    static constexpr int I2S_DOUT_PIN  = 23;
    static constexpr int I2S_FSYNC_PIN = 17;
    // I2S bandwidth = sample_rate * 4 bytes (16-bit stereo).
    // Higher rate = faster DMA buffer turnover = less stale data.
    // 22050 Hz = 88 KB/s.  Lower rates cause more gaps from DMA replay.
    // IMPORTANT: If sensor rates increase, raise this proportionally.
    static constexpr uint32_t I2S_SAMPLE_RATE = 22050;  // Must match OC
 
    // ### Execution Cores ###

    // Sensor (IMU, GNSS, and power) receiver
    static constexpr uint8_t SENSOR_CORE = 0;

};

#endif // CONFIG_H
