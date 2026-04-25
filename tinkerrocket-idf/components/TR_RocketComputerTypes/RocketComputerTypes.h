#ifndef ROCKETCOMPUTERTYPES_H
#define ROCKETCOMPUTERTYPES_H

#include <stdint.h>
#include <stddef.h>

// Logging state flag structure
union LogStateFlags 
{
    uint8_t all;
    struct 
    {
        uint8_t log        : 1;
        uint8_t end_flight : 1;
        uint8_t _unused    : 6;
    } bits;
};
static_assert(sizeof(LogStateFlags) == 1, 
              "LogStateFlags must be 1 byte");

// Data ready flag structure
union DataReadyFlags
{
    uint8_t all;
    struct
    {
        uint8_t gnss      : 1;
        uint8_t power     : 1;
        uint8_t bmp585    : 1;
        uint8_t ism6hg256 : 1;
        uint8_t mmc5983ma : 1;
        uint8_t nonsensor : 1;
        uint8_t logstate  : 1;
        uint8_t _unused   : 1;
    } bits;
};
static_assert(sizeof(DataReadyFlags) == 1, 
              "DataReadyFlags must be 1 byte");

// Rocket‐state enum
enum RocketState : uint8_t
{
    INITIALIZATION,
    READY,
    PRELAUNCH,
    INFLIGHT,
    LANDED
};

// ============================================================================
// Shared LoRa-recovery helpers (issue #71)
// ============================================================================
// Pure functions used by both the rocket and base station firmware so the
// behaviour stays bit-for-bit identical and is unit-testable from the host
// side without dragging in the radio/state machinery.

// Sticky frequency-lock-for-flight transition.  Given the previous lock
// state and the latest rocket state, returns the new lock state:
//   • INFLIGHT       → true  (latch on)
//   • READY/LANDED   → false (clear: back on the ground)
//   • everything else (INITIALIZATION/PRELAUNCH) → unchanged
// PRELAUNCH explicitly does NOT clear, so a post-flight LANDED→PRELAUNCH
// transition (rocket regains GPS while still on the ground) doesn't drop
// the lock prematurely.  Overload taking uint8_t exists because the base
// station receives state numerically from the LoRa downlink.
static inline bool computeFreqLockForFlight(bool prev_locked, RocketState s)
{
    if (s == INFLIGHT)                  return true;
    if (s == LANDED || s == READY)      return false;
    return prev_locked;
}

static inline bool computeFreqLockForFlight(bool prev_locked, uint8_t s)
{
    return computeFreqLockForFlight(prev_locked, (RocketState)s);
}

// Whether the rocket should be transmitting LoRa name beacons in this
// state.  Suppress only in INFLIGHT — telemetry needs that airtime, and
// the BS already knows where we are.  All other states (including
// INITIALIZATION) beacon: this lets the BS find the rocket even before
// the FlightComputer has finished booting and reported READY.
static inline bool shouldBeaconInState(RocketState s)
{
    return s != INFLIGHT;
}

// Payload sent with OUT_STATUS_QUERY so the OUT processor can configure
// its SensorConverter consistently with the FlightComputer.
typedef struct __attribute__((packed))
{
    uint8_t  ism6_low_g_fs_g;     // e.g. 16
    uint16_t ism6_high_g_fs_g;    // e.g. 256
    uint16_t ism6_gyro_fs_dps;    // e.g. 4000
    int16_t  ism6_rot_z_cdeg;     // centi-deg
    int16_t  mmc_rot_z_cdeg;      // centi-deg
    uint8_t  format_version;      // payload format version (2 = has HG bias)
    int16_t  hg_bias_x_cmss;     // high-g bias X, centi-m/s² (0.01 m/s² units)
    int16_t  hg_bias_y_cmss;     // high-g bias Y, centi-m/s²
    int16_t  hg_bias_z_cmss;     // high-g bias Z, centi-m/s²
} OutStatusQueryData;
static_assert(sizeof(OutStatusQueryData) == 16,
              "OutStatusQueryData must be 16 bytes");

// ### Data Structures ###
// Packed and unpacked data structures for each type ---

typedef struct __attribute__((packed))
{
    uint32_t time_us;

    uint16_t year;
    uint8_t  month;
    uint8_t  day;

    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint16_t milli_second;

    uint8_t  fix_mode;     // 0..5 (no fix → time only)
    uint8_t  num_sats;     // 0..255
    uint8_t  pdop_x10;     // PDOP * 10

    int32_t  lat_e7;       // deg * 1e7
    int32_t  lon_e7;       // deg * 1e7
    int32_t  alt_mm;       // mm

    int32_t  vel_e_mmps;   // mm/s
    int32_t  vel_n_mmps;
    int32_t  vel_u_mmps;

    uint8_t  h_acc_m;      // horizontal accuracy (m)
    uint8_t  v_acc_m;      // vertical accuracy (m)

} GNSSData;

static_assert(sizeof(GNSSData) == 42, 
              "GNSSData must be 42 bytes");

// --- GNSS ---
typedef struct 
{
    uint32_t time_us;
    uint16_t year;
    uint8_t month;
    uint8_t day; 
    uint8_t hour;
    uint8_t minute; 
    uint8_t second;
    uint16_t milli_second;
    uint8_t fix_mode; // 0: No Fix, 1: Dead Reckoning, 2: 2D Fix, 3: 3D Fix, 4:GNSS + Dead Reckoning, 5: Time Only
    uint8_t num_sats;
    float pdop;
    double lat; // deg
    double lon; // deg
    double alt; // m
    double vel_e; // m/s
    double vel_n; // m/s
    double vel_u; // m/s
    float horizontal_accuracy; // m
    float vertical_accuracy; // m
} GNSSDataSI;

// --- Power Data ---
typedef struct __attribute__((packed))
{
    uint32_t time_us;
    uint16_t voltage_raw; // (V / 10.0) * 65535        → 0..10 V
    int16_t  current_raw; // (mA / 10000.0) * 32767   → -10000..+10000 mA
    int16_t  soc_raw;     // (soc + 25) * (32767/150) → -25..+125 %

} POWERData;

static_assert(sizeof(POWERData) == 10, "POWERData must be 10 bytes");

typedef struct 
{
    uint32_t time_us;
    float voltage;
    float current;
    float soc;
} POWERDataSI;

// --- BMP585 Pressure and Temperature Data ---
typedef struct __attribute__((packed))
{
    uint32_t time_us;   // micros()
    int32_t  temp_q16;  // degC * 65536
    uint32_t press_q6;  // Pa * 64
} BMP585Data;
static_assert(sizeof(BMP585Data) == 12, 
              "BMP585Data must be 12 bytes");

typedef struct 
{
    uint32_t time_us;
    float pressure;
    float temperature;
} BMP585DataSI;

// --- ISM6HG256 IMU Data ---
typedef struct __attribute__((packed))
{
    int16_t x;
    int16_t y;
    int16_t z;
} Vec3i16;
static_assert(sizeof(Vec3i16) == 6, 
              "Vec3i16 must be 6 bytes");

typedef struct __attribute__((packed))
{
    uint32_t time_us;     // micros()

    Vec3i16 acc_low_raw;  // low-G accel raw counts
    Vec3i16 acc_high_raw; // high-G accel raw counts
    Vec3i16 gyro_raw;     // gyro raw counts

} ISM6HG256Data;

static_assert(sizeof(ISM6HG256Data) == 22,
              "ISM6HG256Data must be 22 bytes");

typedef struct 
{
    uint32_t time_us;
    double low_g_acc_x;
    double low_g_acc_y;
    double low_g_acc_z;
    double high_g_acc_x;
    double high_g_acc_y;
    double high_g_acc_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
} ISM6HG256DataSI;

// --- Magnetometer Data ---
typedef struct __attribute__((packed))
{
    uint32_t time_us;// micros()
    uint32_t mag_x;  // Raw counts
    uint32_t mag_y;
    uint32_t mag_z;

} MMC5983MAData;

static_assert(sizeof(MMC5983MAData) == 16, 
              "MMC5983MAData must be 16 bytes");

typedef struct 
{
    uint32_t time_us;
    double mag_x_uT; // Micro Tesla
    double mag_y_uT;
    double mag_z_uT;
} MMC5983MADataSI;

// --- Non sensor data ---
typedef struct __attribute__((packed))
{
    uint32_t time_us;

    // Attitude quaternion (unit quaternion * 10000, scalar-first)
    // q = q0 + q1*i + q2*j + q3*k,  |q| = 1
    // Range: -10000 .. 10000  (resolution: 0.0001 ≈ 0.01° at small angles)
    int16_t q0;
    int16_t q1;
    int16_t q2;
    int16_t q3;
    int16_t roll_cmd;   // deg * 100

    // Position (cm)
    int32_t e_pos;
    int32_t n_pos;
    int32_t u_pos;

    // Velocity (cm/s)
    int32_t e_vel;
    int32_t n_vel;
    int32_t u_vel;

    // Flags (bitfield)
    uint8_t flags;
    /*
        bit 0: alt_landed_flag
        bit 1: alt_apogee_flag
        bit 2: vel_u_apogee_flag
        bit 3: launch_flag
        bit 4: burnout_detected
        bit 5: guidance_active
        bit 6–7: reserved
    */

    uint8_t rocket_state; // RocketState enum

    // KF-filtered barometric altitude rate from FlightComputer (dm/s = 0.1 m/s)
    int16_t baro_alt_rate_dmps;

    // Pyro channel status (bitfield)
    uint8_t pyro_status;
    /*
        bit 0: ch1 continuity (1 = load present)
        bit 1: ch2 continuity (1 = load present)
        bit 2: ch1 fired
        bit 3: ch2 fired
        bit 4–7: reserved
    */

} NonSensorData;

static_assert(sizeof(NonSensorData) == 43,
              "NonSensorData must be 43 bytes");

static constexpr uint8_t NSF_ALT_LANDED   = (1u << 0);
static constexpr uint8_t NSF_ALT_APOGEE   = (1u << 1);
static constexpr uint8_t NSF_VEL_APOGEE   = (1u << 2);
static constexpr uint8_t NSF_LAUNCH       = (1u << 3);
static constexpr uint8_t NSF_BURNOUT      = (1u << 4);
static constexpr uint8_t NSF_GUIDANCE     = (1u << 5);
static constexpr uint8_t NSF_PYRO1_ARMED  = (1u << 6);
static constexpr uint8_t NSF_PYRO2_ARMED  = (1u << 7);

// Pyro status byte bit masks
static constexpr uint8_t PSF_CH1_CONT  = (1u << 0);
static constexpr uint8_t PSF_CH2_CONT  = (1u << 1);
static constexpr uint8_t PSF_CH1_FIRED = (1u << 2);
static constexpr uint8_t PSF_CH2_FIRED = (1u << 3);
static constexpr uint8_t PSF_REBOOT_RECOVERY = (1u << 4);  // mid-flight reboot recovery occurred
// Reuse of this byte for a non-pyro signal: the FlightComputer's live
// guidance_enabled config. OutComputer uses this as the source of truth
// to avoid iOS/OUT/FC NVS caches silently diverging.
static constexpr uint8_t PSF_GUIDANCE_ENABLED = (1u << 5);

typedef struct
{
    uint32_t time_us;

    // Quaternion (scalar-first, unit quaternion)
    float q0;
    float q1;
    float q2;
    float q3;

    // Euler angles (derived from quaternion, for display)
    float roll;      // deg
    float pitch;     // deg
    float yaw;       // deg
    float roll_cmd;  // deg

    double e_pos;    // m
    double n_pos;
    double u_pos;

    double e_vel;    // m/s
    double n_vel;
    double u_vel;

    float pressure_alt;   // m
    float altitude_rate;  // m/s
    float max_alt;        // m
    float max_speed;      // m/s

    bool alt_landed_flag;
    bool alt_apogee_flag;
    bool vel_u_apogee_flag;
    bool launch_flag;

    RocketState rocket_state;

} NonSensorDataSI;

// --- LoRa Data ---

typedef struct __attribute__((packed)) 
{
    // little-endian signed 24-bit stored in 3 bytes
    uint8_t b0, b1, b2; 
} i24le_t;
static_assert(sizeof(i24le_t) == 3, "i24le_t must be 3 bytes");

// LoRa protocol version — bump on frame format changes
static constexpr uint8_t LORA_PROTO_VERSION = 1;

// LoRa name beacon sync byte (distinguishes from telemetry by size + prefix)
static constexpr uint8_t LORA_BEACON_SYNC = 0xBE;

// Heartbeat uplink command (issue #71).  Sent by the base station roughly
// every 30 s while it's actively hearing rocket telemetry, so the rocket
// has positive proof of comms in the absence of any user-initiated
// uplink.  Without this, the rocket's slow-rendezvous timer would expire
// during a passive monitoring session and waste airtime visiting the
// rendezvous frequency.  The handler is a no-op — last_uplink_rx_ms
// updates unconditionally on any successfully decoded uplink, which is
// all the rocket needs to keep the timer reset.
static constexpr uint8_t LORA_CMD_HEARTBEAT = 0xFE;

// LoRa data to send from rocket to ground station
typedef struct __attribute__((packed))
{
    // --- Routing header (added in proto v1) ---
    uint8_t network_id;      // LoRa network namespace (0..255)
    uint8_t rocket_id;       // Source rocket ID within network (1..254, 0=unset, 255=broadcast)

    // --- Telemetry payload (unchanged from proto v0) ---
    uint8_t num_sats;        // 0..255
    uint8_t pdop_u8;         // 0..100 (as you do now)

    i24le_t ecef_x_m;        // meters, signed 24-bit
    i24le_t ecef_y_m;
    i24le_t ecef_z_m;

    uint8_t hacc_u8;         // 0..100

    uint8_t flags_state;     // bits 0..3 flags, bits 4..6 rocket_state

    int16_t acc_x_x10;       // m/s^2 * 10
    int16_t acc_y_x10;
    int16_t acc_z_x10;

    int16_t gyro_x_x10;      // deg/s * 10
    int16_t gyro_y_x10;
    int16_t gyro_z_x10;

    int16_t temp_x10;        // degC * 10

    uint8_t voltage_u8;      // encodeVoltage_2_10_01()

    int16_t current_ma;      // mA

    int8_t  soc_i8;          // -128..127

    i24le_t pressure_alt_m;  // meters

    int16_t altitude_rate;   // m/s

    i24le_t max_alt_m;       // meters

    int16_t max_speed;       // m/s

    int16_t roll_cd;         // centideg
    int16_t pitch_cd;
    int16_t yaw_cd;

    int16_t q0;              // quaternion × 10000
    int16_t q1;
    int16_t q2;
    int16_t q3;

    int16_t speed;           // m/s

} LoRaData;

static_assert(sizeof(LoRaData) == 59,
              "LoRaData must be 59 bytes (2-byte header + 57-byte payload)");

static constexpr uint8_t LORA_LAUNCH      = (1u << 0);  // bit 0
static constexpr uint8_t LORA_VEL_APOGEE  = (1u << 1);  // bit 1
static constexpr uint8_t LORA_ALT_APOGEE  = (1u << 2);  // bit 2
static constexpr uint8_t LORA_ALT_LANDED  = (1u << 3);  // bit 3
static constexpr uint8_t LORA_STATE_SHIFT = 4;           // bits 4-6: rocket state
static constexpr uint8_t LORA_CAMERA_REC  = (1u << 7);  // bit 7: camera recording

// logging_active is packed into the MSB of num_sats (real range 0-40, 7 bits plenty)
static constexpr uint8_t LORA_LOGGING_BIT = 0x80;

// Readable LoRa data structure
typedef struct
{                                   // Precision    : Range
    uint8_t network_id;             // Routing header: network namespace
    uint8_t rocket_id;              // Routing header: source rocket ID
    uint8_t num_sats;               // int          : 0 to 255
    float   pdop;                   // meter        : 0 to 100
    double  ecef_x, ecef_y, ecef_z; // meter        : +/- 7,000,000
    float   horizontal_accuracy;    // meter        : 0 to 100
    bool    alt_landed_flag,        // bit          : bool
            alt_apogee_flag,        // bit          : bool
            vel_u_apogee_flag,      // bit          : bool
            launch_flag,            // bit          : bool
            camera_recording,       // bit 7        : bool
            logging_active;         // MSB of num_sats byte
    uint8_t rocket_state;           // 3 bits       : states 0 through 4
    float   acc_x;                  // 0.1 m/s2     : -400 to 400
    float   acc_y;                  // 0.1 m/s2     : -400 to 400
    float   acc_z;                  // 0.1 m/s2     : -400 to 400
    float   gyro_x;                 // 0.1 deg/s    : +/- 4500
    float   gyro_y;                 // 0.1 deg/s    : +/- 4500
    float   gyro_z;                 // 0.1 deg/s    : +/- 4500
    float   temp;                   // 0.1 deg      : -40 to 200
    float   voltage;                // 0.1          : 3 to 10
    float   current;                // 1 mA         : -10000 to 10000
    float   soc;                    // 1%           : -25 to 125
    float   base_station_voltage;   // N/A
    float   base_station_current;   // N/A
    float   base_station_soc;       // N/A
    float   rssi;                   // N/A
    float   snr;                    // N/A
    float   pressure_alt;           // 1 m          : -1000 to 100000
    float   altitude_rate;          // 1 m/s        : -2000 to 2000
    float   max_alt;                // 1 m          : -1000 to 400000
    float   max_speed;              // 1 m/s        : 0 to 4000
    float   roll;                   // int16_t      : -180 to 180
    float   pitch;                  // int16_t      : -90 to 90
    float   yaw;                    // int16_t      : -180 to 180
    float   q0, q1, q2, q3;        // quaternion   : -1 to 1
    float   speed;                  // 1 m/s        : 0 to 4000
} LoRaDataSI;


// ### Register Read Addressess ###
static constexpr uint8_t REG_MMC5983MA_DATA = 0x00;
static constexpr uint8_t REG_BMP585_DATA    = 0x01;
static constexpr uint8_t REG_GNSS_DATA      = 0x02;
static constexpr uint8_t REG_ISM6HG256_DATA = 0x03;
static constexpr uint8_t REG_POWER_DATA     = 0x04;
static constexpr uint8_t REG_NONSENSOR_DATA = 0x05;
static constexpr uint8_t REG_DATA_READY     = 0x06;
static constexpr uint8_t REG_LOG_STATE      = 0x07;
static constexpr uint8_t REG_TEST           = 0x08;

// ### Message Types from In ESP32 ###
static constexpr uint8_t OUT_STATUS_QUERY    = 0xA0;
static constexpr uint8_t GNSS_MSG            = 0xA1;
static constexpr uint8_t ISM6HG256_MSG       = 0xA2;
static constexpr uint8_t BMP585_MSG          = 0xA3;
static constexpr uint8_t MMC5983MA_MSG       = 0xA4;
static constexpr uint8_t NON_SENSOR_MSG      = 0xA5;
static constexpr uint8_t POWER_MSG           = 0xA6;
static constexpr uint8_t START_LOGGING       = 0xA7;
static constexpr uint8_t END_FLIGHT          = 0xA8;
static constexpr uint8_t OUT_STATUS_RESPONSE = 0xA9;
static constexpr uint8_t CAMERA_START        = 0xAA;
static constexpr uint8_t CAMERA_STOP         = 0xAB;
static constexpr uint8_t SOUNDS_ENABLE       = 0xAC;
static constexpr uint8_t SOUNDS_DISABLE      = 0xAD;
static constexpr uint8_t SERVO_CONFIG_PENDING = 0xAE;
static constexpr uint8_t PID_CONFIG_PENDING   = 0xAF;
static constexpr uint8_t SERVO_CONFIG_MSG     = 0xB0;
static constexpr uint8_t PID_CONFIG_MSG       = 0xB1;
static constexpr uint8_t SERVO_CTRL_ENABLE    = 0xB2;
static constexpr uint8_t SERVO_CTRL_DISABLE   = 0xB3;
static constexpr uint8_t SIM_CONFIG_PENDING   = 0xB4;
static constexpr uint8_t SIM_CONFIG_MSG       = 0xB5;
static constexpr uint8_t SIM_START_CMD        = 0xB6;
static constexpr uint8_t SIM_STOP_CMD         = 0xB7;
static constexpr uint8_t GROUND_TEST_START   = 0xB8;
static constexpr uint8_t GROUND_TEST_STOP    = 0xB9;
static constexpr uint8_t GYRO_CAL_CMD        = 0xBA;
static constexpr uint8_t GAIN_SCHED_ENABLE   = 0xBB;
static constexpr uint8_t GAIN_SCHED_DISABLE  = 0xBC;
static constexpr uint8_t SERVO_TEST_PENDING  = 0xBD;
static constexpr uint8_t SERVO_TEST_MSG      = 0xBE;
static constexpr uint8_t SERVO_TEST_STOP     = 0xBF;
static constexpr uint8_t ROLL_PROFILE_PENDING = 0xC0;
static constexpr uint8_t ROLL_PROFILE_MSG     = 0xC1;
static constexpr uint8_t ROLL_PROFILE_CLEAR   = 0xC2;
static constexpr uint8_t SERVO_REPLAY_PENDING = 0xC3;
static constexpr uint8_t SERVO_REPLAY_MSG     = 0xC4;
static constexpr uint8_t SERVO_REPLAY_STOP    = 0xC5;
static constexpr uint8_t ROLL_CTRL_CONFIG_PENDING = 0xC6;
static constexpr uint8_t ROLL_CTRL_CONFIG_MSG     = 0xC7;
static constexpr uint8_t GUIDANCE_ENABLE          = 0xC8;
static constexpr uint8_t GUIDANCE_DISABLE         = 0xC9;
static constexpr uint8_t GUIDANCE_TELEM_MSG       = 0xCA;
static constexpr uint8_t CAMERA_CONFIG_PENDING    = 0xCB;
static constexpr uint8_t CAMERA_CONFIG_MSG        = 0xCC;
static constexpr uint8_t PYRO_CONFIG_PENDING      = 0xCD;
static constexpr uint8_t PYRO_CONFIG_MSG          = 0xCE;
static constexpr uint8_t PYRO_CONT_TEST           = 0xCF;  // momentary arm → read continuity → disarm
static constexpr uint8_t PYRO_FIRE_TEST            = 0xD0;  // test-fire a pyro channel from app
static constexpr uint8_t LORA_MSG            = 0xF1;

// Camera types
static constexpr uint8_t CAM_TYPE_NONE   = 0;
static constexpr uint8_t CAM_TYPE_GOPRO  = 1;
static constexpr uint8_t CAM_TYPE_RUNCAM = 2;

// Camera config data (1 byte)
typedef struct __attribute__((packed))
{
    uint8_t camera_type;  // CAM_TYPE_NONE, CAM_TYPE_GOPRO, CAM_TYPE_RUNCAM
} CameraConfigData;
static_assert(sizeof(CameraConfigData) == 1, "CameraConfigData must be 1 byte");

// Pyro trigger modes
enum PyroTriggerMode : uint8_t {
    PYRO_TRIGGER_TIME_AFTER_APOGEE    = 0,
    PYRO_TRIGGER_ALTITUDE_ON_DESCENT  = 1,
};

// Pyro channel configuration (both channels, 12 bytes)
typedef struct __attribute__((packed))
{
    uint8_t  ch1_enabled;       // 0 = disabled, 1 = enabled
    uint8_t  ch1_trigger_mode;  // PyroTriggerMode
    float    ch1_trigger_value; // seconds (time mode) or meters (altitude mode)
    uint8_t  ch2_enabled;
    uint8_t  ch2_trigger_mode;
    float    ch2_trigger_value;
} PyroConfigData;
static_assert(sizeof(PyroConfigData) == 12, "PyroConfigData must be 12 bytes");

// Packed config data structures for BLE → I2C relay
typedef struct __attribute__((packed))
{
    int16_t bias_us[4];   // Per-servo bias in microseconds
    int16_t hz;           // PWM frequency
    int16_t min_us;       // Minimum pulse width
    int16_t max_us;       // Maximum pulse width
} ServoConfigData;
static_assert(sizeof(ServoConfigData) == 14, "ServoConfigData must be 14 bytes");

typedef struct __attribute__((packed))
{
    float kp;
    float ki;
    float kd;
    float min_cmd;
    float max_cmd;
} PIDConfigData;
static_assert(sizeof(PIDConfigData) == 20, "PIDConfigData must be 20 bytes");

typedef struct __attribute__((packed))
{
    float mass_kg;
    float thrust_n;
    float burn_time_s;
    float descent_rate_mps;
} SimConfigData;
static_assert(sizeof(SimConfigData) == 16, "SimConfigData must be 16 bytes");

typedef struct __attribute__((packed))
{
    int16_t angle_cdeg[4];  // Per-servo angle in centi-degrees (-2000 to +2000)
} ServoTestAnglesData;
static_assert(sizeof(ServoTestAnglesData) == 8, "ServoTestAnglesData must be 8 bytes");

// --- Servo Replay Data ---
// Replays recorded flight data through the control loop to observe servo response
typedef struct __attribute__((packed))
{
    int16_t roll_rate_cdps;   // centi-deg/s  (roll rate * 100)
    int16_t speed_cmps;       // centi-m/s    (speed * 100)
} ServoReplayData;
static_assert(sizeof(ServoReplayData) == 4, "ServoReplayData must be 4 bytes");

// --- Roll Profile Data ---
// Max 8 waypoints: fits in single BLE MTU with header
static constexpr uint8_t MAX_ROLL_WAYPOINTS = 8;

typedef struct __attribute__((packed))
{
    float time_s;       // seconds after launch
    float angle_deg;    // target roll angle (deg)
} RollWaypoint;
static_assert(sizeof(RollWaypoint) == 8, "RollWaypoint must be 8 bytes");

typedef struct __attribute__((packed))
{
    uint8_t num_waypoints;      // 0 = no profile (rate-only mode)
    uint8_t _pad[3];            // alignment padding
    RollWaypoint waypoints[MAX_ROLL_WAYPOINTS];
} RollProfileData;
static_assert(sizeof(RollProfileData) == 68, "RollProfileData must be 68 bytes");

// --- Roll Control Config (runtime-configurable from app) ---
typedef struct __attribute__((packed))
{
    uint8_t  use_angle_control;   // 0 = rate-only (null roll), 1 = cascaded angle control w/ profile
    uint8_t  _pad;
    uint16_t roll_delay_ms;       // ms after launch before any roll control activates
} RollControlConfigData;
static_assert(sizeof(RollControlConfigData) == 4, "RollControlConfigData must be 4 bytes");

// --- Guidance Telemetry Data (sent at ~10 Hz during coast) ---
typedef struct __attribute__((packed))
{
    uint32_t time_us;
    int16_t  pitch_cmd_cdeg;    // guidance pitch command (centi-deg)
    int16_t  yaw_cmd_cdeg;      // guidance yaw command (centi-deg)
    int16_t  lateral_offset_cm; // lateral distance from pad vertical (cm)
    int16_t  pitch_fin_cdeg;    // pitch fin command after PID (centi-deg)
    int16_t  yaw_fin_cdeg;      // yaw fin command after PID (centi-deg)
    uint8_t  guid_flags;        // bit 0: guidance_active, bit 1: burnout_detected
} GuidanceTelemData;
static_assert(sizeof(GuidanceTelemData) == 15, "GuidanceTelemData must be 15 bytes");

static constexpr size_t SIZE_OF_GNSS_DATA = sizeof(GNSSData);
static constexpr size_t SIZE_OF_BMP585_DATA     = sizeof(BMP585Data);
static constexpr size_t SIZE_OF_ISM6HG256_DATA  = sizeof(ISM6HG256Data);
static constexpr size_t SIZE_OF_MMC5983MA_DATA  = sizeof(MMC5983MAData);
static constexpr size_t SIZE_OF_POWER_DATA      = sizeof(POWERData);
static constexpr size_t SIZE_OF_NON_SENSOR_DATA = sizeof(NonSensorData);
static constexpr size_t SIZE_OF_LORA_DATA       = sizeof(LoRaData);
static constexpr size_t SIZE_OF_LORA_DATA_SI = sizeof(LoRaDataSI);

// Calculate max message size to get MAX_PAYLOAD and MAX_FRAME
static constexpr size_t P1 = SIZE_OF_ISM6HG256_DATA;
static constexpr size_t P2 = SIZE_OF_BMP585_DATA;
static constexpr size_t P3 = SIZE_OF_MMC5983MA_DATA;
static constexpr size_t P4 = SIZE_OF_GNSS_DATA;
static constexpr size_t P5 = SIZE_OF_POWER_DATA;
static constexpr size_t P6 = SIZE_OF_NON_SENSOR_DATA;
static constexpr size_t P7 = SIZE_OF_LORA_DATA;
static constexpr size_t P8 = sizeof(RollProfileData);

static constexpr size_t M12   = (P1 > P2 ? P1 : P2);
static constexpr size_t M34   = (P3 > P4 ? P3 : P4);
static constexpr size_t M56   = (P5 > P6 ? P5 : P6);
static constexpr size_t M567  = (M56 > P7 ? M56 : P7);
static constexpr size_t M1234 = (M12 > M34 ? M12 : M34);
static constexpr size_t M_SENSOR = (M1234 > M567 ? M1234 : M567);

static constexpr size_t MAX_PAYLOAD = (M_SENSOR > P8 ? M_SENSOR : P8);

// Frame: [0xAA][0x55][0xAA][0x55] + type + length + payload + CRC16
static constexpr size_t MAX_FRAME = 4 + 1 + 1 + MAX_PAYLOAD + 2;
#endif
