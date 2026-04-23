#include <compat.h>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <string>
#include <algorithm>
#include <esp_sleep.h>
#include <esp_pm.h>
#include <TR_NVS.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <esp_mac.h>              // esp_efuse_mac_get_default for unit_id
#include "soc/rtc_cntl_reg.h"  // Brownout detector control
#include "freertos/queue.h"
#include "host/ble_gap.h"       // ble_gap_update_params

// Nimble's os.h (pulled in by host/ble_gap.h) defines `max` and `min` as
// function-style macros, which collide with std::max / std::min. Undo them
// here so subsequent <algorithm> calls compile cleanly.
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

// std::string is used as "String" in non-Arduino builds (same typedef as TR_BLE_To_APP.h)
using String = std::string;

// Helper: format float to std::string with N decimal places (replaces Arduino String(float, decimals))
static inline std::string fmtf(float v, int decimals)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%.*f", decimals, (double)v);
    return std::string(buf);
}

// Helper: format integer to std::string (replaces Arduino String(int))
static inline std::string itos(int v)
{
    return std::to_string(v);
}

#include "config.h"

#include <TR_I2C_Interface.h>
#include <TR_I2S_Stream.h>
#include <TR_LogToFlash.h>
#include <TR_LoRa_Comms.h>
#include <TR_Sensor_Data_Converter.h>
#include <TR_Coordinates.h>
#include <TR_BLE_To_APP.h>
#include <RocketComputerTypes.h>
#include <TR_INA230.h>
#include <TR_FlightLog.h>
#include <TR_NandBackend_esp.h>
#include <NvsBitmapStore.h>
// FlightSimulator.h removed — sim now runs on FlightComputer via TR_Sensor_Collector_Sim

static TR_I2C_Interface i2c_interface(config::I2C_ADDRESS);
static bool i2c_slave_initialized = false;
static TR_I2S_Stream i2s_stream;
static TR_LogToFlash logger;

// Stage 2b (issue #50): shadow TR_FlightLog instance. Constructed with a nullptr
// backend by default; setup() re-wires the backend to `logger` when
// ENABLE_FLIGHTLOG_SHADOW is on, then calls begin() to load the bitmap + dual-
// copy index. Nothing on the flight hot path touches it yet.
static tr_flightlog::TR_NandBackend_esp flightlog_backend;
static tr_flightlog::TR_FlightLog flightlog;
// Stage 2c-3a: persistent bitmap store wired into begin() so bad-block state
// and (future) allocated-block state survive reboots.
static tr_flightlog::NvsBitmapStore flightlog_bitmap_store;

// --- Stage 2c-3b (issue #50): shadow lifecycle hooks -----------------------
// These fire alongside the legacy LFS prepareLogFile / endLogging so we can
// exercise TR_FlightLog::prepareFlight and finalizeFlight end-to-end on real
// hardware while the hot path is still LFS. The shadow finalize immediately
// deletes the empty index entry so repeated prepare/finalize cycles on the
// bench do not exhaust the flight region (each shadow prepare reserves 32 MB;
// 2c-3c fills those blocks with real data and we stop the delete).

static void shadowPrepareFlight()
{
    if (!config::ENABLE_FLIGHTLOG_SHADOW || !flightlog.isInitialized()) return;
    if (flightlog.isFlightActive())
    {
        ESP_LOGW("FLIGHTLOG", "shadow prepareFlight: already active, skipping");
        return;
    }
    uint32_t id = 0;
    auto st = flightlog.prepareFlight(id);
    if (st == tr_flightlog::Status::Ok)
    {
        ESP_LOGI("FLIGHTLOG",
                 "shadow prepareFlight OK: id=%u, range=[%u..%u), pages=%u",
                 (unsigned)id,
                 (unsigned)flightlog.activeStartBlock(),
                 (unsigned)(flightlog.activeStartBlock() + flightlog.activeBlockCount()),
                 (unsigned)flightlog.activeBlockCount());
    }
    else
    {
        ESP_LOGW("FLIGHTLOG", "shadow prepareFlight: %s",
                 tr_flightlog::to_string(st));
    }
}

static void shadowFinalizeFlight()
{
    if (!config::ENABLE_FLIGHTLOG_SHADOW || !flightlog.isInitialized()) return;
    if (!flightlog.isFlightActive())
    {
        // Can happen if prepareFlight failed, or if this is called twice.
        // Not a warning — just a no-op.
        return;
    }
    char name[24];
    std::snprintf(name, sizeof(name), "shadow_%lu.bin",
                  (unsigned long)flightlog.activeFlightId());
    auto st = flightlog.finalizeFlight(name, 0);  // 2c-3b: 0 bytes (hot path still LFS)
    if (st == tr_flightlog::Status::Ok)
    {
        ESP_LOGI("FLIGHTLOG", "shadow finalizeFlight OK: %s", name);
    }
    else
    {
        ESP_LOGW("FLIGHTLOG", "shadow finalizeFlight: %s",
                 tr_flightlog::to_string(st));
        return;
    }
    // Release the allocated blocks immediately so repeat bench tests don't
    // consume chip capacity. 2c-3c will skip this delete once writeFrame
    // starts producing real data.
    auto dl = flightlog.deleteFlight(name);
    if (dl != tr_flightlog::Status::Ok)
    {
        ESP_LOGW("FLIGHTLOG", "shadow deleteFlight: %s",
                 tr_flightlog::to_string(dl));
    }
}
static TR_BLE_To_APP ble_app("TinkerRocket");
static TR_LoRa_Comms lora_comms;
static SensorConverter sensor_converter;
static TR_Coordinates coord;

// --- INA230 power monitor (always-on I2C bus, not behind PWR_PIN) ---
static i2c_master_bus_handle_t ina230_bus = nullptr;
static TR_INA230 ina230(0x40);
static bool ina230_ok = false;
static bool ina_continuous = false;         // INA230 in continuous-averaging mode (low-power)
// Shunt resistor and current LSB
static constexpr float INA230_R_SHUNT_OHM = 0.002f;     // 2 mOhm
static constexpr float INA230_CURRENT_LSB_A = 0.001f;    // 1 mA/bit
static volatile uint8_t pending_out_command = 0U;
static uint8_t  pending_config_data[sizeof(RollProfileData)] = {};
static volatile size_t   pending_config_data_len = 0;
static volatile uint8_t  pending_config_msg_type = 0;

// Helper: sets pending_out_command with a full memory barrier so that all
// prior writes (config data, len, msg_type) are visible to the other core
// before the command flag.  BLE callbacks run on core 0 but the main loop
// (which reads these in queueOutStatusResponse) runs on core 1.
static inline void setPendingCommand(uint8_t cmd)
{
    __sync_synchronize();  // release barrier: flush all prior writes
    pending_out_command = cmd;
}
// Command retry: the ESP32 I2C slave driver's compound-transaction bug
// can garble the response on the wire even though writeToSlave succeeds
// (it only puts bytes into the TX FIFO, not onto the bus).  Repeat each
// command for CMD_REPEAT_LIMIT polls so the FlightComputer has multiple
// chances to receive it.
static const uint8_t CMD_REPEAT_LIMIT = 5;   // ~1.25s at 250ms poll
static uint8_t cmd_delivery_count = 0;
static uint8_t cmd_delivery_id = 0;           // tracks which command the counter belongs to
static bool camera_recording_requested = false;
static volatile bool flash_op_active = false;   // set during blocking NAND ops (file list/delete/download)
// Set alongside flash_op_active. Gates the I2S DMA recv callback so FC
// sensor data stops being ingested while we're busy serving the phone.
// FC sensor data is uninteresting during a file download (the rocket
// isn't flying during a phone-side fetch), and draining + parsing it
// hogs CPU 1, competing with BLE and flash access. Cleaner than trying
// to make the parse loop yield aggressively: the parse task simply
// doesn't get work queued at all, the ring stays quiescent, and we
// discard any stale bytes before resuming.
static volatile bool i2s_ingest_paused = false;

// Forward declarations — defined after the rx ring buffer block below.
static inline void beginPhoneIO();
static inline void endPhoneIO();

// Phone time sync (BLE Command 9) — used for sim file timestamps
// so each sim run gets a unique filename instead of the hardcoded
// GNSS sentinel date (2025-01-01 12:00).
static uint16_t phone_utc_year   = 0;
static uint8_t  phone_utc_month  = 0;
static uint8_t  phone_utc_day    = 0;
static uint8_t  phone_utc_hour   = 0;
static uint8_t  phone_utc_minute = 0;
static uint8_t  phone_utc_second = 0;
static uint32_t phone_sync_millis = 0;
static bool     phone_time_valid = false;

// NVS persistence for LoRa settings (config.h values are factory defaults)
static Preferences prefs;
static float   lora_freq_mhz  = config::LORA_FREQ_MHZ;
static uint8_t lora_sf         = config::LORA_SF;
static float   lora_bw_khz    = config::LORA_BW_KHZ;
static uint8_t lora_cr         = config::LORA_CR;
static int8_t  lora_tx_power   = config::LORA_TX_POWER_DBM;

static RocketState latest_rocket_state = INITIALIZATION;
static bool pwr_pin_on = false;              // Power rail state — starts OFF
static bool peripherals_initialized = false; // Deferred init for peripherals behind PWR_PIN

// Servo/PID config cache (mirrored from FlightComputer for BLE readback)
static int16_t cfg_servo_bias1 = 0;
static int16_t cfg_servo_hz    = 50;
static int16_t cfg_servo_min   = 1000;
static int16_t cfg_servo_max   = 2000;
static float   cfg_pid_kp  = 0.04f;
static float   cfg_pid_ki  = 0.001f;
static float   cfg_pid_kd  = 0.0003f;
static float   cfg_pid_min = -20.0f;
static float   cfg_pid_max = 20.0f;
static bool    cfg_servo_enabled = true;
static bool    cfg_gain_sched   = true;
static bool    cfg_use_angle_ctrl = false;
static uint16_t cfg_roll_delay_ms  = 0;
static bool    cfg_guidance_en  = false;
static uint8_t cfg_camera_type  = CAM_TYPE_RUNCAM;  // default: RunCam
// Pyro config cache
static bool    cfg_pyro1_enabled = false;
static uint8_t cfg_pyro1_trigger_mode = 0;
static float   cfg_pyro1_trigger_value = 0.0f;
static bool    cfg_pyro2_enabled = false;
static uint8_t cfg_pyro2_trigger_mode = 0;
static float   cfg_pyro2_trigger_value = 0.0f;

// Device identity (loaded from NVS "identity" namespace)
static char    unit_id_hex[9] = {0};           // last 4 bytes of MAC as "a1b2c3d4"
static char    unit_name[24]  = "TinkerRocket"; // default until NVS loads
static uint8_t network_id     = config::DEFAULT_NETWORK_ID;
static uint8_t rocket_id      = config::DEFAULT_ROCKET_ID;

static ISM6HG256Data latest_ism6_raw = {};
static BMP585Data latest_bmp_raw = {};
static GNSSData latest_gnss_raw = {};
static POWERData latest_power_raw = {};
static NonSensorData latest_non_sensor = {};

static MMC5983MAData latest_mmc_raw = {};
static MMC5983MADataSI latest_mmc_si = {};
static bool latest_mmc_valid = false;

static bool latest_ism6_valid = false;
static bool latest_bmp_valid = false;
static GNSSDataSI latest_gnss_si = {};
static bool latest_gnss_valid = false;
static bool latest_power_valid = false;
static bool latest_non_sensor_valid = false;

static float ground_pressure_pa = 101325.0f;
static bool ground_pressure_set = false;
static float pressure_alt_m = 0.0f;
static float pressure_alt_rate_mps = 0.0f;
static float max_alt_m = 0.0f;
static float max_speed_mps = 0.0f;

static uint32_t lora_tx_ok = 0;
static uint32_t lora_tx_fail = 0;
static uint32_t last_lora_tx_ms = 0;
static bool     lora_in_rx_mode = false;
static uint32_t lora_uplink_rx_count = 0;

static OutStatusQueryData last_query_cfg = {};

static inline bool nsFlagSet(uint8_t flags, uint8_t mask)
{
    return (flags & mask) != 0U;
}

// Cached Euler angles derived from NonSensorData quaternion
static float ns_roll_deg  = 0.0f;
static float ns_pitch_deg = 0.0f;
static float ns_yaw_deg   = 0.0f;

// Convert packed quaternion (int16 * 10000) from NonSensorData to display angles.
// Quaternion convention: scalar-first [q0, q1, q2, q3], body-to-NED (FRD body).
//
// Roll uses the gimbal-lock-free body-Z azimuth method: the azimuthal angle
// of the body Z-axis projected into the NED horizontal plane.  This is the
// same formula the angle controller uses and is well-defined at all pitch
// angles, unlike the standard ZYX Euler roll which is degenerate at ±90° pitch.
// Pitch and yaw use standard Euler ZYX (both are well-behaved near vertical).
static void updateEulerFromNonSensor()
{
    if (!latest_non_sensor_valid) return;

    const float qw = (float)latest_non_sensor.q0 / 10000.0f;
    const float qx = (float)latest_non_sensor.q1 / 10000.0f;
    const float qy = (float)latest_non_sensor.q2 / 10000.0f;
    const float qz = (float)latest_non_sensor.q3 / 10000.0f;

    // Roll — gimbal-lock-free: azimuth of body Z-axis in NED horizontal plane
    float z_n = 2.0f * (qx * qz + qw * qy);
    float z_e = 2.0f * (qy * qz - qw * qx);
    ns_roll_deg = -atan2f(z_e, z_n) * (180.0f / (float)M_PI);

    // Pitch (rotation about Y) — standard Euler, well-defined at vertical
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f)
        ns_pitch_deg = copysignf(90.0f, sinp);
    else
        ns_pitch_deg = asinf(sinp) * (180.0f / (float)M_PI);

    // Yaw (rotation about Z) — standard Euler
    ns_yaw_deg = atan2f(2.0f * (qw * qz + qx * qy),
                        1.0f - 2.0f * (qy * qy + qz * qz))
                 * (180.0f / (float)M_PI);
}

// Read INA230 and populate latest_power_raw so that the existing telemetry
// pipeline (BLE, LoRa, web) picks up the values automatically.
// Uses triggered mode: fires one conversion (~0.7ms with 1 avg × 332us × 2ch),
// polls CVRF for completion, reads results, then INA returns to power-down.
// Called inline from the main loop at ~100 Hz.
static void readINA230Power()
{
    if (!ina230_ok) return;

    // Trigger a single shunt+bus conversion (INA auto-powers-down after)
    ina230.setMode(INA230_Mode::SHUNT_BUS_TRIG);

    // Poll CVRF (Conversion Ready Flag, bit 3 of Mask/Enable register)
    // instead of a fixed delay.  Conversion takes ~0.7ms (1 avg × 332µs × 2ch).
    for (int i = 0; i < 20; i++)  // max ~2ms total
    {
        delayMicroseconds(100);
        uint16_t me = 0;
        if (ina230.readMaskEnable(&me) == TR_INA230_OK && (me & (1 << 3)))
            break;  // CVRF set — conversion complete
    }

    float bus_v = 0.0f;
    if (ina230.readBusVoltage_V(&bus_v) != TR_INA230_OK) return;
    float current_a = 0.0f;
    if (ina230.readCurrent_A(&current_a) != TR_INA230_OK) return;

    // 2S LiPo voltage-to-SOC lookup (linear interpolation)
    // Per-cell: 4.20V=100%, 3.90V=75%, 3.80V=50%, 3.70V=25%, 3.50V=10%, 3.30V=0%
    // Doubled for 2S pack: 8.40V=100%, 7.80V=75%, 7.60V=50%, 7.40V=25%, 7.00V=10%, 6.60V=0%
    static constexpr float SOC_V[] = { 6.60f, 7.00f, 7.40f, 7.60f, 7.80f, 8.40f };
    static constexpr float SOC_P[] = { 0.0f,  10.0f, 25.0f, 50.0f, 75.0f, 100.0f };
    static constexpr int SOC_N = 6;
    float soc_pct = 0.0f;
    if (bus_v <= SOC_V[0])
        soc_pct = SOC_P[0];
    else if (bus_v >= SOC_V[SOC_N - 1])
        soc_pct = SOC_P[SOC_N - 1];
    else
    {
        for (int i = 0; i < SOC_N - 1; i++)
        {
            if (bus_v <= SOC_V[i + 1])
            {
                float frac = (bus_v - SOC_V[i]) / (SOC_V[i + 1] - SOC_V[i]);
                soc_pct = SOC_P[i] + frac * (SOC_P[i + 1] - SOC_P[i]);
                break;
            }
        }
    }

    POWERDataSI psi = {};
    // Use FC's timebase (from latest NonSensor frame) so power timestamps
    // align with other sensor data in log files.  Fall back to OC micros()
    // if no NonSensor data has arrived yet.
    psi.time_us = (latest_non_sensor_valid && latest_non_sensor.time_us != 0)
                  ? latest_non_sensor.time_us
                  : (uint32_t)micros();
    psi.voltage = bus_v;
    // Sign convention: negative = discharging (power consumed), positive = charging.
    // The INA230 shunt is wired such that load current reads positive, so invert
    // here to match the base-station battery convention.
    psi.current = -current_a * 1000.0f;
    psi.soc     = soc_pct;

    sensor_converter.packPowerData(psi, latest_power_raw);
    latest_power_valid = true;
}

// ---------------------------------------------------------------------------
//  Low-power mode helpers
//  Called at boot and when power rail is turned OFF.
//  NOTE: Light sleep, BT modem sleep, and reduced TX power disabled —
//  they caused BLE discoverability issues on the Arduino build.
//  These optimisations are active in the ESP-IDF build (tinkerrocket-idf)
//  which has proper sdkconfig support.
// ---------------------------------------------------------------------------
static void enterLowPowerMode()
{
#if defined(CONFIG_PM_ENABLE)
    // Low-power idle: 80 MHz max (BLE needs 80 MHz APB), 40 MHz min via DFS.
    // Light sleep disabled (btLS blocks it; USB-Serial-JTAG incompatible).
    esp_pm_config_t pm_cfg = {};
    pm_cfg.max_freq_mhz = 80;
    pm_cfg.min_freq_mhz = 40;
    pm_cfg.light_sleep_enable = false;
    esp_err_t pm_err = esp_pm_configure(&pm_cfg);
    if (pm_err == ESP_OK)
        ESP_LOGI("PWR", "Low-power mode: 80/40 MHz DFS, light sleep OFF");
    else
        ESP_LOGE("PWR", "esp_pm_configure failed: %s", esp_err_to_name(pm_err));

    // BLE TX power reduction handled by NimBLE config in sdkconfig
#else
    // Arduino IDE build: no light sleep available
    ESP_LOGI("PWR", "Low-power mode (CPU stays at %d MHz)", getCpuFrequencyMhz());
#endif
}

static void exitLowPowerMode()
{
#if defined(CONFIG_PM_ENABLE)
    // Disable light sleep and run at full speed for active mode.
    // This prevents BLE notification drops during NAND logging.
    esp_pm_config_t pm_cfg = {};
    pm_cfg.max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;  // full speed
    pm_cfg.min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;  // no DFS
    pm_cfg.light_sleep_enable = false;
    esp_err_t pm_err = esp_pm_configure(&pm_cfg);
    if (pm_err == ESP_OK)
        ESP_LOGI("PWR", "Light sleep DISABLED, CPU locked at %d MHz",
                 CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    else
        ESP_LOGW("PWR", "esp_pm_configure disable failed: %s",
                 esp_err_to_name(pm_err));
#endif
    ESP_LOGI("PWR", "Full performance mode (%d MHz)", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
}

// Request slow BLE connection parameters for low-power idle.
// Only called when a connection exists and power is OFF.
static void requestSlowBLEParams(uint16_t conn_handle)
{
    struct ble_gap_upd_params params = {};
    params.itvl_min = 0x50;              // 100ms  (units of 1.25ms)
    params.itvl_max = 0xA0;              // 200ms
    params.latency = 4;                  // Skip up to 4 connection events (~800ms effective)
    params.supervision_timeout = 400;    // 4 seconds (units of 10ms)
    params.min_ce_len = 0;
    params.max_ce_len = 0;
    ble_gap_update_params(conn_handle, &params);
}

// Request fast BLE connection parameters for file transfer.
// Called when power is turned ON and a connection already exists.
static void requestFastBLEParams(uint16_t conn_handle)
{
    struct ble_gap_upd_params params = {};
    params.itvl_min = 0x06;              // 7.5ms  (units of 1.25ms)
    params.itvl_max = 0x10;              // 20ms
    params.latency = 0;                  // No slave latency
    params.supervision_timeout = 200;    // 2 seconds (units of 10ms)
    params.min_ce_len = 0;
    params.max_ce_len = 0;
    ble_gap_update_params(conn_handle, &params);
}

static int compareFilesDescending(const void* a, const void* b)
{
    const TR_LogFileInfo* fa = (const TR_LogFileInfo*)a;
    const TR_LogFileInfo* fb = (const TR_LogFileInfo*)b;
    return strcmp(fb->filename, fa->filename);  // Reversed for descending (most recent first)
}

static void updateDerivedAltitudeFromBMP()
{
    if (!latest_bmp_valid)
    {
        return;
    }

    BMP585DataSI bmp_si = {};
    sensor_converter.convertBMP585Data(latest_bmp_raw, bmp_si);
    const float p = bmp_si.pressure;
    if (p <= 0.0f)
    {
        return;
    }

    // Track baseline pressure while not in-flight so pressure altitude is near zero before launch.
    if (latest_rocket_state != INFLIGHT)
    {
        ground_pressure_pa = p;
        ground_pressure_set = true;
    }

    if (ground_pressure_set && ground_pressure_pa > 0.0f)
    {
        pressure_alt_m = 44330.0f * (1.0f - powf(p / ground_pressure_pa, 1.0f / 5.255f));
    }
    else
    {
        pressure_alt_m = 0.0f;
    }

    // Altitude rate now comes from FlightComputer KF via NonSensorData
    // (no local finite-difference needed).
    // Reject physically impossible altitudes (corrupt I2S frames)
    if (pressure_alt_m > -500.0f && pressure_alt_m < 100000.0f)
        max_alt_m = std::max(max_alt_m, pressure_alt_m);
}

static void updateDerivedSpeedFromNonSensor()
{
    if (!latest_non_sensor_valid)
    {
        return;
    }

    // Don't track speed until EKF is running (state > INITIALIZATION)
    // — velocity fields are uninitialized garbage before the EKF starts.
    if (latest_non_sensor.rocket_state <= INITIALIZATION)
    {
        return;
    }

    const float e = (float)latest_non_sensor.e_vel / 100.0f;
    const float n = (float)latest_non_sensor.n_vel / 100.0f;
    const float u = (float)latest_non_sensor.u_vel / 100.0f;
    const float speed = sqrtf(e * e + n * n + u * u);

    // Reject physically impossible speeds (corrupt I2S frames can produce garbage)
    if (speed > 1500.0f)
        return;

    const bool alt_apogee = nsFlagSet(latest_non_sensor.flags, NSF_ALT_APOGEE);
    const bool vel_apogee = nsFlagSet(latest_non_sensor.flags, NSF_VEL_APOGEE);
    if (!alt_apogee && !vel_apogee)
    {
        max_speed_mps = std::max(max_speed_mps, speed);
    }
}

static ISM6LowGFullScale decodeISM6LowGFS(uint8_t fs_g)
{
    switch (fs_g)
    {
        case 2:  return ISM6LowGFullScale::FS_2G;
        case 4:  return ISM6LowGFullScale::FS_4G;
        case 8:  return ISM6LowGFullScale::FS_8G;
        case 16: return ISM6LowGFullScale::FS_16G;
        default: return ISM6LowGFullScale::FS_16G;
    }
}

static ISM6HighGFullScale decodeISM6HighGFS(uint16_t fs_g)
{
    switch (fs_g)
    {
        case 32:  return ISM6HighGFullScale::FS_32G;
        case 64:  return ISM6HighGFullScale::FS_64G;
        case 128: return ISM6HighGFullScale::FS_128G;
        case 256: return ISM6HighGFullScale::FS_256G;
        default:  return ISM6HighGFullScale::FS_256G;
    }
}

static ISM6GyroFullScale decodeISM6GyroFS(uint16_t fs_dps)
{
    switch (fs_dps)
    {
        case 250:  return ISM6GyroFullScale::DPS_250;
        case 500:  return ISM6GyroFullScale::DPS_500;
        case 1000: return ISM6GyroFullScale::DPS_1000;
        case 2000: return ISM6GyroFullScale::DPS_2000;
        case 4000: return ISM6GyroFullScale::DPS_4000;
        default:   return ISM6GyroFullScale::DPS_4000;
    }
}

// I2S telemetry ring (high-volume, parsed by I2S receiver task)
static constexpr size_t RX_STREAM_RING = 65536;
static uint8_t rx_ring[RX_STREAM_RING];

// I2C command ring (low-volume, parsed by loopTask)
static constexpr size_t CMD_RING_SIZE = 1024;
static uint8_t cmd_ring[CMD_RING_SIZE];
static size_t cmd_head = 0;
static size_t cmd_tail = 0;

static inline size_t cmdLen()
{
    if (cmd_head >= cmd_tail) return cmd_head - cmd_tail;
    return CMD_RING_SIZE - (cmd_tail - cmd_head);
}
static inline void cmdPush(uint8_t b)
{
    cmd_ring[cmd_head] = b;
    cmd_head = (cmd_head + 1U) % CMD_RING_SIZE;
    if (cmd_head == cmd_tail)
        cmd_tail = (cmd_tail + 1U) % CMD_RING_SIZE;
}
static inline uint8_t cmdPeek(size_t i)
{
    return cmd_ring[(cmd_tail + i) % CMD_RING_SIZE];
}
static inline void cmdConsume(size_t n)
{
    cmd_tail = (cmd_tail + n) % CMD_RING_SIZE;
}
static size_t rx_head = 0;
static size_t rx_tail = 0;
static volatile uint32_t rx_ring_overflow_drops = 0; // ring full in ISR callback
static uint64_t parser_resync_drops = 0;
static uint64_t parser_len_drops = 0;

static uint32_t frames_bad_crc = 0;
static volatile uint32_t dma_cb_count = 0;      // DMA callback invocations
static uint32_t dedup_drops = 0;                 // timestamp dedup rejects
static uint32_t stale_drops = 0;                 // stale timestamp rejects
static uint32_t raw_i2c_reads = 0;
static uint64_t raw_i2c_bytes = 0;
static uint32_t msg_count_query = 0;
static uint32_t msg_count_ism6 = 0;
static uint32_t msg_count_bmp = 0;
static uint32_t msg_count_mmc = 0;
static uint32_t msg_count_gnss = 0;
static uint32_t msg_count_non_sensor = 0;
static uint32_t msg_count_power = 0;
static uint32_t msg_count_start_logging = 0;
static uint32_t msg_count_end_flight = 0;
static uint32_t msg_count_unknown = 0;

static uint32_t prev_msg_count_query = 0;
static uint32_t prev_msg_count_ism6 = 0;
static uint32_t prev_msg_count_bmp = 0;
static uint32_t prev_msg_count_mmc = 0;
static uint32_t prev_msg_count_gnss = 0;
static uint32_t prev_msg_count_non_sensor = 0;
static uint32_t prev_msg_count_power = 0;
static uint32_t prev_msg_count_start_logging = 0;
static uint32_t prev_msg_count_end_flight = 0;
static uint32_t prev_msg_count_unknown = 0;
static uint32_t last_stats_ms = 0;
static uint64_t prev_bytes_rx = 0;
static uint64_t prev_bytes_nand = 0;
static uint64_t prev_raw_i2c_bytes = 0;
static uint32_t prev_ring_overruns = 0;
static uint32_t prev_ring_drop_oldest_bytes = 0;
static uint32_t interval_ring_fill_peak = 0;

static inline size_t rxLen()
{
    if (rx_head >= rx_tail) return rx_head - rx_tail;
    return RX_STREAM_RING - (rx_tail - rx_head);
}

static inline IRAM_ATTR void rxPush(uint8_t b)
{
    rx_ring[rx_head] = b;
    rx_head = (rx_head + 1U) % RX_STREAM_RING;
    if (rx_head == rx_tail)
    {
        rx_ring_overflow_drops++;
        rx_tail = (rx_tail + 1U) % RX_STREAM_RING; // drop oldest on overflow
    }
}

static inline uint8_t rxPeek(size_t i)
{
    return rx_ring[(rx_tail + i) % RX_STREAM_RING];
}

static inline uint8_t rxPop()
{
    const uint8_t b = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1U) % RX_STREAM_RING;
    return b;
}

// beginPhoneIO / endPhoneIO — bracket a phone-serving operation (file
// list / delete / download). Pauses both I2C servicing (flash_op_active)
// and I2S ingest (i2s_ingest_paused), then drains the rx ring on resume
// so stale sensor bytes from before the pause aren't processed as if
// they were current. Must be called from a single task (oc_loop).
static inline void beginPhoneIO()
{
    i2s_ingest_paused = true;
    flash_op_active   = true;
}
static inline void endPhoneIO()
{
    // Drain while the ISR is still suppressed — race-free.
    rx_tail = rx_head;
    i2s_ingest_paused = false;
    flash_op_active   = false;
}

// Commands that carry a config-data payload to be read via readConfigFrame
static bool isConfigCommand(uint8_t cmd)
{
    return cmd == SIM_CONFIG_PENDING  ||
           cmd == SERVO_CONFIG_PENDING ||
           cmd == PID_CONFIG_PENDING  ||
           cmd == SERVO_TEST_PENDING ||
           cmd == ROLL_PROFILE_PENDING ||
           cmd == SERVO_REPLAY_PENDING ||
           cmd == ROLL_CTRL_CONFIG_PENDING ||
           cmd == PYRO_CONFIG_PENDING ||
           cmd == PYRO_CONT_TEST ||
           cmd == PYRO_FIRE_TEST;
}

// The FC reads exactly FC_COMBINED_READ_SIZE bytes per I2C poll.
// The new i2c_slave driver panics if the master clocks out more bytes
// than are in the TX ringbuffer.  Always write exactly this many bytes
// so the slave hardware never underflows.
static constexpr size_t FC_COMBINED_READ_SIZE = 96;
// ESP-IDF I2C slave hardware eats 1 extra byte from the TX ring buffer
// on each master read (prefetches into shift register, discarded at STOP).
// Transmit 1 extra padding byte so the eaten byte is padding, not the
// start of the next response.  FC still reads FC_COMBINED_READ_SIZE.
static constexpr size_t I2C_TX_PAD = 1;
static constexpr size_t I2C_TX_SIZE = FC_COMBINED_READ_SIZE + I2C_TX_PAD;

static void queueOutStatusResponse(bool ready)
{
    const uint8_t cmd = pending_out_command;  // snapshot (volatile)
    __sync_synchronize();  // acquire: see config data written before the command

    if (cmd != 0U)
    {
        // Detect new command (different from what we were repeating)
        if (cmd != cmd_delivery_id)
        {
            cmd_delivery_count = 0;
            cmd_delivery_id = cmd;
        }
        ESP_LOGI("OC", "I2C TX StatusResponse: ready=%d cmd=0x%02X attempt=%u/%u",
                      ready ? 1 : 0, (unsigned)cmd,
                      (unsigned)(cmd_delivery_count + 1), (unsigned)CMD_REPEAT_LIMIT);
    }

    // Build a single padded buffer: FC_COMBINED_READ_SIZE of real data
    // plus I2C_TX_PAD extra byte(s) to absorb the slave-hardware
    // "eat 1 byte" bug on each master read.
    uint8_t tx_buf[I2C_TX_SIZE] = {};  // zero-padded
    size_t  tx_pos = 0;

    // Pack status response into the buffer
    uint8_t payload[2] = { ready ? 1U : 0U, cmd };
    size_t frame_len = 0;
    if (!TR_I2C_Interface::packMessage(OUT_STATUS_RESPONSE,
                                       payload,
                                       sizeof(payload),
                                       tx_buf,
                                       FC_COMBINED_READ_SIZE,
                                       frame_len))
    {
        return;
    }
    tx_pos = frame_len;

    // Append config data for config commands
    if (cmd != 0U && pending_config_data_len > 0
        && isConfigCommand(cmd))
    {
        uint8_t cfg_frame[MAX_FRAME];
        size_t  cfg_frame_len = 0;
        if (TR_I2C_Interface::packMessage(pending_config_msg_type,
                                           pending_config_data,
                                           pending_config_data_len,
                                           cfg_frame,
                                           sizeof(cfg_frame),
                                           cfg_frame_len))
        {
            if (tx_pos + cfg_frame_len <= FC_COMBINED_READ_SIZE)
            {
                memcpy(tx_buf + tx_pos, cfg_frame, cfg_frame_len);
                tx_pos += cfg_frame_len;
            }
            ESP_LOGI("OC", "I2C TX Config frame type=0x%02X len=%u",
                          (unsigned)pending_config_msg_type,
                          (unsigned)cfg_frame_len);
        }
        else
        {
            ESP_LOGE("OC", "I2C TX Config pack FAILED type=0x%02X data_len=%u",
                          (unsigned)pending_config_msg_type,
                          (unsigned)pending_config_data_len);
        }
    }
    else if (cmd != 0U && isConfigCommand(cmd) && pending_config_data_len == 0)
    {
        ESP_LOGW("OC", "I2C TX config cmd=0x%02X but data_len=0",
                      (unsigned)cmd);
    }

    // Non-blocking (timeout=0): if the TX ringbuffer is full, drop this
    // response — the next query cycle will generate a fresh one.
    i2c_interface.writeToSlave(tx_buf, I2C_TX_SIZE, 0);

    // Repeat each command for CMD_REPEAT_LIMIT polls so the FlightComputer
    // has multiple chances to receive it.
    if (cmd != 0U)
    {
        cmd_delivery_count++;
        if (cmd_delivery_count >= CMD_REPEAT_LIMIT)
        {
            ESP_LOGI("OC", "I2C TX Cmd 0x%02X cleared after %u deliveries",
                          (unsigned)cmd, (unsigned)cmd_delivery_count);
            setPendingCommand(0U);
            pending_config_data_len = 0;
            pending_config_msg_type = 0;
            cmd_delivery_count = 0;
            cmd_delivery_id = 0;
        }
    }
}

static const char* rocketStateToString(RocketState s)
{
    switch (s)
    {
        case INITIALIZATION: return "INITIALIZATION";
        case READY:          return "READY";
        case PRELAUNCH:      return "PRELAUNCH";
        case INFLIGHT:       return "INFLIGHT";
        case LANDED:         return "LANDED";
        default:             return "UNKNOWN";
    }
}

// ---- Shared frame processor: used by both real I2C and sim paths ----
// Logs the frame, then dispatches by type to update latest_*_raw and derived fields.
// Known valid message types — reject anything else as a CRC false positive.
static bool isKnownMessageType(uint8_t type)
{
    switch (type)
    {
        case OUT_STATUS_QUERY:
        case GNSS_MSG:
        case ISM6HG256_MSG:
        case BMP585_MSG:
        case MMC5983MA_MSG:
        case NON_SENSOR_MSG:
        case POWER_MSG:
        case START_LOGGING:
        case END_FLIGHT:
        case OUT_STATUS_RESPONSE:
        case CAMERA_START:
        case CAMERA_STOP:
        case SOUNDS_ENABLE:
        case SOUNDS_DISABLE:
        case SERVO_CONFIG_PENDING:
        case SERVO_CONFIG_MSG:
        case PID_CONFIG_PENDING:
        case PID_CONFIG_MSG:
        case SIM_CONFIG_PENDING:
        case SIM_CONFIG_MSG:
        case SIM_START_CMD:
        case SIM_STOP_CMD:
        case GROUND_TEST_START:
        case GROUND_TEST_STOP:
        case SERVO_TEST_PENDING:
        case SERVO_TEST_MSG:
        case ROLL_PROFILE_PENDING:
        case ROLL_PROFILE_MSG:
        case SERVO_REPLAY_MSG:
        case SERVO_REPLAY_STOP:
        case ROLL_CTRL_CONFIG_PENDING:
        case ROLL_CTRL_CONFIG_MSG:
        case GUIDANCE_ENABLE:
        case GUIDANCE_DISABLE:
        case GUIDANCE_TELEM_MSG:
        case CAMERA_CONFIG_PENDING:
        case CAMERA_CONFIG_MSG:
        case LORA_MSG:
            return true;
        default:
            return false;
    }
}

static void processFrame(const uint8_t* frame, size_t frame_len,
                         uint8_t type, const uint8_t* payload, size_t payload_len)
{
    // Reject unknown message types (CRC false positives from I2S noise)
    if (!isKnownMessageType(type))
        return;

    // ── Duplicate & stale frame detection for I2S DMA ──
    // 1. Skip exact-duplicate frames (same type, same time_us) caused by
    //    DMA buffer replay.
    // 2. Skip stale frames whose timestamp is more than 5 seconds behind
    //    the latest seen timestamp.  These can appear when MRAM ring buffer
    //    data from a previous logging session leaks into the current one
    //    (clearRing resets pointers but doesn't zero MRAM contents).
    if (payload_len >= 4)
    {
        static uint32_t prev_time_ism6 = 0, prev_time_bmp = 0,
                         prev_time_mmc = 0, prev_time_gnss = 0,
                         prev_time_ns = 0, prev_time_pwr = 0,
                         prev_time_guid = 0;
        static uint32_t max_time_us = 0;  // Monotonic high-water mark across all types
        uint32_t time_us;
        memcpy(&time_us, payload, sizeof(time_us));
        uint32_t* prev = nullptr;
        switch (type) {
            case ISM6HG256_MSG:       prev = &prev_time_ism6; break;
            case BMP585_MSG:          prev = &prev_time_bmp;  break;
            case MMC5983MA_MSG:       prev = &prev_time_mmc;  break;
            case GNSS_MSG:            prev = &prev_time_gnss; break;
            case NON_SENSOR_MSG:      prev = &prev_time_ns;   break;
            case POWER_MSG:           prev = &prev_time_pwr;  break;
            case GUIDANCE_TELEM_MSG:  prev = &prev_time_guid; break;
            default: break;
        }
        if (prev != nullptr)
        {
            if (time_us == *prev && time_us != 0)
            {
                dedup_drops++;
                return;  // duplicate — skip
            }

            // Reject stale frames: if we've seen timestamps > 5s ahead of this
            // frame, it's from a previous session.  The 5s threshold avoids
            // false positives from minor jitter or sensor startup delays.
            static constexpr uint32_t STALE_THRESHOLD_US = 5'000'000;
            if (time_us != 0 && max_time_us > STALE_THRESHOLD_US &&
                time_us < (max_time_us - STALE_THRESHOLD_US))
            {
                stale_drops++;
                return;  // stale — skip
            }

            *prev = time_us;
            if (time_us > max_time_us)
                max_time_us = time_us;
        }
    }

    (void)logger.enqueueFrame(frame, frame_len); // exact bytes as received

    if (type == OUT_STATUS_QUERY)
    {
        msg_count_query++;
        if (payload_len >= sizeof(OutStatusQueryData))
        {
            memcpy(&last_query_cfg, payload, sizeof(OutStatusQueryData));
            sensor_converter.configureISM6HG256FullScale(
                decodeISM6LowGFS(last_query_cfg.ism6_low_g_fs_g),
                decodeISM6HighGFS(last_query_cfg.ism6_high_g_fs_g),
                decodeISM6GyroFS(last_query_cfg.ism6_gyro_fs_dps));
            sensor_converter.configureISM6HG256RotationZ(
                (float)last_query_cfg.ism6_rot_z_cdeg / 100.0f);
            sensor_converter.configureMMC5983MARotationZ(
                (float)last_query_cfg.mmc_rot_z_cdeg / 100.0f);
            // Apply high-g bias from FlightComputer calibration (format v2+)
            if (last_query_cfg.format_version >= 2)
            {
                sensor_converter.setHighGBias(
                    (float)last_query_cfg.hg_bias_x_cmss / 100.0f,
                    (float)last_query_cfg.hg_bias_y_cmss / 100.0f,
                    (float)last_query_cfg.hg_bias_z_cmss / 100.0f);
            }
        }
        queueOutStatusResponse(true);
    }
    else if (type == ISM6HG256_MSG)
    {
        msg_count_ism6++;
        if (payload_len >= sizeof(ISM6HG256Data))
        {
            memcpy(&latest_ism6_raw, payload, sizeof(ISM6HG256Data));
            latest_ism6_valid = true;
        }
    }
    else if (type == BMP585_MSG)
    {
        msg_count_bmp++;
        if (payload_len >= sizeof(BMP585Data))
        {
            memcpy(&latest_bmp_raw, payload, sizeof(BMP585Data));
            latest_bmp_valid = true;
            updateDerivedAltitudeFromBMP();
        }
    }
    else if (type == MMC5983MA_MSG)
    {
        msg_count_mmc++;
        if (payload_len >= sizeof(MMC5983MAData))
        {
            memcpy(&latest_mmc_raw, payload, sizeof(MMC5983MAData));
            sensor_converter.convertMMC5983MAData(latest_mmc_raw, latest_mmc_si);
            latest_mmc_valid = true;
        }
    }
    else if (type == GNSS_MSG)
    {
        msg_count_gnss++;
        if (payload_len >= sizeof(GNSSData))
        {
            memcpy(&latest_gnss_raw, payload, sizeof(GNSSData));
            sensor_converter.convertGNSSData(latest_gnss_raw, latest_gnss_si);
            latest_gnss_valid = true;
        }
    }
    else if (type == NON_SENSOR_MSG)
    {
        msg_count_non_sensor++;
        if (payload_len >= sizeof(NonSensorData))
        {
            const RocketState prev_state = latest_rocket_state;
            memcpy(&latest_non_sensor, payload, sizeof(NonSensorData));
            latest_non_sensor_valid = true;
            latest_rocket_state = (RocketState)latest_non_sensor.rocket_state;
            if (latest_rocket_state != prev_state && latest_rocket_state == PRELAUNCH)
            {
                max_alt_m = 0.0f;
                max_speed_mps = 0.0f;
                pressure_alt_rate_mps = 0.0f;
                ground_pressure_set = false;  // Re-acquire ground pressure for new flight

                // Pre-create log file now so there's no NAND stall at launch
                logger.prepareLogFile();
                shadowPrepareFlight();
                ESP_LOGI("OC", "PRELAUNCH - pre-creating log file");
            }
            // KF-filtered altitude rate from FlightComputer (or sim equivalent)
            pressure_alt_rate_mps = (float)latest_non_sensor.baro_alt_rate_dmps * 0.1f;
            updateEulerFromNonSensor();
            updateDerivedSpeedFromNonSensor();

            // Adopt the FlightComputer's live guidance_enabled state (carried
            // in pyro_status bit 5) as the source of truth. FC broadcasts this
            // every non-sensor frame, so OUT just tracks it in RAM — no NVS
            // persistence needed (the next boot's first FC frame repopulates
            // it). This used to write NVS inline here, which blocked flash
            // access during BLE file downloads and tripped the task watchdog
            // on CPU 1 when the I2S Parse backlog got large.
            cfg_guidance_en =
                (latest_non_sensor.pyro_status & PSF_GUIDANCE_ENABLED) != 0;
        }
    }
    else if (type == POWER_MSG)
    {
        msg_count_power++;
        if (payload_len >= sizeof(POWERData))
        {
            memcpy(&latest_power_raw, payload, sizeof(POWERData));
            latest_power_valid = true;
        }
    }
    else if (type == START_LOGGING)
    {
        msg_count_start_logging++;
        logger.startLogging();
    }
    else if (type == END_FLIGHT)
    {
        msg_count_end_flight++;
        logger.endLogging();
        shadowFinalizeFlight();
    }
    else
    {
        msg_count_unknown++;
    }
}

// ---- I2C entry point ----
static void handleReceivedFrame(const uint8_t* frame, size_t frame_len,
                                uint8_t type, const uint8_t* payload, size_t payload_len)
{
    // Sim now runs on FlightComputer — all frames (real or simulated) flow
    // through the same pipeline.  No filtering needed.
    processFrame(frame, frame_len, type, payload, payload_len);
}

static void parseRxStream()
{
    uint8_t payload[MAX_PAYLOAD];
    while (rxLen() >= (4 + 1 + 1 + 2))
    {
        if (!(rxPeek(0) == 0xAA &&
              rxPeek(1) == 0x55 &&
              rxPeek(2) == 0xAA &&
              rxPeek(3) == 0x55))
        {
            parser_resync_drops++;
            (void)rxPop();
            continue;
        }

        const size_t payload_len = rxPeek(5);
        const size_t frame_len = 4 + 1 + 1 + payload_len + 2;
        if (payload_len > MAX_PAYLOAD)
        {
            parser_len_drops++;
            (void)rxPop();
            continue;
        }
        if (rxLen() < frame_len)
        {
            return;
        }

        uint8_t frame[MAX_FRAME];
        for (size_t i = 0; i < frame_len; ++i)
        {
            frame[i] = rxPeek(i);
        }

        uint8_t type = 0;
        size_t out_payload_len = 0;
        const bool ok = TR_I2C_Interface::unpackMessage(frame,
                                                        frame_len,
                                                        type,
                                                        payload,
                                                        sizeof(payload),
                                                        out_payload_len,
                                                        true);
        if (!ok)
        {
            frames_bad_crc++;
            (void)rxPop();
            continue;
        }

        for (size_t i = 0; i < frame_len; ++i)
        {
            (void)rxPop();
        }
        handleReceivedFrame(frame, frame_len, type, payload, out_payload_len);
    }
}

// I2S receiver task handle (for cleanup on power-down)
static TaskHandle_t i2s_rx_task_handle = nullptr;

// Zero-copy I2S DMA receive callback.
// Fires from ISR context each time a DMA buffer completes — no stale replays.
// Pushes raw bytes directly into rx_ring and notifies the parser task.
// Diagnostic: count non-zero bytes across all DMA callbacks
static volatile uint32_t dma_nonzero_bytes = 0;
static volatile uint32_t dma_total_bytes = 0;
// One-time hex dump of first DMA buffer with non-zero content after logging starts
static volatile bool dma_dump_done = false;
static volatile bool dma_dump_requested = false;
static uint8_t dma_dump_buf[64];
static volatile size_t dma_dump_len = 0;

static IRAM_ATTR bool i2sRecvCallback(const uint8_t* buf, size_t len, void* user_ctx)
{
    // Early-exit while phone transfers are in flight: we deliberately
    // drop FC sensor data so the I2S parse task doesn't compete with
    // BLE + flash on CPU 1. DMA keeps writing its own internal buffers;
    // we just skip the ring-push + task-notify work. See i2s_ingest_paused
    // declaration for rationale.
    if (i2s_ingest_paused) return false;

    dma_cb_count++;
    dma_total_bytes += len;

    // Count non-zero bytes for diagnostics
    uint32_t nz = 0;
    for (size_t i = 0; i < len; i++)
        if (buf[i] != 0) nz++;
    dma_nonzero_bytes += nz;

    // Capture first non-trivial DMA buffer for hex dump
    if (dma_dump_requested && !dma_dump_done && nz > 0)
    {
        size_t cp = (len < 64) ? len : 64;
        memcpy(dma_dump_buf, buf, cp);
        dma_dump_len = cp;
        dma_dump_done = true;
    }

    // Push DMA bytes into ring buffer using rxPush() which drops oldest
    // byte on overflow (instead of discarding the rest of the DMA buffer).
    for (size_t i = 0; i < len; i++)
    {
        rxPush(buf[i]);
    }

    raw_i2c_reads++;
    raw_i2c_bytes += (uint64_t)len;

    // Notify parser task that data is available
    BaseType_t wake = pdFALSE;
    if (i2s_rx_task_handle)
        vTaskNotifyGiveFromISR(i2s_rx_task_handle, &wake);
    return wake == pdTRUE;
}

// I2S parser task — woken by DMA callback, parses frames from rx_ring.
// Unlike the old polling approach, this never reads stale DMA data because
// the callback only fires when a fresh DMA buffer completes.
static void i2sParserTask(void *)
{
    for (;;)
    {
        // Wait for notification from DMA callback (blocks, no CPU waste)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Parse all complete frames in the ring
        parseRxStream();
    }
}

// parseCmdRing — parses the I2C command ring buffer (same SOF framing).
static void parseCmdRing()
{
    while (cmdLen() >= 8)
    {
        if (cmdPeek(0) != 0xAA || cmdPeek(1) != 0x55 ||
            cmdPeek(2) != 0xAA || cmdPeek(3) != 0x55)
        {
            cmdConsume(1);
            continue;
        }
        const uint8_t payload_len = cmdPeek(5);
        if (payload_len > MAX_PAYLOAD) { cmdConsume(1); continue; }
        const size_t frame_len = 4 + 1 + 1 + payload_len + 2;
        if (cmdLen() < frame_len) break;

        uint8_t frame[MAX_FRAME];
        for (size_t i = 0; i < frame_len; i++) frame[i] = cmdPeek(i);

        uint8_t type = 0;
        uint8_t payload[MAX_PAYLOAD];
        size_t payload_out_len = 0;
        if (TR_I2C_Interface::unpackMessage(frame, frame_len,
                                             type, payload, sizeof(payload),
                                             payload_out_len, true))
        {
            processFrame(frame, frame_len, type, payload, payload_out_len);
            cmdConsume(frame_len);
        }
        else
        {
            cmdConsume(1);
        }
    }
}

// serviceI2CIngress — reads I2C command data from the FlightComputer
// into a separate command ring buffer (not the I2S telemetry ring).
static void serviceI2CIngress()
{
    uint8_t inbuf[256];
    const int n = i2c_interface.readFromSlave(inbuf, sizeof(inbuf), 0);
    if (n > 0)
    {
        ESP_LOGI("I2C_RX", "Got %d bytes from FC", n);
        for (int i = 0; i < n; ++i)
            cmdPush(inbuf[i]);
        parseCmdRing();
    }
}

// ============================================================================
static bool buildLoRaPayload(uint8_t out_payload[SIZE_OF_LORA_DATA])
{
    if (out_payload == nullptr)
    {
        return false;
    }

    LoRaDataSI lora = {};
    // Routing header
    lora.network_id = network_id;
    lora.rocket_id  = rocket_id;

    if (latest_gnss_valid)
    {
        lora.num_sats = latest_gnss_si.num_sats;
        lora.pdop = latest_gnss_si.pdop;
        lora.horizontal_accuracy = latest_gnss_si.horizontal_accuracy;

        coord.geodeticToECEF(latest_gnss_si.lat * TR_Coordinates::DEG2RAD,
                             latest_gnss_si.lon * TR_Coordinates::DEG2RAD,
                             latest_gnss_si.alt,
                             lora.ecef_x,
                             lora.ecef_y,
                             lora.ecef_z);
    }

    lora.rocket_state = (uint8_t)latest_rocket_state;
    lora.camera_recording = camera_recording_requested;
    lora.logging_active = logger.isLoggingActive();

    if (latest_non_sensor_valid)
    {
        lora.roll = ns_roll_deg;
        lora.pitch = ns_pitch_deg;
        lora.yaw = ns_yaw_deg;
        lora.q0 = (float)latest_non_sensor.q0 / 10000.0f;
        lora.q1 = (float)latest_non_sensor.q1 / 10000.0f;
        lora.q2 = (float)latest_non_sensor.q2 / 10000.0f;
        lora.q3 = (float)latest_non_sensor.q3 / 10000.0f;
        lora.launch_flag = nsFlagSet(latest_non_sensor.flags, NSF_LAUNCH);
        lora.vel_u_apogee_flag = nsFlagSet(latest_non_sensor.flags, NSF_VEL_APOGEE);
        lora.alt_apogee_flag = nsFlagSet(latest_non_sensor.flags, NSF_ALT_APOGEE);
        lora.alt_landed_flag = nsFlagSet(latest_non_sensor.flags, NSF_ALT_LANDED);

        const float e = (float)latest_non_sensor.e_vel / 100.0f;
        const float n = (float)latest_non_sensor.n_vel / 100.0f;
        const float u = (float)latest_non_sensor.u_vel / 100.0f;
        lora.speed = sqrtf(e * e + n * n + u * u);
    }

    if (latest_ism6_valid)
    {
        ISM6HG256DataSI ism_si = {};
        sensor_converter.convertISM6HG256Data(latest_ism6_raw, ism_si);
        lora.acc_x = (float)ism_si.low_g_acc_x;
        lora.acc_y = (float)ism_si.low_g_acc_y;
        lora.acc_z = (float)ism_si.low_g_acc_z;
        lora.gyro_x = (float)ism_si.gyro_x;
        lora.gyro_y = (float)ism_si.gyro_y;
        lora.gyro_z = (float)ism_si.gyro_z;
        lora.temp = 0.0f;
    }

    if (latest_power_valid)
    {
        POWERDataSI power_si = {};
        sensor_converter.convertPowerData(latest_power_raw, power_si);
        lora.voltage = power_si.voltage;
        lora.current = power_si.current;
        lora.soc = power_si.soc;
    }

    lora.pressure_alt = pressure_alt_m;
    lora.altitude_rate = pressure_alt_rate_mps;
    lora.max_alt = max_alt_m;
    lora.max_speed = max_speed_mps;

    lora.base_station_voltage = 0.0f;
    lora.base_station_current = 0.0f;
    lora.base_station_soc = 0.0f;
    lora.rssi = 0.0f;
    lora.snr = 0.0f;

    sensor_converter.packLoRaData(lora, out_payload);
    return true;
}

static void serviceLoRa()
{
    if (!config::USE_LORA_RADIO)
    {
        return;
    }

    lora_comms.service();

    const uint32_t now_ms = millis();
    const uint32_t period_ms = (config::LORA_TX_RATE_HZ > 0)
        ? (1000U / config::LORA_TX_RATE_HZ)
        : 40U;
    if ((now_ms - last_lora_tx_ms) < period_ms)
    {
        return;
    }
    if (!lora_comms.canSend())
    {
        return;
    }

    uint8_t payload[SIZE_OF_LORA_DATA] = {0};
    if (!buildLoRaPayload(payload))
    {
        return;
    }
    last_lora_tx_ms = now_ms;
    lora_in_rx_mode = false;  // Exiting RX for TX
    if (lora_comms.send(payload, sizeof(payload)))
    {
        lora_tx_ok++;
    }
    else
    {
        lora_tx_fail++;
    }
}

// Send a LoRa name beacon so the base station can learn this rocket's name.
// Only sent during READY/PRELAUNCH (suppressed in flight to save airtime).
static uint32_t last_beacon_ms = 0;

static void sendLoRaBeacon()
{
    if (!config::USE_LORA_RADIO) return;

    // Only beacon in idle states
    if (latest_rocket_state != READY && latest_rocket_state != PRELAUNCH) return;

    const uint32_t now_ms = millis();
    if ((now_ms - last_beacon_ms) < 2000) return;
    if (!lora_comms.canSend()) return;

    // Beacon format: [0xBE][network_id][rocket_id][unit_name...]
    uint8_t beacon[32];
    beacon[0] = LORA_BEACON_SYNC;
    beacon[1] = network_id;
    beacon[2] = rocket_id;
    size_t name_len = strlen(unit_name);
    if (name_len > sizeof(beacon) - 3) name_len = sizeof(beacon) - 3;
    memcpy(&beacon[3], unit_name, name_len);

    last_beacon_ms = now_ms;
    lora_in_rx_mode = false;
    lora_comms.send(beacon, 3 + name_len);
}

// ============================================================================
// Config readback: send current config to app over BLE
// ============================================================================

static void sendCurrentConfig()
{
    // Split config into two smaller JSON messages to stay within MTU limits.
    // Message 1: servo/PID/LoRa config ("config" type)
    String j = "{\"type\":\"config\"";
    j += ",\"sb1\":"; j += itos(cfg_servo_bias1);
    j += ",\"shz\":"; j += itos(cfg_servo_hz);
    j += ",\"smn\":"; j += itos(cfg_servo_min);
    j += ",\"smx\":"; j += itos(cfg_servo_max);
    j += ",\"kp\":";  j += fmtf(cfg_pid_kp, 4);
    j += ",\"ki\":";  j += fmtf(cfg_pid_ki, 4);
    j += ",\"kd\":";  j += fmtf(cfg_pid_kd, 4);
    j += ",\"pmn\":"; j += fmtf(cfg_pid_min, 1);
    j += ",\"pmx\":"; j += fmtf(cfg_pid_max, 1);
    j += ",\"sen\":"; j += cfg_servo_enabled ? "true" : "false";
    j += ",\"gs\":";  j += cfg_gain_sched ? "true" : "false";
    j += ",\"ac\":";  j += cfg_use_angle_ctrl ? "true" : "false";
    j += ",\"rdly\":"; j += itos(cfg_roll_delay_ms);
    j += ",\"ge\":";  j += cfg_guidance_en ? "true" : "false";
    j += ",\"camt\":"; j += itos(cfg_camera_type);
    // LoRa settings
    j += ",\"lf\":";  j += fmtf(lora_freq_mhz, 1);
    j += ",\"lsf\":"; j += itos(lora_sf);
    j += ",\"lbw\":"; j += fmtf(lora_bw_khz, 0);
    j += ",\"lcr\":"; j += itos(lora_cr);
    j += ",\"lpw\":"; j += itos(lora_tx_power);
    j += "}";
    ble_app.sendConfigJSON(j);
    ESP_LOGI("CFG", "Sent config readback (%u bytes)", (unsigned)j.length());

    delay(50);  // let BLE stack drain before next notification

    // Message 2: pyro config ("config_pyro" type)
    String p = "{\"type\":\"config_pyro\"";
    p += ",\"p1e\":"; p += cfg_pyro1_enabled ? "true" : "false";
    p += ",\"p1m\":"; p += itos(cfg_pyro1_trigger_mode);
    p += ",\"p1v\":"; p += fmtf(cfg_pyro1_trigger_value, 1);
    p += ",\"p2e\":"; p += cfg_pyro2_enabled ? "true" : "false";
    p += ",\"p2m\":"; p += itos(cfg_pyro2_trigger_mode);
    p += ",\"p2v\":"; p += fmtf(cfg_pyro2_trigger_value, 1);
    p += "}";
    ble_app.sendConfigJSON(p);
    ESP_LOGI("CFG", "Sent pyro config readback (%u bytes)", (unsigned)p.length());

    delay(50);

    // Message 3: device identity ("config_identity" type)
    char id_buf[128];
    snprintf(id_buf, sizeof(id_buf),
             "{\"type\":\"config_identity\""
             ",\"uid\":\"%s\""
             ",\"un\":\"%s\""
             ",\"nid\":%u"
             ",\"rid\":%u"
             ",\"dt\":\"%s\"}",
             unit_id_hex, unit_name,
             (unsigned)network_id, (unsigned)rocket_id,
             config::DEVICE_TYPE);
    String id_json(id_buf);
    ble_app.sendConfigJSON(id_json);
    ESP_LOGI("CFG", "Sent identity readback (%u bytes)", (unsigned)id_json.length());
}

// Cache servo config to NVS (mirrors what FlightComputer stores)
static void cacheServoConfig(const uint8_t* payload, size_t len)
{
    if (len < 14) return;
    ServoConfigData sc;
    memcpy(&sc, payload, sizeof(sc));
    cfg_servo_bias1 = sc.bias_us[0];
    cfg_servo_hz    = sc.hz;
    cfg_servo_min   = sc.min_us;
    cfg_servo_max   = sc.max_us;
    prefs.begin("servo", false);
    prefs.putShort("b1",  sc.bias_us[0]);
    prefs.putShort("hz",  sc.hz);
    prefs.putShort("min", sc.min_us);
    prefs.putShort("max", sc.max_us);
    prefs.end();
    ESP_LOGI("CFG", "Servo config cached: bias=%d hz=%d min=%d max=%d",
        sc.bias_us[0], sc.hz, sc.min_us, sc.max_us);
}

// Cache PID config to NVS
static void cachePIDConfig(const uint8_t* payload, size_t len)
{
    if (len < 20) return;
    PIDConfigData pc;
    memcpy(&pc, payload, sizeof(pc));
    cfg_pid_kp = pc.kp; cfg_pid_ki = pc.ki; cfg_pid_kd = pc.kd;
    cfg_pid_min = pc.min_cmd; cfg_pid_max = pc.max_cmd;
    prefs.begin("pid", false);
    prefs.putFloat("kp", pc.kp);
    prefs.putFloat("ki", pc.ki);
    prefs.putFloat("kd", pc.kd);
    prefs.putFloat("mn", pc.min_cmd);
    prefs.putFloat("mx", pc.max_cmd);
    prefs.end();
    ESP_LOGI("CFG", "PID config cached: Kp=%.4f Ki=%.4f Kd=%.4f [%.1f,%.1f]",
        pc.kp, pc.ki, pc.kd, pc.min_cmd, pc.max_cmd);
}

// Cache roll control config to NVS
static void cacheRollControlConfig(const uint8_t* payload, size_t len)
{
    if (len < 4) return;
    RollControlConfigData rc;
    memcpy(&rc, payload, sizeof(rc));
    cfg_use_angle_ctrl = (rc.use_angle_control != 0);
    cfg_roll_delay_ms  = rc.roll_delay_ms;
    prefs.begin("roll", false);
    prefs.putBool("ac", cfg_use_angle_ctrl);
    prefs.putUShort("rdly", cfg_roll_delay_ms);
    prefs.end();
    ESP_LOGI("CFG", "Roll control cached: angle_ctrl=%s delay=%u ms",
        cfg_use_angle_ctrl ? "ON" : "OFF", (unsigned)cfg_roll_delay_ms);
}

// ============================================================================
// LoRa Uplink RX (receive sim commands from BaseStation)
// ============================================================================

static void processUplinkCommand(uint8_t cmd, const uint8_t* payload, size_t payload_len)
{
    ESP_LOGI("LORA", "UPLINK RX cmd=%u payload_len=%u", cmd, (unsigned)payload_len);

    if (cmd == 1)
    {
        // Camera: payload[0] = desired state (1 = on, 0 = off).
        // Payload makes retries idempotent (won't toggle back and forth).
        // Falls back to toggle if no payload (legacy compat).
        bool want_on = (payload_len >= 1) ? (payload[0] != 0)
                                          : !camera_recording_requested;
        if (want_on != camera_recording_requested)
        {
            camera_recording_requested = want_on;
            setPendingCommand(want_on ? CAMERA_START : CAMERA_STOP);
            ESP_LOGI("LORA", "UPLINK Camera %s", want_on ? "START" : "STOP");
        }
        else
        {
            ESP_LOGI("LORA", "UPLINK Camera already %s, ignoring",
                          want_on ? "ON" : "OFF");
        }
    }
    else if (cmd == 23)
    {
        // Logging: payload[0] = desired state (1 = start, 0 = stop).
        // Payload makes retries idempotent (won't toggle back and forth).
        // Falls back to toggle if no payload (legacy compat).
        bool want_on = (payload_len >= 1) ? (payload[0] != 0)
                                          : !logger.isLoggingActive();
        if (want_on && !logger.isLoggingActive())
        {
            logger.startLogging();
            ESP_LOGI("LORA", "UPLINK Logging started");
        }
        else if (!want_on && logger.isLoggingActive())
        {
            logger.endLogging();
            shadowFinalizeFlight();
            ESP_LOGI("LORA", "UPLINK Logging stopped");
        }
        else
        {
            ESP_LOGI("LORA", "UPLINK Logging already %s, ignoring",
                          want_on ? "ON" : "OFF");
        }
    }
    else if (cmd == 24 && payload_len >= sizeof(ServoTestAnglesData))
    {
        // Servo test angles: relay to FlightComputer via I2C
        memcpy(pending_config_data, payload, sizeof(ServoTestAnglesData));
        pending_config_data_len = sizeof(ServoTestAnglesData);
        pending_config_msg_type = SERVO_TEST_MSG;
        setPendingCommand(SERVO_TEST_PENDING);
        ESP_LOGI("LORA", "UPLINK Servo test angles queued");
    }
    else if (cmd == 25)
    {
        // Servo test stop
        setPendingCommand(SERVO_TEST_STOP);
        ESP_LOGI("LORA", "UPLINK Servo test stop");
    }
    else if (cmd == 5 && payload_len >= 12)
    {
        // Sim config: relay to FlightComputer via I2C
        SimConfigData sim_cfg;
        float mass_g;
        memcpy(&mass_g,              payload + 0, 4);
        memcpy(&sim_cfg.thrust_n,    payload + 4, 4);
        memcpy(&sim_cfg.burn_time_s, payload + 8, 4);
        sim_cfg.mass_kg = mass_g / 1000.0f;
        sim_cfg.descent_rate_mps = 0.0f;
        if (payload_len >= 16) {
            memcpy(&sim_cfg.descent_rate_mps, payload + 12, 4);
        }
        memcpy(pending_config_data, &sim_cfg, sizeof(sim_cfg));
        pending_config_data_len = sizeof(sim_cfg);
        pending_config_msg_type = SIM_CONFIG_MSG;
        setPendingCommand(SIM_CONFIG_PENDING);
        ESP_LOGI("LORA", "UPLINK Sim config queued: mass=%.0fg thrust=%.1fN burn=%.1fs descent=%.1fm/s",
                      (double)mass_g, (double)sim_cfg.thrust_n,
                      (double)sim_cfg.burn_time_s, (double)sim_cfg.descent_rate_mps);
    }
    else if (cmd == 6)
    {
        setPendingCommand(SIM_START_CMD);
        ESP_LOGI("LORA", "UPLINK Sim start queued for FlightComputer");
    }
    else if (cmd == 7)
    {
        logger.endLogging();
        shadowFinalizeFlight();
        setPendingCommand(SIM_STOP_CMD);
        ESP_LOGI("LORA", "UPLINK Sim stop queued for FlightComputer (logging ended)");
    }
    else if (cmd == 10 && payload_len >= 11)
    {
        // LoRa reconfiguration via uplink: [freq:4f][bw:4f][sf:1][cr:1][txpwr:1]
        float new_freq, new_bw;
        memcpy(&new_freq, payload + 0, 4);
        memcpy(&new_bw,   payload + 4, 4);
        uint8_t new_sf   = payload[8];
        uint8_t new_cr   = payload[9];
        int8_t  new_pwr  = (int8_t)payload[10];

        if (lora_comms.reconfigure(new_freq, new_sf, new_bw, new_cr, new_pwr))
        {
            lora_freq_mhz = new_freq;
            lora_bw_khz   = new_bw;
            lora_sf        = new_sf;
            lora_cr        = new_cr;
            lora_tx_power  = new_pwr;

            prefs.begin("lora", false);
            prefs.putFloat("freq",  lora_freq_mhz);
            prefs.putFloat("bw",    lora_bw_khz);
            prefs.putUChar("sf",    lora_sf);
            prefs.putUChar("cr",    lora_cr);
            prefs.putChar("txpwr",  lora_tx_power);
            prefs.end();

            ESP_LOGI("LORA", "UPLINK LoRa reconfigured + saved: %.1f MHz SF%u BW%.0f CR%u %d dBm",
                          (double)lora_freq_mhz, (unsigned)lora_sf,
                          (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);
        }
        else
        {
            ESP_LOGE("LORA", "UPLINK LoRa reconfigure FAILED");
        }
    }
    else if (cmd == 12 && payload_len >= 14)
    {
        // Servo config from BaseStation: relay to FlightComputer + cache
        memcpy(pending_config_data, payload, 14);
        pending_config_data_len = 14;
        pending_config_msg_type = SERVO_CONFIG_MSG;
        setPendingCommand(SERVO_CONFIG_PENDING);
        cacheServoConfig(payload, payload_len);
        ESP_LOGI("LORA", "UPLINK Servo config queued for RocketComputer");
    }
    else if (cmd == 13 && payload_len >= 20)
    {
        // PID config from BaseStation: relay to FlightComputer + cache
        memcpy(pending_config_data, payload, 20);
        pending_config_data_len = 20;
        pending_config_msg_type = PID_CONFIG_MSG;
        setPendingCommand(PID_CONFIG_PENDING);
        cachePIDConfig(payload, payload_len);
        ESP_LOGI("LORA", "UPLINK PID config queued for RocketComputer");
    }
    else if (cmd == 14 && payload_len >= 1)
    {
        // Servo control enable/disable from BaseStation
        bool enabled = (payload[0] != 0);
        cfg_servo_enabled = enabled;
        setPendingCommand(enabled ? SERVO_CTRL_ENABLE : SERVO_CTRL_DISABLE);
        ESP_LOGI("LORA", "UPLINK Servo control: %s", enabled ? "ENABLE" : "DISABLE");
    }
    else if (cmd == 22 && payload_len >= 1)
    {
        // Gain schedule enable/disable from BaseStation
        bool enabled = (payload[0] != 0);
        cfg_gain_sched = enabled;
        setPendingCommand(enabled ? GAIN_SCHED_ENABLE : GAIN_SCHED_DISABLE);
        ESP_LOGI("LORA", "UPLINK Gain schedule: %s", enabled ? "ENABLE" : "DISABLE");
    }
    else if (cmd == 31 && payload_len >= 4)
    {
        // Roll control config from BaseStation
        memcpy(pending_config_data, payload, 4);
        pending_config_data_len = 4;
        pending_config_msg_type = ROLL_CTRL_CONFIG_MSG;
        setPendingCommand(ROLL_CTRL_CONFIG_PENDING);
        cacheRollControlConfig(payload, payload_len);
        ESP_LOGI("LORA", "UPLINK Roll control config queued for RocketComputer");
    }
    else if (cmd == 32 && payload_len >= 1)
    {
        // Guidance enable/disable from BaseStation
        bool enabled = (payload[0] != 0);
        cfg_guidance_en = enabled;
        setPendingCommand(enabled ? GUIDANCE_ENABLE : GUIDANCE_DISABLE);
        ESP_LOGI("LORA", "UPLINK Guidance: %s", enabled ? "ENABLE" : "DISABLE");
    }
    else
    {
        ESP_LOGW("LORA", "UPLINK Unknown cmd %u", cmd);
    }
}

/// Enter RX mode between TX cycles and check for uplink commands
static void serviceLoRaUplink()
{
    if (!config::USE_LORA_RADIO) return;

    lora_comms.service();  // Complete any pending TX (auto-enters RX after TX)

    // Only enter RX when radio is idle (not transmitting)
    if (!lora_comms.canSend()) return;

    // service() auto-calls startReceive() after TX completion.
    // Sync our tracking flag to avoid a redundant startReceive() that
    // would reset rx_done_ and potentially drop a received packet.
    if (lora_comms.isInRxMode())
    {
        lora_in_rx_mode = true;
    }

    // Enter RX mode if not already (first call before any TX has occurred)
    if (!lora_in_rx_mode)
    {
        if (!lora_comms.startReceive()) return;
        lora_in_rx_mode = true;
    }

    // Poll DIO1 pin directly as fallback for hardware interrupt.
    // The ISR fires reliably for TX-done events, but on some ESP32-S3
    // boards the interrupt may not trigger for RX-done.  pollDio1()
    // checks the pin level and sets rx_done_ if DIO1 is asserted.
    lora_comms.pollDio1();

    // Non-blocking poll for uplink packet
    uint8_t rx_buf[32];
    size_t rx_len = 0;

    if (lora_comms.readPacket(rx_buf, sizeof(rx_buf), rx_len))
    {
        // Uplink format v1: [0xCA][network_id][target_rocket_id][cmd][len][payload...]
        if (rx_len >= 5 && rx_buf[0] == config::UPLINK_SYNC_BYTE)
        {
            uint8_t pkt_nid = rx_buf[1];
            uint8_t pkt_rid = rx_buf[2];
            // Filter: must match our network, and target us or broadcast (0xFF)
            if (pkt_nid == network_id &&
                (pkt_rid == rocket_id || pkt_rid == 0xFF))
            {
                uint8_t cmd = rx_buf[3];
                uint8_t payload_len = rx_buf[4];
                if (rx_len >= (size_t)(5 + payload_len))
                {
                    processUplinkCommand(cmd, &rx_buf[5], payload_len);
                    lora_uplink_rx_count++;
                }
            }
        }
        // readPacket() internally re-enters RX mode after reading
    }
}

static void printLoRaPayloadDebug()
{
    if (!config::USE_LORA_RADIO)
    {
        return;
    }

    uint8_t payload[SIZE_OF_LORA_DATA] = {0};
    if (!buildLoRaPayload(payload))
    {
        return;
    }

    LoRaDataSI decoded = {};
    sensor_converter.unpackLoRa(payload, decoded);
    ESP_LOGI("LORA", "LoRa tx sats/pdop=%u/%.1f | ecef(m)=%.0f,%.0f,%.0f | alt/rate/max/mspd=%.1f/%.1f/%.1f/%.1f",
                  (unsigned)decoded.num_sats,
                  (double)decoded.pdop,
                  (double)decoded.ecef_x,
                  (double)decoded.ecef_y,
                  (double)decoded.ecef_z,
                  (double)decoded.pressure_alt,
                  (double)decoded.altitude_rate,
                  (double)decoded.max_alt,
                  (double)decoded.max_speed);
    ESP_LOGI("LORA", "LoRa tx state/flags=%u/%u%u%u%u | acc=%.1f,%.1f,%.1f | gyro=%.1f,%.1f,%.1f | v/i/soc=%.2f/%.0f/%.0f",
                  (unsigned)decoded.rocket_state,
                  decoded.launch_flag ? 1U : 0U,
                  decoded.vel_u_apogee_flag ? 1U : 0U,
                  decoded.alt_apogee_flag ? 1U : 0U,
                  decoded.alt_landed_flag ? 1U : 0U,
                  (double)decoded.acc_x,
                  (double)decoded.acc_y,
                  (double)decoded.acc_z,
                  (double)decoded.gyro_x,
                  (double)decoded.gyro_y,
                  (double)decoded.gyro_z,
                  (double)decoded.voltage,
                  (double)decoded.current,
                  (double)decoded.soc);
}

static void printStats()
{
    const uint32_t now = millis();
    if ((now - last_stats_ms) < config::STATS_PERIOD_MS)
    {
        return;
    }
    const uint32_t dt = now - last_stats_ms;
    last_stats_ms = now;

    // --- Low-power mode: send minimal BLE telemetry only ---
    if (!pwr_pin_on)
    {
        readINA230Power();

        TR_BLE_To_APP::TelemetryData ble_telem = {};
        if (latest_power_valid)
        {
            POWERDataSI p = {};
            sensor_converter.convertPowerData(latest_power_raw, p);
            ble_telem.soc = p.soc;
            ble_telem.current = p.current;
            ble_telem.voltage = p.voltage;
        }
        else
        {
            ble_telem.soc = NAN;
            ble_telem.current = NAN;
            ble_telem.voltage = NAN;
        }
        ble_telem.latitude = NAN;
        ble_telem.longitude = NAN;
        ble_telem.gdop = NAN;
        ble_telem.num_sats = 0;
        ble_telem.state = "OFF";
        ble_telem.camera_recording = false;
        ble_telem.logging_active = false;
        ble_telem.active_file = "";
        ble_telem.rx_kbs = NAN;
        ble_telem.wr_kbs = NAN;
        ble_telem.frames_rx = 0;
        ble_telem.frames_drop = 0;
        ble_telem.max_alt_m = NAN;
        ble_telem.max_speed_mps = NAN;
        ble_telem.pressure_alt = NAN;
        ble_telem.altitude_rate = NAN;
        ble_telem.rssi = NAN;
        ble_telem.snr = NAN;
        ble_telem.roll = NAN;
        ble_telem.pitch = NAN;
        ble_telem.yaw = NAN;
        ble_telem.roll_cmd = NAN;
        ble_telem.bs_soc = NAN;
        ble_telem.bs_voltage = NAN;
        ble_telem.bs_current = NAN;
        ble_telem.pwr_pin_on = false;
        ble_app.sendTelemetry(ble_telem);
        return;
    }

    // --- Active mode: full stats and telemetry ---
    TR_LogToFlashStats s = {};
    logger.getStats(s);

    // Capture GNSS timestamp for the active log file (when available)
    if (s.logging_active && latest_gnss_valid && latest_gnss_si.year > 2000)
    {
        uint16_t ts_year   = latest_gnss_si.year;
        uint8_t  ts_month  = latest_gnss_si.month;
        uint8_t  ts_day    = latest_gnss_si.day;
        uint8_t  ts_hour   = latest_gnss_si.hour;
        uint8_t  ts_minute = latest_gnss_si.minute;
        uint8_t  ts_second = latest_gnss_si.second;

        // Sim mode uses hardcoded GNSS date (2025-01-01 12:00:xx).
        // Substitute phone-synced time for unique filenames.
        if (phone_time_valid &&
            ts_year == 2025 && ts_month == 1 && ts_day == 1 &&
            ts_hour == 12 && ts_minute == 0)
        {
            uint32_t elapsed_s = (millis() - phone_sync_millis) / 1000;
            uint32_t total_s = (uint32_t)phone_utc_hour * 3600U +
                               (uint32_t)phone_utc_minute * 60U +
                               (uint32_t)phone_utc_second + elapsed_s;
            ts_year   = phone_utc_year;
            ts_month  = phone_utc_month;
            ts_day    = phone_utc_day;
            if (total_s >= 86400U)
            {
                ts_day += (uint8_t)(total_s / 86400U);
                total_s %= 86400U;
            }
            ts_hour   = (uint8_t)(total_s / 3600U);
            ts_minute = (uint8_t)((total_s % 3600U) / 60U);
            ts_second = (uint8_t)(total_s % 60U);
        }

        logger.setFileTimestamp(logger.currentFilename(),
                               ts_year, ts_month, ts_day,
                               ts_hour, ts_minute, ts_second);
    }

    if (s.ring_fill > interval_ring_fill_peak)
    {
        interval_ring_fill_peak = s.ring_fill;
    }
    const uint64_t rx_delta = s.bytes_received - prev_bytes_rx;
    const uint64_t nand_delta = s.bytes_written_nand - prev_bytes_nand;
    const uint64_t raw_i2c_delta = raw_i2c_bytes - prev_raw_i2c_bytes;
    const uint32_t ring_overrun_delta = s.ring_overruns - prev_ring_overruns;
    const uint32_t ring_drop_oldest_delta = s.ring_drop_oldest_bytes - prev_ring_drop_oldest_bytes;
    prev_bytes_rx = s.bytes_received;
    prev_bytes_nand = s.bytes_written_nand;
    prev_raw_i2c_bytes = raw_i2c_bytes;
    prev_ring_overruns = s.ring_overruns;
    prev_ring_drop_oldest_bytes = s.ring_drop_oldest_bytes;

    const float rx_kbs = (dt > 0) ? ((float)rx_delta / (float)dt) : 0.0f;
    const float wr_kbs = (dt > 0) ? ((float)nand_delta / (float)dt) : 0.0f;
    const float raw_rx_kbs = (dt > 0) ? ((float)raw_i2c_delta / (float)dt) : 0.0f;
    const uint32_t d_query = msg_count_query - prev_msg_count_query;
    const uint32_t d_ism6 = msg_count_ism6 - prev_msg_count_ism6;
    const uint32_t d_bmp = msg_count_bmp - prev_msg_count_bmp;
    const uint32_t d_mmc = msg_count_mmc - prev_msg_count_mmc;
    const uint32_t d_gnss = msg_count_gnss - prev_msg_count_gnss;
    const uint32_t d_non_sensor = msg_count_non_sensor - prev_msg_count_non_sensor;
    const uint32_t d_power = msg_count_power - prev_msg_count_power;
    const uint32_t d_start_logging = msg_count_start_logging - prev_msg_count_start_logging;
    const uint32_t d_end_flight = msg_count_end_flight - prev_msg_count_end_flight;
    const uint32_t d_unknown = msg_count_unknown - prev_msg_count_unknown;
    const float hz_scale = (dt > 0U) ? (1000.0f / (float)dt) : 0.0f;
    const float hz_query = (float)d_query * hz_scale;
    const float hz_ism6 = (float)d_ism6 * hz_scale;
    const float hz_bmp = (float)d_bmp * hz_scale;
    const float hz_mmc = (float)d_mmc * hz_scale;
    const float hz_gnss = (float)d_gnss * hz_scale;
    const float hz_non_sensor = (float)d_non_sensor * hz_scale;
    const float hz_power = (float)d_power * hz_scale;
    const float hz_start_logging = (float)d_start_logging * hz_scale;
    const float hz_end_flight = (float)d_end_flight * hz_scale;
    const float hz_unknown = (float)d_unknown * hz_scale;
    prev_msg_count_query = msg_count_query;
    prev_msg_count_ism6 = msg_count_ism6;
    prev_msg_count_bmp = msg_count_bmp;
    prev_msg_count_mmc = msg_count_mmc;
    prev_msg_count_gnss = msg_count_gnss;
    prev_msg_count_non_sensor = msg_count_non_sensor;
    prev_msg_count_power = msg_count_power;
    prev_msg_count_start_logging = msg_count_start_logging;
    prev_msg_count_end_flight = msg_count_end_flight;
    prev_msg_count_unknown = msg_count_unknown;

    if (config::VERBOSE_DEBUG)
    {
    ESP_LOGI("OC", "RX %.1f KB/s | WR %.1f KB/s | frames rx/drop/bad=%lu/%lu/%lu | RING=%lu/%lu (hi=%lu)",
                  (double)rx_kbs,
                  (double)wr_kbs,
                  (unsigned long)s.frames_received,
                  (unsigned long)s.frames_dropped,
                  (unsigned long)frames_bad_crc,
                  (unsigned long)s.ring_fill,
                  (unsigned long)config::RAM_RING_SIZE,
                  (unsigned long)s.ring_highwater);
    ESP_LOGI("OC", "RING interval peak/overrun/drop_oldest_bytes=%lu/%lu/%lu",
                  (unsigned long)interval_ring_fill_peak,
                  (unsigned long)ring_overrun_delta,
                  (unsigned long)ring_drop_oldest_delta);
    ESP_LOGI("OC", "i2c raw reads/bytes=%lu/%llu",
                  (unsigned long)raw_i2c_reads,
                  (unsigned long long)raw_i2c_bytes);
    ESP_LOGI("OC", "i2c raw_rx=%.1f KB/s | ring_drops=%lu | parser_drops resync/len/crc=%llu/%llu/%lu",
                  (double)raw_rx_kbs,
                  (unsigned long)rx_ring_overflow_drops,
                  (unsigned long long)parser_resync_drops,
                  (unsigned long long)parser_len_drops,
                  (unsigned long)frames_bad_crc);
    ESP_LOGI("LOG", "logging=%c file=%s page=%lu block=%lu prog_fail=%lu erase_fail=%lu",
                  s.logging_active ? 'Y' : 'N',
                  logger.currentFilename(),
                  (unsigned long)s.nand_page,
                  (unsigned long)s.nand_block,
                  (unsigned long)s.nand_prog_fail,
                  (unsigned long)s.nand_erase_fail);
    ESP_LOGI("CFG", "cfg fs=%u/%u/%u rot_z(ism/mmc)=%.2f/%.2f deg v%u",
                  (unsigned)last_query_cfg.ism6_low_g_fs_g,
                  (unsigned)last_query_cfg.ism6_high_g_fs_g,
                  (unsigned)last_query_cfg.ism6_gyro_fs_dps,
                  (double)((float)last_query_cfg.ism6_rot_z_cdeg / 100.0f),
                  (double)((float)last_query_cfg.mmc_rot_z_cdeg / 100.0f),
                  (unsigned)last_query_cfg.format_version);
    ESP_LOGI("OC", "msg Hz q/ism6/bmp/mmc/gnss/ns/pwr/st/en/unk=%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f",
                  (double)hz_query,
                  (double)hz_ism6,
                  (double)hz_bmp,
                  (double)hz_mmc,
                  (double)hz_gnss,
                  (double)hz_non_sensor,
                  (double)hz_power,
                  (double)hz_start_logging,
                  (double)hz_end_flight,
                  (double)hz_unknown);
    if (config::USE_LORA_RADIO)
    {
        TR_LoRa_Comms::Stats ls = {};
        lora_comms.getStats(ls);
        ESP_LOGI("LORA", "LoRa en=%c tx_start/ok/fail=%lu/%lu/%lu local_ok/fail=%lu/%lu last_err=%d tx=%c",
                      ls.enabled ? 'Y' : 'N',
                      (unsigned long)ls.tx_started,
                      (unsigned long)ls.tx_ok,
                      (unsigned long)ls.tx_fail,
                      (unsigned long)lora_tx_ok,
                      (unsigned long)lora_tx_fail,
                      (int)ls.last_error,
                      ls.transmitting ? 'Y' : 'N');
        printLoRaPayloadDebug();
    }
    // I2S pipeline stats
    {
        static uint32_t prev_dma_cb = 0, prev_ring_ovf = 0, prev_dedup = 0, prev_stale = 0, prev_parsed = 0;
        static uint64_t prev_dma_bytes = 0;
        uint32_t d_cb = dma_cb_count - prev_dma_cb;
        uint64_t d_bytes = raw_i2c_bytes - prev_dma_bytes;
        uint32_t d_ovf = rx_ring_overflow_drops - prev_ring_ovf;
        uint32_t d_stale = stale_drops - prev_stale;
        uint32_t d_dedup = dedup_drops - prev_dedup;
        uint32_t d_parsed = msg_count_ism6 + msg_count_bmp + msg_count_mmc + msg_count_non_sensor + msg_count_gnss;
        static uint32_t prev_total_parsed = 0;
        uint32_t d_p = d_parsed - prev_total_parsed;
        // Calculate non-zero byte percentage
        static uint32_t prev_nz = 0;
        static uint32_t prev_tot = 0;
        uint32_t d_nz = dma_nonzero_bytes - prev_nz;
        uint32_t d_tot = dma_total_bytes - prev_tot;
        float nz_pct = (d_tot > 0) ? (d_nz * 100.0f / d_tot) : 0;

        ESP_LOGI("I2S", "dma_cb=%lu KB=%.1f nz=%.1f%% ovf=%lu dedup=%lu stale=%lu parsed=%lu frx=%lu fdr=%lu",
                 (unsigned long)d_cb,
                 (double)(d_bytes / 1024.0),
                 (double)nz_pct,
                 (unsigned long)d_ovf,
                 (unsigned long)d_dedup,
                 (unsigned long)d_stale,
                 (unsigned long)d_p,
                 (unsigned long)s.frames_received,
                 (unsigned long)s.frames_dropped);

        // Print hex dump of first DMA buffer (once)
        if (dma_dump_done && dma_dump_len > 0)
        {
            char hex[64 * 3 + 1];
            size_t n = (dma_dump_len < 48) ? dma_dump_len : 48;
            for (size_t i = 0; i < n; i++)
                sprintf(hex + i * 3, "%02X ", dma_dump_buf[i]);
            hex[n * 3] = '\0';
            ESP_LOGI("I2S", "DMA sample (%u bytes): %s", (unsigned)dma_dump_len, hex);
            dma_dump_done = false;
            dma_dump_len = 0;
        }

        // Compact LoRa uplink stats
        if (config::USE_LORA_RADIO)
        {
            TR_LoRa_Comms::Stats ls = {};
            lora_comms.getStats(ls);
            ESP_LOGI("LORA", "LoRa tx=%lu/%lu rx=%lu crc_fail=%lu isr=%lu uplink_rx=%lu rxmode=%c",
                          (unsigned long)ls.tx_ok,
                          (unsigned long)ls.tx_fail,
                          (unsigned long)ls.rx_count,
                          (unsigned long)ls.rx_crc_fail,
                          (unsigned long)ls.isr_count,
                          (unsigned long)lora_uplink_rx_count,
                          ls.rx_mode ? 'Y' : 'N');
        }

        prev_dma_cb = dma_cb_count;
        prev_dma_bytes = raw_i2c_bytes;
        prev_ring_ovf = rx_ring_overflow_drops;
        prev_dedup = dedup_drops;
        prev_stale = stale_drops;
        prev_total_parsed = d_parsed;
        prev_nz = dma_nonzero_bytes;
        prev_tot = dma_total_bytes;
    }
    } // End VERBOSE_DEBUG

    // Always-on LFS/NAND stall instrumentation — prints the peak duration of
    // each potentially-slow LittleFS/NAND op observed since the last stats
    // window.  Complements the per-op ESP_LOGW("STALL: …") that fires live
    // whenever any single op exceeds 100 ms.
    ESP_LOGI("LOG TIMING",
             "write=%lu sync=%lu erase=%lu open=%lu close=%lu "
             "activate=%lu clr_ring=%lu iter=%lu us  syncs=%lu erases=%lu ring_peak=%lu "
             "bad_blocks=%lu skips=%lu",
             (unsigned long)s.write_max_us,
             (unsigned long)s.sync_max_us,
             (unsigned long)s.erase_max_us,
             (unsigned long)s.open_max_us,
             (unsigned long)s.close_max_us,
             (unsigned long)s.activate_max_us,
             (unsigned long)s.clear_ring_max_us,
             (unsigned long)s.flush_iter_max_us,
             (unsigned long)s.syncs_performed,
             (unsigned long)s.nand_erase_ops,
             (unsigned long)interval_ring_fill_peak,
             (unsigned long)s.known_bad_blocks,
             (unsigned long)s.bad_block_skips);
    logger.resetIntervalTimings();

    // Send telemetry to BLE app
    TR_BLE_To_APP::TelemetryData ble_telem = {};
    ble_telem.soc = NAN;
    ble_telem.current = NAN;
    ble_telem.voltage = NAN;
    ble_telem.latitude = NAN;
    ble_telem.longitude = NAN;
    ble_telem.gdop = NAN;
    ble_telem.num_sats = 0;
    if (latest_power_valid)
    {
        POWERDataSI p = {};
        sensor_converter.convertPowerData(latest_power_raw, p);
        ble_telem.soc = p.soc;
        ble_telem.current = p.current;
        ble_telem.voltage = p.voltage;
    }
    if (latest_gnss_valid)
    {
        ble_telem.latitude = latest_gnss_si.lat;
        ble_telem.longitude = latest_gnss_si.lon;
        ble_telem.gdop = latest_gnss_si.pdop;
        ble_telem.num_sats = (int)latest_gnss_si.num_sats;
    }
    ble_telem.state = rocketStateToString(latest_rocket_state);
    ble_telem.camera_recording = camera_recording_requested;
    ble_telem.logging_active = s.logging_active;
    ble_telem.active_file = logger.currentFilename();
    ble_telem.rx_kbs = rx_kbs;
    ble_telem.wr_kbs = wr_kbs;
    ble_telem.frames_rx = s.frames_received;
    ble_telem.frames_drop = s.frames_dropped;
    ble_telem.max_alt_m = max_alt_m;
    ble_telem.max_speed_mps = max_speed_mps;
    ble_telem.pressure_alt = pressure_alt_m;
    ble_telem.altitude_rate = pressure_alt_rate_mps;
    if (latest_ism6_valid)
    {
        ISM6HG256DataSI ism_si = {};
        sensor_converter.convertISM6HG256Data(latest_ism6_raw, ism_si);
        ble_telem.low_g_x = ism_si.low_g_acc_x;
        ble_telem.low_g_y = ism_si.low_g_acc_y;
        ble_telem.low_g_z = ism_si.low_g_acc_z;
        ble_telem.high_g_x = ism_si.high_g_acc_x;
        ble_telem.high_g_y = ism_si.high_g_acc_y;
        ble_telem.high_g_z = ism_si.high_g_acc_z;
        ble_telem.gyro_x = ism_si.gyro_x;
        ble_telem.gyro_y = ism_si.gyro_y;
        ble_telem.gyro_z = ism_si.gyro_z;
    }
    // Attitude quaternion from FlightComputer
    ble_telem.q0 = (float)latest_non_sensor.q0 / 10000.0f;
    ble_telem.q1 = (float)latest_non_sensor.q1 / 10000.0f;
    ble_telem.q2 = (float)latest_non_sensor.q2 / 10000.0f;
    ble_telem.q3 = (float)latest_non_sensor.q3 / 10000.0f;
    ble_telem.roll_cmd = (float)latest_non_sensor.roll_cmd / 100.0f;
    ble_telem.rssi = NAN;  // LoRa RSSI only meaningful on base station (continuous RX)
    ble_telem.snr = NAN;
    ble_telem.bs_soc = NAN;      // No base station battery
    ble_telem.bs_voltage = NAN;
    ble_telem.bs_current = NAN;
    // Flight event flags
    ble_telem.launch_flag       = nsFlagSet(latest_non_sensor.flags, NSF_LAUNCH);
    ble_telem.vel_u_apogee_flag = nsFlagSet(latest_non_sensor.flags, NSF_VEL_APOGEE);
    ble_telem.alt_apogee_flag   = nsFlagSet(latest_non_sensor.flags, NSF_ALT_APOGEE);
    ble_telem.alt_landed_flag   = nsFlagSet(latest_non_sensor.flags, NSF_ALT_LANDED);
    ble_telem.pwr_pin_on        = pwr_pin_on;
    // Pyro channel status from NonSensorData
    ble_telem.pyro1_armed = nsFlagSet(latest_non_sensor.flags, NSF_PYRO1_ARMED);
    ble_telem.pyro2_armed = nsFlagSet(latest_non_sensor.flags, NSF_PYRO2_ARMED);
    uint8_t ps = latest_non_sensor.pyro_status;
    ble_telem.pyro1_cont  = (ps & PSF_CH1_CONT) != 0;
    ble_telem.pyro2_cont  = (ps & PSF_CH2_CONT) != 0;
    ble_telem.pyro1_fired = (ps & PSF_CH1_FIRED) != 0;
    ble_telem.pyro2_fired = (ps & PSF_CH2_FIRED) != 0;

    static uint32_t telem_send_count = 0;
    static uint32_t telem_skip_count = 0;
    bool connected = ble_app.isConnected();
    if (connected)
    {
        ble_app.sendTelemetry(ble_telem);
        telem_send_count++;
    }
    else
    {
        telem_skip_count++;
    }
    if (config::VERBOSE_DEBUG)
    {
        // Log every 10 seconds
        if ((telem_send_count + telem_skip_count) % 10 == 0)
        {
            ESP_LOGI("BLE", "Telem: sent=%lu skip=%lu connected=%d",
                     (unsigned long)telem_send_count,
                     (unsigned long)telem_skip_count,
                     connected ? 1 : 0);
        }

        // Stack high-water mark (minimum free stack ever seen, in bytes)
        UBaseType_t hwm = uxTaskGetStackHighWaterMark(nullptr);
        ESP_LOGI("OC", "Stack HWM: %u bytes free",
                 (unsigned)(hwm * sizeof(StackType_t)));
    }

    interval_ring_fill_peak = s.ring_fill;
}

// Initialize peripherals that are behind the PWR_PIN power rail.
// Called once when power is first turned on (deferred from setup).
void initPeripherals()
{
    if (peripherals_initialized) return;

    ESP_LOGI("PWR", "Initializing peripherals...");

    SPI.begin(config::SPI_SCK, config::SPI_MISO, config::SPI_MOSI);
    delay(20);

    TR_LogToFlashConfig log_cfg = {};
    log_cfg.nand_cs = config::NAND_CS;
    log_cfg.spi_hz_nand = config::SPI_HZ_NAND;
    log_cfg.spi_mode_nand = config::SPI_MODE_NAND;
    log_cfg.ring_buffer_size = config::RAM_RING_SIZE;
    log_cfg.debug = config::DEBUG;
    // MRAM ring buffer (128 KB on shared SPI bus — replaces 64 KB RAM ring)
    log_cfg.mram_cs = config::MRAM_CS;
    log_cfg.spi_hz_mram = config::SPI_HZ_MRAM;
    log_cfg.spi_mode_mram = config::SPI_MODE_MRAM;
    log_cfg.mram_size = config::MRAM_SIZE;

    // --- LFS partition shrink (issue #50 Stage 2c-2) ------------------------
    // When TR_FlightLog is active we restrict LFS to the first 32 blocks
    // (4 MB) so the flight-log allocator can own the remaining 988 blocks
    // plus the metadata blocks. First boot under the shrunk layout force-
    // formats (wipes any legacy flight_*.bin files) — that's the accepted
    // upgrade cost. Subsequent boots read the NVS marker and skip the wipe.
    bool lfs_wipe_pending = false;
    if (config::ENABLE_FLIGHTLOG_SHADOW)
    {
        log_cfg.lfs_block_count = 32;

        Preferences fl_prefs;
        bool already_shrunk = false;
        if (fl_prefs.begin("flightlog", /*readOnly=*/true))
        {
            already_shrunk = fl_prefs.getBool("lfs_shrunk", false);
            fl_prefs.end();
        }
        if (!already_shrunk)
        {
            log_cfg.force_format = true;
            lfs_wipe_pending = true;
            ESP_LOGW("FLIGHTLOG", "First boot of shrunk-LFS firmware — will wipe legacy flight files");
        }
    }

    if (!logger.begin(SPI, log_cfg))
    {
        ESP_LOGE("PWR", "TR_LogToFlash begin failed");
        return;
    }

    // Record the shrunk-layout marker so subsequent boots skip the force_format.
    if (lfs_wipe_pending)
    {
        Preferences fl_prefs;
        if (fl_prefs.begin("flightlog", /*readOnly=*/false))
        {
            fl_prefs.putBool("lfs_shrunk", true);
            fl_prefs.end();
            ESP_LOGI("FLIGHTLOG", "Recorded shrunk-LFS marker in NVS");
        }
    }

    // --- TR_FlightLog shadow (issue #50 Stage 2b) ---------------------------
    // Bring up the new append-only NAND layer alongside the legacy LFS path.
    // At this point the SPI bus + bad-block bitmap are initialized by
    // logger.begin(); the shadow just reads state (no NAND writes) and logs
    // its view so bench tests can verify it loads cleanly. Hot-path writes
    // still go through LFS until Stage 2c-3.
    if (config::ENABLE_FLIGHTLOG_SHADOW)
    {
        flightlog_backend = tr_flightlog::TR_NandBackend_esp(&logger);
        auto st = flightlog.begin(flightlog_backend,
                                  tr_flightlog::TR_FlightLog::Config{},
                                  &flightlog_bitmap_store);
        if (st == tr_flightlog::Status::Ok)
        {
            ESP_LOGI("FLIGHTLOG", "shadow up: %zu flight(s) in index, %zu bad blocks",
                     flightlog.index().size(),
                     flightlog.bitmap().countInState(tr_flightlog::BLOCK_BAD));
        }
        else
        {
            ESP_LOGE("FLIGHTLOG", "shadow begin failed: %s",
                     tr_flightlog::to_string(st));
        }
    }

    // -------------------------------------------------------------------
    // DIAGNOSTIC ONE-SHOT — wipes the entire filesystem at boot.
    // Flip FORMAT_FS_ON_BOOT to 1, rebuild, flash, boot once, set
    // back to 0, rebuild, flash.  Or leave it on if you want every
    // boot to start with a clean FS (for stress testing).
    // LEAVES ALL EXISTING FLIGHT LOGS PERMANENTLY DELETED.
    //
    // Rationale: confirm whether the multi-hundred-ms LFS stalls we've
    // been chasing (#47) come from accumulated file-metadata bloat on
    // the chip vs from inherent LFS-on-NAND behaviour.  If stalls
    // vanish on a fresh FS, we know fill-up is the cause and can plan
    // the custom log-layer rewrite.  Turn back off once confirmed.
    #define FORMAT_FS_ON_BOOT 0   // ← flip to 1 to wipe the chip at boot
    #if FORMAT_FS_ON_BOOT
    ESP_LOGW("LOG", "============================================");
    ESP_LOGW("LOG", "   FORMAT_FS_ON_BOOT IS SET!");
    ESP_LOGW("LOG", "   WIPING ALL FLIGHT LOGS.");
    ESP_LOGW("LOG", "============================================");
    delay(500);  // make sure the warning hits the serial log before we wipe
    if (!logger.formatFilesystem())
    {
        ESP_LOGE("LOG", "Format failed — continuing with whatever state exists");
    }
    else
    {
        ESP_LOGW("LOG", "Format complete — filesystem is now empty");
    }
    #endif
    // -------------------------------------------------------------------

    // Start the NAND flush task on Core 0 — decouples LittleFS writes from
    // the main loop so the RAM ring can buffer during NAND stalls.
    logger.startFlushTask(/* core */ 0, /* stackSize */ 8192, /* priority */ 1);

    TR_LogToFlashRecoveryInfo recovery = {};
    logger.getRecoveryInfo(recovery);
    if (recovery.recovered)
    {
        ESP_LOGI("LOG", "Startup recovery wrote %lu bytes to %s",
                      (unsigned long)recovery.recovered_bytes,
                      recovery.filename);
    }

    vTaskDelay(1);  // feed watchdog after NAND init

    if (config::USE_LORA_RADIO)
    {
        // Load saved LoRa config from NVS (write config.h defaults if empty)
        prefs.begin("lora", false);  // read-write
        if (!prefs.isKey("freq"))
        {
            // First boot or NVS erased — seed with config.h factory defaults
            prefs.putFloat("freq",  config::LORA_FREQ_MHZ);
            prefs.putUChar("sf",    config::LORA_SF);
            prefs.putFloat("bw",    config::LORA_BW_KHZ);
            prefs.putUChar("cr",    config::LORA_CR);
            prefs.putChar("txpwr",  config::LORA_TX_POWER_DBM);
            ESP_LOGI("CFG", "LoRa NVS empty -- wrote config.h defaults");
        }
        lora_freq_mhz = prefs.getFloat("freq", config::LORA_FREQ_MHZ);
        lora_sf        = prefs.getUChar("sf",   config::LORA_SF);
        lora_bw_khz    = prefs.getFloat("bw",   config::LORA_BW_KHZ);
        lora_cr        = prefs.getUChar("cr",   config::LORA_CR);
        lora_tx_power  = (int8_t)prefs.getChar("txpwr", config::LORA_TX_POWER_DBM);
        prefs.end();
        ESP_LOGI("CFG", "LoRa NVS: %.1f MHz SF%u BW%.0f CR%u %d dBm",
                      (double)lora_freq_mhz, (unsigned)lora_sf,
                      (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);

        // Load cached servo config from NVS
        prefs.begin("servo", false);  // read-write (creates namespace on first boot)
        cfg_servo_bias1 = prefs.getShort("b1",  cfg_servo_bias1);
        cfg_servo_hz    = prefs.getShort("hz",  cfg_servo_hz);
        cfg_servo_min   = prefs.getShort("min", cfg_servo_min);
        cfg_servo_max   = prefs.getShort("max", cfg_servo_max);
        prefs.end();
        ESP_LOGI("CFG", "NVS Servo cache: bias=%d hz=%d min=%d max=%d",
            cfg_servo_bias1, cfg_servo_hz, cfg_servo_min, cfg_servo_max);

        // Load cached PID config from NVS
        prefs.begin("pid", false);
        cfg_pid_kp  = prefs.getFloat("kp",  cfg_pid_kp);
        cfg_pid_ki  = prefs.getFloat("ki",  cfg_pid_ki);
        cfg_pid_kd  = prefs.getFloat("kd",  cfg_pid_kd);
        cfg_pid_min = prefs.getFloat("mn",  cfg_pid_min);
        cfg_pid_max = prefs.getFloat("mx",  cfg_pid_max);
        cfg_gain_sched = prefs.getBool("gs", cfg_gain_sched);
        prefs.end();
        ESP_LOGI("CFG", "NVS PID cache: Kp=%.4f Ki=%.4f Kd=%.4f [%.1f,%.1f] GS=%s",
            cfg_pid_kp, cfg_pid_ki, cfg_pid_kd, cfg_pid_min, cfg_pid_max,
            cfg_gain_sched ? "ON" : "OFF");

        // Load cached roll control config from NVS
        prefs.begin("roll", false);
        cfg_use_angle_ctrl = prefs.getBool("ac", cfg_use_angle_ctrl);
        cfg_roll_delay_ms  = prefs.getUShort("rdly", cfg_roll_delay_ms);
        prefs.end();
        ESP_LOGI("CFG", "NVS Roll control: angle_ctrl=%s delay=%u ms",
            cfg_use_angle_ctrl ? "ON" : "OFF", (unsigned)cfg_roll_delay_ms);

        // Load cached guidance config from NVS
        prefs.begin("guid", false);
        cfg_guidance_en = prefs.getBool("en", cfg_guidance_en);
        prefs.end();
        ESP_LOGI("CFG", "NVS Guidance: %s", cfg_guidance_en ? "ON" : "OFF");

        // Load cached camera type from NVS
        prefs.begin("cam", false);
        cfg_camera_type = prefs.getUChar("type", cfg_camera_type);
        prefs.end();
        ESP_LOGI("CFG", "NVS Camera type: %u (%s)", cfg_camera_type,
                 cfg_camera_type == CAM_TYPE_GOPRO ? "GoPro" :
                 cfg_camera_type == CAM_TYPE_RUNCAM ? "RunCam" : "None");

        // Load cached pyro config from NVS
        prefs.begin("pyro", true);
        size_t pyro_sz = prefs.getBytesLength("cfg");
        if (pyro_sz == sizeof(PyroConfigData)) {
            PyroConfigData pcfg;
            prefs.getBytes("cfg", &pcfg, sizeof(pcfg));
            cfg_pyro1_enabled       = pcfg.ch1_enabled;
            cfg_pyro1_trigger_mode  = pcfg.ch1_trigger_mode;
            cfg_pyro1_trigger_value = pcfg.ch1_trigger_value;
            cfg_pyro2_enabled       = pcfg.ch2_enabled;
            cfg_pyro2_trigger_mode  = pcfg.ch2_trigger_mode;
            cfg_pyro2_trigger_value = pcfg.ch2_trigger_value;
        }
        prefs.end();
        ESP_LOGI("CFG", "NVS Pyro: ch1=%u/%u/%.1f ch2=%u/%u/%.1f",
                 cfg_pyro1_enabled, cfg_pyro1_trigger_mode, (double)cfg_pyro1_trigger_value,
                 cfg_pyro2_enabled, cfg_pyro2_trigger_mode, (double)cfg_pyro2_trigger_value);
    }

    if (config::USE_LORA_RADIO)
    {
        TR_LoRa_Comms::Config lora_cfg = {};
        lora_cfg.enabled = config::USE_LORA_RADIO;
        lora_cfg.cs_pin = config::LORA_CS_PIN;
        lora_cfg.dio1_pin = config::LORA_DIO1_PIN;
        lora_cfg.rst_pin = config::LORA_RST_PIN;
        lora_cfg.busy_pin = config::LORA_BUSY_PIN;
        lora_cfg.spi_sck = config::LORA_SPI_SCK;
        lora_cfg.spi_miso = config::LORA_SPI_MISO;
        lora_cfg.spi_mosi = config::LORA_SPI_MOSI;
        lora_cfg.spi_host = SPI3_HOST;  // SPI2 used by NAND/MRAM
        lora_cfg.freq_mhz = lora_freq_mhz;
        lora_cfg.spreading_factor = lora_sf;
        lora_cfg.bandwidth_khz = lora_bw_khz;
        lora_cfg.coding_rate = lora_cr;
        lora_cfg.preamble_len = config::LORA_PREAMBLE_LEN;
        lora_cfg.tx_power_dbm = lora_tx_power;
        lora_cfg.crc_on = config::LORA_CRC_ON;
        lora_cfg.rx_boosted_gain = config::LORA_RX_BOOSTED_GAIN;
        lora_cfg.syncword_private = config::LORA_SYNCWORD_PRIVATE;
        if (!lora_comms.begin(lora_cfg, config::DEBUG))
        {
            ESP_LOGE("PWR", "LoRa init failed");
        }
    }

    vTaskDelay(1);  // feed watchdog after LoRa init

    // I2C slave init is deferred to main loop — see initI2CSlave().
    // We wait until I2S DMA callbacks confirm the FC is alive, then
    // init the slave so the bus is stable.
    ESP_LOGI("PWR", "I2C slave deferred (waiting for FC I2S activity)");

    // I2S telemetry stream from FlightComputer (DMA-based slave RX)
    // Small DMA buffers (4 × 256 bytes = 1 KB) minimize latency.
    // FRAME_SYNC interrupt gating prevents stale replay regardless of
    // buffer count, but smaller buffers reduce read-to-parse latency.
    if (i2s_stream.beginSlaveRx(config::I2S_BCLK_PIN,
                                 config::I2S_WS_PIN,
                                 config::I2S_DIN_PIN,
                                 config::I2S_FSYNC_PIN,
                                 config::I2S_SAMPLE_RATE,
                                 4,     // dma_desc_num
                                 64) != ESP_OK)  // dma_frame_num → 256 bytes each
    {
        ESP_LOGE("PWR", "I2S slave RX init failed");
    }
    else
    {
        ESP_LOGI("PWR", "I2S slave RX ready");

        // Register zero-copy DMA receive callback.
        // The callback fires from ISR context each time a DMA buffer
        // completes, pushing bytes directly into rx_ring and notifying
        // the parser task.  This eliminates stale DMA re-reads that
        // caused periodic ~90ms gaps in logged data.
        esp_err_t cb_err = i2s_stream.registerRecvCallback(i2sRecvCallback, nullptr);
        if (cb_err != ESP_OK)
            ESP_LOGE("PWR", "I2S recv callback failed: %s", esp_err_to_name(cb_err));

        // Parser task — woken by DMA callback, parses frames from rx_ring.
        // Runs on Core 1 at same priority as loopTask so they round-robin.
        xTaskCreatePinnedToCore(i2sParserTask, "I2S Parse", 4096,
                                nullptr, 1, &i2s_rx_task_handle, 1);
    }

    // NOTE: do NOT pre-queue a status response here.  With the new
    // i2c_slave driver, data sits in the TX ringbuffer until the master
    // reads it.  Pre-queued stale data misaligns subsequent reads.
    // The FC's first OUT_STATUS_QUERY write will trigger a fresh response.

    peripherals_initialized = true;
    ESP_LOGI("PWR", "Peripherals initialized.");
}

// Deferred I2C slave init — called from the main loop once I2S DMA
// confirms the FC is alive and its I2C master GPIO pins are stable.
// Deferred I2C slave init — called from main loop once FC is confirmed alive
// (I2S DMA activity detected).  The ESP-IDF v5.3 i2c_slave driver had a heap
// overflow bug in s_i2c_handle_complete() that we patched locally; see
// esp-idf/components/esp_driver_i2c/i2c_slave.c.  Re-verify on ESP-IDF upgrade.
static bool i2c_slave_init_failed = false;  // latch so we don't retry and leak

static void initI2CSlave()
{
    if (i2c_slave_initialized || i2c_slave_init_failed) return;

    esp_err_t err = i2c_interface.beginSlave(
        config::I2C_SDA_PIN,
        config::I2C_SCL_PIN,
        config::I2C_CLOCK_HZ,    // ignored by slave (master drives clock)
        256,                      // rx_buffer_len (per-transaction, new API)
        config::I2C_SLAVE_TX_BUF, // tx_buffer_len (ringbuffer depth)
        false                     // no internal pull-ups (external on PCB)
    );
    if (err != ESP_OK)
    {
        ESP_LOGE("PWR", "I2C slave init failed: %s", esp_err_to_name(err));
        i2c_slave_init_failed = true;  // don't retry — beginSlave leaks on failure
        return;
    }
    ESP_LOGI("PWR", "I2C slave addr=0x%02X SDA=%d SCL=%d (deferred init OK)",
             config::I2C_ADDRESS, config::I2C_SDA_PIN, config::I2C_SCL_PIN);

    // Pre-fill TX ringbuffer so the first master read has data
    queueOutStatusResponse(true);

    i2c_slave_initialized = true;
}

static void setup_oc()
{
    // Ensure NVS is initialised (ESP-IDF on ESP32-P4/S3 may not auto-init)
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }

    delay(500);

    pinMode(config::PWR_PIN, OUTPUT);
    digitalWrite(config::PWR_PIN, LOW);   // Start with power rail OFF
    pwr_pin_on = false;

    ESP_LOGI("OC", "Starting OutComputer (low-power mode)...");

    // --- Load NVS settings early so config readback to app is correct ---
    {
        prefs.begin("lora", false);
        if (prefs.isKey("freq"))
        {
            lora_freq_mhz = prefs.getFloat("freq", config::LORA_FREQ_MHZ);
            lora_sf        = prefs.getUChar("sf",   config::LORA_SF);
            lora_bw_khz    = prefs.getFloat("bw",   config::LORA_BW_KHZ);
            lora_cr        = prefs.getUChar("cr",    config::LORA_CR);
            lora_tx_power  = (int8_t)prefs.getChar("txpwr", config::LORA_TX_POWER_DBM);
            ESP_LOGI("CFG", "NVS LoRa early load: %.1f MHz SF%u BW%.0f CR%u %d dBm",
                          (double)lora_freq_mhz, (unsigned)lora_sf,
                          (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);
        }
        prefs.end();

        prefs.begin("servo", false);
        if (prefs.isKey("b1"))
        {
            cfg_servo_bias1 = prefs.getShort("b1", 0);
            cfg_servo_hz    = prefs.getUShort("hz",  50);
            cfg_servo_min   = prefs.getUShort("min", 1000);
            cfg_servo_max   = prefs.getUShort("max", 2000);
            cfg_pid_kp      = prefs.getFloat("kp",    0.04f);
            cfg_pid_ki      = prefs.getFloat("ki",    0.001f);
            cfg_pid_kd      = prefs.getFloat("kd",    0.0003f);
            cfg_pid_min     = prefs.getFloat("mincmd", -20.0f);
            cfg_pid_max     = prefs.getFloat("maxcmd",  20.0f);
            cfg_gain_sched  = prefs.getBool("gs", false);
        }
        prefs.end();

        prefs.begin("guid", false);
        cfg_guidance_en = prefs.getBool("en", false);
        prefs.end();

        prefs.begin("roll", false);
        cfg_use_angle_ctrl = prefs.getBool("ac", false);
        cfg_roll_delay_ms  = prefs.getUShort("rdly", 0);
        prefs.end();

        // Early identity load (so config readback on first connect is correct)
        uint8_t mac[6];
        esp_efuse_mac_get_default(mac);
        snprintf(unit_id_hex, sizeof(unit_id_hex), "%02x%02x%02x%02x",
                 mac[2], mac[3], mac[4], mac[5]);

        prefs.begin("identity", false);
        if (!prefs.isKey("un"))
        {
            char default_name[24];
            snprintf(default_name, sizeof(default_name), "TR-R-%.4s", &unit_id_hex[4]);
            prefs.putBytes("un", default_name, strlen(default_name) + 1);
            prefs.putUChar("nid", config::DEFAULT_NETWORK_ID);
            prefs.putUChar("rid", config::DEFAULT_ROCKET_ID);
        }
        // Read unit name from NVS (stored as blob with null terminator)
        char nvs_name_buf[24] = "TinkerRocket";
        size_t un_len = prefs.getBytes("un", nvs_name_buf, sizeof(nvs_name_buf) - 1);
        if (un_len > 0) nvs_name_buf[un_len] = '\0';  // ensure null-terminated
        strncpy(unit_name, nvs_name_buf, sizeof(unit_name) - 1);
        unit_name[sizeof(unit_name) - 1] = '\0';
        network_id = prefs.getUChar("nid", config::DEFAULT_NETWORK_ID);
        rocket_id  = prefs.getUChar("rid", config::DEFAULT_ROCKET_ID);
        prefs.end();

        ESP_LOGI("CFG", "Identity early load: uid=%s name=%s nid=%u rid=%u",
                 unit_id_hex, unit_name, (unsigned)network_id, (unsigned)rocket_id);
        ble_app.setName(unit_name);
    }

    // --- INA230 power monitor (always-on, not behind PWR_PIN) ---
    {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port = I2C_NUM_1;
        bus_cfg.sda_io_num = static_cast<gpio_num_t>(config::PWR_SDA);
        bus_cfg.scl_io_num = static_cast<gpio_num_t>(config::PWR_SCL);
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.flags.enable_internal_pullup = false;

        esp_err_t err = i2c_new_master_bus(&bus_cfg, &ina230_bus);
        if (err != ESP_OK)
        {
            ESP_LOGW("PWR", "I2C bus init failed: %s", esp_err_to_name(err));
        }
    }
    if (ina230_bus != nullptr && ina230.begin(ina230_bus, 400000) == TR_INA230_OK)
    {
        // Start in power-down mode (0.5 uA) — we use triggered reads at 100 Hz.
        // 1 average × 332us keeps each triggered measurement fast (~0.7ms).
        // INA auto-powers-down after each trigger.
        ina230.setConfiguration(INA230_Avg::AVG_1,
                                INA230_ConvTime::CT_332us,
                                INA230_ConvTime::CT_332us,
                                INA230_Mode::POWER_DOWN);
        ina230.calibrate(INA230_R_SHUNT_OHM, INA230_CURRENT_LSB_A);
        ina230.enableConversionReadyAlert(true);  // enable CVRF bit for polling
        ina230_ok = true;
        ESP_LOGI("PWR", "INA230 OK (triggered mode, CVRF polling)");
    }
    else
    {
        ESP_LOGW("PWR", "INA230 not found -- battery monitoring disabled");
    }

    // Only BLE starts at boot — everything else is behind PWR_PIN
    if (!ble_app.begin())
    {
        ESP_LOGE("BLE", "BLE app interface failed to start");
    }

    // Enter low-power mode: 80 MHz, DFS to 40 MHz when idle
    enterLowPowerMode();

    last_stats_ms = millis();
    ESP_LOGI("OC", "OutComputer ready (PWR_PIN OFF, waiting for power-on command).");
}

static void loop_oc()
{
    // Serial debug console removed (was Arduino Serial.available/read).
    // Use ESP-IDF console component or BLE commands for debug interaction.

    // --- Active mode: FlightComputer + sensors powered on ---
    if (pwr_pin_on)
    {
        // Deferred I2C slave init: wait for I2S DMA activity confirming
        // the FC is alive before enabling the slave on the bus.
        if (!i2c_slave_initialized && dma_cb_count > 50)
        {
            initI2CSlave();
        }

        // Sim now runs on FlightComputer — I2C data always flows normally
        // Skip during blocking flash ops (file list/delete/download) to avoid
        // stalling the I2C slave response and causing FC timeout storms.
        if (i2c_slave_initialized && !flash_op_active)
            serviceI2CIngress();

        // Launch-triggered logging: start when NSF_LAUNCH appears in NonSensorData
        {
            static bool prev_ns_launch = false;
            const bool ns_launch = latest_non_sensor_valid &&
                                   nsFlagSet(latest_non_sensor.flags, NSF_LAUNCH);
            if (ns_launch && !prev_ns_launch)
            {
                logger.startLogging();
                ESP_LOGI("OC", "Launch detected - logging started");
            }
            prev_ns_launch = ns_launch;
        }
        logger.service();

        // Read power data at ~100 Hz (always, so BLE telemetry has fresh data
        // even before launch).  Only log to flash when logging is active.
        {
            static uint32_t last_pwr_log_ms = 0;
            uint32_t now_ms = millis();
            if ((now_ms - last_pwr_log_ms) >= 10) {
                last_pwr_log_ms = now_ms;
                readINA230Power();
                if (latest_power_valid && logger.isLoggingActive()) {
                    uint8_t pwr_frame[MAX_FRAME];
                    size_t  pwr_frame_len = 0;
                    if (TR_I2C_Interface::packMessage(POWER_MSG,
                                                       (const uint8_t*)&latest_power_raw,
                                                       sizeof(POWERData),
                                                       pwr_frame, sizeof(pwr_frame),
                                                       pwr_frame_len)) {
                        logger.enqueueFrame(pwr_frame, pwr_frame_len);
                    }
                }
            }
        }

        serviceLoRa();

        // Check for LoRa uplink commands between TX cycles
        serviceLoRaUplink();

        // Send name beacon so base station can identify us
        sendLoRaBeacon();
    }
    else
    {
        // --- Low-power mode: only BLE active ---
        // INA230 runs in continuous averaging mode (1024 samples ≈ 680ms window).
        // Just read the latest averaged result once per second — no trigger needed,
        // and the reading captures the true average including idle periods.
        {
            // ina_continuous is file-scope (reset on power-ON)
            if (!ina_continuous && ina230_ok) {
                ina230.setConfiguration(INA230_Avg::AVG_1024,
                                        INA230_ConvTime::CT_332us,
                                        INA230_ConvTime::CT_332us,
                                        INA230_Mode::SHUNT_BUS_CONTINUOUS);
                ina_continuous = true;
                ESP_LOGI("PWR", "INA230 switched to continuous averaging (1024 samples)");
            }
            static uint32_t last_ina_ms = 0;
            uint32_t now_ms = millis();
            if ((now_ms - last_ina_ms) >= 1000) {
                last_ina_ms = now_ms;
                // Read directly — INA230 is free-running, no trigger needed
                if (ina230_ok) {
                    float bus_v = 0.0f, current_a = 0.0f;
                    if (ina230.readBusVoltage_V(&bus_v) == TR_INA230_OK &&
                        ina230.readCurrent_A(&current_a) == TR_INA230_OK) {
                        POWERDataSI psi = {};
                        psi.time_us = (uint32_t)micros();
                        psi.voltage = bus_v;
                        // Invert so negative = discharging, matching base-station convention.
                        psi.current = -current_a * 1000.0f;
                        // SOC lookup (same as readINA230Power)
                        static constexpr float SOC_V[] = { 6.60f, 7.00f, 7.40f, 7.60f, 7.80f, 8.40f };
                        static constexpr float SOC_P[] = { 0.0f,  10.0f, 25.0f, 50.0f, 75.0f, 100.0f };
                        float soc = 0.0f;
                        if (bus_v <= SOC_V[0]) soc = SOC_P[0];
                        else if (bus_v >= SOC_V[5]) soc = SOC_P[5];
                        else {
                            for (int i = 0; i < 5; i++) {
                                if (bus_v <= SOC_V[i+1]) {
                                    soc = SOC_P[i] + (bus_v - SOC_V[i]) / (SOC_V[i+1] - SOC_V[i]) * (SOC_P[i+1] - SOC_P[i]);
                                    break;
                                }
                            }
                        }
                        psi.soc = soc;
                        sensor_converter.packPowerData(psi, latest_power_raw);
                        latest_power_valid = true;
                    }
                }
            }
        }

        // Yield to FreeRTOS — everything in low-power mode is 1 Hz or slower
        // (INA230 read, BLE telemetry, BLE command check).  BLE commands arrive
        // via NimBLE callbacks on their own task, no polling needed.
        // Longer delay = more time at 40 MHz (DFS min) = lower average current.
        delay(1000);

    }

    // Auto-send config on BLE connect (rising edge)
    {
        static bool ble_was_connected = false;
        bool ble_now = ble_app.isConnected();
        if (ble_now && !ble_was_connected) {
            delay(500); // Let app subscribe to notifications
            sendCurrentConfig();
            // In low-power mode, override the fast params set by onConnect
            // with slow params to save power.  The library's onConnect always
            // requests fast params (needed for file transfer when powered on),
            // so we correct it here after a short delay.
            if (!pwr_pin_on) {
                requestSlowBLEParams(0);
            }
        }
        ble_was_connected = ble_now;
    }

    // Check for BLE commands
    uint8_t ble_cmd = ble_app.getCommand();
    if (ble_cmd != 0)
    {
        ESP_LOGI("OC_CMD", "BLE cmd=%u", (unsigned)ble_cmd);
        if (ble_cmd == 1)
        {
            // Toggle camera recording
            camera_recording_requested = !camera_recording_requested;
            setPendingCommand(camera_recording_requested ? CAMERA_START : CAMERA_STOP);
            ESP_LOGI("BLE", "Camera toggle requested: %s", camera_recording_requested ? "START" : "STOP");
        }
        else if (ble_cmd == 2)
        {
            // Send file list with pagination (5 files per page)
            // Timestamp is parsed from filename on the app side, keeping JSON compact.
            beginPhoneIO();  // pause I2C servicing + I2S ingest during blocking NAND reads
            static constexpr size_t FILES_PER_PAGE = 5;
            uint8_t page = ble_app.getFileListPage();
            static TR_LogFileInfo infos[64];  // static to avoid stack overflow
            size_t total = logger.listFiles(infos, 64);
            qsort(infos, total, sizeof(TR_LogFileInfo), compareFilesDescending);  // Sort newest first

            // Calculate offset and count for this page
            size_t offset = page * FILES_PER_PAGE;
            size_t start = (offset < total) ? offset : total;
            size_t end = (start + FILES_PER_PAGE < total) ? start + FILES_PER_PAGE : total;
            size_t n = end - start;

            String json = "[";
            for (size_t i = start; i < end; ++i)
            {
                if (i > start) json += ",";
                json += "{\"name\":\"";
                json += infos[i].filename;
                json += "\",\"size\":";
                json += std::to_string(infos[i].size_bytes);
                json += "}";
            }
            json += "]";

            ble_app.sendFileList(json);
            endPhoneIO();

            ESP_LOGI("BLE", "Sent file list page %u: %u files (%u of %u, %u bytes)",
                          page, (unsigned)n, (unsigned)start, (unsigned)total, json.length());
        }
        else if (ble_cmd == 23)
        {
            // Toggle logging (manual start/stop from app)
            if (logger.isLoggingActive())
            {
                logger.endLogging();
                shadowFinalizeFlight();
                ESP_LOGI("OC_CMD", "Logging STOPPED (manual)");
            }
            else
            {
                logger.prepareLogFile();
                shadowPrepareFlight();
                logger.startLogging();
                dma_dump_requested = true;  // trigger DMA hex dump
                dma_dump_done = false;
                ESP_LOGI("OC_CMD", "Logging STARTED (manual)");
            }
        }
        else if (ble_cmd == 24)
        {
            // Servo test: set angles
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(ServoTestAnglesData))
            {
                memcpy(pending_config_data, payload, sizeof(ServoTestAnglesData));
                pending_config_data_len = sizeof(ServoTestAnglesData);
                pending_config_msg_type = SERVO_TEST_MSG;
                setPendingCommand(SERVO_TEST_PENDING);
                ESP_LOGI("BLE", "Servo test angles received");
            }
        }
        else if (ble_cmd == 25)
        {
            // Servo test: stop
            setPendingCommand(SERVO_TEST_STOP);
            ESP_LOGI("BLE", "Servo test stop");
        }
        else if (ble_cmd == 3)
        {
            // Delete file
            String filename = ble_app.getDeleteFilename();
            if (filename.length() > 0)
            {
                beginPhoneIO();  // pause I2C servicing + I2S ingest during blocking NAND erase + list
                bool success = logger.deleteFile(filename.c_str());
                ESP_LOGI("BLE", "Delete file '%s': %s", filename.c_str(), success ? "SUCCESS" : "FAILED");

                // Send updated file list after deletion (page 0)
                static TR_LogFileInfo infos[64];  // static to avoid stack overflow
                size_t total = logger.listFiles(infos, 64);
                qsort(infos, total, sizeof(TR_LogFileInfo), compareFilesDescending);  // Sort newest first

                size_t n = (total > 5) ? 5 : total;

                String json = "[";
                for (size_t i = 0; i < n; ++i)
                {
                    if (i > 0) json += ",";
                    json += "{\"name\":\"";
                    json += infos[i].filename;
                    json += "\",\"size\":";
                    json += std::to_string(infos[i].size_bytes);
                    json += "}";
                }
                json += "]";

                ble_app.sendFileList(json);
                endPhoneIO();

                ESP_LOGI("BLE", "Sent updated file list: %u of %u files (%u bytes)",
                              (unsigned)n, (unsigned)total, json.length());
            }
        }

        // Handle file download requests from BLE app
        String download_filename = ble_app.getDownloadFilename();
        if (download_filename.length() > 0)
        {
            beginPhoneIO();  // pause I2C servicing + I2S ingest during blocking flash reads
            ESP_LOGI("BLE", "Download file request: %s", download_filename.c_str());

            // Dynamic chunk size based on negotiated MTU (falls back to 170 if not yet negotiated)
            const size_t chunk_data_size = ble_app.getMaxChunkDataSize();
            ESP_LOGI("BLE", "Chunk data size: %u", (unsigned)chunk_data_size);

            if (chunk_data_size == 0)
            {
                ESP_LOGE("BLE", "chunk data size is 0, aborting download");
                ble_app.sendFileChunk(0, nullptr, 0, true);  // Send EOF to unblock app
            }
            else
            {

            // Frame-aligned BLE transfer: pack complete binary frames into each
            // BLE notification so that a dropped notification only loses whole
            // frames — no frame boundaries are corrupted.
            //
            // Frame format: [AA][55][AA][55][type(1)][len(1)][payload(len)][CRC(2)]
            // Max frame = 4+1+1+255+2 = 263 bytes, always fits in one notification.
            //
            // The iOS app just appends each notification's data.  A dropped
            // notification means a few missing frames rather than a corrupted
            // region of zero-filled gaps with broken CRCs.

            const size_t MAX_FRAME_SIZE = 263;
            const size_t FLASH_READ_SIZE = 4096;
            // read_buf holds carryover from previous iteration + new flash data
            static uint8_t read_buf[FLASH_READ_SIZE + MAX_FRAME_SIZE];
            static uint8_t ble_buf[502];  // Notification payload (max MTU data)

            uint32_t file_offset = 0;     // Current position in flash file
            size_t carryover = 0;         // Bytes carried from previous flash read
            size_t ble_used = 0;          // Bytes accumulated in ble_buf
            uint32_t bytes_sent = 0;      // Total BLE bytes sent
            uint32_t frames_sent = 0;
            uint32_t start_ms = millis();
            bool eof = false;

            // Delay between every BLE notification (matches Legacy base station
            // pacing).  The bursty 3-at-a-time batch scheme overwhelmed the iOS
            // BLE notification queue, causing ~50% data loss.  A consistent
            // per-chunk delay keeps the queue shallow and reliable.
            const unsigned long CHUNK_DELAY_MS = 30;

            while (!eof)
            {
                // Read next block from flash, appended after any carryover bytes
                size_t flash_bytes_read = 0;
                if (!logger.readFileChunk(download_filename.c_str(), file_offset,
                                          read_buf + carryover, FLASH_READ_SIZE,
                                          flash_bytes_read, eof))
                {
                    ESP_LOGE("BLE", "File read error, aborting download");
                    ble_app.sendFileChunk(bytes_sent, nullptr, 0, true);
                    break;
                }
                file_offset += flash_bytes_read;

                size_t buf_len = carryover + flash_bytes_read;
                size_t pos = 0;

                // Scan for complete frames
                while (pos + 8 <= buf_len)  // Min frame: SOF(4)+type(1)+len(1)+CRC(2)
                {
                    // Look for SOF: AA 55 AA 55
                    if (read_buf[pos]   != 0xAA || read_buf[pos+1] != 0x55 ||
                        read_buf[pos+2] != 0xAA || read_buf[pos+3] != 0x55)
                    {
                        pos++;
                        continue;
                    }

                    // Read payload length from frame header
                    uint8_t payload_len = read_buf[pos + 5];
                    size_t frame_size = 4 + 1 + 1 + payload_len + 2;

                    if (pos + frame_size > buf_len)
                    {
                        break;  // Incomplete frame — will carry over to next read
                    }

                    // Complete frame found — flush BLE buffer if this frame won't fit
                    if (ble_used > 0 && ble_used + frame_size > chunk_data_size)
                    {
                        ble_app.sendFileChunk(bytes_sent, ble_buf, ble_used, false);
                        bytes_sent += ble_used;
                        ble_used = 0;
                        delay(CHUNK_DELAY_MS);
                    }

                    // Append frame to BLE buffer
                    memcpy(ble_buf + ble_used, read_buf + pos, frame_size);
                    ble_used += frame_size;
                    frames_sent++;
                    pos += frame_size;
                }

                // Move unparsed bytes to start of buffer for next iteration
                carryover = buf_len - pos;
                if (carryover > 0 && pos > 0)
                {
                    memmove(read_buf, read_buf + pos, carryover);
                }
            }

            // Send remaining data with EOF flag
            if (ble_used > 0)
            {
                ble_app.sendFileChunk(bytes_sent, ble_buf, ble_used, true);
                bytes_sent += ble_used;
            }
            else
            {
                ble_app.sendFileChunk(bytes_sent, nullptr, 0, true);
            }

            // Redundant EOF in case the last notification was dropped
            delay(50);
            ble_app.sendFileChunk(bytes_sent, nullptr, 0, true);

            uint32_t elapsed_ms = millis() - start_ms;
            float kbps = (elapsed_ms > 0) ? (bytes_sent / 1024.0f) / (elapsed_ms / 1000.0f) : 0;
            ESP_LOGI("BLE", "Download complete: %lu frames, %lu bytes in %.1fs (%.1f KB/s)",
                          (unsigned long)frames_sent, (unsigned long)bytes_sent,
                          elapsed_ms / 1000.0f, kbps);
            } // else (chunk_data_size > 0)
            endPhoneIO();
        }

        // Flight simulator commands — relay to FlightComputer via I2C
        if (ble_cmd == 5)
        {
            // Configure simulation: [mass_g:4][thrust_n:4][burn_s:4][descent_rate_mps:4]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 12)
            {
                SimConfigData sim_cfg;
                float mass_g;
                memcpy(&mass_g,              payload + 0, 4);
                memcpy(&sim_cfg.thrust_n,    payload + 4, 4);
                memcpy(&sim_cfg.burn_time_s, payload + 8, 4);
                sim_cfg.mass_kg = mass_g / 1000.0f;
                sim_cfg.descent_rate_mps = 0.0f;
                if (plen >= 16) {
                    memcpy(&sim_cfg.descent_rate_mps, payload + 12, 4);
                }
                memcpy(pending_config_data, &sim_cfg, sizeof(sim_cfg));
                pending_config_data_len = sizeof(sim_cfg);
                pending_config_msg_type = SIM_CONFIG_MSG;
                setPendingCommand(SIM_CONFIG_PENDING);
                ESP_LOGI("OC", "SIM Config queued: mass=%.0fg thrust=%.1fN burn=%.1fs descent=%.1fm/s",
                              (double)mass_g, (double)sim_cfg.thrust_n,
                              (double)sim_cfg.burn_time_s, (double)sim_cfg.descent_rate_mps);
            }
        }
        else if (ble_cmd == 6)
        {
            setPendingCommand(SIM_START_CMD);
            ESP_LOGI("OC", "SIM Start queued for FlightComputer");
        }
        else if (ble_cmd == 7)
        {
            logger.endLogging();
            shadowFinalizeFlight();
            setPendingCommand(SIM_STOP_CMD);
            ESP_LOGI("OC", "SIM Stop queued for FlightComputer (logging ended)");
        }
        else if (ble_cmd == 15)
        {
            setPendingCommand(GROUND_TEST_START);
            ESP_LOGI("OC", "GROUND TEST Start queued for FlightComputer");
        }
        else if (ble_cmd == 16)
        {
            setPendingCommand(GROUND_TEST_STOP);
            ESP_LOGI("OC", "GROUND TEST Stop queued for FlightComputer");
        }
        else if (ble_cmd == 8)
        {
            // Toggle power rail
            pwr_pin_on = !pwr_pin_on;

            if (pwr_pin_on)
            {
                // Exit low-power mode BEFORE powering peripherals — need
                // full CPU speed and no auto light-sleep during init.
                exitLowPowerMode();

                // Power on the FlightComputer + sensors
                digitalWrite(config::PWR_PIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(500));  // Allow power rail to stabilize
                vTaskDelay(1);  // feed watchdog before long init
                initPeripherals();  // Initialize SPI, NAND, LoRa, I2C

                // Restore INA230 to fast single-shot config (low-power mode
                // sets AVG_1024 which makes triggered reads take ~680ms).
                if (ina230_ok) {
                    ina230.setConfiguration(INA230_Avg::AVG_1,
                                            INA230_ConvTime::CT_332us,
                                            INA230_ConvTime::CT_332us,
                                            INA230_Mode::POWER_DOWN);
                    ina_continuous = false;
                }

                // Restore fast BLE connection params for file transfer
                if (ble_app.isConnected())
                {
                    requestFastBLEParams(0);
                }
            }
            else
            {
                // Power off: shut down peripherals to return to low-power state
                ESP_LOGI("PWR", "Shutting down peripherals...");

                digitalWrite(config::PWR_PIN, LOW);

                // Delete the I2S receiver task
                if (i2s_rx_task_handle != nullptr)
                {
                    vTaskDelete(i2s_rx_task_handle);
                    i2s_rx_task_handle = nullptr;
                    ESP_LOGI("PWR", "I2S receiver task deleted");
                }
                // Skip I2S destructor — it blocks waiting for DMA.
                // The FC (clock master) is powered off, so the slave
                // peripheral is harmlessly idle.

                // I2C slave cleanup is handled by i2c_del_slave_device
                // when the interface is destroyed. No explicit cleanup needed here.
                i2c_slave_initialized = false;
                ESP_LOGI("PWR", "I2C slave stopped");

                // End SPI bus (stops LoRa, NAND, MRAM)
                SPI.end();
                ESP_LOGI("PWR", "SPI bus stopped");

                peripherals_initialized = false;

                // Re-enter low-power mode
                enterLowPowerMode();
                if (ble_app.isConnected())
                {
                    requestSlowBLEParams(0);
                }
                ESP_LOGI("PWR", "Low-power mode restored");
            }

            ESP_LOGI("BLE", "Power rail toggled: %s", pwr_pin_on ? "ON" : "OFF");

            // After power-on, NVS config is freshly loaded — resend config
            // readback so the app gets the actual persisted values.
            if (pwr_pin_on && ble_app.isConnected()) {
                delay(100);  // let peripherals finish init
                sendCurrentConfig();
            }
        }
        else if (ble_cmd == 9)
        {
            // Phone time sync: [year_lo][year_hi][month][day][hour][minute][second]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 7)
            {
                phone_utc_year   = payload[0] | ((uint16_t)payload[1] << 8);
                phone_utc_month  = payload[2];
                phone_utc_day    = payload[3];
                phone_utc_hour   = payload[4];
                phone_utc_minute = payload[5];
                phone_utc_second = payload[6];
                phone_sync_millis = millis();
                phone_time_valid = true;
                ESP_LOGI("BLE", "Time sync: %u-%02u-%02u %02u:%02u:%02u UTC",
                              phone_utc_year, phone_utc_month, phone_utc_day,
                              phone_utc_hour, phone_utc_minute, phone_utc_second);
            }
        }
        else if (ble_cmd == 10)
        {
            // LoRa reconfiguration: [freq:4f][bw:4f][sf:1][cr:1][txpwr:1]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 11)
            {
                float new_freq, new_bw;
                memcpy(&new_freq, payload + 0, 4);
                memcpy(&new_bw,   payload + 4, 4);
                uint8_t new_sf   = payload[8];
                uint8_t new_cr   = payload[9];
                int8_t  new_pwr  = (int8_t)payload[10];

                // Always update runtime vars and persist to NVS, even if the
                // radio isn't initialized yet (e.g. config sent before power-on).
                // The saved values will be loaded by initPeripherals() on next boot.
                lora_freq_mhz = new_freq;
                lora_bw_khz   = new_bw;
                lora_sf        = new_sf;
                lora_cr        = new_cr;
                lora_tx_power  = new_pwr;

                prefs.begin("lora", false);
                prefs.putFloat("freq",  lora_freq_mhz);
                prefs.putFloat("bw",    lora_bw_khz);
                prefs.putUChar("sf",    lora_sf);
                prefs.putUChar("cr",    lora_cr);
                prefs.putChar("txpwr",  lora_tx_power);
                prefs.end();

                // Verify NVS write by reading back
                prefs.begin("lora", false);
                float verify_freq = prefs.getFloat("freq", 0.0f);
                prefs.end();
                ESP_LOGI("CFG", "NVS LoRa verify: wrote %.1f, read back %.1f",
                              (double)lora_freq_mhz, (double)verify_freq);

                // Try to apply to live radio (may fail if not yet initialized)
                if (lora_comms.reconfigure(new_freq, new_sf, new_bw, new_cr, new_pwr))
                {
                    lora_comms.startReceive();
                    ESP_LOGI("BLE", "LoRa reconfigured + saved: %.1f MHz SF%u BW%.0f CR%u %d dBm",
                                  (double)lora_freq_mhz, (unsigned)lora_sf,
                                  (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);
                }
                else
                {
                    ESP_LOGI("BLE", "LoRa saved (radio not ready): %.1f MHz SF%u BW%.0f CR%u %d dBm",
                                  (double)lora_freq_mhz, (unsigned)lora_sf,
                                  (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);
                }
                sendCurrentConfig();
            }
        }
        else if (ble_cmd == 11)
        {
            // Rocket computer sound enable/disable: [enabled:1]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1)
            {
                bool enabled = (payload[0] != 0);
                setPendingCommand(enabled ? SOUNDS_ENABLE : SOUNDS_DISABLE);
                ESP_LOGI("BLE", "Sounds: %s (pending for RocketComputer)",
                              enabled ? "ENABLE" : "DISABLE");
            }
        }
        else if (ble_cmd == 12)
        {
            // Servo configuration: [bias1:2][bias2:2][bias3:2][bias4:2][hz:2][min:2][max:2] = 14 bytes
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 14)
            {
                memcpy(pending_config_data, payload, 14);
                pending_config_data_len = 14;
                pending_config_msg_type = SERVO_CONFIG_MSG;
                setPendingCommand(SERVO_CONFIG_PENDING);
                cacheServoConfig(payload, plen);
                ESP_LOGI("BLE", "Servo config queued for RocketComputer");
            }
        }
        else if (ble_cmd == 13)
        {
            // PID configuration: [kp:4f][ki:4f][kd:4f][min_cmd:4f][max_cmd:4f] = 20 bytes
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 20)
            {
                memcpy(pending_config_data, payload, 20);
                pending_config_data_len = 20;
                pending_config_msg_type = PID_CONFIG_MSG;
                setPendingCommand(PID_CONFIG_PENDING);
                cachePIDConfig(payload, plen);
                ESP_LOGI("BLE", "PID config queued for RocketComputer");
            }
        }
        else if (ble_cmd == 14)
        {
            // Servo control enable/disable: [enabled:1]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1)
            {
                bool enabled = (payload[0] != 0);
                cfg_servo_enabled = enabled;
                setPendingCommand(enabled ? SERVO_CTRL_ENABLE : SERVO_CTRL_DISABLE);
                ESP_LOGI("BLE", "Servo control: %s (pending for RocketComputer)",
                              enabled ? "ENABLE" : "DISABLE");
            }
        }
        else if (ble_cmd == 20)
        {
            // Config readback request
            sendCurrentConfig();
        }
        else if (ble_cmd == 21)
        {
            setPendingCommand(GYRO_CAL_CMD);
            ESP_LOGI("BLE", "Gyro cal request -> FlightComputer");
        }
        else if (ble_cmd == 22)
        {
            // Gain schedule enable/disable: [enabled:1]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1) {
                bool enabled = (payload[0] != 0);
                cfg_gain_sched = enabled;
                setPendingCommand(enabled ? GAIN_SCHED_ENABLE : GAIN_SCHED_DISABLE);
                // Cache in OutComputer NVS for config readback
                prefs.begin("pid", false);
                prefs.putBool("gs", enabled);
                prefs.end();
                ESP_LOGI("BLE", "Gain schedule: %s -> FlightComputer",
                              enabled ? "ENABLE" : "DISABLE");
            }
        }
        else if (ble_cmd == 26)
        {
            // Roll profile set: [num_wp:1][pad:3][wp0_time:4f][wp0_angle:4f]...[wp7_time:4f][wp7_angle:4f]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(RollProfileData))
            {
                memcpy(pending_config_data, payload, sizeof(RollProfileData));
                pending_config_data_len = sizeof(RollProfileData);
                pending_config_msg_type = ROLL_PROFILE_MSG;
                setPendingCommand(ROLL_PROFILE_PENDING);
                ESP_LOGI("BLE", "Roll profile (%d waypoints) queued for RocketComputer",
                              payload[0]);
            }
            else
            {
                ESP_LOGW("BLE", "Roll profile payload too short (%u < %u)",
                              (unsigned)plen, (unsigned)sizeof(RollProfileData));
            }
        }
        else if (ble_cmd == 27)
        {
            // Roll profile clear (no payload)
            setPendingCommand(ROLL_PROFILE_CLEAR);
            ESP_LOGI("BLE", "Roll profile CLEAR -> FlightComputer");
        }
        else if (ble_cmd == 29)
        {
            // Servo replay: send flight data sample through control loop
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(ServoReplayData))
            {
                memcpy(pending_config_data, payload, sizeof(ServoReplayData));
                pending_config_data_len = sizeof(ServoReplayData);
                pending_config_msg_type = SERVO_REPLAY_MSG;
                setPendingCommand(SERVO_REPLAY_PENDING);
                ESP_LOGI("BLE", "Servo replay data queued");
            }
        }
        else if (ble_cmd == 30)
        {
            // Servo replay: stop
            setPendingCommand(SERVO_REPLAY_STOP);
            ESP_LOGI("BLE", "Servo replay stop");
        }
        else if (ble_cmd == 31)
        {
            // Roll control config: [use_angle_control:1][pad:1][roll_delay_ms:2]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 4)
            {
                memcpy(pending_config_data, payload, 4);
                pending_config_data_len = 4;
                pending_config_msg_type = ROLL_CTRL_CONFIG_MSG;
                setPendingCommand(ROLL_CTRL_CONFIG_PENDING);
                cacheRollControlConfig(payload, plen);
                ESP_LOGI("BLE", "Roll control config queued");
            }
        }
        else if (ble_cmd == 32)
        {
            // Guidance enable/disable: [enabled:1]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1) {
                bool enabled = (payload[0] != 0);
                cfg_guidance_en = enabled;
                setPendingCommand(enabled ? GUIDANCE_ENABLE : GUIDANCE_DISABLE);
                // Cache in NVS so config readback stays in sync
                Preferences prefs;
                prefs.begin("guid", false);
                prefs.putBool("en", enabled);
                prefs.end();
                ESP_LOGI("BLE", "Guidance: %s", enabled ? "ENABLE" : "DISABLE");
            }
        }
        else if (ble_cmd == 33)
        {
            // Camera type config: [camera_type:1]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1 && payload[0] <= CAM_TYPE_RUNCAM) {
                cfg_camera_type = payload[0];
                // Send to FC via I2C config
                CameraConfigData cam_cfg;
                cam_cfg.camera_type = cfg_camera_type;
                memcpy(pending_config_data, &cam_cfg, sizeof(cam_cfg));
                pending_config_data_len = sizeof(cam_cfg);
                pending_config_msg_type = CAMERA_CONFIG_MSG;
                setPendingCommand(CAMERA_CONFIG_PENDING);
                // Cache in NVS
                Preferences prefs;
                prefs.begin("cam", false);
                prefs.putUChar("type", cfg_camera_type);
                prefs.end();
                ESP_LOGI("BLE", "Camera type: %u (%s)",
                         cfg_camera_type,
                         cfg_camera_type == CAM_TYPE_GOPRO ? "GoPro" :
                         cfg_camera_type == CAM_TYPE_RUNCAM ? "RunCam" : "None");
            }
        }
        else if (ble_cmd == 34)
        {
            // Pyro config: [ch1_en:1][ch1_mode:1][ch1_val:4f][ch2_en:1][ch2_mode:1][ch2_val:4f] = 12 bytes
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(PyroConfigData)) {
                PyroConfigData pcfg;
                memcpy(&pcfg, payload, sizeof(pcfg));
                cfg_pyro1_enabled       = pcfg.ch1_enabled;
                cfg_pyro1_trigger_mode  = pcfg.ch1_trigger_mode;
                cfg_pyro1_trigger_value = pcfg.ch1_trigger_value;
                cfg_pyro2_enabled       = pcfg.ch2_enabled;
                cfg_pyro2_trigger_mode  = pcfg.ch2_trigger_mode;
                cfg_pyro2_trigger_value = pcfg.ch2_trigger_value;
                // Queue for FC via I2C
                memcpy(pending_config_data, &pcfg, sizeof(pcfg));
                pending_config_data_len = sizeof(pcfg);
                pending_config_msg_type = PYRO_CONFIG_MSG;
                setPendingCommand(PYRO_CONFIG_PENDING);
                // Persist to NVS
                Preferences prefs;
                prefs.begin("pyro", false);
                size_t written = prefs.putBytes("cfg", &pcfg, sizeof(pcfg));
                prefs.end();
                ESP_LOGI("BLE", "Pyro config: ch1=%u/%u/%.1f ch2=%u/%u/%.1f (NVS wrote %u bytes)",
                         pcfg.ch1_enabled, pcfg.ch1_trigger_mode, (double)pcfg.ch1_trigger_value,
                         pcfg.ch2_enabled, pcfg.ch2_trigger_mode, (double)pcfg.ch2_trigger_value,
                         (unsigned)written);
            } else {
                ESP_LOGW("BLE", "Pyro config: payload too short (%u < %u)",
                         (unsigned)plen, (unsigned)sizeof(PyroConfigData));
            }
        }
        else if (ble_cmd == 35)
        {
            // Pyro continuity test — 1 byte payload: channel (1 or 2)
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            uint8_t ch = (plen >= 1) ? payload[0] : 0;
            // Pack channel into config data so FC knows which channel
            pending_config_data[0] = ch;
            pending_config_data_len = 1;
            pending_config_msg_type = PYRO_CONT_TEST;
            setPendingCommand(PYRO_CONT_TEST);
            ESP_LOGI("BLE", "Pyro continuity test CH%u", ch);
        }
        else if (ble_cmd == 36)
        {
            // Pyro test fire — 1 byte payload: channel (1 or 2)
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            uint8_t ch = (plen >= 1) ? payload[0] : 0;
            pending_config_data[0] = ch;
            pending_config_data_len = 1;
            pending_config_msg_type = PYRO_FIRE_TEST;
            setPendingCommand(PYRO_FIRE_TEST);
            ESP_LOGI("BLE", "Pyro test fire CH%u", ch);
        }
        // ---- Device Identity Commands ----
        else if (ble_cmd == 40)
        {
            // Set unit name — payload is UTF-8 string, max 20 bytes
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen > 0 && plen <= 20)
            {
                char new_name[24];
                memcpy(new_name, payload, plen);
                new_name[plen] = '\0';
                strncpy(unit_name, new_name, sizeof(unit_name) - 1);
                unit_name[sizeof(unit_name) - 1] = '\0';
                // Persist to NVS
                Preferences prefs;
                prefs.begin("identity", false);
                prefs.putBytes("un", new_name, strlen(new_name) + 1);
                prefs.end();
                // Update BLE advertising name
                ble_app.setName(unit_name);
                // Send updated identity readback to app
                sendCurrentConfig();
                ESP_LOGI("BLE", "Unit name set: %s", unit_name);
            }
        }
        else if (ble_cmd == 41)
        {
            // Set network_id — payload: [nid:1]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1)
            {
                network_id = payload[0];
                Preferences prefs;
                prefs.begin("identity", false);
                prefs.putUChar("nid", network_id);
                prefs.end();
                sendCurrentConfig();
                ESP_LOGI("BLE", "Network ID set: %u", (unsigned)network_id);
            }
        }
        else if (ble_cmd == 42)
        {
            // Set rocket_id — payload: [rid:1]
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1 && payload[0] > 0 && payload[0] < 255)
            {
                rocket_id = payload[0];
                Preferences prefs;
                prefs.begin("identity", false);
                prefs.putUChar("rid", rocket_id);
                prefs.end();
                sendCurrentConfig();
                ESP_LOGI("BLE", "Rocket ID set: %u", (unsigned)rocket_id);
            }
        }
    }

    printStats();
    vTaskDelay(1);  // yield to FreeRTOS scheduler
}

extern "C" void app_main(void)
{
    setup_oc();
    xTaskCreatePinnedToCore([](void*) { while (true) { loop_oc(); } },
                            "oc_loop", 12 * 1024, NULL, 5, NULL, 1);
}
