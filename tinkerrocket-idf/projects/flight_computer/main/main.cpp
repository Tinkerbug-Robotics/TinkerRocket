#include <compat.h>
#include <TR_NVS.h>

// Configuration Parameters
#include "config.h"

// Libraries
#include <TR_Sensor_Collector.h>
#include <TR_Sensor_Collector_Sim.h>
#include <TR_I2C_Interface.h>
#include <TR_I2S_Stream.h>
#include <TR_Sensor_Data_Converter.h>
#include <TR_GpsInsEKF.h>
#include <TR_KinematicChecks.h>
#include <TR_ServoControl_ledc_mult.h>
#include <TR_GuidancePN.h>
#include <TR_ControlMixer.h>
#include <RocketComputerTypes.h>
#include <driver/spi_master.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_private/esp_gpio_reserve.h>
#include <rom/gpio.h>              // esp_rom_gpio_connect_out_signal
#include <soc/gpio_sig_map.h>      // SIG_GPIO_OUT_IDX
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <CRC32.h>
static const char* TAG = "FC";

// EKF timeUpdate()/measUpdate() allocate ~7.5 KB of temporary 15x15 matrices
// on the stack.  The FreeRTOS task in app_main() is created with 16 KB.

// IMU sensor data collection library (real hardware)
SensorCollector sensor_collector_hw(config::ISM6HG256_CS,
                                    config::ISM6HG256_INT,
                                    config::ISM6HG256_UPDATE_RATE,
                                    config::BMP585_CS,
                                    config::BMP585_INT,
                                    config::BMP585_UPDATE_RATE,
                                    config::MMC5983MA_CS,
                                    config::MMC5983MA_INT,
                                    config::MMC5983MA_UPDATE_RATE,
                                    config::IIS2MDC_SDA,
                                    config::IIS2MDC_SCL,
                                    config::IIS2MDC_INT,
                                    config::IIS2MDC_I2C_FREQ_HZ,
                                    config::IIS2MDC_I2C_ADDR,
                                    config::GNSS_UPDATE_RATE,
                                    config::GNSS_RX,
                                    config::GNSS_TX,
                                    config::GNSS_RESET_N,
                                    config::GNSS_SAFEBOOT_N,
                                    config::USE_BMP585,
                                    config::USE_MMC5983MA,
                                    config::USE_IIS2MDC,
                                    config::USE_GNSS,
                                    config::USE_ISM6HG256,
                                    SPI,
                                    config::SPI_SPEED);

// Sim wrapper: passthrough when sim inactive, replaces data when sim active
SensorCollectorSim sensor_collector(sensor_collector_hw);

// I2C interface for command/config channel to OutComputer
TR_I2C_Interface i2c_interface(config::ESP_I2C_ADR);

// I2S stream for high-frequency telemetry to OutComputer
static TR_I2S_Stream i2s_stream;
SensorConverter sensor_converter;

static ISM6HG256Data ism6hg256_data;
static uint8_t ism6hg256_data_buffer[SIZE_OF_ISM6HG256_DATA];
static BMP585Data bmp585_data;
static uint8_t bmp585_data_buffer[SIZE_OF_BMP585_DATA];
static MMC5983MAData mmc5983ma_data;
static uint8_t mmc5983ma_data_buffer[SIZE_OF_MMC5983MA_DATA];
static IIS2MDCData iis2mdc_data;
static uint8_t iis2mdc_data_buffer[SIZE_OF_IIS2MDC_DATA];
static IIS2MDCDataSI iis2mdc_latest_si = {};
static bool have_iis2mdc_si = false;
static GNSSData gnss_data;
static uint8_t gnss_data_buffer[SIZE_OF_GNSS_DATA];
static NonSensorData non_sensor_data;
static uint8_t non_sensor_data_buffer[SIZE_OF_NON_SENSOR_DATA];

typedef struct
{
    uint8_t type;
    uint8_t len;
    uint8_t payload[MAX_PAYLOAD];
} I2STxMessage;

static QueueHandle_t i2s_tx_queue = nullptr;
static TaskHandle_t  i2s_sender_task_handle = nullptr;

// Cache for config frame bytes read in the same I2S transaction as the
// OUT_STATUS_RESPONSE.  The ESP32 I2S slave hardware FIFO discards bytes
// remaining after a master STOP, so both frames must be read in one shot.
static constexpr size_t COMBINED_READ_SIZE = 96;  // 10 (response) + up to 86 (config)
static uint8_t  cfg_read_cache[COMBINED_READ_SIZE];
static size_t   cfg_read_cache_len = 0;

// Flight loop parameters
uint32_t last_flight_loop_update_time = 0;
uint32_t flight_loop_period = (uint32_t)(1000000/config::FLIGHT_LOOP_UPDATE_RATE);
uint32_t last_non_sensor_tx_time_us = 0;
uint32_t non_sensor_tx_period_us = (uint32_t)(1000000UL / config::NON_SENSOR_UPDATE_RATE);

uint32_t i2s_tx_ok = 0;
uint32_t i2s_tx_fail = 0;
int i2s_last_tx_err = ESP_OK;
uint32_t i2c_query_ok = 0;
uint32_t i2c_query_fail = 0;
uint32_t i2c_gor_max_us = 0;  // peak getOutReady() duration per diagnostic window
uint32_t i2s_tx_ism6_ok = 0;
uint32_t i2s_tx_ism6_fail = 0;
uint32_t i2s_tx_bmp_ok = 0;
uint32_t i2s_tx_bmp_fail = 0;
uint32_t i2s_tx_mmc_ok = 0;
uint32_t i2s_tx_mmc_fail = 0;
uint32_t i2s_tx_iis2mdc_ok = 0;
uint32_t i2s_tx_iis2mdc_fail = 0;
uint32_t i2s_tx_gnss_ok = 0;
uint32_t i2s_tx_gnss_fail = 0;
uint32_t i2s_tx_ns_ok = 0;
uint32_t i2s_tx_ns_fail = 0;
uint32_t i2s_tx_enqueue_ok = 0;
uint32_t i2s_tx_enqueue_drop = 0;

// --- Loop timing instrumentation ---
static uint32_t lt_ekf_total_us = 0;
static uint32_t lt_ekf_max_us = 0;
static uint32_t lt_ekf_count = 0;
static uint32_t lt_loop_count = 0;
static uint32_t lt_loop_max_us __attribute__((unused)) = 0;

// --- Converted SI values (latest sample of each sensor) ---
static ISM6HG256DataSI ism6_latest_si = {};
static BMP585DataSI bmp_latest_si = {};
static MMC5983MADataSI mmc_latest_si = {};
static GNSSDataSI gnss_latest_si = {};
static bool have_ism6_si = false;
static bool have_bmp_si = false;
static bool have_mmc_si = false;
static bool have_gnss_si = false;
static bool bmp_new_for_kf = false; // Set when new BMP sample arrives, cleared after KF update

// --- Flight logic state ---
static RocketState rocket_state = INITIALIZATION;
static uint32_t launch_time_millis = 0;
static uint32_t prelaunch_time_millis = 0;
static uint32_t valid_gnss_start_millis = 0;
static uint32_t landed_candidate_start_millis = 0;
static bool gnss_started = false;
static bool ground_pressure_found = false;
static bool out_ready = false;
static uint32_t out_ready_request_time_ms = 0;
static uint8_t out_pending_command = 0U;
static uint8_t last_processed_cmd = 0U;  // dedup: ignore OutComputer repeats
static bool end_flight_sent = false;
static OutStatusQueryData out_status_query_data = {};
static float ground_pressure_pa = 101325.0f;
static float pressure_alt_m = 0.0f;
static float pressure_alt_rate_mps = 0.0f;
static float max_alt_m = 0.0f;
static float max_speed_mps = 0.0f;
static GpsInsEKF ekf;
static TR_KinematicChecks kinematics;
static bool ekf_initialized = false;
// Launch-site reference position for converting EKF LLA to local ENU.
// Running average of GNSS fixes on the pad, frozen at launch or after 2 min.
static double ref_lat_rad = 0.0, ref_lon_rad = 0.0, ref_alt_m = 0.0;
static bool have_ref_pos = false;
static double ref_lat_sum = 0.0, ref_lon_sum = 0.0, ref_alt_sum = 0.0;
static uint32_t ref_pos_count = 0;
static bool ref_pos_frozen = false;
static constexpr uint32_t REF_POS_MAX_AGE_MS = 120000; // 2 minutes
static uint32_t ref_pos_first_time_ms = 0;
// Last GNSS time_us fed to EKF (avoid double-counting same sample)
static uint32_t last_gnss_time_us_for_ekf = 0;
// Duplicate-fix detection: track GPS fix timestamp (second + milli_second)
// to avoid feeding the same fix to the EKF when the poll rate exceeds the
// receiver's actual output rate.
static uint8_t  last_gnss_fix_second = 0xFF;
static uint16_t last_gnss_fix_ms     = 0xFFFF;
static bool landed_actions_done = false;
static bool gopro_recording = false;
static bool gopro_pulse_active = false;
static uint32_t gopro_pulse_end_ms = 0;
static uint8_t runtime_camera_type = config::CAMERA_TYPE;  // can be overridden via BLE
static bool servo_enabled = false;
static bool gain_sched_enabled = config::GAIN_SCHEDULE_ENABLED;
static bool use_angle_control = config::USE_ANGLE_CONTROL;
static uint16_t roll_delay_ms = config::ROLL_CONTROL_DELAY_MS;
// --- Guidance (PN) state ---
static TR_GuidancePN guidance;
static TR_ControlMixer control_mixer;
static TR_PID roll_rate_pid_standalone(config::KP, config::KI, config::KD,
                                       config::MAX_CMD, config::MIN_CMD);
static bool guidance_enabled = config::GUIDANCE_ENABLED;
static bool burnout_detected = false;
static uint32_t burnout_time_ms = 0;
static bool mach_locked_out = false;    // promoted from baro block for apogee voting
static bool gps_new_for_kc = false;     // new GPS sample available for kinematic checks
static bool guidance_active = false;
static bool ground_test_active = false;
// Last roll fin command (deg) produced by the ground-test / guidance paths
// that bypass servo_control's internal PID. Used to populate
// NonSensorData.roll_cmd when those paths are active so telemetry reflects
// what was actually sent to the servos.
static float last_external_roll_cmd_deg = 0.0f;
static bool servo_test_active = false;
static float servo_test_angles[4] = {0, 0, 0, 0};
static bool servo_replay_active = false;
static bool prev_sim_active = false;
// Roll profile: (time, angle) waypoints for cascaded angle controller
static RollProfileData roll_profile = {};  // zeroed → num_waypoints = 0 (rate-only)
static Preferences prefs;
static bool enable_sounds = config::ENABLE_SOUNDS;
static bool ready_chirp_played = false;
static uint32_t last_heartbeat_beep_ms = 0;
static bool piezo_pwm_ready = false;
static bool piezo_wave_active = false;
static bool piezo_pin_high = false;
static volatile int64_t piezo_wave_end_us = 0;
static esp_timer_handle_t piezo_toggle_timer = nullptr;
static uint32_t piezo_half_period_us = 0;
static bool blue_led_flash_active = false;
static uint32_t blue_led_flash_end_ms = 0;

// --- Pyro channel state ---
// Spinlock protects all pyro state flags and GPIO operations.
// Must be held when reading or writing armed/fired flags or toggling ARM/FIRE pins.
static portMUX_TYPE pyro_spinlock = portMUX_INITIALIZER_UNLOCKED;
static PyroConfigData pyro_config = {};  // zeroed = both disabled
static bool pyro1_armed = false;
static bool pyro1_fired = false;
static uint32_t pyro1_fire_start_ms = 0;
static bool pyro2_armed = false;
static bool pyro2_fired = false;
static uint32_t pyro2_fire_start_ms = 0;
static bool pyro_apogee_detected = false;
static uint32_t pyro_apogee_time_ms = 0;
// --- Inflight reboot recovery ---
// Snapshot of critical flight state, sent to OC over I2S at 10 Hz during
// INFLIGHT and stored in OC's MRAM.  On unexpected reboot (brownout,
// watchdog), FC queries OC over I2C and restores state to resume flight ops
// with correct pyro state, EKF navigation, and flight phase.
//
// Wire format is FlightSnapshotData in RocketComputerTypes.h — keeps the
// FC's flash and NVS off the hot path (#104).

// Forward decl — full definition is below the I2S sender setup.
static inline bool enqueueI2STx(uint8_t type, const uint8_t *payload, size_t len);

static bool     reboot_recovery = false;      // true during servo settle period after recovery
static bool     reboot_recovery_telem = false; // true for rest of flight (telemetry flag)
static uint32_t servo_settle_end_ms = 0;      // hold servos neutral until this time
static uint32_t last_snapshot_ms = 0;         // rate-limit NVS writes to 10 Hz

static uint32_t computeSnapshotCRC(const FlightSnapshotData& snap)
{
    CRC32 crc;
    crc.add(reinterpret_cast<const uint8_t*>(&snap),
            offsetof(FlightSnapshotData, crc32));
    return crc.calc();
}

// Build a FlightSnapshotData from current FC state.  Used by both the
// periodic save (sends to OC over I2S) and clearFlightSnapshot (sends a
// state=LANDED snapshot to invalidate recovery on next boot).
static void buildFlightSnapshot(FlightSnapshotData& snap, uint32_t now_ms, uint8_t override_state = 0xFF)
{
    snap.magic        = FlightSnapshotData::MAGIC;
    snap.version      = FlightSnapshotData::VERSION;
    snap.rocket_state = (override_state != 0xFF) ? override_state : (uint8_t)rocket_state;

    snap.flight_elapsed_ms  = now_ms - launch_time_millis;
    snap.apogee_elapsed_ms  = pyro_apogee_detected
                            ? (pyro_apogee_time_ms - launch_time_millis) : 0;
    snap.burnout_elapsed_ms = burnout_detected
                            ? (burnout_time_ms - launch_time_millis) : 0;

    portENTER_CRITICAL(&pyro_spinlock);
    snap.pyro_apogee_detected = pyro_apogee_detected ? 1 : 0;
    snap.pyro1_armed          = pyro1_armed          ? 1 : 0;
    snap.pyro1_fired          = pyro1_fired          ? 1 : 0;
    snap.pyro2_armed          = pyro2_armed          ? 1 : 0;
    snap.pyro2_fired          = pyro2_fired          ? 1 : 0;
    portEXIT_CRITICAL(&pyro_spinlock);

    snap.ground_pressure_pa = ground_pressure_pa;
    snap.ref_lat_rad        = ref_lat_rad;
    snap.ref_lon_rad        = ref_lon_rad;
    snap.ref_alt_m          = ref_alt_m;

    snap.ekf_initialized = ekf_initialized  ? 1 : 0;
    snap.guidance_enabled = guidance_enabled ? 1 : 0;
    snap.burnout_detected = burnout_detected ? 1 : 0;
    snap.servo_enabled    = servo_enabled    ? 1 : 0;

    if (ekf_initialized) {
        EkfStateSnapshot s;
        ekf.getState(s);
        memcpy(snap.ekf_pos_rrm,     s.pos_rrm,     sizeof(snap.ekf_pos_rrm));
        memcpy(snap.ekf_vel_ned_mps, s.vel_ned_mps, sizeof(snap.ekf_vel_ned_mps));
        memcpy(snap.ekf_quat,        s.quat,        sizeof(snap.ekf_quat));
        memcpy(snap.ekf_accel_bias,  s.accel_bias,  sizeof(snap.ekf_accel_bias));
        memcpy(snap.ekf_gyro_bias,   s.gyro_bias,   sizeof(snap.ekf_gyro_bias));
        // getCovDiag wants a 'float (&)[15]' reference; packed-struct
        // fields can't bind to it.  Stage through a local first.
        float diag_tmp[15];
        ekf.getCovDiag(diag_tmp);
        memcpy(snap.ekf_P_diag, diag_tmp, sizeof(diag_tmp));
        snap.ekf_t_prev_us = s.t_prev_us;
        memcpy(snap.ekf_euler,       s.euler,       sizeof(snap.ekf_euler));
    }

    snap.crc32 = computeSnapshotCRC(snap);
}

static void saveFlightSnapshot(uint32_t now_ms)
{
    FlightSnapshotData snap = {};
    buildFlightSnapshot(snap, now_ms);

    // Hand off to the I2S sender — non-blocking, runs entirely from Core 1
    // RAM.  ~13 KB/s additional bandwidth on top of the existing ~78 KB/s
    // sensor stream.  No flash, no NVS, no cache disable.
    (void)enqueueI2STx(SNAPSHOT_MSG,
                       reinterpret_cast<const uint8_t*>(&snap),
                       sizeof(snap));
}

static void clearFlightSnapshot()
{
    // Send a snapshot with rocket_state=LANDED so the OC's MRAM holds an
    // "invalid for recovery" state.  The recovery path on next boot only
    // restores when state==INFLIGHT, so a LANDED snapshot effectively
    // clears the recovery slot via the same I2S path as normal saves.
    FlightSnapshotData snap = {};
    buildFlightSnapshot(snap, millis(), /* override_state = */ (uint8_t)LANDED);
    (void)enqueueI2STx(SNAPSHOT_MSG,
                       reinterpret_cast<const uint8_t*>(&snap),
                       sizeof(snap));
}

static const char* resetReasonStr(esp_reset_reason_t r)
{
    switch (r) {
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_SW:        return "SW";
        default:                return "OTHER";
    }
}

enum class BootChirpPhase : uint8_t { Idle, GapAfterBeep1, WaitingBeep2End };
static BootChirpPhase boot_chirp_phase = BootChirpPhase::Idle;
static uint32_t boot_chirp_next_ms = 0;
static TR_ServoControl servo_control(config::SERVO_PIN_1,
                                     config::SERVO_PIN_2,
                                     config::SERVO_PIN_3,
                                     config::SERVO_PIN_4,
                                     config::SERVO_BIAS_1,
                                     config::SERVO_BIAS_2,
                                     config::SERVO_BIAS_3,
                                     config::SERVO_BIAS_4,
                                     config::SERVO_HZ,
                                     config::SERVO_MIN_US,
                                     config::SERVO_MAX_US,
                                     config::KP,
                                     config::KI,
                                     config::KD,
                                     config::MIN_CMD,
                                     config::MAX_CMD);

static bool servoPinsValid()
{
    return (config::SERVO_PIN_1 != 255U) &&
           (config::SERVO_PIN_2 != 255U) &&
           (config::SERVO_PIN_3 != 255U) &&
           (config::SERVO_PIN_4 != 255U);
}

// Read a config data frame from OutComputer's I2C slave TX buffer.
// Called after getOutReady() returns a config-pending command.
//
// The ESP32 I2C slave driver consumes 1 extra byte from the TX FIFO
// during compound (repeated-START) transactions.  This means the config
// frame's first SOF byte (0xAA) is often already consumed by the time
// we read here.  To handle this we scan the buffer for the
// [type][len] signature and validate with CRC (which covers type+len+payload
// but NOT the SOF bytes), making us alignment-independent.
//
// Reads extra bytes and retries up to 3 times.
static bool readConfigFrame(uint8_t expected_type,
                            size_t  expected_payload_len,
                            uint8_t* payload_out,
                            size_t  payload_capacity,
                            size_t& payload_len_out)
{
    // The "inner" part of the frame: [type:1][len:1][payload:N][CRC:2]
    const size_t inner_size = 1 + 1 + expected_payload_len + 2;
    const size_t frame_size = 4 + inner_size;  // with SOF
    // Read extra bytes to account for up to 20 bytes of misalignment
    const size_t read_size  = frame_size + 20;

    for (int attempt = 0; attempt < 3; attempt++)
    {
        payload_len_out = 0;
        uint8_t rx_buf[MAX_FRAME] = {};
        size_t actual_read = (read_size <= sizeof(rx_buf)) ? read_size : sizeof(rx_buf);

        if (attempt == 0 && cfg_read_cache_len > 0)
        {
            // First attempt: use bytes cached from the combined read that
            // captured both the OUT_STATUS_RESPONSE and the config frame
            // in a single I2C transaction.
            const size_t copy_len = (cfg_read_cache_len < sizeof(rx_buf))
                                        ? cfg_read_cache_len : sizeof(rx_buf);
            memcpy(rx_buf, cfg_read_cache, copy_len);
            actual_read = copy_len;
            cfg_read_cache_len = 0;  // consumed
        }
        else
        {
            // Subsequent attempts: fall back to direct I2C read.
            esp_err_t err = i2c_interface.masterRead(rx_buf, actual_read, 100);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "[CFG READ] attempt %d: i2c err=0x%X (%s)",
                              attempt, (unsigned)err, esp_err_to_name(err));
                delay(5 * (attempt + 1));
                continue;
            }
        }

        // Log raw header bytes for debugging
        ESP_LOGI(TAG, "[CFG READ] attempt %d: raw[0..9]=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X (read=%u)%s",
                      attempt,
                      rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
                      rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7],
                      rx_buf[8], rx_buf[9],
                      (unsigned)actual_read,
                      (attempt == 0) ? " [cached]" : "");

        // Strategy 1: Try full-SOF scan (AA 55 AA 55 + type) — ideal case
        for (size_t off = 0; off + frame_size <= actual_read; off++)
        {
            if (rx_buf[off]     == 0xAA &&
                rx_buf[off + 1] == 0x55 &&
                rx_buf[off + 2] == 0xAA &&
                rx_buf[off + 3] == 0x55 &&
                rx_buf[off + 4] == expected_type)
            {
                uint8_t type = 0;
                if (TR_I2C_Interface::unpackMessage(rx_buf + off, frame_size,
                                                      type, payload_out, payload_capacity,
                                                      payload_len_out, true))
                {
                    if (off > 0)
                    {
                        ESP_LOGI(TAG, "[CFG READ] found at SOF offset %u", (unsigned)off);
                    }
                    return true;
                }
            }
        }

        // Strategy 2: Scan for [type][len] signature and validate with CRC.
        // This handles the ESP32 compound-transaction 1-byte-eat bug where
        // the first SOF byte (0xAA) is consumed before we can read it.
        const uint8_t expected_len_byte = static_cast<uint8_t>(expected_payload_len);
        for (size_t off = 0; off + inner_size <= actual_read; off++)
        {
            if (rx_buf[off] == expected_type && rx_buf[off + 1] == expected_len_byte)
            {
                // Reconstruct a full frame with proper SOF for unpack validation
                uint8_t synth[MAX_FRAME];
                synth[0] = 0xAA;
                synth[1] = 0x55;
                synth[2] = 0xAA;
                synth[3] = 0x55;
                memcpy(synth + 4, rx_buf + off, inner_size);

                uint8_t type = 0;
                if (TR_I2C_Interface::unpackMessage(synth, frame_size,
                                                      type, payload_out, payload_capacity,
                                                      payload_len_out, true))
                {
                    ESP_LOGI(TAG, "[CFG READ] found via type+CRC scan at byte %u (SOF was lost)",
                                  (unsigned)off);
                    return true;
                }
            }
        }

        ESP_LOGW(TAG, "[CFG READ] attempt %d: config frame (type=0x%02X) not found in %u bytes",
                      attempt, (unsigned)expected_type, (unsigned)actual_read);
        delay(5 * (attempt + 1));
    }

    payload_len_out = 0;
    return false;
}

// ── Pyro channel helpers ─────────────────────────────────────────────────────

static void initPyroPins()
{
    // ARM and FIRE pins: output, start LOW (safe)
    // gpio_reset_pin() forces IO MUX back to GPIO function — required on
    // ESP32-P4 because SPI2 default pins (14-16) overlap with pyro pins,
    // and spi_bus_initialize() may claim them at the IO MUX level.
    for (auto pin : {(gpio_num_t)config::PYRO1_ARM_PIN,
                     (gpio_num_t)config::PYRO1_FIRE_PIN,
                     (gpio_num_t)config::PYRO2_ARM_PIN,
                     (gpio_num_t)config::PYRO2_FIRE_PIN}) {
        gpio_reset_pin(pin);
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << pin;
        cfg.mode         = GPIO_MODE_OUTPUT;
        cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&cfg);
        gpio_set_level(pin, 0);
    }
    // CONTINUITY pins: input (external 10k pullup on PCB)
    for (auto pin : {(gpio_num_t)config::PYRO1_CONT_PIN,
                     (gpio_num_t)config::PYRO2_CONT_PIN}) {
        gpio_reset_pin(pin);
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << pin;
        cfg.mode         = GPIO_MODE_INPUT;
        cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&cfg);
    }
    ESP_LOGI(TAG, "[PYRO] Pins initialized (safe)");
}

static void pyroSafeAll()
{
    portENTER_CRITICAL(&pyro_spinlock);
    gpio_set_level((gpio_num_t)config::PYRO1_FIRE_PIN, 0);
    gpio_set_level((gpio_num_t)config::PYRO1_ARM_PIN, 0);
    gpio_set_level((gpio_num_t)config::PYRO2_FIRE_PIN, 0);
    gpio_set_level((gpio_num_t)config::PYRO2_ARM_PIN, 0);
    pyro1_armed = false;
    pyro2_armed = false;
    portEXIT_CRITICAL(&pyro_spinlock);
    ESP_LOGI(TAG, "[PYRO] All channels safed");
}

static void pyroArmEnabled()
{
    if (pyro_config.ch1_enabled) {
        portENTER_CRITICAL(&pyro_spinlock);
        gpio_set_level((gpio_num_t)config::PYRO1_ARM_PIN, 1);
        pyro1_armed = true;
        portEXIT_CRITICAL(&pyro_spinlock);
        // Verify ARM pin actually went HIGH and read continuity
        delay(10);  // 10ms settle for VN5E160 power-up
        int arm_rb = gpio_get_level((gpio_num_t)config::PYRO1_ARM_PIN);
        int cont_raw = gpio_get_level((gpio_num_t)config::PYRO1_CONT_PIN);
        ESP_LOGI(TAG, "[PYRO] CH1 armed: ARM_PIN=%d rb=%d  CONT_PIN=%d raw=%d  mode=%u val=%.1f",
                 config::PYRO1_ARM_PIN, arm_rb,
                 config::PYRO1_CONT_PIN, cont_raw,
                 pyro_config.ch1_trigger_mode, (double)pyro_config.ch1_trigger_value);
        // Re-read CONT 5 more times to check for transient
        for (int i = 0; i < 5; i++) {
            delay(10);
            ESP_LOGI(TAG, "[PYRO] CH1 CONT re-read[%d]: %d", i,
                     gpio_get_level((gpio_num_t)config::PYRO1_CONT_PIN));
        }
    }
    if (pyro_config.ch2_enabled) {
        portENTER_CRITICAL(&pyro_spinlock);
        gpio_set_level((gpio_num_t)config::PYRO2_ARM_PIN, 1);
        pyro2_armed = true;
        portEXIT_CRITICAL(&pyro_spinlock);
        delayMicroseconds(500);
        int cont_raw = gpio_get_level((gpio_num_t)config::PYRO2_CONT_PIN);
        ESP_LOGI(TAG, "[PYRO] CH2 armed (mode=%u val=%.1f) CONT_PIN=%d raw=%d",
                 pyro_config.ch2_trigger_mode, (double)pyro_config.ch2_trigger_value,
                 config::PYRO2_CONT_PIN, cont_raw);
    }
}

static void servicePyroChannels(uint32_t now_ms)
{
    // Detect apogee (N-1 of N voting in TR_KinematicChecks)
    if (!pyro_apogee_detected && kinematics.apogee_flag) {
        pyro_apogee_detected = true;
        pyro_apogee_time_ms = now_ms;
        ESP_LOGI(TAG, "[PYRO] Apogee detected at t=%lu ms (vel=%d baro=%d gps=%d pitch=%d)",
                 (unsigned long)now_ms,
                 kinematics.vel_u_apogee_flag,
                 kinematics.alt_apogee_flag,
                 kinematics.gps_apogee_flag,
                 kinematics.pitch_apogee_flag);
    }

    portENTER_CRITICAL(&pyro_spinlock);

    // --- Channel 1 fire logic ---
    if (pyro1_armed && !pyro1_fired && pyro_config.ch1_enabled) {
        bool should_fire = false;
        if (pyro_config.ch1_trigger_mode == PYRO_TRIGGER_TIME_AFTER_APOGEE
            && pyro_apogee_detected) {
            float elapsed_s = (float)(now_ms - pyro_apogee_time_ms) / 1000.0f;
            if (elapsed_s >= pyro_config.ch1_trigger_value) should_fire = true;
        } else if (pyro_config.ch1_trigger_mode == PYRO_TRIGGER_ALTITUDE_ON_DESCENT
                   && pyro_apogee_detected) {
            if (pressure_alt_m <= pyro_config.ch1_trigger_value
                && pressure_alt_rate_mps < 0.0f) should_fire = true;
        }
        if (should_fire) {
            gpio_set_level((gpio_num_t)config::PYRO1_FIRE_PIN, 1);
            pyro1_fire_start_ms = now_ms;
            pyro1_fired = true;
        }
    }
    // Channel 1 fire pulse timeout
    bool ch1_pulse_done = false;
    if (pyro1_fired && pyro1_fire_start_ms > 0 &&
        (now_ms - pyro1_fire_start_ms) >= config::PYRO_FIRE_DURATION_MS) {
        gpio_set_level((gpio_num_t)config::PYRO1_FIRE_PIN, 0);
        pyro1_fire_start_ms = 0;
        ch1_pulse_done = true;
    }

    // --- Channel 2 fire logic ---
    if (pyro2_armed && !pyro2_fired && pyro_config.ch2_enabled) {
        bool should_fire = false;
        if (pyro_config.ch2_trigger_mode == PYRO_TRIGGER_TIME_AFTER_APOGEE
            && pyro_apogee_detected) {
            float elapsed_s = (float)(now_ms - pyro_apogee_time_ms) / 1000.0f;
            if (elapsed_s >= pyro_config.ch2_trigger_value) should_fire = true;
        } else if (pyro_config.ch2_trigger_mode == PYRO_TRIGGER_ALTITUDE_ON_DESCENT
                   && pyro_apogee_detected) {
            if (pressure_alt_m <= pyro_config.ch2_trigger_value
                && pressure_alt_rate_mps < 0.0f) should_fire = true;
        }
        if (should_fire) {
            gpio_set_level((gpio_num_t)config::PYRO2_FIRE_PIN, 1);
            pyro2_fire_start_ms = now_ms;
            pyro2_fired = true;
        }
    }
    // Channel 2 fire pulse timeout
    bool ch2_pulse_done = false;
    if (pyro2_fired && pyro2_fire_start_ms > 0 &&
        (now_ms - pyro2_fire_start_ms) >= config::PYRO_FIRE_DURATION_MS) {
        gpio_set_level((gpio_num_t)config::PYRO2_FIRE_PIN, 0);
        pyro2_fire_start_ms = 0;
        ch2_pulse_done = true;
    }

    // Snapshot fire events for logging outside critical section
    bool ch1_just_fired = pyro1_fired && pyro1_fire_start_ms == now_ms;
    bool ch2_just_fired = pyro2_fired && pyro2_fire_start_ms == now_ms;

    portEXIT_CRITICAL(&pyro_spinlock);

    // Log outside critical section (ESP_LOG can block)
    if (ch1_just_fired) ESP_LOGW(TAG, "[PYRO] CH1 FIRED at alt=%.1f m", (double)pressure_alt_m);
    if (ch2_just_fired) ESP_LOGW(TAG, "[PYRO] CH2 FIRED at alt=%.1f m", (double)pressure_alt_m);
    if (ch1_pulse_done) ESP_LOGI(TAG, "[PYRO] CH1 fire pulse complete");
    if (ch2_pulse_done) ESP_LOGI(TAG, "[PYRO] CH2 fire pulse complete");
}

// ── Roll profile interpolation ────────────────────────────────────────────────
// Linearly interpolates between waypoints using time since launch.
// Before the first waypoint → hold first angle.
// After the last waypoint → hold last angle.
// If no waypoints (rate-only mode) → return 0 (setpoint passed to rate PID).
static float roll_profile_interpolate(float t_flight_s)
{
    if (roll_profile.num_waypoints == 0)
    {
        return config::ROLL_RATE_SET_POINT;  // no profile → default setpoint (typically 0)
    }
    const uint8_t n = roll_profile.num_waypoints;

    // Before first waypoint: hold first angle
    if (t_flight_s <= roll_profile.waypoints[0].time_s)
    {
        return roll_profile.waypoints[0].angle_deg;
    }
    // After last waypoint: hold last angle
    if (t_flight_s >= roll_profile.waypoints[n - 1].time_s)
    {
        return roll_profile.waypoints[n - 1].angle_deg;
    }
    // Linear interpolation between surrounding waypoints
    for (uint8_t i = 0; i < n - 1; ++i)
    {
        if (t_flight_s < roll_profile.waypoints[i + 1].time_s)
        {
            float t0 = roll_profile.waypoints[i].time_s;
            float t1 = roll_profile.waypoints[i + 1].time_s;
            float a0 = roll_profile.waypoints[i].angle_deg;
            float a1 = roll_profile.waypoints[i + 1].angle_deg;
            if (t1 <= t0) return a0;  // guard against duplicate timestamps
            float frac = (t_flight_s - t0) / (t1 - t0);
            return a0 + frac * (a1 - a0);
        }
    }
    return roll_profile.waypoints[n - 1].angle_deg;
}

static inline void i2sSendWithStats(uint8_t type, const uint8_t *payload, size_t len)
{
    const esp_err_t err = i2s_stream.writeFrame(type, payload, len);
    if (err == ESP_OK)
    {
        i2s_tx_ok++;
        switch (type)
        {
            case ISM6HG256_MSG: i2s_tx_ism6_ok++; break;
            case BMP585_MSG: i2s_tx_bmp_ok++; break;
            case MMC5983MA_MSG: i2s_tx_mmc_ok++; break;
            case IIS2MDC_MSG: i2s_tx_iis2mdc_ok++; break;
            case GNSS_MSG: i2s_tx_gnss_ok++; break;
            case NON_SENSOR_MSG: i2s_tx_ns_ok++; break;
            default: break;
        }
    }
    else
    {
        i2s_tx_fail++;
        i2s_last_tx_err = (int)err;
        switch (type)
        {
            case ISM6HG256_MSG: i2s_tx_ism6_fail++; break;
            case BMP585_MSG: i2s_tx_bmp_fail++; break;
            case MMC5983MA_MSG: i2s_tx_mmc_fail++; break;
            case IIS2MDC_MSG: i2s_tx_iis2mdc_fail++; break;
            case GNSS_MSG: i2s_tx_gnss_fail++; break;
            case NON_SENSOR_MSG: i2s_tx_ns_fail++; break;
            default: break;
        }
    }
}

static inline bool enqueueI2STx(uint8_t type, const uint8_t *payload, size_t len)
{
    if (i2s_tx_queue == nullptr || len > MAX_PAYLOAD || len > 0xFF)
    {
        i2s_tx_enqueue_drop++;
        return false;
    }

    I2STxMessage msg = {};
    msg.type = type;
    msg.len = (uint8_t)len;
    if (len > 0 && payload != nullptr)
    {
        memcpy(msg.payload, payload, len);
    }

    if (xQueueSend(i2s_tx_queue, &msg, 0) == pdTRUE)
    {
        i2s_tx_enqueue_ok++;
        return true;
    }

    i2s_tx_enqueue_drop++;
    return false;
}

static SemaphoreHandle_t i2c_bus_mutex = nullptr;

// I2S sender task — dequeues telemetry and writes to I2S DMA.
// When no frame is available, writes idle fill (zeros) to prevent the
// I2S DMA from replaying stale data.  The OC's parser skips zero runs
// and only processes SOF-framed data.
//
// IMPORTANT: The I2S clock runs continuously at config::I2S_SAMPLE_RATE * 4
// bytes/sec.  Any gap in writes causes the DMA to replay old buffers,
// which the OC sees as duplicate frames.  Continuous zero fill eliminates
// this by ensuring stale DMA slots contain only zeros.
static void i2sSenderTask(void *)
{
    I2STxMessage msg = {};
    for (;;)
    {
        // Try to dequeue with a short timeout (1ms).
        // If a frame is ready, send it immediately.
        // If not, write idle fill zeros to keep the DMA pipe clean.
        if (xQueueReceive(i2s_tx_queue, &msg, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            i2sSendWithStats(msg.type, msg.payload, msg.len);
        }
        else
        {
            // No frame available — write zeros to fill the I2S DMA pipe.
            // This prevents stale data replay when the slave reads.
            i2s_stream.writeIdleFill(64, 1);
        }
    }
}

static inline void triggerBlueLedFlash(uint32_t now_ms)
{
    digitalWrite(config::BLUE_LED_PIN, HIGH);
    blue_led_flash_active = true;
    blue_led_flash_end_ms = now_ms + (uint32_t)config::BLUE_LED_FLASH_MS;
}

static inline void serviceBlueLedFlash(uint32_t now_ms)
{
    if (!blue_led_flash_active)
    {
        return;
    }
    if ((int32_t)(now_ms - blue_led_flash_end_ms) >= 0)
    {
        digitalWrite(config::BLUE_LED_PIN, LOW);
        blue_led_flash_active = false;
    }
}

static void piezoToggleCb(void *)
{
    if (!piezo_wave_active)
    {
        return;
    }
    const int64_t now_us = esp_timer_get_time();
    if (now_us >= piezo_wave_end_us)
    {
        piezo_wave_active = false;
        piezo_pin_high = false;
        gpio_set_level((gpio_num_t)config::PIEZO_PIN, 0);
        if (piezo_toggle_timer != nullptr)
        {
            (void)esp_timer_stop(piezo_toggle_timer);
        }
        return;
    }

    piezo_pin_high = !piezo_pin_high;
    gpio_set_level((gpio_num_t)config::PIEZO_PIN, piezo_pin_high ? 1 : 0);
}

static bool initPiezoTimer()
{
    if (piezo_toggle_timer != nullptr)
    {
        return true;
    }
    esp_timer_create_args_t args = {};
    args.callback = &piezoToggleCb;
    args.arg = nullptr;
    args.name = "piezo";
    return esp_timer_create(&args, &piezo_toggle_timer) == ESP_OK;
}

static inline void piezoStop()
{
    piezo_wave_active = false;
    piezo_pin_high = false;
    if (piezo_toggle_timer != nullptr)
    {
        (void)esp_timer_stop(piezo_toggle_timer);
    }
    if (piezo_pwm_ready)
    {
        digitalWrite(config::PIEZO_PIN, LOW);
    }
}

static inline void piezoStart(uint32_t freq_hz, uint32_t duration_ms, uint32_t now_us)
{
    if (!piezo_pwm_ready || piezo_toggle_timer == nullptr || freq_hz == 0U || duration_ms == 0U)
    {
        return;
    }

    piezo_half_period_us = 500000UL / freq_hz;
    if (piezo_half_period_us < 50U)
    {
        piezo_half_period_us = 50U;
    }
    piezo_wave_end_us = (int64_t)now_us + ((int64_t)duration_ms * 1000LL);
    piezo_pin_high = false;
    piezo_wave_active = true;
    digitalWrite(config::PIEZO_PIN, LOW);
    triggerBlueLedFlash((uint32_t)(now_us / 1000ULL));

    (void)esp_timer_stop(piezo_toggle_timer);
    if (esp_timer_start_periodic(piezo_toggle_timer, piezo_half_period_us) != ESP_OK)
    {
        piezoStop();
    }
}

// ── GoPro control (GPIO pulse on shutter pin) ──
static inline void startGoProPulse(uint32_t now_ms)
{
    if ((config::CAM_SHUTTER_PIN < 0) || !config::USE_GOPRO)
        return;
    digitalWrite(config::CAM_SHUTTER_PIN, LOW);
    gopro_pulse_active = true;
    gopro_pulse_end_ms = now_ms + (uint32_t)config::GOPRO_PULSE_MS;
}

static inline void serviceGoProPulse(uint32_t now_ms)
{
    if (!gopro_pulse_active)
        return;
    if ((int32_t)(now_ms - gopro_pulse_end_ms) >= 0)
    {
        digitalWrite(config::CAM_SHUTTER_PIN, HIGH);
        gopro_pulse_active = false;
    }
}

// ── RunCam control (UART command to toggle recording) ──
// RunCam Split 4 power-button message toggles recording on/off.
static const uint8_t RUNCAM_PWR_CMD[] = {0xCC, 0x01, 0x01, 0xE7};
static constexpr uart_port_t RUNCAM_UART_PORT = UART_NUM_2;
static bool runcam_uart_ready = false;

static void initRunCam()
{
    if (!config::USE_RUNCAM)
        return;

    // Configure pins but keep camera powered OFF at boot.
    // Camera is powered on only when the user presses "Start Recording".
    if (config::RUNCAM_PWR_PIN >= 0)
    {
        pinMode(config::RUNCAM_PWR_PIN, OUTPUT);
        digitalWrite(config::RUNCAM_PWR_PIN, LOW);  // OFF at boot
        ESP_LOGI(TAG, "RunCam pin %d configured (camera OFF)", (int)config::RUNCAM_PWR_PIN);
    }

    // Init UART to RunCam via ESP-IDF driver (ready for when camera powers on)
    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate  = (int)config::RUNCAM_BAUD;
    uart_cfg.data_bits  = UART_DATA_8_BITS;
    uart_cfg.parity     = UART_PARITY_DISABLE;
    uart_cfg.stop_bits  = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_param_config(RUNCAM_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(RUNCAM_UART_PORT,
                                 (int)config::RUNCAM_TX_PIN,
                                 (int)config::RUNCAM_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(RUNCAM_UART_PORT, 256, 256, 0, NULL, 0));

    runcam_uart_ready = true;
    ESP_LOGI(TAG, "RunCam UART on RX=%d TX=%d @ %lu baud",
             (int)config::RUNCAM_RX_PIN, (int)config::RUNCAM_TX_PIN,
             (unsigned long)config::RUNCAM_BAUD);
}

static void sendRunCamToggle()
{
    if (!config::USE_RUNCAM || !runcam_uart_ready)
        return;
    uart_write_bytes(RUNCAM_UART_PORT, RUNCAM_PWR_CMD, sizeof(RUNCAM_PWR_CMD));
    ESP_LOGI(TAG, "RunCam toggle sent");
}

// ── Generic camera start/stop (dispatches based on runtime_camera_type) ──
static void cameraStart(uint32_t now_ms)
{
    if (gopro_recording) return;  // already recording
    if (runtime_camera_type == CAM_TYPE_GOPRO)
    {
        startGoProPulse(now_ms);
        gopro_recording = true;
        ESP_LOGI(TAG, "Camera START (GoPro)");
    }
    else if (runtime_camera_type == CAM_TYPE_RUNCAM)
    {
        // Power on → wait for boot → RunCam auto-starts recording
        if (config::RUNCAM_PWR_PIN >= 0)
            digitalWrite(config::RUNCAM_PWR_PIN, HIGH);
        ESP_LOGI(TAG, "RunCam power ON, waiting for boot...");
        delay(5000);  // RunCam needs ~5s to boot and auto-start recording
        gopro_recording = true;
        ESP_LOGI(TAG, "Camera START (RunCam) — recording");
    }
}

static void cameraStop(uint32_t now_ms)
{
    if (!gopro_recording) return;  // not recording
    if (runtime_camera_type == CAM_TYPE_GOPRO)
    {
        startGoProPulse(now_ms);
        ESP_LOGI(TAG, "Camera STOP (GoPro)");
    }
    else if (runtime_camera_type == CAM_TYPE_RUNCAM)
    {
        // Send toggle to stop recording, then power off
        sendRunCamToggle();
        delay(500);  // let the stop command process
        if (config::RUNCAM_PWR_PIN >= 0)
            digitalWrite(config::RUNCAM_PWR_PIN, LOW);
        ESP_LOGI(TAG, "Camera STOP (RunCam) — powered off");
    }
    gopro_recording = false;
}

static inline void startBootReadyChirp(uint32_t now_ms, uint32_t now_us)
{
    if (!enable_sounds || !piezo_pwm_ready)
    {
        return;
    }
    piezoStart(2600, 90, now_us);
    boot_chirp_phase = BootChirpPhase::GapAfterBeep1;
    boot_chirp_next_ms = now_ms + 150U; // 90ms beep + 60ms gap
}

static inline void serviceBootReadyChirp(uint32_t now_ms, uint32_t now_us)
{
    switch (boot_chirp_phase)
    {
        case BootChirpPhase::Idle:
            return;
        case BootChirpPhase::GapAfterBeep1:
            if ((int32_t)(now_ms - boot_chirp_next_ms) >= 0)
            {
                piezoStart(1900, 130, now_us);
                boot_chirp_phase = BootChirpPhase::WaitingBeep2End;
                boot_chirp_next_ms = now_ms + 130U;
            }
            return;
        case BootChirpPhase::WaitingBeep2End:
            if ((int32_t)(now_ms - boot_chirp_next_ms) >= 0)
            {
                boot_chirp_phase = BootChirpPhase::Idle;
            }
            return;
    }
}

static inline void serviceHeartbeatBeep(uint32_t now_ms)
{
    if (!config::HEARTBEAT_BEEP_IN_FLIGHT &&
        (rocket_state == INFLIGHT || rocket_state == LANDED))
    {
        return;
    }
    if ((now_ms - last_heartbeat_beep_ms) < config::HEARTBEAT_BEEP_INTERVAL_MS)
    {
        return;
    }
    last_heartbeat_beep_ms = now_ms;

    // Keep heartbeat LED indication independent from sound enable.
    triggerBlueLedFlash(now_ms);

    if (!enable_sounds || !piezo_pwm_ready)
    {
        return;
    }
    if (boot_chirp_phase != BootChirpPhase::Idle || piezo_wave_active)
    {
        return;
    }
    piezoStart(config::HEARTBEAT_BEEP_FREQ_HZ,
               config::HEARTBEAT_BEEP_DURATION_MS,
               micros());
}

static void setup_fc()
{
    // Ensure NVS is initialised (ESP-IDF on ESP32-P4 may not auto-init)
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }

    ESP_LOGI(TAG, "NVS init: %s", esp_err_to_name(nvs_err));

    // The flight task runs continuously and starves IDLE tasks on CPU 1.
    // Reconfigure WDT to not monitor IDLE cores (the flight task itself
    // never hangs — it completes each iteration in ~1 ms).
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = 5000,
        .idle_core_mask = 0,       // don't monitor any IDLE tasks
        .trigger_panic = false,
    };
    esp_task_wdt_reconfigure(&wdt_cfg);

    // Pyro channels: safe pins FIRST, before any other peripheral init
    initPyroPins();

    ESP_LOGI(TAG, "Starting ....");
    pinMode(config::RED_LED_PIN, OUTPUT);
    digitalWrite(config::RED_LED_PIN, HIGH);
    pinMode(config::BLUE_LED_PIN, OUTPUT);
    digitalWrite(config::BLUE_LED_PIN, LOW);

    // Disable all SPI chip-selects before bus init to avoid MISO contention.
    pinMode(config::MMC5983MA_CS, OUTPUT); digitalWrite(config::MMC5983MA_CS, HIGH);
    pinMode(config::BMP585_CS, OUTPUT);    digitalWrite(config::BMP585_CS, HIGH);
    pinMode(config::ISM6HG256_CS, OUTPUT); digitalWrite(config::ISM6HG256_CS, HIGH);
    delay(10);

    // SPI bus — compat shim wraps spi_bus_initialize(SPI2_HOST, ...)
    ESP_LOGI(TAG, "SPI init...");
    SPI.begin(config::SPI_SCK, config::SPI_SDO, config::SPI_SDI);
    delay(10);

    // Re-init pyro pins AFTER spi_bus_initialize() — on ESP32-P4, SPI2
    // default pins (14-16) overlap with pyro pins, and the bus init may
    // reclaim them even when custom pins are specified.
    initPyroPins();

    // Initialize I2C interface to Out ESP32
    // Board has external pull-ups — suppress the new driver's pull-up warning
    esp_log_level_set("i2c.master", ESP_LOG_ERROR);
    ESP_LOGI(TAG, "I2C init...");
    if (i2c_interface.beginMaster(config::ESP_SDA_PIN,
                                  config::ESP_SCL_PIN,
                                  config::ESP_I2C_FREQ_HZ,
                                  false) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize ESP I2C interface");
        while (1) { delay(1000); }
    }
    ESP_LOGI(TAG, "I2C master addr=0x%02X SDA=%d SCL=%d clk=%lu",
                  (unsigned)config::ESP_I2C_ADR,
                  (int)config::ESP_SDA_PIN,
                  (int)config::ESP_SCL_PIN,
                  (unsigned long)config::ESP_I2C_FREQ_HZ);

    // I2S telemetry stream to OutComputer (DMA-based, unidirectional)
    if (i2s_stream.beginMasterTx(config::I2S_BCLK_PIN,
                                  config::I2S_WS_PIN,
                                  config::I2S_DOUT_PIN,
                                  config::I2S_FSYNC_PIN,
                                  config::I2S_SAMPLE_RATE) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize I2S TX stream");
        while (1) { delay(1000); }
    }
    i2s_tx_queue = xQueueCreate(config::I2S_TX_QUEUE_LEN, sizeof(I2STxMessage));
    if (i2s_tx_queue == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create I2S TX queue");
        while (1) { delay(1000); }
    }
    i2c_bus_mutex = xSemaphoreCreateMutex();
    if (i2c_bus_mutex == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create I2C bus mutex");
        while (1) { delay(1000); }
    }
    // I2S sender runs on the SAME core as sensor collection (Core 0).
    // Priority 2: below pollIMUdata (4) so sensor SPI reads are never
    // delayed, but above pollGNSSdata (1).  This frees Core 1 entirely
    // for flight task (flight logic + EKF) — eliminating the CPU contention
    // that caused rate regressions when the sender preempted flight task at
    // priority 3, or queue overflow at priority 1 (round-robin starvation).
    // Core 0 budget: ~20% sensor SPI + ~53% I2S send + ~3% GNSS ≈ 76%.
    const uint8_t i2s_sender_core = config::SENSOR_CORE;  // Core 0
    xTaskCreatePinnedToCore(i2sSenderTask,
                            "I2S Sender",
                            4096,
                            nullptr,
                            2,
                            &i2s_sender_task_handle,
                            i2s_sender_core);

    // Load persistent rocket settings from NVS (factory default from config.h)
    ESP_LOGI(TAG, "NVS prefs loading...");
    prefs.begin("rocket", false);  // read-write (creates namespace on first boot)
    enable_sounds = prefs.getBool("sounds", config::ENABLE_SOUNDS);
    servo_enabled = prefs.getBool("servo_en", config::USE_SERVO_CONTROL);
    prefs.end();
    ESP_LOGI(TAG, "NVS: enable_sounds=%s servo_enabled=%s",
                  enable_sounds ? "true" : "false",
                  servo_enabled ? "true" : "false");

    // Load servo/PID NVS settings (namespace "servo")
    int16_t nvs_servo_hz  = config::SERVO_HZ;
    int16_t nvs_servo_min = config::SERVO_MIN_US;
    int16_t nvs_servo_max = config::SERVO_MAX_US;
    bool nvs_servo_timing_changed = false;

    prefs.begin("servo", false);  // read-write (creates namespace on first boot)
    if (prefs.isKey("b1"))
    {
        servo_control.setBias(0, prefs.getShort("b1", config::SERVO_BIAS_1));
        servo_control.setBias(1, prefs.getShort("b2", config::SERVO_BIAS_2));
        servo_control.setBias(2, prefs.getShort("b3", config::SERVO_BIAS_3));
        servo_control.setBias(3, prefs.getShort("b4", config::SERVO_BIAS_4));
        nvs_servo_hz  = prefs.getShort("hz",  config::SERVO_HZ);
        nvs_servo_min = prefs.getShort("min", config::SERVO_MIN_US);
        nvs_servo_max = prefs.getShort("max", config::SERVO_MAX_US);
        nvs_servo_timing_changed = (nvs_servo_hz  != config::SERVO_HZ ||
                                     nvs_servo_min != config::SERVO_MIN_US ||
                                     nvs_servo_max != config::SERVO_MAX_US);
        ESP_LOGI(TAG, "NVS servo: bias=[%d,%d,%d,%d] hz=%d min=%d max=%d",
                      prefs.getShort("b1", 0), prefs.getShort("b2", 0),
                      prefs.getShort("b3", 0), prefs.getShort("b4", 0),
                      nvs_servo_hz, nvs_servo_min, nvs_servo_max);
    }
    if (prefs.isKey("kp"))
    {
        float kp = prefs.getFloat("kp", config::KP);
        float ki = prefs.getFloat("ki", config::KI);
        float kd = prefs.getFloat("kd", config::KD);
        float mincmd = prefs.getFloat("mincmd", config::MIN_CMD);
        float maxcmd = prefs.getFloat("maxcmd", config::MAX_CMD);
        servo_control.setPIDGains(kp, ki, kd);
        servo_control.setPIDLimits(mincmd, maxcmd);
        ESP_LOGI(TAG, "NVS PID: kp=%.4f ki=%.4f kd=%.4f min=%.1f max=%.1f",
                      (double)kp, (double)ki, (double)kd,
                      (double)mincmd, (double)maxcmd);
    }
    gain_sched_enabled = prefs.getBool("gs", config::GAIN_SCHEDULE_ENABLED);
    use_angle_control  = prefs.getBool("ac", config::USE_ANGLE_CONTROL);
    roll_delay_ms      = prefs.getUShort("rdly", config::ROLL_CONTROL_DELAY_MS);
    guidance_enabled   = prefs.getBool("guid_en", config::GUIDANCE_ENABLED);
    prefs.end();
    ESP_LOGI(TAG, "Gain scheduling: %s", gain_sched_enabled ? "ON" : "OFF");
    ESP_LOGI(TAG, "Angle control: %s  Roll delay: %u ms",
                  use_angle_control ? "ON" : "OFF", (unsigned)roll_delay_ms);
#if TR_GUIDANCE_AVAILABLE
    ESP_LOGI(TAG, "PN Guidance: %s (compiled in)", guidance_enabled ? "ON" : "OFF");
#else
    ESP_LOGW(TAG, "PN Guidance: NOT COMPILED IN (TR_GuidancePN submodule not initialized — stub active)");
#endif

    // Configure guidance and control mixer
    guidance.configure(config::PN_NAV_GAIN,
                       config::PN_MAX_ACCEL_MPS2,
                       config::PN_TARGET_ALT_M);
    control_mixer.configure(config::PN_PITCH_KP, config::PN_PITCH_KI, config::PN_PITCH_KD,
                            config::PN_YAW_KP,   config::PN_YAW_KI,   config::PN_YAW_KD,
                            config::PN_MAX_FIN_DEG,
                            config::GAIN_SCHEDULE_V_REF,
                            config::GAIN_SCHEDULE_V_MIN);
    if (gain_sched_enabled) {
        control_mixer.enableGainSchedule(config::GAIN_SCHEDULE_V_REF,
                                         config::GAIN_SCHEDULE_V_MIN);
    }

    // Restore high-g accelerometer bias from NVS (namespace "cal")
    prefs.begin("cal", false);  // read-write (creates namespace on first boot)
    if (prefs.isKey("hgbx"))
    {
        float bx = prefs.getFloat("hgbx", 0.0f);
        float by = prefs.getFloat("hgby", 0.0f);
        float bz = prefs.getFloat("hgbz", 0.0f);
        sensor_converter.setHighGBias(bx, by, bz);
        out_status_query_data.hg_bias_x_cmss = (int16_t)lroundf(bx * 100.0f);
        out_status_query_data.hg_bias_y_cmss = (int16_t)lroundf(by * 100.0f);
        out_status_query_data.hg_bias_z_cmss = (int16_t)lroundf(bz * 100.0f);
        ESP_LOGI(TAG, "NVS HG bias: %.3f, %.3f, %.3f m/s²",
                      (double)bx, (double)by, (double)bz);
    }
    prefs.end();

    // Load roll profile from NVS (namespace "rollp")
    prefs.begin("rollp", false);  // read-write (creates namespace on first boot)
    if (prefs.isKey("prof"))
    {
        size_t prof_len = prefs.getBytesLength("prof");
        if (prof_len == sizeof(RollProfileData))
        {
            prefs.getBytes("prof", &roll_profile, sizeof(RollProfileData));
            // Clamp num_waypoints to valid range
            if (roll_profile.num_waypoints > MAX_ROLL_WAYPOINTS)
            {
                roll_profile.num_waypoints = MAX_ROLL_WAYPOINTS;
            }
            ESP_LOGI(TAG, "NVS roll profile: %d waypoints", roll_profile.num_waypoints);
            for (uint8_t i = 0; i < roll_profile.num_waypoints; ++i)
            {
                ESP_LOGI(TAG, "  WP%d: t=%.1fs angle=%.1f°", i,
                              (double)roll_profile.waypoints[i].time_s,
                              (double)roll_profile.waypoints[i].angle_deg);
            }
        }
        else
        {
            ESP_LOGW(TAG, "NVS roll profile size mismatch (%u vs %u), ignoring",
                          (unsigned)prof_len, (unsigned)sizeof(RollProfileData));
        }
    }
    else
    {
        ESP_LOGI(TAG, "NVS roll profile: none (rate-only mode)");
    }
    prefs.end();

    // Load pyro config from NVS
    prefs.begin("pyro", true);
    size_t pyro_cfg_sz = prefs.getBytesLength("cfg");
    if (pyro_cfg_sz == sizeof(PyroConfigData)) {
        prefs.getBytes("cfg", &pyro_config, sizeof(pyro_config));
        ESP_LOGI(TAG, "NVS pyro: ch1_en=%u mode=%u val=%.1f  ch2_en=%u mode=%u val=%.1f",
                 pyro_config.ch1_enabled, pyro_config.ch1_trigger_mode,
                 (double)pyro_config.ch1_trigger_value,
                 pyro_config.ch2_enabled, pyro_config.ch2_trigger_mode,
                 (double)pyro_config.ch2_trigger_value);
    } else {
        ESP_LOGI(TAG, "NVS pyro: none (both disabled)");
    }
    prefs.end();

    // Always initialise piezo hardware so it's ready if enabled at runtime
    pinMode(config::PIEZO_PIN, OUTPUT);
    digitalWrite(config::PIEZO_PIN, LOW);
    piezo_pwm_ready = initPiezoTimer();
    if (!piezo_pwm_ready)
    {
        ESP_LOGE(TAG, "Piezo timer init failed; sounds disabled");
    }

    // Initialize sensor collector (including sensors) and start polling tasks
    ESP_LOGI(TAG, "Sensor collector init...");
    sensor_collector.begin(config::SENSOR_CORE);
    sensor_converter.configureISM6HG256FullScale(
        static_cast<ISM6LowGFullScale>(config::ISM6_LOW_G_FS_G),
        static_cast<ISM6HighGFullScale>(config::ISM6_HIGH_G_FS_G),
        static_cast<ISM6GyroFullScale>(config::ISM6_GYRO_FS_DPS));
    sensor_converter.configureISM6HG256RotationZ(config::ISM6HG256_ROT_Z_DEG);
    sensor_converter.configureMMC5983MARotationZ(config::MMC5983MA_ROT_Z_DEG);
    sensor_converter.configureIIS2MDCRotationZ(config::IIS2MDC_ROT_Z_DEG);
    sensor_collector.configureSimRotation(config::ISM6HG256_ROT_Z_DEG);

    out_status_query_data.ism6_low_g_fs_g = config::ISM6_LOW_G_FS_G;
    out_status_query_data.ism6_high_g_fs_g = config::ISM6_HIGH_G_FS_G;
    out_status_query_data.ism6_gyro_fs_dps = config::ISM6_GYRO_FS_DPS;
    out_status_query_data.ism6_rot_z_cdeg = (int16_t)lroundf(config::ISM6HG256_ROT_Z_DEG * 100.0f);
    out_status_query_data.mmc_rot_z_cdeg = (int16_t)lroundf(config::MMC5983MA_ROT_Z_DEG * 100.0f);
    out_status_query_data.format_version = 2;

    // NOTE: Do NOT drain the slave TX buffer here.  With the new
    // i2c_slave driver, reading when the slave has no TX data queued
    // causes an underflow crash.  The OC no longer pre-queues responses,
    // so the TX ringbuffer is empty until the first query triggers one.

    ESP_LOGI(TAG, "I2C initial OC query...");
    // Send the first query and read the 96-byte combined response,
    // matching the same protocol used in the main loop.
    (void)i2c_interface.sendMessage(OUT_STATUS_QUERY,
                                    reinterpret_cast<const uint8_t*>(&out_status_query_data),
                                    sizeof(out_status_query_data));
    delay(10); // let OutComputer process query and queue 96-byte response
    {
        uint8_t init_buf[COMBINED_READ_SIZE] = {};
        esp_err_t read_err = i2c_interface.masterRead(
            init_buf, COMBINED_READ_SIZE, 10);
        if (read_err == ESP_OK)
        {
            uint8_t resp_type = 0;
            uint8_t resp_payload[2] = {};
            size_t  resp_payload_len = 0;
            if (TR_I2C_Interface::unpackMessage(
                    init_buf, 10,
                    resp_type, resp_payload, sizeof(resp_payload),
                    resp_payload_len, true)
                && resp_type == OUT_STATUS_RESPONSE
                && resp_payload_len >= 1)
            {
                if (resp_payload[0] != 0) { out_ready = true; }
                if (resp_payload_len >= 2 && resp_payload[1] != 0)
                {
                    out_pending_command = resp_payload[1];
                }
                i2c_query_ok++;
            }
            else
            {
                i2c_query_fail++;
            }
        }
        else
        {
            i2c_query_fail++;
        }
    }
    if (out_ready)
    {
        ESP_LOGI(TAG, "OUT ESP32 ready");
    }
    else
    {
        ESP_LOGW(TAG, "OUT ESP32 not ready yet (will latch on first success)");
    }

    // Camera setup
    if (config::USE_GOPRO)
    {
        if (config::CAM_PWR_PIN >= 0)
        {
            pinMode(config::CAM_PWR_PIN, OUTPUT);
            digitalWrite(config::CAM_PWR_PIN, HIGH);
        }
        if (config::CAM_SHUTTER_PIN >= 0)
        {
            pinMode(config::CAM_SHUTTER_PIN, OUTPUT);
            digitalWrite(config::CAM_SHUTTER_PIN, HIGH);
        }
    }
    else if (config::USE_RUNCAM)
    {
        initRunCam();
    }

    ESP_LOGI(TAG, "Servo init...");
    // Derivative-term LPF for every flight-computer PID. Prevents sample-
    // to-sample gyro noise from being amplified by the D term (raw
    // backward-difference derivative) into visible servo flutter.
    //   - roll_rate_pid_standalone: roll null (ground test & boost)
    //   - servo_control's internal pid: INFLIGHT roll null
    //   - control_mixer's pitch/yaw rate PIDs: guidance
    roll_rate_pid_standalone.setDerivativeFilterCutoffHz(config::D_FILTER_CUTOFF_HZ);
    servo_control.setPIDDerivativeFilterCutoffHz(config::D_FILTER_CUTOFF_HZ);
    control_mixer.setDerivativeFilterCutoffHz(config::D_FILTER_CUTOFF_HZ);
    // Servo setup — always init hardware if pins valid, gate enabled on NVS.
    if (servoPinsValid())
    {
        servo_control.begin();
        if (nvs_servo_timing_changed)
        {
            servo_control.setServoTiming(nvs_servo_hz, nvs_servo_min, nvs_servo_max);
            ESP_LOGI(TAG, "Applied NVS servo timing: hz=%d min=%d max=%d",
                          nvs_servo_hz, nvs_servo_min, nvs_servo_max);
        }
        servo_control.setSetpoint(config::ROLL_RATE_SET_POINT);
        if (gain_sched_enabled)
        {
            servo_control.enableGainSchedule(config::GAIN_SCHEDULE_V_REF, config::GAIN_SCHEDULE_V_MIN);
        }
        if (servo_enabled && config::SERVO_WIGGLE_ON_BOOT)
        {
            servo_control.wiggle();
        }
        ESP_LOGI(TAG, "Servo control %s, gain schedule %s (hardware ready)",
                      servo_enabled ? "enabled" : "disabled",
                      gain_sched_enabled ? "ON" : "OFF");
    }
    else
    {
        servo_enabled = false;
        ESP_LOGW(TAG, "Servo control disabled (set SERVO_PIN_* in config.h)");
    }

    // ── Inflight reboot recovery ────────────────────────────────────────────
    // If the reset was unexpected (brownout, watchdog, panic), query the OC
    // for the latest snapshot it received over I2S and restore state to
    // resume INFLIGHT ops.  Snapshot lives in OC's MRAM (non-volatile, no
    // FC flash writes — see #104).
    {
        esp_reset_reason_t rst = esp_reset_reason();
        ESP_LOGI(TAG, "Reset reason: %s (%d)", resetReasonStr(rst), (int)rst);

        bool unexpected_reset = (rst == ESP_RST_BROWNOUT ||
                                 rst == ESP_RST_PANIC    ||
                                 rst == ESP_RST_INT_WDT  ||
                                 rst == ESP_RST_TASK_WDT ||
                                 rst == ESP_RST_WDT);

        FlightSnapshotData snap = {};
        bool valid = false;

        if (unexpected_reset && !out_ready) {
            ESP_LOGW(TAG, "[RECOVERY] OC not ready — skipping snapshot recovery");
        }
        else if (unexpected_reset) {
            // Send GET_FLIGHT_SNAPSHOT request, then read the SNAPSHOT_MSG
            // response.  OC reads from MRAM (~150 us) and queues the
            // ~224-byte response into its I2C TX ringbuffer (256 B).
            if (i2c_interface.sendMessage(GET_FLIGHT_SNAPSHOT, nullptr, 0) == ESP_OK) {
                delay(20);  // let OC service the request and queue the response

                constexpr size_t kRespFrameLen = 4 + 1 + 1 + sizeof(FlightSnapshotData) + 2;
                uint8_t buf[kRespFrameLen + 16] = {};  // + slack for SOF byte loss
                esp_err_t rerr = i2c_interface.masterRead(buf, sizeof(buf), 100);
                if (rerr == ESP_OK) {
                    // SOF scan — same defensive pattern as the [CFG READ] path.
                    for (size_t off = 0; off + kRespFrameLen <= sizeof(buf); off++) {
                        if (buf[off] == 0xAA && buf[off+1] == 0x55 &&
                            buf[off+2] == 0xAA && buf[off+3] == 0x55 &&
                            buf[off+4] == SNAPSHOT_MSG) {
                            uint8_t type = 0;
                            uint8_t payload[sizeof(FlightSnapshotData)] = {};
                            size_t  payload_len = 0;
                            if (TR_I2C_Interface::unpackMessage(
                                    buf + off, kRespFrameLen,
                                    type, payload, sizeof(payload),
                                    payload_len, true) &&
                                type == SNAPSHOT_MSG &&
                                payload_len == sizeof(FlightSnapshotData)) {
                                memcpy(&snap, payload, sizeof(FlightSnapshotData));
                                if (snap.magic   == FlightSnapshotData::MAGIC &&
                                    snap.version == FlightSnapshotData::VERSION &&
                                    snap.rocket_state == (uint8_t)INFLIGHT &&
                                    snap.crc32 == computeSnapshotCRC(snap)) {
                                    valid = true;
                                } else {
                                    ESP_LOGW(TAG, "[RECOVERY] Snapshot invalid (magic=0x%08lX state=%u crc=%s)",
                                             (unsigned long)snap.magic, snap.rocket_state,
                                             (snap.crc32 == computeSnapshotCRC(snap)) ? "OK" : "FAIL");
                                }
                                break;
                            }
                        }
                    }
                } else {
                    ESP_LOGW(TAG, "[RECOVERY] I2C read for snapshot failed: %s", esp_err_to_name(rerr));
                }
            } else {
                ESP_LOGW(TAG, "[RECOVERY] Failed to send GET_FLIGHT_SNAPSHOT");
            }
        }

        if (valid) {
            ESP_LOGW(TAG, "========================================");
            ESP_LOGW(TAG, "[RECOVERY] INFLIGHT REBOOT RECOVERY from %s", resetReasonStr(rst));
            ESP_LOGW(TAG, "[RECOVERY] Flight elapsed: %lu ms", (unsigned long)snap.flight_elapsed_ms);
            ESP_LOGW(TAG, "========================================");

            const uint32_t now_ms = millis();

            // Restore flight state
            rocket_state = INFLIGHT;

            // Rebase timestamps to new millis() epoch
            launch_time_millis = now_ms - snap.flight_elapsed_ms;
            if (snap.pyro_apogee_detected) {
                pyro_apogee_detected = true;
                pyro_apogee_time_ms = launch_time_millis + snap.apogee_elapsed_ms;
            }
            if (snap.burnout_detected) {
                burnout_detected = true;
                burnout_time_ms = launch_time_millis + snap.burnout_elapsed_ms;
            }

            // Restore pyro state (safety-critical: no double-fire, no missed fire)
            portENTER_CRITICAL(&pyro_spinlock);
            pyro1_fired = snap.pyro1_fired;
            pyro2_fired = snap.pyro2_fired;
            pyro1_fire_start_ms = 0;
            pyro2_fire_start_ms = 0;
            portEXIT_CRITICAL(&pyro_spinlock);

            // Re-arm unfired pyro channels (GPIO ARM pins reset on reboot)
            if (snap.pyro1_armed && !snap.pyro1_fired && pyro_config.ch1_enabled) {
                portENTER_CRITICAL(&pyro_spinlock);
                gpio_set_level((gpio_num_t)config::PYRO1_ARM_PIN, 1);
                pyro1_armed = true;
                portEXIT_CRITICAL(&pyro_spinlock);
                ESP_LOGW(TAG, "[RECOVERY] CH1 re-armed");
            }
            if (snap.pyro2_armed && !snap.pyro2_fired && pyro_config.ch2_enabled) {
                portENTER_CRITICAL(&pyro_spinlock);
                gpio_set_level((gpio_num_t)config::PYRO2_ARM_PIN, 1);
                pyro2_armed = true;
                portEXIT_CRITICAL(&pyro_spinlock);
                ESP_LOGW(TAG, "[RECOVERY] CH2 re-armed");
            }

            ESP_LOGW(TAG, "[RECOVERY] Pyro: apogee=%d ch1_armed=%d ch1_fired=%d ch2_armed=%d ch2_fired=%d",
                     pyro_apogee_detected, pyro1_armed, pyro1_fired, pyro2_armed, pyro2_fired);

            // Restore flight references
            ground_pressure_pa = snap.ground_pressure_pa;
            ref_lat_rad = snap.ref_lat_rad;
            ref_lon_rad = snap.ref_lon_rad;
            ref_alt_m   = snap.ref_alt_m;
            have_ref_pos = true;
            ref_pos_frozen = true;
            ground_pressure_found = true;

            // Restore control state
            ekf_initialized  = snap.ekf_initialized;
            guidance_enabled = snap.guidance_enabled;
            servo_enabled    = snap.servo_enabled;
            landed_actions_done = false;
            end_flight_sent = false;

            // Restore EKF state.  The wire snapshot stores only the
            // diagonal of P (cross-correlations get rebuilt over the
            // next ~0.5-1 s of measurement updates) — see
            // FlightSnapshotData comment in RocketComputerTypes.h.
            if (snap.ekf_initialized) {
                EkfStateSnapshot ekf_state = {};
                memcpy(ekf_state.pos_rrm,     snap.ekf_pos_rrm,     sizeof(ekf_state.pos_rrm));
                memcpy(ekf_state.vel_ned_mps, snap.ekf_vel_ned_mps, sizeof(ekf_state.vel_ned_mps));
                memcpy(ekf_state.quat,        snap.ekf_quat,        sizeof(ekf_state.quat));
                memcpy(ekf_state.accel_bias,  snap.ekf_accel_bias,  sizeof(ekf_state.accel_bias));
                memcpy(ekf_state.gyro_bias,   snap.ekf_gyro_bias,   sizeof(ekf_state.gyro_bias));
                // Zero P here; setCovFromDiag fills the diagonal below.
                memset(ekf_state.P, 0, sizeof(ekf_state.P));
                ekf_state.t_prev_us = snap.ekf_t_prev_us;
                memcpy(ekf_state.euler,       snap.ekf_euler,       sizeof(ekf_state.euler));
                ekf.setState(ekf_state);
                // Copy diag to a local before passing to the float(&)[15]
                // reference — packed-struct fields can't bind directly.
                float P_diag_local[15];
                memcpy(P_diag_local, snap.ekf_P_diag, sizeof(P_diag_local));
                ekf.setCovFromDiag(P_diag_local);

                // Inflate covariance for reboot uncertainty.
                for (int i = 0; i < 3; i++) ekf.inflateCovDiag(i, 25.0f);     // pos: 5m sigma
                for (int i = 3; i < 6; i++) ekf.inflateCovDiag(i, 25.0f);     // vel: 5 m/s
                for (int i = 6; i < 9; i++) ekf.inflateCovDiag(i, 0.03f);     // attitude: ~10°
                ESP_LOGW(TAG, "[RECOVERY] EKF state restored (P diag from snap, off-diag rebuilding)");
            }

            // Hold servos neutral for 500ms while EKF settles
            reboot_recovery = true;
            reboot_recovery_telem = true;
            servo_settle_end_ms = now_ms + 500;

            // Mark launch flag so kinematic checks don't re-trigger launch detection
            kinematics.launch_flag = true;
        }

        // Clear stale snapshot on normal boot (prevents recovery on next power cycle)
        if (!reboot_recovery) {
            clearFlightSnapshot();
        }
    }

    ESP_LOGI(TAG, "Setup complete");
    ESP_LOGI(TAG, "Setup complete…");
    triggerBlueLedFlash(millis());

}

// Loop is responsible for reading sensor data
static void loop_fc()
{ 
    // ### Read and Send Sensor Data ###
    // Poll as fast as possible so high-rate sensor frames are not dropped by
    // loop-period gating.
    static uint32_t dbg_ism6_reads = 0, dbg_bmp_reads = 0, dbg_mmc_reads = 0, dbg_iis2mdc_reads = 0, dbg_gnss_reads = 0;

    if (sensor_collector.getISM6HG256Data(ism6hg256_data))
    {
        dbg_ism6_reads++;
        sensor_converter.convertISM6HG256Data(ism6hg256_data, ism6_latest_si);
        have_ism6_si = true;

        memcpy(ism6hg256_data_buffer,
               &ism6hg256_data,
               SIZE_OF_ISM6HG256_DATA);

        (void)enqueueI2STx(ISM6HG256_MSG,
                           ism6hg256_data_buffer,
                           SIZE_OF_ISM6HG256_DATA);
    }

    if (sensor_collector.getBMP585Data(bmp585_data))
    {
        dbg_bmp_reads++;
        sensor_converter.convertBMP585Data(bmp585_data, bmp_latest_si);
        have_bmp_si = true;
        bmp_new_for_kf = true;

        memcpy(bmp585_data_buffer,
               &bmp585_data,
               SIZE_OF_BMP585_DATA);

        (void)enqueueI2STx(BMP585_MSG,
                           bmp585_data_buffer,
                           SIZE_OF_BMP585_DATA);
    }

    if (sensor_collector.getMMC5983MAData(mmc5983ma_data))
    {
        dbg_mmc_reads++;
        sensor_converter.convertMMC5983MAData(mmc5983ma_data, mmc_latest_si);
        have_mmc_si = true;

        memcpy(mmc5983ma_data_buffer,
               &mmc5983ma_data,
               SIZE_OF_MMC5983MA_DATA);

        (void)enqueueI2STx(MMC5983MA_MSG,
                           mmc5983ma_data_buffer,
                           SIZE_OF_MMC5983MA_DATA);
    }

    if (sensor_collector.getIIS2MDCData(iis2mdc_data))
    {
        dbg_iis2mdc_reads++;
        sensor_converter.convertIIS2MDCData(iis2mdc_data, iis2mdc_latest_si);
        have_iis2mdc_si = true;

        memcpy(iis2mdc_data_buffer,
               &iis2mdc_data,
               SIZE_OF_IIS2MDC_DATA);

        (void)enqueueI2STx(IIS2MDC_MSG,
                           iis2mdc_data_buffer,
                           SIZE_OF_IIS2MDC_DATA);
    }

    if (sensor_collector.getGNSSData(gnss_data))
    {
        dbg_gnss_reads++;
        sensor_converter.convertGNSSData(gnss_data, gnss_latest_si);
        have_gnss_si = true;
        gps_new_for_kc = true;

        memcpy(gnss_data_buffer,
               &gnss_data,
               SIZE_OF_GNSS_DATA);

        (void)enqueueI2STx(GNSS_MSG,
                           gnss_data_buffer,
                           SIZE_OF_GNSS_DATA);
    }

    // --- Flight logic update ---
    const uint32_t logic_now_us = micros();
    if ((logic_now_us - last_flight_loop_update_time) >= flight_loop_period)
    {
        lt_loop_count++;
        last_flight_loop_update_time = logic_now_us;

        // ### Pressure altitude calculation (for kinematic checks) ###
        const uint32_t now_ms = millis();
        float pressure_altitude_m = 0.0f;
        if (have_bmp_si)
        {
            // Track ground pressure continuously through pre-flight phases so the
            // reference is the last steady-state reading before launch, not just
            // the first noisy sample taken right after sensor init.
            // Freeze the reference once we enter PRELAUNCH: if we kept updating,
            // the ratio bmp/ground would always be ~1.0, pressure_altitude_m would
            // stay ~0, the Kalman-filtered altitude rate would never exceed the
            // 1.0 m/s launch threshold, and launch detection would deadlock.
            // The pad pressure is well-established by the time PRELAUNCH is entered
            // (updated through INITIALIZATION + READY, typically 10+ seconds).
            // No isSimActive() guard here: the state check is sufficient, and
            // letting sim data flow through the same path ensures the freeze
            // logic is tested in simulation.
            if (bmp_latest_si.pressure > 0.0f &&
                (rocket_state == INITIALIZATION ||
                 rocket_state == READY))
            {
                ground_pressure_pa = bmp_latest_si.pressure;
                ground_pressure_found = true;
            }
            if (bmp_latest_si.pressure > 0.0f && ground_pressure_pa > 0.0f)
            {
                pressure_altitude_m = 44330.0f *
                                      (1.0f - powf(bmp_latest_si.pressure / ground_pressure_pa, 1.0f / 5.255f));
            }
        }

        // ### State estimation from GPS/INS EKF ###
        float imu_pos[3] = {0.0f, 0.0f, 0.0f};   // local ENU [E,N,U] meters
        float imu_vel[3] = {0.0f, 0.0f, 0.0f};   // local ENU [E,N,U] m/s
        float imu_rpy[3] = {0.0f, 0.0f, 0.0f};   // Euler [roll,pitch,yaw] rad
        float roll_rate_dps = 0.0f;
        float accel_norm = 0.0f;

        if (have_ism6_si)
        {
            const float ax = (float)ism6_latest_si.low_g_acc_x;
            const float ay = (float)ism6_latest_si.low_g_acc_y;
            const float az = (float)ism6_latest_si.low_g_acc_z;
            roll_rate_dps = (float)ism6_latest_si.gyro_x;

            // Switch to high-G channel when any low-G axis approaches saturation
            static constexpr float kLowGSatThreshMps2 =
                (config::ISM6_LOW_G_FS_G - 0.5f) * 9.80665f;
            const bool low_g_near_sat = (fabsf(ax) > kLowGSatThreshMps2) ||
                                        (fabsf(ay) > kLowGSatThreshMps2) ||
                                        (fabsf(az) > kLowGSatThreshMps2);
            const float hx = (float)ism6_latest_si.high_g_acc_x;
            const float hy = (float)ism6_latest_si.high_g_acc_y;
            const float hz = (float)ism6_latest_si.high_g_acc_z;
            const float acc_x = low_g_near_sat ? hx : ax;
            const float acc_y = low_g_near_sat ? hy : ay;
            const float acc_z = low_g_near_sat ? hz : az;
            accel_norm = sqrtf(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);

            // ── Build EKF input: IMU in FRD body frame ──
            // SensorConverter outputs FLU (X=Fwd, Y=Left, Z=Up).
            // EKF expects FRD (X=Fwd, Y=Right, Z=Down): negate Y and Z.
            EkfIMUData ekf_imu = {};
            ekf_imu.time_us = ism6_latest_si.time_us;
            ekf_imu.acc_x  =  (double)acc_x;
            ekf_imu.acc_y  = -(double)acc_y;   // FLU Y=Left → FRD Y=Right
            ekf_imu.acc_z  = -(double)acc_z;   // FLU Z=Up   → FRD Z=Down
            ekf_imu.gyro_x =  (double)ism6_latest_si.gyro_x;
            ekf_imu.gyro_y = -(double)ism6_latest_si.gyro_y;
            ekf_imu.gyro_z = -(double)ism6_latest_si.gyro_z;

            // ── Build EKF input: Magnetometer in FRD body frame ──
            // Prefer IIS2MDC (new PCB) when its samples are flowing, fall
            // back to MMC5983MA on legacy boards. Only one is populated on
            // any given board, so this picks "whichever mag is alive".
            //
            // Magnitude gate: the Mahony AHRS only checks (0,0,0) and then
            // normalises, so an uncalibrated hard-iron offset (e.g. the new
            // PCB shows ~1700 µT) would dominate the heading reference. Skip
            // the sample unless |m| sits in a sensible Earth-field band; the
            // AHRS treats a zeroed input as "no mag, use gyro+accel only".
            EkfMagData ekf_mag = {};
            double mag_x_frd = 0.0, mag_y_frd = 0.0, mag_z_frd = 0.0;
            uint32_t mag_time_us = 0;
            bool have_mag_si = false;
            if (have_iis2mdc_si) {
                mag_time_us = iis2mdc_latest_si.time_us;
                mag_x_frd =  iis2mdc_latest_si.mag_x_uT;
                mag_y_frd = -iis2mdc_latest_si.mag_y_uT;       // FLU→FRD
                mag_z_frd = -iis2mdc_latest_si.mag_z_uT;       // FLU→FRD
                have_mag_si = true;
            } else if (have_mmc_si) {
                mag_time_us = mmc_latest_si.time_us;
                mag_x_frd =  mmc_latest_si.mag_x_uT;
                mag_y_frd = -mmc_latest_si.mag_y_uT;           // FLU→FRD
                mag_z_frd = -mmc_latest_si.mag_z_uT;           // FLU→FRD
                have_mag_si = true;
            }
            if (have_mag_si) {
                const double m2 = mag_x_frd * mag_x_frd
                                + mag_y_frd * mag_y_frd
                                + mag_z_frd * mag_z_frd;
                // Earth field at surface is ~25–65 µT; widen to 15–80 µT.
                if (m2 >= (15.0 * 15.0) && m2 <= (80.0 * 80.0)) {
                    ekf_mag.time_us = mag_time_us;
                    ekf_mag.mag_x = mag_x_frd;
                    ekf_mag.mag_y = mag_y_frd;
                    ekf_mag.mag_z = mag_z_frd;
                }
            }

            // ── Build EKF input: GNSS in LLA + NED ──
            // Layered quality gating:
            //   Gate 1: fix >= 3, sats >= MIN, h_acc < MAX, new timestamp
            //   Gate 3: init requires h_acc < INIT_MAX and vel < INIT_MAX_VEL
            // Gate 2 (chi-squared innovation test) lives inside EKF::measUpdate.
            EkfGNSSDataLLA ekf_gnss = {};
            const bool gnss_fix_is_new =
                (gnss_latest_si.second != last_gnss_fix_second) ||
                (gnss_latest_si.milli_second != last_gnss_fix_ms);
            const bool hacc_ok =
                (config::GNSS_MAX_HACC_M <= 0.0f) ||  // 0 = disable h_acc gate
                (gnss_latest_si.horizontal_accuracy < config::GNSS_MAX_HACC_M);
            const bool gnss_gate1 =
                have_gnss_si &&
                gnss_latest_si.fix_mode >= 3U &&
                gnss_latest_si.num_sats >= config::GNSS_MIN_SATS &&
                hacc_ok &&
                gnss_fix_is_new;
            // Gate 3 (init-specific): tighter accuracy + low velocity on pad
            const float gnss_vel_mag = sqrtf(
                (float)(gnss_latest_si.vel_e * gnss_latest_si.vel_e +
                        gnss_latest_si.vel_n * gnss_latest_si.vel_n +
                        gnss_latest_si.vel_u * gnss_latest_si.vel_u));
            const bool hacc_init_ok =
                (config::GNSS_MAX_HACC_INIT_M <= 0.0f) ||
                (gnss_latest_si.horizontal_accuracy < config::GNSS_MAX_HACC_INIT_M);
            const bool gnss_gate3_init =
                gnss_gate1 &&
                gnss_latest_si.num_sats >= config::GNSS_MIN_SATS_INIT &&
                hacc_init_ok &&
                gnss_vel_mag < config::GNSS_MAX_VEL_INIT_MPS;

            if (gnss_gate1)
            {
                static constexpr double DEG2RAD = M_PI / 180.0;
                ekf_gnss.time_us   = gnss_latest_si.time_us;
                ekf_gnss.lat_rad   = gnss_latest_si.lat * DEG2RAD;
                ekf_gnss.lon_rad   = gnss_latest_si.lon * DEG2RAD;
                ekf_gnss.alt_m     = gnss_latest_si.alt;
                ekf_gnss.vel_n_mps = (float)gnss_latest_si.vel_n;
                ekf_gnss.vel_e_mps = (float)gnss_latest_si.vel_e;
                ekf_gnss.vel_d_mps = -(float)gnss_latest_si.vel_u; // ENU U→NED D
                last_gnss_time_us_for_ekf = gnss_latest_si.time_us;
                last_gnss_fix_second     = gnss_latest_si.second;
                last_gnss_fix_ms         = gnss_latest_si.milli_second;

                // Scale GNSS noise by h_acc — inflate R when receiver is uncertain.
                // Nominal R assumes h_acc ≈ 3 m, so scale = max(1, h_acc / 3).
                const float h_acc = gnss_latest_si.horizontal_accuracy;
                const float gnss_scale = (h_acc > 3.0f) ? (h_acc / 3.0f) : 1.0f;
                ekf.setGpsNoiseScale(gnss_scale);

                // Running average of GNSS fixes for launch-site reference.
                // Only accumulate fixes that pass the stricter init gate (Gate 3).
                if (!ref_pos_frozen && gnss_gate3_init) {
                    if (ref_pos_count == 0) {
                        ref_pos_first_time_ms = now_ms;
                    }
                    ref_lat_sum += ekf_gnss.lat_rad;
                    ref_lon_sum += ekf_gnss.lon_rad;
                    ref_alt_sum += ekf_gnss.alt_m;
                    ref_pos_count++;
                    ref_lat_rad = ref_lat_sum / ref_pos_count;
                    ref_lon_rad = ref_lon_sum / ref_pos_count;
                    ref_alt_m   = ref_alt_sum / ref_pos_count;
                    have_ref_pos = true;
                    // Freeze after 2 minutes of accumulation
                    if ((now_ms - ref_pos_first_time_ms) >= REF_POS_MAX_AGE_MS) {
                        ref_pos_frozen = true;
                        ESP_LOGI(TAG, "[EKF] Ref pos frozen (2 min): n=%lu",
                                      (unsigned long)ref_pos_count);
                    }
                }
            } else {
                // Pass stale GNSS timestamp so EKF skips measurement update
                ekf_gnss.time_us = last_gnss_time_us_for_ekf;
            }

            // ── Initialize or update the EKF ──
            // Wait for valid GNSS fix before init — avoids initializing
            // at lat=0,lon=0 and then corrupting attitude when fix arrives.
            //
            // EKF decimation: run predict+update every Nth flight-loop tick.
            // Sensor consumption and I2C logging still happen every iteration,
            // so no IMU samples are lost — only the expensive EKF math is
            // skipped on the "off" ticks.  The EKF's internal dt tracking
            // (tPrev_us_) automatically accounts for the longer interval.
            static uint8_t ekf_decim_ctr = 0;
            const bool run_ekf_this_tick =
                (++ekf_decim_ctr >= config::EKF_DECIMATION);
            if (run_ekf_this_tick) ekf_decim_ctr = 0;

            if (!ekf_initialized)
            {
                // Gate 3: only init with high-quality GNSS (tight h_acc + low vel)
                if (have_ref_pos && gnss_gate3_init) {
                    ekf.init(ekf_imu, ekf_gnss, ekf_mag);

                    // Pad heading initialization: compute initial quaternion from
                    // accel (pitch) + known heading, bypassing noisy magnetometer.
                    {
                        static constexpr double DEG2RAD_d = M_PI / 180.0;
                        const float g = 9.807f;
                        // acc in FRD: nose-up → acc_x ≈ +g, acc_z ≈ 0
                        float g_mag = sqrtf(ekf_imu.acc_x*ekf_imu.acc_x +
                                            ekf_imu.acc_y*ekf_imu.acc_y +
                                            ekf_imu.acc_z*ekf_imu.acc_z);
                        if (g_mag < 0.1f) g_mag = g;
                        float pitch_rad = asinf((float)ekf_imu.acc_x / g_mag);
                        float roll_rad = 0.0f;
                        if (fabsf(pitch_rad) < 80.0f * (float)DEG2RAD_d) {
                            roll_rad = atan2f(-(float)ekf_imu.acc_y, -(float)ekf_imu.acc_z);
                        }
                        float heading_rad = (float)(config::PAD_HEADING_DEG * DEG2RAD_d);

                        // Build body-to-NED quaternion from ZYX Euler (yaw, pitch, roll)
                        float cy = cosf(heading_rad * 0.5f), sy = sinf(heading_rad * 0.5f);
                        float cp = cosf(pitch_rad * 0.5f),   sp = sinf(pitch_rad * 0.5f);
                        float cr = cosf(roll_rad * 0.5f),     sr = sinf(roll_rad * 0.5f);
                        float q0 = cr*cp*cy + sr*sp*sy;
                        float q1 = sr*cp*cy - cr*sp*sy;
                        float q2 = cr*sp*cy + sr*cp*sy;
                        float q3 = cr*cp*sy - sr*sp*cy;

                        ekf.setQuaternion(q0, q1, q2, q3);
                        ESP_LOGI(TAG, "[EKF] Init: pitch=%.1f roll=%.1f heading=%.1f deg",
                                      (double)(pitch_rad * 180.0f / M_PI),
                                      (double)(roll_rad * 180.0f / M_PI),
                                      (double)config::PAD_HEADING_DEG);
                    }
                    ekf_initialized = true;
                }
            }
            else if (run_ekf_this_tick)
            {
                const uint32_t ekf_t0 = micros();

                // AHRS accel correction: enabled on pad and after apogee (descent).
                // Disabled during thrust and coast where accel is far from 1g.
                // The 0.5g–1.5g magnitude gate inside the EKF provides additional
                // rejection, but blanket-disabling during descent starves the
                // filter of its gravity reference and freezes the velocity estimate.
                const bool post_apogee = kinematics.apogee_flag;
                const bool use_ahrs_acc = (rocket_state != INFLIGHT) || post_apogee;
                ekf.update(use_ahrs_acc, ekf_imu, ekf_gnss, ekf_mag);

                // Barometer measurement update (when new BMP sample available)
                if (bmp_new_for_kf && have_bmp_si)
                {
                    static float prev_baro_alt_m = 0.0f;
                    static bool  prev_baro_valid = false;
                    // mach_locked_out is now file-scope (used by apogee voting)

                    // --- Transonic lockout (hysteresis) ---
                    bool baro_locked = false;
                    if (config::BARO_MACH_LOCKOUT_ON > 0.0f)
                    {
                        float vel_ned_baro[3];
                        ekf.getVelEst(vel_ned_baro);
                        const float spd = sqrtf(vel_ned_baro[0] * vel_ned_baro[0] +
                                                vel_ned_baro[1] * vel_ned_baro[1] +
                                                vel_ned_baro[2] * vel_ned_baro[2]);
                        if (!mach_locked_out && spd > config::BARO_MACH_LOCKOUT_ON)
                            mach_locked_out = true;
                        else if (mach_locked_out && spd < config::BARO_MACH_LOCKOUT_OFF)
                            mach_locked_out = false;
                        baro_locked = mach_locked_out;
                    }

                    // --- Spike rejection ---
                    const float delta = fabsf(pressure_altitude_m - prev_baro_alt_m);
                    const bool  spike = prev_baro_valid &&
                                        (delta > config::BARO_SPIKE_THRESH_M);

                    if (!spike && !baro_locked)
                    {
                        EkfBaroData ekf_baro = {};
                        ekf_baro.time_us = bmp_latest_si.time_us;
                        // ISA pressure altitude relative to launch site
                        ekf_baro.altitude_m = (double)pressure_altitude_m;
                        ekf.baroMeasUpdate(ekf_baro);
                    }
                    else if (spike)
                    {
                        ESP_LOGW(TAG, "[BARO] Spike rejected: delta=%.1f m (thresh=%.1f)",
                                 (double)delta, (double)config::BARO_SPIKE_THRESH_M);
                    }
                    // Always update reference so the next sample compares against
                    // the latest reading — prevents prolonged rejection after a
                    // single transient spike or mach lockout exit.
                    prev_baro_alt_m = pressure_altitude_m;
                    prev_baro_valid = true;
                }

                const uint32_t ekf_us = micros() - ekf_t0;
                lt_ekf_total_us += ekf_us;
                lt_ekf_count++;
                if (ekf_us > lt_ekf_max_us) lt_ekf_max_us = ekf_us;
            }

            // ── Extract EKF outputs (only after init) ──
            if (ekf_initialized) {
                ekf.getOrientEst(imu_rpy);   // [roll, pitch, yaw] rad, NED convention

                // Convert EKF NED velocity to ENU for downstream consumers
                float vel_ned[3];
                ekf.getVelEst(vel_ned);
                imu_vel[0] = vel_ned[1];   // NED East  → ENU East
                imu_vel[1] = vel_ned[0];   // NED North → ENU North
                imu_vel[2] = -vel_ned[2];  // NED Down  → ENU Up

                // Convert EKF LLA position to local ENU relative to launch site
                double pos_lla[3];
                ekf.getPosEst(pos_lla);
                static constexpr double R_EARTH = 6378137.0;
                double dlat = pos_lla[0] - ref_lat_rad;
                double dlon = pos_lla[1] - ref_lon_rad;
                double dalt = pos_lla[2] - ref_alt_m;
                double cos_lat = cos(ref_lat_rad);
                imu_pos[0] = (float)(dlon * (R_EARTH + ref_alt_m) * cos_lat); // East
                imu_pos[1] = (float)(dlat * (R_EARTH + ref_alt_m));            // North
                imu_pos[2] = (float)dalt;                                       // Up
            }
        }

        if (have_gnss_si && (gnss_latest_si.fix_mode >= 3U))
        {
            if (!gnss_started)
            {
                gnss_started = true;
                valid_gnss_start_millis = now_ms;
            }
        }

        // Skip the status query during INFLIGHT — no app commands are
        // processed mid-flight.  I2C is now command-only (telemetry uses I2S)
        // Mutex protects the I2C bus from the sender task.
        //
        // Pipelined protocol: READ first (response from previous query),
        // then SEND the next query.  This gives the OutComputer a full
        // 250 ms poll interval to process each query and pre-load its
        // response into the slave TX FIFO, avoiding the race where the
        // OC hasn't called i2c_slave_transmit() yet.
        if (rocket_state != INFLIGHT && (now_ms - out_ready_request_time_ms) > 250U)
        {
            out_ready_request_time_ms = now_ms;

            xSemaphoreTake(i2c_bus_mutex, portMAX_DELAY);

            // ── Step 1: READ the response queued by the *previous* query ──
            static bool query_pending = false;  // true after first query sent
            bool query_ok = false;
            cfg_read_cache_len = 0;

            if (query_pending)
            {
                const uint32_t gor_start = micros();
                uint8_t combined_buf[COMBINED_READ_SIZE] = {};
                esp_err_t read_err = i2c_interface.masterRead(
                    combined_buf, COMBINED_READ_SIZE, 10);

                if (read_err == ESP_OK)
                {
                    uint8_t resp_type = 0;
                    uint8_t resp_payload[2] = {};
                    size_t  resp_payload_len = 0;
                    if (TR_I2C_Interface::unpackMessage(
                            combined_buf, 10,
                            resp_type, resp_payload, sizeof(resp_payload),
                            resp_payload_len, true)
                        && resp_type == OUT_STATUS_RESPONSE
                        && resp_payload_len >= 1)
                    {
                        query_ok = true;
                        if (resp_payload[0] != 0) { out_ready = true; }
                        if (resp_payload_len >= 2 && resp_payload[1] != 0)
                        {
                            out_pending_command = resp_payload[1];
                        }
                        memcpy(cfg_read_cache, combined_buf + 10,
                               COMBINED_READ_SIZE - 10);
                        cfg_read_cache_len = COMBINED_READ_SIZE - 10;
                    }
                }

                const uint32_t gor_us = micros() - gor_start;
                if (gor_us > i2c_gor_max_us) { i2c_gor_max_us = gor_us; }
                if (query_ok) { i2c_query_ok++; } else { i2c_query_fail++; }

                if (!query_ok) {
                    ESP_LOGW(TAG, "[I2C] read FAIL cmd=0x%02X dur=%lu us",
                                  (unsigned)out_pending_command,
                                  (unsigned long)gor_us);
                }
            }

            // ── Step 2: SEND the next query ──
            // OC will process this and pre-load its response before the
            // next poll iteration reads it.
            esp_err_t send_err = i2c_interface.sendMessage(
                OUT_STATUS_QUERY,
                reinterpret_cast<const uint8_t*>(&out_status_query_data),
                sizeof(out_status_query_data),
                10);
            if (send_err == ESP_OK) { query_pending = true; }

            // NOTE: mutex is held through command processing below, so
            // readConfigFrame() I2C reads have an idle bus.  Released after
            // the command dispatch block.
        }

        // Dedup: OutComputer repeats each command for 5 polls for I2C
        // reliability.  Process only the first delivery.
        if (out_pending_command == 0U)
        {
            last_processed_cmd = 0U;  // reset once OutComputer clears
        }
        if (out_pending_command != 0U && out_pending_command != last_processed_cmd)
        {
            last_processed_cmd = out_pending_command;
            ESP_LOGI(TAG, "[I2C RX] pending_command=0x%02X", (unsigned)out_pending_command);
            if (out_pending_command == CAMERA_START)
            {
                cameraStart(now_ms);
            }
            else if (out_pending_command == CAMERA_STOP)
            {
                cameraStop(now_ms);
            }
            else if (out_pending_command == SOUNDS_ENABLE)
            {
                enable_sounds = true;
                prefs.begin("rocket", false);  // read-write
                prefs.putBool("sounds", true);
                prefs.end();
                ESP_LOGI(TAG, "Sounds ENABLED (saved to NVS)");
                // Confirmation beep so the user knows it worked
                if (piezo_pwm_ready)
                {
                    piezoStart(2600, 100, micros());
                }
            }
            else if (out_pending_command == SOUNDS_DISABLE)
            {
                enable_sounds = false;
                piezoStop();
                prefs.begin("rocket", false);  // read-write
                prefs.putBool("sounds", false);
                prefs.end();
                ESP_LOGI(TAG, "Sounds DISABLED (saved to NVS)");
            }
            else if (out_pending_command == SERVO_CONFIG_PENDING)
            {
                delay(1);  // let slave TX FIFO settle
                uint8_t cfg_payload[14];
                size_t  cfg_len = 0;
                if (readConfigFrame(SERVO_CONFIG_MSG, sizeof(ServoConfigData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(ServoConfigData))
                {
                    ServoConfigData cfg;
                    memcpy(&cfg, cfg_payload, sizeof(cfg));
                    ESP_LOGI(TAG, "[SERVO CFG] bias=[%d,%d,%d,%d] hz=%d min=%d max=%d",
                                  cfg.bias_us[0], cfg.bias_us[1],
                                  cfg.bias_us[2], cfg.bias_us[3],
                                  cfg.hz, cfg.min_us, cfg.max_us);

                    for (int i = 0; i < 4; ++i)
                    {
                        servo_control.setBias(i, cfg.bias_us[i]);
                    }
                    if (rocket_state != INFLIGHT)
                    {
                        servo_control.setServoTiming(cfg.hz, cfg.min_us, cfg.max_us);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "[SERVO CFG] Hz/min/max deferred (INFLIGHT)");
                    }

                    prefs.begin("servo", false);
                    prefs.putShort("b1", cfg.bias_us[0]);
                    prefs.putShort("b2", cfg.bias_us[1]);
                    prefs.putShort("b3", cfg.bias_us[2]);
                    prefs.putShort("b4", cfg.bias_us[3]);
                    prefs.putShort("hz", cfg.hz);
                    prefs.putShort("min", cfg.min_us);
                    prefs.putShort("max", cfg.max_us);
                    prefs.end();
                    ESP_LOGI(TAG, "[SERVO CFG] Saved to NVS");
                }
                else
                {
                    last_processed_cmd = 0U;  // retry on next poll
                    ESP_LOGW(TAG, "[SERVO CFG] Config not in this read, will retry");
                }
            }
            else if (out_pending_command == PID_CONFIG_PENDING)
            {
                delay(1);
                uint8_t cfg_payload[20];
                size_t  cfg_len = 0;
                if (readConfigFrame(PID_CONFIG_MSG, sizeof(PIDConfigData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(PIDConfigData))
                {
                    PIDConfigData cfg;
                    memcpy(&cfg, cfg_payload, sizeof(cfg));
                    ESP_LOGI(TAG, "[PID CFG] kp=%.4f ki=%.4f kd=%.4f min=%.1f max=%.1f",
                                  (double)cfg.kp, (double)cfg.ki, (double)cfg.kd,
                                  (double)cfg.min_cmd, (double)cfg.max_cmd);

                    servo_control.setPIDGains(cfg.kp, cfg.ki, cfg.kd);
                    servo_control.setPIDLimits(cfg.min_cmd, cfg.max_cmd);

                    prefs.begin("servo", false);
                    prefs.putFloat("kp", cfg.kp);
                    prefs.putFloat("ki", cfg.ki);
                    prefs.putFloat("kd", cfg.kd);
                    prefs.putFloat("mincmd", cfg.min_cmd);
                    prefs.putFloat("maxcmd", cfg.max_cmd);
                    prefs.end();
                    ESP_LOGI(TAG, "[PID CFG] Saved to NVS");
                }
                else
                {
                    last_processed_cmd = 0U;  // retry on next poll
                    ESP_LOGW(TAG, "[PID CFG] Config not in this read, will retry");
                }
            }
            else if (out_pending_command == SERVO_CTRL_ENABLE)
            {
                if (!servo_enabled && servoPinsValid())
                {
                    servo_control.setSetpoint(config::ROLL_RATE_SET_POINT);
                }
                servo_enabled = true;
                prefs.begin("rocket", false);
                prefs.putBool("servo_en", true);
                prefs.end();
                ESP_LOGI(TAG, "Servo control ENABLED (saved to NVS)");
            }
            else if (out_pending_command == SERVO_CTRL_DISABLE)
            {
                servo_enabled = false;
                servo_control.stowControl();
                prefs.begin("rocket", false);
                prefs.putBool("servo_en", false);
                prefs.end();
                ESP_LOGI(TAG, "Servo control DISABLED (saved to NVS)");
            }
            else if (out_pending_command == SIM_CONFIG_PENDING)
            {
                delay(5);  // let slave TX FIFO settle
                uint8_t cfg_payload[16];
                size_t  cfg_len = 0;
                if (readConfigFrame(SIM_CONFIG_MSG, sizeof(SimConfigData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(SimConfigData))
                {
                    SimConfigData cfg;
                    memcpy(&cfg, cfg_payload, sizeof(cfg));
                    sensor_collector.configureSim(cfg);
                }
                else
                {
                    last_processed_cmd = 0U;  // retry on next poll
                    ESP_LOGW(TAG, "[SIM CFG] Config not in this read, will retry");
                }
            }
            else if (out_pending_command == SIM_START_CMD)
            {
                // If config and start commands arrived close together (e.g. via LoRa relay),
                // the start may have overwritten the config command before we read it.
                // Try reading any pending config frame before starting the sim.
                {
                    delay(5);
                    uint8_t cfg_payload[16];
                    size_t  cfg_len = 0;
                    if (readConfigFrame(SIM_CONFIG_MSG, sizeof(SimConfigData),
                                        cfg_payload, sizeof(cfg_payload), cfg_len)
                        && cfg_len >= sizeof(SimConfigData))
                    {
                        SimConfigData cfg;
                        memcpy(&cfg, cfg_payload, sizeof(cfg));
                        sensor_collector.configureSim(cfg);
                        ESP_LOGI(TAG, "[SIM] Config frame recovered on start cmd");
                    }
                }
                if (!sensor_collector.isSimConfigured()) {
                    // Config cmd was lost (e.g. LoRa relay) — use sensible defaults
                    SimConfigData defaults = {};
                    defaults.mass_kg          = 0.5f;   // 500g
                    defaults.thrust_n         = 20.0f;
                    defaults.burn_time_s      = 2.0f;
                    defaults.descent_rate_mps = 5.0f;
                    sensor_collector.configureSim(defaults);
                    ESP_LOGW(TAG, "[SIM] No config received, using defaults");
                }
                sensor_collector.startSim(ground_pressure_pa);
                ESP_LOGI(TAG, "[SIM] Start cmd received, sim active=%s ground_p=%.0f",
                              sensor_collector.isSimActive() ? "YES" : "NO",
                              (double)ground_pressure_pa);
            }
            else if (out_pending_command == SIM_STOP_CMD)
            {
                sensor_collector.stopSim();
                ESP_LOGI(TAG, "[SIM] Stop cmd received");
            }
            else if (out_pending_command == GROUND_TEST_START)
            {
                if (rocket_state == INFLIGHT) {
                    ESP_LOGW(TAG, "[GROUND TEST] Rejected — INFLIGHT");
                } else {
                    ground_test_active = true;
                    roll_rate_pid_standalone.reset();
                    // Loudly report the live mode so a mismatch with what the
                    // iOS app shows is obvious in the logs. Also report the
                    // roll-rate deadband so its effect on test behavior is
                    // obvious from the serial log.
                    ESP_LOGI(TAG, "[GROUND TEST] Started - mode=%s (guidance_enabled=%s) "
                                  "roll_rate_deadband=%.2f dps",
                                  guidance_enabled ? "ATTITUDE_HOLD+ROLL" : "ROLL_ONLY",
                                  guidance_enabled ? "true" : "false",
                                  (double)config::GROUND_TEST_ROLL_RATE_DEADBAND_DPS);
                }
            }
            else if (out_pending_command == GROUND_TEST_STOP)
            {
                ground_test_active = false;
                if (servo_enabled) servo_control.stowControl();
                roll_rate_pid_standalone.reset();
                last_external_roll_cmd_deg = 0.0f;
                ESP_LOGI(TAG, "[GROUND TEST] Stopped - servos stowed");
            }
            else if (out_pending_command == GYRO_CAL_CMD)
            {
                ESP_LOGI(TAG, "[CAL] Sensor calibration requested...");
                sensor_collector.calibrateGyro(config::ISM6HG256_ROT_Z_DEG);
                sensor_converter.setHighGBias(sensor_collector.hg_bias_x,
                                              sensor_collector.hg_bias_y,
                                              sensor_collector.hg_bias_z);
                // Propagate bias to OutComputer via status query payload
                out_status_query_data.hg_bias_x_cmss = (int16_t)lroundf(sensor_collector.hg_bias_x * 100.0f);
                out_status_query_data.hg_bias_y_cmss = (int16_t)lroundf(sensor_collector.hg_bias_y * 100.0f);
                out_status_query_data.hg_bias_z_cmss = (int16_t)lroundf(sensor_collector.hg_bias_z * 100.0f);
                // Persist to NVS
                prefs.begin("cal", false);
                prefs.putFloat("hgbx", sensor_collector.hg_bias_x);
                prefs.putFloat("hgby", sensor_collector.hg_bias_y);
                prefs.putFloat("hgbz", sensor_collector.hg_bias_z);
                prefs.end();
                ESP_LOGI(TAG, "[CAL] Sensor calibration complete (saved to NVS)");
            }
            else if (out_pending_command == GAIN_SCHED_ENABLE)
            {
                gain_sched_enabled = true;
                servo_control.enableGainSchedule(config::GAIN_SCHEDULE_V_REF, config::GAIN_SCHEDULE_V_MIN);
                prefs.begin("servo", false);
                prefs.putBool("gs", true);
                prefs.end();
                ESP_LOGI(TAG, "[CFG] Gain scheduling ENABLED");
            }
            else if (out_pending_command == GAIN_SCHED_DISABLE)
            {
                gain_sched_enabled = false;
                servo_control.disableGainSchedule();
                prefs.begin("servo", false);
                prefs.putBool("gs", false);
                prefs.end();
                ESP_LOGI(TAG, "[CFG] Gain scheduling DISABLED");
            }
            else if (out_pending_command == GUIDANCE_ENABLE)
            {
                guidance_enabled = true;
                prefs.begin("servo", false);
                prefs.putBool("guid_en", true);
                prefs.end();
                ESP_LOGI(TAG, "[CFG] PN Guidance ENABLED");
            }
            else if (out_pending_command == GUIDANCE_DISABLE)
            {
                guidance_enabled = false;
                guidance_active = false;
                guidance.reset();
                control_mixer.reset();
                roll_rate_pid_standalone.reset();
                prefs.begin("servo", false);
                prefs.putBool("guid_en", false);
                prefs.end();
                ESP_LOGI(TAG, "[CFG] PN Guidance DISABLED");
            }
            else if (out_pending_command == CAMERA_CONFIG_PENDING)
            {
                delay(1);
                uint8_t cfg_payload[1];
                size_t  cfg_len = 0;
                if (readConfigFrame(CAMERA_CONFIG_MSG, sizeof(CameraConfigData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(CameraConfigData))
                {
                    CameraConfigData cam_cfg;
                    memcpy(&cam_cfg, cfg_payload, sizeof(cam_cfg));
                    // Update runtime camera type — takes effect on next start/stop
                    // Note: the FC config.h CAMERA_TYPE is the compile-time default;
                    // this overrides it at runtime.
                    // Store in a runtime variable (not constexpr)
                    runtime_camera_type = cam_cfg.camera_type;
                    ESP_LOGI(TAG, "[CFG] Camera type: %u (%s)",
                             runtime_camera_type,
                             runtime_camera_type == CAM_TYPE_GOPRO ? "GoPro" :
                             runtime_camera_type == CAM_TYPE_RUNCAM ? "RunCam" : "None");
                }
            }
            else if (out_pending_command == PYRO_CONFIG_PENDING)
            {
                delay(1);
                uint8_t cfg_payload[12];
                size_t  cfg_len = 0;
                if (readConfigFrame(PYRO_CONFIG_MSG, sizeof(PyroConfigData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(PyroConfigData))
                {
                    memcpy(&pyro_config, cfg_payload, sizeof(PyroConfigData));
                    prefs.begin("pyro", false);
                    prefs.putBytes("cfg", &pyro_config, sizeof(pyro_config));
                    prefs.end();
                    ESP_LOGI(TAG, "[PYRO CFG] ch1: en=%u mode=%u val=%.1f  ch2: en=%u mode=%u val=%.1f",
                             pyro_config.ch1_enabled, pyro_config.ch1_trigger_mode,
                             (double)pyro_config.ch1_trigger_value,
                             pyro_config.ch2_enabled, pyro_config.ch2_trigger_mode,
                             (double)pyro_config.ch2_trigger_value);
                }
            }
            else if (out_pending_command == PYRO_CONT_TEST)
            {
                // Momentary arm → read continuity → disarm (ground test only)
                if (rocket_state == INFLIGHT) {
                    ESP_LOGW(TAG, "[PYRO CONT TEST] Rejected — INFLIGHT");
                } else {
                    delay(1);
                    uint8_t cfg_payload[4];
                    size_t  cfg_len = 0;
                    uint8_t ch = 0;
                    if (readConfigFrame(PYRO_CONT_TEST, 1,
                                        cfg_payload, sizeof(cfg_payload), cfg_len)
                        && cfg_len >= 1) {
                        ch = cfg_payload[0];
                    }
                    gpio_num_t arm_pin  = (ch == 2) ? (gpio_num_t)config::PYRO2_ARM_PIN
                                                    : (gpio_num_t)config::PYRO1_ARM_PIN;
                    gpio_num_t cont_pin = (ch == 2) ? (gpio_num_t)config::PYRO2_CONT_PIN
                                                    : (gpio_num_t)config::PYRO1_CONT_PIN;
                    uint8_t cont_bit    = (ch == 2) ? PSF_CH2_CONT : PSF_CH1_CONT;

                    // Reclaim pins from SPI2 (ESP32-P4 default SPI2 pins
                    // overlap with pyro pins 14-16).
                    esp_gpio_revoke(1ULL << arm_pin);
                    esp_gpio_revoke(1ULL << cont_pin);
                    gpio_reset_pin(arm_pin);
                    gpio_reset_pin(cont_pin);

                    gpio_config_t arm_cfg = {};
                    arm_cfg.pin_bit_mask = 1ULL << arm_pin;
                    arm_cfg.mode         = GPIO_MODE_INPUT_OUTPUT;
                    arm_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
                    arm_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
                    gpio_config(&arm_cfg);

                    gpio_config_t cont_cfg = {};
                    cont_cfg.pin_bit_mask = 1ULL << cont_pin;
                    cont_cfg.mode         = GPIO_MODE_INPUT;
                    cont_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
                    cont_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
                    gpio_config(&cont_cfg);

                    // Force GPIO matrix output to simple GPIO signal,
                    // overriding SPI2 peripheral signal on this pin.
                    esp_rom_gpio_connect_out_signal(arm_pin, SIG_GPIO_OUT_IDX, false, false);
                    gpio_iomux_out(arm_pin, 1, false);

                    // Arm, settle, read continuity, disarm (under spinlock for GPIO safety)
                    portENTER_CRITICAL(&pyro_spinlock);
                    gpio_set_level(arm_pin, 1);
                    portEXIT_CRITICAL(&pyro_spinlock);
                    delay(100);
                    int raw = gpio_get_level(cont_pin);
                    portENTER_CRITICAL(&pyro_spinlock);
                    gpio_set_level(arm_pin, 0);
                    portEXIT_CRITICAL(&pyro_spinlock);

                    // Continuity: 1 = load connected, 0 = open (verified with Arduino test)
                    non_sensor_data.pyro_status &= ~cont_bit;
                    if (raw == 1) non_sensor_data.pyro_status |= cont_bit;
                    ESP_LOGI(TAG, "[PYRO CONT TEST] CH%u raw=%d status=0x%02X",
                             ch, raw, non_sensor_data.pyro_status);
                }
            }
            else if (out_pending_command == PYRO_FIRE_TEST)
            {
                // Test-fire a pyro channel from the app (ground test only)
                if (rocket_state == INFLIGHT) {
                    ESP_LOGW(TAG, "[PYRO FIRE TEST] Rejected — INFLIGHT");
                } else {
                    delay(1);
                    uint8_t cfg_payload[4];
                    size_t  cfg_len = 0;
                    uint8_t ch = 0;
                    if (readConfigFrame(PYRO_FIRE_TEST, 1,
                                        cfg_payload, sizeof(cfg_payload), cfg_len)
                        && cfg_len >= 1) {
                        ch = cfg_payload[0];
                    }
                    gpio_num_t arm_pin  = (ch == 2) ? (gpio_num_t)config::PYRO2_ARM_PIN
                                                    : (gpio_num_t)config::PYRO1_ARM_PIN;
                    gpio_num_t fire_pin = (ch == 2) ? (gpio_num_t)config::PYRO2_FIRE_PIN
                                                    : (gpio_num_t)config::PYRO1_FIRE_PIN;
                    uint8_t fired_bit   = (ch == 2) ? PSF_CH2_FIRED : PSF_CH1_FIRED;

                    // Reclaim pins from SPI2
                    esp_gpio_revoke(1ULL << arm_pin);
                    esp_gpio_revoke(1ULL << fire_pin);
                    gpio_reset_pin(arm_pin);
                    gpio_reset_pin(fire_pin);

                    gpio_config_t pin_cfg = {};
                    pin_cfg.pin_bit_mask = (1ULL << arm_pin) | (1ULL << fire_pin);
                    pin_cfg.mode         = GPIO_MODE_INPUT_OUTPUT;
                    pin_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
                    pin_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
                    gpio_config(&pin_cfg);

                    esp_rom_gpio_connect_out_signal(arm_pin, SIG_GPIO_OUT_IDX, false, false);
                    gpio_iomux_out(arm_pin, 1, false);
                    esp_rom_gpio_connect_out_signal(fire_pin, SIG_GPIO_OUT_IDX, false, false);
                    gpio_iomux_out(fire_pin, 1, false);

                    // Arm → fire pulse → disarm
                    portENTER_CRITICAL(&pyro_spinlock);
                    gpio_set_level(arm_pin, 1);
                    gpio_set_level(fire_pin, 1);
                    portEXIT_CRITICAL(&pyro_spinlock);

                    delay(config::PYRO_FIRE_DURATION_MS);

                    portENTER_CRITICAL(&pyro_spinlock);
                    gpio_set_level(fire_pin, 0);
                    gpio_set_level(arm_pin, 0);
                    portEXIT_CRITICAL(&pyro_spinlock);

                    non_sensor_data.pyro_status |= fired_bit;
                    ESP_LOGI(TAG, "[PYRO FIRE TEST] CH%u fired for %u ms",
                             ch, (unsigned)config::PYRO_FIRE_DURATION_MS);
                }
            }
            else if (out_pending_command == SERVO_TEST_PENDING)
            {
                if (rocket_state == INFLIGHT) {
                    ESP_LOGW(TAG, "[SERVO TEST] Rejected — INFLIGHT");
                } else {
                    delay(1);
                    uint8_t cfg_payload[8];
                    size_t  cfg_len = 0;
                    if (readConfigFrame(SERVO_TEST_MSG, sizeof(ServoTestAnglesData),
                                        cfg_payload, sizeof(cfg_payload), cfg_len)
                        && cfg_len >= sizeof(ServoTestAnglesData))
                    {
                        ServoTestAnglesData angles;
                        memcpy(&angles, cfg_payload, sizeof(angles));
                        for (int i = 0; i < 4; ++i)
                        {
                            servo_test_angles[i] = angles.angle_cdeg[i] / 100.0f;
                        }
                        servo_test_active = true;
                        servo_control.setServoAngles(servo_test_angles);
                        ESP_LOGI(TAG, "[SERVO TEST] Angles: [%.1f, %.1f, %.1f, %.1f] deg",
                                      (double)servo_test_angles[0], (double)servo_test_angles[1],
                                      (double)servo_test_angles[2], (double)servo_test_angles[3]);
                    }
                    else
                    {
                        // Config not in this read — allow retry on next poll
                        last_processed_cmd = 0U;
                    }
                }
            }
            else if (out_pending_command == SERVO_TEST_STOP)
            {
                servo_test_active = false;
                servo_control.stowControl();
                ESP_LOGI(TAG, "[SERVO TEST] Stopped - servos stowed");
            }
            else if (out_pending_command == ROLL_PROFILE_PENDING)
            {
                delay(1);
                uint8_t cfg_payload[sizeof(RollProfileData)];
                size_t  cfg_len = 0;
                if (readConfigFrame(ROLL_PROFILE_MSG, sizeof(RollProfileData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(RollProfileData))
                {
                    memcpy(&roll_profile, cfg_payload, sizeof(RollProfileData));
                    // Clamp to valid range
                    if (roll_profile.num_waypoints > MAX_ROLL_WAYPOINTS)
                    {
                        roll_profile.num_waypoints = MAX_ROLL_WAYPOINTS;
                    }
                    // Persist to NVS
                    prefs.begin("rollp", false);
                    prefs.putBytes("prof", &roll_profile, sizeof(RollProfileData));
                    prefs.end();
                    ESP_LOGI(TAG, "[ROLL PROF] Set %d waypoints (saved to NVS)",
                                  roll_profile.num_waypoints);
                    for (uint8_t i = 0; i < roll_profile.num_waypoints; ++i)
                    {
                        ESP_LOGI(TAG, "  WP%d: t=%.1fs angle=%.1f°", i,
                                      (double)roll_profile.waypoints[i].time_s,
                                      (double)roll_profile.waypoints[i].angle_deg);
                    }
                }
                else
                {
                    last_processed_cmd = 0U;  // retry on next poll
                    ESP_LOGW(TAG, "[ROLL PROF] Config not in this read, will retry");
                }
            }
            else if (out_pending_command == ROLL_PROFILE_CLEAR)
            {
                roll_profile.num_waypoints = 0;
                prefs.begin("rollp", false);
                prefs.remove("prof");
                prefs.end();
                ESP_LOGI(TAG, "[ROLL PROF] Cleared (rate-only mode, saved to NVS)");
            }
            else if (out_pending_command == ROLL_CTRL_CONFIG_PENDING)
            {
                delay(1);
                uint8_t cfg_payload[4];
                size_t  cfg_len = 0;
                if (readConfigFrame(ROLL_CTRL_CONFIG_MSG, sizeof(RollControlConfigData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(RollControlConfigData))
                {
                    RollControlConfigData rc;
                    memcpy(&rc, cfg_payload, sizeof(rc));
                    use_angle_control = (rc.use_angle_control != 0);
                    roll_delay_ms     = rc.roll_delay_ms;
                    ESP_LOGI(TAG, "[ROLL CFG] angle_ctrl=%s delay=%u ms",
                                  use_angle_control ? "ON" : "OFF",
                                  (unsigned)roll_delay_ms);
                    prefs.begin("servo", false);
                    prefs.putBool("ac", use_angle_control);
                    prefs.putUShort("rdly", roll_delay_ms);
                    prefs.end();
                    ESP_LOGI(TAG, "[ROLL CFG] Saved to NVS");
                }
                else
                {
                    ESP_LOGW(TAG, "[ROLL CFG] readConfigFrame failed");
                }
            }
            else if (out_pending_command == SERVO_REPLAY_PENDING)
            {
                if (rocket_state == INFLIGHT) {
                    ESP_LOGW(TAG, "[SERVO REPLAY] Rejected — INFLIGHT");
                } else {
                    delay(1);
                    uint8_t cfg_payload[sizeof(ServoReplayData)];
                    size_t  cfg_len = 0;
                    if (readConfigFrame(SERVO_REPLAY_MSG, sizeof(ServoReplayData),
                                        cfg_payload, sizeof(cfg_payload), cfg_len)
                        && cfg_len >= sizeof(ServoReplayData))
                    {
                        ServoReplayData replay;
                        memcpy(&replay, cfg_payload, sizeof(replay));
                        float roll_rate = replay.roll_rate_cdps / 100.0f;
                        float speed     = replay.speed_cmps / 100.0f;

                        if (!servo_replay_active) {
                            servo_replay_active = true;
                            servo_control.resetPID();
                            ESP_LOGI(TAG, "[SERVO REPLAY] Started");
                        }
                        servo_control.controlWithGainSchedule(-roll_rate, speed);
                        ESP_LOGI(TAG, "[SERVO REPLAY] rate=%.1f dps, speed=%.1f m/s, cmd=%.2f deg",
                                      (double)roll_rate, (double)speed,
                                      (double)servo_control.getRollCmdDeg());
                    }
                    else
                    {
                        last_processed_cmd = 0U;  // retry on next poll
                    }
                }
            }
            else if (out_pending_command == SERVO_REPLAY_STOP)
            {
                servo_replay_active = false;
                servo_control.stowControl();
                servo_control.resetPID();
                ESP_LOGI(TAG, "[SERVO REPLAY] Stopped - servos stowed");
            }
            else
            {
                ESP_LOGW(TAG, "[I2C RX] Unknown pending command: 0x%02X", (unsigned)out_pending_command);
            }
            out_pending_command = 0U;
        }

        // Release exclusive bus access now that the query exchange and any
        // follow-up config reads (readConfigFrame) are complete.
        if (i2c_bus_mutex != nullptr && xSemaphoreGetMutexHolder(i2c_bus_mutex) == xTaskGetCurrentTaskHandle())
        {
            xSemaphoreGive(i2c_bus_mutex);
        }

        // Kinematic checks
        kinematics.kinematicChecks(pressure_altitude_m,
                                   accel_norm,
                                   imu_pos,
                                   imu_vel,
                                   roll_rate_dps,
                                   bmp_new_for_kf,
                                   (float)gnss_latest_si.alt,
                                   gps_new_for_kc,
                                   imu_rpy[1],
                                   burnout_detected,
                                   mach_locked_out);
        bmp_new_for_kf = false;
        gps_new_for_kc = false;

        pressure_alt_m = kinematics.alt_est;
        pressure_alt_rate_mps = kinematics.d_alt_est_;
        max_alt_m = kinematics.max_altitude;
        max_speed_mps = kinematics.max_speed;

        if (ground_test_active)
        {
            // Ground test behavior depends on control mode:
            //   Roll Only:      roll rate nulling only (same as boost phase)
            //   Roll+Guidance:  full attitude-hold + roll null (tests guidance pipeline)
            if (servo_enabled && have_ism6_si && ekf_initialized)
            {
                float pitch_fin = 0.0f;
                float yaw_fin   = 0.0f;

                if (guidance_enabled)
                {
                    // Full attitude-hold: tilt correction + roll null
                    float quat[4];
                    ekf.getQuaternion(quat);
                    float q0g = quat[0], q1g = quat[1];
                    float q2g = quat[2], q3g = quat[3];

                    float nose_north = 1.0f - 2.0f*(q2g*q2g + q3g*q3g);
                    float nose_east  = 2.0f*(q1g*q2g + q0g*q3g);

                    float K_tilt = config::GROUND_TEST_TILT_GAIN;
                    float a_cmd_n = -K_tilt * nose_north;
                    float a_cmd_e = -K_tilt * nose_east;
                    float a_cmd_ned[3] = { a_cmd_n, a_cmd_e, 0.0f };

                    float r00 = 1.0f - 2.0f*(q2g*q2g + q3g*q3g);
                    float r10 = 2.0f*(q1g*q2g + q0g*q3g);
                    float r20 = 2.0f*(q1g*q3g - q0g*q2g);
                    float r01 = 2.0f*(q1g*q2g - q0g*q3g);
                    float r11 = 1.0f - 2.0f*(q1g*q1g + q3g*q3g);
                    float r21 = 2.0f*(q2g*q3g + q0g*q1g);
                    float r02 = 2.0f*(q1g*q3g + q0g*q2g);
                    float r12 = 2.0f*(q2g*q3g - q0g*q1g);
                    float r22 = 1.0f - 2.0f*(q1g*q1g + q2g*q2g);

                    float a_body_right = r01*a_cmd_ned[0] + r11*a_cmd_ned[1] + r21*a_cmd_ned[2];
                    float a_body_down  = r02*a_cmd_ned[0] + r12*a_cmd_ned[1] + r22*a_cmd_ned[2];

                    float accel_to_fin = config::GROUND_TEST_ACCEL_TO_FIN;
                    pitch_fin = accel_to_fin * a_body_down;
                    yaw_fin   = accel_to_fin * a_body_right;
                }
                // else: Roll Only — pitch_fin and yaw_fin stay 0

                // Roll rate nulling (both modes).
                // Apply a deadband on the measurement before the PID: on the
                // bench the fins have zero aerodynamic authority, so the PID
                // reacting to gyro noise causes the servos to vibrate the
                // board, which amplifies the noise into a limit-cycle
                // oscillation. The deadband breaks that positive-feedback
                // loop. Real rolling motion is orders of magnitude larger
                // than the deadband, so legitimate response is unaffected.
                float gyro_x_db = roll_rate_dps;
                if (fabsf(gyro_x_db) < config::GROUND_TEST_ROLL_RATE_DEADBAND_DPS)
                    gyro_x_db = 0.0f;
                float roll_fin_cmd = roll_rate_pid_standalone.computePID(
                    config::ROLL_RATE_SET_POINT, -gyro_x_db);
                // Publish the standalone PID output for telemetry — the
                // servo_control path isn't used here so its cached roll_cmd
                // would stay zero otherwise.
                last_external_roll_cmd_deg = roll_fin_cmd;

                // 4-fin cruciform mixing
                float max_fin = config::PN_MAX_FIN_DEG;
                float deflections[4];
                deflections[0] = constrain(+pitch_fin + roll_fin_cmd, -max_fin, max_fin); // top
                deflections[1] = constrain(+yaw_fin   + roll_fin_cmd, -max_fin, max_fin); // right
                deflections[2] = constrain(-pitch_fin + roll_fin_cmd, -max_fin, max_fin); // bottom
                deflections[3] = constrain(-yaw_fin   + roll_fin_cmd, -max_fin, max_fin); // left
                servo_control.setServoAngles(deflections);

                // Periodic debug output (1 Hz)
                static uint32_t gt_last_print_ms = 0;
                if (now_ms - gt_last_print_ms >= 1000U) {
                    gt_last_print_ms = now_ms;
                    ESP_LOGI(TAG, "[GT] mode=%s pitch_fin=%.1f yaw_fin=%.1f roll_fin=%.1f",
                                  guidance_enabled ? "GUIDANCE" : "ROLL_ONLY",
                                  (double)pitch_fin, (double)yaw_fin, (double)roll_fin_cmd);
                }
            }
            else if (servo_enabled)
            {
                // EKF not yet initialized — hold neutral and log why
                servo_control.control(0.0f);
                static uint32_t gt_wait_print_ms = 0;
                if (now_ms - gt_wait_print_ms >= 2000U)
                {
                    gt_wait_print_ms = now_ms;
                    ESP_LOGW(TAG, "[GT] Waiting for EKF: ekf=%d imu=%d servo=%d",
                                  (int)ekf_initialized, (int)have_ism6_si, (int)servo_enabled);
                }
            }
        }
        else if (servo_test_active)
        {
            // Servo test: angles set via I2C command, skip state machine
        }
        else if (servo_replay_active)
        {
            // Servo replay: driven by I2C command handler, skip state machine
        }
        else
        switch (rocket_state)
        {
            case INITIALIZATION:
            {
                if ((now_ms > 1000U) && have_ism6_si && have_bmp_si)
                {
                    rocket_state = READY;
                    if (!ready_chirp_played)
                    {
                        startBootReadyChirp(now_ms, micros());
                        ready_chirp_played = true;
                    }
                    ESP_LOGI(TAG, "[STATE] INITIALIZATION -> READY");
                }
                break;
            }
            case READY:
            {
                const bool gnss_ready = gnss_started &&
                                        (now_ms > valid_gnss_start_millis + 3000U) &&
                                        have_gnss_si &&
                                        (gnss_latest_si.num_sats >= 4U);
                if (out_ready && gnss_ready)
                {
                    rocket_state = PRELAUNCH;
                    prelaunch_time_millis = now_ms;
                    // Camera is manually controlled via app — no auto-start on prelaunch
                    ESP_LOGI(TAG, "[STATE] READY -> PRELAUNCH");
                }
                break;
            }
            case PRELAUNCH:
            {
                if (kinematics.launch_flag)
                {
                    rocket_state = INFLIGHT;
                    launch_time_millis = now_ms;
                    // Safety: warn and disable guidance if EKF never initialized
                    if (!ekf_initialized) {
                        ESP_LOGW(TAG, "[EKF] WARNING: Launching without EKF initialization!");
                        if (guidance_enabled) {
                            ESP_LOGW(TAG, "[GUIDANCE] Auto-disabled — EKF not initialized");
                            guidance_enabled = false;
                        }
                    }
                    // Freeze the ENU reference position at launch
                    if (!ref_pos_frozen && have_ref_pos) {
                        ref_pos_frozen = true;
                        ESP_LOGI(TAG, "[EKF] Ref pos frozen (launch): n=%lu",
                                      (unsigned long)ref_pos_count);
                    }
                    ground_pressure_pa = have_bmp_si ? bmp_latest_si.pressure : ground_pressure_pa;
                    max_alt_m = 0.0f;
                    max_speed_mps = 0.0f;
                    landed_actions_done = false;
                    landed_candidate_start_millis = 0;
                    // Clear apogee/landing flags so they start clean at launch.
                    // launch_flag is intentionally preserved (it got us here).
                    kinematics.alt_apogee_flag = false;
                    kinematics.vel_u_apogee_flag = false;
                    kinematics.alt_landed_flag = false;
                    kinematics.max_speed = 0.0f;
                    // Reset guidance state for new flight
                    guidance.reset();
                    control_mixer.reset();
                    roll_rate_pid_standalone.reset();
                    burnout_detected = false;
                    burnout_time_ms = 0;
                    guidance_active = false;
                    // Reset and arm pyro channels on launch
                    pyro_apogee_detected = false;
                    pyro_apogee_time_ms = 0;
                    portENTER_CRITICAL(&pyro_spinlock);
                    pyro1_fired = false;
                    pyro2_fired = false;
                    pyro1_fire_start_ms = 0;
                    pyro2_fire_start_ms = 0;
                    portEXIT_CRITICAL(&pyro_spinlock);
                    pyroArmEnabled();
                    ESP_LOGI(TAG, "[STATE] PRELAUNCH -> INFLIGHT (ground_p=%.0f)",
                                  (double)ground_pressure_pa);
                }
                break;
            }
            case INFLIGHT:
            {
                // Logging is now triggered by OutComputer detecting NSF_LAUNCH
                // in NonSensorData — no need to send START_LOGGING over I2C.

                // Service pyro channels (check triggers, manage fire pulses)
                servicePyroChannels(now_ms);

                // Save flight snapshot at 10 Hz for reboot recovery
                if (now_ms - last_snapshot_ms >= 100U) {
                    last_snapshot_ms = now_ms;
                    saveFlightSnapshot(now_ms);
                }

                // Servo settling after reboot recovery — hold neutral until EKF stabilizes
                if (reboot_recovery && now_ms >= servo_settle_end_ms) {
                    reboot_recovery = false;  // keep PSF_REBOOT_RECOVERY flag in telemetry
                    ESP_LOGI(TAG, "[RECOVERY] Servo settle complete, control resumed");
                }

                if (servo_enabled)
                {
                    // Delay roll control activation after launch if configured
                    const uint32_t t_since_launch_ms = now_ms - launch_time_millis;
                    if (reboot_recovery || t_since_launch_ms < roll_delay_ms)
                    {
                        // Hold fins neutral until delay/settle elapses
                        servo_control.control(0.0f);
                    }
                    else if (have_ism6_si)
                    {
                        float speed = sqrtf(imu_vel[0]*imu_vel[0] +
                                            imu_vel[1]*imu_vel[1] +
                                            imu_vel[2]*imu_vel[2]);

                        // --- Burnout detection ---
                        // Body-X accel (forward axis in FRD) goes negative when
                        // decelerating after motor burnout.  Wait at least 200ms
                        // after launch to avoid false triggers from vibration.
                        if (!burnout_detected && t_since_launch_ms > 200)
                        {
                            float body_ax = ism6_latest_si.low_g_acc_x;
                            if (body_ax < 0.0f)
                            {
                                burnout_detected = true;
                                burnout_time_ms = now_ms;
                                ESP_LOGI(TAG, "[GUID] Burnout detected at T+%lu ms",
                                              (unsigned long)t_since_launch_ms);
                            }
                        }

                        // --- Guided coast mode (PN guidance) ---
                        // Active only during coast when guidance is enabled and EKF is valid.
                        // Stops at closest point of approach (CPA) or when speed drops.
                        const bool coast_guidance_active =
                            guidance_enabled &&
                            burnout_detected &&
                            ekf_initialized &&
                            (now_ms - burnout_time_ms >= config::PN_COAST_DELAY_MS) &&
                            (speed > config::PN_MIN_SPEED_MPS) &&
                            !guidance.isCpaReached();

                        if (coast_guidance_active)
                        {
                            // Get NED velocity for guidance (convert ENU back to NED)
                            float vel_ned_guid[3];
                            vel_ned_guid[0] = imu_vel[1];   // ENU North → NED North
                            vel_ned_guid[1] = imu_vel[0];   // ENU East  → NED East
                            vel_ned_guid[2] = -imu_vel[2];  // ENU Up    → NED Down

                            // Quaternion for guidance
                            float quat[4];
                            ekf.getQuaternion(quat);

                            // Guidance dt
                            static uint32_t prev_guidance_us = 0;
                            uint32_t now_us_guid = (uint32_t)esp_timer_get_time();
                            float guid_dt = (prev_guidance_us > 0)
                                          ? (float)(now_us_guid - prev_guidance_us) / 1e6f
                                          : 0.002f;
                            prev_guidance_us = now_us_guid;

                            // Run PN guidance → ENU acceleration commands
                            guidance.update(imu_pos, vel_ned_guid, guid_dt);

                            // Get ENU accel commands
                            float a_cmd_e = guidance.getAccelEastCmd();
                            float a_cmd_n = guidance.getAccelNorthCmd();
                            float a_cmd_u = guidance.getAccelUpCmd();

                            // Convert ENU accel → NED
                            float a_cmd_ned[3] = { a_cmd_n, a_cmd_e, -a_cmd_u };

                            // Rotate NED accel to body frame (FRD) using
                            // EKF quaternion (body-FRD to NED).
                            // R_b2ned = quat_to_dcm(q), a_body = R_b2ned^T @ a_ned
                            float q0 = quat[0], q1 = quat[1];
                            float q2 = quat[2], q3 = quat[3];
                            // DCM transpose rows = DCM columns (NED→body)
                            float r00 = 1.0f - 2.0f*(q2*q2 + q3*q3);
                            float r10 = 2.0f*(q1*q2 + q0*q3);
                            float r20 = 2.0f*(q1*q3 - q0*q2);
                            float r01 = 2.0f*(q1*q2 - q0*q3);
                            float r11 = 1.0f - 2.0f*(q1*q1 + q3*q3);
                            float r21 = 2.0f*(q2*q3 + q0*q1);
                            float r02 = 2.0f*(q1*q3 + q0*q2);
                            float r12 = 2.0f*(q2*q3 - q0*q1);
                            float r22 = 1.0f - 2.0f*(q1*q1 + q2*q2);
                            // a_body = R^T @ a_ned
                            float a_body_fwd   = r00*a_cmd_ned[0] + r10*a_cmd_ned[1] + r20*a_cmd_ned[2]; (void)a_body_fwd;
                            float a_body_right = r01*a_cmd_ned[0] + r11*a_cmd_ned[1] + r21*a_cmd_ned[2];
                            float a_body_down  = r02*a_cmd_ned[0] + r12*a_cmd_ned[1] + r22*a_cmd_ned[2];

                            // Body-frame accel → fin deflection (proportional)
                            // Pitch accel = body-Z (down in FRD), negate for nose-up
                            // Yaw accel = body-Y (right in FRD)
                            float pitch_accel = a_body_down;   // sign verified in sim
                            float yaw_accel   = a_body_right;

                            float accel_to_fin_deg = 4.0f;  // tuned in sim
                            float pitch_fin = accel_to_fin_deg * pitch_accel;
                            float yaw_fin   = accel_to_fin_deg * yaw_accel;

                            // Roll control: standalone PID
                            float roll_fin_cmd = roll_rate_pid_standalone.computePID(
                                config::ROLL_RATE_SET_POINT, -roll_rate_dps);

                            // Mix to 4 fins: pitch differential + yaw differential + roll common
                            float max_fin = config::PN_MAX_FIN_DEG;
                            float deflections[4];
                            deflections[0] = constrain(+pitch_fin + roll_fin_cmd, -max_fin, max_fin);  // top
                            deflections[1] = constrain(+yaw_fin   + roll_fin_cmd, -max_fin, max_fin);  // right
                            deflections[2] = constrain(-pitch_fin + roll_fin_cmd, -max_fin, max_fin);  // bottom
                            deflections[3] = constrain(-yaw_fin   + roll_fin_cmd, -max_fin, max_fin);  // left
                            servo_control.setServoAngles(deflections);
                            guidance_active = true;
                        }
                        else
                        {
                            // --- Roll-only mode (powered flight or guidance disabled) ---
                            guidance_active = false;

                            if (use_angle_control && ekf_initialized)
                            {
                                // Quaternion-based roll extraction (gimbal-lock-free).
                                float quat[4];
                                ekf.getQuaternion(quat);
                                float qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
                                float z_north = 2.0f * (qx*qz + qw*qy);
                                float z_east  = 2.0f * (qy*qz - qw*qx);
                                float actual_roll_deg = -atan2f(z_east, z_north) * (180.0f / (float)M_PI);

                                float t_flight = (float)(now_ms - launch_time_millis) / 1000.0f;
                                float target_roll_deg = roll_profile_interpolate(t_flight);

                                servo_control.controlAngle(target_roll_deg,
                                                           actual_roll_deg,
                                                           -roll_rate_dps,
                                                           speed,
                                                           config::KP_ANGLE);
                            }
                            else if (gain_sched_enabled)
                            {
                                servo_control.controlWithGainSchedule(-roll_rate_dps, speed);
                            }
                            else
                            {
                                servo_control.control(-roll_rate_dps);
                            }
                        }
                    }
                    else
                    {
                        // IMU data lost mid-flight — hold fins at neutral (0° command)
                        servo_control.control(0.0f);
                    }
                }
                const bool landing_conditions =
                    kinematics.alt_landed_flag && (fabsf(roll_rate_dps) < 30.0f);
                if (landing_conditions)
                {
                    if (landed_candidate_start_millis == 0U)
                    {
                        landed_candidate_start_millis = now_ms;
                    }
                    if (now_ms - landed_candidate_start_millis > 2000U)
                    {
                        rocket_state = LANDED;
                        ESP_LOGI(TAG, "[STATE] INFLIGHT -> LANDED");
                    }
                }
                else if (landed_candidate_start_millis != 0U &&
                         (now_ms - landed_candidate_start_millis > 2500U))
                {
                    // Only reset debounce timer if conditions have been false
                    // for >500ms beyond the 2s window — prevents single-frame
                    // noise from restarting the entire landing countdown.
                    landed_candidate_start_millis = 0U;
                }

                // Safety timeout: force LANDED if flight exceeds 10 minutes
                // (landing detection failure, stuck at altitude, KF divergence)
                static constexpr uint32_t MAX_FLIGHT_TIME_MS = 600000U;
                if (now_ms - launch_time_millis > MAX_FLIGHT_TIME_MS)
                {
                    pyroSafeAll();
                    rocket_state = LANDED;
                    ESP_LOGW(TAG, "[STATE] INFLIGHT -> LANDED (timeout)");
                }
                break;
            }
            case LANDED:
            {
                if (!landed_actions_done)
                {
                    landed_actions_done = true;
                    pyroSafeAll();
                    clearFlightSnapshot();  // prevent stale recovery on next boot
                    if (servo_enabled)
                    {
                        servo_control.stowControl();
                    }
                    // Reset guidance state
                    guidance.reset();
                    control_mixer.reset();
                    roll_rate_pid_standalone.reset();
                    guidance_active = false;
                    reboot_recovery = false;
                    reboot_recovery_telem = false;
                    cameraStop(now_ms);
                    if (!end_flight_sent)
                    {
                        if (enqueueI2STx(END_FLIGHT, nullptr, 0))
                        {
                            end_flight_sent = true;
                        }
                    }
                }
                break;
            }
            default:
            {
                break;
            }
        }

        // ---- Auto-return to READY after sim completes ----
        {
            const bool curr_sim_active = sensor_collector.isSimActive();
            if (prev_sim_active && !curr_sim_active)
            {
                // Sim just finished — reset state machine so user can run another sim.
                // Must also reset GNSS state: the sim injected synthetic GNSS
                // (fix=3, sats=12) which would otherwise cause an immediate
                // READY -> PRELAUNCH transition on stale data.
                pyroSafeAll();
                rocket_state = READY;
                ground_pressure_found = false;
                out_ready = false;
                end_flight_sent = false;
                landed_actions_done = false;
                landed_candidate_start_millis = 0;
                kinematics.reset();
                ekf_initialized = false;
                have_ref_pos = false;
                ref_pos_frozen = false;
                ref_lat_sum = ref_lon_sum = ref_alt_sum = 0.0;
                ref_pos_count = 0;
                ref_pos_first_time_ms = 0;
                last_gnss_time_us_for_ekf = 0;
                last_gnss_fix_second = 0xFF;
                last_gnss_fix_ms     = 0xFFFF;
                gnss_started = false;
                have_gnss_si = false;
                // Reset guidance state for next sim run
                guidance.reset();
                control_mixer.reset();
                roll_rate_pid_standalone.reset();
                burnout_detected = false;
                burnout_time_ms = 0;
                guidance_active = false;
                reboot_recovery = false;
                reboot_recovery_telem = false;
                clearFlightSnapshot();
                ESP_LOGI(TAG, "[STATE] Sim complete -> READY");
            }
            prev_sim_active = curr_sim_active;
        }

        // ---- Sim-mode diagnostic (once per second) ----
        if (sensor_collector.isSimActive())
        {
            static uint32_t sim_diag_ms = 0;
            if (now_ms - sim_diag_ms > 1000U)
            {
                sim_diag_ms = now_ms;
                ESP_LOGI(TAG, "[SIM DIAG] state=%d gnss_st=%d have_gnss=%d fix=%u sats=%u "
                              "acc=%.1f alt=%.1f d_alt=%.2f launch=%d out_rdy=%d",
                              (int)rocket_state,
                              gnss_started ? 1 : 0,
                              have_gnss_si ? 1 : 0,
                              (unsigned)gnss_latest_si.fix_mode,
                              (unsigned)gnss_latest_si.num_sats,
                              (double)accel_norm,
                              (double)pressure_alt_m,
                              (double)pressure_alt_rate_mps,
                              kinematics.launch_flag ? 1 : 0,
                              out_ready ? 1 : 0);
            }
        }

        // Publish non-sensor summary (SI -> packed) for downstream logging/telem.
        non_sensor_data.time_us = logic_now_us;
        if (ekf_initialized) {
            float imu_quat[4];
            ekf.getQuaternion(imu_quat);
            non_sensor_data.q0 = (int16_t)lroundf(imu_quat[0] * 10000.0f);
            non_sensor_data.q1 = (int16_t)lroundf(imu_quat[1] * 10000.0f);
            non_sensor_data.q2 = (int16_t)lroundf(imu_quat[2] * 10000.0f);
            non_sensor_data.q3 = (int16_t)lroundf(imu_quat[3] * 10000.0f);
            non_sensor_data.e_pos = (int32_t)lroundf(imu_pos[0] * 100.0f);
            non_sensor_data.n_pos = (int32_t)lroundf(imu_pos[1] * 100.0f);
            non_sensor_data.u_pos = (int32_t)lroundf(imu_pos[2] * 100.0f);
            non_sensor_data.e_vel = (int32_t)lroundf(imu_vel[0] * 100.0f);
            non_sensor_data.n_vel = (int32_t)lroundf(imu_vel[1] * 100.0f);
            non_sensor_data.u_vel = (int32_t)lroundf(imu_vel[2] * 100.0f);
        }
        // During ground test the servo_control internal PID is bypassed —
        // fins are driven via setServoAngles() — so publish the standalone
        // roll PID output instead. Otherwise fall back to servo_control's
        // cached command.
        {
            float rc_deg = 0.0f;
            if (servo_enabled)
            {
                rc_deg = ground_test_active
                       ? last_external_roll_cmd_deg
                       : servo_control.getRollCmdDeg();
            }
            non_sensor_data.roll_cmd = (int16_t)lroundf(rc_deg * 100.0f);
        }
        non_sensor_data.baro_alt_rate_dmps = (int16_t)lroundf(kinematics.d_alt_est_ * 10.0f);
        non_sensor_data.flags = 0;
        if (kinematics.alt_landed_flag || (rocket_state == LANDED)) non_sensor_data.flags |= NSF_ALT_LANDED;
        if (kinematics.alt_apogee_flag) non_sensor_data.flags |= NSF_ALT_APOGEE;
        if (kinematics.vel_u_apogee_flag) non_sensor_data.flags |= NSF_VEL_APOGEE;
        if (kinematics.launch_flag || (rocket_state == INFLIGHT)) non_sensor_data.flags |= NSF_LAUNCH;
        if (burnout_detected) non_sensor_data.flags |= NSF_BURNOUT;
        if (guidance_active)  non_sensor_data.flags |= NSF_GUIDANCE;
        // Snapshot pyro state under spinlock for telemetry
        portENTER_CRITICAL(&pyro_spinlock);
        bool p1_armed = pyro1_armed;
        bool p2_armed = pyro2_armed;
        bool p1_fired = pyro1_fired;
        bool p2_fired = pyro2_fired;
        portEXIT_CRITICAL(&pyro_spinlock);
        if (p1_armed) non_sensor_data.flags |= NSF_PYRO1_ARMED;
        if (p2_armed) non_sensor_data.flags |= NSF_PYRO2_ARMED;
        non_sensor_data.rocket_state = (uint8_t)rocket_state;
        // Pyro continuity: when armed (INFLIGHT), read live from GPIO.
        // When not armed, preserve bits set by PYRO_CONT_TEST command.
        // Continuity: GPIO 1 = load connected, 0 = open (verified with Arduino test).
        {
            uint8_t ps = non_sensor_data.pyro_status;
            // Clear armed-channel cont bits, then re-read live
            if (p1_armed) {
                ps &= ~PSF_CH1_CONT;
                if (gpio_get_level((gpio_num_t)config::PYRO1_CONT_PIN) == 1)
                    ps |= PSF_CH1_CONT;
            }
            if (p2_armed) {
                ps &= ~PSF_CH2_CONT;
                if (gpio_get_level((gpio_num_t)config::PYRO2_CONT_PIN) == 1)
                    ps |= PSF_CH2_CONT;
            }
            // Update fired bits
            if (p1_fired) ps |= PSF_CH1_FIRED;
            if (p2_fired) ps |= PSF_CH2_FIRED;
            if (reboot_recovery_telem) ps |= PSF_REBOOT_RECOVERY;
            // Always report the live guidance_enabled config so the
            // OutComputer can use it as the authoritative source of truth
            // (prevents iOS/OUT/FC NVS caches from silently diverging).
            ps &= ~PSF_GUIDANCE_ENABLED;
            if (guidance_enabled) ps |= PSF_GUIDANCE_ENABLED;
            non_sensor_data.pyro_status = ps;
        }

        if ((logic_now_us - last_non_sensor_tx_time_us) >= non_sensor_tx_period_us)
        {
            last_non_sensor_tx_time_us = logic_now_us;
            memcpy(non_sensor_data_buffer, &non_sensor_data, SIZE_OF_NON_SENSOR_DATA);
            (void)enqueueI2STx(NON_SENSOR_MSG,
                               non_sensor_data_buffer,
                               SIZE_OF_NON_SENSOR_DATA);

            // Send guidance telemetry at ~10 Hz during guided coast
            if (guidance_active)
            {
                static uint32_t guid_telem_counter = 0;
                if (++guid_telem_counter >= (config::NON_SENSOR_UPDATE_RATE / 10))
                {
                    guid_telem_counter = 0;
                    GuidanceTelemData gtd = {};
                    gtd.time_us = logic_now_us;
                    gtd.pitch_cmd_cdeg    = (int16_t)lroundf(guidance.getAccelNorthCmd() * 100.0f);
                    gtd.yaw_cmd_cdeg      = (int16_t)lroundf(guidance.getAccelEastCmd() * 100.0f);
                    gtd.lateral_offset_cm = (int16_t)lroundf(guidance.getLateralOffset() * 100.0f);
                    gtd.pitch_fin_cdeg    = (int16_t)lroundf(guidance.getLosAngleDeg() * 100.0f);
                    gtd.yaw_fin_cdeg      = (int16_t)lroundf(guidance.getClosingVelocity() * 100.0f);
                    gtd.guid_flags = (guidance_active ? 1u : 0u) | (burnout_detected ? 2u : 0u);
                    uint8_t gtd_buf[sizeof(GuidanceTelemData)];
                    memcpy(gtd_buf, &gtd, sizeof(GuidanceTelemData));
                    (void)enqueueI2STx(GUIDANCE_TELEM_MSG, gtd_buf, sizeof(GuidanceTelemData));
                }
            }
        }
    }
    
    const uint32_t now_ms_for_sound = millis();
    serviceBootReadyChirp(now_ms_for_sound, micros());
    serviceHeartbeatBeep(now_ms_for_sound);
    serviceBlueLedFlash(now_ms_for_sound);
    serviceGoProPulse(now_ms_for_sound);

    // --- Periodic poll-task timing diagnostics (once per second) ---
    {
        static uint32_t last_poll_diag_ms = 0;
        if ((now_ms_for_sound - last_poll_diag_ms) >= 1000U)
        {
            last_poll_diag_ms = now_ms_for_sound;
            PollTimingSnapshot pt = {};
            sensor_collector.getPollTimingSnapshot(pt);
            ESP_LOGI(TAG, "[GAP DIAG] iter_max=%lu gnss_max=%lu bmp_max=%lu mmc_max=%lu ism6_max=%lu us",
                          (unsigned long)pt.poll_iter_max_us,
                          (unsigned long)pt.gnss_max_us,
                          (unsigned long)pt.bmp_max_us,
                          (unsigned long)pt.mmc_max_us,
                          (unsigned long)pt.ism6_read_max_us);
            ESP_LOGI(TAG, "[GAP DIAG] gaps>10ms=%lu worst=%lu us | gnss calls=%lu >1ms=%lu >5ms=%lu >10ms=%lu",
                          (unsigned long)pt.gap_count,
                          (unsigned long)pt.gap_worst_us,
                          (unsigned long)pt.gnss_calls,
                          (unsigned long)pt.gnss_over_1ms,
                          (unsigned long)pt.gnss_over_5ms,
                          (unsigned long)pt.gnss_over_10ms);
            ESP_LOGI(TAG, "[GAP DIAG] i2s enqueue ok/drop=%lu/%lu tx ok/fail=%lu/%lu last_err=%d q_free=%u",
                          (unsigned long)i2s_tx_enqueue_ok,
                          (unsigned long)i2s_tx_enqueue_drop,
                          (unsigned long)i2s_tx_ok,
                          (unsigned long)i2s_tx_fail,
                          i2s_last_tx_err,
                          (unsigned)(i2s_tx_queue ? uxQueueSpacesAvailable(i2s_tx_queue) : 0));
            ESP_LOGI(TAG, "[GAP DIAG] getOutReady_max=%lu us  query ok/fail=%lu/%lu",
                          (unsigned long)i2c_gor_max_us,
                          (unsigned long)i2c_query_ok,
                          (unsigned long)i2c_query_fail);
            i2c_gor_max_us = 0;
            sensor_collector.resetPollTimingSnapshot();

            // Loop timing instrumentation
            if (lt_ekf_count > 0) {
                ESP_LOGI(TAG, "[TIMING] ekf: avg=%lu max=%lu us cnt=%lu | loop: cnt=%lu/s",
                              (unsigned long)(lt_ekf_total_us / lt_ekf_count),
                              (unsigned long)lt_ekf_max_us,
                              (unsigned long)lt_ekf_count,
                              (unsigned long)lt_loop_count);
            } else {
                ESP_LOGI(TAG, "[TIMING] ekf: not running | loop: cnt=%lu/s",
                              (unsigned long)lt_loop_count);
            }
            lt_ekf_total_us = 0;
            lt_ekf_max_us = 0;
            lt_ekf_count = 0;
            lt_loop_count = 0;

            // Sensor data flow diagnostic
            ESP_LOGI(TAG, "[SENSOR] reads/s: ism6=%lu bmp=%lu mmc=%lu iis=%lu gnss=%lu | bmp_p=%.0f ism6_gz=%.1f drdy_pin=%d",
                          (unsigned long)dbg_ism6_reads,
                          (unsigned long)dbg_bmp_reads,
                          (unsigned long)dbg_mmc_reads,
                          (unsigned long)dbg_iis2mdc_reads,
                          (unsigned long)dbg_gnss_reads,
                          have_bmp_si ? (double)bmp_latest_si.pressure : 0.0,
                          have_ism6_si ? (double)ism6_latest_si.gyro_z : 0.0,
                          gpio_get_level((gpio_num_t)config::ISM6HG256_INT));
            dbg_ism6_reads = 0;
            dbg_bmp_reads = 0;
            dbg_mmc_reads = 0;
            dbg_iis2mdc_reads = 0;
            dbg_gnss_reads = 0;

            // MMC5983MA-specific diagnostic — we've been seeing mmc=0 reads/s
            // with no visible cause from the [SENSOR] line alone. This surfaces
            // the internal counters already maintained by TR_Sensor_Collector
            // so we can tell where MMC data is getting stuck:
            //   isr      = data-ready interrupts fired (ideally ~200/s)
            //   proc     = ISR handler woke the task (should match isr)
            //   read_ok  = SPI readFieldsXYZ returned true
            //   read_fail= SPI read error
            //   clr_ok/f = interrupt-clear SPI ops
            //   lost     = stall-recovery tripped (sensor went silent)
            //   rec ok/a = recovery-reinit attempts / successes
            //   int_pin  = live state of MMC5983MA INT line
            {
                static MMC5983MADebugSnapshot prev_mmc_snap = {};
                MMC5983MADebugSnapshot now_mmc_snap;
                sensor_collector.getMMC5983MADebugSnapshot(now_mmc_snap);
                ESP_LOGI(TAG, "[MMC DIAG] isr=%lu proc=%lu read ok/fail=%lu/%lu clr ok/fail=%lu/%lu "
                              "lost=%lu rec %lu/%lu int_pin=%d",
                              (unsigned long)(now_mmc_snap.isr_hits         - prev_mmc_snap.isr_hits),
                              (unsigned long)(now_mmc_snap.process_hits     - prev_mmc_snap.process_hits),
                              (unsigned long)(now_mmc_snap.read_ok          - prev_mmc_snap.read_ok),
                              (unsigned long)(now_mmc_snap.read_fail        - prev_mmc_snap.read_fail),
                              (unsigned long)(now_mmc_snap.clear_ok         - prev_mmc_snap.clear_ok),
                              (unsigned long)(now_mmc_snap.clear_fail       - prev_mmc_snap.clear_fail),
                              (unsigned long)(now_mmc_snap.cmm_lost_hits    - prev_mmc_snap.cmm_lost_hits),
                              (unsigned long)(now_mmc_snap.recovery_success - prev_mmc_snap.recovery_success),
                              (unsigned long)(now_mmc_snap.recovery_attempts- prev_mmc_snap.recovery_attempts),
                              gpio_get_level((gpio_num_t)config::MMC5983MA_INT));
                prev_mmc_snap = now_mmc_snap;
            }
        }
    }

    // --- Periodic EKF diagnostics (once per second) ---
    {
        static uint32_t last_ekf_diag_ms = 0;
        if (ekf_initialized && (now_ms_for_sound - last_ekf_diag_ms) >= 1000U)
        {
            last_ekf_diag_ms = now_ms_for_sound;

            // Horizontal azimuth of the rocket's body-Z axis in NED. This is *not*
            // a roll in the Euler sense -- it's only a useful roll proxy when the
            // rocket is near vertical (body-Z ≈ world-Z, so its horizontal
            // projection rotates with vehicle roll about the vertical axis).
            // At low pitch this is dominated by yaw and will diverge from
            // euler_roll arbitrarily; that divergence is itself a useful informal
            // indicator of how close euler_roll is to gimbal lock.
            float quat[4];
            ekf.getQuaternion(quat);
            float qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
            float z_n = 2.0f * (qx * qz + qw * qy);
            float z_e = 2.0f * (qy * qz - qw * qx);
            float bodyZ_az_deg = -atan2f(z_e, z_n) * (180.0f / (float)M_PI);

            // Euler angles (erratic near pitch=90° due to gimbal lock)
            float rpy[3];
            ekf.getOrientEst(rpy);
            float euler_roll_deg = rpy[0] * (180.0f / (float)M_PI);
            float pitch_deg = rpy[1] * (180.0f / (float)M_PI);
            float yaw_deg = rpy[2] * (180.0f / (float)M_PI);

            // cos²(pitch) — magnetometer correction scale factor in Mahony AHRS
            float sp = sinf(rpy[1]);
            float cos2p = 1.0f - sp * sp;

            // Magnetometer field strength and validity (EKF gate: 15–80 µT)
            // Prefer IIS2MDC (new PCB), fall back to MMC5983MA (legacy).
            float mag_uT = 0.0f;
            const char* mag_status = "NONE";
            const char* mag_src = "-";
            if (have_iis2mdc_si) {
                float mx = (float)iis2mdc_latest_si.mag_x_uT;
                float my = (float)iis2mdc_latest_si.mag_y_uT;
                float mz = (float)iis2mdc_latest_si.mag_z_uT;
                mag_uT = sqrtf(mx * mx + my * my + mz * mz);
                mag_status = (mag_uT >= 15.0f && mag_uT <= 80.0f) ? "OK" : "REJ";
                mag_src = "IIS";
            } else if (have_mmc_si) {
                float mx = (float)mmc_latest_si.mag_x_uT;
                float my = (float)mmc_latest_si.mag_y_uT;
                float mz = (float)mmc_latest_si.mag_z_uT;
                mag_uT = sqrtf(mx * mx + my * my + mz * mz);
                mag_status = (mag_uT >= 15.0f && mag_uT <= 80.0f) ? "OK" : "REJ";
                mag_src = "MMC";
            }

            // Gyro bias estimate (rad/s → deg/s for readability)
            float gb[3];
            ekf.getRotRateBias(gb);

            ESP_LOGI(TAG, "[EKF DIAG] roll(euler)=%.1f roll(bodyZ_az)=%.1f pitch=%.1f yaw=%.1f cos2p=%.4f",
                          (double)euler_roll_deg,
                          (double)bodyZ_az_deg,
                          (double)pitch_deg,
                          (double)yaw_deg,
                          (double)cos2p);
            ESP_LOGI(TAG, "[EKF DIAG] mag[%s]=%.1fuT(%s) gyro_bias=[%.3f,%.3f,%.3f]dps",
                          mag_src,
                          (double)mag_uT,
                          mag_status,
                          (double)(gb[0] * 180.0f / (float)M_PI),
                          (double)(gb[1] * 180.0f / (float)M_PI),
                          (double)(gb[2] * 180.0f / (float)M_PI));
        }
    }

    // Give back remainder of time slice
    taskYIELD();
}

/* ── ESP-IDF entry point ─────────────────────────────────── */

extern "C" void app_main(void)
{
    setup_fc();
    TaskHandle_t flight_task = nullptr;
    xTaskCreatePinnedToCore(
        [](void *) {
            // Subscribe this task to WDT so esp_task_wdt_reset() works
            esp_task_wdt_add(xTaskGetCurrentTaskHandle());
            for (;;) { loop_fc(); esp_task_wdt_reset(); }
        },
        "flight", 16 * 1024, nullptr, configMAX_PRIORITIES - 1, &flight_task, 1);
}
