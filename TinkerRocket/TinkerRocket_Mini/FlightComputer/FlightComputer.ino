#include <Arduino.h>

// Configuration Parameters
#include "config.h"

// Libraries
#include <TR_Sensor_Collector.h>
#include <TR_Sensor_Collector_Sim.h>
#include <TR_I2C_Interface.h>
#include <TR_Sensor_Data_Converter.h>
#include <TR_GpsInsEKF.h>
#include <TR_KinematicChecks.h>
#include <TR_ServoControl_ledc_mult.h>
#include <RocketComputerTypes.h>
#include <Preferences.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// EKF timeUpdate()/measUpdate() allocate ~7.5 KB of temporary 15x15 matrices
// on the stack. The default Arduino loopTask stack (8 KB) is not enough.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

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
                                    config::GNSS_UPDATE_RATE,
                                    config::GNSS_RX,
                                    config::GNSS_TX,
                                    config::GNSS_RESET_N,
                                    config::GNSS_SAFEBOOT_N,
                                    config::USE_BMP585,
                                    config::USE_MMC5983MA,
                                    config::USE_GNSS,
                                    config::USE_ISM6HG256,
                                    SPI,
                                    config::SPI_SPEED);

// Sim wrapper: passthrough when sim inactive, replaces data when sim active
SensorCollectorSim sensor_collector(sensor_collector_hw);

// I2C inteface from Central ESP32 to Out ESP32
TR_I2C_Interface i2c_interface(config::ESP_I2C_PORT,
                               config::ESP_I2C_ADR);
SensorConverter sensor_converter;

static ISM6HG256Data ism6hg256_data;
static uint8_t ism6hg256_data_buffer[SIZE_OF_ISM6HG256_DATA];
static BMP585Data bmp585_data;
static uint8_t bmp585_data_buffer[SIZE_OF_BMP585_DATA];
static MMC5983MAData mmc5983ma_data;
static uint8_t mmc5983ma_data_buffer[SIZE_OF_MMC5983MA_DATA];
static GNSSData gnss_data;
static uint8_t gnss_data_buffer[SIZE_OF_GNSS_DATA];
static NonSensorData non_sensor_data;
static uint8_t non_sensor_data_buffer[SIZE_OF_NON_SENSOR_DATA];

typedef struct
{
    uint8_t type;
    uint8_t len;
    uint8_t payload[MAX_PAYLOAD];
} I2CTxMessage;

static QueueHandle_t i2c_tx_queue = nullptr;
static TaskHandle_t  i2c_sender_task_handle = nullptr;

// Cache for config frame bytes read in the same I2C transaction as the
// OUT_STATUS_RESPONSE.  The ESP32 I2C slave hardware FIFO discards bytes
// remaining after a master STOP, so both frames must be read in one shot.
static constexpr size_t COMBINED_READ_SIZE = 96;  // 10 (response) + up to 86 (config)
static uint8_t  cfg_read_cache[COMBINED_READ_SIZE];
static size_t   cfg_read_cache_len = 0;

// Flight loop parameters
uint32_t last_flight_loop_update_time = 0;
uint32_t flight_loop_period = (uint32_t)(1000000/config::FLIGHT_LOOP_UPDATE_RATE);
uint32_t last_non_sensor_tx_time_us = 0;
uint32_t non_sensor_tx_period_us = (uint32_t)(1000000UL / config::NON_SENSOR_UPDATE_RATE);

uint32_t i2c_tx_ok = 0;
uint32_t i2c_tx_fail = 0;
int i2c_last_tx_err = ESP_OK;
uint32_t i2c_query_ok = 0;
uint32_t i2c_query_fail = 0;
uint32_t i2c_gor_max_us = 0;  // peak getOutReady() duration per diagnostic window
uint32_t i2c_tx_ism6_ok = 0;
uint32_t i2c_tx_ism6_fail = 0;
uint32_t i2c_tx_bmp_ok = 0;
uint32_t i2c_tx_bmp_fail = 0;
uint32_t i2c_tx_mmc_ok = 0;
uint32_t i2c_tx_mmc_fail = 0;
uint32_t i2c_tx_gnss_ok = 0;
uint32_t i2c_tx_gnss_fail = 0;
uint32_t i2c_tx_ns_ok = 0;
uint32_t i2c_tx_ns_fail = 0;
uint32_t i2c_tx_enqueue_ok = 0;
uint32_t i2c_tx_enqueue_drop = 0;

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
static bool servo_enabled = false;
static bool gain_sched_enabled = config::GAIN_SCHEDULE_ENABLED;
static bool use_angle_control = config::USE_ANGLE_CONTROL;
static uint16_t roll_delay_ms = config::ROLL_CONTROL_DELAY_MS;
static bool ground_test_active = false;
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
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            if (cmd == nullptr)
            {
                Serial.printf("[CFG READ] attempt %d: cmd_link_create failed\n", attempt);
                delay(5);
                continue;
            }

            i2c_master_start(cmd);
            i2c_master_write_byte(cmd,
                                  static_cast<uint8_t>((config::ESP_I2C_ADR << 1) | I2C_MASTER_READ),
                                  true);
            if (actual_read > 1)
            {
                i2c_master_read(cmd, rx_buf, actual_read - 1, I2C_MASTER_ACK);
            }
            i2c_master_read_byte(cmd, &rx_buf[actual_read - 1], I2C_MASTER_NACK);
            i2c_master_stop(cmd);

            esp_err_t err = i2c_master_cmd_begin(config::ESP_I2C_PORT, cmd, pdMS_TO_TICKS(100));
            i2c_cmd_link_delete(cmd);
            if (err != ESP_OK)
            {
                Serial.printf("[CFG READ] attempt %d: i2c err=0x%X (%s)\n",
                              attempt, (unsigned)err, esp_err_to_name(err));
                delay(5 * (attempt + 1));
                continue;
            }
        }

        // Log raw header bytes for debugging
        Serial.printf("[CFG READ] attempt %d: raw[0..9]=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X (read=%u)%s\n",
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
                        Serial.printf("[CFG READ] found at SOF offset %u\n", (unsigned)off);
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
                    Serial.printf("[CFG READ] found via type+CRC scan at byte %u (SOF was lost)\n",
                                  (unsigned)off);
                    return true;
                }
            }
        }

        Serial.printf("[CFG READ] attempt %d: config frame (type=0x%02X) not found in %u bytes\n",
                      attempt, (unsigned)expected_type, (unsigned)actual_read);
        delay(5 * (attempt + 1));
    }

    payload_len_out = 0;
    return false;
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

static inline void i2cSendWithStats(uint8_t type, const uint8_t *payload, size_t len)
{
    const esp_err_t err = i2c_interface.sendMessage(type, payload, len);
    if (err == ESP_OK)
    {
        i2c_tx_ok++;
        switch (type)
        {
            case ISM6HG256_MSG: i2c_tx_ism6_ok++; break;
            case BMP585_MSG: i2c_tx_bmp_ok++; break;
            case MMC5983MA_MSG: i2c_tx_mmc_ok++; break;
            case GNSS_MSG: i2c_tx_gnss_ok++; break;
            case NON_SENSOR_MSG: i2c_tx_ns_ok++; break;
            default: break;
        }
    }
    else
    {
        i2c_tx_fail++;
        i2c_last_tx_err = (int)err;
        switch (type)
        {
            case ISM6HG256_MSG: i2c_tx_ism6_fail++; break;
            case BMP585_MSG: i2c_tx_bmp_fail++; break;
            case MMC5983MA_MSG: i2c_tx_mmc_fail++; break;
            case GNSS_MSG: i2c_tx_gnss_fail++; break;
            case NON_SENSOR_MSG: i2c_tx_ns_fail++; break;
            default: break;
        }
    }
}

static inline bool enqueueI2CTx(uint8_t type, const uint8_t *payload, size_t len)
{
    if (i2c_tx_queue == nullptr || len > MAX_PAYLOAD || len > 0xFF)
    {
        i2c_tx_enqueue_drop++;
        return false;
    }

    I2CTxMessage msg = {};
    msg.type = type;
    msg.len = (uint8_t)len;
    if (len > 0 && payload != nullptr)
    {
        memcpy(msg.payload, payload, len);
    }

    if (xQueueSend(i2c_tx_queue, &msg, 0) == pdTRUE)
    {
        i2c_tx_enqueue_ok++;
        return true;
    }

    i2c_tx_enqueue_drop++;
    return false;
}

static volatile bool i2c_sender_paused = false;

static void i2cSenderTask(void *)
{
    I2CTxMessage msg = {};
    for (;;)
    {
        if (xQueueReceive(i2c_tx_queue, &msg, portMAX_DELAY) == pdTRUE)
        {
            // Wait while paused — the main loop needs exclusive bus access
            // for the OUT_STATUS_QUERY exchange.
            while (i2c_sender_paused) { vTaskDelay(1); }
            i2cSendWithStats(msg.type, msg.payload, msg.len);
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

static inline void startGoProPulse(uint32_t now_ms)
{
    if ((config::CAM_SHUTTER_PIN < 0) || !config::USE_GOPRO)
    {
        return;
    }
    digitalWrite(config::CAM_SHUTTER_PIN, LOW);
    gopro_pulse_active = true;
    gopro_pulse_end_ms = now_ms + (uint32_t)config::GOPRO_PULSE_MS;
}

static inline void serviceGoProPulse(uint32_t now_ms)
{
    if (!gopro_pulse_active)
    {
        return;
    }
    if ((int32_t)(now_ms - gopro_pulse_end_ms) >= 0)
    {
        digitalWrite(config::CAM_SHUTTER_PIN, HIGH);
        gopro_pulse_active = false;
    }
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

void setup() 
{    
    Serial.begin(115200);
    const uint32_t serial_wait_start_ms = millis();
    while (!Serial && (millis() - serial_wait_start_ms < 1500U))
    {
        delay(10);
    }

    Serial.println("Starting ....");
    pinMode(config::RED_LED_PIN, OUTPUT);
    digitalWrite(config::RED_LED_PIN, HIGH);
    pinMode(config::BLUE_LED_PIN, OUTPUT);
    digitalWrite(config::BLUE_LED_PIN, LOW);

    // Disable all SPI chip-selects before bus init to avoid MISO contention.
    pinMode(config::MMC5983MA_CS, OUTPUT); digitalWrite(config::MMC5983MA_CS, HIGH);
    pinMode(config::BMP585_CS, OUTPUT);    digitalWrite(config::BMP585_CS, HIGH);
    pinMode(config::ISM6HG256_CS, OUTPUT); digitalWrite(config::ISM6HG256_CS, HIGH);
    delay(10);

    // SPI bus
    SPI.begin(config::SPI_SCK, config::SPI_SDO, config::SPI_SDI);
    delay(10);

    // Initialzie I2C interface to Out ESP32
    if (i2c_interface.beginMaster(config::ESP_SDA_PIN,
                                  config::ESP_SCL_PIN,
                                  config::ESP_I2C_FREQ_HZ,
                                  false,
                                  false) != ESP_OK)
    {
        Serial.println("Failed to initialize ESP I2C interface");
        while (1) { delay(1000); }
    }
    Serial.printf("I2C master on port=%d addr=0x%02X SDA=%d SCL=%d clk=%lu\n",
                  (int)config::ESP_I2C_PORT,
                  (unsigned)config::ESP_I2C_ADR,
                  (int)config::ESP_SDA_PIN,
                  (int)config::ESP_SCL_PIN,
                  (unsigned long)config::ESP_I2C_FREQ_HZ);
    i2c_tx_queue = xQueueCreate(config::I2C_TX_QUEUE_LEN, sizeof(I2CTxMessage));
    if (i2c_tx_queue == nullptr)
    {
        Serial.println("Failed to create I2C TX queue");
        while (1) { delay(1000); }
    }
    const uint8_t i2c_sender_core = (config::SENSOR_CORE == 0U) ? 1U : 0U;
    xTaskCreatePinnedToCore(i2cSenderTask,
                            "I2C Sender",
                            4096,
                            nullptr,
                            3,
                            &i2c_sender_task_handle,
                            i2c_sender_core);

    // Load persistent rocket settings from NVS (factory default from config.h)
    prefs.begin("rocket", true);  // read-only
    enable_sounds = prefs.getBool("sounds", config::ENABLE_SOUNDS);
    servo_enabled = prefs.getBool("servo_en", config::USE_SERVO_CONTROL);
    prefs.end();
    Serial.printf("NVS: enable_sounds=%s servo_enabled=%s\n",
                  enable_sounds ? "true" : "false",
                  servo_enabled ? "true" : "false");

    // Load servo/PID NVS settings (namespace "servo")
    int16_t nvs_servo_hz  = config::SERVO_HZ;
    int16_t nvs_servo_min = config::SERVO_MIN_US;
    int16_t nvs_servo_max = config::SERVO_MAX_US;
    bool nvs_servo_timing_changed = false;

    prefs.begin("servo", true);  // read-only
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
        Serial.printf("NVS servo: bias=[%d,%d,%d,%d] hz=%d min=%d max=%d\n",
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
        Serial.printf("NVS PID: kp=%.4f ki=%.4f kd=%.4f min=%.1f max=%.1f\n",
                      (double)kp, (double)ki, (double)kd,
                      (double)mincmd, (double)maxcmd);
    }
    gain_sched_enabled = prefs.getBool("gs", config::GAIN_SCHEDULE_ENABLED);
    use_angle_control  = prefs.getBool("ac", config::USE_ANGLE_CONTROL);
    roll_delay_ms      = prefs.getUShort("rdly", config::ROLL_CONTROL_DELAY_MS);
    prefs.end();
    Serial.printf("Gain scheduling: %s\n", gain_sched_enabled ? "ON" : "OFF");
    Serial.printf("Angle control: %s  Roll delay: %u ms\n",
                  use_angle_control ? "ON" : "OFF", (unsigned)roll_delay_ms);

    // Restore high-g accelerometer bias from NVS (namespace "cal")
    prefs.begin("cal", true);
    if (prefs.isKey("hgbx"))
    {
        float bx = prefs.getFloat("hgbx", 0.0f);
        float by = prefs.getFloat("hgby", 0.0f);
        float bz = prefs.getFloat("hgbz", 0.0f);
        sensor_converter.setHighGBias(bx, by, bz);
        out_status_query_data.hg_bias_x_cmss = (int16_t)lroundf(bx * 100.0f);
        out_status_query_data.hg_bias_y_cmss = (int16_t)lroundf(by * 100.0f);
        out_status_query_data.hg_bias_z_cmss = (int16_t)lroundf(bz * 100.0f);
        Serial.printf("NVS HG bias: %.3f, %.3f, %.3f m/s²\n",
                      (double)bx, (double)by, (double)bz);
    }
    prefs.end();

    // Load roll profile from NVS (namespace "rollp")
    prefs.begin("rollp", true);
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
            Serial.printf("NVS roll profile: %d waypoints\n", roll_profile.num_waypoints);
            for (uint8_t i = 0; i < roll_profile.num_waypoints; ++i)
            {
                Serial.printf("  WP%d: t=%.1fs angle=%.1f°\n", i,
                              (double)roll_profile.waypoints[i].time_s,
                              (double)roll_profile.waypoints[i].angle_deg);
            }
        }
        else
        {
            Serial.printf("NVS roll profile size mismatch (%u vs %u), ignoring\n",
                          (unsigned)prof_len, (unsigned)sizeof(RollProfileData));
        }
    }
    else
    {
        Serial.println("NVS roll profile: none (rate-only mode)");
    }
    prefs.end();

    // Always initialise piezo hardware so it's ready if enabled at runtime
    pinMode(config::PIEZO_PIN, OUTPUT);
    digitalWrite(config::PIEZO_PIN, LOW);
    piezo_pwm_ready = initPiezoTimer();
    if (!piezo_pwm_ready)
    {
        Serial.println("Piezo timer init failed; sounds disabled");
    }

    // Initialize sensor collector (including sensors) and start polling tasks
    sensor_collector.begin(config::SENSOR_CORE);
    sensor_converter.configureISM6HG256FullScale(
        static_cast<ISM6LowGFullScale>(config::ISM6_LOW_G_FS_G),
        static_cast<ISM6HighGFullScale>(config::ISM6_HIGH_G_FS_G),
        static_cast<ISM6GyroFullScale>(config::ISM6_GYRO_FS_DPS));
    sensor_converter.configureISM6HG256RotationZ(config::ISM6HG256_ROT_Z_DEG);
    sensor_converter.configureMMC5983MARotationZ(config::MMC5983MA_ROT_Z_DEG);
    sensor_collector.configureSimRotation(config::ISM6HG256_ROT_Z_DEG);

    out_status_query_data.ism6_low_g_fs_g = config::ISM6_LOW_G_FS_G;
    out_status_query_data.ism6_high_g_fs_g = config::ISM6_HIGH_G_FS_G;
    out_status_query_data.ism6_gyro_fs_dps = config::ISM6_GYRO_FS_DPS;
    out_status_query_data.ism6_rot_z_cdeg = (int16_t)lroundf(config::ISM6HG256_ROT_Z_DEG * 100.0f);
    out_status_query_data.mmc_rot_z_cdeg = (int16_t)lroundf(config::MMC5983MA_ROT_Z_DEG * 100.0f);
    out_status_query_data.format_version = 2;

    // Drain any stale data from OutComputer's I2C slave TX buffer.
    // After reflash the slave may have accumulated responses that were never
    // consumed.  Each drain read pulls up to 64 bytes; we stop after a short
    // timeout (buffer empty) or a fixed number of rounds.
    {
        uint8_t drain[64];
        int drained_total = 0;
        for (int dr = 0; dr < 8; dr++)
        {
            i2c_cmd_handle_t dc = i2c_cmd_link_create();
            if (!dc) break;
            i2c_master_start(dc);
            i2c_master_write_byte(dc,
                static_cast<uint8_t>((config::ESP_I2C_ADR << 1) | I2C_MASTER_READ), true);
            i2c_master_read(dc, drain, 63, I2C_MASTER_ACK);
            i2c_master_read_byte(dc, &drain[63], I2C_MASTER_NACK);
            i2c_master_stop(dc);
            esp_err_t de = i2c_master_cmd_begin(config::ESP_I2C_PORT, dc, pdMS_TO_TICKS(5));
            i2c_cmd_link_delete(dc);
            if (de != ESP_OK) break;
            drained_total += 64;
        }
        if (drained_total > 0)
        {
            Serial.printf("I2C TX buffer drain: %d bytes consumed\n", drained_total);
        }
    }
    delay(10); // let OutComputer settle after drain

    // Seed the first OUT_STATUS_QUERY so a response is ready for the main loop.
    // getOutReady() is now read-only, so we send the query explicitly here.
    (void)i2c_interface.sendMessage(OUT_STATUS_QUERY,
                                    reinterpret_cast<const uint8_t*>(&out_status_query_data),
                                    sizeof(out_status_query_data));
    delay(10); // let OutComputer process query and queue response
    {
        bool query_ok = i2c_interface.getOutReady(out_status_query_data, &out_pending_command);
        if (query_ok) { out_ready = true; i2c_query_ok++; } else { i2c_query_fail++; }
    }
    if (out_ready)
    {
        Serial.println("OUT ESP32 ready");
    }
    else
    {
        Serial.println("OUT ESP32 not ready yet (will latch on first success)");
    }

    // GoPro camera setup (power + shutter pin).
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

    // Servo setup — always init hardware if pins valid, gate enabled on NVS.
    if (servoPinsValid())
    {
        servo_control.begin();
        if (nvs_servo_timing_changed)
        {
            servo_control.setServoTiming(nvs_servo_hz, nvs_servo_min, nvs_servo_max);
            Serial.printf("Applied NVS servo timing: hz=%d min=%d max=%d\n",
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
        Serial.printf("Servo control %s, gain schedule %s (hardware ready)\n",
                      servo_enabled ? "enabled" : "disabled",
                      gain_sched_enabled ? "ON" : "OFF");
    }
    else
    {
        servo_enabled = false;
        Serial.println("Servo control disabled (set SERVO_PIN_* in config.h)");
    }

    Serial.println(F("Setup complete…"));
    triggerBlueLedFlash(millis());

}

// Loop is responsible for reading sensor data
void loop() 
{ 
    // ### Read and Send Sensor Data ###
    // Poll as fast as possible so high-rate sensor frames are not dropped by
    // loop-period gating.
    if (sensor_collector.getISM6HG256Data(ism6hg256_data))
    {
        sensor_converter.convertISM6HG256Data(ism6hg256_data, ism6_latest_si);
        have_ism6_si = true;

        memcpy(ism6hg256_data_buffer,
               &ism6hg256_data,
               SIZE_OF_ISM6HG256_DATA);

        (void)enqueueI2CTx(ISM6HG256_MSG,
                           ism6hg256_data_buffer,
                           SIZE_OF_ISM6HG256_DATA);
    }

    if (sensor_collector.getBMP585Data(bmp585_data))
    {
        sensor_converter.convertBMP585Data(bmp585_data, bmp_latest_si);
        have_bmp_si = true;
        bmp_new_for_kf = true;

        memcpy(bmp585_data_buffer,
               &bmp585_data,
               SIZE_OF_BMP585_DATA);

        (void)enqueueI2CTx(BMP585_MSG,
                           bmp585_data_buffer,
                           SIZE_OF_BMP585_DATA);
    }

    if (sensor_collector.getMMC5983MAData(mmc5983ma_data))
    {
        sensor_converter.convertMMC5983MAData(mmc5983ma_data, mmc_latest_si);
        have_mmc_si = true;

        memcpy(mmc5983ma_data_buffer,
               &mmc5983ma_data,
               SIZE_OF_MMC5983MA_DATA);

        (void)enqueueI2CTx(MMC5983MA_MSG,
                           mmc5983ma_data_buffer,
                           SIZE_OF_MMC5983MA_DATA);
    }

    if (sensor_collector.getGNSSData(gnss_data))
    {
        sensor_converter.convertGNSSData(gnss_data, gnss_latest_si);
        have_gnss_si = true;

        memcpy(gnss_data_buffer,
               &gnss_data,
               SIZE_OF_GNSS_DATA);

        (void)enqueueI2CTx(GNSS_MSG,
                           gnss_data_buffer,
                           SIZE_OF_GNSS_DATA);
    }

    // --- Flight logic update ---
    const uint32_t logic_now_us = micros();
    if ((logic_now_us - last_flight_loop_update_time) >= flight_loop_period)
    {
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
            EkfMagData ekf_mag = {};
            ekf_mag.time_us = have_mmc_si ? mmc_latest_si.time_us : 0;
            if (have_mmc_si) {
                ekf_mag.mag_x =  mmc_latest_si.mag_x_uT;
                ekf_mag.mag_y = -mmc_latest_si.mag_y_uT;   // FLU→FRD
                ekf_mag.mag_z = -mmc_latest_si.mag_z_uT;   // FLU→FRD
            }

            // ── Build EKF input: GNSS in LLA + NED ──
            // Only feed new GNSS data when we have a valid 3-D fix AND the
            // GPS fix timestamp (second + milli_second) differs from the
            // last fix we processed.  This prevents duplicate fixes from
            // being treated as independent measurements when the poll rate
            // exceeds the receiver's actual fix rate.
            EkfGNSSDataLLA ekf_gnss = {};
            const bool gnss_fix_is_new =
                (gnss_latest_si.second != last_gnss_fix_second) ||
                (gnss_latest_si.milli_second != last_gnss_fix_ms);
            if (have_gnss_si && gnss_latest_si.fix_mode >= 3U && gnss_fix_is_new)
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

                // Running average of GNSS fixes for launch-site reference.
                // Accumulate while on the pad; freeze at launch or after 2 min.
                if (!ref_pos_frozen) {
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
                        Serial.printf("[EKF] Ref pos frozen (2 min): n=%lu\n",
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
            if (!ekf_initialized)
            {
                if (have_ref_pos) {
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
                        Serial.printf("[EKF] Init: pitch=%.1f roll=%.1f heading=%.1f deg\n",
                                      (double)(pitch_rad * 180.0f / M_PI),
                                      (double)(roll_rad * 180.0f / M_PI),
                                      (double)config::PAD_HEADING_DEG);
                    }
                    ekf_initialized = true;
                }
            }
            else
            {
                // AHRS accel correction: disable during flight (high dynamics)
                const bool use_ahrs_acc = (rocket_state != INFLIGHT);
                ekf.update(use_ahrs_acc, ekf_imu, ekf_gnss, ekf_mag);

                // Barometer measurement update (when new BMP sample available)
                if (bmp_new_for_kf && have_bmp_si)
                {
                    EkfBaroData ekf_baro = {};
                    ekf_baro.time_us = bmp_latest_si.time_us;
                    // ISA pressure altitude relative to launch site
                    ekf_baro.altitude_m = (double)pressure_altitude_m;
                    ekf.baroMeasUpdate(ekf_baro);
                }
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
        // processed mid-flight and the query pauses the I2C sender for
        // ~8 ms, creating periodic data gaps.  Resumes once LANDED.
        if (rocket_state != INFLIGHT && (now_ms - out_ready_request_time_ms) > 250U)
        {
            out_ready_request_time_ms = now_ms;

            // Cooperatively pause the sender task: set the flag, then wait
            // for it to finish any in-progress transaction.
            i2c_sender_paused = true;
            delay(3);  // worst-case single I2C frame at 1.2 MHz ≈ 0.5 ms

            // Send the query directly (bus is now idle).
            esp_err_t send_err = i2c_interface.sendMessage(
                OUT_STATUS_QUERY,
                reinterpret_cast<const uint8_t*>(&out_status_query_data),
                sizeof(out_status_query_data),
                10);

            // Let OutComputer process the query and queue its response.
            delay(5);

            // Single large read captures BOTH the OUT_STATUS_RESPONSE and any
            // config frame that follows it.  Two separate reads don't work
            // because the ESP32 I2C slave hardware FIFO discards un-consumed
            // bytes when the master issues STOP.
            const uint32_t gor_start = micros();
            uint8_t combined_buf[COMBINED_READ_SIZE] = {};
            esp_err_t read_err = i2c_interface.masterRead(
                combined_buf, COMBINED_READ_SIZE, 10);

            bool query_ok = false;
            cfg_read_cache_len = 0;

            if (read_err == ESP_OK)
            {
                // Parse the response from the first 10 bytes.
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
                    // Cache remaining bytes for readConfigFrame().
                    memcpy(cfg_read_cache, combined_buf + 10,
                           COMBINED_READ_SIZE - 10);
                    cfg_read_cache_len = COMBINED_READ_SIZE - 10;
                }
            }

            const uint32_t gor_us = micros() - gor_start;
            if (gor_us > i2c_gor_max_us) { i2c_gor_max_us = gor_us; }
            if (query_ok) { i2c_query_ok++; } else { i2c_query_fail++; }

            Serial.printf("[I2C DBG] send=%s read=%s cmd=0x%02X dur=%lu us\n",
                          esp_err_to_name(send_err),
                          query_ok ? "OK" : "FAIL",
                          (unsigned)out_pending_command,
                          (unsigned long)gor_us);

            // NOTE: sender task stays paused through command processing below,
            // so readConfigFrame() I2C reads have an idle bus.  Resumed after
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
            Serial.printf("[I2C RX] pending_command=0x%02X\n", (unsigned)out_pending_command);
            if (out_pending_command == CAMERA_START)
            {
                if (config::USE_GOPRO && !gopro_recording)
                {
                    startGoProPulse(now_ms);
                    gopro_recording = true;
                }
            }
            else if (out_pending_command == CAMERA_STOP)
            {
                if (config::USE_GOPRO && gopro_recording)
                {
                    startGoProPulse(now_ms);
                    gopro_recording = false;
                }
            }
            else if (out_pending_command == SOUNDS_ENABLE)
            {
                enable_sounds = true;
                prefs.begin("rocket", false);  // read-write
                prefs.putBool("sounds", true);
                prefs.end();
                Serial.println("Sounds ENABLED (saved to NVS)");
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
                Serial.println("Sounds DISABLED (saved to NVS)");
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
                    Serial.printf("[SERVO CFG] bias=[%d,%d,%d,%d] hz=%d min=%d max=%d\n",
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
                        Serial.println("[SERVO CFG] Hz/min/max deferred (INFLIGHT)");
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
                    Serial.println("[SERVO CFG] Saved to NVS");
                }
                else
                {
                    last_processed_cmd = 0U;  // retry on next poll
                    Serial.println("[SERVO CFG] Config not in this read, will retry");
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
                    Serial.printf("[PID CFG] kp=%.4f ki=%.4f kd=%.4f min=%.1f max=%.1f\n",
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
                    Serial.println("[PID CFG] Saved to NVS");
                }
                else
                {
                    last_processed_cmd = 0U;  // retry on next poll
                    Serial.println("[PID CFG] Config not in this read, will retry");
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
                Serial.println("Servo control ENABLED (saved to NVS)");
            }
            else if (out_pending_command == SERVO_CTRL_DISABLE)
            {
                servo_enabled = false;
                servo_control.stowControl();
                prefs.begin("rocket", false);
                prefs.putBool("servo_en", false);
                prefs.end();
                Serial.println("Servo control DISABLED (saved to NVS)");
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
                    Serial.println("[SIM CFG] Config not in this read, will retry");
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
                        Serial.println("[SIM] Config frame recovered on start cmd");
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
                    Serial.println("[SIM] No config received, using defaults");
                }
                sensor_collector.startSim(ground_pressure_pa);
                Serial.printf("[SIM] Start cmd received, sim active=%s ground_p=%.0f\n",
                              sensor_collector.isSimActive() ? "YES" : "NO",
                              (double)ground_pressure_pa);
            }
            else if (out_pending_command == SIM_STOP_CMD)
            {
                sensor_collector.stopSim();
                Serial.println("[SIM] Stop cmd received");
            }
            else if (out_pending_command == GROUND_TEST_START)
            {
                if (rocket_state == INFLIGHT) {
                    Serial.println("[GROUND TEST] Rejected — INFLIGHT");
                } else {
                    ground_test_active = true;
                    Serial.println("[GROUND TEST] Started - servos responding to live sensors");
                }
            }
            else if (out_pending_command == GROUND_TEST_STOP)
            {
                ground_test_active = false;
                if (servo_enabled) servo_control.stowControl();
                Serial.println("[GROUND TEST] Stopped - servos stowed");
            }
            else if (out_pending_command == GYRO_CAL_CMD)
            {
                Serial.println("[CAL] Sensor calibration requested...");
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
                Serial.println("[CAL] Sensor calibration complete (saved to NVS)");
            }
            else if (out_pending_command == GAIN_SCHED_ENABLE)
            {
                gain_sched_enabled = true;
                servo_control.enableGainSchedule(config::GAIN_SCHEDULE_V_REF, config::GAIN_SCHEDULE_V_MIN);
                prefs.begin("servo", false);
                prefs.putBool("gs", true);
                prefs.end();
                Serial.println("[CFG] Gain scheduling ENABLED");
            }
            else if (out_pending_command == GAIN_SCHED_DISABLE)
            {
                gain_sched_enabled = false;
                servo_control.disableGainSchedule();
                prefs.begin("servo", false);
                prefs.putBool("gs", false);
                prefs.end();
                Serial.println("[CFG] Gain scheduling DISABLED");
            }
            else if (out_pending_command == SERVO_TEST_PENDING)
            {
                if (rocket_state == INFLIGHT) {
                    Serial.println("[SERVO TEST] Rejected — INFLIGHT");
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
                        Serial.printf("[SERVO TEST] Angles: [%.1f, %.1f, %.1f, %.1f] deg\n",
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
                Serial.println("[SERVO TEST] Stopped - servos stowed");
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
                    Serial.printf("[ROLL PROF] Set %d waypoints (saved to NVS)\n",
                                  roll_profile.num_waypoints);
                    for (uint8_t i = 0; i < roll_profile.num_waypoints; ++i)
                    {
                        Serial.printf("  WP%d: t=%.1fs angle=%.1f°\n", i,
                                      (double)roll_profile.waypoints[i].time_s,
                                      (double)roll_profile.waypoints[i].angle_deg);
                    }
                }
                else
                {
                    last_processed_cmd = 0U;  // retry on next poll
                    Serial.println("[ROLL PROF] Config not in this read, will retry");
                }
            }
            else if (out_pending_command == ROLL_PROFILE_CLEAR)
            {
                roll_profile.num_waypoints = 0;
                prefs.begin("rollp", false);
                prefs.remove("prof");
                prefs.end();
                Serial.println("[ROLL PROF] Cleared (rate-only mode, saved to NVS)");
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
                    Serial.printf("[ROLL CFG] angle_ctrl=%s delay=%u ms\n",
                                  use_angle_control ? "ON" : "OFF",
                                  (unsigned)roll_delay_ms);
                    prefs.begin("servo", false);
                    prefs.putBool("ac", use_angle_control);
                    prefs.putUShort("rdly", roll_delay_ms);
                    prefs.end();
                    Serial.println("[ROLL CFG] Saved to NVS");
                }
                else
                {
                    Serial.println("[ROLL CFG] readConfigFrame failed");
                }
            }
            else if (out_pending_command == SERVO_REPLAY_PENDING)
            {
                if (rocket_state == INFLIGHT) {
                    Serial.println("[SERVO REPLAY] Rejected — INFLIGHT");
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
                            Serial.println("[SERVO REPLAY] Started");
                        }
                        servo_control.controlWithGainSchedule(-roll_rate, speed);
                        Serial.printf("[SERVO REPLAY] rate=%.1f dps, speed=%.1f m/s, cmd=%.2f deg\n",
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
                Serial.println("[SERVO REPLAY] Stopped - servos stowed");
            }
            else
            {
                Serial.printf("[I2C RX] Unknown pending command: 0x%02X\n", (unsigned)out_pending_command);
            }
            out_pending_command = 0U;
        }

        // Resume the I2C sender task now that the query exchange and any
        // follow-up config reads (readConfigFrame) are complete.
        if (i2c_sender_paused)
        {
            i2c_sender_paused = false;
        }

        // Kinematic checks
        kinematics.kinematicChecks(pressure_altitude_m,
                                   accel_norm,
                                   imu_pos,
                                   imu_vel,
                                   roll_rate_dps,
                                   bmp_new_for_kf);
        bmp_new_for_kf = false;

        pressure_alt_m = kinematics.alt_est;
        pressure_alt_rate_mps = kinematics.d_alt_est_;
        max_alt_m = kinematics.max_altitude;
        max_speed_mps = kinematics.max_speed;

        if (ground_test_active)
        {
            // Ground test: run servo control with real sensor data, skip state machine
            if (servo_enabled && have_ism6_si)
            {
                if (gain_sched_enabled) {
                    servo_control.controlWithGainSchedule(-roll_rate_dps, 0.0f);
                } else {
                    servo_control.control(-roll_rate_dps);
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
                    Serial.println("[STATE] INITIALIZATION -> READY");
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
                    if (config::USE_GOPRO && !gopro_recording)
                    {
                        startGoProPulse(now_ms);
                        gopro_recording = true;
                    }
                    Serial.println("[STATE] READY -> PRELAUNCH");
                }
                break;
            }
            case PRELAUNCH:
            {
                if (kinematics.launch_flag)
                {
                    rocket_state = INFLIGHT;
                    launch_time_millis = now_ms;
                    // Freeze the ENU reference position at launch
                    if (!ref_pos_frozen && have_ref_pos) {
                        ref_pos_frozen = true;
                        Serial.printf("[EKF] Ref pos frozen (launch): n=%lu\n",
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
                    Serial.printf("[STATE] PRELAUNCH -> INFLIGHT (ground_p=%.0f)\n",
                                  (double)ground_pressure_pa);
                }
                break;
            }
            case INFLIGHT:
            {
                // Logging is now triggered by OutComputer detecting NSF_LAUNCH
                // in NonSensorData — no need to send START_LOGGING over I2C.

                if (servo_enabled)
                {
                    // Delay roll control activation after launch if configured
                    const uint32_t t_since_launch_ms = now_ms - launch_time_millis;
                    if (t_since_launch_ms < roll_delay_ms)
                    {
                        // Hold fins neutral until delay elapses
                        servo_control.control(0.0f);
                    }
                    else if (have_ism6_si)
                    {
                        float speed = sqrtf(imu_vel[0]*imu_vel[0] +
                                            imu_vel[1]*imu_vel[1] +
                                            imu_vel[2]*imu_vel[2]);

                        if (use_angle_control && ekf_initialized)
                        {
                            // Quaternion-based roll extraction (gimbal-lock-free).
                            // Computes azimuth of body Z-axis in NED horizontal plane.
                            float quat[4];
                            ekf.getQuaternion(quat);
                            float qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
                            float z_north = 2.0f * (qx*qz + qw*qy);
                            float z_east  = 2.0f * (qy*qz - qw*qx);
                            float actual_roll_deg = -atan2f(z_east, z_north) * (180.0f / (float)M_PI);

                            // Roll profile interpolation
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
                        Serial.println("[STATE] INFLIGHT -> LANDED");
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
                    rocket_state = LANDED;
                    Serial.println("[STATE] INFLIGHT -> LANDED (timeout)");
                }
                break;
            }
            case LANDED:
            {
                if (!landed_actions_done)
                {
                    landed_actions_done = true;
                    if (servo_enabled)
                    {
                        servo_control.stowControl();
                    }
                    if (config::USE_GOPRO && gopro_recording)
                    {
                        startGoProPulse(now_ms);
                        gopro_recording = false;
                    }
                    if (!end_flight_sent)
                    {
                        if (enqueueI2CTx(END_FLIGHT, nullptr, 0))
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
                Serial.println("[STATE] Sim complete -> READY");
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
                Serial.printf("[SIM DIAG] state=%d gnss_st=%d have_gnss=%d fix=%u sats=%u "
                              "acc=%.1f alt=%.1f d_alt=%.2f launch=%d out_rdy=%d\n",
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
        non_sensor_data.roll_cmd = servo_enabled
                                 ? (int16_t)lroundf(servo_control.getRollCmdDeg() * 100.0f)
                                 : 0;
        non_sensor_data.baro_alt_rate_dmps = (int16_t)lroundf(kinematics.d_alt_est_ * 10.0f);
        non_sensor_data.flags = 0;
        if (kinematics.alt_landed_flag || (rocket_state == LANDED)) non_sensor_data.flags |= NSF_ALT_LANDED;
        if (kinematics.alt_apogee_flag) non_sensor_data.flags |= NSF_ALT_APOGEE;
        if (kinematics.vel_u_apogee_flag) non_sensor_data.flags |= NSF_VEL_APOGEE;
        if (kinematics.launch_flag || (rocket_state == INFLIGHT)) non_sensor_data.flags |= NSF_LAUNCH;
        non_sensor_data.rocket_state = (uint8_t)rocket_state;

        if ((logic_now_us - last_non_sensor_tx_time_us) >= non_sensor_tx_period_us)
        {
            last_non_sensor_tx_time_us = logic_now_us;
            memcpy(non_sensor_data_buffer, &non_sensor_data, SIZE_OF_NON_SENSOR_DATA);
            (void)enqueueI2CTx(NON_SENSOR_MSG,
                               non_sensor_data_buffer,
                               SIZE_OF_NON_SENSOR_DATA);
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
            Serial.printf("[GAP DIAG] iter_max=%lu gnss_max=%lu bmp_max=%lu mmc_max=%lu ism6_max=%lu us\n",
                          (unsigned long)pt.poll_iter_max_us,
                          (unsigned long)pt.gnss_max_us,
                          (unsigned long)pt.bmp_max_us,
                          (unsigned long)pt.mmc_max_us,
                          (unsigned long)pt.ism6_read_max_us);
            Serial.printf("[GAP DIAG] gaps>10ms=%lu worst=%lu us | gnss calls=%lu >1ms=%lu >5ms=%lu >10ms=%lu\n",
                          (unsigned long)pt.gap_count,
                          (unsigned long)pt.gap_worst_us,
                          (unsigned long)pt.gnss_calls,
                          (unsigned long)pt.gnss_over_1ms,
                          (unsigned long)pt.gnss_over_5ms,
                          (unsigned long)pt.gnss_over_10ms);
            Serial.printf("[GAP DIAG] i2c enqueue ok/drop=%lu/%lu tx ok/fail=%lu/%lu last_err=%d q_free=%u\n",
                          (unsigned long)i2c_tx_enqueue_ok,
                          (unsigned long)i2c_tx_enqueue_drop,
                          (unsigned long)i2c_tx_ok,
                          (unsigned long)i2c_tx_fail,
                          i2c_last_tx_err,
                          (unsigned)(i2c_tx_queue ? uxQueueSpacesAvailable(i2c_tx_queue) : 0));
            Serial.printf("[GAP DIAG] getOutReady_max=%lu us  query ok/fail=%lu/%lu\n",
                          (unsigned long)i2c_gor_max_us,
                          (unsigned long)i2c_query_ok,
                          (unsigned long)i2c_query_fail);
            i2c_gor_max_us = 0;
            sensor_collector.resetPollTimingSnapshot();
        }
    }

    // --- Periodic EKF diagnostics (once per second) ---
    {
        static uint32_t last_ekf_diag_ms = 0;
        if (ekf_initialized && (now_ms_for_sound - last_ekf_diag_ms) >= 1000U)
        {
            last_ekf_diag_ms = now_ms_for_sound;

            // Quaternion-based roll (gimbal-lock-free): azimuth of body-Z in NED
            float quat[4];
            ekf.getQuaternion(quat);
            float qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
            float z_n = 2.0f * (qx * qz + qw * qy);
            float z_e = 2.0f * (qy * qz - qw * qx);
            float quat_roll_deg = -atan2f(z_e, z_n) * (180.0f / (float)M_PI);

            // Euler angles (erratic near pitch=90° due to gimbal lock)
            float rpy[3];
            ekf.getOrientEst(rpy);
            float euler_roll_deg = rpy[0] * (180.0f / (float)M_PI);
            float pitch_deg = rpy[1] * (180.0f / (float)M_PI);
            float yaw_deg = rpy[2] * (180.0f / (float)M_PI);

            // cos²(pitch) — magnetometer correction scale factor in Mahony AHRS
            float sp = sinf(rpy[1]);
            float cos2p = 1.0f - sp * sp;

            // Magnetometer field strength and validity (EKF rejects <15 or >80 µT)
            float mag_uT = 0.0f;
            const char* mag_status = "NONE";
            if (have_mmc_si) {
                float mx = (float)mmc_latest_si.mag_x_uT;
                float my = (float)mmc_latest_si.mag_y_uT;
                float mz = (float)mmc_latest_si.mag_z_uT;
                mag_uT = sqrtf(mx * mx + my * my + mz * mz);
                mag_status = (mag_uT >= 15.0f && mag_uT <= 80.0f) ? "OK" : "REJ";
            }

            // Gyro bias estimate (rad/s → deg/s for readability)
            float gb[3];
            ekf.getRotRateBias(gb);

            Serial.printf("[EKF DIAG] quat_roll=%.1f euler_roll=%.1f pitch=%.1f yaw=%.1f cos2p=%.4f\n",
                          (double)quat_roll_deg,
                          (double)euler_roll_deg,
                          (double)pitch_deg,
                          (double)yaw_deg,
                          (double)cos2p);
            Serial.printf("[EKF DIAG] mag=%.1fuT(%s) gyro_bias=[%.3f,%.3f,%.3f]dps\n",
                          (double)mag_uT,
                          mag_status,
                          (double)(gb[0] * 180.0f / (float)M_PI),
                          (double)(gb[1] * 180.0f / (float)M_PI),
                          (double)(gb[2] * 180.0f / (float)M_PI));
        }
    }

    // Give back remainder of time slide to same or lower
    // priority tasks
    taskYIELD();
}
