// ==========================================================================
// SECTION: Includes, board selection, and compile-time configuration
// ==========================================================================
#include <TR_NVS.h>

// Configuration Parameters
#include "config.h"
#include "baro_gate_policy.h"

// Libraries
#include <TR_Sensor_Collector.h>
#include <TR_Sensor_Collector_Sim.h>
#include <TR_I2C_Interface.h>
#include <TR_I2S_Stream.h>
#include <TR_Sensor_Data_Converter.h>
#include <TR_Orientation.h>
#include <TR_GpsInsEKF.h>
#include <TR_GeoMag.h>
#include <TR_KinematicChecks.h>
#include <BurnoutDetector.h>      // shared burnout detector (#197/#256)
#include <DeploymentDetector.h>   // shared recovery-deployment detector
#include <TR_MagCalibrator.h>
#include <TR_ServoControl_ledc_mult.h>
#include <TR_GuidancePN.h>
#include <TR_ControlMixer.h>
#include <RocketComputerTypes.h>
#include <GuidancePointGate.h>    // #435: pure cmd-28 acceptance gate (host-tested)
#include <TR_OTA_Receiver.h>      // FC-side OTA relay receiver (#8 Phase 4)
#include <TR_OTA_Backend_esp.h>
#include <esp_system.h>           // esp_restart after a verified relayed OTA
#include <driver/spi_master.h>
#include <esp_timer.h>
#include <esp_ota_ops.h>          // Layer 4: esp_ota_mark_app_valid_cancel_rollback (#8/#13)
#include <esp_app_desc.h>         // esp_app_get_description() for FC_IDENTITY version push (#8)
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_private/esp_gpio_reserve.h>
#include <esp_private/gpio.h>      // gpio_func_sel
#include <rom/gpio.h>              // esp_rom_gpio_connect_out_signal
#include <soc/gpio_sig_map.h>      // SIG_GPIO_OUT_IDX
#include <soc/gpio_sig_map.h>      // SIG_GPIO_OUT_IDX
#include <soc/io_mux_reg.h>        // PIN_FUNC_GPIO
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <nvs_flash.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_rom_sys.h>           // esp_rom_delay_us
#include <CRC32.h>
static const char* TAG = "FC";

// IDF-native timing helpers — replace the Arduino-shim time_ms()/time_us()
// that used to come in via compat.h.  Returned as uint32_t to keep
// arithmetic width unchanged at the (many) call sites that compare
// against a stashed timestamp.  Named with the `time_*()` prefix so
// they don't collide with existing parameters and locals called `now_ms`
// / `now_us` throughout this file.
static inline uint32_t time_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }
static inline uint32_t time_us() { return (uint32_t)esp_timer_get_time(); }
// `delay_ms(N)` (Arduino) → `delay_ms(N)` here, wraps vTaskDelay.  Keeps
// existing single-line call-site form; pdMS_TO_TICKS handles the
// configTICK_RATE_HZ conversion.
static inline void     delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// Firmware revision recorded in the flight settings snapshot (#165).
// Injected by CMake (git rev-parse / git diff); fall back when building
// outside the IDF build (e.g. host tests) so main.cpp stays compilable.
#ifndef FW_GIT_SHA
#define FW_GIT_SHA "unknown"
#endif
#ifndef FW_GIT_DIRTY
#define FW_GIT_DIRTY 0
#endif

// EKF timeUpdate()/measUpdate() allocate ~7.5 KB of temporary 15x15 matrices
// on the stack.  The FreeRTOS task in app_main() is created with 16 KB.

// ==========================================================================
// SECTION: Sensor collector and hardware objects
// ==========================================================================
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
                                    config::SPI_SPEED,
                                    config::ISM6_LOW_G_FS_G,
                                    config::ISM6_HIGH_G_FS_G,
                                    config::ISM6_GYRO_FS_DPS);

// Sim wrapper: passthrough when sim inactive, replaces data when sim active
SensorCollectorSim sensor_collector(sensor_collector_hw);

// I2C interface for command/config channel to OutComputer
TR_I2C_Interface i2c_interface(config::ESP_I2C_ADR);

// I2S stream for high-frequency telemetry to OutComputer
static TR_I2S_Stream i2s_stream;
SensorConverter sensor_converter;

// Hard-iron mag calibration (issue #96).  Runs only while rocket_state ==
// MAG_CALIBRATION; pad cal flow only, gated against any in-air state.
static MagCalibrator mag_calibrator;
static int64_t mag_cal_last_status_us = 0;
static bool    mag_cal_status_dirty   = false;  // true → publish on next tick regardless of cadence

// IIS2MDC OFFSET regs are zeroed by softReset() inside sensor_collector.begin(),
// so a calibrated boot has to defer the chip-side write until after begin()
// completes.  Loaded from NVS in setup(); applied after sensor_collector.begin().
static int16_t pending_mag_cx = 0, pending_mag_cy = 0, pending_mag_cz = 0;
static bool    pending_mag_apply = false;

// Mag-cal session state.
//
// mag_cal_session_active: true between MAG_CAL_START and either a
// successful verify-pass (= APPLIED, NVS written) or any abort path
// (verify-fail-then-abort, plain abort, retry-back-to-sampling, etc).
// Drives the prior-OFFSET cache lifetime: we snapshot the currently-
// persisted offsets at session start so any abort restores them, and
// zero the chip during the session so SAMPLING sees raw mag values
// (otherwise a recalibration would fit the *residual* hard-iron, then
// overwrite the prior full cal with that partial value and immediately
// fail verification — observed by user 2026-05-26).
//
// mag_cal_verify_active: true between MAG_CAL_ACCEPT and the next
// evaluateVerify() — drives the timer-based polling block.
static int64_t mag_cal_verify_start_us = 0;
// #382: explicit "evaluate on the next tick" request from MAG_CAL_VERIFY_DONE.
// The old signal was mag_cal_verify_start_us = 0, relying on (now - 0) >= 60 s
// — which is FALSE for the first 60 s of uptime, so a Done tap during a
// right-after-boot bench cal silently did nothing until the safety timeout.
static bool    mag_cal_verify_eval_now = false;
static int16_t mag_cal_prior_cx = 0, mag_cal_prior_cy = 0, mag_cal_prior_cz = 0;
static bool    mag_cal_session_active = false;
static bool    mag_cal_verify_active  = false;
// Safety timeout for the verify window — was originally a 5 s auto-
// dispatch but #148 turned verify into a user-driven step (the iOS
// "Done verifying" button sends MAG_CAL_VERIFY_DONE).  60 s is a
// safety only: if the user walks away without tapping anything, the
// FC eventually evaluates so the chip isn't left with unverified
// offsets indefinitely.
static constexpr int64_t MAG_CAL_VERIFY_DURATION_US = 60'000'000;

// Gyro zero-rate bias restored from NVS (#132).  Applied to sensor_collector
// after begin(), alongside the mag offsets, so a stored sensor cal survives
// reboots without the app connected.
static int16_t pending_gyro_cal_x = 0, pending_gyro_cal_y = 0, pending_gyro_cal_z = 0;
static bool    pending_gyro_cal_apply = false;

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

// ==========================================================================
// SECTION: I2C link to the Out Computer
// ==========================================================================
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
// Free-running EKF update-tick counter (#529) — unlike the lt_* diagnostics
// above it is never reset.  Published as NonSensorData.ekf_ticks (uint16 wrap)
// so the flight log records the achieved EKF cadence for the replay tool.
static uint32_t ekf_tick_counter = 0;

// --- Converted SI values (latest sample of each sensor) ---
static ISM6HG256DataSI ism6_latest_si = {};
static BMP585DataSI bmp_latest_si = {};
// #260: source-validity bounds for BMP585 pressure (Pa).  Wide enough to pass any
// real flight pressure; because they're finite bounds, the same comparison also
// rejects NaN and +/-Inf (which fail every finite compare).  Downstream trust
// gates (kinematics/scorecard) use a tighter 25-125 kPa window.
static constexpr float BMP_PRESSURE_MIN_PA = 1000.0f;
static constexpr float BMP_PRESSURE_MAX_PA = 120000.0f;
static MMC5983MADataSI mmc_latest_si = {};
static GNSSDataSI gnss_latest_si = {};
static bool have_ism6_si = false;
static bool have_bmp_si = false;
static bool have_mmc_si = false;
static bool have_gnss_si = false;
static bool bmp_new_for_kf = false; // Set when new BMP sample arrives, cleared at loop end.
                                    // Consumed by kinematicChecks (runs every iteration).
                                    // The EKF baro gate deliberately does NOT use it: it
                                    // would miss samples read on EKF-off decimation ticks
                                    // (#450) — see BaroGatePolicy.

// --- Flight logic state ---
static RocketState rocket_state = INITIALIZATION;
// #317: latched true once a flight reaches LANDED. Makes LANDED terminal until a
// hardware reboot — no re-arm, no second flight cycle, no re-armed pyros, and the
// OC won't open a new log. Matches the real-world use case (you don't re-fly
// immediately) and prevents ever re-arming anything dangerous unexpectedly.
// Cleared only at boot (static init) and on sim-completion re-arm.
static bool post_flight_lockout = false;

// ==========================================================================
// SECTION: Command lockout and flight-state flags
// ==========================================================================
// Issue #216 — predicate used to gate ground-test / pyro-fire / servo-test
// BLE commands.  We reject these from INFLIGHT (a launched rocket should
// not be reachable from app-side test commands) AND from MAG_CALIBRATION
// (the user is physically tumbling the rocket; an accidental Fire/Test
// button tap during cal must not arm pyros or kick the servos).  Keeping
// the rule in one place ensures every test-command site agrees.
static inline bool isCommandLockoutState(RocketState s)
{
    return s == INFLIGHT || s == MAG_CALIBRATION;
}
static uint32_t launch_time_millis = 0;
static uint32_t prelaunch_time_millis = 0;
static uint32_t valid_gnss_start_millis = 0;
static uint32_t landed_candidate_start_millis = 0;
static bool landed_candidate_active = false;  // #297: explicit flag (0 collided with now_ms==0)
static bool gnss_started = false;

// #557 GNSS-absent degraded flight.  A dead/deaf-UART module makes the GNSS
// bring-up fail; the FC then flies without GNSS instead of hanging.
//   gnss_absent_mode   — latched once at boot from the collector: the module
//                        failed bring-up.  Makes the baro+IMU-only EKF init
//                        ELIGIBLE (no fix will ever arrive).
//   gnss_absent_flight — set true when the EKF was actually initialized on the
//                        degraded path this session.  Drives the SH_GNSS_ABSENT
//                        telemetry flag and the launch-time guidance force-off.
//                        Reset per sim run so a sim re-evaluates on the normal path.
//   gnss_absent_dwell_ms — earliest millis() a degraded init may fire.  A short
//                        settle after boot / sim-reset so an injected sim GNSS
//                        fix (arrives ~100 ms) reaches the normal init path
//                        before the degraded fallback can commit.  A real dead
//                        module never gets a fix, so it degrades right after.
static constexpr uint32_t GNSS_ABSENT_INIT_DWELL_MS = 5000U;
static bool     gnss_absent_mode    = false;
static bool     gnss_absent_flight  = false;
static uint32_t gnss_absent_dwell_ms = 0;
static bool ground_pressure_found = false;
static bool out_ready = false;
static uint32_t out_ready_request_time_ms = 0;
static uint8_t out_pending_command = 0U;
static uint8_t last_processed_cmd = 0U;  // dedup: ignore OutComputer repeats
static bool end_flight_sent = false;
static OutStatusQueryData out_status_query_data = {};

// Active board→rocket mounting orientation.  Phase 1: static from
// config::BOARD_TO_ROCKET_ORIENT at boot; pad-gravity auto-detect (phase 2)
// and the app setting (phase 3) will call applyBoardToRocketOrientation()
// again at runtime.  Cached here so the OC status query and the flight
// settings snapshot both report what the converter is actually applying.
static uint8_t b2r_active_code = ORIENT_CODE_IDENTITY;
static uint8_t b2r_active_mode = ORIENT_MODE_DEFAULT;
static float   b2r_active_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float   b2r_active_R[9] = {1.0f, 0.0f, 0.0f,
                                  0.0f, 1.0f, 0.0f,
                                  0.0f, 0.0f, 1.0f};
static int16_t b2r_active_residual_cdeg = 0;

// Pad-gravity auto-detect (runs READY/PRELAUNCH, latched at launch) and
// the boost-phase thrust-axis cross-check on the latched orientation.
static OrientationEstimator orient_estimator;
static ThrustAxisCheck      orient_thrust_check;
static bool                 orient_thrust_mismatch = false;
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

// ==========================================================================
// SECTION: Camera phase state machines
// ==========================================================================
// Deferred camera-stop sequence (see cameraStop / serviceCameraStop).
// Idle → DelayBeforeStop (30s post-LANDED) → for RunCam, RunCamToggleSent
// (500ms after toggle, then power off) → Idle.
enum class CameraStopPhase : uint8_t {
    Idle,
    DelayBeforeStop,
    RunCamToggleSent,
};
static CameraStopPhase camera_stop_phase = CameraStopPhase::Idle;
static uint32_t camera_stop_due_ms = 0;
// Deferred camera-start sequence (RunCam only; see cameraStart /
// serviceCameraStart).  The RunCam cold-boots on each power-on and only
// answers UART once ready.  Waiting inline used to block the flight task
// long enough to trip the task watchdog (#146), so the boot wait is
// serviced from the main loop.  A fixed wait + single probe raced the boot
// time and, on a miss, fell back to a record-less power toggle; we now POLL
// GET_DEVICE_INFO and command START_RECORDING the moment the camera answers
// (or blind at the deadline), then resend it for UART reliability.
// The UART pins are parked high-Z whenever the camera is unpowered (battery
// cold-start phantom-feed fix, see config.h) and attached at power-on.  A
// probe window that ends with the camera never having answered gets a
// park+power-cycle retry (RUNCAM_POWER_RETRIES / RUNCAM_RETRY_OFF_MS)
// before the final blind fallback.
//   Idle → WaitingForBoot → Probing → ResendRecord → Idle,
//   with Probing → RetryOff → WaitingForBoot on a silent window.
enum class CameraStartPhase : uint8_t {
    Idle,
    RetryOff,        // gate off, pins parked, waiting to reapply power
    WaitingForBoot,  // min boot grace before the first probe
    Probing,         // polling GET_DEVICE_INFO until the camera answers
    ResendRecord,    // re-sending START_RECORDING for reliability
};
static CameraStartPhase camera_start_phase = CameraStartPhase::Idle;
static uint32_t camera_start_due_ms = 0;
static uint32_t camera_start_power_ms = 0;       // when RUNCAM_PWR_PIN went high
static uint32_t camera_probe_deadline_ms = 0;    // stop polling, record blind
static uint8_t  camera_record_resends_left = 0;  // remaining START_RECORDING resends
static uint8_t  camera_power_retries_left = 0;   // remaining power-cycle attempts
static uint8_t runtime_camera_type = config::CAMERA_TYPE;  // can be overridden via BLE
static bool servo_enabled = false;
static bool gain_sched_enabled = config::GAIN_SCHEDULE_ENABLED;
static bool use_angle_control = config::USE_ANGLE_CONTROL;
static uint16_t roll_delay_ms = config::ROLL_CONTROL_DELAY_MS;
static float kp_angle_rate_cap_dps = config::KP_ANGLE_RATE_CAP_DPS;

// ==========================================================================
// SECTION: Roll control: gains, PID, and roll profile
// ==========================================================================
static float kp_angle_outer = config::KP_ANGLE;  // outer angle-loop P-gain (runtime/app-overridable)
static float integral_sep_threshold_dps = config::INTEGRAL_SEP_THRESHOLD_DPS;  // PID anti-windup threshold
// --- Guidance (PN) state ---
static TR_GuidancePN guidance;
static TR_ControlMixer control_mixer;
static TR_PID roll_rate_pid_standalone(config::KP, config::KI, config::KD,
                                       config::MAX_CMD, config::MIN_CMD);
static bool guidance_enabled = config::GUIDANCE_ENABLED;
// Runtime-tunable guidance params (seeded from config::, overridden by NVS /
// the app's GuidanceConfigData push).  pn_target_e/n are the station-keep aim
// point; MODE_PN ignores them by library contract.
static float    pn_nav_gain       = config::PN_NAV_GAIN;
static float    pn_max_accel      = config::PN_MAX_ACCEL_MPS2;
static float    pn_accel_to_fin   = config::PN_ACCEL_TO_FIN_DEG;
static float    pn_max_fin_deg    = config::PN_MAX_FIN_DEG;
static float    pn_min_speed      = config::PN_MIN_SPEED_MPS;
static uint16_t pn_coast_delay_ms = config::PN_COAST_DELAY_MS;
static uint8_t  pn_target_mode    = config::PN_TARGET_MODE;
static float    pn_target_e       = config::PN_TARGET_E_M;
static float    pn_target_n       = config::PN_TARGET_N_M;
static float    pn_target_alt     = config::PN_TARGET_ALT_M;
static float    pn_kp_pos         = config::PN_KP_POS_PER_S2;
static float    pn_kd_vel         = config::PN_KD_VEL_PER_S;
static uint8_t  pn_guidance_law   = config::GUIDANCE_LAW_DEFAULT;  // GUIDE_LAW_*
// Drift-Cast geodetic aim point (#435, BLE cmd 28).  RAM ONLY — deliberately
// never persisted to NVS: a stale wind-compensated target self-applying on a
// later boot is the hazard; Drift-Cast targets are per-flight ephemeral (the
// NVS gte/gtn/gta keys stay owned exclusively by cmd 65).  The echo state
// lives directly in out_status_query_data.tgt_* (single writer: loop_fc), so
// every OUT_STATUS_QUERY poll carries it for free.
static bool   guid_point_valid   = false;
static double guid_point_lat_deg = 0.0;
static double guid_point_lon_deg = 0.0;
static float  guid_point_alt_m   = 0.0f;
// Shadow of the cmd-65/NVS-OWNED aim point.  pn_target_e/n temporarily
// diverge from these while a Drift-Cast (cmd-28) point is active; every
// path that drops the cmd-28 point outside a flight (cmd-65 reject-keep-
// previous, sim start/stop reset) reverts pn_target_e/n to these — NEVER
// leaving the wind-compensated conversion behind where the unconditional
// cmd-65 NVS persist (or a sim run) could adopt it as a profile point.
static float  pn_profile_target_e = config::PN_TARGET_E_M;
static float  pn_profile_target_n = config::PN_TARGET_N_M;
// Last commanded pre-mix fin deflections from guidance, carried from the
// guidance compute block to the ~10 Hz GuidanceTelemData emit. Only meaningful
// while guidance_active (the only condition under which they are logged).
static float    pn_last_pitch_fin_deg = 0.0f;
static float    pn_last_yaw_fin_deg   = 0.0f;
static bool burnout_detected = false;
static uint32_t burnout_time_ms = 0;
// Consecutive-sample hysteresis for burnout detection (#197). Prevents
// single-sample noise / vibration / wind dips from latching the flag
// prematurely; require N consecutive negative body_ax samples first.
static uint16_t burnout_neg_count = 0;

// --- Recovery deployment detection + dynamic logging rate ---
// The user's logging-rate SETTING (BLE cmd 67, persisted in NVS "imu"/"rate").
// IMU_RATE_DYNAMIC means "let the flight drive it"; anything else is a fixed
// ODR.  Kept separate from the collector's live rate on purpose: in dynamic
// mode the live rate changes mid-flight, and the setting must not.
static uint16_t imu_rate_setting = IMU_RATE_DYNAMIC;
static tr::DeploymentState deployment_state;
// Sticky per-flight latch mirroring deployment_state.detected, for the
// telemetry bit and the rate step-down.  Cleared wherever burnout is cleared.
static bool     deployment_detected = false;
static uint32_t deployment_time_ms  = 0;

// Put the collector on the ODR the current setting resolves to. Called on the
// deployment edge, at every new-flight reset, and when the setting changes.
// Cheap and idempotent: setIsm6Rate stages for the poll task, and re-staging
// the rate already running is a no-op the task applies and forgets.
static void applyImuRateForFlightPhase()
{
    const uint16_t hz = imuRateResolve(imu_rate_setting, deployment_detected);
    if (hz != sensor_collector_hw.ism6Rate()) sensor_collector_hw.setIsm6Rate(hz);
}

static bool mach_locked_out = false;    // promoted from baro block for apogee voting
static bool gps_new_for_kc = false;     // new GPS sample available for kinematic checks
static bool guidance_active = false;
// Tilt-limit safety latch: set true once the off-vertical tilt exceeds the
// phase limit (config::GUIDANCE_TILT_LIMIT_*); guidance then stays roll-only
// for the rest of the flight. Reset at re-arm alongside guidance.reset().
static bool guidance_tilt_inhibited = false;
static bool ground_test_active = false;
// Last roll fin command (deg) produced by the ground-test / guidance paths
// that bypass servo_control's internal PID. Used to populate
// NonSensorData.roll_cmd when those paths are active so telemetry reflects
// what was actually sent to the servos.
static float last_external_roll_cmd_deg = 0.0f;
static bool servo_test_active = false;
static float servo_test_angles[4] = {0, 0, 0, 0};
static bool servo_replay_active = false;
// #345 follow-up: while now_ms < this, READY/PRELAUNCH skip the per-tick
// servo idle() so a just-applied trim stays driven and visible on the pad.
static uint32_t servo_pad_wake_until_ms = 0;
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
// New PCB: single PYRO_ARM_PIN drives all four channels' upstream FET.
// Per-channel fire is a [ArmSettle → Firing → Done] state machine —
// ARM goes HIGH only while at least one channel sits in ArmSettle or
// Firing (or a CONT-TEST is running), then drops LOW once everyone is
// back to Idle/Done. Spinlock protects all pyro state and GPIO ops.
static portMUX_TYPE pyro_spinlock = portMUX_INITIALIZER_UNLOCKED;
static PyroConfigData pyro_config = {};  // zeroed = all four disabled

// ==========================================================================
// SECTION: Pyro channel state
// ==========================================================================
enum class PyroChState : uint8_t {
    Idle,       // no fire requested
    ArmSettle,  // ARM raised by this channel; waiting PYRO_ARM_SETTLE_MS
    Firing,     // FIRE pin HIGH; waiting PYRO_FIRE_DURATION_MS
    Done,       // fired; latched, prevents re-fire this flight
};

struct PyroChRuntime {
    PyroChState state          = PyroChState::Idle;
    uint32_t    phase_start_ms = 0;     // millis when current state began
    bool        cont_known     = false; // true after first cont read post-power-on
    bool        cont           = false; // last reading: true = closed (load present)
};

static PyroChRuntime pyro_ch[4] = {};
static bool          pyro_arm_pin_state   = false;   // last value driven onto PYRO_ARM_PIN
static bool          pyro_apogee_detected = false;
static uint32_t      pyro_apogee_time_ms  = 0;

// PRELAUNCH-edge continuity check: on the new PCB the CONT divider is fed
// from VPP (always on) and is independent of the shared ARM rail, so we
// just sample the four CONT pins directly — no ARM pulse, no settle delay,
// no state machine.
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
// Resend the flight settings snapshot (#165) on the first few INFLIGHT ticks
// so a single dropped I2S frame at launch doesn't lose the record, plus one
// late copy at launch+5 s, clear of the logging-activation edge (7/05 V2 F2
// lost all three early copies to the pre-#418 races).
static uint8_t  settings_emit_count = 0;

// ==========================================================================
// SECTION: Flight snapshot (crash recovery)
// ==========================================================================
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
    // v4: latch sim mode into the snapshot — isSimActive() is false after a
    // reboot, so the restore path can only learn this from the snapshot
    // itself (and refuses to restore when set).
    snap.sim_flight   = sensor_collector.isSimActive() ? 1 : 0;

    snap.flight_elapsed_ms  = now_ms - launch_time_millis;
    snap.apogee_elapsed_ms  = pyro_apogee_detected
                            ? (pyro_apogee_time_ms - launch_time_millis) : 0;
    snap.burnout_elapsed_ms = burnout_detected
                            ? (burnout_time_ms - launch_time_millis) : 0;

    portENTER_CRITICAL(&pyro_spinlock);
    snap.pyro_apogee_detected = pyro_apogee_detected ? 1 : 0;
    snap.pyro1_fired = (pyro_ch[0].state == PyroChState::Done) ? 1 : 0;
    snap.pyro2_fired = (pyro_ch[1].state == PyroChState::Done) ? 1 : 0;
    snap.pyro3_fired = (pyro_ch[2].state == PyroChState::Done) ? 1 : 0;
    snap.pyro4_fired = (pyro_ch[3].state == PyroChState::Done) ? 1 : 0;
    portEXIT_CRITICAL(&pyro_spinlock);

    snap.ground_pressure_pa = ground_pressure_pa;
    snap.ref_lat_rad        = ref_lat_rad;
    snap.ref_lon_rad        = ref_lon_rad;
    snap.ref_alt_m          = ref_alt_m;

    snap.ekf_initialized = ekf_initialized  ? 1 : 0;
    snap.guidance_enabled = guidance_enabled ? 1 : 0;
    snap.burnout_detected = burnout_detected ? 1 : 0;
    snap.servo_enabled    = servo_enabled    ? 1 : 0;

    // Board→rocket orientation latched at launch (v3) — the EKF state
    // below is only meaningful in this rocket frame.
    snap.b2r_code = b2r_active_code;
    snap.b2r_mode = b2r_active_mode;
    for (int i = 0; i < 4; ++i) {
        snap.b2r_q[i] = (int16_t)lroundf(b2r_active_quat[i] * ORIENT_QUAT_WIRE_SCALE);
    }

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
    buildFlightSnapshot(snap, time_ms(), /* override_state = */ (uint8_t)LANDED);
    (void)enqueueI2STx(SNAPSHOT_MSG,
                       reinterpret_cast<const uint8_t*>(&snap),
                       sizeof(snap));
}

// ==========================================================================
// SECTION: Reset-reason reporting
// ==========================================================================
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

// ==========================================================================
// SECTION: Servo control and config-frame reads
// ==========================================================================
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

// #268: the FC runs TWO roll-rate PIDs that must stay in lockstep — servo_control's
// internal PID (INFLIGHT roll null) and roll_rate_pid_standalone (ground-test +
// coast-guidance roll null, whose output is mixed with the PN pitch/yaw fins).
// Route every gain / limit / anti-windup change through these helpers so the
// standalone can't silently run stale gains or with integral-separation disabled
// (the original bug: the threshold + runtime gains were wired to servo_control only).
static void applyRollPidGains(float kp, float ki, float kd, float min_cmd, float max_cmd)
{
    servo_control.setPIDGains(kp, ki, kd);
    servo_control.setPIDLimits(min_cmd, max_cmd);
    roll_rate_pid_standalone.setKp(kp);
    roll_rate_pid_standalone.setKi(ki);
    roll_rate_pid_standalone.setKd(kd);
    roll_rate_pid_standalone.setMinCmd(min_cmd);
    roll_rate_pid_standalone.setMaxCmd(max_cmd);
}

static void applyRollPidSepThreshold(float threshold)
{
    servo_control.setPIDIntegralSeparationThreshold(threshold);
    roll_rate_pid_standalone.setIntegralSeparationThreshold(threshold);
}

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
    // #399: retry reads MUST clock exactly COMBINED_READ_SIZE — the size the
    // OC's slave stages (and pushes per on_request) on the status/config path.
    // The old "frame_size + 20" (e.g. 44 B for a 16 B config) read FEWER bytes
    // than the 96 pushed, leaving residue in the V2 slave's TX ringbuffer; one
    // mismatched read permanently misaligned every subsequent 96-byte poll
    // read (bench 2026-07-03: 100% read failures for the rest of the flight).
    // Reading the full staged size keeps the ring drained; the SOF scan below
    // finds the config frame anywhere within it (every config frame fits —
    // the OC's fit-check guarantees status+config <= COMBINED_READ_SIZE).
    const size_t read_size  = COMBINED_READ_SIZE;

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
                delay_ms(5 * (attempt + 1));
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
        delay_ms(5 * (attempt + 1));
    }

    payload_len_out = 0;
    return false;
}

// ── Pyro channel helpers ─────────────────────────────────────────────────────

// Per-channel pin lookups (indexed 0..3).
static const uint8_t PYRO_FIRE_PINS[4] = {
    config::PYRO1_FIRE_PIN, config::PYRO2_FIRE_PIN,
    config::PYRO3_FIRE_PIN, config::PYRO4_FIRE_PIN,
};
static const uint8_t PYRO_CONT_PINS[4] = {
    config::PYRO1_CONT_PIN, config::PYRO2_CONT_PIN,
    config::PYRO3_CONT_PIN, config::PYRO4_CONT_PIN,
};

// ==========================================================================
// SECTION: Pyro channels: init, safing, and servicing
// ==========================================================================
// Per-channel config accessors so the loop body stays uniform.
static inline bool pyroChEnabled(int ch_idx)
{
    switch (ch_idx) {
        case 0: return pyro_config.ch1_enabled;
        case 1: return pyro_config.ch2_enabled;
        case 2: return pyro_config.ch3_enabled;
        case 3: return pyro_config.ch4_enabled;
    }
    return false;
}
static inline uint8_t pyroChMode(int ch_idx)
{
    switch (ch_idx) {
        case 0: return pyro_config.ch1_trigger_mode;
        case 1: return pyro_config.ch2_trigger_mode;
        case 2: return pyro_config.ch3_trigger_mode;
        case 3: return pyro_config.ch4_trigger_mode;
    }
    return 0;
}
static inline float pyroChValue(int ch_idx)
{
    switch (ch_idx) {
        case 0: return pyro_config.ch1_trigger_value;
        case 1: return pyro_config.ch2_trigger_value;
        case 2: return pyro_config.ch3_trigger_value;
        case 3: return pyro_config.ch4_trigger_value;
    }
    return 0.0f;
}

// New PCB inverts continuity sense vs the old board: with the shared
// arming FET enabled, the channel's CONT line idles HIGH when the squib
// loop is OPEN and gets pulled LOW when a load (closed circuit) is
// present. Old PCB had the opposite polarity. Bench-verified on a populated
// board (#264): raw 0 == continuity present.
static inline bool pyroContFromRaw(int raw) { return raw == 0; }

// Reset a pyro OUTPUT pad to a known LOW-driven state WITHOUT going through
// gpio_reset_pin(), which briefly enables the internal pull-up (~50 kΩ) as
// part of its `GPIO_MODE_DISABLE` config. On the new pyro PCB the gate
// drivers are DTC123J-style pre-biased NPNs (internal 2.2 kΩ base resistor
// + 47 kΩ B-E pull-down): a 50 kΩ pull-up on the GPIO biases the base well
// above Vbe for the microseconds-long window between gpio_reset_pin's
// pull-up config and our subsequent pull-up-disable config — long enough to
// momentarily turn on the ARM / FIRE MOSFETs and twitch the squib rail at
// boot. This sequence pre-loads GPIO_OUT to 0, detaches any peripheral
// output signal, selects plain-GPIO function on the IO MUX, and only then
// enables output drive — so the pad goes from high-Z directly to driving 0
// with no pull-up window.
static void safePyroOutputInit(gpio_num_t pin)
{
    // 1. Pre-load output register to 0. No-op until output is enabled.
    gpio_set_level(pin, 0);
    // 2. Detach any peripheral output signal routed to this pad (e.g.,
    //    SPI2 default on pins 14-19 after spi_bus_initialize()).
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
    // 3. Force IO MUX function back to plain GPIO.
    gpio_func_sel(pin, PIN_FUNC_GPIO);
    // 4. Now enable output drive with no pulls. Pad transitions from high-Z
    //    straight to driving 0 (because step 1 staged a 0 in GPIO_OUT).
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << pin;
    cfg.mode         = GPIO_MODE_OUTPUT;
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
    // Belt + suspenders.
    gpio_set_level(pin, 0);
}

static void initPyroPins()
{
    // ========================================================================
    // SAFETY-CRITICAL: do NOT replace safePyroOutputInit() with
    // gpio_reset_pin() for any of the OUTPUT pins below. The IDF's
    // gpio_reset_pin() transiently enables the internal ~50 kΩ pull-up,
    // which biases the DTC123J gate-driver NPN base above Vbe long enough
    // to twitch the ARM and FIRE MOSFETs — confirmed on bench, the pyro
    // rail flashes on every FC boot.
    //
    // If you add new pyro output pins, they MUST go through
    // safePyroOutputInit(). The CONT input pads are fine on gpio_reset_pin()
    // because they have no gate driver downstream.
    // See project_pyro_safe_init_required.md in the agent memory and
    // commit 421dd63 for the bench observation.
    // ========================================================================
    const gpio_num_t output_pins[] = {
        (gpio_num_t)config::PYRO_ARM_PIN,
        (gpio_num_t)config::PYRO1_FIRE_PIN,
        (gpio_num_t)config::PYRO2_FIRE_PIN,
        (gpio_num_t)config::PYRO3_FIRE_PIN,
        (gpio_num_t)config::PYRO4_FIRE_PIN,
    };
    for (gpio_num_t pin : output_pins) {
        safePyroOutputInit(pin);
    }
    pyro_arm_pin_state = false;

    // CONTINUITY pins: input (external pull on PCB). gpio_reset_pin() is
    // safe here — the input pad has no gate driver downstream, just the
    // CONT divider, so a brief internal pull-up doesn't drive anything
    // dangerous. The subsequent gpio_config() clears it.
    for (uint8_t pin : PYRO_CONT_PINS) {
        gpio_reset_pin((gpio_num_t)pin);
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << pin;
        cfg.mode         = GPIO_MODE_INPUT;
        cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&cfg);
    }
    ESP_LOGI(TAG, "[PYRO] Pins initialized (safe, 4 channels, single ARM)");
}

// Caller MUST hold pyro_spinlock.
static void pyroSetArmLocked(bool want_high)
{
    // #317: once a flight has landed, never re-arm the pyro rail until a hardware
    // reboot — only disarm requests are honored.
    if (post_flight_lockout) want_high = false;
    if (pyro_arm_pin_state == want_high) return;
    gpio_set_level((gpio_num_t)config::PYRO_ARM_PIN, want_high ? 1 : 0);
    pyro_arm_pin_state = want_high;
}

// #386: high-g bias -> cm/s² wire encode with an explicit int16 clamp. The
// raw lroundf cast wrapped sign for |bias| > ~327 m/s², so the OC applied a
// grossly wrong bias to its displayed/logged high-g stream (the FC control
// path uses the float directly and was unaffected). Clamped, a huge bias
// saturates instead of flipping sign.
static inline int16_t encodeHgBiasCmss(float bias_mss)
{
    float v = bias_mss * 100.0f;
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)lroundf(v);
}

static void pyroSafeAll()
{
    portENTER_CRITICAL(&pyro_spinlock);
    for (int i = 0; i < 4; ++i) {
        gpio_set_level((gpio_num_t)PYRO_FIRE_PINS[i], 0);
        // #382: keep the Done latch — safing (incl. the LANDED transition)
        // must not erase the record of a channel having fired, or post-flight
        // pyro_status reads all-zeros and the app/logs misreport what
        // deployed. No safety change: pins still drop, ARM still drops, Done
        // never re-fires (auto-fire only starts from Idle), and the
        // PRELAUNCH->INFLIGHT edge already resets channels for a new flight.
        // Reboot recovery restores Done from the snapshot for the same reason.
        if (pyro_ch[i].state != PyroChState::Done) {
            pyro_ch[i].state = PyroChState::Idle;
        }
        pyro_ch[i].phase_start_ms = 0;
    }
    pyroSetArmLocked(false);
    portEXIT_CRITICAL(&pyro_spinlock);
    ESP_LOGI(TAG, "[PYRO] All channels safed");
}

// PRELAUNCH continuity check. Sample all four CONT pins directly and cache
// the result. Synchronous — no ARM pulse, no settle, no state machine.
// VPP (flight-computer power, always on) feeds the CONT sense divider, so
// ARM is not part of the sense path. Pyros are a V8+ feature, so there is
// no earlier board to branch for.
static void pyroPrelaunchContTest(uint32_t /*now_ms*/)
{
    int  raw[4];
    bool cont[4];
    for (int i = 0; i < 4; ++i) {
        raw[i]  = gpio_get_level((gpio_num_t)PYRO_CONT_PINS[i]);
        cont[i] = pyroContFromRaw(raw[i]);
    }
    portENTER_CRITICAL(&pyro_spinlock);
    for (int i = 0; i < 4; ++i) {
        pyro_ch[i].cont       = cont[i];
        pyro_ch[i].cont_known = true;
    }
    portEXIT_CRITICAL(&pyro_spinlock);
    ESP_LOGI(TAG, "[PYRO] Prelaunch CONT: ch1 raw=%d cont=%d  ch2 raw=%d cont=%d  ch3 raw=%d cont=%d  ch4 raw=%d cont=%d",
             raw[0], cont[0], raw[1], cont[1], raw[2], cont[2], raw[3], cont[3]);
}

// Called each main-loop tick during INFLIGHT (and benign elsewhere — no
// channel will leave Idle until an enabled trigger fires). Drives the
// per-channel ArmSettle → Firing → Done state machine and computes the
// global ARM-pin level from the union of all channel demands.
static void servicePyroChannels(uint32_t now_ms)
{
    // Detect apogee (N-1 of N voting in TR_KinematicChecks)
    if (!pyro_apogee_detected && kinematics.apogee_flag) {
        pyro_apogee_detected = true;
        pyro_apogee_time_ms  = now_ms;
        ESP_LOGI(TAG, "[PYRO] Apogee detected at t=%lu ms (vel=%d baro=%d gps=%d pitch=%d backstop=%d)",
                 (unsigned long)now_ms,
                 kinematics.vel_u_apogee_flag,
                 kinematics.alt_apogee_flag,
                 kinematics.gps_apogee_flag,
                 kinematics.pitch_apogee_flag,
                 kinematics.apogee_backstop_flag);
    }

    bool just_fired[4]  = { false, false, false, false };
    bool pulse_done[4]  = { false, false, false, false };

    portENTER_CRITICAL(&pyro_spinlock);

    for (int i = 0; i < 4; ++i) {
        PyroChRuntime& ch = pyro_ch[i];

        // Idle → ArmSettle when trigger satisfied
        if (ch.state == PyroChState::Idle && pyroChEnabled(i)) {
            bool should_fire = false;
            const uint8_t mode = pyroChMode(i);
            const float   val  = pyroChValue(i);
            if (pyro_apogee_detected && mode == PYRO_TRIGGER_TIME_AFTER_APOGEE) {
                const float elapsed_s = (float)(now_ms - pyro_apogee_time_ms) / 1000.0f;
                if (elapsed_s >= val) should_fire = true;
            } else if (pyro_apogee_detected && mode == PYRO_TRIGGER_ALTITUDE_ON_DESCENT) {
                if (pressure_alt_m <= val && pressure_alt_rate_mps < 0.0f) should_fire = true;
            }
            if (should_fire) {
                ch.state          = PyroChState::ArmSettle;
                ch.phase_start_ms = now_ms;
            }
        }

        // ArmSettle → Firing once the settle elapses.
        // SIM DRY-FIRE: during a simulated flight the full state machine
        // runs — states, fired flags, telemetry, deployment logging — but
        // the FIRE pin is never driven HIGH, so charges connected on a
        // bench stay cold. The apps' "pyros will fire" warning used to be
        // the only protection. PYRO_FIRE_TEST (the explicit user ground
        // test) is deliberately NOT gated — that command means "energize
        // this channel now" regardless of sim state.
        if (ch.state == PyroChState::ArmSettle &&
            (now_ms - ch.phase_start_ms) >= config::PYRO_ARM_SETTLE_MS) {
            if (!sensor_collector.isSimActive()) {
                gpio_set_level((gpio_num_t)PYRO_FIRE_PINS[i], 1);
            }
            ch.state          = PyroChState::Firing;
            ch.phase_start_ms = now_ms;
            just_fired[i]     = true;
        }

        // Firing → Done after pulse width
        if (ch.state == PyroChState::Firing &&
            (now_ms - ch.phase_start_ms) >= config::PYRO_FIRE_DURATION_MS) {
            gpio_set_level((gpio_num_t)PYRO_FIRE_PINS[i], 0);
            ch.state          = PyroChState::Done;
            ch.phase_start_ms = 0;
            pulse_done[i]     = true;
        }
    }

    // While ARM is HIGH (only during a channel's fire window in flight),
    // refresh its CONT bit. Outside a fire window the cached value from the
    // last pad test stands. (#382 comment fix: on the current PCB the CONT
    // divider is VPP-fed and always readable (#264) — ARM=LOW does NOT float
    // CONT; we just don't bother re-sampling outside fire windows.)
    if (pyro_arm_pin_state) {
        for (int i = 0; i < 4; ++i) {
            if (pyro_ch[i].state == PyroChState::ArmSettle ||
                pyro_ch[i].state == PyroChState::Firing) {
                pyro_ch[i].cont       = pyroContFromRaw(
                                          gpio_get_level((gpio_num_t)PYRO_CONT_PINS[i]));
                pyro_ch[i].cont_known = true;
            }
        }
    }

    // Compute ARM demand: any channel mid-fire keeps the pin HIGH.
    // Sim dry-fire: the ARM rail stays LOW too — the whole squib circuit
    // remains unpowered while the state machine sequences.
    bool any_demand = false;
    for (int i = 0; i < 4; ++i) {
        if (pyro_ch[i].state == PyroChState::ArmSettle ||
            pyro_ch[i].state == PyroChState::Firing) {
            any_demand = true;
            break;
        }
    }
    pyroSetArmLocked(any_demand && !sensor_collector.isSimActive());

    portEXIT_CRITICAL(&pyro_spinlock);

    const bool dry = sensor_collector.isSimActive();
    for (int i = 0; i < 4; ++i) {
        if (just_fired[i]) ESP_LOGW(TAG, "[PYRO] CH%d %s at alt=%.1f m",
                                    i + 1, dry ? "DRY-FIRED (sim)" : "FIRED",
                                    (double)pressure_alt_m);
        if (pulse_done[i]) ESP_LOGI(TAG, "[PYRO] CH%d fire pulse complete", i + 1);
    }
}

// ==========================================================================
// SECTION: Roll-profile waypoint lookup
// ==========================================================================
// ── Roll profile lookup ─────────────────────────────────────────────────────
// Returns the desired (angle, mode) for the current flight time.
// FlightSettingsData v4 semantics: the profile is pure (time, angle) waypoints —
// per-waypoint mode bytes are IGNORED (legacy wire field, pre-v4).
//   * before WP1:        NULL_RATE (fins hold zero roll rate through boost)
//   * WPi..WPi+1:        ANGLE, target linearly interpolated WPi→WPi+1 along
//                        the shortest wrapped arc
//   * after WPn:         ANGLE, hold WPn's angle
// To hold an angle (or "null roll") mid-profile, give two consecutive
// waypoints the same angle. An earlier interpolation attempt was reverted
// because it lerped the RAW angles, sweeping the long way round and flipping
// the command at the ±180° error wrap; interpolating the WRAPPED delta keeps
// the target on the shortest arc, so the target never jumps.
// If no waypoints are loaded, defaults to NULL_RATE / 0 deg.
struct RollProfileQuery
{
    float   angle_deg;
    uint8_t mode;
};

static inline float wrap180f(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static RollProfileQuery roll_profile_query(float t_flight_s)
{
    RollProfileQuery out{};
    out.angle_deg = config::ROLL_RATE_SET_POINT;
    out.mode      = ROLL_SEG_NULL_RATE;

    if (roll_profile.num_waypoints == 0)
    {
        return out;
    }
    const uint8_t n = roll_profile.num_waypoints;

    // Before first waypoint: null roll rate (safe boost behavior)
    if (t_flight_s < roll_profile.waypoints[0].time_s)
    {
        return out;
    }
    // After last waypoint: hold last angle
    if (t_flight_s >= roll_profile.waypoints[n - 1].time_s)
    {
        out.angle_deg = roll_profile.waypoints[n - 1].angle_deg;
        out.mode      = ROLL_SEG_ANGLE;
        return out;
    }
    // Inside profile -- find the active segment [i, i+1) and interpolate the
    // target along the shortest wrapped arc from WPi to WPi+1.
    for (uint8_t i = 0; i < n - 1; ++i)
    {
        const RollWaypoint &w0 = roll_profile.waypoints[i];
        const RollWaypoint &w1 = roll_profile.waypoints[i + 1];
        if (t_flight_s < w1.time_s)
        {
            const float span = w1.time_s - w0.time_s;
            const float frac = (span > 1e-3f)
                                   ? (t_flight_s - w0.time_s) / span
                                   : 1.0f;
            out.angle_deg = wrap180f(w0.angle_deg +
                                     wrap180f(w1.angle_deg - w0.angle_deg) * frac);
            out.mode      = ROLL_SEG_ANGLE;
            return out;
        }
    }
    out.angle_deg = roll_profile.waypoints[n - 1].angle_deg;
    out.mode      = ROLL_SEG_ANGLE;
    return out;
}

// ==========================================================================
// SECTION: I2S transmit to the Out Computer
// ==========================================================================
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

// ---- OTA relay receiver (#8 Phase 4) -------------------------------------
// The FC receives an OTA image relayed from the OC. Control (begin/finish/
// abort) arrives over I2C as pending commands; the image bytes arrive over
// I2S in Layer 3. Status is reported back to the OC over I2S (OTA_STATUS_MSG),
// which the OC relays up to the app as ota_status.
static TR_OTA_Backend_esp fc_ota_backend;
static TR_OTA_Receiver    fc_ota_receiver(fc_ota_backend);

// ==========================================================================
// SECTION: OTA relay receiver (image in over I2S)
// ==========================================================================
static void sendOtaRelayStatus(uint8_t state, uint8_t err, uint32_t bytes_written)
{
    OtaRelayStatusData st;
    st.state         = state;
    st.err           = err;
    st.bytes_written = bytes_written;
    (void)enqueueI2STx(OTA_STATUS_MSG, (const uint8_t*)&st, sizeof(st));
}

// Resend an OTA status several times, spread over ~1.4 s.  A long flash op on
// the FC (the ota_1 erase in begin(), or the verify/finalize in finish())
// runs with the cache disabled, which suspends i2sSenderTask — it executes
// from flash, not IRAM.  With nothing feeding the I2S DMA, the continuously-
// running clock replays whatever was in the DMA buffers, so the OC sees a
// burst of stale/duplicate frames for the whole op (the "dedup storm", per
// the i2sSenderTask idle-fill comment).  A single OTA_STATUS_MSG sent the
// instant the op finishes lands in that window: the OC's rx_ring is still
// draining the storm backlog (drop-oldest overflow) and the first real frame
// can be corrupted by the stale->fresh DMA transition (CRC drop).  Either way
// the one-shot status is lost and the app strands at "OTA_BEGIN not accepted".
// Spreading copies over ~1.4 s guarantees at least one lands after the OC has
// drained the backlog and the stream is back to clean idle-fill.  This mirrors
// the #165 flight-settings resend (one-shot frame re-emitted on the first few
// INFLIGHT ticks so a dropped launch frame doesn't lose the record).  Status
// frames are idempotent — the OC just re-relays the same ota_status and the
// app keys off the first matching state — so duplicates are harmless.  Safe to
// block: OTA is refused in flight, and the erase/finish that precedes this
// already blocked the caller for seconds.  delay_ms() is vTaskDelay-based, so
// IDLE still pets the task watchdog between sends.
static void sendOtaRelayStatusRobust(uint8_t state, uint8_t err, uint32_t bytes_written)
{
    static constexpr int      kOtaStatusResends = 7;    // + the immediate send = 8 total
    static constexpr uint32_t kOtaStatusGapMs   = 200;  // 7 * 200 ms ~= 1.4 s span
    sendOtaRelayStatus(state, err, bytes_written);
    for (int i = 0; i < kOtaStatusResends; ++i)
    {
        delay_ms(kOtaStatusGapMs);
        sendOtaRelayStatus(state, err, bytes_written);
    }
}

// --- Phase 4 Layer 3: FC side of the I2S image pump --------------------------
// On OTA_BEGIN (after erasing ota_1 and sending READY over the normal-direction
// I2S) the FC flips its I2S master-TX link to slave-RX, receives the image as
// OTA_DATA_CHUNK frames (each prefixed with its absolute offset), and on
// FINISH/ABORT reverts to master-TX so its terminal status rides the normal I2S
// path back to the OC. i2s_stream is touched by i2sSenderTask (normal TX) and
// the flip (command-handler task), so fc_i2s_mutex serializes them; the sender
// idles while fc_ota_data_mode is set.
static SemaphoreHandle_t fc_i2s_mutex = nullptr;
static volatile bool fc_ota_data_mode = false;   // flipped to slave RX for the image
static uint32_t fc_ota_total_size = 0;            // expected image size (from BEGIN)

// Image RX ring: filled by the I2S recv callback (ISR), drained by
// fcOtaParserTask. Sized to absorb a chunk burst while a flash write (which
// suspends the parser task with the cache off) completes.
static constexpr size_t FC_OTA_RING = 8192;
static uint8_t  fc_ota_ring[FC_OTA_RING];
static volatile size_t   fc_ota_head = 0;
static volatile size_t   fc_ota_tail = 0;
static volatile uint32_t fc_ota_ring_ovf = 0;
static TaskHandle_t fc_ota_parser_task = nullptr;
// OTA RX diagnostics (#8 Phase 4, bench) — counted in fcOtaParseRing, summarized
// at FINISH. Distinguish a frame-loss stall (gaps climb, write_fail=0) from a
// flash-write stall (write_fail>0) from an I2S signal-integrity problem
// (crc_fail climbs).
static uint32_t fc_ota_crc_fail   = 0;   // frames that failed CRC (resync drops)
static uint32_t fc_ota_gap        = 0;   // valid frames ahead of bytesWritten (skipped)
static uint32_t fc_ota_write_fail = 0;   // writeChunk() errors
static uint32_t fc_ota_written_frames = 0;  // frames accepted in order
static volatile uint32_t fc_ota_rx_cb_count = 0;  // I2S RX cb ticks; teardown silence detect

static inline IRAM_ATTR void fcOtaRingPush(uint8_t b)
{
    // #568: drop the NEWEST byte on overflow — the ISR must NEVER write
    // fc_ota_tail. The old drop-oldest advanced tail from ISR context while
    // fcOtaRingPop's non-atomic RMW ran in the parser task; the ISR preempting
    // mid-pop (parser reads tail, ISR advances it, parser writes back stale+1)
    // tore the index and corrupted fcOtaRingLen/framing. Same ISR-touches-tail
    // hazard the OC eliminated in #383 (see OC rxPush) — with the ISR owning
    // only head and the parser owning only tail this is a true SPSC ring.
    // Dropping newest is safe here: chunks are CRC-framed and offset-ordered,
    // so a lost byte costs one chunk resend, same as drop-oldest did.
    const size_t next = (fc_ota_head + 1) % FC_OTA_RING;
    if (next == fc_ota_tail)
    {
        fc_ota_ring_ovf = fc_ota_ring_ovf + 1;           // (-Wvolatile: avoid ++)
        return;                                          // ring full: drop newest
    }
    fc_ota_ring[fc_ota_head] = b;
    fc_ota_head = next;
}
static inline size_t  fcOtaRingLen()         { return (fc_ota_head + FC_OTA_RING - fc_ota_tail) % FC_OTA_RING; }
static inline uint8_t fcOtaRingPeek(size_t i){ return fc_ota_ring[(fc_ota_tail + i) % FC_OTA_RING]; }
static inline void    fcOtaRingPop()         { fc_ota_tail = (fc_ota_tail + 1) % FC_OTA_RING; }

// I2S recv callback (ISR): push image bytes into the ring + wake the parser.
static IRAM_ATTR bool fcOtaRecvCallback(const uint8_t* buf, size_t len, void* /*ctx*/)
{
    if (!fc_ota_data_mode) return false;
    fc_ota_rx_cb_count = fc_ota_rx_cb_count + 1;   // (-Wvolatile: avoid ++) teardown silence
    for (size_t i = 0; i < len; i++) fcOtaRingPush(buf[i]);
    BaseType_t woken = pdFALSE;
    if (fc_ota_parser_task) vTaskNotifyGiveFromISR(fc_ota_parser_task, &woken);
    return woken == pdTRUE;
}

// Extract OTA_DATA_CHUNK frames from the ring and write each by offset. Stale
// repeats (offset < bytesWritten, from an I2S underrun replaying a DMA buffer)
// are skipped; an offset ahead of bytesWritten is a gap (logged).
static void fcOtaParseRing()
{
    uint8_t payload[MAX_PAYLOAD];
    while (fcOtaRingLen() >= (4 + 1 + 1 + 2))
    {
        if (!(fcOtaRingPeek(0) == 0xAA && fcOtaRingPeek(1) == 0x55 &&
              fcOtaRingPeek(2) == 0xAA && fcOtaRingPeek(3) == 0x55))
        {
            fcOtaRingPop();          // resync on SOF
            continue;
        }
        const size_t payload_len = fcOtaRingPeek(5);
        if (payload_len > MAX_PAYLOAD) { fcOtaRingPop(); continue; }
        const size_t frame_len = 4 + 1 + 1 + payload_len + 2;
        if (fcOtaRingLen() < frame_len) return;   // rest hasn't arrived yet

        uint8_t frame[MAX_FRAME];
        for (size_t i = 0; i < frame_len; i++) frame[i] = fcOtaRingPeek(i);

        uint8_t type = 0; size_t out_len = 0;
        if (!TR_I2C_Interface::unpackMessage(frame, frame_len, type,
                                             payload, sizeof(payload), out_len, true))
        {
            fc_ota_crc_fail++;
            fcOtaRingPop();          // bad CRC — resync one byte
            continue;
        }
        for (size_t i = 0; i < frame_len; i++) fcOtaRingPop();

        if (type == OTA_DATA_CHUNK && out_len >= 4)
        {
            uint32_t off = 0; memcpy(&off, payload, 4);
            const uint8_t* img     = payload + 4;
            const size_t   img_len = out_len - 4;
            const uint32_t have    = (uint32_t)fc_ota_receiver.bytesWritten();
            if (off == have)
            {
                const TR_OTA_Receiver::Error e = fc_ota_receiver.writeChunk(off, img, img_len);
                if (e != TR_OTA_Receiver::Error::Ok)
                {
                    fc_ota_write_fail++;
                    ESP_LOGE(TAG, "[OTA] writeChunk @%u (%uB) failed: %d",
                             (unsigned)off, (unsigned)img_len, (int)e);
                }
                else
                {
                    fc_ota_written_frames++;
                }
            }
            else if (off > have)
            {
                fc_ota_gap++;
                ESP_LOGW(TAG, "[OTA] gap: chunk @%u but have %u", (unsigned)off, (unsigned)have);
            }
            // off < have: stale-repeat from an I2S underrun — skip silently.
        }
    }
}

static void fcOtaParserTask(void*)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (fc_ota_data_mode) fcOtaParseRing();
    }
}

// Flip master-TX -> slave-RX to receive the image. Called from the OTA_BEGIN
// handler after the READY resends have drained over the still-TX link.
static void fcFlipToRx()
{
    // Let the queued OTA_RELAY_READY resends fully clock out before flipping.
    for (int i = 0; i < 60 && uxQueueMessagesWaiting(i2s_tx_queue) > 0; i++)
        delay_ms(10);
    delay_ms(50);                       // settle: last frame leaves the DMA
    fc_ota_head = 0;                    // clear any stale ring contents
    fc_ota_tail = 0;
    fc_ota_ring_ovf = 0;                // reset RX diagnostics for this session
    fc_ota_crc_fail = 0;
    fc_ota_gap = 0;
    fc_ota_write_fail = 0;
    fc_ota_written_frames = 0;
    fc_ota_data_mode = true;            // i2sSenderTask idles from here on
    if (fc_i2s_mutex) xSemaphoreTake(fc_i2s_mutex, portMAX_DELAY);
    i2s_stream.end();
    const esp_err_t e = i2s_stream.beginSlaveRx(config::I2S_BCLK_PIN,
                                                config::I2S_WS_PIN,
                                                config::I2S_DOUT_PIN,   // former DOUT now reads DIN
                                                -1,                     // no frame-sync for OTA
                                                OTA_I2S_SAMPLE_RATE_HZ,
                                                4, 64);
    if (e == ESP_OK)
        i2s_stream.registerRecvCallback(fcOtaRecvCallback, nullptr);
    if (fc_i2s_mutex) xSemaphoreGive(fc_i2s_mutex);
    ESP_LOGW(TAG, "[OTA] I2S -> slave RX for image (%s)", esp_err_to_name(e));
}

// Revert slave-RX -> master-TX (normal telemetry + status). Called before
// finalizing/aborting so the terminal status rides the normal I2S path.
static void fcRevertToTx()
{
    if (fc_i2s_mutex) xSemaphoreTake(fc_i2s_mutex, portMAX_DELAY);
    i2s_stream.end();
    const esp_err_t e = i2s_stream.beginMasterTx(config::I2S_BCLK_PIN,
                                                 config::I2S_WS_PIN,
                                                 config::I2S_DOUT_PIN,
                                                 config::I2S_FSYNC_PIN,
                                                 config::I2S_SAMPLE_RATE);
    fc_ota_data_mode = false;           // i2sSenderTask resumes
    if (fc_i2s_mutex) xSemaphoreGive(fc_i2s_mutex);
    ESP_LOGW(TAG, "[OTA] I2S -> master TX (%s)", esp_err_to_name(e));
}

// ==========================================================================
// SECTION: Board-to-rocket orientation
// ==========================================================================
// Apply a discrete board→rocket mounting orientation everywhere it matters:
// the converter (rotates all vector sensors into rocket frame), the sim
// collector (inverse, so synthesized sensor counts survive the forward
// path), the cached b2r_active_* globals (flight settings snapshot), and
// the OC status query payload (keeps the OC's converter consistent — the
// query repeats every poll, so a runtime change propagates within one
// cycle).  Phase 1 calls this once at boot from config; auto-detect
// (phase 2) and the app setting (phase 3) re-call it at runtime.
static void applyBoardToRocketRotation(const float R[9], uint8_t code,
                                       uint8_t mode, int16_t residual_cdeg)
{
    sensor_converter.configureBoardToRocket(R);
    sensor_collector.configureSimBoardToRocket(R);

    memcpy(b2r_active_R, R, sizeof(b2r_active_R));
    b2r_active_code = code;
    b2r_active_mode = mode;
    b2r_active_residual_cdeg = residual_cdeg;
    orientMatrixToQuat(R, b2r_active_quat);

    out_status_query_data.b2r_code = code;
    out_status_query_data.b2r_mode = mode;
    for (int i = 0; i < 4; ++i) {
        out_status_query_data.b2r_q[i] =
            (int16_t)lroundf(b2r_active_quat[i] * ORIENT_QUAT_WIRE_SCALE);
    }

    ESP_LOGI(TAG, "Board→rocket orientation: %s (code %u, mode %u, residual %.2f°)",
             orientCodeName(code), (unsigned)code, (unsigned)mode,
             (double)residual_cdeg / 100.0);
}

static void applyBoardToRocketOrientation(uint8_t code, uint8_t mode)
{
    float R[9];
    orientCodeToMatrix(code, R);
    applyBoardToRocketRotation(R, code, mode, 0);
}

// Snap tolerance: a measured nose vector within this angle of a board axis
// is treated as that axis (covers rail tilt + sensor noise for orthogonal
// mounts).  Beyond it the board is genuinely mounted off-axis and the
// exact shortest-arc rotation is used instead.
static constexpr float ORIENT_SNAP_TOL_DEG   = 15.0f;
// Leave the active orientation alone while the measured nose stays within
// this angle of rocket +X — normal rail tilt never causes churn.
static constexpr float ORIENT_ACCEPT_TOL_DEG = 5.0f;

// One completed pad-gravity window: decide whether the active board→rocket
// rotation still matches how the rocket is actually sitting on the rail.
// Latest filtered pad "up" (specific-force) direction in ROCKET frame, from the
// orientation estimator. Cached so the sim can align its pad frame to how the
// board is ACTUALLY resting when a sim run starts (#508).
static float pad_up_rocket[3]   = {0.0f, 0.0f, 0.0f};
static bool  pad_up_rocket_valid = false;

static void handleOrientationEstimate(const float up_rocket[3])
{
    pad_up_rocket[0] = up_rocket[0];
    pad_up_rocket[1] = up_rocket[1];
    pad_up_rocket[2] = up_rocket[2];
    pad_up_rocket_valid = true;

    float cx = up_rocket[0];
    if (cx > 1.0f) cx = 1.0f;
    if (cx < -1.0f) cx = -1.0f;
    const float off_deg = acosf(cx) * (180.0f / (float)M_PI);

    // A manual orientation is authoritative (it also encodes the roll
    // clocking for the control surfaces, which gravity cannot observe).
    // Auto-detect only warns when gravity disagrees badly.
    if (b2r_active_mode == ORIENT_MODE_MANUAL)
    {
        static uint32_t last_warn_ms = 0;
        const uint32_t now_ms = time_ms();
        if (off_deg > ORIENT_SNAP_TOL_DEG && (now_ms - last_warn_ms) > 10000U)
        {
            last_warn_ms = now_ms;
            ESP_LOGW(TAG, "[ORIENT] pad gravity %.1f° off nose with MANUAL "
                          "orientation %s — check the setting",
                     (double)off_deg, orientCodeName(b2r_active_code));
        }
        return;
    }

    if (off_deg <= ORIENT_ACCEPT_TOL_DEG) return;  // mapping already right

    // Un-rotate the measurement into BOARD frame (transpose = inverse) and
    // re-derive the mapping from scratch.
    float n_board[3];
    n_board[0] = b2r_active_R[0] * up_rocket[0] + b2r_active_R[3] * up_rocket[1] + b2r_active_R[6] * up_rocket[2];
    n_board[1] = b2r_active_R[1] * up_rocket[0] + b2r_active_R[4] * up_rocket[1] + b2r_active_R[7] * up_rocket[2];
    n_board[2] = b2r_active_R[2] * up_rocket[0] + b2r_active_R[5] * up_rocket[1] + b2r_active_R[8] * up_rocket[2];

    float residual_deg = 0.0f;
    const uint8_t code = orientNearestNoseCode(n_board, &residual_deg);
    const int16_t residual_cdeg = (int16_t)lroundf(residual_deg * 100.0f);

    float R[9];
    uint8_t mode;
    if (residual_deg <= ORIENT_SNAP_TOL_DEG)
    {
        if (code == b2r_active_code && b2r_active_mode != ORIENT_MODE_AUTO_EXACT)
        {
            return;  // same discrete mounting — off_deg was just rail tilt
        }
        orientCodeToMatrix(code, R);
        mode = ORIENT_MODE_AUTO_SNAP;
    }
    else
    {
        // Off-orthogonal mounting (or an extreme rail angle — the two are
        // indistinguishable from gravity alone; the thrust-axis check at
        // launch is the cross-check).  Use the exact rotation.
        if (!orientMatrixFromNoseVector(n_board, R)) return;
        mode = ORIENT_MODE_AUTO_EXACT;
        ESP_LOGW(TAG, "[ORIENT] nose %.1f° from nearest board axis — using "
                      "exact rotation (off-orthogonal mount or steep rail?)",
                 (double)residual_deg);
    }

    applyBoardToRocketRotation(R, code, mode, residual_cdeg);

    // EKF attitude/velocity/biases were estimated in the OLD rocket frame.
    // Re-init from scratch (pad-only path: GNSS gates re-pass within ~1 s
    // and the gravity init re-aligns attitude in the new frame).
    if (ekf_initialized)
    {
        ekf_initialized = false;
        ESP_LOGI(TAG, "[ORIENT] EKF re-init scheduled after orientation change");
    }
}

// ==========================================================================
// SECTION: Guidance configuration and flight settings
// ==========================================================================
// Push the guidance law, its gains and the horizontal aim point into the
// guidance object.  ORDER IS LOAD-BEARING: configure()/configureStationKeep()
// set mode_, and TR_GuidancePN consumes the aim point in MODE_STATION_KEEP
// ONLY — setting the aim point before the law would leave it inert on a
// PN->station-keep switch.  Law first, aim point second, always.
static void applyGuidanceConfig()
{
    static_assert((uint8_t)TR_GuidancePN::MODE_PN           == GUIDE_LAW_PN,
                  "GUIDE_LAW_PN must mirror TR_GuidancePN::MODE_PN");
    static_assert((uint8_t)TR_GuidancePN::MODE_STATION_KEEP == GUIDE_LAW_STATION_KEEP,
                  "GUIDE_LAW_STATION_KEEP must mirror TR_GuidancePN::MODE_STATION_KEEP");

    if (pn_guidance_law == GUIDE_LAW_STATION_KEEP) {
        guidance.configureStationKeep(pn_kp_pos, pn_kd_vel, pn_max_accel);
    } else {
        // OVERHEAD PN: target = (0,0,pn_target_alt); the 3-arg form pins
        // horizontal at (0,0) and ignores the aim point set below.
        guidance.configure(pn_nav_gain, pn_max_accel, pn_target_alt);
    }

    // Belt-and-braces magnitude gate.  The wire handler already rejects an
    // over-radius aim point, so the only way to arrive here out of range is a
    // corrupt / stale NVS record; fail SAFE to the pad rather than fly it.
    const float r = sqrtf(pn_target_e * pn_target_e + pn_target_n * pn_target_n);
    if (!std::isfinite(r) || r > config::GUIDANCE_MAX_AIM_RADIUS_M) {
        ESP_LOGW(TAG, "[GUID CFG] stored aim point (%.1f,%.1f) r=%.1f m exceeds %.0f m "
                      "limit — forcing (0,0) overhead",
                      (double)pn_target_e, (double)pn_target_n, (double)r,
                      (double)config::GUIDANCE_MAX_AIM_RADIUS_M);
        pn_target_e = 0.0f;
        pn_target_n = 0.0f;
    }
    guidance.setHorizontalTarget(pn_target_e, pn_target_n);   // inert in MODE_PN
}

// #435: convert a WGS84 geodetic point to pad-relative ENU.  The math is the
// pure host-tested guidanceLlaToEnu() in GuidancePointGate.h (pinned against
// known geodesy values — it must agree with the EKF position path in
// loop_fc's imu_pos block or the flown miss distance silently differs from
// the commanded one); this wrapper only binds the launch-site reference
// statics.  Inputs in degrees; ref_* are radians.  Returns false (and leaves
// e/n untouched) when no launch-site reference exists yet.
static bool guidanceLlaToEnu(double lat_deg, double lon_deg, float& e, float& n)
{
    if (!have_ref_pos)
    {
        return false;
    }
    guidanceLlaToEnu(lat_deg, lon_deg, ref_lat_rad, ref_lon_rad, ref_alt_m, e, n);
    return true;
}

// Build a snapshot of the active roll-control / IMU settings (#165).  Reads
// the live PID gains from servo_control (these can be NVS- or BLE-overridden),
// the runtime override globals, the config:: defaults for the compile-time-only
// knobs, and the active roll profile.  Logged once at flight-start so the
// per-flight .json records exactly what flew.
static void buildFlightSettings(FlightSettingsData& s)
{
    memset(&s, 0, sizeof(s));
    s.time_us = time_us();
    s.version = FlightSettingsData::VERSION;

    uint8_t flags = 0;
    if (use_angle_control)  flags |= (uint8_t)(1u << FlightSettingsData::F_USE_ANGLE_CONTROL);
    if (gain_sched_enabled) flags |= (uint8_t)(1u << FlightSettingsData::F_GAIN_SCHEDULE);
    if (guidance_enabled)   flags |= (uint8_t)(1u << FlightSettingsData::F_GUIDANCE);
    if (servo_enabled)      flags |= (uint8_t)(1u << FlightSettingsData::F_SERVO_ENABLED);
    if (FW_GIT_DIRTY)       flags |= (uint8_t)(1u << FlightSettingsData::F_FW_DIRTY);
    if (enable_sounds)      flags |= (uint8_t)(1u << FlightSettingsData::F_SOUNDS);
    if (pn_guidance_law == GUIDE_LAW_STATION_KEEP)
                            flags |= (uint8_t)(1u << FlightSettingsData::F_GUIDANCE_STATION_KEEP);
    s.flags = flags;
    s.roll_delay_ms = roll_delay_ms;

    // Inner rate PID — live base gains (NVS/BLE-overridable, so read from
    // servo_control rather than config:: which is only the boot default).
    s.kp          = servo_control.getKp();
    s.ki          = servo_control.getKi();
    s.kd          = servo_control.getKd();
    s.d_lpf_hz    = config::D_FILTER_CUTOFF_HZ;
    s.min_cmd_deg = servo_control.getMinCmd();
    s.max_cmd_deg = servo_control.getMaxCmd();

    // Outer (cascaded angle) loop.  kp_angle is now runtime/app-overridable.
    s.kp_angle              = kp_angle_outer;
    s.kp_angle_rate_cap_dps = kp_angle_rate_cap_dps;

    // Gain schedule.  v_ref/v_min are compile-time only; the cap is owned by
    // TR_ServoControl.
    s.gs_v_ref     = config::GAIN_SCHEDULE_V_REF;
    s.gs_v_min     = config::GAIN_SCHEDULE_V_MIN;
    s.gs_scale_cap = TR_ServoControl::GAIN_SCHEDULE_SCALE_CAP;

    s.roll_rate_set_point = config::ROLL_RATE_SET_POINT;

    s.ism6_low_g_fs_g  = config::ISM6_LOW_G_FS_G;
    s.ism6_high_g_fs_g = config::ISM6_HIGH_G_FS_G;
    s.ism6_gyro_fs_dps = config::ISM6_GYRO_FS_DPS;
    s.ism6_update_rate_hz = sensor_collector_hw.ism6Rate();  // live (v5+)
    // Dynamic mode: the rate above is the live one at the snapshot (taken at
    // launch, so the boost rate). The log steps down to
    // IMU_RATE_DYNAMIC_POST_HZ at the first frame carrying NSF2_DEPLOYED —
    // this flag tells a reader to expect that step instead of one fixed rate.
    if (imuRateIsDynamic(imu_rate_setting))
    {
        s.flags |= (uint8_t)(1u << FlightSettingsData::F_IMU_RATE_DYNAMIC);
    }

    // Servo trim + timing (live values from servo_control).
    for (int i = 0; i < 4; ++i) {
        s.servo_bias_us[i] = (int16_t)servo_control.getServoBiasUs(i);
    }
    s.servo_hz     = (int16_t)servo_control.getServoHz();
    s.servo_min_us = (int16_t)servo_control.getServoMinUs();
    s.servo_max_us = (int16_t)servo_control.getServoMaxUs();
    s.fin_min_deg  = servo_control.getFinMinDeg();
    s.fin_max_deg  = servo_control.getFinMaxDeg();

    s.camera_type = runtime_camera_type;
    s.pyro        = pyro_config;

    // fw_git_sha buffer was zeroed by memset above, so strncpy of at most
    // size-1 chars leaves it NUL-terminated.
    strncpy(s.fw_git_sha, FW_GIT_SHA, sizeof(s.fw_git_sha) - 1);

    s.roll_profile = roll_profile;

    // Board→rocket mounting orientation that actually flew (v2).
    s.b2r_code = b2r_active_code;
    s.b2r_mode = b2r_active_mode;
    s.b2r_residual_cdeg = b2r_active_residual_cdeg;
    for (int i = 0; i < 4; ++i) {
        s.b2r_q[i] = (int16_t)lroundf(b2r_active_quat[i] * ORIENT_QUAT_WIRE_SCALE);
    }

    // Flown guidance target (v6+, #435).  Emitted at launch+0/100/200 ms —
    // AFTER enterInflight()'s re-conversion — so a cmd-28 point lands here
    // with the flown E/N values.  src records where the point came from.
    s.guid_tgt_e_m = pn_target_e;
    s.guid_tgt_n_m = pn_target_n;
    s.guid_tgt_src = guidanceFlownTargetSrc(guid_point_valid, pn_target_mode);
}

static void sendFlightSettings()
{
    FlightSettingsData s;
    buildFlightSettings(s);
    (void)enqueueI2STx(FLIGHT_SETTINGS_MSG,
                       reinterpret_cast<const uint8_t*>(&s),
                       sizeof(s));
}

static SemaphoreHandle_t i2c_bus_mutex = nullptr;

// ==========================================================================
// SECTION: I2S sender task
// ==========================================================================
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
        // Phase 4 Layer 3: while an OTA image is streaming the link is flipped
        // to slave RX — stop driving TX entirely.  fc_i2s_mutex serializes the
        // actual channel access vs the flip/revert (double-checked under lock).
        if (fc_ota_data_mode)
        {
            delay_ms(5);
            continue;
        }
        if (fc_i2s_mutex) xSemaphoreTake(fc_i2s_mutex, portMAX_DELAY);
        if (fc_ota_data_mode)
        {
            if (fc_i2s_mutex) xSemaphoreGive(fc_i2s_mutex);
            continue;
        }
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
        if (fc_i2s_mutex) xSemaphoreGive(fc_i2s_mutex);
    }
}

// ==========================================================================
// SECTION: Status LED
// ==========================================================================
static inline void triggerBlueLedFlash(uint32_t now_ms)
{
    gpio_set_level((gpio_num_t)(config::BLUE_LED_PIN), 1);
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
        gpio_set_level((gpio_num_t)(config::BLUE_LED_PIN), 0);
        blue_led_flash_active = false;
    }
}

// ==========================================================================
// SECTION: Calibration status publishing
// ==========================================================================
// Issue #132 — publish a one-shot status frame built from whatever the
// "mag_cal" NVS namespace currently holds.  Used by MAG_CAL_APPLY (after
// persisting a pushed cal) and MAG_CAL_READ (pure query) so the app sees
// the FC's stored cal without waiting for the 5 Hz cadence — and without
// needing the MagCalibrator object, which is in IDLE outside an active
// cal flow.
static void publishMagCalStatusFromNVS()
{
    MagCalStatusData s{};
    s.time_us = (uint32_t)esp_timer_get_time();

    Preferences p;
    p.begin("mag_cal", true);  // read-only
    const bool has_cal = p.isKey("ver") &&
                        p.getUChar("ver", 0) == MAG_CAL_NVS_SCHEMA_VERSION &&
                        p.getBool("done", false);
    if (has_cal)
    {
        s.sub_type        = MAG_CAL_SUB_APPLIED;
        s.offset_x        = (int16_t)p.getShort("cx", 0);
        s.offset_y        = (int16_t)p.getShort("cy", 0);
        s.offset_z        = (int16_t)p.getShort("cz", 0);
        const float R     = p.getFloat("R_uT",   0.0f);
        const float res   = p.getFloat("res_uT", 0.0f);
        s.field_R_uT_x10  = (uint16_t)lroundf(R   * 10.0f);
        s.residual_uT_x10 = (uint16_t)lroundf(res * 10.0f);
        s.reject_code     = MAG_CAL_OK;
    }
    else
    {
        s.sub_type = MAG_CAL_SUB_IDLE;
    }
    p.end();

    uint8_t buf[sizeof(MagCalStatusData)];
    memcpy(buf, &s, sizeof(buf));
    (void)enqueueI2STx(MAG_CAL_STATUS_MSG, buf, sizeof(buf));
}

// Issue #132 — publish the gyro + high-g sensor cal currently stored in NVS
// (namespace "cal") so the app can snapshot it after a pad cal and diff it on
// connect.  valid=0 when nothing is stored.
static void publishSensorCalFromNVS()
{
    SensorCalStatusData s{};

    Preferences p;
    p.begin("cal", true);  // read-only
    const bool has_hg = p.isKey("hgbx");
    const bool has_gyro = p.isKey("gx");
    if (has_hg || has_gyro)
    {
        s.valid  = 1;
        s.gyro_x = (int16_t)p.getShort("gx", 0);
        s.gyro_y = (int16_t)p.getShort("gy", 0);
        s.gyro_z = (int16_t)p.getShort("gz", 0);
        s.hg_x   = p.getFloat("hgbx", 0.0f);
        s.hg_y   = p.getFloat("hgby", 0.0f);
        s.hg_z   = p.getFloat("hgbz", 0.0f);
    }
    p.end();

    uint8_t buf[sizeof(SensorCalStatusData)];
    memcpy(buf, &s, sizeof(buf));
    (void)enqueueI2STx(SENSOR_CAL_STATUS_MSG, buf, sizeof(buf));
}

// ==========================================================================
// SECTION: Piezo buzzer
// ==========================================================================
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
        gpio_set_level((gpio_num_t)(config::PIEZO_PIN), 0);
    }
}

// #382: no now_us parameter — every caller passed a 32-bit-truncated
// time_us(), but piezoToggleCb compares against the full 64-bit
// esp_timer_get_time(). Past ~71.6 min of uptime the truncated "now" wrapped
// small, the computed end time sat in the 64-bit past, and every beep (and
// its LED flash) terminated on the first toggle callback. Read the full
// clock here so the mistake can't be reintroduced at a call site.
static inline void piezoStart(uint32_t freq_hz, uint32_t duration_ms)
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
    const int64_t now_us64 = esp_timer_get_time();
    piezo_wave_end_us = now_us64 + ((int64_t)duration_ms * 1000LL);
    piezo_pin_high = false;
    piezo_wave_active = true;
    gpio_set_level((gpio_num_t)(config::PIEZO_PIN), 0);
    // Same wrapping-ms domain as serviceBlueLedFlash's now_ms (both derive
    // (uint32_t)(esp_timer_get_time()/1000)), so the wrap-safe signed
    // comparison there stays correct.
    triggerBlueLedFlash((uint32_t)(now_us64 / 1000LL));

    (void)esp_timer_stop(piezo_toggle_timer);
    if (esp_timer_start_periodic(piezo_toggle_timer, piezo_half_period_us) != ESP_OK)
    {
        piezoStop();
    }
}

// ==========================================================================
// SECTION: Camera control (GoPro pulse, RunCam serial)
// ==========================================================================
// ── GoPro control (GPIO pulse on shutter pin) ──
static inline void startGoProPulse(uint32_t now_ms)
{
    if ((config::CAM_SHUTTER_PIN < 0) || !config::USE_GOPRO)
        return;
    gpio_set_level((gpio_num_t)(config::CAM_SHUTTER_PIN), 0);
    gopro_pulse_active = true;
    gopro_pulse_end_ms = now_ms + (uint32_t)config::GOPRO_PULSE_MS;
}

static inline void serviceGoProPulse(uint32_t now_ms)
{
    if (!gopro_pulse_active)
        return;
    if ((int32_t)(now_ms - gopro_pulse_end_ms) >= 0)
    {
        gpio_set_level((gpio_num_t)(config::CAM_SHUTTER_PIN), 1);
        gopro_pulse_active = false;
    }
}

// ── RunCam control (UART command to toggle recording) ──
// RunCam Split 4 power-button message toggles recording on/off.
static const uint8_t RUNCAM_PWR_CMD[] = {0xCC, 0x01, 0x01, 0xE7};
static constexpr uart_port_t RUNCAM_UART_PORT = UART_NUM_2;
static bool runcam_uart_ready = false;

// RCDEVICE feature bitfield from GET_DEVICE_INFO (0 until a probe succeeds).
// Bit 0x40 = explicit START_RECORDING support, which we prefer over the
// power-button toggle: on the bench unit the toggle only blips the LED without
// holding record, whereas START_RECORDING holds it (#234).
static constexpr uint16_t RUNCAM_FEAT_START_RECORDING = 0x0040;
static uint16_t runcam_features = 0;

static void runcamUartPark();    // defined below with runcamUartAttach
static void runcamUartAttach();

static void initRunCam()
{
    if (!config::USE_RUNCAM)
        return;

    // Configure pins but keep camera powered OFF at boot.
    // Camera is powered on only when the user presses "Start Recording".
    if (config::RUNCAM_PWR_PIN >= 0)
    {
        gpio_set_direction((gpio_num_t)(config::RUNCAM_PWR_PIN), GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)(config::RUNCAM_PWR_PIN), 0);  // OFF at boot
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
    // Camera is off at boot: park the pins immediately so the idle-high TX
    // can't phantom-feed it (battery cold-start fix, see config.h).
    runcamUartPark();
    ESP_LOGI(TAG, "RunCam UART ready on RX=%d TX=%d @ %lu baud (pins parked, camera OFF)",
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

// ── RunCam Device Protocol RX (issue #146) ──
// The camera-control command (RUNCAM_PWR_CMD, 0xCC 0x01 ...) is
// fire-and-forget: the camera sends no reply.  That's almost certainly
// why listening on the RX pin after a record/toggle command never
// showed anything.  GET_DEVICE_INFO (command 0x00) is the one request
// the camera always answers — a 5-byte reply with the protocol version
// and a feature bitfield.  Querying it both proves the RX wire works and
// tells us whether this camera supports explicit start/stop recording or
// only the power-button toggle (informs the #146 fix).

// RunCam Device Protocol CRC8: poly 0xD5, init 0x00, no reflection.
// (runCamCrc8({0xCC,0x01,0x01}) == 0xE7, matching RUNCAM_PWR_CMD.)
static uint8_t runCamCrc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5)
                               : (uint8_t)(crc << 1);
    }
    return crc;
}

// GET_DEVICE_INFO request frame: header, command 0x00, CRC8.
static const uint8_t RUNCAM_DEVICE_INFO_CMD[] = {0xCC, 0x00, 0x60};

// One GET_DEVICE_INFO exchange.  Returns true on a header- and CRC-valid
// 5-byte reply, caching the feature bitfield in runcam_features.  Quiet on
// failure (the start poll calls this repeatedly); the caller logs the final
// timeout.  Also the RX-wire bench-test path for #146.
static bool runCamProbeOnce()
{
    if (!config::USE_RUNCAM || !runcam_uart_ready)
        return false;

    // Drop stale boot-chatter so the reply parses cleanly.
    uint8_t scratch[32];
    while (uart_read_bytes(RUNCAM_UART_PORT, scratch, sizeof(scratch), 0) > 0) {}

    uart_write_bytes(RUNCAM_UART_PORT, RUNCAM_DEVICE_INFO_CMD,
                     sizeof(RUNCAM_DEVICE_INFO_CMD));

    uint8_t resp[5] = {0};
    int n = uart_read_bytes(RUNCAM_UART_PORT, resp, sizeof(resp),
                            pdMS_TO_TICKS(config::RUNCAM_PROBE_READ_MS));
    if (n < (int)sizeof(resp) || resp[0] != 0xCC)
        return false;
    if (runCamCrc8(resp, 4) != resp[4])
        return false;

    // resp[2..3] is the 16-bit feature bitfield, little-endian (bench reply
    // cc 01 77 00 → 0x0077).
    runcam_features = (uint16_t)resp[2] | ((uint16_t)resp[3] << 8);
    ESP_LOGI(TAG, "RunCam ALIVE — protocol v%u, features 0x%04X",
             resp[1], runcam_features);
    ESP_LOG_BUFFER_HEX(TAG, resp, n);
    return true;
}

// START_RECORDING (CAMERA_CONTROL 0x03).  The unit advertises this (feature
// 0x40) and holds record on it, unlike the power-button toggle which only
// blips the LED (#234) — so the start path always uses this, even when the
// readiness probe never answered.
static void sendRunCamStartRecording()
{
    if (!config::USE_RUNCAM || !runcam_uart_ready)
        return;
    uint8_t rec[4] = {0xCC, 0x01, 0x03, 0x00};
    rec[3] = runCamCrc8(rec, 3);
    uart_write_bytes(RUNCAM_UART_PORT, rec, sizeof(rec));
}

// ── UART park/attach around camera power (battery cold-start fix) ──
// V8 wires the UART pins straight to the connector (no series R — R30/R32
// are V9) and switches the camera's RETURN, so an idle-high TX phantom-feeds
// the "off" camera through its ESD diodes and its power-on-reset never sees
// a clean rise (see config.h).  Park both pins high-impedance (input, no
// pulls) whenever the camera is unpowered; attach the UART matrix only while
// the gate is up.  Nothing ever drives an unpowered camera.
static void runcamUartPark()
{
    if (!config::USE_RUNCAM || !runcam_uart_ready)
        return;
    gpio_reset_pin((gpio_num_t)config::RUNCAM_TX_PIN);
    gpio_reset_pin((gpio_num_t)config::RUNCAM_RX_PIN);
    gpio_set_direction((gpio_num_t)config::RUNCAM_TX_PIN, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)config::RUNCAM_RX_PIN, GPIO_MODE_INPUT);
    // gpio_reset_pin leaves the pull-up enabled — remove it; even a weak
    // pull is a phantom-feed path on a resistor-less line.
    gpio_set_pull_mode((gpio_num_t)config::RUNCAM_TX_PIN, GPIO_FLOATING);
    gpio_set_pull_mode((gpio_num_t)config::RUNCAM_RX_PIN, GPIO_FLOATING);
}

// Reattach the UART matrix (same routing as init; safe to call when already
// attached — the stop path calls it unconditionally so an aborted start can
// never leave the pins parked when a toggle needs to transmit).
static void runcamUartAttach()
{
    if (!config::USE_RUNCAM || !runcam_uart_ready)
        return;
    uart_set_pin(RUNCAM_UART_PORT,
                 (int)config::RUNCAM_TX_PIN,
                 (int)config::RUNCAM_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
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
        // Attach the parked UART, power on, then poll for readiness from
        // serviceCameraStart so the flight task keeps feeding the watchdog
        // (#146).  The camera boots to IDLE and does NOT auto-record over
        // UART (#234); serviceCameraStart commands START_RECORDING as soon
        // as it answers GET_DEVICE_INFO.
        runcamUartAttach();
        if (config::RUNCAM_PWR_PIN >= 0)
            gpio_set_level((gpio_num_t)(config::RUNCAM_PWR_PIN), 1);
        gopro_recording = true;
        runcam_features = 0;  // re-probe each power cycle; don't trust a stale cache
        camera_start_power_ms = now_ms;
        camera_power_retries_left = config::RUNCAM_POWER_RETRIES;
        camera_start_phase = CameraStartPhase::WaitingForBoot;
        camera_start_due_ms = now_ms + config::RUNCAM_BOOT_MIN_MS;
        camera_probe_deadline_ms = now_ms + config::RUNCAM_BOOT_MAX_MS;
        ESP_LOGI(TAG, "Camera START (RunCam) — UART attached, power ON, polling for boot...");
    }
}

static inline void serviceCameraStop(uint32_t now_ms);

// Queue a camera stop.  delay_ms = 0 (default) drives the first phase
// inline so the GPIO/UART action happens in this call (e.g. user manual
// stop).  delay_ms > 0 schedules — LANDED uses CAMERA_STOP_DELAY_MS to
// capture post-impact footage and to avoid stalling the i2s_tx_queue
// long enough to drop END_FLIGHT (#141).  Either way, the RunCam's 500 ms
// toggle-to-power-off step runs non-blockingly via serviceCameraStop().
static void cameraStop(uint32_t now_ms, uint32_t delay_ms = 0)
{
    if (!gopro_recording) return;                            // not recording
    if (camera_stop_phase != CameraStopPhase::Idle) return;  // already stopping
    // A stop during the RunCam boot wait abandons the pending probe;
    // serviceCameraStop powers the camera off regardless.
    camera_start_phase = CameraStartPhase::Idle;
    camera_stop_phase = CameraStopPhase::DelayBeforeStop;
    camera_stop_due_ms = now_ms + delay_ms;
    if (delay_ms > 0)
    {
        ESP_LOGI(TAG, "Camera STOP scheduled in %lu ms",
                 (unsigned long)delay_ms);
        return;
    }
    // Execute first phase immediately so the user-visible stop is sync.
    serviceCameraStop(now_ms);
}

static inline void serviceCameraStop(uint32_t now_ms)
{
    if (camera_stop_phase == CameraStopPhase::Idle) return;
    if ((int32_t)(now_ms - camera_stop_due_ms) < 0) return;

    if (camera_stop_phase == CameraStopPhase::DelayBeforeStop)
    {
        if (runtime_camera_type == CAM_TYPE_GOPRO)
        {
            startGoProPulse(now_ms);
            ESP_LOGI(TAG, "Camera STOP (GoPro)");
            gopro_recording = false;
            camera_stop_phase = CameraStopPhase::Idle;
        }
        else if (runtime_camera_type == CAM_TYPE_RUNCAM)
        {
            // A stop can land mid-retry (pins parked); reattach the UART
            // unconditionally so the toggle actually transmits.
            runcamUartAttach();
            sendRunCamToggle();
            ESP_LOGI(TAG, "RunCam toggle sent — power-off in 500 ms");
            camera_stop_phase = CameraStopPhase::RunCamToggleSent;
            camera_stop_due_ms = now_ms + 500U;  // let the stop command process
        }
        else
        {
            gopro_recording = false;
            camera_stop_phase = CameraStopPhase::Idle;
        }
    }
    else  // RunCamToggleSent
    {
        if (config::RUNCAM_PWR_PIN >= 0)
            gpio_set_level((gpio_num_t)(config::RUNCAM_PWR_PIN), 0);
        runcamUartPark();  // unpowered camera: nothing may feed it (config.h)
        ESP_LOGI(TAG, "Camera STOP (RunCam) — powered off, pins parked");
        gopro_recording = false;
        camera_stop_phase = CameraStopPhase::Idle;
    }
}

// Drives the deferred RunCam start queued by cameraStart, serviced from the
// main loop so it can't stall the flight task into a watchdog reset (#146).
// The camera cold-boots and only answers UART once ready, so from BOOT_MIN_MS
// we poll GET_DEVICE_INFO every PROBE_INTERVAL_MS and command START_RECORDING
// the instant it answers (typically sooner than the old fixed 5 s wait).  If
// it never answers by BOOT_MAX_MS we record blind — the camera supports
// START_RECORDING, so that beats the record-less power toggle the old code
// fell back to.  START_RECORDING is fire-and-forget (no ACK), so we resend it
// a few times.  Each probe's RX read is bounded by RUNCAM_PROBE_READ_MS.
static inline void serviceCameraStart(uint32_t now_ms)
{
    if (camera_start_phase == CameraStartPhase::Idle) return;
    if ((int32_t)(now_ms - camera_start_due_ms) < 0) return;

    if (camera_start_phase == CameraStartPhase::RetryOff)
    {
        // Off-time elapsed: reattach the UART and reapply power.  If launch
        // happened during the off-time, same rule as the probe path — no
        // blocking UART reads in boost, start blind immediately.
        runcamUartAttach();
        if (config::RUNCAM_PWR_PIN >= 0)
            gpio_set_level((gpio_num_t)(config::RUNCAM_PWR_PIN), 1);
        camera_start_power_ms = now_ms;
        if (rocket_state == INFLIGHT)
        {
            ESP_LOGW(TAG, "RunCam: launch during power-cycle retry — starting blind NOW");
            sendRunCamStartRecording();
            camera_record_resends_left = config::RUNCAM_RECORD_RESENDS;
            camera_start_phase = CameraStartPhase::ResendRecord;
            camera_start_due_ms = now_ms + config::RUNCAM_RECORD_RESEND_MS;
            return;
        }
        camera_start_phase = CameraStartPhase::WaitingForBoot;
        camera_start_due_ms = now_ms + config::RUNCAM_BOOT_MIN_MS;
        camera_probe_deadline_ms = now_ms + config::RUNCAM_BOOT_MAX_MS;
        ESP_LOGI(TAG, "RunCam: power reapplied — polling for boot again");
        return;
    }

    if (camera_start_phase == CameraStartPhase::WaitingForBoot)
    {
        ESP_LOGI(TAG, "RunCam boot grace elapsed — polling for UART readiness");
        camera_start_phase = CameraStartPhase::Probing;
        // fall through and probe immediately (due is already <= now)
    }

    if (camera_start_phase == CameraStartPhase::Probing)
    {
        // #568: each probe blocks up to RUNCAM_PROBE_READ_MS (60 ms) on the
        // camera UART — fine on the pad, but if the operator starts the
        // camera and launches inside the probe window (~8 s), those stalls
        // would land in early boost. The flight loop must never wait on a
        // camera in flight: on launch detect, skip the blocking probe and
        // start recording blind immediately (fire-and-forget START_RECORDING
        // — blind beats both a stalled boost loop and a camera that never
        // records).
        const bool in_flight = (rocket_state == INFLIGHT);
        const bool alive = in_flight ? false : runCamProbeOnce();
        const bool deadline = (int32_t)(now_ms - camera_probe_deadline_ms) >= 0;
        if (!alive && !deadline && !in_flight)
        {
            camera_start_due_ms = now_ms + config::RUNCAM_PROBE_INTERVAL_MS;
            return;  // keep polling
        }
        // Whole window silent on the pad: park + power-cycle and retry
        // before the blind fallback (battery cold-start phantom feed — see
        // config.h).  Gate off AND pins parked, so nothing feeds the camera
        // and its own load drains it during the off-time.
        if (!alive && !in_flight && camera_power_retries_left > 0)
        {
            camera_power_retries_left--;
            if (config::RUNCAM_PWR_PIN >= 0)
                gpio_set_level((gpio_num_t)(config::RUNCAM_PWR_PIN), 0);
            runcamUartPark();
            camera_start_phase = CameraStartPhase::RetryOff;
            camera_start_due_ms = now_ms + config::RUNCAM_RETRY_OFF_MS;
            ESP_LOGW(TAG, "RunCam: no UART reply %lu ms after power-on — "
                          "parked+power-cycling (%u retr%s left)",
                     (unsigned long)(now_ms - camera_start_power_ms),
                     (unsigned)camera_power_retries_left,
                     camera_power_retries_left == 1 ? "y" : "ies");
            return;
        }
        if (alive)
            ESP_LOGI(TAG, "RunCam ready %lu ms after power-on (feat 0x%04X%s)",
                     (unsigned long)(now_ms - camera_start_power_ms), runcam_features,
                     (runcam_features & RUNCAM_FEAT_START_RECORDING)
                         ? "" : " — START_RECORDING not advertised!");
        else if (in_flight)
            ESP_LOGW(TAG, "RunCam: launch during boot probe — starting blind NOW "
                          "(no blocking UART reads in boost)");
        else
            ESP_LOGW(TAG, "RunCam: no GET_DEVICE_INFO reply in %lu ms — recording "
                          "blind (check RX wire to FC pin %d and the camera's "
                          "UART-control menu)",
                     (unsigned long)(now_ms - camera_start_power_ms),
                     (int)config::RUNCAM_RX_PIN);
        // Command record start deterministically — never the blip-only toggle.
        sendRunCamStartRecording();
        ESP_LOGI(TAG, "RunCam START_RECORDING sent (%s)", alive ? "probe OK" : "blind");
        camera_record_resends_left = config::RUNCAM_RECORD_RESENDS;
        camera_start_phase = CameraStartPhase::ResendRecord;
        camera_start_due_ms = now_ms + config::RUNCAM_RECORD_RESEND_MS;
        return;
    }

    // ResendRecord: re-issue START_RECORDING a few times (fire-and-forget).
    sendRunCamStartRecording();
    ESP_LOGI(TAG, "RunCam START_RECORDING resent (%u remaining)",
             (unsigned)(camera_record_resends_left - 1));
    if (--camera_record_resends_left == 0)
        camera_start_phase = CameraStartPhase::Idle;
    else
        camera_start_due_ms = now_ms + config::RUNCAM_RECORD_RESEND_MS;
}

// ==========================================================================
// SECTION: Boot chirp and heartbeat beep
// ==========================================================================
static inline void startBootReadyChirp(uint32_t now_ms)
{
    if (!enable_sounds || !piezo_pwm_ready)
    {
        return;
    }
    piezoStart(2600, 90);
    boot_chirp_phase = BootChirpPhase::GapAfterBeep1;
    boot_chirp_next_ms = now_ms + 150U; // 90ms beep + 60ms gap
}

static inline void serviceBootReadyChirp(uint32_t now_ms)
{
    switch (boot_chirp_phase)
    {
        case BootChirpPhase::Idle:
            return;
        case BootChirpPhase::GapAfterBeep1:
            if ((int32_t)(now_ms - boot_chirp_next_ms) >= 0)
            {
                piezoStart(1900, 130);
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
               config::HEARTBEAT_BEEP_DURATION_MS);
}

// ── OTA rollback gate (#8 Phase 4 / Layer 4, #13) ─────────────────────────
// A freshly OTA-installed FC image boots in ESP_OTA_IMG_PENDING_VERIFY (rollback
// is enabled in sdkconfig). Unless we call esp_ota_mark_app_valid_cancel_rollback()
// the bootloader auto-reverts to the previous partition on the next reset — the
// safety net against a bad image. setup_fc() detects pending-verify at boot;
// fcMaybeMarkOtaValid() (called every flight-loop tick) confirms the image only
// after it has run the real flight loop stably for ~10 s. A broken image that
// boot-loops, hangs (WDT reset), or crashes never reaches the mark, so it rolls
// back. Mirrors the OC's maybeMarkOtaValid().
static bool fc_ota_pending_verify = false;

// ==========================================================================
// SECTION: OTA boot validation
// ==========================================================================
static inline void fcMaybeMarkOtaValid()
{
    if (!fc_ota_pending_verify) return;
    static int64_t first_us = 0;                 // first tick after setup_fc() completed
    const int64_t now = esp_timer_get_time();
    if (first_us == 0) { first_us = now; return; }
    if ((now - first_us) < 10LL * 1000000LL) return;   // need ~10 s of healthy running
    const esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK)
        ESP_LOGW(TAG, "[OTA] new image validated after 10 s stable run; rollback cancelled");
    else
        ESP_LOGE(TAG, "[OTA] mark_app_valid_cancel_rollback failed: %s", esp_err_to_name(err));
    fc_ota_pending_verify = false;
}

// ==========================================================================
// SECTION: Boot setup
// ==========================================================================
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

    // OTA boot-state check (#8 Layer 4). A just-OTA'd image boots PENDING_VERIFY;
    // arm the rollback gate so fcMaybeMarkOtaValid() confirms it only after a
    // stable run. A normal USB-flashed image is UNDEFINED here, so this is a no-op.
    {
        const esp_partition_t* running = esp_ota_get_running_partition();
        esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
        if (running && esp_ota_get_state_partition(running, &st) == ESP_OK &&
            st == ESP_OTA_IMG_PENDING_VERIFY)
        {
            fc_ota_pending_verify = true;
            ESP_LOGW(TAG, "[OTA] running PENDING_VERIFY image (partition '%s' @ 0x%08x); "
                          "rollback armed until ~10 s stable run",
                     running->label, (unsigned)running->address);
        }
    }

    // The flight task runs continuously and starves IDLE tasks on CPU 1, so we
    // stop the WDT monitoring IDLE cores (idle_core_mask = 0) — IDLE starvation
    // here is normal, not a fault. The flight task itself IS subscribed
    // (esp_task_wdt_add) and resets the WDT each ~1 ms iteration.
    // #261: trigger_panic = true so that if loop_fc() ever stalls past the 5 s
    // timeout (deadlock, wedged peripheral), the WDT panics → reboot → snapshot
    // / reboot-recovery, instead of only logging and leaving the highest-priority
    // core-1 task hung and the vehicle dead.
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = 5000,
        .idle_core_mask = 0,       // don't monitor any IDLE tasks
        .trigger_panic = true,     // #261: hung flight loop -> reboot + recovery
    };
    esp_task_wdt_reconfigure(&wdt_cfg);

    // Quiet the IDF gpio driver — each gpio_config() (CS pins, INT pins,
    // pyro pins) prints a multi-field INFO line that drowns the rest of
    // boot.  Drop to WARN so genuine GPIO problems still surface.
    esp_log_level_set("gpio", ESP_LOG_WARN);

    // Pyro channels: safe pins FIRST, before any other peripheral init
    initPyroPins();

    // Power gates (#411): on V8 the FC switches the GNSS + servo rails.
    // Raised before any dependent init (the GNSS boot probe and the servo
    // boot wiggle both need their rails up); -1 = rail not FC-switched.
    // SERVO_ACT is the #345 servo-power MOSFET — held on for now;
    // integrating it with SERVO_RELAX_ON_PAD is a follow-up.
    if (config::GPS_ACT_PIN >= 0)
    {
        gpio_set_direction((gpio_num_t)config::GPS_ACT_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)config::GPS_ACT_PIN, 1);
    }
    if (config::SERVO_ACT_PIN >= 0)
    {
        gpio_set_direction((gpio_num_t)config::SERVO_ACT_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)config::SERVO_ACT_PIN, 1);
    }

    ESP_LOGI(TAG, "Starting ....");
    // Board revision + the pyro pins it implies. The pyro map is the one part
    // of the pin map whose mismatch has no runtime symptom (V8 and V9 swap
    // FIRE 2/3 and move ARM, but keep all four CONT pins), so print the
    // numbers rather than just the revision — they can be checked against the
    // connector with a meter before anything is armed.
    ESP_LOGW(TAG, "[BOARD] pin map: %s  (select with -DTR_BOARD_V7/V8/V9=1; no default)",
             TR_BOARD_REV_STR);
    ESP_LOGW(TAG, "[BOARD] pyro: ARM=%d FIRE=%d/%d/%d/%d CONT=%d/%d/%d/%d",
             (int)config::PYRO_ARM_PIN,
             (int)config::PYRO1_FIRE_PIN, (int)config::PYRO2_FIRE_PIN,
             (int)config::PYRO3_FIRE_PIN, (int)config::PYRO4_FIRE_PIN,
             (int)config::PYRO1_CONT_PIN, (int)config::PYRO2_CONT_PIN,
             (int)config::PYRO3_CONT_PIN, (int)config::PYRO4_CONT_PIN);
    gpio_set_direction((gpio_num_t)(config::RED_LED_PIN), GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)(config::RED_LED_PIN), 1);
    gpio_set_direction((gpio_num_t)(config::BLUE_LED_PIN), GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)(config::BLUE_LED_PIN), 0);

    // Disable all SPI chip-selects before bus init to avoid MISO contention.
    if (config::MMC5983MA_CS >= 0)  // part absent on V8 (#411)
    {
        gpio_set_direction((gpio_num_t)(config::MMC5983MA_CS), GPIO_MODE_OUTPUT); gpio_set_level((gpio_num_t)(config::MMC5983MA_CS), 1);
    }
    gpio_set_direction((gpio_num_t)(config::BMP585_CS), GPIO_MODE_OUTPUT);    gpio_set_level((gpio_num_t)(config::BMP585_CS), 1);
    gpio_set_direction((gpio_num_t)(config::ISM6HG256_CS), GPIO_MODE_OUTPUT); gpio_set_level((gpio_num_t)(config::ISM6HG256_CS), 1);
    delay_ms(10);

    // SPI bus init — native IDF.  Matches what compat's SPIClass.begin()
    // was doing under the hood (SPI2_HOST, DMA auto, 4 KB transfer cap)
    // but lifted out of the shim so this file no longer depends on
    // compat.h's SPIClass.
    ESP_LOGI(TAG, "SPI init...");
    {
        // Pin naming follows the sensor's perspective in config.h:
        // SDO = "Sensor Data Out" → MISO on the controller; SDI = "Sensor
        // Data In" → MOSI on the controller.  The compat SPIClass.begin
        // used the same mapping (its signature was sck, miso, mosi, ss).
        spi_bus_config_t buscfg = {};
        buscfg.miso_io_num     = config::SPI_SDO;
        buscfg.mosi_io_num     = config::SPI_SDI;
        buscfg.sclk_io_num     = config::SPI_SCK;
        buscfg.quadwp_io_num   = -1;
        buscfg.quadhd_io_num   = -1;
        buscfg.max_transfer_sz = 4096;
        esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        }
    }
    delay_ms(10);

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
        while (1) { delay_ms(1000); }
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
        while (1) { delay_ms(1000); }
    }
    i2s_tx_queue = xQueueCreate(config::I2S_TX_QUEUE_LEN, sizeof(I2STxMessage));
    if (i2s_tx_queue == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create I2S TX queue");
        while (1) { delay_ms(1000); }
    }
    i2c_bus_mutex = xSemaphoreCreateMutex();
    if (i2c_bus_mutex == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create I2C bus mutex");
        while (1) { delay_ms(1000); }
    }
    // Phase 4 Layer 3: mutex serializing i2s_stream across i2sSenderTask and the
    // OTA flip/revert; created before the sender task starts so it's always set.
    fc_i2s_mutex = xSemaphoreCreateMutex();
    if (fc_i2s_mutex == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create I2S mutex");
        while (1) { delay_ms(1000); }
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

    // Phase 4 Layer 3: OTA image parser. Idle (blocked on notify) until the
    // I2S link is flipped to slave RX and the recv callback starts waking it.
    // Pinned to Core 1 (off the sensor core) at low priority — it only runs
    // during an OTA, when flight is not in progress.
    xTaskCreatePinnedToCore(fcOtaParserTask, "FC OTA RX", 4096, nullptr, 3,
                            &fc_ota_parser_task, 1);

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
    float   nvs_fin_min_deg = config::FIN_MIN_DEG;
    float   nvs_fin_max_deg = config::FIN_MAX_DEG;
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
        nvs_fin_min_deg = prefs.getFloat("fmin", config::FIN_MIN_DEG);
        nvs_fin_max_deg = prefs.getFloat("fmax", config::FIN_MAX_DEG);
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
        applyRollPidGains(kp, ki, kd, mincmd, maxcmd);  // #268: both roll PIDs
        ESP_LOGI(TAG, "NVS PID: kp=%.4f ki=%.4f kd=%.4f min=%.1f max=%.1f",
                      (double)kp, (double)ki, (double)kd,
                      (double)mincmd, (double)maxcmd);
    }
    gain_sched_enabled      = prefs.getBool("gs", config::GAIN_SCHEDULE_ENABLED);
    use_angle_control       = prefs.getBool("ac", config::USE_ANGLE_CONTROL);
    roll_delay_ms           = prefs.getUShort("rdly", config::ROLL_CONTROL_DELAY_MS);
    kp_angle_rate_cap_dps   = prefs.getFloat("rcap", config::KP_ANGLE_RATE_CAP_DPS);
    kp_angle_outer          = prefs.getFloat("kpang", config::KP_ANGLE);
    integral_sep_threshold_dps = prefs.getFloat("iwind", config::INTEGRAL_SEP_THRESHOLD_DPS);
    applyRollPidSepThreshold(integral_sep_threshold_dps);  // #268: both roll PIDs
    guidance_enabled        = prefs.getBool("guid_en", config::GUIDANCE_ENABLED);
    pn_nav_gain       = prefs.getFloat("gng", config::PN_NAV_GAIN);
    pn_max_accel      = prefs.getFloat("gma", config::PN_MAX_ACCEL_MPS2);
    pn_accel_to_fin   = prefs.getFloat("gaf", config::PN_ACCEL_TO_FIN_DEG);
    pn_max_fin_deg    = prefs.getFloat("gmf", config::PN_MAX_FIN_DEG);
    pn_min_speed      = prefs.getFloat("gms", config::PN_MIN_SPEED_MPS);
    pn_coast_delay_ms = prefs.getUShort("gcd", config::PN_COAST_DELAY_MS);
    pn_target_mode    = prefs.getUChar("gtm", config::PN_TARGET_MODE);
    // #568: whitelist the NVS-loaded mode exactly like the command path does
    // (and like pn_guidance_law below) — a corrupt / forward-version record
    // must not flow into guidance unchecked.
    if (pn_target_mode != GUIDE_TARGET_OVERHEAD &&
        pn_target_mode != GUIDE_TARGET_POINT) {
        ESP_LOGW(TAG, "[GUID CFG] stored target mode %u unknown — falling back "
                      "to OVERHEAD", (unsigned)pn_target_mode);
        pn_target_mode = GUIDE_TARGET_OVERHEAD;
    }
    pn_target_e       = prefs.getFloat("gte", config::PN_TARGET_E_M);
    pn_target_n       = prefs.getFloat("gtn", config::PN_TARGET_N_M);
    pn_target_alt     = prefs.getFloat("gta", config::PN_TARGET_ALT_M);
    // The NVS record IS the profile-owned aim point — seed the shadows the
    // cmd-65 reject path / sim reset revert to (see pn_profile_target_*).
    pn_profile_target_e = pn_target_e;
    pn_profile_target_n = pn_target_n;
    pn_kp_pos       = prefs.getFloat("gkp", config::PN_KP_POS_PER_S2);
    pn_kd_vel       = prefs.getFloat("gkd", config::PN_KD_VEL_PER_S);
    pn_guidance_law = prefs.getUChar("glw", config::GUIDANCE_LAW_DEFAULT);
    if (pn_guidance_law > GUIDE_LAW_MAX) {          // corrupt/forward NVS record
        ESP_LOGW(TAG, "[GUID CFG] stored law %u unknown — falling back to PN",
                      (unsigned)pn_guidance_law);
        pn_guidance_law = GUIDE_LAW_PN;
    }
    // Fin→servo layout (servo azimuth + reverse), shared by every mix site.
    float fin_az[4] = {
        prefs.getFloat("fa0", config::FIN_AZIMUTH_0_DEG),
        prefs.getFloat("fa1", config::FIN_AZIMUTH_1_DEG),
        prefs.getFloat("fa2", config::FIN_AZIMUTH_2_DEG),
        prefs.getFloat("fa3", config::FIN_AZIMUTH_3_DEG),
    };
    uint8_t fin_rev  = prefs.getUChar("frv",  config::FIN_REVERSE_MASK);
    uint8_t fin_rrev = prefs.getUChar("frrv", config::FIN_ROLL_REVERSE_MASK);
    prefs.end();
    ESP_LOGI(TAG, "Roll: kp_angle=%.2f rate_cap=%.0f iwindup=%.0f dps",
                  (double)kp_angle_outer, (double)kp_angle_rate_cap_dps,
                  (double)integral_sep_threshold_dps);
    ESP_LOGI(TAG, "Gain scheduling: %s", gain_sched_enabled ? "ON" : "OFF");
    ESP_LOGI(TAG, "Angle control: %s  Roll delay: %u ms  Rate cap: %.1f dps",
                  use_angle_control ? "ON" : "OFF", (unsigned)roll_delay_ms,
                  (double)kp_angle_rate_cap_dps);
#if TR_GUIDANCE_AVAILABLE
    ESP_LOGI(TAG, "Guidance: %s  law=%s (compiled in)",
                  guidance_enabled ? "ON" : "OFF",
                  pn_guidance_law == GUIDE_LAW_STATION_KEEP ? "STATION_KEEP" : "PN");
    if (pn_guidance_law == GUIDE_LAW_STATION_KEEP) {
        ESP_LOGI(TAG, "  station-keep: kp=%.2f /s^2 kd=%.2f /s maxA=%.0f m/s^2 aim=(%.1f,%.1f) m",
                      (double)pn_kp_pos, (double)pn_kd_vel, (double)pn_max_accel,
                      (double)pn_target_e, (double)pn_target_n);
    } else {
        ESP_LOGI(TAG, "  PN: N=%.1f maxA=%.0f m/s^2 alt=%.0f m (aim point ignored by this law)",
                      (double)pn_nav_gain, (double)pn_max_accel, (double)pn_target_alt);
    }
#else
    ESP_LOGW(TAG, "Guidance: NOT COMPILED IN (TR_GuidancePN submodule not initialized — stub active)");
#endif

    // Configure guidance and control mixer (law + gains + aim point).
    applyGuidanceConfig();
    control_mixer.configure(config::PN_PITCH_KP, config::PN_PITCH_KI, config::PN_PITCH_KD,
                            config::PN_YAW_KP,   config::PN_YAW_KI,   config::PN_YAW_KD,
                            config::PN_MAX_FIN_DEG,
                            config::GAIN_SCHEDULE_V_REF,
                            config::GAIN_SCHEDULE_V_MIN);
    if (gain_sched_enabled) {
        control_mixer.enableGainSchedule(config::GAIN_SCHEDULE_V_REF,
                                         config::GAIN_SCHEDULE_V_MIN);
    }
    control_mixer.setFinLayout(fin_az, fin_rev, fin_rrev);
    // The roll-reverse mask has to reach BOTH mixers: control_mixer serves the
    // guided / ground-test path, servo_control the roll-only path (powered
    // flight and guidance-off flight).  Pushing it to only one is what made
    // "Reverse roll" look inert — it flipped the ground test but not the flight.
    servo_control.setRollReverseMask(fin_rrev);
    ESP_LOGI(TAG, "[FIN CFG] az=[%.0f %.0f %.0f %.0f] rev=0x%X rollrev=0x%X",
                  (double)fin_az[0], (double)fin_az[1], (double)fin_az[2], (double)fin_az[3],
                  (unsigned)fin_rev, (unsigned)fin_rrev);

    // Restore the user-selected IMU logging rate setting (BLE cmd 67,
    // namespace "imu").  Staged into the collector BEFORE
    // sensor_collector.begin() below, which programs the chip ODR from it.
    // Whitelist on read so a corrupted NVS value can't run the IMU at an
    // unplanned rate.
    //
    // A boot always starts pre-deployment, so DYNAMIC resolves to the boost
    // rate here.  That includes a mid-flight reboot recovery: re-entering a
    // descent under canopy at the boost rate costs some flash and self-corrects
    // only if the detector fires again, which it may not have signal for — an
    // acceptable trade against the alternative of guessing the airframe is
    // already deployed and logging a boost at 960 Hz.
    prefs.begin("imu", false);
    if (prefs.isKey("rate"))
    {
        const uint16_t nvs_rate = prefs.getUShort("rate", config::ISM6HG256_UPDATE_RATE);
        if (imuRateSettingValid(nvs_rate))
        {
            imu_rate_setting = nvs_rate;
        }
        else
        {
            ESP_LOGW(TAG, "NVS IMU logging rate %u invalid — using default",
                     (unsigned)nvs_rate);
        }
    }
    {
        const uint16_t boot_hz = imuRateResolve(imu_rate_setting, /*deployed=*/false);
        sensor_collector_hw.setIsm6Rate(boot_hz);
        if (imuRateIsDynamic(imu_rate_setting))
        {
            ESP_LOGI(TAG, "IMU logging rate: DYNAMIC (%u Hz to deployment, then %u Hz)",
                     (unsigned)IMU_RATE_DYNAMIC_BOOST_HZ,
                     (unsigned)IMU_RATE_DYNAMIC_POST_HZ);
        }
        else
        {
            ESP_LOGI(TAG, "IMU logging rate: %u Hz (fixed)", (unsigned)boot_hz);
        }
    }
    prefs.end();

    // Restore high-g accelerometer bias from NVS (namespace "cal")
    prefs.begin("cal", false);  // read-write (creates namespace on first boot)
    if (prefs.isKey("hgbx"))
    {
        float bx = prefs.getFloat("hgbx", 0.0f);
        float by = prefs.getFloat("hgby", 0.0f);
        float bz = prefs.getFloat("hgbz", 0.0f);
        sensor_converter.setHighGBias(bx, by, bz);
        out_status_query_data.hg_bias_x_cmss = encodeHgBiasCmss(bx);
        out_status_query_data.hg_bias_y_cmss = encodeHgBiasCmss(by);
        out_status_query_data.hg_bias_z_cmss = encodeHgBiasCmss(bz);
        ESP_LOGI(TAG, "NVS HG bias: %.3f, %.3f, %.3f m/s²",
                      (double)bx, (double)by, (double)bz);
    }
    // Gyro zero-rate bias (#132) — applied after begin() (see below).
    if (prefs.isKey("gx"))
    {
        pending_gyro_cal_x = (int16_t)prefs.getShort("gx", 0);
        pending_gyro_cal_y = (int16_t)prefs.getShort("gy", 0);
        pending_gyro_cal_z = (int16_t)prefs.getShort("gz", 0);
        pending_gyro_cal_apply = true;
        ESP_LOGI(TAG, "NVS gyro cal: %d, %d, %d (apply after begin)",
                      (int)pending_gyro_cal_x, (int)pending_gyro_cal_y, (int)pending_gyro_cal_z);
    }
    prefs.end();

    // Restore magnetometer hard-iron offset from NVS (namespace "mag_cal", issue #96).
    // Apply to IIS2MDC OFFSET_X/Y/Z if the chip is active, and to the
    // MMC5983MA software-offset path in the converter regardless (the
    // converter is a no-op on uncalibrated MMC counts since defaults are
    // zero).  IIS2MDC offsets must be reapplied AFTER sensor_collector.begin()
    // because softReset() inside begin() zeroes them — ensure call order
    // below.  The converter offset is set here too even though the IIS2MDC
    // chip handles its own subtraction; this lets a single NVS schema cover
    // both mag chips without divergence.
    prefs.begin("mag_cal", false);
    if (prefs.isKey("ver"))
    {
        const uint8_t schema = prefs.getUChar("ver", 0);
        if (schema == MAG_CAL_NVS_SCHEMA_VERSION && prefs.getBool("done", false))
        {
            const int16_t cx = (int16_t)prefs.getShort("cx", 0);
            const int16_t cy = (int16_t)prefs.getShort("cy", 0);
            const int16_t cz = (int16_t)prefs.getShort("cz", 0);
            const float   R  = prefs.getFloat("R_uT", 0.0f);
            const int32_t mmc_cx = prefs.getInt("mmc_cx", 0);
            const int32_t mmc_cy = prefs.getInt("mmc_cy", 0);
            const int32_t mmc_cz = prefs.getInt("mmc_cz", 0);
            sensor_converter.setMMCOffset(mmc_cx, mmc_cy, mmc_cz);
            ESP_LOGI(TAG, "NVS mag_cal: offset=(%d,%d,%d) raw counts, R=%.2f µT (apply after sensor_collector.begin)",
                          (int)cx, (int)cy, (int)cz, (double)R);
            // Stash on the side; we'll apply to the IIS2MDC chip after begin()
            // returns (softReset inside begin() zeroes OFFSET_X/Y/Z).
            pending_mag_cx = cx;
            pending_mag_cy = cy;
            pending_mag_cz = cz;
            pending_mag_apply = true;
        }
        else if (schema != MAG_CAL_NVS_SCHEMA_VERSION)
        {
            ESP_LOGW(TAG, "NVS mag_cal: schema %u != %u, ignoring", schema, (unsigned)MAG_CAL_NVS_SCHEMA_VERSION);
        }
    }
    else
    {
        ESP_LOGI(TAG, "NVS mag_cal: none (uncalibrated)");
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
        ESP_LOGI(TAG, "NVS pyro: ch1=%u/%u/%.1f  ch2=%u/%u/%.1f  ch3=%u/%u/%.1f  ch4=%u/%u/%.1f",
                 pyro_config.ch1_enabled, pyro_config.ch1_trigger_mode, (double)pyro_config.ch1_trigger_value,
                 pyro_config.ch2_enabled, pyro_config.ch2_trigger_mode, (double)pyro_config.ch2_trigger_value,
                 pyro_config.ch3_enabled, pyro_config.ch3_trigger_mode, (double)pyro_config.ch3_trigger_value,
                 pyro_config.ch4_enabled, pyro_config.ch4_trigger_mode, (double)pyro_config.ch4_trigger_value);
    } else {
        ESP_LOGI(TAG, "NVS pyro: none (all four disabled) — stored=%u bytes, expected=%u",
                 (unsigned)pyro_cfg_sz, (unsigned)sizeof(PyroConfigData));
    }
    prefs.end();

    // Always initialise piezo hardware so it's ready if enabled at runtime
    gpio_set_direction((gpio_num_t)(config::PIEZO_PIN), GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)(config::PIEZO_PIN), 0);
    piezo_pwm_ready = initPiezoTimer();
    if (!piezo_pwm_ready)
    {
        ESP_LOGE(TAG, "Piezo timer init failed; sounds disabled");
    }

    // Initialize sensor collector (including sensors) and start polling tasks
    ESP_LOGI(TAG, "Sensor collector init...");
    sensor_collector.begin(config::SENSOR_CORE);

    // #557: latch GNSS-absent mode.  isGnssOnline() is false when GNSS is built
    // in but the module failed bring-up (dead/deaf UART).  The FC then flies a
    // baro+IMU-only degraded path instead of hanging on the pad.  begin() blocks
    // until the ~35 s bring-up deadline in that case, so millis() is already
    // well past zero here.
    gnss_absent_mode     = config::USE_GNSS && !sensor_collector_hw.isGnssOnline();
    gnss_absent_dwell_ms = millis() + GNSS_ABSENT_INIT_DWELL_MS;
    if (gnss_absent_mode)
    {
        ESP_LOGW(TAG, "[GNSS] Module absent (bring-up failed) — GNSS-absent "
                      "degraded mode: baro+IMU EKF init, guidance forced off");
    }
    sensor_converter.configureISM6HG256FullScale(
        static_cast<ISM6LowGFullScale>(config::ISM6_LOW_G_FS_G),
        static_cast<ISM6HighGFullScale>(config::ISM6_HIGH_G_FS_G),
        static_cast<ISM6GyroFullScale>(config::ISM6_GYRO_FS_DPS));
    sensor_converter.configureISM6HG256RotationZ(config::ISM6HG256_ROT_Z_DEG);
    sensor_converter.configureMMC5983MARotationZ(config::MMC5983MA_ROT_Z_DEG);
    sensor_converter.configureIIS2MDCRotationZ(config::IIS2MDC_ROT_Z_DEG);
    sensor_collector.configureSimRotation(config::ISM6HG256_ROT_Z_DEG);
    sensor_collector.configureSimIis2mdcRotation(config::IIS2MDC_ROT_Z_DEG);

    // Board→rocket mounting orientation (converter + sim + OC query payload).
    applyBoardToRocketOrientation(
        config::BOARD_TO_ROCKET_ORIENT,
        (config::BOARD_TO_ROCKET_ORIENT == ORIENT_CODE_IDENTITY)
            ? ORIENT_MODE_DEFAULT : ORIENT_MODE_MANUAL);

    // Apply mag hard-iron offset to the IIS2MDC chip now that begin() has
    // finished its softReset (which zeroes OFFSET_X/Y/Z).  Issue #96.
    if (pending_mag_apply && sensor_collector.isIIS2MDCActive())
    {
        const bool ok = sensor_collector.setIIS2MDCHardIronOffset(
                          pending_mag_cx, pending_mag_cy, pending_mag_cz);
        ESP_LOGI(TAG, "IIS2MDC OFFSET applied: (%d,%d,%d) %s",
                 (int)pending_mag_cx, (int)pending_mag_cy, (int)pending_mag_cz,
                 ok ? "OK" : "FAILED");
    }

    // Apply restored gyro zero-rate bias now that begin() is done (#132).
    if (pending_gyro_cal_apply)
    {
        sensor_collector_hw.gyro_cal_x = pending_gyro_cal_x;
        sensor_collector_hw.gyro_cal_y = pending_gyro_cal_y;
        sensor_collector_hw.gyro_cal_z = pending_gyro_cal_z;
        ESP_LOGI(TAG, "Gyro cal applied: (%d,%d,%d)",
                 (int)pending_gyro_cal_x, (int)pending_gyro_cal_y, (int)pending_gyro_cal_z);
    }

    out_status_query_data.ism6_low_g_fs_g = config::ISM6_LOW_G_FS_G;
    out_status_query_data.ism6_high_g_fs_g = config::ISM6_HIGH_G_FS_G;
    out_status_query_data.ism6_gyro_fs_dps = config::ISM6_GYRO_FS_DPS;
    out_status_query_data.ism6_rot_z_cdeg = (int16_t)lroundf(config::ISM6HG256_ROT_Z_DEG * 100.0f);
    out_status_query_data.mmc_rot_z_cdeg = (int16_t)lroundf(config::MMC5983MA_ROT_Z_DEG * 100.0f);
    out_status_query_data.iis2mdc_rot_z_cdeg = (int16_t)lroundf(config::IIS2MDC_ROT_Z_DEG * 100.0f);
    // v3: adds board→rocket orientation (b2r_* fields, filled by
    // applyBoardToRocketOrientation above and on any runtime change).
    // v4: adds per-chip iis2mdc_rot_z_cdeg (#204).
    // v5: adds the guidance-target echo (#435).
    // v6: adds mag_type — which chip is behind the 0xD1 mag stream, so log
    // analysis picks the right count scale per board.
    out_status_query_data.format_version = 6;
#ifdef TR_MAG_DRIVER_QMC5883P
    out_status_query_data.mag_type = MAG_TYPE_QMC5883P;
#else
    out_status_query_data.mag_type = MAG_TYPE_IIS2MDC;
#endif
    // Guidance-target echo boot state (#435): no cmd 28 processed yet; the
    // current target is whatever cmd 65 left in NVS (restored above).
    out_status_query_data.tgt_seq     = 0;
    out_status_query_data.tgt_last_rc = GUID_RC_NONE;
    out_status_query_data.tgt_status  = guidanceFlownTargetSrc(false, pn_target_mode);
    out_status_query_data.tgt_lat_deg = 0.0f;
    out_status_query_data.tgt_lon_deg = 0.0f;
    out_status_query_data.tgt_alt_m   = 0;

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
    delay_ms(10); // let OutComputer process query and queue 96-byte response
    {
        uint8_t init_buf[COMBINED_READ_SIZE] = {};
        // 50 ms timeout (was 10): an ABORTED read leaves residue in the OC's
        // V2 slave TX ring and permanently desyncs the channel (#399's
        // residual hazard — seen on the bench at the LANDED transition, where
        // the OC's finalize churn delayed the serve past 10 ms: one
        // "i2c.master: I2C software timeout" then 100% read failures until
        // reboot).  Waiting longer and completing the transfer is strictly
        // safer than aborting; normal serves finish in ~2.5 ms.
        esp_err_t read_err = i2c_interface.masterRead(
            init_buf, COMBINED_READ_SIZE, 50);
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
            gpio_set_direction((gpio_num_t)(config::CAM_PWR_PIN), GPIO_MODE_OUTPUT);
            gpio_set_level((gpio_num_t)(config::CAM_PWR_PIN), 1);
        }
        if (config::CAM_SHUTTER_PIN >= 0)
        {
            gpio_set_direction((gpio_num_t)(config::CAM_SHUTTER_PIN), GPIO_MODE_OUTPUT);
            gpio_set_level((gpio_num_t)(config::CAM_SHUTTER_PIN), 1);
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
        // #267: apply the physical fin-angle <-> pulse calibration so commanded
        // fin degrees map to real deflection (not the command-clamp span).
        // From NVS if the app has set it, else the config default.
        servo_control.setFinCalibration(nvs_fin_min_deg, nvs_fin_max_deg);
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
        if (servo_enabled)
        {
            // #345 follow-up: settle at the *commanded* neutral (0deg -> the
            // physical fin-neutral incl. trim, via the fin calibration), NOT the
            // raw pulse-midpoint the wiggle returns to.  Those two only coincide
            // for a symmetric fin cal; with a per-model/asymmetric cal the wiggle
            // leaves the tabs off the trimmed straight position.  Drive through
            // the same setServoAngles() path the flight/servo-test neutral uses
            // so the boot rest position matches exactly what was trimmed.
            // #407: settle via the anti-backlash approach (overshoot then back)
            // so the boot rest position is repeatable on a slightly worn servo;
            // the per-tick serviceNeutralSettle() below completes it.
            servo_control.beginNeutralSettle(time_ms());
            // Hold that trimmed neutral briefly before the pad relax kicks in, so
            // the operator can see/verify the tabs sit straight on boot instead
            // of going limp the instant READY starts.
            if (config::SERVO_RELAX_ON_PAD)
                servo_pad_wake_until_ms = time_ms() + config::SERVO_PAD_TRIM_WAKE_MS;
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
                delay_ms(20);  // let OC service the request and queue the response

                constexpr size_t kRespFrameLen = 4 + 1 + 1 + sizeof(FlightSnapshotData) + 2;
                // Read EXACTLY the staged frame size. The OC stages
                // kRespFrameLen bytes; under the V2 slave driver, clocking out
                // more than was staged underflows the TX path (the +16 "SOF
                // byte loss" slack was a V1-only defense and V2 keeps the SOF
                // at offset 0). Over-reading here is the inverse of the #88
                // status-path residue bug. Buffer keeps slack for the scan.
                uint8_t buf[kRespFrameLen + 16] = {};
                esp_err_t rerr = i2c_interface.masterRead(buf, kRespFrameLen, 100);
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
                                    if (snap.sim_flight) {
                                        // The dry-fire gate lives in isSimActive(),
                                        // which a reboot resets — restoring here
                                        // would fire real pyro outputs on bench
                                        // igniters mid-sim.
                                        ESP_LOGW(TAG, "[RECOVERY] Snapshot is from a SIMULATED flight — refusing restore");
                                    } else {
                                        valid = true;
                                    }
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

            const uint32_t now_ms = time_ms();

            // Restore flight state
            rocket_state = INFLIGHT;

            // Rebase timestamps to new time_ms() epoch
            launch_time_millis = now_ms - snap.flight_elapsed_ms;
            if (snap.pyro_apogee_detected) {
                pyro_apogee_detected = true;
                pyro_apogee_time_ms = launch_time_millis + snap.apogee_elapsed_ms;
            }
            if (snap.burnout_detected) {
                burnout_detected = true;
                burnout_time_ms = launch_time_millis + snap.burnout_elapsed_ms;
            }

            // Restore pyro state (safety-critical: no double-fire, no missed fire).
            // With per-fire arming, ARM is never persistent — any channel
            // whose trigger condition is still met will re-arm and fire via
            // servicePyroChannels(). We only need to lock in fired channels
            // so they aren't re-fired.
            portENTER_CRITICAL(&pyro_spinlock);
            const uint8_t fired_snap[4] = { snap.pyro1_fired, snap.pyro2_fired,
                                            snap.pyro3_fired, snap.pyro4_fired };
            for (int i = 0; i < 4; ++i) {
                pyro_ch[i].state          = fired_snap[i] ? PyroChState::Done : PyroChState::Idle;
                pyro_ch[i].phase_start_ms = 0;
            }
            portEXIT_CRITICAL(&pyro_spinlock);

            ESP_LOGW(TAG, "[RECOVERY] Pyro: apogee=%d fired=[%d,%d,%d,%d]",
                     pyro_apogee_detected,
                     snap.pyro1_fired, snap.pyro2_fired,
                     snap.pyro3_fired, snap.pyro4_fired);

            // Restore flight references
            ground_pressure_pa = snap.ground_pressure_pa;
            ref_lat_rad = snap.ref_lat_rad;
            ref_lon_rad = snap.ref_lon_rad;
            ref_alt_m   = snap.ref_alt_m;
            have_ref_pos = true;
            ref_pos_frozen = true;
            ground_pressure_found = true;

            // Restore the board→rocket orientation BEFORE the EKF state —
            // the snapshot's quaternion/velocities/biases were estimated in
            // this rocket frame, and every conversion from here on must
            // match it (the boot default from config may differ when the
            // pad auto-detect re-oriented before launch).
            {
                float q[4];
                for (int i = 0; i < 4; ++i) {
                    q[i] = (float)snap.b2r_q[i] / ORIENT_QUAT_WIRE_SCALE;
                }
                float R[9];
                orientQuatToMatrix(q, R);
                applyBoardToRocketRotation(R, snap.b2r_code, snap.b2r_mode, 0);
                ESP_LOGW(TAG, "[RECOVERY] Board→rocket orientation restored: %s (mode %u)",
                         orientCodeName(snap.b2r_code), (unsigned)snap.b2r_mode);
            }

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
    triggerBlueLedFlash(time_ms());

}

// ==========================================================================
// SECTION: Simulator re-arm
// ==========================================================================
// #393: reset the flight state machine for a sim run.  Used on the sim START
// edge (fresh run — the sim's equivalent of a reboot) and on an explicit sim
// STOP (SIM_STOP_CMD → abort back to READY).  Deliberately NOT called on the
// sim's natural completion (SIM_LANDED → SIM_IDLE auto-stop), so a flown-out sim
// still holds LANDED to validate the post-flight lockout.  `edge` labels the log.
static void resetFlightStateForSim(const char* edge)
{
    // Also resets GNSS state: the sim injects synthetic GNSS (fix=3, sats=12);
    // a stale copy would otherwise immediately re-trip READY -> PRELAUNCH.
    pyroSafeAll();
    rocket_state = READY;
    post_flight_lockout = false;  // #317: a deliberate sim start/stop re-arms
    ground_pressure_found = false;
    out_ready = false;
    end_flight_sent = false;
    landed_actions_done = false;
    landed_candidate_active = false;   // #297
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
    // #557: a sim injects synthetic GNSS, so re-evaluate the degraded path from
    // scratch — clear the degraded-flight latch and restart the settle so the
    // sim's ~100 ms first fix reaches the normal init path before any fallback.
    gnss_absent_flight   = false;
    gnss_absent_dwell_ms = millis() + GNSS_ABSENT_INIT_DWELL_MS;
    guidance.reset();
    control_mixer.reset();
    roll_rate_pid_standalone.reset();
    burnout_detected = false;
    burnout_time_ms = 0;
    burnout_neg_count = 0;
    // Deployment is a per-flight latch like burnout. Clearing it also puts a
    // dynamic logging rate back to the boost rate for the new flight — without
    // this, a second flight in one boot would log its whole boost at the
    // post-deployment rate.
    tr::deploymentReset(deployment_state);
    deployment_detected = false;
    deployment_time_ms  = 0;
    applyImuRateForFlightPhase();
    guidance_active = false;
    reboot_recovery = false;
    reboot_recovery_telem = false;
    clearFlightSnapshot();
    // #435: a sim start/stop is the sim's equivalent of a reboot, and a real
    // reboot drops the (RAM-only) Drift-Cast point.  The ref just discarded
    // above is also the frame the point's receipt-time conversion used — the
    // sim's synthetic GNSS rebuilds a DIFFERENT reference, so keeping the
    // point would either fly a bogus E/N or hit the enterInflight fail-safe
    // while the echo kept asserting GEO_ACTIVE.  Drop it, revert the target
    // to the profile-owned point, and refresh the echo so the app SEES the
    // drop (supersede bumps tgt_seq -> the OC's change-detect republishes).
    if (guid_point_valid) {
        guid_point_valid = false;
        pn_target_e = pn_profile_target_e;
        pn_target_n = pn_profile_target_n;
        applyGuidanceConfig();   // re-runs the radius gate + setHorizontalTarget
        ESP_LOGW(TAG, "[GUID PT] sim %s dropped the Drift-Cast point — "
                      "target reverted to profile (%.1f,%.1f)",
                 edge, (double)pn_target_e, (double)pn_target_n);
    }
    (void)guidanceTargetEchoSupersede(
        out_status_query_data, guidanceFlownTargetSrc(false, pn_target_mode));
    ESP_LOGI(TAG, "[STATE] Sim %s -> READY", edge);
}

// ==========================================================================
// SECTION: INFLIGHT entry
// ==========================================================================
// Loop is responsible for reading sensor data
// INFLIGHT entry — everything that must happen exactly once at launch
// (state flip, per-flight resets, EKF/guidance degraded-mode handling).
// Factored out of the PRELAUNCH case so the #382 READY-launch fallback
// promotes through the IDENTICAL path and the two can never drift.
static void enterInflight(uint32_t now_ms, const char* from_state)
{
    rocket_state = INFLIGHT;
    launch_time_millis = now_ms;
    // Orientation is now latched (estimator only runs in
    // READY/PRELAUNCH).  Start the boost-phase thrust-axis
    // cross-check on what was latched.
    orient_thrust_mismatch = false;
    orient_thrust_check.start(time_us());
    // Safety: warn and disable guidance if EKF never initialized
    if (!ekf_initialized) {
        ESP_LOGW(TAG, "[EKF] WARNING: Launching without EKF initialization!");
        if (guidance_enabled) {
            ESP_LOGW(TAG, "[GUIDANCE] Auto-disabled — EKF not initialized");
            guidance_enabled = false;
        }
    }
    // #557: in a GNSS-absent degraded flight the EKF *is* initialized (baro+IMU),
    // so the gate above does not fire — but there is no absolute position frame,
    // so PN/station-keep guidance must not steer.  Force it off here (roll
    // control is untouched; coast_guidance_active also self-gates on have_ref_pos).
    if (gnss_absent_flight && guidance_enabled) {
        ESP_LOGW(TAG, "[GUIDANCE] Auto-disabled — GNSS-absent degraded flight");
        guidance_enabled = false;
    }
    // Freeze the ENU reference position at launch
    if (!ref_pos_frozen && have_ref_pos) {
        ref_pos_frozen = true;
        ESP_LOGI(TAG, "[EKF] Ref pos frozen (launch): n=%lu",
                      (unsigned long)ref_pos_count);
    }
    // #435: re-convert the Drift-Cast geodetic aim point against the just-
    // frozen reference — the receipt-time conversion used the then-current
    // running pad average, which keeps drifting (normally centimetres) until
    // the freeze.  Runs BEFORE the first sendFlightSettings emit below
    // (launch+0/100/200 ms) so the logged FlightSettings carries the FLOWN
    // E/N.  If have_ref_pos is false here (READY degraded launch), the
    // receipt-time conversion could never have happened (NOREF gate) and
    // resetFlightStateForSim() clears the point whenever it discards the
    // reference — so guid_point_valid is false and the guard is consistent.
    if (guid_point_valid && have_ref_pos) {
        // Keep-vs-wipe policy is the pure host-tested guidanceLaunchReconvert()
        // (GuidancePointGate.h) — same conversion + REJECT-never-clamp radius
        // rule as the receipt gate.
        const GuidanceLaunchTarget lt = guidanceLaunchReconvert(
            guid_point_lat_deg, guid_point_lon_deg,
            ref_lat_rad, ref_lon_rad, ref_alt_m,
            config::GUIDANCE_MAX_AIM_RADIUS_M);
        if (lt.point_kept) {
            ESP_LOGI(TAG, "[GUID PT] launch re-convert: E=%.2f N=%.2f (was %.2f,%.2f)",
                     (double)lt.e_m, (double)lt.n_m,
                     (double)pn_target_e, (double)pn_target_n);
            pn_target_e = lt.e_m;
            pn_target_n = lt.n_m;
        } else {
            // Should be unreachable (accepted at receipt, pad average moves
            // centimetres) — but a boundary point + drift could cross the
            // gate.  Fail SAFE to overhead rather than fly an out-of-policy
            // point; drop guid_point_valid so the settings snapshot records
            // what actually flies, and refresh the echo so the app's Unit
            // target row stops asserting a point that no longer flies (the
            // OC only sees it after landing — the FC skips polls INFLIGHT —
            // but the post-flight state must not lie).
            ESP_LOGW(TAG, "[GUID PT] launch re-convert exceeds %.0f m — "
                          "forcing (0,0) overhead",
                     (double)config::GUIDANCE_MAX_AIM_RADIUS_M);
            pn_target_e = 0.0f;
            pn_target_n = 0.0f;
            guid_point_valid = false;
            (void)guidanceTargetEchoSupersede(out_status_query_data,
                                              GUID_TGT_NONE);
        }
        guidance.setHorizontalTarget(pn_target_e, pn_target_n);
    }
    // #297: do NOT re-capture ground pressure here.  launch_flag
    // lags real liftoff, so bmp_latest_si is already a few m up and
    // would bias AGL (and the ALTITUDE_ON_DESCENT main-deploy height)
    // low for the whole flight.  Keep the reference frozen from
    // READY/PRELAUNCH — the baro path stops updating ground_pressure_pa
    // once state leaves INIT/READY/MAG_CAL, so it's already the last
    // on-pad value here.
    max_alt_m = 0.0f;
    max_speed_mps = 0.0f;
    landed_actions_done = false;
    landed_candidate_active = false;   // #297
    // Clear apogee/landing flags so they start clean at launch.
    // launch_flag is intentionally preserved (it got us here).
    // Per #142/#143: the original reset cleared only baro and
    // velocity flags, leaving stale gps_apogee, pitch_apogee,
    // and master apogee_flag values that could survive a
    // reboot-recovery into the next ascent.
    kinematics.alt_apogee_flag = false;
    kinematics.vel_u_apogee_flag = false;
    kinematics.gps_apogee_flag = false;
    kinematics.pitch_apogee_flag = false;
    kinematics.apogee_flag = false;
    kinematics.alt_landed_flag = false;
    kinematics.max_speed = 0.0f;
    // Reset guidance state for new flight
    guidance.reset();
    control_mixer.reset();
    roll_rate_pid_standalone.reset();
    burnout_detected = false;
    burnout_time_ms = 0;
    burnout_neg_count = 0;
    // Deployment is a per-flight latch like burnout. Clearing it also puts a
    // dynamic logging rate back to the boost rate for the new flight — without
    // this, a second flight in one boot would log its whole boost at the
    // post-deployment rate.
    tr::deploymentReset(deployment_state);
    deployment_detected = false;
    deployment_time_ms  = 0;
    applyImuRateForFlightPhase();
    guidance_active = false;
    guidance_tilt_inhibited = false;   // clear tilt-limit latch
    // Reset pyro channels on launch. ARM stays LOW until a
    // channel's trigger fires (per-fire arming on new PCB).
    pyro_apogee_detected = false;
    pyro_apogee_time_ms  = 0;
    // #382: re-arm the per-flight settings snapshot (#165) —
    // without this only the first flight/sim per boot emitted
    // FlightSettingsData, so sim runs #2+ in one boot had no
    // settings block in their reports.
    settings_emit_count = 0;
    portENTER_CRITICAL(&pyro_spinlock);
    for (int i = 0; i < 4; ++i) {
        pyro_ch[i].state          = PyroChState::Idle;
        pyro_ch[i].phase_start_ms = 0;
    }
    portEXIT_CRITICAL(&pyro_spinlock);
    ESP_LOGI(TAG, "[STATE] %s -> INFLIGHT (ground_p=%.0f)",
                  from_state, (double)ground_pressure_pa);
}

// ==========================================================================
// SECTION: Main loop
// ==========================================================================
static void loop_fc()
{
    // Zero the IMU-queue drop gauge on the first pass: the poll task starts
    // ~4 s before this loop does (servo init holds setup), producing ~8k
    // meaningless pre-consumer drops into an unconsumed queue.  From here on
    // a nonzero [GAP DIAG] imu_q_drops means the consumer actually stalled.
    {
        static bool imu_drop_gauge_zeroed = false;
        if (!imu_drop_gauge_zeroed)
        {
            imu_drop_gauge_zeroed = true;
            sensor_collector_hw.resetIsm6QueueDrops();
        }
    }

    fcMaybeMarkOtaValid();   // Layer 4: confirm a freshly OTA'd image after a stable run

    // ── OTA image pump: yield the core to the OTA RX parser ──
    // During an FC OTA the board is grounded and its I2S is flipped to slave RX.
    // This flight task runs at the top priority (configMAX_PRIORITIES-1) on the
    // SAME core as fcOtaParserTask (prio 3), so without backing off it starves
    // the parser — the 8 KB RX ring then overflows the moment flash writes + the
    // EKF land, and the forward-only image stalls (bench 2026-06-01 stuck at 714,
    // sawtooth gaps = drop-oldest eviction). Nothing on the FC needs to run during
    // the transfer except the I2C control poll (to catch OTA_FINISH/ABORT); the
    // GUI is fed by the OC, not us. So sleep most of each iteration: the parser
    // (now the top READY task on this core) drains the ring, and we still fall
    // through to service I2C. fcRevertToTx() clears the flag and full-rate flight
    // resumes. The EKF is additionally skipped below for the brief awake windows.
    if (fc_ota_data_mode)
    {
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // ==========================================================================
    // SECTION: Sensor drain and conversion
    // ==========================================================================
    // ### Read and Send Sensor Data ###
    // Poll as fast as possible so high-rate sensor frames are not dropped by
    // loop-period gating.
    static uint32_t dbg_ism6_reads = 0, dbg_bmp_reads = 0, dbg_bmp_bad_reads = 0, dbg_mmc_reads = 0, dbg_iis2mdc_reads = 0, dbg_gnss_reads = 0;

    // Drain ALL pending IMU samples each iteration.  The chip runs at
    // ISM6HG256_UPDATE_RATE (1920 Hz) — about two samples per ~1 ms loop pass —
    // and every one is logged to I2S so the recorded IMU rate follows the chip
    // ODR, not the loop rate (single-slot handoff used to cap it at ~980/s).
    // The control/EKF path converts only the FRESHEST drained sample: guidance,
    // kinematics and the EKF all keep running at loop rate exactly as before.
    {
        bool ism6_new_this_iter = false;
        while (sensor_collector.getISM6HG256Data(ism6hg256_data))
        {
            dbg_ism6_reads++;
            ism6_new_this_iter = true;

            memcpy(ism6hg256_data_buffer,
                   &ism6hg256_data,
                   SIZE_OF_ISM6HG256_DATA);

            (void)enqueueI2STx(ISM6HG256_MSG,
                               ism6hg256_data_buffer,
                               SIZE_OF_ISM6HG256_DATA);
        }

        if (ism6_new_this_iter)
        {
            sensor_converter.convertISM6HG256Data(ism6hg256_data, ism6_latest_si);
            have_ism6_si = true;

            // Feed the live low-g accel to the mag calibrator so each
            // incoming mag sample can be bucketed by physical orientation
            // (issue #96 follow-up).  Raw int16 LSB units share the same
            // sign convention as the mag direction-wedge encoding.  The
            // freshest sample is sufficient — the mag runs at 100 Hz.
            mag_calibrator.setLiveAccel(ism6hg256_data.acc_low_raw.x,
                                        ism6hg256_data.acc_low_raw.y,
                                        ism6hg256_data.acc_low_raw.z);
        }
    }

    if (sensor_collector.getBMP585Data(bmp585_data))
    {
        dbg_bmp_reads++;
        // #260: validate pressure at the source before it drives the altitude /
        // apogee math.  Convert into a candidate and commit only if it's in band
        // — the bounded check also rejects NaN/+-Inf — so a sensor glitch or
        // SPI-corrupted sample can't poison ground_pressure_pa / pressure_altitude_m
        // and the kinematics KF (a single NaN/Inf altitude stays NaN forever, since
        // NaN fails every gate).  On reject we keep the last good sample
        // (have_bmp_si stays latched; the 0.5 s downstream staleness gate catches a
        // persistent outage and marks baro unhealthy).
        BMP585DataSI bmp_candidate = {};
        sensor_converter.convertBMP585Data(bmp585_data, bmp_candidate);
        if (bmp_candidate.pressure >= BMP_PRESSURE_MIN_PA &&
            bmp_candidate.pressure <= BMP_PRESSURE_MAX_PA)
        {
            bmp_latest_si = bmp_candidate;
            have_bmp_si = true;
            bmp_new_for_kf = true;
        }
        else
        {
            dbg_bmp_bad_reads++;
        }

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

        // Issue #96 — feed every fresh raw sample into the calibrator while
        // we're in MAG_CALIBRATION.  Outside that state the calibrator is
        // IDLE and addSample() short-circuits.  When the buffer fills it
        // returns true so we publish the REVIEW frame promptly without
        // waiting for the 5 Hz cadence.
        if (rocket_state == MAG_CALIBRATION)
        {
            if (mag_calibrator.addSample(iis2mdc_data.mag_x,
                                         iis2mdc_data.mag_y,
                                         iis2mdc_data.mag_z))
            {
                mag_cal_status_dirty = true;
            }
        }
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

    // ==========================================================================
    // SECTION: Magnetometer calibration status
    // ==========================================================================
    // Issue #96 — periodic mag-cal status frame.  5 Hz cadence in
    // SAMPLING; immediate publish on REVIEW/APPLIED/ABORTED transitions
    // (mag_cal_status_dirty).  Outside MAG_CALIBRATION the calibrator is
    // IDLE; we still publish ABORTED/APPLIED transitions but otherwise
    // stay quiet so the BLE channel isn't carrying useless idle frames.
    {
        constexpr int64_t MAG_CAL_STATUS_PERIOD_US = 200000;  // 5 Hz
        const int64_t now = esp_timer_get_time();
        const bool in_state = (rocket_state == MAG_CALIBRATION);
        const bool elapsed  = (now - mag_cal_last_status_us) >= MAG_CAL_STATUS_PERIOD_US;
        if (mag_cal_status_dirty || (in_state && elapsed))
        {
            MagCalStatusData s{};
            mag_calibrator.buildStatusFrame((uint32_t)now, s);
            uint8_t buf[sizeof(MagCalStatusData)];
            memcpy(buf, &s, sizeof(buf));
            (void)enqueueI2STx(MAG_CAL_STATUS_MSG, buf, sizeof(buf));
            mag_cal_last_status_us = now;
            mag_cal_status_dirty = false;
        }
    }

    // #206 — post-accept verify window.  Polls the calibrator's
    // verify accumulators once the user-tap-Accept dwell has elapsed
    // (MAG_CAL_VERIFY_DURATION_US).  On pass we persist NVS as the
    // old single-step accept used to; on fail we restore the prior
    // OFFSET regs and let the calibrator's own state machine flip
    // back to REVIEW with reject_code = MAG_CAL_REJECT_VERIFY_FAILED
    // so iOS surfaces the right error.
    if (mag_cal_verify_active)
    {
        const int64_t now = esp_timer_get_time();
        if (mag_cal_verify_eval_now ||
            (now - mag_cal_verify_start_us) >= MAG_CAL_VERIFY_DURATION_US)
        {
            mag_cal_verify_eval_now = false;
            float worst_uT = 0.0f;
            const bool pass = mag_calibrator.evaluateVerify(worst_uT);
            mag_cal_verify_active = false;

            if (pass)
            {
                // Calibrator is now APPLIED; pull final fit + persist.
                int16_t cx, cy, cz;
                float R_uT, res_uT;
                uint8_t reject;
                mag_calibrator.getResult(cx, cy, cz, R_uT, res_uT, reject);

                prefs.begin("mag_cal", false);
                prefs.putUChar("ver",    MAG_CAL_NVS_SCHEMA_VERSION);
                prefs.putBool ("done",   true);
                prefs.putShort("cx",     cx);
                prefs.putShort("cy",     cy);
                prefs.putShort("cz",     cz);
                prefs.putFloat("R_uT",   R_uT);
                prefs.putFloat("res_uT", res_uT);
                prefs.putInt  ("mmc_cx", 0);
                prefs.putInt  ("mmc_cy", 0);
                prefs.putInt  ("mmc_cz", 0);
                prefs.end();

                ESP_LOGI(TAG, "[MAGCAL] verify PASS — saved cx=%d cy=%d cz=%d R=%.2fµT res=%.2fµT",
                         (int)cx, (int)cy, (int)cz, (double)R_uT, (double)res_uT);

                mag_cal_session_active = false;
                mag_cal_status_dirty = true;
                rocket_state = READY;
                // Don't clear() here — leave the calibrator in APPLIED so
                // the next status frame published reports sub_type=APPLIED
                // and iOS shows the success banner.  Otherwise clear()
                // would flip state to IDLE before the publish loop runs,
                // and iOS would land on the intro screen with no "Saved"
                // feedback.  calibrator.start() on the next session
                // resets all state cleanly.
            }
            else
            {
                // Calibrator already transitioned back to REVIEW with
                // MAG_CAL_REJECT_VERIFY_FAILED set.  Zero the chip (NOT
                // restore prior) so the user can immediately Retry into
                // SAMPLING and see raw mag values.  If they Abort
                // instead, the abort handler does the prior restore.
                if (sensor_collector.isIIS2MDCActive())
                {
                    const bool ok = sensor_collector.setIIS2MDCHardIronOffset(0, 0, 0);
                    ESP_LOGW(TAG, "[MAGCAL] verify FAIL (worst=%.1fµT) — zeroed OFFSET %s; "
                                  "user can Retry (raw sampling) or Abort (restore prior)",
                             (double)worst_uT, ok ? "OK" : "FAIL");
                }
                // Stamp the worst observed |B| into the calibrator's
                // inst_field slot via buildStatusFrame — already true
                // because the calibrator captured last_x/y/z from the
                // verify samples.  iOS displays inst_field_uT_x10
                // alongside reject_code = VERIFY_FAILED.
                mag_cal_status_dirty = true;
                // Stay in MAG_CALIBRATION so iOS keeps the cal screen
                // up; the user can tap Retry or Abort from REVIEW.
            }
        }
    }

    // ==========================================================================
    // SECTION: Flight logic: pressure altitude and orientation
    // ==========================================================================
    // --- Flight logic update ---
    const uint32_t logic_now_us = time_us();
    if ((logic_now_us - last_flight_loop_update_time) >= flight_loop_period)
    {
        lt_loop_count++;
        last_flight_loop_update_time = logic_now_us;

        // ### Pressure altitude calculation (for kinematic checks) ###
        const uint32_t now_ms = time_ms();
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
                 rocket_state == READY ||
                 rocket_state == MAG_CALIBRATION))
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

            // ── Board→rocket orientation: pad auto-detect + thrust check ──
            // On the pad, the averaged specific-force direction reveals the
            // mounting (continuously re-estimated, so late re-racking after
            // horizontal prep self-corrects; latched by leaving READY/
            // PRELAUNCH at launch).  During boost, the thrust direction
            // cross-checks whatever was latched.
            // Tilting the board during a bench ground test must NOT re-trigger the
            // pad orientation auto-detect — it would keep re-racking the board→rocket
            // frame mid-test (the fins then chase a moving reference).  Freeze the
            // estimator while ground_test_active; it resumes when the test stops.
            if ((rocket_state == READY || rocket_state == PRELAUNCH) && !ground_test_active)
            {
                if (orient_estimator.update(ism6_latest_si.time_us,
                                            acc_x, acc_y, acc_z,
                                            (float)ism6_latest_si.gyro_x,
                                            (float)ism6_latest_si.gyro_y,
                                            (float)ism6_latest_si.gyro_z))
                {
                    float up_rocket[3];
                    if (orient_estimator.takeEstimate(up_rocket))
                    {
                        handleOrientationEstimate(up_rocket);
                    }
                }
            }
            else if (rocket_state == INFLIGHT && !orient_thrust_check.done())
            {
                if (orient_thrust_check.update(ism6_latest_si.time_us,
                                               acc_x, acc_y, acc_z))
                {
                    if (orient_thrust_check.valid() &&
                        orient_thrust_check.angleFromNoseDeg() > 25.0f)
                    {
                        orient_thrust_mismatch = true;
                        ESP_LOGE(TAG, "[ORIENT] thrust axis %.1f° off nose — "
                                      "latched orientation %s (mode %u) suspect",
                                 (double)orient_thrust_check.angleFromNoseDeg(),
                                 orientCodeName(b2r_active_code),
                                 (unsigned)b2r_active_mode);
                    }
                    else if (orient_thrust_check.valid())
                    {
                        ESP_LOGI(TAG, "[ORIENT] thrust-axis check OK (%.1f° off nose)",
                                 (double)orient_thrust_check.angleFromNoseDeg());
                    }
                }
            }

            // ==========================================================================
            // SECTION: EKF input, initialization, and update
            // ==========================================================================
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

            // #297: collect a short recent window of EKF-frame gyro so the EKF
            // bias seed (wBias = mean gyro, valid because the pad is stationary)
            // averages a few samples instead of one noisy sample — shrinks the
            // pad gyro-bias startup transient.  Small window so it reflects the
            // stationary-at-init state, not earlier pad handling.
            static constexpr int kInitGyroWin = 16;
            static double initGyroRing[3][kInitGyroWin] = {};
            static int initGyroHead = 0;
            static int initGyroN = 0;
            if (!ekf_initialized)
            {
                initGyroRing[0][initGyroHead] = ekf_imu.gyro_x;
                initGyroRing[1][initGyroHead] = ekf_imu.gyro_y;
                initGyroRing[2][initGyroHead] = ekf_imu.gyro_z;
                initGyroHead = (initGyroHead + 1) % kInitGyroWin;
                if (initGyroN < kInitGyroWin) initGyroN++;
            }

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
            //
            // EKF decimation gate, computed up-front: the GNSS consume/dedup
            // below must know whether the EKF will actually process the fix this
            // tick.  Marking a fix consumed on a decimation-"off" tick loses it
            // (gnss_gate1 goes false before any EKF tick sees it) and makes the
            // else-branch inject a zeroed measurement with a never-processed
            // timestamp — corrupting position/velocity/heading aiding (#367).
            static uint8_t ekf_decim_ctr = 0;
            const bool run_ekf_this_tick =
                (++ekf_decim_ctr >= config::EKF_DECIMATION) && !fc_ota_data_mode;
            if (run_ekf_this_tick) ekf_decim_ctr = 0;

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
                // #367: only mark the fix consumed when the EKF actually runs
                // this tick.  Advancing these markers on a decimation-"off" tick
                // would drop the fix (gnss_gate1 false on the next EKF tick) and
                // inject a zeroed measurement carrying this (never-processed)
                // timestamp.  Left un-advanced, the same fix survives to the next
                // EKF tick and is used for real; and the else-branch below only
                // ever replays an already-processed timestamp, which the EKF
                // correctly skips.
                if (run_ekf_this_tick)
                {
                    last_gnss_time_us_for_ekf = gnss_latest_si.time_us;
                    last_gnss_fix_second     = gnss_latest_si.second;
                    last_gnss_fix_ms         = gnss_latest_si.milli_second;
                }

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
            // (run_ekf_this_tick is computed above, before the GNSS gate — #367.)
            if (!ekf_initialized && rocket_state != MAG_CALIBRATION)
            {
                // Gate 3: only init with high-quality GNSS (tight h_acc + low vel)
                // Issue #216 — also gate against MAG_CALIBRATION so a tumble
                // (which violates the EKF's "stationary at init" assumption,
                // and can also set kinematics flags we'd rather not pick up)
                // can't trigger an init.  EKF init resumes naturally after
                // the cal session ends and rocket_state returns to READY.
                // #557: degraded init — when the GNSS module is absent (dead/
                // deaf UART) initialize the EKF from baro + IMU only, so attitude
                // + vertical velocity (hence the velocity/pitch apogee voters and
                // roll control) still work.  Gated on initGyroN >= 8 (well-averaged
                // stationary gyro-bias seed, same readiness the normal path needs),
                // on never having seen a fix (!gnss_started — defers to the normal
                // path if a real/sim fix ever arrives), and on a short settle so an
                // injected sim fix wins first.
                const bool normal_init = have_ref_pos && gnss_gate3_init;
                const bool degraded_init =
                    !normal_init && gnss_absent_mode && !gnss_started &&
                    (initGyroN >= 8) &&
                    ((int32_t)(millis() - gnss_absent_dwell_ms) >= 0);
                if (normal_init || degraded_init) {
                    if (degraded_init) {
                        // Pad-relative seed: lat/lon 0 (equator — regular, no
                        // geodetic singularity), alt from baro (pressure_altitude_m
                        // is height above the pad), zero velocity, and a stale GNSS
                        // time so the EKF never runs a GNSS measurement update
                        // against this fake origin.  Baro then bounds the vertical
                        // channel and accel+mag bound attitude.
                        ekf_gnss.time_us   = 0;
                        ekf_gnss.lat_rad   = 0.0;
                        ekf_gnss.lon_rad   = 0.0;
                        ekf_gnss.alt_m     = pressure_altitude_m;
                        ekf_gnss.vel_n_mps = 0.0f;
                        ekf_gnss.vel_e_mps = 0.0f;
                        ekf_gnss.vel_d_mps = 0.0f;
                    }
                    // #297: seed wBias off the recent-window gyro average rather
                    // than the single live sample (see the ring above).
                    if (initGyroN >= 8)
                    {
                        double gsum[3] = {0.0, 0.0, 0.0};
                        for (int k = 0; k < initGyroN; k++)
                        {
                            gsum[0] += initGyroRing[0][k];
                            gsum[1] += initGyroRing[1][k];
                            gsum[2] += initGyroRing[2][k];
                        }
                        ekf_imu.gyro_x = gsum[0] / initGyroN;
                        ekf_imu.gyro_y = gsum[1] / initGyroN;
                        ekf_imu.gyro_z = gsum[2] / initGyroN;
                    }
                    ekf.init(ekf_imu, ekf_gnss, ekf_mag);

                    // Pad attitude initialization: quaternion from measured
                    // gravity (any attitude — see quatFromAccelHeading) plus
                    // the known pad heading, bypassing the noisy magnetometer.
                    // The IMU data is already in ROCKET frame here (the
                    // converter applies the board→rocket mounting rotation),
                    // so this is mounting-agnostic by construction.
                    {
                        static constexpr double DEG2RAD_d = M_PI / 180.0;
                        const float heading_rad =
                            (float)(config::PAD_HEADING_DEG * DEG2RAD_d);
                        float q[4];
                        quatFromAccelHeading((float)ekf_imu.acc_x,
                                             (float)ekf_imu.acc_y,
                                             (float)ekf_imu.acc_z,
                                             heading_rad, q);
                        ekf.setQuaternion(q[0], q[1], q[2], q[3]);
                        ESP_LOGI(TAG, "[EKF] Init: acc=(%.2f,%.2f,%.2f) heading=%.1f deg",
                                      ekf_imu.acc_x, ekf_imu.acc_y, ekf_imu.acc_z,
                                      (double)config::PAD_HEADING_DEG);
                    }

                    // Magnetic declination → true-north heading.  Computed once
                    // here (off the flight-loop hot path) from the averaged pad
                    // position + GPS date via WMM2025; falls back to the config
                    // constant if the GNSS date is implausible.
                    {
                        float decl_deg = config::MAGNETIC_DECLINATION_DEG;
                        const uint16_t yr = gnss_latest_si.year;
                        if (yr >= 2020 && yr <= 2035) {
                            static const int mdays[12] =
                                {31,28,31,30,31,30,31,31,30,31,30,31};
                            int doy = gnss_latest_si.day;
                            for (int mo = 1; mo < gnss_latest_si.month && mo <= 12; mo++)
                                doy += mdays[mo - 1];
                            const bool leap = (yr % 4 == 0 && (yr % 100 != 0 || yr % 400 == 0));
                            if (leap && gnss_latest_si.month > 2) doy += 1;
                            const double decimal_year =
                                (double)yr + (double)(doy - 1) / (leap ? 366.0 : 365.0);
                            const float decl_rad = TR_GeoMag::declinationRad(
                                ref_lat_rad, ref_lon_rad, ref_alt_m, decimal_year);
                            decl_deg = decl_rad * (180.0f / (float)M_PI);
                        }
                        ekf.setDeclination(decl_deg * ((float)M_PI / 180.0f));
                        ESP_LOGI(TAG, "[EKF] Declination %.2f deg (WMM2025, %u-%02u-%02u)",
                                      (double)decl_deg, gnss_latest_si.year,
                                      gnss_latest_si.month, gnss_latest_si.day);
                    }
                    ekf_initialized = true;
                    if (degraded_init) {
                        // #557: mark the session degraded so the SH_GNSS_ABSENT
                        // telemetry flag and the launch-time guidance force-off
                        // (enterInflight) fire.  Guidance also physically can't
                        // engage without a position frame — have_ref_pos stays
                        // false, so coast_guidance_active is false regardless.
                        // Roll control is unaffected (raw-gyro rate-null path).
                        gnss_absent_flight = true;
                        ESP_LOGW(TAG, "[EKF] GNSS-absent degraded init: baro+IMU "
                                      "only, position pad-relative, guidance off "
                                      "(roll control unaffected)");
                    }
                }
            }
            else if (run_ekf_this_tick)
            {
                const uint32_t ekf_t0 = time_us();

                // AHRS accel correction: enabled on pad and after apogee (descent).
                // Disabled during thrust and coast where accel is far from 1g.
                // The 0.5g–1.5g magnitude gate inside the EKF provides additional
                // rejection, but blanket-disabling during descent starves the
                // filter of its gravity reference and freezes the velocity estimate.
                const bool post_apogee = kinematics.apogee_flag;
                const bool use_ahrs_acc = (rocket_state != INFLIGHT) || post_apogee;
                ekf.update(use_ahrs_acc, ekf_imu, ekf_gnss, ekf_mag);

                // Barometer measurement update.  Freshness is tracked by
                // BaroGatePolicy against the sample's own timestamp, NOT the
                // loop-lifetime bmp_new_for_kf flag: that flag is cleared at
                // the end of every loop iteration, so a sample read on an
                // EKF-off iteration (EKF_DECIMATION) was wiped before any EKF
                // tick saw it — the 487 Hz BMP ODR beating against the
                // 479.5 Hz EKF rate starved the EKF of baro in ~66 ms bursts
                // for ~half of every flight (#450).
                if (have_bmp_si)
                {
                    // mach_locked_out is file-scope (used by apogee voting)

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

                    static BaroGatePolicy baro_gate;
                    const BaroGatePolicy::Decision gate =
                        baro_gate.evaluate(bmp_latest_si.time_us,
                                           pressure_altitude_m,
                                           baro_locked,
                                           config::BARO_SPIKE_THRESH_M,
                                           config::BARO_SPIKE_RATE_MPS,
                                           config::BARO_FUSE_MIN_INTERVAL_US);

                    if (gate.accept)
                    {
                        EkfBaroData ekf_baro = {};
                        ekf_baro.time_us = bmp_latest_si.time_us;
                        // pressure_altitude_m is height above the launch pad
                        // (~0 at the pad), but the EKF altitude state is
                        // absolute MSL (init + fused from GNSS hMSL). Re-reference
                        // baro into the GNSS frame by adding the launch-site
                        // elevation, otherwise the baro innovation carries a
                        // constant bias ≈ -(launch elevation) that the 15-state
                        // filter leaks into altitude / accel_bias_z — the
                        // stationary altitude drift in #44. (ref_alt_m is the
                        // frozen running-average pad hMSL; ~0 until the first
                        // GNSS fix, by which point the EKF isn't yet running
                        // baro updates.)
                        ekf_baro.altitude_m = (double)pressure_altitude_m + ref_alt_m;
                        ekf.baroMeasUpdate(ekf_baro);
                    }
                    else if (gate.spike)
                    {
                        ESP_LOGW(TAG, "[BARO] Spike rejected: delta=%.1f m rate=%.0f m/s "
                                 "(thresh=%.1f m @ >%.0f m/s)",
                                 (double)gate.delta_m, (double)gate.apparent_rate_mps,
                                 (double)config::BARO_SPIKE_THRESH_M,
                                 (double)config::BARO_SPIKE_RATE_MPS);
                    }
                }

                const uint32_t ekf_us = time_us() - ekf_t0;
                lt_ekf_total_us += ekf_us;
                lt_ekf_count++;
                ekf_tick_counter++;
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

        // ==========================================================================
        // SECTION: I2C status poll to the Out Computer
        // ==========================================================================
        // Skip the status query during INFLIGHT — no app commands are
        // processed mid-flight.  I2C is now command-only (telemetry uses I2S)
        // Mutex protects the I2C bus from the sender task.
        //
        // #393 exception: keep polling during a SIM flight so SIM_STOP can be
        // delivered mid-flight (a sim is a test, never a real flight — the
        // isSimActive() guard means real INFLIGHT still skips the poll).
        // Without this the sim runs the whole trajectory before the stop is
        // even seen.
        //
        // Pipelined protocol: READ first (response from previous query),
        // then SEND the next query.  This gives the OutComputer a full
        // 250 ms poll interval to process each query and pre-load its
        // response into the slave TX FIFO, avoiding the race where the
        // OC hasn't called i2c_slave_transmit() yet.
        // #402: after sending I2C_TX_RESYNC we suspend ALL bus traffic for a
        // grace window so the OC can reset its slave device with zero risk of
        // racing an in-flight transaction (the #279 constraint).
        static uint32_t i2c_resync_grace_until_ms = 0;
        static bool     i2c_resync_active = false;
        if ((rocket_state != INFLIGHT || sensor_collector.isSimActive())
            && (now_ms - out_ready_request_time_ms) > 250U
            && (int32_t)(now_ms - i2c_resync_grace_until_ms) >= 0)
        {
            out_ready_request_time_ms = now_ms;

            xSemaphoreTake(i2c_bus_mutex, portMAX_DELAY);

            // ── Step 1: READ the response queued by the *previous* query ──
            static bool query_pending = false;  // true after first query sent
            bool query_ok = false;
            cfg_read_cache_len = 0;

            if (query_pending)
            {
                const uint32_t gor_start = time_us();
                uint8_t combined_buf[COMBINED_READ_SIZE] = {};
                // 50 ms timeout (was 10) — an aborted read permanently desyncs
                // the OC's slave TX ring (#399 residual hazard; hit on the
                // bench at LANDED when the OC's finalize churn delayed the
                // serve past 10 ms).  Completing late beats aborting; normal
                // serves take ~2.5 ms, and real-flight INFLIGHT never polls,
                // so the worst case is a rare bounded stall in ground states.
                esp_err_t read_err = i2c_interface.masterRead(
                    combined_buf, COMBINED_READ_SIZE, 50);

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
                        // Reflect the OC's reported pending command every poll,
                        // INCLUDING 0 (idle).  The OC repeats each command for
                        // CMD_REPEAT_LIMIT polls then reports 0; holding the value
                        // across the poll interval lets last_processed_cmd dedup the
                        // repeats to a single execution, and seeing 0 afterwards
                        // resets the dedup so the same command can be issued again.
                        if (resp_payload_len >= 2)
                        {
                            out_pending_command = resp_payload[1];
                        }
                        memcpy(cfg_read_cache, combined_buf + 10,
                               COMBINED_READ_SIZE - 10);
                        cfg_read_cache_len = COMBINED_READ_SIZE - 10;
                    }
                }

                const uint32_t gor_us = time_us() - gor_start;
                if (gor_us > i2c_gor_max_us) { i2c_gor_max_us = gor_us; }
                if (query_ok) { i2c_query_ok++; } else { i2c_query_fail++; }

                // #399: a run of consecutive unpack failures with the transfer
                // completing (bytes clocked, framing garbage) is the signature
                // of OC slave TX-ring desync — every read misaligned.
                // #402: recover in-band.  Master WRITES still work during a
                // desync, so send I2C_TX_RESYNC (the OC resets its slave
                // device) and suspend ALL polling for a grace window so the
                // reset can't race a transaction.  Re-attempt every 8 further
                // failures until the channel recovers.
                static uint32_t consec_read_fails = 0;
                if (query_ok) {
                    if (i2c_resync_active) {
                        i2c_resync_active = false;
                        ESP_LOGW(TAG, "[I2C] channel RECOVERED after RESYNC (#402)");
                    }
                    consec_read_fails = 0;
                } else {
                    consec_read_fails++;
                    ESP_LOGW(TAG, "[I2C] read FAIL cmd=0x%02X dur=%lu us",
                                  (unsigned)out_pending_command,
                                  (unsigned long)gor_us);
                    if (consec_read_fails == 8) {
                        ESP_LOGE(TAG, "[I2C] 8 consecutive read failures — OC slave "
                                      "TX-ring desync (aborted read left residue, "
                                      "#402). Sending RESYNC + suspending polls.");
                    }
                    if (consec_read_fails >= 8 && (consec_read_fails % 8) == 0) {
                        (void)i2c_interface.sendMessage(I2C_TX_RESYNC, nullptr, 0, 10);
                        i2c_resync_grace_until_ms = now_ms + config::I2C_RESYNC_GRACE_MS;
                        i2c_resync_active = true;
                        query_pending = false;  // pipeline state is stale after the quiet window
                        ESP_LOGW(TAG, "[I2C] RESYNC sent — bus quiet for %u ms",
                                      (unsigned)config::I2C_RESYNC_GRACE_MS);
                    }
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

            // Push the FC firmware version to the OC every ~8 polls (~2 s) so it
            // can relay it to the app (config "fc_identity"); the app uses it to
            // tell a real FC-OTA update from a rollback. Static string, separate
            // message type so it bypasses the OC's timestamp dedup. Skipped while
            // an image is streaming in (the version can't change mid-OTA and the
            // OC already has it — keep the OTA control bus quiet). #8 Phase 4.
            //
            // Also skipped during a SIM: the #393 exception keeps this poll block
            // running through a sim's INFLIGHT (for SIM_STOP), so this extra I2C
            // transaction would fire every ~8 polls (2.008 s = 8 × the ~251 ms
            // polled interval). loop_fc() is single-threaded, so that transaction
            // stalls the loop long enough (>5 ms) to punch a gap into the
            // synthetic sensor stream — which, unlike the real path, is re-stamped
            // with micros() at drain time, so a loop stall shows up as a timestamp
            // gap across ISM6/BMP/NonSensor together. The version can't change on a
            // bench sim and the OC already cached it from the pre-sim READY/
            // PRELAUNCH pushes, so there is nothing to relay mid-sim.
            if (!fc_ota_data_mode && !sensor_collector.isSimActive())
            {
                static uint32_t fc_id_throttle = 1000;  // fire on first poll, then ~2 s
                if (++fc_id_throttle >= 8)
                {
                    fc_id_throttle = 0;
                    const esp_app_desc_t* d = esp_app_get_description();
                    const char* v = (d && d->version[0]) ? d->version : "unknown";
                    (void)i2c_interface.sendMessage(
                        FC_IDENTITY, reinterpret_cast<const uint8_t*>(v),
                        strlen(v), 10);
                }
            }

            // NOTE: mutex is held through command processing below, so
            // readConfigFrame() I2C reads have an idle bus.  Released after
            // the command dispatch block.
        }

        // ==========================================================================
        // SECTION: Command dispatch from the Out Computer
        // ==========================================================================
        // Dedup: OutComputer repeats each command for 5 polls for I2C
        // reliability.  Process only the first delivery.  out_pending_command
        // mirrors the OC's reported command (set every poll above, including 0),
        // so it stays non-zero for the whole repeat window — we must NOT clear
        // it at dispatch, or the reset below would fire between polls and every
        // repeat would re-execute.  last_processed_cmd resets only when the OC
        // actually reports 0 (repeat window done), freeing the same command to
        // be issued again later.
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
                    piezoStart(2600, 100);
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
                delay_ms(1);  // let slave TX FIFO settle
                uint8_t cfg_payload[sizeof(ServoConfigData)];
                size_t  cfg_len = 0;
                if (readConfigFrame(SERVO_CONFIG_MSG, sizeof(ServoConfigData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(ServoConfigData))
                {
                    ServoConfigData cfg;
                    memcpy(&cfg, cfg_payload, sizeof(cfg));
                    ESP_LOGI(TAG, "[SERVO CFG] bias=[%d,%d,%d,%d] hz=%d min=%d max=%d fin=[%.1f,%.1f]",
                                  cfg.bias_us[0], cfg.bias_us[1],
                                  cfg.bias_us[2], cfg.bias_us[3],
                                  cfg.hz, cfg.min_us, cfg.max_us,
                                  cfg.fin_min_deg, cfg.fin_max_deg);

                    for (int i = 0; i < 4; ++i)
                    {
                        servo_control.setBias(i, cfg.bias_us[i]);
                    }
                    if (rocket_state != INFLIGHT)
                    {
                        servo_control.setServoTiming(cfg.hz, cfg.min_us, cfg.max_us);
                        // #267: apply fin-angle calibration (skip a degenerate span)
                        if (cfg.fin_max_deg != cfg.fin_min_deg)
                            servo_control.setFinCalibration(cfg.fin_min_deg, cfg.fin_max_deg);
                    }
                    else
                    {
                        ESP_LOGW(TAG, "[SERVO CFG] Hz/min/max/fin deferred (INFLIGHT)");
                    }

                    // #345 follow-up: make trim live on the pad.  Drive the
                    // servos to the freshly-trimmed centre and arm the wake
                    // window so READY/PRELAUNCH stop relaxing them for a few
                    // seconds — otherwise setBias just updates offsets and the
                    // next idle() strips the pulse train, so the operator never
                    // sees the fin move.  Skipped while a test/replay owns the
                    // servos (they suppress idle already and drive their own
                    // angles) and while INFLIGHT (never relax in flight).
                    if (servo_enabled && rocket_state != INFLIGHT &&
                        !servo_test_active && !ground_test_active && !servo_replay_active)
                    {
                        // Drive to the commanded neutral (0deg incl. the new trim,
                        // via the fin calibration) — the same path the boot-settle
                        // and flight neutral use — so the preview shows the tab
                        // exactly where a 0-command holds it, not the raw pulse
                        // midpoint (which differs under an asymmetric fin cal).
                        // #407: approach it anti-backlash (overshoot then settle)
                        // so the preview is repeatable even on a slightly worn
                        // servo; serviceNeutralSettle() completes it over the next
                        // ticks, inside the wake window armed just below.
                        servo_control.beginNeutralSettle(now_ms);
                        servo_pad_wake_until_ms = now_ms + config::SERVO_PAD_TRIM_WAKE_MS;
                    }

                    prefs.begin("servo", false);
                    prefs.putShort("b1", cfg.bias_us[0]);
                    prefs.putShort("b2", cfg.bias_us[1]);
                    prefs.putShort("b3", cfg.bias_us[2]);
                    prefs.putShort("b4", cfg.bias_us[3]);
                    prefs.putShort("hz", cfg.hz);
                    prefs.putShort("min", cfg.min_us);
                    prefs.putShort("max", cfg.max_us);
                    prefs.putFloat("fmin", cfg.fin_min_deg);
                    prefs.putFloat("fmax", cfg.fin_max_deg);
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
                delay_ms(1);
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

                    applyRollPidGains(cfg.kp, cfg.ki, cfg.kd, cfg.min_cmd, cfg.max_cmd);  // #268

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
                delay_ms(5);  // let slave TX FIFO settle
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
                    delay_ms(5);
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
                // #508: carry the sim's pad frame onto the board's ACTUAL resting
                // attitude before the first synthetic sample. Without this, a
                // bench board lying flat makes rocket-frame gravity jump ~90° the
                // instant sim data replaces real data; the EKF (accel/mag updates
                // still live on the pad) can only explain that step by blaming the
                // gyro bias, which slams to ~-190 dps and is then frozen in at
                // launch — wrecking attitude for the whole flight.
                if (pad_up_rocket_valid)
                {
                    const float mag_rocket[3] = {
                        (float)iis2mdc_latest_si.mag_x_uT,
                        (float)iis2mdc_latest_si.mag_y_uT,
                        (float)iis2mdc_latest_si.mag_z_uT };
                    sensor_collector.configureSimPadAlignment(
                        pad_up_rocket, have_iis2mdc_si ? mag_rocket : nullptr);
                }
                else
                {
                    ESP_LOGW(TAG, "[SIM] no pad orientation estimate yet — starting "
                                  "UNALIGNED; expect a gyro-bias excursion if the "
                                  "board isn't nose-up (#508)");
                }

                sensor_collector.startSim(ground_pressure_pa);
                ESP_LOGI(TAG, "[SIM] Start cmd received, sim active=%s ground_p=%.0f",
                              sensor_collector.isSimActive() ? "YES" : "NO",
                              (double)ground_pressure_pa);
            }
            else if (out_pending_command == SIM_STOP_CMD)
            {
                // #393: an explicit Stop aborts the sim flight back to READY.
                // Reset here (not on the isSimActive falling edge) so it fires
                // only for a real user Stop, never a naturally-completed sim.
                // Works whether the sim is still airborne (delivered mid-flight
                // via the #393 INFLIGHT-poll exception) or already LANDED.
                sensor_collector.stopSim();
                resetFlightStateForSim("stop");
                ESP_LOGI(TAG, "[SIM] Stop cmd received — flight state reset (#393)");
            }
            else if (out_pending_command == GROUND_TEST_START)
            {
                if (isCommandLockoutState(rocket_state)) {
                    ESP_LOGW(TAG, "[GROUND TEST] Rejected — state=%u (no test commands while INFLIGHT or in MAG_CALIBRATION)",
                             (unsigned)rocket_state);
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
                out_status_query_data.hg_bias_x_cmss = encodeHgBiasCmss(sensor_collector.hg_bias_x);
                out_status_query_data.hg_bias_y_cmss = encodeHgBiasCmss(sensor_collector.hg_bias_y);
                out_status_query_data.hg_bias_z_cmss = encodeHgBiasCmss(sensor_collector.hg_bias_z);
                // Persist to NVS — high-g bias + gyro zero-rate bias (#132).
                prefs.begin("cal", false);
                prefs.putFloat("hgbx", sensor_collector.hg_bias_x);
                prefs.putFloat("hgby", sensor_collector.hg_bias_y);
                prefs.putFloat("hgbz", sensor_collector.hg_bias_z);
                prefs.putShort("gx", sensor_collector_hw.gyro_cal_x);
                prefs.putShort("gy", sensor_collector_hw.gyro_cal_y);
                prefs.putShort("gz", sensor_collector_hw.gyro_cal_z);
                prefs.end();
                ESP_LOGI(TAG, "[CAL] Sensor calibration complete (saved to NVS)");
                // Push the result up so the app can snapshot it into the profile.
                publishSensorCalFromNVS();
            }
            // Issue #96 — magnetometer hard-iron cal: start / abort / accept / retry.
            // Entry refused only from INFLIGHT (everything else is fair game).
            // The gate is wide because outdoor rockets enter PRELAUNCH quickly
            // from GPS lock; with #216 the MAG_CALIBRATION state now inhibits
            // launch detection, pyro arming, EKF init, servo PWM, and the BLE
            // test commands, so the cal session is safe on the pad even with
            // continuity-armed pyros.  See `case MAG_CALIBRATION:` below for
            // the full list of in-state inhibitions.  All transitions produce
            // one immediate status frame so the iOS UI updates without
            // waiting for the 5 Hz cadence.
            else if (out_pending_command == MAG_CAL_START)
            {
                if (rocket_state == INFLIGHT)
                {
                    ESP_LOGW(TAG, "[MAGCAL] start refused: INFLIGHT");
                }
                else
                {
                    // Cache the currently-persisted offsets so a session-level
                    // abort (or verify-fail abort) can roll back to them.
                    // Default to zero if no prior cal is saved.
                    mag_cal_prior_cx = 0;
                    mag_cal_prior_cy = 0;
                    mag_cal_prior_cz = 0;
                    prefs.begin("mag_cal", true);  // read-only
                    if (prefs.isKey("ver")
                        && prefs.getUChar("ver", 0) == MAG_CAL_NVS_SCHEMA_VERSION
                        && prefs.getBool("done", false))
                    {
                        mag_cal_prior_cx = (int16_t)prefs.getShort("cx", 0);
                        mag_cal_prior_cy = (int16_t)prefs.getShort("cy", 0);
                        mag_cal_prior_cz = (int16_t)prefs.getShort("cz", 0);
                    }
                    prefs.end();

                    // Zero the chip's OFFSET regs so SAMPLING sees raw mag
                    // values.  Without this a recalibration fits the residual
                    // hard-iron (post-prior-offset), then overwrites the prior
                    // cal with that partial value and fails verification.
                    if (sensor_collector.isIIS2MDCActive())
                    {
                        const bool ok = sensor_collector.setIIS2MDCHardIronOffset(0, 0, 0);
                        ESP_LOGI(TAG, "[MAGCAL] start: zeroed IIS2MDC OFFSET %s  "
                                      "prior=(%d,%d,%d)",
                                 ok ? "OK" : "FAIL",
                                 (int)mag_cal_prior_cx, (int)mag_cal_prior_cy,
                                 (int)mag_cal_prior_cz);
                    }

                    ESP_LOGI(TAG, "[MAGCAL] start: entering MAG_CALIBRATION (from state=%u)",
                             (unsigned)rocket_state);
                    rocket_state = MAG_CALIBRATION;
                    mag_calibrator.start();
                    mag_cal_session_active = true;
                    mag_cal_status_dirty = true;

                    // Issue #216 — entering MAG_CALIBRATION must NOT leave
                    // any flight-time effects active.  Specifically:
                    //   * Stow servos so a deflection commanded earlier
                    //     (e.g. ground test, or last INFLIGHT angle that
                    //     survived a reboot-recovery) doesn't continue to
                    //     drive the PWM while the user tumbles the rocket.
                    //   * Reset kinematics so the tumble can't latch
                    //     kinematics.launch_flag = true.  Without this,
                    //     accepting the cal and returning to READY would
                    //     leave a stale launch_flag that the next
                    //     PRELAUNCH tick would immediately re-promote to
                    //     INFLIGHT (with pyro arming).  See the
                    //     `case MAG_CALIBRATION:` block in the state
                    //     machine below, which also skips kinematicChecks
                    //     for the same reason.
                    if (servo_enabled) {
                        servo_control.stowControl();
                        ESP_LOGI(TAG, "[MAGCAL] start: servos stowed for cal session");
                    }
                    kinematics.reset();
                    burnout_detected = false;
                    burnout_neg_count = 0;
                    tr::deploymentReset(deployment_state);
                    deployment_detected = false;
                    deployment_time_ms  = 0;
                }
            }
            else if (out_pending_command == MAG_CAL_ABORT)
            {
                ESP_LOGI(TAG, "[MAGCAL] abort");
                // Full session ends — restore the prior NVS offsets to the
                // chip (regardless of which sub-state we're aborting from).
                // Same code path for "abort during sampling", "abort during
                // review", and "abort during verifying".
                if (mag_cal_session_active && sensor_collector.isIIS2MDCActive())
                {
                    const bool ok = sensor_collector.setIIS2MDCHardIronOffset(
                        mag_cal_prior_cx, mag_cal_prior_cy, mag_cal_prior_cz);
                    ESP_LOGI(TAG, "[MAGCAL] abort — restored prior OFFSET (%d,%d,%d) %s",
                             (int)mag_cal_prior_cx, (int)mag_cal_prior_cy,
                             (int)mag_cal_prior_cz, ok ? "OK" : "FAIL");
                }
                mag_cal_session_active = false;
                mag_cal_verify_active = false;
                mag_calibrator.abort();
                mag_cal_status_dirty = true;
                if (rocket_state == MAG_CALIBRATION) rocket_state = READY;
                // Calibrator is now in ABORTED — leave it there.  iOS
                // treats .aborted and .idle the same (introSection).
                // calibrator.start() on the next session resets cleanly.
            }
            else if (out_pending_command == MAG_CAL_RETRY)
            {
                if (rocket_state != MAG_CALIBRATION)
                {
                    ESP_LOGW(TAG, "[MAGCAL] retry refused: not in MAG_CALIBRATION");
                }
                else
                {
                    ESP_LOGI(TAG, "[MAGCAL] retry");
                    // Retry restarts SAMPLING within the same session, so the
                    // chip must be at zero (not the prior NVS offsets) so the
                    // new sphere fit sees raw mag values.  Only matters when
                    // retrying out of VERIFYING — in REVIEW/SAMPLING the chip
                    // is already at zero from MAG_CAL_START.
                    if (mag_cal_verify_active && sensor_collector.isIIS2MDCActive())
                    {
                        const bool ok = sensor_collector.setIIS2MDCHardIronOffset(0, 0, 0);
                        ESP_LOGI(TAG, "[MAGCAL] retry during VERIFYING — zeroed OFFSET %s",
                                 ok ? "OK" : "FAIL");
                    }
                    mag_cal_verify_active = false;
                    mag_calibrator.retry();
                    mag_cal_status_dirty = true;
                }
            }
            else if (out_pending_command == MAG_CAL_COMPUTE_FIT)
            {
                if (rocket_state != MAG_CALIBRATION)
                {
                    ESP_LOGW(TAG, "[MAGCAL] compute_fit refused: not in MAG_CALIBRATION");
                }
                else if (!mag_calibrator.computeFit())
                {
                    ESP_LOGW(TAG, "[MAGCAL] compute_fit refused: insufficient samples");
                    // Still publish a status frame so the iOS button
                    // doesn't appear stuck.
                    mag_cal_status_dirty = true;
                }
                else
                {
                    ESP_LOGI(TAG, "[MAGCAL] user-triggered fit complete, REVIEW state");
                    mag_cal_status_dirty = true;
                }
            }
            else if (out_pending_command == MAG_CAL_ACCEPT)
            {
                if (rocket_state != MAG_CALIBRATION)
                {
                    ESP_LOGW(TAG, "[MAGCAL] accept refused: not in MAG_CALIBRATION");
                }
                else if (!mag_calibrator.accept())
                {
                    ESP_LOGW(TAG, "[MAGCAL] accept refused: no fit available (still SAMPLING?)");
                }
                else
                {
                    int16_t cx, cy, cz;
                    float R_uT, res_uT;
                    uint8_t reject;
                    mag_calibrator.getResult(cx, cy, cz, R_uT, res_uT, reject);

                    // Prior offsets were already cached at MAG_CAL_START; we
                    // just program the proposed-new ones on the chip so the
                    // verify window measures the corrected stream.  On verify
                    // pass we'll persist; on fail we zero the chip and the
                    // session abort path takes care of full prior restore.
                    if (sensor_collector.isIIS2MDCActive())
                    {
                        const bool ok = sensor_collector.setIIS2MDCHardIronOffset(cx, cy, cz);
                        ESP_LOGI(TAG, "[MAGCAL] IIS2MDC OFFSET applied (pre-verify): "
                                      "(%d,%d,%d) %s  prior=(%d,%d,%d)",
                                 (int)cx, (int)cy, (int)cz, ok ? "OK" : "FAIL",
                                 (int)mag_cal_prior_cx, (int)mag_cal_prior_cy, (int)mag_cal_prior_cz);
                    }
                    // MMC path is the same regardless; clear any prior
                    // converter-side offset (sphere fit runs against
                    // IIS2MDC LSB units, not MMC counts).
                    sensor_converter.setMMCOffset(0, 0, 0);

                    // Enter verification window — the per-tick block
                    // below polls esp_timer_get_time() and calls
                    // evaluateVerify() after MAG_CAL_VERIFY_DURATION_US.
                    mag_cal_verify_start_us = esp_timer_get_time();
                    mag_cal_verify_eval_now = false;  // fresh window — no stale Done (#382)
                    mag_cal_verify_active = true;
                    mag_cal_status_dirty = true;
                    ESP_LOGI(TAG, "[MAGCAL] accept: VERIFYING — user-driven (Done button) "
                                  "with %lld µs safety timeout",
                             (long long)MAG_CAL_VERIFY_DURATION_US);
                }
            }
            // #148 — user-driven verify completion.  iOS sends DONE when
            // the user is satisfied with the verify rotation; the FC then
            // evaluates the accumulators immediately (instead of waiting
            // for the 60 s safety timeout to fire).  Same dispatch logic
            // as the per-tick timer block — kept in sync by sharing the
            // same evaluateVerify + pass/fail handling below.
            else if (out_pending_command == MAG_CAL_VERIFY_DONE)
            {
                if (rocket_state != MAG_CALIBRATION || !mag_cal_verify_active)
                {
                    ESP_LOGW(TAG, "[MAGCAL] verify_done refused: not in VERIFYING");
                }
                else
                {
                    // #382: explicit flag — the old zero-the-timestamp trick
                    // ((now - 0) >= 60 s) was a no-op for the first 60 s of
                    // uptime, silently deferring an early Done tap to the
                    // safety timeout.
                    mag_cal_verify_eval_now = true;
                    ESP_LOGI(TAG, "[MAGCAL] verify_done: user requested evaluation");
                }
            }
            // #148 — user-override save.  iOS sends this from two paths:
            //   1. Review screen "Save and apply" — skip Verify entirely.
            //      Chip OFFSET still needs to be programmed.
            //   2. Verifying screen "Save anyway" — gates partially red;
            //      chip OFFSET was programmed by the prior MAG_CAL_ACCEPT.
            // We accept from either REVIEW or VERIFYING.  The OFFSET-reg
            // write is idempotent so we do it unconditionally — covers
            // the "from REVIEW" path without breaking the "from VERIFYING"
            // path (chip just rewrites the same values).
            else if (out_pending_command == MAG_CAL_FORCE_APPLY)
            {
                if (rocket_state != MAG_CALIBRATION || !mag_cal_session_active)
                {
                    ESP_LOGW(TAG, "[MAGCAL] force_apply refused: not in MAG_CALIBRATION session");
                }
                else
                {
                    int16_t cx, cy, cz;
                    float R_uT, res_uT;
                    uint8_t reject;
                    mag_calibrator.getResult(cx, cy, cz, R_uT, res_uT, reject);
                    // R_uT == 0 means no fit has run — refuse rather than
                    // persist garbage.
                    if (R_uT <= 0.0f)
                    {
                        ESP_LOGW(TAG, "[MAGCAL] force_apply refused: no fit available");
                    }
                    else
                    {
                        // Program chip OFFSET (idempotent — same regs if
                        // we already wrote them via MAG_CAL_ACCEPT).
                        if (sensor_collector.isIIS2MDCActive())
                        {
                            const bool ok = sensor_collector.setIIS2MDCHardIronOffset(cx, cy, cz);
                            ESP_LOGI(TAG, "[MAGCAL] FORCE_APPLY OFFSET (%d,%d,%d) %s",
                                     (int)cx, (int)cy, (int)cz, ok ? "OK" : "FAIL");
                        }
                        sensor_converter.setMMCOffset(0, 0, 0);

                        prefs.begin("mag_cal", false);
                        prefs.putUChar("ver",    MAG_CAL_NVS_SCHEMA_VERSION);
                        prefs.putBool ("done",   true);
                        prefs.putShort("cx",     cx);
                        prefs.putShort("cy",     cy);
                        prefs.putShort("cz",     cz);
                        prefs.putFloat("R_uT",   R_uT);
                        prefs.putFloat("res_uT", res_uT);
                        prefs.putInt  ("mmc_cx", 0);
                        prefs.putInt  ("mmc_cy", 0);
                        prefs.putInt  ("mmc_cz", 0);
                        prefs.end();

                        ESP_LOGI(TAG, "[MAGCAL] FORCE_APPLY — saved cx=%d cy=%d cz=%d R=%.2fµT res=%.2fµT "
                                      "(user override, gates not re-checked)",
                                 (int)cx, (int)cy, (int)cz, (double)R_uT, (double)res_uT);

                        mag_cal_session_active = false;
                        mag_cal_verify_active = false;
                        mag_cal_status_dirty = true;
                        rocket_state = READY;
                        // Leave the calibrator in APPLIED (not IDLE) so
                        // the next status frame shows iOS the "Saved"
                        // banner — same reason as the verify-PASS path.
                        mag_calibrator.markApplied();
                    }
                }
            }
            // #148 — user wants to redo the verify rotation without
            // going all the way back to SAMPLING.  Clears verify
            // min/max/coverage; the proposed-new cal stays on the chip
            // so the next rotation pass measures the same corrected
            // stream.
            else if (out_pending_command == MAG_CAL_VERIFY_RESET)
            {
                if (rocket_state != MAG_CALIBRATION || !mag_cal_verify_active)
                {
                    ESP_LOGW(TAG, "[MAGCAL] verify_reset refused: not in VERIFYING");
                }
                else
                {
                    mag_calibrator.resetVerify();
                    // Restart the safety timer so the user gets another
                    // full window after a reset.
                    mag_cal_verify_start_us = esp_timer_get_time();
                    mag_cal_verify_eval_now = false;  // fresh window — no stale Done (#382)
                    mag_cal_status_dirty = true;
                    ESP_LOGI(TAG, "[MAGCAL] verify_reset: accumulators cleared, timer restarted");
                }
            }
            // Issue #132 — app pushes a saved cal from the active rocket profile
            // back into FC NVS.  Bypasses the sampling / sphere-fit flow entirely
            // but writes the same NVS keys as MAG_CAL_ACCEPT so the boot-time
            // load path is identical.  Gated to READY (same as MAG_CAL_START) so
            // the IIS2MDC OFFSET registers don't change mid-flight or mid-cal.
            else if (out_pending_command == MAG_CAL_APPLY_PENDING)
            {
                if (rocket_state != READY)
                {
                    ESP_LOGW(TAG, "[MAGCAL] apply refused: state=%u (require READY)",
                             (unsigned)rocket_state);
                }
                else
                {
                    delay_ms(1);
                    uint8_t cfg_payload[sizeof(MagCalApplyData)];
                    size_t  cfg_len = 0;
                    if (readConfigFrame(MAG_CAL_APPLY_MSG, sizeof(MagCalApplyData),
                                        cfg_payload, sizeof(cfg_payload), cfg_len)
                        && cfg_len >= sizeof(MagCalApplyData))
                    {
                        MagCalApplyData ap;
                        memcpy(&ap, cfg_payload, sizeof(ap));

                        if (sensor_collector.isIIS2MDCActive())
                        {
                            const bool ok = sensor_collector.setIIS2MDCHardIronOffset(ap.cx, ap.cy, ap.cz);
                            ESP_LOGI(TAG, "[MAGCAL] APPLY IIS2MDC OFFSET (%d,%d,%d) %s",
                                     (int)ap.cx, (int)ap.cy, (int)ap.cz, ok ? "OK" : "FAIL");
                        }
                        sensor_converter.setMMCOffset(0, 0, 0);

                        prefs.begin("mag_cal", false);
                        prefs.putUChar("ver",    MAG_CAL_NVS_SCHEMA_VERSION);
                        prefs.putBool ("done",   true);
                        prefs.putShort("cx",     ap.cx);
                        prefs.putShort("cy",     ap.cy);
                        prefs.putShort("cz",     ap.cz);
                        prefs.putFloat("R_uT",   ap.R_uT);
                        prefs.putFloat("res_uT", ap.res_uT);
                        prefs.putInt  ("mmc_cx", 0);
                        prefs.putInt  ("mmc_cy", 0);
                        prefs.putInt  ("mmc_cz", 0);
                        prefs.end();

                        ESP_LOGI(TAG, "[MAGCAL] APPLY+saved: cx=%d cy=%d cz=%d R=%.2fµT res=%.2fµT",
                                 (int)ap.cx, (int)ap.cy, (int)ap.cz,
                                 (double)ap.R_uT, (double)ap.res_uT);

                        publishMagCalStatusFromNVS();
                    }
                    else
                    {
                        ESP_LOGE(TAG, "[MAGCAL] APPLY payload read failed");
                    }
                }
            }
            // Issue #132 — pure NVS query.  No state side-effect; publishes one
            // status frame so the app can diff against the active profile and
            // decide push / pull / re-cal.
            else if (out_pending_command == MAG_CAL_READ)
            {
                ESP_LOGI(TAG, "[MAGCAL] READ -> publish status from NVS");
                publishMagCalStatusFromNVS();
            }
            // Issue #132 — app pushes a saved sensor cal (gyro + high-g) back
            // into NVS and applies it.  Gated to READY like the pad cal.
            else if (out_pending_command == SENSOR_CAL_APPLY_PENDING)
            {
                if (rocket_state != READY)
                {
                    ESP_LOGW(TAG, "[SENSORCAL] apply refused: state=%u (require READY)",
                             (unsigned)rocket_state);
                }
                else
                {
                    delay_ms(1);
                    uint8_t cfg_payload[sizeof(SensorCalApplyData)];
                    size_t  cfg_len = 0;
                    if (readConfigFrame(SENSOR_CAL_APPLY_MSG, sizeof(SensorCalApplyData),
                                        cfg_payload, sizeof(cfg_payload), cfg_len)
                        && cfg_len >= sizeof(SensorCalApplyData))
                    {
                        SensorCalApplyData ap;
                        memcpy(&ap, cfg_payload, sizeof(ap));

                        sensor_collector_hw.gyro_cal_x = ap.gyro_x;
                        sensor_collector_hw.gyro_cal_y = ap.gyro_y;
                        sensor_collector_hw.gyro_cal_z = ap.gyro_z;
                        sensor_converter.setHighGBias(ap.hg_x, ap.hg_y, ap.hg_z);
                        out_status_query_data.hg_bias_x_cmss = encodeHgBiasCmss(ap.hg_x);
                        out_status_query_data.hg_bias_y_cmss = encodeHgBiasCmss(ap.hg_y);
                        out_status_query_data.hg_bias_z_cmss = encodeHgBiasCmss(ap.hg_z);

                        prefs.begin("cal", false);
                        prefs.putFloat("hgbx", ap.hg_x);
                        prefs.putFloat("hgby", ap.hg_y);
                        prefs.putFloat("hgbz", ap.hg_z);
                        prefs.putShort("gx", ap.gyro_x);
                        prefs.putShort("gy", ap.gyro_y);
                        prefs.putShort("gz", ap.gyro_z);
                        prefs.end();

                        ESP_LOGI(TAG, "[SENSORCAL] APPLY+saved: gyro=(%d,%d,%d) hg=(%.3f,%.3f,%.3f)",
                                 (int)ap.gyro_x, (int)ap.gyro_y, (int)ap.gyro_z,
                                 (double)ap.hg_x, (double)ap.hg_y, (double)ap.hg_z);
                        publishSensorCalFromNVS();
                    }
                    else
                    {
                        ESP_LOGE(TAG, "[SENSORCAL] APPLY payload read failed");
                    }
                }
            }
            else if (out_pending_command == SENSOR_CAL_READ)
            {
                ESP_LOGI(TAG, "[SENSORCAL] READ -> publish status from NVS");
                publishSensorCalFromNVS();
            }
            else if (out_pending_command == OTA_BEGIN_PENDING)
            {
                // #8 Phase 4: OTA relay from the OC. Refuse unless READY so an
                // OTA can't start mid-flight (mirrors the cal handlers). Read
                // the image header (size + sha256) and begin the session
                // (erases ota_1). Image bytes arrive over I2S in Layer 3;
                // Layer 2 exercises just this control handshake.
                if (isCommandLockoutState(rocket_state))
                {
                    // Block OTA only in the genuinely-unsafe states (INFLIGHT,
                    // MAG_CALIBRATION) — same lockout as the test commands (#218).
                    // READY/PRELAUNCH/LANDED are all fine: a GPS lock on the
                    // bench puts us in PRELAUNCH, and the OTA reboot lands in a
                    // safe state anyway.
                    ESP_LOGW(TAG, "[OTA] BEGIN refused: state=%u (no OTA while INFLIGHT or in MAG_CALIBRATION)",
                             (unsigned)rocket_state);
                    sendOtaRelayStatus(OTA_RELAY_VERIFY_FAILED, 0, 0);
                }
                else
                {
                    delay_ms(1);
                    uint8_t hdr[36];
                    size_t  hlen = 0;
                    if (readConfigFrame(OTA_BEGIN_MSG, 36, hdr, sizeof(hdr), hlen)
                        && hlen >= 36)
                    {
                        uint32_t total_size = 0;
                        memcpy(&total_size, hdr, 4);
                        ESP_LOGW(TAG, "[OTA] BEGIN size=%u — erasing ota_1", (unsigned)total_size);
                        const TR_OTA_Receiver::Error e = fc_ota_receiver.begin(total_size, hdr + 4);
                        if (e == TR_OTA_Receiver::Error::Ok)
                        {
                            ESP_LOGI(TAG, "[OTA] ready for image");
                            fc_ota_total_size = total_size;
                            sendOtaRelayStatusRobust(OTA_RELAY_READY, 0, 0);
                            // Layer 3: flip to slave RX and receive the image
                            // over the (now reversed) I2S link. Blocks until the
                            // READY resends have drained off the still-TX link.
                            fcFlipToRx();
                        }
                        else
                        {
                            ESP_LOGE(TAG, "[OTA] begin failed: %d", (int)e);
                            sendOtaRelayStatusRobust(OTA_RELAY_VERIFY_FAILED, (uint8_t)e, 0);
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG, "[OTA] BEGIN header read failed");
                        sendOtaRelayStatus(OTA_RELAY_VERIFY_FAILED, 0, 0);
                    }
                }
            }
            else if (out_pending_command == OTA_FINISH_CMD)
            {
                // Layer 3: the OC pumped the tail of the image just before sending
                // FINISH; give the parser a moment to drain the last in-flight frames,
                // then revert the I2S link to master-TX so the terminal status rides
                // the normal-direction path back. Only finalize when an OTA session is
                // actually active (fc_ota_data_mode); a FINISH with no session is a
                // stale/duplicate (handled in the else). On success the FC reboots.
                if (fc_ota_data_mode)
                {
                    // Wait for the image tail to land. The OC drains its TX queue and
                    // keeps the link clocked briefly after FINISH, so bytesWritten
                    // should reach the total (it stalled 1 frame short before this).
                    for (int i = 0; i < 200 &&
                            (uint32_t)fc_ota_receiver.bytesWritten() < fc_ota_total_size; i++)
                        delay_ms(5);   // up to ~1 s for the tail to land
                    // Do NOT seize the bus as master TX until the OC has stopped
                    // driving BCLK (reverted to slave RX). Otherwise both ends drive
                    // BCLK = contention and the terminal status is garbled — and the
                    // final frame is lost. Silence == the I2S RX callback stops firing.
                    uint32_t last_cb = fc_ota_rx_cb_count;
                    int quiet = 0;
                    for (int i = 0; i < 400 && quiet < 20; i++)   // ~100 ms quiet, <=2 s cap
                    {
                        delay_ms(5);
                        const uint32_t cb = fc_ota_rx_cb_count;
                        if (cb == last_cb) { quiet++; }
                        else { quiet = 0; last_cb = cb; }
                    }
                    fcRevertToTx();

                    ESP_LOGW(TAG, "[OTA] FINISH (bytes_written=%u/%u)",
                             (unsigned)fc_ota_receiver.bytesWritten(),
                             (unsigned)fc_ota_total_size);
                    // RX diagnostics summary (bench): which failure mode stalled it?
                    //   write_fail>0  -> flash-write error (not a transport issue)
                    //   gap>0, crc_fail~0 -> frames lost/reordered (forward-only pump)
                    //   crc_fail high -> I2S signal integrity (back off BCLK)
                    ESP_LOGW(TAG, "[OTA RX] frames_ok=%u gap=%u crc_fail=%u write_fail=%u ring_ovf=%u",
                             (unsigned)fc_ota_written_frames, (unsigned)fc_ota_gap,
                             (unsigned)fc_ota_crc_fail, (unsigned)fc_ota_write_fail,
                             (unsigned)fc_ota_ring_ovf);
                    const TR_OTA_Receiver::Error e = fc_ota_receiver.finish();
                    if (e == TR_OTA_Receiver::Error::Ok)
                    {
                        // Robust resend spans ~1.4 s, which also gives the app
                        // time to catch ready_to_boot before the I2S link drops at
                        // reboot (replaces the old fixed 500 ms pre-restart delay).
                        sendOtaRelayStatusRobust(OTA_RELAY_READY_TO_BOOT, 0,
                                                 (uint32_t)fc_ota_receiver.bytesWritten());
                        esp_restart();
                    }
                    else
                    {
                        ESP_LOGE(TAG, "[OTA] finish failed: %d", (int)e);
                        sendOtaRelayStatusRobust(OTA_RELAY_VERIFY_FAILED, (uint8_t)e,
                                                 (uint32_t)fc_ota_receiver.bytesWritten());
                    }
                }
                else
                {
                    // Stale/duplicate FINISH with no active OTA session — e.g. the OC's
                    // pre-loaded I2C status response, read by the freshly-rebooted FC
                    // right after a *successful* OTA (bench: "finish failed: 2" at boot).
                    // finish() here only returns SessionNotActive and logs a scary error,
                    // so a completed OTA appears to end in failure. Ignore it instead —
                    // the FC is idempotent to a duplicate FINISH regardless of why the
                    // OC re-sent it.
                    ESP_LOGI(TAG, "[OTA] FINISH ignored — no active OTA session (stale/duplicate)");
                }
            }
            else if (out_pending_command == OTA_ABORT_CMD)
            {
                ESP_LOGW(TAG, "[OTA] ABORT");
                if (fc_ota_data_mode) fcRevertToTx();  // back to TX so status rides I2S
                (void)fc_ota_receiver.abort();
                sendOtaRelayStatusRobust(OTA_RELAY_ABORTED, 0, 0);
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
                guidance_tilt_inhibited = false;   // clear tilt-limit latch
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
                delay_ms(1);
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
            else if (out_pending_command == ORIENT_CONFIG_PENDING)
            {
                delay_ms(1);
                uint8_t cfg_payload[sizeof(ImuOrientConfigData)];
                size_t  cfg_len = 0;
                if (readConfigFrame(ORIENT_CONFIG_MSG, sizeof(ImuOrientConfigData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(ImuOrientConfigData))
                {
                    const uint8_t setting = cfg_payload[0];
                    if (rocket_state == INFLIGHT)
                    {
                        // Never re-frame mid-flight: EKF/control state is
                        // only meaningful in the latched frame.
                        ESP_LOGW(TAG, "[CFG] IMU orientation change ignored INFLIGHT");
                    }
                    else if (setting == IMU_ORIENT_AUTO)
                    {
                        // Back to auto.  Only act when leaving MANUAL —
                        // reset to identity and let the pad-gravity detect
                        // re-derive within a couple of seconds.
                        if (b2r_active_mode == ORIENT_MODE_MANUAL)
                        {
                            applyBoardToRocketOrientation(ORIENT_CODE_IDENTITY,
                                                          ORIENT_MODE_DEFAULT);
                            orient_estimator.reset();
                            if (ekf_initialized) ekf_initialized = false;
                            ESP_LOGI(TAG, "[CFG] IMU orientation: AUTO (pad-gravity detect)");
                        }
                    }
                    else if (setting < ORIENT_CODE_COUNT)
                    {
                        // Manual code is authoritative — it also fixes the
                        // roll clocking the control surfaces need, which
                        // auto-detect cannot observe.
                        const bool changed = (b2r_active_code != setting) ||
                                             (b2r_active_mode != ORIENT_MODE_MANUAL);
                        applyBoardToRocketOrientation(setting, ORIENT_MODE_MANUAL);
                        if (changed && ekf_initialized) ekf_initialized = false;
                        ESP_LOGI(TAG, "[CFG] IMU orientation: MANUAL %s",
                                 orientCodeName(setting));
                    }
                }
            }
            else if (out_pending_command == IMU_RATE_CONFIG_PENDING)
            {
                delay_ms(1);
                uint8_t cfg_payload[sizeof(ImuRateConfigData)];
                size_t  cfg_len = 0;
                if (readConfigFrame(IMU_RATE_CONFIG_MSG, sizeof(ImuRateConfigData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(ImuRateConfigData))
                {
                    uint16_t rate_hz;
                    memcpy(&rate_hz, cfg_payload, sizeof(rate_hz));
                    if (rocket_state == INFLIGHT)
                    {
                        // Never mid-flight: the ODR switch produces one
                        // odd-length inter-sample gap and changes the log
                        // cadence — fine on the pad, not during boost.  This
                        // guard is for the USER setting only; the dynamic
                        // mode's own step-down at deployment is deliberate and
                        // runs from the flight loop, not through here.
                        ESP_LOGW(TAG, "[CFG] IMU rate change ignored INFLIGHT");
                    }
                    else if (imuRateSettingValid(rate_hz))
                    {
                        imu_rate_setting = rate_hz;
                        applyImuRateForFlightPhase();
                        prefs.begin("imu", false);
                        prefs.putUShort("rate", rate_hz);
                        prefs.end();
                        if (imuRateIsDynamic(rate_hz))
                        {
                            ESP_LOGI(TAG, "[CFG] IMU logging rate: DYNAMIC "
                                          "(%u Hz to deployment, then %u Hz) (persisted)",
                                     (unsigned)IMU_RATE_DYNAMIC_BOOST_HZ,
                                     (unsigned)IMU_RATE_DYNAMIC_POST_HZ);
                        }
                        else
                        {
                            ESP_LOGI(TAG, "[CFG] IMU logging rate: %u Hz (persisted)",
                                     (unsigned)rate_hz);
                        }
                    }
                    else
                    {
                        ESP_LOGW(TAG, "[CFG] IMU rate %u Hz rejected "
                                      "(not dynamic/960/1920/3840)",
                                 (unsigned)rate_hz);
                    }
                }
            }
            else if (out_pending_command == PYRO_CONFIG_PENDING)
            {
                delay_ms(1);
                uint8_t cfg_payload[sizeof(PyroConfigData)];
                size_t  cfg_len = 0;
                if (readConfigFrame(PYRO_CONFIG_MSG, sizeof(PyroConfigData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(PyroConfigData))
                {
                    memcpy(&pyro_config, cfg_payload, sizeof(PyroConfigData));
                    prefs.begin("pyro", false);
                    prefs.putBytes("cfg", &pyro_config, sizeof(pyro_config));
                    prefs.end();
                    ESP_LOGI(TAG, "[PYRO CFG] ch1=%u/%u/%.1f  ch2=%u/%u/%.1f  ch3=%u/%u/%.1f  ch4=%u/%u/%.1f",
                             pyro_config.ch1_enabled, pyro_config.ch1_trigger_mode, (double)pyro_config.ch1_trigger_value,
                             pyro_config.ch2_enabled, pyro_config.ch2_trigger_mode, (double)pyro_config.ch2_trigger_value,
                             pyro_config.ch3_enabled, pyro_config.ch3_trigger_mode, (double)pyro_config.ch3_trigger_value,
                             pyro_config.ch4_enabled, pyro_config.ch4_trigger_mode, (double)pyro_config.ch4_trigger_value);
                }
            }
            else if (out_pending_command == PYRO_CONT_TEST)
            {
                // Direct CONT read. VPP (flight-computer power, always on)
                // feeds the sense divider, so no ARM pulse is needed.
                // Rejected in flight to honor the app's "ground tests are
                // pad-only" UX guarantee.
                if (isCommandLockoutState(rocket_state)) {
                    ESP_LOGW(TAG, "[PYRO CONT TEST] Rejected — state=%u (no test commands while INFLIGHT or in MAG_CALIBRATION)",
                             (unsigned)rocket_state);
                } else {
                    delay_ms(1);
                    uint8_t cfg_payload[4];
                    size_t  cfg_len = 0;
                    uint8_t ch = 0;
                    if (readConfigFrame(PYRO_CONT_TEST, 1,
                                        cfg_payload, sizeof(cfg_payload), cfg_len)
                        && cfg_len >= 1) {
                        ch = cfg_payload[0];
                    }
                    if (ch < 1 || ch > 4) {
                        ESP_LOGW(TAG, "[PYRO CONT TEST] Invalid channel %u", ch);
                    } else {
                        const int idx = ch - 1;
                        const gpio_num_t cont_pin = (gpio_num_t)PYRO_CONT_PINS[idx];

                        // Defensive CONT-pad reclaim — peripheral defaults
                        // can re-grab the ESP32-P4 IO MUX between boot and
                        // the test.
                        esp_gpio_revoke(1ULL << cont_pin);
                        gpio_reset_pin(cont_pin);
                        gpio_config_t cont_cfg = {};
                        cont_cfg.pin_bit_mask = 1ULL << cont_pin;
                        cont_cfg.mode         = GPIO_MODE_INPUT;
                        cont_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
                        cont_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
                        gpio_config(&cont_cfg);

                        int raw = gpio_get_level(cont_pin);
                        portENTER_CRITICAL(&pyro_spinlock);
                        pyro_ch[idx].cont       = pyroContFromRaw(raw);
                        pyro_ch[idx].cont_known = true;
                        portEXIT_CRITICAL(&pyro_spinlock);
                        ESP_LOGI(TAG, "[PYRO CONT TEST] CH%u raw=%d cont=%d",
                                 ch, raw, pyroContFromRaw(raw) ? 1 : 0);
                    }
                }
            }
            else if (out_pending_command == PYRO_FIRE_TEST)
            {
                // Test-fire a pyro channel from the app (ground test only)
                if (isCommandLockoutState(rocket_state)) {
                    ESP_LOGW(TAG, "[PYRO FIRE TEST] Rejected — state=%u (no test commands while INFLIGHT or in MAG_CALIBRATION)",
                             (unsigned)rocket_state);
                } else {
                    delay_ms(1);
                    uint8_t cfg_payload[4];
                    size_t  cfg_len = 0;
                    uint8_t ch = 0;
                    if (readConfigFrame(PYRO_FIRE_TEST, 1,
                                        cfg_payload, sizeof(cfg_payload), cfg_len)
                        && cfg_len >= 1) {
                        ch = cfg_payload[0];
                    }
                    if (ch < 1 || ch > 4) {
                        ESP_LOGW(TAG, "[PYRO FIRE TEST] Invalid channel %u", ch);
                    } else {
                        const int idx = ch - 1;
                        const gpio_num_t arm_pin  = (gpio_num_t)config::PYRO_ARM_PIN;
                        const gpio_num_t fire_pin = (gpio_num_t)PYRO_FIRE_PINS[idx];

                        esp_gpio_revoke(1ULL << arm_pin);
                        esp_gpio_revoke(1ULL << fire_pin);
                        // #263: bring the ARM/FIRE pads up through the canonical
                        // safe-init — NOT gpio_reset_pin(), which transiently
                        // enables the internal pull-up and twitches the DTC123J
                        // gate driver (the exact boot-fire hazard initPyroPins
                        // guards against). esp_gpio_revoke() above frees the pads
                        // from any prior reservation first. OUTPUT (not
                        // INPUT_OUTPUT) is enough — the test only drives the pins.
                        safePyroOutputInit(arm_pin);
                        safePyroOutputInit(fire_pin);

                        // ARM → settle → FIRE pulse → disarm (synchronous; ground only)
                        portENTER_CRITICAL(&pyro_spinlock);
                        pyroSetArmLocked(true);
                        portEXIT_CRITICAL(&pyro_spinlock);
                        delay_ms(config::PYRO_ARM_SETTLE_MS);
                        portENTER_CRITICAL(&pyro_spinlock);
                        gpio_set_level(fire_pin, 1);
                        portEXIT_CRITICAL(&pyro_spinlock);

                        delay_ms(config::PYRO_FIRE_DURATION_MS);

                        portENTER_CRITICAL(&pyro_spinlock);
                        gpio_set_level(fire_pin, 0);
                        pyroSetArmLocked(false);
                        // Mark the channel as Done so telemetry shows fired
                        pyro_ch[idx].state          = PyroChState::Done;
                        pyro_ch[idx].phase_start_ms = 0;
                        portEXIT_CRITICAL(&pyro_spinlock);

                        // #263: restore the canonical low-driven safe state (the
                        // old path left the pads in INPUT_OUTPUT after the pulse).
                        safePyroOutputInit(arm_pin);
                        safePyroOutputInit(fire_pin);

                        ESP_LOGI(TAG, "[PYRO FIRE TEST] CH%u fired for %u ms",
                                 ch, (unsigned)config::PYRO_FIRE_DURATION_MS);
                    }
                }
            }
            else if (out_pending_command == SERVO_TEST_PENDING)
            {
                if (isCommandLockoutState(rocket_state)) {
                    ESP_LOGW(TAG, "[SERVO TEST] Rejected — state=%u (no test commands while INFLIGHT or in MAG_CALIBRATION)",
                             (unsigned)rocket_state);
                } else {
                    delay_ms(1);
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
                        // Pulses = LEDC ground truth (angle -> fin cal -> +bias
                        // -> clamp): shows whether each command reached its pin.
                        ESP_LOGI(TAG, "[SERVO TEST] Angles: [%.1f, %.1f, %.1f, %.1f] deg -> pulses [%d, %d, %d, %d] us",
                                      (double)servo_test_angles[0], (double)servo_test_angles[1],
                                      (double)servo_test_angles[2], (double)servo_test_angles[3],
                                      servo_control.getServoPulseUs(0), servo_control.getServoPulseUs(1),
                                      servo_control.getServoPulseUs(2), servo_control.getServoPulseUs(3));
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
                // #407: stow via the anti-backlash approach (overshoot then
                // settle to the fin-cal neutral, same as boot/trim) and arm the
                // pad wake window so the stowed neutral stays driven ~6 s.
                // Without the wake window the next READY/PRELAUNCH tick idle()s
                // the servos and a loaded linkage droops within a tick, so the
                // operator never sees where the test actually stowed.
                servo_control.beginNeutralSettle(now_ms);
                servo_pad_wake_until_ms = now_ms + config::SERVO_PAD_TRIM_WAKE_MS;
                ESP_LOGI(TAG, "[SERVO TEST] Stopped - servos stowed (anti-backlash), wake armed");
            }
            else if (out_pending_command == ROLL_PROFILE_PENDING)
            {
                delay_ms(1);
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
                delay_ms(1);
                uint8_t cfg_payload[sizeof(RollControlConfigData)];
                size_t  cfg_len = 0;
                if (readConfigFrame(ROLL_CTRL_CONFIG_MSG, sizeof(RollControlConfigData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(RollControlConfigData))
                {
                    RollControlConfigData rc;
                    memcpy(&rc, cfg_payload, sizeof(rc));
                    use_angle_control = (rc.use_angle_control != 0);
                    roll_delay_ms     = rc.roll_delay_ms;
                    // Outer-loop rate cap: a positive, sane value overrides the
                    // firmware default; <=0 (or garbage) leaves it unchanged.
                    if (rc.kp_angle_rate_cap_dps > 0.0f && rc.kp_angle_rate_cap_dps <= 2000.0f)
                        kp_angle_rate_cap_dps = rc.kp_angle_rate_cap_dps;
                    // Outer angle-loop P-gain: a positive, sane value overrides.
                    if (rc.kp_angle > 0.0f && rc.kp_angle <= 100.0f)
                        kp_angle_outer = rc.kp_angle;
                    // Integral-separation anti-windup threshold: >=0 applies it
                    // to the live PID (0 disables); <0 (or garbage) leaves it.
                    if (rc.integral_sep_threshold_dps >= 0.0f && rc.integral_sep_threshold_dps <= 2000.0f)
                    {
                        integral_sep_threshold_dps = rc.integral_sep_threshold_dps;
                        applyRollPidSepThreshold(integral_sep_threshold_dps);  // #268: both roll PIDs
                    }
                    ESP_LOGI(TAG, "[ROLL CFG] angle_ctrl=%s delay=%u ms rate_cap=%.0f kp_angle=%.2f iwindup=%.0f",
                                  use_angle_control ? "ON" : "OFF",
                                  (unsigned)roll_delay_ms, (double)kp_angle_rate_cap_dps,
                                  (double)kp_angle_outer, (double)integral_sep_threshold_dps);
                    prefs.begin("servo", false);
                    prefs.putBool("ac", use_angle_control);
                    prefs.putUShort("rdly", roll_delay_ms);
                    prefs.putFloat("rcap", kp_angle_rate_cap_dps);
                    prefs.putFloat("kpang", kp_angle_outer);
                    prefs.putFloat("iwind", integral_sep_threshold_dps);
                    prefs.end();
                    ESP_LOGI(TAG, "[ROLL CFG] Saved to NVS");
                }
                else
                {
                    ESP_LOGW(TAG, "[ROLL CFG] readConfigFrame failed");
                }
            }
            else if (out_pending_command == GUIDANCE_CONFIG_PENDING)
            {
                delay_ms(1);
                uint8_t cfg_payload[sizeof(GuidanceConfigData)];
                size_t  cfg_len = 0;
                if (readConfigFrame(GUIDANCE_CONFIG_MSG, sizeof(GuidanceConfigData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(GuidanceConfigData))
                {
                    GuidanceConfigData g;
                    memcpy(&g, cfg_payload, sizeof(g));
                    guidance_enabled = (g.enable != 0);
                    // Positive/sane values override; out-of-range leaves the default.
                    if (g.nav_gain         > 0.0f && g.nav_gain         <= 20.0f)  pn_nav_gain     = g.nav_gain;
                    if (g.max_accel_mps2   > 0.0f && g.max_accel_mps2   <= 200.0f) pn_max_accel    = g.max_accel_mps2;
                    if (g.accel_to_fin_deg > 0.0f && g.accel_to_fin_deg <= 100.0f) pn_accel_to_fin = g.accel_to_fin_deg;
                    if (g.max_fin_deg      > 0.0f && g.max_fin_deg      <= 60.0f)  pn_max_fin_deg  = g.max_fin_deg;
                    if (g.min_speed_mps   >= 0.0f && g.min_speed_mps   <= 200.0f)  pn_min_speed    = g.min_speed_mps;
                    if (g.target_alt_m     > 0.0f)                                 pn_target_alt   = g.target_alt_m;
                    pn_coast_delay_ms = g.coast_delay_ms;
                    // Station-keep PD gains (#534).  Same convention as above:
                    // positive/sane overrides, out-of-range keeps the prior value.
                    // Upper bounds are saturation bounds, not stability bounds —
                    // at kp = 10 /s^2 a 2 m offset already commands the whole
                    // 20 m/s^2 accel budget, and kd = 10 /s is zeta ~ 5.6 at the
                    // default kp (hopelessly overdamped).  Anything past these is
                    // a typo, not a tune.  kd == 0 is REJECTED on purpose: an
                    // undamped PD regulator oscillates.
                    if (g.kp_pos_per_s2 > 0.0f && g.kp_pos_per_s2 <= 10.0f) pn_kp_pos = g.kp_pos_per_s2;
                    else ESP_LOGW(TAG, "[GUID CFG] kp %.3f out of (0,10] — keeping %.3f",
                                       (double)g.kp_pos_per_s2, (double)pn_kp_pos);
                    if (g.kd_vel_per_s  > 0.0f && g.kd_vel_per_s  <= 10.0f) pn_kd_vel = g.kd_vel_per_s;
                    else ESP_LOGW(TAG, "[GUID CFG] kd %.3f out of (0,10] — keeping %.3f",
                                       (double)g.kd_vel_per_s, (double)pn_kd_vel);
                    // Law: an UNKNOWN value must never select an undefined law.
                    // Whitelist, keep the previous law on anything else.
                    if (g.guidance_law <= GUIDE_LAW_MAX) pn_guidance_law = g.guidance_law;
                    else ESP_LOGW(TAG, "[GUID CFG] law %u unknown — keeping %u",
                                       (unsigned)g.guidance_law, (unsigned)pn_guidance_law);
                    // Target mode is now branched on, so whitelist it too
                    // (previously stored raw).
                    if (g.target_mode == GUIDE_TARGET_OVERHEAD ||
                        g.target_mode == GUIDE_TARGET_POINT)   pn_target_mode = g.target_mode;
                    else ESP_LOGW(TAG, "[GUID CFG] target_mode %u unknown — keeping %u",
                                       (unsigned)g.target_mode, (unsigned)pn_target_mode);
                    // Aim point.  The FC — not the library — owns magnitude
                    // policy (TR_GuidancePN.h).  REJECT and keep the previous
                    // point; do NOT clamp, so the flown aim point always equals
                    // the uploaded and logged one.
                    if (pn_target_mode == GUIDE_TARGET_POINT) {
                        const float r = sqrtf(g.target_e_m * g.target_e_m +
                                              g.target_n_m * g.target_n_m);
                        if (std::isfinite(r) && r <= config::GUIDANCE_MAX_AIM_RADIUS_M) {
                            pn_target_e = g.target_e_m;
                            pn_target_n = g.target_n_m;
                        } else {
                            // "Keep the previous point" means the previous
                            // PROFILE point (the shadows) — NOT the live
                            // pn_target_e/n, which may hold a Drift-Cast
                            // (cmd-28) conversion that the unconditional NVS
                            // persist below would otherwise adopt as a
                            // profile point and self-apply on the next boot
                            // (the exact hazard the cmd-28 RAM-only rule
                            // exists to exclude).
                            pn_target_e = pn_profile_target_e;
                            pn_target_n = pn_profile_target_n;
                            ESP_LOGW(TAG, "[GUID CFG] aim point (%.1f,%.1f) r=%.1f m REJECTED "
                                          "(limit %.0f m) — keeping profile (%.1f,%.1f)",
                                          (double)g.target_e_m, (double)g.target_n_m, (double)r,
                                          (double)config::GUIDANCE_MAX_AIM_RADIUS_M,
                                          (double)pn_target_e, (double)pn_target_n);
                        }
                    } else {
                        // OVERHEAD forces horizontal (0,0) so a stale POINT push
                        // can't pollute it.
                        pn_target_e = 0.0f;
                        pn_target_n = 0.0f;
                    }
                    // pn_target_e/n are now profile-owned again in every
                    // branch — refresh the shadows in lockstep.
                    pn_profile_target_e = pn_target_e;
                    pn_profile_target_n = pn_target_n;
                    // Law first, aim point second — see applyGuidanceConfig().
                    applyGuidanceConfig();
                    // #435: every applied cmd-65 guidance config supersedes a
                    // Drift-Cast geodetic point (pn_target_e/n were just
                    // overwritten above).  Refresh the echo so the app SEES
                    // the wipe — e.g. the connect-time profile push asserting
                    // OVERHEAD over a previously-sent Drift-Cast point.  Bump
                    // tgt_seq ONLY when the echo content actually changed,
                    // so a no-op connect-time push (no cmd-28 point in play)
                    // doesn't churn the app's seq baseline.
                    guid_point_valid = false;
                    (void)guidanceTargetEchoSupersede(
                        out_status_query_data,
                        guidanceFlownTargetSrc(false, pn_target_mode));
                    ESP_LOGI(TAG, "[GUID CFG] en=%s law=%s N=%.1f maxA=%.0f a2f=%.1f maxFin=%.0f "
                                  "minV=%.0f mode=%u alt=%.0f kp=%.2f kd=%.2f aim=(%.1f,%.1f)",
                                  guidance_enabled ? "ON" : "OFF",
                                  pn_guidance_law == GUIDE_LAW_STATION_KEEP ? "STATION_KEEP" : "PN",
                                  (double)pn_nav_gain, (double)pn_max_accel, (double)pn_accel_to_fin,
                                  (double)pn_max_fin_deg, (double)pn_min_speed,
                                  (unsigned)pn_target_mode, (double)pn_target_alt,
                                  (double)pn_kp_pos, (double)pn_kd_vel,
                                  (double)pn_target_e, (double)pn_target_n);
                    prefs.begin("servo", false);
                    prefs.putBool("guid_en", guidance_enabled);
                    prefs.putFloat("gng", pn_nav_gain);
                    prefs.putFloat("gma", pn_max_accel);
                    prefs.putFloat("gaf", pn_accel_to_fin);
                    prefs.putFloat("gmf", pn_max_fin_deg);
                    prefs.putFloat("gms", pn_min_speed);
                    prefs.putUShort("gcd", pn_coast_delay_ms);
                    prefs.putUChar("gtm", pn_target_mode);
                    prefs.putFloat("gte", pn_target_e);
                    prefs.putFloat("gtn", pn_target_n);
                    prefs.putFloat("gta", pn_target_alt);
                    prefs.putFloat("gkp", pn_kp_pos);
                    prefs.putFloat("gkd", pn_kd_vel);
                    prefs.putUChar("glw", pn_guidance_law);
                    prefs.end();
                    ESP_LOGI(TAG, "[GUID CFG] Saved to NVS");
                }
                else
                {
                    // #534: this is now ALSO the "app is older than the
                    // firmware" path — a 36-byte-era app frame fails the
                    // cfg_len >= sizeof(GuidanceConfigData) gate.  Echo the
                    // lengths so a version skew is diagnosable from the log
                    // instead of looking like an I2C fault.
                    ESP_LOGW(TAG, "[GUID CFG] readConfigFrame FAILED — got %u bytes, need %u. "
                                  "If the FC is healthy this is an OUT-OF-DATE APP: guidance "
                                  "config was NOT applied and the previous config still flies.",
                                  (unsigned)cfg_len, (unsigned)sizeof(GuidanceConfigData));
                }
            }
            else if (out_pending_command == GUIDANCE_POINT_PENDING)
            {
                // Drift-Cast guidance aim point (#435, BLE cmd 28): 20-byte
                // geodetic GuidancePointData.  Accept/reject policy is the
                // pure guidancePointRc() gate (host-tested, GuidancePointGate.h);
                // the result rides back to the app in the OUT_STATUS_QUERY
                // echo (tgt_* fields) every poll.  RAM only — never NVS.
                delay_ms(1);
                uint8_t cfg_payload[sizeof(GuidancePointData)];
                size_t  cfg_len = 0;
                if (readConfigFrame(GUIDANCE_POINT_MSG, sizeof(GuidancePointData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(GuidancePointData))
                {
                    GuidancePointData gp;
                    memcpy(&gp, cfg_payload, sizeof(gp));
                    // Convert at receipt (running pad average; re-converted at
                    // the launch freeze in enterInflight()).  With no reference
                    // the gate rejects NOREF before ever consulting r.
                    float e = 0.0f, n = 0.0f;
                    const bool converted = guidanceLlaToEnu(gp.lat_deg, gp.lon_deg, e, n);
                    const float r = converted ? sqrtf(e * e + n * n) : 0.0f;
                    const bool ready_or_prelaunch =
                        (rocket_state == READY || rocket_state == PRELAUNCH);
                    // State gate note: the FC skips I2C polls during INFLIGHT,
                    // so a cmd 28 queued just before launch is delivered after
                    // landing — LANDED (+ post_flight_lockout) refuses it here.
                    const uint8_t rc = guidancePointRc(
                        ready_or_prelaunch, post_flight_lockout, pn_guidance_law,
                        have_ref_pos, gp.lat_deg, gp.lon_deg, gp.alt_m, r,
                        config::GUIDANCE_MAX_AIM_RADIUS_M);
                    // Echo transition + "apply iff ACCEPTED" are the pure
                    // host-tested guidancePointEchoApply() (GuidancePointGate.h):
                    // seq bumps on EVERY processed cmd 28, a reject leaves the
                    // display fields (still-active previous target) unchanged,
                    // and the flight target updates ONLY on its true return.
                    if (guidancePointEchoApply(out_status_query_data, rc,
                                               gp.lat_deg, gp.lon_deg, gp.alt_m))
                    {
                        guid_point_valid   = true;
                        guid_point_lat_deg = gp.lat_deg;
                        guid_point_lon_deg = gp.lon_deg;
                        guid_point_alt_m   = gp.alt_m;
                        // RAM only — deliberately NO prefs.put of gte/gtn/gta
                        // (a stale wind-compensated target self-applying on the
                        // next boot is the hazard).  pn_target_mode/pn_target_alt
                        // untouched: alt is stored + echoed only.  The profile
                        // shadows (pn_profile_target_*) are NOT touched either —
                        // they keep the cmd-65/NVS-owned point for the revert
                        // paths.
                        pn_target_e = e;
                        pn_target_n = n;
                        // Law-first ordering + belt-and-braces radius gate both
                        // re-run inside; idempotent.
                        applyGuidanceConfig();
                    }
                    ESP_LOGI(TAG, "[GUID PT] RX lat=%.7f lon=%.7f alt=%.1f -> "
                                  "E=%.2f N=%.2f r=%.2f (ref n=%lu frozen=%d) "
                                  "rc=%u seq=%u",
                             gp.lat_deg, gp.lon_deg, (double)gp.alt_m,
                             (double)e, (double)n, (double)r,
                             (unsigned long)ref_pos_count, (int)ref_pos_frozen,
                             (unsigned)rc,
                             (unsigned)out_status_query_data.tgt_seq);
                }
                else
                {
                    // Mirror the #534 skew diagnostic: echo the lengths so a
                    // short frame is diagnosable from the log.
                    ESP_LOGW(TAG, "[GUID PT] readConfigFrame FAILED — got %u bytes, "
                                  "need %u. Guidance point NOT applied.",
                                  (unsigned)cfg_len, (unsigned)sizeof(GuidancePointData));
                }
            }
            else if (out_pending_command == FIN_CONFIG_PENDING)
            {
                delay_ms(1);
                uint8_t cfg_payload[sizeof(FinConfigData)];
                size_t  cfg_len = 0;
                if (readConfigFrame(FIN_CONFIG_MSG, sizeof(FinConfigData),
                                    cfg_payload, sizeof(cfg_payload), cfg_len)
                    && cfg_len >= sizeof(FinConfigData))
                {
                    FinConfigData f;
                    memcpy(&f, cfg_payload, sizeof(f));
                    // Reject garbage azimuths (range check also rejects NaN/Inf).
                    bool ok = true;
                    for (int i = 0; i < 4; ++i)
                        if (!(f.azimuth_deg[i] >= -360.0f && f.azimuth_deg[i] <= 360.0f)) ok = false;
                    if (ok) {
                        // Copy out of the packed FinConfigData into an aligned
                        // local before passing — &f.azimuth_deg[0] is a
                        // potentially-unaligned pointer (-Waddress-of-packed-member).
                        float az[4];
                        memcpy(az, f.azimuth_deg, sizeof(az));
                        control_mixer.setFinLayout(az, f.reverse_mask,
                                                   f.roll_reverse_mask);
                        // Roll-only control mixes in servo_control, not in
                        // control_mixer — keep both in step (see the boot
                        // restore for why).
                        servo_control.setRollReverseMask(f.roll_reverse_mask);
                        ESP_LOGI(TAG, "[FIN CFG] az=[%.0f %.0f %.0f %.0f] rev=0x%X rollrev=0x%X",
                                      (double)f.azimuth_deg[0], (double)f.azimuth_deg[1],
                                      (double)f.azimuth_deg[2], (double)f.azimuth_deg[3],
                                      (unsigned)f.reverse_mask, (unsigned)f.roll_reverse_mask);
                        prefs.begin("servo", false);
                        prefs.putFloat("fa0", f.azimuth_deg[0]);
                        prefs.putFloat("fa1", f.azimuth_deg[1]);
                        prefs.putFloat("fa2", f.azimuth_deg[2]);
                        prefs.putFloat("fa3", f.azimuth_deg[3]);
                        prefs.putUChar("frv",  f.reverse_mask);
                        prefs.putUChar("frrv", f.roll_reverse_mask);
                        prefs.end();
                        ESP_LOGI(TAG, "[FIN CFG] Saved to NVS");
                    } else {
                        ESP_LOGW(TAG, "[FIN CFG] rejected out-of-range azimuth");
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "[FIN CFG] readConfigFrame failed");
                }
            }
            else if (out_pending_command == SERVO_REPLAY_PENDING)
            {
                if (isCommandLockoutState(rocket_state)) {
                    ESP_LOGW(TAG, "[SERVO REPLAY] Rejected — state=%u (no test commands while INFLIGHT or in MAG_CALIBRATION)",
                             (unsigned)rocket_state);
                } else {
                    delay_ms(1);
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
            // NOTE: do NOT clear out_pending_command here — it mirrors the OC's
            // reported command and is cleared by the next poll when the OC
            // reports 0.  Clearing it here would defeat the dedup (see above).
        }

        // Release exclusive bus access now that the query exchange and any
        // follow-up config reads (readConfigFrame) are complete.
        if (i2c_bus_mutex != nullptr && xSemaphoreGetMutexHolder(i2c_bus_mutex) == xTaskGetCurrentTaskHandle())
        {
            xSemaphoreGive(i2c_bus_mutex);
        }

        // ==========================================================================
        // SECTION: Kinematic checks and sensor health
        // ==========================================================================
        // Kinematic checks.  Issue #216 — skip during MAG_CALIBRATION: the
        // user is physically tumbling the rocket on the bench, so a vigorous
        // shake would otherwise spike `accel_norm` past the launch threshold
        // and latch `kinematics.launch_flag = true`.  That flag survives the
        // exit transition back to READY and would re-trigger PRELAUNCH ->
        // INFLIGHT on the very next PRELAUNCH tick.  We also clear bmp_new
        // and gps_new here to keep them from accumulating during cal — same
        // semantics as if the evaluator had consumed them.
        if (rocket_state != MAG_CALIBRATION)
        {
            // #257: per-sensor health for the adaptive apogee quorum + Layer-2
            // backstop.  baro_healthy = last sample finite and in a plausible
            // range (the range test also rejects NaN/Inf, which compare false)
            // and not stale — have_bmp_si latches once (#259) so it is not a
            // freshness signal by itself.  ekf_healthy = filter initialized and
            // reporting no recent divergence (TR_GpsInsEKF::isHealthy()).
            constexpr uint32_t BARO_STALE_TIMEOUT_US = 500000u;  // 0.5 s -> dead
            const bool baro_healthy =
                have_bmp_si &&
                bmp_latest_si.pressure > 25000.0f &&   // BMP585 valid 30-125 kPa;
                bmp_latest_si.pressure < 125000.0f &&  // 25 kPa floor = margin below spec
                (uint32_t)(time_us() - bmp_latest_si.time_us) < BARO_STALE_TIMEOUT_US;
            const bool ekf_healthy = ekf_initialized && ekf.isHealthy();

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
                                       mach_locked_out,
                                       (float)gnss_latest_si.vel_u,
                                       ekf_healthy,
                                       baro_healthy);

            // --- Recovery deployment detection ---
            // Stepped from the same block, once per loop pass, on the same
            // inputs — but with the RAW pressure altitude rather than the
            // filtered estimate: the rate gate inside kinematicChecks exists
            // to swallow ejection spikes, which are exactly the signal here.
            // Runs only INFLIGHT: on the ground the descent-collapse path
            // would read a stationary airframe as a canopy, and post-landing
            // the latch would be meaningless.
            if (rocket_state == INFLIGHT && !kinematics.alt_landed_flag)
            {
                static constexpr tr::DeploymentConfig kDeployCfg = {
                    config::DEPLOY_SHOCK_MS2,
                    config::DEPLOY_SHOCK_COUNT,
                    config::DEPLOY_BARO_STEP_M,
                    config::DEPLOY_COINCIDENCE_MS,
                    config::DEPLOY_BALLISTIC_MPS,
                    config::DEPLOY_CANOPY_MPS,
                    config::DEPLOY_CANOPY_COUNT,
                    config::DEPLOY_LAUNCH_LOCKOUT_MS,
                };
                const uint32_t t_since_launch = now_ms - launch_time_millis;
                if (tr::deploymentDetectStep(deployment_state, kDeployCfg,
                                             t_since_launch, accel_norm,
                                             pressure_altitude_m, bmp_new_for_kf,
                                             kinematics.d_alt_est_,
                                             burnout_detected))
                {
                    deployment_detected = true;
                    deployment_time_ms  = now_ms;
                    ESP_LOGI(TAG, "[DEPLOY] detected T+%.2f s (reason %s%s), alt %.0f m",
                             t_since_launch / 1000.0f,
                             (deployment_state.reason & tr::kDeployReasonShockBaro)
                                 ? "shock+baro " : "",
                             (deployment_state.reason & tr::kDeployReasonDescentCollapse)
                                 ? "descent-collapse" : "",
                             (double)kinematics.alt_est);
                    // Dynamic logging rate: nothing fast is left to capture
                    // under canopy, so step the ODR down. A no-op for a fixed
                    // rate setting.
                    if (imuRateIsDynamic(imu_rate_setting))
                    {
                        applyImuRateForFlightPhase();
                        ESP_LOGI(TAG, "[DEPLOY] IMU logging rate -> %u Hz (dynamic)",
                                 (unsigned)IMU_RATE_DYNAMIC_POST_HZ);
                    }
                }
            }
        }
        bmp_new_for_kf = false;
        gps_new_for_kc = false;

        pressure_alt_m = kinematics.alt_est;
        pressure_alt_rate_mps = kinematics.d_alt_est_;
        max_alt_m = kinematics.max_altitude;
        max_speed_mps = kinematics.max_speed;

        // #452: periodic pre-flight settings snapshot.  Command-started bench
        // recordings never enter INFLIGHT, where the per-flight emissions
        // (#165/#418) live, so their logs contained no FlightSettingsData and
        // the exported .json had no fw sha / settings provenance.  The FC has
        // no signal for when the OC starts a logging session, so emit a copy
        // every 5 s outside INFLIGHT instead: any session picks one up within
        // seconds, idle copies churn out of the pre-launch ring harmlessly
        // (208 B per 5 s against an 86 KB/s link), and the in-flight schedule
        // is untouched.  Runs at loop level (not in the state machine) so
        // recordings made during ground/servo test modes get provenance too.
        {
            static uint32_t last_preflight_settings_ms = 0;
            if (rocket_state != INFLIGHT &&
                now_ms - last_preflight_settings_ms >= 5000U)
            {
                last_preflight_settings_ms = now_ms;
                sendFlightSettings();
            }
        }

        // ==========================================================================
        // SECTION: Test modes (ground, servo, replay)
        // ==========================================================================
        // #363 SAFETY: the state machine (and servicePyroChannels) live in the
        // `else` of the test-mode chain below, so a ground/servo/replay test
        // left active at launch would suppress PRELAUNCH->INFLIGHT and pyro
        // servicing for the ENTIRE flight -> no drogue/main, ballistic return.
        // kinematicChecks() still latches launch_flag while a test runs, so if
        // launch is detected with any test mode active, force the test off (and
        // stow) here as a failsafe so the flight logic takes over. A bench false
        // positive merely exits the test into READY/PRELAUNCH, which is safe.
        if (kinematics.launch_flag &&
            (ground_test_active || servo_test_active || servo_replay_active))
        {
            ESP_LOGW(TAG, "[SAFETY] Launch detected with a test mode active -- "
                          "clearing test mode so flight logic runs (#363)");
            ground_test_active  = false;
            servo_test_active   = false;
            servo_replay_active = false;
            if (servo_enabled) servo_control.stowControl();
        }

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

                    // First column (r00/r10/r20 — body-forward axis) is unused
                    // here: a_cmd_ned has no down component, so only the right
                    // and down body projections below are needed.
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

                // 4-fin mix via the configured fin layout (servo→azimuth + reverse).
                // #382: use the runtime clamp (app-tunable via guidance config,
                // same as the in-flight path) — the compile-time constant made
                // a bench ground test not match an app-tuned flight clamp.
                float max_fin = pn_max_fin_deg;
                float deflections[4];
                control_mixer.mixToFins(roll_fin_cmd, pitch_fin, yaw_fin, max_fin, deflections);
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
                // EKF not yet initialized — hold neutral and log why.  #270:
                // centre the fins (no PID) and reset the integral, so nothing
                // leaks into the "neutral" output or into control once it starts.
                servo_control.stowControl();
                servo_control.resetPID();
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
        {

        // ==========================================================================
        // SECTION: Flight state machine
        // ==========================================================================
        // #317: once a flight has landed, the computer is terminal until a
        // hardware reboot. Force LANDED so any transition a command or a
        // ground-handling re-trigger of launch-detect tries to make is overridden
        // (no re-arm, no second flight).
        if (post_flight_lockout) rocket_state = LANDED;
        switch (rocket_state)
        {
            case INITIALIZATION:
            {
                if ((now_ms > 1000U) && have_ism6_si && have_bmp_si)
                {
                    rocket_state = READY;
                    if (!ready_chirp_played)
                    {
                        startBootReadyChirp(now_ms);
                        ready_chirp_played = true;
                    }
                    ESP_LOGI(TAG, "[STATE] INITIALIZATION -> READY");
                }
                break;
            }
            case READY:
            {
                // Advance an in-progress anti-backlash neutral settle (#407)
                // started by boot-settle or a pad trim; a no-op otherwise.  Runs
                // before the relax gate so it completes within the wake window.
                if (servo_enabled) servo_control.serviceNeutralSettle(now_ms);

                // Relax the servos on the pad: cut the pulse train so the
                // digital servos stop drawing ~150 mA of holding current while
                // we wait for GNSS/launch.  Re-asserted each tick (idempotent);
                // PWM resumes automatically when INFLIGHT first commands a pulse.
                // Held off during the pad-trim wake window (#345 follow-up) so a
                // just-applied trim stays driven long enough to see.
                if (servo_enabled && config::SERVO_RELAX_ON_PAD &&
                    (int32_t)(now_ms - servo_pad_wake_until_ms) >= 0)
                    servo_control.idle();

                const bool gnss_ready = gnss_started &&
                                        (now_ms > valid_gnss_start_millis + 3000U) &&
                                        have_gnss_si &&
                                        (gnss_latest_si.num_sats >= 4U);
                if (out_ready && gnss_ready)
                {
                    rocket_state = PRELAUNCH;
                    prelaunch_time_millis = now_ms;
                    // Camera is manually controlled via app — no auto-start on prelaunch
                    // Verify pyro continuity on the pad. (#382 comment fix:
                    // this is a synchronous single read of the VPP-fed CONT
                    // dividers (#264) — no arm pulse, no multi-tick sequence.)
                    pyroPrelaunchContTest(now_ms);
                    ESP_LOGI(TAG, "[STATE] READY -> PRELAUNCH");
                }
                else if (kinematics.launch_flag)
                {
                    // #382: launch detected while the OC/GNSS gates were never
                    // met (dead OC, <4 sats, boost before lock). Pre-fallback
                    // the FC sat in READY for the whole flight: no deployment,
                    // no control, no LANDED, no snapshots — only OC logging
                    // survived (NSF_LAUNCH latches from any state). Promote
                    // straight to INFLIGHT through the same entry path;
                    // degraded modes are already handled there (guidance
                    // auto-off without EKF init, ref-pos freeze skipped
                    // without a fix, ground pressure frozen from the pad).
                    ESP_LOGW(TAG, "[STATE] LAUNCH FROM READY — gates unmet "
                                  "(out_ready=%d gnss_ready=%d); promoting to "
                                  "INFLIGHT in degraded mode",
                             (int)out_ready, (int)gnss_ready);
                    enterInflight(now_ms, "READY");
                }
                break;
            }
            case PRELAUNCH:
            {
                // Advance an in-progress anti-backlash neutral settle (#407),
                // e.g. a trim applied while sitting in PRELAUNCH; no-op otherwise.
                if (servo_enabled) servo_control.serviceNeutralSettle(now_ms);

                // Stay relaxed through the pad wait; the INFLIGHT control path
                // re-drives (and so re-energises) the servos the instant
                // launch_flag flips below.  Held off during the pad-trim wake
                // window (#345 follow-up) so a just-applied trim stays visible.
                if (servo_enabled && config::SERVO_RELAX_ON_PAD &&
                    (int32_t)(now_ms - servo_pad_wake_until_ms) >= 0)
                    servo_control.idle();

                if (kinematics.launch_flag)
                {
                    enterInflight(now_ms, "PRELAUNCH");
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

                    // Emit the flight settings snapshot (#165) on the first
                    // few ticks after launch.  Resent for redundancy against a
                    // dropped frame; values are stable in flight so all copies
                    // are identical and the app reads the first one.
                    if (settings_emit_count < 3) {
                        sendFlightSettings();
                        settings_emit_count++;
                    }
                    // One more copy well clear of the logging-activation edge:
                    // the first three ride launch+0/100/200 ms, exactly the
                    // window where the OC's ring drain is busiest and where the
                    // pre-#418 races ate all three on the 7/05 V2 flight (its
                    // .json has no settings block).  By launch+5 s the ring is
                    // long drained, so this copy survives anything short of a
                    // dead link.
                    else if (settings_emit_count == 3 &&
                             now_ms - launch_time_millis >= 5000U) {
                        sendFlightSettings();
                        settings_emit_count++;
                    }
                }

                // Servo settling after reboot recovery — hold neutral until EKF stabilizes
                if (reboot_recovery && now_ms >= servo_settle_end_ms) {
                    reboot_recovery = false;  // keep PSF_REBOOT_RECOVERY flag in telemetry
                    ESP_LOGI(TAG, "[RECOVERY] Servo settle complete, control resumed");
                }

                // --- Burnout detection — runs independently of servo/roll
                // control (#256) ---
                // Apogee detection (TR_KinematicChecks) is gated on
                // burnout_detected, and drogue/main deployment is gated on
                // apogee, so this MUST NOT live inside `if (servo_enabled)` —
                // otherwise recovery silently never fires when roll control is
                // disabled.  Hoisted above the servo block (and ahead of the
                // roll-delay branch, so a non-zero ROLL_CONTROL_DELAY_MS can't
                // suppress it either).  Body-X accel (forward axis in FRD) goes
                // negative after motor burnout; the 200 ms lockout + N-sample
                // hysteresis (#197) reject boost vibration.  Logic is shared
                // with the host unit test via BurnoutDetector.h.
                const uint32_t t_since_launch_ms = now_ms - launch_time_millis;

                // #259: treat the IMU as unavailable when its last sample is
                // stale.  have_ism6_si latches on first read and never clears,
                // so without this a mid-flight IMU dropout would feed a frozen
                // accel into burnout detection (miss / false burnout) and frozen
                // gyro/velocity into roll control.  (Baro staleness is already
                // handled by baro_healthy / the bmp_new_for_kf KF gate.)
                constexpr uint32_t IMU_STALE_TIMEOUT_US = 100000u;  // 0.1 s (~1 kHz IMU)
                const bool ism6_fresh = have_ism6_si &&
                    (uint32_t)(time_us() - ism6_latest_si.time_us) < IMU_STALE_TIMEOUT_US;

                if (ism6_fresh &&
                    tr::burnoutDetectStep(burnout_detected, burnout_neg_count,
                                          ism6_latest_si.low_g_acc_x,
                                          t_since_launch_ms,
                                          config::BURNOUT_NEG_HYSTERESIS))
                {
                    burnout_time_ms = now_ms;
                    ESP_LOGI(TAG, "[GUID] Burnout detected at T+%lu ms",
                                  (unsigned long)t_since_launch_ms);
                }

                if (servo_enabled)
                {
                    // Delay roll control activation after launch if configured
                    if (reboot_recovery || t_since_launch_ms < roll_delay_ms)
                    {
                        // Hold fins neutral until delay/settle elapses.  #270:
                        // centre the fins (no PID) and reset the integral — the
                        // pre-roll_delay window must not accumulate an offset that
                        // would jerk the fins when roll control activates.
                        servo_control.stowControl();
                        servo_control.resetPID();
                        // #568: defensive — can't be true here on a normal flight
                        // (guidance only sets it after roll_delay elapses), but a
                        // stowed vehicle must never report guidance active.
                        guidance_active = false;
                    }
                    else if (ism6_fresh)   // #259: stale IMU -> neutral-hold branch below
                    {
                        float speed = sqrtf(imu_vel[0]*imu_vel[0] +
                                            imu_vel[1]*imu_vel[1] +
                                            imu_vel[2]*imu_vel[2]);

                        // #265: EKF health gates the attitude/velocity-based
                        // control modes below (coast guidance, angle mode, and
                        // the speed-scaled gain schedule).  When the filter is
                        // unhealthy — non-finite or freshly diverged per
                        // GpsInsEKF::isHealthy() — they are all skipped and
                        // control falls through to PURE GYRO rate-null (no EKF
                        // dependency), instead of a stabilizeP() attitude jump
                        // slamming the fins.  Logged on transition so a
                        // flight/bench run shows if the gate ever fired.
                        const bool ekf_ctrl_healthy = ekf.isHealthy();
                        static bool ekf_ctrl_healthy_prev = true;
                        if (ekf_ctrl_healthy != ekf_ctrl_healthy_prev) {
                            ESP_LOGW(TAG, "[CTRL] EKF health %s — roll control %s",
                                     ekf_ctrl_healthy ? "RECOVERED" : "LOST",
                                     ekf_ctrl_healthy ? "resumed attitude/guidance"
                                                      : "fell back to gyro rate-null");
                            ekf_ctrl_healthy_prev = ekf_ctrl_healthy;
                        }

                        // --- Guided mode (PN guidance) ---
                        // Shares roll-control initiation timing: this whole branch is
                        // gated by t_since_launch_ms >= roll_delay_ms above, so guidance
                        // engages WITH roll control (at the roll-control delay after
                        // launch), NOT after burnout.  Set roll_delay_ms to ~burn time to
                        // keep guidance post-boost.  Still needs a healthy EKF + airspeed
                        // (pn_min_speed) and stops at closest point of approach (CPA).
                        // (pn_coast_delay_ms / burnout are no longer gates here.)
                        // Tilt-limit safety gate: off-vertical angle of the nose
                        // (body-X) from EKF attitude. Tilt is well-observed even
                        // when heading is degraded (it's the gravity-aligned part
                        // of the attitude), so the FC can trust its own estimate.
                        // Latched: once tripped, guidance stays off for the flight.
                        {
                            float gq[4];
                            ekf.getQuaternion(gq);
                            // cos(tilt) = up-component of the nose in NED = -D row.
                            float nose_up = -2.0f * (gq[1]*gq[3] - gq[0]*gq[2]);
                            if (nose_up >  1.0f) nose_up =  1.0f;
                            if (nose_up < -1.0f) nose_up = -1.0f;
                            float tilt_deg = acosf(nose_up) * (180.0f / (float)M_PI);
                            float tilt_limit = burnout_detected
                                ? config::GUIDANCE_TILT_LIMIT_COAST_DEG
                                : config::GUIDANCE_TILT_LIMIT_BOOST_DEG;
                            if (guidance_enabled && !guidance_tilt_inhibited &&
                                tilt_deg > tilt_limit) {
                                guidance_tilt_inhibited = true;
                                ESP_LOGW(TAG, "[GUID] tilt %.1f deg > %.0f limit (%s) "
                                         "— guidance LATCHED off, roll-only for flight",
                                         (double)tilt_deg, (double)tilt_limit,
                                         burnout_detected ? "coast" : "boost");
                            }
                        }

                        const bool coast_guidance_active =
                            guidance_enabled &&
                            ekf_initialized &&
                            have_ref_pos &&       // #557: no GNSS-anchored position frame → no steering
                            ekf_ctrl_healthy &&   // #265: don't fly guidance on a diverged EKF
                            !guidance_tilt_inhibited &&  // tilt-limit safety latch
                            (speed > pn_min_speed) &&
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

                            float accel_to_fin_deg = pn_accel_to_fin;  // app/NVS-tunable (was inline 4.0)
                            float pitch_fin = accel_to_fin_deg * pitch_accel;
                            float yaw_fin   = accel_to_fin_deg * yaw_accel;

                            // Stash for the ~10 Hz GuidanceTelemData log frame.
                            pn_last_pitch_fin_deg = pitch_fin;
                            pn_last_yaw_fin_deg   = yaw_fin;

                            // Roll control: standalone PID
                            float roll_fin_cmd = roll_rate_pid_standalone.computePID(
                                config::ROLL_RATE_SET_POINT, -roll_rate_dps);

                            // 4-fin mix via the configured fin layout (servo→azimuth + reverse)
                            float max_fin = pn_max_fin_deg;
                            float deflections[4];
                            control_mixer.mixToFins(roll_fin_cmd, pitch_fin, yaw_fin, max_fin, deflections);
                            servo_control.setServoAngles(deflections);
                            guidance_active = true;
                        }
                        else
                        {
                            // --- Roll-only mode (powered flight or guidance disabled) ---
                            // #568: guidance→roll-only handback. While guiding,
                            // roll was nulled by roll_rate_pid_standalone +
                            // setServoAngles, so servo_control's INTERNAL PID
                            // never ran and its integrator froze at its
                            // pre-guidance value (#268 syncs gains, not state).
                            // Reset it once on the falling edge — guidance_active
                            // still holds last tick's value here — so the
                            // resuming controller doesn't apply the stale
                            // integral as a step on the fins.
                            if (guidance_active)
                            {
                                servo_control.resetPID();
                            }
                            guidance_active = false;

                            // Decide per-segment whether to track angle or null rate.
                            // Profile-mode lookup is only meaningful when angle
                            // control is enabled and the EKF is initialized.
                            const float t_flight = (float)(now_ms - launch_time_millis) / 1000.0f;
                            const bool angle_mode_active =
                                use_angle_control && ekf_initialized && ekf_ctrl_healthy; // #265
                            RollProfileQuery seg = {0.0f, ROLL_SEG_NULL_RATE};
                            if (angle_mode_active)
                            {
                                seg = roll_profile_query(t_flight);
                            }

                            if (angle_mode_active && seg.mode == ROLL_SEG_ANGLE)
                            {
                                // Quaternion-based roll extraction (gimbal-lock-free).
                                float quat[4];
                                ekf.getQuaternion(quat);
                                float qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
                                float z_north = 2.0f * (qx*qz + qw*qy);
                                float z_east  = 2.0f * (qy*qz - qw*qx);
                                float actual_roll_deg = -atan2f(z_east, z_north) * (180.0f / (float)M_PI);

                                servo_control.controlAngle(seg.angle_deg,
                                                           actual_roll_deg,
                                                           -roll_rate_dps,
                                                           speed,
                                                           kp_angle_outer,
                                                           kp_angle_rate_cap_dps);
                            }
                            else if (gain_sched_enabled && ekf_ctrl_healthy)
                            {
                                // Rate-null path (used when angle control is off,
                                // OR when angle control is on but the current
                                // profile segment selects ROLL_SEG_NULL_RATE).
                                // Gated on EKF health (#265): the schedule scales
                                // on EKF speed, so an unhealthy filter drops to
                                // the pure-gyro rate-null below.
                                servo_control.controlWithGainSchedule(-roll_rate_dps, speed);
                            }
                            else
                            {
                                // Pure gyro rate-null — raw gyro only, no EKF
                                // dependency; the safe fallback when the EKF is
                                // unhealthy (#265) or the gain schedule is off.
                                servo_control.control(-roll_rate_dps);
                            }
                        }
                    }
                    else
                    {
                        // IMU data lost mid-flight — hold fins at neutral.  #270:
                        // centre the fins (no PID) and reset the integral, so a
                        // stale pre-loss integral can't deflect them and control
                        // resumes clean if the IMU recovers.
                        servo_control.stowControl();
                        servo_control.resetPID();
                        // #568: without this, guidance_active latched true across
                        // an IMU dropout during guided coast — NSF_GUIDANCE kept
                        // asserting and GuidanceTelemData kept streaming stale
                        // accel commands while the fins were actually stowed.
                        // Telemetry/log fidelity only; mirrors the roll-only path.
                        guidance_active = false;
                    }
                }
                const bool landing_conditions =
                    kinematics.alt_landed_flag && (fabsf(roll_rate_dps) < 30.0f);
                if (landing_conditions)
                {
                    if (!landed_candidate_active)   // #297: bool, not a now_ms==0 sentinel
                    {
                        landed_candidate_start_millis = now_ms;
                        landed_candidate_active = true;
                    }
                    if (now_ms - landed_candidate_start_millis > 2000U)
                    {
                        rocket_state = LANDED;
                        ESP_LOGI(TAG, "[STATE] INFLIGHT -> LANDED");
                    }
                }
                else if (landed_candidate_active &&
                         (now_ms - landed_candidate_start_millis > 2500U))
                {
                    // Only reset debounce timer if conditions have been false
                    // for >500ms beyond the 2s window — prevents single-frame
                    // noise from restarting the entire landing countdown.
                    landed_candidate_active = false;
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
                    post_flight_lockout = true;  // #317: LANDED is terminal until reboot
                    pyroSafeAll();
                    clearFlightSnapshot();  // prevent stale recovery on next boot
                    if (servo_enabled)
                    {
                        // Relax post-flight to stop the idle holding current
                        // (the MOSFET-less interim of the pad relax); otherwise
                        // hold centre as before.
                        if (config::SERVO_RELAX_ON_PAD)
                            servo_control.idle();
                        else
                            servo_control.stowControl();
                    }
                    // Reset guidance state
                    guidance.reset();
                    control_mixer.reset();
                    roll_rate_pid_standalone.reset();
                    guidance_active = false;
                    reboot_recovery = false;
                    reboot_recovery_telem = false;
                    cameraStop(now_ms, (uint32_t)config::CAMERA_STOP_DELAY_MS);
                }
                // Retry END_FLIGHT until enqueue succeeds.  The one-shot
                // cleanup above is latched, but i2s_tx_queue can transiently
                // saturate around LANDED entry and a single failed enqueue
                // would otherwise lose END_FLIGHT forever (#141).
                if (!end_flight_sent)
                {
                    if (enqueueI2STx(END_FLIGHT, nullptr, 0))
                    {
                        end_flight_sent = true;
                    }
                }
                break;
            }
            case MAG_CALIBRATION:
            {
                // No state-machine transitions out — driven entirely by
                // BLE commands (MAG_CAL_ABORT/ACCEPT/RETRY).  The user is
                // physically tumbling the rocket on the bench, so we
                // explicitly do NOT promote to PRELAUNCH on motion or any
                // other auto-detect.  Issue #96 / #216.
                //
                // #216 — additional inhibitions enforced elsewhere in the
                // loop while we're in this state:
                //   * `kinematicChecks(...)` is skipped above, so
                //     `kinematics.launch_flag` cannot latch from the
                //     tumble.  MAG_CAL_START also calls kinematics.reset().
                //   * `ekf.init(...)` is gated against MAG_CALIBRATION so
                //     a tumble can't violate the stationary-at-init
                //     assumption.
                //   * `servicePyroChannels()` only runs from `case
                //     INFLIGHT:` below, so automatic pyro firing is
                //     impossible while we're here.
                //   * BLE test commands (GROUND_TEST_START, PYRO_CONT_TEST,
                //     PYRO_FIRE_TEST, SERVO_TEST, SERVO_REPLAY_PENDING)
                //     all gate on isCommandLockoutState() which returns
                //     true for MAG_CALIBRATION.
                //   * The servo PWM is stowed at MAG_CAL_START entry and
                //     no servo-control loop runs from this case body.
                break;
            }
            default:
            {
                break;
            }
        }
        }

        // ==========================================================================
        // SECTION: Simulator re-arm and diagnostics
        // ==========================================================================
        // ---- Re-arm on a NEW sim run, NOT when a sim ends (#317) ----
        // A sim flight that reaches LANDED must STAY landed — terminal, exactly
        // like real hardware — so the sim faithfully reproduces and can validate
        // the post-flight lockout. Re-arm on the RISING edge of sim-active (the
        // deliberate start of a new run, the sim's equivalent of a reboot).  We
        // do NOT reset on the falling edge: the sim drops to SIM_IDLE on its
        // NATURAL completion too (SIM_LANDED auto-stop), and there we want the FC
        // to hold LANDED so the lockout stays validated.  An explicit Stop is
        // handled in the SIM_STOP_CMD handler instead (#393), which is the only
        // way to distinguish a user abort from a flown-out sim.
        {
            const bool curr_sim_active = sensor_collector.isSimActive();
            if (!prev_sim_active && curr_sim_active)
            {
                resetFlightStateForSim("start");
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

        // ==========================================================================
        // SECTION: Telemetry packing (NonSensorData)
        // ==========================================================================
        // Publish non-sensor summary (SI -> packed) for downstream logging/telem.
        non_sensor_data.time_us = logic_now_us;
        // #529: achieved-EKF-cadence witness; stays 0 until the EKF initializes.
        non_sensor_data.ekf_ticks = (uint16_t)ekf_tick_counter;
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
        // Apogee detector outputs + master vote (#142/#143) plus relocated
        // reboot-recovery and guidance-enabled bits (formerly in pyro_status,
        // moved when pyro_status was reclaimed for 4-channel state).
        non_sensor_data.apogee_flags = 0;
        if (kinematics.gps_apogee_flag)   non_sensor_data.apogee_flags |= NSF2_GPS_APOGEE;
        if (kinematics.pitch_apogee_flag) non_sensor_data.apogee_flags |= NSF2_PITCH_APOGEE;
        if (kinematics.apogee_flag)       non_sensor_data.apogee_flags |= NSF2_MASTER_APOGEE;
        if (reboot_recovery_telem)        non_sensor_data.apogee_flags |= NSF2_REBOOT_RECOVERY;
        if (guidance_enabled)             non_sensor_data.apogee_flags |= NSF2_GUIDANCE_ENABLED;
        if (orient_thrust_mismatch)       non_sensor_data.apogee_flags |= NSF2_ORIENT_THRUST_MISMATCH;
        if (deployment_detected)          non_sensor_data.apogee_flags |= NSF2_DEPLOYED;
        // #474: sticky witness for a stalled sensor loop.  Nonzero drops mean
        // loop_fc() blocked long enough (an I2C poll/config-read) to overflow the
        // ISM6 handoff queue and lose IMU samples — the previously-silent all-
        // stream logging hole.  Read the HW collector (owns the queue + counter,
        // reset once at loop start) so the deepened queue's residual, if any, is
        // recorded in the log rather than vanishing.
        if (sensor_collector_hw.getIsm6QueueDrops() > 0)
            non_sensor_data.apogee_flags |= NSF2_FC_IMU_DROP;
        // Snapshot pyro state under spinlock for telemetry. With per-fire
        // arming the global "armed" bit just mirrors the live PYRO_ARM pin.
        bool arm_now;
        bool cont_known[4], cont_state[4], fired[4];
        portENTER_CRITICAL(&pyro_spinlock);
        arm_now = pyro_arm_pin_state;
        for (int i = 0; i < 4; ++i) {
            cont_known[i] = pyro_ch[i].cont_known;
            cont_state[i] = pyro_ch[i].cont;
            fired[i]      = (pyro_ch[i].state == PyroChState::Done);
        }
        portEXIT_CRITICAL(&pyro_spinlock);
        if (arm_now) non_sensor_data.flags |= NSF_PYRO_ARMED;
        // #393: report sim-active so the app's Stop-sim control survives BLE
        // reconnects (its local latch dies with the recreated BLEDevice).
        if (sensor_collector.isSimActive()) non_sensor_data.flags |= NSF_SIM_ACTIVE;
        non_sensor_data.rocket_state = (uint8_t)rocket_state;
        // 4-channel pyro status byte: bit pairs (cont, fired) per channel.
        // CONT bits reflect the last known reading (set during a CONT test
        // or while ARM is asserted mid-fire). Channels never measured stay
        // 0 — iOS UI surfaces "unknown" until the prelaunch test runs.
        {
            uint8_t ps = 0;
            static constexpr uint8_t CONT_BITS[4]  = {
                PSF_CH1_CONT, PSF_CH2_CONT, PSF_CH3_CONT, PSF_CH4_CONT };
            static constexpr uint8_t FIRED_BITS[4] = {
                PSF_CH1_FIRED, PSF_CH2_FIRED, PSF_CH3_FIRED, PSF_CH4_FIRED };
            for (int i = 0; i < 4; ++i) {
                if (cont_known[i] && cont_state[i]) ps |= CONT_BITS[i];
                if (fired[i])                       ps |= FIRED_BITS[i];
            }
            non_sensor_data.pyro_status = ps;
        }

        // Sensor health scorecard (#303): pack 2-bit verdicts for the operator's
        // pre-launch go/no-go (surfaced on iOS).  FC fills all but battery, which
        // the OC ORs in from POWERData before the LoRa relay.
        {
            uint32_t sh = 0;
            const uint32_t now_us_h = time_us();

            // Baro — in BMP585 range + fresh (mirrors #257 baro_healthy).
            SensorHealthState baro_st = SH_BAD;
            if (have_bmp_si) {
                const bool fresh = (uint32_t)(now_us_h - bmp_latest_si.time_us) < 500000u;
                const bool inrange = bmp_latest_si.pressure > 25000.0f &&
                                     bmp_latest_si.pressure < 125000.0f;
                baro_st = inrange ? (fresh ? SH_OK : SH_DEGRADED) : SH_BAD;
            }
            sh = shSet(sh, SH_BARO_SHIFT, baro_st);

            // IMU — present + fresh (mirrors #259 ism6_fresh).
            SensorHealthState imu_st = SH_BAD;
            if (have_ism6_si) {
                const bool fresh = (uint32_t)(now_us_h - ism6_latest_si.time_us) < 100000u;
                imu_st = fresh ? SH_OK : SH_DEGRADED;
            }
            sh = shSet(sh, SH_IMU_SHIFT, imu_st);

            // EKF — the FILTER's own health, not the board-orientation auto-detect
            // (#238): that residual is a mounting-config signal that reads large off
            // the vertical pad and says nothing about the Kalman state.  BAD until
            // initialized; DEGRADED if diverging (isHealthy()'s NaN-repair cooldown)
            // or the covariance hasn't converged — stabilizeP() pins the diagonals
            // at P_MAX_* on divergence, which a finite isHealthy() can't see, so we
            // check the worst attitude + velocity variance directly.  OK = converged.
            SensorHealthState ekf_st = SH_BAD;
            if (ekf_initialized) {
                float covO[3], covV[3];
                ekf.getCovOrient(covO);
                ekf.getCovVel(covV);
                const float att_var = fmaxf(covO[0], fmaxf(covO[1], covO[2]));
                const float vel_var = fmaxf(covV[0], fmaxf(covV[1], covV[2]));
                const bool converged = att_var < config::EKF_ATT_VAR_OK &&
                                       vel_var < config::EKF_VEL_VAR_OK;
                ekf_st = (ekf.isHealthy() && converged) ? SH_OK : SH_DEGRADED;

                // #508: the gyro bias is FROZEN IN at launch — accel/mag updates
                // are gated off for the whole flight, so whatever it holds when we
                // leave the pad integrates into attitude at that rate for the
                // entire ascent (5 dps → 5°/s → ~50° by apogee on a 10 s climb),
                // and guidance keys off attitude.  Two independent no-go signals:
                //
                //   1. the estimate itself is out of spec / the bound had to bite
                //      (gyroBiasHealthy: |bias| ≤ 3 dps, sigma sane, 0 clips), or
                //   2. the observability gate is TRIPPING on the pad — meaning the
                //      filter is being fed accel/mag data inconsistent with its own
                //      state (bad mounting, a steel rail deflecting the mag, a stale
                //      hard-iron cal).  The bias may have survived intact, but the
                //      setup is wrong and you want to know BEFORE it gets frozen.
                //
                // Amber rather than red: it degrades attitude/guidance, it does not
                // by itself endanger recovery.
                static uint32_t last_gbias_trips   = 0;
                static uint32_t gbias_trip_seen_ms = 0;
                const uint32_t trips_now = ekf.gyroBiasGateTrips();
                if (trips_now != last_gbias_trips) {
                    last_gbias_trips   = trips_now;
                    gbias_trip_seen_ms = now_ms;
                }
                const bool gbias_tripping_recently =
                    gbias_trip_seen_ms != 0 && (now_ms - gbias_trip_seen_ms) < 5000U;

                if ((!ekf.gyroBiasHealthy() || gbias_tripping_recently) && ekf_st == SH_OK) {
                    ekf_st = SH_DEGRADED;
                }
            }
            sh = shSet(sh, SH_EKF_SHIFT, ekf_st);

            // Mag — present (cal-residual refinement is a follow-up; amber-only).
            sh = shSet(sh, SH_MAG_SHIFT,
                       (have_iis2mdc_si || have_mmc_si) ? SH_OK : SH_NA);

            // GNSS — OK needs a 3D fix + enough sats (+ horizontal accuracy,
            // but only when that gate is enabled: config::GNSS_MAX_HACC_M == 0
            // is a "disabled" sentinel, so comparing against it would make OK
            // unreachable).  DEGRADED = 3D fix but marginal; BAD = no 3D fix.
            SensorHealthState gnss_st = SH_BAD;
            if (have_gnss_si && gnss_latest_si.fix_mode >= 3U) {
                const bool hacc_ok = (config::GNSS_MAX_HACC_M <= 0.0f) ||
                                     (gnss_latest_si.horizontal_accuracy < config::GNSS_MAX_HACC_M);
                gnss_st = (gnss_latest_si.num_sats >= config::GNSS_MIN_SATS && hacc_ok)
                          ? SH_OK : SH_DEGRADED;
            }
            sh = shSet(sh, SH_GNSS_SHIFT, gnss_st);

            // Per-channel pyro (#303): only configured channels matter.  Carried
            // in sensor_health because LoRaData has no pyro_status, so on a
            // base-station relay this is the only pyro signal.
            for (int i = 0; i < 4; ++i) {
                SensorHealthState pst = SH_NA;              // not configured -> ignored
                if (pyroChEnabled(i)) {
                    pst = !cont_known[i] ? SH_DEGRADED      // configured, not yet tested
                        : (cont_state[i] ? SH_OK : SH_BAD); // continuity present / none
                }
                sh = shSet(sh, SH_PYRO_SHIFT[i], pst);

                // Measured continuity, NOT config-gated — the ground-test
                // answer, which the gated bits above cannot give for a channel
                // that isn't armed for this flight. Mirrors the pyro_status
                // CONT bit (also ungated) onto the LoRa path, where
                // pyro_status does not exist.
                sh = shSet(sh, SH_PYRO_MEAS_SHIFT[i],
                           !cont_known[i] ? SH_NA
                                          : (cont_state[i] ? SH_OK : SH_BAD));
            }

            // #557: GNSS-absent degraded-flight verdict (distinct from the fix
            // health at SH_GNSS above).  SH_BAD = the FC initialized the EKF on
            // the baro+IMU-only path because the module failed bring-up; guidance
            // is off and there is no absolute position.  Rides sensor_health to
            // the app on BOTH the direct-BLE and LoRa/BS-relay paths so the
            // degraded banner shows regardless of transport.
            sh = shSet(sh, SH_GNSS_ABSENT_SHIFT,
                       gnss_absent_flight ? SH_BAD : SH_NA);

            // Battery bits (12-13) left N/A — the OC fills them from POWERData.
            non_sensor_data.sensor_health = sh;
        }

        if ((logic_now_us - last_non_sensor_tx_time_us) >= non_sensor_tx_period_us)
        {
            last_non_sensor_tx_time_us = logic_now_us;
            memcpy(non_sensor_data_buffer, &non_sensor_data, SIZE_OF_NON_SENSOR_DATA);
            (void)enqueueI2STx(NON_SENSOR_MSG,
                               non_sensor_data_buffer,
                               SIZE_OF_NON_SENSOR_DATA);

            // Send guidance telemetry during guided coast, at
            // config::GUIDANCE_TELEM_RATE_HZ (500 = lockstep with NonSensor).
            if (guidance_active)
            {
                static uint32_t guid_telem_counter = 0;
                if (++guid_telem_counter >=
                    (config::NON_SENSOR_UPDATE_RATE / config::GUIDANCE_TELEM_RATE_HZ))
                {
                    guid_telem_counter = 0;
                    GuidanceTelemData gtd = {};
                    gtd.time_us            = logic_now_us;
                    gtd.accel_cmd_n_cmps2  = (int16_t)lroundf(guidance.getAccelNorthCmd() * 100.0f);
                    gtd.accel_cmd_e_cmps2  = (int16_t)lroundf(guidance.getAccelEastCmd() * 100.0f);
                    gtd.lateral_offset_cm  = (int16_t)lroundf(guidance.getLateralOffset() * 100.0f);
                    gtd.los_angle_cdeg     = (int16_t)lroundf(guidance.getLosAngleDeg() * 100.0f);
                    gtd.closing_vel_cmps   = (int16_t)lroundf(guidance.getClosingVelocity() * 100.0f);
                    gtd.pitch_fin_cmd_cdeg = (int16_t)lroundf(pn_last_pitch_fin_deg * 100.0f);
                    gtd.yaw_fin_cmd_cdeg   = (int16_t)lroundf(pn_last_yaw_fin_deg * 100.0f);
                    gtd.guid_flags = (guidance_active ? 1u : 0u) | (burnout_detected ? 2u : 0u);
                    uint8_t gtd_buf[sizeof(GuidanceTelemData)];
                    memcpy(gtd_buf, &gtd, sizeof(GuidanceTelemData));
                    (void)enqueueI2STx(GUIDANCE_TELEM_MSG, gtd_buf, sizeof(GuidanceTelemData));
                }
            }
        }
    }
    
    const uint32_t now_ms_for_sound = time_ms();
    serviceBootReadyChirp(now_ms_for_sound);
    serviceHeartbeatBeep(now_ms_for_sound);
    serviceBlueLedFlash(now_ms_for_sound);
    serviceGoProPulse(now_ms_for_sound);
    serviceCameraStart(now_ms_for_sound);
    serviceCameraStop(now_ms_for_sound);

    // ==========================================================================
    // SECTION: Periodic diagnostics
    // ==========================================================================
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
            ESP_LOGI(TAG, "[GAP DIAG] gaps>10ms=%lu worst=%lu us | gnss calls=%lu >1ms=%lu >5ms=%lu >10ms=%lu | imu_q_drops=%lu",
                          (unsigned long)pt.gap_count,
                          (unsigned long)pt.gap_worst_us,
                          (unsigned long)pt.gnss_calls,
                          (unsigned long)pt.gnss_over_1ms,
                          (unsigned long)pt.gnss_over_5ms,
                          (unsigned long)pt.gnss_over_10ms,
                          (unsigned long)pt.ism6_queue_drops);
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
            ESP_LOGI(TAG, "[SENSOR] reads/s: ism6=%lu bmp=%lu bmp_bad=%lu mmc=%lu iis=%lu gnss=%lu | bmp_p=%.0f ism6_gz=%.1f drdy_pin=%d",
                          (unsigned long)dbg_ism6_reads,
                          (unsigned long)dbg_bmp_reads,
                          (unsigned long)dbg_bmp_bad_reads,
                          (unsigned long)dbg_mmc_reads,
                          (unsigned long)dbg_iis2mdc_reads,
                          (unsigned long)dbg_gnss_reads,
                          have_bmp_si ? (double)bmp_latest_si.pressure : 0.0,
                          have_ism6_si ? (double)ism6_latest_si.gyro_z : 0.0,
                          gpio_get_level((gpio_num_t)config::ISM6HG256_INT));
            dbg_ism6_reads = 0;
            dbg_bmp_reads = 0;
            dbg_bmp_bad_reads = 0;
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
            // Skipped when the IIS2MDC won the boot-time probe: the MMC path
            // was never started, so every counter is a meaningless zero (and
            // reads as a dead sensor to anyone reviewing the log).
            if (!sensor_collector.isIIS2MDCActive())
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
                              config::MMC5983MA_INT >= 0
                                  ? gpio_get_level((gpio_num_t)config::MMC5983MA_INT)
                                  : -1);
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
            // #508: trips/clips make a laundered bias visible on the console — a
            // healthy pad shows 0/0. Non-zero trips mean accel/mag data
            // inconsistent with the filter's state is arriving; clips mean the
            // physical bound had to bite, which should never happen in the field.
            ESP_LOGI(TAG, "[EKF DIAG] mag[%s]=%.1fuT(%s) gyro_bias=[%.3f,%.3f,%.3f]dps "
                          "(sig=%.2f trips=%u clips=%u %s)",
                          mag_src,
                          (double)mag_uT,
                          mag_status,
                          (double)(gb[0] * 180.0f / (float)M_PI),
                          (double)(gb[1] * 180.0f / (float)M_PI),
                          (double)(gb[2] * 180.0f / (float)M_PI),
                          (double)ekf.gyroBiasSigmaMaxDps(),
                          (unsigned)ekf.gyroBiasGateTrips(),
                          (unsigned)ekf.gyroBiasClipCount(),
                          ekf.gyroBiasHealthy() ? "OK" : "NO-GO");
        }
    }

    // Give back remainder of time slice
    taskYIELD();
}

/* ── ESP-IDF entry point ─────────────────────────────────── */

// ==========================================================================
// SECTION: FreeRTOS entry point
// ==========================================================================
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
