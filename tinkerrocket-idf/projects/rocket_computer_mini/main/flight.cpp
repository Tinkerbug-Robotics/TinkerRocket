// ==========================================================================
// rocket-computer-mini — flight side.
//
// Port of projects/flight_computer/main/main.cpp (single-MCU adaptation).
// The FC's structure, names and safety comments are preserved wherever the
// logic is kept, so future FC fixes can be cross-applied by diff.  The
// transport layer changed: I2S/I2C to the out computer became mini_link
// (logFrame → NAND ring, telem cache → comms task, cmd_queue ← BLE/LoRa).
// Dropped whole: servos, roll/fin/guidance control, camera, piezo, OTA relay.
//
// Issue numbers in comments refer to the originals in the FC source.
// ==========================================================================
#include "config.h"
#include "flight.h"
#include "mini_link.h"

#include <TR_NVS.h>
#include <TR_Sensor_Collector.h>
#include <TR_Sensor_Collector_Sim.h>
#include <TR_Sensor_Data_Converter.h>
#include <TR_I2C_Interface.h>      // packMessage/unpackMessage statics only — no bus
#include <TR_Orientation.h>
#include <TR_GpsInsEKF.h>
#include <TR_GeoMag.h>
#include <TR_KinematicChecks.h>
#include <BurnoutDetector.h>       // shared burnout detector (#197/#256)
#include <DeploymentDetector.h>    // shared recovery-deployment detector
#include <TR_MagCalibrator.h>
#include <RocketComputerTypes.h>
#include <TR_FlightLog.h>          // snapshot tail-scan recovery (replaces OC MRAM)
#include <CRC32.h>

#include <esp_system.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <esp_rom_sys.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_private/esp_gpio_reserve.h>
#include <esp_private/gpio.h>      // gpio_func_sel
#include <rom/gpio.h>              // esp_rom_gpio_connect_out_signal
#include <soc/gpio_sig_map.h>      // SIG_GPIO_OUT_IDX
#include <soc/io_mux_reg.h>        // PIN_FUNC_GPIO
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <nvs_flash.h>
#include <cmath>
#include <cstring>
#include <new>

static const char* TAG = "FC";  // kept from the flight_computer so log greps cross-apply

// IDF-native timing helpers — same as the FC's (which dropped compat.h).
// The FC's remaining millis() call sites are ported as time_ms(); both are
// esp_timer-derived, same epoch and width.
static inline uint32_t time_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }
static inline uint32_t time_us() { return (uint32_t)esp_timer_get_time(); }
static inline void     delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// Firmware revision recorded in the flight settings snapshot (#165).
#ifndef FW_GIT_SHA
#define FW_GIT_SHA "unknown"
#endif
#ifndef FW_GIT_DIRTY
#define FW_GIT_DIRTY 0
#endif

// EKF timeUpdate()/measUpdate() allocate ~7.5 KB of temporary 15x15 matrices
// on the stack.  flightTask is created with 16 KB (main.cpp).

// ==========================================================================
// SECTION: Baro gate policy (verbatim copy)
// ==========================================================================
// VERBATIM copy of projects/flight_computer/main/baro_gate_policy.h (#450) —
// that header lives in the FC's main/ dir, not a shared component, and this
// project may not reach into a sibling project's source tree.  Keep the two
// in lockstep; the full rationale (sample-drop under EKF decimation, the
// rate-aware spike test) is in the FC header.
class BaroGatePolicy
{
public:
    struct Decision
    {
        bool  fresh  = false;  // sample not yet seen by the EKF path
        bool  spike  = false;  // fresh but rejected as a spike
        bool  accept = false;  // fresh && !spike && !locked -> run baroMeasUpdate
        float delta_m = 0.0f;             // |alt - prev alt| (fresh only)
        float apparent_rate_mps = 0.0f;   // delta / actual pair spacing
    };

    Decision evaluate(uint32_t sample_time_us,
                      float    altitude_m,
                      bool     locked,
                      float    spike_thresh_m,
                      float    spike_rate_mps,
                      uint32_t min_fuse_interval_us)
    {
        Decision d;
        d.fresh = !have_consumed_ || (sample_time_us != last_time_us_);
        if (!d.fresh)
        {
            return d;
        }

        if (have_consumed_)
        {
            d.delta_m = fabsf(altitude_m - prev_alt_m_);
            // Pair spacing from the sample's own timestamps; unsigned
            // subtraction is wrap-safe.  Floor keeps the rate finite on a
            // duplicate/garbage timestamp.
            uint32_t dt_us = sample_time_us - last_time_us_;
            if (dt_us < kMinPairDtUs)
            {
                dt_us = kMinPairDtUs;
            }
            d.apparent_rate_mps = d.delta_m / ((float)dt_us * 1e-6f);
            d.spike = (d.delta_m > spike_thresh_m) &&
                      (d.apparent_rate_mps > spike_rate_mps);
        }

        // Always advance the reference — even on spike, lockout or throttle —
        // so one bad sample (or a lockout window) cannot poison every
        // following compare.
        last_time_us_  = sample_time_us;
        prev_alt_m_    = altitude_m;
        have_consumed_ = true;

        // Fusion throttle: wrap-safe elapsed-time check against the last
        // ACCEPTED sample.  The first-ever sample always passes.
        const bool throttled =
            (min_fuse_interval_us > 0u) && have_fused_ &&
            (sample_time_us - last_fused_time_us_) < min_fuse_interval_us;

        d.accept = !d.spike && !locked && !throttled;
        if (d.accept)
        {
            last_fused_time_us_ = sample_time_us;
            have_fused_ = true;
        }
        return d;
    }

private:
    static constexpr uint32_t kMinPairDtUs = 100u;

    uint32_t last_time_us_       = 0;
    float    prev_alt_m_         = 0.0f;
    bool     have_consumed_      = false;
    uint32_t last_fused_time_us_ = 0;
    bool     have_fused_         = false;
};

// ==========================================================================
// SECTION: Sensor collector and hardware objects
// ==========================================================================
// The shared sensor/power I2C bus is created in main.cpp before flight_setup()
// runs; the collector ctor captures the handle BY VALUE, so the collectors
// cannot be namespace-scope statics (they would capture nullptr at load time,
// and the collector would then create a second bus on the IIS2MDC pins).
// Placement-new into aligned static storage keeps this a static allocation
// (no heap on the flight path) while deferring construction to flight_setup().
// The references below are bound to the storage address only — touching them
// before flight_setup() has run is invalid, and nothing does.
extern i2c_master_bus_handle_t shared_i2c_bus;   // defined in main.cpp

alignas(SensorCollector)    static uint8_t collector_hw_storage[sizeof(SensorCollector)];
alignas(SensorCollectorSim) static uint8_t collector_sim_storage[sizeof(SensorCollectorSim)];
static SensorCollector&    sensor_collector_hw = *reinterpret_cast<SensorCollector*>(collector_hw_storage);
static SensorCollectorSim& sensor_collector    = *reinterpret_cast<SensorCollectorSim*>(collector_sim_storage);

static SensorConverter sensor_converter;

// Hard-iron mag calibration (issue #96).  Runs only while rocket_state ==
// MAG_CALIBRATION; pad cal flow only, gated against any in-air state.
static MagCalibrator mag_calibrator;
static int64_t mag_cal_last_status_us = 0;
static bool    mag_cal_status_dirty   = false;  // true → publish on next tick regardless of cadence

// IIS2MDC OFFSET regs are zeroed by softReset() inside sensor_collector.begin(),
// so a calibrated boot has to defer the chip-side write until after begin()
// completes.  Loaded from NVS in flight_setup(); applied after begin().
static int16_t pending_mag_cx = 0, pending_mag_cy = 0, pending_mag_cz = 0;
static bool    pending_mag_apply = false;

// Mag-cal session state — see the FC source for the full lifecycle notes
// (session-active drives the prior-OFFSET cache; verify-active drives the
// timer-based polling block; #382 explains the explicit eval-now flag).
static int64_t mag_cal_verify_start_us = 0;
static bool    mag_cal_verify_eval_now = false;
static int16_t mag_cal_prior_cx = 0, mag_cal_prior_cy = 0, mag_cal_prior_cz = 0;
static bool    mag_cal_session_active = false;
static bool    mag_cal_verify_active  = false;
// Safety timeout for the user-driven verify window (#148): if the user walks
// away without tapping Done, the FC eventually evaluates so the chip isn't
// left with unverified offsets indefinitely.
static constexpr int64_t MAG_CAL_VERIFY_DURATION_US = 60'000'000;

// Gyro zero-rate bias restored from NVS (#132).  Applied after begin(),
// alongside the mag offsets, so a stored sensor cal survives reboots.
static int16_t pending_gyro_cal_x = 0, pending_gyro_cal_y = 0, pending_gyro_cal_z = 0;
static bool    pending_gyro_cal_apply = false;

static ISM6HG256Data ism6hg256_data;
static uint8_t ism6hg256_data_buffer[SIZE_OF_ISM6HG256_DATA];
static BMP585Data bmp585_data;
static uint8_t bmp585_data_buffer[SIZE_OF_BMP585_DATA];
static IIS2MDCData iis2mdc_data;
static uint8_t iis2mdc_data_buffer[SIZE_OF_IIS2MDC_DATA];
static IIS2MDCDataSI iis2mdc_latest_si = {};
static bool have_iis2mdc_si = false;
static GNSSData gnss_data;
static uint8_t gnss_data_buffer[SIZE_OF_GNSS_DATA];
static NonSensorData non_sensor_data;
static uint8_t non_sensor_data_buffer[SIZE_OF_NON_SENSOR_DATA];

// Flight loop parameters
static uint32_t last_flight_loop_update_time = 0;
static uint32_t flight_loop_period = (uint32_t)(1000000 / config::FLIGHT_LOOP_UPDATE_RATE);
static uint32_t last_non_sensor_tx_time_us = 0;
static uint32_t non_sensor_tx_period_us = (uint32_t)(1000000UL / config::NON_SENSOR_UPDATE_RATE);

// --- Loop timing instrumentation ---
static uint32_t lt_ekf_total_us = 0;
static uint32_t lt_ekf_max_us = 0;
static uint32_t lt_ekf_count = 0;
static uint32_t lt_loop_count = 0;
// Free-running EKF update-tick counter (#529) — never reset.  Published as
// NonSensorData.ekf_ticks (uint16 wrap) so the flight log records the
// achieved EKF cadence for the replay tool.
static uint32_t ekf_tick_counter = 0;

// --- Converted SI values (latest sample of each sensor) ---
static ISM6HG256DataSI ism6_latest_si = {};
static BMP585DataSI bmp_latest_si = {};
// #260: source-validity bounds for BMP585 pressure (Pa).  Finite bounds also
// reject NaN/Inf.  Downstream trust gates use a tighter 25-125 kPa window.
static constexpr float BMP_PRESSURE_MIN_PA = 1000.0f;
static constexpr float BMP_PRESSURE_MAX_PA = 120000.0f;
// MMC5983MA is not fitted on the mini.  The SI slot stays (zeroed, never
// written) so the EKF mag-source fallback and the health scorecard port
// unchanged from the FC.
static MMC5983MADataSI mmc_latest_si = {};
static bool have_ism6_si = false;
static bool have_bmp_si = false;
static const bool have_mmc_si = false;   // no MMC on this board — const so the compiler folds the branches
static bool have_gnss_si = false;
static bool bmp_new_for_kf = false; // Set when new BMP sample arrives, cleared at loop end.
                                    // Consumed by kinematicChecks (runs every iteration).
                                    // The EKF baro gate deliberately does NOT use it (#450)
                                    // — see BaroGatePolicy.
static GNSSDataSI gnss_latest_si = {};

// --- Flight logic state ---
static RocketState rocket_state = INITIALIZATION;
// #317: latched true once a flight reaches LANDED. Makes LANDED terminal until
// a hardware reboot — no re-arm, no second flight cycle, no re-armed pyros.
// Cleared only at boot (static init) and on sim-completion re-arm.
static bool post_flight_lockout = false;

// Issue #216 — predicate used to gate the app-side test commands.  Rejected
// from INFLIGHT (a launched rocket must not be reachable from test commands)
// AND from MAG_CALIBRATION (an accidental Fire tap while the user tumbles the
// rocket must not arm pyros).
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

// #557 GNSS-absent degraded flight — see the FC source for the three-flag
// lifecycle.  On the mini a dead LC86G (or a wedged one: no reset line, see
// board_v1.h) takes this path instead of hanging the pad.
static constexpr uint32_t GNSS_ABSENT_INIT_DWELL_MS = 5000U;
static bool     gnss_absent_mode    = false;
static bool     gnss_absent_flight  = false;
static uint32_t gnss_absent_dwell_ms = 0;
static bool ground_pressure_found = false;
// MINI: on the FC this latched only after the out computer answered the first
// I2C status poll.  The comms side here is in-process and is initialized
// BEFORE flightTask is created (power-on order in main.cpp), so it is simply
// true.  resetFlightStateForSim() deliberately does NOT clear it — nothing
// would ever re-latch it (the FC re-latched on the next poll), and a cleared
// flag would deadlock READY→PRELAUNCH after every sim stop.
static const bool out_ready = true;
static bool end_flight_sent = false;

// Active board→rocket mounting orientation (FC: phase-1 static from config,
// refined at runtime by the pad-gravity auto-detect / app setting).
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
// (#741: this reference is still not written into the log stream — known gap.)
static double ref_lat_rad = 0.0, ref_lon_rad = 0.0, ref_alt_m = 0.0;
static bool have_ref_pos = false;
static double ref_lat_sum = 0.0, ref_lon_sum = 0.0, ref_alt_sum = 0.0;
static uint32_t ref_pos_count = 0;
static bool ref_pos_frozen = false;
static constexpr uint32_t REF_POS_MAX_AGE_MS = 120000; // 2 minutes
static uint32_t ref_pos_first_time_ms = 0;
// Last GNSS time_us fed to EKF (avoid double-counting same sample)
static uint32_t last_gnss_time_us_for_ekf = 0;
// Duplicate-fix detection: track GPS fix timestamp (second + milli_second).
static uint8_t  last_gnss_fix_second = 0xFF;
static uint16_t last_gnss_fix_ms     = 0xFFFF;
static bool landed_actions_done = false;

static bool burnout_detected = false;
static uint32_t burnout_time_ms = 0;
// Consecutive-sample hysteresis for burnout detection (#197).
static uint16_t burnout_neg_count = 0;

// --- Recovery deployment detection + dynamic logging rate ---
// The user's logging-rate SETTING (persisted in NVS "imu"/"rate").
// IMU_RATE_DYNAMIC means "let the flight drive it"; anything else is fixed.
static uint16_t imu_rate_setting = IMU_RATE_DYNAMIC;
static tr::DeploymentState deployment_state;
static bool     deployment_detected = false;
static uint32_t deployment_time_ms  = 0;

// Put the collector on the ODR the current setting resolves to.  Cheap and
// idempotent: setIsm6Rate stages for the poll task.
static void applyImuRateForFlightPhase()
{
    const uint16_t hz = imuRateResolve(imu_rate_setting, deployment_detected);
    if (hz != sensor_collector_hw.ism6Rate()) sensor_collector_hw.setIsm6Rate(hz);
}

static bool mach_locked_out = false;    // promoted from baro block for apogee voting
static bool gps_new_for_kc = false;     // new GPS sample available for kinematic checks
static bool prev_sim_active = false;
static Preferences prefs;

// --- Inflight reboot recovery ---
// Snapshot of critical flight state, written into the NAND log stream at
// 10 Hz during INFLIGHT (SNAPSHOT_MSG via mini_link::logFrame — the mini's
// replacement for the OC MRAM slot; #104's "no FC flash on the hot path"
// still holds: logFrame is a RAM-ring enqueue, the flush task owns the NAND).
// On unexpected reboot, flight_setup() tail-scans the brownout-recovered
// flight for the last snapshot and restores flight state.
static bool     reboot_recovery = false;      // true during the settle period after recovery
static bool     reboot_recovery_telem = false; // true for rest of flight (telemetry flag)
// FC name: servo_settle_end_ms.  The mini has no servos; the window is kept
// (renamed) because it also times the reboot_recovery flag clear.
static uint32_t recovery_settle_end_ms = 0;
static uint32_t last_snapshot_ms = 0;         // rate-limit snapshots to 10 Hz
// Resend the flight settings snapshot (#165) on the first few INFLIGHT ticks
// plus one late copy at launch+5 s (see FC source for the 7/05 V2 F2 history).
static uint8_t  settings_emit_count = 0;

// ==========================================================================
// SECTION: mini_link telemetry cache updates
// ==========================================================================
// Replaces the OC's I2S parser cache: each group is copied under telem_mux
// where the FC used to enqueue the equivalent I2S frame.  Critical sections
// are plain copies; the comms task snapshots under the same mux.
template <typename T>
static inline void telemStore(T& dst, const T& src, uint32_t& stamp_us)
{
    portENTER_CRITICAL(&mini_link::telem_mux);
    dst = src;
    stamp_us = time_us();
    portEXIT_CRITICAL(&mini_link::telem_mux);
}

// ==========================================================================
// SECTION: Pyro channel state
// ==========================================================================
// Single PYRO_ARM_PIN drives all four channels' shared low-side return FET
// (U9 on the mini).  Per-channel fire is a [ArmSettle → Firing → Done] state
// machine — ARM goes HIGH only while at least one channel sits in ArmSettle
// or Firing (or a FIRE-TEST is running), then drops LOW once everyone is
// back to Idle/Done.  Spinlock protects all pyro state and GPIO ops.
static portMUX_TYPE pyro_spinlock = portMUX_INITIALIZER_UNLOCKED;
static PyroConfigData pyro_config = {};  // zeroed = all four disabled

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

// Per-channel pin lookups (indexed 0..3).
static const uint8_t PYRO_FIRE_PINS[4] = {
    config::PYRO1_FIRE_PIN, config::PYRO2_FIRE_PIN,
    config::PYRO3_FIRE_PIN, config::PYRO4_FIRE_PIN,
};
static const uint8_t PYRO_CONT_PINS[4] = {
    config::PYRO1_CONT_PIN, config::PYRO2_CONT_PIN,
    config::PYRO3_CONT_PIN, config::PYRO4_CONT_PIN,
};

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

// Continuity polarity — VERIFIED identical to the FC's (#264): the mini forked
// the rocket-computer pyro sheet.  Each CONT node is pulled up to V_MCU_SWTCH
// through an R8-class resistor and an attached igniter pulls it toward
// PYRO_GND through R73 — so raw LOW = continuity present, HIGH = open loop.
// Mini-specific: the pull-ups hang on the SWITCHED rail (board_v1.h), so CONT
// is only meaningful while PWR_PIN's rail is up — which is guaranteed here
// because the whole flight side only runs in ACTIVE mode.
static inline bool pyroContFromRaw(int raw) { return raw == 0; }

// Reset a pyro OUTPUT pad to a known LOW-driven state WITHOUT going through
// gpio_reset_pin(), which briefly enables the internal pull-up (~50 kΩ) as
// part of its `GPIO_MODE_DISABLE` config. The pyro gate drivers are DTC123J-
// style pre-biased NPNs (internal 2.2 kΩ base resistor + 47 kΩ B-E pull-down):
// a 50 kΩ pull-up on the GPIO biases the base well above Vbe for the
// microseconds-long window between gpio_reset_pin's pull-up config and our
// subsequent pull-up-disable config — long enough to momentarily turn on the
// ARM / FIRE MOSFETs and twitch the squib rail at boot. This sequence
// pre-loads GPIO_OUT to 0, detaches any peripheral output signal, selects
// plain-GPIO function on the IO MUX, and only then enables output drive — so
// the pad goes from high-Z directly to driving 0 with no pull-up window.
static void safePyroOutputInit(gpio_num_t pin)
{
    // 1. Pre-load output register to 0. No-op until output is enabled.
    gpio_set_level(pin, 0);
    // 2. Detach any peripheral output signal routed to this pad.
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
    // rail flashes on every boot.
    //
    // If you add new pyro output pins, they MUST go through
    // safePyroOutputInit(). The CONT input pads are fine on gpio_reset_pin()
    // because they have no gate driver downstream.
    // See project_pyro_safe_init_required.md in the agent memory and
    // commit 421dd63 for the bench observation.
    //
    // Mini note: every FIRE pin and ARM sit in the S3's GPIO1-14 group,
    // which glitches LOW ONLY (~60 µs) at power-up (board_v1.h) — the chip
    // cannot drive them high before firmware takes control.  This function
    // is the moment firmware takes control; it must run before any other
    // peripheral init on the flight side.
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
    // #317: once a flight has landed, never re-arm the pyro rail until a
    // hardware reboot — only disarm requests are honored.
    if (post_flight_lockout) want_high = false;
    if (pyro_arm_pin_state == want_high) return;
    gpio_set_level((gpio_num_t)config::PYRO_ARM_PIN, want_high ? 1 : 0);
    pyro_arm_pin_state = want_high;
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
// FC note ported: on the FC the CONT divider is fed from always-on VPP.  On
// the mini it is fed from the SWITCHED rail — equivalent here, because this
// code only ever runs with the rail up (ACTIVE mode).
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
        // bench stay cold. PYRO_FIRE_TEST (the explicit user ground test)
        // is deliberately NOT gated — that command means "energize this
        // channel now" regardless of sim state.
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
    // last pad test stands. (#382 comment fix ported: the CONT divider is
    // rail-fed and always readable while ACTIVE (#264) — ARM=LOW does NOT
    // float CONT; we just don't bother re-sampling outside fire windows.)
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
// SECTION: Flight snapshot (crash recovery)
// ==========================================================================
static uint32_t computeSnapshotCRC(const FlightSnapshotData& snap)
{
    CRC32 crc;
    crc.add(reinterpret_cast<const uint8_t*>(&snap),
            offsetof(FlightSnapshotData, crc32));
    return crc.calc();
}

// Build a FlightSnapshotData from current flight state.  Used by both the
// periodic save (goes into the NAND log stream) and clearFlightSnapshot
// (writes a state=LANDED snapshot so the recovery tail-scan finds an
// invalid-for-recovery record last).
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

    snap.ekf_initialized  = ekf_initialized  ? 1 : 0;
    snap.guidance_enabled = 0;   // no guidance stack on the mini (wire field kept)
    snap.burnout_detected = burnout_detected ? 1 : 0;
    snap.servo_enabled    = 0;   // no servos on the mini (wire field kept)

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

    // Hand off to the log ring — non-blocking RAM enqueue; the flush task
    // owns the NAND.  ~2.3 KB/s at 10 Hz on top of the sensor stream.
    // No flash writes, no NVS, no cache disable from this task (#104 intent
    // preserved).  The frame reaches NAND only while a logging session is
    // active — which is exactly the INFLIGHT window this runs in.
    (void)mini_link::logFrame(SNAPSHOT_MSG,
                              reinterpret_cast<const uint8_t*>(&snap),
                              (uint8_t)sizeof(snap));
}

static void clearFlightSnapshot()
{
    // Write a snapshot with rocket_state=LANDED so the recovery tail-scan
    // (which restores only when the LAST snapshot in the stream is INFLIGHT)
    // finds an invalid-for-recovery record.  Outside a logging session this
    // frame dies in the RAM ring — harmless; the recovery gates (unexpected
    // reset reason + newest-recovered-flight) carry the load there.
    FlightSnapshotData snap = {};
    buildFlightSnapshot(snap, time_ms(), /* override_state = */ (uint8_t)LANDED);
    (void)mini_link::logFrame(SNAPSHOT_MSG,
                              reinterpret_cast<const uint8_t*>(&snap),
                              (uint8_t)sizeof(snap));
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

// ==========================================================================
// SECTION: Board-to-rocket orientation
// ==========================================================================
// Apply a discrete board→rocket mounting orientation everywhere it matters:
// the converter (rotates all vector sensors into rocket frame), the sim
// collector (inverse, so synthesized sensor counts survive the forward
// path), and the cached b2r_active_* globals (flight settings snapshot).
// The FC additionally mirrored this into the OC status-query payload; the
// mini's comms side reads b2r via the FlightSettings frames instead.
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

// Latest filtered pad "up" (specific-force) direction in ROCKET frame, from
// the orientation estimator. Cached so the sim can align its pad frame to how
// the board is ACTUALLY resting when a sim run starts (#508).
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
    // clocking, which gravity cannot observe).  Auto-detect only warns when
    // gravity disagrees badly.
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
// SECTION: Flight settings snapshot
// ==========================================================================
// Build a snapshot of the active settings (#165).  The wire format
// (FlightSettingsData) is unchanged so existing flight-report tooling parses
// mini logs; fields owned by subsystems this board doesn't have (servo/PID/
// gain-schedule/roll/guidance) are zeroed, and their F_* flag bits stay 0 —
// readers already treat that as "subsystem absent".
static void buildFlightSettings(FlightSettingsData& s)
{
    memset(&s, 0, sizeof(s));
    s.time_us = time_us();
    s.version = FlightSettingsData::VERSION;

    uint8_t flags = 0;
    if (FW_GIT_DIRTY)       flags |= (uint8_t)(1u << FlightSettingsData::F_FW_DIRTY);
    s.flags = flags;
    s.roll_delay_ms = 0;

    // PID / angle-loop / gain-schedule / setpoint fields: memset already
    // zeroed them (no roll control on this board).

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

    s.camera_type = 0;   // no camera on this board
    s.pyro        = pyro_config;

    // fw_git_sha buffer was zeroed by memset above, so strncpy of at most
    // size-1 chars leaves it NUL-terminated.
    strncpy(s.fw_git_sha, FW_GIT_SHA, sizeof(s.fw_git_sha) - 1);

    // roll_profile stays zeroed (num_waypoints = 0 → rate-only/none).

    // Board→rocket mounting orientation that actually flew (v2).
    s.b2r_code = b2r_active_code;
    s.b2r_mode = b2r_active_mode;
    s.b2r_residual_cdeg = b2r_active_residual_cdeg;
    for (int i = 0; i < 4; ++i) {
        s.b2r_q[i] = (int16_t)lroundf(b2r_active_quat[i] * ORIENT_QUAT_WIRE_SCALE);
    }

    // guid_tgt_* tail stays zeroed; src 0 = overhead/none (no guidance stack).
}

// #386 clamp (FC parity): a raw lroundf cast wraps sign for |bias| > ~327 m/s².
static inline int16_t encodeHgBiasCmss(float bias_mss)
{
    float v = bias_mss * 100.0f;
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)lroundf(v);
}

// On the two-MCU boards the OC logs the FC's OUT_STATUS_QUERY frames as
// received, so every log carries the sensor config (full scales, rotations,
// HG bias, b2r, mag type) the analysis tooling keys its conversions off.
// The mini has no query wire — build the same frame from the state actually
// applied and log it alongside each FlightSettings snapshot, so mini logs
// parse with this board's real config instead of the parser's big-board
// defaults.
static void logOutStatusQuery()
{
    OutStatusQueryData q = {};
    q.ism6_low_g_fs_g  = config::ISM6_LOW_G_FS_G;
    q.ism6_high_g_fs_g = config::ISM6_HIGH_G_FS_G;
    q.ism6_gyro_fs_dps = config::ISM6_GYRO_FS_DPS;
    q.ism6_rot_z_cdeg  = (int16_t)lroundf(config::ISM6HG256_ROT_Z_DEG * 100.0f);
    q.mmc_rot_z_cdeg   = 0;   // no MMC5983MA fitted
    q.format_version   = 6;
    q.hg_bias_x_cmss = encodeHgBiasCmss(sensor_converter.highGBiasX());
    q.hg_bias_y_cmss = encodeHgBiasCmss(sensor_converter.highGBiasY());
    q.hg_bias_z_cmss = encodeHgBiasCmss(sensor_converter.highGBiasZ());
    q.b2r_code = b2r_active_code;
    q.b2r_mode = b2r_active_mode;
    for (int i = 0; i < 4; ++i) {
        q.b2r_q[i] = (int16_t)lroundf(b2r_active_quat[i] * ORIENT_QUAT_WIRE_SCALE);
    }
    // Read back from the converter rather than config:: so this stays right
    // when the mag rotation constant is renamed/re-derived (#797 QMC seam).
    q.iis2mdc_rot_z_cdeg =
        (int16_t)lroundf(sensor_converter.iis2mdcRotationZDeg() * 100.0f);
    // tgt_* stay zeroed: no guidance stack (GUID_TGT_NONE / GUID_RC_NONE = 0).
#ifdef TR_MAG_DRIVER_QMC5883P
    q.mag_type = MAG_TYPE_QMC5883P;   // #797 mag — counts at 100/3750 µT/LSB
#else
    q.mag_type = MAG_TYPE_IIS2MDC;    // counts at 0.15 µT/LSB
#endif
    (void)mini_link::logFrame(OUT_STATUS_QUERY,
                              reinterpret_cast<const uint8_t*>(&q),
                              (uint8_t)sizeof(q));
}

static void sendFlightSettings()
{
    FlightSettingsData s;
    buildFlightSettings(s);
    (void)mini_link::logFrame(FLIGHT_SETTINGS_MSG,
                              reinterpret_cast<const uint8_t*>(&s),
                              (uint8_t)sizeof(s));
    // The sensor-config snapshot rides every settings emission (launch ×4,
    // 5 s pre-flight cadence, on-demand) — same redundancy story as #418.
    logOutStatusQuery();
}

// ==========================================================================
// SECTION: Calibration status publishing
// ==========================================================================
// Issue #132 — publish a one-shot status frame built from whatever the
// "mag_cal" NVS namespace currently holds.  On the mini "publish" means:
// into the log stream AND into the mini_link telem cache the comms side
// serves to BLE (the OC used to cache these frames off the I2S stream).
static void publishMagCalStatus(const MagCalStatusData& s)
{
    uint8_t buf[sizeof(MagCalStatusData)];
    memcpy(buf, &s, sizeof(buf));
    (void)mini_link::logFrame(MAG_CAL_STATUS_MSG, buf, (uint8_t)sizeof(buf));
    telemStore(mini_link::telem.mag_cal, s, mini_link::telem.mag_cal_update_us);
}

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

    publishMagCalStatus(s);
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
    (void)mini_link::logFrame(SENSOR_CAL_STATUS_MSG, buf, (uint8_t)sizeof(buf));
    telemStore(mini_link::telem.sensor_cal, s, mini_link::telem.sensor_cal_update_us);
}

// ==========================================================================
// SECTION: Simulator re-arm
// ==========================================================================
// #393: reset the flight state machine for a sim run.  Used on the sim START
// edge (fresh run — the sim's equivalent of a reboot) and on an explicit sim
// STOP (SIM_STOP_CMD → abort back to READY).  Deliberately NOT called on the
// sim's natural completion (SIM_LANDED → SIM_IDLE auto-stop), so a flown-out
// sim still holds LANDED to validate the post-flight lockout.
static void resetFlightStateForSim(const char* edge)
{
    // Also resets GNSS state: the sim injects synthetic GNSS (fix=3, sats=12);
    // a stale copy would otherwise immediately re-trip READY -> PRELAUNCH.
    pyroSafeAll();
    rocket_state = READY;
    post_flight_lockout = false;  // #317: a deliberate sim start/stop re-arms
    ground_pressure_found = false;
    // MINI: the FC cleared out_ready here and re-latched it on the next OC
    // poll.  There is no poll to re-latch from — out_ready is const true.
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
    gnss_absent_dwell_ms = time_ms() + GNSS_ABSENT_INIT_DWELL_MS;
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
    reboot_recovery = false;
    reboot_recovery_telem = false;
    clearFlightSnapshot();
    ESP_LOGI(TAG, "[STATE] Sim %s -> READY", edge);
}

// ==========================================================================
// SECTION: INFLIGHT entry
// ==========================================================================
// INFLIGHT entry — everything that must happen exactly once at launch
// (state flip, per-flight resets, degraded-mode handling).  Factored out of
// the PRELAUNCH case so the #382 READY-launch fallback promotes through the
// IDENTICAL path and the two can never drift.
static void enterInflight(uint32_t now_ms, const char* from_state)
{
    rocket_state = INFLIGHT;
    launch_time_millis = now_ms;
    // Orientation is now latched (estimator only runs in READY/PRELAUNCH).
    // Start the boost-phase thrust-axis cross-check on what was latched.
    orient_thrust_mismatch = false;
    orient_thrust_check.start(time_us());
    // Warn if the EKF never initialized — the flight still runs (baro/IMU
    // detectors carry recovery); on the FC this also forced guidance off.
    if (!ekf_initialized) {
        ESP_LOGW(TAG, "[EKF] WARNING: Launching without EKF initialization!");
    }
    // Freeze the ENU reference position at launch
    if (!ref_pos_frozen && have_ref_pos) {
        ref_pos_frozen = true;
        ESP_LOGI(TAG, "[EKF] Ref pos frozen (launch): n=%lu",
                      (unsigned long)ref_pos_count);
    }
    // #297: do NOT re-capture ground pressure here.  launch_flag lags real
    // liftoff, so bmp_latest_si is already a few m up and would bias AGL
    // (and the ALTITUDE_ON_DESCENT main-deploy height) low for the whole
    // flight.  Keep the reference frozen from READY/PRELAUNCH.
    max_alt_m = 0.0f;
    max_speed_mps = 0.0f;
    landed_actions_done = false;
    landed_candidate_active = false;   // #297
    // Clear apogee/landing flags so they start clean at launch.
    // launch_flag is intentionally preserved (it got us here).
    // Per #142/#143: the original reset cleared only baro and velocity flags,
    // leaving stale gps/pitch/master apogee values that could survive a
    // reboot-recovery into the next ascent.
    kinematics.alt_apogee_flag = false;
    kinematics.vel_u_apogee_flag = false;
    kinematics.gps_apogee_flag = false;
    kinematics.pitch_apogee_flag = false;
    kinematics.apogee_flag = false;
    kinematics.alt_landed_flag = false;
    kinematics.max_speed = 0.0f;
    burnout_detected = false;
    burnout_time_ms = 0;
    burnout_neg_count = 0;
    // Deployment is a per-flight latch like burnout (see resetFlightStateForSim).
    tr::deploymentReset(deployment_state);
    deployment_detected = false;
    deployment_time_ms  = 0;
    applyImuRateForFlightPhase();
    // Reset pyro channels on launch. ARM stays LOW until a channel's trigger
    // fires (per-fire arming).
    pyro_apogee_detected = false;
    pyro_apogee_time_ms  = 0;
    // #382: re-arm the per-flight settings snapshot (#165) — without this only
    // the first flight/sim per boot emitted FlightSettingsData.
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
// SECTION: Command dispatch from the comms side (mini_link::cmd_queue)
// ==========================================================================
// Replaces the FC's OUT_STATUS_QUERY poll + pending-command + readConfigFrame
// dance.  The comms side enqueues the SAME message-type frames it used to
// relay over I2C; payloads arrive in-frame, so every readConfigFrame() call
// site becomes a length check + memcpy.  Delivery is exactly-once (FreeRTOS
// queue), so the FC's out_pending_command/last_processed_cmd dedup is gone.
// Handler bodies are otherwise ported 1:1 — same order, same guards, same
// log lines — from the FC's dispatch chain.
static void handleCommandFrame(const mini_link::CmdFrame& cmd, uint32_t now_ms)
{
    (void)now_ms;
    ESP_LOGI(TAG, "[CMD RX] type=0x%02X len=%u", (unsigned)cmd.type, (unsigned)cmd.len);

    if (cmd.type == SIM_CONFIG_MSG)
    {
        if (cmd.len >= sizeof(SimConfigData))
        {
            SimConfigData cfg;
            memcpy(&cfg, cmd.payload, sizeof(cfg));
            sensor_collector.configureSim(cfg);
        }
        else
        {
            ESP_LOGW(TAG, "[SIM CFG] short payload (%u < %u)",
                     (unsigned)cmd.len, (unsigned)sizeof(SimConfigData));
        }
    }
    else if (cmd.type == SIM_START_CMD)
    {
        // The FC re-read a possibly-overwritten config frame here (I2C slave
        // race); the queue can't lose an already-enqueued SIM_CONFIG_MSG, so
        // only the defaults fallback survives.
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
        // attitude before the first synthetic sample. Without this, a bench
        // board lying flat makes rocket-frame gravity jump ~90° the instant
        // sim data replaces real data; the EKF can only explain that step by
        // blaming the gyro bias, which slams to ~-190 dps and is then frozen
        // in at launch — wrecking attitude for the whole flight.
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
    else if (cmd.type == SIM_STOP_CMD)
    {
        // #393: an explicit Stop aborts the sim flight back to READY.
        // Reset here (not on the isSimActive falling edge) so it fires only
        // for a real user Stop, never a naturally-completed sim.
        sensor_collector.stopSim();
        resetFlightStateForSim("stop");
        ESP_LOGI(TAG, "[SIM] Stop cmd received — flight state reset (#393)");
    }
    else if (cmd.type == GROUND_TEST_START || cmd.type == GROUND_TEST_STOP)
    {
        // The FC's ground test drives the fins (attitude-hold / roll null).
        // No fins on this board — acknowledge and do nothing so an app that
        // still offers the button gets a truthful log instead of silence.
        ESP_LOGW(TAG, "[GROUND TEST] unsupported on rocket-computer-mini "
                      "(no control surfaces) — ignored");
    }
    else if (cmd.type == GYRO_CAL_CMD)
    {
        ESP_LOGI(TAG, "[CAL] Sensor calibration requested...");
        sensor_collector.calibrateGyro(config::ISM6HG256_ROT_Z_DEG);
        sensor_converter.setHighGBias(sensor_collector.hg_bias_x,
                                      sensor_collector.hg_bias_y,
                                      sensor_collector.hg_bias_z);
        // (FC also echoed the bias into the OC status-query payload — the
        // mini's comms side reads it back via SENSOR_CAL_STATUS below.)
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
    // Entry refused only from INFLIGHT.  With #216 the MAG_CALIBRATION state
    // inhibits launch detection, pyro arming, EKF init, and the test
    // commands, so the cal session is safe on the pad even with
    // continuity-armed pyros.  All transitions produce one immediate status
    // frame so the iOS UI updates without waiting for the 5 Hz cadence.
    else if (cmd.type == MAG_CAL_START)
    {
        if (rocket_state == INFLIGHT)
        {
            ESP_LOGW(TAG, "[MAGCAL] start refused: INFLIGHT");
        }
        else
        {
            // Cache the currently-persisted offsets so a session-level abort
            // (or verify-fail abort) can roll back to them.
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

            // Zero the chip's OFFSET regs so SAMPLING sees raw mag values.
            // Without this a recalibration fits the residual hard-iron, then
            // overwrites the prior cal with that partial value and fails
            // verification (observed 2026-05-26).
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

            // Issue #216 — entering MAG_CALIBRATION must NOT leave any
            // flight-time effects active: reset kinematics so the tumble
            // can't latch kinematics.launch_flag (which would re-promote
            // PRELAUNCH -> INFLIGHT after the cal ends).  See the
            // `case MAG_CALIBRATION:` block in the state machine, which also
            // skips kinematicChecks for the same reason.  (FC also stowed
            // servos here — none on this board.)
            kinematics.reset();
            burnout_detected = false;
            burnout_neg_count = 0;
            tr::deploymentReset(deployment_state);
            deployment_detected = false;
            deployment_time_ms  = 0;
        }
    }
    else if (cmd.type == MAG_CAL_ABORT)
    {
        ESP_LOGI(TAG, "[MAGCAL] abort");
        // Full session ends — restore the prior NVS offsets to the chip
        // (regardless of which sub-state we're aborting from).
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
        // Calibrator is now in ABORTED — leave it there (iOS treats .aborted
        // and .idle the same); start() on the next session resets cleanly.
    }
    else if (cmd.type == MAG_CAL_RETRY)
    {
        if (rocket_state != MAG_CALIBRATION)
        {
            ESP_LOGW(TAG, "[MAGCAL] retry refused: not in MAG_CALIBRATION");
        }
        else
        {
            ESP_LOGI(TAG, "[MAGCAL] retry");
            // Retry restarts SAMPLING within the same session, so the chip
            // must be at zero (not the prior NVS offsets).  Only matters when
            // retrying out of VERIFYING.
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
    else if (cmd.type == MAG_CAL_COMPUTE_FIT)
    {
        if (rocket_state != MAG_CALIBRATION)
        {
            ESP_LOGW(TAG, "[MAGCAL] compute_fit refused: not in MAG_CALIBRATION");
        }
        else if (!mag_calibrator.computeFit())
        {
            ESP_LOGW(TAG, "[MAGCAL] compute_fit refused: insufficient samples");
            // Still publish a status frame so the iOS button doesn't stick.
            mag_cal_status_dirty = true;
        }
        else
        {
            ESP_LOGI(TAG, "[MAGCAL] user-triggered fit complete, REVIEW state");
            mag_cal_status_dirty = true;
        }
    }
    else if (cmd.type == MAG_CAL_ACCEPT)
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

            // Prior offsets were cached at MAG_CAL_START; program the
            // proposed-new ones so the verify window measures the corrected
            // stream.  Persist on verify pass; zero on fail.
            if (sensor_collector.isIIS2MDCActive())
            {
                const bool ok = sensor_collector.setIIS2MDCHardIronOffset(cx, cy, cz);
                ESP_LOGI(TAG, "[MAGCAL] IIS2MDC OFFSET applied (pre-verify): "
                              "(%d,%d,%d) %s  prior=(%d,%d,%d)",
                         (int)cx, (int)cy, (int)cz, ok ? "OK" : "FAIL",
                         (int)mag_cal_prior_cx, (int)mag_cal_prior_cy, (int)mag_cal_prior_cz);
            }
            // Clear any prior converter-side MMC offset (sphere fit runs
            // against IIS2MDC LSB units, not MMC counts).
            sensor_converter.setMMCOffset(0, 0, 0);

            // Enter verification window — the per-tick block in loop_fc
            // polls esp_timer and calls evaluateVerify().
            mag_cal_verify_start_us = esp_timer_get_time();
            mag_cal_verify_eval_now = false;  // fresh window — no stale Done (#382)
            mag_cal_verify_active = true;
            mag_cal_status_dirty = true;
            ESP_LOGI(TAG, "[MAGCAL] accept: VERIFYING — user-driven (Done button) "
                          "with %lld µs safety timeout",
                     (long long)MAG_CAL_VERIFY_DURATION_US);
        }
    }
    // #148 — user-driven verify completion.
    else if (cmd.type == MAG_CAL_VERIFY_DONE)
    {
        if (rocket_state != MAG_CALIBRATION || !mag_cal_verify_active)
        {
            ESP_LOGW(TAG, "[MAGCAL] verify_done refused: not in VERIFYING");
        }
        else
        {
            // #382: explicit flag — the old zero-the-timestamp trick was a
            // no-op for the first 60 s of uptime.
            mag_cal_verify_eval_now = true;
            ESP_LOGI(TAG, "[MAGCAL] verify_done: user requested evaluation");
        }
    }
    // #148 — user-override save from REVIEW ("Save and apply") or VERIFYING
    // ("Save anyway").  OFFSET-reg write is idempotent, so unconditional.
    else if (cmd.type == MAG_CAL_FORCE_APPLY)
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
            // R_uT == 0 means no fit has run — refuse rather than persist garbage.
            if (R_uT <= 0.0f)
            {
                ESP_LOGW(TAG, "[MAGCAL] force_apply refused: no fit available");
            }
            else
            {
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
                // Leave the calibrator in APPLIED (not IDLE) so the next
                // status frame shows the "Saved" banner — same reason as the
                // verify-PASS path.
                mag_calibrator.markApplied();
            }
        }
    }
    // #148 — redo the verify rotation without going back to SAMPLING.
    else if (cmd.type == MAG_CAL_VERIFY_RESET)
    {
        if (rocket_state != MAG_CALIBRATION || !mag_cal_verify_active)
        {
            ESP_LOGW(TAG, "[MAGCAL] verify_reset refused: not in VERIFYING");
        }
        else
        {
            mag_calibrator.resetVerify();
            // Restart the safety timer so the user gets another full window.
            mag_cal_verify_start_us = esp_timer_get_time();
            mag_cal_verify_eval_now = false;  // fresh window — no stale Done (#382)
            mag_cal_status_dirty = true;
            ESP_LOGI(TAG, "[MAGCAL] verify_reset: accumulators cleared, timer restarted");
        }
    }
    // Issue #132 — app pushes a saved cal from the active rocket profile back
    // into NVS.  Bypasses the sampling / sphere-fit flow but writes the same
    // NVS keys as MAG_CAL_ACCEPT so the boot-time load path is identical.
    // Gated to READY (same as MAG_CAL_START) so the IIS2MDC OFFSET registers
    // don't change mid-flight or mid-cal.
    else if (cmd.type == MAG_CAL_APPLY_MSG)
    {
        if (rocket_state != READY)
        {
            ESP_LOGW(TAG, "[MAGCAL] apply refused: state=%u (require READY)",
                     (unsigned)rocket_state);
        }
        else if (cmd.len >= sizeof(MagCalApplyData))
        {
            MagCalApplyData ap;
            memcpy(&ap, cmd.payload, sizeof(ap));

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
            ESP_LOGE(TAG, "[MAGCAL] APPLY payload short (%u < %u)",
                     (unsigned)cmd.len, (unsigned)sizeof(MagCalApplyData));
        }
    }
    // Issue #132 — pure NVS query.
    else if (cmd.type == MAG_CAL_READ)
    {
        ESP_LOGI(TAG, "[MAGCAL] READ -> publish status from NVS");
        publishMagCalStatusFromNVS();
    }
    // Issue #132 — app pushes a saved sensor cal (gyro + high-g) back into
    // NVS and applies it.  Gated to READY like the pad cal.
    else if (cmd.type == SENSOR_CAL_APPLY_MSG)
    {
        if (rocket_state != READY)
        {
            ESP_LOGW(TAG, "[SENSORCAL] apply refused: state=%u (require READY)",
                     (unsigned)rocket_state);
        }
        else if (cmd.len >= sizeof(SensorCalApplyData))
        {
            SensorCalApplyData ap;
            memcpy(&ap, cmd.payload, sizeof(ap));

            sensor_collector_hw.gyro_cal_x = ap.gyro_x;
            sensor_collector_hw.gyro_cal_y = ap.gyro_y;
            sensor_collector_hw.gyro_cal_z = ap.gyro_z;
            sensor_converter.setHighGBias(ap.hg_x, ap.hg_y, ap.hg_z);

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
            ESP_LOGE(TAG, "[SENSORCAL] APPLY payload short (%u < %u)",
                     (unsigned)cmd.len, (unsigned)sizeof(SensorCalApplyData));
        }
    }
    else if (cmd.type == SENSOR_CAL_READ)
    {
        ESP_LOGI(TAG, "[SENSORCAL] READ -> publish status from NVS");
        publishSensorCalFromNVS();
    }
    else if (cmd.type == ORIENT_CONFIG_MSG)
    {
        if (cmd.len >= sizeof(ImuOrientConfigData))
        {
            const uint8_t setting = cmd.payload[0];
            if (rocket_state == INFLIGHT)
            {
                // Never re-frame mid-flight: EKF state is only meaningful in
                // the latched frame.
                ESP_LOGW(TAG, "[CFG] IMU orientation change ignored INFLIGHT");
            }
            else if (setting == IMU_ORIENT_AUTO)
            {
                // Back to auto.  Only act when leaving MANUAL — reset to
                // identity and let the pad-gravity detect re-derive.
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
                // Manual code is authoritative — it also fixes the roll
                // clocking, which auto-detect cannot observe.
                const bool changed = (b2r_active_code != setting) ||
                                     (b2r_active_mode != ORIENT_MODE_MANUAL);
                applyBoardToRocketOrientation(setting, ORIENT_MODE_MANUAL);
                if (changed && ekf_initialized) ekf_initialized = false;
                ESP_LOGI(TAG, "[CFG] IMU orientation: MANUAL %s",
                         orientCodeName(setting));
            }
        }
    }
    else if (cmd.type == IMU_RATE_CONFIG_MSG)
    {
        if (cmd.len >= sizeof(ImuRateConfigData))
        {
            uint16_t rate_hz;
            memcpy(&rate_hz, cmd.payload, sizeof(rate_hz));
            if (rocket_state == INFLIGHT)
            {
                // Never mid-flight: the ODR switch produces one odd-length
                // inter-sample gap and changes the log cadence.  This guard
                // is for the USER setting only; the dynamic mode's own
                // step-down at deployment is deliberate.
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
    else if (cmd.type == PYRO_CONFIG_MSG)
    {
        if (cmd.len >= sizeof(PyroConfigData))
        {
            memcpy(&pyro_config, cmd.payload, sizeof(PyroConfigData));
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
    else if (cmd.type == PYRO_CONT_TEST)
    {
        // Direct CONT read — the rail-fed sense divider needs no ARM pulse.
        // Rejected in flight to honor the app's "ground tests are pad-only"
        // UX guarantee.
        if (isCommandLockoutState(rocket_state)) {
            ESP_LOGW(TAG, "[PYRO CONT TEST] Rejected — state=%u (no test commands while INFLIGHT or in MAG_CALIBRATION)",
                     (unsigned)rocket_state);
        } else {
            uint8_t ch = (cmd.len >= 1) ? cmd.payload[0] : 0;
            if (ch < 1 || ch > 4) {
                ESP_LOGW(TAG, "[PYRO CONT TEST] Invalid channel %u", ch);
            } else {
                const int idx = ch - 1;
                const gpio_num_t cont_pin = (gpio_num_t)PYRO_CONT_PINS[idx];

                // Defensive CONT-pad reclaim — peripheral defaults can
                // re-grab the IO MUX between boot and the test.
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
    else if (cmd.type == PYRO_FIRE_TEST)
    {
        // Test-fire a pyro channel from the app (ground test only).
        // Deliberately NOT sim-gated — this command means "energize this
        // channel now" regardless of sim state.
        if (isCommandLockoutState(rocket_state)) {
            ESP_LOGW(TAG, "[PYRO FIRE TEST] Rejected — state=%u (no test commands while INFLIGHT or in MAG_CALIBRATION)",
                     (unsigned)rocket_state);
        } else {
            uint8_t ch = (cmd.len >= 1) ? cmd.payload[0] : 0;
            if (ch < 1 || ch > 4) {
                ESP_LOGW(TAG, "[PYRO FIRE TEST] Invalid channel %u", ch);
            } else {
                const int idx = ch - 1;
                const gpio_num_t arm_pin  = (gpio_num_t)config::PYRO_ARM_PIN;
                const gpio_num_t fire_pin = (gpio_num_t)PYRO_FIRE_PINS[idx];

                esp_gpio_revoke(1ULL << arm_pin);
                esp_gpio_revoke(1ULL << fire_pin);
                // #263: bring the ARM/FIRE pads up through the canonical
                // safe-init — NOT gpio_reset_pin(), which transiently enables
                // the internal pull-up and twitches the DTC123J gate driver
                // (the exact boot-fire hazard initPyroPins guards against).
                // OUTPUT (not INPUT_OUTPUT) is enough — the test only drives.
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

                // #263: restore the canonical low-driven safe state (the old
                // path left the pads in INPUT_OUTPUT after the pulse).
                safePyroOutputInit(arm_pin);
                safePyroOutputInit(fire_pin);

                ESP_LOGI(TAG, "[PYRO FIRE TEST] CH%u fired for %u ms",
                         ch, (unsigned)config::PYRO_FIRE_DURATION_MS);
            }
        }
    }
    else if (cmd.type == FLIGHT_SETTINGS_MSG)
    {
        // Comms-side request for an immediate settings snapshot (the 5 s
        // pre-flight cadence covers steady state; this covers on-demand).
        ESP_LOGI(TAG, "[CFG] flight settings snapshot requested");
        sendFlightSettings();
    }
    else if (cmd.type == START_LOGGING || cmd.type == END_FLIGHT)
    {
        // Logging session control is comms-owned on the mini; nothing for the
        // flight side to do.  Logged so a misrouted enqueue is visible.
        ESP_LOGI(TAG, "[CMD RX] logging-control type 0x%02X is comms-owned — ignored",
                 (unsigned)cmd.type);
    }
    else
    {
        // Includes every type the mini dropped with its hardware
        // (SERVO_*/PID_*/ROLL_*/FIN_*/GUIDANCE_*/CAMERA_*/SOUNDS_*/
        // GAIN_SCHED_*/OTA relay): respond gracefully — log and ignore.
        ESP_LOGW(TAG, "[CMD RX] Unknown/unsupported command type: 0x%02X",
                 (unsigned)cmd.type);
    }
}

// ==========================================================================
// SECTION: Snapshot tail-scan recovery (#261, #104 — mini transport)
// ==========================================================================
// The FC stored the 10 Hz snapshot in the OC's MRAM and asked for it back
// over I2C after an unexpected reset.  The mini has no MRAM: the snapshot
// rides the NAND log stream (SNAPSHOT_MSG frames), and an interrupted flight
// surfaces after reboot as a "flight_recovered_<id>.bin" index entry
// (TR_FlightLog::scanForBrownoutRecovery, run inside flightlog.begin() from
// comms_setup_active — which is sequenced BEFORE flight_setup).  Recovery =
// find that entry, scan its payload tail for the LAST valid snapshot frame,
// and restore exactly as the FC did.
//
// Defensive gates, in order (any miss → boot clean and say why):
//   1. unexpected reset reason (same list as the FC),
//   2. flightlog initialized and holding a recovered entry,
//   3. that entry is the NEWEST flight on the chip (highest flight_id) — an
//      old never-deleted recovered file must not resurrect a stale flight,
//   4. the last snapshot frame validates: magic, version, INFLIGHT, CRC32
//      (the same four the FC required),
//   5. the snapshot is not from a SIMULATED flight (sim_flight, v4) — the
//      dry-fire gate does not survive a reboot.
extern tr_flightlog::TR_FlightLog flightlog;   // defined in main.cpp

// One packed snapshot frame in the log stream: SOF(4) | type | len | 224 | CRC16.
static constexpr size_t kSnapFrameLen = 4 + 1 + 1 + sizeof(FlightSnapshotData) + 2;
// How far back to look.  Snapshots ride at 10 Hz inside a ~100 KB/s stream,
// so ~64 KB covers the last ~6 snapshots — plenty to step over a torn tail.
static constexpr uint32_t kTailScanMaxBytes = 16u * 4080u;

static bool snapshotTailScan(const char* filename, uint32_t final_bytes,
                             FlightSnapshotData& snap_out)
{
    // One flash-page-sized read window plus one frame of overlap carried
    // between windows so a snapshot straddling a page boundary still parses.
    static uint8_t tail_buf[4080 + kSnapFrameLen];

    const uint32_t scan_start = (final_bytes > kTailScanMaxBytes)
                              ? final_bytes - kTailScanMaxBytes : 0u;
    uint32_t offset = scan_start;
    size_t   carry  = 0;
    bool     found  = false;

    while (offset < final_bytes)
    {
        size_t got = 0;
        const tr_flightlog::Status st = flightlog.readFlightPage(
            filename, offset, tail_buf + carry, sizeof(tail_buf) - carry, got);
        if (st != tr_flightlog::Status::Ok || got == 0)
        {
            if (st != tr_flightlog::Status::Ok)
            {
                ESP_LOGW(TAG, "[RECOVERY] readFlightPage(%s, %lu) failed: %s",
                         filename, (unsigned long)offset, tr_flightlog::to_string(st));
            }
            break;
        }

        const size_t avail = carry + got;
        size_t pos = 0;
        while (pos + kSnapFrameLen <= avail)
        {
            if (tail_buf[pos]     == 0xAA && tail_buf[pos + 1] == 0x55 &&
                tail_buf[pos + 2] == 0xAA && tail_buf[pos + 3] == 0x55 &&
                tail_buf[pos + 4] == SNAPSHOT_MSG)
            {
                uint8_t type = 0;
                uint8_t payload[sizeof(FlightSnapshotData)] = {};
                size_t  payload_len = 0;
                if (TR_I2C_Interface::unpackMessage(
                        tail_buf + pos, kSnapFrameLen,
                        type, payload, sizeof(payload), payload_len, true) &&
                    type == SNAPSHOT_MSG &&
                    payload_len == sizeof(FlightSnapshotData))
                {
                    // Keep the LAST valid frame in the stream — a later
                    // LANDED "clear" snapshot must win over an earlier
                    // INFLIGHT one, exactly like the MRAM slot being
                    // overwritten did.
                    memcpy(&snap_out, payload, sizeof(snap_out));
                    found = true;
                    pos += kSnapFrameLen;
                    continue;
                }
            }
            pos++;
        }

        carry = avail - pos;            // < kSnapFrameLen by construction
        if (carry > 0) memmove(tail_buf, tail_buf + pos, carry);
        offset += got;
    }
    return found;
}

// ==========================================================================
// SECTION: Boot setup (flight side)
// ==========================================================================
void flight_setup()
{
    // FC main.cpp:2580-2587 port: park both sensor chip-selects HIGH before
    // any bus activity reaches them. From rail-up until
    // sensor_collector.begin() claims these pads (seconds away — pyro init
    // and NVS run first) they would otherwise float next to a live SPI bus,
    // and a sagging CS during another device's clocking is how a sensor ends
    // up half-selected with a corrupted register file.
    const int sensor_cs_pins[] = { (int)config::ISM6HG256_CS, (int)config::BMP585_CS };
    for (int cs : sensor_cs_pins)
    {
        gpio_set_level((gpio_num_t)cs, 1);
        gpio_config_t cs_cfg = {};
        cs_cfg.pin_bit_mask = 1ULL << cs;
        cs_cfg.mode = GPIO_MODE_OUTPUT;
        gpio_config(&cs_cfg);
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // NVS should already be up (main.cpp app_main inits it first); this is
    // the FC's defensive init kept because it is idempotent and keeps this
    // file safe against a future ordering change.
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }
    ESP_LOGI(TAG, "NVS init: %s", esp_err_to_name(nvs_err));

    // The flight task runs continuously and starves IDLE on CPU 1, so stop
    // the WDT monitoring IDLE cores (idle_core_mask = 0) — IDLE starvation
    // there is normal, not a fault.  The flight task itself IS subscribed
    // (flightTask) and resets the WDT each ~1 ms iteration.
    // #261: trigger_panic = true so that if the loop ever stalls past 5 s
    // (deadlock, wedged peripheral), the WDT panics → reboot → snapshot
    // recovery, instead of leaving the highest-priority core-1 task hung and
    // the vehicle dead.
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms = 5000,
        .idle_core_mask = 0,       // don't monitor any IDLE tasks
        .trigger_panic = true,     // #261: hung flight loop -> reboot + recovery
    };
    esp_task_wdt_reconfigure(&wdt_cfg);

    // Quiet the IDF gpio driver — each gpio_config() prints a multi-field
    // INFO line that drowns the rest of boot.
    esp_log_level_set("gpio", ESP_LOG_WARN);

    // Pyro channels: safe pins FIRST, before any other flight-side init.
    initPyroPins();

    ESP_LOGI(TAG, "Flight side starting (board v%d)....", TR_MINI_BOARD);

    // Construct the collectors now that main.cpp has created the shared I2C
    // bus (see the storage/reference comment at the top of this file).
    // Args mirror the FC's global-construction list; pins from board_v1.h.
    new (collector_hw_storage) SensorCollector(
        (uint8_t)config::ISM6HG256_CS,
        (uint8_t)config::ISM6HG256_INT,
        config::ISM6HG256_UPDATE_RATE,
        (uint8_t)config::BMP585_CS,
        (uint8_t)config::BMP585_INT,
        config::BMP585_UPDATE_RATE,
        // MMC5983MA not fitted (board_v1.h): -1 sentinels; the rate is a
        // ctor-positional only, the part is never initialized.
        (uint8_t)-1,
        (uint8_t)-1,
        config::MMC5983MA_UPDATE_RATE,
        (uint8_t)config::PWR_SDA,          // IIS2MDC shares the power-monitor bus
        (uint8_t)config::PWR_SCL,
        (uint8_t)config::IIS2MDC_INT,      // -1: INT/DRDY not wired — poll only
        config::IIS2MDC_I2C_FREQ_HZ,
        config::IIS2MDC_I2C_ADDR,
        config::GNSS_UPDATE_RATE,          // 10 Hz — LC86G ceiling
        (uint8_t)config::GNSS_RX,
        (uint8_t)config::GNSS_TX,
        (int8_t)-1,                        // GNSS RESET_N unconnected (board_v1.h)
        (int8_t)-1,                        // no SAFEBOOT on the LC86G
        config::USE_BMP585,
        config::USE_MMC5983MA,
        config::USE_IIS2MDC,
        config::USE_GNSS,
        config::USE_ISM6HG256,
        config::SPI_SPEED,
        config::ISM6_LOW_G_FS_G,
        config::ISM6_HIGH_G_FS_G,
        config::ISM6_GYRO_FS_DPS,
        shared_i2c_bus);                   // mini seam: app-owned I2C bus
    new (collector_sim_storage) SensorCollectorSim(sensor_collector_hw);

    // Restore the user-selected IMU logging rate setting (NVS "imu"/"rate").
    // Staged into the collector BEFORE begin(), which programs the chip ODR
    // from it.  Whitelist on read so a corrupted NVS value can't run the IMU
    // at an unplanned rate.
    //
    // A boot always starts pre-deployment, so DYNAMIC resolves to the boost
    // rate here — including a mid-flight reboot recovery (see the FC source
    // for the descent-at-boost-rate trade).
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

    // Restore magnetometer hard-iron offset from NVS (namespace "mag_cal",
    // issue #96).  IIS2MDC offsets must be reapplied AFTER begin() because
    // softReset() inside begin() zeroes them.  The converter MMC offset is
    // set too even though no MMC exists — one NVS schema for both mag chips,
    // no divergence.
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
            // Stash on the side; applied to the IIS2MDC chip after begin().
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

    // Re-init pyro pins before the collector touches the buses.  The FC's
    // second call sat directly after its spi_bus_initialize() (P4 SPI2
    // defaults overlapped the pyro pins and the bus init could reclaim them).
    // On the mini both buses come up in main.cpp before flight_setup(), so
    // strictly both calls already run post-bus-init — the pair is kept as
    // cheap insurance and the house pattern (S3 GPIO-matrix routing should
    // never touch these pads; "should" is not a pyro-safety word).
    initPyroPins();

    // Initialize sensor collector (including sensors) and start polling tasks
    ESP_LOGI(TAG, "Sensor collector init...");
    sensor_collector.begin(config::SENSOR_CORE);

    // #557: latch GNSS-absent mode.  isGnssOnline() is false when GNSS is
    // built in but the module failed bring-up (dead/deaf UART).  The flight
    // side then flies a baro+IMU-only degraded path instead of hanging on
    // the pad.  begin() blocks until the driver's bring-up deadline in that
    // case, so time_ms() is already well past zero here.
    gnss_absent_mode     = config::USE_GNSS && !sensor_collector_hw.isGnssOnline();
    gnss_absent_dwell_ms = time_ms() + GNSS_ABSENT_INIT_DWELL_MS;
    if (gnss_absent_mode)
    {
        ESP_LOGW(TAG, "[GNSS] Module absent (bring-up failed) — GNSS-absent "
                      "degraded mode: baro+IMU EKF init");
    }
    sensor_converter.configureISM6HG256FullScale(
        static_cast<ISM6LowGFullScale>(config::ISM6_LOW_G_FS_G),
        static_cast<ISM6HighGFullScale>(config::ISM6_HIGH_G_FS_G),
        static_cast<ISM6GyroFullScale>(config::ISM6_GYRO_FS_DPS));
    sensor_converter.configureISM6HG256RotationZ(config::ISM6HG256_ROT_Z_DEG);
    sensor_converter.configureIIS2MDCRotationZ(config::IIS2MDC_ROT_Z_DEG);
    sensor_collector.configureSimRotation(config::ISM6HG256_ROT_Z_DEG);
    sensor_collector.configureSimIis2mdcRotation(config::IIS2MDC_ROT_Z_DEG);

    // Board→rocket mounting orientation (converter + sim).
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

    // ── Inflight reboot recovery ────────────────────────────────────────────
    // If the reset was unexpected (brownout, watchdog, panic), look for the
    // interrupted flight the brownout scan just indexed and restore state to
    // resume INFLIGHT ops.  See snapshotTailScan above for the gate list.
    {
        esp_reset_reason_t rst = esp_reset_reason();
        ESP_LOGI(TAG, "Reset reason: %s (%d)", resetReasonStr(rst), (int)rst);

        const bool unexpected_reset = (rst == ESP_RST_BROWNOUT ||
                                       rst == ESP_RST_PANIC    ||
                                       rst == ESP_RST_INT_WDT  ||
                                       rst == ESP_RST_TASK_WDT ||
                                       rst == ESP_RST_WDT);

        // Recovery may only run on the boot that interrupted the flight:
        // this power-on must be the ACTIVE-restore policy's automatic
        // re-entry (mini_link::active_restore_boot — main.cpp found the
        // persisted ACTIVE flag) AND this chip reset must itself be
        // unexpected. A manual power-on can carry a stale reset reason from
        // a crash-in-IDLE hours earlier (a chip reset never precedes a
        // commanded power-on), which is why the reason alone is not enough.
        // A battery re-connect after a mid-flight brownout reads POWERON
        // here and correctly declines — same semantics as the FC, where
        // POWERON also skipped MRAM recovery.
        const bool recovery_eligible = mini_link::active_restore_boot && unexpected_reset;

        // The MRAM slot the FC invalidated on every clean boot does not
        // exist here — the equivalent is an NVS marker holding the last
        // recovered-flight id this code has already EVALUATED. Without it, a
        // stale flight_recovered_* file (which nothing deletes, and which
        // stays the newest flight on the chip until the next launch) could
        // satisfy every gate on a later boot and resurrect last week's
        // INFLIGHT snapshot with live igniters connected.
        // done_id alone is not a safe key: flight ids are max-in-index + 1,
        // so deleting files (the normal post-incident download workflow)
        // lets a FUTURE flight reuse the marked id — and a genuine
        // mid-flight reset of that flight would then be refused recovery.
        // The final_bytes discriminator makes an accidental match
        // implausible (same id AND same byte count).
        uint32_t recovery_done_id = 0;
        uint32_t recovery_done_bytes = 0;
        {
            Preferences p;
            if (p.begin("recovery", true)) {
                recovery_done_id = p.getUInt("done_id", 0);
                recovery_done_bytes = p.getUInt("done_by", 0);
                p.end();
            }
        }

        FlightSnapshotData snap = {};
        bool valid = false;
        uint32_t recovered_id = 0;
        uint32_t recovered_bytes = 0;

        if (!flightlog.isInitialized()) {
            if (recovery_eligible) {
                ESP_LOGW(TAG, "[RECOVERY] flightlog not initialized — skipping snapshot recovery");
            }
        } else {
            // Locate the newest recovered entry regardless of eligibility —
            // an ineligible boot still marks it evaluated (the FC's
            // clear-stale-snapshot-on-clean-boot semantics).
            const tr_flightlog::FlightIndex& idx = flightlog.index();
            const tr_flightlog::FlightIndexEntry* recovered = nullptr;
            uint32_t max_flight_id = 0;
            for (size_t i = 0; i < idx.size(); ++i) {
                const tr_flightlog::FlightIndexEntry& e = idx.at(i);
                if (e.flight_id > max_flight_id) max_flight_id = e.flight_id;
                if (strncmp(e.filename, "flight_recovered_", 17) == 0 &&
                    (recovered == nullptr || e.flight_id > recovered->flight_id)) {
                    recovered = &e;
                }
            }
            if (recovered != nullptr) {
                recovered_id = recovered->flight_id;
                recovered_bytes = recovered->final_bytes;
            }
            if (recovered == nullptr) {
                if (recovery_eligible) {
                    ESP_LOGI(TAG, "[RECOVERY] no interrupted flight in the index — booting clean");
                }
            } else if (!recovery_eligible) {
                ESP_LOGI(TAG, "[RECOVERY] interrupted flight %lu present but this boot is not "
                              "an unexpected-reset re-entry (restore_boot=%d reason=%s) — booting clean",
                         (unsigned long)recovered_id,
                         (int)mini_link::active_restore_boot, resetReasonStr(rst));
            } else if (recovered_id == recovery_done_id &&
                       recovered_bytes == recovery_done_bytes) {
                ESP_LOGW(TAG, "[RECOVERY] recovered flight %lu (%lu B) already evaluated on a "
                              "previous boot — booting clean",
                         (unsigned long)recovered_id, (unsigned long)recovered_bytes);
            } else if (recovered->flight_id != max_flight_id) {
                ESP_LOGW(TAG, "[RECOVERY] recovered flight %lu is stale (newest id %lu) — booting clean",
                         (unsigned long)recovered->flight_id, (unsigned long)max_flight_id);
            } else if (!snapshotTailScan(recovered->filename, recovered->final_bytes, snap)) {
                ESP_LOGW(TAG, "[RECOVERY] no snapshot frame in %s tail — booting clean",
                         recovered->filename);
            } else if (snap.magic   == FlightSnapshotData::MAGIC &&
                       snap.version == FlightSnapshotData::VERSION &&
                       snap.rocket_state == (uint8_t)INFLIGHT &&
                       snap.crc32 == computeSnapshotCRC(snap)) {
                if (snap.sim_flight) {
                    // The dry-fire gate lives in isSimActive(), which a reboot
                    // resets — restoring here would fire real pyro outputs on
                    // bench igniters mid-sim.  Falls through to the
                    // evaluated-marker write below like any other decline.
                    ESP_LOGW(TAG, "[RECOVERY] snapshot is from a SIMULATED flight — refusing restore");
                } else {
                    valid = true;
                }
            } else {
                ESP_LOGW(TAG, "[RECOVERY] Snapshot invalid (magic=0x%08lX state=%u crc=%s)",
                         (unsigned long)snap.magic, snap.rocket_state,
                         (snap.crc32 == computeSnapshotCRC(snap)) ? "OK" : "FAIL");
            }

            // Mark the entry evaluated on every DECLINED outcome so it can
            // never be re-evaluated by a later boot. Deliberately NOT
            // written when valid — the restore path below writes it after
            // the restore completes, so a crash mid-restore gets another
            // attempt rather than a half-restored dead end.
            if (recovered != nullptr && !valid &&
                !(recovered_id == recovery_done_id && recovered_bytes == recovery_done_bytes)) {
                Preferences p;
                if (p.begin("recovery", false)) {
                    p.putUInt("done_id", recovered_id);
                    p.putUInt("done_by", recovered_bytes);
                    p.end();
                } else {
                    // A failed write here re-opens the stale-restore window
                    // on the NEXT unexpected reset — be loud about it.
                    ESP_LOGE(TAG, "[RECOVERY] FAILED to persist evaluated-marker for "
                                  "flight %lu — a stale snapshot could be re-evaluated "
                                  "after a future unexpected reset",
                             (unsigned long)recovered_id);
                }
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

            // Restore pyro state (safety-critical: no double-fire, no missed
            // fire).  With per-fire arming, ARM is never persistent — any
            // channel whose trigger condition is still met will re-arm and
            // fire via servicePyroChannels(). We only need to lock in fired
            // channels so they aren't re-fired.
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

            // Restore control state (guidance_enabled/servo_enabled wire
            // fields are ignored — those subsystems do not exist here).
            ekf_initialized  = snap.ekf_initialized;
            landed_actions_done = false;
            end_flight_sent = false;

            // Restore EKF state.  The wire snapshot stores only the diagonal
            // of P (cross-correlations get rebuilt over the next ~0.5-1 s of
            // measurement updates) — see FlightSnapshotData in
            // RocketComputerTypes.h.
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

            // Settle window while the EKF re-converges (FC: servo neutral
            // hold; here it only times the reboot_recovery flag clear).
            reboot_recovery = true;
            reboot_recovery_telem = true;
            recovery_settle_end_ms = now_ms + 500;

            // Mark launch flag so kinematic checks don't re-trigger launch detection
            kinematics.launch_flag = true;

            // Restore complete: mark this recovered flight evaluated so a
            // LATER unexpected reboot (after this recovered flight ends)
            // cannot restore it a second time. The in-flight snapshot
            // stream keeps writing fresh frames for THIS continuation, so
            // a crash while still airborne recovers from the new file, not
            // this marker's.
            {
                Preferences p;
                if (p.begin("recovery", false)) {
                    p.putUInt("done_id", recovered_id);
                    p.putUInt("done_by", recovered_bytes);
                    p.end();
                } else {
                    ESP_LOGE(TAG, "[RECOVERY] FAILED to persist evaluated-marker after "
                                  "restore of flight %lu — a later unexpected reset could "
                                  "restore it again", (unsigned long)recovered_id);
                }
            }
        }

        // The FC also enqueued a LANDED clear-frame on every normal boot.
        // Here that frame dies in a closed log ring (no session is open at
        // boot), so it is NOT the invalidation mechanism — the NVS done_id
        // marker above is. The frame is still sent when a session IS open
        // (in-flight LANDED path) purely to keep the log stream's contents
        // identical to the fleet's.
        if (!reboot_recovery) {
            clearFlightSnapshot();
        }
    }

    ESP_LOGI(TAG, "Setup complete");
}

// ==========================================================================
// SECTION: Main loop
// ==========================================================================
static void loop_fc()
{
    // Zero the IMU-queue drop gauge on the first pass: the poll task starts
    // before this loop does, producing meaningless pre-consumer drops into an
    // unconsumed queue.  From here on a nonzero [GAP DIAG] imu_q_drops means
    // the consumer actually stalled.
    {
        static bool imu_drop_gauge_zeroed = false;
        if (!imu_drop_gauge_zeroed)
        {
            imu_drop_gauge_zeroed = true;
            sensor_collector_hw.resetIsm6QueueDrops();
        }
    }

    // (FC ran fcMaybeMarkOtaValid() + the OTA image pump backoff here — OTA
    // is comms-owned on the mini, self-update only, no flight-side part.)

    // ==========================================================================
    // SECTION: Sensor drain and conversion
    // ==========================================================================
    // Poll as fast as possible so high-rate sensor frames are not dropped by
    // loop-period gating.  Each splice site logs the raw packed struct via
    // mini_link::logFrame (byte-identical stream to the OC's) and refreshes
    // the comms-side telem cache.
    static uint32_t dbg_ism6_reads = 0, dbg_bmp_reads = 0, dbg_bmp_bad_reads = 0, dbg_iis2mdc_reads = 0, dbg_gnss_reads = 0;

    // Drain ALL pending IMU samples each iteration.  The chip runs at the
    // configured ODR — about two samples per ~1 ms loop pass at 1920 Hz —
    // and every one is logged so the recorded IMU rate follows the chip ODR,
    // not the loop rate.  The control/EKF path converts only the FRESHEST
    // drained sample: kinematics and the EKF keep running at loop rate.
    {
        bool ism6_new_this_iter = false;
        while (sensor_collector.getISM6HG256Data(ism6hg256_data))
        {
            dbg_ism6_reads++;
            ism6_new_this_iter = true;

            memcpy(ism6hg256_data_buffer,
                   &ism6hg256_data,
                   SIZE_OF_ISM6HG256_DATA);

            (void)mini_link::logFrame(ISM6HG256_MSG,
                                      ism6hg256_data_buffer,
                                      (uint8_t)SIZE_OF_ISM6HG256_DATA);
        }

        if (ism6_new_this_iter)
        {
            sensor_converter.convertISM6HG256Data(ism6hg256_data, ism6_latest_si);
            have_ism6_si = true;
            // Latest-wins cache for the comms side (one copy per drain pass,
            // not per sample — the cache is a latest-state snapshot, the log
            // stream above carries every sample).
            telemStore(mini_link::telem.imu, ism6hg256_data,
                       mini_link::telem.imu_update_us);

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
        // #260: validate pressure at the source before it drives the altitude
        // / apogee math.  Convert into a candidate and commit only if it's in
        // band — the bounded check also rejects NaN/±Inf — so a sensor glitch
        // or SPI-corrupted sample can't poison ground_pressure_pa /
        // pressure_altitude_m and the kinematics KF.  On reject we keep the
        // last good sample (have_bmp_si stays latched; the 0.5 s downstream
        // staleness gate catches a persistent outage).
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

        (void)mini_link::logFrame(BMP585_MSG,
                                  bmp585_data_buffer,
                                  (uint8_t)SIZE_OF_BMP585_DATA);
        telemStore(mini_link::telem.baro, bmp585_data,
                   mini_link::telem.baro_update_us);
    }

    // (FC drained the MMC5983MA here — part not fitted on the mini.)

    if (sensor_collector.getIIS2MDCData(iis2mdc_data))
    {
        dbg_iis2mdc_reads++;
        sensor_converter.convertIIS2MDCData(iis2mdc_data, iis2mdc_latest_si);
        have_iis2mdc_si = true;

        memcpy(iis2mdc_data_buffer,
               &iis2mdc_data,
               SIZE_OF_IIS2MDC_DATA);

        (void)mini_link::logFrame(IIS2MDC_MSG,
                                  iis2mdc_data_buffer,
                                  (uint8_t)SIZE_OF_IIS2MDC_DATA);
        telemStore(mini_link::telem.mag, iis2mdc_data,
                   mini_link::telem.mag_update_us);

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

        (void)mini_link::logFrame(GNSS_MSG,
                                  gnss_data_buffer,
                                  (uint8_t)SIZE_OF_GNSS_DATA);
        telemStore(mini_link::telem.gnss, gnss_data,
                   mini_link::telem.gnss_update_us);
    }

    // ==========================================================================
    // SECTION: Magnetometer calibration status
    // ==========================================================================
    // Issue #96 — periodic mag-cal status frame.  5 Hz cadence in SAMPLING;
    // immediate publish on REVIEW/APPLIED/ABORTED transitions
    // (mag_cal_status_dirty).  Outside MAG_CALIBRATION the calibrator is
    // IDLE; we still publish transitions but otherwise stay quiet.
    {
        constexpr int64_t MAG_CAL_STATUS_PERIOD_US = 200000;  // 5 Hz
        const int64_t now = esp_timer_get_time();
        const bool in_state = (rocket_state == MAG_CALIBRATION);
        const bool elapsed  = (now - mag_cal_last_status_us) >= MAG_CAL_STATUS_PERIOD_US;
        if (mag_cal_status_dirty || (in_state && elapsed))
        {
            MagCalStatusData s{};
            mag_calibrator.buildStatusFrame((uint32_t)now, s);
            publishMagCalStatus(s);
            mag_cal_last_status_us = now;
            mag_cal_status_dirty = false;
        }
    }

    // #206 — post-accept verify window.  Polls the calibrator's verify
    // accumulators once the user taps Done (or the safety timeout fires).
    // On pass we persist NVS; on fail we zero the chip and the calibrator's
    // own state machine flips back to REVIEW with
    // reject_code = MAG_CAL_REJECT_VERIFY_FAILED so iOS shows the error.
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
                // the next status frame reports sub_type=APPLIED and iOS
                // shows the success banner.  start() on the next session
                // resets all state cleanly.
            }
            else
            {
                // Calibrator already transitioned back to REVIEW with
                // MAG_CAL_REJECT_VERIFY_FAILED set.  Zero the chip (NOT
                // restore prior) so the user can immediately Retry into
                // SAMPLING and see raw mag values.  If they Abort instead,
                // the abort handler does the prior restore.
                if (sensor_collector.isIIS2MDCActive())
                {
                    const bool ok = sensor_collector.setIIS2MDCHardIronOffset(0, 0, 0);
                    ESP_LOGW(TAG, "[MAGCAL] verify FAIL (worst=%.1fµT) — zeroed OFFSET %s; "
                                  "user can Retry (raw sampling) or Abort (restore prior)",
                             (double)worst_uT, ok ? "OK" : "FAIL");
                }
                mag_cal_status_dirty = true;
                // Stay in MAG_CALIBRATION so iOS keeps the cal screen up;
                // the user can tap Retry or Abort from REVIEW.
            }
        }
    }

    // ==========================================================================
    // SECTION: Flight logic: pressure altitude and orientation
    // ==========================================================================
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
            // Track ground pressure continuously through pre-flight phases so
            // the reference is the last steady-state reading before launch.
            // Freeze the reference once we enter PRELAUNCH: if we kept
            // updating, the ratio bmp/ground would always be ~1.0, the
            // filtered altitude rate would never exceed the 1.0 m/s launch
            // threshold, and launch detection would deadlock.  No
            // isSimActive() guard: the state check is sufficient, and letting
            // sim data flow through the same path tests the freeze logic.
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
            // mounting (continuously re-estimated; latched by leaving READY/
            // PRELAUNCH at launch).  During boost, the thrust direction
            // cross-checks whatever was latched.  (The FC also froze the
            // estimator during ground test — no ground test on the mini.)
            if (rocket_state == READY || rocket_state == PRELAUNCH)
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
            // bias seed (wBias = mean gyro, valid because the pad is
            // stationary) averages a few samples instead of one noisy sample.
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
            // Prefer IIS2MDC when its samples are flowing; the MMC fallback
            // branch is kept for FC parity but is dead on this board
            // (have_mmc_si is const false — the compiler folds it away).
            //
            // Magnitude gate: the Mahony AHRS only checks (0,0,0) and then
            // normalises, so an uncalibrated hard-iron offset would dominate
            // the heading reference.  Skip the sample unless |m| sits in a
            // sensible Earth-field band; the AHRS treats a zeroed input as
            // "no mag, use gyro+accel only".
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
            // below must know whether the EKF will actually process the fix
            // this tick (#367).  (The FC also paused the EKF while an OTA
            // image streamed — no such mode here.)
            static uint8_t ekf_decim_ctr = 0;
            const bool run_ekf_this_tick =
                (++ekf_decim_ctr >= config::EKF_DECIMATION);
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
                // this tick.  Advancing these markers on a decimation-"off"
                // tick would drop the fix and inject a zeroed measurement
                // carrying this (never-processed) timestamp.
                if (run_ekf_this_tick)
                {
                    last_gnss_time_us_for_ekf = gnss_latest_si.time_us;
                    last_gnss_fix_second     = gnss_latest_si.second;
                    last_gnss_fix_ms         = gnss_latest_si.milli_second;
                }

                // Scale GNSS noise by h_acc — inflate R when receiver is
                // uncertain.  Nominal R assumes h_acc ≈ 3 m.
                //
                // MINI: 255 is the LC86 driver's "accuracy unknown" sentinel
                // (the $PQTMEPE stream never came up), not a real 255 m
                // estimate. Feeding it through would square to a 7225x
                // position de-weight and the EKF would effectively ignore a
                // healthy fix all flight. Unknown accuracy on an accepted
                // fix gets a modest fixed inflation instead.
                const float h_acc = gnss_latest_si.horizontal_accuracy;
                float gnss_scale;
                if (h_acc >= 255.0f) {
                    gnss_scale = 3.0f;            // unknown: mild de-weight
                } else if (h_acc > 3.0f) {
                    gnss_scale = h_acc / 3.0f;
                } else {
                    gnss_scale = 1.0f;
                }
                ekf.setGpsNoiseScale(gnss_scale);

                // Running average of GNSS fixes for launch-site reference.
                // Only accumulate fixes that pass the stricter init gate.
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
            // Wait for valid GNSS fix before init — avoids initializing at
            // lat=0,lon=0 and then corrupting attitude when a fix arrives.
            // Decimation: run predict+update every Nth tick; sensor
            // consumption and logging still happen every iteration.
            if (!ekf_initialized && rocket_state != MAG_CALIBRATION)
            {
                // Gate 3: only init with high-quality GNSS.  #216: also gate
                // against MAG_CALIBRATION (tumbling violates the stationary-
                // at-init assumption).  #557: degraded init — when the GNSS
                // module is absent, initialize from baro + IMU only so
                // attitude + vertical velocity (hence the velocity/pitch
                // apogee voters) still work.  Gated on initGyroN >= 8, on
                // never having seen a fix, and on a short settle so an
                // injected sim fix wins first.
                const bool normal_init = have_ref_pos && gnss_gate3_init;
                const bool degraded_init =
                    !normal_init && gnss_absent_mode && !gnss_started &&
                    (initGyroN >= 8) &&
                    ((int32_t)(time_ms() - gnss_absent_dwell_ms) >= 0);
                if (normal_init || degraded_init) {
                    if (degraded_init) {
                        // Pad-relative seed: lat/lon 0 (equator — regular, no
                        // geodetic singularity), alt from baro, zero velocity,
                        // and a stale GNSS time so the EKF never runs a GNSS
                        // measurement update against this fake origin.  Baro
                        // then bounds the vertical channel and accel+mag
                        // bound attitude.
                        ekf_gnss.time_us   = 0;
                        ekf_gnss.lat_rad   = 0.0;
                        ekf_gnss.lon_rad   = 0.0;
                        ekf_gnss.alt_m     = pressure_altitude_m;
                        ekf_gnss.vel_n_mps = 0.0f;
                        ekf_gnss.vel_e_mps = 0.0f;
                        ekf_gnss.vel_d_mps = 0.0f;
                    }
                    // #297: seed wBias off the recent-window gyro average
                    // rather than the single live sample (see the ring above).
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
                    // gravity plus the known pad heading, bypassing the noisy
                    // magnetometer.  The IMU data is already in ROCKET frame
                    // here (the converter applies the board→rocket mounting
                    // rotation), so this is mounting-agnostic by construction.
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

                    // Magnetic declination → true-north heading.  Computed
                    // once here (off the hot path) from the averaged pad
                    // position + GPS date via WMM2025; falls back to the
                    // config constant if the GNSS date is implausible.
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
                        // #557: mark the session degraded so the
                        // SH_GNSS_ABSENT telemetry verdict fires.  (The FC
                        // also forced guidance off here — none on the mini.)
                        gnss_absent_flight = true;
                        ESP_LOGW(TAG, "[EKF] GNSS-absent degraded init: baro+IMU "
                                      "only, position pad-relative");
                    }
                }
            }
            else if (run_ekf_this_tick)
            {
                const uint32_t ekf_t0 = time_us();

                // AHRS accel correction: enabled on pad and after apogee
                // (descent).  Disabled during thrust and coast where accel is
                // far from 1g.  The 0.5g–1.5g magnitude gate inside the EKF
                // provides additional rejection, but blanket-disabling during
                // descent starves the filter of its gravity reference.
                const bool post_apogee = kinematics.apogee_flag;
                const bool use_ahrs_acc = (rocket_state != INFLIGHT) || post_apogee;
                ekf.update(use_ahrs_acc, ekf_imu, ekf_gnss, ekf_mag);

                // Barometer measurement update.  Freshness is tracked by
                // BaroGatePolicy against the sample's own timestamp, NOT the
                // loop-lifetime bmp_new_for_kf flag (#450).
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
                        // absolute MSL.  Re-reference baro into the GNSS
                        // frame by adding the launch-site elevation,
                        // otherwise the baro innovation carries a constant
                        // bias ≈ -(launch elevation) that the 15-state filter
                        // leaks into altitude / accel_bias_z — the stationary
                        // altitude drift in #44. (ref_alt_m is ~0 until the
                        // first GNSS fix, by which point the EKF isn't yet
                        // running baro updates.)
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
        // SECTION: Command dispatch from the comms side
        // ==========================================================================
        // Replaces the FC's I2C status poll + pending-command dispatch.  The
        // FC skipped its poll during a real INFLIGHT (no app commands are
        // processed mid-flight) but kept polling during a SIM flight so
        // SIM_STOP could be delivered (#393).  Mirror that exactly: during a
        // real flight, arriving frames are drained AND DISCARDED (the FC
        // equivalent: the OC gave up repeating after its window), so a stale
        // command can't fire after landing.
        {
            mini_link::CmdFrame cmd;
            while (mini_link::cmd_queue != nullptr &&
                   xQueueReceive(mini_link::cmd_queue, &cmd, 0) == pdTRUE)
            {
                if (rocket_state == INFLIGHT && !sensor_collector.isSimActive())
                {
                    ESP_LOGW(TAG, "[CMD RX] type 0x%02X discarded — INFLIGHT",
                             (unsigned)cmd.type);
                    continue;
                }
                handleCommandFrame(cmd, now_ms);
            }
        }

        // ==========================================================================
        // SECTION: Kinematic checks and sensor health
        // ==========================================================================
        // Kinematic checks.  Issue #216 — skip during MAG_CALIBRATION: the
        // user is physically tumbling the rocket, so a vigorous shake would
        // otherwise spike accel_norm past the launch threshold and latch
        // kinematics.launch_flag.  We also clear bmp_new/gps_new here to keep
        // them from accumulating during cal.
        if (rocket_state != MAG_CALIBRATION)
        {
            // #257: per-sensor health for the adaptive apogee quorum +
            // Layer-2 backstop.  baro_healthy = last sample in a plausible
            // range (also rejects NaN/Inf) and not stale — have_bmp_si
            // latches once (#259) so it is not a freshness signal by itself.
            constexpr uint32_t BARO_STALE_TIMEOUT_US = 500000u;  // 0.5 s -> dead
            const bool baro_healthy =
                have_bmp_si &&
                bmp_latest_si.pressure > 25000.0f &&   // BMP585 valid 30-125 kPa;
                bmp_latest_si.pressure < 125000.0f &&  // 25 kPa floor = margin below spec
                (uint32_t)(time_us() - bmp_latest_si.time_us) < BARO_STALE_TIMEOUT_US;
            const bool ekf_healthy = ekf_initialized && ekf.isHealthy();

            // #259 applied to the kinematic checks, matching the FC.
            // ism6_latest_si RETAINS its last value when the IMU stops
            // responding and have_ism6_si latches on first read, so a frozen
            // IMU would otherwise feed a constant accel straight into launch
            // detection and — since #824 — into the landing quiescence
            // detector, whose two gates a frozen sample satisfies
            // indefinitely.  0.0f fails both, i.e. "no evidence from a stale
            // IMU" rather than a synthetic still-and-level reading.
            constexpr uint32_t IMU_STALE_TIMEOUT_US = 100000u;  // 0.1 s (~1 kHz IMU)
            const bool ism6_fresh_kc = have_ism6_si &&
                (uint32_t)(time_us() - ism6_latest_si.time_us) < IMU_STALE_TIMEOUT_US;

            kinematics.kinematicChecks(pressure_altitude_m,
                                       ism6_fresh_kc ? accel_norm : 0.0f,
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
            // would read a stationary airframe as a canopy.
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
        // (#165/#418) live, so their logs contained no FlightSettingsData.
        // Emit a copy every 5 s outside INFLIGHT: any session picks one up
        // within seconds, idle copies churn out of the pre-launch ring
        // harmlessly, and the in-flight schedule is untouched.
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
        // SECTION: Flight state machine
        // ==========================================================================
        // (The FC wrapped this switch in its ground/servo/replay test-mode
        // chain plus the #363 launch-during-test failsafe; those modes are
        // gone with the hardware, so the state machine runs unconditionally.)
        // #317: once a flight has landed, the computer is terminal until a
        // hardware reboot. Force LANDED so any transition a command or a
        // ground-handling re-trigger of launch-detect tries to make is
        // overridden (no re-arm, no second flight).
        if (post_flight_lockout) rocket_state = LANDED;
        switch (rocket_state)
        {
            case INITIALIZATION:
            {
                if ((now_ms > 1000U) && have_ism6_si && have_bmp_si)
                {
                    rocket_state = READY;
                    ESP_LOGI(TAG, "[STATE] INITIALIZATION -> READY");
                }
                break;
            }
            case READY:
            {
                // (FC serviced the servo neutral-settle + pad relax here.)
                const bool gnss_ready = gnss_started &&
                                        (now_ms > valid_gnss_start_millis + 3000U) &&
                                        have_gnss_si &&
                                        (gnss_latest_si.num_sats >= 4U);
                if (out_ready && gnss_ready)
                {
                    rocket_state = PRELAUNCH;
                    prelaunch_time_millis = now_ms;
                    // Verify pyro continuity on the pad. (#382 comment fix:
                    // a synchronous single read of the rail-fed CONT dividers
                    // (#264) — no arm pulse, no multi-tick sequence.)
                    pyroPrelaunchContTest(now_ms);
                    ESP_LOGI(TAG, "[STATE] READY -> PRELAUNCH");
                }
                else if (kinematics.launch_flag)
                {
                    // #382: launch detected while the GNSS gate was never met
                    // (<4 sats, boost before lock). Pre-fallback the FC sat
                    // in READY for the whole flight: no deployment, no
                    // LANDED, no snapshots. Promote straight to INFLIGHT
                    // through the same entry path; degraded modes are already
                    // handled there (ref-pos freeze skipped without a fix,
                    // ground pressure frozen from the pad).
                    ESP_LOGW(TAG, "[STATE] LAUNCH FROM READY — gates unmet "
                                  "(gnss_ready=%d); promoting to INFLIGHT in "
                                  "degraded mode",
                             (int)gnss_ready);
                    enterInflight(now_ms, "READY");
                }
                break;
            }
            case PRELAUNCH:
            {
                if (kinematics.launch_flag)
                {
                    enterInflight(now_ms, "PRELAUNCH");
                }
                break;
            }
            case INFLIGHT:
            {
                // Logging is triggered by the comms side detecting NSF_LAUNCH
                // in the NonSensorData telem cache — same contract as the OC.

                // Service pyro channels (check triggers, manage fire pulses)
                servicePyroChannels(now_ms);

                // Save flight snapshot at 10 Hz for reboot recovery
                if (now_ms - last_snapshot_ms >= 100U) {
                    last_snapshot_ms = now_ms;
                    saveFlightSnapshot(now_ms);

                    // Emit the flight settings snapshot (#165) on the first
                    // few ticks after launch.  Resent for redundancy against
                    // a dropped frame; values are stable in flight so all
                    // copies are identical and the app reads the first one.
                    if (settings_emit_count < 3) {
                        sendFlightSettings();
                        settings_emit_count++;
                    }
                    // One more copy well clear of the logging-activation
                    // edge: the first three ride launch+0/100/200 ms, exactly
                    // the window where the ring drain is busiest (the pre-#418
                    // races ate all three on the 7/05 V2 flight).  By
                    // launch+5 s the ring is long drained.
                    else if (settings_emit_count == 3 &&
                             now_ms - launch_time_millis >= 5000U) {
                        sendFlightSettings();
                        settings_emit_count++;
                    }
                }

                // Settle window after reboot recovery (FC: servo neutral hold)
                if (reboot_recovery && now_ms >= recovery_settle_end_ms) {
                    reboot_recovery = false;  // keep the telemetry flag (reboot_recovery_telem)
                    ESP_LOGI(TAG, "[RECOVERY] settle complete");
                }

                // --- Burnout detection — runs independently of any control
                // path (#256) ---
                // Apogee detection (TR_KinematicChecks) is gated on
                // burnout_detected, and drogue/main deployment is gated on
                // apogee — on the FC this was deliberately hoisted OUT of
                // `if (servo_enabled)` so recovery never silently dies; the
                // mini keeps it unconditional for the same reason.  Body-X
                // accel (forward axis in FRD) goes negative after motor
                // burnout; the 200 ms lockout + N-sample hysteresis (#197)
                // reject boost vibration.  Logic shared with the host unit
                // test via BurnoutDetector.h.
                const uint32_t t_since_launch_ms = now_ms - launch_time_millis;

                // #259: treat the IMU as unavailable when its last sample is
                // stale.  have_ism6_si latches on first read and never
                // clears, so without this a mid-flight IMU dropout would feed
                // a frozen accel into burnout detection.
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
                    ESP_LOGI(TAG, "[FLIGHT] Burnout detected at T+%lu ms",
                                  (unsigned long)t_since_launch_ms);
                }

                // (FC ran the whole roll/guidance control block here.)

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
                    reboot_recovery = false;
                    reboot_recovery_telem = false;
                    // (FC also relaxed servos, reset guidance and scheduled
                    // the camera stop here.)
                }
                // Retry END_FLIGHT until the enqueue succeeds.  The one-shot
                // cleanup above is latched, but the log ring can transiently
                // saturate around LANDED entry and a single failed enqueue
                // would otherwise lose END_FLIGHT forever (#141).  The comms
                // side closes the session off telem.nonsensor.rocket_state;
                // this frame is the in-stream end-of-flight marker the OC's
                // logs carry, kept so the log format stays identical.
                if (!end_flight_sent)
                {
                    if (mini_link::logFrame(END_FLIGHT, nullptr, 0))
                    {
                        end_flight_sent = true;
                    }
                }
                break;
            }
            case MAG_CALIBRATION:
            {
                // No state-machine transitions out — driven entirely by
                // commands (MAG_CAL_ABORT/ACCEPT/RETRY).  The user is
                // physically tumbling the rocket on the bench, so we
                // explicitly do NOT promote to PRELAUNCH on motion or any
                // other auto-detect.  Issue #96 / #216.
                //
                // #216 — additional inhibitions enforced elsewhere in the
                // loop while we're in this state:
                //   * kinematicChecks(...) is skipped above, so
                //     kinematics.launch_flag cannot latch from the tumble.
                //     MAG_CAL_START also calls kinematics.reset().
                //   * ekf.init(...) is gated against MAG_CALIBRATION so a
                //     tumble can't violate the stationary-at-init assumption.
                //   * servicePyroChannels() only runs from `case INFLIGHT:`,
                //     so automatic pyro firing is impossible here.
                //   * Test commands (PYRO_CONT_TEST, PYRO_FIRE_TEST) gate on
                //     isCommandLockoutState(), true for MAG_CALIBRATION.
                break;
            }
            default:
            {
                break;
            }
        }

        // ==========================================================================
        // SECTION: Simulator re-arm and diagnostics
        // ==========================================================================
        // ---- Re-arm on a NEW sim run, NOT when a sim ends (#317) ----
        // A sim flight that reaches LANDED must STAY landed — terminal,
        // exactly like real hardware — so the sim faithfully validates the
        // post-flight lockout. Re-arm on the RISING edge of sim-active (the
        // deliberate start of a new run).  An explicit Stop is handled in the
        // SIM_STOP_CMD handler instead (#393).
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
                              "acc=%.1f alt=%.1f d_alt=%.2f launch=%d",
                              (int)rocket_state,
                              gnss_started ? 1 : 0,
                              have_gnss_si ? 1 : 0,
                              (unsigned)gnss_latest_si.fix_mode,
                              (unsigned)gnss_latest_si.num_sats,
                              (double)accel_norm,
                              (double)pressure_alt_m,
                              (double)pressure_alt_rate_mps,
                              kinematics.launch_flag ? 1 : 0);
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
        // No roll control on this board — the wire field stays 0 so the log
        // format (and every FC-log consumer) is untouched.
        non_sensor_data.roll_cmd = 0;
        non_sensor_data.baro_alt_rate_dmps = (int16_t)lroundf(kinematics.d_alt_est_ * 10.0f);
        non_sensor_data.flags = 0;
        if (kinematics.alt_landed_flag || (rocket_state == LANDED)) non_sensor_data.flags |= NSF_ALT_LANDED;
        if (kinematics.alt_apogee_flag) non_sensor_data.flags |= NSF_ALT_APOGEE;
        if (kinematics.vel_u_apogee_flag) non_sensor_data.flags |= NSF_VEL_APOGEE;
        if (kinematics.launch_flag || (rocket_state == INFLIGHT)) non_sensor_data.flags |= NSF_LAUNCH;
        if (burnout_detected) non_sensor_data.flags |= NSF_BURNOUT;
        // NSF_GUIDANCE never set — no guidance stack on this board.
        // Apogee detector outputs + master vote (#142/#143) plus the
        // reboot-recovery bit.  NSF2_GUIDANCE_ENABLED never set here.
        non_sensor_data.apogee_flags = 0;
        if (kinematics.gps_apogee_flag)   non_sensor_data.apogee_flags |= NSF2_GPS_APOGEE;
        if (kinematics.pitch_apogee_flag) non_sensor_data.apogee_flags |= NSF2_PITCH_APOGEE;
        if (kinematics.apogee_flag)       non_sensor_data.apogee_flags |= NSF2_MASTER_APOGEE;
        if (reboot_recovery_telem)        non_sensor_data.apogee_flags |= NSF2_REBOOT_RECOVERY;
        if (orient_thrust_mismatch)       non_sensor_data.apogee_flags |= NSF2_ORIENT_THRUST_MISMATCH;
        if (deployment_detected)          non_sensor_data.apogee_flags |= NSF2_DEPLOYED;
        // #474: sticky witness for a stalled sensor loop.  Nonzero drops mean
        // loop_fc() blocked long enough to overflow the ISM6 handoff queue
        // and lose IMU samples.  Read the HW collector (owns the queue +
        // counter, reset once at loop start).
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

        // Sensor health scorecard (#303): pack 2-bit verdicts for the
        // operator's pre-launch go/no-go.  Flight side fills all but battery
        // and storage, which the comms side ORs in before the LoRa relay
        // (same split as FC/OC).
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

            // EKF — the FILTER's own health (#238/#303): BAD until
            // initialized; DEGRADED if diverging or the covariance hasn't
            // converged — stabilizeP() pins the diagonals on divergence,
            // which a finite isHealthy() can't see, so check the worst
            // attitude + velocity variance directly.  OK = converged.
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

                // #508: the gyro bias is FROZEN IN at launch — accel/mag
                // updates are gated off for the flight, so whatever it holds
                // when we leave the pad integrates into attitude for the
                // entire ascent.  Two independent no-go signals: the estimate
                // itself out of spec / clipped (gyroBiasHealthy), or the
                // observability gate TRIPPING on the pad (bad mounting, a
                // steel rail deflecting the mag, a stale hard-iron cal).
                // Amber rather than red: it degrades attitude, it does not by
                // itself endanger recovery.
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
            // but only when that gate is enabled: 0 is a "disabled" sentinel).
            // DEGRADED = 3D fix but marginal; BAD = no 3D fix.
            SensorHealthState gnss_st = SH_BAD;
            if (have_gnss_si && gnss_latest_si.fix_mode >= 3U) {
                const bool sh_hacc_ok = (config::GNSS_MAX_HACC_M <= 0.0f) ||
                                        (gnss_latest_si.horizontal_accuracy < config::GNSS_MAX_HACC_M);
                gnss_st = (gnss_latest_si.num_sats >= config::GNSS_MIN_SATS && sh_hacc_ok)
                          ? SH_OK : SH_DEGRADED;
            }
            sh = shSet(sh, SH_GNSS_SHIFT, gnss_st);

            // Per-channel pyro (#303): only configured channels matter.
            // Carried in sensor_health because LoRaData has no pyro_status,
            // so on a base-station relay this is the only pyro signal.
            for (int i = 0; i < 4; ++i) {
                SensorHealthState pst = SH_NA;              // not configured -> ignored
                if (pyroChEnabled(i)) {
                    pst = !cont_known[i] ? SH_DEGRADED      // configured, not yet tested
                        : (cont_state[i] ? SH_OK : SH_BAD); // continuity present / none
                }
                sh = shSet(sh, SH_PYRO_SHIFT[i], pst);

                // Measured continuity, NOT config-gated — the ground-test
                // answer the gated bits above cannot give for a channel that
                // isn't armed for this flight (OC twin has the same block).
                sh = shSet(sh, SH_PYRO_MEAS_SHIFT[i],
                           !cont_known[i] ? SH_NA
                                          : (cont_state[i] ? SH_OK : SH_BAD));
            }

            // #557: GNSS-absent degraded-flight verdict (distinct from the
            // fix health at SH_GNSS above).
            sh = shSet(sh, SH_GNSS_ABSENT_SHIFT,
                       gnss_absent_flight ? SH_BAD : SH_NA);

            // Battery bits left N/A — the comms side fills them from the INA230.
            non_sensor_data.sensor_health = sh;
        }

        if ((logic_now_us - last_non_sensor_tx_time_us) >= non_sensor_tx_period_us)
        {
            last_non_sensor_tx_time_us = logic_now_us;
            memcpy(non_sensor_data_buffer, &non_sensor_data, SIZE_OF_NON_SENSOR_DATA);
            (void)mini_link::logFrame(NON_SENSOR_MSG,
                                      non_sensor_data_buffer,
                                      (uint8_t)SIZE_OF_NON_SENSOR_DATA);
            // Same 500 Hz gate feeds the comms-side cache — the log format
            // and the telemetry source stay in lockstep by construction.
            telemStore(mini_link::telem.nonsensor, non_sensor_data,
                       mini_link::telem.nonsensor_update_us);
            // (FC nested the guidance telemetry emit here — dropped.)
        }
    }

    // ==========================================================================
    // SECTION: Periodic diagnostics
    // ==========================================================================
    // (FC serviced chirp/beep/LED/camera here — none of that hardware exists;
    // the single board LED is comms-owned.)
    const uint32_t diag_now_ms = time_ms();

    // --- Periodic poll-task timing diagnostics (once per second) ---
    {
        static uint32_t last_poll_diag_ms = 0;
        if ((diag_now_ms - last_poll_diag_ms) >= 1000U)
        {
            last_poll_diag_ms = diag_now_ms;
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
            sensor_collector.resetPollTimingSnapshot();
            // (FC's i2s/i2c counters dropped — the log ring's own stats live
            // on the comms side.)

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
            ESP_LOGI(TAG, "[SENSOR] reads/s: ism6=%lu bmp=%lu bmp_bad=%lu iis=%lu gnss=%lu | bmp_p=%.0f ism6_gz=%.1f drdy_pin=%d",
                          (unsigned long)dbg_ism6_reads,
                          (unsigned long)dbg_bmp_reads,
                          (unsigned long)dbg_bmp_bad_reads,
                          (unsigned long)dbg_iis2mdc_reads,
                          (unsigned long)dbg_gnss_reads,
                          have_bmp_si ? (double)bmp_latest_si.pressure : 0.0,
                          have_ism6_si ? (double)ism6_latest_si.gyro_z : 0.0,
                          gpio_get_level((gpio_num_t)config::ISM6HG256_INT));
            dbg_ism6_reads = 0;
            dbg_bmp_reads = 0;
            dbg_bmp_bad_reads = 0;
            dbg_iis2mdc_reads = 0;
            dbg_gnss_reads = 0;
            // (FC's [MMC DIAG] block dropped — part not fitted.)
        }
    }

    // --- Periodic EKF diagnostics (once per second) ---
    {
        static uint32_t last_ekf_diag_ms = 0;
        if (ekf_initialized && (diag_now_ms - last_ekf_diag_ms) >= 1000U)
        {
            last_ekf_diag_ms = diag_now_ms;

            // Horizontal azimuth of the rocket's body-Z axis in NED. This is
            // *not* a roll in the Euler sense — it's only a useful roll proxy
            // when the rocket is near vertical.  At low pitch this diverges
            // from euler_roll arbitrarily; the divergence is itself a useful
            // informal indicator of how close euler_roll is to gimbal lock.
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

            // Magnetometer field strength and validity (EKF gate: 15–80 µT).
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
            // #508: trips/clips make a laundered bias visible on the console
            // — a healthy pad shows 0/0.
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

// ==========================================================================
// SECTION: Task entry + power-off gate
// ==========================================================================
void flightTask(void* /*arg*/)
{
    // Subscribe this task to the WDT so esp_task_wdt_reset() works.  The
    // 5 s / trigger_panic config was set in flight_setup() (#261).
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());
    for (;;)
    {
        loop_fc();
        esp_task_wdt_reset();
    }
}

bool flightSafeToPowerOff()
{
    // Called from the comms task.  Safe before flight_setup() has ever run:
    // the statics below are at their boot defaults (INITIALIZATION, all pyro
    // Idle), which reads as safe — correct for IDLE mode.
    if (rocket_state == PRELAUNCH || rocket_state == INFLIGHT ||
        rocket_state == MAG_CALIBRATION)
    {
        return false;
    }
    // A mag-cal session can hold state outside the MAG_CALIBRATION rocket
    // state only transiently, but the chip-offset bookkeeping must not be
    // cut mid-write — refuse while any session phase is live.
    if (mag_cal_session_active || mag_cal_verify_active)
    {
        return false;
    }
    bool busy = false;
    portENTER_CRITICAL(&pyro_spinlock);
    for (int i = 0; i < 4; ++i)
    {
        if (pyro_ch[i].state == PyroChState::ArmSettle ||
            pyro_ch[i].state == PyroChState::Firing)
        {
            busy = true;
        }
    }
    if (pyro_arm_pin_state) busy = true;
    portEXIT_CRITICAL(&pyro_spinlock);
    return !busy;
}
