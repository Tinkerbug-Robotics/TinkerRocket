// ==========================================================================
// main.cpp — app shell for the rocket-computer-mini (one S3, both jobs)
// ==========================================================================
// Owns: global objects shared by both sides, the boot path into IDLE, the
// shared buses, and the IDLE/ACTIVE power machine (the out_computer's
// pwr_pin state machine adapted — referenced below as "OC L<n>" into
// projects/out_computer/main/main.cpp).
//
// Mode machine:
//   IDLE   (boot default — the OC also boots rail-off, L5962-5969): PWR_PIN
//          low, BLE advertising, INA230 polling, light sleep enabled, LED
//          slow blink.  Nothing rail-backed is initialized.
//   ACTIVE (BLE cmd 8): exitLowPowerMode → rail up → settle → buses →
//          comms_setup_active() → flight_setup() → flight task.
//   power-off (BLE cmd 8 again): flightSafeToPowerOff() gate → finalize
//          logs → rail down → esp_restart() back into IDLE (#9 policy).
//
// The main task never returns from app_main: it parks in the power machine
// and executes the transitions.  flight_setup()'s synchronous sensor
// bring-up (GNSS probing, tens of seconds worst case) runs on THIS task's
// 16 KB stack (CONFIG_ESP_MAIN_TASK_STACK_SIZE) while the comms task keeps
// BLE alive on core 0.
//
// NOT ported from the OC: the #468 DedupRebootPolicy and its dedup baseline
// reset.  That machinery defends against replayed stale I2S TX descriptors;
// in-process there is no I2S, no replay source and no second clock domain,
// so there is nothing for it to police (report-oc-main-anatomy §10).  The
// host-testable header stays with the out_computer project.

#include <cstring>
#include <cstdio>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_pm.h>
#include <esp_sleep.h>
#include <nvs_flash.h>
#include <esp_ota_ops.h>          // esp_ota_mark_app_valid_cancel_rollback (#8)
#include <esp_partition.h>        // esp_ota_get_running_partition (#8)
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/i2c_master.h"
#include "soc/rtc.h"              // #541: rtc_clk_* for the 32k-crystal second chance
#include "esp_private/esp_clk.h"  // #541: esp_clk_slowclk_cal_set after a late 32k start
#include <esp_system.h>           // esp_reset_reason for the ACTIVE-restore policy
#include <TR_NVS.h>               // Preferences for the ACTIVE-restore flag

#include "config.h"
#include "mini_link.h"
#include "comms.h"
#include "flight.h"

#include <TR_LogToFlash.h>
#include <TR_FlightLog.h>
#include <TR_NandBackend_esp.h>
#include <NandBitmapStore.h>
#include <TR_BLE_To_APP.h>
#include <IRadioLink.h>
#include <LoRaDirectBackend.h>
#include <TR_INA230.h>

// comms.cpp (not in comms.h — power-machine-internal contract): ends any
// open log session and blocks until the flush task has committed it.  False
// = do NOT drop the rail.
bool comms_prepare_power_off();

static const char* TAG = "MINI";

// ==========================================================================
// SECTION: global objects (extern'd by comms.cpp and mini_link.cpp)
// ==========================================================================
TR_LogToFlash logger;

// TR_FlightLog (issue #50) owns the flight-log hot path; TR_LogToFlash keeps
// the ring/flush machinery and the shrunken LFS partition.  Wired together in
// comms_setup_active() (write_sink → flightlog.writeFrame).  (OC L109-118)
tr_flightlog::TR_NandBackend_esp flightlog_backend;
tr_flightlog::TR_FlightLog flightlog;
tr_flightlog::NandBitmapStore flightlog_bitmap_store;  // #398: NAND-backed

TR_BLE_To_APP ble_app("TinkerRocket");

// Radio backend: direct SPI LLCC68 only.  The OC's IRadioLink seam is kept
// (historical `lora_comms` name) so every ported call site is untouched; the
// UART modem backend is not built — board_v1.h pins USE_UART_RADIO_MODEM to
// false and the mini has no daughterboard header.  (OC L415-424)
LoRaDirectBackend lora_direct_backend;
IRadioLink& lora_comms = lora_direct_backend;

// INA230 power monitor.  MINI: on the ONE I2C bus (SEN_SDA/SEN_SCL), shared
// with the rail-backed IIS2MDC — unlike the OC's dedicated I2C_NUM_1.  The
// INA230 itself is always powered; the bus must therefore exist from boot.
TR_INA230 ina230(0x40);
bool ina230_ok = false;

// The one I2C master bus (I2C_NUM_0, sda 21, scl 33).  Handle passed to BOTH
// TR_INA230::begin() (comms_setup_idle) and SensorCollector's external-bus
// ctor parameter (flight_setup) — see the collector seam note.
i2c_master_bus_handle_t shared_i2c_bus = nullptr;

// Rail commanded state — starts OFF (OC L581).  volatile: written by the
// power machine on the main task, read by the comms loop on core 0.
volatile bool pwr_pin_on = false;

// ==========================================================================
// SECTION: OTA boot validation and rollback guard (OC L77-101, verbatim)
// ==========================================================================
// True only between boot and the first successful telemetry-while-connected
// event when we booted PENDING_VERIFY.  If we never get there, the
// bootloader auto-reverts to ota_0 on next boot.  (#8)
static bool g_ota_pending_verify = false;

void maybeMarkOtaValid()
{
    if (!g_ota_pending_verify) return;
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK)
    {
        ESP_LOGW(TAG, "OTA: new image validated, rollback cancelled");
    }
    else
    {
        ESP_LOGE(TAG, "OTA: mark_app_valid_cancel_rollback failed: %s",
                 esp_err_to_name(err));
    }
    g_ota_pending_verify = false;
}

// ==========================================================================
// SECTION: #519 power-config guard — fail the BUILD, not the ammeter
// ==========================================================================
// sdkconfig is generated and OVERRIDES sdkconfig.defaults; a stale checkout
// silently builds current source against the old config (third bite: a
// rebuild shipped MAIN_XTAL as the BT LP clock and idle sat at ~7 mA with no
// warning).  If this fires, do NOT edit sdkconfig — regenerate it:
//   rm sdkconfig && idf.py build
// (OC L1082-1114, verbatim — every symbol is one the idle-power contract
// rides on, and the mini carries the same contract.)
#if defined(ESP_PLATFORM)
#if !defined(CONFIG_PM_ENABLE)
#error "Stale sdkconfig: CONFIG_PM_ENABLE missing — DFS + light sleep compile out entirely. rm sdkconfig && idf.py build (#519)"
#endif
#if !defined(CONFIG_FREERTOS_USE_TICKLESS_IDLE)
#error "Stale sdkconfig: CONFIG_FREERTOS_USE_TICKLESS_IDLE missing — esp_pm_configure(light_sleep_enable) will fail at runtime. rm sdkconfig && idf.py build (#519)"
#endif
#if !defined(CONFIG_RTC_CLK_SRC_EXT_CRYS)
#error "Stale sdkconfig: RTC slow clock is not the 32.768 kHz crystal (CONFIG_RTC_CLK_SRC_EXT_CRYS) — prerequisite for the BT 32k LP clock. rm sdkconfig && idf.py build (#519)"
#endif
#if !defined(CONFIG_BT_CTRL_LPCLK_SEL_EXT_32K_XTAL)
#error "Stale sdkconfig: BT low-power clock is not the 32 kHz crystal (CONFIG_BT_CTRL_LPCLK_SEL_EXT_32K_XTAL) — the controller holds no_light_sleep forever and idle is ~7 mA, not ~1 mA. rm sdkconfig && idf.py build (#519)"
#endif
#if !defined(CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION)
#error "Stale sdkconfig: CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION missing — light sleep will kill the USB console on the bench. rm sdkconfig && idf.py build (#519)"
#endif
#endif  // ESP_PLATFORM

// ==========================================================================
// SECTION: #541 boot USB-serial grace (OC L1127-1222, verbatim; bench-debug
// flag renamed TR_OC_BENCH_DEBUG → TR_MINI_BENCH_DEBUG, see main/CMakeLists)
// ==========================================================================
// Light sleep is held off by exactly two locks: the BT controller's (only on
// a MAIN_XTAL fallback) and the USJ driver's — which is only taken once a
// USB connection is DETECTED.  Right after reset, host re-enumeration takes
// ~1 s; losing that race wedges the USJ until a physical re-plug.  Hold
// ESP_PM_NO_LIGHT_SLEEP for the first kBootUsjGraceMs after light sleep is
// enabled, then release.
#if defined(CONFIG_PM_ENABLE)
static esp_pm_lock_handle_t s_boot_usj_grace_lock = nullptr;
static esp_timer_handle_t   s_boot_usj_grace_timer = nullptr;
static bool                 s_boot_usj_grace_held = false;
static constexpr uint32_t   kBootUsjGraceMs = 10000;

static void bootUsjGraceExpired(void*)
{
#if defined(TR_MINI_BENCH_DEBUG)
    // Bench-debug image: never release.  Light sleep powers down the USB
    // Serial/JTAG peripheral the console rides on (USJ is the ONLY console —
    // UART0's pins are spent, see sdkconfig.defaults), so once this lock
    // drops the board goes permanently silent to a host ~11 s after boot.
    // Holding forever costs the whole #519 saving (~22 mA vs ~1 mA), which
    // is why this is opt-in.  Never fly this image.  (OC #627 bench history)
    ESP_LOGW("PWR", "BENCH DEBUG: holding ESP_PM_NO_LIGHT_SLEEP for the whole "
                    "session so the USJ console stays alive — light sleep is "
                    "DISABLED and idle current is ~22 mA. Do NOT fly this image.");
    (void)s_boot_usj_grace_held;
    return;
#else
    if (s_boot_usj_grace_held)
    {
        s_boot_usj_grace_held = false;
        esp_pm_lock_release(s_boot_usj_grace_lock);
        ESP_LOGI("PWR", "USB-enumeration grace over — light sleep unblocked");
    }
#endif
}

static void holdBootUsjGrace()
{
    if (s_boot_usj_grace_lock == nullptr &&
        esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "usj_boot_grace",
                           &s_boot_usj_grace_lock) != ESP_OK)
    {
        return;  // lock unavailable — no grace, same behavior as before #541
    }
    if (s_boot_usj_grace_timer == nullptr)
    {
        const esp_timer_create_args_t args = {
            .callback = bootUsjGraceExpired,
            .arg = nullptr,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "usj_grace",
            .skip_unhandled_events = true,
        };
        if (esp_timer_create(&args, &s_boot_usj_grace_timer) != ESP_OK)
        {
            return;
        }
    }
    if (!s_boot_usj_grace_held)
    {
        s_boot_usj_grace_held = true;
        esp_pm_lock_acquire(s_boot_usj_grace_lock);
    }
    esp_timer_stop(s_boot_usj_grace_timer);
    esp_timer_start_once(s_boot_usj_grace_timer,
                         (uint64_t)kBootUsjGraceMs * 1000ULL);
#if defined(TR_MINI_BENCH_DEBUG)
    ESP_LOGW("PWR", "BENCH DEBUG build — the USB console will stay alive past "
                    "the %lu ms grace instead of going quiet.",
             (unsigned long)kBootUsjGraceMs);
#else
    ESP_LOGI("PWR", "Light sleep held %lu ms for USB enumeration (#541)",
             (unsigned long)kBootUsjGraceMs);
#endif
}
#endif  // CONFIG_PM_ENABLE

// ==========================================================================
// SECTION: #541 second chance for the 32.768 kHz crystal (OC L1224-1287)
// ==========================================================================
// IDF's boot-time selection gives the crystal ~2 × 92 ms before silently
// falling back to the internal RC — and the BT controller then picks main
// XTAL as its LP clock, quietly reverting the whole #519 power win.  A warm
// tuning-fork crystal can need hundreds of ms.  Must run BEFORE
// ble_app.begin() (the controller samples the slow-clock source once).
static void retry32kCrystal()
{
#if defined(CONFIG_RTC_CLK_SRC_EXT_CRYS)
    if (rtc_clk_slow_src_get() == SOC_RTC_SLOW_CLK_SRC_XTAL32K)
    {
        return;  // crystal came up during boot — nothing to do
    }
    ESP_LOGW("PWR", "32k crystal failed at boot (RTC fell back to internal RC) "
                    "— retrying up to 2 s before BT init (#541)");
    rtc_clk_32k_enable(true);
    uint32_t cal = 0;
    int attempts = 0;
    const int64_t deadline_us = esp_timer_get_time() + 2000000;
    while (esp_timer_get_time() < deadline_us)
    {
        attempts++;
        // Times out (returns 0) in ~100 ms if the crystal isn't oscillating.
        cal = rtc_clk_cal(CLK_CAL_32K_XTAL, CONFIG_RTC_CLK_CAL_CYCLES);
        if (cal != 0) break;
    }
    if (cal != 0)
    {
        rtc_clk_slow_src_set(SOC_RTC_SLOW_CLK_SRC_XTAL32K);
        // Recalibrate against the now-selected source and publish the period
        // (same pair of calls IDF's own select_rtc_slow_clk() makes).
        const uint32_t slow_cal =
            rtc_clk_cal(CLK_CAL_RTC_SLOW, CONFIG_RTC_CLK_CAL_CYCLES);
        if (slow_cal != 0)
        {
            esp_clk_slowclk_cal_set(slow_cal);
        }
        ESP_LOGI("PWR", "32k crystal recovered on attempt %d — RTC and BT LP "
                        "clock will use the crystal after all (#541)", attempts);
    }
    else
    {
        rtc_clk_32k_enable(false);
        ESP_LOGE("PWR", "32k CRYSTAL DID NOT START after +2 s of retries "
                        "(#541 warm-start failure?) — BT LP clock falls back "
                        "to main XTAL; #519 light sleep is INERT this boot. "
                        "Hardware check (drive/load caps) indicated.");
    }
#endif  // CONFIG_RTC_CLK_SRC_EXT_CRYS
}

// ==========================================================================
// SECTION: low-power mode helpers (OC L1289-1355, verbatim)
// ==========================================================================
static void enterLowPowerMode()
{
#if defined(CONFIG_PM_ENABLE)
    // Low-power idle: 80 MHz max (BLE needs 80 MHz APB), 40 MHz min via DFS,
    // light sleep ON (#519 — BT LP clock on the 32k crystal, USJ lock covers
    // the bench console).  MEASUREMENT GOTCHA: with USB plugged in the USJ
    // lock is held and the chip will NOT light-sleep — measure on battery.
    esp_pm_config_t pm_cfg = {};
    pm_cfg.max_freq_mhz = 80;
    pm_cfg.min_freq_mhz = 40;
    pm_cfg.light_sleep_enable = true;
    esp_err_t pm_err = esp_pm_configure(&pm_cfg);
    if (pm_err == ESP_OK)
        ESP_LOGI("PWR", "Low-power mode: 80/40 MHz DFS, light sleep ON "
                        "(held off while USB is connected)");
    else
        ESP_LOGE("PWR", "esp_pm_configure failed: %s", esp_err_to_name(pm_err));

    // #541: give USB enumeration a sleep-free window.  Re-armed on every
    // low-power entry — a fresh host attach on the bench is the same race.
    holdBootUsjGrace();
#endif
}

static void exitLowPowerMode()
{
#if defined(CONFIG_PM_ENABLE)
    // Full speed, no DFS, no light sleep for active mode.  On the mini this
    // lock is also what the whole flight stack's timing budget rides on —
    // see the CPU note in sdkconfig.defaults.
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

// ==========================================================================
// SECTION: shared buses (spec: init once, before either side's setup)
// ==========================================================================
// Called from the power-on transition AFTER the rail is up — driving SCK/
// MOSI into unpowered peripherals would back-feed their ESD diodes (the same
// hazard the power-off pin guard below exists for), so the buses must not be
// initialized while the rail is down.
static void initSwitchedBuses()
{
    static bool done = false;
    if (done) return;
    done = true;

    // SPI2 = sensor bus.  SensorCollector hardcodes SPI2_HOST and its own
    // device adds; the flight side's ported init also calls
    // spi_bus_initialize(SPI2_HOST,...) (FC main L2606) and only LOGS on
    // failure — ESP_ERR_INVALID_STATE from this earlier init is benign there.
    {
        spi_bus_config_t buscfg = {};
        buscfg.sclk_io_num     = config::SENS_SPI_SCK;
        buscfg.mosi_io_num     = config::SENS_SPI_MOSI;
        buscfg.miso_io_num     = config::SENS_SPI_MISO;
        buscfg.quadwp_io_num   = -1;
        buscfg.quadhd_io_num   = -1;
        buscfg.max_transfer_sz = 4096;
        esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "spi_bus_initialize(SPI2 sensors) failed: %s",
                     esp_err_to_name(err));
        }
    }

    // SPI3 = memory + radio bus (NAND via TR_Compat SPIClass, LLCC68 via
    // RadioLib EspHal).  THIS is the one authoritative init — 4096-byte max
    // transfer for the NAND page bursts (EspHal's own fallback init would
    // cap it at 256).  Both downstream consumers tolerate
    // ESP_ERR_INVALID_STATE from their own spi_bus_initialize attempts:
    // EspHal explicitly (EspHal.h L153-160, "shared bus"), SPIClass::begin
    // by simply not claiming ownership (compat.h L198-199).  No arbitration
    // beyond the IDF driver's per-transaction locking is needed — radio
    // transactions are 65 B at 2 Hz, NAND pages are the long ones.
    {
        spi_bus_config_t buscfg = {};
        buscfg.sclk_io_num     = config::MEM_SPI_SCK;
        buscfg.mosi_io_num     = config::MEM_SPI_MOSI;
        buscfg.miso_io_num     = config::MEM_SPI_MISO;
        buscfg.quadwp_io_num   = -1;
        buscfg.quadhd_io_num   = -1;
        buscfg.max_transfer_sz = 4096;
        esp_err_t err = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "spi_bus_initialize(SPI3 mem+radio) failed: %s",
                     esp_err_to_name(err));
        }
    }
}

// ==========================================================================
// SECTION: power machine (the OC's BLE cmd 8 handler, L7048-7159, adapted)
// ==========================================================================
static TaskHandle_t s_main_task = nullptr;
static constexpr uint32_t POWER_REQ_ON  = 1u << 0;
static constexpr uint32_t POWER_REQ_OFF = 1u << 1;

// --- ACTIVE-restore across unexpected resets ---
//
// The two-MCU board survives its most likely mid-flight wedge — the FC task
// WDT — because trigger_panic reboots only the P4 while the OC holds the rail
// and the MRAM snapshot restores state (#261). On the mini every reset is a
// whole-system reset AND the boot default is IDLE (rail off), so without help
// a mid-flight reboot would leave the rocket dark and the snapshot tail-scan
// recovery in flight_setup() would never get the chance to run.
//
// Policy: persist "ACTIVE was commanded" in NVS. Set at the START of
// powerOn() (so a crash during bring-up also retries), cleared only by a
// CLEAN power-off. On any boot that finds the flag set, re-enter ACTIVE
// without waiting for a BLE command. A consecutive-attempt counter caps
// this at MAX_AUTO_RESTORES so a deterministic crash-in-ACTIVE can't loop
// forever with pyro hardware powered; the counter clears after the machine
// stays healthy for AUTO_RESTORE_HEALTHY_MS.
static constexpr const char* PWR_NVS_NS      = "power";
static constexpr const char* PWR_KEY_ACTIVE  = "active";
static constexpr const char* PWR_KEY_AUTO_N  = "auto_n";
static constexpr uint8_t MAX_AUTO_RESTORES   = 3;
static constexpr uint32_t AUTO_RESTORE_HEALTHY_MS = 60000;
static int64_t s_active_since_us = 0;

static void pwrFlagWrite(const char* key, uint8_t v)
{
    Preferences p;
    if (p.begin(PWR_NVS_NS, false))
    {
        p.putUChar(key, v);
        p.end();
    }
}

static uint8_t pwrFlagRead(const char* key)
{
    Preferences p;
    uint8_t v = 0;
    if (p.begin(PWR_NVS_NS, true))
    {
        v = p.getUChar(key, 0);
        p.end();
    }
    return v;
}

// Called by comms.cpp's BLE cmd 8 handler (core 0).  Notifications persist
// until the main task consumes them, so a request during a transition is
// processed right after it completes.
void miniRequestPowerOn()
{
    if (s_main_task) xTaskNotify(s_main_task, POWER_REQ_ON, eSetBits);
}

void miniRequestPowerOff()
{
    if (s_main_task) xTaskNotify(s_main_task, POWER_REQ_OFF, eSetBits);
}

// Power-on transition (OC L7053-7086 adapted: the "peripherals" behind the
// rail are now this board's own sensors/GNSS/radio/NAND, and the FC's setup
// runs in-process right here).
static void powerOn()
{
    pwr_pin_on = true;   // commanded state first (OC L7051)

    // Persist the commanded state BEFORE bring-up: a crash anywhere past this
    // point boots back into an automatic re-entry attempt (see the
    // ACTIVE-restore policy above).
    pwrFlagWrite(PWR_KEY_ACTIVE, 1);

    // Exit low-power mode BEFORE powering peripherals — full CPU, no auto
    // light-sleep during init.  (OC L7057)
    exitLowPowerMode();

    // Quectel rail-off dwell (config note): the rail has been low since this
    // boot began (R84 pulldown through reset + our explicit LOW), so time-
    // since-boot is a floor on the off-dwell.  Cheap insurance against a
    // power-off → instant reconnect → power-on racing the LC86G's required
    // >= 1 s below 100 mV.
    while ((uint32_t)(esp_timer_get_time() / 1000) < config::PERIPH_RAIL_OFF_MIN_MS)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    gpio_set_level((gpio_num_t)config::PWR_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(config::RAIL_SETTLE_MS));  // TPS22810 rise + POR (OC L7068)
    vTaskDelay(1);  // feed watchdog before long init (OC L7069)

    // Buses exactly once, before either side's setup (spec).
    initSwitchedBuses();

    // Comms half: NAND logger + flightlog + LoRa.  Failure is degraded, not
    // fatal — the flight side (pyro safety included) still runs; storage
    // health reports BAD and the operator sees it on the scorecard.
    if (!comms_setup_active())
    {
        ESP_LOGE(TAG, "comms_setup_active failed — continuing DEGRADED "
                      "(no NAND log and/or no radio this boot)");
    }

    // Open the flight→log path as soon as the logger exists so
    // flight_setup()'s own frames (snapshot recovery, cal restores) land.
    // Frames sent before a log session opens are dropped by enqueueFrame —
    // that is the normal pad state, not a fault.
    mini_link::enableLogSink(true);

    // Flight half: synchronous sensor bring-up on THIS task's 16 KB stack
    // (GNSS probing takes tens of seconds worst case; comms task keeps BLE
    // alive meanwhile).
    flight_setup();

    // Flight loop: core 1, top priority, task-WDT subscribed inside.
    xTaskCreatePinnedToCore(flightTask, "flight", 16384, nullptr,
                            configMAX_PRIORITIES - 1, nullptr, 1);

    s_active_since_us = esp_timer_get_time();
    ESP_LOGI(TAG, "ACTIVE: rail up, flight task running");
}

// Power-off transition (OC L7088-7159 adapted).  The OC dropped a second
// processor's rail; here the rail kills our own sensors/GNSS/radio/NAND, so
// the gates come first: never in flight, never with pyro business pending,
// never with unflushed log data (the NAND is ON this rail).
static void powerOff()
{
    // Refuse while PRELAUNCH/INFLIGHT, pyro channels busy, or mag cal in
    // progress — the flight side owns that verdict.
    if (!flightSafeToPowerOff())
    {
        ESP_LOGW(TAG, "Power off REFUSED: flight side reports not safe "
                      "(state/pyro/cal) — leave the rail up");
        return;
    }
    // Finalize any open flight log and wait for the flush task to commit it.
    // Refusing on timeout beats dropping the rail mid-NAND-write.
    if (!comms_prepare_power_off())
    {
        ESP_LOGE(TAG, "Power off REFUSED: log finalization incomplete");
        return;
    }

    // Power off: drop the rail and reset.
    //
    // Surgical teardown is error-prone and leaves residual driver state that
    // breaks the next power-on (#9 — the OC's original finding; on the mini
    // SensorCollector additionally has no deinit at all).  A clean reset
    // gets us to the same IDLE state as cold boot: BLE advertising, all
    // driver state freshly initialised.  The app already handles the brief
    // disconnect/reconnect.  (OC L7090-7107)
    ESP_LOGI("PWR", "Power off: resetting for clean idle state (#9)...");
    // Clean shutdown: clear the ACTIVE-restore flag (and the retry counter)
    // so the coming reset boots into IDLE, not an auto re-entry.
    pwrFlagWrite(PWR_KEY_ACTIVE, 0);
    pwrFlagWrite(PWR_KEY_AUTO_N, 0);
    pwr_pin_on = false;
    gpio_set_level((gpio_num_t)config::PWR_PIN, 0);

    // Drive every signal that goes from the MCU to the switched-rail
    // peripherals LOW so back-feed current can't flow through their input
    // ESD diodes into the now-unpowered VCC while the TPS22810 QOD
    // discharges.  gpio_reset_pin detaches the pad from any SPI/UART matrix
    // routing; level 0 holds it LOW for the brief window before the reset.
    // Post-reset IOs default to high-Z, which is also fine.  (#9, OC
    // L7114-7154; I2S/MRAM entries replaced by this board's sensor-SPI and
    // GNSS-UART pins.)
    //
    // Deliberately NOT in this list:
    //   - PYRO_ARM/PYRO*_FIRE: the flight side holds them actively LOW;
    //     gpio_reset_pin would float them for the window before reset.  They
    //     are in the LOW-only-glitch GPIO1-14 group and their loads are
    //     high-impedance, but there is zero upside to touching pyro pins on
    //     the way down.  They stay driven low until the chip resets.
    //   - PWR_SDA/PWR_SCL: the always-on INA230 lives there.
    //   - PYRO*_CONT: inputs; their pull-ups die with the rail.
    static const gpio_num_t kSwitchedRailPins[] = {
        (gpio_num_t)config::MEM_SPI_SCK,
        (gpio_num_t)config::MEM_SPI_MOSI,
        (gpio_num_t)config::MEM_SPI_MISO,
        (gpio_num_t)config::NAND_CS,
        (gpio_num_t)config::MRAM_CS,        // -1 on this board; skipped below
        (gpio_num_t)config::LORA_CS_PIN,
        (gpio_num_t)config::LORA_RST_PIN,
        (gpio_num_t)config::LORA_DIO1_PIN,
        (gpio_num_t)config::LORA_BUSY_PIN,
        (gpio_num_t)config::LORA_RXEN_PIN,
        (gpio_num_t)config::SENS_SPI_SCK,
        (gpio_num_t)config::SENS_SPI_MOSI,
        (gpio_num_t)config::SENS_SPI_MISO,
        (gpio_num_t)config::ISM6HG256_CS,
        (gpio_num_t)config::BMP585_CS,
        (gpio_num_t)config::GNSS_TX,        // our UART TX into the module
        (gpio_num_t)config::GNSS_RX,        // input; pinned low like the OC's
    };
    for (gpio_num_t pin : kSwitchedRailPins) {
        if ((int)pin < 0) continue;  // peripheral absent on this board (#411)
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(pin, GPIO_FLOATING);
        gpio_set_level(pin, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(100));   // let rail drop, caps discharge (OC L7156)
    esp_restart();
    // not reached — boots back into IDLE with the rail low; the Quectel
    // off-dwell is completed by the PERIPH_RAIL_OFF_MIN_MS gate in powerOn().
}

// ==========================================================================
// SECTION: boot into IDLE (port of setup_oc, OC L5950-6223, split with
// comms_setup_idle which owns the NVS/BLE/INA half)
// ==========================================================================
static void setup_mini()
{
    // Ensure NVS is initialised (OC L5953-5958)
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }

    vTaskDelay(pdMS_TO_TICKS(500));   // (OC L5960)

    s_main_task = xTaskGetCurrentTaskHandle();

    // Rail OFF through boot.  R84 already pulls the TPS22810 enable down
    // through reset; make it explicit before anything else runs.  The mini
    // has no separate GPS rail — GNSS hangs off this same switch.
    gpio_reset_pin((gpio_num_t)config::PWR_PIN);
    gpio_set_direction((gpio_num_t)config::PWR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)config::PWR_PIN, 0);
    pwr_pin_on = false;

    // Pyro FIRE/ARM actively LOW from first boot, not just from power-on.
    // flight_setup()'s initPyroPins() (the full FC-ported sequence) only runs
    // at the ACTIVE transition; until then these pads would sit high-Z on
    // circuits that are hardware-safe (BJT bases, gate pulldown R22) — but a
    // flight computer holds its pyro lines, it doesn't rely on them floating
    // benignly. Order matters: level FIRST so the output register is 0 before
    // the pad becomes an output, and NO gpio_reset_pin here — its transient
    // internal pull-up twitches the DTC123J gate drivers (see the boxed
    // warning at safePyroOutputInit in flight.cpp, commit 421dd63).
    {
        const gpio_num_t pyro_out[] = {
            (gpio_num_t)config::PYRO1_FIRE_PIN, (gpio_num_t)config::PYRO2_FIRE_PIN,
            (gpio_num_t)config::PYRO3_FIRE_PIN, (gpio_num_t)config::PYRO4_FIRE_PIN,
            (gpio_num_t)config::PYRO_ARM_PIN,
        };
        for (gpio_num_t pin : pyro_out)
        {
            gpio_set_level(pin, 0);
            gpio_config_t cfg = {};
            cfg.pin_bit_mask = 1ULL << pin;
            cfg.mode = GPIO_MODE_OUTPUT;
            cfg.pull_up_en = GPIO_PULLUP_DISABLE;
            cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
            cfg.intr_type = GPIO_INTR_DISABLE;
            gpio_config(&cfg);
        }
    }

    ESP_LOGI(TAG, "Starting rocket-computer-mini (low-power mode)...");
    ESP_LOGW(TAG, "[BOARD] pin map: v%d", TR_MINI_BOARD);

    // OTA boot-state check (#8, OC L5974-5996).
    {
        const esp_partition_t* running = esp_ota_get_running_partition();
        esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
        if (running && esp_ota_get_state_partition(running, &state) == ESP_OK)
        {
            if (state == ESP_OTA_IMG_PENDING_VERIFY)
            {
                ESP_LOGW(TAG, "OTA: running on PENDING_VERIFY image (partition '%s' @ 0x%08x); "
                              "rollback armed until first telemetry round-trip",
                         running->label, (unsigned)running->address);
                g_ota_pending_verify = true;
            }
            else
            {
                ESP_LOGI(TAG, "OTA: running partition '%s' state=%d", running->label, (int)state);
            }
        }
    }

    // In-process link before either side starts.
    mini_link::init();

    // The ONE I2C master bus (spec).  Created here, at boot, because the
    // always-on INA230 lives on it; the rail-backed IIS2MDC joins later via
    // SensorCollector's external-bus parameter.  Internal pullups ON: the
    // board's external pull-ups sit on the SWITCHED rail, so in IDLE (rail
    // down) the bus would otherwise float and the INA230 would be
    // unreachable.  400 kHz is per-device (set at device add).
    {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port = I2C_NUM_0;
        bus_cfg.sda_io_num = static_cast<gpio_num_t>(config::PWR_SDA);
        bus_cfg.scl_io_num = static_cast<gpio_num_t>(config::PWR_SCL);
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.flags.enable_internal_pullup = true;

        esp_err_t err = i2c_new_master_bus(&bus_cfg, &shared_i2c_bus);
        if (err != ESP_OK)
        {
            ESP_LOGW("PWR", "I2C bus init failed: %s", esp_err_to_name(err));
            shared_i2c_bus = nullptr;
        }
    }

    // #541: must run BEFORE ble_app.begin() (inside comms_setup_idle) — the
    // BT controller samples the RTC slow-clock source once at init.
    retry32kCrystal();

    // Always-on comms half: identity NVS, BLE up + advertising, INA230.
    comms_setup_idle();

    // Enter low-power mode: 80 MHz, DFS to 40 MHz, light sleep armed.
    enterLowPowerMode();

    ESP_LOGI(TAG, "IDLE: rail off, BLE advertising, waiting for power-on command.");
}

// ==========================================================================
// SECTION: FreeRTOS entry point
// ==========================================================================
extern "C" void app_main(void)
{
    setup_mini();

    // Comms loop: core 0, prio 2, 12 KB — below the sensor-poll tasks it
    // will share the core with in ACTIVE mode (see the task table in the
    // project docs).  NOT task-WDT subscribed; the flight task is.
    xTaskCreatePinnedToCore(commsTask, "comms_loop", 12288, nullptr, 2, nullptr, 0);

    // ACTIVE-restore: a set flag here means the last session was commanded
    // ACTIVE and never cleanly powered off — this reset was unexpected
    // (mid-flight brownout, WDT panic, crash during bring-up). Re-enter
    // ACTIVE so the flight side's snapshot tail-scan recovery gets its
    // chance, unless we're plainly crash-looping.
    if (pwrFlagRead(PWR_KEY_ACTIVE))
    {
        const uint8_t attempts = pwrFlagRead(PWR_KEY_AUTO_N);
        if (attempts >= MAX_AUTO_RESTORES)
        {
            ESP_LOGE(TAG, "ACTIVE-restore SUPPRESSED: %u consecutive attempts "
                          "without a healthy hold — staying IDLE. Power on "
                          "manually and read the coredump.", attempts);
            pwrFlagWrite(PWR_KEY_ACTIVE, 0);
            pwrFlagWrite(PWR_KEY_AUTO_N, 0);
        }
        else
        {
            pwrFlagWrite(PWR_KEY_AUTO_N, attempts + 1);
            ESP_LOGW(TAG, "Unexpected reset while ACTIVE (reason %d) — "
                          "auto re-entering ACTIVE (attempt %u/%u)",
                     (int)esp_reset_reason(), attempts + 1, MAX_AUTO_RESTORES);
            xTaskNotify(s_main_task, POWER_REQ_ON, eSetBits);
        }
    }

    // The main task becomes the power machine: it parks here and executes
    // the (long, blocking) transitions so the comms loop never has to.
    for (;;)
    {
        uint32_t bits = 0;
        // The 10 s timeout exists only to service the healthy-hold clear
        // below; requests still wake it immediately.
        (void)xTaskNotifyWait(0, UINT32_MAX, &bits, pdMS_TO_TICKS(10000));
        if ((bits & POWER_REQ_ON) && !pwr_pin_on)
        {
            powerOn();
        }
        if ((bits & POWER_REQ_OFF) && pwr_pin_on)
        {
            powerOff();   // does not return if it succeeds (esp_restart)
        }
        // ACTIVE held healthy long enough: forgive past auto-restore
        // attempts so the next unexpected reset gets its full retry budget.
        if (pwr_pin_on && s_active_since_us != 0 &&
            (esp_timer_get_time() - s_active_since_us) / 1000 > AUTO_RESTORE_HEALTHY_MS &&
            pwrFlagRead(PWR_KEY_AUTO_N) != 0)
        {
            pwrFlagWrite(PWR_KEY_AUTO_N, 0);
            ESP_LOGI(TAG, "ACTIVE healthy for %u ms — auto-restore budget reset",
                     (unsigned)AUTO_RESTORE_HEALTHY_MS);
        }
    }
}
