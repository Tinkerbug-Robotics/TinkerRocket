#include <compat.h>
#include <TR_NVS.h>
#include <algorithm>

#include <esp_app_desc.h>         // esp_app_get_description for firmware version readback (#8)
#include <esp_log.h>
#include <esp_mac.h>              // esp_efuse_mac_get_default for unit_id
#include <esp_ota_ops.h>          // esp_ota_mark_app_valid_cancel_rollback (#8)
#include <esp_partition.h>        // esp_ota_get_running_partition for rollback gate (#8)
#include <esp_vfs_fat.h>
#include <esp_spiffs.h>
#include <sdmmc_cmd.h>
#include <driver/sdmmc_host.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <esp_vfs_fat_nand.h>
#include <driver/i2c_master.h>

#include <cstdio>
#include <cstdlib>                // setenv() — UTC TZ for the log-rename helper (#168)
#include <cstring>
#include <cerrno>
#include <ctime>                  // mktime / gmtime_r for the rename-on-time-sync path (#168)
#include <string>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>               // fsync() — periodic-flush SD commit (#107)

#include "config.h"
#include "bs_log_policy.h"        // parseSequentialFilename() (#137)
#include "bs_uplink_policy.h"     // mayTransmitUplink() scan gate (#379)
#include "bs_download_policy.h"   // nextChunk()/mayEmitChunk() pacing (#380)

#include <TR_LoRa_Comms.h>
#include <LoRaDirectBackend.h>
#include <UartModemBackend.h>
#include <TR_Sensor_Data_Converter.h>
#include <TR_Coordinates.h>
#include <TR_BLE_To_APP.h>
#include <TR_MAX17205G.h>
#include <TR_MAX17303.h>
#include <TR_MP2672.h>
#include <TR_BQ27Z746.h>
#include <RocketComputerTypes.h>

static const char* TAG = "BS";

// OTA rollback gate (#8). True only between boot and the first successful
// telemetry-while-connected event when we booted PENDING_VERIFY (i.e., a
// fresh OTA image). On first hit we call esp_ota_mark_app_valid_cancel_rollback
// once and clear the flag. If we never get there (panic, hang, BLE init
// failure) the bootloader auto-reverts to ota_0 on next boot.
static bool g_ota_pending_verify = false;

static inline void maybeMarkOtaValid()
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

// Forward declarations
static const char* rocketStateToString(uint8_t state);

// Radio backend seam (#410/#414): direct SPI LLCC68 (V1/V2 boards) or the
// UART radio-daughterboard modem (V3), selected by the board header. The
// reference keeps the historical `lora_comms` name so every call site below
// is untouched; the unused backend is never begun.
static LoRaDirectBackend lora_direct_backend;
static UartModemBackend lora_modem_backend;
static IRadioLink& lora_comms =
    config::USE_UART_RADIO_MODEM
        ? static_cast<IRadioLink&>(lora_modem_backend)
        : static_cast<IRadioLink&>(lora_direct_backend);
static SensorConverter sensor_converter;
static TR_Coordinates coord;
static TR_BLE_To_APP ble_app("TinkerBaseStation");

// NVS persistence for LoRa settings (config.h values are factory defaults)
static Preferences prefs;
static float   lora_freq_mhz  = config::LORA_FREQ_MHZ;
static uint8_t lora_sf         = config::LORA_SF;
static float   lora_bw_khz    = config::LORA_BW_KHZ;
static uint8_t lora_cr         = config::LORA_CR;
static int8_t  lora_tx_power   = config::LORA_TX_POWER_DBM;

// Stats tracking
static uint32_t last_stats_ms = 0;
static uint32_t last_rx_count = 0;
// Last time we heard OUR rocket: bumped only on a network-id-matched beacon
// or telemetry decode (#384). Drives the cmd-10 VERIFYING predicate, silence
// recovery, and hop-liveness — none of which should trust foreign traffic.
static uint32_t last_packet_ms = 0;
// CRC-passing decodes whose SNR was below loraMinValidSnrDb(current_sf)
// — almost certainly noise-floor false positives.  Counted but otherwise
// dropped so they don't update last_packet_ms / hop / recovery state.
// (#90 follow-up; the field-confirmed t=79 -12.8 dB SF8 catch is the
// motivating example.)
static uint32_t lora_low_snr_drops = 0;

// #329: telemetry packets dropped by the network_id filter.  Otherwise fully
// silent (no CSV row, no counter) — a top suspect for a base-station log that
// stays empty while the rocket is clearly transmitting, if the rocket's nid
// drifts from the BS default (forced to 0 at boot, see #136).
static uint32_t lora_netid_mismatch_drops = 0;

// Base station battery (MAX17205G fuel gauge via I2C)
static float bs_voltage = NAN;
static float bs_soc = NAN;
static float bs_current = NAN;
static float bs_temperature = NAN;
static uint32_t last_battery_ms = 0;
static i2c_master_bus_handle_t i2c_bus = nullptr;
static TR_MAX17205G fuel_gauge(config::MAX17205_ADDR);
static TR_BQ27Z746  bq_gauge(config::BQ27Z746_ADDR);
static TR_MAX17303  max17303_gauge(config::MAX17303_ADDR);  // V3; shares 0x36 with MAX17205, split by DevName
static TR_MP2672    pack_charger(config::MP2672_ADDR);      // V3 flight-pack charger (HAS_PACK_CHARGER)
// Which gauge runtime detection found (one firmware image, both PCBs).
enum class GaugeKind { None, MAX17205, BQ27Z746, MAX17303 };
static GaugeKind gauge_kind = GaugeKind::None;
static bool fuel_gauge_present = false;   // true if EITHER gauge is present

// Servo/PID config cache (for BLE readback — mirrors what was sent to rocket)
static int16_t cfg_servo_bias1 = 0;
static int16_t cfg_servo_hz    = 50;
static int16_t cfg_servo_min   = 1000;
static int16_t cfg_servo_max   = 2000;
static float   cfg_pid_kp  = 0.10f;
static float   cfg_pid_ki  = 0.0f;
static float   cfg_pid_kd  = 0.0f;
static float   cfg_pid_min = -20.0f;
static float   cfg_pid_max = 20.0f;
static bool    cfg_servo_enabled = true;

// Device identity (loaded from NVS "identity" namespace)
static char    unit_id_hex[9] = {0};              // last 4 bytes of MAC as "a1b2c3d4"
static char    unit_name[24]  = "TinkerBaseStation"; // default until NVS loads
static uint8_t network_id     = config::DEFAULT_NETWORK_ID;

// LoRa uplink state (BaseStation → OutComputer)
static uint8_t  uplink_buf[40];   // 6-byte header + up to ~33 bytes payload (cmd 15 channel-set push needs 27 at BW=125)
static size_t   uplink_len = 0;
static uint8_t  uplink_retries_left = 0;
static uint32_t uplink_last_tx_ms = 0;
static bool     uplink_pending = false;

// Storage mount state. Preferred backend is SD over SDMMC; if that fails at boot
// we fall back to SPIFFS on internal flash (partitions.csv: "spiffs", ~5 MB).
// The pointer is reassigned to /flash on fallback so all downstream paths
// (logger, BLE file list/delete/download) keep working via VFS unchanged.
static const char* SD_MOUNT_POINT = "/sdcard";
static sdmmc_card_t* sd_card = nullptr;
static bool using_internal_flash = false;
static bool using_external_flash = false;   // V2/V3: FAT on the external SPI flash
static spi_device_handle_t      s_ext_spi = nullptr;
static spi_nand_flash_device_t *s_nand    = nullptr;

// Mount a FAT filesystem (Dhara wear-leveling FTL) on the external SPI NAND
// flash used for logging on the V2/V3 PCBs (M_* pins on SPI3_HOST; LoRa owns
// SPI2_HOST on V2). The chip is a FORESEE F35SQB004G (mfr 0xCD, 512 MB), supported via
// the vendored+patched spi_nand_flash component (components/spi_nand_flash/src/
// devices/nand_foresee.c). On success repoints SD_MOUNT_POINT and sets
// using_external_flash; otherwise returns the error so the caller falls back to
// internal SPIFFS.
static esp_err_t mountExternalFlashFat()
{
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num     = config::FLASH_MOSI;
    buscfg.miso_io_num     = config::FLASH_MISO;
    buscfg.sclk_io_num     = config::FLASH_SCK;
    buscfg.quadwp_io_num   = -1;
    buscfg.quadhd_io_num   = -1;
    buscfg.max_transfer_sz = 4096 + 256;     // page + spare
    esp_err_t e = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (e != ESP_OK) { ESP_LOGW(TAG, "ext-flash SPI bus init failed (0x%x)", (int)e); return e; }

    const uint32_t spi_flags = SPI_DEVICE_HALFDUPLEX;
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 20 * 1000 * 1000;   // conservative for bring-up (chip rated 166 MHz)
    devcfg.mode           = 0;
    devcfg.spics_io_num   = config::FLASH_CS;
    devcfg.queue_size     = 10;
    devcfg.flags          = spi_flags;
    e = spi_bus_add_device(SPI3_HOST, &devcfg, &s_ext_spi);
    if (e != ESP_OK) { ESP_LOGW(TAG, "ext-flash add SPI device failed (0x%x)", (int)e); return e; }

    spi_nand_flash_config_t nand_cfg = {};
    nand_cfg.device_handle = s_ext_spi;
    nand_cfg.io_mode       = SPI_NAND_IO_MODE_SIO;
    nand_cfg.flags         = spi_flags;
    e = spi_nand_flash_init_device(&nand_cfg, &s_nand);
    if (e != ESP_OK) { ESP_LOGW(TAG, "SPI NAND init failed (0x%x) -- unsupported chip or wiring", (int)e); return e; }

    uint32_t blocks = 0, block_sz = 0, page_sz = 0;
    spi_nand_flash_get_block_num(s_nand, &blocks);
    spi_nand_flash_get_block_size(s_nand, &block_sz);
    spi_nand_flash_get_page_size(s_nand, &page_sz);
    ESP_LOGI(TAG, "SPI NAND up: %lu blocks x %lu B (page %lu B) = %llu MB",
             (unsigned long)blocks, (unsigned long)block_sz, (unsigned long)page_sz,
             (unsigned long long)((uint64_t)blocks * block_sz / (1024 * 1024)));

    esp_vfs_fat_mount_config_t mcfg = {};
    mcfg.format_if_mount_failed = true;
    mcfg.max_files              = 5;
    mcfg.allocation_unit_size   = 0;   // FATFS default cluster size
    e = esp_vfs_fat_nand_mount("/extflash", s_nand, &mcfg);
    if (e != ESP_OK) { ESP_LOGW(TAG, "ext-flash FAT mount failed (0x%x)", (int)e); return e; }

    SD_MOUNT_POINT = "/extflash";
    using_external_flash = true;
    ESP_LOGI(TAG, "External-flash FAT mounted at %s", SD_MOUNT_POINT);
    return ESP_OK;
}
static const char* SPIFFS_PARTITION_LABEL = "spiffs";

// CSV logging state
static FILE* log_file = nullptr;
static bool logging_active = false;
static uint32_t log_start_ms = 0;
static uint32_t log_last_write_ms = 0;
static uint32_t log_last_flush_ms = 0;
// Throttle for repeated fopen() attempts when startLogging() fails (#107).
// Auto-start fires on every packet until logging_active is true; without
// a throttle a stuck SD card would spam ESP_LOGE on every downlink.
// Reset to 0 by stopLogging() so the next flight can retry immediately
// rather than waiting out the residual window.
static uint32_t log_last_open_attempt_ms = 0;
// #329: CSV writes/flushes that failed.  fprintf() usually buffers a row and
// returns OK even on a full card, so a full disk surfaces only at fflush/fsync
// (ENOSPC) — counting both makes otherwise-silent row loss visible instead of
// vanishing with no warning. (Declared here so startLogging/stopLogging can
// count header-write and close failures too — the #384 gaps.)
static uint32_t log_write_fail_count = 0;
// State-transition tracking (INFLIGHT safety arm time, last state, boot-edge
// guard, per-rocket freq lock) lives per tracked rocket in
// TrackedRocket::log_state (#381) — one global here made two interleaved
// rockets read as a state transition on every packet, closing and reopening
// the CSV once per interleave cycle. See bs_log_policy.h.
// Sticky inhibit set by a manual cmd 23 stop so auto-start doesn't immediately
// re-open on the next packet (#107).  Cleared on any rocket state change or
// by a manual cmd 23 start, so a real next flight is still captured.
static bool     log_manual_inhibit = false;
static bool     last_known_camera_recording = false;  // Track rocket camera state for idempotent uplink
static bool     last_known_rocket_logging = false;    // Actual rocket logging state from LoRa downlink

// Frequency lock for flight (issue #71).  Set when any tracked rocket
// reports INFLIGHT; cleared on LANDED or READY (matches the rocket-side
// sticky flag).  PRELAUNCH does NOT clear, since a post-flight
// LANDED → PRELAUNCH transition (rocket regains GPS on the ground) would
// otherwise leave the lock stuck on indefinitely.  While set:
//   • Silence recovery is suppressed (we don't hop/scan during flight).
//   • Silence tolerance is extended so momentary SNR dips don't alarm.
//   • Transactional Cmd 10 handler still refuses on the rocket side, but
//     we refuse on the base station side too as belt-and-braces.
// The transition logic is shared with the rocket side via
// computeFreqLockForFlight() in RocketComputerTypes.h.
//
// #381: this is now the AGGREGATE across tracked rockets — locked while any
// fresh rocket's per-rocket lock is latched (bs_log_policy::aggregateFreqLock,
// recomputed after each packet). The old single global was overwritten by
// whichever rocket's packet arrived last, so a second rocket's READY cleared
// the lock mid-flight of the first, once per interleaved packet.
static bool freq_locked_for_flight = false;

// ----------------------------------------------------------------------------
// Per-packet channel-hop state (issues #40 / #41, phase 2a — BS side)
// ----------------------------------------------------------------------------
// The BS is purely reactive: the rocket owns the hop sequence, and the BS
// retunes after each successful RX based on the packet's next_channel_idx.
// hop_active_ tracks whether we're currently following a hopping rocket;
// hop_idx_ is the channel we're (about to be) tuned to for the next RX.
//
// Multi-rocket caveat (deliberately punted to a follow-up): if the BS is
// tracking more than one rocket, only one of them can drive the hop
// sequence at a time.  v2a follows whichever rocket's packet arrived
// most recently, which works fine when there's just one rocket on the
// link — the dominant case for our setup.
static bool     hop_active_       = false;
static uint8_t  hop_idx_          = 0;
static bool     hop_needs_retune_ = false;
static uint32_t hop_last_rx_ms_   = 0;

// Operator override (#106): suppress hopping in PRELAUNCH/INFLIGHT and stay
// on lora_freq_mhz instead.  BS is the authority; the BLE handler for
// cmd 17 updates this flag, persists to NVS, and uplinks the same byte to
// the rocket via LORA_CMD_SET_HOP_DISABLED so both sides stay aligned.
// Diagnostic / link-debugging mode only — fixed-frequency in flight is
// not FHSS-compliant.
static bool     lora_hop_disabled = false;

// ----------------------------------------------------------------------------
// Channel-set state (#40 / #41 phase 3)
// ----------------------------------------------------------------------------
// Selected by `loraSelectChannelSet()` after the pre-launch scan finishes,
// persisted to NVS, and pushed to the rocket via LORA_CMD_CHANNEL_SET.
// `channel_set_bw_khz_` is the BW the mask was generated against — used
// to detect when a cmd-10 BW change invalidates the mask (we revert to
// "no skips" until the user re-runs the scan on the new BW).
//
// Rendezvous freq used to live here too (NVS-stored, scan-selected) but
// that allowed the BS and rocket to drift apart with no recovery path —
// see issue #105.  It's now compile-time hardcoded to the shared
// LORA_FACTORY_RENDEZVOUS_MHZ on both sides.
static uint8_t skip_mask_[LORA_SKIP_MASK_MAX_BYTES] = {0};
static uint8_t skip_mask_n_        = 0;        // 0 = no mask yet
static float   channel_set_bw_khz_ = 0.0f;     // BW the mask was built for

// Mask-drift auto-recovery (#105 follow-up).  Set in the hop RX path
// when the rocket's announced next_channel_idx disagrees with the
// BS-side seq-derived channel — that means BS and OC have diverged
// skip-masks (typically: scan ran, cmd-15 push got displaced by a
// follow-on cmd-10, OC never received the new mask).  serviceMaskDriftRepush
// re-queues cmd 15 with a cooldown so it can't spam.
static bool     chset_drift_repush_pending = false;
static uint32_t chset_drift_repush_last_ms_ = 0;
static constexpr uint32_t CHSET_DRIFT_REPUSH_COOLDOWN_MS = 30000;
static uint32_t chset_drift_repush_count   = 0;  // for [STATS]

// Forward declarations — definitions live further down with the other
// channel-set machinery, but a couple of call sites (cmd-10 commit,
// boot-time NVS load) need them earlier in the file.
static void loadChannelSetFromNvs();
static void invalidateSkipMaskForBwChange();
static void analyzeAndPushFromCachedScan();
static void pushCurrentChannelSet();
static void serviceMaskDriftRepush();

// Auto-acquire + auto-scan state (#136).  Full definitions live just
// before setup_bs(); the enum + state variable are declared here so
// finalizeNoiseScan() can branch on them without forward-decl gymnastics.
enum class AutoAcquireState : uint8_t {
    AWAITING_ROCKET,
    GRACE_DELAY,
    SCANNING,
    COMMITTING,
    DONE,
};
static AutoAcquireState auto_acquire_state = AutoAcquireState::AWAITING_ROCKET;
static void autoAcquireOnScanFinalize();

// Noise-scan pass counter — actual scan machinery lives further down,
// but serviceHeartbeat() needs to see this to skip beats mid-scan.
// Moved up from its original spot near the scan code (#136 follow-up: a
// heartbeat TX during the auto-acquire scan caused subsequent passes
// of the multi-pass scan to refuse to start because tx_ongoing_ was
// still set when startScan() was retried).
static uint8_t scan_passes_remaining_ = 0;

// If we're following a hopping rocket and packets dry up for this long,
// give up and fall back to lora_freq_mhz so the existing silence /
// recovery machinery can take over.  Sized to swallow a handful of
// missed hop windows even at the slowest preset (Long Range = ~2 Hz),
// without holding the radio hostage if the rocket really has vanished.
static constexpr uint32_t HOP_SILENCE_FALLBACK_MS = 3000;

// ----------------------------------------------------------------------------
// Hop-session diagnostics (#105)
// ----------------------------------------------------------------------------
// A "hop session" begins when hop_active_ flips false→true (BS starts
// following the rocket's hop schedule) and ends on either silence-fallback
// or rocket-state-change.  The counters here let the operator see, after a
// flight, how often lock was lost, how long sessions lasted, and how many
// packets the BS *observed* missing during each session — together with
// the seq-derived per-packet loss in the CSV, this is the post-flight
// view that issue #105 was opened to enable.
static uint32_t hop_silence_events_count   = 0;   // lifetime, since boot
static uint32_t hop_session_started_ms     = 0;   // 0 = no active session
static uint32_t hop_session_total_pkts     = 0;   // accepted while hop_active_
static uint32_t hop_session_observed_loss  = 0;   // sum of observed gaps (#105)
static uint32_t lora_total_observed_loss   = 0;   // lifetime, since boot

// Per-rocket tracker (replaces single last_decoded for multi-rocket support)
static constexpr int MAX_TRACKED_ROCKETS = 4;
struct TrackedRocket {
    bool       active = false;
    uint8_t    rocket_id = 0;
    char       unit_name[24] = {0};
    LoRaDataSI last_data = {};
    float      last_rssi = NAN;
    float      last_snr  = NAN;
    double     last_lat_deg = NAN;
    double     last_lon_deg = NAN;
    double     last_alt_m = NAN;
    uint32_t   last_seen_ms = 0;
    // Free-running TX seq from the rocket (#105, widened to 16 bits in
    // proto v4).  -1 = no prior packet ("first contact"); on an
    // implausible forward delta we reset to -1 and treat the next
    // packet as fresh contact — most likely the rocket rebooted and
    // lora_tx_seq restarted at 0.  int32_t to hold the full uint16
    // range plus the -1 sentinel.
    int32_t    last_seq = -1;
    // Per-rocket CSV/freq-lock transition state (#381) — replaces the old
    // globals last_rocket_state / have_seen_first_state / inflight_entry_ms.
    bs_log_policy::RocketLogState log_state = {};
};
static TrackedRocket tracked_rockets[MAX_TRACKED_ROCKETS];
static uint8_t active_rocket_idx = 0;  // Which rocket the BLE telemetry currently shows

/// Find or allocate a tracker slot for a given rocket_id.
/// Returns index, or -1 if all slots full.
static int findOrAllocRocket(uint8_t rid) {
    int free_slot = -1;
    for (int i = 0; i < MAX_TRACKED_ROCKETS; i++) {
        if (tracked_rockets[i].active && tracked_rockets[i].rocket_id == rid)
            return i;
        if (!tracked_rockets[i].active && free_slot < 0)
            free_slot = i;
    }
    if (free_slot >= 0) {
        tracked_rockets[free_slot].active = true;
        tracked_rockets[free_slot].rocket_id = rid;
        tracked_rockets[free_slot].unit_name[0] = '\0';
        tracked_rockets[free_slot].last_seen_ms = millis();
        return free_slot;
    }
    // All slots full — evict oldest
    uint32_t oldest_ms = UINT32_MAX;
    int oldest_idx = 0;
    for (int i = 0; i < MAX_TRACKED_ROCKETS; i++) {
        if (tracked_rockets[i].last_seen_ms < oldest_ms) {
            oldest_ms = tracked_rockets[i].last_seen_ms;
            oldest_idx = i;
        }
    }
    tracked_rockets[oldest_idx].active = true;
    tracked_rockets[oldest_idx].rocket_id = rid;
    tracked_rockets[oldest_idx].unit_name[0] = '\0';
    tracked_rockets[oldest_idx].last_seen_ms = millis();
    tracked_rockets[oldest_idx].log_state = {};  // evicted slot: fresh transitions
    return oldest_idx;
}

// Snapshot the tracked slots for the aggregate log-close / freq-lock
// decisions (#381). Pure data out; bs_log_policy does the reasoning.
static void buildRocketViews(bs_log_policy::RocketView out[MAX_TRACKED_ROCKETS])
{
    for (int i = 0; i < MAX_TRACKED_ROCKETS; i++) {
        out[i].active            = tracked_rockets[i].active;
        out[i].last_seen_ms      = tracked_rockets[i].last_seen_ms;
        out[i].state             = tracked_rockets[i].log_state.last_state;
        out[i].inflight_entry_ms = tracked_rockets[i].log_state.inflight_entry_ms;
        out[i].freq_lock         = tracked_rockets[i].log_state.freq_lock;
    }
}

// State of the most recently heard tracked rocket (0 = INITIALIZATION when
// none). The single-rocket predecessor was the last_rocket_state global —
// "state of whichever packet arrived last" — which matches the hop-follow
// code's one-rocket-drives-the-hop semantics (#40/#41 caveat).
static uint8_t lastSeenRocketState()
{
    uint32_t newest_ms = 0;
    uint8_t  state     = 0;
    for (int i = 0; i < MAX_TRACKED_ROCKETS; i++) {
        if (tracked_rockets[i].active &&
            tracked_rockets[i].log_state.have_seen_first &&
            tracked_rockets[i].last_seen_ms >= newest_ms) {
            newest_ms = tracked_rockets[i].last_seen_ms;
            state     = tracked_rockets[i].log_state.last_state;
        }
    }
    return state;
}
static char log_filename[64] = "";

// Time sync state (UTC time reference from phone via BLE)
static bool time_synced = false;
static uint32_t time_sync_millis = 0;
static uint16_t sync_year = 0;
static uint8_t  sync_month = 0, sync_day = 0;
static uint8_t  sync_hour = 0, sync_minute = 0, sync_second = 0;

// ── MAX17205G fuel gauge helpers ──
// The chip is read/configured through the TR_MAX17205G component.
// updateBattery() fans the latest readings into the global fields used by
// the BLE telemetry builder.

// Keep the new-board (BQ27Z746) battery protection FETs enabled. The gauge
// ships with FET_EN=0 (and a reset reverts to it), so we (re)enable whenever
// FET_EN has dropped and there is no active safety fault -- never overriding a
// real protection event. Also flags the "commanded on but not conducting" case
// (gate-drive / assembly fault, e.g. the EFC8811 open joint we hit). No-op on
// the MAX17205 board. Call right after bq_gauge.update() so the flags are fresh.
static void maintainBatteryFets()
{
    if (gauge_kind != GaugeKind::BQ27Z746) return;
    const TR_BQ27Z746_Data& d = bq_gauge.data();

    if (!d.fet_en)
    {
        // enableFetsIfNeeded() re-reads status and refuses while a fault is
        // active, so this is safe even mid-fault; it logs its own result.
        bq_gauge.enableFetsIfNeeded();
    }
    else if (!d.chg_fet_on || !d.dsg_fet_on)
    {
        static uint8_t throttle = 0;            // rate-limit (~every 15th cycle)
        if ((throttle++ % 15) == 0)
        {
            if (d.safety_active)
                ESP_LOGW(TAG, "BQ27Z746 safety protection holding a FET off (BattStatus=0x%04X)",
                         d.batt_status);
            else
                ESP_LOGW(TAG, "BQ27Z746 FET_EN=1, no fault, but CHG=%d DSG=%d -- FETs not conducting "
                              "(gate-drive/assembly?)", d.chg_fet_on, d.dsg_fet_on);
        }
    }
}

static void updateBattery()
{
    if (!fuel_gauge_present) return;
    if (gauge_kind == GaugeKind::BQ27Z746)
    {
        bq_gauge.update();
        maintainBatteryFets();
        bs_voltage     = bq_gauge.voltage();
        bs_soc         = bq_gauge.soc();
        bs_current     = bq_gauge.current();
        bs_temperature = bq_gauge.temperature();
    }
    else if (gauge_kind == GaugeKind::MAX17303)
    {
        max17303_gauge.update();
        bs_voltage     = max17303_gauge.voltage();
        bs_soc         = max17303_gauge.soc();
        bs_current     = max17303_gauge.current();
        bs_temperature = max17303_gauge.temperature();

        // Protector observability (BQ FET saga lesson). No enable action
        // exists or is needed on this part — the protector runs the FETs —
        // but log WHY the battery path opened, once per transition.
        static bool last_fets_on = true;
        static uint16_t last_prot = 0;
        const bool fets_on = max17303_gauge.fetsOn();
        const uint16_t prot = max17303_gauge.protStatus();
        if (fets_on != last_fets_on || (prot != last_prot && prot != 0))
        {
            if (!fets_on || prot != 0)
                ESP_LOGW(TAG, "MAX17303 protector: FETs %s, ProtStatus=0x%04X",
                         fets_on ? "on" : "OFF", prot);
            else
                ESP_LOGI(TAG, "MAX17303 protector: FETs back on, faults clear");
        }
        last_fets_on = fets_on;
        last_prot = prot;
    }
    else
    {
        fuel_gauge.update();
        bs_voltage     = fuel_gauge.voltage();
        bs_soc         = fuel_gauge.soc();
        bs_current     = fuel_gauge.current();
        bs_temperature = fuel_gauge.temperature();
    }
}

// ============================================================================
// CSV Logging
// ============================================================================

// Days in each month (non-leap / leap year)
static const uint8_t days_in_month[2][12] = {
    {31,28,31,30,31,30,31,31,30,31,30,31},  // non-leap
    {31,29,31,30,31,30,31,31,30,31,30,31}   // leap
};

static bool isLeapYear(uint16_t y)
{
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

/// Compute current UTC time from the phone-synced reference + elapsed millis
static void getCurrentTime(uint16_t& year, uint8_t& month, uint8_t& day,
                           uint8_t& hour, uint8_t& minute, uint8_t& second)
{
    uint32_t elapsed_s = (millis() - time_sync_millis) / 1000;

    // Start from sync values
    year   = sync_year;
    month  = sync_month;
    day    = sync_day;
    hour   = sync_hour;
    minute = sync_minute;
    second = sync_second;

    // Add elapsed seconds with carry
    uint32_t total_sec = (uint32_t)second + elapsed_s;
    second = total_sec % 60;

    uint32_t total_min = (uint32_t)minute + total_sec / 60;
    minute = total_min % 60;

    uint32_t total_hr = (uint32_t)hour + total_min / 60;
    hour = total_hr % 24;

    uint32_t extra_days = total_hr / 24;
    while (extra_days > 0)
    {
        uint8_t dim = days_in_month[isLeapYear(year) ? 1 : 0][month - 1];
        if (day + extra_days <= dim)
        {
            day += extra_days;
            extra_days = 0;
        }
        else
        {
            extra_days -= (dim - day + 1);
            day = 1;
            month++;
            if (month > 12)
            {
                month = 1;
                year++;
            }
        }
    }
}

// Sync time → UTC seconds since the Unix epoch.  Relies on TZ=UTC0 being
// set in setup_bs() so mktime() interprets the broken-down time as UTC
// rather than local.  Returns 0 if no sync has happened yet.
static time_t syncedEpoch()
{
    if (!time_synced) return 0;
    struct tm tm_sync = {};
    tm_sync.tm_year = (int)sync_year - 1900;
    tm_sync.tm_mon  = (int)sync_month - 1;
    tm_sync.tm_mday = (int)sync_day;
    tm_sync.tm_hour = (int)sync_hour;
    tm_sync.tm_min  = (int)sync_minute;
    tm_sync.tm_sec  = (int)sync_second;
    return mktime(&tm_sync);
}

// Build "<mount>/lora_YYYYMMDD_HHMMSS.csv" for the given UTC epoch, walking
// _2/_3/.. suffixes on collision until a free path is found.  Mirrors the
// inline collision loop in startLogging() — extracted so the rename-on-
// time-sync path (#168) reuses the same naming + collision rules.
static void buildTimestampedLogPathForEpoch(time_t epoch,
                                             char* out_path, size_t out_len)
{
    struct tm tm_buf = {};
    gmtime_r(&epoch, &tm_buf);
    char basename[40];
    snprintf(basename, sizeof(basename),
             "lora_%04d%02d%02d_%02d%02d%02d.csv",
             tm_buf.tm_year + 1900, tm_buf.tm_mon + 1, tm_buf.tm_mday,
             tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec);
    snprintf(out_path, out_len, "%s/%s", SD_MOUNT_POINT, basename);

    struct stat st;
    if (stat(out_path, &st) != 0) return;

    char base_no_ext[40];
    const size_t blen = strlen(basename);
    const size_t copy = (blen >= 4) ? (blen - 4) : blen;  // strip ".csv"
    memcpy(base_no_ext, basename, copy);
    base_no_ext[copy] = '\0';
    for (int suffix = 2; suffix < 100; suffix++)
    {
        snprintf(out_path, out_len, "%s/%s_%d.csv",
                 SD_MOUNT_POINT, base_no_ext, suffix);
        if (stat(out_path, &st) != 0) return;
    }
}

static uint16_t findNextFileNumber()
{
    uint16_t max_num = 0;
    DIR* dir = opendir(SD_MOUNT_POINT);
    if (!dir) return 1;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        // #137: use the strict parser so timestamped siblings (e.g.
        // lora_20260509_164143.csv) don't get matched as "lora_NNN.csv"
        // with the leading digits truncated to the low 16 bits (= 9885).
        // Pre-fix, that quirk made the BS pick lora_9886.csv after every
        // no-time-sync boot whose SD already held timestamped flights.
        uint16_t num = 0;
        if (bs_log_policy::parseSequentialFilename(entry->d_name, num))
        {
            if (num > max_num) max_num = num;
        }
    }
    closedir(dir);
    return max_num + 1;
}

static void startLogging()
{
    if (logging_active)
    {
        fclose(log_file);
        log_file = nullptr;
        ESP_LOGI(TAG, "[LOG] Closed previous log: %s", log_filename);
    }

    // Build a filename relative to the mount point (log_filename stores VFS path)
    char basename[40];
    if (time_synced)
    {
        // Use timestamped filename (matches rocket's flight_YYYYMMDD_HHMMSS naming)
        uint16_t y; uint8_t mo, d, h, mi, s;
        getCurrentTime(y, mo, d, h, mi, s);
        snprintf(basename, sizeof(basename),
                 "lora_%04u%02u%02u_%02u%02u%02u.csv",
                 y, mo, d, h, mi, s);
    }
    else
    {
        // Fallback to sequential numbering if no time sync
        uint16_t num = findNextFileNumber();
        snprintf(basename, sizeof(basename), "lora_%03u.csv", num);
    }
    snprintf(log_filename, sizeof(log_filename), "%s/%s", SD_MOUNT_POINT, basename);

    // The LANDED-transition close (#107) closes and lets the next packet
    // auto-restart, which can land within the same wall-clock second as
    // the previous open — `fopen("w")` on the same path would truncate
    // the just-finished flight.  If the timestamped name already exists,
    // append _2/_3/.. so each flight stays on its own file.  Sequential
    // names are already unique because findNextFileNumber returns max+1.
    if (time_synced)
    {
        struct stat st;
        if (stat(log_filename, &st) == 0)
        {
            char base_no_ext[40];
            const size_t blen = strlen(basename);
            const size_t copy = (blen >= 4) ? (blen - 4) : blen;  // strip ".csv"
            memcpy(base_no_ext, basename, copy);
            base_no_ext[copy] = '\0';
            for (int suffix = 2; suffix < 100; suffix++)
            {
                snprintf(log_filename, sizeof(log_filename), "%s/%s_%d.csv",
                         SD_MOUNT_POINT, base_no_ext, suffix);
                if (stat(log_filename, &st) != 0) break;
            }
        }
    }

    log_file = fopen(log_filename, "w");
    if (!log_file)
    {
        ESP_LOGE(TAG, "[LOG] Failed to open %s for writing! errno=%d (%s)",
                 log_filename, errno, strerror(errno));
        logging_active = false;
        return;
    }

    // Write CSV header.  The trailing block (next_ch..event) was added
    // for #105 lock-loss diagnostics: per-packet hop target, the freq the
    // BS was actually tuned to, the rocket's free-running seq and the
    // BS-derived gap to the last RX, plus an `event` column populated
    // for non-telemetry rows (hop_active / hop_inactive / hop_silence).
    // rocket_id (#381) attributes each telemetry row to its source rocket
    // now that one file can interleave several; empty on EVENT rows (those
    // are station-level, not per-rocket). All downstream consumers key
    // columns by header name, so the trailing add is non-breaking.
    int hdr_written = fprintf(log_file, "time_ms,state,num_sats,pdop,lat,lon,alt_m,h_acc,"
                      "acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,"
                      "pressure_alt,alt_rate,max_alt,max_speed,"
                      "voltage,current,soc,roll,pitch,yaw,speed,"
                      "launch,vel_apo,alt_apo,landed,rssi,snr,"
                      "next_ch,rx_freq_mhz,seq,gap,event,rocket_id\n");
    if (hdr_written <= 0)
    {
        // #384 (#329 residual): a full/wedged card at open used to fail the
        // header silently; every later name-keyed consumer then sees a
        // headerless CSV.
        log_write_fail_count++;
        ESP_LOGW(TAG, "[LOG] header fprintf() failed (ret=%d, errno=%d %s)",
                 hdr_written, errno, strerror(errno));
    }

    logging_active = true;
    log_start_ms = millis();
    log_last_write_ms = millis();
    log_last_flush_ms = millis();

    ESP_LOGI(TAG, "[LOG] Started logging: %s", log_filename);
}

static void stopLogging()
{
    if (!logging_active) return;

    if (log_file)
    {
        // #384 (#329 residual): fclose flushes the final buffered rows — the
        // LANDED-close tail of a flight. A card-full here was fully silent.
        if (fclose(log_file) != 0)
        {
            log_write_fail_count++;
            ESP_LOGW(TAG, "[LOG] fclose() failed — final buffered rows may be lost (errno=%d %s)",
                     errno, strerror(errno));
        }
        log_file = nullptr;
    }
    logging_active = false;
    log_last_open_attempt_ms = 0;  // allow the next packet to retry immediately (#107)

    // Disarm every rocket's INFLIGHT safety timer with the file it bounds
    // (#381) — a timer that expired while another rocket kept the log open
    // must not instantly close the NEXT session's file.
    for (int i = 0; i < MAX_TRACKED_ROCKETS; i++) {
        tracked_rockets[i].log_state.inflight_entry_ms = 0;
    }

    ESP_LOGI(TAG, "[LOG] Closed log: %s", log_filename);
}

// Called from the BLE time-sync command (#9) right after the sync clock is
// established.  If the currently-open log was opened pre-sync and therefore
// carries a sequential `lora_NNN.csv` name, rename it on disk to its proper
// `lora_YYYYMMDD_HHMMSS.csv` form using the wall-clock at which the file
// actually opened (computed as sync_time minus elapsed millis since open).
// Safe no-op when there's no open log, time isn't synced, or the file is
// already timestamped.  This is the root-cause fix for #168 — the 5/17/26
// test day produced `lora_002.csv` / `lora_003.csv` / `lora_004.csv` halves
// of three flights alongside their timestamped post-landing remnants
// because iOS BLE time-sync landed within a few seconds of each launch.
static void renameOpenLogIfSequential()
{
    if (!logging_active || !time_synced || log_file == nullptr) return;

    const char* slash = strrchr(log_filename, '/');
    const char* basename = slash ? (slash + 1) : log_filename;
    uint16_t parsed_num = 0;
    if (!bs_log_policy::parseSequentialFilename(basename, parsed_num)) {
        // Already timestamped (or shape we don't manage) — nothing to do.
        return;
    }

    const time_t epoch_sync = syncedEpoch();
    if (epoch_sync <= 0) return;
    const uint32_t back_s = (time_sync_millis > log_start_ms)
                              ? (time_sync_millis - log_start_ms) / 1000U
                              : 0U;
    const time_t epoch_open = epoch_sync - (time_t)back_s;

    char new_path[64];
    buildTimestampedLogPathForEpoch(epoch_open, new_path, sizeof(new_path));

    char old_path[64];
    strncpy(old_path, log_filename, sizeof(old_path) - 1);
    old_path[sizeof(old_path) - 1] = '\0';

    // Flush + close, rename on disk, then reopen append.  fflush alone
    // wouldn't be enough — FATFS needs the file closed before rename().
    fflush(log_file);
    if (!using_internal_flash) fsync(fileno(log_file));
    fclose(log_file);
    log_file = nullptr;

    if (rename(old_path, new_path) != 0)
    {
        ESP_LOGW(TAG, "[LOG] rename %s -> %s failed (errno=%d %s); reopening original",
                 old_path, new_path, errno, strerror(errno));
        log_file = fopen(old_path, "a");
        if (!log_file)
        {
            ESP_LOGE(TAG, "[LOG] Could not reopen %s after failed rename — logging stopped",
                     old_path);
            logging_active = false;
        }
        return;
    }

    log_file = fopen(new_path, "a");
    if (!log_file)
    {
        ESP_LOGE(TAG, "[LOG] Could not open %s after rename — logging stopped",
                 new_path);
        logging_active = false;
        return;
    }
    strncpy(log_filename, new_path, sizeof(log_filename) - 1);
    log_filename[sizeof(log_filename) - 1] = '\0';
    log_last_flush_ms = millis();
    ESP_LOGI(TAG, "[LOG] Time-sync arrived; renamed sequential log -> %s", log_filename);
}

static uint32_t log_write_count = 0;  // Tracks calls for periodic flash check


// Query the active log filesystem (SPIFFS or FAT) for total/used bytes.
// Returns true if a backend is mounted and the query succeeded.
static bool bsQueryStorage(uint64_t& total, uint64_t& used)
{
    total = 0; used = 0;
    if (using_internal_flash)
    {
        size_t t = 0, u = 0;
        if (esp_spiffs_info(SPIFFS_PARTITION_LABEL, &t, &u) != ESP_OK) return false;
        total = t; used = u;
        return true;
    }
    FATFS* fs = nullptr;
    DWORD free_clust = 0;
    if (f_getfree("0:", &free_clust, &fs) != FR_OK || fs == nullptr) return false;
    // Use the real sector size (fs->ssize), NOT a hardcoded 512: the external
    // NAND mounts FAT with 4096-byte sectors (CONFIG_FATFS_SECTOR_4096), so a
    // hardcoded 512 under-reported total/free/used by 8x (512 MB read as ~57 MB).
    total = (uint64_t)(fs->n_fatent - 2) * fs->csize * fs->ssize;
    const uint64_t free_bytes = (uint64_t)free_clust * fs->csize * fs->ssize;
    used = (total > free_bytes) ? (total - free_bytes) : 0;
    return true;
}

static void logLoRaPacket(const LoRaDataSI& data, float rssi, float snr,
                          double lat, double lon, double alt,
                          float rx_freq_mhz, int32_t observed_gap)
{
    if (!logging_active) return;

    // Periodic storage usage check (every 100 writes)
    if (++log_write_count % 100 == 0)
    {
        uint64_t total = 0, used = 0;
        bsQueryStorage(total, used);
        if (total > 0 && used > (total * 9 / 10))
        {
            ESP_LOGW(TAG, "[LOG] %s nearly full! %llu/%llu bytes (%.0f%%)",
                     using_internal_flash ? "Internal flash" : using_external_flash ? "External NAND" : "SD card",
                     (unsigned long long)used, (unsigned long long)total,
                     (double)used * 100.0 / (double)total);
        }
    }

    uint32_t time_ms = millis() - log_start_ms;

    int written = fprintf(log_file, "%lu,%s,%u,%.1f,%.7f,%.7f,%.1f,%.1f,"
                    "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,"
                    "%.1f,%.1f,%.1f,%.1f,"
                    "%.2f,%.0f,%.1f,%.1f,%.1f,%.1f,%.1f,"
                    "%d,%d,%d,%d,%.0f,%.1f,"
                    "%u,%.3f,%u,%d,,%u\n",  // empty `event`, then rocket_id (#381)
                    (unsigned long)time_ms,
                    rocketStateToString(data.rocket_state),
                    (unsigned)data.num_sats,
                    (double)data.pdop,
                    lat, lon, alt,
                    (double)data.horizontal_accuracy,
                    (double)data.acc_x, (double)data.acc_y, (double)data.acc_z,
                    (double)data.gyro_x, (double)data.gyro_y, (double)data.gyro_z,
                    (double)data.pressure_alt, (double)data.altitude_rate,
                    (double)data.max_alt, (double)data.max_speed,
                    (double)data.voltage, (double)data.current, (double)data.soc,
                    (double)data.roll, (double)data.pitch, (double)data.yaw,
                    (double)data.speed,
                    data.launch_flag ? 1 : 0,
                    data.vel_u_apogee_flag ? 1 : 0,
                    data.alt_apogee_flag ? 1 : 0,
                    data.alt_landed_flag ? 1 : 0,
                    (double)rssi, (double)snr,
                    (unsigned)data.next_channel_idx, (double)rx_freq_mhz,
                    (unsigned)data.seq, (int)observed_gap,
                    (unsigned)data.rocket_id);

    if (written <= 0)
    {
        log_write_fail_count++;
        ESP_LOGW(TAG, "[LOG] fprintf() failed (ret=%d, errno=%d %s)",
                 written, errno, strerror(errno));
    }

    log_last_write_ms = millis();
}

// ----------------------------------------------------------------------------
// CSV hop-event row (#105)
// ----------------------------------------------------------------------------
// Mirrors the column layout of logLoRaPacket() so a single pandas read_csv()
// pass can ingest both — telemetry rows and event rows in chronological
// order.  All telemetry-typed fields are left empty (",,,") so the event
// stands out and the parsers can drop it with a simple `state == "EVENT"`
// filter.  The `event` column carries a free-text description; downstream
// scripts grep this to plot session timelines and loss histograms.
//
// Skipped when logging_active=false so we don't open a file just to record
// an event with no surrounding telemetry.  Hop transitions still go to
// ESP_LOG via the existing logging at the call site, so nothing is lost.
static void logHopEvent(const char* event_str, float rx_freq_mhz)
{
    if (!logging_active || log_file == nullptr) return;
    uint32_t time_ms = millis() - log_start_ms;
    int written = fprintf(log_file,
                    // 30 telemetry-typed fields blank, then next_ch=- (255 sentinel)
                    // and rx_freq_mhz/seq/gap minimal so the event row still
                    // tells you which freq the BS was sitting on at the moment.
                    "%lu,EVENT,,,,,,,,,,,,,"   // time_ms, state, num_sats..gyro_z (14 cols)
                    ",,,,"                    // pressure_alt, alt_rate, max_alt, max_speed
                    ",,,,,,,"                 // voltage..speed (7)
                    ",,,,,,"                  // launch, vel_apo, alt_apo, landed, rssi, snr
                    "%u,%.3f,,,"              // next_ch=255 sentinel, rx_freq_mhz, seq=, gap=
                    "%s,\n",                  // event, rocket_id empty (station-level row, #381)
                    (unsigned long)time_ms,
                    (unsigned)LORA_NEXT_CH_NO_HOP,
                    (double)rx_freq_mhz,
                    event_str);
    if (written <= 0) {
        log_write_fail_count++;  // #384: event rows now count toward #329 stats
        ESP_LOGW(TAG, "[LOG] fprintf(event) failed");
    }
    log_last_write_ms = millis();
}

// Returns the radio's currently-tuned RX frequency for the hop diagnostics
// CSV — i.e. the channel the BS was sitting on when this packet arrived
// (or when this hop event fired).  Single source of truth so callers don't
// have to re-derive lora_freq_mhz vs loraChannelMHz(...) inline.
static inline float currentRxFreqMHz()
{
    if (hop_active_) {
        const float f = loraChannelMHz(lora_bw_khz, hop_idx_);
        if (f > 0.0f) return f;
    }
    return lora_freq_mhz;
}

// ============================================================================
// BLE File Command Handlers
// ============================================================================

// NOTE: All SD card operations (log, list, delete) run on the main loop task.
// No mutex needed as long as this guarantee holds. If BLE callbacks move to
// a separate task, add a mutex around all SD card access.
static void handleFileListCommand()
{
    uint8_t page = ble_app.getFileListPage();

    // Collect all CSV files
    struct FileEntry { char name[32]; uint32_t size; };
    FileEntry entries[64];
    size_t total = 0;

    DIR* dir = opendir(SD_MOUNT_POINT);
    if (dir)
    {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr && total < 64)
        {
            // Skip directories
            if (entry->d_type == DT_DIR) continue;

            const char* fname = entry->d_name;

            // Derive the active basename from log_filename for comparison
            const char* active_basename = log_filename;
            // log_filename is e.g. "/sdcard/lora_001.csv", strip mount prefix
            if (strncmp(active_basename, SD_MOUNT_POINT, strlen(SD_MOUNT_POINT)) == 0)
                active_basename += strlen(SD_MOUNT_POINT) + 1; // skip "/sdcard/"

            // Skip the currently active log file
            if (logging_active && strcmp(fname, active_basename) == 0)
                continue;

            strncpy(entries[total].name, fname, 31);
            entries[total].name[31] = '\0';

            // Get file size via stat
            char fullpath[64];
            snprintf(fullpath, sizeof(fullpath), "%s/%s", SD_MOUNT_POINT, fname);
            struct stat st;
            entries[total].size = (stat(fullpath, &st) == 0) ? (uint32_t)st.st_size : 0;
            total++;
        }
        closedir(dir);
    }

    // Sort descending by name (newest = highest number first)
    if (total > 1)
    {
        qsort(entries, total, sizeof(FileEntry),
              [](const void* a, const void* b) -> int {
                  return strcmp(((FileEntry*)b)->name, ((FileEntry*)a)->name);
              });
    }

    // Paginate
    size_t start = (size_t)page * config::FILES_PER_PAGE;
    if (start > total) start = total;
    size_t end = start + config::FILES_PER_PAGE;
    if (end > total) end = total;

    // Build JSON
    String json = "[";
    for (size_t i = start; i < end; ++i)
    {
        if (i > start) json += ",";
        json += "{\"name\":\"";
        json += entries[i].name;
        json += "\",\"size\":";
        json += std::to_string(entries[i].size);
        json += "}";
    }
    json += "]";

    ble_app.sendFileList(json);
    ESP_LOGI(TAG, "[BLE] Sent file list page %u: %u files (total %u)",
             (unsigned)page, (unsigned)(end - start), (unsigned)total);
}

static void handleDeleteCommand()
{
    String filename = ble_app.getDeleteFilename();
    if (filename.length() == 0) return;

    // Don't delete the active log file — derive basename from log_filename
    const char* active = log_filename;
    if (strncmp(active, SD_MOUNT_POINT, strlen(SD_MOUNT_POINT)) == 0)
        active += strlen(SD_MOUNT_POINT) + 1;
    if (logging_active && filename == active)
    {
        ESP_LOGW(TAG, "[BLE] Cannot delete active log: %s", filename.c_str());
        return;
    }

    char path[64];
    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, filename.c_str());
    if (remove(path) == 0)
    {
        ESP_LOGI(TAG, "[BLE] Deleted: %s", filename.c_str());
    }
    else
    {
        ESP_LOGE(TAG, "[BLE] Delete failed: %s", filename.c_str());
    }

    // Send updated file list (page 0)
    handleFileListCommand();
}

// ---------------------------------------------------------------------------
// Cooperative BLE file download (#380).
//
// The download used to run to completion inside the command dispatch — a
// synchronous fread/sendFileChunk/delay(15 ms) loop on the single bs_loop
// task. A 120 KB CSV blocked the loop ~11 s (a multi-MB log for minutes),
// starving telemetry RX, uplink retries, heartbeats, the LoRa transaction /
// recovery state machines, and CSV flushes; a >5 min download silence-closed
// the active log. It is now a state machine serviced once per loop iteration
// (mirroring coord_scan_state_): startDownload() opens the file and captures
// state, serviceDownload() emits at most one chunk per pass, gated on the
// same 15 ms wall-time spacing the BLE notify path needs to drain its mbufs
// (bs_download_policy::mayEmitChunk — the loop runs ~1 ms, so pacing must be
// wall-time, not loop cadence). Throughput is unchanged (~11 KB/s); the loop
// keeps servicing everything else between chunks.
static FILE*    dl_file_          = nullptr;
static uint32_t dl_file_size_     = 0;
static uint32_t dl_offset_        = 0;
static uint32_t dl_last_chunk_ms_ = 0;
static char     dl_name_[40]      = {0};  // for logging only

static void finishDownload(const char* outcome)
{
    ESP_LOGI(TAG, "[BLE] Download %s: %s (%lu of %lu bytes sent)",
             outcome, dl_name_, (unsigned long)dl_offset_,
             (unsigned long)dl_file_size_);
    if (dl_file_ != nullptr) fclose(dl_file_);
    dl_file_          = nullptr;
    dl_file_size_     = 0;
    dl_offset_        = 0;
    dl_last_chunk_ms_ = 0;
    dl_name_[0]       = '\0';
}

static void startDownload()
{
    // getDownloadFilename() clears on read — capture everything now; the
    // per-iteration service below must not touch the BLE pending slot.
    String filename = ble_app.getDownloadFilename();
    if (filename.length() == 0) return;

    if (dl_file_ != nullptr)
    {
        // A new request supersedes an in-flight transfer (same "latest wins"
        // semantics as the uplink slot). The app only runs one download UI.
        ESP_LOGW(TAG, "[BLE] New download request supersedes active transfer");
        finishDownload("superseded");
    }

    char path[64];
    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, filename.c_str());
    FILE* f = fopen(path, "r");
    if (!f)
    {
        ESP_LOGE(TAG, "[BLE] Download failed, file not found: %s", filename.c_str());
        ble_app.sendFileChunk(0, nullptr, 0, true);  // Send empty EOF
        return;
    }

    // Size measured at open bounds the whole transfer — a still-growing active
    // log downloads the bytes that existed at request time and terminates.
    fseek(f, 0, SEEK_END);
    dl_file_size_ = (uint32_t)ftell(f);
    fseek(f, 0, SEEK_SET);

    dl_file_          = f;
    dl_offset_        = 0;
    dl_last_chunk_ms_ = 0;
    snprintf(dl_name_, sizeof(dl_name_), "%s", filename.c_str());

    ESP_LOGI(TAG, "[BLE] Starting download: %s (%lu bytes)",
             dl_name_, (unsigned long)dl_file_size_);
}

static void serviceDownload()
{
    if (dl_file_ == nullptr) return;

    if (!ble_app.isConnected())
    {
        // Abort without an EOF chunk, matching the old behavior — the peer is
        // gone, and a reconnecting app re-requests from scratch.
        ESP_LOGW(TAG, "[BLE] Disconnected during download, aborting");
        finishDownload("aborted");
        return;
    }

    const uint32_t now = millis();
    if (!bs_download_policy::mayEmitChunk(now, dl_last_chunk_ms_,
                                          config::BLE_CHUNK_DELAY_MS))
    {
        return;  // BLE notify path still draining — try next iteration
    }

    const auto plan = bs_download_policy::nextChunk(
        dl_file_size_, dl_offset_, config::BLE_FILE_CHUNK_SIZE);

    uint8_t chunk_buf[config::BLE_FILE_CHUNK_SIZE];
    size_t got = 0;
    if (plan.read_len > 0)
    {
        got = fread(chunk_buf, 1, plan.read_len, dl_file_);
        if (got == 0)
        {
            // Unexpected short read (SD error / file truncated underneath us).
            // Terminate with an empty EOF chunk so the app doesn't hang — the
            // old loop just stopped here without ever flagging EOF.
            ESP_LOGE(TAG, "[BLE] Download read failed at offset %lu of %s",
                     (unsigned long)dl_offset_, dl_name_);
            ble_app.sendFileChunk(dl_offset_, nullptr, 0, true);
            finishDownload("failed");
            return;
        }
    }

    const bool eof = plan.eof && (got == plan.read_len);
    ble_app.sendFileChunk(dl_offset_, got ? chunk_buf : nullptr, got, eof);
    dl_offset_ += got;
    dl_last_chunk_ms_ = now;

    if (eof) finishDownload("complete");
}

// ============================================================================
// Telemetry Helpers
// ============================================================================

static const char* rocketStateToString(uint8_t state)
{
    switch (state)
    {
        case 0:  return "INIT";
        case 1:  return "READY";
        case 2:  return "PRELAUNCH";
        case 3:  return "INFLIGHT";
        case 4:  return "LANDED";
        case 5:  return "MAG_CAL";
        default: return "UNKNOWN";
    }
}

static void buildBLETelemetry(const LoRaDataSI& lora, float rssi, float snr,
                              double lat_deg, double lon_deg, double gnss_alt_m,
                              TR_BLE_To_APP::TelemetryData& out)
{
    memset(&out, 0, sizeof(out));

    // Power
    out.soc = lora.soc;
    out.current = lora.current;
    out.voltage = lora.voltage;

    // GPS (pre-computed lat/lon)
    out.latitude = lat_deg;
    out.longitude = lon_deg;
    out.gdop = lora.pdop;
    out.num_sats = (int)lora.num_sats;

    // Sensor health scorecard bitfield (#303) — relayed straight from the
    // FC/OC LoRa downlink; iOS unpacks the 2-bit-per-sensor states.
    out.sensor_health = lora.sensor_health;

    // State
    out.state = rocketStateToString(lora.rocket_state);

    // Camera recording (from LoRa downlink flags)
    out.camera_recording = lora.camera_recording;
    // Rocket logging state (actual, from LoRa downlink)
    out.logging_active = last_known_rocket_logging;
    // Surface the BS log basename as a heartbeat so the operator can
    // confirm logging is live before each flight (#107).  The rocket-side
    // filename isn't shipped over LoRa, so this slot is otherwise unused
    // when the iOS app is connected via the base station.
    if (logging_active)
    {
        const char* slash = strrchr(log_filename, '/');
        out.active_file = slash ? slash + 1 : log_filename;
    }
    else
    {
        out.active_file = "";
    }

    // Base station's own CSV logging state (shown as separate indicator)
    out.bs_logging_active = logging_active;
    // Seconds remaining until silence-timeout fires (#137 follow-up).  When
    // the log is open, iOS renders a countdown next to the Base Stn Log
    // badge so the operator can see the auto-close approaching during long
    // post-flight idles.  We sit-clamp at 0 rather than going negative
    // because the close runs out of band with this packet build.
    if (logging_active)
    {
        const uint32_t age_ms   = millis() - log_last_write_ms;
        const uint32_t remain_ms = (age_ms >= config::LOG_SILENCE_TIMEOUT_MS)
                                 ? 0
                                 : (config::LOG_SILENCE_TIMEOUT_MS - age_ms);
        out.bs_log_silence_remaining_s = (uint16_t)(remain_ms / 1000U);
    }
    else
    {
        out.bs_log_silence_remaining_s = 0xFFFF;  // sentinel — omit from JSON
    }

    // Data rates -- not applicable for base station
    out.rx_kbs = NAN;
    out.wr_kbs = NAN;
    out.frames_rx = 0;
    out.frames_drop = 0;

    // Performance
    out.max_alt_m = lora.max_alt;
    out.max_speed_mps = lora.max_speed;

    // Altitude
    out.pressure_alt = lora.pressure_alt;
    out.altitude_rate = lora.altitude_rate;
    out.gnss_alt = isnan(gnss_alt_m) ? NAN : (float)gnss_alt_m;

    // IMU -- low-g only (high-g not in LoRa packet)
    out.low_g_x = lora.acc_x;
    out.low_g_y = lora.acc_y;
    out.low_g_z = lora.acc_z;
    out.high_g_x = NAN;
    out.high_g_y = NAN;
    out.high_g_z = NAN;
    out.gyro_x = lora.gyro_x;
    out.gyro_y = lora.gyro_y;
    out.gyro_z = lora.gyro_z;

    // Attitude
    out.roll  = lora.roll;
    out.pitch = lora.pitch;
    out.yaw   = lora.yaw;
    out.q0    = lora.q0;
    out.q1    = lora.q1;
    out.q2    = lora.q2;
    out.q3    = lora.q3;

    // LoRa signal quality
    out.rssi = rssi;
    out.snr = snr;

    // Base station battery (local measurement)
    out.bs_soc = bs_soc;
    out.bs_voltage = bs_voltage;
    out.bs_current = bs_current;

    // Flight event flags (from LoRa packet)
    out.launch_flag       = lora.launch_flag;
    out.vel_u_apogee_flag = lora.vel_u_apogee_flag;
    out.alt_apogee_flag   = lora.alt_apogee_flag;
    out.alt_landed_flag   = lora.alt_landed_flag;

    // Source rocket identity (for app-side multi-rocket demux)
    out.source_rocket_id  = lora.rocket_id;
    out.source_unit_name  = nullptr;  // Caller sets from tracker if available
}

static uint32_t last_telem_print_ms = 0;
static uint8_t  last_printed_state  = 0xFF;

static void printTelemetry(const LoRaDataSI& data, float rssi, float snr,
                           double lat_deg, double lon_deg, double alt_m)
{
    const uint32_t now = millis();
    const bool state_changed = (data.rocket_state != last_printed_state);
    const bool timer_elapsed = (now - last_telem_print_ms >= 5000);

    if (!state_changed && !timer_elapsed) return;

    last_telem_print_ms = now;
    last_printed_state  = data.rocket_state;

    // next_channel_idx is logged so we can verify the framing byte is
    // crossing the wire correctly before phase 2 starts using it.  0xFF
    // ("--") is the phase-1 sentinel meaning "no hop"; anything else is
    // already a hop intent.
    char hop_str[8];
    if (data.next_channel_idx == LORA_NEXT_CH_NO_HOP)
        snprintf(hop_str, sizeof(hop_str), "--");
    else
        snprintf(hop_str, sizeof(hop_str), "%u", (unsigned)data.next_channel_idx);

    ESP_LOGI(TAG, "[RX] %s | alt=%.0fm spd=%.1fm/s | %.0fdBm SNR=%.1f | sats=%u | %.2fV %.0f%% | nextCh=%s",
             rocketStateToString(data.rocket_state),
             (double)data.pressure_alt,
             (double)data.max_speed,
             (double)rssi,
             (double)snr,
             (unsigned)data.num_sats,
             (double)data.voltage,
             (double)data.soc,
             hop_str);
}

static void printStats()
{
    if (!config::DEBUG)
    {
        return;
    }

    const uint32_t now = millis();
    if ((now - last_stats_ms) < config::STATS_PERIOD_MS)
    {
        return;
    }
    const uint32_t dt = now - last_stats_ms;
    last_stats_ms = now;

    TR_LoRa_Comms::Stats ls = {};
    lora_comms.getStats(ls);

    const uint32_t rx_delta = ls.rx_count - last_rx_count;
    const float rx_hz = (dt > 0) ? ((float)rx_delta * 1000.0f / (float)dt) : 0.0f;
    last_rx_count = ls.rx_count;

    // "Last pkt N ms ago" was misleading when N == 0 (looked like "just
    // received" but actually meant "never received").  Print "never" in
    // that case so a fresh-BS-no-rocket scenario is unambiguous in logs.
    char last_pkt_str[24];
    if (last_packet_ms > 0)
    {
        snprintf(last_pkt_str, sizeof(last_pkt_str), "%lu ms ago",
                 (unsigned long)(now - last_packet_ms));
    }
    else
    {
        snprintf(last_pkt_str, sizeof(last_pkt_str), "never");
    }

    ESP_LOGI(TAG, "[STATS] RX: %lu pkts (%.1f Hz) | CRC fail: %lu | len drop: %lu | low-SNR drop: %lu | netid drop: %lu | log wr-fail: %lu | ISR: %lu | rx_mode: %d | TX wdog: %lu | Last RSSI: %.0f dBm SNR: %.1f dB | Last pkt %s",
             (unsigned long)ls.rx_count,
             (double)rx_hz,
             (unsigned long)ls.rx_crc_fail,
             (unsigned long)ls.rx_len_drop,
             (unsigned long)lora_low_snr_drops,
             (unsigned long)lora_netid_mismatch_drops,
             (unsigned long)log_write_fail_count,
             (unsigned long)ls.isr_count,
             (int)ls.rx_mode,
             (unsigned long)ls.tx_watchdog_fires,
             (double)ls.last_rssi,
             (double)ls.last_snr,
             last_pkt_str);

    // Hop diagnostics (#105).  hop_active=Y/N is the live link state; the
    // session counters reflect the *current* session and reset on each
    // hop_inactive / hop_silence event, so a "0 / 0" reading mid-flight
    // means we just lost lock.  lifetime totals let the operator spot a
    // run with abnormally many silence events at a glance.  drift_repush
    // counts auto re-pushes of cmd 15 in response to BS/OC mask divergence.
    ESP_LOGI(TAG, "[STATS] HOP: active=%c idx=%u session(pkts=%lu loss=%lu) lifetime(silence=%lu loss=%lu drift_repush=%lu)",
             hop_active_ ? 'Y' : 'N',
             (unsigned)hop_idx_,
             (unsigned long)hop_session_total_pkts,
             (unsigned long)hop_session_observed_loss,
             (unsigned long)hop_silence_events_count,
             (unsigned long)lora_total_observed_loss,
             (unsigned long)chset_drift_repush_count);
}

// ============================================================================
// Config readback: send cached config to app over BLE
// ============================================================================

static void sendCurrentConfig()
{
    char buf[384];
    int n = snprintf(buf, sizeof(buf),
             "{\"type\":\"config\""
             ",\"sb1\":%d,\"shz\":%d,\"smn\":%d,\"smx\":%d"
             ",\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f"
             ",\"pmn\":%.1f,\"pmx\":%.1f"
             ",\"sen\":%s"
             ",\"lf\":%.1f,\"lsf\":%u,\"lbw\":%.0f,\"lcr\":%u,\"lpw\":%d"
             ",\"lhd\":%s}",
             (int)cfg_servo_bias1, (int)cfg_servo_hz,
             (int)cfg_servo_min, (int)cfg_servo_max,
             (double)cfg_pid_kp, (double)cfg_pid_ki, (double)cfg_pid_kd,
             (double)cfg_pid_min, (double)cfg_pid_max,
             cfg_servo_enabled ? "true" : "false",
             (double)lora_freq_mhz, (unsigned)lora_sf,
             (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power,
             lora_hop_disabled ? "true" : "false");
    if (n < 0 || (size_t)n >= sizeof(buf)) {
        ESP_LOGW(TAG, "[CFG] Config JSON truncated! (%d bytes, buf=%u)", n, (unsigned)sizeof(buf));
    }
    String j(buf);
    ble_app.sendConfigJSON(j);
    ESP_LOGI(TAG, "[CFG] Sent config readback to app");

    delay(50);

    // Message 2: device identity ("config_identity" type)
    const esp_app_desc_t* app_desc = esp_app_get_description();
    const char* fw_ver = (app_desc && app_desc->version[0]) ? app_desc->version : "unknown";
    char id_buf[192];
    snprintf(id_buf, sizeof(id_buf),
             "{\"type\":\"config_identity\""
             ",\"uid\":\"%s\""
             ",\"un\":\"%s\""
             ",\"nid\":%u"
             ",\"dt\":\"%s\""
             ",\"fw\":\"%s\"}",
             unit_id_hex, unit_name,
             (unsigned)network_id,
             config::DEVICE_TYPE,
             fw_ver);
    String id_json(id_buf);
    ble_app.sendConfigJSON(id_json);
    ESP_LOGI(TAG, "[CFG] Sent identity readback (%u bytes)", (unsigned)id_json.length());
}

// #285: cacheServoConfig() / cachePIDConfig() were removed along with the
// servo/PID config-relay handlers that were their only callers.  The cfg_servo_*
// / cfg_pid_* globals remain as a read-only readback (loaded from NVS at boot,
// surfaced via sendCurrentConfig) but are no longer written from a relay path,
// consistent with the base station never configuring the rocket.

// ============================================================================
// LoRa Uplink (relay BLE commands to OutComputer)
// ============================================================================

/// Build an uplink packet with routing header.
/// target_rid: destination rocket_id (0xFF = broadcast to all rockets in network)
/// retries: number of TX attempts.  Defaults to config::UPLINK_RETRIES (8) for
///   reliability on important commands; heartbeat-style traffic can pass a
///   smaller value to keep airtime low.
static void buildUplinkPacket(uint8_t cmd, const uint8_t* payload, size_t payload_len,
                              uint8_t target_rid = 0xFF,
                              uint8_t retries = config::UPLINK_RETRIES)
{
    // #286: reject (do NOT truncate) an oversized payload.  Truncating would
    // send a malformed command with a wrong-but-consistent length and no error
    // surfaced.  Checked first so a rejected command also leaves any
    // already-pending uplink untouched.
    constexpr size_t kMaxUplinkPayload = sizeof(uplink_buf) - 6 - 1;  // header + 1 B slack = 33
    if (payload_len > kMaxUplinkPayload)
    {
        ESP_LOGE(TAG, "[UPLINK] cmd=%u payload %u B exceeds max %u — REJECTED, not queued",
                 cmd, (unsigned)payload_len, (unsigned)kMaxUplinkPayload);
        return;
    }

    if (uplink_pending)
    {
        ESP_LOGW(TAG, "[UPLINK] Discarding pending cmd=%u, replacing with cmd=%u",
                 uplink_buf[4], cmd);
    }
    // Uplink format v2: [0xCA][network_id][target_rid][next_channel_idx][cmd][len][payload...]
    uplink_buf[0] = config::UPLINK_SYNC_BYTE;  // 0xCA
    uplink_buf[1] = network_id;
    uplink_buf[2] = target_rid;
    uplink_buf[3] = LORA_NEXT_CH_NO_HOP;       // phase 1: hop logic deferred
    uplink_buf[4] = cmd;
    uplink_buf[5] = (uint8_t)payload_len;
    if (payload_len > 0 && payload != nullptr)
    {
        memcpy(&uplink_buf[6], payload, payload_len);
    }
    uplink_len = 6 + payload_len;
    uplink_retries_left = retries;
    uplink_pending = true;
    uplink_last_tx_ms = 0;
    ESP_LOGI(TAG, "[UPLINK] Queued cmd=%u -> rid=%u payload=%u bytes, %u retries",
             cmd, target_rid, (unsigned)payload_len, (unsigned)retries);
}

static void serviceUplink()
{
    if (!uplink_pending || uplink_retries_left == 0)
    {
        if (uplink_pending)
        {
            // #285: the uplink is blind fire-and-retry — the rocket sends no
            // command ACK, so completing all retries is NOT proof of delivery.
            // Log it honestly so the operator does not read retry completion as
            // success.  (Safety-relevant rocket state — pyro armed/fired,
            // camera, logging — is confirmed separately via the rocket's own
            // telemetry echo, not via this path.)
            ESP_LOGW(TAG, "[UPLINK] cmd=%u: blind retries exhausted — delivery "
                          "UNCONFIRMED (rocket sends no ACK)",
                     (unsigned)uplink_buf[4]);
            // service() already auto-entered RX after the last TX.  Do NOT call
            // startReceive() here: it would reset rx_done_ and drop any downlink
            // packet that arrived between the TX completion and this point.
            uplink_pending = false;
        }
        return;
    }

    lora_comms.pollDio1();  // Catch TX-done if ISR didn't fire (critical in drain loop)
    lora_comms.service();   // Complete any pending TX

    const uint32_t now = millis();
    if (uplink_last_tx_ms != 0 &&
        (now - uplink_last_tx_ms) < config::UPLINK_RETRY_INTERVAL_MS)
    {
        return;  // Wait between retries
    }

    // #379: gate the transmit on radio-ready AND not-scanning. A TX during a
    // frequency scan goes out on the scan's dwell channel (silently missing the
    // rocket, no ACK to catch it) and corrupts that pass's RSSI — so defer.
    // Returning here before send() preserves uplink_retries_left, so the command
    // fires once the scan completes instead of being lost. Mirrors
    // serviceHeartbeat's scan_passes_remaining_ gate.
    if (!bs_uplink_policy::mayTransmitUplink(uplink_pending, uplink_retries_left,
                                             scan_passes_remaining_,
                                             lora_comms.canSend()))
    {
        return;  // radio busy, or a scan owns the channel — retry next pass
    }

    if (lora_comms.send(uplink_buf, uplink_len))
    {
        uplink_retries_left--;
        uplink_last_tx_ms = now;
        // #285: "blind" + "unconfirmed" so the log is not mistaken for an ACK.
        ESP_LOGI(TAG, "[UPLINK] blind TX, %u attempt(s) left (unconfirmed, no ACK)",
                 uplink_retries_left);
    }
    else
    {
        ESP_LOGW(TAG, "[UPLINK] send() failed, will retry");
    }
}

// ============================================================================
// LoRa Config Transaction (issue #71)
// ============================================================================
// Transactional commit of a new LoRa config (freq / bw / sf / cr / pwr).
// Sequence:
//   1. On BLE Cmd 10 the base station takes a rollback snapshot, then queues
//      a broadcast uplink of inner Cmd 10 to every rocket on the OLD channel.
//   2. Once the uplink retries finish (or the upper-bound timer fires), the
//      base station switches its radio to the NEW channel and listens for
//      proof of life (any rocket beacon or telemetry packet bumps
//      last_packet_ms).
//   3. On proof → commit to NVS + send BLE readback.
//      On timeout → reconfigure back to OLD, send BLE readback with OLD
//      values, and leave NVS untouched.  The silence-recovery layer is
//      then responsible for healing any residual divergence.
// The handler is a non-blocking state machine serviced from loop_bs();
// the whole transaction takes ~3 s under normal conditions.

enum class LoRaTxnState : uint8_t {
    IDLE,
    RELAYING,       // Uplink retries in flight on OLD channel
    VERIFYING,      // Listening for rocket beacon/telem on NEW channel
    ROLLING_BACK,   // Restoring OLD channel after verify timed out
};

static LoRaTxnState lora_txn_state = LoRaTxnState::IDLE;

// Target config
static float   txn_new_freq = 0.0f, txn_new_bw = 0.0f;
static uint8_t txn_new_sf = 0, txn_new_cr = 0;
static int8_t  txn_new_pwr = 0;
// Rollback snapshot
static float   txn_old_freq = 0.0f, txn_old_bw = 0.0f;
static uint8_t txn_old_sf = 0, txn_old_cr = 0;
static int8_t  txn_old_pwr = 0;

static uint32_t txn_phase_start_ms = 0;
// last_packet_ms value at the moment we switched to NEW.  Any increase
// during the verify window proves the rocket joined us on NEW.
static uint32_t txn_verify_baseline_packet_ms = 0;

static constexpr uint32_t TXN_VERIFY_WINDOW_MS = 5000;  // Listen 5 s on NEW
static constexpr uint32_t TXN_MAX_RELAY_MS     = 3000;  // Upper bound on relay phase

/// Begin a transactional LoRa reconfigure.  Returns false (and leaves the
/// BS config unchanged, BLE readback sent) if the preconditions fail.
static bool startLoRaTransaction(float new_freq, float new_bw,
                                 uint8_t new_sf, uint8_t new_cr, int8_t new_pwr)
{
    if (freq_locked_for_flight || hop_active_)
    {
        ESP_LOGW(TAG, "[TXN] Refused: %s",
                 freq_locked_for_flight ? "frequency locked for flight"
                                        : "channel hopping active");
        sendCurrentConfig();
        return false;
    }
    if (lora_txn_state != LoRaTxnState::IDLE)
    {
        ESP_LOGW(TAG, "[TXN] Refused: transaction already in progress");
        sendCurrentConfig();
        return false;
    }

    // Take rollback snapshot BEFORE queuing the uplink, so we can always
    // restore whatever was active when the transaction started.
    txn_old_freq = lora_freq_mhz;
    txn_old_bw   = lora_bw_khz;
    txn_old_sf   = lora_sf;
    txn_old_cr   = lora_cr;
    txn_old_pwr  = lora_tx_power;

    txn_new_freq = new_freq;
    txn_new_bw   = new_bw;
    txn_new_sf   = new_sf;
    txn_new_cr   = new_cr;
    txn_new_pwr  = new_pwr;

    // Broadcast uplink to every rocket in the network on the OLD channel.
    // buildUplinkPacket queues + runs 8 retries on its own (~800 ms total).
    uint8_t loraPayload[11];
    memcpy(loraPayload + 0, &new_freq, 4);
    memcpy(loraPayload + 4, &new_bw,   4);
    loraPayload[8]  = new_sf;
    loraPayload[9]  = new_cr;
    loraPayload[10] = (uint8_t)new_pwr;
    buildUplinkPacket(10, loraPayload, 11, /* target_rid = broadcast */ 0xFF);

    lora_txn_state = LoRaTxnState::RELAYING;
    txn_phase_start_ms = millis();
    ESP_LOGI(TAG, "[TXN] Start: relay %.2f MHz SF%u BW%.0f on OLD, then verify on NEW",
             (double)new_freq, (unsigned)new_sf, (double)new_bw);
    return true;
}

/// Run the transaction state machine — call from loop_bs() every iteration.
static void serviceLoRaTransaction()
{
    switch (lora_txn_state)
    {
        case LoRaTxnState::IDLE:
            return;

        case LoRaTxnState::RELAYING:
        {
            const uint32_t now = millis();
            // serviceUplink() clears uplink_pending once the last retry has
            // fired; at that point the rocket has either joined NEW or not.
            // TXN_MAX_RELAY_MS is a safety net in case uplink state is stuck.
            const bool relay_done   = !uplink_pending;
            const bool relay_timeout = (now - txn_phase_start_ms) > TXN_MAX_RELAY_MS;
            if (!relay_done && !relay_timeout) return;

            if (!lora_comms.reconfigure(txn_new_freq, txn_new_sf, txn_new_bw,
                                        txn_new_cr, txn_new_pwr))
            {
                ESP_LOGE(TAG, "[TXN] reconfigure(NEW) failed — rolling back");
                lora_txn_state = LoRaTxnState::ROLLING_BACK;
                txn_phase_start_ms = now;
                return;
            }
            lora_comms.startReceive();  // listen on NEW freq

            txn_verify_baseline_packet_ms = last_packet_ms;
            lora_txn_state = LoRaTxnState::VERIFYING;
            txn_phase_start_ms = now;
            ESP_LOGI(TAG, "[TXN] On NEW %.2f MHz; verifying for %u ms",
                     (double)txn_new_freq, (unsigned)TXN_VERIFY_WINDOW_MS);
            break;
        }

        case LoRaTxnState::VERIFYING:
        {
            const uint32_t now = millis();
            // Any packet (beacon or telem) received after we switched proves
            // the rocket joined us on NEW.  The RX path unconditionally
            // bumps last_packet_ms on every successful receive.
            if (last_packet_ms > txn_verify_baseline_packet_ms)
            {
                // COMMIT: radio already on NEW; persist to NVS + update
                // runtime vars so the rest of the firmware sees the new
                // config in readbacks.
                const float old_bw = lora_bw_khz;
                lora_freq_mhz = txn_new_freq;
                lora_bw_khz   = txn_new_bw;
                lora_sf       = txn_new_sf;
                lora_cr       = txn_new_cr;
                lora_tx_power = txn_new_pwr;

                prefs.begin("lora", false);
                prefs.putFloat("freq",  lora_freq_mhz);
                prefs.putFloat("bw",    lora_bw_khz);
                prefs.putUChar("sf",    lora_sf);
                prefs.putUChar("cr",    lora_cr);
                prefs.putChar("txpwr",  lora_tx_power);
                prefs.end();

                // BW change invalidates the skip-mask (it's sized for
                // the old hop table).  Rendezvous freq survives.
                if (old_bw != lora_bw_khz)
                {
                    invalidateSkipMaskForBwChange();
                }

                // Re-push the channel-set if we have a recent scan
                // (#40 / #41 phase 3).  The original post-scan cmd-15
                // would have been displaced from the uplink queue by
                // this very cmd-10 transaction; re-pushing here is the
                // simplest way to guarantee delivery, and it also
                // re-sizes the skip-mask if BW just changed.  Inert if
                // no scan has happened this session.
                analyzeAndPushFromCachedScan();

                ESP_LOGI(TAG, "[TXN] COMMIT: heard rocket on NEW %.2f MHz, saved",
                         (double)lora_freq_mhz);
                lora_txn_state = LoRaTxnState::IDLE;
                sendCurrentConfig();
                return;
            }
            if ((now - txn_phase_start_ms) >= TXN_VERIFY_WINDOW_MS)
            {
                ESP_LOGW(TAG, "[TXN] TIMEOUT: no rocket on NEW %.2f MHz, rolling back",
                         (double)txn_new_freq);
                lora_txn_state = LoRaTxnState::ROLLING_BACK;
                txn_phase_start_ms = now;
            }
            break;
        }

        case LoRaTxnState::ROLLING_BACK:
        {
            // Restore the OLD config.  If this fails the radio may be stuck;
            // log loudly and return to IDLE — silence recovery is the last
            // line of defence.
            if (lora_comms.reconfigure(txn_old_freq, txn_old_sf, txn_old_bw,
                                       txn_old_cr, txn_old_pwr))
            {
                lora_freq_mhz = txn_old_freq;
                lora_bw_khz   = txn_old_bw;
                lora_sf       = txn_old_sf;
                lora_cr       = txn_old_cr;
                lora_tx_power = txn_old_pwr;
                lora_comms.startReceive();
                ESP_LOGI(TAG, "[TXN] ROLLED BACK to %.2f MHz", (double)txn_old_freq);
            }
            else
            {
                ESP_LOGE(TAG, "[TXN] Rollback reconfigure FAILED — radio may be stuck");
            }
            lora_txn_state = LoRaTxnState::IDLE;
            sendCurrentConfig();
            break;
        }
    }
}

// ============================================================================
// LoRa Silence Recovery (issue #71)
// ============================================================================
// If the base station hears nothing from any rocket for RECOVERY_SILENCE_MS
// while on the ground (not freq_locked_for_flight), hop through known-good
// frequencies looking for the rocket:
//   Phase A (rendezvous): tune to LORA_FACTORY_RENDEZVOUS_MHZ and listen 3 s.  If a
//     beacon / telem arrives, relay Cmd 10 with the saved NVS config to
//     push the rocket back, then return to the saved NVS freq.
//   Phase B (grid scan): if Phase A was silent, scan the saved NVS freq ±
//     2 MHz in 200 kHz steps (21 channels), dwelling one beacon cycle per
//     step.  On hit, relay Cmd 10 on that channel and return to NVS.
//     If the grid completes with nothing heard, give up this cycle and
//     wait for the next silence trip.
// While in flight (freq_locked_for_flight) recovery is fully disabled —
// momentary silence during flight is expected (SNR dips) and hopping would
// guarantee we lose the rest of the telemetry stream.

enum class RecoveryState : uint8_t {
    IDLE,
    PHASE_A_RENDEZVOUS,
    PHASE_B_SCAN,
    COMPLETE,               // Relay in flight; hop home once it drains
};

static RecoveryState recovery_state = RecoveryState::IDLE;
static uint32_t recovery_phase_start_ms     = 0;
static uint32_t recovery_baseline_packet_ms = 0;
static int      recovery_scan_index         = 0;
static float    recovery_scan_current_mhz   = 0.0f;

static constexpr uint32_t RECOVERY_SILENCE_MS       = 10000; // idle trigger
// Phase A dwells on the rendezvous frequency long enough to (a) catch
// many beacons from a rocket sitting permanently on rendezvous (factory
// default case, ~15 beacons in 30 s) and (b) deterministically overlap
// the rocket's 5 s slow-rendezvous window when the two NVS freqs differ.
// Anything shorter than ~10 s is statistical and was the root of the
// "BS comes up, never sees rocket" symptom from the field test.
static constexpr uint32_t RECOVERY_PHASE_A_DWELL_MS = 30000;
static constexpr uint32_t RECOVERY_PHASE_B_DWELL_MS = 2500;  // one beacon cycle + slack
static constexpr int      RECOVERY_PHASE_B_CHANNELS = 21;    // ±10 steps of 200 kHz
static constexpr float    RECOVERY_PHASE_B_STEP_MHZ = 0.200f;
static constexpr float    RECOVERY_PHASE_B_SPAN_MHZ = 2.0f;  // ±2 MHz around NVS

// Hop to the full rendezvous mode (freq + SF/BW/CR/power).  Used for
// Phase A — both ends agree on this exact config as a known-good
// fallback, regardless of what the user set in NVS.  All five values
// are compile-time constants in RocketComputerTypes.h so BS and OC
// cannot disagree on the meeting place (#105 follow-up).
static void recoveryHopToRendezvousMode()
{
    if (lora_comms.reconfigure(LORA_FACTORY_RENDEZVOUS_MHZ,
                                LORA_FACTORY_RENDEZVOUS_SF,
                                LORA_FACTORY_RENDEZVOUS_BW_KHZ,
                                LORA_FACTORY_RENDEZVOUS_CR,
                                LORA_FACTORY_RENDEZVOUS_TX_DBM))
    {
        lora_comms.startReceive();
    }
    else
    {
        ESP_LOGE(TAG, "[RECOVER] reconfigure to rendezvous mode failed");
    }
}

// Hop to a target frequency keeping the user-configured NVS modulation
// (SF/BW/CR/power).  Used for Phase B local scan and the post-recovery
// return to the saved channel.  This catches the common case of "rocket
// is on a slightly off frequency but same SF/BW as the BS".
static void recoveryHopToFreq(float freq_mhz)
{
    if (lora_comms.reconfigure(freq_mhz, lora_sf, lora_bw_khz, lora_cr, lora_tx_power))
    {
        lora_comms.startReceive();
    }
    else
    {
        ESP_LOGE(TAG, "[RECOVER] reconfigure to %.2f MHz failed", (double)freq_mhz);
    }
}

static void recoveryEnterPhaseA()
{
    recoveryHopToRendezvousMode();
    recovery_baseline_packet_ms = last_packet_ms;
    recovery_phase_start_ms     = millis();
    recovery_state              = RecoveryState::PHASE_A_RENDEZVOUS;
    ESP_LOGW(TAG, "[RECOVER] Silent — Phase A rendezvous mode (%.2f MHz SF%u BW%.0f) for %u ms",
             (double)LORA_FACTORY_RENDEZVOUS_MHZ,
             (unsigned)LORA_FACTORY_RENDEZVOUS_SF,
             (double)LORA_FACTORY_RENDEZVOUS_BW_KHZ,
             (unsigned)RECOVERY_PHASE_A_DWELL_MS);
}

static void recoveryEnterPhaseB()
{
    recovery_scan_index       = 0;
    recovery_scan_current_mhz = lora_freq_mhz - RECOVERY_PHASE_B_SPAN_MHZ;
    recoveryHopToFreq(recovery_scan_current_mhz);
    recovery_baseline_packet_ms = last_packet_ms;
    recovery_phase_start_ms     = millis();
    recovery_state              = RecoveryState::PHASE_B_SCAN;
    ESP_LOGW(TAG, "[RECOVER] Phase B scan: %d channels around %.2f MHz",
             RECOVERY_PHASE_B_CHANNELS, (double)lora_freq_mhz);
}

static void recoveryEnd(const char* why)
{
    // Always come back to the saved NVS frequency so we're either settled
    // on the committed channel or poised to hear the rocket once it
    // returns there.
    recoveryHopToFreq(lora_freq_mhz);
    recovery_state = RecoveryState::IDLE;
    ESP_LOGI(TAG, "[RECOVER] Done (%s). Back on %.2f MHz",
             why, (double)lora_freq_mhz);
}

/// Relay Cmd 10 on the current channel to push the rocket back to the
/// saved NVS config.  Used when we re-locate the rocket during recovery.
static void recoveryPushRocketHome()
{
    uint8_t payload[11];
    memcpy(payload + 0, &lora_freq_mhz, 4);
    memcpy(payload + 4, &lora_bw_khz,   4);
    payload[8]  = lora_sf;
    payload[9]  = lora_cr;
    payload[10] = (uint8_t)lora_tx_power;
    buildUplinkPacket(10, payload, 11, /* target_rid = broadcast */ 0xFF);
    ESP_LOGI(TAG, "[RECOVER] Relay Cmd 10 -> %.2f MHz (saved NVS)",
             (double)lora_freq_mhz);
}

static void serviceRecovery()
{
    // #136: with hopping disabled the whole recovery state machine is
    // redundant.  Both ends are pinned to LORA_FACTORY_RENDEZVOUS at
    // boot (and can't drift — no auto-acquire move, no app-driven cmd
    // 10), so Phase A's reconfigure is a no-op and the post-Phase-A
    // "push rocket home" cmd 10 just blasts redundant TX retries at
    // a rocket that's already on the right channel.  Leave the
    // state machine inert until #150 re-enables hopping.
    if (lora_hop_disabled)
    {
        if (recovery_state != RecoveryState::IDLE)
            recoveryEnd("hop disabled — recovery suppressed");
        return;
    }
    // While locked for flight, or while actively hopping with the rocket,
    // accept silence — neither is a recovery scenario.  In flight, momentary
    // SNR dips look like silence; while hopping (#40 / #41), the recovery
    // hop scan and the hop sequence would fight each other for the radio.
    if (freq_locked_for_flight || hop_active_)
    {
        if (recovery_state != RecoveryState::IDLE)
            recoveryEnd(freq_locked_for_flight ? "flight locked" : "hopping active");
        return;
    }
    // Transactional reconfigure takes priority.  A BLE Cmd 10 arriving
    // mid-recovery aborts the recovery cycle; the transaction then
    // self-rolls-back-or-commits, and any residual divergence is healed
    // by the next recovery pass.
    if (lora_txn_state != LoRaTxnState::IDLE)
    {
        if (recovery_state != RecoveryState::IDLE) recoveryEnd("transaction");
        return;
    }

    const uint32_t now = millis();

    switch (recovery_state)
    {
        case RecoveryState::IDLE:
        {
            // Avoid firing during the first RECOVERY_SILENCE_MS after boot
            // — it's normal to be silent while the rocket is still booting.
            if (last_packet_ms == 0 && now < RECOVERY_SILENCE_MS) return;
            const uint32_t silent_for = (last_packet_ms > 0)
                ? (now - last_packet_ms)
                : now;
            if (silent_for >= RECOVERY_SILENCE_MS) recoveryEnterPhaseA();
            break;
        }

        case RecoveryState::PHASE_A_RENDEZVOUS:
        {
            if (last_packet_ms > recovery_baseline_packet_ms)
            {
                recoveryPushRocketHome();
                recovery_phase_start_ms = now;
                recovery_state = RecoveryState::COMPLETE;
                break;
            }
            if ((now - recovery_phase_start_ms) >= RECOVERY_PHASE_A_DWELL_MS)
            {
                // #136: Phase B scans ±2 MHz around lora_freq_mhz to catch
                // a hopping rocket that drifted to an unknown channel.
                // With hopping disabled, the rocket is fixed on its
                // configured channel — walking the BS off-frequency only
                // hides the rocket from us during the very window it's
                // beaconing.  Just end the recovery cycle; next silence
                // tick will start another Phase A dwell on rendezvous,
                // which is the only place a fixed-channel rocket can be.
                if (lora_hop_disabled)
                {
                    recoveryEnd("hop disabled — skipping Phase B scan");
                }
                else
                {
                    recoveryEnterPhaseB();
                }
            }
            break;
        }

        case RecoveryState::PHASE_B_SCAN:
        {
            if (last_packet_ms > recovery_baseline_packet_ms)
            {
                recoveryPushRocketHome();
                recovery_phase_start_ms = now;
                recovery_state = RecoveryState::COMPLETE;
                break;
            }
            if ((now - recovery_phase_start_ms) >= RECOVERY_PHASE_B_DWELL_MS)
            {
                recovery_scan_index++;
                if (recovery_scan_index >= RECOVERY_PHASE_B_CHANNELS)
                {
                    // Exhausted grid with no hits — give up this cycle.
                    // Silence will trip us again after RECOVERY_SILENCE_MS.
                    recoveryEnd("scan exhausted");
                    return;
                }
                recovery_scan_current_mhz += RECOVERY_PHASE_B_STEP_MHZ;
                recoveryHopToFreq(recovery_scan_current_mhz);
                recovery_baseline_packet_ms = last_packet_ms;
                recovery_phase_start_ms = now;
            }
            break;
        }

        case RecoveryState::COMPLETE:
        {
            // Give uplink retries time to land on the current channel
            // before we hop back to NVS — otherwise the rocket might miss
            // the push command and we'd just rediscover it next cycle.
            if (!uplink_pending ||
                (now - recovery_phase_start_ms) > TXN_MAX_RELAY_MS)
            {
                recoveryEnd("pushed rocket home");
            }
            break;
        }
    }
}

// ============================================================================
// Heartbeat (issue #71)
// ============================================================================
// Periodic uplink that gives the rocket positive proof of comms.  Without
// this, a rocket happily streaming telemetry to a base station that's
// receiving fine would still fall into its slow-rendezvous cycle every
// time the user goes a couple of minutes without sending a command —
// because beacons go one way and the rocket has no signal that anyone is
// listening.
//
// Only sent while we ARE hearing the rocket (last_packet_ms recent).  If
// rocket goes silent, the recovery state machine takes over and we stop
// heartbeating until comms are restored.  Uses 2 retries instead of 8 to
// keep airtime per beat small (~100 ms total).
//
// Interval must be < OC RENDEZVOUS_TRIGGER_QUIET_MS (15 s in #105) with
// margin for at least one missed beat — otherwise the rocket fires its
// slow-rendezvous in steady state, drops to the rendezvous channel, and
// the BS spends ~30 s syncing it back via Cmd-10 relay.  10 s gives the
// OC two heartbeats per silence-tolerance window and keeps airtime ~1%.
// (Defined here, after the LoRa transaction & recovery sections, so the
// state-machine enums it depends on are in scope.)

static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 10000;  // every 10 s (#105)
static constexpr uint32_t HEARTBEAT_RX_FRESH_MS = 5000;   // rocket "alive"
static constexpr uint8_t  HEARTBEAT_RETRIES     = 2;
static uint32_t last_heartbeat_tx_ms = 0;

// Safe-window guard for BS uplinks while the rocket is hopping
// (#40 / #41 phase 2b).  The rocket TXes telemetry at a fixed rate
// (typ. 2 Hz = 500 ms cycle); after we RX a packet, we know the
// rocket is in RX mode for the rest of its slot.  TXing within
// ~150 ms of that RX guarantees we finish well before the rocket's
// next TX, avoiding the desync collision pattern that broke the
// link in phase 2a bench testing.
static constexpr uint32_t HOP_BS_TX_SAFE_WINDOW_MS = 150;

static inline bool inHopSafeWindow()
{
    if (!hop_active_) return true;             // not hopping → always safe
    if (last_packet_ms == 0) return false;     // never RX'd → no anchor
    return (millis() - last_packet_ms) <= HOP_BS_TX_SAFE_WINDOW_MS;
}

static void serviceHeartbeat()
{
    if (freq_locked_for_flight)                return;  // No heartbeats in flight
    if (lora_txn_state != LoRaTxnState::IDLE)  return;  // Don't interfere with txn
    if (recovery_state != RecoveryState::IDLE) return;  // Recovery owns the radio
    if (scan_passes_remaining_ != 0)           return;  // #136: don't TX mid-scan
    if (uplink_pending)                        return;  // Don't clobber a real cmd
    // While hopping, only TX in the safe window right after a fresh
    // rocket RX — see HOP_BS_TX_SAFE_WINDOW_MS comment.  With slow-hop
    // dwell=4 (#105 follow-up) the rocket stays on a channel for 2 s,
    // so a 250 ms heartbeat-with-retries fits comfortably inside the
    // dwell.  Without heartbeats during hop, the rocket's
    // hop_session_uplink_count stays 0 and its fallback fires every 30 s
    // (HOP_FALLBACK_TRIGGER_INITIAL_MS), tearing down the session.
    if (!inHopSafeWindow())                    return;

    const uint32_t now = millis();
    // Only heartbeat when we've recently heard the rocket.  If rocket has
    // gone silent, recovery will engage and ramp through rendezvous/scan;
    // beating into the void during that is just wasted airtime.
    if (last_packet_ms == 0)                                  return;
    if ((now - last_packet_ms) > HEARTBEAT_RX_FRESH_MS)       return;
    if ((now - last_heartbeat_tx_ms) < HEARTBEAT_INTERVAL_MS) return;

    buildUplinkPacket(LORA_CMD_HEARTBEAT, nullptr, 0,
                      /* target_rid = broadcast */ 0xFF,
                      /* retries */ HEARTBEAT_RETRIES);
    last_heartbeat_tx_ms = now;
}

// ============================================================================
// Channel-set: multi-pass scan + auto-analyze + push to rocket (#40/#41 phase 3)
// ============================================================================
// On BLE cmd 60 we run a 5-pass noise scan (max-RSSI accumulation per
// channel) instead of a single sweep, so an intermittent jammer that
// only fires every couple of seconds is more likely to be captured.
// When all passes complete, we ship the accumulated grid to the iOS app
// (preserving the existing single-result protocol), run the channel-set
// analyzer locally, persist the result to NVS, and push it to the
// rocket via LORA_CMD_CHANNEL_SET.

static constexpr uint8_t LORA_NOISE_SCAN_PASSES = 5;

static int8_t   scan_peak_rssi_[TR_LoRa_Comms::SCAN_MAX_SAMPLES] = {0};
static size_t   scan_peak_count_ = 0;
static float    scan_peak_start_mhz_ = 0.0f;
static float    scan_peak_step_khz_  = 0.0f;
// scan_passes_remaining_ is declared near the top of the file so
// serviceHeartbeat() can read it.
// Set true after a multi-pass scan completes; used by the cmd-10
// commit path to re-push the channel-set selection (the original
// post-scan push gets clobbered when the iOS app's auto-channel-select
// queues cmd 10 immediately after seeing scan results).
static bool     scan_results_valid_ = false;

// Held across passes so subsequent calls re-use the same parameters.
static float    scan_param_start_mhz_ = 0.0f;
static float    scan_param_stop_mhz_  = 0.0f;
static uint16_t scan_param_step_khz_  = 0;
static uint16_t scan_param_dwell_ms_  = 0;

// ----------------------------------------------------------------------------
// Coordinated hop pause for in-flight scan (#90)
// ----------------------------------------------------------------------------
// When a BLE cmd 60 arrives while we're already following a hopping rocket
// (hop_active_ == true), running the scan directly would silence the link
// for ~9 s and trip the rocket's hop fallback.  Instead, we ask the rocket
// to park on lora_freq_mhz for a known window via cmd 16, then run the
// scan + push cmd 15 inside that window, then both sides re-bootstrap hop.
//
// State flow:
//   IDLE          → AWAITING_PAUSE  (cmd 16 queued; wait for retries done)
//   AWAITING_PAUSE → SCANNING       (reconfigure to lora_freq_mhz, kick scan)
//   SCANNING       → PUSHING_CHSET  (scan finalized, cmd 15 queued)
//   PUSHING_CHSET  → RESUMING       (cmd 15 retries done)
//   RESUMING       → IDLE           (rocket bootstrap RX, or timeout)
enum class CoordScanState : uint8_t {
    IDLE,
    AWAITING_PAUSE,
    SCANNING,
    PUSHING_CHSET,
    RESUMING,
};

static CoordScanState coord_scan_state_       = CoordScanState::IDLE;
static uint32_t       coord_scan_phase_ms_    = 0;
static uint32_t       coord_scan_resume_anchor_ms_ = 0;  // hop_last_rx_ms_ snapshot at RESUMING entry
// Stored scan params so we can re-issue startNoiseScan() after the pause is acked.
static float    coord_scan_start_mhz_ = 0.0f;
static float    coord_scan_stop_mhz_  = 0.0f;
static uint16_t coord_scan_step_khz_  = 0;
static uint16_t coord_scan_dwell_ms_  = 0;

// After cmd 16 retries finish, give the rocket a moment to actually swap
// to lora_freq_mhz before we reconfigure the BS radio.
static constexpr uint32_t COORD_SCAN_PAUSE_GRACE_MS  = 500;
// If the rocket never resumes hopping after we push cmd 15, give up so
// the normal silence/recovery machinery can take over.
static constexpr uint32_t COORD_SCAN_RESUMING_MAX_MS = 5000;
// Slack on top of the computed scan + cmd 15 retry budget so the rocket's
// pause comfortably outlasts our work window.
static constexpr uint32_t COORD_SCAN_PAUSE_SLACK_MS  = 2000;
// "Recent enough" window for treating a non-hop_active_ rocket as still
// in a hop state (#90).  A packet within this window showing PRELAUNCH
// or INFLIGHT means the rocket is conceptually hopping (possibly
// bootstrapping or visiting rendezvous) and a direct scan would still
// drop the link — so route through the coordinated-pause path.  Sized
// to match RECOVERY_SILENCE_MS (10 s): beyond that the recovery layer
// has already taken over and we don't presume anything.
static constexpr uint32_t COORD_HOP_RECENT_MS        = 10000;

// Persist channel-set selection to NVS.  Skip-mask is keyed off the BW
// it was generated for so a later cmd-10 BW change invalidates it
// cleanly (loadChannelSetFromNvs detects mismatch and clears).
// Rendezvous freq is no longer persisted — see #105 / LORA_NVS_SCHEMA_VERSION
// v3 in RocketComputerTypes.h.
static void saveChannelSetToNvs(const LoRaChannelSetSelection& sel,
                                float bw_khz)
{
    Preferences p;
    if (!p.begin("lora", false)) return;
    p.putUChar("chset_n", sel.n_channels);
    p.putFloat("chset_bw", bw_khz);
    const size_t bytes_used = (sel.n_channels + 7) / 8;
    p.putBytes("chset_mask", sel.skip_mask, bytes_used);
    p.end();
}

// Restore channel-set selection from NVS.  If the stored BW doesn't
// match the active operating BW, the skip-mask is treated as stale and
// the runtime state is reset to "no skips".  Rendezvous freq is no
// longer NVS-stored (#105) — it's a compile-time constant.
static void loadChannelSetFromNvs()
{
    Preferences p;
    if (!p.begin("lora", true)) return;
    const uint8_t n_stored  = p.getUChar("chset_n", 0);
    const float   bw_stored = p.getFloat("chset_bw", 0.0f);
    if (n_stored > 0 && bw_stored > 0.0f && bw_stored == lora_bw_khz)
    {
        const size_t bytes_used = (n_stored + 7) / 8;
        size_t got = p.getBytes("chset_mask", skip_mask_, bytes_used);
        if (got == bytes_used)
        {
            skip_mask_n_        = n_stored;
            channel_set_bw_khz_ = bw_stored;
        }
    }
    else
    {
        skip_mask_n_        = 0;
        channel_set_bw_khz_ = 0.0f;
        for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) skip_mask_[i] = 0;
    }
    p.end();
}

// Clear stored skip-mask — called on cmd-10 BW change so the rocket
// doesn't follow a stale skip-mask sized for the previous BW.
static void invalidateSkipMaskForBwChange()
{
    skip_mask_n_        = 0;
    channel_set_bw_khz_ = 0.0f;
    for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) skip_mask_[i] = 0;
    Preferences p;
    if (p.begin("lora", false))
    {
        p.remove("chset_n");
        p.remove("chset_bw");
        p.remove("chset_mask");
        p.end();
    }
    ESP_LOGI(TAG, "[CHSET] BW changed — skip-mask invalidated");
}

// Apply a freshly computed selection to runtime state and queue a push
// to the rocket so the rocket's NVS + runtime mirrors ours.
static void applyAndPushChannelSet(const LoRaChannelSetSelection& sel,
                                    float bw_khz)
{
    // Adopt locally
    skip_mask_n_        = sel.n_channels;
    channel_set_bw_khz_ = bw_khz;
    const size_t bytes_used = (sel.n_channels + 7) / 8;
    for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) skip_mask_[i] = 0;
    for (size_t i = 0; i < bytes_used; i++) skip_mask_[i] = sel.skip_mask[i];

    saveChannelSetToNvs(sel, bw_khz);

    // Wire format: [bw:f4][n:u1][mask: ceil(n/8)]
    // Rendezvous freq used to lead this payload but is now hardcoded on
    // both sides (#105) — see LORA_FACTORY_RENDEZVOUS_MHZ.
    uint8_t payload[40] = {0};
    size_t  off = 0;
    memcpy(payload + off, &bw_khz,             4); off += 4;
    payload[off++] = sel.n_channels;
    for (size_t i = 0; i < bytes_used; i++) payload[off++] = sel.skip_mask[i];

    // Count active for the log so user sees how many channels survived
    uint8_t active = 0;
    for (uint8_t i = 0; i < sel.n_channels; i++)
        if (!loraSkipMaskTest(sel.skip_mask, i)) active++;
    ESP_LOGI(TAG, "[CHSET] %u/%u channels active at BW=%.0f kHz "
                  "(min FCC floor %u) — pushing to rocket",
             (unsigned)active,
             (unsigned)sel.n_channels, (double)bw_khz,
             (unsigned)loraFhssMinChannels(bw_khz));

    buildUplinkPacket(LORA_CMD_CHANNEL_SET, payload, off, /*target_rid=*/0xFF,
                      /*retries=*/config::UPLINK_RETRIES);
}

// Called on a fresh BLE cmd-60 to start the multi-pass scan sequence.
// Returns true if the first pass started successfully.
static bool startNoiseScan(float start_mhz, float stop_mhz,
                            uint16_t step_khz, uint16_t dwell_ms)
{
    if (scan_passes_remaining_ != 0) return false;  // already scanning

    scan_param_start_mhz_  = start_mhz;
    scan_param_stop_mhz_   = stop_mhz;
    scan_param_step_khz_   = step_khz;
    scan_param_dwell_ms_   = dwell_ms;
    scan_peak_count_       = 0;  // signals "first pass, copy not max"
    scan_results_valid_    = false;  // becomes true on finalize
    for (size_t i = 0; i < TR_LoRa_Comms::SCAN_MAX_SAMPLES; i++)
        scan_peak_rssi_[i] = INT8_MIN;

    if (!lora_comms.startScan(start_mhz, stop_mhz, step_khz, dwell_ms))
        return false;

    scan_passes_remaining_ = LORA_NOISE_SCAN_PASSES;
    ESP_LOGI(TAG, "[CHSET] Noise scan started: %u passes × %u ms dwell, %.1f..%.1f MHz",
             (unsigned)LORA_NOISE_SCAN_PASSES, (unsigned)dwell_ms,
             (double)start_mhz, (double)stop_mhz);
    return true;
}

// Merge the just-completed pass's samples into scan_peak_rssi_[] (max).
// On the first pass we also capture the geometry (start_mhz, step_khz)
// for the analyzer.  Returns true if more passes are needed.
static bool absorbScanPass()
{
    const auto* samples = lora_comms.getScanSamples();
    const size_t n = lora_comms.getScanSampleCount();

    if (scan_peak_count_ == 0)
    {
        scan_peak_start_mhz_ = lora_comms.getScanStartMHz();
        scan_peak_step_khz_  = lora_comms.getScanStepKHz();
        scan_peak_count_     = (n > TR_LoRa_Comms::SCAN_MAX_SAMPLES)
                                ? TR_LoRa_Comms::SCAN_MAX_SAMPLES : n;
        for (size_t i = 0; i < scan_peak_count_; i++)
            scan_peak_rssi_[i] = samples[i].rssi_dbm;
    }
    else
    {
        const size_t cap = (n < scan_peak_count_) ? n : scan_peak_count_;
        for (size_t i = 0; i < cap; i++)
            if (samples[i].rssi_dbm > scan_peak_rssi_[i])
                scan_peak_rssi_[i] = samples[i].rssi_dbm;
    }
    lora_comms.consumeScanDone();
    return (--scan_passes_remaining_) > 0;
}

// Re-run the channel-set analyzer over the most recent scan grid using
// `lora_bw_khz` (whatever it is right now), then push to the rocket.
// Used by both finalizeNoiseScan() and the cmd-10 commit path — the
// latter to re-push after the auto-select cmd-10 race clobbered the
// initial cmd-15 in the uplink queue.
static void analyzeAndPushFromCachedScan()
{
    if (!scan_results_valid_ || scan_peak_count_ == 0) return;

    float scan_freqs[TR_LoRa_Comms::SCAN_MAX_SAMPLES];
    for (size_t i = 0; i < scan_peak_count_; i++)
    {
        scan_freqs[i] = scan_peak_start_mhz_
                        + (scan_peak_step_khz_ * (float)i) / 1000.0f;
    }
    LoRaChannelSetSelection sel{};
    loraSelectChannelSet(scan_freqs, scan_peak_rssi_, scan_peak_count_,
                          lora_bw_khz, &sel);
    applyAndPushChannelSet(sel, lora_bw_khz);
}

// Re-push the BS's current skip-mask state to the rocket — used by the
// mask-drift auto-recovery (#105 follow-up) when the rocket's announced
// next_channel_idx doesn't match the BS's seq-derived expectation.  No
// re-analysis (no scan needed); just (re-)delivers what we already have.
// Inert if we have no mask to push.
static void pushCurrentChannelSet()
{
    if (skip_mask_n_ == 0) return;  // nothing to push
    LoRaChannelSetSelection sel = {};
    sel.n_channels = skip_mask_n_;
    for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) {
        sel.skip_mask[i] = skip_mask_[i];
    }
    applyAndPushChannelSet(sel, channel_set_bw_khz_);
}

// Mask-drift auto-recovery service (#105 follow-up).  Watches the flag
// set by the hop RX path and re-pushes cmd 15 when the radio state is
// quiet enough to do so safely.  Cooldown prevents spam if drift
// persists across multiple packets.
static void serviceMaskDriftRepush()
{
    if (!chset_drift_repush_pending) return;
    // Only push when nothing else is using the radio for TX.
    if (uplink_pending)                        return;
    if (lora_txn_state != LoRaTxnState::IDLE)  return;
    if (recovery_state != RecoveryState::IDLE) return;
    // Cooldown: don't re-push more than once every CHSET_DRIFT_REPUSH_COOLDOWN_MS.
    // First-ever push (last_ms_=0) bypasses the cooldown.
    const uint32_t now = millis();
    if (chset_drift_repush_last_ms_ != 0 &&
        (now - chset_drift_repush_last_ms_) < CHSET_DRIFT_REPUSH_COOLDOWN_MS)
    {
        // Cooldown active; clear the flag so we don't spin checking.
        // Next drift event after the cooldown will set it again.
        chset_drift_repush_pending = false;
        return;
    }
    if (skip_mask_n_ == 0) {
        // No mask to push — likely a brand-new BS without scan results.
        // The drift warning is informational only in this case.
        chset_drift_repush_pending = false;
        return;
    }
    chset_drift_repush_pending = false;
    chset_drift_repush_last_ms_ = now;
    chset_drift_repush_count++;
    ESP_LOGI(TAG, "[CHSET] Mask drift detected — re-pushing cmd 15 (count=%lu)",
             (unsigned long)chset_drift_repush_count);
    pushCurrentChannelSet();
}

// All passes done.  Ship results to BLE (preserving the existing
// single-result protocol so the iOS app doesn't need to change), then
// dispatch to either the auto-acquire single-channel picker (#136) or
// the legacy skip-mask push (kept for the hopping path, currently gated
// off by default).
static void finalizeNoiseScan()
{
    ble_app.sendScanResults(scan_peak_start_mhz_, scan_peak_step_khz_,
                            scan_peak_rssi_, (uint8_t)scan_peak_count_);
    scan_results_valid_ = true;

    // #136: when our auto-acquire flow kicked off this scan, the
    // post-scan action is "pick one channel + cmd-10 move", not
    // "compute skip-mask + cmd-15 push".  Any other scan (BLE cmd 60,
    // coordinated scan) falls through to the existing skip-mask path.
    if (auto_acquire_state == AutoAcquireState::SCANNING)
    {
        autoAcquireOnScanFinalize();
    }
    else
    {
        analyzeAndPushFromCachedScan();
    }
}

// Compute the cmd 16 pause duration (ms) for a coordinated scan.  Sized
// to comfortably cover scan + cmd 15 retries + slack, and capped to the
// rocket-side max so a too-large value doesn't silently get truncated
// to a different value than we accounted for.
static uint16_t computeCoordPauseMs(float start_mhz, float stop_mhz,
                                     uint16_t step_khz, uint16_t dwell_ms)
{
    if (step_khz == 0 || stop_mhz <= start_mhz) return LORA_HOP_PAUSE_MAX_MS;
    const uint32_t span_khz   = (uint32_t)((stop_mhz - start_mhz) * 1000.0f);
    const uint32_t channels   = span_khz / step_khz + 1;
    const uint32_t scan_ms    = (uint32_t)LORA_NOISE_SCAN_PASSES * channels
                                * (uint32_t)dwell_ms;
    const uint32_t cmd15_ms   = (uint32_t)config::UPLINK_RETRIES
                                * (uint32_t)config::UPLINK_RETRY_INTERVAL_MS;
    uint32_t total = scan_ms + cmd15_ms + COORD_SCAN_PAUSE_GRACE_MS
                     + COORD_SCAN_PAUSE_SLACK_MS;
    if (total > LORA_HOP_PAUSE_MAX_MS) total = LORA_HOP_PAUSE_MAX_MS;
    return (uint16_t)total;
}

// Kick off a coordinated scan: queue cmd 16 to the rocket and stash the
// scan params for use once retries finish.  Caller has already verified
// hop_active_ and that no coordinated scan is in progress.
static void startCoordinatedScan(float start_mhz, float stop_mhz,
                                  uint16_t step_khz, uint16_t dwell_ms)
{
    coord_scan_start_mhz_ = start_mhz;
    coord_scan_stop_mhz_  = stop_mhz;
    coord_scan_step_khz_  = step_khz;
    coord_scan_dwell_ms_  = dwell_ms;

    const uint16_t pause_ms = computeCoordPauseMs(start_mhz, stop_mhz,
                                                   step_khz, dwell_ms);
    uint8_t payload[2];
    memcpy(payload, &pause_ms, 2);
    buildUplinkPacket(LORA_CMD_HOP_PAUSE, payload, 2,
                      /*target_rid=*/0xFF, config::UPLINK_RETRIES);
    coord_scan_state_    = CoordScanState::AWAITING_PAUSE;
    coord_scan_phase_ms_ = millis();
    ESP_LOGI(TAG, "[CHSET] Coordinated scan start: pausing rocket for %u ms "
                  "(scan range %.1f..%.1f MHz, %u kHz, %u ms dwell)",
             (unsigned)pause_ms, (double)start_mhz, (double)stop_mhz,
             (unsigned)step_khz, (unsigned)dwell_ms);
}

// Drive the coordinated-scan state machine.  Called every loop iteration
// before the scan-done detection so that AWAITING_PAUSE → SCANNING can
// kick off a fresh scan in the same iteration where it transitions.
static void serviceCoordinatedScan()
{
    if (coord_scan_state_ == CoordScanState::IDLE) return;

    const uint32_t now = millis();
    switch (coord_scan_state_)
    {
        case CoordScanState::IDLE: return;
        case CoordScanState::AWAITING_PAUSE:
        {
            // Cmd 16 retries still going — push out the grace anchor so
            // we wait COORD_SCAN_PAUSE_GRACE_MS after the *last* retry,
            // not after we queued the command.
            if (uplink_pending)
            {
                coord_scan_phase_ms_ = now;
                return;
            }
            if ((now - coord_scan_phase_ms_) < COORD_SCAN_PAUSE_GRACE_MS) return;

            // Cmd 16 has been pushed (8 retries / ~800 ms).  We don't
            // get an explicit ack — the rocket pause is fire-and-forget.
            // Reconfigure the BS radio to lora_freq_mhz so we (a) can
            // verify the rocket is actually parked there, and (b) the
            // scan starts from a known frequency.
            if (!lora_comms.reconfigure(lora_freq_mhz, lora_sf, lora_bw_khz,
                                         lora_cr, lora_tx_power))
            {
                ESP_LOGE(TAG, "[CHSET] Coord scan: reconfigure to %.2f MHz failed — abandoning",
                         (double)lora_freq_mhz);
                coord_scan_state_ = CoordScanState::IDLE;
                return;
            }
            (void)lora_comms.startReceive();
            // Drop hop tracking flags so the post-scan packet from the
            // rocket re-enters hop following cleanly (matching a fresh
            // bootstrap, not a continuation).
            hop_active_       = false;
            hop_needs_retune_ = false;

            if (!startNoiseScan(coord_scan_start_mhz_, coord_scan_stop_mhz_,
                                 coord_scan_step_khz_, coord_scan_dwell_ms_))
            {
                ESP_LOGE(TAG, "[CHSET] Coord scan: startNoiseScan failed — abandoning");
                coord_scan_state_ = CoordScanState::IDLE;
                return;
            }
            coord_scan_state_    = CoordScanState::SCANNING;
            coord_scan_phase_ms_ = now;
            ESP_LOGI(TAG, "[CHSET] Coord scan: rocket should be parked, scan started");
            break;
        }
        case CoordScanState::SCANNING:
        {
            // finalizeNoiseScan() runs from the existing scan-done path
            // and (a) sets scan_results_valid_ = true, (b) queues cmd 15
            // via applyAndPushChannelSet().  Either signal works as a
            // transition trigger; combining them is most precise.
            if (scan_results_valid_ && uplink_pending)
            {
                coord_scan_state_    = CoordScanState::PUSHING_CHSET;
                coord_scan_phase_ms_ = now;
            }
            break;
        }
        case CoordScanState::PUSHING_CHSET:
        {
            if (uplink_pending) return;  // cmd 15 retries still going
            // Cmd 15 has been delivered (or all retries exhausted).
            // Rocket should resume hopping when its pause deadline hits.
            // Snapshot hop_last_rx_ms_ so we can detect a fresh RX.
            coord_scan_resume_anchor_ms_ = hop_last_rx_ms_;
            coord_scan_state_            = CoordScanState::RESUMING;
            coord_scan_phase_ms_         = now;
            ESP_LOGI(TAG, "[CHSET] Coord scan: cmd 15 pushed, awaiting rocket hop resume");
            break;
        }
        case CoordScanState::RESUMING:
        {
            // The rocket-side pause expires and a bootstrap packet hits
            // us on lora_freq_mhz.  The existing RX path adopts the new
            // hop_idx_ and sets hop_active_ + hop_last_rx_ms_.  Detect
            // either: a fresh RX (anchor changed) or a timeout.
            if (hop_last_rx_ms_ != coord_scan_resume_anchor_ms_)
            {
                ESP_LOGI(TAG, "[CHSET] Coord scan complete — hop resumed");
                coord_scan_state_ = CoordScanState::IDLE;
            }
            else if ((now - coord_scan_phase_ms_) > COORD_SCAN_RESUMING_MAX_MS)
            {
                ESP_LOGW(TAG, "[CHSET] Coord scan: rocket did not resume hop within %u ms — "
                              "letting normal recovery take over",
                         (unsigned)COORD_SCAN_RESUMING_MAX_MS);
                coord_scan_state_ = CoordScanState::IDLE;
            }
            break;
        }
    }
}

// ----------------------------------------------------------------------------
// Auto-acquire + auto-scan (#136)
// ----------------------------------------------------------------------------
// Every BS power cycle starts on LORA_FACTORY_RENDEZVOUS_MHZ.  Once we
// confirm the rocket is alive on that channel (one RX is enough; the
// rocket beacons at ~2 Hz on the ground), we run a 902..928 MHz noise
// scan, pick the single quietest channel, and drive the existing cmd-10
// transactional reconfigure to move the rocket onto it.  The txn's
// verify/rollback semantics naturally fall back to rendezvous if the
// rocket doesn't follow — exactly the handshake/ack-with-fallback the
// issue calls for.
//
// Why wait for an RX before scanning?  Running the scan immediately on
// boot would race the rocket's own boot.  If the rocket isn't up yet we
// move to a channel it can't hear cmd-10 on, the txn times out and we
// roll back, and we've burned 14 s for no reason.  Acquiring first means
// every scan we run results in a real move (or stays put because the
// rendezvous is already the quietest channel).
//
// One-shot per power cycle.  Re-runs require a BS reboot — keeps the
// behaviour predictable on the bench and means we never drop the link
// mid-flight to chase a different channel.
//
// Enum + auto_acquire_state are forward-declared near the top so
// finalizeNoiseScan() can branch on them; values:
//   AWAITING_ROCKET → No packet seen yet; just listening on rendezvous
//   GRACE_DELAY     → First packet seen; brief settle before we scan
//   SCANNING        → startNoiseScan() running — finalize will dispatch
//   COMMITTING      → cmd-10 transaction in flight — wait for IDLE
//   DONE            → Settled (committed or rolled back); inert
static uint32_t auto_acquire_grace_start_ms = 0;
// Brief settle window after first RX so the rocket has a moment to
// stabilize its beacon cadence and we aren't tearing down the link
// halfway through its first packet.  Short enough to feel responsive,
// long enough to avoid colliding with an in-flight uplink retry.
static constexpr uint32_t AUTO_ACQUIRE_GRACE_MS = 750;
// Scan parameters — matches what the iOS Frequency Scan view used to
// send via BLE cmd 60.  Wide enough to cover the entire US ISM band,
// fine enough that every BW=250 channel has a sample within ±250 kHz.
static constexpr float    AUTO_ACQUIRE_SCAN_START_MHZ = LORA_BAND_LO_MHZ;
static constexpr float    AUTO_ACQUIRE_SCAN_STOP_MHZ  = LORA_BAND_HI_MHZ;
static constexpr uint16_t AUTO_ACQUIRE_SCAN_STEP_KHZ  = 500;
static constexpr uint16_t AUTO_ACQUIRE_SCAN_DWELL_MS  = 30;

// Called from finalizeNoiseScan() when the auto-acquire scan completes.
// Walks the scan grid, picks the quietest channel snapped to the BW
// table, and hands off to the existing cmd-10 transaction.  If the
// quietest channel is the rendezvous we're already on, short-circuit
// to DONE rather than burning a verify window on a no-op move.
static void autoAcquireOnScanFinalize()
{
    if (scan_peak_count_ == 0)
    {
        ESP_LOGW(TAG, "[AUTO] Scan finalized with no samples — staying on %.2f MHz",
                 (double)lora_freq_mhz);
        auto_acquire_state = AutoAcquireState::DONE;
        return;
    }

    float scan_freqs[TR_LoRa_Comms::SCAN_MAX_SAMPLES];
    for (size_t i = 0; i < scan_peak_count_; i++)
    {
        scan_freqs[i] = scan_peak_start_mhz_
                        + (scan_peak_step_khz_ * (float)i) / 1000.0f;
    }
    const float quietest = loraPickQuietestChannelMHz(
        scan_freqs, scan_peak_rssi_, scan_peak_count_, lora_bw_khz);

    // No-op move detection.  loraChannelMHz() returns multiples of the
    // BW spacing offset from LORA_BAND_LO_MHZ + bw/2, so equality here
    // is exact in principle, but compare with a tolerance well below the
    // channel spacing just to be safe against float quirks.
    if (fabsf(quietest - lora_freq_mhz) < 0.05f)
    {
        ESP_LOGI(TAG, "[AUTO] Rendezvous %.2f MHz is already the quietest — staying put",
                 (double)lora_freq_mhz);
        auto_acquire_state = AutoAcquireState::DONE;
        return;
    }

    if (startLoRaTransaction(quietest, lora_bw_khz, lora_sf, lora_cr, lora_tx_power))
    {
        auto_acquire_state = AutoAcquireState::COMMITTING;
        ESP_LOGI(TAG, "[AUTO] Scan done — moving rocket to %.2f MHz (was %.2f MHz)",
                 (double)quietest, (double)lora_freq_mhz);
    }
    else
    {
        // startLoRaTransaction refuses if freq is locked for flight or a
        // txn is already in progress.  Neither is expected this early in
        // a power cycle, but if it happens we just stay on rendezvous.
        ESP_LOGW(TAG, "[AUTO] startLoRaTransaction refused — staying on %.2f MHz",
                 (double)lora_freq_mhz);
        auto_acquire_state = AutoAcquireState::DONE;
    }
}

// Drive the auto-acquire state machine.  Called every loop iteration.
// Cheap when in AWAITING_ROCKET / DONE — most calls are a single switch.
//
// #136 v2: scan-and-move was removed.  The whole point of the issue is to
// stay on the hardcoded rendezvous (915 MHz SF8 BW250) for the duration
// of the test so we can measure raw link performance.  Picking a
// "quieter" channel and moving via cmd 10 adds a fragile transaction
// where any cmd-10 loss makes the BS roll back to 915 — which is
// exactly the no-op we'd have wanted in the first place.  Field-tested
// on 2026-05-11: the BS picked 923.5, the OC didn't follow, BS rolled
// back, and the user (rightly) flagged the BS as no longer "on a
// single frequency".
//
// State machine now just waits for first RX and declares success.  The
// scan helpers (autoAcquireOnScanFinalize, AUTO_ACQUIRE_SCAN_*) are kept
// in source for when #150 re-enables the scan-and-move pathway.
static void serviceAutoAcquire()
{
    if (auto_acquire_state == AutoAcquireState::DONE) return;

    // Only the AWAITING_ROCKET state is reachable now; the rest of the
    // enum is dead-code preserved for #150's follow-up.  Belt-and-braces:
    // if anything ever leaves us in a non-AWAITING state, flush to DONE
    // so we don't get stuck.
    if (auto_acquire_state != AutoAcquireState::AWAITING_ROCKET)
    {
        ESP_LOGW(TAG, "[AUTO] Unexpected state %d — forcing DONE",
                 (int)auto_acquire_state);
        auto_acquire_state = AutoAcquireState::DONE;
        return;
    }

    if (freq_locked_for_flight)
    {
        ESP_LOGW(TAG, "[AUTO] Rocket INFLIGHT before first acquire — done");
        auto_acquire_state = AutoAcquireState::DONE;
        return;
    }
    if (last_packet_ms != 0)
    {
        ESP_LOGI(TAG, "[AUTO] First rocket packet acquired — locked on %.2f MHz "
                      "for this flight (no scan, no move)",
                 (double)lora_freq_mhz);
        auto_acquire_state = AutoAcquireState::DONE;
    }
}

static void setup_bs()
{
    delay(500);
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  TinkerRocket Base Station");
    // Board banner = the wrong-build-dir flash guard (same lesson as the
    // FC/OC -B build_v8 gotcha): if this line doesn't match the physical
    // board, stop and flash the right variant.
    ESP_LOGI(TAG, "  Board: %s (TR_BS_BOARD=%d)", config::BOARD_NAME, TR_BS_BOARD);
    ESP_LOGI(TAG, "======================================");

    // OTA boot-state check (#8). If this image was just OTA-installed it
    // boots PENDING_VERIFY; we hold off the "valid" mark until we've seen
    // BLE work end-to-end (first telemetry sent while connected). If the
    // app crashes or hangs before that, the bootloader auto-rolls back to
    // ota_0 on the next boot.
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

    // BLE time-sync arrives as broken-down UTC; pin TZ so mktime() in the
    // log-rename helper (#168) interprets it as UTC seconds, not local.
    setenv("TZ", "UTC0", 1);
    tzset();

    // Initialize storage. V2/V3 -> wear-leveled FAT on the external SPI NAND;
    // V1 -> SD over SDMMC 4-bit. Either way, fall back to SPIFFS on
    // internal flash. Logging is backend-agnostic (uses SD_MOUNT_POINT + the
    // standard file API), so only the mount differs per board.
    {
        esp_err_t ret = ESP_ERR_NOT_FOUND;
        if (config::HAS_EXT_NAND)
        {
            ret = mountExternalFlashFat();   // logs its own info; repoints SD_MOUNT_POINT on success
        }
        else if (config::HAS_SDMMC)
        {
            sdmmc_host_t host = SDMMC_HOST_DEFAULT();
            host.max_freq_khz = SDMMC_FREQ_DEFAULT;

            sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
            slot.width = 4;
            slot.clk = (gpio_num_t)config::SD_CLK;
            slot.cmd = (gpio_num_t)config::SD_CMD;
            slot.d0  = (gpio_num_t)config::SD_D0;
            slot.d1  = (gpio_num_t)config::SD_D1;
            slot.d2  = (gpio_num_t)config::SD_D2;
            slot.d3  = (gpio_num_t)config::SD_D3;
            slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

            esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {};
            mount_cfg.format_if_mount_failed = false;
            mount_cfg.max_files = 5;
            mount_cfg.allocation_unit_size = 16 * 1024;

            ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot, &mount_cfg, &sd_card);
            if (ret == ESP_OK)
            {
                sdmmc_card_print_info(stdout, sd_card);

                FATFS* fs;
                DWORD free_clust;
                if (f_getfree("0:", &free_clust, &fs) == FR_OK)
                {
                    uint64_t total = (uint64_t)(fs->n_fatent - 2) * fs->csize * fs->ssize;
                    uint64_t free_bytes = (uint64_t)free_clust * fs->csize * fs->ssize;
                    uint64_t used = total - free_bytes;
                    ESP_LOGI(TAG, "SD card mounted: %llu MB total, %llu MB used, %llu MB free",
                             (unsigned long long)(total / (1024 * 1024)),
                             (unsigned long long)(used / (1024 * 1024)),
                             (unsigned long long)(free_bytes / (1024 * 1024)));
                }
            }
        }
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Primary storage mount failed (0x%x) — falling back to internal flash (SPIFFS)",
                     (int)ret);
            sd_card = nullptr;

            esp_vfs_spiffs_conf_t spiffs_conf = {};
            spiffs_conf.base_path              = "/flash";
            spiffs_conf.partition_label        = SPIFFS_PARTITION_LABEL;
            spiffs_conf.max_files              = 5;
            spiffs_conf.format_if_mount_failed = true;

            esp_err_t sret = esp_vfs_spiffs_register(&spiffs_conf);
            if (sret != ESP_OK)
            {
                ESP_LOGE(TAG, "SPIFFS fallback mount FAILED (0x%x) — no logging available",
                         (int)sret);
            }
            else
            {
                SD_MOUNT_POINT = "/flash";
                using_internal_flash = true;

                size_t total = 0, used = 0;
                if (esp_spiffs_info(SPIFFS_PARTITION_LABEL, &total, &used) == ESP_OK)
                {
                    ESP_LOGI(TAG, "SPIFFS mounted at %s: %u KB total, %u KB used, %u KB free",
                             SD_MOUNT_POINT,
                             (unsigned)(total / 1024),
                             (unsigned)(used / 1024),
                             (unsigned)((total - used) / 1024));
                }
            }
        }

        // Write test covers both backends
        if (sd_card || using_internal_flash || using_external_flash)
        {
            char test_path[48];
            snprintf(test_path, sizeof(test_path), "%s/.write_test", SD_MOUNT_POINT);
            FILE* test = fopen(test_path, "w");
            if (test)
            {
                fprintf(test, "ok\n");
                fclose(test);
                remove(test_path);
                ESP_LOGI(TAG, "Storage write test: OK (%s)",
                         using_internal_flash ? "internal flash" : using_external_flash ? "external NAND" : "SD card");
            }
            else
            {
                ESP_LOGE(TAG, "Storage write test FAILED! errno=%d (%s)",
                         errno, strerror(errno));
            }
        }
    }

    // Configure I2C and auto-detect the fuel gauge (one firmware, both PCBs):
    // probe the new-PCB BQ27Z746 (0x55) first, then the original MAX17205 (0x36).
    {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port     = I2C_NUM_0;
        bus_cfg.sda_io_num   = (gpio_num_t)config::I2C_SDA_PIN;
        bus_cfg.scl_io_num   = (gpio_num_t)config::I2C_SCL_PIN;
        bus_cfg.clk_source   = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.flags.enable_internal_pullup = false;

        esp_err_t err = i2c_new_master_bus(&bus_cfg, &i2c_bus);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "I2C bus init failed (%d) — battery readings unavailable", (int)err);
        }
        else if (i2c_master_probe(i2c_bus, config::BQ27Z746_ADDR, pdMS_TO_TICKS(50)) == ESP_OK)
        {
            // New PCB: BQ27Z746 single-cell gauge + protection.
            TR_BQ27Z746_Config bq_cfg;
            bq_cfg.current_invert   = false;  // TODO: verify SRP/SRN polarity on the new board
            bq_cfg.auto_enable_fets = false;  // FET enable/maintenance owned by maintainBatteryFets()
                                              //   (runs via updateBattery() at boot + every cycle)
            if (bq_gauge.begin(i2c_bus, bq_cfg, config::I2C_FREQ_HZ) == ESP_OK)
            {
                gauge_kind = GaugeKind::BQ27Z746;
                fuel_gauge_present = true;
                ESP_LOGI(TAG, "BQ27Z746 fuel gauge found on I2C (0x%02X), devtype 0x%04X",
                         config::BQ27Z746_ADDR, bq_gauge.deviceType());
                updateBattery();
                ESP_LOGI(TAG, "Battery: %.2f V, %.1f%% SoC, %.0f mA",
                         (double)bs_voltage, (double)bs_soc, (double)bs_current);
                bq_gauge.logDiagnostics(TAG);

                // New-PCB gauge bring-up: the BQ27Z746 ships with a 5300 mAh
                // default DesignCapacity and was never configured for our 2800 mAh
                // 18650, so SoC/capacity are meaningless until provisioned. One-shot,
                // self-gated (no-op once correct), read-back verified.
                bq_gauge.provisionDesignCapacity((int16_t)config::BATTERY_DESIGN_MAH);
                // Read-only diagnostic: raw coulomb-counter ADC. If this tracks real
                // battery current (charge vs discharge) the current path is good and
                // only CC Gain needs a known-current calibration; if it's frozen, the
                // shunt/Kelvin sense is the problem.
                int16_t bq_raw_cc = 0;
                (void)bq_gauge.readRawCcCurrent(bq_raw_cc);
            }
            else
            {
                ESP_LOGW(TAG, "BQ27Z746 probe succeeded but begin() failed");
            }
        }
        else if (i2c_master_probe(i2c_bus, config::MAX17205_ADDR, pdMS_TO_TICKS(50)) == ESP_OK)
        {
            // 0x36 is shared by two ModelGauge m5 parts: MAX17303 (V3 PCB,
            // 1S + protector) and MAX17205 (V1 PCB, 2S). Try the MAX17303
            // driver first — its begin() reads DevName and hands the address
            // back (ESP_ERR_NOT_FOUND) when the part isn't a MAX1730x.
            TR_MAX17303_Config m3_cfg;
            m3_cfg.design_mah     = config::BATTERY_DESIGN_MAH;
            m3_cfg.rsense_mohm    = config::RSENSE_MOHM;   // TODO: verify V3 Rsense on the bench
            m3_cfg.current_invert = false;                  // TODO: verify CSP/CSN polarity on V3
            m3_cfg.assume_when_unidentified = config::EXPECT_MAX17303;

            if (max17303_gauge.begin(i2c_bus, m3_cfg, config::I2C_FREQ_HZ) == ESP_OK)
            {
                gauge_kind = GaugeKind::MAX17303;
                fuel_gauge_present = true;
                ESP_LOGI(TAG, "MAX17303 fuel gauge found on I2C (0x%02X), DevName 0x%04X",
                         config::MAX17303_ADDR, max17303_gauge.devName());
                max17303_gauge.initIfNeeded();
                updateBattery();
                ESP_LOGI(TAG, "Battery: %.2f V, %.1f%% SoC, %.0f mA",
                         (double)bs_voltage, (double)bs_soc, (double)bs_current);
                max17303_gauge.logDiagnostics(TAG);
            }
            else
            {
                // Original PCB: MAX17205 gauge.
                TR_MAX17205G_Config fg_cfg;
                fg_cfg.design_mah     = config::BATTERY_DESIGN_MAH;
                fg_cfg.rsense_mohm    = config::RSENSE_MOHM;
                fg_cfg.current_invert = true;   // CSP/CSN swapped on schematic (U7) vs.
                                                //   datasheet typical app circuit; flips
                                                //   displayed current so charge reads
                                                //   positive. Driver uses VFSOC for SoC
                                                //   because the chip's m5 can't be told.
                fg_cfg.num_cells      = config::NUM_BATTERY_CELLS;

                if (fuel_gauge.begin(i2c_bus, fg_cfg, config::I2C_FREQ_HZ) == ESP_OK)
                {
                    gauge_kind = GaugeKind::MAX17205;
                    fuel_gauge_present = true;
                    ESP_LOGI(TAG, "MAX17205G fuel gauge found on I2C (0x%02X)", config::MAX17205_ADDR);
                    fuel_gauge.initIfNeeded();
                    updateBattery();
                    ESP_LOGI(TAG, "Battery: %.2f V, %.1f%% SoC, %.0f mA",
                             (double)bs_voltage, (double)bs_soc, (double)bs_current);
                    fuel_gauge.logDiagnostics(TAG);
                }
                else
                {
                    ESP_LOGW(TAG, "MAX17205G probe succeeded but begin() failed");
                }
            }
        }
        else
        {
            ESP_LOGW(TAG, "No fuel gauge found (neither BQ27Z746 0x55 nor MAX17205/MAX17303 0x36) — battery readings unavailable");
        }
    }

    // V3: external flight-pack charger on the same I2C bus as the gauge.
    // begin() succeeds even with no charge input attached (the chip is
    // unpowered then); the periodic service() detects plug-in and applies
    // the charge config each time.
    if (config::HAS_PACK_CHARGER && i2c_bus != nullptr)
    {
        TR_MP2672_Config chg_cfg;
        chg_cfg.vbatt_reg_code = config::PACK_VBATT_REG_CODE;
        chg_cfg.icc_code       = config::PACK_CHARGE_ICC_CODE;
        chg_cfg.chg_timer_code = config::PACK_CHG_TIMER_CODE;
        if (pack_charger.begin(i2c_bus, chg_cfg, config::I2C_FREQ_HZ) != ESP_OK)
        {
            ESP_LOGW(TAG, "MP2672 pack-charger driver init failed");
        }
    }

    // Load saved LoRa config from NVS (write config.h defaults if empty)
    prefs.begin("lora", false);  // read-write

    // NVS schema gate (#105 follow-up).  If the stored schema version
    // doesn't match the current build, wipe the lora namespace so the
    // device falls back to the shared factory rendezvous and re-syncs
    // with the BS via the standard rendezvous flow.  Without this, a
    // re-flashed device can come up on a stale operating freq the BS
    // doesn't know about — exactly the chicken-and-egg that motivated
    // this gate.  Single namespace clear is fine: every LoRa-related
    // key (freq, sf, bw, cr, txpwr, hopdis, chset_*, plus the now-defunct
    // rdv_mhz from v2 schema) lives under "lora", so one clear() takes
    // them all together.
    {
        const uint8_t stored_v = prefs.getUChar("schemv", 0);
        if (stored_v != LORA_NVS_SCHEMA_VERSION)
        {
            ESP_LOGW(TAG, "[CFG] LoRa NVS schema mismatch (stored=%u, current=%u) — clearing",
                     (unsigned)stored_v, (unsigned)LORA_NVS_SCHEMA_VERSION);
            prefs.clear();
            prefs.putUChar("schemv", LORA_NVS_SCHEMA_VERSION);
        }
    }

    if (!prefs.isKey("freq"))
    {
        // First boot or NVS erased — seed with config.h factory defaults
        prefs.putFloat("freq",  config::LORA_FREQ_MHZ);
        prefs.putUChar("sf",    config::LORA_SF);
        prefs.putFloat("bw",    config::LORA_BW_KHZ);
        prefs.putUChar("cr",    config::LORA_CR);
        prefs.putChar("txpwr",  config::LORA_TX_POWER_DBM);
        ESP_LOGI(TAG, "[CFG] LoRa NVS empty -- wrote config.h defaults");
    }
    lora_freq_mhz = prefs.getFloat("freq", config::LORA_FREQ_MHZ);
    lora_sf        = prefs.getUChar("sf",   config::LORA_SF);
    lora_bw_khz    = prefs.getFloat("bw",   config::LORA_BW_KHZ);
    lora_cr        = prefs.getUChar("cr",   config::LORA_CR);
    lora_tx_power  = (int8_t)prefs.getChar("txpwr", config::LORA_TX_POWER_DBM);
    lora_hop_disabled = prefs.getUChar("hopdis", 0) != 0;  // #106 fixed-frequency override
    prefs.end();
    ESP_LOGI(TAG, "[CFG] LoRa NVS (cached): %.1f MHz SF%u BW%.0f CR%u %d dBm hop_disabled=%d",
             (double)lora_freq_mhz, (unsigned)lora_sf,
             (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power,
             (int)lora_hop_disabled);

    // Issue #136: every BS power cycle starts on the hardcoded rendezvous
    // frequency + preset, regardless of any NVS values left over from
    // prior sessions.  This guarantees that BS and rocket meet on a known
    // channel before the BS scans + picks a quiet one.  cmd-10 commits
    // still write through to NVS during a session for visibility and
    // BLE-readback, but those values no longer drive boot config.
    // Hopping is also gated off at boot — re-enable via cmd 17 if you're
    // testing the hop state machine.  See the "Re-enable LoRa hopping"
    // follow-up enhancement.
    lora_freq_mhz     = LORA_FACTORY_RENDEZVOUS_MHZ;
    lora_sf           = LORA_FACTORY_RENDEZVOUS_SF;
    lora_bw_khz       = LORA_FACTORY_RENDEZVOUS_BW_KHZ;
    lora_cr           = LORA_FACTORY_RENDEZVOUS_CR;
    lora_tx_power     = LORA_FACTORY_RENDEZVOUS_TX_DBM;
    lora_hop_disabled = true;
    ESP_LOGI(TAG, "[CFG] LoRa boot (forced rendezvous): %.1f MHz SF%u BW%.0f CR%u %d dBm",
             (double)lora_freq_mhz, (unsigned)lora_sf,
             (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);

    // Channel-set: skip-mask from the most recent pre-launch scan
    // (#40 / #41 phase 3).  Falls back to no-skips if never scanned, or
    // if the stored mask was generated for a different BW.  Rendezvous
    // freq is no longer scan-selected; both sides use the shared
    // hardcoded LORA_FACTORY_RENDEZVOUS_MHZ (#105).
    loadChannelSetFromNvs();
    if (skip_mask_n_ > 0)
    {
        uint8_t active = 0;
        for (uint8_t i = 0; i < skip_mask_n_; i++)
            if (!loraSkipMaskTest(skip_mask_, i)) active++;
        ESP_LOGI(TAG, "[CHSET] NVS: %u/%u channels active",
                 (unsigned)active, (unsigned)skip_mask_n_);
    }
    else
    {
        ESP_LOGI(TAG, "[CHSET] NVS: no skip-mask");
    }

    // Load cached servo config from NVS
    prefs.begin("servo", true);
    cfg_servo_bias1 = prefs.getShort("b1",  cfg_servo_bias1);
    cfg_servo_hz    = prefs.getShort("hz",  cfg_servo_hz);
    cfg_servo_min   = prefs.getShort("min", cfg_servo_min);
    cfg_servo_max   = prefs.getShort("max", cfg_servo_max);
    prefs.end();

    // Load cached PID config from NVS
    prefs.begin("pid", true);
    cfg_pid_kp  = prefs.getFloat("kp",  cfg_pid_kp);
    cfg_pid_ki  = prefs.getFloat("ki",  cfg_pid_ki);
    cfg_pid_kd  = prefs.getFloat("kd",  cfg_pid_kd);
    cfg_pid_min = prefs.getFloat("mn",  cfg_pid_min);
    cfg_pid_max = prefs.getFloat("mx",  cfg_pid_max);
    prefs.end();
    ESP_LOGI(TAG, "[NVS] Config cache: servo bias=%d hz=%d, PID Kp=%.4f",
             cfg_servo_bias1, cfg_servo_hz, cfg_pid_kp);

    // --- Device Identity (NVS "identity" namespace) ---
    {
        uint8_t mac[6];
        esp_efuse_mac_get_default(mac);
        snprintf(unit_id_hex, sizeof(unit_id_hex), "%02x%02x%02x%02x",
                 mac[2], mac[3], mac[4], mac[5]);

        prefs.begin("identity", false);
        if (!prefs.isKey("un"))
        {
            char default_name[24];
            snprintf(default_name, sizeof(default_name), "TR-B-%.4s", &unit_id_hex[4]);
            prefs.putBytes("un", default_name, strlen(default_name) + 1);
            prefs.putUChar("nid", config::DEFAULT_NETWORK_ID);
            ESP_LOGI(TAG, "[CFG] Identity NVS empty — seeded: name=%s nid=%u",
                     default_name, config::DEFAULT_NETWORK_ID);
        }
        char nvs_name_buf[24] = "TinkerBaseStation";
        size_t un_len = prefs.getBytes("un", nvs_name_buf, sizeof(nvs_name_buf) - 1);
        if (un_len > 0) nvs_name_buf[un_len] = '\0';
        strncpy(unit_name, nvs_name_buf, sizeof(unit_name) - 1);
        unit_name[sizeof(unit_name) - 1] = '\0';
        network_id = prefs.getUChar("nid", config::DEFAULT_NETWORK_ID);
        prefs.end();

        // #136: every boot also forces network_id back to the hardcoded
        // default, mirroring the LoRa rendezvous override.  Without this,
        // a BS and OC can quietly drift apart whenever one device's NVS
        // is wiped (e.g., the LORA_NVS_SCHEMA_VERSION bump in #105 cleared
        // the lora namespace on this BS but left the OC's `identity` nid
        // alone, so OC stayed on nid=180 while the BS came back on nid=0
        // — every beacon and telemetry packet from the rocket got dropped
        // silently by the network_id filter).  Cmd 9 still lets a
        // developer change network_id at runtime, but boot always
        // re-resets, so devices can't drift across power cycles.
        // Reintroduce per-network IDs alongside hopping in #150.
        if (network_id != config::DEFAULT_NETWORK_ID)
        {
            ESP_LOGW(TAG, "[CFG] NVS nid=%u differs from default %u — forcing default",
                     (unsigned)network_id, (unsigned)config::DEFAULT_NETWORK_ID);
            network_id = config::DEFAULT_NETWORK_ID;
        }

        ESP_LOGI(TAG, "[CFG] Identity: uid=%s name=%s nid=%u",
                 unit_id_hex, unit_name, (unsigned)network_id);

        ble_app.setName(unit_name);
    }

    // Configure LoRa radio (uses NVS-saved config or factory defaults).
    // Backend per board header (#410 pattern): V3 talks to the radio
    // daughterboard over UART; V1/V2 drive the on-board LLCC68 over SPI.
    bool radio_ok = false;
    if (config::USE_UART_RADIO_MODEM)
    {
        UartModemBackend::Config mcfg = {};
        mcfg.uart.tx_pin      = config::LORA_UART_TX_PIN;
        mcfg.uart.rx_pin      = config::LORA_UART_RX_PIN;
        mcfg.act_pin          = config::LORA_ACT_PIN;
        mcfg.preamble_len     = config::LORA_PREAMBLE_LEN;
        mcfg.crc_on           = config::LORA_CRC_ON;
        mcfg.rx_boosted_gain  = config::LORA_RX_BOOSTED_GAIN;
        mcfg.syncword_private = config::LORA_SYNCWORD_PRIVATE;
        radio_ok = lora_modem_backend.begin(mcfg, lora_freq_mhz, lora_sf,
                                            lora_bw_khz, lora_cr,
                                            lora_tx_power, config::DEBUG);
        if (!radio_ok)
        {
            // Unlike the direct-SPI boards, a missing daughterboard must not
            // brick the BS: bench bring-up runs gauge/charger/storage/BLE
            // without a radio, and the backend hot-joins a modem that BOOTs
            // later (service() keeps polling the UART).
            ESP_LOGE(TAG, "LoRa modem init FAILED — continuing radio-less "
                          "(daughterboard can hot-join)");
        }
    }
    else
    {
        TR_LoRa_Comms::Config lora_cfg = {};
        lora_cfg.enabled           = true;
        lora_cfg.cs_pin            = config::LORA_CS_PIN;
        lora_cfg.dio1_pin          = config::LORA_DIO1_PIN;
        lora_cfg.rst_pin           = config::LORA_RST_PIN;
        lora_cfg.busy_pin          = config::LORA_BUSY_PIN;
        // V2 PCB: MCU-driven RX half of the RF switch (was defined but never
        // driven — RXEN floated in RX). -1 on the original PCB (no switch).
        lora_cfg.rxen_pin          = config::LORA_RXEN_PIN;
        lora_cfg.spi_sck           = config::LORA_SPI_SCK;
        lora_cfg.spi_miso          = config::LORA_SPI_MISO;
        lora_cfg.spi_mosi          = config::LORA_SPI_MOSI;
        lora_cfg.spi_host          = SPI2_HOST;
        lora_cfg.freq_mhz          = lora_freq_mhz;
        lora_cfg.spreading_factor  = lora_sf;
        lora_cfg.bandwidth_khz     = lora_bw_khz;
        lora_cfg.coding_rate       = lora_cr;
        lora_cfg.preamble_len      = config::LORA_PREAMBLE_LEN;
        lora_cfg.tx_power_dbm      = lora_tx_power;
        lora_cfg.crc_on            = config::LORA_CRC_ON;
        lora_cfg.rx_boosted_gain   = config::LORA_RX_BOOSTED_GAIN;
        lora_cfg.syncword_private  = config::LORA_SYNCWORD_PRIVATE;

        radio_ok = lora_direct_backend.begin(lora_cfg, config::DEBUG);
        if (!radio_ok)
        {
            // On-board radio is soldered to this PCB — a failed init is a
            // hardware fault, keep the historical hard stop.
            ESP_LOGE(TAG, "LoRa init FAILED!");
            while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
        }
    }

    ESP_LOGI(TAG, "LoRa config: %.1f MHz SF%u BW%.0f kHz CR%u %d dBm",
             (double)lora_freq_mhz,
             (unsigned)lora_sf,
             (double)lora_bw_khz,
             (unsigned)lora_cr,
             (int)lora_tx_power);

    // Start continuous receive mode
    if (radio_ok && !lora_comms.startReceive())
    {
        ESP_LOGE(TAG, "LoRa startReceive FAILED!");
        if (!config::USE_UART_RADIO_MODEM)
        {
            while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
        }
    }

    // Initialize BLE app interface
    if (!ble_app.begin())
    {
        ESP_LOGE(TAG, "BLE init FAILED!");
        // Continue anyway - LoRa RX still works without BLE
    }
    else
    {
        ESP_LOGI(TAG, "BLE advertising as 'TinkerBaseStation'");
    }

    last_stats_ms = millis();
    ESP_LOGI(TAG, "Listening for rocket telemetry...");
}

static void loop_bs()
{
    // #289: set when the RX path sends a (fresh) telemetry notification this
    // iteration, so the periodic battery push below can skip a redundant second
    // notify — two back-to-back sends double the notify pressure exactly when
    // the queue may be full, making an RX-path drop more likely.
    bool rx_sent_telem_this_iter = false;

    // Service LoRa (complete any pending TX before checking for RX)
    lora_comms.service();
    lora_comms.pollDio1();          // Fallback if DIO1 interrupt doesn't fire
    lora_comms.serviceTxWatchdog(); // Force-clear stuck tx_ongoing_ (#105)

    // Drive the coordinated-scan state machine first so AWAITING_PAUSE
    // → SCANNING transitions can kick off a scan in the same iteration
    // serviceUplink finishes the cmd 16 retries (#90).
    serviceCoordinatedScan();

    // Hop-silence fallback: if we've been following a hopping rocket but
    // haven't heard from it in HOP_SILENCE_FALLBACK_MS, drop back to the
    // static channel so the standard silence / recovery machinery can
    // take over.  Without this, hop_active_ would pin the BS to a hop
    // channel forever after a lost rocket — recovery itself is
    // suppressed while hop_active_ for the opposite reason (don't
    // recover during normal hop misses).  Suppressed during a
    // coordinated scan (#90): the BS sweeps off the hop channel and the
    // rocket is parked on lora_freq_mhz, so silence is expected.
    if (hop_active_ && coord_scan_state_ == CoordScanState::IDLE &&
        hop_last_rx_ms_ != 0 &&
        (millis() - hop_last_rx_ms_) > HOP_SILENCE_FALLBACK_MS)
    {
        const uint32_t silence_ms = millis() - hop_last_rx_ms_;
        const uint32_t dur_ms     = (hop_session_started_ms != 0)
            ? (millis() - hop_session_started_ms) : 0;
        ESP_LOGW(TAG, "[HOP] Silence > %u ms — falling back to static channel",
                 (unsigned)HOP_SILENCE_FALLBACK_MS);
        hop_silence_events_count++;
        char ev[96];
        snprintf(ev, sizeof(ev),
                 "hop_silence duration_ms=%u pkts=%u loss=%u silence_ms=%u",
                 (unsigned)dur_ms,
                 (unsigned)hop_session_total_pkts,
                 (unsigned)hop_session_observed_loss,
                 (unsigned)silence_ms);
        // Log the event row BEFORE flipping hop_active_ off so currentRxFreqMHz()
        // still reports the hop channel we were sitting on at the moment of loss.
        logHopEvent(ev, currentRxFreqMHz());
        hop_session_started_ms    = 0;
        hop_session_total_pkts    = 0;
        hop_session_observed_loss = 0;
        hop_active_       = false;
        hop_needs_retune_ = true;
    }

    // Honour any pending hop retune — the RX path defers the actual
    // setFrequency call to here so the radio finishes any in-flight
    // operation first (#40 / #41 phase 2a).
    if (hop_needs_retune_ && lora_comms.canSend())
    {
        const float target = hop_active_
            ? loraChannelMHz(lora_bw_khz, hop_idx_)
            : lora_freq_mhz;
        if (target > 0.0f)
        {
            (void)lora_comms.hopToFrequencyMHz(target);
        }
        hop_needs_retune_ = false;
    }

    // Check for received packet
    uint8_t rx_buf[256];
    size_t rx_len = 0;

    bool rx_packet_accepted = false;
    if (lora_comms.readPacket(rx_buf, sizeof(rx_buf), rx_len))
    {
        // SNR floor (#90 follow-up).  Defends against noise-floor
        // false-positive decodes that confuse recovery + hop tracking.
        // The threshold tracks the radio's currently-configured SF
        // because Phase A rendezvous can run at a different SF than
        // operating mode.
        TR_LoRa_Comms::Stats ls_pre = {};
        lora_comms.getStats(ls_pre);
        const float min_snr = loraMinValidSnrDb(lora_comms.currentSpreadingFactor());
        if (ls_pre.last_snr < min_snr)
        {
            lora_low_snr_drops++;
            ESP_LOGW(TAG, "[RX] Drop: SNR %.1f dB < %.1f dB floor "
                          "(SF%u) — likely noise-floor false positive",
                     (double)ls_pre.last_snr, (double)min_snr,
                     (unsigned)lora_comms.currentSpreadingFactor());
            // Fall through; the `if (rx_packet_accepted)` guard below
            // skips state updates while letting the rest of loop_bs
            // (serviceUplink etc.) keep running on schedule.
        }
        else
        {
            rx_packet_accepted = true;
        }
    }

    if (rx_packet_accepted)
    {
        // #384: last_packet_ms deliberately NOT bumped here — this point is
        // before the network-id filter, so foreign-network packets (or any
        // SNR-passing decode of the right shape) counted as proof of life.
        // That suppressed silence recovery and, worse, satisfied the cmd-10
        // VERIFYING predicate ("any last_packet_ms increase = the rocket
        // followed us"), committing a new frequency to NVS on the strength
        // of somebody else's traffic and stranding the BS on a channel our
        // rocket isn't on. The bump now lives in the two netid-matched
        // branches below (beacon + telemetry).

        // --- Name beacon: [0xBE][network_id][rocket_id][unit_name...] ---
        // #384: a telemetry frame whose first byte (network_id) happens to
        // equal LORA_BEACON_SYNC (0xBE = nid 190) would otherwise parse as a
        // beacon and shadow 100% of telemetry. Inert today (boot forces
        // nid 0), so require the length to disambiguate: beacons are short,
        // telemetry is exactly SIZE_OF_LORA_DATA.
        if (rx_len >= 3 && rx_len != SIZE_OF_LORA_DATA && rx_buf[0] == LORA_BEACON_SYNC)
        {
            uint8_t bcn_nid = rx_buf[1];
            uint8_t bcn_rid = rx_buf[2];
            if (bcn_nid == network_id)
            {
                last_packet_ms = millis();  // netid-matched proof of life (#384)
                int slot = findOrAllocRocket(bcn_rid);
                if (slot >= 0 && rx_len > 3)
                {
                    size_t name_len = rx_len - 3;
                    if (name_len >= sizeof(tracked_rockets[slot].unit_name))
                        name_len = sizeof(tracked_rockets[slot].unit_name) - 1;
                    memcpy(tracked_rockets[slot].unit_name, &rx_buf[3], name_len);
                    tracked_rockets[slot].unit_name[name_len] = '\0';
                    tracked_rockets[slot].last_seen_ms = millis();
                    ESP_LOGI(TAG, "[RX] Beacon: rid=%u name=%s",
                             bcn_rid, tracked_rockets[slot].unit_name);
                }
            }
        }
        // --- Telemetry packet: SIZE_OF_LORA_DATA (66) bytes ---
        else if (rx_len == SIZE_OF_LORA_DATA)
        {
            // Decode the telemetry packet
            LoRaDataSI decoded = {};
            sensor_converter.unpackLoRa(rx_buf, decoded);

            // Filter by network_id
            if (decoded.network_id != network_id)
            {
                // Not our network — drop.  Previously fully silent (#329): if
                // the rocket's nid drifts from the BS default (0, #136), every
                // packet lands here and the flight log stays empty with no
                // clue why.  Count it + a throttled warning so it's attributable.
                lora_netid_mismatch_drops++;
                if (lora_netid_mismatch_drops == 1 ||
                    lora_netid_mismatch_drops % 100 == 0)
                {
                    ESP_LOGW(TAG, "[RX] Drop: network_id %u != ours %u "
                                  "(%lu dropped) — rocket on a different network?",
                             (unsigned)decoded.network_id, (unsigned)network_id,
                             (unsigned long)lora_netid_mismatch_drops);
                }
            }
            else
            {
                last_packet_ms = millis();  // netid-matched proof of life (#384)

                // Route to per-rocket tracker
                int slot = findOrAllocRocket(decoded.rocket_id);

                TR_LoRa_Comms::Stats ls = {};
                lora_comms.getStats(ls);

                // Observed-loss attribution (#105).  Use the rocket's free-
                // running seq to compute how many telemetry packets we missed
                // since the last RX from this rocket.  loraComputeObservedLoss
                // returns -1 on first contact, duplicates, or implausible
                // forward deltas (most likely a rocket reboot — lora_tx_seq
                // resets to 0).  We update last_seq unconditionally so the
                // next RX gets a clean baseline either way.
                int32_t observed_gap = -1;
                if (slot >= 0) {
                    observed_gap = loraComputeObservedLoss(
                        tracked_rockets[slot].last_seq, decoded.seq);
                    tracked_rockets[slot].last_seq = (int32_t)decoded.seq;
                    if (observed_gap > 0) {
                        lora_total_observed_loss += (uint32_t)observed_gap;
                        if (hop_active_) {
                            hop_session_observed_loss += (uint32_t)observed_gap;
                        }
                    }
                    if (hop_active_) hop_session_total_pkts++;
                }

                // Convert ECEF to lat/lon — but only when the rocket reports
                // an actual GPS fix.  The rocket sometimes packs nonzero ECEF
                // even with num_sats == 0 (stale GNSS register read), which
                // would otherwise yield a "valid-looking" lat/lon while the
                // fix is invalid (#95).
                double lat_deg = NAN, lon_deg = NAN, alt_m = NAN;
                if (decoded.num_sats > 0 &&
                    (decoded.ecef_x != 0.0 || decoded.ecef_y != 0.0 || decoded.ecef_z != 0.0))
                {
                    coord.ecefToGeodetic(decoded.ecef_x, decoded.ecef_y, decoded.ecef_z,
                                         lat_deg, lon_deg, alt_m);
                }

                printTelemetry(decoded, ls.last_rssi, ls.last_snr, lat_deg, lon_deg, alt_m);

                // Base station CSV logging policy (#107, refined in #168,
                // multi-rocket in #381):
                //   • Start a log whenever a non-LANDED packet arrives and
                //     we aren't already logging.  Every flight state counts
                //     — even READY pre-flight setup is worth keeping.
                //   • Refuse to auto-open while the rocket reports LANDED
                //     (#168).  Without this gate, the close-on-LANDED-
                //     transition (below) would be immediately undone by
                //     the next LANDED packet, generating a remnant file
                //     that only contains post-landing telemetry — that's
                //     the LANDED-only files in the 5/17/26 flight folders.
                //     Operators can still capture LANDED telemetry via the
                //     manual BLE cmd 23 start, which bypasses this gate.
                //   • Close on a rocket's LANDED transition ONLY when it is
                //     the last fresh rocket flying (#381): state edges are
                //     tracked per tracked_rockets slot, so a landed rocket's
                //     repeat LANDED packets interleaved with another rocket's
                //     READY no longer read as close/reopen transitions once
                //     per packet pair (the one-file-per-second shred).
                //   • Throttled by LOG_OPEN_RETRY_MS so a persistent
                //     fopen() failure (wedged SD) doesn't log-spam.
                if (!logging_active && !log_manual_inhibit &&
                    decoded.rocket_state != LANDED &&
                    (log_last_open_attempt_ms == 0 ||
                     (millis() - log_last_open_attempt_ms) >= config::LOG_OPEN_RETRY_MS))
                {
                    log_last_open_attempt_ms = millis();
                    startLogging();
                }

                last_known_rocket_logging = decoded.logging_active;

                if (logging_active)
                {
                    logLoRaPacket(decoded, ls.last_rssi, ls.last_snr,
                                  lat_deg, lon_deg, alt_m,
                                  currentRxFreqMHz(), observed_gap);
                }

                if (slot >= 0)
                {
                    const uint32_t now_ms = millis();
                    auto& rls = tracked_rockets[slot].log_state;
                    const bool was_inflight_armed = (rls.inflight_entry_ms != 0);

                    // Fold this packet into the rocket's own transition state:
                    // detects edges, arms/disarms its INFLIGHT safety timer,
                    // latches its freq lock (bs_log_policy #381).
                    const auto edges =
                        bs_log_policy::updateRocketLogState(rls,
                                                            decoded.rocket_state,
                                                            now_ms);
                    if (!was_inflight_armed && rls.inflight_entry_ms != 0)
                    {
                        ESP_LOGI(TAG, "[LOG] Rocket %u INFLIGHT entry — safety timer armed (%u min)",
                                 (unsigned)decoded.rocket_id,
                                 (unsigned)(config::LOG_INFLIGHT_SAFETY_MS / 60000));
                    }

                    // Freshness for the aggregate decisions below uses
                    // last_seen_ms; stamp it now (the tracker mirror further
                    // down stamps it again — idempotent).
                    tracked_rockets[slot].last_seen_ms = now_ms;

                    bs_log_policy::RocketView views[MAX_TRACKED_ROCKETS];
                    buildRocketViews(views);
                    freq_locked_for_flight = bs_log_policy::aggregateFreqLock(
                        views, MAX_TRACKED_ROCKETS, now_ms,
                        config::LOG_SILENCE_TIMEOUT_MS);

                    // Close on LANDED transition — but only when no other
                    // fresh rocket is still flying (#381). Boot edge is
                    // handled inside updateRocketLogState (no edge on a
                    // rocket's first packet), so a post-flight BS reboot
                    // seeing LANDED first doesn't produce a zero-byte file.
                    if (edges.landed_edge && logging_active)
                    {
                        if (bs_log_policy::noFreshRocketFlying(
                                views, MAX_TRACKED_ROCKETS, now_ms,
                                config::LOG_SILENCE_TIMEOUT_MS,
                                config::LOG_INFLIGHT_SAFETY_MS))
                        {
                            ESP_LOGI(TAG, "[LOG] Rocket %u LANDED (last fresh rocket) — closing flight log",
                                     (unsigned)decoded.rocket_id);
                            stopLogging();
                        }
                        else
                        {
                            ESP_LOGI(TAG, "[LOG] Rocket %u LANDED — log stays open (another rocket still active)",
                                     (unsigned)decoded.rocket_id);
                        }
                    }

                    // This rocket changing state clears the manual stop
                    // inhibit so the next flight gets logged automatically
                    // (#107). Keyed per rocket: a second rocket's steady
                    // stream no longer fabricates state changes.
                    if (edges.state_changed && log_manual_inhibit)
                    {
                        log_manual_inhibit = false;
                        ESP_LOGI(TAG, "[LOG] State change cleared manual stop inhibit");
                    }
                }

                // Hop state follow (#40 / #41 phase 2a, seq-anchored in
                // #105 follow-up).  The rocket and the BS both compute
                // the channel for any seq from loraHopChannelForSeq() —
                // identical formula, no chained handoff.  This means a
                // single missed packet within a dwell window doesn't
                // desync us: the next received packet's seq tells us
                // exactly where the rocket is.  next_channel_idx in the
                // packet header is now a sanity hint; we still honour
                // the NO_HOP sentinel as the "not currently hopping"
                // signal, but for the actual channel we consult seq.
                //
                // #106: when the operator has disabled hopping, ignore
                // the rocket's hop entirely and stay on lora_freq_mhz.
                if (!lora_hop_disabled && shouldHopInState(decoded.rocket_state))
                {
                    if (decoded.next_channel_idx != LORA_NEXT_CH_NO_HOP)
                    {
                        const uint8_t n = loraChannelCount(lora_bw_khz);
                        if (n > 0)
                        {
                            const bool mask_valid = (skip_mask_n_ == n &&
                                                      channel_set_bw_khz_ == lora_bw_khz);
                            static const uint8_t empty_mask[LORA_SKIP_MASK_MAX_BYTES] = {0};
                            const uint8_t* mask = mask_valid ? skip_mask_ : empty_mask;
                            // Compute next channel from seq+1 — what the
                            // rocket will be on for the *next* packet.
                            // Trust the rocket's announced next_channel_idx
                            // as the retune target — that's where the rocket
                            // will actually transmit, regardless of whether
                            // our local skip-mask agrees with theirs.  The
                            // seq-derived computation below is a sanity
                            // CHECK, not the source of truth: if it disagrees
                            // with what the rocket announced, the masks have
                            // drifted (cmd-15 push didn't reach this side, or
                            // the BS just ran a fresh scan and the rocket
                            // hasn't received the new mask yet).  In that
                            // case the mask-warning fires but we still follow
                            // the rocket so the link stays up.
                            const uint8_t expected_next = loraHopChannelForSeq(
                                (uint16_t)(decoded.seq + 1),
                                LORA_HOP_DWELL_PACKETS, mask, n);
                            if (decoded.next_channel_idx != expected_next) {
                                ESP_LOGW(TAG, "[HOP] seq=%u: rocket says next=%u, "
                                              "we'd compute next=%u (mask drift "
                                              "— following rocket)",
                                         (unsigned)decoded.seq,
                                         (unsigned)decoded.next_channel_idx,
                                         (unsigned)expected_next);
                                // Schedule auto re-push of the mask via
                                // cmd 15 — the rocket's mask is out of
                                // sync with ours, fixed by re-sending
                                // ours.  Cooldown enforced in
                                // serviceMaskDriftRepush.
                                chset_drift_repush_pending = true;
                            }
                            const bool was_active = hop_active_;
                            hop_idx_           = decoded.next_channel_idx;
                            hop_active_        = true;
                            hop_needs_retune_  = true;
                            hop_last_rx_ms_    = millis();
                            if (!was_active)
                            {
                                // Start a fresh hop-session counter window
                                // so the eventual hop_inactive / hop_silence
                                // event reports stats for *this* session
                                // (#105 diagnostics).
                                hop_session_started_ms    = millis();
                                hop_session_total_pkts    = 1;  // this packet counts
                                hop_session_observed_loss = 0;
                                ESP_LOGI(TAG, "[HOP] Active: %u channels at BW=%.0f kHz "
                                              "dwell=%u, first hop -> idx=%u (%.3f MHz)",
                                         (unsigned)n, (double)lora_bw_khz,
                                         (unsigned)LORA_HOP_DWELL_PACKETS,
                                         (unsigned)hop_idx_,
                                         (double)loraChannelMHz(lora_bw_khz, hop_idx_));
                                char ev[96];
                                snprintf(ev, sizeof(ev),
                                         "hop_active idx=%u nch=%u dwell=%u",
                                         (unsigned)hop_idx_, (unsigned)n,
                                         (unsigned)LORA_HOP_DWELL_PACKETS);
                                logHopEvent(ev,
                                            loraChannelMHz(lora_bw_khz, hop_idx_));
                            }
                        }
                    }
                }
                else if (hop_active_)
                {
                    // Rocket is no longer in a hop state — return to the
                    // static configured channel so recovery / ground
                    // comms resume on a known frequency.
                    hop_active_       = false;
                    hop_needs_retune_ = true;
                    const uint32_t dur_ms = (hop_session_started_ms != 0)
                        ? (millis() - hop_session_started_ms) : 0;
                    ESP_LOGI(TAG, "[HOP] Inactive (rocket state changed): "
                                  "returning to %.2f MHz", (double)lora_freq_mhz);
                    char ev[96];
                    snprintf(ev, sizeof(ev),
                             "hop_inactive duration_ms=%u pkts=%u loss=%u "
                             "reason=state",
                             (unsigned)dur_ms,
                             (unsigned)hop_session_total_pkts,
                             (unsigned)hop_session_observed_loss);
                    logHopEvent(ev, lora_freq_mhz);
                    hop_session_started_ms    = 0;
                    hop_session_total_pkts    = 0;
                    hop_session_observed_loss = 0;
                }

                last_known_camera_recording = decoded.camera_recording;

                // Update per-rocket tracker
                if (slot >= 0)
                {
                    tracked_rockets[slot].last_data = decoded;
                    tracked_rockets[slot].last_rssi = ls.last_rssi;
                    tracked_rockets[slot].last_snr  = ls.last_snr;
                    tracked_rockets[slot].last_lat_deg = lat_deg;
                    tracked_rockets[slot].last_lon_deg = lon_deg;
                    tracked_rockets[slot].last_alt_m   = alt_m;
                    tracked_rockets[slot].last_seen_ms = millis();
                    active_rocket_idx = (uint8_t)slot;
                }

                // Forward telemetry to BLE app (with rocket_id for app-side demux)
                TR_BLE_To_APP::TelemetryData ble_telem = {};
                buildBLETelemetry(decoded, ls.last_rssi, ls.last_snr,
                                  lat_deg, lon_deg, alt_m, ble_telem);
                if (slot >= 0 && tracked_rockets[slot].unit_name[0]) {
                    ble_telem.source_unit_name = tracked_rockets[slot].unit_name;
                }
                ble_app.sendTelemetry(ble_telem);
                rx_sent_telem_this_iter = true;   // #289
                maybeMarkOtaValid();
            }
        }
        else
        {
            ESP_LOGW(TAG, "[RX] Unexpected packet size: %u (expected %u or beacon)",
                     (unsigned)rx_len, (unsigned)SIZE_OF_LORA_DATA);
        }
    }

    // INFLIGHT safety timeout (#107, per-rocket in #381): close the log if a
    // rocket has been in INFLIGHT too long without a LANDED — lost-LoRa-
    // during-descent / stuck-rocket-state-machine — UNLESS another fresh
    // rocket is still flying (its own flight keeps the file open; the expired
    // rocket already counts as "presumed down" in that aggregate, so the log
    // closes when the last fresh rocket lands). Timers disarm in
    // stopLogging(). Does NOT inhibit the auto-restart on the next packet —
    // a rocket genuinely stuck INFLIGHT past the cap starts a fresh file and
    // the operator sees two files instead of one.
    if (logging_active)
    {
        const uint32_t now_ms = millis();
        bs_log_policy::RocketView views[MAX_TRACKED_ROCKETS];
        buildRocketViews(views);
        if (bs_log_policy::anySafetyExpired(views, MAX_TRACKED_ROCKETS, now_ms,
                                            config::LOG_INFLIGHT_SAFETY_MS) &&
            bs_log_policy::noFreshRocketFlying(views, MAX_TRACKED_ROCKETS, now_ms,
                                               config::LOG_SILENCE_TIMEOUT_MS,
                                               config::LOG_INFLIGHT_SAFETY_MS))
        {
            ESP_LOGW(TAG, "[LOG] INFLIGHT safety timeout (%u min), closing log",
                     (unsigned)(config::LOG_INFLIGHT_SAFETY_MS / 60000));
            stopLogging();
        }
    }

    // Silence timeout: close log if no packets for LOG_SILENCE_TIMEOUT_MS
    // (5 min, #137).  Applies in every rocket state — pre-#137 this was 30 s
    // which closed mid-flight on altitude-driven RX gaps near apogee, but
    // bumping to 5 min handles even the worst-case descent silence on the
    // Estes-class flights we test with while still flushing the log a
    // reasonable interval after the rocket is powered off post-recovery.
    if (logging_active &&
        (millis() - log_last_write_ms) >= config::LOG_SILENCE_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "[LOG] Silence timeout (state=%s), closing log file",
                 rocketStateToString(lastSeenRocketState()));
        stopLogging();
    }

    // Periodic flush to flash.  fflush() only pushes stdio buffers down to
    // the OS — fsync() forces FATFS to commit dirty sectors to the SD card,
    // so a power loss within the flush window doesn't lose buffered
    // telemetry.  Skip fsync on SPIFFS fallback: SPIFFS persists writes
    // synchronously and fsync is a no-op there.  (#107)
    if (logging_active && (millis() - log_last_flush_ms) >= config::LOG_FLUSH_INTERVAL_MS)
    {
        if (log_file)
        {
            // #329: check the return values.  A full card typically doesn't
            // fail at fprintf() (it buffers) — it fails HERE, when fflush/fsync
            // push the buffered rows to FATFS (ENOSPC).  Ignoring these returns
            // is exactly how a flight's telemetry can vanish with no warning.
            bool ok = (fflush(log_file) == 0);
            if (ok && !using_internal_flash && fsync(fileno(log_file)) != 0) ok = false;
            if (!ok)
            {
                log_write_fail_count++;
                ESP_LOGE(TAG, "[LOG] flush/fsync failed (errno=%d %s) — buffered "
                              "rows may be lost (card full?)", errno, strerror(errno));
            }
        }
        log_last_flush_ms = millis();
    }

    // Periodic battery read + standalone BLE update
    if (millis() - last_battery_ms >= config::PWR_UPDATE_PERIOD_MS)
    {
        last_battery_ms = millis();
        // Skip the I2C gauge poll while an OTA is in flight (#17): the
        // esp_ota_begin() partition erase blocks the SPI flash for ~1-2 s
        // and the gauge transaction collides with it, logging spurious
        // i2c_master_transmit_receive failures. The telemetry push below
        // still runs so the app sees liveness + OTA status during the flash.
        if (!ble_app.isOtaActive())
        {
            updateBattery();
            // V3: reconcile the flight-pack charger (presence detect, config
            // re-apply after plug-in / watchdog, 40 s WD kick, status/fault
            // transition logs). Cheap no-op while no charge input is present.
            if (config::HAS_PACK_CHARGER)
            {
                pack_charger.service();
            }
        }

        // Always push base station stats to BLE, even without LoRa packets,
        // so BS battery / logging state / RSSI stay live.  Tag each push
        // with a freshness status (#95) so the iOS app can distinguish:
        //   • LIVE    — re-pushing telemetry that's still recent
        //   • STALE   — re-pushing cached telemetry older than the threshold
        //               (iOS dims + shows "stale (Ns ago)")
        //   • SYNCING — no rocket has ever been caught
        //               (iOS hides rocket fields, shows "Searching for rocket…")
        // The RX path always sends LIVE because it fires on a fresh decode.
        if (ble_app.isConnected())
        {
            TR_BLE_To_APP::TelemetryData ble_telem = {};
            auto& tr = tracked_rockets[active_rocket_idx];
            if (tr.active)
            {
                buildBLETelemetry(tr.last_data, tr.last_rssi, tr.last_snr,
                                  tr.last_lat_deg, tr.last_lon_deg, tr.last_alt_m, ble_telem);
                if (tr.unit_name[0]) {
                    ble_telem.source_unit_name = tr.unit_name;
                }
                const uint32_t age_ms = millis() - tr.last_seen_ms;
                if (age_ms > config::BLE_TELEMETRY_STALE_MS)
                {
                    ble_telem.data_status = TR_BLE_To_APP::TelemetryData::DataStatus::STALE;
                    ble_telem.data_age_ms = age_ms;
                }
            }
            else
            {
                // No rocket tracked — publish base-station-only fields with
                // a SYNCING tag so the iOS app shows "Searching for rocket…"
                // rather than rendering the empty zero-init values as if
                // they were a rocket sitting in INIT.
                LoRaDataSI empty = {};
                buildBLETelemetry(empty, NAN, NAN, NAN, NAN, NAN, ble_telem);
                ble_telem.data_status = TR_BLE_To_APP::TelemetryData::DataStatus::SYNCING;
            }
            // #289: skip this push if the RX path already sent a fresher frame
            // this iteration — the periodic push exists to keep BS stats/liveness
            // current when no packet arrived, so it's redundant right after a
            // fresh decode.  The battery read above still runs every period.
            if (!rx_sent_telem_this_iter)
            {
                ble_app.sendTelemetry(ble_telem);
                maybeMarkOtaValid();
            }

            // Flash-space stats for the app's storage bar (every ~5 s).
            static uint32_t last_bs_storage_ms = 0;
            const uint32_t now_bs = millis();
            if (now_bs - last_bs_storage_ms >= 5000U)
            {
                last_bs_storage_ms = now_bs;
                BaseStationStorageStatsData bss = {};
                uint64_t total = 0, used = 0;
                const bool mounted = bsQueryStorage(total, used);
                bss.total_bytes = total;
                bss.used_bytes  = used;
                bss.free_bytes  = (total > used) ? (total - used) : 0;
                bss.backend = using_internal_flash ? 0 : (using_external_flash ? 2 : 1);
                bss.flags   = mounted ? 0x01 : 0x00;
                ble_app.sendStorageStats(0xCD, reinterpret_cast<const uint8_t*>(&bss), sizeof(bss));
            }
        }
    }

    // Handle BLE commands (file list, delete, download)
    ble_app.loop();

    // Detect BLE connect (rising edge) — no auto-config send since
    // the base station's cached config may not match the rocket's actual values
    {
        static bool ble_was_connected = false;
        bool ble_now = ble_app.isConnected();
        ble_was_connected = ble_now;
    }

    uint8_t ble_cmd = ble_app.getCommand();
    if (ble_cmd == BLE_BS_CMD_FILE_LIST)
    {
        handleFileListCommand();
    }
    else if (ble_cmd == BLE_BS_CMD_FILE_DELETE)
    {
        handleDeleteCommand();
    }
    else if (ble_cmd == BLE_BS_CMD_FILE_DOWNLOAD)
    {
        // #380: only STARTS the transfer (opens the file, captures state).
        // Chunks go out one per loop pass via serviceDownload() below, so the
        // RX/uplink/heartbeat/flush services keep running during the download.
        startDownload();
    }
    else if (ble_cmd == BLE_BS_CMD_CAMERA_TOGGLE)
    {
        // Camera toggle: send desired state (inverse of last known) so LoRa
        // retries are idempotent — won't toggle back and forth on the rocket.
        uint8_t desired = last_known_camera_recording ? 0 : 1;
        buildUplinkPacket(1, &desired, 1);
        ESP_LOGI(TAG, "[BLE->UPLINK] Camera %s", desired ? "START" : "STOP");
    }
    else if (ble_cmd == BLE_BS_CMD_LOGGING_TOGGLE)
    {
        // Logging toggle: starts/stops BOTH rocket flash recording (via LoRa
        // uplink) and base station SD card logging simultaneously.
        // Base station logging state is the toggle authority — rocket follows.
        if (!logging_active)
        {
            log_manual_inhibit = false;  // explicit start clears any prior inhibit (#107)
            startLogging();
            ESP_LOGI(TAG, "[LOG] Base station logging started (manual)");

            if (!last_known_rocket_logging)
            {
                uint8_t desired = 1;
                buildUplinkPacket(23, &desired, 1);
                ESP_LOGI(TAG, "[BLE->UPLINK] Rocket logging START");
            }
        }
        else
        {
            stopLogging();
            // Sticky inhibit: don't auto-restart on the next packet.  Cleared
            // on the next rocket state change so a real next flight is still
            // captured automatically.  (#107)
            log_manual_inhibit = true;
            ESP_LOGI(TAG, "[LOG] Base station logging stopped (manual; auto-restart inhibited until state change)");

            if (last_known_rocket_logging)
            {
                uint8_t desired = 0;
                buildUplinkPacket(23, &desired, 1);
                ESP_LOGI(TAG, "[BLE->UPLINK] Rocket logging STOP");
            }
        }
    }
    else if (ble_cmd == BLE_BS_CMD_SERVO_TEST_ANGLES)
    {
        // Servo test angles: relay 8-byte payload to OutComputer via LoRa uplink
        const uint8_t* payload = ble_app.getCommandPayload();
        const size_t plen = ble_app.getCommandPayloadLength();
        if (plen >= 8)
        {
            buildUplinkPacket(24, payload, 8);
            ESP_LOGI(TAG, "[BLE->UPLINK] Servo test angles");
        }
    }
    else if (ble_cmd == BLE_BS_CMD_SERVO_TEST_STOP)
    {
        // Servo test stop: relay to OutComputer via LoRa uplink
        buildUplinkPacket(25, nullptr, 0);
        ESP_LOGI(TAG, "[BLE->UPLINK] Servo test stop");
    }
    else if (ble_cmd == BLE_BS_CMD_SIM_CONFIG)
    {
        // Sim config: relay to OutComputer via LoRa uplink
        // Payload is 16 bytes: [mass_g:4][thrust_n:4][burn_s:4][descent_rate_mps:4]
        const uint8_t* payload = ble_app.getCommandPayload();
        size_t payload_len = ble_app.getCommandPayloadLength();
        if (payload_len > 16) payload_len = 16;
        buildUplinkPacket(5, payload, payload_len);

        if (payload_len >= 12)
        {
            float mass_g, thrust_n, burn_s;
            memcpy(&mass_g,   payload + 0, 4);
            memcpy(&thrust_n, payload + 4, 4);
            memcpy(&burn_s,   payload + 8, 4);
            float descent = 0.0f;
            if (payload_len >= 16)
            {
                memcpy(&descent, payload + 12, 4);
            }
            ESP_LOGI(TAG, "[BLE->UPLINK] Sim config: mass=%.0fg thrust=%.1fN burn=%.1fs descent=%.1fm/s",
                     (double)mass_g, (double)thrust_n, (double)burn_s, (double)descent);
        }
    }
    else if (ble_cmd == BLE_BS_CMD_SIM_START)
    {
        buildUplinkPacket(6, nullptr, 0);
        ESP_LOGI(TAG, "[BLE->UPLINK] Sim start");
    }
    else if (ble_cmd == BLE_BS_CMD_SIM_STOP)
    {
        buildUplinkPacket(7, nullptr, 0);
        ESP_LOGI(TAG, "[BLE->UPLINK] Sim stop");
    }
    else if (ble_cmd == BLE_BS_CMD_TIME_SYNC)
    {
        // Time sync from phone: [year_lo][year_hi][month][day][hour][minute][second]
        const uint8_t* payload = ble_app.getCommandPayload();
        size_t payload_len = ble_app.getCommandPayloadLength();
        if (payload_len >= 7)
        {
            sync_year   = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
            sync_month  = payload[2];
            sync_day    = payload[3];
            sync_hour   = payload[4];
            sync_minute = payload[5];
            sync_second = payload[6];
            time_sync_millis = millis();
            time_synced = true;

            ESP_LOGI(TAG, "[BLE] Time synced: %04u-%02u-%02u %02u:%02u:%02u UTC",
                     sync_year, sync_month, sync_day,
                     sync_hour, sync_minute, sync_second);

            // If a sequential-named log was opened before sync arrived,
            // promote it to its proper timestamped name now (#168).
            renameOpenLogIfSequential();
        }
    }
    else if (ble_cmd == BLE_BS_CMD_LORA_RECONFIG)
    {
        // LoRa reconfiguration — transactional (issue #71).  The base
        // station relays the new config to every rocket on the OLD channel,
        // then switches to NEW and verifies the rocket joined before
        // committing to NVS.  On timeout, both sides stay on OLD and the
        // silence-recovery layer heals any residual divergence.
        const uint8_t* payload = ble_app.getCommandPayload();
        size_t payload_len = ble_app.getCommandPayloadLength();
        if (payload_len >= 11)
        {
            float new_freq, new_bw;
            memcpy(&new_freq, payload + 0, 4);
            memcpy(&new_bw,   payload + 4, 4);
            uint8_t new_sf   = payload[8];
            uint8_t new_cr   = payload[9];
            int8_t  new_pwr  = (int8_t)payload[10];
            startLoRaTransaction(new_freq, new_bw, new_sf, new_cr, new_pwr);
        }
    }
    // #285: ble_cmd 12/13/14/65/66 (servo / PID / servo-enable / guidance /
    // fin-layout config relay to the rocket) were removed.  The base station
    // is a read-only display of the active rocket and never configures it: the
    // iOS app gates all config editing to a direct rocket connection
    // (SettingsView renders base stations read-only; ActiveRocketSyncer never
    // pushes a profile to a BS).  These relays were therefore unreachable dead
    // paths — and a foot-gun if a future app build ever sent config to a BS —
    // so they are intentionally absent.  Rocket config flows app->rocket over
    // a direct BLE link only.
    else if (ble_cmd == LORA_CMD_SET_HOP_DISABLED)
    {
        // BS-controlled hop enable/disable (#106).  Persist locally,
        // re-tune to lora_freq_mhz immediately if we were tracking a hop,
        // and uplink the same byte to the rocket so both sides agree.
        const uint8_t* payload = ble_app.getCommandPayload();
        size_t payload_len = ble_app.getCommandPayloadLength();
        if (payload_len >= 1)
        {
            const bool new_disabled = (payload[0] != 0);
            if (new_disabled != lora_hop_disabled)
            {
                lora_hop_disabled = new_disabled;
                prefs.begin("lora", false);
                prefs.putUChar("hopdis", lora_hop_disabled ? 1 : 0);
                prefs.end();
                if (lora_hop_disabled && hop_active_)
                {
                    // Drop hop tracking so the next radio service tick
                    // returns us to lora_freq_mhz.
                    hop_active_       = false;
                    hop_needs_retune_ = true;
                }
            }
            buildUplinkPacket(LORA_CMD_SET_HOP_DISABLED, payload, 1);
            ESP_LOGI(TAG, "[BLE->UPLINK] Hop disable: %s",
                     lora_hop_disabled ? "DISABLED (fixed freq)" : "ENABLED");
            sendCurrentConfig();
        }
    }
    else if (ble_cmd == BLE_BS_CMD_CONFIG_READBACK)
    {
        // Config readback request
        sendCurrentConfig();
    }
    // ---- Device Identity Commands ----
    else if (ble_cmd == BLE_BS_CMD_SET_UNIT_NAME)
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
            Preferences id_prefs;
            id_prefs.begin("identity", false);
            id_prefs.putBytes("un", new_name, strlen(new_name) + 1);
            id_prefs.end();
            ble_app.setName(unit_name);
            sendCurrentConfig();
            ESP_LOGI(TAG, "[BLE] Unit name set: %s", unit_name);
        }
    }
    else if (ble_cmd == BLE_BS_CMD_SET_NETWORK_ID)
    {
        // Set network_id — payload: [nid:1]
        const uint8_t* payload = ble_app.getCommandPayload();
        const size_t plen = ble_app.getCommandPayloadLength();
        if (plen >= 1)
        {
            network_id = payload[0];
            Preferences id_prefs;
            id_prefs.begin("identity", false);
            id_prefs.putUChar("nid", network_id);
            id_prefs.end();
            sendCurrentConfig();
            ESP_LOGI(TAG, "[BLE] Network ID set: %u", (unsigned)network_id);
        }
    }
    else if (ble_cmd == BLE_BS_CMD_RELAY_TO_ROCKET)
    {
        // Relay command to a specific rocket via LoRa uplink
        // Payload: [target_rid:1][inner_cmd:1][inner_payload:0..18]
        const uint8_t* payload = ble_app.getCommandPayload();
        const size_t plen = ble_app.getCommandPayloadLength();
        if (plen >= 2)
        {
            uint8_t target_rid = payload[0];
            uint8_t inner_cmd  = payload[1];
            const uint8_t* inner_payload = (plen > 2) ? &payload[2] : nullptr;
            size_t inner_len = (plen > 2) ? (plen - 2) : 0;
            buildUplinkPacket(inner_cmd, inner_payload, inner_len, target_rid);
            ESP_LOGI(TAG, "[BLE->UPLINK] Relay cmd=%u -> rid=%u (%u bytes)",
                     inner_cmd, target_rid, (unsigned)inner_len);
        }
    }
    else if (ble_cmd == BLE_BS_CMD_FREQ_SCAN)
    {
        // Frequency scan (base-station radio, pre-launch collision avoidance).
        // Payload: [start_mhz f32][stop_mhz f32][step_khz u16][dwell_ms u16]
        // All fields are little-endian.
        //
        // Two paths (#90):
        //   • Direct: rocket isn't presumed to be hopping (no recent RX, or
        //     last seen in READY/INIT/LANDED) — scan immediately.
        //   • Coordinated: rocket is presumed hopping → cmd 16 to park it
        //     on lora_freq_mhz, scan, push cmd 15, then re-bootstrap hop.
        //
        // The coordinated trigger uses rocketLikelyHopping() so it also
        // fires when the BS recently caught the rocket in a hop state but
        // the packet's next_channel_idx was NO_HOP (bootstrap, visiting
        // rendezvous, or paused).  In all those cases a direct scan would
        // still drop the link, even though hop_active_ is currently false.
        const uint8_t* payload = ble_app.getCommandPayload();
        const size_t plen = ble_app.getCommandPayloadLength();
        if (plen >= 12)
        {
            float start_mhz, stop_mhz;
            uint16_t step_khz, dwell_ms;
            memcpy(&start_mhz, payload + 0, 4);
            memcpy(&stop_mhz,  payload + 4, 4);
            memcpy(&step_khz,  payload + 8, 2);
            memcpy(&dwell_ms,  payload + 10, 2);

            const bool need_coord = rocketLikelyHopping(
                hop_active_, last_packet_ms, millis(),
                lastSeenRocketState(), COORD_HOP_RECENT_MS);

            if (!need_coord)
            {
                if (startNoiseScan(start_mhz, stop_mhz, step_khz, dwell_ms))
                {
                    ESP_LOGI(TAG, "[BLE] Scan started: %.1f..%.1f MHz, %u kHz, %u ms (×%u passes)",
                             (double)start_mhz, (double)stop_mhz,
                             (unsigned)step_khz, (unsigned)dwell_ms,
                             (unsigned)LORA_NOISE_SCAN_PASSES);
                }
                else
                {
                    ESP_LOGW(TAG, "[BLE] Scan start rejected (busy or invalid range)");
                }
            }
            else if (coord_scan_state_ != CoordScanState::IDLE)
            {
                ESP_LOGW(TAG, "[BLE] Scan rejected: coordinated scan already in progress");
            }
            else if (uplink_pending)
            {
                ESP_LOGW(TAG, "[BLE] Scan rejected: uplink busy — retry shortly");
            }
            else
            {
                startCoordinatedScan(start_mhz, stop_mhz, step_khz, dwell_ms);
            }
        }
    }

    // Service scan state machine (no-op when idle).  Must come before
    // serviceUplink so a TX retry doesn't fire while we're mid-scan —
    // the scan temporarily owns the radio's frequency.
    lora_comms.serviceScan();
    if (lora_comms.isScanDone())
    {
        if (scan_passes_remaining_ > 0)
        {
            // Multi-pass mode (#40 / #41 phase 3): merge this pass's
            // samples into the running max-RSSI accumulator, then
            // either kick off the next pass or finalize.
            const bool more = absorbScanPass();
            if (more)
            {
                (void)lora_comms.startScan(scan_param_start_mhz_,
                                           scan_param_stop_mhz_,
                                           scan_param_step_khz_,
                                           scan_param_dwell_ms_);
            }
            else
            {
                finalizeNoiseScan();
            }
        }
        else
        {
            // Defensive: a scan finished but we have no pass count
            // (shouldn't happen — startNoiseScan is the only path that
            // starts a scan).  Drain to keep the radio sane.
            (void)lora_comms.getScanSampleCount();
            lora_comms.consumeScanDone();
        }
    }

    // Service LoRa uplink retries (TX commands, then resume RX)
    serviceUplink();

    // Advance the transactional reconfigure state machine (issue #71).
    // Must run after serviceUplink so we observe uplink_pending transitions
    // to false in the same loop iteration they happen.
    serviceLoRaTransaction();

    // Silence recovery runs after the transaction service so a freshly
    // started transaction always wins over any pending recovery cycle.
    serviceRecovery();

    // Mask-drift auto-recovery (#105 follow-up).  Re-pushes cmd 15 if
    // the hop RX path detected a divergence between our skip-mask and
    // the rocket's.  Cheap when no drift is pending; only acts when
    // the radio is otherwise idle.
    serviceMaskDriftRepush();

    // Auto-acquire (#136): one-shot per power cycle.  Waits to hear the
    // rocket on rendezvous, runs a noise scan, picks the quietest
    // channel, and uses the existing cmd-10 transactional flow to move
    // both ends onto it.  Inert once DONE.
    serviceAutoAcquire();

    // Heartbeat — quietly tells the rocket "we're hearing you" so its
    // slow-rendezvous timer doesn't expire during normal idle operation.
    // Gates itself on recovery/transaction state internally.
    serviceHeartbeat();

    // BLE file download — at most one 170 B chunk per pass, paced to the BLE
    // notify drain interval. No-op unless a transfer is active (#380).
    serviceDownload();

    printStats();

    // Must use vTaskDelay(1) instead of yield() on ESP-IDF.
    // yield() only yields to equal-or-higher priority tasks, so IDLE
    // (priority 0) never runs and the task watchdog fires.
    vTaskDelay(1);
}

// ============================================================================
// ESP-IDF entry point
// ============================================================================

extern "C" void app_main(void)
{
    setup_bs();
    xTaskCreatePinnedToCore([](void*) { while (true) { loop_bs(); } },
                            "bs_loop", 8 * 1024, NULL, 5, NULL, 1);
}
