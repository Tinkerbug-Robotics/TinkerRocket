// ==========================================================================
// comms.cpp — the out_computer's job, in-process (rocket-computer-mini)
// ==========================================================================
// Port of projects/out_computer/main/main.cpp (referenced below as "OC
// L<n>").  Structure, names and safety comments are preserved so future OC
// fixes cross-apply; every deliberate deviation is marked MINI.
//
// Gone with the second MCU / hardware: I2S slave + parser + dedup, I2C slave,
// the FC command queue (setPendingCommand* → mini_link::sendCommand), the FC
// OTA relay (self-OTA stays), servo/PID/roll/guidance/camera/sounds command
// forwarding (respond unsupported), MRAM recovery, UartModemBackend.
// Telemetry arrives via mini_link::telem instead of I2S frames.

#include <compat.h>     // SPIClass for TR_LogToFlash::begin (the one allowed
                        // shim) + the millis()/delay() the ported OC code uses
#include <cstring>
#include <cmath>
#include <cstdio>
#include <string>
#include <algorithm>
#include <esp_pm.h>
#include <TR_NVS.h>
#include <esp_log.h>
#include <esp_mac.h>              // esp_efuse_mac_get_default for unit_id
#include <esp_app_desc.h>         // esp_app_get_description for "fw" readback (#8)
#include <esp_timer.h>
#include "driver/gpio.h"          // LED
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_gap.h"       // ble_gap_upd_params for the conn-param policy

// Nimble's os.h (pulled in by host/ble_gap.h) defines `max` and `min` as
// function-style macros, which collide with std::max / std::min. Undo them
// here so subsequent <algorithm> calls compile cleanly.  (OC L27-35)
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

// std::string is used as "String" in non-Arduino builds (same typedef as
// TR_BLE_To_APP.h).  (OC L38-52)
using String = std::string;

static inline std::string fmtf(float v, int decimals)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%.*f", decimals, (double)v);
    return std::string(buf);
}

static inline std::string itos(int v)
{
    return std::to_string(v);
}

#include "config.h"
#include "comms.h"
#include "flight.h"
#include "mini_link.h"

#include <TR_I2C_Interface.h>   // packMessage only — no bus
#include <TR_LogToFlash.h>
#include <TR_LoRa_Comms.h>
#include <IRadioLink.h>
#include <LoRaDirectBackend.h>
#include <TR_Sensor_Data_Converter.h>
#include <TR_Orientation.h>
#include <TR_Coordinates.h>
#include <TR_BLE_To_APP.h>
#include <RocketComputerTypes.h>
#include <TR_INA230.h>
#include <TR_FlightLog.h>
#include <TR_NandBackend_esp.h>
#include <NandBitmapStore.h>
#include <WireFormat.h>

// ==========================================================================
// SECTION: globals owned by main.cpp (definitions there; see main.cpp)
// ==========================================================================
extern TR_LogToFlash logger;
extern tr_flightlog::TR_NandBackend_esp flightlog_backend;
extern tr_flightlog::TR_FlightLog flightlog;
extern tr_flightlog::NandBitmapStore flightlog_bitmap_store;
extern TR_BLE_To_APP ble_app;
extern LoRaDirectBackend lora_direct_backend;
extern IRadioLink& lora_comms;
extern TR_INA230 ina230;
extern bool ina230_ok;
extern i2c_master_bus_handle_t shared_i2c_bus;
// Rail commanded state.  Set by main.cpp's power machine at transition START
// (mirrors OC pwr_pin_on, L581/L7051); rail-backed servicing below gates on
// peripherals_initialized instead, which flips at transition END.
extern volatile bool pwr_pin_on;
void maybeMarkOtaValid();     // main.cpp (#8 OTA rollback gate)
void miniRequestPowerOn();    // main.cpp power machine (OC BLE cmd 8 ON half)
void miniRequestPowerOff();   // main.cpp power machine (OC BLE cmd 8 OFF half)

// ==========================================================================
// SECTION: comms-side state
// ==========================================================================
// MINI: the OC's SPI global is SPI2_HOST (compat.h L325); on the mini SPI2 is
// the sensor bus and the NAND/radio share SPI3 (board_v1.h).  Own instance on
// SPI3_HOST for TR_LogToFlash.  main.cpp runs spi_bus_initialize(SPI3_HOST)
// first with max_transfer_sz=4096; SPIClass::begin's own attempt then returns
// ESP_ERR_INVALID_STATE, which it treats as "not mine to free" and continues
// adding devices — exactly the sharing behaviour we need (compat.h L198-199).
static SPIClass mem_spi(SPI3_HOST);

// Phone time sync (BLE Command 9) — used for flight-filename timestamps.
// (OC L120-131)
static uint16_t phone_utc_year   = 0;
static uint8_t  phone_utc_month  = 0;
static uint8_t  phone_utc_day    = 0;
static uint8_t  phone_utc_hour   = 0;
static uint8_t  phone_utc_minute = 0;
static uint8_t  phone_utc_second = 0;
static uint32_t phone_sync_millis = 0;
static bool     phone_time_valid = false;

static bool flightlogWriteSink(void* ctx, const uint8_t* payload, size_t payload_len)
{
    auto* fl = static_cast<tr_flightlog::TR_FlightLog*>(ctx);
    return fl->writeFrame(payload, payload_len) == tr_flightlog::Status::Ok;
}

// #281/#278: classify flight-log storage for the #303 scorecard.  (OC L143-168)
static SensorHealthState ocStorageHealth()
{
    // #566: an uninitialized flight log is the MOST severe storage state, not
    // an inapplicable one — NA hides the row in the app's go/no-go.  See the
    // OC comment for the full history.
    if (!flightlog.isInitialized()) return SH_BAD;
    TR_LogToFlashStats s = {};
    logger.getStats(s);
    const uint32_t free_blocks = flightlog.bitmap().countInState(tr_flightlog::BLOCK_FREE);
    const uint32_t prealloc    = flightlog.config().prealloc_blocks;
    SensorHealthState st = shStorageState(free_blocks, prealloc, s.nand_prog_fail);
    // The flight index is a second, independent capacity limit (#281).
    const size_t used = flightlog.index().size();
    const size_t cap  = tr_flightlog::FlightIndex::MAX_ENTRIES;
    if (used >= cap) st = SH_BAD;
    else if (used + 4 >= cap && st == SH_OK) st = SH_DEGRADED;
    return st;
}

// BLE file-ops backed on TR_FlightLog (issue #50).  (OC L176-198)
static bool flightlogReadChunk(const char* filename, uint32_t offset,
                               uint8_t* buf, size_t max_bytes,
                               size_t& out_bytes, bool& eof)
{
    out_bytes = 0;
    eof = false;
    while (out_bytes < max_bytes)
    {
        size_t got = 0;
        auto st = flightlog.readFlightPage(
            filename, offset + out_bytes,
            buf + out_bytes, max_bytes - out_bytes, got);
        if (st == tr_flightlog::Status::NotFound) return false;
        if (st != tr_flightlog::Status::Ok) return false;
        if (got == 0)
        {
            eof = true;
            break;
        }
        out_bytes += got;
    }
    return true;
}

// Builds the cmd 2 / cmd 3 response JSON.  (OC L205-233)
static std::string flightlogBuildFileListJson(uint8_t page)
{
    static constexpr size_t FILES_PER_PAGE = 5;

    static tr_flightlog::FlightIndexEntry entries[
        tr_flightlog::FlightIndex::MAX_ENTRIES];
    const size_t total = flightlog.listFlights(
        entries, tr_flightlog::FlightIndex::MAX_ENTRIES,
        /*page=*/0, /*per_page=*/tr_flightlog::FlightIndex::MAX_ENTRIES);

    // Reverse for newest-first display order.
    for (size_t i = 0; i < total / 2; ++i)
    {
        tr_flightlog::FlightIndexEntry tmp = entries[i];
        entries[i] = entries[total - 1 - i];
        entries[total - 1 - i] = tmp;
    }

    const size_t start = (size_t)page * FILES_PER_PAGE;
    const size_t end_raw = start + FILES_PER_PAGE;
    const size_t s = (start < total) ? start : total;
    const size_t e = (end_raw < total) ? end_raw : total;
    const size_t n = e - s;

    static char json[512];
    const size_t json_len = tr_flightlog::wire_format::encodeFileListJson(
        entries + s, n, json, sizeof(json));
    return std::string(json, json_len);
}

static void flightlogBeginFlight()
{
    if (!flightlog.isInitialized()) return;
    if (flightlog.isFlightActive())
    {
        ESP_LOGW("FLIGHTLOG", "prepareFlight: already active, skipping");
        return;
    }
    // Defer the 256-block erase loop (~770 ms) to the flush task on Core 0
    // (issue #77).  Inline it would block whatever task is on the call stack
    // — here the comms loop or the BLE cmd 23 handler — and on the mini
    // Core 0 also carries the sensor-poll tasks.  (OC L235-250)
    flightlog.requestPrepareFlight();
}

// Pending-finalize state.  Set by flightlogEndFlight and serviced by
// flightlogFlushTaskHook on the flush task (Core 0) — keeps NAND-heavy work
// (FlightIndex::save reads pages with a ~2 KB on-stack buffer) off the
// requester's stack, and final_bytes is read at service time because the
// flush task may still be draining the ring when the request fires.
// (OC L252-309)
static portMUX_TYPE g_finalize_mux        = portMUX_INITIALIZER_UNLOCKED;
static bool         g_finalize_pending    = false;
static char         g_finalize_name[28]   = {};

static void flightlogServicePendingFinalize()
{
    char name_local[sizeof(g_finalize_name)];
    bool do_it = false;
    portENTER_CRITICAL(&g_finalize_mux);
    // Only act once the drain is fully complete — logger.isLoggingActive()
    // covers both logging_active and the not-yet-cleared end_flight_requested.
    if (g_finalize_pending && !logger.isLoggingActive())
    {
        std::memcpy(name_local, g_finalize_name, sizeof(name_local));
        g_finalize_pending = false;
        do_it = true;
    }
    portEXIT_CRITICAL(&g_finalize_mux);
    if (!do_it) return;

    if (!flightlog.isFlightActive())
    {
        ESP_LOGW("FLIGHTLOG", "finalizeFlight (deferred): no active flight, skipped");
        return;
    }

    // lastClosedSessionBytes() is the sticky snapshot closeLogSession took
    // just before zeroing current_file_bytes — the exact sink byte count.
    const uint32_t bytes = logger.lastClosedSessionBytes();
    auto st = flightlog.finalizeFlight(name_local, bytes);
    if (st == tr_flightlog::Status::Ok)
    {
        ESP_LOGI("FLIGHTLOG",
                 "finalizeFlight OK (deferred): %s (%u bytes, %u extensions, "
                 "%u salvaged bad blocks, %u unrecoverable pages)",
                 name_local, (unsigned)bytes,
                 (unsigned)flightlog.overflowExtensionCount(),
                 (unsigned)flightlog.salvagedBlockCount(),
                 (unsigned)flightlog.unrecoverablePageCount());
    }
    else
    {
        ESP_LOGW("FLIGHTLOG", "finalizeFlight (deferred): %s",
                 tr_flightlog::to_string(st));
    }
}

// Drives the deferred Core-0 work for TR_FlightLog (wired into
// TR_LogToFlash::flushTaskLoop via cfg.flush_task_hook).  (OC L315-352)
static void flightlogFlushTaskHook(void* /*ctx*/)
{
    uint32_t id = 0;
    tr_flightlog::Status st = tr_flightlog::Status::Ok;
    const uint32_t evicted_before = flightlog.autoEvictedCount();
    if (flightlog.servicePendingPrepareFlight(id, st))
    {
        if (st == tr_flightlog::Status::Ok)
        {
            // #315: surface auto-eviction, never silent.
            const uint32_t evicted_now = flightlog.autoEvictedCount() - evicted_before;
            if (evicted_now > 0)
            {
                ESP_LOGW("FLIGHTLOG",
                         "#315 auto-evict: reclaimed %u oldest flight(s) (last id=%u) "
                         "to arm; %u free blocks remain",
                         (unsigned)evicted_now,
                         (unsigned)flightlog.lastEvictedFlightId(),
                         (unsigned)flightlog.bitmap().countInState(tr_flightlog::BLOCK_FREE));
            }
            ESP_LOGI("FLIGHTLOG",
                     "prepareFlight OK (deferred): id=%u, range=[%u..%u), pages=%u",
                     (unsigned)id,
                     (unsigned)flightlog.activeStartBlock(),
                     (unsigned)(flightlog.activeStartBlock() + flightlog.activeBlockCount()),
                     (unsigned)flightlog.activeBlockCount());
        }
        else
        {
            ESP_LOGW("FLIGHTLOG", "prepareFlight (deferred): %s",
                     tr_flightlog::to_string(st));
        }
    }

    flightlogServicePendingFinalize();
}

static void flightlogEndFlight()
{
    if (!flightlog.isInitialized()) return;
    if (!flightlog.isFlightActive())
    {
        // Can happen if prepareFlight failed, or if called twice. No-op.
        return;
    }

    // flight_YYYYMMDD_HHMMSS.bin from phone time when synced, else
    // flight_N.bin.  (OC L354-409)
    char name[28];  // matches FlightIndexEntry::filename[28]
    if (phone_time_valid)
    {
        uint32_t elapsed_s = (millis() - phone_sync_millis) / 1000;
        uint32_t total_s = (uint32_t)phone_utc_hour * 3600U +
                           (uint32_t)phone_utc_minute * 60U +
                           (uint32_t)phone_utc_second + elapsed_s;
        uint16_t y = phone_utc_year;
        uint8_t  mo = phone_utc_month;
        uint8_t  d = phone_utc_day;
        if (total_s >= 86400U)
        {
            d += (uint8_t)(total_s / 86400U);  // day rollover within a month — good enough
            total_s %= 86400U;
        }
        uint8_t h  = (uint8_t)(total_s / 3600U);
        uint8_t mi = (uint8_t)((total_s % 3600U) / 60U);
        uint8_t s  = (uint8_t)(total_s % 60U);
        std::snprintf(name, sizeof(name),
                      "flight_%04u%02u%02u_%02u%02u%02u.bin",
                      (unsigned)y, (unsigned)mo, (unsigned)d,
                      (unsigned)h, (unsigned)mi, (unsigned)s);
    }
    else
    {
        std::snprintf(name, sizeof(name), "flight_%lu.bin",
                      (unsigned long)flightlog.activeFlightId());
    }
    portENTER_CRITICAL(&g_finalize_mux);
    std::memcpy(g_finalize_name, name, sizeof(g_finalize_name));
    g_finalize_pending = true;
    portEXIT_CRITICAL(&g_finalize_mux);
    // Do NOT delete — index entries accumulate; deletion is BLE cmd 3.
}

// ==========================================================================
// SECTION: shared device state (OC L411-435, L572-601 trimmed)
// ==========================================================================
static SensorConverter sensor_converter;
static TR_Coordinates coord;

// INA230 shunt resistor and current LSB (OC L433-435; same 2 mOhm shunt).
static constexpr float INA230_R_SHUNT_OHM = 0.002f;     // 2 mOhm
static constexpr float INA230_CURRENT_LSB_A = 0.001f;    // 1 mA/bit
static bool ina_continuous = false;   // INA230 in continuous-averaging mode (OC L432)

static volatile bool flash_op_active = false;  // set during blocking NAND ops (OC L557)
                                               // MINI: no I2S/I2C consumers left; kept
                                               // as the download-in-progress marker.

// NVS persistence for LoRa settings (config.h values are factory defaults)
static Preferences prefs;
static float   lora_freq_mhz  = config::LORA_FREQ_MHZ;
static uint8_t lora_sf         = config::LORA_SF;
static float   lora_bw_khz    = config::LORA_BW_KHZ;
static uint8_t lora_cr         = config::LORA_CR;
static int8_t  lora_tx_power   = config::LORA_TX_POWER_DBM;

static RocketState latest_rocket_state = INITIALIZATION;
// Rail-backed comms peripherals (NAND logger + radio) are live.  Equivalent
// of the OC's peripherals_initialized (L582); set by comms_setup_active().
static bool peripherals_initialized = false;
// TR_LogToFlash::begin succeeded — gates EVERY logger touchpoint (a failed
// begin leaves null mutexes/ring; even service() can reach an unmounted
// LFS through a launch edge). flightlog guards itself via isInitialized().
static bool logger_ok = false;

bool commsLoggerOk() { return logger_ok; }

// ==========================================================================
// SECTION: LoRa frequency lock and channel hopping (OC L584-788, verbatim)
// ==========================================================================
// Frequency is locked once the rocket enters flight (issue #71).  The
// transition logic is pure and shared (computeFreqLockForFlight).
static bool freq_locked_for_flight = false;

static inline void updateFreqLockFromState(RocketState s)
{
    freq_locked_for_flight = computeFreqLockForFlight(freq_locked_for_flight, s);
}

// Per-packet channel-hop state (issues #40 / #41, phase 2a).  See the OC
// header comment (L603-632) for the full design narrative; the hop sequence
// is linear mod the channel-set count, seq-anchored since #105.
static bool    hop_active_        = false;
static uint8_t hop_idx_           = 0;
static uint8_t hop_bootstrap_left_ = 0;   // #150: full dwell-count bootstrap
static bool    hop_needs_retune_  = false;
// Link mode (#106/#150): NVS-backed, default fixed.  Initialized true so the
// window between boot and the NVS load can never report hop-enabled.
static bool    lora_hop_disabled  = true;

// LoRa transmit mute — the user-facing "LoRa off" mode (mirrors the OC's
// lora_tx_disabled).  When true nothing is transmitted: no telemetry, no name
// beacon, no hop schedule.  The RECEIVER stays up (serviceLoRaUplink and the
// RX-only rendezvous cycle keep running), so a muted rocket is still reachable
// from the base station and can be un-muted over the air.  NVS-backed ("lora"
// namespace, key "txdis"); initialized false so the window between boot and
// the NVS load can never suppress telemetry.
static bool    lora_tx_disabled   = false;

// #150 bench finding (2026-07-15): defer hop activation past the BS's
// cmd-17 mirror-retry train so the bootstrap lands on a listening BS.
static constexpr uint32_t HOP_ENABLE_DEFER_MS = 1500;
static uint32_t hop_enable_apply_at_ms = 0;

// Hop-silence rendezvous fallback state (#40 / #41 phase 2b).
enum class HopFallbackState : uint8_t {
    NORMAL,
    VISITING_RENDEZVOUS,
    PAUSED_FOR_SCAN,        // BS-coordinated cmd 16 pause (#90)
};

static HopFallbackState hop_fallback_state          = HopFallbackState::NORMAL;
static uint32_t         hop_fallback_phase_start_ms = 0;
static uint32_t         hop_active_entered_ms       = 0;
static uint32_t         hop_session_uplink_count    = 0;  // resets each hop session
static uint32_t         hop_pause_until_ms          = 0;  // deadline for PAUSED_FOR_SCAN

// Channel-set skip-mask pushed by the BS via LORA_CMD_CHANNEL_SET (#40/#41
// phase 3); rendezvous freq is compile-time hardcoded on both sides (#105).
static uint8_t skip_mask_[LORA_SKIP_MASK_MAX_BYTES] = {0};
static uint8_t skip_mask_n_        = 0;        // 0 = no mask (all active)
static float   channel_set_bw_khz_ = 0.0f;     // BW the mask was built for

// Tracks whether the radio is currently in RX mode.
static bool lora_in_rx_mode = false;

// #150: packets-per-channel dwell for the CURRENT modulation; 0 means hopping
// is not permitted at this (sf, bw, cr) and every hop entry point refuses.
static inline uint8_t currentHopDwell()
{
    return loraHopDwellForLink(lora_sf, lora_bw_khz, SIZE_OF_LORA_BUDGET, lora_cr);
}

// #150: effective skip mask = cmd-15 noise mask (when valid for the current
// BW) + the implicit home-channel skip.  `buf` must hold
// LORA_SKIP_MASK_MAX_BYTES.
static const uint8_t* effectiveHopMask(uint8_t* buf, uint8_t n_channels)
{
    const bool mask_valid = (skip_mask_n_ == n_channels &&
                              channel_set_bw_khz_ == lora_bw_khz);
    if (mask_valid) memcpy(buf, skip_mask_, LORA_SKIP_MASK_MAX_BYTES);
    else            memset(buf, 0, LORA_SKIP_MASK_MAX_BYTES);
    (void)loraApplyHomeChannelSkip(buf, lora_bw_khz, lora_freq_mhz);
    return buf;
}

static inline void updateHopFromState(RocketState s)
{
    // Fixed-frequency mode (#106/#150 link mode) stays on lora_freq_mhz for
    // every state; #150 also refuses when the modulation can't fit a dwell
    // visit inside the FCC occupancy budget.
    // A muted radio must not hop: the BS follows the schedule by decoding our
    // packets, so hopping without transmitting only walks the receiver away
    // from the channel the BS is calling on.  Folding it in here also gets the
    // ON→OFF branch (retune back to lora_freq_mhz) for free.
    const bool want_active = !lora_tx_disabled && !lora_hop_disabled &&
                             currentHopDwell() > 0 &&
                             shouldHopInState(s);
    if (want_active && !hop_active_)
    {
        // OFF → ON.  Bootstrap: a dwell-count of packets still go out on
        // lora_freq_mhz, each announcing the schedule-entry channel (#150).
        hop_active_              = true;
        hop_bootstrap_left_      = currentHopDwell();
        if (hop_bootstrap_left_ == 0) hop_bootstrap_left_ = 1;  // want_active guarantees dwell>0
        hop_idx_                 = 0;
        hop_needs_retune_        = true;  // ensure we're on lora_freq_mhz before TXing
        hop_active_entered_ms    = millis();
        hop_session_uplink_count = 0;
        hop_fallback_state       = HopFallbackState::NORMAL;
        ESP_LOGI("OC", "[HOP] Active: %u bootstrap pkt(s) on %.2f MHz, then schedule "
                       "(%u channels at BW=%.0f kHz)",
                 (unsigned)hop_bootstrap_left_,
                 (double)lora_freq_mhz, (unsigned)loraChannelCount(lora_bw_khz),
                 (double)lora_bw_khz);
    }
    else if (!want_active && hop_active_)
    {
        // ON → OFF: leave the table and return to the static channel.  If
        // mid-rendezvous-visit, come out of that first so the radio ends up
        // with the saved modulation, not the rendezvous one.
        if (hop_fallback_state == HopFallbackState::VISITING_RENDEZVOUS)
        {
            (void)lora_comms.reconfigure(lora_freq_mhz, lora_sf, lora_bw_khz,
                                          lora_cr, lora_tx_power);
            (void)lora_comms.startReceive();
            lora_in_rx_mode = true;
            hop_fallback_state = HopFallbackState::NORMAL;
        }
        else if (hop_fallback_state == HopFallbackState::PAUSED_FOR_SCAN)
        {
            // Already on lora_freq_mhz with the operating preset (cmd 16
            // reconfigured us there).  Just clear the pause state.
            hop_fallback_state = HopFallbackState::NORMAL;
            hop_pause_until_ms = 0;
        }
        hop_active_         = false;
        hop_bootstrap_left_ = 0;
        hop_needs_retune_   = true;
        ESP_LOGI("OC", "[HOP] Inactive: returning to %.2f MHz", (double)lora_freq_mhz);
    }
}

// Frequency the radio should currently be tuned to, given the hop state.
static inline float hopTargetFreqMHz()
{
    if (hop_active_ && hop_bootstrap_left_ == 0 &&
        hop_fallback_state == HopFallbackState::NORMAL)
    {
        const float f = loraChannelMHz(lora_bw_khz, hop_idx_);
        if (f > 0.0f) return f;
        // Channel table empty (BW invalid) — fall through to static.
    }
    return lora_freq_mhz;
}

// ==========================================================================
// SECTION: Cached configuration (NVS <-> flight side <-> app)
// ==========================================================================
// MINI: servo/PID/roll/guidance/camera caches are gone with the hardware.
// What remains mirrors the OC exactly (same NVS namespaces/keys so a fleet
// migration or backup tool sees the same layout).

// IMU mounting orientation setting (#phase3): IMU_ORIENT_AUTO or 0..23.
// NVS-persisted so a manual roll clocking survives power cycles.  (OC L821)
static uint8_t cfg_imu_orient = IMU_ORIENT_AUTO;

// IMU logging rate setting (BLE cmd 67).  Cached for app readback and
// re-push; the flight side persists it in its own NVS and re-applies at
// boot.  IMU_RATE_DYNAMIC is a MODE, not a rate.  (OC L834-839)
static uint16_t cfg_imu_rate = IMU_RATE_DYNAMIC;

// Pyro config cache (4 channels).  (OC L847-850)
static bool    cfg_pyro_enabled[4]      = { false, false, false, false };
static uint8_t cfg_pyro_trigger_mode[4] = { 0, 0, 0, 0 };
static float   cfg_pyro_trigger_value[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

// Device identity (loaded from NVS "identity" namespace).  (OC L852-856)
static char    unit_id_hex[9] = {0};           // last 4 bytes of MAC as "a1b2c3d4"
static char    unit_name[24]  = "TinkerRocket"; // default until NVS loads
static uint8_t network_id     = config::DEFAULT_NETWORK_ID;
static uint8_t rocket_id      = config::DEFAULT_ROCKET_ID;

// Latest-state caches (OC L858-884).  MINI: refreshed from mini_link::telem
// by serviceTelemFromFlight() instead of the I2S parser; the names stay so
// everything downstream (buildLoRaPayload, printStats) ports unchanged.
static ISM6HG256Data latest_ism6_raw = {};
static BMP585Data latest_bmp_raw = {};
static GNSSData latest_gnss_raw = {};
static POWERData latest_power_raw = {};
static NonSensorData latest_non_sensor = {};

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
// LORA_MSG (0xF1) records accepted by the flight-log ring — diverges from
// lora_tx_ok legitimately when no session is open (see OC L886-895).
static uint32_t lora_tx_logged = 0;
// LORA_UPLINK_MSG (0xF9) records accepted by the ring (OC L896-901).
static uint32_t lora_uplink_logged = 0;
// #150: uplinks dropped by the network-id filter (OC L902-905).
static uint32_t lora_uplink_nid_drops = 0;
static uint32_t last_lora_tx_ms = 0;

// Free-running per-TX sequence counter (#105, u16 since proto v4).
static uint16_t lora_tx_seq = 0;
static uint32_t lora_uplink_rx_count = 0;
// CRC-passing decodes below loraMinValidSnrDb — counted, dropped (#90).
static uint32_t lora_low_snr_drops = 0;

// Slow-rendezvous trackers (issue #71).
static uint32_t last_uplink_rx_ms = 0;
static uint32_t ready_entry_ms    = 0;

// #383: uplink camera/logging refusals while INFLIGHT (undeliverable).
static uint32_t uplink_inflight_refusals = 0;

static inline bool nsFlagSet(uint8_t flags, uint8_t mask)
{
    return (flags & mask) != 0U;
}

// ==========================================================================
// SECTION: Battery sampling and state of charge (OC L982-1080, verbatim)
// ==========================================================================
// Plausible bus-voltage window — floor sits below USB (~5.2 V) so bench power
// is still reported (reads BAD on the 2S scorecard); a failed read passes 0 V
// which is rejected.  (#272/#303, OC L990-1002)
static constexpr float   POWER_BUS_V_MIN      = 3.0f;   // below USB; still rejects 0 V failed reads
static constexpr float   POWER_BUS_V_MAX      = 9.0f;
static constexpr uint8_t POWER_BAD_READ_LIMIT = 3;

static bool commitPowerSample(float bus_v, float current_a)
{
    static uint8_t consec_bad = 0;

    if (!(bus_v == bus_v) || bus_v < POWER_BUS_V_MIN || bus_v > POWER_BUS_V_MAX)
    {
        if (consec_bad < 255) consec_bad++;
        if (consec_bad >= POWER_BAD_READ_LIMIT) latest_power_valid = false;
        return false;
    }
    consec_bad = 0;

    // 2S LiPo voltage-to-SOC lookup (linear interp): 8.40V=100% ... 6.60V=0%.
    static constexpr float SOC_V[] = { 6.60f, 7.00f, 7.40f, 7.60f, 7.80f, 8.40f };
    static constexpr float SOC_P[] = { 0.0f,  10.0f, 25.0f, 50.0f, 75.0f, 100.0f };
    float soc_pct = 0.0f;
    if (bus_v <= SOC_V[0])      soc_pct = SOC_P[0];
    else if (bus_v >= SOC_V[5]) soc_pct = SOC_P[5];
    else for (int i = 0; i < 5; i++)
        if (bus_v <= SOC_V[i + 1])
        {
            soc_pct = SOC_P[i] + (bus_v - SOC_V[i]) / (SOC_V[i + 1] - SOC_V[i]) * (SOC_P[i + 1] - SOC_P[i]);
            break;
        }

    POWERDataSI psi = {};
    // Flight timebase if available (aligns power with the sensor logs); on
    // the mini both sides share esp_timer so this is the same clock either
    // way — kept for OC structure parity.
    psi.time_us = (latest_non_sensor_valid && latest_non_sensor.time_us != 0)
                  ? latest_non_sensor.time_us
                  : (uint32_t)micros();
    psi.voltage = bus_v;
    // Negative = discharging; the INA shunt reads load current positive, so invert.
    psi.current = -current_a * 1000.0f;
    psi.soc     = soc_pct;
    sensor_converter.packPowerData(psi, latest_power_raw);
    latest_power_valid = true;
    return true;
}

static void readINA230Power()
{
    if (!ina230_ok) return;

    // Trigger a single shunt+bus conversion (INA auto-powers-down after)
    ina230.setMode(INA230_Mode::SHUNT_BUS_TRIG);

    // Poll CVRF (Conversion Ready Flag) instead of a fixed delay.
    bool cvrf = false;
    for (int i = 0; i < 20; i++)  // max ~2ms total
    {
        delayMicroseconds(100);
        uint16_t me = 0;
        if (ina230.readMaskEnable(&me) == TR_INA230_OK && (me & (1 << 3)))
        {
            cvrf = true;
            break;  // CVRF set — conversion complete
        }
    }

    // A CVRF timeout means the conversion never completed — treat that (and
    // any I2C error) as a failed sample (#272) by passing 0 V.  NOTE:
    // readMaskEnable clears CVRF, so it must be captured in the poll above.
    float bus_v = 0.0f, current_a = 0.0f;
    const bool read_ok = cvrf
        && ina230.readBusVoltage_V(&bus_v) == TR_INA230_OK
        && ina230.readCurrent_A(&current_a) == TR_INA230_OK;
    commitPowerSample(read_ok ? bus_v : 0.0f, read_ok ? current_a : 0.0f);
}

// ==========================================================================
// SECTION: BLE connection-parameter policy (OC L1357-1468, verbatim)
// ==========================================================================
// #519: no conn_handle argument — TR_BLE_To_APP reads the live handle.
// #524: go through TR_BLE_To_APP so a collision retry re-asks the SAME set.
static void applyBLEParams(const char* what, const struct ble_gap_upd_params& params)
{
    if (ble_app.connHandle() == 0xFFFF)
    {
        ESP_LOGW("BLE", "%s conn params skipped — not connected", what);
        return;
    }
    ble_app.requestConnParams(params.itvl_min, params.itvl_max,
                              params.latency, params.supervision_timeout, what);
}

// #542: how long after the connect-time config push to keep the FAST link
// before dropping to the low-power idle set (covers the app's connect burst).
static constexpr uint32_t kSlowParamsDeferMs = 8000;

// Slow parameters for low-power idle (inside Apple's envelope).
static void requestSlowBLEParams()
{
    struct ble_gap_upd_params params = {};
    params.itvl_min = 0x50;              // 100ms  (units of 1.25ms)
    params.itvl_max = 0xA0;              // 200ms
    params.latency = 4;                  // Skip up to 4 connection events (~1 s effective)
    params.supervision_timeout = 400;    // 4 seconds (units of 10ms)
    params.min_ce_len = 0;
    params.max_ce_len = 0;
    applyBLEParams("Slow (low-power)", params);
}

// Fast parameters for file transfer (#503: within iOS's 15 ms floor).
static void requestFastBLEParams()
{
    struct ble_gap_upd_params params = {};
    params.itvl_min = 0x0C;              // 15ms — iOS minimum; below this is refused
    params.itvl_max = 0x18;              // 30ms — must be >= min + 15 ms
    params.latency = 0;                  // No slave latency
    params.supervision_timeout = 200;    // 2 seconds (units of 10ms)
    params.min_ce_len = 0;
    params.max_ce_len = 0;
    applyBLEParams("Fast (transfer)", params);
}

// #524: never start a transfer on the idle link (see the OC's bench history).
static constexpr uint32_t FAST_LINK_EVENT_MS_MAX = 60;    // 30 ms interval, latency 0, + slack
static constexpr uint32_t FAST_LINK_WAIT_MS      = 3000;

static void ensureFastLinkForTransfer()
{
    if (ble_app.effectiveEventMs() <= FAST_LINK_EVENT_MS_MAX) return;   // already fast

    ESP_LOGW("BLE", "Link too slow for a transfer (%lu ms effective) — asking for fast params",
             (unsigned long)ble_app.effectiveEventMs());
    requestFastBLEParams();

    // Waiting here is free: this is a ground operation.
    const uint32_t deadline = millis() + FAST_LINK_WAIT_MS;
    while ((int32_t)(deadline - millis()) > 0)
    {
        delay(50);
        if (ble_app.effectiveEventMs() <= FAST_LINK_EVENT_MS_MAX)
        {
            ESP_LOGI("BLE", "Link now %lu ms effective — starting transfer",
                     (unsigned long)ble_app.effectiveEventMs());
            return;
        }
    }
    ESP_LOGW("BLE", "Fast params never took (still %lu ms effective) — transfer will be SLOW "
                    "but will complete",
             (unsigned long)ble_app.effectiveEventMs());
}

// ==========================================================================
// SECTION: Derived telemetry (altitude and speed) (OC L1470-1540, verbatim)
// ==========================================================================
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

    // Track baseline pressure while not in-flight so pressure altitude is
    // near zero before launch.
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

    // Altitude rate comes from the flight side's KF via NonSensorData.
    // Reject physically impossible altitudes (was: corrupt I2S frames; on
    // the mini a torn/garbage baro read is the only source, keep the guard).
    if (pressure_alt_m > -500.0f && pressure_alt_m < 100000.0f)
        max_alt_m = std::max(max_alt_m, pressure_alt_m);
}

static void updateDerivedSpeedFromNonSensor()
{
    if (!latest_non_sensor_valid)
    {
        return;
    }

    // Don't track speed until EKF is running (state > INITIALIZATION).
    if (latest_non_sensor.rocket_state <= INITIALIZATION)
    {
        return;
    }

    const float e = (float)latest_non_sensor.e_vel / 100.0f;
    const float n = (float)latest_non_sensor.n_vel / 100.0f;
    const float u = (float)latest_non_sensor.u_vel / 100.0f;
    const float speed = sqrtf(e * e + n * n + u * u);

    // Reject physically impossible speeds.
    if (speed > 1500.0f)
        return;

    const bool alt_apogee = nsFlagSet(latest_non_sensor.flags, NSF_ALT_APOGEE);
    const bool vel_apogee = nsFlagSet(latest_non_sensor.flags, NSF_VEL_APOGEE);
    if (!alt_apogee && !vel_apogee)
    {
        max_speed_mps = std::max(max_speed_mps, speed);
    }
}

// ==========================================================================
// SECTION: Phone-IO PM locks and transfer summary latch (OC L1743-1828)
// ==========================================================================
#if defined(CONFIG_PM_ENABLE)
static esp_pm_lock_handle_t s_phone_io_cpu_lock = nullptr;  // ESP_PM_CPU_FREQ_MAX
static esp_pm_lock_handle_t s_phone_io_ls_lock  = nullptr;  // ESP_PM_NO_LIGHT_SLEEP

// Did the locks actually engage?  Report rather than assume (OC L1747-1750).
static bool s_phone_io_pm_held = false;

static void phoneIoPmAcquire()
{
    if (s_phone_io_cpu_lock == nullptr && s_phone_io_ls_lock == nullptr)
    {
        const esp_err_t e1 = esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX,   0,
                                                "phone_io_cpu", &s_phone_io_cpu_lock);
        const esp_err_t e2 = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0,
                                                "phone_io_ls",  &s_phone_io_ls_lock);
        if (e1 != ESP_OK || e2 != ESP_OK)
        {
            ESP_LOGE("PWR", "phone-IO PM lock create FAILED (cpu=%d ls=%d) — transfers "
                            "will light-sleep on battery", (int)e1, (int)e2);
        }
    }
    const esp_err_t a1 = s_phone_io_cpu_lock ? esp_pm_lock_acquire(s_phone_io_cpu_lock)
                                             : ESP_FAIL;
    const esp_err_t a2 = s_phone_io_ls_lock  ? esp_pm_lock_acquire(s_phone_io_ls_lock)
                                             : ESP_FAIL;
    s_phone_io_pm_held = (a1 == ESP_OK && a2 == ESP_OK);
    if (!s_phone_io_pm_held)
    {
        ESP_LOGE("PWR", "phone-IO PM lock acquire FAILED (cpu=%d ls=%d)", (int)a1, (int)a2);
    }
}
static void phoneIoPmRelease()
{
    if (s_phone_io_ls_lock)  esp_pm_lock_release(s_phone_io_ls_lock);
    if (s_phone_io_cpu_lock) esp_pm_lock_release(s_phone_io_cpu_lock);
    s_phone_io_pm_held = false;
}
#else
static bool s_phone_io_pm_held = false;
static inline void phoneIoPmAcquire() {}
static inline void phoneIoPmRelease() {}
#endif

// #524: latch the last transfer's summary and keep re-emitting it for a few
// minutes so a battery download can be read back over USB afterwards
// (`idf.py monitor --no-reset`).  (OC L1788-1804)
static char     s_xfer_summary[256]     = {0};
static uint32_t s_xfer_reprint_until_ms = 0;
static uint32_t s_xfer_next_reprint_ms  = 0;
static constexpr uint32_t XFER_REPRINT_WINDOW_MS = 240000;  // 4 min to get a cable in
static constexpr uint32_t XFER_REPRINT_EVERY_MS  = 15000;

// beginPhoneIO / endPhoneIO — bracket a phone-serving operation (file list /
// delete / download).  MINI: the OC additionally paused I2S ingest and
// drained its rx ring; the mini has no link to pause — the flight side keeps
// running (the rocket isn't flying during a phone fetch, and its frames are
// dropped by enqueueFrame when no session is open).  PM locks are counting
// and every begin has a matching end on every path.  (OC L1806-1828)
static inline void beginPhoneIO()
{
    flash_op_active   = true;
    phoneIoPmAcquire();
}
static inline void endPhoneIO()
{
    flash_op_active   = false;
    phoneIoPmRelease();
}

// ==========================================================================
// SECTION: Flight-side telemetry ingest (replaces the OC's I2S processFrame)
// ==========================================================================
static const char* rocketStateToString(RocketState s)
{
    switch (s)
    {
        case INITIALIZATION: return "INITIALIZATION";
        case READY:          return "READY";
        case PRELAUNCH:      return "PRELAUNCH";
        case INFLIGHT:       return "INFLIGHT";
        case LANDED:         return "LANDED";
        case MAG_CALIBRATION: return "MAG_CAL";
        default:             return "UNKNOWN";
    }
}

// MINI: the in-process replacement for the OC's I2S frame ingest
// (processFrame, OC L2354-2880).  Snapshots mini_link::telem under its mux,
// refreshes the latest_* caches, and runs the exact side effects the OC hung
// off each frame type:
//   BMP585_MSG      → updateDerivedAltitudeFromBMP        (OC L2605-2613)
//   GNSS_MSG        → convertGNSSData cache               (OC L2718-2727)
//   NON_SENSOR_MSG  → state edges: freq lock, hop machine, READY latch,
//                     PRELAUNCH pre-create, alt rate, max speed
//                                                         (OC L2728-2777)
//   END_FLIGHT      → endLogging + flightlogEndFlight — the FC emitted
//                     END_FLIGHT exactly at LANDED entry (FC main L6949), so
//                     the LANDED transition is the equivalent edge here
//                                                         (OC L2793-2798)
//   MAG_CAL_STATUS_MSG / SENSOR_CAL_STATUS_MSG → direct BLE relay
//                                                         (OC L2635-2656)
// No dedup/stale filtering: that whole block (OC L2390-2508 + the #468
// DedupRebootPolicy) existed to defend against I2S DMA replay of stale TX
// descriptors.  In-process there is no replay source and no second clock
// domain, so it is deliberately absent (report-oc-main-anatomy §10).
static void serviceTelemFromFlight()
{
    mini_link::TelemState snap;
    portENTER_CRITICAL(&mini_link::telem_mux);
    snap = mini_link::telem;
    portEXIT_CRITICAL(&mini_link::telem_mux);

    static uint32_t seen_baro_us = 0;
    static uint32_t seen_nonsensor_us = 0;
    static uint32_t pushed_mag_cal_us = 0;
    static uint32_t pushed_sensor_cal_us = 0;

    if (snap.imu_update_us != 0)
    {
        latest_ism6_raw = snap.imu;
        latest_ism6_valid = true;
    }
    if (snap.baro_update_us != 0 && snap.baro_update_us != seen_baro_us)
    {
        seen_baro_us = snap.baro_update_us;
        latest_bmp_raw = snap.baro;
        latest_bmp_valid = true;
        updateDerivedAltitudeFromBMP();
    }
    if (snap.gnss_update_us != 0)
    {
        latest_gnss_raw = snap.gnss;
        sensor_converter.convertGNSSData(latest_gnss_raw, latest_gnss_si);
        latest_gnss_valid = true;
    }
    if (snap.nonsensor_update_us != 0 && snap.nonsensor_update_us != seen_nonsensor_us)
    {
        seen_nonsensor_us = snap.nonsensor_update_us;
        const RocketState prev_state = latest_rocket_state;
        latest_non_sensor = snap.nonsensor;
        latest_non_sensor_valid = true;
        latest_rocket_state = (RocketState)latest_non_sensor.rocket_state;
        // Flight-freeze sticky flag (issue #71) — safe on every update.
        updateFreqLockFromState(latest_rocket_state);
        // Per-packet hop state machine off the same edge (#40/#41).
        updateHopFromState(latest_rocket_state);

        // Latch READY entry for the slow-rendezvous silence timer.
        if (latest_rocket_state == READY && prev_state != READY)
        {
            ready_entry_ms = millis();
        }
        if (latest_rocket_state != prev_state && latest_rocket_state == PRELAUNCH)
        {
            max_alt_m = 0.0f;
            max_speed_mps = 0.0f;
            pressure_alt_rate_mps = 0.0f;
            ground_pressure_set = false;  // Re-acquire ground pressure for new flight

            // Pre-create log file now so there's no NAND stall at launch
            logger.prepareLogFile();
            flightlogBeginFlight();
            ESP_LOGI("OC", "PRELAUNCH - pre-creating log file");
        }
        // MINI END_FLIGHT equivalent: the FC sent END_FLIGHT at LANDED entry
        // (retried until enqueued, FC main L6945-6955); mirror that edge.
        if (latest_rocket_state == LANDED && prev_state != LANDED &&
            logger.isLoggingActive())
        {
            logger.endLogging();
            flightlogEndFlight();
            ESP_LOGI("OC", "LANDED - flight log closed (END_FLIGHT equivalent)");
        }
        // KF-filtered altitude rate from the flight side.
        pressure_alt_rate_mps = (float)latest_non_sensor.baro_alt_rate_dmps * 0.1f;
        updateDerivedSpeedFromNonSensor();
    }

    // Issue #96 / #132: cal status frames forwarded verbatim to BLE with the
    // 0xCA / 0xCB discriminators.  Only meaningful on a direct connection —
    // which is the only kind the mini has.
    if (snap.mag_cal_update_us != 0 && snap.mag_cal_update_us != pushed_mag_cal_us)
    {
        pushed_mag_cal_us = snap.mag_cal_update_us;
        ble_app.sendMagCalStatus(reinterpret_cast<const uint8_t*>(&snap.mag_cal),
                                 sizeof(MagCalStatusData));
    }
    if (snap.sensor_cal_update_us != 0 && snap.sensor_cal_update_us != pushed_sensor_cal_us)
    {
        pushed_sensor_cal_us = snap.sensor_cal_update_us;
        ble_app.sendSensorCalStatus(reinterpret_cast<const uint8_t*>(&snap.sensor_cal),
                                    sizeof(SensorCalStatusData));
    }
}

// ==========================================================================
// SECTION: LoRa payload build + TX pacing (OC L3134-3454)
// ==========================================================================
// #850: builds ONE of the two downlink frames — see the OC twin for the full
// rationale. The mini has no TPS22811 current monitors, so the slow frame's
// cam_ma / servo_ma go out as 0.
static bool buildLoRaPayload(uint8_t out_payload[SIZE_OF_LORA_BUDGET], uint16_t seq,
                             uint8_t frame_type, size_t& out_len)
{
    out_len = 0;
    if (out_payload == nullptr)
    {
        return false;
    }

    LoRaDataSI lora = {};
    // Routing header
    lora.network_id = network_id;
    lora.rocket_id  = rocket_id;
    lora.seq        = seq;
    // Hop byte (#40/#41): where the BS should expect packet seq+1.  With
    // seq-anchored slow-hop (#105) this is a sanity hint — the BS derives the
    // same channel from seq itself.  0xFF = not hopping.
    if (hop_active_ && hop_fallback_state == HopFallbackState::NORMAL)
    {
        const uint8_t n = loraChannelCount(lora_bw_khz);
        if (n == 0)
        {
            lora.next_channel_idx = LORA_NEXT_CH_NO_HOP;
        }
        else
        {
            // Bootstrap packets (#150) all announce the SCHEDULE ENTRY
            // channel; on-schedule packets announce seq+1 as always.
            uint8_t mask_buf[LORA_SKIP_MASK_MAX_BYTES];
            const uint8_t* mask = effectiveHopMask(mask_buf, n);
            const uint16_t anchor_seq = (hop_bootstrap_left_ > 0)
                ? (uint16_t)(seq + hop_bootstrap_left_)
                : (uint16_t)(seq + 1);
            lora.next_channel_idx = loraHopChannelForSeq(
                anchor_seq, currentHopDwell(), mask, n);
        }
    }
    else if (hop_active_)
    {
        // #150 (review): rendezvous visit / scan pause — session live but
        // this frame is off-schedule.  Tell the BS the truth (0xFE).
        lora.next_channel_idx = LORA_NEXT_CH_HOP_OFFSCHEDULE;
    }
    else
    {
        lora.next_channel_idx = LORA_NEXT_CH_NO_HOP;
    }

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
    lora.camera_recording = false;   // MINI: no camera fitted
    lora.logging_active = logger.isLoggingActive();
    // #835 item 9 — mini half; see the OC comment.
    lora.sim_active     = nsFlagSet(latest_non_sensor.flags, NSF_SIM_ACTIVE);

    if (latest_non_sensor_valid)
    {
        lora.q0 = (float)latest_non_sensor.q0 / 10000.0f;
        lora.q1 = (float)latest_non_sensor.q1 / 10000.0f;
        lora.q2 = (float)latest_non_sensor.q2 / 10000.0f;
        lora.q3 = (float)latest_non_sensor.q3 / 10000.0f;
        lora.launch_flag = nsFlagSet(latest_non_sensor.flags, NSF_LAUNCH);
        lora.vel_u_apogee_flag = nsFlagSet(latest_non_sensor.flags, NSF_VEL_APOGEE);
        lora.alt_apogee_flag = nsFlagSet(latest_non_sensor.flags, NSF_ALT_APOGEE);
        lora.alt_landed_flag = nsFlagSet(latest_non_sensor.flags, NSF_ALT_LANDED);

        // #191: EKF ENU velocity components for the app's ascent prediction.
        lora.vel_e = (float)latest_non_sensor.e_vel / 100.0f;
        lora.vel_n = (float)latest_non_sensor.n_vel / 100.0f;
        lora.vel_u = (float)latest_non_sensor.u_vel / 100.0f;
        lora.burnout_detected = nsFlagSet(latest_non_sensor.flags, NSF_BURNOUT);
    }

    // #390: board→rocket orientation rides flags2 bits 1-7.  MINI: the OC
    // sourced this from the FC status-query cache (last_query_cfg), which has
    // no mini_link::telem equivalent yet.  Wire mode 0 = "not reported" (the
    // pre-#390 value), so nothing false can render at the BS.  If the link
    // struct grows b2r_code/b2r_mode, restore the OC mapping (OC L3237-3268).
    lora.imu_orient_mode = LORA2_OMODE_NONE;
    lora.imu_orient_code = 0;

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
    }

    // #386: BMP585 die temperature as board temperature.
    if (latest_bmp_valid)
    {
        BMP585DataSI bmp_si = {};
        sensor_converter.convertBMP585Data(latest_bmp_raw, bmp_si);
        lora.temp = bmp_si.temperature;
    }

    if (latest_power_valid)
    {
        POWERDataSI power_si = {};
        sensor_converter.convertPowerData(latest_power_raw, power_si);
        lora.voltage = power_si.voltage;
        lora.current = power_si.current;
        lora.soc = power_si.soc;

        // #303: relay the flight side's health verdicts and OR in the battery
        // state (the flight side leaves battery N/A — only comms reads power).
        uint32_t sh = latest_non_sensor_valid ? latest_non_sensor.sensor_health : 0u;
        lora.sensor_health = shSet(sh, SH_BATT_SHIFT, shBatteryState(power_si.voltage));
    }

    // #281/#278: fold in the comms-owned storage verdict regardless of power
    // validity, so a full/failing NAND surfaces even before the pack reads.
    {
        uint32_t sh = latest_power_valid ? lora.sensor_health
                     : (latest_non_sensor_valid ? latest_non_sensor.sensor_health : 0u);
        lora.sensor_health = shSet(sh, SH_STORAGE_SHIFT, ocStorageHealth());
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

    if (frame_type == LORA_FRAME_SLOW)
    {
        sensor_converter.packLoRaSlowBytes(lora, out_payload);
        out_len = SIZE_OF_LORA_SLOW;
    }
    else
    {
        sensor_converter.packLoRaFastBytes(lora, out_payload);
        out_len = SIZE_OF_LORA_FAST;
    }
    return true;
}

static void serviceLoRa()
{
    if (!config::USE_LORA_RADIO)
    {
        return;
    }

    lora_comms.service();

    // Honour any pending hop retune as soon as the radio is idle — canSend()
    // going true is the "TX done" signal.  Skip during a rendezvous visit;
    // serviceHopFallback() is managing the radio then.
    if (hop_needs_retune_ && lora_comms.canSend() &&
        hop_fallback_state == HopFallbackState::NORMAL)
    {
        (void)lora_comms.hopToFrequencyMHz(hopTargetFreqMHz());
        hop_needs_retune_ = false;
    }

    // "LoRa off": every transmit path stops here.  After service() and the
    // pending retune on purpose — those are what finish an in-flight TX and
    // bring the radio back to lora_freq_mhz, and returning above them would
    // strand it muted on a hop channel with no way to be called back.
    if (lora_tx_disabled)
    {
        return;
    }

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

    // #850: five FAST then one SLOW, keyed off the seq that goes on the wire.
    uint8_t payload[SIZE_OF_LORA_BUDGET] = {0};
    size_t  payload_len = 0;
    if (!buildLoRaPayload(payload, lora_tx_seq, loraFrameTypeForSlot(lora_tx_seq), payload_len))
    {
        return;
    }
    last_lora_tx_ms = now_ms;
    lora_in_rx_mode = false;  // Exiting RX for TX
    if (lora_comms.send(payload, payload_len))
    {
        lora_tx_ok++;

        // Persist the exact bytes that went on the air as a LORA_MSG (0xF1)
        // record — 55 B (FAST) or 22 B (SLOW) since #850, hence payload_len
        // rather than sizeof(payload).
        // record — the rocket-side half of the per-packet loss measurement
        // (2026-08-08 Kaua'i range test).  Logged AFTER send() returns true
        // and BEFORE lora_tx_seq++ so the record holds the seq actually
        // transmitted; enqueueFrame() drops it when no session is open.
        // Full rationale at OC L3377-3403.  Beacons are NOT logged.
        {
            uint8_t lora_frame[MAX_FRAME];
            size_t  lora_frame_len = 0;
            if (TR_I2C_Interface::packMessage(LORA_MSG,
                                              payload, payload_len,
                                              lora_frame, sizeof(lora_frame),
                                              lora_frame_len))
            {
                if (logger.enqueueFrame(lora_frame, lora_frame_len)) lora_tx_logged++;
            }
        }

        // Advance the seq AFTER a successful send — a failed startTransmit()
        // means nothing went over the air, so the BS shouldn't see a gap.
        lora_tx_seq++;

        // Advance hop state and schedule the post-TX retune (top of the next
        // serviceLoRa pass applies it).  Skip during a rendezvous visit.
        if (hop_active_ && hop_fallback_state == HopFallbackState::NORMAL)
        {
            if (hop_bootstrap_left_ > 0)
            {
                hop_bootstrap_left_--;
            }
            // Recompute hop_idx_ from the just-incremented seq — MUST equal
            // the next_channel_idx the packet header announced (both sides
            // derive from the same formula).
            const uint8_t n = loraChannelCount(lora_bw_khz);
            if (n > 0)
            {
                uint8_t mask_buf[LORA_SKIP_MASK_MAX_BYTES];
                const uint8_t* mask = effectiveHopMask(mask_buf, n);
                hop_idx_ = loraHopChannelForSeq(
                    lora_tx_seq, currentHopDwell(), mask, n);
            }
            hop_needs_retune_ = true;
        }
    }
    else
    {
        lora_tx_fail++;
    }
}

// Send a LoRa name beacon so the base station can learn this rocket's name.
static uint32_t last_beacon_ms = 0;

static void sendLoRaBeacon()
{
    if (!config::USE_LORA_RADIO) return;
    if (lora_tx_disabled) return;   // "LoRa off" — the beacon is a transmit too

    // Beacon in any state EXCEPT INFLIGHT (issue #71 field test: a rocket
    // whose flight side is slow to report state must still advertise).
    if (!shouldBeaconInState(latest_rocket_state)) return;

    // #150: no beacons while the hop schedule is running — a beacon inside a
    // dwell visit can blow the FCC occupancy budget (see the
    // BeaconSuppressionDuringHopIsLoadBearing host test).
    if (hop_active_ && hop_fallback_state == HopFallbackState::NORMAL) return;

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

// ==========================================================================
// SECTION: Config readback to the app (OC L3501-3712, trimmed)
// ==========================================================================
// #398 item 3: senders enqueue frames here and the loop drains ONE per pass,
// paced >= the BLE connection interval.  Enqueue and drain both run on the
// comms loop, so the ring needs no locking.
static constexpr uint8_t  CFG_RB_CAP     = 8;
static constexpr uint32_t CFG_RB_PACE_MS = 20;   // >= negotiated max conn interval
static String   cfg_rb_queue[CFG_RB_CAP];
static uint8_t  cfg_rb_head  = 0;
static uint8_t  cfg_rb_count = 0;
static uint32_t cfg_rb_last_ms = 0;

static void enqueueConfigReadback(const String& json)
{
    if (json.length() == 0) return;
    if (cfg_rb_count >= CFG_RB_CAP)
    {
        // Backed up — drop the oldest so the freshest snapshot gets through.
        cfg_rb_head = (cfg_rb_head + 1) % CFG_RB_CAP;
        cfg_rb_count--;
        ESP_LOGW("CFG", "readback queue full — dropped oldest frame");
    }
    cfg_rb_queue[(cfg_rb_head + cfg_rb_count) % CFG_RB_CAP] = json;
    cfg_rb_count++;
}

static void serviceConfigReadbackQueue()
{
    if (cfg_rb_count == 0) return;
    const uint32_t now = millis();
    if ((uint32_t)(now - cfg_rb_last_ms) < CFG_RB_PACE_MS) return;
    ble_app.sendConfigJSON(cfg_rb_queue[cfg_rb_head]);
    cfg_rb_queue[cfg_rb_head] = String();   // release the String's heap buffer
    cfg_rb_head = (cfg_rb_head + 1) % CFG_RB_CAP;
    cfg_rb_count--;
    cfg_rb_last_ms = now;
}

static void sendCurrentConfig()
{
    // Message 1: "config".  MINI: trimmed to what exists on this board — the
    // servo/PID/roll/guidance/camera keys the OC emitted are gone with the
    // hardware, and emitting defaults would pretend the features exist.  The
    // app must tolerate the missing keys (flagged for integration test).
    // LoRa keys + hop dwell + IMU rate survive unchanged.
    String j = "{\"type\":\"config\"";
    j += ",\"irate\":"; j += itos(cfg_imu_rate);
    // LoRa settings
    j += ",\"lf\":";  j += fmtf(lora_freq_mhz, 1);
    j += ",\"lsf\":"; j += itos(lora_sf);
    j += ",\"lbw\":"; j += fmtf(lora_bw_khz, 0);
    j += ",\"lcr\":"; j += itos(lora_cr);
    j += ",\"lpw\":"; j += itos(lora_tx_power);
    j += ",\"lhd\":"; j += lora_hop_disabled ? "true" : "false";  // #106
    // "LoRa off" mute; lenient-optional in both apps, so absence means
    // "firmware predates the feature", not "transmitting".
    j += ",\"ltxd\":"; j += lora_tx_disabled ? "true" : "false";
    // #150: airtime-derived hop dwell; 0 greys the option out in the app.
    j += ",\"lhdw\":"; j += itos(currentHopDwell());
    j += "}";
    enqueueConfigReadback(j);   // #398 item 3: paced drain, no delay()
    ESP_LOGI("CFG", "Queued config readback (%u bytes)", (unsigned)j.length());

    // Message 2: pyro config ("config_pyro" type) — 4 channels (OC L3671-3683)
    String p = "{\"type\":\"config_pyro\"";
    static const char* PE_KEYS[4] = { "p1e", "p2e", "p3e", "p4e" };
    static const char* PM_KEYS[4] = { "p1m", "p2m", "p3m", "p4m" };
    static const char* PV_KEYS[4] = { "p1v", "p2v", "p3v", "p4v" };
    for (int i = 0; i < 4; ++i) {
        p += ",\""; p += PE_KEYS[i]; p += "\":"; p += cfg_pyro_enabled[i]      ? "true" : "false";
        p += ",\""; p += PM_KEYS[i]; p += "\":"; p += itos(cfg_pyro_trigger_mode[i]);
        p += ",\""; p += PV_KEYS[i]; p += "\":"; p += fmtf(cfg_pyro_trigger_value[i], 1);
    }
    p += "}";
    enqueueConfigReadback(p);   // #398 item 3
    ESP_LOGI("CFG", "Queued pyro config readback (%u bytes)", (unsigned)p.length());

    // Message 3: device identity ("config_identity" type).  "dt" stays the
    // OC's exact DEVICE_TYPE string so existing apps keep working; "fw" is
    // the app-descriptor version (PROJECT_VER: git sha + board suffix).
    const esp_app_desc_t* app_desc = esp_app_get_description();
    const char* fw_ver = (app_desc && app_desc->version[0]) ? app_desc->version : "unknown";
    char id_buf[192];
    snprintf(id_buf, sizeof(id_buf),
             "{\"type\":\"config_identity\""
             ",\"uid\":\"%s\""
             ",\"un\":\"%s\""
             ",\"nid\":%u"
             ",\"rid\":%u"
             ",\"dt\":\"%s\""
             ",\"fw\":\"%s\"}",
             unit_id_hex, unit_name,
             (unsigned)network_id, (unsigned)rocket_id,
             config::DEVICE_TYPE,
             fw_ver);
    String id_json(id_buf);
    enqueueConfigReadback(id_json);   // #398 item 3
    ESP_LOGI("CFG", "Queued identity readback (%u bytes)", (unsigned)id_json.length());

    // MINI: the OC's fc_identity / imu_orient / guid_target siblings are not
    // sent.  fc_identity described the SECOND MCU's firmware (gone); the app
    // already treats its absence as a pre-#8 device.  imu_orient / guid_target
    // were sourced from the FC status query, which mini_link::telem does not
    // carry yet — the OC itself sent nothing when the query was pre-v3/v5, so
    // absence is the already-handled degraded case.  Restore imu_orient when
    // the link struct grows b2r fields (see buildLoRaPayload note).
}

// ==========================================================================
// SECTION: LoRa uplink command handling (OC L3780-4366)
// ==========================================================================

// Apply the "LoRa off" transmit mute (BLE cmd 68 / uplink cmd 68 both land
// here so the two transports cannot diverge).  Desired-state, not a toggle: a
// retried uplink must not silence a rocket the first copy already un-silenced.
// Returns true when honoured (including already-there), false when refused
// because the rocket is airborne.
static bool applyLoRaTxMute(bool want_disabled, const char* src)
{
    if (!loraTxMuteChangeAllowed(want_disabled, latest_rocket_state))
    {
        ESP_LOGW("LORA", "%s LoRa TX mute REFUSED: rocket INFLIGHT — muting an "
                         "airborne rocket would drop the only tracking link it has",
                 src);
        return false;
    }
    if (want_disabled == lora_tx_disabled)
    {
        ESP_LOGI("LORA", "%s LoRa TX already %s", src,
                 lora_tx_disabled ? "OFF (muted)" : "ON");
        return true;
    }

    lora_tx_disabled = want_disabled;
    prefs.begin("lora", false);
    prefs.putUChar("txdis", lora_tx_disabled ? 1 : 0);
    prefs.end();

    // Re-evaluate the hop schedule: muting takes the ON→OFF branch (radio back
    // to lora_freq_mhz), un-muting restarts hopping if state + link mode want it.
    updateHopFromState(latest_rocket_state);

    // Echo to a directly-connected app so its toggle can't go stale when the
    // base station is the one that changed this.  No-ops when disconnected.
    sendCurrentConfig();

    ESP_LOGW("OC", "[LORA] Transmit %s (%s) — %s",
             lora_tx_disabled ? "MUTED" : "UNMUTED", src,
             lora_tx_disabled
                 ? "no telemetry, no beacon, no hopping; still listening for uplink"
                 : "telemetry and beacon resume");
    return true;
}
static void processUplinkCommand(uint8_t cmd, const uint8_t* payload, size_t payload_len)
{
    ESP_LOGI("LORA", "UPLINK RX cmd=%u payload_len=%u", cmd, (unsigned)payload_len);

    // #383: refuse camera/logging/guidance-point while INFLIGHT (OC gate kept
    // even though 1 and 28 are unsupported below — the refusal-with-count is
    // the honest first answer either way).
    // cmds 35/36 (pyro cont test / TEST-FIRE via BS relay) join: queued
    // mid-flight they would deliver at landing — a delayed FIRE pulse firing
    // while the recovery crew walks up is the stale-command hazard this gate
    // refuses. The flight side's lockout gate is the second layer.
    if ((cmd == 1 || cmd == 23 || cmd == 28 || cmd == 35 || cmd == 36) &&
        latest_rocket_state == INFLIGHT)
    {
        uplink_inflight_refusals++;
        ESP_LOGW("LORA", "UPLINK cmd=%u refused: rocket INFLIGHT (undeliverable"
                         " until landing; %lu refused so far)",
                 cmd, (unsigned long)uplink_inflight_refusals);
        return;
    }

    if (cmd == 23)
    {
        // Logging: payload[0] = desired state; falls back to toggle.
        bool want_on = (payload_len >= 1) ? (payload[0] != 0)
                                          : !logger.isLoggingActive();
        if (want_on && !logger.isLoggingActive())
        {
            // Mirror the BLE cmd 23 start path: prepareLogFile opens the sink
            // session and flightlogBeginFlight allocates the block range —
            // startLogging alone would flip logging_active with every frame
            // rejected at enqueue (#72).
            logger.prepareLogFile();
            flightlogBeginFlight();
            logger.startLogging();
            ESP_LOGI("LORA", "UPLINK Logging started");
        }
        else if (!want_on && logger.isLoggingActive())
        {
            logger.endLogging();
            flightlogEndFlight();
            ESP_LOGI("LORA", "UPLINK Logging stopped");
        }
        else
        {
            ESP_LOGI("LORA", "UPLINK Logging already %s, ignoring",
                          want_on ? "ON" : "OFF");
        }
    }
    else if (cmd == 5 && payload_len >= 12)
    {
        // Sim config → flight side (was: relay to FC over I2C)
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
        mini_link::sendCommand(SIM_CONFIG_MSG,
                               reinterpret_cast<const uint8_t*>(&sim_cfg),
                               sizeof(sim_cfg));
        ESP_LOGI("LORA", "UPLINK Sim config queued: mass=%.0fg thrust=%.1fN burn=%.1fs descent=%.1fm/s",
                      (double)mass_g, (double)sim_cfg.thrust_n,
                      (double)sim_cfg.burn_time_s, (double)sim_cfg.descent_rate_mps);
    }
    else if (cmd == 6)
    {
        mini_link::sendCommand(SIM_START_CMD, nullptr, 0);
        ESP_LOGI("LORA", "UPLINK Sim start queued for flight side");
    }
    else if (cmd == 7)
    {
        logger.endLogging();
        flightlogEndFlight();
        mini_link::sendCommand(SIM_STOP_CMD, nullptr, 0);
        ESP_LOGI("LORA", "UPLINK Sim stop queued for flight side (logging ended)");
    }
    else if (cmd == 35 && payload_len >= 1)
    {
        // Pyro continuity test via BS relay — same handling as BLE cmd 35.
        // No duplicate suppression: the BS retry train just re-runs the
        // momentary arm→read→disarm, which keeps the reading fresh.
        uint8_t ch = payload[0];
        if (ch < 1 || ch > 4) {
            ESP_LOGW("LORA", "UPLINK Pyro continuity test: invalid channel %u", ch);
        } else {
            mini_link::sendCommand(PYRO_CONT_TEST, &ch, 1);
            ESP_LOGI("LORA", "UPLINK Pyro continuity test CH%u", ch);
        }
    }
    else if (cmd == 36 && payload_len >= 1)
    {
        // Pyro TEST-FIRE via BS relay — the LoRa half of the stand-back pyro
        // test (app → BS cmd 50 → here). The uplink is blind fire-and-retry
        // with no sequence number, so one FIRE tap arrives as up to
        // UPLINK_RETRIES identical packets; suppress same-channel repeats.
        // Worst-case train span is ~11 s: 7 inter-retry gaps × (100 ms
        // pacing + the TX-window gate's 1.5 s max_defer, re-armed per
        // attempt). 13 s covers that with margin while staying under the
        // fastest deliberate re-test of the same channel (5 s recording +
        // 10 s countdown ≈ 15 s away at minimum). (OC has the same gate;
        // no rail-off refusal here — the flight side is this same MCU and
        // its command queue drains every flight tick, so nothing can hold a
        // fire for later.)
        static uint32_t last_fire_uplink_ms = 0;
        static uint8_t  last_fire_uplink_ch = 0;
        constexpr uint32_t kFireDedupWindowMs = 13000;
        uint8_t ch = payload[0];
        if (ch < 1 || ch > 4) {
            ESP_LOGW("LORA", "UPLINK Pyro test fire: invalid channel %u", ch);
        } else if (ch == last_fire_uplink_ch && last_fire_uplink_ms != 0 &&
                   (millis() - last_fire_uplink_ms) < kFireDedupWindowMs) {
            ESP_LOGI("LORA", "UPLINK Pyro test fire CH%u: duplicate retry ignored", ch);
        } else {
            last_fire_uplink_ch = ch;
            last_fire_uplink_ms = millis();
            mini_link::sendCommand(PYRO_FIRE_TEST, &ch, 1);
            ESP_LOGI("LORA", "UPLINK Pyro test fire CH%u", ch);
        }
    }
    else if (cmd == 10 && payload_len >= 11)
    {
        // LoRa reconfiguration via uplink: [freq:4f][bw:4f][sf:1][cr:1][txpwr:1]
        // Reject while the link is committed — freq-locked for flight (#71)
        // or actively hopping (#40/#41).
        if (freq_locked_for_flight || hop_active_)
        {
            ESP_LOGW("LORA", "UPLINK Cmd 10 ignored: %s",
                     freq_locked_for_flight ? "frequency locked for flight"
                                            : "channel hopping active");
            return;
        }

        float new_freq, new_bw;
        memcpy(&new_freq, payload + 0, 4);
        memcpy(&new_bw,   payload + 4, 4);
        uint8_t new_sf   = payload[8];
        uint8_t new_cr   = payload[9];
        int8_t  new_pwr  = (int8_t)payload[10];

        if (lora_comms.reconfigure(new_freq, new_sf, new_bw, new_cr, new_pwr))
        {
            const float old_bw = lora_bw_khz;
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
            // BW change invalidates the channel-set skip-mask (#40/#41 ph 3).
            if (old_bw != lora_bw_khz)
            {
                skip_mask_n_        = 0;
                channel_set_bw_khz_ = 0.0f;
                for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) skip_mask_[i] = 0;
                prefs.remove("chset_n");
                prefs.remove("chset_bw");
                prefs.remove("chset_mask");
                ESP_LOGI("LORA", "UPLINK [CHSET] BW changed — skip-mask invalidated");
            }
            prefs.end();

            ESP_LOGI("LORA", "UPLINK LoRa reconfigured + saved: %.1f MHz SF%u BW%.0f CR%u %d dBm",
                          (double)lora_freq_mhz, (unsigned)lora_sf,
                          (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);

            // #569: reconfigure() leaves the radio in STANDBY; without this
            // startReceive() the stale rx-flag suppressed recovery and the
            // radio sat deaf on the NEW modulation until the next TX.
            lora_comms.startReceive();
            lora_in_rx_mode = true;
        }
        else
        {
            ESP_LOGE("LORA", "UPLINK LoRa reconfigure FAILED");
        }
    }
    else if (cmd == LORA_CMD_SET_HOP_DISABLED && payload_len >= 1)
    {
        // BS-controlled hop enable/disable (#106).  Persisted; enable is
        // deferred past the BS's retry train (#150).
        const bool new_disabled = (payload[0] != 0);
        if (!new_disabled && currentHopDwell() == 0)
        {
            // #150: modulation can't fit one packet in the FCC dwell budget.
            ESP_LOGW("LORA", "UPLINK Hop enable REFUSED: dwell=0 at SF%u/BW%.0f",
                     (unsigned)lora_sf, (double)lora_bw_khz);
        }
        else if (new_disabled != lora_hop_disabled)
        {
            lora_hop_disabled = new_disabled;
            prefs.begin("lora", false);
            prefs.putUChar("hopdis", lora_hop_disabled ? 1 : 0);
            prefs.end();
            if (!lora_hop_disabled)
            {
                hop_enable_apply_at_ms = millis() + HOP_ENABLE_DEFER_MS;
                ESP_LOGI("LORA", "UPLINK Hop disable: ENABLED — activating in %u ms (after BS retry train)",
                         (unsigned)HOP_ENABLE_DEFER_MS);
            }
            else
            {
                hop_enable_apply_at_ms = 0;   // a disable cancels any pending enable
                ESP_LOGI("LORA", "UPLINK Hop disable: DISABLED (fixed freq) — re-evaluating hop state");
                updateHopFromState(latest_rocket_state);
            }
        }
        else
        {
            ESP_LOGI("LORA", "UPLINK Hop disable: already %s",
                     lora_hop_disabled ? "DISABLED" : "ENABLED");
        }
    }
    else if (cmd == LORA_CMD_SET_TX_DISABLED && payload_len >= 1)
    {
        // "LoRa off" over the air.  The useful direction is UN-muting — a
        // muted rocket keeps listening precisely so the BS can call it back —
        // but the command is symmetric so the app's toggle behaves the same
        // direct or relayed.
        (void)applyLoRaTxMute(payload[0] != 0, "UPLINK");
    }
    else if (cmd == LORA_CMD_CHANNEL_SET && payload_len >= 5)
    {
        // Channel-set push from BS (#40/#41 phase 3):
        //   [bw:f4][n_channels:u1][skip_mask: ceil(n/8) bytes]
        float new_bw;
        memcpy(&new_bw,  payload + 0, 4);
        const uint8_t new_n = payload[4];
        const size_t  mask_bytes = (size_t)(new_n + 7) / 8;
        if (payload_len < 5 + mask_bytes || mask_bytes > LORA_SKIP_MASK_MAX_BYTES)
        {
            ESP_LOGW("LORA", "UPLINK Cmd 15 ignored: payload truncated (len=%u, need %u)",
                     (unsigned)payload_len, (unsigned)(5 + mask_bytes));
            return;
        }
        // Reject a mask sized for a different hop table (BW mismatch).
        if (new_bw != lora_bw_khz)
        {
            ESP_LOGW("LORA", "UPLINK Cmd 15 ignored: BW mismatch "
                              "(payload=%.0f kHz, ours=%.0f kHz)",
                     (double)new_bw, (double)lora_bw_khz);
            return;
        }
        skip_mask_n_        = new_n;
        channel_set_bw_khz_ = new_bw;
        for (size_t i = 0; i < LORA_SKIP_MASK_MAX_BYTES; i++) skip_mask_[i] = 0;
        for (size_t i = 0; i < mask_bytes;          i++) skip_mask_[i] = payload[5 + i];

        // Persist (rdv_mhz no longer stored — #105 / NVS schema v3)
        Preferences p;
        if (p.begin("lora", false))
        {
            p.putUChar("chset_n", skip_mask_n_);
            p.putFloat("chset_bw", channel_set_bw_khz_);
            p.putBytes("chset_mask", skip_mask_, mask_bytes);
            p.end();
        }

        uint8_t active = 0;
        for (uint8_t i = 0; i < skip_mask_n_; i++)
            if (!loraSkipMaskTest(skip_mask_, i)) active++;
        ESP_LOGI("LORA", "UPLINK Cmd 15: %u/%u active at BW=%.0f kHz",
                 (unsigned)active,
                 (unsigned)skip_mask_n_, (double)channel_set_bw_khz_);
    }
    else if (cmd == LORA_CMD_HOP_PAUSE && payload_len >= 2)
    {
        // Coordinated hop pause from BS (#90): park on lora_freq_mhz with the
        // operating preset for N ms so the BS can scan + push cmd 15.
        uint16_t dur_ms;
        memcpy(&dur_ms, payload + 0, 2);
        if (dur_ms == 0)
        {
            ESP_LOGW("LORA", "UPLINK Cmd 16 ignored: zero duration");
            return;
        }
        if (dur_ms > LORA_HOP_PAUSE_MAX_MS) dur_ms = LORA_HOP_PAUSE_MAX_MS;
        if (!hop_active_)
        {
            ESP_LOGI("LORA", "UPLINK Cmd 16 ignored: not hopping");
            return;
        }
        // Idempotent if already paused — extend the deadline.
        if (hop_fallback_state == HopFallbackState::PAUSED_FOR_SCAN)
        {
            hop_pause_until_ms = millis() + dur_ms;
            ESP_LOGI("OC", "[HOP] Pause extended: +%u ms", (unsigned)dur_ms);
            return;
        }
        if (!lora_comms.reconfigure(lora_freq_mhz, lora_sf, lora_bw_khz,
                                     lora_cr, lora_tx_power))
        {
            ESP_LOGE("LORA", "UPLINK Cmd 16: reconfigure to lora_freq_mhz failed");
            return;
        }
        (void)lora_comms.startReceive();
        lora_in_rx_mode = true;
        hop_pause_until_ms = millis() + dur_ms;
        hop_fallback_state = HopFallbackState::PAUSED_FOR_SCAN;
        ESP_LOGW("OC", "[HOP] Paused for scan: %.2f MHz for %u ms",
                 (double)lora_freq_mhz, (unsigned)dur_ms);
    }
    else if (cmd == LORA_CMD_HEARTBEAT)
    {
        // Heartbeat from BS (issue #71).  last_uplink_rx_ms is already
        // updated in the caller.
        ESP_LOGV("LORA", "UPLINK heartbeat");
    }
    else if (cmd == 1 || cmd == 24 || cmd == 25 || cmd == 28 || cmd == 12 ||
             cmd == 13 || cmd == 14 || cmd == 22 || cmd == 31 || cmd == 32 ||
             cmd == 66)
    {
        // MINI: camera (1), servo test (24/25), guidance point (28), servo
        // config (12), PID (13), servo ctrl (14), gain schedule (22), roll
        // control (31), guidance enable (32), fin layout (66) — hardware
        // gone.  Say so instead of pretending (spec rule).
        ESP_LOGW("LORA", "UPLINK cmd %u unsupported on rocket-computer-mini "
                         "(servo/camera/guidance hardware absent)", cmd);
    }
    else
    {
        ESP_LOGW("LORA", "UPLINK Unknown cmd %u", cmd);
    }
}

// Write one LORA_UPLINK_MSG (0xF9) record for a decode the radio handed up —
// the rocket's only measurement of the RF path; every disposition is
// recorded.  (OC L4209-4245)
static void logUplinkRx(const TR_LoRa_Comms::Stats& ls, uint8_t cmd, uint8_t flags)
{
    LoRaUplinkData ul = {};
    ul.time_us       = (uint32_t)esp_timer_get_time();
    ul.rssi_dbm_x10  = loraPackTenths(ls.last_rssi);
    ul.snr_db_x10    = loraPackTenths(ls.last_snr);
    const float mhz  = lora_comms.currentFrequencyMHz();
    const float khz  = (mhz - 900.0f) * 1000.0f;
    ul.freq_khz_o900 = (khz < 0.0f) ? 0u
                     : (khz > 65535.0f) ? 65535u
                     : (uint16_t)(khz + 0.5f);
    ul.sf            = lora_comms.currentSpreadingFactor();
    ul.cmd           = cmd;
    ul.flags         = flags;

    uint8_t frame[MAX_FRAME];
    size_t  frame_len = 0;
    if (TR_I2C_Interface::packMessage(LORA_UPLINK_MSG,
                                      (const uint8_t*)&ul, sizeof(ul),
                                      frame, sizeof(frame), frame_len))
    {
        if (logger.enqueueFrame(frame, frame_len)) lora_uplink_logged++;
    }
}

/// Enter RX mode between TX cycles and check for uplink commands
static void serviceLoRaUplink()
{
    if (!config::USE_LORA_RADIO) return;

    lora_comms.service();  // Complete any pending TX (auto-enters RX after TX)

    // Watchdog BEFORE the canSend() gate — below it, the one path meant to
    // clear a wedge was unreachable from the wedge (#105, OC L4254-4263).
    lora_comms.serviceTxWatchdog();

    // Only enter RX when radio is idle (not transmitting)
    if (!lora_comms.canSend()) return;

    // Sync the tracking flag to avoid a redundant startReceive() that would
    // reset rx_done_ and potentially drop a received packet.
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

    // Poll DIO1 directly as fallback for the RX-done interrupt.
    lora_comms.pollDio1();

    // Non-blocking poll for uplink packet
    uint8_t rx_buf[32];
    size_t rx_len = 0;

    if (lora_comms.readPacket(rx_buf, sizeof(rx_buf), rx_len))
    {
        // SNR floor (#90 follow-up): drop noise-floor false positives before
        // they can fire processUplinkCommand.
        TR_LoRa_Comms::Stats ls = {};
        lora_comms.getStats(ls);
        const float min_snr = loraMinValidSnrDb(lora_comms.currentSpreadingFactor());
        if (ls.last_snr < min_snr)
        {
            lora_low_snr_drops++;
            // Logged before the return — dropping these would bias the
            // logged RSSI distribution upward exactly where the link is
            // marginal.
            logUplinkRx(ls, 0, LORA_UL_SNR_DROP);
            ESP_LOGW("LORA", "RX drop: SNR %.1f dB < %.1f dB floor (SF%u) "
                              "— likely noise-floor false positive",
                     (double)ls.last_snr, (double)min_snr,
                     (unsigned)lora_comms.currentSpreadingFactor());
            return;  // skip uplink processing for this iteration
        }

        // Uplink format v2: [0xCA][network_id][target_rid][next_channel_idx][cmd][len][payload...]
        if (rx_len >= 6 && rx_buf[0] == config::UPLINK_SYNC_BYTE)
        {
            uint8_t pkt_nid = rx_buf[1];
            uint8_t pkt_rid = rx_buf[2];
            // Filter: must match our network, and target us or broadcast (0xFF)
            if (pkt_nid == network_id &&
                (pkt_rid == rocket_id || pkt_rid == 0xFF))
            {
                // rx_buf[3] = next_channel_idx — sentinel only in phase 1.
                uint8_t cmd = rx_buf[4];
                uint8_t payload_len = rx_buf[5];
                if (rx_len >= (size_t)(6 + payload_len))
                {
                    logUplinkRx(ls, cmd, LORA_UL_ACCEPTED);
                    processUplinkCommand(cmd, &rx_buf[6], payload_len);
                    lora_uplink_rx_count++;
                    last_uplink_rx_ms = millis();
                    if (hop_active_) hop_session_uplink_count++;
                }
                else
                {
                    // Truncated decode — a real signal-quality data point.
                    logUplinkRx(ls, cmd, LORA_UL_MALFORMED);
                }
            }
            else if (pkt_nid != network_id)
            {
                lora_uplink_nid_drops++;   // #150: no more silent nid drops
                logUplinkRx(ls, 0, LORA_UL_NID_DROP);
            }
            else
            {
                // Our network, addressed to a different rocket — still a
                // clean decode, so its RSSI is a valid path-loss sample.
                logUplinkRx(ls, 0, LORA_UL_NOT_FOR_US);
            }
        }
        else
        {
            logUplinkRx(ls, 0, LORA_UL_MALFORMED);
        }
        // readPacket() internally re-enters RX mode after reading
    }
}

// ==========================================================================
// SECTION: LoRa rendezvous and hop fallback (OC L4368-4699, verbatim)
// ==========================================================================
// Slow-rendezvous cycle (issue #71): if silent long enough, hop briefly to
// LORA_FACTORY_RENDEZVOUS_MHZ so the BS's Phase-A recovery has a guaranteed
// meeting point even when the two NVS freqs disagree.  See the OC's header
// comment for trigger/cycle sizing rationale.

enum class RocketRendezvousState : uint8_t {
    IDLE,
    ON_RENDEZVOUS,    // RENDEZVOUS_WINDOW_MS on LORA_FACTORY_RENDEZVOUS_MHZ
    ON_SAVED,         // RENDEZVOUS_SAVED_MS back on lora_freq_mhz (NVS)
};

static RocketRendezvousState rendezvous_state = RocketRendezvousState::IDLE;
static uint32_t rendezvous_phase_start_ms = 0;

// #105 follow-up: 15 s triggers; cycle keeps the BS Phase A 30 s window
// plenty of overlap for a clean handshake.
static constexpr uint32_t RENDEZVOUS_TRIGGER_INITIAL_MS = 15000;  // never heard BS yet
static constexpr uint32_t RENDEZVOUS_TRIGGER_QUIET_MS   = 15000;  // BS seen, then silent
static constexpr uint32_t RENDEZVOUS_WINDOW_MS          = 10000;  // on rendezvous freq
static constexpr uint32_t RENDEZVOUS_SAVED_MS           = 20000;  // back on saved freq

// Hop to the full rendezvous mode (freq + SF/BW/CR/power) — all five values
// compile-time shared with the BS (#105).  #398: returns success so the state
// machine only advances when the radio actually retuned; wait_for_tx=false so
// a mid-TX hop retries next loop instead of spin-waiting out the airtime.
static bool rendezvousHopToRendezvousMode()
{
    if (lora_comms.reconfigure(LORA_FACTORY_RENDEZVOUS_MHZ,
                                LORA_FACTORY_RENDEZVOUS_SF,
                                LORA_FACTORY_RENDEZVOUS_BW_KHZ,
                                LORA_FACTORY_RENDEZVOUS_CR,
                                LORA_FACTORY_RENDEZVOUS_TX_DBM,
                                /*wait_for_tx=*/false))
    {
        lora_comms.startReceive();
        lora_in_rx_mode = true;
        return true;
    }
    // Busy (mid-TX) or failed — caller retries next loop iteration.
    return false;
}

// Hop back to whatever NVS says — the working config the user picked.
static bool rendezvousHopToSavedMode()
{
    if (lora_comms.reconfigure(lora_freq_mhz, lora_sf, lora_bw_khz,
                                lora_cr, lora_tx_power,
                                /*wait_for_tx=*/false))
    {
        lora_comms.startReceive();
        lora_in_rx_mode = true;
        return true;
    }
    return false;
}

static void rendezvousExit(const char* why)
{
    if (rendezvous_state == RocketRendezvousState::IDLE) return;
    if (!rendezvousHopToSavedMode())
    {
        // Radio busy/failed — stay; the silence-broke check re-runs this
        // exit on the next loop iteration.
        return;
    }
    rendezvous_state = RocketRendezvousState::IDLE;
    ESP_LOGI("OC", "[RENDEZVOUS] Exit (%s); back on saved mode %.2f MHz SF%u",
             why, (double)lora_freq_mhz, (unsigned)lora_sf);
}

static void serviceRocketRendezvous()
{
    if (!config::USE_LORA_RADIO)  return;
    if (!peripherals_initialized) return;
    // Suppress while in flight (#71) or actively hopping (#40/#41) — both
    // want to own the radio frequency.
    if (freq_locked_for_flight || hop_active_)
    {
        rendezvousExit(freq_locked_for_flight ? "flight locked" : "hopping active");
        return;
    }

    const uint32_t now = millis();

    // Adaptive trigger: short window if we've never received an uplink.
    const uint32_t trigger_ms = (last_uplink_rx_ms == 0)
        ? RENDEZVOUS_TRIGGER_INITIAL_MS
        : RENDEZVOUS_TRIGGER_QUIET_MS;

    // Silence reference: most-recent uplink or READY entry; else boot.
    const uint32_t last_activity_ms = (last_uplink_rx_ms > ready_entry_ms)
        ? last_uplink_rx_ms : ready_entry_ms;
    const uint32_t silent_for = (last_activity_ms > 0)
        ? (now - last_activity_ms) : now;

    // Any fresh uplink breaks silence and returns us to the saved mode.
    if (rendezvous_state != RocketRendezvousState::IDLE &&
        silent_for < trigger_ms)
    {
        rendezvousExit("silence broke");
        return;
    }

    switch (rendezvous_state)
    {
        case RocketRendezvousState::IDLE:
            if (silent_for >= trigger_ms)
            {
                // #398: advance only when the retune actually happened.
                if (rendezvousHopToRendezvousMode())
                {
                    rendezvous_phase_start_ms = now;
                    rendezvous_state = RocketRendezvousState::ON_RENDEZVOUS;
                    ESP_LOGW("OC", "[RENDEZVOUS] Silent %u s; hop to rendezvous mode %.2f MHz SF%u BW%.0f",
                             (unsigned)(silent_for / 1000),
                             (double)LORA_FACTORY_RENDEZVOUS_MHZ,
                             (unsigned)LORA_FACTORY_RENDEZVOUS_SF,
                             (double)LORA_FACTORY_RENDEZVOUS_BW_KHZ);
                }
            }
            break;

        case RocketRendezvousState::ON_RENDEZVOUS:
            if ((now - rendezvous_phase_start_ms) >= RENDEZVOUS_WINDOW_MS)
            {
                if (rendezvousHopToSavedMode())
                {
                    rendezvous_phase_start_ms = now;
                    rendezvous_state = RocketRendezvousState::ON_SAVED;
                }
            }
            break;

        case RocketRendezvousState::ON_SAVED:
            if ((now - rendezvous_phase_start_ms) >= RENDEZVOUS_SAVED_MS)
            {
                if (rendezvousHopToRendezvousMode())
                {
                    rendezvous_phase_start_ms = now;
                    rendezvous_state = RocketRendezvousState::ON_RENDEZVOUS;
                }
            }
            break;
    }
}

// Hop-silence rendezvous fallback (#40/#41 phase 2b): while hopping, if the
// BS has gone quiet, briefly park on the rendezvous preset so a desynced BS
// can find us.  NOTE the BS must keep heartbeating IN FLIGHT for the QUIET
// trigger to hold (2026-07-16 bench).
static constexpr uint32_t HOP_FALLBACK_TRIGGER_INITIAL_MS = 30000;  // never heard BS yet
static constexpr uint32_t HOP_FALLBACK_TRIGGER_QUIET_MS   = 60000;  // BS seen, then silent
static constexpr uint32_t HOP_FALLBACK_VISIT_MS           = 3000;   // park 3 s on rendezvous

static void serviceHopFallback()
{
    if (!hop_active_) return;
    if (rendezvous_state != RocketRendezvousState::IDLE) return;  // slow_rendezvous owns the radio

    const uint32_t now = millis();

    switch (hop_fallback_state)
    {
        case HopFallbackState::NORMAL:
        {
            uint32_t ref;
            uint32_t trigger;
            if (hop_session_uplink_count == 0)
            {
                ref     = hop_active_entered_ms;
                trigger = HOP_FALLBACK_TRIGGER_INITIAL_MS;
            }
            else
            {
                ref     = last_uplink_rx_ms;
                trigger = HOP_FALLBACK_TRIGGER_QUIET_MS;
            }
            if ((now - ref) < trigger) return;

            // Trigger: visit the shared hardcoded rendezvous (#105).
            if (!lora_comms.reconfigure(LORA_FACTORY_RENDEZVOUS_MHZ,
                                         LORA_FACTORY_RENDEZVOUS_SF,
                                         LORA_FACTORY_RENDEZVOUS_BW_KHZ,
                                         LORA_FACTORY_RENDEZVOUS_CR,
                                         LORA_FACTORY_RENDEZVOUS_TX_DBM))
            {
                ESP_LOGE("OC", "[HOP] Visit failed: reconfigure to rendezvous mode");
                return;
            }
            (void)lora_comms.startReceive();
            lora_in_rx_mode = true;
            hop_fallback_phase_start_ms = now;
            hop_fallback_state = HopFallbackState::VISITING_RENDEZVOUS;
            ESP_LOGW("OC", "[HOP] Silence %u s — visiting rendezvous %.2f MHz for %u s",
                     (unsigned)((now - ref) / 1000),
                     (double)LORA_FACTORY_RENDEZVOUS_MHZ,
                     (unsigned)(HOP_FALLBACK_VISIT_MS / 1000));
            break;
        }
        case HopFallbackState::VISITING_RENDEZVOUS:
        {
            if ((now - hop_fallback_phase_start_ms) < HOP_FALLBACK_VISIT_MS) return;

            // Visit done: back to saved params, restart hopping with a fresh
            // bootstrap so the BS sees a clean transition packet.
            if (!lora_comms.reconfigure(lora_freq_mhz, lora_sf, lora_bw_khz,
                                         lora_cr, lora_tx_power))
            {
                ESP_LOGE("OC", "[HOP] Visit failed: reconfigure back to saved params");
                // Stay in VISITING_RENDEZVOUS; retry on next call.
                return;
            }
            (void)lora_comms.startReceive();
            lora_in_rx_mode = true;

            hop_bootstrap_left_ = currentHopDwell();
            if (hop_bootstrap_left_ == 0) hop_bootstrap_left_ = 1;
            hop_idx_          = 0;
            hop_needs_retune_ = false;  // already on lora_freq_mhz from reconfigure
            hop_fallback_state = HopFallbackState::NORMAL;
            hop_active_entered_ms = now;
            hop_session_uplink_count = 0;
            ESP_LOGI("OC", "[HOP] Visit done — resuming hop (%u bootstrap pkt(s))",
                     (unsigned)hop_bootstrap_left_);
            break;
        }
        case HopFallbackState::PAUSED_FOR_SCAN:
        {
            // Coordinated pause for BS scan (#90): re-bootstrap identical to
            // a fresh PRELAUNCH entry.  Signed delta handles millis() wrap.
            if ((int32_t)(now - hop_pause_until_ms) < 0) return;
            hop_bootstrap_left_      = currentHopDwell();
            if (hop_bootstrap_left_ == 0) hop_bootstrap_left_ = 1;
            hop_idx_                 = 0;
            hop_needs_retune_        = false;  // already on lora_freq_mhz
            hop_active_entered_ms    = now;
            hop_session_uplink_count = 0;
            hop_fallback_state       = HopFallbackState::NORMAL;
            hop_pause_until_ms       = 0;
            ESP_LOGI("OC", "[HOP] Pause done — resuming hop (%u bootstrap pkt(s))",
                     (unsigned)hop_bootstrap_left_);
            break;
        }
    }
}

// ==========================================================================
// SECTION: Diagnostics and periodic statistics (OC L4701-5438, trimmed)
// ==========================================================================
static void printLoRaPayloadDebug()
{
    if (!config::USE_LORA_RADIO)
    {
        return;
    }

    // Always FAST: a SLOW frame would leave most of the printed fields zero.
    uint8_t payload[SIZE_OF_LORA_BUDGET] = {0};
    size_t  payload_len = 0;
    if (!buildLoRaPayload(payload, lora_tx_seq, LORA_FRAME_FAST, payload_len))
    {
        return;
    }

    LoRaDataSI decoded = {};
    sensor_converter.unpackLoRaFastBytes(payload, decoded);
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

// #398: per-task CPU utilization sampler — on the mini this is the primary
// instrument for the merged one-chip flight+comms budget.  (OC L4746-4841)
#if (configUSE_TRACE_FACILITY == 1) && (configGENERATE_RUN_TIME_STATS == 1)
static void logTaskCpuDeltas(uint32_t dt_ms)
{
    if (!config::PROFILE_TASK_CPU) return;

    // Single call site (printStats on the comms loop) — statics keep ~3 KB
    // of snapshot off a stack that already runs deep into BLE sendTelemetry.
    static constexpr UBaseType_t MAXT = 48;
    struct Prev { TaskHandle_t h; uint32_t rt; };
    static Prev prev[MAXT];
    static UBaseType_t prev_n = 0;
    static bool primed = false;

    static TaskStatus_t now[MAXT];
    uint32_t total_run = 0;
    const UBaseType_t n = uxTaskGetSystemState(now, MAXT, &total_run);
    if (n == 0)
    {
        static bool warned = false;
        if (!warned) { warned = true; ESP_LOGW("TASKCPU", "task count > %u — profiler disabled", (unsigned)MAXT); }
        return;
    }

    struct Row { const char* name; UBaseType_t prio; int core; uint32_t d; };
    static Row rows[MAXT];
    UBaseType_t rown = 0;
    uint32_t idle1_d = 0;
    for (UBaseType_t i = 0; i < n; ++i)
    {
        const uint32_t cur = (uint32_t)now[i].ulRunTimeCounter;
        uint32_t before = cur;   // unseen task → 0 delta this round
        for (UBaseType_t j = 0; j < prev_n; ++j)
            if (prev[j].h == now[i].xHandle) { before = prev[j].rt; break; }
        const uint32_t d = cur - before;   // uint32 subtraction wraps correctly
        int core = -1;
        #if (configTASKLIST_INCLUDE_COREID == 1)
        core = (int)now[i].xCoreID;        // 0, 1, or tskNO_AFFINITY (-1)
        #endif
        rows[rown++] = { now[i].pcTaskName, now[i].uxCurrentPriority, core, d };
        if (now[i].pcTaskName && strcmp(now[i].pcTaskName, "IDLE1") == 0)
            idle1_d = d;
    }

    // Persist this snapshot for the next interval's delta.
    prev_n = (n < MAXT) ? n : MAXT;
    for (UBaseType_t i = 0; i < prev_n; ++i)
        prev[i] = { now[i].xHandle, (uint32_t)now[i].ulRunTimeCounter };

    if (!primed) { primed = true; return; }   // first call has no baseline

    const uint32_t win_us = dt_ms * 1000u;
    if (win_us == 0) return;

    // Partial selection sort: bring the top-K consumers to the front.
    const UBaseType_t K = (rown < 6) ? rown : 6;
    for (UBaseType_t a = 0; a < K; ++a)
    {
        UBaseType_t best = a;
        for (UBaseType_t b = a + 1; b < rown; ++b)
            if (rows[b].d > rows[best].d) best = b;
        if (best != a) { Row t = rows[a]; rows[a] = rows[best]; rows[best] = t; }
    }

    // Core-1 utilization = fraction of the interval IDLE1 did NOT run.
    const uint32_t idle1_pct = (idle1_d >= win_us) ? 100u : (idle1_d * 100u) / win_us;
    const uint32_t core1_util = 100u - idle1_pct;

    char line[256];
    int off = snprintf(line, sizeof(line), "win=%lums core1_util=%lu%% top:",
                       (unsigned long)dt_ms, (unsigned long)core1_util);
    for (UBaseType_t a = 0; a < K && off > 0 && off < (int)sizeof(line) - 1; ++a)
    {
        const uint32_t pct = (rows[a].d * 100u) / win_us;
        off += snprintf(line + off, sizeof(line) - off, " %s[c%d p%lu]=%lu%%",
                        rows[a].name ? rows[a].name : "?", rows[a].core,
                        (unsigned long)rows[a].prio, (unsigned long)pct);
    }
    ESP_LOGW("TASKCPU", "%s", line);
}
#else
static inline void logTaskCpuDeltas(uint32_t) {}   // stats facility not compiled in
#endif

// Stats interval state (OC file-scope equivalents, trimmed of I2S/I2C
// counters that have no source here).
static uint32_t last_stats_ms = 0;
static uint64_t prev_bytes_rx = 0;
static uint64_t prev_bytes_nand = 0;
static uint32_t prev_ring_overruns = 0;
static uint32_t prev_ring_drop_oldest_bytes = 0;
static uint32_t prev_ring_bad_sof_clears = 0;
static uint32_t interval_ring_fill_peak = 0;

static void printStats()
{
    const uint32_t now = millis();

    // #524: re-emit the last transfer's summary for a few minutes so a
    // BATTERY download can be read back over USB afterwards.
    if (s_xfer_summary[0] != '\0' && (int32_t)(s_xfer_reprint_until_ms - now) > 0 &&
        (int32_t)(now - s_xfer_next_reprint_ms) >= 0)
    {
        s_xfer_next_reprint_ms = now + XFER_REPRINT_EVERY_MS;
        ESP_LOGW("BLE", "%s", s_xfer_summary);
    }

    if ((now - last_stats_ms) < config::STATS_PERIOD_MS)
    {
        return;
    }
    const uint32_t dt = now - last_stats_ms;
    last_stats_ms = now;

    // --- Low-power mode: send minimal BLE telemetry only ---
    // MINI: gate on peripherals_initialized, not pwr_pin_on — during the
    // (long, GNSS-probing) power-on transition the rail is up but the logger
    // isn't, and this path must not touch it.
    if (!peripherals_initialized)
    {
        // Skip the INA230 poll while an OTA is in flight (#17): the
        // esp_ota_begin() partition erase blocks SPI flash and the gauge
        // transaction collides with it.
        if (!ble_app.isOtaActive())
        {
            readINA230Power();
        }

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
        ble_telem.vel_e = NAN;
        ble_telem.vel_n = NAN;
        ble_telem.vel_u = NAN;
        ble_telem.burnout_flag = false;
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
        // Boots land in low-power mode, so this is the path that runs right
        // after a post-OTA reboot + app reconnect — the critical place to
        // validate the new image (#8).
        if (ble_app.isConnected()) maybeMarkOtaValid();
        return;
    }

    // --- Active mode: full stats and telemetry ---
    TR_LogToFlashStats s = {};
    logger.getStats(s);

    // Flash-space stats for the app's storage bar (every ~3 s on a live link).
    if (ble_app.isConnected())
    {
        static uint32_t last_storage_stats_ms = 0;
        const uint32_t now_ss = millis();
        if (now_ss - last_storage_stats_ms >= 3000U)
        {
            last_storage_stats_ms = now_ss;
            RocketStorageStatsData rss = {};
            if (flightlog.isInitialized())
            {
                const auto& fcfg = flightlog.config();
                rss.flight_region_blocks = (uint16_t)(fcfg.flight_region_end - fcfg.flight_region_start);
                rss.used_blocks   = (uint16_t)flightlog.bitmap().countInState(tr_flightlog::BLOCK_ALLOCATED);
                rss.free_blocks   = (uint16_t)flightlog.bitmap().countInState(tr_flightlog::BLOCK_FREE);
                rss.bad_blocks    = (uint16_t)flightlog.bitmap().countInState(tr_flightlog::BLOCK_BAD);
                rss.system_blocks = (uint16_t)(fcfg.flight_region_start + 4u);  // LFS region + 4 metadata
                rss.flight_count  = (uint16_t)flightlog.index().size();
                // #671: runtime block size (128 on this part's GD5F1GQ5UE); the wire
        // struct is self-describing, the app scales by this field.
        rss.block_size_kb = (uint16_t)(flightlog.pageSize() * flightlog.pagesPerBlock() / 1024u);
                rss.flags         = RSS_FLAG_INITIALIZED;
                if (flightlog.autoEvictedCount() > 0)  // #315
                    rss.flags |= RSS_FLAG_AUTO_EVICTED;
            }
            ble_app.sendStorageStats(0xCC, reinterpret_cast<const uint8_t*>(&rss), sizeof(rss));
        }
    }

    // Capture GNSS timestamp for the active log file (when available)
    if (s.logging_active && latest_gnss_valid && latest_gnss_si.year > 2000)
    {
        uint16_t ts_year   = latest_gnss_si.year;
        uint8_t  ts_month  = latest_gnss_si.month;
        uint8_t  ts_day    = latest_gnss_si.day;
        uint8_t  ts_hour   = latest_gnss_si.hour;
        uint8_t  ts_minute = latest_gnss_si.minute;
        uint8_t  ts_second = latest_gnss_si.second;

        // Sim mode uses the hardcoded GNSS date (2025-01-01 12:00) —
        // substitute phone-synced time for unique filenames.
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
    const uint32_t ring_overrun_delta = s.ring_overruns - prev_ring_overruns;
    const uint32_t ring_drop_oldest_delta = s.ring_drop_oldest_bytes - prev_ring_drop_oldest_bytes;
    const uint32_t ring_bad_sof_delta = s.ring_bad_sof_clears - prev_ring_bad_sof_clears;
    prev_bytes_rx = s.bytes_received;
    prev_bytes_nand = s.bytes_written_nand;
    prev_ring_overruns = s.ring_overruns;
    prev_ring_drop_oldest_bytes = s.ring_drop_oldest_bytes;
    prev_ring_bad_sof_clears = s.ring_bad_sof_clears;

    const float rx_kbs = (dt > 0) ? ((float)rx_delta / (float)dt) : 0.0f;
    const float wr_kbs = (dt > 0) ? ((float)nand_delta / (float)dt) : 0.0f;

    // Persist a LogBufferStats snapshot once per stats interval into the
    // flight log for post-flight tooling (enqueueFrame drops it off-session).
    {
        LogBufferStatsData lbs = {};
        lbs.time_us = (uint32_t)esp_timer_get_time();
        lbs.ring_size = s.ring_size;
        lbs.ring_fill = s.ring_fill;
        lbs.ring_highwater = s.ring_highwater;
        lbs.ring_overruns = s.ring_overruns;
        lbs.ring_drop_oldest_bytes = s.ring_drop_oldest_bytes;
        lbs.ring_bad_sof_clears = s.ring_bad_sof_clears;

        uint8_t lbs_frame[MAX_FRAME];
        size_t  lbs_frame_len = 0;
        if (TR_I2C_Interface::packMessage(LOG_BUFFER_STATS_MSG,
                                           (const uint8_t*)&lbs,
                                           sizeof(lbs),
                                           lbs_frame, sizeof(lbs_frame),
                                           lbs_frame_len))
        {
            (void)logger.enqueueFrame(lbs_frame, lbs_frame_len);
        }
    }

    if (config::VERBOSE_DEBUG)
    {
    ESP_LOGI("OC", "RX %.1f KB/s | WR %.1f KB/s | frames rx/drop=%lu/%lu | RING=%lu/%lu (hi=%lu)",
                  (double)rx_kbs,
                  (double)wr_kbs,
                  (unsigned long)s.frames_received,
                  (unsigned long)s.frames_dropped,
                  (unsigned long)s.ring_fill,
                  (unsigned long)s.ring_size,
                  (unsigned long)s.ring_highwater);
    ESP_LOGI("OC", "RING interval peak/overrun/drop_oldest_bytes/bad_sof=%lu/%lu/%lu/%lu (bad_sof_total=%lu)",
                  (unsigned long)interval_ring_fill_peak,
                  (unsigned long)ring_overrun_delta,
                  (unsigned long)ring_drop_oldest_delta,
                  (unsigned long)ring_bad_sof_delta,
                  (unsigned long)s.ring_bad_sof_clears);
    ESP_LOGI("LOG", "logging=%c file=%s page=%lu block=%lu prog_fail=%lu erase_fail=%lu",
                  s.logging_active ? 'Y' : 'N',
                  logger.currentFilename(),
                  (unsigned long)s.nand_page,
                  (unsigned long)s.nand_block,
                  (unsigned long)s.nand_prog_fail,
                  (unsigned long)s.nand_erase_fail);
    if (config::USE_LORA_RADIO)
    {
        TR_LoRa_Comms::Stats ls = {};
        lora_comms.getStats(ls);
        // txmute= so a flat tx_ok reads as "muted on purpose", not "radio broken".
        ESP_LOGI("LORA", "LoRa en=%c txmute=%c tx_start/ok/fail=%lu/%lu/%lu local_ok/fail=%lu/%lu last_err=%d tx=%c",
                      ls.enabled ? 'Y' : 'N',
                      lora_tx_disabled ? 'Y' : 'N',
                      (unsigned long)ls.tx_started,
                      (unsigned long)ls.tx_ok,
                      (unsigned long)ls.tx_fail,
                      (unsigned long)lora_tx_ok,
                      (unsigned long)lora_tx_fail,
                      (int)ls.last_error,
                      ls.transmitting ? 'Y' : 'N');
        printLoRaPayloadDebug();
        // Compact LoRa uplink stats (kept from the OC's I2S block — the
        // counters are all radio-side and still live here).
        ESP_LOGI("LORA", "LoRa tx=%lu/%lu logged=%lu rx=%lu crc_fail=%lu low_snr=%lu isr=%lu uplink_rx=%lu ul_logged=%lu nid_drop=%lu rxmode=%c",
                      (unsigned long)ls.tx_ok,
                      (unsigned long)ls.tx_fail,
                      (unsigned long)lora_tx_logged,
                      (unsigned long)ls.rx_count,
                      (unsigned long)ls.rx_crc_fail,
                      (unsigned long)lora_low_snr_drops,
                      (unsigned long)ls.isr_count,
                      (unsigned long)lora_uplink_rx_count,
                      (unsigned long)lora_uplink_logged,
                      (unsigned long)lora_uplink_nid_drops,
                      ls.rx_mode ? 'Y' : 'N');
    }
    } // End VERBOSE_DEBUG

    // Always-on LFS/NAND stall instrumentation — peak duration of each
    // potentially-slow op since the last stats window.  MINI: the rx_ovf/
    // rx_peak/parser_max I2S-parser fields are gone with the link.
    ESP_LOGI("LOG TIMING",
             "write=%lu sync=%lu erase=%lu open=%lu close=%lu "
             "activate=%lu clr_ring=%lu iter=%lu us  syncs=%lu erases=%lu ring_peak=%lu "
             "bad_blocks=%lu skips=%lu  spiw=%lu spih=%lu us  "
             "dpg=%lu dby=%lu pop=%lu wsum=%lu m388=%lu/%lu us",
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
             (unsigned long)s.bad_block_skips,
             (unsigned long)s.spi_wait_max_us,   // #398: contention on the shared SPI3
             (unsigned long)s.spi_hold_max_us,   // #398
             (unsigned long)s.drain_pages,       // #510: window SUMS
             (unsigned long)s.drain_bytes,
             (unsigned long)s.pop_sum_us,
             (unsigned long)s.write_sum_us,      // #510
             (unsigned long)flightlog.writeLockWaitMaxUs(),   // #510/#388
             (unsigned long)flightlog.writeLockWaitSumUs());
    logger.resetIntervalTimings();
    flightlog.resetLockWaitStats();  // #510: same window as the logger sums

    // "LoRa off" reminder, at the DEFAULT log level.  The txmute= field on the
    // LoRa stats line above lives inside the VERBOSE_DEBUG block, which ships
    // false — so on a normal bench capture the only radio evidence is a tx
    // counter that never moves, which reads as a dead radio.  This says which
    // it is, once per stats interval, and costs nothing when transmitting.
    if (config::USE_LORA_RADIO && lora_tx_disabled)
    {
        ESP_LOGW("LORA", "transmit MUTED (\"LoRa off\") — no telemetry or beacon "
                         "going out; still listening for uplink on %.2f MHz",
                 (double)lora_comms.currentFrequencyMHz());
    }

    // #398: per-task CPU deltas over this same interval.
    logTaskCpuDeltas(dt);

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
    ble_telem.camera_recording = false;   // MINI: no camera fitted
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
    // #191: EKF ENU velocity + burnout for the app's ascent prediction.
    ble_telem.vel_e = (float)latest_non_sensor.e_vel / 100.0f;
    ble_telem.vel_n = (float)latest_non_sensor.n_vel / 100.0f;
    ble_telem.vel_u = (float)latest_non_sensor.u_vel / 100.0f;
    ble_telem.burnout_flag = nsFlagSet(latest_non_sensor.flags, NSF_BURNOUT);
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
    // Attitude quaternion from the flight side
    ble_telem.q0 = (float)latest_non_sensor.q0 / 10000.0f;
    ble_telem.q1 = (float)latest_non_sensor.q1 / 10000.0f;
    ble_telem.q2 = (float)latest_non_sensor.q2 / 10000.0f;
    ble_telem.q3 = (float)latest_non_sensor.q3 / 10000.0f;
    ble_telem.roll_cmd = (float)latest_non_sensor.roll_cmd / 100.0f;
    // Sensor health scorecard (#303) — direct-BLE path: flight-side bits plus
    // the two comms-owned nibbles (battery, storage).
    {
        uint32_t sh = latest_non_sensor.sensor_health;
        if (latest_power_valid) {
            POWERDataSI p = {};
            sensor_converter.convertPowerData(latest_power_raw, p);
            sh = shSet(sh, SH_BATT_SHIFT, shBatteryState(p.voltage));
        }
        sh = shSet(sh, SH_STORAGE_SHIFT, ocStorageHealth());  // #281/#278
        ble_telem.sensor_health = sh;
    }
    ble_telem.rssi = NAN;  // LoRa RSSI only meaningful on base station
    ble_telem.snr = NAN;
    ble_telem.bs_soc = NAN;      // No base station battery
    ble_telem.bs_voltage = NAN;
    ble_telem.bs_current = NAN;
    // Flight event flags
    ble_telem.launch_flag       = nsFlagSet(latest_non_sensor.flags, NSF_LAUNCH);
    ble_telem.vel_u_apogee_flag = nsFlagSet(latest_non_sensor.flags, NSF_VEL_APOGEE);
    ble_telem.alt_apogee_flag   = nsFlagSet(latest_non_sensor.flags, NSF_ALT_APOGEE);
    ble_telem.alt_landed_flag   = nsFlagSet(latest_non_sensor.flags, NSF_ALT_LANDED);
    ble_telem.sim_active        = nsFlagSet(latest_non_sensor.flags, NSF_SIM_ACTIVE);  // #393
    ble_telem.pwr_pin_on        = pwr_pin_on;
    // Pyro channel status from NonSensorData (shared armed bit mirrors the
    // live ARM pin; 4 per-channel cont/fired bits).
    ble_telem.pyro_armed = nsFlagSet(latest_non_sensor.flags, NSF_PYRO_ARMED);
    {
        const uint8_t ps = latest_non_sensor.pyro_status;
        ble_telem.pyro_cont[0]  = (ps & PSF_CH1_CONT)  != 0;
        ble_telem.pyro_fired[0] = (ps & PSF_CH1_FIRED) != 0;
        ble_telem.pyro_cont[1]  = (ps & PSF_CH2_CONT)  != 0;
        ble_telem.pyro_fired[1] = (ps & PSF_CH2_FIRED) != 0;
        ble_telem.pyro_cont[2]  = (ps & PSF_CH3_CONT)  != 0;
        ble_telem.pyro_fired[2] = (ps & PSF_CH3_FIRED) != 0;
        ble_telem.pyro_cont[3]  = (ps & PSF_CH4_CONT)  != 0;
        ble_telem.pyro_fired[3] = (ps & PSF_CH4_FIRED) != 0;
    }

    static uint32_t telem_send_count = 0;
    static uint32_t telem_skip_count = 0;
    bool connected = ble_app.isConnected();
    if (connected)
    {
        ble_app.sendTelemetry(ble_telem);
        telem_send_count++;
        maybeMarkOtaValid();   // validate a fresh OTA image once telemetry flows (#8)
    }
    else
    {
        telem_skip_count++;
    }
    if (config::VERBOSE_DEBUG)
    {
        if ((telem_send_count + telem_skip_count) % 10 == 0)
        {
            ESP_LOGI("BLE", "Telem: sent=%lu skip=%lu connected=%d",
                     (unsigned long)telem_send_count,
                     (unsigned long)telem_skip_count,
                     connected ? 1 : 0);
        }
        UBaseType_t hwm = uxTaskGetStackHighWaterMark(nullptr);
        ESP_LOGI("OC", "Stack HWM: %u bytes free",
                 (unsigned)(hwm * sizeof(StackType_t)));
    }

    interval_ring_fill_peak = s.ring_fill;
}

// ==========================================================================
// SECTION: Rail-backed bring-up (port of the OC's initPeripherals, L5445-5910)
// ==========================================================================
bool comms_setup_active()
{
    if (peripherals_initialized) return true;

    ESP_LOGI("PWR", "Initializing rail-backed comms peripherals...");

    // main.cpp has already run spi_bus_initialize(SPI3_HOST, ..., 4096) —
    // SPIClass::begin's own attempt returns ESP_ERR_INVALID_STATE and it
    // simply doesn't claim ownership (see mem_spi note above).
    mem_spi.begin(config::MEM_SPI_SCK, config::MEM_SPI_MISO, config::MEM_SPI_MOSI);
    delay(20);

    TR_LogToFlashConfig log_cfg = {};
    log_cfg.nand_cs = config::NAND_CS;
    log_cfg.spi_hz_nand = config::SPI_HZ_NAND;
    log_cfg.spi_mode_nand = config::SPI_MODE_NAND;
    log_cfg.ring_buffer_size = config::RAM_RING_SIZE;
    log_cfg.debug = config::DEBUG;
    // MINI: no MRAM fitted (board_v1.h MRAM_CS = -1) — the driver falls back
    // to the 128 KB heap ring above.  The OC's mram_size / spi_hz_mram /
    // dirty-marker fields are left at defaults (dirty_marker_addr 0 =
    // disabled); the MRAM crash-recovery drain (#274) has no source here and
    // is not ported.  Consequence: frames still in the ring at a brownout are
    // lost (up to ~1.3 s); the SNAPSHOT_MSG stream in the NAND log is the
    // mini's reboot-recovery source instead (see flight.cpp).
    log_cfg.mram_cs = config::MRAM_CS;
    log_cfg.dirty_marker_addr = 0;

    // --- LFS region + hot-path write sink (issue #50) ----------------------
    // LFS holds 32 blocks (4 MB at this part's 128 KB blocks); TR_FlightLog
    // owns the rest up to the chip's top four metadata blocks (#671: runtime
    // geometry — 1020-1023 on this 1024-block GD5F1GQ5UE).  Each
    // (page - 16)-byte drained chunk routes through flightlogWriteSink
    // → flightlog.writeFrame() → one NAND page.
    log_cfg.lfs_block_count = 32;
    log_cfg.write_sink = flightlogWriteSink;
    log_cfg.write_sink_ctx = &flightlog;
    log_cfg.flush_task_hook = flightlogFlushTaskHook;

    // Review fix (f7b8811 review, integration/comms-fidelity x3): the OC's
    // early-return on a dead NAND left peripherals_initialized false, which
    // on the mini parked the comms loop in its transition branch FOREVER
    // while the flight side — pyro state machine included — ran live, and
    // BLE told the operator the board was OFF. A dead NAND must degrade
    // (storage health BAD, no logging), never misreport a powered rocket as
    // dark. logger_ok gates every logger touchpoint below; radio and NVS
    // config proceed regardless.
    logger_ok = logger.begin(mem_spi, log_cfg);
    if (!logger_ok)
    {
        ESP_LOGE("PWR", "TR_LogToFlash begin failed — flight logging DEAD "
                        "this boot; storage health = BAD; continuing with "
                        "radio + BLE");
    }

    // --- TR_FlightLog begin (issue #50) -------------------------------------
    // #398: bitmap persists to NAND metadata blocks [2]/[3], not NVS — NVS
    // compaction disabled the flash cache and stalled core 1.
    flightlog_backend = tr_flightlog::TR_NandBackend_esp(&logger);
    if (logger_ok)
    {
        tr_flightlog::TR_FlightLog::Config fl_cfg{};
        // #671: this board's GD5F1GQ5UE has 1024 blocks of 128 KB — the old
        // compile-time defaults (region end 2044, metadata 2044-2047) were
        // OFF-DIE here, addressing blocks past the end of the chip. Region
        // and metadata now come from the RDID-resolved geometry: metadata is
        // the chip's top four blocks (1020-1023 on this part).
        const auto& ngeom = logger.nandGeometry();
        fl_cfg.flight_region_end = static_cast<uint16_t>(ngeom.block_count - 4);
        for (int i = 0; i < 4; ++i)
            fl_cfg.metadata_blocks[i] = static_cast<uint16_t>(ngeom.block_count - 4 + i);
        // #398/#492/#671: pre-allocate 80 blocks (~10 MB at this part's
        // 128 KB blocks) so most flights never extend mid-flight.
        fl_cfg.prealloc_blocks = 80;
        // #315: rolling-buffer auto-eviction, target ~10% of the flight
        // region free.  Destructive by design; surfaced via log + the
        // RSS_FLAG_AUTO_EVICTED bit, never silent.
        constexpr uint32_t kAutoEvictTargetFreePct = 10;
        fl_cfg.auto_evict_oldest = true;
        fl_cfg.auto_evict_target_free_blocks = static_cast<uint16_t>(
            static_cast<uint32_t>(fl_cfg.flight_region_end - fl_cfg.flight_region_start) *
            kAutoEvictTargetFreePct / 100u);
        flightlog_bitmap_store.bind(&flightlog_backend,
                                    fl_cfg.metadata_blocks[2],
                                    fl_cfg.metadata_blocks[3]);
        auto st = flightlog.begin(flightlog_backend,
                                  fl_cfg,
                                  &flightlog_bitmap_store);
        if (st == tr_flightlog::Status::Ok)
        {
            ESP_LOGI("FLIGHTLOG", "up: %zu flight(s) in index, %zu bad blocks",
                     flightlog.index().size(),
                     flightlog.bitmap().countInState(tr_flightlog::BLOCK_BAD));
        }
        else
        {
            // #566: deliberately non-fatal, but flight logging is DEAD this
            // boot; ocStorageHealth() reports SH_BAD so the scorecard goes
            // red instead of grey N/A.
            ESP_LOGE("FLIGHTLOG", "begin failed: %s — flight logging DEAD this "
                     "boot (all frames will drop); storage health = BAD",
                     tr_flightlog::to_string(st));
        }
    }

    // Start the NAND flush task on Core 0 — decouples NAND writes from this
    // loop so the RAM ring can buffer during stalls.
    if (logger_ok)
    {
        logger.startFlushTask(/* core */ 0, /* stackSize */ 8192, /* priority */ 1);

        TR_LogToFlashRecoveryInfo recovery = {};
        logger.getRecoveryInfo(recovery);
        if (recovery.recovered)
        {
            ESP_LOGI("LOG", "Startup recovery wrote %lu bytes to %s",
                          (unsigned long)recovery.recovered_bytes,
                          recovery.filename);
        }
    }

    vTaskDelay(1);  // feed watchdog after NAND init

    // --- NVS config loads --------------------------------------------------
    // MINI fix (report-oc-main-anatomy §9 note): the OC nested ALL of these
    // reads inside `if (config::USE_LORA_RADIO)` — an unrelated gate that
    // would silently skip config loading on a radio-less board.  Lifted out.
    {
        // Load saved LoRa config from NVS (write config.h defaults if empty)
        prefs.begin("lora", false);  // read-write
        if (!prefs.isKey("freq"))
        {
            prefs.putFloat("freq",  config::LORA_FREQ_MHZ);
            prefs.putUChar("sf",    config::LORA_SF);
            prefs.putFloat("bw",    config::LORA_BW_KHZ);
            prefs.putUChar("cr",    config::LORA_CR);
            prefs.putChar("txpwr",  config::LORA_TX_POWER_DBM);
            prefs.putUChar("hopdis", 1);  // #150: fixed mode is the default
            ESP_LOGI("CFG", "LoRa NVS empty -- wrote config.h defaults");
        }
        lora_freq_mhz = prefs.getFloat("freq", config::LORA_FREQ_MHZ);
        lora_sf        = prefs.getUChar("sf",   config::LORA_SF);
        lora_bw_khz    = prefs.getFloat("bw",   config::LORA_BW_KHZ);
        lora_cr        = prefs.getUChar("cr",   config::LORA_CR);
        lora_tx_power  = (int8_t)prefs.getChar("txpwr", config::LORA_TX_POWER_DBM);
        lora_hop_disabled = prefs.getUChar("hopdis", 1) != 0;  // #150: default fixed mode
        // "LoRa off" mute.  Deliberately NOT in the first-boot seeding block
        // above, and read whether or not "freq" exists: the mute is settable
        // over BLE before this ever runs, so seeding it would silently
        // un-mute the rocket on first power-up.  Absent = 0 = transmitting.
        lora_tx_disabled = prefs.getUChar("txdis", 0) != 0;

        // Issue #136: every boot starts on the hardcoded rendezvous preset
        // regardless of NVS; the BS moves us via the cmd-10 transactional
        // flow.  #150: lora_hop_disabled deliberately NOT overridden — the
        // link mode is user-selected and honors NVS across reboots.
        lora_freq_mhz     = LORA_FACTORY_RENDEZVOUS_MHZ;
        lora_sf           = LORA_FACTORY_RENDEZVOUS_SF;
        lora_bw_khz       = LORA_FACTORY_RENDEZVOUS_BW_KHZ;
        lora_cr           = LORA_FACTORY_RENDEZVOUS_CR;
        lora_tx_power     = LORA_FACTORY_RENDEZVOUS_TX_DBM;

        // Channel-set restore (#40/#41 phase 3): mask keyed off the BW it was
        // generated for.
        const uint8_t chset_n  = prefs.getUChar("chset_n", 0);
        const float   chset_bw = prefs.getFloat("chset_bw", 0.0f);
        if (chset_n > 0 && chset_bw == lora_bw_khz)
        {
            const size_t mask_bytes = (size_t)(chset_n + 7) / 8;
            size_t got = prefs.getBytes("chset_mask", skip_mask_, mask_bytes);
            if (got == mask_bytes)
            {
                skip_mask_n_        = chset_n;
                channel_set_bw_khz_ = chset_bw;
            }
        }
        prefs.end();
        ESP_LOGI("CFG", "LoRa NVS: %.1f MHz SF%u BW%.0f CR%u %d dBm",
                      (double)lora_freq_mhz, (unsigned)lora_sf,
                      (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);
        if (skip_mask_n_ > 0)
        {
            uint8_t active = 0;
            for (uint8_t i = 0; i < skip_mask_n_; i++)
                if (!loraSkipMaskTest(skip_mask_, i)) active++;
            ESP_LOGI("CFG", "[CHSET] NVS: %u/%u channels active",
                     (unsigned)active, (unsigned)skip_mask_n_);
        }
        else
        {
            ESP_LOGI("CFG", "[CHSET] NVS: no skip-mask");
        }

        // MINI: the OC's servo/pid/roll/guid/cam NVS loads are gone with the
        // hardware.  IMU orientation / rate / pyro survive.

        // Cached IMU mounting orientation.  MINI: the OC relied on the
        // status-query self-heal to re-push a MANUAL setting; there is no
        // status query here, so stage it to the flight side directly when
        // set (the flight side also persists its own copy).
        prefs.begin("orient", false);
        cfg_imu_orient = prefs.getUChar("set", cfg_imu_orient);
        prefs.end();
        ESP_LOGI("CFG", "NVS IMU orientation: %s",
                 cfg_imu_orient == IMU_ORIENT_AUTO
                     ? "AUTO" : orientCodeName(cfg_imu_orient));
        if (cfg_imu_orient != IMU_ORIENT_AUTO)
        {
            ImuOrientConfigData ocfg;
            ocfg.setting = cfg_imu_orient;
            mini_link::sendCommand(ORIENT_CONFIG_MSG,
                                   reinterpret_cast<const uint8_t*>(&ocfg),
                                   sizeof(ocfg));
        }

        // Cached IMU logging rate.  Readback/cache only — the flight side
        // owns application (its own NVS survives independently).
        prefs.begin("imurate", false);
        {
            const uint16_t nvs_rate = prefs.getUShort("hz", cfg_imu_rate);
            // Whitelist on read: a corrupted value must not be relayed or
            // echoed as if it were a real setting.
            if (imuRateSettingValid(nvs_rate)) cfg_imu_rate = nvs_rate;
            else ESP_LOGW("CFG", "NVS IMU logging rate %u invalid — keeping default",
                          (unsigned)nvs_rate);
        }
        prefs.end();
        if (imuRateIsDynamic(cfg_imu_rate))
            ESP_LOGI("CFG", "NVS IMU logging rate: DYNAMIC");
        else
            ESP_LOGI("CFG", "NVS IMU logging rate: %u Hz", (unsigned)cfg_imu_rate);

        // Load cached pyro config from NVS (4 channels)
        prefs.begin("pyro", true);
        size_t pyro_sz = prefs.getBytesLength("cfg");
        if (pyro_sz == sizeof(PyroConfigData)) {
            PyroConfigData pcfg;
            prefs.getBytes("cfg", &pcfg, sizeof(pcfg));
            const uint8_t en[4]   = { pcfg.ch1_enabled,      pcfg.ch2_enabled,
                                      pcfg.ch3_enabled,      pcfg.ch4_enabled };
            const uint8_t mode[4] = { pcfg.ch1_trigger_mode, pcfg.ch2_trigger_mode,
                                      pcfg.ch3_trigger_mode, pcfg.ch4_trigger_mode };
            const float   val[4]  = { pcfg.ch1_trigger_value, pcfg.ch2_trigger_value,
                                      pcfg.ch3_trigger_value, pcfg.ch4_trigger_value };
            for (int i = 0; i < 4; ++i) {
                cfg_pyro_enabled[i]       = en[i];
                cfg_pyro_trigger_mode[i]  = mode[i];
                cfg_pyro_trigger_value[i] = val[i];
            }
        }
        prefs.end();
        ESP_LOGI("CFG", "NVS Pyro: ch1=%u/%u/%.1f  ch2=%u/%u/%.1f  ch3=%u/%u/%.1f  ch4=%u/%u/%.1f",
                 cfg_pyro_enabled[0], cfg_pyro_trigger_mode[0], (double)cfg_pyro_trigger_value[0],
                 cfg_pyro_enabled[1], cfg_pyro_trigger_mode[1], (double)cfg_pyro_trigger_value[1],
                 cfg_pyro_enabled[2], cfg_pyro_trigger_mode[2], (double)cfg_pyro_trigger_value[2],
                 cfg_pyro_enabled[3], cfg_pyro_trigger_mode[3], (double)cfg_pyro_trigger_value[3]);
    }

    bool radio_ok = false;
    if (config::USE_LORA_RADIO)
    {
        // Direct-SPI LLCC68 only — the mini's board header pins
        // USE_UART_RADIO_MODEM=false and the modem backend isn't built.
        // Every parameter set explicitly; never rely on struct defaults
        // (the default (SF10, BW125) pair is LLCC68-illegal and silently
        // ignored — see TR_LoRa_Comms.cpp L101 note).
        TR_LoRa_Comms::Config lora_cfg = {};
        lora_cfg.enabled = config::USE_LORA_RADIO;
        lora_cfg.cs_pin = config::LORA_CS_PIN;
        lora_cfg.dio1_pin = config::LORA_DIO1_PIN;
        lora_cfg.rst_pin = config::LORA_RST_PIN;
        lora_cfg.busy_pin = config::LORA_BUSY_PIN;
        // MINI: unlike the V7 OC, the E220-900MM22S's RXEN reaches a GPIO and
        // MUST be driven (board_v1.h: the base station was bitten by a
        // floating RXEN in RX).  RadioLib drives it via setRfSwitchPins;
        // DIO2->TXEN is shorted on the module so TX switching is radio-owned.
        lora_cfg.rxen_pin = config::LORA_RXEN_PIN;
        lora_cfg.spi_sck = config::LORA_SPI_SCK;
        lora_cfg.spi_miso = config::LORA_SPI_MISO;
        lora_cfg.spi_mosi = config::LORA_SPI_MOSI;
        lora_cfg.spi_host = SPI3_HOST;  // shared with the NAND (board_v1.h)
        lora_cfg.freq_mhz = lora_freq_mhz;
        lora_cfg.spreading_factor = lora_sf;
        lora_cfg.bandwidth_khz = lora_bw_khz;
        lora_cfg.coding_rate = lora_cr;
        lora_cfg.preamble_len = config::LORA_PREAMBLE_LEN;
        lora_cfg.tx_power_dbm = lora_tx_power;
        lora_cfg.crc_on = config::LORA_CRC_ON;
        lora_cfg.rx_boosted_gain = config::LORA_RX_BOOSTED_GAIN;
        lora_cfg.syncword_private = config::LORA_SYNCWORD_PRIVATE;
        radio_ok = lora_direct_backend.begin(lora_cfg, config::DEBUG);
        if (!radio_ok)
        {
            ESP_LOGE("PWR", "LoRa init failed");
        }
    }

    vTaskDelay(1);  // feed watchdog after LoRa init

    peripherals_initialized = true;
    ESP_LOGI("PWR", "Rail-backed comms peripherals initialized.");
    return radio_ok || !config::USE_LORA_RADIO;
}

// ==========================================================================
// SECTION: Always-on bring-up (port of the OC's setup_oc NVS/BLE/INA half)
// ==========================================================================
void comms_setup_idle()
{
    // --- Load NVS settings early so config readback to app is correct ---
    // (OC L5998-6145; servo/pid/roll/guid early loads dropped with the
    // hardware.)
    {
        prefs.begin("lora", false);

        // NVS schema gate (#105 follow-up): stored != current → clear the
        // whole lora namespace and fall back to the factory rendezvous.
        {
            const uint8_t stored_v = prefs.getUChar("schemv", 0);
            if (stored_v != LORA_NVS_SCHEMA_VERSION)
            {
                ESP_LOGW("CFG", "LoRa NVS schema mismatch (stored=%u, current=%u) — clearing",
                         (unsigned)stored_v, (unsigned)LORA_NVS_SCHEMA_VERSION);
                prefs.clear();
                prefs.putUChar("schemv", LORA_NVS_SCHEMA_VERSION);
            }
        }

        if (prefs.isKey("freq"))
        {
            lora_freq_mhz = prefs.getFloat("freq", config::LORA_FREQ_MHZ);
            lora_sf        = prefs.getUChar("sf",   config::LORA_SF);
            lora_bw_khz    = prefs.getFloat("bw",   config::LORA_BW_KHZ);
            lora_cr        = prefs.getUChar("cr",    config::LORA_CR);
            lora_tx_power  = (int8_t)prefs.getChar("txpwr", config::LORA_TX_POWER_DBM);
            lora_hop_disabled = prefs.getUChar("hopdis", 1) != 0;  // #150
            ESP_LOGI("CFG", "NVS LoRa early load (cached): %.1f MHz SF%u BW%.0f CR%u %d dBm hop_disabled=%d",
                          (double)lora_freq_mhz, (unsigned)lora_sf,
                          (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power,
                          (int)lora_hop_disabled);
        }
        // Outside the isKey("freq") gate on purpose — see the twin in the
        // rail-backed bring-up: a board that has never powered its flight side
        // can hold a mute and no "freq" key at all, and the config readback the
        // app pulls before power-on must still tell the truth about the radio.
        lora_tx_disabled = prefs.getUChar("txdis", 0) != 0;
        if (lora_tx_disabled)
        {
            ESP_LOGW("CFG", "NVS LoRa: transmit MUTED (\"LoRa off\") — no telemetry "
                            "or beacon will go out until it is turned back on");
        }
        prefs.end();

        // Issue #136: boot-time rendezvous override, here too so anything
        // between this early load and comms_setup_active sees rendezvous
        // values rather than stale NVS.
        lora_freq_mhz     = LORA_FACTORY_RENDEZVOUS_MHZ;
        lora_sf           = LORA_FACTORY_RENDEZVOUS_SF;
        lora_bw_khz       = LORA_FACTORY_RENDEZVOUS_BW_KHZ;
        lora_cr           = LORA_FACTORY_RENDEZVOUS_CR;
        lora_tx_power     = LORA_FACTORY_RENDEZVOUS_TX_DBM;

        // Early identity load (so config readback on first connect is correct)
        uint8_t mac[6];
        esp_efuse_mac_get_default(mac);
        snprintf(unit_id_hex, sizeof(unit_id_hex), "%02x%02x%02x%02x",
                 mac[2], mac[3], mac[4], mac[5]);

        prefs.begin("identity", false);

        // #150: identity versions SEPARATELY from lora and MIGRATES instead
        // of wiping — a wipe caused the #133-era nid regression.  v0→v1
        // resets a stale masked nid to the default ONCE; `un` and `rid` are
        // preserved.  (Full rationale at OC L6081-6115.)
        {
            const uint8_t stored_v = prefs.getUChar("schemv", 0);
            if (stored_v != IDENTITY_NVS_SCHEMA_VERSION)
            {
                if (stored_v == 0)
                {
                    const uint8_t old_nid =
                        prefs.getUChar("nid", config::DEFAULT_NETWORK_ID);
                    if (old_nid != config::DEFAULT_NETWORK_ID)
                    {
                        ESP_LOGW("CFG", "Identity migration: stale masked nid=%u reset to default %u",
                                 (unsigned)old_nid, (unsigned)config::DEFAULT_NETWORK_ID);
                        prefs.putUChar("nid", config::DEFAULT_NETWORK_ID);
                    }
                }
                ESP_LOGW("CFG", "Identity NVS schema %u -> %u (migrated)",
                         (unsigned)stored_v, (unsigned)IDENTITY_NVS_SCHEMA_VERSION);
                prefs.putUChar("schemv", IDENTITY_NVS_SCHEMA_VERSION);
            }
        }

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

    // --- INA230 power monitor (always-on I2C bus, not behind PWR_PIN) ---
    // MINI: the bus itself (shared with the rail-backed IIS2MDC) is created
    // by main.cpp; begin() only adds the INA device.  (OC L6147-6189)
    if (shared_i2c_bus != nullptr && ina230.begin(shared_i2c_bus, 400000) == TR_INA230_OK)
    {
        // Start in power-down mode (0.5 uA) — triggered reads at 100 Hz.
        ina230.setConfiguration(INA230_Avg::AVG_1,
                                INA230_ConvTime::CT_332us,
                                INA230_ConvTime::CT_332us,
                                INA230_Mode::POWER_DOWN);
        // #297: gate ina230_ok on a successful calibrate so a bad calibration
        // can't leave us reading a flat 0 A with an "OK" status.
        if (ina230.calibrate(INA230_R_SHUNT_OHM, INA230_CURRENT_LSB_A) == TR_INA230_OK)
        {
            ina230.enableConversionReadyAlert(true);  // CVRF bit for polling
            ina230_ok = true;
            ESP_LOGI("PWR", "INA230 OK (triggered mode, CVRF polling)");
        }
        else
        {
            ESP_LOGW("PWR", "INA230 calibrate failed — battery monitoring disabled");
        }
    }
    else
    {
        ESP_LOGW("PWR", "INA230 not found -- battery monitoring disabled");
    }

    // #519: comms owns the connection-parameter policy — slow while the rail
    // is off, fast once it comes on.  Take ownership instead of racing the
    // component's own connect-time request.
    ble_app.setAutoConnParams(false);

    // Only BLE starts at boot — everything else is behind PWR_PIN.
    // (retry32kCrystal() already ran in app_main, BEFORE this — the BT
    // controller samples the RTC slow clock once at init. #541)
    if (!ble_app.begin())
    {
        ESP_LOGE("BLE", "BLE app interface failed to start");
    }
    // MINI: no setOtaRelayDelegate — there is no second MCU.  An OTA_BEGIN
    // with target=1 gets a clean "bad_target" from TR_BLE_To_APP itself.

    last_stats_ms = millis();
}

// ==========================================================================
// SECTION: power-off log finalization (called by main.cpp's power machine)
// ==========================================================================
// The OC could drop its FC rail with logging machinery still alive — its
// NAND was always-on.  On the mini the NAND is ON the switched rail, so the
// rail must not drop until the flush task has drained the ring and the
// deferred finalize has committed the index.  Returns false (power-off must
// be REFUSED) if that doesn't complete in time.
bool comms_prepare_power_off()
{
    if (!peripherals_initialized) return true;   // nothing rail-backed is live

    if (logger.isLoggingActive())
    {
        // Mirror the BLE cmd 23 stop path.
        logger.endLogging();
        flightlogEndFlight();
    }
    else if (flightlog.isFlightActive())
    {
        // A flight was prepared (PRELAUNCH pre-create) but never logged or
        // never closed — finalize it now so the index isn't left with a
        // dangling active flight for brownout recovery to chew on.
        flightlogEndFlight();
    }

    // Bound the wait: ring drain + deferred prepare/finalize on the flush
    // task.  10 s covers the worst case (~770 ms erase + full-ring drain +
    // index save) with a wide margin.
    const uint32_t deadline = millis() + 10000;
    for (;;)
    {
        bool finalize_pending;
        portENTER_CRITICAL(&g_finalize_mux);
        finalize_pending = g_finalize_pending;
        portEXIT_CRITICAL(&g_finalize_mux);

        if (!logger.isLoggingActive() && !finalize_pending &&
            !flightlog.isFlightActive())
        {
            return true;
        }
        if ((int32_t)(deadline - millis()) <= 0)
        {
            ESP_LOGE("PWR", "power-off: log finalization did not complete in "
                            "10 s (logging=%d finalize_pending=%d flight_active=%d)"
                            " — REFUSING rail drop",
                     (int)logger.isLoggingActive(), (int)finalize_pending,
                     (int)flightlog.isFlightActive());
            return false;
        }
        delay(50);
    }
}

// ==========================================================================
// SECTION: Main comms loop (port of loop_oc, OC L6225-7717)
// ==========================================================================
// Loop-stall instrumentation (#90 follow-up).  (OC L6228-6259)
static constexpr int64_t LOOP_STALL_THRESHOLD_US = 100'000;  // 100 ms

// Idle (rail-off / low-power) loop period.  20 ms is for RESPONSIVENESS —
// one BLE command drains per pass (#221/#517 history at OC L6240-6250).
static constexpr uint32_t IDLE_LOOP_DELAY_MS = 20;

#define LOOP_STALL_INSTR(name, expr) do {                                       \
    const int64_t _stall_t0_ = esp_timer_get_time();                            \
    expr;                                                                       \
    const int64_t _stall_dt_ = esp_timer_get_time() - _stall_t0_;               \
    if (_stall_dt_ > LOOP_STALL_THRESHOLD_US) {                                 \
        ESP_LOGW("LOOP_STALL", "%s took %lld us", (name), (long long)_stall_dt_); \
    }                                                                           \
} while (0)

// MINI: unsupported-command reply.  There is no NACK on the BLE command
// characteristic, so "cleanly" means: say so loudly, forward nothing, cache
// nothing — never pretend (spec rule).  The dropped hardware is servo /
// camera / piezo / guidance / control stack.
static void unsupportedBleCmd(uint8_t cmd, const char* what)
{
    ESP_LOGW("BLE", "Cmd %u (%s) unsupported on rocket-computer-mini "
                    "(hardware absent)", (unsigned)cmd, what);
}

// MINI: IDLE-mode LED slow blink (spec).  GPIO45 is a strapping pin — safe
// to drive only after boot, which every call site here is.  Short duty (50
// ms per 2 s) keeps the idle-current contract; solid on in ACTIVE mode.
static void serviceLed()
{
    static bool led_inited = false;
    if (!led_inited)
    {
        led_inited = true;
        gpio_set_direction((gpio_num_t)config::LED_PIN, GPIO_MODE_OUTPUT);
    }
    if (peripherals_initialized)
    {
        gpio_set_level((gpio_num_t)config::LED_PIN, 1);
        return;
    }
    const uint32_t phase = millis() % 2000U;
    gpio_set_level((gpio_num_t)config::LED_PIN, (phase < 50U) ? 1 : 0);
}

static void comms_loop()
{
    const int64_t _loop_t0 = esp_timer_get_time();

    // Preemption catch: wall time spent OUTSIDE this function (exit→entry) —
    // the catch-all below is structurally blind to a higher-priority core-0
    // task holding the core between iterations.  (OC L6267-6289)
    static int64_t _loop_last_exit_us = 0;
    if (_loop_last_exit_us != 0) {
        const int64_t _preempt_dt = _loop_t0 - _loop_last_exit_us;
        if (_preempt_dt > LOOP_STALL_THRESHOLD_US) {
            ESP_LOGW("LOOP_STALL",
                     "comms_loop off-core %lld us between iterations (preempted)",
                     (long long)_preempt_dt);
        }
    }

    // #398 item 3: drain one paced config-readback frame per pass.  Outside
    // the mode gate so connect-time readback drains in idle too.
    serviceConfigReadbackQueue();

    serviceLed();

    // --- Active mode: rail-backed peripherals live ---
    if (peripherals_initialized)
    {
        // MINI: in-process telemetry ingest replaces serviceI2CIngress /
        // the I2S parser.  Runs the NON_SENSOR state edges (hop machine,
        // PRELAUNCH pre-create, LANDED end-flight) before the LoRa services
        // below consume the state — same ordering the OC had (parser task
        // preempted loop_oc).
        serviceTelemFromFlight();

        // Launch-triggered logging: start when NSF_LAUNCH appears.  (OC
        // L6366-6404, verbatim incl. the #317 landed lockout.)
        {
            static bool prev_ns_launch = false;
            // #317: once the vehicle has reported LANDED, refuse to start a
            // new flight log until a reboot; cleared only by a sim re-arm
            // (leaving LANDED is impossible in a real flight).
            static bool oc_landed_lockout = false;
            static RocketState prev_rs_lockout = INITIALIZATION;
            if (latest_rocket_state == LANDED)
            {
                oc_landed_lockout = true;
            }
            else if (prev_rs_lockout == LANDED)
            {
                // #317 sim re-arm.
                oc_landed_lockout = false;
            }
            prev_rs_lockout = latest_rocket_state;
            const bool ns_launch = latest_non_sensor_valid &&
                                   nsFlagSet(latest_non_sensor.flags, NSF_LAUNCH);
            if (ns_launch && !prev_ns_launch && !oc_landed_lockout && logger_ok)
            {
                // Mirror the cmd 23 lifecycle; each call is a no-op when the
                // matching state is already set.
                logger.prepareLogFile();
                flightlogBeginFlight();
                logger.startLogging();
                ESP_LOGI("OC", "Launch detected - logging started");
            }
            prev_ns_launch = ns_launch;
        }
        if (logger_ok)
        {
            LOOP_STALL_INSTR("logger.service", logger.service());
        }

        // Read power data at ~100 Hz; log POWER_MSG only while logging.
        {
            static uint32_t last_pwr_log_ms = 0;
            uint32_t now_ms = millis();
            if ((now_ms - last_pwr_log_ms) >= 10) {
                last_pwr_log_ms = now_ms;
                LOOP_STALL_INSTR("readINA230Power", readINA230Power());
                if (latest_power_valid && logger.isLoggingActive()) {
                    uint8_t pwr_frame[MAX_FRAME];
                    size_t  pwr_frame_len = 0;
                    if (TR_I2C_Interface::packMessage(POWER_MSG,
                                                       (const uint8_t*)&latest_power_raw,
                                                       sizeof(POWERData),
                                                       pwr_frame, sizeof(pwr_frame),
                                                       pwr_frame_len)) {
                        LOOP_STALL_INSTR("logger.enqueueFrame(pwr)",
                                         logger.enqueueFrame(pwr_frame, pwr_frame_len));
                    }
                }
            }
        }

        LOOP_STALL_INSTR("serviceLoRa", serviceLoRa());

        // Check for LoRa uplink commands between TX cycles
        LOOP_STALL_INSTR("serviceLoRaUplink", serviceLoRaUplink());

        // Slow-rendezvous cycle (issue #71).
        LOOP_STALL_INSTR("serviceRocketRendezvous", serviceRocketRendezvous());

        // #150: deferred hop-enable — activate once the BS's mirror-retry
        // train has finished.
        if (hop_enable_apply_at_ms != 0 &&
            (int32_t)(millis() - hop_enable_apply_at_ms) >= 0)
        {
            hop_enable_apply_at_ms = 0;
            updateHopFromState(latest_rocket_state);
        }

        // Hop-silence rendezvous (#40/#41 phase 2b).
        LOOP_STALL_INSTR("serviceHopFallback", serviceHopFallback());

        // Send name beacon so base station can identify us
        LOOP_STALL_INSTR("sendLoRaBeacon", sendLoRaBeacon());
    }
    else if (!pwr_pin_on)
    {
        // --- Low-power mode: only BLE active ---
        // INA230 in continuous averaging (1024 samples ≈ 680 ms window);
        // read the latest averaged result once per second.  (OC L6458-6489)
        // Gated on !pwr_pin_on (not just !peripherals_initialized) so the
        // power-on transition window can't flip the INA back to continuous
        // under the post-activation restore.
        {
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
                if (ina230_ok) {
                    float bus_v = 0.0f, current_a = 0.0f;
                    const bool read_ok =
                        ina230.readBusVoltage_V(&bus_v) == TR_INA230_OK &&
                        ina230.readCurrent_A(&current_a) == TR_INA230_OK;
                    // Same validity + SOC + commit policy as the triggered
                    // path (#272); continuous mode has no CVRF to check.
                    commitPowerSample(read_ok ? bus_v : 0.0f, read_ok ? current_a : 0.0f);
                }
            }
        }

        // Responsiveness pacing, not power (#221/#517 — see constant above).
        delay(IDLE_LOOP_DELAY_MS);
    }
    else
    {
        // Power-on transition in progress (rail commanded up, rail-backed
        // init not finished — comms_setup_active/flight_setup running on the
        // main task).  Keep BLE serviced, touch nothing rail-backed.
        delay(IDLE_LOOP_DELAY_MS);
    }

    // MINI: post-activation hook — the tail of the OC's cmd-8 ON handler
    // (L7070-7086, L7161-7168), which ran inline there.  Here activation
    // completes on the main task, so fire once on the edge.
    {
        static bool post_activation_done = false;
        if (peripherals_initialized && !post_activation_done)
        {
            post_activation_done = true;
            // Restore INA230 to fast single-shot config (idle mode set
            // AVG_1024, which makes triggered reads take ~680 ms).
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
                requestFastBLEParams();
                // NVS config is freshly loaded — resend config readback so
                // the app gets the actual persisted values.
                delay(100);  // let peripherals finish settling
                sendCurrentConfig();
            }
            ESP_LOGI("BLE", "Power rail toggled: ON");
        }
    }

    // Auto-send config once the BLE connection is fully ready (#224 — MTU
    // negotiated AND notifications enabled; isReadyForNotify is the gate).
    // (OC L6504-6546)
    {
        static bool ble_was_connected = false;
        static bool config_push_pending = false;
        static uint32_t slow_params_due_ms = 0;   // #542: deferred idle-param drop
        bool ble_now = ble_app.isConnected();
        if (ble_now && !ble_was_connected) config_push_pending = true;    // arm on connect
        if (!ble_now) {                                                   // disarm on disconnect
            config_push_pending = false;
            slow_params_due_ms = 0;
        }
        if (config_push_pending && ble_app.isReadyForNotify()) {
            config_push_pending = false;
            sendCurrentConfig();
            // #542: in low-power mode, SCHEDULE the slow params drop past
            // the app's connect burst instead of requesting here.
            if (!pwr_pin_on) {
                slow_params_due_ms = (uint32_t)millis() + kSlowParamsDeferMs;
            }
        }
        // #542: fire the deferred drop once the burst window has passed;
        // rail-on or disconnect cancels.
        if (slow_params_due_ms != 0 && ble_now) {
            if (pwr_pin_on) {
                slow_params_due_ms = 0;
            } else if ((int32_t)((uint32_t)millis() - slow_params_due_ms) >= 0) {
                slow_params_due_ms = 0;
                requestSlowBLEParams();
            }
        }
        ble_was_connected = ble_now;
    }

    // MINI: the OC's fc_identity / imu_orient / guid_target dirty-flag
    // re-publish blocks (L6548-6575) are dropped with their sources.

    // Service the BLE library's poll-style work — drives the deferred OTA
    // restart.  LOAD-BEARING: handleOtaFinish() sets the boot partition and
    // schedules esp_restart(), but the reboot only fires from this poll.
    //
    // #1176: an OTA reboot is a DELIBERATE restart, so retire the ACTIVE flag
    // before it fires. Recovery eligibility no longer consults the reset
    // reason, so without this the post-update boot would look exactly like a
    // session that ended unexpectedly and could restore an old interrupted
    // flight on the bench. Cheap and idempotent; a flight in progress is
    // already refused an OTA elsewhere.
    if (ble_app.otaRestartDue())
    {
        mini_link::retireActiveFlag("OTA restart");
    }
    ble_app.loop();

    // ======================================================================
    // BLE command dispatch (OC L6598-7702)
    // ======================================================================
    uint8_t ble_cmd = ble_app.getCommand();
    if (ble_cmd != 0)
    {
        ESP_LOGI("OC_CMD", "BLE cmd=%u", (unsigned)ble_cmd);
        if (ble_cmd == 1)
        {
            unsupportedBleCmd(ble_cmd, "camera toggle");
        }
        else if (ble_cmd == 2)
        {
            // Send file list with pagination (5 files per page).
            beginPhoneIO();
            uint8_t page = ble_app.getFileListPage();
            String json = flightlogBuildFileListJson(page).c_str();
            ble_app.sendFileList(json);
            endPhoneIO();
            ESP_LOGI("BLE", "Sent file list page %u: %u bytes",
                     page, (unsigned)json.length());
        }
        else if (ble_cmd == 23)
        {
            // Toggle logging (manual start/stop from app)
            if (!logger_ok)
            {
                ESP_LOGE("OC_CMD", "Logging toggle refused: NAND logger dead "
                                   "this boot (storage health BAD)");
            }
            else if (logger.isLoggingActive())
            {
                logger.endLogging();
                flightlogEndFlight();
                ESP_LOGI("OC_CMD", "Logging STOPPED (manual)");
            }
            else
            {
                logger.prepareLogFile();
                flightlogBeginFlight();
                logger.startLogging();
                ESP_LOGI("OC_CMD", "Logging STARTED (manual)");
            }
        }
        else if (ble_cmd == 24)
        {
            unsupportedBleCmd(ble_cmd, "servo test angles");
        }
        else if (ble_cmd == 25)
        {
            unsupportedBleCmd(ble_cmd, "servo test stop");
        }
        else if (ble_cmd == 3)
        {
            // Delete file, then return the refreshed page-0 listing.
            String filename = ble_app.getDeleteFilename();
            // #383: never while flying (INFLIGHT-only gate by design).
            if (filename.length() > 0 && latest_rocket_state == INFLIGHT)
            {
                ESP_LOGW("BLE", "Delete '%s' refused: rocket INFLIGHT",
                         filename.c_str());
                String json = flightlogBuildFileListJson(/*page=*/0).c_str();
                ble_app.sendFileList(json);
            }
            else if (filename.length() > 0)
            {
                beginPhoneIO();
                auto st = flightlog.deleteFlight(filename.c_str());
                bool success = (st == tr_flightlog::Status::Ok);
                String json = flightlogBuildFileListJson(/*page=*/0).c_str();
                ESP_LOGI("BLE", "Delete '%s': %s", filename.c_str(),
                         success ? "OK" : "FAIL");
                ble_app.sendFileList(json);
                endPhoneIO();
            }
        }

        // Handle file download requests from BLE app
        String download_filename = ble_app.getDownloadFilename();
        // #383/#526: refuse INFLIGHT with EOF|ABORT (a refusal is NOT a
        // complete zero-length file).
        if (download_filename.length() > 0 && latest_rocket_state == INFLIGHT)
        {
            ESP_LOGW("BLE", "Download '%s' refused: rocket INFLIGHT",
                     download_filename.c_str());
            (void)ble_app.sendFileChunk(0, nullptr, 0, true, /*abort=*/true);
            download_filename = "";
        }
        if (download_filename.length() > 0)
        {
            beginPhoneIO();  // PM locks + download-in-progress marker
            ESP_LOGI("BLE", "Download file request: %s", download_filename.c_str());

            // #524: read RSSI on the QUIET link, before load (see the OC's
            // HCI_Read_RSSI trust notes — believe the app if they disagree).
            ESP_LOGI("BLE", "Link before transfer: %lu ms effective, rssi=%d dBm "
                            "(cross-check against the app's own RSSI — if these disagree, "
                            "believe the app)",
                     (unsigned long)ble_app.effectiveEventMs(), (int)ble_app.connRssi());

            // The link must be the fast one before we stream a single byte.
            ensureFastLinkForTransfer();

            // Dynamic chunk size based on negotiated MTU.
            const size_t chunk_data_size = ble_app.getMaxChunkDataSize();
            ESP_LOGI("BLE", "Chunk data size: %u  (link %lu ms effective)",
                     (unsigned)chunk_data_size, (unsigned long)ble_app.effectiveEventMs());

            if (chunk_data_size == 0)
            {
                ESP_LOGE("BLE", "chunk data size is 0, aborting download");
                // #526: a failure, not a completed empty file.
                (void)ble_app.sendFileChunk(0, nullptr, 0, true, /*abort=*/true);
            }
            else
            {

            // Frame-aligned BLE transfer: pack complete binary frames into
            // each notification so a dropped notification only loses whole
            // frames.  Frame: [AA][55][AA][55][type][len][payload][CRC16],
            // max 263 B.  (OC L6744-6759)
            const size_t MAX_FRAME_SIZE = 263;
            const size_t FLASH_READ_SIZE = 4096;
            static uint8_t read_buf[FLASH_READ_SIZE + MAX_FRAME_SIZE];
            static uint8_t ble_buf[502];  // Notification payload (max MTU data)

            uint32_t file_offset = 0;     // Current position in flash file
            size_t carryover = 0;         // Bytes carried from previous flash read
            size_t ble_used = 0;          // Bytes accumulated in ble_buf
            uint32_t bytes_sent = 0;      // Total BLE bytes sent
            uint32_t frames_sent = 0;
            uint32_t start_ms = millis();
            bool eof = false;

            // #524 diagnostic: split the wall clock between flash and BLE.
            uint64_t flash_us = 0;
            uint64_t ble_us   = 0;
            ble_app.resetXferStats();

            // #524: no per-chunk delay — sendFileChunk applies real
            // backpressure and returns false only on genuine failure (the
            // old fixed 30 ms delay was treating the symptom; full history
            // at OC L6777-6795).
            bool send_failed = false;

            while (!eof)
            {
                // Read next block from flash, appended after any carryover
                size_t flash_bytes_read = 0;
                const uint32_t t_flash = micros();
                bool read_ok = flightlogReadChunk(download_filename.c_str(), file_offset,
                                                  read_buf + carryover, FLASH_READ_SIZE,
                                                  flash_bytes_read, eof);
                flash_us += (uint32_t)(micros() - t_flash);
                if (!read_ok)
                {
                    // #558: funnel through the unified post-loop abort
                    // handler so the redundant EOF carries the abort bit too.
                    ESP_LOGE("BLE", "File read error at offset %lu, aborting download",
                             (unsigned long)file_offset);
                    send_failed = true;
                    break;
                }
                file_offset += flash_bytes_read;

                size_t buf_len = carryover + flash_bytes_read;
                size_t pos = 0;

                // Scan for complete frames
                while (pos + 8 <= buf_len)  // Min frame: SOF(4)+type(1)+len(1)+CRC(2)
                {
                    if (read_buf[pos]   != 0xAA || read_buf[pos+1] != 0x55 ||
                        read_buf[pos+2] != 0xAA || read_buf[pos+3] != 0x55)
                    {
                        pos++;
                        continue;
                    }

                    uint8_t payload_len = read_buf[pos + 5];
                    size_t frame_size = 4 + 1 + 1 + payload_len + 2;

                    if (pos + frame_size > buf_len)
                    {
                        break;  // Incomplete frame — carry over to next read
                    }

                    // Complete frame — flush BLE buffer if it won't fit
                    if (ble_used > 0 && ble_used + frame_size > chunk_data_size)
                    {
                        const uint32_t t_ble = micros();
                        const bool sent = ble_app.sendFileChunk(bytes_sent, ble_buf, ble_used, false);
                        ble_us += (uint32_t)(micros() - t_ble);
                        if (!sent)
                        {
                            // #524: could not send even after full
                            // backpressure — do NOT hand the app a file with
                            // a hole in it.
                            ESP_LOGE("BLE", "BLE send failed mid-transfer, aborting download");
                            send_failed = true;
                            break;
                        }
                        bytes_sent += ble_used;
                        ble_used = 0;
                    }

                    // Append frame to BLE buffer
                    memcpy(ble_buf + ble_used, read_buf + pos, frame_size);
                    ble_used += frame_size;
                    frames_sent++;
                    pos += frame_size;
                }

                if (send_failed) break;   // #524: abort, don't limp on

                // Move unparsed bytes to start of buffer for next iteration
                carryover = buf_len - pos;
                if (carryover > 0 && pos > 0)
                {
                    memmove(read_buf, read_buf + pos, carryover);
                }
            }

            if (send_failed)
            {
                // #526: EOF|ABORT, not a bare EOF — the app must REJECT the
                // partial file.  #558: covers flash read errors too.
                ESP_LOGE("BLE", "Download ABORTED after %lu bytes",
                         (unsigned long)bytes_sent);
                (void)ble_app.sendFileChunk(bytes_sent, nullptr, 0, true, /*abort=*/true);
            }
            // Send remaining data with EOF flag
            else if (ble_used > 0)
            {
                (void)ble_app.sendFileChunk(bytes_sent, ble_buf, ble_used, true);
                bytes_sent += ble_used;
            }
            else
            {
                (void)ble_app.sendFileChunk(bytes_sent, nullptr, 0, true);
            }

            // Redundant EOF in case the last notification was dropped —
            // #526: MUST carry the same abort bit.
            delay(50);
            (void)ble_app.sendFileChunk(bytes_sent, nullptr, 0, true, /*abort=*/send_failed);

            uint32_t elapsed_ms = millis() - start_ms;
            float kbps = (elapsed_ms > 0) ? (bytes_sent / 1024.0f) / (elapsed_ms / 1000.0f) : 0;
            ESP_LOGI("BLE", "Download complete: %lu frames, %lu bytes in %.1fs (%.1f KB/s)",
                          (unsigned long)frames_sent, (unsigned long)bytes_sent,
                          elapsed_ms / 1000.0f, kbps);

            // #524 diagnostic — where did the time go?  (Interpretation
            // guide at OC L6919-6937; judge the link by rssi MAX, not avg.)
            const auto xs = ble_app.xferStats();
            const float total_ms = (elapsed_ms > 0) ? (float)elapsed_ms : 1.0f;
            const int rssi_avg = (xs.rssi_n > 0) ? (int)(xs.rssi_sum / (int32_t)xs.rssi_n) : 0;
            snprintf(s_xfer_summary, sizeof(s_xfer_summary),
                     "XFER: %.1f KB/s  flash=%llu ms (%.0f%%)  ble=%llu ms (%.0f%%)  |  "
                     "chunks=%lu  waits=%lu (max %lu)  |  queue depth=%lu chunks  |  "
                     "link=%lu ms pm=%d  rssi avg=%d min=%d max=%d dBm (n=%lu)",
                     kbps,
                     (unsigned long long)(flash_us / 1000),
                     100.0f * (flash_us / 1000.0f) / total_ms,
                     (unsigned long long)(ble_us / 1000),
                     100.0f * (ble_us / 1000.0f) / total_ms,
                     (unsigned long)xs.chunks, (unsigned long)xs.retries_total,
                     (unsigned long)xs.retries_max, (unsigned long)xs.burst_max,
                     (unsigned long)ble_app.effectiveEventMs(),
                     s_phone_io_pm_held ? 1 : 0,
                     rssi_avg, (int)xs.rssi_min, (int)xs.rssi_max,
                     (unsigned long)xs.rssi_n);
            ESP_LOGW("BLE", "%s", s_xfer_summary);

            if (xs.rssi_n > 0 && xs.rssi_max < -85)
            {
                ESP_LOGW("BLE", "  ^ link genuinely WEAK (best sample %d dBm). The LL will "
                                "be retransmitting, and every retransmit costs one of the "
                                "~4 packet slots per connection event that the transfer "
                                "rate is made of.", (int)xs.rssi_max);
            }

            // Keep saying it, so a battery run can be read back over USB.
            s_xfer_reprint_until_ms = millis() + XFER_REPRINT_WINDOW_MS;
            s_xfer_next_reprint_ms  = millis() + XFER_REPRINT_EVERY_MS;
            } // else (chunk_data_size > 0)
            endPhoneIO();

            // #524 follow-up: mirror the connect-edge policy after the
            // transfer — rail off -> slow link (covers the abort paths too).
            if (!pwr_pin_on && ble_app.isConnected())
            {
                requestSlowBLEParams();
            }
        }

        // Flight simulator commands → flight side via mini_link (chain B —
        // a fresh if, as in the OC; the code sets are disjoint).
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
                mini_link::sendCommand(SIM_CONFIG_MSG,
                                       reinterpret_cast<const uint8_t*>(&sim_cfg),
                                       sizeof(sim_cfg));
                ESP_LOGI("OC", "SIM Config queued: mass=%.0fg thrust=%.1fN burn=%.1fs descent=%.1fm/s",
                              (double)mass_g, (double)sim_cfg.thrust_n,
                              (double)sim_cfg.burn_time_s, (double)sim_cfg.descent_rate_mps);
            }
        }
        else if (ble_cmd == 6)
        {
            mini_link::sendCommand(SIM_START_CMD, nullptr, 0);
            ESP_LOGI("OC", "SIM Start queued for flight side");
        }
        else if (ble_cmd == 7)
        {
            logger.endLogging();
            flightlogEndFlight();
            mini_link::sendCommand(SIM_STOP_CMD, nullptr, 0);
            ESP_LOGI("OC", "SIM Stop queued for flight side (logging ended)");
        }
        else if (ble_cmd == 15)
        {
            mini_link::sendCommand(GROUND_TEST_START, nullptr, 0);
            ESP_LOGI("OC", "GROUND TEST Start queued for flight side");
        }
        else if (ble_cmd == 16)
        {
            mini_link::sendCommand(GROUND_TEST_STOP, nullptr, 0);
            ESP_LOGI("OC", "GROUND TEST Stop queued for flight side");
        }
        else if (ble_cmd == 8)
        {
            // Toggle power rail.  MINI: the transition itself runs on the
            // main task (16 KB stack for the synchronous sensor bring-up in
            // flight_setup) — this only requests it, so BLE stays serviced
            // through the long GNSS-probing init.  See main.cpp.
            if (!pwr_pin_on)
            {
                ESP_LOGI("BLE", "Power rail ON requested");
                miniRequestPowerOn();
            }
            else
            {
                // Power off = finalize + rail down + esp_restart back into
                // IDLE (#9 reset-instead-of-teardown policy, adapted).  The
                // request is refused by main.cpp while
                // !flightSafeToPowerOff() (PRELAUNCH/INFLIGHT, pyro busy,
                // mag cal in progress) or while log finalization is pending.
                ESP_LOGI("BLE", "Power rail OFF requested");
                miniRequestPowerOff();
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
            // Direct LoRa reconfig over BLE is not accepted on the rocket
            // side (#106) — LoRa link parameters are owned by the base
            // station.  Send a config readback so the app's UI snaps back.
            ESP_LOGW("BLE", "Cmd 10 refused on rocket: LoRa params are BS-controlled (#106). Send to base station instead.");
            sendCurrentConfig();
        }
        else if (ble_cmd == 11)
        {
            unsupportedBleCmd(ble_cmd, "sounds enable/disable");  // no piezo
        }
        else if (ble_cmd == 12)
        {
            unsupportedBleCmd(ble_cmd, "servo config");
        }
        else if (ble_cmd == 13)
        {
            unsupportedBleCmd(ble_cmd, "PID config");
        }
        else if (ble_cmd == 14)
        {
            unsupportedBleCmd(ble_cmd, "servo control enable");
        }
        else if (ble_cmd == 20)
        {
            // Config readback request
            sendCurrentConfig();
        }
        else if (ble_cmd == 21)
        {
            mini_link::sendCommand(GYRO_CAL_CMD, nullptr, 0);
            ESP_LOGI("BLE", "Gyro cal request -> flight side");
        }
        else if (ble_cmd == 22)
        {
            unsupportedBleCmd(ble_cmd, "gain schedule");
        }
        else if (ble_cmd == 26)
        {
            unsupportedBleCmd(ble_cmd, "roll profile set");
        }
        else if (ble_cmd == 27)
        {
            unsupportedBleCmd(ble_cmd, "roll profile clear");
        }
        else if (ble_cmd == 28)
        {
            unsupportedBleCmd(ble_cmd, "guidance point");  // no guidance stack
        }
        else if (ble_cmd == 29)
        {
            unsupportedBleCmd(ble_cmd, "servo replay");
        }
        else if (ble_cmd == 30)
        {
            unsupportedBleCmd(ble_cmd, "servo replay stop");
        }
        else if (ble_cmd == 31)
        {
            unsupportedBleCmd(ble_cmd, "roll control config");
        }
        else if (ble_cmd == 32)
        {
            unsupportedBleCmd(ble_cmd, "guidance enable");
        }
        else if (ble_cmd == 65)
        {
            unsupportedBleCmd(ble_cmd, "guidance config");
        }
        else if (ble_cmd == 66)
        {
            unsupportedBleCmd(ble_cmd, "fin layout");
        }
        else if (ble_cmd == 33)
        {
            unsupportedBleCmd(ble_cmd, "camera type");
        }
        else if (ble_cmd == 64)
        {
            // IMU mounting orientation: [setting:1] — IMU_ORIENT_AUTO or a
            // TR_Orientation code (0..23).
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1 &&
                (payload[0] == IMU_ORIENT_AUTO || payload[0] < ORIENT_CODE_COUNT))
            {
                cfg_imu_orient = payload[0];
                // Was stageImuOrientConfig() → I2C two-phase; now the same
                // ORIENT_CONFIG_MSG frame straight onto the command queue.
                ImuOrientConfigData ocfg;
                ocfg.setting = cfg_imu_orient;
                mini_link::sendCommand(ORIENT_CONFIG_MSG,
                                       reinterpret_cast<const uint8_t*>(&ocfg),
                                       sizeof(ocfg));
                Preferences p2;
                p2.begin("orient", false);
                p2.putUChar("set", cfg_imu_orient);
                p2.end();
                ESP_LOGI("BLE", "IMU orientation setting: %s",
                         cfg_imu_orient == IMU_ORIENT_AUTO
                             ? "AUTO" : orientCodeName(cfg_imu_orient));
            }
        }
        else if (ble_cmd == 67)
        {
            // IMU logging rate: [rate_hz:2 LE] — IMU_RATE_DYNAMIC (0) or a
            // whitelisted ISM6HG256 ODR (960/1920/3840).
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 2)
            {
                uint16_t rate_hz;
                memcpy(&rate_hz, payload, sizeof(rate_hz));
                if (imuRateSettingValid(rate_hz))
                {
                    cfg_imu_rate = rate_hz;
                    ImuRateConfigData rcfg;
                    rcfg.rate_hz = cfg_imu_rate;
                    mini_link::sendCommand(IMU_RATE_CONFIG_MSG,
                                           reinterpret_cast<const uint8_t*>(&rcfg),
                                           sizeof(rcfg));
                    Preferences p2;
                    p2.begin("imurate", false);
                    p2.putUShort("hz", cfg_imu_rate);
                    p2.end();
                    if (imuRateIsDynamic(rate_hz))
                    {
                        ESP_LOGI("BLE", "IMU logging rate: DYNAMIC -> flight side");
                    }
                    else
                    {
                        ESP_LOGI("BLE", "IMU logging rate: %u Hz -> flight side",
                                 (unsigned)cfg_imu_rate);
                    }
                }
                else
                {
                    ESP_LOGW("BLE", "IMU logging rate %u Hz rejected "
                                    "(not dynamic/960/1920/3840)",
                             (unsigned)rate_hz);
                }
            }
        }
        else if (ble_cmd == LORA_CMD_SET_TX_DISABLED)
        {
            // "LoRa off": [disabled:1] — 1 mutes every LoRa transmit, 0
            // resumes.  Same constant (and number) as the uplink command, so
            // the app's toggle means one thing whether it reaches the rocket
            // directly or relayed through the base station.
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1)
            {
                (void)applyLoRaTxMute(payload[0] != 0, "BLE");
            }
        }
        else if (ble_cmd == 34)
        {
            // Pyro config: 4 × {enabled:1, mode:1, value:4f} = 24 bytes
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(PyroConfigData)) {
                PyroConfigData pcfg;
                memcpy(&pcfg, payload, sizeof(pcfg));
                const uint8_t en[4]   = { pcfg.ch1_enabled,      pcfg.ch2_enabled,
                                          pcfg.ch3_enabled,      pcfg.ch4_enabled };
                const uint8_t mode[4] = { pcfg.ch1_trigger_mode, pcfg.ch2_trigger_mode,
                                          pcfg.ch3_trigger_mode, pcfg.ch4_trigger_mode };
                const float   val[4]  = { pcfg.ch1_trigger_value, pcfg.ch2_trigger_value,
                                          pcfg.ch3_trigger_value, pcfg.ch4_trigger_value };
                for (int i = 0; i < 4; ++i) {
                    cfg_pyro_enabled[i]       = en[i];
                    cfg_pyro_trigger_mode[i]  = mode[i];
                    cfg_pyro_trigger_value[i] = val[i];
                }
                // Queue for the flight side (was: FC via I2C)
                mini_link::sendCommand(PYRO_CONFIG_MSG,
                                       reinterpret_cast<const uint8_t*>(&pcfg),
                                       sizeof(pcfg));
                // Persist to NVS
                Preferences p2;
                p2.begin("pyro", false);
                size_t written = p2.putBytes("cfg", &pcfg, sizeof(pcfg));
                p2.end();
                ESP_LOGI("BLE", "Pyro config: ch1=%u/%u/%.1f  ch2=%u/%u/%.1f  ch3=%u/%u/%.1f  ch4=%u/%u/%.1f (NVS wrote %u bytes)",
                         pcfg.ch1_enabled, pcfg.ch1_trigger_mode, (double)pcfg.ch1_trigger_value,
                         pcfg.ch2_enabled, pcfg.ch2_trigger_mode, (double)pcfg.ch2_trigger_value,
                         pcfg.ch3_enabled, pcfg.ch3_trigger_mode, (double)pcfg.ch3_trigger_value,
                         pcfg.ch4_enabled, pcfg.ch4_trigger_mode, (double)pcfg.ch4_trigger_value,
                         (unsigned)written);
            } else {
                ESP_LOGW("BLE", "Pyro config: payload too short (%u < %u)",
                         (unsigned)plen, (unsigned)sizeof(PyroConfigData));
            }
        }
        else if (ble_cmd == 35)
        {
            // Pyro continuity test — 1 byte payload: channel (1..4).  The OC
            // jumped these to the FRONT of its FC queue; mini_link's queue
            // drains whole every flight tick (~1 ms), so ordering is moot.
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            uint8_t ch = (plen >= 1) ? payload[0] : 0;
            if (ch < 1 || ch > 4) {
                ESP_LOGW("BLE", "Pyro continuity test: invalid channel %u", ch);
            } else if (!mini_link::commandShutterOpen()) {
                // Shutter closed (rail off / power transition): sendCommand
                // would drop this silently — unlike the OC's queue the mini
                // never HOLDS a pyro command, so there's no latent-fire
                // hazard here, but the app deserves the same 0xCE refusal
                // the OC sends instead of a dead button.
                ble_app.sendPyroTestRefusal(35, ch, 1 /* rail off */);
                ESP_LOGW("BLE", "Pyro continuity test CH%u refused: shutter closed", ch);
            } else {
                mini_link::sendCommand(PYRO_CONT_TEST, &ch, 1);
                ESP_LOGI("BLE", "Pyro continuity test CH%u", ch);
            }
        }
        else if (ble_cmd == 36)
        {
            // Pyro test fire — 1 byte payload: channel (1..4)
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            uint8_t ch = (plen >= 1) ? payload[0] : 0;
            if (ch < 1 || ch > 4) {
                ESP_LOGW("BLE", "Pyro test fire: invalid channel %u", ch);
            } else if (!mini_link::commandShutterOpen()) {
                // Shutter closed: same 0xCE refusal as the continuity branch
                // above (and as the OC) so the app's abort flow works
                // identically against both boards.
                ble_app.sendPyroTestRefusal(36, ch, 1 /* rail off */);
                ESP_LOGW("BLE", "Pyro test fire CH%u refused: shutter closed", ch);
            } else {
                mini_link::sendCommand(PYRO_FIRE_TEST, &ch, 1);
                ESP_LOGI("BLE", "Pyro test fire CH%u", ch);
            }
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
                Preferences p2;
                p2.begin("identity", false);
                p2.putBytes("un", new_name, strlen(new_name) + 1);
                p2.end();
                // Update BLE advertising name
                ble_app.setName(unit_name);
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
                Preferences p2;
                p2.begin("identity", false);
                p2.putUChar("nid", network_id);
                p2.end();
                sendCurrentConfig();
                ESP_LOGI("BLE", "Network ID set: %u", (unsigned)network_id);
            }
        }
        else if (ble_cmd == 42)
        {
            // Set rocket_id — payload: [rid:1], validated 1..254
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1 && payload[0] > 0 && payload[0] < 255)
            {
                rocket_id = payload[0];
                Preferences p2;
                p2.begin("identity", false);
                p2.putUChar("rid", rocket_id);
                p2.end();
                sendCurrentConfig();
                ESP_LOGI("BLE", "Rocket ID set: %u", (unsigned)rocket_id);
            }
        }
        // ---- Magnetometer hard-iron calibration (issue #96) ----
        // Thin pass-throughs: the flight side owns all the state-machine +
        // sphere-fit logic (was setPendingCommand → I2C).
        else if (ble_cmd == 50)
        {
            mini_link::sendCommand(MAG_CAL_START, nullptr, 0);
            ESP_LOGI("BLE", "Mag cal START -> flight side");
        }
        else if (ble_cmd == 51)
        {
            mini_link::sendCommand(MAG_CAL_ABORT, nullptr, 0);
            ESP_LOGI("BLE", "Mag cal ABORT -> flight side");
        }
        else if (ble_cmd == 52)
        {
            mini_link::sendCommand(MAG_CAL_ACCEPT, nullptr, 0);
            ESP_LOGI("BLE", "Mag cal ACCEPT -> flight side");
        }
        else if (ble_cmd == 53)
        {
            mini_link::sendCommand(MAG_CAL_RETRY, nullptr, 0);
            ESP_LOGI("BLE", "Mag cal RETRY -> flight side");
        }
        else if (ble_cmd == 54)
        {
            mini_link::sendCommand(MAG_CAL_COMPUTE_FIT, nullptr, 0);
            ESP_LOGI("BLE", "Mag cal COMPUTE_FIT -> flight side");
        }
        // #148 — user-driven verify completion / reset.
        else if (ble_cmd == 56)
        {
            mini_link::sendCommand(MAG_CAL_VERIFY_DONE, nullptr, 0);
            ESP_LOGI("BLE", "Mag cal VERIFY_DONE -> flight side");
        }
        else if (ble_cmd == 57)
        {
            mini_link::sendCommand(MAG_CAL_VERIFY_RESET, nullptr, 0);
            ESP_LOGI("BLE", "Mag cal VERIFY_RESET -> flight side");
        }
        // #148 — user-override save.
        else if (ble_cmd == 58)
        {
            mini_link::sendCommand(MAG_CAL_FORCE_APPLY, nullptr, 0);
            ESP_LOGI("BLE", "Mag cal FORCE_APPLY -> flight side");
        }
        // Issue #132 — app pushes a saved cal back as part of the
        // rocket-profile auto-sync on connect.
        else if (ble_cmd == 55)
        {
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(MagCalApplyData))
            {
                mini_link::sendCommand(MAG_CAL_APPLY_MSG, payload,
                                       sizeof(MagCalApplyData));
                ESP_LOGI("BLE", "Mag cal APPLY queued for flight side");
            }
            else
            {
                ESP_LOGW("BLE", "Mag cal APPLY: payload too short (%u < %u)",
                              (unsigned)plen, (unsigned)sizeof(MagCalApplyData));
            }
        }
        // #132 profile-cal READ + sensor APPLY/READ (61/62/63 — renumbered
        // from 56/57/58, which the #148 verify commands own; must stay in
        // sync with BLEDevice.swift).
        else if (ble_cmd == 61)
        {
            mini_link::sendCommand(MAG_CAL_READ, nullptr, 0);
            ESP_LOGI("BLE", "Mag cal READ -> flight side");
        }
        else if (ble_cmd == 62)
        {
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(SensorCalApplyData))
            {
                mini_link::sendCommand(SENSOR_CAL_APPLY_MSG, payload,
                                       sizeof(SensorCalApplyData));
                ESP_LOGI("BLE", "Sensor cal APPLY queued for flight side");
            }
            else
            {
                ESP_LOGW("BLE", "Sensor cal APPLY: payload too short (%u < %u)",
                              (unsigned)plen, (unsigned)sizeof(SensorCalApplyData));
            }
        }
        else if (ble_cmd == 63)
        {
            mini_link::sendCommand(SENSOR_CAL_READ, nullptr, 0);
            ESP_LOGI("BLE", "Sensor cal READ -> flight side");
        }
    }

    LOOP_STALL_INSTR("printStats", printStats());

    // Catch-all: any iteration whose total wall time exceeds the threshold.
    const int64_t _loop_dt = esp_timer_get_time() - _loop_t0;
    if (_loop_dt > LOOP_STALL_THRESHOLD_US) {
        ESP_LOGW("LOOP_STALL", "comms_loop iteration took %lld us (catch-all)",
                 (long long)_loop_dt);
    }

    _loop_last_exit_us = esp_timer_get_time();
    vTaskDelay(1);  // yield to FreeRTOS scheduler
}

void commsTask(void* /*arg*/)
{
    for (;;)
    {
        comms_loop();
    }
}
