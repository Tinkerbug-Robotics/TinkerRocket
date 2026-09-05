// ==========================================================================
// SECTION: Includes, board selection, and compile-time configuration
// ==========================================================================
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
#include <esp_efuse.h>            // #822: read PSRAM_CAP to verify the board flag
#include <esp_efuse_table.h>      // #822: ESP_EFUSE_PSRAM_CAP
#include <esp_app_desc.h>         // esp_app_get_description for firmware version readback (#8)
#include <esp_ota_ops.h>          // esp_ota_mark_app_valid_cancel_rollback (#8)
#include <esp_partition.h>        // esp_ota_get_running_partition for rollback gate (#8)
#include <TR_OTA_Receiver.h>      // TR_OTA_Receiver::Error — decode the FC's relayed OTA error
#include "soc/rtc_cntl_reg.h"  // Brownout detector control
#include "soc/rtc.h"            // #541: rtc_clk_* for the 32k-crystal second chance
#include "esp_private/esp_clk.h"  // #541: esp_clk_slowclk_cal_set after a late 32k start
#include <esp_timer.h>          // #541: boot USB-enumeration light-sleep grace timer
#include "freertos/queue.h"
#include "freertos/semphr.h"    // oc_i2s_mutex for the Phase 4 Layer 3 I2S flip
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
#include "ota_relay_policy.h"   // #834 items 6/7: I2S relay recovery timing
#include "rail_restore_policy.h"  // #825: boot rail re-assert decision
#include <esp_attr.h>             // RTC_NOINIT_ATTR
#include <esp_system.h>           // esp_reset_reason
#include <esp_adc/adc_oneshot.h>  // #850: camera/servo IMON reads
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include "dedup_reboot_policy.h"
#include "cmd_queue_dedupe_policy.h"  // #837 item 11: pyro tests key on channel

#include <TR_I2C_Interface.h>
#include <TR_I2S_Stream.h>
#include <TR_LogToFlash.h>
#include <TR_LoRa_Comms.h>
#include <IRadioLink.h>
#include <LoRaDirectBackend.h>
#include <UartModemBackend.h>
#include <TR_Sensor_Data_Converter.h>
#include <TR_Orientation.h>
#include <TR_Coordinates.h>
#include <TR_BLE_To_APP.h>
#include <RocketComputerTypes.h>
#include <TR_INA230.h>
#include <TR_FlightLog.h>
#include <SnapshotTailScan.h>   // #846: boot re-seed of the snapshot cache
#include <TR_NandBackend_esp.h>
#include <NvsBitmapStore.h>
#include <NandBitmapStore.h>   // #398: bitmap on NAND (not NVS) — no cache-disable stall
#include <WireFormat.h>
// FlightSimulator.h removed — sim now runs on FlightComputer via TR_Sensor_Collector_Sim

// ==========================================================================
// SECTION: OTA boot validation and rollback guard
// ==========================================================================
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
        ESP_LOGW("OC", "OTA: new image validated, rollback cancelled");
    }
    else
    {
        ESP_LOGE("OC", "OTA: mark_app_valid_cancel_rollback failed: %s",
                 esp_err_to_name(err));
    }
    g_ota_pending_verify = false;
}

// ==========================================================================
// SECTION: Peripheral objects and flight-log storage
// ==========================================================================
static TR_I2C_Interface i2c_interface(config::I2C_ADDRESS);
static bool i2c_slave_initialized = false;
static TR_I2S_Stream i2s_stream;
static TR_LogToFlash logger;

// TR_FlightLog (issue #50) owns the flight-log hot path — append-only NAND
// writes via writeFrame(), with dual-copy index metadata in blocks 1020-1023
// and a persistent 3-state bitmap in NVS. TR_LogToFlash keeps the ring/flush
// machinery and shelled-down 4 MB LFS partition for config; a write_sink
// fn-pointer routes each drained page-sized chunk into flightlog.writeFrame().
static tr_flightlog::TR_NandBackend_esp flightlog_backend;
static tr_flightlog::TR_FlightLog flightlog;
static tr_flightlog::NandBitmapStore flightlog_bitmap_store;  // #398: NAND-backed, bound in initPeripherals

// Phone time sync (BLE Command 9) — used for flight-filename timestamps
// so each flight gets a unique filename instead of the hardcoded
// GNSS sentinel date (2025-01-01 12:00). Defined here (earlier than the
// original site) because flightlogEndFlight below reads these fields.
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

// #822: read the silicon rather than trusting the board flag.
//
// esp_chip_info() is no help on this part — esp_hw_support's S3 port hardcodes
// features to WIFI_BGN|BLE and never sets CHIP_FEATURE_EMB_PSRAM — but eFuse
// BLK1 carries PSRAM_CAP, and it reads the same whether or not CONFIG_SPIRAM
// was compiled in. So even a V7/V8 image, which links no PSRAM support at all,
// can still say what silicon it is sitting on.
//
// Returns in-package PSRAM size in MB, 0 for none, or -1 if unreadable.
static int s3PsramCapMb()
{
    uint8_t cap = 0;
    if (esp_efuse_read_field_blob(ESP_EFUSE_PSRAM_CAP, &cap,
                                  ESP_EFUSE_PSRAM_CAP[0]->bit_count) != ESP_OK)
    {
        return -1;
    }
    // esp_efuse_table.csv: {0: None, 1: 8M, 2: 2M, 3: 16M}. The CSV also lists
    // 4 = 4M, which a 2-bit field cannot encode; treat anything unexpected as
    // unknown rather than inventing a size.
    switch (cap)
    {
        case 0: return 0;
        case 1: return 8;
        case 2: return 2;   // the RH2
        case 3: return 16;
        default: return -1;
    }
}

// #281/#278: classify flight-log storage for the #303 scorecard.  A full or
// write-failing NAND silently dropped the 2026-06-25 guided flight (recovery
// surfaced the previous day's data); folding a verdict into sensor_health makes
// it visible on the pre-launch go/no-go and the live downlink instead.
static SensorHealthState ocStorageHealth()
{
    // #566: an uninitialized flight log is the MOST severe storage state, not
    // an inapplicable one. flightlog.begin() failing at boot (corrupt index /
    // metadata read error) is deliberately non-fatal, so the OC runs — but
    // flightlogWriteSink() then refuses every frame and the #271 drop path
    // discards ALL flight data. Returning SH_NA here hid exactly the silent
    // loss this fold-in exists to surface: the app hides the Storage row on
    // NA and excludes it from the go/no-go, so the operator saw a green
    // board. There is no logger-disabled OC build (begin() is unconditional),
    // so NA is never legitimate once boot completes — report BAD and let the
    // scorecard go red.
    if (!flightlog.isInitialized()) return SH_BAD;
    TR_LogToFlashStats s = {};
    logger.getStats(s);
    const uint32_t free_blocks = flightlog.bitmap().countInState(tr_flightlog::BLOCK_FREE);
    const uint32_t prealloc    = flightlog.config().prealloc_blocks;
    SensorHealthState st = shStorageState(free_blocks, prealloc, s.nand_prog_fail);
    // The flight index is a second, independent capacity limit (#281): once it's
    // full, finalize can't record the flight even with free blocks. Fold it in.
    const size_t used = flightlog.index().size();
    const size_t cap  = tr_flightlog::FlightIndex::MAX_ENTRIES;
    if (used >= cap) st = SH_BAD;
    else if (used + 4 >= cap && st == SH_OK) st = SH_DEGRADED;
    // #826: a board configured for an MRAM that did not answer the boot probe.
    // DEGRADED rather than BAD — frames still reach the NAND, so nothing is
    // lost, but the ring shrank to internal RAM and brownout recovery is gone
    // for the session. Folded in here for the same reason as #566 above: this
    // is precisely the state that used to read green while the part was dead.
    // Never fires on a board that correctly has no MRAM (MRAM_CS < 0).
    if (logger.mramProbeFailed() && st == SH_OK) st = SH_DEGRADED;
    return st;
}

// Stage 3b (issue #50): BLE file-ops re-backed on TR_FlightLog.
// Fills up to `max_bytes` of the downloader's scratch buffer by issuing
// successive readFlightPage calls (each returns at most one page's payload
// bytes — the portion of a single NAND page past its PageHeader). Matches
// the logger.readFileChunk contract: sets eof=true when the last byte of
// the flight has been read; returns false only on underlying NAND I/O error.
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

// Builds the cmd 2 / cmd 3 response JSON. Pulls all index entries, reverses
// to newest-first (flight_id is monotonic, so index order is oldest-first),
// paginates at FILES_PER_PAGE, and encodes via the byte-stable
// wire_format::encodeFileListJson helper (golden-fixture tested in Stage 1).
// Returns the JSON string; caller passes it to ble_app.sendFileList.
static std::string flightlogBuildFileListJson(uint8_t page)
{
    static constexpr size_t FILES_PER_PAGE = 5;

    static tr_flightlog::FlightIndexEntry entries[
        tr_flightlog::FlightIndex::MAX_ENTRIES];
    const size_t total = flightlog.listFlights(
        entries, tr_flightlog::FlightIndex::MAX_ENTRIES,
        /*page=*/0, /*per_page=*/tr_flightlog::FlightIndex::MAX_ENTRIES);

    // Reverse for newest-first display order (matches legacy qsort behavior).
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
    // (issue #77). Running it inline here used to block whatever Core 1
    // task was on the call stack — loop_oc, the BLE/LoRa cmd 23 handler,
    // or the I2S/I2C parse path — which starved the parser task and
    // produced multi-hundred-ms gaps in the recorded sensor stream. The
    // actual prepareFlight runs from flightlogFlushTaskHook below.
    flightlog.requestPrepareFlight();
}

// Pending-finalize state. Set by flightlogEndFlight (which runs in the
// I2S Parse task at END_FLIGHT message receipt) and serviced by
// flightlogFlushTaskHook on the flush task (Core 0). Deferral matches the
// pattern used for prepareFlight — keeps NAND-heavy work (FlightIndex::save
// allocates a 2 KB page buffer on the stack via readPage) off the I2S Parse
// task's 4 KB stack. final_bytes is read at service time (not request time)
// because the flush task may still be draining the ring when the request
// fires; reading it then would undercount by the still-buffered bytes.
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
    // While either is true the flush task hasn't yet popped the last byte to
    // the sink, so currentFileBytes() would undercount.
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

    // closeLogSession (which ran in the same flush-task iteration that drained
    // the ring) has already zeroed current_file_bytes by the time we get here.
    // lastClosedSessionBytes() is a sticky snapshot it took just before the
    // reset, so this is the exact byte count the sink ACCEPTED.
    //
    // "accepted", not "was handed": closeLogSession rewinds the count when the
    // final partial page fails to write (#837 item 8). Before that it reported
    // bytes the NAND never took, and finalizeFlight kept a page that was never
    // programmed — the download came back the advertised length with erased
    // 0xFF as its tail.
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

// Drives the deferred Core-0 work for TR_FlightLog. Wired into
// TR_LogToFlash::flushTaskLoop via cfg.flush_task_hook so it executes on the
// flush task's core (Core 0). Logs the prepareFlight outcome here because
// TR_FlightLog itself stays free of ESP_LOG dependencies (host-testable).
static void flightlogFlushTaskHook(void* /*ctx*/)
{
    uint32_t id = 0;
    tr_flightlog::Status st = tr_flightlog::Status::Ok;
    const uint32_t evicted_before = flightlog.autoEvictedCount();
    if (flightlog.servicePendingPrepareFlight(id, st))
    {
        if (st == tr_flightlog::Status::Ok)
        {
            // #315: if this arm auto-evicted to make room, surface it (never
            // silent). The RSS_FLAG_AUTO_EVICTED storage-stats bit carries it to
            // the app; this log line records which flight(s) went.
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

    // Match the legacy LFS filename scheme: when BLE cmd 9 has delivered a
    // phone-synced wall clock, compute "now" = sync_ts + (millis() - sync_millis)
    // and format flight_YYYYMMDD_HHMMSS.bin. Falls back to flight_N.bin when
    // the app has not yet sent a time sync.
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
            d += (uint8_t)(total_s / 86400U);  // days rolling over within a month — good enough
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
    // Defer the actual finalize to the flush task. flightlog.finalizeFlight
    // calls FlightIndex::save which reads NAND with a ~2 KB on-stack page
    // buffer (FlightIndex.cpp:120). The I2S Parse task (where this runs)
    // only has a 4 KB stack — enough for nominal frame parsing but not for
    // any path that pulls in NAND I/O. flightlogServicePendingFinalize on
    // the flush task does the actual work; final_bytes is read there too,
    // since the flush task may still be draining the ring when this runs.
    portENTER_CRITICAL(&g_finalize_mux);
    std::memcpy(g_finalize_name, name, sizeof(g_finalize_name));
    g_finalize_pending = true;
    portEXIT_CRITICAL(&g_finalize_mux);
    // 2c-3c: do NOT delete — the flight has real data now. Index entries
    // accumulate across reboots; deletion becomes an explicit BLE cmd 3
    // operation (re-backed on TR_FlightLog in Stage 3).
}

// ==========================================================================
// SECTION: LoRa radio backend and shared device state
// ==========================================================================
static TR_BLE_To_APP ble_app("TinkerRocket");
// Radio backend seam (#410): direct SPI LLCC68 (V7 boards) or the UART
// radio-daughterboard modem (V8), selected by the board header. The
// reference keeps the historical `lora_comms` name so every call site
// below is untouched; the unused backend is never begun.
static LoRaDirectBackend lora_direct_backend;
static UartModemBackend lora_modem_backend;
static IRadioLink& lora_comms =
    config::USE_UART_RADIO_MODEM
        ? static_cast<IRadioLink&>(lora_modem_backend)
        : static_cast<IRadioLink&>(lora_direct_backend);
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
static volatile uint8_t pending_out_command = 0U;  // command currently being SERVED to the FC

// ==========================================================================
// SECTION: FC command queue (OC -> FC over I2C)
// ==========================================================================
// ---- FC command queue (#366) -----------------------------------------------
// The old single pending slot meant every setPendingCommand() overwrote the
// previous one — the app's connect-time profile sync bursts ~13 commands
// 60-90 ms apart while the FC drains one per 250 ms poll (or none at all with
// the rail off), so most of the burst was silently lost; even the sim's
// config+start pair raced (SIM_START stomped SIM_CONFIG → sims flew default
// motor params, reproduced on the bench 2026-07-03/04).  Commands (with a
// SNAPSHOT of their config payload) now queue FIFO and are served one at a
// time:
//   - snapshot-at-enqueue: a later staging can't clobber an in-flight payload
//   - dedupe by OPERATION: a re-push replaces the queued payload in place
//     (latest wins) instead of flooding the queue — self-applying settings
//     sliders stay bounded. For most commands the operation is just the
//     command id, because the payload IS the command. The two pyro tests
//     carry a channel byte instead, so their key is (id, channel) — see
//     cmd_queue_dedupe_policy.h and #837 item 11.
//   - PYRO_FIRE_TEST / PYRO_CONT_TEST jump to the FRONT: a manual pyro test
//     keeps its immediacy instead of waiting ~15 s behind a profile sync
//   - one idle (cmd=0) poll is served between commands so the FC's dedup
//     (#368: last_processed_cmd resets only when the OC reports 0) sees a
//     reset edge even between back-to-back identical command ids
// With the rail off the queue simply holds; powering on drains the whole
// sync in order — connecting before power-on now works by design.
// EXCEPTION: pyro test commands are never enqueued while the rail is off —
// a held PYRO_FIRE_TEST (or PYRO_CONT_TEST's momentary ARM) would energize
// the channel at the NEXT power-on, possibly while someone is handling the
// rocket. Every entry point refuses instead: the BLE cmd 35/36 handlers
// (with a 0xCE file-ops refusal frame back to the app) and the LoRa cmd
// 35/36 branches in processUplinkCommand (log-only — the uplink has no
// feedback channel, #285 blind fire-and-retry).
struct QueuedCommand
{
    uint8_t cmd;
    uint8_t cfg_type;
    uint8_t cfg_len;
    uint8_t cfg[sizeof(RollProfileData)];
};
// Sync burst is ~13-15 commands. #837 item 11 raised this from 16: the two
// pyro tests are now keyed by channel, so checking all four continuity
// channels can hold 4 slots where the id-only dedupe collapsed them into 1.
// 15 + 4 = 19 would have overflowed a 16-deep queue and dropped a pyro test
// on the floor (logged, but invisible to the operator) if a profile sync were
// still draining. Costs 4 x sizeof(QueuedCommand) = 320 B of static RAM.
static constexpr size_t CMD_QUEUE_DEPTH = 20;
static QueuedCommand cmd_queue[CMD_QUEUE_DEPTH];
static size_t cmd_queue_head  = 0;   // index of next entry to pop
static size_t cmd_queue_count = 0;
static portMUX_TYPE cmd_queue_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t cmd_queue_drops = 0;            // overflow drops (surfaced in stats)
// Serving copy — queueOutStatusResponse reads ONLY these, never the staging
// globals, so a new enqueue can't tear the frame being repeated to the FC.
static uint8_t serving_cfg_type = 0;
static uint8_t serving_cfg_len  = 0;
static uint8_t serving_cfg[sizeof(RollProfileData)] = {};
static bool    cmd_idle_gap_pending = false;    // serve one cmd=0 poll between commands

// Enqueue a command for the FC with its config payload passed explicitly
// (#476: the old shared staging globals leaked stale payloads into
// command-only entries and were a cross-task hazard — the payload is now
// snapshotted from the caller's own buffer, so any task may call this).
// BLE callbacks run on core 0, the LoRa/main loop on core 1 — the critical
// section covers both enqueue and pop.
static void setPendingCommandWithConfig(uint8_t cmd, uint8_t msg_type,
                                        const void* data, size_t len)
{
    if (cmd == 0U)   // legacy "clear serving slot" (internal use only)
    {
        pending_out_command = 0U;
        return;
    }

    QueuedCommand entry;
    entry.cmd      = cmd;
    entry.cfg_type = msg_type;
    if (len > sizeof(entry.cfg)) len = sizeof(entry.cfg);
    entry.cfg_len = (uint8_t)len;
    if (len > 0 && data != nullptr) memcpy(entry.cfg, data, len);

    const bool front = (cmd == PYRO_FIRE_TEST || cmd == PYRO_CONT_TEST);

    portENTER_CRITICAL(&cmd_queue_mux);
    // Dedupe: replace the payload of an already-queued entry describing the
    // SAME OPERATION (latest value wins, position preserved).
    //
    // #837 item 11: "same operation" is not "same command id". The two pyro
    // tests carry their channel in the payload, so matching on the id alone
    // merged a queued test for one channel into a request for another and
    // silently lost the first. cmdQueueSameOperation() folds the channel byte
    // into the key for exactly those commands; a repeat tap on the same
    // channel still collapses, which is what the flood protection is for.
    const uint8_t entry_sel = (entry.cfg_len > 0) ? entry.cfg[0] : 0U;
    for (size_t i = 0; i < cmd_queue_count; i++)
    {
        QueuedCommand& q = cmd_queue[(cmd_queue_head + i) % CMD_QUEUE_DEPTH];
        const uint8_t q_sel = (q.cfg_len > 0) ? q.cfg[0] : 0U;
        if (cmdQueueSameOperation(q.cmd, q_sel, entry.cmd, entry_sel))
        {
            q.cfg_type = entry.cfg_type;
            q.cfg_len  = entry.cfg_len;
            if (entry.cfg_len > 0) memcpy(q.cfg, entry.cfg, entry.cfg_len);
            portEXIT_CRITICAL(&cmd_queue_mux);
            return;
        }
    }
    if (cmd_queue_count >= CMD_QUEUE_DEPTH)
    {
        cmd_queue_drops++;
        portEXIT_CRITICAL(&cmd_queue_mux);
        ESP_LOGW("OC", "FC cmd queue FULL — dropped cmd 0x%02X (drops=%lu)",
                 (unsigned)cmd, (unsigned long)cmd_queue_drops);
        return;
    }
    if (front)
    {
        cmd_queue_head = (cmd_queue_head + CMD_QUEUE_DEPTH - 1) % CMD_QUEUE_DEPTH;
        cmd_queue[cmd_queue_head] = entry;
    }
    else
    {
        cmd_queue[(cmd_queue_head + cmd_queue_count) % CMD_QUEUE_DEPTH] = entry;
    }
    cmd_queue_count++;
    portEXIT_CRITICAL(&cmd_queue_mux);
}

// Command-only relay (no config payload). Safe from any task.
static void setPendingCommand(uint8_t cmd)
{
    setPendingCommandWithConfig(cmd, 0, nullptr, 0);
}
// Command repeat: each served command is repeated for CMD_REPEAT_LIMIT polls.
// Was 5 as insurance against the lossy pre-#399 channel; with the TX-ring
// desync fixed the link runs 100% clean on the bench (311/0 reads through a
// full flight), so 3 keeps plenty of margin while a queued 13-command profile
// sync drains in ~13 s instead of ~20 s ((3+1 idle) x 250 ms per command).
static const uint8_t CMD_REPEAT_LIMIT = 3;
static uint8_t cmd_delivery_count = 0;
static bool camera_recording_requested = false;
// #383: camera/logging uplinks refused because the rocket was INFLIGHT
// (FC skips I2C polls in flight — the command could never be delivered).
static uint32_t uplink_inflight_refusals = 0;
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

// NVS persistence for LoRa settings (config.h values are factory defaults)
static Preferences prefs;
static float   lora_freq_mhz  = config::LORA_FREQ_MHZ;
static uint8_t lora_sf         = config::LORA_SF;
static float   lora_bw_khz    = config::LORA_BW_KHZ;
static uint8_t lora_cr         = config::LORA_CR;
static int8_t  lora_tx_power   = config::LORA_TX_POWER_DBM;

static RocketState latest_rocket_state = INITIALIZATION;
// #825: rail state retained across resets (RTC memory survives everything
// except a true power-on). Updated at every rail toggle; deliberate_off is
// set immediately before the #9 power-off esp_restart() so THAT reboot — the
// one that implements power-off — keeps the boot-time rail-LOW behaviour it
// depends on. Decision table in rail_restore_policy.h.
// INVARIANT for every writer: assign the WHOLE struct in one statement so
// rail_on and deliberate_off can never drift apart across a reset — the
// restore decision reads them as a unit (rail_restore_policy.h), and a
// partial update (e.g. setting deliberate_off while leaving rail_on=1) would
// re-assert a rail the operator just turned off on the NEXT fault reset.
// restore_attempts bounds the brownout retry loop: a sagging pack that
// browns the OC out under the restored load must not cycle the rail forever
// (pre-#825 it settled into rail-off idle; the bound restores that endpoint).
struct RailRtcState { uint32_t magic; uint8_t rail_on; uint8_t deliberate_off;
                      uint8_t restore_attempts; };
static constexpr uint32_t kRailRtcMagic = 0x5241494D;  // "RAIM" (v2: +attempts)
RTC_NOINIT_ATTR static RailRtcState rail_rtc;
static bool boot_rail_restored = false;  // this boot re-asserted the FC rail
// #825: the restore's cmd-8-ON side effects (initPeripherals etc.) must run
// on the SAME core as the normal path — the I2S RX GDMA interrupt is
// allocated on the calling core, and setup_oc runs on core 0 (the BT core)
// while every other initPeripherals call runs from loop_oc on core 1. Set in
// setup, consumed by the first loop_oc iteration.
static bool boot_rail_restore_init_pending = false;

static bool pwr_pin_on = false;
// #846: the latest FC FlightSnapshotData WIRE frame, cached in plain RAM.
// This is the primary snapshot store on every board now: the common recovery
// case is "the FC reset, the OC did not" (post-#859 an OC fault can't even
// drop the FC's rail mid-flight), and for that case RAM is all the
// non-volatility required. The V7/V8 MRAM slot remains as the fallback for
// an OC-also-reset, and V9/V10 cover that case by re-seeding this cache at
// boot from the NAND log stream (the snapshot frames already ride it — every
// received frame is enqueued byte-exact before dispatch). The FC's LANDED
// "clear" snapshot flows through the same path, so cache invalidation is
// inherited, and the FC re-validates magic/version/state/CRC32/sim on its
// side — a stale or garbage cache can never restore a flight.
static constexpr size_t kSnapFrameLen = 4 + 1 + 1 + sizeof(FlightSnapshotData) + 2;
static uint8_t  snapshot_cache[kSnapFrameLen];
static bool     snapshot_cache_valid = false;
// #846: set once the cache has actually been handed to the FC. The boot
// re-seed's once-marker is written on CONSUMPTION, not on seeding — a
// marker written at seed time would burn the flight's only chance if a
// second reset wiped the RAM cache before the FC ever asked.
static bool     snapshot_served = false;
// #846: a boot re-seed is marked consumed only once the FC has actually
// been served the frame (see snapshot_served). Until then these hold the
// identity to stamp into NVS.
static bool     snapshot_seed_pending_mark = false;
static uint32_t snapshot_seed_id = 0;
static uint32_t snapshot_seed_bytes = 0;
// Serve-TTL: a snapshot older than the FC's MAX_FLIGHT_TIME is definitionally
// unrestorable — the flight would have timed out to LANDED — so serving it
// could only ever re-arm a grounded rocket (an FC WDT reset on the bench
// hours after a failed recovery must NOT get last flight's INFLIGHT back).
// Live 10 Hz frames refresh the stamp continuously, so an ACTIVE flight can
// never expire; only an abandoned cache does.
//
// TWO bounds, because cache age alone is not flight age: a frame re-seeded
// from NAND at boot has a fresh stamp but may describe an ancient flight, so
// the frame's OWN flight_elapsed_ms is bounded as well (past MAX_FLIGHT the
// FC's own timeout would already have forced LANDED). snapshotServable()
// applies both and gates the V7/V8 MRAM fallback too — that store has no
// clock of its own and, now that a failed recovery no longer clears it
// (#834-3), can hold an INFLIGHT frame indefinitely.
static constexpr uint32_t kSnapshotServeTtlMs = 600000;  // = FC MAX_FLIGHT_TIME
static uint32_t snapshot_cache_ms = 0;                   // millis() at last write
// Guards the cache against a torn read: the I2S parser task (core 1, prio 6)
// writes it while loop_oc (prio 5) can be serving a GET from I2C ingress.
static portMUX_TYPE snapshot_cache_mux = portMUX_INITIALIZER_UNLOCKED;

// True when a 232-byte snapshot wire frame is fresh enough to hand back.
// `age_ms` is how long ago WE stored it (0 = unknown, e.g. the MRAM slot).
static bool snapshotServable(const uint8_t* frame, uint32_t age_ms)
{
    if (age_ms >= kSnapshotServeTtlMs) return false;
    FlightSnapshotData snap = {};
    memcpy(&snap, frame + 6, sizeof(snap));   // past SOF(4)+type+len
    if (snap.magic != FlightSnapshotData::MAGIC) return false;
    if (snap.rocket_state != (uint8_t)INFLIGHT) return false;   // a clear
    return snap.flight_elapsed_ms < kSnapshotServeTtlMs;
}
// #825: false after a boot rail restore — our reset wiped the commanded
// camera state while the FC (kept alive by the restored rail) may still be
// recording, so the explicit-state dedup below must forward the first
// command instead of "already OFF, ignoring" it. True everywhere else.
static bool camera_state_known = true;              // Power rail state — starts OFF
static bool peripherals_initialized = false; // Deferred init for peripherals behind PWR_PIN

// ==========================================================================
// SECTION: LoRa frequency lock and channel hopping
// ==========================================================================
// Frequency is locked once the rocket enters flight (issue #71).  Any
// Cmd 10 uplink received in flight is ignored, and the slow-rendezvous
// recovery cycle is suppressed — we cannot afford to leave the operating
// frequency mid-flight, and momentary silence during flight is usually
// an SNR dip, not real divergence.
//
// The transition logic itself is pure and lives in the shared header
// (computeFreqLockForFlight) so both the rocket and base station follow
// identical, unit-tested rules.
static bool freq_locked_for_flight = false;

static inline void updateFreqLockFromState(RocketState s)
{
    freq_locked_for_flight = computeFreqLockForFlight(freq_locked_for_flight, s);
}

// ----------------------------------------------------------------------------
// Per-packet channel-hop state (issues #40 / #41, phase 2a)
// ----------------------------------------------------------------------------
// hop_active_       = true once we've started hopping (PRELAUNCH/INFLIGHT).
// hop_idx_          = index into the channel table for the channel we're
//                     currently tuned to (or, equivalently, the channel of
//                     the packet most recently transmitted).  Meaningless
//                     while inactive.
// hop_bootstrap_left_ = remaining bootstrap packets to transmit on the
//                     static lora_freq_mhz channel before entering the
//                     schedule.  #150 bench finding: a SINGLE bootstrap
//                     frame carried the whole BS handoff, and three
//                     independent BS-deaf windows ate it on the bench
//                     (its cmd-17 mirror-retry train, a heartbeat TX,
//                     and a recovery reconfigure racing the transition).
//                     Now every (re)bootstrap sends a full dwell-count
//                     of packets — same per-visit occupancy as any
//                     scheduled dwell, so the FCC math is unchanged —
//                     each stamped with the SCHEDULE ENTRY channel so
//                     any one of them lets the BS park where the rocket
//                     will arrive.  0 = on-schedule (or inactive).
// hop_needs_retune_ = a retune is owed (e.g. just transitioned, or just
//                     finished a TX and need to step to the next channel).
//                     Honoured at the top of serviceLoRa as soon as the
//                     radio is idle (canSend()).
//
// The hop sequence in v2a is intentionally simple: linear (idx, idx+1, …)
// mod the channel-set count from the active BW.  Replacing this with a
// PRNG seeded by network_id is a follow-up; the wire format does not
// change so it can drop in without coordination.
static bool    hop_active_        = false;
static uint8_t hop_idx_           = 0;
static uint8_t hop_bootstrap_left_ = 0;
static bool    hop_needs_retune_  = false;
// Link mode (#106 origin, user-facing since #150): when true, hopping is
// suppressed even in PRELAUNCH/INFLIGHT/LANDED and we stay on
// lora_freq_mhz.  Set by the BS via LORA_CMD_SET_HOP_DISABLED (cmd 17),
// driven by the app's Fixed/Hopping link-mode picker.  NVS-backed so the
// setting survives reboot.  Both sides must agree; the BS keeps its own
// copy, mirrors changes here, and re-pushes on evidence of a mismatch.
// Initialized true (fixed mode, the default) so the window between boot
// and the NVS load can never report hop-enabled.
static bool    lora_hop_disabled  = true;

// LoRa transmit mute — the user-facing "LoRa off" mode.  When true the OC
// makes NO transmission of any kind: no 2 Hz telemetry, no name beacon, and
// no hop schedule (updateHopFromState folds this in, so hopping is forced
// off and the radio returns to lora_freq_mhz).  The RECEIVER stays up:
// serviceLoRaUplink keeps listening and the slow-rendezvous cycle keeps
// visiting the factory channel, both of which are RX-only, so a muted rocket
// is still reachable from the base station and can be un-muted over the air.
// That listening path is the entire reason this is a TX mute and not a radio
// power-down.
//
// NVS-backed ("lora" namespace, key "txdis") so the setting survives reboot.
// It is deliberately on the schema-versioned namespace that gets WIPED on a
// version bump: the wipe direction is TX back ON, which is the safe way to
// lose this setting.  Initialized false so the window between boot and the
// NVS load can never suppress telemetry.
static bool    lora_tx_disabled   = false;

// #150 bench finding (2026-07-15): when the BS mirrors a cmd-17 ENABLE it
// fires an 8 x 100 ms blind retry train and is deaf (half-duplex TX) for
// most of that window.  Activating the hop immediately on first-retry
// receipt made the bootstrap packet collide with retries 2-8: the BS
// missed it and the link sat dark for ~60 s until the fallback visit
// healed it.  Defer activation past the train so the bootstrap lands on
// a listening BS.  0 = no deferred enable pending.  (A disable cancels
// the defer; a state transition during the window may still activate
// early via updateHopFromState — rare and self-healing.)
static constexpr uint32_t HOP_ENABLE_DEFER_MS = 1500;
static uint32_t hop_enable_apply_at_ms = 0;

// Hop-silence rendezvous fallback state (#40 / #41 phase 2b).
// Definitions of HOP_FALLBACK_*_MS constants and the serviceHopFallback()
// function live further down with the other rendezvous machinery.
enum class HopFallbackState : uint8_t {
    NORMAL,
    VISITING_RENDEZVOUS,
    PAUSED_FOR_SCAN,        // BS-coordinated cmd 16 pause (#90)
};

static HopFallbackState hop_fallback_state          = HopFallbackState::NORMAL;
static uint32_t         hop_fallback_phase_start_ms = 0;
static uint32_t         hop_active_entered_ms       = 0;
static uint32_t         hop_session_uplink_count    = 0;  // resets each hop session
static uint32_t         hop_pause_until_ms          = 0;  // wall-clock deadline for PAUSED_FOR_SCAN

// Channel-set state pushed by the BS via LORA_CMD_CHANNEL_SET (#40 / #41
// phase 3).  Rendezvous freq is no longer scan-selected (it's compile-
// time hardcoded to LORA_FACTORY_RENDEZVOUS_MHZ on both sides — see
// #105 for why); only the skip-mask is pushed via cmd 15 now.  skip_mask_
// is consulted in the per-packet next_channel_idx advance so the hop
// sequence skips noisy channels.
static uint8_t skip_mask_[LORA_SKIP_MASK_MAX_BYTES] = {0};
static uint8_t skip_mask_n_        = 0;        // 0 = no mask (all active)
static float   channel_set_bw_khz_ = 0.0f;     // BW the mask was built for

// Tracks whether the radio is currently in RX mode.  Forward-declared
// here so updateHopFromState() (just below) can reset it cleanly when
// we exit a rendezvous visit.  Initialized at file scope further down
// alongside the other LoRa-stats vars.
static bool lora_in_rx_mode = false;

// #150: packets-per-channel dwell for the CURRENT modulation, derived from
// real airtime so the FCC occupancy bound holds at every preset.  0 means
// hopping is not permitted at this (sf, bw, cr) and every hop entry point
// below refuses to start.  Uses the link gate (not the raw config
// function): CR-only changes are unverifiable over the air, so hopping is
// only offered at the factory CR — see loraHopDwellForLink.
static inline uint8_t currentHopDwell()
{
    return loraHopDwellForLink(lora_sf, lora_bw_khz, SIZE_OF_LORA_BUDGET, lora_cr);
}

// #150: effective skip mask for the hop schedule = the cmd-15 noise mask
// (when valid for the current BW) + the implicit home-channel skip
// (loraApplyHomeChannelSkip — keeps transition packets on lora_freq_mhz
// from stacking with a scheduled visit to the same frequency inside one
// FCC window).  `buf` must hold LORA_SKIP_MASK_MAX_BYTES.
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
    // Fixed-frequency mode (#106/#150 link mode) stays on lora_freq_mhz
    // for every state.  #150 also refuses to hop when the current
    // modulation can't fit a dwell visit inside the FCC occupancy budget
    // (currentHopDwell() == 0, e.g. BW125 @ SF9+).
    // A muted radio must not hop: the BS follows the schedule by decoding our
    // packets, so hopping without transmitting just walks the receiver away
    // from the channel the BS is calling on — the one path left for lifting
    // the mute.  Folding it in here (rather than gating the hop service) also
    // gets the ON→OFF branch below for free, which retunes back to
    // lora_freq_mhz and unwinds a rendezvous visit in progress.
    const bool want_active = !lora_tx_disabled && !lora_hop_disabled &&
                             currentHopDwell() > 0 &&
                             shouldHopInState(s);
    if (want_active && !hop_active_)
    {
        // OFF → ON.  Bootstrap: a dwell-count of packets still go out on
        // lora_freq_mhz, each announcing the schedule-entry channel, so
        // the BS can catch ANY one of them and park where we'll arrive
        // (see hop_bootstrap_left_).
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
        // ON → OFF (e.g. INFLIGHT → LANDED): leave the table and return
        // to the static configured channel so recovery / ground comms
        // resume on a known frequency.  If we were mid-rendezvous-visit,
        // come out of that first so the radio ends up with the saved
        // modulation, not the rendezvous one.
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
            // We're already on lora_freq_mhz with the operating preset
            // (cmd 16 reconfigured us there).  No radio-side cleanup
            // needed — just clear the pause state.
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
// Bootstrap packets, PAUSED_FOR_SCAN (#90), and inactive all stay on
// lora_freq_mhz; the active steady state uses the channel table for the
// current BW.
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
// SECTION: Cached configuration (NVS <-> FC <-> app)
// ==========================================================================
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
// Control-authority speed gate (m/s) cached for config readback.  0 = gate off
// (time delay only), which is also what a legacy 16-byte cmd-31 push means.
static float   cfg_roll_min_speed = 0.0f;
// #253: cached roll-control gains for config readback.  <=0 means "firmware
// default" (mirrors the FC's apply semantics for these fields) — the app shows
// its own default in that case.  Written by cacheRollControlConfig on every
// cmd-31 push (the full struct passes through since #387), persisted
// in the "roll" NVS namespace, surfaced via sendCurrentConfig.
static float   cfg_rate_cap_dps  = 0.0f;
static float   cfg_kp_angle      = 0.0f;
static float   cfg_iwind_dps     = -1.0f;  // <0 = firmware default (0 means "disabled")
static bool    cfg_guidance_en  = false;
static uint8_t cfg_camera_type  = CAM_TYPE_RUNCAM;  // default: RunCam

// IMU mounting orientation setting (#phase3): IMU_ORIENT_AUTO lets the
// FC's pad-gravity detect drive the mapping; 0..23 pins a manual code.
// NVS-persisted so a manual roll clocking survives power cycles.
static uint8_t cfg_imu_orient = IMU_ORIENT_AUTO;

// Stage the orientation setting as a two-phase config command to the FC
// (same shape as camera/servo config).  Called from the BLE handler and
// from the status-query self-heal when an FC reboot dropped a MANUAL
// setting back to auto.
static void stageImuOrientConfig()
{
    ImuOrientConfigData cfg;
    cfg.setting = cfg_imu_orient;
        setPendingCommandWithConfig(ORIENT_CONFIG_PENDING, ORIENT_CONFIG_MSG, &cfg, sizeof(cfg));
}

// IMU logging rate setting (BLE cmd 67).  Cached here for app readback and
// re-push on connect; the FC persists it in its own NVS and re-applies at
// boot, so no status-query self-heal is needed (unlike orientation).
// IMU_RATE_DYNAMIC (the default) is a MODE, not a rate — the OC only relays
// it; the step-down at deployment is entirely the FC's business.
static uint16_t cfg_imu_rate = IMU_RATE_DYNAMIC;

static void stageImuRateConfig()
{
    ImuRateConfigData cfg;
    cfg.rate_hz = cfg_imu_rate;
        setPendingCommandWithConfig(IMU_RATE_CONFIG_PENDING, IMU_RATE_CONFIG_MSG, &cfg, sizeof(cfg));
}
// Pyro config cache (4 channels on new PCB)
static bool    cfg_pyro_enabled[4]      = { false, false, false, false };
static uint8_t cfg_pyro_trigger_mode[4] = { 0, 0, 0, 0 };
static float   cfg_pyro_trigger_value[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

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

static IIS2MDCData latest_iis2mdc_raw = {};
static IIS2MDCDataSI latest_iis2mdc_si = {};
static bool latest_iis2mdc_valid = false;

static bool latest_ism6_valid = false;
static bool latest_bmp_valid = false;
static GNSSDataSI latest_gnss_si = {};
static bool latest_gnss_valid = false;
static bool latest_power_valid = false;
static bool latest_non_sensor_valid = false;
// #831: when the last NonSensorData actually arrived.  latest_non_sensor_valid
// latches once at first receipt and is never cleared, so it cannot answer "is
// the FC still alive?" — and every rocket-derived field in the BLE frame, pyro
// continuity included, is republished from that snapshot at full rate whether
// the FC is alive or not.
static uint32_t latest_non_sensor_rx_ms = 0;

// FC boot progress (FC_BOOT_STATUS_MSG), live only during the FC's setup_fc.
// Deliberately NOT cleared when boot completes: the last frame carries the
// final degraded bitmask, and the app stops consulting it once real
// rocket_state frames start arriving.
static FcBootStatusData fc_boot_status{};
static bool     fc_boot_status_valid = false;
static uint32_t fc_boot_status_rx_ms = 0;
// True once NonSensorData has arrived SINCE the last boot-status frame, i.e.
// "this boot has finished and real state is flowing".
//
// Deliberately NOT latest_non_sensor_valid, which is set once at first receipt
// and never cleared for the life of the OC process.  Gating on that worked
// exactly once — on the first FC boot after an OC reset — and then suppressed
// the boot line on every subsequent FC reboot, because the flag was still true
// from the previous FC session.  (That never-aged latch is the same one behind
// the stale-continuity finding.)  This flag is per boot sequence: cleared when
// a boot-status frame arrives, set when NonSensorData does.
static bool     fc_ns_since_boot = false;

static float ground_pressure_pa = 101325.0f;
static bool ground_pressure_set = false;
static float pressure_alt_m = 0.0f;
static float pressure_alt_rate_mps = 0.0f;
static float max_alt_m = 0.0f;
static float max_speed_mps = 0.0f;

static uint32_t lora_tx_ok = 0;
static uint32_t lora_tx_fail = 0;
// LORA_MSG (0xF1) records accepted by the flight-log ring.  Tracked separately
// from lora_tx_ok because the two diverge for a real reason: enqueueFrame()
// drops the record whenever no logging session is open, so tx_ok climbing
// while logged stays flat is the normal pad state, not a fault.  Once a
// session IS open the two must climb together — a gap between them means the
// ring is overrunning and the seq record the post-flight loss analysis
// depends on is being dropped.
static uint32_t lora_tx_logged = 0;
// LORA_UPLINK_MSG (0xF9) records accepted by the ring — every decode the radio
// handed up, not just the ones that passed the filters, so this climbs faster
// than lora_uplink_rx_count whenever the link is noisy.  That difference is
// itself the signal: uplink_rx flat while logged climbs means packets are
// arriving but landing under the SNR floor.
static uint32_t lora_uplink_logged = 0;
// #150: uplinks dropped by the network-id filter.  Counted (and logged in
// the periodic LoRa stats line) because the #133-era nid-drift regression
// was invisible precisely because this drop path said nothing.
static uint32_t lora_uplink_nid_drops = 0;
static uint32_t last_lora_tx_ms = 0;

// Free-running per-TX sequence counter (#105).  Stamped into LoRaData.seq
// on every actual transmit so the BS can compute observed-loss rates and
// the slow-hop seq-anchored channel schedule.  Widened to 16 bits in
// proto v4 — see loraHopDwellForConfig() for why u8 was insufficient.
// Wraps mod 65536; resets to 0 on reboot.
static uint16_t lora_tx_seq = 0;
// lora_in_rx_mode forward-declared up with the hop state.
static uint32_t lora_uplink_rx_count = 0;
// CRC-passing decodes whose SNR was below loraMinValidSnrDb(current_sf)
// — almost certainly noise-floor false positives.  Counted but otherwise
// dropped so they can't act on a fake uplink command (cmd 10 reconfigure
// on garbage payload would be especially bad).  #90 follow-up.
static uint32_t lora_low_snr_drops = 0;

// Slow-rendezvous trackers (issue #71).  last_uplink_rx_ms bumps on every
// successfully-parsed uplink packet; ready_entry_ms latches on each
// INITIALIZATION→READY transition so we don't fire rendezvous just
// because the rocket briefly sat silent before READY.
static uint32_t last_uplink_rx_ms = 0;
static uint32_t ready_entry_ms    = 0;

static OutStatusQueryData last_query_cfg = {};

// #569: last_query_cfg is overwritten WHOLESALE (memcpy) by the I2S parser
// task (core 1, prio 6) and read multi-field by loop_oc (core 1, prio 5).
// The parser strictly preempts loopTask, so an unsynchronized multi-field
// read could observe a torn mix of two status-query generations (e.g. an old
// b2r_code with a new b2r_mode, or a torn b2r_q quaternion) in the LoRa
// orientation packer / imu_orient echo / guid_target echo. The writer holds
// this spinlock for its memcpy; multi-field readers take a whole-struct
// snapshot and work on the local copy. (The parser's OWN field reads right
// after its memcpy need no lock — same task as the writer. The dirty-flag
// publish pattern is safe with this: data is committed under the lock BEFORE
// the flag is set, so a consumer that sees the flag snapshots data at least
// that new.)
static portMUX_TYPE last_query_cfg_mux = portMUX_INITIALIZER_UNLOCKED;
static inline OutStatusQueryData snapshotQueryCfg()
{
    portENTER_CRITICAL(&last_query_cfg_mux);
    const OutStatusQueryData snap = last_query_cfg;
    portEXIT_CRITICAL(&last_query_cfg_mux);
    return snap;
}

// FC firmware version, relayed from the FC via FC_IDENTITY (#8 Phase 4). Cached
// here so the OC can publish it to the app as a "fc_identity" config message,
// letting the app verify an FC OTA / detect a rollback against the FC's *own*
// version (the OC's "fw" never changes on an FC-only update). Re-published when
// it changes mid-connection (an FC OTA never drops the OC<->app BLE link).
static char          fc_fw_version[40]  = {0};
static volatile bool fc_identity_dirty  = false;

// FC's active board→rocket mounting orientation, mirrored from the v3
// status query.  Re-published to the app when it changes mid-connection
// (the pad auto-detect can re-orient any time before launch).
static volatile bool imu_orient_dirty    = false;
static uint8_t       imu_orient_pub_code = 0xFF;   // last published (0xFF = never)
static uint8_t       imu_orient_pub_mode = 0xFF;

// FC's full config report (#915), mirrored from CONFIG_REPORT_MSG.  This is
// the ONLY source for the settings the app could not otherwise see — servo
// trim 2-4, fin travel, fin layout, the PN guidance parameters, the roll
// waypoints, sounds and the orientation SETTING.  Held in RAM only, never
// NVS: a stale report served after an OC reboot would be a confident lie
// about a vehicle we have not heard from, and the FC re-pushes every few
// seconds anyway.  Until the first one lands the OC omits the frames it
// feeds, and the app keeps saying it cannot verify those groups.
static ConfigReportData fc_config_report = {};
static bool             fc_config_report_valid = false;
static volatile bool    fc_config_report_dirty = false;

// FC's guidance-target echo (#435), mirrored from the v5 status query and
// re-published as a compact "guid_target" JSON frame whenever it changes —
// the app's cmd-28 send confirmation is gated on seeing this echo advance.
// 0xFF sentinels = never published (tgt_seq starts at 0 on the FC, so the
// first real query always looks changed and pushes an initial frame).
static volatile bool guid_target_dirty   = false;
static uint8_t       guid_tgt_pub_seq    = 0xFF;
static uint8_t       guid_tgt_pub_status = 0xFF;
static uint8_t       guid_tgt_pub_rc     = 0xFF;

static inline bool nsFlagSet(uint8_t flags, uint8_t mask)
{
    return (flags & mask) != 0U;
}

// ==========================================================================
// SECTION: Battery sampling and state of charge
// ==========================================================================
// Read INA230 and populate latest_power_raw so that the existing telemetry
// pipeline (BLE, LoRa, web) picks up the values automatically.
// Uses triggered mode: fires one conversion (~0.7ms with 1 avg × 332us × 2ch),
// polls CVRF for completion, reads results, then INA returns to power-down.
// Called inline from the main loop at ~100 Hz.
// Plausible bus-voltage window from the INA230 — accept any real powered-board
// reading and reject only a *failed* read.  The floor sits below USB (~5.2 V) so
// USB-bench power is still reported: the operator wants to see the actual reading
// (and knows it's USB) rather than have it suppressed to N/A, even though 5 V reads
// BAD on the 2S scorecard.  A dropped/failed read (0 V — the failure sentinel
// readINA230Power passes on I2C error / CVRF timeout) or NaN/+-Inf falls below the
// floor and is rejected.  Flight pack is 2S (6.6-8.4 V); shBatteryState classifies
// the reading's health (#272/#303).
static constexpr float   POWER_BUS_V_MIN      = 3.0f;   // below USB; still rejects 0 V failed reads
static constexpr float   POWER_BUS_V_MAX      = 9.0f;
// Invalidate cached power (-> battery N/A on the scorecard) only after this many
// consecutive bad reads, so a one-off glitch holds last-good instead of flickering.
static constexpr uint8_t POWER_BAD_READ_LIMIT = 3;

// Validate one INA230 sample and, if plausible, commit it to latest_power_raw and
// mark latest_power_valid.  A failed/stale read must not masquerade as a healthy
// pack (#272): pass bus_v = 0 to register a hard read failure (I2C error / CVRF
// timeout) — 0 is out of range and counts as bad.  After POWER_BAD_READ_LIMIT
// consecutive rejects, latest_power_valid is cleared so the operator sees battery
// N/A rather than a stale or garbage value.  Returns true when the sample was
// accepted.
// ---------------------------------------------------------------------------
//  #850: camera / servo high-side-switch load currents (V9/V10 only)
// ---------------------------------------------------------------------------
// U26 (camera) and U28 (servo) are TPS22811 switches whose IMON pin sources
// GIMON x ILOAD into a ground-referenced gain resistor, so
//   ILOAD = V(pin) / (IMON_GAIN_A_PER_A * R).
// V7/V8 and the mini set the GPIOs to -1 (no such part) and read NaN, which
// packPowerData encodes as 0.
//
// Attenuation: both channels land at 0.286 V at their design currents (camera
// 1.5 A into R85 = 2k, servo 3 A into R88 = 1k). ADC_ATTEN_DB_0 gives ~950 mV
// full scale = 4.98 A / 9.97 A, about 3.3x design headroom, at the finest
// resolution the part offers (1.22 mA / 2.43 mA per count). Do NOT widen the
// attenuation for "headroom": the accuracy floor is the GIMON spread
// (+/-13%), not the range, and a wider atten only coarsens every reading.
//
// Sampled on the existing 100 Hz INA230 tick, so this adds NO new wakeup and
// does not disturb light sleep (#519) — only ~240 us of conversion inside a
// tick that already blocks ~700 us polling the INA230's CVRF.
static adc_oneshot_unit_handle_t rail_adc_unit = nullptr;
static adc_cali_handle_t         rail_adc_cali = nullptr;
static bool                      rail_adc_ready = false;

static constexpr adc_atten_t kRailAtten = ADC_ATTEN_DB_0;

// GPIO8 = ADC1_CH7, GPIO9 = ADC1_CH8 on the ESP32-S3. ADC1 is not the
// WiFi-shared unit, so there is no contention with the BLE controller.
static inline adc_channel_t railChannelForGpio(int gpio)
{
    return (gpio == 8) ? ADC_CHANNEL_7 : ADC_CHANNEL_8;
}

static void initRailCurrentAdc()
{
    if constexpr (config::CAM_IMON_GPIO < 0 && config::SERVO_IMON_GPIO < 0)
    {
        ESP_LOGI("OC", "[IMON] no high-side current monitors on this board — "
                       "camera/servo current will report 0");
        return;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id  = ADC_UNIT_1;
    unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
    if (adc_oneshot_new_unit(&unit_cfg, &rail_adc_unit) != ESP_OK)
    {
        ESP_LOGE("OC", "[IMON] ADC unit init failed — rail currents unavailable");
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten    = kRailAtten;
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    if constexpr (config::CAM_IMON_GPIO >= 0)
    {
        if (adc_oneshot_config_channel(rail_adc_unit,
                                       railChannelForGpio(config::CAM_IMON_GPIO),
                                       &chan_cfg) != ESP_OK)
        {
            ESP_LOGE("OC", "[IMON] camera channel config failed");
            return;
        }
    }
    if constexpr (config::SERVO_IMON_GPIO >= 0)
    {
        if (adc_oneshot_config_channel(rail_adc_unit,
                                       railChannelForGpio(config::SERVO_IMON_GPIO),
                                       &chan_cfg) != ESP_OK)
        {
            ESP_LOGE("OC", "[IMON] servo channel config failed");
            return;
        }
    }

    // Curve fitting is the S3's scheme and is created FOR THIS attenuation —
    // the curve is per-atten, and mixing them mis-scales silently. Same trap
    // the base station's battery ADC documents.
    adc_cali_curve_fitting_config_t cali_cfg = {};
    cali_cfg.unit_id  = ADC_UNIT_1;
    cali_cfg.atten    = kRailAtten;
    cali_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &rail_adc_cali) != ESP_OK)
    {
        ESP_LOGE("OC", "[IMON] ADC calibration unavailable (no eFuse cal data?) — "
                       "reporting no rail current rather than an uncalibrated guess");
        return;
    }

    rail_adc_ready = true;
    ESP_LOGI("OC", "[IMON] ready: cam GPIO%d (%.0f ohm), servo GPIO%d (%.0f ohm), "
                   "GIMON %.1f uA/A",
             config::CAM_IMON_GPIO, (double)config::CAM_IMON_R_OHM,
             config::SERVO_IMON_GPIO, (double)config::SERVO_IMON_R_OHM,
             (double)(config::IMON_GAIN_A_PER_A * 1e6f));
}

// Amps on the given IMON channel, or NaN when unavailable. NaN rather than 0
// deliberately: a board with no monitor and a board whose ADC failed are both
// "unknown", and packPowerData collapses both to 0 on the wire — but the SI
// path keeps them distinguishable from a genuine measured zero.
static float readRailAmps(int gpio, float r_ohm)
{
    if (!rail_adc_ready || gpio < 0 || r_ohm <= 0.0f) return NAN;

    // Four samples. The source impedance here is the 1-2k gain resistor —
    // ~250x lower than the base station's 500k divider, so this does not need
    // that path's 8x averaging; four is enough to halve the ADC's own noise
    // without meaningfully extending the tick.
    constexpr int kSamples = 4;
    int mv_sum = 0, taken = 0;
    const adc_channel_t ch = railChannelForGpio(gpio);
    for (int i = 0; i < kSamples; ++i)
    {
        int raw = 0;
        if (adc_oneshot_read(rail_adc_unit, ch, &raw) != ESP_OK) continue;
        int mv = 0;
        if (adc_cali_raw_to_voltage(rail_adc_cali, raw, &mv) != ESP_OK) continue;
        mv_sum += mv;
        ++taken;
    }
    if (taken == 0) return NAN;

    const float volts = ((float)mv_sum / (float)taken) * 0.001f;
    return volts / (config::IMON_GAIN_A_PER_A * r_ohm);
}

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
    // FC timebase if available (aligns power with the sensor logs); else OC micros().
    psi.time_us = (latest_non_sensor_valid && latest_non_sensor.time_us != 0)
                  ? latest_non_sensor.time_us
                  : (uint32_t)micros();
    psi.voltage = bus_v;
    // Negative = discharging; the INA shunt reads load current positive, so invert.
    psi.current = -current_a * 1000.0f;
    psi.soc     = soc_pct;
    // #850: the two high-side-switch load currents ride the same sample so
    // they share the INA230's timestamp and land in the same logged frame.
    // NaN on boards without the monitors; packPowerData encodes that as 0.
    psi.cam_current   = readRailAmps(config::CAM_IMON_GPIO, config::CAM_IMON_R_OHM);
    psi.servo_current = readRailAmps(config::SERVO_IMON_GPIO, config::SERVO_IMON_R_OHM);
    sensor_converter.packPowerData(psi, latest_power_raw);
    latest_power_valid = true;
    return true;
}

static void readINA230Power()
{
    if (!ina230_ok) return;

    // Trigger a single shunt+bus conversion (INA auto-powers-down after)
    ina230.setMode(INA230_Mode::SHUNT_BUS_TRIG);

    // Poll CVRF (Conversion Ready Flag, bit 3 of Mask/Enable register)
    // instead of a fixed delay.  Conversion takes ~0.7ms (1 avg × 332µs × 2ch).
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

    // A CVRF timeout means the conversion never completed, so any value read back
    // would be stale — treat that (and any I2C read error) as a failed sample
    // (#272) by passing 0 V, which commitPowerSample rejects.  NOTE: readMaskEnable
    // clears CVRF, so we must capture it in the poll above, not re-read it here.
    float bus_v = 0.0f, current_a = 0.0f;
    const bool read_ok = cvrf
        && ina230.readBusVoltage_V(&bus_v) == TR_INA230_OK
        && ina230.readCurrent_A(&current_a) == TR_INA230_OK;
    commitPowerSample(read_ok ? bus_v : 0.0f, read_ok ? current_a : 0.0f);
}

// ---------------------------------------------------------------------------
//  #519 power-config guard — fail the BUILD, not the ammeter.
//
//  sdkconfig is generated and OVERRIDES sdkconfig.defaults, so a checkout whose
//  sdkconfig predates a defaults change silently builds current source against
//  the old config. Third bite of this trap (after #518, #519): on 2026-07-15 a
//  rebuild from a stale sdkconfig shipped MAIN_XTAL as the BT low-power clock —
//  the controller then holds a permanent no_light_sleep PM lock, light sleep
//  never engages, and OC idle sat at ~7 mA instead of ~1 mA with no warning
//  anywhere. Each symbol below is one the OC's idle-power contract rides on.
//
//  If this fires, do NOT edit sdkconfig — regenerate it from the authoritative
//  defaults:  rm sdkconfig && idf.py build
//  If you are intentionally changing the power config, change
//  sdkconfig.defaults and update this guard in the same commit.
// ---------------------------------------------------------------------------
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
// SECTION: Clock sources, low-power mode, and boot USB-serial grace
// ==========================================================================
// ---------------------------------------------------------------------------
//  Low-power mode helpers
//  Called at boot and when power rail is turned OFF.
//  NOTE: Light sleep, BT modem sleep, and reduced TX power disabled —
//  they caused BLE discoverability issues on the Arduino build.
//  These optimisations are active in the ESP-IDF build (tinkerrocket-idf)
//  which has proper sdkconfig support.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// #541: light-sleep grace so USB enumeration can finish.
//
// Light sleep is held off by exactly two locks: the BT controller's (only when
// its LP clock fell back to main XTAL) and the USJ driver's
// USJ_NO_AUTO_LS_ON_CONNECTION lock — which is only taken once a USB
// connection is DETECTED. Right after a chip reset, host re-enumeration takes
// ~1 s; on a 32k-crystal boot (BT lock absent) nothing holds sleep off during
// that window, and losing the race wedges the USJ so the host never sees the
// port again until a physical re-plug (bench 2026-07-17: 2 transient + 3
// re-plug-required incidents in one day, all immediately after resets; the BS,
// which runs no light sleep, has never dropped once).
//
// Hold ESP_PM_NO_LIGHT_SLEEP for the first kBootUsjGraceMs after light sleep
// is enabled, then release. On battery (no host) this delays the first
// possible sleep by 10 s once per boot — irrelevant to flight power.
// ---------------------------------------------------------------------------
#if defined(CONFIG_PM_ENABLE)
static esp_pm_lock_handle_t s_boot_usj_grace_lock = nullptr;
static esp_timer_handle_t   s_boot_usj_grace_timer = nullptr;
static bool                 s_boot_usj_grace_held = false;
static constexpr uint32_t   kBootUsjGraceMs = 10000;

static void bootUsjGraceExpired(void*)
{
#if defined(TR_OC_BENCH_DEBUG)
    // Bench-debug image: never release. Light sleep powers down the USB
    // Serial/JTAG peripheral the console rides on (ESP_CONSOLE_UART_NUM=-1,
    // secondary none — USJ is the ONLY console), so once this lock drops the
    // OC goes permanently silent to a host, ~11 s after every boot.
    //
    // That used to be masked: before #519 the BT controller took its own
    // no_light_sleep lock on MAIN_XTAL, so an OC with BLE up never slept and
    // the console stayed alive. #519 moved the BT LP clock to the 32k crystal
    // — the power win — and with it went the accidental console lifeline.
    // Found on the bench 2026-07-28 while trying to get OC-side evidence for
    // #627: four capture attempts, all silent from exactly this line onward.
    //
    // Holding forever costs the whole #519 saving (~22 mA vs ~1 mA), which is
    // why this is opt-in and stamped into the version string — a bench image
    // must never be mistaken for a flight image.
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
#if defined(TR_OC_BENCH_DEBUG)
    ESP_LOGW("PWR", "BENCH DEBUG build — the USB console will stay alive past "
                    "the %lu ms grace instead of going quiet (#627 bench).",
             (unsigned long)kBootUsjGraceMs);
#else
    ESP_LOGI("PWR", "Light sleep held %lu ms for USB enumeration (#541)",
             (unsigned long)kBootUsjGraceMs);
#endif
}
#endif  // CONFIG_PM_ENABLE

// ---------------------------------------------------------------------------
// #541: second chance for the 32.768 kHz crystal, BEFORE BT init.
//
// IDF's boot-time selection gives the crystal exactly RTC_XTAL_CAL_RETRY(=1)+1
// calibration attempts of ~92 ms each (RTC_CLK_CAL_CYCLES=3000) before
// silently falling back to the internal 150 kHz RC — and the BT controller
// then picks main XTAL as its LP clock, quietly reverting the whole #519
// power win ("light sleep will not be able to apply"). Bench data (#541):
// this board's crystal starts 7/7 COLD but intermittently fails WARM, and a
// warm tuning-fork crystal can need hundreds of ms — more than IDF's window.
//
// The knobs that don't work: ESP_SYSTEM_RTC_EXT_XTAL_BOOTSTRAP_CYCLES is a
// STUB on S3 (rtc_clk_32k_bootstrap() ignores its argument), and the retry
// count is a hardcoded #define in IDF's clk.c. So: if the slow clock fell
// back, re-enable the crystal here and give it up to 2 s of extra attempts.
// On success, adopt it (source switch + fresh calibration) so BT and light
// sleep still get the crystal; on failure, say so LOUDLY — before this, the
// only tell was one easily-missed BT_INIT info line.
// ---------------------------------------------------------------------------
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
        // so RTC timekeeping stays consistent (same pair of calls IDF's own
        // select_rtc_slow_clk() makes).
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

static void enterLowPowerMode()
{
#if defined(CONFIG_PM_ENABLE)
    // Low-power idle: 80 MHz max (BLE needs 80 MHz APB), 40 MHz min via DFS.
    //
    // #519: light sleep is now ENABLED here. It used to be off "because btLS
    // blocks it; USB-Serial-JTAG incompatible" — both reasons are now handled:
    //
    //  - "btLS blocks it": the BT controller sets no_light_sleep when its
    //    low-power clock is the MAIN_XTAL (bt.c). Pointing that clock at the
    //    board's 32.768 kHz crystal instead (RTC_CLK_SRC_EXT_CRYS +
    //    BT_CTRL_LPCLK_SEL_EXT_32K_XTAL) means it never sets that flag. This was
    //    NOT a hardware limitation, it was a config we chose. Bench-confirmed:
    //    "BLE_INIT: Using external 32.768 kHz crystal/oscillator as clock source".
    //
    //  - "USB-Serial-JTAG incompatible": true, but ESP-IDF handles it for us.
    //    CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION makes the USJ driver hold an
    //    ESP_PM_NO_LIGHT_SLEEP lock while the USB port is actually connected and
    //    release it when it is unplugged. So the console keeps working on the
    //    bench, and light sleep engages in flight when USB is gone — which is the
    //    only time idle current matters anyway.
    //
    // MEASUREMENT GOTCHA: with USB plugged in, that lock is held and the chip will
    // NOT light-sleep. Idle current has to be measured on battery, USB detached,
    // or you will see no change and wrongly conclude this did nothing.
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

    // #541: give USB enumeration a sleep-free window (see holdBootUsjGrace).
    // Re-armed on every low-power entry — a rail-off transition can coincide
    // with a fresh host attach on the bench, same race.
    holdBootUsjGrace();

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

// ==========================================================================
// SECTION: BLE connection-parameter policy
// ==========================================================================
// #519: both of these took a conn_handle argument and BOTH call sites passed a
// literal 0 — while NimBLE had handed out handle 1. So every request failed with
// "GAP update_params: connection not found; conn_handle=0x0000" and neither the
// slow (low-power) nor the fast (file-transfer) parameters have EVER been applied.
// They now take no handle and read the live one from the BLE layer.
// #524: go through TR_BLE_To_APP rather than calling ble_gap_update_params() here.
// It records the requested set, so when iOS rejects with a transaction collision the
// retry re-asks for THE SAME set. Calling the HCI directly bypassed that: the retry
// path had its own hardcoded 15-30 ms, so a SLOW request that lost a collision race
// came back as a FAST one and silently cancelled low-power mode.
//
// The negotiated result is reported by TR_BLE_To_APP's CONN_UPDATE handler, which logs
// the interval actually in force — the peer is free to counter (#503).
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
// before dropping to the low-power idle set. The app's connect burst (identity,
// config readbacks, MagCal read, profile sync) keeps issuing commands for a few
// seconds past the push; dropping to 200 ms/latency-4 mid-burst made every
// remaining exchange cost ~500 ms (measured: first command 57 ms, then
// 300-800 ms — 5-8 s of "connecting…" in the app). 8 s covers the burst with
// generous margin (the whole burst takes ~1-2 s on a fast link) and costs ~8 s
// of fast-interval idle current once per connection — negligible.
static constexpr uint32_t kSlowParamsDeferMs = 8000;

// Slow parameters for low-power idle: fewer BLE wakeups is the single biggest
// lever on idle current. Within Apple's envelope: min >= 15 ms; max >= min + 15 ms;
// latency <= 30; itvl_max * (latency + 1) = 1000 ms <= 2 s; supervision timeout
// (4 s) > itvl_max * (latency + 1) * 3 = 3 s, and <= 6 s.
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

// Fast parameters for file transfer.
// #503: this asked for itvl_min = 0x06 = 7.5 ms, which is BELOW iOS's 15 ms floor
// and could never be granted — the same bug fixed in TR_BLE_To_APP, living on in a
// second copy here. Now inside Apple's envelope (15-30 ms), like the shared path.
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

// #524: never start a transfer on the idle link.
//
// Bench 2026-07-14: the Power-On fast-param request lost a collision race with iOS
// (status=554 x4), the retry budget ran out, and the link stayed at 200 ms / latency 4
// — a ONE SECOND effective event period, 33x slower than the fast link. The download
// that followed aborted after 5 kB of an 8.5 MB file.
//
// This is very likely also the original "~50% data loss" that the old fixed 30 ms
// per-chunk delay was invented to hide: a download on the idle link drains 33x slower,
// so of course the notify queue overflowed. The delay throttled the producer enough to
// survive it, and we spent years believing the phone could not keep up.
//
// So: ask again, at the point where it actually matters, and give the peer a moment to
// answer before streaming. Not fatal if it never lands — sendFileChunk's budget now
// scales with the link, so a slow transfer completes rather than aborting — but say so.
static constexpr uint32_t FAST_LINK_EVENT_MS_MAX = 60;    // 30 ms interval, latency 0, + slack
static constexpr uint32_t FAST_LINK_WAIT_MS      = 3000;

static void ensureFastLinkForTransfer()
{
    if (ble_app.effectiveEventMs() <= FAST_LINK_EVENT_MS_MAX) return;   // already fast

    ESP_LOGW("BLE", "Link too slow for a transfer (%lu ms effective) — asking for fast params",
             (unsigned long)ble_app.effectiveEventMs());
    requestFastBLEParams();

    // Waiting here is free: this is a ground operation, and oc_loop is already given
    // over to the transfer.
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
// SECTION: Derived telemetry (altitude and speed)
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

// ==========================================================================
// SECTION: IMU full-scale decode helpers
// ==========================================================================
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

// ==========================================================================
// SECTION: Ingest ring buffers and frame counters
// ==========================================================================
// I2S telemetry ring (high-volume, parsed by I2S receiver task)
static constexpr size_t RX_STREAM_RING = 65536;
static uint8_t rx_ring[RX_STREAM_RING];

// I2C command ring (low-volume, parsed by loopTask)
static constexpr size_t CMD_RING_SIZE = 1024;
static uint8_t cmd_ring[CMD_RING_SIZE];
static size_t cmd_head = 0;
static size_t cmd_tail = 0;
static uint32_t cmd_ring_drop_count = 0;  // #297: bytes dropped-oldest on overflow (visibility; CRC still rejects the corrupted result)

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
    {
        cmd_tail = (cmd_tail + 1U) % CMD_RING_SIZE;  // drop oldest on overflow
        cmd_ring_drop_count++;                       // #297
    }
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

// High-water mark of rx_ring fill since the last LOG TIMING reset.  Updated
// in rxPush (ISR/parser context) — cheap to track and the only way to see
// if rx_ring filled silently without overflowing.  #104 follow-up: pairs
// with the parser_iter_max_us measurement so we can tell whether bytes
// were lost at the DMA→rx_ring stage or whether the parser fell behind.
static volatile uint32_t rx_ring_peak_fill = 0;
static uint64_t parser_resync_drops = 0;
static uint64_t parser_len_drops = 0;

static uint32_t frames_bad_crc = 0;
static volatile uint32_t dma_cb_count = 0;      // DMA callback invocations
static uint32_t dedup_drops_lt = 0;              // ts strictly less than prev (replay / reorder)
static uint32_t dedup_drops_eq = 0;              // ts exactly equal to prev (byte-duplicate)
static uint32_t dedup_replay_drops = 0;          // #468: >10 s backstep, unconfirmed (replayed TX descriptor)
static uint32_t stale_drops = 0;                 // stale timestamp rejects
static uint32_t raw_i2c_reads = 0;
static uint64_t raw_i2c_bytes = 0;
static uint32_t msg_count_query = 0;
static uint32_t msg_count_ism6 = 0;
static uint32_t msg_count_bmp = 0;
static uint32_t msg_count_mmc = 0;
static uint32_t msg_count_iis2mdc = 0;
static uint32_t msg_count_gnss = 0;
static uint32_t msg_count_non_sensor = 0;
static uint32_t msg_count_power = 0;
static uint32_t msg_count_start_logging = 0;
static uint32_t msg_count_end_flight = 0;
static uint32_t msg_count_unknown = 0;
// #569: types accepted by isKnownMessageType and written to the flight log by
// the enqueue path, but with no live handler branch (GUIDANCE_TELEM_MSG,
// FLIGHT_SETTINGS_MSG, LORA_MSG, GNSS_SAT_MSG). They used to fall through to
// msg_count_unknown++, so a guided flight showed a climbing "unknown" rate
// that read as I2S corruption and masked genuinely unknown frames.
static uint32_t msg_count_logonly = 0;

static uint32_t prev_msg_count_query = 0;
static uint32_t prev_msg_count_ism6 = 0;
static uint32_t prev_msg_count_bmp = 0;
static uint32_t prev_msg_count_mmc = 0;
static uint32_t prev_msg_count_iis2mdc = 0;
static uint32_t prev_msg_count_gnss = 0;
static uint32_t prev_msg_count_non_sensor = 0;
static uint32_t prev_msg_count_power = 0;
static uint32_t prev_msg_count_start_logging = 0;
static uint32_t prev_msg_count_end_flight = 0;
static uint32_t prev_msg_count_unknown = 0;
static uint32_t prev_msg_count_logonly = 0;   // #569
static uint32_t last_stats_ms = 0;
static uint64_t prev_bytes_rx = 0;
static uint64_t prev_bytes_nand = 0;
static uint64_t prev_raw_i2c_bytes = 0;
static uint32_t prev_ring_overruns = 0;
static uint32_t prev_ring_drop_oldest_bytes = 0;
static uint32_t prev_ring_bad_sof_clears = 0;
static uint32_t interval_ring_fill_peak = 0;

static inline size_t rxLen()
{
    if (rx_head >= rx_tail) return rx_head - rx_tail;
    return RX_STREAM_RING - (rx_tail - rx_head);
}

static inline IRAM_ATTR void rxPush(uint8_t b)
{
    // #383: drop the NEWEST byte on overflow. The old drop-oldest advanced
    // rx_tail from the ISR while the parser task does its own non-atomic RMW
    // in rxPop() — an interleave could move the tail backwards and re-parse
    // stale bytes (caught by SOF/CRC, but wasted work exactly when the ring
    // is already drowning). With the ISR never touching rx_tail this is a
    // textbook single-producer/single-consumer ring; one slot is sacrificed
    // as the full marker. Either policy corrupts the in-flight frame during
    // an overflow — framing resynchronizes on the next SOF regardless.
    const size_t next = (rx_head + 1U) % RX_STREAM_RING;
    if (next == rx_tail)
    {
        rx_ring_overflow_drops = rx_ring_overflow_drops + 1;  // volatile: '++' deprecated in C++20 (-Wvolatile)
        return;
    }
    rx_ring[rx_head] = b;
    rx_head = next;
    // High-water tracking — non-atomic but only one ISR context produces.
    const uint32_t fill = (rx_head >= rx_tail)
                            ? (rx_head - rx_tail)
                            : (RX_STREAM_RING - (rx_tail - rx_head));
    if (fill > rx_ring_peak_fill) rx_ring_peak_fill = fill;
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

// ==========================================================================
// SECTION: Phone-I/O power management
// ==========================================================================
// #524: hold the CPU at full speed and out of light sleep while we are serving
// the phone.
//
// A download runs for MINUTES entirely inside enterLowPowerMode()'s regime:
// 80/40 MHz DFS with light sleep armed. Neither is free here. Throughput is set
// by how many packets we land in each 30 ms connection event, and a core that is
// asleep — or at 40 MHz — when the event opens cannot refill the controller's
// queue in time to fill it.
//
// The 34.7 kB/s bench result was measured over USB, where
// CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION holds a NO_LIGHT_SLEEP lock on our behalf,
// so light sleep never actually engaged during that test. On battery it would,
// and we would quietly hand part of the win back. Hold the locks ourselves rather
// than depend on a USB cable being plugged in — the same "measure it on battery"
// trap as #519, in mirror image.
//
// Costs nothing real: idle current only matters when nobody is talking to us, and
// during a transfer the radio is busy regardless.
#if defined(CONFIG_PM_ENABLE)
static esp_pm_lock_handle_t s_phone_io_cpu_lock = nullptr;  // ESP_PM_CPU_FREQ_MAX
static esp_pm_lock_handle_t s_phone_io_ls_lock  = nullptr;  // ESP_PM_NO_LIGHT_SLEEP

// Did the locks actually engage? A silently-failed esp_pm_lock_create() would look
// exactly like "light sleep was never the problem", and we would draw the opposite
// conclusion from the same evidence. So report it rather than assume it.
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

// #524: the XFER diagnostic is worthless if you cannot READ it — and on battery you
// cannot, because the console is USB-Serial-JTAG: unplugging the cable takes the
// console with it. That is exactly the run we most need to see (battery downloads
// measured ~2.7x slower than USB).
//
// So latch the last transfer's summary and keep re-emitting it for a few minutes.
// That is long enough to finish a battery download, plug USB back in, and attach a
// monitor WITHOUT resetting the board:
//
//     idf.py -p <PORT> monitor --no-reset
//
// (a plain `idf.py monitor` resets the chip and you lose it).
static char     s_xfer_summary[256]     = {0};
static uint32_t s_xfer_reprint_until_ms = 0;
static uint32_t s_xfer_next_reprint_ms  = 0;
static constexpr uint32_t XFER_REPRINT_WINDOW_MS = 240000;  // 4 min to get a cable in
static constexpr uint32_t XFER_REPRINT_EVERY_MS  = 15000;

// beginPhoneIO / endPhoneIO — bracket a phone-serving operation (file
// list / delete / download). Pauses both I2C servicing (flash_op_active)
// and I2S ingest (i2s_ingest_paused), then drains the rx ring on resume
// so stale sensor bytes from before the pause aren't processed as if
// they were current. Must be called from a single task (oc_loop).
//
// esp_pm locks are counting, and every begin has a matching end on every path
// (the download loop breaks out, it never returns), so the PM lock cannot leak
// and strand us at full power.
static inline void beginPhoneIO()
{
    i2s_ingest_paused = true;
    flash_op_active   = true;
    phoneIoPmAcquire();
}
static inline void endPhoneIO()
{
    // Drain while the ISR is still suppressed — race-free.
    rx_tail = rx_head;
    i2s_ingest_paused = false;
    flash_op_active   = false;
    phoneIoPmRelease();
}

// ==========================================================================
// SECTION: pre-restart storage quiesce (#834 item 2)
// ==========================================================================
// Every deliberate esp_restart() on this MCU must leave the storage stack in a
// state where the reset is indistinguishable from a clean power cycle.
//
// WHAT IS ACTUALLY AT RISK, AND WHAT IS NOT. U11 (the SPI NAND) sits on the
// ALWAYS-ON +3V3 rail — U18's EN is tied to its own AVIN — so esp_restart()
// removes no power from it. A PROGRAM EXECUTE or BLOCK ERASE the chip has
// already latched is self-timed and lands correctly across the reset, and
// FlightIndex::save() writes the OLDER-or-invalid metadata copy, so a reset
// between its erase and its program still leaves a valid snapshot. The reset
// therefore CANNOT half-program a page — which is what this was first assumed
// to be about.
//
// What it CAN do, and did, on EVERY power-off: throw away up to 512 KB of
// PSRAM log ring plus the staged partial page, the phone-synced filename and
// the exact byte count. That ring is volatile in-package PSRAM on V9/V10
// (there is no MRAM), so nothing else is holding that data. Draining it is
// the whole point of this function. The bus park at the end is the small
// remaining piece: it stops a command being cut mid-clock, which is benign
// here but is what makes the GPIO teardown that follows provably safe rather
// than probably safe.
//
// Why it lives in main.cpp: closing a flight is a three-party handshake — the
// logger drains the ring and closes the session, g_finalize_pending stages the
// index commit, TR_FlightLog writes it — and only main.cpp sees all three.
//
// NEVER REFUSES, unlike the mini's comms_prepare_power_off(). On the mini the
// rail drop kills the NAND, so refusing genuinely protects the data. Here the
// flash is powered regardless, PWR_PIN is already LOW by the time we run, the
// reset IS the power-off, and cmd 8 sends no acknowledgement frame on OFF — so
// a refusal would be an invisible half-off state the operator cannot diagnose.
static constexpr uint32_t kQuiesceCloseTimeoutMs = 10000;
static constexpr uint32_t kSpiParkTimeoutMs      = 250;
static bool s_restart_quiesced = false;

static void quiesceStorageForRestart(const char* why)
{
    // One-shot. Also load-bearing: phoneIoPmAcquire() inside beginPhoneIO() is
    // a COUNTING esp_pm lock with no matching release on this path, so
    // re-entering would leak lock counts.
    if (s_restart_quiesced) return;
    s_restart_quiesced = true;

    const uint32_t t0 = millis();

    // 1. Shut the producers. i2s_ingest_paused gates the DMA recv callback and
    //    flash_op_active gates serviceI2CIngress, so no new flight can be
    //    staged into the very iteration we are trying to end — a straggler
    //    prepareFlight would set flight_active_ again and start an 80-block
    //    erase, and the predicate below would never converge. Deliberately
    //    never un-paused: every path out of here resets.
    beginPhoneIO();

    // 2. Close the flight. endLogging() latches end_flight_requested, which
    //    makes enqueueFrame reject, so the ring cannot grow again and the
    //    drain converges even with frames still in the rx ring.
    if (logger.isLoggingActive())
    {
        logger.endLogging();
        flightlogEndFlight();
    }
    // NOTE deliberately NO else-branch for "flight active but logging never
    // activated" (a flight pre-created at PRELAUNCH that never received data).
    // Finalizing it here would stamp it with logger.lastClosedSessionBytes(),
    // which is sticky from the PREVIOUS flight — producing a phantom
    // multi-megabyte entry over erased pages whose blocks then read as
    // in-index, so brownout recovery can never reclaim them. With
    // auto_evict_oldest on, each phantom also consumes headroom and is always
    // the newest entry, so arming the next flight evicts a REAL one instead.
    // Boot recovery already releases an orphan ALLOCATED run that has no valid
    // pages, which is the correct and lossless handling.

    // 3. Wait for the flush task to drain the ring, close the session and
    //    service the deferred finalize. This needs at least TWO flush
    //    iterations by construction: the hook runs before the end-flight block
    //    within one iteration, so the finalize can only be serviced on the
    //    pass after the one that closed.
    const uint32_t deadline = millis() + kQuiesceCloseTimeoutMs;
    bool closed = false;
    for (;;)
    {
        bool finalize_pending;
        portENTER_CRITICAL(&g_finalize_mux);
        finalize_pending = g_finalize_pending;
        portEXIT_CRITICAL(&g_finalize_mux);

        // What this wait can actually influence: the ring drain and the
        // session close, plus the deferred index commit those stage.
        //
        // isFlightActive() is deliberately NOT a term. It stays true for a
        // flight pre-created at PRELAUNCH that never received data (nothing
        // closes it, and finalizing it here would fabricate a phantom entry —
        // see step 2), and it also stays true forever if a deferred
        // finalizeFlight() failed its metadata write. Either way it is a state
        // this function cannot clear, so including it would just spin out the
        // full timeout on every power-off. Boot recovery reclaims both.
        //
        // isPrepareFlightPending() IS a term: an arm staged but not yet
        // serviced would otherwise let the predicate pass, and the flush task
        // would then start an 80-block erase burst underneath the park.
        if (!logger.isLoggingActive() && !finalize_pending &&
            !flightlog.isPrepareFlightPending())
        {
            closed = true;
            break;
        }
        if ((int32_t)(deadline - millis()) <= 0)          // wrap-safe
        {
            ESP_LOGE("PWR", "%s: log close did not complete in %u ms "
                            "(logging=%d finalize=%d prepare_pending=%d) — "
                            "continuing anyway; an unclosed tail is recoverable",
                     why, (unsigned)kQuiesceCloseTimeoutMs,
                     (int)logger.isLoggingActive(), (int)finalize_pending,
                     (int)flightlog.isPrepareFlightPending());
            break;
        }
        delay(50);
    }

    // 4. Park the SPI bus and never give it back. MUST be the LAST logger or
    //    flightlog call on this path. Uncontended on the clean path (step 3
    //    has converged, so the flush task is idle at its vTaskDelay); the
    //    timeout only matters after a step-3 timeout, and there the bus is not
    //    coming back anyway.
    const bool parked = logger.parkSpiBusForReset(kSpiParkTimeoutMs);
    if (!parked)
    {
        ESP_LOGE("PWR", "%s: SPI bus still held after %u ms — resetting on top "
                        "of it (#834 item 2)", why, (unsigned)kSpiParkTimeoutMs);
    }

    ESP_LOGI("PWR", "%s: storage quiesced in %lu ms (closed=%d parked=%d)",
             why, (unsigned long)(millis() - t0), (int)closed, (int)parked);
}

// ==========================================================================
// SECTION: I2C status-query response to the FC
// ==========================================================================
// The FC reads exactly FC_COMBINED_READ_SIZE bytes per I2C poll.
// The new i2c_slave driver panics if the master clocks out more bytes
// than are in the TX ringbuffer.  Always write exactly this many bytes
// so the slave hardware never underflows.
static constexpr size_t FC_COMBINED_READ_SIZE = 96;
// TX-byte accounting differs by slave-driver version (#88):
//   V1: the hardware "eats" 1 extra byte per master read (prefetched into the
//       shift register, discarded at STOP), so we padded by 1 to keep the
//       ringbuffer drained and responses aligned.
//   V2 (IDF >= 5.4): does NOT consume that extra byte. Anything staged beyond
//       what the FC clocks out lingers in the TX ringbuffer and accumulates
//       across polls, so the SOF drifts forward a byte each read and only the
//       first (aligned) read parses (bench: query ok/fail stuck at 1/N).
//       Under V2, stage exactly what the FC reads (pad = 0). Staged == read
//       also avoids the V2 driver's "master clocked out more than staged"
//       underflow.
// V2 is in use when the 5.4/5.5 opt-in flag is set, OR unconditionally on
// 6.0+ (V1 removed). The flag macro is absent on 6.0, so test the version too.
#include <esp_idf_version.h>
#if (defined(CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2) && CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2) \
    || (ESP_IDF_VERSION_MAJOR >= 6)
static constexpr size_t I2C_TX_PAD = 0;   // V2 slave driver
#else
static constexpr size_t I2C_TX_PAD = 1;   // legacy V1
#endif
static constexpr size_t I2C_TX_SIZE = FC_COMBINED_READ_SIZE + I2C_TX_PAD;

// #569: the combined slave response is [status frame][optional config frame],
// read as EXACTLY FC_COMBINED_READ_SIZE bytes by the FC. A config frame that
// doesn't fit is DROPPED at pack time (now logged, but a drop is still a
// config that never reaches the FC), so guarantee at compile time that the
// largest config payload staged via setPendingCommandWithConfig() fits behind
// the status frame. RollProfileData is the documented largest; if a bigger
// config is ever added, extend this assert (the runtime ESP_LOGE below is the
// backstop for anything missed).
static constexpr size_t I2C_FRAME_OVERHEAD_B = 4 /*SOF*/ + 1 /*type*/ + 1 /*len*/ + 2 /*CRC*/;
static constexpr size_t I2C_STATUS_FRAME_B   = 2 /*payload*/ + I2C_FRAME_OVERHEAD_B;
static_assert(I2C_STATUS_FRAME_B + sizeof(RollProfileData) + I2C_FRAME_OVERHEAD_B
                  <= FC_COMBINED_READ_SIZE,
              "largest config frame (RollProfileData) no longer fits the combined I2C read");

static void queueOutStatusResponse(bool ready)
{
    // #366: serving-slot lifecycle.  When idle, first serve one cmd=0 poll
    // (the FC's dedup reset edge — #368), then pop the next queued command
    // into the serving copy.
    if (pending_out_command == 0U)
    {
        if (cmd_idle_gap_pending)
        {
            cmd_idle_gap_pending = false;   // this poll reports cmd=0
        }
        else
        {
            portENTER_CRITICAL(&cmd_queue_mux);
            if (cmd_queue_count > 0)
            {
                const QueuedCommand& q = cmd_queue[cmd_queue_head];
                serving_cfg_type = q.cfg_type;
                serving_cfg_len  = q.cfg_len;
                if (q.cfg_len > 0) memcpy(serving_cfg, q.cfg, q.cfg_len);
                pending_out_command = q.cmd;
                cmd_queue_head = (cmd_queue_head + 1) % CMD_QUEUE_DEPTH;
                cmd_queue_count--;
                cmd_delivery_count = 0;
            }
            portEXIT_CRITICAL(&cmd_queue_mux);
        }
    }

    const uint8_t cmd = pending_out_command;

    if (cmd != 0U)
    {
        ESP_LOGI("OC", "I2C TX StatusResponse: ready=%d cmd=0x%02X attempt=%u/%u (queued=%u)",
                      ready ? 1 : 0, (unsigned)cmd,
                      (unsigned)(cmd_delivery_count + 1), (unsigned)CMD_REPEAT_LIMIT,
                      (unsigned)cmd_queue_count);
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

    // Append the config payload when the entry carries one (#366: from the
    // SERVING snapshot — a concurrent enqueue can't tear this frame).
    // #476: cfg_len > 0 is authoritative — payloads are passed explicitly to
    // setPendingCommandWithConfig(), so there is no whitelist to desync (the
    // old isConfigCommand() list silently ate the IMU-rate frame in #473)
    // and no stale staging data to leak into command-only entries.
    if (cmd != 0U && serving_cfg_len > 0)
    {
        uint8_t cfg_frame[MAX_FRAME];
        size_t  cfg_frame_len = 0;
        if (TR_I2C_Interface::packMessage(serving_cfg_type,
                                           serving_cfg,
                                           serving_cfg_len,
                                           cfg_frame,
                                           sizeof(cfg_frame),
                                           cfg_frame_len))
        {
            if (tx_pos + cfg_frame_len <= FC_COMBINED_READ_SIZE)
            {
                memcpy(tx_buf + tx_pos, cfg_frame, cfg_frame_len);
                tx_pos += cfg_frame_len;
                ESP_LOGI("OC", "I2C TX Config frame type=0x%02X len=%u",
                              (unsigned)serving_cfg_type,
                              (unsigned)cfg_frame_len);
            }
            else
            {
                // #569: this drop used to be SILENT — worse, the success line
                // above printed regardless, so the log claimed a config was
                // sent that never reached the FC. The static_assert by
                // FC_COMBINED_READ_SIZE guards the known types; this is the
                // runtime backstop for anything it misses.
                ESP_LOGE("OC", "I2C TX Config frame type=0x%02X len=%u does NOT "
                              "fit combined read (%u+%u > %u) — DROPPED, never "
                              "reaches FC",
                              (unsigned)serving_cfg_type, (unsigned)cfg_frame_len,
                              (unsigned)tx_pos, (unsigned)cfg_frame_len,
                              (unsigned)FC_COMBINED_READ_SIZE);
            }
        }
        else
        {
            ESP_LOGE("OC", "I2C TX Config pack FAILED type=0x%02X data_len=%u",
                          (unsigned)serving_cfg_type,
                          (unsigned)serving_cfg_len);
        }
    }

    // Non-blocking (timeout=0): if the TX ringbuffer is full, drop this
    // response — the next query cycle will generate a fresh one.
    i2c_interface.writeToSlave(tx_buf, I2C_TX_SIZE, 0);

    // Repeat each command for CMD_REPEAT_LIMIT polls so the FlightComputer
    // has multiple chances to receive it; then clear the serving slot and
    // schedule one idle poll before the next queued command (#366/#368).
    if (cmd != 0U)
    {
        cmd_delivery_count++;
        if (cmd_delivery_count >= CMD_REPEAT_LIMIT)
        {
            ESP_LOGI("OC", "I2C TX Cmd 0x%02X cleared after %u deliveries (queued=%u)",
                          (unsigned)cmd, (unsigned)cmd_delivery_count,
                          (unsigned)cmd_queue_count);
            pending_out_command = 0U;
            serving_cfg_len = 0;
            serving_cfg_type = 0;
            cmd_delivery_count = 0;
            cmd_idle_gap_pending = true;
        }
    }
}

// ==========================================================================
// SECTION: FC OTA relay: I2S direction flip and image feeder
// ==========================================================================
// ---- OTA relay to the Flight Computer (#8 Phase 4) ------------------------
// TR_BLE_To_APP invokes these when an OTA_BEGIN/FINISH/ABORT with target==1
// arrives over BLE. We stage the matching command (+ the 36-byte image header
// for BEGIN) for the FC to pull on its next I2C poll — identical delivery to
// servo/sim config. The FC reports progress back over I2S (OTA_STATUS_MSG),
// relayed to BLE by the frame processor. (Image bytes are Phase 4 Layer 3.)
// OTA relay status dedupe (#8 Phase 4): the FC resends each OTA status ~8x over
// ~1.4s to survive the I2S dedup storm during its flash erase
// (sendOtaRelayStatusRobust on the FC). That FC->OC (I2S) hop is lossy so the
// resend is needed there, but the OC->app (BLE/GATT) hop is reliable — relaying
// every copy just spams the app with identical notifications. Track the last
// relayed status and collapse identical (state,err,bytes) runs into a single
// notify; WRITING progress (Layer 3) still flows because bytes_written advances
// each tick. Reset at session start so a new session's first status always
// relays even if it matches the prior session's terminal status.
static uint8_t  last_relay_state_ = 0xFF;
static uint8_t  last_relay_err_   = 0xFF;
static uint32_t last_relay_bytes_ = 0xFFFFFFFFu;

// --- Phase 4 Layer 3: OC side of the I2S image pump --------------------------
// During an FC OTA the OC flips its I2S slave-RX link (normal FC->OC telemetry)
// to master-TX, pumps the image to the FC as OTA_DATA_CHUNK frames, then reverts
// to slave-RX so the FC's terminal status (READY_TO_BOOT / VERIFY_FAILED) comes
// back over the normal-direction link. i2s_stream is touched by loop_oc (the
// flip/revert) and the BLE host task (ocOtaRelayData pump), so oc_i2s_mutex
// serializes them. Flip is triggered by OTA_RELAY_READY; revert by FINISH/ABORT.
static SemaphoreHandle_t oc_i2s_mutex = nullptr;
static volatile bool oc_ota_tx_mode = false;                // flipped to master TX
static volatile bool oc_ota_await_flip = false;             // READY seen; waiting for the FC to go quiet
static volatile bool oc_ota_relay_ready_pending = false;    // relay "ready" to app once flipped to TX
static volatile bool oc_ota_revert_to_rx_requested = false; // set on FINISH/ABORT staged
// #834 item 6: oc_ota_tx_mode means "deliberately in master TX". It must NOT
// double as "the I2S channel is fine" — the original bug was exactly that
// conflation, because clearing it to stop the feeder also disabled
// ocRevertToRx(), the only route back to slave RX. This is the honest state.
static volatile bool oc_i2s_rx_broken     = false;  // no working slave RX
static uint32_t      oc_i2s_rx_last_try_ms = 0;
// #834 item 7: last relayed chunk, for the stall watchdog. 0 = not yet armed
// (the app has not been told "ready", so silence is expected).
static volatile uint32_t oc_ota_last_chunk_ms = 0;
static uint32_t oc_ota_frames_pumped = 0;                   // diag: frames enqueued by relay cb
static uint32_t oc_ota_feed_sent     = 0;                   // diag: frames the feeder wrote to I2S
static uint32_t oc_ota_feed_idle     = 0;                   // diag: idle-fill writes (queue empty)
static uint32_t oc_ota_silence_ref_count = 0;               // dma_cb_count snapshot for silence detect
static uint32_t oc_ota_silence_since_ms  = 0;               // when RX last went quiet
static uint32_t oc_ota_warmup_since_ms   = 0;               // post-flip RX-lock warmup start (0 = inactive)
static uint32_t oc_ota_total_size = 0;                      // image size (diag)
// OC->FC image pump. The BLE callback enqueues image frames here; a dedicated
// feeder task (ocOtaTxFeederTask) is the SOLE I2S writer while flipped to master
// TX, writing idle-fill in the gaps between BLE chunks. This is essential: I2S
// master clocks continuously, so any gap in writes makes the DMA replay its last
// buffer — the FC then sees a relentless stream of duplicate frames, its RX ring
// overflows (bench 2026-06-01: ring_ovf=1.37M, transfer stalled at 714 B), and
// that corruption shreds the real in-order frames too. This mirrors the FC's own
// i2sSenderTask, whose comment already documents exactly this replay hazard.
struct OcOtaTxFrame { uint8_t payload[MAX_PAYLOAD]; uint16_t len; };
static QueueHandle_t oc_ota_tx_queue    = nullptr;
static TaskHandle_t  oc_ota_feeder_task = nullptr;
// We must not drive BCLK as master until the FC has released it (flipped to its
// own slave RX). The FC keeps the link active (idle-fill) right up until it
// flips, so "I2S RX has gone quiet for this long" is a robust signal that the
// FC is now in slave RX — more reliable than a fixed delay coupled to the FC's
// READY-resend duration. The exact window is a bench item (#15).
static constexpr uint32_t OTA_FLIP_SILENCE_MS = 500;

// After flipping to master TX, idle-fill this long before telling the app it may
// pump real image data, so the FC's slave RX locks onto the just-started master
// BCLK first. Offset 0 pumped into a not-yet-locked RX is lost, and the
// forward-only receiver never recovers (bench 2026-06-02: frames_ok=0, every
// frame "gap", finish err=7). Generous vs the observed ~tens-of-ms lock; the OTA
// itself is ~15 s so the cost is negligible. Bench-tunable (#15).
static constexpr uint32_t OTA_RX_WARMUP_MS = 150;

static bool i2sRecvCallback(const uint8_t* buf, size_t len, void* user_ctx);  // defined below
static void ocOtaRelayClearPendingFlip();                                    // defined below

// Boot and every recovery path must build the RX channel identically — see
// the note in ocBeginSlaveRxLocked().
static constexpr uint32_t kI2sRxDmaDescNum  = 4;
static constexpr uint32_t kI2sRxDmaFrameNum = 128;   // 512 B ≈ 2.9 ms @ 44100

// #834 items 6/7: (re)establish slave RX. Caller holds oc_i2s_mutex. Records
// oc_i2s_rx_broken so loop_oc can keep retrying — losing the RX channel means
// the OC receives no FC telemetry at all, logs nothing and freezes both
// downlinks, so it can never be left as a terminal state.
static esp_err_t ocBeginSlaveRxLocked(const char* why)
{
    // dma_desc_num/dma_frame_num MUST match the boot init: 128 frames (512 B)
    // spans ~2.9 ms at the 44100 link rate, which is the callback cadence the
    // parser was tuned against (#104). The old revert path passed 64 — a
    // leftover from the retired 22050 rate — which doubled the RX ISR cadence
    // for the rest of the power cycle. Recovery must restore the link the
    // parser expects, not a different one.
    esp_err_t e = i2s_stream.beginSlaveRx(config::I2S_BCLK_PIN,
                                          config::I2S_WS_PIN,
                                          config::I2S_DIN_PIN,
                                          config::I2S_FSYNC_PIN,
                                          config::I2S_SAMPLE_RATE,
                                          kI2sRxDmaDescNum, kI2sRxDmaFrameNum);
    if (e == ESP_OK)
    {
        // The callback IS the ingest path — an enabled channel with no callback
        // registered delivers nothing to processFrame, which is the same dead
        // telemetry we are recovering from. Treat its failure as RX broken, or
        // the retry loop below would never run.
        e = i2s_stream.registerRecvCallback(i2sRecvCallback, nullptr);
        if (e != ESP_OK)
            ESP_LOGE("OC", "OTA relay: registerRecvCallback FAILED (%s): %s",
                     why, esp_err_to_name(e));
    }
    if (e == ESP_OK)
    {
        if (oc_i2s_rx_broken)
            ESP_LOGW("OC", "OTA relay: slave RX restored (%s)", why);
        oc_i2s_rx_broken = false;
    }
    else
    {
        oc_i2s_rx_broken = true;
        ESP_LOGE("OC", "OTA relay: slave RX begin FAILED (%s): %s — "
                       "no FC telemetry until this succeeds; retrying",
                 why, esp_err_to_name(e));
    }
    oc_i2s_rx_last_try_ms = (uint32_t)(esp_timer_get_time() / 1000);
    return e;
}

// Flip slave-RX -> master-TX for the image pump. Runs in loop_oc once the FC's
// I2S TX has gone quiet (it has flipped to slave RX), so there's no contention.
static void ocFlipToTx()
{
    if (oc_ota_tx_mode || oc_i2s_mutex == nullptr) return;
    xSemaphoreTake(oc_i2s_mutex, portMAX_DELAY);
    i2s_stream.end();
    const esp_err_t e = i2s_stream.beginMasterTx(config::I2S_BCLK_PIN,
                                                 config::I2S_WS_PIN,
                                                 config::I2S_DIN_PIN,   // former DIN now drives DOUT
                                                 -1,                    // no frame-sync for OTA
                                                 OTA_I2S_SAMPLE_RATE_HZ,
                                                 4, 64);
    oc_ota_tx_mode = (e == ESP_OK);
    if (e != ESP_OK)
    {
        // #834 item 6: end() already deleted the channel, so right now the OC
        // has NO I2S at all. ocRevertToRx() cannot help — it returns early on
        // !oc_ota_tx_mode — so restore slave RX here, before releasing the
        // mutex. Otherwise the OC never hears the FC again this power cycle.
        //
        // end() first: beginMasterTx's later error exits call i2s_del_channel()
        // directly rather than end(), and del_channel frees the driver without
        // detaching the GPIO matrix or clearing output-enable — so BCLK/WS can
        // still be driven from the half-built master config. end() is null-safe
        // and releases those pads (its GPIO block is outside the handle guard).
        i2s_stream.end();
        ocBeginSlaveRxLocked("flip-to-TX failed");
    }
    else
    {
        // #834 item 7: arm the stall watchdog at the FLIP, not when the app is
        // told "ready". The "ready" release happens inside loop_oc's pwr_pin_on
        // gate while the watchdog is serviced outside it, so arming there left
        // a rail-off during the 150 ms warmup flipped with the watchdog asleep.
        oc_ota_last_chunk_ms = (uint32_t)(esp_timer_get_time() / 1000);
    }
    xSemaphoreGive(oc_i2s_mutex);
    ESP_LOGW("OC", "OTA relay: I2S -> master TX for image pump (%s)", esp_err_to_name(e));
    if (e != ESP_OK)
    {
        // Restoring OUR slave RX is only half the link. The FC flipped to slave
        // RX when the OTA began — that silence is what we waited for before
        // flipping — so with our flip failed BOTH ends are slave and nobody
        // drives BCLK: telemetry stays dead despite a healthy channel. Tell the
        // FC to abort so it reverts to master TX. I2C is independent of I2S, so
        // this reaches it even with the I2S link down.
        setPendingCommand(OTA_ABORT_CMD);
        // "aborted" is in the apps' vocabulary; "error" is not, and an
        // unrecognised token leaves the app waiting out its own timeout.
        // terminal=true clears ota_relay_active_/ota_session_active_.
        ble_app.relayFcOtaStatus("aborted", "i2s_flip_failed", 0, true);
        ocOtaRelayClearPendingFlip();
    }
}

// Revert master-TX -> slave-RX (normal telemetry + OTA status path). loop_oc.
static void ocRevertToRx()
{
    if (!oc_ota_tx_mode || oc_i2s_mutex == nullptr) return;
    // Settle before tearing down TX: let the last pumped image frames fully
    // clock out of the DMA to the FC (a few ms at the OTA BCLK), and catch any
    // chunk that arrived just after FINISH (oc_ota_tx_mode is still true here,
    // so a late ocOtaRelayData still pumps). The FC's FINISH handler separately
    // waits for its byte count to reach the total, covering parse lag.
    vTaskDelay(pdMS_TO_TICKS(100));
    xSemaphoreTake(oc_i2s_mutex, portMAX_DELAY);
    i2s_stream.end();
    const esp_err_t e = ocBeginSlaveRxLocked("revert-to-RX");
    // #834 item 6: clear tx_mode either way — the feeder must stop pumping, and
    // the channel it was pumping into is gone regardless. A failure is carried
    // by oc_i2s_rx_broken instead, which loop_oc retries; the old code cleared
    // tx_mode and kept no record, so a failed revert was silent and permanent.
    oc_ota_tx_mode       = false;
    oc_ota_last_chunk_ms = 0;
    xSemaphoreGive(oc_i2s_mutex);
    ESP_LOGW("OC", "OTA relay: I2S -> slave RX (%s)", esp_err_to_name(e));
}

// Pump one relayed BLE chunk to the FC. Splits into <= (MAX_PAYLOAD-4)-byte
// OTA_DATA_CHUNK frames, each prefixed with its absolute offset so the FC writes
// in order and skips stale-repeat offsets from an I2S underrun. BLE host task;
// oc_i2s_mutex serializes vs the loop_oc flip/revert.
static void ocOtaRelayData(void* /*ctx*/, uint32_t offset, const uint8_t* data, size_t len)
{
    if (!oc_ota_tx_mode || oc_ota_tx_queue == nullptr)
    {
        ESP_LOGW("OC", "OTA relay: chunk @%u dropped — I2S not in TX mode", (unsigned)offset);
        return;
    }
    // #834 item 7: the app is alive and pumping — feed the stall watchdog.
    oc_ota_last_chunk_ms = (uint32_t)(esp_timer_get_time() / 1000);
    static constexpr size_t kMaxImg = MAX_PAYLOAD - 4;   // 4-byte offset header
    size_t sent = 0;
    while (sent < len)
    {
        OcOtaTxFrame f;
        const size_t n = (len - sent > kMaxImg) ? kMaxImg : (len - sent);
        const uint32_t off = offset + (uint32_t)sent;
        memcpy(f.payload, &off, 4);
        memcpy(f.payload + 4, data + sent, n);
        f.len = (uint16_t)(4 + n);
        // The very first frame (offset 0) is reliably lost to the I2S master-TX
        // startup transient on the FC's slave RX: bench 2026-06-01 showed the
        // whole 578 KB image streaming in order EXCEPT offset 0, so writeChunk
        // never started and every later frame was rejected as "ahead of have=0".
        // Resend offset 0 a few times back-to-back; the FC writes whichever copy
        // lands first and skips the rest (offset < bytesWritten -> benign gap).
        // Mirrors the #165 critical-one-shot-frame resend. The image streams
        // cleanly after the first frame, so only offset 0 needs this.
        const int copies = (off == 0) ? 5 : 1;
        for (int c = 0; c < copies; ++c)
        {
            // Block (back-pressure BLE) if the feeder is briefly behind; it drains
            // far faster than BLE delivers, so this should not actually wait. The
            // long timeout is only a safety valve against a BLE-host-task hang.
            if (xQueueSend(oc_ota_tx_queue, &f, pdMS_TO_TICKS(1000)) != pdTRUE)
            {
                ESP_LOGW("OC", "OTA relay: TX queue full, dropped frame @%u", (unsigned)off);
                return;
            }
            oc_ota_frames_pumped++;
        }
        sent += n;
    }
}

// Sole I2S writer while flipped to master TX. Drains the image-frame queue and
// writes idle-fill zeros in the gaps so the I2S DMA never replays a stale buffer
// (see OcOtaTxFrame comment). Idles harmlessly outside TX mode. Mirrors the FC's
// i2sSenderTask. The per-iteration mutex serialises against ocFlipToTx /
// ocRevertToRx (which recreate the channel) — the double-checked oc_ota_tx_mode
// guarantees we never writeFrame into a torn-down channel.
static void ocOtaTxFeederTask(void *)
{
    OcOtaTxFrame f;
    for (;;)
    {
        // Back off the bus once a revert is requested so ocRevertToRx() (in loop_oc,
        // LOWER priority) can take oc_i2s_mutex to tear down TX — while we keep
        // re-grabbing it at higher priority the revert never wins (bench: deadlock,
        // idle-filled forever). BUT only back off after the queue is DRAINED: the app
        // sends FINISH right after the last data chunk, so the image tail can still be
        // queued here; backing off immediately stranded it (bench: FC 1 frame short ->
        // SizeMismatch). Drain the tail first, then yield; we just stop idle-filling.
        const bool revert_drained =
            oc_ota_revert_to_rx_requested &&
            (oc_ota_tx_queue == nullptr || uxQueueMessagesWaiting(oc_ota_tx_queue) == 0);
        if (!oc_ota_tx_mode || oc_i2s_mutex == nullptr || revert_drained)
        {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        xSemaphoreTake(oc_i2s_mutex, portMAX_DELAY);
        if (!oc_ota_tx_mode)            // flipped back to RX while we waited for the mutex
        {
            xSemaphoreGive(oc_i2s_mutex);
            continue;
        }
        if (oc_ota_tx_queue != nullptr &&
            xQueueReceive(oc_ota_tx_queue, &f, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            (void)i2s_stream.writeFrame(OTA_DATA_CHUNK, f.payload, f.len);
            oc_ota_feed_sent++;
        }
        else
        {
            (void)i2s_stream.writeIdleFill(64, 1);   // keep BCLK fed; no stale replay
            oc_ota_feed_idle++;
        }
        xSemaphoreGive(oc_i2s_mutex);
        // Diag (~every 512 writes): is the app actually feeding image frames, or are
        // we only emitting idle-fill?  enq==0/sent==0 -> the app's BLE data never
        // reached ocOtaRelayData (routing/app). sent>0 while the FC sees nothing ->
        // loss on the I2S link / FC RX. This is the step neither log showed before.
        if (((oc_ota_feed_sent + oc_ota_feed_idle) & 0x1FFu) == 0u)
            ESP_LOGW("OC", "[OTA feed] enq=%u sent=%u idle=%u qdepth=%u",
                     (unsigned)oc_ota_frames_pumped, (unsigned)oc_ota_feed_sent,
                     (unsigned)oc_ota_feed_idle,
                     (unsigned)(oc_ota_tx_queue ? uxQueueMessagesWaiting(oc_ota_tx_queue) : 0u));
    }
}

// #383: OTA_BEGIN handoff from the NimBLE host task to the loop task —
// the loop task enqueues the header via setPendingCommandWithConfig().
// volatile flag is sufficient: single producer (BLE task), single
// consumer (loop task), and the consumer is idempotent.
static volatile bool ota_begin_stage_pending = false;
static uint32_t ota_begin_total_size_staged = 0;
static uint8_t  ota_begin_sha256_staged[32] = {};

static void ocOtaRelayBegin(void* /*ctx*/, uint32_t total_size, const uint8_t* sha256)
{
    static_assert(sizeof(QueuedCommand::cfg) >= 4 + 32,
                  "QueuedCommand::cfg too small for the OTA_BEGIN_MSG payload");
    last_relay_state_ = 0xFF;
    last_relay_err_   = 0xFF;
    last_relay_bytes_ = 0xFFFFFFFFu;
    oc_ota_frames_pumped          = 0;
    oc_ota_feed_sent              = 0;
    oc_ota_feed_idle              = 0;
    oc_ota_await_flip             = false;
    oc_ota_relay_ready_pending    = false;
    oc_ota_warmup_since_ms        = 0;
    oc_ota_revert_to_rx_requested = false;
    oc_ota_total_size             = total_size;
    // #383: this callback runs on the NimBLE host task. The enqueue itself
    // is task-safe now (#476: payload passed by argument), but keep the
    // loop-task handoff so the OTA state resets and the enqueue happen in
    // loop order; the flag handoff is SPSC like prepare_request_pending_.
    memcpy(ota_begin_sha256_staged, sha256, 32);
    ota_begin_total_size_staged = total_size;
    ota_begin_stage_pending     = true;
    ESP_LOGI("OC", "OTA relay: OTA_BEGIN stashed for loop-task staging (size=%u)",
             (unsigned)total_size);
}
// #834 item 7 (review): clear the PRE-flip arming state. oc_ota_await_flip is
// armed when the FC's OTA_RELAY_READY arrives and was previously cleared only
// by a new relay or by the flip itself — so a teardown during the silence wait
// left it set and loop_oc flipped to master TX afterwards, with no session.
// That became actively harmful once teardown started staging OTA_ABORT_CMD:
// the FC's abort handler calls fcRevertToTx() with no quiet-wait (unlike its
// FINISH path, which spins until the OC goes silent precisely to avoid this),
// so both ends would drive BCLK/WS until the stall watchdog fired ~10 s later.
static void ocOtaRelayClearPendingFlip()
{
    oc_ota_await_flip          = false;
    oc_ota_relay_ready_pending = false;
    oc_ota_warmup_since_ms     = 0;
}

static void ocOtaRelayFinish(void* /*ctx*/)
{
    setPendingCommand(OTA_FINISH_CMD);
    ocOtaRelayClearPendingFlip();
    // Revert to slave-RX (in loop_oc) so we can hear the FC's terminal status,
    // which it sends over the normal-direction I2S after IT reverts to TX.
    oc_ota_revert_to_rx_requested = true;
    ESP_LOGI("OC", "OTA relay: staged OTA_FINISH for FC");
}
static void ocOtaRelayAbort(void* /*ctx*/)
{
    setPendingCommand(OTA_ABORT_CMD);
    ocOtaRelayClearPendingFlip();
    oc_ota_revert_to_rx_requested = true;
    ESP_LOGI("OC", "OTA relay: staged OTA_ABORT for FC");
}

// ==========================================================================
// SECTION: Frame processing: FC -> OC telemetry ingest
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
        case IIS2MDC_MSG:
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
        case GUIDANCE_CONFIG_PENDING:
        case GUIDANCE_CONFIG_MSG:
        case GUIDANCE_POINT_PENDING:  // Drift-Cast aim point (#435) — OC→FC only,
        case GUIDANCE_POINT_MSG:      // listed like the guidance-config pair
        case FIN_CONFIG_PENDING:
        case FIN_CONFIG_MSG:
        case GUIDANCE_ENABLE:
        case GUIDANCE_DISABLE:
        case GUIDANCE_TELEM_MSG:
        case CAMERA_CONFIG_PENDING:
        case CAMERA_CONFIG_MSG:
        case ORIENT_CONFIG_PENDING:
        case ORIENT_CONFIG_MSG:
        case IMU_RATE_CONFIG_PENDING:
        case IMU_RATE_CONFIG_MSG:
        case LORA_MSG:
        case SNAPSHOT_MSG:           // FC→OC over I2S during INFLIGHT
        case GET_FLIGHT_SNAPSHOT:    // FC→OC over I2C at boot recovery
        case MAG_CAL_STATUS_MSG:     // FC→OC over I2S — issue #96
        case SENSOR_CAL_STATUS_MSG:  // FC→OC over I2S — sensor-cal readback (#132)
        case OTA_STATUS_MSG:         // FC→OC over I2S — OTA relay status (#8 Phase 4)
        case FLIGHT_SETTINGS_MSG:    // FC→OC over I2S at flight-start — issue #165
        case FC_IDENTITY:            // FC→OC over I2C — FC firmware version (#8 Phase 4)
        case I2C_TX_RESYNC:          // FC→OC over I2C — slave TX desync recovery (#402)
        case FC_BOOT_STATUS_MSG:     // FC→OC over I2C during setup_fc — boot progress
        case CONFIG_REPORT_MSG:      // FC→OC over I2S — full config report (#915)
        case GNSS_SAT_MSG:           // FC→OC over I2S — per-satellite C/N0 every GNSS epoch, log-only
            return true;
        default:
            return false;
    }
}

// Decode the FC's relayed OTA failure (TR_OTA_Receiver::Error) into a stable
// token for the app.  Every verify failure used to relay as one "fc_error",
// which threw away the only thing that distinguishes a dropped relay chunk
// (size_mismatch — bytes never all arrived) from a corrupted image
// (sha_mismatch — they arrived wrong) from a flash-side refusal (end_failed /
// set_boot_failed).  The FC logs the code over serial, but an assembled rocket
// is exactly the case where serial isn't reachable — so it has to come up the
// BLE link.  Tokens stay machine-stable and fc_-prefixed so they can't be
// confused with the OC's own OTA errors.
static const char* fcOtaErrToken(uint8_t e)
{
    switch ((TR_OTA_Receiver::Error)e)
    {
        case TR_OTA_Receiver::Error::AlreadyActive:    return "fc_already_active";
        case TR_OTA_Receiver::Error::SessionNotActive: return "fc_session_not_active";
        case TR_OTA_Receiver::Error::BeginFailed:      return "fc_begin_failed";
        case TR_OTA_Receiver::Error::BadOffset:        return "fc_bad_offset";
        case TR_OTA_Receiver::Error::SizeOverflow:     return "fc_size_overflow";
        case TR_OTA_Receiver::Error::WriteFailed:      return "fc_write_failed";
        case TR_OTA_Receiver::Error::SizeMismatch:     return "fc_size_mismatch";
        case TR_OTA_Receiver::Error::ShaMismatch:      return "fc_sha_mismatch";
        case TR_OTA_Receiver::Error::EndFailed:        return "fc_end_failed";
        case TR_OTA_Receiver::Error::SetBootFailed:    return "fc_set_boot_failed";
        // Ok-with-VERIFY_FAILED shouldn't happen; report it rather than imply a
        // specific cause, and keep the legacy token for anything unrecognised
        // (an FC newer than this OC could add a code we don't know yet).
        case TR_OTA_Receiver::Error::Ok:               return "fc_error";
    }
    return "fc_error";
}

static void processFrame(const uint8_t* frame, size_t frame_len,
                         uint8_t type, const uint8_t* payload, size_t payload_len)
{
    // Reject unknown message types (CRC false positives from I2S noise)
    if (!isKnownMessageType(type))
        return;

    // FC firmware-version push (#8 Phase 4): a version *string*, not timestamped
    // telemetry, so handle it before the time_us-based dedup below (which would
    // misread the string's first 4 bytes as a timestamp and drop it). Cache it;
    // if it changed, flag a re-publish to the app — an FC OTA never drops our
    // OC<->app BLE link, so the connect-time "fc_identity" won't refresh itself.
    if (type == FC_IDENTITY)
    {
        char incoming[sizeof(fc_fw_version)] = {0};
        size_t n = (payload_len < sizeof(incoming) - 1) ? payload_len
                                                        : sizeof(incoming) - 1;
        if (n > 0) memcpy(incoming, payload, n);
        if (strncmp(incoming, fc_fw_version, sizeof(fc_fw_version)) != 0)
        {
            memcpy(fc_fw_version, incoming, sizeof(fc_fw_version));
            fc_identity_dirty = true;
        }
        return;
    }

    // FC full config report (#915).  Handled here, BEFORE the log enqueue
    // below, so it stays out of the flight log: it is a pre-flight readback,
    // not flight data, and FlightSettingsData already records what actually
    // flew.  Keeping it out also spares the Data_Analysis parsers a message
    // type they hardcode no length for.  Same early-return shape as
    // FC_IDENTITY above.
    if (type == CONFIG_REPORT_MSG)
    {
        if (payload_len >= sizeof(ConfigReportData))
        {
            ConfigReportData incoming;
            memcpy(&incoming, payload, sizeof(incoming));
            if (incoming.version != ConfigReportData::VERSION)
            {
                // An FC newer or older than this OC.  Refuse rather than
                // reinterpret: every field after a layout change would be
                // silently misparsed, and the app would show it as verified.
                static uint8_t warned_version = 0xFF;
                if (warned_version != incoming.version)
                {
                    warned_version = incoming.version;
                    ESP_LOGW("CFG", "Config report version %u unsupported "
                                    "(expect %u) — ignoring",
                             (unsigned)incoming.version,
                             (unsigned)ConfigReportData::VERSION);
                }
                return;
            }
            // time_us changes on every push, so compare everything EXCEPT it —
            // otherwise the 5 s repeat would re-publish to the app forever.
            const bool changed =
                !fc_config_report_valid ||
                memcmp((const uint8_t*)&incoming + sizeof(incoming.time_us),
                       (const uint8_t*)&fc_config_report + sizeof(incoming.time_us),
                       sizeof(incoming) - sizeof(incoming.time_us)) != 0;
            fc_config_report = incoming;
            fc_config_report_valid = true;
            if (changed) fc_config_report_dirty = true;
        }
        return;
    }

    // ── Duplicate, replay & stale frame detection for I2S DMA ──
    // Within a boot session, each FC-side time_us (= micros() on the FC) is
    // strictly monotonically increasing for a given sensor type. So for each
    // type, any incoming frame whose time_us is <= the last-seen value for
    // that type is either an exact duplicate (== prev) or an older replay
    // (< prev). Both cases are safe to drop. This catches I2S DMA buffer
    // replays observed at boot — issue #69 documented ~170 replayed frames
    // covering the first ~70 ms of every flight log (first 65 IMU frames
    // byte-identical to frames 70-134). The cross-type 5-second stale filter
    // remains as a second line of defence against large cross-session leaks.
    if (payload_len >= 4)
    {
        static uint32_t prev_time_ism6 = 0, prev_time_bmp = 0,
                         prev_time_mmc = 0, prev_time_iis2mdc = 0,
                         prev_time_gnss = 0,
                         prev_time_ns = 0, prev_time_pwr = 0,
                         prev_time_guid = 0;
        // dma_cb_count snapshot taken when each prev_time_* was last updated.
        // Used to diagnose issue #74: when a duplicate frame arrives, compare
        // its current dma_cb_count to the prev_cb value to tell whether the
        // duplicate came from the same DMA delivery or a different one.
        static uint32_t prev_cb_ism6 = 0, prev_cb_bmp = 0,
                         prev_cb_mmc = 0, prev_cb_iis2mdc = 0,
                         prev_cb_gnss = 0,
                         prev_cb_ns = 0, prev_cb_pwr = 0,
                         prev_cb_guid = 0;
        static uint32_t max_time_us = 0;  // Monotonic high-water mark across all types
        uint32_t time_us;
        memcpy(&time_us, payload, sizeof(time_us));
        uint32_t* prev = nullptr;
        uint32_t* prev_cb = nullptr;
        switch (type) {
            case ISM6HG256_MSG:       prev = &prev_time_ism6; prev_cb = &prev_cb_ism6; break;
            case BMP585_MSG:          prev = &prev_time_bmp;  prev_cb = &prev_cb_bmp;  break;
            case MMC5983MA_MSG:       prev = &prev_time_mmc;  prev_cb = &prev_cb_mmc;  break;
            case IIS2MDC_MSG:         prev = &prev_time_iis2mdc; prev_cb = &prev_cb_iis2mdc; break;
            case GNSS_MSG:            prev = &prev_time_gnss; prev_cb = &prev_cb_gnss; break;
            case NON_SENSOR_MSG:      prev = &prev_time_ns;   prev_cb = &prev_cb_ns;   break;
            case POWER_MSG:           prev = &prev_time_pwr;  prev_cb = &prev_cb_pwr;  break;
            case GUIDANCE_TELEM_MSG:  prev = &prev_time_guid; prev_cb = &prev_cb_guid; break;
            default: break;
        }
        if (prev != nullptr)
        {
            // FC-reboot detection (#16, hardened in #468). The FC's time_us
            // restarts near 0 on every reboot — including the one right after a
            // successful OTA. All the prev_time_*/max_time_us state here is from
            // the prior session, so every post-reboot frame looks non-monotonic
            // (dropped below) AND stale (dropped by the 5 s filter) until ts
            // climbs back past the old high-water mark — tens of seconds of lost
            // telemetry, so the app shows stale FC data right after an update. A
            // timestamp far behind the global high-water mark can only mean a
            // new session... OR a replayed stale TX descriptor (#468): an FC
            // sender underrun re-transmits 10+ s-old frames once per DMA ring
            // revolution, interleaved with fresh traffic. DedupRebootPolicy
            // tells them apart: only a backstep that PERSISTS (no fresh frame
            // for 100 ms) is a reboot; a lone stale frame between fresh ones is
            // dropped as a replay without touching the baselines. Also covers a
            // uint32 µs wrap (~71 min) gracefully (wrap = sustained backstep).
            static constexpr uint32_t REBOOT_BACKSTEP_US = 10'000'000;  // 10 s
            static DedupRebootPolicy reboot_policy;  // 100 ms > 2 TX ring revolutions
            const bool far_backstep =
                (time_us != 0 && max_time_us > REBOOT_BACKSTEP_US &&
                 time_us < (max_time_us - REBOOT_BACKSTEP_US));
            switch (reboot_policy.onFrame(far_backstep, millis()))
            {
                case DedupRebootPolicy::Action::RESET_BASELINE:
                    ESP_LOGW("DEDUP", "FC reboot confirmed (ts=%lu << max=%lu, sustained); resetting baseline",
                             (unsigned long)time_us, (unsigned long)max_time_us);
                    prev_time_ism6 = prev_time_bmp = prev_time_mmc = prev_time_iis2mdc =
                        prev_time_gnss = prev_time_ns = prev_time_pwr = prev_time_guid = 0;
                    max_time_us = 0;
                    // *prev now reads 0, so this frame passes the checks below as the
                    // first of the new session and re-seeds the baseline.
                    break;
                case DedupRebootPolicy::Action::DROP_REPLAY:
                    dedup_replay_drops++;
                    return;
                case DedupRebootPolicy::Action::PROCEED:
                    break;
            }

            // Non-monotonic or exact-duplicate frame for this type — drop.
            // time_us == 0 is excluded so the very first frame of each type
            // (which finds *prev == 0) still passes.
            if (time_us != 0 && time_us <= *prev)
            {
                const uint32_t cur_cb = dma_cb_count;
                const bool is_eq = (time_us == *prev);
                if (is_eq) dedup_drops_eq++;
                else       dedup_drops_lt++;

                // Log the first N drops of this boot with prev/cur cb values
                // so we can tell whether the duplicate came from the same DMA
                // callback (cur_cb == prev_cb) or a later one. Rate-limited
                // to avoid flooding the console during a stall-triggered burst.
                static uint32_t logged_count = 0;
                if (logged_count < 50)
                {
                    ESP_LOGW("DEDUP", "drop type=%u %s prev_ts=%lu cur_ts=%lu prev_cb=%lu cur_cb=%lu (dcb=%lu)",
                             (unsigned)type,
                             is_eq ? "==" : "<",
                             (unsigned long)*prev,
                             (unsigned long)time_us,
                             (unsigned long)*prev_cb,
                             (unsigned long)cur_cb,
                             (unsigned long)(cur_cb - *prev_cb));
                    logged_count++;
                }
                return;
            }

            // Cross-type stale filter: ts more than 5 s behind the global
            // high-water mark is almost certainly from a prior session.
            // Avoids false positives on minor jitter between sensor streams.
            static constexpr uint32_t STALE_THRESHOLD_US = 5'000'000;
            if (time_us != 0 && max_time_us > STALE_THRESHOLD_US &&
                time_us < (max_time_us - STALE_THRESHOLD_US))
            {
                stale_drops++;
                return;
            }

            *prev = time_us;
            *prev_cb = dma_cb_count;
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
            // #569: commit the new generation atomically w.r.t. loop_oc's
            // snapshot readers (see last_query_cfg_mux).
            portENTER_CRITICAL(&last_query_cfg_mux);
            memcpy(&last_query_cfg, payload, sizeof(OutStatusQueryData));
            portEXIT_CRITICAL(&last_query_cfg_mux);
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
            // Apply board→rocket mounting orientation (format v3+) so the
            // OC's conversions match the FC's.  The quaternion is the
            // authoritative rotation; code/mode are descriptive.
            if (last_query_cfg.format_version >= 3)
            {
                float q[4];
                for (int i = 0; i < 4; i++)
                {
                    q[i] = (float)last_query_cfg.b2r_q[i] / ORIENT_QUAT_WIRE_SCALE;
                }
                float R[9];
                orientQuatToMatrix(q, R);
                sensor_converter.configureBoardToRocket(R);

                // Surface changes to the app (pre-arm orientation display).
                if (last_query_cfg.b2r_code != imu_orient_pub_code ||
                    last_query_cfg.b2r_mode != imu_orient_pub_mode)
                {
                    imu_orient_dirty = true;
                }

                // Self-heal a stored MANUAL setting: if the FC reports a
                // different mapping (e.g. it rebooted and fell back to
                // auto), re-stage the config.  Idempotent — once applied
                // the query reflects MANUAL+code and this goes quiet.
                // Never during flight (the FC refuses mid-flight anyway).
                //
                // #915: skipped entirely once the FC carries its own NVS
                // record.  This heal existed because the FC was the one
                // config group that could not remember its own setting; a
                // firmware that does remember must not have that memory
                // overwritten from here every time it comes up.  The gate is
                // the FC's own provenance bit, not a version number, so an
                // FC that has genuinely never been told still gets healed.
                const bool fc_remembers_orient =
                    fc_config_report_valid &&
                    (fc_config_report.flags &
                     (1U << ConfigReportData::F_ORIENT_FROM_NVS)) != 0U;
                if (!fc_remembers_orient &&
                    cfg_imu_orient != IMU_ORIENT_AUTO &&
                    latest_rocket_state != INFLIGHT &&
                    (last_query_cfg.b2r_mode != ORIENT_MODE_MANUAL ||
                     last_query_cfg.b2r_code != cfg_imu_orient))
                {
                    static uint32_t last_restage_ms = 0;
                    const uint32_t now = millis();
                    if ((now - last_restage_ms) > 5000U && pending_out_command == 0U)
                    {
                        last_restage_ms = now;
                        stageImuOrientConfig();
                        ESP_LOGI("CFG", "Re-staged MANUAL IMU orientation %u to FC",
                                 (unsigned)cfg_imu_orient);
                    }
                }
            }
            // Guidance-target echo (#435, format v5+): surface changes to the
            // app.  seq alone would suffice for cmd-28 results (it bumps on
            // every processed upload); status/rc are compared too as belt-and-
            // braces against a missed frame.
            if (last_query_cfg.format_version >= 5)
            {
                if (last_query_cfg.tgt_seq     != guid_tgt_pub_seq ||
                    last_query_cfg.tgt_status  != guid_tgt_pub_status ||
                    last_query_cfg.tgt_last_rc != guid_tgt_pub_rc)
                {
                    guid_target_dirty = true;
                }
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
    else if (type == IIS2MDC_MSG)
    {
        msg_count_iis2mdc++;
        if (payload_len >= sizeof(IIS2MDCData))
        {
            memcpy(&latest_iis2mdc_raw, payload, sizeof(IIS2MDCData));
            sensor_converter.convertIIS2MDCData(latest_iis2mdc_raw, latest_iis2mdc_si);
            latest_iis2mdc_valid = true;
        }
    }
    else if (type == MAG_CAL_STATUS_MSG)
    {
        // Issue #96: FC→OC mag cal status frame.  Forward verbatim to BLE
        // on the file-ops characteristic with a 0xCA discriminator added
        // by sendMagCalStatus().  Only meaningful on a direct rocket BLE
        // connection — base-station relay drops these (the BS doesn't
        // process I2S frames from a remote rocket) and the cal flow runs
        // only on the connected rocket anyway.
        if (payload_len >= sizeof(MagCalStatusData))
        {
            ble_app.sendMagCalStatus(payload, sizeof(MagCalStatusData));
        }
    }
    else if (type == SENSOR_CAL_STATUS_MSG)
    {
        // Issue #132: FC→OC sensor cal readback.  Forward to BLE on file-ops
        // with a 0xCB discriminator (sibling of mag cal's 0xCA).
        if (payload_len >= sizeof(SensorCalStatusData))
        {
            ble_app.sendSensorCalStatus(payload, sizeof(SensorCalStatusData));
        }
    }
    else if (type == FC_BOOT_STATUS_MSG)
    {
        // FC boot progress during its ~10 s setup_fc, before any NonSensorData
        // exists.  Latch it for the telemetry builder; the app renders it under
        // the state so the operator sees which step is running (and which one it
        // stalled on) instead of an undifferentiated "INITIALIZATION".
        if (payload_len >= sizeof(FcBootStatusData))
        {
            memcpy(&fc_boot_status, payload, sizeof(FcBootStatusData));
            fc_boot_status_valid = true;
            fc_boot_status_rx_ms = millis();
            // A new boot sequence: any NonSensorData seen before this belongs
            // to the PREVIOUS FC session and must not suppress this one.
            fc_ns_since_boot = false;
            ESP_LOGI("OC", "[FC BOOT] step=%u degraded=0x%02X t=%u ms",
                     (unsigned)fc_boot_status.step,
                     (unsigned)fc_boot_status.degraded,
                     (unsigned)fc_boot_status.elapsed_ms);
        }
    }
    else if (type == OTA_STATUS_MSG)
    {
        // #8 Phase 4: FC→OC OTA relay status. Map the FC's state byte to the
        // ota_status JSON vocabulary the app already understands for local OTA,
        // and forward it up over BLE. relayFcOtaStatus() ends the relay session
        // on a terminal state (ready_to_boot / verify_failed / aborted).
        if (payload_len >= sizeof(OtaRelayStatusData))
        {
            OtaRelayStatusData st;
            memcpy(&st, payload, sizeof(st));
            const char* state = "writing";
            const char* err   = nullptr;
            bool terminal     = false;
            switch (st.state)
            {
                case OTA_RELAY_READY:         state = "ready";
                    // FC has erased ota_1 and (after its READY resends) will flip
                    // to slave-RX. Arm the flip: loop_oc watches for the I2S link
                    // to go quiet (FC released BCLK) before flipping us to
                    // master-TX to pump the image (Phase 4 Layer 3). CRITICAL:
                    // do NOT relay "ready" to the app yet. If the app starts
                    // pumping chunks before we're in TX mode, the early chunks
                    // (including offset 0) are dropped and the FC stalls forever
                    // at bytesWritten=0 (every later chunk is a "gap"). loop_oc
                    // relays "ready" only after the flip to TX completes.
                    if (!oc_ota_tx_mode)
                    {
                        oc_ota_await_flip          = true;
                        oc_ota_relay_ready_pending = true;
                        oc_ota_silence_ref_count   = dma_cb_count;
                        oc_ota_silence_since_ms    = (uint32_t)(esp_timer_get_time() / 1000);
                    }
                    break;
                case OTA_RELAY_WRITING:       state = "writing"; break;
                case OTA_RELAY_READY_TO_BOOT: state = "ready_to_boot"; terminal = true; break;
                case OTA_RELAY_VERIFY_FAILED: state = "verify_failed"; err = fcOtaErrToken(st.err); terminal = true; break;
                case OTA_RELAY_ABORTED:       state = "aborted"; terminal = true; break;
                default: break;
            }
            // Dedupe identical consecutive FC resends (see last_relay_* above):
            // keep the per-frame serial log (confirms the resends arrived over
            // I2S) but only re-notify the app when the status actually changes.
            const bool dup = (st.state == last_relay_state_ &&
                              st.err   == last_relay_err_ &&
                              st.bytes_written == last_relay_bytes_);
            // READY is held until the OC has flipped to master TX (relayed from
            // loop_oc post-flip), so the app can't start pumping into a link
            // that isn't receiving yet.
            const bool defer_ready = (st.state == OTA_RELAY_READY);
            ESP_LOGI("OC", "OTA relay: FC status state=%u err=%u bytes=%u%s",
                     (unsigned)st.state, (unsigned)st.err, (unsigned)st.bytes_written,
                     dup ? " (dup, not relayed)" : (defer_ready ? " (ready held for TX flip)" : ""));
            if (!dup && !defer_ready)
            {
                last_relay_state_ = st.state;
                last_relay_err_   = st.err;
                last_relay_bytes_ = st.bytes_written;
                ble_app.relayFcOtaStatus(state, err, st.bytes_written, terminal);
            }
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
            latest_non_sensor_rx_ms = millis();   // #831
            fc_ns_since_boot = true;   // boot sequence over — real state is flowing
            latest_rocket_state = (RocketState)latest_non_sensor.rocket_state;
            // Update the flight-freeze sticky flag whenever the state
            // changes (issue #71).  Safe to call on every frame — the
            // function only flips the bool on INFLIGHT / READY edges.
            updateFreqLockFromState(latest_rocket_state);
            // Drive the per-packet hop state machine off the same edge
            // (issues #40 / #41).  Pure function; idempotent per state.
            updateHopFromState(latest_rocket_state);

            // Latch the moment we entered READY so the slow-rendezvous
            // silence timer has a fair starting point.  Boot-time READY
            // (first frame from FC) also triggers this.
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
            // KF-filtered altitude rate from FlightComputer (or sim equivalent)
            pressure_alt_rate_mps = (float)latest_non_sensor.baro_alt_rate_dmps * 0.1f;
            updateDerivedSpeedFromNonSensor();

            // Adopt the FlightComputer's live guidance_enabled state (carried
            // in pyro_status bit 5) as the source of truth. FC broadcasts this
            // every non-sensor frame, so OUT just tracks it in RAM — no NVS
            // persistence needed (the next boot's first FC frame repopulates
            // it). This used to write NVS inline here, which blocked flash
            // access during BLE file downloads and tripped the task watchdog
            // on CPU 1 when the I2S Parse backlog got large.
            cfg_guidance_en =
                (latest_non_sensor.apogee_flags & NSF2_GUIDANCE_ENABLED) != 0;
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
        flightlogEndFlight();
    }
    else if (type == SNAPSHOT_MSG)
    {
        // FC sent a flight snapshot (10 Hz during INFLIGHT).  Validate
        // and stash the latest in the reserved MRAM region.  Single slot,
        // serialized on the SPI bus mutex — readers (GET_FLIGHT_SNAPSHOT
        // handler below) acquire the same mutex so they never see a
        // partial write.  ~700 us per write on the SPI bus.
        if (payload_len == sizeof(FlightSnapshotData))
        {
            // #383: SNAPSHOT_MSG bypasses the sensor-frame dedup (its payload
            // starts with the magic, not a timestamp — putting it in that
            // switch would misread the constant magic as a ts and drop every
            // snapshot after the first). Guard the MRAM slot here instead: an
            // I2S DMA replay delivers a snapshot a few hundred ms OLDER than
            // the one already stored, and overwriting would hand brownout
            // recovery stale state. Reject slightly-older frames; a frame
            // MUCH older (>10 s behind) can only be a new flight's near-zero
            // clock — accept it and re-seed the baseline.
            uint32_t elapsed_ms = 0;
            memcpy(&elapsed_ms, payload + offsetof(FlightSnapshotData, flight_elapsed_ms),
                   sizeof(elapsed_ms));
            uint8_t snap_state = 0;
            memcpy(&snap_state, payload + offsetof(FlightSnapshotData, rocket_state),
                   sizeof(snap_state));
            static uint32_t last_snap_elapsed_ms = 0;
            constexpr uint32_t NEW_FLIGHT_BACKSTEP_MS = 10'000;
            // #846 SAFETY: a non-INFLIGHT frame is the FC's "clear" — it can
            // only ever DISARM (the FC restores solely on INFLIGHT), so it
            // must never be dropped as a replay. It routinely looks like one:
            // clearFlightSnapshot() sends elapsed = FC uptime (a few seconds,
            // launch_time_millis == 0 at boot), which lands inside the 10 s
            // backstep below a real flight's elapsed. Dropping it left the
            // store holding a genuine INFLIGHT frame that a later FC panic
            // would restore ON THE GROUND with pyro channels re-armed.
            // Elapsed-ms comparisons are meaningless across an FC reboot
            // anyway; state is the only trustworthy discriminator here.
            const bool is_clear = (snap_state != (uint8_t)INFLIGHT);
            const bool stale_replay =
                !is_clear &&
                elapsed_ms < last_snap_elapsed_ms &&
                (last_snap_elapsed_ms - elapsed_ms) <= NEW_FLIGHT_BACKSTEP_MS;
            if (!stale_replay)
            {
                last_snap_elapsed_ms = elapsed_ms;
                // #846: RAM cache first — the store that actually serves
                // GET_FLIGHT_SNAPSHOT now. Same #383 guard protects it.
                if (frame_len == kSnapFrameLen)
                {
                    const uint32_t now_cache = millis();
                    portENTER_CRITICAL(&snapshot_cache_mux);
                    memcpy(snapshot_cache, frame, kSnapFrameLen);
                    snapshot_cache_valid = true;
                    snapshot_cache_ms = now_cache;
                    portEXIT_CRITICAL(&snapshot_cache_mux);
                }
                // Save the full ~224 B wire frame (SOF + type + len + payload + CRC)
                // so the recovery path can hand the bytes straight back to FC.
                // #822: a no-op returning false on a board with no MRAM
                // (V9/V10) — deliberately unlogged here, because this runs at
                // 10 Hz for the whole flight.  initPeripherals() warns once at
                // boot and the GET_FLIGHT_SNAPSHOT handler warns when the FC
                // actually comes asking; those are the two places that matter.
                (void)logger.mramRawWrite(config::SNAPSHOT_REGION_BASE,
                                          frame, frame_len);
            }
        }
    }
    else if (type == GET_FLIGHT_SNAPSHOT)
    {
        // FC is asking for the latest snapshot at boot recovery. #846: serve
        // the RAM cache first — it holds the newest frame whether it arrived
        // live over I2S or was re-seeded at boot from the NAND log stream.
        // The V7/V8 MRAM slot is the fallback (an OC-also-reset on a board
        // whose cache re-seed doesn't run because the MRAM covers it). The FC
        // validates magic/version/state/CRC32/sim on its side, so garbage
        // from either store is rejected there — no validity check needed here.
        uint8_t snap_frame[kSnapFrameLen] = {};
        bool have = false;
        uint32_t cache_age = 0;
        bool had_cache = false;
        portENTER_CRITICAL(&snapshot_cache_mux);
        if (snapshot_cache_valid)
        {
            memcpy(snap_frame, snapshot_cache, kSnapFrameLen);
            cache_age = millis() - snapshot_cache_ms;
            had_cache = true;
        }
        portEXIT_CRITICAL(&snapshot_cache_mux);

        if (had_cache && snapshotServable(snap_frame, cache_age))
        {
            have = true;
            snapshot_served = true;   // #846: consumed — see the boot re-seed
        }
        else if (!had_cache &&
                 logger.mramRawRead(config::SNAPSHOT_REGION_BASE,
                                    snap_frame, sizeof(snap_frame)) &&
                 snapshotServable(snap_frame, 0))
        {
            // V7/V8 fallback. Age is unknown (the slot has no clock), so only
            // the frame's own flight_elapsed_ms bounds it — enough to refuse
            // a slot left INFLIGHT by a flight that has since timed out.
            have = true;
        }
        else if (had_cache)
        {
            ESP_LOGW("OC", "[RECOVERY] cached snapshot not servable (age %lu ms, "
                           "or not an in-flight frame) — not serving it",
                     (unsigned long)cache_age);
        }
        if (have)
        {
            // Non-blocking write — if the TX ringbuffer is full, drop;
            // FC's masterRead retry path will handle it.
            i2c_interface.writeToSlave(snap_frame, sizeof(snap_frame), 0);
        }
        else
        {
            // No cache (nothing received this session, no boot re-seed) and
            // no MRAM. The FC handles the missing reply safely — masterRead
            // fails or the SOF scan finds nothing, it retries (#364), then
            // cold-starts — but from the OC side the request would otherwise
            // vanish without trace. Fires at most once per FC reset per retry.
            ESP_LOGW("OC", "[RECOVERY] FC asked for a flight snapshot and "
                           "none is available (no cached frame, no MRAM) — "
                           "sending no reply");
        }
    }
    else if (type == I2C_TX_RESYNC)
    {
        // #402: the FC detected a slave TX-ring desync (aborted read left
        // residue) and has suspended ALL polling for its grace window — the
        // bus is guaranteed idle, so resetting the slave device here is safe
        // (the #279 constraint).  The next poll after the grace window gets a
        // freshly staged, aligned response.
        ESP_LOGW("OC", "[I2C] RESYNC from FC — resetting slave TX path (#402)");
        const esp_err_t rerr = i2c_interface.resetSlaveTx();
        if (rerr != ESP_OK)
        {
            ESP_LOGE("OC", "[I2C] slave TX reset FAILED: %s", esp_err_to_name(rerr));
        }
    }
    else if (type == GUIDANCE_TELEM_MSG ||
             type == FLIGHT_SETTINGS_MSG ||
             type == LORA_MSG ||
             type == GNSS_SAT_MSG)
    {
        // #569: log-only types — no live handler by design (they exist for
        // the flight log; GUIDANCE_TELEM streams at up to 500 Hz in guided
        // flight, GNSS_SAT at every GNSS epoch). Count them separately so
        // "unknown" stays a real corruption signal.
        msg_count_logonly++;
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
    // Throughput note (2026-07-09 bench): the previous fixed 64-frame yield
    // slept one tick (~10 ms) per 64 frames, hard-capping parse throughput
    // at ~2.1k frames/s.  That was invisibly adequate at the pre-1920 Hz
    // inflow (~2.0k frames/s) but 30% short of the 1920 Hz-logging inflow
    // (~3.0k frames/s): rx_ring pinned at 65535, the DMA ISR dropped
    // ~45 KB/s (rx_ovf), and recordings captured ~69% of frames uniformly.
    // Yield on TIME instead — parse at full speed, and only give up the core
    // for a tick after a sustained slice so IDLE1 can pet the task watchdog
    // and loop_oc (prio 5, same core) gets serviced under overload (#74's
    // original concern).  20 ms slices = >=2/3 duty even when saturated —
    // far above inflow with the batched copies below.
    static constexpr int64_t PARSE_SLICE_US = 20000;
    int64_t slice_t0 = esp_timer_get_time();
    while (rxLen() >= (4 + 1 + 1 + 2))
    {
        if (!(rxPeek(0) == 0xAA &&
              rxPeek(1) == 0x55 &&
              rxPeek(2) == 0xAA &&
              rxPeek(3) == 0x55))
        {
            // Bulk-skip to the next SOF candidate.  ~45% of the raw stream is
            // the master's zero idle-fill; scanning it one rxPop() at a time
            // (with a 4-byte peek per iteration) was a large share of the
            // parser's per-byte cost.  Scan the contiguous span for the next
            // 0xAA and consume the whole gap in one tail advance.  Only the
            // parser advances rx_tail (#383: the ISR never touches it), so
            // reading the span directly is race-free.
            const size_t tail = rx_tail;
            const size_t head = rx_head;
            const size_t contiguous =
                (head >= tail) ? (head - tail) : (RX_STREAM_RING - tail);
            const uint8_t* p = static_cast<const uint8_t*>(
                memchr(rx_ring + tail + 1, 0xAA, contiguous - 1));
            const size_t skip =
                p ? static_cast<size_t>(p - (rx_ring + tail)) : contiguous;
            parser_resync_drops += skip;
            rx_tail = (tail + skip) % RX_STREAM_RING;
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

        // Copy the frame out in at most two contiguous spans (the old
        // per-byte rxPeek loop paid a modulo per byte, twice per frame).
        uint8_t frame[MAX_FRAME];
        {
            const size_t tail = rx_tail;
            const size_t to_end = RX_STREAM_RING - tail;
            const size_t first = (frame_len <= to_end) ? frame_len : to_end;
            memcpy(frame, rx_ring + tail, first);
            if (first < frame_len)
            {
                memcpy(frame + first, rx_ring, frame_len - first);
            }
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

        // Consume the whole frame with one tail advance (was frame_len
        // single-byte rxPop() calls, each with a modulo).
        rx_tail = (rx_tail + frame_len) % RX_STREAM_RING;
        handleReceivedFrame(frame, frame_len, type, payload, out_payload_len);

        if ((esp_timer_get_time() - slice_t0) >= PARSE_SLICE_US)
        {
            vTaskDelay(1);
            slice_t0 = esp_timer_get_time();
        }
    }
}

// ==========================================================================
// SECTION: I2S DMA callback and parser task
// ==========================================================================
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

    dma_cb_count = dma_cb_count + 1;  // volatile: '++' deprecated in C++20 (-Wvolatile)
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

// Peak wall time of a single parseRxStream() call since the last reset
// (#104 follow-up).  If parser falls behind the DMA producer — for example
// because it blocks on the SPI mutex while the flush task runs a slow NAND
// op — we expect to see this spike alongside a high rx_ring_peak_fill and
// non-zero rx_ring_overflow_drops, all in the same window.
static volatile uint32_t parser_iter_max_us = 0;

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
        const int64_t _t0 = esp_timer_get_time();
        parseRxStream();
        const uint32_t _dt = (uint32_t)(esp_timer_get_time() - _t0);
        if (_dt > parser_iter_max_us) parser_iter_max_us = _dt;
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

// ==========================================================================
// SECTION: I2C ingress from the FC
// ==========================================================================
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

// ==========================================================================
// SECTION: LoRa downlink: telemetry payload and beacon
// ==========================================================================
// #850: builds ONE of the two downlink frames. The caller picks the type —
// the TX path from the slot schedule, the debug printer always FAST so its
// dump shows the full picture. out_len reports what actually went in, because
// the two frames are different sizes and the buffer is sized for the larger.
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
    // Hop byte (#40 / #41).  Tells the BS where to expect packet seq+1.
    // With seq-anchored slow-hop (#105 follow-up), this value is purely
    // a sanity hint — the BS computes the same channel from the seq
    // itself, which lets it self-correct after a single missed packet
    // within the dwell window.  Sentinel 0xFF means "not hopping; stay
    // on lora_freq_mhz".
    if (hop_active_ && hop_fallback_state == HopFallbackState::NORMAL)
    {
        const uint8_t n = loraChannelCount(lora_bw_khz);
        if (n == 0)
        {
            lora.next_channel_idx = LORA_NEXT_CH_NO_HOP;
        }
        else
        {
            // Skip-mask aware schedule (cmd-15 noise mask + the #150
            // implicit home-channel skip).  With no valid stored mask the
            // empty mask makes loraHopChannelForSeq degenerate to
            // (seq/dwell) % n.
            //
            // Bootstrap packets (#150) all announce the SCHEDULE ENTRY
            // channel — the channel of the first on-schedule packet,
            // whose seq is this seq plus the remaining bootstrap count —
            // so the BS can decode ANY one of them and park where we
            // will arrive.  On-schedule packets announce seq+1 as always.
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
        // #150 (review): rendezvous visit / coordinated-scan pause — the
        // hop session is live but this frame is off-schedule.  Tell the
        // BS the truth (0xFE) instead of "not hopping" (0xFF): a
        // fixed-mode BS gets immediate mode-mismatch evidence while we
        // are parked HERE and listening — the one window where its
        // cmd-17 push can land — and a lost BS parked on the rendezvous
        // learns the session is still alive.
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
    lora.camera_recording = camera_recording_requested;
    lora.logging_active = logger.isLoggingActive();
    // #835 item 9: relay sim mode over LoRa.  The direct-BLE path already
    // reports it ("fs" bit 8); without this the base station affirmatively
    // reported sim_active=false for a synthetic flight.
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

        // #191: EKF ENU velocity components — the app's ascent landing
        // prediction integrates [vE,vN,vU].  Euler angles + instantaneous
        // speed left the wire in the same change (the BS re-derives them
        // from the quaternion / these components in unpackLoRa), which is
        // what keeps the frame at 65 B ≤ the 66 B the #150 dwell table
        // was validated at.
        lora.vel_e = (float)latest_non_sensor.e_vel / 100.0f;
        lora.vel_n = (float)latest_non_sensor.n_vel / 100.0f;
        lora.vel_u = (float)latest_non_sensor.u_vel / 100.0f;
        lora.burnout_detected = nsFlagSet(latest_non_sensor.flags, NSF_BURNOUT);
    }

    // #390: board→rocket orientation rides flags2 bits 1-7 so the base
    // station can display the mounting without a BLE link to this rocket.
    // Source is last_query_cfg — the LIVE FC status-query cache, refreshed
    // every poll regardless of BLE state.  (NOT imu_orient_pub_*: those
    // latch only on app-BLE-connected paths, so they're absent in the
    // BS-only field case and go stale if the FC re-orients on the pad
    // after the app disconnects — a confidently WRONG display.)
    // Wire mode 0 = "not reported" (also what pre-#390 firmware and a
    // zero-init last_query_cfg produce), so nothing false can render.
    // #569: snapshot so mode+code come from one query generation (the parser
    // can preempt this packer and overwrite last_query_cfg between reads).
    {
    const OutStatusQueryData q = snapshotQueryCfg();
    if (q.format_version < 3)
    {
        lora.imu_orient_mode = LORA2_OMODE_NONE;   // FC not up / pre-v3 FC
        lora.imu_orient_code = 0;
    }
    else if (q.b2r_mode == ORIENT_MODE_AUTO_EXACT)
    {
        lora.imu_orient_mode = LORA2_OMODE_AUTO;
        lora.imu_orient_code = LORA2_ORIENT_CODE_NONE;  // no discrete code
    }
    else
    {
        lora.imu_orient_mode =
            (q.b2r_mode == ORIENT_MODE_MANUAL)    ? LORA2_OMODE_MANUAL :
            (q.b2r_mode == ORIENT_MODE_AUTO_SNAP) ? LORA2_OMODE_AUTO
                                                  : LORA2_OMODE_DEFAULT;
        lora.imu_orient_code = (uint8_t)(q.b2r_code & LORA2_ORIENT_CODE_MASK);
    }
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
    }

    // #386: temp_x10 was transmitted as a hardwired 0.0 degC.  The BMP585
    // measures die temperature alongside pressure and the OC already latches
    // those frames — relay it as the board temperature.
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

        // #303: relay the FC's health verdicts and OR in the battery state (the
        // FC leaves battery N/A — only the OC reads POWERData).  2S-pack
        // thresholds; tunable, and #272 may refine the low-voltage policy.
        uint32_t sh = latest_non_sensor_valid ? latest_non_sensor.sensor_health : 0u;
        lora.sensor_health = shSet(sh, SH_BATT_SHIFT, shBatteryState(power_si.voltage));
    }

    // #281/#278: fold in the OC-owned flight-log storage verdict regardless of
    // power validity, so a full/failing NAND surfaces even before the pack reads.
    // Seed from the bits already assembled above (battery, when power was valid).
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

    // #850: rail currents ride the SLOW frame. Sampled by the OC on its 100 Hz
    // INA230 tick; 0 on boards with no TPS22811 monitor fitted.
    {
        POWERDataSI p{};
        sensor_converter.convertPowerData(latest_power_raw, p);
        lora.cam_current   = p.cam_current;
        lora.servo_current = p.servo_current;
    }

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

    // Honour any pending hop retune as soon as the radio is idle.  The
    // post-TX path below sets hop_needs_retune_ when the previous TX
    // completed, since send() returns when the TX *starts*, not when it
    // finishes — canSend() going true is our "TX done" signal.  Skip
    // during a rendezvous visit; the radio is being managed by
    // serviceHopFallback() in that case.
    if (hop_needs_retune_ && lora_comms.canSend() &&
        hop_fallback_state == HopFallbackState::NORMAL)
    {
        (void)lora_comms.hopToFrequencyMHz(hopTargetFreqMHz());
        hop_needs_retune_ = false;
    }

    // "LoRa off": every transmit path stops here.  Placed AFTER service() and
    // the pending-retune block on purpose — service() is what completes an
    // in-flight TX and drops the radio back into RX, and the retune is how the
    // ON→OFF hop transition gets the radio back onto lora_freq_mhz.  Returning
    // above either of those would strand the radio on a hop channel, muted,
    // with no way for the base station to call it back.
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

    // #850: five FAST frames then one SLOW, so the slow set rides every 6th
    // transmission (3.0 s at LORA_TX_RATE_HZ = 2). Keyed off the SAME seq that
    // goes on the wire, so the base station can predict the type from the seq
    // alone if it ever needs to.
    uint8_t payload[SIZE_OF_LORA_BUDGET] = {0};
    size_t  payload_len = 0;
    const uint8_t frame_type = loraFrameTypeForSlot(lora_tx_seq);
    if (!buildLoRaPayload(payload, lora_tx_seq, frame_type, payload_len))
    {
        return;
    }
    last_lora_tx_ms = now_ms;
    lora_in_rx_mode = false;  // Exiting RX for TX
    if (lora_comms.send(payload, payload_len))
    {
        lora_tx_ok++;

        // Persist the exact bytes that just went on the air as a LORA_MSG
        // (0xF1) record.  #850: that is now 55 B (FAST) or 22 B (SLOW), so
        // this logs payload_len, NOT sizeof(payload) — the buffer is sized for
        // the larger frame and a slow frame would otherwise be recorded with
        // 33 bytes of trailing zeros that every parser would read as data.  The type has always been in the wire format and
        // every decoder already knows it, but nothing had emitted it — so
        // the rocket log carried no evidence the radio had even keyed up,
        // and a lost base-station log meant the whole downlink measurement
        // was gone (2026-08-08 Kaua'i range test).  The frame carries `seq`,
        // so diffing the seq set logged here against the seq column in the
        // base station's lora_*.csv yields true per-packet loss without
        // needing both logs to survive.
        //
        // Logged AFTER send() returns true and BEFORE lora_tx_seq++, so the
        // record holds the seq actually transmitted, and a failed
        // startTransmit() leaves no record — the log is exactly what was
        // radiated, which is what makes the seq diff meaningful.
        //
        // Deliberately no timestamp field: the payload stays byte-identical
        // to what was radiated (MSG_EXPECTED_LEN accepts both frame sizes),
        // and the record's position between IMU frames dates it to
        // ~256 us — better than an OC timestamp could, since esp_timer here
        // is a different clock domain from the FC time_us that every other
        // log record carries.
        //
        // enqueueFrame() drops the frame when no session is open, so this
        // costs nothing off-session; on-session it averages (5x55 + 22)/6 = 50 B
        // plus 8 B of framing at LORA_TX_RATE_HZ (~116 B/s at 2 Hz, down from
        // 146).  Name beacons are still NOT logged — they are a third,
        // variable-length payload and carry no LoRaFrameHeader to dispatch on.
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

        // Advance the seq AFTER a successful send — failed startTransmit()
        // means nothing went over the air, so the BS shouldn't see a gap.
        lora_tx_seq++;

        // Advance hop state and schedule the post-TX retune.  We can't
        // retune here directly (TX is still in progress); the top of
        // the next serviceLoRa iteration will catch it.  Skip during a
        // rendezvous visit — the radio is on rendezvous mode, not in
        // the hop sequence.
        if (hop_active_ && hop_fallback_state == HopFallbackState::NORMAL)
        {
            if (hop_bootstrap_left_ > 0)
            {
                // A bootstrap packet just went out on lora_freq_mhz.
                // Count it down; once it hits zero the retune below
                // steps onto the schedule-entry channel every bootstrap
                // frame announced.
                hop_bootstrap_left_--;
            }
            // Recompute hop_idx_ from the just-incremented seq via the
            // seq-anchored schedule.  This MUST equal the next_channel_idx
            // we put into the packet header in buildLoRaPayload — both
            // sides derive from the same formula so they cannot disagree.
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
// Only sent during READY/PRELAUNCH (suppressed in flight to save airtime).
static uint32_t last_beacon_ms = 0;

static void sendLoRaBeacon()
{
    if (!config::USE_LORA_RADIO) return;
    if (lora_tx_disabled) return;   // "LoRa off" — the beacon is a transmit too

    // Beacon in any state EXCEPT INFLIGHT.  We used to gate on
    // READY/PRELAUNCH only, but that meant a rocket whose FlightComputer
    // was slow to report state (or never did, e.g. I2C hiccup) would
    // never advertise itself — leaving the base station's recovery
    // scanning forever (issue #71 field test).  INITIALIZATION beaconing
    // costs ~25 ms TOA every 2 s = < 2% duty, so the trade is trivial.
    // INFLIGHT is still suppressed: telemetry needs the airtime, and
    // we don't change channel mid-flight anyway.  Logic shared with
    // tests via shouldBeaconInState() in RocketComputerTypes.h.
    if (!shouldBeaconInState(latest_rocket_state)) return;

    // #150: no beacons while the hop schedule is running.  They would ride
    // hop channels the BS already follows (it knows this rocket by then),
    // and a beacon inside a dwell visit can blow the FCC occupancy budget
    // (at SF10/BW250 a single beacon pushes the visit ~50% over — see the
    // BeaconSuppressionDuringHopIsLoadBearing host test).  Rendezvous
    // visits and scan pauses still beacon — that's their purpose.
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
// SECTION: Config readback to the app
// ==========================================================================

// #398 item 3: config-readback used to send its 4 frames inline with a
// delay(50) between each — a ~235 ms loop_oc stall per readback (pad-only
// today, but a mid-flight app readback would stall telemetry that long).
// Instead the senders enqueue frames here and loop_oc drains ONE per pass,
// paced >= the 7.5-20 ms BLE connection interval so notify_data (which already
// retries on mbuf exhaustion) rarely has to. Enqueue and drain both run in
// loop_oc, so the ring needs no locking. sendConfigJSON no-ops when
// disconnected, so any frames still queued at disconnect drain out harmlessly.
// MUST exceed the connect burst, which sendCurrentConfig() enqueues in ONE
// synchronous pass before any drain runs: config, config_pyro,
// config_identity, fc_identity, imu_orient, guid_target, and the three #915
// report frames = 9.  At the old cap of 8 the ninth enqueue evicted the
// oldest — the main "config" frame — and the app silently never received it.
// Raise this WITH the burst, or the drop is invisible except for one warning
// line on a console nobody is watching on the pad.
static constexpr uint8_t  CFG_RB_CAP     = 12;
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
        // Backed up (app hammering readback faster than the pace). Drop the
        // oldest so the freshest snapshot still gets through.
        cfg_rb_head = (cfg_rb_head + 1) % CFG_RB_CAP;
        cfg_rb_count--;
        ESP_LOGW("CFG", "readback queue full — dropped oldest frame");
    }
    cfg_rb_queue[(cfg_rb_head + cfg_rb_count) % CFG_RB_CAP] = json;
    cfg_rb_count++;
}

// Drained once per loop_oc pass. Sends at most one frame per CFG_RB_PACE_MS so
// the loop never blocks on BLE backpressure.
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

// Publish the FC's relayed firmware version to the app as a compact "fc_identity"
// config message (#8 Phase 4). Kept separate from "config_identity" so it stays
// well under the BLE notify MTU (sendConfigJSON silently drops anything over
// MTU-3). Sent on connect (from sendCurrentConfig) and re-sent whenever the FC
// version changes mid-connection (after an FC OTA), so the app verifies the
// update / spots a rollback against the FC's own version, not the OC's.
static void sendFcIdentity()
{
    const char* fc_fw = fc_fw_version[0] ? fc_fw_version : "unknown";
    char buf[80];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"fc_identity\",\"fc_fw\":\"%s\"}", fc_fw);
    enqueueConfigReadback(String(buf));   // #398 item 3
    ESP_LOGI("CFG", "Queued fc_identity (fc_fw=%s)", fc_fw);
}

// Publish the FC's active board→rocket mounting orientation as its own
// compact config message.  Lets the app show "nose = -Z (auto)" before
// arming so a wrong auto-detect is caught while the rocket is still on
// the ground.  Sent on connect and whenever the FC reports a change
// (auto-detect re-orients on the pad; the status query repeats every
// poll, so changes surface within a cycle).  Kept tiny — sendConfigJSON
// drops anything over the BLE notify MTU.
static void sendImuOrientation()
{
    // #569: snapshot so code/mode/name come from ONE query generation (the
    // parser can preempt this function and overwrite last_query_cfg mid-read).
    const OutStatusQueryData q = snapshotQueryCfg();
    if (q.format_version < 3) return;  // pre-orientation FC
    // "set" is the user's SETTING (0xFF auto / manual code), distinct from
    // code/mode/name which describe what the FC is actively applying.
    // #915: "set" is the VEHICLE's setting when the FC has one of its own.
    // Reporting the OC's cache unconditionally was a lie waiting to happen —
    // reflash the OC alone and it would tell the app AUTO while the FC flew a
    // manual clocking out of its own NVS.
    const bool fc_remembers =
        fc_config_report_valid &&
        (fc_config_report.flags & (1U << ConfigReportData::F_ORIENT_FROM_NVS)) != 0U;
    const uint8_t reported_set =
        fc_remembers ? fc_config_report.imu_orient_setting : cfg_imu_orient;
    char buf[112];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"imu_orient\",\"code\":%u,\"mode\":%u,\"name\":\"%s\",\"set\":%u}",
             (unsigned)q.b2r_code,
             (unsigned)q.b2r_mode,
             orientCodeName(q.b2r_code),
             (unsigned)reported_set);
    enqueueConfigReadback(String(buf));   // #398 item 3
    imu_orient_pub_code = q.b2r_code;
    imu_orient_pub_mode = q.b2r_mode;
    ESP_LOGI("CFG", "Queued imu_orient (%s, mode %u)",
             orientCodeName(q.b2r_code),
             (unsigned)q.b2r_mode);
}

// Publish the FC's guidance-target echo (#435) as its own compact config
// message.  This is the app's cmd-28 send confirmation: DriftCast captures
// the seq baseline before sending and confirms on seq-advance + rc + a
// lat/lon tolerance match.  Kept tiny per the fc_identity/imu_orient
// pattern — sendConfigJSON silently drops frames over MTU-3, so this must
// NEVER be folded into the big "config" frame.  6-decimal degrees keep the
// f32 echo's full precision (~0.11 m/digit at the equator).
static void sendGuidTarget()
{
    // #569: snapshot — seq/status/rc/lat/lon/alt must all come from the same
    // query generation, or a preempting parser memcpy could pair a new seq
    // with an old point (the app confirms cmd-28 sends on exactly this tuple).
    const OutStatusQueryData q = snapshotQueryCfg();
    if (q.format_version < 5) return;               // pre-#435 FC — the app
                                                    // treats absence as
                                                    // unsupported firmware
    char buf[144];
    snprintf(buf, sizeof(buf),
             "{\"type\":\"guid_target\",\"seq\":%u,\"st\":%u,\"rc\":%u,"
             "\"lat\":%.6f,\"lon\":%.6f,\"alt\":%d}",
             (unsigned)q.tgt_seq,
             (unsigned)q.tgt_status,
             (unsigned)q.tgt_last_rc,
             (double)q.tgt_lat_deg,
             (double)q.tgt_lon_deg,
             (int)q.tgt_alt_m);
    enqueueConfigReadback(String(buf));   // #398 item 3: paced drain
    guid_tgt_pub_seq    = q.tgt_seq;
    guid_tgt_pub_status = q.tgt_status;
    guid_tgt_pub_rc     = q.tgt_last_rc;
    ESP_LOGI("CFG", "Queued guid_target (seq=%u st=%u rc=%u)",
             (unsigned)q.tgt_seq,
             (unsigned)q.tgt_status,
             (unsigned)q.tgt_last_rc);
}

// #915: the settings the "config" frame never carried, published from the
// FC's own report so the app can SHOW and VERIFY them instead of displaying
// profile values it has no way to check.  Split across three frames for the
// same reason everything else here is kept small — sendConfigJSON silently
// drops anything over the BLE notify MTU, so a single fat frame would vanish
// without a trace.
//
// Emitted only once a report has landed.  Absence is the app's signal that
// this rocket cannot report them, which is exactly what pre-#915 firmware
// looks like — so the "can't verify" line stays correct on an old rocket
// instead of the app silently believing stale defaults.
static void sendConfigExtras()
{
    if (!fc_config_report_valid) return;
    const ConfigReportData &r = fc_config_report;

    // 1) Servo trim 2-4, fin travel, fin layout, sounds.
    String j = "{\"type\":\"config_servo\"";
    j += ",\"sb2\":"; j += itos(r.servo.bias_us[1]);
    j += ",\"sb3\":"; j += itos(r.servo.bias_us[2]);
    j += ",\"sb4\":"; j += itos(r.servo.bias_us[3]);
    j += ",\"fmn\":"; j += fmtf(r.servo.fin_min_deg, 2);
    j += ",\"fmx\":"; j += fmtf(r.servo.fin_max_deg, 2);
    j += ",\"faz\":[";
    for (int i = 0; i < 4; ++i) {
        if (i) j += ",";
        j += fmtf(r.fin.azimuth_deg[i], 1);
    }
    j += "]";
    j += ",\"frv\":";  j += itos(r.fin.reverse_mask);
    j += ",\"frrv\":"; j += itos(r.fin.roll_reverse_mask);
    j += ",\"snd\":";
    j += (r.flags & (1U << ConfigReportData::F_SOUNDS)) ? "true" : "false";
    j += "}";
    enqueueConfigReadback(j);
    ESP_LOGI("CFG", "Queued config_servo readback (%u bytes)", (unsigned)j.length());

    // 2) The PN / station-keep parameters behind the guidance on/off flag.
    String g = "{\"type\":\"config_guid\"";
    g += ",\"gng\":"; g += fmtf(r.guidance.nav_gain, 2);
    g += ",\"gma\":"; g += fmtf(r.guidance.max_accel_mps2, 1);
    g += ",\"gaf\":"; g += fmtf(r.guidance.accel_to_fin_deg, 2);
    g += ",\"gmf\":"; g += fmtf(r.guidance.max_fin_deg, 1);
    g += ",\"gms\":"; g += fmtf(r.guidance.min_speed_mps, 1);
    g += ",\"gcd\":"; g += itos(r.guidance.coast_delay_ms);
    g += ",\"gtm\":"; g += itos(r.guidance.target_mode);
    g += ",\"gte\":"; g += fmtf(r.guidance.target_e_m, 1);
    g += ",\"gtn\":"; g += fmtf(r.guidance.target_n_m, 1);
    g += ",\"gta\":"; g += fmtf(r.guidance.target_alt_m, 1);
    g += ",\"gkp\":"; g += fmtf(r.guidance.kp_pos_per_s2, 2);
    g += ",\"gkd\":"; g += fmtf(r.guidance.kd_vel_per_s, 2);
    g += ",\"glw\":"; g += itos(r.guidance.guidance_law);
    g += "}";
    enqueueConfigReadback(g);
    ESP_LOGI("CFG", "Queued config_guid readback (%u bytes)", (unsigned)g.length());

    // 3) The roll profile.  num_waypoints == 0 is a real answer ("rate-only"),
    // not an absent one — the app must be able to tell that from "this rocket
    // doesn't report waypoints", which is why the frame is sent either way.
    String w = "{\"type\":\"config_roll\",\"n\":";
    uint8_t n = r.roll.num_waypoints;
    if (n > MAX_ROLL_WAYPOINTS) n = MAX_ROLL_WAYPOINTS;   // corrupt/forward record
    w += itos(n);
    w += ",\"wp\":[";
    for (uint8_t i = 0; i < n; ++i) {
        if (i) w += ",";
        w += "["; w += fmtf(r.roll.waypoints[i].time_s, 2);
        w += ",";  w += fmtf(r.roll.waypoints[i].angle_deg, 1);
        w += "]";
    }
    w += "]}";
    enqueueConfigReadback(w);
    ESP_LOGI("CFG", "Queued config_roll readback (%u bytes, %u wp)",
             (unsigned)w.length(), (unsigned)n);
}

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
    // Control-authority speed gate, m/s (0 = off, time delay only).
    j += ",\"rmspd\":"; j += fmtf(cfg_roll_min_speed, 1);
    // #253: roll-control gain readback (sentinels — rcap/kpang <=0, iwind <0 —
    // mean "firmware default"; the app keeps its local value in that case).
    j += ",\"rcap\":";  j += fmtf(cfg_rate_cap_dps, 1);
    j += ",\"kpang\":"; j += fmtf(cfg_kp_angle, 2);
    j += ",\"iwind\":"; j += fmtf(cfg_iwind_dps, 1);
    j += ",\"ge\":";  j += cfg_guidance_en ? "true" : "false";
    j += ",\"camt\":"; j += itos(cfg_camera_type);
    j += ",\"irate\":"; j += itos(cfg_imu_rate);
    // LoRa settings
    j += ",\"lf\":";  j += fmtf(lora_freq_mhz, 1);
    j += ",\"lsf\":"; j += itos(lora_sf);
    j += ",\"lbw\":"; j += fmtf(lora_bw_khz, 0);
    j += ",\"lcr\":"; j += itos(lora_cr);
    j += ",\"lpw\":"; j += itos(lora_tx_power);
    j += ",\"lhd\":"; j += lora_hop_disabled ? "true" : "false";  // #106
    // "LoRa off" mute.  Both apps decode it lenient-optional, so absence
    // means "firmware predates the feature", not "transmitting".
    j += ",\"ltxd\":"; j += lora_tx_disabled ? "true" : "false";
    // #150: airtime-derived hop dwell for the current preset; 0 tells the
    // app hopping is unavailable at this modulation (option greys out).
    j += ",\"lhdw\":"; j += itos(currentHopDwell());
    j += "}";
    enqueueConfigReadback(j);   // #398 item 3: paced drain in loop_oc, no delay()
    ESP_LOGI("CFG", "Queued config readback (%u bytes)", (unsigned)j.length());

    // Message 2: pyro config ("config_pyro" type) — 4 channels
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

    // Message 3: device identity ("config_identity" type)
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

    // Also push the relayed FC firmware version as its own small message (#8),
    // then the FC's board→rocket mounting orientation (pre-arm display), then
    // the guidance-target echo (#435 — covers connect AND cmd 20).  All
    // enqueue too, so the whole readback drains from loop_oc without blocking.
    sendFcIdentity();
    sendImuOrientation();
    sendGuidTarget();
    sendConfigExtras();   // #915
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
    // Accept the legacy 16-byte payload (an app built before the speed gate)
    // as well as the full struct; the missing tail means "gate off".  memcpy
    // reads whole fields, so zero-fill first and copy only what arrived.
    if (len < ROLL_CTRL_CONFIG_LEN_V1) return;
    RollControlConfigData rc = {};
    memcpy(&rc, payload, (len < sizeof(rc)) ? len : sizeof(rc));
    cfg_use_angle_ctrl = (rc.use_angle_control != 0);
    cfg_roll_delay_ms  = rc.roll_delay_ms;
    // Mirror the FC's apply rule so the readback echoes what the FC kept: a
    // negative or out-of-range value is garbage and leaves the cache alone.
    if (rc.roll_min_speed_mps >= 0.0f && rc.roll_min_speed_mps <= 300.0f)
        cfg_roll_min_speed = rc.roll_min_speed_mps;
    // #253: cache the gains too so the config readback echoes what was pushed
    // (sentinels — cap/kp <=0, iwind <0 — mean "firmware default", matching
    // the FC's apply semantics).
    cfg_rate_cap_dps = rc.kp_angle_rate_cap_dps;
    cfg_kp_angle     = rc.kp_angle;
    cfg_iwind_dps    = rc.integral_sep_threshold_dps;
    prefs.begin("roll", false);
    prefs.putBool("ac", cfg_use_angle_ctrl);
    prefs.putUShort("rdly", cfg_roll_delay_ms);
    prefs.putFloat("rmspd", cfg_roll_min_speed);
    prefs.putFloat("rcap", cfg_rate_cap_dps);
    prefs.putFloat("kpang", cfg_kp_angle);
    prefs.putFloat("iwind", cfg_iwind_dps);
    prefs.end();
    ESP_LOGI("CFG", "Roll control cached: angle_ctrl=%s delay=%u ms minspd=%.1f rcap=%.1f kpang=%.2f iwind=%.1f",
        cfg_use_angle_ctrl ? "ON" : "OFF", (unsigned)cfg_roll_delay_ms,
        (double)cfg_roll_min_speed,
        (double)cfg_rate_cap_dps, (double)cfg_kp_angle, (double)cfg_iwind_dps);
}

// Apply the "LoRa off" transmit mute (BLE cmd 68 / uplink cmd 68 both land
// here so the two transports cannot diverge).  Desired-state, not a toggle:
// `want_disabled` is what the operator asked for, and a repeat of the same
// value is a no-op rather than an inversion — a retried uplink must not
// silence a rocket the first copy already un-silenced.
//
// Returns true when the request was honoured (including the already-there
// case); false when it was refused because the rocket is airborne.
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

    // Re-evaluate the hop schedule against the new mute: muting takes the
    // ON→OFF branch (radio back to lora_freq_mhz, rendezvous visit unwound),
    // un-muting restarts hopping if the rocket's state and link mode want it.
    updateHopFromState(latest_rocket_state);

    // Echo to a directly-connected app.  Matters most on the UPLINK path: the
    // base station can un-mute a rocket the phone is also connected to, and
    // without this the app's toggle would keep claiming the radio is off.
    // No-ops harmlessly when nothing is connected.
    sendCurrentConfig();

    ESP_LOGW("OC", "[LORA] Transmit %s (%s) — %s",
             lora_tx_disabled ? "MUTED" : "UNMUTED", src,
             lora_tx_disabled
                 ? "no telemetry, no beacon, no hopping; still listening for uplink"
                 : "telemetry and beacon resume");
    return true;
}

// ==========================================================================
// SECTION: LoRa uplink command handling
// ==========================================================================

static void processUplinkCommand(uint8_t cmd, const uint8_t* payload, size_t payload_len)
{
    ESP_LOGI("LORA", "UPLINK RX cmd=%u payload_len=%u", cmd, (unsigned)payload_len);

    // #383: the FC skips I2C polls during INFLIGHT, so camera/logging
    // commands are undeliverable mid-flight. The old path flipped
    // camera_recording_requested immediately — the downlink then reported a
    // state change that never happened, and the stale command fired on the
    // ground after landing. Refuse honestly instead (per design decision);
    // the operator re-sends after landing if still wanted.
    // cmd 28 (#435) joins the list: a guidance point queued mid-flight would
    // be delivered after landing and refused by the FC's state gate — but the
    // LoRa path has no echo (#285, blind fire-and-retry), so refusing here is
    // the only honest answer.
    // cmds 35/36 (pyro cont test / TEST-FIRE via BS relay) join too: queued
    // mid-flight they would deliver at landing — a delayed FIRE pulse (or a
    // cont test's momentary ARM) firing on the ground while the recovery crew
    // walks up is exactly the stale-command hazard this gate refuses. The
    // FC's own lockout gate is the second layer, not a reason to skip this.
    if ((cmd == 1 || cmd == 23 || cmd == 28 || cmd == 35 || cmd == 36) &&
        latest_rocket_state == INFLIGHT)
    {
        uplink_inflight_refusals++;
        ESP_LOGW("LORA", "UPLINK cmd=%u refused: rocket INFLIGHT (undeliverable"
                         " until landing; %lu refused so far)",
                 cmd, (unsigned long)uplink_inflight_refusals);
        return;
    }

    if (cmd == 1)
    {
        // Camera: payload[0] = desired state (1 = on, 0 = off).
        // Payload makes retries idempotent (won't toggle back and forth).
        // Falls back to toggle if no payload (legacy compat).
        bool want_on = (payload_len >= 1) ? (payload[0] != 0)
                                          : !camera_recording_requested;
        if (want_on != camera_recording_requested || !camera_state_known)
        {
            camera_state_known = true;   // #825: see the BLE cmd-1 twin
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
            // Mirror the BLE cmd 23 start path: prepareLogFile opens the sink
            // session (file_open=true so enqueueFrame accepts frames) and
            // flightlogBeginFlight allocates the TR_FlightLog block range.
            // startLogging alone would flip logging_active without either, so
            // every incoming frame would be rejected at enqueue and the flight
            // would never land on NAND (#72).
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
    else if (cmd == 24 && payload_len >= sizeof(ServoTestAnglesData))
    {
        // Servo test angles: relay to FlightComputer via I2C
                setPendingCommandWithConfig(SERVO_TEST_PENDING, SERVO_TEST_MSG, payload, sizeof(ServoTestAnglesData));
        ESP_LOGI("LORA", "UPLINK Servo test angles queued");
    }
    else if (cmd == 25)
    {
        // Servo test stop
        setPendingCommand(SERVO_TEST_STOP);
        ESP_LOGI("LORA", "UPLINK Servo test stop");
    }
    else if (cmd == 35 && payload_len >= 1)
    {
        // Pyro continuity test via BS relay — same handling as BLE cmd 35.
        // No duplicate suppression: the BS retry train just re-runs the
        // momentary arm→read→disarm, which keeps the reading fresh.
        uint8_t ch = payload[0];
        if (ch < 1 || ch > 4) {
            ESP_LOGW("LORA", "UPLINK Pyro continuity test: invalid channel %u", ch);
        } else if (!pwr_pin_on) {
            // Rail-off refusal, same as the cmd 36 branch below: the #366
            // queue would HOLD this and deliver the momentary ARM pulse at
            // the next power-on. Like all LoRa refusals this can only log —
            // there is no uplink feedback channel (#285 blind fire-and-retry).
            ESP_LOGW("LORA", "UPLINK Pyro continuity test CH%u refused: FC rail"
                             " off (queued arm pulse would deliver at power-on)", ch);
        } else {
            setPendingCommandWithConfig(PYRO_CONT_TEST, PYRO_CONT_TEST, &ch, 1);
            ESP_LOGI("LORA", "UPLINK Pyro continuity test CH%u", ch);
        }
    }
    else if (cmd == 36 && payload_len >= 1)
    {
        // Pyro TEST-FIRE via BS relay — the LoRa half of the stand-back pyro
        // test (app → BS cmd 50 → here), so the operator can keep LoRa
        // distance from a live charge instead of BLE distance.
        //
        // Rail-off refusal: the FC command queue HOLDS while the rail is off
        // and drains on power-up (#366) — a fire queued now would pulse the
        // channel whenever someone next powers the FC, minutes or hours
        // later, with no telemetry telling the LoRa operator the rail was
        // even off. Same undeliverable-command philosophy as the #383
        // INFLIGHT gate above: refuse honestly, don't queue a latent fire.
        //
        // Dedup: the uplink is blind fire-and-retry with no sequence number,
        // so one FIRE tap arrives as up to UPLINK_RETRIES identical packets,
        // and each would queue a full ARM→pulse→disarm. Worst-case train
        // span is ~11 s: 7 inter-retry gaps × (100 ms pacing + the TX-window
        // gate's 1.5 s max_defer, which re-arms per attempt — bs_uplink_
        // txwin.h / serviceUplink). 13 s covers that with margin while
        // staying under the fastest deliberate re-test of the same channel
        // (5 s recording + 10 s countdown ≈ 15 s away at minimum).
        static uint32_t last_fire_uplink_ms = 0;
        static uint8_t  last_fire_uplink_ch = 0;
        constexpr uint32_t kFireDedupWindowMs = 13000;
        uint8_t ch = payload[0];
        if (ch < 1 || ch > 4) {
            ESP_LOGW("LORA", "UPLINK Pyro test fire: invalid channel %u", ch);
        } else if (!pwr_pin_on) {
            ESP_LOGW("LORA", "UPLINK Pyro test fire CH%u refused: FC rail off "
                             "(queued fire would deliver at next power-on)", ch);
        } else if (ch == last_fire_uplink_ch && last_fire_uplink_ms != 0 &&
                   (millis() - last_fire_uplink_ms) < kFireDedupWindowMs) {
            ESP_LOGI("LORA", "UPLINK Pyro test fire CH%u: duplicate retry ignored", ch);
        } else {
            last_fire_uplink_ch = ch;
            last_fire_uplink_ms = millis();
            setPendingCommandWithConfig(PYRO_FIRE_TEST, PYRO_FIRE_TEST, &ch, 1);
            ESP_LOGI("LORA", "UPLINK Pyro test fire CH%u", ch);
        }
    }
    else if (cmd == 28 && payload_len >= sizeof(GuidancePointData))
    {
        // Drift-Cast guidance point (#435) relayed via base station (BS cmd
        // 50 wrap).  Frame math: uplink [0xCA][nid][rid][ch][cmd][len] + 20 B
        // payload = 26 B, exactly inside the 32-byte RX buffer — this is why
        // cmd 28 is uplink-viable where the 45-byte GuidanceConfigData is not
        // (#383 note above).  NO acceptance feedback reaches the BS/app over
        // LoRa; operators must verify by direct connection before flight.
        setPendingCommandWithConfig(GUIDANCE_POINT_PENDING, GUIDANCE_POINT_MSG,
                                    payload, sizeof(GuidancePointData));
        ESP_LOGI("LORA", "UPLINK Guidance point queued");
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
                setPendingCommandWithConfig(SIM_CONFIG_PENDING, SIM_CONFIG_MSG, &sim_cfg, sizeof(sim_cfg));
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
        flightlogEndFlight();
        setPendingCommand(SIM_STOP_CMD);
        ESP_LOGI("LORA", "UPLINK Sim stop queued for FlightComputer (logging ended)");
    }
    else if (cmd == 10 && payload_len >= 11)
    {
        // LoRa reconfiguration via uplink: [freq:4f][bw:4f][sf:1][cr:1][txpwr:1]

        // Reject reconfigures while the link is "committed" — either
        // freq-locked for flight (issue #71) or actively hopping
        // (#40 / #41).  Either case, changing modulation underneath the
        // hop state machine would desynchronise the channel set on the
        // two sides; the user must drop back to READY first.
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
            // BW change invalidates the channel-set skip-mask (#40/#41
            // phase 3): the mask is sized for the OLD hop table.  The
            // BS will re-push after its own state settles + a new scan.
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

            // #569: reconfigure() leaves the radio in STANDBY (it clears
            // rx_mode_ internally and does not re-enter RX). Every other
            // reconfigure site (cmd 16, rendezvous, hop visit) follows with
            // startReceive(); this one didn't — and serviceLoRaUplink's
            // rx-flag sync only ever sets lora_in_rx_mode TRUE, so the stale
            // true suppressed its startReceive() recovery. The OC sat deaf to
            // uplinks on the NEW modulation until the next telemetry TX
            // re-armed RX (~0.5–2 s), silently missing any follow-up ground
            // command in that window.
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
        // BS-controlled hop enable/disable (#106).  Payload byte 0 = 1 to
        // disable hopping (fixed-frequency mode), 0 to re-enable.  We
        // honour the change immediately by re-evaluating the hop state
        // against the rocket's last known state — this gracefully turns
        // hopping off mid-PRELAUNCH or back on once the operator clears
        // the override.  Persist to NVS so the setting survives reboot.
        const bool new_disabled = (payload[0] != 0);
        if (!new_disabled && currentHopDwell() == 0)
        {
            // #150: this modulation can't fit one packet inside the FCC
            // dwell budget — refuse the enable so we cannot be commanded
            // into a non-compliant schedule.  The BS applies the same
            // gate (and the app greys the option via "lhdw"==0), so
            // reaching here means the two ends disagree on sf/bw; the
            // cmd-10 config sync heals that, after which the enable can
            // be retried.
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
                // #150 bench finding: don't activate while the BS is
                // still blasting its cmd-17 mirror retries (it's deaf
                // mid-TX and would miss our bootstrap, costing ~60 s of
                // fallback healing).  The poll next to
                // serviceHopFallback() applies the enable after the
                // train finishes.
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
        // "LoRa off" over the air.  The only genuinely useful
        // direction here is UN-muting — a muted rocket keeps listening
        // precisely so the base station can call it back — but the command is
        // symmetric so the app's toggle behaves the same whether it is talking
        // to the rocket directly or relaying through the BS.
        (void)applyLoRaTxMute(payload[0] != 0, "UPLINK");
    }
    else if (cmd == 12 && payload_len >= sizeof(ServoConfigData))
    {
        // Servo config from BaseStation: relay to FlightComputer + cache
                setPendingCommandWithConfig(SERVO_CONFIG_PENDING, SERVO_CONFIG_MSG, payload, sizeof(ServoConfigData));
        cacheServoConfig(payload, payload_len);
        ESP_LOGI("LORA", "UPLINK Servo config queued for RocketComputer");
    }
    else if (cmd == 13 && payload_len >= 20)
    {
        // PID config from BaseStation: relay to FlightComputer + cache
                setPendingCommandWithConfig(PID_CONFIG_PENDING, PID_CONFIG_MSG, payload, 20);
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
    else if (cmd == 31 && payload_len >= ROLL_CTRL_CONFIG_LEN_V1)
    {
        // Roll control config from BaseStation. Relay the FULL struct: the FC's
        // readConfigFrame matches on an exact payload length, so a short copy is
        // never found and silently drops every roll-control setting.  A legacy
        // 16-byte uplink is padded to the full struct (speed gate off), the same
        // as the BLE path.
        uint8_t rc_buf[sizeof(RollControlConfigData)] = {};
        const size_t copy_len = (payload_len < sizeof(rc_buf)) ? payload_len : sizeof(rc_buf);
        memcpy(rc_buf, payload, copy_len);
        setPendingCommandWithConfig(ROLL_CTRL_CONFIG_PENDING, ROLL_CTRL_CONFIG_MSG, rc_buf, sizeof(rc_buf));
        cacheRollControlConfig(rc_buf, sizeof(rc_buf));
        ESP_LOGI("LORA", "UPLINK Roll control config queued for RocketComputer (%u B in)",
            (unsigned)payload_len);
    }
    else if (cmd == 32 && payload_len >= 1)
    {
        // Guidance enable/disable from BaseStation
        bool enabled = (payload[0] != 0);
        cfg_guidance_en = enabled;
        setPendingCommand(enabled ? GUIDANCE_ENABLE : GUIDANCE_DISABLE);
        ESP_LOGI("LORA", "UPLINK Guidance: %s", enabled ? "ENABLE" : "DISABLE");
    }
    // #383: no cmd-65 (GuidanceConfigData) uplink branch — deliberately.
    // The LoRa uplink rx_buf is 32 bytes, so a 36-byte GuidanceConfigData
    // payload can never arrive here (the old branch was dead code), and the
    // BS stopped relaying config entirely in #285 — guidance config reaches
    // the FC over BLE (cmd 65 handler in the BLE dispatch above). If uplink
    // config relay ever returns, the buffer needs resizing first.
    else if (cmd == 66 && payload_len >= sizeof(FinConfigData))
    {
        // Full fin layout from BaseStation: relay to FC
                setPendingCommandWithConfig(FIN_CONFIG_PENDING, FIN_CONFIG_MSG, payload, sizeof(FinConfigData));
        ESP_LOGI("LORA", "UPLINK Fin layout queued for RocketComputer");
    }
    else if (cmd == LORA_CMD_CHANNEL_SET && payload_len >= 5)
    {
        // Channel-set push from BS (#40 / #41 phase 3).  Wire format:
        //   [bw:f4][n_channels:u1][skip_mask: ceil(n/8) bytes]
        // Rendezvous freq used to lead this payload but is now hardcoded
        // on both sides (#105) — see LORA_FACTORY_RENDEZVOUS_MHZ.
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
        // Reject if BW doesn't match — the BS sent a mask sized for a
        // different hop table.  Either the user just changed BW and
        // we're racing the cmd-10 transaction, or the BS NVS is stale.
        // Either way, drop this push; the BS will re-push after its
        // own state settles.
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

        // Persist (rdv_mhz no longer stored — see #105 / NVS schema v3)
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
        // Coordinated hop pause from BS (#90).  The BS asks us to park
        // on lora_freq_mhz with our operating preset for N ms, so it can
        // run a noise scan + push cmd 15 without the link dropping.
        // Wire format: [duration_ms:u2 little-endian].
        uint16_t dur_ms;
        memcpy(&dur_ms, payload + 0, 2);
        if (dur_ms == 0)
        {
            ESP_LOGW("LORA", "UPLINK Cmd 16 ignored: zero duration");
            return;
        }
        if (dur_ms > LORA_HOP_PAUSE_MAX_MS) dur_ms = LORA_HOP_PAUSE_MAX_MS;
        // Only meaningful while we're hopping.  In non-hop states the
        // BS uses the existing direct-scan path and never sends cmd 16.
        // Slow-rendezvous (#71) doesn't need to be checked here: while
        // it's parking the radio on the rendezvous channel, the BS is on a
        // hop channel and physically can't deliver cmd 16 to us.
        if (!hop_active_)
        {
            ESP_LOGI("LORA", "UPLINK Cmd 16 ignored: not hopping");
            return;
        }
        // Idempotent if we're already paused — extend the deadline.
        if (hop_fallback_state == HopFallbackState::PAUSED_FOR_SCAN)
        {
            hop_pause_until_ms = millis() + dur_ms;
            ESP_LOGI("OC", "[HOP] Pause extended: +%u ms", (unsigned)dur_ms);
            return;
        }
        // From any other fallback state (NORMAL or mid VISITING_RENDEZVOUS)
        // we move to PAUSED_FOR_SCAN.  Reconfigure with the operating
        // preset on lora_freq_mhz; this is *not* a rendezvous visit.
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
        // Heartbeat from BS (issue #71).  No action — last_uplink_rx_ms
        // is already updated in the caller, which is all this command
        // exists to do.  Verbose-level log only, since these fire every
        // 30 s during normal operation.
        ESP_LOGV("LORA", "UPLINK heartbeat");
    }
    else
    {
        ESP_LOGW("LORA", "UPLINK Unknown cmd %u", cmd);
    }
}

// Write one LORA_UPLINK_MSG (0xF9) record for a decode the radio handed up.
//
// The rocket's only measurement of the RF path.  A transmitter cannot read its
// own downlink RSSI, so 0xF1 gives loss but not signal strength; these records
// give signal strength, on the reciprocal path, whether or not the base
// station's own log survives.  Every disposition is recorded — accepted,
// low-SNR, wrong network, not addressed to us, malformed — because a decode
// that was discarded still measures the channel just as well as one that was
// acted on.
//
// Rate is bounded by how often the base station actually transmits (a ~30 s
// heartbeat plus operator commands), so this is a handful of 13 B records per
// minute, not a stream.  enqueueFrame() drops it when no session is open.
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

    // Watchdog BEFORE the canSend() gate. Below it, the watchdog only ran in
    // the state where it has nothing to do: a stuck TX means canSend() is
    // false, which returned out of this function right here — so the one
    // path meant to clear a wedge was unreachable from the wedge. Latent on
    // V7 (DIO1 + the driver's own polling cleared tx_ongoing_ locally); on
    // V8 the only nominal clear is the modem's TX_RESULT, so one lost UART
    // frame would have pinned canSend() false forever: no telemetry, no
    // beacons, and no uplink RX either (readPacket sits below the same
    // gate). The BS has always ordered these correctly.
    lora_comms.serviceTxWatchdog();  // Force-clear stuck TX (#105)

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
        // SNR floor (#90 follow-up).  Drop noise-floor false positives
        // before they can fire processUplinkCommand — a fake cmd 10 on
        // garbage payload could put the radio on a bad channel that
        // breaks the link entirely.  Threshold tracks the current SF
        // (rendezvous + operating use different presets).
        TR_LoRa_Comms::Stats ls = {};
        lora_comms.getStats(ls);
        const float min_snr = loraMinValidSnrDb(lora_comms.currentSpreadingFactor());
        if (ls.last_snr < min_snr)
        {
            lora_low_snr_drops++;
            // Logged before the return.  These are the samples nearest the
            // floor — dropping them from the record would bias the logged
            // RSSI distribution upward exactly where the link is marginal,
            // which is the regime the log exists to characterise.
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
                // rx_buf[3] = next_channel_idx — phase 1 ignores it (sentinel only).
                // Phase 2 will hop to loraChannelMHz(bw, idx) here unless == 0xFF.
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
                    // Header said payload_len but the frame is short — a
                    // truncated decode, which is a real signal-quality data
                    // point, not a nothing.
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
                // Our network, addressed to a different rocket.  Still a clean
                // decode off the air, so its RSSI is as valid a path-loss
                // sample as one addressed to us.
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
// SECTION: LoRa rendezvous and hop fallback
// ==========================================================================
// If the rocket has been silent (no uplink received) for long enough, hop
// briefly to LORA_FACTORY_RENDEZVOUS_MHZ on a duty cycle so the base station's
// Phase-A recovery has a guaranteed meeting point even when the two NVS
// freqs disagree by more than the BS's ±2 MHz scan range (e.g. the rocket
// kept a previously-scanned channel like 921.5 MHz while the BS rebooted
// to a fresh 915 MHz default).
//
// Trigger is adaptive:
//   • If we've never received an uplink (last_uplink_rx_ms == 0), fire
//     after just RENDEZVOUS_TRIGGER_INITIAL_MS — the BS may genuinely be
//     lost and we want to find it quickly.
//   • Once we've seen at least one uplink, fall back to the longer
//     RENDEZVOUS_TRIGGER_QUIET_MS — the BS is alive, it's just being
//     idle; no need to thrash the channel.
//
// Cycle (30 s period) is sized so a 30 s BS Phase A on the rendezvous
// freq always overlaps at least one full rocket window:
//   10 s on rendezvous → 20 s back on NVS → repeat.
//
// Suppressed while freq_locked_for_flight (set on INFLIGHT, cleared on
// LANDED/READY).  Allowed from any other state — including INITIALIZATION,
// so a rocket whose FlightComputer hasn't booted yet can still surface
// itself on the rendezvous channel.

enum class RocketRendezvousState : uint8_t {
    IDLE,
    ON_RENDEZVOUS,    // RENDEZVOUS_WINDOW_MS on LORA_FACTORY_RENDEZVOUS_MHZ
    ON_SAVED,         // RENDEZVOUS_SAVED_MS back on lora_freq_mhz (NVS)
};

static RocketRendezvousState rendezvous_state = RocketRendezvousState::IDLE;
static uint32_t rendezvous_phase_start_ms = 0;

// #105 follow-up: tightened from 30 s / 120 s to 15 s for both triggers so
// the operator doesn't sit through a long silence after a flash with stale
// NVS, or after any other source of channel divergence.  Cycle (window +
// saved) still gives the BS Phase A 30 s window plenty of overlap for a
// clean handshake.
static constexpr uint32_t RENDEZVOUS_TRIGGER_INITIAL_MS = 15000;  // never heard BS yet
static constexpr uint32_t RENDEZVOUS_TRIGGER_QUIET_MS   = 15000;  // BS seen, then silent
static constexpr uint32_t RENDEZVOUS_WINDOW_MS          = 10000;  // on rendezvous freq
static constexpr uint32_t RENDEZVOUS_SAVED_MS           = 20000;  // back on saved freq

// Hop the radio to the full rendezvous mode (freq + SF/BW/CR/power).
// Uses ALL the rendezvous constants — frequency alone isn't enough if
// the user has configured a non-Standard preset (e.g. Long Range = SF10/
// BW125).  Both sides need to agree on every modulation parameter to
// decode each other; the rendezvous mode is the shared known-good
// fallback.
// #398: returns success so the state machine only advances when the radio
// actually retuned.  Passes wait_for_tx=false — a mid-TX hop returns busy
// immediately instead of spin-waiting out the packet's 100-200 ms airtime
// inside loop_oc; the rendezvous service simply retries next iteration.
// (Previously the state advanced even on failure, leaving the state machine
// and the radio on different modes until the next window boundary.)
static bool rendezvousHopToRendezvousMode()
{
    // All five values are compile-time constants in RocketComputerTypes.h
    // so the BS and OC are guaranteed to agree on the meeting place even
    // when each side's NVS has drifted (#105).
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

// Hop the radio back to whatever NVS says — i.e. the working config the
// user picked (or factory defaults if NVS empty).  Called when exiting
// rendezvous and at the end of each ON_RENDEZVOUS window.
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
        // Radio busy/failed — stay in the current state; the silence-broke
        // check at the top of serviceRocketRendezvous re-runs this exit on
        // the next loop iteration.
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
    // Suppress while in flight (#71) or actively hopping (#40 / #41).
    // The hop state machine and the slow-rendezvous cycle both want to
    // own the radio frequency; running them concurrently would have the
    // rocket disappear from the hop sequence every 30 s to visit the
    // rendezvous freq, which defeats the point.
    // INITIALIZATION/READY/LANDED stay eligible — those are recovery /
    // pre-handshake situations where rendezvous is the right behaviour.
    if (freq_locked_for_flight || hop_active_)
    {
        rendezvousExit(freq_locked_for_flight ? "flight locked" : "hopping active");
        return;
    }

    const uint32_t now = millis();

    // Adaptive trigger: short window if we've never received an uplink
    // (BS may genuinely be lost), longer otherwise (BS exists, just idle).
    const uint32_t trigger_ms = (last_uplink_rx_ms == 0)
        ? RENDEZVOUS_TRIGGER_INITIAL_MS
        : RENDEZVOUS_TRIGGER_QUIET_MS;

    // Silence reference: most-recent uplink receipt or, failing that,
    // most-recent READY entry.  If neither has happened (fresh boot,
    // never reached READY), measure from boot itself.
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
                // #398: advance only when the retune actually happened; a
                // busy radio (mid-TX) retries next loop iteration instead of
                // stalling loop_oc for the packet's airtime.
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

// ============================================================================
// Hop-silence rendezvous fallback (#40 / #41 phase 2b)
// ============================================================================
// While hop_active_, periodically check whether we've heard *anything*
// from the base station recently.  If we haven't, briefly park on the
// rendezvous channel/preset so a desynced BS — one whose 3 s
// hop-silence fallback (loop_bs) fired and dropped to its static
// channel — can find us via its existing recovery scan, which sweeps
// to the rendezvous frequency in Phase A.
//
// This mirrors slow_rendezvous (#71) but with hop-aware triggers:
//   • Reference time is the most recent LoRa uplink during the
//     current hop session (or hop entry, if no uplink heard yet).
//     Heartbeats from the BS — safe-window-scheduled, every 10 s
//     (HEARTBEAT_INTERVAL_MS) — bump last_uplink_rx_ms in nominal
//     operation, so this trigger fires only when comms have actually
//     broken.  NOTE the BS must keep heartbeating IN FLIGHT for this
//     to hold: gating heartbeats on freq_locked_for_flight starved
//     this timer and tore the session down every ~33 s (2026-07-16
//     bench; fixed BS-side in serviceHeartbeat).
//   • Visit is a single short window (no on/off oscillation), then we
//     re-bootstrap hopping with a fresh transition packet on
//     lora_freq_mhz so the BS sees a clean re-entry.
//
// Suppressed when slow_rendezvous is busy (it owns the radio in that
// case) and when not actually hopping.  HopFallbackState and its module
// variables live up with the other hop_* state; constants live here
// alongside the service function that uses them.

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
            // reconfigure() switches BW/SF/CR/freq atomically (with
            // rollback on failure) — same machinery as slow_rendezvous
            // so the radio handling is identical and well-tested.
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

            // Visit done.  Reconfigure back to the saved params and
            // restart hopping with a fresh bootstrap so the BS sees a
            // clean transition packet on lora_freq_mhz with
            // next_channel_idx = 0.
            if (!lora_comms.reconfigure(lora_freq_mhz, lora_sf, lora_bw_khz,
                                         lora_cr, lora_tx_power))
            {
                ESP_LOGE("OC", "[HOP] Visit failed: reconfigure back to saved params");
                // Stay in VISITING_RENDEZVOUS; will retry on next call.
                return;
            }
            (void)lora_comms.startReceive();
            lora_in_rx_mode = true;

            hop_bootstrap_left_ = currentHopDwell();
            if (hop_bootstrap_left_ == 0) hop_bootstrap_left_ = 1;
            hop_idx_          = 0;
            hop_needs_retune_ = false;  // already on lora_freq_mhz from reconfigure
            hop_fallback_state = HopFallbackState::NORMAL;
            // Restart trigger reference so the next visit fires only
            // after a fresh QUIET window (or INITIAL if nothing
            // arrives during this pass either).
            hop_active_entered_ms = now;
            hop_session_uplink_count = 0;
            ESP_LOGI("OC", "[HOP] Visit done — resuming hop (%u bootstrap pkt(s))",
                     (unsigned)hop_bootstrap_left_);
            break;
        }
        case HopFallbackState::PAUSED_FOR_SCAN:
        {
            // Coordinated pause for BS scan (#90).  We're parked on
            // lora_freq_mhz with the operating preset; the BS is
            // sweeping for noise + pushing cmd 15.  When the deadline
            // hits, re-bootstrap hopping identical to a fresh
            // PRELAUNCH entry — including the channel(0) anchor — so
            // both sides re-enter the table cleanly.  Use signed delta
            // to handle millis() wrap.
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
// SECTION: Diagnostics and periodic statistics
// ==========================================================================
static void printLoRaPayloadDebug()
{
    if (!config::USE_LORA_RADIO)
    {
        return;
    }

    // Always FAST here: the debug dump wants the position/attitude picture,
    // and a SLOW frame would leave most of the printed fields at zero.
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

// #398: per-task CPU utilization sampler.
//
// The launch-activation window (~4 s) co-stalls everything on loop_oc, which
// the bench (issue #398 comment 2) read as core-1 CPU starvation by a task
// ABOVE the parser's prio 6 — prime suspect the prio-20 i2c_slv_tx serve, or
// ISR load during the prepareFlight erase burst. This dumps, once per stats
// interval, each task's run-time delta (µs of CPU consumed since the last
// call) as a percent of the wall interval, sorted high-to-low, plus core-1
// utilization derived from the IDLE1 idle task. During the launch window the
// hog's share spikes, naming it without a full scheduling trace.
//
// Needs configUSE_TRACE_FACILITY + configGENERATE_RUN_TIME_STATS (set in this
// project's sdkconfig.defaults). ulRunTimeCounter is the esp_timer µs clock.
#if (configUSE_TRACE_FACILITY == 1) && (configGENERATE_RUN_TIME_STATS == 1)
static void logTaskCpuDeltas(uint32_t dt_ms)
{
    if (!config::PROFILE_TASK_CPU) return;

    // Single call site (printStats on oc_loop), so these can be static — keeps
    // ~3 KB of snapshot off a stack that already runs deep into BLE sendTelemetry.
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

    // Core-1 utilization = fraction of the interval IDLE1 did NOT run. Clamp:
    // tickless-idle skew can make idle1_d edge just past the wall interval.
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

static void printStats()
{
    const uint32_t now = millis();

    // #524: re-emit the last transfer's summary for a few minutes so a BATTERY
    // download can still be read back — plug USB in afterwards and attach with
    // `idf.py monitor --no-reset`. See s_xfer_summary.
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
    if (!pwr_pin_on)
    {
        // Skip the INA230 I2C poll while an OTA is in flight (#17): the
        // esp_ota_begin() partition erase blocks SPI flash for ~1-2 s and the
        // gauge transaction collides with it, logging spurious I2C timeouts.
        // The telemetry push below still runs so the app sees liveness +
        // OTA status during the flash.
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
            ble_telem.cam_current   = p.cam_current;    // #850
            ble_telem.servo_current = p.servo_current;  // #850
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
        // OC boots into low-power mode, so this is the path that runs right
        // after a post-OTA reboot + app reconnect — the critical place to
        // validate the new image. Gate on a live connection so we only cancel
        // rollback once we've proven BLE works end-to-end with a client (#8).
        if (ble_app.isConnected()) maybeMarkOtaValid();
        return;
    }

    // --- Active mode: full stats and telemetry ---
    TR_LogToFlashStats s = {};
    logger.getStats(s);

    // Flash-space stats for the app's storage bar (every ~3 s on a live BLE
    // link).  Same flightlog accounting as ocStorageHealth(); accurate on a
    // direct rocket connection.
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
                // #671: runtime block size — 256 on the V8 part, 128 on the
                // GD5F parts. The wire struct is self-describing, so the app
                // scales by this field.
                rss.block_size_kb = (uint16_t)(flightlog.pageSize() * flightlog.pagesPerBlock() / 1024u);
                rss.flags         = RSS_FLAG_INITIALIZED;
                if (flightlog.autoEvictedCount() > 0)  // #315: rolling-buffer evicted this session
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
    const uint32_t ring_bad_sof_delta = s.ring_bad_sof_clears - prev_ring_bad_sof_clears;
    prev_bytes_rx = s.bytes_received;
    prev_bytes_nand = s.bytes_written_nand;
    prev_raw_i2c_bytes = raw_i2c_bytes;
    prev_ring_overruns = s.ring_overruns;
    prev_ring_drop_oldest_bytes = s.ring_drop_oldest_bytes;
    prev_ring_bad_sof_clears = s.ring_bad_sof_clears;

    const float rx_kbs = (dt > 0) ? ((float)rx_delta / (float)dt) : 0.0f;
    const float wr_kbs = (dt > 0) ? ((float)nand_delta / (float)dt) : 0.0f;
    const float raw_rx_kbs = (dt > 0) ? ((float)raw_i2c_delta / (float)dt) : 0.0f;
    const uint32_t d_query = msg_count_query - prev_msg_count_query;
    const uint32_t d_ism6 = msg_count_ism6 - prev_msg_count_ism6;
    const uint32_t d_bmp = msg_count_bmp - prev_msg_count_bmp;
    const uint32_t d_mmc = msg_count_mmc - prev_msg_count_mmc;
    const uint32_t d_iis2mdc = msg_count_iis2mdc - prev_msg_count_iis2mdc;
    const uint32_t d_gnss = msg_count_gnss - prev_msg_count_gnss;
    const uint32_t d_non_sensor = msg_count_non_sensor - prev_msg_count_non_sensor;
    const uint32_t d_power = msg_count_power - prev_msg_count_power;
    const uint32_t d_start_logging = msg_count_start_logging - prev_msg_count_start_logging;
    const uint32_t d_end_flight = msg_count_end_flight - prev_msg_count_end_flight;
    const uint32_t d_unknown = msg_count_unknown - prev_msg_count_unknown;
    const uint32_t d_logonly = msg_count_logonly - prev_msg_count_logonly;   // #569
    const float hz_scale = (dt > 0U) ? (1000.0f / (float)dt) : 0.0f;
    const float hz_query = (float)d_query * hz_scale;
    const float hz_ism6 = (float)d_ism6 * hz_scale;
    const float hz_bmp = (float)d_bmp * hz_scale;
    const float hz_mmc = (float)d_mmc * hz_scale;
    const float hz_iis2mdc = (float)d_iis2mdc * hz_scale;
    const float hz_gnss = (float)d_gnss * hz_scale;
    const float hz_non_sensor = (float)d_non_sensor * hz_scale;
    const float hz_power = (float)d_power * hz_scale;
    const float hz_start_logging = (float)d_start_logging * hz_scale;
    const float hz_end_flight = (float)d_end_flight * hz_scale;
    const float hz_unknown = (float)d_unknown * hz_scale;
    const float hz_logonly = (float)d_logonly * hz_scale;   // #569
    prev_msg_count_query = msg_count_query;
    prev_msg_count_ism6 = msg_count_ism6;
    prev_msg_count_bmp = msg_count_bmp;
    prev_msg_count_mmc = msg_count_mmc;
    prev_msg_count_iis2mdc = msg_count_iis2mdc;
    prev_msg_count_gnss = msg_count_gnss;
    prev_msg_count_non_sensor = msg_count_non_sensor;
    prev_msg_count_power = msg_count_power;
    prev_msg_count_start_logging = msg_count_start_logging;
    prev_msg_count_end_flight = msg_count_end_flight;
    prev_msg_count_unknown = msg_count_unknown;
    prev_msg_count_logonly = msg_count_logonly;   // #569

    // Persist a LogBufferStats snapshot once per stats interval (~1 Hz) into
    // the flight log so post-flight tooling can read the per-flight peak
    // without needing VERBOSE_DEBUG serial captures.  Cheap (28 B payload +
    // 8 B frame overhead = 36 B/s) and enqueueFrame() internally drops the
    // frame when no session/file is open, so off-session traffic is zero.
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
    // Denominator is s.ring_size (the runtime ring capacity) — used to show
    // 64 KB when the RAM-only fallback ring is in use even with MRAM wired,
    // which obscured the real headroom on the 128 KB MR25H10.
    ESP_LOGI("OC", "RX %.1f KB/s | WR %.1f KB/s | frames rx/drop/bad=%lu/%lu/%lu | RING=%lu/%lu (hi=%lu)",
                  (double)rx_kbs,
                  (double)wr_kbs,
                  (unsigned long)s.frames_received,
                  (unsigned long)s.frames_dropped,
                  (unsigned long)frames_bad_crc,
                  (unsigned long)s.ring_fill,
                  (unsigned long)s.ring_size,
                  (unsigned long)s.ring_highwater);
    ESP_LOGI("OC", "RING interval peak/overrun/drop_oldest_bytes/bad_sof=%lu/%lu/%lu/%lu (bad_sof_total=%lu)",
                  (unsigned long)interval_ring_fill_peak,
                  (unsigned long)ring_overrun_delta,
                  (unsigned long)ring_drop_oldest_delta,
                  (unsigned long)ring_bad_sof_delta,
                  (unsigned long)s.ring_bad_sof_clears);
    ESP_LOGI("OC", "i2c raw reads/bytes=%lu/%llu",
                  (unsigned long)raw_i2c_reads,
                  (unsigned long long)raw_i2c_bytes);
    ESP_LOGI("OC", "i2c raw_rx=%.1f KB/s | ring_drops=%lu | cmd_drops=%lu | parser_drops resync/len/crc=%llu/%llu/%lu",
                  (double)raw_rx_kbs,
                  (unsigned long)rx_ring_overflow_drops,
                  (unsigned long)cmd_ring_drop_count,
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
    ESP_LOGI("OC", "msg Hz q/ism6/bmp/mmc/iis2mdc/gnss/ns/pwr/st/en/unk/lo=%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f/%.1f",
                  (double)hz_query,
                  (double)hz_ism6,
                  (double)hz_bmp,
                  (double)hz_mmc,
                  (double)hz_iis2mdc,
                  (double)hz_gnss,
                  (double)hz_non_sensor,
                  (double)hz_power,
                  (double)hz_start_logging,
                  (double)hz_end_flight,
                  (double)hz_unknown,
                  (double)hz_logonly);   // #569: log-only ≠ unknown
    if (config::USE_LORA_RADIO)
    {
        TR_LoRa_Comms::Stats ls = {};
        lora_comms.getStats(ls);
        // txmute= is here so a flat tx_ok reads as "muted on purpose" rather
        // than "radio broken" — that is the whole diagnostic value of the field.
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
    }
    // I2S pipeline stats
    {
        static uint32_t prev_dma_cb = 0, prev_ring_ovf = 0,
                         prev_dedup_eq = 0, prev_dedup_lt = 0,
                         prev_stale = 0;
        static uint64_t prev_dma_bytes = 0;
        uint32_t d_cb = dma_cb_count - prev_dma_cb;
        uint64_t d_bytes = raw_i2c_bytes - prev_dma_bytes;
        uint32_t d_ovf = rx_ring_overflow_drops - prev_ring_ovf;
        uint32_t d_stale = stale_drops - prev_stale;
        uint32_t d_dedup_eq = dedup_drops_eq - prev_dedup_eq;
        uint32_t d_dedup_lt = dedup_drops_lt - prev_dedup_lt;
        static uint32_t prev_replay = 0;
        uint32_t d_replay = dedup_replay_drops - prev_replay;
        prev_replay = dedup_replay_drops;
        uint32_t d_parsed = msg_count_ism6 + msg_count_bmp + msg_count_mmc + msg_count_iis2mdc + msg_count_non_sensor + msg_count_gnss;
        static uint32_t prev_total_parsed = 0;
        uint32_t d_p = d_parsed - prev_total_parsed;
        // Calculate non-zero byte percentage
        static uint32_t prev_nz = 0;
        static uint32_t prev_tot = 0;
        uint32_t d_nz = dma_nonzero_bytes - prev_nz;
        uint32_t d_tot = dma_total_bytes - prev_tot;
        float nz_pct = (d_tot > 0) ? (d_nz * 100.0f / d_tot) : 0;

        ESP_LOGI("I2S", "dma_cb=%lu KB=%.1f nz=%.1f%% ovf=%lu dedup_eq=%lu dedup_lt=%lu stale=%lu replay=%lu parsed=%lu frx=%lu fdr=%lu",
                 (unsigned long)d_cb,
                 (double)(d_bytes / 1024.0),
                 (double)nz_pct,
                 (unsigned long)d_ovf,
                 (unsigned long)d_dedup_eq,
                 (unsigned long)d_dedup_lt,
                 (unsigned long)d_stale,
                 (unsigned long)d_replay,
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
            // logged= counts LORA_MSG records the flight-log ring accepted.
            // Expect it to track tx= once a session is open, and to stay at 0
            // on the pad with no session — see lora_tx_logged.
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

        prev_dma_cb = dma_cb_count;
        prev_dma_bytes = raw_i2c_bytes;
        prev_ring_ovf = rx_ring_overflow_drops;
        prev_dedup_eq = dedup_drops_eq;
        prev_dedup_lt = dedup_drops_lt;
        prev_stale = stale_drops;
        prev_total_parsed = d_parsed;
        prev_nz = dma_nonzero_bytes;
        prev_tot = dma_total_bytes;
    }
    } // End VERBOSE_DEBUG

    // I2S RX-side stall instrumentation (#104 follow-up).  Snapshot the
    // counters first, then reset them so the next window measures peaks
    // since this print only.  Three signals together catch the
    // FC→OC→bin loss path:
    //   rx_ovf      - bytes silently lost at DMA→rx_ring boundary
    //                 (the silent killer; nothing else flags this)
    //   rx_peak     - high-water of rx_ring fill since last print
    //                 (high but not overflowing = parser barely keeping up)
    //   parser_max  - longest single parseRxStream() call since last print
    //                 (catches parser stalls that the LFS/NAND timers miss)
    static uint32_t s_prev_rx_ovf = 0;
    const uint32_t  cur_rx_ovf  = rx_ring_overflow_drops;
    const uint32_t  d_rx_ovf    = cur_rx_ovf - s_prev_rx_ovf;
    s_prev_rx_ovf = cur_rx_ovf;
    const uint32_t  cur_rx_peak = rx_ring_peak_fill;
    rx_ring_peak_fill = 0;
    const uint32_t  cur_parser  = parser_iter_max_us;
    parser_iter_max_us = 0;

    // Always-on LFS/NAND stall instrumentation — prints the peak duration of
    // each potentially-slow LittleFS/NAND op observed since the last stats
    // window.  Complements the per-op ESP_LOGW("STALL: …") that fires live
    // whenever any single op exceeds 100 ms.
    ESP_LOGI("LOG TIMING",
             "write=%lu sync=%lu erase=%lu open=%lu close=%lu "
             "activate=%lu clr_ring=%lu iter=%lu us  syncs=%lu erases=%lu ring_peak=%lu "
             "bad_blocks=%lu skips=%lu  "
             "rx_ovf=%lu rx_peak=%lu parser_max=%lu spiw=%lu spih=%lu us  "
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
             (unsigned long)d_rx_ovf,
             (unsigned long)cur_rx_peak,
             (unsigned long)cur_parser,
             (unsigned long)s.spi_wait_max_us,   // #398: parser starvation source
             (unsigned long)s.spi_hold_max_us,   // #398
             (unsigned long)s.drain_pages,       // #510: window SUMS — a burst of
             (unsigned long)s.drain_bytes,       //   sub-threshold page writes is
             (unsigned long)s.pop_sum_us,        //   invisible to the maxima above
             (unsigned long)s.write_sum_us,      // #510
             (unsigned long)flightlog.writeLockWaitMaxUs(),   // #510: #388 max/sum —
             (unsigned long)flightlog.writeLockWaitSumUs());  //   mutex share of wsum
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

    // #398: per-task CPU deltas over this same interval — pins the core-1 hog
    // that co-stalls loop_oc during the launch-activation window. Uses `dt`
    // (the actual interval) as the denominator so percentages track jitter.
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

    // FC boot progress: report it ONLY until the FC's first NonSensorData
    // arrives.  From that moment rocket_state is real and the boot keys would
    // be stale, so they stop being emitted and the app falls back to the state
    // machine.  Before ANY boot frame arrives the keys are absent too, which is
    // itself the "flight computer has not spoken" signal the app needs to
    // distinguish a powered-off FC from a booting one.
    ble_telem.boot_valid      = fc_boot_status_valid && !fc_ns_since_boot;
    ble_telem.boot_step       = fc_boot_status.step;
    ble_telem.boot_degraded   = fc_boot_status.degraded;
    ble_telem.boot_elapsed_ms = fc_boot_status.elapsed_ms;
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
    // #191: EKF ENU velocity + burnout for the app's ascent prediction
    // (direct-BLE path; the LoRa path relays the same via LoRaData).
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
    // Attitude quaternion from FlightComputer
    ble_telem.q0 = (float)latest_non_sensor.q0 / 10000.0f;
    ble_telem.q1 = (float)latest_non_sensor.q1 / 10000.0f;
    ble_telem.q2 = (float)latest_non_sensor.q2 / 10000.0f;
    ble_telem.q3 = (float)latest_non_sensor.q3 / 10000.0f;
    ble_telem.roll_cmd = (float)latest_non_sensor.roll_cmd / 100.0f;
    // Sensor health scorecard (#303) — direct-BLE path (no LoRa hop): take the
    // FC's bits and fold in the OC-owned battery verdict.  The FC never reads the
    // pack, so without this the operator would see battery = N/A on a direct link.
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
    // #831: freshness of the FC-sourced half of this frame.
    //
    // Everything above that describes the rocket — attitude, the sensor-health
    // scorecard, the pyro bits — is copied out of latest_non_sensor, an I2C
    // snapshot the OC republishes on every BLE tick regardless of whether the
    // FC is still talking.  latest_non_sensor_valid latches once at first
    // receipt and is never cleared, so an FC that dies, reboots, or has its
    // rail cut leaves the OC sending the last good frame's continuity bits
    // forever, and the app renders a confident green CONT measured before the
    // failure.  That is precisely the held-over green #297 exists to prevent;
    // it just had no backstop on the direct path, because data_status was
    // never set here at all and defaulted to LIVE on every frame.
    //
    // Setting it costs nothing on the wire (the field already exists and is
    // only serialised when non-LIVE) and needs no app change: the app's
    // existing #297 machinery already renders a non-live stream's continuity
    // as NO DATA rather than a stale reading.
    //
    // The FC sends NonSensorData at NON_SENSOR_UPDATE_RATE = 500 Hz, so this
    // threshold is ~1500 missed frames — it cannot fire on an I2S hiccup, only
    // on an FC that has genuinely stopped.  SYNCING rather than STALE before
    // the first frame ever arrives: "no rocket data yet" is a different thing
    // from "rocket data has gone stale", and it is what the app already shows
    // during the FC's ~25 s boot.
    {
        const uint32_t now_ms = millis();
        if (!latest_non_sensor_valid)
        {
            ble_telem.data_status = TR_BLE_To_APP::TelemetryData::DataStatus::SYNCING;
            ble_telem.data_age_ms = 0;
        }
        else
        {
            const uint32_t ns_age_ms = now_ms - latest_non_sensor_rx_ms;
            if (ns_age_ms > config::FC_FRAME_STALE_MS)
            {
                ble_telem.data_status = TR_BLE_To_APP::TelemetryData::DataStatus::STALE;
                ble_telem.data_age_ms = ns_age_ms;
            }
            else
            {
                ble_telem.data_status = TR_BLE_To_APP::TelemetryData::DataStatus::LIVE;
                ble_telem.data_age_ms = 0;
            }
        }
    }
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
    ble_telem.sim_active        = nsFlagSet(latest_non_sensor.flags, NSF_SIM_ACTIVE);  // #393
    ble_telem.pwr_pin_on        = pwr_pin_on;
    // Pyro channel status from NonSensorData (single shared armed bit
    // mirrors the live ARM pin; 4 per-channel cont/fired bits).
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
        maybeMarkOtaValid();   // validate a fresh OTA image once telemetry flows to a connected client (#8)
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

// ==========================================================================
// SECTION: Peripheral initialization (runs on power-on)
// ==========================================================================
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
    // #822: on a board whose MRAM was replaced by in-package PSRAM, put the
    // ring there instead of in the 64 KB internal fallback. 0 on every
    // MRAM-fitted board, which keeps V7/V8 on exactly their old path.
    log_cfg.psram_ring_size = config::RING_IN_PSRAM ? config::PSRAM_RING_SIZE : 0;
    log_cfg.debug = config::DEBUG;
    // MRAM ring buffer (128 KB on shared SPI bus — replaces 64 KB RAM ring)
    log_cfg.mram_cs = config::MRAM_CS;
    log_cfg.spi_hz_mram = config::SPI_HZ_MRAM;
    log_cfg.spi_mode_mram = config::SPI_MODE_MRAM;
    // Shrink the ring so the top SNAPSHOT_REGION_SIZE bytes are reserved
    // for the FlightSnapshot store (#104 follow-up).  Ring uses [0, ring_size_),
    // snapshot region uses [SNAPSHOT_REGION_BASE, MRAM_SIZE).
    log_cfg.mram_size = config::MRAM_SIZE - config::SNAPSHOT_REGION_SIZE;

    // --- LFS shrunk to 4 MB + hot-path write sink (issue #50) ---------------
    // LFS holds 32 blocks for config/placeholder use; TR_FlightLog owns the
    // rest of the chip up to its four metadata blocks (#671: block size and
    // count are runtime chip geometry). Each (page - 16)-byte chunk the
    // flush task drains from the ring is routed through
    // flightlogWriteSink → flightlog.writeFrame(), which wraps it in a
    // PageHeader (CRC32 + seq + flight_id) and programs one NAND page
    // directly.
    log_cfg.lfs_block_count = 32;
    log_cfg.write_sink = flightlogWriteSink;
    log_cfg.write_sink_ctx = &flightlog;
    log_cfg.flush_task_hook = flightlogFlushTaskHook;
    log_cfg.dirty_marker_addr = config::MRAM_DIRTY_MARKER_ADDR;  // #274: sink-mode dirty marker

    if (!logger.begin(SPI, log_cfg))
    {
        // Non-obvious on the RAM-ring path: begin() also fails when the 64 KB
        // internal-RAM ring can't be allocated. That leaves
        // peripherals_initialized false and flightlog uninitialized, so
        // ocStorageHealth() reports SH_BAD and the pre-launch scorecard goes
        // red — the operator does find out. Keep it that way.
        ESP_LOGE("PWR", "TR_LogToFlash begin failed");
        return;
    }

    // #822: V9/V10 deleted the MRAM (U12, MR25H10) — the S3RH2's in-package
    // PSRAM replaced it — so board_v9.h sets MRAM_CS = -1 and TR_LogToFlash
    // falls back to a heap RAM ring. Flight logging is unaffected (the ring
    // still feeds the NAND flush task), but two things that live in the MRAM
    // region are GONE on those boards, not degraded:
    //
    //   * #104 in-flight reboot recovery. The FC asks for its last snapshot
    //     with GET_FLIGHT_SNAPSHOT after a brownout/panic; the handler reads it
    //     out of MRAM, so with no MRAM there is nothing to answer with and the
    //     FC skips recovery.
    //   * #274 dirty-ring replay. The marker sits in the same region, so an
    //     unclean shutdown can no longer be detected, and the volatile ring
    //     would not have survived the reset anyway.
    //
    // Both failures are otherwise completely silent — mramRawWrite/Read just
    // return false, nothing logs, nothing goes red — so the first evidence
    // would be a flight that browned out and never came back. Say it once,
    // loudly, at boot instead. The replacement design already exists on the
    // mini (snapshots written into the NAND log stream, recovered by a
    // tail-scan at boot); porting it to the OC/FC pair is the real fix.
    // #822: cross-check the board flag against the silicon, unconditionally —
    // this is the generalisation of the bug that motivated board_v9.h. A wrong
    // -DTR_BOARD_* flag has no runtime symptom on this MCU (the pin maps are
    // identical), so the only thing that can catch it is asking the chip.
    {
        const int psram_mb = s3PsramCapMb();
        if (psram_mb < 0)
        {
            ESP_LOGI("PWR", "In-package PSRAM per eFuse: unreadable");
        }
        else if (psram_mb == 0)
        {
            ESP_LOGI("PWR", "In-package PSRAM per eFuse: none");
        }
        else
        {
            ESP_LOGI("PWR", "In-package PSRAM per eFuse: %d MB", psram_mb);
        }

        // Disagreements are build/flash mistakes, not hardware faults, so they
        // are worth shouting about while someone is still at the bench.
        if (config::RING_IN_PSRAM && psram_mb == 0)
        {
            ESP_LOGE("PWR", "BOARD FLAG SAYS PSRAM, SILICON SAYS NONE — this is "
                            "not a V9/V10 board, or it was built -DTR_BOARD_V9=1 "
                            "by mistake. Ring falls back to internal RAM.");
        }
        else if (!config::RING_IN_PSRAM && psram_mb > 0)
        {
            ESP_LOGW("PWR", "This chip has %d MB of in-package PSRAM that the "
                            "selected board map does not use. Expected on a V7/V8 "
                            "board (MRAM is fitted); if this IS a V9/V10 board, it "
                            "was built with the wrong -DTR_BOARD_* flag.", psram_mb);
        }
    }

    if (!logger.isMramEnabled())
    {
        TR_LogToFlashStats mram_st = {};
        logger.getStats(mram_st);
        ESP_LOGW("PWR", "========================================");
        ESP_LOGW("PWR", "NO MRAM ON THIS BOARD (MRAM_CS = -1).");
        ESP_LOGW("PWR", "  Log ring: %lu KB of %s, VOLATILE.",
                 (unsigned long)(mram_st.ring_size / 1024),
                 logger.isRingInPsram() ? "in-package PSRAM" : "internal RAM");
        ESP_LOGW("PWR", "  In-flight reboot recovery (#104): RAM cache + NAND");
        ESP_LOGW("PWR", "  tail-scan (#846) — no longer MRAM-dependent.");
        ESP_LOGW("PWR", "  Dirty-ring replay (#274): UNAVAILABLE.");
        ESP_LOGW("PWR", "  Expected on V9/V10. On a V8 board this means the");
        ESP_LOGW("PWR", "  image was built with the wrong -DTR_BOARD_* flag.");
        ESP_LOGW("PWR", "========================================");
        // #822: PSRAM is the ring's intended home on V9/V10. Landing on
        // internal RAM instead means either CONFIG_SPIRAM is off or the part
        // did not come up — the ring is then ~8x smaller than designed, which
        // is exactly the kind of quiet downgrade this block exists to prevent.
        if (config::RING_IN_PSRAM && !logger.isRingInPsram())
        {
            ESP_LOGE("PWR", "  ^ THIS BOARD EXPECTED A PSRAM RING AND DID NOT "
                            "GET ONE — see the PSRAM init line earlier in this "
                            "boot log.");
        }
    }

    // --- TR_FlightLog begin (issue #50) -------------------------------------
    // SPI bus + physical bad-block bitmap are initialized by logger.begin();
    // flightlog.begin() loads the newest-valid of the dual-copy index from the
    // metadata blocks, plus the 3-state block bitmap. #398: the bitmap now
    // persists to NAND (metadata blocks [2]/[3], = 2046/2047) instead of NVS,
    // so prepareFlight/extend/bad-block saves never hit internal flash and can't
    // trigger the NVS-compaction cache-disable that stalled core 1.
    flightlog_backend = tr_flightlog::TR_NandBackend_esp(&logger);
    {
        tr_flightlog::TR_FlightLog::Config fl_cfg{};
        // #671: geometry is runtime — logger.begin() resolved the chip from
        // RDID, and everything below is denominated in ITS blocks. Region and
        // metadata come from the chip's block count (top four blocks are
        // metadata), which is what the Config sentinels would also derive —
        // but the auto-evict target and the bitmap-store bind() below need
        // the concrete numbers before flightlog.begin(), so fill explicitly.
        const auto& ngeom = logger.nandGeometry();
        fl_cfg.flight_region_end = static_cast<uint16_t>(ngeom.block_count - 4);
        for (int i = 0; i < 4; ++i)
            fl_cfg.metadata_blocks[i] = static_cast<uint16_t>(ngeom.block_count - 4 + i);
        // #398 / #492 / #671: the 3840 Hz stream makes larger files, so
        // pre-allocate 80 blocks up front — ~20 MB on the V8 bench part
        // (256 KB blocks), ~10 MB on V9's GD5F2GQ5UE (128 KB blocks); both
        // cover a typical flight (~156 KB/s x 60 s = ~9.4 MB) without the
        // extend path's in-flight erase hiccup.
        fl_cfg.prealloc_blocks = 80;
        // #315: rolling-buffer auto-eviction. When the card fills, prepareFlight
        // reclaims space at arm time by deleting the oldest finalized flight(s)
        // — never the in-progress one — down to a free-block headroom floor, so
        // the operator never hand-deletes and the pre-launch storage verdict
        // stays green. Destructive by nature (an un-downloaded flight can be
        // dropped), so it's a deliberate opt-in; it's surfaced via a log line +
        // the RSS_FLAG_AUTO_EVICTED storage-stats bit, never silent. Target ~10%
        // of the flight region (~2.5 preallocs of headroom).
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
            // #566: deliberately non-fatal (BLE/downlink still run so the fault
            // is reachable), but flight logging is DEAD this boot — every frame
            // will be dropped. ocStorageHealth() reports SH_BAD for this state
            // so the pre-launch scorecard goes red instead of grey N/A.
            ESP_LOGE("FLIGHTLOG", "begin failed: %s — flight logging DEAD this "
                     "boot (all frames will drop); storage health = BAD",
                     tr_flightlog::to_string(st));
        }
    }

    // #274: if the previous session left unflushed frames in the non-volatile
    // MRAM ring (dirty sink-mode boot), replay the surviving ring through the
    // sink into a recovered flight so the touchdown data isn't wiped. Runs after
    // flightlog.begin() (the sink needs an allocated flight) and before the flush
    // task / normal logging start — the sink writes synchronously.
    if (logger.hasPendingMramRecovery())
    {
        uint32_t fid = 0;
        if (flightlog.prepareFlight(fid) == tr_flightlog::Status::Ok)
        {
            const uint32_t bytes = logger.drainMramToSink();
            char name[40];
            snprintf(name, sizeof(name), "flight_mram_recovered_%lu.bin",
                     (unsigned long)fid);
            const auto fst = flightlog.finalizeFlight(name, bytes);
            ESP_LOGW("FLIGHTLOG", "#274: MRAM recovery -> %s (%lu B): %s",
                     name, (unsigned long)bytes, tr_flightlog::to_string(fst));
        }
        else
        {
            ESP_LOGE("FLIGHTLOG", "#274: MRAM recovery — prepareFlight failed (no space?)");
        }
        logger.finishMramRecovery();
    }

    // #846: re-seed the RAM snapshot cache from the NAND log stream — the
    // OC-also-reset half of in-flight reboot recovery on no-MRAM boards. The
    // snapshot frames rode the flight log (every received frame is enqueued
    // byte-exact), and the brownout scanner inside flightlog.begin() has
    // already recovered the un-finalized flight as flight_recovered_<id>.bin.
    //
    // The staleness defense is an NVS once-marker (id + final_bytes, since
    // ids are reused after deletes), and it is load-bearing: without it a
    // recovered flight could answer an FC panic weeks later with INFLIGHT and
    // re-arm pyro on the ground. Three rules make it airtight:
    //   * the marker is written on CONSUMPTION (after the frame is actually
    //     served) or on DECLINE — never merely on seeding, or a second reset
    //     before the FC asks would burn the flight's only chance;
    //   * a POWERON boot still MARKS (without seeding): a cold start is
    //     exactly when an old recovered flight must be retired, and a crash
    //     whose next boot is a full pack dropout would otherwise leave it
    //     unmarked for some later fault reset to seed from;
    //   * NVS unavailable => do not seed at all (fail CLOSED — with no way to
    //     record consumption, seeding could repeat indefinitely).
    // The seeded frame must itself be INFLIGHT: a stream ending in the FC's
    // LANDED clear means the flight is over. The FC re-validates
    // magic/version/INFLIGHT/CRC32/sim regardless, so this is belt on braces.
    if (!logger.isMramEnabled() && flightlog.isInitialized())
    {
        const bool cold_boot = (esp_reset_reason() == ESP_RST_POWERON);
        uint32_t best_id = 0;
        int best_idx = -1;
        uint32_t newest_id = 0;
        for (size_t i = 0; i < flightlog.index().size(); ++i)
        {
            const auto& e = flightlog.index().at(i);
            if (e.flight_id > newest_id) newest_id = e.flight_id;
            if (strncmp(e.filename, "flight_recovered_", 17) == 0 &&
                e.flight_id >= best_id)
            {
                best_id = e.flight_id;
                best_idx = (int)i;
            }
        }
        if (best_idx >= 0 && best_id == newest_id)
        {
            const auto& e = flightlog.index().at((size_t)best_idx);
            Preferences rp;
            if (!rp.begin("snaprec", false))
            {
                ESP_LOGW("FLIGHTLOG", "#846: snaprec NVS unavailable — not "
                         "re-seeding (cannot track consumption)");
            }
            else
            {
                const uint32_t done_id = rp.getUInt("done_id", 0);
                const uint32_t done_by = rp.getUInt("done_by", 0);
                const bool already = (done_id == e.flight_id &&
                                      done_by == e.final_bytes);
                if (already)
                {
                    // Nothing to do — this flight was seeded-and-served or
                    // declined on an earlier boot.
                }
                else if (cold_boot)
                {
                    // Retire it without seeding: a cold start is not a
                    // recovery context, and leaving it unmarked would let a
                    // later fault reset seed from an arbitrarily old flight.
                    rp.putUInt("done_id", e.flight_id);
                    rp.putUInt("done_by", e.final_bytes);
                    ESP_LOGI("FLIGHTLOG", "#846: %s retired on a cold boot "
                             "(not a recovery context)", e.filename);
                }
                else
                {
                    // Window buffer sized for the MAX per-page payload so one
                    // readFlightPage can always fill it at any chip geometry.
                    static uint8_t scan_buf[(4096 - 16) + kSnapFrameLen];
                    constexpr uint32_t kTailWindow = 64u * 1024u;
                    uint8_t last_f[kSnapFrameLen] = {};
                    uint8_t best_f[kSnapFrameLen] = {};
                    const bool got = tr_flightlog::tailScanForFrame(
                        flightlog, e.filename, e.final_bytes, SNAPSHOT_MSG,
                        (uint8_t)sizeof(FlightSnapshotData), kTailWindow,
                        last_f, scan_buf, sizeof(scan_buf),
                        best_f, offsetof(FlightSnapshotData, flight_elapsed_ms));
                    // The LAST frame decides whether the flight ended (a
                    // LANDED clear means it did); the HIGHEST-elapsed frame is
                    // the one to restore from, because an I2S DMA replay can
                    // leave an older snapshot sitting after a newer one in the
                    // stream — the same staleness the #383 guard blocks live.
                    const bool ended = !got || !snapshotServable(last_f, 0);
                    if (got && !ended && snapshotServable(best_f, 0))
                    {
                        portENTER_CRITICAL(&snapshot_cache_mux);
                        memcpy(snapshot_cache, best_f, kSnapFrameLen);
                        snapshot_cache_valid = true;
                        snapshot_cache_ms = millis();
                        portEXIT_CRITICAL(&snapshot_cache_mux);
                        snapshot_seed_pending_mark = true;
                        snapshot_seed_id = e.flight_id;
                        snapshot_seed_bytes = e.final_bytes;
                        ESP_LOGW("FLIGHTLOG", "#846: snapshot cache re-seeded "
                                 "from %s (%lu B) — in-flight reboot recovery "
                                 "armed for the FC's next ask",
                                 e.filename, (unsigned long)e.final_bytes);
                    }
                    else
                    {
                        // Declined: nothing found, or the stream's last word
                        // was a LANDED clear / an out-of-bounds elapsed. Mark
                        // now — a rescan next boot would decline identically.
                        rp.putUInt("done_id", e.flight_id);
                        rp.putUInt("done_by", e.final_bytes);
                        ESP_LOGW("FLIGHTLOG", "#846: no in-flight snapshot in "
                                 "the tail of %s — recovery not re-seeded",
                                 e.filename);
                    }
                }
                rp.end();
            }
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
            prefs.putUChar("hopdis", 1);  // #150: fixed mode is the default
            ESP_LOGI("CFG", "LoRa NVS empty -- wrote config.h defaults");
        }
        lora_freq_mhz = prefs.getFloat("freq", config::LORA_FREQ_MHZ);
        lora_sf        = prefs.getUChar("sf",   config::LORA_SF);
        lora_bw_khz    = prefs.getFloat("bw",   config::LORA_BW_KHZ);
        lora_cr        = prefs.getUChar("cr",   config::LORA_CR);
        lora_tx_power  = (int8_t)prefs.getChar("txpwr", config::LORA_TX_POWER_DBM);
        lora_hop_disabled = prefs.getUChar("hopdis", 1) != 0;  // #150: default fixed mode
        // "LoRa off" mute.  Deliberately NOT part of the first-boot seeding
        // block above, and read whether or not "freq" exists: the mute is
        // settable over BLE with the FC rail OFF, i.e. before this function
        // has ever run and written a "freq" key, so a seed-on-first-boot
        // would silently un-mute the rocket the first time the rail came up.
        // An absent key reads 0 = transmitting, the right default anyway.
        lora_tx_disabled = prefs.getUChar("txdis", 0) != 0;

        // Issue #136: every rocket boot starts on the hardcoded
        // rendezvous frequency + preset regardless of any NVS values
        // left from prior sessions.  The BS reaches here on rendezvous
        // and uses the cmd-10 transactional flow to move us as needed.
        // cmd-10's NVS write still happens during a session for
        // visibility, but boot always re-resets here so the rendezvous
        // is reliable.
        // #150: lora_hop_disabled is deliberately NOT in this override
        // anymore — the link mode is user-selected (app picker → BS →
        // uplink cmd 17) and honors NVS across reboots.  Schema v4
        // wiped any stale pre-#150 hopdis, and the default is 1 (fixed).
        lora_freq_mhz     = LORA_FACTORY_RENDEZVOUS_MHZ;
        lora_sf           = LORA_FACTORY_RENDEZVOUS_SF;
        lora_bw_khz       = LORA_FACTORY_RENDEZVOUS_BW_KHZ;
        lora_cr           = LORA_FACTORY_RENDEZVOUS_CR;
        lora_tx_power     = LORA_FACTORY_RENDEZVOUS_TX_DBM;

        // Channel-set restore (#40 / #41 phase 3): skip-mask pushed by
        // the BS via cmd 15.  Skip-mask is keyed off the BW it was
        // generated for; if NVS BW != active BW, discard the mask (the
        // BS will re-push after the user re-runs the scan).  Rendezvous
        // freq is no longer NVS-stored — see #105.
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
        cfg_roll_min_speed = prefs.getFloat("rmspd", cfg_roll_min_speed);
        cfg_rate_cap_dps   = prefs.getFloat("rcap", cfg_rate_cap_dps);   // #253
        cfg_kp_angle       = prefs.getFloat("kpang", cfg_kp_angle);      // #253
        cfg_iwind_dps      = prefs.getFloat("iwind", cfg_iwind_dps);     // #253
        prefs.end();
        ESP_LOGI("CFG", "NVS Roll control: angle_ctrl=%s delay=%u ms minspd=%.1f rcap=%.1f kpang=%.2f iwind=%.1f",
            cfg_use_angle_ctrl ? "ON" : "OFF", (unsigned)cfg_roll_delay_ms,
            (double)cfg_roll_min_speed,
            (double)cfg_rate_cap_dps, (double)cfg_kp_angle, (double)cfg_iwind_dps);

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

        // Load cached IMU mounting orientation setting from NVS.  A MANUAL
        // value gets pushed to the FC by the status-query self-heal once
        // the FC starts polling — no explicit boot staging needed.
        prefs.begin("orient", false);
        cfg_imu_orient = prefs.getUChar("set", cfg_imu_orient);
        prefs.end();
        ESP_LOGI("CFG", "NVS IMU orientation: %s",
                 cfg_imu_orient == IMU_ORIENT_AUTO
                     ? "AUTO" : orientCodeName(cfg_imu_orient));

        // Load cached IMU logging rate.  Readback/cache only — the FC owns
        // application (its own NVS survives FC reboots independently).
        prefs.begin("imurate", false);
        {
            const uint16_t nvs_rate = prefs.getUShort("hz", cfg_imu_rate);
            // Whitelist on read: a corrupted value must not be relayed to the
            // FC or echoed to the app as if it were a real setting.
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

    if (config::USE_LORA_RADIO)
    {
        bool radio_ok = false;
        if (config::USE_UART_RADIO_MODEM)
        {
            // V8: LoRa daughterboard over UART (#409/#410). NVS-restored
            // modulation params are pushed to the modem; pins/baud from the
            // board header (baud must match projects/radio_board).
            UartModemBackend::Config mcfg = {};
            mcfg.uart.tx_pin = config::LORA_UART_TX_PIN;
            mcfg.uart.rx_pin = config::LORA_UART_RX_PIN;
            mcfg.act_pin = config::LORA_ACT_PIN;
            mcfg.preamble_len = config::LORA_PREAMBLE_LEN;
            mcfg.crc_on = config::LORA_CRC_ON;
            mcfg.rx_boosted_gain = config::LORA_RX_BOOSTED_GAIN;
            mcfg.syncword_private = config::LORA_SYNCWORD_PRIVATE;
            radio_ok = lora_modem_backend.begin(mcfg, lora_freq_mhz, lora_sf,
                                                lora_bw_khz, lora_cr,
                                                lora_tx_power, config::DEBUG);
        }
        else
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
            radio_ok = lora_direct_backend.begin(lora_cfg, config::DEBUG);
        }
        if (!radio_ok)
        {
            ESP_LOGE("PWR", "LoRa init failed");
        }
    }

    vTaskDelay(1);  // feed watchdog after LoRa init

    // I2C slave init is deferred to main loop — see initI2CSlave().
    // We wait until I2S DMA callbacks confirm the FC is alive, then
    // init the slave so the bus is stable.
    ESP_LOGI("PWR", "I2C slave deferred (waiting for FC I2S activity)");

    // Mutex serializing i2s_stream across the loop_oc flip/revert and the BLE
    // task's image pump during an FC OTA (Phase 4 Layer 3). Created before the
    // channel so it's ready if an OTA starts immediately after power-on.
    if (oc_i2s_mutex == nullptr)
        oc_i2s_mutex = xSemaphoreCreateMutex();
    // Image-pump frame queue + feeder task (Phase 4 Layer 3). 16 frames (~4 KB)
    // is several BLE chunks deep; the feeder drains far faster than BLE fills.
    if (oc_ota_tx_queue == nullptr)
        oc_ota_tx_queue = xQueueCreate(16, sizeof(OcOtaTxFrame));

    // I2S telemetry stream from FlightComputer (DMA-based slave RX)
    // Small DMA buffers (4 × 512 bytes = 2 KB) minimize latency.
    // FRAME_SYNC interrupt gating prevents stale replay regardless of
    // buffer count, but smaller buffers reduce read-to-parse latency.
    // dma_frame_num doubled 64 -> 128 alongside the 22050 -> 44100 link rate
    // so each descriptor still spans ~2.9 ms (512 B at 176 KB/s) — the same
    // callback cadence the parser was tuned against at the old rate.
    // #834 items 6/7 (review): go through the SAME helper the recovery paths use
    // so a boot failure sets oc_i2s_rx_broken and loop_oc's 1 Hz retry picks it
    // up. Open-coding begin+register here left the most consequential failure of
    // all — no RX from boot — as the one case the new never-give-up retry did
    // not cover, with the retry loop idle one screen away.
    // The mutex may not exist yet at this point in setup; take it if it does.
    if (oc_i2s_mutex != nullptr) xSemaphoreTake(oc_i2s_mutex, portMAX_DELAY);
    const esp_err_t rx_err = ocBeginSlaveRxLocked("boot");
    if (oc_i2s_mutex != nullptr) xSemaphoreGive(oc_i2s_mutex);
    if (rx_err != ESP_OK)
    {
        ESP_LOGE("PWR", "I2S slave RX init failed — retrying from loop_oc");
    }
    else
    {
        ESP_LOGI("PWR", "I2S slave RX ready");
    }
    // Tasks are created unconditionally: a boot failure is now recoverable, and
    // a channel that comes back via the retry needs a parser to drain rx_ring.
    // Both are harmless with no channel — i2sRecvCallback null-checks
    // i2s_rx_task_handle, and the feeder idles while !oc_ota_tx_mode.
    {
        // Parser task — woken by DMA callback, parses frames from rx_ring.
        // Pinned to Core 1 at priority 6 (one above loopTask at prio 5).
        // Parser preempts loopTask whenever DMA notifies; loopTask runs
        // in the gaps when parser blocks in ulTaskNotifyTake.  Parser is
        // bursty by design (sleeps when rx_ring is empty) so it can't
        // starve loopTask.  Originally created at prio 1 with a
        // misleading "round-robin" comment — the actual scheduling left
        // parser starved during high-throughput INFLIGHT, causing
        // rx_ring to fill until drop_oldest evicted bytes (#104).
        xTaskCreatePinnedToCore(i2sParserTask, "I2S Parse", 4096,
                                nullptr, 6, &i2s_rx_task_handle, 1);

        // OTA image-pump feeder (Phase 4 Layer 3). Sole I2S writer during an FC
        // OTA, keeping the master TX DMA continuously fed (frame-or-idle-fill)
        // so it never replays stale buffers. Idles when not in TX mode. Core 1 /
        // prio 6 like the parser; during OTA the RX parser is blocked (link is
        // flipped to TX), so the feeder gets the core to itself.
        if (oc_ota_feeder_task == nullptr)
            xTaskCreatePinnedToCore(ocOtaTxFeederTask, "OTA Feed", 4096,
                                    nullptr, 6, &oc_ota_feeder_task, 1);
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

// ==========================================================================
// SECTION: Boot setup (rail off, BLE only)
// ==========================================================================
static void setup_oc()
{
    // #825: FIRST thing, before NVS init and the 500 ms boot delay burn the
    // R84/C105 window (~0.8 s from the chip reset) — if the previous session
    // had the FC rail ON and this reset was not a real power-on or the
    // deliberate power-off reboot, re-assert the rail NOW. Without this, any
    // OC panic/WDT/brownout (or a self-OTA restart) mid-flight let the FC —
    // and its pyro channels — power off ~0.8 s later, ballistic.
    {
        const esp_reset_reason_t rst = esp_reset_reason();
        const bool magic_ok = (rail_rtc.magic == kRailRtcMagic);
        boot_rail_restored = RailRestorePolicy::shouldRestore(
            rst == ESP_RST_POWERON, magic_ok,
            magic_ok && rail_rtc.rail_on != 0,
            magic_ok && rail_rtc.deliberate_off != 0,
            magic_ok ? rail_rtc.restore_attempts : 0);
        if (!magic_ok || rst == ESP_RST_POWERON)
        {
            rail_rtc = {kRailRtcMagic, 0, 0, 0};
        }
        rail_rtc.deliberate_off = 0;   // one-shot: consumed by this boot
        if (magic_ok && rail_rtc.rail_on != 0 && !boot_rail_restored &&
            rail_rtc.restore_attempts >= RailRestorePolicy::kMaxRestoreAttempts)
        {
            // Retry budget exhausted: stand down to the stable rail-off idle
            // (the pre-#825 endpoint) instead of brownout-cycling the rail.
            rail_rtc.rail_on = 0;
            rail_rtc.restore_attempts = 0;
        }
        if (boot_rail_restored)
        {
            rail_rtc.restore_attempts++;   // cleared when the restore proves stable
            pinMode(config::PWR_PIN, OUTPUT);
            digitalWrite(config::PWR_PIN, HIGH);
            if (config::GPS_PWR_PIN >= 0)
            {
                pinMode(config::GPS_PWR_PIN, OUTPUT);
                digitalWrite(config::GPS_PWR_PIN, HIGH);
            }
        }
    }

    // Ensure NVS is initialised (ESP-IDF on ESP32-P4/S3 may not auto-init)
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }

    delay(500);

    if (!boot_rail_restored)
    {
        pinMode(config::PWR_PIN, OUTPUT);
        digitalWrite(config::PWR_PIN, LOW);   // Start with power rail OFF
        if (config::GPS_PWR_PIN >= 0)  // -1: board has no separate GPS rail (#411)
        {
            pinMode(config::GPS_PWR_PIN, OUTPUT);
            digitalWrite(config::GPS_PWR_PIN, LOW);  // GPS enable rail off at boot
        }
    }
    else
    {
        ESP_LOGE("PWR", "FC rail RE-ASSERTED at boot: previous session had it "
                        "ON and this was not a power-on or a deliberate "
                        "power-off (#825). Peripherals re-init after setup.");
    }
    pwr_pin_on = boot_rail_restored;

    ESP_LOGI("OC", "Starting OutComputer (low-power mode)...");
    // V9 selects the same map as V8 on this MCU (see config.h) — reported
    // separately so the boot log says which board the image was built for.
    ESP_LOGW("OC", "[BOARD] pin map: %s",
             TR_BOARD_V9 ? "V9/V10 (same pins as V8)" : (TR_BOARD_V8 ? "V8" : "V7"));

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
                ESP_LOGW("OC", "OTA: running on PENDING_VERIFY image (partition '%s' @ 0x%08x); "
                              "rollback armed until first telemetry round-trip",
                         running->label, (unsigned)running->address);
                g_ota_pending_verify = true;
            }
            else
            {
                ESP_LOGI("OC", "OTA: running partition '%s' state=%d", running->label, (int)state);
            }
        }
    }

    // --- Load NVS settings early so config readback to app is correct ---
    {
        prefs.begin("lora", false);

        // NVS schema gate (#105 follow-up).  Stored != current → clear the
        // entire lora namespace so we fall back to the shared factory
        // rendezvous values and let the BS re-sync us via the standard
        // rendezvous flow.  Mirrors the BS side; both must agree on
        // LORA_NVS_SCHEMA_VERSION (defined in RocketComputerTypes.h).
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
            lora_hop_disabled = prefs.getUChar("hopdis", 1) != 0;  // #150: default fixed mode
            ESP_LOGI("CFG", "NVS LoRa early load (cached): %.1f MHz SF%u BW%.0f CR%u %d dBm hop_disabled=%d",
                          (double)lora_freq_mhz, (unsigned)lora_sf,
                          (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power,
                          (int)lora_hop_disabled);
        }
        // Outside the isKey("freq") gate on purpose: "LoRa off" is settable
        // over BLE with the rail off, so a board that has never run
        // initPeripherals() can hold a mute and no "freq" key at all.  Read
        // here as well as there so the config readback the app pulls before
        // power-on already tells the truth about the radio.
        lora_tx_disabled = prefs.getUChar("txdis", 0) != 0;
        if (lora_tx_disabled)
        {
            ESP_LOGW("CFG", "NVS LoRa: transmit MUTED (\"LoRa off\") — no telemetry "
                            "or beacon will go out until it is turned back on");
        }
        prefs.end();

        // Issue #136: same boot-time rendezvous override as the later
        // peripheral init.  Done here too so any code between this early
        // load and initPeripherals() (e.g., config-readback construction)
        // sees the rendezvous values rather than stale NVS.  #150: the
        // hopdis override is gone — link mode honors NVS (see the
        // peripheral-init comment).
        lora_freq_mhz     = LORA_FACTORY_RENDEZVOUS_MHZ;
        lora_sf           = LORA_FACTORY_RENDEZVOUS_SF;
        lora_bw_khz       = LORA_FACTORY_RENDEZVOUS_BW_KHZ;
        lora_cr           = LORA_FACTORY_RENDEZVOUS_CR;
        lora_tx_power     = LORA_FACTORY_RENDEZVOUS_TX_DBM;

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
        cfg_roll_min_speed = prefs.getFloat("rmspd", 0.0f);  // 0 = speed gate off
        cfg_rate_cap_dps   = prefs.getFloat("rcap", 0.0f);   // #253 (<=0 = fw default)
        cfg_kp_angle       = prefs.getFloat("kpang", 0.0f);  // #253 (<=0 = fw default)
        cfg_iwind_dps      = prefs.getFloat("iwind", -1.0f); // #253 (<0 = fw default)
        prefs.end();

        // Early identity load (so config readback on first connect is correct)
        uint8_t mac[6];
        esp_efuse_mac_get_default(mac);
        snprintf(unit_id_hex, sizeof(unit_id_hex), "%02x%02x%02x%02x",
                 mac[2], mac[3], mac[4], mac[5]);

        prefs.begin("identity", false);

        // #150: the identity namespace versions SEPARATELY from the lora
        // namespace and MIGRATES instead of wiping — a wipe here is what
        // caused the #133-era regression (BS came back nid=0, rocket kept
        // nid=180, network-id filter silently dropped everything).
        //
        // v0 -> v1 resets nid to the compile-time default ONCE (review
        // finding): the #136 boot-force this branch removes was RAM-only,
        // so fielded devices still store whatever nid the #133 era left
        // behind — values that have been masked, divergent, and
        // unreachable for months.  Making them live would kill the link
        // on the first post-flash boot with no over-the-air recovery
        // (both directions are nid-filtered).  The pre-upgrade EFFECTIVE
        // nid was always the default, so resetting to it is the
        // behavior-preserving migration.  `un` (user data) and `rid`
        // (never boot-forced) are preserved.
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

        // #150: the #136 boot-time force of network_id back to default is
        // gone.  Per-network IDs are user-set again (BLE cmd 41 / the
        // network-name UI), survive reboot, and drift is now VISIBLE
        // instead of silent: both ends count nid-mismatch drops (the BS
        // surfaces its count to the app as "nidd") and identity NVS
        // migrates rather than wipes.

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
        // #297: gate ina230_ok on a successful calibrate so a bad calibration
        // (guarded divide returns ERROR) can't leave us reading a flat 0 A with
        // an "OK" status.
        if (ina230.calibrate(INA230_R_SHUNT_OHM, INA230_CURRENT_LSB_A) == TR_INA230_OK)
        {
            ina230.enableConversionReadyAlert(true);  // enable CVRF bit for polling
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

    // #850: the rail-current ADC is independent of the INA230 — bring it up
    // whether or not the gauge answered, so a dead or absent INA230 does not
    // also cost the camera/servo current readings.
    initRailCurrentAdc();

    // #519: the OC owns its connection-parameter policy — slow (200 ms, latency 4)
    // while the rail is off to save idle power, fast (30 ms) once it comes on for
    // file transfer. TR_BLE_To_APP's own connect-time request is for the base
    // station, which has no policy; here the two collide, and the bench caught it:
    //
    //   BLE: Slow (low-power) conn params requested (handle=1)
    //   NimBLE: GAP update_params: update already in progress; conn_handle=0x0001
    //   W BLE: Connection param update request failed to send, rc=2
    //
    // The OC won that race, but had it lost, the idle link would have stayed at
    // 30 ms and thrown away most of the power saving. Take ownership instead of
    // relying on who gets there first.
    ble_app.setAutoConnParams(false);

    // #541: must run BEFORE ble_app.begin() — the BT controller samples the
    // RTC slow-clock source once at init to choose its low-power clock.
    retry32kCrystal();

    // Only BLE starts at boot — everything else is behind PWR_PIN
    if (!ble_app.begin())
    {
        ESP_LOGE("BLE", "BLE app interface failed to start");
    }
    // Relay target==1 (Flight Computer) OTA to the FC over I2C (#8 Phase 4).
    ble_app.setOtaRelayDelegate(ocOtaRelayBegin, ocOtaRelayFinish, ocOtaRelayAbort,
                                ocOtaRelayData, nullptr);

    if (boot_rail_restored)
    {
        // #825: the rail came back up at the top of setup. Stay OUT of
        // low-power mode — the FC is running — but DEFER the cmd-8 power-ON
        // side effects (initPeripherals etc.) to the first loop_oc iteration:
        // loop_oc is pinned to core 1, where every other initPeripherals call
        // runs, and the I2S RX GDMA / LoRa interrupts are allocated on the
        // calling core. Running them here (setup_oc = core 0 = the BT core)
        // would park the ~3 ms-cadence I2S ISR against the BLE stack for the
        // rest of the resumed flight.
        boot_rail_restore_init_pending = true;
        // #825: the restored FC may still be mid-recording — our reset wiped
        // the commanded camera state, so the explicit-state dedup must not
        // trust it until the first command re-synchronizes it.
        camera_state_known = false;
        last_stats_ms = millis();
        ESP_LOGW("OC", "OutComputer ready (FC rail RESTORED after reset — "
                       "peripheral re-init deferred to the loop task).");
    }
    else
    {
        // Enter low-power mode: 80 MHz, DFS to 40 MHz when idle
        enterLowPowerMode();

        last_stats_ms = millis();
        ESP_LOGI("OC", "OutComputer ready (PWR_PIN OFF, waiting for power-on command).");
    }
}

// ==========================================================================
// SECTION: Main loop
// ==========================================================================
// ============================================================================
// loop_oc stall instrumentation (#90 follow-up — periodic 745 ms Core-1 stall)
// ============================================================================
// Bench analysis showed all six sensor streams freezing for ~745 ms every
// ~15 s with the Core-1 sensor pipeline resuming within 5 ms across streams
// — a single Core-1 blocker preempting the I2S parser long enough to
// overflow the DMA ring.  These macros log any blocking call inside loop_oc
// (or the LoRa internals) that exceeds LOOP_STALL_THRESHOLD_US so the next
// bench run names the offending op directly.  Modeled on the existing
// LFS_TIMING / STALL_THRESHOLD_US instrumentation in TR_LogToFlash.cpp.
static constexpr int64_t LOOP_STALL_THRESHOLD_US = 100'000;  // 100 ms

// Idle (FC-off / low-power) loop period.  loop_oc drains ONE BLE command per
// iteration via ble_app.getCommand().  This used to be a correctness
// constraint: the BLE command buffer was a single slot, so a loop slower than
// the ~60-90 ms connect-time command spacing let a burst overwrite itself —
// the root cause of #221 (was delay(1000)).  #517 gave that buffer depth, so a
// burst now queues instead of collapsing and correctness no longer rides on the
// loop period.  20 ms stays for RESPONSIVENESS (a 16-deep ring still drains one
// command per pass, and in-loop connection setup lagged up to 1 s at delay(1000)),
// with negligible power cost — light sleep is disabled while BLE is on, so the
// CPU idles at 40 MHz either way.
static constexpr uint32_t IDLE_LOOP_DELAY_MS = 20;

#define LOOP_STALL_INSTR(name, expr) do {                                       \
    const int64_t _stall_t0_ = esp_timer_get_time();                            \
    expr;                                                                       \
    const int64_t _stall_dt_ = esp_timer_get_time() - _stall_t0_;               \
    if (_stall_dt_ > LOOP_STALL_THRESHOLD_US) {                                 \
        ESP_LOGW("LOOP_STALL", "%s took %lld us", (name), (long long)_stall_dt_); \
    }                                                                           \
} while (0)

static void loop_oc()
{
    // #825: one-shot completion of a boot rail restore, on THIS task/core
    // (core 1) so the I2S RX GDMA and LoRa interrupts land where every other
    // initPeripherals() call puts them. The rail has been up since the top of
    // setup_oc — far longer than the cmd-8 path's 500 ms settle.
    if (boot_rail_restore_init_pending)
    {
        boot_rail_restore_init_pending = false;
        vTaskDelay(1);  // feed watchdog before long init
        initPeripherals();
        if (ina230_ok) {
            ina230.setConfiguration(INA230_Avg::AVG_1,
                                    INA230_ConvTime::CT_332us,
                                    INA230_ConvTime::CT_332us,
                                    INA230_Mode::POWER_DOWN);
            ina_continuous = false;
        }
        // The restore has proven stable (system up, peripherals in): reset
        // the brownout retry budget so a LATER fault gets a fresh allowance.
        rail_rtc.restore_attempts = 0;
        ESP_LOGW("OC", "FC rail restore complete — peripherals re-initialized "
                       "on the loop core, telemetry resuming.");
    }

    // #846: a boot-re-seeded snapshot is marked consumed only after the FC has
    // actually been served it — writing the marker at seed time would burn the
    // flight's one chance if another reset wiped the RAM cache before the ask.
    // Done here (loop context) so the NVS write never lands in the I2C ingress
    // path that serves the frame.
    if (snapshot_seed_pending_mark && snapshot_served)
    {
        snapshot_seed_pending_mark = false;
        Preferences rp;
        if (rp.begin("snaprec", false))
        {
            rp.putUInt("done_id", snapshot_seed_id);
            rp.putUInt("done_by", snapshot_seed_bytes);
            rp.end();
            ESP_LOGI("FLIGHTLOG", "#846: re-seeded snapshot consumed by the FC "
                     "— flight %lu marked evaluated",
                     (unsigned long)snapshot_seed_id);
        }
    }

    // Serial debug console removed (was Arduino Serial.available/read).
    // Use ESP-IDF console component or BLE commands for debug interaction.
    const int64_t _loop_oc_t0 = esp_timer_get_time();

    // Preemption catch: wall time spent OUTSIDE this function, measured from
    // the previous iteration's exit to this one's entry.  The catch-all below
    // times the body only, so it is structurally blind to the case where
    // oc_loop passes its body-end timer, yields at vTaskDelay(1), and then a
    // higher-priority Core-1 task (the I2S parser at prio 6, NimBLE, an IRQ)
    // holds the core for a long stretch before oc_loop is scheduled again.
    // That window looks like a clean loop from inside the body but still gaps
    // every stream the loop feeds.
    //
    // Measured exit→entry, not entry→entry, so a slow body is reported once by
    // the catch-all rather than twice.  Nominal gap is the ~1 ms vTaskDelay(1)
    // (CONFIG_FREERTOS_HZ=1000) in both active and idle mode — the 20 ms
    // IDLE_LOOP_DELAY_MS sleep happens inside the body — so at a 100 ms
    // threshold this only fires on a genuine multi-tick deschedule.
    static int64_t _loop_oc_last_exit_us = 0;
    if (_loop_oc_last_exit_us != 0) {
        const int64_t _preempt_dt = _loop_oc_t0 - _loop_oc_last_exit_us;
        if (_preempt_dt > LOOP_STALL_THRESHOLD_US) {
            ESP_LOGW("LOOP_STALL",
                     "loop_oc off-core %lld us between iterations (preempted)",
                     (long long)_preempt_dt);
        }
    }

    // #398 item 3: drain one paced config-readback frame (if any) per pass.
    // Outside the pwr_pin_on gate so connect-time readback (low-power mode)
    // drains too. Replaces the old inline delay(50) chain in sendCurrentConfig.
    serviceConfigReadbackQueue();

    // --- Active mode: FlightComputer + sensors powered on ---
    if (pwr_pin_on)
    {
        // Phase 4 Layer 3: service the I2S direction flip for an FC OTA image
        // pump. Done here (not in the BLE/parser task) so all i2s_stream
        // lifecycle calls live in one place. After READY we wait for the FC's
        // I2S TX to go quiet — proof it has flipped to slave RX and released
        // BCLK — before we flip to master TX, so the two never drive BCLK at
        // once. (The FC idle-fills until it flips, so quiet == flipped.)
        if (oc_ota_await_flip && !oc_ota_tx_mode)
        {
            const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if (dma_cb_count != oc_ota_silence_ref_count)
            {
                oc_ota_silence_ref_count = dma_cb_count;   // still receiving — reset timer
                oc_ota_silence_since_ms  = now_ms;
            }
            else if ((now_ms - oc_ota_silence_since_ms) >= OTA_FLIP_SILENCE_MS)
            {
                oc_ota_await_flip = false;
                ocFlipToTx();                              // FC is quiet → safe to drive
                // Don't release "ready" yet: the FC's slave RX needs a few ms to
                // lock onto the just-started master BCLK. Offset 0 pumped before
                // then is lost and the forward-only receiver never recovers
                // (bench: frames_ok=0, every frame "gap"). Start a warmup window —
                // the feeder idle-fills (BCLK live) so RX can lock; the gate below
                // releases the app once it has. #15.
                if (oc_ota_tx_mode)
                    oc_ota_warmup_since_ms = now_ms;
            }
        }
        // Post-flip RX warmup gate: hold the app's "ready" until the FC slave RX
        // has had time to lock onto BCLK (feeder idle-filling meanwhile), so the
        // first real image frame (offset 0) lands. (Held since READY so the app
        // can't drop offset 0 into a not-yet-locked link.) #8 Phase 4 / #15.
        if (oc_ota_tx_mode && oc_ota_relay_ready_pending && oc_ota_warmup_since_ms != 0)
        {
            const uint32_t warm_now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if ((warm_now_ms - oc_ota_warmup_since_ms) >= OTA_RX_WARMUP_MS)
            {
                oc_ota_relay_ready_pending = false;
                oc_ota_warmup_since_ms     = 0;
                // #834 item 7: arm the stall watchdog only now. Before "ready"
                // the silence is deliberate (the warmup window idle-fills so the
                // FC's slave RX can lock onto BCLK), so arming earlier would
                // abort every OTA.
                oc_ota_last_chunk_ms = warm_now_ms;
                ble_app.relayFcOtaStatus("ready", nullptr, 0, false);
            }
        }
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
            LOOP_STALL_INSTR("serviceI2CIngress", serviceI2CIngress());

        // Launch-triggered logging: start when NSF_LAUNCH appears in NonSensorData
        {
            static bool prev_ns_launch = false;
            // #317: once the vehicle has reported LANDED, refuse to start a new
            // flight log until a reboot. Post-flight ground handling can re-trip
            // the FC launch-detect and would otherwise open a bogus session that
            // only closes on power-off (-> a junk recovered_*.bin full of ground
            // data). Reset by an OC reboot, or by a sim re-arm — the FC leaving
            // LANDED, which is impossible in a real flight (terminal lockout).
            static bool oc_landed_lockout = false;
            static RocketState prev_rs_lockout = INITIALIZATION;
            if (latest_rocket_state == LANDED)
            {
                oc_landed_lockout = true;
            }
            else if (prev_rs_lockout == LANDED)
            {
                // #317 sim re-arm: the FC only leaves LANDED on a deliberate new
                // sim run (real flight is terminal), so clear the lockout to match
                // the FC's sim-start re-arm and let the new sim flight log.
                oc_landed_lockout = false;
            }
            prev_rs_lockout = latest_rocket_state;
            const bool ns_launch = latest_non_sensor_valid &&
                                   nsFlagSet(latest_non_sensor.flags, NSF_LAUNCH);
            if (ns_launch && !prev_ns_launch && !oc_landed_lockout)
            {
                // Mirror the cmd 23 lifecycle so a launch detected without a
                // prior PRELAUNCH transition still gets a flightlog flight
                // and an opened sink session. Each call is a no-op when the
                // matching state has already been set, so the normal
                // PRELAUNCH→LAUNCH path is unaffected.
                logger.prepareLogFile();
                flightlogBeginFlight();
                logger.startLogging();
                ESP_LOGI("OC", "Launch detected - logging started");
            }
            prev_ns_launch = ns_launch;
        }
        LOOP_STALL_INSTR("logger.service", logger.service());

        // Read power data at ~100 Hz (always, so BLE telemetry has fresh data
        // even before launch).  Only log to flash when logging is active.
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

        // Slow-rendezvous cycle: brief visits to LORA_FACTORY_RENDEZVOUS_MHZ when
        // the rocket has been silent in READY for a long time, so the base
        // station's Phase-A recovery has a meeting point (issue #71).
        LOOP_STALL_INSTR("serviceRocketRendezvous", serviceRocketRendezvous());

        // #150: deferred hop-enable (see the cmd-17 handler) — activate
        // once the BS's mirror-retry train has finished so the bootstrap
        // packet lands on a listening BS.
        if (hop_enable_apply_at_ms != 0 &&
            (int32_t)(millis() - hop_enable_apply_at_ms) >= 0)
        {
            hop_enable_apply_at_ms = 0;
            updateHopFromState(latest_rocket_state);
        }

        // Hop-silence rendezvous: same idea but gated for the hopping
        // case (#40 / #41 phase 2b).  Active only while hop_active_ and
        // suppressed when slow_rendezvous owns the radio.
        LOOP_STALL_INSTR("serviceHopFallback", serviceHopFallback());

        // Send name beacon so base station can identify us
        LOOP_STALL_INSTR("sendLoRaBeacon", sendLoRaBeacon());
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
                    const bool read_ok =
                        ina230.readBusVoltage_V(&bus_v) == TR_INA230_OK &&
                        ina230.readCurrent_A(&current_a) == TR_INA230_OK;
                    // Same validity + SOC + commit policy as the triggered path
                    // (#272).  Continuous mode is free-running — no CVRF to check.
                    commitPowerSample(read_ok ? bus_v : 0.0f, read_ok ? current_a : 0.0f);
                }
            }
        }

        // Yield to FreeRTOS.  This was delay(1000) for power; a 1 s loop period
        // let a connect-time command burst overwrite the then-single-slot BLE
        // command buffer (only the last survived) and lagged in-loop connection
        // setup up to 1 s, surfacing as flaky connects / dropped config+cal sync
        // (#221).  The buffer is now a ring (#517) so a burst queues rather than
        // collapsing, but stay responsive anyway: one command drains per pass,
        // and connection setup still runs in this loop.  Light sleep is disabled
        // while BLE is on, so the CPU idles at 40 MHz (DFS min) regardless and
        // the long delay saved little real power.
        delay(IDLE_LOOP_DELAY_MS);

    }

    // ----------------------------------------------------------------------
    // I2S link recovery — deliberately OUTSIDE the pwr_pin_on gate above.
    // #834 items 6/7: every path that can strand the OC's I2S is serviced here.
    // Losing slave RX means no FC telemetry, no logging and two frozen
    // downlinks until a power cycle, so recovery must not depend on the app,
    // the FC, or the switched rail being on — a rail-off mid-relay used to
    // strand it exactly this way, because the revert lived inside that gate.
    // ----------------------------------------------------------------------
    {
        const uint32_t i2s_now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        // The app vanished mid-transfer (walked out of range, screen lock, app
        // killed). onDisconnect stages an abort, but a link that simply stops
        // pumping without disconnecting leaves no event at all — hence a
        // timeout on chunk arrival as well.
        if (OtaRelayPolicy::relayStalled(oc_ota_tx_mode, oc_ota_last_chunk_ms,
                                         i2s_now_ms))
        {
            ESP_LOGE("OC", "OTA relay: no chunk for %u ms while flipped — "
                           "abandoning and reverting to slave RX",
                     (unsigned)OtaRelayPolicy::kRelayStallTimeoutMs);
            oc_ota_last_chunk_ms          = 0;   // disarm before the revert
            setPendingCommand(OTA_ABORT_CMD);
            oc_ota_revert_to_rx_requested = true;
            // End the BLE-side session too. A stall with the link still UP
            // produces no onDisconnect, so without this ota_relay_active_ and
            // ota_session_active_ stay set: battery/power reads remain gated
            // off and a later OTA_BEGIN is rejected as already-in-progress.
            ble_app.relayFcOtaStatus("aborted", "relay_stalled", 0, true);
            ocOtaRelayClearPendingFlip();
        }

        if (oc_ota_revert_to_rx_requested)
        {
            // Clear AFTER the revert completes, not before. The feeder watches this
            // flag to back off oc_i2s_mutex so ocRevertToRx() can acquire it (see the
            // feeder's comment). Clearing first would let the higher-priority feeder
            // resume mid-revert and starve ocRevertToRx()'s mutex take — the FINISH
            // deadlock. ocRevertToRx() sets oc_ota_tx_mode=false, so the feeder idles
            // regardless once it returns.
            ocRevertToRx();
            oc_ota_revert_to_rx_requested = false;
        }

        // A begin that failed anywhere above leaves us with no channel. Keep
        // retrying: a transient DMA-allocation failure (BLE buffers, the
        // flightlog scratch and the 64 KB rx_ring are all live during a relay)
        // costs about a second of telemetry instead of the whole flight.
        // Re-sample: ocRevertToRx() above sleeps 100 ms and stamps
        // oc_i2s_rx_last_try_ms, which would then be AHEAD of the i2s_now_ms
        // captured at the top of this block — unsigned subtraction turns that
        // into ~49 days and fires the retry immediately.
        const uint32_t retry_now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (OtaRelayPolicy::shouldRetryRx(oc_i2s_rx_broken, retry_now_ms,
                                          oc_i2s_rx_last_try_ms) &&
            oc_i2s_mutex != nullptr)
        {
            xSemaphoreTake(oc_i2s_mutex, portMAX_DELAY);
            i2s_stream.end();          // drop whatever half-state is there
            ocBeginSlaveRxLocked("retry");
            xSemaphoreGive(oc_i2s_mutex);
        }
    }

    // Auto-send config once the BLE connection is fully ready — MTU negotiated
    // AND the app has enabled notifications on the telemetry/config char.
    // Edge-armed on connect, then deferred (no blocking delay) until ready.
    // Pushing earlier dropped the config notifies (#224): the faster idle loop
    // (#221) made the old fixed delay(500) race the MTU exchange + CCCD
    // subscribe, so the pushes were skipped (recovered only by the app's later
    // config re-request).  ble_app.isReadyForNotify() is the deterministic gate.
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
            // #542: in low-power mode, SCHEDULE the slow (idle) params instead
            // of requesting them here. This point is ~1 s after connect — the
            // middle of the app's connect burst — and dropping to the 200 ms/
            // latency-4 set here made every remaining handshake exchange cost
            // ~500 ms (see kSlowParamsDeferMs). Let the burst run on the fast
            // params onConnect requested, then drop.
            if (!pwr_pin_on) {
                slow_params_due_ms = (uint32_t)millis() + kSlowParamsDeferMs;
            }
        }
        // #542: fire the deferred drop once the burst window has passed.
        // Rail-on cancels it (fast params are the powered-on policy — see the
        // transfer paths); disconnect cancels above.
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

    // Re-publish the FC version to the app when it changes mid-connection: an FC
    // OTA reboots the FC (new version) but never drops the OC<->app BLE link, so
    // the connect-time fc_identity won't refresh on its own. This is what lets
    // the app's FC-OTA verify see the new version (or catch a rollback). #8.
    if (fc_identity_dirty && ble_app.isConnected() && ble_app.isReadyForNotify())
    {
        fc_identity_dirty = false;
        sendFcIdentity();
    }

    // Same pattern for the board→rocket orientation: the pad auto-detect
    // can re-orient mid-connection, and the app's pre-arm display should
    // follow without a reconnect.
    if (imu_orient_dirty && ble_app.isConnected() && ble_app.isReadyForNotify())
    {
        imu_orient_dirty = false;
        sendImuOrientation();
    }

    // Same pattern for the guidance-target echo (#435): the FC bumps tgt_seq
    // on every processed cmd 28 (and on echo-changing cmd-65 applies), the
    // status-query ingest flags the change, and this pushes the fresh
    // "guid_target" frame the app's send confirmation is waiting on.
    if (guid_target_dirty && ble_app.isConnected() && ble_app.isReadyForNotify())
    {
        guid_target_dirty = false;
        sendGuidTarget();
    }

    // Same pattern for the #915 config report: an edit the app made lands in
    // the FC's NVS and comes back through the report, and a change made over
    // LoRa (or by another phone) reaches this phone without a reconnect.
    if (fc_config_report_dirty && ble_app.isConnected() && ble_app.isReadyForNotify())
    {
        fc_config_report_dirty = false;
        sendConfigExtras();
    }

    // Service the BLE library's poll-style work — currently just the OTA
    // deferred-restart watchdog. handleOtaFinish() sets the boot partition and
    // schedules esp_restart() ~500 ms out, but the reboot only fires from here.
    // Without this, an OTA completes and sets ota_1 yet the device never reboots
    // into the new image (#8 Phase-3 OC bench finding; loop_bs always called it).
    // #834 item 2: ble_app.loop() below fires the OTA deferred reboot — on
    // THIS task, with the flush task still programming pages, and with none of
    // the power-off path's protection: no rail drop, no settle, no GPIO
    // teardown. The 500 ms it waits is a BLE-notify drain window, not a
    // storage window. Gated on otaRestartDue() (ELAPSED, not merely scheduled)
    // so the ring keeps draining for that whole window and we only quiesce on
    // the pass that will actually restart.
    if (ble_app.otaRestartDue())
    {
        // Atomic with the restart on purpose. The quiesce is IRREVERSIBLE (the
        // SPI bus is parked and never handed back, ingest stays paused), while
        // ota_pending_restart_at_ms_ lives on the NimBLE host task and can be
        // cleared by an OTA_ABORT arriving between these two lines. Letting
        // ble_app.loop() re-derive the decision would then leave the OC
        // quiesced and never rebooted — storage dead until a power cycle.
        quiesceStorageForRestart("ota-reboot");
        ESP_LOGW("OC", "OTA: rebooting now to load new partition (#834 item 2)");
        esp_restart();
    }
    ble_app.loop();

    // Check for BLE commands
    // #383: stage a stashed OTA_BEGIN from the loop task (see ocOtaRelayBegin).
    if (ota_begin_stage_pending)
    {
        ota_begin_stage_pending = false;
        uint8_t ota_hdr[36];
        memcpy(ota_hdr, (const void*)&ota_begin_total_size_staged, 4);
        memcpy(ota_hdr + 4, ota_begin_sha256_staged, 32);
        setPendingCommandWithConfig(OTA_BEGIN_PENDING, OTA_BEGIN_MSG,
                                    ota_hdr, sizeof(ota_hdr));
        ESP_LOGI("OC", "OTA relay: staged OTA_BEGIN for FC (size=%u)",
                 (unsigned)ota_begin_total_size_staged);
    }

    // ==========================================================================
    // SECTION: BLE command dispatch
    // ==========================================================================
    uint8_t ble_cmd = ble_app.getCommand();
    if (ble_cmd != 0)
    {
        ESP_LOGI("OC_CMD", "BLE cmd=%u", (unsigned)ble_cmd);
        if (ble_cmd == 1)
        {
            // Camera: payload[0] = desired state (1 = on, 0 = off), same
            // semantics as the LoRa uplink (processUplinkCommand cmd 1) and
            // the BS relay.  A blind toggle inverts whenever the app's idea
            // of the state and ours disagree — bench-observed 2026-08-16: the
            // OC held recording_requested=true across an FC reboot, so the
            // next "start" tap stopped the camera 11 ms after the FC had sent
            // START_RECORDING.  Falls back to toggle with no payload (legacy
            // app compat).
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t   plen    = ble_app.getCommandPayloadLength();
            const bool want_on = (plen >= 1) ? (payload[0] != 0)
                                             : !camera_recording_requested;
            if (want_on != camera_recording_requested || !camera_state_known)
            {
                // #825: after a boot rail restore the flag is a guess (our
                // reset zeroed it; the FC kept running) — forward the first
                // explicit command unconditionally so a stop can always stop.
                camera_state_known = true;
                camera_recording_requested = want_on;
                setPendingCommand(want_on ? CAMERA_START : CAMERA_STOP);
                ESP_LOGI("BLE", "Camera %s%s", want_on ? "START" : "STOP",
                         (plen >= 1) ? "" : " (legacy toggle)");
            }
            else
            {
                ESP_LOGI("BLE", "Camera already %s, ignoring",
                         want_on ? "ON" : "OFF");
            }
        }
        else if (ble_cmd == 2)
        {
            // Send file list with pagination (5 files per page). Encoder lives
            // in wire_format:: and is byte-tested against golden fixtures.
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
            // Logging: payload[0] = desired state (1 = start, 0 = stop),
            // matching the LoRa uplink and BS relay.  Same desync hazard as
            // cmd 1 above — a blind toggle turns "start" into "stop" whenever
            // the app's view disagrees with ours.  Falls back to toggle with
            // no payload (legacy app compat).
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t   plen    = ble_app.getCommandPayloadLength();
            const bool logging_now = logger.isLoggingActive();
            const bool want_on = (plen >= 1) ? (payload[0] != 0) : !logging_now;
            if (want_on == logging_now)
            {
                ESP_LOGI("OC_CMD", "Logging already %s, ignoring",
                         logging_now ? "ACTIVE" : "STOPPED");
            }
            else if (!want_on)
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
                                setPendingCommandWithConfig(SERVO_TEST_PENDING, SERVO_TEST_MSG, payload, sizeof(ServoTestAnglesData));
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
            // Delete file, then return the refreshed page-0 listing.
            String filename = ble_app.getDeleteFilename();
            // #383: deletes erase NAND and pause I2S ingest + I2C service for
            // the duration — never while flying (per design decision the gate
            // is INFLIGHT only; pad-state file ops remain allowed). Answer
            // with the current listing so the app UI refreshes truthfully.
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
        // #383: a download pauses I2S ingest + I2C service for the whole
        // multi-second transfer — mid-flight that would blind the OC and
        // drop the flight data it exists to record. Terminate the request
        // cleanly. #526: send EOF|ABORT, not a bare EOF — a refusal is NOT a
        // complete zero-length file. Without the abort bit the app writes the
        // empty download to disk and reports success (completeDownload's
        // short-transfer guard is gated on the stall timer, not this path).
        // INFLIGHT-only per design decision.
        if (download_filename.length() > 0 && latest_rocket_state == INFLIGHT)
        {
            ESP_LOGW("BLE", "Download '%s' refused: rocket INFLIGHT",
                     download_filename.c_str());
            (void)ble_app.sendFileChunk(0, nullptr, 0, true, /*abort=*/true);
            download_filename = "";
        }
        if (download_filename.length() > 0)
        {
            beginPhoneIO();  // pause I2C servicing + I2S ingest during blocking flash reads
            ESP_LOGI("BLE", "Download file request: %s", download_filename.c_str());

            // #524: read RSSI on the QUIET link, before we put any load on it.
            //
            // The 2026-07-14 run reported -107 dBm with the phone six inches away and
            // real antennas on both ends — which is ~60 dB of loss that the physics does
            // not allow. So this is very likely NOT a weak link but a broken instrument:
            // ble_gap_conn_rssi() issues HCI_Read_RSSI, a command inherited from Classic
            // Bluetooth whose LE behaviour on the ESP32 controller we have never checked.
            //
            // This line settles it without a single guess: if the QUIET link also reads
            // ~-107 at six inches, HCI_Read_RSSI is not usable here and every rssi= number
            // in the XFER line must be thrown away. Cross-check against the app, which
            // reads its own RSSI from CoreBluetooth (BLEDevice.connectedRSSI).
            ESP_LOGI("BLE", "Link before transfer: %lu ms effective, rssi=%d dBm "
                            "(cross-check against the app's own RSSI — if these disagree, "
                            "believe the app)",
                     (unsigned long)ble_app.effectiveEventMs(), (int)ble_app.connRssi());

            // The link must be the fast one before we stream a single byte.
            ensureFastLinkForTransfer();

            // Dynamic chunk size based on negotiated MTU (falls back to 170 if not yet negotiated)
            const size_t chunk_data_size = ble_app.getMaxChunkDataSize();
            ESP_LOGI("BLE", "Chunk data size: %u  (link %lu ms effective)",
                     (unsigned)chunk_data_size, (unsigned long)ble_app.effectiveEventMs());

            if (chunk_data_size == 0)
            {
                ESP_LOGE("BLE", "chunk data size is 0, aborting download");
                // #526: this is a failure, not a completed empty file.
                (void)ble_app.sendFileChunk(0, nullptr, 0, true, /*abort=*/true);
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

            // #524 diagnostic: split the wall clock between the two things a
            // download actually does. The NAND read path is bit-banged, so its
            // share is a genuine unknown — if flash owns a meaningful slice of the
            // transfer then no amount of BLE tuning will show up.
            uint64_t flash_us = 0;
            uint64_t ble_us   = 0;
            ble_app.resetXferStats();

            // #524: there is no per-chunk delay any more.
            //
            // There used to be a fixed 30 ms one, with this rationale: "the bursty
            // 3-at-a-time batch scheme overwhelmed the iOS BLE notification queue,
            // causing ~50% data loss. A consistent per-chunk delay keeps the queue
            // shallow and reliable."
            //
            // The data loss was real, but the delay was treating the symptom. File
            // chunks went out through the same best-effort notify path as telemetry,
            // which retries 3 x 5 ms and then DROPS the notification — fine for a
            // telemetry frame, data loss for a file. So the queue was kept shallow
            // enough that the drop path never fired, which pinned the transfer to one
            // notification per connection event: 502 B / 30 ms ~ 16 kB/s, with the
            // radio idle ~90% of the time.
            //
            // sendFileChunk now applies real backpressure instead — it waits for the
            // controller to drain rather than dropping — and returns false only if it
            // genuinely could not send. So chunks can be fed as fast as the stack
            // accepts them (several per connection event), with no drops.
            bool send_failed = false;

            while (!eof)
            {
                // Read next block from flash, appended after any carryover bytes
                size_t flash_bytes_read = 0;
                const uint32_t t_flash = micros();
                bool read_ok = flightlogReadChunk(download_filename.c_str(), file_offset,
                                                  read_buf + carryover, FLASH_READ_SIZE,
                                                  flash_bytes_read, eof);
                flash_us += (uint32_t)(micros() - t_flash);
                if (!read_ok)
                {
                    // #558: funnel through the unified post-loop abort handler
                    // instead of emitting an inline abort-EOF here. The inline
                    // abort-EOF did NOT set send_failed, so the post-loop fell
                    // through to a BARE EOF and the final redundant EOF also
                    // carried abort=false — so if this one inline abort-EOF was
                    // dropped on the link the app saw only bare EOFs and saved the
                    // truncated flight as complete, defeating #526's redundant-
                    // abort drop guard. Set the flag + break; the post-loop emits
                    // the abort-EOF AND a matching redundant one.
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
                        const uint32_t t_ble = micros();
                        const bool sent = ble_app.sendFileChunk(bytes_sent, ble_buf, ble_used, false);
                        ble_us += (uint32_t)(micros() - t_ble);
                        if (!sent)
                        {
                            // #524: could not send even after full backpressure. Do
                            // NOT keep going — that would hand the app a file with a
                            // hole in it and call it a success.
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

                if (send_failed) break;   // #524: abort the transfer, don't limp on

                // Move unparsed bytes to start of buffer for next iteration
                carryover = buf_len - pos;
                if (carryover > 0 && pos > 0)
                {
                    memmove(read_buf, read_buf + pos, carryover);
                }
            }

            if (send_failed)
            {
                // #526: EOF|ABORT, not a bare EOF. The app must REJECT the partial
                // file, not save the truncated bytes and call it complete. #558:
                // this branch now covers a flash read error as well as a BLE send
                // failure, so the specific cause is logged at each break point
                // rather than named here.
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

            // Redundant EOF in case the last notification was dropped. #526: it
            // must carry the SAME abort bit — otherwise a dropped abort-EOF
            // followed by a bare redundant EOF would resurrect the truncation bug
            // (the app would complete the partial file as if it were whole).
            delay(50);
            (void)ble_app.sendFileChunk(bytes_sent, nullptr, 0, true, /*abort=*/send_failed);

            uint32_t elapsed_ms = millis() - start_ms;
            float kbps = (elapsed_ms > 0) ? (bytes_sent / 1024.0f) / (elapsed_ms / 1000.0f) : 0;
            ESP_LOGI("BLE", "Download complete: %lu frames, %lu bytes in %.1fs (%.1f KB/s)",
                          (unsigned long)frames_sent, (unsigned long)bytes_sent,
                          elapsed_ms / 1000.0f, kbps);

            // #524 diagnostic — where did the time go, and why can't we put more
            // packets in each connection event?
            //
            //   flash% large        -> the bit-banged NAND read is a real cost and
            //                          BLE tuning is chasing the wrong thing
            //   qdepth ~2 chunks    -> the controller's outbound queue is the cap;
            //                          there may be a knob
            //   qdepth deep + slow  -> iOS just won't drain more per event, and the
            //                          only way past it is an L2CAP channel (#526)
            //
            // pm= and rssi= exist to explain the battery-vs-USB gap (battery measured
            // ~2.7x slower). They separate the two candidate causes, which point in
            // opposite directions:
            //   pm=0                -> the PM lock never engaged; we ARE light-sleeping
            //                          through the transfer and the fix is here
            //   pm=1 + rssi much
            //   worse than on USB   -> PM is fine and the link is the problem (the LL
            //                          is silently retransmitting, eating the per-event
            //                          packet budget) — a rail/antenna issue, not code
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

            // JUDGE THE LINK BY max, NOT avg OR min. Bench 2026-07-14, phone six inches
            // away: quiet link -64 dBm; during the transfer avg=-79 min=-113 max=-59
            // over 74 samples.
            //
            // -113 dBm is BELOW the receiver's noise floor — it is not a measurement.
            // HCI_Read_RSSI reports whatever the controller last saw, and under load that
            // includes idle moments between connection events, so the low samples are
            // junk. The MAX tracks the quiet-link reading (-59 vs -64), and that is the
            // real link quality.
            //
            // This is not a nitpick: a single such outlier (-107) was used to explain away
            // a slow run, and it explained nothing — three later runs on the same hardware
            // all hit 35 KB/s. If you are about to blame RSSI for a slow transfer, look at
            // max first, and check the app's own CoreBluetooth RSSI before believing it.
            if (xs.rssi_n > 0 && xs.rssi_max < -85)
            {
                ESP_LOGW("BLE", "  ^ link genuinely WEAK (best sample %d dBm). The LL will "
                                "be retransmitting, and every retransmit costs one of the "
                                "~4 packet slots per connection event that the transfer "
                                "rate is made of.", (int)xs.rssi_max);
            }

            // Keep saying it, so a battery run can be read back over USB afterwards.
            s_xfer_reprint_until_ms = millis() + XFER_REPRINT_WINDOW_MS;
            s_xfer_next_reprint_ms  = millis() + XFER_REPRINT_EVERY_MS;
            } // else (chunk_data_size > 0)
            endPhoneIO();

            // #524 follow-up: ensureFastLinkForTransfer() above moves an FC-off
            // download onto the FAST link, and nothing asked for the slow set back
            // — the slow request only fires on the connect edge — so after one
            // idle-time download the OC sat at the fast link's ~7 mA instead of
            // ~1 mA until the app disconnected. Mirror the connect-edge policy:
            // rail off -> slow link. Sits after endPhoneIO() so it also covers the
            // abort paths (chunk size 0, flash read error, send_failed), which all
            // run after the fast-link switch. A redundant ask when the link never
            // left slow is one LL procedure, not a loop: requestConnParams records
            // the intent and collision-retries the SAME set.
            if (!pwr_pin_on && ble_app.isConnected())
            {
                requestSlowBLEParams();
            }
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
                                setPendingCommandWithConfig(SIM_CONFIG_PENDING, SIM_CONFIG_MSG, &sim_cfg, sizeof(sim_cfg));
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
            flightlogEndFlight();
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
            // Power rail: payload[0] = desired state (1 = on, 0 = off), same
            // semantics as cmds 1/23.  A blind toggle inverts whenever the
            // app's view and ours disagree — and here that means cutting the
            // FC's rail when the operator asked to power it up (or a repeated
            // command double-toggling).  The app knows the true state: the OC
            // stays alive with the rail down and keeps reporting pwr_pin_on,
            // and the UI gates the button on having received telemetry
            // (#377).  No payload still means toggle (legacy app compat).
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t   plen    = ble_app.getCommandPayloadLength();
            const bool was_on  = pwr_pin_on;
            const bool want_on = (plen >= 1) ? (payload[0] != 0) : !pwr_pin_on;

            // #834 item 2: refuse an OFF while flying, matching the gates
            // delete and download already have. Two reasons. (1) Post-#848 the
            // FC holds its own rail through P4_EN_HOLD, so this no longer
            // powers anything down — it just resets the OC and kills the
            // downlink mid-flight. (2) The quiesce above now CLOSES the
            // session, and the #846 boot re-seed only ever reads a
            // flight_recovered_* entry; a properly finalized flight is
            // skipped. Without this gate a mid-flight power-off would destroy
            // the FC's only NAND-side snapshot source while it is still
            // flying. Gating removes that trade-off rather than weakening the
            // quiesce.
            // Freshness is load-bearing: latest_rocket_state NEVER AGES, and
            // cmd 8 is the OC's only reset path. Gating on the latched value
            // alone means an FC that died mid-flight — exactly when you most
            // want to recover the OC — would refuse every power-off for the
            // rest of the boot, with no way out but a battery pull.
            const bool fc_state_fresh =
                latest_non_sensor_valid &&
                (uint32_t)(millis() - latest_non_sensor_rx_ms) <= config::FC_FRAME_STALE_MS;
            const bool refuse_off = !want_on && was_on && fc_state_fresh &&
                                    latest_rocket_state == INFLIGHT;
            if (refuse_off)
            {
                ESP_LOGW("BLE", "Power off REFUSED: rocket is INFLIGHT "
                                "(FC frame %lu ms ago)",
                         (unsigned long)(millis() - latest_non_sensor_rx_ms));
            }
            else if (want_on == was_on)
            {
                ESP_LOGI("BLE", "Power rail already %s, ignoring",
                         was_on ? "ON" : "OFF");
            }
            else if (want_on)
            {
                pwr_pin_on = true;
                // Exit low-power mode BEFORE powering peripherals — need
                // full CPU speed and no auto light-sleep during init.
                exitLowPowerMode();

                // Power on the FlightComputer + sensors. New PCB also gates
                // the GNSS rail off a separate enable that comes up at the
                // same time so the receiver can start its cold-start
                // acquisition concurrently with FC boot.
                digitalWrite(config::PWR_PIN, HIGH);
                if (config::GPS_PWR_PIN >= 0)
                {
                    digitalWrite(config::GPS_PWR_PIN, HIGH);
                }
                rail_rtc = {kRailRtcMagic, 1, 0, 0};  // #825: whole-struct write (see invariant)
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
                    requestFastBLEParams();
                }
            }
            else
            {
                pwr_pin_on = false;
                // Power off: drop the FC rail and reset the OC.
                //
                // Surgically tearing down each peripheral on power-off
                // (I2S DMA + APB lock, SPI bus, I2C master+slave bus,
                // logger flush task, LittleFS, NAND/MRAM driver state,
                // dedup filter prev_ts, BLE connection) is error-prone
                // and leaves residual state that breaks the next
                // power-on cycle (#9 — observed: I2S APB lock held,
                // i2c_new_slave_device fails because bus is still
                // acquired, dedup drops every post-reset frame). A
                // clean reset gets us to the same idle state as cold
                // boot (~16 mA baseline, BLE advertising, all driver
                // state freshly initialised).
                //
                // The iOS app already handles the brief disconnect /
                // reconnect because the existing brownout-on-power-on
                // hardware quirk exercises the same recovery path.
                ESP_LOGI("PWR", "Power off: resetting OC for clean idle state (#9)...");
                // #825: mark this restart as the DELIBERATE power-off so the
                // boot-time re-assert stands down — boot-time rail-LOW is
                // load-bearing for this flow (the reset IS the power-off).
                rail_rtc = {kRailRtcMagic, 0, 1, 0};
                digitalWrite(config::PWR_PIN, LOW);
                if (config::GPS_PWR_PIN >= 0)
                {
                    digitalWrite(config::GPS_PWR_PIN, LOW);  // drop GNSS rail in lockstep
                }

                // #834 item 2: get the log onto the NAND before the reset.
                // AFTER the rail drop on purpose — the wait doubles as
                // rail-discharge time (PWR_PIN is already LOW) and the FC's
                // frame source is gone. BEFORE the GPIO teardown below,
                // because gpio_reset_pin() detaches the pads from the SPI
                // matrix, after which no NAND command can complete.
                quiesceStorageForRestart("power-off");

                // Prevent back-feed into peripherals whose VCC is about to
                // disappear: current from the still-powered OC side through a
                // peripheral's input ESD diode into its dead rail shows up as
                // steady draw on the OC's input. gpio_reset_pin detaches the
                // pad from any peripheral-matrix routing left over from
                // initPeripherals(); gpio_set_level(0) holds it LOW for the
                // brief window before the reset. Post-reset IOs default to
                // high-Z, which is also fine — high-Z is not a back-feed
                // source. (#9)
                //
                // #834 item 2: this list used to name "LoRa, NAND, MRAM" and
                // lead with the SPI bus. Every part of that was wrong for this
                // board, verified against the KiCad netlist:
                //   * U11 (GD5F2GQ5UE SPI NAND — "the flash") has VCC on +3V3,
                //     generated by U18 (TPS62152) whose EN is tied to its own
                //     AVIN: ALWAYS ON. It never loses power here, so there is
                //     no dead rail to protect — and SPI_MISO is an output U11
                //     DRIVES, so forcing the pad low was a push-pull fight with
                //     a live chip. (Same reason the FC's I2S pins were moved to
                //     the high-Z list below.)
                //   * There is no MRAM on V9/V10 at all (#822) — MRAM_CS is -1.
                //   * The switched rail V_MCU_SWTCH (U30) feeds the FC (U17)
                //     and its sensors (U2/U3/U4/U20), none of which are here.
                //   * LoRa is NOT on V_MCU_SWTCH: it is a J5 daughterboard fed
                //     from VBATT through U29, enabled by LoRa_ACT, and talks
                //     over UART — so the pins that genuinely need this are
                //     LORA_UART_TX/RX, which the old list omitted entirely
                //     while listing V7-only SPI-LoRa constants that are all -1
                //     here and skipped. On V9 the loop therefore did nothing
                //     except fight the flash.
                // (There is no TPS22918 on this board either; the load
                // switches are U29/U30 TPS22810 and U26/U28 TPS22811.)
                //
                // Split by direction: OC OUTPUTS into a soon-dead peripheral
                // get driven LOW; anything the peripheral drives is left
                // high-Z so we never contend with a still-powered part.
                // LORA_ACT_PIN first: it is U29's enable, so dropping it is
                // what actually removes the daughterboard's VBATT rail.
                // Driving LORA_UART_TX low while the module is still powered
                // would just hold a break condition on a live receiver; the
                // back-feed this list exists to prevent only becomes possible
                // once the rail is gone.
                static const gpio_num_t kSwitchedRailPins[] = {
                    (gpio_num_t)config::LORA_ACT_PIN,      // U29 EN -> LoRa rail
                    (gpio_num_t)config::LORA_UART_TX_PIN,  // OC -> J5 daughterboard
                    (gpio_num_t)config::LORA_SPI_SCK,      // V7 SPI-LoRa (-1 on V8+)
                    (gpio_num_t)config::LORA_SPI_MOSI,
                    (gpio_num_t)config::LORA_CS_PIN,
                    (gpio_num_t)config::LORA_RST_PIN,
                };
                // I2S signals from the FC (slave RX on the OC) are handled
                // separately: they are FC OUTPUTS, and since #848 the FC can
                // legitimately still be POWERED here (its P4_EN_HOLD latch
                // during a sim/flight means dropping PWR_PIN no longer cuts
                // its rail). Driving a live I2S bit clock hard LOW for the
                // 100 ms below would be a sustained push-pull drive fight.
                // gpio_reset_pin leaves them high-Z inputs — per the comment
                // above, high-Z is not a back-feed source, which was the only
                // reason these were ever in the driven-LOW list (#9).
                // #834 item 2: the LoRa lines the DAUGHTERBOARD drives join
                // them, for the same reason — and so do the flash's, which are
                // simply left alone now: U11 stays powered, and the quiesce
                // above has parked the SPI mutex with CS HIGH and no byte on
                // the wire. (The park takes the mutex; it does NOT tear down
                // the SPI driver — see TR_LogToFlash::parkSpiBusForReset. It
                // does not need to: nothing can start a transaction while the
                // bus is parked.)
                static const gpio_num_t kHighZPins[] = {
                    (gpio_num_t)config::I2S_BCLK_PIN,
                    (gpio_num_t)config::I2S_WS_PIN,
                    (gpio_num_t)config::I2S_DIN_PIN,
                    (gpio_num_t)config::I2S_FSYNC_PIN,
                    (gpio_num_t)config::LORA_UART_RX_PIN,  // J5 -> OC
                    (gpio_num_t)config::LORA_SPI_MISO,     // V7 SPI-LoRa (-1 on V8+)
                    (gpio_num_t)config::LORA_DIO1_PIN,
                    (gpio_num_t)config::LORA_BUSY_PIN,
                };
                for (gpio_num_t pin : kSwitchedRailPins) {
                    if ((int)pin < 0) continue;  // peripheral absent on this board (#411)
                    gpio_reset_pin(pin);
                    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
                    gpio_set_pull_mode(pin, GPIO_FLOATING);
                    gpio_set_level(pin, 0);
                }
                for (gpio_num_t pin : kHighZPins) {
                    if ((int)pin < 0) continue;
                    gpio_reset_pin(pin);
                    // gpio_reset_pin() ENABLES the internal pull-up, so on its
                    // own it does not give the high-Z this list promises — a
                    // ~45 kOhm pull to 3V3 still injects into an unpowered
                    // peripheral, and still fights a live driver. Float it.
                    gpio_set_pull_mode(pin, GPIO_FLOATING);
                }

                vTaskDelay(pdMS_TO_TICKS(100));   // let rail drop, caps discharge
                esp_restart();
                // not reached
            }

            // Only on an actual state change — an ignored duplicate must not
            // claim the rail was switched, nor re-push config.
            // #834 item 2: a REFUSED off never changed the rail, so it must
            // not claim it did — the epilogue logs the transition, stalls the
            // loop 100 ms and re-pushes config, all of which are wrong (and
            // mid-flight, actively harmful) on a refusal.
            if (want_on != was_on && !refuse_off)
            {
                ESP_LOGI("BLE", "Power rail: %s%s", pwr_pin_on ? "ON" : "OFF",
                         (plen >= 1) ? "" : " (legacy toggle)");

                // After power-on, NVS config is freshly loaded — resend config
                // readback so the app gets the actual persisted values.
                if (pwr_pin_on && ble_app.isConnected()) {
                    delay(100);  // let peripherals finish init
                    sendCurrentConfig();
                }
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
            // Direct LoRa reconfig over BLE is no longer accepted on the
            // rocket side (#106).  LoRa link parameters are owned by the
            // base station; allowing the rocket to change them out-of-band
            // can desync the pair (the BS keeps its own copy in NVS and
            // never learns about a rocket-only change).  The iOS Settings
            // view should be sending cmd 10 to the BS, which then relays
            // via uplink cmd 10 (handled separately above) and persists.
            //
            // We send a config readback so the app's UI snaps back to the
            // rocket's actual LoRa values rather than appearing to apply.
            ESP_LOGW("BLE", "Cmd 10 refused on rocket: LoRa params are BS-controlled (#106). Send to base station instead.");
            sendCurrentConfig();
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
            // Servo config: [bias1:2][bias2:2][bias3:2][bias4:2][hz:2][min:2][max:2][fin_min:4][fin_max:4] = 22 bytes
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(ServoConfigData))
            {
                                setPendingCommandWithConfig(SERVO_CONFIG_PENDING, SERVO_CONFIG_MSG, payload, sizeof(ServoConfigData));
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
                                setPendingCommandWithConfig(PID_CONFIG_PENDING, PID_CONFIG_MSG, payload, 20);
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
                                setPendingCommandWithConfig(ROLL_PROFILE_PENDING, ROLL_PROFILE_MSG, payload, sizeof(RollProfileData));
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
        else if (ble_cmd == 28)
        {
            // Drift-Cast guidance point (#435): relay the 20-byte
            // GuidancePointData {lat f64, lon f64, alt f32} LE to the FC.
            // No OC-side INFLIGHT gate here: the FC's state gate is
            // authoritative and its rejection is VISIBLE to the app via the
            // guid_target echo (unlike the LoRa path, which refuses INFLIGHT
            // below because it has no echo).
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(GuidancePointData))
            {
                setPendingCommandWithConfig(GUIDANCE_POINT_PENDING, GUIDANCE_POINT_MSG,
                                            payload, sizeof(GuidancePointData));
                ESP_LOGI("BLE", "Guidance point queued for RocketComputer");
            }
            else
            {
                ESP_LOGW("BLE", "Guidance point payload too short (%u < %u)",
                              (unsigned)plen, (unsigned)sizeof(GuidancePointData));
            }
        }
        else if (ble_cmd == 29)
        {
            // Servo replay: send flight data sample through control loop
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(ServoReplayData))
            {
                                setPendingCommandWithConfig(SERVO_REPLAY_PENDING, SERVO_REPLAY_MSG, payload, sizeof(ServoReplayData));
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
            // Roll control config (RollControlConfigData, 20 B):
            // [use_angle_control:1][pad:1][roll_delay_ms:2]
            // [kp_angle_rate_cap_dps:4][kp_angle:4][integral_sep_threshold_dps:4]
            // [roll_min_speed_mps:4]
            // Relay the FULL struct: the FC's readConfigFrame matches on an
            // exact payload length, so a short copy is not merely truncated —
            // the frame is never found and every roll-control setting is
            // silently dropped.
            //
            // A phone built before the speed gate sends only the first 16
            // bytes.  Pad that to the full struct with roll_min_speed_mps = 0
            // (gate off, which is what that app means) instead of refusing it,
            // so an older app on newer firmware still configures roll control.
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= ROLL_CTRL_CONFIG_LEN_V1)
            {
                uint8_t rc_buf[sizeof(RollControlConfigData)] = {};
                const size_t copy_len = (plen < sizeof(rc_buf)) ? plen : sizeof(rc_buf);
                memcpy(rc_buf, payload, copy_len);
                setPendingCommandWithConfig(ROLL_CTRL_CONFIG_PENDING, ROLL_CTRL_CONFIG_MSG, rc_buf, sizeof(rc_buf));
                cacheRollControlConfig(rc_buf, sizeof(rc_buf));
                ESP_LOGI("BLE", "Roll control config queued (%u B in%s)",
                    (unsigned)plen,
                    (plen < sizeof(RollControlConfigData)) ? ", legacy - speed gate off" : "");
            }
            else
            {
                ESP_LOGW("BLE", "Roll control config too short (%u < %u)",
                    (unsigned)plen, (unsigned)ROLL_CTRL_CONFIG_LEN_V1);
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
        else if (ble_cmd == 65)
        {
            // Full guidance config (GuidanceConfigData): relay the whole struct to the FC
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(GuidanceConfigData))
            {
                                setPendingCommandWithConfig(GUIDANCE_CONFIG_PENDING, GUIDANCE_CONFIG_MSG, payload, sizeof(GuidanceConfigData));
                ESP_LOGI("BLE", "Guidance config queued for RocketComputer");
            }
        }
        else if (ble_cmd == 66)
        {
            // Full fin layout (FinConfigData): relay the whole struct to the FC
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(FinConfigData))
            {
                                setPendingCommandWithConfig(FIN_CONFIG_PENDING, FIN_CONFIG_MSG, payload, sizeof(FinConfigData));
                ESP_LOGI("BLE", "Fin layout queued for RocketComputer");
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
                                setPendingCommandWithConfig(CAMERA_CONFIG_PENDING, CAMERA_CONFIG_MSG, &cam_cfg, sizeof(cam_cfg));
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
        else if (ble_cmd == 64)
        {
            // IMU mounting orientation: [setting:1] — IMU_ORIENT_AUTO or
            // a TR_Orientation code (0..23, manual incl. roll clocking).
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 1 &&
                (payload[0] == IMU_ORIENT_AUTO || payload[0] < ORIENT_CODE_COUNT))
            {
                cfg_imu_orient = payload[0];
                stageImuOrientConfig();
                Preferences prefs;
                prefs.begin("orient", false);
                prefs.putUChar("set", cfg_imu_orient);
                prefs.end();
                ESP_LOGI("BLE", "IMU orientation setting: %s",
                         cfg_imu_orient == IMU_ORIENT_AUTO
                             ? "AUTO" : orientCodeName(cfg_imu_orient));
            }
        }
        else if (ble_cmd == 67)
        {
            // IMU logging rate: [rate_hz:2 LE] — IMU_RATE_DYNAMIC (0) or a
            // whitelisted ISM6HG256 ODR (960/1920/3840).  Relayed to the FC,
            // which applies the ODR live (except INFLIGHT) and persists it in
            // FC NVS.
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= 2)
            {
                uint16_t rate_hz;
                memcpy(&rate_hz, payload, sizeof(rate_hz));
                if (imuRateSettingValid(rate_hz))
                {
                    cfg_imu_rate = rate_hz;
                    stageImuRateConfig();
                    Preferences p2;
                    p2.begin("imurate", false);
                    p2.putUShort("hz", cfg_imu_rate);
                    p2.end();
                    if (imuRateIsDynamic(rate_hz))
                    {
                        ESP_LOGI("BLE", "IMU logging rate: DYNAMIC -> FlightComputer");
                    }
                    else
                    {
                        ESP_LOGI("BLE", "IMU logging rate: %u Hz -> FlightComputer",
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
            // resumes.  Same constant (and therefore the same number) as the
            // uplink command, so the app's toggle means one thing whether it
            // reaches the rocket over BLE or relayed through the base station.
            //
            // Rail state is irrelevant: with the FC rail off the radio isn't
            // even initialized, and the setting simply sits in NVS until
            // initPeripherals reads it — which is exactly how an operator
            // arms "fly quiet" before powering the stack up.
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
                // Queue for FC via I2C
                                setPendingCommandWithConfig(PYRO_CONFIG_PENDING, PYRO_CONFIG_MSG, &pcfg, sizeof(pcfg));
                // Persist to NVS
                Preferences prefs;
                prefs.begin("pyro", false);
                size_t written = prefs.putBytes("cfg", &pcfg, sizeof(pcfg));
                prefs.end();
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
            // Pyro continuity test — 1 byte payload: channel (1..4)
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            uint8_t ch = (plen >= 1) ? payload[0] : 0;
            if (ch < 1 || ch > 4) {
                ESP_LOGW("BLE", "Pyro continuity test: invalid channel %u", ch);
            } else if (!pwr_pin_on) {
                // Rail off: the FC queue would HOLD this and deliver the
                // momentary ARM pulse at the next power-on (see the queue
                // header's pyro exception). Refuse and tell the app.
                ble_app.sendPyroTestRefusal(35, ch, 1 /* FC rail off */);
                ESP_LOGW("BLE", "Pyro continuity test CH%u refused: FC rail off"
                                " (queued arm pulse would deliver at power-on)", ch);
            } else {
                setPendingCommandWithConfig(PYRO_CONT_TEST, PYRO_CONT_TEST, &ch, 1);
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
            } else if (!pwr_pin_on) {
                // Rail off: the FC queue would HOLD this and FIRE the channel
                // at the next power-on — a latent fire delivered while someone
                // may be handling the rocket (see the queue header's pyro
                // exception). Refuse and tell the app.
                ble_app.sendPyroTestRefusal(36, ch, 1 /* FC rail off */);
                ESP_LOGW("BLE", "Pyro test fire CH%u refused: FC rail off"
                                " (queued fire would deliver at power-on)", ch);
            } else {
                setPendingCommandWithConfig(PYRO_FIRE_TEST, PYRO_FIRE_TEST, &ch, 1);
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
        // ---- Magnetometer hard-iron calibration (issue #96) ----
        // Four single-byte commands grouped at 50–53.  Each is a thin
        // pass-through: setPendingCommand routes to the FC over I2C, FC
        // owns all the state-machine + sphere-fit logic.
        else if (ble_cmd == 50)
        {
            setPendingCommand(MAG_CAL_START);
            ESP_LOGI("BLE", "Mag cal START -> FlightComputer");
        }
        else if (ble_cmd == 51)
        {
            setPendingCommand(MAG_CAL_ABORT);
            ESP_LOGI("BLE", "Mag cal ABORT -> FlightComputer");
        }
        else if (ble_cmd == 52)
        {
            setPendingCommand(MAG_CAL_ACCEPT);
            ESP_LOGI("BLE", "Mag cal ACCEPT -> FlightComputer");
        }
        else if (ble_cmd == 53)
        {
            setPendingCommand(MAG_CAL_RETRY);
            ESP_LOGI("BLE", "Mag cal RETRY -> FlightComputer");
        }
        else if (ble_cmd == 54)
        {
            setPendingCommand(MAG_CAL_COMPUTE_FIT);
            ESP_LOGI("BLE", "Mag cal COMPUTE_FIT -> FlightComputer");
        }
        // #148 — user-driven verify completion / reset.
        else if (ble_cmd == 56)
        {
            setPendingCommand(MAG_CAL_VERIFY_DONE);
            ESP_LOGI("BLE", "Mag cal VERIFY_DONE -> FlightComputer");
        }
        else if (ble_cmd == 57)
        {
            setPendingCommand(MAG_CAL_VERIFY_RESET);
            ESP_LOGI("BLE", "Mag cal VERIFY_RESET -> FlightComputer");
        }
        // #148 — user-override save (user explicitly accepting a
        // borderline cal where the iOS-side gates are partially red).
        else if (ble_cmd == 58)
        {
            setPendingCommand(MAG_CAL_FORCE_APPLY);
            ESP_LOGI("BLE", "Mag cal FORCE_APPLY -> FlightComputer");
        }
        // Issue #132 — app pushes a saved cal back into FC NVS as part of the
        // rocket-profile auto-sync on connect.  14-byte payload mirrors the
        // values cmd 52 would have persisted after a fresh sphere fit.
        else if (ble_cmd == 55)
        {
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(MagCalApplyData))
            {
                                setPendingCommandWithConfig(MAG_CAL_APPLY_PENDING, MAG_CAL_APPLY_MSG, payload, sizeof(MagCalApplyData));
                ESP_LOGI("BLE", "Mag cal APPLY queued for FlightComputer");
            }
            else
            {
                ESP_LOGW("BLE", "Mag cal APPLY: payload too short (%u < %u)",
                              (unsigned)plen, (unsigned)sizeof(MagCalApplyData));
            }
        }
        // #132 profile-cal READ + sensor APPLY/READ.  Renumbered from
        // 56/57/58 to 61/62/63: the #148 mag-cal verify commands
        // (VERIFY_DONE/RESET/FORCE_APPLY) own 56/57/58 and are matched first
        // in this if/else-if chain, so at 56/57/58 these three branches were
        // dead and the app's connect-time profile reads mis-fired as
        // mag-verify commands.  Must stay in sync with BLEDevice.swift
        // (sendMagCalRead / sendSensorCalApply / sendSensorCalRead).
        else if (ble_cmd == 61)
        {
            setPendingCommand(MAG_CAL_READ);
            ESP_LOGI("BLE", "Mag cal READ -> FlightComputer");
        }
        // Issue #132 — app pushes a saved sensor cal (gyro + high-g) back into
        // FC NVS as part of the rocket-profile auto-sync on connect.
        // (Renumbered 57 -> 62, see note on the MAG_CAL_READ case above.)
        else if (ble_cmd == 62)
        {
            const uint8_t* payload = ble_app.getCommandPayload();
            const size_t plen = ble_app.getCommandPayloadLength();
            if (plen >= sizeof(SensorCalApplyData))
            {
                                setPendingCommandWithConfig(SENSOR_CAL_APPLY_PENDING, SENSOR_CAL_APPLY_MSG, payload, sizeof(SensorCalApplyData));
                ESP_LOGI("BLE", "Sensor cal APPLY queued for FlightComputer");
            }
            else
            {
                ESP_LOGW("BLE", "Sensor cal APPLY: payload too short (%u < %u)",
                              (unsigned)plen, (unsigned)sizeof(SensorCalApplyData));
            }
        }
        // (Renumbered 58 -> 63, see note on the MAG_CAL_READ case above.)
        else if (ble_cmd == 63)
        {
            setPendingCommand(SENSOR_CAL_READ);
            ESP_LOGI("BLE", "Sensor cal READ -> FlightComputer");
        }
    }

    LOOP_STALL_INSTR("printStats", printStats());

    // Catch-all: any iteration whose total wall time exceeds the threshold
    // gets logged even if no individual wrapped callsite tripped.  This
    // surfaces blocking work outside the named LOOP_STALL_INSTR sites.
    const int64_t _loop_oc_dt = esp_timer_get_time() - _loop_oc_t0;
    if (_loop_oc_dt > LOOP_STALL_THRESHOLD_US) {
        ESP_LOGW("LOOP_STALL", "loop_oc iteration took %lld us (catch-all)",
                 (long long)_loop_oc_dt);
    }

    _loop_oc_last_exit_us = esp_timer_get_time();
    vTaskDelay(1);  // yield to FreeRTOS scheduler
}

// ==========================================================================
// SECTION: FreeRTOS entry point
// ==========================================================================
extern "C" void app_main(void)
{
    setup_oc();
    xTaskCreatePinnedToCore([](void*) { while (true) { loop_oc(); } },
                            "oc_loop", 12 * 1024, NULL, 5, NULL, 1);
}
