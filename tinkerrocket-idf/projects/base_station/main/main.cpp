#include <compat.h>
#include <TR_NVS.h>
#include <algorithm>

#include <esp_log.h>
#include <esp_mac.h>              // esp_efuse_mac_get_default for unit_id
#include <esp_vfs_fat.h>
#include <esp_spiffs.h>
#include <sdmmc_cmd.h>
#include <driver/sdmmc_host.h>
#include <driver/i2c_master.h>

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <string>
#include <dirent.h>
#include <sys/stat.h>

#include "config.h"

#include <TR_LoRa_Comms.h>
#include <TR_Sensor_Data_Converter.h>
#include <TR_Coordinates.h>
#include <TR_BLE_To_APP.h>
#include <TR_MAX17205G.h>
#include <RocketComputerTypes.h>

static const char* TAG = "BS";

// Forward declarations
static const char* rocketStateToString(uint8_t state);

static TR_LoRa_Comms lora_comms;
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
static uint32_t last_packet_ms = 0;

// Base station battery (MAX17205G fuel gauge via I2C)
static float bs_voltage = NAN;
static float bs_soc = NAN;
static float bs_current = NAN;
static float bs_temperature = NAN;
static uint32_t last_battery_ms = 0;
static i2c_master_bus_handle_t i2c_bus = nullptr;
static TR_MAX17205G fuel_gauge(config::MAX17205_ADDR);
static bool fuel_gauge_present = false;

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
static uint8_t  uplink_buf[24];   // 3-byte header + up to 20 bytes payload (PID config needs 20)
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
static const char* SPIFFS_PARTITION_LABEL = "spiffs";

// CSV logging state
static FILE* log_file = nullptr;
static bool logging_active = false;
static uint32_t log_start_ms = 0;
static uint32_t log_last_write_ms = 0;
static uint32_t log_last_flush_ms = 0;
static uint32_t landed_time_ms = 0;     // When LANDED state first seen (0 = not landed)
static uint8_t  last_rocket_state = 0;  // Track state transitions
static bool     last_known_camera_recording = false;  // Track rocket camera state for idempotent uplink
static bool     last_known_rocket_logging = false;    // Actual rocket logging state from LoRa downlink

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
    return oldest_idx;
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
static void updateBattery()
{
    if (!fuel_gauge_present) return;
    fuel_gauge.update();
    bs_voltage     = fuel_gauge.voltage();
    bs_soc         = fuel_gauge.soc();
    bs_current     = fuel_gauge.current();
    bs_temperature = fuel_gauge.temperature();
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

static uint16_t findNextFileNumber()
{
    uint16_t max_num = 0;
    DIR* dir = opendir(SD_MOUNT_POINT);
    if (!dir) return 1;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        uint16_t num = 0;
        if (sscanf(entry->d_name, "lora_%hu.csv", &num) == 1)
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
    char basename[32];
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

    log_file = fopen(log_filename, "w");
    if (!log_file)
    {
        ESP_LOGE(TAG, "[LOG] Failed to open %s for writing! errno=%d (%s)",
                 log_filename, errno, strerror(errno));
        logging_active = false;
        return;
    }

    // Write CSV header
    fprintf(log_file, "time_ms,state,num_sats,pdop,lat,lon,alt_m,h_acc,"
                      "acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,"
                      "pressure_alt,alt_rate,max_alt,max_speed,"
                      "voltage,current,soc,roll,pitch,yaw,speed,"
                      "launch,vel_apo,alt_apo,landed,rssi,snr\n");

    logging_active = true;
    log_start_ms = millis();
    log_last_write_ms = millis();
    log_last_flush_ms = millis();
    landed_time_ms = 0;

    ESP_LOGI(TAG, "[LOG] Started logging: %s", log_filename);
}

static void stopLogging()
{
    if (!logging_active) return;

    if (log_file) { fclose(log_file); log_file = nullptr; }
    logging_active = false;
    landed_time_ms = 0;

    ESP_LOGI(TAG, "[LOG] Closed log: %s", log_filename);
}

static uint32_t log_write_count = 0;  // Tracks calls for periodic flash check

static void logLoRaPacket(const LoRaDataSI& data, float rssi, float snr,
                          double lat, double lon, double alt)
{
    if (!logging_active) return;

    // Periodic storage usage check (every 100 writes)
    if (++log_write_count % 100 == 0)
    {
        uint64_t total = 0, used = 0;
        if (using_internal_flash)
        {
            size_t t = 0, u = 0;
            if (esp_spiffs_info(SPIFFS_PARTITION_LABEL, &t, &u) == ESP_OK)
            {
                total = t;
                used  = u;
            }
        }
        else
        {
            FATFS* fs;
            DWORD free_clust;
            if (f_getfree("0:", &free_clust, &fs) == FR_OK)
            {
                total = (uint64_t)(fs->n_fatent - 2) * fs->csize * 512;
                uint64_t free_bytes = (uint64_t)free_clust * fs->csize * 512;
                used = total - free_bytes;
            }
        }
        if (total > 0 && used > (total * 9 / 10))
        {
            ESP_LOGW(TAG, "[LOG] %s nearly full! %llu/%llu bytes (%.0f%%)",
                     using_internal_flash ? "Internal flash" : "SD card",
                     (unsigned long long)used, (unsigned long long)total,
                     (double)used * 100.0 / (double)total);
        }
    }

    uint32_t time_ms = millis() - log_start_ms;

    int written = fprintf(log_file, "%lu,%s,%u,%.1f,%.7f,%.7f,%.1f,%.1f,"
                    "%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,"
                    "%.1f,%.1f,%.1f,%.1f,"
                    "%.2f,%.0f,%.1f,%.1f,%.1f,%.1f,%.1f,"
                    "%d,%d,%d,%d,%.0f,%.1f\n",
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
                    (double)rssi, (double)snr);

    if (written <= 0)
    {
        ESP_LOGW(TAG, "[LOG] fprintf() failed (write returned <= 0)");
    }

    log_last_write_ms = millis();
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

static void handleDownloadCommand()
{
    String filename = ble_app.getDownloadFilename();
    if (filename.length() == 0) return;

    char path[64];
    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, filename.c_str());
    FILE* f = fopen(path, "r");
    if (!f)
    {
        ESP_LOGE(TAG, "[BLE] Download failed, file not found: %s", filename.c_str());
        ble_app.sendFileChunk(0, nullptr, 0, true);  // Send empty EOF
        return;
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    uint32_t file_size = (uint32_t)ftell(f);
    fseek(f, 0, SEEK_SET);

    uint32_t offset = 0;
    uint8_t chunk_buf[config::BLE_FILE_CHUNK_SIZE];

    ESP_LOGI(TAG, "[BLE] Starting download: %s (%lu bytes)",
             filename.c_str(), (unsigned long)file_size);

    while (!feof(f))
    {
        if (!ble_app.isConnected())
        {
            ESP_LOGW(TAG, "[BLE] Disconnected during download, aborting");
            break;
        }
        size_t got = fread(chunk_buf, 1, config::BLE_FILE_CHUNK_SIZE, f);
        if (got == 0) break;
        bool eof = feof(f);
        ble_app.sendFileChunk(offset, chunk_buf, got, eof);
        offset += got;
        delay(config::BLE_CHUNK_DELAY_MS);
    }

    // Handle empty file
    if (file_size == 0)
    {
        ble_app.sendFileChunk(0, nullptr, 0, true);
    }

    fclose(f);
    ESP_LOGI(TAG, "[BLE] Download complete: %s (%lu bytes sent)",
             filename.c_str(), (unsigned long)offset);
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

    // State
    out.state = rocketStateToString(lora.rocket_state);

    // Camera recording (from LoRa downlink flags)
    out.camera_recording = lora.camera_recording;
    // Rocket logging state (actual, from LoRa downlink)
    out.logging_active = last_known_rocket_logging;
    out.active_file = "";

    // Base station's own CSV logging state (shown as separate indicator)
    out.bs_logging_active = logging_active;

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

    ESP_LOGI(TAG, "[RX] %s | alt=%.0fm spd=%.1fm/s | %.0fdBm SNR=%.1f | sats=%u | %.2fV %.0f%%",
             rocketStateToString(data.rocket_state),
             (double)data.pressure_alt,
             (double)data.max_speed,
             (double)rssi,
             (double)snr,
             (unsigned)data.num_sats,
             (double)data.voltage,
             (double)data.soc);
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

    const uint32_t since_last = (last_packet_ms > 0) ? (now - last_packet_ms) : 0;

    ESP_LOGI(TAG, "[STATS] RX: %lu pkts (%.1f Hz) | CRC fail: %lu | ISR: %lu | rx_mode: %d | Last RSSI: %.0f dBm SNR: %.1f dB | Last pkt %lu ms ago",
             (unsigned long)ls.rx_count,
             (double)rx_hz,
             (unsigned long)ls.rx_crc_fail,
             (unsigned long)ls.isr_count,
             (int)ls.rx_mode,
             (double)ls.last_rssi,
             (double)ls.last_snr,
             (unsigned long)since_last);
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
             ",\"lf\":%.1f,\"lsf\":%u,\"lbw\":%.0f,\"lcr\":%u,\"lpw\":%d}",
             (int)cfg_servo_bias1, (int)cfg_servo_hz,
             (int)cfg_servo_min, (int)cfg_servo_max,
             (double)cfg_pid_kp, (double)cfg_pid_ki, (double)cfg_pid_kd,
             (double)cfg_pid_min, (double)cfg_pid_max,
             cfg_servo_enabled ? "true" : "false",
             (double)lora_freq_mhz, (unsigned)lora_sf,
             (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);
    if (n < 0 || (size_t)n >= sizeof(buf)) {
        ESP_LOGW(TAG, "[CFG] Config JSON truncated! (%d bytes, buf=%u)", n, (unsigned)sizeof(buf));
    }
    String j(buf);
    ble_app.sendConfigJSON(j);
    ESP_LOGI(TAG, "[CFG] Sent config readback to app");

    delay(50);

    // Message 2: device identity ("config_identity" type)
    char id_buf[128];
    snprintf(id_buf, sizeof(id_buf),
             "{\"type\":\"config_identity\""
             ",\"uid\":\"%s\""
             ",\"un\":\"%s\""
             ",\"nid\":%u"
             ",\"dt\":\"%s\"}",
             unit_id_hex, unit_name,
             (unsigned)network_id,
             config::DEVICE_TYPE);
    String id_json(id_buf);
    ble_app.sendConfigJSON(id_json);
    ESP_LOGI(TAG, "[CFG] Sent identity readback (%u bytes)", (unsigned)id_json.length());
}

static void cacheServoConfig(const uint8_t* payload, size_t len)
{
    if (len < 14) return;
    ServoConfigData sc;
    memcpy(&sc, payload, std::min((size_t)len, sizeof(sc)));
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
    ESP_LOGI(TAG, "[CFG] Servo config cached: bias=%d hz=%d min=%d max=%d",
             sc.bias_us[0], sc.hz, sc.min_us, sc.max_us);
}

static void cachePIDConfig(const uint8_t* payload, size_t len)
{
    if (len < 20) return;
    PIDConfigData pc;
    memcpy(&pc, payload, std::min((size_t)len, sizeof(pc)));
    cfg_pid_kp = pc.kp; cfg_pid_ki = pc.ki; cfg_pid_kd = pc.kd;
    cfg_pid_min = pc.min_cmd; cfg_pid_max = pc.max_cmd;
    prefs.begin("pid", false);
    prefs.putFloat("kp", pc.kp);
    prefs.putFloat("ki", pc.ki);
    prefs.putFloat("kd", pc.kd);
    prefs.putFloat("mn", pc.min_cmd);
    prefs.putFloat("mx", pc.max_cmd);
    prefs.end();
    ESP_LOGI(TAG, "[CFG] PID config cached: Kp=%.4f Ki=%.4f Kd=%.4f [%.1f,%.1f]",
             pc.kp, pc.ki, pc.kd, pc.min_cmd, pc.max_cmd);
}

// ============================================================================
// LoRa Uplink (relay BLE commands to OutComputer)
// ============================================================================

/// Build an uplink packet with routing header.
/// target_rid: destination rocket_id (0xFF = broadcast to all rockets in network)
static void buildUplinkPacket(uint8_t cmd, const uint8_t* payload, size_t payload_len,
                              uint8_t target_rid = 0xFF)
{
    if (uplink_pending)
    {
        ESP_LOGW(TAG, "[UPLINK] Discarding pending cmd=%u, replacing with cmd=%u",
                 uplink_buf[3], cmd);
    }

    if (payload_len > 18) payload_len = 18;  // max 18 bytes payload (was 20, now 2 bytes for routing)
    // Uplink format v1: [0xCA][network_id][target_rocket_id][cmd][len][payload...]
    uplink_buf[0] = config::UPLINK_SYNC_BYTE;  // 0xCA
    uplink_buf[1] = network_id;
    uplink_buf[2] = target_rid;
    uplink_buf[3] = cmd;
    uplink_buf[4] = (uint8_t)payload_len;
    if (payload_len > 0 && payload != nullptr)
    {
        memcpy(&uplink_buf[5], payload, payload_len);
    }
    uplink_len = 5 + payload_len;
    uplink_retries_left = config::UPLINK_RETRIES;
    uplink_pending = true;
    uplink_last_tx_ms = 0;
    ESP_LOGI(TAG, "[UPLINK] Queued cmd=%u -> rid=%u payload=%u bytes, %u retries",
             cmd, target_rid, (unsigned)payload_len, config::UPLINK_RETRIES);
}

static void serviceUplink()
{
    if (!uplink_pending || uplink_retries_left == 0)
    {
        if (uplink_pending)
        {
            // All retries done — service() already auto-entered RX after
            // the last TX.  Do NOT call startReceive() here: it would reset
            // rx_done_ and drop any downlink packet that arrived between
            // the TX completion and this point.
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

    if (!lora_comms.canSend())
    {
        return;  // Previous TX still in progress
    }

    if (lora_comms.send(uplink_buf, uplink_len))
    {
        uplink_retries_left--;
        uplink_last_tx_ms = now;
        ESP_LOGI(TAG, "[UPLINK] TX attempt (%u remaining)", uplink_retries_left);
    }
    else
    {
        ESP_LOGW(TAG, "[UPLINK] send() failed, will retry");
    }
}

static void setup_bs()
{
    delay(500);
    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "  TinkerRocket Base Station");
    ESP_LOGI(TAG, "======================================");

    // Initialize storage: prefer SD (SDMMC 4-bit), fall back to SPIFFS on internal flash.
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

        esp_err_t ret = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot,
                                                 &mount_cfg, &sd_card);
        if (ret == ESP_OK)
        {
            sdmmc_card_print_info(stdout, sd_card);

            FATFS* fs;
            DWORD free_clust;
            if (f_getfree("0:", &free_clust, &fs) == FR_OK)
            {
                uint64_t total = (uint64_t)(fs->n_fatent - 2) * fs->csize * 512;
                uint64_t free_bytes = (uint64_t)free_clust * fs->csize * 512;
                uint64_t used = total - free_bytes;
                ESP_LOGI(TAG, "SD card mounted: %llu MB total, %llu MB used, %llu MB free",
                         (unsigned long long)(total / (1024 * 1024)),
                         (unsigned long long)(used / (1024 * 1024)),
                         (unsigned long long)(free_bytes / (1024 * 1024)));
            }
        }
        else
        {
            ESP_LOGW(TAG, "SD card mount failed (0x%x) — falling back to internal flash (SPIFFS)",
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
        if (sd_card || using_internal_flash)
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
                         using_internal_flash ? "internal flash" : "SD card");
            }
            else
            {
                ESP_LOGE(TAG, "Storage write test FAILED! errno=%d (%s)",
                         errno, strerror(errno));
            }
        }
    }

    // Configure I2C for MAX17205G fuel gauge
    {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port     = I2C_NUM_0;
        bus_cfg.sda_io_num   = (gpio_num_t)config::I2C_SDA_PIN;
        bus_cfg.scl_io_num   = (gpio_num_t)config::I2C_SCL_PIN;
        bus_cfg.clk_source   = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.flags.enable_internal_pullup = false;

        esp_err_t err = i2c_new_master_bus(&bus_cfg, &i2c_bus);
        if (err == ESP_OK &&
            i2c_master_probe(i2c_bus, config::MAX17205_ADDR, pdMS_TO_TICKS(50)) == ESP_OK)
        {
            TR_MAX17205G_Config fg_cfg;
            fg_cfg.design_mah     = config::BATTERY_DESIGN_MAH;
            fg_cfg.rsense_mohm    = config::RSENSE_MOHM;
            fg_cfg.current_invert = true;   // R_SENSE reversed on this board
            fg_cfg.num_cells      = config::NUM_BATTERY_CELLS;

            if (fuel_gauge.begin(i2c_bus, fg_cfg, config::I2C_FREQ_HZ) == ESP_OK)
            {
                fuel_gauge_present = true;
                ESP_LOGI(TAG, "MAX17205G fuel gauge found on I2C (0x%02X)", config::MAX17205_ADDR);
                fuel_gauge.initIfNeeded();
                updateBattery();
                ESP_LOGI(TAG, "Battery: %.2f V, %.1f%% SoC, %.0f mA",
                         (double)bs_voltage, (double)bs_soc, (double)bs_current);
            }
            else
            {
                ESP_LOGW(TAG, "MAX17205G probe succeeded but begin() failed");
            }
        }
        else
        {
            ESP_LOGW(TAG, "MAX17205G not found — battery readings unavailable");
        }
    }

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
        ESP_LOGI(TAG, "[CFG] LoRa NVS empty -- wrote config.h defaults");
    }
    lora_freq_mhz = prefs.getFloat("freq", config::LORA_FREQ_MHZ);
    lora_sf        = prefs.getUChar("sf",   config::LORA_SF);
    lora_bw_khz    = prefs.getFloat("bw",   config::LORA_BW_KHZ);
    lora_cr        = prefs.getUChar("cr",   config::LORA_CR);
    lora_tx_power  = (int8_t)prefs.getChar("txpwr", config::LORA_TX_POWER_DBM);
    prefs.end();
    ESP_LOGI(TAG, "[CFG] LoRa NVS: %.1f MHz SF%u BW%.0f CR%u %d dBm",
             (double)lora_freq_mhz, (unsigned)lora_sf,
             (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);

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

        ESP_LOGI(TAG, "[CFG] Identity: uid=%s name=%s nid=%u",
                 unit_id_hex, unit_name, (unsigned)network_id);

        ble_app.setName(unit_name);
    }

    // Configure LoRa radio (uses NVS-saved config or factory defaults)
    TR_LoRa_Comms::Config lora_cfg = {};
    lora_cfg.enabled           = true;
    lora_cfg.cs_pin            = config::LORA_CS_PIN;
    lora_cfg.dio1_pin          = config::LORA_DIO1_PIN;
    lora_cfg.rst_pin           = config::LORA_RST_PIN;
    lora_cfg.busy_pin          = config::LORA_BUSY_PIN;
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

    if (!lora_comms.begin(lora_cfg, config::DEBUG))
    {
        ESP_LOGE(TAG, "LoRa init FAILED!");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    ESP_LOGI(TAG, "LoRa config: %.1f MHz SF%u BW%.0f kHz CR%u %d dBm",
             (double)lora_freq_mhz,
             (unsigned)lora_sf,
             (double)lora_bw_khz,
             (unsigned)lora_cr,
             (int)lora_tx_power);

    // Start continuous receive mode
    if (!lora_comms.startReceive())
    {
        ESP_LOGE(TAG, "LoRa startReceive FAILED!");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
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
    // Service LoRa (complete any pending TX before checking for RX)
    lora_comms.service();
    lora_comms.pollDio1();  // Fallback if DIO1 interrupt doesn't fire

    // Check for received packet
    uint8_t rx_buf[256];
    size_t rx_len = 0;

    if (lora_comms.readPacket(rx_buf, sizeof(rx_buf), rx_len))
    {
        last_packet_ms = millis();

        // --- Name beacon: [0xBE][network_id][rocket_id][unit_name...] ---
        if (rx_len >= 3 && rx_buf[0] == LORA_BEACON_SYNC)
        {
            uint8_t bcn_nid = rx_buf[1];
            uint8_t bcn_rid = rx_buf[2];
            if (bcn_nid == network_id)
            {
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
        // --- Telemetry packet: SIZE_OF_LORA_DATA (59) bytes ---
        else if (rx_len == SIZE_OF_LORA_DATA)
        {
            // Decode the telemetry packet
            LoRaDataSI decoded = {};
            sensor_converter.unpackLoRa(rx_buf, decoded);

            // Filter by network_id
            if (decoded.network_id != network_id)
            {
                // Not our network — ignore
            }
            else
            {
                // Route to per-rocket tracker
                int slot = findOrAllocRocket(decoded.rocket_id);

                TR_LoRa_Comms::Stats ls = {};
                lora_comms.getStats(ls);

                // Convert ECEF to lat/lon
                double lat_deg = NAN, lon_deg = NAN, alt_m = NAN;
                if (decoded.ecef_x != 0.0 || decoded.ecef_y != 0.0 || decoded.ecef_z != 0.0)
                {
                    coord.ecefToGeodetic(decoded.ecef_x, decoded.ecef_y, decoded.ecef_z,
                                         lat_deg, lon_deg, alt_m);
                }

                printTelemetry(decoded, ls.last_rssi, ls.last_snr, lat_deg, lon_deg, alt_m);

                // Base station CSV logging: auto-start on PRELAUNCH transition
                if (decoded.rocket_state == 2 && last_rocket_state != 2)
                {
                    startLogging();
                }

                last_known_rocket_logging = decoded.logging_active;

                if (logging_active)
                {
                    logLoRaPacket(decoded, ls.last_rssi, ls.last_snr,
                                  lat_deg, lon_deg, alt_m);
                }

                // Track LANDED timing
                if (decoded.rocket_state == 4 && landed_time_ms == 0)
                {
                    landed_time_ms = millis();
                    ESP_LOGI(TAG, "[LOG] LANDED state detected, starting post-landing timer");
                }
                else if (decoded.rocket_state != 4)
                {
                    landed_time_ms = 0;
                }

                if (logging_active &&
                    decoded.rocket_state == 1 && last_rocket_state >= 2)
                {
                    ESP_LOGI(TAG, "[LOG] Rocket returned to READY, closing base station log");
                    stopLogging();
                }

                last_rocket_state = decoded.rocket_state;
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
            }
        }
        else
        {
            ESP_LOGW(TAG, "[RX] Unexpected packet size: %u (expected %u or beacon)",
                     (unsigned)rx_len, (unsigned)SIZE_OF_LORA_DATA);
        }
    }

    // Post-landing timeout: close log 30s after LANDED
    if (logging_active && landed_time_ms > 0 &&
        (millis() - landed_time_ms) >= config::LOG_LANDED_TIMEOUT_MS)
    {
        ESP_LOGI(TAG, "[LOG] Post-landing timeout, closing log file");
        stopLogging();
    }

    // Silence timeout (safety net): close log if no packets for 30s
    if (logging_active && (millis() - log_last_write_ms) >= config::LOG_SILENCE_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "[LOG] Silence timeout, closing log file");
        stopLogging();
    }

    // Periodic flush to flash
    if (logging_active && (millis() - log_last_flush_ms) >= config::LOG_FLUSH_INTERVAL_MS)
    {
        if (log_file) fflush(log_file);
        log_last_flush_ms = millis();
    }

    // Periodic battery read + standalone BLE update
    if (millis() - last_battery_ms >= config::PWR_UPDATE_PERIOD_MS)
    {
        last_battery_ms = millis();
        updateBattery();

        // Always push base station stats to BLE, even without LoRa packets.
        // Re-send last-known rocket telemetry so battery/RSSI stay up to date.
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
            }
            else
            {
                // No rocket tracked — publish base-station-only fields so the
                // app still sees BS battery/logging state. Rocket-side fields
                // stay NaN/zero.
                LoRaDataSI empty = {};
                buildBLETelemetry(empty, NAN, NAN, NAN, NAN, NAN, ble_telem);
            }
            ble_app.sendTelemetry(ble_telem);
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
    if (ble_cmd == 2)
    {
        handleFileListCommand();
    }
    else if (ble_cmd == 3)
    {
        handleDeleteCommand();
    }
    else if (ble_cmd == 4)
    {
        handleDownloadCommand();
    }
    else if (ble_cmd == 1)
    {
        // Camera toggle: send desired state (inverse of last known) so LoRa
        // retries are idempotent — won't toggle back and forth on the rocket.
        uint8_t desired = last_known_camera_recording ? 0 : 1;
        buildUplinkPacket(1, &desired, 1);
        ESP_LOGI(TAG, "[BLE->UPLINK] Camera %s", desired ? "START" : "STOP");
    }
    else if (ble_cmd == 23)
    {
        // Logging toggle: starts/stops BOTH rocket flash recording (via LoRa
        // uplink) and base station SD card logging simultaneously.
        // Base station logging state is the toggle authority — rocket follows.
        if (!logging_active)
        {
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
            ESP_LOGI(TAG, "[LOG] Base station logging stopped (manual)");

            if (last_known_rocket_logging)
            {
                uint8_t desired = 0;
                buildUplinkPacket(23, &desired, 1);
                ESP_LOGI(TAG, "[BLE->UPLINK] Rocket logging STOP");
            }
        }
    }
    else if (ble_cmd == 24)
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
    else if (ble_cmd == 25)
    {
        // Servo test stop: relay to OutComputer via LoRa uplink
        buildUplinkPacket(25, nullptr, 0);
        ESP_LOGI(TAG, "[BLE->UPLINK] Servo test stop");
    }
    else if (ble_cmd == 5)
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
    else if (ble_cmd == 6)
    {
        buildUplinkPacket(6, nullptr, 0);
        ESP_LOGI(TAG, "[BLE->UPLINK] Sim start");
    }
    else if (ble_cmd == 7)
    {
        buildUplinkPacket(7, nullptr, 0);
        ESP_LOGI(TAG, "[BLE->UPLINK] Sim stop");
    }
    else if (ble_cmd == 9)
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
        }
    }
    else if (ble_cmd == 10)
    {
        // LoRa reconfiguration (BaseStation only — NOT relayed over uplink)
        // Changing BS frequency over LoRa would break the link to OutComputer.
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

            if (lora_comms.reconfigure(new_freq, new_sf, new_bw, new_cr, new_pwr))
            {
                // Update runtime vars
                lora_freq_mhz = new_freq;
                lora_bw_khz   = new_bw;
                lora_sf        = new_sf;
                lora_cr        = new_cr;
                lora_tx_power  = new_pwr;

                // Persist to NVS
                prefs.begin("lora", false);  // read-write
                prefs.putFloat("freq",  lora_freq_mhz);
                prefs.putFloat("bw",    lora_bw_khz);
                prefs.putUChar("sf",    lora_sf);
                prefs.putUChar("cr",    lora_cr);
                prefs.putChar("txpwr",  lora_tx_power);
                prefs.end();

                // Re-enter RX mode after reconfiguration
                lora_comms.startReceive();

                ESP_LOGI(TAG, "[BLE] LoRa reconfigured + saved: %.1f MHz SF%u BW%.0f CR%u %d dBm",
                         (double)lora_freq_mhz, (unsigned)lora_sf,
                         (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);

                // Send config readback so app can confirm the actual values applied
                sendCurrentConfig();
            }
            else
            {
                ESP_LOGE(TAG, "[BLE] LoRa reconfigure FAILED");
                // Send readback with OLD config so app reverts its display
                sendCurrentConfig();
            }
        }
    }
    else if (ble_cmd == 12)
    {
        // Servo config: cache locally + relay to OutComputer via LoRa uplink
        const uint8_t* payload = ble_app.getCommandPayload();
        size_t payload_len = ble_app.getCommandPayloadLength();
        if (payload_len >= 14)
        {
            cacheServoConfig(payload, payload_len);
            buildUplinkPacket(12, payload, 14);
            ESP_LOGI(TAG, "[BLE->UPLINK] Servo config relayed");
        }
    }
    else if (ble_cmd == 13)
    {
        // PID config: cache locally + relay to OutComputer via LoRa uplink
        const uint8_t* payload = ble_app.getCommandPayload();
        size_t payload_len = ble_app.getCommandPayloadLength();
        if (payload_len >= 20)
        {
            cachePIDConfig(payload, payload_len);
            buildUplinkPacket(13, payload, 20);
            ESP_LOGI(TAG, "[BLE->UPLINK] PID config relayed");
        }
    }
    else if (ble_cmd == 14)
    {
        // Servo control enable/disable: cache + relay
        const uint8_t* payload = ble_app.getCommandPayload();
        size_t payload_len = ble_app.getCommandPayloadLength();
        if (payload_len >= 1)
        {
            cfg_servo_enabled = (payload[0] != 0);
            buildUplinkPacket(14, payload, 1);
            ESP_LOGI(TAG, "[BLE->UPLINK] Servo control: %s",
                     cfg_servo_enabled ? "ENABLE" : "DISABLE");
        }
    }
    else if (ble_cmd == 20)
    {
        // Config readback request
        sendCurrentConfig();
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
            Preferences id_prefs;
            id_prefs.begin("identity", false);
            id_prefs.putBytes("un", new_name, strlen(new_name) + 1);
            id_prefs.end();
            ble_app.setName(unit_name);
            sendCurrentConfig();
            ESP_LOGI(TAG, "[BLE] Unit name set: %s", unit_name);
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
            Preferences id_prefs;
            id_prefs.begin("identity", false);
            id_prefs.putUChar("nid", network_id);
            id_prefs.end();
            sendCurrentConfig();
            ESP_LOGI(TAG, "[BLE] Network ID set: %u", (unsigned)network_id);
        }
    }
    else if (ble_cmd == 50)
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
    else if (ble_cmd == 60)
    {
        // Frequency scan (base-station radio, pre-launch collision avoidance).
        // Payload: [start_mhz f32][stop_mhz f32][step_khz u16][dwell_ms u16]
        // All fields are little-endian.  A scan blocks normal LoRa RX for
        // the scan duration — explicitly user-initiated and short (~1-3 s).
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

            if (lora_comms.startScan(start_mhz, stop_mhz, step_khz, dwell_ms))
            {
                ESP_LOGI(TAG, "[BLE] Scan started: %.1f..%.1f MHz, %u kHz, %u ms",
                         (double)start_mhz, (double)stop_mhz,
                         (unsigned)step_khz, (unsigned)dwell_ms);
            }
            else
            {
                ESP_LOGW(TAG, "[BLE] Scan start rejected (busy or invalid range)");
            }
        }
    }

    // Service scan state machine (no-op when idle).  Must come before
    // serviceUplink so a TX retry doesn't fire while we're mid-scan — the
    // scan temporarily owns the radio's frequency.
    lora_comms.serviceScan();
    if (lora_comms.isScanDone())
    {
        const auto* samples = lora_comms.getScanSamples();
        const size_t n = lora_comms.getScanSampleCount();
        int8_t rssi[TR_LoRa_Comms::SCAN_MAX_SAMPLES];
        const size_t n_send = (n > TR_LoRa_Comms::SCAN_MAX_SAMPLES) ? TR_LoRa_Comms::SCAN_MAX_SAMPLES : n;
        for (size_t i = 0; i < n_send; ++i) rssi[i] = samples[i].rssi_dbm;
        ble_app.sendScanResults(lora_comms.getScanStartMHz(),
                                lora_comms.getScanStepKHz(),
                                rssi, (uint8_t)n_send);
        lora_comms.consumeScanDone();
    }

    // Service LoRa uplink retries (TX commands, then resume RX)
    serviceUplink();

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
