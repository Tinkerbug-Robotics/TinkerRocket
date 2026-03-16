#include <Arduino.h>
#include <SPI.h>
#include <LittleFS.h>
#include <Preferences.h>

#include "config.h"

#include <TR_LoRa_Comms.h>
#include <TR_Sensor_Data_Converter.h>
#include <TR_Coordinates.h>
#include <TR_BLE_To_APP.h>
#include <RocketComputerTypes.h>

static TR_LoRa_Comms lora_comms;
static SPIClass lora_spi(HSPI);
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

// Base station battery
static float bs_voltage = NAN;
static float bs_soc = NAN;
static uint32_t last_battery_ms = 0;

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

// LoRa uplink state (BaseStation → OutComputer)
static uint8_t  uplink_buf[24];   // 3-byte header + up to 20 bytes payload (PID config needs 20)
static size_t   uplink_len = 0;
static uint8_t  uplink_retries_left = 0;
static uint32_t uplink_last_tx_ms = 0;
static bool     uplink_pending = false;

// CSV logging state
static File log_file;
static bool logging_active = false;
static uint32_t log_start_ms = 0;
static uint32_t log_last_write_ms = 0;
static uint32_t log_last_flush_ms = 0;
static uint32_t landed_time_ms = 0;     // When LANDED state first seen (0 = not landed)
static uint8_t  last_rocket_state = 0;  // Track state transitions
static bool     last_known_camera_recording = false;  // Track rocket camera state for idempotent uplink
static bool     last_known_rocket_logging = false;    // Actual rocket logging state from LoRa downlink
static char log_filename[32] = "";

// Time sync state (UTC time reference from phone via BLE)
static bool time_synced = false;
static uint32_t time_sync_millis = 0;
static uint16_t sync_year = 0;
static uint8_t  sync_month = 0, sync_day = 0;
static uint8_t  sync_hour = 0, sync_minute = 0, sync_second = 0;

// Read battery voltage at pack terminals via resistor divider
static float readBatteryVoltage()
{
    int raw = analogRead(config::VOLTAGE_PIN);
    float v_adc = (raw * config::VREF) / config::ADC_MAX;
    return v_adc * ((config::R1 + config::R2) / config::R2);
}

// Estimate SOC from per-cell voltage (simple linear LiPo approximation)
static float estimateSocFromPerCellVoltage(float v_cell)
{
    const float V_MIN = 3.30f;  // empty
    const float V_MAX = 4.20f;  // full
    if (v_cell <= V_MIN) return 0.0f;
    if (v_cell >= V_MAX) return 100.0f;
    return (v_cell - V_MIN) * 100.0f / (V_MAX - V_MIN);
}

static void updateBattery()
{
    bs_voltage = readBatteryVoltage();
    float v_cell = bs_voltage / config::NUM_BATTERY_CELLS;
    bs_soc = estimateSocFromPerCellVoltage(v_cell);
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
    File root = LittleFS.open("/");
    if (!root) return 1;

    File f = root.openNextFile();
    while (f)
    {
        const char* name = f.name();
        // Skip leading '/' if present
        if (name[0] == '/') name++;
        uint16_t num = 0;
        if (sscanf(name, "lora_%hu.csv", &num) == 1)
        {
            if (num > max_num) max_num = num;
        }
        f = root.openNextFile();
    }
    return max_num + 1;
}

static void startLogging()
{
    if (logging_active)
    {
        log_file.close();
        Serial.printf("[LOG] Closed previous log: %s\n", log_filename);
    }

    if (time_synced)
    {
        // Use timestamped filename (matches rocket's flight_YYYYMMDD_HHMMSS naming)
        uint16_t y; uint8_t mo, d, h, mi, s;
        getCurrentTime(y, mo, d, h, mi, s);
        snprintf(log_filename, sizeof(log_filename),
                 "/lora_%04u%02u%02u_%02u%02u%02u.csv",
                 y, mo, d, h, mi, s);
    }
    else
    {
        // Fallback to sequential numbering if no time sync
        uint16_t num = findNextFileNumber();
        snprintf(log_filename, sizeof(log_filename), "/lora_%03u.csv", num);
    }

    log_file = LittleFS.open(log_filename, "w");
    if (!log_file)
    {
        Serial.printf("[LOG] Failed to open %s for writing!\n", log_filename);
        logging_active = false;
        return;
    }

    // Write CSV header
    log_file.println("time_ms,state,num_sats,pdop,lat,lon,alt_m,h_acc,"
                     "acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,"
                     "pressure_alt,alt_rate,max_alt,max_speed,"
                     "voltage,current,soc,roll,pitch,yaw,speed,"
                     "launch,vel_apo,alt_apo,landed,rssi,snr");

    logging_active = true;
    log_start_ms = millis();
    log_last_write_ms = millis();
    log_last_flush_ms = millis();
    landed_time_ms = 0;

    Serial.printf("[LOG] Started logging: %s\n", log_filename);
}

static void stopLogging()
{
    if (!logging_active) return;

    log_file.close();
    logging_active = false;
    landed_time_ms = 0;

    Serial.printf("[LOG] Closed log: %s\n", log_filename);
}

static uint32_t log_write_count = 0;  // Tracks calls for periodic flash check

static void logLoRaPacket(const LoRaDataSI& data, float rssi, float snr,
                          double lat, double lon, double alt)
{
    if (!logging_active) return;

    // Periodic flash usage check (every 100 writes)
    if (++log_write_count % 100 == 0)
    {
        size_t used = LittleFS.usedBytes();
        size_t total = LittleFS.totalBytes();
        if (total > 0 && used > (total * 9 / 10))
        {
            Serial.printf("[LOG] WARNING: Flash nearly full! %u/%u bytes (%.0f%%)\n",
                          (unsigned)used, (unsigned)total,
                          (double)used * 100.0 / (double)total);
        }
    }

    uint32_t time_ms = millis() - log_start_ms;

    int written = log_file.printf("%lu,%s,%u,%.1f,%.7f,%.7f,%.1f,%.1f,"
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
        Serial.println("[LOG] WARNING: log_file.printf() failed (write returned <= 0)");
    }

    log_last_write_ms = millis();
}

// ============================================================================
// BLE File Command Handlers
// ============================================================================

static void handleFileListCommand()
{
    uint8_t page = ble_app.getFileListPage();

    // Collect all CSV files
    struct FileEntry { char name[32]; uint32_t size; };
    FileEntry entries[64];
    size_t total = 0;

    File root = LittleFS.open("/");
    if (root)
    {
        File f = root.openNextFile();
        while (f && total < 64)
        {
            if (!f.isDirectory())
            {
                const char* fname = f.name();
                if (fname[0] == '/') fname++;  // Strip leading slash

                // Skip the currently active log file
                const char* active = log_filename;
                if (active[0] == '/') active++;
                if (logging_active && strcmp(fname, active) == 0)
                {
                    f = root.openNextFile();
                    continue;
                }

                strncpy(entries[total].name, fname, 31);
                entries[total].name[31] = '\0';
                entries[total].size = f.size();
                total++;
            }
            f = root.openNextFile();
        }
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
        json += String(entries[i].size);
        json += "}";
    }
    json += "]";

    ble_app.sendFileList(json);
    Serial.printf("[BLE] Sent file list page %u: %u files (total %u)\n",
                  (unsigned)page, (unsigned)(end - start), (unsigned)total);
}

static void handleDeleteCommand()
{
    String filename = ble_app.getDeleteFilename();
    if (filename.length() == 0) return;

    // Don't delete the active log file
    String active = log_filename;
    if (active.startsWith("/")) active = active.substring(1);
    if (logging_active && filename == active)
    {
        Serial.printf("[BLE] Cannot delete active log: %s\n", filename.c_str());
        return;
    }

    String path = "/" + filename;
    if (LittleFS.remove(path))
    {
        Serial.printf("[BLE] Deleted: %s\n", filename.c_str());
    }
    else
    {
        Serial.printf("[BLE] Delete failed: %s\n", filename.c_str());
    }

    // Send updated file list (page 0)
    // Temporarily set page to 0 for the response
    handleFileListCommand();
}

static void handleDownloadCommand()
{
    String filename = ble_app.getDownloadFilename();
    if (filename.length() == 0) return;

    String path = "/" + filename;
    File f = LittleFS.open(path, "r");
    if (!f)
    {
        Serial.printf("[BLE] Download failed, file not found: %s\n", filename.c_str());
        ble_app.sendFileChunk(0, nullptr, 0, true);  // Send empty EOF
        return;
    }

    uint32_t file_size = f.size();
    uint32_t offset = 0;
    uint8_t chunk_buf[config::BLE_FILE_CHUNK_SIZE];

    Serial.printf("[BLE] Starting download: %s (%lu bytes)\n",
                  filename.c_str(), (unsigned long)file_size);

    while (f.available())
    {
        if (!ble_app.isConnected())
        {
            Serial.println("[BLE] Disconnected during download, aborting");
            break;
        }
        size_t to_read = config::BLE_FILE_CHUNK_SIZE;
        size_t got = f.read(chunk_buf, to_read);
        bool eof = !f.available();
        ble_app.sendFileChunk(offset, chunk_buf, got, eof);
        offset += got;
        delay(config::BLE_CHUNK_DELAY_MS);
    }

    // Handle empty file
    if (file_size == 0)
    {
        ble_app.sendFileChunk(0, nullptr, 0, true);
    }

    f.close();
    Serial.printf("[BLE] Download complete: %s (%lu bytes sent)\n",
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
    out.bs_current = NAN;  // No current sensor on base station

    // Flight event flags (from LoRa packet)
    out.launch_flag       = lora.launch_flag;
    out.vel_u_apogee_flag = lora.vel_u_apogee_flag;
    out.alt_apogee_flag   = lora.alt_apogee_flag;
    out.alt_landed_flag   = lora.alt_landed_flag;
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

    Serial.printf("[RX] %s | alt=%.0fm spd=%.1fm/s | %.0fdBm SNR=%.1f | sats=%u | %.2fV %.0f%%\n",
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

    Serial.printf("[STATS] RX: %lu pkts (%.1f Hz) | CRC fail: %lu | ISR: %lu | rx_mode: %d | Last RSSI: %.0f dBm SNR: %.1f dB | Last pkt %lu ms ago\n",
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
    String j = "{\"type\":\"config\"";
    j += ",\"sb1\":"; j += cfg_servo_bias1;
    j += ",\"shz\":"; j += cfg_servo_hz;
    j += ",\"smn\":"; j += cfg_servo_min;
    j += ",\"smx\":"; j += cfg_servo_max;
    j += ",\"kp\":";  j += String(cfg_pid_kp, 4);
    j += ",\"ki\":";  j += String(cfg_pid_ki, 4);
    j += ",\"kd\":";  j += String(cfg_pid_kd, 4);
    j += ",\"pmn\":"; j += String(cfg_pid_min, 1);
    j += ",\"pmx\":"; j += String(cfg_pid_max, 1);
    j += ",\"sen\":"; j += cfg_servo_enabled ? "true" : "false";
    // LoRa settings — so app can verify actual device config
    j += ",\"lf\":";  j += String(lora_freq_mhz, 1);
    j += ",\"lsf\":"; j += lora_sf;
    j += ",\"lbw\":"; j += String(lora_bw_khz, 0);
    j += ",\"lcr\":"; j += lora_cr;
    j += ",\"lpw\":"; j += lora_tx_power;
    j += "}";
    ble_app.sendConfigJSON(j);
    Serial.println("[CFG] Sent config readback to app");
}

static void cacheServoConfig(const uint8_t* payload, size_t len)
{
    if (len < 14) return;
    ServoConfigData sc;
    memcpy(&sc, payload, min((size_t)len, sizeof(sc)));
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
    Serial.printf("[CFG] Servo config cached: bias=%d hz=%d min=%d max=%d\n",
        sc.bias_us[0], sc.hz, sc.min_us, sc.max_us);
}

static void cachePIDConfig(const uint8_t* payload, size_t len)
{
    if (len < 20) return;
    PIDConfigData pc;
    memcpy(&pc, payload, min((size_t)len, sizeof(pc)));
    cfg_pid_kp = pc.kp; cfg_pid_ki = pc.ki; cfg_pid_kd = pc.kd;
    cfg_pid_min = pc.min_cmd; cfg_pid_max = pc.max_cmd;
    prefs.begin("pid", false);
    prefs.putFloat("kp", pc.kp);
    prefs.putFloat("ki", pc.ki);
    prefs.putFloat("kd", pc.kd);
    prefs.putFloat("mn", pc.min_cmd);
    prefs.putFloat("mx", pc.max_cmd);
    prefs.end();
    Serial.printf("[CFG] PID config cached: Kp=%.4f Ki=%.4f Kd=%.4f [%.1f,%.1f]\n",
        pc.kp, pc.ki, pc.kd, pc.min_cmd, pc.max_cmd);
}

// ============================================================================
// LoRa Uplink (relay BLE commands to OutComputer)
// ============================================================================

static void buildUplinkPacket(uint8_t cmd, const uint8_t* payload, size_t payload_len)
{
    // If an uplink is already pending, discard it and overwrite with the new
    // command.  This avoids a blocking spin-loop that could starve the main
    // loop (BLE, LoRa RX, watchdog) when commands arrive faster than the
    // radio can transmit them.
    if (uplink_pending)
    {
        Serial.printf("[UPLINK] Discarding pending cmd=%u, replacing with cmd=%u\n",
                      uplink_buf[1], cmd);
    }

    if (payload_len > 20) payload_len = 20;
    uplink_buf[0] = config::UPLINK_SYNC_BYTE;  // 0xCA
    uplink_buf[1] = cmd;
    uplink_buf[2] = (uint8_t)payload_len;
    if (payload_len > 0 && payload != nullptr)
    {
        memcpy(&uplink_buf[3], payload, payload_len);
    }
    uplink_len = 3 + payload_len;
    uplink_retries_left = config::UPLINK_RETRIES;
    uplink_pending = true;
    uplink_last_tx_ms = 0;  // Force immediate first send
    Serial.printf("[UPLINK] Queued cmd=%u payload=%u bytes, %u retries\n",
                  cmd, (unsigned)payload_len, config::UPLINK_RETRIES);
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
        Serial.printf("[UPLINK] TX attempt (%u remaining)\n", uplink_retries_left);
    }
    else
    {
        Serial.println("[UPLINK] send() failed, will retry");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("======================================");
    Serial.println("  TinkerRocket Base Station");
    Serial.println("======================================");

    // Initialize LittleFS for CSV logging
    if (!LittleFS.begin(true))  // true = format on first use
    {
        Serial.println("LittleFS mount FAILED!");
    }
    else
    {
        size_t total = LittleFS.totalBytes();
        size_t used = LittleFS.usedBytes();
        Serial.printf("LittleFS mounted: %u KB total, %u KB used, %u KB free\n",
                      (unsigned)(total / 1024),
                      (unsigned)(used / 1024),
                      (unsigned)((total - used) / 1024));
    }

    // Configure ADC for battery voltage reading
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(config::VOLTAGE_PIN, INPUT);
    updateBattery();

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
        Serial.println("[CFG] LoRa NVS empty — wrote config.h defaults");
    }
    lora_freq_mhz = prefs.getFloat("freq", config::LORA_FREQ_MHZ);
    lora_sf        = prefs.getUChar("sf",   config::LORA_SF);
    lora_bw_khz    = prefs.getFloat("bw",   config::LORA_BW_KHZ);
    lora_cr        = prefs.getUChar("cr",   config::LORA_CR);
    lora_tx_power  = (int8_t)prefs.getChar("txpwr", config::LORA_TX_POWER_DBM);
    prefs.end();
    Serial.printf("[CFG] LoRa NVS: %.1f MHz SF%u BW%.0f CR%u %d dBm\n",
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
    Serial.printf("[NVS] Config cache: servo bias=%d hz=%d, PID Kp=%.4f\n",
        cfg_servo_bias1, cfg_servo_hz, cfg_pid_kp);

    // Initialize SPI bus for LoRa radio
    lora_spi.begin(config::LORA_SPI_SCK,
                   config::LORA_SPI_MISO,
                   config::LORA_SPI_MOSI,
                   config::LORA_CS_PIN);

    // Configure LoRa radio (uses NVS-saved config or factory defaults)
    TR_LoRa_Comms::Config lora_cfg = {};
    lora_cfg.enabled           = true;
    lora_cfg.cs_pin            = config::LORA_CS_PIN;
    lora_cfg.dio1_pin          = config::LORA_DIO1_PIN;
    lora_cfg.rst_pin           = config::LORA_RST_PIN;
    lora_cfg.busy_pin          = config::LORA_BUSY_PIN;
    lora_cfg.freq_mhz          = lora_freq_mhz;
    lora_cfg.spreading_factor  = lora_sf;
    lora_cfg.bandwidth_khz     = lora_bw_khz;
    lora_cfg.coding_rate       = lora_cr;
    lora_cfg.preamble_len      = config::LORA_PREAMBLE_LEN;
    lora_cfg.tx_power_dbm      = lora_tx_power;
    lora_cfg.crc_on            = config::LORA_CRC_ON;
    lora_cfg.rx_boosted_gain   = config::LORA_RX_BOOSTED_GAIN;
    lora_cfg.syncword_private  = config::LORA_SYNCWORD_PRIVATE;

    if (!lora_comms.begin(lora_spi, lora_cfg, config::DEBUG))
    {
        Serial.println("LoRa init FAILED!");
        while (true) { delay(1000); }
    }

    Serial.printf("LoRa config: %.1f MHz SF%u BW%.0f kHz CR%u %d dBm\n",
                  (double)lora_freq_mhz,
                  (unsigned)lora_sf,
                  (double)lora_bw_khz,
                  (unsigned)lora_cr,
                  (int)lora_tx_power);

    // Start continuous receive mode
    if (!lora_comms.startReceive())
    {
        Serial.println("LoRa startReceive FAILED!");
        while (true) { delay(1000); }
    }

    // Initialize BLE app interface
    if (!ble_app.begin())
    {
        Serial.println("BLE init FAILED!");
        // Continue anyway - LoRa RX still works without BLE
    }
    else
    {
        Serial.println("BLE advertising as 'TinkerBaseStation'");
    }

    last_stats_ms = millis();
    Serial.println("Listening for rocket telemetry...");
    Serial.println();
}

void loop()
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

        if (rx_len == SIZE_OF_LORA_DATA)
        {
            // Decode the telemetry packet
            LoRaDataSI decoded = {};
            sensor_converter.unpackLoRa(rx_buf, decoded);

            TR_LoRa_Comms::Stats ls = {};
            lora_comms.getStats(ls);

            // Convert ECEF to lat/lon once (used by print, BLE, and CSV log)
            double lat_deg = NAN, lon_deg = NAN, alt_m = NAN;
            if (decoded.ecef_x != 0.0 || decoded.ecef_y != 0.0 || decoded.ecef_z != 0.0)
            {
                coord.ecefToGeodetic(decoded.ecef_x, decoded.ecef_y, decoded.ecef_z,
                                     lat_deg, lon_deg, alt_m);
            }

            printTelemetry(decoded, ls.last_rssi, ls.last_snr, lat_deg, lon_deg, alt_m);

            // Base station CSV logging: auto-start on PRELAUNCH transition.
            // This is the base station's own logging (separate from rocket).
            if (decoded.rocket_state == 2 && last_rocket_state != 2)
            {
                startLogging();
            }

            // Track actual rocket logging state from LoRa downlink
            last_known_rocket_logging = decoded.logging_active;

            // Log packet if active
            if (logging_active)
            {
                logLoRaPacket(decoded, ls.last_rssi, ls.last_snr,
                              lat_deg, lon_deg, alt_m);
            }

            // Track LANDED timing
            if (decoded.rocket_state == 4 && landed_time_ms == 0)
            {
                landed_time_ms = millis();
                Serial.println("[LOG] LANDED state detected, starting post-landing timer");
            }
            else if (decoded.rocket_state != 4)
            {
                landed_time_ms = 0;  // Reset if state changes away from LANDED
            }

            // Auto-stop base station logging when rocket returns to READY after a flight
            if (logging_active &&
                decoded.rocket_state == 1 && last_rocket_state >= 2)
            {
                Serial.println("[LOG] Rocket returned to READY, closing base station log");
                stopLogging();
            }

            last_rocket_state = decoded.rocket_state;
            last_known_camera_recording = decoded.camera_recording;

            // Forward telemetry to BLE app
            TR_BLE_To_APP::TelemetryData ble_telem = {};
            buildBLETelemetry(decoded, ls.last_rssi, ls.last_snr,
                              lat_deg, lon_deg, alt_m, ble_telem);
            ble_app.sendTelemetry(ble_telem);
        }
        else
        {
            Serial.printf("[RX] Unexpected packet size: %u (expected %u)\n",
                          (unsigned)rx_len, (unsigned)SIZE_OF_LORA_DATA);
        }
    }

    // Post-landing timeout: close log 30s after LANDED
    if (logging_active && landed_time_ms > 0 &&
        (millis() - landed_time_ms) >= config::LOG_LANDED_TIMEOUT_MS)
    {
        Serial.println("[LOG] Post-landing timeout, closing log file");
        stopLogging();
    }

    // Silence timeout (safety net): close log if no packets for 30s
    if (logging_active && (millis() - log_last_write_ms) >= config::LOG_SILENCE_TIMEOUT_MS)
    {
        Serial.println("[LOG] Silence timeout, closing log file");
        stopLogging();
    }

    // Periodic flush to flash
    if (logging_active && (millis() - log_last_flush_ms) >= config::LOG_FLUSH_INTERVAL_MS)
    {
        log_file.flush();
        log_last_flush_ms = millis();
    }

    // Periodic battery read
    if (millis() - last_battery_ms >= config::PWR_UPDATE_PERIOD_MS)
    {
        last_battery_ms = millis();
        updateBattery();
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
        Serial.printf("[BLE->UPLINK] Camera %s\n", desired ? "START" : "STOP");
    }
    else if (ble_cmd == 23)
    {
        // Rocket logging toggle via LoRa uplink.
        // Base station logging is automatic (PRELAUNCH → READY) and
        // NOT affected by this button — they are independent.
        // Use last known state (from LoRa downlink) for idempotent desired state.
        uint8_t desired = last_known_rocket_logging ? 0 : 1;
        buildUplinkPacket(23, &desired, 1);
        Serial.printf("[BLE->UPLINK] Rocket logging %s\n", desired ? "START" : "STOP");
    }
    else if (ble_cmd == 24)
    {
        // Servo test angles: relay 8-byte payload to OutComputer via LoRa uplink
        const uint8_t* payload = ble_app.getCommandPayload();
        const size_t plen = ble_app.getCommandPayloadLength();
        if (plen >= 8)
        {
            buildUplinkPacket(24, payload, 8);
            Serial.println("[BLE->UPLINK] Servo test angles");
        }
    }
    else if (ble_cmd == 25)
    {
        // Servo test stop: relay to OutComputer via LoRa uplink
        buildUplinkPacket(25, nullptr, 0);
        Serial.println("[BLE->UPLINK] Servo test stop");
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
            Serial.printf("[BLE->UPLINK] Sim config: mass=%.0fg thrust=%.1fN burn=%.1fs descent=%.1fm/s\n",
                          (double)mass_g, (double)thrust_n, (double)burn_s, (double)descent);
        }
    }
    else if (ble_cmd == 6)
    {
        buildUplinkPacket(6, nullptr, 0);
        Serial.println("[BLE->UPLINK] Sim start");
    }
    else if (ble_cmd == 7)
    {
        buildUplinkPacket(7, nullptr, 0);
        Serial.println("[BLE->UPLINK] Sim stop");
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

            Serial.printf("[BLE] Time synced: %04u-%02u-%02u %02u:%02u:%02u UTC\n",
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

                Serial.printf("[BLE] LoRa reconfigured + saved: %.1f MHz SF%u BW%.0f CR%u %d dBm\n",
                              (double)lora_freq_mhz, (unsigned)lora_sf,
                              (double)lora_bw_khz, (unsigned)lora_cr, (int)lora_tx_power);

                // Send config readback so app can confirm the actual values applied
                sendCurrentConfig();
            }
            else
            {
                Serial.println("[BLE] LoRa reconfigure FAILED");
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
            Serial.println("[BLE->UPLINK] Servo config relayed");
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
            Serial.println("[BLE->UPLINK] PID config relayed");
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
            Serial.printf("[BLE->UPLINK] Servo control: %s\n",
                          cfg_servo_enabled ? "ENABLE" : "DISABLE");
        }
    }
    else if (ble_cmd == 20)
    {
        // Config readback request
        sendCurrentConfig();
    }

    // Service LoRa uplink retries (TX commands, then resume RX)
    serviceUplink();

    printStats();
    yield();
}
