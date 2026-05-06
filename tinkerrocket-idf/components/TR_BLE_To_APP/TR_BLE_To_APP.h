#ifndef TR_BLE_TO_APP_H
#define TR_BLE_TO_APP_H

/*
 * TR_BLE_To_APP — BLE GATT server for TinkerRocket telemetry / commands.
 *
 * Back-end: ESP-IDF NimBLE host stack (host/ble_hs.h).
 * Public API is unchanged from the Arduino-BLE version so that OC and
 * base-station main.cpp files compile without modification.
 */

#ifdef ARDUINO
#include <Arduino.h>            // Pulls in String, millis(), etc.
#else
#include <string>
#include "compat.h"             // millis(), delay(), etc. for pure-IDF builds
using String = std::string;     // API-compatible subset used by callers
#endif

#include <cstdint>
#include <cstddef>

class TR_BLE_To_APP
{
public:
    // Telemetry data structure matching web GUI
    struct TelemetryData
    {
        float soc;              // Battery state of charge %
        float current;          // Battery current mA
        float voltage;          // Battery voltage V
        double latitude;        // GPS latitude degrees
        double longitude;       // GPS longitude degrees
        float gdop;             // GPS dilution of precision
        int num_sats;           // Number of GPS satellites
        const char* state;      // Rocket state string ("READY", "PRELAUNCH", etc.)
        bool camera_recording;  // Camera recording active
        bool logging_active;    // Data logging active
        const char* active_file;// Active log filename
        float rx_kbs;           // I2C RX rate kB/s
        float wr_kbs;           // Flash write rate kB/s
        uint32_t frames_rx;     // Frames received from FlightComputer
        uint32_t frames_drop;   // Frames dropped
        float max_alt_m;        // Maximum altitude meters
        float max_speed_mps;    // Maximum speed m/s
        float pressure_alt;     // Barometric pressure altitude meters
        float altitude_rate;    // Vertical rate m/s
        float gnss_alt;         // GNSS altitude meters (from ECEF conversion)

        // IMU data (ISM6HG256) - SI converted
        float low_g_x, low_g_y, low_g_z;     // Low-G accelerometer m/s²
        float high_g_x, high_g_y, high_g_z;  // High-G accelerometer m/s²
        float gyro_x, gyro_y, gyro_z;        // Gyroscope deg/s

        // Attitude (from FlightComputer onboard estimation)
        float roll;             // Roll angle degrees
        float pitch;            // Pitch angle degrees
        float yaw;              // Yaw angle degrees
        float roll_cmd;         // Roll command degrees (PID output)
        float q0, q1, q2, q3;  // Quaternion (scalar-first, body-to-NED)

        // LoRa signal quality (base station only, NaN for direct connection)
        float rssi;             // LoRa RSSI dBm
        float snr;              // LoRa SNR dB

        // Base station (only meaningful when connected via base station)
        float bs_soc;           // Base station battery SOC %
        float bs_voltage;       // Base station battery voltage V
        float bs_current;       // Base station battery current mA (NaN if no sensor)
        bool  bs_logging_active;// Base station CSV logging active

        // Flight event flags (from FlightComputer state machine)
        bool launch_flag;       // Launch detected
        bool vel_u_apogee_flag; // Velocity apogee (vertical vel crossed zero)
        bool alt_apogee_flag;   // Altitude apogee (alt started decreasing)
        bool alt_landed_flag;   // Landing detected

        // Power rail state
        bool pwr_pin_on;        // true = FlightComputer + sensors powered on

        // Pyro channel status
        bool pyro1_armed;
        bool pyro1_cont;        // true = continuity OK (load present)
        bool pyro1_fired;
        bool pyro2_armed;
        bool pyro2_cont;
        bool pyro2_fired;

        // Source rocket identity (base station only — for multi-rocket demux)
        uint8_t source_rocket_id;       // 0 = not set (direct connection)
        const char* source_unit_name;   // nullptr = not set

        // Telemetry freshness status (#95).  LIVE = packet just decoded
        // (default).  STALE = BS re-pushing cached data older than
        // BLE_TELEMETRY_STALE_MS; iOS dims + shows "stale (Ns ago)".
        // SYNCING = no rocket has ever been caught; iOS hides rocket
        // fields and shows "Searching for rocket…".  Direct rocket
        // connections always send LIVE.
        enum class DataStatus : uint8_t { LIVE = 0, STALE = 1, SYNCING = 2 };
        DataStatus data_status;
        uint32_t   data_age_ms;         // only meaningful when STALE
    };

    // Constructor
    // device_name: BLE device name (e.g., "TR-R-A1B2")
    explicit TR_BLE_To_APP(const char* device_name);

    // Change the advertised BLE device name at runtime.
    // Safe to call before or after begin().  If called after begin(),
    // stops and restarts advertising so scanners see the new name.
    void setName(const char* name);

    // Initialize BLE server and services
    // Returns true on success
    bool begin();

    // Call frequently from main loop (handles BLE events)
    void loop();

    // Check if a device is connected
    bool isConnected() const;

    // Send telemetry update to connected device
    // Sends JSON via BLE notification
    void sendTelemetry(const TelemetryData& data);

    // Get last received command (0 = none, 1 = camera toggle, 2 = file list request,
    // 5 = sim config, 6 = sim start, 7 = sim stop, 8 = power toggle)
    // Clears the command after reading
    uint8_t getCommand();

    // Get file list page number (0-based)
    // Clears the page number after reading
    uint8_t getFileListPage();

    // Get raw command payload (for commands that carry data, e.g. sim config)
    const uint8_t* getCommandPayload() const { return pending_payload_; }
    size_t getCommandPayloadLength() const { return pending_payload_len_; }

    // Get pending delete filename (empty if none)
    // Clears the filename after reading
    String getDeleteFilename();

    // Send file list as JSON array
    // files: JSON string like [{"name":"file1.bin","size":1234},...]
    void sendFileList(const String& files_json);

    // Send frequency-scan result as a compact binary blob on the file-ops
    // characteristic.  Format (little-endian):
    //   [0][0xAA marker] [1..4][start_mhz f32] [5..8][step_khz f32]
    //   [9][n u8] [10..10+n-1][rssi i8 dBm]
    // The 0xAA leading byte disambiguates this from the JSON responses
    // (file list, config) that also use this characteristic.
    void sendScanResults(float start_mhz, float step_khz,
                         const int8_t* rssi, uint8_t n);

    // Get pending download filename (empty if none)
    // Clears the filename after reading
    String getDownloadFilename();

    // Send config JSON to connected device (config readback on connect)
    void sendConfigJSON(const String& json);

    // Send file data chunk
    // offset: byte offset in file
    // data: chunk data
    // len: chunk length
    // eof: true if this is the last chunk
    void sendFileChunk(uint32_t offset, const uint8_t* data, size_t len, bool eof);

    // Get the negotiated MTU (after connection established)
    // Returns 0 if not yet negotiated
    uint16_t getNegotiatedMTU() const { return negotiated_mtu_; }

    // Get max data bytes per BLE chunk (MTU - ATT overhead - our header)
    // Falls back to 170 if MTU not yet negotiated
    size_t getMaxChunkDataSize() const;

private:
    static constexpr size_t MAX_DEVICE_NAME_LEN = 29;   // BLE adv name limit
    char device_name_[MAX_DEVICE_NAME_LEN + 1];          // mutable, null-terminated

    volatile bool device_connected_;
    volatile uint16_t negotiated_mtu_;
    volatile uint16_t conn_handle_;          // NimBLE connection handle
    volatile uint8_t pending_command_;
    volatile uint8_t pending_file_list_page_;
    String pending_delete_filename_;
    String pending_download_filename_;
    uint8_t pending_payload_[72] = {};   // Raw payload for commands with data (max = RollProfileData 68 bytes)
    size_t  pending_payload_len_ = 0;
    String file_list_json_;              // Persistent storage for file list
    uint8_t* chunk_buffer_;              // Persistent storage for file chunks
    size_t chunk_buffer_size_;

    // UUIDs for BLE service and characteristics
    static constexpr const char* SERVICE_UUID           = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
    static constexpr const char* TELEMETRY_CHAR_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
    static constexpr const char* COMMAND_CHAR_UUID      = "cba1d466-344c-4be3-ab3f-189f80dd7518";
    static constexpr const char* FILE_OPS_CHAR_UUID     = "8d53dc1d-1db7-4cd3-868b-8a527460aa84";
    static constexpr const char* FILE_TRANSFER_CHAR_UUID= "1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d";

    // NimBLE GATT attribute handles (filled during registration)
    uint16_t telemetry_val_handle_;
    uint16_t command_val_handle_;
    uint16_t file_ops_val_handle_;
    uint16_t file_transfer_val_handle_;

    // Helper to build JSON string
    String buildTelemetryJSON(const TelemetryData& data);

public:
    // ---- NimBLE callbacks (static, forwarded via user-data pointer) --------
    static int  gap_event_cb(struct ble_gap_event* event, void* arg);
    static int  gatt_svc_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt* ctxt, void* arg);
private:
    void onConnect(uint16_t conn_handle, const struct ble_gap_conn_desc* desc);
    void onDisconnect(uint16_t conn_handle, int reason);
    void onMtuChanged(uint16_t conn_handle, uint16_t mtu);
    void onCommandWrite(const uint8_t* data, size_t length);

    // Start/restart BLE advertising
    void startAdvertising();

    // Register GATT services with the NimBLE host
    void registerGattServices();

    // NimBLE host-reset and host-sync callbacks
    static void on_ble_hs_reset(int reason);
    static void on_ble_hs_sync();

    // Static instance pointer so NimBLE callbacks can reach 'this'
    static TR_BLE_To_APP* s_instance_;
};

#endif // TR_BLE_TO_APP_H
