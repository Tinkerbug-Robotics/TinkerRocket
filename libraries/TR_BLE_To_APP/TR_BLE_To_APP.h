#ifndef TR_BLE_TO_APP_H
#define TR_BLE_TO_APP_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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
    };

    // Constructor
    // device_name: BLE device name (e.g., "TinkerRocket")
    explicit TR_BLE_To_APP(const char* device_name);

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
    const char* device_name_;
    BLEServer* ble_server_;
    BLEService* telemetry_service_;
    BLECharacteristic* telemetry_characteristic_;
    BLECharacteristic* command_characteristic_;
    BLECharacteristic* file_ops_characteristic_;
    BLECharacteristic* file_transfer_characteristic_;  // Separate characteristic for file downloads

    volatile bool device_connected_;
    volatile uint16_t negotiated_mtu_;
    volatile uint8_t pending_command_;
    volatile uint8_t pending_file_list_page_;
    String pending_delete_filename_;
    String pending_download_filename_;
    uint8_t pending_payload_[72] = {};   // Raw payload for commands with data (max = RollProfileData 68 bytes)
    size_t  pending_payload_len_ = 0;
    String file_list_json_;  // Persistent storage for file list
    uint8_t* chunk_buffer_;  // Persistent storage for file chunks
    size_t chunk_buffer_size_;

    // UUIDs for BLE service and characteristics
    static constexpr const char* SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
    static constexpr const char* TELEMETRY_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
    static constexpr const char* COMMAND_CHAR_UUID = "cba1d466-344c-4be3-ab3f-189f80dd7518";
    static constexpr const char* FILE_OPS_CHAR_UUID = "8d53dc1d-1db7-4cd3-868b-8a527460aa84";
    static constexpr const char* FILE_TRANSFER_CHAR_UUID = "1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d";  // New characteristic for file chunks

    // Server callback handlers (NimBLE stack)
    class ServerCallbacks : public BLEServerCallbacks
    {
    public:
        explicit ServerCallbacks(TR_BLE_To_APP* parent) : parent_(parent) {}
        void onConnect(BLEServer* server) override;
        void onConnect(BLEServer* server, ble_gap_conn_desc* desc) override;
        void onDisconnect(BLEServer* server) override;
        void onMtuChanged(BLEServer* server, ble_gap_conn_desc* desc, uint16_t mtu) override;
    private:
        TR_BLE_To_APP* parent_;
    };

    // Command characteristic callback
    class CommandCallbacks : public BLECharacteristicCallbacks
    {
    public:
        explicit CommandCallbacks(TR_BLE_To_APP* parent) : parent_(parent) {}
        void onWrite(BLECharacteristic* characteristic) override;
    private:
        TR_BLE_To_APP* parent_;
    };

    ServerCallbacks* server_callbacks_;
    CommandCallbacks* command_callbacks_;

    // Helper to build JSON string
    String buildTelemetryJSON(const TelemetryData& data);
};

#endif // TR_BLE_TO_APP_H
