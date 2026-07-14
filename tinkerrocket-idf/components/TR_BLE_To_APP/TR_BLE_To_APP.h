#ifndef TR_BLE_TO_APP_H
#define TR_BLE_TO_APP_H

/*
 * TR_BLE_To_APP — BLE GATT server for TinkerRocket telemetry / commands.
 *
 * Back-end: ESP-IDF NimBLE host stack (host/ble_hs.h).
 * Public API is unchanged from the Arduino-BLE version so that OC and
 * base-station main.cpp files compile without modification.
 */

#include <string>
#include "compat.h"             // millis(), delay(), etc.
using String = std::string;     // API-compatible subset used by callers

#include <cstdint>
#include <cstddef>

#include "BleCommandRing.h"      // #517: depth for the command intake
#include "TR_OTA_Receiver.h"
#include "TR_OTA_Backend_esp.h"

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
        // Seconds until the silence-timeout closes the current BS log,
        // when bs_logging_active is true.  Counts down from
        // LOG_SILENCE_TIMEOUT_MS/1000 (300 s today) on every RX and decreases
        // as the BS goes longer without a packet.  Omitted from the BLE
        // payload when bs_logging_active is false (0xFFFF sentinel).
        uint16_t bs_log_silence_remaining_s;

        // Flight event flags (from FlightComputer state machine)
        bool launch_flag;       // Launch detected
        bool vel_u_apogee_flag; // Velocity apogee (vertical vel crossed zero)
        bool alt_apogee_flag;   // Altitude apogee (alt started decreasing)
        bool alt_landed_flag;   // Landing detected
        bool sim_active;        // #393: simulated flight in progress (NSF_SIM_ACTIVE)

        // Power rail state
        bool pwr_pin_on;        // true = FlightComputer + sensors powered on

        // Pyro channel status — new PCB shares one ARM FET across 4
        // channels, so "armed" is a single global bit mirroring the live
        // ARM-pin state (HIGH only during a cont test or fire pulse).
        bool pyro_armed;
        bool pyro_cont[4];      // true = continuity OK (load present, closed)
        bool pyro_fired[4];

        // Per-sensor health scorecard (#303): 2 bits/sensor, NA/OK/DEGRADED/BAD.
        // Layout in RocketComputerTypes.h (SH_*_SHIFT). 0 = nothing reported.
        uint32_t sensor_health;

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

    // Whether this component should request connection parameters itself on
    // connect (#503). Default true, which is what the base station wants — it has
    // no policy of its own.
    //
    // The out computer DOES have one (slow 200 ms while idle, fast 30 ms once the
    // rail is on for file transfer), and the two collide: the bench logged
    // "GAP update_params: update already in progress ... rc=2" as our deferred
    // request landed on top of the OC's. The OC happened to win that race, but had
    // it lost, the idle link would have sat at 30 ms and thrown away most of #519's
    // power saving. So the OC turns this off and owns the policy end to end.
    void setAutoConnParams(bool enabled) { auto_conn_params_ = enabled; }

    // NimBLE's handle for the current connection; 0xFFFF when not connected.
    // #519: the out computer was calling ble_gap_update_params(0, ...) with a
    // HARDCODED zero because this was never exposed. NimBLE handed out handle 1,
    // so every one of its connection-parameter requests failed with "connection
    // not found; conn_handle=0x0000" — including the slow/low-power params that
    // exist precisely to cut idle power. Callers must pass this, not a literal.
    uint16_t connHandle() const { return device_connected_ ? conn_handle_ : 0xFFFF; }

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
    // #384: return the loop-task snapshot taken atomically by getCommand(),
    // not the live latch — a second BLE write between getCommand() and this
    // call could otherwise swap (or tear) the payload under the command.
    const uint8_t* getCommandPayload() const { return consumed_.payload; }
    size_t getCommandPayloadLength() const { return consumed_.payload_len; }

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

    // Send a magnetometer calibration status frame as a compact binary
    // blob on the file-ops characteristic (issue #96).  Format:
    //   [0][0xCA marker] [1..22][MagCalStatusData LE bytes]
    // Distinct discriminator from sendScanResults' 0xAA so the iOS app
    // can route both off the same subscription.  The JSON responses (file
    // list, config) start with '[' or '{' which can never collide with
    // 0xAA / 0xCA, so all four payload kinds coexist on this channel.
    void sendMagCalStatus(const uint8_t* status_bytes, size_t len);

    // Send a sensor (gyro + high-g) calibration readback frame on the
    // file-ops characteristic (issue #132).  Format:
    //   [0][0xCB marker] [1..19][SensorCalStatusData LE bytes]
    // Sibling discriminator to mag cal's 0xCA.
    void sendSensorCalStatus(const uint8_t* status_bytes, size_t len);

    // Send a flash-space stats frame for the storage bar on the file-ops
    // characteristic.  Format: [0]=marker (0xCC rocket / 0xCD base station)
    // [1..]=packed struct LE bytes (RocketStorageStatsData / BaseStationStorageStatsData).
    void sendStorageStats(uint8_t marker, const uint8_t* bytes, size_t len);

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
    // Send one file-transfer chunk. Returns false if the chunk could NOT be
    // notified even after the full reliable backpressure budget (#524) — the
    // caller must then abort the transfer rather than carry on and hand the app a
    // file with a hole in it. Unlike telemetry, a dropped chunk is lost data.
    bool sendFileChunk(uint32_t offset, const uint8_t* data, size_t len, bool eof);

    // Get the negotiated MTU (after connection established)
    // Returns 0 if not yet negotiated
    uint16_t getNegotiatedMTU() const { return negotiated_mtu_; }

    // True once the central is ready to receive a notify-based push: it has
    // negotiated a larger MTU AND enabled notifications (CCCD) on the
    // telemetry/config characteristic.  loop_oc gates the connect-time config
    // auto-push on this so the notifies aren't dropped (#224 — the faster idle
    // loop from #221 made the old fixed delay(500) race the MTU + subscribe).
    bool isReadyForNotify() const {
        // #283: gate on the LIVE ATT MTU (effectiveMtu), not the cached
        // negotiated_mtu_.  Some iOS reconnect paths reuse the prior MTU
        // without re-firing BLE_GAP_EVENT_MTU, leaving negotiated_mtu_ stuck
        // at 0 even though the link MTU is already large — which would block
        // the connect-time config push forever.  "> 23" (the default ATT MTU)
        // means the MTU actually grew, preserving the #224 "wait for MTU
        // before pushing" intent.
        return device_connected_ && effectiveMtu() > 23 && telem_notify_subscribed_;
    }

    // Get max data bytes per BLE chunk (MTU - ATT overhead - our header)
    // Falls back to 170 if MTU not yet negotiated
    size_t getMaxChunkDataSize() const;

    // True from OTA_BEGIN until the session ends (finish→reboot, abort, or
    // failure). main.cpp gates I2C battery-gauge polling on this so the
    // esp_ota_begin() flash erase doesn't collide with the I2C bus (#17).
    bool isOtaActive() const { return ota_session_active_; }

private:
    static constexpr size_t MAX_DEVICE_NAME_LEN = 29;   // BLE adv name limit
    char device_name_[MAX_DEVICE_NAME_LEN + 1];          // mutable, null-terminated

    volatile bool device_connected_;
    volatile uint16_t negotiated_mtu_;
    volatile bool telem_notify_subscribed_;  // central enabled notifications on telemetry/config char
    volatile uint16_t conn_handle_;          // NimBLE connection handle
    // #517: a RING, not a single latch (see BleCommandRing.h). It used to be
    // one slot overwritten by the NimBLE write callback, so two commands
    // arriving between two loop polls lost the first outright.
    //
    // #384 (keep): the ring is written on the NimBLE host task and read on the
    // loop task; push() and pop() each run inside the s_cmd_mux critical
    // section, so a command and its payload/filename/page still travel as one
    // atomic unit and can never be mismatched.
    tr_ble::CommandRing cmd_ring_;

    // Loop-task-only snapshot, filled by getCommand() under the mux. The
    // payload/filename/page accessors read THIS, never the ring, so a later
    // BLE write can't mutate what the current command's handler is using.
    tr_ble::PendingCommand consumed_;
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

    // Authoritative current ATT MTU for this connection (#283).  Reads the live
    // NimBLE value (ble_att_mtu) so a missed BLE_GAP_EVENT_MTU callback can't
    // strand us at the stale cached negotiated_mtu_; falls back to the cache if
    // the stack query fails.  Returns 0 when disconnected, 23 before the
    // exchange completes, the negotiated value after.
    uint16_t effectiveMtu() const;

    // Max payload bytes for one GATT notification = effectiveMtu() - ATT(3),
    // with a defensive 20-byte floor.  Single source of truth for every
    // notify-size check (telemetry / config / OTA status / build cap).
    size_t maxNotifyBytes() const;

    // Helper to build JSON string
    String buildTelemetryJSON(const TelemetryData& data);

    // ---- OTA receive state (#8) --------------------------------------------
    // Owned here because the OTA flow lives entirely on BLE characteristics
    // (cmd 10/11/12 on the command channel, chunks on file-transfer, status
    // JSON on file-ops). main.cpp only needs to drive validation post-boot.
    TR_OTA_Backend_esp ota_backend_;
    TR_OTA_Receiver    ota_receiver_;
    // When non-zero, loop() calls esp_restart() once millis() reaches it.
    // Set by the OTA_FINISH handler after the ready_to_boot notification
    // flushes so the iOS app sees the new partition selection.
    uint32_t ota_pending_restart_at_ms_ = 0;

    // #503: deferred connection-parameter request. Firing it from the connect
    // callback collided with iOS's own connection-update procedure (bench:
    // status=554 = HCI 0x2A "different transaction collision"). Let the peer go
    // first, then ask — and retry if we still lose the race.
    uint32_t conn_param_due_ms_   = 0;   // 0 = nothing scheduled
    uint8_t  conn_param_attempts_ = 0;
    bool     auto_conn_params_    = true;  // see setAutoConnParams()
    static constexpr uint32_t kConnParamDelayMs = 1000;  // let iOS settle the link first
    static constexpr uint32_t kConnParamRetryMs = 750;
    static constexpr uint8_t  kConnParamMaxAttempts = 3;
    // Throttle the per-chunk "writing" status notifications. Updated on
    // every successful chunk; we notify at most ~2 Hz so the BLE notify
    // queue isn't saturated mid-flash.
    uint32_t ota_last_writing_notify_ms_ = 0;
    // True for the duration of an OTA session (#17). Read cross-task by
    // main.cpp via isOtaActive() to pause I2C battery polling during the
    // esp_ota_begin() flash erase. volatile: written on the NimBLE host
    // task, read on the main loop task.
    volatile bool ota_session_active_ = false;

    // ---- OTA relay delegate (#8 Phase 4, OC only) --------------------------
    // When set, an OTA_BEGIN with target==1 is relayed to the FC over I2C (the
    // handlers below invoke these) instead of flashing this device. BS leaves
    // them null → target==1 returns bad_target. Invoked on the NimBLE host task.
    void (*ota_relay_begin_cb_)(void* ctx, uint32_t total_size, const uint8_t* sha256) = nullptr;
    void (*ota_relay_finish_cb_)(void* ctx) = nullptr;
    void (*ota_relay_abort_cb_)(void* ctx) = nullptr;
    // Image chunk handler (Phase 4 Layer 3): the OC pumps each relayed chunk to
    // the FC over the flipped I2S link. offset is the absolute byte offset.
    void (*ota_relay_data_cb_)(void* ctx, uint32_t offset, const uint8_t* data, size_t len) = nullptr;
    void* ota_relay_ctx_ = nullptr;
    bool  ota_relay_active_ = false;  // a target==1 relay session is in progress

    void onFileTransferWrite(const uint8_t* data, size_t length);
    void handleOtaBegin(const uint8_t* data, size_t length);
    void handleOtaFinish();
    void handleOtaAbort();
    void sendOtaStatusJSON(const char* state, const char* err,
                           size_t bytes, const char* fw);
    static void otaStatusCallback(void* user, TR_OTA_Receiver::State state,
                                   TR_OTA_Receiver::Error err, size_t bytes_written);

public:
    // ---- OTA relay wiring (#8 Phase 4, OC only) ----------------------------
    // The OC registers callbacks that relay OTA_BEGIN/FINISH/ABORT to the FC
    // over I2C. relayFcOtaStatus() pushes the FC's status (already mapped to
    // the ota_status JSON vocabulary) back up to the app and ends the relay
    // session on a terminal state. BS never registers → target==1 = bad_target.
    void setOtaRelayDelegate(void (*begin_cb)(void*, uint32_t, const uint8_t*),
                             void (*finish_cb)(void*),
                             void (*abort_cb)(void*),
                             void (*data_cb)(void*, uint32_t, const uint8_t*, size_t),
                             void* ctx);
    bool isOtaRelayActive() const { return ota_relay_active_; }
    void relayFcOtaStatus(const char* state, const char* err,
                          uint32_t bytes_written, bool terminal);

    // ---- NimBLE callbacks (static, forwarded via user-data pointer) --------
    static int  gap_event_cb(struct ble_gap_event* event, void* arg);
    static int  gatt_svc_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt* ctxt, void* arg);
private:
    void onConnect(uint16_t conn_handle, const struct ble_gap_conn_desc* desc);
    void onDisconnect(uint16_t conn_handle, int reason);
    void onMtuChanged(uint16_t conn_handle, uint16_t mtu);
    // #503: connection-parameter negotiation. Deferred out of the connect
    // callback so it doesn't collide with the peer's own update procedure.
    void requestConnParams();
    void onConnParamsUpdated(uint16_t conn_handle, int status);
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
