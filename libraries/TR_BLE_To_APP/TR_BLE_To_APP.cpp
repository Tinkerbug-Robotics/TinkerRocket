#include "TR_BLE_To_APP.h"

// Spinlock protecting pending_command_ against races between the BLE
// callback (runs on the NimBLE host task) and the main loop reading it.
static portMUX_TYPE s_cmd_mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// Constructor
// ============================================================================

TR_BLE_To_APP::TR_BLE_To_APP(const char* device_name)
    : device_name_(device_name),
      ble_server_(nullptr),
      telemetry_service_(nullptr),
      telemetry_characteristic_(nullptr),
      command_characteristic_(nullptr),
      file_ops_characteristic_(nullptr),
      file_transfer_characteristic_(nullptr),
      device_connected_(false),
      negotiated_mtu_(0),
      pending_command_(0),
      pending_file_list_page_(0),
      pending_delete_filename_(""),
      file_list_json_(""),
      chunk_buffer_(nullptr),
      chunk_buffer_size_(0),
      server_callbacks_(nullptr),
      command_callbacks_(nullptr)
{
}

// ============================================================================
// Callback Implementations
// ============================================================================

void TR_BLE_To_APP::ServerCallbacks::onConnect(BLEServer* server)
{
    parent_->device_connected_ = true;
    parent_->negotiated_mtu_ = 0;  // Reset until MTU exchange completes

    Serial.print("[BLE] Device connected! Clients: ");
    Serial.println(server->getConnectedCount());
    Serial.flush();
}

void TR_BLE_To_APP::ServerCallbacks::onConnect(BLEServer* server, ble_gap_conn_desc* desc)
{
    // Request fast connection parameters from iOS for high-throughput file transfer.
    // iOS default is ~30ms interval; requesting 7.5-20ms gives 2-4x throughput.
    struct ble_gap_upd_params params = {};
    params.itvl_min = 0x06;              // 7.5ms  (units of 1.25ms)
    params.itvl_max = 0x10;              // 20ms   (units of 1.25ms)
    params.latency = 0;                  // No slave latency
    params.supervision_timeout = 200;    // 2 seconds (units of 10ms)
    params.min_ce_len = 0;
    params.max_ce_len = 0;

    int rc = ble_gap_update_params(desc->conn_handle, &params);
    if (rc == 0)
    {
        Serial.println("[BLE] Requested fast connection parameters (7.5-20ms interval)");
    }
    else
    {
        Serial.print("[BLE] Connection param update request failed, rc=");
        Serial.println(rc);
    }
}

void TR_BLE_To_APP::ServerCallbacks::onMtuChanged(BLEServer* server, ble_gap_conn_desc* desc, uint16_t mtu)
{
    parent_->negotiated_mtu_ = mtu;
    Serial.print("[BLE] MTU negotiated: ");
    Serial.print(mtu);
    Serial.print(" -> max chunk data: ");
    Serial.print(parent_->getMaxChunkDataSize());
    Serial.println(" bytes");
}

void TR_BLE_To_APP::ServerCallbacks::onDisconnect(BLEServer* server)
{
    parent_->device_connected_ = false;
    parent_->negotiated_mtu_ = 0;
    Serial.println("[BLE] Device disconnected");

    // Restart advertising so new devices can connect
    BLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising restarted");
}

void TR_BLE_To_APP::CommandCallbacks::onWrite(BLECharacteristic* characteristic)
{
    uint8_t* data = characteristic->getData();
    size_t length = characteristic->getLength();

    if (length > 0 && data != nullptr)
    {
        uint8_t cmd = data[0];

        if (cmd == 2 && length > 1)
        {
            // Command 2: File list request (optional page number follows)
            parent_->pending_file_list_page_ = data[1];
            Serial.print("[BLE] File list request, page: ");
            Serial.println(parent_->pending_file_list_page_);
        }
        else if (cmd == 2)
        {
            // Command 2 without page number defaults to page 0
            parent_->pending_file_list_page_ = 0;
        }
        else if (cmd == 3 && length > 1)
        {
            // Command 3: Delete file (filename follows command byte)
            parent_->pending_delete_filename_ = "";
            for (size_t i = 1; i < length; ++i)
            {
                parent_->pending_delete_filename_ += (char)data[i];
            }
            Serial.print("[BLE] Delete file request: ");
            Serial.println(parent_->pending_delete_filename_);
        }
        else if (cmd == 4 && length > 1)
        {
            // Command 4: Download file (filename follows command byte)
            parent_->pending_download_filename_ = "";
            for (size_t i = 1; i < length; ++i)
            {
                parent_->pending_download_filename_ += (char)data[i];
            }
            Serial.print("[BLE] Download file request: ");
            Serial.println(parent_->pending_download_filename_);
        }
        else if (cmd == 5 && length >= 13)
        {
            // Command 5: Sim config [cmd][mass_g:4][thrust_n:4][burn_s:4]
            size_t payload_len = length - 1;
            if (payload_len > sizeof(parent_->pending_payload_))
            {
                payload_len = sizeof(parent_->pending_payload_);
            }
            memcpy(parent_->pending_payload_, data + 1, payload_len);
            parent_->pending_payload_len_ = payload_len;
            Serial.println("[BLE] Sim config received");
        }
        else if (cmd == 6)
        {
            // Command 6: Start simulation
            Serial.println("[BLE] Sim start");
        }
        else if (cmd == 7)
        {
            // Command 7: Stop simulation
            Serial.println("[BLE] Sim stop");
        }
        else if (cmd == 8)
        {
            // Command 8: Toggle power rail (PWR_PIN)
            Serial.println("[BLE] Power toggle");
        }
        else if (cmd == 9 && length >= 8)
        {
            // Command 9: Time sync [cmd][year_lo][year_hi][month][day][hour][minute][second]
            size_t payload_len = length - 1;
            if (payload_len > sizeof(parent_->pending_payload_))
            {
                payload_len = sizeof(parent_->pending_payload_);
            }
            memcpy(parent_->pending_payload_, data + 1, payload_len);
            parent_->pending_payload_len_ = payload_len;
            Serial.println("[BLE] Time sync received");
        }
        else if (length > 1)
        {
            // Generic payload handler for any other command carrying data
            // (e.g. cmd 10 LoRa config, cmd 11 sound config, future commands)
            size_t payload_len = length - 1;
            if (payload_len > sizeof(parent_->pending_payload_))
            {
                payload_len = sizeof(parent_->pending_payload_);
            }
            memcpy(parent_->pending_payload_, data + 1, payload_len);
            parent_->pending_payload_len_ = payload_len;
        }
        portENTER_CRITICAL(&s_cmd_mux);
        if (parent_->pending_command_ != 0)
        {
            // Previous command not yet consumed — log and overwrite
            portEXIT_CRITICAL(&s_cmd_mux);
            Serial.printf("[BLE] WARNING: overwriting unconsumed cmd %u with %u\n",
                          parent_->pending_command_, cmd);
            portENTER_CRITICAL(&s_cmd_mux);
        }
        parent_->pending_command_ = cmd;
        portEXIT_CRITICAL(&s_cmd_mux);

        Serial.print("[BLE] Received command: ");
        Serial.println(cmd);
    }
}

// ============================================================================
// Public API
// ============================================================================

bool TR_BLE_To_APP::begin()
{
    Serial.println("[BLE] Initializing BLE server...");

    // Initialize BLE
    BLEDevice::init(device_name_);

    // Create BLE Server
    ble_server_ = BLEDevice::createServer();
    if (!ble_server_)
    {
        Serial.println("[BLE] Failed to create server!");
        return false;
    }

    // Request larger MTU for bigger JSON packets (default is 23, max is 517)
    BLEDevice::setMTU(512);
    Serial.println("[BLE] Requested MTU: 512 bytes");

    // Set server callbacks
    server_callbacks_ = new ServerCallbacks(this);
    ble_server_->setCallbacks(server_callbacks_);

    // Create Telemetry Service
    telemetry_service_ = ble_server_->createService(SERVICE_UUID);
    if (!telemetry_service_)
    {
        Serial.println("[BLE] Failed to create service!");
        return false;
    }

    // Create Telemetry Characteristic (READ + NOTIFY)
    telemetry_characteristic_ = telemetry_service_->createCharacteristic(
        TELEMETRY_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );

    if (!telemetry_characteristic_)
    {
        Serial.println("[BLE] Failed to create telemetry characteristic!");
        return false;
    }

    // Add descriptor for notifications (required for NOTIFY)
    telemetry_characteristic_->addDescriptor(new BLE2902());

    // Create Command Characteristic (WRITE)
    command_characteristic_ = telemetry_service_->createCharacteristic(
        COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );

    if (!command_characteristic_)
    {
        Serial.println("[BLE] Failed to create command characteristic!");
        return false;
    }

    // Set command callback
    command_callbacks_ = new CommandCallbacks(this);
    command_characteristic_->setCallbacks(command_callbacks_);

    // Create File Operations Characteristic (for file lists - READ + NOTIFY)
    file_ops_characteristic_ = telemetry_service_->createCharacteristic(
        FILE_OPS_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );

    if (!file_ops_characteristic_)
    {
        Serial.println("[BLE] Failed to create file ops characteristic!");
        return false;
    }

    // Add descriptor for notifications
    file_ops_characteristic_->addDescriptor(new BLE2902());

    // Create File Transfer Characteristic (for file chunks - READ + NOTIFY)
    file_transfer_characteristic_ = telemetry_service_->createCharacteristic(
        FILE_TRANSFER_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );

    if (!file_transfer_characteristic_)
    {
        Serial.println("[BLE] Failed to create file transfer characteristic!");
        return false;
    }

    // Add descriptor for notifications
    file_transfer_characteristic_->addDescriptor(new BLE2902());

    // Start the service
    telemetry_service_->start();

    // Start advertising (1 second interval to save power in low-power mode)
    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);  // 7.5ms connection interval hint for after connection
    advertising->setMinInterval(0x640);  // 1000ms (in 0.625ms units: 1000/0.625 = 1600 = 0x640)
    advertising->setMaxInterval(0x640);  // 1000ms
    BLEDevice::startAdvertising();

    Serial.println("[BLE] Server started and advertising");
    Serial.print("[BLE] Device name: ");
    Serial.println(device_name_);

    return true;
}

void TR_BLE_To_APP::loop()
{
    // BLE stack handles events via callbacks (onConnect, onMtuChanged, etc.)
}

size_t TR_BLE_To_APP::getMaxChunkDataSize() const
{
    const size_t HEADER_SIZE = 7;   // offset(4) + length(2) + flags(1)
    const size_t ATT_OVERHEAD = 3;  // ATT notification header

    if (negotiated_mtu_ > (HEADER_SIZE + ATT_OVERHEAD + 20))
    {
        // Use negotiated MTU for chunk size
        return negotiated_mtu_ - ATT_OVERHEAD - HEADER_SIZE;
    }

    // Fallback: conservative 170 bytes (fits in iOS default 185-byte MTU)
    return 170;
}

bool TR_BLE_To_APP::isConnected() const
{
    return device_connected_;
}

void TR_BLE_To_APP::sendTelemetry(const TelemetryData& data)
{
    static bool first_call = true;
    if (first_call)
    {
        Serial.print("[BLE] sendTelemetry called, device_connected_=");
        Serial.println(device_connected_ ? "true" : "false");
        first_call = false;
    }

    if (!device_connected_)
    {
        return; // No device connected, skip
    }

    if (!telemetry_characteristic_)
    {
        Serial.println("[BLE] ERROR: telemetry_characteristic_ is null!");
        return; // Not initialized
    }

    // Build JSON string
    String json = buildTelemetryJSON(data);

    // Check if JSON exceeds MTU — truncated notifications produce unparseable JSON
    size_t max_notify = (negotiated_mtu_ > 3) ? (negotiated_mtu_ - 3) : 20;
    if (json.length() > max_notify)
    {
        // Truncation would corrupt JSON — log warning and skip this update
        static uint32_t last_warn_ms = 0;
        if (millis() - last_warn_ms > 5000)
        {
            Serial.printf("[BLE] WARNING: telemetry JSON %u bytes exceeds MTU notify limit %u, skipping\n",
                          (unsigned)json.length(), (unsigned)max_notify);
            last_warn_ms = millis();
        }
        return;
    }

    // Send via BLE notification
    telemetry_characteristic_->setValue(json.c_str());
    telemetry_characteristic_->notify();

    // Debug: Print first telemetry send (then every 10 seconds)
    static uint32_t last_print_ms = 0;
    if (millis() - last_print_ms > 10000)
    {
        Serial.print("[BLE] Sent telemetry (");
        Serial.print(json.length());
        Serial.println(" bytes)");
        Serial.print("[BLE] JSON: ");
        Serial.println(json);
        last_print_ms = millis();
    }
}

uint8_t TR_BLE_To_APP::getCommand()
{
    portENTER_CRITICAL(&s_cmd_mux);
    uint8_t cmd = pending_command_;
    pending_command_ = 0; // Clear after reading
    portEXIT_CRITICAL(&s_cmd_mux);
    return cmd;
}

uint8_t TR_BLE_To_APP::getFileListPage()
{
    uint8_t page = pending_file_list_page_;
    pending_file_list_page_ = 0; // Clear after reading
    return page;
}

String TR_BLE_To_APP::getDeleteFilename()
{
    String filename = pending_delete_filename_;
    pending_delete_filename_ = ""; // Clear after reading
    return filename;
}

String TR_BLE_To_APP::getDownloadFilename()
{
    String filename = pending_download_filename_;
    pending_download_filename_ = ""; // Clear after reading
    return filename;
}

void TR_BLE_To_APP::sendConfigJSON(const String& json)
{
    if (!device_connected_ || !telemetry_characteristic_) return;
    telemetry_characteristic_->setValue(json.c_str());
    telemetry_characteristic_->notify();
}

void TR_BLE_To_APP::sendFileList(const String& files_json)
{
    if (!device_connected_)
    {
        return; // No device connected
    }

    if (!file_ops_characteristic_)
    {
        Serial.println("[BLE] ERROR: file_ops_characteristic_ is null!");
        return;
    }

    // Store JSON in member variable so it persists
    file_list_json_ = files_json;

    // Send file list as JSON with explicit length
    file_ops_characteristic_->setValue((uint8_t*)file_list_json_.c_str(), file_list_json_.length());
    file_ops_characteristic_->notify();

    static uint32_t file_list_send_count = 0;
    file_list_send_count++;
    Serial.print("[BLE] Sent file list (");
    Serial.print(file_list_json_.length());
    Serial.print(" bytes) #");
    Serial.println(file_list_send_count);
}

void TR_BLE_To_APP::sendFileChunk(uint32_t offset, const uint8_t* data, size_t len, bool eof)
{
    if (!device_connected_)
    {
        return;
    }

    if (!file_transfer_characteristic_)
    {
        Serial.println("[BLE] ERROR: file_transfer_characteristic_ is null!");
        return;
    }

    // Build chunk packet: [offset(4)][length(2)][flags(1)][data(N)]
    const size_t header_size = 7;  // 4 + 2 + 1
    const size_t packet_size = header_size + len;

    // Reallocate buffer if needed (persistent across calls)
    if (chunk_buffer_size_ < packet_size)
    {
        if (chunk_buffer_ != nullptr)
        {
            delete[] chunk_buffer_;
        }
        chunk_buffer_ = new uint8_t[packet_size];
        chunk_buffer_size_ = packet_size;
    }

    // Offset (4 bytes, little-endian)
    chunk_buffer_[0] = (offset >> 0) & 0xFF;
    chunk_buffer_[1] = (offset >> 8) & 0xFF;
    chunk_buffer_[2] = (offset >> 16) & 0xFF;
    chunk_buffer_[3] = (offset >> 24) & 0xFF;

    // Length (2 bytes, little-endian)
    chunk_buffer_[4] = (len >> 0) & 0xFF;
    chunk_buffer_[5] = (len >> 8) & 0xFF;

    // Flags (1 byte): bit 0 = EOF
    chunk_buffer_[6] = eof ? 0x01 : 0x00;

    // Data
    if (len > 0 && data != nullptr)
    {
        memcpy(chunk_buffer_ + header_size, data, len);
    }

    // Send via BLE notification.  Pacing is handled by the caller
    // (consistent per-chunk delay) to prevent overwhelming the iOS
    // BLE notification queue.
    file_transfer_characteristic_->setValue(chunk_buffer_, packet_size);
    file_transfer_characteristic_->notify();

    // Minimal debug: only log first and last chunks to avoid slowing down transfer
    if (offset == 0 || eof)
    {
        Serial.print("[BLE] Chunk offset=");
        Serial.print(offset);
        Serial.print(", len=");
        Serial.print(len);
        Serial.print(", eof=");
        Serial.println(eof ? "true" : "false");
    }
}

// ============================================================================
// Private Helpers
// ============================================================================

String TR_BLE_To_APP::buildTelemetryJSON(const TelemetryData& data)
{
    String json = "{";
    bool first = true;

    // Comma separator helper
    auto sep = [&]() { if (!first) json += ","; first = false; };

    // Optional float — skips NaN values entirely to save BLE payload bytes
    auto addFloat = [&](const char* key, float value, int decimals) {
        if (isnan(value)) return;
        sep(); json += "\""; json += key; json += "\":"; json += String(value, decimals);
    };

    // Optional double — skips NaN values
    auto addDouble = [&](const char* key, double value, int decimals) {
        if (isnan(value)) return;
        sep(); json += "\""; json += key; json += "\":"; json += String(value, decimals);
    };

    // Always-present helpers
    auto addInt = [&](const char* key, int value) {
        sep(); json += "\""; json += key; json += "\":"; json += String(value);
    };
    auto addUint = [&](const char* key, uint32_t value) {
        sep(); json += "\""; json += key; json += "\":"; json += String(value);
    };
    auto addString = [&](const char* key, const char* value) {
        sep(); json += "\""; json += key; json += "\":\""; json += (value ? value : ""); json += "\"";
    };
    auto addBool = [&](const char* key, bool value) {
        sep(); json += "\""; json += key; json += "\":"; json += value ? "true" : "false";
    };

    // Battery
    addFloat("soc", data.soc, 1);
    addFloat("cur", data.current, 1);
    addFloat("vol", data.voltage, 2);

    // GPS
    addDouble("lat", data.latitude, 7);
    addDouble("lon", data.longitude, 7);
    addFloat("gdop", data.gdop, 1);
    addInt("nsat", data.num_sats);

    // State
    addString("st", data.state);

    // Status flags
    addBool("cam", data.camera_recording);
    addBool("log", data.logging_active);
    addString("af", data.active_file);

    // Flight event flags
    addBool("lnch", data.launch_flag);
    addBool("vapo", data.vel_u_apogee_flag);
    addBool("aapo", data.alt_apogee_flag);
    addBool("land", data.alt_landed_flag);

    // Data rates (NaN on base station — will be omitted)
    addFloat("rxk", data.rx_kbs, 1);
    addFloat("wrk", data.wr_kbs, 1);

    // Frame stats
    addUint("frx", data.frames_rx);
    addUint("fdr", data.frames_drop);

    // Max values
    addFloat("malt", data.max_alt_m, 2);
    addFloat("mspd", data.max_speed_mps, 2);

    // Altitude
    addFloat("palt", data.pressure_alt, 1);
    addFloat("arate", data.altitude_rate, 1);
    addFloat("galt", data.gnss_alt, 1);

    // IMU - Low-G accelerometer (m/s²)
    addFloat("lx", data.low_g_x, 2);
    addFloat("ly", data.low_g_y, 2);
    addFloat("lz", data.low_g_z, 2);

    // IMU - High-G accelerometer (NaN on base station — will be omitted)
    addFloat("hx", data.high_g_x, 1);
    addFloat("hy", data.high_g_y, 1);
    addFloat("hz", data.high_g_z, 1);

    // IMU - Gyroscope (deg/s)
    addFloat("gx", data.gyro_x, 1);
    addFloat("gy", data.gyro_y, 1);
    addFloat("gz", data.gyro_z, 1);

    // Attitude (deg) and roll command (deg)
    addFloat("rol", data.roll, 1);
    addFloat("pit", data.pitch, 1);
    addFloat("yaw", data.yaw, 1);
    addFloat("rcmd", data.roll_cmd, 1);
    addFloat("q0", data.q0, 4);
    addFloat("q1", data.q1, 4);
    addFloat("q2", data.q2, 4);
    addFloat("q3", data.q3, 4);

    // LoRa signal quality (NaN on direct connection — will be omitted)
    addFloat("rssi", data.rssi, 0);
    addFloat("snr", data.snr, 1);

    // Base station (NaN/false on direct connection — will be omitted or default)
    addFloat("bsoc", data.bs_soc, 1);
    addFloat("bvol", data.bs_voltage, 2);
    addFloat("bcur", data.bs_current, 0);
    addBool("bslog", data.bs_logging_active);

    // Power rail state
    addBool("pwr", data.pwr_pin_on);

    json += "}";

    return json;
}
