#include "TR_BLE_To_APP.h"

#include <cstring>
#include <cstdio>
#include <cmath>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <host/ble_hs.h>
#include <host/util/util.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>

static const char* BLE_TAG = "BLE";

// Spinlock protecting pending_command_ against races between the BLE
// callback (runs on the NimBLE host task) and the main loop reading it.
static portMUX_TYPE s_cmd_mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// Static instance pointer (needed for C-style NimBLE callbacks)
// ============================================================================

TR_BLE_To_APP* TR_BLE_To_APP::s_instance_ = nullptr;

// ============================================================================
// UUID helpers — convert "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx" to ble_uuid128_t
// ============================================================================

static ble_uuid128_t uuid128_from_string(const char* s)
{
    ble_uuid128_t u;
    u.u.type = BLE_UUID_TYPE_128;

    // Parse 32 hex chars from the UUID string (skip dashes) into 16 bytes
    // NimBLE stores 128-bit UUIDs in little-endian byte order.
    uint8_t buf[16];
    int idx = 0;
    for (int i = 0; s[i] && idx < 16; ++i)
    {
        if (s[i] == '-') continue;
        uint8_t nibble_hi = 0, nibble_lo = 0;
        // high nibble
        if      (s[i] >= '0' && s[i] <= '9') nibble_hi = s[i] - '0';
        else if (s[i] >= 'a' && s[i] <= 'f') nibble_hi = s[i] - 'a' + 10;
        else if (s[i] >= 'A' && s[i] <= 'F') nibble_hi = s[i] - 'A' + 10;
        ++i;
        if (!s[i]) break;
        // low nibble
        if      (s[i] >= '0' && s[i] <= '9') nibble_lo = s[i] - '0';
        else if (s[i] >= 'a' && s[i] <= 'f') nibble_lo = s[i] - 'a' + 10;
        else if (s[i] >= 'A' && s[i] <= 'F') nibble_lo = s[i] - 'A' + 10;
        buf[idx++] = (nibble_hi << 4) | nibble_lo;
    }

    // Reverse into little-endian
    for (int i = 0; i < 16; ++i)
    {
        u.value[i] = buf[15 - i];
    }
    return u;
}

// ============================================================================
// Pre-computed UUIDs (file-scope, initialised once)
// ============================================================================

static ble_uuid128_t s_svc_uuid;
static ble_uuid128_t s_telemetry_uuid;
static ble_uuid128_t s_command_uuid;
static ble_uuid128_t s_file_ops_uuid;
static ble_uuid128_t s_file_transfer_uuid;

static void init_uuids()
{
    static bool done = false;
    if (done) return;
    s_svc_uuid           = uuid128_from_string("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
    s_telemetry_uuid     = uuid128_from_string("beb5483e-36e1-4688-b7f5-ea07361b26a8");
    s_command_uuid       = uuid128_from_string("cba1d466-344c-4be3-ab3f-189f80dd7518");
    s_file_ops_uuid      = uuid128_from_string("8d53dc1d-1db7-4cd3-868b-8a527460aa84");
    s_file_transfer_uuid = uuid128_from_string("1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d");
    done = true;
}

// ============================================================================
// GATT service table — built as a static array for ble_gatts_count_cfg /
// ble_gatts_add_svcs.  NimBLE requires characteristic value handles be
// stored via a pointer at registration time; we point to the member
// variables of the singleton instance (s_instance_).
// ============================================================================

// Forward-declare access callback
static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt* ctxt, void* arg);

// We build the table dynamically in registerGattServices() because
// we need runtime-initialised UUID objects and handle pointers.

// ============================================================================
// Constructor
// ============================================================================

TR_BLE_To_APP::TR_BLE_To_APP(const char* device_name)
    : device_connected_(false),
      negotiated_mtu_(0),
      conn_handle_(0),
      pending_command_(0),
      pending_file_list_page_(0),
      pending_delete_filename_(""),
      pending_download_filename_(""),
      file_list_json_(""),
      chunk_buffer_(nullptr),
      chunk_buffer_size_(0),
      telemetry_val_handle_(0),
      command_val_handle_(0),
      file_ops_val_handle_(0),
      file_transfer_val_handle_(0)
{
    // Copy initial name into the mutable buffer
    strncpy(device_name_, device_name, MAX_DEVICE_NAME_LEN);
    device_name_[MAX_DEVICE_NAME_LEN] = '\0';
}

void TR_BLE_To_APP::setName(const char* name)
{
    strncpy(device_name_, name, MAX_DEVICE_NAME_LEN);
    device_name_[MAX_DEVICE_NAME_LEN] = '\0';

    // If BLE stack is already running, update GAP name and restart advertising
    // so scanners see the new name immediately.
    if (s_instance_ == this)
    {
        ble_svc_gap_device_name_set(device_name_);
        ble_gap_adv_stop();
        startAdvertising();
        ESP_LOGI(BLE_TAG, "BLE name changed to: %s", device_name_);
    }
}

// ============================================================================
// NimBLE host callbacks (static)
// ============================================================================

void TR_BLE_To_APP::on_ble_hs_reset(int reason)
{
    ESP_LOGW(BLE_TAG, "NimBLE host reset, reason=%d", reason);
}

void TR_BLE_To_APP::on_ble_hs_sync()
{
    ESP_LOGI(BLE_TAG, "NimBLE host synced");

    // Make sure we have proper identity address set
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(BLE_TAG, "ble_hs_util_ensure_addr failed, rc=%d", rc);
        return;
    }

    if (s_instance_)
    {
        s_instance_->startAdvertising();
    }
}

// ============================================================================
// GAP event handler
// ============================================================================

int TR_BLE_To_APP::gap_event_cb(struct ble_gap_event* event, void* arg)
{
    TR_BLE_To_APP* self = static_cast<TR_BLE_To_APP*>(arg);

    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0)
        {
            struct ble_gap_conn_desc desc;
            int rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc == 0)
            {
                self->onConnect(event->connect.conn_handle, &desc);
            }
        }
        else
        {
            ESP_LOGW(BLE_TAG, "Connection failed, status=%d", event->connect.status);
            self->startAdvertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        self->onDisconnect(event->disconnect.conn.conn_handle,
                           event->disconnect.reason);
        break;

    case BLE_GAP_EVENT_MTU:
        self->onMtuChanged(event->mtu.conn_handle, event->mtu.value);
        break;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(BLE_TAG, "Connection parameters updated, status=%d",
                 event->conn_update.status);
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        // Advertising ended (e.g. timeout); restart it
        self->startAdvertising();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(BLE_TAG, "Subscribe event: attr_handle=%u, cur_notify=%d",
                 event->subscribe.attr_handle, event->subscribe.cur_notify);
        break;

    default:
        break;
    }

    return 0;
}

// ============================================================================
// GAP event helpers
// ============================================================================

void TR_BLE_To_APP::onConnect(uint16_t conn_handle,
                               const struct ble_gap_conn_desc* desc)
{
    device_connected_ = true;
    conn_handle_ = conn_handle;
    negotiated_mtu_ = 0;  // Reset until MTU exchange completes

    ESP_LOGI(BLE_TAG, "Device connected! handle=%u", conn_handle);

    // Request fast connection parameters from iOS for high-throughput transfer.
    struct ble_gap_upd_params params = {};
    params.itvl_min = 0x06;              // 7.5ms  (units of 1.25ms)
    params.itvl_max = 0x10;              // 20ms   (units of 1.25ms)
    params.latency = 0;                  // No slave latency
    params.supervision_timeout = 200;    // 2 seconds (units of 10ms)
    params.min_ce_len = 0;
    params.max_ce_len = 0;

    int rc = ble_gap_update_params(conn_handle, &params);
    if (rc == 0)
    {
        ESP_LOGI(BLE_TAG, "Requested fast connection parameters (7.5-20ms interval)");
    }
    else
    {
        ESP_LOGW(BLE_TAG, "Connection param update request failed, rc=%d", rc);
    }
}

void TR_BLE_To_APP::onDisconnect(uint16_t conn_handle, int reason)
{
    device_connected_ = false;
    negotiated_mtu_ = 0;
    conn_handle_ = 0;
    ESP_LOGW(BLE_TAG, "Device DISCONNECTED, reason=%d", reason);

    // Restart advertising so new devices can connect
    startAdvertising();
    ESP_LOGI(BLE_TAG, "Advertising restarted");
}

void TR_BLE_To_APP::onMtuChanged(uint16_t conn_handle, uint16_t mtu)
{
    negotiated_mtu_ = mtu;
    ESP_LOGI(BLE_TAG, "MTU negotiated: %u -> max chunk data: %u bytes",
             mtu, (unsigned)getMaxChunkDataSize());
}

// ============================================================================
// GATT access callback
// ============================================================================

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt* ctxt, void* arg)
{
    return TR_BLE_To_APP::gatt_svc_access_cb(conn_handle, attr_handle, ctxt, arg);
}

int TR_BLE_To_APP::gatt_svc_access_cb(uint16_t conn_handle,
                                       uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt* ctxt,
                                       void* arg)
{
    TR_BLE_To_APP* self = s_instance_;
    if (!self) return BLE_ATT_ERR_UNLIKELY;

    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
    {
        // Only the command characteristic accepts writes
        if (attr_handle == self->command_val_handle_)
        {
            // Flatten the mbuf chain into a contiguous buffer
            uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
            if (om_len == 0) return 0;

            uint8_t buf[256];
            uint16_t copy_len = (om_len < sizeof(buf)) ? om_len : sizeof(buf);
            uint16_t out_len = 0;
            int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, copy_len, &out_len);
            if (rc != 0) return BLE_ATT_ERR_UNLIKELY;

            self->onCommandWrite(buf, out_len);
        }
        return 0;
    }

    case BLE_GATT_ACCESS_OP_READ_CHR:
        // For read-only characteristics (telemetry, file_ops, file_transfer):
        // the value is delivered via notifications, reads return empty.
        return 0;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

// ============================================================================
// Command write handler (mirrors old CommandCallbacks::onWrite)
// ============================================================================

void TR_BLE_To_APP::onCommandWrite(const uint8_t* data, size_t length)
{
    if (length == 0 || data == nullptr) return;

    uint8_t cmd = data[0];

    if (cmd == 2 && length > 1)
    {
        // Command 2: File list request (optional page number follows)
        pending_file_list_page_ = data[1];
        ESP_LOGI(BLE_TAG, "File list request, page: %u", pending_file_list_page_);
    }
    else if (cmd == 2)
    {
        // Command 2 without page number defaults to page 0
        pending_file_list_page_ = 0;
    }
    else if (cmd == 3 && length > 1)
    {
        // Command 3: Delete file (filename follows command byte)
        pending_delete_filename_ = "";
        for (size_t i = 1; i < length; ++i)
        {
            pending_delete_filename_ += (char)data[i];
        }
        ESP_LOGI(BLE_TAG, "Delete file request: %s", pending_delete_filename_.c_str());
    }
    else if (cmd == 4 && length > 1)
    {
        // Command 4: Download file (filename follows command byte)
        pending_download_filename_ = "";
        for (size_t i = 1; i < length; ++i)
        {
            pending_download_filename_ += (char)data[i];
        }
        ESP_LOGI(BLE_TAG, "Download file request: %s", pending_download_filename_.c_str());
    }
    else if (cmd == 5 && length >= 13)
    {
        // Command 5: Sim config [cmd][mass_g:4][thrust_n:4][burn_s:4]
        size_t payload_len = length - 1;
        if (payload_len > sizeof(pending_payload_))
        {
            payload_len = sizeof(pending_payload_);
        }
        memcpy(pending_payload_, data + 1, payload_len);
        pending_payload_len_ = payload_len;
        ESP_LOGI(BLE_TAG, "Sim config received");
    }
    else if (cmd == 6)
    {
        // Command 6: Start simulation
        ESP_LOGI(BLE_TAG, "Sim start");
    }
    else if (cmd == 7)
    {
        // Command 7: Stop simulation
        ESP_LOGI(BLE_TAG, "Sim stop");
    }
    else if (cmd == 8)
    {
        // Command 8: Toggle power rail (PWR_PIN)
        ESP_LOGI(BLE_TAG, "Power toggle");
    }
    else if (cmd == 9 && length >= 8)
    {
        // Command 9: Time sync [cmd][year_lo][year_hi][month][day][hour][minute][second]
        size_t payload_len = length - 1;
        if (payload_len > sizeof(pending_payload_))
        {
            payload_len = sizeof(pending_payload_);
        }
        memcpy(pending_payload_, data + 1, payload_len);
        pending_payload_len_ = payload_len;
        ESP_LOGI(BLE_TAG, "Time sync received");
    }
    else if (length > 1)
    {
        // Generic payload handler for any other command carrying data
        size_t payload_len = length - 1;
        if (payload_len > sizeof(pending_payload_))
        {
            payload_len = sizeof(pending_payload_);
        }
        memcpy(pending_payload_, data + 1, payload_len);
        pending_payload_len_ = payload_len;
    }

    portENTER_CRITICAL(&s_cmd_mux);
    if (pending_command_ != 0)
    {
        // Previous command not yet consumed — log and overwrite
        portEXIT_CRITICAL(&s_cmd_mux);
        ESP_LOGW(BLE_TAG, "Overwriting unconsumed cmd %u with %u",
                 pending_command_, cmd);
        portENTER_CRITICAL(&s_cmd_mux);
    }
    pending_command_ = cmd;
    portEXIT_CRITICAL(&s_cmd_mux);

    ESP_LOGI(BLE_TAG, "Received command: %u", cmd);
}

// ============================================================================
// GATT service registration
// ============================================================================

// We must keep the GATT service definition array alive for the lifetime of
// the BLE host, so we make it file-static.
static struct ble_gatt_svc_def s_gatt_svcs[2];   // 1 service + terminator
static struct ble_gatt_chr_def s_gatt_chrs[5];    // 4 chars + terminator

void TR_BLE_To_APP::registerGattServices()
{
    init_uuids();

    // ---------- Characteristics ----------

    // 0: Telemetry (READ + NOTIFY)
    memset(&s_gatt_chrs[0], 0, sizeof(s_gatt_chrs[0]));
    s_gatt_chrs[0].uuid       = &s_telemetry_uuid.u;
    s_gatt_chrs[0].access_cb  = gatt_access_cb;
    s_gatt_chrs[0].flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY;
    s_gatt_chrs[0].val_handle = &telemetry_val_handle_;

    // 1: Command (WRITE)
    memset(&s_gatt_chrs[1], 0, sizeof(s_gatt_chrs[1]));
    s_gatt_chrs[1].uuid       = &s_command_uuid.u;
    s_gatt_chrs[1].access_cb  = gatt_access_cb;
    s_gatt_chrs[1].flags      = BLE_GATT_CHR_F_WRITE;
    s_gatt_chrs[1].val_handle = &command_val_handle_;

    // 2: File Operations (READ + NOTIFY)
    memset(&s_gatt_chrs[2], 0, sizeof(s_gatt_chrs[2]));
    s_gatt_chrs[2].uuid       = &s_file_ops_uuid.u;
    s_gatt_chrs[2].access_cb  = gatt_access_cb;
    s_gatt_chrs[2].flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY;
    s_gatt_chrs[2].val_handle = &file_ops_val_handle_;

    // 3: File Transfer (READ + NOTIFY)
    memset(&s_gatt_chrs[3], 0, sizeof(s_gatt_chrs[3]));
    s_gatt_chrs[3].uuid       = &s_file_transfer_uuid.u;
    s_gatt_chrs[3].access_cb  = gatt_access_cb;
    s_gatt_chrs[3].flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY;
    s_gatt_chrs[3].val_handle = &file_transfer_val_handle_;

    // 4: Terminator
    memset(&s_gatt_chrs[4], 0, sizeof(s_gatt_chrs[4]));

    // ---------- Service ----------
    memset(&s_gatt_svcs[0], 0, sizeof(s_gatt_svcs[0]));
    s_gatt_svcs[0].type            = BLE_GATT_SVC_TYPE_PRIMARY;
    s_gatt_svcs[0].uuid            = &s_svc_uuid.u;
    s_gatt_svcs[0].characteristics = s_gatt_chrs;

    // Terminator
    memset(&s_gatt_svcs[1], 0, sizeof(s_gatt_svcs[1]));

    int rc;

    rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(BLE_TAG, "ble_gatts_count_cfg failed, rc=%d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(BLE_TAG, "ble_gatts_add_svcs failed, rc=%d", rc);
        return;
    }

    ESP_LOGI(BLE_TAG, "GATT services registered");
}

// ============================================================================
// Advertising
// ============================================================================

void TR_BLE_To_APP::startAdvertising()
{
    struct ble_gap_adv_params adv_params = {};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;  // Undirected connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;  // General discoverable

    // Advertising interval: 1000 ms (in 0.625 ms units = 1600)
    adv_params.itvl_min = 0x0640;   // 1000 ms
    adv_params.itvl_max = 0x0640;   // 1000 ms

    // Build advertising data: flags + complete 128-bit service UUID
    struct ble_hs_adv_fields fields = {};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    init_uuids();
    ble_uuid128_t svc_uuid = s_svc_uuid;
    fields.uuids128 = &svc_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGW(BLE_TAG, "ble_gap_adv_set_fields failed, rc=%d", rc);
    }

    // Scan response: device name + min connection interval hint
    struct ble_hs_adv_fields rsp_fields = {};
    rsp_fields.name = (const uint8_t*)device_name_;
    rsp_fields.name_len = strlen(device_name_);
    rsp_fields.name_is_complete = 1;

    // Slave connection interval range hint (7.5 ms min)
    // The range is encoded as two uint16 values: min and max interval
    // in 1.25ms units.  We use the adv field for it.
    rsp_fields.slave_itvl_range = nullptr; // rely on conn param update instead

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0)
    {
        ESP_LOGW(BLE_TAG, "ble_gap_adv_rsp_set_fields failed, rc=%d", rc);
    }

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, nullptr, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, this);
    if (rc != 0 && rc != BLE_HS_EALREADY)
    {
        ESP_LOGE(BLE_TAG, "ble_gap_adv_start failed, rc=%d", rc);
    }
    else
    {
        ESP_LOGI(BLE_TAG, "Advertising started (1000ms interval)");
    }
}

// ============================================================================
// NimBLE host task (runs on its own FreeRTOS task)
// ============================================================================

static void nimble_host_task(void* param)
{
    ESP_LOGI("BLE", "NimBLE host task started");
    nimble_port_run();          // Blocks until nimble_port_stop()
    nimble_port_freertos_deinit();
}

// ============================================================================
// Public API
// ============================================================================

bool TR_BLE_To_APP::begin()
{
    ESP_LOGI(BLE_TAG, "Initializing BLE server (NimBLE)...");

    s_instance_ = this;

    // Initialize the NimBLE host stack
    int rc = nimble_port_init();
    if (rc != ESP_OK)
    {
        ESP_LOGE(BLE_TAG, "nimble_port_init failed, rc=%d", rc);
        return false;
    }

    // Set the device name for GAP
    ble_svc_gap_device_name_set(device_name_);

    // Request larger MTU (default is 23, max is 517)
    rc = ble_att_set_preferred_mtu(512);
    if (rc != 0)
    {
        ESP_LOGW(BLE_TAG, "ble_att_set_preferred_mtu failed, rc=%d", rc);
    }
    ESP_LOGI(BLE_TAG, "Requested MTU: 512 bytes");

    // Register mandatory GAP and GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Register our custom GATT services
    registerGattServices();

    // Configure host callbacks
    ble_hs_cfg.reset_cb = on_ble_hs_reset;
    ble_hs_cfg.sync_cb  = on_ble_hs_sync;

    // Start the NimBLE host task
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(BLE_TAG, "Server started (NimBLE)");
    ESP_LOGI(BLE_TAG, "Device name: %s", device_name_);

    return true;
}

void TR_BLE_To_APP::loop()
{
    // BLE stack handles events via callbacks on the NimBLE host task.
    // Nothing to do here.
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

// ============================================================================
// Helper: send a BLE notification on a given characteristic handle
// ============================================================================

static int notify_data(uint16_t conn_handle, uint16_t attr_handle,
                       const uint8_t* data, size_t len)
{
    // Keep retries short to avoid stalling the main loop.  The original
    // 20 × 25 ms (500 ms worst-case) caused 50-60 ms gaps in INA230 power
    // readings and other time-critical polling.  3 × 5 ms (15 ms max) is
    // enough for transient BLE controller congestion; sustained congestion
    // simply drops the notification — telemetry is best-effort anyway.
    static constexpr int MAX_RETRIES = 3;
    static constexpr int RETRY_DELAY_MS = 5;

    for (int attempt = 0; attempt < MAX_RETRIES; attempt++)
    {
        struct os_mbuf* om = ble_hs_mbuf_from_flat(data, len);
        if (!om)
        {
            // Out of mbufs — wait for BLE controller to drain and retry
            if (attempt < MAX_RETRIES - 1)
            {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            ESP_LOGE("BLE", "ble_hs_mbuf_from_flat failed after %d retries (len=%u)",
                     MAX_RETRIES, (unsigned)len);
            return BLE_HS_ENOMEM;
        }

        int rc = ble_gatts_notify_custom(conn_handle, attr_handle, om);
        if (rc == BLE_HS_ENOMEM)
        {
            // Controller queue full — wait and retry
            if (attempt < MAX_RETRIES - 1)
            {
                vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
                continue;
            }
            ESP_LOGE("BLE", "ble_gatts_notify_custom failed after %d retries, rc=%d",
                     MAX_RETRIES, rc);
        }
        if (rc != 0 && rc != BLE_HS_ENOMEM)
        {
            ESP_LOGW("BLE", "notify_data failed, rc=%d", rc);
        }
        if (attempt > 0 && rc == 0)
        {
            ESP_LOGI("BLE", "notify_data succeeded after %d retries", attempt);
        }
        return rc;
    }
    return BLE_HS_ENOMEM;
}

// ============================================================================
// sendTelemetry
// ============================================================================

void TR_BLE_To_APP::sendTelemetry(const TelemetryData& data)
{
    static bool first_call = true;
    if (first_call)
    {
        ESP_LOGI(BLE_TAG, "sendTelemetry called, device_connected_=%s",
                 device_connected_ ? "true" : "false");
        first_call = false;
    }

    if (!device_connected_) return;

    // Build JSON string
    String json = buildTelemetryJSON(data);

    // Check if JSON exceeds MTU — truncated notifications produce unparseable JSON
    size_t max_notify = (negotiated_mtu_ > 3) ? (negotiated_mtu_ - 3) : 20;
    if (json.length() > max_notify)
    {
        static uint32_t last_warn_ms = 0;
        if (millis() - last_warn_ms > 5000)
        {
            ESP_LOGW(BLE_TAG, "Telemetry JSON %u bytes > MTU limit %u (mtu=%u), SKIPPING",
                     (unsigned)json.length(), (unsigned)max_notify,
                     (unsigned)negotiated_mtu_);
            last_warn_ms = millis();
        }
        return;
    }

    // Send via BLE notification
    int rc = notify_data(conn_handle_, telemetry_val_handle_,
                         (const uint8_t*)json.c_str(), json.length());
    if (rc != 0)
    {
        ESP_LOGW(BLE_TAG, "Telemetry notify failed, rc=%d", rc);
        return;
    }

    // Debug: Print first telemetry send (then every 10 seconds)
    static uint32_t last_print_ms = 0;
    if (millis() - last_print_ms > 10000)
    {
        ESP_LOGI(BLE_TAG, "Sent telemetry (%u bytes)", (unsigned)json.length());
        ESP_LOGI(BLE_TAG, "JSON: %s", json.c_str());
        last_print_ms = millis();
    }
}

uint8_t TR_BLE_To_APP::getCommand()
{
    portENTER_CRITICAL(&s_cmd_mux);
    uint8_t cmd = pending_command_;
    pending_command_ = 0;
    portEXIT_CRITICAL(&s_cmd_mux);
    return cmd;
}

uint8_t TR_BLE_To_APP::getFileListPage()
{
    uint8_t page = pending_file_list_page_;
    pending_file_list_page_ = 0;
    return page;
}

String TR_BLE_To_APP::getDeleteFilename()
{
    String filename = pending_delete_filename_;
    pending_delete_filename_ = "";
    return filename;
}

String TR_BLE_To_APP::getDownloadFilename()
{
    String filename = pending_download_filename_;
    pending_download_filename_ = "";
    return filename;
}

void TR_BLE_To_APP::sendConfigJSON(const String& json)
{
    if (!device_connected_) return;

    // Guard against MTU truncation
    size_t max_notify = (negotiated_mtu_ > 3) ? (negotiated_mtu_ - 3) : 20;
    if (json.length() > max_notify)
    {
        ESP_LOGW(BLE_TAG, "Config JSON %u bytes > MTU limit %u, SKIPPING",
                 (unsigned)json.length(), (unsigned)max_notify);
        return;
    }

    int rc = notify_data(conn_handle_, telemetry_val_handle_,
                         (const uint8_t*)json.c_str(), json.length());
    if (rc != 0)
    {
        ESP_LOGW(BLE_TAG, "Config notify failed, rc=%d", rc);
    }
}

void TR_BLE_To_APP::sendFileList(const String& files_json)
{
    if (!device_connected_) return;

    // Store JSON in member variable so it persists
    file_list_json_ = files_json;

    int rc = notify_data(conn_handle_, file_ops_val_handle_,
                         (const uint8_t*)file_list_json_.c_str(),
                         file_list_json_.length());
    if (rc != 0)
    {
        ESP_LOGW(BLE_TAG, "File list notify failed, rc=%d", rc);
        return;
    }

    static uint32_t file_list_send_count = 0;
    file_list_send_count++;
    ESP_LOGI(BLE_TAG, "Sent file list (%u bytes) #%lu",
             (unsigned)file_list_json_.length(), (unsigned long)file_list_send_count);
}

void TR_BLE_To_APP::sendFileChunk(uint32_t offset, const uint8_t* data,
                                   size_t len, bool eof)
{
    if (!device_connected_) return;

    // Build chunk packet: [offset(4)][length(2)][flags(1)][data(N)]
    const size_t header_size = 7;
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

    // Send via BLE notification
    int rc = notify_data(conn_handle_, file_transfer_val_handle_,
                         chunk_buffer_, packet_size);
    if (rc != 0)
    {
        ESP_LOGW(BLE_TAG, "File chunk notify failed, rc=%d", rc);
    }

    // Minimal debug: only log first and last chunks
    if (offset == 0 || eof)
    {
        ESP_LOGI(BLE_TAG, "Chunk offset=%lu, len=%u, eof=%s",
                 (unsigned long)offset, (unsigned)len, eof ? "true" : "false");
    }
}

// ============================================================================
// Private Helpers
// ============================================================================

String TR_BLE_To_APP::buildTelemetryJSON(const TelemetryData& data)
{
    // Use a pre-allocated char buffer to build JSON efficiently.
    // Max telemetry JSON is well under 512 bytes.
    char buf[512];
    size_t pos = 0;
    bool first = true;

    buf[pos++] = '{';

    // Comma separator helper
    auto sep = [&]() { if (!first) buf[pos++] = ','; first = false; };

    // Append a key (quoted)
    auto appendKey = [&](const char* key) {
        sep();
        buf[pos++] = '"';
        size_t klen = strlen(key);
        memcpy(buf + pos, key, klen);
        pos += klen;
        buf[pos++] = '"';
        buf[pos++] = ':';
    };

    // Optional float — skips NaN values entirely to save BLE payload bytes
    auto addFloat = [&](const char* key, float value, int decimals) {
        if (std::isnan(value)) return;
        appendKey(key);
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%.*f", decimals, (double)value);
    };

    // Optional double — skips NaN values
    auto addDouble = [&](const char* key, double value, int decimals) {
        if (std::isnan(value)) return;
        appendKey(key);
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%.*f", decimals, value);
    };

    // Always-present helpers
    auto addInt = [&](const char* key, int value) {
        appendKey(key);
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%d", value);
    };
    auto addUint = [&](const char* key, uint32_t value) {
        appendKey(key);
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%lu", (unsigned long)value);
    };
    auto addString = [&](const char* key, const char* value) {
        appendKey(key);
        buf[pos++] = '"';
        if (value) {
            size_t vlen = strlen(value);
            memcpy(buf + pos, value, vlen);
            pos += vlen;
        }
        buf[pos++] = '"';
    };
    auto addBool = [&](const char* key, bool value) {
        appendKey(key);
        const char* s = value ? "true" : "false";
        size_t slen = strlen(s);
        memcpy(buf + pos, s, slen);
        pos += slen;
    };

    // Battery
    addFloat("soc", data.soc, 1);
    addFloat("cur", data.current, 1);
    addFloat("vol", data.voltage, 2);

    // GPS
    addDouble("lat", data.latitude, 7);
    addDouble("lon", data.longitude, 7);
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

    // Data rates
    addFloat("rxk", data.rx_kbs, 1);
    addFloat("wrk", data.wr_kbs, 1);
    addUint("frx", data.frames_rx);
    addUint("fdr", data.frames_drop);

    // Max values
    addFloat("malt", data.max_alt_m, 1);
    addFloat("mspd", data.max_speed_mps, 1);

    // Altitude
    addFloat("palt", data.pressure_alt, 1);
    addFloat("arate", data.altitude_rate, 1);
    addFloat("galt", data.gnss_alt, 1);

    // IMU - Low-G accelerometer (m/s^2)
    addFloat("lx", data.low_g_x, 1);
    addFloat("ly", data.low_g_y, 1);
    addFloat("lz", data.low_g_z, 1);

    // IMU - High-G accelerometer (NaN on base station — will be omitted)
    addFloat("hx", data.high_g_x, 1);
    addFloat("hy", data.high_g_y, 1);
    addFloat("hz", data.high_g_z, 1);

    // IMU - Gyroscope (deg/s)
    addFloat("gx", data.gyro_x, 1);
    addFloat("gy", data.gyro_y, 1);
    addFloat("gz", data.gyro_z, 1);

    // Roll command + quaternion
    addFloat("rcmd", data.roll_cmd, 1);
    addFloat("q0", data.q0, 4);
    addFloat("q1", data.q1, 4);
    addFloat("q2", data.q2, 4);
    addFloat("q3", data.q3, 4);

    // LoRa signal quality (NaN on direct connection — will be omitted)
    addFloat("rssi", data.rssi, 0);
    addFloat("snr", data.snr, 1);

    // Base station
    addFloat("bsoc", data.bs_soc, 1);
    addFloat("bvol", data.bs_voltage, 2);
    addFloat("bcur", data.bs_current, 0);
    addBool("bslog", data.bs_logging_active);

    // Power rail state
    addBool("pwr", data.pwr_pin_on);

    // Pyro channel status — packed into single byte to minimize JSON size
    {
        uint8_t ps = (data.pyro1_armed ? 0x01 : 0)
                   | (data.pyro1_cont  ? 0x02 : 0)
                   | (data.pyro1_fired ? 0x04 : 0)
                   | (data.pyro2_armed ? 0x08 : 0)
                   | (data.pyro2_cont  ? 0x10 : 0)
                   | (data.pyro2_fired ? 0x20 : 0);
        if (ps != 0) { addInt("ps", ps); }
    }

    // Source rocket identity (base station relay only)
    if (data.source_rocket_id > 0) {
        addInt("rid", data.source_rocket_id);
        if (data.source_unit_name && data.source_unit_name[0]) {
            addString("run", data.source_unit_name);
        }
    }

    buf[pos++] = '}';
    buf[pos] = '\0';

    return String(buf);
}
