// Radio daughterboard firmware (#409): a transparent UART<->LoRa modem.
//
// The host (OC on the rocket, BS on the ground — this firmware never knows
// which) sends fully-formed air frames over the UART link; we radiate them
// unmodified and tunnel received frames back with RSSI/SNR metadata. All
// protocol knowledge (LORA_PROTO_VERSION, network ids, hop schedules) stays
// on the hosts, so the on-air format is byte-identical to direct-SPI boards
// and deployed base stations interoperate unchanged.
//
// Protocol: components/TR_UART_Link/RadioModemProtocol.h (v1).
// Flow control: modem-side TX queue of TX_QUEUE_CAPACITY frames; every
// accepted TX_FRAME seq is eventually answered with a TX_RESULT (the credit
// return) — never silently dropped.

#include <cstring>

#include <esp_app_desc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <RadioModemProtocol.h>
#include <TR_LoRa_Comms.h>
#include <TR_UART_Link.h>

#include "config.h"

static const char* TAG = "radio_board";

using namespace radio_modem;

static TR_UART_Link uart_link;
static TR_LoRa_Comms radio;
static bool radio_up = false;

// ---- TX queue + in-flight tracking -----------------------------------------

struct TxEntry
{
    uint8_t seq;
    uint8_t len;
    uint8_t data[MAX_AIR_FRAME];
};

static TxEntry tx_queue[TX_QUEUE_CAPACITY];
static uint8_t txq_head = 0;  // next to send
static uint8_t txq_used = 0;

struct InFlight
{
    bool active = false;
    uint8_t seq = 0;
    uint32_t ok_before = 0;
};
static InFlight in_flight;

static void sendTxResult(uint8_t seq, bool ok)
{
    const TxResultData res = {seq, static_cast<uint8_t>(ok ? 1 : 0)};
    uart_link.sendFrame(MSG_TX_RESULT,
                        reinterpret_cast<const uint8_t*>(&res), sizeof(res));
}

// ---- Outbound status/identity ----------------------------------------------

static void sendIdentity(uint8_t msg_type)
{
    ModemIdentityData id = {};
    id.protocol_version = PROTOCOL_VERSION;
    id.chip = CHIP_LLCC68;
    id.max_tx_power_dbm = config::RADIO_MAX_TX_POWER_DBM;
    id.freq_min_mhz = config::RADIO_FREQ_MIN_MHZ;
    id.freq_max_mhz = config::RADIO_FREQ_MAX_MHZ;
    const esp_app_desc_t* app = esp_app_get_description();
    strncpy(id.fw_version, app->version, sizeof(id.fw_version) - 1);
    uart_link.sendFrame(msg_type, reinterpret_cast<const uint8_t*>(&id),
                        sizeof(id));
}

static void sendStatus()
{
    TR_LoRa_Comms::Stats rs = {};
    radio.getStats(rs);
    TR_UART_Link::Stats us = {};
    uart_link.getStats(us);

    ModemStatusData st = {};
    st.uptime_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    st.radio_enabled = radio_up ? 1 : 0;
    st.rx_mode = rs.rx_mode ? 1 : 0;
    st.tx_queue_used = txq_used + (in_flight.active ? 1 : 0);
    st.tx_queue_capacity = TX_QUEUE_CAPACITY;
    st.tx_ok = rs.tx_ok;
    st.tx_fail = rs.tx_fail;
    st.rx_count = rs.rx_count;
    st.rx_crc_fail = rs.rx_crc_fail;
    st.tx_watchdog_fires = rs.tx_watchdog_fires;
    st.uart_rx_crc_fails = us.rx_crc_fails;
    st.uart_rx_resync_bytes = us.rx_resync_bytes;
    st.last_rssi_dbm = rs.last_rssi;
    st.last_snr_db = rs.last_snr;
    st.current_freq_mhz = radio.currentFrequencyMHz();
    st.current_sf = radio.currentSpreadingFactor();
    uart_link.sendFrame(MSG_STATUS, reinterpret_cast<const uint8_t*>(&st),
                        sizeof(st));
}

// ---- Inbound message handling ------------------------------------------------

static TR_LoRa_Comms::Config radioConfigFromMsg(const RadioConfigData& d)
{
    TR_LoRa_Comms::Config cfg = {};
    cfg.enabled = true;
    cfg.cs_pin = config::LORA_CS_PIN;
    cfg.dio1_pin = config::LORA_DIO1_PIN;
    cfg.rst_pin = config::LORA_RST_PIN;
    cfg.busy_pin = config::LORA_BUSY_PIN;
    cfg.spi_sck = config::LORA_SPI_SCK;
    cfg.spi_miso = config::LORA_SPI_MISO;
    cfg.spi_mosi = config::LORA_SPI_MOSI;
    cfg.freq_mhz = d.freq_mhz;
    cfg.spreading_factor = d.spreading_factor;
    cfg.bandwidth_khz = d.bandwidth_khz;
    cfg.coding_rate = d.coding_rate;
    cfg.preamble_len = d.preamble_len;
    cfg.tx_power_dbm = d.tx_power_dbm;
    cfg.crc_on = (d.flags & CFG_FLAG_CRC_ON) != 0;
    cfg.rx_boosted_gain = (d.flags & CFG_FLAG_RX_BOOSTED_GAIN) != 0;
    cfg.syncword_private = (d.flags & CFG_FLAG_SYNCWORD_PRIVATE) != 0;
    return cfg;
}

static void handleTxFrame(const uint8_t* payload, size_t len)
{
    if (len < sizeof(TxFrameHeader) + 1)
    {
        return;  // no seq to answer — malformed, drop
    }
    const uint8_t seq = payload[0];
    const uint8_t* air = payload + sizeof(TxFrameHeader);
    const size_t air_len = len - sizeof(TxFrameHeader);

    if (air_len > MAX_AIR_FRAME || !radio_up)
    {
        sendTxResult(seq, false);
        return;
    }
    if (txq_used >= TX_QUEUE_CAPACITY)
    {
        // Host exceeded its credit window (or credits desynced) — reject so
        // the credit is returned rather than leaked.
        sendTxResult(seq, false);
        return;
    }

    TxEntry& e = tx_queue[(txq_head + txq_used) % TX_QUEUE_CAPACITY];
    e.seq = seq;
    e.len = static_cast<uint8_t>(air_len);
    memcpy(e.data, air, air_len);
    txq_used++;
}

static void handleSetConfig(const uint8_t* payload, size_t len)
{
    if (len != sizeof(RadioConfigData))
    {
        ESP_LOGW(TAG, "SET_CONFIG bad length %u", (unsigned)len);
        return;
    }
    RadioConfigData d;
    memcpy(&d, payload, sizeof(d));

    // Clamp to module capability — hosts should already respect IDENTITY,
    // but a matched-pair mismatch must degrade, not transmit out of spec.
    if (d.tx_power_dbm > config::RADIO_MAX_TX_POWER_DBM)
    {
        d.tx_power_dbm = config::RADIO_MAX_TX_POWER_DBM;
    }

    if (!radio_up)
    {
        // Radio never came up (or wasn't wired at boot) — full begin() so the
        // boot-fixed fields (preamble/flags) from the host apply too.
        radio_up = radio.begin(radioConfigFromMsg(d), config::DEBUG);
    }
    else
    {
        radio.reconfigure(d.freq_mhz, d.spreading_factor, d.bandwidth_khz,
                          d.coding_rate, d.tx_power_dbm);
    }
    if (radio_up && d.start_rx)
    {
        radio.startReceive();
    }
    sendStatus();  // config paths always get a status echo as the ack
}

static void onHostFrame(void* /*ctx*/, uint8_t type, const uint8_t* payload,
                        size_t len)
{
    switch (type)
    {
        case MSG_TX_FRAME:
            handleTxFrame(payload, len);
            break;

        case MSG_SET_CONFIG:
            handleSetConfig(payload, len);
            break;

        case MSG_HOP_FREQ:
            if (len == sizeof(HopFreqData) && radio_up)
            {
                HopFreqData d;
                memcpy(&d, payload, sizeof(d));
                // Fire-and-forget by design (per-packet hop latency path):
                // a mid-TX refusal keeps the current channel, matching the
                // direct driver's can't-retune-mid-TX contract.
                (void)radio.hopToFrequencyMHz(d.freq_mhz);
            }
            break;

        case MSG_START_RX:
            if (radio_up)
            {
                radio.startReceive();
            }
            break;

        case MSG_GET_STATUS:
            sendStatus();
            break;

        case MSG_GET_IDENTITY:
            sendIdentity(MSG_IDENTITY);
            break;

        case MSG_START_SCAN:
            if (len == sizeof(ScanRequestData) && radio_up)
            {
                ScanRequestData d;
                memcpy(&d, payload, sizeof(d));
                radio.startScan(d.start_mhz, d.stop_mhz, d.step_khz, d.dwell_ms);
            }
            break;

        default:
            ESP_LOGW(TAG, "unknown host msg type 0x%02X (%u B)", type,
                     (unsigned)len);
            break;
    }
}

// ---- Radio-side servicing ----------------------------------------------------

static void serviceTx()
{
    if (!radio_up)
    {
        return;
    }

    // Close out a completed transmission: service() has run finishTransmit
    // (or the TX watchdog force-cleared a wedge) once canSend() goes true.
    if (in_flight.active && radio.canSend())
    {
        TR_LoRa_Comms::Stats rs = {};
        radio.getStats(rs);
        sendTxResult(in_flight.seq, rs.tx_ok > in_flight.ok_before);
        in_flight.active = false;
    }

    if (!in_flight.active && txq_used > 0 && radio.canSend() &&
        !radio.isScanActive())
    {
        TxEntry& e = tx_queue[txq_head];
        TR_LoRa_Comms::Stats rs = {};
        radio.getStats(rs);
        if (radio.send(e.data, e.len))
        {
            in_flight.active = true;
            in_flight.seq = e.seq;
            in_flight.ok_before = rs.tx_ok;
        }
        else
        {
            sendTxResult(e.seq, false);
        }
        txq_head = (txq_head + 1) % TX_QUEUE_CAPACITY;
        txq_used--;
    }
}

static void serviceRx()
{
    if (!radio_up)
    {
        return;
    }
    // RX_FRAME payload = RxFrameHeader + air bytes, packed in one buffer.
    static uint8_t out[sizeof(RxFrameHeader) + MAX_AIR_FRAME];
    size_t air_len = 0;
    if (!radio.readPacket(out + sizeof(RxFrameHeader), MAX_AIR_FRAME, air_len))
    {
        return;
    }
    TR_LoRa_Comms::Stats rs = {};
    radio.getStats(rs);
    RxFrameHeader hdr = {rs.last_rssi, rs.last_snr};
    memcpy(out, &hdr, sizeof(hdr));
    uart_link.sendFrame(MSG_RX_FRAME, out, sizeof(hdr) + air_len);
}

static void serviceScanResult()
{
    if (!radio_up || !radio.isScanDone())
    {
        return;
    }

    const size_t n = radio.getScanSampleCount();
    const TR_LoRa_Comms::ScanSample* samples = radio.getScanSamples();

    static uint8_t out[sizeof(ScanResultHeader) + TR_LoRa_Comms::SCAN_MAX_SAMPLES];
    ScanResultHeader hdr = {};
    hdr.count = static_cast<uint8_t>(n);
    hdr.start_mhz = radio.getScanStartMHz();
    hdr.step_khz = radio.getScanStepKHz();
    memcpy(out, &hdr, sizeof(hdr));
    for (size_t i = 0; i < n; i++)
    {
        out[sizeof(hdr) + i] = static_cast<uint8_t>(samples[i].rssi_dbm);
    }
    uart_link.sendFrame(MSG_SCAN_RESULT, out, sizeof(hdr) + n);
    radio.consumeScanDone();
    radio.startReceive();  // resume listening after the scan sweep
}

// ---- Entry -------------------------------------------------------------------

extern "C" void app_main(void)
{
    const esp_app_desc_t* app = esp_app_get_description();
    ESP_LOGI(TAG, "radio_board fw %s (protocol v%u)", app->version,
             PROTOCOL_VERSION);

    TR_UART_Link::Config ucfg = {};
    ucfg.port = config::HOST_UART_PORT;
    ucfg.tx_pin = config::HOST_UART_TX;
    ucfg.rx_pin = config::HOST_UART_RX;
    ucfg.baud = config::HOST_UART_BAUD;
    ESP_ERROR_CHECK(uart_link.begin(ucfg));

    // Bring the radio up on boot defaults so the modem listens immediately;
    // the host's SET_CONFIG replaces these (and is also the recovery path if
    // the radio fails here — see handleSetConfig).
    RadioConfigData boot = {};
    boot.freq_mhz = config::BOOT_FREQ_MHZ;
    boot.bandwidth_khz = config::BOOT_BW_KHZ;
    boot.spreading_factor = config::BOOT_SF;
    boot.coding_rate = config::BOOT_CR;
    boot.tx_power_dbm = config::BOOT_TX_POWER_DBM;
    boot.preamble_len = 16;
    boot.flags = CFG_FLAG_CRC_ON | CFG_FLAG_RX_BOOSTED_GAIN |
                 CFG_FLAG_SYNCWORD_PRIVATE;
    radio_up = radio.begin(radioConfigFromMsg(boot), config::DEBUG);
    if (radio_up)
    {
        radio.startReceive();
        ESP_LOGI(TAG, "radio up, listening at %.1f MHz SF%u",
                 (double)config::BOOT_FREQ_MHZ, config::BOOT_SF);
    }
    else
    {
        ESP_LOGE(TAG, "radio init FAILED — modem alive, RF dead (host sees "
                      "radio_enabled=0)");
    }

    // Announce boot: identity payload doubles as the host's signal to reset
    // its credit window (our TX queue is empty now).
    sendIdentity(MSG_BOOT);

    for (;;)
    {
        uart_link.poll(onHostFrame, nullptr);

        if (radio_up)
        {
            radio.pollDio1();
            radio.service();
            radio.serviceTxWatchdog();
            if (radio.isScanActive())
            {
                radio.serviceScan();
            }
        }

        serviceTx();
        serviceRx();
        serviceScanResult();

        vTaskDelay(1);  // 1 ms at CONFIG_FREERTOS_HZ=1000
    }
}
