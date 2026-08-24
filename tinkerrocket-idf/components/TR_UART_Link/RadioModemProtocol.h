#pragma once

#include <stdint.h>

// UART radio-modem link protocol (#409): host (OC on the rocket, BS on the
// ground, #414) <-> LoRa daughterboard.
//
// The daughterboard is a TRANSPARENT MODEM: TX_FRAME / RX_FRAME payloads are
// the exact bytes the host would have handed to / read from TR_LoRa_Comms on
// a direct-SPI board. The modem never parses them, so the on-air format
// (LORA_PROTO_VERSION, network id, hopping schedule contents...) is owned
// entirely by the hosts and stays byte-identical to direct-radio boards.
// The same daughterboard hardware + firmware binary serves both ends of the
// link — nothing here may assume which end it is plugged into.
//
// Transport: tr_msg framing (TR_MsgCodec.h) over UART. The message types
// below live in a fresh, link-local namespace — they do NOT collide with
// I2C message-type codes or ble_cmd numbers. Keep every code for this link
// in this one enum pair so duplicates stay impossible to miss (see the
// wire-code collision history, #132/#148).
//
// Flow control (in-band credits): each TX_FRAME carries a one-byte seq; the
// modem returns it in a TX_RESULT when the airtime is spent (or the frame is
// rejected). The host may have at most TX_QUEUE_CAPACITY unacknowledged
// seqs outstanding. A MODEM BOOT message resets the credit window (modem
// rebooted; its queue is empty). A TX frame is therefore never silently
// dropped: every accepted seq is answered.
//
// Versioning: PROTOCOL_VERSION is carried in IDENTITY/BOOT. Hosts must
// check it and refuse (loudly — log + status surface) to run a mismatched
// pair rather than mis-parse.

namespace radio_modem
{

static constexpr uint8_t PROTOCOL_VERSION = 1;

// Modem-side TX queue depth == the host's credit window.
static constexpr uint8_t TX_QUEUE_CAPACITY = 8;

// Largest air frame the tunnel carries in either direction: one tr_msg
// payload (255) minus the larger per-frame header (RxFrameHeader, 8 bytes).
// Applies symmetrically so anything a host may transmit is also deliverable
// on the receive side. Generous vs. the ~62-byte frames the LoRa protocol
// actually uses; oversize TX_FRAMEs are rejected with TX_RESULT ok=0, and
// oversize air receptions (foreign traffic) are dropped and counted.
static constexpr size_t MAX_AIR_FRAME = 247;

// ---- Message types: host -> modem -----------------------------------------
enum HostToModem : uint8_t
{
    MSG_TX_FRAME     = 0x01,  // TxFrameHeader + opaque air bytes
    MSG_SET_CONFIG   = 0x02,  // RadioConfigData
    MSG_HOP_FREQ     = 0x03,  // HopFreqData (per-packet hopping; no reply)
    MSG_START_RX     = 0x04,  // (empty) enter RX mode
    MSG_GET_STATUS   = 0x05,  // (empty) -> MSG_STATUS
    MSG_GET_IDENTITY = 0x06,  // (empty) -> MSG_IDENTITY
    MSG_START_SCAN   = 0x07,  // ScanRequestData -> MSG_SCAN_RESULT when done
};

// ---- Message types: modem -> host ------------------------------------------
enum ModemToHost : uint8_t
{
    MSG_RX_FRAME    = 0x81,  // RxFrameHeader + opaque air bytes
    MSG_TX_RESULT   = 0x82,  // TxResultData (returns the seq = credit)
    MSG_STATUS      = 0x83,  // ModemStatusData
    MSG_IDENTITY    = 0x84,  // ModemIdentityData
    MSG_SCAN_RESULT = 0x85,  // ScanResultHeader + int8 RSSI samples
    MSG_BOOT        = 0x86,  // ModemIdentityData; sent once at modem boot —
                             // host must reset its credit window on receipt
};

// ---- Radio chip identifiers (IDENTITY.chip) ---------------------------------
enum RadioChip : uint8_t
{
    CHIP_UNKNOWN = 0,
    CHIP_LLCC68  = 1,
    // Future higher-power / alternate modules get new ids; hosts clamp
    // config to the reported capability fields, not the chip id.
};

// ---- Payload structs --------------------------------------------------------

struct __attribute__((packed)) TxFrameHeader
{
    uint8_t seq;  // echoed in TX_RESULT; host-chosen (wrapping counter)
};
static_assert(sizeof(TxFrameHeader) == 1, "TxFrameHeader must be 1 byte");

// Mirrors the radio-parameter fields of TR_LoRa_Comms::Config. Every field
// is authoritative on every SET_CONFIG: the modem applies freq/sf/bw/cr/power
// via reconfigure() and preamble/flags via applyFrameParams() (all are
// runtime-settable SX126x commands). Hosts may assume the STATUS ack means
// the full config — including preamble — is what is on the air; the airtime
// accounting in RocketComputerTypes.h depends on that.
struct __attribute__((packed)) RadioConfigData
{
    float freq_mhz;
    float bandwidth_khz;
    uint8_t spreading_factor;
    uint8_t coding_rate;
    int8_t tx_power_dbm;
    uint8_t flags;  // bit0 crc_on, bit1 rx_boosted_gain, bit2 syncword_private
    uint16_t preamble_len;
    uint8_t start_rx;  // nonzero: enter RX mode after applying
    uint8_t reserved;
};
static_assert(sizeof(RadioConfigData) == 16, "RadioConfigData must be 16 bytes");

static constexpr uint8_t CFG_FLAG_CRC_ON            = 1 << 0;
static constexpr uint8_t CFG_FLAG_RX_BOOSTED_GAIN   = 1 << 1;
static constexpr uint8_t CFG_FLAG_SYNCWORD_PRIVATE  = 1 << 2;

struct __attribute__((packed)) HopFreqData
{
    float freq_mhz;
};
static_assert(sizeof(HopFreqData) == 4, "HopFreqData must be 4 bytes");

struct __attribute__((packed)) RxFrameHeader
{
    float rssi_dbm;
    float snr_db;
};
static_assert(sizeof(RxFrameHeader) == 8, "RxFrameHeader must be 8 bytes");

struct __attribute__((packed)) TxResultData
{
    uint8_t seq;
    uint8_t ok;  // 1 = radiated, 0 = failed/rejected (queue full, radio down,
                 //     TX watchdog fired)
};
static_assert(sizeof(TxResultData) == 2, "TxResultData must be 2 bytes");

struct __attribute__((packed)) ModemIdentityData
{
    uint8_t protocol_version;  // PROTOCOL_VERSION
    uint8_t chip;              // RadioChip
    int8_t max_tx_power_dbm;   // capability ceiling — hosts clamp to this
    uint8_t reserved;
    float freq_min_mhz;        // capability band edges
    float freq_max_mhz;
    char fw_version[32];       // esp_app_desc version (git sha + build stamp),
                               // NUL-terminated / NUL-padded
};
static_assert(sizeof(ModemIdentityData) == 44, "ModemIdentityData must be 44 bytes");

// ModemStatusData::config_ok values.  0 stays UNKNOWN forever — it is what a
// pre-#835 modem image leaves in the byte it knew as `reserved[0]`.
enum CfgAck : uint8_t
{
    CFG_ACK_UNKNOWN  = 0,
    CFG_ACK_APPLIED  = 1,
    CFG_ACK_REJECTED = 2,
};

struct __attribute__((packed)) ModemStatusData
{
    uint32_t uptime_ms;
    uint8_t radio_enabled;    // 0: radio failed to init — modem alive, RF dead
    uint8_t rx_mode;
    uint8_t tx_queue_used;
    uint8_t tx_queue_capacity;  // == TX_QUEUE_CAPACITY (belt & braces)
    uint32_t tx_ok;
    uint32_t tx_fail;
    uint32_t rx_count;
    uint32_t rx_crc_fail;
    uint32_t tx_watchdog_fires;
    uint32_t uart_rx_crc_fails;    // host->modem link health
    uint32_t uart_rx_resync_bytes;
    float last_rssi_dbm;
    float last_snr_db;
    float current_freq_mhz;
    uint8_t current_sf;
    // #835 item 7: did the LAST SET_CONFIG actually take?  radio_enabled only
    // says the radio is alive, which it still is after a FAILED reconfigure()
    // — TR_LoRa_Comms rolls back to the previous modulation and stays up.  A
    // host that treats the STATUS ack as "config applied" then caches a
    // modulation that is not on the air and writes it to NVS.
    //
    // TRI-STATE, and 0 is deliberately NOT "rejected": this byte was
    // `reserved[0]` in earlier modem images, which zero-fill it.  Treating 0
    // as a rejection would make every config from an older daughterboard
    // fail.  Hosts fall back to comparing current_freq_mhz/current_sf when
    // they see UNKNOWN.  Non-config STATUS frames report the last
    // SET_CONFIG's result, so a periodic poll still tells the truth.
    uint8_t config_ok;   // CfgAck: 0 = unknown (legacy image), 1 = applied,
                         //         2 = rejected / rolled back
    uint8_t reserved[2];
};
static_assert(sizeof(ModemStatusData) == 52, "ModemStatusData must be 52 bytes");

struct __attribute__((packed)) ScanRequestData
{
    float start_mhz;
    float stop_mhz;
    uint16_t step_khz;
    uint16_t dwell_ms;
};
static_assert(sizeof(ScanRequestData) == 12, "ScanRequestData must be 12 bytes");

// SCAN_RESULT payload: this header followed by `count` int8 RSSI samples
// (dBm, clamped); sample i is at freq start_mhz + i * step_khz/1000.
// TR_LoRa_Comms::SCAN_MAX_SAMPLES = 128, so header + samples always fits a
// single tr_msg frame (12 + 128 <= 255) — no chunking needed.
struct __attribute__((packed)) ScanResultHeader
{
    uint8_t count;
    uint8_t reserved[3];
    float start_mhz;
    float step_khz;
};
static_assert(sizeof(ScanResultHeader) == 12, "ScanResultHeader must be 12 bytes");

}  // namespace radio_modem
