#include "UartModemBackend.h"

#include <cstring>

#include <compat.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "RADIO_MODEM";

using namespace radio_modem;

// ---------------------------------------------------------------------------
// begin / config push
// ---------------------------------------------------------------------------

bool UartModemBackend::begin(const Config& cfg, float freq_mhz, uint8_t sf,
                             float bw_khz, uint8_t cr, int8_t tx_power,
                             bool debug)
{
    cfg_ = cfg;
    debug_ = debug;

    if (cfg.act_pin >= 0)
    {
        gpio_set_direction((gpio_num_t)cfg.act_pin, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)cfg.act_pin, 1);  // power the daughterboard
    }

    if (link_.begin(cfg.uart) != ESP_OK)
    {
        ESP_LOGE(TAG, "UART link init failed");
        return false;
    }
    began_ = true;

    // The modem sends BOOT (identity payload) once its firmware is up —
    // S3 boot is ~1 s from the ACT edge; wait generously. If we attached to
    // an already-running modem (warm OC reboot), there is no BOOT coming, so
    // also poke GET_IDENTITY and accept either reply.
    const uint32_t deadline = millis() + 4000;
    (void)link_.sendFrame(MSG_GET_IDENTITY, nullptr, 0);
    while (millis() < deadline && !modem_alive_)
    {
        link_.poll(&UartModemBackend::onFrameTrampoline, this);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!modem_alive_)
    {
        ESP_LOGE(TAG, "no modem answer on the UART link — radio disabled");
        return false;
    }
    if (identity_.protocol_version != PROTOCOL_VERSION)
    {
        // Loud and fatal-for-the-radio: a mismatched matched-pair must not
        // limp along mis-parsing frames.
        ESP_LOGE(TAG, "modem protocol v%u != host v%u — radio disabled "
                      "(mismatched daughterboard pair?)",
                 identity_.protocol_version, PROTOCOL_VERSION);
        modem_alive_ = false;
        return false;
    }
    ESP_LOGI(TAG, "modem up: chip=%u fw=%.32s max_tx=%ddBm band=%.0f-%.0fMHz",
             identity_.chip, identity_.fw_version,
             (int)identity_.max_tx_power_dbm,
             (double)identity_.freq_min_mhz, (double)identity_.freq_max_mhz);

    if (!pushConfig(freq_mhz, sf, bw_khz, cr, tx_power, /*start_rx=*/true,
                    /*ack_timeout_ms=*/500))
    {
        ESP_LOGE(TAG, "initial SET_CONFIG not acknowledged — radio disabled");
        modem_alive_ = false;
        return false;
    }

    stats_.enabled = true;
    ESP_LOGI(TAG, "radio configured via modem: %.1f MHz SF%u BW%.0f CR%u %ddBm",
             (double)freq_mhz, sf, (double)bw_khz, cr, (int)tx_power);
    return true;
}

bool UartModemBackend::pushConfig(float freq_mhz, uint8_t sf, float bw_khz,
                                  uint8_t cr, int8_t tx_power, bool start_rx,
                                  uint32_t ack_timeout_ms)
{
    RadioConfigData d = {};
    d.freq_mhz = freq_mhz;
    d.bandwidth_khz = bw_khz;
    d.spreading_factor = sf;
    d.coding_rate = cr;
    // Clamp host-side too (modem clamps as well — belt & braces for a
    // higher-power pair on one end only).
    d.tx_power_dbm = (identity_.max_tx_power_dbm != 0 &&
                      tx_power > identity_.max_tx_power_dbm)
                         ? identity_.max_tx_power_dbm
                         : tx_power;
    d.preamble_len = cfg_.preamble_len;
    d.flags = (cfg_.crc_on ? CFG_FLAG_CRC_ON : 0) |
              (cfg_.rx_boosted_gain ? CFG_FLAG_RX_BOOSTED_GAIN : 0) |
              (cfg_.syncword_private ? CFG_FLAG_SYNCWORD_PRIVATE : 0);
    d.start_rx = start_rx ? 1 : 0;

    const uint32_t status_before = status_rx_count_;
    if (!link_.sendFrame(MSG_SET_CONFIG, reinterpret_cast<uint8_t*>(&d),
                         sizeof(d)))
    {
        return false;
    }

    // The modem acks every SET_CONFIG with a STATUS frame.
    const uint32_t deadline = millis() + ack_timeout_ms;
    while (millis() < deadline)
    {
        link_.poll(&UartModemBackend::onFrameTrampoline, this);
        if (status_rx_count_ != status_before)
        {
            cfg_freq_mhz_ = freq_mhz;
            cfg_sf_ = sf;
            cfg_bw_khz_ = bw_khz;
            cfg_cr_ = cr;
            cfg_tx_power_ = d.tx_power_dbm;
            return true;
        }
        vTaskDelay(1);
    }
    return false;
}

// ---------------------------------------------------------------------------
// Frame dispatch
// ---------------------------------------------------------------------------

void UartModemBackend::onFrameTrampoline(void* ctx, uint8_t type,
                                         const uint8_t* payload, size_t len)
{
    static_cast<UartModemBackend*>(ctx)->onFrame(type, payload, len);
}

void UartModemBackend::onFrame(uint8_t type, const uint8_t* payload, size_t len)
{
    switch (type)
    {
        case MSG_RX_FRAME:
        {
            if (len <= sizeof(RxFrameHeader))
            {
                break;
            }
            RxFrameHeader hdr;
            memcpy(&hdr, payload, sizeof(hdr));
            const uint8_t* air = payload + sizeof(hdr);
            const size_t air_len = len - sizeof(hdr);
            if (air_len > MAX_AIR_FRAME)
            {
                break;
            }
            if (rxq_used_ >= RX_QUEUE_LEN)
            {
                // Oldest-drop: the OC polls every loop tick, so a full queue
                // means the loop stalled — keep the newest frames.
                rxq_head_ = (rxq_head_ + 1) % RX_QUEUE_LEN;
                rxq_used_--;
                stats_.rx_len_drop++;  // repurposed: queue-overflow drops
            }
            RxEntry& e = rx_queue_[(rxq_head_ + rxq_used_) % RX_QUEUE_LEN];
            e.len = static_cast<uint8_t>(air_len);
            e.rssi = hdr.rssi_dbm;
            e.snr = hdr.snr_db;
            memcpy(e.data, air, air_len);
            rxq_used_++;
            stats_.rx_count++;
            break;
        }

        case MSG_TX_RESULT:
        {
            if (len != sizeof(TxResultData))
            {
                break;
            }
            TxResultData r;
            memcpy(&r, payload, sizeof(r));
            if (tx_in_flight_ && r.seq == tx_seq_)
            {
                tx_in_flight_ = false;
                stats_.transmitting = false;
                if (r.ok)
                {
                    stats_.tx_ok++;
                }
                else
                {
                    stats_.tx_fail++;
                }
            }
            break;
        }

        case MSG_STATUS:
        {
            if (len != sizeof(ModemStatusData))
            {
                break;
            }
            memcpy(&last_status_, payload, sizeof(last_status_));
            status_rx_count_++;
            // Adopt link-health counters the host can't see locally
            stats_.rx_crc_fail = last_status_.rx_crc_fail;
            stats_.tx_watchdog_fires = last_status_.tx_watchdog_fires;
            break;
        }

        case MSG_IDENTITY:
        case MSG_BOOT:
        {
            if (len != sizeof(ModemIdentityData))
            {
                break;
            }
            memcpy(&identity_, payload, sizeof(identity_));
            const bool was_alive = modem_alive_;
            modem_alive_ = true;
            if (type == MSG_BOOT && was_alive)
            {
                // Daughterboard rebooted underneath us (brownout / watchdog):
                // its queue is empty and its radio is on boot defaults.
                // Clear the in-flight slot and re-push our config.
                ESP_LOGW(TAG, "modem REBOOTED — re-pushing radio config");
                if (tx_in_flight_)
                {
                    tx_in_flight_ = false;
                    stats_.tx_fail++;
                }
                (void)pushConfig(cfg_freq_mhz_, cfg_sf_, cfg_bw_khz_, cfg_cr_,
                                 cfg_tx_power_, /*start_rx=*/true,
                                 /*ack_timeout_ms=*/300);
            }
            break;
        }

        default:
            if (debug_)
            {
                ESP_LOGW(TAG, "unknown modem msg 0x%02X (%u B)", type,
                         (unsigned)len);
            }
            break;
    }
}

// ---------------------------------------------------------------------------
// IRadioLink surface
// ---------------------------------------------------------------------------

void UartModemBackend::service()
{
    if (!began_)
    {
        return;
    }
    link_.poll(&UartModemBackend::onFrameTrampoline, this);
}

bool UartModemBackend::send(const uint8_t* payload, size_t len)
{
    if (!modem_alive_ || payload == nullptr || len == 0 ||
        len > MAX_AIR_FRAME || tx_in_flight_)
    {
        return false;
    }

    uint8_t buf[sizeof(TxFrameHeader) + MAX_AIR_FRAME];
    tx_seq_++;
    buf[0] = tx_seq_;
    memcpy(buf + sizeof(TxFrameHeader), payload, len);
    if (!link_.sendFrame(MSG_TX_FRAME, buf, sizeof(TxFrameHeader) + len))
    {
        stats_.tx_fail++;
        return false;
    }
    tx_in_flight_ = true;
    tx_started_ms_ = millis();
    stats_.tx_started++;
    stats_.transmitting = true;
    return true;
}

bool UartModemBackend::canSend() const
{
    return modem_alive_ && !tx_in_flight_;
}

bool UartModemBackend::startReceive()
{
    if (!modem_alive_)
    {
        return false;
    }
    // The modem auto-listens whenever it isn't transmitting; this is a
    // belt-and-braces nudge, always considered successful.
    (void)link_.sendFrame(MSG_START_RX, nullptr, 0);
    return true;
}

bool UartModemBackend::readPacket(uint8_t* buf, size_t maxLen, size_t& len)
{
    len = 0;
    if (rxq_used_ == 0)
    {
        return false;
    }
    RxEntry& e = rx_queue_[rxq_head_];
    rxq_head_ = (rxq_head_ + 1) % RX_QUEUE_LEN;
    rxq_used_--;
    if (e.len > maxLen)
    {
        stats_.rx_len_drop++;
        return false;
    }
    memcpy(buf, e.data, e.len);
    len = e.len;
    stats_.last_rssi = e.rssi;
    stats_.last_snr = e.snr;
    return true;
}

bool UartModemBackend::waitTxIdle(uint32_t timeout_ms)
{
    const uint32_t deadline = millis() + timeout_ms;
    while (tx_in_flight_)
    {
        if (millis() >= deadline)
        {
            return false;
        }
        link_.poll(&UartModemBackend::onFrameTrampoline, this);
        serviceTxWatchdog();
        vTaskDelay(1);
    }
    return true;
}

bool UartModemBackend::reconfigure(float freq_mhz, uint8_t sf, float bw_khz,
                                   uint8_t cr, int8_t tx_power,
                                   bool wait_for_tx)
{
    if (!modem_alive_)
    {
        return false;
    }
    if (tx_in_flight_)
    {
        if (!wait_for_tx)
        {
            // Non-blocking contract (#405 rendezvous paths): busy → caller
            // retries next loop iteration.
            return false;
        }
        if (!waitTxIdle(2000))  // matches the direct driver's 2 s deadline
        {
            return false;
        }
    }
    return pushConfig(freq_mhz, sf, bw_khz, cr, tx_power, /*start_rx=*/true,
                      /*ack_timeout_ms=*/500);
}

bool UartModemBackend::hopToFrequencyMHz(float freq_mhz)
{
    if (!modem_alive_ || tx_in_flight_)
    {
        // Mirrors the direct driver: no retune mid-TX; caller keeps its
        // channel state and retries at the next packet boundary.
        return false;
    }
    HopFreqData d = {freq_mhz};
    if (!link_.sendFrame(MSG_HOP_FREQ, reinterpret_cast<uint8_t*>(&d),
                         sizeof(d)))
    {
        return false;
    }
    cfg_freq_mhz_ = freq_mhz;
    return true;
}

bool UartModemBackend::isInRxMode() const
{
    // Modem policy: always listening except while transmitting.
    return modem_alive_ && !tx_in_flight_;
}

void UartModemBackend::serviceTxWatchdog()
{
    if (tx_in_flight_ &&
        (millis() - tx_started_ms_) > TX_WATCHDOG_MS)
    {
        // TX_RESULT never came back (modem died mid-frame / UART glitch):
        // free the slot so canSend() can't wedge false forever (#105
        // philosophy). The modem's own watchdog guards its side.
        ESP_LOGW(TAG, "TX_RESULT watchdog fired (seq=%u)", tx_seq_);
        tx_in_flight_ = false;
        stats_.transmitting = false;
        stats_.tx_fail++;
        stats_.tx_watchdog_fires++;
    }
}

void UartModemBackend::getStats(TR_LoRa_Comms::Stats& out) const
{
    out = stats_;
    out.enabled = modem_alive_;
    out.rx_mode = isInRxMode();
    out.transmitting = tx_in_flight_;
}
