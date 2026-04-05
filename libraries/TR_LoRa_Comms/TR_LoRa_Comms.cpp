#include "TR_LoRa_Comms.h"
#include <esp_log.h>

static const char* TAG = "LORA";

TR_LoRa_Comms* TR_LoRa_Comms::instance_ = nullptr;

TR_LoRa_Comms::TR_LoRa_Comms()
{
}

bool TR_LoRa_Comms::begin(SPIClass& spi, const Config& cfg, bool debug)
{
    debug_ = debug;
    enabled_ = cfg.enabled;
    stats_ = {};
    stats_.enabled = enabled_;

    if (!enabled_)
    {
        return true;
    }

    module_ = new Module(cfg.cs_pin, cfg.dio1_pin, cfg.rst_pin, cfg.busy_pin, spi);
    radio_ = new LLCC68(module_);
    if (module_ == nullptr || radio_ == nullptr)
    {
        enabled_ = false;
        stats_.enabled = false;
        return false;
    }

    dio1_pin_ = cfg.dio1_pin;
    pinMode(cfg.busy_pin, INPUT);
    pinMode(cfg.dio1_pin, INPUT);
    pinMode(cfg.rst_pin, OUTPUT);
    digitalWrite(cfg.rst_pin, LOW);
    delay(50);
    digitalWrite(cfg.rst_pin, HIGH);
    delay(50);

    (void)radio_->setDio2AsRfSwitch(true);

    int16_t st = radio_->begin();
    stats_.last_error = st;
    if (st != RADIOLIB_ERR_NONE)
    {
        if (debug_)
        {
            ESP_LOGE(TAG, "LoRa begin failed: %d", st);
        }
        enabled_ = false;
        stats_.enabled = false;
        return false;
    }

    st = radio_->setFrequency(cfg.freq_mhz);
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_)
        {
            ESP_LOGE(TAG, "LoRa setFrequency failed: %d", st);
        }
        enabled_ = false;
        stats_.enabled = false;
        return false;
    }

    // LLCC68 validates SF against the current BW, so set BW first
    (void)radio_->setBandwidth(cfg.bandwidth_khz);
    (void)radio_->setSpreadingFactor(cfg.spreading_factor);
    (void)radio_->setCodingRate(cfg.coding_rate);
    if (cfg.syncword_private)
    {
        (void)radio_->setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE);
    }
    (void)radio_->setPreambleLength(cfg.preamble_len);
    (void)radio_->setOutputPower(cfg.tx_power_dbm);
    (void)radio_->setRxBoostedGainMode(cfg.rx_boosted_gain);
    (void)radio_->setCRC(cfg.crc_on);

    // Store last-known-good config for rollback on reconfigure failure
    cfg_freq_mhz_ = cfg.freq_mhz;
    cfg_sf_        = cfg.spreading_factor;
    cfg_bw_khz_    = cfg.bandwidth_khz;
    cfg_cr_        = cfg.coding_rate;
    cfg_tx_power_  = cfg.tx_power_dbm;

    instance_ = this;
    radio_->setDio1Action(TR_LoRa_Comms::onDio1ISR);

    if (debug_)
    {
        ESP_LOGI(TAG, "LoRa radio initialized");
    }
    return true;
}

void TR_LoRa_Comms::service()
{
    if (!enabled_ || radio_ == nullptr)
    {
        return;
    }
    if (!tx_done_)
    {
        return;
    }

    tx_done_ = false;
    const int16_t st = radio_->finishTransmit();
    stats_.last_error = st;
    if (st == RADIOLIB_ERR_NONE)
    {
        stats_.tx_ok++;
    }
    else
    {
        stats_.tx_fail++;
        if (debug_)
        {
            ESP_LOGE(TAG, "LoRa finishTransmit failed: %d", st);
        }
    }
    tx_ongoing_ = false;
    stats_.transmitting = false;

    // Return to RX mode so uplink commands can be received between sends
    startReceive();
}

bool TR_LoRa_Comms::send(const uint8_t* payload, size_t len)
{
    if (!enabled_ || radio_ == nullptr || payload == nullptr || len == 0)
    {
        return false;
    }
    if (tx_ongoing_)
    {
        return false;
    }

    // Switch out of RX mode if active
    rx_mode_ = false;
    rx_done_ = false;

    const int16_t st = radio_->startTransmit(payload, len);
    stats_.last_error = st;
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.tx_fail++;
        if (debug_)
        {
            ESP_LOGE(TAG, "LoRa startTransmit failed: %d", st);
        }
        return false;
    }

    tx_ongoing_ = true;
    stats_.transmitting = true;
    stats_.tx_started++;
    return true;
}

bool TR_LoRa_Comms::canSend() const
{
    return enabled_ && !tx_ongoing_;
}

void TR_LoRa_Comms::getStats(Stats& out) const
{
    out = stats_;
    out.rx_mode = rx_mode_;
    out.isr_count = isr_count_;
}

// ============================================================================
// RX support
// ============================================================================

bool TR_LoRa_Comms::startReceive()
{
    if (!enabled_ || radio_ == nullptr)
    {
        return false;
    }

    rx_mode_ = true;
    rx_done_ = false;
    tx_ongoing_ = false;

    const int16_t st = radio_->startReceive();
    stats_.last_error = st;
    if (st != RADIOLIB_ERR_NONE)
    {
        if (debug_)
        {
            ESP_LOGE(TAG, "LoRa startReceive failed: %d", st);
        }
        rx_mode_ = false;
        return false;
    }

    return true;
}

bool TR_LoRa_Comms::readPacket(uint8_t* buf, size_t maxLen, size_t& len)
{
    len = 0;
    if (!enabled_ || radio_ == nullptr || !rx_done_)
    {
        return false;
    }

    // Clear the flag atomically w.r.t. the DIO1 ISR so we never lose
    // an rx_done_ that fires between the check above and this assignment.
    noInterrupts();
    rx_done_ = false;
    interrupts();

    // Get the length of the received packet
    const size_t pkt_len = radio_->getPacketLength();
    if (pkt_len == 0 || pkt_len > maxLen)
    {
        // Restart RX for next packet
        (void)radio_->startReceive();
        return false;
    }

    // Read the packet data
    const int16_t st = radio_->readData(buf, pkt_len);
    stats_.last_error = st;

    if (st == RADIOLIB_ERR_NONE)
    {
        len = pkt_len;
        stats_.rx_count++;
        stats_.last_rssi = radio_->getRSSI();
        stats_.last_snr = radio_->getSNR();
    }
    else if (st == RADIOLIB_ERR_CRC_MISMATCH)
    {
        stats_.rx_crc_fail++;
        if (debug_)
        {
            ESP_LOGW(TAG, "LoRa RX CRC mismatch");
        }
    }
    else
    {
        if (debug_)
        {
            ESP_LOGE(TAG, "LoRa readData failed: %d", st);
        }
    }

    // Restart RX for the next packet
    (void)radio_->startReceive();

    return (st == RADIOLIB_ERR_NONE && len > 0);
}

// ============================================================================
// DIO1 polling fallback (call from main loop if interrupt doesn't fire)
// ============================================================================

void TR_LoRa_Comms::pollDio1()
{
    if (!enabled_ || dio1_pin_ < 0)
    {
        return;
    }

    // Read and set flags with interrupts disabled so we don't race with
    // the DIO1 ISR (which sets the same flags).
    noInterrupts();
    if (rx_done_ || tx_done_)
    {
        interrupts();
        return;
    }
    // Check DIO1 pin state directly
    if (digitalRead(dio1_pin_) == HIGH)
    {
        isr_count_++;
        if (rx_mode_)
        {
            rx_done_ = true;
        }
        else
        {
            tx_done_ = true;
        }
    }
    interrupts();
}

// ============================================================================
// Runtime reconfiguration
// ============================================================================

bool TR_LoRa_Comms::reconfigure(float freq_mhz, uint8_t sf, float bw_khz, uint8_t cr, int8_t tx_power)
{
    if (!enabled_ || radio_ == nullptr)
    {
        return false;
    }

    // Wait for any in-progress TX to complete (up to 2 s).
    // Without this, reconfigure silently fails ~18% of the time when
    // the radio happens to be mid-transmit, and NVS never gets updated.
    if (tx_ongoing_)
    {
        const uint32_t deadline = millis() + 2000;
        while (tx_ongoing_ && millis() < deadline)
        {
            pollDio1();
            if (tx_done_) { service(); }
            delay(1);
        }
        if (tx_ongoing_) { return false; }  // TX stuck — give up
    }

    // Exit RX mode
    rx_mode_ = false;
    rx_done_ = false;

    // Save old config for rollback on partial failure
    const float   old_bw   = cfg_bw_khz_;
    const uint8_t old_sf   = cfg_sf_;
    const float   old_freq = cfg_freq_mhz_;
    const uint8_t old_cr   = cfg_cr_;
    const int8_t  old_pwr  = cfg_tx_power_;
    int steps_done = 0;

    // LLCC68 validates SF against the current BW, so set BW first
    int16_t st = radio_->setBandwidth(bw_khz);
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "LoRa reconfigure setBW failed: %d", st);
        return false;  // Nothing changed, no rollback needed
    }
    steps_done = 1;

    st = radio_->setSpreadingFactor(sf);
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "LoRa reconfigure setSF failed: %d", st);
        goto rollback;
    }
    steps_done = 2;

    st = radio_->setFrequency(freq_mhz);
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "LoRa reconfigure setFreq failed: %d", st);
        goto rollback;
    }
    steps_done = 3;

    st = radio_->setCodingRate(cr);
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "LoRa reconfigure setCR failed: %d", st);
        goto rollback;
    }
    steps_done = 4;

    st = radio_->setOutputPower(tx_power);
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "LoRa reconfigure setPower failed: %d", st);
        goto rollback;
    }

    // Success — update last-known-good config
    cfg_freq_mhz_ = freq_mhz;
    cfg_sf_        = sf;
    cfg_bw_khz_    = bw_khz;
    cfg_cr_        = cr;
    cfg_tx_power_  = tx_power;

    if (debug_)
    {
        ESP_LOGI(TAG, "LoRa reconfigured: %.1f MHz SF%u BW%.0f CR%u %d dBm",
                      (double)freq_mhz, (unsigned)sf, (double)bw_khz, (unsigned)cr, (int)tx_power);
    }

    return true;

rollback:
    // Best-effort restore of the previous configuration.
    if (debug_) ESP_LOGW(TAG, "LoRa reconfigure rolling back %d steps", steps_done);
    if (steps_done >= 4) (void)radio_->setCodingRate(old_cr);
    if (steps_done >= 3) (void)radio_->setFrequency(old_freq);
    if (steps_done >= 2) (void)radio_->setSpreadingFactor(old_sf);
    if (steps_done >= 1) (void)radio_->setBandwidth(old_bw);
    return false;
}

// ============================================================================
// ISR
// ============================================================================

void IRAM_ATTR TR_LoRa_Comms::onDio1ISR()
{
    if (instance_ != nullptr)
    {
        instance_->isr_count_++;
        if (instance_->rx_mode_)
        {
            instance_->rx_done_ = true;
        }
        else
        {
            instance_->tx_done_ = true;
        }
    }
}
