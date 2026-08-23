#include "TR_LoRa_Comms.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <cmath>

static const char* TAG = "LORA";

// SPI-bus / RadioLib stall instrumentation (#90 follow-up).  Bench logs
// showed every-15 s, ~745 ms blocking on Core 1 with all six rocket sensor
// streams freezing together — the resume sync points to a single Core-1
// blocker.  reconfigure() is the prime suspect (its TX-wait loop has a 2 s
// deadline; individual SPI calls can stack behind a slow NAND op via the
// shared bus mutex).  Anything inside this file that takes longer than
// LORA_STALL_THRESHOLD_US is logged with its step name so the next bench
// run names the offending op.
static constexpr int64_t LORA_STALL_THRESHOLD_US = 50'000;  // 50 ms

#define LORA_STALL_INSTR(name, expr) do {                                      \
    const int64_t _stall_t0_ = esp_timer_get_time();                           \
    expr;                                                                      \
    const int64_t _stall_dt_ = esp_timer_get_time() - _stall_t0_;              \
    if (_stall_dt_ > LORA_STALL_THRESHOLD_US) {                                \
        ESP_LOGW(TAG, "STALL: %s took %lld us", (name), (long long)_stall_dt_); \
    }                                                                          \
} while (0)

TR_LoRa_Comms* TR_LoRa_Comms::instance_ = nullptr;

TR_LoRa_Comms::TR_LoRa_Comms()
{
}

bool TR_LoRa_Comms::begin(const Config& cfg, bool debug)
{
    debug_ = debug;
    enabled_ = cfg.enabled;
    stats_ = {};
    stats_.enabled = enabled_;

    if (!enabled_)
    {
        return true;
    }

    hal_ = new EspHal(cfg.spi_sck, cfg.spi_miso, cfg.spi_mosi, cfg.spi_host);
    module_ = new Module(hal_, cfg.cs_pin, cfg.dio1_pin, cfg.rst_pin, cfg.busy_pin);
    radio_ = new LLCC68(module_);
    if (hal_ == nullptr || module_ == nullptr || radio_ == nullptr)
    {
        enabled_ = false;
        stats_.enabled = false;
        return false;
    }

    dio1_pin_ = cfg.dio1_pin;
    hal_->pinMode(cfg.busy_pin, ESPHAL_INPUT);
    hal_->pinMode(cfg.dio1_pin, ESPHAL_INPUT);
    hal_->pinMode(cfg.rst_pin, ESPHAL_OUTPUT);
    hal_->digitalWrite(cfg.rst_pin, ESPHAL_LOW);
    hal_->delay(50);
    hal_->digitalWrite(cfg.rst_pin, ESPHAL_HIGH);
    hal_->delay(50);

    (void)radio_->setDio2AsRfSwitch(true);
    if (cfg.rxen_pin >= 0)
    {
        // MCU-driven RX-enable half of the RF switch (TX half is DIO2,
        // radio-driven). RadioLib toggles it on every mode transition.
        module_->setRfSwitchPins(static_cast<uint32_t>(cfg.rxen_pin),
                                 RADIOLIB_NC);
    }

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

    // LLCC68 validates SF against the current BW, so set BW first.
    //
    // #835 item 7: these stay non-fatal -- a radio that came up with one
    // setter refused beats no radio, and failing begin() here would take the
    // whole link down. But the results are no longer thrown away: the cache
    // written just below records the REQUESTED values either way, so without
    // begin_clean_ a partial failure is invisible to every consumer, including
    // a host comparing the STATUS echo against its own push.
    begin_clean_ = true;
    auto step = [this](int16_t st) { if (st != RADIOLIB_ERR_NONE) begin_clean_ = false; };

    step(radio_->setBandwidth(cfg.bandwidth_khz));
    step(radio_->setSpreadingFactor(cfg.spreading_factor));
    step(radio_->setCodingRate(cfg.coding_rate));
    if (cfg.syncword_private)
    {
        step(radio_->setSyncWord(RADIOLIB_SX126X_SYNC_WORD_PRIVATE));
    }
    step(radio_->setPreambleLength(cfg.preamble_len));
    step(radio_->setOutputPower(cfg.tx_power_dbm));
    step(radio_->setRxBoostedGainMode(cfg.rx_boosted_gain));
    step(radio_->setCRC(cfg.crc_on));
    if (!begin_clean_)
    {
        ESP_LOGE(TAG, "begin(): a radio setter was refused — the chip is NOT "
                      "fully on the requested config (SF%u BW%.0f CR%u)",
                 cfg.spreading_factor, (double)cfg.bandwidth_khz,
                 cfg.coding_rate);
    }

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
    int16_t st;
    LORA_STALL_INSTR("service finishTransmit", st = radio_->finishTransmit());
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
    tx_started_ms_ = 0;  // watchdog clear (#105)
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
    // A scan owns the radio's tuning; transmitting now would go out on whatever
    // channel it is dwelling on and corrupt the pass. Refuse, mirroring
    // hopToFrequencyMHz's can't-retune-mid-scan guard (#379). Callers should
    // gate on their own scan tracking too (see serviceUplink) — this is the
    // driver-level backstop.
    if (isScanActive())
    {
        return false;
    }

    // Switch out of RX mode if active
    rx_mode_ = false;
    rx_done_ = false;

    int16_t st;
    LORA_STALL_INSTR("send startTransmit", st = radio_->startTransmit(payload, len));
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
    tx_started_ms_ = millis();  // watchdog stamp (#105)
    stats_.transmitting = true;
    stats_.tx_started++;
    return true;
}

bool TR_LoRa_Comms::canSend() const
{
    return enabled_ && !tx_ongoing_ && !isScanActive();  // #379: not mid-scan
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
    tx_started_ms_ = 0;  // watchdog clear (#105)

    int16_t st;
    LORA_STALL_INSTR("startReceive radio_->startReceive", st = radio_->startReceive());
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

    // #520: rx_done_ is only a HINT. It has two racing writers — the DIO1 ISR
    // (edge-triggered, fires once per packet) and pollDio1() (the missed-interrupt
    // fallback, which latches on the DIO1 *level*). The radio holds DIO1 asserted
    // until readData() below clears the RxDone IRQ, several SPI transactions after
    // we clear rx_done_ — so there is a window where rx_done_ is false while DIO1
    // is still high, and a level-based latch landing in it re-arms the flag for a
    // packet we ALREADY consumed. The next call would then re-read the SX126x
    // buffer, which still holds that packet, and hand back a byte-identical
    // duplicate (bench: same seq, same RSSI/SNR to 0.1 dB, ~9 ms later).
    //
    // So ask the radio, not the flag. DIO1 is masked to RxDone only, so a set
    // RX_DONE bit means a genuinely new packet is waiting.
    if (!(radio_->getIrqFlags() & RADIOLIB_SX126X_IRQ_RX_DONE))
    {
        portDISABLE_INTERRUPTS();
        rx_done_ = false;
        portENABLE_INTERRUPTS();
        stats_.rx_spurious++;
        return false;
    }

    // Clear the flag atomically w.r.t. the DIO1 ISR so we never lose
    // an rx_done_ that fires between the check above and this assignment.
    portDISABLE_INTERRUPTS();
    rx_done_ = false;
    portENABLE_INTERRUPTS();

    // Get the length of the received packet
    const size_t pkt_len = radio_->getPacketLength();
    if (pkt_len == 0 || pkt_len > maxLen)
    {
        // #288: count the drop so a stuck/garbage length field is visible in
        // [STATS] instead of vanishing silently.  Implausible in practice
        // (maxLen 256 vs ~62 B real packets), so a nonzero count is a signal.
        stats_.rx_len_drop++;
        if (debug_)
        {
            ESP_LOGW(TAG, "LoRa RX bad length %u (max %u), dropped",
                     (unsigned)pkt_len, (unsigned)maxLen);
        }
        // Restart RX for next packet
        (void)radio_->startReceive();
        return false;
    }

    // Read the packet data
    int16_t st;
    LORA_STALL_INSTR("readPacket readData", st = radio_->readData(buf, pkt_len));
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
    portDISABLE_INTERRUPTS();
    if (rx_done_ || tx_done_)
    {
        portENABLE_INTERRUPTS();
        return;
    }
    // Check DIO1 pin state directly
    if (gpio_get_level((gpio_num_t)dio1_pin_) == 1)
    {
        isr_count_ = isr_count_ + 1;  // volatile: '++' deprecated in C++20 (-Wvolatile)
        if (rx_mode_)
        {
            rx_done_ = true;
        }
        else
        {
            tx_done_ = true;
        }
    }
    portENABLE_INTERRUPTS();
}

// ============================================================================
// TX watchdog (#105 follow-up)
// ============================================================================
// Force-clears tx_ongoing_ when it has been stuck for longer than any
// plausible TX could take.  Without this, a missed DIO1 IRQ + a
// misclassified pollDio1 (rx_mode_ momentarily true at the polling
// instant during a hopToFrequencyMHz retune) leaves tx_ongoing_=true
// forever — wedging hopToFrequencyMHz, reconfigure, send, and ultimately
// the entire LoRa link until reboot.  Field-confirmed failure mode in
// the #105 hop testing.
//
// The watchdog is deliberately generous (TX_WATCHDOG_MS = 3 s) to
// accommodate worst-case ToA at SF12 BW125 (~2.5 s for a 60-byte frame).
// A real TX always completes well inside this window; if we hit it,
// something is genuinely wrong and the recovery is to put the radio
// back into a known RX state.
void TR_LoRa_Comms::serviceTxWatchdog()
{
    if (!enabled_ || radio_ == nullptr) return;
    if (!tx_ongoing_)                   return;
    if (tx_started_ms_ == 0)            return;  // not yet stamped
    const uint32_t elapsed = millis() - tx_started_ms_;
    if (elapsed < TX_WATCHDOG_MS)       return;

    ESP_LOGW(TAG, "TX watchdog: tx_ongoing_ stuck %u ms — force-clearing "
                  "(tx_done=%d rx_mode=%d isr_count=%lu last_err=%d)",
             (unsigned)elapsed,
             (int)tx_done_, (int)rx_mode_,
             (unsigned long)isr_count_,
             (int)stats_.last_error);
    stats_.tx_watchdog_fires++;

    // Clear the stuck flags so the rest of the system unwedges.
    portDISABLE_INTERRUPTS();
    tx_ongoing_   = false;
    tx_done_      = false;
    tx_started_ms_ = 0;
    portENABLE_INTERRUPTS();
    stats_.transmitting = false;

    // Best-effort radio recovery: standby, then re-enter RX so the radio
    // and the driver agree on state.  We don't propagate errors — if
    // these calls fail, the radio is even more wedged and the next
    // bigger hammer (reconfigure on a recovery cycle) will handle it.
    (void)radio_->standby();
    if (radio_->startReceive() == RADIOLIB_ERR_NONE)
    {
        rx_mode_ = true;
        rx_done_ = false;
    }
    else
    {
        rx_mode_ = false;
    }
}

// ============================================================================
// Runtime reconfiguration
// ============================================================================

bool TR_LoRa_Comms::reconfigure(float freq_mhz, uint8_t sf, float bw_khz, uint8_t cr, int8_t tx_power,
                                bool wait_for_tx)
{
    if (!enabled_ || radio_ == nullptr)
    {
        return false;
    }

    // Whole-call timing — gives an easy upper-bound number for the bench
    // logs even before any per-step warning fires.
    const int64_t _reconf_t0 = esp_timer_get_time();

    // A retune mid-TX corrupts the in-flight packet, so an ongoing TX must
    // finish first.  Two modes (#398):
    //   wait_for_tx=true  — block up to 2 s.  For user-initiated config paths
    //                       that won't retry (without this, reconfigure
    //                       silently failed ~18% of the time mid-transmit and
    //                       NVS never got updated).
    //   wait_for_tx=false — return busy immediately.  For main-loop service
    //                       paths (rendezvous): a packet's airtime is
    //                       100-200 ms at SF8/BW250 and the spin-wait stalled
    //                       loop_oc for all of it; the caller retries next
    //                       iteration instead.
    if (tx_ongoing_ && !wait_for_tx)
    {
        return false;  // busy — caller retries
    }
    if (tx_ongoing_)
    {
        const int64_t _wait_t0 = esp_timer_get_time();
        const uint32_t deadline = millis() + 2000;
        while (tx_ongoing_ && millis() < deadline)
        {
            pollDio1();
            if (tx_done_) { service(); }
            vTaskDelay(1);
        }
        const int64_t _wait_dt = esp_timer_get_time() - _wait_t0;
        if (_wait_dt > LORA_STALL_THRESHOLD_US) {
            ESP_LOGW(TAG, "STALL: reconfigure wait-for-TX took %lld us "
                          "(hit_deadline=%d)",
                     (long long)_wait_dt, tx_ongoing_ ? 1 : 0);
        }
        if (tx_ongoing_) { return false; }  // TX stuck -- give up
    }

    // Exit RX mode
    rx_mode_ = false;
    rx_done_ = false;

    // Save old config for rollback on partial failure. setOutputPower is the
    // final step, so there's no successful step after it to unwind — its
    // failure leaves the prior power setting untouched and nothing to restore
    // (hence no old_pwr saved here).
    const float   old_bw   = cfg_bw_khz_;
    const uint8_t old_sf   = cfg_sf_;
    const float   old_freq = cfg_freq_mhz_;
    const uint8_t old_cr   = cfg_cr_;
    int steps_done = 0;

    // LLCC68 validates SF against the current BW, so set BW first
    int16_t st;
    LORA_STALL_INSTR("reconfigure setBandwidth", st = radio_->setBandwidth(bw_khz));
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "LoRa reconfigure setBW failed: %d", st);
        return false;  // Nothing changed, no rollback needed
    }
    steps_done = 1;

    LORA_STALL_INSTR("reconfigure setSpreadingFactor", st = radio_->setSpreadingFactor(sf));
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "LoRa reconfigure setSF failed: %d", st);
        goto rollback;
    }
    steps_done = 2;

    LORA_STALL_INSTR("reconfigure setFrequency", st = radio_->setFrequency(freq_mhz));
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "LoRa reconfigure setFreq failed: %d", st);
        goto rollback;
    }
    steps_done = 3;

    LORA_STALL_INSTR("reconfigure setCodingRate", st = radio_->setCodingRate(cr));
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "LoRa reconfigure setCR failed: %d", st);
        goto rollback;
    }
    steps_done = 4;

    LORA_STALL_INSTR("reconfigure setOutputPower", st = radio_->setOutputPower(tx_power));
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "LoRa reconfigure setPower failed: %d", st);
        goto rollback;
    }

    // Success -- update last-known-good config
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

    {
        const int64_t _reconf_dt = esp_timer_get_time() - _reconf_t0;
        if (_reconf_dt > LORA_STALL_THRESHOLD_US) {
            ESP_LOGW(TAG, "STALL: reconfigure(total) took %lld us "
                          "(target %.2f MHz SF%u BW%.0f)",
                     (long long)_reconf_dt,
                     (double)freq_mhz, (unsigned)sf, (double)bw_khz);
        }
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
// Frame-format parameters (preamble/CRC/gain/syncword) — runtime application
// ============================================================================

bool TR_LoRa_Comms::applyFrameParams(uint16_t preamble_len, bool crc_on,
                                     bool rx_boosted_gain,
                                     bool syncword_private)
{
    if (!enabled_ || radio_ == nullptr)
    {
        return false;
    }
    if (tx_ongoing_ || isScanActive())
    {
        // Same no-retune-mid-TX rule as reconfigure(); the caller retries
        // when idle. No wait mode — the one caller (the UART modem's
        // SET_CONFIG path, #409) runs right after a completed reconfigure.
        return false;
    }

    // Exit RX mode, mirroring reconfigure(): SetPacketParams while the
    // receiver is armed is exactly the ambiguity we avoid there.
    rx_mode_ = false;
    rx_done_ = false;

    int16_t st = radio_->setPreambleLength(preamble_len);
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "applyFrameParams setPreamble failed: %d", st);
        return false;
    }
    st = radio_->setCRC(crc_on);
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "applyFrameParams setCRC failed: %d", st);
        return false;
    }
    st = radio_->setRxBoostedGainMode(rx_boosted_gain);
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "applyFrameParams setRxBoostedGain failed: %d", st);
        return false;
    }
    st = radio_->setSyncWord(syncword_private ? RADIOLIB_SX126X_SYNC_WORD_PRIVATE
                                              : RADIOLIB_SX126X_SYNC_WORD_PUBLIC);
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "applyFrameParams setSyncWord failed: %d", st);
        return false;
    }

    if (debug_)
    {
        ESP_LOGI(TAG, "frame params applied: preamble=%u crc=%d boost=%d "
                      "syncword=%s",
                 (unsigned)preamble_len, (int)crc_on, (int)rx_boosted_gain,
                 syncword_private ? "private" : "public");
    }
    return true;
}

// ============================================================================
// Lightweight frequency-only retune for per-packet hopping (#40 / #41)
// ============================================================================

bool TR_LoRa_Comms::hopToFrequencyMHz(float freq_mhz)
{
    if (!enabled_ || radio_ == nullptr)         return false;
    if (tx_ongoing_)                            return false;  // can't retune mid-TX
    if (scan_state_ != ScanState::Idle &&
        scan_state_ != ScanState::Done)         return false;  // can't retune mid-scan

    // No-op fast path: already on the requested frequency (within
    // floating-point slop).  Avoids unnecessary radio churn when the
    // hop state machine asks for the channel we're already on.
    if (fabsf(freq_mhz - cfg_freq_mhz_) < 0.0005f) return true;

    // Mirror the established pattern from reconfigure() / startScan():
    // mark RX as exited *before* issuing any SPI to the radio.  Two
    // reasons:
    //   1. RadioLib's setFrequency on SX126x is multi-step (standby →
    //      calibrateImage → setRfFreq → setOutputPower).  Field testing
    //      crashed inside ESP-IDF's spi_bus_lock_bg_exit (NULL deref on
    //      lock->acquiring_dev — a TOCTOU race in the driver's
    //      ISR-completion path) when these bursts overlapped with
    //      DIO1-ISR-triggered rx_done_ handling.
    //   2. Without this, our own DIO1 ISR (gated on rx_mode_) would
    //      latch rx_done_ on any spurious DIO1 edge during the retune,
    //      causing the main loop to fire off a readPacket() SPI burst
    //      on top of the in-flight ones.
    const bool was_rx = rx_mode_;
    rx_mode_ = false;
    rx_done_ = false;

    int16_t st;
    LORA_STALL_INSTR("hopToFrequency setFrequency", st = radio_->setFrequency(freq_mhz));
    if (st != RADIOLIB_ERR_NONE)
    {
        stats_.last_error = st;
        if (debug_) ESP_LOGE(TAG, "LoRa hopToFrequency failed: %d", st);
        // Best-effort: if we were in RX, try to get back so a single
        // failed retune doesn't strand the radio in standby.
        if (was_rx) (void)startReceive();
        return false;
    }
    cfg_freq_mhz_ = freq_mhz;

    // Re-enter RX via the canonical wrapper so internal flag state
    // matches what the rest of the codebase expects after a fresh RX.
    if (was_rx) (void)startReceive();
    return true;
}

// ============================================================================
// Spectrum scan
// ============================================================================

bool TR_LoRa_Comms::startScan(float start_mhz, float stop_mhz, uint16_t step_khz, uint16_t dwell_ms)
{
    if (!enabled_ || radio_ == nullptr) return false;
    if (scan_state_ != ScanState::Idle) return false;  // already scanning or result pending
    if (step_khz == 0 || stop_mhz <= start_mhz) return false;

    // Clamp dwell to a reasonable range.  Below ~5 ms the receiver front end
    // hasn't fully settled after setFrequency, so RSSI readings drift high.
    if (dwell_ms < 5)   dwell_ms = 5;
    if (dwell_ms > 500) dwell_ms = 500;

    const float span_khz = (stop_mhz - start_mhz) * 1000.0f;
    uint32_t n = (uint32_t)(span_khz / (float)step_khz) + 1;
    if (n > SCAN_MAX_SAMPLES) n = SCAN_MAX_SAMPLES;

    // Refuse to scan while TX is still in flight — setFrequency mid-transmit
    // corrupts the packet and can leave the radio in an undefined state.
    if (tx_ongoing_) return false;

    rx_mode_ = false;
    rx_done_ = false;

    scan_start_mhz_ = start_mhz;
    scan_step_khz_  = (float)step_khz;
    scan_n_steps_   = (uint16_t)n;
    scan_idx_       = 0;
    scan_dwell_ms_  = dwell_ms;
    scan_count_     = 0;
    // #567: wall-clock backstop. A healthy step costs one SetFreq service
    // pass plus dwell_ms of Dwell passes; 250 ms/step of margin covers even a
    // badly stalled main loop, so this only ever fires on a genuinely stuck
    // machine (unknown radio fault / future regression) — and then force-
    // finishes the scan instead of leaving isScanActive() pinned true, which
    // would kill TX and hopping until reboot.
    scan_deadline_ms_ = millis() + (uint32_t)n * (uint32_t)(dwell_ms + 250u) + 5000u;
    scan_state_     = ScanState::SetFreq;

    if (debug_)
    {
        ESP_LOGI(TAG, "Scan start: %.1f → %.1f MHz, %u kHz step, %u ms dwell, %u steps",
                 (double)start_mhz, (double)stop_mhz,
                 (unsigned)step_khz, (unsigned)dwell_ms, (unsigned)n);
    }
    return true;
}

// #567: the ONLY way out of an active scan. Restores the operating frequency,
// re-enters RX, and marks Done so isScanActive() releases the TX / hop gates.
// The restore calls are best-effort by design: on a wedged SPI bus they fail
// too, but the state transition must happen regardless — a scan that never
// reaches Done pins isScanActive() true and kills send()/canSend()/
// hopToFrequencyMHz() until reboot.
void TR_LoRa_Comms::finishScan(const char* why)
{
    (void)radio_->setFrequency(cfg_freq_mhz_);
    (void)radio_->startReceive();
    rx_mode_ = true;
    scan_state_ = ScanState::Done;
    if (debug_)
    {
        ESP_LOGI(TAG, "Scan %s: %u samples, restored %.1f MHz",
                 why, (unsigned)scan_count_, (double)cfg_freq_mhz_);
    }
}

void TR_LoRa_Comms::serviceScan()
{
    if (!enabled_ || radio_ == nullptr) return;
    if (scan_state_ == ScanState::Idle || scan_state_ == ScanState::Done) return;

    // #567: wall-clock backstop (armed in startScan). The state machine below
    // terminates by construction — both SetFreq outcomes advance it — so this
    // fires only on an unknown failure mode or a future regression. Wrap-safe
    // compare, same idiom as the TX watchdog.
    if ((int32_t)(millis() - scan_deadline_ms_) >= 0)
    {
        ESP_LOGW(TAG, "Scan deadline hit at step %u/%u — force-finishing so the "
                      "link is not held hostage",
                 (unsigned)scan_idx_, (unsigned)scan_n_steps_);
        finishScan("aborted (deadline)");
        return;
    }

    switch (scan_state_)
    {
        case ScanState::SetFreq:
        {
            const float f = scan_start_mhz_ + (scan_step_khz_ * (float)scan_idx_) / 1000.0f;
            int16_t st = radio_->setFrequency(f);
            if (st != RADIOLIB_ERR_NONE)
            {
                stats_.last_error = st;
                // Record a sentinel and skip to next channel.
                if (scan_count_ < SCAN_MAX_SAMPLES)
                {
                    scan_samples_[scan_count_++] = { f, -128 };
                }
                scan_idx_++;
                // #567: this branch used to skip the completion check, so a
                // setFrequency failure on the LAST channel — or a persistent
                // one (range outside the radio's band, wedged SPI) — pushed
                // scan_idx_ past scan_n_steps_ with Done unreachable: the
                // machine spun in SetFreq forever and isScanActive() stayed
                // true until reboot. Mirror the Dwell branch's completion.
                if (scan_idx_ >= scan_n_steps_)
                {
                    finishScan("done (last setFrequency failed)");
                }
            }
            else
            {
                // Enter RX so the front end is active and RSSI is meaningful.
                (void)radio_->startReceive();
                scan_dwell_start_ms_ = millis();
                scan_state_ = ScanState::Dwell;
            }
            break;
        }
        case ScanState::Dwell:
        {
            if ((millis() - scan_dwell_start_ms_) >= scan_dwell_ms_)
            {
                const float rssi = radio_->getRSSI(false);  // instantaneous, not packet
                int rssi_i = (int)rssi;
                if (rssi_i < -128) rssi_i = -128;
                if (rssi_i >  127) rssi_i =  127;
                const float f = scan_start_mhz_ + (scan_step_khz_ * (float)scan_idx_) / 1000.0f;
                if (scan_count_ < SCAN_MAX_SAMPLES)
                {
                    scan_samples_[scan_count_++] = { f, (int8_t)rssi_i };
                }
                scan_idx_++;
                if (scan_idx_ >= scan_n_steps_)
                {
                    // Restore the operating frequency and return to RX so
                    // normal comms with the OutComputer resume.
                    finishScan("done");
                }
                else
                {
                    scan_state_ = ScanState::SetFreq;
                }
            }
            break;
        }
        default:
            break;
    }
}

// ============================================================================
// ISR
// ============================================================================

void IRAM_ATTR TR_LoRa_Comms::onDio1ISR()
{
    if (instance_ != nullptr)
    {
        instance_->isr_count_ = instance_->isr_count_ + 1;  // volatile: '++' deprecated in C++20 (-Wvolatile)
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
