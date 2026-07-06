#pragma once

#include "IRadioLink.h"

// Direct SPI LLCC68 backend (V7 boards): pure 1:1 delegation to
// TR_LoRa_Comms. Deliberately contains zero logic — the V7 flight path
// must be behavior-identical to the pre-seam code (#410).
class LoRaDirectBackend final : public IRadioLink
{
public:
    bool begin(const TR_LoRa_Comms::Config& cfg, bool debug)
    {
        return lora_.begin(cfg, debug);
    }

    void service() override { lora_.service(); }

    bool send(const uint8_t* payload, size_t len) override
    {
        return lora_.send(payload, len);
    }
    bool canSend() const override { return lora_.canSend(); }

    bool startReceive() override { return lora_.startReceive(); }
    bool readPacket(uint8_t* buf, size_t maxLen, size_t& len) override
    {
        return lora_.readPacket(buf, maxLen, len);
    }

    bool reconfigure(float freq_mhz, uint8_t sf, float bw_khz, uint8_t cr,
                     int8_t tx_power, bool wait_for_tx = true) override
    {
        return lora_.reconfigure(freq_mhz, sf, bw_khz, cr, tx_power, wait_for_tx);
    }
    bool hopToFrequencyMHz(float freq_mhz) override
    {
        return lora_.hopToFrequencyMHz(freq_mhz);
    }

    float currentFrequencyMHz() const override { return lora_.currentFrequencyMHz(); }
    uint8_t currentSpreadingFactor() const override { return lora_.currentSpreadingFactor(); }
    bool isInRxMode() const override { return lora_.isInRxMode(); }

    void pollDio1() override { lora_.pollDio1(); }
    void serviceTxWatchdog() override { lora_.serviceTxWatchdog(); }

    void getStats(TR_LoRa_Comms::Stats& out) const override { lora_.getStats(out); }

private:
    TR_LoRa_Comms lora_;
};
