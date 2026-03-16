#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

class TR_LoRa_Comms
{
public:
    struct Config
    {
        bool enabled = false;
        int cs_pin = -1;
        int dio1_pin = -1;
        int rst_pin = -1;
        int busy_pin = -1;
        float freq_mhz = 915.0f;
        uint8_t spreading_factor = 10;
        float bandwidth_khz = 125.0f;
        uint8_t coding_rate = 7;
        uint16_t preamble_len = 16;
        int8_t tx_power_dbm = 20;
        bool crc_on = true;
        bool rx_boosted_gain = true;
        bool syncword_private = true;
    };

    struct Stats
    {
        bool enabled = false;
        bool transmitting = false;
        bool rx_mode = false;
        uint32_t tx_started = 0;
        uint32_t tx_ok = 0;
        uint32_t tx_fail = 0;
        uint32_t rx_count = 0;
        uint32_t rx_crc_fail = 0;
        uint32_t isr_count = 0;
        float last_rssi = 0.0f;
        float last_snr = 0.0f;
        int16_t last_error = RADIOLIB_ERR_NONE;
    };

    TR_LoRa_Comms();
    ~TR_LoRa_Comms() = default;

    bool begin(SPIClass& spi, const Config& cfg, bool debug = false);
    void service();
    bool send(const uint8_t* payload, size_t len);
    bool canSend() const;
    bool isEnabled() const { return enabled_; }
    bool isInRxMode() const { return rx_mode_; }
    void getStats(Stats& out) const;

    // RX support
    bool startReceive();
    bool readPacket(uint8_t* buf, size_t maxLen, size_t& len);

    // Poll DIO1 pin directly (fallback if hardware interrupt doesn't fire)
    void pollDio1();

    // Runtime reconfiguration (LLCC68: must set BW before SF)
    bool reconfigure(float freq_mhz, uint8_t sf, float bw_khz, uint8_t cr, int8_t tx_power);

private:
    static void IRAM_ATTR onDio1ISR();

    static TR_LoRa_Comms* instance_;

    bool enabled_ = false;
    bool debug_ = false;
    volatile bool tx_done_ = false;
    volatile bool rx_done_ = false;
    bool tx_ongoing_ = false;
    bool rx_mode_ = false;
    volatile uint32_t isr_count_ = 0;
    int dio1_pin_ = -1;

    // Last-known-good radio config (for rollback on reconfigure failure)
    float   cfg_freq_mhz_ = 915.0f;
    uint8_t cfg_sf_ = 10;
    float   cfg_bw_khz_ = 125.0f;
    uint8_t cfg_cr_ = 7;
    int8_t  cfg_tx_power_ = 20;

    Module* module_ = nullptr;
    LLCC68* radio_ = nullptr;
    Stats stats_ = {};
};
