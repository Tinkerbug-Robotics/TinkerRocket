#pragma once

#include <cstdint>

namespace config
{
    // --- Debug ---
    static constexpr bool DEBUG = true;
    static constexpr uint32_t STATS_PERIOD_MS = 5000;

    // --- LoRa Radio (LLCC68 via RadioLib) ---
    // Must match OutComputer radio parameters exactly
    static constexpr int LORA_SPI_SCK  = 10;//36
    static constexpr int LORA_SPI_MISO = 13;//34
    static constexpr int LORA_SPI_MOSI = 12;//35
    static constexpr int LORA_CS_PIN   = 11;//38
    static constexpr int LORA_DIO1_PIN = 21;//18
    static constexpr int LORA_RST_PIN  = 36;//37
    static constexpr int LORA_BUSY_PIN = 14;//17

    static constexpr float   LORA_FREQ_MHZ       = 915.0f;
    static constexpr uint8_t LORA_SF              = 8;
    static constexpr float   LORA_BW_KHZ          = 250.0f;
    static constexpr uint8_t LORA_CR              = 5;
    static constexpr uint16_t LORA_PREAMBLE_LEN   = 12;
    static constexpr int8_t  LORA_TX_POWER_DBM    = 12;
    static constexpr bool    LORA_CRC_ON          = true;
    static constexpr bool    LORA_RX_BOOSTED_GAIN = true;
    static constexpr bool    LORA_SYNCWORD_PRIVATE = true;

    // --- LoRa Uplink (BaseStation → OutComputer) ---
    static constexpr uint8_t  UPLINK_SYNC_BYTE        = 0xCA;
    static constexpr uint8_t  UPLINK_RETRIES           = 8;     // TX attempts per command
    static constexpr uint32_t UPLINK_RETRY_INTERVAL_MS = 100;   // Delay between retries

    // --- SD Card (SDMMC 4-bit) ---
    static constexpr int SD_CLK  = 6;
    static constexpr int SD_CMD  = 7;
    static constexpr int SD_D0   = 5;
    static constexpr int SD_D1   = 4;
    static constexpr int SD_D2   = 9;
    static constexpr int SD_D3   = 8;

    // --- LoRa CSV Logging ---
    static constexpr uint32_t LOG_SILENCE_TIMEOUT_MS = 30000;   // Close file after 30s of no packets
    static constexpr uint32_t LOG_LANDED_TIMEOUT_MS  = 30000;   // Close file 30s after LANDED
    static constexpr size_t   BLE_FILE_CHUNK_SIZE    = 170;     // Bytes per BLE download chunk
    static constexpr uint32_t BLE_CHUNK_DELAY_MS     = 15;      // Delay between BLE chunks (ms)
    static constexpr size_t   FILES_PER_PAGE         = 5;       // BLE file list pagination
    static constexpr uint32_t LOG_FLUSH_INTERVAL_MS  = 10000;   // Periodic flush to flash (10s)

    // --- I2C Bus (MAX17205G fuel gauge) ---
    static constexpr int      I2C_SCL_PIN       = 37;
    static constexpr int      I2C_SDA_PIN       = 38;
    static constexpr uint32_t I2C_FREQ_HZ       = 400'000;  // 400 kHz

    // --- Device Identity (factory defaults, overridden by NVS on boot) ---
    static constexpr uint8_t DEFAULT_NETWORK_ID = 0;
    static constexpr const char* DEVICE_TYPE     = "B";  // "B" = base station

    // --- Battery Monitoring (MAX17205G) ---
    static constexpr uint16_t MAX17205_ADDR      = 0x36;     // Primary I2C address
    static constexpr int      NUM_BATTERY_CELLS  = 2;        // 2S NCR18650B
    static constexpr float    RSENSE_MOHM        = 10.0f;    // Sense resistor (mΩ)
    static constexpr uint32_t PWR_UPDATE_PERIOD_MS = 2000;   // Battery read interval
    // Pack design capacity in mAh. Two 2800 mAh 18650 cells in 2S = 2800 mAh
    // (series doubles voltage, capacity stays per-cell). Written to MAX17205
    // DesignCap at boot to seed the ModelGauge m5 algorithm.
    static constexpr uint16_t BATTERY_DESIGN_MAH = 2800;
}
