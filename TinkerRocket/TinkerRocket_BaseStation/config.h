#pragma once

#include <Arduino.h>

namespace config
{
    // --- Debug ---
    static constexpr bool DEBUG = true;
    static constexpr uint32_t STATS_PERIOD_MS = 5000;

    // --- LoRa Radio (LLCC68 via RadioLib) ---
    // Must match OutComputer radio parameters exactly
    static constexpr int LORA_SPI_SCK  = 36;
    static constexpr int LORA_SPI_MISO = 34;
    static constexpr int LORA_SPI_MOSI = 35;
    static constexpr int LORA_CS_PIN   = 38;
    static constexpr int LORA_DIO1_PIN = 18;
    static constexpr int LORA_RST_PIN  = 37;
    static constexpr int LORA_BUSY_PIN = 17;

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

    // --- LoRa CSV Logging ---
    static constexpr uint32_t LOG_SILENCE_TIMEOUT_MS = 30000;   // Close file after 30s of no packets
    static constexpr uint32_t LOG_LANDED_TIMEOUT_MS  = 30000;   // Close file 30s after LANDED
    static constexpr size_t   BLE_FILE_CHUNK_SIZE    = 170;     // Bytes per BLE download chunk
    static constexpr uint32_t BLE_CHUNK_DELAY_MS     = 15;      // Delay between BLE chunks (ms)
    static constexpr size_t   FILES_PER_PAGE         = 5;       // BLE file list pagination
    static constexpr uint32_t LOG_FLUSH_INTERVAL_MS  = 10000;   // Periodic flush to flash (10s)

    // --- Battery Voltage Monitoring ---
    static constexpr int   VOLTAGE_PIN          = 8;        // ADC input pin
    static constexpr int   NUM_BATTERY_CELLS    = 2;        // Number of cells in battery
    static constexpr float R1                   = 220000.0f; // 220kΩ upper resistor
    static constexpr float R2                   = 100000.0f; // 100kΩ lower resistor
    static constexpr float VREF                 = 3.3f;      // ADC reference voltage
    static constexpr int   ADC_MAX              = 4095;      // 12-bit ADC on ESP32-S3
    static constexpr uint32_t PWR_UPDATE_PERIOD_MS = 2000;   // Battery read interval
}
