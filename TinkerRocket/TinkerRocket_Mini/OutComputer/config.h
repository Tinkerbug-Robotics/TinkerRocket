#ifndef OUT_COMPUTER_CONFIG_H
#define OUT_COMPUTER_CONFIG_H

#include <driver/i2c.h>

struct config
{
    // --- Debug ---
    static constexpr bool DEBUG = true;  // Re-enabled (needed for telemetry updates)
    static constexpr bool VERBOSE_DEBUG = false;  // Temporarily disabled to see BLE output
    static constexpr uint32_t STATS_PERIOD_MS = 1000;

    // --- Power rail switch ---
    static constexpr int PWR_PIN = 6;

    // --- Shared SPI bus pins ---
    static constexpr int SPI_SCK = 37;
    static constexpr int SPI_MISO = 35;
    static constexpr int SPI_MOSI = 38;

    // --- NAND chip select ---
    static constexpr int NAND_CS = 36;

    // --- MRAM chip select (MR25H10 on shared SPI bus) ---
    static constexpr int MRAM_CS = 34;
    static constexpr uint32_t MRAM_SIZE = 131072;       // 128 KB
    static constexpr uint32_t SPI_HZ_MRAM = 40'000'000;
    static constexpr uint8_t SPI_MODE_MRAM = SPI_MODE0;

    // --- SPI speeds/modes ---
    static constexpr uint32_t SPI_HZ_NAND = 40'000'000;
    static constexpr uint8_t SPI_MODE_NAND = SPI_MODE0;

    // --- RAM ring buffer (fallback if MRAM disabled) ---
    static constexpr uint32_t RAM_RING_SIZE = 65536;  // 64 KB

    // --- I2C from FlightComputer -> OutComputer ---
    static constexpr i2c_port_t I2C_PORT = I2C_NUM_0;
    static constexpr uint8_t I2C_ADDRESS = 0x42;
    static constexpr int I2C_SDA_PIN = 4;
    static constexpr int I2C_SCL_PIN = 5;
    static constexpr uint32_t I2C_CLOCK_HZ = 1'200'000;
    static constexpr size_t I2C_SLAVE_RX_BUF = 8192;
    static constexpr size_t I2C_SLAVE_TX_BUF = 256;
    // Time-slice ingress to prevent loop starvation under sustained traffic.
    static constexpr uint32_t I2C_INGRESS_BUDGET_US = 1500;
    static constexpr size_t I2C_INGRESS_BUDGET_BYTES = 8192;

    // --- LoRa (LLCC68 via RadioLib) ---
    static constexpr bool USE_LORA_RADIO = true;
    static constexpr int LORA_SPI_SCK = 14;
    static constexpr int LORA_SPI_MISO = 11;
    static constexpr int LORA_SPI_MOSI = 12;
    static constexpr int LORA_CS_PIN = 18;
    static constexpr int LORA_DIO1_PIN = 9;
    static constexpr int LORA_RST_PIN = 17;
    static constexpr int LORA_BUSY_PIN = 13;
    static constexpr float LORA_FREQ_MHZ = 915.0f;
    static constexpr uint8_t LORA_SF = 8;
    static constexpr float LORA_BW_KHZ = 250.0f;
    static constexpr uint8_t LORA_CR = 5;
    static constexpr uint16_t LORA_PREAMBLE_LEN = 12;
    static constexpr int8_t LORA_TX_POWER_DBM = 12;
    static constexpr bool LORA_CRC_ON = true;
    static constexpr bool LORA_RX_BOOSTED_GAIN = true;
    static constexpr bool LORA_SYNCWORD_PRIVATE = true;
    static constexpr uint16_t LORA_TX_RATE_HZ = 2;

    // --- LoRa Uplink RX (commands from BaseStation) ---
    static constexpr uint8_t UPLINK_SYNC_BYTE = 0xCA;
};

#endif
