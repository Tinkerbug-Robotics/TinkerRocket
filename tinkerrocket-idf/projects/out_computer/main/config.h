#ifndef OUT_COMPUTER_CONFIG_H
#define OUT_COMPUTER_CONFIG_H

#include <stdint.h>

struct config
{
    // --- Debug ---
    static constexpr bool DEBUG = true;  // Re-enabled (needed for telemetry updates)
    static constexpr bool VERBOSE_DEBUG = false;  // Temporarily disabled to see BLE output
    static constexpr uint32_t STATS_PERIOD_MS = 1000;

    // --- Power rail switch ---
    static constexpr int PWR_PIN = 6;

    // --- Power Monitoring ---
    static constexpr int PWR_SDA = 7;
    static constexpr int PWR_SCL = 8;

    // --- Memory Shared SPI bus pins ---
    static constexpr int SPI_SCK = 37;
    static constexpr int SPI_MISO = 35;
    static constexpr int SPI_MOSI = 38;

    // --- NAND chip select ---
    static constexpr int NAND_CS = 36;

    // --- MRAM chip select (MR25H10 on shared SPI bus) ---
    // Enabled: 128 KB non-volatile ring buffer survives hard resets.
    // SPI bus mutex prevents contention between Core 1 (ring push) and
    // Core 0 (NAND flush).  On dirty startup, MRAM is drained to a
    // recovery file before clearing.  Set to -1 to fall back to RAM ring.
    static constexpr int MRAM_CS = 34;
    static constexpr uint32_t MRAM_SIZE = 131072;       // 128 KB
    static constexpr uint32_t SPI_HZ_MRAM = 40'000'000;
    static constexpr uint8_t SPI_MODE_MRAM = SPI_MODE0;

    // FlightSnapshot region (#104 follow-up): top 1 KB of MRAM is reserved
    // for the latest FC FlightSnapshotData (~224 B wire frame).  The log
    // ring uses the remaining MRAM_SIZE - SNAPSHOT_REGION_SIZE bytes.
    // Single-slot writes serialized on the SPI bus mutex (write ~700 us;
    // reads on the same mutex never observe a partial write).
    static constexpr uint32_t SNAPSHOT_REGION_SIZE = 1024;
    static constexpr uint32_t SNAPSHOT_REGION_BASE = MRAM_SIZE - SNAPSHOT_REGION_SIZE;

    // --- SPI speeds/modes ---
    static constexpr uint32_t SPI_HZ_NAND = 40'000'000;
    static constexpr uint8_t SPI_MODE_NAND = SPI_MODE0;

    // --- RAM ring buffer (used when MRAM_CS = -1) ---
    // Fallback if MRAM not available. MRAM is preferred (128 KB, no heap cost).
    static constexpr uint32_t RAM_RING_SIZE = 65536;  // 64 KB fallback

    // --- I2C from FlightComputer -> OutComputer ---
    static constexpr uint8_t I2C_ADDRESS = 0x42;
    static constexpr int I2C_SDA_PIN = 4;
    static constexpr int I2C_SCL_PIN = 5;
    static constexpr uint32_t I2C_CLOCK_HZ = 1'200'000;
    static constexpr size_t I2C_SLAVE_RX_BUF = 8192;
    static constexpr size_t I2C_SLAVE_TX_BUF = 256;
    // Time-slice ingress to prevent loop starvation under sustained traffic.
    static constexpr uint32_t I2C_INGRESS_BUDGET_US = 1500;
    static constexpr size_t I2C_INGRESS_BUDGET_BYTES = 8192;

    // --- I2S (high-frequency telemetry RX from FlightComputer) ---
    // Actual wiring: FC 27→OC 21, FC 28→OC 45, FC 23→OC 2, FC 17→OC 1
    static constexpr int I2S_BCLK_PIN  = 21;
    static constexpr int I2S_WS_PIN    = 45;
    static constexpr int I2S_DIN_PIN   = 2;
    static constexpr int I2S_FSYNC_PIN = 1;
    // I2S bandwidth = sample_rate * 4 bytes (16-bit stereo).
    // Higher rate = faster DMA buffer turnover = less stale data.
    // 22050 Hz = 88 KB/s.  Lower rates cause more gaps from DMA replay.
    // IMPORTANT: If sensor rates increase, raise this proportionally.
    static constexpr uint32_t I2S_SAMPLE_RATE = 22050;  // Must match FC

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

    // Known-good rendezvous mode (issue #71).  When the rocket and base
    // station drift apart, both fall back to this complete LoRa config
    // — frequency AND modulation parameters.  Frequency alone isn't
    // enough: the user can configure SF/BW/CR/TX power via the iOS app
    // ("Fast" / "Standard" / "Long Range" presets), and a divergence on
    // those is just as fatal as a frequency mismatch.  All five values
    // must be identical on both firmware builds; the constants below are
    // the "Standard" preset, which is also the NVS factory default — so
    // a fresh-NVS device is already in rendezvous mode at boot.  Never
    // persisted to NVS; NVS holds the working config and this is always
    // the same compile-time fallback.
    static constexpr float   LORA_RENDEZVOUS_MHZ          = 915.0f;
    static constexpr uint8_t LORA_RENDEZVOUS_SF           = 8;
    static constexpr float   LORA_RENDEZVOUS_BW_KHZ       = 250.0f;
    static constexpr uint8_t LORA_RENDEZVOUS_CR           = 5;
    static constexpr int8_t  LORA_RENDEZVOUS_TX_POWER_DBM = 12;

    // --- LoRa Uplink RX (commands from BaseStation) ---
    static constexpr uint8_t UPLINK_SYNC_BYTE = 0xCA;

    // --- Device Identity (factory defaults, overridden by NVS on boot) ---
    static constexpr uint8_t DEFAULT_NETWORK_ID = 0;
    static constexpr uint8_t DEFAULT_ROCKET_ID  = 1;
    static constexpr const char* DEVICE_TYPE     = "R";  // "R" = rocket
};

#endif
