#ifndef OUT_COMPUTER_CONFIG_H
#define OUT_COMPUTER_CONFIG_H

#include <stdint.h>
#include <RocketComputerTypes.h>  // LORA_FACTORY_RENDEZVOUS_* (#105)

// --- Board revision (#411, tracking #408) ---
// Pins + peripheral-presence/topology flags live in the per-board headers;
// everything else in this file is board-independent policy and must not
// fork per revision. Select the V8 map with: idf.py -DTR_BOARD_V8=1 build
// (default stays V7 until V8 bring-up completes).
#ifndef TR_BOARD_V8
#define TR_BOARD_V8 0
#endif
#if TR_BOARD_V8
#include "board/board_v8.h"
#else
#include "board/board_v7.h"
#endif

struct config : board_pins
{
    // --- Debug ---
    static constexpr bool DEBUG = true;  // Re-enabled (needed for telemetry updates)
    static constexpr bool VERBOSE_DEBUG = false;  // Temporarily disabled to see BLE output
    static constexpr uint32_t STATS_PERIOD_MS = 1000;

    // --- MRAM (MR25H10 on shared SPI bus; CS pin in board header) ---
    // Enabled: 128 KB non-volatile ring buffer survives hard resets.
    // SPI bus mutex prevents contention between Core 1 (ring push) and
    // Core 0 (NAND flush).  On dirty startup, MRAM is drained to a
    // recovery file before clearing.  MRAM_CS = -1 falls back to RAM ring.
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
    // #274: 4-byte sink-mode "dirty" marker, parked at the free top of the
    // snapshot region (FlightSnapshot uses ~232 B up from SNAPSHOT_REGION_BASE).
    // Set while a log session is open; a surviving magic on the next boot means
    // an unclean shutdown left unflushed frames in the non-volatile MRAM ring.
    static constexpr uint32_t MRAM_DIRTY_MARKER_ADDR = MRAM_SIZE - 16;

    // --- SPI speeds/modes ---
    static constexpr uint32_t SPI_HZ_NAND = 40'000'000;
    static constexpr uint8_t SPI_MODE_NAND = SPI_MODE0;

    // --- RAM ring buffer (used when MRAM_CS = -1) ---
    // Fallback if MRAM not available. MRAM is preferred (128 KB, no heap cost).
    static constexpr uint32_t RAM_RING_SIZE = 65536;  // 64 KB fallback

    // --- I2C from FlightComputer -> OutComputer (pins in board header) ---
    static constexpr uint8_t I2C_ADDRESS = 0x42;
    static constexpr uint32_t I2C_CLOCK_HZ = 1'200'000;
    static constexpr size_t I2C_SLAVE_RX_BUF = 8192;
    static constexpr size_t I2C_SLAVE_TX_BUF = 256;
    // Time-slice ingress to prevent loop starvation under sustained traffic.
    static constexpr uint32_t I2C_INGRESS_BUDGET_US = 1500;
    static constexpr size_t I2C_INGRESS_BUDGET_BYTES = 8192;

    // --- I2S (high-frequency telemetry RX from FlightComputer; pins in
    //     board header) ---
    // I2S bandwidth = sample_rate * 4 bytes (16-bit stereo).
    // Higher rate = faster DMA buffer turnover = less stale data.
    // 22050 Hz = 88 KB/s.  Lower rates cause more gaps from DMA replay.
    // IMPORTANT: If sensor rates increase, raise this proportionally.
    static constexpr uint32_t I2S_SAMPLE_RATE = 22050;  // Must match FC

    // --- LoRa RF parameters (radio presence + pins in board header) ---
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

    // Known-good rendezvous mode (issue #71).  Sourced from the shared
    // RocketComputerTypes.h header (#105) so both BS and OC firmware are
    // guaranteed to agree at compile time — see LORA_FACTORY_RENDEZVOUS_*
    // there for the full justification.
    static constexpr float   LORA_RENDEZVOUS_MHZ          = LORA_FACTORY_RENDEZVOUS_MHZ;
    static constexpr uint8_t LORA_RENDEZVOUS_SF           = LORA_FACTORY_RENDEZVOUS_SF;
    static constexpr float   LORA_RENDEZVOUS_BW_KHZ       = LORA_FACTORY_RENDEZVOUS_BW_KHZ;
    static constexpr uint8_t LORA_RENDEZVOUS_CR           = LORA_FACTORY_RENDEZVOUS_CR;
    static constexpr int8_t  LORA_RENDEZVOUS_TX_POWER_DBM = LORA_FACTORY_RENDEZVOUS_TX_DBM;

    // --- LoRa Uplink RX (commands from BaseStation) ---
    static constexpr uint8_t UPLINK_SYNC_BYTE = 0xCA;

    // --- Device Identity (factory defaults, overridden by NVS on boot) ---
    static constexpr uint8_t DEFAULT_NETWORK_ID = 0;
    static constexpr uint8_t DEFAULT_ROCKET_ID  = 1;
    static constexpr const char* DEVICE_TYPE     = "R";  // "R" = rocket
};

#endif
