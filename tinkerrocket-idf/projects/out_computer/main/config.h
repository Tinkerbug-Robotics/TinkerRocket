#ifndef OUT_COMPUTER_CONFIG_H
#define OUT_COMPUTER_CONFIG_H

#include <stdint.h>
#include <RocketComputerTypes.h>  // LORA_FACTORY_RENDEZVOUS_* (#105)

// --- Board revision (#411, tracking #408) ---
// Pins + peripheral-presence/topology flags live in the per-board headers;
// everything else in this file is board-independent policy and must not
// fork per revision. Select the V8 map with: idf.py -DTR_BOARD_V8=1 build
// (default stays V7 until V8 bring-up completes).
//
// TR_BOARD_V9=1 selects board_v9.h (V9/V10). The S3's *pin* map is unchanged
// from V8 through V10 — unlike the FC, where V9 moved three pyro pins — so
// this used to be an alias to board_v8.h. #822: it isn't a pin change that
// forced the split, it's a deleted part. V9 removed the MRAM (U12), and an
// alias is presence-blind: it can say "same pins" but not "one fewer device",
// so V9/V10 firmware kept driving an MRAM_CS that goes nowhere and backed the
// log ring + reboot snapshot with an unfitted chip. The two headers differ in
// exactly one value, MRAM_CS (34 vs -1); keep every other pin in step by hand.
#ifndef TR_BOARD_V8
#define TR_BOARD_V8 0
#endif
#ifndef TR_BOARD_V9
#define TR_BOARD_V9 0
#endif
#if TR_BOARD_V8
#include "board/board_v8.h"
#elif TR_BOARD_V9
#include "board/board_v9.h"
#else
#include "board/board_v7.h"
#endif

// The one difference between the V8 and V9 maps, asserted rather than trusted.
// Nothing at runtime notices a wrong MRAM_CS: with 34 on a V9 board the ring
// and the snapshot slot write to a floating pad and report success, and with
// -1 on a V8 board the fitted MRAM simply goes unused. Both are silent. So pin
// it here — re-aliasing the V9 branch back to board_v8.h, or editing MRAM_CS
// in the wrong header, then fails the build instead of the flight.
#if TR_BOARD_V9
static_assert(board_pins::MRAM_CS < 0,
              "V9/V10 deleted the MRAM (U12) — board_v9.h must keep MRAM_CS = -1 (#822)");
#elif TR_BOARD_V8
static_assert(board_pins::MRAM_CS == 34,
              "V8 bench boards have the MRAM fitted on GPIO34 — do not change this (#822)");
#endif

struct config : board_pins
{
    // --- Debug ---
    static constexpr bool DEBUG = true;  // Re-enabled (needed for telemetry updates)
    static constexpr bool VERBOSE_DEBUG = false;  // Temporarily disabled to see BLE output
    static constexpr uint32_t STATS_PERIOD_MS = 1000;
    // #398: dump per-task CPU utilization once per stats interval (the "TASKCPU"
    // line) to identify the core-1 hog during the launch-activation window.
    // Requires the run-time-stats sdkconfig flags; set false to silence.
    static constexpr bool PROFILE_TASK_CPU = true;

    // --- MRAM (MR25H10 on shared SPI bus; CS pin in board header) ---
    // V7/V8 only.  Enabled: 128 KB non-volatile ring buffer survives hard
    // resets.  SPI bus mutex prevents contention between Core 1 (ring push)
    // and Core 0 (NAND flush).  On dirty startup, MRAM is drained to a
    // recovery file before clearing.  MRAM_CS = -1 falls back to RAM ring.
    //
    // V9/V10 have no MRAM (board_v9.h, MRAM_CS = -1), so everything below
    // that is measured from MRAM_SIZE — the snapshot region and the #274
    // dirty marker — addresses a device that isn't there.  The constants stay
    // (they are the V7/V8 geometry, and TR_LogToFlash ignores them outright
    // when use_mram_ is false), but do NOT read them as "the OC always has a
    // non-volatile snapshot store".  It doesn't; see initPeripherals().
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
    // 44100 Hz = 176 KB/s.  Raised from 22050 with the FC's IMU 960 -> 1920 Hz
    // logging step (the link was ~76% full at 22050).  RX DMA descriptors are
    // sized in setup (dma_frame_num) to keep the callback cadence ~3 ms at
    // this rate.  IMPORTANT: must match the FC — flash both together.
    static constexpr uint32_t I2S_SAMPLE_RATE = 44100;  // Must match FC

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
