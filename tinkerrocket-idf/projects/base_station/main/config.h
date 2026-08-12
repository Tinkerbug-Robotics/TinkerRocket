#pragma once

#include <cstdint>
#include <RocketComputerTypes.h>  // LORA_FACTORY_RENDEZVOUS_* (#105)

// --- Board revision (OC #411 pattern) ---
// Pins + peripheral-presence/topology flags live in the per-board headers
// (main/board/board_v{1,2,3}.h); everything below is board-independent policy
// and must not fork per revision. Select with a separate build dir per
// variant (the flag is cached by CMake):
//   V2 (default): idf.py build
//   V1 original:  idf.py -B build_v1 -DTR_BS_BOARD=1 build
//   V3 new PCB:   idf.py -B build_v3 -DTR_BS_BOARD=3 build
// The fuel gauge is still auto-detected at runtime on the board's I2C bus
// (BQ27Z746 0x55, then MAX17205/MAX17303 0x36 split by DevName), so it does
// NOT depend on this switch. Pins the silicon can't probe (LoRa topology,
// storage, the I2C bus itself) are compile-time.
#ifndef TR_BS_BOARD
#define TR_BS_BOARD 2
#endif
#if TR_BS_BOARD == 3
#include "board/board_v3.h"
#elif TR_BS_BOARD == 2
#include "board/board_v2.h"
#elif TR_BS_BOARD == 1
#include "board/board_v1.h"
#else
#error "TR_BS_BOARD must be 1, 2, or 3"
#endif

struct config : board_pins
{
    // --- Debug ---
    static constexpr bool DEBUG = true;
    static constexpr uint32_t STATS_PERIOD_MS = 5000;

    // --- LoRa Radio (LLCC68 via RadioLib, or the UART radio daughterboard) ---
    // Must match OutComputer radio parameters exactly. Pins + topology
    // (direct SPI vs UART modem) are in the board headers.
    static constexpr float   LORA_FREQ_MHZ       = 915.0f;
    static constexpr uint8_t LORA_SF              = 8;
    static constexpr float   LORA_BW_KHZ          = 250.0f;
    static constexpr uint8_t LORA_CR              = 5;
    static constexpr uint16_t LORA_PREAMBLE_LEN   = 12;
    static constexpr int8_t  LORA_TX_POWER_DBM    = 12;
    static constexpr bool    LORA_CRC_ON          = true;
    static constexpr bool    LORA_RX_BOOSTED_GAIN = true;
    static constexpr bool    LORA_SYNCWORD_PRIVATE = true;

    // Known-good rendezvous mode (issue #71).  Sourced from the shared
    // RocketComputerTypes.h header (#105) so both BS and OC firmware are
    // guaranteed to agree at compile time — see LORA_FACTORY_RENDEZVOUS_*
    // there for the full justification.
    static constexpr float   LORA_RENDEZVOUS_MHZ          = LORA_FACTORY_RENDEZVOUS_MHZ;
    static constexpr uint8_t LORA_RENDEZVOUS_SF           = LORA_FACTORY_RENDEZVOUS_SF;
    static constexpr float   LORA_RENDEZVOUS_BW_KHZ       = LORA_FACTORY_RENDEZVOUS_BW_KHZ;
    static constexpr uint8_t LORA_RENDEZVOUS_CR           = LORA_FACTORY_RENDEZVOUS_CR;
    static constexpr int8_t  LORA_RENDEZVOUS_TX_POWER_DBM = LORA_FACTORY_RENDEZVOUS_TX_DBM;

    // --- LoRa Uplink (BaseStation → OutComputer) ---
    static constexpr uint8_t  UPLINK_SYNC_BYTE        = 0xCA;
    static constexpr uint8_t  UPLINK_RETRIES           = 8;     // TX attempts per command
    static constexpr uint32_t UPLINK_RETRY_INTERVAL_MS = 100;   // Delay between retries
    // #565: consecutive send() FAILURES (startTransmit error, nothing on air)
    // before the head command is dropped. Failed attempts are paced at
    // UPLINK_RETRY_INTERVAL_MS, so a wedged radio holds each command at most
    // ~2 s (20 x 100 ms) instead of jamming the queue until reboot. Generous
    // enough that a transient SPI/radio hiccup never costs a command.
    static constexpr uint8_t  UPLINK_MAX_SEND_FAILURES = 20;

    // --- Uplink TX window (#506) ---
    // The radio is half-duplex, so every uplink retry is a deaf window. At
    // SF8/BW250 a downlink packet is ~82 ms on air while the gaps between retries
    // are only ~49 ms — an 82 ms packet cannot fit in a 49 ms gap, so a blind
    // burst loses essentially EVERY packet that arrives during it (bench-measured:
    // 3 of 3). Rather than firing blind, transmit inside the quiet stretch between
    // the rocket's ~500 ms telemetry packets. See bs_uplink_txwin.h.
    //
    // Air to keep clear ahead of the next expected downlink packet: its
    // time-on-air plus this margin for cadence jitter and TX/RX turnaround.
    //
    // #150 bench root-cause (dwell-1 uplink misses): the reserve used to be a
    // flat 140 ms — the SF8 downlink's ~82 ms plus margin baked into one
    // number.  At SF9 the downlink is ~203 ms, so the flat reserve sanctioned
    // uplink attempts that overlapped the head of the rocket's next TX; the
    // resulting half-duplex collision blinded the BS to that packet, and at
    // dwell-1 (channel change every packet) losing one packet breaks the
    // hop-follow — the rest of the retry burst then fired onto stale
    // channels (two full 8-retry cmd-17 bursts lost on the bench, ~50%
    // heartbeat delivery).  serviceUplink now computes the reserve from the
    // ACTUAL telemetry airtime at the live modulation and adds this margin.
    static constexpr uint32_t UPLINK_RX_RESERVE_MARGIN_MS = 60;
    // No packet for this long => no downlink worth protecting; transmit freely.
    // Also the case where we most need to (silence recovery, rendezvous).
    static constexpr uint32_t UPLINK_LINK_STALE_MS = 2000;
    // Liveness backstop: never let the window gate starve a command. The uplink is
    // blind and carries safety-relevant commands, so a command that never goes out
    // is far worse than a dropped telemetry row.
    static constexpr uint32_t UPLINK_MAX_DEFER_MS  = 1500;

    // --- Storage ---
    // V2/V3 log to a FORESEE F35SQB004G SPI NAND (M_* nets, on SPI3_HOST since
    // LoRa owns SPI2_HOST on V2); V1 uses an SDMMC SD card. Pins + presence
    // flags (HAS_EXT_NAND / HAS_SDMMC) are in the board headers. Both mount as
    // FAT behind SD_MOUNT_POINT, so the logging code is backend-agnostic.

    // NAND bring-up (#761).  A single failed probe used to demote the whole
    // boot to the ~1.9 MB internal SPIFFS partition — 512 MB of flight log
    // traded for 1.9 MB on one transient error, and a flight logged there
    // truncates.  Retry the full bring-up (bus -> device -> reset -> probe ->
    // FAT) a bounded number of times before giving up.  Kept short on purpose:
    // the worst case adds well under half a second to boot.
    static constexpr uint8_t  NAND_MOUNT_ATTEMPTS       = 3;
    static constexpr uint32_t NAND_MOUNT_RETRY_DELAY_MS = 25;
    // Settle time before the first command of an attempt.  Covers tPOR on a
    // genuinely cold rail; on a warm boot it is just cheap insurance.
    static constexpr uint32_t NAND_SETTLE_MS            = 5;
    // Ceiling on the post-RESET ready poll.  Worst-case tRST on this class of
    // part (RESET issued during a block erase) is sub-millisecond, so this is
    // ~100x margin — sized so an absent chip holding MISO high fails fast
    // instead of stalling the boot.
    static constexpr uint32_t NAND_READY_TIMEOUT_MS     = 50;
    // Bring-up SPI clock. 20 MHz is already deliberately conservative for a
    // part rated 166 MHz. The last attempt drops to half that: if a unit only
    // ever comes up on the fallback clock, the answer is signal integrity, not
    // timing — and half-speed NAND still beats a 1.9 MB partition.
    static constexpr uint32_t NAND_CLOCK_HZ             = 20 * 1000 * 1000;
    static constexpr uint32_t NAND_CLOCK_FALLBACK_HZ    = 10 * 1000 * 1000;

    // --- LoRa CSV Logging ---
    // Close the log after 5 min of no rocket packets.  Long enough to ride
    // through the worst-case altitude RX dropout on an Estes-class flight
    // (signal back during descent) and a typical high-power coast-to-apogee,
    // short enough that bs_logging_active doesn't stick true forever after
    // the rocket is powered off post-recovery.  Pre-#137 this was 30 s
    // which closed mid-flight on any altitude-driven silence.
    static constexpr uint32_t LOG_SILENCE_TIMEOUT_MS  = 5 * 60 * 1000;
    static constexpr uint32_t LOG_INFLIGHT_SAFETY_MS  = 20 * 60 * 1000; // Safety: close log 20 min after INFLIGHT entry if LANDED never seen (#107)
    static constexpr size_t   BLE_FILE_CHUNK_SIZE    = 170;     // Bytes per BLE download chunk
    static constexpr uint32_t BLE_CHUNK_DELAY_MS     = 15;      // Delay between BLE chunks (ms)
    static constexpr size_t   FILES_PER_PAGE         = 5;       // BLE file list pagination
    static constexpr uint32_t LOG_FLUSH_INTERVAL_MS  = 10000;   // Periodic flush to flash (10s)
    static constexpr uint32_t LOG_OPEN_RETRY_MS      = 5000;    // Retry fopen() at most this often when a start attempt fails (#107)

    // --- BLE telemetry staleness (#95) ---
    // The BS periodic push re-sends the most recent rocket telemetry every
    // PWR_UPDATE_PERIOD_MS so battery / RSSI stay live even between LoRa
    // packets.  Once the underlying packet is older than BLE_TELEMETRY_STALE_MS
    // we mark the BLE payload as stale (with age) so the iOS app can dim
    // the values + show a "stale (Ns ago)" badge instead of pretending the
    // numbers are fresh.  Sized just above the slowest steady-state packet
    // cadence (Long Range preset ≈ 2 Hz) plus a couple of misses.
    static constexpr uint32_t BLE_TELEMETRY_STALE_MS = 3000;

    // --- I2C Bus (fuel gauge + V3 pack charger; pins in board headers) ---
    // External pull-ups on-board (internal OFF).
    static constexpr uint32_t I2C_FREQ_HZ       = 400'000;  // 400 kHz

    // --- Device Identity (factory defaults, overridden by NVS on boot) ---
    static constexpr uint8_t DEFAULT_NETWORK_ID = 0;
    static constexpr const char* DEVICE_TYPE     = "B";  // "B" = base station

    // --- Battery Monitoring ---
    // Fuel gauge is auto-detected at runtime on the board's I2C bus:
    // BQ27Z746 (V2) probed first at 0x55, then 0x36 — where DevName picks
    // MAX17303 (V3, 1S + protector) vs MAX17205 (V1, 2S). All addresses are
    // always defined so one firmware image covers every board's bus. The
    // seed values (cells/Rsense/design mAh) below feed the MAX drivers;
    // the BQ27Z746 self-gauges from its own data-flash config.
    static constexpr uint16_t MAX17205_ADDR      = 0x36;     // V1 gauge
    static constexpr uint16_t BQ27Z746_ADDR      = 0x55;     // V2 gauge
    static constexpr uint16_t MAX17303_ADDR      = 0x36;     // V3 gauge (m5 family
                                                              //   address, same as
                                                              //   MAX17205 — DevName
                                                              //   picks the driver)
    static constexpr int      NUM_BATTERY_CELLS  = 2;        // 2S NCR18650B (original PCB / MAX path)
    static constexpr float    RSENSE_MOHM        = 10.0f;    // Sense resistor (mΩ)
    static constexpr uint32_t PWR_UPDATE_PERIOD_MS = 2000;   // Battery read interval
    // Pack design capacity in mAh. Two 2800 mAh 18650 cells in 2S = 2800 mAh
    // (series doubles voltage, capacity stays per-cell). Written to MAX17205
    // DesignCap at boot to seed the ModelGauge m5 algorithm.
    static constexpr uint16_t BATTERY_DESIGN_MAH = 2800;

    // --- Voltage-based SoC fallback (#501) ---
    // The BQ27Z746 is a 1S gauge, so its Voltage() IS the cell voltage.
    static constexpr int   BQ27Z746_CELLS = 1;
    // Pack internal resistance (Ω) used to back out IR offset from the terminal
    // voltage. ZERO = compensation DISABLED, which is deliberate: the gauge's CC
    // Gain (data flash 0x4006) has never been calibrated against a known current,
    // so its current reading would inject a scaling error rather than remove one.
    // Set this once CC Gain is calibrated — the path is implemented and tested.
    static constexpr float BATTERY_INTERNAL_R_OHM = 0.0f;
    // EMA weight per battery poll (PWR_UPDATE_PERIOD_MS = 2 s) → ~20 s time
    // constant: rides out LoRa TX current spikes, still tracks a real discharge.
    static constexpr float SOC_FILTER_ALPHA = 0.1f;

    // --- Flight-pack charger (MP2672 on V3; gated by HAS_PACK_CHARGER) ---
    static constexpr uint16_t MP2672_ADDR = 0x4B;
    // Fast-charge current code (REG01 ICC[3:0]): I = FS*(5+code)/20 with
    // FS = 12 kΩ / RISET amps — the current scale is set by the board's
    // RISET resistor, not just this register. Code 0 = the selectable
    // minimum (25% of full scale); raise only after RISET is confirmed on
    // the bench, then size for ≲0.5C of the smallest flight pack.
    static constexpr uint8_t PACK_CHARGE_ICC_CODE = 0;
    static constexpr uint8_t PACK_VBATT_REG_CODE  = 1;   // 8.4 V pack (4.2 V/cell) — LiPo max
    static constexpr uint8_t PACK_CHG_TIMER_CODE  = 1;   // 8 h fast-charge safety timer
};
