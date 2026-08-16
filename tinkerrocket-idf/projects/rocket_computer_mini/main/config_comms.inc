// config_comms.inc — comms-side policy constants for the rocket-computer-mini.
//
// Struct-body fragment: bare `static constexpr` members, textually included
// into `struct config : board_pins` by config.h.  Every constant here descends
// from projects/out_computer/main/config.h (line references cite that file at
// the fork point) unless marked MINI-NEW.  Ported comms code references these
// as config::NAME and must compile unchanged — keep the names identical to the
// OC's.  Pins live in board_v1.h, never here.
//
// NOT carried from the OC (hardware/link gone on the mini): MRAM_* /
// SNAPSHOT_REGION_* / MRAM_DIRTY_MARKER_ADDR (no MRAM fitted, mram_cs=-1 RAM
// ring fallback), I2C_* slave link + I2S_SAMPLE_RATE (no second MCU), the
// LORA_RENDEZVOUS_* aliases (the ported code calls the shared
// LORA_FACTORY_RENDEZVOUS_* constants from RocketComputerTypes.h directly).

// --- Debug (OC config.h:24-30) ---
static constexpr bool DEBUG = true;
static constexpr bool VERBOSE_DEBUG = false;
static constexpr uint32_t STATS_PERIOD_MS = 1000;
// #398: dump per-task CPU utilization once per stats interval (the "TASKCPU"
// line).  On the mini this is how the merged one-chip flight+comms budget is
// measured on the bench rather than guessed (see sdkconfig.defaults).
static constexpr bool PROFILE_TASK_CPU = true;

// --- SPI speeds/modes (OC config.h:55-56) ---
static constexpr uint32_t SPI_HZ_NAND = 40'000'000;
static constexpr uint8_t SPI_MODE_NAND = 0;  // SPI_MODE0 (literal: compat.h not visible here)

// --- Log RAM ring buffer (OC config.h:60, resized) ---
// The OC used 64 KB as a fallback behind a preferred 128 KB non-volatile MRAM
// ring.  The mini has no MRAM (board_pins::MRAM_CS = -1), so this heap ring is
// the ONLY brownout buffer between the flight stream and the NAND flush task.
// Sized to the 128 KB the MRAM ring gave the OC: the S3 has 512 KB SRAM, PSRAM
// is deliberately OFF (sdkconfig.defaults), and the mini carries none of the
// OC's I2S DMA rings / 64 KB rx ring / OTA feeder queue — ample headroom.
// ~1.3 s of the ~99 KB/s peak log stream.
static constexpr uint32_t RAM_RING_SIZE = 131072;

// --- LoRa RF parameters (OC config.h:82-91) ---
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

// --- LoRa Uplink RX (OC config.h:104) ---
static constexpr uint8_t UPLINK_SYNC_BYTE = 0xCA;

// --- Device Identity (OC config.h:107-109; NVS overrides at boot) ---
static constexpr uint8_t DEFAULT_NETWORK_ID = 0;
static constexpr uint8_t DEFAULT_ROCKET_ID  = 1;
// Deliberately the OC's exact string: the app keys device behaviour off
// "dt" in config_identity, and the mini must keep existing apps working.
// The board is identifiable via "fw" (PROJECT_VER carries the project name's
// git sha + board suffix) and the boot log.
static constexpr const char* DEVICE_TYPE = "R";  // "R" = rocket

// --- Peripheral rail timing (MINI-NEW; no OC equivalent) ---
// On the OC, PWR_PIN gated a second processor; on the mini it gates
// V_MCU_SWTCH powering IMU+baro+mag+GNSS+LoRa+NAND (board_v1.h).
//
// Minimum rail-OFF dwell before it may be re-raised: the Quectel LC86G
// requires VCC below 100 mV for >= 1 s for a clean restart — a shorter dip
// can leave the receiver wedged, and this board has no GNSS reset line
// (recovery is ONLY via this rail).  1200 ms = 1 s + margin for the TPS22810
// QOD discharge tail.  Enforced across the power-off -> esp_restart -> boot ->
// power-on path by gating power-on on millis() >= this value.
static constexpr uint32_t PERIPH_RAIL_OFF_MIN_MS = 1200;
// Rail settle after PWR_PIN goes high, before any bus traffic: TPS22810 rise
// + sensor/NAND/radio POR.  Spec floor is 50 ms; kept at the OC's
// field-proven 500 ms (out_computer main.cpp L7068) — power-on is a
// human-paced ground operation, cheap insurance is free here.
static constexpr uint32_t RAIL_SETTLE_MS = 500;
