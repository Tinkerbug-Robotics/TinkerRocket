#pragma once

// V3 base-station PCB pin map + board topology.
// Selected with TR_BS_BOARD=3:  idf.py -B build_v3 -DTR_BS_BOARD=3 build
//
// What changed vs V2:
//  - Fuel gauge: MAX17303 (1S, ModelGauge m5 + protector) replaces the
//    BQ27Z746. Runtime-probed at 0x36 and disambiguated from the V1
//    MAX17205 (same address) via DevName.
//  - NEW: MP2672 2-cell charger for external rocket flight packs (same I2C
//    bus as the gauge).
//  - I2C moves to SDA=33 / SCL=34 (33/34 were LoRa RST / unused on V2).
//  - Radio: no on-board LLCC68 — the V8 radio daughterboard (#409,
//    projects/radio_board) connects over UART as a transparent modem,
//    same topology as the V8 out computer (#410).
//  - Storage: unchanged from V2 — FORESEE F35SQB004G SPI NAND on the same
//    M_* nets/pins.
struct board_pins
{
    static constexpr const char* BOARD_NAME = "V3 (MAX17303 + MP2672 + radio daughterboard)";

    // --- LoRa radio: UART daughterboard (#409) — no direct LLCC68 ---
    static constexpr bool USE_UART_RADIO_MODEM = true;
    static constexpr int LORA_SPI_SCK  = -1;
    static constexpr int LORA_SPI_MISO = -1;
    static constexpr int LORA_SPI_MOSI = -1;
    static constexpr int LORA_CS_PIN   = -1;
    static constexpr int LORA_DIO1_PIN = -1;
    static constexpr int LORA_RST_PIN  = -1;
    static constexpr int LORA_BUSY_PIN = -1;
    static constexpr int LORA_RXEN_PIN = -1;
    static constexpr int LORA_DIO2_PIN = -1;
    static constexpr int LORA_DIO3_PIN = -1;

    // --- Radio daughterboard host link ---
    // Schematic labels LoRa_TX (GPIO35) / LoRa_RX (GPIO36): wired as-labeled
    // (TX net = BS transmit), matching the OC V8 convention. CAUTION: TX/RX
    // label perspective has been wrong on this schematic family before
    // (RunCam, #234) — if the daughterboard's BOOT message never arrives on
    // bring-up, swap these two first.
    static constexpr int LORA_UART_TX_PIN = 35;  // LoRa_TX (label-perspective)
    static constexpr int LORA_UART_RX_PIN = 36;  // LoRa_RX (label-perspective)
    // No daughterboard power-gate net identified on the V3 schematic (the OC
    // V8 has LoRa_ACT); set if one exists — it's also the recovery hammer for
    // a wedged daughterboard (#412).
    static constexpr int LORA_ACT_PIN = -1;

    // --- Storage: external SPI NAND (FORESEE F35SQB004G), same as V2 ---
    static constexpr bool HAS_SDMMC    = false;
    static constexpr bool HAS_EXT_NAND = true;
    static constexpr int FLASH_SCK  = 4;   // M_SCK
    static constexpr int FLASH_MOSI = 5;   // M_MOSI
    static constexpr int FLASH_CS   = 6;   // M_FLASH_CS
    static constexpr int FLASH_MISO = 7;   // M_MISO
    static constexpr int SD_CLK = -1;      // no SD slot
    static constexpr int SD_CMD = -1;
    static constexpr int SD_D0  = -1;
    static constexpr int SD_D1  = -1;
    static constexpr int SD_D2  = -1;
    static constexpr int SD_D3  = -1;

    // --- I2C bus (MAX17303 fuel gauge + MP2672 pack charger) ---
    static constexpr int I2C_SCL_PIN = 34;
    static constexpr int I2C_SDA_PIN = 33;
    // BOM assertion: the part at 0x36 IS a MAX17303 — claim it even if its
    // DevName die rev is outside the known 0x404-0x406 range (first bench
    // boot: an unlisted DevName silently demoted the gauge to the MAX17205
    // driver's wrong register map).
    static constexpr bool EXPECT_MAX17303 = true;

    // --- Flight-pack charger (MP2672GD-0000-Z, external 2S packs) ---
    static constexpr bool HAS_PACK_CHARGER = true;
};
