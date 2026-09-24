#pragma once

// Tinker-Base pin map + board topology (hardware/tinker-base, silkscreen V1).
// Selected with TR_BS_BOARD=4:  idf.py -B build_v4 -DTR_BS_BOARD=4 build
//
// The build number continues the base-station sequence rather than the
// silkscreen, as V3 (the PCB V5/V6 full base station) already does: the
// Tinker-Base's own first revision builds as board 4 and stamps "-v4".
//
// Netlist-verified 2026-09-24 against hardware/tinker-base (kicad-cli netlist,
// U3 pad -> pinfunction -> net). The board forked from the full base station
// (hardware/legacy/base-station) and keeps its MCU, boot NOR, crystals, USB,
// buttons and own-battery divider on the same pins. What it changed, per the
// netlist:
//  - Radio: an E220-900MM22S (LLCC68) on the S3's own SPI, where V3 powers and
//    talks to a lora-daughterboard over UART. Every signal lands on the GPIO the
//    daughterboard's ESP32-S3 uses (projects/radio_board/main/config.h), so the
//    direct-SPI path V1/V2 already take drives it unchanged.
//  - No flight-pack charger: the MP2672, its PosADC/MidADC dividers (V3
//    GPIO8/9) and the I2C bus that served it are gone; GPIO33/34 carry the
//    radio's MISO/BUSY instead.
//  - No daughterboard power gate: the radio runs straight off +3V3.
//
// NOT YET RUN ON HARDWARE — no Tinker-Base has been built. Every constant
// below is a netlist fact; none is bench-proven.
struct board_pins
{
    static constexpr const char* BOARD_NAME = "V4 Tinker-Base (on-board E220 LoRa, no gauge)";

    // --- LoRa radio: on-board E220-900MM22S (U16) over SPI ---
    // The E220's RF switch is split: TXEN is looped to the radio's own DIO2 on
    // the module side (L_DI02, no MCU pin), which TR_LoRa_Comms drives through
    // setDio2AsRfSwitch(true); RXEN is the one the MCU drives. DIO3 is left
    // unconnected on purpose — the module carries its own 32 MHz crystal, so
    // it must not be configured for a DIO3-powered TCXO.
    static constexpr bool USE_UART_RADIO_MODEM = false;
    static constexpr int LORA_SPI_SCK  = 17;   // L_SCK   -> U16 SCK
    static constexpr int LORA_SPI_MISO = 33;   // L_MISO  <- U16 MISO
    static constexpr int LORA_SPI_MOSI = 21;   // L_MOSI  -> U16 MOSI
    static constexpr int LORA_CS_PIN   = 18;   // L_CS    -> U16 NSS
    static constexpr int LORA_DIO1_PIN = 2;    // L_DI01  <- U16 DIO1
    static constexpr int LORA_RST_PIN  = 38;   // L_RST   -> U16 NRST
    static constexpr int LORA_BUSY_PIN = 34;   // L_BUSY  <- U16 BUSY
    static constexpr int LORA_RXEN_PIN = 35;   // L_RXEN  -> U16 RXEN
    static constexpr int LORA_DIO2_PIN = -1;   // module-local loop to TXEN
    static constexpr int LORA_DIO3_PIN = -1;   // unconnected (own crystal)

    // --- Radio daughterboard host link: not on this board ---
    static constexpr int LORA_UART_TX_PIN = -1;
    static constexpr int LORA_UART_RX_PIN = -1;
    static constexpr int LORA_ACT_PIN     = -1;

    // --- Storage: the boot NOR only, as on V3 ---
    // U1 is a GD25Q128ESIG (16 MB) on the S3's dedicated SPI0 pins; there is no
    // NAND and no SD slot (GPIO4-7 are unconnected pads). Logs go to spiffs on
    // U1, laid out by the same 16 MB overlay V3 uses (sdkconfig.defaults.v3,
    // partitions_v3.csv).
    static constexpr bool HAS_SDMMC    = false;
    static constexpr bool HAS_EXT_NAND = false;
    static constexpr int FLASH_SCK  = -1;
    static constexpr int FLASH_MOSI = -1;
    static constexpr int FLASH_CS   = -1;
    static constexpr int FLASH_MISO = -1;
    static constexpr int SD_CLK = -1;      // no SD slot
    static constexpr int SD_CMD = -1;
    static constexpr int SD_D0  = -1;
    static constexpr int SD_D1  = -1;
    static constexpr int SD_D2  = -1;
    static constexpr int SD_D3  = -1;

    // --- I2C: no bus on this board ---
    // Nothing on the netlist is I2C: no fuel gauge (the cell is protected by a
    // DW01A + FS8205A pair, as on V3) and no pack charger. main.cpp skips the
    // bus bring-up when these are -1.
    static constexpr int I2C_SCL_PIN = -1;
    static constexpr int I2C_SDA_PIN = -1;
    static constexpr bool EXPECT_MAX17303 = false;
    static constexpr bool HAS_FUEL_GAUGE  = false;

    // --- Own-battery voltage sense (net Volt_Read) — unchanged from V3 ---
    // BT2, a single 18650 charged by U5 BQ21040 (1S linear), so cells = 1.
    // V_SWITCH -> R44 1M -> Volt_Read -> R46 1M -> GND, C30 100 nF at the pin.
    // 12 dB for the same reason as V3: a full 4.2 V cell divides to 2.1 V,
    // outside 6 dB's ~1.75 V calibrated range.
    static constexpr int   BATT_VSENSE_GPIO     = 1;    // Volt_Read, ADC1_CH0
    static constexpr float BATT_VSENSE_DIVIDER  = 2.0f; // R44 1M / R46 1M
    static constexpr int   BATT_VSENSE_ATTEN_DB = 12;   // ADC_ATTEN_DB_12

    // --- Flight-pack charger and its dividers: not on this board ---
    static constexpr bool HAS_PACK_CHARGER = false;
    static constexpr int   PACK_VSENSE_GPIO         = -1;
    static constexpr float PACK_VSENSE_DIVIDER      = 0.0f;
    static constexpr int   PACK_VSENSE_ATTEN_DB     = -1;
    static constexpr int   PACK_MID_VSENSE_GPIO     = -1;
    static constexpr float PACK_MID_VSENSE_DIVIDER  = 0.0f;
    static constexpr int   PACK_MID_VSENSE_ATTEN_DB = -1;
};
