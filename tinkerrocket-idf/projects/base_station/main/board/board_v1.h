#pragma once

// V1 (original) base-station PCB pin map + board topology.
// Selected with TR_BS_BOARD=1:  idf.py -B build_v1 -DTR_BS_BOARD=1 build
//
// Topology: MAX17205 fuel gauge (2S NCR18650B), SD card over SDMMC 4-bit,
// direct SPI LLCC68 on-board. The fuel gauge itself is still runtime-probed
// (see main.cpp) so the board headers only carry what silicon can't answer:
// pins and peripheral presence.
struct board_pins
{
    static constexpr const char* BOARD_NAME = "V1 (original)";

    // --- LoRa radio: direct SPI LLCC68 on-board ---
    static constexpr bool USE_UART_RADIO_MODEM = false;
    static constexpr int LORA_SPI_SCK  = 36;
    static constexpr int LORA_SPI_MISO = 34;
    static constexpr int LORA_SPI_MOSI = 35;
    static constexpr int LORA_CS_PIN   = 38;
    static constexpr int LORA_DIO1_PIN = 18;
    static constexpr int LORA_RST_PIN  = 37;
    static constexpr int LORA_BUSY_PIN = 17;
    static constexpr int LORA_RXEN_PIN = -1;   // no RF switch on the original PCB
    static constexpr int LORA_DIO2_PIN = -1;
    static constexpr int LORA_DIO3_PIN = -1;

    // --- Radio daughterboard host link: not present on this board ---
    static constexpr int LORA_UART_TX_PIN = -1;
    static constexpr int LORA_UART_RX_PIN = -1;
    static constexpr int LORA_ACT_PIN     = -1;

    // --- Storage: SD card over SDMMC 4-bit ---
    static constexpr bool HAS_SDMMC    = true;
    static constexpr bool HAS_EXT_NAND = false;
    static constexpr int SD_CLK = 6;
    static constexpr int SD_CMD = 7;
    static constexpr int SD_D0  = 5;
    static constexpr int SD_D1  = 4;
    static constexpr int SD_D2  = 9;
    static constexpr int SD_D3  = 8;
    static constexpr int FLASH_SCK  = -1;   // no external SPI NAND
    static constexpr int FLASH_MOSI = -1;
    static constexpr int FLASH_CS   = -1;
    static constexpr int FLASH_MISO = -1;

    // --- I2C bus (fuel gauge) ---
    // Carried over unchanged from the pre-board-header config.h ("same pins on
    // both boards"). NOTE: these overlap LORA_RST_PIN(37)/LORA_CS_PIN(38)
    // above — that conflict predates this refactor; if V1 hardware is ever
    // flashed again, verify the gauge bus pins against the V1 schematic.
    static constexpr int I2C_SCL_PIN = 37;
    static constexpr int I2C_SDA_PIN = 38;

    // No MAX17303 on this board — 0x36 is the MAX17205; the strict DevName
    // check keeps the MAX17303 driver from claiming it.
    static constexpr bool EXPECT_MAX17303 = false;

    // --- Flight-pack charger (MP2672): V3-only feature ---
    static constexpr bool HAS_PACK_CHARGER = false;
    // --- Flight-pack voltage sense: not present on this board (see V3) ---
    static constexpr int   PACK_VSENSE_GPIO     = -1;
    static constexpr float PACK_VSENSE_DIVIDER  = 0.0f;
    static constexpr int   PACK_VSENSE_ATTEN_DB = -1;
};
