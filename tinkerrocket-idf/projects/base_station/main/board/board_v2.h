#pragma once

// V2 base-station PCB pin map + board topology (the board the pre-header
// config.h called "the new PCB"). This is the DEFAULT board:
//   idf.py build            (or explicitly: -B build_v2 -DTR_BS_BOARD=2)
//
// Topology: BQ27Z746 fuel gauge (1S, runtime-probed at 0x55), FORESEE
// F35SQB004G SPI NAND for logging (M_* nets, SPI3_HOST — LoRa owns
// SPI2_HOST), direct SPI LLCC68 on-board with the full RF-switch control
// set (RXEN/DIO2/DIO3).
struct board_pins
{
    static constexpr const char* BOARD_NAME = "V2 (BQ27Z746 + NAND + direct LLCC68)";

    // --- LoRa radio: direct SPI LLCC68 on-board ---
    // NOTE: LoRa RST = GPIO33 (differs from BOTH the prior config value 36
    // and the original-board 37).
    static constexpr bool USE_UART_RADIO_MODEM = false;
    static constexpr int LORA_SPI_SCK  = 10;   // L_SCK
    static constexpr int LORA_SPI_MISO = 13;   // L_MISO
    static constexpr int LORA_SPI_MOSI = 12;   // L_MOSI
    static constexpr int LORA_CS_PIN   = 11;   // L_CS
    static constexpr int LORA_DIO1_PIN = 21;   // L_DIO1
    static constexpr int LORA_RST_PIN  = 33;   // L_RST
    static constexpr int LORA_BUSY_PIN = 14;   // L_BUSY
    static constexpr int LORA_RXEN_PIN = 17;   // L_RXEN (RF switch RX enable)
    static constexpr int LORA_DIO2_PIN = 18;   // L_DIO2
    static constexpr int LORA_DIO3_PIN = 3;    // L_DIO3

    // --- Radio daughterboard host link: not present on this board ---
    static constexpr int LORA_UART_TX_PIN = -1;
    static constexpr int LORA_UART_RX_PIN = -1;
    static constexpr int LORA_ACT_PIN     = -1;

    // --- Storage: external SPI NAND (FORESEE F35SQB004G) ---
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

    // --- I2C bus (fuel gauge; SCL_SENS/SDA_SENS nets) ---
    static constexpr int I2C_SCL_PIN = 37;
    static constexpr int I2C_SDA_PIN = 38;

    // --- Flight-pack charger (MP2672): V3-only feature ---
    static constexpr bool HAS_PACK_CHARGER = false;
};
