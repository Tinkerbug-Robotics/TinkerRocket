#pragma once

#include <driver/uart.h>

// Radio daughterboard (ESP32-S3) pin/board configuration (#409).
//
// CURRENT VALUES ARE DEVKIT BENCH WIRING (ESP32-S3-DevKitC + LLCC68 breakout).
// The final V8 daughterboard pin map replaces these when the schematic lands
// (per-board headers arrive with #411's pattern if the daughterboard itself
// ever needs variants).
struct config
{
    // ---- UART link to the host (OC on the rocket, BS on the ground) --------
    // The modem must never care which host it is plugged into (#409:
    // host-agnostic, swappable matched pairs).
    static constexpr uart_port_t HOST_UART_PORT = UART_NUM_1;
    static constexpr int HOST_UART_TX = 17;
    static constexpr int HOST_UART_RX = 18;
    // Sized for tunnel traffic (LoRa airtime dominates), not raw UART
    // capability; must match the host side (#410).
    static constexpr int HOST_UART_BAUD = 921'600;

    // ---- LoRa radio (LLCC68 over SPI) --------------------------------------
    static constexpr int LORA_SPI_SCK = 12;
    static constexpr int LORA_SPI_MISO = 13;
    static constexpr int LORA_SPI_MOSI = 11;
    static constexpr int LORA_CS_PIN = 10;
    static constexpr int LORA_DIO1_PIN = 4;
    static constexpr int LORA_RST_PIN = 5;
    static constexpr int LORA_BUSY_PIN = 6;

    // ---- Radio capability (reported in IDENTITY; hosts clamp to these) -----
    static constexpr int8_t RADIO_MAX_TX_POWER_DBM = 22;   // LLCC68 ceiling
    static constexpr float RADIO_FREQ_MIN_MHZ = 150.0f;    // LLCC68 band edges
    static constexpr float RADIO_FREQ_MAX_MHZ = 960.0f;

    // ---- Boot radio defaults ------------------------------------------------
    // The modem comes up listening on these until the host pushes SET_CONFIG,
    // mirroring TR_LoRa_Comms defaults. The host owns the real config
    // (transparent-modem principle: no NVS on the modem).
    static constexpr float BOOT_FREQ_MHZ = 915.0f;
    static constexpr uint8_t BOOT_SF = 10;
    static constexpr float BOOT_BW_KHZ = 125.0f;
    static constexpr uint8_t BOOT_CR = 7;
    static constexpr int8_t BOOT_TX_POWER_DBM = 20;

    static constexpr bool DEBUG = true;
};
