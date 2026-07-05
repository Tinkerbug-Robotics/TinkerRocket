#pragma once

#include <stdint.h>

// V8 PCB pin map + board topology for the out computer (#411, tracking #408).
// Selected with TR_BOARD_V8=1:  idf.py -DTR_BOARD_V8=1 build
//
// BRING-UP IN PROGRESS. Values marked CONFIRMED are from the V8 schematic;
// values marked TODO are inherited V7 placeholders — replace each as its
// V8 net is confirmed. Placeholder INPUTS are harmless on unknown nets;
// placeholder OUTPUTS match what the V7 firmware already drove on this
// board during initial bring-up, so they introduce no new risk.
struct board_pins
{
    // --- Power rail switch ---
    static constexpr int PWR_PIN = 6;        // TODO: confirm V8 net
    // V7's GPS rail pin (33) is the V8 power-sensor SCL — do NOT inherit.
    // -1 = skipped by the (now guarded) call sites until the V8 net is known.
    static constexpr int GPS_PWR_PIN = -1;   // TODO: confirm V8 net

    // --- Power monitoring (INA230 @ 0x40, always-on I2C bus) ---
    static constexpr int PWR_SDA = 21;       // CONFIRMED (V8 schematic)
    static constexpr int PWR_SCL = 33;       // CONFIRMED (V8 schematic)

    // --- Memory shared SPI bus ---
    static constexpr int SPI_SCK = 37;       // TODO: confirm V8 net
    static constexpr int SPI_MISO = 35;      // TODO: confirm V8 net
    static constexpr int SPI_MOSI = 38;      // TODO: confirm V8 net
    static constexpr int NAND_CS = 36;       // TODO: confirm V8 net
    static constexpr int MRAM_CS = 34;       // TODO: confirm V8 net (-1 if no MRAM)

    // --- I2C slave (commands from FlightComputer) ---
    static constexpr int I2C_SDA_PIN = 4;    // TODO: confirm V8 net
    static constexpr int I2C_SCL_PIN = 5;    // TODO: confirm V8 net

    // --- I2S slave RX (high-frequency telemetry from FlightComputer) ---
    // V7 BCLK (21) is the V8 power-sensor SDA — parked on a free GPIO until
    // the V8 net is known (slave RX pins are inputs; harmless anywhere).
    static constexpr int I2S_BCLK_PIN  = 47; // TODO: confirm V8 net
    static constexpr int I2S_WS_PIN    = 45; // TODO: confirm V8 net
    static constexpr int I2S_DIN_PIN   = 2;  // TODO: confirm V8 net
    static constexpr int I2S_FSYNC_PIN = 1;  // TODO: confirm V8 net

    // --- Radio topology: UART daughterboard (#409/#410) — no direct LLCC68.
    // The direct-SPI radio path is compiled but disabled; the UART-modem
    // backend replaces it in #410.
    static constexpr bool USE_LORA_RADIO = false;
    static constexpr int LORA_SPI_SCK = -1;
    static constexpr int LORA_SPI_MISO = -1;
    static constexpr int LORA_SPI_MOSI = -1;
    static constexpr int LORA_CS_PIN = -1;
    static constexpr int LORA_DIO1_PIN = -1;
    static constexpr int LORA_RST_PIN = -1;
    static constexpr int LORA_BUSY_PIN = -1;
};
