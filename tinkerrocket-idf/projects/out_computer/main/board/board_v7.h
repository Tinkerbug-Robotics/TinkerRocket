#pragma once

#include <stdint.h>

// V7 (current flight PCB) pin map + board topology for the out computer.
// Selected when TR_BOARD_V8=0 (the default) — see config.h (#411).
//
// ONLY pins and peripheral-presence/topology flags live here. Logic and
// protocol constants (I2S sample rate, I2C address/speeds, LoRa RF
// parameters, storage sizes...) stay in config.h and must not fork per
// board revision.
struct board_pins
{
    // --- Power rail switch ---
    static constexpr int PWR_PIN = 6;
    // Dedicated GPS enable rail, toggled in lockstep with PWR_PIN.
    // -1 = no separate GPS rail on this board (call sites skip it).
    static constexpr int GPS_PWR_PIN = 33;

    // --- Power monitoring (INA230 @ 0x40, always-on I2C bus) ---
    static constexpr int PWR_SDA = 7;
    static constexpr int PWR_SCL = 8;

    // --- Memory shared SPI bus ---
    static constexpr int SPI_SCK = 37;
    static constexpr int SPI_MISO = 35;
    static constexpr int SPI_MOSI = 38;
    static constexpr int NAND_CS = 36;
    // MR25H10 on the shared bus; -1 = no MRAM (RAM ring fallback).
    static constexpr int MRAM_CS = 34;

    // --- I2C slave (commands from FlightComputer) ---
    static constexpr int I2C_SDA_PIN = 4;
    static constexpr int I2C_SCL_PIN = 5;

    // --- I2S slave RX (high-frequency telemetry from FlightComputer) ---
    // Actual wiring: FC 27→OC 21, FC 28→OC 45, FC 23→OC 2, FC 17→OC 1
    static constexpr int I2S_BCLK_PIN  = 21;
    static constexpr int I2S_WS_PIN    = 45;
    static constexpr int I2S_DIN_PIN   = 2;
    static constexpr int I2S_FSYNC_PIN = 1;

    // --- Radio topology: direct LLCC68 on SPI ---
    // (V8 replaces this with the UART radio daughterboard, #410/#414.)
    static constexpr bool USE_LORA_RADIO = true;
    static constexpr bool USE_UART_RADIO_MODEM = false;
    static constexpr int LORA_SPI_SCK = 14;
    static constexpr int LORA_SPI_MISO = 11;
    static constexpr int LORA_SPI_MOSI = 12;
    static constexpr int LORA_CS_PIN = 18;
    static constexpr int LORA_DIO1_PIN = 9;
    static constexpr int LORA_RST_PIN = 17;
    static constexpr int LORA_BUSY_PIN = 13;

    // --- Radio daughterboard host link (V8-only topology, #410) ---
    static constexpr int LORA_UART_TX_PIN = -1;
    static constexpr int LORA_UART_RX_PIN = -1;
    static constexpr int LORA_ACT_PIN = -1;
    // --- High-side-switch current monitors: not present on V7 (see V8) ---
    static constexpr int   CAM_IMON_GPIO      = -1;
    static constexpr int   SERVO_IMON_GPIO    = -1;
    static constexpr float IMON_GAIN_A_PER_A  = 0.0f;
    static constexpr float CAM_IMON_R_OHM     = 0.0f;
    static constexpr float SERVO_IMON_R_OHM   = 0.0f;
};
