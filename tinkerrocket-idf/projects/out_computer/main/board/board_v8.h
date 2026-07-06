#pragma once

#include <stdint.h>

// V8 PCB pin map + board topology for the out computer (#411, tracking #408).
// Selected with TR_BOARD_V8=1:  idf.py -B build_v8 -DTR_BOARD_V8=1 build
//
// Net names in comments are the V8 schematic nets. Values marked TODO are
// still unconfirmed — replace as bring-up proceeds.
struct board_pins
{
    // --- Power rail switch ---
    // V8 net Power_Switch. NOTE: V7's PWR_PIN (6) is the V8 ESP_SCL net —
    // running the V7 map on this board toggled the FC I2C clock on every
    // power on/off.
    static constexpr int PWR_PIN = 7;        // Power_Switch (CONFIRMED)
    // No separate GPS rail identified on V8 yet; -1 = call sites skip it.
    static constexpr int GPS_PWR_PIN = -1;   // TODO: confirm whether V8 has one

    // --- Power monitoring (INA230 @ 0x40, always-on I2C bus) ---
    static constexpr int PWR_SDA = 21;       // CONFIRMED (bench-validated)
    static constexpr int PWR_SCL = 33;       // CONFIRMED (bench-validated)

    // --- Memory shared SPI bus (same pins as V7) ---
    static constexpr int SPI_SCK = 37;       // M_SCK      (CONFIRMED)
    static constexpr int SPI_MISO = 35;      // M_MISO     (CONFIRMED)
    static constexpr int SPI_MOSI = 38;      // M_MOSI     (CONFIRMED)
    static constexpr int NAND_CS = 36;       // M_FLASH_CS (CONFIRMED)
    static constexpr int MRAM_CS = 34;       // MRAM_CS    (CONFIRMED; -1 if unfitted)

    // --- I2C slave (commands from FlightComputer) ---
    static constexpr int I2C_SDA_PIN = 5;    // ESP_SDA (CONFIRMED)
    static constexpr int I2C_SCL_PIN = 6;    // ESP_SCL (CONFIRMED)

    // --- I2S slave RX (high-frequency telemetry from FlightComputer) ---
    // ESP_* net convention, fixed against the FC-side schematic (FC pins:
    // ESP_SCLK=21, ESP_CS=18, ESP_SDO=19, ESP_SDI=20) and mirrored in the
    // FC's board_v8.h — both ends must agree PER NET:
    //   ESP_SCLK = bit clock, ESP_SDO = FC data out (our DIN),
    //   ESP_CS = word select, ESP_SDI = frame sync.
    static constexpr int I2S_BCLK_PIN  = 2;  // ESP_SCLK
    static constexpr int I2S_WS_PIN    = 1;  // ESP_CS
    static constexpr int I2S_DIN_PIN   = 3;  // ESP_SDO
    static constexpr int I2S_FSYNC_PIN = 4;  // ESP_SDI

    // --- Radio topology: UART daughterboard (#409/#410) — no direct LLCC68.
    // USE_LORA_RADIO=true: the radio paths run, through the UART-modem
    // backend rather than the direct SPI driver.
    static constexpr bool USE_LORA_RADIO = true;
    static constexpr bool USE_UART_RADIO_MODEM = true;
    static constexpr int LORA_SPI_SCK = -1;
    static constexpr int LORA_SPI_MISO = -1;
    static constexpr int LORA_SPI_MOSI = -1;
    static constexpr int LORA_CS_PIN = -1;
    static constexpr int LORA_DIO1_PIN = -1;
    static constexpr int LORA_RST_PIN = -1;
    static constexpr int LORA_BUSY_PIN = -1;

    // --- Radio daughterboard host link (consumed by the UART-modem backend,
    //     #410; constants staged here so the pin map is complete) ---
    // Schematic labels LoRa_TX/LoRa_RX: wired as-labeled (TX net = OC
    // transmit). CAUTION: TX/RX label perspective has been wrong on this
    // schematic family before (RunCam, #234) — if the daughterboard's BOOT
    // message never arrives on bring-up, swap these two first.
    static constexpr int LORA_UART_TX_PIN = 11;  // LoRa_TX (label-perspective)
    static constexpr int LORA_UART_RX_PIN = 10;  // LoRa_RX (label-perspective)
    // Daughterboard power gate (high = powered), like CAM_ACT for the RunCam.
    // Also the recovery hammer for a wedged/bricked daughterboard (#412).
    static constexpr int LORA_ACT_PIN = 12;      // LoRa_ACT (CONFIRMED)
};
