#pragma once

#include <stdint.h>

// V8 PCB pin map + board topology for the out computer (#411, tracking #408).
// Selected with TR_BOARD_V8=1:  idf.py -B build_v8 -DTR_BOARD_V8=1 build
//
// Net names in comments are the V8 schematic nets. Values marked TODO are
// still unconfirmed — replace as bring-up proceeds.
//
// ALSO THE V9/V10 MAP — this MCU did not move (verified 2026-08-17 by walking
// U15's pad -> pinfunction -> net in hardware/rocket-computer.kicad_pcb at
// every commit that ever touched it, and against a kicad-cli netlist export
// of the V10 schematic). -DTR_BOARD_V9=1 selects this same header; see
// main/config.h. That is the opposite of the FC, whose V9 map moved PYRO_ARM
// and swapped two FIRE pins and therefore has its own board_v9.h.
//
// Two things changed around this map without changing it:
//   * PWR_PIN's net was renamed Power_Switch -> P4_EN_S3 at the V9 pre-fab
//     close-out. Same GPIO7, same job: it is the enable that powers the P4's
//     rail (U30, a TPS22810). On V9+ the P4 also has its own hold line into
//     the same enable (its GPIO5, net P4_EN_HOLD) which its firmware does not
//     currently drive — so dropping PWR_PIN still powers the FC down, but
//     check flight_computer/main/board/board_v9.h before assuming that.
//   * MRAM_CS (GPIO34) is V8-only. There is no MRAM_CS net anywhere in the
//     V9/V10 schematic; the pin is unconnected and the part is unfitted.
//     Left at 34 because it is correct for V8 and harmless on V9/V10 (it
//     drives a floating pad), but set it to -1 if an MRAM-present code path
//     ever costs more than a probe.
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
    // CONFIRMED against both PCBs' pad->net maps, not the labels alone: our
    // J5.4 (net LoRa_TX, GPIO11) lands on the daughterboard's J6.4 (its net
    // LoRa_RX, its GPIO5), and our J5.3 (net LoRa_RX, GPIO10) lands on its
    // J6.3 (its net LoRa_TX, its GPIO6). The two boards name the nets from
    // their own perspective, so the same pair of names appears on both ends
    // of a crossed link — the cable pin number is the only unambiguous
    // reference. (This is the trap that #234/RunCam hit; it is resolved here,
    // so do NOT "fix" it by swapping these two on bring-up.)
    static constexpr int LORA_UART_TX_PIN = 11;  // LoRa_TX -> J5.4 (CONFIRMED)
    static constexpr int LORA_UART_RX_PIN = 10;  // LoRa_RX <- J5.3 (CONFIRMED)
    // Daughterboard power gate (high = powered), like CAM_ACT for the RunCam.
    // Also the recovery hammer for a wedged/bricked daughterboard (#412).
    static constexpr int LORA_ACT_PIN = 12;      // LoRa_ACT (CONFIRMED)

    // --- High-side-switch current monitors (V9 close-out: TPS22811 U26/U28) ---
    // IMON sources GIMON x ILOAD into a ground-referenced gain resistor:
    //   ILOAD = V(pin) / (IMON_GAIN_A_PER_A * R).
    // GIMON is 95.3 uA/A typical but 82.9-107.6 over temp (datasheet, at
    // 1.5 A) — calibrate against a known load before trusting absolute amps.
    // Gain resistors per hardware PR #728: CAM R85 = 2.0 k (2.2 k in the
    // original V9 close-out) and SERVO R88 = 1.0 k — deliberately landing
    // BOTH channels near 0.286 V at their design currents (camera 1.5 A,
    // servo 3 A per high-side-switch-design.md). Nothing reads these yet;
    // staged so the first implementation cannot inherit the 2.2 k scale.
    static constexpr int   CAM_IMON_GPIO      = 8;         // CAM_IMON, ADC1_CH7
    static constexpr int   SERVO_IMON_GPIO    = 9;         // SERVO_IMON, ADC1_CH8
    static constexpr float IMON_GAIN_A_PER_A  = 95.3e-6f;  // TPS22811 GIMON typ
    static constexpr float CAM_IMON_R_OHM     = 2000.0f;   // R85 (PR #728)
    static constexpr float SERVO_IMON_R_OHM   = 1000.0f;   // R88
};
