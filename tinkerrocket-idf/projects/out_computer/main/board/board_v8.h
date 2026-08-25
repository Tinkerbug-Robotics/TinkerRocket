#pragma once

#include <stdint.h>

// V8 PCB pin map + board topology for the out computer (#411, tracking #408).
// Selected with TR_BOARD_V8=1:  idf.py -B build_v8 -DTR_BOARD_V8=1 build
//
// Net names in comments are the V8 schematic nets. Values marked TODO are
// still unconfirmed — replace as bring-up proceeds.
//
// V8 ONLY. The S3's GPIO map really is unchanged V8 -> V10 (verified
// 2026-08-17 by walking U15's pad -> pinfunction -> net in
// hardware/rocket-computer.kicad_pcb at every commit that ever touched it,
// and against a kicad-cli netlist export of the V10 schematic), and until
// #822 -DTR_BOARD_V9=1 therefore selected this header. It no longer does:
// V9/V10 deleted the MRAM (U12), and an alias cannot express "same pins, one
// fewer device". board_v9.h is that revision's header; the ONLY value that
// differs is MRAM_CS (34 here, -1 there).
//
// Keep the two in step. Anything that moves an S3 pin has to move it in both
// files — nothing in CI diffs them, and a pin edited here alone would be
// silently absent from every V9/V10 image.
//
// One thing changed around this map without changing it: PWR_PIN's net was
// renamed Power_Switch -> P4_EN_S3 at the V9 pre-fab close-out. Same GPIO7,
// same job: it is the enable that powers the P4's rail (U30, a TPS22810). On
// V9+ the P4 also has its own hold line into the same enable (its GPIO5, net
// P4_EN_HOLD), which FC firmware HOLDS while INFLIGHT (#848) — so on V9+
// hardware dropping PWR_PIN powers the FC down on the ground but NOT during
// a flight/sim; see flight_computer/main/board/board_v9.h for the
// hold/release policy. (Irrelevant on a physical V8: no hold line exists.)
struct board_pins
{
    // --- Power rail switch ---
    // V8 net Power_Switch. NOTE: V7's PWR_PIN (6) is the V8 ESP_SCL net —
    // running the V7 map on this board toggled the FC I2C clock on every
    // power on/off.
    static constexpr int PWR_PIN = 7;        // Power_Switch (CONFIRMED)
    // No separate GPS rail identified on V8 yet; -1 = call sites skip it.
    // (On V9/V10 the GNSS rail is gated by the FC, not the S3 — see board_v9.h.)
    static constexpr int GPS_PWR_PIN = -1;   // TODO: confirm whether V8 has one

    // --- Power monitoring (INA230 @ 0x40, always-on I2C bus) ---
    static constexpr int PWR_SDA = 21;       // CONFIRMED (bench-validated)
    static constexpr int PWR_SCL = 33;       // CONFIRMED (bench-validated)

    // --- Memory shared SPI bus (same pins as V7) ---
    static constexpr int SPI_SCK = 37;       // M_SCK      (CONFIRMED)
    static constexpr int SPI_MISO = 35;      // M_MISO     (CONFIRMED)
    static constexpr int SPI_MOSI = 38;      // M_MOSI     (CONFIRMED)
    static constexpr int NAND_CS = 36;       // M_FLASH_CS (CONFIRMED)
    // Fitted on V8 only; board_v9.h sets this to -1 (U12 deleted in V9).
    static constexpr int MRAM_CS = 34;       // MRAM_CS    (CONFIRMED)
    // MRAM is fitted, so the ring lives there and never reaches the RAM/PSRAM
    // path at all. false regardless of what this board's S3 turns out to be —
    // no V8 artwork was ever committed, so its part is unproven either way.
    static constexpr bool RING_IN_PSRAM = false;

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

    // --- High-side-switch current monitors: NOT FITTED ON V8 (#850) ---
    // This block previously declared GPIO8/GPIO9 here, carried over verbatim
    // from the V9 close-out (its own header said "V9 close-out: TPS22811
    // U26/U28"). But the TPS22811 high-side switches are a V9 change: V9
    // replaced V8's LOW-SIDE MOSFETs with them, and only U26 (camera) and U28
    // (servo) are TPS22811 parts with an IMON pin at all — see the corrected
    // note in flight_computer board_v9.h (#837 item 2).
    //
    // Leaving them non-negative made a V8 build read two ADC pins that have no
    // monitor driving them and report the result as amps. Set to -1 so
    // readRailCurrents() skips the channel and POWERData reports 0.
    //
    // NOT VERIFIED against V8 artwork, which lives outside this repo — this is
    // the safe direction, not a measurement. If a V8 board is ever confirmed to
    // carry the monitors, restore the pins and the V9 gain constants together.
    static constexpr int   CAM_IMON_GPIO      = -1;
    static constexpr int   SERVO_IMON_GPIO    = -1;
    static constexpr float IMON_GAIN_A_PER_A  = 0.0f;
    static constexpr float CAM_IMON_R_OHM     = 0.0f;
    static constexpr float SERVO_IMON_R_OHM   = 0.0f;
};
