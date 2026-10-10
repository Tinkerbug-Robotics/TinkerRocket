#pragma once

#include <stdint.h>

// Tinker-Mantis V10 pin map + board topology for the out computer
// (ESP32-S3RH2, U15). Selected with TR_BOARD_V10=1:
//   idf.py -B build_v10 -DTR_BOARD_V10=1 build
//
// The map of hardware/tinker-mantis/ at HEAD (title block V10). The fabbed
// V9 is board_v9.h. Derived 2026-10-10 (#1593) from a pad -> pinfunction ->
// net walk of U15 against the V9 tag, and a kicad-cli netlist export of the
// V10 schematic for the circuits behind each new pin.
//
// Every S3 pin V9 uses is unchanged (the I2S nets were renamed ESP_* ->
// ESP_I2S_*). V10 ADDS four pins, all on pads that are bare on V9:
//
//   S3 GPIO   net         what it is
//     14      OC_ARM_EN   our half of the supervised arm (output)
//     17      PG_RAIL     U19 eFuse power-good, low = VBATT off (input)
//     18      INA_ALERT   U23 INA230 ALERT, low = limit crossed (input)
//     34      VBUCK_OK    V_BUCK through a divider, low = on hold-up (input)
//
// OC_ARM_EN is the one that matters for flight: without it a V10 cannot fire
// a channel at all. A V9 out-computer image has no consent pin, so a V10
// running it never arms, even with a V10 flight-computer image.
//
// Hardware differences behind unchanged pins: GPIO0 gained R141 (10 k to
// +3V3), and ESP_I2S_SD (our GPIO3) gained R143 (100 k to GND), so the data
// input is defined while the flight computer is off.
struct board_pins
{
    // --- Power rail switch ---
    // Net P4_EN_S3 into D9, the enable of U30 (TPS22810), which powers the P4.
    // The P4 holds the same enable through P4_EN_HOLD while INFLIGHT (#848).
    static constexpr int PWR_PIN = 7;        // P4_EN_S3 (CONFIRMED)
    // The GNSS rail is gated by the P4 (GPS_ACT -> U27), not by us.
    static constexpr int GPS_PWR_PIN = -1;   // switched by the FC, not the S3

    // --- Pyro arm consent (net OC_ARM_EN) ---
    // Our half of the two-processor arm, the Beetle's design on the Mantis:
    // OC_ARM_EN drives the base of Q14 (DTC123J), which sits in series with
    // Q12 (driven by the P4's FC_ARM, its GPIO33). Only with both on is Q13's
    // base pulled, and Q13 drives the gate of U9, the squib-return FET, from
    // VBAT_CON. Neither processor can arm the board alone.
    //
    // setup_oc() drives it LOW before PWR_PIN can come up. After that it goes
    // high for exactly two reasons, a PYRO_FIRE_TEST being served to the FC or
    // a real (not simulated) flight in progress: arm_consent_policy.h. #1553's
    // fault-current trip can drop it (INA_ALERT below). GPIO14 is not a
    // strapping pin and has no pull at reset; Q14's built-in 47 k holds it off
    // until we drive it. Scope it across reset on the first article (bring-up
    // checklist, hardware/tinker-mantis/v10-bring-up.md).
    //
    // This is a real vote. Do not tie it high for bench convenience; that
    // collapses the design back to a single-processor arm.
    static constexpr int ARM_CONSENT_PIN = 14;   // OC_ARM_EN (Q14 base)
    // #1553: U23's ALERT, open drain with R142 (100 k) to +3V3. The fault trip
    // takes the falling edge in pyroFaultAlertIsr and drops OC_ARM_EN at once;
    // the shunt poll stays as the backstop.
    static constexpr int INA_ALERT_PIN = 18;     // INA_ALERT

    // --- Supply status inputs (#1409, #1593) ---
    // Read as plain inputs, no internal pulls (each has its own), and reported
    // on the console: at boot, on every change, and in the stats lines. Not in
    // the flight log or telemetry yet; the RTC-latched cause is #1409.
    //
    // PG_RAIL: U19 (TPS259824, the pack eFuse, VBAT_CON -> VBATT) power-good,
    // open drain with R59 (100 k) to +3V3. LOW = VBATT is off: the eFuse has
    // tripped or is in UVLO. The pull-up is on the supercap-backed +3V3, so it
    // reads truthfully through the event it exists to explain.
    static constexpr int PG_RAIL_PIN = 17;       // PG_RAIL
    // VBUCK_OK: V_BUCK (the TPS62152 output ahead of the TPS61094 hold-up
    // converter) through R137 (100 k) over R138 (1 M), so the pad sits at
    // about 0.91 x V_BUCK. GPIO34 has no ADC, so it is read as a level: high
    // while the buck supplies, low once the board runs on the hold-up cap.
    // The threshold is the pad's VIH (0.75 x VDD), i.e. V_BUCK below ~2.7 V.
    static constexpr int VBUCK_OK_PIN = 34;      // VBUCK_OK

    // --- Power monitoring (INA230 @ 0x40, always-on I2C bus) ---
    static constexpr int PWR_SDA = 21;       // PWR_SDA, R69 5.11 k
    static constexpr int PWR_SCL = 33;       // PWR_SCL, R67 5.11 k

    // --- Memory shared SPI bus (same pins as V7/V8/V9) ---
    static constexpr int SPI_SCK = 37;       // M_SCK
    static constexpr int SPI_MISO = 35;      // M_MISO
    static constexpr int SPI_MOSI = 38;      // M_MOSI
    static constexpr int NAND_CS = 36;       // M_FLASH_CS (U11)
    // No MRAM, as on V9: U12 was deleted for the S3RH2's in-package PSRAM.
    // board_v9.h has the full consequences.
    static constexpr int MRAM_CS = -1;
    static constexpr bool RING_IN_PSRAM = true;   // 2 MB in-package

    // #1485: the highest IMU logging rate this board flies. The flight
    // computer's board_v10.h keeps FIFO capture on; test_imu_rate_board_parity
    // pins the two equal.
    static constexpr uint16_t IMU_RATE_MAX_HZ = 7680;

    // --- I2C slave (commands from FlightComputer) ---
    static constexpr int I2C_SDA_PIN = 5;    // ESP_SDA, R58 5.11 k
    static constexpr int I2C_SCL_PIN = 6;    // ESP_SCL, R55 5.11 k

    // --- I2S slave RX (high-frequency telemetry from FlightComputer) ---
    // FC pins: ESP_I2S_BCLK=21, ESP_I2S_WS=18, ESP_I2S_SD=19,
    // ESP_I2S_FSYNC=20. Both ends must agree per net. ESP_I2S_FSYNC is a plain
    // GPIO the master pulses per frame, not an I2S peripheral signal.
    static constexpr int I2S_BCLK_PIN  = 2;  // ESP_I2S_BCLK
    static constexpr int I2S_WS_PIN    = 1;  // ESP_I2S_WS
    static constexpr int I2S_DIN_PIN   = 3;  // ESP_I2S_SD (R143 100 k to GND)
    static constexpr int I2S_FSYNC_PIN = 4;  // ESP_I2S_FSYNC

    // --- Radio topology: UART daughterboard, as on V9 ---
    static constexpr bool USE_LORA_RADIO = true;
    static constexpr bool USE_UART_RADIO_MODEM = true;
    static constexpr int LORA_SPI_SCK = -1;
    static constexpr int LORA_SPI_MISO = -1;
    static constexpr int LORA_SPI_MOSI = -1;
    static constexpr int LORA_CS_PIN = -1;
    static constexpr int LORA_DIO1_PIN = -1;
    static constexpr int LORA_RST_PIN = -1;
    static constexpr int LORA_BUSY_PIN = -1;
    // Same J5 and the same crossed naming as V9 (board_v9.h): the cable pin
    // number is the only unambiguous reference. Do not swap these on bring-up.
    static constexpr int LORA_UART_TX_PIN = 11;  // LoRa_TX -> J5.4
    static constexpr int LORA_UART_RX_PIN = 10;  // LoRa_RX <- J5.3
    static constexpr int LORA_ACT_PIN = 12;      // LoRa_ACT -> U29 (TPS22810)

    // --- High-side-switch current monitors (TPS22811 U26/U28), as on V9 ---
    // ILOAD = V(pin) / (IMON_GAIN_A_PER_A * R). Calibrate before trusting amps.
    // No hold-up sense on this processor: V10's V_SCAP_ADC goes to the flight
    // computer's GPIO17, and nothing reads it yet (#1409).
    static constexpr int   SCAP_ADC_PIN       = -1;
    static constexpr float SCAP_DIVIDER_RATIO = 1.0f;    // unused with no pin
    static constexpr int   CAM_IMON_GPIO      = 8;         // CAM_IMON, ADC1_CH7
    static constexpr int   SERVO_IMON_GPIO    = 9;         // SERVO_IMON, ADC1_CH8
    static constexpr float IMON_GAIN_A_PER_A  = 95.3e-6f;  // TPS22811 GIMON typ
    static constexpr float CAM_IMON_R_OHM     = 2000.0f;   // R85
    static constexpr float SERVO_IMON_R_OHM   = 1000.0f;   // R88
};
