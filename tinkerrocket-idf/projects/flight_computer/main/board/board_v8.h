#pragma once

#include <stdint.h>

// V8 PCB pin map + board topology for the flight computer (ESP32-P4)
// (#411, tracking #408). Selected with TR_BOARD_V8=1:
//   idf.py -B build_v8 -DTR_BOARD_V8=1 build
//
// Net names in comments are the V8 schematic nets (2026-07-05 capture).
// Values marked TODO are unconfirmed — replace as bring-up proceeds.
struct board_pins
{
    // ### Sensor SPI bus (SENS_* nets) ###
    static constexpr uint8_t SPI_SCK = 53;  // SENS_SCLK
    static constexpr uint8_t SPI_SDO = 51;  // SENS_SDO
    static constexpr uint8_t SPI_SDI = 52;  // SENS_SDI

    // ### GNSS serial ###
    // Schematic: GNSS_TX=3, GNSS_RX=4 (label perspective unverified — the
    // driver's 9600-baud boot probe auto-detects a swap, as it did on V7).
    // GNSS_RXD2 net on GPIO2 (receiver's second UART) is unused by firmware.
    static constexpr uint8_t GNSS_RX = 4;   // GNSS_RX
    static constexpr uint8_t GNSS_TX = 3;   // GNSS_TX
    static constexpr int8_t GNSS_RESET_N = -1;
    static constexpr int8_t GNSS_SAFEBOOT_N = -1;

    // ### Sensor CS / bus pins ###
    static constexpr int MMC5983MA_CS = -1;    // no MMC5983MA on V8
    static constexpr uint8_t BMP585_CS = 41;   // BMP585_CS
    static constexpr uint8_t ISM6HG256_CS = 49;// ISM6HG256_CS
    static constexpr uint8_t IIS2MDC_SDA = 47; // IIS2MDCTR_SDA
    static constexpr uint8_t IIS2MDC_SCL = 48; // IIS2MDCTR_SCL

    // ### Part presence ###
    static constexpr bool USE_BMP585 = true;
    static constexpr bool USE_MMC5983MA = false;  // not populated on V8
    static constexpr bool USE_GNSS = true;
    static constexpr bool USE_ISM6HG256 = true;
    static constexpr bool USE_IIS2MDC = true;

    // ### Sensor interrupt pins ###
    static constexpr uint8_t ISM6HG256_INT = 50;  // ISM6HG256_INT1
    static constexpr uint8_t BMP585_INT = 42;     // BMP585_INT
    static constexpr int MMC5983MA_INT = -1;      // no MMC5983MA on V8
    static constexpr uint8_t IIS2MDC_INT = 46;    // IIS2MDCTR_INT

    // ### Camera (same nets as V7: Camera_TX=30, Camera_RX=31, CAM_ACT=32) ###
    static constexpr int8_t CAM_PWR_PIN = 30;      // GoPro path, unused
    static constexpr int8_t CAM_SHUTTER_PIN = 31;  // GoPro path, unused
    static constexpr int8_t RUNCAM_RX_PIN = 31;    // FC receives (Camera_TX)
    static constexpr int8_t RUNCAM_TX_PIN = 30;    // FC transmits (Camera_RX)
    static constexpr int8_t RUNCAM_PWR_PIN = 32;   // camera power gate (CAM_ACT)

    // ### Pyro channels — FULL REMAP vs V7 ###
    // SAFETY: first V8 power-up must scope-verify the FIRE pads stay quiet
    // at boot (no squibs connected). All output init goes through
    // safePyroOutputInit; check each pin's P4 IO-MUX default function
    // (the V7 hazard was pins 14-19 defaulting to SPI2/SPI3 — see 421dd63).
    static constexpr uint8_t PYRO_ARM_PIN   = 5;   // PYRO_ARM
    static constexpr uint8_t PYRO1_FIRE_PIN = 6;   // PYRO1_FIRE
    static constexpr uint8_t PYRO1_CONT_PIN = 7;   // PYRO1_CONT
    static constexpr uint8_t PYRO2_FIRE_PIN = 9;   // PYRO2_FIRE
    static constexpr uint8_t PYRO2_CONT_PIN = 10;  // PYRO2_CONT
    static constexpr uint8_t PYRO3_FIRE_PIN = 11;  // PYRO3_FIRE
    static constexpr uint8_t PYRO3_CONT_PIN = 12;  // PYRO3_CONT
    static constexpr uint8_t PYRO4_FIRE_PIN = 13;  // PYRO4_FIRE
    static constexpr uint8_t PYRO4_CONT_PIN = 14;  // PYRO4_CONT

    // ### Fin servos ###
    // CONFIRMED against the board files: each EXP net's package pin was read
    // out of hardware/rocket-computer/rocket-computer.kicad_pcb and matched to
    // the ESP32-P4 pin assignment in the schematic. The four servo signals are
    // EXP_01..04 in order, which is what was already assumed here.
    //
    // The full expansion map, for anyone wiring the other eight channels:
    //
    //   net       GPIO   pkg pin      net       GPIO   pkg pin
    //   EXP_01     45      87         EXP_07     29      58
    //   EXP_02     44      86         EXP_08     28      57
    //   EXP_03     43      84         EXP_09     38      70
    //   EXP_04     54      98         EXP_10     37      69
    //   EXP_05     39      80         EXP_11     34      65
    //   EXP_06     40      81         EXP_12     33      64
    //
    // All twelve come out on the 16-pin Molex 878321620: pins 3..8 carry
    // EXP_01..06 ascending, then pins 9..14 carry EXP_12..07 DESCENDING --
    // pin 9 is EXP_12, not EXP_07. Pins 1-2 are a MOSFET-switched return
    // (Q8, PMPB14XNX, source on GND); pins 15-16 are VBATT.
    static constexpr uint8_t SERVO_PIN_1 = 45;  // EXP_01, connector pin 3
    static constexpr uint8_t SERVO_PIN_2 = 44;  // EXP_02, connector pin 4
    static constexpr uint8_t SERVO_PIN_3 = 43;  // EXP_03, connector pin 5
    static constexpr uint8_t SERVO_PIN_4 = 54;  // EXP_04, connector pin 6

    // ### Indicators ###
    static constexpr uint8_t PIEZO_PIN = 17;      // PIEZZO
    // TODO: confirm which IND_* is which color (guess: IND_1=red, IND_2=blue).
    static constexpr uint8_t RED_LED_PIN = 27;    // IND_1
    static constexpr uint8_t BLUE_LED_PIN = 26;   // IND_2

    // ### I2C command/config channel to the OutComputer ###
    static constexpr uint8_t ESP_SDA_PIN = 23;  // ESP_SDA (OC pin 5)
    static constexpr uint8_t ESP_SCL_PIN = 22;  // ESP_SCL (OC pin 6)

    // ### I2S high-frequency telemetry to the OutComputer ###
    // ESP_* net convention (must match the OC's board_v8.h per-net):
    //   ESP_SCLK = bit clock, ESP_SDO = FC data out, ESP_CS = word select,
    //   ESP_SDI = frame sync.
    static constexpr int I2S_BCLK_PIN  = 21;  // ESP_SCLK (OC pin 2)
    static constexpr int I2S_WS_PIN    = 18;  // ESP_CS   (OC pin 1)
    static constexpr int I2S_DOUT_PIN  = 19;  // ESP_SDO  (OC pin 3)
    static constexpr int I2S_FSYNC_PIN = 20;  // ESP_SDI  (OC pin 4)

    // ### Power gates (new on V8: FC switches these rails) ###
    static constexpr int GPS_ACT_PIN = 15;    // GPS_ACT — GNSS rail enable
    static constexpr int SERVO_ACT_PIN = 8;   // SERVO_ACT — servo-power MOSFET
                                              // (the #345 "guaranteed fix";
                                              // driven high at boot for now,
                                              // pad-relax integration is a
                                              // follow-up)
};
