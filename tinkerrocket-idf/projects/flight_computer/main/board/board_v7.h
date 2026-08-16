#pragma once

#include <stdint.h>

// V7 (current flight PCB) pin map + board topology for the flight computer
// (ESP32-P4). Selected when TR_BOARD_V8=0 (the default) — see config.h (#411).
//
// ONLY pins and part-presence flags live here (values verbatim from the
// pre-#411 config.h). Logic, rates, gains, and protocol constants stay in
// config.h and must not fork per board revision.
struct board_pins
{
    // ### Sensor SPI bus ###
    static constexpr uint8_t SPI_SCK = 9;
    static constexpr uint8_t SPI_SDO = 10;
    static constexpr uint8_t SPI_SDI = 8;

    // ### GNSS serial ###
    // Schematic-labeled wiring. The driver auto-detects the RX/TX swap on
    // boards where the labeling is reversed via a fast 9600-baud activity
    // probe at boot.
    static constexpr uint8_t GNSS_RX = 3;
    static constexpr uint8_t GNSS_TX = 4;
    static constexpr int8_t GNSS_RESET_N = -1;
    static constexpr int8_t GNSS_SAFEBOOT_N = -1;

    // ### Sensor CS / bus pins ###
    // Pin 13 is shared between MMC5983MA_CS and IIS2MDC_SDA — only one of
    // the two mag parts is populated on any given V7 board (boot auto-detect
    // probes IIS2MDC over I2C first, falls back to MMC5983MA over SPI).
    static constexpr int MMC5983MA_CS = 13;
    static constexpr uint8_t BMP585_CS = 6;
    static constexpr uint8_t ISM6HG256_CS = 7;
    static constexpr uint8_t IIS2MDC_SDA = 13;
    static constexpr uint8_t IIS2MDC_SCL = 20;

    // ### Part presence ###
    static constexpr bool USE_BMP585 = true;
    static constexpr bool USE_MMC5983MA = true;
    static constexpr bool USE_GNSS = true;
    static constexpr bool USE_ISM6HG256 = true;
    // When USE_MMC5983MA is true, the magnetometer slot first probes for an
    // IIS2MDC over I2C; if not detected, the MMC5983MA SPI path runs.
    static constexpr bool USE_IIS2MDC = true;

    // ### Sensor interrupt pins ###
    static constexpr uint8_t ISM6HG256_INT = 21;
    static constexpr uint8_t BMP585_INT = 11;
    static constexpr int MMC5983MA_INT = 12;
    static constexpr uint8_t IIS2MDC_INT = 12;  // shared with MMC5983MA_INT

    // ### Camera (RunCam wiring bench-confirmed, #234/#251) ###
    // Direction in camera-side terms: GPIO31 listens to the camera's TX pad,
    // GPIO30 drives its RX pad. (The V7 schematic isn't in-repo; V8+ names
    // these nets FC-perspective — Camera_RX on 31, Camera_TX on 30, the
    // reverse of the GNSS convention — see the note in board_v8.h.)
    static constexpr int8_t CAM_PWR_PIN = 30;      // GoPro path, unused
    static constexpr int8_t CAM_SHUTTER_PIN = 31;  // GoPro path, unused
    static constexpr int8_t RUNCAM_RX_PIN = 31;    // FC receives (camera's TX pad)
    static constexpr int8_t RUNCAM_TX_PIN = 30;    // FC transmits (camera's RX pad)
    static constexpr int8_t RUNCAM_PWR_PIN = 32;   // camera power gate (CAM_ACT)

    // ### Pyro channels ###
    // One shared arming FET feeds all four squib drivers. ALL output-pad
    // init MUST go through safePyroOutputInit (see main.cpp) — never
    // gpio_reset_pin on a FIRE/ARM pad.
    static constexpr uint8_t PYRO_ARM_PIN   = 14;
    static constexpr uint8_t PYRO1_CONT_PIN = 15;
    static constexpr uint8_t PYRO1_FIRE_PIN = 16;
    static constexpr uint8_t PYRO2_CONT_PIN = 18;
    static constexpr uint8_t PYRO2_FIRE_PIN = 19;
    static constexpr uint8_t PYRO3_CONT_PIN = 38;
    static constexpr uint8_t PYRO3_FIRE_PIN = 34;
    static constexpr uint8_t PYRO4_CONT_PIN = 51;
    static constexpr uint8_t PYRO4_FIRE_PIN = 50;

    // ### Fin servos ###
    static constexpr uint8_t SERVO_PIN_1 = 43;
    static constexpr uint8_t SERVO_PIN_2 = 44;
    static constexpr uint8_t SERVO_PIN_3 = 45;
    static constexpr uint8_t SERVO_PIN_4 = 46;

    // ### Indicators ###
    static constexpr uint8_t PIEZO_PIN = 53;
    static constexpr uint8_t RED_LED_PIN = 2;
    static constexpr uint8_t BLUE_LED_PIN = 29;

    // ### I2C command/config channel to the OutComputer ###
    static constexpr uint8_t ESP_SDA_PIN = 41;
    static constexpr uint8_t ESP_SCL_PIN = 42;

    // ### I2S high-frequency telemetry to the OutComputer ###
    static constexpr int I2S_BCLK_PIN  = 27;
    static constexpr int I2S_WS_PIN    = 28;
    static constexpr int I2S_DOUT_PIN  = 23;
    static constexpr int I2S_FSYNC_PIN = 17;

    // ### Power gates (V8-only topology; -1 = rail not FC-switched) ###
    static constexpr int GPS_ACT_PIN = -1;
    static constexpr int SERVO_ACT_PIN = -1;
};
