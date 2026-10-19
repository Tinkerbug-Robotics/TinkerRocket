#pragma once

#include <stdint.h>

// V5 ("TinkerRocket Full V5", fabbed 2026-04-17) pin map + board topology for
// the flight computer (ESP32-P4). Selected with TR_BOARD_V5=1:
//   idf.py -B build_v5 -DTR_BOARD_V5=1 build
//
// Derived from a netlist export of the archived project (Circuit Board Designs
// ARCHIVE 2026-07-24/TinkerRocket Full V5). Its Gerbers carry exactly the net
// names of that schematic, so the schematic is the board that was built; the
// .txt BOM beside it is older (2 pyro channels, MMC5983MA) and wrong.
//
// V7 is a respin of this layout: every part sits in the same place and every
// MCU pad but these carries the same net. Differences from board_v7.h:
//   - Pyro: one ARM per channel (V7 has one shared ARM), and FIRE/CONT swapped
//     on channels 1-3. A V7 image on a V5 can fire nothing: it drives the
//     channel 1-3 CONT lines as FIRE and never raises the channel 2-4 arms.
//   - Pyro continuity is the squib driver's STATUS output: it reads only while
//     the arm powers the driver, and HIGH means a load is present (V7+: LOW).
//   - Camera: CAM_ACT on 30 and Camera_TX on 32, the reverse of V7.
//   - No SERVO_ACT (V7's GPIO22 is PYRO2_ARM here) and no GPS rail (V7's
//     OC GPIO33 GPS_ACT is an unconnected NEOPIX label here).
//
// ONLY pins and part-presence flags live here. Logic, rates, gains, and
// protocol constants stay in config.h and must not fork per board revision.
struct board_pins
{
    // ### Sensor SPI bus ###
    static constexpr uint8_t SPI_SCK = 9;    // SENS_SCLK
    static constexpr uint8_t SPI_SDO = 10;   // SENS_SDO
    static constexpr uint8_t SPI_SDI = 8;    // SENS_SDI

    // ### GNSS serial (J3) ###
    // Module-side names, as on V7. The driver auto-detects an RX/TX swap at
    // boot. GNSS_RXD2 (J3.4, GPIO5) is not used. The receiver is on VPP (J3.5):
    // no reset line, no safeboot line, no separate rail.
    static constexpr uint8_t GNSS_RX = 3;
    static constexpr uint8_t GNSS_TX = 4;
    static constexpr int8_t GNSS_RESET_N = -1;
    static constexpr int8_t GNSS_SAFEBOOT_N = -1;

    // ### Sensor CS / bus pins ###
    // The magnetometer is an IIS2MDC (U23) on its own I2C pair. No MMC5983MA
    // is fitted, so GPIO13 is the IIS2MDC's SDA only.
    static constexpr int MMC5983MA_CS = -1;    // no MMC5983MA on V5
    static constexpr uint8_t BMP585_CS = 6;
    static constexpr uint8_t ISM6HG256_CS = 7;
    static constexpr uint8_t IIS2MDC_SDA = 13; // IIS2MDCTR_SDA
    static constexpr uint8_t IIS2MDC_SCL = 20; // IIS2MDCTR_SCL
    // IIS2MDC -> board rotation (deg, CCW about +Z) on its normalized frame.
    // U23 is on F.Cu at rotation 0 with the ISM6HG256 (U5) at 45, exactly as
    // V9 places them (U3 at 0, U2 at 45), so V9's bench-measured +90 holds.
    static constexpr float MAG_ROT_Z_DEG_IIS2MDC = 90.0f;

    // ### Part presence ###
    static constexpr bool USE_BMP585 = true;
    static constexpr bool USE_MMC5983MA = false;  // no MMC5983MA net on this board
    static constexpr bool USE_GNSS = true;
    static constexpr bool USE_ISM6HG256 = true;
    // One read per DRDY edge, as on V7: the FIFO path (#1485) has not been
    // benched on this board.
    static constexpr bool ISM6_FIFO_CAPTURE = false;
    static constexpr bool USE_IIS2MDC = true;

    // ### Sensor interrupt pins ###
    static constexpr uint8_t ISM6HG256_INT = 21;  // ISM6HG256_INT1
    static constexpr uint8_t BMP585_INT = 11;
    static constexpr int MMC5983MA_INT = -1;      // no MMC5983MA on V5
    static constexpr uint8_t IIS2MDC_INT = 12;    // IIS2MDCTR_INT

    // ### Camera (J6) ###
    // FC-perspective net names, as on V8 (see the note in board_v8.h):
    // Camera_TX (GPIO32) -> J6.4, Camera_RX (GPIO31) -> J6.3, no series
    // resistors. J6 has V8's pinout, so the GoPro shutter is the line the FC
    // drives, Camera_TX — the same pad as RUNCAM_TX_PIN. CAM_ACT (GPIO30)
    // drives Q3 through 1k, a low-side switch on the camera's return (J6.1);
    // J6.2 is always-on VCC through F5.
    static constexpr int8_t CAM_SHUTTER_PIN = 32;  // GoPro shutter (net Camera_TX, J6.4)
    static constexpr int8_t RUNCAM_RX_PIN = 31;    // FC receives (net Camera_RX, camera's TX pad)
    static constexpr int8_t RUNCAM_TX_PIN = 32;    // FC transmits (net Camera_TX, camera's RX pad)
    static constexpr int8_t RUNCAM_PWR_PIN = 30;   // camera power gate (CAM_ACT)

    // ### Pyro channels ###
    // Each channel is a VN5E160 high-side driver (U9/U10/U21/U22). Its supply
    // comes through a TPN4R712MD P-FET (U7/U24/U25/U26) whose gate a DTC123J
    // (Q2/Q1/Q5/Q6) pulls down when that channel's ARM is HIGH. FIRE drives
    // the driver's INPUT through 1k (10k pull-down). CONT is its STATUS output
    // through 1k, pulled up to 3V3 by 10k.
    //
    // The firmware drives the four ARM lines as ONE arm (owner, 2026-10-18):
    // PYRO_ARM_PIN plus PYRO_ARM_GANG_PINS move together, which is the
    // shared arm FET of V7-V10. A channel whose FIRE stays LOW keeps its
    // driver off while its supply is up.
    //
    // ALL output-pad init MUST go through safePyroOutputInit (see main.cpp) —
    // never gpio_reset_pin on a FIRE/ARM pad. GPIO15-19 default to SPI2 on the
    // IO MUX (the V7 boot-twitch, 421dd63); same pads here.
    //
    // CHANNEL 3 THROUGH BOOT. GPIO37 and GPIO38 are the P4's UART0 TXD and
    // RXD out of reset (and, with CONT3's GPIO34, strapping pins). So ARM3
    // sits on the boot ROM's console TX, which idles HIGH, and FIRE3 on its
    // RX, whose weak pull-up the 10k pull-down behind 1k should hold near
    // 0.6-0.8 V: under the VN5E160's 0.9 V input-low limit, not by much.
    // Expected result: from reset until initPyroPins() takes the pads, the
    // channel 3 driver is powered with its input only just LOW. No firmware
    // runs in that window. V7 moved both lines off these pads. This is
    // inferred from the P4's reset defaults and has not been scoped: do that
    // before a charge goes on channel 3 (docs/plans/v5-board-bring-up.md).
    static constexpr uint8_t PYRO_ARM_PIN   = 14;  // PYRO1_ARM (Q2)
    // The other three ARM lines, driven in lockstep with PYRO_ARM_PIN.
    static constexpr int8_t PYRO_ARM_GANG_PINS[3] = {
        22,  // PYRO2_ARM (Q1)
        37,  // PYRO3_ARM (Q5)
        49,  // PYRO4_ARM (Q6)
    };
    static constexpr uint8_t PYRO1_FIRE_PIN = 15;
    static constexpr uint8_t PYRO1_CONT_PIN = 16;
    static constexpr uint8_t PYRO2_FIRE_PIN = 18;
    static constexpr uint8_t PYRO2_CONT_PIN = 19;
    static constexpr uint8_t PYRO3_FIRE_PIN = 38;
    static constexpr uint8_t PYRO3_CONT_PIN = 34;
    static constexpr uint8_t PYRO4_FIRE_PIN = 50;
    static constexpr uint8_t PYRO4_CONT_PIN = 51;
    // Continuity is the VN5E160's off-state open-load diagnostic. With the
    // supply up and INPUT LOW, R35 (50k, VCC -> OUTPUT) pulls an open output
    // high and STATUS goes LOW; a squib holds the output down and STATUS
    // stays HIGH. Unarmed, the driver is unpowered and STATUS floats to the
    // pull-up: every channel reads "load present" with nothing connected. So
    // the arm must be up for the read (~0.2 mA through the squib via R35).
    static constexpr int  PYRO_CONT_CLOSED_LEVEL = 1;
    static constexpr bool PYRO_CONT_NEEDS_ARM    = true;

    // ### Fin servos (J7, EXP_01..EXP_04; EXP_05/06 on 47/48 unused) ###
    static constexpr uint8_t SERVO_PIN_1 = 43;
    static constexpr uint8_t SERVO_PIN_2 = 44;
    static constexpr uint8_t SERVO_PIN_3 = 45;
    static constexpr uint8_t SERVO_PIN_4 = 46;

    // ### Indicators ###
    // IND_2 -> D5 red, IND_1 -> D4 blue. GPIO53 (PIEZZO) is left undriven.
    static constexpr uint8_t RED_LED_PIN = 2;
    static constexpr uint8_t BLUE_LED_PIN = 29;

    // ### I2C command/config channel to the OutComputer ###
    static constexpr uint8_t ESP_SDA_PIN = 41;
    static constexpr uint8_t ESP_SCL_PIN = 42;

    // ### I2S high-frequency telemetry to the OutComputer ###
    // The nets are named for SPI (ESP_SCLK/ESP_CS/ESP_SDI/ESP_SDO) but the
    // wiring is V7's: FC 27->OC 21, 28->45, 23->2, 17->1.
    static constexpr int I2S_BCLK_PIN  = 27;  // ESP_SDI
    static constexpr int I2S_WS_PIN    = 28;  // ESP_SDO
    static constexpr int I2S_DOUT_PIN  = 23;  // ESP_SCLK
    static constexpr int I2S_FSYNC_PIN = 17;  // ESP_CS

    // ### Power gates (V8+ topology; -1 = rail not FC-switched) ###
    static constexpr int GPS_ACT_PIN = -1;
    static constexpr int SERVO_ACT_PIN = -1;

    // ### Power latch (V9+ only; -1 = the FC cannot hold its own rail) ###
    static constexpr int PWR_HOLD_PIN = -1;
};
