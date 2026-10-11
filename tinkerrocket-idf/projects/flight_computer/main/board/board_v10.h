#pragma once

#include <stdint.h>

// Tinker-Mantis V10 pin map + board topology for the flight computer
// (ESP32-P4, U17). Selected with TR_BOARD_V10=1:
//   idf.py -B build_v10 -DTR_BOARD_V10=1 build
//
// The map of the Tinker-Mantis V10 as ordered: tag tinker-mantis-v10.0.0
// (86aa08de, plotted 2026-09-30). The fabbed V9 is tag rocket-computer-v9.0.0
// and has its own map, board_v9.h. Until #1593 that header claimed to cover
// V10 as well; it does not, and a V9 image cannot arm a V10 (below).
//
// Derived 2026-10-10 (#1593) two ways: a pad -> pinfunction -> net walk of U17
// and U15 in tinker-mantis.kicad_pcb against the same walk of the V9 tag, and
// a `kicad-cli sch export netlist` of tinker-mantis.kicad_sch, which every
// circuit description below is traced from.
//
// The board in work after it is V11: the #1554 TPS1HC08-Q1 pyro switches, a
// stretched outline and the #1568 USB connector. As of 2026-10-10 it changes
// no processor pad, so this map would fit it, but its out computer's fault-trip
// threshold should not be V10's (#1595). Decide whether V11 gets its own build
// before it is ordered.
//
//   P4 GPIO   V9 (board_v9.h)    here (V10)
//     16      PYRO_ARM           unconnected
//     33      EXP_12             FC_ARM        <- the arm moved
//     46      IIS2MDCTR_INT      EXP_12
//     17      PIEZZO             V_SCAP_ADC    (left undriven, #1409)
//     47/48   IIS2MDC SDA/SCL    MAG_SDA/SCL   (U3 is a QMC5883P now)
//     28/29,34,37,38             same nets, J3 order reversed (servo block)
//
// Everything else (all four FIRE and CONT pins, sensor SPI and chip selects,
// GNSS, camera, servos 1-4, LEDs, I2C and I2S to the OC, both rail gates and
// the power latch) is on the same GPIO as V9. The I2S nets were renamed
// ESP_* -> ESP_I2S_*; same function. The OC side changes too: see
// out_computer/main/board/board_v10.h.
//
// ### The arm needs BOTH processors (supervised arm, as on the Beetle) ###
// FC_ARM -> R132 (100 R) -> Q12 (DTC123J) base; Q12's collector is Q14's
// emitter, and Q14's base is the OC's OC_ARM_EN (S3 GPIO14). Q14's collector
// pulls the base of Q13 (DTA123J, emitter on VBAT_CON), whose collector drives
// R139 (1 k) -> ARM_GATE -> R21 (100 R) -> the gate of U9 (AON7534), with R22
// (100 k) holding the gate down. U9 closes PYRO_GND, the squib return at J2.5,
// to ground. With U9 open the return reaches ground only through R73 (2.2 k),
// the continuity-sense path: enough to light a bench LED, nowhere near enough
// to fire an e-match.
//
// So this pin is half a vote. The OC raises its half only while serving a
// PYRO_FIRE_TEST or during a real (not simulated) flight; the rule is
// out_computer/main/arm_consent_policy.h. A channel that "fires" on the bench
// with only an LED lighting is this design working, not a dead igniter.
//
// ### Why flashing the wrong image is a silent no-deploy, both ways ###
//   * V9 image on a V10: it drives GPIO16 (unconnected) as ARM and never
//     touches GPIO33, and a V9 out-computer image has no consent pin, so U9
//     never closes. Continuity reads correctly and LEDs light through R73.
//   * V10 image on a V9: it drives GPIO33 (V9's EXP_12, J3.9) as ARM and
//     never touches the V9's real arm on GPIO16, so nothing fires there
//     either, and anything on J3.9 sees the arm pulse.
// The OTA receiver refuses a cross-board image (TR_BOARD_OTA_SUFFIX "-v10"),
// and the boot log prints the map (`[BOARD] pin map:` / `[BOARD] pyro:`);
// check it against the board before arming anything.
//
// ### SAFETY: first power-up of this revision — NOT YET DONE ###
// V10 is ordered but not yet built. With NO squibs connected, scope all four
// FIRE pads, FC_ARM (P4 GPIO33, either end of R132), OC_ARM_EN (S3 GPIO14, Q14
// base) and
// U9's gate across power-on, reset of each processor, and the whole boot
// sequence, and confirm they stay quiet. FC_ARM was not part of any earlier
// scope check: on V9 this pad was EXP_12. P4 GPIO33 is input-enabled with no
// pull at reset (datasheet pin 64) and is not a strapping pin, so Q12's
// built-in 47 k should hold it off until initPyroPins() drives it low through
// safePyroOutputInit() — confirm that on the scope, do not assume it. All
// output init goes through safePyroOutputInit() (main.cpp); never
// gpio_reset_pin() a FIRE or ARM pad (the V7 hazard, commit 421dd63). The
// first-article checklist is hardware/tinker-mantis/v10-bring-up.md.
struct board_pins
{
    // ### Sensor SPI bus (SENS_* nets) ###
    static constexpr uint8_t SPI_SCK = 53;  // SENS_SCLK
    static constexpr uint8_t SPI_SDO = 51;  // SENS_SDO
    static constexpr uint8_t SPI_SDI = 52;  // SENS_SDI

    // ### GNSS serial ###
    // Same nets, connector and label-perspective trap as V9: the nets are named
    // from the MODULE's point of view, so OUR receive pin is 3 (net GNSS_TX)
    // and OUR transmit pin is 4 (net GNSS_RX). These are the FC's own rx/tx,
    // passed straight to uartBegin(baud, rx, tx); do not "fix" them to match
    // the net names.
    //
    // J1 pin 5 (net GNSS_RXD2, GPIO2) is carrier-dependent: a PPS output on the
    // SAM-M10Q carrier, a receiver UART input on the px1105r. board_v9.h has
    // the full story (#837 item 4). GPIO2 stays unused until firmware can tell
    // which carrier is fitted.
    static constexpr uint8_t GNSS_RX = 3;   // FC receives; module TXD drives this
    static constexpr uint8_t GNSS_TX = 4;   // FC drives;   module RXD listens here
    static constexpr int8_t GNSS_RESET_N = -1;
    static constexpr int8_t GNSS_SAFEBOOT_N = -1;

    // ### Sensor CS / bus pins ###
    static constexpr int MMC5983MA_CS = -1;    // no MMC5983MA on V10
    static constexpr uint8_t BMP585_CS = 41;   // BMP585_CS (U4 is a BMP581, below)
    static constexpr uint8_t ISM6HG256_CS = 49;// ISM6HG256_CS
    // Magnetometer I2C. U3 is a QST QMC5883P (fixed address 0x2C), driven by
    // TR_QMC5883P through TR_Sensor_Collector's TR_MAG_DRIVER_QMC5883P seam,
    // which CMakeLists.txt sets for V10 and config.h requires. The IIS2MDC_*
    // names are the slot's, not the part's. Part, pull-ups (R40/R43, 5.11 k)
    // and this processor are all on V_MCU_SWTCH.
    static constexpr uint8_t IIS2MDC_SDA = 47; // MAG_SDA
    static constexpr uint8_t IIS2MDC_SCL = 48; // MAG_SCL

    // ### Magnetometer -> board rotation (deg, CCW about +Z) ###
    // Applied to the chip's normalized frame (magTypeChipSign in
    // RocketComputerTypes.h): right-handed, Z out of the top. U3 sits on F.Cu
    // at rotation 0; 180 is inferred from the Beetle's bench-measured axes
    // turned back by the 90 deg its footprint is turned (#1590). No V10 has
    // been built: confirm it on the first article, lying flat at rest, where
    // the field must dip about 65 deg DOWN.
    static constexpr float MAG_ROT_Z_DEG_QMC5883P = 180.0f;

    // ### Part presence ###
    // U4 is a BMP581, not V9's BMP585. The driver seam is shared (TR_BMP585
    // accepts either chip ID, 0x50 or 0x51) and USE_BMP585 selects it, as on
    // the Beetle.
    static constexpr bool USE_BMP585 = true;
    static constexpr bool USE_MMC5983MA = false;  // no MMC5983MA net on this board
    static constexpr bool USE_GNSS = true;
    static constexpr bool USE_ISM6HG256 = true;
    // #1485: FIFO capture, as on V9: same part, same bus and pins, same
    // interrupt. Benched on a V9 at 3,840 and 7,680 Hz, not yet on a V10.
    static constexpr bool ISM6_FIFO_CAPTURE = true;
    static constexpr bool USE_IIS2MDC = true;     // the mag slot; a QMC5883P behind it

    // ### Sensor interrupt pins ###
    static constexpr uint8_t ISM6HG256_INT = 50;  // ISM6HG256_INT1
    static constexpr uint8_t BMP585_INT = 42;     // BMP585_INT
    static constexpr int MMC5983MA_INT = -1;      // no MMC5983MA on V10
    // The QMC5883P has no DRDY pin, so the magnetometer is poll-only. GPIO46,
    // V9's IIS2MDCTR_INT, is EXP_12 on this board.
    static constexpr int IIS2MDC_INT = -1;

    // ### Camera ###
    // Unchanged from V9: Camera_TX=30 (R32 1 k), Camera_RX=31 (R30 1 k),
    // CAM_ACT=32 enabling U26 (TPS22811 high-side switch, camera V+ on J4.1).
    // The camera nets are named from the FC's perspective, the opposite of the
    // GNSS convention above. board_v9.h has the full notes.
    static constexpr int8_t CAM_SHUTTER_PIN = 30;  // GoPro shutter (net Camera_TX, J4.4)
    static constexpr int8_t RUNCAM_RX_PIN = 31;    // FC receives (net Camera_RX, camera's TX pad)
    static constexpr int8_t RUNCAM_TX_PIN = 30;    // FC transmits (net Camera_TX, camera's RX pad)
    static constexpr int8_t RUNCAM_PWR_PIN = 32;   // camera power gate (CAM_ACT)

    // ### Pyro channels — same FIRE/CONT pins as V9, ARM moved to GPIO33 ###
    // Each FIRE net carries a pull-down to GND (R78/R79/R80/R81 for channels
    // 1/2/3/4, 5.11 k), which holds the channel off while the pad is high-Z.
    // Behind it, a DTC123J (Q5/Q3/Q4/Q6) pulls the gate of a WSD20L50DN33
    // P-FET (U8/U6/U7/U10) whose source is VBAT_CON (not V9's V_CAP, which
    // does not exist here), with a 10 k gate pull-up to VBAT_CON
    // (R15/R16/R17/R23). V11 replaces each with a TPS1HC08-Q1 current-limited
    // switch under the same U references, EN fed from the FIRE net through the
    // same 10 k (#1554). Active-high on both. The designators are NOT in
    // channel order:
    //
    //   ch | FIRE net   | P4 pin | driver        | FET    | squib out
    //   ---+------------+--------+---------------+--------+-----------------
    //    1 | PYRO1_FIRE |      6 | Q5            | U8     | PYRO1_EXT (J2.1)
    //    2 | PYRO2_FIRE |     11 | Q3            | U6     | PYRO2_EXT (J2.2)
    //    3 | PYRO3_FIRE |      9 | Q4            | U7     | PYRO3_EXT (J2.3)
    //    4 | PYRO4_FIRE |     13 | Q6            | U10    | PYRO4_EXT (J2.4)
    //
    // CONT: each PYROn_CONT is the anode of a BAT54 (D1-D4) with a 100 k
    // (R9/R11/R13/R19), the same circuit as V9.
    //
    // ALL output-pad init MUST go through safePyroOutputInit(): never
    // gpio_reset_pin() a FIRE or ARM pad.
    static constexpr uint8_t PYRO_ARM_PIN   = 33;  // FC_ARM (V9: 16; needs OC_ARM_EN too)
    static constexpr uint8_t PYRO1_FIRE_PIN = 6;   // PYRO1_FIRE
    static constexpr uint8_t PYRO1_CONT_PIN = 7;   // PYRO1_CONT
    static constexpr uint8_t PYRO2_FIRE_PIN = 11;  // PYRO2_FIRE
    static constexpr uint8_t PYRO2_CONT_PIN = 10;  // PYRO2_CONT
    static constexpr uint8_t PYRO3_FIRE_PIN = 9;   // PYRO3_FIRE
    static constexpr uint8_t PYRO3_CONT_PIN = 12;  // PYRO3_CONT
    static constexpr uint8_t PYRO4_FIRE_PIN = 13;  // PYRO4_FIRE
    static constexpr uint8_t PYRO4_CONT_PIN = 14;  // PYRO4_CONT

    // ### Fin servos ###
    // Same four servo pins as V9. Full expansion map (net, GPIO, J3 pin):
    //
    //   net      GPIO  J3       net      GPIO  J3
    //   EXP_01    45    3       EXP_07    29    9
    //   EXP_02    44    4       EXP_08    28   10
    //   EXP_03    43    5       EXP_09    38   11
    //   EXP_04    54    6       EXP_10    37   12
    //   EXP_05    39    7       EXP_11    34   13
    //   EXP_06    40    8       EXP_12    46   14
    //
    // J3 pins 9..14 carry EXP_07..12 ASCENDING. The V9 has the same connector
    // with the opposite order (pin 9 = EXP_12 on GPIO33, which is FC_ARM here),
    // so a V9 harness on pins 9-14 does not carry over (schematic review
    // 2026-09-21). EXP_09/10/11 are the P4 strapping pins GPIO38/37/34, and
    // GPIO37 carries the ROM boot log (FABRICATION-NOTES B9).
    //
    // J3 pins 1-2 are the switched servo positive (U28, a TPS22811 gated by
    // SERVO_ACT) and pins 15-16 are GND, as on V9.
    static constexpr uint8_t SERVO_PIN_1 = 45;  // EXP_01, connector pin 3
    static constexpr uint8_t SERVO_PIN_2 = 44;  // EXP_02, connector pin 4
    static constexpr uint8_t SERVO_PIN_3 = 43;  // EXP_03, connector pin 5
    static constexpr uint8_t SERVO_PIN_4 = 54;  // EXP_04, connector pin 6

    // ### Indicators ###
    // IND_1 -> R66 (10 k) -> D7 red, IND_2 -> R70 (10 k) -> D8 blue, both
    // driven high to light: the same as the V9 first article (#847).
    //
    // No piezo. GPIO17 is V_SCAP_ADC: R125/R126 (100 k / 100 k) halve the
    // hold-up capacitor voltage onto it, with C143 (100 nF) at the pin.
    // Nothing reads it yet (#1409) and nothing may drive it.
    static constexpr uint8_t RED_LED_PIN = 27;    // IND_1
    static constexpr uint8_t BLUE_LED_PIN = 26;   // IND_2

    // ### I2C command/config channel to the OutComputer ###
    static constexpr uint8_t ESP_SDA_PIN = 23;  // ESP_SDA (OC GPIO5), R58 5.11 k
    static constexpr uint8_t ESP_SCL_PIN = 22;  // ESP_SCL (OC GPIO6), R55 5.11 k

    // ### I2S high-frequency telemetry to the OutComputer ###
    // Same GPIOs as V9, ESP_I2S_* net names (must match the OC's board header
    // per net). New on V10: R143 (100 k) pulls ESP_I2S_SD to ground, so the
    // OC's data input is defined while this processor is unpowered.
    // ESP_I2S_FSYNC is a plain GPIO the master pulses per frame, not an I2S
    // peripheral signal.
    static constexpr int I2S_BCLK_PIN  = 21;  // ESP_I2S_BCLK  (OC GPIO2)
    static constexpr int I2S_WS_PIN    = 18;  // ESP_I2S_WS    (OC GPIO1)
    static constexpr int I2S_DOUT_PIN  = 19;  // ESP_I2S_SD    (OC GPIO3)
    static constexpr int I2S_FSYNC_PIN = 20;  // ESP_I2S_FSYNC (OC GPIO4)

    // ### Power gates ###
    // Same pins and parts as V9: GPS_ACT -> U27 (TPS22810, no IMON),
    // SERVO_ACT -> U28 (TPS22811, IMON read by the OC on its GPIO9). Both are
    // active-high enables with a 100 k pull-down (R82/R87).
    static constexpr int GPS_ACT_PIN = 15;    // GPS_ACT — GNSS rail enable
    static constexpr int SERVO_ACT_PIN = 8;   // SERVO_ACT — servo rail enable

    // ### Power latch ###
    // As on V9: GPIO5 = P4_EN_HOLD, one anode of D9 (BAV170M); the other anode
    // is the OC's P4_EN_S3 (its GPIO7), and the common cathode POWER_SWITCH is
    // the enable of U30 (TPS22810), whose output V_MCU_SWTCH feeds this
    // processor. R84 (100 k) and C105 (10 uF) hold the enable up for a
    // fraction of a second after the last driver lets go; board_m1.h has the
    // arithmetic for the same parts.
    //
    // #848: held HIGH with gpio_hold_en while INFLIGHT (pwr_hold_policy.h).
    // GPIO5 is an LP IO pad, so the hold survives this processor's own panic
    // and watchdog resets.
    static constexpr int PWR_HOLD_PIN = 5;    // P4_EN_HOLD (held while INFLIGHT, #848)

    // P4 CHIP_PU is a named net on V10 (FC_CHIP_PU): R42 10 k to V_MCU_SWTCH
    // and C39 1 uF. Nothing else is on it, so neither processor can reset
    // this one except by cycling U30.
};
