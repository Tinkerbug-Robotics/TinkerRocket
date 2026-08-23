#pragma once

#include <stdint.h>

// V9/V10 PCB pin map + board topology for the flight computer (ESP32-P4).
// Selected with TR_BOARD_V9=1:
//   idf.py -B build_v9 -DTR_BOARD_V9=1 build
//
// This is the map of the board files in hardware/rocket-computer/ — the
// .kicad_pcb title block reads V9 up to 25fb08d and V10 at HEAD. It is NOT
// board_v8.h: no V8 PCB was ever committed (the physical V8 bench boards
// predate the import), and three pyro pins moved between them.
//
// Verified 2026-08-17 two independent ways: `kicad-cli sch export netlist` on
// rocket-computer.kicad_sch, and a pad -> pinfunction -> net walk of U17 in
// rocket-computer.kicad_pcb at every commit that has ever touched that file
// (199ecec, bdcc507, 8f87aa0, 25fb08d, HEAD). Every GPIO below is identical
// across all of them; the one thing that ever changed is GPIO5, unconnected
// in the first two and wired to P4_EN_HOLD from the V9 pre-fab close-out
// (8f87aa0) onward. So this single header covers V9 and V10.
//
//   signal        board_v8.h    here (V9/V10)
//   PYRO_ARM           5             16
//   PYRO2_FIRE         9             11
//   PYRO3_FIRE        11              9
//
// Everything else — all four CONT pins, PYRO1/PYRO4 FIRE, and every sensor,
// servo, I2S, I2C, indicator and power-gate pin — is the same on both maps.
//
// ### Why flashing V9/V10 hardware with TR_BOARD_V8=1 is a silent hazard ###
//   * ARM never fires. On this board the arming FET is U9 (CSD16323Q3),
//     gate driven from PYRO_ARM through R21 (100 R), drain on PYRO_GND (the
//     squib return at J2). With GPIO16 undriven the return never reaches
//     ground and the only path left is R73, a 1 k bleed — roughly 8 mA off
//     the pack. That is enough to light a bench LED and nowhere near enough
//     to fire a real igniter.
//   * Channels 2 and 3 fire each other's connector, while PYRO2_CONT and
//     PYRO3_CONT still read their own channel correctly — so the app shows
//     the right channel armed and continuous, and the wrong squib goes.
//   * GPIO5 is this board's power latch (P4_EN_HOLD, below), not ARM, so an
//     arm request holds the FC's own rail up and a disarm releases it.
//
// ### SAFETY: first power-up of this revision ###
// Carried forward from board_v8.h and NOT yet done on V9/V10 hardware: with
// NO squibs connected, scope all four FIRE pads and the ARM pad across reset
// and the whole boot sequence and confirm they stay quiet. All output init
// goes through safePyroOutputInit() (main.cpp); check each pin's P4 IO-MUX
// default function before trusting it — the V7 hazard was pins 14-19
// defaulting to SPI2/SPI3, see commit 421dd63. Note that the pins this
// revision uses for ARM (16) and PYRO2/PYRO3 FIRE (11, 9) were NOT part of
// the V8 bring-up scope check, so none of them has been cleared yet.
struct board_pins
{
    // ### Sensor SPI bus (SENS_* nets) ###
    static constexpr uint8_t SPI_SCK = 53;  // SENS_SCLK
    static constexpr uint8_t SPI_SDO = 51;  // SENS_SDO
    static constexpr uint8_t SPI_SDI = 52;  // SENS_SDI

    // ### GNSS serial ###
    // Same nets and same label-perspective trap as V8: the nets are named
    // from the MODULE's point of view, not ours —
    //   net GNSS_TX = P4 GPIO3 -> J1 -> receiver TXD (it drives)
    //   net GNSS_RX = P4 GPIO4 -> J1 -> receiver RXD (it listens)
    // so OUR receive pin is 3 and OUR transmit pin is 4. These constants are
    // the FC's own rx/tx (passed straight to uartBegin(baud, rx, tx)) and
    // must stay in that order; do not "fix" them to match the net names.
    // GNSS_RXD2 on GPIO2 (the receiver's second UART) is unused by firmware.
    static constexpr uint8_t GNSS_RX = 3;   // FC receives; module TXD drives this
    static constexpr uint8_t GNSS_TX = 4;   // FC drives;   module RXD listens here
    static constexpr int8_t GNSS_RESET_N = -1;
    static constexpr int8_t GNSS_SAFEBOOT_N = -1;

    // ### Sensor CS / bus pins ###
    static constexpr int MMC5983MA_CS = -1;    // no MMC5983MA on V9/V10
    static constexpr uint8_t BMP585_CS = 41;   // BMP585_CS
    static constexpr uint8_t ISM6HG256_CS = 49;// ISM6HG256_CS
    static constexpr uint8_t IIS2MDC_SDA = 47; // IIS2MDCTR_SDA
    static constexpr uint8_t IIS2MDC_SCL = 48; // IIS2MDCTR_SCL

    // ### Part presence ###
    static constexpr bool USE_BMP585 = true;
    static constexpr bool USE_MMC5983MA = false;  // no MMC5983MA net on this board
    static constexpr bool USE_GNSS = true;
    static constexpr bool USE_ISM6HG256 = true;
    static constexpr bool USE_IIS2MDC = true;

    // ### Sensor interrupt pins ###
    static constexpr uint8_t ISM6HG256_INT = 50;  // ISM6HG256_INT1
    static constexpr uint8_t BMP585_INT = 42;     // BMP585_INT
    static constexpr int MMC5983MA_INT = -1;      // no MMC5983MA on V9/V10
    static constexpr uint8_t IIS2MDC_INT = 46;    // IIS2MDCTR_INT

    // ### Camera ###
    // Unchanged from V8 (Camera_TX=30, Camera_RX=31, CAM_ACT=32). These nets
    // are named from the FC's perspective — Camera_TX is the line the FC
    // transmits on, into the camera's RX pad — the OPPOSITE of the
    // GNSS_TX/GNSS_RX convention above. Bench-proven on V8 hardware with two
    // RunCam units; don't "fix" them to a module-perspective reading.
    //
    // What DID change: CAM_ACT now enables U26, a TPS22811 HIGH-side switch
    // whose output is the camera V+ on J4.1 (V8 switched the camera's ground
    // through a low-side MOSFET). Still active-high, so firmware is unchanged,
    // but the connector's polarity is not what a V8 harness expects. The
    // switch's IMON is read by the OutComputer, not by us.
    // GoPro shutter = net Camera_TX -> R32 (1k) -> J4.4, pulled to ground for a
    // short press. Same pad as RUNCAM_TX_PIN — one mode owns it at a time. No
    // separate GoPro power pin: power is the CAM_ACT gate below. Unlike V8, V9
    // does have 1k series resistors (R30/R32) on both camera signal lines.
    static constexpr int8_t CAM_SHUTTER_PIN = 30;  // GoPro shutter (net Camera_TX, J4.4)
    static constexpr int8_t RUNCAM_RX_PIN = 31;    // FC receives (net Camera_RX, camera's TX pad)
    static constexpr int8_t RUNCAM_TX_PIN = 30;    // FC transmits (net Camera_TX, camera's RX pad)
    static constexpr int8_t RUNCAM_PWR_PIN = 32;   // camera power gate (CAM_ACT)

    // ### Pyro channels — ARM and FIRE 2/3 differ from V8 (see header note) ###
    // One shared arming FET (U9) feeds all four squib drivers; each FIRE pin
    // drives a DTC123J (Q2-Q5). ALL output-pad init MUST go through
    // safePyroOutputInit() — never gpio_reset_pin() a FIRE or ARM pad.
    static constexpr uint8_t PYRO_ARM_PIN   = 16;  // PYRO_ARM   (V8: 5)
    static constexpr uint8_t PYRO1_FIRE_PIN = 6;   // PYRO1_FIRE
    static constexpr uint8_t PYRO1_CONT_PIN = 7;   // PYRO1_CONT
    static constexpr uint8_t PYRO2_FIRE_PIN = 11;  // PYRO2_FIRE (V8: 9)
    static constexpr uint8_t PYRO2_CONT_PIN = 10;  // PYRO2_CONT
    static constexpr uint8_t PYRO3_FIRE_PIN = 9;   // PYRO3_FIRE (V8: 11)
    static constexpr uint8_t PYRO3_CONT_PIN = 12;  // PYRO3_CONT
    static constexpr uint8_t PYRO4_FIRE_PIN = 13;  // PYRO4_FIRE
    static constexpr uint8_t PYRO4_CONT_PIN = 14;  // PYRO4_CONT

    // ### Fin servos ###
    // Same four EXP nets as V8, same P4 pins. Full expansion map (net, GPIO,
    // package pin), read out of the V10 .kicad_pcb:
    //
    //   net       GPIO   pkg pin      net       GPIO   pkg pin
    //   EXP_01     45      87         EXP_07     29      58
    //   EXP_02     44      86         EXP_08     28      57
    //   EXP_03     43      84         EXP_09     38      70
    //   EXP_04     54      98         EXP_10     37      69
    //   EXP_05     39      80         EXP_11     34      65
    //   EXP_06     40      81         EXP_12     33      64
    //
    // All twelve come out on the 16-pin Molex J3: pins 3..8 carry EXP_01..06
    // ascending, then pins 9..14 carry EXP_12..07 DESCENDING — pin 9 is
    // EXP_12, not EXP_07.
    //
    // CONNECTOR POLARITY CHANGED vs V8: J3 pins 1-2 are now the SWITCHED
    // POSITIVE servo rail (output of U28, a TPS22811 high-side switch gated
    // by SERVO_ACT) and pins 15-16 are GND. On V8 that was the other way
    // around — pins 1-2 a MOSFET-switched return, pins 15-16 VBATT. A V8
    // servo harness plugged into a V9/V10 board is reverse-polarity.
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
    static constexpr uint8_t ESP_SDA_PIN = 23;  // ESP_SDA (OC GPIO5)
    static constexpr uint8_t ESP_SCL_PIN = 22;  // ESP_SCL (OC GPIO6)

    // ### I2S high-frequency telemetry to the OutComputer ###
    // ESP_* net convention (must match the OC's board header per-net):
    //   ESP_SCLK = bit clock, ESP_SDO = FC data out, ESP_CS = word select,
    //   ESP_SDI = frame sync.
    static constexpr int I2S_BCLK_PIN  = 21;  // ESP_SCLK (OC GPIO2)
    static constexpr int I2S_WS_PIN    = 18;  // ESP_CS   (OC GPIO1)
    static constexpr int I2S_DOUT_PIN  = 19;  // ESP_SDO  (OC GPIO3)
    static constexpr int I2S_FSYNC_PIN = 20;  // ESP_SDI  (OC GPIO4)

    // ### Power gates ###
    // Same pins as V8, but both now enable TPS22811 high-side switches
    // (GPS_ACT -> U27, SERVO_ACT -> U28) instead of low-side MOSFETs. Both
    // are still active-high enables, so the firmware sequence is unchanged.
    static constexpr int GPS_ACT_PIN = 15;    // GPS_ACT — GNSS rail enable
    static constexpr int SERVO_ACT_PIN = 8;   // SERVO_ACT — servo rail enable

    // ### Power latch — new on V9, no equivalent on V7/V8 ###
    // GPIO5 = net P4_EN_HOLD, one anode of D9 (BAV170M dual diode). The other
    // anode is P4_EN_S3, which is the OutComputer's PWR_PIN (its GPIO7). The
    // common cathode is POWER_SWITCH = EN/UVLO of U30 (TPS22810), and U30's
    // output V_MCU_SWTCH feeds every P4 supply pin. So the OC switches this
    // board on, and either MCU can hold it on.
    //
    // #848: firmware DRIVES this pin while INFLIGHT (asserted in
    // enterInflight and the reboot-recovery restore; released at LANDED, sim
    // reset, and boot reconciliation — pwr_hold_policy.h has the full table).
    // The assert uses gpio_hold_en, so the pad stays latched HIGH through the
    // FC's own panic/WDT resets: an OC fault reset can no longer power the FC
    // off mid-flight (#825), and neither can the FC's own crash. Outside
    // flight the pin is driven LOW (== high-Z through D9 — an anode cannot
    // pull the rail down), so ground power-off via the OC works unchanged.
    // For reference, R84 (100 k) and C105 (10 uF) on POWER_SWITCH hold the
    // rail roughly 0.8 s after the last driver lets go.
    static constexpr int PWR_HOLD_PIN = 5;    // P4_EN_HOLD (held while INFLIGHT, #848)
};
