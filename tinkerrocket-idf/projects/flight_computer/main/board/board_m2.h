#pragma once

#include <stdint.h>

// Tinker-Beetle V2 pin map for the FLIGHT COMPUTER (ESP32-S3FH4R2, U32).
// Selected with TR_BOARD_M2=1:  idf.py -B build_m2 -DTR_BOARD_M2=1 build
//
// Board files: hardware/tinker-beetle/ (TinkerRocket-Hardware), sheet
// fc_esp32s3.kicad_sch, as of 2026-11-23. Every constant below was taken from
// a kicad-cli netlist export of that schematic, compared pad by pad against the
// V1 netlist (tag rocket-computer-mini-v1.0.1) that board_m1.h was taken from.
// Walk U32's pad -> pinfunction -> net if you need to re-derive one.
//
// The V2 is the V1 with the GNSS receiver, the magnetometer and the boot flash
// changed, and the IMU and magnetometer moved to the BACK of the board.
// Everything not listed here is the V1's net for net, so board_m1.h's notes on
// the pyro map, the power latch, the strapping pads and the I2S pairs hold
// here unchanged; read them there. What changed on U32:
//
//   pad  GPIO  V1 (board_m1.h)          V2 (this file)
//   6    1     MAG_SDA (QMC5883P I2C)    MAG_CS  (MMC5983MA SPI chip select)
//   7    2     MAG_SCL                   not connected
//   21   15    32 kHz crystal            not connected
//   22   16    32 kHz crystal            not connected
//   30-35      external boot flash       not connected (flash in the package)
//   36   48    not connected             IND_1 (D10, green)
//   37   47    not connected             PPS   (the PX100's 1PPS)
//   49   43    IND_1                     not connected
//
// ### The pyro map is the V1's (board_m1.h), not the V9's ###
// config.h asserts it for M2 exactly as for M1.
//
// ### Boot flash: 4 MB, in the package ###
// The ESP32-S3FH4R2 boots from 4 MB of flash inside the package (and has 2 MB
// of quad PSRAM beside it); the V1's external NOR is gone. The base sdkconfig
// declares 8 MB for the shared partition table, which on this part is the
// fatal direction (the bootloader stops before any firmware runs), so the
// project CMakeLists gives an M2 build partitions_4mb.csv and the 4 MB
// declaration (sdkconfig.defaults.flash4mb), and main.cpp refuses an M2 image
// that does not declare 4 MB.
//
// ### GNSS: a SkyTraq PX100 (PX1105R) soldered on, on this processor's UART ###
// The LC86G is gone. The PX100 is the receiver the PX1105R carrier flies on the
// V9/V10's J1, driven by the same driver (TR_GNSS_DRIVER_PX1105R, which the
// project CMakeLists sets for M2), here on the V1's UART pads.
//
// ### The IMU and the magnetometer are on the BACK ###
// See "IMU and MMC5983MA placement" below: each reading is turned over as it is
// read, so the log, the out computer, the apps and every tool see a part on the
// front turned only about Z, as on every other board.
struct board_pins
{
    // --- Sensor SPI bus (IMU + barometer + magnetometer) ---
    // The V1's nets; the MMC5983MA joins them (U3 pads 1, 16, 5).
    static constexpr int SPI_SCK = 6;   // SENS_SCLK (pad 11)
    static constexpr int SPI_SDO = 7;    // SENS_SDO (pad 12) — sensors drive, we read
    static constexpr int SPI_SDI = 5;    // SENS_SDI (pad 10) — we drive, sensors read

    // --- GNSS (SkyTraq PX100, U5) ---
    // Net names are from the module's perspective, as on the V1: GNSS_TX is
    // U5's TXD1 (pin 19, 10 k pull-up R24), so we receive on it; GNSS_RX
    // reaches U5's RXD1 (pin 18) through R149 (1 k). No reset line, no
    // safeboot: a wedged receiver is recovered only by the out computer
    // cycling the whole rail, as on the V1.
    static constexpr int GNSS_RX = 39;         // net GNSS_TX (pad 44, MTCK), module->us
    static constexpr int GNSS_TX = 40;         // net GNSS_RX (pad 45, MTDO), us->module
    static constexpr int GNSS_RESET_N = -1;    // not wired
    static constexpr int GNSS_SAFEBOOT_N = -1; // not wired
    // The PX100's 1PPS (U5 pin 31) reaches GPIO47 (pad 37, SPICLK_P) beside
    // the D7 LED it lights. A fact of the board, not yet read: the PX1105R
    // driver takes no PPS (config.h's GNSS_PPS_PIN stays -1 for it), so the
    // pad is never configured. GPIO47 sits on VDD_SPI, which the GPIO45 strap
    // holds at 3.3 V (board_m1.h, "Indicators").
    static constexpr int GNSS_PPS_PAD = 47;    // PPS (pad 37) — not read yet

    // --- Sensor chip selects ---
    static constexpr int MMC5983MA_CS = 1;      // MAG_CS (pad 6, 10 k pull-up R118)
    static constexpr int BMP585_CS = 9;         // BMP585_CS   (pad 14)
    static constexpr int ISM6HG256_CS = 4;      // ISM6HG256_CS (pad 9)
    // No I2C magnetometer on this board: the IIS2MDC-named slot is empty.
    static constexpr int IIS2MDC_SDA = -1;
    static constexpr int IIS2MDC_SCL = -1;
    // The slot's rotations, for config::magRotZDeg(); unused with no part.
    static constexpr float MAG_ROT_Z_DEG_IIS2MDC  = 0.0f;
    static constexpr float MAG_ROT_Z_DEG_QMC5883P = 0.0f;

    // --- IMU and MMC5983MA placement (sensor -> board frame) ---
    // A reading is multiplied by AXIS_SIGN, chip axis by chip axis, as it is
    // read (SensorCollector::configureAxisSigns), and the converter then turns
    // it ROT_Z_DEG, CCW about +Z (+X forward, +Y left, +Z out of the front).
    // Both parts are on the BACK of this board, so both have signs: the sign
    // is the part turned over about the board's Y axis (X and Z reversed),
    // and the angle what is left. Everything after the read -- the log, the
    // out computer, the apps' converters, the Python tools -- then sees a part
    // on the front turned ROT_Z_DEG, as on every other board, and reads the
    // angle from the status query as it always has.
    //
    // U2, ISM6HG256X, back, rotation -45 in the layout. Carried over pad by
    // pad from the V1's U2 (front, -45: Rz(-45), the mapping the M1 flew) to
    // this one:
    //   board = R * chip,  R = [ -0.707 -0.707  0 ]
    //                          [ -0.707  0.707  0 ]
    //                          [  0      0     -1 ]  = Rz(+45) * diag(-1, 1, -1)
    // so chip Z points out of the back (board -Z), as a part turned over must.
    static constexpr float  ISM6HG256_ROT_Z_DEG    = 45.0f;
    static constexpr int8_t ISM6HG256_AXIS_SIGN[3] = {-1, 1, -1};
    // U3, MMC5983MA, back, rotation 0. Its datasheet axes (MEMSIC MMC5983MA,
    // "Sensing direction"): X+ from pin 1 toward pin 4, Y+ from pin 16 toward
    // pin 13, Z+ into the package -- a LEFT-handed set. On this layout pin 1
    // -> 4 runs toward board -Y, pin 16 -> 13 toward board -X, and "into the
    // package" from the back is board +Z:
    //   board = R * chip,  R = [  0 -1  0 ]
    //                          [ -1  0  0 ]
    //                          [  0  0  1 ]  = Rz(+90) * diag(-1, 1, 1)
    // det R = -1: that is the chip's handedness, not the board's. Turned over
    // (X and Z reversed) and made right-handed (Z reversed) leaves X alone
    // reversed. The V7, the only other board with this part, flies Rz(180)
    // with no sign and has never been checked against a dip angle.
    static constexpr float  MMC5983MA_ROT_Z_DEG    = 90.0f;
    static constexpr int8_t MMC5983MA_AXIS_SIGN[3] = {-1, 1, 1};

    // --- Which sensors this board actually has ---
    // The barometer is the V1's BMP581, on the USE_BMP585 seam.
    static constexpr bool USE_BMP585 = true;
    static constexpr bool USE_MMC5983MA = true;   // U3, on the sensor SPI bus
    static constexpr bool USE_GNSS = true;
    static constexpr bool USE_ISM6HG256 = true;
    // #1485, as on the V1: the ISM6 is read from its FIFO.
    static constexpr bool ISM6_FIFO_CAPTURE = true;
    static constexpr bool USE_IIS2MDC = false;    // no I2C magnetometer

    // --- Sensor interrupts ---
    static constexpr int ISM6HG256_INT = 8;    // ISM6HG256_INT1 (pad 13)
    static constexpr int BMP585_INT = 41;      // BMP585_INT     (pad 47, MTDI)
    // U3's INT (pad 15) is not connected: the collector polls the part's
    // status register instead (TR_Sensor_Collector, "MMC5983MA").
    static constexpr int MMC5983MA_INT = -1;
    static constexpr int IIS2MDC_INT = -1;     // no part

    // --- No camera on this board ---
    static constexpr int CAM_SHUTTER_PIN = -1;
    static constexpr int RUNCAM_RX_PIN = -1;
    static constexpr int RUNCAM_TX_PIN = -1;
    static constexpr int RUNCAM_PWR_PIN = -1;

    // --- Pyro (ALL FOUR CHANNELS ARE OURS) — the V1's, see board_m1.h ---
    static constexpr int PYRO_ARM_PIN   = 44;  // FC_ARM     (pad 50, U0RXD)
    static constexpr int PYRO1_FIRE_PIN = 38;  // PYRO1_FIRE (pad 43)
    static constexpr int PYRO1_CONT_PIN = 10;  // PYRO1_CONT (pad 15)
    static constexpr int PYRO2_FIRE_PIN = 35;  // PYRO2_FIRE (pad 40)
    static constexpr int PYRO2_CONT_PIN = 11;  // PYRO2_CONT (pad 16)
    static constexpr int PYRO3_FIRE_PIN = 34;  // PYRO3_FIRE (pad 39)
    static constexpr int PYRO3_CONT_PIN = 12;  // PYRO3_CONT (pad 17)
    static constexpr int PYRO4_FIRE_PIN = 33;  // PYRO4_FIRE (pad 38)
    static constexpr int PYRO4_CONT_PIN = 42;  // PYRO4_CONT (pad 48, MTMS)
    static constexpr int8_t PYRO_ARM_GANG_PINS[3] = {-1, -1, -1};
    static constexpr int  PYRO_CONT_CLOSED_LEVEL = 0;
    static constexpr bool PYRO_CONT_NEEDS_ARM    = false;

    // --- No servos on this board ---
    static constexpr int SERVO_PIN_1 = -1;
    static constexpr int SERVO_PIN_2 = -1;
    static constexpr int SERVO_PIN_3 = -1;
    static constexpr int SERVO_PIN_4 = -1;

    // --- Indicators ---
    // TWO LEDs, both cathode-to-GND through 10 k, both driven high to light:
    //   IND_1 -> R70 -> D10, GREEN -> GPIO48 (pad 36, SPICLK_N)
    //   IND_2 -> R71 -> D11, BLUE  -> GPIO45 (VDD_SPI voltage strap)
    // RED_LED_PIN is the code's name for the first indicator, not its colour:
    // D10 became green in the V2's parts consolidation. IND_1 moved off GPIO43
    // (U0TXD), which is bare now. GPIO48 is an ordinary pad on this part
    // (its PSRAM is quad, so the octal pads are free) with no strap role and no
    // reset pull. GPIO45 is the V1's, with the V1's reasoning (board_m1.h):
    // a blue die keeps the strap low at reset; never move a lower-Vf colour
    // onto it.
    static constexpr int RED_LED_PIN = 48;     // IND_1 (pad 36) -> D10 green
    static constexpr int BLUE_LED_PIN = 45;    // IND_2 (pad 51) -> D11 blue (VDD_SPI strap)

    // The hold-up sense (V_SCAP_ADC) is the out computer's, as on the V1.
    static constexpr int   SCAP_ADC_PIN       = -1;
    static constexpr float SCAP_DIVIDER_RATIO = 1.0f;   // unused with no pin

    // --- I2C master to the OutComputer ---
    static constexpr int ESP_SDA_PIN = 36;     // ESP_SDA (pad 41)
    static constexpr int ESP_SCL_PIN = 37;     // ESP_SCL (pad 42)

    // --- I2S master TX (high-frequency telemetry to the OutComputer) ---
    // The V1's four nets (board_m1.h explains 13/14 against the P4's 19/20).
    static constexpr int I2S_BCLK_PIN  = 21;   // ESP_I2S_BCLK  (pad 27)
    static constexpr int I2S_WS_PIN    = 18;   // ESP_I2S_WS    (pad 24)
    static constexpr int I2S_DOUT_PIN  = 13;   // ESP_I2S_SD    (pad 18)
    static constexpr int I2S_FSYNC_PIN = 14;   // ESP_I2S_FSYNC (pad 19)

    // --- No peripheral rail gates of our own (the V1's U30 arrangement) ---
    static constexpr int GPS_ACT_PIN = -1;
    static constexpr int SERVO_ACT_PIN = -1;

    // --- Power hold latch (#848) — the V1's GPIO17; see board_m1.h ---
    // MUST stay inside GPIO0-21 (the RTC hold), and never GPIO3; config.h
    // asserts both for M2 as for M1.
    static constexpr int PWR_HOLD_PIN = 17;    // FC_EN_HOLD (pad 23)

    // --- Hold-up supply monitor — the V1's GPIO3 (ADC1_CH2); see board_m1.h ---
    static constexpr int VBUCK_OK_PIN = 3;     // VBUCK_OK (pad 8)
};
