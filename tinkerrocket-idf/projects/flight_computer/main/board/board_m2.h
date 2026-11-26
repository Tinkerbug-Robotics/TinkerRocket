#pragma once

#include <stdint.h>

// Tinker-Beetle V2 pin map for the FLIGHT COMPUTER (ESP32-S3FH4R2, U32).
// Selected with TR_BOARD_M2=1:  idf.py -B build_m2 -DTR_BOARD_M2=1 build
//
// Board files: hardware/tinker-beetle/ (TinkerRocket-Hardware), sheet
// fc_esp32s3.kicad_sch, as of 2026-11-23. Every constant below was taken from
// a kicad-cli netlist export of that schematic, compared pad by pad against the
// V1 netlist (tag rocket-computer-mini-v1.0.1). Walk U32's pad -> pinfunction
// -> net if you need to re-derive one.
//
// The V2 is the V1 with the GNSS receiver, the magnetometer and the boot flash
// changed, and the IMU and magnetometer moved to the BACK of the board.
// Everything not listed here is the V1's net for net: the pyro map, the power
// latch, the strapping pads and the I2S pairs, whose reasoning this file
// carries (the V1's own header went with its build when the V1 was retired).
// What changed on U32:
//
//   pad  GPIO  V1 (rev1)                V2 (this file)
//   6    1     MAG_SDA (QMC5883P I2C)    MAG_CS  (MMC5983MA SPI chip select)
//   7    2     MAG_SCL                   not connected
//   21   15    32 kHz crystal            not connected
//   22   16    32 kHz crystal            not connected
//   30-35      external boot flash       not connected (flash in the package)
//   36   48    not connected             IND_1 (D10, green)
//   37   47    not connected             PPS   (the PX100's 1PPS)
//   49   43    IND_1                     not connected
//
// ### THE FLIGHT COMPUTER ON THIS BOARD IS AN ESP32-S3, NOT A P4 ###
// Every board_v*.h in this project maps an ESP32-P4. This one does not. The
// topology is the same as rocket-computer's -- the FC starts off, the out
// computer switches it on, the FC holds its own rail up, and the FC owns
// all four pyro channels -- but the pin numbers come from a different part
// with far fewer usable pads, and several P4 subsystems do not exist here.
//
// ### The pyro map is the V1's, NOT the V9's. Read this before firing anything. ###
//   channel   V9 (P4)          here (S3)
//   ARM       16               44
//   1 FIRE    6                38
//   1 CONT    7                10
//   2 FIRE    11               35
//   2 CONT    10               11
//   3 FIRE    9                34
//   3 CONT    12               12   (coincidence, not a shared constant)
//   4 FIRE    13               33
//   4 CONT    14               42
// The FIRE pins moved on the V1 on 2026-09-03 (GPIO4/5/6/7 -> 38/35/34/33)
// when the sensor SPI group and the pyro group were swapped across the package
// to shorten the routing. Every one is a no-pull pad at reset per S3 datasheet
// Table 2-1 -- no FIRE line has a boot-time pull-up. They are on pads in the
// VDD3P3_CPU domain rather than VDD3P3_RTC. Three of them (PYRO2/3/4 on
// GPIO35/34/33) are on pads the datasheet lists as VDD_SPI/VDD3P3_CPU, whose
// supply follows the GPIO45 VDD_SPI strap; PYRO1 on GPIO38 is VDD3P3_CPU only.
// GPIO45 has an internal WPD and reads low at boot, giving 3.3 V. A 1.8 V
// VDD_SPI would leave three of the four FIRE lines unable to assert, which
// fails safe but would be a silent no-deploy.
//
// Nothing about these overlaps usefully. Building a V9 image for this board
// fires the wrong channel silently, which is why config.h refuses to default
// and asserts this map.
//
// ### What this board does not have ###
//   * no camera        -- no CAM_SHUTTER, no RunCam UART, no camera rail
//   * no servos        -- no fin control, no SERVO_ACT
//   * no separate GNSS rail -- the receiver is on V_MCU_SWTCH with us
//   * two indicator LEDs: IND_1 on GPIO48 and IND_2 on GPIO45 (a strapping
//     pad) -- see RED_LED_PIN / BLUE_LED_PIN
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
    // holds at 3.3 V ("Indicators" below).
    //
    // GPIO47 IS SPICLK_P (pad 37) AND GPIO48 IS SPICLK_N (pad 36), not the
    // other way round (ESP-IDF io_mux_reg.h: IO_MUX_GPIO47_REG =
    // PERIPHS_IO_MUX_SPICLK_P_U). They serve octal PSRAM only; this part's
    // PSRAM is quad, so both behave as ordinary GPIO.
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
    // pad from the V1's U2 (front, -45: Rz(-45), the mapping the V1 flew) to
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
    // BMP585_CS = 9 (pad 14) and ISM6HG256_INT = 8 (pad 13) since the V1's
    // 2026-09-03 pin swap moved the sensor SPI bus; both are asserted in
    // config.h. An older note put them on GPIO48 and GPIO47, which here carry
    // IND_1 and the PX100's PPS: a barometer whose chip select never asserts
    // and an IMU interrupt that never fires, and the build passes. GPIO41 is
    // MTDI, one of the four JTAG pads this board deliberately spends.
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

    // --- Pyro (ALL FOUR CHANNELS ARE OURS) — the V1's map ---
    // See the map comparison in the file header before changing anything.
    // Every one of these is an unconditionally-free GPIO -- no strapping pad,
    // no JTAG pad, no PSRAM pad -- except PYRO4_CONT, which spends MTMS, and
    // PYRO_ARM, which spends U0RXD. PYRO4_CONT is deliberate: continuity is a
    // digital read whose level at boot depends on whether an igniter happens
    // to be connected, so it must never sit on GPIO45 (which sets the flash
    // rail voltage and could stop the board booting). Spending a JTAG pad was
    // the cheaper trade.
    //
    // PYRO_ARM is GPIO44 (net FC_ARM). GPIO8 carries ISM6HG256_INT1; a
    // V9-or-earlier image drives GPIO8 and the arm FET never turns on --
    // nothing fires, which is the safe direction, but it fails a pad test
    // that looks like a dead igniter.
    //
    // GPIO44 is U0RXD, so it carries the ROM console's weak pull-up from
    // power-on until we configure the pad. That pull-up is real and it does
    // reach the arm FET's gate -- it is not enough on its own to arm the
    // board, because the out computer's consent transistor is in series and
    // boots off. Arming needs BOTH processors driving high; see the pyro
    // sheet's "Supervised arm" note and the README. Do not treat the OC's
    // consent as belt-and-braces and skip driving this pin low early.
    static constexpr int PYRO_ARM_PIN   = 44;  // FC_ARM     (pad 50, U0RXD)
    static constexpr int PYRO1_FIRE_PIN = 38;  // PYRO1_FIRE (pad 43)
    static constexpr int PYRO1_CONT_PIN = 10;  // PYRO1_CONT (pad 15)
    static constexpr int PYRO2_FIRE_PIN = 35;  // PYRO2_FIRE (pad 40)
    static constexpr int PYRO2_CONT_PIN = 11;  // PYRO2_CONT (pad 16)
    static constexpr int PYRO3_FIRE_PIN = 34;  // PYRO3_FIRE (pad 39)
    static constexpr int PYRO3_CONT_PIN = 12;  // PYRO3_CONT (pad 17)
    static constexpr int PYRO4_FIRE_PIN = 33;  // PYRO4_FIRE (pad 38)
    static constexpr int PYRO4_CONT_PIN = 42;  // PYRO4_CONT (pad 48, MTMS)
    // One arm for all four channels: no further arm lines (V5's per-channel
    // arms ride PYRO_ARM_GANG_PINS). Continuity reads with the arm down, and
    // LOW means a load is present (#264).
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
    // reset pull.
    //
    // GPIO45 (the VDD_SPI voltage strap) is safe as drawn, and the reason is
    // the LED, not the resistor: 10 k in series with a BLUE die (Vf ~2.7-3.0
    // V) is non-conducting near 0 V, so at reset the branch pulls the same
    // direction as the pad's internal pull-down and VDD_SPI still latches
    // 3.3 V. Driving it high after boot lights it. Do not add a pull-up here,
    // and do not move a lower-Vf colour onto this pad.
    //
    // The console is USB-Serial-JTAG on GPIO19/20 through the U1 mux, which
    // also carries flashing and JTAG. The host re-enumerates after every
    // reset, so the first fraction of a second of boot output is lost, and
    // the mux serves one MCU at a time. GPIO44 (U0RXD) is FC_ARM, so UART0's
    // IO_MUX receive pad is spent.
    static constexpr int RED_LED_PIN = 48;     // IND_1 (pad 36) -> D10 green
    static constexpr int BLUE_LED_PIN = 45;    // IND_2 (pad 51) -> D11 blue (VDD_SPI strap)

    // The hold-up sense (V_SCAP_ADC) is the out computer's, as on the V1.
    static constexpr int   SCAP_ADC_PIN       = -1;
    static constexpr float SCAP_DIVIDER_RATIO = 1.0f;   // unused with no pin

    // --- I2C master to the OutComputer ---
    static constexpr int ESP_SDA_PIN = 36;     // ESP_SDA (pad 41)
    static constexpr int ESP_SCL_PIN = 37;     // ESP_SCL (pad 42)

    // --- I2S master TX (high-frequency telemetry to the OutComputer) ---
    // The V1's four nets. BCLK and WS match the P4's V9 numbers; the DATA
    // PAIR DOES NOT. On the P4 the data and frame-sync nets are GPIO19/20, but
    // on an S3 those two pads are USB D-/D+ and this board spends them on the
    // FSUSB63UMX mux so either processor can be flashed over the single USB-C
    // port. 13/14 take their place. Both ends must agree PER NET; the OC side
    // is 2/1/3/4.
    //
    // ESP_I2S_FSYNC is NOT an I2S peripheral signal. Three of these four are
    // (BCLK/WS/SD); the fourth is a plain GPIO the master pulses around each
    // writeFrame() and the slave reads to resync. It shares the ESP_I2S_
    // prefix because it belongs to this link, not because the peripheral
    // drives it.
    static constexpr int I2S_BCLK_PIN  = 21;   // ESP_I2S_BCLK  (pad 27)
    static constexpr int I2S_WS_PIN    = 18;   // ESP_I2S_WS    (pad 24)
    static constexpr int I2S_DOUT_PIN  = 13;   // ESP_I2S_SD    (pad 18)
    static constexpr int I2S_FSYNC_PIN = 14;   // ESP_I2S_FSYNC (pad 19)

    // --- No peripheral rail gates of our own (the V1's U30 arrangement) ---
    // rocket-computer's FC gates the GNSS rail (GPS_ACT) and the servo rail
    // (SERVO_ACT). Neither exists here: one switch, U30, carries the GNSS,
    // the sensors, the radio and the NAND together, and the out computer
    // owns its enable.
    static constexpr int GPS_ACT_PIN = -1;
    static constexpr int SERVO_ACT_PIN = -1;
    static constexpr bool SERVO_ACT_SWITCHES_SUPPLY = false;   // #1768: no switch

    // ======================================================================
    // Power hold latch (#848) -- the V1's GPIO17, and why it is there
    // ======================================================================
    // GPIO17 is the second anode of the D9 diode-OR into U30's enable, so
    // driving it HIGH holds our own rail up regardless of the out computer.
    // LOW and high-Z are electrically identical through the diode -- an anode
    // cannot pull the rail down -- so this pin can only ever ADD power. Every
    // glitch is harmless and "release" just hands the rail back to the OC's
    // PWR_PIN. R84 (100 k) and C105 (10 uF) hold the enable up for roughly
    // 0.9 s after the last driver lets go. tau = 100 k x 10 uF = 1.0 s, the
    // node starts at 3.3 V minus a BAV170M drop (~0.4 V at the 28 uA R84
    // pulls) and has to fall to U30's V_ENF, so t = 1.0 s x ln(2.9/1.13) =
    // 0.94 s. (An older note claimed 1.4 s, which reproduces only as
    // ln(3.3/0.8): no diode drop AND a 0.8 V threshold, two errors pushing
    // the same way.) Derate C105 for DC bias -- it is an 0402 6.3 V X5R
    // sitting at ~2.9 V, which typically costs half its capacitance -- and the
    // real window is nearer 0.45 s.
    //
    // Note the decay is on the ENABLE node, not on V_MCU_SWTCH, so we stay at
    // full rail for the whole window and then drop sharply -- there is no
    // brownout race on the way down.
    //
    // None of this is load-bearing in flight: once pwrHoldAssert() has run,
    // GPIO17 holds the node itself and no decay is in progress. The window
    // only matters on the ground, where the OC alone drives the enable -- and
    // there a dropped rail is the DESIGNED behaviour (see pwrHoldAssert's
    // "NEVER asserted on the ground"). See #1270 for why the part was left
    // as-is.
    //
    // *** THIS PIN MUST STAY INSIDE GPIO0-21. ***
    // The assert is drive HIGH + gpio_hold_en, and on the ESP32-S3 that only
    // reaches the reset-surviving RTC latch for an RTC-capable pad: ESP-IDF
    // dispatches to rtc_gpio_hold_en() when rtc_gpio_is_valid_gpio() passes
    // and otherwise falls back to the digital hold, which is DEEP-SLEEP ONLY.
    // This part has SOC_RTCIO_PIN_COUNT = 22 (GPIO0-GPIO21) and
    // RTCIO_GPIO17_CHANNEL exists, so GPIO17 latches exactly the way the P4's
    // GPIO5 does. Move this above GPIO21 and gpio_hold_en still returns
    // ESP_OK, the latch still looks asserted, and it evaporates on the first
    // panic reset -- reintroducing #825 (an OC fault reset powering the FC and
    // all four pyro channels off mid-flight, ballistic) with no error to
    // notice it by.
    //
    // Deliberately NOT GPIO3, where the single-MCU map had the rail enable:
    // GPIO3 is a strapping pin (JTAG source select), and gpio_hold_en on it
    // would latch that strap HIGH through every in-flight reset. GPIO3 carries
    // VBUCK_OK (below), which is an input and never held. config.h asserts
    // both rules.
    static constexpr int PWR_HOLD_PIN = 17;    // FC_EN_HOLD (pad 23)

    // --- Hold-up supply monitor -- the V1's GPIO3 (ADC1_CH2) ---
    // Net VBUCK_OK, a divider off the buck's output ahead of the hold-up
    // converter, read as ADC1_CH2. It answers one question: is the main buck
    // still supplying, or are we running out of the hold-up capacitors?
    //
    // Read it as a level, not a flag. The divider sits at roughly 2.71 V
    // while the buck regulates and sags toward about 2.39 V once the board is
    // on cap energy alone, so a threshold near 2.55 V separates them with
    // margin either side. Both numbers are design-intent, not yet measured on
    // a built board -- re-derive them from hardware before gating anything
    // that matters on this.
    //
    // ADC1 is safe to sample while WiFi is up; ADC2 is not. That is why this
    // is on GPIO3 (ADC1) and not one of the free ADC2 pads.
    //
    // GPIO3's strapping role is JTAG source select and it is only sampled
    // when the EFUSE_STRAP_JTAG_SEL efuse is burned. It ships 0, so the pad
    // is a plain input at boot. Burning that efuse would make a divider that
    // idles near mid-rail a boot-time coin flip.
    static constexpr int VBUCK_OK_PIN = 3;     // VBUCK_OK (pad 8)
};
