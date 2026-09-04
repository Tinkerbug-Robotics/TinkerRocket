#pragma once

#include <stdint.h>

// rocket-computer-mini rev1 pin map for the FLIGHT COMPUTER (ESP32-S3RH2, U32).
// Selected with TR_BOARD_M1=1:  idf.py -B build_m1 -DTR_BOARD_M1=1 build
//
// Board files: hardware/rocket-computer-mini/, sheet fc_esp32s3.kicad_sch.
// Every constant below was taken from a kicad-cli netlist export of that
// schematic at the merge of the second-processor work, not from the design
// docs — walk U32's pad -> pinfunction -> net if you need to re-derive it.
//
// ### THE FLIGHT COMPUTER ON THIS BOARD IS AN ESP32-S3, NOT A P4 ###
// Every other board_v*.h in this project maps an ESP32-P4. This one does not.
// The topology is the same as rocket-computer's — the FC starts off, the out
// computer switches it on, the FC holds its own rail up, and the FC owns all
// four pyro channels — but the pin numbers come from a different part with
// far fewer usable pads, and several P4 subsystems simply do not exist here.
//
// ### The pyro map is NOT the V9 map. Read this before firing anything. ###
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
// The FIRE pins moved 2026-09-03 (GPIO4/5/6/7 -> 38/35/34/33) when the sensor
// SPI group and the pyro group were swapped across the package to shorten the
// routing. Every one is still a no-pull pad at reset per S3 datasheet Table
// 2-1 — no FIRE line acquired a boot-time pull-up. They are now on pads in the
// VDD3P3_CPU domain rather than VDD3P3_RTC. Three of them (PYRO2/3/4 on
// GPIO35/34/33) are on pads the datasheet lists as VDD_SPI/VDD3P3_CPU, whose
// supply follows the GPIO45 VDD_SPI strap; PYRO1 on GPIO38 is VDD3P3_CPU only.
// GPIO45 has an internal WPD and reads low at boot, giving 3.3 V. A 1.8 V
// VDD_SPI would leave three of the four FIRE lines unable to assert,
// which fails safe but would be a silent no-deploy.
//
// Nothing about these overlaps usefully. Building a V9 image for this board
// fires the wrong channel silently, which is why config.h refuses to default.
//
// ### What this board does not have ###
//   * no camera        — no CAM_SHUTTER, no RunCam UART, no camera rail
//   * no servos        — no fin control, no SERVO_ACT
//   * no piezo         — no sounds
//   * no separate GNSS rail — the receiver is on V_MCU_SWTCH with us
//   * two indicator LEDs: IND_1 on GPIO43 (U0TXD) and IND_2 on GPIO45 (a
//     strapping pad) — see RED_LED_PIN / BLUE_LED_PIN
//
// The magnetometer is OURS, on its own I2C bus (MAG_SCL/MAG_SDA), exactly as
// rocket-computer puts its IIS2MDCTR on the flight computer. It briefly was
// not: the single-MCU mini shared it onto the power-monitor bus to save two
// pins, and the second processor left it stranded on the out computer, which
// has no magnetometer driver at all. Moving it here cost two spare pads and
// closed that gap. Build this project with -DTR_MAG_DRIVER_QMC5883P.
struct board_pins
{
    // --- Sensor SPI bus (IMU + barometer) ---
    // Named for the NET, as in board_v9.h: SENS_SDI is the sensors' data in
    // (our MOSI) and SENS_SDO is their data out (our MISO). Do not swap them
    // to match a "SDO = our output" reading.
    static constexpr int SPI_SCK = 6;   // SENS_SCLK (pad 11)
    static constexpr int SPI_SDO = 7;    // SENS_SDO (pad 12) — sensors drive, we read
    static constexpr int SPI_SDI = 5;    // SENS_SDI (pad 10) — we drive, sensors read

    // --- GNSS (Quectel LC86G, UART only) ---
    // Net names are from the module's perspective: GNSS_TX is the module's
    // TXD, so we receive on it. No PPS to the MCU (1PPS drives LED D7), no
    // reset line, no safeboot. A wedged receiver is recovered only by the
    // out computer cycling the whole rail.
    static constexpr int GNSS_RX = 39;         // net GNSS_TX, module->us
    static constexpr int GNSS_TX = 40;         // net GNSS_RX, us->module
    static constexpr int GNSS_RESET_N = -1;    // not wired (CONFIRMED)
    static constexpr int GNSS_SAFEBOOT_N = -1; // not wired (CONFIRMED)

    // --- Sensor chip selects ---
    static constexpr int MMC5983MA_CS = -1;    // not fitted on the mini
    static constexpr int BMP585_CS = 9;        // BMP585_CS   (pad 14)
    static constexpr int ISM6HG256_CS = 4;      // ISM6HG256_CS (pad 9)
    // Magnetometer I2C. U3 is a QMC5883P, reached through the TR_IIS2MDC
    // component's TR_MAG_DRIVER_QMC5883P seam (#797) — hence the constant
    // names. Its own bus, like rocket-computer's IIS2MDCTR_SCL/SDA: the part,
    // its master and its pull-ups (R117/R118, 5.11 k) are all on
    // V_MCU_SWTCH, so nothing drives a pad whose supply is down.
    static constexpr int IIS2MDC_SDA = 1;     // MAG_SDA (CONFIRMED)
    static constexpr int IIS2MDC_SCL = 2;     // MAG_SCL (CONFIRMED)

    // --- Which sensors this board actually has ---
    // The part fitted is a BMP581, not a BMP585; the driver seam is shared
    // and USE_BMP585 selects it, as on the single-MCU mini map.
    static constexpr bool USE_BMP585 = true;
    static constexpr bool USE_MMC5983MA = false;  // not fitted
    static constexpr bool USE_GNSS = true;
    static constexpr bool USE_ISM6HG256 = true;
    static constexpr bool USE_IIS2MDC = true;     // QMC5883P via the #797 seam

    // --- Sensor interrupts ---
    // GPIO47/48 serve OCTAL PSRAM only; this part is quad, so they behave as
    // ordinary GPIO. GPIO41 is MTDI — one of the four JTAG pads this board
    // deliberately spends.
    //
    // GPIO47 IS SPICLK_P AND GPIO48 IS SPICLK_N, not the other way round
    // (ESP-IDF io_mux_reg.h: IO_MUX_GPIO47_REG = PERIPHS_IO_MUX_SPICLK_P_U).
    // These two constants were reversed until 2026-08-30 because pin-budget.md
    // paired them "GPIO47, GPIO48 = SPICLK_N / SPICLK_P". The schematic puts
    // BMP585_CS on pad 36 (SPICLK_N) and ISM6HG256_INT1 on pad 37 (SPICLK_P),
    // so the correct numbers are 48 and 47. With the old values the barometer
    // chip select would never have asserted and the IMU interrupt line would
    // have been driven as an output.
    static constexpr int ISM6HG256_INT = 8;    // ISM6HG256_INT1 (pad 13)
    static constexpr int BMP585_INT = 41;      // BMP585_INT     (CONFIRMED)
    static constexpr int MMC5983MA_INT = -1;   // not fitted
    // The QMC5883P land on this board exposes no DRDY/INT — every other pad
    // is NC — so the magnetometer is poll-only. Not an omission.
    static constexpr int IIS2MDC_INT = -1;     // no DRDY pin (CONFIRMED)

    // --- No camera on this board ---
    static constexpr int CAM_SHUTTER_PIN = -1;
    static constexpr int RUNCAM_RX_PIN = -1;
    static constexpr int RUNCAM_TX_PIN = -1;
    static constexpr int RUNCAM_PWR_PIN = -1;

    // --- Pyro (ALL FOUR CHANNELS ARE OURS) ---
    // See the map comparison in the file header before changing anything.
    // Every one of these is an unconditionally-free GPIO — no strapping pad,
    // no JTAG pad, no PSRAM pad — except PYRO4_CONT, which spends MTMS, and
    // PYRO_ARM, which spends U0RXD. PYRO4_CONT is deliberate: continuity is a
    // digital read whose level at boot depends on whether an igniter happens
    // to be connected, so it must never sit on GPIO45 (which sets the flash
    // rail voltage and could stop the board booting). Spending a JTAG pad was
    // the cheaper trade.
    //
    // PYRO_ARM is GPIO44 (net FC_ARM), NOT the GPIO8 the first single-MCU
    // build used. GPIO8 was briefly the hardware watchdog's pet line, and after
    // the 2026-09-03 pin swap it carries ISM6HG256_INT1; the arm moved to
    // GPIO44 and stayed there. A V9-or-earlier
    // image drives GPIO8 and the arm FET simply never turns on — nothing
    // fires, which is the safe direction, but it fails a pad test that looks
    // like a dead igniter.
    //
    // GPIO44 is U0RXD, so it carries the ROM console's weak pull-up from
    // power-on until we configure the pad. That pull-up is real and it does
    // reach the arm FET's gate — it is not enough on its own to arm the
    // board, because the out computer's consent transistor is in series and
    // boots off. Arming needs BOTH processors driving high; see the pyro
    // sheet's "Supervised arm" note and the README. Do not treat the OC's
    // consent as belt-and-braces and skip driving this pin low early.
    //
    // Cost of spending U0RXD: the serial console is TX-only on this board.
    static constexpr int PYRO_ARM_PIN   = 44;  // FC_ARM     (CONFIRMED; U0RXD)
    static constexpr int PYRO1_FIRE_PIN = 38;  // PYRO1_FIRE (pad 43)
    static constexpr int PYRO1_CONT_PIN = 10;  // PYRO1_CONT (CONFIRMED)
    static constexpr int PYRO2_FIRE_PIN = 35;  // PYRO2_FIRE (pad 40)
    static constexpr int PYRO2_CONT_PIN = 11;  // PYRO2_CONT (CONFIRMED)
    static constexpr int PYRO3_FIRE_PIN = 34;  // PYRO3_FIRE (pad 39)
    static constexpr int PYRO3_CONT_PIN = 12;  // PYRO3_CONT (CONFIRMED)
    static constexpr int PYRO4_FIRE_PIN = 33;  // PYRO4_FIRE (pad 38)
    static constexpr int PYRO4_CONT_PIN = 42;  // PYRO4_CONT (CONFIRMED; MTMS)

    // --- No servos, no piezo on this board ---
    static constexpr int SERVO_PIN_1 = -1;
    static constexpr int SERVO_PIN_2 = -1;
    static constexpr int SERVO_PIN_3 = -1;
    static constexpr int SERVO_PIN_4 = -1;
    static constexpr int PIEZO_PIN = -1;

    // --- Indicators ---
    // TWO LEDs, both cathode-to-GND through 10 k, both driven high to light:
    //   IND_1 -> R70 -> D10, RED   -> GPIO43 (U0TXD; was GPIO38 before the
    //                                 2026-09-03 swap - GPIO38 is PYRO1_FIRE now)
    //   IND_2 -> R71 -> D11, BLUE  -> GPIO45 (VDD_SPI voltage strap)
    // This header claimed one white LED on GPIO45 called D5 through R114 until
    // 2026-08-30. None of that matched the schematic: it named the blue LED
    // red, left the red one unclaimed, and cited parts that do not exist. Take
    // IND_1/IND_2 from the netlist, not from the reference designators here —
    // rocket-computer had the same colour swap on D7/D8 (fixed 2026-08-26).
    //
    // GPIO45 is safe as drawn, and the reason is the LED, not the resistor:
    // 10 k in series with a BLUE die (Vf ~2.7-3.0 V) is non-conducting near
    // 0 V, so at reset the branch pulls the same direction as the pad's
    // internal pull-down and VDD_SPI still latches 3.3 V. Driving it high
    // after boot lights it. Do not add a pull-up here, and do not move a
    // lower-Vf colour onto this pad.
    // IND_1 moved to GPIO43 (pad 49) after the 2026-09-03 pin swap; GPIO48 is
    // a bare pad again. GPIO43 is U0TXD, and GPIO44 is already FC_ARM, so the
    // UART0 *IO_MUX* pins are both spent. That is not the same as having no
    // console: the S3 routes any UART through the GPIO matrix, so a console can
    // be put on GPIO47 or GPIO48 (the only two free pads left) if one is ever
    // wanted. Day to day the console is USB-Serial-JTAG on GPIO19/20 through
    // the U1 mux — which also carries flashing and JTAG, so it is strictly more
    // capable than UART0 was. Its two limits are worth knowing: the host has to
    // re-enumerate after every reset, so the first fraction of a second of boot
    // output is lost, and the mux serves one MCU at a time.
    //
    // GPIO43 carries a weak pull-up at reset (S3 datasheet Table 2-1, U0TXD).
    // Through R70 that is ~73 uA into a red die — far below its conduction
    // knee, so the LED stays dark at boot and the strap-style caution that
    // applies to GPIO45 below does not apply here.
    static constexpr int RED_LED_PIN = 43;    // IND_1 (pad 49, U0TXD) -> D10 red
    static constexpr int BLUE_LED_PIN = 45;    // IND_2 -> D11 blue (VDD_SPI strap)

    // --- I2C master to the OutComputer ---
    static constexpr int ESP_SDA_PIN = 36;     // ESP_SDA (OC GPIO5)
    static constexpr int ESP_SCL_PIN = 37;     // ESP_SCL (OC GPIO6)

    // --- I2S master TX (high-frequency telemetry to the OutComputer) ---
    // BCLK and WS match the P4's V9 numbers; the DATA PAIR DOES NOT. On the
    // P4 the data and frame-sync nets are GPIO19/20, but on an S3 those two
    // pads are USB D-/D+ and this board spends them on the FSUSB63UMX mux so
    // either processor can be flashed over the single USB-C port. 13/14 take
    // their place. Both ends must agree PER NET; the OC side is 2/1/3/4.
    //
    // ESP_I2S_FSYNC is NOT an I2S peripheral signal. Three of these four are
    // (BCLK/WS/SD); the fourth is a plain GPIO the master pulses around each
    // writeFrame() and the slave reads to resync. It shares the ESP_I2S_
    // prefix because it belongs to this link, not because the peripheral
    // drives it.
    static constexpr int I2S_BCLK_PIN  = 21;   // ESP_I2S_BCLK  (OC GPIO2)
    static constexpr int I2S_WS_PIN    = 18;   // ESP_I2S_WS    (OC GPIO1)
    static constexpr int I2S_DOUT_PIN  = 13;   // ESP_I2S_SD    (OC GPIO3)  [P4: 19]
    static constexpr int I2S_FSYNC_PIN = 14;   // ESP_I2S_FSYNC (OC GPIO4)  [P4: 20]

    // --- No peripheral rail gates of our own ---
    // rocket-computer's FC gates the GNSS rail (GPS_ACT) and the servo rail
    // (SERVO_ACT). Neither exists here: one switch, U30, carries the GNSS,
    // the sensors, the radio and the NAND together, and the out computer
    // owns its enable.
    static constexpr int GPS_ACT_PIN = -1;
    static constexpr int SERVO_ACT_PIN = -1;

    // ======================================================================
    // Power hold latch (#848) — the whole reason this pin is where it is
    // ======================================================================
    // GPIO17 is the second anode of the D9 diode-OR into U30's enable, so
    // driving it HIGH holds our own rail up regardless of the out computer.
    // LOW and high-Z are electrically identical through the diode — an anode
    // cannot pull the rail down — so this pin can only ever ADD power. Every
    // glitch is harmless and "release" just hands the rail back to the OC's
    // PWR_PIN. R84 (100 k) and C105 (10 uF) hold the enable up roughly 1.4 s
    // after the last driver lets go; note the decay is on the ENABLE node,
    // not on V_MCU_SWTCH, so we stay at full rail for the whole window and
    // then drop sharply — there is no brownout race on the way down.
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
    // panic reset — reintroducing #825 (an OC fault reset powering the FC and
    // all four pyro channels off mid-flight, ballistic) with no error to
    // notice it by.
    //
    // Deliberately NOT GPIO3, where the single-MCU map had the rail enable:
    // GPIO3 is a strapping pin (JTAG source select), and gpio_hold_en on it
    // would latch that strap HIGH through every in-flight reset. GPIO3 now
    // carries VBUCK_OK (below), which is an input and never held.
    static constexpr int PWR_HOLD_PIN = 17;    // FC_EN_HOLD (CONFIRMED)

    // --- Hold-up supply monitor ---
    // Net VBUCK_OK, a divider off the buck's output ahead of the hold-up
    // converter, read as ADC1_CH2. It answers one question: is the main buck
    // still supplying, or are we running out of the hold-up capacitors?
    //
    // Read it as a level, not a flag. The divider sits at roughly 2.71 V
    // while the buck regulates and sags toward about 2.39 V once the board is
    // on cap energy alone, so a threshold near 2.55 V separates them with
    // margin either side. Both numbers are design-intent, not yet measured on
    // a built board — re-derive them from hardware before gating anything
    // that matters on this.
    //
    // ADC1 is safe to sample while WiFi is up; ADC2 is not. That is why this
    // is on GPIO3 (ADC1) and not one of the free ADC2 pads.
    //
    // GPIO3's strapping role is JTAG source select and it is only sampled
    // when the EFUSE_STRAP_JTAG_SEL efuse is burned. It ships 0, so the pad
    // is a plain input at boot. Burning that efuse would make a divider that
    // idles near mid-rail a boot-time coin flip.
    static constexpr int VBUCK_OK_PIN = 3;     // VBUCK_OK   (CONFIRMED; ADC1_CH2)
};
