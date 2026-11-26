#pragma once

#include <stdint.h>

// Tinker-Beetle V2 pin map for the OUT COMPUTER (ESP32-S3FH4R2, U15).
// Selected with TR_BOARD_M2=1:  idf.py -B build_m2 -DTR_BOARD_M2=1 build
//
// The Beetle V3 (in work) is this map plus one difference that changes what
// this processor may write: U23's ALERT drives a hardware arm trip
// (INA_ALERT_DRIVES_ARM_TRIP below). Its image adds TR_BEETLE_V3=1:
//   idf.py -B build_m3 -DTR_BOARD_M2=1 -DTR_BEETLE_V3=1 build
// It names itself M3 (TR_BOARD_REV_STR) and stamps "-m3" (version and OTA
// suffix), so a V2 image is refused over the air on a V3 and the reverse.
// No S3 pad moved between V2 and V3 as of 2026-11-26; walk the V3 netlist
// again before a V3 image goes on a board, as for V2 below.
//
// Board files: hardware/tinker-beetle/, title block V2. V2 is IN WORK, not
// ordered. Every constant below was taken on 2026-11-21 from a kicad-cli
// netlist export of the working schematic, by walking U15's
// pad -> pinfunction -> net, on the day the layout moved seven of this
// processor's signals. Repeat that walk against the ordered commit before a V2
// image goes on a board, and correct this file first if the two disagree.
// rev1 (tag rocket-computer-mini-v1.0.1) is retired, and its header with it;
// this file carries the reasoning of rev1's that still holds here.
//
// ### This board has TWO ESP32-S3s, not an S3 and a P4 ###
// rocket-computer splits the work between an ESP32-S3 out computer and an
// ESP32-P4 flight computer. The Beetle does the same split with an S3 on BOTH
// ends. That is why this file exists next to board_v7/v8/v9.h rather than
// being an alias: the topology is rocket-computer's, but the far side of the
// link is a different part with a different pin map, and the Beetle carries
// its radio and GNSS on-board instead of on daughterboards. Do not alias
// board_v9.h: the radio and GNSS blocks below are completely different, and a
// UART-modem backend on this board talks to nothing.
//
// What is NOT on this board, and why the constants are -1:
//   * no LoRa daughterboard  — the E220 is on-board, on the memory SPI bus
//   * no LoRa_ACT rail gate  — the radio sits on V_MCU_SWTCH with everything
//                              else behind U30, so PWR_PIN is its only switch
//   * no camera, no servo    — so no TPS22811 high-side switches, no IMON
//   * no separate GNSS rail  — the receiver is on V_MCU_SWTCH too
//
// ### Not yet run on hardware ###
// The other processor's map is flight_computer/main/board/board_m2.h. No V2
// board has been built: nothing here has run on hardware.
//
// ### Boot flash: 4 MB, in the package ###
// U15 is an ESP32-S3FH4R2 on V2: it boots from the 4 MB of flash in its
// package, and rev1's external 16 MB NOR is gone. The 8 MB every other board
// of this project declares would stop a V2 in its bootloader, so an M2 build
// takes sdkconfig.defaults.flash4mb: a 4 MB declaration and partitions_4mb.csv,
// whose two app slots are 1.875 MB where the others have 3 MB. main.cpp
// refuses to compile an M2 image that does not declare 4 MB.
//
// ### Seven signals moved in the V2 layout (2026-11-21) ###
// The hold-up charger (U47, L6, L11 and their zones) sits on the back of the
// board under U15's GPIO1-14 corner, so the signals on those pads had nowhere
// to drop a via. They moved to pads on the other two sides of the package:
//
//   net             rev1 GPIO   V2 GPIO   V2 pad
//   FC_EN_OC            7         48      36 (SPICLK_N)
//   L_CS               13         47      37 (SPICLK_P)
//   ESP_I2S_FSYNC       4         34      39
//   ESP_I2S_WS          1         39      44 (MTCK)
//   ESP_I2S_BCLK        2         40      45 (MTDO)
//   L_BUSY             14         41      47 (MTDI)
//   L_RXEN             10         42      48 (MTMS)
//
// ESP_I2S_SD (3), ESP_SDA (5), ESP_SCL (6) and every other pin are where rev1
// has them, and the flight computer's end of every net is unchanged. The link
// GPIOs therefore no longer equal board_v9.h's, and this file is the only
// place the V2's numbers live. config.h pins all seven with static_asserts.
//
// What the new pads do at reset, read from the ESP32-S3 datasheet v2.2
// (Table 2-1 pin overview, Table 2-2 power-up glitches, Table 3-1 strapping
// pins) on 2026-11-21:
//   * None of the seven is a strapping pin, and none is in the power-up glitch
//     table. rev1's GPIO1-14 all are: each drives low for ~60 us at power-up.
//   * GPIO47 and GPIO48: input enabled, no pull, at and after reset.
//   * GPIO34, GPIO40, GPIO41, GPIO42: input enabled after reset, no pull.
//   * GPIO39 (MTCK) is the exception: the chip turns on its weak pull-up
//     (45 k typical) after reset while EFUSE_DIS_PAD_JTAG is unburnt, which is
//     every board. I2S_WS_PIN below has what that costs and what setup_oc()
//     does about it. Never move FC_EN_OC or L_RXEN onto this pad: against
//     their 100 k pull-downs that pull-up reads HIGH.
//   * None is an RTC pad (those are GPIO0-21): no rtc_gpio hold and no ext
//     wake-up on these seven. Nothing in this project uses either on them.
//   * The four JTAG pads are spent, so this processor has no pad JTAG. Console
//     and debugger stay on USB Serial/JTAG (GPIO19/20), the chip's default
//     JTAG source.
//   * GPIO39-42 reset to their JTAG function, not to GPIO (Table 2-4; GPIO34,
//     47 and 48 reset to GPIO). gpio_config(), gpio_set_direction() and
//     gpio_reset_pin() each select the GPIO function (IDF 6.0), and every pin
//     below is set up through one of them or through a driver that does. A
//     register-level write to one of the four would have to select the
//     function itself.
//   * GPIO33-37, GPIO47 and GPIO48 are powered from VDD3P3_CPU or VDD_SPI as an
//     eFuse decides (Table 2-1 note 3). Both are 3.3 V on this part.
//
// ### What else differs from rev1 for this processor ###
//   * The part. rev1's ESP32-S3RH2 had no flash in the package; the FH4R2 has
//     4 MB of quad flash on SPICS0 beside the same 2 MB of quad PSRAM on
//     SPICS1. Pads 28 and 30-35 (GPIO26-32) are theirs and are no-connects on
//     the board. GPIO33-38 stay free for the memory bus: the GPIO33-37
//     restriction covers octal parts only.
//   * FC_EN_OC reaches POWER_SWITCH through D12, a diode of its own. D9 is the
//     flight computer's FC_EN_HOLD.
//   * Still the same: the INA230's ALERT (U23 pin 3) is not connected, the
//     radio and the NAND sit behind U30 on V_MCU_SWTCH, and the hold-up sense
//     is R125/R126 into GPIO8.
//
// Spare pads on V2: GPIO1, 2, 4, 7, 9, 10, 12, 13, 14 (bare, and not reachable
// in this layout) and GPIO43, 44, 45. GPIO43/44 keep the ROM console's reset
// pull-ups and GPIO45 is the VDD_SPI strap.
//
// KNOWN FIRMWARE GAP, carried over from rev1: LORA_RXEN_PIN is still not
// consumed by this project. TR_LoRa_Comms takes it (Config::rxen_pin), and
// main.cpp does not pass it, so R142 holds the E220's receive-enable low. The
// E220-900MM22S needs its RF switch driven; the base station has already been
// bitten by an RXEN left floating in RX. Wire it up before trusting a link
// budget.
struct board_pins
{
    // --- Flight-computer rail switch ---
    // Net FC_EN_OC into D12, onto POWER_SWITCH, the enable of U30 (TPS22810).
    // Raising this starts the flight computer AND everything else behind U30:
    // sensors, GNSS, radio, NAND. The FC then holds the rail up itself through
    // FC_EN_HOLD (D9), so dropping this pin powers the board down on the
    // ground but NOT during a flight (flight_computer/main/board/board_m2.h has
    // the hold policy and the R84/C105 arithmetic).
    //
    // MOVED on V2: GPIO7 -> GPIO48 (pad 36, SPICLK_N). The pad is a high-Z
    // input from reset until setup_oc() drives it, so R84 (100 k) holds
    // POWER_SWITCH low and the rail stays off, as on rev1; an out computer
    // reset in flight still relies on the R84/C105 window and the #825 rail
    // restore. Unlike GPIO7 it has no power-up glitch. It is not an RTC pad:
    // nothing here holds or wakes on it, and light sleep runs only with the
    // rail off, when this pin is low anyway.
    static constexpr int PWR_PIN = 48;       // FC_EN_OC (CONFIRMED; was 7)
    // No separate GNSS rail on this board: the receiver hangs off V_MCU_SWTCH
    // with the rest. -1 = call sites skip it. A wedged receiver is recovered
    // only by cycling PWR_PIN, which takes the radio, the sensors and the NAND
    // with it.
    static constexpr int GPS_PWR_PIN = -1;   // no separate rail (CONFIRMED)

    // --- Power monitoring (INA230 @ 0x40, always-on I2C bus) ---
    // U23, this MCU and the pull-ups R67/R69 (5.11 k) are all on the always-on
    // +3V3 rail, and nothing else is on the bus. Keep it that way: on an early
    // rev1 the magnetometer hung here while its own supply sat behind U30, and
    // those +3V3 pull-ups drove its pads against a rail the switch was
    // actively discharging — which clamped the bus below VIL and made the pack
    // monitor unreadable during pad standby.
    static constexpr int PWR_SDA = 21;       // SEN_SDA (CONFIRMED)
    static constexpr int PWR_SCL = 33;       // SEN_SCL (CONFIRMED)

    // --- Memory + radio shared SPI bus ---
    // The NAND (U11) and the radio (U16) share this host, as on rev1. Both are
    // behind U30, with R140 and R141 (100 k to V_MCU_SWTCH) holding their chip
    // selects high while this processor's pads are high-Z. Unlike
    // rocket-computer, the telemetry radio lives on THIS bus: the NAND's
    // page-sized bursts and the radio's twice-a-second packet share a host;
    // the sensors deliberately do not, because head-of-line blocking behind a
    // NAND page lands as jitter on the sample timebase.
    static constexpr int SPI_SCK = 37;       // M_SCK      (CONFIRMED)
    static constexpr int SPI_MISO = 35;      // M_MISO     (CONFIRMED)
    static constexpr int SPI_MOSI = 38;      // M_MOSI     (CONFIRMED)
    static constexpr int NAND_CS = 36;       // M_FLASH_CS (CONFIRMED)
    // No MRAM on this board and never was. As on V9/V10 this costs the FC's
    // in-flight reboot recovery and the dirty-ring replay their storage; the
    // NAND-log snapshot + boot tail-scan is the replacement.
    static constexpr int MRAM_CS = -1;       // not fitted (CONFIRMED)
    // The FH4R2's 2 MB of in-package quad PSRAM is the same PSRAM rev1's
    // S3RH2 carries, on the same chip select (SPICS1, GPIO26). Still volatile:
    // this buys ring SIZE, never reboot recovery.
    static constexpr bool RING_IN_PSRAM = true;   // 2 MB in-package (CONFIRMED)

    // #1485: the highest IMU logging rate this board flies. rev1's value,
    // because the IMU (U2, ISM6HG256X) and the link are rev1's. 7680 Hz needs
    // the FC's FIFO capture, which its board_m2.h turns on; the flight
    // computer derives its own limit from that, and the two must be equal:
    // test_imu_rate_board_parity pins that for M2. The OC rejects a setting
    // above it and reports it to the apps as "irmax".
    static constexpr uint16_t IMU_RATE_MAX_HZ = 7680;

    // --- Pyro arm consent (net OC_ARM_EN) ---
    // The out computer's half of the two-processor arm, unchanged from rev1:
    // into the base of Q14, in series with the flight computer's FC_ARM, and
    // the arm FET only turns on when BOTH are high. Neither processor can arm
    // the board alone, which is the whole point: an FC that hangs with its arm
    // line stuck high, or one whose GPIO44 is still sitting at the ROM
    // console's boot pull-up, cannot fire anything while we hold this low.
    //
    // Drive it LOW explicitly and EARLY — before the FC rail comes up through
    // PWR_PIN. This pad floats from reset until we configure it, and a
    // floating gate on a consent transistor is not a consent decision.
    // setup_oc() drives it low first thing. After that it goes high for
    // exactly two reasons: a PYRO_FIRE_TEST being served to the FC, or a real
    // (not simulated) flight in progress. The rule is in arm_consent_policy.h.
    // Without it the board cannot fire a channel at all.
    //
    // This is a real vote, not a formality. Do not wire it to "always high at
    // boot" for bench convenience; that silently collapses the design back to
    // a single-processor arm.
    static constexpr int ARM_CONSENT_PIN = 11;   // OC_ARM_EN (CONFIRMED)
    // U23's ALERT (pin 3) is still not connected on V2, so the fault-current
    // trip (pyro_fault_trip_policy.h) polls the shunt register, as on rev1.
    // On the V3 it is wired, but to the hardware arm trip only, not to this
    // processor: the poll stays this processor's half there too.
    static constexpr int INA_ALERT_PIN = -1;     // U23 ALERT open (CONFIRMED)
    // V3 (TR_BEETLE_V3): ALERT drives the hardware arm trip. R150 (100 k)
    // pulls it up; low turns on Q15, and Q16 pulls U9's gate low (branch
    // hardware/pyro-pfet-alert-trip, replacing the TPS2HC16 channel switches
    // with the V10's P-FETs). Anything the INA230 puts on ALERT then holds U9
    // off, so this image must never route CNVR there: with it in the guard
    // (V2's word, SOL | CNVR) a V3 could not fire. The V2 as ordered keeps its
    // TPS2HC16s and an open ALERT, and its words do not change.
#if TR_BEETLE_V3
    static constexpr bool INA_ALERT_DRIVES_ARM_TRIP = true;    // Q15/Q16 -> U9 (V3)
#else
    static constexpr bool INA_ALERT_DRIVES_ARM_TRIP = false;   // ALERT open (V2)
#endif
    // No eFuse power-good line to this processor, and VBUCK_OK goes to the
    // flight computer (its GPIO3), not here.
    static constexpr int PG_RAIL_PIN = -1;
    static constexpr int VBUCK_OK_PIN = -1;

    // --- Hold-up capacitor voltage (net V_SCAP_ADC) ---
    // R125 / R126 100 k / 100 k off the TPS61094's (U47) supercap, read as
    // ADC1_CH7: the pin sees V_SCAP / 2. C144 100 nF at the pin lets the 50 k
    // source drive the SAR sample-and-hold. Unchanged from rev1.
    static constexpr int SCAP_ADC_PIN = 8;       // V_SCAP_ADC (CONFIRMED; ADC1_CH7)
    static constexpr float SCAP_DIVIDER_RATIO = 2.0f;
    // Read on this processor (above), not relayed by the flight computer.
    static constexpr bool SCAP_FROM_FC = false;

    // --- I2C slave (commands from the FlightComputer) ---
    // Pull-ups R115/R116 (5.11 k) are on V_MCU_SWTCH, with the flight computer.
    static constexpr int I2C_SDA_PIN = 5;    // ESP_SDA (CONFIRMED; FC GPIO36)
    static constexpr int I2C_SCL_PIN = 6;    // ESP_SCL (CONFIRMED; FC GPIO37)

    // --- I2S slave RX (high-frequency telemetry from the FlightComputer) ---
    // Both ends must agree PER NET, not per role name. The flight computer's
    // end is where rev1 has it; three of our four moved on V2.
    //   ESP_I2S_BCLK = bit clock,   ESP_I2S_SD    = FC data out (our DIN),
    //   ESP_I2S_WS   = word select, ESP_I2S_FSYNC = frame sync.
    // ESP_I2S_FSYNC is a plain GPIO, not a peripheral signal: the FC pulses it
    // around each frame and we read it to resync. GPIO34 is above the first
    // GPIO bank, which gpio_get_level() handles; a direct read of GPIO.in
    // would not.
    //
    // These six link nets are the ONLY signals live during pad standby with
    // the flight computer off. Park every one of them Hi-Z whenever
    // V_MCU_SWTCH is down, or they inject through the FC's ESD diodes into a
    // dead rail.
    //
    // ESP_I2S_WS is on MTCK, the one pad of the seven with a reset pull: the
    // chip turns on a weak pull-up (45 k typical) after every reset. Left
    // alone it feeds about 60 uA into the unpowered flight computer's pad for
    // the whole pad standby, until the first power-off parks the link pins.
    // setup_oc() turns it off at boot (config::LINK_NET_ON_MTCK). The I2S
    // driver does not: it selects the pad and enables its input, and leaves
    // the pulls as it finds them.
    static constexpr int I2S_BCLK_PIN  = 40; // ESP_I2S_BCLK  (FC GPIO21; was 2)
    static constexpr int I2S_WS_PIN    = 39; // ESP_I2S_WS    (FC GPIO18; was 1)
    static constexpr int I2S_DIN_PIN   = 3;  // ESP_I2S_SD    (FC GPIO13; R34 100 k to GND)
    static constexpr int I2S_FSYNC_PIN = 34; // ESP_I2S_FSYNC (FC GPIO14; was 4)

    // --- Radio: bare E220-900MM22S on-board, direct SPI ---
    // U16, on the memory bus above with its own chip select, as on rev1.
    static constexpr bool USE_LORA_RADIO = true;
    static constexpr bool USE_UART_RADIO_MODEM = false;
    static constexpr int LORA_SPI_SCK = SPI_SCK;    // shared with the NAND
    static constexpr int LORA_SPI_MISO = SPI_MISO;  // shared with the NAND
    static constexpr int LORA_SPI_MOSI = SPI_MOSI;  // shared with the NAND
    // MOVED on V2: GPIO13 -> GPIO47 (pad 37, SPICLK_P). High-Z from reset, so
    // R141 keeps the radio deselected until the driver takes the pin.
    static constexpr int LORA_CS_PIN = 47;   // L_CS   (CONFIRMED; was 13)
    // MOVED on V2: GPIO14 -> GPIO41 (pad 47, MTDI). An input from the radio.
    static constexpr int LORA_BUSY_PIN = 41; // L_BUSY (CONFIRMED; was 14)
    static constexpr int LORA_DIO1_PIN = 17; // L_DI01 (CONFIRMED)
    static constexpr int LORA_RST_PIN = 18;  // L_RST  (CONFIRMED)
    // RF switch receive-enable. DIO2 drives transmit-enable inside the module.
    //
    // MOVED on V2: GPIO10 -> GPIO42 (pad 48, MTMS). What made GPIO10 right
    // holds here: no strapping role and no reset pull, so R142 (100 k to GND)
    // keeps the receive path off from power-on until firmware drives the pin.
    //
    // Why that matters (#1015): rev1 first had it on GPIO44, which is U0RXD
    // and carries the ROM console's weak pull-up from power-on until the pad
    // is configured, forcing RXEN high through every reset — before any code
    // runs. Reset the board mid-transmit and DIO2 is still holding TXEN high
    // for the tail of that packet, so both arms of the RF switch are asserted
    // at once on a 22 dBm module. Off is the safe default; the failure this
    // avoids is silent, because it degrades RECEIVE while transmit keeps
    // working, so telemetry looks healthy while the uplink (arming, the pyro
    // commands, mute) quietly stops answering.
    //
    // If this board is ever put into light sleep while it needs to hear
    // uplink, this pin needs gpio_hold_en — otherwise the pad goes high-Z,
    // R142 wins, and the receiver goes deaf with no error (the same shape as
    // the V9 GPIO12 lesson). GPIO42 is not an RTC pad; gpio_hold_en() still
    // works on it in light sleep, through the digital pad hold.
    //
    // NOT YET CONSUMED by this project: see the file header.
    static constexpr int LORA_RXEN_PIN = 42; // L_RXEN (CONFIRMED; was 10)

    // --- No radio daughterboard on this board ---
    static constexpr int LORA_UART_TX_PIN = -1;
    static constexpr int LORA_UART_RX_PIN = -1;
    // No LoRa_ACT net: the radio is behind U30 with everything else.
    static constexpr int LORA_ACT_PIN = -1;  // no separate gate (CONFIRMED)

    // --- No high-side switches on this board ---
    // No camera, no servo, no IMON. The gain constants keep the contract of
    // the other headers; nothing reads them with the GPIOs at -1.
    static constexpr int   CAM_IMON_GPIO      = -1;    // no camera channel
    static constexpr int   SERVO_IMON_GPIO    = -1;    // no servo channel
    static constexpr float IMON_GAIN_A_PER_A  = 0.0f;  // unused (no channel)
    static constexpr float CAM_IMON_R_OHM     = 0.0f;  // unused (no channel)
    static constexpr float SERVO_IMON_R_OHM   = 0.0f;  // unused (no channel)
};
