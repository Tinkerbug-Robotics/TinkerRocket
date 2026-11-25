#ifndef OUT_COMPUTER_CONFIG_H
#define OUT_COMPUTER_CONFIG_H

#include <stdint.h>
#include <RocketComputerTypes.h>  // LORA_FACTORY_RENDEZVOUS_* (#105)

// --- Board revision (#411, tracking #408) ---
// Pins + peripheral-presence/topology flags live in the per-board headers;
// everything else in this file is board-independent policy and must not
// fork per revision. Select the V8 map with: idf.py -DTR_BOARD_V8=1 build
// (default stays V7 until V8 bring-up completes).
//
// TR_BOARD_V9=1 selects board_v9.h (V9). The S3's *pin* map is unchanged
// from V8 through V9 — unlike the FC, where V9 moved three pyro pins — so
// this used to be an alias to board_v8.h. #822: it isn't a pin change that
// forced the split, it's a deleted part. V9 removed the MRAM (U12), and an
// alias is presence-blind: it can say "same pins" but not "one fewer device",
// so V9 firmware kept driving an MRAM_CS that goes nowhere and backed the
// log ring + reboot snapshot with an unfitted chip. The two headers differ in
// exactly one value, MRAM_CS (34 vs -1); keep every other pin in step by hand.
//
// TR_BOARD_V10=1 selects board_v10.h (#1593): every V9 pin plus four new ones,
// the first of which is the supervised arm's consent output. A V9 image on a
// V10 never raises it, so that board cannot fire a channel.
#ifndef TR_BOARD_V5
#define TR_BOARD_V5 0
#endif
#ifndef TR_BOARD_V7
#define TR_BOARD_V7 0
#endif
#ifndef TR_BOARD_V8
#define TR_BOARD_V8 0
#endif
#ifndef TR_BOARD_V9
#define TR_BOARD_V9 0
#endif
#ifndef TR_BOARD_V10
#define TR_BOARD_V10 0
#endif
// TR_BOARD_M2=1 selects board_m2.h: the Tinker-Beetle V2, in work and not
// ordered. The far side of the link is a second ESP32-S3 rather than a P4,
// the radio is an on-board E220 on the memory SPI bus rather than a UART
// daughterboard, and there is no camera, servo or separate GNSS rail. The
// V1's two-S3 board with seven of this processor's signals on other pads (the
// 2026-11-21 layout move) and an ESP32-S3FH4R2 whose boot flash is the 4 MB
// in its package; an M2 build declares 4 MB with a partition table of its own
// (sdkconfig.defaults.flash4mb). The V1 (rev1) is retired and has no build
// here.
#ifndef TR_BOARD_M2
#define TR_BOARD_M2 0
#endif
// No default (#834 sweep). main/CMakeLists.txt raises the same error with a
// friendlier message; this is the backstop for anything that includes config.h
// without going through it.
#if (TR_BOARD_V5 + TR_BOARD_V7 + TR_BOARD_V8 + TR_BOARD_V9 + TR_BOARD_V10 + TR_BOARD_M2) != 1
#error "Set exactly one board revision: -DTR_BOARD_V5=1, -DTR_BOARD_V7=1, -DTR_BOARD_V8=1, -DTR_BOARD_V9=1, -DTR_BOARD_V10=1 or -DTR_BOARD_M2=1. There is no default — V7's PWR_PIN (GPIO6) is ESP_SCL on every later board, V9 and V10 need the sdkconfig overlay that enables the PSRAM log ring, V10's arm needs this processor's consent (GPIO14), and M2 (Tinker-Beetle V2) drives its radio over SPI where V8/V9/V10 drive a UART daughterboard."
#endif
// TR_BOARD_REV_STR names the board in log lines. It sits next to the header it
// describes, as in the flight computer's config.h, so the two cannot disagree.
// V7 is a case of its own: a board flag with no branch here stops the build
// instead of compiling V7's pins under V7's name (#1316).
#if TR_BOARD_V8
#include "board/legacy/board_v8.h"
#define TR_BOARD_REV_STR "V8"
#elif TR_BOARD_V9
#include "board/board_v9.h"
#define TR_BOARD_REV_STR "V9"
#elif TR_BOARD_V10
#include "board/board_v10.h"
#define TR_BOARD_REV_STR "V10"
#elif TR_BOARD_M2
#include "board/board_m2.h"
#define TR_BOARD_REV_STR "M2 (Tinker-Beetle V2)"
#elif TR_BOARD_V7
#include "board/legacy/board_v7.h"
#define TR_BOARD_REV_STR "V7"
#elif TR_BOARD_V5
#include "board/legacy/board_v5.h"
#define TR_BOARD_REV_STR "V5"
#else
#error "This board flag selects no board header: add it to the chain in config.h."
#endif

// The one difference between the V8 and V9 maps, asserted rather than trusted.
// Nothing at runtime notices a wrong MRAM_CS: with 34 on a V9 board the ring
// and the snapshot slot write to a floating pad and report success, and with
// -1 on a V8 board the fitted MRAM simply goes unused. Both are silent. So pin
// it here — re-aliasing the V9 branch back to board_v8.h, or editing MRAM_CS
// in the wrong header, then fails the build instead of the flight.
#if TR_BOARD_M2
// The Beetle never had an MRAM to delete — U12 does not exist in its schematic.
static_assert(board_pins::MRAM_CS < 0,
              "the Tinker-Beetle has no MRAM (no U12) — its board header must keep MRAM_CS = -1");
// The radio is a bare E220 on the memory SPI bus, not a UART daughterboard.
// Getting this backwards points the modem backend at pins that are -1 and the
// link simply never comes up, with no obvious symptom at the pin level.
static_assert(!board_pins::USE_UART_RADIO_MODEM,
              "the Tinker-Beetle drives its radio over SPI — its board header must keep USE_UART_RADIO_MODEM = false");
static_assert(board_pins::LORA_CS_PIN >= 0 && board_pins::LORA_SPI_SCK >= 0,
              "Tinker-Beetle board header: the on-board E220 needs a real chip select and clock");
// The E220's RF switch receive-enable MUST be driven; the base station has
// already been bitten by an RXEN left floating in RX.
static_assert(board_pins::LORA_RXEN_PIN >= 0,
              "Tinker-Beetle board header: L_RXEN reaches a GPIO on this board and must be driven");
#endif

#if TR_BOARD_M2
// The seven signals the V2 layout moved (2026-11-21), pinned. Each of these is
// silent at runtime when wrong: rev1's number drives a bare pad, so the rail
// never comes up (PWR_PIN), the radio never answers (LORA_CS_PIN, LORA_BUSY_PIN)
// or the link delivers nothing (the three I2S pins). Re-derive them from a
// netlist export of the V2 schematic before changing one — V2 is not ordered,
// and its map can still move.
static_assert(board_pins::PWR_PIN == 48 && board_pins::LORA_CS_PIN == 47,
              "board_m2.h: FC_EN_OC is S3 GPIO48 (pad 36) and L_CS is GPIO47 (pad 37) on the Tinker-Beetle V2");
static_assert(board_pins::I2S_BCLK_PIN == 40 && board_pins::I2S_WS_PIN == 39 &&
              board_pins::I2S_FSYNC_PIN == 34 && board_pins::I2S_DIN_PIN == 3,
              "board_m2.h: ESP_I2S_BCLK/WS/FSYNC are S3 GPIO40/39/34 on the Tinker-Beetle V2, and ESP_I2S_SD stays on GPIO3");
static_assert(board_pins::LORA_BUSY_PIN == 41 && board_pins::LORA_RXEN_PIN == 42,
              "board_m2.h: L_BUSY is S3 GPIO41 (pad 47) and L_RXEN is GPIO42 (pad 48) on the Tinker-Beetle V2");
// The two outputs that must stay low through a reset sit behind 100 k
// pull-downs (R84 on POWER_SWITCH, R142 on L_RXEN). A pad the chip pulls up at
// reset beats those; of the pads a signal could move to, that is GPIO39 (MTCK)
// and GPIO43/44 (the ROM console). ESP32-S3 datasheet v2.2, Table 2-1.
static_assert(board_pins::PWR_PIN != 39 && board_pins::PWR_PIN != 43 && board_pins::PWR_PIN != 44 &&
              board_pins::LORA_RXEN_PIN != 39 && board_pins::LORA_RXEN_PIN != 43 &&
              board_pins::LORA_RXEN_PIN != 44,
              "board_m2.h: FC_EN_OC and L_RXEN must not sit on a pad the chip pulls up at reset");
#endif

// The GNSS antenna offset (cmd 81) is the flight computer's to keep, and the
// Beetle's keeps none (RocketComputerTypes.h, F_ANTENNA_FROM_NVS), so on that
// board this processor neither stages one nor reads one back, and the apps
// lock the setting. M2 follows the V1, as the V2's flight computer does
// (flight_computer config.h, TR_FC_ANTENNA_OFFSET): the two must agree, or one
// processor stages a setting the other never reads back.
#define TR_OC_ANTENNA_OFFSET (!TR_BOARD_M2)

#if TR_BOARD_V9
static_assert(board_pins::MRAM_CS < 0,
              "V9 deleted the MRAM (U12) — board_v9.h must keep MRAM_CS = -1 (#822)");
#elif TR_BOARD_V10
static_assert(board_pins::MRAM_CS < 0,
              "V10 has no MRAM (U12) — board_v10.h must keep MRAM_CS = -1 (#822)");
// #1593: the V10's arm is a two-processor vote, and OC_ARM_EN is ours. -1 here
// compiles consent out, and the board then cannot fire a channel at all (the
// Beetle's #1168 bug, rebuilt). The INA230 alert edge is the fault trip's
// interrupt path (#1553).
static_assert(board_pins::ARM_CONSENT_PIN == 14,
              "board_v10.h: OC_ARM_EN is S3 GPIO14 — re-derive it from the netlist before changing it (#1593)");
static_assert(board_pins::INA_ALERT_PIN == 18,
              "board_v10.h: INA_ALERT is S3 GPIO18 (#1553, #1593)");
static_assert(board_pins::PG_RAIL_PIN == 17 && board_pins::VBUCK_OK_PIN == 34,
              "board_v10.h: PG_RAIL is S3 GPIO17 and VBUCK_OK is GPIO34 (#1593)");
#elif TR_BOARD_V8
static_assert(board_pins::MRAM_CS == 34,
              "V8 bench boards have the MRAM fitted on GPIO34 — do not change this (#822)");
#endif

// #1166: the hold-up cap sense (V_SCAP_ADC, R125/R126) reaches this processor
// on the Beetle, and its verdict is the only thing on the board that reports a
// hold-up that silently is not there.  Setting the pin to -1 there would
// compile the monitor out and bring that silence back.
#if TR_BOARD_M2
static_assert(board_pins::SCAP_ADC_PIN >= 0,
              "Tinker-Beetle board header: V_SCAP_ADC reaches the out computer on the Beetle and must be read (#1166)");
#endif
// Wherever the sense exists it must sit on ADC1 (GPIO1-10 on the S3): ADC2 is
// unusable with the radio up, and a wrong pad reads a floating input as a
// verdict.
static_assert(board_pins::SCAP_ADC_PIN < 0 ||
              (board_pins::SCAP_ADC_PIN >= 1 && board_pins::SCAP_ADC_PIN <= 10),
              "SCAP_ADC_PIN must be an ADC1 pad (GPIO1-10) or -1 (#1166)");
// #1409: one source per board. A sense line here AND the flight computer's
// relayed reading would be two verdicts fighting over one telemetry key.
static_assert(!(board_pins::SCAP_FROM_FC && board_pins::SCAP_ADC_PIN >= 0),
              "SCAP_FROM_FC and SCAP_ADC_PIN are exclusive: the cap is read on one processor (#1409)");
#if TR_BOARD_V10
// The V10's (and V11's) V_SCAP_ADC is P4 GPIO17: the flight computer reads it
// and this processor judges it. false here would put the V10 back to "scap"
// absent and the hold-up unwatched.
static_assert(board_pins::SCAP_FROM_FC,
              "board_v10.h: V_SCAP_ADC reaches the flight computer, which relays it — SCAP_FROM_FC (#1409)");
#endif
// #1553: the INA230 alert edge drops OC_ARM_EN, so it means nothing on a board
// with no consent pin to drop.
static_assert(board_pins::INA_ALERT_PIN < 0 || board_pins::ARM_CONSENT_PIN >= 0,
              "INA_ALERT_PIN is the fault-current trip's input: it needs ARM_CONSENT_PIN (#1553)");
// The consent output must not be an ESP32-S3 strapping pad (GPIO0, 3, 45, 46):
// setup_oc drives it low, and a strap held at a level by the arm chain could
// change how the board boots.
static_assert(board_pins::ARM_CONSENT_PIN != 0 && board_pins::ARM_CONSENT_PIN != 3 &&
              board_pins::ARM_CONSENT_PIN != 45 && board_pins::ARM_CONSENT_PIN != 46,
              "ARM_CONSENT_PIN landed on an ESP32-S3 strapping pad");

struct config : board_pins
{
    // --- Debug ---
    static constexpr bool DEBUG = true;  // Re-enabled (needed for telemetry updates)
    static constexpr bool VERBOSE_DEBUG = false;  // Temporarily disabled to see BLE output
    static constexpr uint32_t STATS_PERIOD_MS = 1000;

    // #831: how long the OC keeps calling its FC-sourced telemetry LIVE after
    // the last NonSensorData frame.  The FC sends those at
    // NON_SENSOR_UPDATE_RATE = 500 Hz, so 3 s is ~1500 missed frames — far
    // beyond any I2S hiccup, and matched to the base station's own
    // BLE_TELEMETRY_STALE_MS so both transports age their data alike.
    static constexpr uint32_t FC_FRAME_STALE_MS = 3000;
    // #398: dump per-task CPU utilization once per stats interval (the "TASKCPU"
    // line) to identify the core-1 hog during the launch-activation window.
    // Requires the run-time-stats sdkconfig flags; set false to silence.
    static constexpr bool PROFILE_TASK_CPU = true;

    // --- MRAM (MR25H10 on shared SPI bus; CS pin in board header) ---
    // V7/V8 only.  Enabled: 128 KB non-volatile ring buffer survives hard
    // resets.  SPI bus mutex prevents contention between Core 1 (ring push)
    // and Core 0 (NAND flush).  On dirty startup, MRAM is drained to a
    // recovery file before clearing.  MRAM_CS = -1 falls back to RAM ring.
    //
    // V9/V10 have no MRAM (board_v9.h, MRAM_CS = -1), so everything below
    // that is measured from MRAM_SIZE — the snapshot region and the #274
    // dirty marker — addresses a device that isn't there.  The constants stay
    // (they are the V7/V8 geometry, and TR_LogToFlash ignores them outright
    // when use_mram_ is false), but do NOT read them as "the OC always has a
    // non-volatile snapshot store".  It doesn't; see initPeripherals().
    static constexpr uint32_t MRAM_SIZE = 131072;       // 128 KB
    static constexpr uint32_t SPI_HZ_MRAM = 40'000'000;
    static constexpr uint8_t SPI_MODE_MRAM = SPI_MODE0;

    // FlightSnapshot region (#104 follow-up): top 1 KB of MRAM is reserved
    // for the latest FC FlightSnapshotData (~224 B wire frame).  The log
    // ring uses the remaining MRAM_SIZE - SNAPSHOT_REGION_SIZE bytes.
    // Single-slot writes serialized on the SPI bus mutex (write ~700 us;
    // reads on the same mutex never observe a partial write).
    static constexpr uint32_t SNAPSHOT_REGION_SIZE = 1024;
    static constexpr uint32_t SNAPSHOT_REGION_BASE = MRAM_SIZE - SNAPSHOT_REGION_SIZE;
    // #274: 4-byte sink-mode "dirty" marker, parked at the free top of the
    // snapshot region (FlightSnapshot uses ~232 B up from SNAPSHOT_REGION_BASE).
    // Set while a log session is open; a surviving magic on the next boot means
    // an unclean shutdown left unflushed frames in the non-volatile MRAM ring.
    static constexpr uint32_t MRAM_DIRTY_MARKER_ADDR = MRAM_SIZE - 16;

    // --- SPI speeds/modes ---
    static constexpr uint32_t SPI_HZ_NAND = 40'000'000;
    static constexpr uint8_t SPI_MODE_NAND = SPI_MODE0;

    // --- RAM ring buffer (used when MRAM_CS = -1) ---
    // Last-resort fallback: internal SRAM, used only when neither MRAM nor
    // PSRAM is available. Kept small because the S3 has 512 KB of SRAM total
    // and everything else has to fit alongside it.
    static constexpr uint32_t RAM_RING_SIZE = 65536;  // 64 KB fallback

    // --- PSRAM ring buffer (#822; used when RING_IN_PSRAM && MRAM_CS = -1) ---
    // The V9/V10 story: the MRAM was deleted in favour of the S3RH2's 2 MB of
    // in-package PSRAM, which is there to buffer NAND writes. Board presence is
    // the header's call (RING_IN_PSRAM); the size is policy and lives here.
    //
    // 512 KB is a deliberate first step, not a maximum. Arithmetic at the
    // shipped default IMU rate (~156 KB/s from the pad):
    //   MRAM  128 KB ring -> 97536 B prelaunch cap -> ~0.61 s of history
    //   here  512 KB ring -> 384 KB prelaunch cap  -> ~2.5 s
    // and ~3.3 s of total NAND-stall headroom against the ~0.8 s the MRAM ever
    // gave. That is a 4x improvement while leaving 1.5 MB of the part unspoken
    // for. Raising it toward 1-2 MB is reasonable once there is bench data on
    // a real V9 — watch the #398 gauges (parser_max, spi_wait, ring_overruns)
    // and the activation drain, since a larger prelaunch ring means a longer
    // backlog for the flush task to work off after launch detect.
    static constexpr uint32_t PSRAM_RING_SIZE = 524288;  // 512 KB

    // --- I2C from FlightComputer -> OutComputer (pins in board header) ---
    static constexpr uint8_t I2C_ADDRESS = 0x42;
    static constexpr uint32_t I2C_CLOCK_HZ = 1'200'000;
    static constexpr size_t I2C_SLAVE_RX_BUF = 8192;
    static constexpr size_t I2C_SLAVE_TX_BUF = 256;
    // Time-slice ingress to prevent loop starvation under sustained traffic.
    static constexpr uint32_t I2C_INGRESS_BUDGET_US = 1500;
    static constexpr size_t I2C_INGRESS_BUDGET_BYTES = 8192;

    // --- I2S (high-frequency telemetry RX from FlightComputer; pins in
    //     board header) ---
    // I2S bandwidth = sample_rate * 4 bytes (16-bit stereo).  The rate is the
    // shared I2S_LINK_SAMPLE_RATE_HZ (RocketComputerTypes.h, 88200 = 352.8
    // KB/s), which the FC's master clock reads too.  RX DMA descriptors are
    // sized from it in setup (kI2sRxDmaFrameNum) to keep the callback cadence
    // ~2.9 ms.  Update the OC before the FC: set for this rate, the slave
    // still reads an older FC clocking at 44100; the reverse does not hold.
    static constexpr uint32_t I2S_SAMPLE_RATE = I2S_LINK_SAMPLE_RATE_HZ;

    // The one usable ESP32-S3 pad that comes out of reset pulled UP without
    // being a console or strapping pad: MTCK, GPIO39. The chip turns on its
    // weak pull-up (45 k typical) after every reset while EFUSE_DIS_PAD_JTAG is
    // unburnt (datasheet v2.2, Table 2-1 note 7), and no driver turns it off.
    // A flight-computer link net on that pad feeds about 60 uA into the
    // unpowered flight computer for the whole pad standby, so setup_oc() turns
    // the pull off at boot on a board that has one there. That is the
    // Tinker-Beetle V2 (ESP_I2S_WS, board_m2.h) and no other board.
    static constexpr int S3_MTCK_GPIO = 39;
    static constexpr bool LINK_NET_ON_MTCK =
        I2S_BCLK_PIN == S3_MTCK_GPIO || I2S_WS_PIN == S3_MTCK_GPIO ||
        I2S_DIN_PIN == S3_MTCK_GPIO || I2S_FSYNC_PIN == S3_MTCK_GPIO ||
        I2C_SDA_PIN == S3_MTCK_GPIO || I2C_SCL_PIN == S3_MTCK_GPIO;

    // #1484: the Beetle's E220 rides the memory bus (board_m2.h wires LORA_SPI_*
    // to SPI_*). A radio there must join the NAND's host (SPI2) as a second
    // device. Starting SPI3 on the same pins re-routes the pads and leaves the
    // NAND's host with no clock, which silently drops every write after the
    // radio starts. V7's SPI radio has pins of its own and keeps SPI3.
    static constexpr bool LORA_ON_MEMORY_BUS =
        !USE_UART_RADIO_MODEM && LORA_SPI_SCK == SPI_SCK;
    static_assert((LORA_SPI_SCK == SPI_SCK) == (LORA_SPI_MISO == SPI_MISO) &&
                      (LORA_SPI_SCK == SPI_SCK) == (LORA_SPI_MOSI == SPI_MOSI),
                  "the radio shares all three memory-bus pins or none of them (#1484)");

    // --- LoRa RF parameters (radio presence + pins in board header) ---
    static constexpr float LORA_FREQ_MHZ = 915.0f;
    static constexpr uint8_t LORA_SF = 8;
    static constexpr float LORA_BW_KHZ = 250.0f;
    static constexpr uint8_t LORA_CR = 5;
    static constexpr uint16_t LORA_PREAMBLE_LEN = 12;
    static constexpr int8_t LORA_TX_POWER_DBM = 12;
    static constexpr bool LORA_CRC_ON = true;
    static constexpr bool LORA_RX_BOOSTED_GAIN = true;
    static constexpr bool LORA_SYNCWORD_PRIVATE = true;
    static constexpr uint16_t LORA_TX_RATE_HZ = 2;

    // Known-good rendezvous mode (issue #71).  Sourced from the shared
    // RocketComputerTypes.h header (#105) so both BS and OC firmware are
    // guaranteed to agree at compile time — see LORA_FACTORY_RENDEZVOUS_*
    // there for the full justification.
    static constexpr float   LORA_RENDEZVOUS_MHZ          = LORA_FACTORY_RENDEZVOUS_MHZ;
    static constexpr uint8_t LORA_RENDEZVOUS_SF           = LORA_FACTORY_RENDEZVOUS_SF;
    static constexpr float   LORA_RENDEZVOUS_BW_KHZ       = LORA_FACTORY_RENDEZVOUS_BW_KHZ;
    static constexpr uint8_t LORA_RENDEZVOUS_CR           = LORA_FACTORY_RENDEZVOUS_CR;
    static constexpr int8_t  LORA_RENDEZVOUS_TX_POWER_DBM = LORA_FACTORY_RENDEZVOUS_TX_DBM;

    // --- LoRa Uplink RX (commands from BaseStation) ---
    static constexpr uint8_t UPLINK_SYNC_BYTE = 0xCA;

    // --- Device Identity (factory defaults, overridden by NVS on boot) ---
    static constexpr uint8_t DEFAULT_NETWORK_ID = 0;
    static constexpr uint8_t DEFAULT_ROCKET_ID  = 1;
    static constexpr const char* DEVICE_TYPE     = "R";  // "R" = rocket

    // --- #1166: hold-up supercap health (the Tinker-Beetle, V10/V11) ---
    // SCAP_ADC_PIN / SCAP_DIVIDER_RATIO come from the board header (-1 on
    // boards without the sense line).  The TPS61094 charges C130 (5 F) at
    // 100 mA to a 2.5 V termination — about two minutes from empty — and
    // nothing else on the board reports whether it did (holdup_policy.h).
    // #1409: on the Tinker-Mantis V10/V11 (SCAP_FROM_FC) the sense line is the
    // flight computer's; it sends its reading in every FC_STATUS_MSG, and the
    // same rules below run on that.  C130 is 2 F there: ~50 s from empty.
    // With no FC_STATUS_MSG for FC_FRAME_STALE_MS (the FC rail off, a reboot)
    // the verdict is withheld, not judged on a stale number.
    //   CHARGED_V: at or above this the backup is there (2.2 V of the 2.5 V
    //     termination is 77 % of the energy).
    //   LOW_ADVISORY_MS: after this long continuously under CHARGED_V the OC
    //     reports NOT_CHARGING — one advisory line in the app, never an arm
    //     block.  Measured from boot on a cold start and from the last charged
    //     reading otherwise, so a cap a hold-up event just drained gets its
    //     recharge time before anyone is told.
    //   TRACE_DELTA_V / TRACE_PERIOD_MS: the [HOLDUP] console line — one
    //     whenever V_SCAP has moved TRACE_DELTA_V since the last line, at
    //     least every TRACE_PERIOD_MS when steady, and on every change of
    //     verdict.  Through the 100 mA ramp that is a line every ~2.5 s (the
    //     first-article cold-start trace); terminated, one a minute; a
    //     discharge riding the cap logs every second.
    //   PIN_CEILING_MV: the 6 dB range is good to ~1.75 V at the pad; a
    //     2.5 V termination is 1.25 V and the VIN − 800 mV bound ~1.33 V.
    //     Above this, once per boot, say the divider or VCHG is not as built
    //     rather than report a confidently clipped number.
    static constexpr float    HOLDUP_CHARGED_V        = 2.2f;
    static constexpr uint32_t HOLDUP_LOW_ADVISORY_MS  = 180000u;
    static constexpr float    HOLDUP_TRACE_DELTA_V    = 0.05f;
    static constexpr uint32_t HOLDUP_TRACE_PERIOD_MS  = 60000u;
    static constexpr int      HOLDUP_PIN_CEILING_MV   = 1600;
    //   JUMP_V / JUMPS_NOT_FITTED / CALM_TO_CLEAR (#1485): is a capacitor on
    //     the sense node at all? A fitted 5 F cap moves at most ~0.15 V/s
    //     (the end of a full-load hold-up) and ~0.02 V/s charging; an empty
    //     footprint's node jumped 0.4-0.8 V between one-second readings and
    //     read CHARGED on voltage alone. JUMPS_NOT_FITTED of the last five
    //     steps over JUMP_V report NOT_FITTED — one advisory line, like
    //     NOT_CHARGING — and CALM_TO_CLEAR calm steps in a row withdraw it.
    static constexpr float    HOLDUP_JUMP_V           = 0.3f;
    static constexpr uint8_t  HOLDUP_JUMPS_NOT_FITTED = 3;
    static constexpr uint8_t  HOLDUP_CALM_TO_CLEAR    = 10;
};

#endif
