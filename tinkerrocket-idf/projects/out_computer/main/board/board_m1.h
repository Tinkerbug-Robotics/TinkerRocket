#pragma once

#include <stdint.h>

// rocket-computer-mini rev1 pin map for the OUT COMPUTER (ESP32-S3RH2, U15).
// Selected with TR_BOARD_M1=1:  idf.py -B build_m1 -DTR_BOARD_M1=1 build
//
// Board files: hardware/rocket-computer-mini/. Every constant below was taken
// from a kicad-cli netlist export of that schematic at the merge of the
// second-processor work, not from the design docs — walk U15's
// pad -> pinfunction -> net if you need to re-derive it.
//
// ### This board has TWO ESP32-S3s, not an S3 and a P4 ###
// rocket-computer splits the work between an ESP32-S3 out computer and an
// ESP32-P4 flight computer. The mini does the same split with an S3 on BOTH
// ends. That is why this file exists next to board_v7/v8/v9.h rather than
// being an alias: the topology is rocket-computer's, but the far side of the
// link is a different part with a different pin map, and the mini carries its
// radio and GNSS on-board instead of on daughterboards.
//
// ### The six link GPIOs are IDENTICAL to board_v9.h ###
// ESP_I2S_WS=1, ESP_I2S_BCLK=2, ESP_I2S_SD=3, ESP_I2S_FSYNC=4, ESP_SDA=5,
// ESP_SCL=6, and the FC enable on 7. That is deliberate — the hardware was drawn to match the
// reference so this project ports with a board header and nothing else. Do
// not "simplify" by aliasing board_v9.h anyway: the radio and GNSS blocks
// below are completely different, and a UART-modem backend on this board
// talks to nothing.
//
// What is NOT on this board, and why the constants are -1:
//   * no LoRa daughterboard  — the E220 is on-board, on the memory SPI bus
//   * no LoRa_ACT rail gate  — the radio sits on V_MCU_SWTCH with everything
//                              else behind U30, so PWR_PIN is its only switch
//   * no camera, no servo    — so no TPS22811 high-side switches, no IMON
//   * no separate GNSS rail  — the receiver is on V_MCU_SWTCH too
//
// KNOWN FIRMWARE GAP on this board — read before bring-up:
// LORA_RXEN_PIN is new to this project. The E220-900MM22S needs its RF switch
// driven; the base station has already been bitten by an RXEN left floating in
// RX. rocket_computer_mini/comms.cpp consumes it (lora_cfg.rxen_pin);
// out_computer/comms.cpp does NOT yet. Wire it up before trusting a link
// budget.
//
// The magnetometer used to hang off our power-monitor bus and does not any
// more — it moved to the flight computer's own MAG_SCL/MAG_SDA, where the
// driver actually lives. PWR_SDA/PWR_SCL is a pure pack-monitor bus now, the
// same shape as rocket-computer's PWR_SCL/PWR_SDA.
struct board_pins
{
    // --- Flight-computer rail switch ---
    // Net FC_EN_OC into D9's second anode (rocket-computer calls the same
    // signal P4_EN_S3, also on GPIO7). Raising this starts the flight
    // computer AND everything else behind U30: sensors, GNSS, radio, NAND.
    // The FC then holds the rail up itself through FC_EN_HOLD, so dropping
    // this pin powers the board down on the ground but NOT during a flight —
    // see flight_computer/main/board/board_m1.h for the hold policy.
    static constexpr int PWR_PIN = 7;        // FC_EN_OC (CONFIRMED)
    // No separate GNSS rail on this board — the receiver hangs off
    // V_MCU_SWTCH with the rest. -1 = call sites skip it. A wedged receiver
    // is recovered only by cycling PWR_PIN, which takes the radio, the
    // sensors and the NAND with it.
    static constexpr int GPS_PWR_PIN = -1;   // no separate rail (CONFIRMED)

    // --- Power monitoring (INA230 @ 0x40, always-on I2C bus) ---
    // Only the pack monitor is on this bus, and it, this MCU and the pull-ups
    // R67/R69 are all on the always-on +3V3 rail. Keep it that way: the
    // magnetometer used to hang here while its own supply sat behind U30, and
    // those +3V3 pull-ups drove its pads against a rail the switch was
    // actively discharging — which clamped the bus below VIL and made the
    // pack monitor unreadable during pad standby.
    static constexpr int PWR_SDA = 21;       // SEN_SDA (CONFIRMED)
    static constexpr int PWR_SCL = 33;       // SEN_SCL (CONFIRMED)

    // --- Memory + radio shared SPI bus ---
    // Unlike rocket-computer, the telemetry radio lives on THIS bus (see the
    // radio block below). The NAND's page-sized bursts and the radio's twice
    // a second packet share a host; the sensors deliberately do not, because
    // head-of-line blocking behind a NAND page lands as jitter on the sample
    // timebase. Same GPIOs as V7/V8/V9.
    static constexpr int SPI_SCK = 37;       // M_SCK      (CONFIRMED)
    static constexpr int SPI_MISO = 35;      // M_MISO     (CONFIRMED)
    static constexpr int SPI_MOSI = 38;      // M_MOSI     (CONFIRMED)
    static constexpr int NAND_CS = 36;       // M_FLASH_CS (CONFIRMED)
    // No MRAM on this board and never was — U12 does not exist in the mini's
    // schematic. As on V9/V10 this costs the FC's in-flight reboot recovery
    // and the dirty-ring replay their storage; the NAND-log snapshot +
    // boot tail-scan is the replacement.
    static constexpr int MRAM_CS = -1;       // not fitted (CONFIRMED)
    // U15 is an ESP32-S3RH2: 2 MB in-package quad PSRAM, no in-package flash,
    // VDD_SPI 3.3 V (datasheet Table 1-1). Quad PSRAM sits on SPICS1 (pad 28,
    // externally NC by design), which is why GPIO26 is unusable on this part
    // and why GPIO35-38 stay free for the NAND — the GPIO33-37 restriction
    // covers OCTAL parts only. Still volatile: this buys ring SIZE, never
    // reboot recovery.
    static constexpr bool RING_IN_PSRAM = true;   // 2 MB in-package (CONFIRMED)

    // --- I2C slave (commands from the FlightComputer) ---
    static constexpr int I2C_SDA_PIN = 5;    // ESP_SDA (CONFIRMED)
    static constexpr int I2C_SCL_PIN = 6;    // ESP_SCL (CONFIRMED)

    // --- I2S slave RX (high-frequency telemetry from the FlightComputer) ---
    // Both ends must agree PER NET, not per role name. FC side on this board:
    // ESP_I2S_BCLK=21, ESP_I2S_WS=18, ESP_I2S_SD=13, ESP_I2S_FSYNC=14 — note
    // the data pair is NOT the P4's 19/20, because on an S3 those pads are USB
    // D-/D+ and this board spends them on the USB mux. Our four are unchanged
    // from V9.
    //   ESP_I2S_BCLK = bit clock,   ESP_I2S_SD    = FC data out (our DIN),
    //   ESP_I2S_WS   = word select, ESP_I2S_FSYNC = frame sync.
    // ESP_I2S_FSYNC is a plain GPIO, not a peripheral signal — the FC pulses
    // it around each frame and we read it to resync.
    //
    // These six link nets are the ONLY signals live during pad standby with
    // the flight computer off. Park every one of them Hi-Z whenever
    // V_MCU_SWTCH is down or they inject through the FC's ESD diodes into a
    // dead rail — this is the failure that i2s_del_channel() caused on
    // rocket-computer by leaving BCLK/WS/DOUT driven.
    static constexpr int I2S_BCLK_PIN  = 2;  // ESP_I2S_BCLK  (FC GPIO21)
    static constexpr int I2S_WS_PIN    = 1;  // ESP_I2S_WS    (FC GPIO18)
    static constexpr int I2S_DIN_PIN   = 3;  // ESP_I2S_SD    (FC GPIO13)
    static constexpr int I2S_FSYNC_PIN = 4;  // ESP_I2S_FSYNC (FC GPIO14)

    // --- Radio: bare E220-900MM22S on-board, direct SPI (V7 topology) ---
    // NOT the UART daughterboard. This is the V7 arrangement returning: the
    // direct-SPI driver was never deleted, it was set to -1 across V8/V9 when
    // the radio moved off-board. The module shares the memory bus above and
    // keeps its own chip select.
    static constexpr bool USE_LORA_RADIO = true;
    static constexpr bool USE_UART_RADIO_MODEM = false;
    static constexpr int LORA_SPI_SCK = SPI_SCK;    // shared with the NAND
    static constexpr int LORA_SPI_MISO = SPI_MISO;  // shared with the NAND
    static constexpr int LORA_SPI_MOSI = SPI_MOSI;  // shared with the NAND
    static constexpr int LORA_CS_PIN = 13;   // L_CS   (CONFIRMED)
    static constexpr int LORA_BUSY_PIN = 14; // L_BUSY (CONFIRMED)
    static constexpr int LORA_DIO1_PIN = 17; // L_DI01 (CONFIRMED)
    static constexpr int LORA_RST_PIN = 18;  // L_RST  (CONFIRMED)
    // RF switch receive-enable. DIO2 drives transmit-enable inside the module
    // once configured, so only RXEN reaches a GPIO — and it MUST be driven.
    // GPIO44 is U0RXD, so this board has console TX but no console RX; that
    // was the deliberate trade for the pin. NOT YET CONSUMED by this
    // project's comms.cpp — see the gap note in the file header.
    static constexpr int LORA_RXEN_PIN = 44; // L_RXEN (CONFIRMED; U0RXD spent)

    // --- No radio daughterboard on this board ---
    static constexpr int LORA_UART_TX_PIN = -1;
    static constexpr int LORA_UART_RX_PIN = -1;
    // No LoRa_ACT net: the radio is behind U30 with everything else, so the
    // only way to power-cycle it is PWR_PIN, which drops the flight computer
    // too. That is the accepted cost of this board's single-switch rail.
    static constexpr int LORA_ACT_PIN = -1;  // no separate gate (CONFIRMED)

    // --- No high-side switches on this board ---
    // rocket-computer's TPS22811 camera and servo channels (U26/U28) do not
    // exist here — no camera, no servo, no IMON. The gain constants are kept
    // so the contract matches board_v9.h, but nothing can read them with the
    // GPIOs at -1.
    static constexpr int   CAM_IMON_GPIO      = -1;    // no camera channel
    static constexpr int   SERVO_IMON_GPIO    = -1;    // no servo channel
    static constexpr float IMON_GAIN_A_PER_A  = 0.0f;  // unused (no channel)
    static constexpr float CAM_IMON_R_OHM     = 0.0f;  // unused (no channel)
    static constexpr float SERVO_IMON_R_OHM   = 0.0f;  // unused (no channel)
};
