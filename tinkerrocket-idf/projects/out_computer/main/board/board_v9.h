#pragma once

#include <stdint.h>

// V9/V10 PCB pin map + board topology for the out computer (ESP32-S3).
// Selected with TR_BOARD_V9=1:  idf.py -B build_v9 -DTR_BOARD_V9=1 build
//
// This is the map of the board files in hardware/rocket-computer/ — the
// .kicad_pcb title block reads V9 up to 25fb08d and V10 at HEAD.
//
// ### Every S3 GPIO here is identical to board_v8.h ###
// Verified 2026-08-17 by walking U15's pad -> pinfunction -> net in
// hardware/rocket-computer.kicad_pcb at every commit that ever touched it,
// and against a kicad-cli netlist export of the V10 schematic. That is the
// opposite of the FC, whose V9 map moved PYRO_ARM and swapped two FIRE pins.
//
// So why a separate header? Because a board revision is not only its pin map.
// A part was DELETED:
//
//   part            board_v8.h    here (V9/V10)
//   MRAM (U12)      MRAM_CS = 34  MRAM_CS = -1  (not fitted)
//
// An alias to board_v8.h is pin-correct and presence-blind; it cannot express
// "same pins, one fewer device", and TR_LogToFlash enables the MRAM ring on
// the CS pin alone with no device probe. Until #822 this file did not exist
// and TR_BOARD_V9=1 selected board_v8.h, so V9/V10 firmware addressed an MRAM
// that is not on the board. See the MRAM_CS entry below for what that costs.
//
// Two things changed around this map without changing it:
//   * PWR_PIN's net was renamed Power_Switch -> P4_EN_S3 at the V9 pre-fab
//     close-out. Same GPIO7, same job: it is the enable that powers the P4's
//     rail (U30, a TPS22810). On V9+ the P4 also has its own hold line into
//     the same enable (its GPIO5, net P4_EN_HOLD) which its firmware does not
//     currently drive — so dropping PWR_PIN still powers the FC down, but
//     check flight_computer/main/board/board_v9.h before assuming that.
//   * The GNSS rail gained a switch, but not one of ours — see GPS_PWR_PIN.
struct board_pins
{
    // --- Power rail switch ---
    // V9/V10 net P4_EN_S3 (V8 called the same net Power_Switch). NOTE: V7's
    // PWR_PIN (6) is this board's ESP_SCL net — running the V7 map here
    // toggles the FC I2C clock on every power on/off.
    static constexpr int PWR_PIN = 7;        // P4_EN_S3 (CONFIRMED)
    // The S3 has no GNSS rail switch, and on this revision it does not need
    // one: the GNSS rail is gated by the P4 (net GPS_ACT, FC GPIO15 -> U27),
    // not by us. -1 = call sites skip it. (board_v8.h carried a "TODO:
    // confirm whether V8 has one" here; confirmed and closed with #822.)
    static constexpr int GPS_PWR_PIN = -1;   // switched by the FC, not the S3

    // --- Power monitoring (INA230 @ 0x40, always-on I2C bus) ---
    static constexpr int PWR_SDA = 21;       // CONFIRMED (bench-validated)
    static constexpr int PWR_SCL = 33;       // CONFIRMED (bench-validated)

    // --- Memory shared SPI bus (same pins as V7/V8) ---
    static constexpr int SPI_SCK = 37;       // M_SCK      (CONFIRMED)
    static constexpr int SPI_MISO = 35;      // M_MISO     (CONFIRMED)
    static constexpr int SPI_MOSI = 38;      // M_MOSI     (CONFIRMED)
    static constexpr int NAND_CS = 36;       // M_FLASH_CS (CONFIRMED)
    // NO MRAM ON THIS BOARD. U12 (MR25H10) was deliberately deleted during the
    // V9 design, replaced by the S3RH2's in-package PSRAM — recorded in
    // hardware/rocket-computer/prefab-review-2026-07-30.md ("I18. MRAM
    // deletion fully implemented: no U12/MRAM in schematic, BOM, or PCB") and
    // confirmed from the netlist, where GPIO34 is
    // `unconnected-(U15-GPIO34-Pad39)` and `net 'MRAM'` returns nothing.
    //
    // -1 routes TR_LogToFlash to its heap RAM ring (config::RAM_RING_SIZE,
    // 64 KB) instead of a 128 KB MRAM ring. Consequences, all of them real and
    // none of them a fault to be fixed by changing this value back:
    //   * the log ring halves (130048 B -> 65536 B), so the pre-launch history
    //     the flight file opens with halves too: prelaunchCap() is 3/4 of the
    //     ring, and at the shipped default IMU rate the FC streams ~156 KB/s
    //     from the pad (IMU_RATE_DYNAMIC boosts to 3840 Hz at boot, not at
    //     launch), so ~0.61 s of pre-launch history becomes ~0.31 s.
    //     config::RAM_RING_SIZE is deliberately NOT raised to compensate: it
    //     is board-independent policy, and the internal-RAM headroom at
    //     initPeripherals() has never been measured on a V9 board. A ring that
    //     fails to allocate is a far worse outcome than a short one — begin()
    //     returns false and flight logging is dead for the boot. Revisit with
    //     hardware and a free-heap number, not before.
    //
    //     Read that as "64 KB is what we can safely use today", NOT as "64 KB
    //     is all this board has". The MRAM's designated replacement is on the
    //     die: the RH2 carries 2 MB of in-package quad PSRAM, and the whole
    //     point of the swap was for the ring to live there. It is unused —
    //     CONFIG_SPIRAM is unset — and turning it on is a hardware question,
    //     not a flag:
    //       * prefab-review-2026-07-30.md H7: VDD_SPI feeds the in-package
    //         PSRAM *and* the external boot flash through the S3's internal
    //         ~14 ohm R_SPI, so concurrent flash program + PSRAM traffic lands
    //         at ~2.53 V against a 2.7 V minimum for both — out of spec
    //         exactly during heavy logging. The prescribed fix is a schematic
    //         change (move U13's VCC to +3V3) and is still unaddressed.
    //       * rocket_computer_mini/sdkconfig.defaults already carries a
    //         "PSRAM: deliberately OFF — do not enable" note for this same
    //         part, citing rail sag measured on radio_board and base station.
    //       * TR_LogToFlash asks for MALLOC_CAP_INTERNAL, so enabling PSRAM
    //         alone would not move the ring anyway.
    //     And note it would not buy back the reboot features below: PSRAM is
    //     volatile. It restores ring SIZE, never recovery.
    //   * the ring is volatile, so the FC's in-flight reboot recovery (#104,
    //     GET_FLIGHT_SNAPSHOT) and the #274 dirty-ring replay have nowhere to
    //     live and are UNAVAILABLE. main.cpp says so at boot rather than
    //     failing quietly; the mini's design (snapshots written into the NAND
    //     log stream + a boot tail-scan) is the replacement when someone gets
    //     to it.
    // Leaving it at 34 would not "keep MRAM working" — it would drive a
    // floating pad and hand both features an unconnected device.
    static constexpr int MRAM_CS = -1;       // U12 not fitted (CONFIRMED)
    // ...and this is where the ring actually goes. U15 is an ESP32-S3RH2 with
    // 2 MB of in-package quad PSRAM (datasheet Table 1-1; the RH2 is the
    // official upgrade of the EOL S3R2). Deleting the MRAM and moving to the
    // embedded-PSRAM part were ONE decision, so the ring belongs in PSRAM here
    // — not in the 64 KB internal fallback, which is what firmware used until
    // #822 simply because CONFIG_SPIRAM was never enabled.
    // Wiring is correct as fabbed: quad PSRAM sits on SPICS1 (pad 28, NC
    // externally by design), and GPIO35-38 stay free for the NAND because the
    // GPIO33-37 restriction applies only to OCTAL parts. The VDD_SPI rail
    // objection (H7) was fixed before fab — see sdkconfig.defaults.
    // Still volatile: this buys ring SIZE, never reboot recovery.
    static constexpr bool RING_IN_PSRAM = true;   // 2 MB in-package (CONFIRMED)

    // --- I2C slave (commands from FlightComputer) ---
    static constexpr int I2C_SDA_PIN = 5;    // ESP_SDA (CONFIRMED)
    static constexpr int I2C_SCL_PIN = 6;    // ESP_SCL (CONFIRMED)

    // --- I2S slave RX (high-frequency telemetry from FlightComputer) ---
    // ESP_* net convention, fixed against the FC-side schematic (FC pins:
    // ESP_SCLK=21, ESP_CS=18, ESP_SDO=19, ESP_SDI=20) and mirrored in the
    // FC's board headers — both ends must agree PER NET:
    //   ESP_SCLK = bit clock, ESP_SDO = FC data out (our DIN),
    //   ESP_CS = word select, ESP_SDI = frame sync.
    static constexpr int I2S_BCLK_PIN  = 2;  // ESP_SCLK
    static constexpr int I2S_WS_PIN    = 1;  // ESP_CS
    static constexpr int I2S_DIN_PIN   = 3;  // ESP_SDO
    static constexpr int I2S_FSYNC_PIN = 4;  // ESP_SDI

    // --- Radio topology: UART daughterboard (#409/#410) — no direct LLCC68.
    // USE_LORA_RADIO=true: the radio paths run, through the UART-modem
    // backend rather than the direct SPI driver.
    static constexpr bool USE_LORA_RADIO = true;
    static constexpr bool USE_UART_RADIO_MODEM = true;
    static constexpr int LORA_SPI_SCK = -1;
    static constexpr int LORA_SPI_MISO = -1;
    static constexpr int LORA_SPI_MOSI = -1;
    static constexpr int LORA_CS_PIN = -1;
    static constexpr int LORA_DIO1_PIN = -1;
    static constexpr int LORA_RST_PIN = -1;
    static constexpr int LORA_BUSY_PIN = -1;

    // --- Radio daughterboard host link (consumed by the UART-modem backend,
    //     #410; constants staged here so the pin map is complete) ---
    // CONFIRMED against both PCBs' pad->net maps, not the labels alone: our
    // J5.4 (net LoRa_TX, GPIO11) lands on the daughterboard's J6.4 (its net
    // LoRa_RX, its GPIO5), and our J5.3 (net LoRa_RX, GPIO10) lands on its
    // J6.3 (its net LoRa_TX, its GPIO6). The two boards name the nets from
    // their own perspective, so the same pair of names appears on both ends
    // of a crossed link — the cable pin number is the only unambiguous
    // reference. (This is the trap that #234/RunCam hit; it is resolved here,
    // so do NOT "fix" it by swapping these two on bring-up.)
    static constexpr int LORA_UART_TX_PIN = 11;  // LoRa_TX -> J5.4 (CONFIRMED)
    static constexpr int LORA_UART_RX_PIN = 10;  // LoRa_RX <- J5.3 (CONFIRMED)
    // Daughterboard power gate (high = powered), like CAM_ACT for the RunCam.
    // Also the recovery hammer for a wedged/bricked daughterboard (#412).
    static constexpr int LORA_ACT_PIN = 12;      // LoRa_ACT (CONFIRMED)

    // --- High-side-switch current monitors (V9 close-out: TPS22811 U26/U28) ---
    // IMON sources GIMON x ILOAD into a ground-referenced gain resistor:
    //   ILOAD = V(pin) / (IMON_GAIN_A_PER_A * R).
    // GIMON is 95.3 uA/A typical but 82.9-107.6 over temp (datasheet, at
    // 1.5 A) — calibrate against a known load before trusting absolute amps.
    // Gain resistors per hardware PR #728: CAM R85 = 2.0 k (2.2 k in the
    // original V9 close-out) and SERVO R88 = 1.0 k — deliberately landing
    // BOTH channels near 0.286 V at their design currents (camera 1.5 A,
    // servo 3 A per high-side-switch-design.md). Nothing reads these yet;
    // staged so the first implementation cannot inherit the 2.2 k scale.
    static constexpr int   CAM_IMON_GPIO      = 8;         // CAM_IMON, ADC1_CH7
    static constexpr int   SERVO_IMON_GPIO    = 9;         // SERVO_IMON, ADC1_CH8
    static constexpr float IMON_GAIN_A_PER_A  = 95.3e-6f;  // TPS22811 GIMON typ
    static constexpr float CAM_IMON_R_OHM     = 2000.0f;   // R85 (PR #728)
    static constexpr float SERVO_IMON_R_OHM   = 1000.0f;   // R88
};
