#pragma once

#include <driver/uart.h>

// Radio daughterboard (ESP32-S3) pin/board configuration (#409).
//
// Board: "LoRa Board V3", the as-built daughterboard — ESP32-S3 (U28) + EBYTE
// E220-900MM22S (U14, LLCC68 + its own 32 MHz crystal) + W25Q64 boot flash,
// USB-C console/programming, 4-pin JST-SH host link (J6). Every pin below is
// the schematic net, verified against that board's .kicad_pcb pad->net map.
//
// A later revision exists (hardware/lora-daughterboard: S3RH2, W25Q128,
// different passives). Its GPIO net map is IDENTICAL — pad-by-pad diffed —
// so everything in this file serves both boards; only the flash size in
// sdkconfig.defaults is revision-specific.
struct config
{
    // ---- UART link to the host (OC on the rocket, BS on the ground) --------
    // The modem must never care which host it is plugged into (#409:
    // host-agnostic, swappable matched pairs).
    //
    // J6 mates 1:1 with rocket-computer J5 and base-station J6:
    //   J6.1 VSS  <- switched ground return (rocket: Q10 / LoRa_ACT)
    //   J6.2 +BATT<- 6.4-8.4 V pack (rocket) or ~4.6 V V_LORA (base station)
    //   J6.3 net LoRa_TX = GPIO6 -> host's RX   (we drive it)
    //   J6.4 net LoRa_RX = GPIO5 <- host's TX   (host drives it)
    // The net *names* are self-perspective on each board, so the same two
    // names appear on both ends of a crossed pair — the cable pin number is
    // the only unambiguous reference. Host ends: OC GPIO11 TX / GPIO10 RX
    // (board_v8.h), BS GPIO35 TX / GPIO36 RX (board_v3.h).
    static constexpr uart_port_t HOST_UART_PORT = UART_NUM_1;
    // Never derive these from the host's net names — the crossover above is
    // what makes the link work. Reversing them drives GPIO5 into the host's
    // TX: two push-pull CMOS outputs on one wire, nothing listening back.
    static constexpr int HOST_UART_TX = 6;   // LoRa_TX -> J6.3 (CONFIRMED)
    static constexpr int HOST_UART_RX = 5;   // LoRa_RX <- J6.4 (CONFIRMED)
    // Sized for tunnel traffic (LoRa airtime dominates), not raw UART
    // capability; must match the host side (#410).
    static constexpr int HOST_UART_BAUD = 921'600;

    // ---- LoRa radio (LLCC68 over SPI) — daughterboard L_* nets -------------
    static constexpr int LORA_SPI_SCK = 17;   // L_SCK
    static constexpr int LORA_SPI_MISO = 33;  // L_MISO
    static constexpr int LORA_SPI_MOSI = 21;  // L_MOSI
    static constexpr int LORA_CS_PIN = 18;    // L_CS
    static constexpr int LORA_DIO1_PIN = 2;   // L_DIO1
    static constexpr int LORA_RST_PIN = 38;   // L_RST
    static constexpr int LORA_BUSY_PIN = 34;  // L_BUSY
    static constexpr int LORA_RXEN_PIN = 35;  // L_RXEN -> U14 pad 10.
                                              // TX side is DIO2-driven: the
                                              // board shorts U14 DIO2 (19) to
                                              // TXEN (9), which is the split
                                              // EBYTE prescribes. Only the RX
                                              // half reaches an MCU pin.
    // U14 DIO3 is left floating on purpose — the E220 carries its own 32 MHz
    // passive crystal, so this radio must NOT be configured for a DIO3-powered
    // TCXO (TR_LoRa_Comms does not, which is why it is correct here).

    // ---- Indicator LEDs ------------------------------------------------------
    // TX LED is lit for the duration of a transmission (airtime-length flash);
    // RX LED pulses per received air packet. Both are anode-to-GPIO through a
    // series resistor with the cathode on GND, so active level is HIGH.
    //   GPIO7 = net IND_2 -> R60 -> D5 (red)
    //   GPIO8 = net IND_1 -> R59 -> D4 (blue)
    // D6 is a hardwired power LED — no GPIO, nothing to drive.
    // Bench note: R59/R60 are 10 k on LoRa V3, i.e. ~140 uA into the red and
    // 0-70 uA into the blue (hardware pre-fab review finding 9). If the blue
    // TX LED never visibly lights on the first article, that is the resistor
    // value, not this firmware.
    static constexpr int LED_RX_PIN = 7;
    static constexpr int LED_TX_PIN = 8;
    static constexpr uint32_t LED_ACTIVE_LEVEL = 1;  // flip if wired to VCC
    static constexpr uint32_t LED_RX_PULSE_MS = 40;

    // ---- Radio capability (reported in IDENTITY; hosts clamp to these) -----
    // These are the MODULE's limits, not the bare LLCC68 die's (150-960 MHz):
    // the E220-900MM22S has a fixed 900 MHz matching network and RF switch, so
    // the die's band edges are not reachable capability. Reporting the module
    // limits is the whole point of IDENTITY — it is what lets a different
    // module variant drop in later without a host firmware change.
    static constexpr int8_t RADIO_MAX_TX_POWER_DBM = 22;   // E220-900MM22S
    static constexpr float RADIO_FREQ_MIN_MHZ = 850.0f;    // E220-900MM22S
    static constexpr float RADIO_FREQ_MAX_MHZ = 930.0f;    //   850-930 MHz

    // ---- Boot radio defaults ------------------------------------------------
    // The modem comes up listening on these until the host pushes SET_CONFIG,
    // mirroring TR_LoRa_Comms defaults. The host owns the real config
    // (transparent-modem principle: no NVS on the modem).
    static constexpr float BOOT_FREQ_MHZ = 915.0f;
    static constexpr uint8_t BOOT_SF = 10;
    static constexpr float BOOT_BW_KHZ = 125.0f;
    static constexpr uint8_t BOOT_CR = 7;
    static constexpr int8_t BOOT_TX_POWER_DBM = 20;

    static constexpr bool DEBUG = true;
};
