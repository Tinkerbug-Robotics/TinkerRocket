#pragma once

#include <driver/uart.h>

// Radio daughterboard (ESP32-S3) pin/board configuration (#409).
//
// LoRa + LED pins are from the V8 daughterboard schematic (L_* / LED nets).
struct config
{
    // ---- UART link to the host (OC on the rocket, BS on the ground) --------
    // The modem must never care which host it is plugged into (#409:
    // host-agnostic, swappable matched pairs).
    static constexpr uart_port_t HOST_UART_PORT = UART_NUM_1;
    // Each board names these nets from ITS OWN perspective and the cable is
    // straight-through, so one wire has two different names depending which end
    // you read it from. Cable pin 3 is "LoRa_TX" here and "LoRa_RX" on the host;
    // pin 4 is the reverse. Never derive these from the host's net names -- the
    // crossover is what makes the link work. Verified against both hosts:
    //
    //   GPIO6 -> LoRa_TX -> J6.3 -> host RX   (OC GPIO10 / BS GPIO36)
    //   GPIO5 <- LoRa_RX <- J6.4 <- host TX   (OC GPIO11 / BS GPIO35)
    //
    // Reversing them drives GPIO5 into the host's TX: two push-pull CMOS
    // outputs on one wire, and nothing listening on the other.
    static constexpr int HOST_UART_TX = 6;
    static constexpr int HOST_UART_RX = 5;
    // Sized for tunnel traffic (LoRa airtime dominates), not raw UART
    // capability; must match the host side (#410).
    static constexpr int HOST_UART_BAUD = 921'600;

    // ---- LoRa radio (LLCC68 over SPI) — V8 daughterboard nets ---------------
    static constexpr int LORA_SPI_SCK = 17;   // L_SCK
    static constexpr int LORA_SPI_MISO = 33;  // L_MISO
    static constexpr int LORA_SPI_MOSI = 21;  // L_MOSI
    static constexpr int LORA_CS_PIN = 18;    // L_CS
    static constexpr int LORA_DIO1_PIN = 2;   // L_DIO1
    static constexpr int LORA_RST_PIN = 38;   // L_RST
    static constexpr int LORA_BUSY_PIN = 34;  // L_BUSY
    static constexpr int LORA_RXEN_PIN = 35;  // L_RXEN (RF switch RX enable;
                                              // TX side is DIO2-driven)

    // ---- Indicator LEDs ------------------------------------------------------
    // TX LED is lit for the duration of a transmission (airtime-length flash);
    // RX LED pulses per received air packet.
    static constexpr int LED_RX_PIN = 7;
    static constexpr int LED_TX_PIN = 8;
    static constexpr uint32_t LED_ACTIVE_LEVEL = 1;  // flip if wired to VCC
    static constexpr uint32_t LED_RX_PULSE_MS = 40;

    // ---- Radio capability (reported in IDENTITY; hosts clamp to these) -----
    static constexpr int8_t RADIO_MAX_TX_POWER_DBM = 22;   // LLCC68 ceiling
    static constexpr float RADIO_FREQ_MIN_MHZ = 150.0f;    // LLCC68 band edges
    static constexpr float RADIO_FREQ_MAX_MHZ = 960.0f;

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
