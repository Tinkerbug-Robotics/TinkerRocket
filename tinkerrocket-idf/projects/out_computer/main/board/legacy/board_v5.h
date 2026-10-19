#pragma once

#include <stdint.h>

// V5 ("TinkerRocket Full V5", fabbed 2026-04-17) pin map + board topology for
// the out computer. Selected with TR_BOARD_V5=1:
//   idf.py -B build_v5 -DTR_BOARD_V5=1 build
//
// Derived from a netlist export of the archived project; see the flight
// computer's board_v5.h for provenance. On this processor V5 and V7 differ in
// one pad: V7's GPIO33 is GPS_ACT, the GNSS rail enable, and V5's is a NEOPIX
// label with nothing on it — the GNSS connector (J3) takes VPP directly. Every
// other pin below is board_v7.h's, net for net.
//
// ONLY pins and peripheral-presence/topology flags live here. Logic and
// protocol constants (I2S sample rate, I2C address/speeds, LoRa RF
// parameters, storage sizes...) stay in config.h and must not fork per
// board revision.
struct board_pins
{
    // --- Power rail switch ---
    static constexpr int PWR_PIN = 6;            // POWER_SWITCH
    // No GNSS rail of its own (see above). -1 = call sites skip it.
    static constexpr int GPS_PWR_PIN = -1;
    // The arm lines are the flight computer's alone; no consent stage here.
    static constexpr int ARM_CONSENT_PIN = -1;
    static constexpr int INA_ALERT_PIN = -1;     // #1553: none
    static constexpr int PG_RAIL_PIN = -1;       // #1593: V10 only
    static constexpr int VBUCK_OK_PIN = -1;      // #1593: V10 only

    // --- Power monitoring (INA230 @ 0x40, always-on I2C bus) ---
    static constexpr int PWR_SDA = 7;
    static constexpr int PWR_SCL = 8;

    // --- Memory shared SPI bus ---
    static constexpr int SPI_SCK = 37;           // M_SCK
    static constexpr int SPI_MISO = 35;          // M_MISO
    static constexpr int SPI_MOSI = 38;          // M_MOSI
    static constexpr int NAND_CS = 36;           // M_FLASH_CS (U11)
    // MR25H10 (U12) on the shared bus.
    static constexpr int MRAM_CS = 34;
    // MRAM is fitted; the ring lives there. See board_v9.h.
    static constexpr bool RING_IN_PSRAM = false;

    // #1485: the highest IMU logging rate this board flies. 3840 Hz until its
    // FC reads the IMU from the FIFO (ISM6_FIFO_CAPTURE in the FC's board
    // header), which 7680 Hz needs; test_imu_rate_board_parity pins the two.
    static constexpr uint16_t IMU_RATE_MAX_HZ = 3840;

    // --- I2C slave (commands from FlightComputer) ---
    static constexpr int I2C_SDA_PIN = 4;        // ESP_SDA
    static constexpr int I2C_SCL_PIN = 5;        // ESP_SCL

    // --- I2S slave RX (high-frequency telemetry from FlightComputer) ---
    // Wiring: FC 27->OC 21, FC 28->OC 45, FC 23->OC 2, FC 17->OC 1 (as V7).
    static constexpr int I2S_BCLK_PIN  = 21;     // ESP_SDI
    static constexpr int I2S_WS_PIN    = 45;     // ESP_SDO
    static constexpr int I2S_DIN_PIN   = 2;      // ESP_SCLK
    static constexpr int I2S_FSYNC_PIN = 1;      // ESP_CS

    // --- Radio topology: E220-900MM22S (LLCC68) on SPI ---
    // TXEN is tied to the module's DIO2. L_RXEN reaches GPIO10 with no pull
    // resistor and, as on V7 (the same net), this project does not drive it.
    static constexpr bool USE_LORA_RADIO = true;
    static constexpr bool USE_UART_RADIO_MODEM = false;
    static constexpr int LORA_SPI_SCK = 14;
    static constexpr int LORA_SPI_MISO = 11;
    static constexpr int LORA_SPI_MOSI = 12;
    static constexpr int LORA_CS_PIN = 18;
    static constexpr int LORA_DIO1_PIN = 9;
    static constexpr int LORA_RST_PIN = 17;
    static constexpr int LORA_BUSY_PIN = 13;

    // --- Radio daughterboard host link (V8-only topology, #410) ---
    static constexpr int LORA_UART_TX_PIN = -1;
    static constexpr int LORA_UART_RX_PIN = -1;
    static constexpr int LORA_ACT_PIN = -1;
    // --- High-side-switch current monitors: not present on V5 ---
    static constexpr int   SCAP_ADC_PIN       = -1;
    static constexpr float SCAP_DIVIDER_RATIO = 1.0f;    // unused with no pin
    static constexpr int   CAM_IMON_GPIO      = -1;
    static constexpr int   SERVO_IMON_GPIO    = -1;
    static constexpr float IMON_GAIN_A_PER_A  = 0.0f;
    static constexpr float CAM_IMON_R_OHM     = 0.0f;
    static constexpr float SERVO_IMON_R_OHM   = 0.0f;
};
