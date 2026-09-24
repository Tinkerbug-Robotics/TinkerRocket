#include <gtest/gtest.h>

#include "bs_storage_policy.h"
#include "config.h"

// #835 items 1, 2 and 4 — board_v3.h described hardware that is not on the
// board. Netlist-verified against hardware/legacy/base-station (rev V6, electrically
// identical to the base-station-v5.0.0 fab tag on every pin that matters).
//
// Each of the three was a claim no test could contradict, because nothing
// asserted the headers against anything. This file is compiled once per board
// (TR_BS_BOARD=1/2/3/4) so a claim has to survive being stated on every board.
// Board 4 is the Tinker-Base, netlist-verified against hardware/tinker-base.
//
// The V2 map is deliberately NOT asserted against the netlist: the bench unit
// is a real board that predates the repo's hardware import (which starts at
// V5), so every pin it names is an unconnected pad on the tracked schematic.
// Its numbers are bench-verified, not netlist-verified — do not "fix" them.

#if TR_BS_BOARD == 3

// Item 1. The only flash is U1, the boot NOR on the S3's DEDICATED SPI0 pins.
// GPIO4/5/6/7 are unconnected pads. Claiming a NAND made
// bs_storage_policy::demoted() report a demotion on every boot and log
// "**** STORAGE DEMOTED ****" for hardware working exactly as designed.
TEST(BsBoardMapV3, HasNoExternalNandOrSdSlot) {
    EXPECT_FALSE(config::HAS_EXT_NAND);
    EXPECT_FALSE(config::HAS_SDMMC);
    EXPECT_EQ(config::FLASH_SCK, -1);
    EXPECT_EQ(config::FLASH_MOSI, -1);
    EXPECT_EQ(config::FLASH_CS, -1);
    EXPECT_EQ(config::FLASH_MISO, -1);
}

// The consequence, stated directly: with no NAND declared, SPIFFS is the
// intended backing and not a demotion.
TEST(BsBoardMapV3, SpiffsIsNotADemotionOnThisBoard) {
    EXPECT_FALSE(bs_storage_policy::demoted(config::HAS_EXT_NAND, /*on_spiffs=*/true))
        << "SPIFFS is where this board is supposed to log";
}

// Item 2. The MAX17303 was deleted in 15da738, two weeks before the fab tag,
// and replaced by DW01A + FS8205A. The cell is read through a divider instead.
TEST(BsBoardMapV3, HasNoFuelGaugeAndReadsADivider) {
    EXPECT_FALSE(config::HAS_FUEL_GAUGE);
    EXPECT_FALSE(config::EXPECT_MAX17303);
    EXPECT_EQ(config::BATT_VSENSE_GPIO, 1);          // net Volt_Read, ADC1_CH0
    EXPECT_FLOAT_EQ(config::BATT_VSENSE_DIVIDER, 2.0f);  // R44 1M / R46 1M
}

// The attenuation and the divider are one decision, not two: a full 4.2 V cell
// divides to 2.1 V, which is outside the ~1.75 V calibrated range at 6 dB. The
// cali curve is per-attenuation, so a mismatch mis-scales every read silently.
TEST(BsBoardMapV3, AttenuationCoversAFullCell) {
    const float full_cell_v = 4.2f;
    const float at_pin_v = full_cell_v / config::BATT_VSENSE_DIVIDER;
    EXPECT_NEAR(at_pin_v, 2.1f, 1e-4);
    EXPECT_EQ(config::BATT_VSENSE_ATTEN_DB, 12)
        << at_pin_v << " V at the pin does not fit 6 dB's ~1.75 V range";
}

// #714. The flight pack on J4 is read on two dividers: PosADC, the whole 2S
// pack (R40 1 M / R42 180 k since hardware PR #728), and MidADC, the top of
// cell 1 (R50 100 k / R51 100 k). Netlist: external_charger.kicad_sch.
TEST(BsBoardMapV3, ReadsTheFlightPackOnTwoDividers) {
    EXPECT_TRUE(config::HAS_PACK_CHARGER);
    EXPECT_EQ(config::PACK_VSENSE_GPIO, 8);                  // PosADC, ADC1_CH7
    EXPECT_NEAR(config::PACK_VSENSE_DIVIDER, 1180.0f / 180.0f, 1e-4);
    EXPECT_EQ(config::PACK_MID_VSENSE_GPIO, 9);              // MidADC, ADC1_CH8
    EXPECT_FLOAT_EQ(config::PACK_MID_VSENSE_DIVIDER, 2.0f);
}

// The two attenuations differ on purpose. A full 8.4 V pack puts 1.281 V on
// PosADC, inside 6 dB's ~1.75 V calibrated range; a full 4.2 V cell puts
// 2.1 V on MidADC, outside it — so the mid tap runs at 12 dB, and each
// channel needs its own adc_cali handle because the curve is per-attenuation.
TEST(BsBoardMapV3, PackAttenuationsMatchTheirDividers) {
    const float pos_at_pin = 8.4f / config::PACK_VSENSE_DIVIDER;
    EXPECT_NEAR(pos_at_pin, 1.281f, 1e-3);
    EXPECT_LT(pos_at_pin, 1.75f);
    EXPECT_EQ(config::PACK_VSENSE_ATTEN_DB, 6);
    const float mid_at_pin = 4.2f / config::PACK_MID_VSENSE_DIVIDER;
    EXPECT_NEAR(mid_at_pin, 2.1f, 1e-4);
    EXPECT_GT(mid_at_pin, 1.75f) << "would fit 6 dB, and then it should use it";
    EXPECT_EQ(config::PACK_MID_VSENSE_ATTEN_DB, 12);
}

// Item 4. LoRa_EN = GPIO21 -> R18 1k -> TPS61023 EN, R17 100k to +3V3.
TEST(BsBoardMapV3, HasTheDaughterboardPowerGate) {
    EXPECT_EQ(config::LORA_ACT_PIN, 21);
}

// The crossed UART link. Confirmed on BOTH ends of the cable, and the header
// warns against "fixing" it — pin numbers are the only unambiguous reference.
TEST(BsBoardMapV3, UartPinsMatchTheConnector) {
    EXPECT_EQ(config::LORA_UART_TX_PIN, 35);   // -> J6.4
    EXPECT_EQ(config::LORA_UART_RX_PIN, 36);   // <- J6.3
}

#elif TR_BS_BOARD == 4  // the Tinker-Base

// The radio is the on-board E220-900MM22S, driven directly over SPI — the path
// V1/V2 take — not the V3 daughterboard link. Pin for pin these are the nets
// on U3 in hardware/tinker-base, and the GPIOs the lora-daughterboard's own
// ESP32-S3 uses for the same module (projects/radio_board/main/config.h).
TEST(BsBoardMapV4, DrivesTheOnBoardRadioOverSpi) {
    EXPECT_FALSE(config::USE_UART_RADIO_MODEM);
    EXPECT_EQ(config::LORA_SPI_SCK, 17);    // L_SCK
    EXPECT_EQ(config::LORA_SPI_MISO, 33);   // L_MISO
    EXPECT_EQ(config::LORA_SPI_MOSI, 21);   // L_MOSI
    EXPECT_EQ(config::LORA_CS_PIN, 18);     // L_CS
    EXPECT_EQ(config::LORA_DIO1_PIN, 2);    // L_DI01
    EXPECT_EQ(config::LORA_RST_PIN, 38);    // L_RST
    EXPECT_EQ(config::LORA_BUSY_PIN, 34);   // L_BUSY
    EXPECT_EQ(config::LORA_RXEN_PIN, 35);   // L_RXEN
}

// TXEN is looped to the radio's own DIO2 on the module side and DIO3 is
// unconnected (the module has its own crystal), so the MCU drives neither.
// There is no daughterboard link and no power gate to drive either.
TEST(BsBoardMapV4, HasNoDaughterboardLinkOrMcuDrivenTxSwitch) {
    EXPECT_EQ(config::LORA_DIO2_PIN, -1);
    EXPECT_EQ(config::LORA_DIO3_PIN, -1);
    EXPECT_EQ(config::LORA_UART_TX_PIN, -1);
    EXPECT_EQ(config::LORA_UART_RX_PIN, -1);
    EXPECT_EQ(config::LORA_ACT_PIN, -1);
}

// No radio signal may sit on an S3 strapping pin (0, 3, 45, 46), the USB pair
// (19, 20) or the SPI0/1 pins the boot NOR and the in-package PSRAM use
// (26-32), and no two may share a pin.
TEST(BsBoardMapV4, RadioPinsAreDistinctAndOffReservedPins) {
    const int pins[] = {config::LORA_SPI_SCK, config::LORA_SPI_MISO, config::LORA_SPI_MOSI,
                        config::LORA_CS_PIN, config::LORA_DIO1_PIN, config::LORA_RST_PIN,
                        config::LORA_BUSY_PIN, config::LORA_RXEN_PIN};
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        const int p = pins[i];
        EXPECT_NE(p, 0);  EXPECT_NE(p, 3);  EXPECT_NE(p, 45); EXPECT_NE(p, 46);
        EXPECT_NE(p, 19); EXPECT_NE(p, 20);
        EXPECT_FALSE(p >= 26 && p <= 32) << "GPIO" << p << " is an SPI0/1 flash/PSRAM pin";
        for (size_t j = i + 1; j < sizeof(pins) / sizeof(pins[0]); ++j)
            EXPECT_NE(p, pins[j]) << "two radio signals on GPIO" << p;
    }
}

// Storage as on V3: the only flash is U1, the boot NOR; GPIO4-7 are
// unconnected pads, so SPIFFS is the intended backing, not a demotion.
TEST(BsBoardMapV4, LogsToSpiffsOnTheBootNor) {
    EXPECT_FALSE(config::HAS_EXT_NAND);
    EXPECT_FALSE(config::HAS_SDMMC);
    EXPECT_EQ(config::FLASH_CS, -1);
    EXPECT_FALSE(bs_storage_policy::demoted(config::HAS_EXT_NAND, /*on_spiffs=*/true));
}

// No I2C device on the netlist at all: no gauge (DW01A + FS8205A protect the
// cell, as on V3) and no pack charger. main.cpp skips the bus on -1 pins.
TEST(BsBoardMapV4, HasNoI2cBusGaugeOrPackCharger) {
    EXPECT_EQ(config::I2C_SDA_PIN, -1);
    EXPECT_EQ(config::I2C_SCL_PIN, -1);
    EXPECT_FALSE(config::HAS_FUEL_GAUGE);
    EXPECT_FALSE(config::EXPECT_MAX17303);
    EXPECT_FALSE(config::HAS_PACK_CHARGER);
    EXPECT_EQ(config::PACK_VSENSE_GPIO, -1);
    EXPECT_EQ(config::PACK_MID_VSENSE_GPIO, -1);
}

// The cell divider is V3's, net for net: V_SWITCH -> R44 1M -> Volt_Read ->
// R46 1M -> GND on GPIO1, read at 12 dB because a full cell gives 2.1 V.
TEST(BsBoardMapV4, ReadsTheCellOnV3sDivider) {
    EXPECT_EQ(config::BATT_VSENSE_GPIO, 1);
    EXPECT_FLOAT_EQ(config::BATT_VSENSE_DIVIDER, 2.0f);
    EXPECT_EQ(config::BATT_VSENSE_ATTEN_DB, 12);
}

#else  // V1 / V2 — gauged boards

// The flag must be present and true on every gauged board, or the shared ADC
// path in updateBattery() would start reporting a voltage from a pin that is
// not connected to a divider.
TEST(BsBoardMapGauged, DeclaresAFuelGauge) {
    EXPECT_TRUE(config::HAS_FUEL_GAUGE);
    EXPECT_EQ(config::BATT_VSENSE_GPIO, -1) << "no divider on a gauged board";
}

// #714: no charger jack on the gauged boards, so no pack dividers — the
// shared initSenseAdc() skips a -1 and updatePackSense() never runs.
TEST(BsBoardMapGauged, HasNoFlightPackDividers) {
    EXPECT_FALSE(config::HAS_PACK_CHARGER);
    EXPECT_EQ(config::PACK_VSENSE_GPIO, -1);
    EXPECT_EQ(config::PACK_MID_VSENSE_GPIO, -1);
}

#endif

// True on every board: the two must agree, or main.cpp's `if constexpr`
// selects a path the hardware cannot serve.
TEST(BsBoardMapAll, GaugeFlagAndDividerAreConsistent) {
    if (config::HAS_FUEL_GAUGE) {
        EXPECT_EQ(config::BATT_VSENSE_GPIO, -1);
    } else {
        EXPECT_GE(config::BATT_VSENSE_GPIO, 0)
            << "no gauge AND no divider would leave the battery unreadable";
    }
}
