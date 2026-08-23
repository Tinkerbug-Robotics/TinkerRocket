#include <gtest/gtest.h>

#include "bs_storage_policy.h"
#include "config.h"

// #835 items 1, 2 and 4 — board_v3.h described hardware that is not on the
// board. Netlist-verified against hardware/base-station (rev V6, electrically
// identical to the base-station-v5.0.0 fab tag on every pin that matters).
//
// Each of the three was a claim no test could contradict, because nothing
// asserted the headers against anything. This file is compiled once per board
// (TR_BS_BOARD=1/2/3) so a claim has to survive being stated three times.
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

#else  // V1 / V2 — gauged boards

// The flag must be present and true on every gauged board, or the shared ADC
// path in updateBattery() would start reporting a voltage from a pin that is
// not connected to a divider.
TEST(BsBoardMapGauged, DeclaresAFuelGauge) {
    EXPECT_TRUE(config::HAS_FUEL_GAUGE);
    EXPECT_EQ(config::BATT_VSENSE_GPIO, -1) << "no divider on a gauged board";
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
