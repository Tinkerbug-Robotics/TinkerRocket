// #1593: the Tinker-Mantis V10's supply status lines on the out computer.
// PG_RAIL (the pack eFuse's power-good) and VBUCK_OK (the buck output, through
// a divider) both read high when all is well. The OC reports their level at
// boot and every change after it; the counts ride the stats lines.
#include <gtest/gtest.h>
#include "supply_status_policy.h"
#include <SPI.h>       // host shim: SPI_MODE0, which the OC config.h names
#include "config.h"    // the SHIPPED V10 pins (TR_BOARD_V10 from CMake)

using SupplyStatus::Change;
using SupplyStatus::Line;
using SupplyStatus::sample;

TEST(SupplyStatus1593, TheBootLevelIsReportedAndCountsNothing)
{
    Line l;
    EXPECT_EQ(sample(l, true, 100), Change::First);
    EXPECT_TRUE(l.high);
    EXPECT_EQ(l.falls, 0u);
    EXPECT_EQ(l.rises, 0u);

    // A board that boots with the pack already cut reports it as the boot
    // level, not as a fall.
    Line low;
    EXPECT_EQ(sample(low, false, 100), Change::First);
    EXPECT_FALSE(low.high);
    EXPECT_EQ(low.falls, 0u);
}

TEST(SupplyStatus1593, EachChangeIsOneFallOrRiseAndARepeatIsNone)
{
    Line l;
    sample(l, true, 0);
    EXPECT_EQ(sample(l, true, 10), Change::None);
    EXPECT_EQ(sample(l, false, 20), Change::Fell);
    EXPECT_EQ(l.changed_ms, 20u);
    EXPECT_EQ(sample(l, false, 30), Change::None);
    EXPECT_EQ(sample(l, true, 40), Change::Rose);
    EXPECT_EQ(sample(l, false, 50), Change::Fell);
    EXPECT_EQ(l.falls, 2u);
    EXPECT_EQ(l.rises, 1u);
    EXPECT_FALSE(l.high);
    EXPECT_EQ(l.changed_ms, 50u);
}

TEST(SupplyStatus1593, TheV10ShipsTheNetlistPins)
{
    // board_v10.h, from the netlist: PG_RAIL on S3 GPIO17, VBUCK_OK on GPIO34.
    EXPECT_EQ(config::PG_RAIL_PIN, 17);
    EXPECT_EQ(config::VBUCK_OK_PIN, 34);
    // And the arm consent the same board carries, which the stats sit beside.
    EXPECT_EQ(config::ARM_CONSENT_PIN, 14);
    EXPECT_EQ(config::INA_ALERT_PIN, 18);
}
