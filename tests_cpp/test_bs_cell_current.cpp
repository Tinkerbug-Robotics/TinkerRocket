// The Tinker-Base V2's INA230 -> the base station's battery fields
// (bs_cell_current.h). IN+ is the cell's + terminal and IN- is VCC, so the
// part reads discharge positive; bs_current, BLE "bcur" and the SoC
// estimator's IR term all take charge positive.
#include <gtest/gtest.h>

#include "bs_battery_soc.h"
#include "bs_cell_current.h"

using bs_cell_current::fromIna230;

// Running on the cell: 180 mA out of B+ into VCC is +3.6 mV across R62's
// 20 mOhm and +0.180 A in the CURRENT register. The base station reports -180.
TEST(BsCellCurrent, DischargeReadsNegative) {
    const auto r = fromIna230(3.8000f, 3.6f, 0.180f);
    EXPECT_FLOAT_EQ(r.current_ma, -180.0f);
}

// On USB with U5 charging: 400 mA into the cell flows VCC -> B+, so the shunt
// (-8 mV) and the CURRENT register both read negative. The base station
// reports +400.
TEST(BsCellCurrent, ChargeReadsPositive) {
    const auto r = fromIna230(4.1000f, -8.0f, -0.400f);
    EXPECT_FLOAT_EQ(r.current_ma, 400.0f);
}

TEST(BsCellCurrent, NoCurrentIsZero) {
    const auto r = fromIna230(3.9f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(r.current_ma, 0.0f);
    EXPECT_FLOAT_EQ(r.cell_v, 3.9f);
}

// BUS is VCC, the shunt's load end. Discharging, the cell sits ABOVE VCC by
// the shunt drop; charging, below it. BUS + SHUNT is B+ either way.
TEST(BsCellCurrent, CellVoltageAddsTheShuntDropBack) {
    EXPECT_NEAR(fromIna230(3.8000f, 3.6f, 0.180f).cell_v, 3.8036f, 1e-6);
    EXPECT_NEAR(fromIna230(4.1000f, -8.0f, -0.400f).cell_v, 4.0920f, 1e-6);
}

// The SoC estimator's IR term is written for charge-positive current: OCV =
// V - I*R. Fed through fromIna230, a discharge must raise the OCV estimate
// above the terminal (the cell sags under load), and a charge lower it.
TEST(BsCellCurrent, MatchesTheEstimatorsSign) {
    const float r_int = 0.05f;
    const auto dis = fromIna230(3.70f, 10.0f, 0.500f);
    EXPECT_GT(bs_battery_soc::ocvFromTerminal(dis.cell_v, dis.current_ma, r_int), dis.cell_v);
    const auto chg = fromIna230(4.10f, -10.0f, -0.500f);
    EXPECT_LT(bs_battery_soc::ocvFromTerminal(chg.cell_v, chg.current_ma, r_int), chg.cell_v);
}
