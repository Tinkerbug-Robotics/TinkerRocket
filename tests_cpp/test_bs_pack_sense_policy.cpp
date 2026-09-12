#include <gtest/gtest.h>

#include <cmath>

#include "bs_pack_sense_policy.h"

using namespace bs_pack_sense;

// #714 — the base station's flight-pack sense. Two dividers on the charger
// jack (PosADC: the whole 2S pack; MidADC: the top of cell 1) become pack and
// per-cell voltages here. The hardware facts behind the numbers are in
// board_v3.h and test_bs_board_map.cpp; this file is about what the two
// readings MEAN.

namespace {
void expectNoPack(const Reading& r) {
    EXPECT_TRUE(std::isnan(r.pack_v));
    EXPECT_TRUE(std::isnan(r.cell1_v));
    EXPECT_TRUE(std::isnan(r.cell2_v));
}
}  // namespace

TEST(BsPackSense, AHealthyPackSplitsIntoItsCells) {
    const Reading r = derive(8.31f, 4.17f, /*charger_says_absent=*/false);
    EXPECT_NEAR(r.pack_v, 8.31f, 1e-5);
    EXPECT_NEAR(r.cell1_v, 4.17f, 1e-5);
    EXPECT_NEAR(r.cell2_v, 4.14f, 1e-4);
}

TEST(BsPackSense, AnImbalancedPackIsReportedAsItIs) {
    // The split is the fact; whether 0.40 V apart is worth a word is the
    // app's one line under the row, not a verdict taken here.
    const Reading r = derive(7.90f, 4.15f, false);
    EXPECT_NEAR(r.cell1_v, 4.15f, 1e-5);
    EXPECT_NEAR(r.cell2_v, 3.75f, 1e-4);
}

TEST(BsPackSense, AnOpenJackIsNoPack) {
    // Charger leakage through the divider: tens of millivolts.
    expectNoPack(derive(0.04f, 0.02f, false));
}

TEST(BsPackSense, TheFloorIsATwoCellFloor) {
    // A live 2S pack cannot sit below ~5 V without being destroyed, so 5 V
    // is where "no number about it is worth acting on" ends.
    expectNoPack(derive(4.99f, 2.5f, false));
    EXPECT_NEAR(derive(5.00f, 2.5f, false).pack_v, 5.0f, 1e-5);
}

TEST(BsPackSense, AnUnreadableChannelIsNoPack) {
    // The ADC could not be read at all: NaN in, NaN out — never a number.
    expectNoPack(derive(NAN, 4.1f, false));
}

TEST(BsPackSense, TheChargerRegulatingOntoAnEmptyJackIsNoPack) {
    // Charge input present, no pack: the MP2672 drives its own output onto
    // the pack terminal and PosADC reads a convincing 8.4 V of nothing. Its
    // BATTFLOAT flag is the only thing that can tell, so it vetoes.
    expectNoPack(derive(8.4f, 4.2f, /*charger_says_absent=*/true));
}

TEST(BsPackSense, AnUnreadableMidTapStillReportsThePack) {
    const Reading r = derive(8.31f, NAN, false);
    EXPECT_NEAR(r.pack_v, 8.31f, 1e-5);
    EXPECT_TRUE(std::isnan(r.cell1_v));
    EXPECT_TRUE(std::isnan(r.cell2_v));
}

TEST(BsPackSense, AMidTapAtEitherRailIsNotACell) {
    // Open tap (reads ~0) or a tap shorted to the pack top: the pack is
    // still a pack, the cells are unknown rather than 0.1 V and 8.2 V.
    Reading r = derive(8.31f, 0.1f, false);
    EXPECT_NEAR(r.pack_v, 8.31f, 1e-5);
    EXPECT_TRUE(std::isnan(r.cell1_v));
    r = derive(8.31f, 8.2f, false);
    EXPECT_NEAR(r.pack_v, 8.31f, 1e-5);
    EXPECT_TRUE(std::isnan(r.cell2_v));
}
