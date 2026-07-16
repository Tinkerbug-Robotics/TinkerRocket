#include <gtest/gtest.h>

#include "bs_battery_soc.h"

using namespace bs_battery_soc;

// The #501 fix: the BQ27Z746 reported 0% SoC with the pack at 4.17 V and
// accepting 449 mA (RemainingCapacity=0, FullChargeCapacity=27932 mAh vs a
// 2800 mAh design). Impedance Track derives RM/FCC/SoC by integrating current,
// and this part's CC Gain was never calibrated (GAUGE_EN also ships at 0), so
// all three are garbage while Voltage() — a direct ADC read — stays correct.
//
// These tests pin the voltage-based fallback and, just as importantly, the
// plausibility check that decides when NOT to believe the gauge.

namespace {
constexpr float kDesignMah = 2800.0f;
constexpr int   k1S = 1;
}  // namespace

// ---- The plausibility gate ----

// Exactly the bench reading that opened #501. This must be rejected.
TEST(BsBatterySoc, TheBenchGaugeReadingIsRejected) {
    EXPECT_FALSE(gaugeSocPlausible(/*gauge_en=*/false, /*soc=*/0.0f, /*fcc=*/27932.0f,
                                   kDesignMah, /*pack_v=*/4.172f, k1S));
}

TEST(BsBatterySoc, GaugingDisabledIsNeverPlausible) {
    // Even a perfectly reasonable-looking SoC is meaningless if Impedance Track
    // never ran — nothing integrated the current that produced it.
    EXPECT_FALSE(gaugeSocPlausible(/*gauge_en=*/false, /*soc=*/75.0f, /*fcc=*/2800.0f,
                                   kDesignMah, /*pack_v=*/3.95f, k1S));
}

TEST(BsBatterySoc, WildFullChargeCapacityIsRejected) {
    // FCC ~10x design — the smoking gun for an uncalibrated coulomb counter.
    EXPECT_FALSE(gaugeSocPlausible(true, 50.0f, 27932.0f, kDesignMah, 3.71f, k1S));
    // ...and the other direction (a badly under-learned pack).
    EXPECT_FALSE(gaugeSocPlausible(true, 50.0f, 800.0f, kDesignMah, 3.71f, k1S));
}

TEST(BsBatterySoc, ZeroSocOnAFullCellIsRejected) {
    // 0% at 4.17 V is physically impossible; don't pass it to the app.
    EXPECT_FALSE(gaugeSocPlausible(true, 0.0f, 2800.0f, kDesignMah, 4.17f, k1S));
}

TEST(BsBatterySoc, ZeroSocOnAGenuinelyEmptyCellIsAccepted) {
    // The mirror case: 0% at 3.2 V is real, and must NOT be second-guessed.
    EXPECT_TRUE(gaugeSocPlausible(true, 0.0f, 2800.0f, kDesignMah, 3.2f, k1S));
}

TEST(BsBatterySoc, OutOfRangeSocIsRejected) {
    EXPECT_FALSE(gaugeSocPlausible(true, -1.0f, 2800.0f, kDesignMah, 3.8f, k1S));
    EXPECT_FALSE(gaugeSocPlausible(true, 101.0f, 2800.0f, kDesignMah, 3.8f, k1S));
}

TEST(BsBatterySoc, AHealthyLearnedGaugeIsBelieved) {
    // The whole point of the gate: a working coulomb counter still wins.
    EXPECT_TRUE(gaugeSocPlausible(true, 72.0f, 2750.0f, kDesignMah, 3.90f, k1S));
}

// ---- The OCV curve ----

TEST(BsBatterySoc, CurveEndpointsAndClamping) {
    EXPECT_FLOAT_EQ(socFromCellOcv(4.20f), 100.0f);
    EXPECT_FLOAT_EQ(socFromCellOcv(3.00f), 0.0f);
    // Clamped, not extrapolated.
    EXPECT_FLOAT_EQ(socFromCellOcv(4.35f), 100.0f);
    EXPECT_FLOAT_EQ(socFromCellOcv(2.50f), 0.0f);
}

TEST(BsBatterySoc, CurveIsMonotonicNonDecreasing) {
    float prev = -1.0f;
    for (float v = 2.9f; v <= 4.3f; v += 0.005f) {
        const float s = socFromCellOcv(v);
        EXPECT_GE(s, prev) << "SoC must never fall as voltage rises (v=" << v << ")";
        EXPECT_GE(s, 0.0f);
        EXPECT_LE(s, 100.0f);
        prev = s;
    }
}

TEST(BsBatterySoc, InterpolatesBetweenTablePoints) {
    // Midway between {3.71, 50} and {3.75, 55} -> ~52.5%.
    EXPECT_NEAR(socFromCellOcv(3.73f), 52.5f, 0.2f);
}

// The pack that started all this: 4.17 V is nearly full, and must NOT read 0%.
TEST(BsBatterySoc, TheBenchVoltageReadsNearlyFull) {
    const float soc = socFromCellOcv(4.172f);
    EXPECT_GT(soc, 90.0f) << "4.17 V is a nearly full cell — the gauge said 0%";
    EXPECT_LE(soc, 100.0f);
}

// ---- IR compensation ----

TEST(BsBatterySoc, IrCompensationIsDisabledAtZeroResistance) {
    // The base station ships with r_int = 0 because the gauge's current is not
    // trustworthy — compensating with it would add error, not remove it.
    EXPECT_FLOAT_EQ(ocvFromTerminal(4.17f, /*current_ma=*/449.0f, /*r=*/0.0f), 4.17f);
}

TEST(BsBatterySoc, ChargingTerminalVoltageIsCorrectedDown) {
    // +449 mA into 0.1 Ohm lifts the terminal ~45 mV above true OCV.
    EXPECT_NEAR(ocvFromTerminal(4.17f, 449.0f, 0.1f), 4.1251f, 1e-3f);
}

TEST(BsBatterySoc, DischargingTerminalVoltageIsCorrectedUp) {
    EXPECT_NEAR(ocvFromTerminal(3.70f, -500.0f, 0.1f), 3.75f, 1e-3f);
}

// ---- The filtered estimator ----

TEST(BsBatterySoc, EstimatorSnapsToTheFirstSampleRatherThanRampingFromZero) {
    Estimator est(0.1f);
    EXPECT_FALSE(est.primed());

    const float soc = est.update(4.172f, k1S);
    EXPECT_TRUE(est.primed());
    // Must not report a near-empty pack for the first ~10 polls while an EMA
    // seeded at 0 V crawls upward.
    EXPECT_GT(soc, 90.0f);
}

TEST(BsBatterySoc, EstimatorSmoothsATransientVoltageSag) {
    Estimator est(0.1f);
    est.update(3.80f, k1S);                 // settled
    const float before = est.soc();

    // One LoRa TX spike drags the rail down hard for a single sample.
    const float during = est.update(3.55f, k1S);

    // The estimate should move a little, not collapse to the sagged voltage.
    const float sag_soc = socFromCellOcv(3.55f);
    EXPECT_LT(during, before);
    EXPECT_GT(during, sag_soc + 10.0f) << "one sag sample must not drag SoC to the sag value";
}

TEST(BsBatterySoc, EstimatorConvergesOnASustainedChange) {
    Estimator est(0.5f);
    est.update(4.20f, k1S);
    for (int i = 0; i < 40; i++) est.update(3.70f, k1S);

    EXPECT_NEAR(est.filteredCellV(), 3.70f, 0.01f);
    EXPECT_NEAR(est.soc(), socFromCellOcv(3.70f), 1.0f);
}

TEST(BsBatterySoc, EstimatorTracksADischargeDownward) {
    Estimator est(0.5f);
    float prev = est.update(4.20f, k1S);
    for (float v = 4.15f; v >= 3.40f; v -= 0.05f) {
        float s = 0.0f;
        for (int i = 0; i < 10; i++) s = est.update(v, k1S);   // let it settle
        EXPECT_LE(s, prev + 0.01f) << "SoC must not rise while the pack drains (v=" << v << ")";
        prev = s;
    }
    EXPECT_LT(prev, 30.0f) << "a pack at 3.4 V should read low, giving a usable warning";
}

TEST(BsBatterySoc, EstimatorHandlesMultiCellPacks) {
    // A 2S pack at 8.34 V is two 4.17 V cells — the per-cell curve must be used.
    Estimator est(1.0f);
    const float soc = est.update(8.344f, /*cells=*/2);
    EXPECT_NEAR(soc, socFromCellOcv(4.172f), 0.5f);
}

TEST(BsBatterySoc, EstimatorGuardsAgainstZeroCells) {
    Estimator est(1.0f);
    EXPECT_NO_FATAL_FAILURE(est.update(4.17f, /*cells=*/0));   // treated as 1S, no div-by-zero
}

TEST(BsBatterySoc, ResetClearsTheFilter) {
    Estimator est(0.1f);
    est.update(4.20f, k1S);
    ASSERT_TRUE(est.primed());
    est.reset();
    EXPECT_FALSE(est.primed());
    // Re-primes cleanly on the next sample rather than blending across the reset.
    EXPECT_NEAR(est.update(3.70f, k1S), socFromCellOcv(3.70f), 0.01f);
}
