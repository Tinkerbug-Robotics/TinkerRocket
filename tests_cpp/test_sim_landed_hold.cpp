// #971: a simulated flight never reached LANDED.
//
// The sim held SIM_LANDED for a fixed 9000 ms, chosen from a comment that
// budgeted 5 s for alt_landed_flag plus a 2 s debounce and assumed 2 s of
// margin.  Measured on flight_20260827_122854 the flag took 7.0 s, and the FC
// wants the flag held for STRICTLY MORE than 2000 ms on top — so 9000 ms was
// exactly the requirement and the transition missed by a margin of zero, on
// every sim flight, silently.
//
// Nothing compared those two numbers, which is why it went unnoticed.  These
// tests are that comparison.

#include <gtest/gtest.h>
#include "sim_landed_hold.h"

namespace {

constexpr uint8_t INFLIGHT = 3;
constexpr uint8_t LANDED   = 4;

using sim_landed::Exit;
using sim_landed::decide;

// ── The regression, stated as arithmetic ────────────────────────────────────

TEST(SimLandedHold, legacyHoldCouldNeverSatisfyTheFcRequirement)
{
    // The FC needs the flag held STRICTLY LONGER than the debounce, so the
    // total requirement is strictly greater than latch + debounce.
    constexpr uint32_t required =
        sim_landed::MEASURED_FLAG_LATCH_MS + sim_landed::FC_LANDED_DEBOUNCE_MS;

    EXPECT_EQ(sim_landed::LEGACY_HOLD_MS, required)
        << "the old hold was exactly the requirement, not the requirement plus margin";
    EXPECT_FALSE(sim_landed::LEGACY_HOLD_MS > required)
        << "a strictly-greater comparison can never be satisfied at equality";
}

TEST(SimLandedHold, backstopComfortablyExceedsTheRequirement)
{
    constexpr uint32_t required =
        sim_landed::MEASURED_FLAG_LATCH_MS + sim_landed::FC_LANDED_DEBOUNCE_MS;
    EXPECT_GT(sim_landed::HOLD_MAX_MS, required)
        << "the backstop must outlast the real path, or it truncates it again";
    // 1 Hz sub-flag counters quantize the latch, so leave room for a whole
    // extra tick beyond the measured value.
    EXPECT_GE(sim_landed::HOLD_MAX_MS, required + 1000)
        << "no margin for the 1 Hz quantization on the slow vote";
}

// ── The rule itself ─────────────────────────────────────────────────────────

TEST(SimLandedHold, endsAsSoonAsTheFcReportsLanded)
{
    // The whole point: stop guessing, ask the FC.  Even at t=0.
    EXPECT_EQ(decide(LANDED, LANDED, 0), Exit::FcLanded);
    EXPECT_EQ(decide(LANDED, LANDED, 12345), Exit::FcLanded);
}

TEST(SimLandedHold, keepsFeedingWhileTheFcIsStillInflight)
{
    EXPECT_EQ(decide(INFLIGHT, LANDED, 0), Exit::Hold);
    // Past the OLD 9 s hold the sim must still be feeding data — this is the
    // window the legacy number cut off.
    EXPECT_EQ(decide(INFLIGHT, LANDED, sim_landed::LEGACY_HOLD_MS), Exit::Hold);
    EXPECT_EQ(decide(INFLIGHT, LANDED, sim_landed::HOLD_MAX_MS - 1), Exit::Hold);
}

TEST(SimLandedHold, givesUpAtTheBackstopSoABrokenDetectorCannotHang)
{
    EXPECT_EQ(decide(INFLIGHT, LANDED, sim_landed::HOLD_MAX_MS), Exit::GaveUp);
    EXPECT_EQ(decide(INFLIGHT, LANDED, sim_landed::HOLD_MAX_MS + 5000), Exit::GaveUp);
}

TEST(SimLandedHold, landedWinsOverTheBackstop)
{
    // A late LANDED still reports success, not a give-up — the distinction is
    // what the log line tells the operator.
    EXPECT_EQ(decide(LANDED, LANDED, sim_landed::HOLD_MAX_MS + 1), Exit::FcLanded);
}

}  // namespace
