// #971: a simulated flight never reached LANDED.
//
// The sim held SIM_LANDED for a fixed 9000 ms, chosen from a comment that
// budgeted 5 s for alt_landed_flag plus a 2 s debounce and assumed 2 s of
// margin.  Measured on flight_20260827_122854 the flag took 7.0 s, and the FC
// wanted the flag held for STRICTLY MORE than 2000 ms on top — so 9000 ms was
// exactly the requirement and the transition missed by a margin of zero, on
// every sim flight, silently.  (#1137 item 8 has since made that comparison
// `>=`; the hold is dynamic either way, so the margin no longer rides on it.)
//
// Nothing compared those two numbers, which is why it went unnoticed.  These
// tests are that comparison.

#include <gtest/gtest.h>
#include "sim_landed_hold.h"
#include "landing_transition_policy.h"

// #1137 item 8 moved the dwell into landing_transition_policy.h and relaxed
// the comparison from `> 2000U` to `>= kDwellMs`.  FC_LANDED_DEBOUNCE_MS is
// now a MIRROR of that constant, and a mirror nobody checks is how #971
// happened in the first place -- two numbers that had to agree, in two files,
// with nothing comparing them.  This is the comparison.
static_assert(sim_landed::FC_LANDED_DEBOUNCE_MS ==
                  landing_transition::kDwellMs,
              "sim_landed_hold.h's debounce mirror has drifted from "
              "landing_transition_policy.h's kDwellMs");

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

// #574: the healthy path is not the slowest one.  With the IMU stale from
// burnout, the ONLY route to LANDED is the baro-only backstop, whose
// alt_landed dwell is ~30 s -- measured at 30.8 s on a V9 (TR_SIM_DEAD_IMU,
// 2026-09-12), with LANDED 2 s later.  A backstop sized for the healthy vote
// expires inside that and the sim gives up before the flag can latch, which is
// how #574's dead-IMU half sat unverifiable.  Size off the slowest path.
TEST(SimLandedHold, backstopOutlastsTheDeadImuBaroDwell)
{
    // Measured dwell + the FC's state debounce, plus a 1 Hz quantization tick.
    constexpr uint32_t dead_imu_dwell_ms = 31000;
    constexpr uint32_t required =
        dead_imu_dwell_ms + sim_landed::FC_LANDED_DEBOUNCE_MS + 1000;
    EXPECT_GT(sim_landed::HOLD_MAX_MS, required)
        << "the backstop must outlast the DEAD-IMU baro dwell, not just the "
           "healthy vote -- at 30000 it expired within a second of the latch";
    EXPECT_EQ(decide(INFLIGHT, LANDED, dead_imu_dwell_ms), Exit::Hold)
        << "the sim must still be holding when the dead-IMU flag latches";
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
