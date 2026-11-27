// #1772: the Beetle V2's raw-GNSS filter start check (config.h with
// TR_BOARD_M2 and TR_GNSS_DRIVER_PX1105R, as its CMake sets them; the rule,
// flight_computer/main/raw_nav_policy.h roomForFlightTask).
//
// startRawShadow does not start the raw filter -- the Beetle V2's only
// navigation filter -- without room for the flight task beside it. Elsewhere
// that is a floor on the largest internal block (56 KB, sized on the ESP32-P4).
// The Beetle V2's ESP32-S3 has far less internal RAM, in several heaps, so a
// floor on its largest block says little (the Beetle V1 itself would not have
// met one of 50 KB). It probes instead: with blocks the size of the filter's
// two stacks taken, the flight task's 16 KB stack must still find one block,
// with a margin free beside it.
#include <gtest/gtest.h>

#include "config.h"         // flight_computer/main/config.h
#include "raw_nav_policy.h"

#if !TR_BOARD_M2 || !(defined(TR_GNSS_DRIVER_PX1105R) && TR_GNSS_DRIVER_PX1105R)
#error "test_beetle_v2_raw_floor is built with TR_BOARD_M2=1 TR_GNSS_DRIVER_PX1105R=1"
#endif

namespace {

bool room(size_t largest_after, size_t free_after, bool probes_got = true)
{
    return RawNavPolicy::roomForFlightTask(probes_got, largest_after, free_after, config::FLIGHT_TASK_STACK_B,
                                           config::RAW_INTERNAL_SLACK_B, config::RAW_INTERNAL_MARGIN_B);
}

}  // namespace

TEST(BeetleV2RawFloor, ItsImageRunsTheCheck)
{
    // the PX1105R image: the raw filter its one navigation filter, the check in its start
    EXPECT_EQ(TR_RAW_GNSS_PRIMARY, 1);
    EXPECT_EQ(TR_RAW_NAV_PX1105R, 1);
    EXPECT_EQ(TR_RAW_NAV_P4, 1);
    EXPECT_TRUE(config::RAW_INTERNAL_PROBE) << "the S3 probes; no largest-block floor";
}

TEST(BeetleV2RawFloor, TheStacksAndTheMargin)
{
    EXPECT_EQ(config::RAW_FILTER_STACK_B, 8192u);           // "Raw GNSS"
    EXPECT_EQ(config::RAW_MEAS_STACK_B, 6144u);             // "GNSS meas"
    EXPECT_EQ(config::FLIGHT_TASK_STACK_B, 16u * 1024u);    // "flight"
    EXPECT_EQ(config::RAW_INTERNAL_SLACK_B, 1024u);
    EXPECT_EQ(config::RAW_INTERNAL_MARGIN_B, 8u * 1024u);   // a stand-in until a Beetle V2 has booted
}

TEST(BeetleV2RawFloor, TheFlightTaskFindsItsBlockOrTheFilterDoesNotStart)
{
    const size_t block = config::FLIGHT_TASK_STACK_B + config::RAW_INTERNAL_SLACK_B;   // 17 KB
    const size_t total = block + config::RAW_INTERNAL_MARGIN_B;                         // 25 KB
    EXPECT_TRUE(room(block, total));
    EXPECT_FALSE(room(block - 1, total + 50000)) << "no block for it, however much is free";
    EXPECT_FALSE(room(block, total - 1)) << "its block, but no margin beside it";
    EXPECT_FALSE(room(block + 10000, total + 10000, false)) << "the probes were not both had";
    // the Beetle bench image of 2026-10-26 (de834d52): after a 12 KB stack, 22,408 B free and an 11,264 B largest
    // block, where its flight task was never made: not started here
    EXPECT_FALSE(room(11264, 22408));
    // a Beetle V1 as flown (2026-10-22: 37,852 B left after its filter's stacks): started, if a block held 17 KB
    EXPECT_TRUE(room(20000, 37852));
}
