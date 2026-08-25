// Host-side test for the OC relay queue's dedupe key (#837 item 11).
//
// The queue collapses a re-pushed command onto the one already queued so a
// self-applying settings slider cannot flood it. That is right when the
// payload IS the command, and wrong for PYRO_CONT_TEST / PYRO_FIRE_TEST,
// whose payload is a channel byte: keying on the command id alone rewrote a
// queued test's channel and silently destroyed the earlier request.
//
// These tests pin both halves of the rule — different channels stay separate,
// the same channel still collapses.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "cmd_queue_dedupe_policy.h"

namespace
{

// A stand-in for QueuedCommand carrying only what the dedupe key reads.
struct Entry
{
    uint8_t cmd;
    uint8_t sel;   // first payload byte; 0 when there is no payload
};

// Mirrors the enqueue path in setPendingCommandWithConfig(): scan for a
// same-operation entry and overwrite it in place, otherwise append. Returns
// the resulting queue so a test can assert what survived.
std::vector<Entry> enqueueAll(const std::vector<Entry>& requests)
{
    std::vector<Entry> q;
    for (const Entry& e : requests)
    {
        bool merged = false;
        for (Entry& slot : q)
        {
            if (cmdQueueSameOperation(slot.cmd, slot.sel, e.cmd, e.sel))
            {
                slot = e;
                merged = true;
                break;
            }
        }
        if (!merged) q.push_back(e);
    }
    return q;
}

// A representative non-pyro command: the payload is a whole config struct, so
// latest-wins on the command id is the intended behaviour.
constexpr uint8_t kServoConfig = SERVO_CONFIG_PENDING;

}  // namespace

// ---------------------------------------------------------------------------
// The rule itself
// ---------------------------------------------------------------------------

TEST(CmdQueueDedupePolicy, DifferentCommandIdsAreNeverTheSameOperation)
{
    EXPECT_FALSE(cmdQueueSameOperation(kServoConfig, 0, PYRO_CONT_TEST, 1));
    EXPECT_FALSE(cmdQueueSameOperation(PYRO_CONT_TEST, 1, PYRO_FIRE_TEST, 1));
}

TEST(CmdQueueDedupePolicy, NonSelectorCommandsMatchOnIdAlone)
{
    // The payload is the command; a re-push must replace it whatever the
    // bytes happen to be. This is the flood protection and it must not regress.
    EXPECT_TRUE(cmdQueueSameOperation(kServoConfig, 0x11, kServoConfig, 0x77));
}

TEST(CmdQueueDedupePolicy, PyroTestsOnDifferentChannelsAreDistinctOperations)
{
    // The defect: these merged, and channel 1's test ceased to exist.
    EXPECT_FALSE(cmdQueueSameOperation(PYRO_CONT_TEST, 1, PYRO_CONT_TEST, 2));
    EXPECT_FALSE(cmdQueueSameOperation(PYRO_FIRE_TEST, 3, PYRO_FIRE_TEST, 4));
}

TEST(CmdQueueDedupePolicy, PyroTestsOnTheSameChannelStillCollapse)
{
    // Repeat-tapping one channel must not consume extra slots.
    EXPECT_TRUE(cmdQueueSameOperation(PYRO_CONT_TEST, 2, PYRO_CONT_TEST, 2));
    EXPECT_TRUE(cmdQueueSameOperation(PYRO_FIRE_TEST, 4, PYRO_FIRE_TEST, 4));
}

TEST(CmdQueueDedupePolicy, OnlyThePyroTestsAreSelectorKeyed)
{
    EXPECT_TRUE(cmdQueueKeyIncludesSelector(PYRO_CONT_TEST));
    EXPECT_TRUE(cmdQueueKeyIncludesSelector(PYRO_FIRE_TEST));
    EXPECT_FALSE(cmdQueueKeyIncludesSelector(kServoConfig));
    EXPECT_FALSE(cmdQueueKeyIncludesSelector(SERVO_TEST_PENDING));
    EXPECT_FALSE(cmdQueueKeyIncludesSelector(PYRO_CONFIG_PENDING));
}

// ---------------------------------------------------------------------------
// The operator-visible behaviour it exists to protect
// ---------------------------------------------------------------------------

TEST(CmdQueueDedupePolicy, CheckingAllFourContinuityChannelsTestsAllFour)
{
    // The pad workflow: tap each channel's Test Continuity button in turn,
    // faster than the ~1 s a served command sits at the queue head. Before the
    // fix this left a single entry holding channel 4, and channels 1-3 showed
    // "TESTING" (#411) for the full window before reverting to a stale value.
    const std::vector<Entry> q = enqueueAll({
        {PYRO_CONT_TEST, 1},
        {PYRO_CONT_TEST, 2},
        {PYRO_CONT_TEST, 3},
        {PYRO_CONT_TEST, 4},
    });

    ASSERT_EQ(q.size(), 4u);
    for (uint8_t ch = 1; ch <= 4; ch++)
    {
        EXPECT_EQ(q[ch - 1].cmd, PYRO_CONT_TEST);
        EXPECT_EQ(q[ch - 1].sel, ch) << "channel " << (int)ch << " was swallowed";
    }
}

TEST(CmdQueueDedupePolicy, TwoTestFiresOnDifferentChannelsBothSurvive)
{
    // The same collapse applied to PYRO_FIRE_TEST, where it produced exactly
    // one fire — on the second channel — for two distinct requests.
    const std::vector<Entry> q = enqueueAll({
        {PYRO_FIRE_TEST, 1},
        {PYRO_FIRE_TEST, 2},
    });

    ASSERT_EQ(q.size(), 2u);
    EXPECT_EQ(q[0].sel, 1);
    EXPECT_EQ(q[1].sel, 2);
}

TEST(CmdQueueDedupePolicy, RepeatedTapsOnOneChannelStayBounded)
{
    const std::vector<Entry> q = enqueueAll({
        {PYRO_CONT_TEST, 2},
        {PYRO_CONT_TEST, 2},
        {PYRO_CONT_TEST, 2},
    });

    ASSERT_EQ(q.size(), 1u);
    EXPECT_EQ(q[0].sel, 2);
}

TEST(CmdQueueDedupePolicy, ASettingsBurstStillCollapsesToOneSlot)
{
    // Regression guard on the reason the dedupe exists: a slider drag must not
    // occupy more than one slot, and the final value must win.
    const std::vector<Entry> q = enqueueAll({
        {kServoConfig, 10},
        {kServoConfig, 20},
        {kServoConfig, 30},
    });

    ASSERT_EQ(q.size(), 1u);
    EXPECT_EQ(q[0].sel, 30);
}

TEST(CmdQueueDedupePolicy, PyroTestsDoNotDisturbAQueuedSettingsBurst)
{
    // Mixed traffic: a profile sync draining while the operator checks
    // continuity. Each config command keeps its single slot; each channel
    // gets its own.
    const std::vector<Entry> q = enqueueAll({
        {kServoConfig, 10},
        {PYRO_CONT_TEST, 1},
        {kServoConfig, 20},
        {PYRO_CONT_TEST, 2},
        {SERVO_TEST_PENDING, 5},
    });

    ASSERT_EQ(q.size(), 4u);
    EXPECT_EQ(q[0].cmd, kServoConfig);
    EXPECT_EQ(q[0].sel, 20);          // latest wins, position preserved
    EXPECT_EQ(q[1].cmd, PYRO_CONT_TEST);
    EXPECT_EQ(q[1].sel, 1);
    EXPECT_EQ(q[2].cmd, PYRO_CONT_TEST);
    EXPECT_EQ(q[2].sel, 2);
    EXPECT_EQ(q[3].cmd, SERVO_TEST_PENDING);
}
