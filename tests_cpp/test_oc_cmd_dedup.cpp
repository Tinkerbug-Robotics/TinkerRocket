// Host-side test for the FC's OC-command dedup key and config-retry budget
// (#1112).
//
// The OC serves each command for CMD_REPEAT_LIMIT polls and the FC mirrors
// the served byte on every successful poll read; the dedup key executes each
// command once per window and clears when the OC reports 0. A config handler
// that finds no config frame clears the key to "retry on next poll" — but the
// dispatch ran on every loop pass, not only the pass that polled, so the
// cleared key re-fired ~1 ms later against the same stale mirror and burned
// ~38 ms of readConfigFrame() per pass: a ~26 Hz retry storm in place of the
// 1 kHz flight loop, latched on for the whole flight once a real INFLIGHT
// stopped the poll. The fix gates the dispatch on the poll pass and gives
// each served command a retry budget.
//
// Each test drives the dedup the way loop_fc() does: take(polled, mirror)
// once per pass, armRetry() from inside a config handler that found nothing.

#include <gtest/gtest.h>

#include <cstdint>

#include "RocketComputerTypes.h"
#include "oc_cmd_dedup.h"

namespace
{

// A poll pass: the mirror carries `served`, which the OC is serving.
uint8_t pollPass(OcCmdDedup& d, uint8_t served) { return d.take(true, served); }

// A pass between polls: the mirror is whatever the last poll left in it.
uint8_t idlePass(OcCmdDedup& d, uint8_t mirror) { return d.take(false, mirror); }

TEST(OcCmdDedup, ClearedKeyDoesNotFireBetweenPolls)
{
    // The issue's storm: the frame was not in the read, the handler re-armed,
    // and the ~250 passes until the next poll must all be no-ops.
    OcCmdDedup d;
    EXPECT_EQ(pollPass(d, SERVO_CONFIG_PENDING), SERVO_CONFIG_PENDING);
    EXPECT_TRUE(d.armRetry());
    for (int pass = 0; pass < 250; ++pass)
    {
        ASSERT_EQ(idlePass(d, SERVO_CONFIG_PENDING), 0U) << "pass " << pass;
    }
    // The OC's second delivery re-stages the frame: this is the retry.
    EXPECT_EQ(pollPass(d, SERVO_CONFIG_PENDING), SERVO_CONFIG_PENDING);
}

TEST(OcCmdDedup, FrozenMirrorInFlightNeverDispatches)
{
    // Launch detect fired inside the retry window: a real INFLIGHT skips the
    // poll, so the mirror is frozen non-zero with the key cleared. Before the
    // fix that re-ran the handler on every pass for the whole flight.
    OcCmdDedup d;
    EXPECT_EQ(pollPass(d, PID_CONFIG_PENDING), PID_CONFIG_PENDING);
    EXPECT_TRUE(d.armRetry());
    for (int pass = 0; pass < 100000; ++pass)
    {
        ASSERT_EQ(idlePass(d, PID_CONFIG_PENDING), 0U) << "pass " << pass;
    }
}

TEST(OcCmdDedup, RepeatDeliveriesExecuteOnce)
{
    // The OC serves the command three times, then idles: one execution, and
    // the idle poll frees the same command to be issued again later.
    OcCmdDedup d;
    EXPECT_EQ(pollPass(d, CAMERA_START), CAMERA_START);
    EXPECT_EQ(pollPass(d, CAMERA_START), 0U);
    EXPECT_EQ(pollPass(d, CAMERA_START), 0U);
    EXPECT_EQ(pollPass(d, 0), 0U);
    EXPECT_EQ(pollPass(d, CAMERA_START), CAMERA_START);
}

TEST(OcCmdDedup, RetrySucceedsOnASecondDelivery)
{
    // The normal life of a retry: the frame arrives intact on delivery 2, the
    // handler applies it and does not re-arm, deliveries 3 and beyond dedup.
    OcCmdDedup d;
    EXPECT_EQ(pollPass(d, SERVO_CONFIG_PENDING), SERVO_CONFIG_PENDING);
    EXPECT_TRUE(d.armRetry());
    EXPECT_EQ(pollPass(d, SERVO_CONFIG_PENDING), SERVO_CONFIG_PENDING);
    EXPECT_EQ(pollPass(d, SERVO_CONFIG_PENDING), 0U);
    EXPECT_EQ(pollPass(d, 0), 0U);
}

TEST(OcCmdDedup, StaleMirrorAfterAFailedReadRetriesOncePerPoll)
{
    // A poll whose read failed leaves the mirror stale but still polled: the
    // retry runs once on that pass — never more than once per poll.
    OcCmdDedup d;
    EXPECT_EQ(pollPass(d, SIM_CONFIG_PENDING), SIM_CONFIG_PENDING);
    EXPECT_TRUE(d.armRetry());
    EXPECT_EQ(idlePass(d, SIM_CONFIG_PENDING), 0U);
    EXPECT_EQ(pollPass(d, SIM_CONFIG_PENDING), SIM_CONFIG_PENDING);   // read failed, mirror stale
    EXPECT_EQ(idlePass(d, SIM_CONFIG_PENDING), 0U);
}

TEST(OcCmdDedup, RetryBudgetIsSpentThenTheCommandIsHeldUntilIdle)
{
    // The OC dropped the frame for size (#569) or stopped answering with the
    // mirror frozen non-zero: after CFG_RETRY_LIMIT retries the key keeps the
    // command, so no further poll re-runs the handler until the OC reports 0.
    OcCmdDedup d;
    EXPECT_EQ(pollPass(d, ROLL_PROFILE_PENDING), ROLL_PROFILE_PENDING);
    for (uint8_t i = 0; i < OcCmdDedup::CFG_RETRY_LIMIT; ++i)
    {
        ASSERT_TRUE(d.armRetry()) << "retry " << (unsigned)i;
        ASSERT_EQ(pollPass(d, ROLL_PROFILE_PENDING), ROLL_PROFILE_PENDING) << "retry " << (unsigned)i;
    }
    EXPECT_FALSE(d.armRetry());
    for (int poll = 0; poll < 100; ++poll)
    {
        ASSERT_EQ(pollPass(d, ROLL_PROFILE_PENDING), 0U) << "poll " << poll;
    }
    // The OC clears its slot: the budget is fresh for the next issue.
    EXPECT_EQ(pollPass(d, 0), 0U);
    EXPECT_EQ(pollPass(d, ROLL_PROFILE_PENDING), ROLL_PROFILE_PENDING);
    EXPECT_TRUE(d.armRetry());
}

TEST(OcCmdDedup, BudgetBelongsToOneCommand)
{
    // The idle poll between two commands was lost to a read failure, so B
    // follows A's spent budget with no reset in between: B gets its own.
    OcCmdDedup d;
    EXPECT_EQ(pollPass(d, SERVO_CONFIG_PENDING), SERVO_CONFIG_PENDING);
    for (uint8_t i = 0; i < OcCmdDedup::CFG_RETRY_LIMIT; ++i)
    {
        ASSERT_TRUE(d.armRetry());
        ASSERT_EQ(pollPass(d, SERVO_CONFIG_PENDING), SERVO_CONFIG_PENDING);
    }
    EXPECT_FALSE(d.armRetry());
    EXPECT_EQ(pollPass(d, PID_CONFIG_PENDING), PID_CONFIG_PENDING);
    for (uint8_t i = 0; i < OcCmdDedup::CFG_RETRY_LIMIT; ++i)
    {
        ASSERT_TRUE(d.armRetry()) << "retry " << (unsigned)i;
        ASSERT_EQ(pollPass(d, PID_CONFIG_PENDING), PID_CONFIG_PENDING);
    }
    EXPECT_FALSE(d.armRetry());
}

TEST(OcCmdDedup, NothingDispatchesWithoutAPoll)
{
    // A fresh boot whose first passes precede the first poll (or a real
    // flight): whatever the mirror holds, no poll means no dispatch.
    OcCmdDedup d;
    EXPECT_EQ(idlePass(d, PYRO_FIRE_TEST), 0U);
    EXPECT_EQ(idlePass(d, SERVO_CONFIG_PENDING), 0U);
    EXPECT_EQ(idlePass(d, 0), 0U);
    // ...and the first poll then dispatches normally.
    EXPECT_EQ(pollPass(d, SERVO_CONFIG_PENDING), SERVO_CONFIG_PENDING);
}

TEST(OcCmdDedup, IdlePollResetsKeyAndBudget)
{
    OcCmdDedup d;
    EXPECT_EQ(pollPass(d, SERVO_TEST_PENDING), SERVO_TEST_PENDING);
    EXPECT_TRUE(d.armRetry());
    EXPECT_EQ(pollPass(d, 0), 0U);
    EXPECT_EQ(d.last_processed_cmd, 0U);
    EXPECT_EQ(d.retries, 0U);
    EXPECT_EQ(d.retry_cmd, 0U);
}

}  // namespace
