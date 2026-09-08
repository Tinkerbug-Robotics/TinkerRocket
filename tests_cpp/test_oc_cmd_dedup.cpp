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

// ── #1137 item 4: the idle poll must stop being load bearing ──
//
// The OC delivers every command CMD_REPEAT_LIMIT (3) times and the idle
// (cmd=0) poll that separates two commands EXACTLY ONCE, while the FC cleared
// its dedup key only on observing that one idle. So the command half of the
// protocol was triple-redundant and the window-separator half was single-shot,
// and only the fragile half could wedge the channel: one failed read of that
// poll and the next command was discarded as a duplicate whenever it happened
// to carry the same id.
//
// The OC now stamps each serving window with an epoch (status payload byte 2,
// never 0), so a new window is identifiable whether or not the gap was seen.

TEST(OcCmdDedup, LostIdlePollNoLongerSwallowsTheNextIdenticalCommand) {
    OcCmdDedup d;
    // Window 1: command 12, delivered three times.
    EXPECT_EQ(d.take(true, 12, 1), 12);
    EXPECT_EQ(d.take(true, 12, 1), 0);
    EXPECT_EQ(d.take(true, 12, 1), 0);
    // The single idle poll is LOST — the read failed, so the FC never sees it.
    // Window 2 is the same command id, and must still execute.
    EXPECT_EQ(d.take(true, 12, 2), 12);
}

TEST(OcCmdDedup, LostIdlePollWithoutAnEpochStillSwallowsIt) {
    // The pre-fix behaviour, kept reachable for an OC that predates the byte.
    // Documented rather than desired: it is why the epoch exists.
    OcCmdDedup d;
    EXPECT_EQ(d.take(true, 12), 12);
    EXPECT_EQ(d.take(true, 12), 0);
    EXPECT_EQ(d.take(true, 12), 0);
    EXPECT_EQ(d.take(true, 12), 0) << "no epoch: the id-only rule still applies";
}

TEST(OcCmdDedup, RepeatsWithinAWindowAreStillDeduped) {
    // The epoch must not turn the repeat deliveries into three executions --
    // that is the whole reason the dedup exists.
    OcCmdDedup d;
    int executed = 0;
    for (int i = 0; i < 3; ++i) if (d.take(true, 40, 7) != 0) executed++;
    EXPECT_EQ(executed, 1);
}

TEST(OcCmdDedup, TheIdlePollStillResetsWhenItIsSeen) {
    OcCmdDedup d;
    EXPECT_EQ(d.take(true, 12, 1), 12);
    EXPECT_EQ(d.take(true, 0, 0), 0);      // idle, seen
    EXPECT_EQ(d.take(true, 12, 2), 12);    // next window executes
}

TEST(OcCmdDedup, DifferentCommandsInBackToBackWindowsBothExecute) {
    // The case that always worked (different ids), pinned so the epoch change
    // cannot regress it.
    OcCmdDedup d;
    EXPECT_EQ(d.take(true, 12, 1), 12);
    EXPECT_EQ(d.take(true, 13, 2), 13);
    EXPECT_EQ(d.take(true, 13, 2), 0);
}

TEST(OcCmdDedup, EpochWrapsPastZeroWithoutCollidingWithIdle) {
    // The OC's counter skips 0 on wrap because 0 means "idle / no window".
    // Two consecutive windows numbered 255 then 1 must both execute.
    OcCmdDedup d;
    EXPECT_EQ(d.take(true, 12, 255), 12);
    EXPECT_EQ(d.take(true, 12, 255), 0);
    EXPECT_EQ(d.take(true, 12, 1), 12);
}

TEST(OcCmdDedup, RetryBudgetStillWorksUnderEpochDedup) {
    // armRetry() re-arms by clearing last_processed_cmd; under the epoch rule
    // that has to keep meaning "dispatch this again on the next poll", or the
    // #1112 config-retry path silently stops working.
    OcCmdDedup d;
    ASSERT_EQ(d.take(true, 12, 4), 12);
    ASSERT_TRUE(d.armRetry());
    EXPECT_EQ(d.take(true, 12, 4), 12) << "same window, but a retry was armed";
}

TEST(OcCmdDedup, RetryBudgetIsStillBoundedUnderEpochDedup) {
    OcCmdDedup d;
    ASSERT_EQ(d.take(true, 12, 4), 12);
    int granted = 0;
    for (int i = 0; i < 10; ++i) {
        if (!d.armRetry()) break;
        granted++;
        (void)d.take(true, 12, 4);
    }
    EXPECT_EQ(granted, OcCmdDedup::CFG_RETRY_LIMIT);
    EXPECT_EQ(d.take(true, 12, 4), 0) << "budget spent: no further dispatch";
}

TEST(OcCmdDedup, NoDispatchWithoutAPollEvenWithAFreshEpoch) {
    OcCmdDedup d;
    EXPECT_EQ(d.take(false, 12, 9), 0);
}
