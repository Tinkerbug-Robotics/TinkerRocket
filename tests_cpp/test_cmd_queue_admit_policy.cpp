// Host tests for the OC relay queue's admission rule (#1116).
//
// The queue drops on overflow. For a config sync that is a lost setting the
// app can re-send; for OTA_FINISH_CMD / OTA_ABORT_CMD it used to strand the FC
// in slave-RX image mode, because those two are the FC's only way out and
// nothing re-stages them. Two slots are now held back for them.

#include <gtest/gtest.h>
#include "cmd_queue_admit_policy.h"

namespace {
constexpr size_t kDepth = 20;   // CMD_QUEUE_DEPTH in out_computer/main.cpp
constexpr uint8_t kOrdinary = SERVO_CONFIG_PENDING;
}

TEST(CmdQueueAdmitPolicy, ExactlyTheTwoSessionEndingCommands)
{
    EXPECT_TRUE(cmdEndsOtaSession(OTA_FINISH_CMD));
    EXPECT_TRUE(cmdEndsOtaSession(OTA_ABORT_CMD));
    // The begin opens a session; it competes for slots like any config push.
    EXPECT_FALSE(cmdEndsOtaSession(OTA_BEGIN_PENDING));
    EXPECT_FALSE(cmdEndsOtaSession(SERVO_CONFIG_PENDING));
    // The front-inserted pyro tests are not in the reserve either: they have
    // their own priority path, and they do not end anything.
    EXPECT_FALSE(cmdEndsOtaSession(PYRO_FIRE_TEST));
    EXPECT_FALSE(cmdEndsOtaSession(PYRO_CONT_TEST));
}

TEST(CmdQueueAdmitPolicy, OrdinaryCommandsStopShortOfTheReserve)
{
    for (size_t count = 0; count + kCmdQueueReservedForOtaTeardown < kDepth; count++)
        EXPECT_TRUE(cmdQueueAdmits(kOrdinary, count, kDepth)) << "count=" << count;
    EXPECT_FALSE(cmdQueueAdmits(kOrdinary, kDepth - kCmdQueueReservedForOtaTeardown, kDepth));
    EXPECT_FALSE(cmdQueueAdmits(kOrdinary, kDepth - 1, kDepth));
    EXPECT_FALSE(cmdQueueAdmits(kOrdinary, kDepth, kDepth));
}

TEST(CmdQueueAdmitPolicy, SessionEndingCommandsMayUseTheReserve)
{
    for (size_t count = 0; count < kDepth; count++)
    {
        EXPECT_TRUE(cmdQueueAdmits(OTA_ABORT_CMD,  count, kDepth)) << "count=" << count;
        EXPECT_TRUE(cmdQueueAdmits(OTA_FINISH_CMD, count, kDepth)) << "count=" << count;
    }
}

TEST(CmdQueueAdmitPolicy, NothingIsAdmittedToAPhysicallyFullQueue)
{
    EXPECT_FALSE(cmdQueueAdmits(OTA_ABORT_CMD,  kDepth, kDepth));
    EXPECT_FALSE(cmdQueueAdmits(OTA_FINISH_CMD, kDepth, kDepth));
    EXPECT_FALSE(cmdQueueAdmits(kOrdinary,      kDepth, kDepth));
    EXPECT_FALSE(cmdQueueAdmits(OTA_ABORT_CMD,  kDepth + 5, kDepth));   // never index past the ring
}

TEST(CmdQueueAdmitPolicy, TheScenarioFromTheIssue)
{
    // A profile sync (13 commands) plus the OTA_BEGIN and a few config pushes
    // has filled the ordinary slots; the OC's stall watchdog now stages the
    // abort. Before: dropped with a log line, FC stranded. Now: admitted.
    size_t count = 0;
    while (cmdQueueAdmits(kOrdinary, count, kDepth)) count++;
    EXPECT_EQ(count, kDepth - kCmdQueueReservedForOtaTeardown);
    EXPECT_TRUE(cmdQueueAdmits(OTA_ABORT_CMD, count, kDepth));
    count++;
    // A FINISH that was already queued when the app disconnected, plus the
    // abort the disconnect stages: both fit. (The dedupe upstream collapses a
    // repeat of either onto its queued copy, so two is the most ever needed.)
    EXPECT_TRUE(cmdQueueAdmits(OTA_FINISH_CMD, count, kDepth));
}

TEST(CmdQueueAdmitPolicy, TheReserveLeavesRoomForADocumentedProfileSync)
{
    // out_computer/main.cpp's own worked example is a 13-command profile
    // sync; with the OTA_BEGIN behind it that is 14 ordinary entries, which
    // must still fit without a drop.
    EXPECT_GE(kDepth - kCmdQueueReservedForOtaTeardown, 14u);
    EXPECT_LT(kCmdQueueReservedForOtaTeardown, kDepth);
}
