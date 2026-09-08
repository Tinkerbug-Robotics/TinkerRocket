// Host-side test for the OC relay queue's FC-session retirement (#1105).
//
// The queue and its serving slot outlive the FC. An FC reset inside a
// command's repeat window freezes the slot on that command, and a one-shot
// still waiting behind another command's window is the same hazard one step
// earlier. When the FC reports a boot, one-shot actuating commands are
// retired from both; everything else (a config sync queued across power-on)
// keeps its place and order.
//
// The ring compaction is the policy, so these drive the real template with a
// stand-in QueuedCommand, including the wrap-around cases.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "cmd_queue_session_policy.h"

namespace
{

// Stand-in for main.cpp's QueuedCommand: the members the policy reads.
struct Entry
{
    uint8_t cmd;
    uint8_t cfg_type;
    uint8_t cfg_len;
    uint8_t cfg[4];
};

constexpr size_t kDepth = 8;

Entry cmdOnly(uint8_t cmd)
{
    return Entry{cmd, 0, 0, {0, 0, 0, 0}};
}

Entry withChannel(uint8_t cmd, uint8_t ch)
{
    return Entry{cmd, cmd, 1, {ch, 0, 0, 0}};
}

// Fill ring[head..] FIFO-style, wrapping, the way setPendingCommandWithConfig
// appends. Returns count.
size_t load(Entry (&ring)[kDepth], size_t head, const std::vector<Entry>& entries)
{
    for (size_t i = 0; i < entries.size(); i++)
    {
        ring[(head + i) % kDepth] = entries[i];
    }
    return entries.size();
}

std::vector<uint8_t> cmdsInOrder(const Entry (&ring)[kDepth], size_t head, size_t count)
{
    std::vector<uint8_t> out;
    for (size_t i = 0; i < count; i++)
    {
        out.push_back(ring[(head + i) % kDepth].cmd);
    }
    return out;
}

TEST(CmdQueueSessionPolicy, ServingSlotRule)
{
    EXPECT_TRUE(cmdQueueRetireServingOnFcBoot(PYRO_FIRE_TEST));
    EXPECT_TRUE(cmdQueueRetireServingOnFcBoot(PYRO_CONT_TEST));
    EXPECT_TRUE(cmdQueueRetireServingOnFcBoot(SERVO_TEST_PENDING));
    EXPECT_FALSE(cmdQueueRetireServingOnFcBoot(0));                     // idle slot
    EXPECT_FALSE(cmdQueueRetireServingOnFcBoot(SERVO_CONFIG_PENDING));  // a sync in flight
    EXPECT_FALSE(cmdQueueRetireServingOnFcBoot(CAMERA_START));
}

TEST(CmdQueueSessionPolicy, EmptyQueueIsANoop)
{
    Entry ring[kDepth] = {};
    size_t count = 0;
    CmdQueueRetired out[kDepth];
    EXPECT_EQ(cmdQueueRetireOneShots(ring, 0, count, out, kDepth), 0u);
    EXPECT_EQ(count, 0u);
}

TEST(CmdQueueSessionPolicy, ConfigSyncIsUntouched)
{
    // The connect-time profile sync must drain across the boundary as before.
    Entry ring[kDepth] = {};
    size_t count = load(ring, 0, {cmdOnly(SERVO_CONFIG_PENDING), cmdOnly(PID_CONFIG_PENDING),
                                  cmdOnly(ROLL_PROFILE_PENDING), cmdOnly(CAMERA_START)});
    CmdQueueRetired out[kDepth];
    EXPECT_EQ(cmdQueueRetireOneShots(ring, 0, count, out, kDepth), 0u);
    EXPECT_EQ(count, 4u);
    EXPECT_EQ(cmdsInOrder(ring, 0, count),
              (std::vector<uint8_t>{SERVO_CONFIG_PENDING, PID_CONFIG_PENDING,
                                    ROLL_PROFILE_PENDING, CAMERA_START}));
}

TEST(CmdQueueSessionPolicy, FireTestAtTheFrontIsRetiredAndTheSyncKeepsItsOrder)
{
    // The pyro tests front-queue; a fire tapped just before the FC reset sits
    // ahead of a sync that must survive.
    Entry ring[kDepth] = {};
    size_t count = load(ring, 0, {withChannel(PYRO_FIRE_TEST, 2), cmdOnly(SERVO_CONFIG_PENDING),
                                  cmdOnly(PID_CONFIG_PENDING)});
    CmdQueueRetired out[kDepth];
    EXPECT_EQ(cmdQueueRetireOneShots(ring, 0, count, out, kDepth), 1u);
    EXPECT_EQ(count, 2u);
    EXPECT_EQ(cmdsInOrder(ring, 0, count),
              (std::vector<uint8_t>{SERVO_CONFIG_PENDING, PID_CONFIG_PENDING}));
    EXPECT_EQ(out[0].cmd, PYRO_FIRE_TEST);
    EXPECT_EQ(out[0].sel, 2u);
}

TEST(CmdQueueSessionPolicy, InterleavedOneShotsAreRetiredInOrder)
{
    Entry ring[kDepth] = {};
    size_t count = load(ring, 0, {cmdOnly(SERVO_CONFIG_PENDING), withChannel(PYRO_CONT_TEST, 1),
                                  cmdOnly(PID_CONFIG_PENDING), withChannel(PYRO_FIRE_TEST, 3),
                                  cmdOnly(SERVO_TEST_PENDING), cmdOnly(CAMERA_STOP)});
    CmdQueueRetired out[kDepth];
    EXPECT_EQ(cmdQueueRetireOneShots(ring, 0, count, out, kDepth), 3u);
    EXPECT_EQ(count, 3u);
    EXPECT_EQ(cmdsInOrder(ring, 0, count),
              (std::vector<uint8_t>{SERVO_CONFIG_PENDING, PID_CONFIG_PENDING, CAMERA_STOP}));
    EXPECT_EQ(out[0].cmd, PYRO_CONT_TEST);
    EXPECT_EQ(out[0].sel, 1u);
    EXPECT_EQ(out[1].cmd, PYRO_FIRE_TEST);
    EXPECT_EQ(out[1].sel, 3u);
    EXPECT_EQ(out[2].cmd, SERVO_TEST_PENDING);
    EXPECT_EQ(out[2].sel, 0u) << "no payload -> selector 0";
}

TEST(CmdQueueSessionPolicy, WrapAroundCompactsInPlace)
{
    // head near the end of the ring so the live entries straddle the wrap.
    Entry ring[kDepth] = {};
    const size_t head = kDepth - 2;   // slots 6, 7, 0, 1, 2
    size_t count = load(ring, head, {withChannel(PYRO_FIRE_TEST, 4), cmdOnly(SERVO_CONFIG_PENDING),
                                     cmdOnly(GROUND_TEST_START), cmdOnly(PID_CONFIG_PENDING),
                                     cmdOnly(SIM_START_CMD)});
    CmdQueueRetired out[kDepth];
    EXPECT_EQ(cmdQueueRetireOneShots(ring, head, count, out, kDepth), 3u);
    EXPECT_EQ(count, 2u);
    // Survivors are contiguous from the SAME head, across the wrap.
    EXPECT_EQ(cmdsInOrder(ring, head, count),
              (std::vector<uint8_t>{SERVO_CONFIG_PENDING, PID_CONFIG_PENDING}));
    EXPECT_EQ(ring[head].cmd, SERVO_CONFIG_PENDING);
    EXPECT_EQ(ring[(head + 1) % kDepth].cmd, PID_CONFIG_PENDING);
}

TEST(CmdQueueSessionPolicy, AllOneShotsLeavesTheQueueEmpty)
{
    Entry ring[kDepth] = {};
    size_t count = load(ring, 3, {withChannel(PYRO_CONT_TEST, 1), withChannel(PYRO_CONT_TEST, 2),
                                  withChannel(PYRO_CONT_TEST, 3), withChannel(PYRO_CONT_TEST, 4)});
    CmdQueueRetired out[kDepth];
    EXPECT_EQ(cmdQueueRetireOneShots(ring, 3, count, out, kDepth), 4u);
    EXPECT_EQ(count, 0u);
    for (size_t i = 0; i < 4; i++)
    {
        EXPECT_EQ(out[i].cmd, PYRO_CONT_TEST);
        EXPECT_EQ(out[i].sel, i + 1);
    }
}

TEST(CmdQueueSessionPolicy, RecordCapIsHonouredButTheCountIsNot)
{
    Entry ring[kDepth] = {};
    size_t count = load(ring, 0, {withChannel(PYRO_FIRE_TEST, 1), withChannel(PYRO_FIRE_TEST, 2),
                                  withChannel(PYRO_FIRE_TEST, 3)});
    CmdQueueRetired out[1] = {{0xFF, 0xFF}};
    EXPECT_EQ(cmdQueueRetireOneShots(ring, 0, count, out, 1), 3u);
    EXPECT_EQ(count, 0u);
    EXPECT_EQ(out[0].sel, 1u) << "only the first fits";
    // A null record buffer is allowed.
    count = load(ring, 0, {withChannel(PYRO_FIRE_TEST, 1), cmdOnly(SERVO_CONFIG_PENDING)});
    EXPECT_EQ(cmdQueueRetireOneShots(ring, 0, count, nullptr, 0), 1u);
    EXPECT_EQ(count, 1u);
    EXPECT_EQ(ring[0].cmd, SERVO_CONFIG_PENDING);
}

}  // namespace

// ---------------------------------------------------------------------------
// #1149 item 1 — priority commands must stay FIFO among THEMSELVES.
//
// PYRO_CONT_TEST / PYRO_FIRE_TEST jump the queue so they precede the app's
// ~13-command profile sync. The old insert did an unconditional push-front,
// which made them a STACK: two tests enqueued before the first was served were
// delivered newest-first, reversing the operator's channel order and letting a
// FIRE overtake a CONT test they tapped earlier. Since #837 item 11 the dedupe
// key includes the channel byte, so per-channel tests each take a slot instead
// of collapsing into one — which is what made the reversal reachable.
//
// This models the ring arithmetic the OC uses, so the wrap cases are covered.
// ---------------------------------------------------------------------------

namespace {

constexpr size_t kFrontDepth = 20;

struct MiniEntry { uint8_t cmd = 0; uint8_t tag = 0; };

struct MiniQueue {
    MiniEntry ring[kFrontDepth];
    size_t head = 0;
    size_t count = 0;

    void push(uint8_t cmd, uint8_t tag) {
        MiniEntry e{cmd, tag};
        if (cmdIsFrontPriority(cmd)) {
            size_t front_run = 0;
            while (front_run < count && cmdIsFrontPriority(ring[(head + front_run) % kFrontDepth].cmd)) {
                front_run++;
            }
            for (size_t i = count; i > front_run; --i) {
                ring[(head + i) % kFrontDepth] = ring[(head + i - 1) % kFrontDepth];
            }
            ring[(head + front_run) % kFrontDepth] = e;
        } else {
            ring[(head + count) % kFrontDepth] = e;
        }
        count++;
    }
    MiniEntry pop() {
        MiniEntry e = ring[head];
        head = (head + 1) % kFrontDepth;
        count--;
        return e;
    }
};

}  // namespace

TEST(CmdQueueFrontOrder, PriorityCommandsKeepTheOperatorsOrder) {
    MiniQueue q;
    q.push(PYRO_CONT_TEST, 1);   // operator taps channel 1 first
    q.push(PYRO_CONT_TEST, 2);
    q.push(PYRO_CONT_TEST, 3);
    EXPECT_EQ(q.pop().tag, 1) << "the first tap must be delivered first";
    EXPECT_EQ(q.pop().tag, 2);
    EXPECT_EQ(q.pop().tag, 3);
}

TEST(CmdQueueFrontOrder, AFireDoesNotOvertakeAnEarlierContTest) {
    MiniQueue q;
    q.push(PYRO_CONT_TEST, 10);
    q.push(PYRO_FIRE_TEST, 11);
    EXPECT_EQ(q.pop().cmd, PYRO_CONT_TEST);
    EXPECT_EQ(q.pop().cmd, PYRO_FIRE_TEST);
}

TEST(CmdQueueFrontOrder, PriorityStillPrecedesTheProfileSync) {
    MiniQueue q;
    for (uint8_t i = 0; i < 5; ++i) q.push(0x40 + i, i);   // ordinary sync burst
    q.push(PYRO_CONT_TEST, 99);
    EXPECT_EQ(q.pop().tag, 99) << "a priority command must still jump the queue";
    EXPECT_EQ(q.pop().tag, 0)  << "and the sync burst must keep its own order";
    EXPECT_EQ(q.pop().tag, 1);
}

TEST(CmdQueueFrontOrder, OrdinaryCommandsAreUntouchedByAFrontInsert) {
    MiniQueue q;
    for (uint8_t i = 0; i < 6; ++i) q.push(0x50 + i, i);
    q.push(PYRO_FIRE_TEST, 200);
    (void)q.pop();                       // the priority one
    for (uint8_t i = 0; i < 6; ++i) {
        EXPECT_EQ(q.pop().tag, i) << "the shift corrupted the ordinary tail at " << (int)i;
    }
}

TEST(CmdQueueFrontOrder, SurvivesRingWrap) {
    MiniQueue q;
    // Drive head deep into the ring so the insert and the shift both wrap.
    for (uint8_t i = 0; i < 18; ++i) q.push(0x60, i);
    for (uint8_t i = 0; i < 18; ++i) (void)q.pop();
    ASSERT_EQ(q.count, 0u);

    q.push(0x61, 1);
    q.push(0x61, 2);
    q.push(PYRO_CONT_TEST, 7);
    q.push(PYRO_CONT_TEST, 8);
    EXPECT_EQ(q.pop().tag, 7);
    EXPECT_EQ(q.pop().tag, 8);
    EXPECT_EQ(q.pop().tag, 1);
    EXPECT_EQ(q.pop().tag, 2);
}
