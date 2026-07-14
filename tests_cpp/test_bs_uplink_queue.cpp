#include <gtest/gtest.h>

#include "bs_uplink_queue.h"

using bs_uplink_queue::Entry;
using bs_uplink_queue::PushResult;
using bs_uplink_queue::Queue;

// The #502 fix: the uplink was a single slot, so queuing a command overwrote
// the pending one along with its remaining retries. The uplink is BLIND — the
// rocket sends no command ACK — so the retry count IS the delivery mechanism.
// Discarding it directly cuts a command's odds of arriving, silently.
//
// These tests pin the FIFO behaviour that replaces the slot: commands drain in
// order, each keeps its own retry budget, and a full queue drops loudly instead
// of trampling something real.

namespace {

constexpr uint8_t SYNC    = 0xCA;  // config::UPLINK_SYNC_BYTE
constexpr uint8_t NID     = 0;
constexpr uint8_t BCAST   = 0xFF;
constexpr uint8_t NO_HOP  = 0;     // LORA_NEXT_CH_NO_HOP
constexpr uint8_t RETRIES = 8;     // config::UPLINK_RETRIES

PushResult push(Queue& q, uint8_t cmd, uint8_t retries = RETRIES,
                const uint8_t* payload = nullptr, size_t len = 0) {
    return q.push(SYNC, NID, BCAST, NO_HOP, cmd, payload, len, retries);
}

// Drain the head command's retries the way serviceUplink() does, and pop it on
// the pass after the last retry. Returns the number of transmissions it got.
int drainHead(Queue& q) {
    int sent = 0;
    while (Entry* e = q.head()) {
        if (e->retries_left == 0) { q.pop(); break; }
        e->retries_left--;
        sent++;
    }
    return sent;
}

}  // namespace

// ---- The regression: a second command must not eat the first one's retries ----

// Replays the exact bench-log sequence from #502: sim-config (cmd 5) is queued
// with 8 retries, gets 3 transmissions out, and then sim-start (cmd 6) arrives.
// Before the fix, cmd 5 was discarded with 5 of its 8 retries unsent.
TEST(BsUplinkQueue, NewCommandDoesNotDiscardPendingCommandsRetries) {
    Queue q;
    ASSERT_EQ(push(q, /*cmd=*/5), PushResult::Queued);

    for (int i = 0; i < 3; i++) q.head()->retries_left--;  // 3 blind TXs went out
    ASSERT_EQ(q.head()->cmd(), 5);
    ASSERT_EQ(q.head()->retries_left, 5);

    ASSERT_EQ(push(q, /*cmd=*/6), PushResult::Queued);

    // cmd 5 is still the head and still owns its remaining 5 retries.
    EXPECT_EQ(q.size(), 2u);
    EXPECT_EQ(q.head()->cmd(), 5);
    EXPECT_EQ(q.head()->retries_left, 5);

    // It drains those 5, and only then does cmd 6 get the radio — with its own
    // full budget of 8, not a truncated one.
    EXPECT_EQ(drainHead(q), 5);
    ASSERT_NE(q.head(), nullptr);
    EXPECT_EQ(q.head()->cmd(), 6);
    EXPECT_EQ(q.head()->retries_left, 8);

    EXPECT_EQ(drainHead(q), 8);
    EXPECT_TRUE(q.empty());
}

// ---- Ordering ----

TEST(BsUplinkQueue, CommandsDrainInFifoOrder) {
    Queue q;
    for (uint8_t cmd = 1; cmd <= 4; cmd++) ASSERT_EQ(push(q, cmd), PushResult::Queued);

    for (uint8_t expected = 1; expected <= 4; expected++) {
        ASSERT_NE(q.head(), nullptr);
        EXPECT_EQ(q.head()->cmd(), expected);
        drainHead(q);
    }
    EXPECT_TRUE(q.empty());
}

// The ring must wrap without corrupting order once entries have been popped.
TEST(BsUplinkQueue, RingWrapsWithoutLosingOrder) {
    Queue q;
    // Fill, drain, and refill several times over so head_ wraps past kDepth.
    for (int round = 0; round < 3; round++) {
        for (size_t i = 0; i < bs_uplink_queue::kDepth; i++) {
            ASSERT_EQ(push(q, (uint8_t)(i + 1)), PushResult::Queued);
        }
        EXPECT_TRUE(q.full());
        for (size_t i = 0; i < bs_uplink_queue::kDepth; i++) {
            ASSERT_NE(q.head(), nullptr);
            EXPECT_EQ(q.head()->cmd(), (uint8_t)(i + 1)) << "round " << round;
            drainHead(q);
        }
        EXPECT_TRUE(q.empty());
    }
}

// ---- busy() is the successor to the old uplink_pending flag ----

TEST(BsUplinkQueue, BusyIsTrueUntilTheQueueFullyDrains) {
    Queue q;
    EXPECT_FALSE(q.busy());

    push(q, /*cmd=*/5);
    push(q, /*cmd=*/6);
    EXPECT_TRUE(q.busy());

    drainHead(q);
    EXPECT_TRUE(q.busy()) << "cmd 6 is still queued — the uplink is not done";

    drainHead(q);
    EXPECT_FALSE(q.busy());
}

// ---- Full queue: drop the newcomer loudly, never the pending work ----

TEST(BsUplinkQueue, FullQueueRejectsNewCommandAndKeepsExistingOnes) {
    Queue q;
    for (size_t i = 0; i < bs_uplink_queue::kDepth; i++) {
        ASSERT_EQ(push(q, (uint8_t)(i + 1)), PushResult::Queued);
    }
    ASSERT_TRUE(q.full());

    EXPECT_EQ(push(q, /*cmd=*/99), PushResult::RejectedFull);

    // The rejection must not have displaced anything: same depth, same head.
    EXPECT_EQ(q.size(), bs_uplink_queue::kDepth);
    EXPECT_EQ(q.head()->cmd(), 1);
    EXPECT_EQ(q.head()->retries_left, RETRIES);
}

// ---- #286: oversized payloads are rejected, not truncated ----

TEST(BsUplinkQueue, OversizedPayloadIsRejectedNotTruncated) {
    Queue q;
    uint8_t payload[bs_uplink_queue::kMaxPayload + 1] = {};

    EXPECT_EQ(push(q, /*cmd=*/7, RETRIES, payload, sizeof(payload)),
              PushResult::RejectedOversized);
    EXPECT_TRUE(q.empty()) << "a rejected command must not occupy a slot";
}

TEST(BsUplinkQueue, MaxSizePayloadIsAccepted) {
    Queue q;
    uint8_t payload[bs_uplink_queue::kMaxPayload] = {};
    payload[0] = 0xAB;
    payload[bs_uplink_queue::kMaxPayload - 1] = 0xCD;

    ASSERT_EQ(push(q, /*cmd=*/7, RETRIES, payload, sizeof(payload)), PushResult::Queued);
    const Entry* e = q.head();
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->len, bs_uplink_queue::kHeaderBytes + bs_uplink_queue::kMaxPayload);
    EXPECT_EQ(e->buf[bs_uplink_queue::kHeaderBytes], 0xAB);
    EXPECT_EQ(e->buf[e->len - 1], 0xCD);
}

// An oversized payload must be reported as oversized even when the queue is
// also full — otherwise the operator gets told "queue full" for a command that
// could never have been sent at any depth.
TEST(BsUplinkQueue, OversizedIsReportedEvenWhenQueueIsFull) {
    Queue q;
    for (size_t i = 0; i < bs_uplink_queue::kDepth; i++) push(q, (uint8_t)(i + 1));
    ASSERT_TRUE(q.full());

    uint8_t payload[bs_uplink_queue::kMaxPayload + 1] = {};
    EXPECT_EQ(push(q, /*cmd=*/99, RETRIES, payload, sizeof(payload)),
              PushResult::RejectedOversized);
}

// ---- Wire format: the header the rocket parses ----

TEST(BsUplinkQueue, BuildsTheV2WireHeader) {
    Queue q;
    const uint8_t payload[3] = {0x11, 0x22, 0x33};
    ASSERT_EQ(q.push(SYNC, /*nid=*/7, /*target_rid=*/42, /*next_ch=*/0,
                     /*cmd=*/10, payload, sizeof(payload), RETRIES),
              PushResult::Queued);

    const Entry* e = q.head();
    ASSERT_NE(e, nullptr);
    // [0xCA][network_id][target_rid][next_channel_idx][cmd][len][payload...]
    EXPECT_EQ(e->buf[0], 0xCA);
    EXPECT_EQ(e->buf[1], 7);
    EXPECT_EQ(e->buf[2], 42);
    EXPECT_EQ(e->buf[3], 0);
    EXPECT_EQ(e->buf[4], 10);
    EXPECT_EQ(e->buf[5], 3);
    EXPECT_EQ(e->buf[6], 0x11);
    EXPECT_EQ(e->buf[7], 0x22);
    EXPECT_EQ(e->buf[8], 0x33);
    EXPECT_EQ(e->len, bs_uplink_queue::kHeaderBytes + 3u);
    EXPECT_EQ(e->cmd(), 10);
}

TEST(BsUplinkQueue, ZeroLengthPayloadIsHeaderOnly) {
    Queue q;
    ASSERT_EQ(push(q, /*cmd=*/6), PushResult::Queued);  // e.g. sim-start
    const Entry* e = q.head();
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->len, bs_uplink_queue::kHeaderBytes);
    EXPECT_EQ(e->buf[5], 0);
}

// Each entry owns its own buffer — a later push must not scribble on the
// packet bytes of one already queued (the single-slot bug, in miniature).
TEST(BsUplinkQueue, QueuedEntriesDoNotShareBuffers) {
    Queue q;
    const uint8_t a[2] = {0xAA, 0xAA};
    const uint8_t b[2] = {0xBB, 0xBB};
    ASSERT_EQ(push(q, /*cmd=*/5, RETRIES, a, sizeof(a)), PushResult::Queued);
    ASSERT_EQ(push(q, /*cmd=*/6, RETRIES, b, sizeof(b)), PushResult::Queued);

    const Entry* first = q.head();
    EXPECT_EQ(first->cmd(), 5);
    EXPECT_EQ(first->buf[6], 0xAA);
    EXPECT_EQ(first->buf[7], 0xAA);
}

// ---- Retry budgets are per-command ----

TEST(BsUplinkQueue, EachCommandKeepsItsOwnRetryBudget) {
    Queue q;
    push(q, /*cmd=*/1, /*retries=*/2);  // heartbeat-style: fewer retries, low airtime
    push(q, /*cmd=*/5, /*retries=*/8);  // config: full budget

    EXPECT_EQ(drainHead(q), 2);
    ASSERT_NE(q.head(), nullptr);
    EXPECT_EQ(q.head()->retries_left, 8);
    EXPECT_EQ(drainHead(q), 8);
    EXPECT_TRUE(q.empty());
}

// ---- Empty-queue safety ----

TEST(BsUplinkQueue, HeadIsNullAndPopIsSafeWhenEmpty) {
    Queue q;
    EXPECT_EQ(q.head(), nullptr);
    q.pop();  // must not underflow
    EXPECT_TRUE(q.empty());
    EXPECT_EQ(q.size(), 0u);
    EXPECT_EQ(q.head(), nullptr);
}
