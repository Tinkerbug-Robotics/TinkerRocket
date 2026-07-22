#pragma once

// Base-station uplink command FIFO (#502).
//
// The uplink used to be a single slot: queuing a command overwrote whatever
// was pending, along with its remaining retries. Because the uplink is BLIND —
// the rocket sends no command ACK — the retry count *is* the delivery
// mechanism, so discarding retries directly cuts a command's odds of arriving.
// The bench log caught the sim-config (cmd 5) getting 3 of its 8 transmissions
// out before sim-start (cmd 6) clobbered it. Same class of bug as #366 on the
// OC side.
//
// This is the FIFO that replaces the slot: commands drain in order, each
// keeping its own retry budget, and a genuinely full queue drops loudly rather
// than silently trampling a pending command.
//
// Pure — no ESP-IDF / FreeRTOS / radio dependencies — so the host gtest can
// drive it directly. Same pattern as bs_uplink_policy.h / bs_log_policy.h.

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace bs_uplink_queue {

// Wire format v2: [0xCA][network_id][target_rid][next_channel_idx][cmd][len][payload...]
inline constexpr size_t kHeaderBytes = 6;
inline constexpr size_t kMaxPacket   = 40;  // cmd 15 channel-set push needs 27 payload at BW=125
// 1 byte of slack, matching the pre-FIFO bound so #286's reject threshold is unchanged.
inline constexpr size_t kMaxPayload  = kMaxPacket - kHeaderBytes - 1;  // 33

// Depth 8 covers the bursts the app actually sends (config-then-start is 2;
// a full profile sync is a handful) with room to spare. Each entry is ~44 B,
// so the whole FIFO is ~350 B of static RAM.
inline constexpr size_t kDepth = 8;

// Byte offset of the command id inside a built packet.
inline constexpr size_t kCmdOffset = 4;

enum class PushResult : uint8_t
{
    Queued,
    RejectedOversized,  // #286: reject rather than truncate into a malformed command
    RejectedFull,
};

struct Entry
{
    uint8_t buf[kMaxPacket];
    size_t  len          = 0;
    uint8_t retries_left = 0;
    // #565: consecutive send() FAILURES (startTransmit error — nothing went on
    // air). Deliberately separate from retries_left, which counts blind
    // TRANSMISSIONS and is the delivery mechanism: an SPI hiccup must not eat
    // a command's delivery odds. serviceUplink pops the head when this hits
    // UPLINK_MAX_SEND_FAILURES so a wedged radio can't jam the queue forever.
    uint8_t send_failures = 0;

    uint8_t cmd() const { return buf[kCmdOffset]; }
};

class Queue
{
public:
    bool   empty() const { return count_ == 0; }
    bool   full()  const { return count_ == kDepth; }
    size_t size()  const { return count_; }

    // True while any command is queued or mid-retry. This is the successor to
    // the old `uplink_pending` flag: callers use it to mean "the uplink is
    // busy, don't start something else on the radio".
    bool busy() const { return count_ != 0; }

    PushResult push(uint8_t sync_byte, uint8_t network_id, uint8_t target_rid,
                    uint8_t next_channel, uint8_t cmd,
                    const uint8_t* payload, size_t payload_len, uint8_t retries)
    {
        // Checked before the full() test so an oversized payload is reported as
        // oversized regardless of queue depth, and never displaces a real entry.
        if (payload_len > kMaxPayload) return PushResult::RejectedOversized;
        if (full())                    return PushResult::RejectedFull;

        Entry& e = slots_[(head_ + count_) % kDepth];
        e.buf[0] = sync_byte;
        e.buf[1] = network_id;
        e.buf[2] = target_rid;
        e.buf[3] = next_channel;
        e.buf[4] = cmd;
        e.buf[5] = (uint8_t)payload_len;
        if (payload_len > 0 && payload != nullptr)
        {
            memcpy(&e.buf[kHeaderBytes], payload, payload_len);
        }
        e.len           = kHeaderBytes + payload_len;
        e.retries_left  = retries;
        e.send_failures = 0;  // #565: slots are reused — never inherit a stale count
        count_++;
        return PushResult::Queued;
    }

    // Command currently transmitting; nullptr when idle.
    Entry*       head()       { return count_ ? &slots_[head_] : nullptr; }
    const Entry* head() const { return count_ ? &slots_[head_] : nullptr; }

    void pop()
    {
        if (count_ == 0) return;
        head_ = (head_ + 1) % kDepth;
        count_--;
    }

private:
    Entry  slots_[kDepth];
    size_t head_  = 0;
    size_t count_ = 0;
};

}  // namespace bs_uplink_queue
