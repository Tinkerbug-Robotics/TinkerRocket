#pragma once

// BLE command ring (#517).
//
// The app's command intake used to be a SINGLE latch: the NimBLE write callback
// overwrote `pending_command_` unconditionally, and the loop task drained it
// once per iteration. Two commands arriving between two polls lost the first
// outright — and the app's connect-time sync fires ~13 commands back-to-back
// with no pacing (#366), so any loop stall longer than their ~60-90 ms spacing
// dropped one. #221 papered over that by shortening the OC loop period; giving
// the buffer depth is the actual fix.
//
// Each entry carries the command AND everything that travels with it. Exactly
// one of {payload, file_list_page, delete_name, download_name} is populated per
// command (onCommandWrite's dispatch chain is mutually exclusive), but they are
// distinct fields so a delete target can never be handed back as a download
// target. Keeping them per-entry also closes the gap #384 left open: the
// filename/page accessors used to read the live latch, so a BLE write landing
// between getCommand() and getDeleteFilename() could swap the name mid-handler.
//
// Pure — no NimBLE / ESP-IDF / FreeRTOS dependencies — so the host gtest can
// drive it directly. TR_BLE_To_APP owns the locking: push() and pop() are each
// called inside the s_cmd_mux critical section, which is what preserves #384's
// "a command and its payload travel as one atomic unit" guarantee.

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace tr_ble {

inline constexpr size_t kMaxPayload  = 80;  // max = RollProfileData, 76 bytes
inline constexpr size_t kMaxFilename = 64;

// 16 comfortably holds the ~13-command connect-time sync burst (#366) even when
// the loop is stalled through all of it — OC cmd-8 log recovery blocks loop_oc
// for 30-90 s.
inline constexpr size_t kRingDepth = 16;

struct PendingCommand
{
    uint8_t cmd            = 0;
    uint8_t payload[kMaxPayload] = {};
    size_t  payload_len    = 0;
    uint8_t file_list_page = 0;
    char    delete_name[kMaxFilename]   = {};
    char    download_name[kMaxFilename] = {};
    bool    download_l2cap = false;   // #526: download_name is a cmd-44 (L2CAP) request
};

class CommandRing
{
public:
    bool   empty() const { return count_ == 0; }
    bool   full()  const { return count_ == kRingDepth; }
    size_t size()  const { return count_; }

    // Append. Returns false (and keeps the ring untouched) when full — the
    // caller drops the NEWEST command rather than rotating out older ones the
    // app sent first and is still waiting on.
    bool push(const PendingCommand& e)
    {
        if (full()) return false;
        entries_[(head_ + count_) % kRingDepth] = e;
        count_++;
        return true;
    }

    // Pop the OLDEST entry into `out`. Returns false and leaves `out` untouched
    // when empty.
    bool pop(PendingCommand& out)
    {
        if (empty()) return false;
        out   = entries_[head_];
        head_ = (head_ + 1) % kRingDepth;
        count_--;
        return true;
    }

private:
    PendingCommand entries_[kRingDepth];
    size_t head_  = 0;   // next to consume
    size_t count_ = 0;
};

}  // namespace tr_ble
