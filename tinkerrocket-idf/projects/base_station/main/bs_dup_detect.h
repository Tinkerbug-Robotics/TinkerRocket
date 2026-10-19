#pragma once

// Duplicate rocket-ID detector (three-rocket support).
//
// Every rocket computer ships as rocket_id 1. Two rockets on one rid look to
// the base station like ONE rocket whose telemetry jumps around: the tracker
// forward-fills one rocket's frame with the other's fields, the seq gap counts
// garbage, and the app books both onto one tile. Nothing on the radio can tell
// them apart, so the BS can only notice the symptoms and say so ("dup":1 in the
// Tier 1 telemetry) so the operator gives one of them a new ID.
//
// Two symptoms, either one sets the flag:
//   * NAMES: the unit name in the name beacons keeps changing. A single rename
//     (A A A B B B) is an operator renaming the rocket; a name that changes
//     again before the previous one held for kAgreeBeacons beacons (A B A,
//     A A B A, A B C) is two rockets taking turns. The first name written after
//     the slot was allocated (empty -> name) is not a change.
//   * SEQ: two free-running seq counters interleaved on one rid make
//     loraComputeObservedLoss return -1 (a backwards or implausible jump) on
//     most frames. One reboot or one duplicate receive is a single -1, so the
//     flag needs kSeqAnomalies of them within the last kSeqWindow frames.
//
// Clearing is deliberately slow: only kAgreeBeacons consecutive beacons with one
// name, with the seq stream clean, and never while the rocket is in flight. A
// rocket stops beaconing in INFLIGHT (and while hopping), so a flag raised on
// the pad holds through the flight; the rocket's LANDED report clears it when
// its seq stream is clean, and the beacons resume after that anyway.
// "In flight" is the latest frame's state, so a rocket that reboots out of
// INFLIGHT can clear by its beacons too.
//
// Pure — no ESP-IDF / radio dependencies — so the host gtest drives it directly.
// Same pattern as bs_uplink_queue.h / bs_uplink_txwin.h.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "RocketComputerTypes.h"  // loraComputeObservedLoss, RocketState

namespace bs_dup_detect {

inline constexpr uint8_t kAgreeBeacons = 3;   // beacons one name must hold
inline constexpr uint8_t kSeqWindow    = 16;  // frames of seq history (8 s at 2 Hz)
inline constexpr uint8_t kSeqAnomalies = 3;   // -1 gaps in the window that flag

enum class Edge : uint8_t { None, Set, Cleared };

// Why the flag is up (bit mask; both can be true).
inline constexpr uint8_t kReasonNames = 0x01;
inline constexpr uint8_t kReasonSeq   = 0x02;

struct State
{
    bool     flagged    = false;
    uint8_t  reasons    = 0;      // kReason* that set (or re-set) the flag
    uint8_t  agree      = 0;      // consecutive beacons with the current name
    uint8_t  votes      = 0;      // of those, heard with the seq stream clean
    bool     changed    = false;  // a name change has been seen since reset
    bool     in_flight  = false;  // the latest frame said INFLIGHT
    uint16_t seq_hist   = 0;      // 1 bit per frame, newest in bit 0: 1 = -1 gap
    uint8_t  seq_frames = 0;      // frames in seq_hist (saturates at kSeqWindow)

    void reset() { *this = State{}; }
};

inline uint8_t seqAnomalyCount(const State& s)
{
    uint8_t n = 0;
    for (uint16_t h = s.seq_hist; h != 0; h &= (uint16_t)(h - 1)) n++;
    return n;
}

inline bool seqDirty(const State& s) { return seqAnomalyCount(s) >= kSeqAnomalies; }

namespace detail {

inline Edge set(State& s, uint8_t reason)
{
    s.reasons |= reason;
    if (s.flagged) return Edge::None;
    s.flagged = true;
    return Edge::Set;
}

inline Edge clear(State& s)
{
    if (!s.flagged) return Edge::None;
    s.flagged = false;
    s.reasons = 0;
    return Edge::Cleared;
}

}  // namespace detail

// A name beacon from this rid. cur_name is what the tracker holds now (empty
// right after allocation or eviction); new_name is the beacon's name, already
// cut to the tracker's 23 characters. Call BEFORE the tracker stores new_name.
inline Edge onBeacon(State& s, const char* cur_name, const char* new_name)
{
    if (cur_name == nullptr || new_name == nullptr) return Edge::None;

    // A beacon only votes for "one rocket" while the seq stream says so too:
    // two rockets that share a name agree on every beacon.
    const bool clean = !seqDirty(s);

    if (cur_name[0] == '\0')
    {
        // First name since allocation/eviction: not a change.
        s.agree = 1;
        s.votes = clean ? 1 : 0;
        return Edge::None;
    }

    if (strcmp(cur_name, new_name) == 0)
    {
        if (s.agree < 255) s.agree++;
        s.votes = clean ? (uint8_t)(s.votes < 255 ? s.votes + 1 : 255) : 0;
        if (s.flagged && !s.in_flight && s.votes >= kAgreeBeacons)
        {
            return detail::clear(s);
        }
        return Edge::None;
    }

    // The name changed. A second change before the previous name settled is
    // two rockets taking turns; the first change ever is a rename.
    const bool settled = (s.agree >= kAgreeBeacons);
    const bool first   = !s.changed;
    s.changed = true;
    s.agree   = 1;
    s.votes   = clean ? 1 : 0;
    if (!first && !settled) return detail::set(s, kReasonNames);
    return Edge::None;
}

// A telemetry frame from this rid. prev_seq is the tracker's last_seq BEFORE
// this frame (-1 = first contact); state is the frame's rocket_state.
inline Edge onTelemetry(State& s, int32_t prev_seq, uint16_t seq, uint8_t state)
{
    Edge edge = Edge::None;

    if (prev_seq >= 0)
    {
        const bool anomaly = loraComputeObservedLoss(prev_seq, seq) < 0;
        s.seq_hist = (uint16_t)((s.seq_hist << 1) | (anomaly ? 1u : 0u));
        if (s.seq_frames < kSeqWindow) s.seq_frames++;
        if (seqDirty(s))
        {
            edge = detail::set(s, kReasonSeq);
            s.votes = 0;   // beacons before this proved nothing
        }
    }

    // The hold follows the latest frame, not a latch: a rocket lost in flight
    // that reboots into READY can clear by its beacons again. A pair of
    // rockets (one flying, one on the pad) still cannot: their interleaved
    // seq keeps the window dirty and the beacon votes at zero.
    s.in_flight = (state == INFLIGHT);
    if (state == LANDED && s.flagged && !seqDirty(s)) edge = detail::clear(s);
    return edge;
}

}  // namespace bs_dup_detect
