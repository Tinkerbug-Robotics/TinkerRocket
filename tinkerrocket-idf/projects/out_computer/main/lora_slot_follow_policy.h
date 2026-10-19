#pragma once

#include <stdint.h>
#include <RocketComputerTypes.h>   // LORA_TX_SLOT_RELEASE, LORA_TX_SLOT_COUNT

// Base-station-run transmit slots, the rocket's half (three rockets on one
// base station; the wire contract sits next to LORA_CMD_TX_SLOT_ADJUST).
//
// Every rocket used to transmit free-running at 2 Hz from whenever it booted,
// so three rockets on one channel collided at random.  The base station now
// times each rocket's frames on its own clock and sends that rocket uplink 82
// [ref_seq u16 LE][delta_ms i16 LE]: "put your grid at (the moment you sent
// frame ref_seq) + delta_ms, then every period".  The rocket only remembers
// when it sent its last few frames, by seq, and transmits on that grid.  Its
// own send latency (direct radio vs the UART daughterboard) sits in both the
// remembered time and the frame the base station measured, so it cancels, and
// the rockets never need to hear each other.
//
// A rocket that never gets an adjust keeps today's free-running timing
// exactly.  The lock lives in RAM: a reboot, a release, a retune, hop mode or
// a mute returns the rocket to free-running (main.cpp owns those events).
//
// Pure so the timing is host-testable (rail_restore_policy.h pattern).  All
// times are millis() and every comparison is a wrapped difference, so the
// 49.7-day millis wrap is invisible.

namespace lora_slot_follow
{

// Frames remembered for a ref_seq lookup: 8 s at 2 Hz, several times the base
// station's measure -> uplink -> receive path.  An adjust whose frame has
// already left the ring is ignored; the base station sends a fresh one.
inline constexpr uint8_t kRingSize = 16;

// Margin kept between the end of a beacon and the end of this rocket's slot:
// loop latency between the SLOW frame's TX-done and the beacon's send, plus
// whatever guard the base station leaves at the slot edge.
inline constexpr uint32_t kBeaconGuardMs = 10;

struct State
{
    // (seq, millis at send) of the last kRingSize frames that went on the air.
    uint16_t ring_seq[kRingSize] = {};
    uint32_t ring_ms[kRingSize]  = {};
    uint8_t  ring_next  = 0;     // index the next record overwrites
    uint8_t  ring_count = 0;     // valid records, <= kRingSize

    bool     locked        = false;
    uint32_t anchor_ms     = 0;  // a grid point; rebased to the latest one on every frame
    uint32_t next_due_ms   = 0;  // locked: the grid point the next frame goes on
    uint32_t frame_grid_ms = 0;  // locked: the grid point the last frame was due on
    bool     beacon_owed   = false;  // locked: the last frame was SLOW and no beacon since
};

// ---- the frame-time ring ---------------------------------------------------

inline void recordTx(State& s, uint16_t seq, uint32_t ms)
{
    s.ring_seq[s.ring_next] = seq;
    s.ring_ms[s.ring_next]  = ms;
    s.ring_next = (uint8_t)((s.ring_next + 1) % kRingSize);
    if (s.ring_count < kRingSize) s.ring_count++;
}

// The newest record for `seq`.  A failed send radiates nothing and keeps its
// seq for the retry, so only frames that went out are recorded; the newest-
// first search is belt and braces.
inline bool findTx(const State& s, uint16_t seq, uint32_t& ms_out)
{
    for (uint8_t i = 0; i < s.ring_count; ++i)
    {
        const uint8_t idx = (uint8_t)((s.ring_next + kRingSize - 1 - i) % kRingSize);
        if (s.ring_seq[idx] == seq)
        {
            ms_out = s.ring_ms[idx];
            return true;
        }
    }
    return false;
}

// ---- grid arithmetic -------------------------------------------------------

// The first grid point (anchor + k * period, any integer k) at or after t.
inline uint32_t gridAtOrAfter(uint32_t anchor_ms, uint32_t period_ms, uint32_t t_ms)
{
    if (period_ms == 0) return t_ms;
    const int64_t rel = (int32_t)(t_ms - anchor_ms);
    const int64_t p   = (int64_t)period_ms;
    const int64_t k   = (rel >= 0) ? (rel + p - 1) / p : -((-rel) / p);
    return anchor_ms + (uint32_t)(k * p);
}

// How far the grid through `to_ms` sits from the one through `from_ms`,
// wrapped into (-period/2, period/2].  Positive = frames go out later.
inline int32_t phaseShiftMs(uint32_t from_ms, uint32_t to_ms, uint32_t period_ms)
{
    if (period_ms == 0) return 0;
    const int64_t p = (int64_t)period_ms;
    int64_t m = (int64_t)(int32_t)(to_ms - from_ms) % p;
    if (m < 0) m += p;
    if (m > p / 2) m -= p;
    return (int32_t)m;
}

// ---- transmit timing -------------------------------------------------------

// Free-running: exactly today's rule, a period since the last attempt.
// Locked: the grid point is here or has passed.  A pass that is late still
// sends at once and stays on the grid (advanceGrid steps past any grid points
// it missed rather than re-anchoring to now).
inline bool txDue(const State& s, uint32_t now_ms, uint32_t last_tx_ms, uint32_t period_ms)
{
    if (!s.locked) return (uint32_t)(now_ms - last_tx_ms) >= period_ms;
    return (int32_t)(now_ms - s.next_due_ms) >= 0;
}

// Called where the frame is committed (with last_lora_tx_ms = now), success or
// not, so a failed attempt uses up its grid point the way it uses up the
// period today.  The next frame goes on the first grid point after now.
inline void advanceGrid(State& s, uint32_t now_ms, uint32_t period_ms)
{
    s.beacon_owed = false;
    if (!s.locked) return;
    const uint32_t next = gridAtOrAfter(s.anchor_ms, period_ms, now_ms + 1);
    s.frame_grid_ms = next - period_ms;
    s.anchor_ms     = s.frame_grid_ms;   // same grid; keeps the wrapped differences small
    s.next_due_ms   = next;
}

// Called after the radio took the frame: remember when seq went, and owe a
// beacon after a SLOW frame on the grid.
inline void onFrameSent(State& s, uint16_t seq, uint32_t now_ms, bool slow)
{
    recordTx(s, seq, now_ms);
    s.beacon_owed = s.locked && slow;
}

// ---- the adjust ------------------------------------------------------------

inline void decodeAdjust(const uint8_t* p, uint16_t& ref_seq, int16_t& delta_ms)
{
    ref_seq  = (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
    delta_ms = (int16_t)(uint16_t)(p[2] | ((uint16_t)p[3] << 8));
}

// A grid adjust names one rocket's frame, so it must be addressed to that
// rocket; a release is the same for everyone and may be broadcast (0xFF).
inline bool adjustAddressedToUs(uint8_t target_rid, uint8_t my_rid, int16_t delta_ms)
{
    if (target_rid == my_rid) return true;
    return target_rid == 0xFF && delta_ms == LORA_TX_SLOT_RELEASE;
}

enum class AdjustResult : uint8_t
{
    Locked,        // on the new grid (first lock or a move)
    Released,      // was locked, now free-running
    AlreadyFree,   // a release with nothing locked
    RefNotFound,   // ref_seq is not in the ring: ignored
};

struct AdjustOutcome
{
    AdjustResult result      = AdjustResult::RefNotFound;
    bool         was_locked  = false;
    int32_t      shift_ms    = 0;   // Locked: new grid vs the old (or the free-running phase)
    uint32_t     next_due_ms = 0;   // Locked: when the next frame goes
};

// Apply one uplink 82.  The new grid passes through tx_ms(ref_seq) + delta.
// The first frame on it waits at least half a period after the last frame
// (last_tx_ms), so a move never sends two frames on top of each other; on the
// grid already, that is just the next grid point, which is what makes a
// repeated adjust a no-op.
inline AdjustOutcome applyAdjust(State& s, uint16_t ref_seq, int16_t delta_ms,
                                 uint32_t now_ms, uint32_t last_tx_ms, uint32_t period_ms)
{
    AdjustOutcome out;
    out.was_locked = s.locked;

    if (delta_ms == LORA_TX_SLOT_RELEASE)
    {
        out.result = s.locked ? AdjustResult::Released : AdjustResult::AlreadyFree;
        s.locked = false;
        s.beacon_owed = false;
        return out;
    }

    uint32_t ref_ms = 0;
    if (!findTx(s, ref_seq, ref_ms))
    {
        out.result = AdjustResult::RefNotFound;
        return out;
    }

    const uint32_t anchor = ref_ms + (uint32_t)(int32_t)delta_ms;
    out.shift_ms = phaseShiftMs(s.locked ? s.anchor_ms : last_tx_ms, anchor, period_ms);

    uint32_t earliest = last_tx_ms + period_ms / 2;
    if ((int32_t)(now_ms - earliest) > 0) earliest = now_ms;

    s.locked      = true;
    s.anchor_ms   = anchor;
    s.next_due_ms = gridAtOrAfter(anchor, period_ms, earliest);
    s.beacon_owed = false;

    out.result      = AdjustResult::Locked;
    out.next_due_ms = s.next_due_ms;
    return out;
}

// Back to free-running.  True if it was locked (for the log line).
inline bool unlock(State& s)
{
    const bool was = s.locked;
    s.locked = false;
    s.beacon_owed = false;
    return was;
}

// ---- the name beacon -------------------------------------------------------

// Free-running: no say (the 2 s timer and the state gates are main.cpp's).
// Locked: only right after a SLOW frame, and only if the beacon still ends
// inside this rocket's slot (period / LORA_TX_SLOT_COUNT from the frame's grid
// point).  At SF8/BW250 a FAST frame (102 ms) plus a beacon (up to 71 ms)
// overflows the 166 ms slot; a SLOW frame (56 ms) plus a beacon fits.
inline bool beaconAllowed(const State& s, uint32_t now_ms, uint32_t beacon_toa_ms,
                          uint32_t period_ms)
{
    if (!s.locked) return true;
    if (!s.beacon_owed) return false;
    const uint32_t slot_ms = period_ms / LORA_TX_SLOT_COUNT;
    const uint32_t since   = now_ms - s.frame_grid_ms;
    if (since >= slot_ms) return false;
    return since + beacon_toa_ms + kBeaconGuardMs <= slot_ms;
}

inline void onBeaconSent(State& s)
{
    s.beacon_owed = false;
}

}  // namespace lora_slot_follow
