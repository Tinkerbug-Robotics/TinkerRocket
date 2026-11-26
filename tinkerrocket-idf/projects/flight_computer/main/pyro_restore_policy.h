#pragma once

#include <stdint.h>

// #1760: a pyro channel that had started to fire when the FC reset is spent.
//
// The flight snapshot (FlightSnapshotData.pyroN_fired) used to record a channel
// only once it was Done, at the end of its 200 ms pulse, and the restore set
// every other channel back to Idle with its trigger already met ("no missed
// fire"). A channel in its 10 ms arm settle or its pulse at the reset, or Done
// less than one 10 Hz snapshot before it, was therefore fired again. That is
// the rescue it was meant to be for an unrelated reset, but nothing told it
// apart from a fire that CAUSED the reset — an e-match shorting as it fires,
// sagging the pack — and that fire then repeated on every boot down to the
// ground, pulling the main into the same pulse once below its altitude.
//
// Owner decision (2026-11-25): a fire that had started is never repeated. An
// e-match lights within a few ms of current, so a channel that reached its
// pulse has done its job; the cost is an unrelated reset inside the 10 ms
// settle, before any current flowed, losing that one channel.
//
// So the snapshot carries three values per channel, the FC sends one on the
// pass a channel leaves Idle (retried each pass until the I2S queue takes it),
// and the channel's pulse waits until that frame has reached the OC
// (pulseMayStart): the OC then holds "started" before the channel's FET turns
// on, whatever resets the FC after that. The settle alone (10 ms) is too short
// for it: the sender keeps the I2S DMA ring full, so a frame waits a whole ring
// (~23 ms) behind whatever is queued. ARM is up meanwhile, which puts no
// current through an e-match; only the pulse does.
//
// Pure so the table is host-testable (test_pyro_restore_policy); main.cpp owns
// the channels, the snapshot and the restore.

namespace PyroRestorePolicy {

// FlightSnapshotData.pyroN_fired.
inline constexpr uint8_t kNotStarted = 0;   // Idle
inline constexpr uint8_t kFired      = 1;   // Done: the pulse ran its length (every older writer's "fired")
inline constexpr uint8_t kStarted    = 2;   // ArmSettle or Firing when the snapshot was taken

// The channel as the snapshot sees it.
enum class Phase : uint8_t
{
    Idle,
    Settling,   // PyroChState::ArmSettle
    Firing,     // PyroChState::Firing
    Done,       // PyroChState::Done
};

// What the snapshot records for a channel in `p`.
inline uint8_t snapshotValue(Phase p)
{
    switch (p)
    {
    case Phase::Done:     return kFired;
    case Phase::Settling:
    case Phase::Firing:   return kStarted;
    default:              return kNotStarted;
    }
}

// The restore: does a channel the snapshot recorded as `v` stay off for the
// rest of the flight? Any value but kNotStarted does, so a value from a newer
// writer than this one is also read as spent: a fail-safe that never repeats
// a fire, which is the point of this file.
inline bool restoredSpent(uint8_t v)
{
    return v != kNotStarted;
}

// For the restore's log: was the channel mid-fire, not finished, at the reset?
inline bool restoredMidFire(uint8_t v)
{
    return v != kNotStarted && v != kFired;
}

// Send a snapshot on this pass, off the 10 Hz cadence: a channel left Idle.
inline bool edgeSnapshotDue(const bool left_idle[4])
{
    return left_idle[0] || left_idle[1] || left_idle[2] || left_idle[3];
}

// Past the DMA ring's own transit time, before the frame counts as at the OC.
inline constexpr uint32_t kWireMarginMs = 5;
// The longest a pulse waits for its frame. A stalled or dead link never holds
// a deployment back longer than this from the trigger (3 m at a drogue's
// 20 m/s); past it the pulse starts, and only a reset in that pulse could
// repeat it.
inline constexpr uint32_t kMaxPulseWaitMs = 150;

// May a channel in its arm settle start its pulse? settle_ms: since it left
// Idle; min_settle_ms: PYRO_ARM_SETTLE_MS; on_wire: the snapshot that records
// it as started has left the FC's queue for the DMA ring, wire_ms ago;
// ring_ms: how long the ring takes to send. The frame is then at the OC.
inline bool pulseMayStart(uint32_t settle_ms, uint32_t min_settle_ms,
                          bool on_wire, uint32_t wire_ms, uint32_t ring_ms)
{
    if (settle_ms < min_settle_ms) return false;
    if (on_wire && wire_ms >= ring_ms + kWireMarginMs) return true;
    return settle_ms >= kMaxPulseWaitMs;
}

}  // namespace PyroRestorePolicy
