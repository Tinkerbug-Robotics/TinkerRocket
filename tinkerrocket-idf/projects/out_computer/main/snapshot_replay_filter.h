// snapshot_replay_filter.h — which of the FC's SNAPSHOT_MSG frames the OC keeps
// (#383, #846).
//
// The FC sends a FlightSnapshotData at 10 Hz while INFLIGHT. The OC keeps the
// newest in its RAM cache (and the V7/V8 MRAM slot) to hand back if the FC
// reboots. An I2S DMA replay can deliver a snapshot a few hundred ms OLDER
// than the one stored, and storing it would hand recovery stale state, so an
// INFLIGHT frame at most kNewFlightBackstepMs older than the newest one is
// dropped. A frame MUCH older can only be a new flight's near-zero clock.
//
// A non-INFLIGHT frame is the FC's clear. It can only ever disarm (the FC
// restores solely on INFLIGHT), so it is never dropped (#846 SAFETY: dropping
// one left a genuine INFLIGHT frame stored that a later FC panic would restore
// on the ground with the pyros re-armed).
//
// What a clear does to the baseline is the part that changed. The clear's
// flight_elapsed_ms is `now - launch_time_millis`:
//   * A clear that ends a flight (the LANDED one-shot, a sim stop after a
//     sim flight) carries that flight's time, the upper bound of every frame
//     it could still have replayed. It raises the baseline to it, as before,
//     so a replay of a pre-landing frame arriving after the clear is still
//     dropped.
//   * A clear sent with no flight in progress carries the FC's UPTIME
//     (launch_time_millis is 0): a sim start, or the boot clear after a refused
//     frame. Taking that as the baseline was the bug. With an uptime under the
//     10 s backstep, the next flight's INFLIGHT frames all looked like replays
//     until their elapsed passed it. On the V8 bench (2026-10-17), a sim started
//     8.8 s after an FC boot kept no snapshot for its first 8.8 s, and an FC
//     reset 3 s into it found no flight to restore. Such a clear leaves the
//     baseline alone.
//
// The residual: a flight shorter than the backstep, followed by another in the
// same OC session (in practice a short sim), still loses the second flight's
// first seconds. The elapsed time alone cannot tell those frames from replays.

#pragma once

#include <stdint.h>

namespace SnapshotReplayFilter {

inline constexpr uint32_t kNewFlightBackstepMs = 10000;

struct State {
    bool     have_baseline = false;   // an INFLIGHT frame, or a flight-ending clear, was kept
    uint32_t baseline_ms   = 0;       // the newest flight time kept
    bool     in_flight     = false;   // an INFLIGHT frame was kept since the last clear
};

/// Whether to keep this SNAPSHOT_MSG (`inflight` is its rocket_state ==
/// INFLIGHT, `elapsed_ms` its flight_elapsed_ms), updating `st` when it is
/// kept. Dropped frames leave `st` alone.
inline bool keep(State& st, bool inflight, uint32_t elapsed_ms)
{
    if (!inflight)
    {
        // A clear is always kept. Only one that ends a flight moves the
        // baseline, and only up: its elapsed bounds that flight's frames.
        if (st.in_flight && (!st.have_baseline || elapsed_ms > st.baseline_ms))
        {
            st.baseline_ms   = elapsed_ms;
            st.have_baseline = true;
        }
        st.in_flight = false;
        return true;
    }
    if (st.have_baseline && elapsed_ms <= st.baseline_ms &&
        st.baseline_ms - elapsed_ms <= kNewFlightBackstepMs)
    {
        return false;   // a replay: no newer than what is kept, and close behind it
    }
    st.baseline_ms   = elapsed_ms;
    st.have_baseline = true;
    st.in_flight     = true;
    return true;
}

}  // namespace SnapshotReplayFilter
