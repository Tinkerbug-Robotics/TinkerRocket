#pragma once
// Collision watchdog for the transmit slots (three-rocket support).
//
// Two rockets that transmit at the same instant reach the base station as one
// garbled frame: the radio syncs, the payload fails CRC, and neither rocket is
// decoded. The slot controller can only move a rocket it can time, so it can
// never pull such a pair apart, and locked rockets hold their grid exactly, so
// they never drift apart either. Bench 2026-10-21: the Beetle and the V5 sat
// on one grid point after sharing rid 1; the base station logged a CRC failure
// every 500 ms and decoded nothing but the occasional beacon for minutes.
//
// The signature is a steady stream of CRC failures. When the base station
// counts at least kMinCrcFails in a kWindowMs window it broadcasts a release:
// every rocket drops its lock and moves to a random phase (the rocket side's
// lastTxForRandomPhase), the pair comes apart (each draws its own phase), and
// the controller slots them again once it hears them. Not while any rocket is
// in flight (a pair locks together on the ground, where the slots are made;
// in flight a release would only shuffle healthy rockets), and at most once
// per kMinIntervalMs, so a weak rocket at the edge of range, which fails CRC on
// its own, costs a re-slot now and then rather than a storm of releases.
//
// Pure — no ESP-IDF / radio dependencies — so the host gtest drives it directly.
// Same pattern as bs_dup_detect.h / bs_uplink_txwin.h.
#include <cstdint>

namespace bs_collision_watch {

inline constexpr uint32_t kWindowMs      = 10000;  // counting window
inline constexpr uint32_t kMinCrcFails   = 10;     // 1 Hz: a pair at 2 Hz is ~20
inline constexpr uint32_t kMinIntervalMs = 15000;  // between releases

struct Watch
{
    bool     started        = false;
    uint32_t win_start_ms   = 0;
    uint32_t crc_at_start   = 0;    // the radio's running CRC-fail count
    bool     have_release   = false;
    uint32_t last_release_ms = 0;
    uint32_t last_window_fails = 0; // for the log line

    // Call every loop pass with the radio's running CRC-fail counter.
    // armed: slots are on and in use (fixed link mode, nothing hopping).
    // Returns true when a release should be broadcast now.
    bool onTick(uint32_t now_ms, uint32_t crc_total, bool armed, bool in_flight)
    {
        if (!started || !armed || in_flight)
        {
            started      = true;
            win_start_ms = now_ms;
            crc_at_start = crc_total;
            return false;
        }
        if ((uint32_t)(now_ms - win_start_ms) < kWindowMs) return false;

        last_window_fails = crc_total - crc_at_start;
        win_start_ms = now_ms;
        crc_at_start = crc_total;
        if (last_window_fails < kMinCrcFails) return false;
        if (have_release && (uint32_t)(now_ms - last_release_ms) < kMinIntervalMs) return false;

        have_release    = true;
        last_release_ms = now_ms;
        return true;
    }
};

}  // namespace bs_collision_watch
