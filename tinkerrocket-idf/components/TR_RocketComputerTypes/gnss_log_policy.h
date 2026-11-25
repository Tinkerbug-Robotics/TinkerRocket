#pragma once

// The REDUCED GNSS log (GNSS_LOG_REDUCED, RocketComputerTypes.h): which of a
// mosaic-G5's SBF blocks the FC still logs.
//
// The receiver sends MeasEpoch, MeasExtra and PVTGeodetic every 50 ms, its
// status once a second and its navigation data as it changes. The two raw
// measurement blocks are the bulk: ~1.5 and ~1.6 kB an epoch on a
// 33-satellite sky, ~56 of the ~60 kB/s the Goblin's log carried at 3840 Hz
// (2026-10-16). At 7680 Hz the IMU alone takes ~230 kB/s of the OC's
// ~310 kB/s flash drain, so the reduced log keeps them once a second and
// everything else as it comes: PVTGeodetic at 20 Hz (the fix GNSSData
// carries, with the receiver's own mode and error fields), the status, and
// the navigation data any later processing of the raw measurements needs.
//
// Once a second is the first epoch of each GPS second, both of its blocks:
// on the receiver's 20 Hz grid that is the epoch on the whole second, the one
// its PPS marks. It is keyed on the epoch's time of week, not on the whole
// second exactly, so a grid that misses the whole second (another rate, a
// receiver clock that steps) still gives one epoch a second.
//
// What is not an SBF block passes: the same record (GNSS_RTCM_MSG) carries
// the LC86G's RTCM3 on the Beetle. A measurement block with no time yet (TOW
// do-not-use, before the receiver has GPS time) does not: it cannot be placed
// in the flight, and the receiver's own microSD card keeps it.
//
// Pure so the cut is host-tested (tests_cpp/test_gnss_log_policy.cpp), on
// recorded blocks as well as made-up ones.

#include <stddef.h>
#include <stdint.h>

namespace gnss_log
{

// The blocks the reduced log thins (mosaic-G5 Reference Guide, appendix B).
// SbfParser.h checks that it names them the same.
inline constexpr uint16_t kSbfMeasEpoch = 4027;
inline constexpr uint16_t kSbfMeasExtra = 4000;
inline constexpr uint32_t kTowDnu = 4294967295u;   // SBF's TOW do-not-use

// The epoch kept last. One for both blocks, so the MeasExtra kept is the
// MeasEpoch's own epoch.
struct SbfCut
{
    uint32_t kept_s   = 0xFFFFFFFFu;   // its GPS second (TOW / 1000)
    uint32_t kept_tow = kTowDnu;       // its time of week, ms
};

// True when `frame` (a GNSS_RTCM_MSG frame, whole, before it is split into
// parts) goes into the log. Only `reduced` cuts anything: without it every
// frame goes, as before the setting existed.
inline bool keep(SbfCut& cut, const uint8_t* frame, size_t len, bool reduced)
{
    if (!reduced) return true;
    // "$@", CRC u2, ID u2 (block number in bits 0-12), Length u2, TOW u4, WNc u2.
    if (frame == nullptr || len < 14 || frame[0] != '$' || frame[1] != '@') return true;
    const uint16_t number = (uint16_t)((frame[4] | (frame[5] << 8)) & 0x1FFF);
    if (number != kSbfMeasEpoch && number != kSbfMeasExtra) return true;

    const uint32_t tow = (uint32_t)frame[8] | ((uint32_t)frame[9] << 8) |
                         ((uint32_t)frame[10] << 16) | ((uint32_t)frame[11] << 24);
    if (tow == kTowDnu) return false;
    if (tow == cut.kept_tow) return true;   // the other block of the epoch kept
    const uint32_t s = tow / 1000u;
    if (s == cut.kept_s) return false;      // this second's epoch is already in
    cut.kept_s   = s;
    cut.kept_tow = tow;
    return true;
}

}  // namespace gnss_log
