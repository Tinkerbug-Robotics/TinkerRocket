#pragma once

#include <stddef.h>
#include <stdint.h>

#include "RocketComputerTypes.h"   // SensorHealthState, shStorageState

// #281/#278: classify flight-log storage for the #303 scorecard. A full or
// write-failing NAND silently dropped the 2026-06-25 guided flight (recovery
// surfaced the previous day's data); folding a verdict into sensor_health makes
// it visible on the pre-launch go/no-go and the live downlink instead.
//
// ocStorageHealth() in main.cpp gathers the inputs from the flight log and the
// logger; the decision lives here so the table is host-testable
// (inflight_refusal_policy.h pattern). It is the most severe of several
// independent limits, each of which has lost a flight on its own:
//
//   - #566: an uninitialized flight log is the MOST severe storage state, not
//     an inapplicable one. flightlog.begin() failing at boot (corrupt index /
//     metadata read error) is deliberately non-fatal, so the OC runs — but
//     flightlogWriteSink() then refuses every frame and the #271 drop path
//     discards ALL flight data. Returning SH_NA hid exactly the silent loss
//     this fold-in exists to surface: the app hides the Storage row on NA and
//     excludes it from the go/no-go, so the operator saw a green board. There
//     is no logger-disabled OC build (begin() is unconditional), so NA is never
//     legitimate once boot completes — report BAD and let the scorecard go red.
//   - #1127: a failed recovery scan LEAVES the log surface initialized so the
//     stored flights can be downloaded and deleted — but no new flight can be
//     logged this boot, which is the same silent loss. Keep it red.
//   - shStorageState() on the FLIGHT-REGION free count (#1235 item 3). The
//     bitmap's whole-die count also holds the 32 LFS pre-region blocks and the
//     4 metadata blocks, which are never allocated and always read FREE, so it
//     is 36 high on every shipping geometry. Compared against prealloc_blocks
//     (80) that moved the BAD trip point from 80 genuinely free region blocks
//     to 44 and DEGRADED from 160 to 124: the go/no-go read OK or DEGRADED
//     when the region had less room than the policy intends. The input is
//     TR_FlightLog::regionFreeBlocks(), never bitmap().countInState().
//   - #281: the flight index is a second, independent capacity limit. Once it
//     is full, finalize cannot record the flight even with free blocks.
//   - #826: a board configured for an MRAM that did not answer the boot probe.
//     DEGRADED rather than BAD — frames still reach the NAND, so nothing is
//     lost, but the ring shrank to internal RAM and brownout recovery is gone
//     for the session. Never fires on a board that correctly has no MRAM.
//   - #1235 item 4: a V9/V10 board whose PSRAM ring did not allocate. The
//     logger falls back to the 64 KB internal ring and records the outcome only
//     in isRingInPsram(); the boot log does shout about it, but the OC console
//     is USB-Serial/JTAG and light sleep silences it, so nothing reached the
//     operator and the scorecard stayed green on an 8x-shrunk shock absorber.
//     Same DEGRADED-not-BAD reasoning as #826, and the same class of quiet
//     downgrade the #566 and #826 fold-ins were written for.
namespace StorageHealthPolicy {

struct Inputs
{
    bool     flightlog_initialized = false;  // TR_FlightLog::isInitialized()
    bool     recovery_failed       = false;  // TR_FlightLog::recoveryFailed() (#1127)
    uint32_t region_free_blocks    = 0;      // TR_FlightLog::regionFreeBlocks() (#1235 item 3)
    uint32_t prealloc_blocks       = 0;      // Config::prealloc_blocks — one flight's reservation
    uint32_t nand_prog_fail        = 0;      // TR_LogToFlashStats::nand_prog_fail, this session
    size_t   index_used            = 0;      // FlightIndex::size()
    size_t   index_capacity        = 0;      // FlightIndex::MAX_ENTRIES
    bool     mram_probe_failed     = false;  // TR_LogToFlash::mramProbeFailed() (#826)
    bool     psram_ring_expected   = false;  // config::RING_IN_PSRAM — the board header's call
    bool     mram_enabled          = false;  // TR_LogToFlash::isMramEnabled(): there is no RAM ring then
    bool     ring_in_psram         = false;  // TR_LogToFlash::isRingInPsram()
};

// Index slots below which the verdict degrades: room for fewer than this many
// more flights reads DEGRADED (the #281 fold-in's original margin).
inline constexpr size_t kIndexHeadroomEntries = 4;

// #1235 item 4 on its own: the board expected its RAM ring in PSRAM and got
// internal RAM instead. Meaningless (false) while an MRAM ring is in use.
inline bool psramRingMissing(const Inputs& in)
{
    return in.psram_ring_expected && !in.mram_enabled && !in.ring_in_psram;
}

inline SensorHealthState verdict(const Inputs& in)
{
    if (!in.flightlog_initialized) return SH_BAD;   // #566
    if (in.recovery_failed)        return SH_BAD;   // #1127
    SensorHealthState st = shStorageState(in.region_free_blocks,
                                          in.prealloc_blocks,
                                          in.nand_prog_fail);
    // #281: the index limit.
    if (in.index_used >= in.index_capacity) st = SH_BAD;
    else if (in.index_used + kIndexHeadroomEntries >= in.index_capacity && st == SH_OK)
        st = SH_DEGRADED;
    // #826 and #1235 item 4: the ring landed somewhere smaller than designed.
    // Both only ever lower OK to DEGRADED — a BAD verdict already says more.
    if (in.mram_probe_failed && st == SH_OK) st = SH_DEGRADED;
    if (psramRingMissing(in)  && st == SH_OK) st = SH_DEGRADED;
    return st;
}

}  // namespace StorageHealthPolicy
