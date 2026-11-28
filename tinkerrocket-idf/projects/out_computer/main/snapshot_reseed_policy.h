// snapshot_reseed_policy.h — what the OC does at boot with the flight a reboot
// cut off, found on NAND before the deferred scan has indexed it (#1717).
//
// The #846 re-seed looked only for a flight_recovered_* index entry. Since
// #1307 the scan that creates that entry runs later, on its own task, and it
// reads every page of the flight (minutes for a long one), so the entry was
// never there when the re-seed looked, and the FC got the flight token's
// launch-edge frame or nothing. TR_FlightLog::newestOrphanTail now finds the
// same flight from far fewer reads. This header decides what to do with it.
//
// THE ONCE-MARKER. As #846's: an NVS record of the last cut-off flight this OC
// served, declined or retired, so it can never answer a fault reset weeks
// later with INFLIGHT. One slot is enough: only the newest orphan is ever
// served, and a newer one replaces it. The orphan's identity is its flight id,
// its first block and its last page's sequence number (ids are reused after
// deletes). Written:
//   - on consumption, once the FC has been served (never on seeding, or a
//     second reset before the ask would burn the flight's only chance);
//   - on decline: a tail with no servable frame, or one that ends in the FC's
//     LANDED clear, declines identically on every later boot;
//   - on a cold boot, without seeding: a cold start is not a recovery context,
//     and an unmarked orphan would be served by some later fault reset;
//   - when the RTC copy served this boot instead (snapshot_rtc_copy.h): that
//     frame is newer than the NAND tail, so once it has been served the tail is
//     history too;
//   - when the RTC copy says the flight is over (Verdict::Over): the tail is
//     older than that word, so it declines.
// The #846 indexed path honors the same marker: once the scan has indexed the
// orphan, its entry has the same id, and its block range holds the orphan's
// first block (it can start lower, on blank blocks of the run).
//
// Pure and host-tested (tests_cpp/test_snapshot_reseed.cpp).

#pragma once

#include <stdint.h>

namespace SnapshotReseed {

/// An NVS key the OC has not written reads back as this: no flight id or block
/// is ever this value.
inline constexpr uint32_t kUnset = 0xFFFFFFFFu;

struct OrphanId
{
    uint32_t flight_id   = kUnset;
    uint32_t start_block = kUnset;
    uint32_t last_seq    = kUnset;
};

inline constexpr bool sameOrphan(const OrphanId& a, const OrphanId& b)
{
    return a.flight_id != kUnset && a.flight_id == b.flight_id &&
           a.start_block == b.start_block && a.last_seq == b.last_seq;
}

enum class OrphanAction : uint8_t
{
    Nothing,       // the marker already records it
    Retire,        // a cold boot: mark it, seed nothing
    MarkOnServe,   // the RTC copy seeded the cache: mark it once the FC is served
    Seed,          // seed the cache from its tail; mark it once the FC is served
    Decline,       // its tail has no servable frame, or ends in LANDED: mark it now
};

/// `marked`: the NVS marker records this orphan.
/// `cold_boot`: neither the #825 rail restore nor the #1176 token.
/// `newer_in_cache`: the cache already holds a newer frame (the RTC copy's, or
/// the live link's on a logger retry).
/// `rtc_says_over`: the RTC copy is intact on this boot but not servable.
/// `tail_servable`: its tail was read to the end with no gap, and its last and
/// newest frames are servable in-flight snapshots.
inline constexpr OrphanAction orphanAction(bool marked, bool cold_boot, bool newer_in_cache,
                                           bool rtc_says_over, bool tail_servable)
{
    if (marked) return OrphanAction::Nothing;
    if (cold_boot) return OrphanAction::Retire;
    if (newer_in_cache) return OrphanAction::MarkOnServe;
    if (rtc_says_over) return OrphanAction::Decline;
    return tail_servable ? OrphanAction::Seed : OrphanAction::Decline;
}

/// The #846 indexed path's test: has the OC already served, declined or
/// retired this flight_recovered_* entry? By its own marker (id and length),
/// or by the orphan marker for the same flight from before the scan indexed
/// it: the same id, its first block inside the entry's block range.
inline constexpr bool indexedAlreadyDone(uint32_t done_id, uint32_t done_bytes,
                                         const OrphanId& orphan_mark, uint32_t flight_id,
                                         uint32_t final_bytes, uint32_t start_block,
                                         uint32_t n_blocks)
{
    return (done_id == flight_id && done_bytes == final_bytes) ||
           (orphan_mark.flight_id != kUnset && orphan_mark.flight_id == flight_id &&
            orphan_mark.start_block >= start_block &&
            orphan_mark.start_block < start_block + n_blocks);
}

enum class IndexedAction : uint8_t
{
    Nothing,       // already done
    Retire,        // a cold boot: mark it, seed nothing
    MarkOnServe,   // the cache holds a newer frame: mark it once the FC is served
    Decline,       // the RTC copy says the flight is over: mark it now
    Scan,          // tail-scan it: seed from it, or decline
};

/// The #846 indexed path, for the newest stored flight when it is a
/// flight_recovered_* entry and no cut-off flight is newer.
inline constexpr IndexedAction indexedAction(bool already, bool cold_boot, bool newer_in_cache,
                                             bool rtc_says_over)
{
    if (already) return IndexedAction::Nothing;
    if (cold_boot) return IndexedAction::Retire;
    if (newer_in_cache) return IndexedAction::MarkOnServe;
    if (rtc_says_over) return IndexedAction::Decline;
    return IndexedAction::Scan;
}

}  // namespace SnapshotReseed
