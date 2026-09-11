// #1235 items 3 and 4 — the flight-log storage verdict on the pre-launch
// scorecard (#281/#278, #303).
//
// Two things are pinned here.  The free-block input has to be the FLIGHT
// REGION's count: the bitmap's whole-die count carries the 32 LFS pre-region
// blocks and the 4 metadata blocks, which are never allocated and always read
// FREE, so it sits 36 high and moved the BAD trip point from 80 genuinely free
// blocks to 44 (item 3).  And a V9/V10 PSRAM ring that fell back to the 64 KB
// internal ring has to reach the verdict at all — the only other notice was a
// boot-log line on a console that light sleep silences (item 4).
//
// The table is otherwise the pre-#1235 ocStorageHealth() moved verbatim into
// storage_health_policy.h, so its older fold-ins (#566, #1127, #281, #826) are
// pinned too — they were never host-tested before.

#include <gtest/gtest.h>

#include "storage_health_policy.h"

using StorageHealthPolicy::Inputs;
using StorageHealthPolicy::kIndexHeadroomEntries;
using StorageHealthPolicy::psramRingMissing;
using StorageHealthPolicy::verdict;

namespace {

// The shipping V9/V10 numbers: prealloc 80, index cap 64, a healthy board with
// plenty of room.  Every test starts here and changes one thing.
constexpr uint32_t kPrealloc = 80;
constexpr size_t   kIndexCap = 64;

Inputs healthy()
{
    Inputs in;
    in.flightlog_initialized = true;
    in.recovery_failed       = false;
    in.region_free_blocks    = 1000;
    in.prealloc_blocks       = kPrealloc;
    in.nand_prog_fail        = 0;
    in.index_used            = 3;
    in.index_capacity        = kIndexCap;
    in.mram_probe_failed     = false;
    in.psram_ring_expected   = true;    // a V9/V10 board
    in.mram_enabled          = false;
    in.ring_in_psram         = true;    // ...whose ring landed where it should
    return in;
}

}  // namespace

// ── item 3: the region count is what the thresholds were written for ────────

TEST(StorageHealthPolicy, HealthyBoardIsOk)
{
    EXPECT_EQ(verdict(healthy()), SH_OK);
}

TEST(StorageHealthPolicy, TripPointsAreInFlightRegionBlocks)
{
    Inputs in = healthy();
    in.region_free_blocks = 2 * kPrealloc;      EXPECT_EQ(verdict(in), SH_OK);
    in.region_free_blocks = 2 * kPrealloc - 1;  EXPECT_EQ(verdict(in), SH_DEGRADED);
    in.region_free_blocks = kPrealloc;          EXPECT_EQ(verdict(in), SH_DEGRADED);
    in.region_free_blocks = kPrealloc - 1;      EXPECT_EQ(verdict(in), SH_BAD);
    in.region_free_blocks = 0;                  EXPECT_EQ(verdict(in), SH_BAD);
}

TEST(StorageHealthPolicy, TheWholeDieCountWouldHaveShiftedBothTripPointsBy36)
{
    // What the old input reported for a region with N free blocks was N + 36
    // (32 LFS blocks below the region, 4 metadata blocks above it).  Feeding
    // that number to the same thresholds is the defect; stated as arithmetic
    // so the shift is visible rather than implied.
    constexpr uint32_t kOutsideRegion = 32 + 4;

    Inputs in = healthy();
    // 44 genuinely free blocks: no room for a flight.  The die count read 80,
    // "room for exactly one" — DEGRADED on a board that cannot record.
    in.region_free_blocks = 44;
    EXPECT_EQ(verdict(in), SH_BAD);
    in.region_free_blocks = 44 + kOutsideRegion;
    EXPECT_EQ(verdict(in), SH_DEGRADED) << "the verdict the old input produced";

    // 124 free: room for one flight, not two.  The die count read 160 — OK.
    in.region_free_blocks = 124;
    EXPECT_EQ(verdict(in), SH_DEGRADED);
    in.region_free_blocks = 124 + kOutsideRegion;
    EXPECT_EQ(verdict(in), SH_OK) << "the verdict the old input produced";
}

// ── item 4: the PSRAM ring fallback reaches the scorecard ───────────────────

TEST(StorageHealthPolicy, PsramRingFallbackDegrades)
{
    Inputs in = healthy();
    in.ring_in_psram = false;   // heap_caps_malloc(SPIRAM) returned null; 64 KB internal ring
    EXPECT_TRUE(psramRingMissing(in));
    EXPECT_EQ(verdict(in), SH_DEGRADED);
}

TEST(StorageHealthPolicy, PsramRingFallbackIsDegradedNotBad)
{
    // Frames still reach the NAND — the shock absorber is 8x smaller, nothing
    // is lost yet.  Same reasoning as the #826 MRAM fold-in.
    Inputs in = healthy();
    in.ring_in_psram = false;
    EXPECT_NE(verdict(in), SH_BAD);
}

TEST(StorageHealthPolicy, BoardsThatNeverExpectedPsramAreUntouched)
{
    // V7/V8: RING_IN_PSRAM is false, the ring is MRAM (or internal RAM by
    // design).  isRingInPsram() is false there and must not read as a fault.
    Inputs in = healthy();
    in.psram_ring_expected = false;
    in.ring_in_psram       = false;
    in.mram_enabled        = true;
    EXPECT_FALSE(psramRingMissing(in));
    EXPECT_EQ(verdict(in), SH_OK);

    in.mram_enabled = false;    // V7/V8 with MRAM_CS wired but RING_IN_PSRAM false
    EXPECT_FALSE(psramRingMissing(in));
    EXPECT_EQ(verdict(in), SH_OK);
}

TEST(StorageHealthPolicy, PsramFlagIsMeaninglessWhileAnMramRingIsInUse)
{
    // TR_LogToFlash::isRingInPsram() is documented meaningless while
    // isMramEnabled() — there is no ring_buf_ at all.  A misconfigured build
    // that expects PSRAM but found an MRAM must not report a missing ring.
    Inputs in = healthy();
    in.mram_enabled  = true;
    in.ring_in_psram = false;
    EXPECT_FALSE(psramRingMissing(in));
    EXPECT_EQ(verdict(in), SH_OK);
}

TEST(StorageHealthPolicy, PsramFallbackNeverLiftsAWorseVerdict)
{
    Inputs in = healthy();
    in.ring_in_psram = false;
    in.region_free_blocks = kPrealloc - 1;
    EXPECT_EQ(verdict(in), SH_BAD);
    in.region_free_blocks = kPrealloc;
    EXPECT_EQ(verdict(in), SH_DEGRADED);
}

// ── the older fold-ins, pinned for the first time ───────────────────────────

TEST(StorageHealthPolicy, UninitializedFlightLogIsBadNeverNa)
{
    // #566: every frame is dropped; NA would hide the Storage row and drop it
    // from the go/no-go.
    Inputs in = healthy();
    in.flightlog_initialized = false;
    EXPECT_EQ(verdict(in), SH_BAD);
}

TEST(StorageHealthPolicy, FailedRecoveryScanIsBad)
{
    // #1127: the surface stays initialized for download/delete, but nothing
    // new can be logged this boot.
    Inputs in = healthy();
    in.recovery_failed = true;
    EXPECT_EQ(verdict(in), SH_BAD);
}

TEST(StorageHealthPolicy, WriteFailuresAreBadRegardlessOfSpace)
{
    Inputs in = healthy();
    in.nand_prog_fail = 1;
    EXPECT_EQ(verdict(in), SH_BAD);
}

TEST(StorageHealthPolicy, IndexCapacityIsAnIndependentLimit)
{
    // #281: a full index cannot record the flight even with free blocks.
    Inputs in = healthy();
    in.index_used = kIndexCap;
    EXPECT_EQ(verdict(in), SH_BAD);
    in.index_used = kIndexCap - kIndexHeadroomEntries;
    EXPECT_EQ(verdict(in), SH_DEGRADED);
    in.index_used = kIndexCap - kIndexHeadroomEntries - 1;
    EXPECT_EQ(verdict(in), SH_OK);
}

TEST(StorageHealthPolicy, MramProbeFailureDegrades)
{
    // #826: a fitted part that did not answer.
    Inputs in = healthy();
    in.psram_ring_expected = false;   // a V8: MRAM board
    in.ring_in_psram       = false;
    in.mram_enabled        = false;   // ...that fell back to the RAM ring
    in.mram_probe_failed   = true;
    EXPECT_EQ(verdict(in), SH_DEGRADED);
    in.region_free_blocks = kPrealloc - 1;
    EXPECT_EQ(verdict(in), SH_BAD) << "never lifts a worse verdict";
}

TEST(StorageHealthPolicy, UnconfiguredPreallocIsNa)
{
    // shStorageState's own NA: begin() rejects prealloc 0, so this is only
    // reachable in a test, but the table must not turn it into a verdict.
    Inputs in = healthy();
    in.prealloc_blocks = 0;
    EXPECT_EQ(verdict(in), SH_NA);
}
