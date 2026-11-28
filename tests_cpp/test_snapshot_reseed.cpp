// #1717: the OC's boot re-seed of its flight-snapshot cache after a reset of
// its own: the RTC copy (snapshot_rtc_copy.h) and what to do with a flight a
// reboot cut off, found on NAND before the deferred scan indexed it
// (snapshot_reseed_policy.h).

#include <gtest/gtest.h>

#include <cstring>

#include "snapshot_reseed_policy.h"
#include "snapshot_rtc_copy.h"

using SnapshotReseed::OrphanAction;
using SnapshotReseed::OrphanId;
using SnapshotReseed::kUnset;

namespace {

constexpr size_t kN = 232;   // the snapshot wire frame (flight_snapshot_answer::kFrameLen)
using Copy = SnapshotRtcCopy::Copy<kN>;

void fill(uint8_t* f, uint8_t seed)
{
    for (size_t i = 0; i < kN; ++i) f[i] = static_cast<uint8_t>(seed + i * 7);
}

}  // namespace

// ---------------------------------------------------------------------------
// The RTC copy
// ---------------------------------------------------------------------------

TEST(SnapshotRtcCopy, AStoredFrameReadsBackIntactWithItsAge)
{
    Copy c;
    memset(&c, 0xA5, sizeof(c));   // a power-on's garbage
    EXPECT_FALSE(SnapshotRtcCopy::intact(c));
    uint8_t f[kN];
    fill(f, 3);
    SnapshotRtcCopy::store(c, f, 5'000'000ull);
    EXPECT_TRUE(SnapshotRtcCopy::intact(c));
    EXPECT_EQ(0, memcmp(c.frame, f, kN));
    uint32_t age = 0;
    ASSERT_TRUE(SnapshotRtcCopy::ageMs(c, 7'250'000ull, age));
    EXPECT_EQ(age, 2250u);
}

TEST(SnapshotRtcCopy, ATornOrFlippedCopyIsNotIntact)
{
    Copy c;
    uint8_t f[kN];
    fill(f, 9);
    SnapshotRtcCopy::store(c, f, 1000);
    Copy frame_flip = c;
    frame_flip.frame[100] ^= 0x01;
    EXPECT_FALSE(SnapshotRtcCopy::intact(frame_flip)) << "a frame byte";
    Copy stamp_flip = c;
    stamp_flip.stamp_us ^= 1;
    EXPECT_FALSE(SnapshotRtcCopy::intact(stamp_flip)) << "the stamp is covered too";
    Copy magic_off = c;
    magic_off.magic = 0;
    EXPECT_FALSE(SnapshotRtcCopy::intact(magic_off));
    // a reset mid-store: the new frame half written, the old CRC left
    Copy torn = c;
    uint8_t g[kN];
    fill(g, 77);
    memcpy(torn.frame, g, kN / 2);
    EXPECT_FALSE(SnapshotRtcCopy::intact(torn));
}

TEST(SnapshotRtcCopy, ACounterBelowTheStampMeansAPowerOn)
{
    Copy c;
    uint8_t f[kN];
    fill(f, 1);
    SnapshotRtcCopy::store(c, f, 9'000'000ull);
    uint32_t age = 123;
    EXPECT_FALSE(SnapshotRtcCopy::ageMs(c, 8'999'999ull, age));
    EXPECT_TRUE(SnapshotRtcCopy::ageMs(c, 9'000'000ull, age));
    EXPECT_EQ(age, 0u);
}

TEST(SnapshotRtcCopy, AnAgeTooLargeForMsIsHeldAtTheTop)
{
    Copy c;
    uint8_t f[kN];
    fill(f, 1);
    SnapshotRtcCopy::store(c, f, 0);
    uint32_t age = 0;
    ASSERT_TRUE(SnapshotRtcCopy::ageMs(c, 0xFFFFFFFFull * 1000ull * 2ull, age));
    EXPECT_EQ(age, 0xFFFFFFFFu);
}

TEST(SnapshotRtcCopy, RetiredIsNotIntact)
{
    Copy c;
    uint8_t f[kN];
    fill(f, 2);
    SnapshotRtcCopy::store(c, f, 1);
    SnapshotRtcCopy::retire(c);
    EXPECT_FALSE(SnapshotRtcCopy::intact(c));
}

TEST(SnapshotRtcCopy, TheVerdictForEachBoot)
{
    using SnapshotRtcCopy::Verdict;
    using SnapshotRtcCopy::verdict;
    constexpr uint32_t ttl = 1'800'000;
    EXPECT_EQ(verdict(true, true, 500, ttl, true), Verdict::Seed);
    EXPECT_EQ(verdict(true, true, ttl - 1, ttl, true), Verdict::Seed);
    EXPECT_EQ(verdict(false, true, 500, ttl, true), Verdict::NotResuming) << "not resuming a flight";
    EXPECT_EQ(verdict(false, false, 0, ttl, false), Verdict::NotResuming);
    EXPECT_EQ(verdict(true, false, 0, ttl, false), Verdict::Absent) << "not intact, or a power-on";
    // intact on a boot resuming a flight, but not servable: the flight is over
    EXPECT_EQ(verdict(true, true, ttl, ttl, true), Verdict::Over) << "as old as the TTL";
    EXPECT_EQ(verdict(true, true, 500, ttl, false), Verdict::Over) << "the FC's LANDED clear";
}

// ---------------------------------------------------------------------------
// The cut-off flight on NAND
// ---------------------------------------------------------------------------

TEST(SnapshotReseed, AnOrphanIsTheSameOnlyByIdFirstBlockAndLastPage)
{
    const OrphanId o{7, 120, 4400};
    EXPECT_TRUE(SnapshotReseed::sameOrphan(o, OrphanId{7, 120, 4400}));
    EXPECT_FALSE(SnapshotReseed::sameOrphan(o, OrphanId{8, 120, 4400})) << "id";
    EXPECT_FALSE(SnapshotReseed::sameOrphan(o, OrphanId{7, 121, 4400})) << "first block";
    EXPECT_FALSE(SnapshotReseed::sameOrphan(o, OrphanId{7, 120, 4401})) << "last page";
    EXPECT_FALSE(SnapshotReseed::sameOrphan(OrphanId{}, OrphanId{})) << "an unwritten marker";
}

TEST(SnapshotReseed, TheActionForEachBoot)
{
    using SnapshotReseed::orphanAction;
    // (marked, cold_boot, newer_in_cache, rtc_says_over, tail_servable)
    EXPECT_EQ(orphanAction(true, false, false, false, true), OrphanAction::Nothing) << "already marked";
    EXPECT_EQ(orphanAction(true, true, false, false, true), OrphanAction::Nothing);
    EXPECT_EQ(orphanAction(false, true, false, false, true), OrphanAction::Retire) << "a cold boot never seeds";
    EXPECT_EQ(orphanAction(false, true, true, false, true), OrphanAction::Retire);
    EXPECT_EQ(orphanAction(false, false, true, false, true), OrphanAction::MarkOnServe)
        << "the RTC copy is newer than the NAND tail";
    EXPECT_EQ(orphanAction(false, false, true, false, false), OrphanAction::MarkOnServe);
    EXPECT_EQ(orphanAction(false, false, false, true, true), OrphanAction::Decline)
        << "the RTC copy, newer than the tail, says the flight is over";
    EXPECT_EQ(orphanAction(false, false, false, false, true), OrphanAction::Seed);
    EXPECT_EQ(orphanAction(false, false, false, false, false), OrphanAction::Decline);
}

TEST(SnapshotReseed, TheIndexedPathHonorsTheOrphanMarkerForTheSameFlight)
{
    using SnapshotReseed::indexedAlreadyDone;
    const OrphanId none{};
    // its own marker: id and length
    EXPECT_TRUE(indexedAlreadyDone(7, 409600, none, 7, 409600, 120, 4));
    EXPECT_FALSE(indexedAlreadyDone(7, 409600, none, 7, 413680, 120, 4));
    EXPECT_FALSE(indexedAlreadyDone(6, 409600, none, 7, 409600, 120, 4));
    // the orphan marker from before the scan indexed it: the id, and its
    // first block inside the entry's range (which can start lower, on blank
    // blocks of the scan's run)
    const OrphanId mark{7, 120, 99};
    EXPECT_TRUE(indexedAlreadyDone(0, 0, mark, 7, 409600, 120, 4));
    EXPECT_TRUE(indexedAlreadyDone(0, 0, mark, 7, 409600, 80, 44)) << "the run starts lower";
    EXPECT_FALSE(indexedAlreadyDone(0, 0, mark, 7, 409600, 80, 40)) << "ends just below it";
    EXPECT_FALSE(indexedAlreadyDone(0, 0, mark, 7, 409600, 121, 4)) << "another flight with a reused id";
    EXPECT_FALSE(indexedAlreadyDone(0, 0, mark, 8, 409600, 120, 4));
    // the NVS defaults never match
    EXPECT_FALSE(indexedAlreadyDone(0, 0, none, kUnset, 0, kUnset, 0));
}

TEST(SnapshotReseed, TheIndexedPathsActionForEachBoot)
{
    using SnapshotReseed::IndexedAction;
    using SnapshotReseed::indexedAction;
    // (already, cold_boot, newer_in_cache, rtc_says_over)
    EXPECT_EQ(indexedAction(true, false, false, false), IndexedAction::Nothing);
    EXPECT_EQ(indexedAction(true, true, true, true), IndexedAction::Nothing);
    EXPECT_EQ(indexedAction(false, true, false, false), IndexedAction::Retire);
    EXPECT_EQ(indexedAction(false, true, true, true), IndexedAction::Retire);
    EXPECT_EQ(indexedAction(false, false, true, false), IndexedAction::MarkOnServe);
    EXPECT_EQ(indexedAction(false, false, true, true), IndexedAction::MarkOnServe);
    EXPECT_EQ(indexedAction(false, false, false, true), IndexedAction::Decline);
    EXPECT_EQ(indexedAction(false, false, false, false), IndexedAction::Scan);
}
