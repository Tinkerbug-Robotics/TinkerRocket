#include <gtest/gtest.h>

#include "BlockStateBitmap.h"
#include "FlightIndex.h"
#include "TR_FlightLog.h"
#include "TR_NandBackend_esp.h"
#include "fakes/fake_nand_backend.h"
#include "fakes/memory_bitmap_store.h"

#include <cstring>
#include <vector>

using tr_flightlog::BLOCK_ALLOCATED;
using tr_flightlog::BLOCK_BAD;
using tr_flightlog::BLOCK_FREE;
using tr_flightlog::BlockStateBitmap;
using tr_flightlog::FlightIndex;
using tr_flightlog::FlightIndexEntry;
using tr_flightlog::FLGT_MAGIC;
using tr_flightlog::FPAG_MAGIC;
using tr_flightlog::PageHeader;
using tr_flightlog::NAND_BLOCK_COUNT;
using tr_flightlog::NAND_PAGES_PER_BLK;
using tr_flightlog::NAND_PAGE_SIZE;
using tr_flightlog::Status;
using tr_flightlog::TR_FlightLog;
using tr_flightlog_test::FakeNandBackend;
using tr_flightlog_test::MemoryBitmapStore;

namespace {

// Helper: build a filled-in FlightIndexEntry for tests.
FlightIndexEntry makeEntry(uint32_t id, const char* filename,
                           uint16_t start_block, uint16_t n_blocks,
                           uint32_t final_bytes) {
    FlightIndexEntry e{};
    e.magic       = FLGT_MAGIC;
    e.flight_id   = id;
    std::strncpy(e.filename, filename, sizeof(e.filename) - 1);
    e.start_block = start_block;
    e.n_blocks    = n_blocks;
    e.final_bytes = final_bytes;
    e.crc16       = 0;  // per-entry CRC not used by FlightIndex dual-copy
    return e;
}

constexpr uint32_t META_A = 1020;
constexpr uint32_t META_B = 1021;

}  // namespace

// ================================================================
// FakeNandBackend sanity — every later test depends on these semantics
// ================================================================

TEST(FakeNandBackend, FreshChipIsAllOnes) {
    FakeNandBackend nand;
    std::vector<uint8_t> buf(NAND_PAGE_SIZE);
    ASSERT_TRUE(nand.readPage(0, 0, buf.data()));
    for (auto b : buf) EXPECT_EQ(b, 0xFF);
}

TEST(FakeNandBackend, ProgramOnlyClearsBits) {
    // Real NAND: programming a page without prior erase can only set 1->0.
    // Writing 0xAA then 0x55 to the same page should yield 0xAA & 0x55 = 0x00.
    FakeNandBackend nand;
    std::vector<uint8_t> aa(NAND_PAGE_SIZE, 0xAA);
    std::vector<uint8_t> five(NAND_PAGE_SIZE, 0x55);

    ASSERT_TRUE(nand.programPage(10, 0, aa.data()));
    ASSERT_TRUE(nand.programPage(10, 0, five.data()));

    std::vector<uint8_t> out(NAND_PAGE_SIZE);
    ASSERT_TRUE(nand.readPage(10, 0, out.data()));
    for (auto b : out) EXPECT_EQ(b, 0x00);
}

TEST(FakeNandBackend, EraseResetsBlockToOnes) {
    FakeNandBackend nand;
    std::vector<uint8_t> zero(NAND_PAGE_SIZE, 0x00);
    // Fill several pages in a block with zeros.
    for (uint32_t p = 0; p < 4; ++p) {
        ASSERT_TRUE(nand.programPage(42, p, zero.data()));
    }
    ASSERT_TRUE(nand.eraseBlock(42));

    std::vector<uint8_t> out(NAND_PAGE_SIZE);
    for (uint32_t p = 0; p < NAND_PAGES_PER_BLK; ++p) {
        ASSERT_TRUE(nand.readPage(42, p, out.data()));
        for (auto b : out) EXPECT_EQ(b, 0xFF);
    }
}

TEST(FakeNandBackend, BadBlockInjectionPersists) {
    FakeNandBackend nand;
    EXPECT_FALSE(nand.isBlockBad(7));
    nand.injectFactoryBadBlock(7);
    EXPECT_TRUE(nand.isBlockBad(7));
}

TEST(FakeNandBackend, MarkBlockBadIsSticky) {
    FakeNandBackend nand;
    EXPECT_FALSE(nand.isBlockBad(99));
    ASSERT_TRUE(nand.markBlockBad(99));
    EXPECT_TRUE(nand.isBlockBad(99));
}

TEST(FakeNandBackend, ProgramFailOnceFiresOnlyOnce) {
    FakeNandBackend nand;
    nand.injectProgramFailOnce(50, 5);

    std::vector<uint8_t> buf(NAND_PAGE_SIZE, 0xA5);
    EXPECT_FALSE(nand.programPage(50, 5, buf.data()));
    EXPECT_TRUE(nand.programPage(50, 5, buf.data()));  // second call succeeds
}

TEST(FakeNandBackend, ReadErrorPersistent) {
    FakeNandBackend nand;
    nand.injectReadErrorPersistent(100, 0);

    std::vector<uint8_t> out(NAND_PAGE_SIZE);
    EXPECT_FALSE(nand.readPage(100, 0, out.data()));
    EXPECT_FALSE(nand.readPage(100, 0, out.data()));   // still fails
    EXPECT_TRUE(nand.readPage(100, 1, out.data()));    // other pages OK
}

TEST(FakeNandBackend, OutOfRangeAddressesFail) {
    FakeNandBackend nand;
    std::vector<uint8_t> buf(NAND_PAGE_SIZE);
    EXPECT_FALSE(nand.readPage(NAND_BLOCK_COUNT, 0, buf.data()));
    EXPECT_FALSE(nand.readPage(0, NAND_PAGES_PER_BLK, buf.data()));
    EXPECT_FALSE(nand.programPage(NAND_BLOCK_COUNT, 0, buf.data()));
    EXPECT_FALSE(nand.eraseBlock(NAND_BLOCK_COUNT));
}

TEST(FakeNandBackend, CountersAdvance) {
    FakeNandBackend nand;
    std::vector<uint8_t> buf(NAND_PAGE_SIZE, 0x33);

    nand.programPage(0, 0, buf.data());
    nand.programPage(0, 1, buf.data());
    nand.readPage(0, 0, buf.data());
    nand.eraseBlock(0);

    EXPECT_EQ(nand.programCount(), 2u);
    EXPECT_EQ(nand.readCount(), 1u);
    EXPECT_EQ(nand.eraseCount(), 1u);
}

TEST(FakeNandBackend, ResetWipesEverything) {
    FakeNandBackend nand;
    std::vector<uint8_t> buf(NAND_PAGE_SIZE, 0x00);
    nand.programPage(5, 0, buf.data());
    nand.markBlockBad(5);
    nand.injectProgramFailOnce(6, 0);

    nand.reset();

    std::vector<uint8_t> out(NAND_PAGE_SIZE);
    ASSERT_TRUE(nand.readPage(5, 0, out.data()));
    for (auto b : out) EXPECT_EQ(b, 0xFF);
    EXPECT_FALSE(nand.isBlockBad(5));
    EXPECT_EQ(nand.programCount(), 0u);
}

// ================================================================
// TR_FlightLog scaffold — initialization only. Functional tests land as
// each subsequent step fills in behavior.
// ================================================================

TEST(TRFlightLogScaffold, BeginAcceptsDefaultConfig) {
    FakeNandBackend nand;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    EXPECT_EQ(fl.begin(nand, cfg), Status::Ok);
    EXPECT_TRUE(fl.isInitialized());
    EXPECT_EQ(fl.config().prealloc_blocks, 256);
}

TEST(TRFlightLogScaffold, BeginRejectsInvertedRegion) {
    FakeNandBackend nand;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    cfg.flight_region_start = 500;
    cfg.flight_region_end   = 100;
    EXPECT_EQ(fl.begin(nand, cfg), Status::OutOfRange);
    EXPECT_FALSE(fl.isInitialized());
}

TEST(TRFlightLogScaffold, BeginRejectsRegionBeyondChip) {
    FakeNandBackend nand;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    cfg.flight_region_end = NAND_BLOCK_COUNT + 10;
    EXPECT_EQ(fl.begin(nand, cfg), Status::OutOfRange);
}

TEST(TRFlightLogScaffold, BeginRejectsPreallocLargerThanRegion) {
    FakeNandBackend nand;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    cfg.flight_region_start = 32;
    cfg.flight_region_end   = 64;      // only 32 blocks available
    cfg.prealloc_blocks     = 256;     // asking for more than fits
    EXPECT_EQ(fl.begin(nand, cfg), Status::OutOfRange);
}

TEST(TRFlightLogScaffold, HotPathMethodsReturnNotInitializedBeforeBegin) {
    TR_FlightLog fl;
    uint32_t flight_id = 0;
    uint8_t buf[NAND_PAGE_SIZE] = {};
    size_t out_len = 0;

    EXPECT_EQ(fl.prepareFlight(flight_id), Status::NotInitialized);
    EXPECT_EQ(fl.writePage(buf), Status::NotInitialized);
    EXPECT_EQ(fl.finalizeFlight("x", 0), Status::NotInitialized);
    EXPECT_EQ(fl.readFlightPage("x", 0, buf, sizeof(buf), out_len), Status::NotInitialized);
    EXPECT_EQ(fl.deleteFlight("x"), Status::NotInitialized);
    EXPECT_EQ(fl.renameFlight("x", "y"), Status::NotInitialized);
    EXPECT_EQ(fl.markBlockBad(5), Status::NotInitialized);
}

TEST(TRFlightLogTypes, FlightIndexEntryLayoutIsStable) {
    // This is a wire/storage format guard. Changes here force a format bump.
    EXPECT_EQ(sizeof(tr_flightlog::FlightIndexEntry), 48u);
    EXPECT_EQ(sizeof(tr_flightlog::PageHeader), 16u);
    EXPECT_EQ(tr_flightlog::FLGT_MAGIC, 0x54474C46u);  // 'FLGT'
    EXPECT_EQ(tr_flightlog::FPAG_MAGIC, 0x47415046u);  // 'FPAG'
}

// ================================================================
// BlockStateBitmap — 3-state (FREE / ALLOCATED / BAD), 2 bits/block
// ================================================================

TEST(BlockStateBitmap, DefaultIsAllFree) {
    BlockStateBitmap bm;
    EXPECT_EQ(bm.countInState(BLOCK_FREE), NAND_BLOCK_COUNT);
    EXPECT_EQ(bm.countInState(BLOCK_BAD), 0u);
    EXPECT_EQ(bm.countInState(BLOCK_ALLOCATED), 0u);
}

TEST(BlockStateBitmap, GetSetSingleBlocks) {
    BlockStateBitmap bm;
    bm.set(0, BLOCK_BAD);
    bm.set(1, BLOCK_ALLOCATED);
    bm.set(2, BLOCK_FREE);
    bm.set(1023, BLOCK_BAD);

    EXPECT_EQ(bm.get(0), BLOCK_BAD);
    EXPECT_EQ(bm.get(1), BLOCK_ALLOCATED);
    EXPECT_EQ(bm.get(2), BLOCK_FREE);
    EXPECT_EQ(bm.get(3), BLOCK_FREE);
    EXPECT_EQ(bm.get(1023), BLOCK_BAD);
}

TEST(BlockStateBitmap, SetAcrossByteBoundaryDoesNotClobberNeighbors) {
    BlockStateBitmap bm;
    // Blocks 0..3 share byte[0]; block 4 is first in byte[1]. Make sure
    // setting each state independently doesn't leak into neighbors.
    bm.set(3, BLOCK_ALLOCATED);
    bm.set(4, BLOCK_BAD);
    EXPECT_EQ(bm.get(3), BLOCK_ALLOCATED);
    EXPECT_EQ(bm.get(4), BLOCK_BAD);
    EXPECT_EQ(bm.get(2), BLOCK_FREE);
    EXPECT_EQ(bm.get(5), BLOCK_FREE);
}

TEST(BlockStateBitmap, OutOfRangeGetReturnsBad) {
    BlockStateBitmap bm;
    EXPECT_EQ(bm.get(NAND_BLOCK_COUNT), BLOCK_BAD);
    EXPECT_EQ(bm.get(9999), BLOCK_BAD);
}

TEST(BlockStateBitmap, MarkAllocatedRangeSkipsBadBlocks) {
    BlockStateBitmap bm;
    bm.set(105, BLOCK_BAD);
    size_t touched = bm.markAllocatedRange(100, 10);
    EXPECT_EQ(touched, 9u);  // 10 blocks, 1 bad skipped
    EXPECT_EQ(bm.get(104), BLOCK_ALLOCATED);
    EXPECT_EQ(bm.get(105), BLOCK_BAD);       // sticky
    EXPECT_EQ(bm.get(106), BLOCK_ALLOCATED);
}

TEST(BlockStateBitmap, MarkFreeRangePreservesBad) {
    BlockStateBitmap bm;
    bm.markAllocatedRange(200, 20);
    bm.set(210, BLOCK_BAD);
    size_t touched = bm.markFreeRange(200, 20);
    EXPECT_EQ(touched, 19u);
    EXPECT_EQ(bm.get(210), BLOCK_BAD);
    for (uint32_t b : {200u, 205u, 219u}) EXPECT_EQ(bm.get(b), BLOCK_FREE);
}

TEST(BlockStateBitmap, FindContiguousFreeHappyPath) {
    BlockStateBitmap bm;
    uint32_t start = 0;
    ASSERT_TRUE(bm.findContiguousFree(256, 32, 1020, start));
    EXPECT_EQ(start, 32u);
}

TEST(BlockStateBitmap, FindContiguousFreeSkipsAllocatedAndBad) {
    BlockStateBitmap bm;
    bm.markAllocatedRange(32, 100);          // blocks 32..131 occupied
    bm.set(200, BLOCK_BAD);                  // splits later run
    uint32_t start = 0;
    // First 256-block run in [32, 1020) must start after block 200, so at 201.
    // But note 132..199 is only 68 free blocks — too small for 256.
    // Next candidate run is [201 .. 1020) = 819 blocks. Pick starts at 201.
    ASSERT_TRUE(bm.findContiguousFree(256, 32, 1020, start));
    EXPECT_EQ(start, 201u);
}

TEST(BlockStateBitmap, FindContiguousFreeReturnsFalseWhenNoFit) {
    BlockStateBitmap bm;
    // Occupy every even block — longest free run is 1 block.
    for (uint32_t b = 0; b < NAND_BLOCK_COUNT; b += 2) {
        bm.set(b, BLOCK_ALLOCATED);
    }
    uint32_t start = 42;
    EXPECT_FALSE(bm.findContiguousFree(2, 0, NAND_BLOCK_COUNT, start));
    EXPECT_EQ(start, 42u);  // unchanged on failure
}

TEST(BlockStateBitmap, SerializeRoundTrip) {
    BlockStateBitmap a;
    a.markAllocatedRange(32, 256);
    a.set(100, BLOCK_BAD);
    a.set(1023, BLOCK_BAD);

    uint8_t buf[BlockStateBitmap::SERIALIZED_SIZE] = {};
    a.serializeTo(buf, sizeof(buf));

    BlockStateBitmap b;
    ASSERT_TRUE(b.deserializeFrom(buf, sizeof(buf)));

    for (uint32_t blk = 0; blk < NAND_BLOCK_COUNT; ++blk) {
        EXPECT_EQ(a.get(blk), b.get(blk)) << "mismatch at block " << blk;
    }
}

TEST(BlockStateBitmap, DeserializeRejectsShortBuffer) {
    BlockStateBitmap bm;
    uint8_t buf[BlockStateBitmap::SERIALIZED_SIZE - 1] = {};
    EXPECT_FALSE(bm.deserializeFrom(buf, sizeof(buf)));
}

TEST(BlockStateBitmap, SerializedSizeIs256Bytes) {
    // Storage layout guard: 1024 blocks * 2 bits = 256 bytes.
    EXPECT_EQ(BlockStateBitmap::SERIALIZED_SIZE, 256u);
}

// ================================================================
// TR_FlightLog + BlockStateBitmap integration
// ================================================================

TEST(TRFlightLogBitmap, FreshBeginSeedsBadBlocksFromBackend) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    nand.injectFactoryBadBlock(42);
    nand.injectFactoryBadBlock(500);

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);

    EXPECT_EQ(fl.bitmap().get(42), BLOCK_BAD);
    EXPECT_EQ(fl.bitmap().get(500), BLOCK_BAD);
    EXPECT_EQ(fl.bitmap().get(0), BLOCK_FREE);
}

TEST(TRFlightLogBitmap, FreshBeginPersistsSeededBitmap) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    nand.injectFactoryBadBlock(7);

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);

    EXPECT_GE(store.saveCount(), 1u);
    EXPECT_EQ(store.raw().size(), BlockStateBitmap::SERIALIZED_SIZE);
}

TEST(TRFlightLogBitmap, RebootRestoresBitmapFromStore) {
    FakeNandBackend nand;
    MemoryBitmapStore store;

    // First boot — factory bad block at 300.
    nand.injectFactoryBadBlock(300);
    {
        TR_FlightLog fl;
        ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
        ASSERT_EQ(fl.markBlockBad(301), Status::Ok);  // runtime-discovered bad
    }

    // Simulate reboot — backend no longer remembers that 301 was bad, only the
    // bitmap did. Clear the factory-bad injection to prove we're loading
    // from the store, not rescanning.
    nand.reset();

    TR_FlightLog fl2;
    size_t saves_before = store.saveCount();
    ASSERT_EQ(fl2.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);

    EXPECT_EQ(fl2.bitmap().get(300), BLOCK_BAD);
    EXPECT_EQ(fl2.bitmap().get(301), BLOCK_BAD);
    EXPECT_EQ(store.loadCount(), 1u);
    // Loading a valid saved bitmap should not trigger a fresh save.
    EXPECT_EQ(store.saveCount(), saves_before);
}

TEST(TRFlightLogBitmap, MarkBlockBadPersistsImmediately) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);

    size_t before = store.saveCount();
    ASSERT_EQ(fl.markBlockBad(555), Status::Ok);
    EXPECT_EQ(store.saveCount(), before + 1);
    EXPECT_EQ(fl.bitmap().get(555), BLOCK_BAD);
    EXPECT_TRUE(nand.isBlockBad(555));
}

TEST(TRFlightLogBitmap, VolatileModeWorksWithNullStore) {
    FakeNandBackend nand;
    nand.injectFactoryBadBlock(11);
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, /*store=*/nullptr), Status::Ok);
    EXPECT_EQ(fl.bitmap().get(11), BLOCK_BAD);
    EXPECT_EQ(fl.markBlockBad(12), Status::Ok);  // no crash, no-op on persist
}

// ================================================================
// FlightIndex — in-memory collection
// ================================================================

TEST(FlightIndex, AppendAndFindByFilename) {
    FlightIndex idx;
    EXPECT_EQ(idx.append(makeEntry(1, "flight_001.bin", 32, 256, 1000)), Status::Ok);
    EXPECT_EQ(idx.append(makeEntry(2, "flight_002.bin", 288, 256, 2000)), Status::Ok);
    EXPECT_EQ(idx.size(), 2u);

    auto* e = idx.findByFilename("flight_002.bin");
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->flight_id, 2u);
    EXPECT_EQ(e->start_block, 288u);

    EXPECT_EQ(idx.findByFilename("nope.bin"), nullptr);
}

TEST(FlightIndex, NextFlightIdIsMaxPlusOne) {
    FlightIndex idx;
    EXPECT_EQ(idx.nextFlightId(), 1u);
    idx.append(makeEntry(5, "a.bin", 0, 1, 0));
    idx.append(makeEntry(10, "b.bin", 0, 1, 0));
    idx.append(makeEntry(7, "c.bin", 0, 1, 0));
    EXPECT_EQ(idx.nextFlightId(), 11u);
}

TEST(FlightIndex, RemoveByFilenameShiftsEntries) {
    FlightIndex idx;
    idx.append(makeEntry(1, "a.bin", 0, 1, 0));
    idx.append(makeEntry(2, "b.bin", 0, 1, 0));
    idx.append(makeEntry(3, "c.bin", 0, 1, 0));

    EXPECT_TRUE(idx.removeByFilename("b.bin"));
    EXPECT_EQ(idx.size(), 2u);
    EXPECT_EQ(idx.at(0).flight_id, 1u);
    EXPECT_EQ(idx.at(1).flight_id, 3u);

    EXPECT_FALSE(idx.removeByFilename("b.bin"));  // already gone
}

TEST(FlightIndex, RenameChangesFilename) {
    FlightIndex idx;
    idx.append(makeEntry(1, "flight_001.bin", 0, 1, 0));
    EXPECT_TRUE(idx.rename("flight_001.bin", "flight_20260422_143000.bin"));

    auto* e = idx.findByFilename("flight_20260422_143000.bin");
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->flight_id, 1u);
    EXPECT_EQ(idx.findByFilename("flight_001.bin"), nullptr);
}

TEST(FlightIndex, AppendUpToMaxEntries) {
    FlightIndex idx;
    for (size_t i = 0; i < FlightIndex::MAX_ENTRIES; ++i) {
        char name[20];
        std::snprintf(name, sizeof(name), "f_%zu.bin", i);
        EXPECT_EQ(idx.append(makeEntry(static_cast<uint32_t>(i + 1),
                                       name, 0, 1, 0)), Status::Ok);
    }
    EXPECT_EQ(idx.size(), FlightIndex::MAX_ENTRIES);

    EXPECT_EQ(idx.append(makeEntry(999, "overflow.bin", 0, 1, 0)), Status::NoSpace);
}

// ================================================================
// FlightIndex — persistence via dual-copy metadata
// ================================================================

TEST(FlightIndexPersist, FreshChipLoadsEmpty) {
    FakeNandBackend nand;
    FlightIndex idx;
    EXPECT_EQ(idx.load(nand, META_A, META_B), Status::Ok);
    EXPECT_EQ(idx.size(), 0u);
    EXPECT_EQ(idx.sequence(), 0u);
}

TEST(FlightIndexPersist, SaveThenLoadRoundTrip) {
    FakeNandBackend nand;
    FlightIndex idx;
    idx.append(makeEntry(1, "flight_001.bin", 32, 256, 1234));
    idx.append(makeEntry(2, "flight_002.bin", 288, 256, 5678));
    ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);

    FlightIndex loaded;
    ASSERT_EQ(loaded.load(nand, META_A, META_B), Status::Ok);
    EXPECT_EQ(loaded.size(), 2u);
    EXPECT_EQ(loaded.at(0).flight_id, 1u);
    EXPECT_EQ(loaded.at(1).flight_id, 2u);
    EXPECT_STREQ(loaded.at(0).filename, "flight_001.bin");
    EXPECT_STREQ(loaded.at(1).filename, "flight_002.bin");
}

TEST(FlightIndexPersist, SequenceIncrementsOnEachSave) {
    FakeNandBackend nand;
    FlightIndex idx;
    idx.append(makeEntry(1, "a.bin", 0, 1, 0));
    ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);
    uint32_t seq1 = idx.sequence();
    ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);
    uint32_t seq2 = idx.sequence();
    ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);
    uint32_t seq3 = idx.sequence();
    EXPECT_LT(seq1, seq2);
    EXPECT_LT(seq2, seq3);
}

TEST(FlightIndexPersist, DualCopyWritesAlternateBlocks) {
    // After repeated saves, both metadata blocks should have been erased at
    // least once. This guards the "write the older copy" logic.
    FakeNandBackend nand;
    FlightIndex idx;
    idx.append(makeEntry(1, "a.bin", 0, 1, 0));
    for (int i = 0; i < 4; ++i) {
        ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);
    }
    EXPECT_GE(nand.eraseCount(), 2u);
}

TEST(FlightIndexPersist, LoadFallsBackToShadowWhenActiveCorrupt) {
    FakeNandBackend nand;
    FlightIndex idx;
    idx.append(makeEntry(1, "a.bin", 32, 1, 100));
    ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);           // lands in A
    idx.append(makeEntry(2, "b.bin", 33, 1, 200));
    ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);           // lands in B

    // Corrupt the newer copy (B): erase it, leaving it all-0xFF.
    nand.eraseBlock(META_B);

    FlightIndex loaded;
    ASSERT_EQ(loaded.load(nand, META_A, META_B), Status::Ok);
    // A had only entry 1; that's what we should recover.
    EXPECT_EQ(loaded.size(), 1u);
    EXPECT_EQ(loaded.at(0).flight_id, 1u);
}

TEST(FlightIndexPersist, LoadReturnsEmptyWhenBothCopiesInvalid) {
    FakeNandBackend nand;
    FlightIndex idx;
    idx.append(makeEntry(1, "a.bin", 0, 1, 0));
    ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);
    ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);

    // Nuke both.
    nand.eraseBlock(META_A);
    nand.eraseBlock(META_B);

    FlightIndex loaded;
    EXPECT_EQ(loaded.load(nand, META_A, META_B), Status::Ok);  // clean fresh-chip state
    EXPECT_EQ(loaded.size(), 0u);
}

TEST(FlightIndexPersist, SavePicksHigherSequenceSourceAfterExternalEdit) {
    // Two independent writers see the same ground truth: if one writer crashed
    // mid-save (older seq in a block), the next save should pick that older
    // block to overwrite, preserving the newer one.
    FakeNandBackend nand;

    FlightIndex idx;
    idx.append(makeEntry(1, "a.bin", 0, 1, 0));
    ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);  // seq=1 in A
    ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);  // seq=2 in B
    // Now META_B holds seq=2. If we add an entry and save, the older copy (A,
    // seq=1) should be overwritten; B (seq=2) survives until the next save.
    idx.append(makeEntry(2, "b.bin", 0, 1, 0));
    uint64_t erases_before = nand.eraseCount();
    ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);  // seq=3 in A
    EXPECT_EQ(nand.eraseCount(), erases_before + 1);

    // Now corrupt B (higher seq before this last save — should be seq=2), and
    // verify that load picks A (seq=3).
    nand.eraseBlock(META_B);
    FlightIndex loaded;
    ASSERT_EQ(loaded.load(nand, META_A, META_B), Status::Ok);
    EXPECT_EQ(loaded.size(), 2u);
}

TEST(FlightIndexPersist, CrcMismatchDetectedOnBitFlip) {
    // Bit flip in an entry field -> CRC check must reject that snapshot.
    FakeNandBackend nand;
    FlightIndex idx;
    idx.append(makeEntry(1, "a.bin", 32, 256, 999));
    ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);

    // Zero out the CRC32 field of the first metadata block's page 0. NAND
    // program can only drive bits 1->0, so writing zeros over the CRC is
    // always a legal corruption simulation and reliably breaks the CRC check.
    uint8_t page[NAND_PAGE_SIZE];
    ASSERT_TRUE(nand.readPage(META_A, 0, page));
    std::memset(page, 0, 4);  // CRC32 field at offset 0
    ASSERT_TRUE(nand.programPage(META_A, 0, page));

    FlightIndex loaded;
    // Load should now fall back to B (empty / invalid) -> Ok, empty index.
    Status st = loaded.load(nand, META_A, META_B);
    EXPECT_EQ(st, Status::Ok);
    EXPECT_EQ(loaded.size(), 0u);
}

// ================================================================
// TR_FlightLog + FlightIndex integration
// ================================================================

TEST(TRFlightLogIndex, FreshChipBeginHasEmptyIndex) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    EXPECT_EQ(fl.index().size(), 0u);
}

// ================================================================
// prepareFlight — range pick + erase + bitmap transition
// ================================================================

namespace {
// Return true if every byte of every page in [start_block, start_block+n) is 0xFF.
bool rangeIsErased(FakeNandBackend& nand, uint32_t start_block, uint32_t n_blocks) {
    for (uint32_t b = start_block; b < start_block + n_blocks; ++b) {
        for (uint32_t p = 0; p < NAND_PAGES_PER_BLK; ++p) {
            const uint8_t* page = nand.peekPage(b, p);
            for (size_t i = 0; i < NAND_PAGE_SIZE; ++i) {
                if (page[i] != 0xFF) return false;
            }
        }
    }
    return true;
}
}  // namespace

TEST(TRFlightLogPrepare, FreshChipPicksRangeStart) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;  // default prealloc_blocks = 256
    ASSERT_EQ(fl.begin(nand, cfg, &store), Status::Ok);

    uint32_t flight_id = 0;
    ASSERT_EQ(fl.prepareFlight(flight_id), Status::Ok);
    EXPECT_EQ(flight_id, 1u);

    for (uint32_t b = cfg.flight_region_start;
         b < cfg.flight_region_start + cfg.prealloc_blocks; ++b) {
        EXPECT_EQ(fl.bitmap().get(b), BLOCK_ALLOCATED) << "block " << b;
    }
    EXPECT_TRUE(rangeIsErased(nand, cfg.flight_region_start, cfg.prealloc_blocks));
}

TEST(TRFlightLogPrepare, RefusesWhenAlreadyFlying) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    uint32_t a = 0, b = 0;
    ASSERT_EQ(fl.prepareFlight(a), Status::Ok);
    EXPECT_EQ(fl.prepareFlight(b), Status::Error);
}

TEST(TRFlightLogPrepare, SkipsKnownBadBlocksWhenPicking) {
    // Inject bad blocks at the start of the flight region; picker should jump
    // past them to the first contiguous free run.
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    // Factory-bad a block partway through the first natural 256-block range.
    nand.injectFactoryBadBlock(cfg.flight_region_start + 100);

    ASSERT_EQ(fl.begin(nand, cfg, &store), Status::Ok);
    uint32_t flight_id = 0;
    ASSERT_EQ(fl.prepareFlight(flight_id), Status::Ok);

    // The chosen start must be >= flight_region_start + 101 (first fresh free
    // run after the bad block).
    // Walk the bitmap to find the chosen range.
    uint32_t first_alloc = 0;
    for (uint32_t b = 0; b < NAND_BLOCK_COUNT; ++b) {
        if (fl.bitmap().get(b) == BLOCK_ALLOCATED) { first_alloc = b; break; }
    }
    EXPECT_EQ(first_alloc, cfg.flight_region_start + 101);
    EXPECT_EQ(fl.bitmap().get(cfg.flight_region_start + 100), BLOCK_BAD);
}

TEST(TRFlightLogPrepare, NoSpaceWhenInsufficientContiguousFree) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;

    TR_FlightLog::Config cfg;
    // Shrink the flight region so 256 blocks don't fit, then try to prepare.
    cfg.flight_region_start = 32;
    cfg.flight_region_end   = 200;   // only 168 blocks available
    cfg.prealloc_blocks     = 256;
    // Can't even pass begin with this config (guard rejects prealloc > region).
    EXPECT_EQ(fl.begin(nand, cfg, &store), Status::OutOfRange);

    // Retry with a valid config but artificially fragment the bitmap.
    cfg.flight_region_end = 1020;
    cfg.prealloc_blocks   = 256;
    ASSERT_EQ(fl.begin(nand, cfg, &store), Status::Ok);
    // Inject bad blocks every 100 blocks so no 256-block run exists.
    for (uint32_t b = cfg.flight_region_start; b < cfg.flight_region_end; b += 100) {
        ASSERT_EQ(fl.markBlockBad(b), Status::Ok);
    }
    uint32_t flight_id = 0;
    EXPECT_EQ(fl.prepareFlight(flight_id), Status::NoSpace);
}

TEST(TRFlightLogPrepare, AssignsMonotonicFlightId) {
    // Seed the index with a couple of entries, then begin + prepareFlight
    // should assign the next id in sequence.
    FakeNandBackend nand;
    MemoryBitmapStore store;

    FlightIndex idx;
    idx.append(makeEntry(5, "a.bin", 0, 1, 0));
    idx.append(makeEntry(9, "b.bin", 0, 1, 0));
    ASSERT_EQ(idx.save(nand, META_A, META_B), Status::Ok);

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    EXPECT_EQ(fl.index().size(), 2u);

    uint32_t flight_id = 0;
    ASSERT_EQ(fl.prepareFlight(flight_id), Status::Ok);
    EXPECT_EQ(flight_id, 10u);  // max(5,9)+1
}

TEST(TRFlightLogPrepare, PersistsBitmapOnSuccess) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);

    size_t saves_before = store.saveCount();
    uint32_t flight_id = 0;
    ASSERT_EQ(fl.prepareFlight(flight_id), Status::Ok);
    EXPECT_GT(store.saveCount(), saves_before);
}

// ================================================================
// writePage — hot path, bad-block skip, overflow extend
// ================================================================

TEST(TRFlightLogWrite, RefusesWhenNoFlightActive) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);

    uint8_t page[NAND_PAGE_SIZE] = {};
    EXPECT_EQ(fl.writePage(page), Status::Error);
}

TEST(TRFlightLogWrite, RejectsNullPagePointer) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
    EXPECT_EQ(fl.writePage(nullptr), Status::OutOfRange);
}

TEST(TRFlightLogWrite, HappyPathAdvancesPageCounter) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);

    std::vector<uint8_t> buf(NAND_PAGE_SIZE, 0x42);
    for (int i = 0; i < 10; ++i) {
        ASSERT_EQ(fl.writePage(buf.data()), Status::Ok);
    }
    EXPECT_EQ(fl.activePagesWritten(), 10u);
    EXPECT_EQ(nand.programCount(), 10u);

    // Every page landed at the expected (start + i/PAGES, i%PAGES) address.
    for (uint32_t p = 0; p < 10; ++p) {
        uint32_t b = fl.activeStartBlock() + p / NAND_PAGES_PER_BLK;
        uint32_t pib = p % NAND_PAGES_PER_BLK;
        const uint8_t* actual = nand.peekPage(b, pib);
        EXPECT_EQ(actual[0], 0x42) << "page " << p;
    }
}

TEST(TRFlightLogWrite, ProgramFailMarksBlockBadAndSkipsToNextBlock) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    ASSERT_EQ(fl.begin(nand, cfg, &store), Status::Ok);
    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);

    // Write one successful page so we know the base block.
    std::vector<uint8_t> buf(NAND_PAGE_SIZE, 0xAA);
    ASSERT_EQ(fl.writePage(buf.data()), Status::Ok);

    // Poison the next page's program call.
    const uint32_t next_rel = fl.activePagesWritten();
    const uint32_t bad_block = fl.activeStartBlock() + next_rel / NAND_PAGES_PER_BLK;
    const uint32_t bad_page  = next_rel % NAND_PAGES_PER_BLK;
    nand.injectProgramFailOnce(bad_block, bad_page);

    // Subsequent writePage should skip the now-bad block entirely and land at
    // the start of the next block.
    ASSERT_EQ(fl.writePage(buf.data()), Status::Ok);
    EXPECT_EQ(fl.bitmap().get(bad_block), BLOCK_BAD);
    EXPECT_TRUE(nand.isBlockBad(bad_block));

    // Next write position: next-block-page 1 (we just wrote page 0 of the new block).
    const uint32_t next_written_rel = fl.activePagesWritten();
    EXPECT_EQ(next_written_rel, (next_rel / NAND_PAGES_PER_BLK + 1) * NAND_PAGES_PER_BLK + 1);
}

TEST(TRFlightLogWrite, OverflowExtendSucceedsWhenAdjacentFree) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    cfg.prealloc_blocks = 4;   // tiny flight, easy to overrun
    cfg.extend_blocks   = 2;
    ASSERT_EQ(fl.begin(nand, cfg, &store), Status::Ok);

    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
    EXPECT_EQ(fl.activeBlockCount(), 4u);

    // Fill the entire prealloc range: 4 blocks * 64 pages = 256 pages.
    std::vector<uint8_t> buf(NAND_PAGE_SIZE, 0x11);
    for (int i = 0; i < 4 * static_cast<int>(NAND_PAGES_PER_BLK); ++i) {
        ASSERT_EQ(fl.writePage(buf.data()), Status::Ok);
    }

    // One more write triggers the extend path.
    EXPECT_EQ(fl.overflowExtensionCount(), 0u);
    ASSERT_EQ(fl.writePage(buf.data()), Status::Ok);
    EXPECT_EQ(fl.activeBlockCount(), 6u);
    EXPECT_EQ(fl.overflowExtensionCount(), 1u);
}

TEST(TRFlightLogWrite, OverflowExtendFailsWhenRegionExhausted) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    cfg.flight_region_start = 32;
    cfg.flight_region_end   = 36;    // only 4 blocks in the region
    cfg.prealloc_blocks     = 4;
    cfg.extend_blocks       = 2;
    ASSERT_EQ(fl.begin(nand, cfg, &store), Status::Ok);

    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
    std::vector<uint8_t> buf(NAND_PAGE_SIZE, 0x22);
    for (int i = 0; i < 4 * static_cast<int>(NAND_PAGES_PER_BLK); ++i) {
        ASSERT_EQ(fl.writePage(buf.data()), Status::Ok);
    }
    EXPECT_EQ(fl.writePage(buf.data()), Status::NoSpace);
}

TEST(TRFlightLogWrite, OverflowExtendFailsWhenAdjacentAllocated) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    cfg.prealloc_blocks = 4;
    cfg.extend_blocks   = 2;
    ASSERT_EQ(fl.begin(nand, cfg, &store), Status::Ok);

    // Reserve blocks adjacent to the flight region so extension has no
    // contiguous room. The first prepareFlight will pick blocks 32..35; mark
    // blocks 36..37 as BAD (same effect as allocated for extension).
    ASSERT_EQ(fl.markBlockBad(cfg.flight_region_start + cfg.prealloc_blocks), Status::Ok);
    ASSERT_EQ(fl.markBlockBad(cfg.flight_region_start + cfg.prealloc_blocks + 1), Status::Ok);

    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
    std::vector<uint8_t> buf(NAND_PAGE_SIZE, 0x33);
    for (int i = 0; i < 4 * static_cast<int>(NAND_PAGES_PER_BLK); ++i) {
        ASSERT_EQ(fl.writePage(buf.data()), Status::Ok);
    }
    EXPECT_EQ(fl.writePage(buf.data()), Status::NoSpace);
}

TEST(TRFlightLogPrepare, PicksNextRangeAfterFirstAllocation) {
    // Prepare+finalize a first flight, then prepare again — second range
    // should start right after the first.
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    ASSERT_EQ(fl.begin(nand, cfg, &store), Status::Ok);

    uint32_t first_id = 0;
    ASSERT_EQ(fl.prepareFlight(first_id), Status::Ok);
    ASSERT_EQ(fl.finalizeFlight("flight_001.bin", 1000), Status::Ok);

    uint32_t second_id = 0;
    ASSERT_EQ(fl.prepareFlight(second_id), Status::Ok);
    EXPECT_EQ(fl.activeStartBlock(),
              cfg.flight_region_start + cfg.prealloc_blocks);
}

// ================================================================
// finalizeFlight + listFlights + readFlightPage
// ================================================================

TEST(TRFlightLogFinalize, RefusesWhenNoFlightActive) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    EXPECT_EQ(fl.finalizeFlight("x.bin", 0), Status::Error);
}

TEST(TRFlightLogFinalize, AppendsEntryAndClearsActiveState) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);

    std::vector<uint8_t> page(NAND_PAGE_SIZE, 0x77);
    for (int i = 0; i < 5; ++i) ASSERT_EQ(fl.writePage(page.data()), Status::Ok);

    ASSERT_EQ(fl.finalizeFlight("flight_001.bin", 5 * NAND_PAGE_SIZE), Status::Ok);
    EXPECT_FALSE(fl.isFlightActive());
    EXPECT_EQ(fl.activeFlightId(), 0u);
    EXPECT_EQ(fl.index().size(), 1u);

    auto* e = fl.index().findByFilename("flight_001.bin");
    ASSERT_NE(e, nullptr);
    EXPECT_EQ(e->flight_id, id);
    EXPECT_EQ(e->n_blocks, 256u);
    EXPECT_EQ(e->final_bytes, 5u * NAND_PAGE_SIZE);
}

TEST(TRFlightLogFinalize, PersistsIndexAcrossReboot) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    {
        TR_FlightLog fl;
        ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
        uint32_t id = 0;
        ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
        ASSERT_EQ(fl.finalizeFlight("flight_001.bin", 1234), Status::Ok);
    }
    TR_FlightLog fl2;
    ASSERT_EQ(fl2.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    EXPECT_EQ(fl2.index().size(), 1u);
    EXPECT_STREQ(fl2.index().at(0).filename, "flight_001.bin");
}

TEST(TRFlightLogList, EmptyIndexReturnsZero) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    FlightIndexEntry out[10] = {};
    EXPECT_EQ(fl.listFlights(out, 10, 0, 10), 0u);
}

TEST(TRFlightLogList, PaginationSegments) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    cfg.prealloc_blocks = 4;   // cheap flights so we can run many
    ASSERT_EQ(fl.begin(nand, cfg, &store), Status::Ok);

    for (int i = 0; i < 5; ++i) {
        uint32_t id = 0;
        ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
        char name[20]; std::snprintf(name, sizeof(name), "flight_%03d.bin", i + 1);
        ASSERT_EQ(fl.finalizeFlight(name, 100 * (i + 1)), Status::Ok);
    }

    FlightIndexEntry p0[3] = {};
    FlightIndexEntry p1[3] = {};
    FlightIndexEntry p2[3] = {};

    EXPECT_EQ(fl.listFlights(p0, 3, /*page=*/0, /*per_page=*/3), 3u);
    EXPECT_STREQ(p0[0].filename, "flight_001.bin");
    EXPECT_STREQ(p0[2].filename, "flight_003.bin");

    EXPECT_EQ(fl.listFlights(p1, 3, /*page=*/1, /*per_page=*/3), 2u);
    EXPECT_STREQ(p1[0].filename, "flight_004.bin");
    EXPECT_STREQ(p1[1].filename, "flight_005.bin");

    EXPECT_EQ(fl.listFlights(p2, 3, /*page=*/2, /*per_page=*/3), 0u);
}

TEST(TRFlightLogRead, UnknownFilenameReturnsNotFound) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    uint8_t buf[64];
    size_t out_len = 0;
    EXPECT_EQ(fl.readFlightPage("nope.bin", 0, buf, sizeof(buf), out_len),
              Status::NotFound);
    EXPECT_EQ(out_len, 0u);
}

TEST(TRFlightLogRead, OffsetPastEndReturnsEOF) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
    std::vector<uint8_t> page(NAND_PAGE_SIZE, 0x88);
    ASSERT_EQ(fl.writePage(page.data()), Status::Ok);
    ASSERT_EQ(fl.finalizeFlight("f.bin", NAND_PAGE_SIZE), Status::Ok);

    uint8_t out[16];
    size_t out_len = 42;
    EXPECT_EQ(fl.readFlightPage("f.bin", NAND_PAGE_SIZE, out, sizeof(out), out_len),
              Status::Ok);
    EXPECT_EQ(out_len, 0u);
}

TEST(TRFlightLogRead, RoundTripMatchesWrittenBytes) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);

    // Write 3 distinct pages.
    std::vector<uint8_t> p0(NAND_PAGE_SIZE, 0xAA);
    std::vector<uint8_t> p1(NAND_PAGE_SIZE, 0xBB);
    std::vector<uint8_t> p2(NAND_PAGE_SIZE, 0xCC);
    for (auto& buf : {&p0, &p1, &p2}) {
        ASSERT_EQ(fl.writePage(buf->data()), Status::Ok);
    }
    ASSERT_EQ(fl.finalizeFlight("f.bin", 3 * NAND_PAGE_SIZE), Status::Ok);

    // Read each page back.
    for (uint32_t p = 0; p < 3; ++p) {
        std::vector<uint8_t> out(NAND_PAGE_SIZE);
        size_t out_len = 0;
        ASSERT_EQ(fl.readFlightPage("f.bin", p * NAND_PAGE_SIZE,
                                     out.data(), out.size(), out_len),
                  Status::Ok);
        EXPECT_EQ(out_len, NAND_PAGE_SIZE);
        uint8_t expected = (p == 0) ? 0xAA : (p == 1) ? 0xBB : 0xCC;
        for (auto b : out) EXPECT_EQ(b, expected) << "page " << p;
    }
}

// ================================================================
// deleteFlight + renameFlight
// ================================================================

TEST(TRFlightLogDelete, RemovesEntryAndReleasesBlocks) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    cfg.prealloc_blocks = 4;
    ASSERT_EQ(fl.begin(nand, cfg, &store), Status::Ok);
    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
    const uint32_t start = fl.activeStartBlock();
    ASSERT_EQ(fl.finalizeFlight("flight_001.bin", 100), Status::Ok);
    EXPECT_EQ(fl.bitmap().get(start), BLOCK_ALLOCATED);

    ASSERT_EQ(fl.deleteFlight("flight_001.bin"), Status::Ok);
    EXPECT_EQ(fl.index().size(), 0u);
    for (uint32_t b = start; b < start + 4; ++b) {
        EXPECT_EQ(fl.bitmap().get(b), BLOCK_FREE) << "block " << b;
    }
}

TEST(TRFlightLogDelete, UnknownFilenameReturnsNotFound) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    EXPECT_EQ(fl.deleteFlight("nope.bin"), Status::NotFound);
}

TEST(TRFlightLogDelete, PreservesBadBlocksWithinFreedRange) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    cfg.prealloc_blocks = 4;
    ASSERT_EQ(fl.begin(nand, cfg, &store), Status::Ok);

    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
    const uint32_t start = fl.activeStartBlock();

    // Simulate a runtime program-fail that marked one block bad mid-flight.
    ASSERT_EQ(fl.markBlockBad(start + 1), Status::Ok);
    // Re-mark the other three ALLOCATED since markBlockBad only touches one;
    // the range was already allocated from prepareFlight so state is:
    //   [ALLOC, BAD, ALLOC, ALLOC]
    ASSERT_EQ(fl.finalizeFlight("flight_001.bin", 100), Status::Ok);
    ASSERT_EQ(fl.deleteFlight("flight_001.bin"), Status::Ok);

    EXPECT_EQ(fl.bitmap().get(start + 0), BLOCK_FREE);
    EXPECT_EQ(fl.bitmap().get(start + 1), BLOCK_BAD);   // sticky
    EXPECT_EQ(fl.bitmap().get(start + 2), BLOCK_FREE);
    EXPECT_EQ(fl.bitmap().get(start + 3), BLOCK_FREE);
}

TEST(TRFlightLogDelete, DeletedBlocksReusedByNextPrepare) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    TR_FlightLog::Config cfg;
    cfg.prealloc_blocks = 8;
    cfg.flight_region_end = cfg.flight_region_start + 8;  // only fits one flight
    ASSERT_EQ(fl.begin(nand, cfg, &store), Status::Ok);

    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
    ASSERT_EQ(fl.finalizeFlight("flight_001.bin", 100), Status::Ok);

    // No room for a second flight without delete.
    uint32_t id2 = 0;
    EXPECT_EQ(fl.prepareFlight(id2), Status::NoSpace);

    // After delete, the second prepare succeeds — blocks are reclaimed.
    ASSERT_EQ(fl.deleteFlight("flight_001.bin"), Status::Ok);
    EXPECT_EQ(fl.prepareFlight(id2), Status::Ok);
}

TEST(TRFlightLogDelete, PersistsAcrossReboot) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    {
        TR_FlightLog fl;
        ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
        uint32_t id = 0;
        ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
        ASSERT_EQ(fl.finalizeFlight("flight_001.bin", 100), Status::Ok);
        ASSERT_EQ(fl.deleteFlight("flight_001.bin"), Status::Ok);
    }
    TR_FlightLog fl2;
    ASSERT_EQ(fl2.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    EXPECT_EQ(fl2.index().size(), 0u);
}

TEST(TRFlightLogRename, UpdatesFilenameInPlace) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
    ASSERT_EQ(fl.finalizeFlight("flight_001.bin", 100), Status::Ok);

    ASSERT_EQ(fl.renameFlight("flight_001.bin",
                              "flight_20260422_143000.bin"), Status::Ok);
    EXPECT_EQ(fl.index().findByFilename("flight_001.bin"), nullptr);
    EXPECT_NE(fl.index().findByFilename("flight_20260422_143000.bin"), nullptr);
}

TEST(TRFlightLogRename, UnknownFilenameReturnsNotFound) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    EXPECT_EQ(fl.renameFlight("absent.bin", "whatever.bin"), Status::NotFound);
}

// ================================================================
// writeFrame + brownout scanner / recovery
// ================================================================

TEST(TRFlightLogWriteFrame, EmbedsPageHeaderInWrittenPage) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);

    uint8_t payload[32];
    for (size_t i = 0; i < sizeof(payload); ++i) payload[i] = static_cast<uint8_t>(i);
    ASSERT_EQ(fl.writeFrame(payload, sizeof(payload)), Status::Ok);

    // Inspect the raw NAND page — PageHeader should be at offset 0.
    const uint8_t* raw = nand.peekPage(fl.activeStartBlock(), 0);
    PageHeader hdr;
    std::memcpy(&hdr, raw, sizeof(hdr));
    EXPECT_EQ(hdr.magic, FPAG_MAGIC);
    EXPECT_EQ(hdr.flight_id, id);
    EXPECT_EQ(hdr.seq_number, 0u);
    // Payload bytes follow the header unchanged.
    for (size_t i = 0; i < sizeof(payload); ++i) {
        EXPECT_EQ(raw[sizeof(hdr) + i], payload[i]) << "byte " << i;
    }
}

TEST(TRFlightLogWriteFrame, RejectsOversizedPayload) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
    std::vector<uint8_t> too_big(NAND_PAGE_SIZE, 0);
    EXPECT_EQ(fl.writeFrame(too_big.data(), too_big.size()), Status::OutOfRange);
}

TEST(TRFlightLogWriteFrame, RefusesWhenNoFlightActive) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    uint8_t payload[4] = {};
    EXPECT_EQ(fl.writeFrame(payload, sizeof(payload)), Status::Error);
}

TEST(TRFlightLogBrownout, NoOrphansOnFreshChip) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    size_t saves_before = store.saveCount();
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    EXPECT_EQ(fl.index().size(), 0u);
    // First boot persists the freshly-seeded bitmap exactly once; scanner
    // should not add extra saves beyond that.
    EXPECT_EQ(store.saveCount(), saves_before + 1);
}

TEST(TRFlightLogBrownout, RecoversPartialFlightWithValidHeaders) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    uint32_t expected_flight_id = 0;

    // Simulated first boot — start a flight and crash (scope exit with no
    // finalizeFlight call leaves the blocks orphaned).
    {
        TR_FlightLog fl;
        ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
        ASSERT_EQ(fl.prepareFlight(expected_flight_id), Status::Ok);

        uint8_t payload[64];
        for (size_t i = 0; i < sizeof(payload); ++i) payload[i] = 0xA0 + (i & 0xF);
        for (int i = 0; i < 50; ++i) {
            ASSERT_EQ(fl.writeFrame(payload, sizeof(payload)), Status::Ok);
        }
    }

    // Simulated reboot.
    TR_FlightLog fl2;
    ASSERT_EQ(fl2.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    ASSERT_EQ(fl2.index().size(), 1u);

    const auto& e = fl2.index().at(0);
    EXPECT_EQ(e.flight_id, expected_flight_id);
    EXPECT_EQ(e.n_blocks, 256u);
    // Last valid seq was 49; final_bytes = (49+1) * NAND_PAGE_SIZE.
    EXPECT_EQ(e.final_bytes, 50u * NAND_PAGE_SIZE);
    // Filename is synthesized — caller can rename later.
    EXPECT_NE(std::strstr(e.filename, "recovered"), nullptr);
}

TEST(TRFlightLogBrownout, OrphanedRangeWithNoValidPagesIsFreed) {
    FakeNandBackend nand;
    MemoryBitmapStore store;

    // First boot — prepareFlight then crash before any writes.
    {
        TR_FlightLog fl;
        ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
        uint32_t id = 0;
        ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
    }

    TR_FlightLog fl2;
    ASSERT_EQ(fl2.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    EXPECT_EQ(fl2.index().size(), 0u);
    // The orphaned range should have been released.
    TR_FlightLog::Config cfg;
    for (uint32_t b = cfg.flight_region_start;
         b < cfg.flight_region_start + cfg.prealloc_blocks; ++b) {
        EXPECT_EQ(fl2.bitmap().get(b), BLOCK_FREE) << "block " << b;
    }
}

TEST(TRFlightLogBrownout, TornLastPageReverts) {
    // A page that was partially programmed (CRC mismatch) must be rejected;
    // recovery should land on the previous good page.
    FakeNandBackend nand;
    MemoryBitmapStore store;
    uint32_t start_block = 0;

    {
        TR_FlightLog fl;
        ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
        uint32_t id = 0;
        ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
        start_block = fl.activeStartBlock();
        uint8_t payload[16] = {0};
        for (int i = 0; i < 10; ++i) {
            ASSERT_EQ(fl.writeFrame(payload, sizeof(payload)), Status::Ok);
        }
    }

    // Corrupt the CRC field of the 10th page (seq=9) by writing zeros. NAND
    // only allows 1->0 transitions; zero-clearing the CRC field always
    // invalidates the page.
    {
        uint8_t page[NAND_PAGE_SIZE];
        uint32_t abs_block = start_block + 9 / NAND_PAGES_PER_BLK;
        uint32_t page_in_blk = 9 % NAND_PAGES_PER_BLK;
        ASSERT_TRUE(nand.readPage(abs_block, page_in_blk, page));
        std::memset(page, 0, 4);  // CRC32 field
        ASSERT_TRUE(nand.programPage(abs_block, page_in_blk, page));
    }

    TR_FlightLog fl2;
    ASSERT_EQ(fl2.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    ASSERT_EQ(fl2.index().size(), 1u);
    // Recovery should stop at seq=8 (last intact page); final_bytes = 9 pages.
    EXPECT_EQ(fl2.index().at(0).final_bytes, 9u * NAND_PAGE_SIZE);
}

// ================================================================
// TR_NandBackend_esp — Stage 1 stub is not yet wired to real hardware.
// These tests only verify the class satisfies the interface; Stage 2 will
// replace the stubs with real SPI NAND calls and land proper tests.
// ================================================================

TEST(TRNandBackendEsp, StubInstantiatesAndReturnsFailures) {
    tr_flightlog::TR_NandBackend_esp backend;
    uint8_t buf[tr_flightlog::NAND_PAGE_SIZE] = {};
    EXPECT_FALSE(backend.readPage(0, 0, buf));
    EXPECT_FALSE(backend.programPage(0, 0, buf));
    EXPECT_FALSE(backend.eraseBlock(0));
    EXPECT_FALSE(backend.markBlockBad(0));
    EXPECT_TRUE(backend.isBlockBad(0));  // conservative — all blocks "bad"
}

TEST(TRFlightLogBrownout, FinalizedFlightsAreNotRecovered) {
    // A normal finalized flight must not be double-counted by the scanner.
    FakeNandBackend nand;
    MemoryBitmapStore store;
    {
        TR_FlightLog fl;
        ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
        uint32_t id = 0;
        ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
        uint8_t payload[16] = {0};
        ASSERT_EQ(fl.writeFrame(payload, sizeof(payload)), Status::Ok);
        ASSERT_EQ(fl.finalizeFlight("flight_001.bin", NAND_PAGE_SIZE), Status::Ok);
    }

    TR_FlightLog fl2;
    ASSERT_EQ(fl2.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    // Expect exactly the one entry we finalized — no synthetic recovery one.
    EXPECT_EQ(fl2.index().size(), 1u);
    EXPECT_STREQ(fl2.index().at(0).filename, "flight_001.bin");
}

TEST(TRFlightLogRename, PersistsAcrossReboot) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    {
        TR_FlightLog fl;
        ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
        uint32_t id = 0;
        ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
        ASSERT_EQ(fl.finalizeFlight("flight_001.bin", 100), Status::Ok);
        ASSERT_EQ(fl.renameFlight("flight_001.bin",
                                  "flight_20260422_143000.bin"), Status::Ok);
    }
    TR_FlightLog fl2;
    ASSERT_EQ(fl2.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    EXPECT_EQ(fl2.index().size(), 1u);
    EXPECT_STREQ(fl2.index().at(0).filename, "flight_20260422_143000.bin");
}

TEST(TRFlightLogRead, CapsReadAtFileEnd) {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
    uint32_t id = 0;
    ASSERT_EQ(fl.prepareFlight(id), Status::Ok);

    // Write one page but declare final_bytes = 100.
    std::vector<uint8_t> page(NAND_PAGE_SIZE, 0x5A);
    ASSERT_EQ(fl.writePage(page.data()), Status::Ok);
    ASSERT_EQ(fl.finalizeFlight("f.bin", 100), Status::Ok);

    uint8_t out[NAND_PAGE_SIZE] = {};
    size_t out_len = 0;
    ASSERT_EQ(fl.readFlightPage("f.bin", 0, out, sizeof(out), out_len), Status::Ok);
    EXPECT_EQ(out_len, 100u);  // capped at final_bytes
    for (size_t i = 0; i < 100; ++i) EXPECT_EQ(out[i], 0x5A);
}
