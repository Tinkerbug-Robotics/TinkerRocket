// #398: NandBitmapStore — persist the TR_FlightLog 3-state block bitmap to two
// NAND metadata blocks (dual-copy, newest-sequence wins) instead of NVS, so the
// frequent prepareFlight/extend/bad-block saves never hit internal flash (whose
// erase-driven cache-disable stalled core 1 during NVS compaction).
//
// These tests pin the durability contract: roundtrip, newest-wins across the
// two blocks, power-safe fallback when the newest copy is interrupted/corrupt,
// and the fresh-chip / migration path (both blocks blank -> load() == false).

#include <gtest/gtest.h>

#include "NandBitmapStore.h"
#include "TR_FlightLog_types.h"
#include "fakes/fake_nand_backend.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using tr_flightlog::NandBitmapStore;
using tr_flightlog::NAND_PAGE_SIZE_MAX;

namespace {

// #671: run the suite at the env-selected chip geometry (same contract as
// test_tr_flightlog_core — CMake registers legacy/gd2g/gd1g variants). The
// store's blocks are the chip's top two (metadata_blocks[2]/[3]) and the
// bitmap payload is the 2-bit map's serialized size for that block count —
// 512 B at 2048 blocks, 256 B at the mini's 1024.
struct TestGeo { uint32_t page, ppb, blocks; };
inline TestGeo testGeoFromEnv() {
    const char* g = std::getenv("TR_TEST_NAND_GEOMETRY");
    if (g && std::string(g) == "gd2g") return {2048, 64, 2048};
    if (g && std::string(g) == "gd1g") return {2048, 64, 1024};
    return {4096, 64, 2048};
}
const TestGeo G = testGeoFromEnv();
const uint32_t BLK_A = G.blocks - 2;
const uint32_t BLK_B = G.blocks - 1;
const size_t   BITMAP_LEN = (G.blocks + 3) / 4;

struct FakeNandBackend : tr_flightlog_test::FakeNandBackend {
    FakeNandBackend() : tr_flightlog_test::FakeNandBackend(G.page, G.ppb, G.blocks) {}
};

// A recognizable bitmap payload that varies with `seed`.
std::vector<uint8_t> makeBitmap(uint8_t seed, size_t len = BITMAP_LEN) {
    std::vector<uint8_t> v(len);
    for (size_t i = 0; i < len; ++i) v[i] = static_cast<uint8_t>(seed + (i * 7));
    return v;
}

}  // namespace

TEST(NandBitmapStore, SaveLoadRoundtrip) {
    FakeNandBackend nand;
    NandBitmapStore store(&nand, BLK_A, BLK_B);

    auto in = makeBitmap(0x11);
    ASSERT_TRUE(store.save(in.data(), in.size()));

    std::vector<uint8_t> out(BITMAP_LEN, 0);
    ASSERT_TRUE(store.load(out.data(), out.size()));
    EXPECT_EQ(out, in);
}

TEST(NandBitmapStore, NewestSequenceWinsAcrossBlocks) {
    FakeNandBackend nand;
    NandBitmapStore store(&nand, BLK_A, BLK_B);

    // Five saves alternate between the two blocks; load must always return the
    // most recent one.
    for (uint8_t s = 1; s <= 5; ++s) {
        auto in = makeBitmap(s);
        ASSERT_TRUE(store.save(in.data(), in.size())) << "save " << int(s);
        std::vector<uint8_t> out(BITMAP_LEN, 0);
        ASSERT_TRUE(store.load(out.data(), out.size()));
        EXPECT_EQ(out, in) << "after save " << int(s);
    }
}

TEST(NandBitmapStore, InterruptedSaveKeepsLastGoodSnapshot) {
    FakeNandBackend nand;
    NandBitmapStore store(&nand, BLK_A, BLK_B);

    auto a = makeBitmap(0xA0);   // seq 1 -> BLK_A
    auto b = makeBitmap(0xB0);   // seq 2 -> BLK_B
    ASSERT_TRUE(store.save(a.data(), a.size()));
    ASSERT_TRUE(store.save(b.data(), b.size()));

    // Next save targets the OLDER block (BLK_A). Make its page program fail so
    // the save is interrupted after the erase — BLK_A is now blank/invalid.
    auto c = makeBitmap(0xC0);
    nand.injectProgramFailOnce(BLK_A, 0);
    EXPECT_FALSE(store.save(c.data(), c.size()));

    // The newest COMPLETE snapshot (b, seq 2 on BLK_B) must survive.
    std::vector<uint8_t> out(BITMAP_LEN, 0);
    ASSERT_TRUE(store.load(out.data(), out.size()));
    EXPECT_EQ(out, b);
}

TEST(NandBitmapStore, CorruptNewestFallsBackToOlder) {
    FakeNandBackend nand;
    NandBitmapStore store(&nand, BLK_A, BLK_B);

    auto a = makeBitmap(0x1A);   // seq 1 -> BLK_A
    auto b = makeBitmap(0x2B);   // seq 2 -> BLK_B
    ASSERT_TRUE(store.save(a.data(), a.size()));
    ASSERT_TRUE(store.save(b.data(), b.size()));

    // Make the newest copy (BLK_B) unreadable -> CRC/validity fails -> the
    // store must fall back to the older-but-valid BLK_A.
    nand.injectReadErrorPersistent(BLK_B, 0);
    std::vector<uint8_t> out(BITMAP_LEN, 0);
    ASSERT_TRUE(store.load(out.data(), out.size()));
    EXPECT_EQ(out, a);
}

TEST(NandBitmapStore, FreshBlocksReturnFalse) {
    FakeNandBackend nand;
    NandBitmapStore store(&nand, BLK_A, BLK_B);

    // Migration / first-boot: both metadata blocks blank (erased) -> no valid
    // snapshot -> load() returns false so begin() seeds a fresh bitmap.
    nand.eraseBlock(BLK_A);
    nand.eraseBlock(BLK_B);

    std::vector<uint8_t> out(BITMAP_LEN, 0);
    EXPECT_FALSE(store.load(out.data(), out.size()));
}

TEST(NandBitmapStore, UnboundStoreFailsCleanly) {
    NandBitmapStore store;   // never bound to a backend
    auto in = makeBitmap(0x33);
    EXPECT_FALSE(store.save(in.data(), in.size()));
    std::vector<uint8_t> out(BITMAP_LEN, 0);
    EXPECT_FALSE(store.load(out.data(), out.size()));
}

TEST(NandBitmapStore, ShorterStoredBitmapZeroPadsLoadBuffer) {
    FakeNandBackend nand;
    NandBitmapStore store(&nand, BLK_A, BLK_B);

    // Firmware that shrank the bitmap: store 256 B, load into 512 B — the tail
    // must be zero-filled, not garbage.
    auto in = makeBitmap(0x55, 256);
    ASSERT_TRUE(store.save(in.data(), in.size()));

    std::vector<uint8_t> out(512, 0xEE);
    ASSERT_TRUE(store.load(out.data(), out.size()));
    for (size_t i = 0; i < 256; ++i) EXPECT_EQ(out[i], in[i]) << "byte " << i;
    for (size_t i = 256; i < 512; ++i) EXPECT_EQ(out[i], 0) << "pad byte " << i;
}
