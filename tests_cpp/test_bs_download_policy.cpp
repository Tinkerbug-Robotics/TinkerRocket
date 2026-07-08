#include <gtest/gtest.h>

#include "bs_download_policy.h"

using bs_download_policy::mayEmitChunk;
using bs_download_policy::nextChunk;

// #380: the BLE download is a per-loop-iteration state machine; these helpers
// carry its chunk arithmetic (bounded by the size measured at open) and the
// 15 ms wall-time pacing the BLE notify path needs to drain its mbufs.

namespace {
constexpr uint32_t CHUNK = 170;   // config::BLE_FILE_CHUNK_SIZE
constexpr uint32_t DELAY = 15;    // config::BLE_CHUNK_DELAY_MS
}

// ---- nextChunk ----

TEST(BsDownloadPolicy, EmptyFileIsSingleEmptyEofChunk) {
    const auto p = nextChunk(/*file_size=*/0, /*offset=*/0, CHUNK);
    EXPECT_EQ(p.read_len, 0u);
    EXPECT_TRUE(p.eof);
}

TEST(BsDownloadPolicy, FileSmallerThanChunkIsSingleEofChunk) {
    const auto p = nextChunk(100, 0, CHUNK);
    EXPECT_EQ(p.read_len, 100u);
    EXPECT_TRUE(p.eof);
}

TEST(BsDownloadPolicy, MidFileChunkIsFullSizeNotEof) {
    const auto p = nextChunk(10 * CHUNK, 3 * CHUNK, CHUNK);
    EXPECT_EQ(p.read_len, CHUNK);
    EXPECT_FALSE(p.eof);
}

TEST(BsDownloadPolicy, ExactMultipleFlagsEofOnFinalFullChunk) {
    // size == 4 chunks: the 4th chunk is full AND terminal — eof must ride on
    // it, not require a 5th empty chunk.
    const auto p = nextChunk(4 * CHUNK, 3 * CHUNK, CHUNK);
    EXPECT_EQ(p.read_len, CHUNK);
    EXPECT_TRUE(p.eof);
}

TEST(BsDownloadPolicy, RemainderTailChunkFlagsEof) {
    const auto p = nextChunk(4 * CHUNK + 37, 4 * CHUNK, CHUNK);
    EXPECT_EQ(p.read_len, 37u);
    EXPECT_TRUE(p.eof);
}

TEST(BsDownloadPolicy, OffsetAtOrPastSizeIsEmptyEof) {
    // Defensive: never a negative/underflowed read, always terminal.
    for (uint32_t off : {4 * CHUNK, 4 * CHUNK + 1, 100 * CHUNK}) {
        const auto p = nextChunk(4 * CHUNK, off, CHUNK);
        EXPECT_EQ(p.read_len, 0u) << "offset " << off;
        EXPECT_TRUE(p.eof) << "offset " << off;
    }
}

TEST(BsDownloadPolicy, WalkedTransferDeliversExactlyFileSizeAndTerminates) {
    // The no-hang property: iterating plan->advance covers the whole file,
    // flags eof exactly once (on the last chunk), and never overshoots.
    const uint32_t size = 3 * CHUNK + 121;  // 3 full chunks + a tail
    uint32_t offset = 0;
    uint32_t chunks = 0;
    bool done = false;
    while (!done) {
        ASSERT_LT(chunks, 100u) << "transfer did not terminate";
        const auto p = nextChunk(size, offset, CHUNK);
        offset += p.read_len;
        done = p.eof;
        ++chunks;
    }
    EXPECT_EQ(offset, size);
    EXPECT_EQ(chunks, 4u);
}

// ---- mayEmitChunk (pacing) ----

TEST(BsDownloadPolicy, FirstChunkEmitsImmediately) {
    EXPECT_TRUE(mayEmitChunk(/*now=*/12345, /*last=*/0, DELAY));
}

TEST(BsDownloadPolicy, WithinDrainIntervalHolds) {
    EXPECT_FALSE(mayEmitChunk(1000, 990, DELAY));   // 10 ms < 15 ms
    EXPECT_FALSE(mayEmitChunk(1014, 1000, DELAY));  // 14 ms
}

TEST(BsDownloadPolicy, AtOrPastDrainIntervalEmits) {
    EXPECT_TRUE(mayEmitChunk(1015, 1000, DELAY));
    EXPECT_TRUE(mayEmitChunk(2000, 1000, DELAY));
}

TEST(BsDownloadPolicy, MillisWraparoundStillPaces) {
    // last stamped just before uint32 wrap; now just after. Unsigned
    // subtraction must see the true elapsed time, not a huge negative.
    const uint32_t last = 0xFFFFFFF8u;              // 8 ticks before wrap
    EXPECT_FALSE(mayEmitChunk(/*now=*/2, last, DELAY));   // elapsed 10 ms
    EXPECT_TRUE(mayEmitChunk(/*now=*/7, last, DELAY));    // elapsed 15 ms
}
