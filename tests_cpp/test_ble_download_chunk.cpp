// #1160 / #1155 item 2 — a log frame larger than the chunk capacity must never
// be handed to sendFileChunk() whole. NimBLE truncates an over-MTU notification
// to the MTU and returns success, so the OC counted bytes the phone never got.
#include <gtest/gtest.h>
#include <numeric>
#include <vector>
#include "BleDownloadChunk.h"

using tr_ble::planAppend;
using tr_ble::pieceLen;

namespace {
// The sizes from the issue: a 232-byte SNAPSHOT_MSG frame against the 175-byte
// chunk an iOS MTU of 185 yields.
constexpr size_t kSnapshotFrame = 232;
constexpr size_t kChunkAtMtu185 = 175;
constexpr size_t kMaxFrame      = 263;   // MAX_FRAME_SIZE in the download loops
}

TEST(BleDownloadChunk, FrameThatFitsBehindTheStageIsAppended)
{
    const auto p = planAppend(/*used=*/100, /*frame=*/50, /*chunk=*/175);
    EXPECT_FALSE(p.flush_first);
    EXPECT_EQ(p.direct_pieces, 0u);
}

TEST(BleDownloadChunk, FrameThatFitsAnEmptyStageFlushesThenAppends)
{
    const auto p = planAppend(/*used=*/100, /*frame=*/120, /*chunk=*/175);
    EXPECT_TRUE(p.flush_first) << "120 does not fit behind 100 in 175, but fits alone";
    EXPECT_EQ(p.direct_pieces, 0u);
}

// The regression. Old rule: `if (ble_used > 0 && ...) flush; append;` — with
// nothing staged, a 232-byte frame was appended whole and sent as a 239-byte
// notification on a 185-byte MTU.
TEST(BleDownloadChunk, OversizedFrameAtEmptyStageIsStreamedNotAppended)
{
    const auto p = planAppend(/*used=*/0, kSnapshotFrame, kChunkAtMtu185);
    EXPECT_FALSE(p.flush_first) << "nothing staged, nothing to flush";
    EXPECT_EQ(p.direct_pieces, 2u) << "232 bytes over 175-byte chunks is two pieces";
    EXPECT_EQ(pieceLen(kSnapshotFrame, kChunkAtMtu185, 0), 175u);
    EXPECT_EQ(pieceLen(kSnapshotFrame, kChunkAtMtu185, 1), 57u);
    EXPECT_EQ(pieceLen(kSnapshotFrame, kChunkAtMtu185, 2), 0u) << "no third piece";
}

TEST(BleDownloadChunk, OversizedFrameBehindAStageFlushesFirst)
{
    const auto p = planAppend(/*used=*/40, kSnapshotFrame, kChunkAtMtu185);
    EXPECT_TRUE(p.flush_first);
    EXPECT_EQ(p.direct_pieces, 2u);
}

// Exhaustive: for every (used, frame, chunk) the loop can see, no single
// notification payload may exceed the chunk capacity, and a streamed frame's
// pieces must sum to the frame exactly (the .bin is reassembled by offset).
TEST(BleDownloadChunk, NoPayloadEverExceedsTheChunkAndPiecesSumToTheFrame)
{
    for (size_t chunk : {13u, 20u, 175u, 182u, 244u, 488u})
      for (size_t frame = 8; frame <= kMaxFrame; ++frame)
        for (size_t used = 0; used <= chunk; ++used)
        {
            const auto p = planAppend(used, frame, chunk);
            if (p.flush_first) ASSERT_GT(used, 0u) << "flush with nothing staged";
            if (p.direct_pieces == 0)
            {
                // appended: what would be staged afterwards must fit one chunk
                const size_t after = (p.flush_first ? 0 : used) + frame;
                ASSERT_LE(after, chunk) << "used=" << used << " frame=" << frame << " chunk=" << chunk;
            }
            else
            {
                ASSERT_GT(frame, chunk) << "streamed a frame that fit";
                size_t sum = 0;
                for (size_t i = 0; i < p.direct_pieces; ++i)
                {
                    const size_t l = pieceLen(frame, chunk, i);
                    ASSERT_GT(l, 0u); ASSERT_LE(l, chunk);
                    sum += l;
                }
                ASSERT_EQ(sum, frame) << "pieces must reassemble to the frame";
                ASSERT_EQ(pieceLen(frame, chunk, p.direct_pieces), 0u) << "no piece past the end";
            }
        }
}

// The pre-#1160 rule, verbatim, so the defect is stated rather than implied:
// with nothing staged it did not flush, and it always appended — so the
// notification payload became the whole frame, MTU or not.
static size_t legacyPayloadAfterAppend(size_t used, size_t frame, size_t chunk)
{
    if (used > 0 && used + frame > chunk) used = 0;   // flush
    return used + frame;                               // append
}

TEST(BleDownloadChunk, TheOldRuleProducedAnOverMtuPayloadExactlyWhereTheNewOneStreams)
{
    // 232-byte SNAPSHOT_MSG, empty stage, 175-byte chunk (iOS MTU 185).
    EXPECT_EQ(legacyPayloadAfterAppend(0, kSnapshotFrame, kChunkAtMtu185), 232u)
        << "old rule: a 239-byte notification on a 185-byte MTU";
    const auto p = planAppend(0, kSnapshotFrame, kChunkAtMtu185);
    EXPECT_GT(p.direct_pieces, 0u) << "new rule: never handed to sendFileChunk whole";
}

TEST(BleDownloadChunk, ZeroChunkIsInertRatherThanDividingByZero)
{
    const auto p = planAppend(10, 100, 0);
    EXPECT_FALSE(p.flush_first);
    EXPECT_EQ(p.direct_pieces, 0u);
    EXPECT_EQ(pieceLen(100, 0, 0), 0u);
}

// The MTU-23 window: a download requested before the MTU exchange lands gets a
// 13-byte chunk. A 263-byte frame must become 21 pieces, not one truncated one.
TEST(BleDownloadChunk, TinyChunkStreamsInManyPieces)
{
    const auto p = planAppend(0, kMaxFrame, 13);
    EXPECT_EQ(p.direct_pieces, 21u);   // ceil(263 / 13)
    size_t sum = 0;
    for (size_t i = 0; i < p.direct_pieces; ++i) sum += pieceLen(kMaxFrame, 13, i);
    EXPECT_EQ(sum, kMaxFrame);
}
