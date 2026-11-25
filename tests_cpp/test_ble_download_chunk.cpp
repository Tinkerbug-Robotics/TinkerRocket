// #1160 / #1155 item 2 — a log frame larger than the chunk capacity must never
// be handed to sendFileChunk() whole. NimBLE truncates an over-MTU notification
// to the MTU and returns success, so the OC counted bytes the phone never got.
#include <gtest/gtest.h>
#include <algorithm>
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

// ============================================================================
// nextSegment: the stream must be the stored file, byte for byte.
//
// The download loop used to send only complete frames: it stepped over any
// byte that did not start a frame and dropped a partial frame left at end of
// file. The listing reports the stored size and both apps reject a download
// that falls short of it — the V5 nose cone's 2026-10-24 flight listed
// 6,901,811 B and streamed 6,901,781. These tests replay the out computer's
// loop (4 KB reads, carry-over, planAppend/pieceLen, the final EOF flush)
// against stored files of every shape and require the reassembled stream to
// equal the file.
// ============================================================================

using tr_ble::nextSegment;
using tr_ble::Segment;

namespace {

std::vector<uint8_t> frame(uint8_t type, uint8_t len, uint8_t fill = 0x11)
{
    std::vector<uint8_t> f = {0xAA, 0x55, 0xAA, 0x55, type, len};
    for (uint8_t i = 0; i < len; ++i) f.push_back(static_cast<uint8_t>(fill + i));
    f.push_back(0x12);   // the CRC is opaque to the stream
    f.push_back(0x34);
    return f;
}

void append(std::vector<uint8_t>& to, const std::vector<uint8_t>& what)
{
    to.insert(to.end(), what.begin(), what.end());
}

struct Streamed
{
    std::vector<uint8_t> bytes;   // what the app reassembles
    size_t frames = 0;
    size_t notifications = 0;
    size_t max_payload = 0;
};

// The out computer's cmd-4 loop, minus BLE and timing (main.cpp, "Every stored
// byte goes out"). read_size stands for FLASH_READ_SIZE; a read returns up to
// read_size bytes and reports eof when it comes back short, as
// flightlogReadChunk does.
Streamed streamFile(const std::vector<uint8_t>& file, size_t chunk, size_t read_size = 4096)
{
    constexpr size_t kMaxFrame = 263;
    std::vector<uint8_t> read_buf(read_size + kMaxFrame);
    std::vector<uint8_t> ble_buf;
    Streamed out;
    auto send = [&](const uint8_t* p, size_t n) {
        out.bytes.insert(out.bytes.end(), p, p + n);
        out.notifications++;
        if (n > out.max_payload) out.max_payload = n;
    };
    size_t file_offset = 0, carryover = 0;
    bool eof = false;
    while (!eof)
    {
        const size_t n = std::min(read_size, file.size() - file_offset);
        eof = n < read_size;
        std::copy(file.begin() + file_offset, file.begin() + file_offset + n, read_buf.begin() + carryover);
        file_offset += n;
        const size_t buf_len = carryover + n;
        size_t pos = 0;
        for (;;)
        {
            const Segment seg = nextSegment(read_buf.data() + pos, buf_len - pos, eof);
            if (seg.len == 0) break;
            const auto plan = planAppend(ble_buf.size(), seg.len, chunk);
            if (plan.flush_first) { send(ble_buf.data(), ble_buf.size()); ble_buf.clear(); }
            if (plan.direct_pieces == 0)
            {
                ble_buf.insert(ble_buf.end(), read_buf.begin() + pos, read_buf.begin() + pos + seg.len);
            }
            else
            {
                size_t off = 0;
                for (size_t i = 0; i < plan.direct_pieces; ++i)
                {
                    const size_t piece = pieceLen(seg.len, chunk, i);
                    send(read_buf.data() + pos + off, piece);
                    off += piece;
                }
            }
            if (seg.frame) out.frames++;
            pos += seg.len;
        }
        carryover = buf_len - pos;
        EXPECT_LE(carryover, kMaxFrame - 1) << "the carry-over must fit read_buf's slack";
        std::copy(read_buf.begin() + pos, read_buf.begin() + buf_len, read_buf.begin());
    }
    EXPECT_EQ(carryover, 0u) << "nothing may be left behind at end of file";
    if (!ble_buf.empty()) send(ble_buf.data(), ble_buf.size());
    return out;
}

// The loop before this fix (frames only), so the defect is stated, not implied.
size_t legacyStreamedBytes(const std::vector<uint8_t>& file)
{
    size_t pos = 0, sent = 0;
    while (pos + 8 <= file.size())
    {
        if (file[pos] != 0xAA || file[pos + 1] != 0x55 || file[pos + 2] != 0xAA || file[pos + 3] != 0x55)
        {
            pos++;
            continue;
        }
        const size_t f = 8 + file[pos + 5];
        if (pos + f > file.size()) break;
        sent += f;
        pos += f;
    }
    return sent;
}

std::vector<uint8_t> flightOfFrames(size_t n_frames, uint32_t seed = 1)
{
    std::vector<uint8_t> f;
    uint32_t x = seed;
    for (size_t i = 0; i < n_frames; ++i)
    {
        x = x * 1664525u + 1013904223u;
        append(f, frame(static_cast<uint8_t>(0xA0 + (x >> 28)), static_cast<uint8_t>((x >> 8) % 256)));
    }
    return f;
}

}  // namespace

TEST(BleDownloadSegment, WholeFrameAtTheFront)
{
    const auto f = frame(0xA2, 22);
    const Segment s = nextSegment(f.data(), f.size(), false);
    EXPECT_EQ(s.len, 30u);
    EXPECT_TRUE(s.frame);
}

TEST(BleDownloadSegment, PartialFrameWaitsUnlessAtEndOfFile)
{
    const auto f = frame(0xA5, 52);
    EXPECT_EQ(nextSegment(f.data(), 40, false).len, 0u) << "the rest of the frame is in the next read";
    const Segment last = nextSegment(f.data(), 40, true);
    EXPECT_EQ(last.len, 40u) << "at end of file a partial frame still goes out";
    EXPECT_FALSE(last.frame);
    EXPECT_EQ(nextSegment(f.data(), 5, false).len, 0u) << "frame start without its length byte yet";
    EXPECT_EQ(nextSegment(f.data(), 5, true).len, 5u);
}

TEST(BleDownloadSegment, BytesBeforeTheNextFrameStartGoOutAsTheyAre)
{
    std::vector<uint8_t> b = {0x00, 0x00, 0xFF, 0x07};
    append(b, frame(0xA8, 0));
    const Segment s = nextSegment(b.data(), b.size(), false);
    EXPECT_EQ(s.len, 4u);
    EXPECT_FALSE(s.frame);
}

TEST(BleDownloadSegment, APossibleFrameStartSplitAcrossReadsIsHeldBack)
{
    for (size_t k = 1; k <= 3; ++k)
    {
        std::vector<uint8_t> b = {0x01, 0x02, 0x03};
        const uint8_t sof[4] = {0xAA, 0x55, 0xAA, 0x55};
        b.insert(b.end(), sof, sof + k);
        const Segment s = nextSegment(b.data(), b.size(), false);
        EXPECT_EQ(s.len, 3u) << "hold back the " << k << "-byte prefix of AA55AA55";
        EXPECT_EQ(nextSegment(b.data(), b.size(), true).len, b.size()) << "not at end of file";
    }
    const std::vector<uint8_t> aa = {0xAA};
    EXPECT_EQ(nextSegment(aa.data(), 1, false).len, 0u);
    EXPECT_EQ(nextSegment(aa.data(), 1, true).len, 1u);
    EXPECT_EQ(nextSegment(nullptr, 0, true).len, 0u);
}

TEST(BleDownloadStream, FramesOnlyStreamExactlyAsBefore)
{
    const auto file = flightOfFrames(2000);
    const auto s = streamFile(file, 488);
    EXPECT_EQ(s.bytes, file);
    EXPECT_EQ(s.frames, 2000u);
    EXPECT_EQ(legacyStreamedBytes(file), file.size()) << "a clean file never showed the defect";
}

// The V5's shape: a whole flight ending in END_FLIGHT (0xA8, no payload), then
// 30 stored bytes that are not a whole frame.
TEST(BleDownloadStream, BytesAfterTheLastFrameAreSent)
{
    for (const bool partial_frame_tail : {false, true})
    {
        auto file = flightOfFrames(3000, 7);
        append(file, frame(0xA8, 0));
        if (partial_frame_tail)
        {
            const auto f = frame(0xA2, 40);
            file.insert(file.end(), f.begin(), f.begin() + 30);   // a frame cut at 30 of 48 bytes
        }
        else
        {
            file.insert(file.end(), 30, 0x00);
        }
        const auto s = streamFile(file, 488);
        EXPECT_EQ(s.bytes.size(), file.size()) << "streamed must equal the listed (stored) size";
        EXPECT_EQ(s.bytes, file);
        EXPECT_EQ(file.size() - legacyStreamedBytes(file), 30u) << "the old loop came up 30 B short";
    }
}

TEST(BleDownloadStream, JunkBetweenFramesIsSentInPlace)
{
    auto file = flightOfFrames(500, 3);
    std::vector<uint8_t> junk = {0x00, 0xFF, 0xAA, 0x55, 0xAA, 0x00, 0x13};   // looks like half a frame start
    file.insert(file.begin() + 1000, junk.begin(), junk.end());
    file.insert(file.begin() + 9000, 2032, 0x00);   // a zero-filled page payload
    const auto s = streamFile(file, 175);
    EXPECT_EQ(s.bytes, file);
    EXPECT_LE(s.max_payload, 175u);
}

TEST(BleDownloadStream, ExactMultipleOfTheReadSizeAndEmptyFiles)
{
    std::vector<uint8_t> file(4096 * 3, 0x5A);
    EXPECT_EQ(streamFile(file, 488).bytes, file);
    const std::vector<uint8_t> empty;
    const auto s = streamFile(empty, 488);
    EXPECT_TRUE(s.bytes.empty());
    EXPECT_EQ(s.notifications, 0u);
}

// Random files of frames, junk and frame-start look-alikes, every chunk size
// and several read sizes: the stream is always the file, no notification is
// ever larger than a chunk.
TEST(BleDownloadStream, FuzzStreamEqualsTheStoredFile)
{
    uint32_t x = 12345;
    auto rnd = [&](uint32_t n) { x = x * 1664525u + 1013904223u; return (x >> 8) % n; };
    for (int trial = 0; trial < 300; ++trial)
    {
        std::vector<uint8_t> file;
        const size_t pieces = 1 + rnd(400);
        for (size_t i = 0; i < pieces; ++i)
        {
            switch (rnd(6))
            {
            case 0: { std::vector<uint8_t> j(1 + rnd(40)); for (auto& b : j) b = (uint8_t)rnd(256); append(file, j); break; }
            case 1: { const uint8_t sof[4] = {0xAA, 0x55, 0xAA, 0x55}; file.insert(file.end(), sof, sof + 1 + rnd(3)); break; }
            default: append(file, frame((uint8_t)(0xA0 + rnd(16)), (uint8_t)rnd(256))); break;
            }
        }
        if (rnd(2)) { const auto f = frame(0xA7, 200); file.insert(file.end(), f.begin(), f.begin() + 1 + rnd(200)); }
        for (const size_t chunk : {13u, 175u, 244u, 488u})
            for (const size_t read_size : {4096u, 1000u, 300u})
            {
                const auto s = streamFile(file, chunk, read_size);
                ASSERT_EQ(s.bytes, file) << "trial " << trial << " chunk " << chunk << " read " << read_size;
                ASSERT_LE(s.max_payload, chunk);
            }
    }
}
