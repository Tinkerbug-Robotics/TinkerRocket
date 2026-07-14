// Host tests for FileStreamFraming.h — the #526 in-band record framing.
//
// iOS delivers the L2CAP channel as a byte stream with no SDU boundaries, so the
// decoder MUST reassemble records that were split at arbitrary byte offsets. The
// central test feeds the same encoded stream one byte at a time and proves it
// decodes identically to a single bulk feed. The Swift decoder is a twin of this.

#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "Crc32.h"
#include "FileStreamFraming.h"

using namespace tr_filestream;

namespace {

// A sink that records everything it is told, so a test can assert the decoded
// record sequence and the reassembled DATA payload.
struct RecordingSink : Sink
{
    struct Begin { uint8_t ver, status; uint32_t size_hint; std::string name; };
    struct End   { uint32_t bytes, crc; uint8_t status; };

    std::vector<Begin> begins;
    std::vector<uint8_t> data;   // all DATA payloads concatenated
    int data_calls = 0;
    std::vector<End> ends;
    std::vector<std::string> errors;

    void onBegin(uint8_t ver, uint8_t status, uint32_t size_hint,
                 const char* name, uint8_t name_len) override
    {
        begins.push_back({ver, status, size_hint, std::string(name, name_len)});
    }
    void onData(const uint8_t* d, size_t len) override
    {
        data.insert(data.end(), d, d + len);
        ++data_calls;
    }
    void onEnd(uint32_t bytes, uint32_t crc, uint8_t status) override
    {
        ends.push_back({bytes, crc, status});
    }
    void onError(const char* reason) override { errors.emplace_back(reason); }
};

// Encode a whole logical transfer: BEGIN, one DATA record carrying `payload`,
// END with the CRC over `payload`.
std::vector<uint8_t> encodeTransfer(const std::string& name,
                                    const std::vector<uint8_t>& payload)
{
    std::vector<uint8_t> out(4096 + payload.size());
    size_t o = 0;
    o += encodeBegin(out.data() + o, out.size() - o, kProtoVersion, kBeginOk,
                     (uint32_t)payload.size(), name.c_str(), (uint8_t)name.size());
    o += encodeDataHeader(out.data() + o, out.size() - o, (uint32_t)payload.size());
    std::memcpy(out.data() + o, payload.data(), payload.size());
    o += payload.size();
    const uint32_t crc = tr_crc32::compute(payload.data(), payload.size());
    o += encodeEnd(out.data() + o, out.size() - o, (uint32_t)payload.size(), crc, kEndOk);
    out.resize(o);
    return out;
}

}  // namespace

TEST(FileStreamFraming, EncodeDecodeRoundTrip)
{
    std::vector<uint8_t> payload(1500);
    for (size_t i = 0; i < payload.size(); ++i) payload[i] = (uint8_t)((i * 7 + 3) & 0xFF);
    auto stream = encodeTransfer("flight_20260714_112346.bin", payload);

    RecordingSink sink;
    Decoder dec(&sink);
    dec.feed(stream.data(), stream.size());

    ASSERT_EQ(sink.errors.size(), 0u);
    ASSERT_EQ(sink.begins.size(), 1u);
    EXPECT_EQ(sink.begins[0].ver, kProtoVersion);
    EXPECT_EQ(sink.begins[0].status, kBeginOk);
    EXPECT_EQ(sink.begins[0].size_hint, payload.size());
    EXPECT_EQ(sink.begins[0].name, "flight_20260714_112346.bin");
    EXPECT_EQ(sink.data, payload);
    ASSERT_EQ(sink.ends.size(), 1u);
    EXPECT_EQ(sink.ends[0].bytes, payload.size());
    EXPECT_EQ(sink.ends[0].crc, tr_crc32::compute(payload.data(), payload.size()));
    EXPECT_EQ(sink.ends[0].status, kEndOk);
}

// THE central property: a record split at ANY byte boundary decodes the same.
TEST(FileStreamFraming, SurvivesByteAtATimeFeed)
{
    std::vector<uint8_t> payload(777);
    for (size_t i = 0; i < payload.size(); ++i) payload[i] = (uint8_t)(i & 0xFF);
    auto stream = encodeTransfer("f.bin", payload);

    RecordingSink sink;
    Decoder dec(&sink);
    for (uint8_t b : stream) dec.feed(&b, 1);   // one byte per feed()

    ASSERT_EQ(sink.errors.size(), 0u);
    ASSERT_EQ(sink.begins.size(), 1u);
    EXPECT_EQ(sink.begins[0].name, "f.bin");
    EXPECT_EQ(sink.data, payload);
    ASSERT_EQ(sink.ends.size(), 1u);
    EXPECT_EQ(sink.ends[0].crc, tr_crc32::compute(payload.data(), payload.size()));
}

// And it must decode the same for EVERY possible single split point.
TEST(FileStreamFraming, EquivalentUnderEverySplit)
{
    std::vector<uint8_t> payload(300);
    for (size_t i = 0; i < payload.size(); ++i) payload[i] = (uint8_t)((i ^ 0x5A) & 0xFF);
    auto stream = encodeTransfer("x", payload);

    for (size_t split = 0; split <= stream.size(); ++split)
    {
        RecordingSink sink;
        Decoder dec(&sink);
        dec.feed(stream.data(), split);
        dec.feed(stream.data() + split, stream.size() - split);

        ASSERT_EQ(sink.errors.size(), 0u) << "split=" << split;
        ASSERT_EQ(sink.begins.size(), 1u) << "split=" << split;
        EXPECT_EQ(sink.data, payload) << "split=" << split;
        ASSERT_EQ(sink.ends.size(), 1u) << "split=" << split;
    }
}

// Many DATA records (as the firmware actually sends: one per SDU) concatenate
// into the whole file — this is what makes the cmd-4/cmd-44 sha256 A/B an oracle.
TEST(FileStreamFraming, MultipleDataRecordsConcatenate)
{
    std::vector<uint8_t> full;
    std::vector<uint8_t> stream(8192);
    size_t o = 0;
    o += encodeBegin(stream.data() + o, stream.size() - o, kProtoVersion, kBeginOk,
                     0, "m", 1);
    // three DATA records of different sizes
    for (uint32_t n : {uint32_t(100), uint32_t(1), uint32_t(488)})
    {
        std::vector<uint8_t> part(n);
        for (uint32_t i = 0; i < n; ++i) part[i] = (uint8_t)((n + i) & 0xFF);
        o += encodeDataHeader(stream.data() + o, stream.size() - o, n);
        std::memcpy(stream.data() + o, part.data(), n);
        o += n;
        full.insert(full.end(), part.begin(), part.end());
    }
    const uint32_t crc = tr_crc32::compute(full.data(), full.size());
    o += encodeEnd(stream.data() + o, stream.size() - o, (uint32_t)full.size(), crc, kEndOk);
    stream.resize(o);

    RecordingSink sink;
    Decoder dec(&sink);
    dec.feed(stream.data(), stream.size());

    EXPECT_EQ(sink.errors.size(), 0u);
    EXPECT_EQ(sink.data, full);
    ASSERT_EQ(sink.ends.size(), 1u);
    EXPECT_EQ(sink.ends[0].crc, crc);
}

// A refusal (BEGIN status=2) carries no data and no END — it is a complete,
// well-formed stream that simply is not a successful download.
TEST(FileStreamFraming, RefusalBeginDecodes)
{
    std::vector<uint8_t> stream(64);
    size_t o = encodeBegin(stream.data(), stream.size(), kProtoVersion, kBeginRefused,
                           0, "", 0);
    stream.resize(o);

    RecordingSink sink;
    Decoder dec(&sink);
    dec.feed(stream.data(), stream.size());

    ASSERT_EQ(sink.begins.size(), 1u);
    EXPECT_EQ(sink.begins[0].status, kBeginRefused);
    EXPECT_EQ(sink.begins[0].name, "");
    EXPECT_EQ(sink.data.size(), 0u);
    EXPECT_EQ(sink.ends.size(), 0u);
    EXPECT_EQ(sink.errors.size(), 0u);
}

// ---- malformed input: the decoder must latch failure, never wander ----

TEST(FileStreamFraming, RejectsUnknownRecordType)
{
    uint8_t bad[] = {0x09, 0x00, 0x00, 0x00, 0x00};
    RecordingSink sink;
    Decoder dec(&sink);
    dec.feed(bad, sizeof(bad));
    EXPECT_TRUE(dec.failed());
    EXPECT_EQ(sink.errors.size(), 1u);
}

TEST(FileStreamFraming, RejectsOversizeDataLength)
{
    uint8_t hdr[5];
    encodeDataHeader(hdr, sizeof(hdr), kMaxDataPayload + 1);
    RecordingSink sink;
    Decoder dec(&sink);
    dec.feed(hdr, sizeof(hdr));
    EXPECT_TRUE(dec.failed());
}

TEST(FileStreamFraming, RejectsWrongEndLength)
{
    // END must be exactly 9 bytes of payload.
    uint8_t bad[] = {kTypeEnd, 0x08, 0x00, 0x00, 0x00};
    RecordingSink sink;
    Decoder dec(&sink);
    dec.feed(bad, sizeof(bad));
    EXPECT_TRUE(dec.failed());
}

TEST(FileStreamFraming, StaysFailedAfterError)
{
    uint8_t bad[] = {0xFF};
    RecordingSink sink;
    Decoder dec(&sink);
    dec.feed(bad, sizeof(bad));
    ASSERT_TRUE(dec.failed());

    // Feeding a perfectly good stream afterwards must NOT resurrect it — a byte
    // stream cannot be resynchronised after a desync.
    std::vector<uint8_t> good = encodeTransfer("y", {1, 2, 3});
    dec.feed(good.data(), good.size());
    EXPECT_TRUE(dec.failed());
    EXPECT_EQ(sink.begins.size(), 0u);
}
