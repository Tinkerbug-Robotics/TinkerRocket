#include <gtest/gtest.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>
#include "i2s_link_check.h"

// The bench monitor's view of the I2S link (i2s_link_check.h): frames
// reassembled across DMA buffers as the OC's parser does, matched in write
// order against the sender's log, and ISM6 batch gaps.

namespace {

constexpr size_t BUF = 1024;
constexpr size_t MAX_PAYLOAD = 224;   // RocketComputerTypes.h

// A frame as TR_I2C_Interface::packMessage builds it.
std::vector<uint8_t> frame(uint8_t type, const std::vector<uint8_t> &payload)
{
    std::vector<uint8_t> f = { 0xAA, 0x55, 0xAA, 0x55, type, (uint8_t)payload.size() };
    f.insert(f.end(), payload.begin(), payload.end());
    const uint16_t crc = crc16Frame(f.data() + 4, payload.size() + 2);
    f.push_back((uint8_t)(crc >> 8));
    f.push_back((uint8_t)(crc & 0xFF));
    return f;
}

std::vector<uint8_t> payloadOf(uint32_t seed, size_t len)
{
    std::vector<uint8_t> p(len);
    for (size_t i = 0; i < len; ++i)
    {
        seed = seed * 1664525u + 1013904223u;
        p[i] = (uint8_t)(seed >> 24);
    }
    return p;
}

struct Placed
{
    uint8_t type;
    std::vector<uint8_t> payload;
    size_t at;   // stream offset of its SOF
};

struct Got
{
    uint8_t type;
    std::vector<uint8_t> payload;
    uint32_t buf_id;
    size_t offset;
};

// Cut a stream into BUF-sized buffers and feed them in order.
std::vector<Got> parse(const std::vector<uint8_t> &stream, i2slink::FrameStream<BUF> &fs)
{
    std::vector<Got> got;
    for (size_t b = 0; b * BUF < stream.size(); ++b)
    {
        const size_t n = std::min(BUF, stream.size() - b * BUF);
        fs.feed(stream.data() + b * BUF, n, (uint32_t)b, [&](const i2slink::FrameAt &f) {
            got.push_back({ f.type, std::vector<uint8_t>(f.payload, f.payload + f.len), f.buf_id, f.offset });
        });
    }
    return got;
}

void expectAll(const std::vector<Placed> &want, const std::vector<Got> &got)
{
    ASSERT_EQ(got.size(), want.size());
    for (size_t i = 0; i < want.size(); ++i)
    {
        EXPECT_EQ(got[i].type, want[i].type) << "frame " << i;
        EXPECT_EQ(got[i].payload, want[i].payload) << "frame " << i;
        EXPECT_EQ(got[i].buf_id, want[i].at / BUF) << "frame " << i;
        EXPECT_EQ(got[i].offset, want[i].at % BUF) << "frame " << i;
    }
}

}  // namespace

TEST(I2sLinkFrameStream, ReassemblesFramesWhereverTheBufferBoundariesFall)
{
    // Frames of every size between runs of idle zeros, some back to back.
    std::vector<uint8_t> stream;
    std::vector<Placed> want;
    uint32_t seed = 7;
    for (int k = 0; k < 400; ++k)
    {
        seed = seed * 1664525u + 1013904223u;
        const size_t gap = (seed >> 28) == 0 ? 0 : (seed >> 24) % 61;
        stream.insert(stream.end(), gap, 0);
        const size_t len = 1 + (seed >> 8) % MAX_PAYLOAD;
        const uint8_t type = (uint8_t)(0x90 + k % 20);
        const std::vector<uint8_t> p = payloadOf(seed, len);
        want.push_back({ type, p, stream.size() });
        const std::vector<uint8_t> f = frame(type, p);
        stream.insert(stream.end(), f.begin(), f.end());
    }
    i2slink::FrameStream<BUF> fs(MAX_PAYLOAD);
    expectAll(want, parse(stream, fs));
    EXPECT_EQ(fs.crcFailures(), 0u);
}

TEST(I2sLinkFrameStream, AFrameStartingInTheLastBytesOfABufferIsKept)
{
    // The tail too short to read a length must be carried, not dropped: a
    // monitor that dropped it lost ~2% of the frames it watched.
    for (size_t k = 1; k <= 12; ++k)
    {
        std::vector<uint8_t> stream(BUF - k, 0);
        const std::vector<uint8_t> p = payloadOf((uint32_t)k, 40);
        const std::vector<Placed> want = { { 0xA3, p, BUF - k } };
        const std::vector<uint8_t> f = frame(0xA3, p);
        stream.insert(stream.end(), f.begin(), f.end());
        stream.resize(3 * BUF, 0);
        i2slink::FrameStream<BUF> fs(MAX_PAYLOAD);
        expectAll(want, parse(stream, fs));
    }
}

TEST(I2sLinkFrameStream, ACorruptFrameIsCountedAndTheNextOneStillFound)
{
    const std::vector<uint8_t> a = payloadOf(1, 100), b = payloadOf(2, 60);
    std::vector<uint8_t> stream(10, 0);
    std::vector<uint8_t> fa = frame(0x93, a);
    fa[50] ^= 0x01;
    stream.insert(stream.end(), fa.begin(), fa.end());
    const size_t at_b = stream.size();
    const std::vector<uint8_t> fb = frame(0xA5, b);
    stream.insert(stream.end(), fb.begin(), fb.end());
    stream.resize(BUF, 0);
    i2slink::FrameStream<BUF> fs(MAX_PAYLOAD);
    expectAll({ { 0xA5, b, at_b } }, parse(stream, fs));
    EXPECT_EQ(fs.crcFailures(), 1u);
}

TEST(I2sLinkFrameStream, AFalseSofWithAnImpossibleLengthIsSkipped)
{
    std::vector<uint8_t> stream = { 0xAA, 0x55, 0xAA, 0x55, 0x93, 0xFF, 0x00 };
    const size_t at = stream.size();
    const std::vector<uint8_t> p = payloadOf(3, 30);
    const std::vector<uint8_t> f = frame(0xD1, p);
    stream.insert(stream.end(), f.begin(), f.end());
    stream.resize(BUF, 0);
    i2slink::FrameStream<BUF> fs(MAX_PAYLOAD);
    expectAll({ { 0xD1, p, at } }, parse(stream, fs));
}

TEST(I2sLinkFrameStream, ResetDropsAPartialFrame)
{
    const std::vector<uint8_t> f = frame(0x93, payloadOf(4, 200));
    std::vector<uint8_t> first(BUF - 50, 0);
    first.insert(first.end(), f.begin(), f.begin() + 50);   // half a frame
    i2slink::FrameStream<BUF> fs(MAX_PAYLOAD);
    int frames = 0;
    fs.feed(first.data(), first.size(), 0, [&](const i2slink::FrameAt &) { ++frames; });
    fs.reset();
    std::vector<uint8_t> rest(f.begin() + 50, f.end());
    rest.resize(BUF, 0);
    fs.feed(rest.data(), rest.size(), 1, [&](const i2slink::FrameAt &) { ++frames; });
    EXPECT_EQ(frames, 0);
}

TEST(I2sLinkMatcher, MatchesInWriteOrderAndNamesTheMissingAndTheLate)
{
    i2slink::WriteLog<64> log;
    std::vector<std::vector<uint8_t>> p;
    for (uint32_t i = 0; i < 6; ++i)
    {
        p.push_back(payloadOf(10 + i, 22));
        log.add(0x93, 22, p.back().data(), 1000 + i);
    }
    auto key = [&](size_t i) { return i2slink::frameKey(0x93, 22, p[i].data()); };
    using Kind = i2slink::Matcher<64>::Kind;
    i2slink::Matcher<64> m;
    i2slink::Write w{};
    EXPECT_EQ(m.seen(log, 0x93, key(0), 2000, &w), Kind::Matched);
    EXPECT_EQ(w.t_us, 1000u);
    EXPECT_EQ(m.seen(log, 0x93, key(2), 2000, &w), Kind::Matched);   // 1 skipped
    EXPECT_EQ(m.seen(log, 0x93, key(4), 2000, &w), Kind::Matched);   // 3 skipped
    EXPECT_EQ(m.seen(log, 0x93, key(1), 2100, &w), Kind::Late);      // 1 arrives late
    const std::vector<uint8_t> other = payloadOf(99, 22);
    EXPECT_EQ(m.seen(log, 0x93, i2slink::frameKey(0x93, 22, other.data()), 2100, &w), Kind::NotInLog);
    EXPECT_EQ(m.seen(log, 0xA3, key(5), 2100, &w), Kind::NotInLog);  // same bytes, other type

    std::vector<uint32_t> missing;
    m.expire(2000 + 50, 100, [&](const i2slink::Write &x) { missing.push_back(x.t_us); });
    EXPECT_TRUE(missing.empty());                                     // not held long enough yet
    m.expire(2000 + 101, 100, [&](const i2slink::Write &x) { missing.push_back(x.t_us); });
    ASSERT_EQ(missing.size(), 1u);
    EXPECT_EQ(missing[0], 1003u);                                     // write 3, once
    m.expire(2000 + 500, 100, [&](const i2slink::Write &x) { missing.push_back(x.t_us); });
    EXPECT_EQ(missing.size(), 1u);
    EXPECT_EQ(m.untracked(), 0u);
}

TEST(I2sLinkImuGapTracker, CountsAMissingBatchAndIgnoresALateOne)
{
    constexpr size_t REC = 22;   // sizeof(ISM6HG256Data)
    auto batch = [&](uint32_t t0, uint8_t n) {
        std::vector<uint8_t> b(1 + n * REC, 0);
        b[0] = n;
        for (uint8_t i = 0; i < n; ++i)
        {
            const uint32_t t = t0 + 137u * i;
            memcpy(&b[1 + i * REC], &t, 4);
        }
        return b;
    };
    i2slink::ImuGapTracker g(REC);
    uint32_t t = 100000;
    for (int i = 0; i < 20; ++i, t += 137u * 10)
    {
        const auto b = batch(t, 10);
        g.batch(b.data(), b.size());
    }
    EXPECT_EQ(g.gaps, 0u);
    const auto late = batch(t, 10);   // this batch goes missing ...
    t += 137u * 10;
    const auto next = batch(t, 10);
    g.batch(next.data(), next.size());
    EXPECT_EQ(g.gaps, 1u);
    EXPECT_EQ(g.lost, 10u);
    g.batch(late.data(), late.size());   // ... and turns up afterwards
    EXPECT_EQ(g.gaps, 1u);
    t += 137u * 10;
    const auto after = batch(t, 5);
    g.batch(after.data(), after.size());
    EXPECT_EQ(g.gaps, 1u);
}
