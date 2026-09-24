#include <gtest/gtest.h>
#include <vector>
#include "ism6_fifo_decoder.h"

// The ISM6HG256X FIFO word decoder (#1485). Pure byte logic: the SPI burst
// reads that feed it are bench-only.

namespace {

// One FIFO word: tag byte (sensor code << 3 | slot counter << 1), then x, y, z
// as little-endian int16.
void word(std::vector<uint8_t>& buf, uint8_t sensor, uint8_t cnt, int16_t x, int16_t y, int16_t z)
{
    buf.push_back((uint8_t)((sensor << 3) | ((cnt & 0x3u) << 1)));
    for (int16_t v : {x, y, z})
    {
        buf.push_back((uint8_t)((uint16_t)v & 0xFFu));
        buf.push_back((uint8_t)((uint16_t)v >> 8));
    }
}

// A whole slot, gyro first, with values derived from `base` so each slot is
// distinguishable.
void slot(std::vector<uint8_t>& buf, uint8_t cnt, int16_t base)
{
    word(buf, Ism6FifoDecoder::TAG_GY, cnt, base, (int16_t)(base + 1), (int16_t)(base + 2));
    word(buf, Ism6FifoDecoder::TAG_XL, cnt, (int16_t)(base + 10), (int16_t)(base + 11), (int16_t)(base + 12));
    word(buf, Ism6FifoDecoder::TAG_XL_HG, cnt, (int16_t)(base + 20), (int16_t)(base + 21), (int16_t)(base + 22));
}

size_t words(const std::vector<uint8_t>& buf) { return buf.size() / Ism6FifoDecoder::WORD_BYTES; }

}  // namespace

TEST(Ism6FifoDecoder, CompleteSlotsComeOutInOrder)
{
    std::vector<uint8_t> buf;
    slot(buf, 0, 100);
    slot(buf, 1, 200);
    slot(buf, 2, 300);

    Ism6FifoDecoder d;
    std::vector<Ism6FifoSample> out;
    d.feed(buf.data(), words(buf), [&](const Ism6FifoSample& s) { out.push_back(s); });

    ASSERT_EQ(out.size(), 3u);
    EXPECT_EQ(out[0].g[0], 100);
    EXPECT_EQ(out[0].g[2], 102);
    EXPECT_EQ(out[0].lg[1], 111);
    EXPECT_EQ(out[0].hg[2], 122);
    EXPECT_EQ(out[2].g[0], 300);
    EXPECT_EQ(d.samples, 3u);
    EXPECT_EQ(d.incomplete_slots, 0u);
    EXPECT_EQ(d.counter_gaps, 0u);
}

TEST(Ism6FifoDecoder, WordOrderInsideASlotDoesNotMatter)
{
    std::vector<uint8_t> buf;
    word(buf, Ism6FifoDecoder::TAG_XL_HG, 1, 7, 8, 9);
    word(buf, Ism6FifoDecoder::TAG_GY, 1, 1, 2, 3);
    word(buf, Ism6FifoDecoder::TAG_XL, 1, 4, 5, 6);

    Ism6FifoDecoder d;
    std::vector<Ism6FifoSample> out;
    d.feed(buf.data(), words(buf), [&](const Ism6FifoSample& s) { out.push_back(s); });

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].g[0], 1);
    EXPECT_EQ(out[0].lg[0], 4);
    EXPECT_EQ(out[0].hg[0], 7);
}

TEST(Ism6FifoDecoder, ASlotSplitAcrossTwoBurstsIsFinishedByTheSecond)
{
    std::vector<uint8_t> buf;
    slot(buf, 3, 40);

    Ism6FifoDecoder d;
    std::vector<Ism6FifoSample> out;
    auto sink = [&](const Ism6FifoSample& s) { out.push_back(s); };
    d.feed(buf.data(), 2, sink);  // gyro + low-g only
    EXPECT_TRUE(out.empty());
    d.feed(buf.data() + 2 * Ism6FifoDecoder::WORD_BYTES, 1, sink);  // high-g

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].hg[0], 60);
    EXPECT_EQ(d.incomplete_slots, 0u);
}

TEST(Ism6FifoDecoder, ASlotMissingAWordIsCountedAndNotEmitted)
{
    std::vector<uint8_t> buf;
    word(buf, Ism6FifoDecoder::TAG_GY, 0, 1, 1, 1);
    word(buf, Ism6FifoDecoder::TAG_XL, 0, 2, 2, 2);
    // no high-g for slot 0
    slot(buf, 1, 500);

    Ism6FifoDecoder d;
    std::vector<Ism6FifoSample> out;
    d.feed(buf.data(), words(buf), [&](const Ism6FifoSample& s) { out.push_back(s); });

    ASSERT_EQ(out.size(), 1u);
    EXPECT_EQ(out[0].g[0], 500);
    EXPECT_EQ(d.incomplete_slots, 1u);
    EXPECT_EQ(d.counter_gaps, 0u);
}

TEST(Ism6FifoDecoder, ASkippedSlotCounterIsCounted)
{
    std::vector<uint8_t> buf;
    slot(buf, 0, 10);
    slot(buf, 2, 30);  // slot 1 lost (an overflow drops whole words)

    Ism6FifoDecoder d;
    std::vector<Ism6FifoSample> out;
    d.feed(buf.data(), words(buf), [&](const Ism6FifoSample& s) { out.push_back(s); });

    EXPECT_EQ(out.size(), 2u);
    EXPECT_EQ(d.counter_gaps, 1u);
}

TEST(Ism6FifoDecoder, TheCounterWrapsFromThreeToZeroWithoutAGap)
{
    std::vector<uint8_t> buf;
    slot(buf, 3, 10);
    slot(buf, 0, 20);
    slot(buf, 1, 30);

    Ism6FifoDecoder d;
    size_t n = 0;
    d.feed(buf.data(), words(buf), [&](const Ism6FifoSample&) { n++; });

    EXPECT_EQ(n, 3u);
    EXPECT_EQ(d.counter_gaps, 0u);
}

TEST(Ism6FifoDecoder, ConfigChangeAndOtherWordsAreSkipped)
{
    std::vector<uint8_t> buf;
    word(buf, Ism6FifoDecoder::TAG_CFG_CHANGE, 0, 0, 0, 0);
    word(buf, Ism6FifoDecoder::TAG_TIMESTAMP, 0, 0, 0, 0);
    slot(buf, 1, 70);

    Ism6FifoDecoder d;
    size_t n = 0;
    d.feed(buf.data(), words(buf), [&](const Ism6FifoSample&) { n++; });

    EXPECT_EQ(n, 1u);
    EXPECT_EQ(d.cfg_changes, 1u);
    EXPECT_EQ(d.other_words, 1u);
}

TEST(Ism6FifoDecoder, AxesAreLittleEndianSigned)
{
    std::vector<uint8_t> buf;
    word(buf, Ism6FifoDecoder::TAG_GY, 0, 32767, -32768, -1);
    word(buf, Ism6FifoDecoder::TAG_XL, 0, 0, 1, -2);
    word(buf, Ism6FifoDecoder::TAG_XL_HG, 0, 256, -256, 255);

    Ism6FifoDecoder d;
    Ism6FifoSample got = {};
    d.feed(buf.data(), words(buf), [&](const Ism6FifoSample& s) { got = s; });

    EXPECT_EQ(got.g[0], 32767);
    EXPECT_EQ(got.g[1], -32768);
    EXPECT_EQ(got.g[2], -1);
    EXPECT_EQ(got.lg[2], -2);
    EXPECT_EQ(got.hg[0], 256);
    EXPECT_EQ(got.hg[1], -256);
    EXPECT_EQ(got.hg[2], 255);
}

TEST(Ism6FifoDecoder, ResetForgetsAHalfFilledSlot)
{
    std::vector<uint8_t> buf;
    slot(buf, 2, 90);

    Ism6FifoDecoder d;
    size_t n = 0;
    auto sink = [&](const Ism6FifoSample&) { n++; };
    d.feed(buf.data(), 2, sink);
    d.reset();
    d.feed(buf.data() + 2 * Ism6FifoDecoder::WORD_BYTES, 1, sink);

    EXPECT_EQ(n, 0u);
    EXPECT_EQ(d.samples, 0u);
}

TEST(Ism6FifoDecoder, APartialSlotIsReportedUntilItCompletes)
{
    std::vector<uint8_t> buf;
    slot(buf, 1, 5);

    Ism6FifoDecoder d;
    auto sink = [](const Ism6FifoSample&) {};
    EXPECT_FALSE(d.hasPartialSlot());
    d.feed(buf.data(), 1, sink);
    EXPECT_TRUE(d.hasPartialSlot());
    d.feed(buf.data() + Ism6FifoDecoder::WORD_BYTES, 2, sink);
    EXPECT_FALSE(d.hasPartialSlot());
}
