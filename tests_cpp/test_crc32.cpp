// Host tests for Crc32.h — the integrity check that gates #526 download completion.
//
// The firmware CRCs the file incrementally as it streams, and iOS re-CRCs the
// bytes it received; a download completes only if the two agree. So the one thing
// that MUST be nailed down is the exact algorithm — a byte-identical twin
// (Crc32.swift) is pinned to the same check value in the app's test suite.

#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

#include "Crc32.h"

// The canonical CRC-32 check value, quoted in every reference (zlib, PNG, etc.).
TEST(Crc32, CanonicalCheckValue)
{
    const char* s = "123456789";
    EXPECT_EQ(tr_crc32::compute((const uint8_t*)s, 9), 0xCBF43926u);
}

TEST(Crc32, EmptyInputIsZero)
{
    // finalize(init) = 0xFFFFFFFF ^ 0xFFFFFFFF = 0.
    EXPECT_EQ(tr_crc32::compute(nullptr, 0), 0x00000000u);
}

TEST(Crc32, KnownVectors)
{
    // Independently verifiable ASCII vectors (standard CRC-32/ISO-HDLC).
    auto crc = [](const std::string& s) {
        return tr_crc32::compute((const uint8_t*)s.data(), s.size());
    };
    EXPECT_EQ(crc("a"), 0xE8B7BE43u);
    EXPECT_EQ(crc("abc"), 0x352441C2u);
    EXPECT_EQ(crc("The quick brown fox jumps over the lazy dog"), 0x414FA339u);
}

// The whole reason for the running API: the firmware never holds the file, it
// folds each DATA record in as it arrives. Splitting the input anywhere must not
// change the result.
TEST(Crc32, IncrementalEqualsOneShot)
{
    std::vector<uint8_t> buf(4096);
    for (size_t i = 0; i < buf.size(); ++i) buf[i] = (uint8_t)((i * 31 + 7) & 0xFF);

    const uint32_t oneshot = tr_crc32::compute(buf.data(), buf.size());

    for (size_t split : {size_t(0), size_t(1), size_t(500), size_t(4095), size_t(4096)})
    {
        uint32_t c = tr_crc32::init();
        c = tr_crc32::update(c, buf.data(), split);
        c = tr_crc32::update(c, buf.data() + split, buf.size() - split);
        EXPECT_EQ(tr_crc32::finalize(c), oneshot) << "split=" << split;
    }
}

TEST(Crc32, ManyTinyChunksEqualOneShot)
{
    std::vector<uint8_t> buf(1000);
    for (size_t i = 0; i < buf.size(); ++i) buf[i] = (uint8_t)(i & 0xFF);

    uint32_t c = tr_crc32::init();
    for (size_t i = 0; i < buf.size(); ++i) c = tr_crc32::update(c, &buf[i], 1);
    EXPECT_EQ(tr_crc32::finalize(c), tr_crc32::compute(buf.data(), buf.size()));
}

// A single flipped bit anywhere must change the CRC — otherwise it is not
// protecting the download from the silent-corruption failure mode #526 fears.
TEST(Crc32, DetectsSingleBitFlip)
{
    std::vector<uint8_t> buf(256);
    for (size_t i = 0; i < buf.size(); ++i) buf[i] = (uint8_t)i;
    const uint32_t good = tr_crc32::compute(buf.data(), buf.size());

    for (size_t i : {size_t(0), size_t(128), size_t(255)})
    {
        auto flipped = buf;
        flipped[i] ^= 0x01;
        EXPECT_NE(tr_crc32::compute(flipped.data(), flipped.size()), good) << "byte " << i;
    }
}
