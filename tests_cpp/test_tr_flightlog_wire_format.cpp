#include <gtest/gtest.h>

#include "TR_FlightLog_types.h"
#include "WireFormat.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

// Byte-for-byte regression guard on the BLE wire formats iOS depends on.
// Fixture files under fixtures/ble_wire_format/ are the source of truth;
// any change to the encoders must either match the fixtures exactly, or
// be accompanied by updated fixtures + an explicit format-version bump.

using tr_flightlog::FlightIndexEntry;
using tr_flightlog::FLGT_MAGIC;
namespace wire = tr_flightlog::wire_format;

namespace {

// Compile-time path to the fixtures directory, resolved from the test build
// via a CMake-supplied define (see tests_cpp/CMakeLists.txt).
#ifndef TR_FLIGHTLOG_FIXTURES_DIR
#error "TR_FLIGHTLOG_FIXTURES_DIR must be defined by the build"
#endif

std::vector<uint8_t> loadFixture(const char* name) {
    std::string path = TR_FLIGHTLOG_FIXTURES_DIR;
    path += "/";
    path += name;
    std::ifstream in(path, std::ios::binary);
    if (!in) return {};
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),
                                 std::istreambuf_iterator<char>());
}

FlightIndexEntry makeEntry(const char* filename, uint32_t size) {
    FlightIndexEntry e{};
    e.magic       = FLGT_MAGIC;
    std::strncpy(e.filename, filename, sizeof(e.filename) - 1);
    e.final_bytes = size;
    return e;
}

}  // namespace

// ================================================================
// cmd 2 — file list JSON
// ================================================================

TEST(WireFormatFileList, EmptyListMatchesFixture) {
    auto golden = loadFixture("cmd2_list_empty.json");
    ASSERT_FALSE(golden.empty());

    char out[64] = {};
    size_t n = wire::encodeFileListJson(nullptr, 0, out, sizeof(out));
    ASSERT_EQ(n, golden.size());
    EXPECT_EQ(0, std::memcmp(out, golden.data(), n));
}

TEST(WireFormatFileList, ThreeFilesMatchFixture) {
    auto golden = loadFixture("cmd2_list_3_files.json");
    ASSERT_FALSE(golden.empty());

    FlightIndexEntry entries[3] = {
        makeEntry("flight_001.bin",              1234),
        makeEntry("flight_002.bin",              999999),
        makeEntry("flight_20260422_143000.bin",  0),
    };
    char out[256] = {};
    size_t n = wire::encodeFileListJson(entries, 3, out, sizeof(out));
    ASSERT_EQ(n, golden.size())
        << "encoded: " << std::string(out, n) << "\n"
        << "golden:  " << std::string(reinterpret_cast<const char*>(golden.data()),
                                      golden.size());
    EXPECT_EQ(0, std::memcmp(out, golden.data(), n));
}

TEST(WireFormatFileList, RejectsUndersizedBuffer) {
    FlightIndexEntry e = makeEntry("f.bin", 1);
    char out[8] = {};
    EXPECT_EQ(wire::encodeFileListJson(&e, 1, out, sizeof(out)), 0u);
}

// ================================================================
// cmd 4 — download chunk framing
// ================================================================

TEST(WireFormatFileChunk, FirstChunkMatchesFixture) {
    auto golden = loadFixture("cmd4_chunk_first.bin");
    ASSERT_FALSE(golden.empty());

    uint8_t data[16];
    for (size_t i = 0; i < sizeof(data); ++i) data[i] = static_cast<uint8_t>(i);
    uint8_t out[64] = {};
    size_t n = wire::encodeFileChunk(0, data, sizeof(data), false, out, sizeof(out));
    ASSERT_EQ(n, golden.size());
    EXPECT_EQ(0, std::memcmp(out, golden.data(), n));
}

TEST(WireFormatFileChunk, MidChunkMatchesFixture) {
    auto golden = loadFixture("cmd4_chunk_mid.bin");
    ASSERT_FALSE(golden.empty());

    const uint8_t data[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};
    uint8_t out[64] = {};
    size_t n = wire::encodeFileChunk(2048, data, sizeof(data), false, out, sizeof(out));
    ASSERT_EQ(n, golden.size());
    EXPECT_EQ(0, std::memcmp(out, golden.data(), n));
}

TEST(WireFormatFileChunk, EofFlagMatchesFixture) {
    auto golden = loadFixture("cmd4_chunk_last.bin");
    ASSERT_FALSE(golden.empty());

    const uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t out[64] = {};
    size_t n = wire::encodeFileChunk(4096, data, sizeof(data), /*eof=*/true,
                                     out, sizeof(out));
    ASSERT_EQ(n, golden.size());
    EXPECT_EQ(0, std::memcmp(out, golden.data(), n));
}

TEST(WireFormatFileChunk, EmptyChunkStillEmitsHeader) {
    auto golden = loadFixture("cmd4_chunk_empty.bin");
    ASSERT_FALSE(golden.empty());

    uint8_t out[16] = {};
    size_t n = wire::encodeFileChunk(0, nullptr, 0, /*eof=*/true, out, sizeof(out));
    ASSERT_EQ(n, golden.size());
    EXPECT_EQ(0, std::memcmp(out, golden.data(), n));
}

TEST(WireFormatFileChunk, DecodeRoundTripRecoversFields) {
    const uint8_t data[] = {1, 2, 3, 4, 5};
    uint8_t pkt[32] = {};
    size_t n = wire::encodeFileChunk(1234, data, sizeof(data), true, pkt, sizeof(pkt));
    ASSERT_EQ(n, sizeof(data) + wire::CHUNK_HEADER_SIZE);

    uint32_t off = 0;
    uint16_t len = 0;
    bool eof = false;
    const uint8_t* data_ptr = nullptr;
    ASSERT_TRUE(wire::decodeFileChunk(pkt, n, off, len, eof, &data_ptr));
    EXPECT_EQ(off, 1234u);
    EXPECT_EQ(len, 5u);
    EXPECT_TRUE(eof);
    ASSERT_NE(data_ptr, nullptr);
    EXPECT_EQ(0, std::memcmp(data_ptr, data, sizeof(data)));
}

TEST(WireFormatFileChunk, DecodeRejectsShortPacket) {
    uint8_t pkt[3] = {};
    uint32_t off = 0; uint16_t len = 0; bool eof = false; const uint8_t* d = nullptr;
    EXPECT_FALSE(wire::decodeFileChunk(pkt, sizeof(pkt), off, len, eof, &d));
}

TEST(WireFormatFileChunk, EncodeRejectsOversizedLength) {
    // Length field is 16-bit; > 65535 must fail cleanly.
    std::vector<uint8_t> huge(70000);
    uint8_t out[8] = {};
    EXPECT_EQ(wire::encodeFileChunk(0, huge.data(), huge.size(), false, out, sizeof(out)),
              0u);
}
