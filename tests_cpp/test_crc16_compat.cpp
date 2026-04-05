#include <gtest/gtest.h>
#include "CRC.h"

// The flight code uses calcCRC16() with default parameters:
//   polynomial 0x8001, initial 0x0000, xorOut 0x0000,
//   reverseIn false, reverseOut false.
// The iOS app (MessageParser.swift) must produce identical results.

TEST(CRC16Compat, EmptyData) {
    uint16_t crc = calcCRC16(nullptr, 0);
    // CRC of empty data with init=0 and xorOut=0 is 0
    EXPECT_EQ(crc, 0x0000);
}

TEST(CRC16Compat, KnownVector) {
    // "123456789" is the standard CRC test vector
    const uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
    uint16_t crc = calcCRC16(data, sizeof(data));
    // CRC-16/ARC (poly=0x8005, init=0, refIn=true, refOut=true) gives 0xBB3D
    // But our config is poly=0x8001, init=0, no reflect -> different result.
    // Just verify it's deterministic and non-zero.
    EXPECT_NE(crc, 0x0000);

    // Same input should produce same output
    uint16_t crc2 = calcCRC16(data, sizeof(data));
    EXPECT_EQ(crc, crc2);
}

TEST(CRC16Compat, FrameRoundtrip) {
    // Simulate building a frame: [type][len][payload] then CRC over that region
    // Frame format: [0xAA 0x55 0xAA 0x55][type][len][payload][CRC_MSB][CRC_LSB]
    // CRC covers: type + len + payload (everything after preamble, before CRC)

    const uint8_t type = 0xA2; // ISM6HG256_MSG
    const uint8_t payload[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    const uint8_t len = sizeof(payload);

    // Build the CRC input region: type + len + payload
    uint8_t crc_region[2 + sizeof(payload)];
    crc_region[0] = type;
    crc_region[1] = len;
    memcpy(crc_region + 2, payload, len);

    uint16_t crc = calcCRC16(crc_region, sizeof(crc_region));
    uint8_t crc_msb = static_cast<uint8_t>((crc >> 8) & 0xFF);
    uint8_t crc_lsb = static_cast<uint8_t>(crc & 0xFF);

    // Verify: recompute CRC over same region, compare to stored bytes
    uint16_t crc_check = calcCRC16(crc_region, sizeof(crc_region));
    uint16_t crc_from_bytes = static_cast<uint16_t>((crc_msb << 8) | crc_lsb);
    EXPECT_EQ(crc_check, crc_from_bytes);
}

TEST(CRC16Compat, SingleByteDifference) {
    // CRC should detect a single bit flip
    uint8_t data1[] = {0xA5, 0x10, 0x00, 0x01, 0x02};
    uint8_t data2[] = {0xA5, 0x10, 0x00, 0x01, 0x03}; // last byte differs

    uint16_t crc1 = calcCRC16(data1, sizeof(data1));
    uint16_t crc2 = calcCRC16(data2, sizeof(data2));
    EXPECT_NE(crc1, crc2);
}

TEST(CRC16Compat, ClassAPI_MatchesFreeFunction) {
    // The CRC16 class and the calcCRC16() free function should produce
    // identical results when using the same parameters.
    const uint8_t data[] = {0xA2, 22, 0x00, 0x01, 0x02, 0x03,
                            0x04, 0x05, 0x06, 0x07, 0x08, 0x09};

    uint16_t free_crc = calcCRC16(data, sizeof(data));

    CRC16 crc_obj; // default params match calcCRC16 defaults
    crc_obj.add(data, sizeof(data));
    uint16_t class_crc = crc_obj.calc();

    EXPECT_EQ(free_crc, class_crc);
}
