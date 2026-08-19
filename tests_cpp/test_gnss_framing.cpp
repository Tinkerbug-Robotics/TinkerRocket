// Host tests for the GNSS bring-up framing test (gnss_framing.h).
//
// The behaviour under test is what stops bring-up wasting ~3 s per wrong baud:
// bytes alone must NOT count as a live link, only real UBX or NMEA framing.
// See the header for the bench measurement that motivated it.

#include <gtest/gtest.h>
#include <cstring>
#include <string>
#include <vector>

#include "gnss_framing.h"

namespace {

std::vector<uint8_t> bytes(const std::string& s)
{
    return std::vector<uint8_t>(s.begin(), s.end());
}

bool framed(const std::vector<uint8_t>& v)
{
    return tr::gnssFramingDetected(v.data(), v.size());
}

/// Build a valid NMEA sentence: '$' + payload + '*' + XOR checksum.
std::string nmea(const std::string& payload)
{
    uint8_t sum = 0;
    for (char c : payload) sum ^= static_cast<uint8_t>(c);
    char tail[8];
    snprintf(tail, sizeof(tail), "*%02X\r\n", sum);
    return "$" + payload + tail;
}

}  // namespace

TEST(GnssFraming, ValidNmeaSentenceIsFramed) {
    EXPECT_TRUE(framed(bytes(nmea("GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W"))));
    EXPECT_TRUE(framed(bytes(nmea("GPGGA,000000.00,,,,,0,00,99.99,,,,,,"))));
}

TEST(GnssFraming, UbxSyncIsFramed) {
    // 0xB5 0x62 + class/id/len — the sync pair is what we key on.
    const std::vector<uint8_t> ubx = {0xB5, 0x62, 0x01, 0x07, 0x5C, 0x00, 0x00};
    EXPECT_TRUE(framed(ubx));
}

TEST(GnssFraming, UbxSyncFoundMidBufferIsFramed) {
    // A window that opens mid-frame still catches the next sync pair.
    std::vector<uint8_t> v = {0x11, 0x9F, 0x00, 0xC3};
    v.insert(v.end(), {0xB5, 0x62, 0x01, 0x07});
    EXPECT_TRUE(framed(v));
}

TEST(GnssFraming, EmptyAndNullAreNotFramed) {
    EXPECT_FALSE(tr::gnssFramingDetected(nullptr, 0));
    EXPECT_FALSE(tr::gnssFramingDetected(nullptr, 16));
    const std::vector<uint8_t> empty;
    EXPECT_FALSE(framed(empty));
}

// The regression this whole change exists for: a wrong-baud sample delivers
// BYTES, which the old `uartAvailable() > 0` accepted, costing ~3 s per baud.
TEST(GnssFraming, WrongBaudGarbageIsNotFramed) {
    // Bit-shredded bytes of the sort a 9600 stream produces when sampled at
    // 38400 — plenty of data, no framing.
    const std::vector<uint8_t> garbage = {
        0xFE, 0x03, 0x80, 0xC1, 0xF0, 0x1F, 0xE0, 0x07, 0x9C, 0x38,
        0xE3, 0x8F, 0x3C, 0xF0, 0xC3, 0x0F, 0xFC, 0x00, 0x7F, 0x81,
    };
    EXPECT_FALSE(framed(garbage));
}

TEST(GnssFraming, NmeaWithBadChecksumIsNotFramed) {
    // Correct shape, wrong checksum — exactly what a near-miss baud can fake.
    EXPECT_FALSE(framed(bytes("$GNRMC,123519,A,4807.038,N*00\r\n")));
}

TEST(GnssFraming, TruncatedNmeaIsNotFramed) {
    // Sentence cut before the checksum digits: not yet provable, so no.
    EXPECT_FALSE(framed(bytes("$GNRMC,123519,A,4807.038,N")));
    EXPECT_FALSE(framed(bytes("$GNRMC,123519,A,4807.038,N*")));
    EXPECT_FALSE(framed(bytes("$GNRMC,123519,A,4807.038,N*4")));
}

TEST(GnssFraming, NonHexChecksumIsNotFramed) {
    EXPECT_FALSE(framed(bytes("$GNRMC,123519,A*ZZ\r\n")));
}

TEST(GnssFraming, SentenceAfterATruncatedOneIsStillFound) {
    // A window that starts mid-sentence must not be poisoned by the fragment:
    // the second, complete sentence still validates.
    const std::string s = "RMC,123519,A,4807.038,N*" + nmea("GNGLL,4916.45,N,12311.12,W,225444,A");
    EXPECT_TRUE(framed(bytes(s)));
}

TEST(GnssFraming, LowercaseChecksumHexIsAccepted) {
    std::string s = nmea("GNGLL,4916.45,N,12311.12,W,225444,A");
    for (char& c : s) c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
    // Lowercasing mangles the payload, so recompute against the lowered text.
    const size_t star = s.rfind('*');
    ASSERT_NE(star, std::string::npos);
    uint8_t sum = 0;
    for (size_t i = 1; i < star; i++) sum ^= static_cast<uint8_t>(s[i]);
    char tail[8];
    snprintf(tail, sizeof(tail), "*%02x\r\n", sum);
    s = s.substr(0, star) + tail;
    EXPECT_TRUE(framed(bytes(s)));
}
