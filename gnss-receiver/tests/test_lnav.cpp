extern "C" {
#include "gnss/lnav.h"
}

#include <gtest/gtest.h>

#include <vector>

namespace {

// IS-GPS-200 Table 20-XIV, written out again by index (independently of the decoder's masks).
const std::vector<std::vector<int>> kCover = {
    {1, 2, 3, 5, 6, 10, 11, 12, 13, 14, 17, 18, 20, 23},        // D25, with D29*
    {2, 3, 4, 6, 7, 11, 12, 13, 14, 15, 18, 19, 21, 24},        // D26, with D30*
    {1, 3, 4, 5, 7, 8, 12, 13, 14, 15, 16, 19, 20, 22},         // D27, with D29*
    {2, 4, 5, 6, 8, 9, 13, 14, 15, 16, 17, 20, 21, 23},         // D28, with D30*
    {1, 3, 5, 6, 7, 9, 10, 14, 15, 16, 17, 18, 21, 22, 24},     // D29, with D30*
    {3, 5, 6, 8, 9, 10, 11, 13, 15, 19, 22, 23, 24},            // D30, with D29*
};
const int kPrev[6] = {29, 30, 29, 30, 30, 29};

// Encodes 24 source data bits (d1 = bit 23) as the transmitted 30-bit word.
uint32_t encode(uint32_t d, int d29s, int d30s)
{
    uint32_t w = (d30s ? (d ^ 0xFFFFFFu) : d) << 6;
    for (int p = 0; p < 6; p++) {
        int b = kPrev[p] == 29 ? d29s : d30s;
        for (int i : kCover[p]) {
            b ^= (d >> (24 - i)) & 1;
        }
        w |= uint32_t(b) << (5 - p);
    }
    return w;
}

// A subframe: preamble, then HOW with the TOW count and ID; the rest pseudo-random.
std::vector<uint32_t> subframe_data(uint32_t tow_count, int id, uint32_t seed)
{
    std::vector<uint32_t> d(10);
    d[0] = (0x8Bu << 16) | (seed & 0xFFFFu);
    d[1] = (tow_count << 7) | (uint32_t(id) << 2);
    uint32_t x = seed * 2654435761u + 1;
    for (int k = 2; k < 10; k++) {
        x = x * 1664525u + 1013904223u;
        d[k] = x >> 8;
    }
    return d;
}

std::vector<int> transmit(const std::vector<uint32_t> &data, int &d29, int &d30)
{
    std::vector<int> bits;
    for (uint32_t d : data) {
        uint32_t w = encode(d, d29, d30);
        for (int k = 29; k >= 0; k--) {
            bits.push_back((w >> k) & 1);
        }
        d29 = (w >> 1) & 1;
        d30 = w & 1;
    }
    return bits;
}

}  // namespace

TEST(Lnav, ParityAcceptsEncodedWordsAndRejectsFlips)
{
    for (uint32_t d : {0x8B1234u, 0x000000u, 0xFFFFFFu, 0x5A5A5Au}) {
        for (int s = 0; s < 4; s++) {
            int d29 = s >> 1, d30 = s & 1;
            uint32_t w = encode(d, d29, d30), out = 0;
            ASSERT_EQ(lnav_parity(w, d29, d30, &out), 1) << std::hex << d << " " << s;
            EXPECT_EQ(out, d);
            for (int b = 0; b < 30; b += 7) {
                EXPECT_EQ(lnav_parity(w ^ (1u << b), d29, d30, &out), 0);
            }
        }
    }
}

TEST(Lnav, FrameSyncInBothPolaritiesGivesTheTimeOfWeek)
{
    for (int inverted : {0, 1}) {
        lnav_t l;
        lnav_init(&l, 7);
        int d29 = 0, d30 = 0;
        std::vector<int> bits;
        for (int j = 0; j < 37; j++) {  // junk, ending in the D29*, D30* = 0, 0 the encoder chained from
            bits.push_back(j < 35 && (j * 7) % 3 == 0);
        }
        auto sf1 = transmit(subframe_data(1000, 2, 11), d29, d30);
        auto sf2 = transmit(subframe_data(1001, 3, 12), d29, d30);
        bits.insert(bits.end(), sf1.begin(), sf1.end());
        bits.insert(bits.end(), sf2.begin(), sf2.end());
        int ids[2] = {0, 0}, n = 0;
        for (size_t k = 0; k < bits.size(); k++) {
            int b = bits[k] ^ inverted;
            int id = lnav_push(&l, b ? 1 : -1, uint32_t(1000 + 20 * k));
            if (id) {
                if (n < 2) {
                    ids[n] = id;
                }
                n++;
                if (n == 1) {
                    // The first subframe began at bit 37; HOW count 1000 means it began at 6000 - 6 s.
                    EXPECT_EQ(l.sf_period, uint32_t(1000 + 20 * 37));
                    EXPECT_DOUBLE_EQ(l.sf_tow, 5994.0);
                }
            }
        }
        EXPECT_EQ(n, 2) << inverted;
        EXPECT_EQ(ids[0], 2);
        EXPECT_EQ(ids[1], 3);
        EXPECT_EQ(l.inverted, inverted);
        EXPECT_DOUBLE_EQ(l.sf_tow, 6000.0);
    }
}

TEST(Lnav, ACorruptedBitStopsTheSubframe)
{
    lnav_t l;
    lnav_init(&l, 3);
    int d29 = 0, d30 = 0;
    auto bits = transmit(subframe_data(50, 1, 5), d29, d30);
    bits.insert(bits.begin(), {0, 0});
    bits[2 + 30 * 4 + 10] ^= 1;
    int got = 0;
    for (size_t k = 0; k < bits.size(); k++) {
        got |= lnav_push(&l, bits[k] ? 1 : -1, uint32_t(k));
    }
    EXPECT_EQ(got, 0);
    EXPECT_EQ(l.synced, 0);
    EXPECT_GE(l.n_parity_fail, 1u);
}
