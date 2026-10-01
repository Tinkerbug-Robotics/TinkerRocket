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
    for (size_t i = 0; i < data.size(); i++) {
        uint32_t d = data[i], w = encode(d, d29, d30);
        if (i == 1 || i == 9) {
            // As the satellites do (IS-GPS-200 20.3.3.2): the HOW's and word 10's data bits 23-24
            // are solved so that their parity bits 29-30 come out zero (so the next TLM starts clean).
            for (uint32_t t = 0; t < 4 && (w & 3u) != 0; t++) {
                w = encode((d & ~3u) | t, d29, d30);
            }
        }
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
                    // The first subframe is released when the second's HOW confirms the grid: count
                    // 1001, 300 bits on. The timing is then the second's: it began at 6006 - 6 s.
                    EXPECT_EQ(k, size_t(37 + 300 + 59));  // the second HOW's last bit
                    EXPECT_EQ(l.sf_period, uint32_t(1000 + 20 * 337));
                    EXPECT_DOUBLE_EQ(l.sf_tow, 6000.0);
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

// A corrupted word stops the subframe's data, not its timing: the next subframe's HOW confirms the
// grid 1.2 s into it (milestone 7: in flight, 6 s of clean bits are rare).
TEST(Lnav, ACorruptedWordStopsTheSubframeButNotItsTiming)
{
    lnav_t l;
    lnav_init(&l, 3);
    int d29 = 0, d30 = 0;
    auto bits = transmit(subframe_data(50, 1, 5), d29, d30);
    auto next = transmit(subframe_data(51, 2, 6), d29, d30);
    bits[30 * 4 + 10] ^= 1;
    bits.insert(bits.end(), next.begin(), next.end());
    bits.insert(bits.begin(), {0, 0});
    std::vector<int> ids;
    int synced_at = -1;
    for (size_t k = 0; k < bits.size(); k++) {
        int id = lnav_push(&l, bits[k] ? 1 : -1, uint32_t(1000 + 20 * k));
        if (id) {
            ids.push_back(id);
        }
        if (l.synced && synced_at < 0) {
            synced_at = int(k);
        }
    }
    EXPECT_EQ(synced_at, 2 + 300 + 59);  // the second HOW's last bit
    EXPECT_EQ(l.sf_period, uint32_t(1000 + 20 * 302));
    EXPECT_DOUBLE_EQ(l.sf_tow, 51 * 6.0 - 6.0);
    ASSERT_EQ(ids.size(), 1u);  // only the clean subframe's data
    EXPECT_EQ(ids[0], 2);
    EXPECT_GE(l.n_parity_fail, 1u);
}

// What PSAS's flight showed: an almanac page (the same on every satellite) whose word 9 begins like
// a preamble, with an ID in word 10's bits, gives a HOW-like pair that passes every one-shot check.
// Only the next subframe's HOW, 6 s on with the count one up, sets the time.
TEST(Lnav, AnAlmanacWordThatLooksLikeAPreambleSetsNoTime)
{
    lnav_t l;
    lnav_init(&l, 9);
    int d29 = 0, d30 = 0;
    auto page = subframe_data(10973, 4, 21);
    page[8] = (0x8Bu << 16) | 0x1234u;                  // word 9: a preamble look-alike
    page[9] = (75676u << 7) | (4u << 2) | (page[9] & 3u);  // word 10: an ID of 4, a count far off
    std::vector<int> bits = {0, 0};
    for (auto sf : {page, subframe_data(10974, 5, 22), subframe_data(10975, 1, 23)}) {
        auto b = transmit(sf, d29, d30);
        bits.insert(bits.end(), b.begin(), b.end());
    }
    for (size_t k = 0; k < bits.size(); k++) {
        lnav_push(&l, bits[k] ? 1 : -1, uint32_t(20 * k));
        if (l.synced) {
            EXPECT_LT(l.sf_tow, 604800.0 / 2) << "a time from the almanac's words";
        }
    }
    ASSERT_TRUE(l.synced);
    EXPECT_DOUBLE_EQ(l.sf_tow, 10975 * 6.0 - 6.0);
}

// A bit error in the TLM or HOW keeps the time unset.
TEST(Lnav, ACorruptedHowSetsNoTime)
{
    lnav_t l;
    lnav_init(&l, 3);
    int d29 = 0, d30 = 0;
    auto bits = transmit(subframe_data(50, 1, 5), d29, d30);
    bits.insert(bits.begin(), {0, 0});
    bits[2 + 30 + 12] ^= 1;
    for (size_t k = 0; k < bits.size(); k++) {
        lnav_push(&l, bits[k] ? 1 : -1, uint32_t(k));
    }
    EXPECT_EQ(l.synced, 0);
}

// The 10-bit week resolves against a reference (milestone 7: PSAS's 2015 recording read 1024 weeks
// late with the fixed 2019-2038 era); the default keeps that era.
TEST(Lnav, ResolvesTheWeekAgainstAReference)
{
    EXPECT_EQ(lnav_resolve_week(830, 1854), 1854);
    EXPECT_EQ(lnav_resolve_week(830, LNAV_WEEK_REF), 2878);
    EXPECT_EQ(lnav_resolve_week(384, LNAV_WEEK_REF), 2432);
    EXPECT_EQ(lnav_resolve_week(0, LNAV_WEEK_REF), 2048);
    EXPECT_EQ(lnav_resolve_week(1023, LNAV_WEEK_REF), 3071);
    EXPECT_EQ(lnav_resolve_week(1023, 2049), 2047);
}
