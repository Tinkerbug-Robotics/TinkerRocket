// #1659: the launch-edge flight-token write (W2) must not pay for a sector
// erase in the boost, so W1 makes room for it on the pad.
//
// The load-bearing row is the cross-check against IDF's own rule: the
// 10-entry threshold is derived from Storage::writeMultiPageBlob, and if that
// derivation is wrong for any page fill, a W2 that "fits" would still make a
// page request -- the one place NVS erases.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>

#include "token_page_policy.h"

using namespace TokenPagePolicy;

namespace {

constexpr uint32_t kTokenBytes = 248;   // sizeof(FlightToken) in out_computer/main.cpp

// IDF v6.0.1's rule, transcribed from Storage::writeMultiPageBlob and
// Page::getVarDataTailroom: does writing a `len`-byte blob onto the ACTIVE
// page, whose next free entry is `next`, request a new page? Only for blobs
// up to CHUNK_MAX_SIZE / 10, which never split across pages.
bool idfRequestsPage(uint32_t next, uint32_t len)
{
    const uint32_t chunk_max = kEntrySize * (kEntryCount - 1);
    const uint32_t tailroom =
        (next < kEntryCount - 1) ? (kEntryCount - next - 1) * kEntrySize : 0;
    if (tailroom < len && tailroom < chunk_max / 10) return true;   // first chunk too small
    if (tailroom == 0) return true;
    const uint32_t chunk = std::min(len, tailroom);
    return (len - chunk) > 0 || (tailroom - chunk) < kEntrySize;     // split, or no room for the index
}

void setEntryState(uint8_t table[kEntryTableBytes], uint32_t i, uint8_t state)
{
    const uint32_t shift = (i % 4) * 2;
    table[i / 4] = (uint8_t)((table[i / 4] & ~(0x3u << shift)) | ((state & 0x3u) << shift));
}

constexpr uint8_t kWritten = 0x2;
constexpr uint8_t kErased  = 0x0;

}  // namespace

TEST(TokenPagePolicy, TheTokenIsTenEntries)
{
    // Data item header, 8 entries of data, blob index.
    EXPECT_EQ(blobEntries(kTokenBytes), 10u);
    EXPECT_EQ(blobEntries(32), 3u);
    EXPECT_EQ(blobEntries(33), 4u);
}

TEST(TokenPagePolicy, TheThresholdIsIdfsOwnRuleForEveryPageFill)
{
    for (uint32_t len = 1; len <= kEntrySize * (kEntryCount - 1) / 10; ++len)
    {
        for (uint32_t next = 0; next <= kEntryCount; ++next)
        {
            const bool fits = writesFitOnActivePage(kEntryCount - next, blobEntries(len), 1);
            EXPECT_EQ(fits, !idfRequestsPage(next, len)) << "len " << len << " next " << next;
        }
    }
}

TEST(TokenPagePolicy, TwoWritesNeedTwiceTheRoom)
{
    EXPECT_TRUE(writesFitOnActivePage(20, 10, kWritesToCover));
    EXPECT_FALSE(writesFitOnActivePage(19, 10, kWritesToCover));
    EXPECT_TRUE(writesFitOnActivePage(10, 10, 1));
    EXPECT_FALSE(writesFitOnActivePage(9, 10, 1));
}

TEST(TokenPagePolicy, AnErasedPageAppendsAtTheStart)
{
    uint8_t table[kEntryTableBytes];
    std::memset(table, 0xFF, sizeof(table));
    EXPECT_EQ(firstEmptyEntry(table), 0u);
}

TEST(TokenPagePolicy, NextFreeIsTheFirstEmptyEntry)
{
    uint8_t table[kEntryTableBytes];
    std::memset(table, 0xFF, sizeof(table));
    // The namespace entry, two token copies (the older one erased), then the
    // live one: 31 entries used, in NVS's append order.
    for (uint32_t i = 0; i < 31; ++i)
        setEntryState(table, i, (i >= 1 && i < 21) ? kErased : kWritten);
    EXPECT_EQ(firstEmptyEntry(table), 31u);

    setEntryState(table, 31, kWritten);   // entries 31..33 cross a byte boundary
    setEntryState(table, 32, kErased);
    setEntryState(table, 33, kWritten);
    EXPECT_EQ(firstEmptyEntry(table), 34u);
}

TEST(TokenPagePolicy, AFullTableHasNoFreeEntry)
{
    uint8_t table[kEntryTableBytes];
    std::memset(table, 0xFF, sizeof(table));
    for (uint32_t i = 0; i < kEntryCount; ++i) setEntryState(table, i, kErased);
    EXPECT_EQ(firstEmptyEntry(table), kEntryCount);

    setEntryState(table, kEntryCount - 1, 0x3);   // only the last one left
    EXPECT_EQ(firstEmptyEntry(table), kEntryCount - 1);
}

namespace {

// The token partition at steady state, one session after another. A flight is
// W1 (ARMED), W2 (INFLIGHT), W4 (the LANDED clear); a session that never
// launches is W1 and the power-off clear. A write that does not fit makes a
// page request -- an erase once the partition is down to its reserved free
// page -- and lands on the fresh page, which starts with at most the
// namespace entry copied over. Returns how many flights' W2 made a request.
uint32_t runSessions(uint32_t start_free, uint32_t seed, int sessions, bool turn)
{
    const uint32_t E = blobEntries(kTokenBytes);
    uint32_t free = start_free;
    uint32_t w2_requests = 0;
    auto write = [&]() {
        const bool requested = !writesFitOnActivePage(free, E, 1);
        if (requested) free = kEntryCount - 1;
        free -= E;
        return requested;
    };
    for (int s = 0; s < sessions; ++s)
    {
        seed = seed * 1103515245u + 12345u;
        const bool flight = ((seed >> 16) % 3) != 0;   // two sessions in three fly
        (void)write();                                  // W1
        if (turn)
        {
            uint32_t rewrites = 0;
            while (!writesFitOnActivePage(free, E, kWritesToCover))
            {
                EXPECT_LT(rewrites, kMaxTurnWrites);
                if (rewrites++ == kMaxTurnWrites) break;
                (void)write();
            }
        }
        if (flight && write()) ++w2_requests;           // W2
        (void)write();                                  // W4, or the power-off clear
    }
    return w2_requests;
}

}  // namespace

TEST(TokenPagePolicy, WithTheTurnW2NeverRequestsAPage)
{
    for (uint32_t start = 0; start <= kEntryCount; ++start)
        for (uint32_t seed = 1; seed <= 8; ++seed)
            EXPECT_EQ(runSessions(start, seed, 200, true), 0u)
                << "start " << start << " seed " << seed;
}

TEST(TokenPagePolicy, WithoutItSomeFlightsPayForTheErase)
{
    // The control, so the row above is not vacuous: without the turn the same
    // sessions put W2 on the page change on about one flight in eleven (36 of
    // the 409 flights here). A page takes twelve token writes, a page change
    // comes every four or five sessions, and W2 is one write in four. One of
    // the six logs flown on 2026-10-24 had the gap.
    const uint32_t hits = runSessions(0, 1, 600, false);
    EXPECT_GT(hits, 20u);
    EXPECT_LT(hits, 70u);
}
