#pragma once

#include <stdint.h>

// #1659: keep the launch-edge flight-token write (W2) a plain append.
//
// W2 rewrites the #1176 token at the PRELAUNCH -> INFLIGHT edge, in the boost.
// Most rewrites are appends: NVS writes the new copy after the last entry on
// the ACTIVE page and marks the old one erased, which is programming only.
// But when the active page has no room left, the write makes NVS take a new
// page, and once the partition is down to its one reserved free page that
// means reclaiming a full page with a 4 KB sector erase. On the Beetle's
// 2026-10-24 RIM66 flight that erase cut ~40 ms out of the log. The V5 bench
// (2026-10-25) measured 59-65 ms per erase, and showed the I2C poll from the
// FC stalling through it too. A page takes twelve token writes, so it fills
// every four or five sessions, and the write that fills it is W2 on about
// one flight in eleven (tests_cpp/test_token_page_policy.cpp).
//
// So the room is made on the pad. After W1 (ARMED, on reaching PRELAUNCH)
// the OC checks that W2 and one spare write both fit on the active page, and
// if they do not it rewrites the token until NVS has moved to a fresh page.
// Any erase that costs then lands on the pad, where a stall is harmless.
//
// The check reads the page from flash, because nvs_get_stats cannot answer
// it: its free_entries counts the erased entries on FULL pages too, so with a
// single live key it reads the same whatever page is active.
//
// The format is ESP-IDF's NVS on-flash layout (components/nvs_flash/
// private_include/nvs_constants.h, the "NVS internals" chapter of the IDF
// guide). Each 4 KB page has a 32-byte header, whose first word is the
// page state, then a 32-byte entry-state table (2 bits per entry, packed
// little-endian, 0b11 = EMPTY), then 126 entries of 32 bytes.
//
// Pure: no IDF, the caller does the flash reads, so tests_cpp drives it.
namespace TokenPagePolicy {

inline constexpr uint32_t kPageSize         = 4096;
inline constexpr uint32_t kEntrySize        = 32;
inline constexpr uint32_t kEntryCount       = 126;
inline constexpr uint32_t kEntryTableOffset = 32;
inline constexpr uint32_t kEntryTableBytes  = 32;
inline constexpr uint32_t kEntryDataOffset  = 64;
inline constexpr uint32_t kPageStateActive  = 0xFFFFFFFEu;
inline constexpr uint8_t  kEntryStateEmpty  = 0x3;

// Entries a blob of `len` bytes takes when it lands on one page: the data
// item's own header entry, its data, and the blob-index entry
// (Storage::writeMultiPageBlob). 10 for the 248-byte token.
//
// It is also the threshold. writeMultiPageBlob requests a new page whenever
// the data and its index do not both fit; Page::getVarDataTailroom reserves
// the data item's header, so "fits" means exactly this many free entries.
inline constexpr uint32_t blobEntries(uint32_t len)
{
    return 1 + (len + kEntrySize - 1) / kEntrySize + 1;
}

// Where NVS appends next: the first EMPTY entry in the page's state table, as
// Page::load finds it (kEntryCount when there is none). Page::load then steps
// over any entry from there whose first word is written, which a power cut
// mid-write leaves behind. The caller reads those words and applies that part.
inline uint32_t firstEmptyEntry(const uint8_t table[kEntryTableBytes])
{
    for (uint32_t i = 0; i < kEntryCount; ++i)
    {
        const uint8_t state = (uint8_t)((table[i / 4] >> ((i % 4) * 2)) & 0x3);
        if (state == kEntryStateEmpty) return i;
    }
    return kEntryCount;
}

// Do `writes` writes of `entries_per_write` each, one after another, all land
// on the active page? A page request is the only place NVS erases
// (PageManager::requestNewPage), and none is made while this holds. A page
// change can also be erase-free, when two or more free pages remain, but
// that is not relied on here: this sufficient condition needs no view of
// the other pages.
inline constexpr bool writesFitOnActivePage(uint32_t free_entries,
                                            uint32_t entries_per_write,
                                            uint32_t writes)
{
    return free_entries >= entries_per_write * writes;
}

// What W1 makes room for: W2, plus one spare. The spare covers an OC reset in
// the air, whose boot consumes a restore attempt with a write of its own.
inline constexpr uint32_t kWritesToCover = 2;

// Rewrites the turn may spend. Two always suffice: one takes a page that fits
// one write but not two below one write's worth, and the next makes NVS take
// a fresh page. The third is slack.
inline constexpr uint32_t kMaxTurnWrites = 3;

}  // namespace TokenPagePolicy
