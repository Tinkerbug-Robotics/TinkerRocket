// Host-side tests for the base-station BLE file-list window (#835 item 5).
//
// The bug this replaces: handleFileListCommand() read the directory into
// `FileEntry entries[64]`, stopped at 64, and sorted descending afterwards.
// readdir returns on-disk slot order (creation order on a card nothing has
// been deleted from), so the files it never reached were the NEWEST ones and
// the sort could not rescue them.  With 70 logs on the card, the flight the
// operator just recorded was in no page at all.
//
// The decisive test below is OldTruncateThenSortRegression: it offers names in
// creation order, more of them than the window holds, and asserts the window
// keeps the greatest names rather than the first ones.  Truncate-then-sort
// fails it; stream-and-keep-top-N passes.

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "bs_file_list.h"

using bs_file_list::Entry;
using bs_file_list::TopNames;
using bs_file_list::windowFor;

namespace {

// "lora_001.csv" .. "lora_NNN.csv", in creation order (oldest first) — the
// order readdir hands back on a freshly-filled card.
std::vector<std::string> creationOrder(int n)
{
    std::vector<std::string> v;
    for (int i = 1; i <= n; ++i)
    {
        char b[32];
        snprintf(b, sizeof(b), "lora_%03d.csv", i);
        v.emplace_back(b);
    }
    return v;
}

// What the window holds, top rank first.
std::vector<std::string> drain(const TopNames& w)
{
    std::vector<std::string> out;
    for (size_t i = 0; i < w.count(); ++i) out.emplace_back(w.at(i));
    return out;
}

// The page slice handleFileListCommand() emits, given the same window sizing.
std::vector<std::string> pageOf(const std::vector<std::string>& dir_order,
                                uint32_t page, size_t per_page, size_t max_window)
{
    const size_t want = windowFor(page, per_page, max_window);
    if (want == 0) return {};

    std::vector<Entry> storage(want);
    TopNames w(storage.data(), want);
    for (const auto& n : dir_order) w.offer(n.c_str());

    std::vector<std::string> out;
    const size_t start = (size_t)page * per_page;
    for (size_t i = start; i < w.count() && out.size() < per_page; ++i)
        out.emplace_back(w.at(i));
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// windowFor
// ---------------------------------------------------------------------------

TEST(BsFileListWindowFor, SizesToTheRequestedPage)
{
    EXPECT_EQ(windowFor(0, 5, 250), 5u);
    EXPECT_EQ(windowFor(1, 5, 250), 10u);
    EXPECT_EQ(windowFor(3, 5, 250), 20u);
    EXPECT_EQ(windowFor(49, 5, 250), 250u);  // last page that fits exactly
}

TEST(BsFileListWindowFor, RefusesPagesPastTheCap)
{
    EXPECT_EQ(windowFor(50, 5, 250), 0u);
    EXPECT_EQ(windowFor(255, 5, 250), 0u);  // deepest page a uint8 can ask for
}

TEST(BsFileListWindowFor, DoesNotOverflowOnAHugePage)
{
    // 64-bit intermediate: a page*per_page that would wrap 32 bits must still
    // come back as "too deep", never as a small accidental window.
    EXPECT_EQ(windowFor(0xFFFFFFFFu, 5, 250), 0u);
}

// ---------------------------------------------------------------------------
// The regression the old truncate-then-sort code fails
// ---------------------------------------------------------------------------

TEST(BsFileList, OldTruncateThenSortRegression)
{
    // 70 logs, offered oldest-first exactly as readdir returns them.  The old
    // code stopped at index 63 and then sorted, so it answered page 0 with
    // lora_064..lora_060 and the five newest were unreachable in every page.
    const auto dir = creationOrder(70);

    std::vector<Entry> storage(5);
    TopNames w(storage.data(), 5);
    for (const auto& n : dir) w.offer(n.c_str());

    EXPECT_EQ(drain(w), (std::vector<std::string>{
        "lora_070.csv", "lora_069.csv", "lora_068.csv",
        "lora_067.csv", "lora_066.csv"}));

    // ...and the true directory count survives the window.
    EXPECT_EQ(w.total(), 70u);
    EXPECT_EQ(w.count(), 5u);
}

TEST(BsFileList, NewestFileIsOnPageZeroForAnyDirectorySize)
{
    // Page 0 must not depend on how many files exist — that is the whole point
    // of streaming rather than collecting.
    // Stay under 1000: lora_%03u.csv is a MINIMUM width, so file 1000 is
    // named lora_1000.csv and strcmp-sorts below lora_999.csv.  That ordering
    // break predates this window (the old qsort was strcmp too) and is tracked
    // separately — see SortIsStrcmpNotNumeric below.
    for (int n : {1, 5, 63, 64, 65, 70, 250, 251, 999})
    {
        char newest[32];
        snprintf(newest, sizeof(newest), "lora_%03d.csv", n);
        const auto p0 = pageOf(creationOrder(n), 0, 5, 250);
        ASSERT_FALSE(p0.empty()) << "n=" << n;
        EXPECT_EQ(p0.front(), newest) << "n=" << n;
    }
}

// ---------------------------------------------------------------------------
// Ordering equivalence with a full sort
// ---------------------------------------------------------------------------

TEST(BsFileList, DirectoryThatFitsListsExactlyAsAFullSortWould)
{
    auto dir = creationOrder(12);
    // Shuffle deterministically — readdir order must not matter.
    std::rotate(dir.begin(), dir.begin() + 7, dir.end());
    std::swap(dir[0], dir[9]);

    std::vector<Entry> storage(20);
    TopNames w(storage.data(), 20);
    for (const auto& n : dir) w.offer(n.c_str());

    auto expected = dir;
    std::sort(expected.begin(), expected.end(), std::greater<std::string>());

    EXPECT_EQ(drain(w), expected);
    EXPECT_EQ(w.count(), 12u);
    EXPECT_EQ(w.total(), 12u);
}

TEST(BsFileList, KeepsTopNRegardlessOfOfferOrder)
{
    auto dir = creationOrder(40);
    std::reverse(dir.begin(), dir.end());  // newest first this time

    std::vector<Entry> storage(6);
    TopNames w(storage.data(), 6);
    for (const auto& n : dir) w.offer(n.c_str());

    auto expected = creationOrder(40);
    std::sort(expected.begin(), expected.end(), std::greater<std::string>());
    expected.resize(6);

    EXPECT_EQ(drain(w), expected);
}

// ---------------------------------------------------------------------------
// Paging
// ---------------------------------------------------------------------------

TEST(BsFileList, PagesTileTheDirectoryNewestFirstWithNoGapsOrRepeats)
{
    const auto dir = creationOrder(70);

    std::vector<std::string> seen;
    for (uint32_t page = 0;; ++page)
    {
        const auto p = pageOf(dir, page, 5, 250);
        if (p.empty()) break;
        seen.insert(seen.end(), p.begin(), p.end());
        ASSERT_LT(page, 100u) << "paging did not terminate";
    }

    auto expected = dir;
    std::sort(expected.begin(), expected.end(), std::greater<std::string>());
    EXPECT_EQ(seen, expected);  // every file, once, newest first
}

TEST(BsFileList, ShortPageEndsTheListAndTheNextPageIsEmpty)
{
    const auto dir = creationOrder(12);            // 2 full pages + 2
    EXPECT_EQ(pageOf(dir, 2, 5, 250).size(), 2u);  // the short page
    EXPECT_TRUE(pageOf(dir, 3, 5, 250).empty());
}

TEST(BsFileList, PagePastTheCapIsEmptyRatherThanWrong)
{
    // Better an honest "list ended" than a page built from the wrong window.
    EXPECT_TRUE(pageOf(creationOrder(400), 50, 5, 250).empty());
}

TEST(BsFileList, PageBeyondTheDirectoryIsEmpty)
{
    EXPECT_TRUE(pageOf(creationOrder(3), 1, 5, 250).empty());
    EXPECT_TRUE(pageOf({}, 0, 5, 250).empty());
}

// ---------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------

// Ordering is strcmp, not numeric — the same contract the old qsort had.
// This bites once the no-RTC sequential name crosses 4 digits: lora_%03u.csv
// is a minimum width and parseSequentialFilename() accepts up to 65535, so
// lora_1000.csv lands between lora_0xx and lora_2xx instead of on top.  Pinned
// here so the follow-up fix has a failing expectation to flip.
TEST(BsFileList, SortIsStrcmpNotNumeric)
{
    std::vector<Entry> storage(3);
    TopNames w(storage.data(), 3);
    w.offer("lora_999.csv");
    w.offer("lora_1000.csv");
    w.offer("lora_1001.csv");
    EXPECT_STREQ(w.at(0), "lora_999.csv");   // numerically the OLDEST of the three
    EXPECT_STREQ(w.at(1), "lora_1001.csv");
    EXPECT_STREQ(w.at(2), "lora_1000.csv");
}

TEST(BsFileList, TotalCountsEveryOfferedName)
{
    std::vector<Entry> storage(3);
    TopNames w(storage.data(), 3);
    for (const auto& n : creationOrder(500)) w.offer(n.c_str());
    EXPECT_EQ(w.total(), 500u);
    EXPECT_EQ(w.count(), 3u);
}

TEST(BsFileList, TruncatesOverlongNamesAndSortsByTheStoredForm)
{
    std::vector<Entry> storage(2);
    TopNames w(storage.data(), 2);
    // 40 chars — longer than NAME_CAP-1 (31).
    w.offer("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    w.offer("b");
    ASSERT_EQ(w.count(), 2u);
    EXPECT_STREQ(w.at(0), "b");
    EXPECT_EQ(std::string(w.at(1)).size(), bs_file_list::NAME_CAP - 1);
    EXPECT_EQ(std::string(w.at(1)), std::string(31, 'a'));
}

TEST(BsFileList, HandlesEqualNames)
{
    std::vector<Entry> storage(3);
    TopNames w(storage.data(), 3);
    w.offer("same.csv");
    w.offer("same.csv");
    w.offer("same.csv");
    w.offer("same.csv");
    EXPECT_EQ(w.count(), 3u);
    EXPECT_EQ(w.total(), 4u);
    for (size_t i = 0; i < w.count(); ++i) EXPECT_STREQ(w.at(i), "same.csv");
}

TEST(BsFileList, ZeroCapacityAndNullStorageAreSafe)
{
    TopNames zero(nullptr, 0);
    zero.offer("lora_001.csv");
    EXPECT_EQ(zero.count(), 0u);
    EXPECT_EQ(zero.total(), 1u);
    EXPECT_EQ(zero.at(0), nullptr);

    Entry buf[1];
    TopNames null_storage(nullptr, 4);  // capacity is clamped to 0
    EXPECT_EQ(null_storage.capacity(), 0u);
    null_storage.offer("lora_001.csv");
    EXPECT_EQ(null_storage.count(), 0u);
    (void)buf;
}

TEST(BsFileList, NullNameIsIgnoredButStillCounted)
{
    std::vector<Entry> storage(2);
    TopNames w(storage.data(), 2);
    w.offer("lora_001.csv");
    w.offer(nullptr);
    EXPECT_EQ(w.count(), 1u);
    EXPECT_EQ(w.total(), 2u);
}

TEST(BsFileList, AtIsOutOfRangeSafe)
{
    std::vector<Entry> storage(4);
    TopNames w(storage.data(), 4);
    w.offer("a.csv");
    EXPECT_STREQ(w.at(0), "a.csv");
    EXPECT_EQ(w.at(1), nullptr);   // allocated but unfilled
    EXPECT_EQ(w.at(99), nullptr);
}
