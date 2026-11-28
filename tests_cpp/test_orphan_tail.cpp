// #1717: TR_FlightLog::newestOrphanTail, the out computer's quick way to the
// last snapshot of a flight a reboot cut off, before the deferred scan (which
// reads every page, minutes for a long flight) has indexed it.
//
// Run against the real TR_FlightLog over the fake NAND at both page sizes. The
// reference answer is the #846 path: let runDeferredRecovery() index the
// orphan as flight_recovered_<id>.bin, then tailScanForFrame over it. The quick
// path must find the same last and best frames, from far fewer reads.

#include <gtest/gtest.h>

#include "SnapshotTailScan.h"
#include "TR_FlightLog.h"
#include "fakes/fake_nand_backend.h"
#include "fakes/memory_bitmap_store.h"
#include <CRC.h>

#include <cstring>
#include <memory>
#include <vector>

using tr_flightlog::FrameTailSearch;
using tr_flightlog::OrphanTailFrameScan;
using tr_flightlog::PageHeader;
using tr_flightlog::Status;
using tr_flightlog::TR_FlightLog;
using tr_flightlog::tailScanForFrame;
using tr_flightlog_test::FakeNandBackend;
using tr_flightlog_test::MemoryBitmapStore;

namespace {

constexpr uint8_t kSnapType   = 0xD2;   // SNAPSHOT_MSG
constexpr uint8_t kPayloadLen = 224;    // sizeof(FlightSnapshotData)
constexpr size_t  kFrameLen   = 4 + 1 + 1 + kPayloadLen + 2;
constexpr uint16_t kOrderOff  = 0;      // the synthetic payload's order field

// A snapshot-shaped frame whose ordering field (payload offset 0) is `order`.
std::vector<uint8_t> frame(uint32_t order, uint8_t seed)
{
    std::vector<uint8_t> f{0xAA, 0x55, 0xAA, 0x55, kSnapType, kPayloadLen};
    std::vector<uint8_t> pl(kPayloadLen);
    memcpy(pl.data(), &order, sizeof(order));
    for (int i = 4; i < kPayloadLen; ++i) pl[i] = static_cast<uint8_t>(seed + i * 3);
    f.insert(f.end(), pl.begin(), pl.end());
    const uint16_t crc = calcCRC16(f.data() + 4, 2 + kPayloadLen);
    f.push_back((crc >> 8) & 0xFF);
    f.push_back(crc & 0xFF);
    return f;
}

void add(std::vector<uint8_t>& s, const std::vector<uint8_t>& v) { s.insert(s.end(), v.begin(), v.end()); }
void pad(std::vector<uint8_t>& s, size_t n, uint8_t b) { s.insert(s.end(), n, b); }

TR_FlightLog::Config deferred()
{
    TR_FlightLog::Config c;
    c.defer_recovery = true;
    return c;
}

// One chip across boots. Each boot is a fresh TR_FlightLog over the same NAND
// and bitmap store, as a reboot is.
struct Chip
{
    FakeNandBackend   nand;
    MemoryBitmapStore store;
    explicit Chip(uint32_t page) : nand(page, 64, 256) {}

    // A boot that logs `stream` into one flight and then loses power (no
    // finalize). Returns the flight id; `start` gets its first block.
    uint32_t flyAndCut(const std::vector<uint8_t>& stream, uint32_t* start = nullptr,
                       bool defer = true)
    {
        TR_FlightLog fl;
        EXPECT_EQ(fl.begin(nand, defer ? deferred() : TR_FlightLog::Config{}, &store), Status::Ok);
        uint32_t id = 0;
        EXPECT_EQ(fl.prepareFlight(id), Status::Ok);
        if (start) *start = fl.activeStartBlock();
        writeStream(fl, stream);
        return id;
    }
    static void writeStream(TR_FlightLog& fl, const std::vector<uint8_t>& stream)
    {
        size_t off = 0;
        while (off < stream.size()) {
            const size_t n = std::min<size_t>(fl.payloadPerPage(), stream.size() - off);
            ASSERT_EQ(fl.writeFrame(stream.data() + off, n), Status::Ok);
            off += n;
        }
    }
};

struct QuickResult
{
    Status st = Status::Error;
    TR_FlightLog::OrphanTail ot;
    bool found = false;
    bool overflow = false;
    uint8_t last[kFrameLen] = {};
    uint8_t best[kFrameLen] = {};
};

QuickResult quick(TR_FlightLog& fl, uint32_t window_bytes = 64 * 1024)
{
    static uint8_t scratch[(4096 - 16) + kFrameLen];
    QuickResult r;
    OrphanTailFrameScan scan;
    scan.fs.frame_type  = kSnapType;
    scan.fs.payload_len = kPayloadLen;
    scan.fs.last_out    = r.last;
    scan.fs.best_out    = r.best;
    scan.fs.order_off   = kOrderOff;
    scan.scratch        = scratch;
    scan.scratch_len    = sizeof(scratch);
    const uint32_t pages = (window_bytes + fl.payloadPerPage() - 1) / fl.payloadPerPage();
    r.st = fl.newestOrphanTail(pages, &OrphanTailFrameScan::sink, &scan, r.ot);
    r.found = scan.fs.found;
    r.overflow = scan.overflow;
    return r;
}

// The #846 reference: index the orphan with the deferred scan, then scan the
// tail of the recovered flight.
struct ScanResult
{
    bool found = false;
    uint8_t last[kFrameLen] = {};
    uint8_t best[kFrameLen] = {};
    uint32_t final_bytes = 0;
};

ScanResult indexedScan(TR_FlightLog& fl, uint32_t id)
{
    static uint8_t scratch[(4096 - 16) + kFrameLen];
    ScanResult r;
    EXPECT_EQ(fl.runDeferredRecovery(), Status::Ok);
    char name[40];
    snprintf(name, sizeof(name), "flight_recovered_%lu.bin", (unsigned long)id);
    for (size_t i = 0; i < fl.index().size(); ++i)
        if (strcmp(fl.index().at(i).filename, name) == 0) r.final_bytes = fl.index().at(i).final_bytes;
    EXPECT_GT(r.final_bytes, 0u);
    r.found = tailScanForFrame(fl, name, r.final_bytes, kSnapType, kPayloadLen, 64 * 1024,
                               r.last, scratch, sizeof(scratch), r.best, kOrderOff);
    return r;
}

// A flight of `pages` full pages with a snapshot every ~`every` bytes, the
// newest last, and an older replay just before the end (#383's case: stream
// order and payload order disagree).
std::vector<uint8_t> flightStream(size_t payload, uint32_t pages, size_t every = 3000)
{
    std::vector<uint8_t> s;
    uint32_t order = 1000;
    const size_t total = payload * pages;
    while (s.size() + kFrameLen + every < total) {
        pad(s, every, 0x11);
        add(s, frame(order, static_cast<uint8_t>(order)));
        order += 100;
    }
    add(s, frame(order - 250, 0x5A));   // a replay, older than the frame before it
    pad(s, total - s.size(), 0x22);
    return s;
}

class OrphanTailTest : public ::testing::TestWithParam<uint32_t> {};

}  // namespace

TEST_P(OrphanTailTest, FindsTheSameFramesAsTheScanOnceItHasIndexedTheFlight)
{
    Chip chip(GetParam());
    const uint32_t payload = GetParam() - 16;
    const uint32_t pages = 64 * 3 + 17;   // into a fourth block
    uint32_t start = 0;
    const uint32_t id = chip.flyAndCut(flightStream(payload, pages), &start);

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    const QuickResult q = quick(fl);
    ASSERT_EQ(q.st, Status::Ok);
    EXPECT_EQ(q.ot.flight_id, id);
    EXPECT_EQ(q.ot.start_block, start);
    EXPECT_EQ(q.ot.last_seq, pages - 1);
    EXPECT_TRUE(q.ot.end_fed);
    EXPECT_EQ(q.ot.gaps, 0u);
    ASSERT_TRUE(q.found);
    EXPECT_FALSE(q.overflow);

    const ScanResult s = indexedScan(fl, id);
    ASSERT_TRUE(s.found);
    EXPECT_EQ(s.final_bytes, pages * payload) << "the scan's length is (last_seq + 1) pages";
    EXPECT_EQ(0, memcmp(q.last, s.last, kFrameLen)) << "the stream's final word";
    EXPECT_EQ(0, memcmp(q.best, s.best, kFrameLen)) << "the newest by its order field";
    uint32_t last_order = 0, best_order = 0;
    memcpy(&last_order, q.last + 6, 4);
    memcpy(&best_order, q.best + 6, 4);
    EXPECT_LT(last_order, best_order) << "the replay is last, the newest is best";
}

TEST_P(OrphanTailTest, ReadsFarFewerPagesThanTheScan)
{
    Chip chip(GetParam());
    const uint32_t payload = GetParam() - 16;
    const uint32_t pages = 64 * 12 + 5;   // a 13-block flight
    const uint32_t id = chip.flyAndCut(flightStream(payload, pages, 20000));

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    const uint64_t r0 = chip.nand.readCount();
    const QuickResult q = quick(fl);
    const uint64_t quick_reads = chip.nand.readCount() - r0;
    ASSERT_EQ(q.st, Status::Ok);
    EXPECT_EQ(q.ot.pages_read, quick_reads);

    const uint64_t r1 = chip.nand.readCount();
    (void)indexedScan(fl, id);
    const uint64_t scan_reads = chip.nand.readCount() - r1;
    // page 0 of each orphaned block, the last block from the top, and the
    // window (17 or 33 pages)
    const uint32_t orphaned_blocks = fl.config().flight_region_end - fl.config().flight_region_start;
    EXPECT_LE(quick_reads, orphaned_blocks + 64u + 34u + 2u);
    EXPECT_LT(quick_reads * 4u, scan_reads) << "quick " << quick_reads << ", scan " << scan_reads;
}

TEST_P(OrphanTailTest, AFrameAcrossABlockBoundaryParses)
{
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    std::vector<uint8_t> s;
    pad(s, payload * 64 - kFrameLen / 2, 0x33);   // the frame straddles blocks 0 and 1
    const auto f = frame(7777, 0x44);
    add(s, f);
    pad(s, 100, 0x55);
    chip.flyAndCut(s);

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    const QuickResult q = quick(fl);
    ASSERT_EQ(q.st, Status::Ok);
    ASSERT_TRUE(q.found);
    EXPECT_EQ(0, memcmp(q.last, f.data(), kFrameLen));
}

TEST_P(OrphanTailTest, ABadBlockMidFlightIsPassedOver)
{
    // A block that failed to program mid-flight is marked BAD and the flight
    // goes on in the next block: the window spans the gap.
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    uint32_t id = 0, start = 0;
    std::vector<uint8_t> tail;
    {
        TR_FlightLog fl;
        ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
        ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
        start = fl.activeStartBlock();
        std::vector<uint8_t> s;
        pad(s, payload * 63, 0x10);
        add(s, frame(5000, 0x01));        // straddles into what should be block 1
        pad(s, payload, 0x10);
        chip.nand.injectProgramFailOnce(start + 1, 0);
        Chip::writeStream(fl, s);
        add(tail, frame(6000, 0x02));
        pad(tail, payload / 2, 0x20);
        Chip::writeStream(fl, tail);
    }
    ASSERT_TRUE(chip.nand.isBlockBad(start + 1));

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    const QuickResult q = quick(fl);
    ASSERT_EQ(q.st, Status::Ok);
    ASSERT_TRUE(q.found);
    EXPECT_EQ(q.ot.gaps, 0u) << "a bad block's pages were moved on: nothing is lost";
    const ScanResult s = indexedScan(fl, id);
    ASSERT_TRUE(s.found);
    EXPECT_EQ(0, memcmp(q.last, s.last, kFrameLen));
    EXPECT_EQ(0, memcmp(q.best, s.best, kFrameLen));
    uint32_t order = 0;
    memcpy(&order, q.last + 6, 4);
    EXPECT_EQ(order, 6000u);
}

TEST_P(OrphanTailTest, AProgramFailureMidBlockIsPassedOver)
{
    // A program failure at page 10 of a block: its ten pages are moved to the
    // next block and the failed page retried after them, with the header it
    // was built with, so seq_number jumps inside that block.
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    uint32_t id = 0, start = 0;
    {
        TR_FlightLog fl;
        ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
        ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
        start = fl.activeStartBlock();
        chip.nand.injectProgramFailOnce(start + 1, 10);
        Chip::writeStream(fl, flightStream(payload, 64 + 40, 2500));
    }
    ASSERT_TRUE(chip.nand.isBlockBad(start + 1));

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    const QuickResult q = quick(fl);
    ASSERT_EQ(q.st, Status::Ok);
    EXPECT_TRUE(q.ot.end_fed);
    ASSERT_TRUE(q.found);
    const ScanResult s = indexedScan(fl, id);
    ASSERT_TRUE(s.found);
    EXPECT_EQ(0, memcmp(q.last, s.last, kFrameLen));
    EXPECT_EQ(0, memcmp(q.best, s.best, kFrameLen));
}

TEST_P(OrphanTailTest, ATornLastPageIsPassedOver)
{
    // A cut mid-program leaves a page that fails its CRC above the flight's
    // last good page: its frames are lost, the ones before it are found.
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    std::vector<uint8_t> s;
    pad(s, payload * 2, 0x10);
    const auto good = frame(100, 0x01);
    add(s, good);
    pad(s, payload - kFrameLen, 0x10);    // three full pages
    uint32_t start = 0;
    chip.flyAndCut(s, &start);

    // The torn fourth page: a header and a frame, the CRC wrong.
    std::vector<uint8_t> torn(GetParam(), 0x00);
    PageHeader hdr;
    memcpy(&hdr, chip.nand.peekPage(start, 0), sizeof(hdr));
    hdr.seq_number = 3;
    hdr.crc32 ^= 0xDEADBEEF;
    memcpy(torn.data(), &hdr, sizeof(hdr));
    const auto lost = frame(200, 0x02);
    memcpy(torn.data() + sizeof(hdr), lost.data(), kFrameLen);
    ASSERT_TRUE(chip.nand.programPage(start, 3, torn.data()));

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    const QuickResult q = quick(fl);
    ASSERT_EQ(q.st, Status::Ok);
    EXPECT_EQ(q.ot.last_seq, 2u);
    ASSERT_TRUE(q.found);
    EXPECT_EQ(0, memcmp(q.last, good.data(), kFrameLen));
}

TEST_P(OrphanTailTest, TheLastPageReadingBackBadTheSecondTimeLeavesTheEndUnfed)
{
    // The stream's final word must come from the last page: if that page reads
    // back bad when the window reaches it, the caller is told.
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    std::vector<uint8_t> s;
    pad(s, payload * 5, 0x10);
    uint32_t start = 0;
    chip.flyAndCut(s, &start);

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    int reads_of_last = 0;
    chip.nand.after_read = [&](uint32_t b, uint32_t p) {
        if (b == start && p == 4 && ++reads_of_last == 1) chip.nand.injectReadErrorPersistent(start, 4);
    };
    const QuickResult q = quick(fl);
    chip.nand.after_read = nullptr;
    ASSERT_EQ(q.st, Status::Ok);
    EXPECT_EQ(q.ot.last_seq, 4u);
    EXPECT_FALSE(q.ot.end_fed);
    EXPECT_EQ(q.ot.gaps, 1u);
}

TEST_P(OrphanTailTest, AnUnreadablePageInTheWindowIsCountedAndBreaksTheFrameAcrossIt)
{
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    std::vector<uint8_t> s;
    pad(s, payload - kFrameLen / 2, 0x10);
    add(s, frame(300, 0x03));             // across pages 0 and 1
    pad(s, payload * 2, 0x10);
    const auto after = frame(400, 0x04);  // on page 3
    add(s, after);
    pad(s, payload / 2, 0x10);
    uint32_t start = 0;
    chip.flyAndCut(s, &start);
    chip.nand.injectReadErrorPersistent(start, 1);

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    const QuickResult q = quick(fl);
    ASSERT_EQ(q.st, Status::Ok);
    EXPECT_TRUE(q.ot.end_fed);
    EXPECT_EQ(q.ot.gaps, 1u) << "the caller refuses a tail with a page it could not read";
    ASSERT_TRUE(q.found);
    EXPECT_EQ(0, memcmp(q.last, after.data(), kFrameLen));
    uint32_t best = 0;
    memcpy(&best, q.best + 6, 4);
    EXPECT_EQ(best, 400u) << "the frame across the unreadable page is not found";
}

TEST_P(OrphanTailTest, APageFailingItsCrcInsideTheWindowIsNotRead)
{
    // A page inside the window whose header CRC no longer holds (bits lost
    // after it was written): its bytes are not trusted, the frame on it
    // included, though that frame's own CRC16 holds.
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    std::vector<uint8_t> s;
    pad(s, payload * 3, 0x10);
    add(s, frame(5000, 0x05));            // on page 3
    pad(s, payload * 5 - s.size(), 0x10);
    add(s, frame(100, 0x01));             // on page 5, the flight's last
    pad(s, 100, 0x10);
    uint32_t start = 0;
    chip.flyAndCut(s, &start);
    std::vector<uint8_t> clear(GetParam(), 0xFF);
    clear[0] = 0x00;                      // clears bits of page 3's CRC field
    ASSERT_TRUE(chip.nand.programPage(start, 3, clear.data()));

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    const QuickResult q = quick(fl);
    ASSERT_EQ(q.st, Status::Ok);
    ASSERT_TRUE(q.found);
    uint32_t best = 0;
    memcpy(&best, q.best + 6, 4);
    EXPECT_EQ(best, 100u) << "the frame on the failing page is not read";
    EXPECT_EQ(q.ot.gaps, 1u);
}

TEST_P(OrphanTailTest, AnOrphanOlderThanAStoredFlightIsNotFound)
{
    // The cut flight, then a later flight logged and stored before any scan:
    // the orphan is history, not a flight in progress.
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    std::vector<uint8_t> s;
    pad(s, payload * 2, 0x10);
    const uint32_t old_id = chip.flyAndCut(s);
    {
        TR_FlightLog fl;
        ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
        uint32_t id = 0;
        ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
        EXPECT_GT(id, old_id);
        Chip::writeStream(fl, s);
        ASSERT_EQ(fl.finalizeFlight("f_later.bin", (uint32_t)s.size()), Status::Ok);
    }
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    EXPECT_EQ(quick(fl).st, Status::NotFound);
}

TEST_P(OrphanTailTest, TwoCutFlights_TheNewestIsFound)
{
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    std::vector<uint8_t> a, b;
    // the older flight fills one block, its last frame at the block's end and
    // with a higher order than the newest flight's
    pad(a, payload * 62, 0x10);
    add(a, frame(999, 0x01));
    pad(a, payload * 64 - a.size(), 0x10);
    pad(b, payload * 2, 0x10);
    const auto newest = frame(2, 0x02);
    add(b, newest);
    pad(b, 100, 0x10);
    const uint32_t id_a = chip.flyAndCut(a);
    const uint32_t id_b = chip.flyAndCut(b);   // no scan between the two cuts
    ASSERT_GT(id_b, id_a);

    TR_FlightLog swept;   // the quick path's own sweep picks the newest too
    ASSERT_EQ(swept.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    swept.forgetBootOrphanForTest();
    TR_FlightLog::OrphanTail own;
    ASSERT_EQ(swept.newestOrphanTail(0, nullptr, nullptr, own), Status::Ok);
    EXPECT_EQ(own.flight_id, id_b);

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    const QuickResult q = quick(fl);
    ASSERT_EQ(q.st, Status::Ok);
    EXPECT_EQ(q.ot.flight_id, id_b);
    ASSERT_TRUE(q.found);
    EXPECT_EQ(0, memcmp(q.last, newest.data(), kFrameLen));
    EXPECT_EQ(0, memcmp(q.best, newest.data(), kFrameLen)) << "the older flight's blocks are not read";
    // the window costs the newest flight's own pages only: the walk stops at
    // its first block, above the older flight's blank prealloc
    TR_FlightLog::OrphanTail located;
    ASSERT_EQ(fl.newestOrphanTail(0, nullptr, nullptr, located), Status::Ok);
    EXPECT_EQ(q.ot.pages_read - located.pages_read, 3u);
}

TEST_P(OrphanTailTest, TheActiveFlightIsNotAnOrphan)
{
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    std::vector<uint8_t> s;
    pad(s, payload * 2, 0x10);
    const uint32_t cut = chip.flyAndCut(s);

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    uint32_t live = 0;
    ASSERT_EQ(fl.prepareFlight(live), Status::Ok);
    Chip::writeStream(fl, s);
    const QuickResult q = quick(fl);
    ASSERT_EQ(q.st, Status::Ok);
    EXPECT_EQ(q.ot.flight_id, cut) << "the flight being logged is skipped";
}

TEST_P(OrphanTailTest, StoredFlightsBlocksAreNotRead)
{
    // A card holding stored flights: the quick path reads page 0 of the
    // orphaned blocks only, then the last block from the top. Without a sink,
    // nothing else.
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    for (int f = 0; f < 3; ++f) {
        TR_FlightLog fl;
        ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
        uint32_t id = 0;
        ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
        std::vector<uint8_t> s(payload * 64 * 3, 0x30);   // three blocks each
        Chip::writeStream(fl, s);
        char name[32];
        snprintf(name, sizeof(name), "f_%d.bin", f);
        ASSERT_EQ(fl.finalizeFlight(name, (uint32_t)s.size()), Status::Ok);
    }
    const uint32_t pages = 64 + 21;   // the cut flight ends on page 20 of its second block
    std::vector<uint8_t> cut(payload * pages, 0x40);
    chip.flyAndCut(cut);

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    uint32_t orphaned = 0;
    for (uint32_t b = fl.config().flight_region_start; b < fl.config().flight_region_end; ++b) {
        if (fl.bitmap().get(b) != tr_flightlog::BLOCK_ALLOCATED) continue;
        bool stored = false;
        for (size_t i = 0; i < fl.index().size(); ++i) {
            const auto& e = fl.index().at(i);
            if (b >= e.start_block && b < (uint32_t)e.start_block + e.n_blocks) stored = true;
        }
        if (!stored) ++orphaned;
    }
    ASSERT_EQ(fl.index().size(), 3u);
    TR_FlightLog::OrphanTail ot;
    fl.forgetBootOrphanForTest();   // the sweep itself, as after a change since begin()
    ASSERT_EQ(fl.newestOrphanTail(0, nullptr, nullptr, ot), Status::Ok);
    EXPECT_EQ(ot.pages_read, orphaned + (64u - 20u))
        << "page 0 of each orphaned block, then pages 63 down to 20 of the last";

    // begin() made those page-0 reads already: used, they are not repeated
    TR_FlightLog fl2;
    ASSERT_EQ(fl2.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    TR_FlightLog::OrphanTail ot2;
    ASSERT_EQ(fl2.newestOrphanTail(0, nullptr, nullptr, ot2), Status::Ok);
    EXPECT_EQ(ot2.pages_read, 64u - 20u);
    EXPECT_EQ(ot2.flight_id, ot.flight_id);
    EXPECT_EQ(ot2.start_block, ot.start_block);
    EXPECT_EQ(ot2.last_seq, ot.last_seq);
}

TEST_P(OrphanTailTest, BeginsSweepIsDroppedOnceAFlightIsPrepared)
{
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    std::vector<uint8_t> s(payload * 3, 0x10);
    const uint32_t cut = chip.flyAndCut(s);
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    TR_FlightLog::OrphanTail before;
    ASSERT_EQ(fl.newestOrphanTail(0, nullptr, nullptr, before), Status::Ok);
    uint32_t live = 0;
    ASSERT_EQ(fl.prepareFlight(live), Status::Ok);
    TR_FlightLog::OrphanTail after;
    ASSERT_EQ(fl.newestOrphanTail(0, nullptr, nullptr, after), Status::Ok);
    EXPECT_EQ(after.flight_id, cut);
    EXPECT_GT(after.pages_read, before.pages_read) << "it sweeps again: the blocks changed";
}

TEST_P(OrphanTailTest, ABlockOfTheFlightWhosePage0ReadsBadIsCountedAsGaps)
{
    // Block 1 of a three-block flight reads back bad at page 0: its pages
    // cannot be placed, so they are counted, and the caller refuses the tail.
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    std::vector<uint8_t> s(payload * (64 * 2 + 5), 0x10);
    uint32_t start = 0;
    chip.flyAndCut(s, &start);
    chip.nand.injectReadErrorPersistent(start + 1, 0);
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    const QuickResult q = quick(fl);
    ASSERT_EQ(q.st, Status::Ok);
    EXPECT_EQ(q.ot.last_seq, 64u * 2 + 4);
    EXPECT_EQ(q.ot.gaps, 64u);
}

TEST_P(OrphanTailTest, TheScansEntryHoldsTheFirstBlockTheQuickPathNames)
{
    // The #846 indexed path matches the orphan marker by the id and the
    // first block inside the entry's range. The scan's entry can start lower:
    // here a flight armed and cut before its first page left a blank range
    // just below, and the scan's run takes it in.
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    chip.flyAndCut({});                    // armed, cut before any page
    std::vector<uint8_t> s(payload * 70, 0x10);
    uint32_t start = 0;
    const uint32_t id = chip.flyAndCut(s, &start);

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    TR_FlightLog::OrphanTail ot;
    ASSERT_EQ(fl.newestOrphanTail(0, nullptr, nullptr, ot), Status::Ok);
    EXPECT_EQ(ot.flight_id, id);
    EXPECT_EQ(ot.start_block, start);
    ASSERT_EQ(fl.runDeferredRecovery(), Status::Ok);
    bool seen = false;
    for (size_t i = 0; i < fl.index().size(); ++i) {
        const auto& e = fl.index().at(i);
        if (e.flight_id != id) continue;
        seen = true;
        EXPECT_LE(e.start_block, ot.start_block);
        EXPECT_LT(ot.start_block, (uint32_t)e.start_block + e.n_blocks);
    }
    ASSERT_TRUE(seen);
}

TEST_P(OrphanTailTest, TheWindowStartsWhereItIsAsked)
{
    // An out-of-order frame (a higher order) early in the window's oldest
    // block, before the window starts: not read, as the indexed path's 64 KB
    // window does not read it.
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    std::vector<uint8_t> s;
    pad(s, payload * 10, 0x10);
    add(s, frame(9999, 0x09));
    pad(s, payload * 64 - s.size(), 0x10);
    pad(s, payload * 5, 0x10);
    add(s, frame(100, 0x01));
    pad(s, 100, 0x10);
    const uint32_t id = chip.flyAndCut(s);

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    const QuickResult q = quick(fl);
    ASSERT_EQ(q.st, Status::Ok);
    ASSERT_TRUE(q.found);
    uint32_t best = 0;
    memcpy(&best, q.best + 6, 4);
    EXPECT_EQ(best, 100u);
    const ScanResult r = indexedScan(fl, id);
    ASSERT_TRUE(r.found);
    EXPECT_EQ(0, memcmp(q.best, r.best, kFrameLen));
}

TEST_P(OrphanTailTest, NothingCutNothingFound)
{
    Chip chip(GetParam());
    {
        TR_FlightLog fl;
        ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
        const QuickResult fresh = quick(fl);
        EXPECT_EQ(fresh.st, Status::NotFound) << "a fresh chip";
        EXPECT_EQ(fresh.ot.pages_read, 0u) << "begin()'s sweep found none: nothing to read";
        uint32_t id = 0;
        ASSERT_EQ(fl.prepareFlight(id), Status::Ok);
        std::vector<uint8_t> s(1000, 0x10);
        Chip::writeStream(fl, s);
        ASSERT_EQ(fl.finalizeFlight("f_done.bin", (uint32_t)s.size()), Status::Ok);
    }
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    EXPECT_EQ(quick(fl).st, Status::NotFound) << "a finalized flight is not an orphan";
}

TEST_P(OrphanTailTest, WithNoSinkItOnlyLocatesTheFlight)
{
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    std::vector<uint8_t> s;
    pad(s, payload * 70, 0x10);
    uint32_t start = 0;
    const uint32_t id = chip.flyAndCut(s, &start);

    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    TR_FlightLog::OrphanTail ot;
    ASSERT_EQ(fl.newestOrphanTail(0, nullptr, nullptr, ot), Status::Ok);
    EXPECT_EQ(ot.flight_id, id);
    EXPECT_EQ(ot.start_block, start);
    EXPECT_EQ(ot.last_seq, 69u);
    EXPECT_FALSE(ot.end_fed);
}

TEST_P(OrphanTailTest, TheIndexAndBitmapAreLeftForTheScan)
{
    // Read-only: the scan later indexes the flight exactly as it would have.
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    const uint32_t id = chip.flyAndCut(flightStream(payload, 80));
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    const size_t saves = chip.store.saveCount();
    const size_t entries = fl.index().size();
    (void)quick(fl);
    EXPECT_EQ(chip.store.saveCount(), saves);
    EXPECT_EQ(fl.index().size(), entries);
    const ScanResult s = indexedScan(fl, id);
    EXPECT_TRUE(s.found);
}

TEST_P(OrphanTailTest, TheWindowTakesFourBlocksAtMost)
{
    // The frame is in block 0; the flight ends on page 0 of block 4. Four
    // blocks back from there (4, 3, 2, 1) do not reach it, however large the
    // window asked for.
    Chip chip(GetParam());
    const size_t payload = GetParam() - 16;
    std::vector<uint8_t> s;
    add(s, frame(1, 0x01));
    pad(s, payload * 64 * 4, 0x10);
    chip.flyAndCut(s);
    TR_FlightLog fl;
    ASSERT_EQ(fl.begin(chip.nand, deferred(), &chip.store), Status::Ok);
    const QuickResult q = quick(fl, payload * 64 * 8);
    ASSERT_EQ(q.st, Status::Ok);
    EXPECT_FALSE(q.found) << "a frame five blocks back is out of the window";
}

TEST(OrphanTailFrameScan, AScratchTooSmallForAPageSaysSo)
{
    uint8_t last[kFrameLen] = {};
    uint8_t small[kFrameLen + 10];
    OrphanTailFrameScan scan;
    scan.fs.frame_type  = kSnapType;
    scan.fs.payload_len = kPayloadLen;
    scan.fs.last_out    = last;
    scan.scratch        = small;
    scan.scratch_len    = sizeof(small);
    std::vector<uint8_t> page(2032, 0x00);
    const auto f = frame(5, 0x05);
    memcpy(page.data() + 100, f.data(), kFrameLen);
    OrphanTailFrameScan::sink(&scan, page.data(), page.size(), false);
    EXPECT_TRUE(scan.overflow);
    EXPECT_FALSE(scan.fs.found);
}

TEST(FrameTailSearch, TheSharedSearchLeavesUnderAFrameAsCarry)
{
    uint8_t last[kFrameLen] = {};
    FrameTailSearch fs;
    fs.frame_type = kSnapType;
    fs.payload_len = kPayloadLen;
    fs.last_out = last;
    std::vector<uint8_t> buf(100, 0x00);
    const auto f = frame(9, 0x09);
    add(buf, f);
    buf.insert(buf.end(), f.begin(), f.begin() + 50);   // a frame's first 50 bytes
    const size_t pos = fs.search(buf.data(), buf.size());
    EXPECT_TRUE(fs.found);
    EXPECT_EQ(0, memcmp(last, f.data(), kFrameLen));
    EXPECT_LT(buf.size() - pos, kFrameLen);
    EXPECT_GE(buf.size() - pos, 50u);
}

INSTANTIATE_TEST_SUITE_P(Geometries, OrphanTailTest,
                         ::testing::Values(4096u, 2048u),
                         [](const ::testing::TestParamInfo<uint32_t>& i) {
                             return i.param == 4096 ? "legacy4k" : "gd5f2k";
                         });
