// #846: the tail-scan that re-seeds the OC's snapshot cache from the NAND log
// stream after an OC-also-reset. Runs against the REAL TR_FlightLog over the
// fake NAND at both page geometries (4096 legacy / 2048 GD5F), because the
// scan's carry logic exists precisely for frames straddling page-payload
// boundaries and those boundaries move with chip geometry (#671).

#include <gtest/gtest.h>

#include "SnapshotTailScan.h"
#include "TR_FlightLog.h"
#include "fakes/fake_nand_backend.h"
#include "fakes/memory_bitmap_store.h"
#include <CRC.h>

#include <cstring>
#include <vector>

using tr_flightlog::TR_FlightLog;
using tr_flightlog::Status;
using tr_flightlog::tailScanForFrame;
using tr_flightlog_test::FakeNandBackend;
using tr_flightlog_test::MemoryBitmapStore;

namespace {

constexpr uint8_t kSnapType   = 0xD2;   // SNAPSHOT_MSG
constexpr uint8_t kPayloadLen = 224;    // sizeof(FlightSnapshotData)
constexpr size_t  kFrameLen   = 4 + 1 + 1 + kPayloadLen + 2;

// The standard wire framing (mirrors TR_I2C_Interface::packMessage and the
// wire_fixtures_lib reimplementation): SOF, type, len, payload, CRC16 over
// type..payload.
std::vector<uint8_t> makeFrame(uint8_t type, uint8_t seed) {
    std::vector<uint8_t> f{0xAA, 0x55, 0xAA, 0x55, type, kPayloadLen};
    for (int i = 0; i < kPayloadLen; ++i)
        f.push_back(static_cast<uint8_t>(seed + i * 3));
    const uint16_t crc = calcCRC16(f.data() + 4, 2 + kPayloadLen);
    f.push_back((crc >> 8) & 0xFF);
    f.push_back(crc & 0xFF);
    return f;
}

std::vector<uint8_t> filler(size_t n, uint8_t b) { return std::vector<uint8_t>(n, b); }

struct Rig {
    FakeNandBackend nand;
    MemoryBitmapStore store;
    TR_FlightLog fl;
    uint32_t flight_id = 0;
    uint32_t bytes = 0;

    explicit Rig(uint32_t page, uint32_t blocks)
        : nand(page, 64, blocks) {
        EXPECT_EQ(fl.begin(nand, TR_FlightLog::Config{}, &store), Status::Ok);
        EXPECT_EQ(fl.prepareFlight(flight_id), Status::Ok);
    }
    // Append a stream chunk exactly as the OC's flush sink would: whole
    // frames packed into writeFrame payloads up to payloadPerPage().
    void append(const std::vector<uint8_t>& data) {
        size_t off = 0;
        while (off < data.size()) {
            const size_t n = std::min<size_t>(fl.payloadPerPage(), data.size() - off);
            ASSERT_EQ(fl.writeFrame(data.data() + off, n), Status::Ok);
            off += n;
        }
        bytes += data.size();
    }
    bool scan(uint8_t* out, uint32_t window = 64 * 1024) {
        static uint8_t scratch[(4096 - 16) + kFrameLen];
        char name[32];
        snprintf(name, sizeof(name), "f_%lu.bin", (unsigned long)flight_id);
        EXPECT_EQ(fl.finalizeFlight(name, bytes), Status::Ok);
        return tailScanForFrame(fl, name, bytes, kSnapType, kPayloadLen,
                                window, out, scratch, sizeof(scratch));
    }
};

class TailScan : public ::testing::TestWithParam<uint32_t> {};

}  // namespace

TEST_P(TailScan, LastValidFrameWins) {
    Rig r(GetParam(), 256);
    std::vector<uint8_t> stream;
    auto add = [&](const std::vector<uint8_t>& v) { stream.insert(stream.end(), v.begin(), v.end()); };
    add(filler(500, 0x11));
    add(makeFrame(kSnapType, 0x10));       // older snapshot
    add(filler(300, 0x22));
    auto last = makeFrame(kSnapType, 0x77);
    add(last);                             // the one that must win
    add(filler(90, 0x33));                 // trailing non-frame bytes
    r.append(stream);

    uint8_t out[kFrameLen] = {};
    ASSERT_TRUE(r.scan(out));
    EXPECT_EQ(0, memcmp(out, last.data(), kFrameLen));
}

TEST_P(TailScan, FrameStraddlingPageBoundaryParses) {
    Rig r(GetParam(), 256);
    const size_t ppp = r.fl.payloadPerPage();
    std::vector<uint8_t> stream;
    // Place the frame so it starts kFrameLen/2 before a page-payload boundary.
    stream.assign(ppp - kFrameLen / 2, 0x44);
    auto f = makeFrame(kSnapType, 0x55);
    stream.insert(stream.end(), f.begin(), f.end());
    stream.insert(stream.end(), 64, 0x66);
    r.append(stream);

    uint8_t out[kFrameLen] = {};
    ASSERT_TRUE(r.scan(out));
    EXPECT_EQ(0, memcmp(out, f.data(), kFrameLen));
}

TEST_P(TailScan, TornTailFallsBackToPreviousFrame) {
    Rig r(GetParam(), 256);
    std::vector<uint8_t> stream;
    auto good = makeFrame(kSnapType, 0x21);
    stream.insert(stream.end(), good.begin(), good.end());
    auto torn = makeFrame(kSnapType, 0x99);
    torn.resize(torn.size() - 5);          // truncated: brownout mid-write
    stream.insert(stream.end(), torn.begin(), torn.end());
    r.append(stream);

    uint8_t out[kFrameLen] = {};
    ASSERT_TRUE(r.scan(out));
    EXPECT_EQ(0, memcmp(out, good.data(), kFrameLen));
}

TEST_P(TailScan, CorruptCrcRejected) {
    Rig r(GetParam(), 256);
    auto bad = makeFrame(kSnapType, 0x42);
    bad[10] ^= 0xFF;                        // payload bit-flip: CRC16 must catch
    std::vector<uint8_t> stream(bad.begin(), bad.end());
    r.append(stream);

    uint8_t out[kFrameLen] = {};
    EXPECT_FALSE(r.scan(out));
}

TEST_P(TailScan, WindowExcludesOldFrames) {
    Rig r(GetParam(), 256);
    std::vector<uint8_t> stream;
    auto old_frame = makeFrame(kSnapType, 0x31);
    stream.insert(stream.end(), old_frame.begin(), old_frame.end());
    auto pad = filler(8 * 1024, 0x00);
    stream.insert(stream.end(), pad.begin(), pad.end());
    r.append(stream);

    uint8_t out[kFrameLen] = {};
    // A 4 KB window over an 8 KB tail of filler: the old frame is out of reach.
    EXPECT_FALSE(r.scan(out, 4 * 1024));
}

TEST_P(TailScan, ReadErrorFailsRatherThanReturningAStaleFrame) {
    // A read error mid-window means the stream's real last word was never
    // reached — a LANDED clear could be sitting past the unreadable page. The
    // scan must fail, not hand back the older frame it happened to see.
    Rig r(GetParam(), 256);
    std::vector<uint8_t> stream;
    auto f = makeFrame(kSnapType, 0x61);
    stream.insert(stream.end(), f.begin(), f.end());
    // Pad past the first page so the scan needs a second read to finish.
    auto pad = filler(r.fl.payloadPerPage() * 2, 0x00);
    stream.insert(stream.end(), pad.begin(), pad.end());
    r.append(stream);

    char name[32];
    snprintf(name, sizeof(name), "f_%lu.bin", (unsigned long)r.flight_id);
    ASSERT_EQ(r.fl.finalizeFlight(name, r.bytes), Status::Ok);
    // Break the LAST page of the flight: the frame is readable, the tail is not.
    const uint32_t start = r.fl.activeStartBlock();
    (void)start;
    r.nand.injectReadErrorPersistent(r.fl.index().at(0).start_block, 2);

    static uint8_t scratch[(4096 - 16) + kFrameLen];
    uint8_t out[kFrameLen] = {};
    EXPECT_FALSE(tailScanForFrame(r.fl, name, r.bytes, kSnapType, kPayloadLen,
                                  64 * 1024, out, scratch, sizeof(scratch)));
}

TEST_P(TailScan, BestByOrderFieldBeatsStreamOrder) {
    // An I2S DMA replay can append an OLDER snapshot after a newer one. The
    // last frame is still the stream's final word (used to detect a LANDED
    // clear), but `best_out` must pick the highest ordering field so recovery
    // restores from the newest state, not the replayed one.
    Rig r(GetParam(), 256);
    // Ordering field at payload offset 0 for the test's synthetic payload.
    auto withOrder = [](uint32_t v, uint8_t seed) {
        std::vector<uint8_t> f{0xAA, 0x55, 0xAA, 0x55, kSnapType, kPayloadLen};
        std::vector<uint8_t> pl(kPayloadLen);
        memcpy(pl.data(), &v, sizeof(v));
        for (int i = 4; i < kPayloadLen; ++i) pl[i] = static_cast<uint8_t>(seed + i);
        f.insert(f.end(), pl.begin(), pl.end());
        const uint16_t crc = calcCRC16(f.data() + 4, 2 + kPayloadLen);
        f.push_back((crc >> 8) & 0xFF);
        f.push_back(crc & 0xFF);
        return f;
    };
    std::vector<uint8_t> stream;
    auto newest = withOrder(41300, 0x01);
    auto replay = withOrder(40900, 0x02);   // older, but LAST in the stream
    stream.insert(stream.end(), newest.begin(), newest.end());
    stream.insert(stream.end(), replay.begin(), replay.end());
    r.append(stream);

    char name[32];
    snprintf(name, sizeof(name), "f_%lu.bin", (unsigned long)r.flight_id);
    ASSERT_EQ(r.fl.finalizeFlight(name, r.bytes), Status::Ok);
    static uint8_t scratch[(4096 - 16) + kFrameLen];
    uint8_t last_f[kFrameLen] = {}, best_f[kFrameLen] = {};
    ASSERT_TRUE(tailScanForFrame(r.fl, name, r.bytes, kSnapType, kPayloadLen,
                                 64 * 1024, last_f, scratch, sizeof(scratch),
                                 best_f, 0));
    EXPECT_EQ(0, memcmp(last_f, replay.data(), kFrameLen));   // stream order
    EXPECT_EQ(0, memcmp(best_f, newest.data(), kFrameLen));   // payload order
}

TEST_P(TailScan, EmptyFlightFindsNothing) {
    Rig r(GetParam(), 256);
    r.append(filler(1000, 0x5A));
    uint8_t out[kFrameLen] = {};
    EXPECT_FALSE(r.scan(out));
}

INSTANTIATE_TEST_SUITE_P(Geometries, TailScan,
                         ::testing::Values(4096u, 2048u),
                         [](const ::testing::TestParamInfo<uint32_t>& i) {
                             return i.param == 4096 ? "legacy4k" : "gd5f2k";
                         });
