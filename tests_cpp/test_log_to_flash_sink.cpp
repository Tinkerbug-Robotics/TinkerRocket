/**
 * test_log_to_flash_sink.cpp — Host coverage for TR_LogToFlash's sink-mode
 * drain and close accounting (#837 item 8, #271, #837 item 9).
 *
 * THE INVARIANT.  A session's reported byte count must equal the bytes the
 * sink ACCEPTED — not the bytes popped off the ring.  Everything downstream
 * trusts it: closeLogSession snapshots current_file_bytes into
 * last_closed_session_bytes_, the OC hands that to
 * TR_FlightLog::finalizeFlight as final_bytes, and readFlightPage then serves
 * the download at exactly that length.  Count a page the NAND never took and
 * the app receives a file of the advertised length whose tail is erased 0xFF,
 * with the storage scorecard green because nand_prog_fail never moved
 * (#837 item 8).  The mirror-image failure is #271: a rejected page that does
 * not rewind and reset the staging index wedges the drain and sheds the rest
 * of the flight as ring overruns.
 *
 * WHAT THIS HARNESS IS.  TR_LogToFlash.cpp is compiled for the host against
 * tests_cpp/host_shim (FreeRTOS, esp_timer, heap_caps, NVS, SPI) and
 * fakes/lfs_host_stub.c.  Only SINK mode is exercised: openLogSession,
 * flushRingToNand and closeLogSession all branch on cfg.write_sink before
 * they touch LittleFS, so the sink path needs no filesystem — which is why
 * lfs is stubbed rather than built.  See fakes/lfs_host_stub.c.
 *
 * The ring is real (RAM backing, no MRAM), the frames are real, the drain
 * arithmetic is real; the NAND is not.  Both shipping page geometries are
 * covered — the sink payload quantum is 4080 B on the V8 bench part and
 * 2032 B on the GD5F parts in V9/V10 and the mini — because the accounting
 * is denominated in that quantum.
 *
 * WHAT IT CANNOT COVER.  The harness is single-threaded, so the cross-core
 * races the same code guards (#74's rb_head clobber, #365's consume-on-
 * observe request flags, #370's push/pop serialization) are out of reach.
 * Those need two cores, and a bench board is still the only place they show.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <ostream>
#include <vector>

#include <TR_LogToFlash.h>
#include <TR_NVS.h>   // host_shim's in-memory Preferences, for _host_shim::nvsReset

#include "fakes/fake_write_sink.h"

namespace {

// ─── Frame generation ──────────────────────────────────────────────────────
// The ring stores whatever bytes it is handed, but the pre-launch drop-oldest
// path WALKS them: [0xAA 0x55 0xAA 0x55][type][len][payload][crc16], with
// frame_size = 8 + len.  A malformed frame there trips the bad-SOF clear
// instead of dropping one frame at a time, so the generator emits real ones.

constexpr uint32_t kFrameOverhead = 8;    // SOF(4) + type + len + crc16(2)
constexpr uint32_t kNominalFrame  = 128;  // 120-byte payload

/// Emits well-formed frames and remembers the exact byte stream, so a test
/// can compare what the sink received against what was logged.
class FrameStream
{
public:
    /// Push exactly `total` bytes as a sequence of frames.  `total` must be
    /// 0 or at least one minimum frame.
    void push(TR_LogToFlash& lf, uint32_t total)
    {
        ASSERT_TRUE(total == 0 || total >= kFrameOverhead) << "total=" << total;
        uint32_t remaining = total;
        while (remaining > 0)
        {
            uint32_t size = std::min(remaining, kNominalFrame);
            // Never leave a remainder too small to be a legal frame.
            if (remaining - size != 0 && remaining - size < kFrameOverhead)
            {
                size = remaining - kFrameOverhead;
            }
            std::vector<uint8_t> frame = build(size);
            ASSERT_TRUE(lf.enqueueFrame(frame.data(), frame.size()))
                << "enqueueFrame rejected a " << size << "-byte frame with "
                << remaining << " to go";
            sent_.insert(sent_.end(), frame.begin(), frame.end());
            remaining -= size;
        }
    }

    const std::vector<uint8_t>& sent() const { return sent_; }

private:
    std::vector<uint8_t> build(uint32_t size)
    {
        const uint8_t payload_len = static_cast<uint8_t>(size - kFrameOverhead);
        std::vector<uint8_t> f(size);
        f[0] = 0xAA; f[1] = 0x55; f[2] = 0xAA; f[3] = 0x55;
        f[4] = 0xA1;                 // type — opaque to the ring
        f[5] = payload_len;
        // Payload bytes vary per frame so an ordering fault in the drain
        // shows up as a mismatched stream rather than as identical filler.
        for (uint32_t i = 6; i + 2 < size; ++i)
        {
            f[i] = static_cast<uint8_t>(seq_ + i);
        }
        f[size - 2] = seq_;
        f[size - 1] = static_cast<uint8_t>(~seq_);
        seq_++;
        return f;
    }

    uint8_t              seq_ = 0;
    std::vector<uint8_t> sent_;
};

// ─── Fixture ───────────────────────────────────────────────────────────────

struct GeomCase
{
    const char* name;
    uint8_t     mid;
    uint8_t     did;
    uint32_t    sink_payload;   // page_size - sizeof(PageHeader)
};

// Without this gtest prints the raw struct bytes (pointer included) on a
// failure, which is both unreadable and different every run.
std::ostream& operator<<(std::ostream& os, const GeomCase& g)
{
    return os << g.name << " (RDID 0x" << std::hex << int(g.mid) << int(g.did)
              << std::dec << ", " << g.sink_payload << " B sink payload)";
}

/// Yields spent before a driven task loop is unwound.  Generous enough that
/// no healthy path runs out, small enough that a wedged drain (#271) reports
/// a ring that never emptied instead of hanging CI.
constexpr int kYieldBudget = 64;

/// One TR_LogToFlash per test.  The component has no destructor — on target
/// it is a global that lives for the boot — so each begin() leaks its ring,
/// its LFS buffers and its two mutexes.  That is inherent to the class, not
/// something the fixture can clean up.
class LogToFlashSink : public ::testing::TestWithParam<GeomCase>
{
protected:
    void SetUp() override
    {
        _host_shim::clearTasks();
        _host_shim::nvsReset();

        spi_.setNandRdid(GetParam().mid, GetParam().did);

        cfg_.nand_cs         = 10;
        cfg_.mram_cs         = -1;      // RAM ring
        cfg_.psram_ring_size = 0;       // internal-RAM branch
        cfg_.ring_buffer_size = 65536;
        cfg_.write_sink      = &FakeWriteSink::trampoline;
        cfg_.write_sink_ctx  = &sink_;

        ASSERT_TRUE(lf_.begin(spi_, cfg_));
        ASSERT_EQ(lf_.sinkPayloadSize(), GetParam().sink_payload);
    }

    void TearDown() override { _host_shim::clearTasks(); }

    TR_LogToFlashStats stats() const
    {
        TR_LogToFlashStats s;
        lf_.getStats(s);
        return s;
    }

    uint32_t chunk() const { return GetParam().sink_payload; }

    /// Run the registered flush task on this thread until its yield budget
    /// is spent.  This is the only way to reach closeLogSession: the
    /// end-of-flight drain and close live in flushTaskLoop, and service()
    /// covers only the open/activate/flush half.
    void pumpFlushTask() { _host_shim::runPendingTasks(kYieldBudget); }

    /// Take the session from begin() to "logging, ring accepting frames"
    /// through the flush task, the way the OC does at launch detect.
    void openAndActivateViaTask()
    {
        lf_.startFlushTask();
        lf_.prepareLogFile();
        lf_.startLogging();
        pumpFlushTask();
        ASSERT_TRUE(lf_.isLoggingActive());
    }

    SPIClass              spi_;
    TR_LogToFlashConfig   cfg_;
    FakeWriteSink         sink_;
    TR_LogToFlash         lf_;
    FrameStream           frames_;
};

// ─── #837 item 8: the final partial page ───────────────────────────────────

TEST_P(LogToFlashSink, FailedTailPageIsExcludedFromTheSessionByteCount)
{
    openAndActivateViaTask();

    constexpr uint32_t kTail = 700;
    const uint32_t full_pages = 2 * chunk();
    frames_.push(lf_, full_pages + kTail);

    // The tail is the only short page a session produces, so rejecting short
    // pages rejects exactly the closeLogSession flush.
    sink_.policy = FakeWriteSink::rejectShortPages(chunk());

    lf_.endLogging();
    pumpFlushTask();

    ASSERT_FALSE(lf_.isLoggingActive());
    ASSERT_EQ(sink_.callCount(), 3u) << "two full pages plus the tail";
    EXPECT_EQ(sink_.calls[2].len, kTail);
    EXPECT_FALSE(sink_.calls[2].accepted);

    // The invariant: reported bytes == bytes the sink took.
    EXPECT_EQ(lf_.lastClosedSessionBytes(), sink_.accepted_bytes);
    EXPECT_EQ(lf_.lastClosedSessionBytes(), full_pages);

    const TR_LogToFlashStats s = stats();
    // Without the increment the storage scorecard stays green through a
    // truncated download — shStorageState(.., write_fail=1) is what turns
    // this into a BAD verdict (pinned in test_rocket_computer_types).
    EXPECT_EQ(s.nand_prog_fail, 1u);
    EXPECT_EQ(s.nand_prog_ops, 2u);
    EXPECT_EQ(s.bytes_written_nand, full_pages);
}

// ─── #837 item 8: the clean tail is still counted ──────────────────────────

TEST_P(LogToFlashSink, CleanSessionAccountsEveryPageIncludingTheTail)
{
    openAndActivateViaTask();

    constexpr uint32_t kTail = 700;
    const uint32_t total = 2 * chunk() + kTail;
    frames_.push(lf_, total);

    lf_.endLogging();
    pumpFlushTask();

    ASSERT_FALSE(lf_.isLoggingActive());
    ASSERT_EQ(sink_.callCount(), 3u);
    EXPECT_EQ(sink_.calls[0].len, chunk());
    EXPECT_EQ(sink_.calls[1].len, chunk());
    EXPECT_EQ(sink_.calls[2].len, kTail);

    EXPECT_EQ(lf_.lastClosedSessionBytes(), sink_.accepted_bytes);
    EXPECT_EQ(lf_.lastClosedSessionBytes(), total);

    const TR_LogToFlashStats s = stats();
    EXPECT_EQ(s.nand_prog_fail, 0u);
    // The tail page is a program op like any other.  These were one page
    // short per session before #837 item 8.
    EXPECT_EQ(s.nand_prog_ops, 3u);
    EXPECT_EQ(s.bytes_written_nand, total);

    // The sink saw the logged bytes, in order, with nothing inserted or lost.
    EXPECT_EQ(sink_.accepted_stream, frames_.sent());
}

// ─── #271: a mid-drain rejection must not wedge the drain ──────────────────

TEST_P(LogToFlashSink, MidDrainFailureRewindsAndKeepsDraining)
{
    // Driven through service() rather than the flush task on purpose:
    // service() calls flushRingToNand exactly once, so "did this drain make
    // progress" is a direct observation.  The end-of-flight path wraps the
    // same call in `while (remaining > 0)`, where a wedge spins until the
    // harness's yield budget unwinds it — bounded, but a muddier signal.
    lf_.startLogging();
    lf_.service();                        // consumes the request: open + activate
    ASSERT_TRUE(lf_.isLoggingActive());

    const uint32_t total = 3 * chunk();
    frames_.push(lf_, total);
    ASSERT_EQ(stats().ring_fill, total);

    sink_.policy = FakeWriteSink::rejectCall(1);   // the second page

    // Bounded: a wedged drain fails the ring_fill assertion below instead of
    // spinning here.
    for (int i = 0; i < 8 && stats().ring_fill > 0; ++i)
    {
        lf_.service();
    }

    EXPECT_EQ(stats().ring_fill, 0u) << "drain stopped making progress";
    ASSERT_EQ(sink_.callCount(), 3u) << "the page after the failure was never offered";
    EXPECT_FALSE(sink_.calls[1].accepted);

    // current_file_bytes rewound by exactly the chunk that never landed.
    EXPECT_EQ(lf_.currentFileBytes(), sink_.accepted_bytes);
    EXPECT_EQ(lf_.currentFileBytes(), 2 * chunk());

    const TR_LogToFlashStats s = stats();
    EXPECT_EQ(s.nand_prog_fail, 1u);
    EXPECT_EQ(s.nand_prog_ops, 2u);
    EXPECT_EQ(s.bytes_written_nand, 2u * chunk());
}

// ─── #837 item 9: the pre-launch ring cap is 75%, not 50% ──────────────────

TEST_P(LogToFlashSink, PreLaunchRingCapsAtThreeQuarters)
{
    lf_.startFlushTask();
    lf_.prepareLogFile();
    pumpFlushTask();                      // file pre-created; NOT activated
    ASSERT_FALSE(lf_.isLoggingActive());

    const uint32_t ring = stats().ring_size;
    ASSERT_EQ(ring, cfg_.ring_buffer_size);

    // Push a full ring's worth so drop-oldest has to engage.
    frames_.push(lf_, ring);

    const TR_LogToFlashStats s = stats();
    EXPECT_GT(s.ring_drop_oldest_bytes, 0u) << "drop-oldest never ran";
    EXPECT_EQ(s.ring_bad_sof_clears, 0u) << "the frame walk lost sync";
    EXPECT_LE(s.ring_fill, (ring / 4) * 3);
    // The load-bearing half: the cap was raised from 50% to 75% so ~1 s of
    // pre-ignition history survives at the 1920 Hz stream.  A fill above half
    // the ring is what distinguishes the two.
    EXPECT_GT(s.ring_fill, ring / 2);
}

// ─── Activation lifts the cap ──────────────────────────────────────────────

TEST_P(LogToFlashSink, ActivationLiftsTheCapAndCloseRestoresIt)
{
    openAndActivateViaTask();

    // Above the 75% pre-launch cap but inside the ring: accepted only
    // because activateLogging raised the cap to the full ring.
    const uint32_t above_cap = (cfg_.ring_buffer_size / 4) * 3 + 4096;
    frames_.push(lf_, above_cap);
    EXPECT_EQ(stats().ring_drop_oldest_bytes, 0u)
        << "in-flight pushes must never drop from the tail";
    EXPECT_EQ(stats().ring_fill, above_cap);

    lf_.endLogging();
    pumpFlushTask();
    ASSERT_FALSE(lf_.isLoggingActive());
    EXPECT_EQ(stats().ring_fill, 0u);
    EXPECT_EQ(lf_.lastClosedSessionBytes(), sink_.accepted_bytes);
    EXPECT_EQ(lf_.lastClosedSessionBytes(), above_cap);

    // ...and the next pre-launch is capped again, so the ring cannot fill to
    // the brim on the pad and leave the launch transient no headroom.
    const uint32_t drops_before = stats().ring_drop_oldest_bytes;
    lf_.prepareLogFile();
    pumpFlushTask();
    ASSERT_FALSE(lf_.isLoggingActive());
    frames_.push(lf_, cfg_.ring_buffer_size);
    EXPECT_GT(stats().ring_drop_oldest_bytes, drops_before);
    EXPECT_LE(stats().ring_fill, (cfg_.ring_buffer_size / 4) * 3);
}

INSTANTIATE_TEST_SUITE_P(
    Geometries, LogToFlashSink,
    ::testing::Values(
        GeomCase{"F35SQB004G", 0xCD, 0x53, 4096 - 16},   // V8 bench, 4 KB pages
        GeomCase{"GD5F2GQ5UE", 0xC8, 0x52, 2048 - 16}),  // V9/V10 and the mini
    [](const ::testing::TestParamInfo<GeomCase>& i) { return i.param.name; });

}  // namespace
