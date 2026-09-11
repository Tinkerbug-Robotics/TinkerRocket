/**
 * test_log_to_flash_begin_retry.cpp — TR_LogToFlash::begin() re-entered
 * after a failure (#1228).
 *
 * THE CASE.  #1132 stopped a dead flight logger from taking the OC's radio
 * and FC link down with it, and in doing so set the flag that had gated a
 * second initPeripherals() call — so a NAND that failed to mount once (a
 * marginal rail at power-on) was never tried again.  #1228 retries the
 * logging half on its own, which makes begin() the first function in this
 * component to run TWICE in one boot, with the I2S parser task live.
 *
 * WHAT HAS TO HOLD.  A failed begin() must leave the object inert (no frame
 * accepted, no session opened on a launch edge, no flush task started) and
 * RETRYABLE: the second begin() reuses the two mutexes and the ring slot
 * instead of allocating a fresh set, re-probes the chip, and comes up as a
 * working logger.  A begin() on a logger that is already up is a no-op.
 *
 * THE HARNESS.  Same host build as test_log_to_flash_sink (host_shim +
 * fakes/lfs_host_stub.c).  The stub's lfs_host_stub_fail_mount knob drives
 * begin() down its real "mount after format failed" exit — the one a NAND
 * that does not answer takes on target, with the ring and both mutexes
 * already allocated — and the shim's liveMutexes / liveHeapCapsBlocks
 * counters make a leaked set show up as a count that moved.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include <TR_LogToFlash.h>
#include <TR_NVS.h>          // host_shim: _host_shim::nvsReset
#include <esp_heap_caps.h>   // host_shim: _host_shim::liveHeapCapsBlocks
#include <freertos/semphr.h> // host_shim: _host_shim::liveMutexes
#include <freertos/task.h>   // host_shim: _host_shim::pendingTasks / runPendingTasks

#include "fakes/fake_write_sink.h"

extern "C" int lfs_host_stub_fail_mount;

namespace {

constexpr uint8_t kV8Mid = 0xCD, kV8Did = 0x53;   // F35SQB004G: 4 KB pages, 4080 B sink payload
constexpr uint8_t kGdMid = 0xC8, kGdDid = 0x52;   // GD5F2GQ5UE: 2 KB pages, 2032 B sink payload
constexpr uint32_t kV8SinkPayload = 4096 - 16;
constexpr uint32_t kGdSinkPayload = 2048 - 16;

/// Yields spent before a driven flush task is unwound (see the sink test).
constexpr int kYieldBudget = 64;

/// A well-formed frame: [0xAA 0x55 0xAA 0x55][type][len][payload][crc16].
/// The pre-launch drop-oldest path walks frames, so the bytes have to parse.
std::vector<uint8_t> makeFrame(uint32_t size, uint8_t seq)
{
    constexpr uint32_t kOverhead = 8;
    std::vector<uint8_t> f(size);
    f[0] = 0xAA; f[1] = 0x55; f[2] = 0xAA; f[3] = 0x55;
    f[4] = 0xA1;
    f[5] = static_cast<uint8_t>(size - kOverhead);
    for (uint32_t i = 6; i + 2 < size; ++i) f[i] = static_cast<uint8_t>(seq + i);
    f[size - 2] = seq;
    f[size - 1] = static_cast<uint8_t>(~seq);
    return f;
}

class LogToFlashBeginRetry : public ::testing::Test
{
protected:
    void SetUp() override
    {
        _host_shim::clearTasks();
        _host_shim::nvsReset();
        lfs_host_stub_fail_mount = 0;
        mutexes_before_ = _host_shim::liveMutexes();
        blocks_before_  = _host_shim::liveHeapCapsBlocks();

        spi_.setNandRdid(kV8Mid, kV8Did);

        cfg_.nand_cs          = 10;
        cfg_.mram_cs          = -1;      // RAM ring
        cfg_.psram_ring_size  = 0;       // internal-RAM branch
        cfg_.ring_buffer_size = 65536;
        cfg_.write_sink       = &FakeWriteSink::trampoline;
        cfg_.write_sink_ctx   = &sink_;
    }

    void TearDown() override
    {
        lfs_host_stub_fail_mount = 0;
        _host_shim::clearTasks();
    }

    /// Mutexes / capability-heap blocks this test's logger holds right now.
    int mutexesHeld() const { return _host_shim::liveMutexes() - mutexes_before_; }
    int ringsHeld()   const { return _host_shim::liveHeapCapsBlocks() - blocks_before_; }

    /// begin() down the "NAND would not mount" exit.
    void beginAndFail()
    {
        lfs_host_stub_fail_mount = 1;
        ASSERT_FALSE(lf_.begin(spi_, cfg_));
        lfs_host_stub_fail_mount = 0;
        ASSERT_FALSE(lf_.hasBegun());
    }

    void pumpFlushTask() { _host_shim::runPendingTasks(kYieldBudget); }

    /// From a live logger to "logging, ring accepting frames", the way the
    /// OC does it at launch detect.
    void openAndActivateViaTask()
    {
        lf_.startFlushTask();
        lf_.prepareLogFile();
        lf_.startLogging();
        pumpFlushTask();
        ASSERT_TRUE(lf_.isLoggingActive());
    }

    /// Push `total` bytes as frames and return the exact stream.
    std::vector<uint8_t> pushBytes(uint32_t total)
    {
        std::vector<uint8_t> sent;
        uint8_t seq = 0;
        while (total > 0)
        {
            uint32_t size = total < 128 ? total : 128;
            if (total - size != 0 && total - size < 8) size = total - 8;
            std::vector<uint8_t> f = makeFrame(size, seq++);
            EXPECT_TRUE(lf_.enqueueFrame(f.data(), f.size()));
            sent.insert(sent.end(), f.begin(), f.end());
            total -= size;
        }
        return sent;
    }

    SPIClass            spi_;
    TR_LogToFlashConfig cfg_;
    FakeWriteSink       sink_;
    TR_LogToFlash       lf_;
    int                 mutexes_before_ = 0;
    int                 blocks_before_  = 0;
};

// ─── A failed begin() leaves an inert object, not a half-alive one ─────────

TEST_F(LogToFlashBeginRetry, FailedBeginLeavesTheLoggerInert)
{
    beginAndFail();

    // The allocations a retry has to cope with are exactly these: both
    // mutexes and the ring exist; the LFS buffers were freed on the way out.
    EXPECT_EQ(mutexesHeld(), 2);
    EXPECT_EQ(ringsHeld(), 1);

    // The I2S parser keeps calling this on every frame of a flight (#1132
    // brings the link up around a dead logger). It must be refused, not
    // pushed into a ring nothing drains.
    const std::vector<uint8_t> f = makeFrame(128, 0);
    EXPECT_FALSE(lf_.enqueueFrame(f.data(), f.size()));

    // A launch edge seen while dead: the OC calls prepare / start and then
    // services the logger every loop pass. None of it may open a session.
    lf_.prepareLogFile();
    lf_.startLogging();
    for (int i = 0; i < 5; ++i) lf_.service();
    EXPECT_FALSE(lf_.isLoggingActive());
    EXPECT_EQ(sink_.callCount(), 0u);

    // And no flush task is created to drain a store that never mounted.
    lf_.startFlushTask();
    EXPECT_TRUE(_host_shim::pendingTasks().empty());

    TR_LogToFlashStats s;
    lf_.getStats(s);
    EXPECT_EQ(s.frames_received, 0u);
}

// ─── The retry reuses what the failure allocated ───────────────────────────

TEST_F(LogToFlashBeginRetry, RetryReusesTheMutexesAndTheRingSlot)
{
    beginAndFail();
    ASSERT_EQ(mutexesHeld(), 2);
    ASSERT_EQ(ringsHeld(), 1);

    ASSERT_TRUE(lf_.begin(spi_, cfg_));
    EXPECT_TRUE(lf_.hasBegun());

    // Not 4 and 2: the retry did not allocate a second set. On target a
    // leaked 64 KB internal-RAM ring per attempt would exhaust the heap the
    // flush task and the BLE stack live in.
    EXPECT_EQ(mutexesHeld(), 2);
    EXPECT_EQ(ringsHeld(), 1);
    EXPECT_EQ(lf_.sinkPayloadSize(), kV8SinkPayload);

    // And it is a working logger: a whole session through the sink, with the
    // same accounting the sink test pins.
    openAndActivateViaTask();
    ASSERT_EQ(_host_shim::pendingTasks().size(), 1u);
    const uint32_t total = 2 * kV8SinkPayload + 700;
    const std::vector<uint8_t> sent = pushBytes(total);
    lf_.endLogging();
    pumpFlushTask();
    ASSERT_FALSE(lf_.isLoggingActive());
    EXPECT_EQ(sink_.accepted_bytes, total);
    EXPECT_EQ(lf_.lastClosedSessionBytes(), total);
    ASSERT_EQ(sink_.callCount(), 3u);
    EXPECT_EQ(sink_.calls[2].len, 700u);
}

// ─── Requests raised while dead are stale by the time the retry succeeds ───

TEST_F(LogToFlashBeginRetry, StaleRequestsFromTheDeadPeriodDoNotOpenASession)
{
    beginAndFail();

    // A PRELAUNCH edge and a launch edge went by while the logger was dead.
    lf_.prepareLogFile();
    lf_.startLogging();
    lf_.endLogging();       // and a LANDED edge — must not latch end_flight_requested

    ASSERT_TRUE(lf_.begin(spi_, cfg_));
    lf_.startFlushTask();
    pumpFlushTask();

    // The recovered logger starts clean: nothing pre-created, nothing
    // logging. The OC's launch edge re-issues the whole lifecycle.
    EXPECT_FALSE(lf_.isLoggingActive());
    EXPECT_EQ(sink_.callCount(), 0u);

    // ...and it is not wedged: a fresh start request opens a session.
    lf_.startLogging();
    pumpFlushTask();
    EXPECT_TRUE(lf_.isLoggingActive());
}

// ─── The retry re-probes the chip ──────────────────────────────────────────

TEST_F(LogToFlashBeginRetry, RetryReresolvesTheChipGeometry)
{
    // First attempt read one part and failed to mount; the retry answers as
    // another (the marginal-rail RDID case). The geometry everything is
    // denominated in must come from the attempt that succeeded.
    spi_.setNandRdid(kV8Mid, kV8Did);
    beginAndFail();
    EXPECT_EQ(lf_.sinkPayloadSize(), kV8SinkPayload);

    spi_.setNandRdid(kGdMid, kGdDid);
    ASSERT_TRUE(lf_.begin(spi_, cfg_));
    EXPECT_EQ(lf_.sinkPayloadSize(), kGdSinkPayload);
}

// ─── begin() on a live logger is a no-op ───────────────────────────────────

TEST_F(LogToFlashBeginRetry, SecondBeginOnALiveLoggerIsIgnored)
{
    ASSERT_TRUE(lf_.begin(spi_, cfg_));
    ASSERT_EQ(mutexesHeld(), 2);
    ASSERT_EQ(ringsHeld(), 1);
    openAndActivateViaTask();

    // Reported as success, changes nothing: the session stays open and no
    // allocation moves. A re-initialisation here would orphan the flush
    // task's ring under it.
    EXPECT_TRUE(lf_.begin(spi_, cfg_));
    EXPECT_TRUE(lf_.hasBegun());
    EXPECT_TRUE(lf_.isLoggingActive());
    EXPECT_EQ(mutexesHeld(), 2);
    EXPECT_EQ(ringsHeld(), 1);
}

// ─── Three failures in a row still hold one set of allocations ─────────────

TEST_F(LogToFlashBeginRetry, RepeatedFailuresDoNotAccumulateAllocations)
{
    // LoggerRetryPolicy allows three retries after the initial failure: four
    // failed attempts is the worst case a boot can produce.
    for (int attempt = 0; attempt < 4; ++attempt)
    {
        beginAndFail();
        EXPECT_EQ(mutexesHeld(), 2) << "after failed attempt " << attempt + 1;
        EXPECT_EQ(ringsHeld(), 1)   << "after failed attempt " << attempt + 1;
    }
    ASSERT_TRUE(lf_.begin(spi_, cfg_));
    EXPECT_EQ(mutexesHeld(), 2);
    EXPECT_EQ(ringsHeld(), 1);
}

}  // namespace
