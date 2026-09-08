// Host tests for FcOtaSessionPolicy (#1116).
//
// The FC's I2S link is flipped to slave RX for an OTA image and the only thing
// that flipped it back was an OTA_FINISH_CMD / OTA_ABORT_CMD delivered by the
// OC over I2C — a command that rides a bounded queue, is served three times
// and is never re-checked. When it did not arrive the FC sat in the flipped
// state (no telemetry, EKF paused, loop throttled) until a battery pull. The
// imperative half of the fix (the quiet wait, the I2S revert, the receiver
// abort) is not testable off-target; what is, and what these lock down, is
// the decision of WHEN the FC gives up on its own, and that it always gives
// the OC's own stall watchdog the first chance.

#include <gtest/gtest.h>
#include "fc_ota_session_policy.h"
#include "ota_relay_policy.h"   // the OC's stall watchdog, for the ordering checks

using namespace FcOtaSessionPolicy;

namespace {
constexpr uint32_t kFlip = 100000;   // an arbitrary flip time
}

// ---------------------------------------------------------------------------
// Not in data mode: nothing to decide
// ---------------------------------------------------------------------------
TEST(FcOtaSessionPolicy, NothingToDoOutsideDataMode)
{
    // Normal telemetry operation. However stale the stamps, no verdict.
    EXPECT_EQ(evaluate(false, kFlip + 10 * kNoProgressHardCapMs, kFlip, kFlip),
              Verdict::Continue);
}

// ---------------------------------------------------------------------------
// The first window after the flip
// ---------------------------------------------------------------------------
TEST(FcOtaSessionPolicy, AFreshFlipIsNotAStall)
{
    // Both stamps are set at the flip. The first accepted byte is ~1 s away
    // (the OC waits for our silence, flips, warms up, releases "ready"), and
    // the link is silent for the first ~0.5 s of that — neither may count.
    EXPECT_EQ(evaluate(true, kFlip,        kFlip, kFlip), Verdict::Continue);
    EXPECT_EQ(evaluate(true, kFlip + 500,  kFlip, kFlip), Verdict::Continue);
    EXPECT_EQ(evaluate(true, kFlip + 2000, kFlip, kFlip), Verdict::Continue);
    EXPECT_EQ(evaluate(true, kFlip + kNoProgressTimeoutMs - 1, kFlip, kFlip),
              Verdict::Continue);
}

TEST(FcOtaSessionPolicy, AnOcThatNeverFlipsIsGivenUpOnAtTheTimeout)
{
    // The OC never took the clock (it never saw our READY, or its own flip
    // failed): no RX callback ever, no progress ever. The link has been quiet
    // since the flip, so the timeout alone decides.
    EXPECT_EQ(evaluate(true, kFlip + kNoProgressTimeoutMs, kFlip, kFlip),
              Verdict::AbandonLinkQuiet);
}

// ---------------------------------------------------------------------------
// A healthy transfer
// ---------------------------------------------------------------------------
TEST(FcOtaSessionPolicy, ASteadilyLandingImageNeverStalls)
{
    // Chunks accepted every 200 ms for two minutes — longer than the hard cap,
    // which must never be reached while progress is being made. The loop bound
    // is a constant computed up front (see the OTA relay test for why).
    const uint32_t end = kFlip + 120000;
    uint32_t last_progress = kFlip;
    uint32_t iterations = 0;
    for (uint32_t t = kFlip; t < end; t += 200)
    {
        ASSERT_EQ(evaluate(true, t, last_progress, t), Verdict::Continue) << "t=" << t;
        last_progress = t;   // chunk accepted; the link is clocked throughout
        ++iterations;
    }
    EXPECT_EQ(iterations, 600u);
}

TEST(FcOtaSessionPolicy, ProgressResetsTheClockEvenAfterALongGap)
{
    // 29.9 s with nothing landing, then a chunk: the window starts over.
    const uint32_t gap_end = kFlip + kNoProgressTimeoutMs - 100;
    ASSERT_EQ(evaluate(true, gap_end, kFlip, gap_end), Verdict::Continue);
    const uint32_t chunk = gap_end;
    EXPECT_EQ(evaluate(true, chunk + kNoProgressTimeoutMs - 1, chunk, chunk + kNoProgressTimeoutMs - 1),
              Verdict::Continue);
}

// ---------------------------------------------------------------------------
// The stall cases
// ---------------------------------------------------------------------------
TEST(FcOtaSessionPolicy, StalledAndQuietIsAbandonedAtTheTimeout)
{
    // The OC pumped, then everything stopped at once — an OC reboot, or the
    // OC's own watchdog reverting it to slave RX with its abort dropped.
    const uint32_t stop = kFlip + 5000;
    EXPECT_EQ(evaluate(true, stop + kNoProgressTimeoutMs - 1, stop, stop), Verdict::Continue);
    EXPECT_EQ(evaluate(true, stop + kNoProgressTimeoutMs,     stop, stop), Verdict::AbandonLinkQuiet);
    EXPECT_EQ(evaluate(true, stop + 10 * kNoProgressTimeoutMs, stop, stop), Verdict::AbandonLinkQuiet);
}

TEST(FcOtaSessionPolicy, StalledButClockedWaitsForTheOcToLetGo)
{
    // Nothing lands but the OC is still master (idle fill keeps the RX
    // callback ticking). Seizing BCLK now would put two push-pull drivers on
    // one wire, so the FC holds — the OC's own FINISH/ABORT or stall watchdog
    // ends this — until the hard cap.
    const uint32_t stop = kFlip + 5000;
    for (uint32_t t = stop; t < stop + kNoProgressHardCapMs; t += 1000)
    {
        ASSERT_EQ(evaluate(true, t, stop, /*last_rx=*/t), Verdict::Continue) << "t=" << t;
    }
    EXPECT_EQ(evaluate(true, stop + kNoProgressHardCapMs, stop, stop + kNoProgressHardCapMs),
              Verdict::AbandonLinkClocked);
}

TEST(FcOtaSessionPolicy, TheOcReleasingTheClockEndsTheWait)
{
    // Stalled at `stop`; the OC keeps clocking for another 40 s, then reverts
    // to slave RX. One second of silence after that is the go-ahead.
    const uint32_t stop     = kFlip + 5000;
    const uint32_t released = stop + 40000;
    ASSERT_EQ(evaluate(true, released, stop, released), Verdict::Continue);
    EXPECT_EQ(evaluate(true, released + kLinkQuietMs - 1, stop, released), Verdict::Continue);
    EXPECT_EQ(evaluate(true, released + kLinkQuietMs,     stop, released), Verdict::AbandonLinkQuiet);
}

TEST(FcOtaSessionPolicy, QuietIsMeasuredFromTheLastCallbackNotTheLastChunk)
{
    // A link that went quiet only just now is not "quiet" even if progress
    // stopped long ago: the 1 s is counted from the last RX callback.
    const uint32_t stop = kFlip + 5000;
    const uint32_t now  = stop + 2 * kNoProgressTimeoutMs;
    EXPECT_EQ(evaluate(true, now, stop, now - kLinkQuietMs + 1), Verdict::Continue);
    EXPECT_EQ(evaluate(true, now, stop, now - kLinkQuietMs),     Verdict::AbandonLinkQuiet);
}

// ---------------------------------------------------------------------------
// The scenario from the issue, on one timeline with the OC's own watchdog
// ---------------------------------------------------------------------------
TEST(FcOtaSessionPolicy, TheOcWatchdogAlwaysGetsThereFirst)
{
    // The phone walks away at t1. The OC's stall watchdog (fed by the chunks
    // it forwards) fires at t1 + kRelayStallTimeoutMs and reverts the OC to
    // slave RX; the link goes quiet. Its OTA_ABORT_CMD is dropped by a full
    // queue. The FC — whose last accepted byte was that same last chunk —
    // holds until its own timeout and only then reverts, with the link long
    // quiet. At no point does the FC act before the OC has.
    const uint32_t t1 = kFlip + 8000;             // last chunk
    const uint32_t oc_gives_up = t1 + OtaRelayPolicy::kRelayStallTimeoutMs;
    const uint32_t link_quiet_from = oc_gives_up + 100;   // ocRevertToRx settles 100 ms

    for (uint32_t t = t1; t < oc_gives_up; t += 250)
    {
        ASSERT_FALSE(OtaRelayPolicy::relayStalled(true, t1, t)) << "t=" << t;
        ASSERT_EQ(evaluate(true, t, t1, /*last_rx=*/t), Verdict::Continue) << "t=" << t;
    }
    ASSERT_TRUE(OtaRelayPolicy::relayStalled(true, t1, oc_gives_up));

    uint32_t fc_reverted_at = 0;
    for (uint32_t t = oc_gives_up; t < t1 + 2 * kNoProgressTimeoutMs; t += 250)
    {
        const uint32_t last_rx = (t < link_quiet_from) ? t : link_quiet_from;
        if (evaluate(true, t, t1, last_rx) == Verdict::AbandonLinkQuiet)
        {
            fc_reverted_at = t;
            break;
        }
    }
    ASSERT_NE(fc_reverted_at, 0u) << "the FC never recovered";
    EXPECT_EQ(fc_reverted_at, t1 + kNoProgressTimeoutMs);
    EXPECT_GT(fc_reverted_at, oc_gives_up);
}

TEST(FcOtaSessionPolicy, TheTimeoutIsStrictlyTheBackstop)
{
    // The OC's abort is the normal ending; the FC's own exit exists for when
    // that abort never arrives. It must come well after the OC has given up,
    // including the ~1 s between our flip and the OC arming its watchdog.
    EXPECT_GE(kNoProgressTimeoutMs, 2 * OtaRelayPolicy::kRelayStallTimeoutMs);
    EXPECT_GT(kNoProgressHardCapMs, kNoProgressTimeoutMs);
    EXPECT_LT(kLinkQuietMs, kNoProgressTimeoutMs);
}

// ---------------------------------------------------------------------------
// Clock wraparound
// ---------------------------------------------------------------------------
TEST(FcOtaSessionPolicy, SurvivesMillisWraparound)
{
    // esp_timer ms wraps every ~49.7 days; unsigned subtraction must carry it.
    const uint32_t stop = 0xFFFFF000u;
    EXPECT_EQ(evaluate(true, (uint32_t)(stop + 100), stop, stop), Verdict::Continue);
    EXPECT_EQ(evaluate(true, (uint32_t)(stop + kNoProgressTimeoutMs), stop, stop),
              Verdict::AbandonLinkQuiet);
    const uint32_t now = (uint32_t)(stop + kNoProgressTimeoutMs);
    EXPECT_EQ(evaluate(true, now, stop, (uint32_t)(now - 10)), Verdict::Continue);
}

// ---------------------------------------------------------------------------
// #1123 — a freshly OTA'd image may only cancel its own rollback once the OC
// link is PROVEN, not merely once it has ticked for 10 s.
//
// The flight computer has no update path of its own: every image arrives
// through the out computer (BLE -> I2C OTA_BEGIN_PENDING -> I2S image pump). So
// an image whose I2S TX config, I2C master poll or frame CRC is broken used to
// tick the loop happily for 10 s, mark itself valid, and permanently remove
// both the telemetry link AND the only way to replace itself — recoverable only
// by opening the airframe and flashing over USB.
//
// docs/plans/08-ota-firmware-update.md:331 is the contract this restores.
// ---------------------------------------------------------------------------

TEST(FcOtaMarkValid, TimeAloneIsNotEnough) {
    // The exact defect: a long, stable, completely mute run.
    EXPECT_FALSE(FcOtaSessionPolicy::mayCancelRollback(60000, false, 0));
}

TEST(FcOtaMarkValid, BothLinksAndTheWindowAreRequired) {
    const uint32_t ok = FcOtaSessionPolicy::kStableRunMs;
    EXPECT_TRUE (FcOtaSessionPolicy::mayCancelRollback(ok,     true,  1));
    EXPECT_FALSE(FcOtaSessionPolicy::mayCancelRollback(ok - 1, true,  1)) << "window";
    EXPECT_FALSE(FcOtaSessionPolicy::mayCancelRollback(ok,     false, 1)) << "no I2C round trip";
    EXPECT_FALSE(FcOtaSessionPolicy::mayCancelRollback(ok,     true,  0)) << "no I2S frame accepted";
}

TEST(FcOtaMarkValid, EitherLinkAloneIsInsufficient) {
    // Either one alone still leaves the image unreplaceable: I2S carries
    // telemetry to the OC, I2C carries the commands that start the next update.
    EXPECT_FALSE(FcOtaSessionPolicy::mayCancelRollback(30000, true,  0));
    EXPECT_FALSE(FcOtaSessionPolicy::mayCancelRollback(30000, false, 50));
}

TEST(FcOtaMarkValid, IsNotYetRatherThanNever) {
    // The caller re-evaluates every tick, so a slow OC bring-up only delays
    // validation — the same inputs later must succeed.
    EXPECT_FALSE(FcOtaSessionPolicy::mayCancelRollback(12000, false, 0));
    EXPECT_TRUE (FcOtaSessionPolicy::mayCancelRollback(45000, true,  3));
}

TEST(FcOtaMarkValid, AHealthyImageValidatesAtTheWindow) {
    // The normal case must not regress: a working image has both links well
    // before 10 s, so it validates the moment the window closes.
    EXPECT_TRUE(FcOtaSessionPolicy::mayCancelRollback(FcOtaSessionPolicy::kStableRunMs,
                                                      true, 400));
}
