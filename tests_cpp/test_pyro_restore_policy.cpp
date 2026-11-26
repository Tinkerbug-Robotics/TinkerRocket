// #1760: a pyro channel that had started to fire when the FC reset is spent.
//
// The load-bearing rows:
//   - the snapshot records a channel in its settle or its pulse as started, and
//     the restore keeps every started or fired channel off;
//   - an older snapshot's values (0, 1) restore as before, and an unknown
//     value from a newer writer restores as spent, never as a repeat;
//   - with the extra snapshot on the pass a channel leaves Idle, and the pulse
//     held until that frame is at the OC, the OC holds "started" before the
//     FET turns on, whatever the queue's wait and the 10 Hz cadence's phase,
//     so a reset any time in the pulse finds the channel spent. Without the
//     hold, the DMA ring's ~23 ms still repeats a fire cut short early in its
//     pulse; without either (the old encoding and cadence) the harness
//     reproduces #1760's repeated fire.

#include <gtest/gtest.h>

#include <deque>
#include <utility>

#include "pyro_restore_policy.h"

uint32_t fcPyroArmSettleMs();      // pyro_timing_fc.cpp: the FC's own config.h
uint32_t fcPyroFireDurationMs();

using namespace PyroRestorePolicy;

TEST(PyroRestorePolicy, TheSnapshotRecordsAStartedChannel) {
    EXPECT_EQ(snapshotValue(Phase::Idle),     kNotStarted);
    EXPECT_EQ(snapshotValue(Phase::Settling), kStarted);
    EXPECT_EQ(snapshotValue(Phase::Firing),   kStarted);
    EXPECT_EQ(snapshotValue(Phase::Done),     kFired);
}

TEST(PyroRestorePolicy, AStartedOrFiredChannelIsSpent) {
    EXPECT_FALSE(restoredSpent(kNotStarted)) << "an unstarted channel may still fire";
    EXPECT_TRUE(restoredSpent(kFired));
    EXPECT_TRUE(restoredSpent(kStarted));
}

TEST(PyroRestorePolicy, OlderSnapshotsRestoreAsBefore) {
    // Every older writer wrote 0 or 1 (Done).
    EXPECT_FALSE(restoredSpent(0));
    EXPECT_TRUE(restoredSpent(1));
    EXPECT_FALSE(restoredMidFire(1));
}

TEST(PyroRestorePolicy, AnUnknownValueIsSpentNeverARepeat) {
    for (int v = 3; v <= 255; ++v) EXPECT_TRUE(restoredSpent((uint8_t)v)) << v;
}

TEST(PyroRestorePolicy, OnlyAStartedChannelIsLoggedAsMidFire) {
    EXPECT_FALSE(restoredMidFire(kNotStarted));
    EXPECT_FALSE(restoredMidFire(kFired));
    EXPECT_TRUE(restoredMidFire(kStarted));
}

TEST(PyroRestorePolicy, AnyChannelLeavingIdleSendsASnapshot) {
    bool none[4] = { false, false, false, false };
    EXPECT_FALSE(edgeSnapshotDue(none));
    for (int i = 0; i < 4; ++i) {
        bool one[4] = { false, false, false, false };
        one[i] = true;
        EXPECT_TRUE(edgeSnapshotDue(one)) << i;
    }
}

TEST(PyroRestorePolicy, TheTimingIsTheFlightComputers) {
    // The window the edge snapshot has to reach the OC before current flows.
    EXPECT_EQ(fcPyroArmSettleMs(), 10u);
    EXPECT_EQ(fcPyroFireDurationMs(), 200u);
}

TEST(PyroRestorePolicy, ThePulseWaitsForItsFrameToReachTheOc) {
    const uint32_t settle = fcPyroArmSettleMs(), ring = 24;
    EXPECT_FALSE(pulseMayStart(settle - 1, settle, true, 1000, ring)) << "never inside the settle";
    EXPECT_FALSE(pulseMayStart(settle, settle, false, 0, ring)) << "the frame has not left the queue";
    EXPECT_FALSE(pulseMayStart(settle + 20, settle, true, ring + kWireMarginMs - 1, ring)) << "still in the ring";
    EXPECT_TRUE(pulseMayStart(settle + 20, settle, true, ring + kWireMarginMs, ring));
    EXPECT_TRUE(pulseMayStart(settle, settle, true, ring + kWireMarginMs, ring)) << "the settle and the frame both done";
}

TEST(PyroRestorePolicy, AStalledLinkHoldsThePulseNoLongerThanTheCap) {
    const uint32_t settle = fcPyroArmSettleMs();
    EXPECT_FALSE(pulseMayStart(kMaxPulseWaitMs - 1, settle, false, 0, 24));
    EXPECT_TRUE(pulseMayStart(kMaxPulseWaitMs, settle, false, 0, 24));
    EXPECT_TRUE(pulseMayStart(kMaxPulseWaitMs, settle, true, 0, 24));
    EXPECT_GT(kMaxPulseWaitMs, 24u + kWireMarginMs + settle) << "the cap is past the normal wait";
}

// ---------------------------------------------------------------------------
// A channel's fire as servicePyroChannels runs it, over the real link: every
// snapshot waits queue_ms in the FC's I2S queue, is then written into the DMA
// ring, and reaches the OC a ring later. The OC serves the last frame it has.
// ---------------------------------------------------------------------------

namespace {
// 8 descriptors x 256 frames x 4 B at 88.2 kHz x 4 B a frame: 23.2 ms, which
// main.cpp rounds up to 24 (txRingUs() / 1000 + 1).
constexpr uint32_t kRingMs = 24;

struct Fire
{
    uint8_t  held_at_reset = 0;   // what the OC holds when the FC resets
    uint32_t pulse_ms      = 0;   // when the FET turned on; 0 = not by the reset
    uint8_t  held_at_pulse = 0;   // what the OC held when it did
};

// The trigger is met at fire_ms; the FC resets at reset_ms. Snapshots go at
// 10 Hz from cadence_phase and, with `edge`, on the pass the channel leaves
// Idle. `gate` holds the pulse as main.cpp does (pulseMayStart); without it
// the pulse starts at the settle (before #1760). `old_encoding` records only
// Done as fired.
Fire fly(uint32_t fire_ms, uint32_t reset_ms, uint32_t cadence_phase, uint32_t queue_ms,
         bool edge, bool gate, bool old_encoding)
{
    Phase    phase = Phase::Idle;
    uint32_t since = 0;
    std::deque<std::pair<uint32_t, uint8_t>>  to_oc;    // (arrives at the OC, value)
    std::deque<std::pair<uint32_t, uint32_t>> to_dma;   // (written into the ring, ordinal)
    uint32_t queued = 0, written = 0, edge_seq = 0, wire_ms = 0;
    bool     on_wire = false;
    uint8_t  held = kNotStarted;
    Fire     out;
    const auto send = [&](uint32_t t) {
        const uint8_t v = old_encoding ? (phase == Phase::Done ? 1 : 0) : snapshotValue(phase);
        ++queued;
        to_dma.push_back({t + queue_ms, queued});
        to_oc.push_back({t + queue_ms + kRingMs, v});
        return queued;
    };
    for (uint32_t t = 0; t < reset_ms; ++t) {
        while (!to_oc.empty() && to_oc.front().first <= t)   { held = to_oc.front().second; to_oc.pop_front(); }
        while (!to_dma.empty() && to_dma.front().first <= t) { written = to_dma.front().second; to_dma.pop_front(); }
        bool left_idle[4] = { false, false, false, false };
        if (phase == Phase::Idle && t >= fire_ms) {
            phase = Phase::Settling; since = t; edge_seq = 0; on_wire = false; left_idle[0] = true;
        }
        if (phase == Phase::Settling && edge_seq != 0 && !on_wire && written >= edge_seq) {
            on_wire = true; wire_ms = t;
        }
        const bool may = gate ? pulseMayStart(t - since, fcPyroArmSettleMs(), on_wire, t - wire_ms, kRingMs)
                              : (t - since >= fcPyroArmSettleMs());
        if (phase == Phase::Settling && may) {
            phase = Phase::Firing; since = t; out.pulse_ms = t; out.held_at_pulse = held;
        }
        if (phase == Phase::Firing && t - since >= fcPyroFireDurationMs()) { phase = Phase::Done; since = t; }
        if (edge && edgeSnapshotDue(left_idle)) edge_seq = send(t);
        if (t % 100 == cadence_phase) send(t);
    }
    out.held_at_reset = held;
    return out;
}
}  // namespace

TEST(PyroRestoreModel, TheOcHoldsStartedBeforeTheFetTurnsOn) {
    for (uint32_t queue_ms : {0u, 3u, 20u, 60u, 110u})
        for (uint32_t phase = 0; phase < 100; phase += 9) {
            const Fire f = fly(1000, 1400, phase, queue_ms, true, true, false);
            SCOPED_TRACE(::testing::Message() << "queue " << queue_ms << " ms, cadence " << phase);
            ASSERT_NE(f.pulse_ms, 0u);
            EXPECT_TRUE(restoredSpent(f.held_at_pulse)) << "the FET came on before the OC knew";
            EXPECT_LE(f.pulse_ms - 1000, kMaxPulseWaitMs);
        }
}

TEST(PyroRestoreModel, AResetAnyTimeInThePulseFindsTheChannelSpent) {
    for (uint32_t queue_ms : {0u, 20u, 60u})
        for (uint32_t phase = 0; phase < 100; phase += 13) {
            const uint32_t pulse = fly(1000, 1400, phase, queue_ms, true, true, false).pulse_ms;
            for (uint32_t reset = pulse; reset < pulse + 400; reset += 3)
                ASSERT_TRUE(restoredSpent(fly(1000, reset, phase, queue_ms, true, true, false).held_at_reset))
                    << "queue " << queue_ms << " cadence " << phase << " reset at pulse+" << (reset - pulse);
        }
}

TEST(PyroRestoreModel, AResetBeforeTheTriggerStillLetsTheChannelFire) {
    for (uint32_t phase = 0; phase < 100; phase += 7)
        for (uint32_t reset = 1; reset <= 1000; reset += 13)
            ASSERT_FALSE(restoredSpent(fly(1000, reset, phase, 0, true, true, false).held_at_reset)) << reset;
}

TEST(PyroRestoreModel, AStalledLinkStartsThePulseAtTheCap) {
    // The frame never leaves in time: the deployment goes at the cap anyway.
    const Fire f = fly(1000, 1400, 0, 1000, true, true, false);
    EXPECT_EQ(f.pulse_ms, 1000u + kMaxPulseWaitMs);
}

TEST(PyroRestoreModel, WithoutTheHoldTheRingStillRepeatedAFireCutShortEarly) {
    // The review's finding: the edge snapshot alone, pulse at the 10 ms
    // settle, an empty queue. The frame is still in the ring for the first
    // ~14 ms of the pulse, so a reset there repeats the fire.
    const uint32_t pulse = fly(1000, 1400, 50, 0, true, false, false).pulse_ms;
    uint32_t repeated = 0;
    for (uint32_t reset = pulse + 1; reset < pulse + 30; ++reset)
        repeated += !restoredSpent(fly(1000, reset, 50, 0, true, false, false).held_at_reset);
    EXPECT_GT(repeated, 10u);
}

TEST(PyroRestoreModel, BeforeThisFixAFireCutShortWasRepeated) {
    // Old encoding (Done only) at the 10 Hz cadence: every reset in the
    // settle and the pulse restored the channel unfired with its trigger met.
    uint32_t repeated = 0;
    for (uint32_t reset = 1001; reset < 1000 + 210; ++reset)
        repeated += !restoredSpent(fly(1000, reset, 0, 0, false, false, true).held_at_reset);
    EXPECT_EQ(repeated, 209u);
}
