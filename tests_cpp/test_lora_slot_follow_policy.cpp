// Base-station-run transmit slots, the rocket's half: the frame-time ring, the
// grid arithmetic, the adjust and release, and the beacon rule.
#include <gtest/gtest.h>
#include <lora_slot_follow_policy.h>

#include <vector>

using namespace lora_slot_follow;

static constexpr uint32_t P = 500;   // LORA_TX_RATE_HZ = 2

// ---- the ring --------------------------------------------------------------

TEST(LoRaSlotFollowRing, EmptyRingFindsNothing)
{
    State s;
    uint32_t ms = 0;
    EXPECT_FALSE(findTx(s, 0, ms));
}

TEST(LoRaSlotFollowRing, FindsEachOfTheLastSixteen)
{
    State s;
    for (uint16_t q = 0; q < kRingSize; ++q) recordTx(s, q, 1000u + 500u * q);
    for (uint16_t q = 0; q < kRingSize; ++q)
    {
        uint32_t ms = 0;
        ASSERT_TRUE(findTx(s, q, ms)) << q;
        EXPECT_EQ(ms, 1000u + 500u * q);
    }
}

TEST(LoRaSlotFollowRing, SeventeenthFrameEvictsTheOldest)
{
    State s;
    for (uint16_t q = 0; q <= kRingSize; ++q) recordTx(s, q, 100u * q);
    uint32_t ms = 0;
    EXPECT_FALSE(findTx(s, 0, ms));
    ASSERT_TRUE(findTx(s, 1, ms));
    EXPECT_EQ(ms, 100u);
    ASSERT_TRUE(findTx(s, kRingSize, ms));
    EXPECT_EQ(ms, 100u * kRingSize);
}

TEST(LoRaSlotFollowRing, SeqWrapsFrom65535ToZero)
{
    State s;
    uint16_t q = 65530;
    for (int i = 0; i < 16; ++i, ++q) recordTx(s, q, 7000u + 500u * (uint32_t)i);
    uint32_t ms = 0;
    ASSERT_TRUE(findTx(s, 65535, ms));
    EXPECT_EQ(ms, 7000u + 500u * 5);
    ASSERT_TRUE(findTx(s, 0, ms));
    EXPECT_EQ(ms, 7000u + 500u * 6);
    ASSERT_TRUE(findTx(s, 9, ms));
    EXPECT_EQ(ms, 7000u + 500u * 15);
    EXPECT_FALSE(findTx(s, 65529, ms));
}

TEST(LoRaSlotFollowRing, NewestRecordForASeqWins)
{
    State s;
    recordTx(s, 42, 1000);
    recordTx(s, 42, 1500);
    uint32_t ms = 0;
    ASSERT_TRUE(findTx(s, 42, ms));
    EXPECT_EQ(ms, 1500u);
}

// ---- grid arithmetic -------------------------------------------------------

TEST(LoRaSlotFollowGrid, GridAtOrAfter)
{
    EXPECT_EQ(gridAtOrAfter(1000, P, 1000), 1000u);
    EXPECT_EQ(gridAtOrAfter(1000, P, 1001), 1500u);
    EXPECT_EQ(gridAtOrAfter(1000, P, 999), 1000u);
    EXPECT_EQ(gridAtOrAfter(1000, P, 400), 500u);
    EXPECT_EQ(gridAtOrAfter(1000, P, 0), 0u);
    EXPECT_EQ(gridAtOrAfter(1000, P, 8 * P + 1000 + 1), 9 * P + 1000);
}

TEST(LoRaSlotFollowGrid, GridAtOrAfterAcrossTheMillisWrap)
{
    const uint32_t anchor = 0xFFFFFF00u;          // 256 ms before the wrap
    EXPECT_EQ(gridAtOrAfter(anchor, P, 0x100u), anchor + 2 * P);   // 744
    EXPECT_EQ(gridAtOrAfter(anchor, P, anchor + 1), anchor + P);   // 244
    EXPECT_EQ(gridAtOrAfter(0x100u, P, anchor), 0x100u - P);       // the grid runs back over the wrap
}

TEST(LoRaSlotFollowGrid, PhaseShiftWrapsIntoPlusMinusHalfAPeriod)
{
    EXPECT_EQ(phaseShiftMs(0, 200, P), 200);
    EXPECT_EQ(phaseShiftMs(0, 300, P), -200);
    EXPECT_EQ(phaseShiftMs(0, 250, P), 250);
    EXPECT_EQ(phaseShiftMs(0, 251, P), -249);
    EXPECT_EQ(phaseShiftMs(0, 500, P), 0);
    EXPECT_EQ(phaseShiftMs(0, 7 * P, P), 0);
    EXPECT_EQ(phaseShiftMs(1000, 900, P), -100);
    EXPECT_EQ(phaseShiftMs(0xFFFFFFF0u, 0x10u, P), 32);
}

// ---- free-running timing is today's ----------------------------------------

TEST(LoRaSlotFollowTiming, UnlockedIsTodaysPeriodRule)
{
    State s;
    EXPECT_FALSE(txDue(s, 1499, 1000, P));
    EXPECT_TRUE(txDue(s, 1500, 1000, P));
    EXPECT_TRUE(txDue(s, 9000, 1000, P));
    EXPECT_FALSE(txDue(s, 0xF3u, 0xFFFFFF00u, P));   // 499 ms across the wrap
    EXPECT_TRUE(txDue(s, 0xF4u, 0xFFFFFF00u, P));    // 500 ms across the wrap
    // advanceGrid / onFrameSent touch no timing while free-running.
    advanceGrid(s, 1500, P);
    onFrameSent(s, 3, 1500, /*slow=*/true);
    EXPECT_FALSE(s.locked);
    EXPECT_FALSE(s.beacon_owed);
}

// Lock onto a grid through 10 000 (frame 7 sent at 9 900, delta +100).
static State lockedAt10000()
{
    State s;
    recordTx(s, 7, 9900);
    const AdjustOutcome o = applyAdjust(s, 7, 100, /*now=*/9950, /*last_tx=*/9900, P);
    EXPECT_EQ(o.result, AdjustResult::Locked);
    EXPECT_EQ(o.next_due_ms, 10500u);   // 10 000 is under half a period after 9 900
    return s;
}

TEST(LoRaSlotFollowTiming, OnTimePassSendsOnTheGridPoint)
{
    State s = lockedAt10000();
    EXPECT_FALSE(txDue(s, 10499, 9900, P));
    ASSERT_TRUE(txDue(s, 10500, 9900, P));
    advanceGrid(s, 10500, P);
    EXPECT_EQ(s.frame_grid_ms, 10500u);
    EXPECT_EQ(s.next_due_ms, 11000u);
}

TEST(LoRaSlotFollowTiming, LateWithinAPeriodSendsAtOnceAndKeepsTheGrid)
{
    State s = lockedAt10000();
    ASSERT_TRUE(txDue(s, 10800, 9900, P));
    advanceGrid(s, 10800, P);
    EXPECT_EQ(s.frame_grid_ms, 10500u);
    EXPECT_EQ(s.next_due_ms, 11000u);   // not 10 800 + 500
}

TEST(LoRaSlotFollowTiming, OnePeriodLateStepsOneGridPoint)
{
    State s = lockedAt10000();
    ASSERT_TRUE(txDue(s, 11030, 9900, P));
    advanceGrid(s, 11030, P);
    EXPECT_EQ(s.frame_grid_ms, 11000u);
    EXPECT_EQ(s.next_due_ms, 11500u);
}

TEST(LoRaSlotFollowTiming, SevenPeriodsLateStepsSevenGridPoints)
{
    State s = lockedAt10000();
    const uint32_t now = 10500 + 7 * P + 120;
    ASSERT_TRUE(txDue(s, now, 9900, P));
    advanceGrid(s, now, P);
    EXPECT_EQ(s.frame_grid_ms, 10500u + 7 * P);
    EXPECT_EQ(s.next_due_ms, 10500u + 8 * P);
    EXPECT_EQ(phaseShiftMs(10000, s.next_due_ms, P), 0);   // never re-anchored to now
}

TEST(LoRaSlotFollowTiming, GridRunsThroughTheMillisWrap)
{
    State s;
    const uint32_t sent = 0xFFFFFE00u;   // 512 ms before the wrap
    recordTx(s, 100, sent);
    ASSERT_EQ(applyAdjust(s, 100, 0, sent + 20, sent, P).result, AdjustResult::Locked);
    EXPECT_EQ(s.next_due_ms, sent + P);              // 0xFFFFFFF4
    uint32_t now = sent + P;
    std::vector<uint32_t> sends;
    for (int i = 0; i < 4; ++i)
    {
        ASSERT_TRUE(txDue(s, now, now - P, P));
        advanceGrid(s, now, P);
        sends.push_back(now);
        EXPECT_FALSE(txDue(s, now + P - 1, now, P));
        now += P;
    }
    EXPECT_EQ(sends[1], sent + 2 * P);               // 0x1E8, after the wrap
    // A 3-period stall across the wrap still lands on the grid.
    const uint32_t late = s.next_due_ms + 3 * P + 7;
    ASSERT_TRUE(txDue(s, late, now - P, P));
    advanceGrid(s, late, P);
    EXPECT_EQ(phaseShiftMs(sent, s.frame_grid_ms, P), 0);
    EXPECT_EQ((uint32_t)(late - s.frame_grid_ms), 7u);
}

TEST(LoRaSlotFollowTiming, FailedAttemptUsesUpItsGridPoint)
{
    // advanceGrid runs where last_lora_tx_ms is set, before send() reports; a
    // failed send is retried on the next grid point, as today a period later.
    State s = lockedAt10000();
    advanceGrid(s, 10500, P);
    EXPECT_FALSE(txDue(s, 10501, 10500, P));
    EXPECT_TRUE(txDue(s, 11000, 10500, P));
}

// ---- the adjust ------------------------------------------------------------

TEST(LoRaSlotFollowAdjust, DecodesLittleEndianPayload)
{
    uint16_t ref = 0;
    int16_t  d   = 0;
    const uint8_t a[4] = {0x34, 0x12, 0x2C, 0x01};
    decodeAdjust(a, ref, d);
    EXPECT_EQ(ref, 0x1234);
    EXPECT_EQ(d, 300);
    const uint8_t b[4] = {0xFF, 0xFF, 0x38, 0xFF};
    decodeAdjust(b, ref, d);
    EXPECT_EQ(ref, 65535);
    EXPECT_EQ(d, -200);
    const uint8_t c[4] = {0x00, 0x00, 0x00, 0x80};
    decodeAdjust(c, ref, d);
    EXPECT_EQ(d, LORA_TX_SLOT_RELEASE);
}

TEST(LoRaSlotFollowAdjust, AddressedToThisRocketOrAReleaseBroadcast)
{
    EXPECT_TRUE(adjustAddressedToUs(2, 2, 40));
    EXPECT_TRUE(adjustAddressedToUs(2, 2, LORA_TX_SLOT_RELEASE));
    EXPECT_TRUE(adjustAddressedToUs(0xFF, 2, LORA_TX_SLOT_RELEASE));
    EXPECT_FALSE(adjustAddressedToUs(0xFF, 2, 40));   // a grid names one rocket's frame
    EXPECT_FALSE(adjustAddressedToUs(3, 2, 40));
    EXPECT_FALSE(adjustAddressedToUs(3, 2, LORA_TX_SLOT_RELEASE));
}

TEST(LoRaSlotFollowAdjust, EvictedRefSeqIsIgnored)
{
    State s;
    for (uint16_t q = 0; q <= kRingSize; ++q) recordTx(s, q, 1000u + P * q);
    const uint32_t last = 1000u + P * kRingSize;
    const AdjustOutcome o = applyAdjust(s, 0, 100, last + 50, last, P);
    EXPECT_EQ(o.result, AdjustResult::RefNotFound);
    EXPECT_FALSE(s.locked);
    EXPECT_TRUE(txDue(s, last + P, last, P));     // still today's timing
}

TEST(LoRaSlotFollowAdjust, IgnoredAdjustLeavesAnExistingLockAlone)
{
    State s = lockedAt10000();
    const uint32_t due = s.next_due_ms;
    EXPECT_EQ(applyAdjust(s, 999, 100, 10100, 9900, P).result, AdjustResult::RefNotFound);
    EXPECT_TRUE(s.locked);
    EXPECT_EQ(s.next_due_ms, due);
}

TEST(LoRaSlotFollowAdjust, ReapplyingTheSameAdjustGivesTheSameGrid)
{
    State s;
    recordTx(s, 7, 9900);
    const AdjustOutcome a = applyAdjust(s, 7, 100, 9950, 9900, P);
    const AdjustOutcome b = applyAdjust(s, 7, 100, 9960, 9900, P);
    EXPECT_EQ(a.next_due_ms, b.next_due_ms);
    EXPECT_EQ(b.shift_ms, 0);
    EXPECT_TRUE(b.was_locked);

    // ... and after frames have gone out on the grid (anchor rebased).
    advanceGrid(s, 10500, P);
    onFrameSent(s, 8, 10500, false);
    const AdjustOutcome c = applyAdjust(s, 7, 100, 10620, 10500, P);
    EXPECT_EQ(c.result, AdjustResult::Locked);
    EXPECT_EQ(c.shift_ms, 0);
    EXPECT_EQ(c.next_due_ms, 11000u);
}

TEST(LoRaSlotFollowAdjust, SameGridFromALaterFrameIsNoShift)
{
    State s = lockedAt10000();
    advanceGrid(s, 10503, P);          // 3 ms of loop latency
    onFrameSent(s, 8, 10503, false);
    // The base station measured frame 8 and asks for the same grid: -3 ms.
    const AdjustOutcome o = applyAdjust(s, 8, -3, 10700, 10503, P);
    EXPECT_EQ(o.shift_ms, 0);
    EXPECT_EQ(o.next_due_ms, 11000u);
}

TEST(LoRaSlotFollowAdjust, MoveNeverPutsTwoFramesOnTopOfEachOther)
{
    // Free-running, last frame at 0 (seq 4).  A +200 move puts the next frame
    // at 700, a -200 move at 300: 500 +- the shift, never under half a period.
    {
        State s;
        recordTx(s, 4, 0);
        const AdjustOutcome o = applyAdjust(s, 4, 200, 100, 0, P);
        EXPECT_EQ(o.shift_ms, 200);
        EXPECT_EQ(o.next_due_ms, 700u);
    }
    {
        State s;
        recordTx(s, 4, 0);
        const AdjustOutcome o = applyAdjust(s, 4, -200, 100, 0, P);
        EXPECT_EQ(o.shift_ms, -200);
        EXPECT_EQ(o.next_due_ms, 300u);
    }
    {
        State s;
        recordTx(s, 4, 0);
        const AdjustOutcome o = applyAdjust(s, 4, -249, 100, 0, P);
        EXPECT_EQ(o.next_due_ms, 251u);
    }
    {
        // Delta of whole periods is the same grid as no delta.
        State s;
        recordTx(s, 4, 0);
        const AdjustOutcome o = applyAdjust(s, 4, (int16_t)(3 * P + 20), 100, 0, P);
        EXPECT_EQ(o.shift_ms, 20);
        EXPECT_EQ(o.next_due_ms, 520u);
    }
}

TEST(LoRaSlotFollowAdjust, ReleaseReturnsToFreeRunning)
{
    State s = lockedAt10000();
    advanceGrid(s, 10500, P);
    const AdjustOutcome o = applyAdjust(s, 0, LORA_TX_SLOT_RELEASE, 10600, 10500, P);
    EXPECT_EQ(o.result, AdjustResult::Released);
    EXPECT_TRUE(o.was_locked);
    EXPECT_FALSE(s.locked);
    EXPECT_FALSE(txDue(s, 10999, 10500, P));
    EXPECT_TRUE(txDue(s, 11000, 10500, P));      // a period after the last attempt
    EXPECT_EQ(applyAdjust(s, 0, LORA_TX_SLOT_RELEASE, 10700, 10500, P).result,
              AdjustResult::AlreadyFree);
}

TEST(LoRaSlotFollowAdjust, ReleaseNeedsNoRingEntry)
{
    State s = lockedAt10000();
    EXPECT_EQ(applyAdjust(s, 55555, LORA_TX_SLOT_RELEASE, 10000, 9900, P).result,
              AdjustResult::Released);
}

TEST(LoRaSlotFollowAdjust, UnlockReportsWhetherItWasLocked)
{
    State s = lockedAt10000();
    EXPECT_TRUE(unlock(s));
    EXPECT_FALSE(s.locked);
    EXPECT_FALSE(unlock(s));
}

// ---- the beacon ------------------------------------------------------------

TEST(LoRaSlotFollowBeacon, FreeRunningLeavesTheBeaconAlone)
{
    State s;
    EXPECT_TRUE(beaconAllowed(s, 12345, 71, P));
}

TEST(LoRaSlotFollowBeacon, LockedBeaconsOnlyRightAfterASlowFrame)
{
    State s = lockedAt10000();
    advanceGrid(s, 10500, P);
    onFrameSent(s, 8, 10500, /*slow=*/false);
    EXPECT_FALSE(beaconAllowed(s, 10600, 40, P));   // FAST 102 ms + beacon overflows the slot

    advanceGrid(s, 11000, P);
    onFrameSent(s, 9, 11000, /*slow=*/true);
    // SLOW 56 ms + loop latency, then a 71 ms beacon: 60 + 71 + 10 <= 166.
    EXPECT_TRUE(beaconAllowed(s, 11060, 71, P));
    // Too late in the slot for that beacon to end inside it.
    EXPECT_FALSE(beaconAllowed(s, 11090, 71, P));
    onBeaconSent(s);
    EXPECT_FALSE(beaconAllowed(s, 11060, 71, P));   // one per SLOW frame

    // The next frame (FAST) clears an unsent beacon.
    advanceGrid(s, 14000, P);
    onFrameSent(s, 15, 14000, /*slow=*/true);
    advanceGrid(s, 14500, P);
    onFrameSent(s, 16, 14500, /*slow=*/false);
    EXPECT_FALSE(beaconAllowed(s, 14560, 40, P));
}

TEST(LoRaSlotFollowBeacon, LateSlowFrameCountsFromItsGridPoint)
{
    State s = lockedAt10000();
    advanceGrid(s, 10540, P);            // SLOW went 40 ms late
    onFrameSent(s, 8, 10540, true);
    EXPECT_FALSE(beaconAllowed(s, 10600, 71, P));   // 100 + 71 + 10 > 166
    EXPECT_TRUE(beaconAllowed(s, 10600, 40, P));    // a short name still fits
}

TEST(LoRaSlotFollowBeacon, ReleaseDropsAnOwedBeaconGate)
{
    State s = lockedAt10000();
    advanceGrid(s, 10500, P);
    onFrameSent(s, 8, 10500, true);
    unlock(s);
    EXPECT_TRUE(beaconAllowed(s, 10560, 71, P));    // back to main.cpp's own gates
    EXPECT_FALSE(s.beacon_owed);
}

// ---- a run through the loop ------------------------------------------------

// serviceLoRa's order: txDue, advanceGrid at the commit, onFrameSent after the
// radio took it.  1 ms passes, a 0-3 ms loop latency, a radio busy for the
// frame's airtime.  Free-running first, then a lock, then a release.
TEST(LoRaSlotFollowRun, FramesLandOnTheGridAfterTheLock)
{
    State s;
    uint32_t last_tx = 0;
    uint16_t seq = 0;
    uint32_t busy_until = 0;
    std::vector<uint32_t> free_sends, locked_sends, released_sends;
    bool adjusted = false, released = false;

    for (uint32_t now = 1; now < 30000; ++now)
    {
        if (!adjusted && now == 6000)
        {
            // Base station: frame (seq - 2) should have gone 137 ms later.
            const AdjustOutcome o = applyAdjust(s, (uint16_t)(seq - 2), 137, now, last_tx, P);
            ASSERT_EQ(o.result, AdjustResult::Locked);
            adjusted = true;
        }
        if (!released && now == 20000)
        {
            ASSERT_EQ(applyAdjust(s, 0, LORA_TX_SLOT_RELEASE, now, last_tx, P).result,
                      AdjustResult::Released);
            released = true;
        }
        if ((now % 7) != 0) continue;          // a pass every 7 ms
        if (!txDue(s, now, last_tx, P)) continue;
        if ((int32_t)(now - busy_until) < 0) continue;   // canSend() false
        const bool slow = (seq % 6) == 5;
        last_tx = now;
        advanceGrid(s, now, P);
        onFrameSent(s, seq, now, slow);
        busy_until = now + (slow ? 56 : 102);
        (s.locked ? locked_sends : (released ? released_sends : free_sends)).push_back(now);
        seq++;
    }

    ASSERT_GE(free_sends.size(), 10u);
    ASSERT_GE(locked_sends.size(), 20u);
    ASSERT_GE(released_sends.size(), 10u);
    for (size_t i = 1; i < free_sends.size(); ++i)
    {
        EXPECT_GE(free_sends[i] - free_sends[i - 1], P);
        EXPECT_LT(free_sends[i] - free_sends[i - 1], P + 7);
    }
    // Locked frames go out within one loop pass of their grid point, and the
    // grid never drifts with the pass latency.
    const uint32_t g0 = s.anchor_ms;   // still a grid point after the release
    for (uint32_t t : locked_sends)
    {
        const int32_t off = phaseShiftMs(g0, t, P);
        EXPECT_GE(off, 0) << t;
        EXPECT_LT(off, 7) << t;
    }
    for (size_t i = 1; i < locked_sends.size(); ++i)
    {
        EXPECT_GE(locked_sends[i] - locked_sends[i - 1], P - 7);
        EXPECT_LE(locked_sends[i] - locked_sends[i - 1], P + 7);
    }
    for (size_t i = 1; i < released_sends.size(); ++i)
    {
        EXPECT_GE(released_sends[i] - released_sends[i - 1], P);
    }
}

// ---- random phase on a release or an ID change (bench 2026-10-21) ---------

TEST(LoRaSlotFollowRandomPhase, NextFrameLandsAtTheDrawnPointOfThePeriod)
{
    const uint32_t P = 500;
    for (uint32_t rnd : {0u, 1u, 166u, 499u, 500u, 12345u, 0xFFFFFFFFu})
    {
        const uint32_t now  = 100000;
        const uint32_t last = lastTxForRandomPhase(now, P, rnd);
        const uint32_t wait = rnd % P;
        State s;   // free-running
        if (wait > 0) EXPECT_FALSE(txDue(s, now + wait - 1, last, P)) << rnd;
        EXPECT_TRUE(txDue(s, now + wait, last, P)) << rnd;
    }
}

TEST(LoRaSlotFollowRandomPhase, WorksAcrossTheMillisWrap)
{
    const uint32_t P = 500;
    const uint32_t now = 0xFFFFFF00u;
    const uint32_t last = lastTxForRandomPhase(now, P, 300);
    State s;
    EXPECT_FALSE(txDue(s, now + 299, last, P));
    EXPECT_TRUE(txDue(s, now + 300, last, P));     // past the wrap
}

TEST(LoRaSlotFollowRandomPhase, TwoRocketsOnOneGridPointComeApart)
{
    // Two rockets locked onto one grid point (they shared a rid when the
    // base station slotted it). After a release each draws its own phase.
    const uint32_t P = 500, now = 50000;
    const uint32_t a = lastTxForRandomPhase(now, P, 37);
    const uint32_t b = lastTxForRandomPhase(now, P, 291);
    EXPECT_EQ((uint32_t)(b - a), 254u);   // 254 ms apart: FAST frames (102 ms) clear
}

TEST(LoRaSlotFollowRandomPhase, ZeroPeriodIsHarmless)
{
    EXPECT_EQ(lastTxForRandomPhase(1234, 0, 99), 1234u);
}
