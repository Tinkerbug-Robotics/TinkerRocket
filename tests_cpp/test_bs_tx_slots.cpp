#include <gtest/gtest.h>

#include <cstdlib>
#include <vector>

#include "bs_tx_slots.h"

using namespace bs_tx_slots;

// Three rockets, one channel: the base station times every rocket's frames and
// moves each one onto its own third of the 500 ms period with a targeted
// LORA_CMD_TX_SLOT_ADJUST [ref_seq][delta]. These pin the controller's rules.

namespace {

// Fill a rocket's window with frames on a grid at `phase` (+ per-frame delay).
void feed(RocketSlot& r, uint32_t phase, uint16_t first_seq, const uint32_t delays[4],
          uint32_t t0 = 100000)
{
    for (int i = 0; i < 4; i++) {
        const uint32_t start = t0 + phase + (uint32_t)i * 500 + delays[i];
        r.onFast((uint16_t)(first_seq + i), start, start + 110);
    }
}

const uint32_t kNoDelay[4] = {0, 0, 0, 0};

}  // namespace

// ---- Geometry ----

TEST(BsTxSlots, SlotOffsetsAreThirdsOfThePeriod) {
    EXPECT_EQ(slotOffsetMs(0), 0u);
    EXPECT_EQ(slotOffsetMs(1), 167u);
    EXPECT_EQ(slotOffsetMs(2), 333u);
    EXPECT_EQ(kSlots, 3);
}

TEST(BsTxSlots, PreferredSlotIsRidMinusOneModThree) {
    EXPECT_EQ(preferredSlot(1), 0);
    EXPECT_EQ(preferredSlot(2), 1);
    EXPECT_EQ(preferredSlot(3), 2);
    EXPECT_EQ(preferredSlot(4), 0);
    EXPECT_EQ(preferredSlot(0), -1);      // unset
    EXPECT_EQ(preferredSlot(0xFF), -1);   // broadcast
}

TEST(BsTxSlots, DeltaWrapsIntoPlusMinusHalfAPeriod) {
    EXPECT_EQ(wrapHalf(0), 0);
    EXPECT_EQ(wrapHalf(249), 249);
    EXPECT_EQ(wrapHalf(250), 250);        // (-250, 250]: +250 stays
    EXPECT_EQ(wrapHalf(-250), 250);       // -250 is the same phase as +250
    EXPECT_EQ(wrapHalf(251), -249);
    EXPECT_EQ(wrapHalf(-251), 249);
    EXPECT_EQ(wrapHalf(1260), 260 - 500);
    EXPECT_EQ(wrapHalf(-1740), -240);     // -1740 = 260 = -240 (mod 500)
    EXPECT_EQ(phaseDiff(1000u, 0xFFFFFF00u), wrapHalf(1000 + 256));   // across the millis() wrap
}

TEST(BsTxSlots, AirtimeFitsOnlyWhereAFastFramePlusGuardFitsASlot) {
    EXPECT_EQ(fastAirtimeMs(8, 250.0f, 5), 102u);
    EXPECT_TRUE (airtimeFits(8, 250.0f, 5));    // 102 + 30 <= 166.7
    EXPECT_TRUE (airtimeFits(7, 250.0f, 5));
    EXPECT_FALSE(airtimeFits(9, 250.0f, 5));    // 183 ms frame
    EXPECT_FALSE(airtimeFits(8, 250.0f, 8));    // CR4/8: 148 + 30 > 166.7
    EXPECT_FALSE(airtimeFits(8, 125.0f, 5));    // BW125 doubles it
}

// ---- Slot assignment ----

TEST(BsTxSlots, AssignmentFollowsTheRidRule) {
    {
        const uint8_t rids[] = {1, 2, 3};
        int8_t out[3];
        assignSlots(rids, nullptr, 3, out);
        EXPECT_EQ(out[0], 0); EXPECT_EQ(out[1], 1); EXPECT_EQ(out[2], 2);
    }
    {
        // 1 and 4 both want slot 0: the higher rid takes the lowest free slot.
        const uint8_t rids[] = {4, 1};
        int8_t out[2];
        assignSlots(rids, nullptr, 2, out);
        EXPECT_EQ(out[1], 0);
        EXPECT_EQ(out[0], 1);
    }
    {
        // 1, 2, 4: 4 clashes with 1; slot 1 is 2's, so 4 gets 2.
        const uint8_t rids[] = {2, 4, 1};
        int8_t out[3];
        assignSlots(rids, nullptr, 3, out);
        EXPECT_EQ(out[2], 0); EXPECT_EQ(out[0], 1); EXPECT_EQ(out[1], 2);
    }
    {
        // Every rocket shipped as rid 1 plus two renumbered: 1, 4, 7 all want 0.
        const uint8_t rids[] = {7, 1, 4};
        int8_t out[3];
        assignSlots(rids, nullptr, 3, out);
        EXPECT_EQ(out[1], 0); EXPECT_EQ(out[2], 1); EXPECT_EQ(out[0], 2);
    }
    {
        // A fourth rocket gets no slot; empty entries get none.
        const uint8_t rids[] = {1, 2, 3, 5, 0};
        int8_t out[5];
        assignSlots(rids, nullptr, 5, out);
        EXPECT_EQ(out[3], -1);
        EXPECT_EQ(out[4], -1);
    }
}

TEST(BsTxSlots, FrozenAssignmentKeepsTheSlotsItHas) {
    // In flight: rid 4 holds slot 1 (it lost the clash with rid 1 earlier).
    // rid 2 shows up and wants slot 1 — rid 4 keeps it, rid 2 takes what is left.
    const uint8_t rids[] = {1, 4, 2};
    const int8_t keep[] = {0, 1, -1};
    int8_t out[3];
    assignSlots(rids, keep, 3, out);
    EXPECT_EQ(out[0], 0);
    EXPECT_EQ(out[1], 1);
    EXPECT_EQ(out[2], 2);
}

// ---- Timing estimate ----

TEST(BsTxSlots, EstimateIsTheEarliestOfTheLastFourFastFrames) {
    RocketSlot r;
    const uint32_t delays[4] = {3, 0, 7, 2};   // the second frame was measured best
    feed(r, 40, 500, delays);
    uint16_t ref = 0;
    uint32_t start = 0;
    ASSERT_TRUE(r.estimate(ref, start));
    EXPECT_EQ(ref, 501);
    EXPECT_EQ(start, 100000u + 40 + 500);
}

TEST(BsTxSlots, EstimateNeedsAFullWindow) {
    RocketSlot r;
    uint16_t ref;
    uint32_t start;
    r.onFast(1, 1000, 1110);
    r.onFast(2, 1500, 1610);
    r.onFast(3, 2000, 2110);
    EXPECT_FALSE(r.estimate(ref, start));
    r.onFast(4, 2500, 2610);
    EXPECT_TRUE(r.estimate(ref, start));
}

TEST(BsTxSlots, EstimateSurvivesASkippedPeriod) {
    // A SLOW frame (or a lost one) between FAST frames: the comparison is
    // modulo the period, not by seq count.
    RocketSlot r;
    r.onFast(10, 10000 + 4, 0);
    r.onFast(11, 10500 + 1, 0);
    r.onFast(13, 11500 + 0, 0);   // seq 12 was SLOW
    r.onFast(14, 12000 + 6, 0);
    uint16_t ref;
    uint32_t start;
    ASSERT_TRUE(r.estimate(ref, start));
    EXPECT_EQ(ref, 13);
    EXPECT_EQ(start, 11500u);
}

TEST(BsTxSlots, OldWindowFramesAreDropped) {
    RocketSlot r;
    r.onFast(1, 1000, 1100);
    r.onFast(2, 1500, 1600);
    r.onFast(3, 2000, 2100);
    r.onFast(9, 6000, 6100);    // a 4 s dropout: start over
    EXPECT_EQ(r.n, 1);
}

// ---- Controller ----

TEST(BsTxSlots, FirstRocketFixesTheBaseAndStillGetsAnAdjust) {
    Controller c;
    RocketSlot r;
    r.slot = 1;                      // rid 2
    feed(r, 123, 50, kNoDelay);
    const Decision d = c.evaluate(r, true, true, false);
    EXPECT_TRUE(d.base_set);
    EXPECT_TRUE(c.have_base);
    EXPECT_TRUE(d.adjust);           // free-running: put it on a grid
    EXPECT_EQ(d.delta_ms, 0);        // ...right where it is
    EXPECT_EQ(d.err_ms, 0);
    EXPECT_EQ(d.ref_seq, 53);
    // Slot 0 is 167 ms before this rocket's frames.
    EXPECT_EQ(phaseDiff(c.targetMs(1), 100000u + 123), 0);
    EXPECT_EQ(phaseDiff(c.targetMs(0), 100000u + 123 - 167), 0);
}

TEST(BsTxSlots, SecondRocketIsMovedOntoItsSlot) {
    Controller c;
    RocketSlot a, b;
    a.slot = 0;
    b.slot = 1;
    feed(a, 0, 10, kNoDelay);
    ASSERT_TRUE(c.evaluate(a, true, true, false).base_set);
    // Rocket b free-running 30 ms after a: its slot is +167, so it is 137 early.
    feed(b, 30, 900, kNoDelay);
    const Decision d = c.evaluate(b, true, true, false);
    EXPECT_FALSE(d.base_set);
    EXPECT_TRUE(d.adjust);
    EXPECT_EQ(d.err_ms, 30 - 167);
    EXPECT_EQ(d.delta_ms, 137);
    EXPECT_EQ(d.ref_seq, 903);
}

TEST(BsTxSlots, DeltaTakesTheShortWayRound) {
    Controller c;
    RocketSlot a, b;
    a.slot = 0;
    b.slot = 2;                      // target +333
    feed(a, 0, 1, kNoDelay);
    c.evaluate(a, true, true, false);
    feed(b, 50, 1, kNoDelay);        // 283 early = 217 late
    const Decision d = c.evaluate(b, true, true, false);
    EXPECT_EQ(d.err_ms, 217);
    EXPECT_EQ(d.delta_ms, -217);
}

TEST(BsTxSlots, TenMillisecondToleranceOnTheGround) {
    Controller c;
    RocketSlot a;
    a.slot = 0;
    feed(a, 0, 1, kNoDelay);
    c.evaluate(a, true, true, false);
    Controller::onAdjustQueued(a, 0);
    Controller::onAdjustDone(a);

    // On target within 10 ms after the adjust: locked, no re-send.
    RocketSlot ok = a;
    feed(ok, 10, 20, kNoDelay, 200000);
    Decision d = c.evaluate(ok, true, true, false);
    EXPECT_FALSE(d.adjust);
    EXPECT_TRUE(d.lock);
    EXPECT_TRUE(ok.locked);
    // Next frames still on target: no new lock edge.
    feed(ok, 10, 30, kNoDelay, 300000);
    d = c.evaluate(ok, true, true, false);
    EXPECT_FALSE(d.lock);
    EXPECT_FALSE(d.adjust);

    // 11 ms off: re-sent.
    RocketSlot off = a;
    feed(off, 11, 20, kNoDelay, 200000);
    d = c.evaluate(off, true, true, false);
    EXPECT_TRUE(d.adjust);
    EXPECT_EQ(d.delta_ms, -11);
    EXPECT_FALSE(d.lock);
}

TEST(BsTxSlots, LockedRocketThatDriftsOffIsUnlockedAndReadjusted) {
    Controller c;
    RocketSlot a;
    a.slot = 0;
    feed(a, 0, 1, kNoDelay);
    c.evaluate(a, true, true, false);
    Controller::onAdjustQueued(a, 0);
    Controller::onAdjustDone(a);
    feed(a, 2, 10, kNoDelay, 200000);
    ASSERT_TRUE(c.evaluate(a, true, true, false).lock);
    feed(a, 14, 20, kNoDelay, 300000);
    const Decision d = c.evaluate(a, true, true, false);
    EXPECT_TRUE(d.unlock);
    EXPECT_TRUE(d.adjust);
    EXPECT_EQ(d.err_ms, 14);
}

TEST(BsTxSlots, InFlightOnlyARebootIsAdjusted) {
    Controller c;
    RocketSlot a;
    a.slot = 0;
    feed(a, 0, 1, kNoDelay);
    c.evaluate(a, true, true, false);
    Controller::onAdjustQueued(a, 0);
    Controller::onAdjustDone(a);
    feed(a, 0, 10, kNoDelay, 200000);
    c.evaluate(a, true, true, false);   // locked

    // Drift of 49 ms during the flight: left alone.
    RocketSlot drift = a;
    feed(drift, 49, 20, kNoDelay, 300000);
    EXPECT_FALSE(c.evaluate(drift, true, true, /*inflight*/true).adjust);
    // 51 ms: that is a reboot onto a new phase — adjusted even in flight.
    RocketSlot reboot = a;
    feed(reboot, 51, 20, kNoDelay, 300000);
    EXPECT_TRUE(c.evaluate(reboot, true, true, true).adjust);
}

TEST(BsTxSlots, InFlightNothingNewIsStarted) {
    // No base yet and a rocket in flight: no base, no adjust.
    Controller c;
    RocketSlot a;
    a.slot = 0;
    feed(a, 0, 1, kNoDelay);
    Decision d = c.evaluate(a, true, true, true);
    EXPECT_FALSE(d.adjust);
    EXPECT_FALSE(c.have_base);
    // With a base, a never-adjusted rocket 30 ms off is not locked in flight...
    c.evaluate(a, true, true, false);
    RocketSlot b;
    b.slot = 1;
    feed(b, 167 + 30, 1, kNoDelay);
    EXPECT_FALSE(c.evaluate(b, true, true, true).adjust);
    // ...but is on the ground.
    EXPECT_TRUE(c.evaluate(b, true, true, false).adjust);
}

TEST(BsTxSlots, IneligibleRocketsAreNeverAdjusted) {
    Controller c;
    RocketSlot a;
    a.slot = 0;
    feed(a, 77, 1, kNoDelay);
    EXPECT_FALSE(c.evaluate(a, /*eligible*/false, true, false).adjust);   // hopping / other modulation / dup
    EXPECT_FALSE(c.evaluate(a, true, /*airtime*/false, false).adjust);    // SF9+, CR4/8
    RocketSlot none;
    feed(none, 77, 1, kNoDelay);                                          // no slot (4th rocket)
    EXPECT_FALSE(c.evaluate(none, true, true, false).adjust);
    c.enabled = false;                                                    // slots off
    EXPECT_FALSE(c.evaluate(a, true, true, false).adjust);
    EXPECT_FALSE(c.have_base);
}

TEST(BsTxSlots, FramesWhileTheAdjustIsQueuedAreIgnored) {
    Controller c;
    RocketSlot a;
    a.slot = 0;
    feed(a, 0, 1, kNoDelay);
    ASSERT_TRUE(c.evaluate(a, true, true, false).adjust);
    Controller::onAdjustQueued(a, 1000);
    EXPECT_EQ(a.adjusts_sent, 1);
    feed(a, 90, 10, kNoDelay, 200000);   // old phase, still retrying
    EXPECT_EQ(a.n, 0);
    EXPECT_FALSE(c.evaluate(a, true, true, false).adjust);
    EXPECT_FALSE(awaitingExpired(a, 1000 + kAwaitMaxMs));
    EXPECT_TRUE(awaitingExpired(a, 1001 + kAwaitMaxMs));
    Controller::onAdjustDone(a);
    feed(a, 0, 20, kNoDelay, 300000);
    EXPECT_EQ(a.n, 4);
}

TEST(BsTxSlots, ARocketThatIgnoresAdjustsIsResentEverMoreRarely) {
    Controller c;
    RocketSlot a;
    a.slot = 0;
    feed(a, 0, 1, kNoDelay);
    c.evaluate(a, true, true, false);
    RocketSlot b;
    b.slot = 1;
    std::vector<int> frames_between;
    int frames = 0;
    uint32_t t = 200000;
    uint16_t seq = 1;
    for (int i = 0; i < 200; i++, t += 500) {
        b.onFast(seq++, t + 60, t + 170);   // never moves: 107 ms early
        frames++;
        const Decision d = c.evaluate(b, true, true, false);
        if (d.adjust) {
            frames_between.push_back(frames);
            frames = 0;
            Controller::onAdjustQueued(b, t);
            Controller::onAdjustDone(b);
        }
    }
    ASSERT_GE(frames_between.size(), 5u);
    EXPECT_EQ(frames_between[1], 4);    // first re-check: 4 frames
    EXPECT_EQ(frames_between[2], 8);
    EXPECT_EQ(frames_between[3], 16);
    EXPECT_EQ(frames_between[4], 32);   // capped
    if (frames_between.size() > 5) EXPECT_EQ(frames_between[5], 32);
}

TEST(BsTxSlots, ReleasePayload) {
    uint8_t p[4];
    packAdjust(p, 0, LORA_TX_SLOT_RELEASE);
    EXPECT_EQ(p[0], 0x00);
    EXPECT_EQ(p[1], 0x00);
    EXPECT_EQ(p[2], 0x00);
    EXPECT_EQ(p[3], 0x80);   // INT16_MIN little-endian
    packAdjust(p, 0x1234, -137);
    EXPECT_EQ(p[0], 0x34);
    EXPECT_EQ(p[1], 0x12);
    EXPECT_EQ((int16_t)(p[2] | (p[3] << 8)), -137);
}

// ---- Closed loop: three rockets that follow the adjusts ----
//
// A model of the rocket side as the contract describes it: free-running at
// 500 ms + its loop latency until an adjust arrives, then on the grid
// (send time of frame ref_seq) + delta + k * 500, its own clock off by `ppm`.
// Its frame goes on air `lat` ms after the send time it records (direct radio
// vs the UART daughterboard), and the base station hears it 0-1 ms late.

namespace {

struct SimRocket
{
    uint8_t  rid;
    double   lat;          // send -> on air, ms
    double   ppm;          // clock error
    double   next_send;    // BS ms
    bool     on_grid = false;
    double   grid = 0;     // BS ms of a grid tick
    uint16_t seq = 0;
    double   sent_at[16] = {};   // ring of the last 16 send times
    uint16_t sent_seq[16] = {};
    RocketSlot slot;
};

uint32_t rnd(uint32_t n) { return (uint32_t)(rand() % (int)n); }

}  // namespace

TEST(BsTxSlots, ThreeRocketsConvergeOntoSeparateSlots) {
    srand(1650);
    for (int trial = 0; trial < 20; trial++) {
        SimRocket rk[3] = {{1, 2.0, 30e-6, 0}, {2, 14.0, -25e-6, 0}, {3, 3.0, 10e-6, 0}};
        for (auto& r : rk) r.next_send = 5000 + rnd(500);
        Controller c;
        // Adjusts in flight to a rocket: deliver at `at`.
        struct Pending { int who; double at; uint16_t ref; int16_t delta; double done; };
        std::vector<Pending> pend;

        int adjusts[3] = {0, 0, 0};
        for (double now = 5000; now < 5000 + 120000; now += 1) {
            for (int i = 0; i < 3; i++) {
                SimRocket& r = rk[i];
                if (now < r.next_send) continue;
                const double send = r.next_send;
                r.sent_at[r.seq % 16] = send;
                r.sent_seq[r.seq % 16] = r.seq;
                const bool fast = (r.seq % 6) != 5;
                // BS: measured start = on air + 0..1 ms
                const uint32_t start = (uint32_t)(send + r.lat + rnd(2));
                if (fast) {
                    uint8_t rids[3] = {1, 2, 3};
                    int8_t out[3];
                    assignSlots(rids, nullptr, 3, out);
                    r.slot.slot = out[i];
                    r.slot.onFast(r.seq, start, (uint32_t)now);
                    const Decision d = c.evaluate(r.slot, true, true, false);
                    if (d.adjust) {
                        Controller::onAdjustQueued(r.slot, (uint32_t)now);
                        adjusts[i]++;
                        // Delivered 150 ms later; retries done after 700 ms.
                        pend.push_back({i, now + 150, d.ref_seq, d.delta_ms, now + 700});
                    }
                }
                r.seq++;
                const double period = 500.0 * (1.0 + r.ppm);
                if (r.on_grid) {
                    while (r.grid <= send + 1) r.grid += period;
                    r.next_send = r.grid + rnd(3);           // loop jitter
                } else {
                    r.next_send = send + period + 1 + rnd(4);  // free-running drift
                }
            }
            for (auto it = pend.begin(); it != pend.end();) {
                SimRocket& r = rk[it->who];
                if (it->at >= 0 && now >= it->at) {
                    // Rocket applies [ref][delta] if ref is in its ring.
                    const uint16_t ref = it->ref;
                    if (r.sent_seq[ref % 16] == ref) {
                        r.on_grid = true;
                        r.grid = r.sent_at[ref % 16] + it->delta;
                        const double period = 500.0 * (1.0 + r.ppm);
                        while (r.grid <= now) r.grid += period;
                        r.next_send = r.grid;
                    }
                    it->at = -1;
                }
                if (now >= it->done) {
                    Controller::onAdjustDone(r.slot);
                    it = pend.erase(it);
                } else {
                    ++it;
                }
            }
        }
        // All three on distinct slots, within tolerance, each adjusted a few times.
        bool seen[3] = {false, false, false};
        for (int i = 0; i < 3; i++) {
            ASSERT_GE(rk[i].slot.slot, 0);
            seen[rk[i].slot.slot] = true;
            EXPECT_TRUE(rk[i].slot.locked) << "trial " << trial << " rid " << (int)rk[i].rid;
            EXPECT_LE(std::abs(rk[i].slot.last_err_ms), (int)kTolMs)
                << "trial " << trial << " rid " << (int)rk[i].rid;
            EXPECT_LE(adjusts[i], 6) << "trial " << trial << " rid " << (int)rk[i].rid;
            EXPECT_GE(adjusts[i], 1) << "every rocket starts free-running";
            EXPECT_TRUE(rk[i].on_grid);
        }
        EXPECT_TRUE(seen[0] && seen[1] && seen[2]);
    }
}

TEST(BsTxSlots, RxStampIsTrustedOnlyWithinThePacketsOwnAirtime) {
    // FAST frame, 102 ms on air. The loop read it 3 ms after RX done: its own.
    EXPECT_TRUE(rxStampIsThisPackets(10003, 10000, 102));
    EXPECT_TRUE(rxStampIsThisPackets(10000, 10000, 102));
    // The loop stalled through a second packet: DIO1 never dropped, so the
    // packet read now carries the first one's stamp — at least its own
    // airtime old. Ambiguous: not timed.
    EXPECT_FALSE(rxStampIsThisPackets(10102, 10000, 102));
    EXPECT_FALSE(rxStampIsThisPackets(10170, 10000, 102));
    // A stamp from the future is not believed either.
    EXPECT_FALSE(rxStampIsThisPackets(9999, 10000, 102));
    // Across the millis() wrap.
    EXPECT_TRUE(rxStampIsThisPackets(5, 0xFFFFFFF0u, 56));
}
