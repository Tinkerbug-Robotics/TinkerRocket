// test_bs_log_policy.cpp
// Host-side gtest for the BS LoRa-logging policy helpers extracted from
// projects/base_station/main/bs_log_policy.h (#137).
//
// Currently covers parseSequentialFilename() — the sscanf bug that
// produced bogus `lora_9886.csv` filenames was the most concrete
// regression risk from the 5/9/26 test flights.  Other policy decisions
// (silence-timeout threshold, auto-start gating) live as simple inline
// conditionals in main.cpp and would need an integration / bench test to
// exercise meaningfully — that's what tests/bench/test_lora_log_capture.py
// is for.

#include <gtest/gtest.h>
#include <string>

#include "bs_log_policy.h"

// ---------------------------------------------------------------------------
// parseSequentialFilename()
// ---------------------------------------------------------------------------

TEST(BSLogPolicyParseFilename, AcceptsValidSequentialNames)
{
    uint16_t n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_001.bin", n));
    EXPECT_EQ(n, 1u);

    n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_42.bin", n));
    EXPECT_EQ(n, 42u);

    n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_9886.bin", n));
    EXPECT_EQ(n, 9886u);

    n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_65535.bin", n));
    EXPECT_EQ(n, 65535u);

    n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_0.bin", n));
    EXPECT_EQ(n, 0u);
}

TEST(BSLogPolicyParseFilename, RejectsTimestampedNames)
{
    // Regression for the core #137 sscanf bug.  Pre-fix:
    //   sscanf("lora_20260509_164143.bin", "lora_%hu.bin", &n) == 1, n=9885
    // This made findNextFileNumber inflate max+1 to 9886, producing
    // `lora_9886.csv` on the next no-time-sync boot.
    uint16_t n = 0xFFFF;
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_20260509_164143.bin", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_20260509_144144.bin", n));
    // _2 collision-suffix variant should also reject (it's still timestamped)
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_20260509_164143_2.bin", n));
}

TEST(BSLogPolicyParseFilename, RejectsUnrelatedNames)
{
    uint16_t n = 0xFFFF;
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "flight_20260509_144128.bin", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora.bin", n));               // no number
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_001.txt", n));           // wrong extension
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_001", n));               // missing extension
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_001.csv.bak", n));       // trailing junk
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        ".write_test", n));            // BS boot probe file
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(nullptr, n));
}

TEST(BSLogPolicyParseFilename, RejectsNegativeSignAndNonDigits)
{
    // The parser walks digits explicitly (rather than letting sscanf %hu
    // accept signs / whitespace and wrap), so any non-digit between
    // "lora_" and ".bin" disqualifies the match.
    uint16_t n = 0xFFFF;
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_-1.bin", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_+1.bin", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_ 1.bin", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_1a.bin", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_abc.bin", n));
}

TEST(BSLogPolicyParseFilename, RejectsOversized)
{
    // 6-digit run can't fit uint16 (max=65535) — and even values <=65535
    // that happen to have 6 digits ("099999" hypothetically) are rejected
    // because they'd have come from a sequence we never produce.  Real-
    // world BS code writes %03u, so anything >5 digits is malformed.
    uint16_t n = 0xFFFF;
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_99999.bin", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_123456.bin", n));
    // Boundary: 65535 is the highest uint16 (5 digits) — accepted.
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_65535.bin", n));
}

// ---------------------------------------------------------------------------
// #381 — multi-rocket log transitions
//
// The regression scenario: rocket A has landed and keeps transmitting LANDED
// at ~2 Hz while rocket B powers up READY on the pad. With the old single
// global last_rocket_state, every interleaved A-then-B pair looked like a
// LANDED transition (close!) followed by a non-LANDED packet (reopen!) —
// one CSV file per interleave cycle. These tests drive the per-rocket
// transition state + aggregate close/lock policy through that exact traffic.
// ---------------------------------------------------------------------------

namespace {

using bs_log_policy::RocketLogState;
using bs_log_policy::RocketView;
using bs_log_policy::updateRocketLogState;
using bs_log_policy::noFreshRocketFlying;
using bs_log_policy::anySafetyExpired;
using bs_log_policy::aggregateFreqLock;
using bs_log_policy::FreqLockLatch;

constexpr uint32_t FRESH_MS  = 5 * 60 * 1000;   // config::LOG_SILENCE_TIMEOUT_MS
constexpr uint32_t SAFETY_MS = 20 * 60 * 1000;  // config::LOG_INFLIGHT_SAFETY_MS
constexpr int      NSLOTS    = 4;

// Minimal stand-in for the main.cpp wiring: per-slot state, the same
// open/close decisions serviceUplink's RX block makes, and open/close
// counters so a test can assert "exactly one file".
struct SimBS {
    RocketLogState slots[NSLOTS] = {};
    uint32_t       last_seen[NSLOTS] = {};
    // #835 item 6 residual: telemetry-only clock. packet() advances both;
    // beacon() advances only last_seen, as the firmware's rx_len>3 branch does.
    uint32_t       last_telem[NSLOTS] = {};
    bool           slot_active[NSLOTS] = {};
    bool           logging = false;
    int            opens = 0;
    int            closes = 0;

    void views(RocketView out[NSLOTS]) const {
        for (int i = 0; i < NSLOTS; ++i) {
            out[i].active            = slot_active[i];
            out[i].last_seen_ms      = last_seen[i];
            out[i].state             = slots[i].last_state;
            out[i].inflight_entry_ms = slots[i].inflight_entry_ms;
            out[i].freq_lock         = slots[i].freq_lock;
            out[i].last_telem_ms     = last_telem[i];
        }
    }

    // A NAME BEACON: the base station heard the rocket, but decoded no
    // telemetry. Mirrors main.cpp's rx_len>3 branch, which stamps last_seen_ms
    // and touches nothing else.
    void beacon(int slot, uint32_t now) {
        slot_active[slot] = true;
        last_seen[slot]   = now;
    }

    // One received packet from rocket in `slot` reporting `state` at `now`.
    // Mirrors main.cpp: auto-open gate, per-rocket fold, aggregate close.
    void packet(int slot, uint8_t state, uint32_t now) {
        if (!logging && state != LANDED) { logging = true; ++opens; }

        slot_active[slot] = true;
        const auto edges = updateRocketLogState(slots[slot], state, now);
        last_seen[slot]  = now;
        last_telem[slot] = now;

        RocketView v[NSLOTS];
        views(v);
        latch.update(v, NSLOTS, now, FRESH_MS, SAFETY_MS);  // main.cpp RX path
        if (edges.landed_edge && logging &&
            noFreshRocketFlying(v, NSLOTS, now, FRESH_MS, SAFETY_MS)) {
            logging = false;
            ++closes;
            // stopLogging() disarms all safety timers with the file.
            for (auto& s : slots) s.inflight_entry_ms = 0;
        }
    }

    // The freq-lock CACHE, driven exactly the way main.cpp drives it (#835
    // item 6).  This replaced an accessor that recomputed aggregateFreqLock
    // on every read -- a pull semantics the firmware never had, which is
    // precisely why FreqLockStableAcrossInterleave below passed against a
    // base station whose lock could never expire.
    FreqLockLatch latch;

    // The loop_bs freshness sweep: re-evaluate with no packet in hand.
    void tick(uint32_t now) {
        RocketView v[NSLOTS];
        views(v);
        latch.update(v, NSLOTS, now, FRESH_MS, SAFETY_MS);
    }

    bool flightLock(uint32_t now) const { return latch.flightLockedAt(now, FRESH_MS); }
    bool retuneLock(uint32_t now) const { return latch.retuneLockedAt(now, SAFETY_MS); }

    // The raw cached bit, with no read-side expiry -- what the pre-#835
    // firmware exposed to every consumer.
    bool rawFlight() const { return latch.flight; }
};

}  // namespace

TEST(BSLogPolicyMultiRocket, InterleavedLandedReadyDoesNotChurnFiles)
{
    // A flies and lands; B powers up READY. 60 s of interleaved 2 Hz traffic
    // must produce exactly ONE open and ZERO closes (B is still active) —
    // pre-#381 this produced a close+reopen pair roughly once per second.
    SimBS bs;
    uint32_t t = 1000;
    bs.packet(0, READY, t);          t += 500;   // A pad
    bs.packet(0, INFLIGHT, t);       t += 500;   // A flies
    bs.packet(0, LANDED, t);         t += 500;   // A lands — only rocket so far
    EXPECT_EQ(bs.closes, 1);                     // single-rocket close intact
    bs.packet(1, READY, t);          t += 500;   // B powers up -> reopen
    EXPECT_EQ(bs.opens, 2);

    for (int i = 0; i < 120; ++i) {              // 60 s interleave
        bs.packet(0, LANDED, t);     t += 250;
        bs.packet(1, READY, t);      t += 250;
    }
    EXPECT_EQ(bs.opens, 2) << "interleave must not reopen";
    EXPECT_EQ(bs.closes, 1) << "A's repeat LANDED must not close while B is fresh";
    EXPECT_TRUE(bs.logging);
}

TEST(BSLogPolicyMultiRocket, LastFreshRocketLandingCloses)
{
    SimBS bs;
    uint32_t t = 1000;
    bs.packet(0, LANDED, t);   t += 500;  // A: first packet LANDED (boot edge — no open)
    bs.packet(1, READY, t);    t += 500;
    bs.packet(1, INFLIGHT, t); t += 500;
    bs.packet(0, LANDED, t);   t += 500;  // A repeats — no edge
    EXPECT_EQ(bs.closes, 0);
    bs.packet(1, LANDED, t);              // B, the last fresh flyer, lands
    EXPECT_EQ(bs.closes, 1);
    EXPECT_FALSE(bs.logging);
}

TEST(BSLogPolicyMultiRocket, StaleRocketDoesNotVetoClose)
{
    // B was READY once but has been silent past the freshness window — a
    // powered-off rocket must not hold the log open forever.
    SimBS bs;
    uint32_t t = 1000;
    bs.packet(1, READY, t);
    bs.packet(0, READY, t + 500);
    // A flies and lands; B never transmits again.
    bs.packet(0, INFLIGHT, t + 1000);
    const uint32_t t_land = t + 1000 + FRESH_MS + 1000;  // B now stale
    bs.packet(0, LANDED, t_land);
    EXPECT_EQ(bs.closes, 1);
    EXPECT_FALSE(bs.logging);
}

TEST(BSLogPolicyMultiRocket, SafetyExpiredRocketCountsAsDown)
{
    // A went INFLIGHT and vanished (timer armed, now expired). When B lands,
    // A must not veto the close — it is presumed down, not "still flying".
    SimBS bs;
    uint32_t t = 1000;
    bs.packet(0, READY, t);          t += 500;
    bs.packet(0, INFLIGHT, t);                   // A armed at t
    const uint32_t t_a_armed = t;    t += 500;
    bs.packet(1, READY, t);          t += 500;
    bs.packet(1, INFLIGHT, t);       t += 500;

    // B lands within A's freshness window but after A's safety expiry.
    // Keep A "fresh" (recently seen) so only the expiry logic can clear it.
    uint32_t t_b_lands = t_a_armed + SAFETY_MS + 1000;
    bs.packet(0, INFLIGHT, t_b_lands - 1000);    // A still transmitting INFLIGHT (stuck)
    bs.packet(1, LANDED, t_b_lands);
    EXPECT_EQ(bs.closes, 1) << "expired-INFLIGHT rocket must not veto the close";
}

TEST(BSLogPolicyMultiRocket, PeriodicSafetyCloseHonorsOtherFlyer)
{
    // The periodic path: A armed+expired fires anySafetyExpired, but B fresh
    // and INFLIGHT (not expired) keeps the log open; once B is gone stale,
    // the close condition holds.
    RocketLogState a{}, b{};
    uint32_t t = 1000;
    updateRocketLogState(a, READY, t);
    updateRocketLogState(a, INFLIGHT, t + 500);       // A armed at t+500
    updateRocketLogState(b, READY, t + 1000);
    updateRocketLogState(b, INFLIGHT, t + 90000);     // B armed later

    const uint32_t now = t + 500 + SAFETY_MS + 1;     // A expired; B not
    RocketView v[2] = {
        {true, now - 1000, a.last_state, a.inflight_entry_ms, a.freq_lock},
        {true, now - 1000, b.last_state, b.inflight_entry_ms, b.freq_lock},
    };
    EXPECT_TRUE(anySafetyExpired(v, 2, now, SAFETY_MS));
    EXPECT_FALSE(noFreshRocketFlying(v, 2, now, FRESH_MS, SAFETY_MS))
        << "B is fresh, armed, and not expired — must hold the log open";

    // B goes silent past the freshness window: nothing fresh is flying.
    v[1].last_seen_ms = now - FRESH_MS - 1000;
    EXPECT_TRUE(noFreshRocketFlying(v, 2, now, FRESH_MS, SAFETY_MS));
}

TEST(BSLogPolicyMultiRocket, FreqLockStableAcrossInterleave)
{
    // A INFLIGHT latches the aggregate lock; B's READY stream must not clear
    // it (the old global flapped once per interleaved packet). A landing
    // clears A's lock, and with B READY (lock false) the aggregate drops.
    SimBS bs;
    uint32_t t = 1000;
    bs.packet(0, INFLIGHT, t);       t += 500;
    EXPECT_TRUE(bs.flightLock(t));
    for (int i = 0; i < 20; ++i) {
        bs.packet(1, READY, t);      t += 250;
        EXPECT_TRUE(bs.flightLock(t)) << "B's READY cleared A's in-flight lock";
        bs.packet(0, INFLIGHT, t);   t += 250;
    }
    bs.packet(0, LANDED, t);
    EXPECT_FALSE(bs.flightLock(t));
}

TEST(BSLogPolicyMultiRocket, BootEdgeAndSafetyArmSemantics)
{
    // First packet never edges (post-flight BS reboot seeing LANDED first
    // must not close/open anything)...
    RocketLogState s{};
    auto e = updateRocketLogState(s, LANDED, 1000);
    EXPECT_FALSE(e.landed_edge);
    EXPECT_FALSE(e.state_changed);

    // ...but a first packet already INFLIGHT arms the safety timer (BS
    // booted mid-flight; the log must still be bounded).
    RocketLogState s2{};
    updateRocketLogState(s2, INFLIGHT, 2000);
    EXPECT_EQ(s2.inflight_entry_ms, 2000u);

    // Leaving INFLIGHT disarms.
    updateRocketLogState(s2, LANDED, 3000);
    EXPECT_EQ(s2.inflight_entry_ms, 0u);
}

// ---------------------------------------------------------------------------
// Freq-lock expiry (#835 item 6)
//
// The bug was never in aggregateFreqLock — that predicate is pure, stateless
// and correct. It was in the SCHEDULE: main.cpp assigned its cached copy from
// exactly one place, the accepted-telemetry RX path, so when the packets
// stopped the last `true` froze until a power cycle and the freshness window
// was unreachable. A test of the predicate alone therefore cannot see this
// bug; these tests drive the cache (FreqLockLatch) instead.
//
// Two windows, because the consumers are not equally dangerous. The short one
// gates silence recovery and fixed-mode heartbeats. The long one gates the
// cmd-10 transaction, the only consumer that physically retunes the radio —
// releasing that at five minutes would let an operator retune off an airborne
// rocket's channel, and the relay is a broadcast whose transaction commits on
// any netid-matching packet, so a second rocket on the pad can strand the
// airborne one for the rest of its descent.
// ---------------------------------------------------------------------------

TEST(BSLogPolicyFreqLock, LockExpiresWhenTelemetryStops)
{
    // The headline regression. One INFLIGHT packet, then silence forever.
    // Pre-fix nothing recomputed the aggregate, so this stayed true until the
    // base station was power-cycled.
    SimBS bs;
    const uint32_t t0 = 1000;
    bs.packet(0, INFLIGHT, t0);
    EXPECT_TRUE(bs.flightLock(t0));

    // Sweep at 1 s steps right up to the window: still locked.
    for (uint32_t dt = 1000; dt <= FRESH_MS; dt += 1000) {
        bs.tick(t0 + dt);
        ASSERT_TRUE(bs.flightLock(t0 + dt)) << "dropped early at dt=" << dt;
    }

    bs.tick(t0 + FRESH_MS + 1);
    EXPECT_FALSE(bs.flightLock(t0 + FRESH_MS + 1))
        << "silence past the freshness window must release the flight lock";
}

TEST(BSLogPolicyFreqLock, RawCacheIsStickyButReadsExpire)
{
    // Documents the shape of the original defect AND the backstop. With no
    // sweep at all the cached bit stays true — that is the frozen value every
    // consumer read pre-#835 — but the read-side check still expires it, so a
    // future regression that deletes the loop_bs sweep degrades to a bounded
    // window instead of back to "locked until reboot".
    SimBS bs;
    const uint32_t t0 = 1000;
    bs.packet(0, INFLIGHT, t0);

    EXPECT_TRUE(bs.rawFlight()) << "the cached bit itself never decays";
    EXPECT_TRUE(bs.flightLock(t0 + FRESH_MS));
    EXPECT_FALSE(bs.flightLock(t0 + FRESH_MS + 1))
        << "read-side backstop must expire a cache nobody refreshed";
}

TEST(BSLogPolicyFreqLock, StampAdvancesOnEveryUpdateNotJustEdges)
{
    // The fail-DANGEROUS direction, and the reason update() must never sit
    // behind an edge guard. If last_eval_ms only advanced when the aggregate
    // CHANGED, it would freeze at the first INFLIGHT packet and the backstop
    // would expire the lock one window later mid-flight, however much
    // telemetry kept arriving. Silence-only tests cannot see that.
    SimBS bs;
    const uint32_t t0 = 1000;
    bs.packet(0, INFLIGHT, t0);

    // Continuous 2 Hz telemetry for three whole windows. The aggregate never
    // changes value across any of it.
    for (uint32_t dt = 500; dt <= 3 * FRESH_MS; dt += 500) {
        bs.packet(0, INFLIGHT, t0 + dt);
        ASSERT_TRUE(bs.flightLock(t0 + dt))
            << "lock expired while telemetry was still arriving, dt=" << dt;
        ASSERT_TRUE(bs.retuneLock(t0 + dt)) << "dt=" << dt;
    }
}

TEST(BSLogPolicyFreqLock, RetuneLockOutlivesFlightLock)
{
    // The two-window split. After the silence window the passive consumers
    // are released, but the radio-moving one stays shut for the whole
    // plausible flight.
    SimBS bs;
    const uint32_t t0 = 1000;
    bs.packet(0, INFLIGHT, t0);

    const uint32_t after_flight = t0 + FRESH_MS + 1;
    bs.tick(after_flight);
    EXPECT_FALSE(bs.flightLock(after_flight)) << "recovery/heartbeat released";
    EXPECT_TRUE(bs.retuneLock(after_flight))
        << "cmd-10 must stay refused while the rocket may still be descending";

    const uint32_t after_safety = t0 + SAFETY_MS + 1;
    bs.tick(after_safety);
    EXPECT_FALSE(bs.retuneLock(after_safety))
        << "the retune lock must clear itself too — never wedged until reboot";
}

TEST(BSLogPolicyFreqLock, LandedClearsBothImmediately)
{
    // A clean landing releases everything on the fold, not a window later.
    SimBS bs;
    uint32_t t = 1000;
    bs.packet(0, INFLIGHT, t);   t += 500;
    ASSERT_TRUE(bs.flightLock(t));
    ASSERT_TRUE(bs.retuneLock(t));
    bs.packet(0, LANDED, t);
    EXPECT_FALSE(bs.flightLock(t));
    EXPECT_FALSE(bs.retuneLock(t));
}

TEST(BSLogPolicyFreqLock, ReassertsWhenTheRocketComesBack)
{
    // Expiry is not a one-way door: the per-rocket latch survives the silence
    // (computeFreqLockForFlight clears only on READY/LANDED), so one packet
    // re-locks. The whole safety argument for expiring at all rests on this.
    SimBS bs;
    const uint32_t t0 = 1000;
    bs.packet(0, INFLIGHT, t0);

    const uint32_t gone = t0 + SAFETY_MS + 60000;
    bs.tick(gone);
    ASSERT_FALSE(bs.flightLock(gone));
    ASSERT_FALSE(bs.retuneLock(gone));

    bs.packet(0, INFLIGHT, gone + 1000);
    EXPECT_TRUE(bs.flightLock(gone + 1000));
    EXPECT_TRUE(bs.retuneLock(gone + 1000));
}

TEST(BSLogPolicyFreqLock, HoldsAcrossAShortGapInclusiveOfTheBoundary)
{
    // rocketFresh uses <=, so the boundary tick is still fresh. An off-by-one
    // here re-enables recovery hopping mid-descent.
    SimBS bs;
    const uint32_t t0 = 1000;
    bs.packet(0, INFLIGHT, t0);
    bs.tick(t0 + FRESH_MS);
    EXPECT_TRUE(bs.flightLock(t0 + FRESH_MS)) << "boundary must still be fresh";
}

TEST(BSLogPolicyFreqLock, SweepAloneNeverSetsTheLock)
{
    // Guards a mis-ordered argument list in the sweep. rocketFresh short-
    // circuits on r.active, so the zeroed last_seen_ms of an untouched slot
    // cannot read as fresh during the first minutes of uptime.
    SimBS bs;
    for (uint32_t t = 0; t < 2 * FRESH_MS; t += 10000) {
        bs.tick(t);
        ASSERT_FALSE(bs.flightLock(t)) << "t=" << t;
        ASSERT_FALSE(bs.retuneLock(t)) << "t=" << t;
    }
}

TEST(BSLogPolicyFreqLock, SurvivesMillisWraparound)
{
    // Unsigned subtraction throughout — both in rocketFresh and in the
    // read-side backstop.
    SimBS bs;
    const uint32_t t0 = 0xFFFFFF00u;  // ~1 minute before the wrap
    bs.packet(0, INFLIGHT, t0);

    const uint32_t mid = (uint32_t)(t0 + FRESH_MS / 2);   // wrapped
    bs.tick(mid);
    EXPECT_TRUE(bs.flightLock(mid)) << "wrap must not release the lock early";

    const uint32_t past = (uint32_t)(t0 + FRESH_MS + 1000);
    bs.tick(past);
    EXPECT_FALSE(bs.flightLock(past)) << "wrap must not make the lock permanent";
}

TEST(BSLogPolicyFreqLock, SecondFreshRocketHoldsTheLockForASilentOne)
{
    // The aggregate is across rockets: A silent past the window, B still
    // INFLIGHT and fresh, so the lock stays. Pins that the fix did not turn
    // the aggregate into a per-rocket check.
    SimBS bs;
    const uint32_t t0 = 1000;
    bs.packet(0, INFLIGHT, t0);
    bs.packet(1, INFLIGHT, t0);

    const uint32_t late = t0 + FRESH_MS + 1;
    bs.packet(1, INFLIGHT, late);   // only B keeps talking
    EXPECT_TRUE(bs.flightLock(late)) << "B is fresh and latched";

    bs.tick(late + FRESH_MS + 1);   // now both are stale
    EXPECT_FALSE(bs.flightLock(late + FRESH_MS + 1));
}

// ---------------------------------------------------------------------------
// #835 item 6 residuals. The expiry timer landed separately (FreqLockLatch);
// these cover the two things it does not: the clock it keys on, and the
// underlying per-rocket latch.
// ---------------------------------------------------------------------------
namespace {
bs_log_policy::RocketView lockedView(uint32_t telem_ms, uint32_t seen_ms)
{
    bs_log_policy::RocketView r;
    r.active        = true;
    r.freq_lock     = true;
    r.last_telem_ms = telem_ms;
    r.last_seen_ms  = seen_ms;
    return r;
}
constexpr uint32_t kWin = 5u * 60u * 1000u;
}  // namespace

TEST(BsFreqLockResiduals, BeaconAloneDoesNotHoldTheLock) {
    // The rocket is beaconing (last_seen_ms current) but its telemetry stopped
    // a window ago. Keying on last_seen_ms would hold the lock all session.
    const uint32_t now = 1000 + kWin + 1;
    bs_log_policy::RocketView v[1] = { lockedView(/*telem*/1000, /*seen*/now) };
    EXPECT_FALSE(bs_log_policy::aggregateFreqLock(v, 1, now, kWin))
        << "a name beacon must not hold the flight frequency lock";
}

TEST(BsFreqLockResiduals, LiveTelemetryStillHoldsIt) {
    const uint32_t now = 1000 + kWin;
    bs_log_policy::RocketView v[1] = { lockedView(now, now) };
    EXPECT_TRUE(bs_log_policy::aggregateFreqLock(v, 1, now, kWin));
}

TEST(BsFreqLockResiduals, ExpiredLatchIsReportedForClearing) {
    const uint32_t now = 1000 + kWin + 1;
    bs_log_policy::RocketView fresh = lockedView(now, now);
    bs_log_policy::RocketView stale = lockedView(1000, now);   // beaconing
    bs_log_policy::RocketView never = lockedView(1000, 1000);
    never.freq_lock = false;

    EXPECT_FALSE(bs_log_policy::freqLockExpired(fresh, now, kWin));
    EXPECT_TRUE (bs_log_policy::freqLockExpired(stale, now, kWin))
        << "a beaconing rocket's stale latch must still be clearable";
    EXPECT_FALSE(bs_log_policy::freqLockExpired(never, now, kWin))
        << "nothing to clear when the latch was never set";
}

TEST(BsFreqLockResiduals, ClearingTheLatchStopsTheRebootReArm) {
    // Model the sequence: latch goes stale, sweep clears it, then the rocket
    // reboots and its first packet arrives in INITIALIZATION — where
    // computeFreqLockForFlight leaves the lock UNCHANGED. With the latch
    // cleared there is no stale bit to re-latch from.
    const uint32_t now = 1000 + kWin + 1;
    bs_log_policy::RocketView v[1] = { lockedView(1000, 1000) };
    ASSERT_TRUE(bs_log_policy::freqLockExpired(v[0], now, kWin));
    v[0].freq_lock = false;                       // what the sweep does

    v[0].last_telem_ms = now;                     // first post-reboot packet
    v[0].last_seen_ms  = now;
    EXPECT_FALSE(bs_log_policy::aggregateFreqLock(v, 1, now, kWin))
        << "a rebooted rocket must not re-lock the BS from a stale latch";
}

TEST(BsFreqLockResiduals, TelemetryFreshnessSurvivesWraparound) {
    const uint32_t t = 0xFFFFF000u;
    bs_log_policy::RocketView v[1] = { lockedView(t, t) };
    EXPECT_TRUE (bs_log_policy::aggregateFreqLock(v, 1, (uint32_t)(t + 1000), kWin));
    EXPECT_FALSE(bs_log_policy::aggregateFreqLock(v, 1, (uint32_t)(t + kWin + 1), kWin));
}

TEST(BsFreqLockResiduals, BeaconingRocketReleasesTheLockThroughTheFullPath) {
    // End-to-end through the harness rather than hand-built views: one INFLIGHT
    // telemetry packet latches the lock, then the rocket only ever beacons.
    // Pre-residual the lock keyed on last_seen_ms, which beacon() advances, so
    // it never released no matter how long telemetry had been gone.
    SimBS bs;
    const uint32_t t0 = 1000;
    bs.packet(0, INFLIGHT, t0);
    EXPECT_TRUE(bs.flightLock(t0));

    // Beacons keep arriving every 30 s, well past the freshness window.
    for (uint32_t dt = 30000; dt <= FRESH_MS + 60000; dt += 30000) {
        bs.beacon(0, t0 + dt);
        bs.tick(t0 + dt);
    }
    const uint32_t now = t0 + FRESH_MS + 60000;
    EXPECT_FALSE(bs.flightLock(now))
        << "beacons kept last_seen_ms current, but telemetry stopped — the "
           "flight lock must still release";
}

TEST(BsFreqLockResiduals, BeaconsDoNotDelayReleaseVersusSilence) {
    // The beaconing rocket and the fully silent one must release at the same
    // moment: the only clock that matters here is telemetry.
    SimBS beaconing, silent;
    const uint32_t t0 = 1000;
    beaconing.packet(0, INFLIGHT, t0);
    silent.packet(0, INFLIGHT, t0);

    const uint32_t now = t0 + FRESH_MS + 1;
    beaconing.beacon(0, now);
    beaconing.tick(now);
    silent.tick(now);

    EXPECT_EQ(beaconing.flightLock(now), silent.flightLock(now));
    EXPECT_FALSE(beaconing.flightLock(now));
}

// ── #927 regression: the parser's suffix must track the writer's ──────────
//
// #927 switched the BS from writing CSV to writing binary and updated this
// parser's COMMENTS to say ".bin" — but not its SUFFIX constant. It then
// matched nothing on a post-#927 card, so findNextFileNumber() saw max_num = 0
// and returned 1 on every no-time-sync boot: each session overwrote
// lora_001.bin. Silent data loss, invisible to any test still feeding it the
// old CSV names — which is what these tests were doing.
TEST(BsLogPolicy, ParsesTheSuffixTheWriterActuallyEmits) {
    uint16_t n = 0;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_00007.bin", n));
    EXPECT_EQ(n, 7u);
    // The retired CSV name must NOT match, or a card holding both would let a
    // stale .csv set the counter above the real .bin high-water mark.
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_00007.csv", n));
}

// ── #897: fixed width, so strcmp order == numeric order ──────────────────
TEST(BsLogPolicy, FiveDigitNamesSortNumericallyByStrcmp) {
    // "%03u" is a MINIMUM width: lora_1000.bin is 4 chars of digits and
    // strcmp-sorts BELOW lora_999.bin, so the app's file list mis-ordered once
    // the counter passed 999.
    EXPECT_LT(std::string("lora_00999.bin"), std::string("lora_01000.bin"));
    EXPECT_LT(std::string("lora_00001.bin"), std::string("lora_00002.bin"));
    EXPECT_LT(std::string("lora_09999.bin"), std::string("lora_10000.bin"));
    // The defect, stated so the test says what it prevents.
    EXPECT_LT(std::string("lora_1000.bin"), std::string("lora_999.bin"));
}

TEST(BsLogPolicy, OlderThreeDigitNamesStillParse) {
    // Migration: a card written before the width change keeps contributing to
    // the high-water mark, so the counter resumes instead of restarting at 1.
    uint16_t n = 0;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_042.bin", n));
    EXPECT_EQ(n, 42u);
}

// ── #902: evict the OLDEST, including across the millis() wrap ────────────
namespace {
struct FakeRocket { bool active = true; uint32_t last_seen_ms = 0; };
}

TEST(BsLogPolicy, EvictsTheOldestSlotNormally) {
    FakeRocket r[4];
    r[0].last_seen_ms = 5000; r[1].last_seen_ms = 1000;
    r[2].last_seen_ms = 9000; r[3].last_seen_ms = 7000;
    EXPECT_EQ(bs_log_policy::evictOldestIndex(r, 4, 10000), 1);
}

// THE regression: the freshest rocket must not be evicted across the wrap.
TEST(BsLogPolicy, DoesNotEvictTheFreshestRocketAcrossTheWrap) {
    FakeRocket r[4];
    // now has just wrapped past 0. Slot 3 was heard 10 ms ago (post-wrap);
    // the others are pre-wrap and genuinely old.
    const uint32_t now = 10;
    r[0].last_seen_ms = 0xFFFFFF00u;   // ~4 s before the wrap
    r[1].last_seen_ms = 0xF0000000u;   // much older
    r[2].last_seen_ms = 0xFFFFFFF0u;   // ~0.3 s before the wrap
    r[3].last_seen_ms = 0u;            // 10 ms ago — the FRESHEST
    EXPECT_EQ(bs_log_policy::evictOldestIndex(r, 4, now), 1)
        << "a raw `<` on absolute millis picks slot 3, the freshest";
}

TEST(BsLogPolicy, EvictTiesGoToTheLowestIndex) {
    FakeRocket r[4];
    for (auto& x : r) x.last_seen_ms = 1234;
    EXPECT_EQ(bs_log_policy::evictOldestIndex(r, 4, 5000), 0);
}

// ── #901: a fleet reconfigure commits only when the fleet followed ────────
TEST(BsLogPolicy, ReconfigureCommitsOnlyWhenEveryTrackedRocketFollowed) {
    // Two rockets expected (slots 0 and 1); only slot 0 arrived on NEW.
    EXPECT_FALSE(bs_log_policy::reconfigureMayCommit(0b011, 0b001, 0, 0))
        << "the airborne rocket is stranded on OLD and the BS just committed NEW";
    EXPECT_TRUE(bs_log_policy::reconfigureMayCommit(0b011, 0b011, 0, 0));
}

TEST(BsLogPolicy, ReconfigureFallsBackToAnyPacketWithNoTrackedRockets) {
    // Bench / single-rocket case: nothing can be stranded, and demanding an
    // impossible quorum would make cmd-10 unusable.
    EXPECT_TRUE(bs_log_policy::reconfigureMayCommit(0, 0, /*last*/ 500, /*base*/ 400));
    EXPECT_FALSE(bs_log_policy::reconfigureMayCommit(0, 0, /*last*/ 300, /*base*/ 400));
}

TEST(BsLogPolicy, ReconfigureFallbackIsWrapSafe) {
    // last_packet just after the wrap, baseline just before it: a raw `>`
    // reads this as "no packet since" and refuses to commit forever.
    EXPECT_TRUE(bs_log_policy::reconfigureMayCommit(0, 0, /*last*/ 5,
                                                    /*base*/ 0xFFFFFFF0u));
}
