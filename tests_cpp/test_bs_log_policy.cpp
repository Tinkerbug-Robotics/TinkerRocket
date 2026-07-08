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

#include "bs_log_policy.h"

// ---------------------------------------------------------------------------
// parseSequentialFilename()
// ---------------------------------------------------------------------------

TEST(BSLogPolicyParseFilename, AcceptsValidSequentialNames)
{
    uint16_t n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_001.csv", n));
    EXPECT_EQ(n, 1u);

    n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_42.csv", n));
    EXPECT_EQ(n, 42u);

    n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_9886.csv", n));
    EXPECT_EQ(n, 9886u);

    n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_65535.csv", n));
    EXPECT_EQ(n, 65535u);

    n = 0xFFFF;
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_0.csv", n));
    EXPECT_EQ(n, 0u);
}

TEST(BSLogPolicyParseFilename, RejectsTimestampedNames)
{
    // Regression for the core #137 sscanf bug.  Pre-fix:
    //   sscanf("lora_20260509_164143.csv", "lora_%hu.csv", &n) == 1, n=9885
    // This made findNextFileNumber inflate max+1 to 9886, producing
    // `lora_9886.csv` on the next no-time-sync boot.
    uint16_t n = 0xFFFF;
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_20260509_164143.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_20260509_144144.csv", n));
    // _2 collision-suffix variant should also reject (it's still timestamped)
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora_20260509_164143_2.csv", n));
}

TEST(BSLogPolicyParseFilename, RejectsUnrelatedNames)
{
    uint16_t n = 0xFFFF;
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "flight_20260509_144128.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename(
        "lora.csv", n));               // no number
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
    // "lora_" and ".csv" disqualifies the match.
    uint16_t n = 0xFFFF;
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_-1.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_+1.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_ 1.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_1a.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_abc.csv", n));
}

TEST(BSLogPolicyParseFilename, RejectsOversized)
{
    // 6-digit run can't fit uint16 (max=65535) — and even values <=65535
    // that happen to have 6 digits ("099999" hypothetically) are rejected
    // because they'd have come from a sequence we never produce.  Real-
    // world BS code writes %03u, so anything >5 digits is malformed.
    uint16_t n = 0xFFFF;
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_99999.csv", n));
    EXPECT_FALSE(bs_log_policy::parseSequentialFilename("lora_123456.csv", n));
    // Boundary: 65535 is the highest uint16 (5 digits) — accepted.
    EXPECT_TRUE(bs_log_policy::parseSequentialFilename("lora_65535.csv", n));
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

constexpr uint32_t FRESH_MS  = 5 * 60 * 1000;   // config::LOG_SILENCE_TIMEOUT_MS
constexpr uint32_t SAFETY_MS = 20 * 60 * 1000;  // config::LOG_INFLIGHT_SAFETY_MS
constexpr int      NSLOTS    = 4;

// Minimal stand-in for the main.cpp wiring: per-slot state, the same
// open/close decisions serviceUplink's RX block makes, and open/close
// counters so a test can assert "exactly one file".
struct SimBS {
    RocketLogState slots[NSLOTS] = {};
    uint32_t       last_seen[NSLOTS] = {};
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
        }
    }

    // One received packet from rocket in `slot` reporting `state` at `now`.
    // Mirrors main.cpp: auto-open gate, per-rocket fold, aggregate close.
    void packet(int slot, uint8_t state, uint32_t now) {
        if (!logging && state != LANDED) { logging = true; ++opens; }

        slot_active[slot] = true;
        const auto edges = updateRocketLogState(slots[slot], state, now);
        last_seen[slot] = now;

        RocketView v[NSLOTS];
        views(v);
        if (edges.landed_edge && logging &&
            noFreshRocketFlying(v, NSLOTS, now, FRESH_MS, SAFETY_MS)) {
            logging = false;
            ++closes;
            // stopLogging() disarms all safety timers with the file.
            for (auto& s : slots) s.inflight_entry_ms = 0;
        }
    }

    bool freqLock(uint32_t now) {
        RocketView v[NSLOTS];
        views(v);
        return aggregateFreqLock(v, NSLOTS, now, FRESH_MS);
    }
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
    EXPECT_TRUE(bs.freqLock(t));
    for (int i = 0; i < 20; ++i) {
        bs.packet(1, READY, t);      t += 250;
        EXPECT_TRUE(bs.freqLock(t)) << "B's READY cleared A's in-flight lock";
        bs.packet(0, INFLIGHT, t);   t += 250;
    }
    bs.packet(0, LANDED, t);
    EXPECT_FALSE(bs.freqLock(t));
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
