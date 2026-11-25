// #1553: the OC drops arm consent when the pack current says an e-match or its
// harness shorted mid-pulse, then re-arms so a later channel can still fire.
//
// The load-bearing rows:
//   - a hard short (60-115 A, which clips the INA230 at 40.96 A) always trips,
//     and one or two channels firing together never do;
//   - the hold-off outlasts the FC's settle + pulse on the shorted channel,
//     wherever in it the trip landed, and is pinned to the FC's own config.h;
//   - after the hold-off consent comes back (the main still fires), unless the
//     pack still reads over the limit; a dead INA230 never holds it low;
//   - while consent is up SOL owns the INA230's alert pin; #1409's function
//     only gets it while consent is down;
//   - a config that can put 3+ channels in Firing at once is flagged;
//   - after a stuck reading's release the poll leaves the pin alone, so consent
//     is not chopped at the loop rate (#1757). OcPin below drives the pin the
//     way main.cpp's poll task, loop_oc and the INA_ALERT ISR do.

#include <gtest/gtest.h>

#include "arm_consent_policy.h"
#include "pyro_fault_trip_policy.h"

uint32_t fcPyroArmSettleMs();      // pyro_timing_fc.cpp
uint32_t fcPyroFireDurationMs();

using namespace PyroFaultTrip;

namespace {
constexpr uint8_t kFlight   = (uint8_t)ArmConsentPolicy::Reason::Flight;
constexpr uint8_t kFireTest = (uint8_t)ArmConsentPolicy::Reason::FireTest;

int16_t amps(float a)
{
    const float c = a * (float)kCountsPerAmp;
    return c >= 32767.0f ? (int16_t)32767 : (int16_t)c;   // the register clips
}

Reading reading(float a) { Reading r; r.valid = true; r.counts = amps(a); return r; }
Reading noReading() { return Reading{}; }

ChannelCfg ch(bool en, uint8_t mode, float v)
{
    ChannelCfg c; c.enabled = en; c.mode = mode; c.value = v; return c;
}
}  // namespace

// ---------------------------------------------------------------------------
// Threshold
// ---------------------------------------------------------------------------

TEST(PyroFaultTripThreshold, HardShortTripsEvenWhereTheRegisterClips) {
    EXPECT_TRUE(overLimit(amps(60.0f)));
    EXPECT_TRUE(overLimit(amps(115.0f)));
    EXPECT_TRUE(overLimit(32767));
    EXPECT_TRUE(overLimit(amps(30.0f)));
}

TEST(PyroFaultTripThreshold, OneOrTwoChannelsFiringNeverTrip) {
    const float board = kBoardDrawMilliA / 1000.0f;
    const float match = kChannelDrawMilliA / 1000.0f;
    EXPECT_FALSE(overLimit(amps(board + match)));
    EXPECT_FALSE(overLimit(amps(board + 2.0f * match)));
    EXPECT_FALSE(overLimit(amps(29.9f)));
    // Reverse current (charging, or a negative offset) is never a fault.
    EXPECT_FALSE(overLimit(-32768));
}

TEST(PyroFaultTripThreshold, ThreeChannelsCanTripWhichIsWhyTheWarningExists) {
    const float board = kBoardDrawMilliA / 1000.0f;
    const float match = kChannelDrawMilliA / 1000.0f;
    EXPECT_TRUE(overLimit(amps(board + 3.0f * match)));
    EXPECT_EQ(kMaxConcurrentChannels, 2);
}

TEST(PyroFaultTripThreshold, ScaleIsR72AtTheInaLsb) {
    EXPECT_EQ(kCountsPerAmp, 800);
    EXPECT_EQ(kTripLimitCounts, 24000);
    EXPECT_EQ(kSolAlertLimit, 24000u);
    EXPECT_EQ(countsToDeciAmps(24000), 300);
    EXPECT_EQ(countsToDeciAmps(32767), 409);   // the clip, 40.9 A
    EXPECT_EQ(countsToDeciAmps(0), 0);
}

// ---------------------------------------------------------------------------
// Timing, against the FC
// ---------------------------------------------------------------------------

TEST(PyroFaultTripTiming, MirrorsTheFlightComputersFireTiming) {
    EXPECT_EQ(kFcArmSettleMs, fcPyroArmSettleMs())
        << "flight_computer config.h PYRO_ARM_SETTLE_MS changed: update kFcArmSettleMs";
    EXPECT_EQ(kFcFireMs, fcPyroFireDurationMs())
        << "flight_computer config.h PYRO_FIRE_DURATION_MS changed: update kFcFireMs "
           "(the hold-off must outlast the pulse)";
}

TEST(PyroFaultTripTiming, HoldOffOutlastsTheWholePulseWhereverTheTripLands) {
    // The earliest a short can draw current is the start of the pulse (FIRE
    // high after the settle); the trip lands then at the earliest, so the rest
    // of the pulse is at most kFcFireMs. A trip during the settle itself
    // (a short that U9 alone closes) leaves settle + pulse.
    EXPECT_GT(kHoldOffMs, kFcArmSettleMs + kFcFireMs);
    EXPECT_GE(kHoldOffMs - (kFcArmSettleMs + kFcFireMs), 20u) << "margin for both loops";
}

// ---------------------------------------------------------------------------
// The trip
// ---------------------------------------------------------------------------

TEST(PyroFaultTrip, TripDropsConsentAndRecordsTheEvent) {
    State s;
    EXPECT_TRUE(pinHigh(true, s.holding));
    EXPECT_TRUE(onOverLimit(s, 1000, amps(40.0f), kFlight, Source::Poll));
    EXPECT_TRUE(s.holding);
    EXPECT_FALSE(pinHigh(true, s.holding));
    EXPECT_EQ(s.trips, 1u);
    EXPECT_EQ(s.last.at_ms, 1000u);
    EXPECT_EQ(s.last.reason, kFlight);
    EXPECT_EQ(s.last.source, (uint8_t)Source::Poll);
    EXPECT_EQ(s.last.peak_counts, amps(40.0f));
}

TEST(PyroFaultTrip, ConsentWantedLowStaysLowWhateverTheTrip) {
    State s;
    EXPECT_FALSE(pinHigh(false, s.holding));
    onOverLimit(s, 0, 32767, kFireTest, Source::Alert);
    EXPECT_FALSE(pinHigh(false, s.holding));
}

TEST(PyroFaultTrip, OverLimitWhileHoldingIsOneTripWithAHigherPeak) {
    State s;
    EXPECT_TRUE(onOverLimit(s, 1000, kTripLimitCounts, kFlight, Source::Alert));
    EXPECT_FALSE(onOverLimit(s, 1001, 32767, kFlight, Source::Poll));
    notePeak(s, amps(35.0f));   // lower: ignored
    EXPECT_EQ(s.trips, 1u);
    EXPECT_EQ(s.current.peak_counts, 32767);
    EXPECT_EQ(s.current.source, (uint8_t)Source::Alert) << "the first sighting names the source";
    EXPECT_EQ(s.current.at_ms, 1000u) << "the hold-off runs from the first sighting";
}

TEST(PyroFaultTrip, NotePeakRaisesTheLastEventToo) {
    State s;
    onOverLimit(s, 0, kTripLimitCounts, kFlight, Source::Alert);
    notePeak(s, 32000);
    EXPECT_EQ(s.last.peak_counts, 32000);
    EXPECT_EQ(s.current.peak_counts, 32000);
}

TEST(PyroFaultTrip, NotePeakOutsideATripChangesNothing) {
    State s;
    notePeak(s, 32767);
    EXPECT_FALSE(s.holding);
    EXPECT_EQ(s.trips, 0u);
    EXPECT_EQ(s.last.peak_counts, 0);
}

// ---------------------------------------------------------------------------
// Hold-off and re-arm
// ---------------------------------------------------------------------------

TEST(PyroFaultTripRearm, HeldForTheHoldOffThenReleased) {
    State s;
    onOverLimit(s, 5000, 32767, kFlight, Source::Poll);
    EXPECT_TRUE(stillHolding(s, 5000, reading(0.1f)));
    EXPECT_TRUE(stillHolding(s, 5000 + kHoldOffMs - 1, reading(0.1f)));
    EXPECT_FALSE(stillHolding(s, 5000 + kHoldOffMs, reading(0.1f)));
    EXPECT_FALSE(s.holding);
    EXPECT_TRUE(pinHigh(true, s.holding)) << "the main must still be able to fire";
}

TEST(PyroFaultTripRearm, DoesNotReArmWhileThePackStillReadsOverTheLimit) {
    State s;
    onOverLimit(s, 0, 32767, kFlight, Source::Poll);
    EXPECT_TRUE(stillHolding(s, kHoldOffMs, reading(35.0f)));
    EXPECT_TRUE(stillHolding(s, kMaxHoldMs - 1, reading(41.0f)));
    EXPECT_FALSE(stillHolding(s, kMaxHoldMs - 1, reading(29.0f)));
    EXPECT_FALSE(s.implausible) << "it came back under the limit: a real trip";
}

// V10 pre-flight review (2026-11-18): past the hold-off U9 is open, so a
// reading still over the limit is not the fire loop (a bad shunt, its traces,
// the INA230). It used to hold consent low for the rest of the flight and veto
// every later deployment; now it lets go at kMaxHoldMs and the poll stops
// tripping on it until a reading under the limit.
TEST(PyroFaultTripRearm, AReadingStuckOverTheLimitLetsGoAtTheMaximumHold) {
    State s;
    ASSERT_TRUE(onOverLimit(s, 0, amps(41.0f), kFlight, Source::Poll));
    EXPECT_TRUE(stillHolding(s, kMaxHoldMs - 1, reading(41.0f)));
    EXPECT_FALSE(stillHolding(s, kMaxHoldMs, reading(41.0f)));
    EXPECT_TRUE(s.implausible);
    EXPECT_FALSE(onOverLimit(s, kMaxHoldMs + 1, amps(41.0f), kFlight, Source::Poll))
        << "the poll does not re-trip on the stuck reading";
    EXPECT_FALSE(stillHolding(s, kMaxHoldMs + 2, reading(41.0f)));
    EXPECT_TRUE(pinHigh(true, s.holding)) << "consent follows the arm rule again";
    EXPECT_EQ(s.trips, 1u);

    noteUnderLimit(s);   // the measurement is the pack's again
    EXPECT_FALSE(s.implausible);
    EXPECT_TRUE(onOverLimit(s, kMaxHoldMs + 10, amps(41.0f), kFlight, Source::Poll))
        << "a real over-limit after that trips as before";
}

TEST(PyroFaultTripRearm, AnAlertEdgeStillTripsWhileThePollIsSuspended) {
    State s;
    onOverLimit(s, 0, amps(41.0f), kFlight, Source::Poll);
    stillHolding(s, kMaxHoldMs, reading(41.0f));
    ASSERT_TRUE(s.implausible);
    EXPECT_TRUE(onOverLimit(s, kMaxHoldMs + 5, kTripLimitCounts, kFlight, Source::Alert));
}

TEST(PyroFaultTripRearm, ADeadInaNeverHoldsConsentLow) {
    State s;
    onOverLimit(s, 0, 32767, kFlight, Source::Poll);
    EXPECT_TRUE(stillHolding(s, kHoldOffMs - 1, noReading())) << "the hold-off itself still runs";
    EXPECT_FALSE(stillHolding(s, kHoldOffMs, noReading()));
}

TEST(PyroFaultTripRearm, WrapSafeAcrossTheMillisRollover) {
    State s;
    const uint32_t t0 = 0xFFFFFFFFu - 100u;
    onOverLimit(s, t0, 32767, kFlight, Source::Poll);
    EXPECT_TRUE(stillHolding(s, t0 + 100u, reading(0.0f)));        // wrapped to ~0
    EXPECT_FALSE(stillHolding(s, t0 + kHoldOffMs, reading(0.0f)));
}

TEST(PyroFaultTripRearm, ReleasedTripIsNotResurrectedByTime) {
    State s;
    onOverLimit(s, 0, 32767, kFlight, Source::Poll);
    EXPECT_FALSE(stillHolding(s, kHoldOffMs, reading(0.0f)));
    // 2^31 ms later a stale deadline compared by signed difference would read
    // as live again; `holding` is cleared, so it cannot.
    EXPECT_FALSE(stillHolding(s, 0x80000000u, reading(0.0f)));
}

TEST(PyroFaultTripRearm, ASecondShortTripsAgainAndCounts) {
    // Drogue harness shorted, then the main's: both trip, both are logged,
    // and consent comes back after each.
    State s;
    EXPECT_TRUE(onOverLimit(s, 1000, 32767, kFlight, Source::Poll));
    EXPECT_FALSE(stillHolding(s, 1000 + kHoldOffMs, reading(0.0f)));
    EXPECT_TRUE(onOverLimit(s, 60000, 32767, kFlight, Source::Poll));
    EXPECT_EQ(s.trips, 2u);
    EXPECT_EQ(s.last.at_ms, 60000u);
    EXPECT_FALSE(stillHolding(s, 60000 + kHoldOffMs, reading(0.0f)));
}

TEST(PyroFaultTripRearm, TripCountSaturates) {
    State s;
    s.trips = 0xFFFF;
    onOverLimit(s, 0, 32767, kFlight, Source::Poll);
    EXPECT_EQ(s.trips, 0xFFFFu);
}

// ---------------------------------------------------------------------------
// The poll's side of the pin (#1757)
// ---------------------------------------------------------------------------

namespace {
// A trip released at kMaxHoldMs on a reading still over the limit.
State releasedImplausible()
{
    State s;
    onOverLimit(s, 0, 32767, kFlight, Source::Poll);
    stillHolding(s, kMaxHoldMs, reading(41.0f));
    return s;
}
}  // namespace

TEST(PyroFaultTripPoll, AnOverLimitReadingTripsAndDropsThePin) {
    State s; Reading latest; bool edge = false;
    const OverLimit r = pollShunt(s, latest, edge, true, amps(41.0f), 1000, kFlight);
    EXPECT_TRUE(r.started);
    EXPECT_TRUE(r.drop_pin);
    EXPECT_TRUE(s.holding);
    EXPECT_EQ(s.last.source, (uint8_t)Source::Poll);
}

TEST(PyroFaultTripPoll, WhileATripHoldsThePinStaysDropped) {
    State s; Reading latest; bool edge = false;
    pollShunt(s, latest, edge, true, 32767, 1000, kFlight);
    const OverLimit r = pollShunt(s, latest, edge, true, 32767, 1001, kFlight);
    EXPECT_FALSE(r.started) << "one trip";
    EXPECT_TRUE(r.drop_pin);
}

TEST(PyroFaultTripPoll, AnImplausibleReadingLeavesThePinAlone) {
    State s = releasedImplausible();
    ASSERT_TRUE(s.implausible);
    ASSERT_FALSE(s.holding);
    Reading latest; bool edge = false;
    const OverLimit r = pollShunt(s, latest, edge, true, 32767, kMaxHoldMs + 1, kFlight);
    EXPECT_FALSE(r.started);
    EXPECT_FALSE(r.drop_pin) << "nothing holds the pin, so dropping it only chops consent";
    EXPECT_FALSE(s.holding);
    EXPECT_EQ(s.trips, 1u);
}

TEST(PyroFaultTripPoll, OverLimitSeenDropsExactlyWhileATripHolds) {
    // Every state the poll or an edge can meet: the pin drops iff a trip holds after it.
    for (int implausible = 0; implausible < 2; ++implausible)
        for (int holding = 0; holding < 2; ++holding)
            for (Source src : {Source::Poll, Source::Alert})
            {
                State s;
                s.implausible = implausible != 0;
                if (holding) { s.holding = true; s.current.at_ms = 0; }
                const OverLimit r = overLimitSeen(s, 10, 32767, kFlight, src);
                EXPECT_EQ(r.drop_pin, s.holding);
                EXPECT_EQ(r.drop_pin, holding || !(implausible && src == Source::Poll))
                    << "implausible=" << implausible << " holding=" << holding
                    << " src=" << (int)src;
                EXPECT_EQ(r.started, !holding && !(implausible && src == Source::Poll));
            }
}

TEST(PyroFaultTripPoll, AnAlertEdgeTripsEvenOnAnImplausibleReading) {
    State s = releasedImplausible();
    Reading latest; bool edge = true;
    const OverLimit r = pollShunt(s, latest, edge, true, 32767, kMaxHoldMs + 5, kFlight);
    EXPECT_FALSE(edge) << "the edge is consumed";
    EXPECT_TRUE(r.started);
    EXPECT_TRUE(r.drop_pin);
    EXPECT_EQ(s.current.source, (uint8_t)Source::Alert);
    EXPECT_EQ(s.trips, 2u);
}

TEST(PyroFaultTripPoll, AnAlertEdgeRecordsAtLeastTheLimit) {
    State s; Reading latest; bool edge = true;
    const OverLimit r = pollShunt(s, latest, edge, true, amps(8.0f), 1000, kFlight);
    EXPECT_TRUE(r.started);
    EXPECT_TRUE(r.drop_pin);
    EXPECT_EQ(s.current.peak_counts, kTripLimitCounts);
    EXPECT_FALSE(s.implausible);
}

TEST(PyroFaultTripPoll, AFailedReadTripsNothingAndIsStoredAsMissing) {
    State s; Reading latest; bool edge = false;
    const OverLimit r = pollShunt(s, latest, edge, false, 32767, 1000, kFlight);
    EXPECT_FALSE(r.started);
    EXPECT_FALSE(r.drop_pin);
    EXPECT_FALSE(latest.valid);
    EXPECT_EQ(latest.counts, 0);
}

TEST(PyroFaultTripPoll, TheReadingIsStoredForTheReArm) {
    State s; Reading latest; bool edge = false;
    pollShunt(s, latest, edge, true, amps(3.0f), 1000, kFlight);
    EXPECT_TRUE(latest.valid);
    EXPECT_EQ(latest.counts, amps(3.0f));
}

TEST(PyroFaultTripPoll, AReadingUnderTheLimitMakesThePollTripAgain) {
    State s = releasedImplausible();
    Reading latest; bool edge = false;
    pollShunt(s, latest, edge, true, amps(2.0f), kMaxHoldMs + 1, kFlight);
    EXPECT_FALSE(s.implausible);
    const OverLimit r = pollShunt(s, latest, edge, true, 32767, kMaxHoldMs + 2, kFlight);
    EXPECT_TRUE(r.started);
    EXPECT_TRUE(r.drop_pin);
}

// ---------------------------------------------------------------------------
// The pin through main.cpp's tasks (#1757)
// ---------------------------------------------------------------------------
//
// main.cpp's three writers of OC_ARM_EN, in its order: the poll task every
// 1 ms tick (pollShunt, then pyroFaultApplyLocked's drop), loop_oc every pass
// (servicePyroFaultTrip's stillHolding, then armConsentApplyPin's pinHigh),
// and on the V10 board the INA_ALERT ISR (pin low, then the edge flag).

namespace {
struct OcPin
{
    State   s;
    Reading latest;
    bool    edge  = false;
    bool    level = false;
    int     drops = 0, raises = 0;
    bool    old_wiring = false;   // main.cpp before #1757: the poll dropped the pin first

    void drop()
    {
        if (level) { level = false; drops++; }
    }
    void poll(bool ok, int16_t raw, uint32_t now)
    {
        if (old_wiring)
        {
            latest.valid  = ok;
            latest.counts = ok ? raw : 0;
            if (edge)
            {
                edge = false;
                drop();
                onOverLimit(s, now, (ok && raw > kTripLimitCounts) ? raw : kTripLimitCounts,
                            kFlight, Source::Alert);
            }
            if (ok && overLimit(raw)) { drop(); onOverLimit(s, now, raw, kFlight, Source::Poll); }
            else if (ok) { notePeak(s, raw); noteUnderLimit(s); }
            return;
        }
        const OverLimit r = pollShunt(s, latest, edge, ok, raw, now, kFlight);
        if (r.drop_pin) drop();
    }
    void loopPass(bool wanted, uint32_t now)
    {
        if (s.holding) stillHolding(s, now, latest);
        const bool high = pinHigh(wanted, s.holding || edge);
        if (high != level)
        {
            level = high;
            if (high) raises++; else drops++;
        }
    }
    void alertEdge()
    {
        drop();
        edge = true;
    }
};

// What the pin did over a run.
struct PinRun
{
    int      drops_after_release  = 0;   // pin drops once the first trip let go
    int      low_ms_after_release = 0;   // ticks the pin sat low after that
    int      high_ms_while_held   = 0;   // ticks the pin sat high while a trip held
    uint32_t released_at          = 0;   // first release; 0 = never
};

// 1 ms ticks from 0 to end_ms with consent wanted throughout. `counts(t)`
// is the INA230's reading at t (INT16_MIN: the read fails); `edges` are the
// ticks an INA_ALERT edge lands on. The loop runs every loop_ms, at `phase`
// in it, before or after the poll on a shared tick.
template <class Counts>
PinRun drive(OcPin& oc, uint32_t end_ms, uint32_t loop_ms, uint32_t phase, bool loop_first,
        Counts counts, std::initializer_list<uint32_t> edges = {})
{
    PinRun out;
    for (uint32_t t = 0; t <= end_ms; ++t)
    {
        for (uint32_t e : edges)
            if (e == t) oc.alertEdge();
        const int32_t c   = counts(t);
        const bool    ok  = c != INT16_MIN;
        const bool    due = t % loop_ms == phase;
        const bool    was_holding = oc.s.holding;
        const int     drops_before = oc.drops;
        if (due && loop_first) oc.loopPass(true, t);
        oc.poll(ok, ok ? (int16_t)c : 0, t);
        if (due && !loop_first) oc.loopPass(true, t);
        if (was_holding && !oc.s.holding && out.released_at == 0) out.released_at = t;
        if (out.released_at != 0 && t > out.released_at)
        {
            out.drops_after_release += oc.drops - drops_before;
            // a loop pass due on the release tick may come before it: give it one period
            if (!oc.level && t > out.released_at + loop_ms) out.low_ms_after_release++;
        }
        if (oc.s.holding && oc.level) out.high_ms_while_held++;
    }
    return out;
}
}  // namespace

TEST(PyroFaultTripPin, AStuckReadingNoLongerChopsConsent) {
    // A failed INA230 or an open sense joint reads 41 A from t = 1 s on, and
    // keeps reading it. One trip, held to kMaxHoldMs, then consent stays up.
    for (uint32_t loop_ms : {1u, 2u, 5u, 10u})
        for (uint32_t phase = 0; phase < loop_ms; ++phase)
            for (bool loop_first : {true, false})
            {
                OcPin oc;
                const PinRun r = drive(oc, 4000, loop_ms, phase, loop_first,
                                  [](uint32_t t) { return t >= 1000 ? 32767 : amps(0.5f); });
                SCOPED_TRACE(::testing::Message() << "loop " << loop_ms << " ms, phase " << phase
                                                  << (loop_first ? ", loop first" : ", poll first"));
                EXPECT_EQ(oc.s.trips, 1u);
                EXPECT_TRUE(oc.s.implausible);
                EXPECT_GE(r.released_at, 1000u + kMaxHoldMs);
                EXPECT_LT(r.released_at, 1000u + kMaxHoldMs + loop_ms);
                EXPECT_EQ(r.high_ms_while_held, 0) << "the trip held the pin low";
                EXPECT_EQ(r.drops_after_release, 0) << "consent chopped after the release";
                EXPECT_EQ(r.low_ms_after_release, 0);
                EXPECT_EQ(oc.drops, 1);
                EXPECT_EQ(oc.raises, 2) << "up at boot, and back up at the release";
                EXPECT_TRUE(oc.level);
            }
}

TEST(PyroFaultTripPin, BeforeThisFixTheStuckReadingChoppedConsent) {
    // The model reproduces #1757 with main.cpp's old order: drop, then ask.
    OcPin oc;
    oc.old_wiring = true;
    const PinRun r = drive(oc, 4000, 1, 0, true,
                      [](uint32_t t) { return t >= 1000 ? 32767 : amps(0.5f); });
    EXPECT_EQ(oc.s.trips, 1u);
    EXPECT_GT(r.drops_after_release, 2000) << "one drop per poll from the release on";
}

TEST(PyroFaultTripPin, ARealShortStillTripsHoldsAndReArms) {
    // The drogue's harness shorts for its 150 ms of pulse left, then the main's
    // fires clean at t = 3 s, then the main's harness shorts at t = 5 s.
    for (uint32_t loop_ms : {1u, 5u})
        for (bool loop_first : {true, false})
        {
            OcPin oc;
            const PinRun r = drive(oc, 6000, loop_ms, 0, loop_first, [](uint32_t t) -> int32_t {
                if (t >= 1000 && t < 1150) return 32767;
                if (t >= 3000 && t < 3200) return amps(8.0f);
                if (t >= 5000 && t < 5100) return 32767;
                return amps(0.5f);
            });
            SCOPED_TRACE(::testing::Message() << "loop " << loop_ms << " ms"
                                              << (loop_first ? ", loop first" : ", poll first"));
            EXPECT_EQ(oc.s.trips, 2u);
            EXPECT_FALSE(oc.s.implausible);
            EXPECT_GE(r.released_at, 1000u + kHoldOffMs);
            EXPECT_LT(r.released_at, 1000u + kHoldOffMs + loop_ms);
            EXPECT_EQ(r.high_ms_while_held, 0);
            EXPECT_EQ(oc.drops, 2) << "one per short";
            EXPECT_TRUE(oc.level);
        }
}

TEST(PyroFaultTripPin, AStuckReadingThatClearsThenOverAgainTripsAgain) {
    // Stuck for 2 s, under the limit for 50 ms, then over again: the poll trusts
    // the measurement again and trips once more, then lets go at kMaxHoldMs.
    OcPin oc;
    drive(oc, 6000, 2, 1, true, [](uint32_t t) -> int32_t {
        if (t >= 1000 && t < 3000) return 32767;
        if (t >= 3000 && t < 3050) return amps(1.0f);
        if (t >= 3050) return 32767;
        return amps(0.5f);
    });
    EXPECT_EQ(oc.s.trips, 2u);
    EXPECT_EQ(oc.drops, 2);
    EXPECT_TRUE(oc.level);
}

TEST(PyroFaultTripPin, OnTheV10AnAlertEdgeOnAStuckReadingHoldsThenLetsGo) {
    // The INA230's alert is transparent: a reading that stays over the limit
    // keeps ALERT low and makes one edge. A later edge (a conversion under the
    // limit the poll never saw, then over again) trips again.
    OcPin oc;
    const PinRun r = drive(oc, 5000, 1, 0, true,
                      [](uint32_t t) { return t >= 1000 ? 32767 : amps(0.5f); },
                      {1000u, 3000u});
    EXPECT_EQ(oc.s.trips, 2u);
    EXPECT_EQ(oc.s.last.source, (uint8_t)Source::Alert);
    EXPECT_EQ(r.high_ms_while_held, 0);
    EXPECT_GE(r.released_at, 1000u + kMaxHoldMs);
    EXPECT_EQ(oc.drops, 2) << "one per edge, none from the poll";
    EXPECT_TRUE(oc.level) << "the second trip let go at kMaxHoldMs too";
}

TEST(PyroFaultTripPin, AFailedInaNeverTouchesThePin) {
    OcPin oc;
    drive(oc, 3000, 1, 0, false, [](uint32_t) { return (int32_t)INT16_MIN; });
    EXPECT_EQ(oc.drops, 0);
    EXPECT_EQ(oc.s.trips, 0u);
    EXPECT_TRUE(oc.level);
}

// ---------------------------------------------------------------------------
// INA230 configuration
// ---------------------------------------------------------------------------

TEST(PyroFaultTripIna, GuardWheneverConsentIsUpWhateverTheRail) {
    EXPECT_EQ(inaConfig(true, true), InaConfig::Guard);
    EXPECT_EQ(inaConfig(true, false), InaConfig::Guard);
    EXPECT_EQ(inaConfig(false, true), InaConfig::RailOn);
    EXPECT_EQ(inaConfig(false, false), InaConfig::LowPower);
}

TEST(PyroFaultTripIna, ConfigWordsMatchTheDatasheetFields) {
    // AVG_1, 140 us bus, 140 us shunt, shunt+bus continuous.
    EXPECT_EQ(configWord(InaConfig::Guard), 0x0007u);
    // AVG_1, 332 us, 332 us, continuous: what ina230ConfigureForRailOn wrote.
    EXPECT_EQ(configWord(InaConfig::RailOn), 0x0097u);
    // AVG_1024, 332 us, 332 us, continuous: the low-power branch.
    EXPECT_EQ(configWord(InaConfig::LowPower), 0x0E97u);
}

// ---------------------------------------------------------------------------
// The alert pin, shared with #1409
// ---------------------------------------------------------------------------

TEST(PyroFaultTripAlert, SolOwnsThePinWhileConsentIsUp) {
    EXPECT_EQ(alertOwner(true, false), AlertOwner::Sol);
    EXPECT_EQ(alertOwner(true, true), AlertOwner::Sol) << "#1409's brownout waits for consent down";
}

TEST(PyroFaultTripAlert, BrownoutGetsThePinOnlyWhileConsentIsDown) {
    EXPECT_EQ(alertOwner(false, true), AlertOwner::Brownout);
    EXPECT_EQ(alertOwner(false, false), AlertOwner::None);
}

TEST(PyroFaultTripAlert, ConversionReadyStaysOffAWiredAlertPin) {
    // With CNVR on, ALERT asserts once per conversion: on a wired pin that is
    // a trip every 280 us.
    EXPECT_EQ(solMaskEnable(true), kMeSol);
    EXPECT_EQ(solMaskEnable(true) & kMeCnvr, 0u);
    // Unwired (the Beetle): keep the CNVR bit the rail-on path has always run with.
    EXPECT_EQ(solMaskEnable(false), (uint16_t)(kMeSol | kMeCnvr));
    // Transparent, active low: LEN (bit 0) and APOL (bit 1) clear either way.
    EXPECT_EQ(solMaskEnable(true) & 0x3u, 0u);
    EXPECT_EQ(solMaskEnable(false) & 0x3u, 0u);
}

// ---------------------------------------------------------------------------
// The concurrency warning
// ---------------------------------------------------------------------------

TEST(PyroFaultTripConcurrency, NothingEnabledIsZero) {
    const ChannelCfg c[4] = {};
    EXPECT_EQ(maxConcurrentChannels(c), 0u);
}

TEST(PyroFaultTripConcurrency, DrogueAndMainAreOne) {
    const ChannelCfg c[4] = { ch(true, kModeTime, 1.0f), ch(true, kModeAltitude, 150.0f),
                              ch(false, 0, 0.0f), ch(false, 0, 0.0f) };
    EXPECT_EQ(maxConcurrentChannels(c), 1u);
    EXPECT_FALSE(concurrencyWarning(maxConcurrentChannels(c)));
}

TEST(PyroFaultTripConcurrency, RedundantPairIsTwoAndCleared) {
    const ChannelCfg c[4] = { ch(true, kModeTime, 1.0f), ch(true, kModeTime, 1.0f),
                              ch(true, kModeAltitude, 150.0f), ch(true, kModeAltitude, 150.0f) };
    EXPECT_EQ(maxConcurrentChannels(c), 2u);
    EXPECT_FALSE(concurrencyWarning(maxConcurrentChannels(c)));
}

TEST(PyroFaultTripConcurrency, ThreeOnOneTriggerIsWarned) {
    const ChannelCfg c[4] = { ch(true, kModeTime, 2.0f), ch(true, kModeTime, 2.0f),
                              ch(true, kModeTime, 2.0f), ch(true, kModeAltitude, 150.0f) };
    EXPECT_EQ(maxConcurrentChannels(c), 3u);
    EXPECT_TRUE(concurrencyWarning(maxConcurrentChannels(c)));
}

TEST(PyroFaultTripConcurrency, AllFourOnOneAltitudeIsFour) {
    const ChannelCfg c[4] = { ch(true, kModeAltitude, 200.0f), ch(true, kModeAltitude, 200.0f),
                              ch(true, kModeAltitude, 200.0f), ch(true, kModeAltitude, 200.0f) };
    EXPECT_EQ(maxConcurrentChannels(c), 4u);
}

TEST(PyroFaultTripConcurrency, NearbyTimesOverlapButSpreadOnesDoNot) {
    // Starts 0.1 s apart: the first pulse is still on when the next begins.
    const ChannelCfg near[4] = { ch(true, kModeTime, 1.0f), ch(true, kModeTime, 1.1f),
                                 ch(true, kModeTime, 1.2f), ch(false, 0, 0.0f) };
    EXPECT_EQ(maxConcurrentChannels(near), 3u);
    // 0.25 s apart: each pulse (0.21 s) is over before the next starts.
    const ChannelCfg spread[4] = { ch(true, kModeTime, 1.0f), ch(true, kModeTime, 1.25f),
                                   ch(true, kModeTime, 1.5f), ch(false, 0, 0.0f) };
    EXPECT_EQ(maxConcurrentChannels(spread), 1u);
}

TEST(PyroFaultTripConcurrency, WindowIsCountedFromEachStartNotPairwiseChained) {
    // 1.0, 1.2, 1.4: neighbours overlap, but 1.0 and 1.4 do not, so at most
    // two pulses are on at once.
    const ChannelCfg c[4] = { ch(true, kModeTime, 1.0f), ch(true, kModeTime, 1.2f),
                              ch(true, kModeTime, 1.4f), ch(false, 0, 0.0f) };
    EXPECT_EQ(maxConcurrentChannels(c), 2u);
}

TEST(PyroFaultTripConcurrency, MixedModesAreNotCounted) {
    const ChannelCfg c[4] = { ch(true, kModeTime, 1.0f), ch(true, kModeTime, 1.0f),
                              ch(true, kModeAltitude, 1.0f), ch(false, 0, 0.0f) };
    EXPECT_EQ(maxConcurrentChannels(c), 2u);
}

TEST(PyroFaultTripConcurrency, DisabledChannelsAreNotCounted) {
    const ChannelCfg c[4] = { ch(true, kModeTime, 1.0f), ch(false, kModeTime, 1.0f),
                              ch(false, kModeTime, 1.0f), ch(true, kModeTime, 1.0f) };
    EXPECT_EQ(maxConcurrentChannels(c), 2u);
}
