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
//   - a config that can put 3+ channels in Firing at once is flagged.

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

TEST(PyroFaultTripRearm, NeverReArmsWhileThePackStillReadsOverTheLimit) {
    State s;
    onOverLimit(s, 0, 32767, kFlight, Source::Poll);
    EXPECT_TRUE(stillHolding(s, kHoldOffMs, reading(35.0f)));
    EXPECT_TRUE(stillHolding(s, 10 * kHoldOffMs, reading(41.0f)));
    EXPECT_FALSE(stillHolding(s, 10 * kHoldOffMs + 1, reading(29.0f)));
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
