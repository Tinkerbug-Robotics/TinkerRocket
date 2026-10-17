// Host tests for RawVoteGuard (mosaic-only build): after burnout and before
// apogee, a raw-GNSS filter whose vertical velocity reads more than 10 m/s
// below the barometer's climb rate for a continuous second loses its velocity
// and pitch apogee votes for the rest of the flight.  The guard is the only
// cross-check left when the raw filter is the one navigation filter, so these
// pin both halves: it latches on a real disagreement, and it never acts on a
// barometer that cannot vouch (unhealthy, flat, fast), on a filter reading
// high, before burnout or after apogee, and never lets the filter back.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "raw_vote_guard.h"

using namespace RawVoteGuard;

namespace {

// One step per millisecond, the kinematics pass at ~1 kHz.  The defaults are
// a coasting rocket the guard can judge, with the two rates agreeing.
struct Sim {
    State  st;
    Inputs in;
    Config cfg;

    Sim()
    {
        in.now_ms           = 100000;
        in.after_burnout    = true;
        in.apogee           = false;
        in.baro_healthy     = true;
        in.mach_locked      = false;
        in.filter_healthy   = true;
        in.filter_vz_up_mps = 20.0f;
        in.baro_rate_mps    = 20.0f;
    }

    // Advance `ms` passes; returns step()'s answer on the last one.
    bool run(uint32_t ms)
    {
        bool withheld = st.latched;
        for (uint32_t i = 0; i < ms; ++i)
        {
            in.now_ms += 1;
            withheld = step(st, in, cfg);
        }
        return withheld;
    }

    // The filter reads `diff` m/s above the barometer (below when negative).
    void disagreeBy(float diff)
    {
        in.filter_vz_up_mps = in.baro_rate_mps + diff;
    }
};

}  // namespace

// ---------------------------------------------------------------------------
// The defaults are the plan's numbers
// ---------------------------------------------------------------------------
TEST(RawVoteGuard, DefaultsAreTenMpsForOneSecondBetweenOneAndSixtyMps)
{
    const Config cfg;
    EXPECT_FLOAT_EQ(cfg.diff_mps, 10.0f);
    EXPECT_EQ(cfg.hold_ms, 1000u);
    EXPECT_FLOAT_EQ(cfg.rate_max_mps, 60.0f);
    EXPECT_FLOAT_EQ(cfg.rate_flat_mps, 1.0f);

    const State st;
    EXPECT_FALSE(st.latched);
    EXPECT_FALSE(st.disagreeing);

    const Inputs in;
    EXPECT_FALSE(in.apogee);
}

TEST(RawVoteGuard, DefaultInputsCannotLatch)
{
    // Every gate defaults shut, so a caller that forgets a field never takes
    // the filter's votes away.
    State st;
    Inputs in;
    in.filter_vz_up_mps = -30.0f;
    in.baro_rate_mps    = 30.0f;
    for (uint32_t t = 1; t <= 5000; ++t)
    {
        in.now_ms = t;
        EXPECT_FALSE(step(st, in));
    }
    EXPECT_FALSE(st.latched);
}

// ---------------------------------------------------------------------------
// Latching: one-sided, on a filter reading below the barometer
// ---------------------------------------------------------------------------
TEST(RawVoteGuard, LatchesAfterOneSecondOfA12MpsDisagreement)
{
    Sim s;
    s.disagreeBy(-12.0f);   // the filter reads a descent the barometer does not
    // The run starts on the first disagreeing pass; 1000 passes later it has
    // been held 999 ms.
    EXPECT_FALSE(s.run(1000));
    EXPECT_FALSE(s.st.latched);
    EXPECT_TRUE(s.st.disagreeing);
    EXPECT_TRUE(s.run(1));
    EXPECT_TRUE(s.st.latched);
}

TEST(RawVoteGuard, NeverLatchesOnAFilterReadingHigh)
{
    // A filter that reads high only holds back its own velocity vote, which
    // the quorum rides out; the barometer's own failures (a sealed port, the
    // burnout snap-back) read low, i.e. look exactly like this.
    Sim s;
    s.in.baro_rate_mps = 5.0f;
    s.disagreeBy(+12.0f);
    EXPECT_FALSE(s.run(20000));
    s.disagreeBy(+50.0f);
    EXPECT_FALSE(s.run(20000));
    EXPECT_FALSE(s.st.latched);
    EXPECT_FALSE(s.st.disagreeing);
}

TEST(RawVoteGuard, NoLatchAt8Mps)
{
    Sim s;
    s.disagreeBy(-8.0f);
    EXPECT_FALSE(s.run(20000));
    EXPECT_FALSE(s.st.latched);
    EXPECT_FALSE(s.st.disagreeing);
}

TEST(RawVoteGuard, ExactlyTheThresholdIsAgreement)
{
    Sim s;
    s.in.baro_rate_mps    = 5.0f;
    s.in.filter_vz_up_mps = -5.0f;   // 10.0 below, exactly in float
    ASSERT_EQ(s.in.baro_rate_mps - s.cfg.diff_mps, s.in.filter_vz_up_mps);
    EXPECT_FALSE(s.run(5000));
}

TEST(RawVoteGuard, TheConfigIsHonored)
{
    Sim s;
    s.cfg.diff_mps = 5.0f;
    s.cfg.hold_ms  = 250;
    s.disagreeBy(-6.0f);
    EXPECT_FALSE(s.run(250));
    EXPECT_TRUE(s.run(1));
}

// ---------------------------------------------------------------------------
// Never while the barometer cannot vouch, before burnout or after apogee
// ---------------------------------------------------------------------------
TEST(RawVoteGuard, NoneWhileMachLocked)
{
    Sim s;
    s.in.mach_locked = true;
    s.disagreeBy(-30.0f);
    EXPECT_FALSE(s.run(10000));
    EXPECT_FALSE(s.st.disagreeing);
}

TEST(RawVoteGuard, NoneBeforeBurnout)
{
    Sim s;
    s.in.after_burnout = false;
    s.disagreeBy(-30.0f);
    EXPECT_FALSE(s.run(10000));
    EXPECT_FALSE(s.st.disagreeing);
}

TEST(RawVoteGuard, NoneAfterApogee)
{
    // The call is made: kinematicChecks reads ekf_healthy only before it, and
    // the ejection charge and the canopy move the bay pressure.  A latch here
    // would only be a false alarm (and a withheld vote in the next flight).
    Sim s;
    s.in.apogee = true;
    s.disagreeBy(-30.0f);
    EXPECT_FALSE(s.run(10000));
    EXPECT_FALSE(s.st.disagreeing);
    EXPECT_FALSE(s.st.latched);

    // A run in progress when apogee is declared never completes.
    Sim r;
    r.disagreeBy(-15.0f);
    EXPECT_FALSE(r.run(900));
    r.in.apogee = true;
    EXPECT_FALSE(r.run(10000));
    EXPECT_FALSE(r.st.latched);
}

TEST(RawVoteGuard, NoneWithTheBarometerUnhealthy)
{
    Sim s;
    s.in.baro_healthy = false;
    s.disagreeBy(-30.0f);
    EXPECT_FALSE(s.run(10000));
    EXPECT_FALSE(s.st.disagreeing);
}

TEST(RawVoteGuard, NoneWithTheFilterUnhealthy)
{
    // kinematicChecks already drops both of the filter's voters then.
    Sim s;
    s.in.filter_healthy = false;
    s.disagreeBy(-30.0f);
    EXPECT_FALSE(s.run(10000));
    EXPECT_FALSE(s.st.disagreeing);
}

TEST(RawVoteGuard, NoneOnAFlatBarometer)
{
    // A sealed port reads flat: no witness, however far below the filter is.
    Sim s;
    s.in.baro_rate_mps = 0.0f;
    s.disagreeBy(-30.0f);
    EXPECT_FALSE(s.run(5000));
    s.in.baro_rate_mps    = 0.99f;
    s.in.filter_vz_up_mps = -40.0f;
    EXPECT_FALSE(s.run(5000));
    s.in.baro_rate_mps    = -0.99f;
    EXPECT_FALSE(s.run(5000));
    EXPECT_FALSE(s.st.latched);

    // At the bar itself the barometer is moving, and judged.
    s.in.baro_rate_mps    = -1.0f;
    s.in.filter_vz_up_mps = -12.0f;
    EXPECT_FALSE(s.run(1000));
    EXPECT_TRUE(s.run(1));
}

TEST(RawVoteGuard, NoneWhileTheBarometerReadsSixtyOrMore)
{
    Sim s;
    s.in.baro_rate_mps = 60.0f;      // the bound itself is not judged
    s.disagreeBy(-20.0f);
    EXPECT_FALSE(s.run(5000));
    s.in.baro_rate_mps = 90.0f;
    s.disagreeBy(-40.0f);
    EXPECT_FALSE(s.run(5000));
    s.in.baro_rate_mps = -60.0f;     // the bound is on |rate|
    s.disagreeBy(-20.0f);
    EXPECT_FALSE(s.run(5000));
    EXPECT_FALSE(s.st.latched);

    // Just under it, the same 20 m/s disagreement is judged.
    s.in.baro_rate_mps = 59.9f;
    s.disagreeBy(-20.0f);
    EXPECT_FALSE(s.run(1000));
    EXPECT_TRUE(s.run(1));
}

TEST(RawVoteGuard, AFilterReadingAFastDescentIsJudged)
{
    // The bound is on the barometer, so the filter under test cannot switch
    // its own check off by reading fast: before apogee the rocket cannot be
    // descending at 60 m/s, and this is the reading that votes early.
    for (const float vz : {-60.0f, -65.0f, -200.0f})
    {
        SCOPED_TRACE(vz);
        Sim s;
        s.in.baro_rate_mps    = 30.0f;
        s.in.filter_vz_up_mps = vz;
        EXPECT_FALSE(s.run(1000));
        EXPECT_TRUE(s.run(1));
    }
}

TEST(RawVoteGuard, NonFiniteRatesNeverLatch)
{
    Sim s;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    s.in.filter_vz_up_mps = nan;
    EXPECT_FALSE(s.run(3000));
    s.in.filter_vz_up_mps = -inf;    // below anything, and still no witness
    EXPECT_FALSE(s.run(3000));
    s.in.filter_vz_up_mps = 8.0f;
    s.in.baro_rate_mps    = nan;
    EXPECT_FALSE(s.run(3000));
    s.in.baro_rate_mps    = inf;     // a dead rate filter is no witness either
    EXPECT_FALSE(s.run(3000));
    s.in.baro_rate_mps    = -inf;
    EXPECT_FALSE(s.run(3000));
    EXPECT_FALSE(s.st.latched);

    // And one non-finite pass inside a real disagreement breaks the run.
    s.in.baro_rate_mps = 20.0f;
    s.disagreeBy(-12.0f);
    EXPECT_FALSE(s.run(900));
    s.in.baro_rate_mps = nan;
    EXPECT_FALSE(s.run(1));
    EXPECT_FALSE(s.st.disagreeing);
    s.in.baro_rate_mps = 20.0f;
    EXPECT_FALSE(s.run(1000));
    EXPECT_TRUE(s.run(1));
}

// ---------------------------------------------------------------------------
// Continuity: a gap restarts the timer
// ---------------------------------------------------------------------------
TEST(RawVoteGuard, AGapInTheDisagreementRestartsTheTimer)
{
    Sim s;
    s.disagreeBy(-12.0f);
    EXPECT_FALSE(s.run(900));
    s.disagreeBy(-8.0f);         // one pass back inside the threshold
    EXPECT_FALSE(s.run(1));
    EXPECT_FALSE(s.st.disagreeing);
    s.disagreeBy(-12.0f);
    EXPECT_FALSE(s.run(1000));   // 999 ms since the restart, 1.9 s in total
    EXPECT_TRUE(s.run(1));
}

TEST(RawVoteGuard, APassThatCannotJudgeRestartsTheTimer)
{
    // The hold is a second the barometer witnessed: a dropout in the middle
    // does not count, whichever gate shut.  The run is the filter at 8 m/s
    // against a barometer at 20.
    struct Gap { const char* name; void (*shut)(Inputs&); void (*open)(Inputs&); };
    const Gap gaps[] = {
        {"barometer unhealthy", [](Inputs& in) { in.baro_healthy = false; },
                                [](Inputs& in) { in.baro_healthy = true; }},
        {"mach-locked",         [](Inputs& in) { in.mach_locked = true; },
                                [](Inputs& in) { in.mach_locked = false; }},
        {"filter unhealthy",    [](Inputs& in) { in.filter_healthy = false; },
                                [](Inputs& in) { in.filter_healthy = true; }},
        {"barometer flat",      [](Inputs& in) { in.baro_rate_mps = 0.5f;
                                                 in.filter_vz_up_mps = -20.0f; },
                                [](Inputs& in) { in.baro_rate_mps = 20.0f;
                                                 in.filter_vz_up_mps = 8.0f; }},
    };
    for (const Gap& g : gaps)
    {
        SCOPED_TRACE(g.name);
        Sim s;
        s.disagreeBy(-12.0f);
        EXPECT_FALSE(s.run(900));
        g.shut(s.in);
        EXPECT_FALSE(s.run(1));
        EXPECT_FALSE(s.st.disagreeing);
        g.open(s.in);
        EXPECT_FALSE(s.run(1000));
        EXPECT_TRUE(s.run(1));
    }
}

TEST(RawVoteGuard, BarometerSpeedAboveTheBoundRestartsTheTimer)
{
    Sim s;
    s.in.baro_rate_mps = 55.0f;
    s.disagreeBy(-12.0f);        // barometer at 55 m/s: judged
    EXPECT_FALSE(s.run(900));
    s.in.baro_rate_mps = 61.0f;  // 61 m/s: not judged, with the filter 18 below
    EXPECT_FALSE(s.run(1));
    EXPECT_FALSE(s.st.disagreeing);
    s.in.baro_rate_mps = 55.0f;
    EXPECT_FALSE(s.run(1000));
    EXPECT_TRUE(s.run(1));
}

// ---------------------------------------------------------------------------
// Latched for the flight
// ---------------------------------------------------------------------------
TEST(RawVoteGuard, NeverUnlatches)
{
    Sim s;
    s.disagreeBy(-15.0f);
    ASSERT_TRUE(s.run(1001));

    // Agreement, then every input that would stop it judging: still withheld,
    // and step() reports it on every pass, not only the latching one.
    s.disagreeBy(0.0f);
    EXPECT_TRUE(s.run(5000));
    s.disagreeBy(+30.0f);
    EXPECT_TRUE(s.run(1000));
    s.in.baro_healthy = false;
    EXPECT_TRUE(s.run(1000));
    s.in.baro_healthy = true;
    s.in.mach_locked = true;
    EXPECT_TRUE(s.run(1000));
    s.in.mach_locked = false;
    s.in.filter_healthy = false;
    EXPECT_TRUE(s.run(1000));
    s.in.filter_healthy = true;
    s.in.baro_rate_mps = 0.0f;
    EXPECT_TRUE(s.run(1000));
    s.in.baro_rate_mps = 80.0f;
    EXPECT_TRUE(s.run(1000));
    s.in.apogee = true;
    EXPECT_TRUE(s.run(1000));
    s.in.after_burnout = false;
    EXPECT_TRUE(s.run(1000));
    EXPECT_TRUE(s.st.latched);
}

TEST(RawVoteGuard, ResetClearsTheLatchAndTheRun)
{
    Sim s;
    s.disagreeBy(-15.0f);
    ASSERT_TRUE(s.run(1001));
    reset(s.st);
    EXPECT_FALSE(s.st.latched);
    EXPECT_FALSE(s.st.disagreeing);

    // A fresh second is needed after a reset.
    EXPECT_FALSE(s.run(1000));
    EXPECT_TRUE(s.run(1));

    // And a reset mid-run throws the run away.
    reset(s.st);
    EXPECT_FALSE(s.run(900));
    reset(s.st);
    EXPECT_FALSE(s.run(1000));
    EXPECT_TRUE(s.run(1));
}

// ---------------------------------------------------------------------------
// millis() rollover
// ---------------------------------------------------------------------------
TEST(RawVoteGuard, HoldsAcrossTheMillisWrap)
{
    Sim s;
    s.in.now_ms = 0xFFFFFFFFu - 500u;
    s.disagreeBy(-12.0f);
    EXPECT_FALSE(s.run(1000));   // the run spans the wrap
    EXPECT_LT(s.in.now_ms, 1000u);
    EXPECT_TRUE(s.run(1));
}

TEST(RawVoteGuard, ARunStartingAtMillisZeroCounts)
{
    // since_ms == 0 is a real start time after the wrap, not "no run".
    Sim s;
    s.in.now_ms = 0xFFFFFFFFu;   // the next pass is at 0
    s.disagreeBy(-12.0f);
    EXPECT_FALSE(s.run(1));
    EXPECT_EQ(s.in.now_ms, 0u);
    EXPECT_TRUE(s.st.disagreeing);
    EXPECT_EQ(s.st.since_ms, 0u);
    EXPECT_FALSE(s.run(999));
    EXPECT_TRUE(s.run(1));
}

// ---------------------------------------------------------------------------
// A coast, end to end
// ---------------------------------------------------------------------------
namespace {

// Truth: burnout at 55 m/s, then a 10 m/s^2 deceleration through apogee
// (T+5.5 s) and on down, with no apogee call: the guard must hold its own
// even when the call is slow.  The barometer's rate is `baro(t, vz)`; the
// filter reads the truth plus `bias(t)`.
template <typename Bias, typename Baro>
bool flyCoast(Bias bias, Baro baro, uint32_t* latched_at_ms)
{
    State st;
    Inputs in;
    in.after_burnout  = true;
    in.baro_healthy   = true;
    in.filter_healthy = true;
    *latched_at_ms = 0;
    for (uint32_t t = 0; t <= 8000; ++t)
    {
        const float vz = 55.0f - 10.0f * (float)t * 1e-3f;
        in.now_ms           = 50000u + t;
        in.baro_rate_mps    = baro(t, vz);
        in.filter_vz_up_mps = vz + bias(t);
        const bool was = st.latched;
        step(st, in);
        if (!was && st.latched)
        {
            *latched_at_ms = t;
        }
    }
    return st.latched;
}

// A working barometer: its rate lags the deceleration by 2.7 m/s (a model of
// the altitude filter).
float laggingBaro(uint32_t, float vz) { return vz + 2.7f; }

}  // namespace

TEST(RawVoteGuard, AHealthyCoastNeverLatches)
{
    uint32_t at = 0;
    EXPECT_FALSE(flyCoast([](uint32_t) { return 0.0f; }, laggingBaro, &at));
    // Filter noise and a few m/s of its own error stay well inside the band.
    EXPECT_FALSE(flyCoast([](uint32_t t) { return 3.0f * sinf((float)t * 0.05f) - 4.0f; },
                          laggingBaro, &at));
    // A filter reading high all coast holds back its own vote; not judged.
    EXPECT_FALSE(flyCoast([](uint32_t) { return 25.0f; }, laggingBaro, &at));
}

TEST(RawVoteGuard, ACoastFaultRampLatchesOneSecondAfterItCrossesTheBand)
{
    // A common Doppler bias ramping 0 -> -15 m/s over 3 s from T+2 s (burnout
    // time 0).  With the barometer's +2.7 m/s lag the disagreement crosses
    // 10 m/s when the bias reaches -7.3 m/s, 1.46 s into the ramp.
    uint32_t at = 0;
    const bool latched = flyCoast(
        [](uint32_t t) {
            if (t < 2000) return 0.0f;
            const float r = (float)(t - 2000) / 3000.0f;
            return -15.0f * (r < 1.0f ? r : 1.0f);
        },
        laggingBaro, &at);
    ASSERT_TRUE(latched);
    EXPECT_NEAR((double)at, 2000.0 + 1460.0 + 1000.0, 3.0);
}

TEST(RawVoteGuard, ASealedPortNeverLatchesOnACorrectFilter)
{
    // A sealed or taped static port reads fresh, in range and flat, with a
    // slow drift as the bay warms.  The correct filter coasts from 58 m/s
    // through apogee (T+4.83 s) and on to -62 m/s, with no apogee call: a
    // two-sided guard latched at T+1 s here and no apogee was ever called.
    State st;
    Inputs in;
    in.after_burnout  = true;
    in.baro_healthy   = true;    // fresh and in range; not yet judged stuck
    in.filter_healthy = true;
    for (uint32_t t = 0; t <= 10000; ++t)
    {
        in.now_ms           = 50000u + t;
        in.filter_vz_up_mps = 58.0f - 12.0f * (float)t * 1e-3f;
        in.baro_rate_mps    = 0.9f * sinf((float)t * 1e-3f);
        EXPECT_FALSE(step(st, in)) << "at T+" << t << " ms";
        if (st.latched) break;
    }
    EXPECT_FALSE(st.latched);
    EXPECT_FALSE(st.disagreeing);
}

TEST(RawVoteGuard, ABurnoutSnapBackNeverLatches)
{
    // 7/05 V2 F1: at tail-off the bay pressure snaps back, 15 m in 0.25 s
    // while climbing at 46 m/s.  The barometer's rate dives, reading low: not
    // the judged side, however long it lasts (here 40 m/s below the truth for
    // 1.5 s, three times the 7/05 one).
    uint32_t at = 0;
    EXPECT_FALSE(flyCoast([](uint32_t) { return 0.0f; },
                          [](uint32_t t, float vz) {
                              return (t >= 200 && t < 1700) ? vz - 40.0f : vz + 2.7f;
                          },
                          &at));
}
