// Host tests for RawVoteGuard (mosaic-only build): after burnout, a raw-GNSS
// filter whose vertical velocity disagrees with the barometer's climb rate by
// more than 10 m/s for a continuous second loses its velocity and pitch apogee
// votes for the rest of the flight.  The guard is the only cross-check left
// when the raw filter is the one navigation filter, so these pin both halves:
// it latches on a real disagreement, and it never acts on a barometer that
// cannot vouch, before burnout, or at speed, and never lets the filter back.

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

    // The filter reads `diff` m/s above the barometer.
    void disagreeBy(float diff)
    {
        in.filter_vz_up_mps = in.baro_rate_mps + diff;
    }
};

}  // namespace

// ---------------------------------------------------------------------------
// The defaults are the plan's numbers
// ---------------------------------------------------------------------------
TEST(RawVoteGuard, DefaultsAreTenMpsForOneSecondBelowSixtyMps)
{
    const Config cfg;
    EXPECT_FLOAT_EQ(cfg.diff_mps, 10.0f);
    EXPECT_EQ(cfg.hold_ms, 1000u);
    EXPECT_FLOAT_EQ(cfg.vz_max_mps, 60.0f);

    const State st;
    EXPECT_FALSE(st.latched);
    EXPECT_FALSE(st.disagreeing);
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
// Latching
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

TEST(RawVoteGuard, LatchesWhenTheFilterReadsHighToo)
{
    // A coherent Doppler bias that holds the filter's climb up delays its
    // votes instead of advancing them, but it is the same fault: the filter
    // and its only witness disagree.
    Sim s;
    s.in.baro_rate_mps = 5.0f;
    s.disagreeBy(+12.0f);
    EXPECT_FALSE(s.run(1000));
    EXPECT_TRUE(s.run(1));
}

TEST(RawVoteGuard, NoLatchAt8Mps)
{
    Sim s;
    s.disagreeBy(-8.0f);
    EXPECT_FALSE(s.run(20000));
    s.disagreeBy(+8.0f);
    EXPECT_FALSE(s.run(20000));
    EXPECT_FALSE(s.st.latched);
    EXPECT_FALSE(s.st.disagreeing);
}

TEST(RawVoteGuard, ExactlyTheThresholdIsAgreement)
{
    Sim s;
    s.in.baro_rate_mps    = -5.0f;
    s.in.filter_vz_up_mps = 5.0f;   // 10.0 exactly in float
    ASSERT_EQ(fabsf(s.in.filter_vz_up_mps - s.in.baro_rate_mps), 10.0f);
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
// Never while the barometer cannot vouch, before burnout, or at speed
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

TEST(RawVoteGuard, NoneAtOrAboveSixtyMps)
{
    Sim s;
    s.in.baro_rate_mps    = 40.0f;
    s.in.filter_vz_up_mps = 60.0f;   // the bound itself is not judged
    EXPECT_FALSE(s.run(5000));
    s.in.filter_vz_up_mps = 90.0f;
    EXPECT_FALSE(s.run(5000));
    s.in.baro_rate_mps    = -20.0f;
    s.in.filter_vz_up_mps = -60.0f;  // the bound is on |vz|
    EXPECT_FALSE(s.run(5000));
    EXPECT_FALSE(s.st.latched);

    // Just under it, the same 20 m/s disagreement is judged.
    s.in.baro_rate_mps    = 39.9f;
    s.in.filter_vz_up_mps = 59.9f;
    EXPECT_FALSE(s.run(1000));
    EXPECT_TRUE(s.run(1));
}

TEST(RawVoteGuard, NonFiniteRatesNeverLatch)
{
    Sim s;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    s.in.filter_vz_up_mps = nan;
    EXPECT_FALSE(s.run(3000));
    s.in.filter_vz_up_mps = 20.0f;
    s.in.baro_rate_mps    = nan;
    EXPECT_FALSE(s.run(3000));
    s.in.baro_rate_mps    = inf;   // a dead rate filter is no witness either
    EXPECT_FALSE(s.run(3000));
    s.in.baro_rate_mps    = -inf;
    EXPECT_FALSE(s.run(3000));
    s.in.filter_vz_up_mps = inf;
    s.in.baro_rate_mps    = 0.0f;
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
    // does not count, whichever gate shut.
    struct Gap { const char* name; void (*shut)(Inputs&); void (*open)(Inputs&); };
    const Gap gaps[] = {
        {"barometer unhealthy", [](Inputs& in) { in.baro_healthy = false; },
                                [](Inputs& in) { in.baro_healthy = true; }},
        {"mach-locked",         [](Inputs& in) { in.mach_locked = true; },
                                [](Inputs& in) { in.mach_locked = false; }},
        {"filter unhealthy",    [](Inputs& in) { in.filter_healthy = false; },
                                [](Inputs& in) { in.filter_healthy = true; }},
    };
    for (const Gap& g : gaps)
    {
        SCOPED_TRACE(g.name);
        Sim s;
        s.disagreeBy(-12.0f);
        EXPECT_FALSE(s.run(900));
        g.shut(s.in);
        EXPECT_FALSE(s.run(1));
        g.open(s.in);
        EXPECT_FALSE(s.run(1000));
        EXPECT_TRUE(s.run(1));
    }
}

TEST(RawVoteGuard, SpeedAboveTheBoundRestartsTheTimer)
{
    Sim s;
    s.in.baro_rate_mps = 45.0f;
    s.disagreeBy(+12.0f);        // 57 m/s: judged
    EXPECT_FALSE(s.run(900));
    s.disagreeBy(+16.0f);        // 61 m/s: not judged
    EXPECT_FALSE(s.run(1));
    s.disagreeBy(+12.0f);
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
    s.in.baro_healthy = false;
    EXPECT_TRUE(s.run(1000));
    s.in.baro_healthy = true;
    s.in.mach_locked = true;
    EXPECT_TRUE(s.run(1000));
    s.in.mach_locked = false;
    s.in.filter_healthy = false;
    EXPECT_TRUE(s.run(1000));
    s.in.filter_healthy = true;
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

// Truth: burnout at 55 m/s, then a 10 m/s^2 deceleration through apogee.  The
// barometer's rate lags the deceleration by 2.7 m/s (a model of the altitude
// filter); the filter reads the truth plus `bias(t)`.
template <typename Bias>
bool flyCoast(Bias bias, uint32_t* latched_at_ms)
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
        in.baro_rate_mps    = vz + 2.7f;
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

}  // namespace

TEST(RawVoteGuard, AHealthyCoastNeverLatches)
{
    uint32_t at = 0;
    EXPECT_FALSE(flyCoast([](uint32_t) { return 0.0f; }, &at));
    // Filter noise and a few m/s of its own error stay well inside the band.
    EXPECT_FALSE(flyCoast([](uint32_t t) { return 3.0f * sinf((float)t * 0.05f) - 4.0f; },
                          &at));
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
        &at);
    ASSERT_TRUE(latched);
    EXPECT_NEAR((double)at, 2000.0 + 1460.0 + 1000.0, 3.0);
}
