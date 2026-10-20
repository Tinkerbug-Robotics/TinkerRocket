// Host tests for the satellite cap's governor (TR_RawGnssShadow/CapGovernor.h,
// exp/v9-cap-adapt; registered in v9raw/cap_governor.cmake): off unless a
// ceiling above the floor is given; a satellite at a time up under slack, to
// the ceiling and never past it; a cut on a lag that stays (two of four
// closes), not on one epoch's spike; the floor on an epoch worked at once,
// closed too old or lost; the floor through the boost and its hold;
// no cascade from the epochs built before a change; and no thrashing -- the
// band between the raise and the cut, and the backoff before a cap cut from
// is tried again, doubled while its retries fail -- also against a load whose
// ages grow with the cap, as the open-sky bench's did.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "CapGovernor.h"

using gnss_raw::CapGovernor;

namespace
{

constexpr double kDt = 0.1;   // the mosaic's fused grid: an epoch closed every 0.1 s

CapGovernor::Config cfg16to24()
{
    CapGovernor::Config c;
    c.floor = 16;
    c.ceiling = 24;
    return c;                  // hard 0.2 s: raise at 0.12 or under, cut at 0.16
}

// n epochs closed age_s old, one every kDt from t; returns the time after
double closes(CapGovernor& g, int n, double age_s, double t)
{
    for (int i = 0; i < n; ++i, t += kDt)
        g.onClose(age_s, false, t);
    return t;
}

}  // namespace

TEST(CapGovernor, OffUnlessACeilingAboveTheFloor)
{
    // No ceiling, a ceiling at the floor, or no floor (no cap at all, the
    // Beetle's): the cap is the floor and nothing moves it
    for (const auto& fc : {std::pair<int, int>{16, 0}, {16, 16}, {16, 12}, {0, 24}})
    {
        CapGovernor::Config c;
        c.floor = fc.first;
        c.ceiling = fc.second;
        CapGovernor g(c);
        EXPECT_FALSE(g.on());
        double t = closes(g, 200, 0.02, 0.0);
        g.onLoss(t);
        g.onBoost(t);
        g.onClose(0.5, true, t);
        EXPECT_EQ(g.cap(), fc.first);
        EXPECT_EQ(g.stats().closes, 0u);
        EXPECT_EQ(g.stats().raises + g.stats().cuts + g.stats().floors, 0u);
    }
    CapGovernor g(cfg16to24());
    EXPECT_TRUE(g.on());
    EXPECT_EQ(g.cap(), 16);   // it starts at the floor
}

TEST(CapGovernor, RisesOneSatelliteAtATimeUnderSlackToTheCeiling)
{
    CapGovernor g(cfg16to24());
    double t = 0.0;
    // nine clean closes are not enough; the tenth raises
    t = closes(g, 9, 0.10, t);
    EXPECT_EQ(g.cap(), 16);
    t = closes(g, 1, 0.10, t);
    EXPECT_EQ(g.cap(), 17);
    // the next two were built at 16 (settle_epochs): they count for nothing,
    // so the next raise takes 2 + 10 closes
    t = closes(g, 11, 0.10, t);
    EXPECT_EQ(g.cap(), 17);
    t = closes(g, 1, 0.10, t);
    EXPECT_EQ(g.cap(), 18);
    // and so on to the ceiling, never past it
    t = closes(g, 6 * 12, 0.10, t);
    EXPECT_EQ(g.cap(), 24);
    t = closes(g, 200, 0.05, t);
    EXPECT_EQ(g.cap(), 24);
    EXPECT_EQ(g.stats().raises, 8u);
    EXPECT_EQ(g.stats().cuts + g.stats().floors, 0u);
    // exactly at the raise age counts as clean; a hair over does not
    CapGovernor h(cfg16to24());
    t = closes(h, 10, h.raiseAgeS(), 0.0);
    EXPECT_EQ(h.cap(), 17);
    CapGovernor k(cfg16to24());
    t = closes(k, 9, 0.10, 0.0);
    k.onClose(k.raiseAgeS() + 1e-6, false, t);   // in the band: the run starts over
    t = closes(k, 9, 0.10, t + kDt);
    EXPECT_EQ(k.cap(), 16);
    t = closes(k, 1, 0.10, t);
    EXPECT_EQ(k.cap(), 17);
}

TEST(CapGovernor, CutsOnALagThatStaysNotOnOneSpike)
{
    CapGovernor g(cfg16to24());
    const double cut = g.cutAgeS();
    double t = closes(g, 200, 0.05, 0.0);
    ASSERT_EQ(g.cap(), 24);
    // one epoch at the cut age is a spike (the orbit refresh's): nothing; a
    // second within four closes is a lag: cut_step off
    g.onClose(cut, false, t);
    EXPECT_EQ(g.cap(), 24);
    t = closes(g, 3, 0.05, t + kDt);
    g.onClose(cut, false, t);       // the first fell out of the four
    EXPECT_EQ(g.cap(), 24);
    t = closes(g, 2, 0.05, t + kDt);
    g.onClose(0.19, false, t);      // two of the last four
    EXPECT_EQ(g.cap(), 22);
    EXPECT_EQ(g.stats().cuts, 1u);
    // the next two were built at 24: no second cut from them (no cascade)
    g.onClose(0.18, false, t + 0.1);
    g.onClose(0.17, false, t + 0.2);
    EXPECT_EQ(g.cap(), 22);
    // then two built at 22 that are as late: cut again
    g.onClose(0.17, false, t + 0.3);
    EXPECT_EQ(g.cap(), 22);
    g.onClose(0.17, false, t + 0.4);
    EXPECT_EQ(g.cap(), 20);
    // just under the cut age never cuts, however many
    CapGovernor h(cfg16to24());
    t = closes(h, 200, 0.05, 0.0);
    t = closes(h, 100, cut - 1e-6, t);
    EXPECT_EQ(h.cap(), 24);
    EXPECT_EQ(h.stats().cuts, 0u);
}

TEST(CapGovernor, FloorsOnAnEpochWorkedAtOnceTooOldOrLost)
{
    CapGovernor g(cfg16to24());
    double t = closes(g, 200, 0.05, 0.0);
    ASSERT_EQ(g.cap(), 24);
    t = closes(g, 2, g.cutAgeS(), t);
    ASSERT_EQ(g.cap(), 22);
    // worked at once (late), at any age: the floor, settling or not
    g.onClose(0.05, true, t);
    EXPECT_EQ(g.cap(), 16);
    EXPECT_EQ(g.stats().floors, 1u);
    EXPECT_EQ(g.stats().late, 1u);

    CapGovernor h(cfg16to24());
    t = closes(h, 200, 0.05, 0.0);
    h.onClose(0.2, false, t);                // closed at the at-once point
    EXPECT_EQ(h.cap(), 16);
    CapGovernor k(cfg16to24());
    t = closes(k, 200, 0.05, 0.0);
    k.onLoss(t);                             // an epoch lost on the way
    EXPECT_EQ(k.cap(), 16);
    EXPECT_EQ(k.stats().lost, 1u);
    EXPECT_EQ(k.stats().floors, 1u);
    // at the floor already: counted, nothing else moves
    k.onLoss(t + 0.1);
    k.onClose(0.3, true, t + 0.2);
    EXPECT_EQ(k.cap(), 16);
    EXPECT_EQ(k.stats().floors, 1u);
    EXPECT_EQ(k.stats().lost, 2u);
    EXPECT_EQ(k.stats().late, 1u);
    // a cut never goes under the floor
    CapGovernor m(cfg16to24());
    t = closes(m, 10, 0.05, 0.0);
    ASSERT_EQ(m.cap(), 17);
    t = closes(m, 2, 0.05, t);               // settle
    t = closes(m, 2, 0.17, t);
    EXPECT_EQ(m.cap(), 16);
}

TEST(CapGovernor, HoldsTheFloorThroughTheBoostAndItsHold)
{
    CapGovernor g(cfg16to24());
    double t = closes(g, 200, 0.05, 0.0);
    ASSERT_EQ(g.cap(), 24);
    // the launch: the floor at once; every step of the boost says so again
    const double launch = t;
    g.onBoost(launch);
    EXPECT_EQ(g.cap(), 16);
    double last = launch;
    for (int k = 0; k < 300; ++k)
    {
        last = launch + 0.01 * k;
        g.onBoost(last);
    }
    EXPECT_EQ(g.stats().boosts, 1u);
    EXPECT_TRUE(g.boostHeld(last + 1.99));
    EXPECT_FALSE(g.boostHeld(last + 2.0));
    // clean epochs during the boost and its hold raise nothing
    for (t = launch; t < last + 2.0; t += kDt)
        g.onClose(0.05, false, t);
    EXPECT_EQ(g.cap(), 16);
    EXPECT_EQ(g.stats().raises, 8u);
    // after the hold the climb starts over, a satellite a raise_epochs
    t = last + 2.0;
    t = closes(g, 10, 0.05, t);
    EXPECT_EQ(g.cap(), 17);
    // the boost is no failure: no backoff on the caps it took the cap from
    t = closes(g, 7 * 12, 0.05, t);
    EXPECT_EQ(g.cap(), 24);
    EXPECT_EQ(g.stats().cuts + g.stats().floors, 0u);
    // a second boost hold is counted as one
    g.onBoost(t);
    EXPECT_EQ(g.stats().boosts, 2u);
}

TEST(CapGovernor, TheBandBetweenTheRaiseAndTheCutHoldsTheCap)
{
    // Ages between 0.12 and 0.16 s neither raise nor cut, however long
    CapGovernor g(cfg16to24());
    double t = closes(g, 3 * 12 - 2, 0.05, 0.0);
    ASSERT_EQ(g.cap(), 19);
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> band(0.1201, 0.1599);
    for (int i = 0; i < 6000; ++i, t += kDt)
        g.onClose(band(rng), false, t);
    EXPECT_EQ(g.cap(), 19);
    EXPECT_EQ(g.stats().raises, 3u);
    EXPECT_EQ(g.stats().cuts + g.stats().floors, 0u);
}

TEST(CapGovernor, ACapCutFromWaitsForTheBackoffAndItsFailedRetriesDoubleIt)
{
    CapGovernor g(cfg16to24());
    double t = closes(g, 3 * 12, 0.05, 0.0);   // the last two settle
    ASSERT_EQ(g.cap(), 19);
    // cut from 19 to 17 at t0
    g.onClose(0.17, false, t);
    const double t0 = t + kDt;
    g.onClose(0.17, false, t0);
    ASSERT_EQ(g.cap(), 17);
    EXPECT_DOUBLE_EQ(g.backoff(), 10.0);
    // 18 is under the cap cut from: it may come back after its run
    t = closes(g, 12, 0.05, t0 + kDt);
    EXPECT_EQ(g.cap(), 18);
    // 19 waits for the backoff, however clean
    while (t < t0 + 10.0 - 1e-9)
    {
        g.onClose(0.05, false, t);
        EXPECT_EQ(g.cap(), 18) << t - t0;
        t += kDt;
    }
    t = std::max(t, t0 + 10.0);
    g.onClose(0.05, false, t);   // the run was long done: the first close after it raises
    EXPECT_EQ(g.cap(), 19);
    // 19 fails again: its backoff doubles, and doubles up to 80 s
    double expect = 10.0;
    for (int retry = 0; retry < 5; ++retry)
    {
        t = closes(g, 2, 0.05, t + kDt);     // settle
        g.onClose(0.17, false, t);
        t += kDt;
        g.onClose(0.17, false, t);
        expect = std::min(2.0 * expect, 80.0);
        EXPECT_DOUBLE_EQ(g.backoff(), expect) << retry;
        ASSERT_EQ(g.cap(), 17);
        const double cut_t = t;
        t = closes(g, 12, 0.05, t + kDt);
        ASSERT_EQ(g.cap(), 18);
        while (t < cut_t + expect - 1e-9)
        {
            g.onClose(0.05, false, t);
            ASSERT_EQ(g.cap(), 18);
            t += kDt;
        }
        t = std::max(t, cut_t + expect);
        g.onClose(0.05, false, t);
        ASSERT_EQ(g.cap(), 19);
    }
    // a cut from another cap starts the backoff over
    t = closes(g, 12, 0.05, t + kDt);
    ASSERT_EQ(g.cap(), 20);
    t = closes(g, 2, 0.05, t);
    t = closes(g, 2, 0.17, t);
    EXPECT_EQ(g.cap(), 18);
    EXPECT_DOUBLE_EQ(g.backoff(), 10.0);
}

TEST(CapGovernor, SettlesWithoutThrashingOnALoadWhoseAgesGrowWithTheCap)
{
    // The open-sky bench's slope: the age at close about 5 ms more a
    // satellite, around 109 ms at 16 (boost-screen costs), with a spread and
    // every 50th epoch (the 5-s orbit refresh) 25 ms later; and two cheaper
    // satellites. The governor must find the cap where the ages stay under
    // the raise age and stay there: over 10 minutes, the climb and a few
    // probes at most, never the floor.
    struct Load
    {
        double base_ms, per_sat_ms;
    };
    for (const Load& L : {Load{109.0 - 16 * 5.0, 5.0}, Load{60.0 - 16 * 2.5, 2.5}, Load{70.0 - 16 * 3.0, 3.0}})
    {
        CapGovernor g(cfg16to24());
        std::mt19937 rng(3);
        std::normal_distribution<double> noise(0.0, 8.0);
        double t = 0.0;
        int changes = 0, last_cap = g.cap();
        std::vector<int> caps;
        for (int i = 0; i < 6000; ++i, t += kDt)
        {
            double age_ms = L.base_ms + L.per_sat_ms * g.cap() + noise(rng);
            if (i % 50 == 49)
                age_ms += 25.0;
            g.onClose(age_ms * 1e-3, false, t);
            if (g.cap() != last_cap)
                changes++;
            last_cap = g.cap();
            if (i >= 3000)
                caps.push_back(g.cap());
        }
        int lo = 99, hi = 0;
        for (int c : caps)
        {
            lo = std::min(lo, c);
            hi = std::max(hi, c);
        }
        // over the last five minutes it stays within a satellite
        EXPECT_LE(hi - lo, 1) << L.per_sat_ms;
        // and it changed at most a dozen times in ten minutes (the climb and
        // the probes the backoff spaces out), never to the floor
        EXPECT_LE(changes, 12) << L.per_sat_ms;
        EXPECT_EQ(g.stats().floors, 0u) << L.per_sat_ms;
        if (L.per_sat_ms == 5.0)
        {
            EXPECT_GE(lo, 17);        // today's cost: a few satellites over the floor,
            EXPECT_LE(hi, 20);        //   its ages under the raise age most of the time
        }
        else
        {
            EXPECT_EQ(lo, 24);        // a cheaper satellite: the ceiling
        }
    }
}

TEST(CapGovernor, ResetIsAStart)
{
    CapGovernor g(cfg16to24());
    double t = closes(g, 200, 0.05, 0.0);
    t = closes(g, 2, 0.17, t);
    g.onBoost(t + 0.1);
    g.reset();
    EXPECT_EQ(g.cap(), 16);
    EXPECT_FALSE(g.boostHeld(t + 0.2));
    // no backoff carried over: a clean run raises from the start
    t = closes(g, 8 * 12, 0.05, t + 0.2);
    EXPECT_EQ(g.cap(), 24);
}

TEST(CapGovernor, TheWindowKeepsTheCapsRangeAndTheOldestClose)
{
    CapGovernor g(cfg16to24());
    int lo = 0, hi = 0;
    double amax = 0.0;
    double t = closes(g, 22, 0.05, 0.0);
    g.onClose(0.11, false, t);
    g.takeWindow(lo, hi, amax);
    EXPECT_EQ(lo, 16);
    EXPECT_EQ(hi, 18);
    EXPECT_DOUBLE_EQ(amax, 0.11);
    g.onLoss(t + 0.1);
    g.takeWindow(lo, hi, amax);
    EXPECT_EQ(lo, 16);
    EXPECT_EQ(hi, 18);
    EXPECT_DOUBLE_EQ(amax, 0.0);
    g.takeWindow(lo, hi, amax);
    EXPECT_EQ(lo, 16);
    EXPECT_EQ(hi, 16);
}
