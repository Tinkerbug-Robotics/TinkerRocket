// The ephemeris store at the mosaic-G5 build's size (TR_EPH_CAPACITY=128,
// this target's alone: v9raw/stage_v9.cmake) through a pad wait.
//
// The mosaic sends the navigation data of every GPS, Galileo and BeiDou
// satellite it tracks, each as it changes: a Galileo I/NAV every 10 min, a
// BeiDou D1 every hour, a GPS LNAV every 2 h with its toe 2 h ahead (the
// middle of its fit). Over a 2-4 h wait about 30 satellites are up at a time
// and ~50 different ones rise and set, two each ~100 ephemerides; the
// Beetle's 64 would drop ones still in use. Here a synthetic sky (no real
// place or date: the week turns over mid-wait) is fed to the store as the
// receiver sends it, each ephemeris once, and every 30 s each satellite up
// must have its two latest ephemerides, and one to be built by whenever it
// was sent one within 2 h. A sky that turns over faster than any real one
// fills the pool: the ephemeris furthest in time from the new one goes, a
// set satellite's, and the satellites up still keep two each.
//
// Its limit: with passes under ~2 h (90-100 satellites in 4 h), the set
// satellites' ephemerides are all nearer a new one than a GPS ephemeris an
// hour ahead, and that goes, a satellite's only one, until the receiver
// sends it again (the pad's resend while tracked_no_eph > 0).
//
// The same wait through the stage, as the mosaic's driver files them
// (RawGnssMeasStage::addEphemeris, each with its week, the receiver's
// epochs seen between): its check of a toe against the receiver's time
// refuses none of them, across the week's turn too. And the stage, the store
// and this file are built with one capacity (ephCapacityAgrees).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "EphemerisStore.h"
#include "RawGnssMeasStage.h"
#include "gnss/eph.h"
#include "gnss/types.h"

using gnss_raw::EphemerisStore;
using gnss_raw::RawGnssMeasStage;

namespace
{

constexpr double kWeek = 604800.0;

struct Sat
{
    int    sys, prn;
    double rise, set;   // continuous GPS seconds
};

// Seconds of week from continuous seconds.
double sow(double t)
{
    const double w = std::fmod(t, kWeek);
    return w < 0.0 ? w + kWeek : w;
}

// The toe (continuous, in the satellite's own system time) of the ephemeris
// a satellite sends at its system time t.
double toeAt(int sys, double t)
{
    if (sys == GNSS_SYS_GAL)
        return std::floor(t / 600.0) * 600.0;
    if (sys == GNSS_SYS_BDS)
        return std::floor(t / 3600.0) * 3600.0;
    return std::floor(t / 7200.0) * 7200.0 + 7200.0;   // GPS: 2 h ahead
}

struct WaitResult
{
    size_t peak_used = 0;   // the pool's fullest
    int    most_up = 0;     // satellites up at once, at most
    int    checks = 0;      // satellite checks made
    int    missing_two = 0; // a satellite up without its two latest
    int    missing_one = 0; // a satellite up with none to be built by, though it was sent one within 2 h
    uint32_t refused = 0;   // through the stage: ephemerides it refused
};

// n_sats different satellites over hours, 30 of them up at a time: the
// first 30 up when the wait starts (the receiver has just started: none of
// their ephemerides yet), each later one rising as one sets, every pass the
// same length. Into a store, or through a stage (via_stage) that sees an
// epoch every 30 s.
WaitResult padWait(int n_sats, double hours, bool via_stage = false)
{
    const int kUp = 30;
    const double t0 = kWeek * 2440.0 + kWeek - 2.0 * 3600.0 + 123.0;   // 2 h before a week turns over
    const double span = hours * 3600.0;
    const double pass = kUp * span / (n_sats - kUp);
    std::vector<Sat> sats;
    for (int i = 0; i < n_sats; ++i)
    {
        Sat s;
        s.sys = i % 3;                                   // GPS, Galileo, BeiDou in turn
        s.prn = 1 + i / 3 + (s.sys == GNSS_SYS_BDS ? 18 : 0);   // BeiDou's MEO PRNs from 19
        if (i < kUp)
        {
            s.rise = t0 - 3600.0;
            s.set = t0 + (i + 0.5) / kUp * pass;
        }
        else
        {
            s.rise = t0 + (i - kUp + 0.5) / (n_sats - kUp) * span;
            s.set = s.rise + pass;
        }
        sats.push_back(s);
    }

    EphemerisStore bare;
    RawGnssMeasStage stage;
    const EphemerisStore& store = via_stage ? stage.ephemerides() : bare;
    WaitResult r;
    std::vector<std::vector<double>> sent(sats.size());   // each satellite's toes, as sent
    for (double t = t0; t <= t0 + span; t += 30.0)
    {
        if (via_stage)
        {
            gnss_raw::TimedRawEpoch ep;                   // the receiver's time, no satellites
            ep.epoch.tow_s = sow(t);
            ep.gps_week = static_cast<uint16_t>(std::floor(t / kWeek));
            stage.see(ep);
        }
        int up = 0;
        for (size_t i = 0; i < sats.size(); ++i)
        {
            const Sat& s = sats[i];
            if (t < s.rise || t >= s.set)
                continue;
            up++;
            const double ts = t + (s.sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0);
            const double toe = toeAt(s.sys, ts);
            std::vector<double>& mine = sent[i];
            if (mine.empty() || mine.back() != toe)
            {
                gps_eph_t e;
                std::memset(&e, 0, sizeof(e));
                e.valid = 1;
                e.sys = s.sys;
                e.prn = s.prn;
                e.week = static_cast<int>(std::floor(toe / kWeek));   // BeiDou's: BDT's + 1356, as SbfNav gives it
                e.toe = e.toc = sow(toe);
                e.iode = static_cast<int>(std::fmod(toe / 600.0, 256.0));
                e.sqrt_a = 5153.7;
                EXPECT_TRUE(via_stage ? stage.addEphemeris(e) : bare.add(e));
                mine.push_back(toe);
            }
            // the two latest held, and one to build by if one sent is within 2 h
            r.checks++;
            const size_t want = std::min<size_t>(2, mine.size());
            size_t held = 0;
            for (size_t k = mine.size() - want; k < mine.size(); ++k)
            {
                const gps_eph_t* h = store.select(s.sys, s.prn, sow(mine[k]));
                held += (h != nullptr && h->toe == sow(mine[k])) ? 1 : 0;
            }
            r.missing_two += held < want ? 1 : 0;
            bool recent = false;
            for (double x : mine)
                recent = recent || std::fabs(ts - x) < EphemerisStore::kMaxAgeS;
            r.missing_one += (recent && store.select(s.sys, s.prn, sow(ts)) == nullptr) ? 1 : 0;
        }
        r.most_up = std::max(r.most_up, up);
        r.peak_used = std::max(r.peak_used, store.size());
    }
    r.refused = stage.stats().eph_bad;
    return r;
}

}  // namespace

TEST(EphemerisStorePadWait, TheMosaicBuildHolds128)
{
    static_assert(EphemerisStore::kCapacity == 128, "this target is built with TR_EPH_CAPACITY=128");
    // EphemerisStore.cpp and RawGnssMeasStage.cpp were built with it too
    EXPECT_EQ(EphemerisStore::builtCapacity(), 128u);
    EXPECT_TRUE(RawGnssMeasStage::ephCapacityAgrees(EphemerisStore::kCapacity));
    EXPECT_FALSE(RawGnssMeasStage::ephCapacityAgrees(64));   // a component built without it
}

TEST(EphemerisStorePadWait, TwoToFourHoursOfFiftySatellitesKeepTwoEachForEveryOneUp)
{
    for (double hours : {2.0, 3.0, 4.0})
    {
        const WaitResult r = padWait(50, hours);
        EXPECT_EQ(r.most_up, 30) << hours << " h";
        EXPECT_GT(r.checks, 0);
        EXPECT_EQ(r.missing_two, 0) << hours << " h";
        EXPECT_EQ(r.missing_one, 0) << hours << " h";
        EXPECT_GT(r.peak_used, 64u) << hours << " h";   // more than the Beetle's pool
        EXPECT_LE(r.peak_used, 100u) << hours << " h";  //   and room to spare in 128
    }
}

TEST(EphemerisStorePadWait, AFullPoolDropsASetSatellitesEphemerisFirst)
{
    // 64 to 80 different satellites in 4 h -- passes of 3.5 h down to
    // 2.4 h, half a real MEO pass -- fill the pool. What goes is furthest in
    // time from what comes, and that is never a satellite up.
    for (int n : {64, 70, 80})
    {
        const WaitResult r = padWait(n, 4.0);
        EXPECT_EQ(r.most_up, 30) << n;
        EXPECT_EQ(r.missing_two, 0) << n << " satellites";
        EXPECT_EQ(r.missing_one, 0) << n << " satellites";
        if (n >= 70)
            EXPECT_EQ(r.peak_used, EphemerisStore::kCapacity) << n;   // full, and evicting
    }
}

TEST(EphemerisStorePadWait, TheStageFilesEveryOneWithItsWeek)
{
    // As the mosaic's driver files them, each with its week, the receiver's
    // time known: a GPS toe 2 h ahead, BeiDou's in its own time and the
    // week's turn are all within the stage's check, and the store keeps what
    // it keeps when filled directly.
    for (double hours : {2.0, 4.0})
    {
        const WaitResult direct = padWait(50, hours);
        const WaitResult r = padWait(50, hours, true);
        EXPECT_EQ(r.refused, 0u) << hours << " h";
        EXPECT_EQ(r.missing_two, 0) << hours << " h";
        EXPECT_EQ(r.missing_one, 0) << hours << " h";
        EXPECT_EQ(r.checks, direct.checks) << hours << " h";
        EXPECT_EQ(r.peak_used, direct.peak_used) << hours << " h";
    }
}
