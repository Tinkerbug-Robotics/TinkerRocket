// CENJARS 2026-11-01: the OC's verdict on whether the camera is running, from
// its rail current (#850 monitor, 10 Hz). Pins the thresholds against the
// currents the logs show, the half-second debounce, the hysteresis, and that an
// unreadable monitor is "unknown", never "dark".

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "cam_draw_policy.h"

using CamDrawPolicy::Debounce;
using CamDrawPolicy::kReads;

namespace {

void feed(Debounce& d, float amps, int n)
{
    for (int i = 0; i < n; ++i) d.update(amps);
}

}  // namespace

TEST(CamDrawPolicy, StartsUnmonitoredAndDark)
{
    Debounce d;
    EXPECT_FALSE(d.monitored());
    EXPECT_FALSE(d.drawing());
}

TEST(CamDrawPolicy, GateOffReadsMonitoredAndDark)
{
    Debounce d;
    feed(d, 0.000f, 20);
    EXPECT_TRUE(d.monitored());
    EXPECT_FALSE(d.drawing());
}

TEST(CamDrawPolicy, RunningNeedsHalfASecondOfRunningCurrent)
{
    Debounce d;
    feed(d, 0.0f, 3);
    feed(d, 0.085f, kReads - 1);   // the RunCam's boot sample, one read short
    EXPECT_FALSE(d.drawing());
    d.update(0.085f);
    EXPECT_TRUE(d.drawing());
}

TEST(CamDrawPolicy, ThresholdsSitBetweenTheLoggedLevels)
{
    // Every running level in the logs is at or above the on threshold, and the
    // gate-off reading is below the off threshold.
    for (float a : {0.085f, 0.101f, 0.140f, 0.225f, 0.321f,      // RunCam boot, recording
                    0.17f, 0.35f, 0.59f, 1.085f})                // GoPro idle, recording
    {
        Debounce d;
        feed(d, a, kReads);
        EXPECT_TRUE(d.drawing()) << a << " A";
    }
    Debounce d;
    feed(d, 0.0f, 50);
    EXPECT_FALSE(d.drawing());
}

TEST(CamDrawPolicy, OneLowReadDoesNotDropARunningCamera)
{
    Debounce d;
    feed(d, 0.2f, kReads);
    ASSERT_TRUE(d.drawing());
    feed(d, 0.0f, kReads - 1);
    d.update(0.2f);                // run broken before it completed
    feed(d, 0.0f, kReads - 1);
    EXPECT_TRUE(d.drawing());
    d.update(0.0f);
    EXPECT_FALSE(d.drawing());     // kReads in a row: the gate went off
}

TEST(CamDrawPolicy, HysteresisBandHoldsTheVerdict)
{
    Debounce on;
    feed(on, 0.2f, kReads);
    feed(on, 0.040f, 50);          // between off (30 mA) and on (50 mA)
    EXPECT_TRUE(on.drawing());

    Debounce off;
    feed(off, 0.0f, kReads);
    feed(off, 0.040f, 50);
    EXPECT_FALSE(off.drawing());
}

TEST(CamDrawPolicy, NoReadingIsUnknownNotDark)
{
    Debounce d;
    feed(d, 0.2f, kReads);
    ASSERT_TRUE(d.drawing());
    d.update(NAN);
    EXPECT_FALSE(d.monitored());
    EXPECT_FALSE(d.drawing());     // drawing() never claims "running" unmonitored
    // And the debounce starts over: a fresh number has to earn "running" again.
    feed(d, 0.2f, kReads - 1);
    EXPECT_TRUE(d.monitored());
    EXPECT_FALSE(d.drawing());
    d.update(0.2f);
    EXPECT_TRUE(d.drawing());
}

TEST(CamDrawPolicy, GtvWhitePadSequence)
{
    // GTV White's camera current on the pad as the FC's ladder drove it,
    // at 10 Hz (LoRa 3 s samples, held): on at 0.178, off 3 s, on, off, on.
    struct Seg { float amps; int reads; bool want; };
    const std::vector<Seg> segs = {
        {0.000f, 10, false}, {0.178f, 80, true},  {0.000f, 30, false},
        {0.236f, 80, true},  {0.000f, 30, false}, {0.220f, 40, true},
    };
    Debounce d;
    for (const Seg& s : segs)
    {
        feed(d, s.amps, s.reads);
        EXPECT_EQ(d.drawing(), s.want) << s.amps << " A";
    }
}
