// Host tests for lc86::GnssTimeSync: the MCU clock against GPS time from the
// receiver's sentence stamps alone (#1600). A simulated LC86G ends each
// $PQTMPVT a fixed delay after its GPS epoch and sends the rest of its burst
// behind it; a simulated GNSS poll reads the UART every 27 ms (give or take
// its scheduling) and backdates the sentence by the bytes behind it, as
// TR_GNSSReceiverLC86Serial::pollNewPVT does. The MCU crystal runs 30 ppm fast.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>

#include "GnssClockRecord.h"
#include "GnssTimeSync.h"

using lc86::GnssTimeSync;

namespace
{

struct Receiver
{
    double  drift = 30e-6;             // the MCU crystal against GPS
    double  latency = 0.083;           // GPS epoch -> last byte of its $PQTMPVT, s
    double  burst_after = 0.015;       // the rest of the burst behind it, s
    double  poll = 0.027;              // the GNSS poll period, s
    double  poll_jitter = 0.002;       // its scheduling, s
    int64_t mcu_at_gps0_us = 123456789;
    double  gps0 = 2433 * 604800.0 + 500000.0;
    std::mt19937 rng{7};
    double  next_poll_us = 0.0;

    // The exact MCU time (us) of a GPS instant.
    double mcuUs(double gps) const { return mcu_at_gps0_us + (gps - gps0) * (1.0 + drift) * 1.0e6; }

    // The stamp the driver gives the $PQTMPVT of epoch `gps`: the first poll
    // after its last byte, less the burst bytes that had arrived behind it by
    // then. Exact while the burst is still arriving; late by the idle gap after.
    int64_t stamp(double gps)
    {
        const double end = mcuUs(gps) + latency * 1.0e6;
        const double burst_end = end + burst_after * 1.0e6;
        std::uniform_real_distribution<double> jit(-poll_jitter * 1.0e6, poll_jitter * 1.0e6);
        if (next_poll_us == 0.0)
            next_poll_us = end - 1000.0;
        while (next_poll_us < end)
            next_poll_us += poll * 1.0e6 + jit(rng);
        const double behind = std::min(next_poll_us, burst_end) - end;
        return static_cast<int64_t>(std::llround(next_poll_us - behind));
    }

    // Ten epochs a second from GPS time `from` for `seconds`.
    void feed(GnssTimeSync& ts, double from, double seconds)
    {
        for (int k = 0; k < static_cast<int>(std::lround(seconds * 10.0)); ++k)
        {
            const double gps = from + k * 0.1;
            ts.addSample(gps, stamp(gps));
        }
    }
};

GnssTimeSync calibrated(const Receiver& rx)
{
    GnssTimeSync::Config cfg;
    cfg.latency_s = rx.latency;
    return GnssTimeSync(cfg);
}

double errorUs(const GnssTimeSync& ts, const Receiver& rx, double gps)
{
    int64_t mcu = 0;
    EXPECT_TRUE(ts.mcuAt(gps, mcu));
    return static_cast<double>(mcu) - rx.mcuUs(gps);
}

}  // namespace

TEST(GnssTimeSync, PlacesGpsTimeOnTheMcuClockWithinAMillisecond)
{
    Receiver rx;
    GnssTimeSync ts = calibrated(rx);
    rx.feed(ts, rx.gps0, 60.0);
    ASSERT_TRUE(ts.valid());
    const double last = rx.gps0 + 59.9;
    for (double gps : {last, last - 30.0, last - 59.0, last + 0.5})
        EXPECT_LT(std::fabs(errorUs(ts, rx, gps)), 500.0) << "at GPS + " << gps - rx.gps0;
    EXPECT_NEAR(ts.driftPpm(), 30.0, 2.0);
    EXPECT_EQ(ts.restarts(), 0u);
}

TEST(GnssTimeSync, AnswersOnlyAfterTenSecondsOfHistory)
{
    Receiver rx;
    GnssTimeSync ts = calibrated(rx);
    rx.feed(ts, rx.gps0, 5.0);
    EXPECT_FALSE(ts.valid());
    int64_t mcu = 0;
    double gps = 0.0;
    EXPECT_FALSE(ts.mcuAt(rx.gps0, mcu));
    EXPECT_FALSE(ts.gpsAt(rx.mcu_at_gps0_us, gps));
    rx.feed(ts, rx.gps0 + 5.0, 7.0);
    EXPECT_TRUE(ts.valid());
}

TEST(GnssTimeSync, GpsAtInvertsMcuAt)
{
    Receiver rx;
    GnssTimeSync ts = calibrated(rx);
    rx.feed(ts, rx.gps0, 40.0);
    for (double gps : {rx.gps0 + 1.0, rx.gps0 + 39.95, rx.gps0 + 45.0})
    {
        int64_t mcu = 0;
        double back = 0.0;
        ASSERT_TRUE(ts.mcuAt(gps, mcu));
        ASSERT_TRUE(ts.gpsAt(mcu, back));
        EXPECT_NEAR(back, gps, 2.0e-6);
    }
}

TEST(GnssTimeSync, ReportsHowLateThePollLeftTheStampsAndHowTightTheEnvelopeIs)
{
    Receiver rx;
    GnssTimeSync ts = calibrated(rx);
    rx.feed(ts, rx.gps0, 60.0);
    // about half the polls land mid-burst and backdate exactly; the rest are
    // late by the idle gap after it, up to a poll (27 ms) less the burst (15 ms)
    EXPECT_GT(ts.lateness90Ms(), 2.0);
    EXPECT_LT(ts.lateness90Ms(), 15.0);
    EXPECT_LT(ts.envelopeRmsMs(), 0.5);
    EXPECT_EQ(ts.samples(), 600u);
}

TEST(GnssTimeSync, TwoCorruptEarlyStampsDoNotDragTheEnvelope)
{
    Receiver rx;
    GnssTimeSync ts = calibrated(rx);
    rx.feed(ts, rx.gps0, 40.0);
    // two sentences stamped 40 ms before they could have arrived
    for (double gps : {rx.gps0 + 40.0, rx.gps0 + 42.0})
        ts.addSample(gps, static_cast<int64_t>(rx.mcuUs(gps) + (rx.latency - 0.040) * 1.0e6));
    rx.feed(ts, rx.gps0 + 43.0, 5.0);
    EXPECT_LT(std::fabs(errorUs(ts, rx, rx.gps0 + 47.9)), 1000.0);
    EXPECT_EQ(ts.restarts(), 0u);
}

TEST(GnssTimeSync, ASingleStalledPollIsSetAside)
{
    Receiver rx;
    GnssTimeSync ts = calibrated(rx);
    rx.feed(ts, rx.gps0, 30.0);
    const double gps = rx.gps0 + 30.0;
    EXPECT_FALSE(ts.addSample(gps, static_cast<int64_t>(rx.mcuUs(gps) + 3.0e6)));   // 3 s late
    rx.feed(ts, rx.gps0 + 30.1, 2.0);
    EXPECT_EQ(ts.restarts(), 0u);
    EXPECT_LT(std::fabs(errorUs(ts, rx, rx.gps0 + 32.0)), 1000.0);
}

TEST(GnssTimeSync, AStepInTheReceiversTimeRestartsTheHistory)
{
    Receiver rx;
    GnssTimeSync ts = calibrated(rx);
    rx.feed(ts, rx.gps0, 30.0);
    // the receiver re-solves its time 2 s later than before: every epoch it
    // now names is 2 s on from the one the bytes belong to
    for (int k = 0; k < 200; ++k)
    {
        const double true_gps = rx.gps0 + 30.0 + k * 0.1;
        ts.addSample(true_gps + 2.0, rx.stamp(true_gps));
    }
    EXPECT_EQ(ts.restarts(), 1u);
    ASSERT_TRUE(ts.valid());
    // the new history maps the receiver's new times to the same MCU clock
    int64_t mcu = 0;
    ASSERT_TRUE(ts.mcuAt(rx.gps0 + 49.0 + 2.0, mcu));
    EXPECT_LT(std::fabs(static_cast<double>(mcu) - rx.mcuUs(rx.gps0 + 49.0)), 1000.0);
}

TEST(GnssTimeSync, TimeRunningBackwardsRestarts)
{
    Receiver rx;
    GnssTimeSync ts = calibrated(rx);
    rx.feed(ts, rx.gps0, 20.0);
    EXPECT_FALSE(ts.addSample(rx.gps0 - 30.0, rx.stamp(rx.gps0 + 20.0)));
    EXPECT_EQ(ts.restarts(), 1u);
    EXPECT_FALSE(ts.valid());
}

// driftFitted (the mosaic driver's PPS tag check judges only once it holds):
// not while the line answers flat on its first blocks, then from the first
// fit, and not again after a restart until the new history fits its own.
TEST(GnssTimeSync, SaysWhenTheDriftIsFitted)
{
    Receiver rx;
    GnssTimeSync ts = calibrated(rx);
    EXPECT_FALSE(ts.driftFitted());
    rx.feed(ts, rx.gps0, 15.0);
    ASSERT_TRUE(ts.valid());
    EXPECT_FALSE(ts.driftFitted());   // valid, and still flat
    EXPECT_EQ(ts.driftPpm(), 0.0);
    rx.feed(ts, rx.gps0 + 15.0, 30.0);
    EXPECT_TRUE(ts.driftFitted());
    EXPECT_NEAR(ts.driftPpm(), rx.drift * 1e6, 10.0);
    EXPECT_FALSE(ts.addSample(rx.gps0 - 30.0, rx.stamp(rx.gps0 + 45.0)));   // time backwards: a restart
    EXPECT_EQ(ts.restarts(), 1u);
    EXPECT_FALSE(ts.driftFitted());
    ts.reset();
    EXPECT_FALSE(ts.driftFitted());
}

TEST(GnssTimeSync, CarriesTheDriftAcrossAGapInTheFixes)
{
    Receiver rx;
    rx.drift = 45e-6;
    GnssTimeSync ts = calibrated(rx);
    rx.feed(ts, rx.gps0, 60.0);
    // 30 s without a fix: 45 ppm would be 1.35 ms if the drift were not tracked
    EXPECT_LT(std::fabs(errorUs(ts, rx, rx.gps0 + 90.0)), 500.0);
}

TEST(GnssTimeSync, AnUncalibratedReceiverPlacesEveryEpochLateByItsLatency)
{
    Receiver rx;
    GnssTimeSync ts;                       // latency_s = 0
    rx.feed(ts, rx.gps0, 60.0);
    const double err = errorUs(ts, rx, rx.gps0 + 59.9);
    EXPECT_NEAR(err, rx.latency * 1.0e6, 500.0);
}

// ---- the line as GNSS_CLOCK_MSG logs it (GnssClockRecord.h)

namespace
{

// mcuAt from a logged record alone, as a replay computes it
int64_t mcuFromRecord(const GnssClockData& c, double gps_s)
{
    const double t = gps_s - static_cast<double>(c.gps0_s);
    const double rel = t + (c.a_s + c.drift * (t - c.t_ref_s)) - c.latency_s;
    return c.mcu0_us + static_cast<int64_t>(std::llround(rel * 1.0e6));
}

}  // namespace

TEST(GnssTimeSync, TheLoggedLinePlacesEpochsExactlyAsMcuAt)
{
    for (bool cal : {true, false})
    {
        Receiver rx;
        GnssTimeSync ts = cal ? calibrated(rx) : GnssTimeSync();
        rx.feed(ts, rx.gps0, 60.0);
        GnssTimeSync::Snapshot s;
        ts.snapshot(s);
        GnssClockData c;
        lc86::gnssClockRecord(s, 4242u, c);
        EXPECT_EQ(c.time_us, 4242u);
        EXPECT_EQ(c.flags, GCL_VALID | GCL_STARTED);
        EXPECT_EQ(c.seq, ts.lineSeq());
        EXPECT_EQ(c.samples, ts.samples());
        EXPECT_DOUBLE_EQ(c.drift * 1.0e6, ts.driftPpm());
        EXPECT_DOUBLE_EQ(c.latency_s, cal ? rx.latency : 0.0);
        EXPECT_NEAR(c.env_rms_ms, ts.envelopeRmsMs(), 1e-6);
        for (double gps : {rx.gps0 + 0.05, rx.gps0 + 1.0, rx.gps0 + 59.9, rx.gps0 + 61.0})
        {
            int64_t mcu = 0;
            ASSERT_TRUE(ts.mcuAt(gps, mcu));
            EXPECT_EQ(mcuFromRecord(c, gps), mcu) << "at GPS + " << gps - rx.gps0;
        }
    }
}

TEST(GnssTimeSync, TheLineSeqCountsEachSecondFiledAndEachRestart)
{
    Receiver rx;
    GnssTimeSync ts = calibrated(rx);
    GnssTimeSync::Snapshot s;
    ts.snapshot(s);
    EXPECT_EQ(ts.lineSeq(), 0u);
    EXPECT_FALSE(s.started);
    EXPECT_FALSE(s.valid);
    // the first sample starts the line; the rest of its second leave it
    ts.addSample(rx.gps0, rx.stamp(rx.gps0));
    EXPECT_EQ(ts.lineSeq(), 1u);
    for (int k = 1; k < 10; ++k)
        ts.addSample(rx.gps0 + 0.1 * k, rx.stamp(rx.gps0 + 0.1 * k));
    EXPECT_EQ(ts.lineSeq(), 1u);
    // each new second moves it once
    rx.feed(ts, rx.gps0 + 1.0, 29.0);
    EXPECT_EQ(ts.lineSeq(), 30u);
    ts.snapshot(s);
    EXPECT_TRUE(s.started);
    EXPECT_TRUE(s.valid);
    EXPECT_EQ(s.bins, 30);
    // a step in the receiver's time restarts the history, and the line with it
    for (int k = 0; k < 10; ++k)
    {
        const double true_gps = rx.gps0 + 30.0 + k * 0.1;
        ts.addSample(true_gps + 2.0, rx.stamp(true_gps));
    }
    EXPECT_EQ(ts.restarts(), 1u);
    ts.snapshot(s);
    EXPECT_EQ(s.restarts, 1u);
    EXPECT_FALSE(s.valid);                 // 10 s of history before it answers again
    EXPECT_EQ(s.bins, 1);
    EXPECT_GT(ts.lineSeq(), 30u);
}

// ---- exact samples: a 1PPS line (Config::exact_step_s, the mosaic-G5's)

namespace
{

// The 1PPS edges of a GPS second each, on an MCU clock 16 ppm fast (the
// V9's), each stamped a few microseconds late by the interrupt; from
// step_from_s on, the receiver's 1PPS sits step_ms later (the sky it tracks
// was delayed: on the RF rig, a transmitter underrun).
struct PpsEdges
{
    double  gps0 = 2440 * 604800.0 + 302400.0;
    double  drift = 16e-6;
    int64_t mcu0 = 50000000;
    double  step_from_s = 1e9;   // s after gps0
    double  step_ms = 0.0;

    double mcuUs(double gps) const { return mcu0 + (gps - gps0) * (1.0 + drift) * 1.0e6; }
    int64_t edge(int k) const
    {
        const double at = mcuUs(gps0 + k) + (k >= step_from_s ? step_ms * 1.0e3 : 0.0);
        return static_cast<int64_t>(std::llround(at)) + (k * 7919) % 5;   // 0-4 us of interrupt latency
    }
    void feed(GnssTimeSync& ts, int from, int to) const
    {
        for (int k = from; k < to; ++k)
            ts.addSample(gps0 + k, edge(k));
    }
    // the line's place for second k against its edge, us
    double offUs(const GnssTimeSync& ts, int k) const
    {
        int64_t mcu = 0;
        EXPECT_TRUE(ts.mcuAt(gps0 + k, mcu));
        return static_cast<double>(edge(k) - mcu);
    }
};

GnssTimeSync ppsLine()
{
    GnssTimeSync::Config cfg;
    cfg.exact_step_s = 100e-6;
    return GnssTimeSync(cfg);
}

}  // namespace

// The 10/17 bench (V9, RF rig): a 6.854 ms transmitter underrun delayed the
// sky, and the receiver's 1PPS followed it when it re-acquired. The lower
// envelope keeps the minute of seconds before: every tag placed on it was
// 6.854 ms early (the PPS tag check read -6855 us at each whole second) until
// those seconds left its window. As the LC86G's line (exact_step_s 0) it
// still does that; as a 1PPS line it moves onto the step after three edges,
// with the crystal's drift kept.
TEST(GnssTimeSync, APpsLineFollowsAStepLaterOfItsEdges)
{
    PpsEdges pps;
    pps.step_from_s = 120;
    pps.step_ms = 6.854;

    GnssTimeSync lc86;   // the envelope alone
    pps.feed(lc86, 0, 150);
    EXPECT_NEAR(pps.offUs(lc86, 150), 6854.0, 10.0);   // half a minute after the step: still the old seconds
    pps.feed(lc86, 150, 175);
    EXPECT_GT(pps.offUs(lc86, 175), 4000.0);            // its drift blocks starting to tilt it: still ms off
    EXPECT_EQ(lc86.restarts(), 0u);

    GnssTimeSync ts = ppsLine();
    pps.feed(ts, 0, 120);
    ASSERT_TRUE(ts.driftFitted());
    const double drift_before = ts.driftPpm();
    EXPECT_NEAR(drift_before, 16.0, 0.5);
    EXPECT_NEAR(pps.offUs(ts, 120), 6854.0, 20.0);     // the first edge after the step
    // the first two later edges are set aside, the line where it was
    EXPECT_FALSE(ts.addSample(pps.gps0 + 120, pps.edge(120)));
    EXPECT_FALSE(ts.addSample(pps.gps0 + 121, pps.edge(121)));
    EXPECT_NEAR(pps.offUs(ts, 122), 6854.0, 20.0);
    EXPECT_EQ(ts.steps(), 0u);
    // the third moves the history onto them
    EXPECT_TRUE(ts.addSample(pps.gps0 + 122, pps.edge(122)));
    EXPECT_EQ(ts.steps(), 1u);
    EXPECT_NEAR(ts.lastStepMs(), 6.854, 0.01);
    EXPECT_EQ(ts.restarts(), 0u);
    EXPECT_TRUE(ts.valid());
    for (int k = 123; k < 400; ++k)
    {
        EXPECT_LT(std::fabs(pps.offUs(ts, k)), 20.0) << "second " << k;
        ts.addSample(pps.gps0 + k, pps.edge(k));
    }
    EXPECT_NEAR(ts.driftPpm(), drift_before, 0.5);   // the crystal did not step
    EXPECT_EQ(ts.steps(), 1u);
}

// A step earlier: the envelope takes it after three seconds, but its drift
// blocks keep both levels for ten minutes and the drift comes out wrong. A
// 1PPS line moves the whole history instead.
TEST(GnssTimeSync, APpsLineFollowsAStepEarlierWithItsDriftKept)
{
    PpsEdges pps;
    pps.step_from_s = 120;
    pps.step_ms = -6.854;
    GnssTimeSync lc86;
    pps.feed(lc86, 0, 200);
    GnssTimeSync ts = ppsLine();
    pps.feed(ts, 0, 200);
    EXPECT_EQ(ts.steps(), 1u);
    EXPECT_NEAR(ts.lastStepMs(), -6.854, 0.01);
    EXPECT_NEAR(ts.driftPpm(), 16.0, 0.5);
    EXPECT_GT(std::fabs(lc86.driftPpm() - 16.0), 5.0);   // the envelope's blocks across the step
    for (int k = 200; k < 260; ++k)
    {
        EXPECT_LT(std::fabs(pps.offUs(ts, k)), 20.0) << "second " << k;
        ts.addSample(pps.gps0 + k, pps.edge(k));
    }
}

// What is no step: an edge stamped late (a flash write held the interrupt
// off), two in a row, or three that do not agree; and nothing before the
// drift is fitted, while a flat line parts from the edges by the crystal.
TEST(GnssTimeSync, APpsLineTakesNoLateEdgeForAStep)
{
    PpsEdges pps;
    GnssTimeSync ts = ppsLine();
    pps.feed(ts, 0, 60);
    ASSERT_TRUE(ts.driftFitted());
    // one 3 ms late, then two 3 ms late, then three late by 1, 2.5 and 4 ms
    EXPECT_FALSE(ts.addSample(pps.gps0 + 60, pps.edge(60) + 3000));
    pps.feed(ts, 61, 70);
    EXPECT_FALSE(ts.addSample(pps.gps0 + 70, pps.edge(70) + 3000));
    EXPECT_FALSE(ts.addSample(pps.gps0 + 71, pps.edge(71) + 3000));
    pps.feed(ts, 72, 80);
    EXPECT_FALSE(ts.addSample(pps.gps0 + 80, pps.edge(80) + 1000));
    EXPECT_FALSE(ts.addSample(pps.gps0 + 81, pps.edge(81) + 2500));
    EXPECT_FALSE(ts.addSample(pps.gps0 + 82, pps.edge(82) + 4000));
    pps.feed(ts, 83, 100);
    EXPECT_EQ(ts.steps(), 0u);
    EXPECT_EQ(ts.restarts(), 0u);
    EXPECT_LT(std::fabs(pps.offUs(ts, 100)), 20.0);

    // before the drift is fitted the line runs flat and the edges part from
    // it by 16 us a second: no step taken for that
    GnssTimeSync fresh = ppsLine();
    for (int k = 0; k < 40 && !fresh.driftFitted(); ++k)
        fresh.addSample(pps.gps0 + k, pps.edge(k));
    EXPECT_EQ(fresh.steps(), 0u);
}

// exact_step_s left at 0 (the LC86G): the line is the envelope it was, sample
// for sample, through the same late edges and step.
TEST(GnssTimeSync, WithoutExactStepsTheLineIsTheEnvelopeItWas)
{
    PpsEdges pps;
    pps.step_from_s = 100;
    pps.step_ms = 6.854;
    GnssTimeSync a, b;
    GnssTimeSync::Config c;
    c.exact_step_s = 0.0;
    b.setConfig(c);
    for (int k = 0; k < 200; ++k)
    {
        const int64_t e = pps.edge(k) + (k == 50 ? 3000 : 0);
        EXPECT_EQ(a.addSample(pps.gps0 + k, e), b.addSample(pps.gps0 + k, e));
        GnssTimeSync::Snapshot sa, sb;
        a.snapshot(sa);
        b.snapshot(sb);
        EXPECT_EQ(sa.a_s, sb.a_s);
        EXPECT_EQ(sa.drift, sb.drift);
    }
    EXPECT_EQ(b.steps(), 0u);
}
