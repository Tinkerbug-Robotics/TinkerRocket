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
