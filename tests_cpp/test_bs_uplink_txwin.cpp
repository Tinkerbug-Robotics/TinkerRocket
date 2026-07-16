#include <gtest/gtest.h>

#include "bs_uplink_txwin.h"

using namespace bs_uplink_txwin;

// The #506 fix: the radio is half-duplex, so every blind uplink retry is a deaf
// window. The 2026-07-14 bench run measured the cost exactly — the ONLY downlink
// loss in a clean 92 s flight (3 packets; zero CRC/length/net-id drops) fell
// inside the cmd5+cmd6 blind-TX burst.
//
// It is arithmetic, not luck. At SF8/BW250/CR4-5 a downlink packet is ~82 ms on
// air, while the gaps between 100 ms-spaced retries are only ~49 ms — an 82 ms
// packet cannot fit in a 49 ms gap, so a burst loses essentially EVERY packet
// that arrives during it. The fix is to transmit inside the quiet stretch between
// the rocket's ~500 ms telemetry packets.

namespace {

// Measured on the bench: 502 ms median inter-packet interval.
constexpr uint32_t kBenchPeriod = 502;

Params benchParams(uint32_t tx_airtime = 52) {
    Params p;
    p.period_ms     = kBenchPeriod;
    p.tx_airtime_ms = tx_airtime;
    p.rx_reserve_ms = 140;
    p.link_stale_ms = 2000;
    p.max_defer_ms  = 1500;
    return p;
}

}  // namespace

// ---- Time-on-air: the numbers the whole fix rests on ----

// If these are wrong the window is wrong, so pin them against the Semtech formula.
TEST(BsUplinkTxwin, TimeOnAirMatchesTheBenchRadioConfig) {
    // SF8, BW250 kHz, CR 4/5 — the configured link (log: "915.0 MHz SF8 BW250 CR5").
    EXPECT_NEAR(timeOnAirMs(46, 8, 250.0f, 5), 82u, 2u);   // downlink telemetry
    EXPECT_NEAR(timeOnAirMs(22, 8, 250.0f, 5), 51u, 2u);   // uplink sim-config (6+16)
    EXPECT_NEAR(timeOnAirMs(6,  8, 250.0f, 5), 31u, 2u);   // uplink sim-start / heartbeat
}

// The core arithmetic of the bug: the retry gap is smaller than a downlink packet.
TEST(BsUplinkTxwin, ARetryGapCannotFitADownlinkPacket) {
    const uint32_t downlink = timeOnAirMs(46, 8, 250.0f, 5);
    const uint32_t uplink   = timeOnAirMs(22, 8, 250.0f, 5);
    const uint32_t gap      = 100 - uplink;   // 100 ms retry interval
    EXPECT_LT(gap, downlink)
        << "if this ever passes, blind retries would be survivable and #506 is moot";
}

TEST(BsUplinkTxwin, TimeOnAirGrowsWithPayloadAndSpreadingFactor) {
    EXPECT_GT(timeOnAirMs(40, 8, 250.0f, 5), timeOnAirMs(6, 8, 250.0f, 5));
    EXPECT_GT(timeOnAirMs(22, 10, 250.0f, 5), timeOnAirMs(22, 8, 250.0f, 5));
    EXPECT_GT(timeOnAirMs(22, 8, 125.0f, 5), timeOnAirMs(22, 8, 250.0f, 5));  // narrower BW
    EXPECT_GT(timeOnAirMs(22, 8, 250.0f, 8), timeOnAirMs(22, 8, 250.0f, 5));  // heavier FEC
}

TEST(BsUplinkTxwin, TimeOnAirHandlesDegenerateInputs) {
    EXPECT_GT(timeOnAirMs(0, 8, 250.0f, 5), 0u);        // header+preamble only
    EXPECT_GT(timeOnAirMs(22, 99, 250.0f, 99), 0u);     // clamped, no div-by-zero
    EXPECT_GT(timeOnAirMs(22, 8, 0.0f, 5), 0u);         // bad BW falls back
}

TEST(BsUplinkTxwin, DelegatesToTheSharedLoraTimeOnAir) {
    // #150 moved the formula into RocketComputerTypes.h so the hop-dwell
    // math and the TX-window math cannot drift apart.  Pin the delegation
    // (this call site keeps its historical preamble-8 default).
    const size_t lens[] = {0, 6, 22, 46, 66};
    for (uint8_t sf = 7; sf <= 11; sf++) {
        for (size_t len : lens) {
            EXPECT_EQ(timeOnAirMs(len, sf, 250.0f, 5),
                      loraTimeOnAirMs(len, sf, 250.0f, 5, 8))
                << "sf=" << (int)sf << " len=" << len;
        }
    }
}

// ---- The window ----

// Right after a packet lands, the air is ours.
TEST(BsUplinkTxwin, TransmitsImmediatelyAfterAPacketArrives) {
    const auto p = benchParams();
    EXPECT_TRUE(mayStartTx(/*now=*/1000, /*last_rx=*/1000, /*deferred=*/0, p));
    EXPECT_TRUE(mayStartTx(1100, 1000, 100, p));
    EXPECT_TRUE(mayStartTx(1200, 1000, 200, p));
    EXPECT_TRUE(mayStartTx(1300, 1000, 300, p));
}

// ...and we shut up before the next one is due.
TEST(BsUplinkTxwin, StaysQuietWhenTheNextDownlinkPacketIsDue) {
    const auto p = benchParams();
    // window_end = 502 - 140 - 52 = 310 ms
    EXPECT_TRUE (mayStartTx(1310, 1000, 310, p));
    EXPECT_FALSE(mayStartTx(1311, 1000, 311, p));
    EXPECT_FALSE(mayStartTx(1450, 1000, 450, p));  // packet inbound about now
}

// A transmission must be judged by when it FINISHES, not when it starts — a long
// channel-set push has to start earlier than a short sim-start.
TEST(BsUplinkTxwin, ABiggerPacketGetsASmallerWindow) {
    const auto small = benchParams(timeOnAirMs(6,  8, 250.0f, 5));   // ~31 ms
    const auto big   = benchParams(timeOnAirMs(33, 8, 250.0f, 5));   // ~66 ms

    // At age 320 ms the short packet still fits; the long one would run into the
    // inbound downlink, so it waits.
    EXPECT_TRUE (mayStartTx(1320, 1000, 320, small));
    EXPECT_FALSE(mayStartTx(1320, 1000, 320, big));
}

TEST(BsUplinkTxwin, WindowTracksAFasterTelemetryCadence) {
    auto p = benchParams();
    p.period_ms = 250;   // rocket doubled its rate
    // window_end = 250 - 140 - 52 = 58 ms — much tighter, but still exists.
    EXPECT_TRUE (mayStartTx(1050, 1000, 50, p));
    EXPECT_FALSE(mayStartTx(1100, 1000, 100, p));
}

// ---- Liveness: the uplink is BLIND and carries pyro/camera/logging commands.
//      A command that never goes out is far worse than a dropped telemetry row. ----

TEST(BsUplinkTxwin, TransmitsFreelyBeforeWeHaveEverHeardTheRocket) {
    // last_rx == 0: rendezvous / recovery must be able to shout into the void.
    EXPECT_TRUE(mayStartTx(50000, /*last_rx=*/0, /*deferred=*/9999, benchParams()));
}

TEST(BsUplinkTxwin, TransmitsFreelyOnceTheLinkGoesStale) {
    const auto p = benchParams();
    // 2 s of silence: there is no downlink to protect, and this is exactly when we
    // most need to transmit (silence recovery pushing the rocket home).
    EXPECT_TRUE(mayStartTx(1000 + 2000, 1000, 0, p));
    EXPECT_TRUE(mayStartTx(1000 + 5000, 1000, 0, p));
}

TEST(BsUplinkTxwin, BackstopFiresRatherThanStarveACommand) {
    const auto p = benchParams();
    // Deep inside the downlink slot — normally we'd hold off...
    EXPECT_FALSE(mayStartTx(1450, 1000, /*deferred=*/0, p));
    // ...but not forever. After max_defer_ms the command goes out regardless.
    EXPECT_TRUE(mayStartTx(1450, 1000, /*deferred=*/1500, p));
}

TEST(BsUplinkTxwin, NeverDeadlocksWhenNoWindowCanExist) {
    auto p = benchParams();
    p.period_ms = 150;    // reserve(140) + airtime(52) > period: no gap is possible
    // Must fall through to "transmit", not block every pass forever.
    EXPECT_TRUE(mayStartTx(1000, 1000, 0, p));
    EXPECT_TRUE(mayStartTx(1100, 1000, 100, p));
    EXPECT_TRUE(mayStartTx(1140, 1000, 140, p));
}

// ---- Cadence learning ----

TEST(BsUplinkTxwin, LearnsTheRocketsCadence) {
    RxCadence c;
    EXPECT_FALSE(c.valid());
    EXPECT_EQ(c.periodMs(), kDefaultPeriodMs) << "falls back to the configured rate";

    uint32_t t = 1000;
    for (int i = 0; i < 12; i++) { c.onPacket(t); t += 502; }
    EXPECT_TRUE(c.valid());
    EXPECT_NEAR(c.periodMs(), 502u, 5u);
    EXPECT_EQ(c.lastMs(), t - 502);
}

TEST(BsUplinkTxwin, NeedsTwoIntervalsBeforeItIsTrusted) {
    RxCadence c;
    c.onPacket(1000);
    EXPECT_FALSE(c.valid()) << "one packet is not an interval";
    c.onPacket(1500);
    EXPECT_FALSE(c.valid()) << "one interval is not a cadence";
    c.onPacket(2000);
    EXPECT_TRUE(c.valid());
}

// The bench log contained a duplicate packet logged 8 ms apart — that is not a
// cadence, and must not drag the estimate toward zero.
TEST(BsUplinkTxwin, IgnoresDuplicatePacketsAndLongDropouts) {
    RxCadence c;
    uint32_t t = 1000;
    for (int i = 0; i < 8; i++) { c.onPacket(t); t += 500; }
    const uint32_t settled = c.periodMs();
    ASSERT_NEAR(settled, 500u, 5u);

    c.onPacket(t);          // normal packet
    c.onPacket(t + 8);      // duplicate, 8 ms later — ignore
    EXPECT_NEAR(c.periodMs(), settled, 5u);

    c.onPacket(t + 8 + 9000);   // long dropout — not a cadence either
    EXPECT_NEAR(c.periodMs(), settled, 5u);
}

TEST(BsUplinkTxwin, AdaptsWhenTheCadenceGenuinelyChanges) {
    RxCadence c;
    uint32_t t = 1000;
    for (int i = 0; i < 10; i++) { c.onPacket(t); t += 500; }
    ASSERT_NEAR(c.periodMs(), 500u, 5u);

    for (int i = 0; i < 30; i++) { c.onPacket(t); t += 250; }
    EXPECT_NEAR(c.periodMs(), 250u, 10u) << "must follow a real rate change";
}

TEST(BsUplinkTxwin, ResetClearsTheEstimate) {
    RxCadence c;
    uint32_t t = 1000;
    for (int i = 0; i < 5; i++) { c.onPacket(t); t += 500; }
    ASSERT_TRUE(c.valid());
    c.reset();
    EXPECT_FALSE(c.valid());
    EXPECT_EQ(c.lastMs(), 0u);
    EXPECT_EQ(c.periodMs(), kDefaultPeriodMs);
}

// ---- End to end: replay the bench burst and show no packet is stepped on ----

// 8 retries at 100 ms against the measured 502 ms cadence. Every transmission
// must land in a gap, and the command must still finish promptly.
TEST(BsUplinkTxwin, ReplayOfTheBenchBurstStepsOnNoDownlinkPacket) {
    const uint32_t period   = 502;
    const uint32_t dl_air   = timeOnAirMs(46, 8, 250.0f, 5);   // ~82 ms
    const uint32_t ul_air   = timeOnAirMs(22, 8, 250.0f, 5);   // ~51 ms
    Params p = benchParams(ul_air);
    p.period_ms = period;

    // The rocket's packet N is on the air over [tx_start, tx_start + dl_air];
    // we timestamp it at the END, so last_rx = tx_start + dl_air.
    uint32_t last_rx = 10000;
    uint32_t now     = last_rx;
    uint32_t last_tx = 0;
    int retries_left = 8;
    int collisions   = 0;
    int sent         = 0;

    // Simulate 3 telemetry cycles, 1 ms steps.
    for (uint32_t step = 0; step < 3 * period && retries_left > 0; step++, now++) {
        // Next downlink packet occupies [last_rx + period - dl_air, last_rx + period].
        const uint32_t dl_start = last_rx + period - dl_air;
        const uint32_t dl_end   = last_rx + period;
        if (now == dl_end) { last_rx = now; continue; }   // packet received

        if (last_tx != 0 && (now - last_tx) < 100) continue;   // retry interval
        if (!mayStartTx(now, last_rx, /*deferred=*/0, p)) continue;

        // We transmit over [now, now + ul_air]. Does it overlap the inbound packet?
        if (now + ul_air >= dl_start && now <= dl_end) collisions++;
        last_tx = now;
        retries_left--;
        sent++;
    }

    EXPECT_EQ(collisions, 0) << "an uplink retry landed on top of a downlink packet";
    EXPECT_EQ(retries_left, 0) << "the command must still get all 8 retries out";
    EXPECT_EQ(sent, 8);
}
