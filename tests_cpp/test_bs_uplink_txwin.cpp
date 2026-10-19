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

// ===========================================================================
// Multi-rocket gap gate (three-rocket support)
// ===========================================================================
// Three rockets on one channel, slotted 500/3 ms apart: each FAST frame is
// 102 ms on air and leaves a hole of ~54 ms before the next slot's guarded
// start. Short commands fit those holes; a long one falls back to the focused
// gate after its budget.

namespace {

constexpr uint32_t kFast = 102;   // pinned below
constexpr uint32_t kChan = 0x915250;

// A slotted air: rocket i's frames start at base + offset_i + k * 500.
struct Air
{
    uint32_t base = 100000;
    uint32_t offsets[3] = {0, 167, 333};
    int      n = 3;

    // The last frame of rocket i that had finished by `now` (what the BS heard).
    RocketPhase heard(int i, uint32_t now) const
    {
        RocketPhase r;
        const uint32_t first = base + offsets[i];
        if (now < first + kFast) return r;
        const uint32_t k = (now - first - kFast) / 500;
        r.valid    = true;
        r.start_ms = first + k * 500;
        r.heard_ms = r.start_ms + kFast;
        r.chan_key = kChan;
        return r;
    }
    void phases(uint32_t now, RocketPhase* out) const
    {
        for (int i = 0; i < n; i++) out[i] = heard(i, now);
    }
    // Does [t, t + air] overlap any rocket's frame?
    bool hits(uint32_t t, uint32_t air) const
    {
        for (int i = 0; i < n; i++) {
            for (int k = -1; k < 400; k++) {
                const int64_t s = (int64_t)base + offsets[i] + (int64_t)k * 500;
                if ((int64_t)t < s + kFast && (int64_t)t + air > s) return true;
            }
        }
        return false;
    }
};

uint32_t pkt(size_t len) { return timeOnAirMs(len, 8, 250.0f, 5, LORA_TELEM_PREAMBLE_SYMS); }

}  // namespace

TEST(BsUplinkGapGate, PreambleTwelveAirtimes) {
    // The telemetry frames as the base station must reckon them: 12-symbol
    // preamble (LORA_PREAMBLE_LEN in both config.h). The old preamble-8 default
    // under-counts every uplink by ~4 ms.
    EXPECT_EQ(timeOnAirMs(SIZE_OF_LORA_FAST, 8, 250.0f, 5, LORA_TELEM_PREAMBLE_SYMS), 102u);
    EXPECT_EQ(timeOnAirMs(SIZE_OF_LORA_SLOW, 8, 250.0f, 5, LORA_TELEM_PREAMBLE_SYMS), 56u);
    EXPECT_EQ(kFast, 102u);
    // Uplink packets (6 B header + payload).
    EXPECT_EQ(pkt(6),  35u);   // heartbeat / sim start
    EXPECT_EQ(pkt(10), 40u);   // slot adjust (4 B payload)
    EXPECT_EQ(pkt(17), 50u);   // cmd-10 relay (11 B payload)
    EXPECT_EQ(pkt(33), 71u);   // cmd-15 channel set at BW 250 (27 B payload)
    EXPECT_GT(pkt(10), timeOnAirMs(10, 8, 250.0f, 5));   // preamble 8 is shorter
}

TEST(BsUplinkGapGate, ShortCommandsFitEverySlottedHole) {
    Air air;
    RocketPhase ph[3];
    for (uint32_t len : {6u, 10u}) {
        const uint32_t a = pkt(len);
        // Over one full period after everyone has been heard, the allowed starts
        // are exactly [frame end, next guarded start - airtime] after each frame.
        int holes = 0;
        bool prev = false;
        for (uint32_t now = air.base + 1000; now < air.base + 1500; now++) {
            air.phases(now, ph);
            const bool ok = gapClear(now, a, kFast, ph, 3, kChan);
            if (ok) {
                EXPECT_FALSE(air.hits(now, a)) << "len=" << len << " t=" << (now - air.base);
            }
            if (ok && !prev) holes++;
            prev = ok;
        }
        EXPECT_EQ(holes, 3) << "len=" << len << ": one usable hole per slot";
    }
}

TEST(BsUplinkGapGate, HoleEdgesKeepTheMarginBeforeTheNextFrame) {
    Air air;
    RocketPhase ph[3];
    const uint32_t a = pkt(10);   // 40 ms
    // Slot 0's frame (base + 1000) ends at +102; slot 1 starts at +167.
    const uint32_t f0 = air.base + 1000;
    air.phases(f0 + 101, ph);
    EXPECT_FALSE(gapClear(f0 + 101, a, kFast, ph, 3, kChan));   // still on air
    air.phases(f0 + 102, ph);
    EXPECT_TRUE(gapClear(f0 + 102, a, kFast, ph, 3, kChan));
    // Latest start: 167 - 10 margin - 40 = +117.
    air.phases(f0 + 117, ph);
    EXPECT_TRUE(gapClear(f0 + 117, a, kFast, ph, 3, kChan));
    air.phases(f0 + 118, ph);
    EXPECT_FALSE(gapClear(f0 + 118, a, kFast, ph, 3, kChan));
}

TEST(BsUplinkGapGate, LongCommandNeverFitsASlottedHole) {
    Air air;
    RocketPhase ph[3];
    const uint32_t a = pkt(33);   // 71 ms > the 54 ms hole
    for (uint32_t now = air.base + 1000; now < air.base + 1500; now++) {
        air.phases(now, ph);
        EXPECT_FALSE(gapClear(now, a, kFast, ph, 3, kChan)) << "t=" << (now - air.base);
    }
}

TEST(BsUplinkGapGate, StaleOrOffChannelRocketsAreIgnored) {
    RocketPhase r;
    r.valid = true;
    r.start_ms = 10000;
    r.heard_ms = 10102;
    r.chan_key = kChan;
    const uint32_t a = pkt(10);
    // 10500 is that rocket's next predicted start: blocked while fresh.
    EXPECT_FALSE(gapClear(10480, a, kFast, &r, 1, kChan));
    // Heard 2 s ago or more: no longer on the air as far as the gate knows.
    EXPECT_TRUE(gapClear(10102 + kGapFreshMs, a, kFast, &r, 1, kChan));
    // Heard on another channel / modulation: not our air.
    EXPECT_TRUE(gapClear(10480, a, kFast, &r, 1, kChan + 1));
    // Never measured.
    RocketPhase none;
    EXPECT_TRUE(gapClear(10480, a, kFast, &none, 1, kChan));
}

TEST(BsUplinkGapGate, PredictsWithTheFixedPeriodAcrossMissedFrames) {
    // The last frame heard was nearly 3 periods ago (two lost since): the gate
    // still expects the rocket at start + k * 500.
    RocketPhase r;
    r.valid = true;
    r.start_ms = 20000;
    r.heard_ms = 20102;
    r.chan_key = kChan;
    const uint32_t a = pkt(10);   // 40 ms
    // Next predicted start 21500, guarded from 21490: the latest start is 21450.
    EXPECT_TRUE (gapClear(21450, a, kFast, &r, 1, kChan));
    EXPECT_FALSE(gapClear(21451, a, kFast, &r, 1, kChan));
    EXPECT_FALSE(gapClear(21550, a, kFast, &r, 1, kChan));   // on air (FAST assumed)
    EXPECT_FALSE(gapClear(21601, a, kFast, &r, 1, kChan));
    EXPECT_TRUE (gapClear(21602, a, kFast, &r, 1, kChan));   // after it
}

TEST(BsUplinkGapGate, BudgetChargesOnlyWhatTheFocusedGateWouldHaveSent) {
    GapBudget b;
    GateMode m;
    uint32_t t = 5000;
    // Held by both gates: free (today's gate would have waited too).
    for (int i = 0; i < 1000; i++, t++) {
        EXPECT_FALSE(gateDecide(b, t, true, /*gap*/false, /*focused*/false, m));
    }
    EXPECT_EQ(b.charged_ms, 0u);
    EXPECT_FALSE(b.exhausted);
    // Held by the gap gate only: charged, and exhausted at the budget.
    int passes = 0;
    while (!gateDecide(b, t, true, false, true, m)) { t++; passes++; ASSERT_LT(passes, 1000); }
    EXPECT_TRUE(b.exhausted);
    EXPECT_EQ(m, GateMode::Focused);
    EXPECT_GE(b.charged_ms, kGapBudgetMs);
    EXPECT_LE(b.charged_ms, kGapBudgetMs + 1);
    // From now on the focused gate decides, even when a gap opens.
    EXPECT_FALSE(gateDecide(b, t + 1, true, /*gap*/true, /*focused*/false, m));
    EXPECT_EQ(m, GateMode::Focused);
}

TEST(BsUplinkGapGate, NotApplicableUsesTheFocusedGate) {
    // Following a hop: one rocket, one schedule — today's gate, untouched.
    GapBudget b;
    GateMode m;
    EXPECT_TRUE(gateDecide(b, 100, /*applicable*/false, /*gap*/false, /*focused*/true, m));
    EXPECT_EQ(m, GateMode::Focused);
    EXPECT_FALSE(gateDecide(b, 101, false, /*gap*/true, /*focused*/false, m));
    EXPECT_EQ(b.charged_ms, 0u);
}

namespace {

struct TrainResult { int sent = 0; int gap_sends = 0; int hits = 0; uint32_t duration_ms = 0; bool fell_back = false; };

// One command of `retries` blind transmissions through serviceUplink's order of
// checks: retry interval, then gateDecide over the gap gate and the focused
// gate (rocket 0 focused, its cadence learned at 500 ms).
TrainResult runTrain(const Air& air, uint32_t len, int retries, uint32_t t0)
{
    TrainResult res;
    GapBudget b;
    RocketPhase ph[3];
    const uint32_t a = pkt(len);
    Params p;
    p.period_ms     = 500;
    p.tx_airtime_ms = a;
    p.rx_reserve_ms = kFast + 60;
    uint32_t last_tx = 0, defer_start = 0;
    int left = retries;
    for (uint32_t now = t0; left > 0 && now < t0 + 20000; now++) {
        if (last_tx != 0 && (now - last_tx) < 100) continue;
        air.phases(now, ph);
        const uint32_t focus_rx = ph[0].valid ? ph[0].heard_ms : 0;
        if (defer_start == 0) defer_start = now;
        const bool focused = mayStartTx(now, focus_rx, now - defer_start, p);
        const bool gap = gapClear(now, a, kFast, ph, air.n, kChan);
        GateMode m;
        if (!gateDecide(b, now, true, gap, focused, m)) continue;
        if (air.hits(now, a)) res.hits++;
        if (m == GateMode::Gap) res.gap_sends++;
        res.sent++;
        left--;
        last_tx = now;
        defer_start = 0;
        res.duration_ms = now - t0 + a;
    }
    res.fell_back = b.exhausted;
    return res;
}

}  // namespace

TEST(BsUplinkGapGate, SlottedTrainOfShortCommandsHitsNoFrame) {
    Air air;
    for (uint32_t len : {6u, 10u, 17u}) {
        for (uint32_t phase = 0; phase < 500; phase += 37) {
            const TrainResult r = runTrain(air, len, 8, air.base + 2000 + phase);
            EXPECT_EQ(r.sent, 8) << "len=" << len;
            EXPECT_EQ(r.hits, 0) << "len=" << len << " phase=" << phase;
            EXPECT_EQ(r.gap_sends, 8) << "len=" << len << ": never needed the fallback";
            EXPECT_FALSE(r.fell_back);
            // One retry per slot: 8 retries in under 1.5 s, inside the 3 s
            // cmd-10 relay bound with room to spare.
            EXPECT_LT(r.duration_ms, 1500u) << "len=" << len << " phase=" << phase;
        }
    }
}

TEST(BsUplinkGapGate, LongCommandFallsBackAfterTheBudget) {
    Air air;
    for (uint32_t phase = 0; phase < 500; phase += 37) {
        const uint32_t t0 = air.base + 2000 + phase;
        const TrainResult r = runTrain(air, 33, 8, t0);
        EXPECT_EQ(r.sent, 8);
        EXPECT_TRUE(r.fell_back);
        EXPECT_EQ(r.gap_sends, 0);
        // What the gate added over today's focused-only behaviour is bounded
        // by the budget (plus at most one focused window it was waiting on).
        const TrainResult today = [&] {
            TrainResult x;
            RocketPhase ph[3];
            const uint32_t a = pkt(33);
            Params p;
            p.period_ms = 500; p.tx_airtime_ms = a; p.rx_reserve_ms = kFast + 60;
            uint32_t last_tx = 0, defer_start = 0;
            int left = 8;
            for (uint32_t now = t0; left > 0; now++) {
                if (last_tx != 0 && (now - last_tx) < 100) continue;
                air.phases(now, ph);
                if (defer_start == 0) defer_start = now;
                if (!mayStartTx(now, ph[0].heard_ms, now - defer_start, p)) continue;
                left--; last_tx = now; defer_start = 0; x.duration_ms = now - t0 + a;
            }
            return x;
        }();
        EXPECT_LE(r.duration_ms, today.duration_ms + kGapBudgetMs + 500)
            << "phase=" << phase;
        EXPECT_LT(r.duration_ms, 3000u) << "phase=" << phase << ": TXN_MAX_RELAY_MS";
    }
}

TEST(BsUplinkGapGate, NoRocketsHeardMeansNoGate) {
    RocketPhase ph[3];
    EXPECT_TRUE(gapClear(1000, pkt(33), kFast, ph, 3, kChan));
}
