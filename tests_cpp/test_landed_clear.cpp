// #1779: the FC's LANDED snapshot clear reaches the OC, or END_FLIGHT does
// its job.
//
// The FC tries the clear every 20 ms until it is in the I2S queue, queues
// END_FLIGHT behind it, and sends the clear again every second while LANDED
// (landed_sends_policy.h). The OC, given END_FLIGHT while
// its cache still reads INFLIGHT, marks the cache and its RTC copy LANDED
// (snapshot_end_flight_policy.h). Either way, an FC reset after landing is
// not handed the flight.
//
// The landing tests run LANDED passes at 1 kHz against a model of the I2S
// queue (full for a while, then drained) and of a link that can lose frames,
// into a model of the OC's two stores.

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <deque>
#include <string>
#include <utility>
#include <vector>

#include "flight_snapshot_answer.h"
#include "landed_sends_policy.h"
#include "snapshot_end_flight_policy.h"
#include "snapshot_rtc_copy.h"

namespace {

constexpr size_t kN = flight_snapshot_answer::kFrameLen;
using Frame = std::array<uint8_t, kN>;
using Copy = SnapshotRtcCopy::Copy<kN>;

Frame frameOf(uint8_t state, uint32_t elapsed_ms)
{
    FlightSnapshotData s = {};
    s.magic             = FlightSnapshotData::MAGIC;
    s.version           = FlightSnapshotData::VERSION;
    s.rocket_state      = state;
    s.flight_elapsed_ms = elapsed_ms;
    s.max_alt_m         = 812.5f;
    Frame f = {};
    f[0] = 0xAA; f[1] = 0x55; f[2] = 0xAA; f[3] = 0x55;   // SOF
    f[4] = SNAPSHOT_MSG;
    f[5] = (uint8_t)sizeof(FlightSnapshotData);
    memcpy(&f[6], &s, sizeof(s));
    f[kN - 2] = 0x5C; f[kN - 1] = 0x1D;                    // the link CRC, as sent
    return f;
}

uint8_t stateOf(const uint8_t* frame)
{
    FlightSnapshotData s = {};
    memcpy(&s, frame + SnapshotEndFlight::kPayloadOffset, sizeof(s));
    return s.rocket_state;
}

// main.cpp's snapshotServable, as far as these frames go: a fresh INFLIGHT
// snapshot. The age and flight-time bounds are not in play here.
bool servable(const uint8_t* frame)
{
    FlightSnapshotData s = {};
    memcpy(&s, frame + SnapshotEndFlight::kPayloadOffset, sizeof(s));
    return s.magic == FlightSnapshotData::MAGIC && s.rocket_state == (uint8_t)INFLIGHT;
}

// ---------------------------------------------------------------------------
// The FC: the two LANDED sends
// ---------------------------------------------------------------------------

using LandedSends::kRepeatMs;
using LandedSends::kRetryMs;

TEST(LandedSends, BothGoOutOnTheFirstPassThenOnlyTheClearRepeats)
{
    LandedSends::State s;
    std::vector<std::pair<uint32_t, std::string>> sent;
    for (uint32_t t = 0; t <= 2500; ++t)
        LandedSends::service(s, 5000 + t, [&] { sent.push_back({t, "clear"}); return true; },
                             [&] { sent.push_back({t, "end"}); return true; });
    EXPECT_EQ(sent, (std::vector<std::pair<uint32_t, std::string>>{
                        {0, "clear"}, {0, "end"}, {kRepeatMs, "clear"}, {2 * kRepeatMs, "clear"}}));
}

TEST(LandedSends, AClearThatDidNotGoInIsTriedEvery20Ms)
{
    LandedSends::State s;
    std::vector<uint32_t> tries;
    int end_tries = 0;
    for (uint32_t t = 0; t <= 100; ++t)
        LandedSends::service(s, t, [&] { tries.push_back(t); return t >= 60; },
                             [&] { ++end_tries; return true; });
    EXPECT_EQ(tries, (std::vector<uint32_t>{0, 20, 40, 60}));
    EXPECT_EQ(end_tries, 1) << "once, behind the clear";
}

TEST(LandedSends, EndFlightWaitsForTheClearAndIsRetriedUntilItGoesIn)
{
    LandedSends::State s;
    std::vector<std::string> order;
    int end_fails = 2;
    for (uint32_t t = 0; t <= 45; ++t)
        LandedSends::service(s, t, [&] { order.push_back("clear"); return t >= 40; },
                             [&] { order.push_back("end"); return end_fails-- <= 0; });
    EXPECT_EQ(order, (std::vector<std::string>{"clear", "clear", "clear", "end", "end", "end"}))
        << "never ahead of the clear; then on every pass until it goes in";
    EXPECT_TRUE(s.end_flight_sent);
}

TEST(LandedSends, AQueueThatNeverDrainsCostsOneBuildEvery20Ms)
{
    LandedSends::State s;
    int clear_tries = 0, end_tries = 0;
    for (uint32_t t = 0; t < 10000; ++t)
        LandedSends::service(s, t, [&] { ++clear_tries; return false; },
                             [&] { ++end_tries; return false; });
    EXPECT_EQ(clear_tries, 10000 / (int)kRetryMs);
    EXPECT_EQ(end_tries, 0);
}

TEST(LandedSends, TheTimingHoldsAcrossTheMillisecondWrap)
{
    LandedSends::State s;
    std::vector<uint32_t> tries;
    const uint32_t t0 = 0xFFFFFFFFu - 30;
    for (uint32_t k = 0; k <= 50; ++k)
        LandedSends::service(s, t0 + k, [&] { tries.push_back(k); return false; }, [] { return true; });
    EXPECT_EQ(tries, (std::vector<uint32_t>{0, 20, 40}));
}

TEST(LandedSends, AFreshStateSendsBothAgain)
{
    // the restore, a sim's reset and a launch start the latch over
    LandedSends::State s;
    LandedSends::service(s, 100, [] { return true; }, [] { return true; });
    s = LandedSends::State{};
    int n = 0;
    LandedSends::service(s, 101, [&] { ++n; return true; }, [&] { ++n; return true; });
    EXPECT_EQ(n, 2) << "at once, not a repeat period later";
}

// ---------------------------------------------------------------------------
// The OC: END_FLIGHT and a snapshot still INFLIGHT
// ---------------------------------------------------------------------------

TEST(SnapshotEndFlight, AnInflightFrameIsMarkedLandedAndNothingElseChanges)
{
    Frame f = frameOf(INFLIGHT, 41200);
    const Frame before = f;
    ASSERT_TRUE(servable(f.data()));
    EXPECT_TRUE(SnapshotEndFlight::markLanded(f.data(), kN));
    EXPECT_EQ(stateOf(f.data()), (uint8_t)LANDED);
    EXPECT_FALSE(servable(f.data()));
    const size_t at = SnapshotEndFlight::kPayloadOffset + offsetof(FlightSnapshotData, rocket_state);
    for (size_t i = 0; i < kN; ++i)
        if (i != at) EXPECT_EQ(f[i], before[i]) << "byte " << i;
    EXPECT_FALSE(SnapshotEndFlight::markLanded(f.data(), kN)) << "once";
}

TEST(SnapshotEndFlight, ADeliveredClearAndOtherFramesAreLeftAlone)
{
    for (uint8_t st : {(uint8_t)LANDED, (uint8_t)PRELAUNCH, (uint8_t)READY})
    {
        Frame f = frameOf(st, 41200);
        const Frame before = f;
        EXPECT_FALSE(SnapshotEndFlight::markLanded(f.data(), kN)) << (int)st;
        EXPECT_EQ(f, before);
    }
    // the OC's own "no flight" answer, a zeroed slot, a short buffer
    Frame nf = frameOf(LANDED, 0);
    const uint32_t no_flight = flight_snapshot_answer::kNoFlightMagic;
    memcpy(&nf[SnapshotEndFlight::kPayloadOffset + offsetof(FlightSnapshotData, magic)], &no_flight, 4);
    nf[SnapshotEndFlight::kPayloadOffset + offsetof(FlightSnapshotData, rocket_state)] = INFLIGHT;
    EXPECT_FALSE(SnapshotEndFlight::markLanded(nf.data(), kN)) << "not a snapshot";
    Frame zero = {};
    EXPECT_FALSE(SnapshotEndFlight::markLanded(zero.data(), kN));
    Frame f = frameOf(INFLIGHT, 41200);
    EXPECT_FALSE(SnapshotEndFlight::markLanded(f.data(), kN - 3));
    EXPECT_EQ(stateOf(f.data()), (uint8_t)INFLIGHT);
}

TEST(SnapshotEndFlight, TheRtcCopyIsMarkedAndStaysIntactSoALaterBootSaysOver)
{
    Copy c;
    const Frame f = frameOf(INFLIGHT, 41200);
    SnapshotRtcCopy::store(c, f.data(), 9'000'000ull);
    ASSERT_TRUE(SnapshotRtcCopy::intact(c));
    EXPECT_TRUE(SnapshotEndFlight::markLanded(c));
    EXPECT_TRUE(SnapshotRtcCopy::intact(c)) << "its CRC follows the change";
    EXPECT_EQ(c.stamp_us, 9'000'000ull) << "the stamp is the last frame's";
    EXPECT_EQ(stateOf(c.frame), (uint8_t)LANDED);
    // a later boot of the OC resuming a flight: intact, aged, not servable
    uint32_t age = 0;
    ASSERT_TRUE(SnapshotRtcCopy::ageMs(c, 9'500'000ull, age));
    EXPECT_EQ(SnapshotRtcCopy::verdict(true, true, age, 1'800'000u, servable(c.frame)),
              SnapshotRtcCopy::Verdict::Over);
    EXPECT_FALSE(SnapshotEndFlight::markLanded(c)) << "once";
}

TEST(SnapshotEndFlight, ACopyThatIsNotIntactIsLeftAlone)
{
    Copy c;
    const Frame f = frameOf(INFLIGHT, 41200);
    SnapshotRtcCopy::store(c, f.data(), 1000);
    c.frame[40] ^= 0x10;   // torn
    EXPECT_FALSE(SnapshotEndFlight::markLanded(c));
    EXPECT_FALSE(SnapshotRtcCopy::intact(c)) << "not made intact by the mark";
    EXPECT_EQ(stateOf(c.frame), (uint8_t)INFLIGHT);
    Copy retired;
    SnapshotRtcCopy::store(retired, f.data(), 1000);
    SnapshotRtcCopy::retire(retired);
    EXPECT_FALSE(SnapshotEndFlight::markLanded(retired));
    EXPECT_FALSE(SnapshotRtcCopy::intact(retired));
}

// ---------------------------------------------------------------------------
// Landing: the FC's passes, the I2S queue, the OC's stores
// ---------------------------------------------------------------------------

enum class Msg { Clear, EndFlight };

struct Link
{
    size_t capacity = 512;
    size_t held = 0;            // entries already in the queue, the sender held off
    std::deque<Msg> queue;      // ours, behind them
    bool enqueue(Msg m)
    {
        if (held + queue.size() >= capacity) return false;
        queue.push_back(m);
        return true;
    }
};

struct Oc
{
    Frame cache = frameOf(INFLIGHT, 41200);   // the flight's last frame
    Copy rtc;
    Oc() { SnapshotRtcCopy::store(rtc, cache.data(), 7'000'000ull); }
    void receive(Msg m)
    {
        if (m == Msg::Clear)
        {
            cache = frameOf(LANDED, 41900);
            SnapshotRtcCopy::store(rtc, cache.data(), 7'800'000ull);
        }
        else
        {
            SnapshotEndFlight::markLanded(cache.data(), kN);
            SnapshotEndFlight::markLanded(rtc);
        }
    }
    bool servesTheFlight() const { return servable(cache.data()) || servable(rtc.frame); }
};

struct Landing
{
    std::vector<std::pair<uint32_t, Msg>> received;
    Oc oc;
    int clear_tries = 0;
    bool served_at_end = true;
};

// LANDED passes at 1 kHz for `ms`. The queue is full for the first
// `full_ms`; the link drops what is sent in [lost_from, lost_to) and, with
// `lose_clear`, every clear before `lose_clear_to`.
struct Scenario
{
    uint32_t ms = 3000;
    uint32_t full_ms = 0;
    uint32_t lost_from = 0, lost_to = 0;
    uint32_t lose_clear_to = 0;
    bool old_fc = false;   // before #1779: the clear once, in the one-shot actions
};

Landing land(const Scenario& r)
{
    Landing l;
    Link link;
    link.held = r.full_ms ? link.capacity : 0;
    LandedSends::State s;
    for (uint32_t t = 0; t < r.ms; ++t)
    {
        if (t == r.full_ms) link.held = 0;   // the sender runs again
        if (!r.old_fc)
        {
            LandedSends::service(s, t, [&] { ++l.clear_tries; return link.enqueue(Msg::Clear); },
                                 [&] { return link.enqueue(Msg::EndFlight); });
        }
        else
        {
            if (t == 0) { ++l.clear_tries; (void)link.enqueue(Msg::Clear); }
            if (!s.end_flight_sent) s.end_flight_sent = link.enqueue(Msg::EndFlight);
        }
        if (link.held == 0)
        {
            while (!link.queue.empty())
            {
                const Msg m = link.queue.front();
                link.queue.pop_front();
                if (t >= r.lost_from && t < r.lost_to) continue;
                if (m == Msg::Clear && t < r.lose_clear_to) continue;
                l.received.push_back({t, m});
                l.oc.receive(m);
            }
        }
    }
    l.served_at_end = l.oc.servesTheFlight();
    return l;
}

TEST(LandedClear, AFullQueueAtLandingDelaysTheClearButItArrivesFirst)
{
    Scenario r;
    r.full_ms = 30;
    r.ms = 500;
    const Landing l = land(r);
    EXPECT_EQ(l.clear_tries, 3) << "at 0, 20 and 40 ms";
    ASSERT_GE(l.received.size(), 2u);
    EXPECT_EQ(l.received[0], std::make_pair(40u, Msg::Clear));
    EXPECT_EQ(l.received[1], std::make_pair(40u, Msg::EndFlight));
    EXPECT_EQ(l.received.size(), 2u) << "no repeat inside the first second";
    EXPECT_FALSE(l.served_at_end);
    EXPECT_EQ(stateOf(l.oc.cache.data()), (uint8_t)LANDED);
}

TEST(LandedClear, AClearLostOnTheLinkIsCoveredByEndFlightThenTheRepeat)
{
    Scenario r;
    r.lose_clear_to = 1;   // the first clear only
    r.ms = 1500;
    const Landing l = land(r);
    ASSERT_EQ(l.received.size(), 2u);
    EXPECT_EQ(l.received[0], std::make_pair(0u, Msg::EndFlight)) << "END_FLIGHT marks the cache";
    EXPECT_EQ(l.received[1], std::make_pair(1000u, Msg::Clear)) << "and the repeat follows";
    EXPECT_FALSE(l.served_at_end);
    EXPECT_TRUE(SnapshotRtcCopy::intact(l.oc.rtc));
}

TEST(LandedClear, BothLostAcrossAnOcResetAreCoveredByTheRepeat)
{
    // the OC resets across the landing and seeds its cache with the flight
    // from its RTC copy; everything the FC sends for 600 ms is lost
    Scenario r;
    r.lost_from = 0;
    r.lost_to = 600;
    r.ms = 1500;
    const Landing l = land(r);
    ASSERT_EQ(l.received.size(), 1u);
    EXPECT_EQ(l.received[0], std::make_pair(1000u, Msg::Clear));
    EXPECT_FALSE(l.served_at_end);
}

TEST(LandedClear, BeforeTheFixAFullQueueLostTheClearAndOnlyTheOcCoversIt)
{
    // the old FC: one try, into a full queue; END_FLIGHT retried
    Scenario r;
    r.full_ms = 30;
    r.ms = 3000;
    r.old_fc = true;
    const Landing l = land(r);
    EXPECT_EQ(l.clear_tries, 1);
    ASSERT_EQ(l.received.size(), 1u);
    EXPECT_EQ(l.received[0].second, Msg::EndFlight);
    EXPECT_FALSE(l.served_at_end) << "END_FLIGHT marked the cache";
    // and without the OC's mark the flight would be served
    Oc untouched;
    EXPECT_TRUE(untouched.servesTheFlight());
}

}  // namespace
