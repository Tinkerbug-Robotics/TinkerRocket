// Which of the FC's SNAPSHOT_MSG frames the OC keeps (snapshot_replay_filter.h).
//
// The bug this pins: a clear sent with no flight in progress carries the FC's
// uptime as its elapsed time, and the OC took it as the replay baseline. On the
// V8 bench (2026-10-17) a sim started 8.8 s after an FC boot kept no snapshot
// for its first 8.8 s, so an FC reset 3 s into it found no flight to restore.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "snapshot_replay_filter.h"

using SnapshotReplayFilter::keep;
using SnapshotReplayFilter::kNewFlightBackstepMs;
using SnapshotReplayFilter::State;

namespace {

constexpr bool kInflight = true;
constexpr bool kClear    = false;

// The filter before this change, copied from the OC's SNAPSHOT_MSG branch.
struct OldFilter {
    uint32_t last = 0;
    bool keep(bool inflight, uint32_t elapsed) {
        const bool stale = inflight && elapsed < last &&
                           (last - elapsed) <= kNewFlightBackstepMs;
        if (!stale) last = elapsed;
        return !stale;
    }
};

// Feed a 10 Hz flight from `from_ms` to `to_ms` and count the frames kept.
int flyKept(State& st, uint32_t from_ms, uint32_t to_ms)
{
    int kept = 0;
    for (uint32_t t = from_ms; t <= to_ms; t += 100) kept += keep(st, kInflight, t) ? 1 : 0;
    return kept;
}

}  // namespace

TEST(SnapshotReplayFilter, AFlightIsKeptAndItsReplaysAreDropped)
{
    State st;
    EXPECT_EQ(flyKept(st, 100, 30000), 300);
    EXPECT_FALSE(keep(st, kInflight, 29900));   // a few hundred ms older
    EXPECT_FALSE(keep(st, kInflight, 29100));
    EXPECT_FALSE(keep(st, kInflight, 30000));   // the newest frame again
    EXPECT_TRUE(keep(st, kInflight, 30100));    // the flight goes on
}

TEST(SnapshotReplayFilter, ASimStartedSoonAfterBootKeepsItsFirstSeconds)
{
    // The bench case: no flight this OC session, then the sim start's clear
    // with the FC's uptime (8.8 s), then the sim flight from 100 ms.
    State st;
    EXPECT_TRUE(keep(st, kClear, 8800));
    EXPECT_EQ(flyKept(st, 100, 3000), 30);   // was 0 before the fix
    EXPECT_EQ(flyKept(st, 3100, 9000), 60);

    OldFilter old;
    EXPECT_TRUE(old.keep(kClear, 8800));
    int old_kept = 0;
    for (uint32_t t = 100; t <= 3000; t += 100) old_kept += old.keep(kInflight, t) ? 1 : 0;
    EXPECT_EQ(old_kept, 0);   // what the bench saw
}

TEST(SnapshotReplayFilter, ABootClearWithNoFlightLeavesTheNextFlightWhole)
{
    State st;
    EXPECT_TRUE(keep(st, kClear, 5000));      // the FC's boot, uptime 5 s
    EXPECT_TRUE(keep(st, kClear, 6200));      // and another
    EXPECT_EQ(flyKept(st, 100, 20000), 200);
}

TEST(SnapshotReplayFilter, AReplayAfterTheLandingClearIsStillDropped)
{
    // #846 SAFETY: after LANDED, a replay of a pre-landing INFLIGHT frame must
    // not be kept, or a later FC panic restores INFLIGHT on the ground.
    State st;
    flyKept(st, 100, 60000);
    EXPECT_TRUE(keep(st, kClear, 60100));     // the LANDED clear: always kept
    EXPECT_FALSE(keep(st, kInflight, 59900));
    EXPECT_FALSE(keep(st, kInflight, 60000));
    // A pre-landing frame delivered late, newer than the last one kept but
    // older than the clear: the clear raised the baseline to its own time.
    EXPECT_FALSE(keep(st, kInflight, 60050));
    // And for as long as the replays keep coming.
    for (int i = 0; i < 50; ++i) EXPECT_FALSE(keep(st, kInflight, 59500 + 10 * i));
}

TEST(SnapshotReplayFilter, ASecondFlightAfterALandingIsKept)
{
    State st;
    flyKept(st, 100, 60000);
    keep(st, kClear, 60100);
    EXPECT_EQ(flyKept(st, 100, 30000), 300);  // more than the backstep behind
}

TEST(SnapshotReplayFilter, ABootClearAfterARefusedSimNeverLowersTheBaseline)
{
    // A sim flown to 30 s, the FC reset mid-sim, its boot clear at uptime 5 s.
    // The old filter took 5 s as the baseline and so kept replays of the sim's
    // frames between 5 s and 30 s.
    State st;
    flyKept(st, 100, 30000);
    EXPECT_TRUE(keep(st, kClear, 5000));
    EXPECT_FALSE(keep(st, kInflight, 29800));

    OldFilter old;
    for (uint32_t t = 100; t <= 30000; t += 100) old.keep(kInflight, t);
    old.keep(kClear, 5000);
    EXPECT_TRUE(old.keep(kInflight, 29800));  // the old hole

    // A new flight is still kept: it starts more than the backstep behind.
    EXPECT_EQ(flyKept(st, 100, 5000), 50);
}

TEST(SnapshotReplayFilter, AClearIsNeverDropped)
{
    State st;
    flyKept(st, 100, 20000);
    for (uint32_t e : {0u, 100u, 15000u, 19950u, 20000u, 25000u, 900000u})
        EXPECT_TRUE(keep(st, kClear, e)) << e;
}

TEST(SnapshotReplayFilter, AShortFlightStillCostsTheNextOneItsFirstSeconds)
{
    // The residual, pinned so a change to it is deliberate: a 3 s flight, then
    // another in the same OC session. The elapsed time alone can't tell the
    // second flight's first 3 s from replays of the first.
    State st;
    flyKept(st, 100, 3000);
    keep(st, kClear, 3100);
    EXPECT_EQ(flyKept(st, 100, 3100), 0);
    EXPECT_EQ(flyKept(st, 3200, 6000), 29);
}

TEST(SnapshotReplayFilter, WithNoClearsItKeepsWhatTheOldFilterKept)
{
    // Every stream without clears, apart from an exact repeat of the newest
    // frame (which the old filter rewrote over itself and this one drops).
    const std::vector<uint32_t> seq = {100, 200, 300, 250, 400, 399, 12000, 100, 200,
                                       15000, 4000, 4999, 5000, 3000, 30000, 19999,
                                       20000, 31000, 20999, 50, 60};
    State st;
    OldFilter old;
    uint32_t newest = 0;
    bool any = false;
    for (uint32_t e : seq)
    {
        const bool repeat = any && e == newest;
        const bool now = keep(st, kInflight, e);
        const bool was = old.keep(kInflight, e);
        if (!repeat) EXPECT_EQ(now, was) << "elapsed " << e;
        if (now) { newest = e; any = true; }
    }
}
