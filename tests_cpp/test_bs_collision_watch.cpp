#include <gtest/gtest.h>

#include <vector>

#include "bs_collision_watch.h"

using namespace bs_collision_watch;

// Two rockets on one grid point reach the base station as one garbled frame
// every 500 ms (bench 2026-10-21: a CRC failure per frame, nothing decoded).
// The watchdog turns that steady CRC stream into a broadcast release.

namespace {

// Drive the watch with `fails_per_s` CRC failures a second for `seconds`,
// ticking every 50 ms; returns the times a release fired.
std::vector<uint32_t> run(Watch& w, uint32_t& t, uint32_t& crc, double fails_per_s,
                          uint32_t seconds, bool armed = true, bool in_flight = false)
{
    std::vector<uint32_t> fired;
    double acc = 0;
    for (uint32_t i = 0; i < seconds * 20; i++)
    {
        t += 50;
        acc += fails_per_s / 20.0;
        while (acc >= 1.0) { crc++; acc -= 1.0; }
        if (w.onTick(t, crc, armed, in_flight)) fired.push_back(t);
    }
    return fired;
}

}  // namespace

TEST(BsCollisionWatch, ACollidingPairIsReleasedWithinOneWindow) {
    Watch w;
    uint32_t t = 1000, crc = 0;
    const auto fired = run(w, t, crc, 2.0, 12);
    ASSERT_EQ(fired.size(), 1u);
    EXPECT_LE(fired[0] - 1000, kWindowMs + 100);
    EXPECT_GE(w.last_window_fails, 19u);
}

TEST(BsCollisionWatch, AQuietChannelNeverReleases) {
    Watch w;
    uint32_t t = 0, crc = 0;
    EXPECT_TRUE(run(w, t, crc, 0.1, 120).empty());   // an occasional noise hit
}

TEST(BsCollisionWatch, ReleasesAreAtLeastTheIntervalApart) {
    // A pair that drew close phases again, or a weak rocket at the edge of
    // range that fails CRC on its own: re-releases, but no storm.
    Watch w;
    uint32_t t = 0, crc = 0;
    const auto fired = run(w, t, crc, 2.0, 120);
    ASSERT_GE(fired.size(), 2u);
    for (size_t i = 1; i < fired.size(); i++)
    {
        EXPECT_GE(fired[i] - fired[i - 1], kMinIntervalMs);
    }
}

TEST(BsCollisionWatch, NothingWhileARocketIsInFlight) {
    Watch w;
    uint32_t t = 0, crc = 0;
    EXPECT_TRUE(run(w, t, crc, 3.0, 60, /*armed=*/true, /*in_flight=*/true).empty());
    // ...and the failures counted in flight do not fire the moment it lands.
    EXPECT_TRUE(run(w, t, crc, 0.0, 12).empty());
}

TEST(BsCollisionWatch, NothingWhileDisarmed) {
    // Slots off, hop mode or hopping: the slots are not in use.
    Watch w;
    uint32_t t = 0, crc = 0;
    EXPECT_TRUE(run(w, t, crc, 3.0, 60, /*armed=*/false).empty());
}

TEST(BsCollisionWatch, CounterWrapIsHarmless) {
    Watch w;
    uint32_t t = 0, crc = 0xFFFFFFF0u;
    const auto fired = run(w, t, crc, 2.0, 12);
    EXPECT_EQ(fired.size(), 1u);
}
