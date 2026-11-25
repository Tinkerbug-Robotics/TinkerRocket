// test_level_gate.cpp — when the raw-GNSS shadow may level on its
// accelerometer in flight (LevelGate.h, #1600): only once its velocity has
// held steady from epoch to epoch and the specific force is g's size. The
// numbers are the rig replay of rp5_0829's (2026-10-13): the drogue opening
// with the rocket still moving sideways, the steady descent under it, and
// the main's opening.
#include <gtest/gtest.h>

#include "LevelGate.h"

using gnss_raw::LevelGate;

namespace
{

const float kStill[3] = {0.0f, 0.0f, -9.81f};   // a steady descent: the drag is g, straight up

double g_last_t = 0.0;   // the last epoch's time: the tests ask just after it

void epoch(LevelGate& g, double t, float vn, float ve, float vd)
{
    const float v[3] = {vn, ve, vd};
    g.onEpoch(t, v);
    g_last_t = t;
}

double tNow(const LevelGate&) { return g_last_t + 0.01; }

}  // namespace

TEST(LevelGate, NotUntilTheVelocityHasHeldSteadyTwiceInARow)
{
    LevelGate g;
    g_last_t = 0.0;
    g.onStep(0.0, kStill);
    EXPECT_FALSE(g.allow(0.01, kStill));         // no epochs: an outage, g's size only 0.01 s
    epoch(g, 20.0, 2.0, 2.5, 16.3);
    EXPECT_FALSE(g.allow(tNow(g), kStill));               // one: nothing to compare
    epoch(g, 21.0, 2.1, 2.6, 16.3);              // 0.14 m/s^2 of drift with the wind
    EXPECT_FALSE(g.allow(tNow(g), kStill));
    epoch(g, 22.0, 2.2, 2.7, 16.3);
    EXPECT_TRUE(g.steady());
    EXPECT_TRUE(g.allow(tNow(g), kStill));
}

TEST(LevelGate, TheDrogueOpeningIsNotSteady)
{
    // rp5_0829 from apogee: 12 m/s sideways bleeding off over 3 s while the
    // descent builds -- the drag 20-45 degrees off vertical
    LevelGate g;
    epoch(g, 12.0, 8.6, 8.6, 6.0);
    epoch(g, 13.0, 8.4, 8.4, 11.8);
    epoch(g, 14.0, 4.1, 4.1, 14.3);
    epoch(g, 15.0, 1.4, 1.4, 15.3);
    EXPECT_FALSE(g.steady());
    // its size is already g's by 15 s, 18 degrees off vertical: the size alone would have let it in
    const float drag[3] = {0.0f, 2.9f, -8.99f};
    EXPECT_FALSE(g.allow(tNow(g), drag));
    epoch(g, 16.0, 1.1, 1.1, 15.9);
    epoch(g, 17.0, 1.7, 1.7, 16.2);
    EXPECT_FALSE(g.steady());                    // 0.9 m/s^2: still turning
    epoch(g, 18.0, 2.1, 2.1, 16.3);
    epoch(g, 19.0, 2.3, 2.3, 16.3);
    epoch(g, 20.0, 2.5, 2.5, 16.3);
    EXPECT_TRUE(g.allow(tNow(g), kStill));                // 3 degrees and settling
}

TEST(LevelGate, TheMainOpeningIsTheWrongSizeEvenWhenSteady)
{
    LevelGate g;
    for (int k = 0; k < 4; ++k)
        epoch(g, 30.0 + k, 3.4f, 0.0f, 16.1f);
    ASSERT_TRUE(g.steady());
    const float opening[3] = {0.0f, 0.0f, -12.0f};   // vertical, but 1.2 g: it would go into the bias
    EXPECT_FALSE(g.allow(tNow(g), opening));
    const float near_g[3] = {0.3f, -0.2f, -9.5f};
    EXPECT_TRUE(g.allow(tNow(g), near_g));
}

TEST(LevelGate, AGapOrAResetStartsOver)
{
    LevelGate g;
    for (int k = 0; k < 3; ++k)
        epoch(g, 40.0 + k, 0.0f, 0.0f, 4.5f);
    ASSERT_TRUE(g.steady());
    epoch(g, 45.0, 0.0f, 0.0f, 4.5f);            // 3 s without an epoch: not a change it can judge
    EXPECT_FALSE(g.steady());
    epoch(g, 46.0, 0.0f, 0.0f, 4.5f);
    epoch(g, 47.0, 0.0f, 0.0f, 4.5f);
    EXPECT_TRUE(g.steady());
    g.reset();
    EXPECT_FALSE(g.steady());
    epoch(g, 48.0, 0.0f, 0.0f, 4.5f);
    EXPECT_FALSE(g.steady());
}

TEST(LevelGate, OneJoltStartsTheCountAgain)
{
    LevelGate g;
    for (int k = 0; k < 3; ++k)
        epoch(g, 48.0 + k, 2.9f, 0.0f, 12.0f);
    ASSERT_TRUE(g.steady());
    epoch(g, 51.0, 2.8f, 0.0f, 4.5f);            // the main's second opening: 7.5 m/s in a second
    EXPECT_FALSE(g.steady());
    epoch(g, 52.0, 2.8f, 0.0f, 4.5f);
    EXPECT_FALSE(g.steady());
    epoch(g, 53.0, 2.8f, 0.0f, 4.5f);
    EXPECT_TRUE(g.steady());
}

TEST(LevelGate, WithNoSatellitesItLevelsOnceTheForceHasBeenGsSizeFiveSeconds)
{
    // Under a chute with no satellites: no epochs to call it steady, and a
    // gyro bias tilts the attitude unchecked unless it levels
    LevelGate g;
    const float opening[3] = {0.0f, 3.0f, -6.0f};   // the drag still settling: 6.7 m/s^2
    double t = 40.0;
    for (; t < 42.0; t += 0.01)
    {
        g.onStep(t, opening);
        EXPECT_FALSE(g.allow(t, opening));
    }
    for (; t < 46.99; t += 0.01)
    {
        g.onStep(t, kStill);
        EXPECT_FALSE(g.allow(t, kStill));   // g's size, but not for 5 s yet
    }
    t = 47.01;
    g.onStep(t, kStill);
    EXPECT_TRUE(g.allow(t, kStill));
    // a jolt starts the 5 s again
    g.onStep(t + 0.01, opening);
    g.onStep(t + 0.02, kStill);
    EXPECT_FALSE(g.allow(t + 0.02, kStill));
    // with epochs coming, their steadiness decides, not the force's hold
    epoch(g, 50.0, 0.0f, 0.0f, 12.0f);
    EXPECT_FALSE(g.allow(50.01, kStill));
}
