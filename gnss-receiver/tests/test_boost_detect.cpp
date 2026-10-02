// Launch and burnout from the IMU (core/include/gnss/boost_detect.h): the receiver's fast trigger and its
// rest exit, the flight computer's slower rules, and the axial reading that keeps drag in the coast from
// counting as a burn.

extern "C" {
#include "gnss/boost_detect.h"
}

#include <gtest/gtest.h>

namespace {

const float G = 9.80665f;

struct Det {
    boost_detect_t d;
    explicit Det(bool fc = false)
    {
        boost_detect_cfg_t c;
        if (fc) {
            boost_detect_fc(&c);
        } else {
            boost_detect_default(&c);
        }
        boost_detect_init(&d, &c);
    }
    // n samples at f; the number of them with the boost profile on.
    int run(int n, float f)
    {
        int on = 0;
        for (int k = 0; k < n; k++) {
            on += boost_detect_step(&d, f);
        }
        return on;
    }
};

}  // namespace

TEST(BoostDetect, OnThePadNeverFires)
{
    Det t;
    EXPECT_EQ(t.run(600000, G), 0);  // ten minutes at 1 g
    EXPECT_EQ(t.d.phase, BOOST_PAD);
}

TEST(BoostDetect, FastTriggerAfter20ms)
{
    Det t;
    t.run(1000, G);
    EXPECT_EQ(t.run(19, 10.0f * G), 0);
    EXPECT_EQ(boost_detect_step(&t.d, 10.0f * G), 1);  // the 20th sample over 20 m/s^2
    EXPECT_EQ(t.d.phase, BOOST_BURN);
}

TEST(BoostDetect, FlightComputerRulesAfter250ms)
{
    Det t(true);
    t.run(1000, G);
    EXPECT_EQ(t.run(249, 10.0f * G), 0);
    EXPECT_EQ(boost_detect_step(&t.d, 10.0f * G), 1);
    // Any sample at or below its 30 m/s^2 starts the count again.
    Det u(true);
    u.run(200, 10.0f * G);
    u.run(1, 2.5f * G);
    EXPECT_EQ(u.run(249, 10.0f * G), 0);
    EXPECT_EQ(u.run(1, 10.0f * G), 1);
}

TEST(BoostDetect, BurnoutAfter50msBelowZeroThenTheHold)
{
    Det t;
    t.run(20, 10.0f * G);  // launch
    EXPECT_EQ(t.run(3000, 10.0f * G), 3000);
    EXPECT_EQ(t.run(49, -3.0f * G), 49);  // drag after burnout, not yet 50 samples
    EXPECT_EQ(t.d.phase, BOOST_BURN);
    t.run(1, -3.0f * G);
    EXPECT_EQ(t.d.phase, BOOST_HOLD);
    EXPECT_EQ(t.run(1999, -3.0f * G), 1999);  // the 2 s hold
    EXPECT_EQ(t.run(1, -3.0f * G), 0);
    EXPECT_EQ(t.d.phase, BOOST_COAST);
}

TEST(BoostDetect, NoBurnoutInTheLockout)
{
    Det t;
    t.run(20, 10.0f * G);
    t.run(100, -1.0f * G);  // a 100 ms dip right after launch: inside the 200 ms lockout
    EXPECT_EQ(t.d.phase, BOOST_BURN);
    t.run(100, 10.0f * G);
    EXPECT_EQ(t.d.phase, BOOST_BURN);
}

TEST(BoostDetect, AKnockOnThePadGoesBackToRest)
{
    Det t;
    t.run(1000, G);
    EXPECT_EQ(t.run(20, 3.0f * G), 1);    // a 30 ms knock trips the fast trigger ...
    EXPECT_EQ(t.run(10, 3.0f * G), 10);
    EXPECT_EQ(t.run(689, G), 689);        // ... the 200 ms lockout, then 500 ms at rest ...
    EXPECT_EQ(boost_detect_step(&t.d, G), 0);  // ... and the profile is off
    EXPECT_EQ(t.d.phase, BOOST_PAD);
    // The flight computer's rules have no way back: their launch is a latch.
    Det u(true);
    u.run(250, 3.5f * G);
    EXPECT_EQ(u.run(10000, G), 10000);
    EXPECT_EQ(u.d.phase, BOOST_BURN);
}

TEST(BoostDetect, ALowThrustBurnIsNotRest)
{
    Det t;
    t.run(20, 2.5f * G);
    EXPECT_EQ(t.run(5000, 2.5f * G), 5000);  // 2.5 g along the axis is outside 1 g +- 5 m/s^2
    EXPECT_EQ(t.d.phase, BOOST_BURN);
}

TEST(BoostDetect, DragInTheCoastIsNotABurnButASecondStageIs)
{
    Det t;
    t.run(20, 10.0f * G);
    t.run(3000, 10.0f * G);
    t.run(50 + 2000, -4.0f * G);  // burnout and the hold
    EXPECT_EQ(t.d.phase, BOOST_COAST);
    EXPECT_EQ(t.run(5000, -4.0f * G), 0);  // 4 g of drag, backwards: no burn
    EXPECT_EQ(t.run(20, 8.0f * G), 1);     // a second motor lights
    EXPECT_EQ(t.d.phase, BOOST_BURN);
}
