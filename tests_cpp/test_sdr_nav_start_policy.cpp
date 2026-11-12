// The V10's gnss-sdr-dev image's start without the board, and a late board's
// take-over (flight_computer/main/sdr_nav_start_policy.h): the degraded start
// once no usable fix came 2 min after boot on the pad, at once when armed,
// never in flight, before the gyro is averaged or within the settle; a late fix restarts it on
// the pad unarmed only; the boot's verdict on a silent board.
#include <gtest/gtest.h>

#include "sdr_nav_start_policy.h"

using namespace sdr_nav_start;

TEST(SdrNavStart, NoFixTwoMinutesAfterBootOnThePadStartsWithoutTheBoard)
{
    EXPECT_FALSE(degradedStartDue(false, true, true, kNoFixMs - 1, READY));
    EXPECT_TRUE(degradedStartDue(false, true, true, kNoFixMs, READY));
    EXPECT_TRUE(degradedStartDue(false, true, true, 10u * kNoFixMs, READY));
    EXPECT_EQ(kNoFixMs, 120000u);
}

TEST(SdrNavStart, ArmedItStartsAtOnce)
{
    EXPECT_TRUE(degradedStartDue(false, true, true, 1000u, PRELAUNCH));
    EXPECT_TRUE(degradedStartDue(false, true, true, 0u, PRELAUNCH));
}

TEST(SdrNavStart, NeverWithAFixToStartOnOrWithoutTheGyroOrInFlight)
{
    // nor within the GNSS-absent settle (after boot, after a sim's reset)
    EXPECT_FALSE(degradedStartDue(false, true, false, 10u * kNoFixMs, READY));
    EXPECT_FALSE(degradedStartDue(false, true, false, 0u, PRELAUNCH));
    EXPECT_FALSE(degradedStartDue(true, true, true, 10u * kNoFixMs, READY));
    EXPECT_FALSE(degradedStartDue(true, true, true, 0u, PRELAUNCH));
    EXPECT_FALSE(degradedStartDue(false, false, true, 10u * kNoFixMs, READY));
    EXPECT_FALSE(degradedStartDue(false, false, true, 0u, PRELAUNCH));
    // the accelerometer is no tilt in flight: the start stays the pad's job
    for (RocketState s : {INITIALIZATION, INFLIGHT, LANDED, MAG_CALIBRATION})
        EXPECT_FALSE(degradedStartDue(false, true, true, 10u * kNoFixMs, s)) << int(s);
}

TEST(SdrNavStart, ALateFixRestartsItOnThePadUnarmedOnly)
{
    EXPECT_TRUE(restartOnFix(true, true, READY));
    EXPECT_FALSE(restartOnFix(true, false, READY));    // no usable fix yet
    EXPECT_FALSE(restartOnFix(false, true, READY));    // not a degraded start
    for (RocketState s : {PRELAUNCH, INFLIGHT, LANDED, MAG_CALIBRATION, INITIALIZATION})
        EXPECT_FALSE(restartOnFix(true, true, s)) << int(s);   // armed or flying: it stands
}

TEST(SdrNavStart, TheBoardIsSilentOnlyAfterItsSecondsWithNoFrame)
{
    EXPECT_FALSE(boardSilent(false, kHeardMs - 1));
    EXPECT_TRUE(boardSilent(false, kHeardMs));
    EXPECT_FALSE(boardSilent(true, 10u * kHeardMs));
    EXPECT_GE(kHeardMs, 2000u);   // its status comes once a second
}
