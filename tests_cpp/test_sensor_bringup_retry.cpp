// #1773: a sensor's bounded bring-up (TR_Sensor_Collector/sensor_bringup_retry.h),
// as the MMC5983MA's runs it: a try, then a soft reset and 1 s of waits
// between tries, given up 2 s after the first.
//
// It used to retry forever inside setup_fc: a magnetometer that did not
// answer kept the FC from finishing its boot, even on a reboot in flight.

#include <gtest/gtest.h>

#include <cstdint>

#include "sensor_bringup_retry.h"

namespace {

constexpr uint32_t kTimeoutMs = 2000;   // TR_Sensor_Collector.cpp kMmcBringupTimeoutMs
constexpr uint32_t kRecoverMs = 1000;   // its retry: 500 ms, a soft reset, 500 ms

struct Part
{
    uint32_t now = 0;          // the clock
    int answers_on = 0;        // the try it first answers (0: never)
    uint32_t try_ms = 1;       // one product-ID read
    int tries = 0, recovers = 0;

    bool attempt()
    {
        now += try_ms;
        if (++calls > 100)
        {
            ADD_FAILURE() << "unbounded: still trying after 100 tries";
            return true;   // end it, rather than hang the suite
        }
        return answers_on != 0 && ++tries >= answers_on;
    }
    int calls = 0;
    void recover()
    {
        recovers++;
        now += kRecoverMs;
    }
    bool bringUp(uint32_t& attempts, uint32_t timeout_ms = kTimeoutMs)
    {
        tries = 0;
        return SensorBringup::retryUntil([this] { return attempt(); }, [this] { recover(); },
                                         [this] { return now; }, timeout_ms, attempts);
    }
};

TEST(SensorBringupRetry, APartThatNeverAnswersIsGivenUpInAboutTwoSeconds)
{
    Part p;   // a cold joint on CS or MISO, a missing or dead part: never answers
    uint32_t attempts = 0;
    EXPECT_FALSE(p.bringUp(attempts));
    EXPECT_EQ(attempts, 3u);              // at 0, ~1 s and ~2 s
    EXPECT_EQ(p.recovers, 2);
    EXPECT_LE(p.now, kTimeoutMs + 100);   // and no longer
}

TEST(SensorBringupRetry, AHealthyPartTakesOneTryAndNoReset)
{
    Part p;
    p.answers_on = 1;
    uint32_t attempts = 0;
    EXPECT_TRUE(p.bringUp(attempts));
    EXPECT_EQ(attempts, 1u);
    EXPECT_EQ(p.recovers, 0);
}

TEST(SensorBringupRetry, APartThatAnswersAfterAResetIsKept)
{
    for (int on = 2; on <= 3; ++on)
    {
        Part p;
        p.answers_on = on;
        uint32_t attempts = 0;
        EXPECT_TRUE(p.bringUp(attempts)) << on;
        EXPECT_EQ(attempts, static_cast<uint32_t>(on)) << on;
        EXPECT_EQ(p.recovers, on - 1) << on;
    }
    Part late;   // one that would have answered on its fourth try: past the deadline
    late.answers_on = 4;
    uint32_t attempts = 0;
    EXPECT_FALSE(late.bringUp(attempts));
    EXPECT_EQ(attempts, 3u);
}

TEST(SensorBringupRetry, OneFullTryAlwaysRunsEvenPastTheDeadline)
{
    Part p;
    p.try_ms = 5000;   // a try that takes longer than the whole budget
    p.answers_on = 1;
    uint32_t attempts = 0;
    EXPECT_TRUE(p.bringUp(attempts));
    Part q;
    q.try_ms = 5000;
    EXPECT_FALSE(q.bringUp(attempts));
    EXPECT_EQ(attempts, 1u) << "no retry once the deadline has passed";
    EXPECT_EQ(q.recovers, 0);
}

TEST(SensorBringupRetry, AFlightInProgressGetsOneTryAndNoReset)
{
    // magOneTryThisBoot: the budget is 0
    Part dead;
    uint32_t attempts = 0;
    EXPECT_FALSE(dead.bringUp(attempts, 0));
    EXPECT_EQ(attempts, 1u);
    EXPECT_EQ(dead.recovers, 0);
    Part healthy;
    healthy.answers_on = 1;
    EXPECT_TRUE(healthy.bringUp(attempts, 0));
    EXPECT_EQ(attempts, 1u);
}

TEST(SensorBringupRetry, TheDeadlineIsWrapSafe)
{
    Part p;
    p.now = 0xFFFFFF00u;   // millis() just before it wraps
    uint32_t attempts = 0;
    EXPECT_FALSE(p.bringUp(attempts));
    EXPECT_EQ(attempts, 3u);
    EXPECT_EQ(p.recovers, 2);
}

}  // namespace
