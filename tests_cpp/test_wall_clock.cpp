// #1155 item 3: flight filenames derived from a phone time sync must carry
// elapsed days through the month and year, and must not wrap after 256 days.
#include <gtest/gtest.h>
#include <WallClock.h>

using tr_wall_clock::Civil;
using tr_wall_clock::advance;
using tr_wall_clock::civilFromDays;
using tr_wall_clock::daysFromCivil;

static void expectCivil(const Civil& c, unsigned y, unsigned mo, unsigned d,
                        unsigned h, unsigned mi, unsigned s)
{
    EXPECT_EQ(c.year, y);   EXPECT_EQ(c.month, mo);  EXPECT_EQ(c.day, d);
    EXPECT_EQ(c.hour, h);   EXPECT_EQ(c.minute, mi); EXPECT_EQ(c.second, s);
}

TEST(WallClock, KnownDayCounts)
{
    EXPECT_EQ(daysFromCivil(1970, 1, 1), 0);
    EXPECT_EQ(daysFromCivil(2000, 3, 1), 11017);
    EXPECT_EQ(daysFromCivil(2026, 9, 9), 20705);   // 56 years + 14 leap days + 251
}

TEST(WallClock, DayCountRoundTripsAcrossLeapRules)
{
    for (int64_t z = -2000; z <= 40000; z += 3)
    {
        uint16_t y; uint8_t m, d;
        civilFromDays(z, y, m, d);
        EXPECT_EQ(daysFromCivil(y, m, d), z) << "z=" << z;
        EXPECT_GE(m, 1); EXPECT_LE(m, 12);
        EXPECT_GE(d, 1); EXPECT_LE(d, 31);
    }
    uint16_t y; uint8_t m, d;
    civilFromDays(daysFromCivil(2024, 2, 28) + 1, y, m, d);   // leap year
    EXPECT_EQ(y, 2024); EXPECT_EQ(m, 2); EXPECT_EQ(d, 29);
    civilFromDays(daysFromCivil(2100, 2, 28) + 1, y, m, d);   // century, not leap
    EXPECT_EQ(y, 2100); EXPECT_EQ(m, 3); EXPECT_EQ(d, 1);
    civilFromDays(daysFromCivil(2000, 2, 28) + 1, y, m, d);   // 400-year, leap
    EXPECT_EQ(y, 2000); EXPECT_EQ(m, 2); EXPECT_EQ(d, 29);
}

TEST(WallClock, ZeroElapsedIsIdentity)
{
    expectCivil(advance({2026, 9, 9, 14, 30, 5}, 0), 2026, 9, 9, 14, 30, 5);
}

TEST(WallClock, CarriesIntoTheNextMonth)
{
    // A sync at 23:59:50 on the 30th, a flight 20 s later: the old code
    // produced 2026-09-31.
    expectCivil(advance({2026, 9, 30, 23, 59, 50}, 20), 2026, 10, 1, 0, 0, 10);
}

TEST(WallClock, CarriesIntoTheNextYear)
{
    expectCivil(advance({2026, 12, 31, 23, 0, 0}, 7200), 2027, 1, 1, 1, 0, 0);
}

TEST(WallClock, CarriesThroughALeapDay)
{
    expectCivil(advance({2028, 2, 28, 12, 0, 0}, 86400), 2028, 2, 29, 12, 0, 0);
    expectCivil(advance({2028, 2, 28, 12, 0, 0}, 2 * 86400), 2028, 3, 1, 12, 0, 0);
}

TEST(WallClock, LongUptimeDoesNotWrapTheDay)
{
    // 300 days after a New Year sync. The old uint8_t accumulator held
    // (uint8_t)(1 + 300) = 45 — an impossible day — and would have wrapped
    // again at 256 days of further uptime.
    expectCivil(advance({2026, 1, 1, 0, 0, 0}, 300u * 86400u), 2026, 10, 28, 0, 0, 0);
    expectCivil(advance({2026, 1, 1, 6, 0, 0}, 400u * 86400u), 2027, 2, 5, 6, 0, 0);
}

TEST(WallClock, AdvanceIsAdditive)
{
    const Civil sync{2026, 3, 15, 22, 10, 45};
    const uint32_t steps[] = {1u, 59u, 3600u, 86399u, 86400u, 31u * 86400u, 400u * 86400u};
    for (uint32_t a : steps)
        for (uint32_t b : steps)
        {
            const Civil two  = advance(advance(sync, a), b);
            const Civil once = advance(sync, a + b);
            EXPECT_EQ(daysFromCivil(two.year, two.month, two.day),
                      daysFromCivil(once.year, once.month, once.day)) << a << "+" << b;
            EXPECT_EQ(two.hour, once.hour); EXPECT_EQ(two.minute, once.minute);
            EXPECT_EQ(two.second, once.second);
        }
}
