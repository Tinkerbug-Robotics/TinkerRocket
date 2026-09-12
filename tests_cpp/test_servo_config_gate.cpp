// ServoConfigGate.h — the SERVO_CONFIG acceptance policy shared by the FC (which
// applies it) and the OC (which caches and reports it).
//
// These exist because the policy previously lived ONLY inside TR_ServoControl,
// where the OUT computer could not reach it.  #1141 item 3 and #1137 item 2
// therefore landed on the flight computer alone: the FC refused a bad timing or
// a degenerate fin span and kept the previous value, while the OC cached the
// refused bytes, wrote them to its own NVS, and reported them in the cmd-20
// readback — so the app showed "shz":0 while the FC was running 56, and still
// showed 0 after a power cycle.  Measured on the V9 bench 2026-09-12.
//
// The bench cases are reproduced here verbatim, so the regression is caught on
// the host next time rather than on hardware.

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "ServoConfigGate.h"

// ---- servo timing -------------------------------------------------------

TEST(ServoTimingGate, TheBenchDefaultIsAccepted)
{
    // Exactly what the V9 bench board carries.
    EXPECT_TRUE(servoTimingSane(56, 1100, 1900));
    EXPECT_EQ(servoTimingRc(56, 1100, 1900), SERVO_TIMING_OK);
}

TEST(ServoTimingGate, HzZeroIsRefusedAndNamed)
{
    // duty = pulse_us * hz * max_duty / 1e6, so hz == 0 is duty 0 on every
    // channel — four relaxed fins, indistinguishable from idle().
    EXPECT_FALSE(servoTimingSane(0, 1100, 1900));
    EXPECT_EQ(servoTimingRc(0, 1100, 1900), SERVO_TIMING_REJ_HZ);
    EXPECT_STREQ(servoTimingRcName(servoTimingRc(0, 1100, 1900)),
                 "hz out of range");
}

TEST(ServoTimingGate, NegativeHzIsRefusedRatherThanWrapped)
{
    // A negative hz casts to ~4.29e9 downstream and the 32-bit duty product
    // wraps to an arbitrary value.  Refuse it before it gets there.
    EXPECT_FALSE(servoTimingSane(-1, 1100, 1900));
    EXPECT_EQ(servoTimingRc(-1, 1100, 1900), SERVO_TIMING_REJ_HZ);
}

TEST(ServoTimingGate, HzBoundsAreInclusive)
{
    EXPECT_TRUE(servoTimingSane(SERVO_MIN_HZ, 1100, 1900));
    EXPECT_TRUE(servoTimingSane(SERVO_MAX_HZ, 1100, 1900));
    EXPECT_FALSE(servoTimingSane(SERVO_MIN_HZ - 1, 1100, 1900));
    EXPECT_FALSE(servoTimingSane(SERVO_MAX_HZ + 1, 1100, 1900));
}

TEST(ServoTimingGate, PulseEndpointsOutOfRangeAreNamedSeparately)
{
    EXPECT_EQ(servoTimingRc(56, SERVO_MIN_PULSE_US - 1, 1900),
              SERVO_TIMING_REJ_PULSE);
    EXPECT_EQ(servoTimingRc(56, 1100, SERVO_MAX_PULSE_US + 1),
              SERVO_TIMING_REJ_PULSE);
}

TEST(ServoTimingGate, TooNarrowASpanIsItsOwnReason)
{
    // Both endpoints legal, span not: a servo that cannot move far enough to
    // be told apart from one that is stuck.
    const int mn = 1400;
    const int mx = mn + SERVO_MIN_PULSE_SPAN_US - 1;
    EXPECT_EQ(servoTimingRc(56, mn, mx), SERVO_TIMING_REJ_SPAN);
    EXPECT_TRUE(servoTimingSane(56, mn, mn + SERVO_MIN_PULSE_SPAN_US));
}

TEST(ServoTimingGate, InvertedEndpointsAreRefused)
{
    EXPECT_FALSE(servoTimingSane(56, 1900, 1100));
}

// ---- fin calibration ----------------------------------------------------

TEST(FinCalGate, TheBenchDefaultIsAccepted)
{
    EXPECT_TRUE(finCalSane(-30.0f, 30.0f));
}

TEST(FinCalGate, DegenerateSpanIsRefused)
{
    // The bench case: fin_min == fin_max == 10.0 froze the fins at centre for
    // a whole flight before #1137 item 2.
    EXPECT_FALSE(finCalSane(10.0f, 10.0f));
    EXPECT_EQ(finCalRc(10.0f, 10.0f), FIN_CAL_REJ_SPAN);
}

TEST(FinCalGate, SpanExactlyAtTheFloorIsAccepted)
{
    // `>=` is the predicate, and the bench confirmed 0.0/2.0 applies: a
    // legitimate narrow travel must not be rejected.
    EXPECT_TRUE(finCalSane(0.0f, FIN_MIN_SPAN_DEG));
    EXPECT_FALSE(finCalSane(0.0f, std::nextafterf(FIN_MIN_SPAN_DEG, 0.0f)));
}

TEST(FinCalGate, InvertedSpanIsRefused)
{
    EXPECT_FALSE(finCalSane(30.0f, -30.0f));
    EXPECT_EQ(finCalRc(30.0f, -30.0f), FIN_CAL_REJ_SPAN);
}

TEST(FinCalGate, NonFiniteIsRefusedAndDistinguishedFromANarrowSpan)
{
    // Neither app clamps its free-text field, so "inf"/"1e30"/a ten-digit
    // paste reaches the wire.  The reason has to be tellable apart from a
    // merely narrow range, which is an operator error of a different kind.
    const float inf = std::numeric_limits<float>::infinity();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(finCalRc(nan, 30.0f), FIN_CAL_REJ_NONFINITE);
    EXPECT_EQ(finCalRc(-30.0f, nan), FIN_CAL_REJ_NONFINITE);
    EXPECT_EQ(finCalRc(-inf, 30.0f), FIN_CAL_REJ_NONFINITE);
    EXPECT_EQ(finCalRc(-30.0f, inf), FIN_CAL_REJ_NONFINITE);
    EXPECT_STREQ(finCalRcName(finCalRc(nan, nan)), "non-finite endpoint");
}

TEST(FinCalGate, AnInfiniteSpanIsRefusedEvenThoughItIsWide)
{
    // (inf - -inf) is inf, which passes `>= FIN_MIN_SPAN_DEG` on its own.
    // The finite check has to come first, and this is why.
    const float inf = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(finCalSane(-inf, inf));
}
