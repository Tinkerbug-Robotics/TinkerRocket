// #1176: should this OC boot bring the FC's rail up because a flight was in
// progress, even though the reset looks like a cold start?
//
// The load-bearing rows are the REFUSALS. A false negative costs one flight's
// recovery; a false positive powers a vehicle up believing it is airborne. The
// arming interlock is what makes the latter survivable, but this policy is the
// first line and is tested as if it were the only one.

#include <gtest/gtest.h>

#include "flight_token_policy.h"

using FlightTokenPolicy::shouldAutoRestore;
using FlightTokenPolicy::kMaxPoweronRestores;
using FlightTokenPolicy::kTokNone;
using FlightTokenPolicy::kTokArmed;
using FlightTokenPolicy::kTokInflight;

namespace {
// The healthy in-flight row, so each test below varies exactly one thing.
bool restore(uint8_t state = kTokInflight, uint8_t attempts = 0,
             bool deliberate_off = false, bool tier1_stood_down = false,
             bool nvs_writable = true, bool magic_ok = true,
             bool crc_ok = true)
{
    return shouldAutoRestore(magic_ok, crc_ok, state, attempts,
                             deliberate_off, tier1_stood_down, nvs_writable);
}
}  // namespace

TEST(FlightTokenPolicy, AnInterruptedFlightRestores)
{
    // The 2026-08-29 case: a power cut under boost, the token still says a
    // flight is in progress, and nothing else objects.
    EXPECT_TRUE(restore());
}

TEST(FlightTokenPolicy, TheResetReasonIsNotAnInput)
{
    // Stated as a test because it is the whole point of the policy. The
    // signature carries no reset reason, so a POWERON and a BROWNOUT that the
    // S3 cannot distinguish (both 0x01 in its reset-cause register) reach the
    // identical verdict. If someone adds a reason parameter, this stops
    // compiling, which is the intent.
    EXPECT_TRUE(restore());
}

TEST(FlightTokenPolicy, ArmedNeverRestores)
{
    // PRELAUNCH reached but never launched: a scrubbed pad session. The board
    // must boot normally so the operator gets an ordinary session, not a
    // vehicle that powers itself up believing it is flying.
    EXPECT_FALSE(restore(kTokArmed));
}

TEST(FlightTokenPolicy, NoTokenNeverRestores)
{
    EXPECT_FALSE(restore(kTokNone));
}

TEST(FlightTokenPolicy, DeliberatePowerOffStandsDown)
{
    // The operator's power-off is IMPLEMENTED as a reboot, so booting rail-low
    // is load-bearing for it. Restoring here would undo the command that was
    // just given. Same row, and same reasoning, as RailRestorePolicy's.
    EXPECT_FALSE(restore(kTokInflight, 0, /*deliberate_off=*/true));
    // ...even with everything else pointing at a restore.
    EXPECT_FALSE(restore(kTokInflight, 0, true, false, true, true, true));
}

TEST(FlightTokenPolicy, GarbageOrCorruptTokenRefuses)
{
    EXPECT_FALSE(restore(kTokInflight, 0, false, false, true,
                         /*magic_ok=*/false, /*crc_ok=*/true));
    EXPECT_FALSE(restore(kTokInflight, 0, false, false, true,
                         /*magic_ok=*/true, /*crc_ok=*/false));
}

TEST(FlightTokenPolicy, TheTwoCrashLoopBoundsCompose)
{
    // Tier 1 having exhausted its own restore budget must not be escapable by
    // falling through to tier 2. A sagging pack that browns the OC out
    // repeatedly is exactly the case both bounds exist for, and without this
    // the second silently re-arms the first.
    EXPECT_FALSE(restore(kTokInflight, 0, false, /*tier1_stood_down=*/true));
}

TEST(FlightTokenPolicy, WriteFailureRefusesBecauseTheBoundIsAWrite)
{
    // Fail closed in the WRITE direction too. The attempt counter is the only
    // thing bounding repeated restores; if it cannot be incremented and read
    // back, the bound does not exist, and restoring with a void budget is how
    // a board ends up cycling indefinitely.
    EXPECT_FALSE(restore(kTokInflight, 0, false, false, /*nvs_writable=*/false));
}

TEST(FlightTokenPolicy, TheRestoreBudgetIsBounded)
{
    for (uint8_t a = 0; a < kMaxPoweronRestores; ++a)
    {
        EXPECT_TRUE(restore(kTokInflight, a)) << "attempt " << (int)a;
    }
    EXPECT_FALSE(restore(kTokInflight, kMaxPoweronRestores));
    EXPECT_FALSE(restore(kTokInflight, (uint8_t)(kMaxPoweronRestores + 1)));
    EXPECT_FALSE(restore(kTokInflight, 255));
}

TEST(FlightTokenPolicy, EveryRefusalHoldsIndependently)
{
    // No single healthy input can rescue a row that any other input refuses:
    // the refusals are conjunctive, not a vote. Swept exhaustively because the
    // cost of one accidental `||` here is a vehicle powering itself up on the
    // ground believing it is in flight.
    for (int bits = 0; bits < 64; ++bits)
    {
        const bool magic  = bits & 1;
        const bool crc    = bits & 2;
        const bool delib  = bits & 4;
        const bool tier1  = bits & 8;
        const bool writ   = bits & 16;
        const bool inflt  = bits & 32;
        const bool want = !delib && !tier1 && writ && magic && crc && inflt;
        EXPECT_EQ(want, shouldAutoRestore(magic, crc,
                                          inflt ? kTokInflight : kTokArmed,
                                          0, delib, tier1, writ))
            << "bits " << bits;
    }
}
