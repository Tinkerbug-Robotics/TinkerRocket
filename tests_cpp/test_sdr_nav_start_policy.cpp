// The V10's gnss-sdr-dev image's start without the board, and a late board's
// take-over (flight_computer/main/sdr_nav_start_policy.h): the degraded start
// once no usable fix came 2 min after boot on the pad, at once in PRELAUNCH,
// never in flight, before the gyro is averaged or within the settle; a late
// fix restarts it anywhere on the pad (READY or PRELAUNCH, #1762's follow-up);
// the boot's verdict on a silent board.
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

TEST(SdrNavStart, InPrelaunchItStartsAtOnce)
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

TEST(SdrNavStart, ALateFixRestartsItAnywhereOnThePad)
{
    EXPECT_TRUE(restartOnFix(true, true, READY, true));
    EXPECT_TRUE(restartOnFix(true, true, PRELAUNCH, true));  // the FC's own step, no arm
    EXPECT_FALSE(restartOnFix(true, true, PRELAUNCH, false)); // not still: carried, or a lit motor
    EXPECT_FALSE(restartOnFix(true, false, READY, true));    // no usable fix yet
    EXPECT_FALSE(restartOnFix(false, true, READY, true));    // not a degraded start
    for (RocketState s : {INFLIGHT, LANDED, MAG_CALIBRATION, INITIALIZATION})
        EXPECT_FALSE(restartOnFix(true, true, s, true)) << int(s);   // flying: it stands
}

TEST(SdrNavStart, TheBoardIsSilentOnlyAfterItsSecondsWithNoFrame)
{
    EXPECT_FALSE(boardSilent(false, kHeardMs - 1));
    EXPECT_TRUE(boardSilent(false, kHeardMs));
    EXPECT_FALSE(boardSilent(true, 10u * kHeardMs));
    EXPECT_GE(kHeardMs, 2000u);   // its status comes once a second
}

// ---- #1756: a re-init on the pad waits for the board's next fix --------------

TEST(SdrNavStart, TheBoardIsWaitedForAfterAUsableFixAndAfterAReinit)
{
    EXPECT_FALSE(boardWait(0, 0, 50000u));                       // never a fix, never a re-init
    EXPECT_TRUE(boardWait(10000u, 0, 10000u + kUsableFixMs - 1));
    EXPECT_FALSE(boardWait(10000u, 0, 10000u + kUsableFixMs));
    EXPECT_TRUE(boardWait(0, 20000u, 20000u + kReinitGraceMs - 1));
    EXPECT_FALSE(boardWait(0, 20000u, 20000u + kReinitGraceMs));
    // across the 32-bit millisecond wrap
    EXPECT_TRUE(boardWait(0xFFFFFF00u, 0, 0x00000100u));
    EXPECT_TRUE(boardWait(0, 0xFFFFFF00u, 0x00000100u));
    EXPECT_EQ(kUsableFixMs, 3000u);
    EXPECT_EQ(kReinitGraceMs, 5000u);
}

namespace {

// main.cpp's pass order for the gnss-sdr-dev board's filter start, at 1 kHz.
// `fix`: the board's fix passed the init gate this pass. It is usable (what
// the normal start takes) with the pad datum, which the pad average sets on
// such a fix unless a degraded start flies (fixesHeldOut).  Then the stamp
// of the last usable fix; the late board's restart; MAG_CALIBRATION
// does nothing; an unstarted filter stamps a re-init of a filter that flew on
// the board, then starts normally on a usable fix or degraded when
// degradedStartDue says so, and says which (gnss_absent_flight, `degraded`,
// is the start's, #1762's follow-up).  magCalExit() is magCalBackToReady.  `old_wiring`
// is main.cpp before #1756: degradedStartDue told of a usable fix on THIS
// pass, and no stamps.
struct Pad
{
    RocketState state = READY;
    bool     ekf = false, degraded = false, on_board = false, ever_degraded = false, have_ref = false;
    uint32_t last_usable = 0, reinit = 0, degraded_at = 0;
    bool     old_wiring = false;

    void pass(uint32_t now, bool fix)
    {
        if (fix && !fixesHeldOut(degraded, state)) have_ref = true;
        const bool usable = fix && have_ref;
        if (usable) last_usable = now ? now : 1u;
        if (ekf && degraded && restartOnFix(true, usable, state, /*still=*/true))
        {
            ekf = false;
            degraded = false;
        }
        if (state == MAG_CALIBRATION) return;
        if (!ekf)
        {
            if (on_board && !old_wiring)
            {
                on_board = false;
                reinit = now ? now : 1u;
            }
            const bool wait = old_wiring ? usable : (usable || boardWait(last_usable, reinit, now));
            const bool deg = degradedStartDue(wait, true, true, now, state);
            if (usable || deg)
            {
                ekf = true;
                on_board = !deg;
                degraded = deg;
                if (deg && !ever_degraded)
                {
                    ever_degraded = true;
                    degraded_at = now;
                }
            }
        }
    }

    void magCalExit(uint32_t now)
    {
        state = READY;
        if (!old_wiring && (on_board || reinit != 0)) reinit = now ? now : 1u;
        ekf = false;
    }
};

// Run [from, to) with a usable fix every `fix_ms` (phase `phase_ms`), or none.
void run(Pad& p, uint32_t from, uint32_t to, uint32_t fix_ms, uint32_t phase_ms = 0, bool fixes = true)
{
    for (uint32_t t = from; t < to; t++) p.pass(t, fixes && fix_ms && (t % fix_ms) == phase_ms);
}

// Started on the board in READY, PRELAUNCH at 60 s, running to `at`.
Pad prelaunchOnTheBoard(uint32_t fix_ms, uint32_t at, bool old_wiring = false)
{
    Pad p;
    p.old_wiring = old_wiring;
    run(p, 5000, 60000, fix_ms);
    p.state = PRELAUNCH;
    run(p, 60000, at, fix_ms);
    return p;
}

} // namespace

TEST(SdrNavStartReinit, AReinitBetweenFixesInPrelaunchStartsNormallyOnTheNextFix)
{
    // An orientation re-snap or setting change in PRELAUNCH, at any
    // phase between two fixes, at 1 Hz and 10 Hz.
    for (uint32_t fix_ms : {1000u, 100u})
        for (uint32_t phase : {1u, fix_ms / 3, fix_ms - 1})
        {
            Pad p = prelaunchOnTheBoard(fix_ms, 300000u + phase);
            ASSERT_TRUE(p.ekf && p.on_board);
            p.ekf = false;   // the re-init
            run(p, 300000u + phase, 310000u, fix_ms);
            EXPECT_FALSE(p.ever_degraded) << fix_ms << " ms fixes, phase " << phase;
            EXPECT_TRUE(p.ekf && p.on_board && !p.degraded);
        }
}

TEST(SdrNavStartReinit, Before1756TheSameReinitStartedDegraded)
{
    for (uint32_t fix_ms : {1000u, 100u})
    {
        Pad p = prelaunchOnTheBoard(fix_ms, 300000u + fix_ms / 3, /*old_wiring=*/true);
        p.ekf = false;
        run(p, 300000u + fix_ms / 3, 310000u, fix_ms);
        EXPECT_TRUE(p.ever_degraded) << fix_ms;
        // in PRELAUNCH it stood for the flight until #1762's follow-up; the
        // board's next fix now starts it over
        EXPECT_FALSE(p.degraded) << fix_ms;
        EXPECT_TRUE(p.ekf && p.on_board);
    }
}

TEST(SdrNavStartReinit, AMagCalThatTumblesTheRocketStartsNormallyOnceItIsSetDown)
{
    // The session runs 30 s with fixes that fail the init gate (the rocket is
    // being turned); its exit clears the filter and returns READY, and the
    // pad's state machine re-arms on the next pass. The board's fixes are
    // usable again 1.5 s after the exit. At 10 min after boot, where READY's
    // own 2 min rule would also have started degraded at the exit.
    for (uint32_t fix_ms : {1000u, 100u})
        for (bool old_wiring : {false, true})
        {
            Pad p = prelaunchOnTheBoard(fix_ms, 600000u, old_wiring);
            p.state = MAG_CALIBRATION;
            run(p, 600000u, 630000u, fix_ms, 0, /*fixes=*/false);
            p.magCalExit(630000u);
            p.pass(630000u, false);
            p.state = PRELAUNCH;   // PRELAUNCH again on the next pass
            run(p, 630001u, 631500u, fix_ms, 0, false);
            run(p, 631500u, 640000u, fix_ms);
            if (old_wiring)
                EXPECT_TRUE(p.ever_degraded) << fix_ms;
            else
            {
                EXPECT_FALSE(p.ever_degraded) << fix_ms;
                EXPECT_TRUE(p.ekf && p.on_board);
            }
        }
}

TEST(SdrNavStartReinit, ABoardThatWentSilentStillStartsDegradedAfterTheGrace)
{
    Pad p = prelaunchOnTheBoard(1000u, 300000u);
    // The board's last fix at 299 s; it goes silent, then a re-init at 300 s.
    p.ekf = false;
    run(p, 300000u, 320000u, 1000u, 0, /*fixes=*/false);
    EXPECT_TRUE(p.ever_degraded);
    EXPECT_EQ(p.degraded_at, 300000u + kReinitGraceMs);
    EXPECT_TRUE(p.ekf && p.degraded);
}

TEST(SdrNavStartReinit, ABoardThatNeverGaveAFixStillStartsAtOnceWhenArmed)
{
    Pad p;
    p.state = PRELAUNCH;
    run(p, 5000u, 6000u, 1000u, 0, /*fixes=*/false);
    EXPECT_TRUE(p.ever_degraded);
    EXPECT_EQ(p.degraded_at, 5000u);
}

TEST(SdrNavStartReinit, AReSnapThenAMagCalBeforeTheNextFixStillWaitsForTheBoard)
{
    // An orientation re-snap in PRELAUNCH, then a mag cal started before the board's
    // next fix: the re-snap's grace runs out during the session, whose fixes
    // fail the init gate (the rocket is being turned). Its exit is a re-init
    // of its own, so the grace starts again there.
    for (uint32_t fix_ms : {1000u, 100u})
    {
        Pad p = prelaunchOnTheBoard(fix_ms, 300000u + fix_ms / 3);
        p.ekf = false;                    // the re-snap
        p.pass(300000u + fix_ms / 3, false);
        p.state = MAG_CALIBRATION;        // before the next fix
        run(p, 300000u + fix_ms / 3 + 1, 330000u, fix_ms, 0, /*fixes=*/false);
        p.magCalExit(330000u);
        p.pass(330000u, false);
        p.state = PRELAUNCH;
        run(p, 330001u, 331500u, fix_ms, 0, false);
        run(p, 331500u, 340000u, fix_ms);
        EXPECT_FALSE(p.ever_degraded) << fix_ms;
        EXPECT_TRUE(p.ekf && p.on_board);
    }
}
