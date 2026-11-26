// #1762: the navigation filter's start without GNSS, on every receiver
// (flight_computer/main/nav_start_policy.h).
//
// The load-bearing rows:
//   - a receiver that failed its bring-up starts degraded at once, as
//     GNSS-absent mode always has;
//   - one that answers but gives no usable fix starts degraded 2 min after
//     boot on the pad, or at once when armed, never in flight;
//   - a later usable fix restarts it, on the pad unarmed only;
//   - the three routes the review found (no fix ever, a 3-D fix held at 4
//     satellites, a fix lost before a pad re-init) all end with a filter;
//     under the old rule (GNSS-absent mode, and only before any fix) none did.

#include <gtest/gtest.h>

#include "nav_start_policy.h"

using namespace nav_start;

TEST(NavStart, AReceiverThatFailedBringUpStartsAtOnce) {
    EXPECT_TRUE(degradedStartDue(false, true, true, 6000, READY, /*receiver_absent=*/true));
    EXPECT_TRUE(degradedStartDue(false, true, true, 6000, PRELAUNCH, true));
}

TEST(NavStart, AnAnsweringReceiverWithNoUsableFixWaitsTwoMinutesOrTheArm) {
    EXPECT_FALSE(degradedStartDue(false, true, true, kNoFixMs - 1, READY, false));
    EXPECT_TRUE(degradedStartDue(false, true, true, kNoFixMs, READY, false));
    EXPECT_TRUE(degradedStartDue(false, true, true, 6000, PRELAUNCH, false)) << "armed: at once";
}

TEST(NavStart, NeverWithAFixToStartOnWithoutTheGyroOrBeforeTheSettle) {
    for (bool absent : {false, true}) {
        EXPECT_FALSE(degradedStartDue(true, true, true, kNoFixMs, PRELAUNCH, absent));
        EXPECT_FALSE(degradedStartDue(false, false, true, kNoFixMs, PRELAUNCH, absent));
        EXPECT_FALSE(degradedStartDue(false, true, false, kNoFixMs, PRELAUNCH, absent));
    }
}

TEST(NavStart, InFlightOnlyAnAbsentReceiversStartIsUnchanged) {
    // An answering receiver never starts degraded in flight (the accelerometer
    // is no measure of the tilt there); GNSS-absent mode's rule never asked.
    EXPECT_FALSE(degradedStartDue(false, true, true, kNoFixMs * 10, INFLIGHT, false));
    EXPECT_TRUE(degradedStartDue(false, true, true, kNoFixMs * 10, INFLIGHT, true));
}

TEST(NavStart, ALateFixRestartsItOnThePadUnarmedOnly) {
    EXPECT_TRUE(restartOnFix(true, true, READY));
    EXPECT_FALSE(restartOnFix(true, true, PRELAUNCH));
    EXPECT_FALSE(restartOnFix(true, true, INFLIGHT));
    EXPECT_FALSE(restartOnFix(true, false, READY));
    EXPECT_FALSE(restartOnFix(false, true, READY));
}

TEST(NavStart, TheReceiverIsWaitedForAfterAUsableFixAndAfterAReinit) {
    EXPECT_FALSE(receiverWait(0, 0, 100000));
    EXPECT_TRUE(receiverWait(100000, 0, 100000 + kUsableFixMs - 1));
    EXPECT_FALSE(receiverWait(100000, 0, 100000 + kUsableFixMs));
    EXPECT_TRUE(receiverWait(0, 100000, 100000 + kReinitGraceMs - 1));
    EXPECT_FALSE(receiverWait(0, 100000, 100000 + kReinitGraceMs));
    EXPECT_TRUE(receiverWait(0xFFFFFF00u, 0, 0x100u)) << "across the wrap";
}

// ---------------------------------------------------------------------------
// main.cpp's pass order at 1 kHz (as test_sdr_nav_start_policy.cpp models it
// for the board): `usable` is a fix the normal start takes (the init gate and
// the pad datum); `fix3d` is any 3-D fix, which latches gnss_started. The
// last usable fix is stamped, a degraded start gives way to a usable fix on
// the pad unarmed, an unstarted filter stamps a re-init of a filter that flew
// on fixes, then starts normally or degraded. `old_rule` is main.cpp before
// #1762: degraded only in GNSS-absent mode and before any 3-D fix.
// ---------------------------------------------------------------------------

namespace {
struct Pad
{
    RocketState state = READY;
    bool     absent = false, old_rule = false;
    bool     ekf = false, degraded = false, on_fixes = false, started_3d = false;
    uint32_t last_usable = 0, reinit = 0;

    void pass(uint32_t now, bool usable, bool fix3d)
    {
        if (fix3d) started_3d = true;
        if (usable) last_usable = now ? now : 1u;
        if (ekf && degraded && restartOnFix(true, usable, state)) { ekf = false; degraded = false; }
        if (state == MAG_CALIBRATION) return;
        if (!ekf)
        {
            if (on_fixes) { on_fixes = false; reinit = now ? now : 1u; }
            const bool deg = old_rule
                ? (!usable && absent && !started_3d && now >= 5000)
                : degradedStartDue(usable || receiverWait(last_usable, reinit, now), true, now >= 5000, now, state,
                                   absent);
            if (usable || deg) { ekf = true; on_fixes = !deg; degraded = deg; }
        }
    }
    void reinitNow() { ekf = false; }   // an orientation re-snap or setting change
};

// [from, to): a usable fix every 100 ms while `usable`, a 3-D fix every 100 ms while `fix3d`
void run(Pad& p, uint32_t from, uint32_t to, bool usable, bool fix3d)
{
    for (uint32_t t = from; t < to; ++t) {
        const bool tick = t % 100 == 0;
        p.pass(t, usable && tick, (usable || fix3d) && tick);
    }
}
}  // namespace

TEST(NavStartRoutes, AReceiverThatNeverFixesStartsTwoMinutesAfterBoot) {
    for (bool old_rule : {false, true}) {
        Pad p;
        p.old_rule = old_rule;
        run(p, 0, kNoFixMs + 1000, false, false);
        if (old_rule) EXPECT_FALSE(p.ekf) << "before #1762: no filter";
        else          EXPECT_TRUE(p.ekf && p.degraded);
    }
}

TEST(NavStartRoutes, ItsLateUsableFixOnThePadStartsItOverNormally) {
    Pad p;
    run(p, 0, kNoFixMs + 1000, false, false);
    ASSERT_TRUE(p.degraded);
    run(p, kNoFixMs + 1000, kNoFixMs + 2000, true, true);
    EXPECT_TRUE(p.ekf);
    EXPECT_FALSE(p.degraded);
}

TEST(NavStartRoutes, AFixHeldAtFourSatellitesStartsDegradedWhenArmed) {
    // PRELAUNCH takes 4 satellites, the start 5: armed at 60 s.
    for (bool old_rule : {false, true}) {
        Pad p;
        p.old_rule = old_rule;
        run(p, 0, 60000, false, true);
        p.state = PRELAUNCH;
        run(p, 60000, 61000, false, true);
        if (old_rule) EXPECT_FALSE(p.ekf) << "before #1762: no filter";
        else          EXPECT_TRUE(p.ekf && p.degraded);
    }
}

TEST(NavStartRoutes, AFixLostBeforeAPadReinitStartsDegradedAfterTheGrace) {
    // Usable fixes, then none (the fix lost); armed; then a re-snap.
    for (bool old_rule : {false, true}) {
        Pad p;
        p.old_rule = old_rule;
        run(p, 0, 50000, true, true);
        ASSERT_TRUE(p.ekf && !p.degraded);
        run(p, 50000, 55000, false, false);
        p.state = PRELAUNCH;
        p.reinitNow();
        run(p, 55000, 55000 + kReinitGraceMs + 1000, false, false);
        if (old_rule) EXPECT_FALSE(p.ekf) << "before #1762: no filter";
        else          EXPECT_TRUE(p.ekf && p.degraded);
    }
}

TEST(NavStartRoutes, AReceiverThatFailedBringUpStillStartsAtOnce) {
    for (bool old_rule : {false, true}) {
        Pad p;
        p.absent   = true;
        p.old_rule = old_rule;
        run(p, 0, 6000, false, false);
        EXPECT_TRUE(p.ekf && p.degraded) << (old_rule ? "old" : "new");
    }
}

TEST(NavStartRoutes, AGoodReceiverNeverStartsDegraded) {
    Pad p;
    run(p, 0, 50000, true, true);
    p.state = PRELAUNCH;
    run(p, 50000, 300000, true, true);
    EXPECT_TRUE(p.ekf);
    EXPECT_FALSE(p.degraded);
}
