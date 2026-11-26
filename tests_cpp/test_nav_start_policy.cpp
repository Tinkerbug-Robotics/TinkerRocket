// #1762: the navigation filter's start without GNSS, on every receiver
// (flight_computer/main/nav_start_policy.h).
//
// The load-bearing rows:
//   - a receiver that failed its bring-up starts degraded at once, as
//     GNSS-absent mode always has;
//   - one that answers but gives no usable fix starts degraded 2 min after
//     boot in READY, or at once in PRELAUNCH, never in flight;
//   - a later usable fix restarts it anywhere on the pad, READY or PRELAUNCH
//     (the follow-up: PRELAUNCH is the FC's own step on a 4-satellite fix, no
//     arm), once the rocket has been still for a second (not carried, no
//     motor lit before launch is called), and the receiver's fixes build the
//     pad datum there, which the restart needs; once it flies they stay out;
//   - the three routes the review found (no fix ever, a 3-D fix held at 4
//     satellites, a fix lost before a pad re-init) all end with a filter;
//     under the old rule (GNSS-absent mode, and only before any fix) none did;
//   - a cold start whose first fixes have 4 satellites ends on fixes (the
//     merged #1762 left it degraded for the flight);
//   - a normal start after a degraded one's filter was cleared is no degraded
//     start; a reboot in flight keeps a degraded one (the snapshot's byte).

#include <gtest/gtest.h>

#include <cmath>

#include "nav_start_policy.h"

using namespace nav_start;

TEST(NavStart, AReceiverThatFailedBringUpStartsAtOnce) {
    EXPECT_TRUE(degradedStartDue(false, true, true, 6000, READY, /*receiver_absent=*/true));
    EXPECT_TRUE(degradedStartDue(false, true, true, 6000, PRELAUNCH, true));
}

TEST(NavStart, AnAnsweringReceiverWithNoUsableFixWaitsTwoMinutesOrPrelaunch) {
    EXPECT_FALSE(degradedStartDue(false, true, true, kNoFixMs - 1, READY, false));
    EXPECT_TRUE(degradedStartDue(false, true, true, kNoFixMs, READY, false));
    EXPECT_TRUE(degradedStartDue(false, true, true, 6000, PRELAUNCH, false)) << "launch may come: at once";
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

TEST(NavStart, ALateFixRestartsItAnywhereOnThePadOnceStill) {
    EXPECT_TRUE(restartOnFix(true, true, READY, true));
    EXPECT_TRUE(restartOnFix(true, true, PRELAUNCH, true)) << "no arm: the FC's own step on a 4-satellite fix";
    for (RocketState s : {INFLIGHT, LANDED, MAG_CALIBRATION, INITIALIZATION})
        EXPECT_FALSE(restartOnFix(true, true, s, true)) << int(s);
    EXPECT_FALSE(restartOnFix(true, false, READY, true));
    EXPECT_FALSE(restartOnFix(false, true, READY, true));
    EXPECT_FALSE(restartOnFix(false, true, PRELAUNCH, true));
    EXPECT_FALSE(restartOnFix(true, true, READY, false)) << "carried";
    EXPECT_FALSE(restartOnFix(true, true, PRELAUNCH, false)) << "a lit motor before launch is called";
}

// ---- RestWatch: the rest a restart waits for (judged as #1755's pad tests) --

namespace {
enum class Motion { Quiet, Sway4, Sway7, Offset, Noise, Bounce, Carried, Stomped, Turned, Thrust, LowThrust, Heavy };

// One 1 kHz IMU pass, EKF frame (m/s^2, deg/s), nose up (+X).
void imuAt(Motion m, uint32_t t_ms, double out[6])
{
    const double t = t_ms * 1e-3, w = 2.0 * M_PI * 2.0;   // 2 Hz
    double a[3] = {kStandardGravityMps2, 0, 0}, g[3] = {0.3, -0.2, 0.1};   // a small gyro bias
    switch (m) {
    case Motion::Quiet: break;
    case Motion::Sway4: g[1] += 4.0 * cos(w * t); break;                   // #1755's measured rail sway
    case Motion::Sway7: g[1] += 7.5 * cos(w * t); g[2] += 7.5 * sin(w * t); break;
    case Motion::Offset: a[0] += 1.0; break;                               // an uncalibrated accelerometer
    case Motion::Noise: a[0] += 0.3 * sin(t_ms * 1.7); g[0] += 2.0 * sin(t_ms * 2.3); break;
    case Motion::Carried: g[1] += 40.0 * cos(w * t); a[2] += 1.0 * sin(w * t); break;
    case Motion::Bounce: a[0] += 1.5 * sin(w * t); break;                  // along gravity: tilts nothing
    case Motion::Stomped: a[0] += 3.0 * sin(w * t); break;                 // steps harder than 0.25 g
    case Motion::Turned: g[2] += 6.0; break;                               // a rail raised, a slow turn
    case Motion::Thrust: a[0] = 1.3 * kStandardGravityMps2; break;
    case Motion::LowThrust: break;                                         // under the weight: the pad holds it
    case Motion::Heavy: a[0] += 2.2; break;                                // a steady 1.22 g no rocket at rest reads
    }
    out[0] = a[0]; out[1] = a[1]; out[2] = a[2]; out[3] = g[0]; out[4] = g[1]; out[5] = g[2];
}

void feedFor(RestWatch& r, Motion m, uint32_t from, uint32_t to)
{
    double v[6];
    for (uint32_t t = from; t < to; ++t) { imuAt(m, t, v); r.feed(t, v[0], v[1], v[2], v[3], v[4], v[5]); }
}
}  // namespace

TEST(NavStartRest, AQuietPadIsAtRestAfterASecond) {
    RestWatch r;
    feedFor(r, Motion::Quiet, 5000, 5000 + kRestartStillMs - 10);
    EXPECT_FALSE(r.atRest());
    feedFor(r, Motion::Quiet, 5000 + kRestartStillMs - 10, 5000 + kRestartStillMs + kRestBlockMs);
    EXPECT_TRUE(r.atRest());
    EXPECT_EQ(kRestartStillMs, 1000u);
}

TEST(NavStartRest, RailSwayAnOffsetAccelerometerAndNoiseAreAtRest) {
    for (Motion m : {Motion::Sway4, Motion::Sway7, Motion::Offset, Motion::Noise, Motion::Bounce, Motion::LowThrust}) {
        RestWatch r;
        feedFor(r, m, 0, 3000);
        EXPECT_TRUE(r.atRest()) << int(m);
    }
}

TEST(NavStartRest, CarriedStompedTurnedOrThrustIsNot) {
    for (Motion m : {Motion::Carried, Motion::Stomped, Motion::Turned, Motion::Thrust, Motion::Heavy}) {
        RestWatch r;
        feedFor(r, m, 0, 10000);
        EXPECT_FALSE(r.atRest()) << int(m);
    }
}

TEST(NavStartRest, ThrustEndsTheRestOnItsFirstPass) {
    RestWatch r;
    feedFor(r, Motion::Quiet, 0, 5000);
    ASSERT_TRUE(r.atRest());
    feedFor(r, Motion::Thrust, 5000, 5001);
    EXPECT_FALSE(r.atRest()) << "one pass over ~1.25 g";
    EXPECT_LT(kRestVetoMps2, 20.0 - kStandardGravityMps2) << "under the 2 g launch counts";
}

TEST(NavStartRest, PickedUpEndsTheRestWithinTheMotionsFirstTensOfMs) {
    for (uint32_t phase : {0u, 60u, 125u, 190u}) {   // anywhere in a block
        RestWatch r;
        feedFor(r, Motion::Quiet, 0, 5000 + phase);
        ASSERT_TRUE(r.atRest());
        uint32_t t = 5000 + phase;
        double v[6];
        while (r.atRest() && t < 6000) { imuAt(Motion::Carried, t, v); r.feed(t, v[0], v[1], v[2], v[3], v[4], v[5]); ++t; }
        EXPECT_LT(t - (5000 + phase), 150u) << phase;
    }
}

TEST(NavStartRest, ASlowTurnEndsARestAtItsBlocksEnd) {
    // Its spread passes; only a block's mean rate fails it, at that block's
    // end: within two blocks of the turn's start (the first may be mostly
    // still).
    RestWatch r;
    feedFor(r, Motion::Quiet, 0, 5000);
    ASSERT_TRUE(r.atRest());
    feedFor(r, Motion::Turned, 5000, 5000 + 2 * (kRestBlockMs + 1) + 1);
    EXPECT_FALSE(r.atRest());
}

TEST(NavStartRest, OneBadBlockStartsTheSecondOver) {
    RestWatch r;
    feedFor(r, Motion::Quiet, 0, 5000);
    ASSERT_TRUE(r.atRest());
    feedFor(r, Motion::Carried, 5000, 5300);   // picked up for a block
    EXPECT_FALSE(r.atRest());
    feedFor(r, Motion::Quiet, 5300, 5300 + kRestartStillMs - kRestBlockMs);
    EXPECT_FALSE(r.atRest());
    feedFor(r, Motion::Quiet, 5300 + kRestartStillMs - kRestBlockMs, 5300 + kRestartStillMs + 2 * kRestBlockMs);
    EXPECT_TRUE(r.atRest());
}

TEST(NavStart, ItsFixesBuildThePadDatumUntilItFlies) {
    for (RocketState s : {READY, PRELAUNCH, MAG_CALIBRATION, INITIALIZATION})
        EXPECT_FALSE(fixesHeldOut(true, s)) << int(s);
    EXPECT_TRUE(fixesHeldOut(true, INFLIGHT));
    EXPECT_TRUE(fixesHeldOut(true, LANDED));
    for (RocketState s : {READY, PRELAUNCH, INFLIGHT, LANDED})
        EXPECT_FALSE(fixesHeldOut(false, s)) << "a start on fixes: " << int(s);
}

TEST(NavStart, TheSnapshotByteKeepsADegradedStartThroughAReboot) {
    EXPECT_EQ(snapshotFilterByte(false, false), 0u);
    EXPECT_EQ(snapshotFilterByte(false, true), 0u) << "no filter: nothing to keep";
    EXPECT_EQ(snapshotFilterByte(true, false), 1u) << "every older writer's started";
    EXPECT_EQ(snapshotFilterByte(true, true), (unsigned)(SNAP_EKF_STARTED | SNAP_EKF_WITHOUT_GNSS));
    EXPECT_TRUE(restoredWithoutGnss(snapshotFilterByte(true, true)));
    EXPECT_FALSE(restoredWithoutGnss(snapshotFilterByte(true, false)));
    EXPECT_FALSE(restoredWithoutGnss(0u));
    EXPECT_FALSE(restoredWithoutGnss(1u)) << "an older snapshot: started on fixes, as restored before";
    EXPECT_FALSE(restoredWithoutGnss(SNAP_EKF_WITHOUT_GNSS)) << "not started";
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
// for the board). `gate3` is a fix through the init gate (5 satellites, under
// 2 m/s); `fix3d` is any 3-D fix, which latches gnss_started. The pad datum
// takes a gate-3 fix unless fixesHeldOut; a fix is usable (the normal start
// takes it) with the datum. The last usable fix is stamped, a degraded start
// gives way to a usable fix on the pad, an unstarted filter stamps a re-init
// of a filter that flew on fixes, then starts normally or degraded and says
// which (gnss_absent_flight = the start's). `motion`: what the IMU reads,
// fed to the RestWatch every pass as main.cpp feeds it. Last, with
// `auto_prelaunch`, the
// state machine: READY -> PRELAUNCH 3 s into a run of fresh 3-D fixes (fresh:
// one in the last 2 s), the OC up. `old_rule` is main.cpp before #1762:
// degraded only in GNSS-absent mode and before any 3-D fix; `merged` the
// merged #1762 (76e10d7b): the restart in READY only, the datum held out of a
// degraded start off READY, and gnss_absent_flight set by a degraded start
// alone. Both keep those holds (the board's image had them before #1762).
// ---------------------------------------------------------------------------

namespace {
struct Pad
{
    RocketState state = READY;
    bool     absent = false, old_rule = false, merged = false, auto_prelaunch = false;
    bool     ekf = false, degraded = false, on_fixes = false, started_3d = false, have_ref = false;
    bool     fresh_prev = false;
    Motion   motion = Motion::Quiet;
    RestWatch rest;
    uint32_t last_usable = 0, reinit = 0, last_3d = 0, dwell_from = 0;
    int      restarts = 0;

    void pass(uint32_t now, bool gate3, bool fix3d)
    {
        if (fix3d) { started_3d = true; last_3d = now; }
        double v[6];
        imuAt(motion, now, v);
        rest.feed(now, v[0], v[1], v[2], v[3], v[4], v[5]);
        const bool old_holds = old_rule || merged;
        const bool held = old_holds ? (degraded && state != READY) : fixesHeldOut(degraded, state);
        if (gate3 && !held) have_ref = true;
        const bool usable = gate3 && have_ref;
        if (usable) last_usable = now ? now : 1u;
        const bool restart = old_holds ? (degraded && usable && state == READY)
                                       : restartOnFix(degraded, usable, state, rest.atRest());
        if (ekf && degraded && restart) { ekf = false; degraded = false; ++restarts; }
        if (state != MAG_CALIBRATION && !ekf)
        {
            if (on_fixes) { on_fixes = false; reinit = now ? now : 1u; }
            const bool deg = old_rule
                ? (!usable && absent && !started_3d && now >= 5000)
                : degradedStartDue(usable || receiverWait(last_usable, reinit, now), true, now >= 5000, now, state,
                                   absent);
            if (usable || deg)
            {
                ekf = true;
                on_fixes = !deg;
                if (old_holds) degraded = degraded || deg;
                else           degraded = deg;
            }
        }
        if (auto_prelaunch && state == READY)
        {
            const bool fresh = started_3d && now - last_3d < 2000u;
            if (fresh && !fresh_prev) dwell_from = now;
            fresh_prev = fresh;
            if (fresh && now > dwell_from + 3000u) state = PRELAUNCH;
        }
    }
    void reinitNow() { ekf = false; }   // an orientation re-snap or setting change
};

// [from, to): a gate-3 fix every 100 ms while `gate3`, a 3-D fix every 100 ms while `fix3d`
void run(Pad& p, uint32_t from, uint32_t to, bool gate3, bool fix3d)
{
    for (uint32_t t = from; t < to; ++t) {
        const bool tick = t % 100 == 0;
        p.pass(t, gate3 && tick, (gate3 || fix3d) && tick);
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

TEST(NavStartRoutes, AFixHeldAtFourSatellitesStartsDegradedInPrelaunch) {
    // PRELAUNCH takes 4 satellites, the start 5: the FC enters it on its own
    // 3 s into the fixes.
    for (bool old_rule : {false, true}) {
        Pad p;
        p.old_rule = old_rule;
        p.auto_prelaunch = true;
        run(p, 0, 60000, false, true);
        ASSERT_EQ(p.state, PRELAUNCH);
        if (old_rule) EXPECT_FALSE(p.ekf) << "before #1762: no filter";
        else          EXPECT_TRUE(p.ekf && p.degraded);
    }
}

TEST(NavStartRoutes, AColdStartOnFourSatellitesEndsOnFixes) {
    // A cold start: 3-D fixes with 4 satellites from 20 s, so PRELAUNCH at
    // 23 s and a degraded start at once; usable fixes from 40 s. The datum
    // forms in PRELAUNCH and the first usable fix starts the filter over.
    // The merged #1762 held the datum out of a degraded start off READY and
    // restarted only in READY: degraded for the flight.
    for (bool merged : {false, true}) {
        Pad p;
        p.merged = merged;
        p.auto_prelaunch = true;
        run(p, 0, 20000, false, false);
        run(p, 20000, 40000, false, true);
        ASSERT_EQ(p.state, PRELAUNCH);
        ASSERT_TRUE(p.ekf && p.degraded) << "degraded at once in PRELAUNCH";
        run(p, 40000, 60000, true, true);
        if (merged) {
            EXPECT_TRUE(p.ekf && p.degraded) << "the merged #1762: degraded for the flight";
            EXPECT_FALSE(p.have_ref) << "and no datum";
        } else {
            EXPECT_TRUE(p.ekf && !p.degraded);
            EXPECT_TRUE(p.have_ref);
            EXPECT_EQ(p.restarts, 1);
        }
    }
}

TEST(NavStartRoutes, AFixLateInPrelaunchStartsItOverToo) {
    // No usable fix for 2 min (degraded in READY), PRELAUNCH on a 4-satellite
    // fix, then the first usable fix.
    Pad p;
    p.auto_prelaunch = true;
    run(p, 0, kNoFixMs + 1000, false, false);
    ASSERT_TRUE(p.degraded);
    run(p, kNoFixMs + 1000, kNoFixMs + 10000, false, true);
    ASSERT_EQ(p.state, PRELAUNCH);
    run(p, kNoFixMs + 10000, kNoFixMs + 11000, true, true);
    EXPECT_TRUE(p.ekf && !p.degraded);
}

TEST(NavStartRoutes, OnceItFliesTheDegradedStartStandsAndTheDatumTakesNothing) {
    Pad p;
    p.auto_prelaunch = true;
    run(p, 0, 10000, false, true);
    ASSERT_TRUE(p.degraded);
    p.state = INFLIGHT;
    run(p, 10000, 20000, true, true);
    EXPECT_TRUE(p.ekf && p.degraded);
    EXPECT_FALSE(p.have_ref);
    EXPECT_EQ(p.restarts, 0);
}

TEST(NavStartRoutes, ANormalStartAfterADegradedOnesFilterWasClearedIsNoDegradedStart) {
    // A datum from usable fixes, the fix lost, PRELAUNCH, a re-snap: degraded
    // after the grace. Then a second re-snap, and the receiver's usable fixes
    // come back: the next start is on a fix. The merged #1762 left
    // gnss_absent_flight set by the degraded start, and in PRELAUNCH nothing
    // cleared it: its fix-started classic filter fused the time-0 fix (0 N
    // 0 E, EkfDegradedStart in test_ekf.cpp), and the receiver's fixes stayed
    // out for the flight.
    for (bool merged : {false, true}) {
        Pad p;
        p.merged = merged;
        run(p, 0, 50000, true, true);
        run(p, 50000, 55000, false, false);
        p.state = PRELAUNCH;
        p.reinitNow();
        run(p, 55000, 62000, false, false);
        ASSERT_TRUE(p.ekf && p.degraded);
        p.reinitNow();
        run(p, 62000, 63000, true, true);
        ASSERT_TRUE(p.ekf && p.on_fixes) << "started on a fix";
        EXPECT_EQ(p.degraded, merged) << (merged ? "merged: still marked degraded" : "");
        EXPECT_EQ(p.restarts, 0) << "no restart needed: the start itself said it was on a fix";
    }
}

TEST(NavStartRoutes, AFixLostBeforeAPadReinitStartsDegradedAfterTheGrace) {
    // Usable fixes, then none (the fix lost); PRELAUNCH; then a re-snap.
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

TEST(NavStartRoutes, TheRestartWaitsUntilTheRocketIsSetDown) {
    // Degraded in READY; the rocket carried to the pad with usable fixes
    // coming, then set down at 130 s: the restart a second later.
    Pad p;
    run(p, 0, kNoFixMs + 1000, false, false);
    ASSERT_TRUE(p.degraded);
    p.motion = Motion::Carried;
    run(p, kNoFixMs + 1000, kNoFixMs + 1500, false, false);   // picked up, then the fixes come
    run(p, kNoFixMs + 1500, 130000, true, true);
    EXPECT_TRUE(p.ekf && p.degraded) << "no restart while carried";
    p.motion = Motion::Sway4;   // racked
    run(p, 130000, 130000 + kRestartStillMs - 50, true, true);
    EXPECT_TRUE(p.degraded);
    run(p, 130000 + kRestartStillMs - 50, 130000 + kRestartStillMs + kRestBlockMs + 100, true, true);
    EXPECT_TRUE(p.ekf && !p.degraded);
    EXPECT_EQ(p.restarts, 1);
}

TEST(NavStartRoutes, NoRestartInALitMotorBeforeLaunchIsCalled) {
    // Degraded in PRELAUNCH (4-satellite fixes from boot); the first usable
    // fix lands in the thrust, 60 ms before launch is called. The degraded
    // start stands for the flight: a restart in the thrust took its attitude
    // from a thrust-loaded accelerometer, and on the raw-GNSS images missed
    // the raw filter's pad restart.
    Pad q;
    q.auto_prelaunch = true;
    run(q, 0, 20000, false, true);
    ASSERT_EQ(q.state, PRELAUNCH);
    ASSERT_TRUE(q.degraded);
    q.motion = Motion::Thrust;             // the motor's thrust past 1.25 times the weight
    for (uint32_t t = 20000; t < 20060; ++t) q.pass(t, t % 10 == 0, true);
    q.state = INFLIGHT;                    // launch called
    run(q, 20060, 30000, true, true);
    EXPECT_TRUE(q.ekf && q.degraded);
    EXPECT_EQ(q.restarts, 0);

    // The control: the same fix with the rocket still restarts it.
    Pad p;
    p.auto_prelaunch = true;
    run(p, 0, 20000, false, true);
    run(p, 20000, 20200, true, true);
    EXPECT_TRUE(p.ekf && !p.degraded);
    EXPECT_EQ(p.restarts, 1);
}
