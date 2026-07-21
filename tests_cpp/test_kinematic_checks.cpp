#include <gtest/gtest.h>
#include "TR_KinematicChecks.h"

// TR_KinematicChecks depends on millis() via the host shim.
// Tests must call setMockMillis() to advance time.

class KinematicChecksTest : public ::testing::Test {
protected:
    TR_KinematicChecks kc;

    void SetUp() override {
        setMockMillis(0);
        kc.reset();
    }

    // Helper: call kinematicChecks with typical stationary data
    void callStationary(float alt = 0.0f, float acc_mag = 9.81f, bool new_baro = true) {
        float pos[3] = {0, 0, alt};
        float vel[3] = {0, 0, 0};
        kc.kinematicChecks(alt, acc_mag, pos, vel, 0.0f, new_baro);
    }

    // Helper: call with flight-like data
    void callFlight(float alt, float acc_mag, float vel_u, float roll_rate = 0.0f,
                    float gps_alt = 0.0f, bool new_gps = false,
                    float pitch_rad = 1.57f, bool burnout = false, bool baro_lockout = false,
                    float gps_vel_u = 0.0f, bool ekf_healthy = true, bool baro_healthy = true) {
        float pos[3] = {0, 0, alt};
        float vel[3] = {0, 0, vel_u};
        kc.kinematicChecks(alt, acc_mag, pos, vel, roll_rate, true, gps_alt, new_gps,
                           pitch_rad, burnout, baro_lockout, gps_vel_u, ekf_healthy, baro_healthy);
    }
};

TEST_F(KinematicChecksTest, NoLaunch_BelowThreshold) {
    for (int i = 0; i < 200; i++) {
        setMockMillis(i);
        callStationary(0.0f, 15.0f); // below 20 m/s^2 threshold
    }
    EXPECT_FALSE(kc.launch_flag);
}

TEST_F(KinematicChecksTest, Launch_SustainedAccel) {
    // First, let the altitude KF see some upward motion
    // Feed altitude increasing + high accel for 60+ calls
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2); // 2ms steps
        float alt = 0.5f * i; // altitude climbing
        callFlight(alt, 25.0f, 10.0f); // high accel, positive velocity
    }
    EXPECT_TRUE(kc.launch_flag);
}

TEST_F(KinematicChecksTest, Launch_BriefSpike_NoTrigger) {
    // Only 10 samples of high accel -> should NOT trigger launch
    for (int i = 0; i < 10; i++) {
        setMockMillis(i * 2);
        callFlight(0.0f, 25.0f, 0.0f);
    }
    // Drop back below threshold
    for (int i = 0; i < 200; i++) {
        setMockMillis(20 + i * 2);
        callStationary(0.0f, 5.0f);
    }
    EXPECT_FALSE(kc.launch_flag);
}

// ── #258: accel-only launch fallback when the baro is invalid ──

// Baro invalid (dead) -> d_alt_est_ never confirms a climb.  Sustained >3 g for
// >500 samples must still latch launch so recovery arms (a missed launch = no
// pyro arming = ballistic).  callFlight's last arg is baro_healthy.
TEST_F(KinematicChecksTest, Launch_DeadBaro_AccelOnlyFallbackFires) {
    for (int i = 0; i < 600; i++) {
        setMockMillis(i * 2);
        // flat altitude (no climb), ~3.5 g, baro UNHEALTHY
        callFlight(0.0f, 35.0f, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f,
                   /*ekf_healthy=*/true, /*baro_healthy=*/false);
    }
    EXPECT_TRUE(kc.launch_flag);
}

// Baro invalid but only ~300 samples of >3 g (< 500) -> fallback must NOT fire.
TEST_F(KinematicChecksTest, Launch_DeadBaro_ShortHighG_NoLaunch) {
    for (int i = 0; i < 300; i++) {
        setMockMillis(i * 2);
        callFlight(0.0f, 35.0f, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f,
                   true, /*baro_healthy=*/false);
    }
    EXPECT_FALSE(kc.launch_flag);
}

// Explicit goal: accel must NOT decide launch while the baro is VALID.
// Static-fire / clamped case — healthy baro shows no climb, sustained >3 g.
TEST_F(KinematicChecksTest, Launch_HealthyBaro_HighGNoClimb_NoLaunch) {
    for (int i = 0; i < 600; i++) {
        setMockMillis(i * 2);
        callFlight(0.0f, 35.0f, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f,
                   true, /*baro_healthy=*/true);
    }
    EXPECT_FALSE(kc.launch_flag);
}

TEST_F(KinematicChecksTest, MaxAltitude_SpikeRejection) {
    // Per #142, max_altitude tracks the KF-smoothed altitude (alt_est)
    // rather than the raw pressure_altitude so individual noise spikes
    // can't ratchet the running max above the true climb.  This test
    // verifies both: (a) a single huge spike does not drag max with it,
    // and (b) max still rises when the smoothed altitude rises.
    //
    // The KF takes a few samples to converge, so we feed a short ramp
    // up to ~100m before the spike to seed the filter.

    // Seed the filter at ~100m altitude (held steady — converges fast).
    for (int i = 0; i < 60; i++) {
        setMockMillis(i * 2);
        callFlight(100.0f, 5.0f, 0.0f);
    }
    EXPECT_NEAR(kc.max_altitude, 100.0f, 2.0f);
    const float max_before_spike = kc.max_altitude;

    // Single 400m upward spike — the KF damps it heavily and the
    // window-reject backstop catches whatever leaks through.
    setMockMillis(122);
    callFlight(500.0f, 5.0f, 0.0f);
    EXPECT_LT(kc.max_altitude - max_before_spike, 50.0f)
        << "single spike ratcheted max_altitude by "
        << (kc.max_altitude - max_before_spike) << " m";

    // A sustained rise to ~120m must still update max.  Feed enough
    // samples for the KF to recover from the prior spike (d_alt_est_
    // overshoots, then KF reels alt_est back to truth).  We accept a
    // small overshoot in the upper bound because the spike injected a
    // transient into the rate estimate — what matters is the order of
    // magnitude, not exact equality with 120m.
    for (int i = 0; i < 60; i++) {
        setMockMillis(124 + i * 2);
        callFlight(120.0f, 5.0f, 0.0f);
    }
    EXPECT_GT(kc.max_altitude, 115.0f);
    EXPECT_LT(kc.max_altitude, 135.0f);
}

TEST_F(KinematicChecksTest, Apogee_VoteGatedOnBurnout) {
    // The 4-test apogee VOTE is gated on burnout_detected.  With burnout never
    // latched, a descent too shallow to trip the (burnout-independent, #556)
    // baro backstop must NOT declare apogee.  The backstop's own dead-IMU path
    // is covered by Apogee_DeadIMUInBoost_BaroBackstopStillFires below.
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);

    // Genuine but shallow descent (~15 m) from the ~79 m launch peak, burnout
    // NOT detected.  vel/pitch here would satisfy the vote had burnout latched.
    for (int i = 0; i < 50; i++) {
        setMockMillis(160 + i * 2);
        callFlight(79.0f - i * 0.3f, 5.0f, -10.0f, 0.0f, 0.0f, false, -0.2f, /*burnout*/false);
    }
    // Guard: confirm the descent stayed within APOGEE_BACKSTOP_DROP_M (30 m), so
    // the assertion below tests the burnout gate — not an insufficient descent.
    EXPECT_LT(kc.max_altitude - kc.alt_est, 30.0f);
    EXPECT_FALSE(kc.apogee_flag) << "vote is burnout-gated; a sub-backstop descent must not fire";
}

TEST_F(KinematicChecksTest, Apogee_DeadIMUInBoost_BaroBackstopStillFires) {
    // #556 regression: if the IMU dies during boost, burnout_detected never
    // latches (it only latches from a fresh-IMU accel sample).  Before the fix
    // the whole apogee block — including the baro-only Layer-2 backstop that is
    // documented to survive a dead IMU — was nested under the burnout gate, so
    // apogee was never declared and drogue/main never fired (ballistic).  This
    // mirrors Apogee_EKFUnhealthy_BaroBackstopFires but with burnout==false
    // (dead IMU) rather than a merely-unhealthy EKF: the backstop must still fire.
    kc.launch_flag = true;

    // Seed the baro KF at a 140 m apogee, then pin the running peak.  burnout is
    // NEVER set (the IMU stopped producing fresh samples during boost).
    for (int i = 0; i < 250; i++) {
        setMockMillis(i * 2);
        callFlight(140.0f, 9.81f, 0.0f, 0.0f, 0.0f, false, 1.0f, /*burnout*/false);
    }
    ASSERT_GT(kc.alt_est, 130.0f);
    ASSERT_FALSE(kc.apogee_flag) << "no apogee at the top of coast";
    kc.max_altitude = 140.0f;

    // Descend ~0.5 m/call (inside the baro rate-gate), burnout still FALSE.  The
    // backstop must stay silent until > 30 m below the peak, then latch apogee.
    uint32_t t = 600;
    float alt = 140.0f;
    bool fired_too_high = false;
    for (int i = 0; i < 160; i++, t += 2) {
        alt -= 0.5f;
        setMockMillis(t);
        callFlight(alt, 5.0f, -10.0f, 0.0f, 0.0f, false, -0.5f, /*burnout*/false);
        if (kc.apogee_flag && (140.0f - kc.alt_est) < 28.0f) fired_too_high = true;
    }
    EXPECT_TRUE(kc.apogee_flag)
        << "baro backstop must declare apogee without burnout (dead-IMU boost dropout, #556)";
    EXPECT_TRUE(kc.apogee_backstop_flag)
        << "Layer-2 backstop should be the firing path";
    EXPECT_FALSE(fired_too_high)
        << "backstop must not fire < 30 m below the peak";
}

TEST_F(KinematicChecksTest, Apogee_WithBurnout_DetectsApogee) {
    // Force launch
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);

    // Ascending phase
    for (int i = 0; i < 50; i++) {
        setMockMillis(160 + i * 2);
        callFlight(80.0f + i, 5.0f, 10.0f, 0.0f, 0.0f, false, 1.0f, true);
    }

    // Descending with burnout detected
    for (int i = 0; i < 50; i++) {
        setMockMillis(260 + i * 2);
        float alt = 130.0f - i * 2;
        // EKF velocity negative, altitude decreasing, pitch below horizontal
        callFlight(alt, 5.0f, -10.0f, 0.0f, alt, true, -0.2f, true);
    }
    EXPECT_TRUE(kc.apogee_flag);
}

// #262: GPS is now a voter (re-enabled after the GNSS dynamic-model fix), but a
// single concurring sensor must still never fire the master — the floor-of-2
// quorum holds.  Drive ONLY GPS descending (others say still-ascending): the
// flag is computed, but apogee must not latch on one voter.
TEST_F(KinematicChecksTest, Apogee_GPSAlone_BelowQuorumFloor) {
    for (int i = 0; i < 80; i++) {           // launch
        setMockMillis(i * 2);
        callFlight(0.5f * i, 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);

    for (int i = 0; i < 60; i++) {
        setMockMillis(200 + i * 2);
        callFlight(/*alt*/100.0f + i, /*acc*/5.0f, /*vel_u*/10.0f, /*roll*/0.0f,
                   /*gps_alt*/0.0f, /*new_gps*/true, /*pitch*/1.0f, /*burnout*/true,
                   /*baro_lockout*/false, /*gps_vel_u*/-10.0f);
    }
    EXPECT_TRUE(kc.gps_apogee_flag)  << "GPS apogee flag should be computed";
    EXPECT_FALSE(kc.apogee_flag)     << "one voter (GPS) must not meet the 2-concurring floor";
}

// #262: N-2-when-N>3 quorum.  With all four voters available, exactly TWO
// concurring must fire (== the old 2-of-3) — NOT three.  Isolate the quorum
// arithmetic with deterministic voters: vel (EKF v<0) and GPS (Doppler descent)
// pass; pitch is held nose-up and baro is held at constant altitude (so it is
// AVAILABLE — not locked, healthy — but never < peak-5, so it does not pass).
// Under the old strict N-1 this 2-of-4 would have demanded a 3rd voter.
TEST_F(KinematicChecksTest, Apogee_N2Quorum_TwoOfFourFires) {
    for (int i = 0; i < 80; i++) {           // launch
        setMockMillis(i * 2);
        callFlight(0.5f * i, 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    for (int i = 0; i < 60; i++) {
        setMockMillis(200 + i * 2);
        callFlight(/*alt*/100.0f /*constant → baro available, not passing*/, 5.0f,
                   /*vel_u*/-10.0f /*descending → vel passes*/, 0.0f,
                   /*gps_alt*/100.0f, /*new_gps*/true,
                   /*pitch*/1.0f /*nose-up → NOT passing*/, /*burnout*/true,
                   /*baro_lockout*/false, /*gps_vel_u*/-10.0f /*GPS passes*/);
    }
    EXPECT_TRUE(kc.vel_u_apogee_flag);
    EXPECT_TRUE(kc.gps_apogee_flag);
    EXPECT_FALSE(kc.alt_apogee_flag) << "constant-alt baro is available but must not pass";
    EXPECT_FALSE(kc.pitch_apogee_flag);
    EXPECT_TRUE(kc.apogee_flag) << "2 of 4 concurring must fire under N-2 (would need 3 under N-1)";
}

// Baro settle window after burnout (7/05 V2 F1 flight). At thrust tail-off the
// bay pressure snaps back from its boost-suction offset: indicated altitude
// fell 15 m in 0.25 s while the rocket climbed at 46 m/s — which satisfies the
// baro apogee test (alt < ratcheted max − 5) the instant its burnout gate
// opens.  The baro voter must stay silent through BARO_BURNOUT_SETTLE_MS, then
// work normally on the real descent.
TEST_F(KinematicChecksTest, Apogee_BurnoutBaroTransient_NoVoteInSettleWindow) {
    for (int i = 0; i < 80; i++) {           // launch, climbing to ~40 m indicated
        setMockMillis(i * 2);
        callFlight(0.5f * i, 25.0f, 45.0f);
    }
    ASSERT_TRUE(kc.launch_flag);

    // Burnout at t=160 ms: indicated altitude dives 40 → 22 m over 250 ms
    // while EKF velocity says +45 m/s (still climbing hard).  Nose-up, no GPS.
    for (int i = 0; i < 25; i++) {
        setMockMillis(160 + i * 10);
        callFlight(40.0f - 0.72f * i, 2.0f, 45.0f, 0.0f, 0.0f, false,
                   /*pitch*/1.0f, /*burnout*/true);
        EXPECT_FALSE(kc.alt_apogee_flag)
            << "baro must not vote during the post-burnout settle window (i=" << i << ")";
    }
    EXPECT_FALSE(kc.apogee_flag);

    // Recovery + continued climb through the rest of the settle window.
    for (int i = 0; i < 80; i++) {
        setMockMillis(410 + i * 10);
        callFlight(25.0f + 1.0f * i, 2.0f, 30.0f, 0.0f, 0.0f, false, 1.0f, true);
    }
    EXPECT_FALSE(kc.alt_apogee_flag);

    // Well past the window (t≈1.2 s+ after burnout): genuine descent from the
    // peak — the baro voter must work normally again.
    for (int i = 0; i < 60; i++) {
        setMockMillis(1210 + i * 10);
        callFlight(105.0f - 2.0f * i, 2.0f, -10.0f, 0.0f, 0.0f, false, 1.0f, true);
    }
    EXPECT_TRUE(kc.alt_apogee_flag) << "baro voter must recover after the settle window";
}

// #262 CORE: during mach-lockout (baro excluded) a single EKF-voter fault would
// sink the old 2-of-2 {vel,pitch}.  With GPS restored as a non-EKF voter the
// vote becomes 2-of-3 {vel,gps,pitch}, so pitch+GPS carry it.  Here EKF velocity
// is faulted (reads +5 while truly descending), pitch + GPS agree on descent.
TEST_F(KinematicChecksTest, Apogee_GPSRescuesMachLockout_OneEKFFault) {
    for (int i = 0; i < 80; i++) {           // launch
        setMockMillis(i * 2);
        callFlight(0.5f * i, 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    for (int i = 0; i < 60; i++) {           // mach-locked descent, faulted vel
        setMockMillis(200 + i * 2);
        callFlight(/*alt*/100.0f, 5.0f, /*vel_u*/+5.0f /*FAULT: says ascending*/,
                   0.0f, /*gps_alt*/100.0f, /*new_gps*/true,
                   /*pitch*/-0.5f /*descending*/, /*burnout*/true,
                   /*baro_lockout*/true, /*gps_vel_u*/-10.0f /*descending*/);
    }
    EXPECT_FALSE(kc.vel_u_apogee_flag) << "faulted EKF velocity must not pass";
    EXPECT_TRUE(kc.gps_apogee_flag);
    EXPECT_TRUE(kc.pitch_apogee_flag);
    EXPECT_TRUE(kc.apogee_flag) << "GPS+pitch (2-of-3) must carry during lockout (#262)";
}

// Companion: identical lockout + faulted-vel scenario but with NO GPS fix — the
// vote falls back to 2-of-2 {vel,pitch} and CANNOT fire (the pre-#262 failure).
TEST_F(KinematicChecksTest, Apogee_MachLockout_OneEKFFault_NoGPS_DoesNotFire) {
    for (int i = 0; i < 80; i++) {           // launch
        setMockMillis(i * 2);
        callFlight(0.5f * i, 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    for (int i = 0; i < 60; i++) {           // same as above, but new_gps=false
        setMockMillis(200 + i * 2);
        callFlight(/*alt*/100.0f, 5.0f, /*vel_u*/+5.0f, 0.0f, /*gps_alt*/0.0f,
                   /*new_gps*/false, /*pitch*/-0.5f, /*burnout*/true,
                   /*baro_lockout*/true, /*gps_vel_u*/0.0f);
    }
    EXPECT_TRUE(kc.pitch_apogee_flag);
    EXPECT_FALSE(kc.apogee_flag) << "without GPS, lockout+vel-fault leaves 1-of-2 — no fire";
}

// #262 freshness gate: a GPS apogee flag latched from earlier fixes must NOT
// keep voting once the fix goes stale (> GPS_APOGEE_FRESH_MS).  Phase 1 latches
// gps_apogee_flag (pitch not yet passing → no fire).  Phase 2 stops GPS updates
// and lets the clock pass the freshness window while pitch starts passing: GPS
// is now stale, so the vote is only {vel,pitch}=1 and must not fire.
TEST_F(KinematicChecksTest, Apogee_StaleGPS_DoesNotVote) {
    for (int i = 0; i < 80; i++) {           // launch
        setMockMillis(i * 2);
        callFlight(0.5f * i, 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    int t = 200;
    for (int i = 0; i < 40; i++) {           // Phase 1: latch GPS (pitch nose-up)
        setMockMillis(t); t += 2;
        callFlight(/*alt*/100.0f, 5.0f, /*vel_u*/+5.0f, 0.0f, /*gps_alt*/100.0f,
                   /*new_gps*/true, /*pitch*/1.0f, /*burnout*/true,
                   /*baro_lockout*/true, /*gps_vel_u*/-10.0f);
    }
    ASSERT_TRUE(kc.gps_apogee_flag);
    ASSERT_FALSE(kc.apogee_flag);
    // Phase 2: no more GPS; jump past the freshness window; pitch now passes.
    t += 700;                                 // > GPS_APOGEE_FRESH_MS (500) since last fix
    for (int i = 0; i < 40; i++) {
        setMockMillis(t); t += 2;
        callFlight(/*alt*/100.0f, 5.0f, /*vel_u*/+5.0f, 0.0f, /*gps_alt*/0.0f,
                   /*new_gps*/false, /*pitch*/-0.5f /*now descending*/, /*burnout*/true,
                   /*baro_lockout*/true, /*gps_vel_u*/0.0f);
    }
    EXPECT_TRUE(kc.pitch_apogee_flag);
    EXPECT_FALSE(kc.apogee_flag) << "stale GPS must not count — only pitch passes (1-of-2)";
}

TEST_F(KinematicChecksTest, Landing_StableAlt) {
    // Force launch and establish max_altitude > 15m
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    ASSERT_GT(kc.max_altitude, 15.0f);

    // Landing voting is gated on apogee_flag (#166) — set it the same
    // way Landing_FastPath_ImpactTriggers does.
    kc.apogee_flag = true;

    // Now simulate landed: alt < 50, stable, low roll rate, accel ~1g.
    // Voting needs the slow detectors to accumulate over ~4 s.
    for (int second = 0; second < 7; second++) {
        uint32_t base = 1000 + second * 1000;
        // Call many times within each second (landing_check_dt = 1000ms)
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(5.0f, 9.81f, 0.0f, 0.1f); // stable at 5m, low roll rate
        }
    }
    EXPECT_TRUE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_NotPremature) {
    // If max_altitude was never > 15m, landing should NOT trigger
    for (int second = 0; second < 10; second++) {
        uint32_t base = second * 1000;
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(5.0f, 9.81f, 0.0f, 0.1f);
        }
    }
    EXPECT_FALSE(kc.alt_landed_flag); // max_altitude < 15
}

TEST_F(KinematicChecksTest, AltKF_ConvergesToMeasurement) {
    // Feed constant altitude measurements
    for (int i = 0; i < 500; i++) {
        setMockMillis(i * 2);
        callStationary(100.0f, 9.81f);
    }
    EXPECT_NEAR(kc.alt_est, 100.0f, 1.0f);
}

TEST_F(KinematicChecksTest, AltKF_TracksRamp) {
    // Feed linearly increasing altitude
    for (int i = 0; i < 500; i++) {
        setMockMillis(i * 2);
        float alt = float(i) * 0.1f; // 50 m/s altitude rate
        callStationary(alt, 9.81f);
    }
    // The filtered rate should be positive
    EXPECT_GT(kc.d_alt_est_, 0.0f);
}

TEST_F(KinematicChecksTest, Reset_ClearsAll) {
    // Force launch
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);

    kc.reset();

    EXPECT_FALSE(kc.launch_flag);
    EXPECT_FALSE(kc.alt_landed_flag);
    EXPECT_FALSE(kc.alt_apogee_flag);
    EXPECT_FALSE(kc.vel_u_apogee_flag);
    EXPECT_FALSE(kc.gps_apogee_flag);
    EXPECT_FALSE(kc.pitch_apogee_flag);
    EXPECT_FALSE(kc.apogee_flag);
    EXPECT_FALSE(kc.apogee_backstop_flag);
    EXPECT_FLOAT_EQ(kc.max_altitude, 0.0f);
    EXPECT_FLOAT_EQ(kc.max_speed, 0.0f);
}

// ── Tests for issue #113: relaxed gyro threshold + impact fast path ──

TEST_F(KinematicChecksTest, Landing_RollRate15dps_StillPasses) {
    // The relaxed 20 dps threshold accepts steady 15 dps wobble (e.g. wind
    // on a landed rocket). Old 2 dps threshold would fail this case.
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    ASSERT_GT(kc.max_altitude, 15.0f);

    // Landing voting is gated on apogee_flag (#166).
    kc.apogee_flag = true;

    for (int second = 0; second < 7; second++) {
        uint32_t base = 1000 + second * 1000;
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(5.0f, 9.81f, 0.0f, 15.0f);
        }
    }
    EXPECT_TRUE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_RollRate25dps_DoesNotTrigger) {
    // 25 dps exceeds the 20 dps threshold -- still rejected by slow path.
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);

    for (int second = 0; second < 7; second++) {
        uint32_t base = 1000 + second * 1000;
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(5.0f, 9.81f, 0.0f, 25.0f);
        }
    }
    EXPECT_FALSE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_FastPath_ImpactTriggers) {
    // apogee + low altitude + >15g for 5 consecutive samples -> landed
    kc.apogee_flag = true;
    for (int i = 0; i < 10; i++) {
        setMockMillis(1000 + i);
        callFlight(5.0f, 200.0f, -10.0f, 0.0f);  // ~20g, 5m alt
    }
    EXPECT_TRUE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_FastPath_GatedOnApogee) {
    // Same impact-magnitude accel pre-apogee -> NO trigger (e.g. boost spike)
    for (int i = 0; i < 50; i++) {
        setMockMillis(1000 + i);
        callFlight(5.0f, 200.0f, 10.0f, 0.0f);
    }
    EXPECT_FALSE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_FastPath_GatedOnAltitude) {
    // Apogee + impact-magnitude accel at altitude (e.g. ejection at apogee)
    // -> NO trigger because pressure_altitude > 20m
    kc.apogee_flag = true;
    for (int i = 0; i < 50; i++) {
        setMockMillis(1000 + i);
        callFlight(100.0f, 200.0f, -10.0f, 0.0f);
    }
    EXPECT_FALSE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_FastPath_BelowG_NoTrigger) {
    // Apogee + low altitude but accel below 15 g threshold -> NO trigger
    kc.apogee_flag = true;
    for (int i = 0; i < 50; i++) {
        setMockMillis(1000 + i);
        callFlight(5.0f, 100.0f, -10.0f, 0.0f);  // ~10g, below threshold
    }
    EXPECT_FALSE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_FastPath_BriefSpike_CounterResets) {
    // Single high-g sample then back to quiet -> count resets, no trigger.
    // Verifies the noise-rejection behavior of the consecutive-sample gate.
    kc.apogee_flag = true;
    setMockMillis(1000);
    callFlight(5.0f, 200.0f, -10.0f, 0.0f);  // 1 sample at impact magnitude
    for (int i = 0; i < 100; i++) {
        setMockMillis(1001 + i);
        callFlight(5.0f, 9.81f, 0.0f, 0.0f);  // back to gravity floor
    }
    EXPECT_FALSE(kc.alt_landed_flag);
}

// ── Test for issue #192: landing sub-flag counters reset on apogee rising edge ──
TEST_F(KinematicChecksTest, Landing_SubflagsResetOnApogeeRisingEdge) {
    // The 1 Hz landing sub-detectors (gyro_quiet, gps_stationary, accel_1g,
    // baro_stable) tick regardless of flight state, so a rocket flying
    // straight pre-apogee (low roll rate, ~1g coast) can latch their flags
    // before apogee. The apogee-rising-edge code in kinematicChecks() must
    // zero those counters + flags so post-apogee voting sees only post-
    // apogee evidence. Verifies the observable: pre-latched flags reset
    // when apogee_flag transitions False → True inside the function.

    // Bypass launch detection (already covered by other tests) and force
    // max_altitude so baro_stable's > 15 m gate is satisfied.
    kc.launch_flag = true;
    kc.max_altitude = 50.0f;

    // Seed the KF at 30 m so alt_est tracks above 15 m (needed later for
    // baro/vel apogee tests). 80 calls @ 2 ms = ~160 ms, plenty for the KF
    // to converge.
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(30.0f, 9.81f, 0.0f, 0.1f);  // hold at 30 m, ~1g, quiet
    }

    // Drive 6 s of "quiet coast" (low gyro, ~1g, vel=0). With the 1 Hz
    // sub-detector gate, gyro_quiet_count_ rises one tick/second; flag
    // latches at count >= 4 (~T+4 s into quiet).
    for (int second = 0; second < 6; second++) {
        uint32_t base = 200 + second * 1000;
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(30.0f, 9.81f, 0.0f, 0.1f);  // quiet
        }
    }

    ASSERT_TRUE(kc.gyro_quiet_flag) << "Quiet inputs should latch gyro_quiet_flag pre-apogee";
    ASSERT_TRUE(kc.accel_1g_flag)  << "Quiet inputs should latch accel_1g_flag pre-apogee";
    ASSERT_FALSE(kc.apogee_flag)   << "apogee_flag should still be false";

    // Now drive apogee-triggering inputs to flip apogee_flag inside the
    // function (this is the rising edge the reset hooks on).
    //   vel_pass:   alt > 15 (pos[2]) && velocity[2] < 0
    //   baro_pass:  alt_est > 15 && alt_est < max_altitude - 5 && d_alt_est < 20
    //   pitch_pass: pitch_rad < -0.087  (5° below horizontal)
    // APOGEE_COUNT_HI = 6 so 6+ consecutive passing calls latches each
    // sub-flag; one more call after that fires the master vote.
    for (int i = 0; i < 20; i++) {
        setMockMillis(6500 + i * 2);
        float alt = 30.0f - i * 0.5f;  // descending from 30 m
        callFlight(alt, 5.0f, -10.0f, 0.1f,
                   /*gps_alt=*/0.0f, /*new_gps=*/false,
                   /*pitch_rad=*/-0.5f, /*burnout=*/true, /*baro_lockout=*/false);
    }

    EXPECT_TRUE(kc.apogee_flag)        << "apogee_flag should have transitioned to true";
    EXPECT_FALSE(kc.gyro_quiet_flag)   << "gyro_quiet_flag should reset on apogee rising edge";
    EXPECT_FALSE(kc.accel_1g_flag)     << "accel_1g_flag should reset on apogee rising edge";
    EXPECT_FALSE(kc.gps_stationary_flag)
        << "gps_stationary_flag should reset on apogee rising edge";
    EXPECT_FALSE(kc.baro_stable_flag)
        << "baro_stable_flag should reset on apogee rising edge";
}

// ── #257: health-aware adaptive quorum + Layer-2 baro descent backstop ──

// EKF demonstrably unhealthy → velocity + pitch voters excluded, so the primary
// vote is starved.  A healthy baro that has dropped > APOGEE_BACKSTOP_DROP_M
// (30 m) below the peak while descending must still fire apogee via Layer 2 —
// the rocket must not come in ballistic just because the EKF died.
TEST_F(KinematicChecksTest, Apogee_EKFUnhealthy_BaroBackstopFires) {
    kc.launch_flag = true;

    // Seed the baro KF at a 140 m apogee, then pin the running peak.
    for (int i = 0; i < 250; i++) {
        setMockMillis(i * 2);
        callFlight(140.0f, 9.81f, 0.0f, 0.0f, 0.0f, false, 1.0f, true);
    }
    ASSERT_GT(kc.alt_est, 130.0f);     // KF converged near the apogee
    kc.max_altitude = 140.0f;

    // Descend ~0.5 m/call (inside the baro rate-gate) with the EKF UNHEALTHY
    // and baro healthy.  The backstop must stay silent until > 30 m below the
    // peak, then latch apogee.
    uint32_t t = 600;
    float alt = 140.0f;
    bool fired_too_high = false;
    for (int i = 0; i < 160; i++, t += 2) {
        alt -= 0.5f;
        setMockMillis(t);
        callFlight(alt, 5.0f, -10.0f, 0.0f, 0.0f, false, -0.5f, true,
                   false, 0.0f, /*ekf_healthy=*/false, /*baro_healthy=*/true);
        if (kc.apogee_flag && (140.0f - kc.alt_est) < 28.0f) fired_too_high = true;
    }
    EXPECT_TRUE(kc.apogee_flag)          << "backstop should fire once > 30 m below peak";
    EXPECT_TRUE(kc.apogee_backstop_flag) << "Layer-2 backstop should be the firing path";
    EXPECT_FALSE(fired_too_high)         << "backstop must not fire < 30 m below the peak";
}

// False-positive guard: the EKF flagged unhealthy DURING ascent must not fire
// apogee.  The vote is starved and the backstop stays silent because altitude
// is rising (never 30 m below the peak).
TEST_F(KinematicChecksTest, Apogee_NoFalsePositive_AscentWithUnhealthyEKF) {
    for (int i = 0; i < 80; i++) {                       // launch
        setMockMillis(i * 2);
        callFlight(0.5f * i, 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    for (int i = 0; i < 250; i++) {                      // long ascent, worst case
        setMockMillis(200 + i * 2);
        callFlight(40.0f + i * 1.0f, 5.0f, 10.0f, 0.0f, 0.0f, false, 1.0f, true,
                   false, 0.0f, /*ekf_healthy=*/false, /*baro_healthy=*/true);
    }
    EXPECT_FALSE(kc.apogee_flag)          << "no apogee during ascent even with EKF unhealthy";
    EXPECT_FALSE(kc.apogee_backstop_flag);
}

// Baro flagged unhealthy → excluded; a healthy EKF (velocity + pitch) must
// still carry the vote (floor of 2 = both EKF voters).  This is the
// mach-lockout-equivalent path, now also reachable on a baro fault.
TEST_F(KinematicChecksTest, Apogee_BaroUnhealthy_EKFVotersCarry) {
    for (int i = 0; i < 80; i++) {                       // launch
        setMockMillis(i * 2);
        callFlight(0.5f * i, 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    for (int i = 0; i < 50; i++) {                       // ascent, burnout latches
        setMockMillis(200 + i * 2);
        callFlight(80.0f + i, 5.0f, 10.0f, 0.0f, 0.0f, false, 1.0f, true);
    }
    for (int i = 0; i < 40; i++) {                       // descent, baro UNHEALTHY
        setMockMillis(300 + i * 2);
        callFlight(130.0f - i * 2, 5.0f, -10.0f, 0.0f, 0.0f, false, -0.5f, true,
                   false, 0.0f, /*ekf_healthy=*/true, /*baro_healthy=*/false);
    }
    EXPECT_TRUE(kc.apogee_flag)           << "EKF vel+pitch should carry when baro unhealthy";
    EXPECT_FALSE(kc.apogee_backstop_flag) << "primary vote fired, not the backstop";
}

// With NO healthy sensor there is nothing to detect apogee from, so apogee
// stays false (documented blind case — beyond recovery's reach).
TEST_F(KinematicChecksTest, Apogee_BothUnhealthy_NoFire) {
    for (int i = 0; i < 80; i++) {                       // launch
        setMockMillis(i * 2);
        callFlight(0.5f * i, 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    for (int i = 0; i < 120; i++) {                      // descend, both unhealthy
        setMockMillis(200 + i * 2);
        callFlight(130.0f - i, 5.0f, -10.0f, 0.0f, 0.0f, false, -0.5f, true,
                   false, 0.0f, /*ekf_healthy=*/false, /*baro_healthy=*/false);
    }
    EXPECT_FALSE(kc.apogee_flag);
    EXPECT_FALSE(kc.apogee_backstop_flag);
}
