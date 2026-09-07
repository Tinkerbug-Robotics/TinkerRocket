#include <gtest/gtest.h>
#include "TR_KinematicChecks.h"
#include "fixtures/boost_20260517_rp54.h"
#include "GroundRefFreeze.h"

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
                    float gps_vel_u = 0.0f, bool ekf_healthy = true, bool baro_healthy = true,
                    bool imu_healthy = true) {
        float pos[3] = {0, 0, alt};
        float vel[3] = {0, 0, vel_u};
        kc.kinematicChecks(alt, acc_mag, pos, vel, roll_rate, true, gps_alt, new_gps,
                           pitch_rad, burnout, baro_lockout, gps_vel_u, ekf_healthy, baro_healthy,
                           imu_healthy);
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
    EXPECT_EQ(kc.launch_path, TR_KinematicChecks::LaunchPath::BaroClimb);
    EXPECT_TRUE(kc.launch_baro_healthy);
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
// >250 samples (~250 ms at the 1 kHz flight-logic rate) must still latch launch
// so recovery arms (a missed launch = no pyro arming = ballistic).  callFlight's
// last arg is baro_healthy.
TEST_F(KinematicChecksTest, Launch_DeadBaro_AccelOnlyFallbackFires) {
    for (int i = 0; i < 300; i++) {
        setMockMillis(i * 2);
        // flat altitude (no climb), ~3.5 g, baro UNHEALTHY
        callFlight(0.0f, 35.0f, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f,
                   /*ekf_healthy=*/true, /*baro_healthy=*/false);
    }
    EXPECT_TRUE(kc.launch_flag);
    EXPECT_EQ(kc.launch_path, TR_KinematicChecks::LaunchPath::AccelOnly);
    EXPECT_FALSE(kc.launch_baro_healthy);   // the #258 case: the baro was dead
}

// Baro invalid but only ~150 samples of >3 g (< 250) -> fallback must NOT fire.
TEST_F(KinematicChecksTest, Launch_DeadBaro_ShortHighG_NoLaunch) {
    for (int i = 0; i < 150; i++) {
        setMockMillis(i * 2);
        callFlight(0.0f, 35.0f, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f,
                   true, /*baro_healthy=*/false);
    }
    EXPECT_FALSE(kc.launch_flag);
}

// The threshold is UNINTERRUPTED: a single sample at or below the 20 m/s2 reset
// floor zeroes both counters, so an oscillating stimulus (hand motion, a bump,
// vibration) can never accumulate to the bar no matter how long it goes on.
// This is what makes ~250 ms safe to auto-promote on — see the INITIALIZATION
// launch escape in flight_computer/main/main.cpp.
TEST_F(KinematicChecksTest, Launch_DeadBaro_InterruptedHighG_NeverLatches) {
    for (int i = 0; i < 2000; i++) {
        setMockMillis(i * 2);
        // 200 samples above the bar, then one sample at the reset floor.
        const float acc = (i % 201 == 200) ? 15.0f : 35.0f;
        callFlight(0.0f, acc, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f,
                   true, /*baro_healthy=*/false);
    }
    EXPECT_FALSE(kc.launch_flag);
}

// Boundary: 249 sustained samples is still short of the bar, 251 clears it.
TEST_F(KinematicChecksTest, Launch_DeadBaro_FallbackBoundary) {
    for (int i = 0; i < 249; i++) {
        setMockMillis(i * 2);
        callFlight(0.0f, 35.0f, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f,
                   true, /*baro_healthy=*/false);
    }
    EXPECT_FALSE(kc.launch_flag) << "249 samples must not latch";
    for (int i = 249; i < 252; i++) {
        setMockMillis(i * 2);
        callFlight(0.0f, 35.0f, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f,
                   true, /*baro_healthy=*/false);
    }
    EXPECT_TRUE(kc.launch_flag) << "251 samples must latch";
}

// ── #1102: the fallback is NOT gated on baro health ──
//
// A blocked or taped static port leaves the barometer fresh and in range
// (baro_healthy == true) while indicated altitude never moves.  The primary
// cannot see a climb, and the #258 fallback used to be gated off exactly here,
// so the FC never left PRELAUNCH.  Sustained >3 g must latch regardless, and
// the latch must record that the baro was "healthy" and silent — that is the
// blocked-port signature the INFLIGHT-entry log reports.
TEST_F(KinematicChecksTest, Launch_HealthyFlatBaro_SustainedHighG_AccelOnlyLatches) {
    for (int i = 0; i < 600; i++) {
        setMockMillis(i * 2);
        callFlight(0.0f, 35.0f, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f,
                   true, /*baro_healthy=*/true);
        if (i == 249) EXPECT_FALSE(kc.launch_flag) << "bar is 250 samples, not fewer";
        if (i == 251) EXPECT_TRUE(kc.launch_flag)  << "must latch as soon as the bar is met";
    }
    EXPECT_TRUE(kc.launch_flag);
    EXPECT_EQ(kc.launch_path, TR_KinematicChecks::LaunchPath::AccelOnly);
    EXPECT_TRUE(kc.launch_baro_healthy);
}

// Ungating the branch must not lower the bar: short high-G with a healthy,
// flat baro still does not launch...
TEST_F(KinematicChecksTest, Launch_HealthyFlatBaro_ShortHighG_NoLaunch) {
    for (int i = 0; i < 150; i++) {
        setMockMillis(i * 2);
        callFlight(0.0f, 35.0f, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f,
                   true, /*baro_healthy=*/true);
    }
    EXPECT_FALSE(kc.launch_flag);
}

// ...nor does an oscillating stimulus (the handling / vibration case), however
// long it goes on — the reset floor still zeroes the counters.
TEST_F(KinematicChecksTest, Launch_HealthyFlatBaro_InterruptedHighG_NeverLatches) {
    for (int i = 0; i < 2000; i++) {
        setMockMillis(i * 2);
        const float acc = (i % 201 == 200) ? 15.0f : 35.0f;
        callFlight(0.0f, acc, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f,
                   true, /*baro_healthy=*/true);
    }
    EXPECT_FALSE(kc.launch_flag);
}

// ...nor does the primary's 2 g floor on its own: above 20 m/s2 but below the
// 30 m/s2 fallback bar, a flat baro means no launch no matter how long.
TEST_F(KinematicChecksTest, Launch_HealthyFlatBaro_BelowFallbackBar_NoLaunch) {
    for (int i = 0; i < 600; i++) {
        setMockMillis(i * 2);
        callFlight(0.0f, 25.0f, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f,
                   true, /*baro_healthy=*/true);
    }
    EXPECT_FALSE(kc.launch_flag);
}

// #1102 on real data.  The noisiest logged boost (2026-05-17 Rolly Polly
// 54 mm: +/-300 m/s2 of motor vibration about the thrust level, gyro driven
// into garbage) with the barometer replaced by a healthy-but-flat one, i.e. a
// taped static port.  One sample per flight-loop tick, the FC's own channel
// pick, generated by Data_Analysis/analyze_launch_fallback.py.  The fallback
// must latch inside the 873 ms burn and not before the bar is met.
TEST_F(KinematicChecksTest, Launch_RealBoost_BlockedPort_AccelOnlyLatchesInsideTheBurn) {
    int latched_at = -1;
    for (int i = 0; i < kBoost20260517Rp54_n; i++) {
        setMockMillis(kBoost20260517Rp54_t_ms[i]);
        callFlight(0.0f, kBoost20260517Rp54_accel[i], 0.0f, 0.0f, 0.0f, false, 1.57f,
                   false, false, 0.0f, true, /*baro_healthy=*/true);
        if (kc.launch_flag && latched_at < 0) latched_at = i;
    }
    ASSERT_GE(latched_at, 0) << "a real boost with a blocked port must still latch";
    const int ms_after_ignition = (int)kBoost20260517Rp54_t_ms[latched_at]
                                - (int)kBoost20260517Rp54_t_ms[kBoost20260517Rp54_boost0];
    EXPECT_GE(ms_after_ignition, 250);   // never before the bar
    EXPECT_LE(ms_after_ignition, 400);   // corpus replay measured +300 ms; burnout is +873
    EXPECT_EQ(kc.launch_path, TR_KinematicChecks::LaunchPath::AccelOnly);
    EXPECT_TRUE(kc.launch_baro_healthy);
}

// The same flight as flown: real accel AND the real barometer, which does see
// the climb.  The primary wins well before the accel fallback counter can
// fill, and says so.  This is the argument for dropping the gate — on a
// truthful baro the fallback never gets to decide.
TEST_F(KinematicChecksTest, Launch_RealBoost_AsFlown_PrimaryWinsAndRecordsPath) {
    int latched_at = -1;
    for (int i = 0; i < kBoost20260517Rp54_n; i++) {
        setMockMillis(kBoost20260517Rp54_t_ms[i]);
        float pos[3] = {0, 0, 0}, vel[3] = {0, 0, 0};
        kc.kinematicChecks(kBoost20260517Rp54_palt[i], kBoost20260517Rp54_accel[i], pos, vel, 0.0f,
                           kBoost20260517Rp54_baro_new[i] != 0);
        if (kc.launch_flag && latched_at < 0) latched_at = i;
    }
    ASSERT_GE(latched_at, 0);
    const int ms_after_ignition = (int)kBoost20260517Rp54_t_ms[latched_at]
                                - (int)kBoost20260517Rp54_t_ms[kBoost20260517Rp54_boost0];
    EXPECT_LT(ms_after_ignition, 250) << "the FC's own flag rose at +96 ms on this flight";
    EXPECT_EQ(kc.launch_path, TR_KinematicChecks::LaunchPath::BaroClimb);
    EXPECT_TRUE(kc.launch_baro_healthy);
}

// ── #1108: baro-only fallback when the IMU is stale or absent ──
//
// Every latch above sits inside `acc_mag > 20`, and the FC passes 0 for a
// stale IMU, so an IMU that died on the pad made INFLIGHT unreachable while
// the barometer recorded the whole flight.  A sustained filtered climb, well
// off the pad and still rising across the run, must latch on its own — but
// ONLY with the IMU unusable (see the constant's comment: a barometric false
// launch is not contained the way an accel one is).

// Helper: a dead-IMU tick with the given pressure altitude.
#define DEAD_IMU_TICK(alt) \
    callFlight((alt), 0.0f, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f, \
               true, /*baro_healthy=*/true, /*imu_healthy=*/false)

// A 20 m/s climb with no IMU: latches after the altitude bar (15 m) plus the
// 250-tick run, and records the baro-only path.
TEST_F(KinematicChecksTest, Launch_DeadIMU_SustainedClimb_BaroOnlyLatches) {
    int latched_at = -1;
    for (int i = 0; i < 3000; i++) {
        setMockMillis(i);
        DEAD_IMU_TICK(20.0f * i * 1e-3f);
        if (kc.launch_flag && latched_at < 0) latched_at = i;
    }
    ASSERT_GE(latched_at, 0);
    // 15 m at 20 m/s is 750 ms; the run then needs 250 more, plus filter lag.
    EXPECT_GE(latched_at, 1000);
    EXPECT_LE(latched_at, 1200);
    EXPECT_EQ(kc.launch_path, TR_KinematicChecks::LaunchPath::BaroOnly);
    EXPECT_TRUE(kc.launch_baro_healthy);
}

// A lift, stairs or a car on a grade: 3 m/s for 20 s reaches 60 m and never
// launches, because the rate bar is 10 m/s.
TEST_F(KinematicChecksTest, Launch_DeadIMU_LiftSpeedClimb_NoLaunch) {
    for (int i = 0; i < 20000; i++) {
        setMockMillis(i);
        DEAD_IMU_TICK(3.0f * i * 1e-3f);
    }
    EXPECT_FALSE(kc.launch_flag);
}

// Fast but shallow: 20 m/s up to 12 m, then flat.  Never above the 15 m bar
// while climbing, so no run ever starts.
TEST_F(KinematicChecksTest, Launch_DeadIMU_ShallowClimb_NoLaunch) {
    for (int i = 0; i < 5000; i++) {
        setMockMillis(i);
        const float alt = i < 600 ? 20.0f * i * 1e-3f : 12.0f;
        DEAD_IMU_TICK(alt);
    }
    EXPECT_FALSE(kc.launch_flag);
}

// A pressure step (a door, HVAC, an ejection-charge ground test in the bay):
// 30 m in one sample, held for a second, then back.  The rate gate rejects
// the step, then accepts it after MAX_CONSEC_BARO_REJECTS, and the filter's
// rate and altitude both swing while it converges — but the raw reading sits
// flat at 30 m, so the gain term refuses the run.  Tested at both signs.
TEST_F(KinematicChecksTest, Launch_DeadIMU_PressureStep_NoLaunch) {
    for (int i = 0; i < 4000; i++) {
        setMockMillis(i);
        const float alt = (i >= 1000 && i < 2000) ? 30.0f : 0.0f;
        DEAD_IMU_TICK(alt);
    }
    EXPECT_FALSE(kc.launch_flag);
    kc.reset();
    for (int i = 0; i < 4000; i++) {
        setMockMillis(i);
        const float alt = (i >= 1000 && i < 2000) ? 8.0f : 0.0f;
        DEAD_IMU_TICK(alt);
    }
    EXPECT_FALSE(kc.launch_flag);
}

// The gate: a working IMU that shows no boost vetoes the barometer.  Same
// 20 m/s climb, IMU fresh at 1 g — the car-window case — must NOT launch.
TEST_F(KinematicChecksTest, Launch_HealthyIMU_ClimbAlone_NoLaunch) {
    for (int i = 0; i < 3000; i++) {
        setMockMillis(i);
        callFlight(20.0f * i * 1e-3f, 9.81f, 0.0f, 0.0f, 0.0f, false, 1.57f, false, false, 0.0f,
                   true, /*baro_healthy=*/true, /*imu_healthy=*/true);
    }
    EXPECT_FALSE(kc.launch_flag);
}

// #1108 on real data, PRELAUNCH case (datum frozen on the pad): the 2026-05-17
// 54 mm boost with the IMU replaced by a dead one.  The real barometer trace
// must latch the baro-only path inside the log, well after the 15 m bar.
TEST_F(KinematicChecksTest, Launch_RealBoost_DeadIMU_BaroOnlyLatches) {
    int latched_at = -1;
    for (int i = 0; i < kBoost20260517Rp54_n; i++) {
        setMockMillis(kBoost20260517Rp54_t_ms[i]);
        float pos[3] = {0, 0, 0}, vel[3] = {0, 0, 0};
        kc.kinematicChecks(kBoost20260517Rp54_palt[i], 0.0f, pos, vel, 0.0f,
                           kBoost20260517Rp54_baro_new[i] != 0, 0.0f, false, 1.57f, false, false,
                           0.0f, true, /*baro_healthy=*/true, /*imu_healthy=*/false);
        if (kc.launch_flag && latched_at < 0) latched_at = i;
    }
    ASSERT_GE(latched_at, 0) << "a real boost with a dead IMU must still latch";
    const int ms_after_ignition = (int)kBoost20260517Rp54_t_ms[latched_at]
                                - (int)kBoost20260517Rp54_t_ms[kBoost20260517Rp54_boost0];
    EXPECT_GE(ms_after_ignition, 500);    // never before the vehicle is well off the pad
    EXPECT_LE(ms_after_ignition, 1300);   // corpus replay: see the constant's comment
    EXPECT_EQ(kc.launch_path, TR_KinematicChecks::LaunchPath::BaroOnly);
}

// #1108 on real data, READY case: the datum re-seeds from every sample the way
// INITIALIZATION / READY do, through GroundRefFreeze, exactly as main.cpp does
// it.  The freeze must catch the climb early, roll the datum back to the pad,
// and the baro-only path must then latch off a datum that is within a couple
// of metres of the true pad pressure.
TEST_F(KinematicChecksTest, Launch_RealBoost_DeadIMU_FromReady_DatumHeldThenBaroOnlyLatches) {
    GroundRefFreeze::State gr;
    float ref = kBoost20260517Rp54_p_pa[0];
    int latched_at = -1, held_at = -1;
    for (int i = 0; i < kBoost20260517Rp54_n; i++) {
        setMockMillis(kBoost20260517Rp54_t_ms[i]);
        if (!kc.launch_flag) {
            switch (GroundRefFreeze::step(gr, kBoost20260517Rp54_t_ms[i] * 1000u, kBoost20260517Rp54_p_pa[i])) {
                case GroundRefFreeze::Verdict::Track:      ref = kBoost20260517Rp54_p_pa[i]; break;
                case GroundRefFreeze::Verdict::FreezeEdge: ref = gr.rollback_pa; if (held_at < 0) held_at = i; break;
                case GroundRefFreeze::Verdict::Frozen:     break;
            }
        }
        const float palt = 44330.0f * (1.0f - powf(kBoost20260517Rp54_p_pa[i] / ref, 1.0f / 5.255f));
        float pos[3] = {0, 0, 0}, vel[3] = {0, 0, 0};
        kc.kinematicChecks(palt, 0.0f, pos, vel, 0.0f,
                           kBoost20260517Rp54_baro_new[i] != 0, 0.0f, false, 1.57f, false, false,
                           0.0f, true, /*baro_healthy=*/true, /*imu_healthy=*/false);
        if (kc.launch_flag && latched_at < 0) latched_at = i;
    }
    ASSERT_GE(held_at, 0) << "the datum must be held during the boost";
    ASSERT_GE(latched_at, 0) << "a launch from READY with a dead IMU must still latch";
    const int t0 = (int)kBoost20260517Rp54_t_ms[kBoost20260517Rp54_boost0];
    EXPECT_LE((int)kBoost20260517Rp54_t_ms[held_at] - t0, 600) << "held late";
    EXPECT_GE(latched_at, held_at);
    EXPECT_EQ(kc.launch_path, TR_KinematicChecks::LaunchPath::BaroOnly);
    // datum error vs the pad mean, in metres
    const float datum_err_m = 44330.0f * (1.0f - powf(ref / kBoost20260517Rp54_p_ref, 1.0f / 5.255f));
    EXPECT_LT(fabsf(datum_err_m), 3.0f) << "datum drifted " << datum_err_m << " m";
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

    // Descend ~0.5 m/call (inside the baro rate-gate), burnout still FALSE AND
    // baro_locked_out=TRUE.  This is the exact dead-IMU failure mode found on the
    // bench (2026-07-21): a dead IMU freezes the EKF velocity, so the transonic
    // mach lockout latches true and never releases (it clears only below
    // BARO_MACH_LOCKOUT_OFF) — which vetoed the backstop and left the vehicle
    // ballistic.  The backstop must NOT be gated on the lockout and must fire
    // anyway.  Stays silent until > 30 m below the peak, then latches.
    uint32_t t = 600;
    float alt = 140.0f;
    bool fired_too_high = false;
    for (int i = 0; i < 160; i++, t += 2) {
        alt -= 0.5f;
        setMockMillis(t);
        callFlight(alt, 5.0f, -10.0f, 0.0f, 0.0f, false, -0.5f,
                   /*burnout*/false, /*baro_lockout*/true);
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
    EXPECT_EQ(kc.launch_path, TR_KinematicChecks::LaunchPath::None);
    EXPECT_FALSE(kc.launch_baro_healthy);
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

// ---------------------------------------------------------------------------
// #824: the landing vote must not be satisfiable by altitude-blind evidence.
// ---------------------------------------------------------------------------

TEST_F(KinematicChecksTest, Landing_NoGPS_SteadyDescentAloft_DoesNotLatch) {
    // The reported failure: with GPS stale the vote drops to 2-of-3, and
    // gyro_quiet + accel_1g both pass under a canopy at terminal velocity
    // (an accelerometer reads 1 g in steady descent).  Neither knows the
    // rocket is 300 m up, so the pair must not be able to latch LANDED.
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    kc.apogee_flag = true;

    // 60 s under canopy: quiet in roll, 1 g, no GPS, descending 3 m/s from
    // 300 m — still well above the pad when the window ends.
    for (int second = 0; second < 60; second++) {
        uint32_t base = 1000 + second * 1000;
        float alt = 300.0f - 3.0f * float(second);
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(alt, 9.81f, -3.0f, 1.0f);
        }
    }
    EXPECT_TRUE(kc.gyro_quiet_flag);   // the altitude-blind pair does pass...
    EXPECT_TRUE(kc.accel_1g_flag);
    EXPECT_FALSE(kc.baro_stable_flag); // ...but the altitude-aware one does not
    EXPECT_FALSE(kc.alt_landed_flag);  // so no latch
}

TEST_F(KinematicChecksTest, Landing_Quiescent_OutOfBandBaro_EventuallyLatches) {
    // Requiring baro_stable must not strand a flight whose barometer can
    // never satisfy it.  Landing 400 m off the pad reference puts palt
    // outside BARO_STABLE_PALT_MAX forever; the quiescence backstop still
    // ends the flight after ~30 s of genuine stillness.
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    kc.apogee_flag = true;

    for (int second = 0; second < 40; second++) {
        uint32_t base = 1000 + second * 1000;
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(400.0f, 9.80665f, 0.0f, 0.2f);
        }
    }
    EXPECT_FALSE(kc.baro_stable_flag);
    EXPECT_TRUE(kc.quiescent_flag);
    EXPECT_TRUE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_Quiescent_UnderChute_DoesNotLatch) {
    // Same 40 s aloft, but rolling the way the flight logs actually show
    // (median 40-460 dps under chute).  Quiescence must stay clear.
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    kc.apogee_flag = true;

    for (int second = 0; second < 40; second++) {
        uint32_t base = 1000 + second * 1000;
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(300.0f, 9.81f, -6.0f, 40.0f);
        }
    }
    EXPECT_FALSE(kc.quiescent_flag);
    EXPECT_FALSE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_Quiescent_PendulumAccel_DoesNotLatch) {
    // The case the roll gate alone would miss: an airframe descending
    // without rolling.  A canopy pendulum modulates |a| well beyond the
    // 0.05 g quiescence tolerance even when roll is near zero.
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    kc.apogee_flag = true;

    for (int second = 0; second < 40; second++) {
        uint32_t base = 1000 + second * 1000;
        // +/- 0.2 g swing, alternating each second
        float acc = (second % 2 == 0) ? 11.77f : 7.85f;
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(300.0f, acc, -6.0f, 0.5f);
        }
    }
    EXPECT_FALSE(kc.quiescent_flag);
    EXPECT_FALSE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_Quiescent_NotArmedBeforeApogee) {
    // A rocket sitting on the pad is quiescent by definition.  The apogee
    // rising-edge reset must zero the counter so the backstop cannot fire
    // the instant apogee latches.
    for (int second = 0; second < 60; second++) {
        for (int i = 0; i < 50; i++) {
            setMockMillis(second * 1000 + i * 2);
            callStationary(0.0f, 9.80665f);
        }
    }
    for (int i = 0; i < 80; i++) {
        setMockMillis(60000 + i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);

    kc.apogee_flag = true;
    setMockMillis(62000);
    callFlight(300.0f, 9.80665f, -6.0f, 0.0f);
    EXPECT_FALSE(kc.quiescent_flag);
    EXPECT_FALSE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_FrozenBaro_CannotSupplyTheMandatoryVoter) {
    // #824 follow-up: baro_stable is mandatory precisely because it is the
    // only altitude-aware voter, so an unhealthy barometer must not be able
    // to satisfy it.  A frozen sensor retains its last reading, which makes
    // landing_altitude_change exactly 0 — maximally "stable" — so without the
    // baro_healthy gate a baro stuck at an in-band value would hand the vote
    // its mandatory voter while the rocket is still descending.
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    kc.apogee_flag = true;

    // Baro frozen at 30 m (in band, zero delta) while actually descending;
    // the IMU pair is quiet, so the vote would otherwise be 3 of 3.
    for (int second = 0; second < 20; second++) {
        uint32_t base = 1000 + second * 1000;
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(30.0f, 9.81f, -6.0f, 1.0f, 0.0f, false, 1.57f,
                       false, false, 0.0f, true, /*baro_healthy=*/false);
        }
    }
    EXPECT_TRUE(kc.gyro_quiet_flag);
    EXPECT_TRUE(kc.accel_1g_flag);
    EXPECT_FALSE(kc.baro_stable_flag) << "an unhealthy baro must not satisfy baro_stable";
    EXPECT_FALSE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_HealthyBaro_StillLatchesNormally) {
    // The gate above must not break the ordinary case: a healthy baro settled
    // near the pad still latches baro_stable and votes the rocket down.
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    kc.apogee_flag = true;

    for (int second = 0; second < 7; second++) {
        uint32_t base = 1000 + second * 1000;
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(5.0f, 9.81f, 0.0f, 1.0f);
        }
    }
    EXPECT_TRUE(kc.baro_stable_flag);
    EXPECT_TRUE(kc.alt_landed_flag);
}

TEST_F(KinematicChecksTest, Landing_DeadBaro_QuiescenceStillNeedsAQuietIMU) {
    // With baro_healthy false the quiescence detector drops its altitude term
    // and runs on the IMU alone.  A frozen IMU is the one input that would
    // satisfy both gates forever, which is why both callers zero acc_mag when
    // the IMU is stale.  Confirm a moving airframe still cannot latch.
    for (int i = 0; i < 80; i++) {
        setMockMillis(i * 2);
        callFlight(float(i), 25.0f, 10.0f);
    }
    ASSERT_TRUE(kc.launch_flag);
    kc.apogee_flag = true;

    for (int second = 0; second < 60; second++) {
        uint32_t base = 1000 + second * 1000;
        float alt = 300.0f - 3.0f * float(second);
        // Real canopy accel scatter: the 0.05 g gate is missed most ticks.
        float acc = (second % 3 == 0) ? 9.80665f : 10.8f;
        for (int i = 0; i < 50; i++) {
            setMockMillis(base + i * 2);
            callFlight(alt, acc, -3.0f, 1.0f, 0.0f, false, 1.57f,
                       false, false, 0.0f, true, /*baro_healthy=*/false);
        }
    }
    EXPECT_FALSE(kc.quiescent_flag) << "a <50% duty cycle cannot climb a leaky counter";
    EXPECT_FALSE(kc.alt_landed_flag);
}
