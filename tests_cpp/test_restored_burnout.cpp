// #1776: burnout on a flight restored from a snapshot that has none
// (RestoredBurnout.h).
//
// The OC serves the flight token's frame, copied at the PRELAUNCH -> INFLIGHT
// edge, when it boots from a power loss with no RAM cache and no NAND frame
// (#1717). The FC restores from it with launch set and burnout and apogee
// unset. The apogee vote, and with a healthy IMU the barometer backstop, wait
// for burnout; the live detector latches on 50 ms of negative axial specific
// force, which a section hanging nose-up under the drogue never reads.
//
// The policy tests drive the recovery arm gate and the rule as the flight loop
// steps them. The flight tests add the real kinematic checks, the live burnout
// detector and the main-deploy gate, at the flight loop's 1 kHz, from such a
// restore under a drogue.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "BurnoutDetector.h"
#include "MainDeployGate.h"
#include "RecoveryArmGate.h"
#include "RestoredBurnout.h"
#include "TR_KinematicChecks.h"

using RestoredBurnout::Evidence;

namespace {

constexpr float kG = 9.80665f;
constexpr uint32_t kT0 = 10000;   // the restore's boot time, ms

// ---------------------------------------------------------------------------
// The policy, with the gate, at a 1 ms tick.
// ---------------------------------------------------------------------------

RecoveryArmGate::Inputs drogueNoseUp()
{
    RecoveryArmGate::Inputs in;
    in.baro_healthy   = true;
    in.baro_rate_mps  = -20.0f;
    in.imu_fresh      = true;
    in.accel_norm_ms2 = kG;
    in.gyro_norm_dps  = 10.0f;
    return in;
}

RecoveryArmGate::Inputs tableStill()
{
    RecoveryArmGate::Inputs in;
    in.baro_healthy   = true;
    in.baro_rate_mps  = 0.0f;
    in.imu_fresh      = true;
    in.accel_norm_ms2 = kG;
    in.gyro_norm_dps  = 0.0f;
    return in;
}

struct Rig
{
    RecoveryArmGate::State gate;
    RestoredBurnout::State rb;
    bool     burnout    = false;
    uint32_t latched_ms = 0;
    Evidence ev         = Evidence::None;

    Rig() { RecoveryArmGate::reset(gate, kT0); }

    // One flight-loop tick: the gate, then the rule, as main.cpp steps them.
    void tick(RecoveryArmGate::Inputs in, uint32_t now_ms, bool step_gate = true)
    {
        in.now_ms = now_ms;
        in.flight_elapsed_ms = now_ms - kT0;
        if (step_gate) RecoveryArmGate::step(gate, in);
        const Evidence e = RestoredBurnout::step(rb, !burnout, gate, in);
        if (e != Evidence::None)
        {
            burnout    = true;
            latched_ms = now_ms;
            ev         = e;
        }
    }
    // [from, to) at 1 ms, relative to the restore.
    void run(const RecoveryArmGate::Inputs& in, uint32_t from_rel, uint32_t to_rel,
             bool step_gate = true)
    {
        for (uint32_t t = from_rel; t < to_rel; ++t) tick(in, kT0 + t, step_gate);
    }
};

TEST(RestoredBurnoutPolicy, NoseUpUnderTheDrogue_LatchesOnA3sDescentOnceTheGateIsOpen)
{
    Rig r;
    r.run(drogueNoseUp(), 0, 5000);
    ASSERT_EQ(r.gate.verdict, RecoveryArmGate::Verdict::Open);
    EXPECT_EQ(r.gate.opened_by, RecoveryArmGate::Arm::Descent);
    EXPECT_EQ(r.gate.open_since, kT0 + 1600u) << "the gate's own 1 s descent arm";
    ASSERT_TRUE(r.burnout);
    EXPECT_EQ(r.ev, Evidence::Descent);
    // the gate's 600 ms barometer settle, then the latch's 3 s descent hold
    EXPECT_EQ(r.latched_ms, kT0 + 3600u);
    EXPECT_EQ(RestoredBurnout::Config{}.descent_hold_ms, 3000u);
    EXPECT_EQ(RestoredBurnout::Config{}.gate.descent_mps, RecoveryArmGate::Config{}.descent_mps);
}

TEST(RestoredBurnoutPolicy, NeverWhileTheGateIsLocked_AndAtOnceWhenItOpens)
{
    Rig r;
    // The evidence holds, but the gate is kept Locked (not stepped).
    r.run(drogueNoseUp(), 0, 5000, /*step_gate=*/false);
    EXPECT_FALSE(r.burnout);
    r.gate.verdict    = RecoveryArmGate::Verdict::Open;
    r.gate.open_since = kT0 + 5000u;
    r.tick(drogueNoseUp(), kT0 + 5000u, /*step_gate=*/false);
    EXPECT_TRUE(r.burnout) << "evidence this boot measured while Locked counts once Open";
    EXPECT_EQ(r.latched_ms, kT0 + 5000u);
}

TEST(RestoredBurnoutPolicy, InertOnceBurnoutIsSet_AndItsHoldsClear)
{
    Rig r;
    r.run(drogueNoseUp(), 0, 1000);   // evidence accumulating, not yet latched
    ASSERT_NE(r.rb.calm_since, 0u);
    ASSERT_NE(r.rb.descent_since, 0u);
    r.burnout = true;   // then the live detector latches it
    r.run(drogueNoseUp(), 1000, 5000);
    EXPECT_EQ(r.ev, Evidence::None);
    EXPECT_EQ(r.rb.descent_since, 0u);
    EXPECT_EQ(r.rb.freefall_since, 0u);
    EXPECT_EQ(r.rb.calm_since, 0u);
}

TEST(RestoredBurnoutPolicy, UnderThrust_APortDipDescentNeverLatches)
{
    // #1706's case after a restore: a burning motor (4 g) with the static
    // port reading a 30 m/s descent. The gate opens on its boost arm.
    RecoveryArmGate::Inputs in = drogueNoseUp();
    in.accel_norm_ms2 = 4.0f * kG;
    in.baro_rate_mps  = -30.0f;
    Rig r;
    r.run(in, 0, 10000);
    EXPECT_EQ(r.gate.opened_by, RecoveryArmGate::Arm::Boost);
    EXPECT_FALSE(r.burnout);
}

TEST(RestoredBurnoutPolicy, TwoGIsTheBar)
{
    RecoveryArmGate::Inputs at = drogueNoseUp();
    at.accel_norm_ms2 = RestoredBurnout::Config{}.calm_max_ms2;
    Rig a;
    a.run(at, 0, 5000);
    EXPECT_TRUE(a.burnout) << "at 2 g";

    RecoveryArmGate::Inputs over = at;
    over.accel_norm_ms2 = std::nextafter(at.accel_norm_ms2, 100.0f);
    Rig b;
    b.run(over, 0, 10000);
    EXPECT_FALSE(b.burnout) << "just over 2 g";
    EXPECT_FLOAT_EQ(RestoredBurnout::Config{}.calm_max_ms2, 2.0f * RecoveryArmGate::kG_MS2);
}

TEST(RestoredBurnoutPolicy, ASwingOver2gRestartsTheQuarterSecondHold)
{
    Rig r;
    RecoveryArmGate::Inputs in = drogueNoseUp();
    RecoveryArmGate::Inputs snatch = in;
    snatch.accel_norm_ms2 = 25.0f;
    // One tick over 2 g every 200 ms for 5 s: the descent holds, calm never does.
    for (uint32_t t = 0; t < 5000; ++t) r.tick(t % 200 == 0 ? snatch : in, kT0 + t);
    EXPECT_EQ(r.gate.verdict, RecoveryArmGate::Verdict::Open);
    EXPECT_FALSE(r.burnout);
    // The descent held from 3600; the last swing was at 4800 ms, so calm
    // from 4801, and 250 ms later.
    r.run(in, 5000, 6000);
    ASSERT_TRUE(r.burnout);
    EXPECT_EQ(r.latched_ms, kT0 + 4801u + 250u);
}

TEST(RestoredBurnoutPolicy, AStaleImuNeverLatches)
{
    // The flight loop passes 0 m/s^2 for a stale IMU: not calm, not free fall.
    RecoveryArmGate::Inputs in = drogueNoseUp();
    in.imu_fresh      = false;
    in.accel_norm_ms2 = 0.0f;
    in.gyro_norm_dps  = 0.0f;
    Rig r;
    r.run(in, 0, 10000);
    EXPECT_EQ(r.gate.opened_by, RecoveryArmGate::Arm::Descent);
    EXPECT_FALSE(r.burnout);
}

TEST(RestoredBurnoutPolicy, FreeFall_LatchesOnTheGatesFreeFallArm)
{
    RecoveryArmGate::Inputs in = drogueNoseUp();
    in.baro_healthy   = false;   // no descent evidence
    in.accel_norm_ms2 = 1.0f;
    Rig r;
    r.run(in, 0, 5000);
    EXPECT_EQ(r.gate.opened_by, RecoveryArmGate::Arm::FreeFall);
    ASSERT_TRUE(r.burnout);
    EXPECT_EQ(r.ev, Evidence::FreeFall);
    EXPECT_EQ(r.latched_ms, kT0 + 2000u);
}

TEST(RestoredBurnoutPolicy, ALowThrustBurn_APortStepReadAsDescent_DoesNotLatch)
{
    // A long-burn motor at 1.5 g of specific force, restored from the token
    // during its burn, with the static port reading a 10 m/s descent for
    // 2.5 s (a transonic step, smoothed by the altitude filter), then the
    // climb again. The gate opens on its 1 s descent arm.
    RecoveryArmGate::Inputs dip = drogueNoseUp();
    dip.accel_norm_ms2 = 1.5f * kG;
    dip.baro_rate_mps  = -10.0f;
    RecoveryArmGate::Inputs climb = dip;
    climb.baro_rate_mps = 200.0f;
    Rig r;
    r.run(dip, 0, 3100);
    r.run(climb, 3100, 10000);
    EXPECT_EQ(r.gate.opened_by, RecoveryArmGate::Arm::Descent);
    EXPECT_FALSE(r.burnout) << "the 3 s descent hold outlasts the step";

    // With the gate's own 1 s hold it would have latched under thrust.
    RestoredBurnout::Config one_second;
    one_second.descent_hold_ms = 1000;
    RecoveryArmGate::State gate;
    RecoveryArmGate::reset(gate, kT0);
    RestoredBurnout::State rb;
    bool latched = false;
    for (uint32_t t = 0; t < 3100 && !latched; ++t)
    {
        RecoveryArmGate::Inputs in = dip;
        in.now_ms = kT0 + t;
        in.flight_elapsed_ms = t;
        RecoveryArmGate::step(gate, in);
        latched = RestoredBurnout::step(rb, true, gate, in, one_second) != Evidence::None;
    }
    EXPECT_TRUE(latched);
}

TEST(RestoredBurnoutPolicy, ATailOffThroughTwoGWhileClimbing_DoesNotLatch)
{
    // A restore under thrust at 4 g, the tail-off through 2 g to 1 g of drag,
    // and the coast climbing at 150 m/s: calm, but no descent.
    Rig r;
    for (uint32_t t = 0; t < 6000; ++t)
    {
        RecoveryArmGate::Inputs in = drogueNoseUp();
        in.baro_rate_mps  = 150.0f;
        in.accel_norm_ms2 = t < 1000 ? 4.0f * kG
                          : t < 1500 ? (4.0f - 3.0f * (float)(t - 1000) / 500.0f) * kG
                                     : 1.0f * kG;
        r.tick(in, kT0 + t);
    }
    EXPECT_EQ(r.gate.opened_by, RecoveryArmGate::Arm::Boost);
    EXPECT_FALSE(r.burnout);
}

TEST(RestoredBurnoutPolicy, AGateOpenedByItsBoostArm_StillWaitsForItsOwnDescent)
{
    // A restore under thrust: 5 g, climbing, for 1 s. Then the drogue.
    RecoveryArmGate::Inputs boost = drogueNoseUp();
    boost.accel_norm_ms2 = 5.0f * kG;
    boost.baro_rate_mps  = 100.0f;
    Rig r;
    r.run(boost, 0, 1000);
    ASSERT_EQ(r.gate.opened_by, RecoveryArmGate::Arm::Boost);
    EXPECT_FALSE(r.burnout);
    r.run(drogueNoseUp(), 1000, 5000);
    ASSERT_TRUE(r.burnout);
    EXPECT_EQ(r.ev, Evidence::Descent);
    EXPECT_EQ(r.latched_ms, kT0 + 1000u + 3000u) << "the 3 s descent hold, from the drogue";
}

TEST(RestoredBurnoutPolicy, AGateOpenedByItsSpinArm_LatchesOnceTheBarometerShowsTheDescent)
{
    // Under a spinning drogue with the barometer not yet back: the spin arm
    // opens the gate at 2 s. The barometer returns at 3 s, and the descent
    // holds 3 s from there.
    RecoveryArmGate::Inputs spin = drogueNoseUp();
    spin.gyro_norm_dps = 300.0f;
    spin.baro_healthy  = false;
    Rig r;
    r.run(spin, 0, 3000);
    ASSERT_EQ(r.gate.opened_by, RecoveryArmGate::Arm::Spin);
    EXPECT_FALSE(r.burnout);
    spin.baro_healthy = true;
    r.run(spin, 3000, 8000);
    ASSERT_TRUE(r.burnout);
    EXPECT_EQ(r.latched_ms, kT0 + 6000u);
}

TEST(RestoredBurnoutPolicy, TheBarometerIsBelievedAfterTheGatesSettle)
{
    // The gate forced Open from the restore: the descent still waits for the
    // altitude filter's settle (600 ms), then holds 3 s.
    Rig r;
    r.gate.verdict    = RecoveryArmGate::Verdict::Open;
    r.gate.open_since = kT0;
    r.run(drogueNoseUp(), 0, 3599, /*step_gate=*/false);
    EXPECT_FALSE(r.burnout);
    r.run(drogueNoseUp(), 3599, 3601, /*step_gate=*/false);
    ASSERT_TRUE(r.burnout);
    EXPECT_EQ(r.latched_ms, kT0 + 3600u);
}

TEST(RestoredBurnoutPolicy, NotANumberNeverLatches)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    RecoveryArmGate::Inputs a = drogueNoseUp();
    a.accel_norm_ms2 = nan;
    Rig ra;
    ra.gate.verdict = RecoveryArmGate::Verdict::Open;
    ra.run(a, 0, 5000, /*step_gate=*/false);
    EXPECT_FALSE(ra.burnout) << "a NaN specific force is not calm";

    RecoveryArmGate::Inputs b = drogueNoseUp();
    b.baro_rate_mps = nan;
    Rig rb;
    rb.gate.verdict = RecoveryArmGate::Verdict::Open;
    rb.run(b, 0, 5000, /*step_gate=*/false);
    EXPECT_FALSE(rb.burnout) << "a NaN rate is no descent";
}

TEST(RestoredBurnoutPolicy, OnTheGround_AStaleTokenNeverLatchesAndTheGateRefutes)
{
    Rig r;
    r.run(tableStill(), 0, 31000);
    EXPECT_EQ(r.gate.verdict, RecoveryArmGate::Verdict::Refuted);
    EXPECT_FALSE(r.burnout);
}

TEST(RestoredBurnoutPolicy, EvidenceNames)
{
    EXPECT_STREQ(RestoredBurnout::evidenceName(Evidence::Descent), "a barometric descent");
    EXPECT_STREQ(RestoredBurnout::evidenceName(Evidence::FreeFall), "free fall");
    EXPECT_STREQ(RestoredBurnout::evidenceName(Evidence::None), "none");
}

// ---------------------------------------------------------------------------
// A restored flight under the drogue, end to end, at 1 kHz.
// ---------------------------------------------------------------------------

enum class Hang { NoseUp, NoseDown, OnTheTable };

struct FlightOpts
{
    Hang  hang        = Hang::NoseUp;
    bool  imu_fresh   = true;
    bool  rule        = true;     // false: main before #1776
    float h0_m        = 700.0f;   // AGL at the restore
    float descent_mps = 20.0f;    // under the drogue
    float main_alt_m  = 150.0f;   // altitude-on-descent main
    float main_after_s = -1.0f;   // >= 0: a time-after-apogee main instead
    float baro_noise_m = 0.0f;    // uniform +- on each reading
};

struct FlightResult
{
    bool     burnout = false, burnout_by_rule = false;
    float    burnout_s = 0.0f;
    bool     apogee = false, backstop = false;
    float    apogee_s = 0.0f;
    bool     drogue = false;
    float    drogue_s = 0.0f;
    bool     main = false;
    float    main_s = 0.0f, main_alt_m = 0.0f;
    RecoveryArmGate::Verdict verdict = RecoveryArmGate::Verdict::Locked;
};

FlightResult flyRestored(const FlightOpts& o)
{
    TR_KinematicChecks kc;
    setMockMillis(kT0);
    kc.reset();
    // The restore from the token's launch-edge frame: launch set, the maxima
    // 0, burnout and apogee unset, the flight 0 ms old.
    kc.launch_flag = true;
    kc.restoreFlightMaxima(0.0f, 0.0f, /*past_apogee=*/false);
    const uint32_t launch_ms = kT0;

    RecoveryArmGate::State gate;
    RecoveryArmGate::reset(gate, kT0);
    RestoredBurnout::State rb;
    MainDeployGate::State mdg;
    MainDeployGate::reset(mdg, kT0, /*after_reboot=*/true);

    bool     burnout = false, apogee = false, apogee_armed = false;
    uint16_t neg = 0;
    uint32_t apogee_ms = 0;
    FlightResult res;

    const float t_land = o.hang == Hang::OnTheTable ? 40.0f : o.h0_m / o.descent_mps;
    uint32_t lcg = 12345u;   // deterministic barometer noise
    float baro_reading = 0.0f;
    for (uint32_t ms = 0; ms < (uint32_t)(t_land * 1000.0f) - 1000u; ++ms)
    {
        const uint32_t now = kT0 + ms;
        setMockMillis(now);
        const float t = ms / 1000.0f;
        const float alt = o.hang == Hang::OnTheTable ? 0.0f : o.h0_m - o.descent_mps * t;
        // the swing under the drogue, about +-1 m/s^2 on the axis
        const float swing = o.hang == Hang::OnTheTable ? 0.0f : 1.0f * sinf(2.0f * 3.14159265f * 0.7f * t);
        const float fx = (o.hang == Hang::NoseDown ? -kG : kG) + swing;
        const float roll_dps = o.hang == Hang::OnTheTable ? 0.0f : 10.0f;
        const float accel = o.imu_fresh ? fabsf(fx) : 0.0f;
        const bool new_baro = (ms % 10u) == 0u;
        if (new_baro)
        {
            lcg = lcg * 1664525u + 1013904223u;
            const float u = (float)(lcg >> 8) / 16777216.0f;   // [0, 1)
            baro_reading = alt + o.baro_noise_m * (2.0f * u - 1.0f);
        }

        // the kinematic checks, as the flight loop calls them after a restore:
        // the vote by sensor with the inertial voters not flying, no GNSS
        float pos[3] = {0.0f, 0.0f, 0.0f};
        float vel[3] = {0.0f, 0.0f, 0.0f};
        TR_KinematicChecks::InertialVote iv;
        iv.in_use  = true;
        iv.healthy = false;
        kc.kinematicChecks(baro_reading, accel, pos, vel, roll_dps, new_baro, 0.0f, false,
                           /*pitch*/ 1.57f, burnout, /*baro_lockout*/ false, 0.0f,
                           /*ekf_healthy*/ true, /*baro_healthy*/ true, o.imu_fresh,
                           /*gnss_may_vote*/ false, /*baro_reading_fresh*/ true, &iv);
        if (!apogee && kc.apogee_flag)
        {
            apogee = true;
            apogee_ms = now;
            res.apogee = true;
            res.apogee_s = t;
            res.backstop = kc.apogee_backstop_flag;
        }

        // the main-deploy gate, then the recovery arm gate and the rule
        MainDeployGate::Inputs g;
        g.now_ms         = now;
        g.apogee         = apogee;
        g.impact_flag    = kc.impact_flag;
        g.quiescent_flag = kc.quiescent_flag;
        g.baro_healthy   = true;
        g.baro_alt_m     = kc.alt_est;
        g.baro_rate_mps  = kc.d_alt_est_;
        MainDeployGate::step(mdg, g);

        RecoveryArmGate::Inputs r;
        r.now_ms            = now;
        r.flight_elapsed_ms = now - launch_ms;
        r.baro_healthy      = true;
        r.baro_rate_mps     = kc.d_alt_est_;
        r.imu_fresh         = o.imu_fresh;
        r.accel_norm_ms2    = accel;
        r.gyro_norm_dps     = o.imu_fresh ? roll_dps : 0.0f;
        r.quiescent_flag    = kc.quiescent_flag;
        RecoveryArmGate::step(gate, r);
        if (o.rule)
        {
            const Evidence e = RestoredBurnout::step(rb, !burnout, gate, r);
            if (e != Evidence::None)
            {
                burnout = true;
                res.burnout = true;
                res.burnout_by_rule = true;
                res.burnout_s = t;
            }
        }
        if (!apogee_armed && RecoveryArmGate::apogeeArmed(gate, now)) apogee_armed = true;

        // the triggers: a drogue at 0 s after apogee, and the main
        const bool apogee_usable = apogee && apogee_armed && RecoveryArmGate::armingPermitted(gate);
        if (apogee_usable && !res.drogue)
        {
            res.drogue = true;
            res.drogue_s = t;
        }
        if (apogee_usable && !res.main)
        {
            const bool due = o.main_after_s >= 0.0f
                ? (float)(now - apogee_ms) / 1000.0f >= o.main_after_s
                : MainDeployGate::evaluate(mdg, g, o.main_alt_m) != MainDeployGate::Source::None;
            if (due)
            {
                res.main = true;
                res.main_s = t;
                res.main_alt_m = alt;
            }
        }

        // the live burnout detector, later in the pass
        if (o.imu_fresh && tr::burnoutDetectStep(burnout, neg, fx, now - launch_ms, 50))
        {
            res.burnout = true;
            res.burnout_s = t;
        }
    }
    res.verdict = gate.verdict;
    return res;
}

TEST(RestoredBurnoutFlight, NoseUpUnderTheDrogue_BeforeTheRule_NeverCallsApogee)
{
    FlightOpts o;
    o.rule = false;
    const FlightResult r = flyRestored(o);
    EXPECT_EQ(r.verdict, RecoveryArmGate::Verdict::Open);
    EXPECT_FALSE(r.burnout);
    EXPECT_FALSE(r.apogee) << "the issue's defect: the vote never runs, the backstop is held";
    EXPECT_FALSE(r.main);
}

TEST(RestoredBurnoutFlight, NoseUpUnderTheDrogue_BurnoutLatches_ThenApogeeAndTheMain)
{
    const FlightResult r = flyRestored(FlightOpts{});
    ASSERT_TRUE(r.burnout_by_rule);
    EXPECT_GE(r.burnout_s, 3.6f);
    EXPECT_LT(r.burnout_s, 4.5f);
    ASSERT_TRUE(r.apogee);
    EXPECT_TRUE(r.backstop) << "the barometer alone calls after a restore";
    EXPECT_GE(r.apogee_s, r.burnout_s + 1.0f) << "the backstop keeps its settle after burnout";
    EXPECT_LT(r.apogee_s, 6.0f);
    ASSERT_TRUE(r.drogue);
    ASSERT_TRUE(r.main);
    EXPECT_LE(r.main_alt_m, 150.0f);
    EXPECT_GT(r.main_alt_m, 110.0f) << "fired " << r.main_alt_m << " m";
}

TEST(RestoredBurnoutFlight, NoseUpUnderTheDrogue_ANoisyBarometerStillLatchesAndTheMainFires)
{
    FlightOpts o;
    o.baro_noise_m = 2.0f;
    const FlightResult r = flyRestored(o);
    ASSERT_TRUE(r.burnout_by_rule);
    EXPECT_LT(r.burnout_s, 5.0f);
    ASSERT_TRUE(r.apogee);
    ASSERT_TRUE(r.main);
    EXPECT_LE(r.main_alt_m, 150.0f);
    EXPECT_GT(r.main_alt_m, 110.0f) << "fired " << r.main_alt_m << " m";
}

TEST(RestoredBurnoutFlight, ALowRestore_TheMainFiresLowerThanSet)
{
    // Restored at 200 m: the 3 s hold and the backstop's settle put the call
    // under the main's 150 m, and it fires on the handover.
    FlightOpts o;
    o.h0_m = 200.0f;
    const FlightResult r = flyRestored(o);
    ASSERT_TRUE(r.apogee);
    ASSERT_TRUE(r.main);
    EXPECT_LT(r.main_alt_m, 150.0f);
    EXPECT_GT(r.main_alt_m, 90.0f) << "fired " << r.main_alt_m << " m";
}

TEST(RestoredBurnoutFlight, NoseUpUnderTheDrogue_ATimeAfterApogeeMainFires)
{
    FlightOpts o;
    o.main_after_s = 8.0f;
    const FlightResult r = flyRestored(o);
    ASSERT_TRUE(r.apogee);
    ASSERT_TRUE(r.main);
    EXPECT_NEAR(r.main_s - r.apogee_s, 8.0f, 0.01f);
}

TEST(RestoredBurnoutFlight, NoseDownUnderTheDrogue_TheLiveDetectorLatchesFirst)
{
    FlightOpts o;
    o.hang = Hang::NoseDown;
    const FlightResult r = flyRestored(o);
    ASSERT_TRUE(r.burnout);
    EXPECT_FALSE(r.burnout_by_rule);
    EXPECT_LT(r.burnout_s, 0.3f) << "the 200 ms lockout, then 50 samples";
    ASSERT_TRUE(r.apogee);
    ASSERT_TRUE(r.main);
}

TEST(RestoredBurnoutFlight, NoseUp_AStaleImu_TheBackstopCallsWithoutBurnout)
{
    FlightOpts o;
    o.imu_fresh = false;
    const FlightResult r = flyRestored(o);
    EXPECT_FALSE(r.burnout);
    ASSERT_TRUE(r.apogee);
    EXPECT_TRUE(r.backstop);
    ASSERT_TRUE(r.main);
}

TEST(RestoredBurnoutFlight, OnTheGround_AStaleTokenArmsNothing)
{
    FlightOpts o;
    o.hang = Hang::OnTheTable;
    const FlightResult r = flyRestored(o);
    EXPECT_EQ(r.verdict, RecoveryArmGate::Verdict::Refuted);
    EXPECT_FALSE(r.burnout);
    EXPECT_FALSE(r.apogee);
    EXPECT_FALSE(r.drogue);
    EXPECT_FALSE(r.main);
}

}  // namespace
