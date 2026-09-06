// #1176: may a flight restored by reboot recovery arm a deployment channel?
//
// The gate's whole purpose is that its failure mode is REFUSAL. These tests
// are therefore weighted toward the ground cases: the interesting question is
// never "does a real descent open it" (it does, trivially) but "can anything
// that is not a real descent open it".
//
// Two owner rulings are encoded here and the tests are how they stay true:
//   * no arm may use absolute altitude — a reboot cannot establish it;
//   * slow or low never arms — under a main nothing arms, under a drogue it
//     does, because the main charge still has to fire.

#include <gtest/gtest.h>

#include "RecoveryArmGate.h"

using namespace RecoveryArmGate;

namespace {

// Drive the gate for `ms` at a 10 ms tick with a fixed input, advancing
// now_ms. Returns the tick count consumed so callers can keep their clock.
uint32_t run(State& st, Inputs in, uint32_t from_ms, uint32_t ms,
             const Config& cfg = Config{})
{
    for (uint32_t t = 0; t < ms; t += 10)
    {
        in.now_ms = from_ms + t;
        step(st, in, cfg);
    }
    return from_ms + ms;
}

// A vehicle sitting on the ground: 1 g, no rotation, no vertical rate.
Inputs still()
{
    Inputs in;
    in.baro_healthy   = true;
    in.baro_rate_mps  = 0.0f;
    in.imu_fresh      = true;
    in.accel_norm_ms2 = 9.71f;   // pad gravity, m/s^2 — NOT 1.0
    in.gyro_norm_dps  = 0.0f;
    in.gnss_ok        = false;
    in.quiescent_flag = false;
    return in;
}

}  // namespace

// ---------------------------------------------------------------------------
// The default: nothing happens.

TEST(RecoveryArmGate, StartsLockedAndStaysLockedWithNoEvidence)
{
    State st; reset(st, 0);
    EXPECT_FALSE(armingPermitted(st));

    Inputs in = still();
    run(st, in, 0, 20000);
    EXPECT_FALSE(armingPermitted(st));
    EXPECT_FALSE(refuted(st));
    EXPECT_EQ(st.verdict, Verdict::Locked);
}

TEST(RecoveryArmGate, DeadSensorsNeverArm)
{
    // Owner's ruling: there is NO sensorless backstop. A recovery boot with
    // both the barometer and the IMU dead never arms, and the cost — a real
    // flight that loses both makes no deployment — is accepted deliberately.
    State st; reset(st, 0);
    Inputs in;
    in.baro_healthy = false;
    in.imu_fresh    = false;
    in.gnss_ok      = false;
    run(st, in, 0, 120000);
    EXPECT_FALSE(armingPermitted(st));
}

// ---------------------------------------------------------------------------
// Real flight opens it.

TEST(RecoveryArmGate, DrogueDescentOpensOnTheDescentArm)
{
    // ~15 m/s under a drogue: the case that matters, because the main charge
    // still has to fire after the reboot.
    State st; reset(st, 0);
    Inputs in = still();
    in.baro_rate_mps = -15.0f;

    uint32_t t = run(st, in, 0, 500);           // inside the baro settle
    EXPECT_FALSE(armingPermitted(st));
    t = run(st, in, t, 2000);
    EXPECT_TRUE(armingPermitted(st));
    EXPECT_EQ(st.opened_by, Arm::Descent);
}

TEST(RecoveryArmGate, BoostOpensOnTheAccelerationArm)
{
    State st; reset(st, 0);
    Inputs in = still();
    in.accel_norm_ms2 = 120.0f;                 // ~12 g, a live boost
    run(st, in, 0, 1500);
    EXPECT_TRUE(armingPermitted(st));
    EXPECT_EQ(st.opened_by, Arm::Boost);
}

TEST(RecoveryArmGate, BaroDeadUnderDrogueStillOpensOnSpin)
{
    // The reason the spin arm exists at all. Barometer dead, vehicle under a
    // drogue: it is at 1 g (so free-fall is out) and descending (so the
    // descent arm has no sensor), and the main charge still has to fire.
    State st; reset(st, 0);
    Inputs in = still();
    in.baro_healthy  = false;
    in.gyro_norm_dps = 300.0f;
    run(st, in, 0, 2500);
    EXPECT_TRUE(armingPermitted(st));
    EXPECT_EQ(st.opened_by, Arm::Spin);
}

TEST(RecoveryArmGate, GnssOpensOnEitherSignOfVerticalSpeed)
{
    // A reboot can land anywhere in the profile; climbing is as much proof of
    // flight as descending.
    for (float v : {-30.0f, 30.0f})
    {
        State st; reset(st, 0);
        Inputs in = still();
        in.baro_healthy   = false;
        in.gnss_ok        = true;
        in.gnss_vel_u_mps = v;
        run(st, in, 0, 1500);
        EXPECT_TRUE(armingPermitted(st)) << "vertical speed " << v;
        EXPECT_EQ(st.opened_by, Arm::Gnss);
    }
}

// ---------------------------------------------------------------------------
// Ground handling must not open it. This is the half that matters.

TEST(RecoveryArmGate, UnderAMainDoesNotArm)
{
    // Owner's ruling: "There is no need to arm if the rocket is under a main
    // chute. If we are slow or low don't come back and arm any charges."
    State st; reset(st, 0);
    Inputs in = still();
    in.baro_rate_mps = -4.5f;      // a main descent, below the descent bar
    run(st, in, 0, 60000);
    EXPECT_FALSE(armingPermitted(st));
}

TEST(RecoveryArmGate, DroppingTheRocketDoesNotArm)
{
    // The free-fall arm's dwell is its whole defence. Free fall for the full
    // 2 s needs a drop of about 20 m; a drop from bench height gives a few
    // hundred ms and then a hard stop. An earlier draft used 500 ms, which
    // needs only 1.2 m — i.e. it would have armed on a dropped airframe.
    State st; reset(st, 0);
    Inputs in = still();
    in.accel_norm_ms2 = 0.4f;                    // in free fall
    uint32_t t = run(st, in, 0, 600);            // ~1.8 m of fall
    EXPECT_FALSE(armingPermitted(st));

    in.accel_norm_ms2 = 90.0f;                   // impact
    t = run(st, in, t, 100);
    in = still();                                // at rest on the floor
    run(st, in, t, 5000);
    EXPECT_FALSE(armingPermitted(st));
}

TEST(RecoveryArmGate, CarryingAndBumpingDoesNotArm)
{
    // Handling is transient; every arm requires an unbroken hold, so bumps
    // and jostling reset the accumulators before they can mature.
    State st; reset(st, 0);
    Inputs in = still();
    uint32_t t = 0;
    for (int i = 0; i < 40; ++i)
    {
        in.accel_norm_ms2 = 45.0f;  in.gyro_norm_dps = 300.0f;   // a jolt
        t = run(st, in, t, 300);
        in.accel_norm_ms2 = 9.71f;  in.gyro_norm_dps = 20.0f;    // settle
        t = run(st, in, t, 700);
    }
    EXPECT_FALSE(armingPermitted(st));
}

TEST(RecoveryArmGate, WalkingDownhillDoesNotArm)
{
    // The deleted "travel since boot" arm was satisfiable by exactly this.
    // Nothing here uses absolute altitude, so a walk down a hill is invisible:
    // a brisk descent on foot is well under a metre per second.
    State st; reset(st, 0);
    Inputs in = still();
    in.baro_rate_mps = -0.8f;
    run(st, in, 0, 300000);          // five minutes of walking downhill
    EXPECT_FALSE(armingPermitted(st));
}

TEST(RecoveryArmGate, SlowHandRotationDoesNotArm)
{
    // The spin arm is the weakest and is sized against this: 250 dps is about
    // 42 rpm, which a person can produce by hand, so the 2 s unbroken hold is
    // what separates it from handling.
    State st; reset(st, 0);
    Inputs in = still();
    in.gyro_norm_dps = 200.0f;       // below the bar, however long it is held
    run(st, in, 0, 60000);
    EXPECT_FALSE(armingPermitted(st));
}

// ---------------------------------------------------------------------------
// Refutation and the terminal verdicts.

TEST(RecoveryArmGate, SustainedStillnessRefutes)
{
    State st; reset(st, 0);
    Inputs in = still();
    in.quiescent_flag = true;

    uint32_t t = run(st, in, 0, 20000);
    EXPECT_FALSE(refuted(st));                   // not yet — 30 s
    run(st, in, t, 12000);
    EXPECT_TRUE(refuted(st));
    EXPECT_FALSE(armingPermitted(st));
}

TEST(RecoveryArmGate, InterruptedStillnessDoesNotRefute)
{
    // The interruption has to be PHYSICAL, not merely a cleared flag: the gate
    // accumulates stillness from the raw terms as well as from the shipped
    // detector, precisely so a restored flight (whose fresh kinematics can
    // never latch that flag) can still refute. Picking the rocket up shows in
    // both.
    State st; reset(st, 0);
    Inputs in = still();
    in.quiescent_flag = true;
    uint32_t t = run(st, in, 0, 25000);

    in.quiescent_flag = false;
    in.accel_norm_ms2 = 14.0f;                   // lifted
    in.gyro_norm_dps  = 40.0f;
    t = run(st, in, t, 200);

    in = still();
    in.quiescent_flag = true;
    run(st, in, t, 25000);
    EXPECT_FALSE(refuted(st));                   // both holds restarted
}

TEST(RecoveryArmGate, RefutesOnARestoredFlightWithNoQuiescentFlag)
{
    // THE REGRESSION TEST FOR A REAL DEFECT. The shipped quiescence detector
    // gates its pass on apogee_flag, and a restored flight starts with a fresh
    // TR_KinematicChecks whose apogee flag is false — so on the one path this
    // gate exists for, quiescent_flag can NEVER become true. A refutation built
    // only on it is dead code, and a stale-token board would sit INFLIGHT for
    // the full ten-minute flight timeout instead of self-healing in ~30 s.
    State st; reset(st, 0);
    Inputs in = still();
    in.quiescent_flag = false;                   // as a restored flight sees it
    uint32_t t = run(st, in, 0, 20000);
    EXPECT_FALSE(refuted(st));                   // not yet
    run(st, in, t, 12000);
    EXPECT_TRUE(refuted(st));
}

TEST(RecoveryArmGate, AFrozenImuAtOneGIsNotStillness)
{
    // A frozen IMU retains its last sample, and if that sample happened to be
    // ~1 g it would look exactly like a vehicle at rest. Freshness gates the
    // stillness terms for the same reason it gates the arming ones.
    State st; reset(st, 0);
    Inputs in = still();
    in.imu_fresh = false;
    run(st, in, 0, 60000);
    EXPECT_FALSE(refuted(st));
    EXPECT_FALSE(overrideAllowed(st, 60000));
}

TEST(RecoveryArmGate, DescendingUnderCanopyIsNotStillness)
{
    // The inertial terms alone cannot separate sitting in a field from a
    // smooth descent — under a canopy the airframe is near 1 g and may barely
    // roll. The barometric term is what separates them, whenever the baro
    // is usable.
    State st; reset(st, 0);
    Inputs in = still();
    in.baro_rate_mps = -5.5f;                    // under a main
    run(st, in, 0, 60000);
    EXPECT_FALSE(refuted(st));
    EXPECT_FALSE(overrideAllowed(st, 60000));
}

// ---------------------------------------------------------------------------
// The operator override (#1176 step 6).

TEST(RecoveryArmGate, OverrideNeedsPositiveStillnessNotMerelyLocked)
{
    // THE CENTRAL SAFETY PROPERTY. "Not Open" is not "not flying": Locked is
    // the pre-decision state, and a genuine restored flight sits in it for
    // seconds after every restore — and for an entire descent when the
    // barometer is dead. An override that merely required Locked would let a
    // ground station end a real flight in exactly that window.
    State st; reset(st, 0);
    Inputs in = still();
    in.baro_healthy   = false;                   // dead baro: Locked all descent
    in.accel_norm_ms2 = 9.75f;                   // ~1 g under a canopy
    in.gyro_norm_dps  = 30.0f;                   // but rolling
    uint32_t t = run(st, in, 0, 60000);
    ASSERT_EQ(st.verdict, Verdict::Locked);      // Locked, and airborne
    EXPECT_FALSE(overrideAllowed(st, t));        // must still refuse
}

TEST(RecoveryArmGate, OverrideAllowedOnAStationaryBench)
{
    State st; reset(st, 0);
    Inputs in = still();
    uint32_t t = run(st, in, 0, 4000);
    EXPECT_FALSE(overrideAllowed(st, t));        // dwell not met yet
    t = run(st, in, t, 2000);
    EXPECT_TRUE(overrideAllowed(st, t));
}

TEST(RecoveryArmGate, OverrideRefusedOnceTheGateHasOpened)
{
    // Once live evidence has proven the vehicle is flying, no operator
    // assertion may override it.
    State st; reset(st, 0);
    Inputs in = still();
    in.baro_rate_mps = -15.0f;
    uint32_t t = run(st, in, 0, 2000);
    ASSERT_TRUE(armingPermitted(st));

    in = still();                                 // now sitting perfectly still
    t = run(st, in, t, 10000);
    EXPECT_FALSE(overrideAllowed(st, t));
}

TEST(RecoveryArmGate, AFlightPastTheTimeoutIsRefutedRegardlessOfMotion)
{
    // A restored flight older than the FC's own flight timeout is over by
    // definition. Checked before the arms, so a stale token that also happens
    // to be moving cannot open the gate.
    State st; reset(st, 0);
    Inputs in = still();
    in.baro_rate_mps   = -20.0f;                 // would otherwise open A1
    in.flight_elapsed_ms = 600000;
    run(st, in, 0, 3000);
    EXPECT_TRUE(refuted(st));
    EXPECT_FALSE(armingPermitted(st));
}

TEST(RecoveryArmGate, OpenIsLatchedAndSurvivesTheVehicleGoingQuiet)
{
    // Once genuinely flying, a lull must not close the gate — and landing is
    // the landing detector's job, not this one's.
    State st; reset(st, 0);
    Inputs in = still();
    in.baro_rate_mps = -15.0f;
    uint32_t t = run(st, in, 0, 2000);
    ASSERT_TRUE(armingPermitted(st));

    in = still();
    in.quiescent_flag = true;
    run(st, in, t, 60000);
    EXPECT_TRUE(armingPermitted(st));            // still Open
    EXPECT_FALSE(refuted(st));
}

TEST(RecoveryArmGate, RefutedIsTerminalAndCannotBeReopened)
{
    State st; reset(st, 0);
    Inputs in = still();
    in.quiescent_flag = true;
    uint32_t t = run(st, in, 0, 31000);
    ASSERT_TRUE(refuted(st));

    in = still();
    in.baro_rate_mps = -20.0f;                   // would open A1 from Locked
    run(st, in, t, 5000);
    EXPECT_TRUE(refuted(st));
    EXPECT_FALSE(armingPermitted(st));
}

// ---------------------------------------------------------------------------
// The apogee hand-over is a second checkpoint on the same evidence.

TEST(RecoveryArmGate, ApogeeArmsOnlyAfterOpenPersists)
{
    State st; reset(st, 0);
    Inputs in = still();
    in.baro_rate_mps = -15.0f;
    uint32_t t = run(st, in, 0, 2000);
    ASSERT_TRUE(armingPermitted(st));

    // Open, but not yet for long enough to hand over the restored apogee.
    EXPECT_FALSE(apogeeArmed(st, st.open_since));
    EXPECT_FALSE(apogeeArmed(st, st.open_since + 999));
    EXPECT_TRUE(apogeeArmed(st, st.open_since + 1000));
    (void)t;
}

TEST(RecoveryArmGate, ApogeeNeverArmsWhileLockedOrRefuted)
{
    State st; reset(st, 0);
    EXPECT_FALSE(apogeeArmed(st, 100000));       // Locked

    Inputs in = still();
    in.quiescent_flag = true;
    run(st, in, 0, 31000);
    ASSERT_TRUE(refuted(st));
    EXPECT_FALSE(apogeeArmed(st, 100000));       // Refuted
}

// ---------------------------------------------------------------------------
// Structural properties the design rests on.

TEST(RecoveryArmGate, AFrozenImuCannotArm)
{
    // ism6_latest_si retains its last value when the drain yields nothing, so
    // a freeze while the last sample was large would re-present it forever.
    // The caller must gate on freshness; with imu_fresh false, no inertial
    // arm may accumulate however extreme the retained value is.
    State st; reset(st, 0);
    Inputs in = still();
    in.baro_healthy   = false;
    in.imu_fresh      = false;
    in.accel_norm_ms2 = 200.0f;
    in.gyro_norm_dps  = 900.0f;
    run(st, in, 0, 60000);
    EXPECT_FALSE(armingPermitted(st));
}

TEST(RecoveryArmGate, BaroArmIsIgnoredUntilTheFilterHasSettled)
{
    // The altitude filter restarts cold on a recovery boot and free-runs until
    // it has real samples; believing its rate immediately is how a stale rate
    // would arm the gate.
    Config cfg;
    State st; reset(st, 0);
    Inputs in = still();
    in.baro_rate_mps = -50.0f;                   // a free-running filter
    run(st, in, 0, cfg.baro_settle_ms - 100, cfg);
    EXPECT_FALSE(armingPermitted(st));
}

TEST(RecoveryArmGate, ArmingDependsOnRateAloneAndNotOnAccumulatedHeight)
{
    // The structural guarantee, as behaviour: the gate carries no altitude
    // state, so the same descent rate produces the same verdict no matter how
    // much height has already been lost. A reboot cannot establish absolute
    // altitude — a stale token supplies a foreign ground reference and
    // barometric AGL against it is wrong by hundreds of metres — so any arm
    // that accumulated height would be reading a fiction.
    //
    // The counterpart is the deleted "travel since boot" arm, which failed
    // exactly here: it integrated a rate into a distance and a hillside walk
    // satisfied it.
    const uint32_t kElapsed[] = {0, 30000, 120000, 400000};
    for (uint32_t elapsed : kElapsed)
    {
        State st; reset(st, 0);
        Inputs in = still();
        in.flight_elapsed_ms = elapsed;

        in.baro_rate_mps = -4.5f;                // below the bar
        uint32_t t = run(st, in, 0, 5000);
        EXPECT_FALSE(armingPermitted(st)) << "elapsed " << elapsed;

        in.baro_rate_mps = -15.0f;               // above it
        run(st, in, t, 2000);
        EXPECT_TRUE(armingPermitted(st)) << "elapsed " << elapsed;
        EXPECT_EQ(st.opened_by, Arm::Descent);
    }
}
