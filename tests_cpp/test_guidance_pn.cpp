#include <gtest/gtest.h>
#include "TR_GuidancePN.h"
#include <cmath>

class GuidancePNTest : public ::testing::Test {
protected:
    TR_GuidancePN guid;
    static constexpr float TARGET_ALT = 600.0f;
    static constexpr float DT = 0.002f;

    void SetUp() override {
        guid.configure(3.0f, 20.0f, TARGET_ALT);
    }
};

TEST_F(GuidancePNTest, DirectlyBelowTarget_NearZeroLateral) {
    // Rocket at (0, 0, 300) moving straight up toward target at (0, 0, 600)
    float pos_enu[3] = {0.0f, 0.0f, 300.0f};
    // vel_ned: [vN, vE, vD] - moving up means vD < 0
    float vel_ned[3] = {0.0f, 0.0f, -50.0f}; // 50 m/s upward

    guid.update(pos_enu, vel_ned, DT);

    EXPECT_TRUE(guid.isActive());
    // Lateral commands should be near zero when directly below target
    EXPECT_NEAR(guid.getAccelEastCmd(), 0.0f, 0.5f);
    EXPECT_NEAR(guid.getAccelNorthCmd(), 0.0f, 0.5f);
    EXPECT_NEAR(guid.getLateralOffset(), 0.0f, 0.1f);
}

TEST_F(GuidancePNTest, OffsetFromPad_LateralCorrection) {
    // Rocket 50m east of pad, moving up
    float pos_enu[3] = {50.0f, 0.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, -50.0f}; // moving up

    guid.update(pos_enu, vel_ned, DT);

    EXPECT_TRUE(guid.isActive());
    // East accel should be negative (pushing back toward pad)
    EXPECT_LT(guid.getAccelEastCmd(), 0.0f);
    EXPECT_NEAR(guid.getLateralOffset(), 50.0f, 0.1f);
}

TEST_F(GuidancePNTest, AccelClamping) {
    // Large offset -> should clamp to max_accel_mps2 (20 m/s^2)
    float pos_enu[3] = {200.0f, 200.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, -50.0f};

    guid.update(pos_enu, vel_ned, DT);

    float a_mag = std::sqrt(guid.getAccelEastCmd() * guid.getAccelEastCmd() +
                            guid.getAccelNorthCmd() * guid.getAccelNorthCmd() +
                            guid.getAccelUpCmd() * guid.getAccelUpCmd());
    EXPECT_LE(a_mag, 20.0f + 0.01f);
}

TEST_F(GuidancePNTest, CloseRange_Deactivates) {
    // Rocket very close to target -> should deactivate (range < 1m)
    float pos_enu[3] = {0.0f, 0.0f, TARGET_ALT - 0.5f};
    float vel_ned[3] = {0.0f, 0.0f, -1.0f};

    bool active = guid.update(pos_enu, vel_ned, DT);

    EXPECT_FALSE(active);
    EXPECT_FALSE(guid.isActive());
    EXPECT_FLOAT_EQ(guid.getAccelEastCmd(), 0.0f);
    EXPECT_FLOAT_EQ(guid.getAccelNorthCmd(), 0.0f);
    EXPECT_FLOAT_EQ(guid.getAccelUpCmd(), 0.0f);
}

TEST_F(GuidancePNTest, ClosingVelocity_Approaching) {
    // Moving toward target -> positive closing velocity
    float pos_enu[3] = {0.0f, 0.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, -50.0f}; // up toward target

    guid.update(pos_enu, vel_ned, DT);

    EXPECT_GT(guid.getClosingVelocity(), 0.0f);
    EXPECT_FALSE(guid.isCpaReached());
}

TEST_F(GuidancePNTest, CPA_FlagsWhenReceding) {
    // Moving away from target -> negative closing velocity -> CPA reached
    float pos_enu[3] = {0.0f, 0.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, 50.0f}; // downward, away from target

    guid.update(pos_enu, vel_ned, DT);

    EXPECT_LE(guid.getClosingVelocity(), 0.0f);
    EXPECT_TRUE(guid.isCpaReached());
}

TEST_F(GuidancePNTest, Range_Computed) {
    float pos_enu[3] = {30.0f, 40.0f, 100.0f};
    float vel_ned[3] = {0.0f, 0.0f, -10.0f};

    guid.update(pos_enu, vel_ned, DT);

    // Range to target at (0,0,600): sqrt(30^2 + 40^2 + 500^2) = sqrt(900+1600+250000) ~ 502.5
    float expected_range = std::sqrt(30.0f*30.0f + 40.0f*40.0f + 500.0f*500.0f);
    EXPECT_NEAR(guid.getRange(), expected_range, 0.1f);
}

TEST_F(GuidancePNTest, Reset_ClearsAll) {
    float pos_enu[3] = {50.0f, 50.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, -50.0f};
    guid.update(pos_enu, vel_ned, DT);

    guid.reset();

    EXPECT_FLOAT_EQ(guid.getAccelEastCmd(), 0.0f);
    EXPECT_FLOAT_EQ(guid.getAccelNorthCmd(), 0.0f);
    EXPECT_FLOAT_EQ(guid.getAccelUpCmd(), 0.0f);
    EXPECT_FLOAT_EQ(guid.getRange(), 0.0f);
    EXPECT_FLOAT_EQ(guid.getLateralOffset(), 0.0f);
    EXPECT_FALSE(guid.isActive());
    EXPECT_FALSE(guid.isCpaReached());
}

TEST_F(GuidancePNTest, Configure_UpdatesParams) {
    guid.configure(5.0f, 30.0f, 800.0f);

    // Verify new target altitude is used
    float pos_enu[3] = {0.0f, 0.0f, 799.5f};
    float vel_ned[3] = {0.0f, 0.0f, -1.0f};

    // Should deactivate (range < 1m from new 800m target)
    bool active = guid.update(pos_enu, vel_ned, DT);
    EXPECT_FALSE(active);
}

// MODE_PN must DELIBERATELY ignore the horizontal aim point: its r < 1 early
// return latches the sticky cpa_reached_, which the FC uses to kill guidance
// for the rest of the flight.  An overhead target above apogee is never
// reached; a REACHABLE offset target would be, turning a rare failure into the
// nominal case.  Pinned by replaying an identical scenario on two objects that
// differ only in whether a (large, asymmetric) aim point was pushed.
TEST_F(GuidancePNTest, ModePN_IgnoresHorizontalTarget) {
    TR_GuidancePN targeted;
    targeted.configure(3.0f, 20.0f, TARGET_ALT);
    ASSERT_TRUE(targeted.setHorizontalTarget(500.0f, -350.0f));

    float pos_enu[3] = {50.0f, -20.0f, 300.0f};
    float vel_ned[3] = {12.0f, -8.0f, -50.0f};

    guid.update(pos_enu, vel_ned, DT);
    targeted.update(pos_enu, vel_ned, DT);

    EXPECT_FLOAT_EQ(targeted.getAccelEastCmd(),  guid.getAccelEastCmd());
    EXPECT_FLOAT_EQ(targeted.getAccelNorthCmd(), guid.getAccelNorthCmd());
    EXPECT_FLOAT_EQ(targeted.getAccelUpCmd(),    guid.getAccelUpCmd());
    EXPECT_FLOAT_EQ(targeted.getRange(),         guid.getRange());
    EXPECT_FLOAT_EQ(targeted.getLosAngleDeg(),   guid.getLosAngleDeg());
    // In MODE_PN tracking error IS the pad-relative offset.
    EXPECT_FLOAT_EQ(targeted.getTargetOffset(), targeted.getLateralOffset());
}

// ─────────────────────── MODE_STATION_KEEP (#435 / #534) ───────────────────
// Station-keep had ZERO coverage before this file's #435 work: the
// GuidancePNTest fixture above configures MODE_PN, so all 9 tests exercised
// the legacy law only.

class GuidanceStationKeepTest : public ::testing::Test {
protected:
    TR_GuidancePN guid;
    // Gains chosen so the hand-computed commands below stay UNDER the clamp;
    // the clamp gets its own dedicated test.
    static constexpr float KP  = 0.1f;
    static constexpr float KD  = 0.5f;
    static constexpr float AMAX = 20.0f;
    static constexpr float DT  = 0.002f;

    void SetUp() override {
        guid.configureStationKeep(KP, KD, AMAX);
    }
};

// Regression guard: with the aim point at its (0,0) default, the law must
// reduce algebraically to the pre-#435 form.  Hand-computed with the ORIGINAL
// gains, so a change to either term shows up as a numeric mismatch.
TEST_F(GuidanceStationKeepTest, DefaultTarget_ReproducesLegacyLaw) {
    guid.configureStationKeep(0.5f, 1.5f, 20.0f);

    float pos_enu[3] = {60.0f, -25.0f, 300.0f};
    float vel_ned[3] = {12.0f, -8.0f, -40.0f};   // vN=12, vE=-8, vD=-40

    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    // a_e = -0.5*60   - 1.5*(-8) = -30  + 12 = -18
    // a_n = -0.5*(-25)- 1.5*(12) =  12.5- 18 =  -5.5
    EXPECT_NEAR(guid.getAccelEastCmd(),  -18.0f, 1e-4f);
    EXPECT_NEAR(guid.getAccelNorthCmd(),  -5.5f, 1e-4f);
    EXPECT_FLOAT_EQ(guid.getAccelUpCmd(), 0.0f);
    // No aim point set => tracking error == pad offset.
    EXPECT_FLOAT_EQ(guid.getTargetOffset(), guid.getLateralOffset());
    EXPECT_FALSE(guid.isCpaReached());
}

TEST_F(GuidanceStationKeepTest, OnTarget_ZeroVelocity_CommandsZero) {
    ASSERT_TRUE(guid.setHorizontalTarget(120.0f, -80.0f));

    float pos_enu[3] = {120.0f, -80.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, -45.0f};     // purely vertical

    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    EXPECT_NEAR(guid.getAccelEastCmd(),  0.0f, 1e-5f);
    EXPECT_NEAR(guid.getAccelNorthCmd(), 0.0f, 1e-5f);
    EXPECT_NEAR(guid.getTargetOffset(),  0.0f, 1e-3f);
}

// DISCRIMINATING sign test, EAST axis.  The vehicle sits BETWEEN the pad and
// the aim point, so a pad-seeking law and an aim-point-seeking law command
// OPPOSITE directions.  A test with the vehicle outside the target would pass
// under both laws and prove nothing.
TEST_F(GuidanceStationKeepTest, EastOfPad_ShortOfTarget_CommandsEast) {
    ASSERT_TRUE(guid.setHorizontalTarget(80.0f, 0.0f));

    float pos_enu[3] = {60.0f, 0.0f, 300.0f};    // between pad and aim point
    float vel_ned[3] = {0.0f, 0.0f, -45.0f};

    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    // err_E = 60 - 80 = -20 -> a_e = -0.1*(-20) = +2.0 (EAST, toward target).
    // A pad-seeking law would give -0.1*60 = -6.0 (WEST).
    EXPECT_GT(guid.getAccelEastCmd(), 0.0f);
    EXPECT_NEAR(guid.getAccelEastCmd(), 2.0f, 1e-4f);
    EXPECT_NEAR(guid.getAccelNorthCmd(), 0.0f, 1e-5f);
}

// Same discrimination on the NORTH axis, independently — a sign error on one
// axis alone must not hide behind the other.
TEST_F(GuidanceStationKeepTest, NorthOfPad_ShortOfTarget_CommandsNorth) {
    ASSERT_TRUE(guid.setHorizontalTarget(0.0f, 75.0f));

    float pos_enu[3] = {0.0f, 45.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, -45.0f};

    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    // err_N = 45 - 75 = -30 -> a_n = +3.0 (NORTH).  Pad-seeking: -4.5 (SOUTH).
    EXPECT_GT(guid.getAccelNorthCmd(), 0.0f);
    EXPECT_NEAR(guid.getAccelNorthCmd(), 3.0f, 1e-4f);
    EXPECT_NEAR(guid.getAccelEastCmd(), 0.0f, 1e-5f);
}

// Every quantity asymmetric (|E| != |N|, opposite signs, distinct magnitudes)
// so an E/N swap anywhere in the position path cannot pass by symmetry.
TEST_F(GuidanceStationKeepTest, AsymmetricTarget_DetectsEastNorthSwap) {
    ASSERT_TRUE(guid.setHorizontalTarget(40.0f, -15.0f));

    float pos_enu[3] = {10.0f, 25.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, -45.0f};

    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    // err_E = 10 - 40  = -30 -> a_e = +3.0
    // err_N = 25 -(-15)= +40 -> a_n = -4.0
    EXPECT_NEAR(guid.getAccelEastCmd(),   3.0f, 1e-4f);
    EXPECT_NEAR(guid.getAccelNorthCmd(), -4.0f, 1e-4f);
    // Magnitudes differ, so a swap flips both the values AND the sign pattern.
    EXPECT_NEAR(guid.getTargetOffset(), std::sqrt(30.0f*30.0f + 40.0f*40.0f),
                1e-3f);
}

// The damping term is the only consumer of velocity, and update() takes pos in
// ENU but vel in NED — a live footgun.  Park the vehicle exactly on the aim
// point so the position term vanishes and the command IS the velocity term.
TEST_F(GuidanceStationKeepTest, VelocityTerm_UsesNedToEnuConversion) {
    ASSERT_TRUE(guid.setHorizontalTarget(30.0f, -60.0f));

    float pos_enu[3] = {30.0f, -60.0f, 300.0f};  // on target: position term = 0
    float vel_ned[3] = {7.0f, -3.0f, 11.0f};     // vN=7, vE=-3, vD=+11 (falling)

    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    // a_e = -KD * vel_e = -0.5*(-3) = +1.5   (a swap would give -3.5)
    // a_n = -KD * vel_n = -0.5*( 7) = -3.5   (a swap would give +1.5)
    EXPECT_NEAR(guid.getAccelEastCmd(),   1.5f, 1e-4f);
    EXPECT_NEAR(guid.getAccelNorthCmd(), -3.5f, 1e-4f);
    // vD must not leak into the lateral command; vertical stays ballistic.
    EXPECT_FLOAT_EQ(guid.getAccelUpCmd(), 0.0f);
    // v_cl is plain vertical speed in station-keep: ENU up = -NED down.
    EXPECT_NEAR(guid.getClosingVelocity(), -11.0f, 1e-4f);
}

// There is no integrator anywhere in the law, so an arbitrarily distant FINITE
// target must simply saturate the fins — no windup, no instability.  This is
// why the library deliberately does not range-limit the aim point.
TEST_F(GuidanceStationKeepTest, DistantTarget_ClampsToMaxAccel) {
    ASSERT_TRUE(guid.setHorizontalTarget(100000.0f, -70000.0f));

    float pos_enu[3] = {5.0f, -2.0f, 300.0f};
    float vel_ned[3] = {3.0f, -4.0f, -45.0f};

    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    const float a_mag = std::sqrt(
        guid.getAccelEastCmd()  * guid.getAccelEastCmd() +
        guid.getAccelNorthCmd() * guid.getAccelNorthCmd());
    EXPECT_NEAR(a_mag, AMAX, 1e-3f);
    // Saturated, but still pointing at the aim point (NE quadrant is +E,-N).
    EXPECT_GT(guid.getAccelEastCmd(),  0.0f);
    EXPECT_LT(guid.getAccelNorthCmd(), 0.0f);
}

// LOG-INTEGRITY INVARIANT.  getLateralOffset() feeds the logged, int16-cm,
// saturating GuidanceTelemData::lateral_offset_cm and is read as pad-relative
// by three Data_Analysis decoders — it must stay pad-relative for ALL targets.
// Aim-point-relative error goes to the NEW getTargetOffset() instead.
TEST_F(GuidanceStationKeepTest, LateralOffsetStaysPadRelative_TargetOffsetIsError) {
    ASSERT_TRUE(guid.setHorizontalTarget(90.0f, -120.0f));

    float pos_enu[3] = {90.0f, -120.0f, 300.0f};  // exactly ON the aim point
    float vel_ned[3] = {0.0f, 0.0f, -45.0f};

    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    // Tracking error is zero...
    EXPECT_NEAR(guid.getTargetOffset(), 0.0f, 1e-3f);
    // ...while the PAD-relative offset is the full 150 m. These must NOT be
    // the same number, or the logged field has silently changed meaning.
    EXPECT_NEAR(guid.getLateralOffset(), 150.0f, 1e-2f);
}

// getRange() is allowed to be aim-point-relative: it is on no wire struct and
// the FC never calls it.
TEST_F(GuidanceStationKeepTest, Range_IsAimPointRelative) {
    guid.configureStationKeep(KP, KD, AMAX);
    guid.configure(3.0f, AMAX, 600.0f);            // set target_alt via PN cfg
    guid.configureStationKeep(KP, KD, AMAX);       // back to station-keep
    ASSERT_TRUE(guid.setHorizontalTarget(30.0f, -40.0f));

    float pos_enu[3] = {0.0f, 0.0f, 100.0f};
    float vel_ned[3] = {0.0f, 0.0f, -45.0f};

    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    // err = (-30, +40), dh = 600 - 100 = 500 -> sqrt(900+1600+250000)
    const float expected = std::sqrt(30.0f*30.0f + 40.0f*40.0f + 500.0f*500.0f);
    EXPECT_NEAR(guid.getRange(), expected, 0.1f);
}

// SOLE LOG CHANNEL for station-keep tracking error.  getLateralOffset() is
// frozen pad-relative and getTargetOffset() is on no wire struct, so
// los_angle_cdeg (main.cpp:6521) is the ONLY field through which regulation
// error reaches the log — and |pos - target| is NOT reconstructible post-hoc
// from the logged |pos| magnitude plus a known target vector.  A regression to
// atan2f(lateral_offset_m_, dh) is therefore undetectable after the flight.
// Both legs are non-zero AND unequal so the pad-relative mutant gives a
// different number rather than coincidentally matching.
TEST_F(GuidanceStationKeepTest, LosAngle_IsAimPointRelative) {
    ASSERT_TRUE(guid.setHorizontalTarget(30.0f, -40.0f));

    float pos_enu[3] = {60.0f, 80.0f, 100.0f};   // pad offset 100 m ...
    float vel_ned[3] = {0.0f, 0.0f, -45.0f};

    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    // err = (60-30, 80-(-40)) = (30, 120) -> target_offset = sqrt(15300)
    const float target_off = std::sqrt(30.0f*30.0f + 120.0f*120.0f);
    const float dh         = 600.0f - 100.0f;    // default target_alt_m_
    ASSERT_NEAR(guid.getTargetOffset(), target_off, 1e-2f);
    // ... which must NOT equal the pad-relative offset, or this test is blind.
    ASSERT_NEAR(guid.getLateralOffset(), 100.0f, 1e-2f);

    const float expected = std::atan2(target_off, dh) * 180.0f / (float)M_PI;
    EXPECT_NEAR(guid.getLosAngleDeg(), expected, 1e-3f);

    // Explicitly pin that it is NOT the pad-relative angle (~11.31 deg).
    const float pad_relative = std::atan2(100.0f, dh) * 180.0f / (float)M_PI;
    EXPECT_GT(std::fabs(guid.getLosAngleDeg() - pad_relative), 1.0f);
}

// #435 CORE CONTRACT.  TR_GuidancePN.h:71-72 keeps the aim point out of
// configure*() precisely because Drift-Cast RE-PUSHES it as the wind forecast
// updates.  Every other test sets a valid target exactly once per object, and
// NonFiniteTarget_RejectedAndPreviousKept only exercises REJECTED follow-ups —
// so a write-once-latching setter would fly the stalest forecast for the whole
// flight with a fully green suite.  This is the only assertion that a second
// VALID push takes effect.
TEST_F(GuidanceStationKeepTest, SecondValidTarget_ReplacesFirst) {
    ASSERT_TRUE(guid.setHorizontalTarget(50.0f, 0.0f));
    ASSERT_FLOAT_EQ(guid.getTargetEast(),  50.0f);
    ASSERT_FLOAT_EQ(guid.getTargetNorth(),  0.0f);

    // Forecast update: a new aim point in a different quadrant.
    ASSERT_TRUE(guid.setHorizontalTarget(-120.0f, 90.0f));
    EXPECT_FLOAT_EQ(guid.getTargetEast(),  -120.0f);
    EXPECT_FLOAT_EQ(guid.getTargetNorth(),   90.0f);

    // Sitting on the SECOND aim point must command nothing.  Under a latching
    // setter the vehicle is 210 m west / 90 m north of the stale (50,0) target
    // and the law commands hard back toward it.
    float pos_enu[3] = {-120.0f, 90.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, -45.0f};
    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    EXPECT_NEAR(guid.getTargetOffset(),  0.0f, 1e-3f);
    EXPECT_NEAR(guid.getAccelEastCmd(),  0.0f, 1e-4f);
    EXPECT_NEAR(guid.getAccelNorthCmd(), 0.0f, 1e-4f);

    // And the frozen pad-relative field still reports the true displacement.
    EXPECT_NEAR(guid.getLateralOffset(),
                std::sqrt(120.0f*120.0f + 90.0f*90.0f), 1e-2f);
}

TEST_F(GuidanceStationKeepTest, GetMode_TracksLastConfigureCall) {
    EXPECT_EQ(guid.getMode(), TR_GuidancePN::MODE_STATION_KEEP);

    guid.configure(3.0f, 20.0f, 600.0f);
    EXPECT_EQ(guid.getMode(), TR_GuidancePN::MODE_PN);

    guid.configureStationKeep(KP, KD, AMAX);
    EXPECT_EQ(guid.getMode(), TR_GuidancePN::MODE_STATION_KEEP);

    // A fresh object defaults to the legacy law.
    TR_GuidancePN fresh;
    EXPECT_EQ(fresh.getMode(), TR_GuidancePN::MODE_PN);
}

// MERGE BLOCKER.  The FC calls reset() from enterInflight() — AT LAUNCH, after
// both configure sites, with no re-configure anywhere on the launch path.  A
// reset() that cleared the aim point would zero it for the entire flight and
// #435 would fly overhead every time while passing every pre-launch bench
// check.  The pre-existing Reset_ClearsAll test cannot catch this: it never
// re-runs update() afterwards.
TEST_F(GuidanceStationKeepTest, StationKeep_TargetSurvivesReset) {
    ASSERT_TRUE(guid.setHorizontalTarget(50.0f, -35.0f));

    guid.reset();

    // Configuration survived...
    EXPECT_FLOAT_EQ(guid.getTargetEast(),   50.0f);
    EXPECT_FLOAT_EQ(guid.getTargetNorth(), -35.0f);
    EXPECT_EQ(guid.getMode(), TR_GuidancePN::MODE_STATION_KEEP);
    // ...but the per-update output was cleared.
    EXPECT_FLOAT_EQ(guid.getTargetOffset(), 0.0f);

    // The decisive check: sitting on the aim point after a reset must command
    // nothing.  A cleared target would command the vehicle back to the pad.
    float pos_enu[3] = {50.0f, -35.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, -45.0f};
    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    EXPECT_NEAR(guid.getTargetOffset(),  0.0f, 1e-3f);
    EXPECT_NEAR(guid.getAccelEastCmd(),  0.0f, 1e-4f);
    EXPECT_NEAR(guid.getAccelNorthCmd(), 0.0f, 1e-4f);
}

// setHorizontalTarget() and configure*() touch disjoint members, so either
// ordering must work.  Pinned because the FC's #435 wiring is free to pick.
TEST_F(GuidanceStationKeepTest, TargetBeforeConfigure_EquivalentToAfter) {
    TR_GuidancePN before, after;

    before.setHorizontalTarget(70.0f, -45.0f);
    before.configureStationKeep(KP, KD, AMAX);

    after.configureStationKeep(KP, KD, AMAX);
    after.setHorizontalTarget(70.0f, -45.0f);

    float pos_enu[3] = {15.0f, 20.0f, 300.0f};
    float vel_ned[3] = {6.0f, -9.0f, -45.0f};
    before.update(pos_enu, vel_ned, DT);
    after.update(pos_enu, vel_ned, DT);

    EXPECT_FLOAT_EQ(before.getAccelEastCmd(),  after.getAccelEastCmd());
    EXPECT_FLOAT_EQ(before.getAccelNorthCmd(), after.getAccelNorthCmd());
    EXPECT_FLOAT_EQ(before.getTargetOffset(),  after.getTargetOffset());
}

// The finiteness guard is load-bearing: the magnitude clamp tests
// `a_mag > max_accel_mps2_`, which is FALSE for NaN, so a NaN aim point would
// flow UNCLAMPED into the fin command.
TEST_F(GuidanceStationKeepTest, NonFiniteTarget_RejectedAndPreviousKept) {
    ASSERT_TRUE(guid.setHorizontalTarget(25.0f, -18.0f));

    EXPECT_FALSE(guid.setHorizontalTarget(NAN, -18.0f));
    EXPECT_FALSE(guid.setHorizontalTarget(25.0f, NAN));
    EXPECT_FALSE(guid.setHorizontalTarget(INFINITY, -18.0f));
    EXPECT_FALSE(guid.setHorizontalTarget(25.0f, -INFINITY));

    // Previous aim point intact.
    EXPECT_FLOAT_EQ(guid.getTargetEast(),   25.0f);
    EXPECT_FLOAT_EQ(guid.getTargetNorth(), -18.0f);

    float pos_enu[3] = {5.0f, -3.0f, 300.0f};
    float vel_ned[3] = {2.0f, -1.0f, -45.0f};
    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    EXPECT_TRUE(std::isfinite(guid.getAccelEastCmd()));
    EXPECT_TRUE(std::isfinite(guid.getAccelNorthCmd()));
    EXPECT_TRUE(std::isfinite(guid.getTargetOffset()));
}

// Station-keep has no CPA singularity, so it never latches the sticky flag the
// FC uses to disable guidance.  Flagged in the ruling as a #534 hazard: wiring
// station-keep leaves that shutdown path permanently open, and #534 owes a
// replacement deactivation criterion before any flight.
TEST_F(GuidanceStationKeepTest, NeverLatchesCpa_EvenWhileDescending) {
    ASSERT_TRUE(guid.setHorizontalTarget(40.0f, -20.0f));

    float pos_enu[3] = {10.0f, 5.0f, 300.0f};
    float vel_ned[3] = {0.0f, 0.0f, 30.0f};      // vD > 0 => descending

    EXPECT_TRUE(guid.update(pos_enu, vel_ned, DT));

    EXPECT_LT(guid.getClosingVelocity(), 0.0f);  // receding
    EXPECT_FALSE(guid.isCpaReached());           // ...but no latch
    EXPECT_TRUE(guid.isActive());
}
