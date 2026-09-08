// #1137 item 6 — which roll-control law runs this tick.
//
// The three-way choice was spread across three `if` conditions in the flight
// loop, and they did not agree about the EKF: the quorum predicate asked
// `ekf_initialized && isHealthy()`, the CONTROL gate asked only `isHealthy()`.
//
// isHealthy() answers "has the filter diverged recently", not "has it ever
// run" — so on a launch where the EKF never initialised (no GNSS fix, no
// origin) it returns true, and control took the GAIN SCHEDULE branch scaling
// its gains on an EKF speed that was never computed. Speed pinned at 0 sits at
// the schedule's low-speed end, which is its MAXIMUM gain, so a no-GNSS launch
// flew the whole flight at the cap instead of dropping to pure-gyro rate-null.

#include <gtest/gtest.h>

#include "roll_control_mode_policy.h"

using RollControlModePolicy::Mode;
using RollControlModePolicy::select;

TEST(RollControlMode, NeverInitialisedEkfFallsToPureGyro) {
    // THE BUG: healthy-but-never-initialised used to reach GainSchedule.
    EXPECT_EQ(select(/*angle=*/false, /*init=*/false, /*healthy=*/true,
                     /*gain_sched=*/true, /*angle_seg=*/false),
              Mode::PureGyro);
}

TEST(RollControlMode, NeverInitialisedEkfCannotReachAngleModeEither) {
    EXPECT_EQ(select(true, false, true, true, true), Mode::PureGyro);
}

TEST(RollControlMode, InitialisedAndHealthyUsesTheGainSchedule) {
    EXPECT_EQ(select(false, true, true, true, false), Mode::GainSchedule);
}

TEST(RollControlMode, AngleSegmentWinsWhenAngleControlIsOn) {
    EXPECT_EQ(select(true, true, true, true, true), Mode::Angle);
}

TEST(RollControlMode, AngleControlOnButRateSegmentUsesTheSchedule) {
    // Angle mode enabled, but this instant's profile segment is NULL_RATE.
    EXPECT_EQ(select(true, true, true, true, false), Mode::GainSchedule);
}

TEST(RollControlMode, DivergedEkfFallsToPureGyroAsBefore) {
    // Existing #265 behaviour, unchanged by this fix.
    EXPECT_EQ(select(false, true, false, true, false), Mode::PureGyro);
    EXPECT_EQ(select(true, true, false, true, true), Mode::PureGyro);
}

TEST(RollControlMode, GainScheduleOffFallsToPureGyro) {
    EXPECT_EQ(select(false, true, true, false, false), Mode::PureGyro);
}

TEST(RollControlMode, AngleModeDoesNotRequireTheGainSchedule) {
    // The two are independent knobs; angle mode must not depend on gain
    // scheduling being enabled.
    EXPECT_EQ(select(true, true, true, false, true), Mode::Angle);
}

TEST(RollControlMode, PureGyroIsTheOnlyModeWithNoEkfDependency) {
    // Exhaustive over the EKF terms: every combination that is not
    // (initialised AND healthy) must land on the no-EKF path, whatever the
    // other knobs say.
    for (int angle = 0; angle < 2; ++angle) {
        for (int gs = 0; gs < 2; ++gs) {
            for (int seg = 0; seg < 2; ++seg) {
                EXPECT_EQ(select(angle, false, false, gs, seg), Mode::PureGyro);
                EXPECT_EQ(select(angle, false, true,  gs, seg), Mode::PureGyro);
                EXPECT_EQ(select(angle, true,  false, gs, seg), Mode::PureGyro);
            }
        }
    }
}
