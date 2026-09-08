// #1137 item 5 — the #363 launch failsafe after the flight is over.
//
// The failsafe force-clears a ground/servo/replay test (and, since #1121, an
// OTA session) when launch_flag latches, because those modes sit in the `else`
// of the test-mode chain and would otherwise suppress the state machine and
// pyro servicing for the whole flight.
//
// But launch_flag is LATCHED and never cleared for the rest of the session, so
// after touchdown it is still true and the failsafe kept firing — cancelling
// any test the operator started, ~5 ms in, with a log line claiming a launch.
//
// The OTA half is worse and recent: #1121 (2026-09-07) put fc_ota_data_mode on
// the same failsafe. OTA_BEGIN documents LANDED as admissible and neither app
// gates the flight-computer update button, so from that commit every FC update
// pushed to a landed board — including a bench sim flown out to LANDED, which
// is exactly how one would test it — was aborted within ~5 ms of BEGIN.

#include <gtest/gtest.h>

#include "test_mode_gate_policy.h"

using TestModeGatePolicy::launchFailsafeShouldCancel;
using TestModeGatePolicy::testCommandRefused;

// ---- the failsafe -------------------------------------------------------

TEST(LaunchFailsafe, CancelsInFlightExactlyAsBefore) {
    // The regression fence: every in-flight row must be bit-identical to the
    // behaviour before the qualifier was added.
    EXPECT_TRUE(launchFailsafeShouldCancel(/*launch=*/true, /*active=*/true,
                                           /*landed=*/false));
    EXPECT_FALSE(launchFailsafeShouldCancel(true, false, false));
    EXPECT_FALSE(launchFailsafeShouldCancel(false, true, false));
    EXPECT_FALSE(launchFailsafeShouldCancel(false, false, false));
}

TEST(LaunchFailsafe, StopsCancellingOnceTheFlightIsOver) {
    // THE BUG: launch_flag is still latched after touchdown.
    EXPECT_FALSE(launchFailsafeShouldCancel(true, true, /*landed=*/true));
}

TEST(LaunchFailsafe, TheOtaRegressionIsTheSameRow) {
    // #1121 passes fc_ota_data_mode as `mode_active`. A landed board must be
    // able to take a firmware update.
    const bool ota_session_open = true;
    EXPECT_FALSE(launchFailsafeShouldCancel(true, ota_session_open, true))
        << "an FC update on a landed board must not be aborted by the failsafe";
    // ...but a launch DURING an update still aborts it (#1121's actual purpose).
    EXPECT_TRUE(launchFailsafeShouldCancel(true, ota_session_open, false));
}

TEST(LaunchFailsafe, ABootWithNoFlightNeverCancels) {
    // post_flight_lockout is statically false at boot, so a reboot-recovery
    // boot mid-flight still gets the failsafe — pin it, because that is the
    // case the qualifier must not break.
    EXPECT_TRUE(launchFailsafeShouldCancel(true, true, false));
}

// ---- the command gates --------------------------------------------------

TEST(TestCommandGate, RefusesInTheLockoutStatesAsBefore) {
    EXPECT_TRUE(testCommandRefused(/*lockout_state=*/true, /*landed=*/false));
}

TEST(TestCommandGate, NowAlsoRefusesAfterLanding) {
    EXPECT_TRUE(testCommandRefused(false, /*landed=*/true));
}

TEST(TestCommandGate, AllowsOnThePad) {
    EXPECT_FALSE(testCommandRefused(false, false));
}

TEST(TestCommandGate, BothTermsTogetherStillRefuse) {
    EXPECT_TRUE(testCommandRefused(true, true));
}

TEST(TestCommandGate, MatchesThePyroFireTestPrecedent) {
    // PYRO_FIRE_TEST has read `isCommandLockoutState(rocket_state) ||
    // post_flight_lockout` since #317. The three test commands now resolve
    // identically for every input, which is the point of the shared predicate.
    for (int lock = 0; lock < 2; ++lock) {
        for (int landed = 0; landed < 2; ++landed) {
            const bool pyro_fire_would_refuse = lock || landed;
            EXPECT_EQ(testCommandRefused(lock, landed), pyro_fire_would_refuse);
        }
    }
}
