#pragma once

// #1137 item 5: when does the #363 launch failsafe apply, and when is a test
// command refused?
//
// The failsafe exists because the state machine and servicePyroChannels live in
// the `else` of the test-mode chain: a ground/servo/replay test left active at
// launch would suppress PRELAUNCH->INFLIGHT and pyro servicing for the whole
// flight. kinematicChecks() still latches launch_flag while a test runs, so the
// failsafe force-clears the test and lets the flight logic take over.
//
// But launch_flag is LATCHED and never cleared for the rest of the session, so
// after touchdown it is still true. Every subsequent pass therefore cancels any
// test the operator starts, ~5 ms after it begins, with a log line that says a
// launch was detected. That is the whole of item 5.
//
// It became sharper on 2026-09-07: #1121 added the OTA session to the same
// failsafe. OTA_BEGIN documents READY/PRELAUNCH/LANDED as all admissible and
// neither app state-gates the flight-computer update button, so from that
// commit every FC firmware update pushed to a board that has landed — a real
// flight, or a bench sim flown out to LANDED, which is exactly how someone
// would test one — was aborted and the I2S link reverted within ~5 ms of BEGIN.
//
// post_flight_lockout is the right qualifier rather than `rocket_state !=
// LANDED`. It is written in exactly two places — set on the INFLIGHT->LANDED
// edge, cleared only by resetFlightStateForSim — so it is provably false for
// boot, pad, ascent and descent, which keeps the in-flight behaviour of the
// failsafe bit-identical. It also covers the case where post_flight_lockout
// itself forces the state back to LANDED.
//
// Pure so the table is host-testable, in the style of inflight_refusal_policy.h.
namespace TestModeGatePolicy {

// Should the #363 failsafe cancel an active mode this pass?
//
// `mode_active` is (ground_test_active || servo_test_active ||
// servo_replay_active) for the test half, or fc_ota_data_mode for the OTA half.
inline bool launchFailsafeShouldCancel(bool launch_flag,
                                       bool mode_active,
                                       bool post_flight_lockout)
{
    return launch_flag && mode_active && !post_flight_lockout;
}

// Should a test command be refused outright?
//
// `command_lockout_state` is isCommandLockoutState(rocket_state) — true for
// INFLIGHT and MAG_CALIBRATION. The lockout term is the #1137 item 5 addition,
// and mirrors what PYRO_FIRE_TEST already does; both apps already gate these
// commands on on-pad state, so this makes the firmware agree with the UI
// rather than relying on the UI to be the gate.
//
// Deliberately NOT applied to OTA_BEGIN_PENDING (LANDED is admissible by
// design and is how a post-flight update is done) or to PYRO_CONT_TEST (a
// post-flight continuity check is a working, deliberately offered feature and
// both apps show the button in LANDED).
inline bool testCommandRefused(bool command_lockout_state,
                               bool post_flight_lockout)
{
    return command_lockout_state || post_flight_lockout;
}

}  // namespace TestModeGatePolicy
