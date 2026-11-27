#pragma once

#include <stdint.h>

// #1770: when the FC relaxes the fin servos, with servo control on or off.
//
// With servo control off (NVS servo_en false) the servos are still live on
// the V10 board: setup_fc runs TR_ServoControl::begin() whenever the pins are
// valid, and the servo rail is up from boot. The servo test drives them
// whatever servo_en says. But every path that brought them back was gated on
// servo_enabled: the neutral settle's service, the READY and PRELAUNCH relax,
// the #363 launch failsafe's stow and LANDED's relax. So a SERVO_TEST_STOP
// (the app sends one whenever its Servo Test screen closes) started the
// anti-backlash settle -- all four fins 6 deg past neutral -- and nothing ever
// finished it: the fins flew deflected, powered, until a reboot.
//
// main.cpp now services the settle on every pass, and settles, stows and
// relaxes by these rules.
//
// Pure so it is host-testable (tests_cpp/test_servo_pad_policy.cpp).
namespace ServoPadPolicy {

// READY and PRELAUNCH, each pass: relax the servos (cut the pulse train)?
// With servo control on, as before: the pad relax (SERVO_RELAX_ON_PAD). With
// it off, always: nothing holds the fins then, and begin() leaves them
// relaxed at boot. Either way not while the boot sweep or a neutral settle is
// driving them, nor inside the wake window that keeps a just-stowed or
// just-trimmed neutral visible (signed compare: wrap-safe).
inline bool relaxOnPad(bool servo_enabled, bool relax_on_pad, bool wiggling, bool settling,
                       uint32_t now_ms, uint32_t wake_until_ms)
{
    return (!servo_enabled || relax_on_pad) && !wiggling && !settling &&
           (int32_t)(now_ms - wake_until_ms) >= 0;
}

// SERVO_TEST_STOP: begin the anti-backlash settle to neutral (and arm the wake
// window)? While a test is running, or on the pad (READY or PRELAUNCH): there
// a stop always stows, as before, which also brings back fins a reboot
// mid-test left relaxed at the test's angles. Off the pad a stop with no test
// running has nothing to stow, and drives nothing: LANDED never relaxes the
// servos again, so a stray stop there would hold them powered until a reboot.
inline bool stopSettles(bool servo_test_active, bool on_pad)
{
    return servo_test_active || on_pad;
}

// A servo test or replay cancelled under the operator -- the #363 launch
// failsafe, the start of a mag-cal session: stow the fins? With servo control
// on, always, as before. With it off, only when the cancelled mode was
// driving them: a ground test never does then, and the fins are already
// relaxed where boot left them.
inline bool stowOnCancel(bool servo_enabled, bool servo_test_active, bool servo_replay_active)
{
    return servo_enabled || servo_test_active || servo_replay_active;
}

// LANDED's one-shot: relax (true) or hold the calibrated neutral (false).
// Relax with servo control off, with the pad relax, or once the eject has
// cut the fins (#1768); otherwise hold, as before.
inline bool relaxAtLanding(bool servo_enabled, bool relax_on_pad, bool fins_cut)
{
    return !servo_enabled || relax_on_pad || fins_cut;
}

}  // namespace ServoPadPolicy
