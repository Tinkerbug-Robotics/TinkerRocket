#pragma once

// Are the four servo outputs mapped to real GPIOs?  servoPinsValid() in
// main.cpp is this predicate over config::SERVO_PIN_1..4.
//
// The board headers spell "no pin" two ways. V7/V8/V9 declare the servo pins
// as uint8_t GPIO numbers, where 255 is the unmapped sentinel. M1 (the
// rocket-computer-mini, which has no servos) declares all four as
// `static constexpr int ... = -1`, the spelling it also uses for its other
// absent pins (SERVO_ACT_PIN, PIEZO_PIN).
//
// main.cpp used to test each pin with `!= 255U`. Under the usual arithmetic
// conversions an int -1 is converted to unsigned (0xFFFFFFFF), which is not
// 255, so M1 read as fully mapped: setup_fc ran servo_control.begin() with
// every pin narrowed to uint8_t 255, all four ledc_channel_config() calls
// failed, and the branch that forces servo_enabled off on a board without
// servo pins never ran.
//
// Taking the pin as int covers both spellings: a uint8_t widens to 0..255
// with its value intact, and an int keeps its sign.
//
// Pure so the rule is host-testable, in the style of test_mode_gate_policy.h.
namespace ServoPinPolicy {

// One pin: a GPIO number, not a sentinel (negative, or 255).
constexpr bool pinMapped(int pin)
{
    return pin >= 0 && pin != 255;
}

// All four, as before: servo_control drives the outputs as one set, so a
// single unmapped pin leaves servo control disabled.
constexpr bool allPinsMapped(int pin1, int pin2, int pin3, int pin4)
{
    return pinMapped(pin1) && pinMapped(pin2) &&
           pinMapped(pin3) && pinMapped(pin4);
}

}  // namespace ServoPinPolicy
