#pragma once

#include <stdint.h>

// #1768: the fins' power once the eject has occurred.
//
// The FC has no descent state: INFLIGHT runs from launch to LANDED, and the
// servo block in it had no deployment term. The control-authority speed gate
// latches open for the flight (with the default gate of 0, on the first
// tick), so a roll law kept driving the fins, and the servo rail stayed up,
// from the eject to LANDED:
//   - the gain schedule at its 3x low-speed cap under a drogue (15-28 m/s) or
//     a main (5-8 m/s), where a section hanging tail-first reverses the fins'
//     torque and the rate loop spins it up instead of damping it;
//   - angle mode toward the last waypoint, against a roll a hanging airframe
//     does not define.
// On a nominal flight the servos disconnect at the drogue's eject, so the
// rail and the pulse lines were left live on an open harness for the whole
// descent.
//
// Now roll control and guidance run as before up to the eject -- through
// apogee and any deployment delay -- and the eject ends them for the flight:
//   - CutBegin, the first pass the eject is seen: the pulse train is cut
//     (TR_ServoControl::idle: every signal line held low) and the outputs
//     locked (lockOutputs: no command drives them again until the rail is
//     back up), the rate PID reset;
//   - Cutting, until rail_delay_ms after it: still cut, the rail still up.
//     The LEDC latches a duty change at the next PWM period, so the pulse
//     under way when the cut is written runs to its end: the rail waits out
//     that pulse, and no line is high when it goes;
//   - RailOff, the first pass rail_delay_ms after the cut: the servo rail
//     switched off (SERVO_ACT, where it switches the supply);
//   - Cut, every pass after it, for the rest of the flight: nothing to do,
//     the locked outputs write nothing whatever command arrives.
// The eject is main.cpp's: the FC's deployment detection once it has called
// apogee, a deployment charge starting its fire, or a cut restored from the
// snapshot after a reboot.
//
// Pure so it is host-testable (tests_cpp/test_fin_eject_policy.cpp).
namespace FinEjectPolicy {

enum class Fins : uint8_t {
    Powered,    // before the eject: control (or its hold) as before
    CutBegin,   // the eject: pulses cut, outputs locked, PID reset
    Cutting,    // the pulse under way runs out; the rail still up
    RailOff,    // the rail switched off
    Cut,        // the rest of the flight: pulses cut, outputs locked, rail off
};

struct State {
    bool     cut      = false;   // the eject has been acted on
    bool     rail_off = false;   // and the rail switched off
    uint32_t since_ms = 0;       // when the pulses were cut
};

// Launch, a sim's reset, a restore: a flight with no eject acted on yet.
inline void reset(State& s) { s = State{}; }

// Once per INFLIGHT pass. `ejected`: the eject has occurred this flight.
// Once cut, cut for the flight.
inline Fins step(State& s, bool ejected, uint32_t now_ms, uint32_t rail_delay_ms)
{
    if (!s.cut)
    {
        if (!ejected) return Fins::Powered;
        s.cut = true;
        s.since_ms = now_ms;
        return Fins::CutBegin;
    }
    if (s.rail_off) return Fins::Cut;
    if ((uint32_t)(now_ms - s.since_ms) < rail_delay_ms) return Fins::Cutting;
    s.rail_off = true;
    return Fins::RailOff;
}

// Whether the eject has cut the fins this flight: no roll law, no guidance.
inline bool cut(const State& s) { return s.cut; }

// The pass's servo and rail commands. Templated so the host test drives the
// real TR_ServoControl through the same lines main.cpp runs; `set_rail(bool)`
// switches the servo rail.
template <class Servo, class Rail>
inline void drive(Fins f, Servo& servo, Rail&& set_rail)
{
    switch (f)
    {
        case Fins::CutBegin:
            servo.idle();               // every signal line low...
            servo.lockOutputs(true);    // ...and held low until the rail is back
            servo.resetPID();
            break;
        case Fins::RailOff:
            set_rail(false);            // the lines low and locked since CutBegin
            break;
        case Fins::Cutting:
        case Fins::Cut:
        case Fins::Powered:
            break;                      // locked: nothing can drive them
    }
}

}  // namespace FinEjectPolicy
