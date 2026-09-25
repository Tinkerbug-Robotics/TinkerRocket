#pragma once

#include <stdint.h>

// When the OC gives its half of the Beetle's two-processor pyro arm.
//
// On the Beetle (TR_BOARD_M1) the squib return FET U9 closes only while BOTH
// FC_ARM (FC GPIO44) and OC_ARM_EN (OC GPIO11, config::ARM_CONSENT_PIN) are
// high: Q12 and Q14 in series pull Q13's base, and Q13 drives U9's gate from
// the pack. With U9 open the return reaches ground only through R73 (2.2 k),
// the continuity-sense path: ~3.6 mA into an e-match, two orders of magnitude
// short of firing it, yet enough to light a bench LED. That is how the bench
// found this (2026-09-24). #1168 drove consent LOW at boot and nothing ever
// raised it, so no channel could fire in a ground test OR in flight, while
// continuity read true and an LED "fired" (design review 2026-09-21, findings
// 50 and 53).
//
// The FC raises FC_ARM only around a pulse, PYRO_ARM_SETTLE_MS (10) plus
// PYRO_FIRE_DURATION_MS (200), from either the in-flight deployment state
// machine or the PYRO_FIRE_TEST ground test. Consent must already be up when
// it does. There are exactly two reasons for it to be:
//
//   FireTest — the OC is serving a PYRO_FIRE_TEST to the FC. Every staging of
//     the command (CMD_REPEAT_LIMIT deliveries, plus any re-stage after a
//     dropped stage) opens or extends a window to kFireTestHoldMs past that
//     staging. It is keyed on the serve, not on the BLE/LoRa enqueue, because
//     the queue can hold the command behind the one being served for a second
//     or more. The FC reads a staged frame on its NEXT 250 ms poll and then runs
//     ARM -> settle -> FIRE synchronously: about 0.5 s from staging to the end
//     of the pulse at nominal timing, so 2 s is roughly 4x. Both entry points
//     refuse a fire test while the FC rail is off or the rocket is INFLIGHT,
//     and the FC refuses one after landing, so this window never opens in
//     flight.
//
//   Flight — the OC believes a REAL flight may be in progress: the #1162
//     InflightRefusalPolicy hold (the FC's latest word is INFLIGHT, and if the
//     FC has gone silent, less than MAX_FLIGHT_TIME has passed since the OC
//     first saw INFLIGHT). It is the same predicate behind the cmd-8 power-off
//     refusal and the self-OTA veto, so the three can never disagree about
//     whether the rocket is flying.
//
//     Held THROUGH a silent FC on purpose (owner decision, 2026-09-24). The
//     README's original contract dropped consent on FC heartbeat loss, which
//     would turn an FC->OC link dropout, or an FC reboot near apogee, into a
//     lost deployment: the FC's fire state machine never retries, and a channel
//     whose 200 ms passed with U9 open is marked Done. The FC does not poll the
//     OC at all while INFLIGHT, so it cannot receive a fire test then; this
//     hold is the ONLY way consent is up in flight.
//
//     A simulated flight does not count. The FC dry-fires a sim (ARM and FIRE
//     stay low, sim_flight_policy.h), so consent would buy nothing, and
//     holding it would leave a charge on the bench behind one processor for the
//     whole run. The FC latches its dry-fire decision for the flight (#1104)
//     because the sim's give-up tick drops isSimActive() while the FC is still
//     INFLIGHT. SimFlightLatch mirrors that latch from NSF_SIM_ACTIVE.
//
// Everywhere else, consent is LOW. That covers the pad, where PRELAUNCH can
// last hours with people around the rocket, and everything after LANDED.
// There, neither the FC's boot-time GPIO44 pull-up nor a stuck arm line can
// close U9 on its own.
//
// Reboots. Nothing fires through a boot, and that is deliberate (owner,
// 2026-09-25):
//
//   - An OC reset drops consent. GPIO11 floats from reset (Q14's built-in 47 k
//     holds it off), setup_oc drives it low, and the Flight reason returns only
//     once the first NonSensorData frame after the reboot has been processed, a
//     second or more into the new session. A pulse the FC fires inside that
//     window does not reach the squib. Carrying consent across the reset (RTC
//     memory, the flight token) was considered and declined; do not add it
//     (design review finding 53).
//
//   - An FC reset in flight keeps consent (the silent-FC hold above), and that
//     cannot fire through the FC's boot either. The four FIRE pads
//     (GPIO33/34/35/38) carry no pull at or after reset (ESP32-S3 datasheet
//     Table 2-1: IE only), so each Q3-Q6 driver's built-in 47 k holds its
//     channel off until initPyroPins() drives the pad low. GPIO44's reset
//     pull-up can close U9 for that stretch, but with no channel switched on,
//     no squib current flows.
//
// Pure so the decision table is host-testable (the inflight_refusal_policy.h
// pattern). main.cpp owns the pin: serviceArmConsent() evaluates this every
// loop_oc pass, and once more the moment a fire test is staged.

namespace ArmConsentPolicy {

// How long one staging of a PYRO_FIRE_TEST keeps consent up (see above).
inline constexpr uint32_t kFireTestHoldMs = 2000;

// Why consent is up. None = OC_ARM_EN low.
enum class Reason : uint8_t
{
    None     = 0,
    FireTest = 1,
    Flight   = 2,
};

// ---- FireTest window -------------------------------------------------------

struct FireTestWindow
{
    bool     open     = false;
    uint32_t until_ms = 0;   // millis() deadline, meaningful only while open
};

// A PYRO_FIRE_TEST was staged for the FC at now_ms: open the window, or push
// its deadline out if it is already open.
inline void onFireTestStaged(FireTestWindow& w, uint32_t now_ms)
{
    w.open     = true;
    w.until_ms = now_ms + kFireTestHoldMs;
}

// Is the window still open at now_ms? Closes it once the deadline has passed.
// Closing is what makes the check wrap-safe: a deadline compared by signed
// difference reads as "open" again 2^31 ms (~24.8 days) after it expired if
// nothing ever clears the flag, and an OC can sit powered on a bench that long.
inline bool fireTestWindowOpen(FireTestWindow& w, uint32_t now_ms)
{
    if (w.open && (int32_t)(now_ms - w.until_ms) >= 0) w.open = false;
    return w.open;
}

// ---- Simulated-flight latch --------------------------------------------------

struct SimFlightLatch
{
    bool prev_inflight = false;
    bool sim           = false;
};

// Feed every NonSensorData frame's state (== INFLIGHT) and NSF_SIM_ACTIVE bit,
// in arrival order. Returns whether the current INFLIGHT phase is a
// simulation, and false whenever the state is not INFLIGHT. A fresh INFLIGHT
// phase is decided by the flag on its first frame, which is also where an OC
// that rebooted mid-flight joins. Within the phase the flag can only latch it
// on, never off: the sim's give-up tick clears NSF_SIM_ACTIVE while the FC
// still says INFLIGHT, and the FC is still dry-firing then.
inline bool step(SimFlightLatch& s, bool state_inflight, bool sim_active)
{
    if (!state_inflight)       s.sim = false;
    else if (!s.prev_inflight) s.sim = sim_active;
    else if (sim_active)       s.sim = true;
    s.prev_inflight = state_inflight;
    return s.sim;
}

// ---- The decision ------------------------------------------------------------

// inflight_hold   — InflightRefusalPolicy::refuse(), via inflightHold().
// inflight_is_sim — step() above, for the frame that set the state.
// fire_test_open  — fireTestWindowOpen().
inline Reason decide(bool fire_test_open, bool inflight_hold, bool inflight_is_sim)
{
    if (inflight_hold && !inflight_is_sim) return Reason::Flight;
    if (fire_test_open)                    return Reason::FireTest;
    return Reason::None;
}

inline bool pinHigh(Reason r)
{
    return r != Reason::None;
}

}  // namespace ArmConsentPolicy
