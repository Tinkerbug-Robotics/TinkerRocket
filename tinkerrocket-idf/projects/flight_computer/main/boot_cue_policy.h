#pragma once

#include <stdint.h>

// #1188 (#1176 decision 4): which local LED cue this boot shows.
//
// The owner's ruling is that a board which came up believing a flight was in
// progress must be unmistakable at the prep table, because with a live flight
// token the out computer raises this rail the moment the pack is connected
// instead of waiting for the app's power button. The restore block already
// covers the half where the flight computer actually RESTORES a flight: both
// LEDs held on, a state no other boot produces.
//
// The other half was missing. The OC raises the rail from a token, the FC asks
// for the snapshot and declines it — a simulated frame, a LANDED clear, no
// answer at all — and then boots exactly as if the operator had pressed the
// button: red on, blue off, the ordinary heartbeat blip. The board is ON,
// uncommanded, and the only evidence is a log line on a serial port nobody has
// attached. The OC has no LED of its own, so the FC has to show it; the OC now
// says on every status poll whether this session started from a token
// (OUT_STATUS_TOKEN_POWERED_BIT, RocketComputerTypes.h).
//
// Three cues, in priority order:
//   RestoredFlight  — a restored flight is in progress: both LEDs solid. The
//                     arming interlock decides whether a channel may leave
//                     Idle; the cue only says "do not treat me as idle".
//   SelfPoweredIdle — the OC says this session self-powered, and the FC is on
//                     the ground with no restored flight in progress: red on,
//                     blue at a slow 50 % blink, which nothing else produces
//                     (the heartbeat is a 40 ms blip once a second). Shown on a
//                     token boot the FC declined, AND after a restored flight
//                     the interlock refuted to LANDED — which gives the
//                     operator the live read they actually want: solid means
//                     "still believes it is flying", blink means "decided it is
//                     on the ground; power-cycle me".
//   None            — everything else, including any flight launched in this
//                     session: the launch is its own evidence, and the
//                     heartbeat owns the blue LED in the air.
//
// Stateless on purpose. The cue is a function of the board's CURRENT belief,
// so it can never be left showing a fact that has stopped being true, and the
// same boot can move solid -> blink -> off as that belief changes. Pure so the
// table is host-testable, in the house style of pwr_hold_policy.h.

namespace BootCuePolicy {

enum class Cue : uint8_t {
    None,             // normal indication: the heartbeat owns the blue LED
    RestoredFlight,   // both LEDs solid
    SelfPoweredIdle,  // red solid, blue slow blink
};

// restored_flight_in_progress — recovery_gate_active && rocket_state == INFLIGHT
// oc_self_powered             — the OC has reported OUT_STATUS_TOKEN_POWERED_BIT
//                               at least once this session
// on_ground                   — READY, PRELAUNCH or LANDED: not in the air and
//                               not still initialising
inline Cue cueFor(bool restored_flight_in_progress, bool oc_self_powered,
                  bool on_ground)
{
    if (restored_flight_in_progress) return Cue::RestoredFlight;
    if (oc_self_powered && on_ground) return Cue::SelfPoweredIdle;
    return Cue::None;
}

}  // namespace BootCuePolicy
