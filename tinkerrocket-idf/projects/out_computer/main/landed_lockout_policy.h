#pragma once

#include "RocketComputerTypes.h"   // RocketState
#include "sim_flag_run.h"

// #317: once the vehicle has reported LANDED, the OC opens no new flight-log
// session until it reboots. Post-flight ground handling can re-trip the FC's
// launch detect, and a session opened then is a junk recovered_*.bin full of
// ground data that only closes at power-off.
//
// The gate covers EVERY automatic session open on the OC: the READY->PRELAUNCH
// pre-create in processFrame and the NSF_LAUNCH auto-start in loop_oc. Until
// #1235 item 5 the PRELAUNCH hook had no gate at all — and it is the more
// dangerous of the two, because flightlogBeginFlight() can #315-auto-evict the
// oldest stored flight to make room, so an ungated hook could delete a real
// flight to open a junk one.
//
// WHEN THE LOCKOUT CLEARS. The old rule cleared on ANY departure from LANDED,
// reasoning that the FC only leaves LANDED on a deliberate sim re-arm. That is
// wrong: the FC's post_flight_lockout is plain RAM (flight_computer main.cpp,
// not RTC memory or NVS), so an FC-only reset — possible while the OC holds the
// rail and stays up — walks it back through INITIALIZATION -> READY ->
// PRELAUNCH with its lockout gone, which is precisely the sequence the OC's
// lockout exists to refuse. The departure has to be DEMONSTRABLY the FC's own
// sim reset (resetFlightStateForSim), and the OC can see that two ways:
//
//   (a) a frame that is not LANDED and carries NSF_SIM_ACTIVE: the FC is
//       running a simulation, which only a sim START — a deliberate operator
//       action — brings about. An FC reset never sets the flag on its own.
//   (b) the departure from a LANDED phase that was itself a SIM flight (the
//       flag was set in its LANDED frames, or in the INFLIGHT frames that led
//       to it). Since #1766 the FC keeps the flag up for the whole of a
//       flown-out sim's held LANDED (sim_flight::reportsSim), and drops it
//       with the reset the operator's sim Stop makes, back to READY. Before,
//       the flag was up there for one loop pass at most (#971: the sim ends
//       on the next IMU drain), and the flight's own INFLIGHT frames, which
//       carry the flag throughout, decided it -- they still do, for a frame
//       stream that missed the LANDED ones. Clearing here is what lets a real
//       flight after a bench sim be logged without an OC reboot, which the
//       old rule allowed and this one must not lose.
//
// A real flight's LANDED (no sim flag in any of its frames) is therefore left
// only by an FC reset, and the lockout holds through it: the OC has to reboot
// to fly again, which is what docs/architecture/out-computer.md has always
// said. Both flags ride the same NonSensorData frame as the state byte, so the
// step is fed in processFrame, in arrival order, and needs no cross-task
// sampling to get the edges right.
//
// #1774: "carries NSF_SIM_ACTIVE" in (a) and (b) means the flag on
// SimFlagRun::kFrames frames in a row (sim_flag_run.h). One frame with the flag
// set by a link error the CRC missed would make a real flight's landing a sim's
// landing, and an FC reset after it would then clear the lockout. A real sim
// carries the flag from its Start for seconds before PRELAUNCH, so (a) and (b)
// clear as before.
namespace LandedLockout {

struct State
{
    bool        locked         = false;
    bool        landed_was_sim = false;   // NSF_SIM_ACTIVE seen in the current LANDED phase or its flight
    bool        flight_was_sim = false;   // NSF_SIM_ACTIVE seen in the current INFLIGHT phase
    RocketState prev           = INITIALIZATION;
    SimFlagRun::Run flag;                 // #1774: the flag's run of frames
};

// Feed every NonSensorData frame's rocket_state and its NSF_SIM_ACTIVE bit, in
// arrival order, once each. Returns the new `locked`. The flag counts only once
// SimFlagRun::kFrames frames in a row have carried it.
inline bool step(State& s, RocketState state, bool sim_flag_bit)
{
    const bool sim_active = SimFlagRun::step(s.flag, sim_flag_bit);
    if (state == INFLIGHT)
    {
        if (s.prev != INFLIGHT) s.flight_was_sim = false; // a fresh flight
        if (sim_active) s.flight_was_sim = true;
    }
    else if (state != LANDED)
    {
        s.flight_was_sim = false;                         // the pad: no flight to judge
    }
    if (state == LANDED)
    {
        if (s.prev != LANDED) s.landed_was_sim = s.flight_was_sim;   // a fresh LANDED phase
        if (sim_active) s.landed_was_sim = true;
        s.locked = true;
    }
    else if (s.locked)
    {
        if (sim_active)                                   s.locked = false;  // (a)
        else if (s.prev == LANDED && s.landed_was_sim)    s.locked = false;  // (b)
        // else: a real flight's LANDED, left without a sim — an FC reset. Hold.
    }
    s.prev = state;
    return s.locked;
}

}  // namespace LandedLockout
