#pragma once
// #1104: what the flight computer does when the simulator's active flag
// changes, and when a pyro decision is "dry".  Lives next to
// sim_landed_hold.h because the two are one contract: the hold decides when
// the sim ends, this decides what the FC does about it — and the retired
// single-MCU mini's flight task, which shared this component, needed the same
// rules.
//
// The FC used to act only on the RISING edge of isSimActive(), on the
// assumption that the falling edge always coincides with the FC in LANDED —
// the sim's natural completion, where the FC must HOLD LANDED so the
// post-flight lockout stays validated (#317).  #971 broke that assumption:
// the SIM_LANDED hold now gives up after sim_landed::HOLD_MAX_MS when landing
// detection never fires, and drops the sim to SIM_IDLE with the FC still
// INFLIGHT.  From that tick the dry-fire gate (keyed off isSimActive()) was
// OFF and the I2C command poll (skipped in a non-sim INFLIGHT) stopped: real
// ARM/FIRE outputs on the bench, and no way to reach the FC until the
// 10-minute flight backstop.
//
// Two rules close it, and #1113 adds a third — whether a SIM_STOP has a sim
// flight to act on at all.  All are pure, so
// tests_cpp/test_sim_flight_policy.cpp pins them instead of trusting them.

#include <cstdint>

namespace sim_flight
{

enum class Edge : uint8_t
{
    None,         // no change
    Start,        // sim went active: a fresh run — reset the flight state and
                  // latch "this flight is simulated"
    EndedLanded,  // sim went idle with the FC in LANDED: the flown-out sim —
                  // hold LANDED so the post-flight lockout stays validated
    EndedEarly,   // sim went idle with the FC NOT in LANDED (the #971 give-up,
                  // or any future early exit): treat exactly like a user Stop —
                  // safe the pyros and reset the flight state to READY
};

/// Pure decision on the isSimActive() edge.  Keys on the FC's OWN state, not
/// on the sim's stated reason, because the FC's state is what decides whether
/// leaving things alone is safe.  `fc_landed` is `rocket_state == LANDED`,
/// passed in so this header stays free of the firmware type.
constexpr Edge classify(bool prev_active, bool curr_active, bool fc_landed)
{
    if (!prev_active && curr_active) return Edge::Start;
    if (prev_active && !curr_active) return fc_landed ? Edge::EndedLanded
                                                      : Edge::EndedEarly;
    return Edge::None;
}

/// Is the flight being flown right now a simulated one?  True if the sim is
/// active OR this flight was started by the sim (a latch set on the Start
/// edge and cleared only when the flight state is reset).  The pyro dry-fire
/// gate and the snapshot's sim stamp key on THIS rather than on isSimActive()
/// alone: the sim steps its physics inside the FC's IMU read, so on the pass
/// in which it gives up, servicePyroChannels() runs with isSimActive() already
/// false — one tick BEFORE the edge above is handled.  Sampled, that tick
/// would drive real outputs; latched, it cannot.
constexpr bool simulated(bool sim_flight_latched, bool sim_active)
{
    return sim_flight_latched || sim_active;
}

/// #1766: does the FC report NSF_SIM_ACTIVE?  For the whole of a sim's flight
/// state (simulated()), not only while isSimActive(): a flown-out sim ends at
/// LANDED and the FC holds it (#317) until the operator's Stop, the one way
/// to re-arm without a reboot, which the apps offer only on their SIM banner.
/// With the flag down for that hold, a phone reconnect or a switch to the
/// base-station relay lost the banner, and a held sim LANDED looked like a
/// rocket to fly.  The flag drops with the latch, at the reset the Stop
/// (or the next Start) makes; an FC reset clears the latch (RAM), so a real
/// flight's LANDED never carries it (the bench-only TR_BENCH_RESTORE_SIM
/// restore sets it again for a restored sim flight, as it should).  The OC's
/// readers keep their meaning: its landed lockout clears on leaving a LANDED
/// that carried the flag (the sim's); arm consent and the stray-Stop gate read
/// it only in INFLIGHT.  There the flag is now also up on the sim's give-up
/// pass and through a bench-restored sim flight: the stray-Stop gate lets a
/// Stop through then, and arm consent stays low while the FC dry-fires.
constexpr bool reportsSim(bool sim_flight_latched, bool sim_active)
{
    return simulated(sim_flight_latched, sim_active);
}

/// #1113: does a SIM_STOP_CMD have a sim flight to act on?  The Stop handler
/// resets the whole flight state to READY and clears the #317 post-flight
/// lockout — it is the one command that deliberately re-arms — and it has no
/// state gate of its own (LANDED still polls).  So it must run ONLY when this
/// boot's flight state was produced by the sim: the sim is still active (a
/// Stop mid-flight), or it was started this run and has since gone idle (the
/// flown-out sim holding LANDED, which the Stop is how the user re-arms from).
/// With neither, the Stop is a stray — a broadcast uplink meant for the bench
/// rocket, a cmd 7 queued during a real flight and delivered on the first poll
/// after touchdown, an app whose SIM MODE banner never cleared — and honouring
/// it ended a REAL flight's terminal LANDED with the deployment latches
/// cleared and a failed channel's e-match still live.  Same predicate as
/// simulated(): the latch is what tells a flown-out sim from a real landing,
/// because isSimActive() is false for both.
constexpr bool stopApplies(bool sim_flight_latched, bool sim_active)
{
    return simulated(sim_flight_latched, sim_active);
}

/// #1153 item 3: may a SIM_START_CMD start a run?  `command_lockout_state` is
/// isCommandLockoutState(rocket_state) — INFLIGHT or MAG_CALIBRATION — the
/// term every other test-class command (ground test, servo test, pyro test,
/// OTA entry) already refuses on.  The sim start had no state gate at all,
/// and it could reach both states:
///   * INFLIGHT: the only INFLIGHT that polls for commands is a SIM flight
///     (#393), so a Start there is a second Start under a running run.
///     startSim() rewinds the physics to the pad while the FC stays INFLIGHT
///     with launch latched — the vehicle it is flying vanishes from under
///     it.  Stop first; the Stop is the reset.
///   * MAG_CALIBRATION: the operator is tumbling the rocket with the chip's
///     hard-iron OFFSET registers zeroed for sampling.  The Start edge reset
///     the flight state to READY without ending the session, so the sampling
///     feed (gated on the state) stopped, the session could never complete,
///     and nothing restored the priors: every later flight in that boot flew
///     an uncalibrated magnetometer whose readings the EKF's magnitude gate
///     rejected outright — heading aiding silently dead.
/// post_flight_lockout is deliberately NOT a term, unlike
/// TestModeGatePolicy::testCommandRefused: a Start from a flown-out sim's
/// LANDED is the ordinary re-run, and its Start edge is the reset that
/// re-arms it (#317).
constexpr bool startRefused(bool command_lockout_state)
{
    return command_lockout_state;
}

/// #1780: does this edge leave the OC a flight log to end?  The OC opens one
/// on the FC's NSF_LAUNCH rising edge (`launch_reported`: the launch latched,
/// or INFLIGHT) and ends it on END_FLIGHT, which the FC sends from LANDED, or
/// on a user Stop, which the OC acts on itself before passing it to the FC.
/// The give-up (EndedEarly) is neither: the FC drops straight to READY, so
/// the log stayed open, every pad frame after it was written into the sim's
/// flight, and the next real flight was appended to that flight. If the open
/// range then ran into the next stored flight, its pages were dropped.
/// So on that edge, with launch reported, the FC sends END_FLIGHT itself.
/// Without launch reported the OC opened no log; what it holds is the flight
/// it prepares at PRELAUNCH, which the next real PRELAUNCH uses as it is.
constexpr bool endsOcLog(Edge e, bool launch_reported)
{
    return e == Edge::EndedEarly && launch_reported;
}

/// One pass of the END_FLIGHT owed by endsOcLog(): sent, and retried on every
/// pass until its enqueue goes through, as LANDED retries its own (the I2S
/// queue takes it with no wait). `send_end` enqueues it and says whether it
/// went in. The FC drops what is still owed at its next launch: by then the
/// OC is opening that flight's log, which this must not end.
template <typename SendEnd>
inline void serviceOcLogEnd(bool& owed, SendEnd&& send_end)
{
    if (owed && send_end()) owed = false;
}

}  // namespace sim_flight
