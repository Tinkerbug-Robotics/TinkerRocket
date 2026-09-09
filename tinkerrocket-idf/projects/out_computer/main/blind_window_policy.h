#pragma once
// ==========================================================================
// #1271 / #917 — what a phone-IO blind window actually cost, as a decision
// table, so the rule that decides a PERSISTENT operator-facing warning is a
// test rather than an inline conditional.
//
// A BLE file-list, delete or download pauses I2S ingest on the OC
// (`i2s_ingest_paused`), and `endPhoneIO()` then discards the backlog
// outright (`rx_tail = rx_head`). So frames the FC sent during the pause are
// genuinely gone. The question this header answers is whether that LOST A
// FLIGHT or merely clipped the front of one, because only the first deserves
// a warning the operator has to acknowledge.
//
// THE THING THAT MAKES A DURATION FLOOR NECESSARY. The OC starts a flight log
// from an EDGE in loop_oc:
//
//     if (ns_launch && !prev_ns_launch && !oc_landed_lockout) { ... startLogging(); }
//
// and loop_oc is blocked for the whole pause. So the launch edge is DELAYED,
// not missed: it fires on the first resumed frame and logging starts then.
// A short pause therefore yields a log that EXISTS and begins `blind_ms`
// late. Only a pause long enough to swallow the ascent produces the "there is
// no log" outcome. #917's original message claimed the latter unconditionally,
// which cost nothing while it was one ESP_LOGE nobody reads — and would be a
// false alarm as a latched warning, because `beginPhoneIO()` also brackets a
// cmd-2 file list (no INFLIGHT gate at all) and a cmd-3 delete (allowed in
// PRELAUNCH, which is entered automatically on a GNSS fix, not by arming).
//
// WHY SIMS ARE EXCLUDED FROM THE LATCH. The FC sets NSF_LAUNCH for a simulated
// flight too (`kinematics.launch_flag || rocket_state == INFLIGHT`), so a sim
// is indistinguishable here — except that NSF_SIM_ACTIVE rides the same flags
// byte and is free to read. Without this rule the bench procedure for testing
// this very feature (start a sim during a download) would leave a permanent
// acknowledgement-required latch on the unit every single run.
// ==========================================================================
#include <stdint.h>

namespace BlindWindowPolicy
{

// Below this, a launch across the window clipped the front of a log that
// still exists; at or above it, treat the flight as lost. Comfortably longer
// than a file list or a delete's block erases, and short enough that anything
// swallowing real boost data still trips.
static constexpr uint32_t kLatchFloorMs = 1000;

enum class Verdict : uint8_t
{
    Quiet = 0,      // nothing to say
    NoLaunchLong,   // a long pause, but the vehicle did not launch across it
    LaunchClipped,  // launched across a SHORT window: the log exists, front clipped
    LaunchSim,      // a simulated launch across the window: say it, never latch
    LaunchLost,     // launched across a long window: the flight is gone. LATCH.
};

// `pre_launch_latched` is the state captured when the pause began: if the
// vehicle was ALREADY flying, the window did not lose us a launch we would
// otherwise have caught, so there is nothing new to report.
inline Verdict classify(bool now_launched,
                        bool pre_launch_latched,
                        bool sim_active,
                        uint32_t blind_ms,
                        uint32_t floor_ms = kLatchFloorMs)
{
    if (now_launched && !pre_launch_latched)
    {
        if (sim_active)             return Verdict::LaunchSim;
        if (blind_ms < floor_ms)    return Verdict::LaunchClipped;
        return Verdict::LaunchLost;
    }
    // No new launch across the window. The pre-existing #917 behaviour: only
    // remark on it when the window was long enough to be worth noting.
    if (blind_ms > 5000UL) return Verdict::NoLaunchLong;
    return Verdict::Quiet;
}

// The single question the persistence and the wire flag both hang off.
inline bool shouldLatch(Verdict v) { return v == Verdict::LaunchLost; }

}   // namespace BlindWindowPolicy
