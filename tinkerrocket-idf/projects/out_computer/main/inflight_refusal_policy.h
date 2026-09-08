#pragma once

#include <stdint.h>

// #1162: does the OC's belief that the rocket is INFLIGHT still bind the two
// actions that must not happen mid-flight — the BLE cmd-8 power-off (#834
// item 2) and the OC self-OTA (#1106)?
//
// latest_rocket_state never ages: it is the state byte of the last
// NonSensorData frame, and nothing clears it when the FC stops sending. Both
// gates therefore need a way out for an FC that died mid-flight, or the OC
// would refuse every power-off for the rest of the boot with no exit but a
// battery pull. Until #1162 that way out was the 3 s telemetry-freshness term
// (config::FC_FRAME_STALE_MS): no frame for 3 s and the gate was OPEN. But
// 3 s is a display constant, not a death test — an FC panic/WDT reboot spends
// ~10 s in setup_fc before its first NonSensorData, and the I2S RX-break retry
// (#834 items 6/7) and a paused ingest (#917) are longer still. Every one of
// those left the OC KNOWING the rocket was INFLIGHT while a power-off cut the
// FC's rail on V7/V8 (no P4_EN_HOLD latch) — a ballistic recovery — or, on
// V9/V10, reset the OC, killed the downlink and finalized the flight log the
// #846 re-seed would then skip.
//
// The escape hatch is bounded by FLIGHT TIME instead. The FC forces
// INFLIGHT -> LANDED at launch + MAX_FLIGHT_TIME_MS (flight_computer
// main.cpp), a restored flight back-dates its launch time from the snapshot,
// and the OC can only observe INFLIGHT after launch. So once the OC's OWN
// first sighting of INFLIGHT is MAX_FLIGHT_TIME old, no live FC could still be
// flying this flight, and a dead one has no deployment left to protect. It is
// the same bound kSnapshotServeTtlMs applies to a cached snapshot, for the
// same reason.
//
// A live FC is never overridden by the bound: while frames are fresh and say
// INFLIGHT the refusal holds regardless of age, exactly as before #1162. The
// bound only decides what a SILENT FC means.
//
// Pure so the decision table is host-testable (rail_restore_policy.h
// pattern). Inputs:
//   state_inflight  — latest_rocket_state == INFLIGHT, the latched belief.
//   fc_frame_fresh  — a NonSensorData frame arrived within FC_FRAME_STALE_MS:
//                     the FC is alive and state_inflight is its current word.
//   inflight_age_ms — millis() since the OC first saw INFLIGHT for this
//                     flight: the non-INFLIGHT -> INFLIGHT edge in
//                     processFrame, or the first frame after an OC reboot.
//                     Both are at or after launch, so the bound is
//                     conservative in the direction of refusing.

namespace InflightRefusalPolicy {

// = flight_computer MAX_FLIGHT_TIME_MS, the FC's own INFLIGHT timeout.
inline constexpr uint32_t kMaxFlightTimeMs = 600000;

inline bool refuse(bool state_inflight, bool fc_frame_fresh, uint32_t inflight_age_ms)
{
    if (!state_inflight) return false;   // pad, LANDED, booting: nothing to protect
    if (fc_frame_fresh)  return true;    // a live FC says INFLIGHT: no timeout overrides it
    return inflight_age_ms < kMaxFlightTimeMs;   // silent FC: hold until no flight could remain
}

// How much of the silent-FC hold is left, for the refusal log line.
inline uint32_t holdRemainingMs(uint32_t inflight_age_ms)
{
    return inflight_age_ms < kMaxFlightTimeMs ? kMaxFlightTimeMs - inflight_age_ms : 0;
}

// #1147 items 8 and 9: WHICH commands the INFLIGHT rule covers.
//
// The #383 rule was written into processUplinkCommand's refusal list and never
// applied to the BLE half, so the same command is refused over LoRa and
// accepted over Bluetooth. Shared here so the two dispatches cannot drift
// again — the list was already extended once (#1130 added 5 and 6) and only
// the LoRa site moved.
//
//   1  camera on/off        — the FC cannot receive it until after landing, and
//                             a CAMERA_START delivered on the first post-landing
//                             poll powers the camera on with no stop armed
//   5  sim config           — #1130
//   6  sim start            — #1130: resets the flight state the #317 terminal
//                             LANDED lockout exists to protect
//   23 logging toggle       — #1159
//   28 guidance target      — DELIBERATELY EXEMPT on the BLE side, see below
//   35 pyro continuity test — #1147 item 9
//   36 pyro test fire       — #1147 item 9
//
// 28 is on the LoRa list but is deliberately NOT refused on the BLE path: its
// rejection is echoed to the app via guid_target, which is the feedback the
// LoRa path cannot give. Callers gate 28 themselves; this predicate reports
// membership of the rule, and the BLE cmd-28 site documents its exemption.
inline bool refusedInflight(uint8_t cmd)
{
    return cmd == 1 || cmd == 5 || cmd == 6 || cmd == 23 ||
           cmd == 28 || cmd == 35 || cmd == 36;
}

}  // namespace InflightRefusalPolicy
