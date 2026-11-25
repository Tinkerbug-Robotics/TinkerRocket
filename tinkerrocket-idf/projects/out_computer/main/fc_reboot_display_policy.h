#pragma once

#include "RocketComputerTypes.h"   // RocketState

// What the direct BLE link shows while the FC reboots and the OC stays up.
//
// latest_rocket_state is the last NonSensorData's state, and it is kept on
// purpose through an FC-only reset: the snapshot the OC hands a booting FC,
// the in-flight refusals, the LoRa hop and beacon, the landed lockout and the
// flight log all need the rocket's phase, not the FC's. But the apps were shown
// it too. After an FC-only reset on the pad they read a green "PRELAUNCH" (or
// an armed READY) over a boot line saying "Starting sensors", "Rescue mode" or
// "no flight loop", with a red "STALE DATA — link degraded" banner, when
// nothing was wrong with the link (Beetle, 2026-10-26).
//
// So while the FC reports boot steps again (the same window in which the OC
// publishes the "bs" keys), a GROUND state is published as INITIALIZATION
// and the data status as SYNCING: exactly what the first boot after power-on
// shows. Only the published words change; latest_rocket_state does not.
//
// INFLIGHT and LANDED are left alone. They describe the rocket, not the FC: an
// FC that reboots in flight is still flying (and resumes from the snapshot),
// and after a landing the apps' landing and recovery views must not reset.
namespace FcRebootDisplay {

// `fc_booting`: a boot-status frame has arrived since the last NonSensorData.
inline bool groundReboot(RocketState stored, bool fc_booting)
{
    return fc_booting && stored != INFLIGHT && stored != LANDED;
}

inline RocketState publishedState(RocketState stored, bool fc_booting)
{
    return groundReboot(stored, fc_booting) ? INITIALIZATION : stored;
}

}  // namespace FcRebootDisplay
