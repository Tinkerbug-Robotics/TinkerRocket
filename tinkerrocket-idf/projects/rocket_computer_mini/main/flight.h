#pragma once

#include <stdint.h>

// Flight side (flight.cpp): the flight_computer's job, in-process.
// Everything here runs only in ACTIVE mode — the peripheral rail must be up
// and the buses initialized before flight_setup() is called.

// Synchronous bring-up: sensors (SensorCollector.begin — takes tens of
// seconds worst case with GNSS probing), EKF statics, NVS-backed config,
// pyro pins (safe init), snapshot recovery scan. Called from the main task
// (16 KB stack exists for exactly this — see sdkconfig.defaults).
void flight_setup();

// The "flight" task entry: free-running loop, pinned core 1, prio
// configMAX_PRIORITIES-1, 16 KB stack, task-WDT subscribed. Never returns.
void flightTask(void* arg);

// True when powering the rail down is safe: not PRELAUNCH/INFLIGHT, pyro
// channels all idle, no mag-cal in progress. The comms side checks this
// before honoring a power-off command.
bool flightSafeToPowerOff();
