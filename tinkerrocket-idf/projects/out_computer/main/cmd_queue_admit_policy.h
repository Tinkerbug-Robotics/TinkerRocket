#pragma once

#include <cstddef>
#include <cstdint>

#include "RocketComputerTypes.h"

// #1116: the relay queue must never drop the command that ends an FC OTA
// image session.
//
// OTA_FINISH_CMD and OTA_ABORT_CMD are the ONLY way the OC ends a session on
// the FC: while an image is streaming the FC's I2S link is flipped to slave
// RX, so nothing else reaches it and nothing comes back. Every teardown path —
// the app's own finish or abort, the BLE disconnect, the stall watchdog, a
// failed flip — stages the command once, through the same bounded FIFO as a
// config sync, and none of them re-checks that it landed. A queue full of a
// profile sync at that moment dropped the command with a log line, and the FC
// stayed flipped: no telemetry, no logging, EKF off, until a battery pull.
//
// The FC now times such a session out on its own (fc_ota_session_policy.h,
// 30 s), so the drop is no longer permanent — but a dropped FINISH still turns
// a fully received image into an aborted update, and a dropped ABORT still
// costs those 30 s. Holding two slots back for these two commands means they
// always have somewhere to land. The dedupe in setPendingCommandWithConfig
// collapses a repeat onto the queued copy, so two is enough for both at once
// (a FINISH already queued when the app disconnects and stages the ABORT).
//
// Pure (the only dependency is the shared command-id constants) so the host
// suite can exercise it; the caller owns the queue and the locking.

// Slots held back for the session-ending commands. Every other command sees
// a queue of (depth - this) entries.
inline constexpr size_t kCmdQueueReservedForOtaTeardown = 2;

// The two commands that end an FC OTA image session.
inline constexpr bool cmdEndsOtaSession(uint8_t cmd)
{
    return cmd == OTA_FINISH_CMD || cmd == OTA_ABORT_CMD;
}

// May `cmd` take a slot when `count` of `depth` are already in use?
inline constexpr bool cmdQueueAdmits(uint8_t cmd, size_t count, size_t depth)
{
    if (count >= depth) return false;                        // physically full
    if (cmdEndsOtaSession(cmd)) return true;                 // may use the reserve
    return count + kCmdQueueReservedForOtaTeardown < depth;  // all else stops short of it
}
