#pragma once

#include <stdint.h>

// #1151: keeping the OC's I2C slave device recoverable after a failed
// re-create.
//
// resetSlaveTx() (#402's desync recovery) deletes the slave device and builds
// a new one. Both halves can fail — i2c_new_slave_device() wants a fresh rx
// ringbuffer, a receive buffer, a tx ringbuffer and an interrupt allocation,
// and returns ESP_ERR_NO_MEM under heap pressure. The event that calls it is
// itself a symptom of trouble, so the allocation is being attempted at the
// worst moment: a BLE log download, the 64 KB I2S ring and the NAND buffers
// can all be live.
//
// Before #1151 a single such failure was terminal. The interface's guard was
// "does a device exist" and the function nulls the handle before re-creating,
// so after one failure every later call — including the next I2C_TX_RESYNC,
// the exact event that wants a retry — returned ESP_ERR_INVALID_STATE. The
// FC<->OC command path was then dead for the rest of the power cycle: no
// camera start/stop, no deployment-channel config or continuity test, no sim
// stop, no mag-cal, no orientation push, and no snapshot service on a later FC
// reboot. Telemetry, LoRa and BLE all keep working through this, so nothing
// else signals that the rocket has stopped accepting commands — the FC just
// logs read failures and re-sends RESYNCs that can never succeed.
//
// The interface now keys its guard off "a device is WANTED" instead, which
// makes the retry possible. What lives here is when to spend it.
//
// The imperative half (which IDF calls, under which mutex) stays in main.cpp.
// This is the timing, because that is what is worth testing off-target.

namespace I2cSlaveRecoveryPolicy {

// Retry cadence. The same 1 s OtaRelayPolicy::kRxRetryIntervalMs uses, and for
// the same reason: fast enough that a transient allocation failure costs about
// a second of command availability, slow enough that a hard failure (a wedged
// port, a pin reassigned) does not spin the loop task or the log.
static constexpr uint32_t kRetryIntervalMs = 1000;

// How often to say so while it stays broken. A rocket that accepts no commands
// must be visible on the bench without reading every line, but the retry
// itself is once a second and does not deserve a line each time.
static constexpr uint32_t kComplainIntervalMs = 10000;

// Should we attempt the re-create this pass?
//
// `broken` is the latch: set when a re-create fails, cleared when one
// succeeds. Deliberately NOT derived from "is the device absent" — the device
// is also absent for the few microseconds inside resetSlaveTx() itself, and
// keying off absence is what made the original bug possible.
inline bool shouldRetry(bool broken, uint32_t now_ms, uint32_t last_try_ms)
{
    if (!broken) return false;
    return (uint32_t)(now_ms - last_try_ms) >= kRetryIntervalMs;
}

// Should we log the still-broken line this pass?
inline bool shouldComplain(bool broken, uint32_t now_ms, uint32_t last_log_ms)
{
    if (!broken) return false;
    return (uint32_t)(now_ms - last_log_ms) >= kComplainIntervalMs;
}

}  // namespace I2cSlaveRecoveryPolicy
