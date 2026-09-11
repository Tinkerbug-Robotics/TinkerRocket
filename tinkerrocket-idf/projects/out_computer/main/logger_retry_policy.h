// logger_retry_policy.h — when the OC retries a flight logger whose
// TR_LogToFlash::begin() failed (#1228).
//
// #1132 made a dead logger non-fatal: initPeripherals() brings the radio and
// the FC link up around it and then sets peripherals_initialized — the flag
// that used to make a later initPeripherals() call re-run everything,
// logger.begin() included. So a NAND that failed to mount once (a marginal
// rail at power-on is the plausible case) stayed dead for the whole boot, with
// nothing but a power cycle to bring it back. Since #1228 loop_oc re-enters
// the LOGGING half on its own, on the schedule below; the radio and link half
// is never touched again.
//
// Why a short schedule and not the I2S RX retry's 1 Hz never-give-up loop: a
// begin() against a NAND that does not answer is not cheap. The LittleFS
// mount-then-format fallback polls a status register that never clears, at
// nandWaitReady()'s two-second timeout per operation, so ONE failed attempt
// can hold loop_oc — the task that drives the LoRa downlink and BLE — for ten
// seconds or more. Three retries spread over ~100 s recover the "would mount on
// a second try" case the retry exists for; a chip still dead after that is
// dead, the storage scorecard is already red, and the next power cycle retries
// from scratch anyway (a cmd-8 OFF is an esp_restart()).
//
// Why the holds are Deferred rather than spent: the retry re-enters NAND
// bring-up, which must not run during a flight (INFLIGHT per
// inflight_refusal_policy.h, silent-FC hold included — the same gate as the
// cmd-8 power-off), and is pointless after LANDED: the #317 lockout refuses a
// new flight log until a reboot, so a recovered logger could log nothing this
// boot. A sim re-arm that leaves LANDED makes it useful again, which is why
// LANDED defers instead of giving up. An FC OTA relay defers for the flight's
// reason: a multi-second stall in loop_oc trips the relay's stall watchdog.

#pragma once

#include <stdint.h>

namespace LoggerRetryPolicy {

inline constexpr uint8_t  kMaxRetries = 3;
inline constexpr uint32_t kRetryDelayMs[kMaxRetries] = { 10000u, 30000u, 60000u };

/// Milliseconds between a failed attempt and the next retry, given how many
/// retries have already been spent. 0 once the budget is gone.
inline uint32_t delayBeforeRetryMs(uint8_t retries_spent)
{
    return retries_spent < kMaxRetries ? kRetryDelayMs[retries_spent] : 0u;
}

enum class Verdict : uint8_t
{
    Idle,       // the logger is up: nothing to do
    Exhausted,  // budget spent: dead until the next power cycle
    Wait,       // a retry is scheduled but not yet due
    Deferred,   // due, but a flight / LANDED / OTA hold is on: ask again later
    Retry,      // due and clear: re-enter the logging half now
};

/// One decision per loop pass. `due_ms` is the millis() stamp the OC set when
/// the last attempt failed (now + delayBeforeRetryMs); the comparison is
/// signed so a due time across the 49-day millis() wrap still reads as due.
inline Verdict decide(bool     logger_ok,
                      uint8_t  retries_spent,
                      uint32_t now_ms,
                      uint32_t due_ms,
                      bool     flight_hold,
                      bool     ota_relay_active)
{
    if (logger_ok) return Verdict::Idle;
    if (retries_spent >= kMaxRetries) return Verdict::Exhausted;
    if ((int32_t)(now_ms - due_ms) < 0) return Verdict::Wait;
    if (flight_hold || ota_relay_active) return Verdict::Deferred;
    return Verdict::Retry;
}

}  // namespace LoggerRetryPolicy
