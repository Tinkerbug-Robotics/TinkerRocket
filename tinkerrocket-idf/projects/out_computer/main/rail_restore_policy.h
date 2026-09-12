#pragma once

#include <stdint.h>

// #825: should this OC boot re-assert the FC power rail?
//
// setup_oc() historically treated every boot as a cold power-up and drove
// PWR_PIN LOW, waiting for BLE cmd 8. Through the D9 diode-OR that LOW is
// electrically identical to letting go, so after an OC fault reset the FC
// rail decays on R84/C105 (~0.8 s from the chip reset) and the flight
// computer — and all four pyro channels — power off mid-flight, ballistic.
//
// The fix: rail state is retained in RTC memory (survives every reset except
// a true power-on) and the boot re-asserts EARLY — before nvs_flash_init and
// the 500 ms boot delay, which alone would spend the whole R84/C105 window.
//
// Pure so the decision table is host-testable (dedup_reboot_policy.h
// pattern). Inputs:
//   reset_is_poweron — esp_reset_reason() == ESP_RST_POWERON: a real cold
//                      start; RTC memory is garbage and the rail is off.
//   magic_valid      — the RTC block carries the expected magic.
//   rail_was_on      — retained flag: the rail was commanded ON when the
//                      reset happened (updated at every rail toggle).
//   deliberate_off   — set immediately before the #9 power-off esp_restart():
//                      that reboot IMPLEMENTS power-off, and boot-time LOW is
//                      load-bearing for it — restoring the rail would undo
//                      the command the operator just gave.
//
// Everything else — fault resets (panic/WDT/brownout) and the OC self-OTA
// restart — restores the rail: the FC was running, and only a fault or an
// update stands between it and its telemetry.

namespace RailRestorePolicy {

// Consecutive restore boots allowed before standing down to rail-off idle.
// Bounds the brownout loop: a pack that sags under the restored FC+GNSS load
// browns the OC out again and again — pre-#825 firmware settled into the
// stable rail-off idle after ONE brownout, and this bound restores that
// endpoint after a few honest tries. The counter is cleared when a restore
// proves stable (loop task up, peripherals re-initialized) and on every
// deliberate cmd-8 power-on.
inline constexpr uint8_t kMaxRestoreAttempts = 3;

// #1129: how long after a restore boot the budget may be cleared.
//
// The budget was cleared on the OC's OWN init completing — the first loop_oc
// pass, right after initPeripherals(), roughly the 500 ms boot delay plus the
// OC's peripheral init. But the load it exists to bound is the FC's: the FC
// raises GPS_ACT and runs servo_control.wiggle() later in its own setup, and
// the FC's own comment puts the wiggle alone at 4.2 s. So the current draw
// that rail_restore_policy.h names as the trigger arrived strictly AFTER the
// counter had been zeroed, every attempt started from 0, and the stand-down
// branch was unreachable. A sagging pack cycled the FC rail forever instead of
// settling into the stable rail-off idle this bound exists to restore.
//
// 12 s clears the FC's wiggle with margin without being so long that a healthy
// board carries a stale count into a genuinely later fault.
inline constexpr uint32_t kRestoreProvenMs = 12000;

// Has this restore proven stable enough to hand back a fresh retry budget?
//
// Two conditions, and both matter:
//   fc_frame_seen — a NonSensorData frame has arrived since this boot, which
//                   means the FC reached its main loop with GNSS and servos
//                   powered. This is the evidence the OC's own init never was.
//   uptime        — past the FC's boot transient, so a frame that arrives
//                   early (a warm FC that never actually dropped) does not
//                   clear the budget before the load has been applied.
//
// Deliberately not "the rail is still on": the rail being on is the premise,
// not the proof. It is on the whole time the brownout loop is running.
inline bool restoreProven(bool boot_rail_restored, bool fc_frame_seen,
                          uint32_t uptime_ms)
{
    if (!boot_rail_restored) return false;   // nothing to prove
    return fc_frame_seen && uptime_ms >= kRestoreProvenMs;
}

inline bool shouldRestore(bool reset_is_poweron, bool magic_valid,
                          bool rail_was_on, bool deliberate_off,
                          uint8_t restore_attempts)
{
    if (reset_is_poweron) return false;   // cold start: rail legitimately off
    if (!magic_valid)     return false;   // RTC garbage: trust nothing
    if (deliberate_off)   return false;   // the power-off command's own reboot
    if (restore_attempts >= kMaxRestoreAttempts) return false;  // brownout loop bound
    return rail_was_on;
}

}  // namespace RailRestorePolicy
