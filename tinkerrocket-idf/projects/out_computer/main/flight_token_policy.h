#pragma once

#include <stdint.h>

// #1176: should this OC boot bring the FC's rail up because a flight was in
// progress, even though the reset looks like a cold start?
//
// THE DEFECT THIS CLOSES. On 2026-08-29 a V8 lost power for an instant under
// boost, rebooted, and did nothing for the remaining ~75 s of the flight.
// Every layer of the recovery machinery refused, and each refused by design,
// because they all ask the same question — "was this reset a fault?" — and a
// power interruption is precisely the event that destroys the evidence needed
// to answer it. On V7/V8 the first refusal is fatal on its own: the FC has no
// hold pin of its own (board_v7.h:107, board_v8.h:188), so PWR_PIN is its only
// power, and RailRestorePolicy declines before the retained state is even read.
//
// WHY THE RESET REASON CANNOT CARRY THE FIX, and this is not a preference.
// On the ESP32-S3 the reset-cause register aliases a chip-level brownout to a
// power-on: soc/esp32s3/include/soc/reset_reasons.h defines
// RESET_REASON_CHIP_POWER_ON, RESET_REASON_CHIP_BROWN_OUT and
// RESET_REASON_CHIP_SUPER_WDT ALL as 0x01. The silicon cannot tell them apart.
// The P4 additionally emits CPU_LOCKUP, PWR_GLITCH and (for a supply glitch
// over 50 ns) codes IDF maps to ESP_RST_UNKNOWN, none of which appear in any
// whitelist in this tree. A predicate built on that register is both unsound
// and incomplete, and the failure it misses is intermittent by dip depth —
// which is exactly why this never reproduced on a bench.
//
// So the durable evidence is "a flight was in progress and never cleanly
// ended", written to NVS where it survives a power cut, and the reset reason
// is NOT an input to tier 2 at all. The mini already works this way
// (rocket_computer_mini/main/main.cpp), and it is the one place in the tree
// that recovers correctly after a full power loss.
//
// TIER 2, NOT A REPLACEMENT FOR TIER 1. RailRestorePolicy stays first and
// untouched: it wins the ~0.8 s R84/C105 decay race on fault resets, which is
// the entire point of #825, and it must not grow a millisecond. Tier 2 is
// consulted only once tier 1 has DECLINED — by which point the rail is already
// down and there is no decay window left to lose, which is what licenses
// reading NVS here at all.
//
// THE TWO CRASH-LOOP BOUNDS COMPOSE, they do not override. `tier1_stood_down`
// carries RailRestorePolicy's own exhausted-budget verdict into this decision,
// so a pack that browns the OC out repeatedly cannot escape tier 1's stand-down
// by falling through to tier 2. It must be read BEFORE the caller's stand-down
// block zeroes rail_rtc, and `deliberate_off` likewise before that flag is
// consumed.
//
// FAIL CLOSED IN BOTH DIRECTIONS. A read failure refuses, which is obvious.
// Less obvious, and the reason `nvs_writable` is an input: the attempt counter
// is the only bound on repeated restores, and it is a WRITE. If it cannot be
// incremented and read back, the bound does not exist, so the restore must be
// refused rather than performed with a void budget.
//
// Pure so the decision table is host-testable, in the house style of
// rail_restore_policy.h and pwr_hold_policy.h beside it.

namespace FlightTokenPolicy {

// Consecutive automatic restores allowed before the board stands down and
// waits for an operator.
//
// Five rather than a tighter number for two reasons that only hold together:
// the arming interlock means a wrong restore cannot energise anything, and
// refutation now drives a stale-token ground restore to LANDED, which retires
// the token after ONE restore. So this budget only ever bounds genuine
// repeated power loss in the air — a chattering battery connector, which is
// repetitive by nature and is the failure that started all of this.
//
// Exhaustion REFUSES; it does not clear the token. Clearing would erase the
// evidence that the vehicle was flying, which is the one thing worth keeping
// when a board has given up trying to come back.
inline constexpr uint8_t kMaxPoweronRestores = 5;

inline constexpr uint32_t kFlightTokenMagic = 0x464C544Bu;  // 'FLTK'

enum : uint8_t {
    kTokNone     = 0,   // no flight in progress
    kTokArmed    = 1,   // on the pad, PRELAUNCH reached
    kTokInflight = 2,   // launched, never cleanly ended
};

// Tier 2 of the boot rail decision. See the header note for why the reset
// reason is deliberately absent from the inputs.
inline bool shouldAutoRestore(bool magic_ok, bool crc_ok, uint8_t state,
                              uint8_t attempts, bool deliberate_off,
                              bool tier1_stood_down, bool nvs_writable)
{
    if (deliberate_off)        return false;  // the power-off command's own reboot
    if (tier1_stood_down)      return false;  // compose with tier 1, never override it
    if (!nvs_writable)         return false;  // no bound is possible: refuse
    if (!magic_ok || !crc_ok)  return false;  // trust nothing
    if (state != kTokInflight) return false;  // ARMED is a pad state, never a restore
    return attempts < kMaxPoweronRestores;
}

}  // namespace FlightTokenPolicy
