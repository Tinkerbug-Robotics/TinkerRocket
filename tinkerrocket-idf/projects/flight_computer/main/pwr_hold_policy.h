#pragma once

// #848: boot-time reconciliation policy for the P4_EN_HOLD power latch.
//
// The FC asserts PWR_HOLD_PIN (V9/V10 GPIO5 -> D9 diode-OR -> U30 enable)
// while INFLIGHT so an OC fault-reset cannot drop the FC's rail mid-flight
// (#825), using gpio_hold_en so the pad stays latched HIGH through the FC's
// OWN panic/WDT resets (LP_IOMUX pad hold survives every digital reset; IDF
// clears it only after deep-sleep wake). That latch means a rebooting FC can
// find the pin already held from before the reset, and boot has to decide
// what to do with it. Pure function so the decision table is host-testable —
// nothing else in the FC tree can exercise GPIO hold semantics off-target.
//
// Inputs:
//   hold_latched   — the RTC-memory flag says the previous boot asserted the
//                    hold and never released it (survives the same resets the
//                    pad latch does; magic-validated by the caller).
//   state_inflight — post-recovery rocket_state == INFLIGHT (the #104 restore
//                    ran and validated a mid-flight snapshot).
//   oc_ready       — the OC answered the boot status query (out_ready): the
//                    other rail driver is alive.
//
// The asymmetry on the orphaned-hold row is deliberate: a latched hold is
// only ever set in INFLIGHT, so finding one at boot without a recovered
// flight means the rocket flew and something ended the software's knowledge
// of it (crash, landing the recovery could not see, dead OC). If the OC is
// alive it owns the rail again — release. If the OC is NOT answering, the
// hold is the only thing keeping the GNSS tracker alive on a downed rocket —
// keep it, and accept that power-off needs a battery pull.

namespace PwrHoldPolicy {

enum class BootAction {
    None,       // no hold pin activity at boot (normal cold start)
    Assert,     // (re)assert: we are in flight — latch the rail
    Release,    // hand rail control back to the OC
    KeepHold,   // leave the latched hold in place (orphaned-flight tracker mode)
};

inline BootAction bootAction(bool hold_latched, bool state_inflight, bool oc_ready)
{
    if (state_inflight) return BootAction::Assert;   // recovery restored a flight
    if (!hold_latched)  return BootAction::None;     // nothing latched, nothing to do
    return oc_ready ? BootAction::Release : BootAction::KeepHold;
}

}  // namespace PwrHoldPolicy
