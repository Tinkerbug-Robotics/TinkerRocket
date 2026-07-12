#ifndef MRAM_DIRTY_POLICY_H
#define MRAM_DIRTY_POLICY_H

// #417 / #274: single source of truth for the sink-mode "dirty" marker.
//
// The marker lives in non-volatile MRAM (see cfg.dirty_marker_addr). Its job is
// to tell the *next* boot that a log session was interrupted mid-flight (an
// in-flight brownout, #274) so the surviving MRAM ring is replayed into a
// recovered flight instead of being wiped.
//
// The bug (#417): the marker used to be written at openLogSession() — including
// the PRELAUNCH *pre-create* — and only cleared by a clean closeLogSession().
// On the bench the flow is PRELAUNCH (pre-create + marker) -> cut power (never
// launched, never cleanly closed), so the next boot saw the surviving marker
// and spawned a bogus flight_mram_recovered_*.bin on nearly every power cycle.
// What it "recovered" was just the ~500 ms prelaunch drop-oldest pad buffer.
//
// This policy encodes the fixed semantics: the marker means "logging was
// ACTIVATED" (real launch detect, or a deliberate manual cmd-23 start — both
// route through activateLogging), not merely "a file was pre-created". It is a
// pure RAM state machine with no hardware dependency; TR_LogToFlash drives it
// from the log-session lifecycle and mirrors markerShouldBeSet() to the
// physical MRAM marker after each transition. Being hardware-free, it is
// exercised host-side in test_mram_dirty_policy.cpp.
//
// Boot note: this object is fresh RAM at every boot (starts clean). The marker
// that *survived* power loss is read from hardware (checkMramDirty) and fed to
// shouldRecoverOnBoot() — the surviving hardware bit, not this live state, is
// what gates recovery.
class MramDirtyPolicy
{
public:
    // Log file pre-created at PRELAUNCH. The session is not active yet, so the
    // marker must stay clean (#417) — a power-cut here has no flight data worth
    // recovering.
    void onPreCreate() { /* intentionally does not dirty the marker */ }

    // Logging activated: real launch detect, or a deliberate manual cmd-23
    // start. From here an interrupted session has genuine ring data (preserved
    // prelaunch buffer + in-flight frames) that the #274 path should recover.
    void onActivate() { dirty_ = true; }

    // Clean end-of-session: end-of-flight drain or a manual stop. The ring was
    // handed off cleanly, so there is nothing to recover on the next boot.
    void onCloseClean() { dirty_ = false; }

    // The surviving ring has been drained into a recovered flight at boot; clear
    // the marker so it does not re-fire on the following boot.
    void onRecoveryFinished() { dirty_ = false; }

    // Desired physical-marker state for the current session. TR_LogToFlash
    // writes (or clears) the MRAM marker to match after each transition.
    bool markerShouldBeSet() const { return dirty_; }

    // Boot-time gate: replay the surviving ring iff the marker survived set.
    // Kept as a named function (rather than a bare bool) so the recovery
    // decision has one documented home shared by firmware and tests.
    static bool shouldRecoverOnBoot(bool marker_survived_set)
    {
        return marker_survived_set;
    }

private:
    bool dirty_ = false;
};

#endif  // MRAM_DIRTY_POLICY_H
