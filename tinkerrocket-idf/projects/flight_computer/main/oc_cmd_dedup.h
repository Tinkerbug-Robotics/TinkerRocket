#pragma once

#include <cstdint>

// #1112: dedup key + config-retry budget for the commands the OC serves.
//
// The OC serves each command for CMD_REPEAT_LIMIT polls, re-staging its
// config frame with every delivery, then serves one idle (cmd=0) poll. The
// FC mirrors the served command byte into out_pending_command on every
// successful poll read and executes each command once per serving window:
// the dedup key (last_processed_cmd) holds the command that was executed
// and clears only when the OC reports 0 (#368).
//
// A config-pending handler that finds no config frame in the read wants
// "retry on the next poll" — the OC's next delivery re-stages the frame and
// a fresh combined read may carry it intact. Clearing the key expressed that,
// but the dispatch used to run on EVERY loop pass (~1 kHz), not only on the
// pass that polled. The mirror is written only by the poll, so a cleared key
// re-dispatched the same stale mirror ~1 ms later, and every pass paid the
// full readConfigFrame() — three 96-byte reads plus 30 ms of back-off —
// against the same bytes that had just failed: a ~26 Hz retry storm in place
// of the 1 kHz flight loop for the rest of the OC's repeat window. And since
// the poll is the mirror's only writer, whatever stopped the poll — a real
// INFLIGHT, the #402 resync quiet window — froze the storm on for good, with
// the flight flown at ~26 Hz and the ISM6 handoff queue overflowing.
//
// Two rules, both here so the host suite can drive them pass by pass:
//
//   1. Dispatch is part of the poll transaction. take() hands out a command
//      only on a pass that ran the poll block; on every other pass it is a
//      no-op, so a cleared key cannot fire between polls, in a real flight,
//      or during the resync quiet window. (It also keeps every readConfigFrame
//      read under the bus mutex the poll took, which the dispatch always
//      assumed.)
//
//   2. A retry budget per served command. armRetry() clears the key at most
//      CFG_RETRY_LIMIT times for one command; once spent, the key keeps the
//      command and the handler does not run again until the OC reports 0.
//      That bounds a frame the OC dropped for size (#569) or an OC that
//      stopped answering with the mirror frozen non-zero — otherwise every
//      poll would re-run readConfigFrame() forever. The budget resets on the
//      OC's idle poll and whenever a different command is dispatched.
//
// Pure — three bytes of state and no I2C.
struct OcCmdDedup
{
    // The OC delivers each command 3 times (its CMD_REPEAT_LIMIT). The first
    // attempt rides delivery 1, so two retries cover deliveries 2 and 3 when
    // every read succeeds; the third absorbs one failed poll read in between
    // (the mirror stays at the command, and the OC re-stages the frame).
    static constexpr uint8_t CFG_RETRY_LIMIT = 3;

    uint8_t last_processed_cmd = 0;  // executed this serving window (0 = none)
    uint8_t retry_cmd = 0;           // the command the retry budget belongs to
    uint8_t retries   = 0;           // armRetry() calls granted for retry_cmd
    // #1137 item 4: the OC's serving-window epoch this key belongs to (0 = the
    // OC does not send one).  See take().
    uint8_t last_epoch = 0;

    // Call once per loop pass. `polled` says whether this pass ran the OC
    // poll block — the mirror may or may not have been refreshed by it (a
    // failed read leaves it stale, which is fine: the OC serves the same
    // command for the whole window). `pending` is the mirrored command byte.
    // Returns the command to dispatch on this pass, or 0.
    /// `epoch` is the OC's serving-window number from status payload byte 2
    /// (#1137 item 4), or 0 when the OC does not send one -- an older OC
    /// build, or the idle poll itself.
    uint8_t take(bool polled, uint8_t pending, uint8_t epoch = 0)
    {
        if (!polled)
        {
            return 0;   // no poll, no dispatch — never between polls
        }
        if (pending == 0)
        {
            last_processed_cmd = 0;   // the OC cleared its slot: window done
            last_epoch = 0;
            retry_cmd = 0;
            retries   = 0;
            return 0;
        }

        // #1137 item 4: a repeat delivery is one from the SAME serving window,
        // and the epoch is what names the window.  Dedup used to key on the
        // command id alone, which meant the single idle (cmd=0) poll between
        // two windows was the only thing separating them -- while the command
        // itself was delivered three times.  One lost read of that poll and
        // the next command was discarded as a duplicate whenever it happened
        // to carry the same id.
        //
        // With an epoch the idle poll stops being load bearing.  Without one
        // (an OC that predates this) the old rule still applies, so a mixed
        // pair of firmwares behaves exactly as it did before.
        const bool repeat = (epoch != 0)
            ? (epoch == last_epoch && last_processed_cmd != 0)
            : (pending == last_processed_cmd);
        if (repeat)
        {
            return 0;
        }
        if (pending != retry_cmd)
        {
            retry_cmd = 0;   // a different command: a fresh budget
            retries   = 0;
        }
        last_processed_cmd = pending;
        last_epoch         = epoch;
        return pending;
    }

    // A config-pending handler found no config frame in this poll's read.
    // Re-arm the dispatch for the NEXT poll while the budget lasts; returns
    // false once it is spent, in which case the key keeps the command and
    // the handler will not run again until the OC reports 0.
    bool armRetry()
    {
        if (retries >= CFG_RETRY_LIMIT)
        {
            return false;
        }
        ++retries;
        retry_cmd          = last_processed_cmd;
        last_processed_cmd = 0;
        return true;
    }
};
