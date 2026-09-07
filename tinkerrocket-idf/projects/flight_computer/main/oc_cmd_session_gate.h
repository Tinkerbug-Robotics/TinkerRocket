#pragma once

#include <cstdint>

#include "RocketComputerTypes.h"

// #1105: admission gate for one-shot actuating commands served by the OC.
//
// The OC serves each command for CMD_REPEAT_LIMIT polls and advances its
// serving slot only on polls it actually receives, so an FC reset inside a
// repeat window freezes the slot on the command that was being served. The
// rebooted FC's dedup key (last_processed_cmd) boots to zero, so the frozen
// command looks new and is executed a second time — for PYRO_FIRE_TEST that
// is a second ARM + FIRE pulse with no operator action, after a gap bounded
// only by the FC's downtime. Neither the setup_fc status read nor the first
// main-loop poll can tell a frozen command from a fresh one by its value, and
// the OC's slave TX re-serves its last staged response on every master read,
// so the boot read can return the pre-reset response even after the OC has
// cleaned its slot. The FC needs a rule of its own.
//
// The distinguishing fact is the EDGE. The OC always serves one idle (cmd=0)
// poll between consecutive commands (#366/#368), so a command issued to THIS
// session is always preceded, within this session, by an observed idle poll.
// A command observed before any idle poll can only be one that was already
// being served when this boot started — issued to the previous session, or
// tapped during this boot before the FC could act — and either way not
// something to execute now. Config commands are exempt: delivering one twice
// is idempotent, and the connect-time sync that queues across power-on is
// legitimately served to a booting FC with no idle poll in front of it.
//
// Pure — one bit of state and no I2C — so the host suite can drive it with
// the exact read sequences the FC sees: boot read, loop reads, failed reads.
struct OcCmdSessionGate
{
    bool idle_seen = false;   // an OUT_STATUS_RESPONSE with cmd == 0 has been read this boot

    // Call with the command byte of EVERY successfully unpacked status
    // response — the setup_fc boot read and each main-loop poll read alike.
    // A failed read (I2C error, unpack failure) must not call this: it says
    // nothing about the slot.
    void observe(uint8_t served_cmd)
    {
        if (served_cmd == 0U)
        {
            idle_seen = true;
        }
    }

    // May the dispatcher execute `cmd` now? Non-actuating commands: always.
    // One-shot actuating commands: only once an idle poll has been seen this
    // boot, i.e. only on a 0 -> cmd edge that happened in this session. The
    // bit is sticky on purpose: after the first idle poll, a later missed one
    // (an I2C read failure landing on the idle gap) must not refuse anything.
    bool admits(uint8_t cmd) const
    {
        return !cmdIsOneShotActuating(cmd) || idle_seen;
    }
};
