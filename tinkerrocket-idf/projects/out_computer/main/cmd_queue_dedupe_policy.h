#pragma once

#include <cstdint>

#include "RocketComputerTypes.h"

// #837 item 11: decide whether a newly-arriving FC command is the SAME
// OPERATION as one already sitting in the OC's relay queue, and may
// therefore replace it in place ("latest value wins") rather than taking a
// slot of its own.
//
// The queue dedupes so a self-applying settings slider cannot flood it: drag
// a servo trim and the app emits a stream of SERVO_CONFIG_PENDING, of which
// only the final value matters. For those commands the payload IS the
// command, so matching on the command id alone is exactly right.
//
// It is wrong for PYRO_CONT_TEST and PYRO_FIRE_TEST. Their payload is a
// single CHANNEL byte (main.cpp passes `&ch, 1`), so "test channel 1" and
// "test channel 2" are two distinct operations that happen to share one
// command id. Keying on the id alone merged them: the queued entry's channel
// byte was overwritten 1 -> 2 and channel 1's test ceased to exist. Nothing
// reported it — no ack is sent for a QUEUED pyro test, and the LoRa uplink
// has no feedback channel at all (#285) — while the iOS per-channel "TESTING"
// indicator (#411) went on asserting for the full 2.5 s / 8 s relay window
// that the swallowed test was in flight, then reverted to the stale cached
// reading that #411 exists to suppress. An operator checking all four
// channels before walking back from the pad got one real measurement and
// three stale ones, with nothing to distinguish them.
//
// The rule below keeps the flood protection where it belongs — a repeated tap
// on the SAME channel still collapses — while giving different channels
// different slots.
//
// Pure header (the only dependency is the shared command-id constants) so the
// host suite can exercise it; the caller owns the queue and the locking.

// True for commands whose payload's first byte selects WHICH instance of the
// operation this is, rather than carrying the operation's whole value. For
// these, equal command ids are not sufficient for a dedupe match.
//
// Every other enqueue site passes a complete config struct (ServoConfigData,
// RollProfileData, MagCalApplyData, ...) where replacing the payload wholesale
// is the intended behaviour, so this list is exactly the two pyro tests.
inline constexpr bool cmdQueueKeyIncludesSelector(uint8_t cmd)
{
    return cmd == PYRO_CONT_TEST || cmd == PYRO_FIRE_TEST;
}

// The dedupe predicate. `queued_sel` / `new_sel` are the first payload byte
// (pass 0 when the entry carries no payload — selector-keyed commands always
// carry one, so the value is never consulted for the command-only case).
inline constexpr bool cmdQueueSameOperation(uint8_t queued_cmd, uint8_t queued_sel,
                                            uint8_t new_cmd, uint8_t new_sel)
{
    if (queued_cmd != new_cmd)
    {
        return false;
    }
    if (cmdQueueKeyIncludesSelector(new_cmd))
    {
        return queued_sel == new_sel;
    }
    return true;
}
