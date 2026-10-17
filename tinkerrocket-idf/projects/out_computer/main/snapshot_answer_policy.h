// snapshot_answer_policy.h — what the OC says when the FC asks for its flight
// snapshot at boot (GET_FLIGHT_SNAPSHOT; #846, #1176).
//
// With a servable in-flight frame (the RAM cache, the V7/V8 MRAM slot or the
// flight token's carried frame), it serves that. Without one, it says "no
// flight in progress" (flight_snapshot_answer::noFlightPayload), and the FC
// stops asking at the first exchange instead of spending all five tries.
//
// That answer is a promise: the FC cold-starts on it, and a cold start in the
// middle of a descent is a ballistic. So the OC gives it only when it is
// certain, which is either of:
//
//   * This OC boot is not a recovery context: no #825 rail restore, no #1176
//     token restore. initPeripherals() then retires the MRAM slot and the NAND
//     re-seed retires its flight without seeding, so the one store that can
//     hold a flight is the RAM cache the live I2S link fills, and the caller
//     has just looked in it.
//   * The boot-time stores have had their chance: the logger is up, so the
//     MRAM slot is readable or the NAND re-seed has run.
//
// Otherwise it stays silent, as it always did before: a recovery boot whose
// logger hasn't come up yet, or an MRAM slot that read back garbled (neither
// the zeros of a retired slot nor a snapshot, so possibly a torn flight frame).
// The FC keeps asking for its five tries, and a logger retry or a clean
// second read inside that window can still serve the flight.

#pragma once

#include <stdint.h>

namespace SnapshotAnswerPolicy {

enum class Reply : uint8_t {
    Serve,      // hand back the servable frame
    NoFlight,   // stage the explicit "no flight in progress"
    Silent,     // stage nothing; the FC asks again
};

/// `servable`: a store produced a frame that snapshotServable() accepts.
/// `store_unreadable`: the V7/V8 MRAM slot read back garbled.
/// `recovery_context`: this OC boot restored the FC rail (#825) or powered it
/// for a token restore (#1176), latched for the session.
/// `stores_settled`: the MRAM slot is readable or the NAND re-seed has run.
inline constexpr Reply decide(bool servable, bool store_unreadable,
                              bool recovery_context, bool stores_settled)
{
    if (servable) return Reply::Serve;
    if (store_unreadable) return Reply::Silent;
    if (!recovery_context || stores_settled) return Reply::NoFlight;
    return Reply::Silent;
}

}  // namespace SnapshotAnswerPolicy
