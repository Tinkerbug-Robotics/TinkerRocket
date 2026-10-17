#pragma once

// The out computer's answer to the flight computer's boot-time
// GET_FLIGHT_SNAPSHOT (#104, #846, #1176), as both ends read it.
//
// The FC asks on every boot and acts on what comes back:
//
//   Restore    a live INFLIGHT snapshot -- resume the flight.
//   SimFlight  an INFLIGHT snapshot from a simulated flight -- refuse it. The
//              dry-fire gate lives in isSimActive(), which a reboot resets, so
//              restoring would fire real pyro outputs on bench igniters.
//   NoFlight   the OC's explicit "no flight in progress" (noFlightPayload
//              below), or a well-formed snapshot (magic and CRC good) that is
//              not a restorable flight: the FC's own LANDED clear, or a frame
//              from an image with another snapshot VERSION.
//   Garbage    anything else -- not an answer, the FC asks again.
//
// The first three are definitive and end the FC's retries.
//
// Until the OC sent noFlightPayload, it said nothing when it held no flight.
// The FC's read then returned the OC's last staged I2C reply, a status frame
// (type 0xA9), so every boot spent all five tries (about 1.5 s) and logged five
// "[RECOVERY] read OK but no SNAPSHOT frame" warnings.
//
// The answer has a magic of its own, kNoFlightMagic, and that is deliberate.
// An FC from before it treats every well-formed non-restore snapshot as "no
// flight" and sends its LANDED clear, which carries the FC's uptime as its
// flight_elapsed_ms; the OC's #846 replay filter would then drop the next
// flight's first INFLIGHT frames as replays. With its own magic, such an FC
// reads the answer as garbage: it keeps its five tries and never clears, as it
// always did. Only an FC that knows the magic takes the shortcut.
//
// The CRC is the caller's to compute: both ends use components/CRC's CRC32 over
// the kCrcBytes in front of crc32, and pass whether it matched. That keeps this
// header as dependency-free as the rest of TR_RocketComputerTypes.

#include <stddef.h>
#include <stdint.h>

#include "RocketComputerTypes.h"

namespace flight_snapshot_answer
{

// The bytes the snapshot's CRC32 covers: everything in front of crc32.
inline constexpr size_t kCrcBytes = offsetof(FlightSnapshotData, crc32);

// SOF(4) + type + len + payload + CRC16(2). The OC stages exactly this many
// bytes and the FC reads exactly this many (the #399 wire-alignment rule).
inline constexpr size_t kFrameLen = 4 + 1 + 1 + sizeof(FlightSnapshotData) + 2;

// The magic of the OC's "no flight in progress" ("NOFL"), in place of
// FlightSnapshotData::MAGIC, so that it can never be read as a snapshot.
inline constexpr uint32_t kNoFlightMagic = 0x4E4F464Cu;
static_assert(kNoFlightMagic != FlightSnapshotData::MAGIC,
              "the no-flight answer must not read as a snapshot");

enum class Verdict : uint8_t
{
    Restore,
    SimFlight,
    NoFlight,
    Garbage,
};

inline constexpr bool definitive(Verdict v) { return v != Verdict::Garbage; }

/// `crc_ok` is "s.crc32 equals the CRC32 over the first kCrcBytes of s".
inline Verdict classify(const FlightSnapshotData& s, bool crc_ok)
{
    if (!crc_ok) return Verdict::Garbage;
    if (s.magic == kNoFlightMagic) return Verdict::NoFlight;
    if (s.magic != FlightSnapshotData::MAGIC) return Verdict::Garbage;
    if (s.version == FlightSnapshotData::VERSION &&
        s.rocket_state == (uint8_t)INFLIGHT)
    {
        return s.sim_flight ? Verdict::SimFlight : Verdict::Restore;
    }
    return Verdict::NoFlight;
}

/// The OC handed back an in-flight frame this FC won't restore (a sim flight,
/// another snapshot VERSION). The OC's store still holds it, so the FC's boot
/// sends its LANDED clear to retire it. Nothing else wants that clear, the
/// OC's "no flight" least of all: the clear carries the FC's uptime as its
/// flight_elapsed_ms, which the OC's #846 replay filter would hold against the
/// next flight's first INFLIGHT frames.
inline bool refusedInflight(Verdict v, const FlightSnapshotData& s)
{
    return v == Verdict::SimFlight ||
           (v == Verdict::NoFlight && s.rocket_state == (uint8_t)INFLIGHT);
}

/// The OC's explicit "no flight in progress": kNoFlightMagic, the LANDED state,
/// every other field zero. The caller stamps crc32 before packing it as a
/// SNAPSHOT_MSG.
inline FlightSnapshotData noFlightPayload()
{
    FlightSnapshotData s = {};
    s.magic        = kNoFlightMagic;
    s.version      = FlightSnapshotData::VERSION;
    s.rocket_state = (uint8_t)LANDED;
    return s;
}

/// The answer is the OC's explicit "no flight in progress".
inline bool isNoFlightAnswer(const FlightSnapshotData& s, bool crc_ok)
{
    return crc_ok && s.magic == kNoFlightMagic;
}

}  // namespace flight_snapshot_answer
