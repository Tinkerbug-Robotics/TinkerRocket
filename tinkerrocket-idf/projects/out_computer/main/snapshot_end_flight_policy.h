#pragma once

// #1779: END_FLIGHT retires a snapshot the FC's LANDED clear did not reach.
//
// The FC sends END_FLIGHT only from LANDED, once its LANDED snapshot clear is
// in the same FIFO queue (flight_computer/main/landed_sends_policy.h). If the
// OC's RAM cache still reads INFLIGHT when END_FLIGHT arrives, that clear was
// lost on the link. The FC sends the clear again every second while LANDED;
// this covers the time until the next one arrives. Left so, the cache would keep the flight's last in-flight frame and
// serve it for 30 min, and an FC reset in that time would restore INFLIGHT on
// the ground. So the OC marks that frame LANDED, in place, in the cache and in
// its RTC copy (#1717). The OC then holds what a delivered clear leaves: a
// cache that is not servable, and an RTC copy that, intact but not servable,
// says the flight is over on a later boot of the OC (Verdict::Over). The V7/V8
// MRAM slot is not marked; the FC's next repeated clear writes it.
//
// Only the state byte changes. The frame's own CRCs are left as they were, so
// it no longer reads as a valid frame on the link. It is never sent: every
// serve path needs an INFLIGHT frame (main.cpp's snapshotServable), and the FC
// checks the CRCs besides. The RTC copy's own CRC is recomputed, so it stays
// intact.
//
// A frame that is already a clear, or is not a snapshot, is left alone.
//
// Pure so the host tests can step it (tests_cpp/test_landed_clear.cpp).

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "RocketComputerTypes.h"
#include "snapshot_rtc_copy.h"

namespace SnapshotEndFlight {

// The payload's place in a snapshot wire frame: past SOF (4), type and length.
inline constexpr size_t kPayloadOffset = 6;

// Mark a snapshot wire frame that reads INFLIGHT as LANDED. Whether it did.
inline bool markLanded(uint8_t* frame, size_t len)
{
    if (len < kPayloadOffset + sizeof(FlightSnapshotData)) return false;
    uint32_t magic = 0;
    memcpy(&magic, frame + kPayloadOffset + offsetof(FlightSnapshotData, magic), sizeof(magic));
    uint8_t* state = frame + kPayloadOffset + offsetof(FlightSnapshotData, rocket_state);
    if (magic != FlightSnapshotData::MAGIC || *state != (uint8_t)INFLIGHT) return false;
    *state = (uint8_t)LANDED;
    return true;
}

// The same for the RTC copy, which stays intact. A copy that is not intact is
// left alone: it is no record of a flight.
template <size_t N>
inline bool markLanded(SnapshotRtcCopy::Copy<N>& c)
{
    if (!SnapshotRtcCopy::intact(c)) return false;
    if (!markLanded(c.frame, N)) return false;
    c.crc = SnapshotRtcCopy::crcOf(c);
    return true;
}

}  // namespace SnapshotEndFlight
