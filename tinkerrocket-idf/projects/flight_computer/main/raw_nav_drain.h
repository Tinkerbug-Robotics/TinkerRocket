// raw_nav_drain.h — when the mosaic-only image's GNSS task files the
// receiver's navigation blocks into the raw-GNSS filter's ephemeris store
// (pure, header-only, host-tested)
//
// Mosaic-only build (TR_RAW_NAV_MOSAIC; main.cpp's GNSS task,
// rawGnssMeasTask). The mosaic-G5 sends its GPSNav, GALNav and BDSNav blocks
// when they change and again on a resend, and the driver queues them whole.
// The stage's store goes by second of week alone, so an ephemeris a whole
// number of weeks old would be placed as today's. RawGnssMeasStage::
// addEphemeris refuses one whose toe is far from the receiver's time, but it
// knows that time only from an epoch it has seen, and the boot's first
// blocks -- the receiver's stored copies, sent when the outputs come on and
// at the link setup's resend -- are queued before the GNSS task runs, whose
// pass takes the blocks before the epochs. So the blocks wait in the
// driver's queue until an epoch has named the receiver's week; from then on
// each is decoded (SbfNav.h: its week made whole against its own WNc stamp,
// or against the receiver's where that is do-not-use) and offered to the
// stage, which checks it against that epoch's time.
//
// A queue that filled while the blocks waited dropped some. noteWeek says
// when the week is first named after a wait, so the caller can ask the
// receiver for its navigation data once more, now that it can be checked.
//
// Tested in tests_cpp/test_raw_nav_drain.cpp.
#ifndef RAW_NAV_DRAIN_H
#define RAW_NAV_DRAIN_H

#include <cstddef>
#include <cstdint>

#include "SbfNav.h"
#include "gnss/eph.h"

namespace RawNavDrain
{

struct State
{
    uint16_t week = mosaic::kWncDnu;   // the receiver's week, from the last epoch that named it
    bool     held = false;             // a pass found the week not yet named and took nothing
    uint32_t refused = 0;              // blocks SbfNav would not decode
};

// An epoch's week, noted before the stage sees that epoch (offer or skip,
// whose see() gives the stage the time it checks against). True only the
// first time a week is named after a pass held the blocks: the caller then
// asks for a resend if the queue dropped any meanwhile.
inline bool noteWeek(State& s, uint16_t gps_week)
{
    if (gps_week == 0 || gps_week == mosaic::kWncDnu)
        return false;
    const bool first = s.week == mosaic::kWncDnu;
    s.week = gps_week;
    return first && s.held;
}

// One pass of the GNSS task, before its epochs: nothing taken until an
// epoch has named the week; then every block take(buf, cap, len) gives,
// decoded and handed to file(const gps_eph_t&). True if any block was taken.
template <class Take, class File>
bool drain(State& s, Take&& take, File&& file)
{
    if (s.week == mosaic::kWncDnu)
    {
        s.held = true;   // they wait in the driver's queue
        return false;
    }
    uint8_t nav[mosaic::kSbfNavMaxLen];
    size_t len = 0;
    bool any = false;
    while (take(nav, sizeof(nav), len))
    {
        gps_eph_t eph;
        if (mosaic::decodeSbfEphemeris(nav, len, s.week, eph) == mosaic::SbfEphStatus::OK)
            file(eph);   // the stage counts what it files and refuses
        else
            s.refused++;
        any = true;
    }
    return any;
}

}  // namespace RawNavDrain

#endif  // RAW_NAV_DRAIN_H
