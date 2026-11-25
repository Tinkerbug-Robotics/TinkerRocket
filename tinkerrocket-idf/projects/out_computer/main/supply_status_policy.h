#pragma once

#include <stdint.h>

// #1593: the Tinker-Mantis V10's two supply-status inputs on the out computer.
//
//   PG_RAIL  (S3 GPIO17) U19 eFuse power-good. High = VBATT on; low = the
//            eFuse has cut the pack (trip or UVLO).
//   VBUCK_OK (S3 GPIO34) V_BUCK through a divider. High = the buck supplies;
//            low = the board is running on the hold-up capacitor.
//
// Both read high when all is well, and both pull-ups/dividers sit on rails
// that stay up through the event (the supercap-backed +3V3), so the level is
// truthful while it matters. For bring-up they are reported on the console
// only: the level at boot, every change, and the counts in the stats lines.
// The RTC-latched cause and the flight-log fields are #1409's.
//
// Polled (100 Hz with the FC rail on, 1 Hz with it off), not interrupt
// driven, so a fall shorter than the poll can go unseen. That is acceptable
// for a console report and is exactly what #1409's latch is for.
//
// Pure so the edge counting is host-testable; main.cpp owns the pins.

namespace SupplyStatus {

struct Line
{
    bool     seen       = false;  // the first sample has been taken
    bool     high       = false;  // the last level read
    uint32_t falls      = 0;      // high -> low since boot
    uint32_t rises      = 0;      // low -> high since boot
    uint32_t changed_ms = 0;      // millis() of the last change or the first sample
};

enum class Change : uint8_t
{
    None  = 0,  // same level as last time
    First = 1,  // the first sample: report the level, count nothing
    Fell  = 2,  // went low: the supply this line watches went away
    Rose  = 3,  // came back
};

// Feed one reading. Returns what changed, and keeps the counts.
inline Change sample(Line& l, bool high, uint32_t now_ms)
{
    if (!l.seen)
    {
        l.seen       = true;
        l.high       = high;
        l.changed_ms = now_ms;
        return Change::First;
    }
    if (high == l.high) return Change::None;
    l.high       = high;
    l.changed_ms = now_ms;
    if (high)
    {
        ++l.rises;
        return Change::Rose;
    }
    ++l.falls;
    return Change::Fell;
}

}  // namespace SupplyStatus
