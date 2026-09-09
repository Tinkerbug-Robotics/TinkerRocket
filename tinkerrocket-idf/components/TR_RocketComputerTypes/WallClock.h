#pragma once
#include <cstdint>

// #1155 item 3: the phone-synced wall clock, advanced by elapsed seconds with a
// real calendar carry.
//
// Both rocket firmwares name a flight from "sync + (millis() - sync_millis)".
// The old arithmetic added whole elapsed days straight onto the synced
// day-of-month — no carry into the month or year — so a sync older than the
// days left in that month produced an impossible date (flight_20260932_...),
// and the uint8_t day accumulator wrapped after 256 days of uptime. This is
// the proleptic-Gregorian day count (Howard Hinnant's days_from_civil /
// civil_from_days), pure integer arithmetic, no time.h, no locale, valid for
// every date the apps can send. Host-tested in tests_cpp/test_wall_clock.cpp.
//
// millis() wraps every 49.7 days; the unsigned subtraction the callers do is
// correct across one wrap and ambiguous after two, which no rocket reaches.
namespace tr_wall_clock {

struct Civil
{
    uint16_t year;
    uint8_t  month;   // 1..12
    uint8_t  day;     // 1..31
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
};

// Days since 1970-01-01 for a civil date (month 1..12, day 1..31).
inline int64_t daysFromCivil(int64_t y, unsigned m, unsigned d)
{
    y -= (m <= 2) ? 1 : 0;
    const int64_t  era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);              // [0, 399]
    const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;  // [0, 365]
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;            // [0, 146096]
    return era * 146097 + static_cast<int64_t>(doe) - 719468;
}

// Inverse of daysFromCivil.
inline void civilFromDays(int64_t z, uint16_t& year, uint8_t& month, uint8_t& day)
{
    z += 719468;
    const int64_t  era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);                    // [0, 146096]
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;      // [0, 399]
    const int64_t  y   = static_cast<int64_t>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);                    // [0, 365]
    const unsigned mp  = (5 * doy + 2) / 153;                                         // [0, 11]
    const unsigned d   = doy - (153 * mp + 2) / 5 + 1;                                // [1, 31]
    const unsigned m   = mp < 10 ? mp + 3 : mp - 9;                                   // [1, 12]
    year  = static_cast<uint16_t>(y + ((m <= 2) ? 1 : 0));
    month = static_cast<uint8_t>(m);
    day   = static_cast<uint8_t>(d);
}

// sync + elapsed_s, carried through minutes, hours, days, months and years.
inline Civil advance(const Civil& sync, uint32_t elapsed_s)
{
    const uint64_t total_s = static_cast<uint64_t>(sync.hour) * 3600u +
                             static_cast<uint64_t>(sync.minute) * 60u +
                             sync.second + elapsed_s;
    const int64_t days = daysFromCivil(sync.year, sync.month, sync.day) +
                         static_cast<int64_t>(total_s / 86400u);
    const uint32_t sod = static_cast<uint32_t>(total_s % 86400u);
    Civil out{};
    civilFromDays(days, out.year, out.month, out.day);
    out.hour   = static_cast<uint8_t>(sod / 3600u);
    out.minute = static_cast<uint8_t>((sod % 3600u) / 60u);
    out.second = static_cast<uint8_t>(sod % 60u);
    return out;
}

}  // namespace tr_wall_clock
