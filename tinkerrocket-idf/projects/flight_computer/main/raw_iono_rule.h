// raw_iono_rule.h — which GPS broadcast ionosphere the mosaic-only image's
// raw-GNSS filter builds its epochs with (pure, header-only, host-tested)
//
// Mosaic-only build (TR_RAW_NAV_MOSAIC; main.cpp's GNSS task, rawIonoOffer).
// The filter's measurement stage has two sources for the eight Klobuchar
// bytes GPS broadcasts:
//   - the set the app sends with a pad calibration, kept in NVS: the GPS day
//     of the navigation file it came from (ShadowIono::gps_day);
//   - the receiver's own GPSIon block (SbfNav.h decodeGpsIon): the GPS day it
//     decoded it.
// One rule for both, so neither undoes the other:
//   - a set older than the one in use (its GPS day) is not taken;
//   - on the same day, the receiver's wins: it heard the satellites today,
//     the app's file may be from any hour of that day;
//   - a set whose bytes are the ones in use is never applied again, whatever
//     its day or source: each change starts the pad survey over
//     (RawGnssMeasStage::chooseIono), and the receiver's GPSIon is offered on
//     every pass of the GNSS task.
// The Beetle's stage takes the app's set as it comes (main.cpp) and does not
// use this.
//
// Tested in tests_cpp/test_raw_iono_rule.cpp.
#ifndef RAW_IONO_RULE_H
#define RAW_IONO_RULE_H

#include <cstdint>
#include <cstring>

namespace RawIonoRule
{

struct State
{
    bool     have = false;      // a set is in use
    bool     from_rx = false;   //   the receiver's GPSIon (false: the app's)
    int8_t   raw[8] = {};       //   its bytes, alpha0..3 and beta0..3 as broadcast
    uint16_t gps_day = 0;       //   its GPS day (days since 1980-01-06)
    uint32_t sets = 0;          // sets taken since boot
};

// A set offered from one source. True when it is taken: `st` then holds it,
// and the caller applies it (RawGnssMeasStage::setIono). False when the set
// in use stands.
inline bool offer(State& st, const int8_t raw[8], uint16_t gps_day, bool from_rx)
{
    if (st.have)
    {
        if (std::memcmp(raw, st.raw, sizeof(st.raw)) == 0)
            return false;   // the same bytes: nothing to apply, the pad survey keeps its work
        if (gps_day < st.gps_day)
            return false;   // older than the one in use
        if (gps_day == st.gps_day && st.from_rx && !from_rx)
            return false;   // the same day: the receiver's stands
    }
    std::memcpy(st.raw, raw, sizeof(st.raw));
    st.gps_day = gps_day;
    st.from_rx = from_rx;
    st.have = true;
    st.sets++;
    return true;
}

}  // namespace RawIonoRule

#endif  // RAW_IONO_RULE_H
