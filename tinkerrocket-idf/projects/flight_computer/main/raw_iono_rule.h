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

// The rig replay's rule (a TR_SIM_REPLAY_BLOB build of the mosaic-only image,
// main.cpp's TR_RAW_NAV_RIG_REPLAY). The rig plays a recorded day's sky -- its
// scenario's broadcast file, weeks or months before the day the app's set was
// downloaded -- so by the date rule above an app or NVS set would always beat
// the receiver's own GPSIon, and the filter would build the rig's epochs with
// another day's ionosphere. Here the sky's set wins whatever the days:
//   - no set dated more than a day from the sky's (sky_day: the replayed
//     flight's GPS day; 0, not known, judges none) is taken, from either
//     source -- a pad calibration's from today, or one the receiver decoded
//     from the real sky before its cold start;
//   - an app set is taken only while the receiver has given none (by the rule
//     above), and never displaces one of the receiver's;
//   - the receiver's set replaces an app set, whatever their days;
//   - between two of the receiver's sets the rule above stands (no older day;
//     the same bytes never applied twice -- the receiver's bytes matching the
//     app set in use only make it the receiver's).
// The rig's files never broadcast GPSIon: subframe 4 page 18 comes once every
// 12.5 min, and the three flights' files (TOW 573606 on, 241-257 s) hold pages
// 21-25 and 1-4 only. So a replay build carries the sky's own set
// (-DTR_SIM_REPLAY_GPS_IONO), which the sim wrapper hands over as the
// receiver's; without it no set passes the day test above, and the filter
// builds the sky with none.
inline bool offerRigReplay(State& st, const int8_t raw[8], uint16_t gps_day, bool from_rx, uint16_t sky_day = 0)
{
    if (sky_day != 0 && (gps_day > sky_day + 1 || gps_day + 1 < sky_day))
        return false;   // not the sky's day: another day's ionosphere
    if (st.have && st.from_rx != from_rx)
    {
        if (!from_rx)
            return false;   // an app or NVS set never displaces the receiver's
        if (std::memcmp(raw, st.raw, sizeof(st.raw)) == 0)
        {
            st.from_rx = true;   // the receiver's own bytes already in use: nothing to apply
            st.gps_day = gps_day;
            return false;
        }
        st.have = false;    // the receiver's replaces the app's, whatever their days
    }
    return offer(st, raw, gps_day, from_rx);
}

}  // namespace RawIonoRule

#endif  // RAW_IONO_RULE_H
