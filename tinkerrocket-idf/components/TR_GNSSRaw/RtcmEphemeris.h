// RtcmEphemeris.h — RTCM 10403.3 broadcast-ephemeris messages to the receiver's gps_eph_t
//
// With $PAIR436,1 the Beetle's LC86G sends the broadcast ephemeris it decodes,
// one RTCM3 message a second round-robin: 1019 GPS LNAV, 1046 Galileo I/NAV,
// 1042 BeiDou D1 (and 1020/1044 for GLONASS and QZSS, not used here). A
// raw-measurement filter places each satellite from them, so the flight
// computer needs no ephemeris of its own (decoded fields checked against the
// day's broadcast file on the bench: identical, gnss/sim/tinkerrocket_gnss).
//
// A decoded ephemeris keeps its system's own time, as gnss/eph.h expects:
// toe/toc in GPS seconds of week for GPS and Galileo (GST tracks GPS), BeiDou
// time (GPS - 14 s) for BeiDou. Units are SI and radians; field references are
// the RTCM data fields (DFnnn). Pure and host-compilable
// (tests_cpp/test_gnss_raw.cpp).
#ifndef RTCM_EPHEMERIS_H
#define RTCM_EPHEMERIS_H

#include <cstddef>
#include <cstdint>

#include "gnss/eph.h"

namespace gnss_raw
{

// The 12-bit message number at the start of an RTCM3 payload (DF002); 0 if too short.
uint16_t rtcmMessageType(const uint8_t* payload, size_t len);

// A GPS week that RTCM sends modulo 1024 (DF076), made whole by taking the
// one nearest a week known to be close (the receiver's current week, or any
// week within ~9 years of it).
int fullGpsWeek(int week_mod_1024, int near_gps_week);

// Decode one ephemeris payload (the message body, after the 3-byte frame
// header and before the CRC; the caller checks the CRC). True for a complete
// 1019, 1046 or 1042 message; false for anything else, a short payload, or a
// BeiDou GEO (PRN 1-5 and 59-63: D2, which gnss/eph.h does not place).
//
// `week` becomes the full GPS week of toe: GPS from the 10-bit week and
// `near_gps_week`, Galileo GST week + 1024, BeiDou BDT week + 1356.
// `health` is 0 for usable: GPS the 6-bit SV health, Galileo the E1-B signal
// health and data validity ((HS << 1) | DVS), BeiDou SatH1. `tgd` is the
// group delay for the signal the LC86G tracks: GPS TGD, Galileo BGD(E1,E5b)
// (the I/NAV clock's), BeiDou TGD1 (B1I).
bool decodeEphemeris(const uint8_t* payload, size_t len, int near_gps_week, gps_eph_t& out);

}  // namespace gnss_raw

#endif  // RTCM_EPHEMERIS_H
