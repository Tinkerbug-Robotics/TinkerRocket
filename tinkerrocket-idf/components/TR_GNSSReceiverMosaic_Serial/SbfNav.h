// SbfNav.h — the mosaic-G5's broadcast ephemerides (GPSNav, GALNav, BDSNav) as gps_eph_t, and its GPS ionosphere as the 8 broadcast bytes (pure, host-compilable)
//
// The receiver decodes each satellite's navigation message itself and sends
// it as SBF floats in the ICD's own units, so these decoders only scale
// semicircles to radians and fill gps_eph_t by the conventions
// RtcmEphemeris.h uses for the LC86G's RTCM 1019/1046/1042, so the same
// EphemerisStore and RawMeasBuilder take either:
//   - week: the full GPS week of toe (GPS from the 10-bit week, Galileo from
//     the GPS week mod 4096 the receiver gives, BeiDou BDT week + 1356);
//   - toe/toc in the system's own time (BeiDou's in BDT, 14 s behind GPS);
//   - tgd: the group delay for the one signal the filter uses of each system
//     (GPS T_gd for L1 C/A, Galileo BGD(E1,E5b) of the I/NAV clock for E1,
//     BeiDou T_GD1 for B1I);
//   - health 0 = usable: GPS the 6-bit SV health, Galileo (E1-B HS << 1) |
//     E1-B DVS, BeiDou SatH1.
// The SBF floats are the broadcast integers times their ICD scale exactly
// (checked on the 10/16 V9 captures), so an ephemeris decoded here equals
// the RTCM one of the same issue field for field.
//
// What is refused, and why (the 10/16 captures):
//   - Galileo F/NAV (Source 16): it repeats the I/NAV IODnav and toe with the
//     (E1,E5a) clock and a do-not-use BGD(E1,E5b); EphemerisStore keeps the
//     first of a (IODE, toe) pair, so an F/NAV copy would win half the time.
//   - BeiDou GEO (C01-C05, C59-C63): gnss/eph.h cannot place a D2 orbit.
//   - GPS at a cutover (IODE2, IODE3 and IODC's low byte differ): the
//     subframes are of two issues.
//   - a do-not-use group delay or orbit float.
//
// Reference: mosaic-G5 Reference Guide, firmware 1.0.1, 4.2.x (GPSNav 5891,
// GPSIon 5893, GALNav 4002, BDSNav 4081). Pinned by
// tests_cpp/test_sbf_nav.cpp against the LC86G's RTCM golden ephemerides.
#ifndef SBFNAV_H
#define SBFNAV_H

#include <cstddef>
#include <cstdint>

#include "SbfParser.h"
#include "gnss/eph.h"

namespace mosaic
{

// Block numbers (Reference Guide, appendix B).
static constexpr uint16_t kSbfGpsNav = 5891;
static constexpr uint16_t kSbfGalNav = 4002;
static constexpr uint16_t kSbfBdsNav = 4081;
static constexpr uint16_t kSbfGpsIon = 5893;

// The longest ephemeris block a raw-filter queue item must hold. Today's
// are 140 (GPSNav, BDSNav) and 152 (GALNav) bytes.
static constexpr size_t kSbfNavMaxLen = 160;

// gps_eph_t::health bits beyond the broadcast ones, for a Galileo ephemeris
// whose E1-B status the block does not carry (Health_OSSOL bit 0 clear) or
// whose SISA(E1,E5b) is "no accuracy prediction available" (255). Either
// makes it unhealthy, so EphemerisStore::select passes it over.
static constexpr int kGalHealthNoStatus = 0x100;
static constexpr int kGalHealthNoSisa   = 0x200;

// GPSNav, GALNav or BDSNav.
bool isSbfEphemerisBlock(uint16_t number);

enum class SbfEphStatus : uint8_t
{
    OK,
    NOT_EPHEMERIS,   // not "$@", or not GPSNav/GALNav/BDSNav
    SHORT,           // fewer bytes than its Length, or a Length short of the fields read
    NO_WEEK,         // neither the block's WNc nor the caller's week is known
    BAD_SVID,        // a satellite number outside the system's range
    FNAV_SKIPPED,    // GALNav from any source but I/NAV (Source 2)
    GEO,             // a BeiDou GEO
    IOD_MISMATCH,    // GPS IODE2, IODE3 and IODC & 0xFF disagree
    DNU_FIELD,       // a do-not-use (or NaN) float the ephemeris needs
};

// Decode one CRC-checked GPSNav, GALNav or BDSNav block into out (only on
// OK). The weeks the blocks send modulo 1024 (GPS), 4096 (Galileo) and 8192
// (BeiDou) are made whole against the block's own WNc stamp (the time the
// receiver got the message, a GPS week); `near_wnc`, a GPS week known to be
// close (the receiver's current WNc), stands in when that stamp is
// do-not-use. Pass kWncDnu when there is none.
SbfEphStatus decodeSbfEphemeris(const uint8_t* b, size_t len, uint16_t near_wnc, gps_eph_t& out);

// The cheap check a receiver driver makes before queueing a nav block for
// decodeSbfEphemeris: an ephemeris block long enough to decode, and not one
// it would refuse for its source (Galileo F/NAV) or orbit (BeiDou GEO). It
// does not check the queue item's size against kSbfNavMaxLen.
bool sbfNavKeep(const uint8_t* b, size_t len);

// GPSIon (5893): the GPS Klobuchar parameters as the 8 bytes the satellites
// broadcast them in (alpha0..3, beta0..3; RawMeasBuilder::ionoFromBroadcast
// and ShadowIono take these), and the GPS day they were received (WNc * 7 +
// TOW / 1 day). The receiver's floats are the broadcast integers times
// 2^-30, 2^-27, 2^-24, 2^-24, 2^11, 2^14, 2^16, 2^16 exactly; false for any
// that is not (or is out of int8 range), for another block, a short one, or
// a do-not-use time stamp.
bool decodeGpsIon(const uint8_t* b, size_t len, int8_t raw[8], uint16_t& gps_day);

}  // namespace mosaic

#endif  // SBFNAV_H
