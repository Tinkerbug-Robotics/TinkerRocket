// rinex_nav.h -- broadcast ephemerides from a RINEX 3 navigation file, as the
// mosaic-G5's SBF navigation blocks give them (raw_replay --rinex).
//
// The 10/16 V9 logs carry few ephemerides of their own (the receiver sends a
// satellite's navigation data when it changes, and the logs start long after
// the receiver did), so a replay takes the day's broadcast file to place the
// rest. Each record is filled into gps_eph_t by the conventions
// TR_GNSSReceiverMosaic_Serial/SbfNav.cpp uses for GPSNav, GALNav and BDSNav
// (and RtcmEphemeris for the LC86G), so the stage takes either and places a
// satellite the same:
//   - week: the full GPS week of toe (RINEX's GPS and Galileo weeks are GPS
//     weeks already; BeiDou's BDT week + 1356);
//   - toe and toc in the system's own time (BeiDou's in BDT); toc from the
//     record's epoch, which RINEX writes in that system's time;
//   - tgd: GPS T_GD, Galileo BGD(E5b,E1) of the I/NAV clock, BeiDou T_GD1 (B1I);
//   - health 0 = usable: GPS the 6-bit SV health, Galileo the E1-B DVS and HS
//     bits ((HS << 1) | DVS, RINEX's bits 0-2), with SbfNav's kGalHealthNoStatus
//     where the record holds no E1-B I/NAV and kGalHealthNoSisa where its SISA
//     is "no accuracy prediction" (RINEX writes -1), BeiDou SatH1;
//   - iode/iodc: GPS IODE and IODC, Galileo IODnav (both), BeiDou AODE and
//     AODC;
//   - ura: the index the receiver sends (GPS and BeiDou URA from RINEX's
//     metres, Galileo SISA); nothing places a satellite by it.
// Left out, as SbfNav refuses them: Galileo F/NAV (data source bit 1, or no
// E5b,E1 clock: bit 9 clear), BeiDou GEO (C01-C05, C59-C63), and every other
// system. RINEX writes angles in radians already.
//
// Each record also gets the GPS time it is first usable at (avail_gps_s): the
// mosaic's own nav blocks show when a receiver has each issue -- a GPS set 2 h
// before its toe (7182 s, every one of the 10/15 P1 capture's 32), a Galileo
// I/NAV batch 665 s after its toe, a BeiDou set 18 s after (BDT) -- so a
// replay hands the stage no issue before a receiver could have had it.
//
// Pure C++ (tests_cpp/test_rinex_nav.cpp).
#ifndef RAW_REPLAY_RINEX_NAV_H
#define RAW_REPLAY_RINEX_NAV_H

#include <string>
#include <vector>

#include "gnss/eph.h"

namespace rinex
{

// When a receiver first has an issue, from its toe (s, the system's own time
// scale; BeiDou's then moved to GPS time).
constexpr double kGpsLeadS = -7200.0;
constexpr double kGalLagS = 665.0;
constexpr double kBdsLagS = 18.0;

struct NavRecord
{
    gps_eph_t eph;
    double toe_gps_s = 0.0;     // its toe as continuous GPS seconds (week * 604800 + toe, BeiDou moved by 14 s)
    double avail_gps_s = 0.0;   // when a receiver first has it, continuous GPS seconds
};

struct NavFile
{
    std::vector<NavRecord> recs;   // GPS, Galileo I/NAV and BeiDou MEO/IGSO, file order
    int version_major = 0;
    int skipped_system = 0;        // GLONASS, SBAS, QZSS, NavIC records
    int skipped_fnav = 0;          // Galileo F/NAV, or with no E5b,E1 clock
    int skipped_geo = 0;           // BeiDou GEO
    int bad = 0;                   // records that did not parse
};

// Days from 1980-01-06 to a civil date (proleptic Gregorian).
long daysFromGpsEpoch(int year, int month, int day);

// A civil date and time in a system's own time scale as its week (counted as
// GPS weeks: BDT's from 1356) and seconds of week.
void calendarToWeek(int year, int month, int day, int hour, int minute, double second, int& week, double& sow);

// The URA index (IS-GPS-200 20.3.3.3.1.3, as BeiDou's) of an accuracy in
// metres; the Galileo SISA index of one (255: none, RINEX's negative).
int uraIndex(double metres);
int sisaIndex(double metres);

// Read a RINEX 3 navigation file. False if it cannot be opened or is not
// RINEX 3 navigation data; err says why.
bool readNavFile(const std::string& path, NavFile& out, std::string& err);

// The same from text in memory (the tests).
bool parseNav(const std::string& text, NavFile& out, std::string& err);

}  // namespace rinex

#endif  // RAW_REPLAY_RINEX_NAV_H
