// SkyTraqNav.h — the PX1105R's forwarded navigation data (0xE0 GPS LNAV subframes, 0xE2 BeiDou D1 subframes, 0xE6 Galileo I/NAV page pairs) assembled into gps_eph_t ephemerides and the GPS Klobuchar ionosphere (pure, host-compilable)
//
// Unlike the LC86G (RTCM 1019/1046/1042) and the mosaic-G5 (GPSNav/GALNav/
// BDSNav), the PX1105R does not send decoded ephemerides: it forwards each
// satellite's navigation message as it demodulates it, one subframe (GPS,
// BeiDou) or one page pair (Galileo) at a time, parity removed. This file
// keeps the pieces per satellite and decodes an ephemeris when the last one
// arrives, by the layouts of the bench prototype that has placed the
// PX1105R's satellites on real data since 2026-09
// (gnss/sim/tinkerrocket_gnss/gnss_raw.py: EphStore, decode_eph,
// decode_gal_inav, decode_bds_d1; those follow RTKLIB rcvraw.c/skytraq.c).
//
// The ephemerides follow the conventions RtcmEphemeris.h uses, so the same
// EphemerisStore and RawMeasBuilder take them:
//   - week: the full GPS week of toe. GPS from the broadcast 10-bit week,
//     made whole against the caller's week (gnss_raw::fullGpsWeek); Galileo
//     GST week + 1024; BeiDou BDT week + 1356. Each is then moved to the
//     week toe falls in, by toe against the message's own transmit time
//     (RTKLIB's handover rule): an ephemeris sent late on a Saturday can
//     have its toe in the next week;
//   - toe/toc in the system's own time: GPS and Galileo GPS seconds of week
//     (GST's track GPS's), BeiDou BDT (14 s behind);
//   - radians (semicircles * GPS_PI_ICD);
//   - iode/iodc: GPS IODE and the 10-bit IODC; Galileo IODnav in both;
//     BeiDou AODE and AODC;
//   - ura: GPS URA index, Galileo SISA, BeiDou URAI (as broadcast);
//   - tgd: GPS T_GD, Galileo BGD(E1,E5b) (the I/NAV clock's), BeiDou TGD1
//     (B1I): the group delay of the one signal of each system the filter uses;
//   - health 0 = usable: GPS the 6-bit SV health, Galileo
//     (E1-B HS << 1) | E1-B DVS, BeiDou SatH1.
//
// When an ephemeris completes (09-26, from each satellite's first frame):
//   - GPS on subframe 3, once subframes 1 and 2 are held: the first after
//     24-42 s, then every 30 s (one frame);
//   - Galileo on word type 5, once word types 1-4 are held: the first after
//     32 s, then every 30 s (one I/NAV subframe on E1-B);
//   - BeiDou on subframe 3, when subframes 1, 2 and 3 are of one 30 s frame
//     (their seconds of week 6 s apart): the first after 36 s for a strong
//     satellite, but 2-8 min for a weak one, which seldom gives three clean
//     subframes in a row (C11 348 s, C27 474 s).
// Each completion returns the ephemeris again; EphemerisStore::add ignores a
// repeat (same IODE and toe). counters().issues counts the new ones. A new
// issue: GPS every 2 h, Galileo every 10 min, BeiDou every hour.
//
// What is refused, and why:
//   - GPS at a cutover: IODE (subframe 2), IODE (subframe 3) and IODC's low
//     byte (subframe 1) disagree, so the subframes are of two issues; the
//     next frame of the new issue completes it;
//   - a GPS ephemeris while the caller's week is unknown (near_gps_week 0):
//     it is held, and takePending() gives it once the week is known;
//   - a Galileo page pair that fails its CRC-24Q, an alert page, or one not
//     an even page followed by an odd one; words 1-4 of different IODnav;
//     word 4's SVID not the satellite's;
//   - BeiDou GEOs (C01-C05, C59-C63): they send D2, which gnss/eph.h cannot
//     place (RTKLIB has D2 as 0xE3, not taken). The 0xE2's PRN + 200 in a
//     byte cannot number C56-C63 at all; none came on 09-26;
//   - BeiDou subframes 1-3 not of one frame, or toc != toe (the prototype's
//     same-issue check: only subframe 1 carries the AODE).
//
// Message layouts (as the prototype and RTKLIB skytraq.c read them; every
// frame of the 09-26 capture, bench-backups/px1105r_outdoor_20hz_1200s_
// 2026-09-26, agrees, down to the constant bytes):
//   0xE0 (33 B)  [1] PRN  [2] subframe ID  [3..32] words 1-10, 24 data bits
//                each (parity removed, MSB first): TLM (preamble 0x8B), HOW
//                (TOW count, subframe ID), words 3-10
//   0xE2 (31 B)  [1] PRN + 200  [2] subframe ID  [3..30] word 1's 26 data
//                bits then words 2-10's 22 each (224 bits, parity removed)
//   0xE6 (37 B)  [1] 1  [2] GNSS type (bits 0-3) | signal type (bits 4-7)
//                [3] SVID  [4] 8 (dwords)  [5..36] an I/NAV page pair as 8
//                big-endian dwords: the even page's 120 bits and the odd
//                page's 120 bits, each in 128. Only [2] = 0x03 is taken:
//                Galileo (3) E1-B (0), the I/NAV the prototype decodes; the
//                PX1105R also tracks E5a, whose F/NAV pages are another format
//                (none came on 09-26: all 3121 0xE6 were 0x03).
// The PRN/SVID byte, the subframe ID and the Galileo word type are read from
// the message; the GPS subframe ID from the HOW (it equals [2] on 09-26).
//
// Pinned by tests_cpp/test_skytraq_nav.cpp against the prototype's decode of
// the 09-26 capture. Needs TR_GNSSRaw (gnss/eph.h, RtcmEphemeris.h).
#ifndef SKYTRAQ_NAV_H
#define SKYTRAQ_NAV_H

#include <cstddef>
#include <cstdint>

#include "gnss/eph.h"

namespace skytraq
{

// The message IDs this file takes (SkyTraqBinary.h's MSG_GPS_SUBFRAME,
// MSG_BDS_D1, MSG_GAL_PAGE), the 0xE6 sub-type, and each message's length.
static constexpr uint8_t kNavIdGps     = 0xE0;
static constexpr uint8_t kNavIdBdsD1   = 0xE2;
static constexpr uint8_t kNavIdGnss    = 0xE6;
static constexpr uint8_t kNavGnssGalE1 = 0x03;   // 0xE6 [2]: Galileo, E1-B (I/NAV)
static constexpr size_t  kNavGpsLen    = 33;
static constexpr size_t  kNavBdsLen    = 31;
static constexpr size_t  kNavGalLen    = 37;

// A body (from its message ID on, after A0 A1 and the length, before the
// checksum) that NavAssembler::add takes: an 0xE0, an 0xE2, or an 0xE6 of
// sub-type 0x03 (Galileo I/NAV on E1-B). Lengths are add()'s to check.
bool isNavMessage(const uint8_t* payload, size_t len);

enum class NavStatus : uint8_t
{
    PART,            // filed toward an ephemeris; nothing complete yet
    EPHEMERIS,       // eph holds a complete ephemeris
    IONO,            // iono[8] holds the GPS Klobuchar parameters (subframe 4 page 18)
    UNUSED,          // a good message with nothing needed here: GPS subframes 4-5 but page 18,
                     //   Galileo word types 0 and 6-63, BeiDou subframes 4-5
    NOT_NAV,         // not an 0xE0, 0xE2 or 0xE6 of sub-type 0x03
    SHORT,           // shorter than its message's length
    BAD_PRN,         // outside GPS 1-32, Galileo 1-36, BeiDou 1-63
    GEO,             // a BeiDou GEO (1-5, 59-63)
    BAD_PREAMBLE,    // GPS TLM not 0x8B; BeiDou word 1 not 11100010010
    BAD_SUBFRAME,    // GPS HOW subframe ID not 1-5; BeiDou [2] not 1-5, or not word 1's FraID
    BAD_PAGE,        // Galileo: not an even page then an odd one
    ALERT_PAGE,      // Galileo alert page
    CRC_FAIL,        // Galileo CRC-24Q
    IOD_MISMATCH,    // GPS IODE2, IODE3 and IODC & 0xFF disagree; Galileo IODnav of words 1-4 differ
    FRAME_MISMATCH,  // BeiDou subframes 1-3 not of one frame, or toc != toe
    SVID_MISMATCH,   // Galileo word 4's SVID is not the page's satellite
    NO_WEEK,         // GPS complete but near_gps_week unknown (<= 0): held for takePending()
    COUNT
};

// A short name for a status line ("EPHEMERIS", "CRC_FAIL", ...).
const char* navStatusName(NavStatus s);

// Counters for a status line. Indices of the per-system arrays are
// gnss_sys_t: 0 GPS, 1 Galileo, 2 BeiDou.
struct NavCounters
{
    uint32_t frames[3];       // messages offered of each system (an 0xE0, 0xE6 sub-type 0x03, 0xE2)
    uint32_t ephemerides[3];  // complete ephemerides returned, a repeat of an issue included
    uint32_t issues[3];       //   of which a new issue (IODE, toe) for its satellite
    uint32_t iono;            // ionosphere pages returned
    uint32_t status[static_cast<size_t>(NavStatus::COUNT)];   // every add() result, by NavStatus
};

class NavAssembler
{
public:
    // Satellites kept, each in its own fixed slot (no heap): GPS 1-32,
    // Galileo 1-36, BeiDou 6-58 (MEO and IGSO).
    static constexpr int kGpsPrns  = 32;
    static constexpr int kGalPrns  = 36;
    static constexpr int kBdsFirst = 6;
    static constexpr int kBdsLast  = 58;

    NavAssembler() { reset(); }

    // Forget every satellite's pieces and the counters (after a receiver restart).
    void reset();

    // File one checksum-valid body (from its message ID on). `near_gps_week`
    // is the receiver's current full GPS week (0xE5 [3..4]); 0 = not yet
    // known, which holds a completed GPS ephemeris back as NO_WEEK (its
    // subframes are kept: takePending() or the satellite's next subframe 3
    // completes it). Galileo and BeiDou carry their own week and do not need
    // it. On EPHEMERIS, eph is complete (valid = 1); on IONO, iono[8] holds
    // the 8 broadcast bytes alpha0..3, beta0..3 (the raw integers, as
    // mosaic::decodeGpsIon gives them: scale 2^-30, 2^-27, 2^-24, 2^-24,
    // 2^11, 2^14, 2^16, 2^16) and ionoGpsDay() the day they came. Neither is
    // touched otherwise. A page 18 comes from every GPS satellite tracked,
    // once per 12.5 min frame; satellites uploaded at different times can
    // send different sets (on 09-26, G15/G23 one set, G18/G24/G32 another).
    NavStatus add(const uint8_t* payload, size_t len, int near_gps_week, gps_eph_t& eph, int8_t iono[8]);

    // A GPS ephemeris held as NO_WEEK, now that the week is known: EPHEMERIS
    // with eph filled for one satellite (call again for the next), NO_WEEK
    // while near_gps_week is still unknown, PART when none is held, or
    // IOD_MISMATCH if the satellite's subframes have since become two issues
    // (its next frame completes it the normal way).
    NavStatus takePending(int near_gps_week, gps_eph_t& eph);

    // The GPS day (days since 1980-01-06) the last IONO came on, from the
    // subframe's HOW time and the near_gps_week given with it; 0 if that
    // week was unknown or no IONO yet. Its HOW time of week (s) and PRN.
    uint16_t ionoGpsDay() const { return iono_day_; }
    uint32_t ionoTowS() const { return iono_tow_s_; }
    int      ionoPrn() const { return iono_prn_; }

    const NavCounters& counters() const { return counters_; }

private:
    // GPS: subframes 1-3 as the 0xE0 carries them (30 B, 10 words x 24 bits).
    struct GpsSat
    {
        uint32_t last_issue;     // (IODE << 20) | toe of the last ephemeris returned; kNoIssue none
        uint8_t  sf[3][30];
        uint8_t  have;           // bit k: subframe k + 1 held
        uint8_t  pending;        // completed while the week was unknown
    };
    // Galileo: I/NAV word types 1-5, 128 bits each (the even page's 112 data
    // bits and the odd page's 16).
    struct GalSat
    {
        uint32_t last_issue;
        uint8_t  word[5][16];
        uint8_t  have;           // bit k: word type k + 1 held
    };
    // BeiDou D1: subframes 1-3 as the 0xE2 carries them (224 bits).
    struct BdsSat
    {
        uint32_t last_issue;
        uint8_t  sf[3][28];
        uint8_t  have;
    };

    NavStatus addGps(const uint8_t* p, size_t len, int near_gps_week, gps_eph_t& eph, int8_t iono[8]);
    NavStatus addGal(const uint8_t* p, size_t len, gps_eph_t& eph);
    NavStatus addBds(const uint8_t* p, size_t len, gps_eph_t& eph);
    NavStatus completeGps(int prn, int near_gps_week, gps_eph_t& eph);
    // Count a completed ephemeris, and whether it is a new issue for its satellite.
    void issued(const gps_eph_t& e, uint32_t& last_issue);

    GpsSat      gps_[kGpsPrns];
    GalSat      gal_[kGalPrns];
    BdsSat      bds_[kBdsLast - kBdsFirst + 1];
    NavCounters counters_;
    uint32_t    iono_tow_s_ = 0;
    uint16_t    iono_day_ = 0;
    uint8_t     iono_prn_ = 0;
};

}  // namespace skytraq

#endif  // SKYTRAQ_NAV_H
