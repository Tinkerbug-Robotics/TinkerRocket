// SkyTraqNav.cpp — see SkyTraqNav.h. Pure: no ESP-IDF, host-tested.
#include "SkyTraqNav.h"

#include <cmath>
#include <cstring>

#include "RtcmEphemeris.h"
#include "gnss/types.h"

namespace skytraq
{

namespace
{

constexpr double   SC = GPS_PI_ICD;          // semicircles -> rad, the ICDs' own pi (as RtcmEphemeris.cpp)
constexpr uint32_t kNoIssue = 0xFFFFFFFFu;
constexpr int      kGalWeek0 = 1024;         // GST week 0 = GPS week 1024 (1999-08-22)
constexpr int      kBdtWeek0 = 1356;         // BDT week 0 = GPS week 1356 (2006-01-01)
constexpr double   kHalfWeek = 302400.0;

inline double P2(int n) { return std::ldexp(1.0, n); }   // 2^n; BeiDou's af2 needs 2^-66

// MSB-first bit fields, as the navigation messages pack them (len <= 32).
uint32_t getbitu(const uint8_t* b, size_t pos, int len)
{
    uint32_t v = 0;
    for (int i = 0; i < len; ++i, ++pos)
        v = (v << 1) | ((b[pos >> 3] >> (7 - (pos & 7))) & 1u);
    return v;
}

int32_t signExtend(uint32_t v, int len)
{
    if (len < 32 && (v & (1u << (len - 1))))
        v |= ~0u << len;
    return static_cast<int32_t>(v);
}

int32_t getbits(const uint8_t* b, size_t pos, int len) { return signExtend(getbitu(b, pos, len), len); }

void setbitu(uint8_t* b, size_t pos, int len, uint32_t v)
{
    for (int i = 0; i < len; ++i, ++pos)
    {
        const uint8_t m = static_cast<uint8_t>(0x80u >> (pos & 7));
        if ((v >> (len - 1 - i)) & 1u)
            b[pos >> 3] = static_cast<uint8_t>(b[pos >> 3] | m);
        else
            b[pos >> 3] = static_cast<uint8_t>(b[pos >> 3] & ~m);
    }
}

// CRC-24Q (RTCM's, and Galileo's page CRC): polynomial 0x1864CFB, initial 0.
uint32_t crc24q(const uint8_t* b, size_t n)
{
    uint32_t crc = 0;
    for (size_t k = 0; k < n; ++k)
    {
        crc ^= static_cast<uint32_t>(b[k]) << 16;
        for (int i = 0; i < 8; ++i)
        {
            crc <<= 1;
            if (crc & 0x1000000u) crc ^= 0x1864CFBu;
        }
    }
    return crc & 0xFFFFFFu;
}

// The week toe falls in, from the week of the message's transmit time:
// toe is never more than half a week from it (RTKLIB's handover rule).
int weekOfToe(int week, double toe, double t_transmit)
{
    const double d = toe - t_transmit;
    if (d > kHalfWeek) return week - 1;
    if (d < -kHalfWeek) return week + 1;
    return week;
}

// The key a satellite's issue is told by: IODE (<= 10 bits) and toe (< 2^20 s).
uint32_t issueKey(const gps_eph_t& e)
{
    return (static_cast<uint32_t>(e.iode) << 20) | (static_cast<uint32_t>(e.toe) & 0xFFFFFu);
}

// ── GPS LNAV (IS-GPS-200 20.3.3; positions in the 24-bit-word packing the 0xE0 uses) ──
// sf[k]: subframe k + 1, 30 B. RTKLIB decode_subfrm1-3; gnss_raw.py decode_eph.
NavStatus decodeGps(const uint8_t (&sf)[3][30], int prn, int near_week, gps_eph_t& e)
{
    const uint8_t* b1 = sf[0];
    const uint8_t* b2 = sf[1];
    const uint8_t* b3 = sf[2];
    // A cutover first: the three must be of one issue.
    const uint32_t iodc = (getbitu(b1, 70, 2) << 8) | getbitu(b1, 168, 8);
    const uint32_t iode2 = getbitu(b2, 48, 8);
    const uint32_t iode3 = getbitu(b3, 216, 8);
    if (iode2 != iode3 || iode2 != (iodc & 0xFFu)) return NavStatus::IOD_MISMATCH;
    if (near_week <= 0) return NavStatus::NO_WEEK;

    e.sys = GNSS_SYS_GPS;
    e.prn = prn;
    // Subframe 1
    const double ttr = getbitu(b1, 24, 17) * 6.0 - 6.0;   // HOW: the TOW count is the NEXT subframe's start
    const int week = gnss_raw::fullGpsWeek(static_cast<int>(getbitu(b1, 48, 10)), near_week);
    e.ura = static_cast<int>(getbitu(b1, 60, 4));
    e.health = static_cast<int>(getbitu(b1, 64, 6));
    e.iodc = static_cast<int>(iodc);
    const int32_t tgd = getbits(b1, 160, 8);                // -128 = not available
    e.tgd = tgd == -128 ? 0.0 : tgd * P2(-31);
    e.toc = getbitu(b1, 176, 16) * 16.0;
    e.af2 = getbits(b1, 192, 8) * P2(-55);
    e.af1 = getbits(b1, 200, 16) * P2(-43);
    e.af0 = getbits(b1, 216, 22) * P2(-31);
    // Subframe 2
    e.iode = static_cast<int>(iode2);
    e.crs = getbits(b2, 56, 16) * P2(-5);
    e.delta_n = getbits(b2, 72, 16) * P2(-43) * SC;
    e.m0 = getbits(b2, 88, 32) * P2(-31) * SC;
    e.cuc = getbits(b2, 120, 16) * P2(-29);
    e.e = getbitu(b2, 136, 32) * P2(-33);
    e.cus = getbits(b2, 168, 16) * P2(-29);
    e.sqrt_a = getbitu(b2, 184, 32) * P2(-19);
    e.toe = getbitu(b2, 216, 16) * 16.0;
    // Subframe 3
    e.cic = getbits(b3, 48, 16) * P2(-29);
    e.omega0 = getbits(b3, 64, 32) * P2(-31) * SC;
    e.cis = getbits(b3, 96, 16) * P2(-29);
    e.i0 = getbits(b3, 112, 32) * P2(-31) * SC;
    e.crc = getbits(b3, 144, 16) * P2(-5);
    e.omega = getbits(b3, 160, 32) * P2(-31) * SC;
    e.omega_dot = getbits(b3, 192, 24) * P2(-43) * SC;
    e.idot = getbits(b3, 224, 14) * P2(-43) * SC;
    e.week = weekOfToe(week, e.toe, ttr < 0.0 ? ttr + 604800.0 : ttr);
    e.valid = 1;
    return NavStatus::EPHEMERIS;
}

// ── Galileo I/NAV (OS SIS ICD 4.3.5; w[k]: word type k + 1, 128 bits) ──
// RTKLIB decode_gal_inav_eph; gnss_raw.py decode_gal_inav.
NavStatus decodeGal(const uint8_t (&w)[5][16], int prn, gps_eph_t& e)
{
    const uint8_t* w1 = w[0];
    const uint8_t* w2 = w[1];
    const uint8_t* w3 = w[2];
    const uint8_t* w4 = w[3];
    const uint8_t* w5 = w[4];
    const uint32_t iod = getbitu(w1, 6, 10);
    if (getbitu(w2, 6, 10) != iod || getbitu(w3, 6, 10) != iod || getbitu(w4, 6, 10) != iod)
        return NavStatus::IOD_MISMATCH;
    if (static_cast<int>(getbitu(w4, 16, 6)) != prn) return NavStatus::SVID_MISMATCH;

    e.sys = GNSS_SYS_GAL;
    e.prn = prn;
    e.iode = static_cast<int>(iod);                       // IODnav
    e.iodc = e.iode;
    // Word type 1
    e.toe = getbitu(w1, 16, 14) * 60.0;
    e.m0 = getbits(w1, 30, 32) * P2(-31) * SC;
    e.e = getbitu(w1, 62, 32) * P2(-33);
    e.sqrt_a = getbitu(w1, 94, 32) * P2(-19);
    // Word type 2
    e.omega0 = getbits(w2, 16, 32) * P2(-31) * SC;
    e.i0 = getbits(w2, 48, 32) * P2(-31) * SC;
    e.omega = getbits(w2, 80, 32) * P2(-31) * SC;
    e.idot = getbits(w2, 112, 14) * P2(-43) * SC;
    // Word type 3
    e.omega_dot = getbits(w3, 16, 24) * P2(-43) * SC;
    e.delta_n = getbits(w3, 40, 16) * P2(-43) * SC;
    e.cuc = getbits(w3, 56, 16) * P2(-29);
    e.cus = getbits(w3, 72, 16) * P2(-29);
    e.crc = getbits(w3, 88, 16) * P2(-5);
    e.crs = getbits(w3, 104, 16) * P2(-5);
    e.ura = static_cast<int>(getbitu(w3, 120, 8));       // SISA(E1,E5b)
    // Word type 4
    e.cic = getbits(w4, 22, 16) * P2(-29);
    e.cis = getbits(w4, 38, 16) * P2(-29);
    e.toc = getbitu(w4, 54, 14) * 60.0;
    e.af0 = getbits(w4, 68, 31) * P2(-34);
    e.af1 = getbits(w4, 99, 21) * P2(-46);
    e.af2 = getbits(w4, 120, 6) * P2(-59);
    // Word type 5: ionosphere (6..46), BGD(E1,E5a) 47, BGD(E1,E5b) 57, E5b HS 67,
    // E1-B HS 69, E5b DVS 71, E1-B DVS 72, WN 73, TOW 85
    e.tgd = getbits(w5, 57, 10) * P2(-32);                // BGD(E1,E5b), the I/NAV clock's
    const uint32_t hs = getbitu(w5, 69, 2);
    const uint32_t dvs = getbitu(w5, 72, 1);
    e.health = static_cast<int>((hs << 1) | dvs);
    const int wn = static_cast<int>(getbitu(w5, 73, 12));
    const double tow = getbitu(w5, 85, 20);
    e.week = weekOfToe(wn + kGalWeek0, e.toe, tow);
    e.valid = 1;
    return NavStatus::EPHEMERIS;
}

// ── BeiDou D1 (BDS-SIS-ICD-B1I 5.2.4) ──
// Positions as RTKLIB numbers them, in 30-bit words (word 1: 26 data bits and
// 4 parity, words 2-10: 22 and 8), read from the 0xE2's packing with the
// parity removed: word 1 at 0-25, word k (2-10) at 26 + 22 (k - 2).
size_t d1Bit(size_t pos)
{
    const size_t w = pos / 30, o = pos % 30;
    return w == 0 ? o : 26 + 22 * (w - 1) + o;
}

uint32_t d1u(const uint8_t* b, size_t pos, int len)
{
    uint32_t v = 0;
    for (int i = 0; i < len; ++i)
    {
        const size_t q = d1Bit(pos + static_cast<size_t>(i));
        v = (v << 1) | ((b[q >> 3] >> (7 - (q & 7))) & 1u);
    }
    return v;
}

int32_t d1s(const uint8_t* b, size_t pos, int len) { return signExtend(d1u(b, pos, len), len); }

// A field split over two words (RTKLIB getbitu2 / getbits2).
uint32_t d1u2(const uint8_t* b, size_t p1, int l1, size_t p2, int l2) { return (d1u(b, p1, l1) << l2) | d1u(b, p2, l2); }
int32_t  d1s2(const uint8_t* b, size_t p1, int l1, size_t p2, int l2) { return signExtend(d1u2(b, p1, l1, p2, l2), l1 + l2); }

// sf[k]: subframe k + 1. RTKLIB decode_bds_d1_eph; gnss_raw.py decode_bds_d1.
NavStatus decodeBds(const uint8_t (&sf)[3][28], int prn, gps_eph_t& e)
{
    const uint8_t* s1 = sf[0];
    const uint8_t* s2 = sf[1];
    const uint8_t* s3 = sf[2];
    const uint32_t sow1 = d1u2(s1, 18, 8, 30, 12);
    const uint32_t sow2 = d1u2(s2, 18, 8, 30, 12);
    const uint32_t sow3 = d1u2(s3, 18, 8, 30, 12);
    const uint32_t toc = d1u2(s1, 73, 9, 90, 8);          // 8 s units
    const uint32_t toe = (d1u(s2, 290, 2) << 15) | d1u2(s3, 42, 10, 60, 5);
    // One frame (a 30 s frame never straddles the week's end), and toc = toe
    // ties subframe 1 to 2 and 3, which carry no AODE of their own.
    if (sow2 != sow1 + 6 || sow3 != sow2 + 6 || toc != toe) return NavStatus::FRAME_MISMATCH;

    e.sys = GNSS_SYS_BDS;
    e.prn = prn;
    // Subframe 1
    e.health = static_cast<int>(d1u(s1, 42, 1));          // SatH1
    e.iodc = static_cast<int>(d1u(s1, 43, 5));            // AODC
    e.ura = static_cast<int>(d1u(s1, 48, 4));             // URAI
    const int wn = static_cast<int>(d1u(s1, 60, 13));
    e.toc = toc * 8.0;                                    // BDT
    e.tgd = d1s(s1, 98, 10) * 1e-10;                      // TGD1 (B1I), 0.1 ns
    e.af2 = d1s(s1, 214, 11) * P2(-66);
    e.af0 = d1s2(s1, 225, 7, 240, 17) * P2(-33);
    e.af1 = d1s2(s1, 257, 5, 270, 17) * P2(-50);
    e.iode = static_cast<int>(d1u(s1, 287, 5));           // AODE
    // Subframe 2
    e.delta_n = d1s2(s2, 42, 10, 60, 6) * P2(-43) * SC;
    e.cuc = d1s2(s2, 66, 16, 90, 2) * P2(-31);
    e.m0 = d1s2(s2, 92, 20, 120, 12) * P2(-31) * SC;
    e.e = d1u2(s2, 132, 10, 150, 22) * P2(-33);
    e.cus = d1s(s2, 180, 18) * P2(-31);
    e.crc = d1s2(s2, 198, 4, 210, 14) * P2(-6);
    e.crs = d1s2(s2, 224, 8, 240, 10) * P2(-6);
    e.sqrt_a = d1u2(s2, 250, 12, 270, 20) * P2(-19);
    // Subframe 3
    e.i0 = d1s2(s3, 65, 17, 90, 15) * P2(-31) * SC;
    e.cic = d1s2(s3, 105, 7, 120, 11) * P2(-31);
    e.omega_dot = d1s2(s3, 131, 11, 150, 13) * P2(-43) * SC;
    e.cis = d1s2(s3, 163, 9, 180, 9) * P2(-31);
    e.idot = d1s2(s3, 189, 13, 210, 1) * P2(-43) * SC;
    e.omega0 = d1s2(s3, 211, 21, 240, 11) * P2(-31) * SC;
    e.omega = d1s2(s3, 251, 11, 270, 21) * P2(-31) * SC;
    e.toe = toe * 8.0;                                    // BDT
    e.week = weekOfToe(wn + kBdtWeek0, e.toe, sow1);
    e.valid = 1;
    return NavStatus::EPHEMERIS;
}

// ── GLONASS strings (ICD L1/L2 FDMA 5.1 table 4.5; RTKLIB rcvraw.c decode_glostr) ──
// Each string's 72 data bits as the 0xE1 carries them: bit 0 is the ICD's bit 80.

// A sign-magnitude field (the ICD's: the top bit the sign, 1 negative).
double getbitg(const uint8_t* b, size_t pos, int len)
{
    const double m = static_cast<double>(getbitu(b, pos + 1, len - 1));
    return getbitu(b, pos, 1) ? -m : m;
}

// Strings 1-4 into e (its time and week left to the caller). Returns the
// slot string 4 names.
int decodeGlo(const uint8_t (&s)[4][9], gps_eph_t& e)
{
    std::memset(&e, 0, sizeof(e));
    e.sys = GNSS_SYS_GLO;
    // string 1: -- 2, P1 2, tk 12 (h 5, min 6, 30 s 1), x' 24, x'' 5, x 27
    e.glo_vel[0] = getbitg(s[0], 16, 24) * P2(-20) * 1.0e3;
    e.glo_acc[0] = getbitg(s[0], 40, 5) * P2(-30) * 1.0e3;
    e.glo_pos[0] = getbitg(s[0], 45, 27) * P2(-11) * 1.0e3;
    // string 2: Bn 3, P2 1, tb 7, -- 5, y' 24, y'' 5, y 27
    const uint32_t bn = getbitu(s[1], 0, 3);
    const uint32_t tb = getbitu(s[1], 4, 7);
    e.glo_vel[1] = getbitg(s[1], 16, 24) * P2(-20) * 1.0e3;
    e.glo_acc[1] = getbitg(s[1], 40, 5) * P2(-30) * 1.0e3;
    e.glo_pos[1] = getbitg(s[1], 45, 27) * P2(-11) * 1.0e3;
    // string 3: P3 1, gamma_n 11, -- 1, P 2, l_n 1, z' 24, z'' 5, z 27
    const double gamma_n = getbitg(s[2], 1, 11) * P2(-40);
    const uint32_t ln = getbitu(s[2], 15, 1);
    e.glo_vel[2] = getbitg(s[2], 16, 24) * P2(-20) * 1.0e3;
    e.glo_acc[2] = getbitg(s[2], 40, 5) * P2(-30) * 1.0e3;
    e.glo_pos[2] = getbitg(s[2], 45, 27) * P2(-11) * 1.0e3;
    // string 4: tau_n 22, dtau_n 5, E_n 5, -- 14, P4 1, F_T 4, -- 3, N_T 11, n 5, M 2
    const double tau_n = getbitg(s[3], 0, 22) * P2(-30);
    e.ura = static_cast<int>(getbitu(s[3], 47, 4));          // F_T
    const int slot = static_cast<int>(getbitu(s[3], 65, 5));
    e.af0 = -tau_n;
    e.af1 = gamma_n;
    e.iode = e.iodc = static_cast<int>(tb);
    e.health = static_cast<int>(((bn >> 2) & 1u) | (ln << 1));
    e.toe = static_cast<double>(tb);                          // tb's index until the caller places it
    return slot;
}

// tb (15 min steps of the Moscow day) as continuous GPS seconds: on the UTC
// day that puts it within 12 h of the receiver's time gps_s.
double gloToeGpsS(uint32_t tb, double gps_s)
{
    const double utc = gps_s - kGpsMinusUtcS;
    const double day0 = std::floor(utc / 86400.0) * 86400.0;
    double t = day0 + static_cast<double>(tb) * 900.0 - 10800.0;   // Moscow -> UTC
    if (t < utc - 43200.0) t += 86400.0;
    else if (t > utc + 43200.0) t -= 86400.0;
    return t + kGpsMinusUtcS;
}

}  // namespace

bool isNavMessage(const uint8_t* payload, size_t len)
{
    if (payload == nullptr || len < 1) return false;
    if (payload[0] == kNavIdGps || payload[0] == kNavIdBdsD1 || payload[0] == kNavIdGlo) return true;
    return payload[0] == kNavIdGnss && len >= 3 && payload[2] == kNavGnssGalE1;
}

const char* navStatusName(NavStatus s)
{
    switch (s)
    {
        case NavStatus::PART: return "PART";
        case NavStatus::EPHEMERIS: return "EPHEMERIS";
        case NavStatus::IONO: return "IONO";
        case NavStatus::UNUSED: return "UNUSED";
        case NavStatus::NOT_NAV: return "NOT_NAV";
        case NavStatus::SHORT: return "SHORT";
        case NavStatus::BAD_PRN: return "BAD_PRN";
        case NavStatus::GEO: return "GEO";
        case NavStatus::BAD_PREAMBLE: return "BAD_PREAMBLE";
        case NavStatus::BAD_SUBFRAME: return "BAD_SUBFRAME";
        case NavStatus::BAD_PAGE: return "BAD_PAGE";
        case NavStatus::ALERT_PAGE: return "ALERT_PAGE";
        case NavStatus::CRC_FAIL: return "CRC_FAIL";
        case NavStatus::IOD_MISMATCH: return "IOD_MISMATCH";
        case NavStatus::FRAME_MISMATCH: return "FRAME_MISMATCH";
        case NavStatus::SVID_MISMATCH: return "SVID_MISMATCH";
        case NavStatus::NO_WEEK: return "NO_WEEK";
        case NavStatus::NO_TIME: return "NO_TIME";
        case NavStatus::COUNT: break;
    }
    return "?";
}

void NavAssembler::reset()
{
    std::memset(gps_, 0, sizeof(gps_));
    std::memset(gal_, 0, sizeof(gal_));
    std::memset(bds_, 0, sizeof(bds_));
    std::memset(glo_, 0, sizeof(glo_));
    for (auto& s : gps_) s.last_issue = kNoIssue;
    for (auto& s : gal_) s.last_issue = kNoIssue;
    for (auto& s : bds_) s.last_issue = kNoIssue;
    for (auto& s : glo_) s.last_issue = kNoIssue;
    for (auto& k : glo_k_) k = kGloKUnknown;
    std::memset(&counters_, 0, sizeof(counters_));
    iono_tow_s_ = 0;
    iono_day_ = 0;
    iono_prn_ = 0;
}

void NavAssembler::issued(const gps_eph_t& e, uint32_t& last_issue)
{
    counters_.ephemerides[e.sys]++;
    const uint32_t key = issueKey(e);
    if (key != last_issue) counters_.issues[e.sys]++;
    last_issue = key;
}

NavStatus NavAssembler::add(const uint8_t* payload, size_t len, int near_gps_week, gps_eph_t& eph, int8_t iono[8],
                            double gps_s)
{
    NavStatus st = NavStatus::NOT_NAV;
    if (payload != nullptr && len >= 1)
    {
        switch (payload[0])
        {
            case kNavIdGps:
                counters_.frames[GNSS_SYS_GPS]++;
                st = addGps(payload, len, near_gps_week, eph, iono);
                break;
            case kNavIdBdsD1:
                counters_.frames[GNSS_SYS_BDS]++;
                st = addBds(payload, len, eph);
                break;
            case kNavIdGlo:
                counters_.frames[GNSS_SYS_GLO]++;
                st = addGlo(payload, len, gps_s, eph);
                break;
            case kNavIdGnss:
                if (len < 3)
                    st = NavStatus::SHORT;           // too short to tell its sub-type
                else if (payload[2] == kNavGnssGalE1)
                {
                    counters_.frames[GNSS_SYS_GAL]++;
                    st = addGal(payload, len, eph);
                }
                break;
            default: break;
        }
    }
    counters_.status[static_cast<size_t>(st)]++;
    return st;
}

NavStatus NavAssembler::completeGps(int prn, int near_gps_week, gps_eph_t& eph)
{
    GpsSat& s = gps_[prn - 1];
    gps_eph_t e;
    std::memset(&e, 0, sizeof(e));
    const NavStatus st = decodeGps(s.sf, prn, near_gps_week, e);
    s.pending = st == NavStatus::NO_WEEK ? 1 : 0;
    if (st != NavStatus::EPHEMERIS) return st;
    issued(e, s.last_issue);
    eph = e;
    return st;
}

NavStatus NavAssembler::addGps(const uint8_t* p, size_t len, int near_gps_week, gps_eph_t& eph, int8_t iono[8])
{
    if (len < kNavGpsLen) return NavStatus::SHORT;
    const int prn = p[1];
    if (prn < 1 || prn > kGpsPrns) return NavStatus::BAD_PRN;   // QZSS (193-202) comes here too
    const uint8_t* sb = p + 3;
    if (sb[0] != 0x8B) return NavStatus::BAD_PREAMBLE;
    const int sfid = static_cast<int>(getbitu(sb, 43, 3));      // HOW bits 20-22
    if (sfid < 1 || sfid > 5) return NavStatus::BAD_SUBFRAME;
    GpsSat& s = gps_[prn - 1];
    if (sfid <= 3)
    {
        std::memcpy(s.sf[sfid - 1], sb, sizeof(s.sf[0]));
        s.have = static_cast<uint8_t>(s.have | (1u << (sfid - 1)));
        if (sfid != 3 || (s.have & 0x07) != 0x07) return NavStatus::PART;
        return completeGps(prn, near_gps_week, eph);
    }
    // Subframe 4 page 18 (SV ID 56, word 3 bits 2-7): the Klobuchar alphas
    // and betas, eight bytes from word 3 bit 8 on.
    if (sfid == 4 && getbitu(sb, 50, 6) == 56)
    {
        for (int k = 0; k < 8; ++k)
            iono[k] = static_cast<int8_t>(getbits(sb, 56 + 8 * static_cast<size_t>(k), 8));
        // The HOW's TOW count is the next subframe's start, so just after
        // this one arrived: the receiver's current time of week.
        iono_tow_s_ = getbitu(sb, 24, 17) * 6u;
        iono_day_ = near_gps_week > 0 ? static_cast<uint16_t>(near_gps_week * 7 + static_cast<int>(iono_tow_s_ / 86400u))
                                      : 0;
        iono_prn_ = static_cast<uint8_t>(prn);
        counters_.iono++;
        return NavStatus::IONO;
    }
    return NavStatus::UNUSED;
}

NavStatus NavAssembler::takePending(int near_gps_week, gps_eph_t& eph)
{
    for (int prn = 1; prn <= kGpsPrns; ++prn)
    {
        if (!gps_[prn - 1].pending) continue;
        if (near_gps_week <= 0) return NavStatus::NO_WEEK;
        return completeGps(prn, near_gps_week, eph);
    }
    return NavStatus::PART;
}

NavStatus NavAssembler::addGal(const uint8_t* p, size_t len, gps_eph_t& eph)
{
    if (len < kNavGalLen) return NavStatus::SHORT;
    const int prn = p[3];
    if (prn < 1 || prn > kGalPrns) return NavStatus::BAD_PRN;
    // The page pair: the even page's 120 bits in bits 0-127, the odd page's in 128-255.
    const uint8_t* buf = p + 5;
    if (getbitu(buf, 0, 1) != 0 || getbitu(buf, 128, 1) != 1) return NavStatus::BAD_PAGE;
    if (getbitu(buf, 1, 1) || getbitu(buf, 129, 1)) return NavStatus::ALERT_PAGE;
    // CRC-24Q over the even page's first 114 bits and the odd page's first 82,
    // after 4 zero bits to make 25 bytes; the CRC follows at odd bit 82.
    uint8_t c[26] = {};
    for (int i = 0; i < 15; ++i)
        setbitu(c, 4 + 8 * static_cast<size_t>(i), 8, getbitu(buf, 8 * static_cast<size_t>(i), 8));
    for (int i = 0; i < 11; ++i)
        setbitu(c, 118 + 8 * static_cast<size_t>(i), 8, getbitu(buf, 128 + 8 * static_cast<size_t>(i), 8));
    if (crc24q(c, 25) != getbitu(buf, 128 + 82, 24)) return NavStatus::CRC_FAIL;
    // The 128-bit word: the even page's 112 data bits (2-113) and the odd page's 16 (130-145).
    const uint32_t wtype = getbitu(buf, 2, 6);
    if (wtype < 1 || wtype > 5) return NavStatus::UNUSED;
    GalSat& s = gal_[prn - 1];
    uint8_t* w = s.word[wtype - 1];
    for (int i = 0; i < 14; ++i) w[i] = static_cast<uint8_t>(getbitu(buf, 2 + 8 * static_cast<size_t>(i), 8));
    for (int i = 14; i < 16; ++i)
        w[i] = static_cast<uint8_t>(getbitu(buf, 130 + 8 * static_cast<size_t>(i - 14), 8));
    s.have = static_cast<uint8_t>(s.have | (1u << (wtype - 1)));
    if (wtype != 5 || (s.have & 0x1F) != 0x1F) return NavStatus::PART;
    gps_eph_t e;
    std::memset(&e, 0, sizeof(e));
    const NavStatus st = decodeGal(s.word, prn, e);
    if (st != NavStatus::EPHEMERIS) return st;
    issued(e, s.last_issue);
    eph = e;
    return st;
}

NavStatus NavAssembler::addBds(const uint8_t* p, size_t len, gps_eph_t& eph)
{
    if (len < kNavBdsLen) return NavStatus::SHORT;
    const int prn = static_cast<int>(p[1]) - 200;
    if (prn < 1 || prn > 63) return NavStatus::BAD_PRN;
    if (prn < kBdsFirst || prn > kBdsLast) return NavStatus::GEO;
    const uint8_t* d = p + 3;
    if (getbitu(d, 0, 11) != 0x712) return NavStatus::BAD_PREAMBLE;
    const int sfid = p[2];
    if (sfid < 1 || sfid > 5 || static_cast<int>(getbitu(d, 15, 3)) != sfid) return NavStatus::BAD_SUBFRAME;
    if (sfid > 3) return NavStatus::UNUSED;
    BdsSat& s = bds_[prn - kBdsFirst];
    std::memcpy(s.sf[sfid - 1], d, sizeof(s.sf[0]));
    s.have = static_cast<uint8_t>(s.have | (1u << (sfid - 1)));
    if (sfid != 3 || (s.have & 0x07) != 0x07) return NavStatus::PART;
    gps_eph_t e;
    std::memset(&e, 0, sizeof(e));
    const NavStatus st = decodeBds(s.sf, prn, e);
    if (st != NavStatus::EPHEMERIS) return st;
    issued(e, s.last_issue);
    eph = e;
    return st;
}

void NavAssembler::noteGloFreq(int slot, int k)
{
    if (slot >= 1 && slot <= kGloSlots && k >= -7 && k <= 6)
        glo_k_[slot] = static_cast<int8_t>(k);
}

NavStatus NavAssembler::addGlo(const uint8_t* p, size_t len, double gps_s, gps_eph_t& eph)
{
    if (len < kNavGloLen) return NavStatus::SHORT;
    const int slot = p[1] > 64 ? p[1] - 64 : p[1];
    if (slot < 1 || slot > kGloSlots) return NavStatus::BAD_PRN;
    const int m = p[2];
    if (m == 0) return NavStatus::BAD_SUBFRAME;
    if (m > 4) return NavStatus::UNUSED;                      // string 5 and the almanac's
    GloSat& g = glo_[slot - 1];
    std::memcpy(g.str[m - 1], p + 3, 9);
    g.at[m - 1] = gps_s;
    g.have |= static_cast<uint8_t>(1u << (m - 1));
    if (m != 4 || (g.have & 0x0F) != 0x0F) return NavStatus::PART;
    if (!(gps_s > 0.0)) return NavStatus::NO_TIME;
    for (int k = 0; k < 3; ++k)
        if (!(g.at[k] > 0.0) || std::fabs(gps_s - g.at[k]) > kGloFrameS) return NavStatus::FRAME_MISMATCH;
    gps_eph_t e;
    if (decodeGlo(g.str, e) != slot) return NavStatus::SVID_MISMATCH;
    const double toe = gloToeGpsS(static_cast<uint32_t>(e.toe), gps_s);
    e.week = static_cast<int>(std::floor(toe / 604800.0));
    e.toe = e.toc = toe - e.week * 604800.0;
    e.prn = slot;
    e.glo_k = glo_k_[slot] == kGloKUnknown ? 0 : glo_k_[slot];
    e.valid = 1;
    issued(e, g.last_issue);
    eph = e;
    return NavStatus::EPHEMERIS;
}

}  // namespace skytraq
