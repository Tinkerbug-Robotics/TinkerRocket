// RtcmEphemeris.cpp — see RtcmEphemeris.h
#include "RtcmEphemeris.h"

#include <cmath>
#include <cstring>

#include "gnss/types.h"

namespace gnss_raw
{

namespace
{

// MSB-first bit fields, as RTCM packs them.
uint32_t getbitu(const uint8_t* b, size_t pos, int len)
{
    uint32_t v = 0;
    for (int i = 0; i < len; ++i, ++pos)
        v = (v << 1) | ((b[pos >> 3] >> (7 - (pos & 7))) & 1u);
    return v;
}

int32_t getbits(const uint8_t* b, size_t pos, int len)
{
    uint32_t v = getbitu(b, pos, len);
    if (len < 32 && (v & (1u << (len - 1))))
        v |= ~0u << len;                       // sign-extend
    return static_cast<int32_t>(v);
}

// Reads fields in order from a payload, so each decoder below lists its
// message's layout top to bottom, as the standard tabulates it.
struct Reader
{
    const uint8_t* b;
    size_t         pos;
    uint32_t u(int len) { uint32_t v = getbitu(b, pos, len); pos += len; return v; }
    int32_t  s(int len) { int32_t v = getbits(b, pos, len); pos += len; return v; }
    void     skip(int len) { pos += len; }
};

inline double P2(int n) { return std::ldexp(1.0, n); }    // 2^n; BeiDou's af2 needs 2^-66
constexpr double SC = GPS_PI_ICD;              // semicircles -> rad, the ICDs' own pi

// 1019: GPS LNAV ephemeris, 488 bits.
bool decode1019(const uint8_t* p, int near_week, gps_eph_t& e)
{
    Reader r{p, 12};
    e.sys = GNSS_SYS_GPS;
    e.prn = static_cast<int>(r.u(6));                      // DF009
    e.week = fullGpsWeek(static_cast<int>(r.u(10)), near_week);  // DF076
    e.ura = static_cast<int>(r.u(4));                      // DF077
    r.skip(2);                                             // DF078 L2 codes
    e.idot = r.s(14) * P2(-43) * SC;                       // DF079
    e.iode = static_cast<int>(r.u(8));                     // DF071
    e.toc = r.u(16) * 16.0;                                // DF081
    e.af2 = r.s(8) * P2(-55);                              // DF082
    e.af1 = r.s(16) * P2(-43);                             // DF083
    e.af0 = r.s(22) * P2(-31);                             // DF084
    e.iodc = static_cast<int>(r.u(10));                    // DF085
    e.crs = r.s(16) * P2(-5);                              // DF086
    e.delta_n = r.s(16) * P2(-43) * SC;                    // DF087
    e.m0 = r.s(32) * P2(-31) * SC;                         // DF088
    e.cuc = r.s(16) * P2(-29);                             // DF089
    e.e = r.u(32) * P2(-33);                               // DF090
    e.cus = r.s(16) * P2(-29);                             // DF091
    e.sqrt_a = r.u(32) * P2(-19);                          // DF092
    e.toe = r.u(16) * 16.0;                                // DF093
    e.cic = r.s(16) * P2(-29);                             // DF094
    e.omega0 = r.s(32) * P2(-31) * SC;                     // DF095
    e.cis = r.s(16) * P2(-29);                             // DF096
    e.i0 = r.s(32) * P2(-31) * SC;                         // DF097
    e.crc = r.s(16) * P2(-5);                              // DF098
    e.omega = r.s(32) * P2(-31) * SC;                      // DF099
    e.omega_dot = r.s(24) * P2(-43) * SC;                  // DF100
    const int32_t tgd = r.s(8);                            // DF101; -128 = not available
    e.tgd = tgd == -128 ? 0.0 : tgd * P2(-31);
    e.health = static_cast<int>(r.u(6));                   // DF102
    return true;
}

// 1046: Galileo I/NAV ephemeris, 504 bits.
bool decode1046(const uint8_t* p, gps_eph_t& e)
{
    Reader r{p, 12};
    e.sys = GNSS_SYS_GAL;
    e.prn = static_cast<int>(r.u(6));                      // DF252
    e.week = static_cast<int>(r.u(12)) + 1024;             // DF289 GST week; GST week 0 = GPS week 1024
    e.iode = static_cast<int>(r.u(10));                    // DF290 IODnav
    e.iodc = e.iode;
    e.ura = static_cast<int>(r.u(8));                      // DF286 SISA
    e.idot = r.s(14) * P2(-43) * SC;                       // DF292
    e.toc = r.u(14) * 60.0;                                // DF293
    e.af2 = r.s(6) * P2(-59);                              // DF294
    e.af1 = r.s(21) * P2(-46);                             // DF295
    e.af0 = r.s(31) * P2(-34);                             // DF296
    e.crs = r.s(16) * P2(-5);                              // DF297
    e.delta_n = r.s(16) * P2(-43) * SC;                    // DF298
    e.m0 = r.s(32) * P2(-31) * SC;                         // DF299
    e.cuc = r.s(16) * P2(-29);                             // DF300
    e.e = r.u(32) * P2(-33);                               // DF301
    e.cus = r.s(16) * P2(-29);                             // DF302
    e.sqrt_a = r.u(32) * P2(-19);                          // DF303
    e.toe = r.u(14) * 60.0;                                // DF304
    e.cic = r.s(16) * P2(-29);                             // DF305
    e.omega0 = r.s(32) * P2(-31) * SC;                     // DF306
    e.cis = r.s(16) * P2(-29);                             // DF307
    e.i0 = r.s(32) * P2(-31) * SC;                         // DF308
    e.crc = r.s(16) * P2(-5);                              // DF309
    e.omega = r.s(32) * P2(-31) * SC;                      // DF310
    e.omega_dot = r.s(24) * P2(-43) * SC;                  // DF311
    r.skip(10);                                            // DF312 BGD(E1,E5a)
    e.tgd = r.s(10) * P2(-32);                             // DF313 BGD(E1,E5b), the I/NAV clock's
    r.skip(2 + 1);                                         // DF316 E5b health, DF317 E5b validity
    const uint32_t hs = r.u(2);                            // DF287 E1-B signal health
    const uint32_t dvs = r.u(1);                           // DF288 E1-B data validity
    e.health = static_cast<int>((hs << 1) | dvs);
    return true;
}

// 1042: BeiDou D1 ephemeris, 511 bits.
bool decode1042(const uint8_t* p, gps_eph_t& e)
{
    Reader r{p, 12};
    e.sys = GNSS_SYS_BDS;
    e.prn = static_cast<int>(r.u(6));                      // DF488
    if (e.prn <= 5 || e.prn >= 59)
        return false;                                      // GEO: D2 orbit, not placed by gnss/eph.h
    e.week = static_cast<int>(r.u(13)) + 1356;             // DF489 BDT week; BDT week 0 = GPS week 1356
    e.ura = static_cast<int>(r.u(4));                      // DF490 URAI
    e.idot = r.s(14) * P2(-43) * SC;                       // DF491
    e.iode = static_cast<int>(r.u(5));                     // DF492 AODE
    e.toc = r.u(17) * 8.0;                                 // DF493, BDT
    e.af2 = r.s(11) * P2(-66);                             // DF494
    e.af1 = r.s(22) * P2(-50);                             // DF495
    e.af0 = r.s(24) * P2(-33);                             // DF496
    e.iodc = static_cast<int>(r.u(5));                     // DF497 AODC
    e.crs = r.s(18) * P2(-6);                              // DF498
    e.delta_n = r.s(16) * P2(-43) * SC;                    // DF499
    e.m0 = r.s(32) * P2(-31) * SC;                         // DF500
    e.cuc = r.s(18) * P2(-31);                             // DF501
    e.e = r.u(32) * P2(-33);                               // DF502
    e.cus = r.s(18) * P2(-31);                             // DF503
    e.sqrt_a = r.u(32) * P2(-19);                          // DF504
    e.toe = r.u(17) * 8.0;                                 // DF505, BDT
    e.cic = r.s(18) * P2(-31);                             // DF506
    e.omega0 = r.s(32) * P2(-31) * SC;                     // DF507
    e.cis = r.s(18) * P2(-31);                             // DF508
    e.i0 = r.s(32) * P2(-31) * SC;                         // DF509
    e.crc = r.s(18) * P2(-6);                              // DF510
    e.omega = r.s(32) * P2(-31) * SC;                      // DF511
    e.omega_dot = r.s(24) * P2(-43) * SC;                  // DF512
    e.tgd = r.s(10) * 1e-10;                               // DF513 TGD1, B1I
    r.skip(10);                                            // DF514 TGD2
    e.health = static_cast<int>(r.u(1));                   // DF515 SatH1
    return true;
}

}  // namespace

uint16_t rtcmMessageType(const uint8_t* payload, size_t len)
{
    if (payload == nullptr || len < 2)
        return 0;
    return static_cast<uint16_t>(getbitu(payload, 0, 12));
}

int fullGpsWeek(int week_mod_1024, int near_gps_week)
{
    int w = week_mod_1024 + (near_gps_week - week_mod_1024 + 512) / 1024 * 1024;
    return w < 0 ? week_mod_1024 : w;
}

bool decodeEphemeris(const uint8_t* payload, size_t len, int near_gps_week, gps_eph_t& out)
{
    const uint16_t type = rtcmMessageType(payload, len);
    gps_eph_t e;
    std::memset(&e, 0, sizeof(e));
    bool ok = false;
    switch (type)
    {
        case 1019: ok = len * 8 >= 488 && decode1019(payload, near_gps_week, e); break;
        case 1046: ok = len * 8 >= 504 && decode1046(payload, e); break;
        case 1042: ok = len * 8 >= 511 && decode1042(payload, e); break;
        default: break;
    }
    if (!ok)
        return false;
    e.valid = 1;
    out = e;
    return true;
}

}  // namespace gnss_raw
