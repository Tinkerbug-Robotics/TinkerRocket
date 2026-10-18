// SbfNav.cpp — see SbfNav.h. Pure: no ESP-IDF, host-tested.
#include "SbfNav.h"

#include <cmath>
#include <cstring>

#include "RtcmEphemeris.h"
#include "SbfFields.h"
#include "SbfMeas.h"
#include "gnss/types.h"

namespace mosaic
{

using sbf::u16;
using sbf::u32;

namespace
{

constexpr double SC = GPS_PI_ICD;   // semicircles -> rad, the ICDs' own pi (as RtcmEphemeris.cpp)

// The documented revision-0 lengths: every field these decoders read is inside.
constexpr uint16_t kGpsNavLen = 140;
constexpr uint16_t kGalNavLen = 149;
constexpr uint16_t kBdsNavLen = 140;
constexpr uint16_t kGpsIonLen = 48;

constexpr int kBdtWeek0 = 1356;   // BDT week 0 = GPS week 1356 (2006-01-01)

// Floats read at their offsets, noting whether any is do-not-use.
struct Floats
{
    const uint8_t* b;
    bool ok = true;
    double f4(size_t at)
    {
        const double x = sbf::f32(b + at);
        ok = ok && !sbf::dnu(x);
        return x;
    }
    double f8(size_t at)
    {
        const double x = sbf::f64(b + at);
        ok = ok && !sbf::dnu(x);
        return x;
    }
};

// A week sent modulo `modulus`, made whole as the one nearest `near`.
int unwrapWeek(int week_mod, int modulus, int near)
{
    const int k = static_cast<int>(std::floor((near - week_mod) / static_cast<double>(modulus) + 0.5));
    return week_mod + k * modulus;
}

// GPSNav and BDSNav share their layout from t_oc (32) to IDOT (132).
void keplerGpsLayout(Floats& f, const uint8_t* b, gps_eph_t& e)
{
    e.toc = u32(b + 32);
    e.af2 = f.f4(36);
    e.af1 = f.f4(40);
    e.af0 = f.f4(44);
    e.crs = f.f4(48);
    e.delta_n = f.f4(52) * SC;
    e.m0 = f.f8(56) * SC;
    e.cuc = f.f4(64);
    e.e = f.f8(68);
    e.cus = f.f4(76);
    e.sqrt_a = f.f8(80);
    e.toe = u32(b + 88);
    e.cic = f.f4(92);
    e.omega0 = f.f8(96) * SC;
    e.cis = f.f4(104);
    e.i0 = f.f8(108) * SC;
    e.crc = f.f4(116);
    e.omega = f.f8(120) * SC;
    e.omega_dot = f.f4(128) * SC;
    e.idot = f.f4(132) * SC;
}

// GPSNav 5891: GPS LNAV subframes 1-3.
SbfEphStatus decodeGpsNav(const uint8_t* b, int ref_week, gps_eph_t& e)
{
    const uint8_t prn = b[14];
    if (prn < 1 || prn > 37) return SbfEphStatus::BAD_SVID;
    const uint16_t iodc = u16(b + 22);
    const uint8_t iode2 = b[24], iode3 = b[25];
    if (iode2 != iode3 || iode2 != (iodc & 0xFF)) return SbfEphStatus::IOD_MISMATCH;
    Floats f{b};
    e.sys = GNSS_SYS_GPS;
    e.prn = prn;
    e.week = gnss_raw::fullGpsWeek(u16(b + 138) & 0x3FF, ref_week);   // WNt_oe, mod 1024
    e.ura = b[19];
    e.health = b[20] & 0x3F;
    e.iodc = iodc;
    e.iode = iode2;
    e.tgd = f.f4(28);                                                   // T_gd
    keplerGpsLayout(f, b, e);
    return f.ok ? SbfEphStatus::OK : SbfEphStatus::DNU_FIELD;
}

// GALNav 4002: Galileo I/NAV (Source 2) only.
SbfEphStatus decodeGalNav(const uint8_t* b, int ref_week, gps_eph_t& e)
{
    const uint8_t svid = b[14];
    if (svid < 71 || svid > 106) return SbfEphStatus::BAD_SVID;
    if (b[15] != 2) return SbfEphStatus::FNAV_SKIPPED;
    Floats f{b};
    e.sys = GNSS_SYS_GAL;
    e.prn = svid - 70;
    e.sqrt_a = f.f8(16);
    e.m0 = f.f8(24) * SC;
    e.e = f.f8(32);
    e.i0 = f.f8(40) * SC;
    e.omega = f.f8(48) * SC;
    e.omega0 = f.f8(56) * SC;
    e.omega_dot = f.f4(64) * SC;
    e.idot = f.f4(68) * SC;
    e.delta_n = f.f4(72) * SC;
    e.cuc = f.f4(76);
    e.cus = f.f4(80);
    e.crc = f.f4(84);
    e.crs = f.f4(88);
    e.cic = f.f4(92);   // the guide's text swaps the sine/cosine words; by name, as the 10/16 data agree
    e.cis = f.f4(96);
    e.toe = u32(b + 100);
    e.toc = u32(b + 104);
    e.af2 = f.f4(108);
    e.af1 = f.f4(112);
    e.af0 = f.f8(116);
    // WNt_oe is the GPS week mod 4096 (not the GST week): no 1024 to add
    e.week = unwrapWeek(u16(b + 124) % 4096, 4096, ref_week);
    e.iode = u16(b + 128) & 0x3FF;   // IODnav
    e.iodc = e.iode;
    const uint16_t hos = u16(b + 130);
    const uint8_t sisa_e5b = b[134];
    e.ura = sisa_e5b;
    // Health_OSSOL: bit 0 says bits 1-3 (E1-B DVS, E1-B HS) were received
    e.health = (hos & 0x01) ? static_cast<int>((((hos >> 2) & 0x03) << 1) | ((hos >> 1) & 0x01)) : kGalHealthNoStatus;
    if (sisa_e5b == 255) e.health |= kGalHealthNoSisa;
    e.tgd = f.f4(140);   // BGD(E1,E5b), the I/NAV clock's
    return f.ok ? SbfEphStatus::OK : SbfEphStatus::DNU_FIELD;
}

// BDSNav 4081: BeiDou D1 (MEO and IGSO).
SbfEphStatus decodeBdsNav(const uint8_t* b, int ref_week, gps_eph_t& e)
{
    uint8_t prn;
    if (sbfSvid(b[14], prn) != SbfSys::BDS) return SbfEphStatus::BAD_SVID;   // the PRN field is an SVID
    if (prn <= 5 || prn >= 59) return SbfEphStatus::GEO;
    Floats f{b};
    e.sys = GNSS_SYS_BDS;
    e.prn = prn;
    e.week = unwrapWeek(u16(b + 138) % 8192, 8192, ref_week - kBdtWeek0) + kBdtWeek0;   // WNt_oe, BDT mod 8192
    e.ura = b[18];
    e.health = b[19];   // SatH1
    e.iodc = b[20];     // AODC
    e.iode = b[21];     // AODE
    e.tgd = f.f4(24);   // T_GD1, B1I
    keplerGpsLayout(f, b, e);
    return f.ok ? SbfEphStatus::OK : SbfEphStatus::DNU_FIELD;
}

uint16_t minLength(uint16_t number)
{
    switch (number)
    {
        case kSbfGpsNav: return kGpsNavLen;
        case kSbfGalNav: return kGalNavLen;
        case kSbfBdsNav: return kBdsNavLen;
        default: return 0;
    }
}

}  // namespace

bool isSbfEphemerisBlock(uint16_t number)
{
    return number == kSbfGpsNav || number == kSbfGalNav || number == kSbfBdsNav;
}

SbfEphStatus decodeSbfEphemeris(const uint8_t* b, size_t len, uint16_t near_wnc, gps_eph_t& out)
{
    if (b == nullptr || len < 8) return SbfEphStatus::SHORT;
    const uint16_t number = static_cast<uint16_t>(u16(b + 4) & 0x1FFF);
    if (b[0] != '$' || b[1] != '@' || !isSbfEphemerisBlock(number)) return SbfEphStatus::NOT_EPHEMERIS;
    const uint16_t length = u16(b + 6);
    if (length > len || length < minLength(number)) return SbfEphStatus::SHORT;
    const uint16_t wnc = u16(b + 12);
    const uint16_t ref = wnc != kWncDnu ? wnc : near_wnc;
    if (ref == kWncDnu) return SbfEphStatus::NO_WEEK;

    gps_eph_t e;
    std::memset(&e, 0, sizeof(e));
    SbfEphStatus st;
    switch (number)
    {
        case kSbfGpsNav: st = decodeGpsNav(b, ref, e); break;
        case kSbfGalNav: st = decodeGalNav(b, ref, e); break;
        default:         st = decodeBdsNav(b, ref, e); break;
    }
    if (st != SbfEphStatus::OK) return st;
    e.valid = 1;
    out = e;
    return SbfEphStatus::OK;
}

bool sbfNavKeep(const uint8_t* b, size_t len)
{
    if (b == nullptr || len < 16 || b[0] != '$' || b[1] != '@') return false;
    const uint16_t number = static_cast<uint16_t>(u16(b + 4) & 0x1FFF);
    if (!isSbfEphemerisBlock(number)) return false;
    const uint16_t length = u16(b + 6);
    if (length > len || length < minLength(number)) return false;
    if (number == kSbfGalNav) return b[15] == 2;
    if (number == kSbfBdsNav)
    {
        uint8_t prn;
        return sbfSvid(b[14], prn) == SbfSys::BDS && prn > 5 && prn < 59;
    }
    return true;
}

bool decodeGpsIon(const uint8_t* b, size_t len, int8_t raw[8], uint16_t& gps_day)
{
    static constexpr double kScale[8] = {
        9.313225746154785e-10, 7.450580596923828e-09, 5.960464477539063e-08, 5.960464477539063e-08,
        2048.0, 16384.0, 65536.0, 65536.0};   // 2^-30, 2^-27, 2^-24, 2^-24, 2^11, 2^14, 2^16, 2^16
    if (b == nullptr || len < 14 || b[0] != '$' || b[1] != '@') return false;
    if ((u16(b + 4) & 0x1FFF) != kSbfGpsIon) return false;
    const uint16_t length = u16(b + 6);
    if (length > len || length < kGpsIonLen) return false;
    const uint32_t tow = u32(b + 8);
    const uint16_t wnc = u16(b + 12);
    if (tow == kTowDnu || wnc == kWncDnu) return false;
    int8_t r[8];
    for (int k = 0; k < 8; k++)
    {
        const double x = sbf::f32(b + 16 + 4 * k);
        if (!std::isfinite(x)) return false;
        const double v = std::round(x / kScale[k]);
        if (v < -128.0 || v > 127.0 || v * kScale[k] != x) return false;
        r[k] = static_cast<int8_t>(v);
    }
    std::memcpy(raw, r, sizeof(r));
    gps_day = static_cast<uint16_t>(wnc * 7U + tow / 86400000U);
    return true;
}

}  // namespace mosaic
