// Host tests for the mosaic-G5's navigation decoders
// (TR_GNSSReceiverMosaic_Serial/SbfNav.cpp): GPSNav, GALNav and BDSNav to
// gps_eph_t, and GPSIon to the 8 broadcast Klobuchar bytes. Every ephemeris
// the Beetle's LC86G sent in fixtures/raw_meas_golden.h (RTCM 1019/1046/1042,
// the COCOM rig) is written here as the SBF block the mosaic would send for
// it, from the guide's field list (Reference Guide, firmware 1.0.1, 4.2),
// and must decode to the RTCM ephemeris field for field and place the
// satellite identically. Then what is refused: Galileo F/NAV, BeiDou GEO, a
// GPS cutover, a do-not-use group delay. With TR_SBF_PRIVATE_CAPTURE set,
// every navigation block of the 10/16 V9 captures is decoded too (statistics
// only).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <vector>

#include "EphemerisStore.h"
#include "RawMeasBuilder.h"
#include "RtcmEphemeris.h"
#include "SbfMeas.h"
#include "SbfNav.h"
#include "fixtures/raw_meas_golden.h"
#include "gnss/eph.h"
#include "gnss/types.h"
#include "sbf_build.h"
#include "sbf_private_capture.h"

using namespace mosaic;

namespace {

constexpr int kNearWeek = 2440;
constexpr double kDnu = -2e10;
constexpr double kGalPairMaxS = 1800.0;   // Galileo issues at most this far apart get the tight bound

// A block body written at the guide's absolute offsets (the body starts at 14).
struct Body
{
    std::vector<uint8_t> v;
    explicit Body(size_t block_len) : v(block_len - 14, 0) {}
    void put(size_t at, const void* p, size_t n) { std::memcpy(v.data() + (at - 14), p, n); }
    void u8(size_t at, uint8_t x) { put(at, &x, 1); }
    void u16(size_t at, uint16_t x) { const uint8_t b[2] = {static_cast<uint8_t>(x), static_cast<uint8_t>(x >> 8)}; put(at, b, 2); }
    void u32(size_t at, uint32_t x)
    {
        const uint8_t b[4] = {static_cast<uint8_t>(x), static_cast<uint8_t>(x >> 8), static_cast<uint8_t>(x >> 16),
                              static_cast<uint8_t>(x >> 24)};
        put(at, b, 4);
    }
    void f32(size_t at, double x) { const float f = static_cast<float>(x); put(at, &f, 4); }   // little-endian host
    void f64(size_t at, double x) { put(at, &x, 8); }
};

// An angle in semicircles as the receiver sends it: the broadcast integer at
// 2^-31 semicircles, so f8 M_0, OMEGA_0, i_0 and omega carry no rounding.
double semi31(double rad) { return std::ldexp(std::round(std::ldexp(rad / GPS_PI_ICD, 31)), -31); }
// An f4 rate in semicircles/s (2^-43 integers of at most 24 bits: exact in a float).
double semiF(double rad) { return rad / GPS_PI_ICD; }

// GPSNav and BDSNav from t_oc (32) to IDOT (132).
void putKeplerGpsLayout(Body& b, const gps_eph_t& e)
{
    b.u32(32, static_cast<uint32_t>(e.toc));
    b.f32(36, e.af2);
    b.f32(40, e.af1);
    b.f32(44, e.af0);
    b.f32(48, e.crs);
    b.f32(52, semiF(e.delta_n));
    b.f64(56, semi31(e.m0));
    b.f32(64, e.cuc);
    b.f64(68, e.e);
    b.f32(76, e.cus);
    b.f64(80, e.sqrt_a);
    b.u32(88, static_cast<uint32_t>(e.toe));
    b.f32(92, e.cic);
    b.f64(96, semi31(e.omega0));
    b.f32(104, e.cis);
    b.f64(108, semi31(e.i0));
    b.f32(116, e.crc);
    b.f64(120, semi31(e.omega));
    b.f32(128, semiF(e.omega_dot));
    b.f32(132, semiF(e.idot));
}

std::vector<uint8_t> gpsNav(const gps_eph_t& e, uint16_t wnc, uint32_t tow = 345600000)
{
    Body b(140);
    b.u8(14, static_cast<uint8_t>(e.prn));
    b.u16(16, static_cast<uint16_t>(e.week % 1024));
    b.u8(18, 1);
    b.u8(19, static_cast<uint8_t>(e.ura));
    b.u8(20, static_cast<uint8_t>(e.health));
    b.u16(22, static_cast<uint16_t>(e.iodc));
    b.u8(24, static_cast<uint8_t>(e.iode));
    b.u8(25, static_cast<uint8_t>(e.iode));
    b.f32(28, e.tgd);
    putKeplerGpsLayout(b, e);
    b.u16(136, static_cast<uint16_t>(e.week % 1024));
    b.u16(138, static_cast<uint16_t>(e.week % 1024));
    return sbftest::block(kSbfGpsNav, 0, tow, wnc, b.v);
}

std::vector<uint8_t> galNav(const gps_eph_t& e, uint16_t wnc, uint8_t source = 2, uint32_t tow = 345600000)
{
    Body b(152);
    b.u8(14, static_cast<uint8_t>(e.prn + 70));
    b.u8(15, source);
    b.f64(16, e.sqrt_a);
    b.f64(24, semi31(e.m0));
    b.f64(32, e.e);
    b.f64(40, semi31(e.i0));
    b.f64(48, semi31(e.omega));
    b.f64(56, semi31(e.omega0));
    b.f32(64, semiF(e.omega_dot));
    b.f32(68, semiF(e.idot));
    b.f32(72, semiF(e.delta_n));
    b.f32(76, e.cuc);
    b.f32(80, e.cus);
    b.f32(84, e.crc);
    b.f32(88, e.crs);
    b.f32(92, e.cic);
    b.f32(96, e.cis);
    b.u32(100, static_cast<uint32_t>(e.toe));
    b.u32(104, static_cast<uint32_t>(e.toc));
    b.f32(108, e.af2);
    b.f32(112, e.af1);
    b.f64(116, e.af0);
    b.u16(124, static_cast<uint16_t>(e.week % 4096));
    b.u16(126, static_cast<uint16_t>(e.week % 4096));
    b.u16(128, static_cast<uint16_t>(e.iode));
    // Health_OSSOL: E1-B valid, DVS and HS from the RTCM (HS << 1) | DVS;
    // E5b valid and healthy
    const uint16_t hos = static_cast<uint16_t>(0x01 | ((e.health & 1) << 1) | (((e.health >> 1) & 3) << 2) | 0x10);
    b.u16(130, hos);
    b.u8(133, source == 2 ? 255 : static_cast<uint8_t>(e.ura));
    b.u8(134, source == 2 ? static_cast<uint8_t>(e.ura) : 255);
    b.u8(135, 255);
    b.f32(136, source == 2 ? kDnu : -2.0e-9);
    b.f32(140, source == 2 ? e.tgd : kDnu);   // an F/NAV block has no BGD(E1,E5b)
    b.f32(144, kDnu);
    b.u8(148, 255);
    return sbftest::block(kSbfGalNav, 0, tow, wnc, b.v);
}

std::vector<uint8_t> bdsNav(const gps_eph_t& e, uint16_t wnc, uint8_t svid = 0, uint32_t tow = 345600000)
{
    Body b(140);
    b.u8(14, svid != 0 ? svid : static_cast<uint8_t>(e.prn + 140));
    b.u16(16, static_cast<uint16_t>(e.week - 1356));
    b.u8(18, static_cast<uint8_t>(e.ura));
    b.u8(19, static_cast<uint8_t>(e.health));
    b.u8(20, static_cast<uint8_t>(e.iodc));
    b.u8(21, static_cast<uint8_t>(e.iode));
    b.f32(24, e.tgd);
    b.f32(28, kDnu);
    putKeplerGpsLayout(b, e);
    b.u16(136, static_cast<uint16_t>((e.week - 1356) % 8192));
    b.u16(138, static_cast<uint16_t>((e.week - 1356) % 8192));
    return sbftest::block(kSbfBdsNav, 0, tow, wnc, b.v);
}

std::vector<uint8_t> sbfFor(const gps_eph_t& e, uint16_t wnc)
{
    switch (e.sys)
    {
        case GNSS_SYS_GPS: return gpsNav(e, wnc);
        case GNSS_SYS_GAL: return galNav(e, wnc);
        default: return bdsNav(e, wnc);
    }
}

std::vector<gps_eph_t> rtcmGoldens()
{
    std::vector<gps_eph_t> out;
    for (int i = 0; i < kGoldenRmEphN; ++i)
    {
        gps_eph_t e;
        EXPECT_TRUE(gnss_raw::decodeEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len),
                                              kNearWeek, e));
        out.push_back(e);
    }
    return out;
}

gps_eph_t firstOf(int sys)
{
    for (const gps_eph_t& e : rtcmGoldens())
        if (e.sys == sys) return e;
    ADD_FAILURE() << "no golden ephemeris of system " << sys;
    return gps_eph_t{};
}

std::vector<uint8_t> gpsIon(const int8_t raw[8], uint32_t tow, uint16_t wnc)
{
    static constexpr int kExp[8] = {-30, -27, -24, -24, 11, 14, 16, 16};
    Body b(48);
    b.u8(14, 12);
    for (int k = 0; k < 8; k++) b.f32(16 + 4 * k, std::ldexp(static_cast<double>(raw[k]), kExp[k]));
    return sbftest::block(kSbfGpsIon, 0, tow, wnc, b.v);
}

}  // namespace

TEST(SbfNav, EveryGoldenEphemerisDecodesAsItsRtcm)
{
    const std::vector<gps_eph_t> goldens = rtcmGoldens();
    ASSERT_EQ(goldens.size(), 15u);
    int per_sys[3] = {};
    for (const gps_eph_t& r : goldens)
    {
        const auto blk = sbfFor(r, static_cast<uint16_t>(r.week));
        gps_eph_t s;
        ASSERT_EQ(decodeSbfEphemeris(blk.data(), blk.size(), kWncDnu, s), SbfEphStatus::OK) << r.sys << "/" << r.prn;
        per_sys[r.sys]++;
        SCOPED_TRACE(testing::Message() << "sys " << r.sys << " prn " << r.prn);
        EXPECT_EQ(s.valid, 1);
        EXPECT_EQ(s.sys, r.sys);
        EXPECT_EQ(s.prn, r.prn);
        EXPECT_EQ(s.week, r.week);
        EXPECT_EQ(s.iodc, r.iodc);
        EXPECT_EQ(s.iode, r.iode);
        EXPECT_EQ(s.health, r.health);
        EXPECT_EQ(s.ura, r.ura);
        EXPECT_EQ(s.toe, r.toe);
        EXPECT_EQ(s.toc, r.toc);
        // the SBF floats are the broadcast integers at their scales, so the
        // orbit and clock come out bit for bit
        EXPECT_EQ(s.sqrt_a, r.sqrt_a);
        EXPECT_EQ(s.e, r.e);
        EXPECT_EQ(s.i0, r.i0);
        EXPECT_EQ(s.omega0, r.omega0);
        EXPECT_EQ(s.omega, r.omega);
        EXPECT_EQ(s.m0, r.m0);
        EXPECT_EQ(s.delta_n, r.delta_n);
        EXPECT_EQ(s.idot, r.idot);
        EXPECT_EQ(s.omega_dot, r.omega_dot);
        EXPECT_EQ(s.cuc, r.cuc);
        EXPECT_EQ(s.cus, r.cus);
        EXPECT_EQ(s.crc, r.crc);
        EXPECT_EQ(s.crs, r.crs);
        EXPECT_EQ(s.cic, r.cic);
        EXPECT_EQ(s.cis, r.cis);
        EXPECT_EQ(s.af0, r.af0);
        EXPECT_EQ(s.af1, r.af1);
        EXPECT_EQ(s.af2, r.af2);
        // BeiDou's TGD1 is a 0.1 ns integer, which an f4 holds to 1e-16 s
        if (r.sys == GNSS_SYS_BDS)
            EXPECT_EQ(s.tgd, static_cast<double>(static_cast<float>(r.tgd)));
        else
            EXPECT_EQ(s.tgd, r.tgd);

        // and the satellite is where the RTCM ephemeris puts it
        for (double dt : {-3600.0, 0.0, 1800.0, 7000.0})
        {
            double pr[3], vr[3], cr, ps[3], vs[3], cs;
            gps_sat_pos(&r, r.toe + dt, pr, vr, &cr);
            gps_sat_pos(&s, s.toe + dt, ps, vs, &cs);
            EXPECT_LT(std::hypot(std::hypot(ps[0] - pr[0], ps[1] - pr[1]), ps[2] - pr[2]), 1e-3);
            EXPECT_LT(std::fabs(cs - cr), 1e-9);
        }
    }
    EXPECT_EQ(per_sys[GNSS_SYS_GPS], 10);
    EXPECT_EQ(per_sys[GNSS_SYS_GAL], 4);
    EXPECT_EQ(per_sys[GNSS_SYS_BDS], 1);
}

TEST(SbfNav, WeeksAreMadeWhole)
{
    // GPS: WNt_oe 392 = 2440 mod 1024; Galileo: 2440 as sent (mod 4096, GPS
    // frame); BeiDou: BDT 1084 = GPS 2440
    gps_eph_t g = firstOf(GNSS_SYS_GPS), a = firstOf(GNSS_SYS_GAL), c = firstOf(GNSS_SYS_BDS);
    g.week = a.week = c.week = 2440;
    const auto gb = gpsNav(g, 2440), ab = galNav(a, 2440), cb = bdsNav(c, 2440);
    EXPECT_EQ(gb[138] | (gb[139] << 8), 392);
    EXPECT_EQ(cb[138] | (cb[139] << 8), 1084);
    gps_eph_t e;
    ASSERT_EQ(decodeSbfEphemeris(gb.data(), gb.size(), kWncDnu, e), SbfEphStatus::OK);
    EXPECT_EQ(e.week, 2440);
    ASSERT_EQ(decodeSbfEphemeris(ab.data(), ab.size(), kWncDnu, e), SbfEphStatus::OK);
    EXPECT_EQ(e.week, 2440);
    ASSERT_EQ(decodeSbfEphemeris(cb.data(), cb.size(), kWncDnu, e), SbfEphStatus::OK);
    EXPECT_EQ(e.week, 2440);

    // a toe in the week before the block's stamp
    const auto late = gpsNav(g, 2441);
    ASSERT_EQ(decodeSbfEphemeris(late.data(), late.size(), kWncDnu, e), SbfEphStatus::OK);
    EXPECT_EQ(e.week, 2440);

    // no stamp: the caller's week stands in; neither: no week
    const auto unstamped = gpsNav(g, kWncDnu);
    ASSERT_EQ(decodeSbfEphemeris(unstamped.data(), unstamped.size(), 2441, e), SbfEphStatus::OK);
    EXPECT_EQ(e.week, 2440);
    EXPECT_EQ(decodeSbfEphemeris(unstamped.data(), unstamped.size(), kWncDnu, e), SbfEphStatus::NO_WEEK);
    const auto bds_unstamped = bdsNav(c, kWncDnu);
    ASSERT_EQ(decodeSbfEphemeris(bds_unstamped.data(), bds_unstamped.size(), 2440, e), SbfEphStatus::OK);
    EXPECT_EQ(e.week, 2440);
}

TEST(SbfNav, RefusesGalileoFnav)
{
    const gps_eph_t a = firstOf(GNSS_SYS_GAL);
    // E25's case on the nose-cone capture: the same IODnav and toe from both sources
    const auto inav = galNav(a, 2440, 2);
    const auto fnav = galNav(a, 2440, 16);
    gps_eph_t e;
    EXPECT_EQ(decodeSbfEphemeris(inav.data(), inav.size(), kWncDnu, e), SbfEphStatus::OK);
    EXPECT_TRUE(sbfNavKeep(inav.data(), inav.size()));
    e.valid = 0;
    EXPECT_EQ(decodeSbfEphemeris(fnav.data(), fnav.size(), kWncDnu, e), SbfEphStatus::FNAV_SKIPPED);
    EXPECT_EQ(e.valid, 0);   // untouched
    EXPECT_FALSE(sbfNavKeep(fnav.data(), fnav.size()));

    // an I/NAV block whose BGD(E1,E5b) is do-not-use
    auto dnu = inav;
    const float bad = -2e10f;
    std::memcpy(dnu.data() + 140, &bad, 4);
    EXPECT_EQ(decodeSbfEphemeris(dnu.data(), dnu.size(), kWncDnu, e), SbfEphStatus::DNU_FIELD);
}

TEST(SbfNav, RefusesBeidouGeo)
{
    gps_eph_t c = firstOf(GNSS_SYS_BDS);
    gps_eph_t e;
    // C01, C03, C05 and C59, C63
    for (uint8_t svid : {141, 143, 145, 241, 245})
    {
        const auto b = bdsNav(c, 2440, svid);
        EXPECT_EQ(decodeSbfEphemeris(b.data(), b.size(), kWncDnu, e), SbfEphStatus::GEO) << int(svid);
        EXPECT_FALSE(sbfNavKeep(b.data(), b.size())) << int(svid);
    }
    // C06 (IGSO), C40, C41 and C58 are placed; the C41-C63 SVIDs are 223-245
    for (uint8_t svid : {146, 180, 223, 240})
    {
        const auto b = bdsNav(c, 2440, svid);
        ASSERT_EQ(decodeSbfEphemeris(b.data(), b.size(), kWncDnu, e), SbfEphStatus::OK) << int(svid);
        EXPECT_EQ(e.prn, svid <= 180 ? svid - 140 : svid - 182);
        EXPECT_TRUE(sbfNavKeep(b.data(), b.size())) << int(svid);
    }
    // an SVID that is not BeiDou's
    const auto qz = bdsNav(c, 2440, 183);
    EXPECT_EQ(decodeSbfEphemeris(qz.data(), qz.size(), kWncDnu, e), SbfEphStatus::BAD_SVID);
    EXPECT_FALSE(sbfNavKeep(qz.data(), qz.size()));
}

TEST(SbfNav, RefusesAGpsCutover)
{
    const gps_eph_t g = firstOf(GNSS_SYS_GPS);
    gps_eph_t e;
    auto b = gpsNav(g, 2440);
    b[25] = static_cast<uint8_t>(g.iode + 1);    // IODE3 of the next issue
    EXPECT_EQ(decodeSbfEphemeris(b.data(), b.size(), kWncDnu, e), SbfEphStatus::IOD_MISMATCH);
    b = gpsNav(g, 2440);
    b[24] = b[25] = static_cast<uint8_t>(g.iode + 1);   // IODC of the last issue
    EXPECT_EQ(decodeSbfEphemeris(b.data(), b.size(), kWncDnu, e), SbfEphStatus::IOD_MISMATCH);
    // the driver's pre-queue check leaves this to the decoder
    EXPECT_TRUE(sbfNavKeep(b.data(), b.size()));

    b = gpsNav(g, 2440);
    b[14] = 38;
    EXPECT_EQ(decodeSbfEphemeris(b.data(), b.size(), kWncDnu, e), SbfEphStatus::BAD_SVID);
}

TEST(SbfNav, GalileoHealthAndAccuracy)
{
    const gps_eph_t a = firstOf(GNSS_SYS_GAL);
    gps_eph_t e;
    auto withHos = [&](uint16_t hos, uint8_t sisa) {
        auto b = galNav(a, 2440);
        b[130] = static_cast<uint8_t>(hos);
        b[131] = static_cast<uint8_t>(hos >> 8);
        b[134] = sisa;
        EXPECT_EQ(decodeSbfEphemeris(b.data(), b.size(), kWncDnu, e), SbfEphStatus::OK);
        return e.health;
    };
    EXPECT_EQ(withHos(0x0001, 107), 0);                 // E1-B healthy and valid
    EXPECT_EQ(withHos(0x0003, 107), 1);                 // DVS: working without guarantee
    EXPECT_EQ(withHos(0x0009, 107), 4);                 // HS 2: out of service
    EXPECT_EQ(withHos(0x00F1, 107), 0);                 // E5b's bits are not E1's
    EXPECT_EQ(withHos(0x0000, 107), kGalHealthNoStatus);
    EXPECT_EQ(withHos(0x0001, 255), kGalHealthNoSisa);
    EXPECT_EQ(e.ura, 255);

    // the store passes over an unhealthy one
    gnss_raw::EphemerisStore store;
    withHos(0x0001, 255);
    store.add(e);
    EXPECT_EQ(store.select(GNSS_SYS_GAL, a.prn, a.toe), nullptr);
}

TEST(SbfNav, RefusesShortAndOtherBlocks)
{
    const gps_eph_t g = firstOf(GNSS_SYS_GPS);
    const auto b = gpsNav(g, 2440);
    gps_eph_t e;
    EXPECT_EQ(decodeSbfEphemeris(b.data(), b.size() - 1, kWncDnu, e), SbfEphStatus::SHORT);
    EXPECT_EQ(decodeSbfEphemeris(b.data(), 4, kWncDnu, e), SbfEphStatus::SHORT);
    EXPECT_EQ(decodeSbfEphemeris(nullptr, 0, kWncDnu, e), SbfEphStatus::SHORT);
    // a Length short of the fields
    auto shortened = sbftest::block(kSbfGpsNav, 0, 1000, 2440, std::vector<uint8_t>(b.begin() + 14, b.begin() + 136));
    EXPECT_EQ(decodeSbfEphemeris(shortened.data(), shortened.size(), kWncDnu, e), SbfEphStatus::SHORT);
    EXPECT_FALSE(sbfNavKeep(shortened.data(), shortened.size()));

    const int8_t raw[8] = {21, 0, -2, 0, 60, -1, -4, 3};
    const auto ion = gpsIon(raw, 1000, 2440);
    EXPECT_EQ(decodeSbfEphemeris(ion.data(), ion.size(), kWncDnu, e), SbfEphStatus::NOT_EPHEMERIS);
    EXPECT_FALSE(sbfNavKeep(ion.data(), ion.size()));
    auto bad_sync = b;
    bad_sync[0] = '#';
    EXPECT_EQ(decodeSbfEphemeris(bad_sync.data(), bad_sync.size(), kWncDnu, e), SbfEphStatus::NOT_EPHEMERIS);

    // a do-not-use orbit float
    auto dnu = b;
    const double bad = -2e10;
    std::memcpy(dnu.data() + 80, &bad, 8);   // SQRT_A
    EXPECT_EQ(decodeSbfEphemeris(dnu.data(), dnu.size(), kWncDnu, e), SbfEphStatus::DNU_FIELD);

    EXPECT_TRUE(isSbfEphemerisBlock(5891));
    EXPECT_TRUE(isSbfEphemerisBlock(4002));
    EXPECT_TRUE(isSbfEphemerisBlock(4081));
    EXPECT_FALSE(isSbfEphemerisBlock(4251));   // BDSCNav1: B-CNAV1, which gps_eph_t cannot hold
    EXPECT_FALSE(isSbfEphemerisBlock(5893));
}

TEST(SbfIon, GpsIonIsTheBroadcastBytesExactly)
{
    // the 10/16 captures' set
    const int8_t raw[8] = {21, 0, -2, 0, 60, -1, -4, 3};
    const uint32_t tow = 3 * 86400000U + 12345678;
    const auto b = gpsIon(raw, tow, 2440);
    int8_t got[8] = {};
    uint16_t day = 0;
    ASSERT_TRUE(decodeGpsIon(b.data(), b.size(), got, day));
    for (int k = 0; k < 8; k++) EXPECT_EQ(got[k], raw[k]) << k;
    EXPECT_EQ(day, 2440 * 7 + 3);

    // what the builder makes of them is the receiver's floats
    const gps_iono_t io = gnss_raw::RawMeasBuilder::ionoFromBroadcast(got);
    for (int k = 0; k < 4; k++)
    {
        float a, bb;
        std::memcpy(&a, b.data() + 16 + 4 * k, 4);
        std::memcpy(&bb, b.data() + 32 + 4 * k, 4);
        EXPECT_EQ(io.alpha[k], static_cast<double>(a));
        EXPECT_EQ(io.beta[k], static_cast<double>(bb));
    }

    // the extremes of int8
    const int8_t ends[8] = {-128, 127, -128, 127, -128, 127, -128, 127};
    const auto e = gpsIon(ends, tow, 2440);
    ASSERT_TRUE(decodeGpsIon(e.data(), e.size(), got, day));
    for (int k = 0; k < 8; k++) EXPECT_EQ(got[k], ends[k]) << k;
}

TEST(SbfIon, RefusesWhatIsNotABroadcastSet)
{
    const int8_t raw[8] = {21, 0, -2, 0, 60, -1, -4, 3};
    int8_t got[8];
    uint16_t day;
    auto b = gpsIon(raw, 1000, 2440);
    auto poke = [&](int k, float x) {
        auto c = b;
        std::memcpy(c.data() + 16 + 4 * k, &x, 4);
        return c;
    };
    auto off = poke(0, std::ldexp(21.5f, -30));            // between two broadcast values
    EXPECT_FALSE(decodeGpsIon(off.data(), off.size(), got, day));
    auto big = poke(4, std::ldexp(128.0f, 11));             // past int8
    EXPECT_FALSE(decodeGpsIon(big.data(), big.size(), got, day));
    auto nan = poke(2, std::nanf(""));
    EXPECT_FALSE(decodeGpsIon(nan.data(), nan.size(), got, day));
    auto untimed = gpsIon(raw, kTowDnu, 2440);
    EXPECT_FALSE(decodeGpsIon(untimed.data(), untimed.size(), got, day));
    untimed = gpsIon(raw, 1000, kWncDnu);
    EXPECT_FALSE(decodeGpsIon(untimed.data(), untimed.size(), got, day));
    EXPECT_FALSE(decodeGpsIon(b.data(), b.size() - 4, got, day));
    const auto nav = gpsNav(firstOf(GNSS_SYS_GPS), 2440);
    EXPECT_FALSE(decodeGpsIon(nav.data(), nav.size(), got, day));
}

// ── The 10/16 captures (private; TR_SBF_PRIVATE_CAPTURE) ─────────────────

TEST(SbfNavPrivateCapture, EveryNavBlockDecodes)
{
    const auto paths = sbftest::privateCapturePaths();
    if (paths.empty()) GTEST_SKIP() << "TR_SBF_PRIVATE_CAPTURE not set";
    for (const std::string& path : paths)
    {
        std::map<uint16_t, std::map<SbfEphStatus, uint32_t>> st;
        uint32_t keep_disagrees = 0, ion_ok = 0, ion_bad = 0, fnav_bgd_dnu = 0, over_len = 0;
        std::map<std::pair<int, int>, std::vector<gps_eph_t>> by_sat;
        sbftest::CaptureScan scan;
        const bool read = sbftest::scanCapture(path, [&](const uint8_t* b, size_t len) {
            SbfHeader h;
            if (!sbfHeader(b, len, h)) return;
            if (isSbfEphemerisBlock(h.number))
            {
                gps_eph_t e;
                const SbfEphStatus s = decodeSbfEphemeris(b, len, kWncDnu, e);
                st[h.number][s]++;
                const bool refusal = s == SbfEphStatus::FNAV_SKIPPED || s == SbfEphStatus::GEO;
                if (sbfNavKeep(b, len) != !refusal) keep_disagrees++;
                if (len > kSbfNavMaxLen) over_len++;
                if (s == SbfEphStatus::FNAV_SKIPPED)
                {
                    float bgd;
                    std::memcpy(&bgd, b + 140, 4);
                    if (!(bgd > -1e10f)) fnav_bgd_dnu++;
                }
                if (s == SbfEphStatus::OK)
                {
                    auto& v = by_sat[{e.sys, e.prn}];
                    if (v.empty() || v.back().iode != e.iode || v.back().toe != e.toe) v.push_back(e);
                }
            }
            else if (h.number == kSbfGpsIon)
            {
                int8_t raw[8];
                uint16_t day;
                if (decodeGpsIon(b, len, raw, day)) ion_ok++; else ion_bad++;
            }
        }, scan);
        ASSERT_TRUE(read) << path;

        // consecutive issues of one satellite agree at the middle of their toes
        // (a swapped or mis-scaled field shows as kilometres). Galileo's I/NAV
        // issues, 10 min apart, agree much closer, and that bound is what
        // tells its C_ic from its C_is: the guide's text swaps the two words,
        // the builder above writes them where the decoder reads them, and on
        // the 10/16 captures the swapped reading leaves 0.58 and 0.98 m where
        // the right one leaves 0.19 and 0.16 m
        double worst_m = 0.0, worst_gal_m = 0.0;
        int pairs = 0, gal_pairs = 0;
        for (const auto& kv : by_sat)
            for (size_t i = 1; i < kv.second.size(); i++)
            {
                const gps_eph_t& p = kv.second[i - 1];
                const gps_eph_t& q = kv.second[i];
                const double dt = gps_time_diff(q.toe, p.toe);
                if (!(dt > 0.0 && dt <= 7200.0)) continue;
                const double t = p.toe + dt / 2;
                double a[3], va[3], ca, c[3], vc[3], cc;
                gps_sat_pos(&p, t, a, va, &ca);
                gps_sat_pos(&q, t, c, vc, &cc);
                const double d = std::hypot(std::hypot(a[0] - c[0], a[1] - c[1]), a[2] - c[2]);
                worst_m = std::max(worst_m, d);
                pairs++;
                if (p.sys == GNSS_SYS_GAL && dt <= kGalPairMaxS)
                {
                    worst_gal_m = std::max(worst_gal_m, d);
                    gal_pairs++;
                }
            }

        const char* names[] = {"OK", "NOT_EPHEMERIS", "SHORT", "NO_WEEK", "BAD_SVID", "FNAV_SKIPPED", "GEO",
                               "IOD_MISMATCH", "DNU_FIELD"};
        std::printf("[capture] %s:", sbftest::captureName(path).c_str());
        for (const auto& kv : st)
        {
            std::printf(" %s", kv.first == kSbfGpsNav ? "GPSNav" : kv.first == kSbfGalNav ? "GALNav" : "BDSNav");
            for (const auto& s : kv.second) std::printf(" %s %u", names[static_cast<int>(s.first)], s.second);
            std::printf(";");
        }
        std::printf(" GPSIon %u ok %u bad; sats with an ephemeris %zu; consecutive issues %d, worst mid-toe "
                    "difference %.2f m (Galileo within %.0f min: %d, %.2f m)\n", ion_ok, ion_bad, by_sat.size(),
                    pairs, worst_m, kGalPairMaxS / 60.0, gal_pairs, worst_gal_m);

        EXPECT_EQ(scan.crc_fails, 0u);
        for (const auto& kv : st)
            for (const auto& s : kv.second)
            {
                const bool allowed = s.first == SbfEphStatus::OK ||
                                     (kv.first == kSbfGalNav && s.first == SbfEphStatus::FNAV_SKIPPED) ||
                                     (kv.first == kSbfBdsNav && s.first == SbfEphStatus::GEO);
                EXPECT_TRUE(allowed) << kv.first << " " << names[static_cast<int>(s.first)] << " x" << s.second;
            }
        EXPECT_EQ(fnav_bgd_dnu, st[kSbfGalNav][SbfEphStatus::FNAV_SKIPPED]);
        EXPECT_EQ(keep_disagrees, 0u);
        EXPECT_EQ(over_len, 0u);
        EXPECT_GT(st[kSbfGalNav][SbfEphStatus::OK], 0u);
        EXPECT_GT(ion_ok, 0u);
        EXPECT_EQ(ion_bad, 0u);
        if (pairs > 0) EXPECT_LT(worst_m, 5.0);
        if (gal_pairs > 0) EXPECT_LT(worst_gal_m, 0.4);
    }
}
