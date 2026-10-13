// Host tests for TR_GNSSRaw: the LC86G's RTCM broadcast ephemeris decoded,
// satellites placed, and the atmosphere modelled exactly as the bench
// prototype that scored the raw-measurement filter does (gnss repo
// sim/tinkerrocket_gnss). The reference values in fixtures/gnss_raw_golden.h
// come from that prototype, run on ephemeris payloads the Beetle's own LC86G
// sent on the COCOM rig.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

#include "EphemerisStore.h"
#include "RtcmEphemeris.h"
#include "fixtures/gnss_raw_golden.h"
#include "gnss/atmo.h"
#include "gnss/eph.h"
#include "gnss/types.h"

using gnss_raw::EphemerisStore;
using gnss_raw::decodeEphemeris;
using gnss_raw::fullGpsWeek;
using gnss_raw::rtcmMessageType;

namespace
{

constexpr int kNearWeek = 2440;   // GPS week of 2026-10-12; the captures' sky (2026-08-29) is 2433

gps_eph_t decodeGolden(const GoldenEph& g)
{
    gps_eph_t e;
    EXPECT_TRUE(decodeEphemeris(g.payload, static_cast<size_t>(g.len), kNearWeek, e))
        << "sys " << g.sys << " prn " << g.prn;
    return e;
}

}  // namespace

TEST(GnssRawEphemeris, DecodesEveryCapturedPayloadAsThePrototypeDoes)
{
    ASSERT_EQ(kGoldenEphN, 9);
    for (int i = 0; i < kGoldenEphN; ++i)
    {
        const GoldenEph& g = kGoldenEph[i];
        SCOPED_TRACE(testing::Message() << "msg " << g.msg_type << " prn " << g.prn);
        EXPECT_EQ(rtcmMessageType(g.payload, g.len), g.msg_type);
        const gps_eph_t e = decodeGolden(g);
        EXPECT_EQ(e.valid, 1);
        EXPECT_EQ(e.sys, g.sys);
        EXPECT_EQ(e.prn, g.prn);
        EXPECT_EQ(e.iode, g.iode);
        EXPECT_EQ(e.health, g.health);
        // the same integers times the same powers of two: exact, but for sqrt(A),
        // which the prototype keeps squared
        EXPECT_EQ(e.toe, g.toe);
        EXPECT_EQ(e.toc, g.toc);
        EXPECT_NEAR(e.sqrt_a, g.sqrt_a, 1e-12 * g.sqrt_a);
        EXPECT_EQ(e.e, g.e);
        EXPECT_EQ(e.i0, g.i0);
        EXPECT_EQ(e.omega0, g.omega0);
        EXPECT_EQ(e.omega, g.omega);
        EXPECT_EQ(e.m0, g.m0);
        EXPECT_EQ(e.delta_n, g.delta_n);
        EXPECT_EQ(e.idot, g.idot);
        EXPECT_EQ(e.omega_dot, g.omega_dot);
        EXPECT_EQ(e.cuc, g.cuc);
        EXPECT_EQ(e.cus, g.cus);
        EXPECT_EQ(e.crc, g.crc);
        EXPECT_EQ(e.crs, g.crs);
        EXPECT_EQ(e.cic, g.cic);
        EXPECT_EQ(e.cis, g.cis);
        EXPECT_EQ(e.af0, g.af0);
        EXPECT_EQ(e.af1, g.af1);
        EXPECT_EQ(e.af2, g.af2);
        EXPECT_EQ(e.tgd, g.tgd);
    }
}

TEST(GnssRawEphemeris, WeeksAreWholeGpsWeeks)
{
    for (int i = 0; i < kGoldenEphN; ++i)
    {
        const gps_eph_t e = decodeGolden(kGoldenEph[i]);
        // 2026-08-29 is in GPS week 2433; toe may sit on either side of a week edge
        EXPECT_GE(e.week, 2432) << "sys " << e.sys << " prn " << e.prn;
        EXPECT_LE(e.week, 2434) << "sys " << e.sys << " prn " << e.prn;
    }
    EXPECT_EQ(fullGpsWeek(391, 2439), 2439);
    EXPECT_EQ(fullGpsWeek(391, 2100), 2439);     // within half a rollover
    EXPECT_EQ(fullGpsWeek(1023, 2048), 2047);    // the nearest, across a rollover
    EXPECT_EQ(fullGpsWeek(0, 2047), 2048);
    EXPECT_EQ(fullGpsWeek(391, 0), 391);         // no reference: as sent
}

TEST(GnssRawEphemeris, PlacesSatellitesAndClocksAsThePrototypeDoes)
{
    for (int i = 0; i < kGoldenEphN; ++i)
    {
        const GoldenEph& g = kGoldenEph[i];
        const gps_eph_t e = decodeGolden(g);
        for (int k = 0; k < 3; ++k)
        {
            SCOPED_TRACE(testing::Message() << "sys " << g.sys << " prn " << g.prn << " t " << g.t[k]);
            double pos[3], vel[3], clk = 0.0;
            gps_sat_pos(&e, g.t[k], pos, vel, &clk);
            for (int a = 0; a < 3; ++a)
                EXPECT_NEAR(pos[a], g.pos[k][a], 1e-3);        // 1 mm
            EXPECT_NEAR(clk, g.clk[k], 1e-12);                 // 0.3 mm of range
            const double speed = std::sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]);
            EXPECT_GT(speed, 2000.0);                           // ECEF speed of a MEO/IGSO satellite
            EXPECT_LT(speed, 4500.0);
        }
    }
}

TEST(GnssRawEphemeris, RefusesWhatItCannotPlace)
{
    const GoldenEph* gps = nullptr;
    const GoldenEph* bds = nullptr;
    for (int i = 0; i < kGoldenEphN; ++i)
    {
        if (kGoldenEph[i].msg_type == 1019 && gps == nullptr) gps = &kGoldenEph[i];
        if (kGoldenEph[i].msg_type == 1042 && bds == nullptr) bds = &kGoldenEph[i];
    }
    ASSERT_NE(gps, nullptr);
    ASSERT_NE(bds, nullptr);
    gps_eph_t e;
    // one byte short of the 488 bits
    EXPECT_FALSE(decodeEphemeris(gps->payload, 60, kNearWeek, e));
    // a BeiDou GEO (PRN 3): its D2 orbit is not the Keplerian set gnss/eph.h places
    uint8_t geo[64];
    std::memcpy(geo, bds->payload, sizeof(geo));
    geo[1] = static_cast<uint8_t>((geo[1] & 0xF0) | (3 >> 2));     // DF488 spans bits 12..17
    geo[2] = static_cast<uint8_t>((geo[2] & 0x3F) | ((3 & 0x3) << 6));
    EXPECT_FALSE(decodeEphemeris(geo, static_cast<size_t>(bds->len), kNearWeek, e));
    // an MSM7 (1077) header is not an ephemeris
    const uint8_t msm[] = {0x43, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    EXPECT_EQ(rtcmMessageType(msm, sizeof(msm)), 1077);
    EXPECT_FALSE(decodeEphemeris(msm, sizeof(msm), kNearWeek, e));
    EXPECT_FALSE(decodeEphemeris(nullptr, 0, kNearWeek, e));
}

TEST(GnssRawAtmosphere, KlobucharMatchesThePrototype)
{
    gps_iono_t io;
    for (int k = 0; k < 4; ++k)
    {
        io.alpha[k] = kGoldenIon[k];
        io.beta[k] = kGoldenIon[4 + k];
    }
    io.valid = 1;
    for (int i = 0; i < kGoldenAtmoN; ++i)
    {
        const GoldenAtmo& a = kGoldenAtmo[i];
        EXPECT_NEAR(iono_klobuchar(&io, a.lat, a.lon, a.az, a.el, a.tod), a.iono, 1e-6) << "point " << i;
    }
}

TEST(GnssRawAtmosphere, SaastamoinenMatchesThePrototypeToAMillimetre)
{
    // The prototype carries SignalSim's dry constant (0.0022767, what the rig's
    // files were made with); the receiver keeps Saastamoinen's 0.0022768. That
    // is 1 mm at 5 degrees, nothing at the zenith.
    for (int i = 0; i < kGoldenAtmoN; ++i)
    {
        const GoldenAtmo& a = kGoldenAtmo[i];
        EXPECT_NEAR(tropo_saastamoinen(a.lat, a.h, a.el), a.tropo, 2e-3) << "point " << i;
    }
    EXPECT_EQ(tropo_saastamoinen(0.7, 41000.0, 0.5), 0.0);    // above the troposphere's 40 km
    EXPECT_EQ(tropo_saastamoinen(0.7, 100.0, -0.1), 0.0);     // below the horizon
}

TEST(GnssRawGeodesy, Wgs84MatchesThePrototype)
{
    for (int i = 0; i < kGoldenGeoN; ++i)
    {
        const GoldenGeo& g = kGoldenGeo[i];
        double x[3];
        geo_to_ecef(g.lat, g.lon, g.h, x);
        EXPECT_NEAR(x[0], g.x, 1e-6);
        EXPECT_NEAR(x[1], g.y, 1e-6);
        EXPECT_NEAR(x[2], g.z, 1e-6);
        double lat, lon, h;
        ecef_to_geo(x, &lat, &lon, &h);
        EXPECT_NEAR(lat, g.lat, 1e-11);
        EXPECT_NEAR(lon, g.lon, 1e-11);
        EXPECT_NEAR(h, g.h, 1e-4);
    }
    // straight up from the pad is elevation 90 degrees
    double x[3], up[3], az, el;
    geo_to_ecef(0.6888, -1.3141, 0.0, x);
    up[0] = std::cos(0.6888) * std::cos(-1.3141);
    up[1] = std::cos(0.6888) * std::sin(-1.3141);
    up[2] = std::sin(0.6888);
    gnss_az_el(0.6888, -1.3141, up, &az, &el);
    EXPECT_NEAR(el, M_PI / 2.0, 1e-9);
}

TEST(GnssRawEphemerisStore, SelectsTheNearestHealthyEphemeris)
{
    EphemerisStore store;
    for (int i = 0; i < kGoldenEphN; ++i)
        EXPECT_TRUE(store.add(decodeGolden(kGoldenEph[i])));
    EXPECT_EQ(store.size(), static_cast<size_t>(kGoldenEphN));
    EXPECT_EQ(store.satellites(GNSS_SYS_GPS), 3);
    EXPECT_EQ(store.satellites(GNSS_SYS_GAL), 3);
    EXPECT_EQ(store.satellites(GNSS_SYS_BDS), 3);

    const GoldenEph& g = kGoldenEph[0];
    gps_eph_t e = decodeGolden(g);
    EXPECT_FALSE(store.add(e));                               // a repeat is ignored
    ASSERT_NE(store.select(g.sys, g.prn, g.toe + 600.0), nullptr);
    EXPECT_EQ(store.select(g.sys, g.prn, g.toe + 600.0)->toe, g.toe);
    EXPECT_EQ(store.select(g.sys, g.prn, g.toe + 7300.0), nullptr);     // past 2 h
    EXPECT_EQ(store.select(g.sys, 31, g.toe), nullptr);                // a satellite never filed
    EXPECT_EQ(store.select(GNSS_SYS_GAL, g.prn, g.toe), nullptr);      // the PRN under another system

    // the next upload: selected for times nearer it, the old one kept for the epochs before
    gps_eph_t next = e;
    next.toe = e.toe + 7200.0;
    next.iode = (e.iode + 1) & 0xFF;
    EXPECT_TRUE(store.add(next));
    EXPECT_EQ(store.select(g.sys, g.prn, e.toe + 3000.0)->toe, e.toe);
    EXPECT_EQ(store.select(g.sys, g.prn, e.toe + 4000.0)->toe, next.toe);

    // a third replaces the oldest; one older than both is refused
    gps_eph_t third = next;
    third.toe = next.toe + 7200.0;
    third.iode = (next.iode + 1) & 0xFF;
    EXPECT_TRUE(store.add(third));
    EXPECT_EQ(store.select(g.sys, g.prn, e.toe), nullptr);
    gps_eph_t stale = e;
    stale.toe = e.toe - 7200.0;
    stale.iode = (e.iode + 7) & 0xFF;
    EXPECT_FALSE(store.add(stale));

    // an unhealthy one is never selected
    gps_eph_t sick = decodeGolden(kGoldenEph[1]);
    sick.toe += 7200.0;
    sick.iode = (sick.iode + 1) & 0xFF;
    sick.health = 1;
    EXPECT_TRUE(store.add(sick));
    EXPECT_EQ(store.select(sick.sys, sick.prn, sick.toe), nullptr);
}

TEST(GnssRawEphemerisStore, AFullPoolDropsTheEphemerisFurthestInTime)
{
    EphemerisStore store;
    gps_eph_t e = decodeGolden(kGoldenEph[0]);
    for (size_t i = 0; i < EphemerisStore::kCapacity; ++i)
    {
        e.prn = static_cast<int>(i % 32) + 1;
        e.sys = i < 32 ? GNSS_SYS_GPS : GNSS_SYS_GAL;
        e.toe = 7200.0 * (i % 5);
        EXPECT_TRUE(store.add(e));
    }
    EXPECT_EQ(store.size(), EphemerisStore::kCapacity);
    gps_eph_t late = e;
    late.sys = GNSS_SYS_BDS;
    late.prn = 20;
    late.toe = 30000.0;
    EXPECT_TRUE(store.add(late));
    EXPECT_EQ(store.size(), EphemerisStore::kCapacity);
    EXPECT_NE(store.select(GNSS_SYS_BDS, 20, 30000.0), nullptr);
    // the dropped one had toe 0, the furthest from 30000
    int with_toe0 = 0;
    for (int prn = 1; prn <= 32; ++prn)
        with_toe0 += store.select(GNSS_SYS_GPS, prn, 0.0) != nullptr ? 1 : 0;
    EXPECT_EQ(with_toe0, 6);
}
