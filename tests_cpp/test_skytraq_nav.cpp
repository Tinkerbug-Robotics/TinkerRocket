// Host tests for the PX1105R's navigation-message assembler
// (TR_GNSSReceiverPX1105R_Serial/SkyTraqNav.cpp): 0xE0 GPS LNAV subframes,
// 0xE6 Galileo I/NAV page pairs and 0xE2 BeiDou D1 subframes to gps_eph_t,
// and GPS subframe 4 page 18 to the 8 Klobuchar bytes.
//
//   - Goldens from real data: the frames that completed nine satellites'
//     ephemerides on the 09-26 outdoor capture (fixtures/skytraq_nav_frames.h)
//     must decode to the bench prototype's values (gnss_raw.py EphStore)
//     field for field, and place each satellite on its system's orbit.
//   - What is refused: bad preamble, a GPS cutover, a Galileo CRC failure,
//     alert and malformed pages, a BeiDou GEO, subframes of two frames, short
//     frames; a GPS ephemeris held while the week is unknown, then given.
//   - With TR_SKYTRAQ_NAV_CAPTURE naming the capture (.log or .log.gz), the
//     whole of it: the same counts and issues as the prototype, and every
//     satellite's elevation and azimuth against the receiver's own (0xE8),
//     and its pseudoranges against the receiver's own fix (0xDF): metres,
//     so every orbit and clock field is checked. Statistics only, never a
//     position. TR_SKYTRAQ_NAV_VERBOSE prints them per satellite;
//     TR_SKYTRAQ_NAV_DUMP names a file to write every distinct ephemeris to
//     (for a field-by-field comparison with the prototype's dump, outside
//     the repository: all 728 fields of its 28 bit-identical on 2026-10-21).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "EphemerisStore.h"
#include "SkyTraqNav.h"
#include "fixtures/skytraq_nav_frames.h"
#include "gnss/eph.h"
#include "gnss/types.h"

using namespace skytraq;
namespace fx = skytraq_nav_fixture;

namespace {

std::vector<uint8_t> hex(const char* s)
{
    std::vector<uint8_t> v;
    for (size_t i = 0; s[i] && s[i + 1]; i += 2)
    {
        char b[3] = {s[i], s[i + 1], 0};
        v.push_back(static_cast<uint8_t>(std::strtoul(b, nullptr, 16)));
    }
    return v;
}

uint32_t getbitu(const uint8_t* b, size_t pos, int len)
{
    uint32_t v = 0;
    for (int i = 0; i < len; ++i, ++pos) v = (v << 1) | ((b[pos >> 3] >> (7 - (pos & 7))) & 1u);
    return v;
}

void setbitu(uint8_t* b, size_t pos, int len, uint32_t v)
{
    for (int i = 0; i < len; ++i, ++pos)
    {
        const uint8_t m = static_cast<uint8_t>(0x80u >> (pos & 7));
        b[pos >> 3] = ((v >> (len - 1 - i)) & 1u) ? static_cast<uint8_t>(b[pos >> 3] | m)
                                                   : static_cast<uint8_t>(b[pos >> 3] & ~m);
    }
}

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

// After editing a Galileo page pair (an 0xE6 body), write its CRC-24Q again
// (Galileo OS SIS ICD 4.3.2.2: even page bits 0-113 and odd page bits 0-81).
void resealGalPage(std::vector<uint8_t>& body)
{
    uint8_t* buf = body.data() + 5;
    uint8_t c[26] = {};
    for (size_t i = 0; i < 114; ++i) setbitu(c, 4 + i, 1, getbitu(buf, i, 1));
    for (size_t i = 0; i < 82; ++i) setbitu(c, 118 + i, 1, getbitu(buf, 128 + i, 1));
    setbitu(buf, 128 + 82, 24, crc24q(c, 25));
}

// Bit positions inside a body: GPS words start at body byte 3, a Galileo
// word's 128 bits are the even page's 2-113 and the odd page's 130-145 from
// body byte 5, a BeiDou subframe's packed data starts at body byte 3.
constexpr size_t kGpsBit = 24;      // body bit of subframe bit 0
constexpr size_t kGalBit = 40 + 2;  // body bit of word bit 0 (bits 0-111 of a word)

void expectEph(const gps_eph_t& want, const gps_eph_t& got, const char* label)
{
    SCOPED_TRACE(label);
    EXPECT_EQ(got.valid, 1);
    EXPECT_EQ(got.sys, want.sys);
    EXPECT_EQ(got.prn, want.prn);
    EXPECT_EQ(got.week, want.week);
    EXPECT_EQ(got.iodc, want.iodc);
    EXPECT_EQ(got.iode, want.iode);
    EXPECT_EQ(got.health, want.health);
    EXPECT_EQ(got.ura, want.ura);
    // The same integers times the same scales in the same order as the
    // prototype: bit for bit, not just close.
    EXPECT_EQ(got.toe, want.toe);
    EXPECT_EQ(got.toc, want.toc);
    EXPECT_EQ(got.sqrt_a, want.sqrt_a);
    EXPECT_EQ(got.e, want.e);
    EXPECT_EQ(got.i0, want.i0);
    EXPECT_EQ(got.omega0, want.omega0);
    EXPECT_EQ(got.omega, want.omega);
    EXPECT_EQ(got.m0, want.m0);
    EXPECT_EQ(got.delta_n, want.delta_n);
    EXPECT_EQ(got.idot, want.idot);
    EXPECT_EQ(got.omega_dot, want.omega_dot);
    EXPECT_EQ(got.cuc, want.cuc);
    EXPECT_EQ(got.cus, want.cus);
    EXPECT_EQ(got.crc, want.crc);
    EXPECT_EQ(got.crs, want.crs);
    EXPECT_EQ(got.cic, want.cic);
    EXPECT_EQ(got.cis, want.cis);
    EXPECT_EQ(got.af0, want.af0);
    EXPECT_EQ(got.af1, want.af1);
    EXPECT_EQ(got.af2, want.af2);
    EXPECT_EQ(got.tgd, want.tgd);
}

// Feed a golden's frames to `nav`; the result of the last.
NavStatus feed(NavAssembler& nav, const fx::Golden& g, int week, gps_eph_t& eph)
{
    int8_t iono[8];
    NavStatus st = NavStatus::NOT_NAV;
    for (int k = 0; k < g.n_frames; ++k)
    {
        const std::vector<uint8_t> b = hex(g.frames[k]);
        st = nav.add(b.data(), b.size(), week, eph, iono);
        if (k + 1 < g.n_frames) EXPECT_EQ(st, NavStatus::PART) << g.label << " frame " << k;
    }
    return st;
}

const fx::Golden& golden(const char* label)
{
    for (const auto& g : fx::kGoldens)
        if (std::strcmp(g.label, label) == 0) return g;
    ADD_FAILURE() << "no golden " << label;
    return fx::kGoldens[0];
}

NavStatus addBody(NavAssembler& nav, const std::vector<uint8_t>& b, int week = fx::kWeek)
{
    gps_eph_t eph{};
    int8_t iono[8];
    return nav.add(b.data(), b.size(), week, eph, iono);
}

}  // namespace

// ── Goldens from the 09-26 capture ───────────────────────────────────────────

TEST(SkyTraqNav, GoldensMatchThePrototype)
{
    for (const auto& g : fx::kGoldens)
    {
        NavAssembler nav;
        gps_eph_t eph{};
        ASSERT_EQ(feed(nav, g, fx::kWeek, eph), NavStatus::EPHEMERIS) << g.label;
        expectEph(g.expect(), eph, g.label);
        EXPECT_EQ(eph.sqrt_a * eph.sqrt_a, g.py_A) << g.label;   // the prototype keeps A
        EXPECT_EQ(nav.counters().ephemerides[eph.sys], 1u);
        EXPECT_EQ(nav.counters().issues[eph.sys], 1u);
    }
}

TEST(SkyTraqNav, GoldensFlyOnTheirSystemsOrbits)
{
    // Semi-major axes (km): GPS 26559.7, Galileo 29600.3, BeiDou MEO 27906.1,
    // IGSO 42164.2. At toe the satellite is within a(1 +- e) of the centre
    // (the harmonic corrections are a few hundred metres), and an MEO's
    // speed in ECEF is 2.6-4.0 km/s.
    for (const auto& g : fx::kGoldens)
    {
        NavAssembler nav;
        gps_eph_t eph{};
        ASSERT_EQ(feed(nav, g, fx::kWeek, eph), NavStatus::EPHEMERIS);
        const double a_km = eph.sqrt_a * eph.sqrt_a / 1e3;
        double nominal = eph.sys == GNSS_SYS_GPS ? 26559.7 : eph.sys == GNSS_SYS_GAL ? 29600.3 : 27906.1;
        if (eph.sys == GNSS_SYS_BDS && a_km > 35000.0) nominal = 42164.2;
        EXPECT_NEAR(a_km, nominal, 100.0) << g.label;
        double pos[3], vel[3], clk = 0.0;
        gps_sat_pos(&eph, eph.toe, pos, vel, &clk);
        const double r_km = std::sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]) / 1e3;
        EXPECT_GT(r_km, a_km * (1.0 - eph.e) - 1.0) << g.label;
        EXPECT_LT(r_km, a_km * (1.0 + eph.e) + 1.0) << g.label;
        EXPECT_NEAR(r_km, nominal, 100.0 + a_km * eph.e) << g.label;
        const double v_kms = std::sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]) / 1e3;
        EXPECT_GT(v_kms, 2.0) << g.label;
        EXPECT_LT(v_kms, 4.5) << g.label;
        // GPS holds its clocks within 1 ms of system time, Galileo not (E11's
        // af0 is 5.8 ms); the capture test checks each clock to metres.
        EXPECT_LT(std::fabs(clk), 1e-2) << g.label;
    }
}

TEST(SkyTraqNav, EphemerisStoreTakesThem)
{
    gnss_raw::EphemerisStore store;
    for (const auto& g : fx::kGoldens)
    {
        NavAssembler nav;
        gps_eph_t eph{};
        ASSERT_EQ(feed(nav, g, fx::kWeek, eph), NavStatus::EPHEMERIS);
        EXPECT_TRUE(store.add(eph)) << g.label;
        EXPECT_FALSE(store.add(eph)) << g.label;   // a repeat of the issue is ignored
        EXPECT_NE(store.select(eph.sys, eph.prn, eph.toe + 600.0), nullptr) << g.label;
    }
}

TEST(SkyTraqNav, OrderOfArrivalAndRepeats)
{
    // GPS: subframe 3 before 1 is filed but completes nothing; the next 3
    // completes it. Each completion returns the ephemeris; one issue counted.
    const auto& g = golden("G32 IOD 120");
    const auto s1 = hex(g.frames[0]), s2 = hex(g.frames[1]), s3 = hex(g.frames[2]);
    NavAssembler nav;
    EXPECT_EQ(addBody(nav, s2), NavStatus::PART);
    EXPECT_EQ(addBody(nav, s3), NavStatus::PART);
    EXPECT_EQ(addBody(nav, s1), NavStatus::PART);
    EXPECT_EQ(addBody(nav, s3), NavStatus::EPHEMERIS);
    EXPECT_EQ(addBody(nav, s3), NavStatus::EPHEMERIS);
    EXPECT_EQ(nav.counters().ephemerides[GNSS_SYS_GPS], 2u);
    EXPECT_EQ(nav.counters().issues[GNSS_SYS_GPS], 1u);
    EXPECT_EQ(nav.counters().frames[GNSS_SYS_GPS], 5u);
    EXPECT_EQ(nav.counters().status[static_cast<size_t>(NavStatus::PART)], 3u);

    // Galileo: word type 5 before the rest completes nothing.
    const auto& ge = golden("E11 IOD 58");
    NavAssembler nav2;
    EXPECT_EQ(addBody(nav2, hex(ge.frames[4])), NavStatus::PART);
    gps_eph_t eph{};
    EXPECT_EQ(feed(nav2, ge, fx::kWeek, eph), NavStatus::EPHEMERIS);

    // Satellites do not mix: G23's subframe 3 does not complete G32.
    NavAssembler nav3;
    EXPECT_EQ(addBody(nav3, s1), NavStatus::PART);
    EXPECT_EQ(addBody(nav3, s2), NavStatus::PART);
    EXPECT_EQ(addBody(nav3, hex(golden("G23 IOD 144").frames[2])), NavStatus::PART);

    nav.reset();
    EXPECT_EQ(nav.counters().frames[GNSS_SYS_GPS], 0u);
    EXPECT_EQ(addBody(nav, s3), NavStatus::PART);   // reset forgot subframes 1 and 2
}

// ── Ionosphere ───────────────────────────────────────────────────────────────

TEST(SkyTraqNav, IonosphereFromSubframe4Page18)
{
    static constexpr double kScale[8] = {
        9.313225746154785e-10, 7.450580596923828e-09, 5.960464477539063e-08, 5.960464477539063e-08,
        2048.0, 16384.0, 65536.0, 65536.0};   // 2^-30, 2^-27, 2^-24, 2^-24, 2^11, 2^14, 2^16, 2^16
    const auto b = hex(fx::kIonoFrame);
    NavAssembler nav;
    gps_eph_t eph{};
    int8_t iono[8] = {};
    ASSERT_EQ(nav.add(b.data(), b.size(), fx::kWeek, eph, iono), NavStatus::IONO);
    for (int k = 0; k < 8; ++k)
    {
        EXPECT_EQ(iono[k], fx::kIonoRaw[k]) << k;
        EXPECT_EQ(iono[k] * kScale[k], fx::kPyIon[k]) << k;   // the prototype's EphStore.ion
    }
    EXPECT_EQ(nav.ionoPrn(), fx::kIonoPrn);
    EXPECT_EQ(nav.ionoTowS(), fx::kIonoTowS);
    EXPECT_EQ(nav.ionoGpsDay(), fx::kIonoGpsDay);
    EXPECT_EQ(nav.counters().iono, 1u);

    // Without a week the parameters still come, the day is unknown.
    NavAssembler nav2;
    int8_t iono2[8] = {};
    ASSERT_EQ(nav2.add(b.data(), b.size(), 0, eph, iono2), NavStatus::IONO);
    EXPECT_EQ(std::memcmp(iono, iono2, 8), 0);
    EXPECT_EQ(nav2.ionoGpsDay(), 0u);

    // Another subframe 4 page (SV ID 57 here) is not the ionosphere.
    auto other = b;
    setbitu(other.data(), kGpsBit + 50, 6, 57);
    int8_t untouched[8] = {1, 1, 1, 1, 1, 1, 1, 1};
    EXPECT_EQ(nav.add(other.data(), other.size(), fx::kWeek, eph, untouched), NavStatus::UNUSED);
    EXPECT_EQ(untouched[0], 1);
}

// ── GPS week ─────────────────────────────────────────────────────────────────

TEST(SkyTraqNav, GpsWaitsForTheWeekThenCompletes)
{
    const auto& g = golden("G32 IOD 120");
    NavAssembler nav;
    gps_eph_t eph{};
    eph.prn = -1;
    EXPECT_EQ(feed(nav, g, 0, eph), NavStatus::NO_WEEK);
    EXPECT_EQ(eph.prn, -1);                                         // untouched
    EXPECT_EQ(nav.takePending(0, eph), NavStatus::NO_WEEK);         // still unknown
    ASSERT_EQ(nav.takePending(fx::kWeek, eph), NavStatus::EPHEMERIS);
    expectEph(g.expect(), eph, g.label);
    EXPECT_EQ(nav.takePending(fx::kWeek, eph), NavStatus::PART);    // given once
    EXPECT_EQ(nav.counters().status[static_cast<size_t>(NavStatus::NO_WEEK)], 1u);

    // Or the satellite's next subframe 3, once the week is known, completes it
    // and nothing stays pending.
    NavAssembler nav2;
    EXPECT_EQ(feed(nav2, g, 0, eph), NavStatus::NO_WEEK);
    EXPECT_EQ(addBody(nav2, hex(g.frames[2]), fx::kWeek), NavStatus::EPHEMERIS);
    EXPECT_EQ(nav2.takePending(fx::kWeek, eph), NavStatus::PART);

    // Galileo and BeiDou carry their own week: no wait.
    for (const char* label : {"E11 IOD 58", "C41 IOD 1"})
    {
        NavAssembler nav3;
        EXPECT_EQ(feed(nav3, golden(label), 0, eph), NavStatus::EPHEMERIS) << label;
        EXPECT_EQ(eph.week, fx::kWeek) << label;
    }
}

TEST(SkyTraqNav, GpsWeekIsTheWeekOfToe)
{
    // An ephemeris sent in the last minutes of a week whose toe is in the
    // next: G32 with its HOW at TOW count 100799 (604794 s) and toe 3600 s.
    const auto& g = golden("G32 IOD 120");
    auto s1 = hex(g.frames[0]), s2 = hex(g.frames[1]), s3 = hex(g.frames[2]);
    setbitu(s1.data(), kGpsBit + 24, 17, 100799);
    setbitu(s2.data(), kGpsBit + 216, 16, 3600 / 16);
    NavAssembler nav;
    addBody(nav, s1);
    addBody(nav, s2);
    gps_eph_t eph{};
    int8_t iono[8];
    ASSERT_EQ(nav.add(s3.data(), s3.size(), fx::kWeek, eph, iono), NavStatus::EPHEMERIS);
    EXPECT_EQ(eph.toe, 3600.0);
    EXPECT_EQ(eph.week, fx::kWeek + 1);

    // And one sent early in a week whose toe was late in the last.
    setbitu(s1.data(), kGpsBit + 24, 17, 100);
    setbitu(s2.data(), kGpsBit + 216, 16, 604000 / 16);
    addBody(nav, s1);
    addBody(nav, s2);
    ASSERT_EQ(nav.add(s3.data(), s3.size(), fx::kWeek, eph, iono), NavStatus::EPHEMERIS);
    EXPECT_EQ(eph.week, fx::kWeek - 1);
}

// ── Refusals ─────────────────────────────────────────────────────────────────

TEST(SkyTraqNav, GpsCutoverIsRefused)
{
    const auto& g = golden("G32 IOD 120");
    const auto s1 = hex(g.frames[0]), s2 = hex(g.frames[1]), s3 = hex(g.frames[2]);
    // Subframe 3 of a new issue (IODE 121) after subframes 1 and 2 of the old.
    auto s3_new = s3;
    setbitu(s3_new.data(), kGpsBit + 216, 8, 121);
    NavAssembler nav;
    addBody(nav, s1);
    addBody(nav, s2);
    EXPECT_EQ(addBody(nav, s3_new), NavStatus::IOD_MISMATCH);
    EXPECT_EQ(addBody(nav, s3), NavStatus::EPHEMERIS);   // the old issue's 3 completes it again

    // Subframe 1 of a new issue (IODC low byte 121).
    auto s1_new = s1;
    setbitu(s1_new.data(), kGpsBit + 168, 8, 121);
    addBody(nav, s1_new);
    EXPECT_EQ(addBody(nav, s3), NavStatus::IOD_MISMATCH);
    // The check is on IODC's low byte only (IS-GPS-200 20.3.4.4; the prototype and RTKLIB alike).
    auto s1_hi = s1;
    setbitu(s1_hi.data(), kGpsBit + 70, 2, 3);
    addBody(nav, s1_hi);
    gps_eph_t eph{};
    int8_t iono[8];
    ASSERT_EQ(nav.add(s3.data(), s3.size(), fx::kWeek, eph, iono), NavStatus::EPHEMERIS);
    EXPECT_EQ(eph.iodc, 0x300 | 120);
    EXPECT_EQ(nav.counters().status[static_cast<size_t>(NavStatus::IOD_MISMATCH)], 2u);

    // A cutover held while the week is unknown is not given later.
    NavAssembler nav2;
    addBody(nav2, s1, 0);
    addBody(nav2, s2, 0);
    EXPECT_EQ(addBody(nav2, s3, 0), NavStatus::NO_WEEK);
    addBody(nav2, s1_new, 0);
    EXPECT_EQ(nav2.takePending(fx::kWeek, eph), NavStatus::IOD_MISMATCH);
    EXPECT_EQ(nav2.takePending(fx::kWeek, eph), NavStatus::PART);
}

TEST(SkyTraqNav, BadPreamble)
{
    auto gps = hex(golden("G32 IOD 120").frames[0]);
    gps[3] = 0x8A;
    NavAssembler nav;
    EXPECT_EQ(addBody(nav, gps), NavStatus::BAD_PREAMBLE);
    auto bds = hex(golden("C41 IOD 1").frames[0]);
    bds[3] ^= 0x80;
    EXPECT_EQ(addBody(nav, bds), NavStatus::BAD_PREAMBLE);
    EXPECT_EQ(nav.counters().status[static_cast<size_t>(NavStatus::BAD_PREAMBLE)], 2u);
}

TEST(SkyTraqNav, BadSubframeAndPrn)
{
    NavAssembler nav;
    auto gps = hex(golden("G32 IOD 120").frames[0]);
    for (uint32_t id : {0u, 6u, 7u})
    {
        auto b = gps;
        setbitu(b.data(), kGpsBit + 43, 3, id);
        EXPECT_EQ(addBody(nav, b), NavStatus::BAD_SUBFRAME) << id;
    }
    for (int prn : {0, 33, 193})   // QZSS's 193 is not taken
    {
        auto b = gps;
        b[1] = static_cast<uint8_t>(prn);
        EXPECT_EQ(addBody(nav, b), NavStatus::BAD_PRN) << prn;
    }
    auto gal = hex(golden("E11 IOD 58").frames[0]);
    for (int prn : {0, 37})
    {
        auto b = gal;
        b[3] = static_cast<uint8_t>(prn);
        EXPECT_EQ(addBody(nav, b), NavStatus::BAD_PRN) << prn;
    }
    auto bds = hex(golden("C41 IOD 1").frames[0]);
    auto b = bds;
    b[2] = 6;
    EXPECT_EQ(addBody(nav, b), NavStatus::BAD_SUBFRAME);
    b = bds;
    b[2] = 2;   // the FraID in word 1 says 1
    EXPECT_EQ(addBody(nav, b), NavStatus::BAD_SUBFRAME);
    b = bds;
    b[1] = 199;   // PRN -1
    EXPECT_EQ(addBody(nav, b), NavStatus::BAD_PRN);
    b[1] = 200;   // PRN 0
    EXPECT_EQ(addBody(nav, b), NavStatus::BAD_PRN);
}

TEST(SkyTraqNav, BeidouGeoIsRefused)
{
    // C01-C05 send D2; the 0xE2 cannot even number C59-C63 (PRN + 200 in a byte).
    auto bds = hex(golden("C41 IOD 1").frames[0]);
    NavAssembler nav;
    for (int prn = 1; prn <= 5; ++prn)
    {
        auto b = bds;
        b[1] = static_cast<uint8_t>(200 + prn);
        EXPECT_EQ(addBody(nav, b), NavStatus::GEO) << prn;
    }
    bds[1] = 206;   // C06, an IGSO
    EXPECT_EQ(addBody(nav, bds), NavStatus::PART);
    EXPECT_EQ(nav.counters().status[static_cast<size_t>(NavStatus::GEO)], 5u);
}

TEST(SkyTraqNav, BeidouSubframesOfTwoFramesAreRefused)
{
    const auto& g = golden("C41 IOD 1");
    const auto s1 = hex(g.frames[0]), s2 = hex(g.frames[1]), s3 = hex(g.frames[2]);
    // Subframe 2 from the next frame (its SOW 30 s on; SOW bits 18-25 and
    // 26-37 of the packed data, contiguous there).
    auto s2_late = s2;
    const uint32_t sow2 = getbitu(s2.data() + 3, 18, 20);
    setbitu(s2_late.data() + 3, 18, 20, sow2 + 30);
    NavAssembler nav;
    addBody(nav, s1);
    addBody(nav, s2_late);
    EXPECT_EQ(addBody(nav, s3), NavStatus::FRAME_MISMATCH);
    addBody(nav, s2);
    EXPECT_EQ(addBody(nav, s3), NavStatus::EPHEMERIS);

    // toc (subframe 1, packed bits 26 + 22 + 13 .. : RTKLIB's 73 and 90) not toe.
    auto s1_toc = s1;
    const size_t toc_lo = 26 + 22 * 2;   // RTKLIB bit 90: word 4, first data bit
    setbitu(s1_toc.data() + 3, toc_lo, 8, getbitu(s1.data() + 3, toc_lo, 8) ^ 1u);
    NavAssembler nav2;
    addBody(nav2, s1_toc);
    addBody(nav2, s2);
    EXPECT_EQ(addBody(nav2, s3), NavStatus::FRAME_MISMATCH);
    EXPECT_EQ(nav2.counters().status[static_cast<size_t>(NavStatus::FRAME_MISMATCH)], 1u);
}

TEST(SkyTraqNav, GalileoCrcFailure)
{
    const auto& g = golden("E11 IOD 58");
    NavAssembler nav;
    // A flipped bit in the data (word 3's IODnav), in the odd page's data
    // and in the CRC itself.
    for (size_t bit : {size_t(40 + 2 + 8), size_t(40 + 130), size_t(40 + 128 + 82 + 5)})
    {
        auto b = hex(g.frames[3]);
        b[bit >> 3] ^= static_cast<uint8_t>(0x80u >> (bit & 7));
        EXPECT_EQ(addBody(nav, b), NavStatus::CRC_FAIL) << bit;
    }
    EXPECT_EQ(nav.counters().status[static_cast<size_t>(NavStatus::CRC_FAIL)], 3u);
    // Resealed, the edit passes the CRC: the helper below is right.
    auto b = hex(g.frames[3]);
    b[(40 + 130) >> 3] ^= static_cast<uint8_t>(0x80u >> ((40 + 130) & 7));
    resealGalPage(b);
    EXPECT_EQ(addBody(nav, b), NavStatus::PART);
    // And a CRC-failed word is not filed: the good frames still give the golden.
    gps_eph_t eph{};
    NavAssembler nav2;
    auto bad = hex(g.frames[4]);
    bad[10] ^= 0x01;   // even page bit 47, inside the CRC (bits 120-127 of each half are padding)
    EXPECT_EQ(addBody(nav2, bad), NavStatus::CRC_FAIL);
    ASSERT_EQ(feed(nav2, g, fx::kWeek, eph), NavStatus::EPHEMERIS);
    expectEph(g.expect(), eph, g.label);
}

TEST(SkyTraqNav, GalileoAlertAndMalformedPages)
{
    const auto& g = golden("E11 IOD 58");
    NavAssembler nav;
    auto alert = hex(g.frames[0]);
    setbitu(alert.data(), 40 + 1, 1, 1);   // even page type: alert
    resealGalPage(alert);
    EXPECT_EQ(addBody(nav, alert), NavStatus::ALERT_PAGE);
    auto odd_first = hex(g.frames[0]);
    setbitu(odd_first.data(), 40 + 0, 1, 1);   // "even" page marked odd
    resealGalPage(odd_first);
    EXPECT_EQ(addBody(nav, odd_first), NavStatus::BAD_PAGE);
    // Word type 0 (spare) and 6 are good pages with nothing needed here.
    auto w0 = hex(g.frames[0]);
    setbitu(w0.data(), kGalBit, 6, 0);
    resealGalPage(w0);
    EXPECT_EQ(addBody(nav, w0), NavStatus::UNUSED);
    auto w6 = hex(g.frames[0]);
    setbitu(w6.data(), kGalBit, 6, 6);
    resealGalPage(w6);
    EXPECT_EQ(addBody(nav, w6), NavStatus::UNUSED);
}

TEST(SkyTraqNav, GalileoIssuesMustAgree)
{
    const auto& g = golden("E11 IOD 58");
    // Word type 3 of another IODnav (59), resealed.
    std::vector<std::vector<uint8_t>> f;
    for (int k = 0; k < g.n_frames; ++k) f.push_back(hex(g.frames[k]));
    int w3 = -1, w4 = -1;
    for (int k = 0; k < g.n_frames; ++k)
    {
        const uint32_t t = getbitu(f[k].data(), kGalBit, 6);
        if (t == 3) w3 = k;
        if (t == 4) w4 = k;
    }
    ASSERT_GE(w3, 0);
    ASSERT_GE(w4, 0);
    {
        auto ff = f;
        setbitu(ff[w3].data(), kGalBit + 6, 10, 59);
        resealGalPage(ff[w3]);
        NavAssembler nav;
        NavStatus st = NavStatus::NOT_NAV;
        for (const auto& b : ff) st = addBody(nav, b);
        EXPECT_EQ(st, NavStatus::IOD_MISMATCH);
    }
    {
        // Word type 4's SVID not the page's satellite.
        auto ff = f;
        setbitu(ff[w4].data(), kGalBit + 16, 6, 12);
        resealGalPage(ff[w4]);
        NavAssembler nav;
        NavStatus st = NavStatus::NOT_NAV;
        for (const auto& b : ff) st = addBody(nav, b);
        EXPECT_EQ(st, NavStatus::SVID_MISMATCH);
    }
}

TEST(SkyTraqNav, ShortAndForeignMessages)
{
    NavAssembler nav;
    const auto gps = hex(golden("G32 IOD 120").frames[0]);
    const auto gal = hex(golden("E11 IOD 58").frames[0]);
    const auto bds = hex(golden("C41 IOD 1").frames[0]);
    ASSERT_EQ(gps.size(), kNavGpsLen);
    ASSERT_EQ(gal.size(), kNavGalLen);
    ASSERT_EQ(bds.size(), kNavBdsLen);
    gps_eph_t eph{};
    int8_t iono[8];
    EXPECT_EQ(nav.add(gps.data(), gps.size() - 1, fx::kWeek, eph, iono), NavStatus::SHORT);
    EXPECT_EQ(nav.add(gal.data(), gal.size() - 1, fx::kWeek, eph, iono), NavStatus::SHORT);
    EXPECT_EQ(nav.add(bds.data(), bds.size() - 1, fx::kWeek, eph, iono), NavStatus::SHORT);
    EXPECT_EQ(nav.add(gal.data(), 2, fx::kWeek, eph, iono), NavStatus::SHORT);
    EXPECT_EQ(nav.add(gps.data(), 0, fx::kWeek, eph, iono), NavStatus::NOT_NAV);
    EXPECT_EQ(nav.add(nullptr, 33, fx::kWeek, eph, iono), NavStatus::NOT_NAV);
    const uint8_t e5[14] = {0xE5, 1};
    EXPECT_EQ(nav.add(e5, sizeof(e5), fx::kWeek, eph, iono), NavStatus::NOT_NAV);
    // An 0xE6 of another sub-type (E5a, signal type 4) is not an I/NAV page.
    auto e6_e5a = gal;
    e6_e5a[2] = 0x43;
    EXPECT_EQ(nav.add(e6_e5a.data(), e6_e5a.size(), fx::kWeek, eph, iono), NavStatus::NOT_NAV);
    EXPECT_EQ(nav.counters().status[static_cast<size_t>(NavStatus::SHORT)], 4u);
    EXPECT_EQ(nav.counters().status[static_cast<size_t>(NavStatus::NOT_NAV)], 4u);
    // Longer bodies are fine (only the message's own bytes are read).
    auto longer = gps;
    longer.push_back(0);
    EXPECT_EQ(addBody(nav, longer), NavStatus::PART);

    EXPECT_TRUE(isNavMessage(gps.data(), gps.size()));
    EXPECT_TRUE(isNavMessage(gal.data(), gal.size()));
    EXPECT_TRUE(isNavMessage(bds.data(), bds.size()));
    EXPECT_FALSE(isNavMessage(e6_e5a.data(), e6_e5a.size()));
    EXPECT_FALSE(isNavMessage(gal.data(), 2));
    EXPECT_FALSE(isNavMessage(e5, sizeof(e5)));
    EXPECT_FALSE(isNavMessage(nullptr, 10));
}

TEST(SkyTraqNav, StatusNames)
{
    std::set<std::string> names;
    for (int s = 0; s < static_cast<int>(NavStatus::COUNT); ++s)
    {
        const std::string n = navStatusName(static_cast<NavStatus>(s));
        EXPECT_NE(n, "?") << s;
        names.insert(n);
    }
    EXPECT_EQ(names.size(), static_cast<size_t>(NavStatus::COUNT));
}

TEST(SkyTraqNav, Footprint)
{
    // Fixed tables for GPS 1-32, Galileo 1-36 and BeiDou 6-58: ~11 KB.
    EXPECT_LT(sizeof(NavAssembler), 12u * 1024u);
    std::printf("[ info ] sizeof(NavAssembler) = %zu bytes\n", sizeof(NavAssembler));
}

// ── The whole 09-26 capture (TR_SKYTRAQ_NAV_CAPTURE) ─────────────────────────

namespace {

double be64f(const uint8_t* p)
{
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v = (v << 8) | p[i];
    double d;
    std::memcpy(&d, &v, 8);
    return d;
}

// Elevation and azimuth (deg) of sat from rx (ECEF, m), on the WGS-84 ellipsoid.
void elAz(const double rx[3], const double sat[3], double& el, double& az)
{
    const double a = 6378137.0, f = 1.0 / 298.257223563, e2 = f * (2.0 - f);
    const double p = std::hypot(rx[0], rx[1]);
    double lat = std::atan2(rx[2], p * (1.0 - e2));
    for (int i = 0; i < 6; ++i)
    {
        const double n = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
        const double h = p / std::cos(lat) - n;
        lat = std::atan2(rx[2], p * (1.0 - e2 * n / (n + h)));
    }
    const double lon = std::atan2(rx[1], rx[0]);
    const double d[3] = {sat[0] - rx[0], sat[1] - rx[1], sat[2] - rx[2]};
    const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
    const double east = -so * d[0] + co * d[1];
    const double north = -sl * co * d[0] - sl * so * d[1] + cl * d[2];
    const double up = cl * co * d[0] + cl * so * d[1] + sl * d[2];
    el = std::atan2(up, std::hypot(east, north)) * 180.0 / M_PI;
    az = std::atan2(east, north) * 180.0 / M_PI;
    if (az < 0.0) az += 360.0;
}

constexpr double kC = 299792458.0;

// A pseudorange's residual (m) against the receiver's own fix: the
// measurement less its clock bias, less the range to where the ephemeris puts
// the satellite at transmit (Earth rotation included), plus the satellite
// clock (relativity and group delay included). What is left is the
// atmosphere (+5..+30 m), noise and multipath: an ephemeris wrong in any
// orbit or clock field leaves kilometres.
double prResidual(const gps_eph_t& e, double t_rx_gps, const double rx[3], double pr_less_bias, double* el_deg)
{
    const double to_sys = e.sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0;
    const double t_sv = t_rx_gps + to_sys - pr_less_bias / kC;
    const double dts = gps_sat_clock(&e, t_sv);
    double pos[3], clk = 0.0;
    gps_sat_pos(&e, t_sv - dts, pos, nullptr, &clk);
    double tau = 0.07, r = 0.0, rs[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i)
    {
        const double th = (e.sys == GNSS_SYS_BDS ? BDS_OMEGA_E : GPS_OMEGA_E) * tau;
        rs[0] = std::cos(th) * pos[0] + std::sin(th) * pos[1];
        rs[1] = -std::sin(th) * pos[0] + std::cos(th) * pos[1];
        rs[2] = pos[2];
        r = std::sqrt((rs[0] - rx[0]) * (rs[0] - rx[0]) + (rs[1] - rx[1]) * (rs[1] - rx[1]) + (rs[2] - rx[2]) * (rs[2] - rx[2]));
        tau = r / kC;
    }
    double az;
    elAz(rx, rs, *el_deg, az);
    return pr_less_bias - r + kC * clk;
}

}  // namespace

TEST(SkyTraqNav, WholeCaptureMatchesThePrototype)
{
    const char* path = std::getenv("TR_SKYTRAQ_NAV_CAPTURE");
    if (path == nullptr || *path == 0) GTEST_SKIP() << "TR_SKYTRAQ_NAV_CAPTURE not set";
    const std::string p(path);
    const bool gz = p.size() > 3 && p.compare(p.size() - 3, 3, ".gz") == 0;
    FILE* f = gz ? popen(("gzip -dc '" + p + "'").c_str(), "r") : std::fopen(path, "r");
    ASSERT_NE(f, nullptr) << path;

    NavAssembler nav;
    int week = 0;
    double rx[3] = {0, 0, 0}, rx_tow = -1.0;
    std::set<std::tuple<int, int, int, double>> issues;
    std::map<std::pair<int, int>, gps_eph_t> latest;   // (sys, prn) -> its latest ephemeris
    std::vector<gps_eph_t> distinct;
    int iono_n = 0, first_eph_line[3] = {-1, -1, -1};
    double el_err_max[3] = {0, 0, 0}, az_err_max[3] = {0, 0, 0};
    int el_checks[3] = {0, 0, 0};
    std::vector<double> res[3];        // pseudorange residuals less their epoch's median, el >= 15 deg
    std::vector<double> res_raw[3];    //   and as they are
    std::map<std::pair<int, int>, std::vector<double>> res_sat;   // per satellite, less the epoch's median
    const bool verbose = std::getenv("TR_SKYTRAQ_NAV_VERBOSE") != nullptr;
    std::vector<uint8_t> e5;           // the last 0xE5, until its 0xDF
    std::vector<char> line(1 << 14);
    int ln = 0;
    while (std::fgets(line.data(), static_cast<int>(line.size()), f) != nullptr)
    {
        ++ln;
        char* sp1 = std::strchr(line.data(), ' ');
        if (sp1 == nullptr || sp1[1] != 'B' || sp1[2] != ' ') continue;
        char* h = sp1 + 3;
        h[std::strcspn(h, "\r\n")] = 0;
        const std::vector<uint8_t> b = hex(h);
        if (b.empty()) continue;
        if (b[0] == 0xE5 && b.size() >= 14)
        {
            week = (b[3] << 8) | b[4];
            e5 = b;
        }
        if (b[0] == 0xDF && b.size() >= 57 && b[2] >= 2)
        {
            rx_tow = be64f(&b[5]);   // GPS time: the 0xE5's time of week less the bias
            for (int k = 0; k < 3; ++k) rx[k] = be64f(&b[13 + 8 * k]);
            const double bias_m = be64f(&b[49]);
            // The 0xE5 of the same epoch (IOD [2] = 0xDF [1]): its L1-band
            // pseudoranges, one epoch in ten (the residuals barely change).
            if (!e5.empty() && e5[2] == b[1] && e5[2] % 10 == 0)
            {
                std::vector<std::tuple<int, int, double>> epoch;   // sys, prn, residual
                for (int j = 0; j < e5[13] && 14 + 31 * (j + 1) <= static_cast<int>(e5.size()); ++j)
                {
                    const uint8_t* c = &e5[14 + 31 * j];
                    const uint16_t ind = static_cast<uint16_t>((c[27] << 8) | c[28]);
                    const int sys = (c[0] >> 4) != 0 ? -1 : (c[0] & 0xF) == 0 ? GNSS_SYS_GPS : (c[0] & 0xF) == 3 ? GNSS_SYS_GAL
                                                                                  : (c[0] & 0xF) == 5 ? GNSS_SYS_BDS : -1;
                    const auto it = latest.find({sys, c[1]});
                    if (sys < 0 || !(ind & 1) || it == latest.end()) continue;
                    double el = 0.0;
                    const double r = prResidual(it->second, rx_tow, rx, be64f(c + 4) - bias_m, &el);
                    if (el >= 15.0) epoch.emplace_back(sys, c[1], r);
                }
                // The epoch's median takes out what all satellites share: the
                // receiver clock as the 0xDF gives it (the residuals sit
                // 30-45 m below zero on 09-26), and the mean atmosphere.
                if (epoch.size() >= 5)
                {
                    std::vector<double> v;
                    for (const auto& x : epoch) v.push_back(std::get<2>(x));
                    std::nth_element(v.begin(), v.begin() + static_cast<long>(v.size() / 2), v.end());
                    const double med = v[v.size() / 2];
                    for (const auto& x : epoch)
                    {
                        res[std::get<0>(x)].push_back(std::get<2>(x) - med);
                        res_raw[std::get<0>(x)].push_back(std::get<2>(x));
                        res_sat[{std::get<0>(x), std::get<1>(x)}].push_back(std::get<2>(x) - med);
                    }
                }
            }
            e5.clear();
        }
        if (b[0] == 0xE8 && b.size() >= 4 && rx_tow >= 0.0)
        {
            // [3] satellites, 6 B each: GNSS type, SVID, elevation u16, azimuth u16 (deg)
            for (int k = 0; k < b[3] && 4 + 6 * k + 6 <= static_cast<int>(b.size()); ++k)
            {
                const uint8_t* c = &b[4 + 6 * k];
                const int sys = c[0] == 0 ? GNSS_SYS_GPS : c[0] == 3 ? GNSS_SYS_GAL : c[0] == 5 ? GNSS_SYS_BDS : -1;
                const auto it = latest.find({sys, c[1]});
                const double el_rx = (c[2] << 8) | c[3], az_rx = (c[4] << 8) | c[5];
                if (sys < 0 || it == latest.end() || el_rx < 10.0) continue;
                const double t = rx_tow + (sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0);
                double pos[3];
                gps_sat_pos(&it->second, t, pos, nullptr, nullptr);
                double el, az;
                elAz(rx, pos, el, az);
                double daz = std::fabs(az - az_rx);
                if (daz > 180.0) daz = 360.0 - daz;
                el_err_max[sys] = std::max(el_err_max[sys], std::fabs(el - el_rx));
                az_err_max[sys] = std::max(az_err_max[sys], daz * std::cos(el * M_PI / 180.0));
                el_checks[sys]++;
            }
        }
        if (!isNavMessage(b.data(), b.size())) continue;
        gps_eph_t eph{};
        int8_t iono[8];
        const NavStatus st = nav.add(b.data(), b.size(), week, eph, iono);
        if (st == NavStatus::IONO) ++iono_n;
        if (st != NavStatus::EPHEMERIS) continue;
        if (first_eph_line[eph.sys] < 0) first_eph_line[eph.sys] = ln;
        latest[{eph.sys, eph.prn}] = eph;
        if (issues.insert({eph.sys, eph.prn, eph.iode, eph.toe}).second) distinct.push_back(eph);
    }
    if (gz) pclose(f); else std::fclose(f);

    const NavCounters& c = nav.counters();
    EXPECT_EQ(c.frames[GNSS_SYS_GPS], static_cast<uint32_t>(fx::kCaptureGpsFrames));
    EXPECT_EQ(c.frames[GNSS_SYS_GAL], static_cast<uint32_t>(fx::kCaptureGalFrames));
    EXPECT_EQ(c.frames[GNSS_SYS_BDS], static_cast<uint32_t>(fx::kCaptureBdsFrames));
    EXPECT_EQ(c.ephemerides[GNSS_SYS_GPS], static_cast<uint32_t>(fx::kCaptureGpsDecodes));
    EXPECT_EQ(c.ephemerides[GNSS_SYS_GAL], static_cast<uint32_t>(fx::kCaptureGalDecodes));
    EXPECT_EQ(c.ephemerides[GNSS_SYS_BDS], static_cast<uint32_t>(fx::kCaptureBdsDecodes));
    EXPECT_EQ(c.issues[GNSS_SYS_GPS], static_cast<uint32_t>(fx::kCaptureGpsIssues));
    EXPECT_EQ(c.issues[GNSS_SYS_GAL], static_cast<uint32_t>(fx::kCaptureGalIssues));
    EXPECT_EQ(c.issues[GNSS_SYS_BDS], static_cast<uint32_t>(fx::kCaptureBdsIssues));
    EXPECT_EQ(c.status[static_cast<size_t>(NavStatus::CRC_FAIL)], static_cast<uint32_t>(fx::kCaptureGalCrcFails));
    std::set<std::tuple<int, int, int, double>> want;
    for (const auto& k : fx::kCaptureIssues) want.insert({k.sys, k.prn, k.iode, k.toe});
    EXPECT_EQ(issues, want);
    for (const auto& e : distinct) EXPECT_EQ(e.week, fx::kWeek) << e.sys << " " << e.prn;

    // The receiver's own elevation and azimuth (whole degrees) of every
    // satellite above 10 degrees, against where these ephemerides put it.
    for (int s = 0; s < 3; ++s)
    {
        EXPECT_GT(el_checks[s], 100) << s;
        EXPECT_LT(el_err_max[s], 1.5) << s;   // whole degrees, and the 0xE8 is 1 Hz
        EXPECT_LT(az_err_max[s], 1.5) << s;
    }

    // Every pseudorange above 15 degrees, less what its epoch's satellites
    // share. What is left is the atmosphere's difference between satellites,
    // multipath and each system's receiver delay (BeiDou sits ~8 m above GPS
    // and Galileo on 09-26), all tens of metres; an orbit or clock field
    // decoded wrong leaves hundreds of metres to kilometres on that
    // satellite. Each satellite's median, and each system's 1-99 % band;
    // single measurements reach 118 m (G24, multipath), so not the extremes.
    for (int s = 0; s < 3; ++s)
    {
        ASSERT_GT(res[s].size(), 100u) << s;
        std::vector<double> v = res[s], w = res_raw[s];
        std::sort(v.begin(), v.end());
        std::sort(w.begin(), w.end());
        const double p01 = v[v.size() / 100], p99 = v[v.size() * 99 / 100];
        std::printf("[ info ] %c pseudorange residuals (el >= 15 deg): %zu, as they are median %.1f m; less the "
                    "epoch's median: median %.1f, 1-99 %% %.1f..%.1f, min %.1f, max %.1f m\n",
                    "GEC"[s], v.size(), w[w.size() / 2], v[v.size() / 2], p01, p99, v.front(), v.back());
        EXPECT_GT(p01, -50.0) << s;
        EXPECT_LT(p99, 50.0) << s;
    }
    int sats_checked = 0;
    double worst_median = 0.0;
    for (auto& kv : res_sat)
    {
        std::vector<double> v = kv.second;
        std::sort(v.begin(), v.end());
        const double med = v[v.size() / 2];
        if (verbose)
            std::printf("[ info ]   %c%02d %4zu: median %6.1f min %6.1f max %6.1f m\n", "GEC"[kv.first.first],
                        kv.first.second, v.size(), med, v.front(), v.back());
        if (v.size() < 50) continue;
        ++sats_checked;
        worst_median = std::max(worst_median, std::fabs(med));
        EXPECT_LT(std::fabs(med), 40.0) << "GEC"[kv.first.first] << kv.first.second;
    }
    EXPECT_EQ(sats_checked, 20);   // every satellite with an ephemeris: 6 GPS, 6 Galileo, 8 BeiDou
    std::printf("[ info ] %d satellites' median pseudorange residual within %.1f m\n", sats_checked, worst_median);

    std::printf("[ info ] statuses:");
    for (int s = 0; s < static_cast<int>(NavStatus::COUNT); ++s)
        if (c.status[s]) std::printf(" %s %u", navStatusName(static_cast<NavStatus>(s)), c.status[s]);
    std::printf("\n[ info ] ephemerides G/E/C %u/%u/%u, issues %u/%u/%u, iono %d (prototype %d/%d/%d, %d/%d/%d)\n",
                c.ephemerides[0], c.ephemerides[1], c.ephemerides[2], c.issues[0], c.issues[1], c.issues[2], iono_n,
                fx::kCaptureGpsDecodes, fx::kCaptureGalDecodes, fx::kCaptureBdsDecodes, fx::kCaptureGpsIssues,
                fx::kCaptureGalIssues, fx::kCaptureBdsIssues);
    std::printf("[ info ] first ephemeris at capture line G %d, E %d, C %d\n", first_eph_line[0], first_eph_line[1],
                first_eph_line[2]);
    std::printf("[ info ] vs 0xE8 (el >= 10 deg): max |d el| G %.2f E %.2f C %.2f deg, max |d az| cos el G %.2f E %.2f "
                "C %.2f deg, checks %d/%d/%d\n",
                el_err_max[0], el_err_max[1], el_err_max[2], az_err_max[0], az_err_max[1], az_err_max[2], el_checks[0],
                el_checks[1], el_checks[2]);

    if (const char* dump = std::getenv("TR_SKYTRAQ_NAV_DUMP"))
    {
        FILE* o = std::fopen(dump, "w");
        ASSERT_NE(o, nullptr) << dump;
        for (const auto& e : distinct)
            std::fprintf(o,
                         "%d %d %d %d %d %d %d %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g "
                         "%.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n",
                         e.sys, e.prn, e.week, e.iodc, e.iode, e.health, e.ura, e.toe, e.toc, e.sqrt_a, e.e, e.i0,
                         e.omega0, e.omega, e.m0, e.delta_n, e.idot, e.omega_dot, e.cuc, e.cus, e.crc, e.crs, e.cic,
                         e.cis, e.af0, e.af1, e.af2, e.tgd);
        std::fclose(o);
    }
}
