// Host tests for the replay harness's RINEX 3 navigation reader
// (tools/raw_replay/rinex_nav.cpp): the day's broadcast ephemerides filled
// into gps_eph_t by the conventions the mosaic-G5's SBF decoder uses
// (TR_GNSSReceiverMosaic_Serial/SbfNav.cpp), so a replay places a satellite
// the same from either. Every ephemeris the Beetle's LC86G sent in
// fixtures/raw_meas_golden.h (RTCM 1019/1046/1042, which test_sbf_nav.cpp
// shows decode bit for bit as their SBF blocks) is written here as the RINEX
// record a broadcast file carries for it and must come back to 1 cm of
// satellite position and 1 ns of clock, with the same issue, week, health
// and times. Then what is left out (Galileo F/NAV, BeiDou GEO, other
// systems), the Galileo health marks, the dates, the availability times.
// With TR_SBF_PRIVATE_CAPTURE (the 10/16 V9 captures) and TR_RINEX_NAV (the
// day-289 broadcast file) set, every navigation block of the captures is
// compared with the RINEX record of the same issue: statistics only.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "RtcmEphemeris.h"
#include "SbfNav.h"
#include "fixtures/raw_meas_golden.h"
#include "gnss/eph.h"
#include "gnss/types.h"
#include "rinex_nav.h"
#include "sbf_private_capture.h"

namespace {

constexpr int kNearWeek = 2440;

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

// RINEX's D19.12, as a broadcast file writes it
std::string d19(double x)
{
    char b[40];
    std::snprintf(b, sizeof(b), "%19.12e", x);
    return b;
}

std::string orbitLine(double a, double b, double c, double d)
{
    return "    " + d19(a) + d19(b) + d19(c) + d19(d) + "\n";
}

// A week and second of week (counted as GPS weeks: BDT's from 1356) as the
// civil date and time RINEX writes an epoch in.
std::string epochText(int week, double sow)
{
    long days = static_cast<long>(week) * 7 + static_cast<long>(std::floor(sow / 86400.0));
    const double sod = sow - std::floor(sow / 86400.0) * 86400.0;
    // civil from days since 1980-01-06 (H. Hinnant's civil_from_days)
    long z = days + 3657 + 719468;
    const long era = (z >= 0 ? z : z - 146096) / 146097;
    const long doe = z - era * 146097;
    const long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    long y = yoe + era * 400;
    const long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const long mp = (5 * doy + 2) / 153;
    const long d = doy - (153 * mp + 2) / 5 + 1;
    const long m = mp + (mp < 10 ? 3 : -9);
    y += (m <= 2);
    const int s = static_cast<int>(std::lround(sod));
    char b[64];
    std::snprintf(b, sizeof(b), "%04ld %02ld %02ld %02d %02d %02d", y, m, d, s / 3600, (s / 60) % 60, s % 60);
    return b;
}

double uraMetres(int idx)
{
    static const double kBound[15] = {2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0,
                                      96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0};
    return idx >= 0 && idx < 15 ? kBound[idx] : 6200.0;
}

double sisaMetres(int idx)
{
    if (idx < 50) return idx * 0.01;
    if (idx < 75) return 0.5 + (idx - 50) * 0.02;
    if (idx < 100) return 1.0 + (idx - 75) * 0.04;
    if (idx < 126) return 2.0 + (idx - 100) * 0.16;
    return -1.0;
}

// The record a broadcast file carries for e (RINEX 3.05, section A8-A10).
std::string rinexRecord(const gps_eph_t& e, long gal_source = 517, double gal_health_word = -1.0)
{
    const char sys = e.sys == GNSS_SYS_GPS ? 'G' : (e.sys == GNSS_SYS_GAL ? 'E' : 'C');
    char head[16];
    std::snprintf(head, sizeof(head), "%c%02d ", sys, e.prn);
    std::string r = head + epochText(e.week, e.toc) + d19(e.af0) + d19(e.af1) + d19(e.af2) + "\n";
    r += orbitLine(e.iode, e.crs, e.delta_n, e.m0);
    r += orbitLine(e.cuc, e.e, e.cus, e.sqrt_a);
    r += orbitLine(e.toe, e.cic, e.omega0, e.cis);
    r += orbitLine(e.i0, e.crc, e.omega, e.omega_dot);
    if (e.sys == GNSS_SYS_GPS)
    {
        r += orbitLine(e.idot, 1.0, e.week, 0.0);
        r += orbitLine(uraMetres(e.ura), e.health, e.tgd, e.iodc);
        r += orbitLine(e.toe - 7200.0, 4.0, 0.0, 0.0);
    }
    else if (e.sys == GNSS_SYS_GAL)
    {
        r += orbitLine(e.idot, static_cast<double>(gal_source), e.week, 0.0);
        r += orbitLine(sisaMetres(e.ura), gal_health_word >= 0.0 ? gal_health_word : (e.health & 0x07), -1.4e-9,
                       e.tgd);
        r += orbitLine(e.toe + 665.0, 0.0, 0.0, 0.0);
    }
    else
    {
        r += orbitLine(e.idot, 0.0, e.week - 1356, 0.0);
        r += orbitLine(uraMetres(e.ura), e.health, e.tgd, -2.0e-9);
        r += orbitLine(e.toe + 18.0, e.iodc, 0.0, 0.0);
    }
    return r;
}

const char* kHeader =
    "     3.05           N: GNSS NAV DATA    M: MIXED            RINEX VERSION / TYPE\n"
    "tests               tests               20261016 235341 UTC PGM / RUN BY / DATE\n"
    "                                                            END OF HEADER\n";

// GLONASS and SBAS records: three orbit lines, a different layout
const char* kOtherSystems =
    "R01 2026 10 16 00 15 00 1.000000000000e-05 0.000000000000e+00 4.500000000000e+04\n"
    "     1.000000000000e+04 1.000000000000e+00 0.000000000000e+00 0.000000000000e+00\n"
    "     1.000000000000e+04 1.000000000000e+00 0.000000000000e+00 1.000000000000e+00\n"
    "     1.000000000000e+04 1.000000000000e+00 0.000000000000e+00 0.000000000000e+00\n"
    "S27 2026 10 16 00 01 04 0.000000000000e+00 0.000000000000e+00 4.326400000000e+05\n"
    "     4.000000000000e+07 0.000000000000e+00 0.000000000000e+00 1.270000000000e+02\n"
    "    -1.000000000000e+04 0.000000000000e+00 0.000000000000e+00 3.200000000000e+01\n"
    "     0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 9.000000000000e+01\n";

void expectPlacedAlike(const gps_eph_t& a, const gps_eph_t& b, double pos_m, double clk_s)
{
    for (double dt = -7200.0; dt <= 7200.0; dt += 900.0)
    {
        double pa[3], va[3], ca, pb[3], vb[3], cb;
        gps_sat_pos(&a, a.toe + dt, pa, va, &ca);
        gps_sat_pos(&b, a.toe + dt, pb, vb, &cb);
        const double d = std::sqrt((pa[0] - pb[0]) * (pa[0] - pb[0]) + (pa[1] - pb[1]) * (pa[1] - pb[1]) +
                                   (pa[2] - pb[2]) * (pa[2] - pb[2]));
        EXPECT_LE(d, pos_m) << "at toe" << dt;
        EXPECT_LE(std::fabs(ca - cb), clk_s) << "at toe" << dt;
    }
}

}  // namespace

TEST(RinexNav, EveryGoldenEphemerisComesBackAsItsBroadcast)
{
    const std::vector<gps_eph_t> goldens = rtcmGoldens();
    ASSERT_EQ(goldens.size(), 15u);
    std::string text = kHeader;
    for (const gps_eph_t& g : goldens)
        text += rinexRecord(g);
    rinex::NavFile nf;
    std::string err;
    ASSERT_TRUE(rinex::parseNav(text, nf, err)) << err;
    EXPECT_EQ(nf.version_major, 3);
    ASSERT_EQ(nf.recs.size(), goldens.size());
    EXPECT_EQ(nf.bad, 0);
    int per_sys[3] = {};
    for (size_t i = 0; i < goldens.size(); ++i)
    {
        const gps_eph_t& g = goldens[i];
        const gps_eph_t& r = nf.recs[i].eph;
        SCOPED_TRACE(testing::Message() << "sys " << g.sys << " prn " << g.prn);
        per_sys[g.sys]++;
        EXPECT_EQ(r.valid, 1);
        EXPECT_EQ(r.sys, g.sys);
        EXPECT_EQ(r.prn, g.prn);
        EXPECT_EQ(r.week, g.week);
        EXPECT_EQ(r.iode, g.iode);
        EXPECT_EQ(r.iodc, g.iodc);
        EXPECT_EQ(r.health, g.health);
        EXPECT_EQ(r.ura, g.ura);
        EXPECT_EQ(r.toe, g.toe);
        EXPECT_EQ(r.toc, g.toc);
        // 13 significant digits in the file
        auto near = [](double x, double y) { return std::fabs(x - y) <= 1e-12 * std::max(std::fabs(y), 1e-30) + 1e-25; };
        EXPECT_TRUE(near(r.sqrt_a, g.sqrt_a));
        EXPECT_TRUE(near(r.e, g.e));
        EXPECT_TRUE(near(r.m0, g.m0));
        EXPECT_TRUE(near(r.omega0, g.omega0));
        EXPECT_TRUE(near(r.omega, g.omega));
        EXPECT_TRUE(near(r.i0, g.i0));
        EXPECT_TRUE(near(r.af0, g.af0));
        EXPECT_TRUE(near(r.tgd, g.tgd));
        // and the satellite is where the receiver's own decode puts it
        expectPlacedAlike(g, r, 0.01, 1e-9);
    }
    EXPECT_GT(per_sys[GNSS_SYS_GPS], 0);
    EXPECT_GT(per_sys[GNSS_SYS_GAL], 0);
    EXPECT_GT(per_sys[GNSS_SYS_BDS], 0);
}

TEST(RinexNav, WhatTheSbfDecoderRefusesIsLeftOut)
{
    const std::vector<gps_eph_t> goldens = rtcmGoldens();
    gps_eph_t gal{}, bds{};
    for (const gps_eph_t& g : goldens)
    {
        if (g.sys == GNSS_SYS_GAL) gal = g;
        if (g.sys == GNSS_SYS_BDS) bds = g;
    }
    ASSERT_EQ(gal.valid, 1);
    ASSERT_EQ(bds.valid, 1);
    gps_eph_t geo = bds;
    geo.prn = 3;   // C03: GEO
    std::string text = kHeader;
    text += kOtherSystems;
    text += rinexRecord(gal, 258);   // F/NAV, with the (E5a,E1) clock
    text += rinexRecord(gal, 513);   // I/NAV E1-B: kept
    text += rinexRecord(geo);
    text += rinexRecord(gal, 517 & ~0x200);   // I/NAV without the (E5b,E1) clock
    text += rinexRecord(bds);
    rinex::NavFile nf;
    std::string err;
    ASSERT_TRUE(rinex::parseNav(text, nf, err)) << err;
    EXPECT_EQ(nf.skipped_system, 2);
    EXPECT_EQ(nf.skipped_fnav, 2);
    EXPECT_EQ(nf.skipped_geo, 1);
    EXPECT_EQ(nf.bad, 0);
    ASSERT_EQ(nf.recs.size(), 2u);
    EXPECT_EQ(nf.recs[0].eph.sys, GNSS_SYS_GAL);
    EXPECT_EQ(nf.recs[1].eph.sys, GNSS_SYS_BDS);
    EXPECT_EQ(nf.recs[1].eph.prn, bds.prn);
}

TEST(RinexNav, GalileoHealthAsSbfNavMarksIt)
{
    gps_eph_t gal{};
    for (const gps_eph_t& g : rtcmGoldens())
        if (g.sys == GNSS_SYS_GAL)
        {
            gal = g;
            break;
        }
    ASSERT_EQ(gal.valid, 1);
    std::string text = kHeader;
    // E1-B HS 1 (bits 1-2) and E5b HS (bits 7-8), the E1-B DVS clear: health 2, as SbfNav's (HS << 1) | DVS
    // (the file's health word carries the E5a and E5b bits too: only bits 0-2 are the E1-B's)
    text += rinexRecord(gal, 517, 130.0);
    // I/NAV from E5b alone: no E1-B status, unhealthy as SbfNav's kGalHealthNoStatus
    text += rinexRecord(gal, 516);
    // SISA "no accuracy prediction" (-1 in the file): NAPA, 255, and kGalHealthNoSisa
    gps_eph_t napa = gal;
    napa.ura = 255;
    text += rinexRecord(napa);
    rinex::NavFile nf;
    std::string err;
    ASSERT_TRUE(rinex::parseNav(text, nf, err)) << err;
    ASSERT_EQ(nf.recs.size(), 3u);
    EXPECT_EQ(nf.recs[0].eph.health, 2);
    EXPECT_EQ(nf.recs[1].eph.health & mosaic::kGalHealthNoStatus, mosaic::kGalHealthNoStatus);
    EXPECT_EQ(nf.recs[2].eph.ura, 255);
    EXPECT_TRUE(nf.recs[2].eph.health & mosaic::kGalHealthNoSisa);
}

TEST(RinexNav, DatesWeeksAndAvailability)
{
    int w = 0;
    double s = 0.0;
    rinex::calendarToWeek(1980, 1, 6, 0, 0, 0.0, w, s);
    EXPECT_EQ(w, 0);
    EXPECT_EQ(s, 0.0);
    rinex::calendarToWeek(2026, 10, 16, 12, 30, 15.0, w, s);   // a Friday
    EXPECT_EQ(w, 2440);
    EXPECT_EQ(s, 5 * 86400.0 + 12 * 3600.0 + 30 * 60.0 + 15.0);
    rinex::calendarToWeek(2026, 10, 17, 23, 59, 59.0, w, s);   // the week's last second
    EXPECT_EQ(w, 2440);
    EXPECT_EQ(s, 604799.0);
    rinex::calendarToWeek(2026, 10, 18, 0, 0, 0.0, w, s);      // and the next week's first
    EXPECT_EQ(w, 2441);
    EXPECT_EQ(s, 0.0);
    rinex::calendarToWeek(2006, 1, 1, 0, 0, 0.0, w, s);        // BDT week 0
    EXPECT_EQ(w, 1356);
    EXPECT_EQ(s, 0.0);
    EXPECT_EQ(epochText(2440, 5 * 86400.0 + 3661.0), "2026 10 16 01 01 01");

    EXPECT_EQ(rinex::uraIndex(2.0), 0);
    EXPECT_EQ(rinex::uraIndex(2.4), 0);
    EXPECT_EQ(rinex::uraIndex(2.8), 1);
    EXPECT_EQ(rinex::uraIndex(6144.0), 14);
    EXPECT_EQ(rinex::uraIndex(1.0e4), 15);
    EXPECT_EQ(rinex::sisaIndex(3.12), 107);
    EXPECT_EQ(rinex::sisaIndex(-1.0), 255);
    EXPECT_EQ(rinex::sisaIndex(0.3), 30);

    // when a receiver first has each: GPS 2 h before its toe, Galileo 665 s after, BeiDou 18 s after (in BDT,
    // 14 s behind GPS)
    std::string text = kHeader;
    const std::vector<gps_eph_t> goldens = rtcmGoldens();
    for (const gps_eph_t& g : goldens)
        text += rinexRecord(g);
    rinex::NavFile nf;
    std::string err;
    ASSERT_TRUE(rinex::parseNav(text, nf, err)) << err;
    for (const rinex::NavRecord& r : nf.recs)
    {
        const double toe = r.eph.week * 604800.0 + r.eph.toe;
        if (r.eph.sys == GNSS_SYS_GPS)
        {
            EXPECT_EQ(r.toe_gps_s, toe);
            EXPECT_EQ(r.avail_gps_s, toe - 7200.0);
        }
        else if (r.eph.sys == GNSS_SYS_GAL)
        {
            EXPECT_EQ(r.avail_gps_s, toe + 665.0);
        }
        else
        {
            EXPECT_EQ(r.toe_gps_s, toe + 14.0);
            EXPECT_EQ(r.avail_gps_s, toe + 14.0 + 18.0);
        }
    }
}

TEST(RinexNav, NotRinex3NavigationIsRefused)
{
    rinex::NavFile nf;
    std::string err;
    EXPECT_FALSE(rinex::parseNav("     2.11           N: GPS NAV DATA                         RINEX VERSION / TYPE\n"
                                 "                                                            END OF HEADER\n",
                                 nf, err));
    EXPECT_FALSE(err.empty());
    EXPECT_FALSE(rinex::parseNav("     3.05           O: OBSERVATION DATA M: MIXED            RINEX VERSION / TYPE\n",
                                 nf, err));
    EXPECT_TRUE(rinex::parseNav(kHeader, nf, err)) << err;   // a header alone: no records
    EXPECT_EQ(nf.recs.size(), 0u);
    EXPECT_FALSE(rinex::readNavFile("/nonexistent/brdc.rnx", nf, err));
}

TEST(RinexNav, FortranExponentsAndBlankFields)
{
    gps_eph_t gps{};
    for (const gps_eph_t& g : rtcmGoldens())
        if (g.sys == GNSS_SYS_GPS)
        {
            gps = g;
            break;
        }
    ASSERT_EQ(gps.valid, 1);
    std::string rec = rinexRecord(gps);
    for (char& c : rec)
        if (c == 'e')
            c = 'D';
    // a blank spare field (BeiDou's and some writers' GPS) reads as 0
    const size_t last = rec.rfind("\n", rec.size() - 2);
    rec = rec.substr(0, last + 1) + "    " + d19(gps.toe - 7200.0) + std::string(19, ' ') + "\n";
    rinex::NavFile nf;
    std::string err;
    ASSERT_TRUE(rinex::parseNav(kHeader + rec, nf, err)) << err;
    ASSERT_EQ(nf.recs.size(), 1u);
    expectPlacedAlike(gps, nf.recs[0].eph, 0.01, 1e-9);
}

// The 10/16 captures' own navigation blocks against the day's broadcast file:
// the same issue of each satellite placed alike (G2: 1 cm, 1 ns).
TEST(RinexNav, PrivateCapturesAgreeWithTheBroadcastFile)
{
    const std::vector<std::string> paths = sbftest::privateCapturePaths();
    const char* rnx = std::getenv("TR_RINEX_NAV");
    if (paths.empty() || rnx == nullptr)
        GTEST_SKIP() << "TR_SBF_PRIVATE_CAPTURE and TR_RINEX_NAV not set";
    rinex::NavFile nf;
    std::string err;
    ASSERT_TRUE(rinex::readNavFile(rnx, nf, err)) << err;
    std::multimap<std::tuple<int, int, int, int, double>, gps_eph_t> by_issue;
    for (const rinex::NavRecord& r : nf.recs)
        by_issue.emplace(std::make_tuple(r.eph.sys, r.eph.prn, r.eph.iode, r.eph.week, r.eph.toe), r.eph);
    int matched = 0;
    for (const std::string& p : paths)
    {
        int decoded[3] = {}, found[3] = {}, health_differs[3] = {};
        double worst_m[3] = {}, worst_s[3] = {};
        sbftest::CaptureScan scan;
        ASSERT_TRUE(sbftest::scanCapture(
            p,
            [&](const uint8_t* b, size_t len) {
                gps_eph_t e;
                if (mosaic::decodeSbfEphemeris(b, len, mosaic::kWncDnu, e) != mosaic::SbfEphStatus::OK)
                    return;
                decoded[e.sys]++;
                auto it = by_issue.find(std::make_tuple(e.sys, e.prn, e.iode, e.week, e.toe));
                if (it == by_issue.end())
                    return;
                found[e.sys]++;
                for (double dt = -7200.0; dt <= 7200.0; dt += 600.0)
                {
                    double pa[3], va[3], ca, pb[3], vb[3], cb;
                    gps_sat_pos(&e, e.toe + dt, pa, va, &ca);
                    gps_sat_pos(&it->second, e.toe + dt, pb, vb, &cb);
                    const double d = std::sqrt((pa[0] - pb[0]) * (pa[0] - pb[0]) + (pa[1] - pb[1]) * (pa[1] - pb[1]) +
                                               (pa[2] - pb[2]) * (pa[2] - pb[2]));
                    worst_m[e.sys] = std::max(worst_m[e.sys], d);
                    worst_s[e.sys] = std::max(worst_s[e.sys], std::fabs(ca - cb));
                }
                if (e.health != it->second.health)
                    health_differs[e.sys]++;
                EXPECT_DOUBLE_EQ(e.toc, it->second.toc);
            },
            scan));
        static const char* kName[3] = {"GPS", "Galileo", "BeiDou"};
        for (int s = 0; s < 3; ++s)
        {
            std::printf("[ capture ] %s %s: %d ephemerides decoded, %d of the same issue in the broadcast file, "
                        "worst %.4f m and %.3f ns over toe +-2 h, health read differently %d\n",
                        sbftest::captureName(p).c_str(), kName[s], decoded[s], found[s], worst_m[s],
                        worst_s[s] * 1e9, health_differs[s]);
            EXPECT_LE(worst_m[s], 0.01);
            EXPECT_LE(worst_s[s], 1e-9);
            matched += found[s];
        }
    }
    EXPECT_GT(matched, 0);
}
