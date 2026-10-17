// Host tests for the raw-GNSS measurement stage's mosaic-G5 additions
// (RawGnssMeasStage.h; registered in v9raw/stage_v9.cmake): an ephemeris a
// driver decoded itself (addEphemeris), whether a satellite has one
// (hasEphemeris), and the satellite cap in offer() -- with the cap off,
// today's build exactly. And the ephemeris store's default size, the
// Beetle's. The store at the mosaic build's 128 is
// test_eph_store_pad_wait.cpp's, a target of its own.
//
// The cap's skies are synthetic: Keplerian MEO shells over a receiver on the
// equator at the prime meridian, no real place. The LC86G's golden epochs
// (fixtures/raw_meas_golden.h) check that a cap above the sky changes nothing.
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <set>
#include <utility>
#include <vector>

#include "Lc86Parser.h"
#include "Lc86RawEpoch.h"
#include "RawGnssMeasStage.h"
#include "Rtcm3Msm.h"
#include "RtcmEphemeris.h"
#include "fixtures/raw_meas_golden.h"
#include "gnss/atmo.h"
#include "gnss/types.h"

using gnss_raw::BuiltEpoch;
using gnss_raw::EphemerisStore;
using gnss_raw::RawGnssMeasStage;
using gnss_raw::RawObs;
using gnss_raw::TimedRawEpoch;

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int64_t kT0 = 5000000000LL;    // MCU time of the first epoch, us
constexpr uint16_t kGoldenWeek = 2433;   // the golden capture's GPS week
constexpr double kTow0 = 302400.0;       // the synthetic skies' first epoch: midweek

std::vector<uint8_t> frame(const GoldenPayload& p)
{
    std::vector<uint8_t> f(static_cast<size_t>(p.len) + rtcm3::kFrameOverhead);
    f[0] = rtcm3::kPreamble;
    f[1] = static_cast<uint8_t>((p.len >> 8) & 0x03);
    f[2] = static_cast<uint8_t>(p.len & 0xFF);
    for (int i = 0; i < p.len; ++i)
        f[3 + i] = p.bytes[i];
    const uint32_t crc = rtcm3::crc24q(f.data(), static_cast<size_t>(p.len) + 3);
    f[p.len + 3] = static_cast<uint8_t>(crc >> 16);
    f[p.len + 4] = static_cast<uint8_t>(crc >> 8);
    f[p.len + 5] = static_cast<uint8_t>(crc);
    return f;
}

// The golden capture's two MSM7 epochs through the flight parser, a second
// apart on the MCU clock.
std::vector<TimedRawEpoch> goldenEpochs()
{
    lc86::Lc86Parser parser;
    std::vector<TimedRawEpoch> epochs;
    for (int i = 0; i < kGoldenRmMsmN; ++i)
        for (uint8_t b : frame(kGoldenRmMsm[i]))
            if (parser.feed(b) == lc86::Event::MSM_EPOCH)
            {
                TimedRawEpoch e;
                EXPECT_TRUE(lc86::rawEpochFromMsm(parser.msmEpoch(), e.epoch));
                e.mcu_us = kT0 + static_cast<int64_t>(epochs.size()) * 1000000;
                e.gps_week = kGoldenWeek;
                epochs.push_back(e);
            }
    return epochs;
}

gnss_raw::ShadowNav navAt(const double lla[3])
{
    gnss_raw::ShadowNav nav{};
    for (int k = 0; k < 3; ++k)
        nav.lla[k] = lla[k];
    nav.running = 1;
    return nav;
}

gnss_raw::ShadowNav goldenNav()
{
    double lla[3];
    ecef_to_geo(kGoldenRmRx, &lla[0], &lla[1], &lla[2]);
    return navAt(lla);
}

// A satellite of a MEO shell (GPS's 6 planes, Galileo's 3, BeiDou's 3), its
// ephemeris's toe the GPS time of week tow (BeiDou's in its own time).
gps_eph_t orbit(int sys, int prn, double tow)
{
    gps_eph_t e;
    std::memset(&e, 0, sizeof(e));
    e.valid = 1;
    e.sys = sys;
    e.prn = prn;
    e.week = 2440;
    e.iode = 7;
    e.toe = e.toc = tow + (sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0);
    e.sqrt_a = sys == GNSS_SYS_GPS ? 5153.7 : (sys == GNSS_SYS_GAL ? 5440.6 : 5282.6);
    e.e = 0.005;
    e.i0 = (sys == GNSS_SYS_GAL ? 56.0 : 55.0) * kPi / 180.0;
    const int planes = sys == GNSS_SYS_GPS ? 6 : 3;
    e.omega0 = (prn % planes) * 2.0 * kPi / planes;
    e.m0 = (prn / planes) * 2.0 * kPi / (sys == GNSS_SYS_GPS ? 5.4 : 9.0) + (prn % planes) * 0.7;
    return e;
}

const double kRxLla[3] = {0.0, 0.0, 0.0};   // the synthetic receiver

// A synthetic sky at tow: every satellite of the three shells above 15 deg
// from the receiver, at most n of them (GPS, then Galileo, then BeiDou), each
// with its ephemeris (toe at kTow0), a pseudorange (its range: the receiver
// clock at 0), a range rate and C/N0 40 dB-Hz.
struct Sky
{
    std::vector<gps_eph_t> eph;
    TimedRawEpoch e;
};

Sky sky(int n_max, double tow = kTow0)
{
    double rr[3];
    geo_to_ecef(kRxLla[0], kRxLla[1], kRxLla[2], rr);
    Sky s;
    s.e.epoch.tow_s = tow;
    s.e.mcu_us = kT0;
    s.e.gps_week = 2440;
    const int sys_order[3] = {GNSS_SYS_GPS, GNSS_SYS_GAL, GNSS_SYS_BDS};
    for (int sys : sys_order)
    {
        const int prn0 = sys == GNSS_SYS_BDS ? 19 : 1;   // BeiDou's MEO (its GEOs are not placed)
        const int prn1 = sys == GNSS_SYS_GPS ? 32 : (sys == GNSS_SYS_GAL ? 36 : 46);
        for (int prn = prn0; prn <= prn1 && s.e.epoch.n < n_max; ++prn)
        {
            const gps_eph_t e = orbit(sys, prn, kTow0);
            double rs[3], clk;
            gps_sat_pos(&e, tow + (sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0), rs, nullptr, &clk);
            const double d[3] = {rs[0] - rr[0], rs[1] - rr[1], rs[2] - rr[2]};
            double az, el;
            gnss_az_el(kRxLla[0], kRxLla[1], d, &az, &el);
            if (el < 15.0 * kPi / 180.0)
                continue;
            RawObs& o = s.e.epoch.obs[s.e.epoch.n++];
            o = RawObs{};
            o.sys = static_cast<uint8_t>(sys);
            o.prn = static_cast<uint8_t>(prn);
            o.has_pr = true;
            o.has_rate = true;
            o.cn0_dbhz = 40.0f;
            o.pr_m = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            o.rate_mps = 100.0;
            o.lock_ms = 60000;
            s.eph.push_back(e);
        }
    }
    return s;
}

// A stage with the mosaic's C/N0 floors (25 dB-Hz) and the cap, holding sky's ephemerides.
RawGnssMeasStage::Config capCfg(int cap)
{
    RawGnssMeasStage::Config c;
    for (int s = 0; s < 3; ++s)
        c.meas.sig[s].cn0_min_pr = c.meas.sig[s].cn0_min_rr = 25.0f;
    c.sat_cap = cap;
    return c;
}

void addAll(RawGnssMeasStage& st, const Sky& s)
{
    for (const gps_eph_t& e : s.eph)
        ASSERT_TRUE(st.addEphemeris(e));
}

RawObs* find(TimedRawEpoch& e, int sys, int prn)
{
    for (int i = 0; i < e.epoch.n; ++i)
        if (e.epoch.obs[i].sys == sys && e.epoch.obs[i].prn == prn)
            return &e.epoch.obs[i];
    return nullptr;
}

std::set<std::pair<int, int>> sats(const BuiltEpoch& b)
{
    std::set<std::pair<int, int>> out;
    for (int i = 0; i < b.n; ++i)
        out.insert({b.meas[i].sys, b.meas[i].prn});
    return out;
}

int count(const BuiltEpoch& b, int sys)
{
    int n = 0;
    for (int i = 0; i < b.n; ++i)
        n += b.meas[i].sys == sys ? 1 : 0;
    return n;
}

}  // namespace

// ------------------------------------------------- addEphemeris, hasEphemeris

TEST(RawGnssMeasStageV9, AnEphemerisDecodedByItsDriverIsFiledAsAnRtcmOneIs)
{
    // The mosaic's driver decodes its SBF navigation blocks itself; the
    // store and the counts must come out as the LC86G's RTCM ones do.
    RawGnssMeasStage rtcm, own;
    auto expectSame = [&]() {
        EXPECT_EQ(own.stats().eph_added, rtcm.stats().eph_added);
        EXPECT_EQ(own.stats().eph_bad, rtcm.stats().eph_bad);
        EXPECT_EQ(own.stats().eph_sats, rtcm.stats().eph_sats);
        EXPECT_EQ(own.ephemerides().size(), rtcm.ephemerides().size());
    };
    std::vector<gps_eph_t> decoded;
    for (int pass = 0; pass < 2; ++pass)   // the second: every one a repeat
        for (int i = 0; i < kGoldenRmEphN; ++i)
        {
            const GoldenPayload& p = kGoldenRmEph[i];
            gps_eph_t e;
            // as onEphemeris decodes it: no epoch yet, so the default week
            ASSERT_TRUE(gnss_raw::decodeEphemeris(p.bytes, static_cast<size_t>(p.len),
                                                  RawGnssMeasStage::Config{}.default_week, e));
            EXPECT_TRUE(rtcm.onEphemeris(p.bytes, static_cast<size_t>(p.len)));
            EXPECT_TRUE(own.addEphemeris(e));
            expectSame();
            if (pass == 0)
                decoded.push_back(e);
        }
    EXPECT_EQ(own.stats().eph_added, static_cast<uint32_t>(kGoldenRmEphN));
    EXPECT_EQ(own.ephemerisSatellites(), kGoldenRmEphN);
    for (const gps_eph_t& e : decoded)
    {
        const gps_eph_t* a = rtcm.ephemerides().select(e.sys, e.prn, e.toe);
        const gps_eph_t* b = own.ephemerides().select(e.sys, e.prn, e.toe);
        ASSERT_NE(a, nullptr);
        ASSERT_NE(b, nullptr);
        EXPECT_EQ(std::memcmp(a, b, sizeof(gps_eph_t)), 0);
        EXPECT_TRUE(own.hasEphemeris(e.sys, e.prn, e.toe - (e.sys == GNSS_SYS_BDS ? BDT_MINUS_GPST : 0.0)));
    }

    // what did not decode is counted bad either way
    EXPECT_FALSE(rtcm.onEphemeris(kGoldenRmEph[0].bytes, 10));
    gps_eph_t none;
    std::memset(&none, 0, sizeof(none));
    EXPECT_FALSE(own.addEphemeris(none));
    expectSame();
    EXPECT_EQ(own.stats().eph_bad, 1u);
}

TEST(RawGnssMeasStageV9, HasEphemerisTakesBeiDouInItsOwnTime)
{
    // BeiDou's ephemerides keep BeiDou time, 14 s behind GPS: asked at a GPS
    // time, the stage looks 14 s earlier, as the builder does.
    RawGnssMeasStage st;
    gps_eph_t bds = orbit(GNSS_SYS_BDS, 23, 0.0);
    bds.toe = bds.toc = 100000.0;                   // BDT
    gps_eph_t gps = orbit(GNSS_SYS_GPS, 5, 0.0);
    gps.toe = gps.toc = 100000.0;                   // GPS time
    ASSERT_TRUE(st.addEphemeris(bds));
    ASSERT_TRUE(st.addEphemeris(gps));
    const double edge = EphemerisStore::kMaxAgeS;
    // 7207 s of GPS time after toe is 7193 s of BeiDou time: still within 2 h
    EXPECT_TRUE(st.hasEphemeris(GNSS_SYS_BDS, 23, 100000.0 + edge + 7.0));
    EXPECT_FALSE(st.hasEphemeris(GNSS_SYS_GPS, 5, 100000.0 + edge + 7.0));
    // and 7190 s before is 7204 s before in BeiDou time: not
    EXPECT_FALSE(st.hasEphemeris(GNSS_SYS_BDS, 23, 100000.0 - edge + 10.0));
    EXPECT_TRUE(st.hasEphemeris(GNSS_SYS_GPS, 5, 100000.0 - edge + 10.0));
    EXPECT_FALSE(st.hasEphemeris(GNSS_SYS_BDS, 24, 100000.0));   // never filed
    EXPECT_FALSE(st.hasEphemeris(GNSS_SYS_GAL, 23, 100000.0));   // the PRN under another system
}

TEST(RawGnssMeasStageV9, HasEphemerisIsWhatTheTrackedCountGoesBy)
{
    // The golden epochs seen with none of the capture's ephemerides, about
    // half of them, and all: the satellites hasEphemeris says have none are
    // the ones see() counts.
    RawGnssMeasStage st;
    const std::vector<TimedRawEpoch> epochs = goldenEpochs();
    int filed = 0;
    bool some = false;
    for (int upto : {0, kGoldenRmEphN / 2, kGoldenRmEphN})
    {
        for (; filed < upto; ++filed)
            ASSERT_TRUE(st.onEphemeris(kGoldenRmEph[filed].bytes, static_cast<size_t>(kGoldenRmEph[filed].len)));
        for (const TimedRawEpoch& e : epochs)
        {
            st.see(e);
            int tracked = 0, no_eph = 0;
            for (int i = 0; i < e.epoch.n; ++i)
            {
                const RawObs& o = e.epoch.obs[i];
                if (!o.has_pr || o.sys > GNSS_SYS_BDS)
                    continue;
                tracked++;
                no_eph += st.hasEphemeris(o.sys, o.prn, e.epoch.tow_s) ? 0 : 1;
            }
            EXPECT_GT(tracked, 0);
            EXPECT_EQ(st.stats().tracked, tracked);
            EXPECT_EQ(st.stats().tracked_no_eph, no_eph) << upto << " filed";
            if (upto == 0)
                EXPECT_EQ(no_eph, tracked);
            some = some || (no_eph > 0 && no_eph < tracked);
        }
    }
    EXPECT_TRUE(some);   // half the ephemerides: some satellites with one, some without
}

// --------------------------------------------------------------- the cap

TEST(RawGnssMeasStageV9, ThirtySatellitesAreBuiltAsSixteenWithFourGpsFirst)
{
    Sky s = sky(30);
    ASSERT_EQ(s.e.epoch.n, 30);
    int n_gps = 0;
    for (int i = 0; i < s.e.epoch.n; ++i)
        n_gps += s.e.epoch.obs[i].sys == GNSS_SYS_GPS ? 1 : 0;
    ASSERT_GE(n_gps, 6);
    // the GPS satellites the weakest: the cap still takes four of them, the
    // strongest four, and fills the rest from the strongest of the others
    for (int i = 0; i < s.e.epoch.n; ++i)
    {
        RawObs& o = s.e.epoch.obs[i];
        o.cn0_dbhz = o.sys == GNSS_SYS_GPS ? 30.0f + i : 40.0f + i * 0.25f;
    }
    RawGnssMeasStage st(capCfg(16));
    addAll(st, s);
    BuiltEpoch b;
    ASSERT_TRUE(st.offer(s.e, navAt(kRxLla), b));
    ASSERT_EQ(b.n, 16);
    EXPECT_EQ(count(b, GNSS_SYS_GPS), 4);
    EXPECT_EQ(st.stats().tracked, 30);               // seen whole
    EXPECT_EQ(st.stats().tracked_no_eph, 0);
    EXPECT_EQ(st.cappedOut(), 14u);
    // which: the last four GPS (the strongest) and the last twelve others
    std::set<std::pair<int, int>> want;
    for (int i = n_gps - 4; i < n_gps; ++i)
        want.insert({s.e.epoch.obs[i].sys, s.e.epoch.obs[i].prn});
    for (int i = s.e.epoch.n - 12; i < s.e.epoch.n; ++i)
        want.insert({s.e.epoch.obs[i].sys, s.e.epoch.obs[i].prn});
    EXPECT_EQ(sats(b), want);
    // in the epoch's own order
    int last = -1;
    for (int i = 0; i < b.n; ++i)
    {
        int at = -1;
        for (int j = 0; j < s.e.epoch.n; ++j)
            if (s.e.epoch.obs[j].sys == b.meas[i].sys && s.e.epoch.obs[j].prn == b.meas[i].prn)
                at = j;
        EXPECT_GT(at, last);
        last = at;
    }

    // with two GPS satellites, both; with a cap under four, GPS only
    Sky two = sky(30);
    int kept = 0;
    for (int i = 0; i < two.e.epoch.n; ++i)
    {
        RawObs& o = two.e.epoch.obs[i];
        if (o.sys == GNSS_SYS_GPS && kept++ >= 2)
            o.sys = GNSS_SYS_QZS;                    // not a system the filter builds
        o.cn0_dbhz = o.sys == GNSS_SYS_GPS ? 26.0f : 45.0f;
    }
    RawGnssMeasStage st2(capCfg(16));
    addAll(st2, two);
    ASSERT_TRUE(st2.offer(two.e, navAt(kRxLla), b));
    EXPECT_EQ(b.n, 16);
    EXPECT_EQ(count(b, GNSS_SYS_GPS), 2);
    RawGnssMeasStage st3(capCfg(3));
    addAll(st3, s);
    ASSERT_TRUE(st3.offer(s.e, navAt(kRxLla), b));
    EXPECT_EQ(b.n, 3);
    EXPECT_EQ(count(b, GNSS_SYS_GPS), 3);
}

TEST(RawGnssMeasStageV9, TheCapPicksOnlySatellitesTheBuilderWouldBuild)
{
    // A satellite under the C/N0 floor, with no ephemeris or with no
    // pseudorange never takes a place, however strong: the cap's twelve are
    // all built. The tracked count is the whole sky's.
    Sky s = sky(30);
    ASSERT_EQ(s.e.epoch.n, 30);
    RawGnssMeasStage st(capCfg(12));
    for (size_t i = 0; i < s.eph.size(); ++i)
        if (i % 6 != 1)                               // five without an ephemeris
            ASSERT_TRUE(st.addEphemeris(s.eph[i]));
    std::set<std::pair<int, int>> bad;
    for (int i = 0; i < s.e.epoch.n; ++i)
    {
        RawObs& o = s.e.epoch.obs[i];
        o.cn0_dbhz = 50.0f;                           // the strongest, these, unless built
        if (i % 6 == 1)
            bad.insert({o.sys, o.prn});
        else if (i % 6 == 2)
            o.cn0_dbhz = 24.5f;                       // under the floor
        else if (i % 6 == 3)
            o.has_pr = false;
        else
            o.cn0_dbhz = 30.0f + 0.5f * i;
        if (i % 6 == 2 || i % 6 == 3)
            bad.insert({o.sys, o.prn});
    }
    BuiltEpoch b;
    ASSERT_TRUE(st.offer(s.e, navAt(kRxLla), b));
    EXPECT_EQ(b.n, 12);
    for (const auto& k : sats(b))
        EXPECT_EQ(bad.count(k), 0u) << "sys " << k.first << " prn " << k.second;
    EXPECT_EQ(st.stats().tracked, 25);                // with a pseudorange
    EXPECT_EQ(st.stats().tracked_no_eph, 5);          //   and no ephemeris, while capped
    EXPECT_EQ(st.cappedOut(), 3u);                    // the candidates: 30 less 15; twelve taken
}

TEST(RawGnssMeasStageV9, ASatelliteBuiltLastEpochKeepsItsPlaceWithinThreeDb)
{
    Sky s = sky(30);
    ASSERT_EQ(s.e.epoch.n, 30);
    // four GPS satellites well above the rest (the GPS four never change),
    // the other GPS ones below, and the rest spread 0.5 dB apart
    for (int i = 0; i < s.e.epoch.n; ++i)
    {
        RawObs& o = s.e.epoch.obs[i];
        o.cn0_dbhz = o.sys == GNSS_SYS_GPS ? (i < 4 ? 49.0f : 26.0f) : 30.0f + 0.5f * i;
    }
    RawGnssMeasStage st(capCfg(8));
    addAll(st, s);
    BuiltEpoch b;
    ASSERT_TRUE(st.offer(s.e, navAt(kRxLla), b));
    ASSERT_EQ(b.n, 8);
    // the weakest of the four non-GPS picks, and the strongest left out
    const RawObs& in_last = s.e.epoch.obs[s.e.epoch.n - 4];
    const RawObs& out_first = s.e.epoch.obs[s.e.epoch.n - 5];
    const std::pair<int, int> held{in_last.sys, in_last.prn}, waiting{out_first.sys, out_first.prn};
    ASSERT_EQ(sats(b).count(held), 1u);
    ASSERT_EQ(sats(b).count(waiting), 0u);

    // the waiting one 2.5 dB stronger than the held one: not enough
    TimedRawEpoch e = s.e;
    e.epoch.tow_s += 0.1;
    e.mcu_us += 100000;
    find(e, waiting.first, waiting.second)->cn0_dbhz = in_last.cn0_dbhz + 2.5f;
    ASSERT_TRUE(st.offer(e, navAt(kRxLla), b));
    EXPECT_EQ(sats(b).count(held), 1u);
    EXPECT_EQ(sats(b).count(waiting), 0u);
    // 3.5 dB: it takes the place
    e.epoch.tow_s += 0.1;
    e.mcu_us += 100000;
    find(e, waiting.first, waiting.second)->cn0_dbhz = in_last.cn0_dbhz + 3.5f;
    ASSERT_TRUE(st.offer(e, navAt(kRxLla), b));
    EXPECT_EQ(sats(b).count(held), 0u);
    EXPECT_EQ(sats(b).count(waiting), 1u);
    // and now it is the one held: 2.5 dB under the one it replaced, it stays
    e.epoch.tow_s += 0.1;
    e.mcu_us += 100000;
    find(e, waiting.first, waiting.second)->cn0_dbhz = in_last.cn0_dbhz - 2.5f;
    ASSERT_TRUE(st.offer(e, navAt(kRxLla), b));
    EXPECT_EQ(sats(b).count(waiting), 1u);
    EXPECT_EQ(sats(b).count(held), 0u);
}

TEST(RawGnssMeasStageV9, TiesGoToTheLowerSystemThenTheLowerPrnInAnyOrder)
{
    // Every satellite at the same C/N0: four GPS by PRN, then GPS, Galileo,
    // BeiDou by PRN -- whatever order the receiver lists them in.
    Sky s = sky(30);
    ASSERT_EQ(s.e.epoch.n, 30);
    std::vector<std::pair<int, int>> all;
    for (int i = 0; i < s.e.epoch.n; ++i)
        all.push_back({s.e.epoch.obs[i].sys, s.e.epoch.obs[i].prn});   // already by system, then PRN
    const std::set<std::pair<int, int>> want(all.begin(), all.begin() + 12);
    for (int order = 0; order < 3; ++order)
    {
        TimedRawEpoch e = s.e;
        if (order == 1)
            for (int i = 0; i < e.epoch.n / 2; ++i)
                std::swap(e.epoch.obs[i], e.epoch.obs[e.epoch.n - 1 - i]);   // reversed
        if (order == 2)
            for (int i = 0; i + 7 < e.epoch.n; i += 3)
                std::swap(e.epoch.obs[i], e.epoch.obs[i + 7]);               // shuffled
        RawGnssMeasStage st(capCfg(12));
        addAll(st, s);
        BuiltEpoch b;
        ASSERT_TRUE(st.offer(e, navAt(kRxLla), b));
        EXPECT_EQ(sats(b), want) << "order " << order;
        // the epoch's own order kept
        int k = 0;
        for (int i = 0; i < e.epoch.n && k < b.n; ++i)
            if (e.epoch.obs[i].sys == b.meas[k].sys && e.epoch.obs[i].prn == b.meas[k].prn)
                ++k;
        EXPECT_EQ(k, b.n) << "order " << order;
    }
}

TEST(RawGnssMeasStageV9, ACapAboveTheSkyBuildsWhatNoCapBuilds)
{
    // The LC86G's golden epochs and a synthetic sky: a cap of 48 leaves
    // nothing out, and the built epochs are today's, field for field.
    auto expectSame = [](const BuiltEpoch& a, const BuiltEpoch& b) {
        ASSERT_EQ(a.n, b.n);
        EXPECT_EQ(a.mcu_us, b.mcu_us);
        EXPECT_EQ(a.tow_s, b.tow_s);
        EXPECT_EQ(a.gps_week, b.gps_week);
        EXPECT_EQ(a.use_broadcast, b.use_broadcast);
        EXPECT_EQ(a.iono_changed, b.iono_changed);
        for (int k = 0; k < 3; ++k)
            EXPECT_EQ(a.r_approx[k], b.r_approx[k]);
        for (int i = 0; i < a.n; ++i)
        {
            EXPECT_EQ(std::memcmp(&a.meas[i], &b.meas[i], sizeof(a.meas[i])), 0) << "meas " << i;
            EXPECT_EQ(a.lock_age_s[i], b.lock_age_s[i]);
        }
    };
    {
        RawGnssMeasStage off, cap{[] {
            RawGnssMeasStage::Config c;
            c.sat_cap = 48;
            return c;
        }()};
        for (int i = 0; i < kGoldenRmEphN; ++i)
        {
            off.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
            cap.onEphemeris(kGoldenRmEph[i].bytes, static_cast<size_t>(kGoldenRmEph[i].len));
        }
        std::vector<TimedRawEpoch> epochs = goldenEpochs();
        ASSERT_EQ(epochs.size(), 2u);
        for (const TimedRawEpoch& e : epochs)
        {
            BuiltEpoch a, b;
            ASSERT_TRUE(off.offer(e, goldenNav(), a));
            ASSERT_TRUE(cap.offer(e, goldenNav(), b));
            EXPECT_GE(a.n, 7);
            expectSame(a, b);
        }
        EXPECT_EQ(cap.cappedOut(), 0u);
        const gnss_raw::ShadowMeasStats& x = off.stats();
        const gnss_raw::ShadowMeasStats& y = cap.stats();
        EXPECT_EQ(x.epochs, y.epochs);
        EXPECT_EQ(x.built, y.built);
        EXPECT_EQ(x.eph_added, y.eph_added);
        EXPECT_EQ(x.eph_sats, y.eph_sats);
        EXPECT_EQ(x.tracked, y.tracked);
        EXPECT_EQ(x.tracked_no_eph, y.tracked_no_eph);
    }
    {
        RawGnssMeasStage off(capCfg(0)), cap(capCfg(48));
        for (int k = 0; k < 20; ++k)                  // two seconds at 10 Hz
        {
            Sky s = sky(40, kTow0 + 0.1 * k);
            s.e.mcu_us = kT0 + 100000LL * k;
            if (k == 0)
            {
                addAll(off, s);
                addAll(cap, s);
            }
            BuiltEpoch a, b;
            ASSERT_TRUE(off.offer(s.e, navAt(kRxLla), a));
            ASSERT_TRUE(cap.offer(s.e, navAt(kRxLla), b));
            EXPECT_EQ(a.n, s.e.epoch.n);
            expectSame(a, b);
        }
    }
}

TEST(RawGnssMeasStageV9, ASatelliteLeftOutKeepsItsLockAge)
{
    // Four seconds at 10 Hz with the mosaic's 0.5 s relock gap and a cap of
    // 16 over 30 satellites: the GPS four always in, and the other two
    // groups' strengths swapped every second, so each of them is left out a
    // second at a time -- twice the relock gap. Every satellite's lock age
    // runs on from the first epoch all the same: it is the uncapped stage's,
    // and nothing is built as a relock.
    RawGnssMeasStage::Config c = capCfg(16);
    c.meas.relock_gap_s = 0.5;
    RawGnssMeasStage::Config c0 = c;
    c0.sat_cap = 0;
    RawGnssMeasStage st(c), ref(c0), naive(c0);
    Sky s0 = sky(30);
    ASSERT_EQ(s0.e.epoch.n, 30);
    addAll(st, s0);
    addAll(ref, s0);
    addAll(naive, s0);
    std::vector<std::pair<int, int>> others;
    for (int i = 0; i < s0.e.epoch.n; ++i)
        if (s0.e.epoch.obs[i].sys != GNSS_SYS_GPS)
            others.push_back({s0.e.epoch.obs[i].sys, s0.e.epoch.obs[i].prn});
    std::set<std::pair<int, int>> ever_out_and_back;
    std::set<std::pair<int, int>> was_out;
    bool naive_relocked = false;
    for (int k = 0; k <= 40; ++k)
    {
        const double tow = kTow0 + 0.1 * k;
        Sky s = sky(30, tow);
        s.e.mcu_us = kT0 + 100000LL * k;
        const bool even = (k / 10) % 2 == 0;
        int g = 0;
        for (int i = 0; i < s.e.epoch.n; ++i)
        {
            RawObs& o = s.e.epoch.obs[i];
            if (o.sys == GNSS_SYS_GPS)
                o.cn0_dbhz = i < 4 ? 48.0f : 30.0f;
            else
                o.cn0_dbhz = ((g++ % 2 == 0) == even) ? 46.0f : 36.0f;
        }
        BuiltEpoch b, r, nb;
        ASSERT_TRUE(st.offer(s.e, navAt(kRxLla), b));
        ASSERT_TRUE(ref.offer(s.e, navAt(kRxLla), r));
        ASSERT_EQ(b.n, 16);
        EXPECT_EQ(count(b, GNSS_SYS_GPS), 4);
        const std::set<std::pair<int, int>> picked = sats(b);
        for (const auto& sat : others)
        {
            if (picked.count(sat) == 0)
                was_out.insert(sat);
            else if (was_out.count(sat) != 0)
                ever_out_and_back.insert(sat);
        }
        for (int i = 0; i < b.n; ++i)
        {
            EXPECT_NEAR(b.lock_age_s[i], tow - kTow0, 1e-9);
            int j = 0;
            while (j < r.n && !(r.meas[j].sys == b.meas[i].sys && r.meas[j].prn == b.meas[i].prn))
                ++j;
            ASSERT_LT(j, r.n);
            EXPECT_EQ(b.lock_age_s[i], r.lock_age_s[j]);
            EXPECT_EQ(b.meas[i].sigma_pr_corr, r.meas[j].sigma_pr_corr);   // the relock term decays as the reference's
            EXPECT_EQ(b.meas[i].sigma_rr, r.meas[j].sigma_rr);
        }
        // what a stage shown only the picks would make of it: relocks
        TimedRawEpoch only = s.e;
        only.epoch.n = 0;
        for (int i = 0; i < s.e.epoch.n; ++i)
            if (picked.count({s.e.epoch.obs[i].sys, s.e.epoch.obs[i].prn}) != 0)
                only.epoch.obs[only.epoch.n++] = s.e.epoch.obs[i];
        ASSERT_TRUE(naive.offer(only, navAt(kRxLla), nb));
        for (int i = 0; i < nb.n; ++i)
            naive_relocked = naive_relocked || nb.lock_age_s[i] < tow - kTow0 - 0.5;
    }
    EXPECT_GE(ever_out_and_back.size(), 10u);   // left out a second and back, more than once over
    EXPECT_TRUE(naive_relocked);                // the test can tell
    EXPECT_EQ(st.stats().tracked, 30);
}

// ------------------------------------------------------- the store's size

TEST(RawGnssMeasStageV9, TheStoreKeepsTheBeetlesSizeByDefault)
{
    // TR_EPH_CAPACITY is the mosaic build's alone (128); every other build,
    // the Beetle's among them, keeps the pool it had.
    struct Before
    {
        gps_eph_t pool[64];
        size_t    used;
    };
    static_assert(EphemerisStore::kCapacity == 64, "the default store holds 64");
    static_assert(sizeof(EphemerisStore) == sizeof(Before), "the default store's size is unchanged");
    EphemerisStore store;
    EXPECT_EQ(store.size(), 0u);
}
