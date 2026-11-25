// Host tests for when the mosaic-only image's GNSS task files the receiver's
// navigation blocks (flight_computer/main/raw_nav_drain.h; registered in
// v9raw/raw_nav_drain.cmake), through the real decoder (SbfNav.cpp) and the
// real measurement stage (RawGnssMeasStage::addEphemeris). The store goes by
// second of week alone, and the stage can refuse a copy weeks old only once
// an epoch has named the receiver's week -- so a stored copy in the boot's
// burst, queued before the first epoch, must wait for one. Without the wait
// it is filed and placed as today's (the first test shows the failure the
// wait prevents).
//
// The blocks are synthetic GPSNav blocks of a synthetic orbit, no real
// satellite or place.
#include <gtest/gtest.h>

#include <cstring>
#include <deque>
#include <vector>

#include "RawGnssMeasStage.h"
#include "SbfNav.h"
#include "gnss/eph.h"
#include "gnss/types.h"
#include "raw_nav_drain.h"
#include "sbf_build.h"

using gnss_raw::RawGnssMeasStage;
using gnss_raw::TimedRawEpoch;

namespace {

constexpr uint16_t kWeek = 2440;
constexpr double   kTow = 302400.0;   // midweek, the receiver's time of week
constexpr double   kPi = 3.14159265358979323846;

gps_eph_t orbit(int prn, int week, double toe)
{
    gps_eph_t e;
    std::memset(&e, 0, sizeof(e));
    e.valid = 1;
    e.sys = GNSS_SYS_GPS;
    e.prn = prn;
    e.week = week;
    e.iode = 7;
    e.iodc = 7;
    e.toe = e.toc = toe;
    e.sqrt_a = 5153.7;
    e.e = 0.005;
    e.i0 = 55.0 * kPi / 180.0;
    e.omega0 = (prn % 6) * kPi / 3.0;
    e.m0 = prn * 0.7;
    return e;
}

// A GPSNav block as the receiver sends it (Reference Guide 4.2, GPSNav 5891;
// the layout test_sbf_nav.cpp checks against the RTCM goldens), stamped with
// the GPS week it was received in (the block's WNc).
std::vector<uint8_t> gpsNav(const gps_eph_t& e, uint16_t wnc)
{
    std::vector<uint8_t> v(140 - 14, 0);
    auto put = [&](size_t at, const void* p, size_t n) { std::memcpy(v.data() + (at - 14), p, n); };
    auto u8 = [&](size_t at, uint8_t x) { put(at, &x, 1); };
    auto u16 = [&](size_t at, uint16_t x) { const uint8_t b[2] = {uint8_t(x), uint8_t(x >> 8)}; put(at, b, 2); };
    auto u32 = [&](size_t at, uint32_t x) {
        const uint8_t b[4] = {uint8_t(x), uint8_t(x >> 8), uint8_t(x >> 16), uint8_t(x >> 24)};
        put(at, b, 4);
    };
    auto f32 = [&](size_t at, double x) { const float f = static_cast<float>(x); put(at, &f, 4); };
    auto f64 = [&](size_t at, double x) { put(at, &x, 8); };
    u8(14, static_cast<uint8_t>(e.prn));
    u16(16, static_cast<uint16_t>(e.week % 1024));
    u8(18, 1);
    u16(22, static_cast<uint16_t>(e.iodc));
    u8(24, static_cast<uint8_t>(e.iode));
    u8(25, static_cast<uint8_t>(e.iode));
    u32(32, static_cast<uint32_t>(e.toc));
    f64(56, e.m0 / GPS_PI_ICD);
    f64(68, e.e);
    f64(80, e.sqrt_a);
    u32(88, static_cast<uint32_t>(e.toe));
    f64(96, e.omega0 / GPS_PI_ICD);
    f64(108, e.i0 / GPS_PI_ICD);
    u16(136, static_cast<uint16_t>(e.week % 1024));
    u16(138, static_cast<uint16_t>(e.week % 1024));
    return sbftest::block(mosaic::kSbfGpsNav, 0, 345600000, wnc, v);
}

// The driver's queue, as the GNSS task takes from it.
struct Queue
{
    std::deque<std::vector<uint8_t>> q;
    int takes = 0;
    bool take(uint8_t* out, size_t cap, size_t& len)
    {
        takes++;
        if (q.empty()) return false;
        const std::vector<uint8_t> b = q.front();
        q.pop_front();
        if (b.size() > cap) return false;
        std::memcpy(out, b.data(), b.size());
        len = b.size();
        return true;
    }
};

TimedRawEpoch epochAt(uint16_t week, double tow)
{
    TimedRawEpoch e;
    e.epoch.tow_s = tow;
    e.gps_week = week;
    return e;
}

// One pass of the GNSS task: the blocks, then the epochs (main.cpp).
void pass(RawNavDrain::State& d, Queue& q, RawGnssMeasStage& st, const std::vector<TimedRawEpoch>& epochs,
          bool* week_named = nullptr)
{
    RawNavDrain::drain(
        d, [&](uint8_t* b, size_t cap, size_t& n) { return q.take(b, cap, n); },
        [&](const gps_eph_t& e) { (void)st.addEphemeris(e); });
    for (const TimedRawEpoch& e : epochs)
    {
        const bool named = RawNavDrain::noteWeek(d, e.gps_week);
        if (week_named != nullptr && named) *week_named = true;
        st.skip(e);   // see() before anything else, as offer() does
    }
}

}  // namespace

// The failure the wait prevents: the stage files a week-old copy offered
// before any epoch, and then places the satellite by it at today's time.
TEST(RawNavDrain, WithoutTheWaitAStoredCopyIsTakenAsToday)
{
    RawGnssMeasStage st;
    const std::vector<uint8_t> stale = gpsNav(orbit(5, kWeek - 1, kTow), kWeek - 1);
    gps_eph_t e;
    ASSERT_EQ(mosaic::decodeSbfEphemeris(stale.data(), stale.size(), mosaic::kWncDnu, e), mosaic::SbfEphStatus::OK);
    EXPECT_EQ(e.week, kWeek - 1);
    EXPECT_TRUE(st.addEphemeris(e));   // no week yet: not checked
    st.skip(epochAt(kWeek, kTow));
    EXPECT_TRUE(st.hasEphemeris(GNSS_SYS_GPS, 5, kTow));   // a week-old orbit, used as today's
}

// The boot's burst: a stored copy a week old and a current one, queued
// before the GNSS task's first pass. Nothing is taken until an epoch names
// the week; then the old copy is refused and the current one filed.
TEST(RawNavDrain, BlocksWaitForTheWeekAndAStoredCopyIsRefused)
{
    RawGnssMeasStage st;
    RawNavDrain::State d;
    Queue q;
    q.q.push_back(gpsNav(orbit(5, kWeek - 1, kTow), kWeek - 1));   // stored last week, same time of week
    q.q.push_back(gpsNav(orbit(6, kWeek, kTow), kWeek));

    pass(d, q, st, {});   // the first pass: no epoch yet
    EXPECT_TRUE(d.held);
    EXPECT_EQ(q.takes, 0);
    EXPECT_EQ(q.q.size(), 2u);
    EXPECT_EQ(st.stats().eph_added, 0u);

    bool named = false;
    pass(d, q, st, {epochAt(kWeek, kTow)}, &named);   // the first epoch names the week
    EXPECT_TRUE(named);
    EXPECT_EQ(d.week, kWeek);
    EXPECT_EQ(q.q.size(), 2u);   // still waiting: this pass took the blocks before its epoch

    pass(d, q, st, {epochAt(kWeek, kTow + 0.1)});
    EXPECT_TRUE(q.q.empty());
    EXPECT_EQ(st.stats().eph_bad, 1u);     // the week-old copy
    EXPECT_EQ(st.stats().eph_added, 1u);   // the current one
    EXPECT_FALSE(st.hasEphemeris(GNSS_SYS_GPS, 5, kTow));
    EXPECT_TRUE(st.hasEphemeris(GNSS_SYS_GPS, 6, kTow));
    EXPECT_EQ(d.refused, 0u);
}

// Once the week is named, blocks are filed on the pass that takes them, and
// a block whose own WNc stamp is do-not-use is made whole against the
// receiver's week.
TEST(RawNavDrain, AfterTheWeekBlocksAreFiledAsTheyCome)
{
    RawGnssMeasStage st;
    RawNavDrain::State d;
    Queue q;
    pass(d, q, st, {epochAt(kWeek, kTow)});
    q.q.push_back(gpsNav(orbit(7, kWeek, kTow + 3600.0), kWeek));
    q.q.push_back(gpsNav(orbit(8, kWeek, kTow), mosaic::kWncDnu));
    pass(d, q, st, {});
    EXPECT_TRUE(q.q.empty());
    EXPECT_EQ(st.stats().eph_added, 2u);
    EXPECT_TRUE(st.hasEphemeris(GNSS_SYS_GPS, 7, kTow + 3600.0));
    EXPECT_TRUE(st.hasEphemeris(GNSS_SYS_GPS, 8, kTow));
}

// What SbfNav refuses is counted, and nothing of it reaches the stage.
TEST(RawNavDrain, BlocksTheDecoderRefusesAreCounted)
{
    RawGnssMeasStage st;
    RawNavDrain::State d;
    Queue q;
    pass(d, q, st, {epochAt(kWeek, kTow)});
    gps_eph_t cut = orbit(9, kWeek, kTow);
    cut.iodc = 8;   // IODC's low byte against IODE: a GPS cutover
    q.q.push_back(gpsNav(cut, kWeek));
    std::vector<uint8_t> shortened = gpsNav(orbit(10, kWeek, kTow), kWeek);
    shortened.resize(40);
    q.q.push_back(shortened);
    pass(d, q, st, {});
    EXPECT_EQ(d.refused, 2u);
    EXPECT_EQ(st.stats().eph_added, 0u);
    EXPECT_EQ(st.stats().eph_bad, 0u);
}

// noteWeek: an epoch with no week names none; the first week named after a
// held pass is said once (the caller's one resend), never again.
TEST(RawNavDrain, TheWeekIsNamedOnce)
{
    RawNavDrain::State d;
    EXPECT_FALSE(RawNavDrain::noteWeek(d, 0));
    EXPECT_FALSE(RawNavDrain::noteWeek(d, mosaic::kWncDnu));
    EXPECT_EQ(d.week, mosaic::kWncDnu);
    EXPECT_TRUE(RawNavDrain::drain(
                    d, [](uint8_t*, size_t, size_t&) { return false; }, [](const gps_eph_t&) {}) == false);
    EXPECT_TRUE(d.held);
    EXPECT_TRUE(RawNavDrain::noteWeek(d, kWeek));
    EXPECT_FALSE(RawNavDrain::noteWeek(d, kWeek));
    EXPECT_FALSE(RawNavDrain::noteWeek(d, kWeek + 1));   // the week's turn: followed, not said
    EXPECT_EQ(d.week, kWeek + 1);

    RawNavDrain::State never_held;   // a week named before any pass held nothing to ask again for
    EXPECT_FALSE(RawNavDrain::noteWeek(never_held, kWeek));
}
