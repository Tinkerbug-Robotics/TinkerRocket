// Host tests for EkfGnssFeed (#1107) — what the flight loop hands the EKF as
// its GNSS input on every tick, and the filter-side contract it relies on.
//
// The loop used to keep its own "consumed" markers beside the filter's dedup
// and, once they said "seen", handed the EKF a zeroed EkfGNSSDataLLA under the
// marker's timestamp.  On a tick where updateCore() returned at the #440
// frozen-IMU-timestamp skip the markers advanced but the filter never fused
// the fix, and the next tick fused lat=0/lon=0/alt=0/vel=0 as a real fix.
// The feed holds the last accepted fix instead and re-presents it unchanged
// until a newer one arrives; the filter fuses each time_us once.
#include <gtest/gtest.h>
#include "EkfGnssFeed.h"
#include "TR_GpsInsEKF.h"
#include <cmath>

namespace {

constexpr double LAT_DEG = 33.7, LON_DEG = -118.4, ALT_M = 100.0;
constexpr double DEG2RAD = M_PI / 180.0;
constexpr uint32_t TICK_US = 2000;         // ~500 Hz EKF
constexpr int TICKS_PER_FIX = 27;           // ~18.5 Hz GNSS

// A receiver record as the converter produces it: `fix_no` stands in for the
// receiver's own fix time (second + millisecond), `time_us` for the loop's
// receive stamp.
GNSSDataSI record(uint32_t time_us, uint32_t fix_no,
                  double lat_deg = LAT_DEG, double lon_deg = LON_DEG, double alt_m = ALT_M,
                  double vel_e = 0.0, double vel_n = 0.0, double vel_u = 0.0)
{
    GNSSDataSI r = {};
    r.time_us = time_us;
    r.second = (uint8_t)(fix_no / 20u);
    r.milli_second = (uint16_t)((fix_no % 20u) * 50u);
    r.fix_mode = 3; r.num_sats = 12; r.horizontal_accuracy = 1.5f;
    r.lat = lat_deg; r.lon = lon_deg; r.alt = alt_m;
    r.vel_e = vel_e; r.vel_n = vel_n; r.vel_u = vel_u;
    return r;
}

// Nose-up stationary fixtures consistent with the EKF's init attitude.
EkfIMUData imuNoseUp(uint32_t t) { EkfIMUData i; i.time_us = t; i.acc_x = 9.807; return i; }
EkfMagData magNoseUp(uint32_t t) { EkfMagData m; m.time_us = t; m.mag_x = -42.0; m.mag_z = 22.0; return m; }

// Metres between two EKF LLA estimates (small-angle, NED).
double horizontalMetres(const double a[3], const double b[3])
{
    const double R = 6371000.0;
    const double dn = (b[0] - a[0]) * R;
    const double de = (b[1] - a[1]) * R * std::cos(a[0]);
    return std::sqrt(dn * dn + de * de);
}

// Bring a filter to a converged stationary state through the feed: the
// caller's pattern exactly (offer every tick, hand the EKF current()).
struct Rig {
    GpsInsEKF ekf;
    EkfGnssFeed feed;
    uint32_t t = 1000;
    uint32_t fix_no = 1;

    Rig()
    {
        feed.offer(record(t, fix_no), true);
        ekf.init(imuNoseUp(t), feed.current(), magNoseUp(t));
    }
    void tick(bool new_fix, bool run_ekf = true)
    {
        t += TICK_US;
        if (new_fix) fix_no++;
        feed.offer(record(t, fix_no), true);
        if (run_ekf) ekf.update(true, imuNoseUp(t), feed.current(), magNoseUp(t));
    }
    void settle(int seconds)
    {
        const int n = seconds * 1000000 / TICK_US;
        for (int i = 1; i <= n; i++) tick(i % TICKS_PER_FIX == 0);
    }
};

} // namespace

// ---------- the feed on its own ----------

TEST(EkfGnssFeed, NothingIsFabricatedBeforeTheFirstFix)
{
    EkfGnssFeed feed;
    EXPECT_FALSE(feed.haveFix());
    const EkfGNSSDataLLA& g = feed.current();
    EXPECT_EQ(g.time_us, 0u);
    EXPECT_EQ(g.lat_rad, 0.0);
    EXPECT_EQ(g.lon_rad, 0.0);
    EXPECT_EQ(g.alt_m, 0.0);
    EXPECT_EQ(g.vel_n_mps, 0.0f);
    EXPECT_EQ(g.vel_e_mps, 0.0f);
    EXPECT_EQ(g.vel_d_mps, 0.0f);

    // A rejected record is not held either.
    EXPECT_FALSE(feed.offer(record(5000, 1), false));
    EXPECT_FALSE(feed.haveFix());
    EXPECT_EQ(feed.current().time_us, 0u);
}

TEST(EkfGnssFeed, HeldFixMirrorsTheRecordInEkfUnits)
{
    EkfGnssFeed feed;
    ASSERT_TRUE(feed.offer(record(123456, 7, LAT_DEG, LON_DEG, ALT_M, 1.5, -2.0, 3.0), true));
    ASSERT_TRUE(feed.haveFix());
    const EkfGNSSDataLLA& g = feed.current();
    EXPECT_EQ(g.time_us, 123456u);
    EXPECT_DOUBLE_EQ(g.lat_rad, LAT_DEG * DEG2RAD);
    EXPECT_DOUBLE_EQ(g.lon_rad, LON_DEG * DEG2RAD);
    EXPECT_DOUBLE_EQ(g.alt_m, ALT_M);
    EXPECT_FLOAT_EQ(g.vel_n_mps, -2.0f);
    EXPECT_FLOAT_EQ(g.vel_e_mps, 1.5f);
    EXPECT_FLOAT_EQ(g.vel_d_mps, -3.0f);   // ENU up -> NED down
}

TEST(EkfGnssFeed, OfferReportsEachAcceptedFixExactlyOnce)
{
    EkfGnssFeed feed;
    // The loop re-reads the same fix for ~25 ticks: reported new once.
    EXPECT_TRUE(feed.offer(record(1000, 1), true));
    for (int i = 0; i < 30; i++) {
        EXPECT_FALSE(feed.offer(record(1000, 1), true)) << "tick " << i;
    }
    EXPECT_EQ(feed.current().time_us, 1000u);

    // A new fix time is a new fix.
    EXPECT_TRUE(feed.offer(record(56000, 2), true));
    EXPECT_EQ(feed.current().time_us, 56000u);

    // A rejected newer record does not replace the held fix and does not
    // advance the arrival key: once it passes the gate it is reported new.
    EXPECT_FALSE(feed.offer(record(111000, 3), false));
    EXPECT_EQ(feed.current().time_us, 56000u);
    EXPECT_TRUE(feed.offer(record(111000, 3), true));
    EXPECT_EQ(feed.current().time_us, 111000u);
}

TEST(EkfGnssFeed, ResetForgetsTheHeldFix)
{
    EkfGnssFeed feed;
    ASSERT_TRUE(feed.offer(record(1000, 1), true));
    feed.reset();
    EXPECT_FALSE(feed.haveFix());
    EXPECT_EQ(feed.current().time_us, 0u);
    EXPECT_EQ(feed.current().lat_rad, 0.0);
    // Same receiver fix time as before the reset: still reported new (a sim
    // restart replays synthetic fixes from the start).
    EXPECT_TRUE(feed.offer(record(1000, 1), true));
}

// ---------- the feed driving the real filter ----------

// The filter-side contract the feed relies on: a fix is fused exactly once,
// however many ticks it is re-presented on.
TEST(EkfGnssFeed, FilterFusesEachHeldFixOnce)
{
    Rig rig;
    rig.settle(5);
    const uint32_t fused = rig.ekf.lastGnssTimeUs();
    EXPECT_EQ(fused, rig.feed.current().time_us);

    float p_after_fix[3];
    rig.ekf.getCovPos(p_after_fix);
    // 20 more ticks of the same held fix: no further fusion (the position
    // covariance only grows between fixes, it never contracts again).
    for (int i = 0; i < 20; i++) {
        rig.tick(false);
        EXPECT_EQ(rig.ekf.lastGnssTimeUs(), fused) << "tick " << i;
        float p[3];
        rig.ekf.getCovPos(p);
        EXPECT_GE(p[0], p_after_fix[0]) << "tick " << i;
        p_after_fix[0] = p[0];
    }
}

// #367 parity: a fix that arrives on an EKF-off decimation tick is not lost —
// it is still the held fix on the next EKF tick and is fused then.
TEST(EkfGnssFeed, FixArrivingOnAnEkfOffTickIsFusedOnTheNextEkfTick)
{
    Rig rig;
    rig.settle(5);
    rig.tick(true, /*run_ekf=*/false);         // fix arrives, EKF decimated off
    const uint32_t fix_time = rig.feed.current().time_us;
    EXPECT_NE(rig.ekf.lastGnssTimeUs(), fix_time);
    rig.tick(false);                            // EKF on, nothing new arrived
    EXPECT_EQ(rig.ekf.lastGnssTimeUs(), fix_time);
}

// #1107 itself: the fix arrives on a tick whose IMU timestamp is frozen, so
// updateCore() returns at the #440 skip before its GNSS block.  The next tick
// must fuse THAT fix — not a zeroed placeholder — so the estimate stays at
// the site and the descent rate is untouched.
TEST(EkfGnssFeed, FixArrivingOnAFrozenImuTickIsFusedNextTickNotNullIsland)
{
    Rig rig;
    rig.settle(20);
    // A converged filter in a 20 m/s descent (the failure scenario's state).
    rig.ekf.setVelocity(0.0f, 0.0f, 20.0f);
    double p0[3]; rig.ekf.getPosEst(p0);

    // Tick N: the IMU timestamp does NOT advance; a new fix (reporting the
    // same 20 m/s descent, 3 m north of the estimate) arrives on this tick.
    rig.fix_no++;
    const double north3m_deg = 3.0 / 6371000.0 / DEG2RAD;
    rig.feed.offer(record(rig.t, rig.fix_no, LAT_DEG + north3m_deg, LON_DEG, ALT_M,
                          0.0, 0.0, -20.0), true);
    const uint32_t fix_time = rig.feed.current().time_us;
    const uint32_t skips_before = rig.ekf.frozenDtSkips();
    rig.ekf.update(true, imuNoseUp(rig.t), rig.feed.current(), magNoseUp(rig.t));
    ASSERT_EQ(rig.ekf.frozenDtSkips(), skips_before + 1) << "test premise: tick N was skipped";
    ASSERT_NE(rig.ekf.lastGnssTimeUs(), fix_time) << "test premise: the fix was not fused on tick N";

    // Tick N+1: 2 ms later, nothing new from the receiver.
    rig.t += TICK_US;
    rig.feed.offer(record(rig.t - TICK_US, rig.fix_no, LAT_DEG + north3m_deg, LON_DEG, ALT_M,
                          0.0, 0.0, -20.0), true);   // the loop re-reads the same fix
    rig.ekf.update(true, imuNoseUp(rig.t), rig.feed.current(), magNoseUp(rig.t));

    // The held fix was fused now ...
    EXPECT_EQ(rig.ekf.lastGnssTimeUs(), fix_time);
    // ... and the estimate moved toward IT (north, by less than the 3 m
    // offset), not toward 0N/0E.
    double p1[3]; rig.ekf.getPosEst(p1);
    const double dn_m = (p1[0] - p0[0]) * 6371000.0;
    EXPECT_GT(dn_m, 0.0);
    EXPECT_LT(dn_m, 3.0);
    EXPECT_LT(horizontalMetres(p0, p1), 3.0);
    EXPECT_NEAR(p1[0], LAT_DEG * DEG2RAD, 1e-5);
    EXPECT_NEAR(p1[1], LON_DEG * DEG2RAD, 1e-5);
    // The descent rate is untouched: the old placeholder pulled it ~1.5 m/s
    // toward zero per occurrence.
    float v[3]; rig.ekf.getVelEst(v);
    EXPECT_NEAR(v[2], 20.0f, 0.05f);
    EXPECT_TRUE(rig.ekf.isHealthy());
}
