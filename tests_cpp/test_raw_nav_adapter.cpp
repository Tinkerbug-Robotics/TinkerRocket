// test_raw_nav_adapter.cpp — GpsInsEKF's face on the raw-GNSS filter's state,
// the Beetle's navigation filter (raw_nav_adapter.h, #1600): what it answers
// before the raw filter runs (where it will start) and after (its state,
// heights turned from the ellipsoid to the geoid), when it stops calling the
// filter healthy, the snapshot a reboot restores, and the magnetometer and
// the start it hands the raw filter.
#include <gtest/gtest.h>

#include <cmath>

#include "raw_nav_adapter.h"

using gnss_raw::ShadowLink;
using gnss_raw::ShadowMag;
using gnss_raw::ShadowNav;
using gnss_raw::ShadowSeed;

namespace
{

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr float kSep = -33.1f;   // the rp5_0829 pad's geoid separation (the receiver's)

ShadowNav navAt(double h_ell, float pitch_deg)
{
    ShadowNav n{};
    n.time_us = 1000000;
    n.lla[0] = 39.4683565 * kDeg;
    n.lla[1] = -75.29286495 * kDeg;
    n.lla[2] = h_ell;
    n.vel_ned[0] = 1.0f;
    n.vel_ned[1] = 2.0f;
    n.vel_ned[2] = -90.0f;
    const double p = pitch_deg * kDeg;
    n.quat[0] = float(std::cos(p / 2));   // pitch about body y: nose up
    n.quat[2] = float(std::sin(p / 2));
    for (int i = 0; i < 15; ++i)
        n.var[i] = 0.01f * float(i + 1);
    n.gyro_bias[2] = 0.01f;
    n.running = 1;
    n.gnss_aided = 1;
    return n;
}

}  // namespace

TEST(RawNavAdapter, BeforeTheRawFilterRunsItAnswersWhereItWillStart)
{
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    ekf.setGeoidSeparation(kSep);
    ekf.refresh(10000);
    EXPECT_FALSE(ekf.isHealthy());

    EkfIMUData imu;
    imu.gyro_z = 0.5;                            // deg/s: the still gyro's mean
    EkfGNSSDataLLA gnss;
    gnss.lat_rad = 39.4683565 * kDeg;
    gnss.lon_rad = -75.29286495 * kDeg;
    gnss.alt_m = 32.4;                           // the receiver's, above the geoid
    ekf.init(imu, gnss, EkfMagData());
    ekf.setQuaternion(0.70710678f, 0.0f, 0.70710678f, 0.0f);   // nose up, heading a guess
    ekf.setDeclination(-11.6f * float(kDeg));
    const float ref[3] = {21.0f, -4.3f, 45.5f};
    ekf.setMagReference(ref);

    ShadowSeed s{};
    ekf.startSeed(s);
    EXPECT_DOUBLE_EQ(s.lla[0], gnss.lat_rad);
    EXPECT_NEAR(s.lla[2], 32.4 + kSep, 1e-4);    // above the ellipsoid
    EXPECT_FLOAT_EQ(s.quat[2], 0.70710678f);
    EXPECT_NEAR(s.gyro_bias[2], 0.5f * float(kDeg), 1e-7f);
    EXPECT_EQ(s.have_declination, 1);
    EXPECT_NEAR(s.declination_rad, -11.6f * float(kDeg), 1e-6f);
    EXPECT_NEAR(s.field_ut, 50.3f, 0.1f);
    EXPECT_EQ(s.restore, 0);
    EXPECT_EQ(s.start_gen, 1);

    // what main.cpp reads until it runs: the start, above the geoid
    double p[3];
    ekf.getPosEst(p);
    EXPECT_NEAR(p[2], 32.4, 1e-4);
    float q[4];
    ekf.getQuaternion(q);
    EXPECT_FLOAT_EQ(q[0], 0.70710678f);
    EXPECT_FALSE(ekf.isHealthy());

    // a second start (the flight computer started over): a new generation
    ekf.init(imu, gnss, EkfMagData());
    ekf.startSeed(s);
    EXPECT_EQ(s.start_gen, 2);
}

TEST(RawNavAdapter, RunningItAnswersFromTheRawFilter)
{
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    ekf.setGeoidSeparation(kSep);
    link.nav.write(navAt(500.0, 80.0f));
    ekf.refresh(2000000);
    ASSERT_TRUE(ekf.isHealthy());

    double p[3];
    ekf.getPosEst(p);
    EXPECT_NEAR(p[2], 500.0 - kSep, 1e-4);       // above the geoid, as main.cpp keeps heights
    float v[3];
    ekf.getVelEst(v);
    EXPECT_FLOAT_EQ(v[2], -90.0f);
    float e[3];
    ekf.getOrientEst(e);
    EXPECT_NEAR(e[1], 80.0f * float(kDeg), 1e-4f);   // GpsInsEKF's Euler: the pitch voter's input
    EXPECT_NEAR(e[0], 0.0f, 1e-4f);
    float c[3];
    ekf.getCovOrient(c);
    EXPECT_FLOAT_EQ(c[0], 0.07f);
    ekf.getCovVel(c);
    EXPECT_FLOAT_EQ(c[2], 0.06f);
    float wb[3];
    ekf.getRotRateBias(wb);
    EXPECT_FLOAT_EQ(wb[2], 0.01f);
    EXPECT_NEAR(ekf.gyroBiasMaxDps(), 0.573f, 1e-3f);
    EXPECT_TRUE(ekf.gnssAided());
}

TEST(RawNavAdapter, ACoreThatStopsPublishingIsNotHealthy)
{
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    link.nav.write(navAt(100.0, 85.0f));
    ekf.refresh(5000000);
    EXPECT_TRUE(ekf.isHealthy());
    ekf.refresh(5100000);                        // nothing new for 0.1 s: still
    EXPECT_TRUE(ekf.isHealthy());
    ekf.refresh(5300000);                        // 0.3 s
    EXPECT_FALSE(ekf.isHealthy());
    link.nav.write(navAt(100.0, 85.0f));
    ekf.refresh(5310000);
    EXPECT_TRUE(ekf.isHealthy());
    ShadowNav stopped{};
    link.nav.write(stopped);                     // it stopped
    ekf.refresh(5320000);
    EXPECT_FALSE(ekf.isHealthy());
    ShadowNav nan = navAt(100.0, 85.0f);
    nan.vel_ned[1] = std::nanf("");
    link.nav.write(nan);
    ekf.refresh(5330000);
    EXPECT_FALSE(ekf.isHealthy());
}

TEST(RawNavAdapter, TheSnapshotRestoresTheRawFiltersOwnHeight)
{
    // The receiver a reboot cold-starts has no geoid separation for a while:
    // the snapshot keeps the raw filter's height above the ellipsoid
    ShadowLink link;
    RawNavAdapter before;
    before.attach(&link);
    before.setGeoidSeparation(kSep);
    link.nav.write(navAt(612.5, -20.0f));
    before.refresh(1000);
    EkfStateSnapshot snap;
    before.getState(snap);
    EXPECT_NEAR(snap.pos_rrm[2], 612.5, 1e-4);
    EXPECT_FLOAT_EQ(snap.P[6][6], 0.07f);

    RawNavAdapter after;                          // rebooted: no separation yet, nothing running
    ShadowLink link2;
    after.attach(&link2);
    after.setState(snap);
    EXPECT_TRUE(after.restorePending());
    ShadowSeed s{};
    after.startSeed(s);
    EXPECT_EQ(s.restore, 1);
    EXPECT_NEAR(s.lla[2], 612.5, 1e-4);
    EXPECT_FLOAT_EQ(s.vel_ned[2], -90.0f);
    for (int i = 0; i < 4; ++i)
        EXPECT_FLOAT_EQ(s.quat[i], snap.quat[i]);
    // once the raw filter runs, the restore is done
    link2.nav.write(navAt(600.0, -20.0f));
    after.refresh(2000);
    EXPECT_FALSE(after.restorePending());
}

TEST(RawNavAdapter, TheMagnetometerGoesToTheRawFilterOnce)
{
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    const float ref[3] = {21.0f, -4.3f, 45.5f};   // 50.3 uT
    ekf.setMagReference(ref);
    EkfMagData m;
    m.time_us = 1234;
    m.mag_x = 30.0;
    m.mag_y = 10.0;
    m.mag_z = 38.0;                               // 49.4 uT
    ekf.update(true, EkfIMUData(), EkfGNSSDataLLA(), m);
    ekf.update(true, EkfIMUData(), EkfGNSSDataLLA(), m);   // the same sample: once
    ShadowMag got{};
    ASSERT_TRUE(link.mag.pop(got));
    EXPECT_FLOAT_EQ(got.mag[2], 38.0f);
    EXPECT_FALSE(link.mag.pop(got));
    EXPECT_FALSE(ekf.getMagCalSuspect());
    m.time_us = 1300;
    m.mag_z = 60.0;                               // 68 uT: a hard iron
    ekf.update(true, EkfIMUData(), EkfGNSSDataLLA(), m);
    EXPECT_TRUE(ekf.getMagCalSuspect());
}
