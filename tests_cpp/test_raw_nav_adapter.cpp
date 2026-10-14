// test_raw_nav_adapter.cpp — GpsInsEKF's face on the raw-GNSS filter's state,
// the Beetle's navigation filter (raw_nav_adapter.h, #1600): what it answers
// before the raw filter runs (where it will start) and after (its state,
// heights turned from the ellipsoid to the geoid, brought forward to the
// latest IMU tick, #1626 K2), when it stops calling the filter healthy, the
// snapshot a reboot restores, and the magnetometer and the start it hands
// the raw filter.
#include <gtest/gtest.h>

#include <cmath>

#include "TR_RawGnssEKF.h"
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
    ekf.refresh(1010000);                        // 10 ms after the state's tick
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
    ShadowNav n = navAt(100.0, 85.0f);
    n.time_us = 4995000;
    link.nav.write(n);
    ekf.refresh(5000000);
    EXPECT_TRUE(ekf.isHealthy());
    ekf.refresh(5100000);                        // nothing new for 0.1 s: still
    EXPECT_TRUE(ekf.isHealthy());
    ekf.refresh(5300000);                        // 0.3 s
    EXPECT_FALSE(ekf.isHealthy());
    n.time_us = 5305000;
    link.nav.write(n);
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

TEST(RawNavAdapter, ItSaysHowOldAStateTheLoopRead)
{
    // the raw filter publishes after every step; the loop reads the latest:
    // the oldest it was handed in a window is the diagnostic (#1600)
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    ShadowNav n = navAt(100.0, 85.0f);
    n.time_us = 5000000;
    link.nav.write(n);
    ekf.refresh(5004000);                        // 4 ms old
    ekf.refresh(5012000);                        // still that state: 12 ms
    EXPECT_EQ(ekf.takeMaxAgeUs(), 12000u);
    EXPECT_EQ(ekf.takeMaxAgeUs(), 0u);           // taken: a new window
    n.time_us = 5010000;
    link.nav.write(n);
    ekf.refresh(5013000);
    EXPECT_EQ(ekf.takeMaxAgeUs(), 3000u);
    n.time_us = 5020000;                         // a state stamped after the loop's clock
    link.nav.write(n);
    ekf.refresh(5015000);
    EXPECT_EQ(ekf.takeMaxAgeUs(), 0u);           // is not a 4294-second-old one
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

// ------------------------------------------- brought forward to now (#1626 K2)

namespace
{

gnss_raw::ShadowImu tick(uint32_t t_us, const float acc[3], const float gyro[3])
{
    gnss_raw::ShadowImu s{};
    s.time_us = t_us;
    for (int i = 0; i < 3; ++i)
    {
        s.acc[i] = acc[i];
        s.gyro[i] = gyro[i];
    }
    return s;
}

// A level state (body = NED) at t0, 20 m/s north and climbing at 100 m/s.
ShadowNav levelNav(uint32_t t0)
{
    ShadowNav n{};
    n.time_us = t0;
    n.lla[0] = 32.99 * kDeg;
    n.lla[1] = -106.97 * kDeg;
    n.lla[2] = 1400.0;
    n.vel_ned[0] = 20.0f;
    n.vel_ned[2] = -100.0f;
    n.quat[0] = 1.0f;
    n.running = 1;
    return n;
}

double northOf(double lat, double lat0, double h)
{
    // metres north at lat0 (TR_RawGnssEKF's earthRad)
    const double s = std::sin(lat0);
    const double d = 1.0 - 0.0066943799901 * s * s;
    return (lat - lat0) * (6378137.0 * (1.0 - 0.0066943799901) / (d * std::sqrt(d)) + h);
}

}  // namespace

TEST(RawNavAdapter, ItCarriesTheStateForwardThroughTheTicksSince)
{
    // The filter's state 40 ms behind the IMU; the ticks since, 2.5 ms apart:
    // the loop reads it at the latest tick. Level, 30 m/s^2 up (a specific
    // force of g + 30 up, less an accelerometer bias of 0.2): the velocity
    // gains 30 m/s^2 for 40 ms, the position follows it a tick at a time, as
    // the raw filter's steps do (velocity, then position from it).
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    const uint32_t t0 = 7000000;
    ShadowNav n = levelNav(t0);
    n.accel_bias[2] = -0.2f;
    const float f[3] = {0.0f, 0.0f, -9.807f - 30.0f - 0.2f};
    const float w[3] = {0.0f, 0.0f, 0.0f};
    for (uint32_t t = t0 - 10000; t <= t0; t += 2500)
        ekf.noteImu(tick(t, f, w));               // kept, not in it: the state has them
    link.nav.write(n);
    ekf.refresh(t0 + 1000);
    const int N = 16;
    for (int i = 1; i <= N; ++i)
        ekf.noteImu(tick(t0 + i * 2500, f, w));
    ekf.refresh(t0 + N * 2500 + 1000);
    ASSERT_TRUE(ekf.isHealthy());
    float v[3];
    ekf.getVelEst(v);
    EXPECT_NEAR(v[0], 20.0f, 1e-4f);
    EXPECT_NEAR(v[1], 0.0f, 1e-4f);
    EXPECT_NEAR(v[2], -100.0f - 30.0f * 0.04f, 2e-3f);
    double p[3];
    ekf.getPosEst(p);
    const double dt = 0.0025;
    const double climb = 100.0 * N * dt + 30.0 * dt * dt * N * (N + 1) / 2.0;   // a tick at a time
    EXPECT_NEAR(p[2] - 1400.0, climb, 1e-3);
    EXPECT_NEAR(northOf(p[0], n.lla[0], n.lla[2]), 20.0 * N * dt, 1e-3);
    EXPECT_EQ(ekf.takeMaxAgeUs(), 1000u);         // the loop read it a tick behind the IMU, at most
    EXPECT_EQ(ekf.takeMaxForwardUs(), 41000u);    //   brought 41 ms forward from the filter's
    EkfStateSnapshot snap;
    ekf.getState(snap);
    EXPECT_EQ(snap.t_prev_us, t0 + N * 2500);     // the snapshot is the state at its latest tick
}

TEST(RawNavAdapter, ItCarriesTheStateForwardAsTheRawFilterWould)
{
    // Through a boost -- 15 g along the nose, spinning at 6 rad/s, pitching
    // over at 0.2 rad/s, a gyro bias on every axis -- tick by tick: where the
    // raw filter's own propagation (TR_RawGnssEKF::propagate) takes its state
    // over the same ticks, the forward lands.
    const uint32_t t0 = 9000000;
    gnss_raw::RawGnssEKF<float> f{gnss_raw::RawGnssEKFParams{}};
    const double lla0[3] = {32.99 * kDeg, -106.97 * kDeg, 1800.0};
    const float v0[3] = {5.0f, -3.0f, -80.0f};
    const double pitch = 80.0 * kDeg, yaw = 30.0 * kDeg;
    const float q0[4] = {float(std::cos(yaw / 2) * std::cos(pitch / 2)), float(-std::sin(yaw / 2) * std::sin(pitch / 2)),
                         float(std::cos(yaw / 2) * std::sin(pitch / 2)), float(std::sin(yaw / 2) * std::cos(pitch / 2))};
    f.init(lla0, v0, q0, t0 * 1e-6);
    const float wb[3] = {0.002f, -0.001f, 0.003f};
    f.seedGyroBias(wb);
    ShadowNav n{};
    n.time_us = t0;
    for (int i = 0; i < 3; ++i)
    {
        n.lla[i] = f.lla()[i];
        n.vel_ned[i] = f.velNed()[i];
        n.gyro_bias[i] = f.gyroBias()[i];
        n.accel_bias[i] = f.accelBias()[i];
    }
    for (int i = 0; i < 4; ++i)
        n.quat[i] = f.quat()[i];
    n.running = 1;
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    link.nav.write(n);
    ekf.refresh(t0 + 500);
    const float acc[3] = {147.1f, 0.3f, -0.2f};
    const float gyro[3] = {6.0f, 0.2f, 0.0f};
    for (int i = 1; i <= 20; ++i)                 // 50 ms of ticks
    {
        f.propagate(acc, gyro, 0.0025f);
        ekf.noteImu(tick(t0 + i * 2500, acc, gyro));
    }
    ekf.refresh(t0 + 50500);
    double p[3];
    float v[3], q[4];
    ekf.getPosEst(p);
    ekf.getVelEst(v);
    ekf.getQuaternion(q);
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(v[i], f.velNed()[i], 2e-3f) << i;
    EXPECT_NEAR(northOf(p[0], f.lla()[0], f.lla()[2]), 0.0, 1e-3);
    EXPECT_NEAR(p[2], f.lla()[2], 1e-3);
    const double dlon_m = (p[1] - f.lla()[1]) * 6378137.0 * std::cos(f.lla()[0]);
    EXPECT_NEAR(dlon_m, 0.0, 1e-3);
    const float dot = std::fabs(q[0] * f.quat()[0] + q[1] * f.quat()[1] + q[2] * f.quat()[2] + q[3] * f.quat()[3]);
    EXPECT_GT(dot, 1.0f - 1e-6f);                 // the same attitude
    // and it moved: 50 ms of 15 g and a third of a turn of spin
    EXPECT_GT(std::fabs(v[2] - v0[2]), 5.0f);
}

TEST(RawNavAdapter, ANewStateFromTheFilterStartsTheForwardOver)
{
    // Each state the filter publishes already holds the ticks up to its own:
    // the forward starts from it, through only the ticks after, none twice.
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    const uint32_t t0 = 3000000;
    const float f[3] = {0.0f, 0.0f, -9.807f - 10.0f};   // 10 m/s^2 up
    const float w[3] = {0.0f, 0.0f, 0.0f};
    link.nav.write(levelNav(t0));
    ekf.refresh(t0);
    for (int i = 1; i <= 8; ++i)
        ekf.noteImu(tick(t0 + i * 2500, f, w));
    ShadowNav n1 = levelNav(t0 + 10000);         // the filter at its next step, with a velocity of its own
    n1.vel_ned[2] = -150.0f;
    link.nav.write(n1);
    ekf.refresh(t0 + 20500);
    float v[3];
    ekf.getVelEst(v);
    EXPECT_NEAR(v[2], -150.0f - 10.0f * 0.01f, 1e-3f);   // its own, plus the 4 ticks after it
    double p[3];
    ekf.getPosEst(p);
    EXPECT_NEAR(p[2] - 1400.0, 150.0 * 0.01 + 10.0 * 0.0025 * 0.0025 * 10.0, 1e-3);
}

TEST(RawNavAdapter, AStateOlderThanTheTicksKeptIsReadAsPublished)
{
    // A gap the forward would step over is not stepped over: the state is
    // read as the filter published it, and counted.
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    const uint32_t t0 = 3000000;
    const float f[3] = {0.0f, 0.0f, -9.807f - 10.0f};
    const float w[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 1; i <= RawNavAdapter::kTicks + 2; ++i)
        ekf.noteImu(tick(t0 + i * 1000, f, w));  // more ticks after it than are kept
    link.nav.write(levelNav(t0));
    ekf.refresh(t0 + (RawNavAdapter::kTicks + 2) * 1000);
    EXPECT_EQ(ekf.forwardShort(), 1u);
    float v[3];
    ekf.getVelEst(v);
    EXPECT_FLOAT_EQ(v[2], -100.0f);
    ekf.noteImu(tick(t0 + (RawNavAdapter::kTicks + 3) * 1000, f, w));
    ekf.getVelEst(v);
    EXPECT_FLOAT_EQ(v[2], -100.0f);              // nor carried on from it
}

TEST(RawNavAdapter, AFilterThatStopsSteppingIsNotHealthy)
{
    // #1626 N4: its task republishes every pass, so a new sequence alone says
    // nothing -- a state 0.2 s old is not healthy, however fresh the copy.
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    const uint32_t t0 = 4000000;
    for (uint32_t now = t0; now <= t0 + 300000; now += 4000)
    {
        link.nav.write(levelNav(t0));            // the same state, stepped no further
        ekf.refresh(now);
        EXPECT_EQ(ekf.isHealthy(), now - t0 <= RawNavAdapter::kMaxStaleUs) << (now - t0);
    }
}
