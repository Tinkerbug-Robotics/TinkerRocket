// test_raw_nav_adapter_mosaic.cpp — what the mosaic-only image adds to
// GpsInsEKF's face on the raw-GNSS filter (raw_nav_adapter.h under
// TR_RAW_NAV_MOSAIC, which this target defines): the geoid separation the
// snapshot carries, whether the filter has run, a restore's start height
// rebased to the barometer and its accelerometer bias, and a stop in flight
// held over as the last good state, carried forward and restarted from. The
// rest of the adapter is test_raw_nav_adapter.cpp's, which also runs against
// this build of it (test_raw_nav_adapter_mosaic_base): with the hold off it
// answers exactly as the Beetle's.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "TR_RawGnssEKF.h"
#include "raw_nav_adapter.h"

#if !TR_RAW_NAV_MOSAIC
#error "this target builds the adapter as the mosaic-only image does (TR_RAW_NAV_MOSAIC=1)"
#endif

using gnss_raw::ShadowImu;
using gnss_raw::ShadowLink;
using gnss_raw::ShadowNav;
using gnss_raw::ShadowSeed;

namespace
{

constexpr float kSep = 21.5f;   // a synthetic geoid separation, m

// A synthetic flight state (no real place): level, nose north, climbing at
// 60 m/s; in flight unless said otherwise.
ShadowNav flying(uint32_t t_us, double h_ell, bool in_flight = true)
{
    ShadowNav n{};
    n.time_us = t_us;
    n.lla[0] = 0.30;
    n.lla[1] = 0.60;
    n.lla[2] = h_ell;
    n.vel_ned[0] = 5.0f;
    n.vel_ned[2] = -60.0f;
    n.quat[0] = 1.0f;
    for (int i = 0; i < 15; ++i)
        n.var[i] = 0.02f * float(i + 1);
    n.accel_bias[0] = 0.11f;
    n.accel_bias[1] = -0.07f;
    n.accel_bias[2] = 0.05f;
    n.gyro_bias[2] = 0.002f;
    n.running = 1;
    n.gnss_aided = 1;
    n.in_flight = in_flight ? 1 : 0;
    return n;
}

// One IMU tick: the specific force (body FRD) of a level body accelerating
// north at `a_n` (gravity's -g on z, and the bias the filter publishes, so
// the forward takes it back out).
ShadowImu tick(uint32_t t_us, float a_n)
{
    ShadowImu s{};
    s.time_us = t_us;
    s.acc[0] = a_n + 0.11f;
    s.acc[1] = -0.07f;
    s.acc[2] = -9.807f + 0.05f;
    s.gyro[2] = 0.002f;
    return s;
}

// A filter that ran in flight and then stopped: its last good state at 1 s,
// ticks to 1.1 s, then the stop's publish.
void runThenStop(ShadowLink& link, RawNavAdapter& ekf, float a_n = 0.0f)
{
    for (uint32_t t = 900000; t <= 1000000; t += 2500)
        ekf.noteImu(tick(t, a_n));
    link.nav.write(flying(1000000, 500.0));
    ekf.refresh(1000000);
    ASSERT_TRUE(ekf.isHealthy());
    for (uint32_t t = 1002500; t <= 1100000; t += 2500)
        ekf.noteImu(tick(t, a_n));
    link.nav.write(ShadowNav{});   // stopped: running 0
    ekf.refresh(1100000);
}

}  // namespace

TEST(RawNavAdapterMosaic, TheSnapshotsGeoidSeparationGivesThePreRebootGeoidHeight)
{
    ShadowLink link;
    RawNavAdapter before;
    before.attach(&link);
    before.setGeoidSeparation(kSep);
    link.nav.write(flying(1000000, 612.5));
    before.refresh(1000000);
    EXPECT_FLOAT_EQ(before.geoidSeparation(), kSep);
    double p_before[3];
    before.getPosEst(p_before);
    EkfStateSnapshot snap;
    before.getState(snap);

    // rebooted: the receiver cold, or left out after a crash -- no separation
    // but the snapshot's (main.cpp: FlightSnapshotData::reserved_v5)
    ShadowLink link2;
    RawNavAdapter after;
    after.attach(&link2);
    EXPECT_FLOAT_EQ(after.geoidSeparation(), 0.0f);
    after.setGeoidSeparation(before.geoidSeparation());
    after.setState(snap);
    double p_after[3];
    after.getPosEst(p_after);
    EXPECT_NEAR(p_after[2], p_before[2], 1e-4);   // 612.5 - 21.5 above the geoid
    EXPECT_NEAR(p_after[2], 612.5 - kSep, 1e-4);
    // without it the height read would be the ellipsoid's
    RawNavAdapter bare;
    bare.setState(snap);
    double p_bare[3];
    bare.getPosEst(p_bare);
    EXPECT_NEAR(p_bare[2], 612.5, 1e-4);
}

TEST(RawNavAdapterMosaic, ARestoresStartHeightIsRebasedWhileItWaits)
{
    ShadowLink link;
    RawNavAdapter before;
    before.attach(&link);
    before.setGeoidSeparation(kSep);
    link.nav.write(flying(1000000, 612.5));
    before.refresh(1000000);
    EkfStateSnapshot snap;
    before.getState(snap);

    RawNavAdapter after;
    ShadowLink link2;
    after.attach(&link2);
    after.setGeoidSeparation(kSep);
    after.setState(snap);
    after.rebaseRestoreHeight(880.0);   // the pad's datum + the barometer's height above it, MSL
    ShadowSeed s{};
    after.startSeed(s);
    EXPECT_EQ(s.restore, 1);
    EXPECT_NEAR(s.lla[2], 880.0 + kSep, 1e-6);   // to the ellipsoid
    EXPECT_DOUBLE_EQ(s.lla[0], snap.pos_rrm[0]);  // only the height moves
    double p[3];
    after.getPosEst(p);
    EXPECT_NEAR(p[2], 880.0, 1e-6);

    // once it runs the restore is done, and a rebase moves nothing
    link2.nav.write(flying(2000000, 900.0));
    after.refresh(2000000);
    ASSERT_FALSE(after.restorePending());
    after.rebaseRestoreHeight(10.0);
    after.startSeed(s);
    EXPECT_NEAR(s.lla[2], 880.0 + kSep, 1e-6);

    // nor on the pad's start
    RawNavAdapter pad;
    EkfGNSSDataLLA g;
    g.alt_m = 40.0;
    pad.init(EkfIMUData(), g, EkfMagData());
    pad.rebaseRestoreHeight(500.0);
    pad.startSeed(s);
    EXPECT_NEAR(s.lla[2], 40.0, 1e-6);
}

TEST(RawNavAdapterMosaic, TheAccelerometerBiasReachesTheRestoresSeed)
{
    EkfStateSnapshot snap{};
    snap.quat[0] = 1.0f;
    snap.accel_bias[0] = 0.2f;
    snap.accel_bias[1] = -0.1f;
    snap.accel_bias[2] = 0.3f;
    RawNavAdapter ekf;
    ekf.setState(snap);
    ShadowSeed s{};
    ekf.startSeed(s);
    EXPECT_EQ(s.has_accel_bias, 1);
    EXPECT_FLOAT_EQ(s.accel_bias[0], 0.2f);
    EXPECT_FLOAT_EQ(s.accel_bias[1], -0.1f);
    EXPECT_FLOAT_EQ(s.accel_bias[2], 0.3f);

    // the pad's start carries none: the filter learns its own there
    ekf.init(EkfIMUData(), EkfGNSSDataLLA(), EkfMagData());
    ShadowSeed p{};
    ekf.startSeed(p);
    EXPECT_EQ(p.restore, 0);
    EXPECT_EQ(p.has_accel_bias, 0);
    EXPECT_FLOAT_EQ(p.accel_bias[0], 0.0f);
}

TEST(RawNavAdapterMosaic, HasRunIsFalseUntilARunningStateIsPublished)
{
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    EkfStateSnapshot snap{};
    snap.quat[0] = 1.0f;
    snap.vel_ned_mps[2] = -280.0f;   // a restore's start: fast, and no measurement of anything
    ekf.setState(snap);
    ekf.refresh(1000);
    EXPECT_FALSE(ekf.hasRun());
    link.nav.write(ShadowNav{});      // the filter's task says it waits
    ekf.refresh(2000);
    EXPECT_FALSE(ekf.hasRun());
    link.nav.write(flying(3000, 100.0));
    ekf.refresh(3000);
    EXPECT_TRUE(ekf.hasRun());
    link.nav.write(ShadowNav{});      // and a stop later does not take it back
    ekf.refresh(4000);
    EXPECT_TRUE(ekf.hasRun());
}

TEST(RawNavAdapterMosaic, TheStatesOwnTimeIsTheTickItWasCarriedTo)
{
    // The rig replay's [REPLAY-NAV] judges the state the loop reads against
    // the truth at this instant, not at the loop's: the loop's clock is a
    // pass on from the last EKF tick every other pass, and the IMU window's
    // stamp is its middle (main.cpp)
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    uint32_t t_us = 0;
    EXPECT_FALSE(ekf.stateTimeUs(t_us));   // before the filter runs: no state of its own
    for (uint32_t t = 990000; t <= 1000000; t += 2500)
        ekf.noteImu(tick(t, 2.0f));
    link.nav.write(flying(1000000, 500.0, false));
    ekf.refresh(1000000);
    ASSERT_TRUE(ekf.stateTimeUs(t_us));
    EXPECT_EQ(t_us, 1000000u);   // as published
    ekf.noteImu(tick(1002500, 2.0f));
    ekf.noteImu(tick(1005000, 2.0f));
    ASSERT_TRUE(ekf.stateTimeUs(t_us));
    EXPECT_EQ(t_us, 1005000u);   // carried forward through the ticks since
    float v[3];
    ekf.getVelEst(v);
    EXPECT_NEAR(v[0], 5.0f + 2.0f * 0.005f, 2e-3f);   // the velocity at that tick, not at the loop's now
    // a stop on the ground (no hold): no state of its own again
    link.nav.write(ShadowNav{});
    ekf.refresh(1010000);
    EXPECT_FALSE(ekf.stateTimeUs(t_us));
}

TEST(RawNavAdapterMosaic, WithTheHoldOnAStopInFlightKeepsTheLastGoodStateGoing)
{
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    ekf.setHoldLastGood(true);
    EkfGNSSDataLLA pad;
    pad.alt_m = 3.0;                  // the pad's start: nothing like the flight
    ekf.init(EkfIMUData(), pad, EkfMagData());
    runThenStop(link, ekf, 2.0f);     // accelerating north at 2 m/s^2

    EXPECT_TRUE(ekf.stoppedInFlight());
    EXPECT_FALSE(ekf.isHealthy());
    EXPECT_FALSE(ekf.gnssAided());
    float v[3];
    ekf.getVelEst(v);
    EXPECT_NEAR(v[0], 5.0f + 2.0f * 0.1f, 1e-3f);   // forward to 1.1 s
    EXPECT_NEAR(v[2], -60.0f, 1e-3f);
    double p[3];
    ekf.getPosEst(p);
    EXPECT_NEAR(p[2], 500.0 + 60.0 * 0.1, 1e-2);

    // and on through the ticks after, as a free INS
    for (uint32_t t = 1102500; t <= 1600000; t += 2500)
        ekf.noteImu(tick(t, 2.0f));
    link.nav.write(ShadowNav{});      // the stopped filter keeps saying so
    ekf.refresh(1600000);
    EXPECT_TRUE(ekf.stoppedInFlight());
    EXPECT_FALSE(ekf.isHealthy());
    ekf.getVelEst(v);
    EXPECT_NEAR(v[0], 5.0f + 2.0f * 0.6f, 2e-3f);
    ekf.getPosEst(p);
    EXPECT_NEAR(p[2], 500.0 + 60.0 * 0.6, 2e-2);

    // the snapshot holds the flight, not the pad
    EkfStateSnapshot snap;
    ekf.getState(snap);
    EXPECT_NEAR(snap.pos_rrm[2], 536.0, 2e-2);
    EXPECT_NEAR(snap.vel_ned_mps[2], -60.0f, 1e-3f);
    EXPECT_FLOAT_EQ(snap.accel_bias[0], 0.11f);
}

TEST(RawNavAdapterMosaic, WithTheHoldOffAStopReadsAsTheBeetlesDoes)
{
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    EkfGNSSDataLLA pad;
    pad.alt_m = 3.0;
    pad.vel_d_mps = 0.0f;
    ekf.init(EkfIMUData(), pad, EkfMagData());
    runThenStop(link, ekf);
    EXPECT_FALSE(ekf.stoppedInFlight());
    EXPECT_FALSE(ekf.isHealthy());
    float v[3];
    ekf.getVelEst(v);
    EXPECT_FLOAT_EQ(v[2], 0.0f);      // the start's
    double p[3];
    ekf.getPosEst(p);
    EXPECT_NEAR(p[2], 3.0, 1e-9);
}

TEST(RawNavAdapterMosaic, OnTheGroundAStopIsNotHeld)
{
    // a new start on the pad stops the filter for a moment: not a flight's
    // state to carry on
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    ekf.setHoldLastGood(true);
    EkfGNSSDataLLA pad;
    pad.alt_m = 3.0;
    ekf.init(EkfIMUData(), pad, EkfMagData());
    link.nav.write(flying(1000000, 500.0, false));
    ekf.refresh(1000000);
    link.nav.write(ShadowNav{});
    ekf.refresh(1010000);
    EXPECT_FALSE(ekf.stoppedInFlight());
    double p[3];
    ekf.getPosEst(p);
    EXPECT_NEAR(p[2], 3.0, 1e-9);
}

TEST(RawNavAdapterMosaic, ItRestartsFromTheLastGoodStateThroughTheRestore)
{
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    ekf.setHoldLastGood(true);
    ekf.setGeoidSeparation(kSep);
    runThenStop(link, ekf);
    ASSERT_TRUE(ekf.stoppedInFlight());
    ASSERT_FALSE(ekf.restorePending());

    EXPECT_TRUE(ekf.restartFromLastGood());
    EXPECT_TRUE(ekf.restorePending());
    EXPECT_TRUE(ekf.stoppedInFlight());   // held until the filter runs
    ShadowSeed s{};
    ekf.startSeed(s);
    EXPECT_EQ(s.restore, 1);
    EXPECT_NEAR(s.lla[2], 500.0 + 60.0 * 0.1, 1e-2);   // the held state, the filter's own height
    EXPECT_NEAR(s.vel_ned[2], -60.0f, 1e-3f);
    EXPECT_FLOAT_EQ(s.quat[0], 1.0f);
    EXPECT_EQ(s.has_accel_bias, 1);
    EXPECT_FLOAT_EQ(s.accel_bias[0], 0.11f);
    EXPECT_FLOAT_EQ(s.gyro_bias[2], 0.002f);

    // the filter runs again: its own state from here
    link.nav.write(flying(1200000, 510.0));
    ekf.refresh(1200000);
    EXPECT_FALSE(ekf.stoppedInFlight());
    EXPECT_FALSE(ekf.restorePending());
    EXPECT_TRUE(ekf.isHealthy());
    double p[3];
    ekf.getPosEst(p);
    EXPECT_NEAR(p[2], 510.0 - kSep, 1e-6);
}

TEST(RawNavAdapterMosaic, ANewStartOnThePadDropsAHeldState)
{
    // a sim's reset after a stop in its flight: the start's state, not the held one
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    ekf.setHoldLastGood(true);
    runThenStop(link, ekf);
    ASSERT_TRUE(ekf.stoppedInFlight());
    EkfGNSSDataLLA pad;
    pad.alt_m = 7.0;
    ekf.init(EkfIMUData(), pad, EkfMagData());
    EXPECT_FALSE(ekf.stoppedInFlight());
    double p[3];
    ekf.getPosEst(p);
    EXPECT_NEAR(p[2], 7.0, 1e-9);
    float v[3];
    ekf.getVelEst(v);
    EXPECT_FLOAT_EQ(v[2], 0.0f);
}

TEST(RawNavAdapterMosaic, AStateTheTicksNoLongerReachIsHeldFromTheNewestTick)
{
    // a state older than every tick kept is read as published; held over a
    // stop it is carried on from the newest tick
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    ekf.setHoldLastGood(true);
    for (int k = 0; k < RawNavAdapter::kTicks; ++k)
        ekf.noteImu(tick(2000000 + 2500 * (uint32_t)k, 1.0f));
    link.nav.write(flying(1000000, 500.0));   // a second older than the oldest tick
    ekf.refresh(2400000);
    EXPECT_EQ(ekf.forwardShort(), 1u);
    link.nav.write(ShadowNav{});
    ekf.refresh(2400000);
    ASSERT_TRUE(ekf.stoppedInFlight());
    const uint32_t t0 = 2000000 + 2500 * (uint32_t)(RawNavAdapter::kTicks - 1);
    for (uint32_t t = t0 + 2500; t <= t0 + 100000; t += 2500)
        ekf.noteImu(tick(t, 1.0f));
    float v[3];
    ekf.getVelEst(v);
    EXPECT_NEAR(v[0], 5.0f + 1.0f * 0.1f, 1e-3f);
}

TEST(RawNavAdapterMosaic, WithTheHoldOnTheGroundReadsExactlyAsWithoutIt)
{
    // the mosaic image turns the hold on at the filter's start, so on the pad
    // (and after landing) its refresh must answer as the Beetle's does: the
    // same sequence of states, stops and ticks, read through both
    ShadowLink la, lb;
    RawNavAdapter a, b;
    a.attach(&la);
    b.attach(&lb);
    a.setHoldLastGood(true);
    EkfGNSSDataLLA pad;
    pad.alt_m = 3.0;
    a.init(EkfIMUData(), pad, EkfMagData());
    b.init(EkfIMUData(), pad, EkfMagData());
    a.setGeoidSeparation(kSep);
    b.setGeoidSeparation(kSep);
    uint32_t t = 1000000;
    for (int step = 0; step < 400; ++step, t += 2500)
    {
        a.noteImu(tick(t, 0.3f));
        b.noteImu(tick(t, 0.3f));
        if (step % 4 == 0)
        {
            // running on the ground; a stop now and then (a new start on the pad)
            const ShadowNav n = (step % 100 == 40) ? ShadowNav{} : flying(t - 5000, 40.0 + 0.001 * step, false);
            la.nav.write(n);
            lb.nav.write(n);
        }
        a.refresh(t + 300);
        b.refresh(t + 300);
        ASSERT_EQ(a.isHealthy(), b.isHealthy()) << step;
        ASSERT_EQ(a.gnssAided(), b.gnssAided()) << step;
        ASSERT_FALSE(a.stoppedInFlight());
        double pa[3], pb[3];
        a.getPosEst(pa);
        b.getPosEst(pb);
        float va[3], vb[3], qa[4], qb[4];
        a.getVelEst(va);
        b.getVelEst(vb);
        a.getQuaternion(qa);
        b.getQuaternion(qb);
        for (int i = 0; i < 3; ++i)
        {
            ASSERT_EQ(pa[i], pb[i]) << step;
            ASSERT_EQ(va[i], vb[i]) << step;
        }
        for (int i = 0; i < 4; ++i)
            ASSERT_EQ(qa[i], qb[i]) << step;
    }
    EXPECT_EQ(a.takeMaxAgeUs(), b.takeMaxAgeUs());
    EXPECT_EQ(a.takeMaxForwardUs(), b.takeMaxForwardUs());
    EXPECT_EQ(a.forwardShort(), b.forwardShort());
}

TEST(RawNavAdapterMosaic, ALastStateThatIsNotFiniteIsNotHeld)
{
    // a running state whose attitude went non-finite (the raw filter's own
    // stop check reads only the position and velocity): no good state to
    // carry on or restart from, so the stop reads as the Beetle's does
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    ekf.setHoldLastGood(true);
    EkfGNSSDataLLA pad;
    pad.alt_m = 3.0;
    ekf.init(EkfIMUData(), pad, EkfMagData());
    for (uint32_t t = 900000; t <= 1000000; t += 2500)
        ekf.noteImu(tick(t, 0.0f));
    ShadowNav bad = flying(1000000, 500.0);
    bad.quat[0] = std::numeric_limits<float>::quiet_NaN();
    link.nav.write(bad);
    ekf.refresh(1000000);
    EXPECT_FALSE(ekf.isHealthy());
    for (uint32_t t = 1002500; t <= 1100000; t += 2500)
        ekf.noteImu(tick(t, 0.0f));
    link.nav.write(ShadowNav{});
    ekf.refresh(1100000);
    EXPECT_FALSE(ekf.stoppedInFlight());
    float v[3];
    ekf.getVelEst(v);
    EXPECT_FLOAT_EQ(v[2], 0.0f);       // the start's
    double p[3];
    ekf.getPosEst(p);
    EXPECT_NEAR(p[2], 3.0, 1e-9);
    // a non-finite bias is no good state either
    ShadowLink link2;
    RawNavAdapter b2;
    b2.attach(&link2);
    b2.setHoldLastGood(true);
    for (uint32_t t = 900000; t <= 1000000; t += 2500)
        b2.noteImu(tick(t, 0.0f));
    ShadowNav nb = flying(1000000, 500.0);
    nb.gyro_bias[1] = std::numeric_limits<float>::infinity();
    link2.nav.write(nb);
    b2.refresh(1000000);
    link2.nav.write(ShadowNav{});
    b2.refresh(1010000);
    EXPECT_FALSE(b2.stoppedInFlight());
}

TEST(RawNavAdapterMosaic, AHeldStateTheForwardSpoilsIsNeitherRestartedNorKept)
{
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    ekf.setHoldLastGood(true);
    EkfGNSSDataLLA pad;
    pad.alt_m = 3.0;
    ekf.init(EkfIMUData(), pad, EkfMagData());
    runThenStop(link, ekf);
    ASSERT_TRUE(ekf.stoppedInFlight());
    // a tick that is not finite carries the held state off
    ShadowImu k = tick(1102500, 0.0f);
    k.acc[0] = std::numeric_limits<float>::quiet_NaN();
    ekf.noteImu(k);
    EXPECT_FALSE(ekf.restartFromLastGood());   // no seed from it
    EXPECT_FALSE(ekf.restorePending());
    ShadowSeed s{};
    ekf.startSeed(s);
    EXPECT_EQ(s.restore, 0);
    EXPECT_NEAR(s.lla[2], 3.0, 1e-9);          // the start's, untouched
    // and the next stop is read in: nothing held
    link.nav.write(ShadowNav{});
    ekf.refresh(1110000);
    EXPECT_FALSE(ekf.stoppedInFlight());
    double p[3];
    ekf.getPosEst(p);
    EXPECT_NEAR(p[2], 3.0, 1e-9);
}

TEST(RawNavAdapterMosaic, ASnapshotDuringTheRestoresWaitKeepsTheRestoredBiases)
{
    // the restore waits for the barometer (up to 3 s), or a crash-loop boot
    // never starts the filter: the snapshots the OC keeps meanwhile carry the
    // biases the reboot restored, not the zeros a filter not running publishes
    EkfStateSnapshot snap{};
    snap.quat[0] = 1.0f;
    snap.pos_rrm[2] = 700.0;
    snap.vel_ned_mps[2] = -40.0f;
    snap.accel_bias[0] = 0.21f;
    snap.accel_bias[2] = -0.13f;
    snap.gyro_bias[1] = 0.004f;
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    ekf.setHoldLastGood(true);
    ekf.setState(snap);
    link.nav.write(ShadowNav{});   // the filter's task: waiting
    ekf.refresh(1000);
    EkfStateSnapshot out;
    ekf.getState(out);
    EXPECT_FLOAT_EQ(out.accel_bias[0], 0.21f);
    EXPECT_FLOAT_EQ(out.accel_bias[2], -0.13f);
    EXPECT_FLOAT_EQ(out.gyro_bias[1], 0.004f);
    EXPECT_NEAR(out.pos_rrm[2], 700.0, 1e-9);
    EXPECT_FLOAT_EQ(out.vel_ned_mps[2], -40.0f);
    // once it runs, its own
    link.nav.write(flying(2000, 690.0));
    ekf.refresh(2000);
    ekf.getState(out);
    EXPECT_FLOAT_EQ(out.accel_bias[0], 0.11f);
    EXPECT_FLOAT_EQ(out.gyro_bias[1], 0.0f);
}
