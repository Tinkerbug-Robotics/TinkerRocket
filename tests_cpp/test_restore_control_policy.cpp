// #1769: roll control after an FC reboot in flight
// (flight_computer/main/restore_control_policy.h and main.cpp's restore hold).
//
// The #1176 restore restarts the navigation filter from the OC's snapshot:
// its velocity just before the reset. After a reset in the boost that speed is
// short by everything the motor added in the gap, and once the 500 ms settle
// ended and the filter answered healthy, the gain schedule ran on it: up to
// 26x the gain at 100-200 m/s. Now the fins hold neutral past the settle
// until the filter's velocity is trusted again (healthy, every axis's
// variance under the EKF row's EKF_VEL_VAR_OK), and angle mode stays off for
// the rest of a restored flight.
//
// Four parts:
//   - the policy's tables;
//   - the real RawNavAdapter, built as the V10's raw images build it (its
//     hold over a stop in flight included), and the real GpsInsEKF after a
//     restore, as main.cpp makes it;
//   - a restored flight with the real TR_ServoControl and RawNavAdapter, each
//     pass written as main.cpp runs it: the hold's release, then the roll
//     chain (the hold, else the gain schedule on the filter's speed, else the
//     pure gyro). The filter is scripted from the review's harness: 80 m/s
//     restored against 200 true, the gap's 120 m/s carried until GNSS is back,
//     then the error and the variance falling together. That coupling is the
//     script's, so these runs check the chain's use of the policy (what is
//     held, when it releases, which law runs on which speed), not the filter;
//     tests/unit/test_restore_roll_hold_wiring.py ties the chain to main.cpp;
//   - the gain the schedule ran on, against the one the true speed gives,
//     measured on the real servo controller.
// A reboot in the boost through the real raw filter, whose variance and
// error are its own, is in test_raw_gnss_shadow.cpp
// (TheRestoreHoldWaitsForATrustedSpeed, TheRestoreHoldHoldsOnTheBarometerAlone).

#include <gtest/gtest.h>
#include <Arduino.h>   // host shim: setMockMicros()

#include <cmath>
#include <cstdint>
#include <vector>

#include "TR_GpsInsEKF.h"
#include "TR_RawGnssEKF.h"
#include "TR_ServoControl_ledc_mult.h"
#include "raw_nav_adapter.h"
#include "restore_control_policy.h"

using gnss_raw::ShadowLink;
using gnss_raw::ShadowNav;

namespace {

constexpr float kVarOk = 4.0f;           // config::EKF_VEL_VAR_OK
constexpr uint32_t kSettleMs = 500;      // main.cpp: servo_settle_end_ms = now_ms + 500
constexpr float kVRef = 50.0f;           // config::GAIN_SCHEDULE_V_REF
constexpr float kVMin = 25.0f;           // config::GAIN_SCHEDULE_V_MIN
constexpr float kRestoreVelSigma = 20.0f;   // config::RAW_RESTORE_VEL_SIGMA_MPS

// ---------------------------------------------------------------------------
// The policy's tables
// ---------------------------------------------------------------------------

TEST(RestoreControlPolicy, TheVelocityIsTrustedWhenHealthyAndEveryAxisIsUnderTheBound)
{
    const float ok[3] = {1.0f, 2.0f, 3.9f};
    EXPECT_TRUE(RestoreControlPolicy::velocityTrusted(true, ok, kVarOk));
    EXPECT_FALSE(RestoreControlPolicy::velocityTrusted(false, ok, kVarOk)) << "not healthy";
    for (int axis = 0; axis < 3; ++axis)
    {
        float v[3] = {1.0f, 1.0f, 1.0f};
        v[axis] = kVarOk;   // the bound itself is not under it
        EXPECT_FALSE(RestoreControlPolicy::velocityTrusted(true, v, kVarOk)) << axis;
        v[axis] = kRestoreVelSigma * kRestoreVelSigma;   // the raw filter's restore
        EXPECT_FALSE(RestoreControlPolicy::velocityTrusted(true, v, kVarOk)) << axis;
        v[axis] = std::nanf("");
        EXPECT_FALSE(RestoreControlPolicy::velocityTrusted(true, v, kVarOk)) << axis;
        v[axis] = INFINITY;
        EXPECT_FALSE(RestoreControlPolicy::velocityTrusted(true, v, kVarOk)) << axis;
        v[axis] = -1.0f;
        EXPECT_FALSE(RestoreControlPolicy::velocityTrusted(true, v, kVarOk)) << axis;
    }
}

TEST(RestoreControlPolicy, OnARestoredFlightOnlyTheRestoredFilterIsUsedForControl)
{
    // a flight that was never restored: as before (started and healthy)
    EXPECT_TRUE(RestoreControlPolicy::filterUsableForControl(true, true, false, false));
    EXPECT_FALSE(RestoreControlPolicy::filterUsableForControl(false, true, false, false));
    EXPECT_FALSE(RestoreControlPolicy::filterUsableForControl(true, false, false, false));
    // a restore that brought a filter back: that filter, healthy
    EXPECT_TRUE(RestoreControlPolicy::filterUsableForControl(true, true, true, true));
    EXPECT_FALSE(RestoreControlPolicy::filterUsableForControl(true, false, true, true));
    // a restore that found none: never a filter started in flight after it
    EXPECT_FALSE(RestoreControlPolicy::filterUsableForControl(true, true, true, false));
}

TEST(RestoreControlPolicy, TheHoldLastsTheSettleThenWhileARestoredFilterIsUntrusted)
{
    // the settle: held whatever the filter says
    for (int init = 0; init < 2; ++init)
        for (int trusted = 0; trusted < 2; ++trusted)
            EXPECT_TRUE(RestoreControlPolicy::holdFins(499, 500, init != 0, trusted != 0)) << init << trusted;
    // past it: a restored filter holds until its velocity is trusted
    EXPECT_TRUE(RestoreControlPolicy::holdFins(500, 500, true, false));
    EXPECT_FALSE(RestoreControlPolicy::holdFins(500, 500, true, true));
    // a restore that found no filter has no speed to be stale: released, as before
    EXPECT_FALSE(RestoreControlPolicy::holdFins(500, 500, false, false));
}

TEST(RestoreControlPolicy, TheSettleIsWrapSafe)
{
    const uint32_t end = 0x00000100u;   // armed just before the wrap, ends past it
    EXPECT_TRUE(RestoreControlPolicy::holdFins(0xFFFFFF00u, end, false, true));
    EXPECT_FALSE(RestoreControlPolicy::holdFins(end, end, false, true));
}

TEST(RestoreControlPolicy, AngleModeNeverRunsOnARestoredFlight)
{
    EXPECT_TRUE(RestoreControlPolicy::angleModeAllowed(true, true, false));
    EXPECT_FALSE(RestoreControlPolicy::angleModeAllowed(true, true, true));
    EXPECT_FALSE(RestoreControlPolicy::angleModeAllowed(false, true, false));
    EXPECT_FALSE(RestoreControlPolicy::angleModeAllowed(true, false, false));
}

// ---------------------------------------------------------------------------
// The filters as main.cpp reads them after a restore
// ---------------------------------------------------------------------------

constexpr double kDeg = 3.14159265358979323846 / 180.0;

ShadowNav navState(uint32_t t_us, float vel_d, float var_vel)
{
    ShadowNav n{};
    n.time_us = t_us;
    n.lla[0] = 32.99 * kDeg;
    n.lla[1] = -106.97 * kDeg;
    n.lla[2] = 3000.0;
    n.vel_ned[2] = vel_d;
    n.quat[0] = 1.0f;
    for (int i = 0; i < 15; ++i)
        n.var[i] = 0.01f;
    for (int i = 3; i < 6; ++i)
        n.var[i] = var_vel;
    n.running = 1;
    n.in_flight = 1;
    return n;
}

bool adapterTrusted(const RawNavAdapter& ekf)
{
    float v[3];
    ekf.getCovVel(v);
    return RestoreControlPolicy::velocityTrusted(ekf.isHealthy(), v, kVarOk);
}

TEST(RestoreControlPolicy, TheRawFiltersVelocityIsTrustedOnlyOnceItConverges)
{
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);

    // before its first state (the restore waits up to 3 s for the barometer):
    // the variance reads zero, but the filter is not running
    ekf.refresh(10000);
    float v[3];
    ekf.getCovVel(v);
    EXPECT_EQ(v[2], 0.0f);
    EXPECT_FALSE(ekf.isHealthy());
    EXPECT_FALSE(adapterTrusted(ekf));

    // the restore's first state: healthy, its velocity 20 m/s unsure
    link.nav.write(navState(100000, -80.0f, kRestoreVelSigma * kRestoreVelSigma));
    ekf.refresh(100000);
    EXPECT_TRUE(ekf.isHealthy());
    EXPECT_FALSE(adapterTrusted(ekf));

    // GNSS's velocity back: under the bound on every axis
    link.nav.write(navState(200000, -170.0f, 1.0f));
    ekf.refresh(200000);
    EXPECT_TRUE(adapterTrusted(ekf));

    // one axis at the bound: not yet
    ShadowNav n = navState(300000, -170.0f, 1.0f);
    n.var[4] = kVarOk;
    link.nav.write(n);
    ekf.refresh(300000);
    EXPECT_TRUE(ekf.isHealthy());
    EXPECT_FALSE(adapterTrusted(ekf));

    // a filter that stopped publishing is not healthy, however small its variance
    link.nav.write(navState(400000, -170.0f, 1.0f));
    ekf.refresh(400000);
    EXPECT_TRUE(adapterTrusted(ekf));
    ekf.refresh(400000 + RawNavAdapter::kMaxStaleUs + 1000);
    EXPECT_FALSE(adapterTrusted(ekf));
}

TEST(RestoreControlPolicy, ARawFilterHeldOverAStopInFlightIsNotTrusted)
{
    // The V10's raw images hold the last good state over a stop in flight
    // (setHoldLastGood): getCovVel then still answers that state's small
    // variance, but the filter is not running, so not healthy.
    ShadowLink link;
    RawNavAdapter ekf;
    ekf.attach(&link);
    ekf.setHoldLastGood(true);
    link.nav.write(navState(100000, -170.0f, 1.0f));
    ekf.refresh(100000);
    ASSERT_TRUE(adapterTrusted(ekf));
    ShadowNav stopped = navState(110000, -170.0f, 1.0f);
    stopped.running = 0;
    link.nav.write(stopped);
    ekf.refresh(110000);
    ASSERT_TRUE(ekf.stoppedInFlight());
    float v[3];
    ekf.getCovVel(v);
    EXPECT_LT(v[2], kVarOk);
    EXPECT_FALSE(ekf.isHealthy());
    EXPECT_FALSE(adapterTrusted(ekf));
    // restarted from the held state (restartFromLastGood): the restore's 400 again
    link.nav.write(navState(200000, -165.0f, kRestoreVelSigma * kRestoreVelSigma));
    ekf.refresh(200000);
    EXPECT_FALSE(ekf.stoppedInFlight());
    EXPECT_FALSE(adapterTrusted(ekf));
}

TEST(RestoreControlPolicy, GpsInsEkfsRestoreStartsUntrusted)
{
    // main.cpp's restore: the snapshot's state, the covariance from its
    // diagonal, then 25 (m/s)^2 more on each velocity axis
    GpsInsEKF ekf;
    EkfStateSnapshot s{};
    s.pos_rrm[0] = 32.99 * kDeg;
    s.pos_rrm[1] = -106.97 * kDeg;
    s.pos_rrm[2] = 3000.0;
    s.vel_ned_mps[2] = -80.0f;
    s.quat[0] = 1.0f;
    ekf.setState(s);
    float diag[15];
    for (int i = 0; i < 15; ++i)
        diag[i] = 0.05f;   // a converged filter's, as the snapshot carries it
    ekf.setCovFromDiag(diag);
    float v[3];
    ekf.getCovVel(v);
    EXPECT_TRUE(RestoreControlPolicy::velocityTrusted(ekf.isHealthy(), v, kVarOk)) << "the snapshot's alone";
    for (int i = 3; i < 6; ++i)
        ekf.inflateCovDiag(i, 25.0f);
    ekf.getCovVel(v);
    EXPECT_TRUE(ekf.isHealthy());
    EXPECT_GE(v[0], 25.0f);
    EXPECT_FALSE(RestoreControlPolicy::velocityTrusted(ekf.isHealthy(), v, kVarOk));
}

// ---------------------------------------------------------------------------
// A restored flight with the real servo controller and nav adapter
// ---------------------------------------------------------------------------

constexpr uint32_t kTickMs = 2;
constexpr uint32_t kFirstStateMs = 3000;   // the restore's barometer wait (raw_nav_policy.h kRestoreBaroWaitMs)
constexpr uint32_t kGnssMs = 5000;         // the receiver's velocity back
constexpr uint32_t kEndMs = 9000;
constexpr float kGapMps = 120.0f;          // 80 m/s restored against 200 true
constexpr float kRollRateDps = 1.0f;       // a steady roll rate the loop nulls

enum class Law { Held, GainSchedule, PureGyro };

struct Pass
{
    uint32_t t_ms;
    Law law;
    float speed_used;   // the speed the schedule ran on (GainSchedule)
    float speed_true;
    bool neutral;       // every servo at its calibrated neutral
};

class RestoredFlight : public ::testing::Test
{
protected:
    // A 1000-2000 us, +/-20 deg calibration: neutral 1500 us.
    TR_ServoControl servo{1, 2, 3, 4, 0, 0, 0, 0, 50, 1000, 2000, 0.12f, 0.01f, 0.0f, -20.0f, 20.0f};
    ShadowLink link;
    RawNavAdapter ekf;

    bool fixed = true;              // main.cpp after #1769, or before it
    bool ekf_initialized = true;    // the snapshot's: the filter ran before the reset
    bool filter_starts = true;      // the restored filter publishes from kFirstStateMs
    bool gnss_back = true;          // the receiver's velocity at kGnssMs
    // a restore that found no filter: a degraded start in flight at this time
    // (a receiver absent this boot), at zero velocity and sure of it (0: none)
    uint32_t degraded_start_ms = 0;
    bool restore_had_filter = true; // set by fly(): the restore's ekf_initialized

    bool reboot_recovery = false;
    uint32_t settle_end_ms = 0;
    float v_true = 200.0f;
    std::vector<Pass> passes;

    void SetUp() override
    {
        hostLedcLogReset();
        setMockMicros(1000);
        ASSERT_TRUE(servo.setFinCalibration(-20.0f, 20.0f));
        servo.begin();
        servo.enableGainSchedule(kVRef, kVMin);
        ekf.attach(&link);
    }

    // The scripted raw filter: the gap's speed short until GNSS is back, its
    // variance the restore's 400 (m/s)^2; then both converge in ~0.7 s.
    void publish(uint32_t t_ms)
    {
        if (degraded_start_ms != 0 && t_ms >= degraded_start_ms)
        {
            ekf_initialized = true;
            if (t_ms % 10 == 0)
                link.nav.write(navState(t_ms * 1000u, 0.0f, 0.1f));
            return;
        }
        if (!filter_starts || t_ms < kFirstStateMs || t_ms % 10 != 0)
            return;
        float err = kGapMps, var = kRestoreVelSigma * kRestoreVelSigma;
        if (gnss_back && t_ms >= kGnssMs)
        {
            const float k = std::exp(-float(t_ms - kGnssMs) / 150.0f);
            err = kGapMps * k;
            var = std::max(0.5f, var * k);
        }
        link.nav.write(navState(t_ms * 1000u, -(v_true - err), var));
    }

    // One flight-loop pass in INFLIGHT, as main.cpp runs it.
    void pass(uint32_t t_ms)
    {
        setMockMicros(1000 + t_ms * 1000u);
        publish(t_ms);
        ekf.refresh(t_ms * 1000u);

        // the restore hold's release
        if (reboot_recovery)
        {
            if (fixed)
            {
                float var_vel[3];
                ekf.getCovVel(var_vel);
                const bool trusted = RestoreControlPolicy::velocityTrusted(ekf.isHealthy(), var_vel, kVarOk);
                if (!RestoreControlPolicy::holdFins(t_ms, settle_end_ms, restore_had_filter, trusted))
                    reboot_recovery = false;
            }
            else if (t_ms >= settle_end_ms)
                reboot_recovery = false;
        }

        // the roll chain (roll delay, speed gate and the eject's cut all open)
        Pass p{t_ms, Law::Held, 0.0f, v_true, false};
        if (reboot_recovery)
        {
            servo.stowControl();
            servo.resetPID();
        }
        else
        {
            const bool ekf_ctrl_healthy =
                fixed ? RestoreControlPolicy::filterUsableForControl(ekf_initialized, ekf.isHealthy(), true,
                                                                     restore_had_filter)
                      : ekf_initialized && ekf.isHealthy();
            if (ekf_ctrl_healthy)
            {
                float v[3];
                ekf.getVelEst(v);
                const float speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                servo.controlWithGainSchedule(-kRollRateDps, speed);
                p.law = Law::GainSchedule;
                p.speed_used = speed;
            }
            else
            {
                servo.control(-kRollRateDps);
                p.law = Law::PureGyro;
            }
        }
        p.neutral = true;
        for (int i = 0; i < 4; ++i)
            p.neutral = p.neutral && servo.getServoPulseUs(i) == 1500;
        passes.push_back(p);

        // the truth: coasting after the restore, drag 16 m/s^2 at 200 m/s
        const float dt = kTickMs / 1000.0f;
        v_true -= (9.81f + 16.0f * (v_true / 200.0f) * (v_true / 200.0f)) * dt;
    }

    void fly()
    {
        // setup_fc's restore arms the hold
        restore_had_filter = ekf_initialized;
        reboot_recovery = true;
        settle_end_ms = 0 + kSettleMs;
        for (uint32_t t = 0; t <= kEndMs; t += kTickMs)
            pass(t);
    }
};

// The schedule's gain at a speed, against its reference speed's, measured on
// the real servo controller: its command for the same roll rate (the
// proportional term; the integrator's share over one 2 ms step is 0.02%).
float gainAt(float speed)
{
    auto cmd = [](float v) {
        TR_ServoControl s{1, 2, 3, 4, 0, 0, 0, 0, 50, 1000, 2000, 0.12f, 0.01f, 0.0f, -20.0f, 20.0f};
        s.setFinCalibration(-20.0f, 20.0f);
        s.begin();
        s.enableGainSchedule(kVRef, kVMin);
        setMockMicros(5000000);
        s.controlWithGainSchedule(-kRollRateDps, v);   // a first call only starts the PID's clock
        setMockMicros(5002000);
        s.controlWithGainSchedule(-kRollRateDps, v);
        return s.getRollCmdDeg();
    };
    return cmd(speed) / cmd(kVRef);
}

// The pure gyro's base gains: the schedule's at its reference speed.
constexpr float kBaseGain = 1.0f;

float gainUsed(const Pass& p)
{
    return p.law == Law::GainSchedule ? gainAt(p.speed_used) : kBaseGain;
}

TEST_F(RestoredFlight, TheFinsHoldNeutralUntilTheVelocityIsTrustedThenTheGainIsTheTrueSpeeds)
{
    fly();
    uint32_t release_ms = 0;
    for (const Pass& p : passes)
    {
        if (p.law == Law::Held)
        {
            EXPECT_TRUE(p.neutral) << p.t_ms;
            continue;
        }
        if (release_ms == 0)
            release_ms = p.t_ms;
        EXPECT_EQ(p.law, Law::GainSchedule) << p.t_ms;   // never the pure gyro
        EXPECT_LT(std::fabs(p.speed_used - p.speed_true), 2.0f) << p.t_ms;
        const float ratio = gainUsed(p) / gainAt(p.speed_true);
        EXPECT_GT(ratio, 0.9f) << p.t_ms;
        EXPECT_LT(ratio, 1.1f) << p.t_ms;
    }
    std::printf("restored flight: fins held to %u ms (GNSS back at %u ms), then the gain within 10%% of the "
                "true speed's\n", release_ms, kGnssMs);
    // past the settle and the filter's first state, until GNSS's velocity converged
    EXPECT_GE(release_ms, kGnssMs + 600);
    EXPECT_LE(release_ms, kGnssMs + 800);
    EXPECT_EQ(passes.back().law, Law::GainSchedule);
}

TEST_F(RestoredFlight, BeforeTheFixTheScheduleRanOnTheSnapshotsSpeed)
{
    fixed = false;
    fly();
    float worst = 0.0f, worst_gyro = 0.0f;
    uint32_t worst_ms = 0;
    for (const Pass& p : passes)
    {
        if (p.law == Law::Held)
        {
            EXPECT_LT(p.t_ms, kSettleMs);
            continue;
        }
        const float ratio = gainUsed(p) / gainAt(p.speed_true);
        if (p.law == Law::PureGyro)
            worst_gyro = std::max(worst_gyro, ratio);
        else if (ratio > worst)
        {
            worst = ratio;
            worst_ms = p.t_ms;
        }
    }
    std::printf("before #1769: the pure gyro to the filter's first state at up to %.1fx the true speed's gain, "
                "then the schedule at up to %.1fx (at %u ms)\n", worst_gyro, worst, worst_ms);
    EXPECT_GT(worst_gyro, 10.0f);   // the base gains at 150-200 m/s
    EXPECT_GT(worst, 20.0f);        // the review's 26x
}

TEST_F(RestoredFlight, WithNoGnssTheFinsHoldNeutralForTheFlight)
{
    gnss_back = false;
    fly();
    for (const Pass& p : passes)
    {
        ASSERT_EQ(p.law, Law::Held) << p.t_ms;
        ASSERT_TRUE(p.neutral) << p.t_ms;
    }
}

TEST_F(RestoredFlight, ARestoredFilterThatNeverStartsHoldsTheFinsNeutral)
{
    filter_starts = false;
    fly();
    for (const Pass& p : passes)
    {
        ASSERT_EQ(p.law, Law::Held) << p.t_ms;
        ASSERT_TRUE(p.neutral) << p.t_ms;
    }
}

TEST_F(RestoredFlight, AFlightWhoseFilterNeverRanFliesThePureGyroAfterTheSettleAsBefore)
{
    ekf_initialized = false;
    filter_starts = false;
    fly();
    std::vector<Pass> after = passes;
    passes.clear();
    fixed = false;
    ekf_initialized = false;
    v_true = 200.0f;
    hostLedcLogReset();
    fly();
    ASSERT_EQ(after.size(), passes.size());
    for (size_t i = 0; i < passes.size(); ++i)
    {
        EXPECT_EQ(after[i].law, passes[i].law) << after[i].t_ms;
        EXPECT_EQ(after[i].law, after[i].t_ms < kSettleMs ? Law::Held : Law::PureGyro) << after[i].t_ms;
    }
}

}  // namespace

namespace {

// A restore that found no filter, and a receiver absent this boot: the
// degraded start runs in flight 5 s after the boot, at zero velocity and sure
// of it. Before the fix the gain schedule then ran on that zero (its 3x cap);
// now the flight stays on the pure gyro, as it flew before the reset.
TEST_F(RestoredFlight, AFilterStartedInFlightAfterAFilterlessRestoreIsNeverUsed)
{
    ekf_initialized = false;
    filter_starts = false;
    degraded_start_ms = 5000;
    fly();
    for (const Pass& p : passes)
        ASSERT_EQ(p.law, p.t_ms < kSettleMs ? Law::Held : Law::PureGyro) << p.t_ms;

    passes.clear();
    fixed = false;
    ekf_initialized = false;
    v_true = 200.0f;
    fly();
    float worst = 0.0f;
    for (const Pass& p : passes)
        if (p.law == Law::GainSchedule)
        {
            EXPECT_GE(p.t_ms, degraded_start_ms);
            worst = std::max(worst, gainUsed(p) / gainAt(p.speed_true));
        }
    std::printf("before #1769: after a filterless restore, the in-flight start's zero speed at up to %.1fx the "
                "true speed's gain\n", worst);
    EXPECT_GT(worst, 10.0f);
}

}  // namespace
