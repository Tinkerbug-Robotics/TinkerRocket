// #1764: a barometer judged stuck casts no apogee vote for the rest of the
// flight (GnssAscentGate::baroVotes).
//
// The real TR_KinematicChecks, by sensor, and the real GnssAscentGate, wired
// as main.cpp's kinematics block wires them: the gate stepped first, the vote
// handed gnssVotes() and baroVotes() (or, for "before", a barometer that
// always may vote).  A 1-D flight: 1 s on the pad, 1.5 s at 8 g, a drag-free
// coast to about 620 m, a ballistic fall.  The flight loop runs at 1 kHz.  The
// static port is sealed: the barometer reads fresh, in range and flat.
// InertialVertical is out (a reboot's restored flight, or a failure in
// flight), so the accelerometer's and the gyro's votes are out too.  GNSS
// sends a usable fix at 10 Hz, or once a second with the GNSS board's
// settings (main.cpp: a 1.5 s fix gap and freshness, a count of 2).
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "GnssAscentGate.h"
#include "TR_KinematicChecks.h"

namespace
{

constexpr double kG = 9.80665;

enum class Port
{
    Sealed,        // flat at the pad's altitude from the start
    Good,          // reads the true height
    FrozenInCoast, // reads the true height, then freezes 5 s before apogee
};

enum class Vote
{
    Latched,       // the fix: baroVotes(), latched for the flight
    Always,        // before it: a barometer that always may vote
    Leaky,         // the fusion's leaky verdict: !baro_stuck
    Never,         // a barometer that never may vote (as if latched from the start)
};

struct Flight
{
    Port port = Port::Sealed;
    Vote vote = Vote::Latched;
    bool inertial_healthy = false;  // InertialVertical flying (until a reboot)
    uint32_t fix_period_ms = 100;   // 10 Hz; 1000 for the GNSS board
    bool gnss_held_out = false;     // GNSS never admitted to the vote
    double rx_lag_s = 0.0;          // the receiver's fix reads the truth this late
    double baro_noise_m = 0.0;      // uniform noise on the barometer's reading
    double reboot_at_s = -1.0;      // an FC reboot (restored flight) at this flight time
};

struct Outcome
{
    bool called = false;
    bool single = false;
    bool backstop = false;
    double after_apogee_s = 0.0;
    GnssAscentGate::Reason admitted = GnssAscentGate::Reason::None;
    bool baro_stuck_seen = false;
};

Outcome fly(const Flight& f)
{
    TR_KinematicChecks kc;
    setMockMillis(0);
    kc.reset();
    const bool once_a_second = f.fix_period_ms >= 1000;
    GnssAscentGate::State gate;
    GnssAscentGate::Config gcfg;
    if (once_a_second) gcfg.max_fix_gap_ms = 1500;

    const double t_pad = 1.0, burn = 1.5, a_boost = 7.0 * kG;   // 8 g specific force
    const double v_bo = a_boost * burn, h_bo = 0.5 * a_boost * burn * burn;
    const double t_bo = t_pad + burn, t_apogee = t_bo + v_bo / kG;
    const double h_apogee = h_bo + v_bo * v_bo / (2.0 * kG);
    const double t_end = t_apogee + std::sqrt(2.0 * h_apogee / kG) - 0.3;   // just short of the ground
    // height, up velocity and specific force at flight time t
    auto truth = [&](double t, double& h, double& v, double& sf) {
        h = 0.0, v = 0.0, sf = kG;
        if (t >= t_pad && t < t_bo)
        {
            const double tb = t - t_pad;
            h = 0.5 * a_boost * tb * tb;
            v = a_boost * tb;
            sf = a_boost + kG;
        }
        else if (t >= t_bo)
        {
            const double tc = t - t_bo;
            h = h_bo + v_bo * tc - 0.5 * kG * tc * tc;
            v = v_bo - kG * tc;
            sf = 0.0;
        }
    };

    double baro_frozen_m = -1.0;
    uint32_t fix_id = 0, next_fix_ms = 0, last_fix_ms = 0, lcg = 12345u;
    double fix_h = 0.0, fix_v = 0.0;   // the latest fix, held between fixes (gnss_latest_si)
    bool have_fix = false, launched = false, rebooted = false;
    uint8_t snap_code = 0;             // the 10 Hz flight snapshot's gnss_admission byte
    Outcome out;
    for (uint32_t ms = 0; (double)ms * 1e-3 < t_end; ++ms)
    {
        const double t = ms * 1e-3;
        double h, v, sf;
        truth(t, h, v, sf);
        double baro_m = h;
        if (f.port == Port::Sealed) baro_m = 0.0;
        if (f.port == Port::FrozenInCoast && t >= t_apogee - 5.0)
        {
            if (baro_frozen_m < 0.0) baro_frozen_m = h;
            baro_m = baro_frozen_m;
        }
        lcg = lcg * 1664525u + 1013904223u;
        baro_m += f.baro_noise_m * (((double)(lcg >> 8) / 8388608.0) * 2.0 - 1.0);
        setMockMillis(ms);

        if (f.reboot_at_s >= 0.0 && !rebooted && t >= f.reboot_at_s)
        {
            // the restore: a new boot's checks with the launch latched, the
            // gate from the last snapshot, InertialVertical out (never launched)
            rebooted = true;
            kc.reset();
            kc.launch_flag = true;
            GnssAscentGate::restore(gate, snap_code, ms);
        }
        kc.setGpsApogeeFreshMs(once_a_second ? 1500u : 500u);   // every pass, as main.cpp
        kc.setGpsApogeeCountHi(once_a_second ? 2u : 4u);

        bool new_gps = false;
        if (ms >= next_fix_ms)
        {
            next_fix_ms = ms + f.fix_period_ms;
            ++fix_id;
            new_gps = true;
            have_fix = true;
            last_fix_ms = ms;
            double sf_unused;
            truth(t - f.rx_lag_s, fix_h, fix_v, sf_unused);
        }
        const bool burnout = t >= t_bo + 0.1;

        // GnssAscentGate first, as main.cpp steps it, on the latest fix
        GnssAscentGate::Inputs ga;
        ga.now_ms = ms;
        ga.in_flight = launched;
        ga.burnout = burnout;
        ga.apogee = kc.apogee_flag;
        ga.gnss_fix_ok = have_fix && (ms - last_fix_ms) < 2000;
        ga.gnss_fix_id = fix_id;
        ga.gnss_agl_m = (float)fix_h;
        ga.gnss_vel_u_mps = (float)fix_v;
        ga.baro_healthy = true;
        ga.baro_agl_m = (float)baro_m;
        ga.baro_rate_mps = kc.d_alt_est_;
        ga.ekf_valid = false;
        GnssAscentGate::step(gate, ga, gcfg);
        if (ms % 100 == 0) snap_code = GnssAscentGate::encode(gate);

        bool baro_may_vote = true;
        if (f.vote == Vote::Latched) baro_may_vote = GnssAscentGate::baroVotes(gate);
        if (f.vote == Vote::Leaky) baro_may_vote = !gate.baro_stuck;
        if (f.vote == Vote::Never) baro_may_vote = false;

        TR_KinematicChecks::InertialVote iv;
        iv.in_use = true;
        iv.healthy = f.inertial_healthy && !rebooted;
        iv.vz_up_mps = (float)v;
        iv.height_m = (float)h;
        iv.pitch_rad = v > 0.0 ? 1.4f : -1.4f;
        iv.pitch_may_vote = true;
        float pos[3] = {0.0f, 0.0f, 0.0f};   // the filter followed the stuck barometer
        float vel[3] = {0.0f, 0.0f, 0.0f};
        kc.kinematicChecks((float)baro_m, (float)sf, pos, vel, 0.0f, true, (float)fix_h, new_gps, 1.57f,
                           burnout, /*baro_locked_out*/ false, (float)fix_v, /*ekf_healthy*/ true,
                           /*baro_healthy*/ true, /*imu_healthy*/ true,
                           !f.gnss_held_out && GnssAscentGate::gnssVotes(gate),
                           /*baro_reading_fresh*/ true, &iv, baro_may_vote);
        if (!launched && kc.launch_flag)   // enterInflight
        {
            launched = true;
            GnssAscentGate::onLaunch(gate, ms);
        }
        if (kc.apogee_flag)
        {
            out.called = true;
            out.single = kc.apogee_single_sensor_flag;
            out.backstop = kc.apogee_backstop_flag;
            out.after_apogee_s = t - t_apogee;
            break;
        }
    }
    out.admitted = gate.reason;
    out.baro_stuck_seen = gate.baro_stuck_seen;
    return out;
}

// The issue's case: a sealed port with the IMU's voters out.  GNSS is admitted
// on the stuck verdict, the barometer is out of the vote, and GNSS calls apogee
// alone once it reads a clear descent.  Counted, the flat barometer held the
// vote at one of two to the ground.
TEST(BaroStuckVote, ASealedPortWithTheImuOutIsCalledByGnssAlone)
{
    for (uint32_t period : {100u, 1000u})
    {
        Flight f;
        f.fix_period_ms = period;
        const Outcome now = fly(f);
        EXPECT_EQ(now.admitted, GnssAscentGate::Reason::BaroStuck) << period;
        ASSERT_TRUE(now.called) << period;
        EXPECT_TRUE(now.single) << period;
        EXPECT_GT(now.after_apogee_s, 1.5) << period;   // 15 m/s down first
        EXPECT_LT(now.after_apogee_s, period >= 1000 ? 3.6 : 2.2) << period;
        f.vote = Vote::Always;
        EXPECT_FALSE(fly(f).called) << period << ": before, no apogee to the ground";
        std::printf("[ sealed, %4u ms fixes ] GNSS alone at %+.2f s\n", period, now.after_apogee_s);
    }
}

// The fusion's leaky verdict is not enough: it needs GNSS climbing, so it
// lapses about 2 s after the climb ends, and the barometer is back in the
// vote before GNSS reads 15 m/s down.
TEST(BaroStuckVote, TheLeakyVerdictLapsesBeforeTheCall)
{
    Flight f;
    f.vote = Vote::Leaky;
    EXPECT_FALSE(fly(f).called);
}

// A barometer that freezes in the coast, after GNSS was admitted on its
// agreement: the latch takes it out of the vote too.
TEST(BaroStuckVote, ABarometerFrozenAfterAdmissionIsCalledByGnssAlone)
{
    Flight f;
    f.port = Port::FrozenInCoast;
    const Outcome now = fly(f);
    EXPECT_EQ(now.admitted, GnssAscentGate::Reason::BaroAgreed);
    EXPECT_TRUE(now.baro_stuck_seen);
    ASSERT_TRUE(now.called);
    EXPECT_TRUE(now.single);
    EXPECT_LT(now.after_apogee_s, 2.2);
    f.vote = Vote::Always;
    EXPECT_FALSE(fly(f).called);
}

// Nothing changes for a good port: the barometer keeps its vote, and the
// call comes on the same pass as before.
TEST(BaroStuckVote, AGoodPortVotesAsBefore)
{
    for (bool inertial : {false, true})
    {
        Flight f;
        f.port = Port::Good;
        f.inertial_healthy = inertial;
        const Outcome now = fly(f);
        EXPECT_FALSE(now.baro_stuck_seen) << inertial;
        EXPECT_EQ(now.admitted, GnssAscentGate::Reason::BaroAgreed) << inertial;
        ASSERT_TRUE(now.called) << inertial;
        EXPECT_FALSE(now.single) << inertial;
        f.vote = Vote::Always;
        const Outcome before = fly(f);
        ASSERT_TRUE(before.called);
        EXPECT_DOUBLE_EQ(now.after_apogee_s, before.after_apogee_s) << inertial;
    }
}

// A good port with a receiver a second behind and a noisy barometer: near
// apogee the barometer is flat while the receiver still climbs, which must not
// latch.  The barometer keeps its vote; the call is the same pass as before.
TEST(BaroStuckVote, ALaggingReceiverAndANoisyBarometerDoNotCostTheVote)
{
    for (uint32_t period : {100u, 1000u})
        for (double lag : {0.5, 1.0, 2.0})
        {
            Flight f;
            f.port = Port::Good;
            f.fix_period_ms = period;
            f.rx_lag_s = lag;
            f.baro_noise_m = 1.0;
            const Outcome now = fly(f);
            EXPECT_FALSE(now.baro_stuck_seen) << period << " ms, " << lag << " s";
            ASSERT_TRUE(now.called) << period << " ms, " << lag << " s";
            f.vote = Vote::Always;
            const Outcome before = fly(f);
            ASSERT_TRUE(before.called);
            EXPECT_DOUBLE_EQ(now.after_apogee_s, before.after_apogee_s) << period << " ms, " << lag << " s";
        }
}

// The issue's failure scenario: the IMU up, a reboot in the boost restores the
// flight with InertialVertical out, the port is sealed.  The restored gate
// holds GNSS out, the verdict comes after burnout, and GNSS calls apogee alone.
TEST(BaroStuckVote, ARebootInBoostWithASealedPortIsCalledByGnssAlone)
{
    for (uint32_t period : {100u, 1000u})
    {
        Flight f;
        f.inertial_healthy = true;
        f.reboot_at_s = 2.0;   // 1 s into the burn
        f.fix_period_ms = period;
        const Outcome now = fly(f);
        EXPECT_EQ(now.admitted, GnssAscentGate::Reason::BaroStuck) << period;
        ASSERT_TRUE(now.called) << period;
        EXPECT_TRUE(now.single) << period;
        EXPECT_LT(now.after_apogee_s, period >= 1000 ? 3.6 : 2.2) << period;
        f.vote = Vote::Always;
        EXPECT_FALSE(fly(f).called) << period << ": before, no apogee to the ground";
    }
}

// A reboot after the verdict, near apogee: GNSS has stopped climbing, so the
// verdict could not be re-derived.  The snapshot carries the lost vote.
TEST(BaroStuckVote, ARebootAfterTheVerdictKeepsTheBarometerOut)
{
    Flight f;
    f.reboot_at_s = 12.5;   // the verdict came at about 4.6 s; apogee at about 13.0 s
    const Outcome now = fly(f);
    EXPECT_TRUE(now.baro_stuck_seen);
    ASSERT_TRUE(now.called);
    EXPECT_TRUE(now.single);
    EXPECT_LT(now.after_apogee_s, 2.2);
    f.vote = Vote::Always;
    EXPECT_FALSE(fly(f).called);
}

// The vote is all it loses: the barometer-only backstop (Layer 2) still reads
// it.  A good port, the IMU out, GNSS held out, the barometer with no vote:
// the backstop calls apogee once the barometer has fallen 30 m.
TEST(BaroStuckVote, TheBackstopStillReadsABarometerWithNoVote)
{
    Flight f;
    f.port = Port::Good;
    f.gnss_held_out = true;
    f.vote = Vote::Never;
    const Outcome o = fly(f);
    ASSERT_TRUE(o.called);
    EXPECT_TRUE(o.backstop);
    EXPECT_GT(o.after_apogee_s, 1.0);
    EXPECT_LT(o.after_apogee_s, 4.0);
}

// A sealed port with the IMU's voters up: the accelerometer and GNSS agree,
// so the quorum calls it, with or without the barometer counted.
TEST(BaroStuckVote, ASealedPortWithTheImuUpIsCalledByTheQuorum)
{
    Flight f;
    f.inertial_healthy = true;
    const Outcome now = fly(f);
    ASSERT_TRUE(now.called);
    EXPECT_FALSE(now.single);
    EXPECT_LT(now.after_apogee_s, 1.0);
    f.vote = Vote::Always;
    const Outcome before = fly(f);
    ASSERT_TRUE(before.called);
    EXPECT_LT(before.after_apogee_s, 1.0);
}

}  // namespace
