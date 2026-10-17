// Host tests for the replay harness's apogee quorum (tools/raw_replay/
// flight_votes.h): main.cpp's kinematics block a flight-loop pass at a time,
// on the firmware's own TR_KinematicChecks, GnssAscentGate, burnout detector,
// raw vote guard and RawNavAdapter. A synthetic flight (a point mass on an
// 85 deg rail, 1.5 s of 8 g, drag, the nose on the velocity) is the truth;
// the navigation filter's published state, the barometer and the receiver's
// fix are taken from it. On the truth the quorum calls after the true apogee
// within the voters' counter latency, and no vote is up while the rocket
// climbs. A filter reading 15 m/s low from a second after burnout latches the
// guard; with the receiver's PVT reading low with it (one receiver behind
// three voters, a coherent fault) the quorum without the guard calls apogee
// seconds early, and with it waits for the barometer. The plan's coherent
// coast fault (a 0 -> 5 m/s ramp over the 3 s before apogee) is the guard's
// gap: it never latches, and the filter's velocity votes while the rocket
// still climbs at over 2 m/s. The filter's tick flags follow the quorum: its
// in-flight levelling starts at the quorum's own call, whatever the events
// file's level bit says, and with state_from_quorum the launch and the
// landing are the quorum's too.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "ShadowLink.h"
#include "flight_votes.h"

namespace {

constexpr double kG = 9.80665;

struct Sample
{
    double t;          // s from launch
    double h;          // m above the pad
    double vh, vu;     // m/s, horizontal (north) and up
    double pitch;      // rad, the nose's elevation
    double fx, fz;     // specific force, body x (along the nose) and z, m/s^2
};

// 1 kHz from launch until 8 s after apogee
std::vector<Sample> flight(double& t_apogee)
{
    std::vector<Sample> out;
    const double dt = 1e-3, rail = 85.0 * M_PI / 180.0, thrust = 8.0 * kG, burn = 1.5, k = 4e-4;
    double t = 0.0, h = 0.0, vh = 0.0, vu = 0.0;
    bool off_rail = false;
    t_apogee = -1.0;
    for (int i = 0; i < 40000; ++i)
    {
        const double v = std::sqrt(vh * vh + vu * vu);
        off_rail = off_rail || v > 20.0;
        const double pitch = off_rail ? std::atan2(vu, vh) : rail;   // off the rail it weathercocks
        const double ft = (t < burn ? thrust : 0.0) - k * v * v;      // along the nose
        const double ah = ft * std::cos(pitch), au = ft * std::sin(pitch) - kG;
        // the specific force in the body: along the nose, and gravity's share across it (the rail's reaction)
        const double fz = t < 0.05 ? -kG * std::cos(pitch) : 0.0;
        out.push_back({t, h, vh, vu, pitch, ft, fz});
        const double vu0 = vu;
        vh += ah * dt;
        vu += au * dt;
        h += 0.5 * (vu0 + vu) * dt;
        t += dt;
        if (t_apogee < 0.0 && vu0 > 0.0 && vu <= 0.0)
            t_apogee = t - dt * vu / (vu - vu0);
        if (t_apogee > 0.0 && t > t_apogee + 8.0)
            break;
    }
    return out;
}

struct Faults
{
    double filter_vz_bias = 0.0;   // m/s, up-positive, on the filter's vertical velocity...
    double gps_vu_bias = 0.0;      // ...and the receiver's
    double from_s = 1.0e9;         // from this flight time
    // the plan's coherent coast fault (G4/G5): the filter's vz and the receiver's vU both biased down by a ramp
    // from 0 to ramp_mps over the ramp_s before the true apogee, held after it
    double ramp_mps = 0.0;
    double ramp_s = 3.0;
};

// How the flight is flown through the loop, besides the faults.
struct Flying
{
    bool guard = true;
    bool state_from_quorum = false;     // launch and landing from the quorum's own kinematics
    double ticks_in_flight_from_s = 0.0;   // the IMU ticks' own in-flight flag rises at this flight time
    double rest_s = 0.0;                // after the flight: at rest on the ground this long (a landing)
    // each pass, after it: the MCU time (s), the quorum, and the tick's own in-flight flag
    std::function<void(double, const rr::FlightVotes&, bool)> observe;
};

struct Flown
{
    rr::FlightVotes::Report rep;
    double t_apogee = 0.0;
    double t_launch_us = 0.0;
    double t_under60 = 0.0;   // when the climb first fell under 60 m/s after the boost (the guard judges below it)
    std::vector<Sample> truth;
};

// The flight through the loop: 3 s on the pad, then the flight. MCU time =
// flight time + 100 s.
Flown fly(const Faults& f, const Flying& how)
{
    double t_apogee = 0.0;
    std::vector<Sample> tr = flight(t_apogee);
    const size_t n_flight = tr.size();
    if (how.rest_s > 0.0)
    {
        // lying still on the ground (palt 0): the landing detector's quiet barometer, gyro and 1 g
        const double t_end = tr.back().t, pitch = tr.back().pitch;
        for (int k = 1; k <= static_cast<int>(how.rest_s * 1000.0); ++k)
            tr.push_back({t_end + k * 1e-3, 0.0, 0.0, 0.0, pitch, kG, 0.0});
    }
    rr::FlightVotes::Config cfg;
    cfg.guard = how.guard;
    cfg.state_from_quorum = how.state_from_quorum;
    rr::FlightVotes votes(cfg);
    auto link = std::make_unique<gnss_raw::ShadowLink>();
    RawNavAdapter ekf;
    ekf.attach(link.get());
    votes.setInitialized(true);
    const double t0 = 100.0;            // the launch, MCU s
    const double lat0 = 0.5, lon0 = 2.0, h0 = 150.0;   // synthetic: nowhere anyone flies
    const double p0 = 101325.0;
    auto bias = [&](double tf, double step_bias) {
        double b = tf >= f.from_s ? step_bias : 0.0;
        if (f.ramp_mps != 0.0)
        {
            const double x = (tf - (t_apogee - f.ramp_s)) / f.ramp_s;
            b -= f.ramp_mps * std::max(0.0, std::min(1.0, x));
        }
        return b;
    };
    auto pass = [&](double tf, const Sample& s) {
        const int64_t t_us = static_cast<int64_t>(std::llround((t0 + tf) * 1e6));
        const bool in_flight = tf >= how.ticks_in_flight_from_s;
        // the filter's published state, every pass
        gnss_raw::ShadowNav n{};
        n.time_us = static_cast<uint32_t>(t_us);
        n.lla[0] = lat0;
        n.lla[1] = lon0;
        n.lla[2] = h0 + s.h;
        n.vel_ned[0] = static_cast<float>(s.vh);
        n.vel_ned[1] = 0.0f;
        n.vel_ned[2] = static_cast<float>(-(s.vu + bias(tf, f.filter_vz_bias)));
        n.quat[0] = static_cast<float>(std::cos(0.5 * s.pitch));
        n.quat[2] = static_cast<float>(std::sin(0.5 * s.pitch));
        for (float& v : n.var)
            v = 0.01f;
        n.running = 1;
        n.gnss_aided = 1;
        n.baro_aided = 1;
        link->nav.write(n);
        const int64_t step = static_cast<int64_t>(std::llround(tf * 1000.0));
        if ((step & 1) == 0)   // the EKF ticks, 500 a second
        {
            const float acc[3] = {static_cast<float>(s.fx), 0.0f, static_cast<float>(s.fz)};
            const float gyro[3] = {};
            votes.onImu(t_us, acc, gyro, in_flight);
        }
        if ((step & 1) == 1)   // the barometer, ~500 Hz
        {
            const double pa = p0 * std::pow(1.0 - s.h / 44330.0, 5.255);
            votes.onBaro(t_us, static_cast<float>(s.h), static_cast<float>(pa));
        }
        if (step % 50 == 0)   // the receiver, 20 Hz
        {
            rr::FixIn fx;
            fx.t_us = t_us;
            fx.fix_mode = 3;
            fx.num_sats = 20;
            fx.alt_m = h0 + s.h;
            fx.vel_n = s.vh;
            fx.vel_u = s.vu + bias(tf, f.gps_vu_bias);
            votes.onFix(fx);
        }
        votes.pass(t_us, ekf);
        if (how.observe)
            how.observe(t0 + tf, votes, in_flight);
    };
    // the pad
    const Sample pad{0.0, 0.0, 0.0, 0.0, 85.0 * M_PI / 180.0, kG * std::sin(85.0 * M_PI / 180.0),
                     -kG * std::cos(85.0 * M_PI / 180.0)};
    for (int i = -3000; i < 0; ++i)
        pass(i * 1e-3, pad);
    for (const Sample& s : tr)
        pass(s.t, s);
    Flown r;
    for (size_t i = 0; i < n_flight; ++i)
        if (tr[i].t > 1.5 && tr[i].vu < 60.0)
        {
            r.t_under60 = t0 + tr[i].t;
            break;
        }
    r.rep = votes.report();
    r.t_apogee = t_apogee + t0;
    r.t_launch_us = t0;
    r.truth.assign(tr.begin(), tr.begin() + static_cast<long>(n_flight));
    return r;
}

Flown fly(const Faults& f, bool guard)
{
    Flying how;
    how.guard = guard;
    return fly(f, how);
}

// The truth's vertical velocity at MCU time t (s)
double trueVu(const Flown& r, double t)
{
    const double tf = t - r.t_launch_us;
    for (const Sample& s : r.truth)
        if (s.t >= tf)
            return s.vu;
    return r.truth.back().vu;
}

double firstOn(const rr::FlightVotes::Edges& e) { return e.on.empty() ? 1.0e9 : e.on.front(); }

}  // namespace

TEST(RawReplayVotes, OnTheTruthTheQuorumCallsJustAfterApogee)
{
    const Flown r = fly(Faults{}, true);
    const rr::FlightVotes::Report& q = r.rep;
    ASSERT_GT(r.t_apogee, 105.0);   // a flight worth the name
    EXPECT_NEAR(q.launch_s, 100.0, 0.002);
    EXPECT_LT(q.kc_launch_s - 100.0, 0.2);
    // burnout: 50 passes of negative body x acceleration, after the thrust ends at 1.5 s
    EXPECT_GT(q.burnout_s - 100.0, 1.5);
    EXPECT_LT(q.burnout_s - 100.0, 1.6);
    // GNSS admitted against the barometer (the mosaic build's only way) well before apogee
    EXPECT_LT(q.admitted_s, r.t_apogee - 2.0);
    EXPECT_EQ(q.admitted_reason, static_cast<int>(GnssAscentGate::Reason::BaroAgreed));
    // the call: after the true apogee, within the counters' latency (the velocity's 6 passes, the GPS's 4 fixes)
    ASSERT_FALSE(std::isnan(q.apogee_s));
    EXPECT_GT(q.apogee_s, r.t_apogee);
    EXPECT_LT(q.apogee_s - r.t_apogee, 0.3);
    std::printf("[ truth ] apogee T+%.3f, call +%.3f s (vel %d baro %d gps %d pitch %d), first votes: vel +%.3f, "
                "pitch +%.3f, gps +%.3f, baro +%.3f\n",
                r.t_apogee - 100.0, q.apogee_s - r.t_apogee, q.call_vel, q.call_baro, q.call_gps, q.call_pitch,
                firstOn(q.vel) - r.t_apogee, firstOn(q.pitch) - r.t_apogee, firstOn(q.gps) - r.t_apogee,
                firstOn(q.baro) - r.t_apogee);
    // no vote up while the rocket climbs
    EXPECT_GE(firstOn(q.vel), r.t_apogee);
    EXPECT_GE(firstOn(q.pitch), r.t_apogee);
    EXPECT_GE(firstOn(q.gps), r.t_apogee);
    EXPECT_GE(firstOn(q.baro), r.t_apogee);
    EXPECT_TRUE(q.call_ekf_healthy);
    EXPECT_TRUE(std::isnan(q.guard_s));   // and the guard never latched
    EXPECT_GE(q.ref_count, 50u);
    EXPECT_TRUE(q.ref_from_nav);
    EXPECT_EQ(q.mach_lockouts, 0u);
}

TEST(RawReplayVotes, AFilterReadingLowLatchesTheGuard)
{
    Faults f;
    f.filter_vz_bias = -15.0;
    f.from_s = 2.5;   // a second after burnout
    const Flown r = fly(f, true);
    const rr::FlightVotes::Report& q = r.rep;
    ASSERT_FALSE(std::isnan(q.guard_s));
    // held a second from when the guard can judge: the fault is on from 2.5 s, but the barometer's rate is a
    // witness only under 60 m/s
    EXPECT_GT(r.t_under60, 102.5);
    EXPECT_GT(q.guard_s - r.t_under60, 1.0);
    EXPECT_LT(q.guard_s - r.t_under60, 1.3);
    EXPECT_LT(q.guard_vz, q.guard_baro_rate - 10.0f);
    // the call waits for the barometer and the receiver, after apogee
    ASSERT_FALSE(std::isnan(q.apogee_s));
    EXPECT_GT(q.apogee_s, r.t_apogee);
    EXPECT_FALSE(q.call_ekf_healthy);
    EXPECT_TRUE(q.call_baro);
    EXPECT_TRUE(q.call_gps);
}

TEST(RawReplayVotes, ACoherentReceiverFaultIsWhatTheGuardIsFor)
{
    Faults f;
    f.filter_vz_bias = -15.0;
    f.gps_vu_bias = -15.0;
    f.from_s = 2.5;
    // without the guard: the filter's velocity and the receiver's both vote seconds early
    const Flown bare = fly(f, false);
    ASSERT_FALSE(std::isnan(bare.rep.apogee_s));
    EXPECT_LT(bare.rep.apogee_s, bare.t_apogee - 1.0);
    EXPECT_TRUE(bare.rep.call_vel);
    EXPECT_TRUE(bare.rep.call_gps);
    EXPECT_FALSE(std::isnan(bare.rep.guard_s));   // judged all the same (reported, not applied)
    // with it: latched long before, and the call waits for the barometer
    const Flown guarded = fly(f, true);
    ASSERT_FALSE(std::isnan(guarded.rep.guard_s));
    EXPECT_LT(guarded.rep.guard_s, bare.rep.apogee_s);
    ASSERT_FALSE(std::isnan(guarded.rep.apogee_s));
    EXPECT_GT(guarded.rep.apogee_s, guarded.t_apogee);
    EXPECT_TRUE(guarded.rep.call_baro);
    std::printf("[ coherent ] apogee T+%.3f: without the guard called at %+.3f s, with it latched at T+%.3f and "
                "called at %+.3f s\n",
                bare.t_apogee - 100.0, bare.rep.apogee_s - bare.t_apogee, guarded.rep.guard_s - 100.0,
                guarded.rep.apogee_s - guarded.t_apogee);
}

TEST(RawReplayVotes, ACoherentCoastRampIsTheGuardsGap)
{
    // The plan's coherent coast fault (G4/G5): the filter's vertical velocity and the receiver's both ramp to 5 m/s
    // low over the 3 s before the true apogee. The guard is one-sided at 10 m/s, so it never latches; the filter's
    // velocity votes while the rocket still climbs above +2 m/s, which G5's hard bar forbids. This states the gap,
    // it does not pass G5: the bar on this pattern needs something other than this guard, or a changed bar.
    Faults f;
    f.ramp_mps = 5.0;
    const Flown r = fly(f, true);
    const rr::FlightVotes::Report& q = r.rep;
    EXPECT_TRUE(std::isnan(q.guard_s));   // never latched
    ASSERT_FALSE(q.vel.on.empty());
    const double t_vel = q.vel.on.front();
    EXPECT_LT(t_vel, r.t_apogee);
    EXPECT_GT(trueVu(r, t_vel), 2.0);     // a raw velocity vote while the true vz is above +2 m/s
    ASSERT_FALSE(std::isnan(q.apogee_s));
    std::printf("[ ramp 5 ] apogee T+%.3f: first raw velocity vote %+.3f s (true vz %.2f), GPS %+.3f, call %+.3f s "
                "(true vz %.2f), guard %s\n",
                r.t_apogee - 100.0, t_vel - r.t_apogee, trueVu(r, t_vel), firstOn(q.gps) - r.t_apogee,
                q.apogee_s - r.t_apogee, trueVu(r, q.apogee_s), std::isnan(q.guard_s) ? "silent" : "latched");
    // without the ramp, nothing votes before the true apogee (the same flight: OnTheTruth...)
    const Flown clean = fly(Faults{}, true);
    EXPECT_GE(firstOn(clean.rep.vel), clean.t_apogee);
}

TEST(RawReplayVotes, LevellingInFlightStartsAtTheQuorumsCall)
{
    // The events file's level bit says the log's own call came 2 s after the true apogee (a twin's call is its
    // own): the filter's tick levels in flight from the quorum's call (use_ahrs_acc), not from the file's time.
    double t_level = NAN, t_apogee_bit = NAN, t_file_level = NAN;
    bool level_before = false;
    double t_apogee_true = 0.0;
    flight(t_apogee_true);
    const double file_call = 100.0 + t_apogee_true + 2.0;
    Flying how;
    how.observe = [&](double t, const rr::FlightVotes& v, bool tick_in_flight) {
        using namespace gnss_raw;
        const bool file_level = !tick_in_flight || t >= file_call;
        const uint8_t tick = static_cast<uint8_t>((tick_in_flight ? kShadowInFlight : 0) | (file_level ? kShadowLevel : 0));
        const uint8_t got = v.shadowFlags(tick);
        if (tick_in_flight && file_level && std::isnan(t_file_level))
            t_file_level = t;
        if ((got & kShadowInFlight) && (got & kShadowLevel))
        {
            if (std::isnan(t_level))
                t_level = t;
            if (!v.apogee())
                level_before = true;
        }
        if ((got & kShadowApogee) && std::isnan(t_apogee_bit))
            t_apogee_bit = t;
        EXPECT_EQ((got & kShadowInFlight) != 0, tick_in_flight);   // the file's in-flight flag stands
    };
    const Flown r = fly(Faults{}, how);
    const rr::FlightVotes::Report& q = r.rep;
    ASSERT_FALSE(std::isnan(q.apogee_s));
    ASSERT_FALSE(std::isnan(t_level));
    EXPECT_NEAR(t_level, q.apogee_s, 1e-6);        // the same pass the kinematics called it
    EXPECT_NEAR(t_apogee_bit, q.apogee_s, 1e-6);   // and kShadowApogee with it
    EXPECT_FALSE(level_before);
    EXPECT_GT(t_file_level - t_level, 1.5);        // the file's level time is not what the filter gets
}

TEST(RawReplayVotes, WithStateFromQuorumTheLaunchAndLandingAreTheQuorums)
{
    // A sim's ticks say in flight from before the motor lights (E0's thrust_start_s, 0.25 s early here); with
    // state_from_quorum the flight computer's own state machine decides: INFLIGHT in the pass the kinematics'
    // launch flag latches, LANDED after the landing dwell on its landed flag.
    double t_in = NAN, t_landed_bit = NAN, t_out = NAN;
    Flying how;
    how.state_from_quorum = true;
    how.ticks_in_flight_from_s = -0.25;
    how.rest_s = 15.0;
    how.observe = [&](double t, const rr::FlightVotes& v, bool) {
        using namespace gnss_raw;
        const uint8_t got = v.shadowFlags(kShadowInFlight);   // the ticks' own flag says in flight throughout
        if ((got & kShadowInFlight) && std::isnan(t_in))
            t_in = t;
        if (!std::isnan(t_in) && !(got & kShadowInFlight) && std::isnan(t_out))
        {
            t_out = t;
            EXPECT_TRUE(got & kShadowLevel);   // LANDED: not in flight, levelled
        }
        if ((got & kShadowLanded) && std::isnan(t_landed_bit))
            t_landed_bit = t;
    };
    const Flown r = fly(Faults{}, how);
    const rr::FlightVotes::Report& q = r.rep;
    ASSERT_FALSE(std::isnan(q.kc_launch_s));
    EXPECT_NEAR(q.launch_s, q.kc_launch_s, 1e-6);   // enterInflight in the pass the launch flag latched
    EXPECT_GT(q.launch_s, 100.0);                   // not the ticks' edge, 0.25 s before the motor
    EXPECT_NEAR(t_in, q.launch_s, 1e-6);
    ASSERT_FALSE(std::isnan(q.apogee_s));
    EXPECT_GT(q.apogee_s, r.t_apogee);
    // the landing: the kinematics' landed flag first (kShadowLanded while still INFLIGHT), LANDED after the dwell
    ASSERT_FALSE(std::isnan(q.landed_s));
    ASSERT_FALSE(std::isnan(t_landed_bit));
    EXPECT_GT(t_landed_bit, q.apogee_s);
    EXPECT_GE(q.landed_s - t_landed_bit, landing_transition::kDwellMs * 1e-3 - 0.002);
    EXPECT_NEAR(t_out, q.landed_s, 1e-6);
    // and by default the ticks' own edge is the launch
    Flying logged = how;
    logged.state_from_quorum = false;
    logged.observe = nullptr;
    const Flown rl = fly(Faults{}, logged);
    EXPECT_NEAR(rl.rep.launch_s, 100.0 - 0.25, 0.002);
    EXPECT_TRUE(std::isnan(rl.rep.landed_s));
}

TEST(RawReplayVotes, PassesRunOnTheLoopsGrid)
{
    rr::FlightVotes::Config c;
    c.loop_period_us = 1000;
    rr::FlightVotes v(c);
    EXPECT_EQ(v.nextPass(100000), 101000);
    EXPECT_EQ(v.nextPass(100999), 101000);
    c.loop_period_us = 2000;
    rr::FlightVotes v2(c);
    EXPECT_EQ(v2.nextPass(100500), 102000);
}
