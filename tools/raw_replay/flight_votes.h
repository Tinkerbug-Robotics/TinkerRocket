// flight_votes.h -- the flight computer's apogee quorum, offline (raw_replay
// --quorum; on with --receiver mosaic).
//
// On the mosaic-only build the raw-GNSS filter is the one navigation filter:
// its vertical velocity and its pitch are two of the four apogee voters, the
// receiver's own fix (PVTGeodetic) is the GPS voter once GnssAscentGate admits
// it, and the barometer is the fourth. This runs main.cpp's kinematics block
// a flight-loop pass at a time with the firmware's own code:
// TR_KinematicChecks (the four leaky voters, the 2-of-N quorum and the
// Layer-2 barometer backstop), GnssAscentGate (admitted against the
// barometer only on the mosaic build: ekf_valid false), tr::burnoutDetectStep,
// the raw vote guard (raw_vote_guard.h) and RawNavAdapter, the face the loop
// reads the filter by (its state brought forward to the latest IMU tick, its
// health, heights above the geoid). What feeds them is the replay's: each
// IMU tick, barometer sample and receiver fix as it arrives, and the
// filter's state as its task published it.
//
// The pass, in main.cpp's order:
//   - the filter's state (RawNavAdapter::refresh), the transonic lockout on
//     its speed (260 on, 240 off), the attitude, ENU velocity and the height
//     above the pad reference (imu_rpy, imu_vel, imu_pos);
//   - the pad reference (ref_alt_m): the receiver's fixes that pass the
//     start's gate (3-D, 5 satellites, under 2 m/s), then from the filter's
//     first GNSS-aided state only its own heights (ref_from_nav), averaged
//     until launch or 2 minutes;
//   - GnssAscentGate::step, then (mosaic) RawVoteGuard::step with the
//     barometer as GnssAscentGate lets it vouch (baroFusable), then
//     kinematicChecks with ekf_healthy && !latched;
//   - in flight, burnout from the body x acceleration (50 passes negative,
//     after 200 ms), read by the next pass's vote as on the flight computer.
// The launch (enterInflight: GnssAscentGate::onLaunch, the guard reset, the
// reference frozen, the kinematics' apogee and landing flags cleared) is, by
// default, the replay's in-flight edge: the flags a log's own state machine
// gave the IMU ticks. With Config::state_from_quorum it is this quorum's own,
// as main.cpp's state machine takes it: enterInflight in the pass the
// kinematics' launch flag latches, and LANDED once landing_transition's dwell
// on its landed flag passes -- for a sim, whose events file has no flight
// computer behind it. The pass rate is the flight loop's (1 kHz by default);
// the leaky counters count passes.
//
// shadowFlags() is what the raw filter's tick is given from that state
// (rawShadowPushImu): in flight, levelled while not in flight or once the
// kinematics have called apogee (use_ahrs_acc), landed on the kinematics'
// landed flag or the LANDED state, and the apogee call (pyro_apogee_detected).
//
// Pure, given the host shim's millis() (tests_cpp/test_raw_replay_votes.cpp).
#ifndef RAW_REPLAY_FLIGHT_VOTES_H
#define RAW_REPLAY_FLIGHT_VOTES_H

#include <cmath>
#include <cstdint>
#include <vector>

#include <Arduino.h>   // the host shim: setMockMillis drives TR_KinematicChecks' clock

#include "BurnoutDetector.h"
#include "GnssAscentGate.h"
#include "ShadowLink.h"
#include "TR_KinematicChecks.h"
#include "landing_transition_policy.h"
#include "raw_nav_adapter.h"
#include "raw_vote_guard.h"

namespace rr
{

// The receiver's fix as the flight loop holds it (gnss_latest_si).
struct FixIn
{
    int64_t t_us = 0;          // when it arrived (MCU)
    uint8_t fix_mode = 0;      // 3: 3-D
    uint8_t num_sats = 0;
    double  alt_m = 0.0;       // above the geoid (the filter's heights, as RawNavAdapter gives them)
    double  vel_n = 0.0, vel_e = 0.0, vel_u = 0.0;
};

class FlightVotes
{
public:
    struct Config
    {
        bool     gate_ekf_valid = false;  // GnssAscentGate may admit on the filter's own altitude (not on the mosaic)
        bool     guard = true;            // the raw vote guard withholds the filter's two votes once latched
        bool     state_from_quorum = false;   // launch and landing from these kinematics, not the IMU ticks' flags
        uint32_t loop_period_us = 1000;   // the flight loop's pass (config::FLIGHT_LOOP_UPDATE_RATE)
        // config.h
        float    mach_on_mps = 260.0f, mach_off_mps = 240.0f;   // BARO_MACH_LOCKOUT_ON/OFF
        uint16_t burnout_neg_hysteresis = 50;                    // BURNOUT_NEG_HYSTERESIS
        uint8_t  gnss_min_sats = 4;                              // GNSS_MIN_SATS
        uint8_t  gnss_min_sats_init = 5;                         // GNSS_MIN_SATS_INIT
        float    gnss_max_vel_init_mps = 2.0f;                   // GNSS_MAX_VEL_INIT_MPS
        uint32_t ref_max_age_ms = 120000;                        // REF_POS_MAX_AGE_MS
    };

    // One voter's flag over the flight: each time it rose (and fell), s (MCU).
    struct Edges
    {
        std::vector<double> on, off;
        bool cur = false;
        void note(bool v, double t)
        {
            if (v != cur)
            {
                (v ? on : off).push_back(t);
                cur = v;
            }
        }
    };

    struct Report
    {
        double launch_s = NAN, kc_launch_s = NAN, burnout_s = NAN, admitted_s = NAN, guard_s = NAN;
        double apogee_s = NAN, backstop_s = NAN, landed_s = NAN;   // landed_s: LANDED (state_from_quorum only)
        int    admitted_reason = 0;   // GnssAscentGate::Reason
        Edges  vel, baro, gps, pitch;
        // at the call: the voters that were available and passed, and what the loop read
        bool   call_vel = false, call_baro = false, call_gps = false, call_pitch = false;
        bool   call_ekf_healthy = false, call_baro_ok = false, call_gps_ok = false;
        float  call_vz = NAN, call_pitch_deg = NAN, call_baro_rate = NAN, call_gps_vu = NAN, call_agl = NAN;
        // the guard, judged whatever Config::guard: when it latched, and the two values then
        float  guard_vz = NAN, guard_baro_rate = NAN;
        uint32_t passes = 0, fixes = 0, ref_count = 0;
        bool   ref_from_nav = false;
        uint32_t mach_lockouts = 0;
    };

    FlightVotes() : FlightVotes(Config{}) {}
    explicit FlightVotes(const Config& c) : cfg_(c)
    {
        kin_.reset();
        GnssAscentGate::reset(gate_);
        RawVoteGuard::reset(guard_);
    }

    // The filter is initialised (main.cpp's ekf_initialized): from here the
    // loop reads its state.
    void setInitialized(bool v) { initialized_ = v; }

    // Each IMU tick the filter is handed (an EKF tick): body FRD, m/s^2, rad/s.
    // in_flight: the tick's own in-flight flag (the launch edge unless
    // state_from_quorum).
    void onImu(int64_t t_us, const float acc[3], const float gyro[3], bool in_flight)
    {
        have_imu_ = true;
        imu_t_us_ = t_us;
        acc_x_ = acc[0];
        acc_norm_ = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        roll_dps_ = gyro[0] * 57.29577951f;
        in_flight_next_ = in_flight;
    }

    // Each barometer sample: its pressure altitude above the pad and its pressure.
    void onBaro(int64_t t_us, float palt_m, float pressure_pa)
    {
        have_baro_ = true;
        baro_t_us_ = t_us;
        palt_m_ = palt_m;
        pressure_pa_ = pressure_pa;
        baro_new_ = true;
    }

    // Each receiver fix (PVTGeodetic, or a sim's), as the loop's gnss_latest_si.
    void onFix(const FixIn& f)
    {
        fix_ = f;
        have_fix_ = true;
        fix_new_ = true;
        fix_id_++;
        rep_.fixes++;
    }

    // Whether the transonic lockout holds (the replay's barometer to the
    // filter is gated by it, as the EKF tick gates it) and whether
    // GnssAscentGate lets the barometer vouch.
    bool machLocked() const { return mach_locked_; }
    bool baroFusable(bool baro_in_range) const { return GnssAscentGate::baroFusable(gate_, baro_in_range); }

    // One flight-loop pass at t_us.
    void pass(int64_t t_us, RawNavAdapter& ekf)
    {
        rep_.passes++;
        const uint32_t now_ms = static_cast<uint32_t>(t_us / 1000);
        setMockMillis(now_ms);
        const double t = t_us * 1.0e-6;
        ekf.refresh(static_cast<uint32_t>(t_us));

        // the launch edge (enterInflight) from the ticks' flags: a log's own state machine
        if (!cfg_.state_from_quorum)
        {
            if (in_flight_next_ && !in_flight_)
                enterInflight(now_ms, t);
            else if (!in_flight_next_ && in_flight_)
                in_flight_ = false;   // landed or back on the pad: the call stays as made
        }

        // the EKF tick's part: the lockout on the filter's speed, its outputs
        if (initialized_)
        {
            float v[3];
            ekf.getVelEst(v);
            const float spd = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            if (!mach_locked_ && spd > cfg_.mach_on_mps)
            {
                mach_locked_ = true;
                rep_.mach_lockouts++;
            }
            else if (mach_locked_ && spd < cfg_.mach_off_mps)
            {
                mach_locked_ = false;
            }
            ekf.getOrientEst(rpy_);
            imu_vel_[0] = v[1];
            imu_vel_[1] = v[0];
            imu_vel_[2] = -v[2];
            double lla[3];
            ekf.getPosEst(lla);
            pos_alt_ = lla[2];
        }

        // the pad reference, once a new fix
        if (fix_new_ && have_fix_ && !ref_frozen_ && fix_.fix_mode >= 3 && fix_.num_sats >= cfg_.gnss_min_sats)
        {
            const double vmag = std::sqrt(fix_.vel_n * fix_.vel_n + fix_.vel_e * fix_.vel_e + fix_.vel_u * fix_.vel_u);
            const bool gate3 = fix_.num_sats >= cfg_.gnss_min_sats_init && vmag < cfg_.gnss_max_vel_init_mps;
            const bool nav_now = initialized_ && ekf.gnssAided();
            if (nav_now && !ref_from_nav_ && gate3)
            {
                ref_from_nav_ = true;
                ref_sum_ = 0.0;
                ref_n_ = 0;
            }
            if (gate3 && (!ref_from_nav_ || nav_now))
            {
                if (ref_n_ == 0)
                    ref_t0_ms_ = now_ms;
                ref_sum_ += ref_from_nav_ ? pos_alt_ : fix_.alt_m;
                ref_n_++;
                ref_alt_ = ref_sum_ / ref_n_;
                have_ref_ = true;
                if (now_ms - ref_t0_ms_ >= cfg_.ref_max_age_ms)
                    ref_frozen_ = true;
            }
        }
        const float imu_pos_up = static_cast<float>(pos_alt_ - ref_alt_);

        // the kinematics block
        const bool baro_in_range = pressure_pa_ >= 25000.0f && pressure_pa_ <= 125000.0f;
        const bool baro_healthy = have_baro_ && baro_in_range && (t_us - baro_t_us_) < 500000;
        const bool ekf_healthy = initialized_ && ekf.isHealthy();
        const bool imu_fresh = have_imu_ && (t_us - imu_t_us_) < 100000;
        const bool apogee_now = kin_.apogee_flag || pyro_apogee_;
        {
            GnssAscentGate::Inputs ga;
            ga.now_ms = now_ms;
            ga.in_flight = in_flight_;
            ga.burnout = burnout_;
            ga.apogee = apogee_now;
            ga.gnss_fix_ok = have_fix_ && fix_.fix_mode >= 3 && fix_.num_sats >= cfg_.gnss_min_sats &&
                             (t_us - fix_.t_us) < 2000000;
            ga.gnss_fix_id = fix_id_;
            ga.gnss_agl_m = static_cast<float>(fix_.alt_m - ref_alt_);
            ga.gnss_vel_u_mps = static_cast<float>(fix_.vel_u);
            ga.baro_healthy = baro_healthy;
            ga.baro_locked = mach_locked_;
            ga.baro_agl_m = palt_m_;
            ga.baro_rate_mps = kin_.d_alt_est_;
            ga.ekf_valid = cfg_.gate_ekf_valid && ekf_healthy;
            ga.ekf_agl_m = imu_pos_up;
            const GnssAscentGate::Phase was = gate_.phase;
            GnssAscentGate::step(gate_, ga);
            if (was != gate_.phase && gate_.phase == GnssAscentGate::Phase::Admitted && std::isnan(rep_.admitted_s))
            {
                rep_.admitted_s = t;
                rep_.admitted_reason = static_cast<int>(gate_.reason);
            }
        }
        {
            RawVoteGuard::Inputs gi;
            gi.now_ms = now_ms;
            gi.after_burnout = burnout_;
            gi.apogee = apogee_now;
            gi.baro_healthy = GnssAscentGate::baroFusable(gate_, baro_healthy);
            gi.mach_locked = mach_locked_;
            gi.filter_healthy = ekf_healthy;
            gi.filter_vz_up_mps = imu_vel_[2];
            gi.baro_rate_mps = kin_.d_alt_est_;
            const bool was = guard_.latched;
            RawVoteGuard::step(guard_, gi);
            if (!was && guard_.latched)
            {
                rep_.guard_s = t;
                rep_.guard_vz = gi.filter_vz_up_mps;
                rep_.guard_baro_rate = gi.baro_rate_mps;
            }
        }
        const bool votes_ok = ekf_healthy && !(cfg_.guard && guard_.latched);
        float pos[3] = {0.0f, 0.0f, imu_pos_up};
        float vel[3] = {imu_vel_[0], imu_vel_[1], imu_vel_[2]};
        const bool had_launch = kin_.launch_flag;
        const bool had_apogee = kin_.apogee_flag;
        const bool had_backstop = kin_.apogee_backstop_flag;
        kin_.kinematicChecks(palt_m_, imu_fresh ? acc_norm_ : 0.0f, pos, vel, roll_dps_, baro_new_,
                             static_cast<float>(fix_.alt_m), fix_new_, rpy_[1], burnout_, mach_locked_,
                             static_cast<float>(fix_.vel_u), votes_ok, baro_healthy, imu_fresh,
                             GnssAscentGate::gnssVotes(gate_));
        baro_new_ = false;
        fix_new_ = false;
        if (!had_launch && kin_.launch_flag)
            rep_.kc_launch_s = t;
        rep_.vel.note(kin_.vel_u_apogee_flag, t);
        rep_.baro.note(kin_.alt_apogee_flag, t);
        rep_.gps.note(kin_.gps_apogee_flag, t);
        rep_.pitch.note(kin_.pitch_apogee_flag, t);
        if (!had_apogee && kin_.apogee_flag)
        {
            rep_.apogee_s = t;
            rep_.call_vel = kin_.vel_u_apogee_flag;
            rep_.call_baro = kin_.alt_apogee_flag;
            rep_.call_gps = kin_.gps_apogee_flag;
            rep_.call_pitch = kin_.pitch_apogee_flag;
            rep_.call_ekf_healthy = votes_ok;
            rep_.call_baro_ok = !mach_locked_ && baro_healthy;
            rep_.call_gps_ok = GnssAscentGate::gnssVotes(gate_) && have_fix_ && (t_us - fix_.t_us) < 500000;
            rep_.call_vz = imu_vel_[2];
            rep_.call_pitch_deg = rpy_[1] * 57.29577951f;
            rep_.call_baro_rate = kin_.d_alt_est_;
            rep_.call_gps_vu = static_cast<float>(fix_.vel_u);
            rep_.call_agl = imu_pos_up;
        }
        if (!had_backstop && kin_.apogee_backstop_flag)
            rep_.backstop_s = t;
        if (kin_.apogee_flag)
            pyro_apogee_ = true;

        // later in the pass, the state machine's switch on this pass's state
        if (cfg_.state_from_quorum && !in_flight_)
        {
            // READY/PRELAUNCH (INITIALIZATION too): the launch flag enters INFLIGHT (enterInflight); LANDED stays
            if (!landed_state_ && kin_.launch_flag)
                enterInflight(now_ms, t);
        }
        else if (in_flight_)
        {
            // INFLIGHT: burnout (the next pass's vote reads it), then the landing's dwell
            if (imu_fresh &&
                tr::burnoutDetectStep(burnout_, burnout_neg_, acc_x_, now_ms - launch_ms_, cfg_.burnout_neg_hysteresis))
                rep_.burnout_s = t;
            if (cfg_.state_from_quorum &&
                landing_transition::step(landed_debounce_, kin_.alt_landed_flag, imu_fresh, roll_dps_, now_ms) ==
                    landing_transition::Action::Land)
            {
                in_flight_ = false;
                landed_state_ = true;
                rep_.landed_s = t;
            }
        }

        rep_.ref_count = ref_n_;
        rep_.ref_from_nav = ref_from_nav_;
    }

    // The flags the raw filter's IMU tick gets from the state after the last
    // pass (rawShadowPushImu), given the tick's own (the events file's):
    // in flight (the quorum's state, else the tick's); levelling while not in
    // flight or once the kinematics have called apogee (use_ahrs_acc); landed
    // on the kinematics' landed flag or the LANDED state (a log's: its tick
    // landed and not in flight); the apogee call (pyro_apogee_detected). Other
    // bits (the sim's) pass through.
    uint8_t shadowFlags(uint8_t tick_flags) const
    {
        using namespace gnss_raw;
        const bool tick_in_flight = (tick_flags & kShadowInFlight) != 0;
        const bool in_flight = cfg_.state_from_quorum ? in_flight_ : tick_in_flight;
        const bool landed_state =
            cfg_.state_from_quorum ? landed_state_ : ((tick_flags & kShadowLanded) != 0 && !tick_in_flight);
        uint8_t f = static_cast<uint8_t>(tick_flags & ~(kShadowLevel | kShadowInFlight | kShadowLanded | kShadowApogee));
        if (in_flight)
            f |= kShadowInFlight;
        if (!in_flight || kin_.apogee_flag)
            f |= kShadowLevel;
        if (kin_.alt_landed_flag || landed_state)
            f |= kShadowLanded;
        if (pyro_apogee_)
            f |= kShadowApogee;
        return f;
    }

    // The next pass due at or after t_us.
    int64_t nextPass(int64_t t_us) const
    {
        const int64_t p = static_cast<int64_t>(cfg_.loop_period_us);
        return (t_us / p + 1) * p;
    }

    const Report& report() const { return rep_; }
    const TR_KinematicChecks& kinematics() const { return kin_; }
    const GnssAscentGate::State& gate() const { return gate_; }
    const RawVoteGuard::State& guard() const { return guard_; }
    bool apogee() const { return pyro_apogee_; }
    bool burnout() const { return burnout_; }
    bool inFlight() const { return in_flight_; }
    bool landed() const { return landed_state_; }
    double refAlt() const { return ref_alt_; }
    const Config& config() const { return cfg_; }

private:
    // enterInflight's part in this: the launch time, burnout, GnssAscentGate's
    // launch, the guard reset, the reference frozen, and the kinematics' and
    // pyro logic's apogee and landing flags cleared (launch_flag kept)
    void enterInflight(uint32_t now_ms, double t)
    {
        in_flight_ = true;
        launch_ms_ = now_ms;
        rep_.launch_s = t;
        burnout_ = false;
        burnout_neg_ = 0;
        GnssAscentGate::onLaunch(gate_, now_ms);
        RawVoteGuard::reset(guard_);
        if (have_ref_)
            ref_frozen_ = true;
        landed_debounce_ = landing_transition::State{};
        kin_.alt_apogee_flag = false;
        kin_.vel_u_apogee_flag = false;
        kin_.gps_apogee_flag = false;
        kin_.pitch_apogee_flag = false;
        kin_.apogee_flag = false;
        kin_.alt_landed_flag = false;
        kin_.max_speed = 0.0f;
        pyro_apogee_ = false;
    }

    Config cfg_;
    Report rep_;
    TR_KinematicChecks kin_;
    GnssAscentGate::State gate_;
    RawVoteGuard::State guard_;
    bool initialized_ = false;
    // the IMU
    bool have_imu_ = false;
    int64_t imu_t_us_ = 0;
    float acc_x_ = 0.0f, acc_norm_ = 0.0f, roll_dps_ = 0.0f;
    bool in_flight_next_ = false, in_flight_ = false;
    bool landed_state_ = false;                     // LANDED (state_from_quorum)
    landing_transition::State landed_debounce_;
    uint32_t launch_ms_ = 0;
    bool burnout_ = false;
    uint16_t burnout_neg_ = 0;
    bool pyro_apogee_ = false;
    bool mach_locked_ = false;
    // the barometer
    bool have_baro_ = false, baro_new_ = false;
    int64_t baro_t_us_ = 0;
    float palt_m_ = 0.0f, pressure_pa_ = 0.0f;
    // the receiver
    FixIn fix_;
    bool have_fix_ = false, fix_new_ = false;
    uint32_t fix_id_ = 0;
    // the filter as the loop reads it
    float rpy_[3] = {0.0f, 1.57f, 0.0f};
    float imu_vel_[3] = {};
    double pos_alt_ = 0.0;
    // the pad reference
    bool ref_from_nav_ = false, ref_frozen_ = false, have_ref_ = false;
    double ref_sum_ = 0.0, ref_alt_ = 0.0;
    uint32_t ref_n_ = 0, ref_t0_ms_ = 0;
};

}  // namespace rr

#endif  // RAW_REPLAY_FLIGHT_VOTES_H
