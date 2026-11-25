// gnss_only_arm.h -- the replay's GNSS-only arm (raw_replay --gnss-only-from-s).
//
// On a static or spin-rig log the pad's zero-velocity updates hold the
// velocity to millimetres a second, and its levelling holds the tilt to the
// accelerometer, whatever the receiver gives: they score the pad, not GNSS.
// From the arm's time, on the ground, both are off, and the filter has the
// IMU, GNSS and the barometer, as in flight.
//
// As in flight with act, the arm also takes what act's launch edge does to the
// attitude (RawGnssShadow::stepHeading): the flight gate (heading_gate's: the
// whole attitude from launch, else the heading only), the launch prior on the
// heading (the magnetometer's, else the fallback) and the tilt floor. The
// ground's gate is the EKF config's cos^4 pitch, about 1e-7 on a payload that
// stands nose up, and it scales only the attitude's gain, not the gyro
// bias's: an arm that kept it would leave the tilt to the gyro alone while
// GNSS still moved the bias. A restart on the pad (a stop far from the seed)
// puts the ground's gate back, so the arm takes the launch edge again after
// each one. With observe or off the launch edge leaves the gate alone, and so
// does the arm. kGateGround keeps the ground's gate: E4's first arm, before
// the 10/17 review, for comparison.
//
// With keep_level (raw_replay --gnss-only-keep-level) the levelling stays on:
// the descent under a chute as flown, where the shadow levels once the drag is
// steady and g's size (LevelGate.h). A payload turning steadily on the spin
// rig passes that test throughout, so the ground's size test stands for it.
// No pad ZUPT, act's launch edge on the attitude, the levelling and no
// magnetometer (raw_replay hands none from the switch on: in flight the
// filter takes none): what the filter has under a slowly turning drogue, not
// GNSS alone.
//
// It reaches into RawGnssShadow (its config, its EKF, setFlightGate): include
// RawGnssShadow.h with its private members open first, as raw_replay.cpp
// does. Pure C++: tests_cpp/test_raw_replay_gnss_only.cpp.
#ifndef RAW_REPLAY_GNSS_ONLY_ARM_H
#define RAW_REPLAY_GNSS_ONLY_ARM_H

#include <cstdint>

#include "RawGnssShadow.h"

namespace rr
{

class GnssOnlyArm
{
public:
    enum Gate : uint8_t
    {
        kGateGround = 0,   // the ground's gate kept
        kGateLaunch = 1,   // act's launch edge: the flight gate, the heading prior, the tilt floor (the default)
    };

    GnssOnlyArm(double from_s, Gate gate, bool keep_level = false)
        : from_s_(from_s), gate_(gate), keep_level_(keep_level) {}

    bool asked() const { return from_s_ < 1.0e8; }
    bool on() const { return on_; }
    Gate gate() const { return gate_; }
    bool keepLevel() const { return keep_level_; }
    // how many times the arm took act's launch edge (the switch, then each restart)
    int launches() const { return launches_; }
    // the heading prior it set last, deg (-1: none, observe or off)
    double priorDeg() const { return prior_deg_; }

    // One IMU tick at flight time t_flight_s, before the shadow steps on it
    // (t_s: the shadow's own clock, the tick's MCU time): the tick's flags as
    // the shadow should see them. In flight it does nothing: the launch edge
    // there is the shadow's own.
    uint8_t tick(gnss_raw::RawGnssShadow& sh, double t_flight_s, double t_s, uint8_t flags)
    {
        if (!asked() || t_flight_s < from_s_ || (flags & gnss_raw::kShadowInFlight))
            return flags;
        if (!on_)
        {
            on_ = true;
            sh.cfg_.pad_zupt = false;
        }
        if (gate_ == kGateLaunch && sh.running() && sh.stats().starts != starts_seen_)
        {
            starts_seen_ = sh.stats().starts;
            launches_++;
            prior_deg_ = launchEdge(sh, t_s);
        }
        return keep_level_ ? flags : static_cast<uint8_t>(flags & ~gnss_raw::kShadowLevel);
    }

    // What act's launch edge does to the attitude (RawGnssShadow::stepHeading),
    // at the shadow's time t_s: the heading prior it set (deg), -1 where the
    // heading mode is not act and the edge leaves the gate the config's.
    static double launchEdge(gnss_raw::RawGnssShadow& sh, double t_s)
    {
        const gnss_raw::RawGnssShadow::Config& c = sh.cfg_;
        if (c.heading_mode != gnss_raw::RawGnssShadow::kHeadingAct)
            return -1.0;
        sh.setFlightGate(c.heading_gate == gnss_raw::RawGnssShadow::kGateFullLaunch);
        const bool mag_heading = c.primary && sh.mag_fused_run_ > 0 &&
                                 t_s - sh.last_mag_fused_t_ <= c.heading_mag_recent_s;
        // none fused at all: any heading, and the fit's hand-off armed until the apogee call
        sh.hdg_unset_ = c.primary && sh.mag_fused_run_ == 0;
        sh.hdg_apogee_ = false;
        const double prior = mag_heading     ? c.heading_pad_sigma_deg
                             : sh.hdg_unset_ ? c.heading_pad_sigma_unset_deg
                                             : c.heading_pad_sigma_fallback_deg;
        sh.ekf_.setHeadingSigma(static_cast<float>(prior / kR2D));
        if (c.heading_launch_tilt_sigma_deg > 0.0)
            sh.ekf_.setTiltSigmaFloor(static_cast<float>(c.heading_launch_tilt_sigma_deg / kR2D));
        return prior;
    }

private:
    static constexpr double kR2D = 57.29577951308232;
    double from_s_;
    Gate gate_;
    bool keep_level_;
    bool on_ = false;
    uint32_t starts_seen_ = 0;
    int launches_ = 0;
    double prior_deg_ = -1.0;
};

}  // namespace rr

#endif  // RAW_REPLAY_GNSS_ONLY_ARM_H
