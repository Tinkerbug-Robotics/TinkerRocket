// LevelGate.h — when the raw-GNSS shadow may take its accelerometer as
// gravity in flight (#1600).
//
// The flight filter levels its attitude on the accelerometer on the ground
// and after apogee (main.cpp, use_ahrs_acc), and the shadow followed the same
// flag. Under a parachute that holds only once the descent is steady. On the
// rig replay of rp5_0829 (2026-10-13) the drogue opened with the rocket still
// moving 10-12 m/s sideways: for 5 s the drag pulled 10-45 degrees off
// vertical, its size within 0.4 m/s^2 of g by the end of that. Levelling on it
// turned the shadow's attitude 19 degrees (the flight filter's 31). The
// shadow has its raw epochs once a second, so between them that attitude
// error became 4 m/s of velocity a second, 19 m and 9 m/s off the truth. And
// each time the main opened the drag was vertical but 12-17 m/s^2, which a
// level update puts into the accelerometer bias.
//
// So in flight the shadow levels only once its own velocity, just after each
// raw epoch, has held steady from one epoch to the next steady_epochs times in
// a row (at most accel_max_mps2 between them: the drag at most that far off
// vertical, 3 degrees at 0.5) and the step's specific force is g's size
// (within g_tol_mps2). On the ground it needs only the size (gSized): between
// ignition and the launch detector's verdict the vehicle is still "on the
// pad" with its motor burning -- levelling on that 4 g threw a sim's attitude
// and accelerometer bias on the Beetle (2026-10-13), a 5.5 m/s velocity step
// and a 1.6 m/s^2 error after it.
//
// With no satellites there are no epochs to call the descent steady from, and
// unlevelled a gyro bias tilts the attitude unchecked: half a degree a second
// under a sim's chute on the Beetle put gravity across the tilt and the
// navigation filter 1.5 km east in 40 s. So with no epoch for outage_s, it
// levels once the specific force has been g's size for g_hold_s running: the
// drogue's drag on the rig settled to within 1 degree of vertical about 7 s
// after it opened, its size within 0.4 m/s^2 of g after 2.
//
// The steady test counts epochs, and the Beetle's come once a second. At 10
// Hz (the mosaic-G5) two steady changes would be 0.2 s, a pendulum's turning
// point under a drogue, its drag far off vertical. So with min_dt_s the test
// takes an epoch only once it is that long after the last one it took (0.95 s:
// the 1 Hz test at 10 Hz); every epoch still counts as a sign of satellites
// for the outage. At 0 every epoch is taken, as before.
//
// Pure, header-only: RawGnssShadow drives it, tests_cpp/test_level_gate.cpp
// tests it.
#ifndef LEVEL_GATE_H
#define LEVEL_GATE_H

#include <cmath>

namespace gnss_raw
{

class LevelGate
{
public:
    struct Config
    {
        double accel_max_mps2 = 0.5;   // the velocity's change from one epoch to the next, at most
        int    steady_epochs  = 2;     // that many such changes in a row
        double g_tol_mps2     = 0.5;   // the specific force's size within this of g
        double max_gap_s      = 2.5;   // epochs further apart do not make a change (start over)
        double outage_s       = 2.0;   // no epoch for this long: no satellites
        double g_hold_s       = 5.0;   // then level once the force has been g's size this long
        double min_dt_s       = 0.0;   // the steady test takes epochs at least this far apart (0: every one)
    };

    static constexpr double kG = 9.80665;

    LevelGate() = default;
    explicit LevelGate(const Config& cfg) : cfg_(cfg) {}

    void reset()
    {
        have_ = false;
        seen_ = false;
        steady_n_ = 0;
        g_since_ = -1.0;
    }

    // The filter's velocity (NED, m/s) just after an epoch's update, at t (s).
    void onEpoch(double t, const float v[3])
    {
        seen_ = true;
        t_seen_ = t;
        if (have_ && cfg_.min_dt_s > 0.0 && t >= t_ && t - t_ < cfg_.min_dt_s)
            return;                       // too soon after the last one the test took
        if (have_)
        {
            const double dt = t - t_;
            if (dt > 0.0 && dt <= cfg_.max_gap_s)
            {
                const double dx = v[0] - v_[0], dy = v[1] - v_[1], dz = v[2] - v_[2];
                const double a = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
                steady_n_ = a <= cfg_.accel_max_mps2 ? steady_n_ + 1 : 0;
            }
            else
            {
                steady_n_ = 0;
            }
        }
        have_ = true;
        t_ = t;
        for (int i = 0; i < 3; ++i)
            v_[i] = v[i];
    }

    bool steady() const { return steady_n_ >= cfg_.steady_epochs; }

    // Each step's specific force at t (s): how long it has been g's size.
    void onStep(double t, const float acc[3])
    {
        if (!gSized(acc))
            g_since_ = -1.0;
        else if (g_since_ < 0.0)
            g_since_ = t;
    }

    // This specific force (m/s^2, the body frame) is g's size: gravity, if
    // nothing else is pushing.
    bool gSized(const float acc[3]) const
    {
        const double f = std::sqrt(static_cast<double>(acc[0]) * acc[0] + static_cast<double>(acc[1]) * acc[1]
                                   + static_cast<double>(acc[2]) * acc[2]);
        return std::fabs(f - kG) <= cfg_.g_tol_mps2;
    }

    // In flight, at t: whether this specific force may level the attitude --
    // steady by the last epochs, or with none lately, g's size this long.
    bool allow(double t, const float acc[3]) const
    {
        if (!gSized(acc))
            return false;
        const bool outage = !seen_ || t - t_seen_ > cfg_.outage_s;
        if (!outage)
            return steady();
        return g_since_ >= 0.0 && t - g_since_ >= cfg_.g_hold_s;
    }

private:
    Config cfg_;
    bool   have_ = false;   // the steady test's last epoch: when, and the velocity then
    double t_ = 0.0;
    double v_[3] = {};
    bool   seen_ = false;   // any epoch's (the outage's clock)
    double t_seen_ = 0.0;
    int    steady_n_ = 0;
    double g_since_ = -1.0;   // the force g's size since (s), < 0: not now
};

}  // namespace gnss_raw

#endif  // LEVEL_GATE_H
