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
// (within g_tol_mps2). On the ground it levels as before.
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
    };

    static constexpr double kG = 9.80665;

    LevelGate() = default;
    explicit LevelGate(const Config& cfg) : cfg_(cfg) {}

    void reset()
    {
        have_ = false;
        steady_n_ = 0;
    }

    // The filter's velocity (NED, m/s) just after an epoch's update, at t (s).
    void onEpoch(double t, const float v[3])
    {
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

    // In flight: whether this specific force (m/s^2, the body frame) may
    // level the attitude.
    bool allow(const float acc[3]) const
    {
        if (!steady())
            return false;
        const double f = std::sqrt(static_cast<double>(acc[0]) * acc[0] + static_cast<double>(acc[1]) * acc[1]
                                   + static_cast<double>(acc[2]) * acc[2]);
        return std::fabs(f - kG) <= cfg_.g_tol_mps2;
    }

private:
    Config cfg_;
    bool   have_ = false;
    double t_ = 0.0;
    double v_[3] = {};
    int    steady_n_ = 0;
};

}  // namespace gnss_raw

#endif  // LEVEL_GATE_H
