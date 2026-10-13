// FixCheck.h — the receiver's own fix against the IMU (#1508): a fix that
// says the vehicle moved in a way it physically did not.
//
// The flight filter takes the receiver's fix as given, and #1508 caught the
// LC86G publishing valid-flagged fixes that were nothing like the flight:
// frozen near the pad while the vehicle fell from 79 km, climbing 4 m/s
// through a 3 g boost, kilometres below the truth after a dropout. Checking a
// fix against another position needs that position to be right; checking it
// against the IMU needs only physics. From the last fix that passed, its
// position and velocity and the IMU's own motion since (the velocity change
// and the displacement it adds, RawGnssEKF::imuMotion) predict where the next
// fix should be and how fast it should be going. Outside the tolerance -- the
// fixes' own noise, their timing (each fix's GPS instant is on the MCU clock
// to some tens of ms, which matters at hundreds of m/s and under thrust), and
// the IMU's error growing with the time since that fix -- a fix is flagged,
// and the next ones are judged from the same last good fix until one agrees
// again. Once that fix is older than the IMU history reaches, a fix cannot be
// judged; it becomes the one the next are judged from.
//
// It sees what is physically impossible: a jump, a frozen fix on a moving
// vehicle, a velocity the specific force does not allow. A fix that is wrong
// by the same amount all along moves as the vehicle does, and passes.
//
// Report-only for now: the flight filter takes every fix as before. Whether a
// flagged fix is kept from the flight filter is the owner's call once flights
// show the check right (#1508).
//
// Pure C++: RawGnssShadow drives it, tests_cpp/test_gnss_checks.cpp tests it.
#ifndef FIX_CHECK_H
#define FIX_CHECK_H

#include <cstdint>

#include "RawMeas.h"

namespace gnss_raw
{

class FixCheck
{
public:
    struct Config
    {
        double pos_m      = 3.0;    // two fixes' position noise, relative to each other, m
        double vel_mps    = 0.6;    // two fixes' velocity noise, m/s
        double jitter_s   = 0.02;   // one fix's time against another's, s (times the speed and acceleration)
        double time_s     = 0.05;   // every fix's time against the IMU's, s: a common lateness, which
                                    // cancels but for the velocity and acceleration that changed
        double acc_err    = 0.5;    // the IMU's acceleration error, m/s^2 ...
        double acc_frac   = 0.035;  // ... plus this share of the specific force (2 degrees of attitude)
        double max_span_s = 4.0;    // judged from a good fix at most this old (the IMU history's reach)
    };

    enum Verdict : uint8_t
    {
        kUnjudged = 0,   // no good fix near enough to judge from (it becomes the next one's)
        kPass = 1,
        kFlagged = 2,
    };

    struct Result
    {
        uint8_t verdict = kUnjudged;
        float   span_s = 0.0f;      // from the good fix it was judged from
        float   pos_err_m = 0.0f, pos_tol_m = 0.0f;
        float   vel_err_mps = 0.0f, vel_tol_mps = 0.0f;
    };

    struct Stats
    {
        uint32_t fixes = 0;         // valid fixes seen
        uint32_t passed = 0, flagged = 0, unjudged = 0;
        uint32_t lapsed = 0;        // runs of flagged fixes that outlived the history (judged no more)
    };

    FixCheck() { reset(); }
    explicit FixCheck(const Config& cfg) : cfg_(cfg) { reset(); }

    void reset();

    // The time (s, the IMU's clock) of the good fix the next is judged from;
    // false if there is none.
    bool anchorTime(double& t) const
    {
        t = anchor_t_;
        return have_anchor_;
    }

    // A valid fix at time t (s, the IMU's clock). dv, dp: the IMU's velocity
    // change and the displacement it adds from the anchor's time to t (NED),
    // or nullptr where the IMU history does not reach. acc: the IMU's
    // acceleration at t (NED, m/s^2), which the fixes' timing is weighed by;
    // nullptr if not known.
    Result onFix(const TimedFix& f, double t, const float* dv, const float* dp, const float* acc);

    const Result& last() const { return last_; }
    const Stats& stats() const { return st_; }

private:
    Config   cfg_;
    bool     have_anchor_ = false;
    double   anchor_t_ = 0.0;
    TimedFix anchor_ = {};
    float    anchor_acc_[3] = {};
    bool     anchor_has_acc_ = false;
    Result   last_;
    Stats    st_;
};

}  // namespace gnss_raw

#endif  // FIX_CHECK_H
