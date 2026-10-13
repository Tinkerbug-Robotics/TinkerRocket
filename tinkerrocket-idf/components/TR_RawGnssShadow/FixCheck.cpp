// FixCheck.cpp — see FixCheck.h
#include "FixCheck.h"

#include <cmath>

namespace gnss_raw
{

namespace
{

constexpr double kA  = 6378137.0;           // WGS84
constexpr double kE2 = 0.0066943799901;
constexpr double kG0 = 9.807;               // the filters' gravity

double norm3(const double v[3])
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

}  // namespace

void FixCheck::reset()
{
    have_anchor_ = false;
    anchor_t_ = 0.0;
    anchor_ = TimedFix();
    anchor_has_acc_ = false;
    last_ = Result();
    st_ = Stats();
}

FixCheck::Result FixCheck::onFix(const TimedFix& f, double t, const float* dv, const float* dp,
                                 const float* acc)
{
    st_.fixes++;
    Result r;
    const double span = t - anchor_t_;
    auto anchorOn = [&]() {
        have_anchor_ = true;
        anchor_t_ = t;
        anchor_ = f;
        anchor_has_acc_ = acc != nullptr;
        for (int i = 0; i < 3; ++i)
            anchor_acc_[i] = acc != nullptr ? acc[i] : 0.0f;
    };
    if (!have_anchor_ || dv == nullptr || dp == nullptr || !(span > 0.0) || span > cfg_.max_span_s)
    {
        // nothing near enough to judge it from: it is the next one's anchor
        if (last_.verdict == kFlagged)
            st_.lapsed++;
        st_.unjudged++;
        anchorOn();
        last_ = r;
        return r;
    }

    // the fix from the anchor, NED (the anchor's curvature: metres-exact over km)
    const double s = std::sin(anchor_.lat_rad);
    const double w = std::sqrt(1.0 - kE2 * s * s);
    const double rn = kA * (1.0 - kE2) / (w * w * w) + anchor_.alt_m;
    const double re = kA / w + anchor_.alt_m;
    double dlon = f.lon_rad - anchor_.lon_rad;
    if (dlon > M_PI)
        dlon -= 2.0 * M_PI;
    else if (dlon < -M_PI)
        dlon += 2.0 * M_PI;
    const double p_fix[3] = {(f.lat_rad - anchor_.lat_rad) * rn, dlon * re * std::cos(anchor_.lat_rad),
                             -(f.alt_m - anchor_.alt_m)};

    // what the IMU says, from the anchor's position and velocity
    double e_p[3], e_v[3], va[3], dvd[3];
    for (int i = 0; i < 3; ++i)
    {
        va[i] = anchor_.vel_ned[i];
        dvd[i] = dv[i];
        const double p_pred = va[i] * span + dp[i];
        e_p[i] = p_fix[i] - p_pred;
        e_v[i] = static_cast<double>(f.vel_ned[i]) - (va[i] + dvd[i]);
    }

    // The tolerance: the fixes' noise and timing, and the IMU's error growing
    // with the span (an acceleration error, part of it the attitude's share of
    // the specific force). A fix late by t_c reads v(t + t_c): a lateness
    // every fix shares cancels from one to the next but for what changed in
    // between (the velocity, times t_c; the acceleration, at ignition or
    // burnout, times t_c), and one fix's jitter against another's costs the
    // speed and the acceleration times that.
    double a[3], sf[3], da[3];
    for (int i = 0; i < 3; ++i)
        a[i] = dvd[i] / span;
    sf[0] = a[0];
    sf[1] = a[1];
    sf[2] = a[2] - kG0;
    double a_now = norm3(a), da_n = 2.0 * a_now;   // without both ends' acceleration: any change up to twice it
    if (acc != nullptr && anchor_has_acc_)
    {
        for (int i = 0; i < 3; ++i)
            da[i] = static_cast<double>(acc[i]) - static_cast<double>(anchor_acc_[i]);
        da_n = norm3(da);
        const double an[3] = {acc[0], acc[1], acc[2]}, aa[3] = {anchor_acc_[0], anchor_acc_[1], anchor_acc_[2]};
        a_now = std::fmax(norm3(an), norm3(aa));
    }
    const double acc_err = cfg_.acc_err + cfg_.acc_frac * norm3(sf);
    const double jit = std::sqrt(2.0) * cfg_.jitter_s;
    const double tol_p = cfg_.pos_m + jit * norm3(va) + cfg_.time_s * norm3(dvd) + 0.5 * acc_err * span * span;
    const double tol_v = cfg_.vel_mps + jit * a_now + cfg_.time_s * da_n + acc_err * span;

    r.span_s = static_cast<float>(span);
    r.pos_err_m = static_cast<float>(norm3(e_p));
    r.vel_err_mps = static_cast<float>(norm3(e_v));
    r.pos_tol_m = static_cast<float>(tol_p);
    r.vel_tol_mps = static_cast<float>(tol_v);
    if (r.pos_err_m <= r.pos_tol_m && r.vel_err_mps <= r.vel_tol_mps)
    {
        r.verdict = kPass;
        st_.passed++;
        anchorOn();
    }
    else
    {
        r.verdict = kFlagged;
        st_.flagged++;
    }
    last_ = r;
    return r;
}

}  // namespace gnss_raw
