// RawGnssWindup.h -- the carrier wind-up a turning antenna puts on every
// Doppler and carrier phase, from the gyro: its model, the correction's
// pieces, and the roll-rate monitor (the V9 + mosaic-G5 at high roll rates,
// gnss-attitude-study/v9-mosaic-filter-plan/evidence/windup).
//
// The model. A right-hand circularly polarized antenna turning about its own
// boresight b by an angle phi puts -phi / 2pi cycles on the carrier phase of
// every satellite it tracks, range-like (Wu et al. 1993: the phase range
// grows by -lambda phi / 2pi). On the Goblin's helix the spin rig measured
// +0.991 and +0.994 cycles per clockwise turn seen from above, nose up (10/16,
// the clock-free geometry-free combination, every band within 0.5 %): a
// clockwise turn from above is -1 turn about the boresight (up), so k = 0.99
// cycles per turn. In rate form, for a body turning at w (rad/s) and a
// satellite along e (unit, receiver to satellite), both in the body frame:
//
//   dW/dt = -k (lambda / 2pi) w.(b + e) / (1 + b.e)          [m/s, range-like]
//
// (checked against the RTKLIB form of Wu over 4,876 random geometries, worst
// relative error 1.7e-5). Roll alone, w = p b, gives -k lambda p / 2pi on
// every satellite in front of or behind the antenna alike: common to all of
// one wavelength, so a receiver clock's drift state takes it -- but not across
// wavelengths: BeiDou B1I's is 0.92 % longer than L1/E1's, 1.7 mm/s per rev/s
// left on each BeiDou Doppler. The rate across the boresight adds a part each
// satellite has its own: -k (lambda / 2pi) w_perp.e / (1 + b.e), at most
// (lambda / 2pi) |w_perp| tan(theta / 2), theta off the boresight -- 30 mm/s
// per rad/s at 90 deg, a tenth of what an antenna 0.3 m up the nose moves at
// that rate. At the null directly behind (b.e = -1) the model has no limit;
// an antenna has no gain there, and the denominator is held at den_min.
//
// Units and sign as the filter's measurements have them: the mosaic's range
// rate is rate_mps = -lambda * Doppler (MosaicRawEpoch.h), so it carries
// dW/dt as written, and the correction adds its negative; a carrier rate's
// phase change (cr_dphi_m) carries -k lambda dphi / 2pi over its interval.
// The boresight is the body's +x, the nose: the Goblin's helix sits in the
// nose cone. p is the gyro's x less its bias, right-handed about the nose.
//
// The monitor. The wind-up is a second gyro, about the boresight only:
//   - two bands of one satellite (geometry-free and clock-free): rr_a - rr_b
//     = -k (lambda_a - lambda_b) p / 2pi, plus the ionosphere's rate
//     (gfRoll); L1-L5 is a 64.5 mm lever, 64.5 mm/s per rev/s. Not on board
//     yet: the mosaic adapter builds one signal per satellite;
//   - one band per satellite, two wavelengths in the sky (InterbandFit): the
//     epoch's Dopplers fitted for a velocity, a clock rate and the roll rate
//     whose wind-up differs between the wavelengths (BeiDou against GPS and
//     Galileo) -- 1.7 mm/s per rev/s, so about 9 rev/s 1-sigma an epoch with
//     a dozen satellites: only a gross disagreement shows.
// Monitor keeps a leaky weighted mean of what an epoch's estimate says less
// the gyro, and a leaky regression of one on the other (the gyro's scale, as
// the wind-up sees it).
//
// Pure, header-only and host-compilable (tests_cpp/test_raw_gnss_windup.cpp).
#ifndef RAW_GNSS_WINDUP_H
#define RAW_GNSS_WINDUP_H

#include <cmath>

namespace gnss_raw
{
namespace windup
{

constexpr double kTwoPi = 6.283185307179586;

// Roll alone: the range-like wind-up rate (m/s) on a carrier of wavelength
// lambda_m while the body rolls at p_rps about the boresight, k cycles a turn.
inline double rollRate(double lambda_m, double k, double p_rps)
{
    return -k * lambda_m * p_rps / kTwoPi;
}

// Wu's in full, boresight +x: w (rad/s) and e (unit, receiver to satellite),
// body frame. den_min holds 1 + b.e off the null behind the antenna.
inline double wuRate(double lambda_m, double k, const double w[3], const double e[3], double den_min = 0.1)
{
    double den = 1.0 + e[0];
    if (den < den_min)
        den = den_min;
    return -k * lambda_m / kTwoPi * (w[0] * (1.0 + e[0]) + w[1] * e[1] + w[2] * e[2]) / den;
}

// The roll rate (rad/s) one satellite's two Dopplers imply, a on lambda_a and
// b on lambda_b (m/s, range rates), k cycles a turn: geometry- and clock-free.
inline double gfRoll(double rr_a, double lambda_a, double rr_b, double lambda_b, double k)
{
    return -kTwoPi * (rr_a - rr_b) / (k * (lambda_a - lambda_b));
}

// and its 1-sigma from the two Dopplers' (m/s)
inline double gfRollSigma(double s_a, double lambda_a, double s_b, double lambda_b, double k)
{
    return kTwoPi * std::sqrt(s_a * s_a + s_b * s_b) / std::fabs(k * (lambda_a - lambda_b));
}

// The gyro's steps, newest last: each its end time (the filter's clock, s)
// and its mean rate over the step since the one before (body, rad/s). The
// correction reads the rate at a Doppler's own instant, up to an epoch's
// largest age before the step that fuses it.
template <int Cap>
class RateRing
{
public:
    void reset() { n_ = head_ = 0; }
    void push(double t_end, const float w[3])
    {
        Entry& e = buf_[head_];
        e.t = t_end;
        e.w[0] = w[0];
        e.w[1] = w[1];
        e.w[2] = w[2];
        head_ = (head_ + 1) % Cap;
        if (n_ < Cap)
            n_++;
    }
    int size() const { return n_; }
    // The step that holds t (its span is (the one before's end, its end]):
    // its rate. Past the newest, the newest's; false before the oldest step's
    // span (its start is not kept: the oldest itself is not used either).
    bool rateAt(double t, float w[3]) const
    {
        if (n_ < 2)
            return false;
        const Entry& newest = at(n_ - 1);
        if (t > newest.t)
        {
            copy(newest, w);
            return true;
        }
        for (int k = n_ - 1; k >= 1; --k)
            if (t > at(k - 1).t)
            {
                copy(at(k), w);
                return true;
            }
        return false;
    }
    // The roll angle (about x, rad) turned from t0 to t1 (t0 <= t1): each
    // step's rate times its share of the interval. False unless the steps
    // cover it.
    bool rollOver(double t0, double t1, double& dphi) const
    {
        dphi = 0.0;
        if (n_ < 2 || !(t1 >= t0) || t0 < at(0).t)
            return false;
        const Entry& newest = at(n_ - 1);
        for (int k = 1; k < n_; ++k)
        {
            const double s0 = at(k - 1).t, s1 = at(k).t;
            const double lo = s0 > t0 ? s0 : t0, hi = s1 < t1 ? s1 : t1;
            if (hi > lo)
                dphi += static_cast<double>(at(k).w[0]) * (hi - lo);
        }
        if (t1 > newest.t)
            dphi += static_cast<double>(newest.w[0]) * (t1 - (t0 > newest.t ? t0 : newest.t));
        return true;
    }

private:
    struct Entry
    {
        double t;
        float  w[3];
    };
    const Entry& at(int k) const { return buf_[(head_ + Cap - n_ + k) % Cap]; }
    static void copy(const Entry& e, float w[3])
    {
        w[0] = e.w[0];
        w[1] = e.w[1];
        w[2] = e.w[2];
    }
    Entry buf_[Cap] = {};
    int   n_ = 0, head_ = 0;
};

// One epoch's Dopplers, single band per satellite, fitted for a velocity
// correction (3, NED), a clock rate and a roll rate p (rad/s): each row's
// innovation y (m/s) at the filter's prior, its line of sight u (NED,
// receiver to satellite), its 1-sigma s and its wind-up column h -- the
// wind-up rate per rad/s of roll it carries beyond the reference
// wavelength's: -k (lambda - lambda_ref) / 2pi (0 on the reference's). The
// rows are summed into the normal equations as they come (no arrays: the
// flight computer's task has little stack). solve: false under six rows, for
// a sky whose rows all have one h (no roll in it to see), or if the geometry
// gives no velocity; else p, the roll rate the rows' wind-up says beyond
// whatever was already taken off them, and its 1-sigma.
class InterbandFit
{
public:
    static constexpr double kHScale = 1.0e-3;   // h's size (m/s per rad/s), for the pivots to compare
    void reset()
    {
        for (int a = 0; a < 5; ++a)
            for (int b = 0; b < 6; ++b)
                M_[a][b] = 0.0;
        n_ = 0;
        hmin_ = 1.0e300;
        hmax_ = -1.0e300;
    }
    InterbandFit() { reset(); }
    void add(const double u[3], double y, double s, double h)
    {
        if (!(s > 0.0) || !std::isfinite(y))
            return;
        const double row[5] = {-u[0], -u[1], -u[2], 1.0, h / kHScale};
        const double w = 1.0 / (s * s);
        for (int a = 0; a < 5; ++a)
        {
            for (int b = 0; b < 5; ++b)
                M_[a][b] += w * row[a] * row[b];
            M_[a][5] += w * row[a] * y;
        }
        n_++;
        if (h < hmin_)
            hmin_ = h;
        if (h > hmax_)
            hmax_ = h;
    }
    int  rows() const { return n_; }
    bool solve(double& p, double& sigma) const
    {
        p = sigma = 0.0;
        if (n_ < 6 || !(hmax_ > hmin_))
            return false;
        // [N | b | I] -> [I | x | N^-1], Gauss-Jordan with partial pivoting
        double A[5][11];
        double scale = 0.0;
        for (int a = 0; a < 5; ++a)
        {
            scale = std::fmax(scale, std::fabs(M_[a][a]));
            for (int b = 0; b < 6; ++b)
                A[a][b] = M_[a][b];
            for (int b = 0; b < 5; ++b)
                A[a][6 + b] = a == b ? 1.0 : 0.0;
        }
        for (int c = 0; c < 5; ++c)
        {
            int piv = c;
            for (int r = c + 1; r < 5; ++r)
                if (std::fabs(A[r][c]) > std::fabs(A[piv][c]))
                    piv = r;
            if (!(std::fabs(A[piv][c]) > 1e-12 * scale))
                return false;
            if (piv != c)
                for (int k = 0; k < 11; ++k)
                {
                    const double t = A[c][k];
                    A[c][k] = A[piv][k];
                    A[piv][k] = t;
                }
            const double d = A[c][c];
            for (int k = 0; k < 11; ++k)
                A[c][k] /= d;
            for (int r = 0; r < 5; ++r)
            {
                if (r == c)
                    continue;
                const double f = A[r][c];
                if (f == 0.0)
                    continue;
                for (int k = 0; k < 11; ++k)
                    A[r][k] -= f * A[c][k];
            }
        }
        const double var = A[4][10];
        if (!(var > 0.0) || !std::isfinite(A[4][5]))
            return false;
        p = A[4][5] / kHScale;
        sigma = std::sqrt(var) / kHScale;
        return true;
    }

private:
    double M_[5][6];
    int    n_ = 0;
    double hmin_ = 0.0, hmax_ = 0.0;
};

// What the wind-up says against the gyro, over time: each estimate p_wu
// (rad/s, 1-sigma sigma) beside the gyro's p_gyro at the same instant, into a
// leaky weighted mean of their difference (tau_s) and a leaky regression of
// p_wu on p_gyro through the origin (the gyro's scale as the wind-up sees it:
// 1 for a gyro and an antenna that agree; under 1 once the gyro saturates).
class Monitor
{
public:
    explicit Monitor(double tau_s = 1.0) : tau_(tau_s) {}
    void setTau(double tau_s) { tau_ = tau_s; }
    void reset() { sw_ = swd_ = sxy_ = sxx_ = 0.0; t_ = -1.0e300; n_ = 0; }
    void add(double t, double p_wu, double sigma, double p_gyro)
    {
        if (!(sigma > 0.0) || !std::isfinite(p_wu))
            return;
        const double lam = t > t_ && tau_ > 0.0 ? std::exp(-(t - t_) / tau_) : (tau_ > 0.0 ? 1.0 : 0.0);
        const double w = 1.0 / (sigma * sigma);
        sw_ = lam * sw_ + w;
        swd_ = lam * swd_ + w * (p_wu - p_gyro);
        sxy_ = lam * sxy_ + w * p_gyro * p_wu;
        sxx_ = lam * sxx_ + w * p_gyro * p_gyro;
        t_ = t;
        n_++;
    }
    int    count() const { return n_; }
    // the leaky mean of p_wu - p_gyro (rad/s) and its 1-sigma
    double diff() const { return sw_ > 0.0 ? swd_ / sw_ : 0.0; }
    double diffSigma() const { return sw_ > 0.0 ? 1.0 / std::sqrt(sw_) : 0.0; }
    // the gyro's scale as the wind-up sees it, and its 1-sigma (0, 0 with no roll yet)
    double scale() const { return sxx_ > 0.0 ? sxy_ / sxx_ : 0.0; }
    double scaleSigma() const { return sxx_ > 0.0 ? 1.0 / std::sqrt(sxx_) : 0.0; }

private:
    double tau_;
    double sw_ = 0.0, swd_ = 0.0, sxy_ = 0.0, sxx_ = 0.0;
    double t_ = -1.0e300;
    int    n_ = 0;
};

}  // namespace windup
}  // namespace gnss_raw

#endif  // RAW_GNSS_WINDUP_H
