// TR_RawGnssEKF.cpp — see TR_RawGnssEKF.h
#include "TR_RawGnssEKF.h"

#include <cmath>
#include <cstring>

#include "RawGnssEKFGeometry.h"
#include "gnss/types.h"

namespace gnss_raw
{

namespace
{

// the geometry every update predicts with (RawGnssEKFGeometry.h, shared with
// TR_RawGnssCarrier.cpp's once-an-epoch code)
using ekf_geom::kC;
using ekf_geom::kOmegaE;
using ekf_geom::Los;
using ekf_geom::lineOfSight;
using ekf_geom::satRangeAccel;
using ekf_geom::corrInflation;
using ekf_geom::backDate;

constexpr double kRe     = 6378137.0;           // m
constexpr double kE2     = 0.0066943799901;     // as TR_GpsInsEKF and the prototype
constexpr double kG0     = 9.807;               // the flight filter's G_NOMINAL
constexpr double kNever  = -1.0e9;              // no update yet

// WGS84 normal gravity (gnss repo tc_ekf.py gravity_wgs84)
constexpr double kGammaE = 9.7803253359;
constexpr double kSomK   = 0.00193185265241;
constexpr double kFlat   = 1.0 / 298.257223563;
constexpr double kGravM  = 0.00344978650684;

void earthRad(double lat, double& rew, double& rns)
{
    const double s = std::sin(lat);
    const double d = std::fabs(1.0 - kE2 * s * s);
    rew = kRe / std::sqrt(d);
    rns = kRe * (1.0 - kE2) / (d * std::sqrt(d));
}

// numpy's median: the mean of the two middle values for an even count.
double median(double* v, int n)
{
    for (int i = 1; i < n; ++i)
    {
        const double x = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > x)
        {
            v[j + 1] = v[j];
            --j;
        }
        v[j + 1] = x;
    }
    return (n % 2) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

template <typename T>
void quat2dcm(const T q[4], T M[3][3])
{
    const T w = q[0], x = q[1], y = q[2], z = q[3];
    const T one = T(1), two = T(2);
    M[0][0] = one - two * (y * y + z * z); M[0][1] = two * (x * y + w * z);       M[0][2] = two * (x * z - w * y);
    M[1][0] = two * (x * y - w * z);       M[1][1] = one - two * (x * x + z * z); M[1][2] = two * (y * z + w * x);
    M[2][0] = two * (x * z + w * y);       M[2][1] = two * (y * z - w * x);       M[2][2] = one - two * (x * x + y * y);
}

template <typename T>
void quatMult(const T a[4], const T b[4], T o[4])
{
    o[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    o[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    o[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    o[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

template <typename T>
void normalize4(T q[4])
{
    const T n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int i = 0; i < 4; ++i)
        q[i] /= n;
}

}  // namespace

template <typename T>
RawGnssEKF<T>::RawGnssEKF(const RawGnssEKFParams& p)
{
    setParams(p);
    for (int s = 0; s < 3; ++s)
        for (int k = 0; k < 64; ++k)
            last_upd_[s][k][0] = last_upd_[s][k][1] = kNever;
}

template <typename T>
void RawGnssEKF<T>::setParams(const RawGnssEKFParams& p)
{
    p_ = p;
    q_acc_  = T(p.a_noise * p.a_noise);
    q_att_  = T(0.25 * p.w_noise * p.w_noise);                // G = -0.5 I on the attitude
    q_ab_   = T(2.0 * p.a_markov_sigma * p.a_markov_sigma / p.a_markov_tau);
    q_wb_   = T(2.0 * p.w_markov_sigma * p.w_markov_sigma / p.w_markov_tau);
    f_ab_   = T(-1.0 / p.a_markov_tau);
    f_wb_   = T(-1.0 / p.w_markov_tau);
    q_clkb_ = T(p.clk_bias_psd);
    q_clkd_ = T(p.clk_drift_psd);
    q_isb_  = T(p.isb_psd);
}

template <typename T>
void RawGnssEKF<T>::init(const double lla[3], const T v_ned[3], const T q[4], double t0)
{
    for (int i = 0; i < 3; ++i)
    {
        lla_[i] = lla[i];
        v_[i] = v_ned[i];
        ab_[i] = wb_[i] = a_ned_[i] = T(0);
    }
    for (int i = 0; i < 4; ++i)
        q_[i] = q[i];
    normalize4(q_);
    clk_b_ = 0.0;
    clk_d_ = T(0);
    isb_[0] = isb_[1] = T(0);
    clk_ready_ = false;
    g_excess_ = g_peak_ = T(0);
    rate_peak_ = rate_across_peak_ = T(0);
    att_hold_ = false;
    t_ = t0;

    std::memset(P_, 0, sizeof(P_));
    const double d[N] = {
        p_.p0_pos, p_.p0_pos, p_.p0_pos, p_.p0_vel, p_.p0_vel, p_.p0_vel,
        p_.p0_att, p_.p0_att, p_.p0_att, p_.p0_abias, p_.p0_abias, p_.p0_abias,
        p_.p0_wbias, p_.p0_wbias, p_.p0_wbias, p_.p0_clk_bias, p_.p0_clk_drift,
        p_.p0_isb, p_.p0_isb};
    for (int i = 0; i < N; ++i)
        P_[i][i] = T(d[i] * d[i]);
    // The attitude: p0_att about the two horizontal axes, p0_hdg about the
    // vertical -- NED down in the body frame, at the attitude it starts from
    // (#1626 N10): setHeadingSigma on p0_att's sphere (a rotation's sigma,
    // twice the half-angle's). On body z, as it was, a nose-up rocket's "any
    // heading" lay across it, a tilt the levelling then took, and its real
    // heading had the tilt's p0_att. A level start (q = 1) is what it was. So
    // q must be the best attitude there is, not a placeholder: the navigation
    // filter's pad seed is the accelerometer's tilt (main.cpp's
    // quatFromAccelHeading), and the levelling moves it a fraction of a
    // degree. (A start degrees off gave some of the heading's variance to the
    // levelling as the linearization moved -- 3 deg: 57 deg of heading was 41
    // after 10 s -- until the levelling was made heading-free,
    // level_heading_free.) A body far off its start, an identity start on a
    // nose-up rocket, would leave the heading p0_att's, as before.
    setHeadingSigma(T(2.0 * p_.p0_hdg));

    hhead_ = hn_ = 0;
    hopen_ = t0;
    std::memset(rejects_, 0, sizeof(rejects_));
    for (int s = 0; s < 3; ++s)
        for (int k = 0; k < 64; ++k)
            last_upd_[s][k][0] = last_upd_[s][k][1] = kNever;
    // the carrier: no receiver kept, nothing screened
    cr_cur_.valid = cr_prev_.valid = false;
    cr_ninfo_ = cr_nfuse_ = 0;
    cr_roll_sum_ = cr_roll_t_ = 0.0;
    for (int i = 0; i < 3; ++i)
        cr_ex_[i] = 0.0;
}

template <typename T>
void RawGnssEKF<T>::setQuaternion(const T q[4])
{
    for (int i = 0; i < 4; ++i)
        q_[i] = q[i];
    normalize4(q_);
    for (int i = kAtt; i < kAtt + 3; ++i)
        for (int j = 0; j < N; ++j)
            P_[i][j] = P_[j][i] = T(0);
    for (int i = kAtt; i < kAtt + 3; ++i)
        P_[i][i] = T(1e-6);
}

template <typename T>
void RawGnssEKF<T>::seedGyroBias(const T gyro_mean[3])
{
    for (int i = 0; i < 3; ++i)
        wb_[i] = gyro_mean[i];
}

template <typename T>
void RawGnssEKF<T>::setHeadingSigma(T sigma_rad)
{
    T M[3][3];
    quat2dcm(q_, M);
    const T e[3] = {M[0][2], M[1][2], M[2][2]};       // the vertical in the body frame
    // the attitude rows and columns without their part about e (Q = I - e e^T)
    for (int j = 0; j < N; ++j)
    {
        if (j >= kAtt && j < kAtt + 3)
            continue;
        const T d = e[0] * P_[kAtt][j] + e[1] * P_[kAtt + 1][j] + e[2] * P_[kAtt + 2][j];
        for (int a = 0; a < 3; ++a)
            P_[kAtt + a][j] = P_[j][kAtt + a] = P_[kAtt + a][j] - e[a] * d;
    }
    T B[3][3], QB[3][3];
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
            B[a][b] = P_[kAtt + a][kAtt + b];
    for (int b = 0; b < 3; ++b)
    {
        const T d = e[0] * B[0][b] + e[1] * B[1][b] + e[2] * B[2][b];
        for (int a = 0; a < 3; ++a)
            QB[a][b] = B[a][b] - e[a] * d;
    }
    // the state is the half-angle's vector part: a rotation sigma is sigma/2 there
    const T s2 = T(0.25) * sigma_rad * sigma_rad;
    for (int a = 0; a < 3; ++a)
    {
        const T d = QB[a][0] * e[0] + QB[a][1] * e[1] + QB[a][2] * e[2];
        for (int b = 0; b < 3; ++b)
            P_[kAtt + a][kAtt + b] = QB[a][b] - d * e[b] + s2 * e[a] * e[b];
    }
    symmetrize();
}

template <typename T>
void RawGnssEKF<T>::setTiltSigmaFloor(T sigma_rad)
{
    if (!(sigma_rad > T(0)))
        return;
    T M[3][3];
    quat2dcm(q_, M);                                   // NED -> body: columns 0 and 1 are north and east in it
    // the state is the half-angle's vector part: a rotation sigma is sigma/2 there
    const T floor2 = T(0.25) * sigma_rad * sigma_rad;
    // north, then east (the one's deficit leaves the other's variance as it was)
    for (int k = 0; k < 2; ++k)
    {
        T v = T(0);
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b)
                v += M[a][k] * P_[kAtt + a][kAtt + b] * M[b][k];
        const T d = floor2 - v;
        if (d > T(0))
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b)
                    P_[kAtt + a][kAtt + b] += d * (M[a][k] * M[b][k]);   // (a, b) and (b, a) alike: P stays symmetric
    }
}

template <typename T>
void RawGnssEKF<T>::turnHeading(T dpsi)
{
    // body -> NED, then about NED down: q' = qz(dpsi) q
    const T qz[4] = {std::cos(T(0.5) * dpsi), T(0), T(0), std::sin(T(0.5) * dpsi)};
    T qn[4];
    quatMult(qz, q_, qn);
    for (int i = 0; i < 4; ++i)
        q_[i] = qn[i];
    normalize4(q_);
}

template <typename T>
void RawGnssEKF<T>::setBiases(const T ab[3], const T wb[3], const T Pb[6][6])
{
    for (int i = 0; i < 3; ++i)
    {
        ab_[i] = ab[i];
        wb_[i] = wb[i];
    }
    for (int i = kAccBias; i < kAccBias + 6; ++i)
        for (int j = 0; j < N; ++j)
            P_[i][j] = P_[j][i] = T(0);
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            P_[kAccBias + i][kAccBias + j] = Pb[i][j];
    symmetrize();
}

template <typename T>
void RawGnssEKF<T>::biasCovariance(T Pb[6][6]) const
{
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            Pb[i][j] = P_[kAccBias + i][kAccBias + j];
}

template <typename T>
void RawGnssEKF<T>::releaseVelocity(T sigma)
{
    T d[3];
    for (int a = 0; a < 3; ++a)
        d[a] = P_[kVel + a][kVel + a] > sigma * sigma ? P_[kVel + a][kVel + a] : sigma * sigma;
    for (int i = kVel; i < kVel + 3; ++i)
        for (int j = 0; j < N; ++j)
            P_[i][j] = P_[j][i] = T(0);
    for (int a = 0; a < 3; ++a)
        P_[kVel + a][kVel + a] = d[a];
}

template <typename T>
void RawGnssEKF<T>::clearRejects()
{
    std::memset(rejects_, 0, sizeof(rejects_));
}

template <typename T>
void RawGnssEKF<T>::inflateRestoreSigmas(T pos_m, T vel_mps, T att_rad)
{
    // the state is the attitude's half-angle: a rotation sigma is sigma/2 there
    const T var[3] = {pos_m * pos_m, vel_mps * vel_mps, T(0.25) * att_rad * att_rad};
    const bool set[3] = {pos_m > T(0), vel_mps > T(0), att_rad > T(0)};
    const int base[3] = {kPos, kVel, kAtt};
    for (int b = 0; b < 3; ++b)
    {
        if (!set[b])
            continue;
        for (int i = base[b]; i < base[b] + 3; ++i)
        {
            for (int j = 0; j < N; ++j)
                P_[i][j] = P_[j][i] = T(0);
            P_[i][i] = var[b];
        }
    }
}

template <typename T>
void RawGnssEKF<T>::setAccelBias(const T ab[3])
{
    for (int i = 0; i < 3; ++i)
        ab_[i] = ab[i];
}

template <typename T>
T RawGnssEKF<T>::isb(int sys) const
{
    return sys == GNSS_SYS_GAL ? isb_[0] : (sys == GNSS_SYS_BDS ? isb_[1] : T(0));
}

template <typename T>
T RawGnssEKF<T>::sigma(int i) const
{
    return P_[i][i] > T(0) ? std::sqrt(P_[i][i]) : T(0);
}

template <typename T>
void RawGnssEKF<T>::attSigmaNed(T out[3]) const
{
    T M[3][3];
    quat2dcm(q_, M);                                   // NED -> body
    // the full angle is twice the state's half-angle (4 P), and body -> NED is
    // M^T: the NED variance about axis i is sum_ab M[a][i] (4 P)_ab M[b][i]
    for (int i = 0; i < 3; ++i)
    {
        T s = T(0);
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b)
                s += M[a][i] * P_[kAtt + a][kAtt + b] * M[b][i];
        s *= T(4);
        out[i] = s > T(0) ? std::sqrt(s) : T(0);
    }
}

template <typename T>
T RawGnssEKF<T>::takeGPeak()
{
    const T g = g_peak_;
    g_peak_ = T(0);
    return g;
}

template <typename T>
void RawGnssEKF<T>::takeRatePeak(T& all, T& across)
{
    all = rate_peak_;
    across = rate_across_peak_;
    rate_peak_ = rate_across_peak_ = T(0);
}

template <typename T>
void RawGnssEKF<T>::gravityNed(T g[3]) const
{
    if (!p_.gravity_wgs84)
    {
        g[0] = g[1] = T(0);
        g[2] = T(kG0);
        return;
    }
    // Somigliana on the ellipsoid, the second-order height term, and the
    // small north component above it (tc_ekf.py gravity_wgs84)
    const double lat = lla_[0], h = lla_[2];
    const double s2 = std::sin(lat) * std::sin(lat);
    const double g0 = kGammaE * (1.0 + kSomK * s2) / std::sqrt(1.0 - kE2 * s2);
    const double gd = g0 * (1.0 - 2.0 / kRe * (1.0 + kFlat + kGravM - 2.0 * kFlat * s2) * h
                            + 3.0 / (kRe * kRe) * h * h);
    g[0] = T(-8.08e-9 * h * std::sin(2.0 * lat));
    g[1] = T(0);
    g[2] = T(gd);
}

template <typename T>
void RawGnssEKF<T>::propagate(const T acc[3], const T gyro[3], T dt)
{
    // the pre-propagation attitude, as the flight filter
    T Tn2b[3][3], Tb2n[3][3];
    quat2dcm(q_, Tn2b);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            Tb2n[j][i] = Tn2b[i][j];
    T g[3];
    gravityNed(g);
    // f_b: the specific force (what the accelerometer measures, less its
    // bias); a_b: the body's kinematic acceleration, with gravity, which the
    // state integrates. The covariance takes f_b (#1626 E1).
    T w[3], f_b[3], a_b[3];
    for (int i = 0; i < 3; ++i)
        w[i] = gyro[i] - wb_[i];
    for (int i = 0; i < 3; ++i)
    {
        f_b[i] = acc[i] - ab_[i];
        a_b[i] = f_b[i] + (Tn2b[i][0] * g[0] + Tn2b[i][1] * g[1] + Tn2b[i][2] * g[2]);
    }

    // attitude: rotation vector, exact for a constant rate over the step
    {
        const T th[3] = {w[0] * dt, w[1] * dt, w[2] * dt};
        const T ang = std::sqrt(th[0] * th[0] + th[1] * th[1] + th[2] * th[2]);
        T dq[4];
        if (ang > T(1e-8))
        {
            const T ha = T(0.5) * ang;
            const T s = std::sin(ha) / ang;
            dq[0] = std::cos(ha);
            dq[1] = s * th[0];
            dq[2] = s * th[1];
            dq[3] = s * th[2];
        }
        else
        {
            dq[0] = T(1);
            dq[1] = T(0.5) * th[0];
            dq[2] = T(0.5) * th[1];
            dq[3] = T(0.5) * th[2];
        }
        T qn[4];
        quatMult(q_, dq, qn);
        for (int i = 0; i < 4; ++i)
            q_[i] = qn[i];
        normalize4(q_);
    }

    // velocity, then position from it
    for (int i = 0; i < 3; ++i)
        a_ned_[i] = Tb2n[i][0] * a_b[0] + Tb2n[i][1] * a_b[1] + Tb2n[i][2] * a_b[2];
    for (int i = 0; i < 3; ++i)
        v_[i] += a_ned_[i] * dt;
    {
        double rew, rns;
        earthRad(lla_[0], rew, rns);
        const double dts = static_cast<double>(dt);
        const double dlat = dts * static_cast<double>(v_[0]) / (rns + lla_[2]);
        const double dlon = dts * static_cast<double>(v_[1]) / ((rew + lla_[2]) * std::cos(lla_[0]));
        const double dh = -dts * static_cast<double>(v_[2]);
        lla_[0] += dlat;
        lla_[1] += dlon;
        lla_[2] += dh;
    }

    const T fn = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    g_excess_ = fn / T(kG0) - T(1);
    if (g_excess_ > g_peak_)
        g_peak_ = g_excess_;
    if (track_rate_)
    {
        const T across = std::sqrt(w[1] * w[1] + w[2] * w[2]);
        const T all = std::sqrt(w[0] * w[0] + across * across);
        if (all > rate_peak_)
            rate_peak_ = all;
        if (across > rate_across_peak_)
            rate_across_peak_ = across;
    }

    propagateCovariance(Tb2n, f_b, w, g[2], dt);
    clk_b_ += static_cast<double>(clk_d_) * static_cast<double>(dt);
    t_ += static_cast<double>(dt);
    pushHistory(dt, a_ned_);
    // the carrier's noise grows with the roll rate (body x, the nose): its
    // mean size until the next screen
    if (p_.cr_mode != kCarrierOff)
    {
        cr_roll_sum_ += std::fabs(static_cast<double>(w[0])) * static_cast<double>(dt);
        cr_roll_t_ += static_cast<double>(dt);
    }
}

// P <- (I + F dt) P (I + F dt)' + Qd, Qd = Qc dt + (F Qc + Qc F') dt^2 / 2,
// touching only F's non-zero blocks (the flight filter's sparse form):
//   F[0:3, 3:6]   = I                    F[5, 2] = 2 g / R
//   F[3:6, 6:9]   = -2 Tb2n [f_b x]      F[3:6, 9:12] = -Tb2n
//   F[6:9, 6:9]   = -[w x]               F[6:9, 12:15] = -I / 2
//   F[9:12]       = -I / tau_a           F[12:15] = -I / tau_w
//   F[15, 16]     = 1
// The tilt to velocity is the specific force's (#1626 E1): v' = Tb2n f + g,
// and gravity, in NED, does not turn with the attitude, so q = q^ (x) [1, a]
// gives dv' = -2 Tb2n [f x] a. Built from the kinematic acceleration, as
// GpsInsEKF's still is, it said a tilt theta drives no velocity on the pad or
// under a chute, where it drives g theta, and g theta in a coast, where it
// drives none.
// Qc is diagonal: the accelerometer noise is the same on every axis, so
// Tb2n diag(q) Tb2n' = q I.
template <typename T>
void RawGnssEKF<T>::propagateCovariance(const T Tb2n[3][3], const T f_b[3], const T w_b[3],
                                        T g_d, T dt)
{
    T Fva[3][3], Fvb[3][3], Fww[3][3];
    {
        const T sa[3][3] = {{T(0), -f_b[2], f_b[1]}, {f_b[2], T(0), -f_b[0]}, {-f_b[1], f_b[0], T(0)}};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
            {
                Fva[i][j] = T(-2) * (Tb2n[i][0] * sa[0][j] + Tb2n[i][1] * sa[1][j] + Tb2n[i][2] * sa[2][j]);
                Fvb[i][j] = -Tb2n[i][j];
            }
        const T sw[3][3] = {{T(0), -w_b[2], w_b[1]}, {w_b[2], T(0), -w_b[0]}, {-w_b[1], w_b[0], T(0)}};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                Fww[i][j] = -sw[i][j];
    }
    const T f52 = T(2) * g_d / T(kRe);
    const T half = T(0.5);

    // rows: A = (I + F dt) P
    T A[N][N];
    std::memcpy(A, P_, sizeof(A));
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < N; ++j)
            A[i][j] += dt * P_[i + 3][j];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < N; ++j)
        {
            T s = T(0);
            for (int k = 0; k < 3; ++k)
                s += Fva[i][k] * P_[6 + k][j] + Fvb[i][k] * P_[9 + k][j];
            if (i == 2)
                s += f52 * P_[2][j];
            A[3 + i][j] += dt * s;
        }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < N; ++j)
        {
            T s = T(0);
            for (int k = 0; k < 3; ++k)
                s += Fww[i][k] * P_[6 + k][j];
            s -= half * P_[12 + i][j];
            A[6 + i][j] += dt * s;
        }
    const T dab = T(1) + dt * f_ab_, dwb = T(1) + dt * f_wb_;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < N; ++j)
        {
            A[9 + i][j] = dab * P_[9 + i][j];
            A[12 + i][j] = dwb * P_[12 + i][j];
        }
    for (int j = 0; j < N; ++j)
        A[15][j] += dt * P_[16][j];

    // columns: P = A (I + F dt)'
    for (int i = 0; i < N; ++i)
    {
        T* Pi = P_[i];
        const T* Ai = A[i];
        for (int j = 0; j < N; ++j)
            Pi[j] = Ai[j];
        for (int j = 0; j < 3; ++j)
            Pi[j] += dt * Ai[j + 3];
        for (int j = 0; j < 3; ++j)
        {
            T s = T(0);
            for (int k = 0; k < 3; ++k)
                s += Ai[6 + k] * Fva[j][k] + Ai[9 + k] * Fvb[j][k];
            if (j == 2)
                s += f52 * Ai[2];
            Pi[3 + j] += dt * s;
        }
        for (int j = 0; j < 3; ++j)
        {
            T s = T(0);
            for (int k = 0; k < 3; ++k)
                s += Ai[6 + k] * Fww[j][k];
            s -= half * Ai[12 + j];
            Pi[6 + j] += dt * s;
        }
        for (int j = 0; j < 3; ++j)
        {
            Pi[9 + j] = dab * Ai[9 + j];
            Pi[12 + j] = dwb * Ai[12 + j];
        }
        Pi[15] += dt * Ai[16];
    }

    // Qd
    const T h2 = half * dt * dt;
    for (int i = 0; i < 3; ++i)
    {
        P_[3 + i][3 + i] += dt * q_acc_;
        P_[6 + i][6 + i] += dt * q_att_;
        P_[9 + i][9 + i] += dt * q_ab_ + h2 * T(2) * f_ab_ * q_ab_;
        P_[12 + i][12 + i] += dt * q_wb_ + h2 * T(2) * f_wb_ * q_wb_;
        const T pv = h2 * q_acc_;                       // position-velocity
        P_[i][3 + i] += pv;
        P_[3 + i][i] += pv;
        const T aw = h2 * (-half) * q_wb_;              // attitude-gyro bias
        P_[6 + i][12 + i] += aw;
        P_[12 + i][6 + i] += aw;
        for (int k = 0; k < 3; ++k)
        {
            const T va = h2 * Fva[i][k] * q_att_;       // velocity-attitude
            P_[3 + i][6 + k] += va;
            P_[6 + k][3 + i] += va;
            const T vb = h2 * Fvb[i][k] * q_ab_;        // velocity-accel bias
            P_[3 + i][9 + k] += vb;
            P_[9 + k][3 + i] += vb;
        }
    }
    P_[kClkBias][kClkBias] += dt * q_clkb_;
    P_[kClkDrift][kClkDrift] += dt * q_clkd_;
    P_[kClkBias][kClkDrift] += h2 * q_clkd_;
    P_[kClkDrift][kClkBias] += h2 * q_clkd_;
    P_[kIsbGal][kIsbGal] += dt * q_isb_;
    P_[kIsbBds][kIsbBds] += dt * q_isb_;

    symmetrize();
    stabilize();
}

template <typename T>
void RawGnssEKF<T>::symmetrize()
{
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < i; ++j)
        {
            const T s = T(0.5) * (P_[i][j] + P_[j][i]);
            P_[i][j] = s;
            P_[j][i] = s;
        }
}

// A floor and a ceiling on each variance, as the flight filter's stabilizeP:
// a non-finite or vanishing one restarts its row and column at the floor, one
// past its cap is scaled down with its correlations.
template <typename T>
void RawGnssEKF<T>::stabilize()
{
    static const T pmax[N] = {
        T(1e8), T(1e8), T(1e8), T(1e4), T(1e4), T(1e4), T(10), T(10), T(10),
        T(10), T(10), T(10), T(1), T(1), T(1), T(1e12), T(1e6), T(1e6), T(1e6)};
    const T floor = T(1e-12);
    for (int i = 0; i < N; ++i)
    {
        if (!std::isfinite(P_[i][i]) || P_[i][i] < floor)
        {
            for (int j = 0; j < N; ++j)
                P_[i][j] = P_[j][i] = T(0);
            P_[i][i] = floor;
            continue;
        }
        if (P_[i][i] > pmax[i])
        {
            const T s = std::sqrt(pmax[i] / P_[i][i]);
            for (int j = 0; j < N; ++j)
            {
                P_[i][j] *= s;
                P_[j][i] *= s;
            }
            P_[i][i] = pmax[i];
        }
    }
}

template <typename T>
T RawGnssEKF<T>::hph(const int* hidx, const T* hval, int hn) const
{
    T s = T(0);
    for (int a = 0; a < hn; ++a)
        for (int b = 0; b < hn; ++b)
            s += hval[a] * P_[hidx[a]][hidx[b]] * hval[b];
    return s;
}

// One scalar measurement (H sparse: hval at columns hidx), innovation y,
// variance R. gate > 0: refused, nothing changed, when y^2 / S exceeds it.
// att scales the gain's attitude rows. Joseph form exploiting the rank-1 K H
// (GpsInsEKF::applyScalarMeasUpdate), which stays exact with the gain so
// altered. Writes the correction to dx for the caller to inject. Returns 1
// applied, 0 refused by the gate, -1 for a non-positive S.
template <typename T>
int RawGnssEKF<T>::scalarUpdate(const int* hidx, const T* hval, int hn, T y, T R, T gate, T att,
                                T dx[N], T* nis, uint32_t kmask, const T* att_axis, const T* att_off_axis)
{
    T HP[N] = {};
    for (int m = 0; m < hn; ++m)
    {
        const T h = hval[m];
        const T* row = P_[hidx[m]];
        for (int j = 0; j < N; ++j)
            HP[j] += h * row[j];
    }
    T S = R;
    for (int m = 0; m < hn; ++m)
        S += HP[hidx[m]] * hval[m];
    if (!(S > T(0)))
        return -1;
    const T Sinv = T(1) / S;
    if (nis)
        *nis = y * y * Sinv;
    if (gate > T(0) && y * y * Sinv > gate)
        return 0;

    T K[N];
    for (int i = 0; i < N; ++i)
        K[i] = HP[i] * Sinv;
    gateGain(K, att, att_axis);
    if (kmask != 0xFFFFFFFFu)
        for (int i = 0; i < N; ++i)
            if (!(kmask & (1u << i)))
                K[i] = T(0);
    if (att_off_axis != nullptr)
    {
        // nothing about the axis: the attitude's and the gyro bias's
        // corrections across it only (the Joseph form below keeps P exact for
        // the gain so altered: the attitude about the axis, the gyro bias
        // along it and their covariance come through as they were)
        const int bases[2] = {kAtt, kGyroBias};
        for (int base : bases)
        {
            const T d = att_off_axis[0] * K[base] + att_off_axis[1] * K[base + 1] + att_off_axis[2] * K[base + 2];
            for (int a = 0; a < 3; ++a)
                K[base + a] -= att_off_axis[a] * d;
        }
    }

    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            P_[i][j] -= K[i] * HP[j];
    T Acol[N];
    for (int i = 0; i < N; ++i)
    {
        T s = T(0);
        for (int m = 0; m < hn; ++m)
            s += P_[i][hidx[m]] * hval[m];
        Acol[i] = s;
    }
    for (int i = 0; i < N; ++i)
    {
        const T KR = K[i] * R;
        for (int j = 0; j < N; ++j)
            P_[i][j] += -Acol[i] * K[j] + KR * K[j];
    }
    symmetrize();
    for (int i = 0; i < N; ++i)
        dx[i] = K[i] * y;
    return 1;
}

template <typename T>
void RawGnssEKF<T>::gateGain(T K[N], T att, const T* att_axis)
{
    if (att != T(1))
        for (int i = kAtt; i < kAtt + 3; ++i)
            K[i] *= att;
    if (att_axis != nullptr)
    {
        // only along the axis: the attitude's correction, and the gyro bias's
        // (across the vertical a gyro bias integrates into a tilt)
        const int bases[2] = {kAtt, kGyroBias};
        for (int base : bases)
        {
            const T d = att_axis[0] * K[base] + att_axis[1] * K[base + 1] + att_axis[2] * K[base + 2];
            for (int a = 0; a < 3; ++a)
                K[base + a] = att_axis[a] * d;
        }
    }
}

template <typename T>
void RawGnssEKF<T>::inject(const T dx[N])
{
    double rew, rns;
    earthRad(lla_[0], rew, rns);
    lla_[2] -= static_cast<double>(dx[2]);
    lla_[0] += static_cast<double>(dx[0]) / (rns + lla_[2]);
    lla_[1] += static_cast<double>(dx[1]) / ((rew + lla_[2]) * std::cos(lla_[0]));
    for (int i = 0; i < 3; ++i)
    {
        v_[i] += dx[kVel + i];
        ab_[i] += dx[kAccBias + i];
        wb_[i] += dx[kGyroBias + i];
    }
    clk_b_ += static_cast<double>(dx[kClkBias]);
    clk_d_ += dx[kClkDrift];
    isb_[0] += dx[kIsbGal];
    isb_[1] += dx[kIsbBds];
    T dq[4] = {T(1), dx[kAtt], dx[kAtt + 1], dx[kAtt + 2]};
    normalize4(dq);
    T qn[4];
    quatMult(q_, dq, qn);
    for (int i = 0; i < 4; ++i)
        q_[i] = qn[i];
    normalize4(q_);
    // The carrier's receiver snapshots (an epoch's, the one before's): a
    // correction now moves the trajectory at an earlier time t by dp - dv
    // (now - t), to first order -- whatever made it
    if (p_.cr_mode != kCarrierOff)
    {
        CrSnap* snaps[2] = {&cr_cur_, &cr_prev_};
        for (CrSnap* sn : snaps)
        {
            if (!sn->valid)
                continue;
            const double age = t_ - sn->t;
            for (int i = 0; i < 3; ++i)
                sn->d_ned[i] += static_cast<double>(dx[kPos + i]) - static_cast<double>(dx[kVel + i]) * age;
        }
    }
}

// How much of a GNSS correction reaches the attitude: cos^4 pitch, as the
// flight filter (GNSS sees a tilt only through the specific force, and
// nothing of the rotation about a vertical thrust).
template <typename T>
T RawGnssEKF<T>::attGate() const
{
    if (!p_.att_gate_cos4 || p_.att_gate_vertical)
        return T(1);
    T M[3][3];
    quat2dcm(q_, M);
    const T sp = -M[0][2];
    const T c2 = T(1) - sp * sp;
    return c2 * c2;
}

// The axis a GNSS correction's attitude part is held to: the vertical in the
// body frame (vert) with att_gate_vertical, else none. With attGate and
// gateGain, the attitude gate every GNSS update -- range, range rate,
// carrier -- goes through.
template <typename T>
const T* RawGnssEKF<T>::gnssAttAxis(T vert[3]) const
{
    T M[3][3];
    quat2dcm(q_, M);
    for (int a = 0; a < 3; ++a)
        vert[a] = M[a][2];
    return p_.att_gate_vertical ? vert : nullptr;
}

template <typename T>
void RawGnssEKF<T>::updateAccelLevel(const T acc[3])
{
    T M[3][3], g[3], ag[3], y[3];
    quat2dcm(q_, M);
    gravityNed(g);
    for (int i = 0; i < 3; ++i)
        ag[i] = M[i][0] * g[0] + M[i][1] * g[1] + M[i][2] * g[2];
    for (int i = 0; i < 3; ++i)
        y[i] = (acc[i] - ab_[i]) + ag[i];
    const int idx[3][3] = {{7, 8, 9}, {6, 8, 10}, {6, 7, 11}};
    const T val[3][3] = {{T(2) * ag[2], T(-2) * ag[1], T(1)},
                         {T(-2) * ag[2], T(2) * ag[0], T(1)},
                         {T(2) * ag[1], T(-2) * ag[0], T(1)}};
    // Gravity says nothing of a turn about the vertical (its H has no part
    // along it), but through the attitude's correlations with the tilt and
    // the accelerometer bias the gain would turn the heading, and move the
    // gyro bias along the vertical that turns it later: those rows, along the
    // vertical in the body frame -- NED down, M's third column -- are zeroed
    // (level_heading_free, TR_RawGnssEKF.h). The dual of the GNSS heading
    // gate (att_gate_vertical), which keeps only them.
    const T vert[3] = {M[0][2], M[1][2], M[2][2]};
    const T* off = p_.level_heading_free ? vert : nullptr;
    // One vector update with R = accel_var I, as three scalar ones on the
    // same linearization: each innovation less what the rows before it
    // already corrected, the correction injected once at the end.
    T dx_sum[N] = {};
    const T R = T(p_.accel_var);
    for (int r = 0; r < 3; ++r)
    {
        T yr = y[r];
        for (int m = 0; m < 3; ++m)
            yr -= val[r][m] * dx_sum[idx[r][m]];
        T dx[N];
        if (scalarUpdate(idx[r], val[r], 3, yr, R, T(0), T(1), dx, nullptr, 0xFFFFFFFFu, nullptr, off) == 1)
            for (int i = 0; i < N; ++i)
                dx_sum[i] += dx[i];
    }
    inject(dx_sum);
    stabilize();
}

template <typename T>
void RawGnssEKF<T>::updateHeight(double h_m, T var)
{
    // A height says nothing about the attitude or the body's accelerometer
    // bias: through the cross-covariance the IMU's propagation builds, a
    // barometer whose descent the IMU never felt (the firmware sim's step to
    // 5 m/s at apogee) tilted the attitude 6 degrees, and with the attitude
    // held but the bias free it pulled the bias off the levelling's tie
    // between the two (a roll and the body's sideways bias look the same to
    // gravity under a chute): either way, with no satellites, gravity on a
    // wrong tilt became kilometres of horizontal drift. So it moves the
    // vertical position and velocity and the receiver clock (height and clock
    // correlate in the satellites' geometry), nothing else.
    const uint32_t kmask = (1u << (kPos + 2)) | (1u << (kVel + 2)) | (0xFu << kClkBias);
    const int idx[1] = {kPos + 2};
    const T val[1] = {T(-1)};
    const T y = static_cast<T>(h_m - lla_[2]);
    T dx[N];
    if (scalarUpdate(idx, val, 1, y, var, T(0), T(1), dx, nullptr, kmask) == 1)
    {
        inject(dx);
        stabilize();
    }
}

template <typename T>
void RawGnssEKF<T>::updateZeroVelocity(T sigma)
{
    // position and velocity (states 0-5) only
    const uint32_t kmask = (1u << (kVel + 3)) - 1u;
    const T R = sigma * sigma;
    for (int i = 0; i < 3; ++i)
    {
        const int idx[1] = {kVel + i};
        const T val[1] = {T(1)};
        T dx[N];
        if (scalarUpdate(idx, val, 1, -v_[i], R, T(0), T(1), dx, nullptr, kmask) == 1)
            inject(dx);
    }
    stabilize();
}

template <typename T>
bool RawGnssEKF<T>::updateMagHeading(const T acc[3], const T mag[3], bool accel_is_gravity, T declination_rad,
                                     T field_err_ut)
{
    T M[3][3];
    quat2dcm(q_, M);                                   // NED -> body
    const T d[3] = {M[0][2], M[1][2], M[2][2]};        // body down, the heading error's axis
    T dm[3] = {d[0], d[1], d[2]};
    T sig_tilt;
    if (accel_is_gravity)
    {
        const T an = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        if (an < T(0.01))
            return false;
        for (int i = 0; i < 3; ++i)
            dm[i] = -acc[i] / an;
        sig_tilt = static_cast<T>(p_.mag_tilt_sigma_mps2) / an;
    }
    else
    {
        // the state's tilt sigma, about the two horizontal NED axes
        T pn[2] = {T(0), T(0)};
        for (int k = 0; k < 2; ++k)
            for (int a = 0; a < 3; ++a)
                for (int b = 0; b < 3; ++b)
                    pn[k] += M[a][k] * P_[kAtt + a][kAtt + b] * M[b][k];
        const T pmx = pn[0] > pn[1] ? pn[0] : pn[1];
        sig_tilt = T(2) * std::sqrt(pmx > T(0) ? pmx : T(0));
        if (sig_tilt > T(1))
            sig_tilt = T(1);
    }
    // the shortest arc taking dm onto d, applied to the field
    T mb[3] = {mag[0], mag[1], mag[2]};
    {
        T k[3] = {dm[1] * d[2] - dm[2] * d[1], dm[2] * d[0] - dm[0] * d[2], dm[0] * d[1] - dm[1] * d[0]};
        const T sn = std::sqrt(k[0] * k[0] + k[1] * k[1] + k[2] * k[2]);
        const T cs = dm[0] * d[0] + dm[1] * d[1] + dm[2] * d[2];
        if (sn > T(1e-6))
        {
            for (int a = 0; a < 3; ++a)
                k[a] /= sn;
            const T kxm[3] = {k[1] * mb[2] - k[2] * mb[1], k[2] * mb[0] - k[0] * mb[2],
                              k[0] * mb[1] - k[1] * mb[0]};
            const T kdm = k[0] * mb[0] + k[1] * mb[1] + k[2] * mb[2];
            for (int a = 0; a < 3; ++a)
                mb[a] = mb[a] * cs + kxm[a] * sn + k[a] * kdm * (T(1) - cs);
        }
    }
    T mn[3];                                           // the field in NED
    for (int a = 0; a < 3; ++a)
        mn[a] = M[0][a] * mb[0] + M[1][a] * mb[1] + M[2][a] * mb[2];
    const T bh2 = mn[0] * mn[0] + mn[1] * mn[1];
    if (!(bh2 >= T(9)))                                // under 3 uT across: no heading in it
        return false;
    T y = declination_rad - std::atan2(mn[1], mn[0]);
    const T pi = T(3.14159265358979323846);
    while (y > pi)
        y -= T(2) * pi;
    while (y < -pi)
        y += T(2) * pi;

    // the field's noise, the tilt reference's leaking the vertical field
    // across, floored and capped; then a hard iron the magnitude allows
    T r_cal = T(0);
    if (field_err_ut >= T(0))
    {
        T frac = field_err_ut / std::sqrt(bh2);
        if (frac > T(1))
            frac = T(1);
        const T dpsi = std::asin(frac);
        r_cal = dpsi * dpsi;
    }
    const T sm = static_cast<T>(p_.mag_sigma_ut);
    T r = T(2) * sm * sm / bh2 + sig_tilt * sig_tilt * (mn[2] * mn[2]) / bh2;
    if (r < static_cast<T>(p_.mag_r_floor))
        r = static_cast<T>(p_.mag_r_floor);
    if (r > static_cast<T>(p_.mag_r_ceiling))
        r = static_cast<T>(p_.mag_r_ceiling);
    r += r_cal;

    const int idx[3] = {kAtt, kAtt + 1, kAtt + 2};
    const T val[3] = {T(2) * d[0], T(2) * d[1], T(2) * d[2]};
    // a large innovation (a disturbance, a bad sample) does not move the gyro bias
    T S = r + hph(idx, val, 3);
    uint32_t kmask = 0xFFFFFFFFu;
    if (!(S > T(0)) || y * y / S > static_cast<T>(p_.mag_bias_nis))
        for (int i = kGyroBias; i < kGyroBias + 3; ++i)
            kmask &= ~(1u << i);
    T dx[N];
    if (scalarUpdate(idx, val, 3, y, r, T(0), T(1), dx, nullptr, kmask) != 1)
        return false;
    inject(dx);
    stabilize();
    return true;
}

template <typename T>
void RawGnssEKF<T>::pushHistory(T dt, const T a_ned[3])
{
    // the carrier's integration excess, this step's: a dt^2 / 2
    const bool cr = p_.cr_mode != kCarrierOff;
    T ex[3] = {};
    if (cr)
        for (int i = 0; i < 3; ++i)
        {
            ex[i] = T(0.5) * a_ned[i] * dt * dt;
            cr_ex_[i] += static_cast<double>(ex[i]);
        }
    if (hn_ > 0 && (t_ - hopen_) < p_.hist_spacing_s)
    {
        Hist& e = hist_[(hhead_ + kHist - 1) % kHist];
        e.t = t_;
        for (int i = 0; i < 3; ++i)
        {
            e.dv[i] += a_ned[i] * dt;
            e.dp[i] += v_[i] * dt;
        }
        if (cr)
            for (int i = 0; i < 3; ++i)
                e.ex[i] += ex[i];
        return;
    }
    hopen_ = hn_ > 0 ? hist_[(hhead_ + kHist - 1) % kHist].t : t_ - static_cast<double>(dt);
    Hist& e = hist_[hhead_];
    e.t = t_;
    for (int i = 0; i < 3; ++i)
    {
        e.dv[i] = a_ned[i] * dt;
        e.dp[i] = v_[i] * dt;
        e.ex[i] = ex[i];
    }
    hhead_ = (hhead_ + 1) % kHist;
    if (hn_ < kHist)
        hn_++;
}

// v(now) - v(now - tau) and p(now) - p(now - tau), NED, from the IMU's own
// increments (no filter corrections in them), interpolated within an entry
// (tc_ekf.py velocity_change). Where the history does not reach back that
// far: the last acceleration and the velocity times tau, and false.
template <typename T>
bool RawGnssEKF<T>::historyChange(double tau, T dv[3], T dp[3]) const
{
    for (int i = 0; i < 3; ++i)
        dv[i] = dp[i] = T(0);
    if (tau <= 0.0)
        return true;
    const double t0 = t_ - tau;
    auto at = [&](int k) -> const Hist& { return hist_[(hhead_ + kHist - hn_ + k) % kHist]; };
    if (hn_ > 1 && at(0).t <= t0 && at(hn_ - 1).t >= t_ - 1e-9)
    {
        T sv[3] = {}, sp[3] = {};
        for (int k = hn_ - 1; k > 0; --k)
        {
            const Hist& b = at(k);
            const Hist& a = at(k - 1);
            if (a.t <= t0)
            {
                const T w = b.t > a.t ? T((t0 - a.t) / (b.t - a.t)) : T(0);
                for (int i = 0; i < 3; ++i)
                {
                    dv[i] = sv[i] + (T(1) - w) * b.dv[i];
                    dp[i] = sp[i] + (T(1) - w) * b.dp[i];
                }
                return true;
            }
            for (int i = 0; i < 3; ++i)
            {
                sv[i] += b.dv[i];
                sp[i] += b.dp[i];
            }
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        dv[i] = a_ned_[i] * T(tau);
        dp[i] = v_[i] * T(tau);
    }
    return false;
}

template <typename T>
void RawGnssEKF<T>::receiverState(double r[3], double v[3], double Tm[3][3]) const
{
    // llaToEcef and eToNed on one set of sines and cosines (software double
    // trig is most of what an update costs on the flight computer)
    const double sl = std::sin(lla_[0]), cl = std::cos(lla_[0]);
    const double so = std::sin(lla_[1]), co = std::cos(lla_[1]);
    const double d = std::fabs(1.0 - kE2 * sl * sl);
    const double rew = kRe / std::sqrt(d);
    r[0] = (rew + lla_[2]) * cl * co;
    r[1] = (rew + lla_[2]) * cl * so;
    r[2] = (rew * (1.0 - kE2) + lla_[2]) * sl;
    Tm[0][0] = -sl * co; Tm[0][1] = -sl * so; Tm[0][2] = cl;
    Tm[1][0] = -so;      Tm[1][1] = co;       Tm[1][2] = 0.0;
    Tm[2][0] = -cl * co; Tm[2][1] = -cl * so; Tm[2][2] = -sl;
    for (int i = 0; i < 3; ++i)
        v[i] = Tm[0][i] * static_cast<double>(v_[0]) + Tm[1][i] * static_cast<double>(v_[1])
             + Tm[2][i] * static_cast<double>(v_[2]);
}

template <typename T>
void RawGnssEKF<T>::receiverEcef(double r[3], double v[3]) const
{
    double Tm[3][3];
    receiverState(r, v, Tm);
}


template <typename T>
void RawGnssEKF<T>::receiverEcefAt(double age_s, double r_e[3], double v_e[3]) const
{
    double r[3], v[3], Tm[3][3];
    T dp[3], dv[3];
    receiverState(r, v, Tm);
    historyChange(age_s, dv, dp);
    backDate(r, v, Tm, dp, dv, r_e, v_e);
}

template <typename T>
bool RawGnssEKF<T>::imuMotion(double t1, double t2, T dv[3], T dp[3]) const
{
    for (int i = 0; i < 3; ++i)
        dv[i] = dp[i] = T(0);
    if (!(t2 >= t1))
        return false;
    auto at = [&](int k) -> const Hist& { return hist_[(hhead_ + kHist - hn_ + k) % kHist]; };
    // The filter's time is a sum of float steps, a fraction of a millisecond
    // from the IMU's clock that t1 and t2 are on: that much past either end
    // of the history is its end.
    constexpr double kSlack = 1.0e-3;
    if (hn_ < 2 || at(0).t > t1 + kSlack || at(hn_ - 1).t < t2 - kSlack)
        return false;
    // Each entry spans (the previous entry's end, its own end] at a constant
    // acceleration: its share of [t1, t2] adds that much velocity, and the
    // velocity gathered so far, plus half its own, times its length.
    double V[3] = {0.0, 0.0, 0.0}, P[3] = {0.0, 0.0, 0.0};
    for (int k = 1; k < hn_; ++k)
    {
        const double s0 = at(k - 1).t, s1 = at(k).t;
        if (s1 <= t1 || s1 <= s0)
            continue;
        if (s0 >= t2)
            break;
        const double lo = s0 > t1 ? s0 : t1, hi = s1 < t2 ? s1 : t2;
        const double len = hi - lo;
        for (int i = 0; i < 3; ++i)
        {
            const double d = static_cast<double>(at(k).dv[i]) * len / (s1 - s0);
            P[i] += (V[i] + 0.5 * d) * len;
            V[i] += d;
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        dv[i] = T(V[i]);
        dp[i] = T(P[i]);
    }
    return true;
}

template <typename T>
bool RawGnssEKF<T>::initClockFrom(const RawMeas* m, int n, double age_s)
{
    double r[3], v[3], Tm[3][3], r_e[3], v_e[3];
    T dp[3], dv[3];
    receiverState(r, v, Tm);
    historyChange(age_s, dv, dp);
    backDate(r, v, Tm, dp, dv, r_e, v_e);

    double by[3][RawEpoch::kMaxObs];
    int nby[3] = {0, 0, 0};
    double d[RawEpoch::kMaxObs];
    int nd = 0;
    int ref = -1;
    for (int i = 0; i < n; ++i)
    {
        if (m[i].sys > GNSS_SYS_BDS)
            continue;
        Los L;
        lineOfSight(m[i], r_e, v_e, L);
        if (m[i].has_pr && nby[m[i].sys] < RawEpoch::kMaxObs)
        {
            by[m[i].sys][nby[m[i].sys]++] = m[i].pr - L.rho;
            if (ref < 0)
                ref = m[i].sys;
        }
        if (m[i].has_rr && nd < RawEpoch::kMaxObs)
            d[nd++] = m[i].rr - L.rate;
    }
    if (ref < 0)
        return false;
    if (nby[GNSS_SYS_GPS] > 0)
        ref = GNSS_SYS_GPS;
    const double b0 = median(by[ref], nby[ref]);
    const double d0 = nd > 0 ? median(d, nd) : 0.0;
    clk_d_ = T(d0);
    clk_b_ = b0 + d0 * age_s;
    isb_[0] = nby[GNSS_SYS_GAL] > 0 ? T(median(by[GNSS_SYS_GAL], nby[GNSS_SYS_GAL]) - b0) : T(0);
    isb_[1] = nby[GNSS_SYS_BDS] > 0 ? T(median(by[GNSS_SYS_BDS], nby[GNSS_SYS_BDS]) - b0) : T(0);

    const T pz = P_[2][2];
    for (int i = kClkBias; i < N; ++i)
        for (int j = 0; j < N; ++j)
            P_[i][j] = P_[j][i] = T(0);
    P_[kClkBias][kClkBias] = T(p_.p0_clk_bias * p_.p0_clk_bias) + pz;
    P_[kClkDrift][kClkDrift] = T(p_.p0_clk_drift * p_.p0_clk_drift);
    P_[kIsbGal][kIsbGal] = P_[kIsbBds][kIsbBds] = T(p_.p0_isb * p_.p0_isb);
    clk_ready_ = true;
    return true;
}

template <typename T>
RawGnssStats RawGnssEKF<T>::updateGnssRaw(const RawMeas* m, int n, double age_s)
{
    RawGnssStats st;
    if (beginGnssRaw(m, n, age_s, st))
        updateGnssRawPart(m, 0, n, age_s, st);
    if (beginCarrier(m, n, age_s, st) > 0)
        fuseCarrier(m, st);
    endCarrier(st);
    return st;
}

template <typename T>
bool RawGnssEKF<T>::beginGnssRaw(const RawMeas* m, int n, double age_s, RawGnssStats& st)
{
    if (age_s < 0.0)
        age_s = 0.0;
    if (!clk_ready_ && !initClockFrom(m, n, age_s))
        return false;

    // the receiver's history back to the epoch
    T dp_e[3], dv_e[3];
    historyChange(age_s, dv_e, dp_e);

    // Receiver clock steps: every satellite's innovation agrees on one
    {
        double r[3], v[3], Tm[3][3], r_e[3], v_e[3];
        receiverState(r, v, Tm);
        backDate(r, v, Tm, dp_e, dv_e, r_e, v_e);
        double inn[RawEpoch::kMaxObs], inn_rr[RawEpoch::kMaxObs], dev[RawEpoch::kMaxObs];
        int ni = 0, nr = 0;
        const double b_e = clk_b_ - static_cast<double>(clk_d_) * age_s;
        for (int i = 0; i < n; ++i)
        {
            if (m[i].sys > GNSS_SYS_BDS)
                continue;
            Los L;
            lineOfSight(m[i], r_e, v_e, L);
            if (m[i].has_pr && ni < RawEpoch::kMaxObs)
                inn[ni++] = m[i].pr - (L.rho + b_e + static_cast<double>(isb(m[i].sys)));
            if (m[i].has_rr && nr < RawEpoch::kMaxObs)
                inn_rr[nr++] = m[i].rr - (L.rate + static_cast<double>(clk_d_));
        }
        if (ni > 0)
        {
            for (int i = 0; i < ni; ++i)
                dev[i] = inn[i];
            const double med = median(dev, ni);
            const double ms = kC * 1e-3;
            const long k = std::lround(med / ms);
            if (k != 0 && std::fabs(med - static_cast<double>(k) * ms) < 0.1 * ms)
            {
                clk_b_ += static_cast<double>(k) * ms;
                st.clock_jump_ms = static_cast<int>(k);
            }
            else if (ni >= 4 && std::fabs(med) > p_.clock_step_m)
            {
                for (int i = 0; i < ni; ++i)
                    dev[i] = std::fabs(inn[i] - med);
                if (median(dev, ni) < p_.clock_step_spread_m)
                {
                    clk_b_ += med;
                    st.clock_step_m = med;
                }
            }
        }
        if (nr >= 4)
        {
            for (int i = 0; i < nr; ++i)
                dev[i] = inn_rr[i];
            const double med = median(dev, nr);
            if (std::fabs(med) > p_.drift_step_mps)
            {
                for (int i = 0; i < nr; ++i)
                    dev[i] = std::fabs(inn_rr[i] - med);
                if (median(dev, nr) < p_.drift_step_spread_mps)
                {
                    clk_d_ += T(med);
                    st.drift_step_mps = med;
                }
            }
        }
    }
    // The carrier's receiver at this epoch: back-dated as the pseudoranges
    // take it, before they correct it (each correction from here moves it)
    if (p_.cr_mode != kCarrierOff)
        crSnapshot(age_s, dp_e);
    return true;
}

template <typename T>
void RawGnssEKF<T>::updateGnssRawPart(const RawMeas* m, int i0, int i1, double age_s, RawGnssStats& st)
{
    if (age_s < 0.0)
        age_s = 0.0;
    // the receiver's history back to the epoch, and to its Doppler's instant,
    // as it stands now: a part fused later takes the epoch back further
    const double tau = p_.rr_lag_s > 0.0 ? p_.rr_lag_s : 0.0;
    T dp_e[3], dv_e[3], dp_l[3], dv_l[3];
    historyChange(age_s, dv_e, dp_e);
    historyChange(age_s + tau, dv_l, dp_l);

    const T att = attGate();
    T vert[3];                            // the vertical in the body frame (att_gate_vertical)
    const T* axis = gnssAttAxis(vert);
    // att_gnss_off: the attitude and the gyro bias kept out of every GNSS row
    // here, and only here (scalarUpdate's other callers -- the levelling, the
    // height, zero velocity, the magnetometer -- keep theirs). The
    // accelerometer bias keeps its gain (RawGnssEKFParams::att_gnss_off says
    // what that leaves). The hold (setAttHold) is the same mask for a while.
    const uint32_t kmask =
        p_.att_gnss_off || att_hold_ ? ~((0x7u << kAtt) | (0x7u << kGyroBias)) : 0xFFFFFFFFu;
    for (int i = i0; i < i1; ++i)
    {
        const RawMeas& mi = m[i];
        if (mi.sys > GNSS_SYS_BDS || mi.prn >= 64)
            continue;
        if ((!mi.has_pr || !fuse_pr_) && !mi.has_rr)
            continue;
        // The geometry once per satellite, at the state every satellite
        // before it left (the prototype redoes it per measurement too, but
        // this satellite's own pseudorange update moves the receiver by
        // metres at most, turning its line of sight by ~1e-7 rad).
        double r[3], v[3], Tm[3][3], r_e[3], v_e[3];
        receiverState(r, v, Tm);
        backDate(r, v, Tm, dp_e, dv_e, r_e, v_e);
        Los L;
        lineOfSight(mi, r_e, v_e, L);
        T u_ned[3];
        for (int a = 0; a < 3; ++a)
            u_ned[a] = T(Tm[a][0] * L.u[0] + Tm[a][1] * L.u[1] + Tm[a][2] * L.u[2]);
        for (int kind = 0; kind < 2; ++kind)
        {
            if (kind == 0 ? (!mi.has_pr || !fuse_pr_) : !mi.has_rr)
                continue;
            double pred_rate = 0.0;
            if (kind == 1)
            {
                // the receiver's velocity as the pseudorange update left it,
                // taken back to the Doppler's instant (a late Doppler: tau
                // before the epoch, the satellite's own range acceleration
                // backed out)
                double vr[3], v_l[3];
                for (int a = 0; a < 3; ++a)
                    vr[a] = Tm[0][a] * static_cast<double>(v_[0]) + Tm[1][a] * static_cast<double>(v_[1])
                          + Tm[2][a] * static_cast<double>(v_[2]);
                const T* dv = tau > 0.0 ? dv_l : dv_e;
                for (int a = 0; a < 3; ++a)
                    v_l[a] = vr[a] - (Tm[0][a] * static_cast<double>(dv[0]) + Tm[1][a] * static_cast<double>(dv[1])
                                      + Tm[2][a] * static_cast<double>(dv[2]));
                Los Lr = L;
                Lr.rate = 0.0;
                for (int a = 0; a < 3; ++a)
                    Lr.rate += L.u[a] * (L.vs[a] - v_l[a]);
                pred_rate = tau > 0.0 ? Lr.rate - tau * satRangeAccel(mi, Lr, v_l) : Lr.rate;
            }

            int hidx[9];
            T hval[9];
            int hn = 0;
            T y, R;
            const double dt_i = t_ - last_upd_[mi.sys][mi.prn][kind];
            if (kind == 0)
            {
                for (int a = 0; a < 3; ++a)
                {
                    hidx[hn] = kPos + a;
                    hval[hn++] = -u_ned[a];
                }
                hidx[hn] = kClkBias;
                hval[hn++] = T(1);
                if (mi.sys == GNSS_SYS_GAL || mi.sys == GNSS_SYS_BDS)
                {
                    hidx[hn] = mi.sys == GNSS_SYS_GAL ? kIsbGal : kIsbBds;
                    hval[hn++] = T(1);
                }
                if (age_s > 0.0)
                {
                    // the position and clock bias then: now less velocity and drift times age
                    for (int a = 0; a < 3; ++a)
                    {
                        hidx[hn] = kVel + a;
                        hval[hn++] = T(age_s) * u_ned[a];
                    }
                    hidx[hn] = kClkDrift;
                    hval[hn++] = T(-age_s);
                }
                const double b_e = clk_b_ - static_cast<double>(clk_d_) * age_s;
                y = T(mi.pr - (L.rho + b_e + static_cast<double>(isb(mi.sys))));
                const double s_w = static_cast<double>(mi.sigma_pr), s_c = static_cast<double>(mi.sigma_pr_corr);
                R = T(s_w * s_w + s_c * s_c * corrInflation(dt_i, static_cast<double>(mi.tau_pr)));
            }
            else
            {
                for (int a = 0; a < 3; ++a)
                {
                    hidx[hn] = kVel + a;
                    hval[hn++] = -u_ned[a];
                }
                hidx[hn] = kClkDrift;
                hval[hn++] = T(1);
                y = T(mi.rr - (pred_rate + static_cast<double>(clk_d_)));
                const double s_r = static_cast<double>(mi.sigma_rr);
                R = T(s_r * s_r * corrInflation(dt_i, static_cast<double>(mi.tau_rr)));
            }

            uint8_t& rej = rejects_[mi.sys][mi.prn][kind];
            T gate = T(p_.gate_chi2);
            if (rej >= rejectPersist(kind))
            {
                // persistent disagreement: the filter is the suspect, so let
                // it in, its variance scaled by its NIS over the gate (the
                // prototype's rule: on R alone, it would just have passed)
                gate = T(0);
                const T nis0 = y * y / (hph(hidx, hval, hn) + R);
                if (nis0 > T(p_.gate_chi2))
                    R *= nis0 / T(p_.gate_chi2);
            }
            T dx[N];
            T nis = T(0);
            const bool ok = scalarUpdate(hidx, hval, hn, y, R, gate, att, dx, &nis, kmask, axis) == 1;
            if (ok)
            {
                inject(dx);
                stabilize();
                last_upd_[mi.sys][mi.prn][kind] = t_;
                rej = 0;
                if (static_cast<float>(nis) > st.nis_max)
                    st.nis_max = static_cast<float>(nis);
                if (kind == 0)
                    st.used_pr++;
                else
                    st.used_rr++;
            }
            else
            {
                if (rej < 255)
                    rej++;
                if (kind == 0)
                    st.rejected_pr++;
                else
                    st.rejected_rr++;
            }
        }
    }
}

template class RawGnssEKF<float>;
template class RawGnssEKF<double>;

double sagnacRange(const RawMeas& m, const double r[3])
{
    const double still[3] = {0.0, 0.0, 0.0};
    Los L;
    lineOfSight(m, r, still, L);
    return L.rho;
}

namespace ekf_detail
{

double sagnacLos(const double rs[3], const double r[3], double u[3])
{
    const double still[3] = {0.0, 0.0, 0.0};
    Los L;
    lineOfSight(rs, still, r, still, L);
    for (int a = 0; a < 3; ++a)
        u[a] = L.u[a];
    return L.rho;
}

double medianOf(double* v, int n)
{
    return median(v, n);
}

}  // namespace ekf_detail

double stillRangeRate(const RawMeas& m, const double r[3], double u[3])
{
    const double still[3] = {0.0, 0.0, 0.0};
    Los L;
    lineOfSight(m, r, still, L);
    for (int a = 0; a < 3; ++a)
        u[a] = L.u[a];
    return L.rate;
}

}  // namespace gnss_raw
