// HeadingFit.cpp — see HeadingFit.h
#include "HeadingFit.h"

#include <cmath>

namespace gnss_raw
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kStep = 2.0 * kPi / HeadingFit::kGrid;   // 0.25 deg
constexpr int    kCoarse = 10;                            // the coarse pass: every 10th grid point
constexpr int    kWalk = HeadingFit::kGrid / 4;           // the sigma: out to 90 deg (k < kWalk)

inline double gridAngle(int k)
{
    return -kPi + static_cast<double>(k) * kStep;
}

// cos and sin by their series, at compile time: |x| <= pi/4, to double's
// rounding (the next term is under 1e-26)
struct CS
{
    double c, s;
};
constexpr CS seriesCS(double x)
{
    const double x2 = x * x;
    double c = 1.0, s = x, tc = 1.0, ts = x;
    for (int k = 1; k <= 12; ++k)
    {
        tc *= -x2 / static_cast<double>((2 * k - 1) * (2 * k));
        ts *= -x2 / static_cast<double>((2 * k) * (2 * k + 1));
        c += tc;
        s += ts;
    }
    return {c, s};
}

// cos and sin of h half-degrees: reduced exactly to the nearest quarter turn
constexpr CS halfDegreesCS(int h)
{
    const int q = (h >= 0 ? h + 90 : h - 90) / 180;
    const CS v = seriesCS(static_cast<double>(h - 180 * q) * (kPi / 360.0));
    switch (((q % 4) + 4) % 4)
    {
    case 0: return {v.c, v.s};
    case 1: return {-v.s, v.c};
    case 2: return {-v.c, -v.s};
    default: return {v.s, -v.c};
    }
}

// The grid's turns: cos and sin of one step and of a coarse stride, and of
// each coarse point. Built by the compiler, so it sits in flash: no libm
// doubles (software on the ESP32-S3) and no internal RAM.
struct Turns
{
    double c1, s1, cK, sK;
    double cc[HeadingFit::kGrid / kCoarse], sc[HeadingFit::kGrid / kCoarse];
};
constexpr Turns makeTurns()
{
    Turns t{};
    const CS one = seriesCS(kStep), stride = seriesCS(kCoarse * kStep);
    t.c1 = one.c;
    t.s1 = one.s;
    t.cK = stride.c;
    t.sK = stride.s;
    for (int j = 0; j < HeadingFit::kGrid / kCoarse; ++j)
    {
        const CS v = halfDegreesCS(-360 + 5 * j);     // -180 deg + 2.5 j
        t.cc[j] = v.c;
        t.sc[j] = v.s;
    }
    return t;
}
constexpr Turns kTurns = makeTurns();
// the quarter turns exactly, and one between them to rounding
static_assert(kTurns.cc[0] == -1.0 && kTurns.sc[0] == 0.0, "-180 deg");
static_assert(kTurns.cc[36] == 0.0 && kTurns.sc[36] == -1.0, "-90 deg");
static_assert(kTurns.cc[72] == 1.0 && kTurns.sc[72] == 0.0, "0 deg");
static_assert(kTurns.cc[108] == 0.0 && kTurns.sc[108] == 1.0, "90 deg");
static_assert(kTurns.cc[18] + 0.70710678118654752 < 1e-15 && kTurns.cc[18] + 0.70710678118654752 > -1e-15 &&
                  kTurns.sc[18] + 0.70710678118654752 < 1e-15 && kTurns.sc[18] + 0.70710678118654752 > -1e-15,
              "-135 deg");

// (c, s) of an angle turned by (cd, sd)
inline void turn(double& c, double& s, double cd, double sd)
{
    const double cn = c * cd - s * sd;
    s = s * cd + c * sd;
    c = cn;
}

// (c, s) turned by a small angle d (under a grid step: the series' next
// terms are below double's rounding)
inline void turnBy(double& c, double& s, double d)
{
    const double d2 = d * d;
    turn(c, s, 1.0 - 0.5 * d2 * (1.0 - d2 / 12.0), d * (1.0 - d2 / 6.0 * (1.0 - d2 / 20.0)));
}

inline int wrapIndex(int k)
{
    k %= HeadingFit::kGrid;
    return k < 0 ? k + HeadingFit::kGrid : k;
}

// grid points between two, around the circle
inline int gridApart(int a, int b)
{
    const int d = wrapIndex(a - b);
    return d < HeadingFit::kGrid - d ? d : HeadingFit::kGrid - d;
}

// A periodic profile's local minima, its values fed in order: the lowest
// few kept (a degree-two profile has two; rounding can add one where it is
// flat).
template <typename V>
struct Minima
{
    static constexpr int kMax = 4;
    int k[kMax];
    V   v[kMax];
    int n = 0;
    V   first = 0, second = 0, a = 0, b = 0;
    int fed = 0;
    void take(int kk, V vv)
    {
        int pos = n;
        while (pos > 0 && vv < v[pos - 1])
            pos--;
        if (pos >= kMax)
            return;
        for (int m = n < kMax ? n : kMax - 1; m > pos; --m)
        {
            k[m] = k[m - 1];
            v[m] = v[m - 1];
        }
        k[pos] = kk;
        v[pos] = vv;
        if (n < kMax)
            n++;
    }
    void feed(V x)
    {
        if (fed == 0)
            first = x;
        else if (fed == 1)
            second = x;
        if (fed >= 2 && b < a && b <= x)
            take(fed - 1, b);
        a = b;
        b = x;
        fed++;
    }
    void close()                       // around the circle: the last point, and the first
    {
        if (fed < 3)
            return;
        if (b < a && b <= first)
            take(fed - 1, b);
        if (first < b && first <= second)
            take(0, first);
    }
};

// body -> NED (the transpose of Quat2DCM, NED -> body)
void bodyToNed(const float q[4], float C[3][3])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    C[0][0] = 1.0f - 2.0f * (y * y + z * z); C[1][0] = 2.0f * (x * y + w * z);        C[2][0] = 2.0f * (x * z - w * y);
    C[0][1] = 2.0f * (x * y - w * z);        C[1][1] = 1.0f - 2.0f * (x * x + z * z); C[2][1] = 2.0f * (y * z + w * x);
    C[0][2] = 2.0f * (x * z + w * y);        C[1][2] = 2.0f * (y * z - w * x);        C[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

void quatMult(const float a[4], const float b[4], float o[4])
{
    o[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    o[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    o[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    o[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void normalize(float q[4])
{
    const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int i = 0; i < 4; ++i)
        q[i] /= n;
}

// A symmetric positive-definite 4x4's inverse (Gauss-Jordan, partial pivoting).
template <typename T>
bool invert4(const T A[4][4], T Ai[4][4])
{
    T M[4][8];
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
        {
            M[i][j] = A[i][j];
            M[i][4 + j] = T(i == j ? 1 : 0);
        }
    for (int c = 0; c < 4; ++c)
    {
        int p = c;
        for (int i = c + 1; i < 4; ++i)
            if (std::fabs(M[i][c]) > std::fabs(M[p][c]))
                p = i;
        if (!(std::fabs(M[p][c]) > T(1e-30)))
            return false;
        if (p != c)
            for (int j = 0; j < 8; ++j)
            {
                const T tmp = M[c][j];
                M[c][j] = M[p][j];
                M[p][j] = tmp;
            }
        const T d = T(1) / M[c][c];
        for (int j = 0; j < 8; ++j)
            M[c][j] *= d;
        for (int i = 0; i < 4; ++i)
        {
            if (i == c || M[i][c] == T(0))
                continue;
            const T f = M[i][c];
            for (int j = 0; j < 8; ++j)
                M[i][j] -= f * M[c][j];
        }
    }
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            Ai[i][j] = M[i][4 + j];
    return true;
}

}  // namespace

void HeadingFit::reset()
{
    started_ = false;
    hhead_ = 0;
    hn_ = 0;
    nobs_ = 0;
    sum_ = Sums();
    have_last_ = false;
}

void HeadingFit::start(const float q_seed[4], const float gyro_bias[3], double t0)
{
    reset();
    started_ = true;
    t0_ = t_ = t0;
    for (int i = 0; i < 4; ++i)
        q_[i] = q_seed[i];
    normalize(q_);
    for (int i = 0; i < 3; ++i)
    {
        wb_[i] = gyro_bias[i];
        dv_[i] = 0.0f;
    }
    float C[3][3];
    bodyToNed(q_, C);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
        {
            C0L_[i][j] = C[i][1 + j];
            BL_[i][j] = 0.0f;
        }
    Hist& h = hist_[hhead_];
    h.t = t0;
    for (int i = 0; i < 3; ++i)
        h.dv[i] = 0.0f;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            h.BL[i][j] = 0.0f;
    hhead_ = (hhead_ + 1) % kHist;
    hn_ = 1;
}

void HeadingFit::imu(const float acc[3], const float gyro[3], float dt)
{
    if (!started_)
        return;
    float C[3][3];
    bodyToNed(q_, C);
    for (int i = 0; i < 3; ++i)
        dv_[i] += (C[i][0] * acc[0] + C[i][1] * acc[1] + C[i][2] * acc[2]) * dt;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            BL_[i][j] += C[i][1 + j] * dt;
    float th[3];
    for (int i = 0; i < 3; ++i)
        th[i] = (gyro[i] - wb_[i]) * dt;
    const float a = std::sqrt(th[0] * th[0] + th[1] * th[1] + th[2] * th[2]);
    float dq[4];
    if (a > 1e-9f)
    {
        const float k = std::sin(0.5f * a) / a;
        dq[0] = std::cos(0.5f * a);
        dq[1] = k * th[0];
        dq[2] = k * th[1];
        dq[3] = k * th[2];
    }
    else
    {
        dq[0] = 1.0f;
        dq[1] = 0.5f * th[0];
        dq[2] = 0.5f * th[1];
        dq[3] = 0.5f * th[2];
    }
    float qn[4];
    quatMult(q_, dq, qn);
    for (int i = 0; i < 4; ++i)
        q_[i] = qn[i];
    normalize(q_);
    t_ += static_cast<double>(dt);
    Hist& h = hist_[hhead_];
    h.t = t_;
    for (int i = 0; i < 3; ++i)
        h.dv[i] = dv_[i];
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            h.BL[i][j] = BL_[i][j];
    hhead_ = (hhead_ + 1) % kHist;
    if (hn_ < kHist)
        hn_++;
}

bool HeadingFit::at(double t, float dv[3], float BL[2][2]) const
{
    if (hn_ == 0)
        return false;
    const Hist& newest = hist_[(hhead_ - 1 + kHist) % kHist];
    const Hist& oldest = hist_[(hhead_ - hn_ + kHist) % kHist];
    if (t > newest.t + 1e-9 || t < oldest.t - 1e-9)
        return false;
    for (int k = 0; k + 1 < hn_; ++k)
    {
        const Hist& b = hist_[(hhead_ - 1 - k + 2 * kHist) % kHist];
        const Hist& a = hist_[(hhead_ - 2 - k + 2 * kHist) % kHist];
        if (a.t <= t)
        {
            const float f = b.t <= a.t ? 0.0f : static_cast<float>((t - a.t) / (b.t - a.t));
            for (int i = 0; i < 3; ++i)
                dv[i] = a.dv[i] + f * (b.dv[i] - a.dv[i]);
            for (int i = 0; i < 2; ++i)
                for (int j = 0; j < 2; ++j)
                    BL[i][j] = a.BL[i][j] + f * (b.BL[i][j] - a.BL[i][j]);
            return true;
        }
    }
    for (int i = 0; i < 3; ++i)
        dv[i] = oldest.dv[i];
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            BL[i][j] = oldest.BL[i][j];
    return true;
}

bool HeadingFit::gnss(double t, const double v_ne[2], double var_h)
{
    if (!started_ || nobs_ >= kMaxEpochs)
        return false;
    float dv[3], BL[2][2];
    if (!at(t, dv, BL) || t <= t0_ ||
        v_ne[0] * v_ne[0] + v_ne[1] * v_ne[1] < cfg_.min_speed_mps * cfg_.min_speed_mps)
        return false;
    Obs& o = obs_[nobs_];
    o.t = t;
    for (int i = 0; i < 2; ++i)
    {
        o.y[i] = static_cast<float>(v_ne[i]);
        o.x[i] = dv[i];
        for (int j = 0; j < 2; ++j)
            o.M[i][j] = -(dv[2] / static_cast<float>(kG)) * C0L_[i][j] - BL[i][j];
    }
    const double d = cfg_.drift_mps2 * (t - t0_);
    const double w_raw = 1.0 / (var_h + d * d);
    double share = 1.0;
    if (have_last_)
    {
        share = (t - t_last_) / cfg_.tau_s;
        if (share > 1.0)
            share = 1.0;
    }
    have_last_ = true;
    t_last_ = t;
    o.w_raw = static_cast<float>(w_raw);
    o.w = static_cast<float>(w_raw * share);
    accumulate(sum_, o, 1.0);
    nobs_++;
    return true;
}

// One epoch's part of the sums, added (sign 1) or taken out (-1).
void HeadingFit::accumulate(Sums& s, const Obs& o, double sign)
{
    const double w = sign * static_cast<double>(o.w);
    const double u[2] = {o.y[0], o.y[1]};
    const double up[2] = {o.y[1], -o.y[0]};           // R(psi)^T y = cos psi u + sin psi up
    const double x[2] = {o.x[0], o.x[1]};
    for (int a = 0; a < 2; ++a)
    {
        const double Ma0 = o.M[0][a], Ma1 = o.M[1][a];
        for (int b = 0; b < 2; ++b)
            s.A[a][b] += w * (Ma0 * o.M[0][b] + Ma1 * o.M[1][b]);
        s.Su[a] += w * (Ma0 * u[0] + Ma1 * u[1]);
        s.Sp[a] += w * (Ma0 * up[0] + Ma1 * up[1]);
        s.Sx[a] += w * (Ma0 * x[0] + Ma1 * x[1]);
    }
    s.uu += w * (u[0] * u[0] + u[1] * u[1]);
    s.ux += w * (u[0] * x[0] + u[1] * x[1]);
    s.px += w * (up[0] * x[0] + up[1] * x[1]);
    s.xx += w * (x[0] * x[0] + x[1] * x[1]);
    s.n += sign > 0.0 ? 1 : -1;
}

// The sums over the epochs kept, afresh (the prototype's way).
void HeadingFit::sumsOf(const bool* keep, Sums& s) const
{
    s = Sums();
    for (int k = 0; k < nobs_; ++k)
        if (keep[k])
            accumulate(s, obs_[k], 1.0);
}

// The profile's coefficients from the sums and the biases' prior.
void HeadingFit::profile(const Sums& s, Profile& p) const
{
    const double prior = 1.0 / (cfg_.bias_sigma_mps2 * cfg_.bias_sigma_mps2);
    const double A[2][2] = {{s.A[0][0] + prior, s.A[0][1]}, {s.A[1][0], s.A[1][1] + prior}};
    for (int a = 0; a < 2; ++a)
    {
        p.Su[a] = s.Su[a];
        p.Sp[a] = s.Sp[a];
        p.Sx[a] = s.Sx[a];
    }
    const double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    p.Ai[0][0] = A[1][1] / det;
    p.Ai[0][1] = -A[0][1] / det;
    p.Ai[1][0] = -A[1][0] / det;
    p.Ai[1][1] = A[0][0] / det;
    auto quad = [&p](const double* a, const double* b) {
        return a[0] * (p.Ai[0][0] * b[0] + p.Ai[0][1] * b[1]) + a[1] * (p.Ai[1][0] * b[0] + p.Ai[1][1] * b[1]);
    };
    // cost = uu - 2(c ux + s px) + xx - g^T Ai g, g = c Su + s Sp - Sx
    p.a[0] = s.uu + s.xx - quad(p.Sx, p.Sx);
    p.a[1] = -2.0 * s.ux + 2.0 * quad(p.Su, p.Sx);
    p.a[2] = -2.0 * s.px + 2.0 * quad(p.Sp, p.Sx);
    p.a[3] = -quad(p.Su, p.Su);
    p.a[4] = -quad(p.Sp, p.Sp);
    p.a[5] = -2.0 * quad(p.Su, p.Sp);
}

bool HeadingFit::fit(const Sums& sums, bool full_grid, Result& r, double cs[2]) const
{
    const int n = sums.n;
    if (n < 2)
        return false;
    Profile p;
    profile(sums, p);
    const Turns& tr = kTurns;
    int evals = 0;

    // the grid's minimum (i; ci, si its cos and sin), and the profile's other
    // minimum if it has one (its rival: a separate valley, more than a coarse
    // stride away)
    int i = 0;
    double ci = 0.0, si = 0.0;
    bool has_rival = false;
    double rival_v = 0.0;
    auto rival = [&](int k, double v) {
        if (gridApart(k, i) > kCoarse && (!has_rival || v < rival_v))
        {
            has_rival = true;
            rival_v = v;
        }
    };
    if (full_grid)
    {
        double c = -1.0, s = 0.0, best = 0.0;
        Minima<double> mins;
        for (int k = 0; k < kGrid; ++k)
        {
            if (k % 64 == 0)
            {
                c = std::cos(gridAngle(k));       // re-anchored against the turns' rounding
                s = std::sin(gridAngle(k));
            }
            const double v = p.eval(c, s);
            if (k == 0 || v < best)
            {
                best = v;
                i = k;
            }
            mins.feed(v);
            turn(c, s, tr.c1, tr.s1);
        }
        mins.close();
        for (int m = 0; m < mins.n; ++m)
            rival(mins.k[m], mins.v[m]);
        evals += kGrid;
        ci = std::cos(gridAngle(i));
        si = std::sin(gridAngle(i));
    }
    else
    {
        // every tenth point (2.5 deg) -- a degree-two profile has no feature
        // narrower than that -- in float, as cost less cost(-pi): its two
        // harmonics' own terms, none larger than their coefficients (the
        // large constant left out, so nothing cancels). Each of the pass's
        // local minima is walked down the grid in double: the lowest valley
        // is the fit's, as the full grid's would be, and the next its rival
        const float K1 = static_cast<float>(p.a[1]), K2 = static_cast<float>(p.a[2]);
        const float K3 = static_cast<float>(0.5 * (p.a[3] - p.a[4])), K4 = static_cast<float>(0.5 * p.a[5]);
        const float cK = static_cast<float>(tr.cK), sK = static_cast<float>(tr.sK);
        float cf = -1.0f, sf = 0.0f, vlo = 0.0f;
        int lo = 0;
        Minima<float> mins;
        for (int j = 0; j < kGrid / kCoarse; ++j)
        {
            const float c2 = 2.0f * cf * cf - 1.0f, s2 = 2.0f * sf * cf;
            const float v = K1 * (cf + 1.0f) + K2 * sf + K3 * (c2 - 1.0f) + K4 * s2;
            if (j == 0 || v < vlo)
            {
                vlo = v;
                lo = j;
            }
            mins.feed(v);
            const float cn = cf * cK - sf * sK;
            sf = sf * cK + cf * sK;
            cf = cn;
        }
        mins.close();
        evals += kGrid / kCoarse;
        if (mins.n == 0)                                    // flat to float: its lowest point
            mins.take(lo, vlo);
        // where each walk ended: its grid point, value, cos and sin (the lowest first)
        int end_k[Minima<float>::kMax] = {};
        double end_v[Minima<float>::kMax] = {}, end_c[Minima<float>::kMax] = {}, end_s[Minima<float>::kMax] = {};
        for (int m = 0; m < mins.n; ++m)
        {
            int ii = mins.k[m] * kCoarse;
            double ci = tr.cc[mins.k[m]], si = tr.sc[mins.k[m]];
            double bi = p.eval(ci, si);
            evals++;
            for (int it = 0; it < kGrid; ++it)
            {
                double cm = ci, sm = si, cp = ci, sp = si;
                turn(cm, sm, tr.c1, -tr.s1);
                turn(cp, sp, tr.c1, tr.s1);
                const double vm = p.eval(cm, sm), vp = p.eval(cp, sp);
                evals += 2;
                if (vm < bi && vm <= vp)
                {
                    bi = vm;
                    ii = wrapIndex(ii - 1);
                    ci = cm;
                    si = sm;
                }
                else if (vp < bi)
                {
                    bi = vp;
                    ii = wrapIndex(ii + 1);
                    ci = cp;
                    si = sp;
                }
                else
                {
                    break;
                }
            }
            end_k[m] = ii;
            end_v[m] = bi;
            end_c[m] = ci;
            end_s[m] = si;
            if (m > 0 && (bi < end_v[0] || (bi == end_v[0] && ii < end_k[0])))   // the lowest first
            {
                end_k[m] = end_k[0];
                end_v[m] = end_v[0];
                end_c[m] = end_c[0];
                end_s[m] = end_s[0];
                end_k[0] = ii;
                end_v[0] = bi;
                end_c[0] = ci;
                end_s[0] = si;
            }
        }
        i = end_k[0];
        ci = end_c[0];
        si = end_s[0];
        for (int m = 1; m < mins.n; ++m)
            rival(end_k[m], end_v[m]);
    }

    // a parabola through the minimum and its neighbours
    double cm = ci, sm = si, cp = ci, sp = si;
    turn(cm, sm, tr.c1, -tr.s1);
    turn(cp, sp, tr.c1, tr.s1);
    const double vm = p.eval(cm, sm), v0 = p.eval(ci, si), vp = p.eval(cp, sp);
    evals += 3;
    const double den = vm - 2.0 * v0 + vp;
    double off = den > 0.0 ? 0.5 * (vm - vp) / den : 0.0;
    off = off < -1.0 ? -1.0 : (off > 1.0 ? 1.0 : off);
    double psi = gridAngle(i) + off * kStep;
    const double cmin = den > 0.0 ? v0 - 0.25 * (vm - vp) * off : v0;

    // the sigma: where the profile has risen by one on both sides (by the
    // fit's own scatter, when that is worse)
    const int dof = 2 * n - 1 > 1 ? 2 * n - 1 : 1;
    const double scale = cmin / dof > 1.0 ? cmin / dof : 1.0;
    auto risen = [&](double c, double s) { return (p.eval(c, s) - cmin) / scale >= 1.0; };
    int ks = -1;
    if (full_grid)
    {
        double cu = ci, su = si, cd = ci, sd = si;
        for (int k = 1; k < kWalk; ++k)
        {
            turn(cu, su, tr.c1, tr.s1);
            turn(cd, sd, tr.c1, -tr.s1);
            evals += 2;
            if (risen(cu, su) && risen(cd, sd))
            {
                ks = k;
                break;
            }
        }
    }
    else
    {
        // a stride at a time, then the steps of the stride that first rose
        double cu = ci, su = si, cd = ci, sd = si;
        int k0 = 0;
        while (ks < 0 && k0 < kWalk - 1)
        {
            const int kk = k0 + kCoarse < kWalk ? k0 + kCoarse : kWalk - 1;
            const double cu0 = cu, su0 = su, cd0 = cd, sd0 = sd;
            if (kk - k0 == kCoarse)
            {
                turn(cu, su, tr.cK, tr.sK);
                turn(cd, sd, tr.cK, -tr.sK);
            }
            else
            {
                for (int j = k0; j < kk; ++j)
                {
                    turn(cu, su, tr.c1, tr.s1);
                    turn(cd, sd, tr.c1, -tr.s1);
                }
            }
            evals += 2;
            if (risen(cu, su) && risen(cd, sd))
            {
                double cu2 = cu0, su2 = su0, cd2 = cd0, sd2 = sd0;
                for (int j = k0 + 1; j <= kk; ++j)
                {
                    turn(cu2, su2, tr.c1, tr.s1);
                    turn(cd2, sd2, tr.c1, -tr.s1);
                    evals += 2;
                    if (risen(cu2, su2) && risen(cd2, sd2))
                    {
                        ks = j;
                        break;
                    }
                }
                if (ks < 0)
                    ks = kk;
            }
            k0 = kk;
        }
    }

    double cc = ci, ss = si;
    turnBy(cc, ss, off * kStep);
    cs[0] = cc;
    cs[1] = ss;
    const double g[2] = {cc * p.Su[0] + ss * p.Sp[0] - p.Sx[0], cc * p.Su[1] + ss * p.Sp[1] - p.Sx[1]};
    if (psi >= kPi)
        psi -= 2.0 * kPi;
    if (psi < -kPi)
        psi += 2.0 * kPi;
    r.ok = true;
    r.psi = psi;
    r.sigma = ks > 0 ? ks * kStep : -1.0;
    r.rival = has_rival ? (rival_v - cmin) / scale : -1.0;
    r.bias[0] = p.Ai[0][0] * g[0] + p.Ai[0][1] * g[1];
    r.bias[1] = p.Ai[1][0] * g[0] + p.Ai[1][1] * g[1];
    r.n = n;
    r.rms = std::sqrt((cmin > 0.0 ? cmin : 0.0) / (2 * n > 1 ? 2 * n : 1));
    r.evals += evals;
    return true;
}

// c, s: cos and sin of r.psi (once a fit, not once an epoch)
double HeadingFit::chi2(const Obs& o, const Result& r, double c, double s) const
{
    double z[2] = {c * o.y[0] + s * o.y[1], -s * o.y[0] + c * o.y[1]};
    for (int i = 0; i < 2; ++i)
        z[i] -= o.x[i] + o.M[i][0] * r.bias[0] + o.M[i][1] * r.bias[1];
    return static_cast<double>(o.w_raw) * (z[0] * z[0] + z[1] * z[1]);
}

HeadingFit::Result HeadingFit::solveWith(bool full_grid) const
{
    Result r;
    if (nobs_ < 2)
        return r;
    bool keep[kMaxEpochs];
    for (int k = 0; k < nobs_; ++k)
        keep[k] = true;
    // the running sums (an epoch cut taken back out), or -- the prototype's
    // way, the check -- the kept epochs' afresh at each fit
    Sums sums = sum_;
    int kept = nobs_, cut = 0, evals = 0;
    const int max_cut = nobs_ / 4 > 1 ? nobs_ / 4 : 1;
    for (;;)
    {
        if (full_grid)
            sumsOf(keep, sums);
        Result f;
        f.evals = evals;
        double cs[2];
        if (!fit(sums, full_grid, f, cs))
            return Result();
        evals = f.evals;
        // drop the worst epoch while it is out of the 2-dof gate, refitting
        // each time: at least three stay, and at most a quarter go
        int worst = -1;
        double worst_chi = -1.0;
        const double cp = cs[0], sp = cs[1];
        if (full_grid)
        {
            for (int k = 0; k < nobs_; ++k)
            {
                if (!keep[k])
                    continue;
                const double c = chi2(obs_[k], f, cp, sp);
                if (c > worst_chi)
                {
                    worst_chi = c;
                    worst = k;
                }
            }
        }
        else
        {
            // each epoch's residual is metres a second against tens of them: float holds it
            const float c = static_cast<float>(cp), s = static_cast<float>(sp);
            const float b0 = static_cast<float>(f.bias[0]), b1 = static_cast<float>(f.bias[1]);
            float wc = -1.0f;
            for (int k = 0; k < nobs_; ++k)
            {
                if (!keep[k])
                    continue;
                const Obs& o = obs_[k];
                const float z0 = c * o.y[0] + s * o.y[1] - o.x[0] - o.M[0][0] * b0 - o.M[0][1] * b1;
                const float z1 = -s * o.y[0] + c * o.y[1] - o.x[1] - o.M[1][0] * b0 - o.M[1][1] * b1;
                const float chi = o.w_raw * (z0 * z0 + z1 * z1);
                if (chi > wc)
                {
                    wc = chi;
                    worst = k;
                }
            }
            worst_chi = wc;
        }
        if (worst_chi <= cfg_.cut_chi2 || kept <= 3 || cut >= max_cut)
        {
            f.n_cut = cut;
            return f;
        }
        keep[worst] = false;
        if (!full_grid)
            accumulate(sums, obs_[worst], -1.0);
        kept--;
        cut++;
    }
}

void HeadingFit::attitude(double psi, float q[4]) const
{
    const float h = static_cast<float>(0.5 * psi);
    const float qz[4] = {std::cos(h), 0.0f, 0.0f, std::sin(h)};
    quatMult(qz, q_, q);
    normalize(q);
}

double headingBetween(const float a[4], const float b[4])
{
    // r = a conj(b); its twist about z is [r0, 0, 0, r3]
    const double a0 = a[0], a1 = a[1], a2 = a[2], a3 = a[3];
    const double b0 = b[0], b1 = -b[1], b2 = -b[2], b3 = -b[3];
    const double r0 = a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3;
    const double r3 = a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0;
    double h = 2.0 * std::atan2(r3, r0);
    if (h >= kPi)
        h -= 2.0 * kPi;
    if (h < -kPi)
        h += 2.0 * kPi;
    return h;
}

bool dopplerVelocity(const RawMeas* m, int n, const double r[3], double v[3], double cov[3][3], int& used,
                     int min_sats, double cut)
{
    // each satellite's line of sight and a still receiver's range rate, in
    // float after one double difference: the Earth's turn over the travel time
    // (under 1e-4 rad) to first order, a millimetre of the satellite's place
    // and a hundredth of a millimetre a second of its velocity
    constexpr double kTurn = 7.2921151467e-5 / 299792458.0;   // rad per metre of travel
    struct Row
    {
        float u[3];    // the line of sight (the solve's H row is -u, 1)
        float w;       // 1 / sigma^2
        float y;       // the range rate less a still receiver's
    };
    Row rows[RawEpoch::kMaxObs];
    bool use[RawEpoch::kMaxObs];
    int nr = 0;
    for (int i = 0; i < n && nr < RawEpoch::kMaxObs; ++i)
    {
        const RawMeas& mi = m[i];
        if (!mi.has_rr || !(mi.sigma_rr > 0.0f))
            continue;
        float d[3] = {static_cast<float>(mi.sat_pos[0] - r[0]), static_cast<float>(mi.sat_pos[1] - r[1]),
                      static_cast<float>(mi.sat_pos[2] - r[2])};
        const float th = static_cast<float>(kTurn) * std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        const float sx = static_cast<float>(mi.sat_pos[0]), sy = static_cast<float>(mi.sat_pos[1]);
        d[0] += th * sy;
        d[1] -= th * sx;
        const float rho = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        Row& w = rows[nr];
        for (int a = 0; a < 3; ++a)
            w.u[a] = d[a] / rho;
        const float vx = static_cast<float>(mi.sat_vel[0]), vy = static_cast<float>(mi.sat_vel[1]);
        const float rate = w.u[0] * (vx + th * vy) + w.u[1] * (vy - th * vx) + w.u[2] * static_cast<float>(mi.sat_vel[2]);
        w.y = static_cast<float>(mi.rr - static_cast<double>(rate));
        w.w = 1.0f / (mi.sigma_rr * mi.sigma_rr);
        use[nr] = true;
        nr++;
    }
    // the solve in float too, refined once: a part in ten million of the
    // clock's and the receiver's hundreds of m/s, times the conditioning of
    // the few high satellites a boost leaves, is centimetres a second; one
    // pass on its own residuals takes that to a fraction of a millimetre
    int nuse = nr;
    while (nuse >= min_sats)
    {
        float N[4][4] = {}, b[4] = {};
        for (int k = 0; k < nr; ++k)
        {
            if (!use[k])
                continue;
            const Row& w = rows[k];
            const float h[4] = {-w.u[0], -w.u[1], -w.u[2], 1.0f};
            for (int a = 0; a < 4; ++a)
            {
                const float wh = w.w * h[a];
                b[a] += wh * w.y;
                for (int c = a; c < 4; ++c)
                    N[a][c] += wh * h[c];
            }
        }
        for (int a = 0; a < 4; ++a)
            for (int c = 0; c < a; ++c)
                N[a][c] = N[c][a];
        float Ni[4][4];
        if (!invert4(N, Ni))
            return false;
        float x[4];
        for (int a = 0; a < 4; ++a)
            x[a] = Ni[a][0] * b[0] + Ni[a][1] * b[1] + Ni[a][2] * b[2] + Ni[a][3] * b[3];
        float g[4] = {};
        for (int k = 0; k < nr; ++k)
        {
            if (!use[k])
                continue;
            const Row& w = rows[k];
            const float we = w.w * (w.y - (-w.u[0] * x[0] - w.u[1] * x[1] - w.u[2] * x[2] + x[3]));
            g[0] -= we * w.u[0];
            g[1] -= we * w.u[1];
            g[2] -= we * w.u[2];
            g[3] += we;
        }
        for (int a = 0; a < 4; ++a)
            x[a] += Ni[a][0] * g[0] + Ni[a][1] * g[1] + Ni[a][2] * g[2] + Ni[a][3] * g[3];
        int worst = -1;
        float worst_res = -1.0f;
        for (int k = 0; k < nr; ++k)
        {
            if (!use[k])
                continue;
            const Row& w = rows[k];
            const float e = w.y - (-w.u[0] * x[0] - w.u[1] * x[1] - w.u[2] * x[2] + x[3]);
            const float res = e * e * w.w;          // squared, against cut^2
            if (res > worst_res)
            {
                worst_res = res;
                worst = k;
            }
        }
        if (worst_res <= static_cast<float>(cut * cut))
        {
            for (int a = 0; a < 3; ++a)
            {
                v[a] = x[a];
                for (int c = 0; c < 3; ++c)
                    cov[a][c] = Ni[a][c];
            }
            used = nuse;
            return true;
        }
        use[worst] = false;
        nuse--;
    }
    return false;
}

bool dopplerVelocityNed(const RawMeas* m, int n, const double r[3], double lat, double lon, double v_ned[3],
                        double& var_h, int& used, int min_sats, double cut)
{
    double v[3], cov[3][3];
    if (!dopplerVelocity(m, n, r, v, cov, used, min_sats, cut))
        return false;
    // in float: a velocity's turn needs no more
    const float la = static_cast<float>(lat), lo = static_cast<float>(lon);
    const float sl = std::sin(la), cl = std::cos(la), so = std::sin(lo), co = std::cos(lo);
    const float R[3][3] = {{-sl * co, -sl * so, cl}, {-so, co, 0.0f}, {-cl * co, -cl * so, -sl}};   // ECEF -> NED
    float vf[3], C[3][3];
    for (int i = 0; i < 3; ++i)
    {
        vf[i] = static_cast<float>(v[i]);
        for (int j = 0; j < 3; ++j)
            C[i][j] = static_cast<float>(cov[i][j]);
    }
    float var2 = 0.0f;
    for (int i = 0; i < 3; ++i)
    {
        v_ned[i] = R[i][0] * vf[0] + R[i][1] * vf[1] + R[i][2] * vf[2];
        if (i == 2)
            break;
        for (int j = 0; j < 3; ++j)
            var2 += R[i][j] * (C[j][0] * R[i][0] + C[j][1] * R[i][1] + C[j][2] * R[i][2]);
    }
    var_h = 0.5 * static_cast<double>(var2);
    return true;
}

}  // namespace gnss_raw
