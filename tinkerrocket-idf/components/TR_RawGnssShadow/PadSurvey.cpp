// PadSurvey.cpp — see PadSurvey.h
#include "PadSurvey.h"

#include <cmath>

#include "TR_RawGnssEKF.h"

namespace gnss_raw
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

// The median of v[0..n) (sorted in place; the mean of the middle two for an even count).
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

// Solve the n x n system a x = b in place (Gaussian elimination, partial
// pivoting); false if it is singular.
bool solve(double a[6][6], double b[6], int n, double x[6])
{
    for (int c = 0; c < n; ++c)
    {
        int p = c;
        for (int r = c + 1; r < n; ++r)
            if (std::fabs(a[r][c]) > std::fabs(a[p][c]))
                p = r;
        if (std::fabs(a[p][c]) < 1.0e-9)
            return false;
        if (p != c)
        {
            for (int k = 0; k < n; ++k)
            {
                const double t = a[c][k];
                a[c][k] = a[p][k];
                a[p][k] = t;
            }
            const double t = b[c];
            b[c] = b[p];
            b[p] = t;
        }
        for (int r = c + 1; r < n; ++r)
        {
            const double f = a[r][c] / a[c][c];
            for (int k = c; k < n; ++k)
                a[r][k] -= f * a[c][k];
            b[r] -= f * b[c];
        }
    }
    for (int r = n - 1; r >= 0; --r)
    {
        double s = b[r];
        for (int k = r + 1; k < n; ++k)
            s -= a[r][k] * x[k];
        x[r] = s / a[r][r];
    }
    return true;
}

}  // namespace

double PadSurvey::towDiff(double a, double b)
{
    double d = a - b;
    if (d < -302400.0)
        d += 604800.0;
    else if (d > 302400.0)
        d -= 604800.0;
    return d;
}

void PadSurvey::reset()
{
    for (auto& sys : sat_)
        for (Sat& s : sys)
            s = Sat();
    have_pad_ = false;
    sum_ = Summary();
}

void PadSurvey::observe(const RawMeas* m, int n, const double* lock_age_s, double tow_s, const double pad[3])
{
    if (have_pad_)
    {
        const double d0 = pad[0] - pad_[0], d1 = pad[1] - pad_[1], d2 = pad[2] - pad_[2];
        if (d0 * d0 + d1 * d1 + d2 * d2 > cfg_.pad_move_m * cfg_.pad_move_m)
            reset();
    }
    for (int a = 0; a < 3; ++a)
        pad_[a] = pad[a];
    have_pad_ = true;

    // each settled satellite's residual against the pad
    struct Resid
    {
        uint8_t sys, prn;
        double  res;
        double  u[3];   // the line of sight from the pad
    };
    Resid res[RawEpoch::kMaxObs];
    int nr = 0;
    int tracked = 0, settling = 0;
    for (int i = 0; i < n && nr < RawEpoch::kMaxObs; ++i)
    {
        const RawMeas& mi = m[i];
        if (!mi.has_pr || mi.sys >= kSys || mi.prn >= kPrn)
            continue;
        tracked++;
        Sat& s = sat_[mi.sys][mi.prn];
        const bool relocked = s.seen && towDiff(tow_s, s.last_obs) > cfg_.gap_s;
        const bool young = lock_age_s != nullptr && lock_age_s[i] < cfg_.settle_s;
        if (relocked || young)
            s = Sat();
        s.seen = true;
        s.last_obs = tow_s;
        if (young)
        {
            settling++;
            continue;
        }
        Resid& rk = res[nr++];
        rk.sys = mi.sys;
        rk.prn = mi.prn;
        rk.res = mi.pr - sagnacRange(mi, pad);
        double d = 0.0;
        for (int a = 0; a < 3; ++a)
        {
            rk.u[a] = mi.sat_pos[a] - pad[a];
            d += rk.u[a] * rk.u[a];
        }
        d = std::sqrt(d);
        for (int a = 0; a < 3; ++a)
            rk.u[a] /= d;
    }

    // per system, the clock and its offset: the median of the residuals less
    // their corrections, over the satellites in use (else those started, else all)
    for (int sys = 0; sys < kSys; ++sys)
    {
        double v[RawEpoch::kMaxObs];
        int nv = 0;
        for (int pass = 0; pass < 3 && nv == 0; ++pass)
            for (int k = 0; k < nr; ++k)
            {
                if (res[k].sys != sys)
                    continue;
                const Sat& s = sat_[sys][res[k].prn];
                if (pass == 0 ? usable(s) : (pass == 1 ? s.n > 0 : true))
                    v[nv++] = res[k].res - s.corr;
            }
        if (nv == 0)
            continue;
        const double b = median(v, nv);
        for (int k = 0; k < nr; ++k)
        {
            if (res[k].sys != sys)
                continue;
            Sat& s = sat_[sys][res[k].prn];
            const double x = res[k].res - b;
            if (s.n == 0)
            {
                s.corr = x;
                s.first = s.last_upd = tow_s;
                s.n = 1;
                continue;
            }
            if (usable(s) && std::fabs(x - s.corr) > cfg_.max_resid_m)
                continue;
            const double dt = towDiff(tow_s, s.last_upd);
            const double cum = 1.0 / static_cast<double>(s.n + 1);
            const double ew = 1.0 - std::exp(-(dt > 0.0 ? dt : 0.0) / cfg_.tau_s);
            s.corr += (cum > ew ? cum : ew) * (x - s.corr);
            s.n++;
            s.last_upd = tow_s;
        }
    }

    // the summary: how many are in use, and their spread about each system's mean
    int used = 0;
    double ss = 0.0;
    for (int sys = 0; sys < kSys; ++sys)
    {
        double sum = 0.0, sum2 = 0.0;
        int k = 0;
        for (int j = 0; j < nr; ++j)
        {
            if (res[j].sys != sys)
                continue;
            const Sat& s = sat_[sys][res[j].prn];
            if (!usable(s))
                continue;
            sum += s.corr;
            sum2 += s.corr * s.corr;
            k++;
        }
        if (k > 0)
        {
            ss += sum2 - sum * sum / k;
            used += k;
        }
    }
    // Where the corrections in use put the pad: corr = clock(system) - u . dp,
    // dp the offset from the receiver's average, by least squares. A clock
    // per system seen, so it needs two satellites more than unknowns.
    double nrm[6][6] = {}, rhs[6] = {}, x[6] = {};
    int col[kSys] = {-1, -1, -1};
    int nu = 3, nused = 0;
    for (int k = 0; k < nr; ++k)
    {
        const Sat& s = sat_[res[k].sys][res[k].prn];
        if (!usable(s))
            continue;
        if (col[res[k].sys] < 0)
            col[res[k].sys] = nu++;
        double row[6] = {-res[k].u[0], -res[k].u[1], -res[k].u[2], 0.0, 0.0, 0.0};
        row[col[res[k].sys]] = 1.0;
        for (int i = 0; i < nu; ++i)
        {
            rhs[i] += row[i] * s.corr;
            for (int j = 0; j < nu; ++j)
                nrm[i][j] += row[i] * row[j];
        }
        nused++;
    }
    sum_.pad_ok = false;
    sum_.pad_offset_m = 0.0f;
    if (nused >= nu + 2 && solve(nrm, rhs, nu, x))
    {
        const double off = std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
        sum_.pad_offset_m = static_cast<float>(off);
        sum_.pad_ok = off <= cfg_.max_pad_offset_m;
    }

    auto u8 = [](int x) { return static_cast<uint8_t>(x > 255 ? 255 : x); };
    sum_.tracked = u8(tracked);
    sum_.settling = u8(settling);
    sum_.corrected = u8(used);
    sum_.rms_m = used > 0 ? static_cast<float>(std::sqrt((ss > 0.0 ? ss : 0.0) / used)) : 0.0f;
    sum_.last_tow_s = tow_s;
}

int PadSurvey::correct(RawMeas* m, int n, const double r[3]) const
{
    // a pad the raw measurements do not vouch for corrects nothing
    if (!sum_.pad_ok)
        return 0;
    // with none in use every satellite keeps the same ionosphere, much of it
    // common, which the filter's clock takes up: the measurements are left as
    // they were (phase 4's shadow), not all widened alike
    int in_use = 0;
    for (int i = 0; i < n; ++i)
        if (m[i].has_pr && m[i].sys < kSys && m[i].prn < kPrn && usable(sat_[m[i].sys][m[i].prn]))
            in_use++;
    if (in_use == 0)
        return 0;
    const double rn = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    int k = 0;
    for (int i = 0; i < n; ++i)
    {
        RawMeas& mi = m[i];
        if (!mi.has_pr || mi.sys >= kSys || mi.prn >= kPrn)
            continue;
        const Sat& s = sat_[mi.sys][mi.prn];
        if (usable(s))
        {
            mi.pr -= s.corr;
            k++;
            continue;
        }
        // no correction: the ionosphere it keeps, by Klobuchar's obliquity
        // factor at its elevation (geocentric: within a fifth of a degree)
        double d[3], dn = 0.0, dot = 0.0;
        for (int a = 0; a < 3; ++a)
        {
            d[a] = mi.sat_pos[a] - r[a];
            dn += d[a] * d[a];
            dot += d[a] * r[a];
        }
        dn = std::sqrt(dn);
        double sin_el = (dn > 0.0 && rn > 0.0) ? dot / (dn * rn) : 1.0;
        sin_el = sin_el > 1.0 ? 1.0 : (sin_el < 0.0 ? 0.0 : sin_el);
        const double e = 0.53 - std::asin(sin_el) / kPi;
        const double extra = static_cast<double>(cfg_.iono_zenith_m) * (1.0 + 16.0 * e * e * e);
        const double sc = static_cast<double>(mi.sigma_pr_corr);
        mi.sigma_pr_corr = static_cast<float>(std::sqrt(sc * sc + extra * extra));
    }
    return k;
}

bool PadSurvey::correction(int sys, int prn, double& corr_m) const
{
    if (sys < 0 || sys >= kSys || prn < 0 || prn >= kPrn)
        return false;
    const Sat& s = sat_[sys][prn];
    if (!usable(s))
        return false;
    corr_m = s.corr;
    return true;
}

}  // namespace gnss_raw
