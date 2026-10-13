// SatJumpCheck.cpp — see SatJumpCheck.h
#include "SatJumpCheck.h"

#include <cmath>

#include "TR_RawGnssEKF.h"

namespace gnss_raw
{

namespace
{

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

double towDiff(double a, double b)
{
    double d = a - b;
    if (d < -302400.0)
        d += 604800.0;
    else if (d > 302400.0)
        d -= 604800.0;
    return d;
}

}  // namespace

void SatJumpCheck::reset()
{
    for (auto& sys : sat_)
        for (Sat& s : sys)
            s = Sat();
    sum_ = Summary();
}

int SatJumpCheck::check(const RawMeas* m, int n, double tow_s, const double r_prior[3], bool* flagged)
{
    sum_ = Summary();
    double d[RawEpoch::kMaxObs] = {};
    int idx[RawEpoch::kMaxObs];
    int nd = 0;
    for (int i = 0; i < n; ++i)
    {
        flagged[i] = false;
        const RawMeas& mi = m[i];
        if (!mi.has_pr || mi.sys >= kSys || mi.prn >= kPrn)
            continue;
        Sat& s = sat_[mi.sys][mi.prn];
        s.pr_new = mi.pr;
        s.tow_new = tow_s;
        const double dt = s.tow < 0.0 ? -1.0 : towDiff(tow_s, s.tow);
        if (!(dt > 0.0 && dt <= cfg_.max_dt_s) || nd >= RawEpoch::kMaxObs)
            continue;
        // the pseudorange's change less the range's, as the IMU carried the receiver
        d[nd] = (mi.pr - s.pr) - (sagnacRange(mi, r_prior) - s.range);
        idx[nd++] = i;
    }
    sum_.judged = nd;
    if (nd < cfg_.min_sats)
        return 0;
    double sorted[RawEpoch::kMaxObs] = {};
    for (int k = 0; k < nd; ++k)
        sorted[k] = d[k];
    const double clock = median(sorted, nd);
    sum_.clock_m = clock;
    for (int k = 0; k < nd; ++k)
    {
        const double off = std::fabs(d[k] - clock);
        if (off > sum_.worst_m)
            sum_.worst_m = off;
        if (off > cfg_.jump_m)
        {
            flagged[idx[k]] = true;
            sum_.flagged++;
        }
    }
    return sum_.flagged;
}

void SatJumpCheck::commit(const RawMeas* m, int n, double tow_s, const double r_post[3])
{
    for (int i = 0; i < n; ++i)
    {
        const RawMeas& mi = m[i];
        if (!mi.has_pr || mi.sys >= kSys || mi.prn >= kPrn)
            continue;
        Sat& s = sat_[mi.sys][mi.prn];
        if (s.tow_new != tow_s)
            continue;                     // check() did not see it this epoch
        s.pr = s.pr_new;
        s.range = sagnacRange(mi, r_post);
        s.tow = tow_s;
    }
}

}  // namespace gnss_raw
