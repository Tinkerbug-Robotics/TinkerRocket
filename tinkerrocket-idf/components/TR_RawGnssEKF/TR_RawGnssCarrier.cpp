// TR_RawGnssCarrier.cpp — TR_RawGnssEKF's carrier range rate (see
// TR_RawGnssEKF.h): the screen, the fusion with the epoch's common rate, the
// epoch's close, and the integration's excess back-dated; the boost screen's
// gate (screenRangeRates); the Doppler change check's rows (dopplerChange),
// the heading share (hdgObsScaleNow) and the antenna's offset where it is
// known (lever_on: its history, its back-dating, its rows -- the history's
// one write a step, pushLever, is the exception to what follows; the Beetle
// never turns it on), and the epoch's shared geometry (share_los: losOf,
// frameOf, injectLla, which every update's inject takes with it on, and the
// share_los form of beginGnssRaw and updateGnssRawPart, RawGnssEKFUpdate.h's;
// the Beetle never turns it on either). Once an epoch, so apart from the
// filter's per-step code: on the Beetle this file stays in flash where the
// rest of the filter runs from internal RAM (raw_gnss_iram.lf), 11 kB less of
// it. Its per-step part -- the receiver snapshots that every correction moves
// (inject), the roll rate and the integration's excess (propagate,
// pushHistory) -- is TR_RawGnssEKF.cpp's, and the geometry both predict with
// RawGnssEKFGeometry.h's.
#include "TR_RawGnssEKF.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <cstring>

#include "RawGnssEKFGeometry.h"
#include "RawGnssEKFUpdate.h"
#include "gnss/types.h"

namespace gnss_raw
{

namespace
{

constexpr double kC      = 299792458.0;
constexpr double kTwoPi  = 6.283185307179586;
// An epoch's snapshot is the epoch screened if their times are this close
// (the filter's time is a sum of float steps), and a rate's interval matches
// the snapshots' to this (consecutive epochs are a second apart: a mismatch
// is a whole epoch)
constexpr double kCrSameEpochS = 0.01;
constexpr double kCrIntervalTolS = 0.1;

// the geometry every update predicts with (RawGnssEKFGeometry.h): the boost
// screen's Dopplers predicted as updateGnssRawPart predicts them
using ekf_geom::Los;
using ekf_geom::lineOfSight;
using ekf_geom::satRangeAccel;
using ekf_geom::corrInflation;
using ekf_geom::backDate;

constexpr double kNever = -1.0e9;   // no boost yet (TR_RawGnssEKF.cpp's kNever)

// body -> NED from the filter's q (scalar first; TR_RawGnssEKF.cpp's quat2dcm
// is NED -> body, this its transpose)
template <typename T>
void bodyToNed(const T q[4], T C[3][3])
{
    const T w = q[0], x = q[1], y = q[2], z = q[3];
    const T one = T(1), two = T(2);
    C[0][0] = one - two * (y * y + z * z); C[1][0] = two * (x * y + w * z);       C[2][0] = two * (x * z - w * y);
    C[0][1] = two * (x * y - w * z);       C[1][1] = one - two * (x * x + z * z); C[2][1] = two * (y * z + w * x);
    C[0][2] = two * (x * z + w * y);       C[1][2] = two * (y * z - w * x);       C[2][2] = one - two * (x * x + y * y);
}

}  // namespace

// This epoch's receiver, age_s back: now less the history's displacement
// since (NED, into ECEF by the present frame: beginGnssRaw's back-dating),
// and the integration's excess to then. Corrections from here move it
// (inject).
template <typename T>
void RawGnssEKF<T>::crSnapshot(double age_s, const T dp_e[3])
{
    double r[3], v[3], Tm[3][3], ex[3];
    receiverState(r, v, Tm);
    historyExcess(age_s, ex);
    for (int i = 0; i < 3; ++i)
    {
        double sp = 0.0;
        for (int k = 0; k < 3; ++k)
            sp += Tm[k][i] * static_cast<double>(dp_e[k]);
        cr_cur_.r[i] = r[i] - sp;
        cr_cur_.d_ned[i] = 0.0;
        cr_cur_.ex[i] = cr_ex_[i] - ex[i];
    }
    cr_cur_.t = t_ - age_s;
    cr_cur_.valid = true;
}

// The integration excess over the last tau: whole entries' and the part of
// the one tau falls in after it (a step's excess grows as the square of its
// time: (1 - w)^2 of it). Past the history: the last acceleration's.
template <typename T>
void RawGnssEKF<T>::historyExcess(double tau, double ex[3]) const
{
    for (int i = 0; i < 3; ++i)
        ex[i] = 0.0;
    if (tau <= 0.0)
        return;
    const double t0 = t_ - tau;
    auto at = [&](int k) -> const Hist& { return hist_[(hhead_ + kHist - hn_ + k) % kHist]; };
    if (hn_ > 1 && at(0).t <= t0 && at(hn_ - 1).t >= t_ - 1e-9)
    {
        double s[3] = {0.0, 0.0, 0.0};
        for (int k = hn_ - 1; k > 0; --k)
        {
            const Hist& b = at(k);
            const Hist& a = at(k - 1);
            if (a.t <= t0)
            {
                const double w = b.t > a.t ? (t0 - a.t) / (b.t - a.t) : 0.0;
                for (int i = 0; i < 3; ++i)
                    ex[i] = s[i] + (1.0 - w) * (1.0 - w) * static_cast<double>(b.ex[i]);
                return;
            }
            for (int i = 0; i < 3; ++i)
                s[i] += static_cast<double>(b.ex[i]);
        }
    }
    for (int i = 0; i < 3; ++i)
        ex[i] = 0.5 * static_cast<double>(a_ned_[i]) * tau * tau;
}

// One carrier rate with the epoch's common rate c on the augmented state
// [x; c]: c's H is 1, its cross-covariance P_xc and variance P_cc beside P.
// The same Joseph form as scalarUpdate, written out for the extra row and
// column: P1 = Pa - K (h'Pa) (not symmetric when the gain is gated), A = P1
// h, Pa = P1 - A K' + K R K', symmetrized. The state's gain gated as the
// other GNSS updates' (gateGain); c's is its own.
template <typename T>
int RawGnssEKF<T>::carrierUpdate(const int* hidx, const T* hval, int hn, T y, T R, T att, const T* att_axis,
                                 T dx[N], T& dc, T* nis)
{
    T HP[N];
    for (int j = 0; j < N; ++j)
        HP[j] = cr_pxc_[j];
    for (int m = 0; m < hn; ++m)
    {
        const T h = hval[m];
        const T* row = P_[hidx[m]];
        for (int j = 0; j < N; ++j)
            HP[j] += h * row[j];
    }
    T HPc = cr_pcc_;
    for (int m = 0; m < hn; ++m)
        HPc += hval[m] * cr_pxc_[hidx[m]];
    T S = R + HPc;
    for (int m = 0; m < hn; ++m)
        S += HP[hidx[m]] * hval[m];
    if (!(S > T(0)))
        return -1;
    const T Sinv = T(1) / S;
    if (nis)
        *nis = y * y * Sinv;
    T K[N];
    for (int i = 0; i < N; ++i)
        K[i] = HP[i] * Sinv;
    const T Kc = HPc * Sinv;
    gateGain(K, att, att_axis);

    // P1 = Pa - K (h'Pa): the state's block in place, c's column (state rows)
    // and row (state columns) apart
    T colc[N], rowc[N];
    for (int i = 0; i < N; ++i)
    {
        for (int j = 0; j < N; ++j)
            P_[i][j] -= K[i] * HP[j];
        colc[i] = cr_pxc_[i] - K[i] * HPc;
        rowc[i] = cr_pxc_[i] - Kc * HP[i];
    }
    T pcc = cr_pcc_ - Kc * HPc;
    // A = P1 h
    T A[N];
    for (int i = 0; i < N; ++i)
    {
        T a = colc[i];
        for (int m = 0; m < hn; ++m)
            a += P_[i][hidx[m]] * hval[m];
        A[i] = a;
    }
    T Ac = pcc;
    for (int m = 0; m < hn; ++m)
        Ac += rowc[hidx[m]] * hval[m];
    // Pa = P1 - A K' + K R K'
    for (int i = 0; i < N; ++i)
    {
        const T KR = K[i] * R;
        for (int j = 0; j < N; ++j)
            P_[i][j] += -A[i] * K[j] + KR * K[j];
        colc[i] += -A[i] * Kc + KR * Kc;
        rowc[i] += -Ac * K[i] + Kc * R * K[i];
    }
    pcc += -Ac * Kc + Kc * R * Kc;
    symmetrize();
    for (int i = 0; i < N; ++i)
        cr_pxc_[i] = T(0.5) * (colc[i] + rowc[i]);
    cr_pcc_ = pcc;
    for (int i = 0; i < N; ++i)
        dx[i] = K[i] * y;
    dc = Kc * y;
    return 1;
}

// The epoch's receiver and the one before's, each as its snapshot and the
// corrections since (NED, carried into ECEF by the present frame: the frame
// turns 1.6e-4 rad a kilometre, the corrections are metres at most).
template <typename T>
void RawGnssEKF<T>::crEnds(const double Tm[3][3], double rk[3], double rp[3]) const
{
    // and the epoch's end less the integration's excess over the interval:
    // the displacement the velocity's own path gives
    double dk[3];
    for (int k = 0; k < 3; ++k)
        dk[k] = cr_cur_.d_ned[k] - (cr_cur_.ex[k] - cr_prev_.ex[k]);
    for (int i = 0; i < 3; ++i)
    {
        double sk = 0.0, sp = 0.0;
        for (int k = 0; k < 3; ++k)
        {
            sk += Tm[k][i] * dk[k];
            sp += Tm[k][i] * cr_prev_.d_ned[k];
        }
        rk[i] = cr_cur_.r[i] + sk;
        rp[i] = cr_prev_.r[i] + sp;
    }
}

template <typename T>
double RawGnssEKF<T>::carrierRate(const RawMeas& m, const double Tm[3][3], T u_ned[3]) const
{
    double rk[3], rp[3], uk[3], up[3];
    crEnds(Tm, rk, rp);
    const double rho_k = ekf_detail::sagnacLos(m.sat_pos, rk, uk);
    const double rho_p = ekf_detail::sagnacLos(m.sat_pos_prev, rp, up);
    for (int a = 0; a < 3; ++a)
        u_ned[a] = T(Tm[a][0] * uk[0] + Tm[a][1] * uk[1] + Tm[a][2] * uk[2]);
    return (m.cr_dphi_m - (rho_k - rho_p)) / static_cast<double>(m.cr_dt);
}

template <typename T>
int RawGnssEKF<T>::beginCarrier(const RawMeas* m, int n, double age_s, RawGnssStats& st)
{
    cr_ninfo_ = cr_nfuse_ = 0;
    if (p_.cr_mode == kCarrierOff)
        return 0;
    if (age_s < 0.0)
        age_s = 0.0;
    // the roll rate since the last screen: its share of each rate's noise
    const double roll = cr_roll_t_ > 0.0 ? cr_roll_sum_ / cr_roll_t_ / kTwoPi : 0.0;   // rev/s
    cr_roll_sum_ = cr_roll_t_ = 0.0;
    st.cr_roll_rev_s = static_cast<float>(roll);
    const double s_roll = p_.cr_sigma_per_rev_s * roll;
    // this epoch's receiver (taken at its beginGnssRaw) and the one before's
    const bool ends = cr_cur_.valid && cr_prev_.valid && std::fabs(cr_cur_.t - (t_ - age_s)) < kCrSameEpochS;
    const double interval = cr_cur_.t - cr_prev_.t;
    double r[3], v[3], Tm[3][3];
    if (ends)
        receiverState(r, v, Tm);

    // every rate's innovation at the prior, before the common rate, and its
    // line of sight (NED)
    CrFit& fit = cr_fit_;
    fit.n = 0;
    int k = 0;
    for (int i = 0; i < n && k < RawEpoch::kMaxObs; ++i)
    {
        const RawMeas& mi = m[i];
        if (!mi.has_cr || mi.sys > GNSS_SYS_BDS || mi.prn >= 64)
            continue;
        st.cr_built++;
        CarrierInfo& c = cr_info_[k++];
        c.idx = static_cast<uint8_t>(i);
        c.sys = mi.sys;
        c.prn = mi.prn;
        c.half = 0;
        c.sigma = static_cast<float>(std::hypot(static_cast<double>(mi.sigma_cr), s_roll));
        c.y = 0.0;
        if (!ends || !(mi.cr_dt > 0.0f) || std::fabs(interval - static_cast<double>(mi.cr_dt)) > kCrIntervalTolS)
        {
            c.verdict = CarrierInfo::kUnmatched;
            st.cr_unmatched++;
            continue;
        }
        T u[3];
        c.y = carrierRate(mi, Tm, u);
        c.verdict = CarrierInfo::kIn;
        const int j = fit.n++;
        fit.info[j] = k - 1;
        fit.y[j] = c.y;
        fit.in[j] = true;
        for (int a = 0; a < 3; ++a)
            fit.u[j][a] = static_cast<double>(u[a]);
    }
    cr_ninfo_ = k;
    if (fit.n == 0)
        return 0;
    if (fit.n < 4)
    {
        // nothing to screen them against: none taken
        st.cr_too_few = true;
        for (int j = 0; j < k; ++j)
            if (cr_info_[j].verdict == CarrierInfo::kIn)
                cr_info_[j].verdict = CarrierInfo::kTooFew;
        return 0;
    }

    // The screen. Each rate's innovation is the common rate c, plus the
    // prior's velocity error along its line of sight, plus its noise: the
    // epoch's median takes c out, but after a Doppler's velocity (0.05-0.2
    // m/s) the velocity's part still puts good rates many sigmas apart. So
    // c and a velocity correction e are fitted to them -- e weighted by the
    // filter's own velocity covariance, c about the median with the common
    // rate's prior. First the rates the fit cannot hold are set aside, the
    // worst first, each judged on its residual left out of the fit (r / (1 -
    // leverage) against max(k sigma, floor), sigma / sqrt(1 - leverage)), and
    // the rest refitted, until all it holds are within. Then each set aside
    // is judged against that clean fit, on its own (several steps in one
    // epoch do not hide each other): within it after all, it is back in;
    // within 3 sigma of a half cycle, lambda/(2 dt), either way where the
    // system's can come unflagged, it is a half-cycle step, repaired or
    // dropped as the profile says; else dropped.
    for (int j = 0; j < fit.n; ++j)
        fit.d[j] = fit.y[j];
    const double med = ekf_detail::medianOf(fit.d, fit.n);
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
            fit.pvv[a][b] = static_cast<double>(P_[kVel + a][kVel + b]);
    fit.c0 = med;
    fit.sc2 = p_.cr_common_sigma_mps * p_.cr_common_sigma_mps;
    bool* in = fit.in;
    const double* d = fit.d;
    const double* sd = fit.sd;
    int nin = fit.n;
    double z[4] = {0.0, 0.0, 0.0, med};
    bool ok = true;                       // the fit solved (a velocity covariance to weigh e by)
    while (nin >= 4)
    {
        ok = crFitSolve(z);
        if (!ok)
            break;
        int worst = -1;
        double worst_ratio = 1.0;
        for (int j = 0; j < fit.n; ++j)
        {
            if (!in[j])
                continue;
            const double ratio = std::fabs(d[j]) / std::fmax(p_.cr_screen_k * sd[j], p_.cr_screen_floor_mps);
            if (ratio > worst_ratio)
            {
                worst_ratio = ratio;
                worst = j;
            }
        }
        if (worst < 0)
            break;
        in[worst] = false;
        nin--;
    }
    if (ok && nin >= 4)
    {
        bool again = false;
        for (int j = 0; j < fit.n; ++j)
        {
            if (in[j])
                continue;
            CarrierInfo& c = cr_info_[fit.info[j]];
            const RawMeas& mi = m[c.idx];
            if (std::fabs(d[j]) <= std::fmax(p_.cr_screen_k * sd[j], p_.cr_screen_floor_mps))
            {
                in[j] = again = true;   // set aside beside another: it fits
                nin++;
                continue;
            }
            const double hstep = 0.5 * kC / p_.cr_carrier_hz[mi.sys] / static_cast<double>(mi.cr_dt);
            int half = 0;
            if (p_.cr_half_cycle_unflagged[mi.sys])
            {
                if (std::fabs(d[j] - hstep) < 3.0 * sd[j])
                    half = 1;
                else if (std::fabs(d[j] + hstep) < 3.0 * sd[j])
                    half = -1;
            }
            c.half = static_cast<int8_t>(half);
            if (half != 0 && p_.cr_half_cycle == kHalfCycleRepair)
            {
                c.verdict = CarrierInfo::kRepaired;
                c.y -= half * hstep;
                fit.y[j] = c.y;
                in[j] = again = true;
                nin++;
                continue;
            }
            c.verdict = half != 0 ? CarrierInfo::kHalfCycle : CarrierInfo::kOther;
        }
        if (again)
            ok = crFitSolve(z);
    }
    else if (ok)
    {
        for (int j = 0; j < fit.n; ++j)
            if (!in[j])
                cr_info_[fit.info[j]].verdict = CarrierInfo::kOther;
    }
    if (!ok)
    {
        // no fit (the velocity covariance would not invert): nothing screened, none taken
        for (int j = 0; j < fit.n; ++j)
        {
            in[j] = true;
            cr_info_[fit.info[j]].verdict = CarrierInfo::kIn;
        }
        nin = 0;
    }
    // what is left with under four stands unscreened: none taken
    if (nin < 4)
        st.cr_too_few = true;
    int n_in = 0;
    double ss = 0.0;
    for (int j = 0; j < fit.n; ++j)
    {
        CarrierInfo& c = cr_info_[fit.info[j]];
        if (!in[j])
        {
            if (c.verdict == CarrierInfo::kHalfCycle)
                st.cr_half++;
            else
                st.cr_other++;
            continue;
        }
        if (nin < 4)
        {
            c.verdict = CarrierInfo::kTooFew;
            continue;
        }
        if (c.verdict == CarrierInfo::kRepaired)
            st.cr_repaired++;
        st.cr_in++;
        n_in++;
        ss += (c.y - z[3]) * (c.y - z[3]);
    }
    if (n_in > 0)
    {
        // the common rate and the velocity the rates point to (the fit's), and
        // the screened innovations about that rate, and about the whole fit
        double sr = 0.0;
        for (int j = 0; j < fit.n; ++j)
        {
            if (!in[j])
                continue;
            const double res = fit.y[j] - (fit.u[j][0] * z[0] + fit.u[j][1] * z[1] + fit.u[j][2] * z[2] + z[3]);
            sr += res * res;
        }
        st.cr_common_mps = z[3];
        st.cr_rms_mps = static_cast<float>(std::sqrt(ss / n_in));
        st.cr_fit_rms_mps = static_cast<float>(std::sqrt(sr / n_in));
        st.cr_dv_mps = static_cast<float>(std::sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]));
    }
    if (p_.cr_mode != kCarrierFuse || n_in == 0)
        return 0;
    // fuse: the common rate's prior about the median
    cr_c_ = med;
    cr_nfuse_ = n_in;
    return cr_nfuse_;
}

// The screen's fit (beginCarrier): the screened rates' common rate and
// velocity correction z = (e_n, e_e, e_d, c), each rate y_j = u_j . e + c +
// noise, e's prior the filter's velocity covariance, c's the common rate's
// about the median. Each rate's residual left out of the fit and its sigma.
// False if the normal matrix will not factor.
template <typename T>
bool RawGnssEKF<T>::crFitSolve(double z[4])
{
    CrFit& f = cr_fit_;
    const bool* in = f.in;
    double* d = f.d;
    double* sd = f.sd;
    // the priors' information: the velocity covariance's inverse (adjugate)
    double A[4][4] = {}, b[4] = {};
    {
        const double(&P)[3][3] = f.pvv;
        const double c00 = P[1][1] * P[2][2] - P[1][2] * P[2][1], c01 = P[1][2] * P[2][0] - P[1][0] * P[2][2],
                     c02 = P[1][0] * P[2][1] - P[1][1] * P[2][0];
        const double det = P[0][0] * c00 + P[0][1] * c01 + P[0][2] * c02;
        if (!(det > 0.0))
            return false;
        const double inv[3][3] = {{c00, P[0][2] * P[2][1] - P[0][1] * P[2][2], P[0][1] * P[1][2] - P[0][2] * P[1][1]},
                                  {c01, P[0][0] * P[2][2] - P[0][2] * P[2][0], P[0][2] * P[1][0] - P[0][0] * P[1][2]},
                                  {c02, P[0][1] * P[2][0] - P[0][0] * P[2][1], P[0][0] * P[1][1] - P[0][1] * P[1][0]}};
        for (int a = 0; a < 3; ++a)
            for (int c = 0; c < 3; ++c)
                A[a][c] = 0.5 * (inv[a][c] + inv[c][a]) / det;
        A[3][3] = 1.0 / f.sc2;
        b[3] = f.c0 / f.sc2;
    }
    for (int j = 0; j < f.n; ++j)
    {
        if (!in[j])
            continue;
        const double w = 1.0 / (static_cast<double>(cr_info_[f.info[j]].sigma) * cr_info_[f.info[j]].sigma);
        const double a[4] = {f.u[j][0], f.u[j][1], f.u[j][2], 1.0};
        for (int r = 0; r < 4; ++r)
        {
            b[r] += w * a[r] * f.y[j];
            for (int c = 0; c < 4; ++c)
                A[r][c] += w * a[r] * a[c];
        }
    }
    // Cholesky, A = L L'
    double L[4][4] = {};
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c <= r; ++c)
        {
            double sum = A[r][c];
            for (int k = 0; k < c; ++k)
                sum -= L[r][k] * L[c][k];
            if (r == c)
            {
                if (!(sum > 0.0))
                    return false;
                L[r][r] = std::sqrt(sum);
            }
            else
                L[r][c] = sum / L[c][c];
        }
    // z = A^-1 b, and A^-1 a_j for each rate's leverage
    auto solve = [&](const double rhs[4], double x[4]) {
        double t[4];
        for (int r = 0; r < 4; ++r)
        {
            double sum = rhs[r];
            for (int k = 0; k < r; ++k)
                sum -= L[r][k] * t[k];
            t[r] = sum / L[r][r];
        }
        for (int r = 3; r >= 0; --r)
        {
            double sum = t[r];
            for (int k = r + 1; k < 4; ++k)
                sum -= L[k][r] * x[k];
            x[r] = sum / L[r][r];
        }
    };
    solve(b, z);
    for (int j = 0; j < f.n; ++j)
    {
        const double a[4] = {f.u[j][0], f.u[j][1], f.u[j][2], 1.0};
        const double sig = static_cast<double>(cr_info_[f.info[j]].sigma);
        const double res = f.y[j] - (a[0] * z[0] + a[1] * z[1] + a[2] * z[2] + z[3]);
        double x[4];
        solve(a, x);
        const double q = a[0] * x[0] + a[1] * x[1] + a[2] * x[2] + a[3] * x[3];   // a' A^-1 a
        if (in[j])
        {
            // in the fit: its residual with itself taken out
            const double lev = q / (sig * sig);
            const double one = lev < 1.0 - 1e-9 ? 1.0 - lev : 1e-9;
            d[j] = res / one;
            sd[j] = sig / std::sqrt(one);
        }
        else
        {
            d[j] = res;
            sd[j] = std::sqrt(sig * sig + q);
        }
    }
    return true;
}

template <typename T>
void RawGnssEKF<T>::fuseCarrier(const RawMeas* m, RawGnssStats& st)
{
    if (p_.cr_mode != kCarrierFuse || cr_nfuse_ == 0)
        return;
    cr_pcc_ = T(p_.cr_common_sigma_mps * p_.cr_common_sigma_mps);
    for (int i = 0; i < N; ++i)
        cr_pxc_[i] = T(0);
    // the attitude gate the GNSS updates have, and the present frame (the
    // corrections here are centimetres: its turn across them is nothing)
    const T att = attGate();
    T vert[3];
    const T* axis = gnssAttAxis(vert);
    double r[3], v[3], Tm[3][3];
    receiverState(r, v, Tm);
    for (int j = 0; j < cr_ninfo_; ++j)
    {
        const CarrierInfo& c = cr_info_[j];
        if (c.verdict != CarrierInfo::kIn && c.verdict != CarrierInfo::kRepaired)
            continue;
        const RawMeas& mi = m[c.idx];
        T u[3];
        double y0 = carrierRate(mi, Tm, u);
        if (c.half != 0)
            y0 -= c.half * 0.5 * kC / p_.cr_carrier_hz[mi.sys] / static_cast<double>(mi.cr_dt);
        const int hidx[3] = {kVel, kVel + 1, kVel + 2};
        const T hval[3] = {-u[0], -u[1], -u[2]};
        const T R = T(static_cast<double>(c.sigma) * static_cast<double>(c.sigma));
        T dx[N], dc = T(0), nis = T(0);
        if (carrierUpdate(hidx, hval, 3, T(y0 - cr_c_), R, att, axis, dx, dc, &nis) != 1)
            continue;
        inject(dx);
        stabilize();
        // the common rate's cross-covariance kept within what the variances
        // allow (stabilize may have restarted or scaled a row of P)
        if (!(cr_pcc_ > T(0)) || !std::isfinite(cr_pcc_))
        {
            cr_pcc_ = T(1e-12);
            for (int i = 0; i < N; ++i)
                cr_pxc_[i] = T(0);
        }
        for (int i = 0; i < N; ++i)
        {
            const T lim = std::sqrt(P_[i][i] * cr_pcc_);
            if (!std::isfinite(cr_pxc_[i]))
                cr_pxc_[i] = T(0);
            else if (cr_pxc_[i] > lim)
                cr_pxc_[i] = lim;
            else if (cr_pxc_[i] < -lim)
                cr_pxc_[i] = -lim;
        }
        cr_c_ += static_cast<double>(dc);
        st.used_cr++;
        if (static_cast<float>(nis) > st.nis_max)
            st.nis_max = static_cast<float>(nis);
    }
    st.cr_common_mps = cr_c_;
}

template <typename T>
void RawGnssEKF<T>::endCarrier(RawGnssStats& st)
{
    (void)st;
    // the common rate is dropped with the epoch; its receiver is the next
    // epoch's earlier end (none, if it had none)
    cr_nfuse_ = 0;
    cr_pcc_ = T(0);
    for (int i = 0; i < N; ++i)
        cr_pxc_[i] = T(0);
    if (cr_cur_.valid)
        cr_prev_ = cr_cur_;
    else
        cr_prev_.valid = false;
    cr_cur_.valid = false;
}


// The boost screen (screenRangeRates, RawGnssShadow's): the epoch's Dopplers
// at the state as it stands, each as updateGnssRawPart would predict and gate
// it were it the epoch's first update: the same
// prediction (the receiver back to the Doppler's instant, the satellite's
// range acceleration out), H, R and chi-square, the same let-in for a
// satellite refused reject_persist epochs running. Nothing is fused, and the
// state, its covariance and the satellites' counts stay as they are. The
// geometry once: no update moves the receiver between satellites here.
// And, for the shadow's vertical check, one velocity (NED) and clock rate
// fitted to the Dopplers' innovations by least squares, each weighted by its
// own 1-sigma as the update weights it (a relocking satellite's 3 m/s counts
// for little beside a tracking one's 0.1: unweighted, the noisy rows pulled
// the fit and the loop below left out the precise ones -- the cloud session's
// sim, 2026-10-16): its down part is the epoch's vertical velocity against the
// filter's, which the IMU and barometer hold whatever the heading (a receiver
// that has lost its loops in a boost hands over the Dopplers of one still on
// the pad: 82 m/s down at T+1 on the rig's c30, its Dopplers within 0.8 m/s of
// each other). A satellite on its own far off the fit -- more than
// kVertOutlier of its own 1-sigmas -- is left out and the rest fitted again,
// while five stay (verticalFit): one bad satellite among good ones (GTV's
// E13, 50-63 m/s off for four epochs) is the per-row gate's to refuse, not the
// epoch's to lose, and one that stays bad would otherwise drop every epoch. A
// whole receiver gone stale has no such satellite: its Dopplers agree with
// one another.
template <typename T>
void RawGnssEKF<T>::screenRangeRates(const RawMeas* m, int n, double age_s, RawGnssStats& st) const
{
    st.screen_rr = st.screen_refused = 0;
    st.screen_fit_n = 0;
    st.screen_dv_down_mps = 0.0f;
    st.screen_fit_out = 0;
    // the rows in the filter's own scratch, not on the task's stack (1.8 KB free on the rig)
    double (&Hf)[RawEpoch::kMaxObs][4] = vfit_h_;
    double* yf = vfit_y_;
    double* sf = vfit_s_;
    int nfit = 0;
    if (age_s < 0.0)
        age_s = 0.0;
    const double tau = rrLag();
    T dp_e[3], dv_e[3], dp_l[3], dv_l[3];
    historyChange(age_s, dv_e, dp_e);
    historyChange(age_s + tau, dv_l, dp_l);
    // lever_on: the antenna's, as the update predicts them
    LeverEpoch lv;
    if (p_.lever_on)
        leverBack(age_s, tau, dp_e, dv_e, dv_l, lv);
    double r[3], v[3], Tm[3][3], r_e[3], v_e[3];
    const bool share = p_.share_los;
    if (share)
        frameOf(r, v, Tm);
    else
        receiverState(r, v, Tm);
    backDate(r, v, Tm, dp_e, dv_e, r_e, v_e);
    double vr[3], v_l[3];
    for (int a = 0; a < 3; ++a)
        vr[a] = Tm[0][a] * static_cast<double>(v_[0]) + Tm[1][a] * static_cast<double>(v_[1])
              + Tm[2][a] * static_cast<double>(v_[2]);
    const T* dv = tau > 0.0 ? dv_l : dv_e;
    for (int a = 0; a < 3; ++a)
        v_l[a] = vr[a] - (Tm[0][a] * static_cast<double>(dv[0]) + Tm[1][a] * static_cast<double>(dv[1])
                          + Tm[2][a] * static_cast<double>(dv[2]));
    // rr_float: the update's prediction, the receiver's part in NED (TR_RawGnssEKF.h)
    const bool rrf = share && p_.rr_float && tau <= 0.0;
    T vl_ned[3];
    for (int a = 0; a < 3; ++a)
        vl_ned[a] = v_[a] - dv[a];
    const double te = t_ - age_s;
    for (int i = 0; i < n; ++i)
    {
        const RawMeas& mi = m[i];
        if (mi.sys > GNSS_SYS_BDS || mi.prn >= 64 || !mi.has_rr)
            continue;
        Los L;
        if (share)
        {
            L.rho = losOf(m, n, i, te, r_e, L.u, L.vs);   // the epoch's (TR_RawGnssEKF.h share_los)
            L.rate = 0.0;
        }
        else
            lineOfSight(mi, r_e, v_e, L);
        T u_ned[3];
        for (int a = 0; a < 3; ++a)
            u_ned[a] = T(Tm[a][0] * L.u[0] + Tm[a][1] * L.u[1] + Tm[a][2] * L.u[2]);
        double pred_rate;
        if (rrf)
            pred_rate = rrFloat(L.u[0] * L.vs[0] + L.u[1] * L.vs[1] + L.u[2] * L.vs[2], u_ned, vl_ned);
        else
        {
            Los Lr = L;
            Lr.rate = 0.0;
            for (int a = 0; a < 3; ++a)
                Lr.rate += L.u[a] * (L.vs[a] - v_l[a]);
            pred_rate = tau > 0.0
                          ? Lr.rate - tau * (rr_paired_ ? pairAccel(i, mi, Lr, v_l) : satRangeAccel(mi, Lr, v_l))
                          : Lr.rate;
        }
        int hidx[10];
        T hval[10];
        int hn = 0;
        for (int a = 0; a < 3; ++a)
        {
            hidx[hn] = kVel + a;
            hval[hn++] = -u_ned[a];
        }
        hidx[hn] = kClkDrift;
        hval[hn++] = T(1);
        const T y = T(mi.rr - (pred_rate + static_cast<double>(clk_d_)));
        // lever_on: the update's lever terms, fused or considered (R below takes them)
        T r_lev = T(0);
        if (p_.lever_on)
            hn = leverRows(u_ned, lv, hidx, hval, hn, r_lev);
        if (nfit < RawEpoch::kMaxObs)
        {
            for (int a = 0; a < 3; ++a)
                Hf[nfit][a] = -static_cast<double>(u_ned[a]);
            Hf[nfit][3] = 1.0;
            yf[nfit] = mi.rr - (pred_rate + static_cast<double>(clk_d_));
            sf[nfit] = mi.sigma_rr > 0.0f ? static_cast<double>(mi.sigma_rr) : 1.0;
            nfit++;
        }
        const double s_r = static_cast<double>(mi.sigma_rr);
        const double dt_i = t_ - last_upd_[mi.sys][mi.prn][1];
        const T R = T(s_r * s_r * corrInflation(dt_i, static_cast<double>(mi.tau_rr))) + r_lev;
        st.screen_rr++;
        if (rejects_[mi.sys][mi.prn][1] >= rejectPersist(1))
            continue;                     // the update lets it in, whatever its innovation
        // S and the test as scalarUpdate forms them
        T HP[N] = {};
        for (int k = 0; k < hn; ++k)
        {
            const T h = hval[k];
            const T* row = P_[hidx[k]];
            for (int j = 0; j < N; ++j)
                HP[j] += h * row[j];
        }
        T S = R;
        for (int k = 0; k < hn; ++k)
            S += HP[hidx[k]] * hval[k];
        const T gate = T(p_.gate_chi2);
        if (!(S > T(0)) || (gate > T(0) && y * y * (T(1) / S) > gate))
            st.screen_refused++;
    }
    // the fit: four unknowns, so four Dopplers at least
    int nin = 0, nout = 0;
    double x[4];
    if (!verticalFit(Hf, yf, sf, vfit_in_, nfit, x, nin, nout))
        return;                           // under four, or the geometry gives no velocity: unjudged
    st.screen_fit_n = nin;
    st.screen_fit_out = nout;
    st.screen_dv_down_mps = static_cast<float>(x[2]);
}

// The vertical check's fit (screenRangeRates): rows H (velocity NED, clock
// rate), innovations y, their 1-sigmas s. Fitted weighted by 1/s^2; while
// more than five stay in, the one most of its own 1-sigmas off the fit, past
// kVertOutlier, is left out and the rest fitted again. x the last fit; in
// marks the rows kept (nin) and left out (nout). False under four rows or if
// the geometry gives no velocity.
template <typename T>
bool RawGnssEKF<T>::verticalFit(const double H[][4], const double* y, const double* s, bool* in, int n, double x[4],
                                int& nin, int& nout)
{
    nin = n;
    nout = 0;
    if (n < 4)
        return false;
    for (int i = 0; i < n; ++i)
        in[i] = true;
    for (;;)
    {
        if (!fitVelocityClock(H, y, s, in, n, x))
            return false;
        if (nin <= 5)
            return true;
        int worst = -1;
        double worst_z = kVertOutlier;
        for (int i = 0; i < n; ++i)
        {
            if (!in[i])
                continue;
            double r = y[i];
            for (int j = 0; j < 4; ++j)
                r -= H[i][j] * x[j];
            const double z = std::fabs(r) / s[i];
            if (z > worst_z)
            {
                worst_z = z;
                worst = i;
            }
        }
        if (worst < 0)
            return true;
        in[worst] = false;
        nin--;
        nout++;
    }
}

// One velocity and clock rate by least squares, each row weighted by 1/s^2, on
// the rows in (Gauss-Jordan, partial pivoting); false if the geometry gives no
// velocity.
template <typename T>
bool RawGnssEKF<T>::fitVelocityClock(const double H[][4], const double* y, const double* s, const bool* in, int n,
                                     double x[4])
{
    double M[4][5] = {};
    for (int i = 0; i < n; ++i)
    {
        if (!in[i])
            continue;
        const double w = 1.0 / (s[i] * s[i]);
        for (int j = 0; j < 4; ++j)
        {
            M[j][4] += w * H[i][j] * y[i];
            for (int k = 0; k < 4; ++k)
                M[j][k] += w * H[i][j] * H[i][k];
        }
    }
    double scale = 0.0;                   // the normal matrix's size, for a pivot test that ignores the weights' units
    for (int j = 0; j < 4; ++j)
        scale = std::max(scale, std::fabs(M[j][j]));
    for (int c = 0; c < 4; ++c)
    {
        int piv = c;
        for (int r2 = c + 1; r2 < 4; ++r2)
            if (std::fabs(M[r2][c]) > std::fabs(M[piv][c]))
                piv = r2;
        if (!(std::fabs(M[piv][c]) > 1e-12 * scale))
            return false;
        for (int k = 0; k < 5; ++k)
            std::swap(M[c][k], M[piv][k]);
        for (int r2 = 0; r2 < 4; ++r2)
        {
            if (r2 == c)
                continue;
            const double f = M[r2][c] / M[c][c];
            for (int k = 0; k < 5; ++k)
                M[r2][k] -= f * M[c][k];
        }
    }
    for (int j = 0; j < 4; ++j)
        x[j] = M[j][4] / M[j][j];
    return true;
}

// An epoch in pairs (setRrPaired): the satellite's own range acceleration,
// which its Doppler's every consumer backs out over the epoch's lag -- the
// change check (twice), the boost screen, the update and the screen's scratch
// copy of it -- taken by the first and reused by the rest: four or five a
// satellite an epoch become one (16 us each on the V9's soft doubles: 1.0-1.3
// ms an epoch at 16 satellites, half of it in the change check's unbroken
// unit, become 0.26 ms). The rest would have taken it at the receiver's
// velocity as they see it, which differs from the first's by the filter's
// corrections, the lever's velocity and the IMU's motion between the units:
// metres per second at most, which move the term by ~4e-4 m/s^2 each (twice
// the satellite's speed over its range). Over the pair's 25 ms that is at most
// 3e-6 m/s on Grid B's boosts and coasts (exp/v9-dop-avg's replays), a
// ten-thousandth of the Doppler's noise. An epoch not in pairs takes it each
// time, as before.
template <typename T>
double RawGnssEKF<T>::pairAccel(int i, const RawMeas& m, const Los& L, const double v[3]) const
{
    const bool slot = i >= 0 && i < RawEpoch::kMaxObs;
    const uint64_t bit = slot ? uint64_t{1} << i : 0;
    if (slot && (acc_memo_.have & bit) != 0 && acc_memo_.sys[i] == m.sys && acc_memo_.prn[i] == m.prn)
        return acc_memo_.a[i];
    const double a = satRangeAccel(m, L, v);
    if (slot)
    {
        acc_memo_.have |= bit;
        acc_memo_.sys[i] = m.sys;
        acc_memo_.prn[i] = m.prn;
        acc_memo_.a[i] = a;
    }
    return a;
}

// The Doppler change check's rows (TR_RawGnssEKF.h, DopplerChange.h). Each
// end predicted as updateGnssRawPart predicts a Doppler: the receiver taken
// back to the epoch by the history, the satellite's motion over the signal's
// travel (lineOfSight), a late Doppler's own range acceleration backed out --
// at ref's velocity carried by the IMU at this end, at ref's own at that end,
// so the filter's velocity, and every correction made between the two, drop
// out. Each end takes its own line of sight: what is left of the base
// velocity's error is that error times the line of sight's turn between the
// ends (1e-5-1e-4 rad over 0.1-1 s), a few m/s of it well under the noise.
template <typename T>
int RawGnssEKF<T>::dopplerChange(const RawMeas* m, int n, double age_s, const DopplerRef& ref, DopplerRef& out,
                                 dvc::Row* rows, DopplerChangeImu& imu) const
{
    imu = DopplerChangeImu();
    if (age_s < 0.0)
        age_s = 0.0;
    const double tau = rrLag();
    T dp_e[3], dv_e[3], dp_l[3], dv_l[3];
    historyChange(age_s, dv_e, dp_e);
    historyChange(age_s + tau, dv_l, dp_l);
    double r[3], v[3], Tm[3][3], r_e[3], v_e[3];
    const bool share = p_.share_los;
    if (share)
        frameOf(r, v, Tm);
    else
        receiverState(r, v, Tm);
    backDate(r, v, Tm, dp_e, dv_e, r_e, v_e);
    const T* dv = tau > 0.0 ? dv_l : dv_e;
    // the filter's velocity at this end's Doppler (NED), and its time
    double v_now[3];
    for (int a = 0; a < 3; ++a)
        v_now[a] = static_cast<double>(v_[a]) - static_cast<double>(dv[a]);
    const double t_now = t_ - age_s - tau;
    // lever_on: the antenna's velocity against the IMU at this end, in every
    // prediction here (ref's has its own end's in its y), so the change is
    // the antenna's: the IMU's change and the lever's
    double v_lev[3] = {0.0, 0.0, 0.0};
    if (p_.lever_on)
    {
        T pl[3], vl[3];
        leverAt(age_s + tau, pl, vl);
        for (int a = 0; a < 3; ++a)
        {
            v_lev[a] = static_cast<double>(vl[a]);
            imu.v_lev[a] = static_cast<float>(vl[a]);
        }
    }
    // ref's velocity carried to this end by the IMU alone
    double v_imu[3] = {0.0, 0.0, 0.0};
    if (ref.valid && ref.n > 0)
    {
        T dvi[3], dpi[3];
        if (imuMotion(ref.t, t_now, dvi, dpi))
        {
            T g[3];
            gravityNed(g);
            imu.ok = true;
            imu.dt = t_now - ref.t;
            for (int a = 0; a < 3; ++a)
            {
                imu.dv[a] = static_cast<float>(dvi[a]);
                imu.dv_sf[a] = static_cast<float>(static_cast<double>(dvi[a])
                                                  - static_cast<double>(g[a]) * imu.dt);
                v_imu[a] = static_cast<double>(ref.v_ned[a]) + static_cast<double>(dvi[a]);
            }
        }
    }
    out.valid = true;
    out.t = t_now;
    for (int a = 0; a < 3; ++a)
        out.v_ned[a] = static_cast<float>(v_now[a]);
    out.n = 0;
    int nr = 0;
    // rr_float: the update's prediction, the receiver's part in NED (TR_RawGnssEKF.h)
    const bool rrf = share && p_.rr_float && tau <= 0.0;
    const double te = t_ - age_s;
    for (int i = 0; i < n; ++i)
    {
        const RawMeas& mi = m[i];
        if (mi.sys > GNSS_SYS_BDS || mi.prn >= 64 || !mi.has_rr)
            continue;
        Los L;
        if (share)
        {
            L.rho = losOf(m, n, i, te, r_e, L.u, L.vs);   // the epoch's (TR_RawGnssEKF.h share_los)
            L.rate = 0.0;
        }
        else
            lineOfSight(mi, r_e, v_e, L);
        double u_ned[3];
        for (int a = 0; a < 3; ++a)
            u_ned[a] = Tm[a][0] * L.u[0] + Tm[a][1] * L.u[1] + Tm[a][2] * L.u[2];
        // the satellite's part of the predicted rate, and the receiver's at a velocity (NED)
        double us = 0.0;
        for (int a = 0; a < 3; ++a)
            us += L.u[a] * L.vs[a];
        T uf[3] = {};
        if (rrf)
            for (int a = 0; a < 3; ++a)
                uf[a] = T(u_ned[a]);
        auto pred = [&](const double vi[3]) {
            double ve[3], va[3];
            const double* vn = vi;
            if (p_.lever_on)
            {
                for (int a = 0; a < 3; ++a)
                    va[a] = vi[a] + v_lev[a];   // the antenna's
                vn = va;
            }
            if (rrf)
            {
                const T vf[3] = {T(vn[0]), T(vn[1]), T(vn[2])};
                return rrFloat(us, uf, vf);
            }
            for (int a = 0; a < 3; ++a)
                ve[a] = Tm[0][a] * vn[0] + Tm[1][a] * vn[1] + Tm[2][a] * vn[2];
            Los Lr = L;
            Lr.rate = us - (L.u[0] * ve[0] + L.u[1] * ve[1] + L.u[2] * ve[2]);
            return tau > 0.0 ? Lr.rate - tau * (rr_paired_ ? pairAccel(i, mi, Lr, ve) : satRangeAccel(mi, Lr, ve))
                             : Lr.rate;
        };
        const int iout = out.n;
        if (out.n < RawEpoch::kMaxObs)
        {
            out.sys[out.n] = mi.sys;
            out.prn[out.n] = mi.prn;
            out.y[out.n] = mi.rr - pred(v_now);
            out.s[out.n] = mi.sigma_rr;
            for (int a = 0; a < 3; ++a)
                out.u[out.n][a] = static_cast<float>(u_ned[a]);
            out.n++;
        }
        if (!imu.ok)
            continue;
        int j = -1;
        for (int k = 0; k < ref.n; ++k)
            if (ref.sys[k] == mi.sys && ref.prn[k] == mi.prn)
            {
                j = k;
                break;
            }
        if (j < 0 || nr >= RawEpoch::kMaxObs)
            continue;
        dvc::Row& row = rows[nr++];
        for (int a = 0; a < 3; ++a)
            row.u[a] = u_ned[a];
        row.z = (mi.rr - pred(v_imu)) - ref.y[j];
        const double si = static_cast<double>(mi.sigma_rr), sj = static_cast<double>(ref.s[j]);
        row.s = std::sqrt(si * si + sj * sj);
        row.im = i;
        row.iref = j;
        row.iout = iout < RawEpoch::kMaxObs ? iout : -1;
        row.in = true;
        row.e = 0.0;
    }
    return nr;
}

// share_los (TR_RawGnssEKF.h): the epoch's lines of sight, every satellite's
// at once, at the receiver r its first unit back-dates to. Here, in flash with
// the rest of the once-an-epoch code: the Beetle never turns it on.
template <typename T>
void RawGnssEKF<T>::losFill(const RawMeas* m, int n, double te, const double r[3]) const
{
    LosShare& S = los_;
    const double still[3] = {0.0, 0.0, 0.0};
    S.te = te;
    S.n = n < RawEpoch::kMaxObs ? n : RawEpoch::kMaxObs;
    for (int a = 0; a < 3; ++a)
        S.r0[a] = r[a];
    for (int i = 0; i < S.n; ++i)
    {
        const RawMeas& mi = m[i];
        S.sys[i] = mi.sys;
        S.prn[i] = mi.prn;
        S.sx[i] = mi.sat_pos[0];
        S.ok[i] = mi.sys <= GNSS_SYS_BDS;
        if (!S.ok[i])
            continue;
        Los L;
        lineOfSight(mi, r, still, L);
        S.rho[i] = L.rho;
        for (int a = 0; a < 3; ++a)
        {
            S.u[i][a] = L.u[a];
            S.vs[i][a] = L.vs[a];
        }
    }
    S.valid = true;
}

// Satellite i's range, line of sight and turned velocity at r: from the
// epoch's (taken first, by a unit with n), to second order in d = r - r0 --
// rho(r) = rho0 - u0.d, u(r) = u0 - (d - u0 (u0.d)) / rho0 -- and the same
// bits at r0 itself. The satellite's turn by the Earth's rotation over the
// travel moves with the range by 2.4e-13 rad a metre: kept. Where they cannot
// serve (another epoch's, another measurement at i, a unit with no n before
// any was taken), lineOfSight's own.
template <typename T>
double RawGnssEKF<T>::losOf(const RawMeas* m, int n, int i, double te, const double r[3], double u[3],
                            double vs[3]) const
{
    LosShare& S = los_;
    const RawMeas& mi = m[i];
    bool epoch = S.valid && std::fabs(te - S.te) <= kLosSameEpochS;
    bool same = epoch && r[0] == S.r0[0] && r[1] == S.r0[1] && r[2] == S.r0[2];
    double d[3] = {0.0, 0.0, 0.0};
    if (epoch && !same)
    {
        for (int a = 0; a < 3; ++a)
            d[a] = r[a] - S.r0[a];
        if (d[0] * d[0] + d[1] * d[1] + d[2] * d[2] > kLosRetakeM * kLosRetakeM)
        {
            losFill(m, n > 0 ? n : S.n, te, r);   // corrections past kLosRetakeM since: again, here
            same = true;
        }
    }
    else if (!epoch && n > 0)
    {
        losFill(m, n, te, r);                     // the epoch's first unit
        epoch = same = true;
    }
    if (!(epoch && i < S.n && S.ok[i] && S.sys[i] == mi.sys && S.prn[i] == mi.prn && S.sx[i] == mi.sat_pos[0]))
    {
        const double still[3] = {0.0, 0.0, 0.0};
        Los L;
        lineOfSight(mi, r, still, L);
        for (int a = 0; a < 3; ++a)
        {
            u[a] = L.u[a];
            vs[a] = L.vs[a];
        }
        return L.rho;
    }
    const double* u0 = S.u[i];
    for (int a = 0; a < 3; ++a)
        vs[a] = S.vs[i][a];
    if (same)
    {
        for (int a = 0; a < 3; ++a)
            u[a] = u0[a];
        return S.rho[i];
    }
    const double p = u0[0] * d[0] + u0[1] * d[1] + u0[2] * d[2];
    const double k = 1.0 / S.rho[i];
    for (int a = 0; a < 3; ++a)
        u[a] = u0[a] - (d[a] - u0[a] * p) * k;
    return S.rho[i] - p;
}

// receiverState's position and frame, kept while lla_ is the one they were
// taken at bit for bit (then the same bits), or carried there by inject's own
// corrections (injectLla). v, if asked, from the velocity as it stands.
template <typename T>
void RawGnssEKF<T>::frameOf(double r[3], double v[3], double Tm[3][3]) const
{
    LosShare& S = los_;
    if (!(S.frame_ok && lla_[0] == S.f_lla[0] && lla_[1] == S.f_lla[1] && lla_[2] == S.f_lla[2]))
    {
        double vv[3];
        receiverState(S.f_r, vv, S.f_Tm);
        for (int a = 0; a < 3; ++a)
        {
            S.f_lla[a] = lla_[a];
            r[a] = S.f_r[a];
            if (v != nullptr)
                v[a] = vv[a];
            for (int b = 0; b < 3; ++b)
                Tm[a][b] = S.f_Tm[a][b];
        }
        S.carried_m = 0.0;
        S.frame_ok = true;
        return;
    }
    for (int a = 0; a < 3; ++a)
    {
        r[a] = S.f_r[a];
        for (int b = 0; b < 3; ++b)
            Tm[a][b] = S.f_Tm[a][b];
    }
    // receiverState's own expression, so a frame kept gives its very bits
    if (v != nullptr)
        for (int i = 0; i < 3; ++i)
            v[i] = Tm[0][i] * static_cast<double>(v_[0]) + Tm[1][i] * static_cast<double>(v_[1])
                 + Tm[2][i] * static_cast<double>(v_[2]);
}

// inject's position part with share_los: the earth radii (and the latitude's
// sine and cosine) kept while the latitude moves under kRadiiKeepRad -- the
// radii then within 3e-9 of their own, the cosine taken to first order from
// them -- and the frame (frameOf) carried along: the position's ECEF change is
// the NED correction turned by the frame, to 1.6e-7 m for a metre's, its axes
// left as they were (1.6e-7 rad a metre), until kFrameCarryM in all.
template <typename T>
void RawGnssEKF<T>::injectLla(const T dx[N])
{
    LosShare& S = los_;
    const bool carry = S.frame_ok && lla_[0] == S.f_lla[0] && lla_[1] == S.f_lla[1] && lla_[2] == S.f_lla[2];
    if (!(S.rad_ok && std::fabs(lla_[0] - S.rad_lat) <= kRadiiKeepRad))
    {
        // TR_RawGnssEKF.cpp's earthRad, its expressions
        constexpr double kRe = 6378137.0, kE2 = 0.0066943799901;
        const double s = std::sin(lla_[0]);
        const double dd = std::fabs(1.0 - kE2 * s * s);
        S.rad_rew = kRe / std::sqrt(dd);
        S.rad_rns = kRe * (1.0 - kE2) / (dd * std::sqrt(dd));
        S.rad_s = s;
        S.rad_c = std::cos(lla_[0]);
        S.rad_lat = lla_[0];
        S.rad_ok = true;
    }
    lla_[2] -= static_cast<double>(dx[2]);
    lla_[0] += static_cast<double>(dx[0]) / (S.rad_rns + lla_[2]);
    const double c = S.rad_c - S.rad_s * (lla_[0] - S.rad_lat);
    lla_[1] += static_cast<double>(dx[1]) / ((S.rad_rew + lla_[2]) * c);
    if (!carry)
        return;
    const double d[3] = {static_cast<double>(dx[0]), static_cast<double>(dx[1]), static_cast<double>(dx[2])};
    S.carried_m += std::fabs(d[0]) + std::fabs(d[1]) + std::fabs(d[2]);
    if (S.carried_m > kFrameCarryM)
    {
        S.frame_ok = false;
        return;
    }
    for (int i = 0; i < 3; ++i)
    {
        S.f_r[i] += S.f_Tm[0][i] * d[0] + S.f_Tm[1][i] * d[1] + S.f_Tm[2][i] * d[2];
        S.f_lla[i] = lla_[i];
    }
}

template <typename T>
const double* RawGnssEKF<T>::sharedRanges(const RawMeas* m, int n, double age_s, const double r[3]) const
{
    if (!p_.share_los)
        return nullptr;
    const double te = t_ - (age_s > 0.0 ? age_s : 0.0);
    LosShare& S = los_;
    double u[3], vs[3];
    const int nn = n < RawEpoch::kMaxObs ? n : RawEpoch::kMaxObs;
    for (int i = 0; i < nn; ++i)
        S.rng[i] = m[i].has_pr && m[i].sys <= GNSS_SYS_BDS ? losOf(m, n, i, te, r, u, vs) : 0.0;
    return S.rng;
}

// The Doppler's prediction with rr_float (TR_RawGnssEKF.h): the satellite's
// part us = u.vs (double), less the receiver's velocity v (NED) along the line
// of sight u_ned in float.
template <typename T>
double RawGnssEKF<T>::rrFloat(double us, const T u_ned[3], const T v_ned[3])
{
    const float uv = static_cast<float>(u_ned[0]) * static_cast<float>(v_ned[0])
                   + static_cast<float>(u_ned[1]) * static_cast<float>(v_ned[1])
                   + static_cast<float>(u_ned[2]) * static_cast<float>(v_ned[2]);
    return us - static_cast<double>(uv);
}

// LosShare's initializers (TR_RawGnssEKF.h), out of the filter's constructor.
template <typename T>
RawGnssEKF<T>::LosShare::LosShare() = default;

// The share_los members' own instantiations (the macro below has the rest of
// this file's), and beginGnssRaw's and updateGnssRawPart's share_los form
// (RawGnssEKFUpdate.h; TR_RawGnssEKF.cpp has the other).
#define TR_RAW_GNSS_LOS_INSTANTIATE(T)                                                                            \
    template void RawGnssEKF<T>::losFill(const RawMeas*, int, double, const double[3]) const;                    \
    template double RawGnssEKF<T>::losOf(const RawMeas*, int, int, double, const double[3], double[3], double[3])  \
        const;                                                                                                    \
    template void RawGnssEKF<T>::frameOf(double[3], double[3], double[3][3]) const;                              \
    template void RawGnssEKF<T>::injectLla(const T[RawGnssEKF<T>::N]);                                           \
    template const double* RawGnssEKF<T>::sharedRanges(const RawMeas*, int, double, const double[3]) const;      \
    template double RawGnssEKF<T>::rrFloat(double, const T[3], const T[3]);                                      \
    template RawGnssEKF<T>::LosShare::LosShare();                                                                 \
    template bool RawGnssEKF<T>::beginGnssRawAs<true>(const RawMeas*, int, double, RawGnssStats&);               \
    template void RawGnssEKF<T>::updateGnssRawPartAs<true>(const RawMeas*, int, int, double, RawGnssStats&);
TR_RAW_GNSS_LOS_INSTANTIATE(float)
TR_RAW_GNSS_LOS_INSTANTIATE(double)
#undef TR_RAW_GNSS_LOS_INSTANTIATE

// hdg_obs_mode's s (TR_RawGnssEKF.h): how much of a GNSS row's heading
// correction an epoch age_s old may make. Open (1) with the mode off or not
// applied, and in the boost: the IMU past hdg_obs_open_g at any time from
// hdg_obs_open_hold_s before the epoch's own time until now. (The window
// starts again at any later step past it -- a drogue's snatch can be one --
// so where the window applies in the descent, the scope says.) Once an epoch
// (beginGnssRaw), and only with the mode on: here, so on the Beetle, which
// never turns it on, it stays in flash (raw_gnss_iram.lf).
template <typename T>
T RawGnssEKF<T>::hdgObsScaleNow(double age_s) const
{
    if (p_.hdg_obs_mode == kHdgObsOff || !hdg_obs_on_)
        return T(1);
    const double since_boost = t_ - (age_s > 0.0 ? age_s : 0.0) - hdg_open_t_;
    if (since_boost <= p_.hdg_obs_open_hold_s)
        return T(1);
    double s = 1.0;
    const double fh = std::sqrt(static_cast<double>(fh_ew_[0]) * static_cast<double>(fh_ew_[0])
                                + static_cast<double>(fh_ew_[1]) * static_cast<double>(fh_ew_[1]));
    switch (p_.hdg_obs_mode)
    {
    case kHdgObsForce:
        s = p_.hdg_obs_fh_mps2 > 0.0 ? fh / p_.hdg_obs_fh_mps2 : 1.0;
        break;
    case kHdgObsInfo:
    {
        // the velocity a 1-sigma heading error makes over tau, in the
        // Doppler velocity's own noise: a heading-only update's gain on it
        T sig[3];
        attSigmaNed(sig);
        const double x = p_.hdg_obs_sigma_v > 0.0
                             ? fh * p_.hdg_obs_tau_s * static_cast<double>(sig[2]) / p_.hdg_obs_sigma_v
                             : 1.0e9;
        s = x * x / (1.0 + x * x);
        break;
    }
    case kHdgObsWindow:
        // closed for the window after a boost; open before any (the pad) and after
        s = hdg_open_t_ > kNever && since_boost <= p_.hdg_obs_open_hold_s + p_.hdg_obs_window_s ? 0.0 : 1.0;
        break;
    default:
        break;
    }
    if (s < p_.hdg_obs_floor)
        s = p_.hdg_obs_floor;
    return T(s < 1.0 ? s : 1.0);
}

// The antenna's offset (TR_RawGnssEKF.h lever_on): set where the owner's
// setting changes, on the ground. Here, in flash, with the rest of the lever's
// code: the Beetle never turns it on, and its per-step code in internal RAM
// keeps only the calls.
template <typename T>
void RawGnssEKF<T>::setLever(bool on, const double b_m[3], uint8_t att_mode)
{
    p_.lever_on = on;
    for (int i = 0; i < 3; ++i)
        p_.lever_b_m[i] = on ? b_m[i] : 0.0;
    p_.lever_att_mode = on ? att_mode : static_cast<uint8_t>(kLeverAttOff);
}

// Each step's antenna, into the history entry the step went into (the newest:
// pushHistory either opened it or added the step to it). The body's rate is
// the step's (the gyro less its bias, its mean over the step), the attitude
// the step's end.
template <typename T>
void RawGnssEKF<T>::pushLever(const T w_b[3])
{
    T C[3][3];
    bodyToNed(q_, C);
    const T r[3] = {T(p_.lever_b_m[0]), T(p_.lever_b_m[1]), T(p_.lever_b_m[2])};
    const T wr[3] = {w_b[1] * r[2] - w_b[2] * r[1], w_b[2] * r[0] - w_b[0] * r[2], w_b[0] * r[1] - w_b[1] * r[0]};
    T* e = lev_[(hhead_ + kHist - 1) % kHist];
    for (int i = 0; i < 3; ++i)
    {
        e[i] = C[i][0] * r[0] + C[i][1] * r[1] + C[i][2] * r[2];
        e[3 + i] = C[i][0] * wr[0] + C[i][1] * wr[1] + C[i][2] * wr[2];
    }
    for (int i = 0; i < 4; ++i)
        e[6 + i] = q_[i];
}

template <typename T>
void RawGnssEKF<T>::leverAt(double age_s, T p_ned[3], T v_ned[3], T nose_ned[3], T q_ned[4]) const
{
    T qs[4];
    T* q = q_ned != nullptr ? q_ned : qs;
    for (int i = 0; i < 3; ++i)
        p_ned[i] = v_ned[i] = T(0);
    for (int i = 0; i < 4; ++i)
        q[i] = T(0);
    if (p_.lever_on && hn_ < 1)
    {
        // no history yet: where the antenna sits now, not moving
        T C[3][3];
        bodyToNed(q_, C);
        for (int i = 0; i < 3; ++i)
            p_ned[i] = T(C[i][0] * T(p_.lever_b_m[0]) + C[i][1] * T(p_.lever_b_m[1]) + C[i][2] * T(p_.lever_b_m[2]));
        for (int i = 0; i < 4; ++i)
            q[i] = q_[i];
    }
    else if (p_.lever_on)
    {
        auto slot = [&](int k) { return (hhead_ + kHist - hn_ + k) % kHist; };
        const double t0 = t_ - (age_s > 0.0 ? age_s : 0.0);
        // the newest entry ending at or before t0 is k - 1, the one after it k
        int k = hn_ - 1;
        while (k > 0 && hist_[slot(k - 1)].t >= t0)
            --k;
        const T* b = lev_[slot(k)];
        if (k == 0 || hist_[slot(k)].t <= t0)
        {
            // before the oldest entry's end, or after the newest's: that entry's
            for (int i = 0; i < 3; ++i)
            {
                p_ned[i] = b[i];
                v_ned[i] = b[3 + i];
            }
            for (int i = 0; i < 4; ++i)
                q[i] = b[6 + i];
        }
        else
        {
            const T* a = lev_[slot(k - 1)];
            const double ta = hist_[slot(k - 1)].t, tb = hist_[slot(k)].t;
            const T w = tb > ta ? T((t0 - ta) / (tb - ta)) : T(1);
            for (int i = 0; i < 3; ++i)
            {
                p_ned[i] = a[i] + w * (b[i] - a[i]);
                v_ned[i] = a[3 + i] + w * (b[3 + i] - a[3 + i]);
            }
            // the attitude: the two entries' quaternions on one side (q and -q are one attitude), their
            // weighted mean normalized -- a step apart, within 4 urad of the arc at 10 rad/s and 10 ms
            const T* qa = a + 6;
            const T* qb = b + 6;
            const T sb = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3] < T(0) ? T(-1) : T(1);
            T n2 = T(0);
            for (int i = 0; i < 4; ++i)
            {
                q[i] = qa[i] + w * (sb * qb[i] - qa[i]);
                n2 += q[i] * q[i];
            }
            if (n2 > T(0))
            {
                const T s = T(1) / std::sqrt(n2);
                for (int i = 0; i < 4; ++i)
                    q[i] *= s;
            }
        }
    }
    if (nose_ned != nullptr)
    {
        // body x, NED: C(q)'s first column in its homogeneous form, so none where there is no attitude (the
        // lever off, or an entry from before it was on)
        nose_ned[0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        nose_ned[1] = T(2) * (q[1] * q[2] + q[0] * q[3]);
        nose_ned[2] = T(2) * (q[1] * q[3] - q[0] * q[2]);
    }
}

// The history's change from the epoch (dp_e, dv_e) and from the Doppler's
// instant (dv_l) is the IMU's; less the antenna's offset and velocity there,
// back-dating takes the receiver to the antenna: r_e = r - (dp_e - p_lev),
// v = v_now - (dv - v_lev). The Doppler's instant's offset and attitude are
// leverRows'.
template <typename T>
void RawGnssEKF<T>::leverBack(double age_s, double tau, T dp_e[3], T dv_e[3], T dv_l[3], LeverEpoch& lv) const
{
    T q_l[4];
    leverAt(age_s, lv.p_e, lv.v_e, nullptr, q_l);
    if (tau > 0.0)
    {
        leverAt(age_s + tau, lv.p_l, lv.v_l, nullptr, q_l);
    }
    else
    {
        for (int i = 0; i < 3; ++i)
        {
            lv.p_l[i] = lv.p_e[i];
            lv.v_l[i] = lv.v_e[i];
        }
    }
    bodyToNed(q_l, lv.C_l);
    for (int i = 0; i < 3; ++i)
    {
        dp_e[i] -= lv.p_e[i];
        dv_e[i] -= lv.v_e[i];
        dv_l[i] -= lv.v_l[i];
    }
}

template <typename T>
int RawGnssEKF<T>::leverRows(const T u_ned[3], const LeverEpoch& lv, int* hidx, T* hval, int hn, T& R) const
{
    T C[3][3];
    bodyToNed(q_, C);
    const T* v = lv.v_l;
    const T* p = lv.p_l;
    const T (&Cl)[3][3] = lv.C_l;
    // u x v and u x p, NED
    const T uv[3] = {u_ned[1] * v[2] - u_ned[2] * v[1], u_ned[2] * v[0] - u_ned[0] * v[2],
                     u_ned[0] * v[1] - u_ned[1] * v[0]};
    const T up[3] = {u_ned[1] * p[2] - u_ned[2] * p[1], u_ned[2] * p[0] - u_ned[0] * p[2],
                     u_ned[0] * p[1] - u_ned[1] * p[0]};
    // the attitude's (the state a half-angle, in the body now: a rotation 2a turns the antenna's velocity
    // by 2 a x v in the body, C (2 a x (w x r)) = -2 [v x] C a -- an error turns NED alike then and now --
    // which the range rate sees as +2 (u x v)' C a), and the gyro bias's (a bias db turns the rate the
    // body had at the Doppler's instant by -db, in the body as it stood then: C_l (-db x r) = [p x] C_l db,
    // seen as -(u x p)' C_l db; C now would be off by the body's turn since, 89 deg at 10 rad/s and 0.155 s)
    T h[6];
    for (int j = 0; j < 3; ++j)
    {
        h[j] = T(2) * (uv[0] * C[0][j] + uv[1] * C[1][j] + uv[2] * C[2][j]);
        h[3 + j] = -(up[0] * Cl[0][j] + up[1] * Cl[1][j] + up[2] * Cl[2][j]);
    }
    if (p_.lever_att_mode == kLeverAttFuse)
    {
        for (int j = 0; j < 3; ++j)
        {
            hidx[hn] = kAtt + j;
            hval[hn++] = h[j];
        }
        for (int j = 0; j < 3; ++j)
        {
            hidx[hn] = kGyroBias + j;
            hval[hn++] = h[3 + j];
        }
        return hn;
    }
    // considered: h P h' over the attitude and gyro bias (their cross-covariances in), onto R
    const int idx[6] = {kAtt, kAtt + 1, kAtt + 2, kGyroBias, kGyroBias + 1, kGyroBias + 2};
    T hph6 = T(0);
    for (int a = 0; a < 6; ++a)
        for (int b = 0; b < 6; ++b)
            hph6 += h[a] * P_[idx[a]][idx[b]] * h[b];
    if (hph6 > T(0))
        R += hph6;
    return hn;
}

// att_exact's blocks of Phi (TR_RawGnssEKF.h, propagateCovariance). With K =
// [w x] / |w| and th = |w| dt (Rodrigues),
//   exp(-[w x] s)            = I - sin(|w| s) K + (1 - cos(|w| s)) K^2,
//   int_0^dt exp(-[w x] s) ds = dt I - (1 - cos th) / |w| K + (dt - sin th / |w|) K^2.
// Per step, not per epoch, but only with att_exact: here, with the carrier's
// code, which main/raw_gnss_iram.lf leaves in flash. On the Beetle, which
// never turns it on, that costs nothing. On a -DTR_RAW_GNSS_IRAM=1 bench of
// the mosaic image, propagateCovariance runs from internal RAM but fetches
// this from flash every step, so that bench's prop_us is not the cost of an
// internal-RAM placement (the flight image keeps the whole filter in flash).
template <typename T>
void RawGnssEKF<T>::attExactBlocks(const T w_b[3], T dt, T Ra[3][3], T Ga[3][3]) const
{
    const T wn = std::sqrt(w_b[0] * w_b[0] + w_b[1] * w_b[1] + w_b[2] * w_b[2]);
    T a = T(0), b = T(0), c = T(0), d = T(0);
    T K[3][3] = {};
    if (wn > T(1e-9))
    {
        const T th = wn * dt;
        const T k[3] = {w_b[0] / wn, w_b[1] / wn, w_b[2] / wn};
        K[0][1] = -k[2];
        K[0][2] = k[1];
        K[1][0] = k[2];
        K[1][2] = -k[0];
        K[2][0] = -k[1];
        K[2][1] = k[0];
        a = std::sin(th);
        b = T(1) - std::cos(th);
        c = b / wn;
        d = dt - a / wn;
    }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            const T k2 = K[i][0] * K[0][j] + K[i][1] * K[1][j] + K[i][2] * K[2][j];
            const T I = i == j ? T(1) : T(0);
            Ra[i][j] = I - a * K[i][j] + b * k2;
            Ga[i][j] = T(-0.5) * (dt * I - c * K[i][j] + d * k2);
        }
}

// Each member here, for both scalars (TR_RawGnssEKF.cpp instantiates the rest).
#define TR_RAW_GNSS_CARRIER_INSTANTIATE(T)                                                                        \
    template void RawGnssEKF<T>::crSnapshot(double, const T[3]);                                                 \
    template void RawGnssEKF<T>::historyExcess(double, double[3]) const;                                         \
    template int RawGnssEKF<T>::carrierUpdate(const int*, const T*, int, T, T, T, const T*, T[RawGnssEKF<T>::N], \
                                              T&, T*);                                                           \
    template void RawGnssEKF<T>::crEnds(const double[3][3], double[3], double[3]) const;                         \
    template double RawGnssEKF<T>::carrierRate(const RawMeas&, const double[3][3], T[3]) const;                  \
    template int RawGnssEKF<T>::beginCarrier(const RawMeas*, int, double, RawGnssStats&);                        \
    template bool RawGnssEKF<T>::crFitSolve(double[4]);                                                          \
    template void RawGnssEKF<T>::fuseCarrier(const RawMeas*, RawGnssStats&);                                     \
    template void RawGnssEKF<T>::endCarrier(RawGnssStats&);                                                      \
    template void RawGnssEKF<T>::screenRangeRates(const RawMeas*, int, double, RawGnssStats&) const;           \
    template bool RawGnssEKF<T>::fitVelocityClock(const double[][4], const double*, const double*, const bool*, int, \
                                                  double[4]);                                                   \
    template bool RawGnssEKF<T>::verticalFit(const double[][4], const double*, const double*, bool*, int, double[4],  \
                                             int&, int&);                                                       \
    template int RawGnssEKF<T>::dopplerChange(const RawMeas*, int, double, const DopplerRef&, DopplerRef&,         \
                                              dvc::Row*, DopplerChangeImu&) const;                              \
    template double RawGnssEKF<T>::pairAccel(int, const RawMeas&, const Los&, const double[3]) const;           \
    template T RawGnssEKF<T>::hdgObsScaleNow(double) const;                                                     \
    template void RawGnssEKF<T>::setLever(bool, const double[3], uint8_t);                                       \
    template void RawGnssEKF<T>::pushLever(const T[3]);                                                          \
    template void RawGnssEKF<T>::leverAt(double, T[3], T[3], T[3], T[4]) const;                                  \
    template void RawGnssEKF<T>::leverBack(double, double, T[3], T[3], T[3], LeverEpoch&) const;                 \
    template int RawGnssEKF<T>::leverRows(const T[3], const LeverEpoch&, int*, T*, int, T&) const;              \
    template void RawGnssEKF<T>::attExactBlocks(const T[3], T, T[3][3], T[3][3]) const;
TR_RAW_GNSS_CARRIER_INSTANTIATE(float)
TR_RAW_GNSS_CARRIER_INSTANTIATE(double)
#undef TR_RAW_GNSS_CARRIER_INSTANTIATE

}  // namespace gnss_raw
