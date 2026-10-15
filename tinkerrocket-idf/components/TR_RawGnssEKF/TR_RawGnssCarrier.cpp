// TR_RawGnssCarrier.cpp — TR_RawGnssEKF's carrier range rate (see
// TR_RawGnssEKF.h): the screen, the fusion with the epoch's common rate, the
// epoch's close, and the integration's excess back-dated. Once an epoch, so
// apart from the filter's per-step code: on the Beetle this file stays in
// flash where the rest of the filter runs from internal RAM
// (raw_gnss_iram.lf), 9 kB less of it. Its per-step part -- the receiver
// snapshots that every correction moves (inject), the roll rate and the
// integration's excess (propagate, pushHistory) -- is TR_RawGnssEKF.cpp's.
#include "TR_RawGnssEKF.h"

#include <cmath>
#include <cstring>

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
    template void RawGnssEKF<T>::endCarrier(RawGnssStats&);
TR_RAW_GNSS_CARRIER_INSTANTIATE(float)
TR_RAW_GNSS_CARRIER_INSTANTIATE(double)
#undef TR_RAW_GNSS_CARRIER_INSTANTIATE

}  // namespace gnss_raw
