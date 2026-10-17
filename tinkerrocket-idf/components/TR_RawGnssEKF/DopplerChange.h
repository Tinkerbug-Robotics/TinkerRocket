// DopplerChange.h -- the arithmetic of the Doppler change check
// (RawGnssShadow's dv_check; TR_RawGnssEKF::dopplerChange builds its rows).
//
// A Doppler that is stale -- a tracking loop still settling after the thrust,
// a receiver handing over an old velocity -- can sit within the innovation
// gate, which judges each Doppler against the filter's own velocity, so a
// filter already pulled off agrees with it. Its CHANGE from one epoch to the
// next cannot hide so: the IMU says how the receiver's velocity changed over
// the interval, and each satellite's range rate must have changed by that
// change along its line of sight, plus the satellite's own (its ephemeris),
// plus the receiver clock's drift change, which every satellite shares. The
// filter's velocity, right or wrong, is the same at both ends and drops out.
//
// Per satellite, over the interval from the reference epoch to this one:
//
//   z = [rr - pred(v0 + dv_imu)]_now - [rr - pred(v0)]_then
//     = -u . w + c + (the loop's own change)
//
// with v0 the filter's velocity at the reference (any velocity would do: it
// cancels to first order), dv_imu the IMU's velocity change (specific force
// and gravity, no filter correction), u the line of sight (NED, receiver to
// satellite), w the receiver's true velocity change less the IMU's, and c the
// clock drift's change. The epoch's rows are fitted for w and c by weighted
// least squares. Then:
//
//   - a satellite: its residual off the fit, in its own 1-sigmas. Whatever
//     the receiver did as a whole -- an attitude error turning the IMU's
//     change, an antenna off the IMU turning with the body (an unknown lever
//     arm), the IMU's scale, the clock -- is one velocity and one rate, and
//     the fit takes it out: only a satellite on its own is left;
//   - the epoch (the common-mode part): w against its covariance and what
//     the IMU itself can be off by -- its specific force's change turned by
//     an attitude allowance (a filter whose heading is wrong must not lose
//     the epochs that right it: an attitude error only turns the change, and
//     a 30 deg allowance leaves the part along the specific force to judge),
//     scaled by a scale allowance, a bias allowance's velocity over the
//     interval, and a lever arm's change at the gyro's rates. A receiver
//     whose loops lag reports less of the change, along it: that is what is
//     left to see.
//
// Pure and host-compilable (tests_cpp/test_doppler_change.cpp).
#ifndef RAW_GNSS_DOPPLER_CHANGE_H
#define RAW_GNSS_DOPPLER_CHANGE_H

#include <cmath>

namespace gnss_raw
{
namespace dvc
{

// One satellite's change (TR_RawGnssEKF::dopplerChange fills all but in and e)
struct Row
{
    double u[3];       // its line of sight, NED, receiver to satellite
    double z;          // its range rate's change less the predicted (m/s)
    double s;          // its 1-sigma (m/s)
    int    im;         // its index in the epoch's measurements
    int    iref;       //   in the reference's
    int    iout;       //   in the reference the epoch leaves for the next
    bool   in;         // in the fit (fit)
    double e;          // its residual off the fit, in its own 1-sigmas (fit)
};

// The fit: w (the receiver's velocity change less the IMU's, NED, m/s) and c
// (the clock drift's change, m/s), x = (w, c), and its covariance
struct Fit
{
    bool   ok = false;
    int    n = 0;          // rows
    int    nin = 0;        //   in the last fit
    int    nout = 0;       //   left out of it, each far off on its own
    double x[4] = {};
    double C[4][4] = {};
    double chi2 = 0.0;     // the rows in: their e squared, summed
};

// x = (w, c) by least squares on the rows in, each weighted by 1/s^2, H =
// [-u, 1]; C its covariance (H'WH)^-1. False under four rows in, or if the
// geometry gives no velocity.
inline bool solve(const Row* r, int n, double x[4], double C[4][4])
{
    double M[4][5] = {};
    int nin = 0;
    for (int i = 0; i < n; ++i)
    {
        if (!r[i].in)
            continue;
        nin++;
        const double h[4] = {-r[i].u[0], -r[i].u[1], -r[i].u[2], 1.0};
        const double w = 1.0 / (r[i].s * r[i].s);
        for (int j = 0; j < 4; ++j)
        {
            for (int k = 0; k < 4; ++k)
                M[j][k] += w * h[j] * h[k];
            M[j][4] += w * h[j] * r[i].z;
        }
    }
    if (nin < 4)
        return false;
    // [N | b | I] -> [I | x | N^-1], Gauss-Jordan with partial pivoting
    double scale = 0.0;                   // the normal matrix's size, for a pivot test that ignores the weights' units
    for (int j = 0; j < 4; ++j)
        scale = std::fmax(scale, std::fabs(M[j][j]));
    double A[4][9];
    for (int j = 0; j < 4; ++j)
    {
        for (int k = 0; k < 5; ++k)
            A[j][k] = M[j][k];
        for (int k = 0; k < 4; ++k)
            A[j][5 + k] = j == k ? 1.0 : 0.0;
    }
    for (int c = 0; c < 4; ++c)
    {
        int piv = c;
        for (int r2 = c + 1; r2 < 4; ++r2)
            if (std::fabs(A[r2][c]) > std::fabs(A[piv][c]))
                piv = r2;
        if (!(std::fabs(A[piv][c]) > 1e-12 * scale))
            return false;
        if (piv != c)
            for (int k = 0; k < 9; ++k)
            {
                const double t = A[c][k];
                A[c][k] = A[piv][k];
                A[piv][k] = t;
            }
        const double d = A[c][c];
        for (int k = 0; k < 9; ++k)
            A[c][k] /= d;
        for (int r2 = 0; r2 < 4; ++r2)
        {
            if (r2 == c)
                continue;
            const double f = A[r2][c];
            if (f == 0.0)
                continue;
            for (int k = 0; k < 9; ++k)
                A[r2][k] -= f * A[c][k];
        }
    }
    for (int j = 0; j < 4; ++j)
    {
        x[j] = A[j][4];
        for (int k = 0; k < 4; ++k)
            C[j][k] = A[j][5 + k];
    }
    return true;
}

// Each row's residual off x, in its own 1-sigmas
inline void residuals(Row* r, int n, const double x[4])
{
    for (int i = 0; i < n; ++i)
    {
        const double pred = -r[i].u[0] * x[0] - r[i].u[1] * x[1] - r[i].u[2] * x[2] + x[3];
        r[i].e = (r[i].z - pred) / r[i].s;
    }
}

// The fit, robust to a few satellites gone on their own: while more than
// min_in rows are in, the one most of its own 1-sigmas off the fit, past k,
// is left out and the rest fitted again (the vertical check's way,
// TR_RawGnssEKF::verticalFit). Every row's e is then off the last fit, in or
// out. Not ok under four rows or with no velocity in the geometry.
inline Fit fit(Row* r, int n, double k, int min_in = 6)
{
    Fit f;
    f.n = n;
    for (int i = 0; i < n; ++i)
        r[i].in = true;
    f.nin = n;
    for (;;)
    {
        if (!solve(r, n, f.x, f.C))
            return f;
        residuals(r, n, f.x);
        if (f.nin <= min_in)
            break;
        int worst = -1;
        double worst_e = k;
        for (int i = 0; i < n; ++i)
            if (r[i].in && std::fabs(r[i].e) > worst_e)
            {
                worst_e = std::fabs(r[i].e);
                worst = i;
            }
        if (worst < 0)
            break;
        r[worst].in = false;
        f.nin--;
        f.nout++;
    }
    f.chi2 = 0.0;
    for (int i = 0; i < n; ++i)
        if (r[i].in)
            f.chi2 += r[i].e * r[i].e;
    f.ok = true;
    return f;
}

// What the IMU's own velocity change can be off by (the epoch test's
// allowances), each a 1-sigma
struct Allow
{
    double att_rad = 0.0;   // the attitude: the specific force's change turned by this, about any axis
    double scale = 0.0;     // the accelerometer's scale: that change off by this fraction, along it
    double bias_mps = 0.0;  // a bias over the interval: this much on every axis (m/s)
    double lever_mps = 0.0; // an antenna off the IMU turning with the body: this much on every axis (m/s)
};

// The epoch test's covariance: Cw (the fit's, w's 3x3) plus the allowances
// about sf (the IMU's specific-force velocity change, NED, m/s):
//   att^2 (|sf|^2 I - sf sf') + scale^2 sf sf' + (bias^2 + lever^2) I
inline void allowCov(const double Cw[3][3], const double sf[3], const Allow& a, double S[3][3])
{
    const double s2 = sf[0] * sf[0] + sf[1] * sf[1] + sf[2] * sf[2];
    const double a2 = a.att_rad * a.att_rad, k2 = a.scale * a.scale;
    const double iso = a.bias_mps * a.bias_mps + a.lever_mps * a.lever_mps;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            S[i][j] = Cw[i][j] + (k2 - a2) * sf[i] * sf[j] + (i == j ? a2 * s2 + iso : 0.0);
}

// w' S^-1 w for a symmetric S (its inverse by cofactors); -1 if S is singular
inline double chi2Sym3(const double w[3], const double S[3][3])
{
    const double c00 = S[1][1] * S[2][2] - S[1][2] * S[2][1];
    const double c01 = S[1][2] * S[2][0] - S[1][0] * S[2][2];
    const double c02 = S[1][0] * S[2][1] - S[1][1] * S[2][0];
    const double det = S[0][0] * c00 + S[0][1] * c01 + S[0][2] * c02;
    if (!(std::fabs(det) > 0.0) || !std::isfinite(det))
        return -1.0;
    const double inv[3][3] = {
        {c00 / det, (S[0][2] * S[2][1] - S[0][1] * S[2][2]) / det, (S[0][1] * S[1][2] - S[0][2] * S[1][1]) / det},
        {c01 / det, (S[0][0] * S[2][2] - S[0][2] * S[2][0]) / det, (S[0][2] * S[1][0] - S[0][0] * S[1][2]) / det},
        {c02 / det, (S[0][1] * S[2][0] - S[0][0] * S[2][1]) / det, (S[0][0] * S[1][1] - S[0][1] * S[1][0]) / det}};
    double q = 0.0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            q += w[i] * inv[i][j] * w[j];
    return q;
}

// The epoch test: w's chi-square (3 dof) against the fit's covariance and
// the allowances
inline double epochChi2(const double w[3], const double Cw[3][3], const double sf[3], const Allow& a)
{
    double S[3][3];
    allowCov(Cw, sf, a, S);
    return chi2Sym3(w, S);
}

// w's part along the specific force's change (m/s; 0 with none): a loop that
// lags reports less of the change, so this reads negative
inline double alongSf(const double w[3], const double sf[3])
{
    const double s = std::sqrt(sf[0] * sf[0] + sf[1] * sf[1] + sf[2] * sf[2]);
    return s > 0.0 ? (w[0] * sf[0] + w[1] * sf[1] + w[2] * sf[2]) / s : 0.0;
}

}  // namespace dvc
}  // namespace gnss_raw

#endif  // RAW_GNSS_DOPPLER_CHANGE_H
