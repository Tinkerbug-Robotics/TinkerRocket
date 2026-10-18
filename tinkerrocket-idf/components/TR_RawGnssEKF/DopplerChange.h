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
//     left to see. Where nothing measured the heading (no magnetometer on
//     the pad), a heading allowance about the vertical too: a heading 150
//     deg off reverses the horizontal change, which no small attitude
//     allowance covers, and the vertical is left to judge.
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
// out. keep_out: the rows whose in is false at the call start out of it --
// satellites already found stale, whose slow bias would otherwise pass into
// the receiver's change -- unless that leaves under five in. Not ok under
// four rows or with no velocity in the geometry.
inline Fit fit(Row* r, int n, double k, int min_in = 6, bool keep_out = false)
{
    Fit f;
    f.n = n;
    int nin = 0;
    if (keep_out)
        for (int i = 0; i < n; ++i)
            nin += r[i].in ? 1 : 0;
    if (!keep_out || nin < 5)
    {
        for (int i = 0; i < n; ++i)
            r[i].in = true;
        nin = n;
    }
    f.nin = nin;
    f.nout = n - nin;
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
    double hdg_rad = 0.0;   // the heading: that change turned this far about the vertical (pi or more: any
                            //   heading; 0: att_rad alone)
};

// The heading allowance's size, per unit of the specific force's horizontal
// change: a turn by h about the vertical moves the horizontal part by
// 2 sin(h/2) of its size (the chord), in a direction that depends on h's sign
// and size -- so as an isotropic horizontal 1-sigma. att_rad's small-angle
// form takes only the part across the change; a heading 150 deg off also
// reverses most of the part along it, which only this allows.
inline double headingChord(double hdg_rad)
{
    return hdg_rad >= 3.141592653589793 ? 2.0 : 2.0 * std::sin(0.5 * hdg_rad);
}

// The lever allowance where the antenna's offset is known (RawGnssShadow's
// antenna offset, the EKF's lever_on). Each end's Dopplers are then predicted
// at the antenna -- the IMU's velocity plus the lever's, C (omega x r) at the
// filter's attitude -- so w is the change less the lever's predicted change,
// and what is left of the lever's is what that prediction can be off by: the
// offset's own doubt (doubt_m) at the gyro's rates, and the attitude's error
// turning the predicted change. With the heading known that turn is within
// the attitude allowance: its chord of the offset (r_m, its size). With the
// heading unset it is any heading's, up to twice the change itself, so the
// prediction is taken back out: dv_lev is this end's lever velocity less the
// reference's, as the two ends' predictions took them, and w + dv_lev is the
// change the IMU alone predicts (the fit is linear: a velocity common to
// every row of one end moves its change by that velocity, less only that
// velocity times the line of sight's turn between the ends, 1e-4 rad), whose
// lever is allowed as an unknown offset's bound is, at the whole offset:
// r_m + doubt_m. Returns the arm (m) the allowance's rate term takes; wu is
// the change the epoch test and its window judge.
inline double knownLeverArm(const double w[3], const double dv_lev[3], double r_m, double doubt_m, bool hdg_known,
                            double att_rad, double wu[3])
{
    for (int i = 0; i < 3; ++i)
        wu[i] = hdg_known ? w[i] : w[i] + dv_lev[i];
    return hdg_known ? doubt_m + headingChord(att_rad) * r_m : r_m + doubt_m;
}

// The epoch test's covariance: Cw (the fit's, w's 3x3) plus the allowances
// about sf (the IMU's specific-force velocity change, NED, m/s):
//   att^2 (|sf|^2 I - sf sf') + scale^2 sf sf' + (bias^2 + lever^2) I
//   + chord(hdg)^2 |sf_horizontal|^2 on north and east (hdg_rad > 0 only)
inline void allowCov(const double Cw[3][3], const double sf[3], const Allow& a, double S[3][3])
{
    const double s2 = sf[0] * sf[0] + sf[1] * sf[1] + sf[2] * sf[2];
    const double a2 = a.att_rad * a.att_rad, k2 = a.scale * a.scale;
    const double iso = a.bias_mps * a.bias_mps + a.lever_mps * a.lever_mps;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            S[i][j] = Cw[i][j] + (k2 - a2) * sf[i] * sf[j] + (i == j ? a2 * s2 + iso : 0.0);
    if (a.hdg_rad > 0.0)
    {
        // a heading nothing measured: the IMU's horizontal change can be off
        // by up to its own size twice over, and the vertical not at all
        const double c = headingChord(a.hdg_rad);
        const double h2 = c * c * (sf[0] * sf[0] + sf[1] * sf[1]);
        S[0][0] += h2;
        S[1][1] += h2;
    }
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

// The latch's release (RawGnssShadow's dv_latch_k): the epoch's Dopplers as
// they stand, each less its prediction at the filter's velocity, fitted for
// one velocity offset dv (fit) -- how far the receiver's velocity is from the
// filter's -- against the fit's covariance Cw, the filter's own velocity
// covariance Pv (what the IMU, the barometer and the pseudoranges leave it
// unsure of while the Dopplers are held) and an isotropic floor:
// dv' (Cw + Pv + floor_var I)^-1 dv, 3 dof; -1 if singular. No lever-arm
// allowance: an antenna off the IMU shows here only while the body turns,
// and that allowance at a drogue's deployment (1.5 m/s at 5 rad/s and 0.3 m)
// would let a receiver still biased by more than that back in; a turn only
// keeps the latch on longer. Nor the epoch test's attitude allowance on the
// specific force since: a deployment's shock across it swamps the bias too.
inline double levelChi2(const double dv[3], const double Cw[3][3], const double Pv[3][3], double floor_var)
{
    double S[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            S[i][j] = Cw[i][j] + Pv[i][j] + (i == j ? floor_var : 0.0);
    return chi2Sym3(dv, S);
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
