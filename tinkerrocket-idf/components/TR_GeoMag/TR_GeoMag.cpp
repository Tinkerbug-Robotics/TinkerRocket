// ─── World Magnetic Model 2025 — declination evaluation ─────────────────────
//
// Spherical-harmonic field evaluation adapted from the NOAA/NCEI reference
// implementation (geomag70.c, public domain).  Degree/order 12.  Returns
// magnetic declination only.  See TR_GeoMag.h for provenance and the 2030.0
// expiry.

#include "TR_GeoMag.h"

#include <cmath>

namespace TR_GeoMag {

namespace {

constexpr int    MAXORD = 12;
constexpr double A_WGS84 = 6378.137;        // semi-major axis (km)
constexpr double B_WGS84 = 6356.7523142;    // semi-minor axis (km)
constexpr double RE_GEO  = 6371.2;          // geomagnetic reference radius (km)

// WMM2025 Gauss coefficients (Schmidt semi-normalized), epoch 2025.0.
// One row per term: { n, m, g_nm (nT), h_nm (nT), gdot (nT/yr), hdot (nT/yr) }.
struct Coef { int n, m; double g, h, gd, hd; };
constexpr Coef WMM2025[] = {
    { 1, 0, -29351.8,     0.0,  12.0,   0.0},
    { 1, 1,  -1410.8,  4545.4,   9.7, -21.5},
    { 2, 0,  -2556.6,     0.0, -11.6,   0.0},
    { 2, 1,   2951.1, -3133.6,  -5.2, -27.7},
    { 2, 2,   1649.3,  -815.1,  -8.0, -12.1},
    { 3, 0,   1361.0,     0.0,  -1.3,   0.0},
    { 3, 1,  -2404.1,   -56.6,  -4.2,   4.0},
    { 3, 2,   1243.8,   237.5,   0.4,  -0.3},
    { 3, 3,    453.6,  -549.5, -15.6,  -4.1},
    { 4, 0,    895.0,     0.0,  -1.6,   0.0},
    { 4, 1,    799.5,   278.6,  -2.4,  -1.1},
    { 4, 2,     55.7,  -133.9,  -6.0,   4.1},
    { 4, 3,   -281.1,   212.0,   5.6,   1.6},
    { 4, 4,     12.1,  -375.6,  -7.0,  -4.4},
    { 5, 0,   -233.2,     0.0,   0.6,   0.0},
    { 5, 1,    368.9,    45.4,   1.4,  -0.5},
    { 5, 2,    187.2,   220.2,   0.0,   2.2},
    { 5, 3,   -138.7,  -122.9,   0.6,   0.4},
    { 5, 4,   -142.0,    43.0,   2.2,   1.7},
    { 5, 5,     20.9,   106.1,   0.9,   1.9},
    { 6, 0,     64.4,     0.0,  -0.2,   0.0},
    { 6, 1,     63.8,   -18.4,  -0.4,   0.3},
    { 6, 2,     76.9,    16.8,   0.9,  -1.6},
    { 6, 3,   -115.7,    48.8,   1.2,  -0.4},
    { 6, 4,    -40.9,   -59.8,  -0.9,   0.9},
    { 6, 5,     14.9,    10.9,   0.3,   0.7},
    { 6, 6,    -60.7,    72.7,   0.9,   0.9},
    { 7, 0,     79.5,     0.0,  -0.0,   0.0},
    { 7, 1,    -77.0,   -48.9,  -0.1,   0.6},
    { 7, 2,     -8.8,   -14.4,  -0.1,   0.5},
    { 7, 3,     59.3,    -1.0,   0.5,  -0.8},
    { 7, 4,     15.8,    23.4,  -0.1,   0.0},
    { 7, 5,      2.5,    -7.4,  -0.8,  -1.0},
    { 7, 6,    -11.1,   -25.1,  -0.8,   0.6},
    { 7, 7,     14.2,    -2.3,   0.8,  -0.2},
    { 8, 0,     23.2,     0.0,  -0.1,   0.0},
    { 8, 1,     10.8,     7.1,   0.2,  -0.2},
    { 8, 2,    -17.5,   -12.6,   0.0,   0.5},
    { 8, 3,      2.0,    11.4,   0.5,  -0.4},
    { 8, 4,    -21.7,    -9.7,  -0.1,   0.4},
    { 8, 5,     16.9,    12.7,   0.3,  -0.5},
    { 8, 6,     15.0,     0.7,   0.2,  -0.6},
    { 8, 7,    -16.8,    -5.2,  -0.0,   0.3},
    { 8, 8,      0.9,     3.9,   0.2,   0.2},
    { 9, 0,      4.6,     0.0,  -0.0,   0.0},
    { 9, 1,      7.8,   -24.8,  -0.1,  -0.3},
    { 9, 2,      3.0,    12.2,   0.1,   0.3},
    { 9, 3,     -0.2,     8.3,   0.3,  -0.3},
    { 9, 4,     -2.5,    -3.3,  -0.3,   0.3},
    { 9, 5,    -13.1,    -5.2,   0.0,   0.2},
    { 9, 6,      2.4,     7.2,   0.3,  -0.1},
    { 9, 7,      8.6,    -0.6,  -0.1,  -0.2},
    { 9, 8,     -8.7,     0.8,   0.1,   0.4},
    { 9, 9,    -12.9,    10.0,  -0.1,   0.1},
    {10, 0,     -1.3,     0.0,   0.1,   0.0},
    {10, 1,     -6.4,     3.3,   0.0,   0.0},
    {10, 2,      0.2,     0.0,   0.1,  -0.0},
    {10, 3,      2.0,     2.4,   0.1,  -0.2},
    {10, 4,     -1.0,     5.3,  -0.0,   0.1},
    {10, 5,     -0.6,    -9.1,  -0.3,  -0.1},
    {10, 6,     -0.9,     0.4,   0.0,   0.1},
    {10, 7,      1.5,    -4.2,  -0.1,   0.0},
    {10, 8,      0.9,    -3.8,  -0.1,  -0.1},
    {10, 9,     -2.7,     0.9,  -0.0,   0.2},
    {10,10,     -3.9,    -9.1,  -0.0,  -0.0},
    {11, 0,      2.9,     0.0,   0.0,   0.0},
    {11, 1,     -1.5,     0.0,  -0.0,  -0.0},
    {11, 2,     -2.5,     2.9,   0.0,   0.1},
    {11, 3,      2.4,    -0.6,   0.0,  -0.0},
    {11, 4,     -0.6,     0.2,   0.0,   0.1},
    {11, 5,     -0.1,     0.5,  -0.1,  -0.0},
    {11, 6,     -0.6,    -0.3,   0.0,  -0.0},
    {11, 7,     -0.1,    -1.2,  -0.0,   0.1},
    {11, 8,      1.1,    -1.7,  -0.1,  -0.0},
    {11, 9,     -1.0,    -2.9,  -0.1,   0.0},
    {11,10,     -0.2,    -1.8,  -0.1,   0.0},
    {11,11,      2.6,    -2.3,  -0.1,   0.0},
    {12, 0,     -2.0,     0.0,   0.0,   0.0},
    {12, 1,     -0.2,    -1.3,   0.0,  -0.0},
    {12, 2,      0.3,     0.7,  -0.0,   0.0},
    {12, 3,      1.2,     1.0,  -0.0,  -0.1},
    {12, 4,     -1.3,    -1.4,  -0.0,   0.1},
    {12, 5,      0.6,    -0.0,  -0.0,  -0.0},
    {12, 6,      0.6,     0.6,   0.1,  -0.0},
    {12, 7,      0.5,    -0.1,  -0.0,  -0.0},
    {12, 8,     -0.1,     0.8,   0.0,   0.0},
    {12, 9,     -0.4,     0.1,   0.0,  -0.0},
    {12,10,     -0.2,    -1.0,  -0.1,  -0.0},
    {12,11,     -1.3,     0.1,  -0.0,   0.0},
    {12,12,     -0.7,     0.2,  -0.1,  -0.1},
};

// Schmidt-renormalized coefficients (filled once).  g[n][m], h[n][m] and the
// secular-variation rates.  k/fn/fm are the Legendre-recursion constants.
struct Model {
    double g[MAXORD + 1][MAXORD + 1] = {};
    double h[MAXORD + 1][MAXORD + 1] = {};
    double gd[MAXORD + 1][MAXORD + 1] = {};
    double hd[MAXORD + 1][MAXORD + 1] = {};
    double k[MAXORD + 1][MAXORD + 1] = {};
    double fn[MAXORD + 1] = {};
    double fm[MAXORD + 1] = {};
    bool   ready = false;
};

Model& model() {
    static Model M;
    if (M.ready) return M;

    // Load raw coefficients into [n][m].
    for (const Coef& c : WMM2025) {
        M.g[c.n][c.m]  = c.g;   M.h[c.n][c.m]  = c.h;
        M.gd[c.n][c.m] = c.gd;  M.hd[c.n][c.m] = c.hd;
    }

    // Schmidt semi-normalization for the geomag70 Legendre recursion.
    // snorm[n][m] carries the normalization factor; coefficients are scaled by
    // it, and k[n][m] are the recursion constants.
    double snorm[MAXORD + 1][MAXORD + 1] = {};
    snorm[0][0] = 1.0;
    for (int n = 1; n <= MAXORD; n++) {
        snorm[n][0] = snorm[n - 1][0] * (double)(2 * n - 1) / (double)n;
        int j = 2;
        for (int m = 0; m <= n; m++) {
            M.k[n][m] = (double)(((n - 1) * (n - 1)) - (m * m)) /
                        (double)((2 * n - 1) * (2 * n - 3));
            if (m > 0) {
                double flnmj = (double)((n - m + 1) * j) / (double)(n + m);
                snorm[n][m] = snorm[n][m - 1] * std::sqrt(flnmj);
                j = 1;
                M.h[n][m]  *= snorm[n][m];
                M.hd[n][m] *= snorm[n][m];
            }
            M.g[n][m]  *= snorm[n][m];
            M.gd[n][m] *= snorm[n][m];
        }
        M.fn[n] = (double)(n + 1);
        M.fm[n] = (double)n;
    }
    M.k[1][1] = 0.0;
    M.ready = true;
    return M;
}

}  // namespace

float declinationRad(double lat_rad, double lon_rad, double alt_m, double decimal_year) {
    const Model& M = model();

    // Clamp the epoch to the model's valid range (extrapolation only degrades).
    double t = decimal_year;
    if (t < WMM_VALID_MIN) t = WMM_VALID_MIN;
    if (t > WMM_VALID_MAX) t = WMM_VALID_MAX;
    const double dt = t - WMM_EPOCH;

    const double glat = lat_rad;        // already radians
    const double alt_km = alt_m / 1000.0;

    const double srlon = std::sin(lon_rad), crlon = std::cos(lon_rad);
    const double srlat = std::sin(glat),   crlat = std::cos(glat);
    const double srlat2 = srlat * srlat, crlat2 = crlat * crlat;

    // Geodetic → geocentric spherical coordinates.
    const double a2 = A_WGS84 * A_WGS84, b2 = B_WGS84 * B_WGS84;
    const double c2 = a2 - b2, a4 = a2 * a2, b4 = b2 * b2, c4 = a4 - b4;
    const double q  = std::sqrt(a2 - c2 * srlat2);
    const double q1 = alt_km * q;
    const double q2 = ((q1 + a2) / (q1 + b2)) * ((q1 + a2) / (q1 + b2));
    const double ct = srlat / std::sqrt(q2 * crlat2 + srlat2);   // cos(colatitude)
    const double st = std::sqrt(1.0 - ct * ct);                  // sin(colatitude)
    const double r  = std::sqrt(alt_km * alt_km + 2.0 * q1 +
                                (a4 - c4 * srlat2) / (q * q));
    const double d  = std::sqrt(a2 * crlat2 + b2 * srlat2);
    const double ca = (alt_km + d) / r;                          // cos(geocentric-geodetic)
    const double sa = c2 * srlat * crlat / (r * d);              // sin(geocentric-geodetic)

    double sp[MAXORD + 1], cp[MAXORD + 1];
    sp[0] = 0.0; cp[0] = 1.0;
    sp[1] = srlon; cp[1] = crlon;
    for (int m = 2; m <= MAXORD; m++) {
        sp[m] = sp[1] * cp[m - 1] + cp[1] * sp[m - 1];
        cp[m] = cp[1] * cp[m - 1] - sp[1] * sp[m - 1];
    }

    // Associated Legendre values (snorm) + derivatives (dp), rebuilt each call.
    double snorm[MAXORD + 1][MAXORD + 1] = {};
    double dp[MAXORD + 1][MAXORD + 1] = {};
    double pp[MAXORD + 1] = {};
    snorm[0][0] = 1.0; pp[0] = 1.0; dp[0][0] = 0.0;

    const double aor = RE_GEO / r;
    double ar = aor * aor;
    double br = 0.0, bt = 0.0, bp = 0.0, bpp = 0.0;

    for (int n = 1; n <= MAXORD; n++) {
        ar *= aor;
        for (int m = 0; m <= n; m++) {
            // Schmidt Legendre recursion (geomag70).
            if (n == m) {
                snorm[n][m] = st * snorm[n - 1][m - 1];
                dp[n][m] = st * dp[n - 1][m - 1] + ct * snorm[n - 1][m - 1];
            } else if (n == 1 && m == 0) {
                snorm[n][m] = ct * snorm[n - 1][m];
                dp[n][m] = ct * dp[n - 1][m] - st * snorm[n - 1][m];
            } else {  // n > 1 && n != m
                if (m > n - 2) { snorm[n - 2][m] = 0.0; dp[n - 2][m] = 0.0; }
                snorm[n][m] = ct * snorm[n - 1][m] - M.k[n][m] * snorm[n - 2][m];
                dp[n][m] = ct * dp[n - 1][m] - st * snorm[n - 1][m] -
                           M.k[n][m] * dp[n - 2][m];
            }

            // Time-adjusted coefficients.
            const double gt = M.g[n][m] + dt * M.gd[n][m];
            const double ht = M.h[n][m] + dt * M.hd[n][m];

            const double par = ar * snorm[n][m];
            double temp1, temp2;
            if (m == 0) { temp1 = gt * cp[m];              temp2 = gt * sp[m]; }
            else        { temp1 = gt * cp[m] + ht * sp[m]; temp2 = gt * sp[m] - ht * cp[m]; }

            bt -= ar * temp1 * dp[n][m];
            bp += M.fm[m] * temp2 * par;
            br += M.fn[n] * temp1 * par;

            // North-geographic-pole limit (st → 0): use the L'Hôpital branch.
            if (st == 0.0 && m == 1) {
                if (n == 1) pp[n] = pp[n - 1];
                else        pp[n] = ct * pp[n - 1] - M.k[n][m] * pp[n - 2];
                bpp += M.fm[m] * temp2 * (ar * pp[n]);
            }
        }
    }

    if (st == 0.0) bp = bpp;
    else           bp /= st;

    // Spherical → geodetic field components (North, East).  Declination needs
    // only the horizontal pair.
    const double bx = -bt * ca - br * sa;   // geodetic north
    const double by = bp;                   // east

    return (float)std::atan2(by, bx);       // declination, radians (east +)
}

}  // namespace TR_GeoMag
