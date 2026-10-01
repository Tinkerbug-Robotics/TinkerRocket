#include "gnss/pvt.h"

#include "gnss/sig.h"

#include <math.h>
#include <string.h>

#define WGS84_A  6378137.0
#define WGS84_F  (1.0 / 298.257223563)
#define WGS84_E2 (WGS84_F * (2.0 - WGS84_F))
#define PI       3.14159265358979323846

void pvt_default_opt(pvt_opt_t *o)
{
    o->el_mask = 5.0 * PI / 180.0;
    o->use_iono = 1;
    o->use_tropo = 1;
    o->raim = 1;
    o->raim_pfa = 1e-4;
    o->raim_max_excl = 2;
    o->coarse_time = 0;
}

void geo_to_ecef(double lat, double lon, double h, double x[3])
{
    double s = sin(lat), c = cos(lat);
    double n = WGS84_A / sqrt(1.0 - WGS84_E2 * s * s);
    x[0] = (n + h) * c * cos(lon);
    x[1] = (n + h) * c * sin(lon);
    x[2] = (n * (1.0 - WGS84_E2) + h) * s;
}

void ecef_to_geo(const double x[3], double *lat, double *lon, double *h)
{
    double p = sqrt(x[0] * x[0] + x[1] * x[1]);
    double la = atan2(x[2], p * (1.0 - WGS84_E2));
    double n = WGS84_A;
    for (int i = 0; i < 10; i++) {
        double s = sin(la);
        n = WGS84_A / sqrt(1.0 - WGS84_E2 * s * s);
        double hh = p / cos(la) - n;
        if (p < 1e-3) {
            hh = fabs(x[2]) - WGS84_A * sqrt(1.0 - WGS84_E2);
        }
        la = atan2(x[2], p * (1.0 - WGS84_E2 * n / (n + hh)));
        *h = hh;
    }
    *lat = la;
    *lon = atan2(x[1], x[0]);
    double s = sin(la);
    n = WGS84_A / sqrt(1.0 - WGS84_E2 * s * s);
    *h = (p > 1e-3) ? p / cos(la) - n : fabs(x[2]) - WGS84_A * sqrt(1.0 - WGS84_E2);
}

/* Unit vector rx -> sat in local east/north/up, giving azimuth and elevation. */
static void az_el(double lat, double lon, const double d[3], double *az, double *el)
{
    double sl = sin(lat), cl = cos(lat), so = sin(lon), co = cos(lon);
    double e = -so * d[0] + co * d[1];
    double n = -sl * co * d[0] - sl * so * d[1] + cl * d[2];
    double u = cl * co * d[0] + cl * so * d[1] + sl * d[2];
    *az = atan2(e, n);
    *el = asin(u / sqrt(e * e + n * n + u * u));
}

double iono_klobuchar(const gps_iono_t *io, double lat, double lon, double az, double el, double t)
{
    /* IS-GPS-200 20.3.3.5.2.5; angles in semicircles. */
    double phi_u = lat / PI, lam_u = lon / PI, e = el / PI;
    double psi = 0.0137 / (e + 0.11) - 0.022;
    double phi_i = phi_u + psi * cos(az);
    if (phi_i > 0.416) {
        phi_i = 0.416;
    } else if (phi_i < -0.416) {
        phi_i = -0.416;
    }
    double lam_i = lam_u + psi * sin(az) / cos(phi_i * PI);
    double phi_m = phi_i + 0.064 * cos((lam_i - 1.617) * PI);
    double tl = fmod(4.32e4 * lam_i + t, 86400.0);
    if (tl < 0.0) {
        tl += 86400.0;
    }
    double f = 1.0 + 16.0 * pow(0.53 - e, 3.0);
    double amp = io->alpha[0] + phi_m * (io->alpha[1] + phi_m * (io->alpha[2] + phi_m * io->alpha[3]));
    double per = io->beta[0] + phi_m * (io->beta[1] + phi_m * (io->beta[2] + phi_m * io->beta[3]));
    if (amp < 0.0) {
        amp = 0.0;
    }
    if (per < 72000.0) {
        per = 72000.0;
    }
    double x = 2.0 * PI * (tl - 50400.0) / per;
    double delay = (fabs(x) < 1.57) ? f * (5e-9 + amp * (1.0 - x * x / 2.0 + x * x * x * x / 24.0)) : f * 5e-9;
    return GNSS_C * delay;
}

double tropo_saastamoinen(double lat, double h, double el)
{
    /* No ceiling at 10 km, where the delay is still 0.6 m at the zenith: a rocket climbs through
     * the rest of it. The troposphere's pressure law carried on upward stays within a few cm of the
     * standard atmosphere's stratosphere, and reaches nothing by 40 km. */
    if (h < -100.0 || h > 4e4 || el <= 0.0) {
        return 0.0;
    }
    double hh = h < 0.0 ? 0.0 : h;
    /* Standard atmosphere, 70 % relative humidity. */
    double p = 1013.25 * pow(1.0 - 2.2557e-5 * hh, 5.2568);
    double tk = 15.0 - 6.5e-3 * hh + 273.16;
    double e = 6.108 * 0.7 * exp((17.15 * tk - 4684.0) / (tk - 38.45));
    double z = PI / 2.0 - el;
    double dry = 0.0022768 * p / (1.0 - 0.00266 * cos(2.0 * lat) - 0.00028 * hh / 1e3) / cos(z);
    double wet = 0.002277 * (1255.0 / tk + 0.05) * e / cos(z);
    return dry + wet;
}

/* Unknowns at most: position, the GPS clock, and an offset for each other system. */
#define PVT_MAX_X (4 + GNSS_SYS_COUNT)

/* Solves the n x n normal equations a x = b in place by Cholesky; a is replaced by its inverse. */
static int solve_spd(double *a, double *b, int n)
{
    double l[PVT_MAX_X * PVT_MAX_X] = {0}, inv[PVT_MAX_X * PVT_MAX_X] = {0};
    for (int i = 0; i < n; i++) {
        for (int j = 0; j <= i; j++) {
            double s = a[i * n + j];
            for (int k = 0; k < j; k++) {
                s -= l[i * n + k] * l[j * n + k];
            }
            if (i == j) {
                if (s <= 0.0) {
                    return -1;
                }
                l[i * n + i] = sqrt(s);
            } else {
                l[i * n + j] = s / l[j * n + j];
            }
        }
    }
    /* Inverse column by column; the solution is inv * b. */
    for (int c = 0; c < n; c++) {
        double y[PVT_MAX_X];
        for (int i = 0; i < n; i++) {
            double s = (i == c) ? 1.0 : 0.0;
            for (int k = 0; k < i; k++) {
                s -= l[i * n + k] * y[k];
            }
            y[i] = s / l[i * n + i];
        }
        for (int i = n - 1; i >= 0; i--) {
            double s = y[i];
            for (int k = i + 1; k < n; k++) {
                s -= l[k * n + i] * inv[k * n + c];
            }
            inv[i * n + c] = s / l[i * n + i];
        }
    }
    double x[PVT_MAX_X];
    for (int i = 0; i < n; i++) {
        x[i] = 0.0;
        for (int k = 0; k < n; k++) {
            x[i] += inv[i * n + k] * b[k];
        }
    }
    memcpy(b, x, sizeof(double) * (size_t)n);
    memcpy(a, inv, sizeof(double) * (size_t)(n * n));
    return 0;
}

typedef struct {
    double pos[3], vel[3];   /* at transmit time, rotated to the receive-time ECEF frame */
    double clk, clk_rate;    /* s, s/s */
} sat_t;

/* A satellite at transmit time t_sv on its own clock: its state, and the time in its system. */
static void sat_state(const gps_eph_t *e, double t_sv, sat_t *s, double *t_sys)
{
    const double dt = gps_sat_clock(e, t_sv);
    *t_sys = t_sv - dt;  /* the system's own time; Klobuchar only needs seconds of day */
    double c0, c1, p1[3];
    gps_sat_pos(e, *t_sys, s->pos, s->vel, &c0);
    gps_sat_pos(e, *t_sys + 1.0, p1, NULL, &c1);
    s->clk = c0;
    s->clk_rate = c1 - c0;
}

double pvt_sigma_pr(float cn0, float smooth_s)
{
    /*
     * A 0.2 m floor for what the models miss, plus the code's thermal noise: 0.6 m at 40 dB-Hz
     * on one 0.1 s epoch, rising faster below 32 dB-Hz as the squaring loss sets in. Carrier
     * smoothing averages it down; the code noise decorrelates in about 2 s. Set from the boost
     * runs' residuals on the pad (milestone 7), a little on the safe side: raw, 1.1 m at
     * 33 dB-Hz and 2.4 m at 31; after 10-40 s of smoothing, 0.3-0.7 m.
     */
    const double r = pow(10.0, (40.0 - (double)cn0) / 10.0);
    const double raw2 = 0.36 * r * (1.0 + pow(10.0, (32.0 - (double)cn0) / 10.0));
    const double s = smooth_s < 0.0f ? 0.0 : (smooth_s > 100.0f ? 100.0 : (double)smooth_s);
    return sqrt(0.04 + raw2 / (1.0 + s / 2.0));
}

double pvt_sigma_dop(float cn0, int locked, float pll_bw)
{
    /*
     * The phase-derived Doppler: 0.06 m/s at 43 dB-Hz behind a 10 Hz PLL, with the carrier
     * phase's noise. A wider loop is noisier by more than its phase jitter, (bw / 10)^0.75, since
     * its rate state feeds the observable's lag correction: measured on the pad, 20 Hz loops
     * 1.2-1.3 times the 10 Hz noise and 50 Hz loops 3.1-3.6 times. A channel pulling in reports
     * its FLL's frequency, five times worse.
     */
    const double s = 0.06 * pow(10.0, (43.0 - (double)cn0) / 20.0);
    const double bw = pll_bw > 0.0f ? (double)pll_bw : 10.0;
    return sqrt(0.0004 + s * s) * pow(bw / 10.0, 0.75) * (locked ? 1.0 : 5.0);
}

/* The chi-square line for dof degrees of freedom at false-alarm probability pfa: Wilson and
 * Hilferty's cube, with the normal quantile from Abramowitz and Stegun 26.2.23. */
static double chi2_line(int dof, double pfa)
{
    const double t = sqrt(-2.0 * log(pfa));
    const double z = t - (2.515517 + 0.802853 * t + 0.010328 * t * t) /
                             (1.0 + 1.432788 * t + 0.189269 * t * t + 0.001308 * t * t * t);
    const double k = (double)dof, a = 2.0 / (9.0 * k), c = 1.0 - a + z * sqrt(a);
    return k * c * c * c;
}

/*
 * Weighted least squares for position and clocks over the measurements marked ok, from x (in:
 * the seed, out: the solution; nx unknowns). Fills sol's per-measurement fields, nsat, iter and
 * resid_rms, and returns the weighted residual sum in *chi2; -1 when it cannot solve.
 */
static int solve_position(const pvt_meas_t *m, int n, const sat_t *sat, const double *t_gps, const int *ok,
                          const double *w, const gps_iono_t *iono, const pvt_opt_t *opt, double *x, int *nx_out,
                          double *chi2, pvt_sol_t *sol)
{
    /* Unknowns: x, y, z, the GPS clock, then one offset per other system present. */
    int col[GNSS_SYS_COUNT] = {3, 0, 0, 0};
    int nx = 4;
    for (int i = 0; i < n; i++) {
        if (ok[i] && m[i].sys != GNSS_SYS_GPS && col[m[i].sys] == 0) {
            col[m[i].sys] = nx++;
        }
    }
    const int ct = opt->coarse_time ? nx++ : -1;
    for (int k = 4; k < PVT_MAX_X; k++) {
        if (k >= nx) {
            x[k] = 0.0;
        }
    }
    double lat = 0.0, lon = 0.0, h = -WGS84_A, var_t = 0.0;
    int nused = 0;
    for (int it = 0; it < 12; it++) {
        double ata[PVT_MAX_X * PVT_MAX_X] = {0}, atb[PVT_MAX_X] = {0};
        int near_earth = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]) > 6.0e6;
        if (near_earth) {
            ecef_to_geo(x, &lat, &lon, &h);
        }
        nused = 0;
        double ss = 0.0, sw = 0.0;
        for (int i = 0; i < n; i++) {
            sol->used[i] = 0;
            if (!ok[i]) {
                continue;
            }
            /* Earth rotation during the flight time (Sagnac). */
            double d0[3] = {sat[i].pos[0] - x[0], sat[i].pos[1] - x[1], sat[i].pos[2] - x[2]};
            double tau = sqrt(d0[0] * d0[0] + d0[1] * d0[1] + d0[2] * d0[2]) / GNSS_C;
            double a = GPS_OMEGA_E * tau, ca = cos(a), sa = sin(a);
            double sp[3] = {ca * sat[i].pos[0] + sa * sat[i].pos[1], -sa * sat[i].pos[0] + ca * sat[i].pos[1],
                            sat[i].pos[2]};
            double d[3] = {sp[0] - x[0], sp[1] - x[1], sp[2] - x[2]};
            double rho = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            double corr = 0.0;
            if (near_earth) {
                az_el(lat, lon, d, &sol->az[i], &sol->el[i]);
                if (it > 2 && sol->el[i] < opt->el_mask) {
                    continue;
                }
                if (opt->use_iono && iono && iono->valid) {
                    corr += iono_klobuchar(iono, lat, lon, sol->az[i], sol->el[i], t_gps[i]);
                }
                if (opt->use_tropo) {
                    corr += tropo_saastamoinen(lat, h, sol->el[i]);
                }
            }
            const int sys = m[i].sys, cs = col[sys];
            sol->los[i][0] = d[0] / rho;
            sol->los[i][1] = d[1] / rho;
            sol->los[i][2] = d[2] / rho;
            double res = m[i].pr + GNSS_C * sat[i].clk - (rho + x[3] + (sys != GNSS_SYS_GPS ? x[cs] : 0.0) + corr);
            double hrow[PVT_MAX_X] = {-d[0] / rho, -d[1] / rho, -d[2] / rho, 1.0};
            if (sys != GNSS_SYS_GPS) {
                hrow[cs] = 1.0;
            }
            if (ct >= 0) {
                /* Satellites read at transmit times running x[ct] late sit where the satellite
                 * will be: the computed range runs over by the satellite's own range rate times
                 * that, so the model takes it back off. */
                hrow[ct] = -(sat[i].vel[0] * d[0] + sat[i].vel[1] * d[1] + sat[i].vel[2] * d[2]) / rho;
                res -= hrow[ct] * x[ct];
            }
            for (int r = 0; r < nx; r++) {
                atb[r] += w[i] * hrow[r] * res;
                for (int c = 0; c < nx; c++) {
                    ata[r * nx + c] += w[i] * hrow[r] * hrow[c];
                }
            }
            sol->resid[i] = res;
            sol->used[i] = 1;
            ss += res * res;
            sw += w[i] * res * res;
            nused++;
        }
        if (nused < nx || solve_spd(ata, atb, nx) != 0) {
            return -1;
        }
        for (int k = 0; k < nx; k++) {
            x[k] += atb[k];
        }
        var_t = ct >= 0 ? ata[ct * nx + ct] : 0.0;
        sol->iter = it + 1;
        sol->resid_rms = sqrt(ss / nused);
        *chi2 = sw;
        if (sqrt(atb[0] * atb[0] + atb[1] * atb[1] + atb[2] * atb[2]) < 1e-4 && it > 3) {
            break;
        }
    }
    sol->nsat = nused;
    for (int s = 1; s < GNSS_SYS_COUNT; s++) {
        sol->isb[s] = col[s] ? x[col[s]] : 0.0;
    }
    sol->time_offset = ct >= 0 ? x[ct] : 0.0;
    sol->time_sigma = sqrt(var_t);
    *nx_out = nx;
    return 0;
}

int pvt_solve(const pvt_meas_t *m, int n, const gps_eph_t (*eph)[GNSS_MAX_PRN + 1], const gps_iono_t *iono,
              const pvt_opt_t *opt, const double *pos0, pvt_sol_t *sol)
{
    memset(sol, 0, sizeof(*sol));
    if (n > PVT_MAX_SAT) {
        n = PVT_MAX_SAT;
    }
    sat_t sat[PVT_MAX_SAT];
    double t_gps[PVT_MAX_SAT], w[PVT_MAX_SAT];
    int ok[PVT_MAX_SAT], ok0[PVT_MAX_SAT];
    for (int i = 0; i < n; i++) {
        const int sys = m[i].sys;
        ok[i] = sys >= 0 && sys < GNSS_SYS_COUNT && m[i].prn >= 1 && m[i].prn <= GNSS_MAX_PRN;
        const gps_eph_t *e = ok[i] ? &eph[sys][m[i].prn] : NULL;
        ok0[i] = ok[i] = ok[i] && e->valid && e->health == 0;
        if (!ok[i]) {
            continue;
        }
        sat_state(e, m[i].t_sv, &sat[i], &t_gps[i]);
        const double sg = m[i].sigma > 0.0f ? (double)m[i].sigma : pvt_sigma_pr(m[i].cn0, 0.0f);
        w[i] = 1.0 / (sg * sg);
    }

    /* Position, then the residual test, leaving out the worst measurement while it fails. With
     * coarse time an offset over a millisecond puts the satellites where it says and solves
     * again: the range-rate partial alone leaves a few cm, and the satellites' velocities up to
     * 0.3 m/s out, per 0.6 s. */
    double x[PVT_MAX_X] = {0.0};
    if (pos0) {
        x[0] = pos0[0];
        x[1] = pos0[1];
        x[2] = pos0[2];
    }
    int nx = 4;
    double t_shift = 0.0;
    for (int pass = 0;; pass++) {
        for (;;) {
            double chi2 = 0.0;
            if (solve_position(m, n, sat, t_gps, ok, w, iono, opt, x, &nx, &chi2, sol) != 0) {
                return -1;
            }
            const int dof = sol->nsat - nx;
            sol->dof = dof;
            sol->chi2 = chi2;
            sol->chi2_lim = dof > 0 ? chi2_line(dof, opt->raim_pfa) : 0.0;
            if (!opt->raim || dof < 1 || chi2 <= sol->chi2_lim) {
                break;
            }
            if (dof < 2 || sol->nexcl >= opt->raim_max_excl) {
                return -2;  /* inconsistent, and no single measurement to blame */
            }
            int worst = -1;
            double zw = 0.0;
            for (int i = 0; i < n; i++) {
                const double z = sol->used[i] ? fabs(sol->resid[i]) * sqrt(w[i]) : 0.0;
                if (z > zw) {
                    zw = z;
                    worst = i;
                }
            }
            ok[worst] = 0;
            sol->excluded[worst] |= 1;
            sol->nexcl++;
        }
        if (!opt->coarse_time || pass > 0 || fabs(sol->time_offset) < 1e-3) {
            sol->time_offset += t_shift;
            break;
        }
        t_shift = sol->time_offset;
        for (int i = 0; i < n; i++) {
            if (ok0[i]) {
                sat_state(&eph[m[i].sys][m[i].prn], m[i].t_sv - t_shift, &sat[i], &t_gps[i]);
            }
            ok[i] = ok0[i];
        }
        memset(sol->excluded, 0, sizeof(sol->excluded));
        sol->nexcl = 0;
        x[nx - 1] = 0.0;  /* the offset's column is the last */
    }
    double lat, lon, h;
    ecef_to_geo(x, &lat, &lon, &h);
    sol->pos[0] = x[0];
    sol->pos[1] = x[1];
    sol->pos[2] = x[2];
    sol->clk_bias = x[3];
    sol->lat = lat;
    sol->lon = lon;
    sol->h = h;

    /* DOP in the local frame, from the geometry alone. */
    {
        double g[PVT_MAX_X * PVT_MAX_X] = {0}, gb[PVT_MAX_X] = {0};
        int col[GNSS_SYS_COUNT] = {3, 0, 0, 0}, ng = 4;
        for (int i = 0; i < n; i++) {
            if (sol->used[i] && m[i].sys != GNSS_SYS_GPS && col[m[i].sys] == 0) {
                col[m[i].sys] = ng++;
            }
        }
        const int ct = opt->coarse_time ? ng++ : -1;  /* the offset widens the DOP it costs */
        for (int i = 0; i < n; i++) {
            if (!sol->used[i]) {
                continue;
            }
            double hrow[PVT_MAX_X] = {-sol->los[i][0], -sol->los[i][1], -sol->los[i][2], 1.0};
            if (m[i].sys != GNSS_SYS_GPS) {
                hrow[col[m[i].sys]] = 1.0;
            }
            if (ct >= 0) {
                hrow[ct] = -(sat[i].vel[0] * sol->los[i][0] + sat[i].vel[1] * sol->los[i][1] +
                             sat[i].vel[2] * sol->los[i][2]) / 1000.0;  /* km/s: scale leaves the rest alone */
            }
            for (int r = 0; r < ng; r++) {
                for (int c = 0; c < ng; c++) {
                    g[r * ng + c] += hrow[r] * hrow[c];
                }
            }
        }
        if (solve_spd(g, gb, ng) == 0) {
            double sl = sin(lat), cl = cos(lat), so = sin(lon), co = cos(lon);
            double rot[3][3] = {{-so, co, 0.0}, {-sl * co, -sl * so, cl}, {cl * co, cl * so, sl}};
            double qe[3] = {0};
            for (int r = 0; r < 3; r++) {
                for (int a = 0; a < 3; a++) {
                    for (int b = 0; b < 3; b++) {
                        qe[r] += rot[r][a] * g[a * ng + b] * rot[r][b];
                    }
                }
            }
            sol->hdop = sqrt(qe[0] + qe[1]);
            sol->vdop = sqrt(qe[2]);
            sol->pdop = sqrt(qe[0] + qe[1] + qe[2]);
        }
    }

    /* Velocity and clock drift from Doppler (range rate = -lambda * D), weighted and tested the
     * same way; a failure leaves the fix standing and vel_valid at 0. */
    const double lambda = GNSS_C / 1575.42e6;
    const double up[3] = {cos(lat) * cos(lon), cos(lat) * sin(lon), sin(lat)};
    double hv[PVT_MAX_SAT][4], rv[PVT_MAX_SAT], wv[PVT_MAX_SAT];
    int vok[PVT_MAX_SAT];
    for (int i = 0; i < n; i++) {
        vok[i] = sol->used[i];
        if (!vok[i]) {
            continue;
        }
        double d[3] = {sat[i].pos[0] - x[0], sat[i].pos[1] - x[1], sat[i].pos[2] - x[2]};
        double rho = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        double u[3] = {d[0] / rho, d[1] / rho, d[2] / rho};
        double rate = -lambda * m[i].dop + GNSS_C * sat[i].clk_rate;
        double vs = sat[i].vel[0] * u[0] + sat[i].vel[1] * u[1] + sat[i].vel[2] * u[2];
        rv[i] = rate - vs;  /* = -v_rx . u + dT/dh v_up + drift */
        hv[i][0] = -u[0];
        hv[i][1] = -u[1];
        hv[i][2] = -u[2];
        hv[i][3] = 1.0;
        if (opt->use_tropo && sol->el[i] > 0.0) {
            /* The troposphere thins as the receiver climbs: at 1 km/s through 5 km its delay falls
             * by ~0.15 m/s at the zenith, over 0.4 m/s at 20 deg. */
            const double g = 0.5 * (tropo_saastamoinen(lat, h + 1.0, sol->el[i]) -
                                    tropo_saastamoinen(lat, h - 1.0, sol->el[i]));
            hv[i][0] += g * up[0];
            hv[i][1] += g * up[1];
            hv[i][2] += g * up[2];
        }
        const double sd = m[i].sigma_dop > 0.0f ? (double)m[i].sigma_dop : pvt_sigma_dop(m[i].cn0, 1, 10.0f);
        wv[i] = 1.0 / (sd * sd);
    }
    for (int nexcl_v = 0;;) {
        double ata[16] = {0}, atb[4] = {0};
        int nv = 0;
        for (int i = 0; i < n; i++) {
            if (!vok[i]) {
                continue;
            }
            for (int r = 0; r < 4; r++) {
                atb[r] += wv[i] * hv[i][r] * rv[i];
                for (int c = 0; c < 4; c++) {
                    ata[r * 4 + c] += wv[i] * hv[i][r] * hv[i][c];
                }
            }
            nv++;
        }
        if (nv < 4 || solve_spd(ata, atb, 4) != 0) {
            break;
        }
        sol->vel[0] = atb[0];
        sol->vel[1] = atb[1];
        sol->vel[2] = atb[2];
        sol->clk_drift = atb[3];
        double chi2 = 0.0, zw = 0.0;
        int worst = -1;
        for (int i = 0; i < n; i++) {
            if (!vok[i]) {
                continue;
            }
            const double e = rv[i] - (hv[i][0] * atb[0] + hv[i][1] * atb[1] + hv[i][2] * atb[2] + atb[3]);
            const double z = fabs(e) * sqrt(wv[i]);
            chi2 += z * z;
            if (z > zw) {
                zw = z;
                worst = i;
            }
        }
        const int dof = nv - 4;
        sol->vdof = dof;
        if (!opt->raim || dof < 1 || chi2 <= chi2_line(dof, opt->raim_pfa)) {
            sol->vel_valid = 1;
            break;
        }
        if (dof < 2 || nexcl_v >= opt->raim_max_excl) {
            break;
        }
        vok[worst] = 0;
        sol->excluded[worst] |= 2;
        nexcl_v++;
    }
    sol->valid = 1;
    return 0;
}
