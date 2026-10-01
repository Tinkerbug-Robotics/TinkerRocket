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
    if (h < -100.0 || h > 1e4 || el <= 0.0) {
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

int pvt_solve(const pvt_meas_t *m, int n, const gps_eph_t (*eph)[GNSS_MAX_PRN + 1], const gps_iono_t *iono,
              const pvt_opt_t *opt, const double *pos0, pvt_sol_t *sol)
{
    memset(sol, 0, sizeof(*sol));
    if (n > PVT_MAX_SAT) {
        n = PVT_MAX_SAT;
    }
    sat_t sat[PVT_MAX_SAT];
    double t_gps[PVT_MAX_SAT];
    int ok[PVT_MAX_SAT];
    /* Unknowns: x, y, z, the GPS clock, then one offset per other system present. */
    int col[GNSS_SYS_COUNT] = {3, 0, 0, 0};
    int nx = 4;
    for (int i = 0; i < n; i++) {
        const int sys = m[i].sys;
        ok[i] = sys >= 0 && sys < GNSS_SYS_COUNT && m[i].prn >= 1 && m[i].prn <= GNSS_MAX_PRN;
        const gps_eph_t *e = ok[i] ? &eph[sys][m[i].prn] : NULL;
        ok[i] = ok[i] && e->valid && e->health == 0;
        if (!ok[i]) {
            continue;
        }
        if (sys != GNSS_SYS_GPS && col[sys] == 0) {
            col[sys] = nx++;
        }
        double dt = gps_sat_clock(e, m[i].t_sv);
        t_gps[i] = m[i].t_sv - dt;  /* the system's own time; Klobuchar only needs seconds of day */
        double c0, c1, p1[3];
        gps_sat_pos(e, t_gps[i], sat[i].pos, sat[i].vel, &c0);
        gps_sat_pos(e, t_gps[i] + 1.0, p1, NULL, &c1);
        sat[i].clk = c0;
        sat[i].clk_rate = c1 - c0;
    }

    double x[PVT_MAX_X] = {0.0};
    if (pos0) {
        x[0] = pos0[0];
        x[1] = pos0[1];
        x[2] = pos0[2];
    }
    double lat = 0.0, lon = 0.0, h = -WGS84_A, q[PVT_MAX_X * PVT_MAX_X];
    int nused = 0;
    for (int it = 0; it < 12; it++) {
        double ata[PVT_MAX_X * PVT_MAX_X] = {0}, atb[PVT_MAX_X] = {0};
        int near_earth = sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]) > 6.0e6;
        if (near_earth) {
            ecef_to_geo(x, &lat, &lon, &h);
        }
        nused = 0;
        double ss = 0.0;
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
            double res = m[i].pr + GNSS_C * sat[i].clk - (rho + x[3] + (sys != GNSS_SYS_GPS ? x[cs] : 0.0) + corr);
            double hrow[PVT_MAX_X] = {-d[0] / rho, -d[1] / rho, -d[2] / rho, 1.0};
            if (sys != GNSS_SYS_GPS) {
                hrow[cs] = 1.0;
            }
            for (int r = 0; r < nx; r++) {
                atb[r] += hrow[r] * res;
                for (int c = 0; c < nx; c++) {
                    ata[r * nx + c] += hrow[r] * hrow[c];
                }
            }
            sol->resid[i] = res;
            sol->used[i] = 1;
            ss += res * res;
            nused++;
        }
        if (nused < nx || solve_spd(ata, atb, nx) != 0) {
            return -1;
        }
        for (int k = 0; k < nx; k++) {
            x[k] += atb[k];
        }
        memcpy(q, ata, sizeof(double) * (size_t)(nx * nx));
        sol->iter = it + 1;
        sol->resid_rms = sqrt(ss / nused);
        if (sqrt(atb[0] * atb[0] + atb[1] * atb[1] + atb[2] * atb[2]) < 1e-4 && it > 3) {
            break;
        }
    }
    ecef_to_geo(x, &lat, &lon, &h);
    sol->pos[0] = x[0];
    sol->pos[1] = x[1];
    sol->pos[2] = x[2];
    sol->clk_bias = x[3];
    for (int s = 1; s < GNSS_SYS_COUNT; s++) {
        sol->isb[s] = col[s] ? x[col[s]] : 0.0;
    }
    sol->lat = lat;
    sol->lon = lon;
    sol->h = h;
    sol->nsat = nused;

    /* DOP in the local frame. */
    double sl = sin(lat), cl = cos(lat), so = sin(lon), co = cos(lon);
    double rot[3][3] = {{-so, co, 0.0}, {-sl * co, -sl * so, cl}, {cl * co, cl * so, sl}};
    double qe[3] = {0};
    for (int r = 0; r < 3; r++) {
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
                qe[r] += rot[r][a] * q[a * nx + b] * rot[r][b];
            }
        }
    }
    sol->hdop = sqrt(qe[0] + qe[1]);
    sol->vdop = sqrt(qe[2]);
    sol->pdop = sqrt(qe[0] + qe[1] + qe[2]);

    /* Velocity and clock drift from Doppler: range rate = -lambda * D. */
    const double lambda = GNSS_C / 1575.42e6;
    double ata[16] = {0}, atb[4] = {0};
    int nv = 0;
    for (int i = 0; i < n; i++) {
        if (!sol->used[i]) {
            continue;
        }
        double d[3] = {sat[i].pos[0] - x[0], sat[i].pos[1] - x[1], sat[i].pos[2] - x[2]};
        double rho = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        double u[3] = {d[0] / rho, d[1] / rho, d[2] / rho};
        double rate = -lambda * m[i].dop + GNSS_C * sat[i].clk_rate;
        double vs = sat[i].vel[0] * u[0] + sat[i].vel[1] * u[1] + sat[i].vel[2] * u[2];
        double res = rate - vs;  /* = -v_rx . u + drift */
        double hrow[4] = {-u[0], -u[1], -u[2], 1.0};
        for (int r = 0; r < 4; r++) {
            atb[r] += hrow[r] * res;
            for (int c = 0; c < 4; c++) {
                ata[r * 4 + c] += hrow[r] * hrow[c];
            }
        }
        nv++;
    }
    if (nv >= 4 && solve_spd(ata, atb, 4) == 0) {
        sol->vel[0] = atb[0];
        sol->vel[1] = atb[1];
        sol->vel[2] = atb[2];
        sol->clk_drift = atb[3];
    }
    sol->valid = 1;
    return 0;
}
