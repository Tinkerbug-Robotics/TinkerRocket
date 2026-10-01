#include "gnss/eph.h"
#include "gnss/types.h"

#include <math.h>

double gps_time_diff(double t1, double t0)
{
    double d = t1 - t0;
    if (d > 302400.0) {
        d -= 604800.0;
    } else if (d < -302400.0) {
        d += 604800.0;
    }
    return d;
}

/* Each system's constants (the GPS ones for GPS, so its results are unchanged). */
static double mu_of(const gps_eph_t *e)
{
    return e->sys == GNSS_SYS_GAL ? GAL_MU : (e->sys == GNSS_SYS_BDS ? BDS_MU : GPS_MU);
}

static double omega_of(const gps_eph_t *e)
{
    return e->sys == GNSS_SYS_BDS ? BDS_OMEGA_E : GPS_OMEGA_E;
}

static double frel_of(const gps_eph_t *e)
{
    return e->sys == GNSS_SYS_GPS ? GPS_F_REL : GAL_F_REL;
}

/* Eccentric anomaly at t. */
static double ecc_anomaly(const gps_eph_t *e, double t)
{
    double a = e->sqrt_a * e->sqrt_a;
    double n = sqrt(mu_of(e) / (a * a * a)) + e->delta_n;
    double m = e->m0 + n * gps_time_diff(t, e->toe);
    double ek = m;
    for (int i = 0; i < 30; i++) {
        double next = m + e->e * sin(ek);
        if (fabs(next - ek) < 1e-14) {
            return next;
        }
        ek = next;
    }
    return ek;
}

static void position(const gps_eph_t *e, double t, double pos[3])
{
    double a = e->sqrt_a * e->sqrt_a;
    double tk = gps_time_diff(t, e->toe);
    double ek = ecc_anomaly(e, t);
    double nu = atan2(sqrt(1.0 - e->e * e->e) * sin(ek), cos(ek) - e->e);
    double phi = nu + e->omega;
    double s2 = sin(2.0 * phi), c2 = cos(2.0 * phi);
    double u = phi + e->cus * s2 + e->cuc * c2;
    double r = a * (1.0 - e->e * cos(ek)) + e->crs * s2 + e->crc * c2;
    double i = e->i0 + e->cis * s2 + e->cic * c2 + e->idot * tk;
    double xp = r * cos(u), yp = r * sin(u);
    const double we = omega_of(e);
    double om = e->omega0 + (e->omega_dot - we) * tk - we * e->toe;
    double co = cos(om), so = sin(om), ci = cos(i), si = sin(i);
    pos[0] = xp * co - yp * ci * so;
    pos[1] = xp * so + yp * ci * co;
    pos[2] = yp * si;
}

static double clock_at(const gps_eph_t *e, double t)
{
    double tc = gps_time_diff(t, e->toc);
    double rel = frel_of(e) * e->e * e->sqrt_a * sin(ecc_anomaly(e, t));
    return e->af0 + e->af1 * tc + e->af2 * tc * tc + rel - e->tgd;
}

void gps_sat_pos(const gps_eph_t *e, double t, double pos[3], double vel[3], double *clk)
{
    position(e, t, pos);
    if (vel) {
        double p0[3], p1[3];
        position(e, t - 0.5, p0);
        position(e, t + 0.5, p1);
        for (int k = 0; k < 3; k++) {
            vel[k] = p1[k] - p0[k];
        }
    }
    if (clk) {
        *clk = clock_at(e, t);
    }
}

double gps_sat_clock(const gps_eph_t *e, double t_sv)
{
    double dt = 0.0;
    for (int i = 0; i < 3; i++) {
        dt = clock_at(e, t_sv - dt);
    }
    return dt;
}
