/* From the GNSS receiver (gnss repo receiver/core/nav/eph.c at 7511d11), with GLONASS added here (eph.h). */
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

/* GLONASS (eph.h): the equations of motion in PZ-90.11's rotating frame, the Earth's central term
 * and J2 and the broadcast lunisolar acceleration (held for the interval), ICD 5.1 appendix A.3.1.2 */
static void glo_deriv(const double x[6], const double acc[3], double xd[6])
{
    const double r2 = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];
    if (r2 <= 0.0) {
        for (int k = 0; k < 6; k++) {
            xd[k] = 0.0;
        }
        return;
    }
    const double r3 = r2 * sqrt(r2);
    const double w2 = GLO_OMEGA_E * GLO_OMEGA_E;
    const double a = 1.5 * GLO_J20 * GLO_MU * GLO_AE * GLO_AE / (r2 * r3);
    const double b = 5.0 * x[2] * x[2] / r2;
    const double c = -GLO_MU / r3 - a * (1.0 - b);
    xd[0] = x[3];
    xd[1] = x[4];
    xd[2] = x[5];
    xd[3] = (c + w2) * x[0] + 2.0 * GLO_OMEGA_E * x[4] + acc[0];
    xd[4] = (c + w2) * x[1] - 2.0 * GLO_OMEGA_E * x[3] + acc[1];
    xd[5] = (c - 2.0 * a) * x[2] + acc[2];
}

/* The state at t (GPS s of week): tb's carried there in GLO_STEP_S steps, the last one short */
static void glo_state(const gps_eph_t *e, double t, double x[6])
{
    for (int k = 0; k < 3; k++) {
        x[k] = e->glo_pos[k];
        x[3 + k] = e->glo_vel[k];
    }
    double left = gps_time_diff(t, e->toe);
    while (fabs(left) > 1e-9) {
        const double h = fabs(left) > GLO_STEP_S ? (left > 0.0 ? GLO_STEP_S : -GLO_STEP_S) : left;
        double k1[6], k2[6], k3[6], k4[6], y[6];
        glo_deriv(x, e->glo_acc, k1);
        for (int k = 0; k < 6; k++) {
            y[k] = x[k] + 0.5 * h * k1[k];
        }
        glo_deriv(y, e->glo_acc, k2);
        for (int k = 0; k < 6; k++) {
            y[k] = x[k] + 0.5 * h * k2[k];
        }
        glo_deriv(y, e->glo_acc, k3);
        for (int k = 0; k < 6; k++) {
            y[k] = x[k] + h * k3[k];
        }
        glo_deriv(y, e->glo_acc, k4);
        for (int k = 0; k < 6; k++) {
            x[k] += h / 6.0 * (k1[k] + 2.0 * k2[k] + 2.0 * k3[k] + k4[k]);
        }
        left -= h;
    }
}

static double clock_at(const gps_eph_t *e, double t)
{
    if (e->sys == GNSS_SYS_GLO) {
        return e->af0 + e->af1 * gps_time_diff(t, e->toc);   /* -tau_n + gamma_n (t - tb): relativity in it */
    }
    double tc = gps_time_diff(t, e->toc);
    double rel = frel_of(e) * e->e * e->sqrt_a * sin(ecc_anomaly(e, t));
    return e->af0 + e->af1 * tc + e->af2 * tc * tc + rel - e->tgd;
}

void gps_sat_pos(const gps_eph_t *e, double t, double pos[3], double vel[3], double *clk)
{
    if (e->sys == GNSS_SYS_GLO) {
        double x[6];
        glo_state(e, t, x);
        for (int k = 0; k < 3; k++) {
            pos[k] = x[k];
            if (vel) {
                vel[k] = x[3 + k];
            }
        }
        if (clk) {
            *clk = clock_at(e, t);
        }
        return;
    }
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
