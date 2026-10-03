/*
 * trksim: one tracking channel (core/trk) against a satellite's true Doppler, at
 * the level of correlator dumps. It is fast enough to sweep loop designs and C/N0
 * in seconds, where gnssrx needs real time on IQ files.
 *
 * The dumps come from the analytic correlation of the true carrier and code with
 * the channel's own NCOs, under the correlator contract's command delay (period s
 * steers period s + 3). They carry random data bits and complex Gaussian noise at a
 * set C/N0, with early, prompt and late noise correlated as the code's triangle
 * makes it. The 2-bit front end and the band limit are not modelled; gnssrx covers
 * those.
 *
 *   trksim TRUTH.csv [options]
 *
 * TRUTH.csv holds t_s,dop_hz rows: the line-of-sight Doppler at L1, e.g. at 10 Hz
 * from py/los_truth.py. Between rows the Doppler is linear, as the rig's smoothed
 * gps-sdr-sim files make it. Its el_deg and az_deg columns, when there, place the
 * satellite for --spin.
 *
 * Spin (milestone 7): the vehicle rolls about local up (these flights go straight up) at
 * --spin's rate, and its patch antenna turns with it, in the nose looking up the roll axis
 * or on the side looking out. The patch is two crossed dipoles fed 90 degrees apart, the
 * second rho times the first (rho from the axial ratio, which worsens off the axis). The
 * right-hand circular wave's voltage on them gives the carrier its phase (the antenna's
 * wind-up: a cycle a revolution) and its amplitude (the pattern, and the ripple an imperfect
 * patch adds). Behind a side patch the body blocks it: the floor.
 */
#define _POSIX_C_SOURCE 200809L

#include "gnss/trk.h"
#include "loops_arg.h"
#include "rng.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FS        6.75e6
#define T_PER     1e-3
#define CHIP_HZ   1.023e6
#define SUB       10            /* phase samples per period for the correlation */
#define MAX_DELAY 8
#define MAX_STEPS 16
#define TWO_PI    6.283185307179586476925286766559

typedef struct {
    double *t, *f, *el, *az;  /* el and az in rad, 0 when the file has none */
    size_t n;
} truth_t;

static int read_truth(const char *path, truth_t *tr)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        return -1;
    }
    size_t cap = 1024;
    tr->t = malloc(sizeof(double) * cap);
    tr->f = malloc(sizeof(double) * cap);
    tr->el = malloc(sizeof(double) * cap);
    tr->az = malloc(sizeof(double) * cap);
    tr->n = 0;
    char line[512];
    while (fgets(line, sizeof(line), fp)) {
        double a, b, r = 0.0, el = 0.0, az = 0.0;
        const int got = sscanf(line, "%lf,%lf,%lf,%lf,%lf", &a, &b, &r, &el, &az);
        if (got < 2) {
            continue;  /* header */
        }
        if (tr->n == cap) {
            cap *= 2;
            tr->t = realloc(tr->t, sizeof(double) * cap);
            tr->f = realloc(tr->f, sizeof(double) * cap);
            tr->el = realloc(tr->el, sizeof(double) * cap);
            tr->az = realloc(tr->az, sizeof(double) * cap);
        }
        tr->t[tr->n] = a;
        tr->f[tr->n] = b;
        tr->el[tr->n] = got >= 4 ? el * TWO_PI / 360.0 : 0.0;
        tr->az[tr->n] = got >= 5 ? az * TWO_PI / 360.0 : 0.0;
        tr->n++;
    }
    fclose(fp);
    return tr->n >= 2 ? 0 : -1;
}

/* Doppler at t, linear between rows (held beyond the ends). */
static double dop_at(const truth_t *tr, double t, size_t *hint)
{
    size_t k = *hint;
    while (k + 1 < tr->n - 1 && tr->t[k + 1] <= t) {
        k++;
    }
    while (k > 0 && tr->t[k] > t) {
        k--;
    }
    *hint = k;
    if (t <= tr->t[0]) {
        return tr->f[0];
    }
    if (t >= tr->t[tr->n - 1]) {
        return tr->f[tr->n - 1];
    }
    double u = (t - tr->t[k]) / (tr->t[k + 1] - tr->t[k]);
    return tr->f[k] + u * (tr->f[k + 1] - tr->f[k]);
}

/* Elevation and azimuth (rad) at t, linear between rows (azimuth across north the short way). */
static void geo_at(const truth_t *tr, double t, size_t hint, double *el, double *az)
{
    size_t k = hint;
    if (t <= tr->t[0] || tr->n < 2) {
        *el = tr->el[0];
        *az = tr->az[0];
        return;
    }
    if (t >= tr->t[tr->n - 1]) {
        *el = tr->el[tr->n - 1];
        *az = tr->az[tr->n - 1];
        return;
    }
    while (k + 1 < tr->n - 1 && tr->t[k + 1] <= t) {
        k++;
    }
    while (k > 0 && tr->t[k] > t) {
        k--;
    }
    const double u = (t - tr->t[k]) / (tr->t[k + 1] - tr->t[k]);
    double da = tr->az[k + 1] - tr->az[k];
    da -= TWO_PI * floor(da / TWO_PI + 0.5);
    *el = tr->el[k] + u * (tr->el[k + 1] - tr->el[k]);
    *az = tr->az[k] + u * da;
}

static double tri(double x)
{
    x = fabs(x);
    return x < 1.0 ? 1.0 - x : 0.0;
}

/* The vehicle's roll and its antenna. */
typedef struct {
    int on;
    int side;               /* 0: in the nose, looking up the roll axis; 1: on the side, looking out */
    double rate_hz;         /* roll rate reached */
    double t0, t1;          /* the rate ramps from 0 at t0 to rate_hz at t1 */
    double floor_db;        /* the least gain (behind a side patch: the body) */
    double ar0_db, ar90_db; /* axial ratio on the boresight and 90 deg off it (linear between) */
} spin_t;

/* Roll angle (rev) at t: the integral of the ramped rate. */
static double spin_angle(const spin_t *sp, double t)
{
    if (t <= sp->t0) {
        return 0.0;
    }
    const double ramp = sp->t1 > sp->t0 ? sp->t1 - sp->t0 : 0.0;
    if (t < sp->t1) {
        const double u = t - sp->t0;
        return 0.5 * sp->rate_hz * u * u / ramp;
    }
    return 0.5 * sp->rate_hz * ramp + sp->rate_hz * (t - sp->t1);
}

static void cross3(const double a[3], const double b[3], double c[3])
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

static double dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/*
 * The antenna toward a satellite at el, az (rad; ENU frame, roll psi rev): the phase of the right-
 * hand wave's voltage on it (rev, wrapped) and its power gain (linear; 1 on the boresight).
 */
static void spin_antenna(const spin_t *sp, double psi, double el, double az, double *wind, double *gain)
{
    const double c = cos(TWO_PI * psi), s = sin(TWO_PI * psi);
    const double up[3] = {0.0, 0.0, 1.0};
    const double yb[3] = {c, s, 0.0}, zb[3] = {-s, c, 0.0};  /* body axes turning about up */
    const double *bore = sp->side ? yb : up;
    const double *xa = sp->side ? zb : yb, *ya = sp->side ? up : zb;  /* xa x ya = boresight */
    const double k[3] = {cos(el) * sin(az), cos(el) * cos(az), sin(el)};  /* toward the satellite */
    const double ks[3] = {-k[0], -k[1], -k[2]};                            /* the signal's direction */
    /* The wave's basis across its direction: north projected, then ks x p. Its field is p - j q. */
    const double n[3] = {0.0, 1.0, 0.0};
    const double kn = dot3(ks, n);
    double pv[3] = {n[0] - ks[0] * kn, n[1] - ks[1] * kn, n[2] - ks[2] * kn}, qv[3];
    const double pn = sqrt(dot3(pv, pv));
    for (int j = 0; j < 3; j++) {
        pv[j] /= pn;
    }
    cross3(ks, pv, qv);
    const double co = dot3(bore, k), off = acos(co < -1.0 ? -1.0 : (co > 1.0 ? 1.0 : co));
    const double ar = sp->ar0_db + (sp->ar90_db - sp->ar0_db) * fmin(off / (0.25 * TWO_PI), 1.0);
    const double rho = pow(10.0, -ar / 20.0), rho0 = pow(10.0, -sp->ar0_db / 20.0);
    /* V = E.xa - j rho E.ya with E = p - j q. */
    const double vr = dot3(pv, xa) - rho * dot3(qv, ya), vi = -dot3(qv, xa) - rho * dot3(pv, ya);
    *wind = atan2(vi, vr) / TWO_PI;
    double g_db = 10.0 * log10(fmax((vr * vr + vi * vi) / ((1.0 + rho0) * (1.0 + rho0)), 1e-12));
    if (g_db < sp->floor_db || (sp->side && co < 0.0)) {
        g_db = sp->floor_db;  /* (behind a side patch, the body) */
    }
    *gain = pow(10.0, g_db / 10.0);
}

/* The wind-up (rev, wrapped) and gain at time t for the satellite in tr. */
static void spin_at(const spin_t *sp, const truth_t *tr, size_t hint, double t, double *wind, double *gain)
{
    double el, az;
    geo_at(tr, t, hint, &el, &az);
    spin_antenna(sp, spin_angle(sp, t), el, az, wind, gain);
}

static double wrap_half(double x)
{
    return x - floor(x + 0.5);
}

static void usage(void)
{
    fprintf(stderr,
            "usage: trksim TRUTH.csv [options]\n"
            "  --from S --to S            window, truth-file seconds (default: all)\n"
            "  --score-from S             metrics from here (default: --from + 10)\n"
            "  --cn0 DBHZ                 C/N0 (default 45)\n"
            "  --cn0-at S:DBHZ            C/N0 from time S on (repeatable)\n"
            "  --quiet PF,PP,PD/LF,LP,LD[:MS]  loop bandwidths, Hz: pull-in FLL,PLL,DLL / locked; FLL block, ms\n"
            "                             (default: trk's quiet profile)\n"
            "  --boost PF,PP,PD/LF,LP,LD[:MS]  the boost profile over --boost-at (default: trk's)\n"
            "  --boost-at S0,S1           (default: never)\n"
            "  --delay N                  periods from a dump to the period its command steers (default 3)\n"
            "  --dop-err HZ               the acquisition's Doppler error at the start (default 30)\n"
            "  --seed N                   noise and data (default 1)\n"
            "  --split T1,T2              also report the window's parts before T1, T1-T2 and after T2\n"
            "  --aid LAG_MS,SF,BIAS       feed the loop the true Doppler rate as an IMU would: LAG_MS late,\n"
            "                             scaled by 1 + SF, plus BIAS Hz/s (51.5 Hz/s is 1 g)\n"
            "  --spin HZ[,T0,T1]          the vehicle rolls about up, the rate ramping from 0 at T0 to HZ at T1\n"
            "                             (default: HZ throughout)\n"
            "  --antenna nose|side        the patch in the nose looking up the roll axis (default), or on the\n"
            "                             side looking out\n"
            "  --floor DB                 the least gain (default -25); behind a side patch, the body\n"
            "  --ar AR0_DB,AR90_DB        the patch's axial ratio on its boresight and 90 deg off it (default 0,0)\n"
            "  --aid-spin LAG_MS          feed the loop the antenna's wind-up rate as the gyro and attitude would\n"
            "                             predict it, LAG_MS late\n"
            "  --csv PATH                 a row every 10 ms\n");
}

int main(int argc, char **argv)
{
    const char *truth_path = NULL, *csv_path = NULL;
    double t_from = NAN, t_to = NAN, score_from = NAN, boost0 = INFINITY, boost1 = -INFINITY, dop_err = 30.0;
    double split1 = INFINITY, split2 = INFINITY;
    double aid_lag = -1.0, aid_sf = 0.0, aid_bias = 0.0, aid_spin_lag = -1.0;
    spin_t sp;
    memset(&sp, 0, sizeof(sp));
    sp.t0 = sp.t1 = -INFINITY;
    sp.floor_db = -25.0;
    double cn0_base = 45.0, step_t[MAX_STEPS], step_cn0[MAX_STEPS];
    int nsteps = 0, delay = 3;
    uint64_t seed = 1;
    trk_profile_t quiet = trk_profile_quiet, boost = trk_profile_boost;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
        if (a[0] != '-') {
            truth_path = a;
            continue;
        }
        if (!v) {
            usage();
            return 2;
        }
        i++;
        if (!strcmp(a, "--from")) {
            t_from = atof(v);
        } else if (!strcmp(a, "--to")) {
            t_to = atof(v);
        } else if (!strcmp(a, "--score-from")) {
            score_from = atof(v);
        } else if (!strcmp(a, "--cn0")) {
            cn0_base = atof(v);
        } else if (!strcmp(a, "--cn0-at") && nsteps < MAX_STEPS) {
            if (sscanf(v, "%lf:%lf", &step_t[nsteps], &step_cn0[nsteps]) != 2) {
                usage();
                return 2;
            }
            nsteps++;
        } else if (!strcmp(a, "--quiet")) {
            if (loops_arg_parse(v, &quiet)) {
                usage();
                return 2;
            }
        } else if (!strcmp(a, "--boost")) {
            if (loops_arg_parse(v, &boost)) {
                usage();
                return 2;
            }
        } else if (!strcmp(a, "--boost-at")) {
            if (sscanf(v, "%lf,%lf", &boost0, &boost1) != 2) {
                usage();
                return 2;
            }
        } else if (!strcmp(a, "--delay")) {
            delay = atoi(v);
        } else if (!strcmp(a, "--dop-err")) {
            dop_err = atof(v);
        } else if (!strcmp(a, "--seed")) {
            seed = strtoull(v, NULL, 10);
        } else if (!strcmp(a, "--aid")) {
            if (sscanf(v, "%lf,%lf,%lf", &aid_lag, &aid_sf, &aid_bias) != 3) {
                usage();
                return 2;
            }
        } else if (!strcmp(a, "--spin")) {
            const int got = sscanf(v, "%lf,%lf,%lf", &sp.rate_hz, &sp.t0, &sp.t1);
            if (got != 1 && got != 3) {
                usage();
                return 2;
            }
            sp.on = 1;
        } else if (!strcmp(a, "--antenna")) {
            sp.side = !strcmp(v, "side");
        } else if (!strcmp(a, "--floor")) {
            sp.floor_db = atof(v);
        } else if (!strcmp(a, "--ar")) {
            if (sscanf(v, "%lf,%lf", &sp.ar0_db, &sp.ar90_db) != 2) {
                usage();
                return 2;
            }
        } else if (!strcmp(a, "--aid-spin")) {
            aid_spin_lag = atof(v);
        } else if (!strcmp(a, "--split")) {
            if (sscanf(v, "%lf,%lf", &split1, &split2) != 2) {
                usage();
                return 2;
            }
        } else if (!strcmp(a, "--csv")) {
            csv_path = v;
        } else {
            usage();
            return 2;
        }
    }
    truth_t tr;
    if (!truth_path || read_truth(truth_path, &tr) != 0 || delay < 1 || delay > MAX_DELAY) {
        usage();
        return 2;
    }
    if (isnan(t_from)) {
        t_from = tr.t[0];
    }
    if (isnan(t_to)) {
        t_to = tr.t[tr.n - 1];
    }
    if (isnan(score_from)) {
        score_from = t_from + 10.0;
    }
    FILE *csv = csv_path ? fopen(csv_path, "w") : NULL;
    if (csv) {
        fprintf(csv, "t,dop_true,dop_nco,ferr_hz,phase_err_cyc,code_err_chips,state,pll_lock,cn0,boost\n");
    }

    /* Words as the receiver forms them at 6.75 MS/s (the IF cancels out; any will do). */
    const double carr_k = 4294967296.0 / FS, code_k = 1099511627776.0 / FS;
    const int32_t if_word = (int32_t)llround(-2.658052e6 / FS * 4294967296.0);
    const uint64_t code_word0 = (uint64_t)llround(CHIP_HZ / FS * 1099511627776.0);

    trk_profile_t cur = quiet;  /* the loops in force, as rx_tick moves them */
    rng_t rng;
    rng_seed(&rng, seed);
    size_t hint = 0;
    trk_ch_t ch;
    trk_start(&ch, 1, (float)(dop_at(&tr, t_from, &hint) + dop_err), 0.25f, if_word, code_word0, (float)carr_k,
              (float)code_k);

    /* NCO frequencies (Hz from L1) and code rates (chips/s) in force for the next MAX_DELAY periods. */
    double f_q[MAX_DELAY], r_q[MAX_DELAY];
    {
        int32_t cw;
        uint64_t kw;
        trk_words(&ch, &cw, &kw);
        for (int k = 0; k < MAX_DELAY; k++) {
            f_q[k] = (double)(cw - if_word) / carr_k;
            r_q[k] = CHIP_HZ + (double)(int64_t)(kw - code_word0) / code_k;
        }
    }
    double theta = 0.0, phi = 0.0, tau = 0.0;  /* true and NCO carrier phase (cycles), code error (chips) */
    int bit = 1;
    /* The antenna's wind-up, unwrapped (rev), and its last wrapped value; and the gain's floor seen. */
    double wind = 0.0, wind_prev = 0.0, gain_min_db = 0.0, wind_start = 0.0;
    if (sp.on) {
        double g0;
        spin_at(&sp, &tr, 0, t_from, &wind_prev, &g0);
    }

    /* Metrics over the scoring window ([0]) and its parts ([1..3], with --split). */
    typedef struct {
        double ferr_max, ferr_sq, code_max, ph_sq, unlocked;
        long n, nlocked, slips;
    } part_t;
    part_t pt[4];
    memset(pt, 0, sizeof(pt));
    double cn0_min = 1e9, off_at = -1.0;
    long half0 = 0, half_prev = 0;
    int have_half = 0;
    double e10 = 0.0;
    int n10 = 0;

    const long nper = (long)floor((t_to - t_from) / T_PER);
    for (long k = 0; k < nper; k++) {
        const double t0 = t_from + (double)k * T_PER;
        double cn0 = cn0_base;
        for (int j = 0; j < nsteps; j++) {
            if (step_t[j] <= t0) {
                cn0 = step_cn0[j];
            }
        }
        const int boosting = t0 >= boost0 && t0 < boost1;
        const double f_nco = f_q[0], r_nco = r_q[0];

        /* The period's correlation: carrier phase error sampled SUB times, code error at mid-period. */
        double cr = 0.0, ci = 0.0, f_mid = 0.0, amp = 1.0;
        const double dt = T_PER / SUB;
        wind_start = wind;
        double fa = dop_at(&tr, t0, &hint);
        for (int m = 0; m < SUB; m++) {
            double fb = dop_at(&tr, t0 + (m + 1) * dt, &hint);
            double th_mid = theta + 0.25 * (fa + 0.5 * (fa + fb)) * dt;  /* phase at the sub-step's middle */
            double ph_mid = phi + f_nco * 0.5 * dt;
            if (sp.on) {
                double w, g;
                spin_at(&sp, &tr, hint, t0 + (m + 0.5) * dt, &w, &g);
                wind += wrap_half(w - wind_prev);
                wind_prev = w;
                th_mid += wind;
                if (m == SUB / 2) {
                    amp = sqrt(g);
                    const double gdb = 10.0 * log10(g);
                    if (t0 >= score_from && gdb < gain_min_db) {
                        gain_min_db = gdb;
                    }
                }
            }
            double e = th_mid - ph_mid;
            cr += cos(TWO_PI * e);
            ci += sin(TWO_PI * e);
            theta += 0.5 * (fa + fb) * dt;
            phi += f_nco * dt;
            if (m == SUB / 2) {
                f_mid = fa;
            }
            fa = fb;
        }
        cr /= SUB;
        ci /= SUB;
        const double tau_mid = tau + 0.5 * ((CHIP_HZ + f_mid / 1540.0) - r_nco) * T_PER;
        tau += ((CHIP_HZ + f_mid / 1540.0) - r_nco) * T_PER;
        if (k % 20 == 0 && rng_uniform(&rng) < 0.5) {
            bit = -bit;
        }
        const double sig = sqrt(1.0 / (2.0 * pow(10.0, cn0 / 10.0) * T_PER));
        double n[6];
        rng_gauss2(&rng, &n[0], &n[1]);
        rng_gauss2(&rng, &n[2], &n[3]);
        rng_gauss2(&rng, &n[4], &n[5]);
        /* Early, prompt, late noise with correlations 0.75 (adjacent) and 0.5 (early-late). */
        double ne_i = n[0], np_i = 0.75 * n[0] + 0.6614378 * n[2], nl_i = 0.5 * n[0] + 0.5669467 * n[2] + 0.6546537 * n[4];
        double ne_q = n[1], np_q = 0.75 * n[1] + 0.6614378 * n[3], nl_q = 0.5 * n[1] + 0.5669467 * n[3] + 0.6546537 * n[5];
        const double ae = amp * bit * tri(tau_mid - 0.25), ap = amp * bit * tri(tau_mid),
                     al = amp * bit * tri(tau_mid + 0.25);
        corr_dump_t d;
        memset(&d, 0, sizeof(d));
        d.seq = (uint32_t)k;
        d.ie = (float)(ae * cr + sig * ne_i);
        d.qe = (float)(ae * ci + sig * ne_q);
        d.ip = (float)(ap * cr + sig * np_i);
        d.qp = (float)(ap * ci + sig * np_q);
        d.il = (float)(al * cr + sig * nl_i);
        d.ql = (float)(al * ci + sig * nl_q);
        int b;
        uint32_t bp;
        if (aid_lag >= 0.0 || aid_spin_lag >= 0.0) {
            double ff = 0.0;
            if (aid_lag >= 0.0) {
                /* The rate the aiding saw aid_lag ms ago, from the truth's Doppler slope there. */
                size_t h2 = hint;
                double ta = t0 + 0.5 * T_PER - 1e-3 * aid_lag;
                double r = (dop_at(&tr, ta + 0.5e-3, &h2) - dop_at(&tr, ta - 0.5e-3, &h2)) / 1e-3;
                ff += r * (1.0 + aid_sf) + aid_bias;
            }
            if (sp.on && aid_spin_lag >= 0.0) {
                /* The wind-up's rate of change as the gyro and attitude predict it, aid_spin_lag late. */
                const double ta = t0 + 0.5 * T_PER - 1e-3 * aid_spin_lag, h = 0.5e-3;
                double wm, w0, wp, g;
                spin_at(&sp, &tr, hint, ta - h, &wm, &g);
                spin_at(&sp, &tr, hint, ta, &w0, &g);
                spin_at(&sp, &tr, hint, ta + h, &wp, &g);
                ff += (wrap_half(wp - w0) - wrap_half(w0 - wm)) / (h * h);
            }
            ch.ff_rate = (float)ff;
        }
        trk_profile_step(&cur, boosting ? &boost : &quiet, (float)T_PER);
        trk_update(&ch, &cur, &d, (float)T_PER, &b, &bp);

        /* The command from this dump steers period k + delay. */
        for (int j = 0; j < MAX_DELAY - 1; j++) {
            f_q[j] = f_q[j + 1];
            r_q[j] = r_q[j + 1];
        }
        int32_t cw;
        uint64_t kw;
        trk_words(&ch, &cw, &kw);
        for (int j = delay - 1; j < MAX_DELAY; j++) {
            f_q[j] = (double)(cw - if_word) / carr_k;
            r_q[j] = CHIP_HZ + (double)(int64_t)(kw - code_word0) / code_k;
        }

        /* The true frequency is the Doppler plus the antenna's wind-up rate over the period. */
        const double e_now = theta + wind - phi, ferr = f_mid + (wind - wind_start) / T_PER - f_nco;
        e10 += e_now;
        n10++;
        if (n10 == 10) {
            double em = e10 / 10.0;
            long half = lround(2.0 * em);
            if (t0 >= score_from) {
                if (!have_half) {
                    half0 = half_prev = half;
                    have_half = 1;
                } else if (half != half_prev) {
                    int part = t0 < split1 ? 1 : (t0 < split2 ? 2 : 3);
                    pt[0].slips++;
                    pt[part].slips++;
                    half_prev = half;
                }
            }
            if (csv) {
                fprintf(csv, "%.3f,%.3f,%.3f,%.3f,%.4f,%.5f,%d,%.3f,%.2f,%d\n", t0, f_mid, f_nco, ferr, em, tau,
                        (int)ch.state, ch.pll_lock, ch.cn0, boosting);
            }
            e10 = 0.0;
            n10 = 0;
        }
        if (t0 >= score_from) {
            const int part = t0 < split1 ? 1 : (t0 < split2 ? 2 : 3);
            const int idx[2] = {0, part};
            for (int q = 0; q < 2; q++) {
                part_t *m = &pt[idx[q]];
                m->n++;
                m->ferr_sq += ferr * ferr;
                if (fabs(ferr) > m->ferr_max) {
                    m->ferr_max = fabs(ferr);
                }
                if (fabs(tau) > m->code_max) {
                    m->code_max = fabs(tau);
                }
                if (ch.state == TRK_LOCKED) {
                    double r = e_now - 0.5 * round(2.0 * e_now);
                    m->ph_sq += r * r;
                    m->nlocked++;
                } else {
                    m->unlocked += T_PER;
                }
            }
            if (ch.cn0 > 0.0f && ch.cn0 < cn0_min) {
                cn0_min = ch.cn0;
            }
        }
        if (ch.state == TRK_OFF) {
            off_at = t0;
            break;
        }
    }
    static const char *names[4] = {"", "pad_", "boost_", "coast_"};
    for (int q = 0; q < (isinf(split1) ? 1 : 4); q++) {
        const part_t *m = &pt[q];
        printf("%sferr_max=%.2f %sferr_rms=%.2f %sunlocked_s=%.2f %sslips=%ld %sphase_rms_deg=%.1f %scode_max_m=%.2f ",
               names[q], m->ferr_max, names[q], m->n ? sqrt(m->ferr_sq / (double)m->n) : 0.0, names[q], m->unlocked,
               names[q], m->slips, names[q], m->nlocked ? 360.0 * sqrt(m->ph_sq / (double)m->nlocked) : 0.0, names[q],
               m->code_max * 299792458.0 / CHIP_HZ);
    }
    printf("net_half=%ld cn0_min=%.1f off_at=%.2f gain_min_db=%.1f\n", have_half ? half_prev - half0 : 0,
           cn0_min < 1e9 ? cn0_min : 0.0, off_at, gain_min_db);
    if (csv) {
        fclose(csv);
    }
    free(tr.t);
    free(tr.f);
    free(tr.el);
    free(tr.az);
    return 0;
}
