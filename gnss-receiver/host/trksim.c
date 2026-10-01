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
 * gps-sdr-sim files make it.
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
    double *t, *f;
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
    tr->n = 0;
    char line[512];
    while (fgets(line, sizeof(line), fp)) {
        double a, b;
        if (sscanf(line, "%lf,%lf", &a, &b) != 2) {
            continue;  /* header */
        }
        if (tr->n == cap) {
            cap *= 2;
            tr->t = realloc(tr->t, sizeof(double) * cap);
            tr->f = realloc(tr->f, sizeof(double) * cap);
        }
        tr->t[tr->n] = a;
        tr->f[tr->n] = b;
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

static double tri(double x)
{
    x = fabs(x);
    return x < 1.0 ? 1.0 - x : 0.0;
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
            "  --csv PATH                 a row every 10 ms\n");
}

int main(int argc, char **argv)
{
    const char *truth_path = NULL, *csv_path = NULL;
    double t_from = NAN, t_to = NAN, score_from = NAN, boost0 = INFINITY, boost1 = -INFINITY, dop_err = 30.0;
    double split1 = INFINITY, split2 = INFINITY;
    double aid_lag = -1.0, aid_sf = 0.0, aid_bias = 0.0;
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
        double cr = 0.0, ci = 0.0, f_mid = 0.0;
        const double dt = T_PER / SUB;
        double fa = dop_at(&tr, t0, &hint);
        for (int m = 0; m < SUB; m++) {
            double fb = dop_at(&tr, t0 + (m + 1) * dt, &hint);
            double th_mid = theta + 0.25 * (fa + 0.5 * (fa + fb)) * dt;  /* phase at the sub-step's middle */
            double ph_mid = phi + f_nco * 0.5 * dt;
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
        const double ae = bit * tri(tau_mid - 0.25), ap = bit * tri(tau_mid), al = bit * tri(tau_mid + 0.25);
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
        if (aid_lag >= 0.0) {
            /* The rate the aiding saw aid_lag ms ago, from the truth's Doppler slope there. */
            size_t h2 = hint;
            double ta = t0 + 0.5 * T_PER - 1e-3 * aid_lag;
            double r = (dop_at(&tr, ta + 0.5e-3, &h2) - dop_at(&tr, ta - 0.5e-3, &h2)) / 1e-3;
            ch.ff_rate = (float)(r * (1.0 + aid_sf) + aid_bias);
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

        const double e_now = theta - phi, ferr = f_mid - f_nco;
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
    printf("net_half=%ld cn0_min=%.1f off_at=%.2f\n", have_half ? half_prev - half0 : 0, cn0_min < 1e9 ? cn0_min : 0.0,
           off_at);
    if (csv) {
        fclose(csv);
    }
    free(tr.t);
    free(tr.f);
    return 0;
}
