/*
 * mcsim: a flight's every tracked signal at once, dump by dump, through the receiver's own tracking code
 * (core/trk), for the Monte Carlo runs of a loop design (README "A realistic flight").
 *
 * Each channel is one satellite's signal: GPS L1 C/A, Galileo E1-C and the BeiDou B1C pilot on L1; GPS
 * L5Q, Galileo E5a-Q and the BeiDou B2a pilot on L5. The scenario file (py/mc_scenario.py) gives each
 * one's line-of-sight Doppler, elevation and C/N0 every 0.1 s, from the real satellites' positions over
 * the flight and a link budget. Here, as trksim does for one channel: correlator dumps with each signal's
 * correlation shape and correlated tap noise, data bits on L1 C/A, and the contract's command delay, fed
 * to trk_update. Shared by every channel, as on the board:
 *   - the IMU (host/imu_ism6: the board's ISM6HG256X, its scale error drawn from the datasheet's
 *     spread), which detects the launch and burnout (core boost_detect) and, in an aided design, feeds
 *     each loop a.u / lambda;
 *   - the reference oscillator, whose frequency moves by gamma times the axial force, every carrier and
 *     code by the same fraction. An aided design feeds that forward once the P4 has learnt gamma in flight
 *     (README "Oscillator g-sensitivity"): from CLK_TRUST s after the launch it detects, with a first
 *     error CLK_ERR that decays over a second.
 *
 *   mcsim SCEN.csv FLIGHT.csv --design B|BA|BG|Q|QA [options] --out SUMMARY.csv
 */
#include "gnss/boost_detect.h"
#include "gnss/trk.h"
#include "gnss/types.h"
#include "imu_ism6.h"
#include "loops_arg.h"
#include "rng.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TWO_PI     6.283185307179586476925286766559
#define C_LIGHT    299792458.0
#define G0         9.80665
#define SUB_PER_MS 10        /* phase samples per millisecond of dump */
#define MAX_CH     128
#define MAX_DELAY  8
#define FS         20.46e6   /* the NCO words' sample rate */

typedef struct {
    const char *name;        /* as the scenario file names it */
    int sig;                 /* gnss_sig_t */
    double T;                /* dump, s */
    double fc, chip;         /* carrier, Hz; chip rate, chips/s */
    int boc, data;           /* BOC(1,1) correlation; data bits on the tracked code (L1 C/A) */
    double tap;              /* early and late offset, chips */
} sigdef_t;

static const sigdef_t SIGS[] = {
    {"L1CA", GNSS_SIG_GPS_L1CA, 1e-3, GNSS_FREQ_L1_HZ, 1.023e6, 0, 1, 0.25},
    {"E1C", GNSS_SIG_GAL_E1C, 4e-3, GNSS_FREQ_L1_HZ, 1.023e6, 1, 0, 0.1},
    {"B1CP", GNSS_SIG_BDS_B1CP, 10e-3, GNSS_FREQ_L1_HZ, 1.023e6, 1, 0, 0.1},
    {"L5Q", GNSS_SIG_GPS_L5Q, 1e-3, GNSS_FREQ_L5_HZ, 10.23e6, 0, 0, 0.25},
    {"E5AQ", GNSS_SIG_GAL_E5AQ, 1e-3, GNSS_FREQ_L5_HZ, 10.23e6, 0, 0, 0.25},
    {"B2AP", GNSS_SIG_BDS_B2AP, 1e-3, GNSS_FREQ_L5_HZ, 10.23e6, 0, 0, 0.25},
};
#define N_SIGDEF ((int)(sizeof(SIGS) / sizeof(SIGS[0])))

static double r_bpsk(double x)
{
    x = fabs(x);
    return x < 1.0 ? 1.0 - x : 0.0;
}

static double r_boc(double x)
{
    x = fabs(x);
    return x <= 0.5 ? 1.0 - 3.0 * x : (x <= 1.0 ? x - 1.0 : 0.0);
}

/* A signal's taps (BPSK: early, prompt, late; BOC: very early, early, prompt, late, very late) and the
 * Cholesky factor of their noise's correlation, which is the code's autocorrelation at their spacing. */
typedef struct {
    int n;
    double off[5];
    double L[5][5];
} taps_t;

static void taps_init(taps_t *tp, const sigdef_t *s)
{
    if (s->boc) {
        const double o[5] = {-0.5, -s->tap, 0.0, s->tap, 0.5};
        tp->n = 5;
        memcpy(tp->off, o, sizeof(o));
    } else {
        const double o[3] = {-s->tap, 0.0, s->tap};
        tp->n = 3;
        memcpy(tp->off, o, sizeof(o));
    }
    double c[5][5];
    for (int i = 0; i < tp->n; i++) {
        for (int j = 0; j < tp->n; j++) {
            const double d = tp->off[i] - tp->off[j];
            c[i][j] = (s->boc ? r_boc(d) : r_bpsk(d)) + (i == j ? 1e-9 : 0.0);
        }
    }
    memset(tp->L, 0, sizeof(tp->L));
    for (int i = 0; i < tp->n; i++) {
        for (int j = 0; j <= i; j++) {
            double sum = c[i][j];
            for (int k = 0; k < j; k++) {
                sum -= tp->L[i][k] * tp->L[j][k];
            }
            tp->L[i][j] = i == j ? sqrt(sum > 0.0 ? sum : 0.0) : (tp->L[j][j] > 0.0 ? sum / tp->L[j][j] : 0.0);
        }
    }
}

/* The flight: the axial kinematic acceleration over each 0.1 s interval from t[k] (s from ignition). */
typedef struct {
    int n;
    double *t, *acc;
    double *tm, *fm;         /* interval midpoints and the specific force there, for the oscillator */
} flight_t;

/* The IMU's truth: the specific force along the thrust, constant over each interval (+1 g before it). */
static double force_true(void *ctx, double t)
{
    const flight_t *f = (const flight_t *)ctx;
    if (f->n == 0 || t < f->t[0]) {
        return G0;
    }
    int lo = 0, hi = f->n - 1;
    if (t >= f->t[hi]) {
        return f->acc[hi] + G0;
    }
    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        if (f->t[mid] <= t) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return f->acc[lo] + G0;
}

/* The oscillator's force: linear between the interval midpoints, as gnssrx's osc_force; and its rate. */
static double force_osc(const flight_t *f, double t, double *rate)
{
    *rate = 0.0;
    if (f->n < 2 || t <= f->tm[0]) {
        return f->n ? f->fm[0] : G0;
    }
    if (t >= f->tm[f->n - 1]) {
        return f->fm[f->n - 1];
    }
    int lo = 0, hi = f->n - 1;
    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        if (f->tm[mid] <= t) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    *rate = (f->fm[hi] - f->fm[lo]) / (f->tm[hi] - f->tm[lo]);
    return f->fm[lo] + *rate * (t - f->tm[lo]);
}

typedef struct {
    const sigdef_t *sd;
    const taps_t *tp;
    char sat[8];
    int id;
    int n;
    double *t, *dop, *el, *cn0;
    trk_ch_t ch;
    rng_t rng;
    double e;                /* carrier phase error, cycles: the true phase less the NCO's */
    double tau;              /* code error, chips */
    double f_q[MAX_DELAY], r_q[MAX_DELAY];
    int bit;
    long k;                  /* dumps done */
    int per_ms;              /* milliseconds a dump */
    int32_t if_word;
    uint64_t code_word0;
    double carr_k, code_k;
    /* Slips: the phase error over about 10 ms, in half cycles (L1 C/A's Costas loop) or cycles (pilots). */
    double e_acc;
    int e_n, e_len, have_q;
    long q_prev;
    /* Per window: 0 the pad's last 2 s, 1 the boost (ignition to burnout), 2 the 20 s after. */
    int off[3];
    double unl[3], cn0min[3];
    long slips[3];
    int locked_at_ign, ign_seen;
    double t_off, el0, cn00;
} chan_t;

static double interp(const double *t, const double *y, int n, double x)
{
    if (x <= t[0]) {
        return y[0];
    }
    if (x >= t[n - 1]) {
        return y[n - 1];
    }
    const double step = t[1] - t[0];
    int k = (int)((x - t[0]) / step);
    if (k < 0) {
        k = 0;
    }
    if (k > n - 2) {
        k = n - 2;
    }
    while (k > 0 && t[k] > x) {
        k--;
    }
    while (k < n - 2 && t[k + 1] < x) {
        k++;
    }
    const double u = (x - t[k]) / (t[k + 1] - t[k]);
    return y[k] + u * (y[k + 1] - y[k]);
}

static const sigdef_t *find_sig(const char *name)
{
    for (int i = 0; i < N_SIGDEF; i++) {
        if (!strcmp(SIGS[i].name, name)) {
            return &SIGS[i];
        }
    }
    return NULL;
}

static int read_flight(const char *path, flight_t *f)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        return -1;
    }
    int cap = 1024;
    f->t = malloc(sizeof(double) * (size_t)cap);
    f->acc = malloc(sizeof(double) * (size_t)cap);
    f->n = 0;
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        double a, b;
        if (sscanf(line, "%lf,%lf", &a, &b) != 2) {
            continue;
        }
        if (f->n == cap) {
            cap *= 2;
            f->t = realloc(f->t, sizeof(double) * (size_t)cap);
            f->acc = realloc(f->acc, sizeof(double) * (size_t)cap);
        }
        f->t[f->n] = a;
        f->acc[f->n] = b;
        f->n++;
    }
    fclose(fp);
    f->tm = malloc(sizeof(double) * (size_t)(f->n > 0 ? f->n : 1));
    f->fm = malloc(sizeof(double) * (size_t)(f->n > 0 ? f->n : 1));
    for (int k = 0; k < f->n; k++) {
        f->tm[k] = f->t[k] + 0.5 * (k + 1 < f->n ? f->t[k + 1] - f->t[k] : 0.1);
        f->fm[k] = f->acc[k] + G0;
    }
    return f->n > 1 ? 0 : -1;
}

/* SCEN.csv: ch,sig,sat,t_s,dop_hz,el_deg,cn0_dbhz, each channel's rows together and in time order. */
static int read_scen(const char *path, chan_t *ch, int *nch)
{
    FILE *fp = fopen(path, "r");
    if (!fp) {
        return -1;
    }
    char line[256];
    int cur = -1, cap = 0;
    *nch = 0;
    while (fgets(line, sizeof(line), fp)) {
        int id;
        char sig[16], sat[8];
        double t, dop, el, cn0;
        if (sscanf(line, "%d,%15[^,],%7[^,],%lf,%lf,%lf,%lf", &id, sig, sat, &t, &dop, &el, &cn0) != 7) {
            continue;
        }
        if (cur < 0 || ch[cur].id != id) {
            if (*nch == MAX_CH) {
                fclose(fp);
                return -2;
            }
            cur = (*nch)++;
            memset(&ch[cur], 0, sizeof(ch[cur]));
            ch[cur].id = id;
            ch[cur].sd = find_sig(sig);
            if (!ch[cur].sd) {
                fprintf(stderr, "mcsim: unknown signal %s\n", sig);
                fclose(fp);
                return -3;
            }
            snprintf(ch[cur].sat, sizeof(ch[cur].sat), "%s", sat);
            cap = 512;
            ch[cur].t = malloc(sizeof(double) * (size_t)cap);
            ch[cur].dop = malloc(sizeof(double) * (size_t)cap);
            ch[cur].el = malloc(sizeof(double) * (size_t)cap);
            ch[cur].cn0 = malloc(sizeof(double) * (size_t)cap);
        }
        chan_t *c = &ch[cur];
        if (c->n == cap) {
            cap *= 2;
            c->t = realloc(c->t, sizeof(double) * (size_t)cap);
            c->dop = realloc(c->dop, sizeof(double) * (size_t)cap);
            c->el = realloc(c->el, sizeof(double) * (size_t)cap);
            c->cn0 = realloc(c->cn0, sizeof(double) * (size_t)cap);
        }
        c->t[c->n] = t;
        c->dop[c->n] = dop;
        c->el[c->n] = el * TWO_PI / 360.0;
        c->cn0[c->n] = cn0;
        c->n++;
    }
    fclose(fp);
    return 0;
}

static void usage(void)
{
    fprintf(stderr,
            "usage: mcsim SCEN.csv FLIGHT.csv --design B|BA|BG|Q|QA [options]\n"
            "  SCEN.csv                   ch,sig,sat,t_s,dop_hz,el_deg,cn0_dbhz every 0.1 s (py/mc_scenario.py);\n"
            "                             sig L1CA, E1C, B1CP, L5Q, E5AQ or B2AP; t from ignition\n"
            "  FLIGHT.csv                 t_s,acc_up: the axial kinematic acceleration over each 0.1 s interval\n"
            "  --design D                 B: 50 Hz boost loops, no feed-forward; BA: IMU + 20 Hz boost loops;\n"
            "                             BG: BA gated to ignition and burnout; Q: quiet loops; QA: quiet + IMU\n"
            "  --from S --to S            (default -12, 25)\n"
            "  --burnout S                the true burnout, for the windows (default 4.0)\n"
            "  --after S                  the window after burnout (default 20)\n"
            "  --gamma PPB                the oscillator's sensitivity along the thrust, ppb/g (default 0)\n"
            "  --clk-trust S --clk-err E  aided: the clock feed-forward from S after the detected launch, its\n"
            "                             learnt gamma first E off (a fraction), the error decaying over 1 s\n"
            "                             (default 0.5, 0)\n"
            "  --cn0-offset DB            added to every signal's C/N0: a weaker antenna or installation\n"
            "  --odr HZ                   the IMU's output data rate (default 960)\n"
            "  --imu-seed N               the IMU part's scale error (datasheet spread) and its noise\n"
            "  --seed N                   thermal noise, data bits, starting errors\n"
            "  --out PATH                 a line a channel: windows' off, unlocked s, slips, least C/N0\n");
}

int main(int argc, char **argv)
{
    const char *scen = NULL, *flight_path = NULL, *design = NULL, *out_path = NULL;
    double t_from = -12.0, t_to = 25.0, burnout = 4.0, after = 20.0, gamma_ppb = 0.0, clk_trust = 0.5,
           clk_err = 0.0, odr = 960.0, cn0_off = 0.0;
    uint64_t seed = 1, imu_seed = 1;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (a[0] != '-') {
            if (!scen) {
                scen = a;
            } else {
                flight_path = a;
            }
            continue;
        }
        if (i + 1 >= argc) {
            usage();
            return 2;
        }
        const char *v = argv[++i];
        if (!strcmp(a, "--design")) {
            design = v;
        } else if (!strcmp(a, "--from")) {
            t_from = atof(v);
        } else if (!strcmp(a, "--to")) {
            t_to = atof(v);
        } else if (!strcmp(a, "--burnout")) {
            burnout = atof(v);
        } else if (!strcmp(a, "--after")) {
            after = atof(v);
        } else if (!strcmp(a, "--gamma")) {
            gamma_ppb = atof(v);
        } else if (!strcmp(a, "--clk-trust")) {
            clk_trust = atof(v);
        } else if (!strcmp(a, "--clk-err")) {
            clk_err = atof(v);
        } else if (!strcmp(a, "--cn0-offset")) {
            cn0_off = atof(v);
        } else if (!strcmp(a, "--odr")) {
            odr = atof(v);
        } else if (!strcmp(a, "--imu-seed")) {
            imu_seed = strtoull(v, NULL, 10);
        } else if (!strcmp(a, "--seed")) {
            seed = strtoull(v, NULL, 10);
        } else if (!strcmp(a, "--out")) {
            out_path = v;
        } else {
            usage();
            return 2;
        }
    }
    if (!scen || !flight_path || !design || !out_path) {
        usage();
        return 2;
    }

    /* The design: its boost profile, whether it widens at all, whether it aids, whether it gates. */
    trk_profile_t quiet = trk_profile_quiet, boost = trk_profile_boost, aided20;
    loops_arg_parse("10,20,2/0,20,0.5:2", &aided20);
    int aided = 0, gate = 0;
    const trk_profile_t *prof_boost = &boost;
    if (!strcmp(design, "B")) {
        prof_boost = &boost;
    } else if (!strcmp(design, "BA")) {
        prof_boost = &aided20;
        aided = 1;
    } else if (!strcmp(design, "BG")) {
        prof_boost = &aided20;
        aided = 1;
        gate = 1;
    } else if (!strcmp(design, "Q")) {
        prof_boost = &quiet;
    } else if (!strcmp(design, "QA")) {
        prof_boost = &quiet;
        aided = 1;
    } else {
        usage();
        return 2;
    }

    flight_t fl;
    if (read_flight(flight_path, &fl)) {
        fprintf(stderr, "mcsim: cannot read %s\n", flight_path);
        return 1;
    }
    static chan_t chs[MAX_CH];
    int nch = 0;
    if (read_scen(scen, chs, &nch) || nch == 0) {
        fprintf(stderr, "mcsim: cannot read %s\n", scen);
        return 1;
    }
    static taps_t taps[N_SIGDEF];
    for (int i = 0; i < N_SIGDEF; i++) {
        taps_init(&taps[i], &SIGS[i]);
    }

    /* The IMU: the typical part, its scale error drawn from the datasheet's +-1 % (3 sigma). */
    ism6_cfg_t icfg;
    ism6_cfg_default(&icfg, 0);
    icfg.odr_hz = odr;
    rng_t prng;
    rng_seed(&prng, imu_seed * 7919 + 17);
    {
        double z0, z1;
        rng_gauss2(&prng, &z0, &z1);
        icfg.sf_lg = icfg.sf_hg = z0 * 0.01 / 3.0;
        rng_gauss2(&prng, &z0, &z1);
        icfg.off_lg_g = z0 * 10e-3;
        icfg.off_hg_g = z1 * 250e-3;
    }
    ism6_t imu;
    ism6_init(&imu, &icfg, t_from, imu_seed);
    const double imu_delay = ism6_mean_delay(&icfg);

    boost_detect_cfg_t bdc;
    boost_detect_default(&bdc);
    if (gate) {
        boost_detect_gate(&bdc, 1.0f, 0.9f, 10);
    }
    boost_detect_t bd;
    boost_detect_init(&bd, &bdc);
    trk_profile_t cur = quiet;

    double f_pad_rate;
    const double f_pad = force_osc(&fl, t_from, &f_pad_rate);
    const double gamma = gamma_ppb * 1e-9;
    const int delay = 3;

    for (int i = 0; i < nch; i++) {
        chan_t *c = &chs[i];
        const sigdef_t *sd = c->sd;
        c->tp = &taps[sd - SIGS];
        rng_seed(&c->rng, seed * 1000003ull + (uint64_t)c->id * 7919ull + 1);
        c->per_ms = (int)lround(sd->T * 1e3);
        c->e_len = c->per_ms >= 10 ? 1 : 10 / c->per_ms;
        c->carr_k = 4294967296.0 / FS;
        c->code_k = 1099511627776.0 / FS;
        c->if_word = (int32_t)llround(-4.0e6 / FS * 4294967296.0);
        c->code_word0 = (uint64_t)llround(sd->chip / FS * 1099511627776.0);
        double z0, z1;
        rng_gauss2(&c->rng, &z0, &z1);
        const double dop0 = interp(c->t, c->dop, c->n, t_from), f0 = dop0 + 10.0 * z0;
        c->tau = 0.1 * z1;
        c->e = rng_uniform(&c->rng);
        c->bit = 1;
        int prn = 0;
        sscanf(c->sat + 1, "%d", &prn);
        trk_start(&c->ch, prn, (float)f0, (float)sd->tap, c->if_word, c->code_word0, (float)c->carr_k,
                  (float)c->code_k);
        if (sd->sig != GNSS_SIG_GPS_L1CA) {
            trk_set_signal(&c->ch, sd->sig);
        }
        for (int j = 0; j < MAX_DELAY; j++) {
            c->f_q[j] = f0;
            c->r_q[j] = sd->chip * (1.0 + f0 / sd->fc);
        }
        for (int w = 0; w < 3; w++) {
            c->cn0min[w] = 99.0;
        }
        c->t_off = NAN;
        c->el0 = interp(c->t, c->el, c->n, 0.0) * 360.0 / TWO_PI;
        c->cn00 = interp(c->t, c->cn0, c->n, 0.0) + cn0_off;
    }

    double t_launch = NAN;
    const long nticks = (long)floor((t_to - t_from) * 1e3 + 0.5);
    for (long it = 0; it < nticks; it++) {
        const double t_end = t_from + (double)(it + 1) * 1e-3;
        const double f_imu = ism6_read(&imu, t_end, force_true, &fl);
        const int on = boost_detect_step(&bd, (float)f_imu);
        if (isnan(t_launch) && bd.phase != BOOST_PAD) {
            t_launch = t_end;
        }
        trk_profile_step(&cur, on ? prof_boost : &quiet, 1e-3f);
        const double a_up_imu = f_imu - G0;
        /* The learnt clock feed-forward at L1, Hz/s: the oscillator's rate as late as the IMU, through the
         * learnt gamma. */
        double clk_l1 = 0.0;
        if (aided && gamma != 0.0 && !isnan(t_launch) && t_end >= t_launch + clk_trust) {
            double rate;
            force_osc(&fl, t_end - imu_delay, &rate);
            const double g_est = gamma * (1.0 + clk_err * exp(-(t_end - t_launch - clk_trust) / 1.0));
            clk_l1 = -GNSS_FREQ_L1_HZ * g_est * rate / G0;
        }
        for (int i = 0; i < nch; i++) {
            chan_t *c = &chs[i];
            if ((it + 1) % c->per_ms != 0) {
                continue;
            }
            const sigdef_t *sd = c->sd;
            const double T = sd->T, t0 = t_end - T;
            if (c->ch.state != TRK_OFF) {
                const int nsub = SUB_PER_MS * c->per_ms;
                const double dt = T / nsub, f_nco = c->f_q[0], r_nco = c->r_q[0], kf = sd->fc / GNSS_FREQ_L1_HZ;
                double cr = 0.0, ci = 0.0, f_mid = 0.0;
                for (int m = 0; m < nsub; m++) {
                    const double ts = t0 + (m + 0.5) * dt;
                    double rate;
                    const double osc = -GNSS_FREQ_L1_HZ * gamma * (force_osc(&fl, ts, &rate) - f_pad) / G0;
                    const double f_true = interp(c->t, c->dop, c->n, ts) + osc * kf;
                    const double de = (f_true - f_nco) * dt;
                    const double e_mid = c->e + 0.5 * de;
                    c->e += de;
                    cr += cos(TWO_PI * e_mid);
                    ci += sin(TWO_PI * e_mid);
                    if (m == nsub / 2) {
                        f_mid = f_true;
                    }
                }
                cr /= nsub;
                ci /= nsub;
                const double r_true = sd->chip * (1.0 + f_mid / sd->fc);
                const double tau_mid = c->tau + 0.5 * (r_true - r_nco) * T;
                c->tau += (r_true - r_nco) * T;
                if (sd->data && c->k % 20 == 0 && rng_uniform(&c->rng) < 0.5) {
                    c->bit = -c->bit;
                }
                const double cn0 = interp(c->t, c->cn0, c->n, t0 + 0.5 * T) + cn0_off;
                const double sig = sqrt(1.0 / (2.0 * pow(10.0, cn0 / 10.0) * T));
                const taps_t *tp = c->tp;
                double zi[5], zq[5], vi[5], vq[5];
                for (int j = 0; j < tp->n; j++) {
                    rng_gauss2(&c->rng, &zi[j], &zq[j]);
                }
                for (int j = 0; j < tp->n; j++) {
                    double ni = 0.0, nq = 0.0;
                    for (int q = 0; q <= j; q++) {
                        ni += tp->L[j][q] * zi[q];
                        nq += tp->L[j][q] * zq[q];
                    }
                    const double s = c->bit * (sd->boc ? r_boc(tau_mid + tp->off[j]) : r_bpsk(tau_mid + tp->off[j]));
                    vi[j] = s * cr + sig * ni;
                    vq[j] = s * ci + sig * nq;
                }
                corr_dump_t d;
                memset(&d, 0, sizeof(d));
                d.seq = (uint32_t)c->k;
                if (sd->boc) {
                    d.ive = (float)vi[0], d.qve = (float)vq[0];
                    d.ie = (float)vi[1], d.qe = (float)vq[1];
                    d.ip = (float)vi[2], d.qp = (float)vq[2];
                    d.il = (float)vi[3], d.ql = (float)vq[3];
                    d.ivl = (float)vi[4], d.qvl = (float)vq[4];
                } else {
                    d.ie = (float)vi[0], d.qe = (float)vq[0];
                    d.ip = (float)vi[1], d.qp = (float)vq[1];
                    d.il = (float)vi[2], d.ql = (float)vq[2];
                }
                if (aided) {
                    const double el = interp(c->t, c->el, c->n, t_end);
                    c->ch.ff_rate = (float)(a_up_imu * sin(el) / (C_LIGHT / sd->fc) + clk_l1 * kf);
                    c->ch.ff_lead = (float)(delay * T);
                }
                int b;
                uint32_t bp;
                trk_update(&c->ch, &cur, &d, (float)T, &b, &bp);
                for (int j = 0; j < MAX_DELAY - 1; j++) {
                    c->f_q[j] = c->f_q[j + 1];
                    c->r_q[j] = c->r_q[j + 1];
                }
                int32_t cw;
                uint64_t kw;
                trk_words(&c->ch, &cw, &kw);
                for (int j = delay - 1; j < MAX_DELAY; j++) {
                    c->f_q[j] = (double)(cw - c->if_word) / c->carr_k;
                    c->r_q[j] = sd->chip + (double)(int64_t)(kw - c->code_word0) / c->code_k;
                }
                /* Slips, over about 10 ms of phase error. */
                c->e_acc += c->e;
                if (++c->e_n == c->e_len) {
                    const double em = c->e_acc / c->e_n;
                    const long q = lround(sd->data ? 2.0 * em : em);
                    if (t_end >= -1.0) {
                        if (!c->have_q) {
                            c->q_prev = q;
                            c->have_q = 1;
                        } else if (q != c->q_prev) {
                            const int w = t_end <= 0.0 ? 0 : (t_end <= burnout ? 1 : (t_end <= burnout + after ? 2 : -1));
                            if (w >= 0) {
                                c->slips[w] += labs(q - c->q_prev);
                            }
                            c->q_prev = q;
                        }
                    }
                    c->e_acc = 0.0;
                    c->e_n = 0;
                }
            }
            c->k++;
            /* The windows. */
            if (!c->ign_seen && t_end >= -0.5) {
                c->ign_seen = 1;
                c->locked_at_ign = c->ch.state == TRK_LOCKED;
            }
            const int w = t_end < -2.0 ? -1 : (t_end <= 0.0 ? 0 : (t_end <= burnout ? 1 : (t_end <= burnout + after ? 2 : -1)));
            if (w >= 0) {
                if (c->ch.state != TRK_LOCKED) {
                    c->unl[w] += T;
                }
                if (c->ch.state == TRK_OFF) {
                    c->off[w] = 1;
                } else if (c->ch.cn0 > 0.0f && c->ch.cn0 < c->cn0min[w]) {
                    c->cn0min[w] = c->ch.cn0;
                }
            }
            if (c->ch.state == TRK_OFF && isnan(c->t_off)) {
                c->t_off = t_end;
            }
        }
    }

    FILE *out = fopen(out_path, "w");
    if (!out) {
        fprintf(stderr, "mcsim: cannot write %s\n", out_path);
        return 1;
    }
    fprintf(out, "ch,sig,sat,el0,cn0_0,locked_at_ign,t_off,launch_s,"
                 "pad_off,pad_unl,pad_slips,boost_off,boost_unl,boost_slips,boost_cn0min,"
                 "after_off,after_unl,after_slips,after_cn0min\n");
    for (int i = 0; i < nch; i++) {
        const chan_t *c = &chs[i];
        fprintf(out, "%d,%s,%s,%.2f,%.2f,%d,%.3f,%.3f,%d,%.3f,%ld,%d,%.3f,%ld,%.2f,%d,%.3f,%ld,%.2f\n", c->id,
                c->sd->name, c->sat, c->el0, c->cn00, c->locked_at_ign, c->t_off, t_launch, c->off[0], c->unl[0],
                c->slips[0], c->off[1], c->unl[1], c->slips[1], c->cn0min[1], c->off[2], c->unl[2], c->slips[2],
                c->cn0min[2]);
    }
    fclose(out);
    return 0;
}
