/*
 * gnssrx: the stage-0 receiver on a recorded IQ file.
 *
 *   gnssrx FILE [source options] [receiver options]
 *
 * The file runs through the front-end emulation (host/source.c) into a
 * correlator bank: the bit-exact FPGA model (fpga/model/corr_model.c, the
 * default for 2-bit streams) or the float correlator (host/corr_float.c). The
 * receiver core (core/) gets a 1 ms tick with the dumps, exactly as it will
 * from the FPGA's interrupt on the P4. Writes trk.csv, obs.csv, pvt.csv and
 * eph.csv into the output directory, and HDL test vectors on request.
 */
#define _POSIX_C_SOURCE 200809L

#include "corr_float.h"
#include "corr_model.h"
#include "fe_format.h"
#include "gnss/boost_detect.h"
#include "gnss/rx.h"
#include "loops_arg.h"
#include "manifest.h"
#include "rng.h"
#include "rinex_nav.h"
#include "source.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#define PI 3.14159265358979323846
#define MAX_DUMPS (4 * CORR_MAX_CH)
#define MAX_CMDS (2 * CORR_MAX_CH)

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

static void usage(void)
{
    fprintf(stderr,
            "usage: gnssrx FILE [source options] [receiver options]\n"
            "receiver options:\n"
            "  --out DIR                 output directory (default runs/gnssrx)\n"
            "  --corr golden|float       correlator: the bit-exact FPGA model (default for 2-bit\n"
            "                            streams) or the float one (always for native streams)\n"
            "  --lut-bits N --lut-amp A  golden model: a 2^N-sector carrier table with levels\n"
            "                            round(A * cos), for trade studies (default: corr_params.h)\n"
            "  --vectors DIR --vectors-ms N   write HDL test vectors for the first N ms (golden only)\n"
            "  --meas-hz N               observables and PVT rate (default 10)\n"
            "  --p4-latency-us T         commands reach the correlator T us after the tick (< 1000),\n"
            "                            as the P4's processing would deliver them\n"
            "  --acq-threshold M         acquisition detection threshold\n"
            "  --acq-interval S          seconds between searches for satellites not in a channel (default 5)\n"
            "  --hatch S                 carrier smoothing of the pseudoranges over S seconds (default 100; 0 off)\n"
            "  --hatch-slip K[,TAU_S]    restart a channel's smoothing when its code walks K times its noise\n"
            "                            from its carrier, against the others, averaged over TAU_S (4,8; 0 off)\n"
            "  --iono A0,A1,A2,A3,B0,B1,B2,B3   preload Klobuchar parameters (default: the manifest's)\n"
            "  --no-iono --no-tropo      leave the atmosphere uncorrected\n"
            "  --no-raim                 skip the fix's residual test\n"
            "  --pvt-unweighted          equal weights in the fix (with --no-raim; for comparisons)\n"
            "  --pvt-adapt-tau S         learn each satellite's residual spread over S seconds (default 30;\n"
            "                            0 = the C/N0 model alone)\n"
            "  --truth static:LAT,LON,H  truth for the error statistics (default: the manifest's)\n"
            "  --loops-quiet SPEC --loops-boost SPEC   tracking loop profiles, PF,PP,PD/LF,LP,LD[:MS]:\n"
            "                            pull-in / locked FLL,PLL,DLL bandwidths (Hz), FLL block (ms)\n"
            "  --boost-at S0,S1          the boost profile from file second S0 to S1, as the flight\n"
            "                            computer would call it (default: never)\n"
            );
    fprintf(stderr,
            "  --boost-detect default|fc|LAUNCH,LAUNCH_MS,BURNOUT_MS,HOLD_S[,REST,REST_MS]   the boost profile\n"
            "                            from the launch the emulated IMU detects (--imu) to HOLD_S past the\n"
            "                            burnout it detects: the axial specific force over LAUNCH m/s^2 for\n"
            "                            LAUNCH_MS, then below zero for BURNOUT_MS; within REST of 1 g for\n"
            "                            REST_MS is a false start. default: the receiver's fast trigger,\n"
            "                            20,20,50,2,5,500; fc: the flight computer's rules, 30,250,50,2, no rest\n"
            "  --boost-gate IGN_S,TAIL_FRAC,TAIL_MS   with --boost-detect, the profile wide only around the\n"
            "                            transitions: IGN_S after launch, and from the tail-off (the axial force\n"
            "                            under TAIL_FRAC of its peak for TAIL_MS) to the hold past burnout\n"
            "  --imu SCEN.csv            IMU aiding emulated from a scenario trajectory (10 Hz t,lat,lon,h;\n"
            "                            file seconds): its acceleration, as the generator's carrier sees it\n"
            "  --imu-err LAG,SF,BIAS,NOISE[,TILT]   the IMU's faults: latency ms, scale error (fraction),\n"
            "                            bias m/s^2 along local up, white noise m/s^2 per axis, and an attitude\n"
            "                            error in degrees that tips the acceleration from up toward east\n"
            "                            (default 5,0.03,0.5,0.1,0)\n"
            "  --imu-at S0,S1            aid only from file second S0 to S1 (default: all the run)\n"
            "  --imu-no-aid              the IMU only detects launch and burnout (--boost-detect); no feed-forward\n"
            "  --osc-g GAMMA[,COMP]      the reference oscillator's g-sensitivity along the thrust axis, ppb/g:\n"
            "                            its frequency follows the specific force of the trajectory (--osc-traj,\n"
            "                            default the --imu one), and COMP of it (0..1, default 0) is fed forward\n"
            "                            to every channel from the IMU (rx_set_clock_rate), as a calibrated\n"
            "                            sensitivity would be. COMP -1: the P4 learns it in flight, from the\n"
            "                            fixes' clock drift against the IMU's specific force since the pad\n"
            "  --osc-vib F_HZ,A_G[,S0,S1]   a vibration tone along the thrust axis, A_G peak, from file second S0\n"
            "                            to S1 (default the --boost-at window), through the same sensitivity\n"
            "  --osc-traj SCEN.csv       the trajectory for --osc-g\n"
            "  --pilot-dumps             write pilot channels' raw dumps (before secondary-code wipe-off)\n"
            "                            to pilot_dumps.csv, for checking the wipe-off\n"
            "  --nav FILE | --no-nav     RINEX navigation file to preload Galileo and BeiDou ephemerides\n"
            "                            from, for aided starts (default: the manifest's nav)\n"
            "  --preload-gps             preload GPS ephemerides from it too, as the flight computer could\n"
            "                            hand them over (decoded ones replace them)\n"
            "  --prior ERR_M,ERR_MS[,SIGMA_MS[,VEL_SIGMA[,POS_SIGMA]]]\n"
            "                            seed the receiver (rx_set_seed) at the run's start with the manifest's\n"
            "                            static truth moved ERR_M east and its start_gpst ERR_MS late, as the\n"
            "                            flight computer would from the pad and its clock; it claims the time\n"
            "                            good to SIGMA_MS (default |ERR_MS|, at least 0.001), zero velocity\n"
            "                            good to VEL_SIGMA m/s (default 1) and the position good to POS_SIGMA m\n"
            "                            (default |ERR_M|, at least 100)\n");
    src_usage();
}

/* "YYYY-MM-DD HH:MM:SS" (GPS time) to seconds since the GPS epoch, 1980-01-06; returns -1 if unparsed. */
static int gpst_parse(const char *s, double *t)
{
    int y, mo, d, hh, mi;
    double ss;
    if (!s || sscanf(s, "%d-%d-%d %d:%d:%lf", &y, &mo, &d, &hh, &mi, &ss) != 6) {
        return -1;
    }
    int yy = y - (mo <= 2), mm = mo;
    long era = (yy >= 0 ? yy : yy - 399) / 400, yoe = yy - era * 400;
    long doy = (153 * (mm + (mm > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    long days = era * 146097 + yoe * 365 + yoe / 4 - yoe / 100 + doy - 719468 - 3657;
    *t = (double)days * 86400.0 + hh * 3600.0 + mi * 60.0 + ss;
    return 0;
}

static FILE *open_csv(const char *dir, const char *name, const char *header)
{
    char p[4200];
    snprintf(p, sizeof(p), "%s/%s", dir, name);
    FILE *f = fopen(p, "w");
    if (f && header) {
        fprintf(f, "%s\n", header);
    }
    return f;
}

/* Truth "static:LAT,LON,H" as ECEF; returns 1 if there is one. */
static int static_truth(const char *truth, double ref[3], double *lat, double *lon)
{
    double la, lo, h;
    if (sscanf(truth, "static:%lf,%lf,%lf", &la, &lo, &h) != 3) {
        return 0;
    }
    *lat = la * PI / 180.0;
    *lon = lo * PI / 180.0;
    geo_to_ecef(*lat, *lon, h, ref);
    return 1;
}

/*
 * IMU emulation from a scenario trajectory (10 Hz t,lat,lon,h, file seconds). gps-sdr-sim's
 * smoothed carrier ramps between the central-difference velocities at the samples, so the
 * acceleration the signal shows is constant over each 0.1 s step: that is the "true" IMU.
 */
typedef struct {
    int n;
    double *t;
    double (*acc)[3];      /* over [t[k], t[k+1]) */
    double up[3];          /* local up at the first sample, for the bias */
} imu_traj_t;

static int imu_load(const char *path, imu_traj_t *m)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        return -1;
    }
    int cap = 1024, n = 0;
    double *t = malloc(sizeof(double) * (size_t)cap), (*p)[3] = malloc(sizeof(double[3]) * (size_t)cap);
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        double tt, la, lo, h;
        if (sscanf(line, "%lf,%lf,%lf,%lf", &tt, &la, &lo, &h) != 4) {
            continue;
        }
        if (n == cap) {
            cap *= 2;
            t = realloc(t, sizeof(double) * (size_t)cap);
            p = realloc(p, sizeof(double[3]) * (size_t)cap);
        }
        t[n] = tt;
        geo_to_ecef(la * PI / 180.0, lo * PI / 180.0, h, p[n]);
        if (n == 0) {
            m->up[0] = cos(la * PI / 180.0) * cos(lo * PI / 180.0);
            m->up[1] = cos(la * PI / 180.0) * sin(lo * PI / 180.0);
            m->up[2] = sin(la * PI / 180.0);
        }
        n++;
    }
    fclose(f);
    if (n < 3) {
        free(t);
        free(p);
        return -1;
    }
    double(*v)[3] = malloc(sizeof(double[3]) * (size_t)n);
    for (int k = 0; k < n; k++) {
        int a = k > 0 ? k - 1 : 0, b = k < n - 1 ? k + 1 : n - 1;
        for (int j = 0; j < 3; j++) {
            v[k][j] = (p[b][j] - p[a][j]) / (t[b] - t[a]);
        }
    }
    m->acc = malloc(sizeof(double[3]) * (size_t)n);
    for (int k = 0; k < n; k++) {
        for (int j = 0; j < 3; j++) {
            m->acc[k][j] = k < n - 1 ? (v[k + 1][j] - v[k][j]) / (t[k + 1] - t[k]) : 0.0;
        }
    }
    free(v);
    free(p);
    m->t = t;
    m->n = n;
    return 0;
}

static void imu_at(const imu_traj_t *m, double t, double acc[3])
{
    int lo = 0, hi = m->n - 1;
    if (t < m->t[0] || t >= m->t[m->n - 1]) {
        acc[0] = acc[1] = acc[2] = 0.0;
        return;
    }
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (m->t[mid] <= t) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    memcpy(acc, m->acc[lo], sizeof(double[3]));
}

/* The reference oscillator under acceleration: its fractional frequency error is gamma (per g) times
 * the specific force along the thrust axis (local up: these flights go straight up), linear between
 * the trajectory's interval midpoints, plus a vibration tone. */
typedef struct {
    int n;
    double *tm, *fa;       /* midpoints, and the specific force along up there (m/s^2) */
    double gamma;          /* per g */
    double vib_hz, vib_g, vib0, vib1;
} osc_model_t;

#define G0 9.80665

static void osc_build(osc_model_t *o, const imu_traj_t *m)
{
    o->n = m->n - 1;
    o->tm = malloc(sizeof(double) * (size_t)o->n);
    o->fa = malloc(sizeof(double) * (size_t)o->n);
    for (int k = 0; k < o->n; k++) {
        o->tm[k] = 0.5 * (m->t[k] + m->t[k + 1]);
        o->fa[k] = m->acc[k][0] * m->up[0] + m->acc[k][1] * m->up[1] + m->acc[k][2] * m->up[2] + G0;
    }
}

/* The specific force along up at t (m/s^2), and its rate (m/s^3). */
static double osc_force(const osc_model_t *o, double t, double *rate)
{
    *rate = 0.0;
    if (o->n < 2 || t <= o->tm[0]) {
        return o->n ? o->fa[0] : G0;
    }
    if (t >= o->tm[o->n - 1]) {
        return o->fa[o->n - 1];
    }
    int lo = 0, hi = o->n - 1;
    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        if (o->tm[mid] <= t) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    *rate = (o->fa[hi] - o->fa[lo]) / (o->tm[hi] - o->tm[lo]);
    return o->fa[lo] + *rate * (t - o->tm[lo]);
}

static double osc_eps(void *ctx, double t)
{
    const osc_model_t *o = (const osc_model_t *)ctx;
    double rate, f = osc_force(o, t, &rate);
    if (o->vib_g != 0.0 && t >= o->vib0 && t < o->vib1) {
        f += o->vib_g * G0 * sin(2.0 * PI * o->vib_hz * t);
    }
    return o->gamma * f / G0;
}

static void enu(const double x[3], const double ref[3], double lat, double lon, double out[3])
{
    double d[3] = {x[0] - ref[0], x[1] - ref[1], x[2] - ref[2]};
    double sl = sin(lat), cl = cos(lat), so = sin(lon), co = cos(lon);
    out[0] = -so * d[0] + co * d[1];
    out[1] = -sl * co * d[0] - sl * so * d[1] + cl * d[2];
    out[2] = cl * co * d[0] + cl * so * d[1] + sl * d[2];
}

/* The correlator in use, and the test-vector files. */
typedef struct {
    int golden;
    corr_model_t *cm;
    corr_float_t *cf;
    FILE *v_samples, *v_cmds, *v_dumps;
    uint64_t v_end;     /* vectors cover samples [0, v_end) */
    int half;           /* u2 packing: a nibble waiting for its partner */
    uint8_t half_val;
} bank_t;

static void apply(bank_t *b, uint64_t t_now, const corr_cmd_t *c)
{
    if (b->golden) {
        corr_model_command(b->cm, c);
    } else {
        corr_float_command(b->cf, c);
    }
    if (b->v_cmds && t_now < b->v_end) {
        fprintf(b->v_cmds, "%llu,%d,%d,%d,%llu,%llu,%llu,%d,%llu,%u,%d,%llu\n", (unsigned long long)t_now, c->type,
                c->ch, c->prn, (unsigned long long)c->t_start, (unsigned long long)c->code_phase,
                (unsigned long long)c->tap_offset, c->carr_word, (unsigned long long)c->code_word, c->apply_seq, c->sig,
                (unsigned long long)c->tap_offset2);
    }
}

static void vec_samples(bank_t *b, const uint8_t *codes, size_t n)
{
    for (size_t k = 0; k < n; k++) {
        if (b->half) {
            uint8_t byte = (uint8_t)((b->half_val << 4) | (codes[k] & 0xF));
            fwrite(&byte, 1, 1, b->v_samples);
            b->half = 0;
        } else {
            b->half_val = codes[k] & 0xF;
            b->half = 1;
        }
    }
}

static void vec_dumps(bank_t *b, const corr_dump_t *d, int nd)
{
    for (int k = 0; k < nd; k++) {
        if (d[k].t_samp > b->v_end) {
            continue;
        }
        fprintf(b->v_dumps, "%d,%u,%llu,%llu,%u,%u,%d,%llu,%u,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\n",
                d[k].ch, d[k].seq, (unsigned long long)d[k].t_samp, (unsigned long long)d[k].code_phase,
                d[k].carr_phase, d[k].carr_cycles, d[k].carr_word, (unsigned long long)d[k].code_word, d[k].flags,
                (double)d[k].ie, (double)d[k].qe, (double)d[k].ip, (double)d[k].qp, (double)d[k].il, (double)d[k].ql,
                (double)d[k].ive, (double)d[k].qve, (double)d[k].ivl, (double)d[k].qvl, (double)d[k].id,
                (double)d[k].qd);
    }
}

int main(int argc, char **argv)
{
    src_opts_t so;
    src_default_opts(&so);
    const char *out_dir = "runs/gnssrx", *corr_arg = NULL, *vec_dir = NULL, *truth_arg = NULL;
    double meas_hz = 10.0, vec_ms = 50.0, lut_amp = 0.0, p4_latency_us = 0.0;
    float acq_thr = -1.0f, acq_interval = -1.0f, hatch_s = -1.0f, slip_k = -1.0f, slip_tau = -1.0f;
    int no_iono = 0, no_tropo = 0, lut_bits = 0, no_raim = 0, pvt_unweighted = 0, preload_gps = 0, seed = 0;
    double seed_err_m = 0.0, seed_err_ms = 0.0, seed_sigma_ms = -1.0, seed_vel_sigma = 1.0, seed_pos_sigma = -1.0;
    double pvt_adapt_tau = -1.0;
    gps_iono_t iono_aid;
    memset(&iono_aid, 0, sizeof(iono_aid));
    const char *loops_quiet = NULL, *loops_boost = NULL, *nav_arg = NULL, *imu_path = NULL, *osc_traj = NULL;
    double osc_gamma_ppb = 0.0, osc_comp = 0.0, vib_hz = 0.0, vib_g = 0.0, vib0 = -1.0, vib1 = -1.0;
    int no_nav = 0, pilot_dumps = 0;
    double imu_lag_ms = 5.0, imu_sf = 0.03, imu_bias = 0.5, imu_noise = 0.1, imu_tilt = 0.0, imu0 = -INFINITY,
           imu1 = INFINITY;
    double boost0 = INFINITY, boost1 = -INFINITY;
    int boost_detect = 0, imu_no_aid = 0;
    double gate_ign = -1.0, gate_frac = 0.9, gate_ms = 10.0;
    boost_detect_cfg_t bdc;
    boost_detect_default(&bdc);
    for (int i = 1; i < argc; i++) {
        int r = src_parse_opt(&so, argc, argv, &i);
        if (r == 1) {
            continue;
        }
        if (r < 0) {
            usage();
            return 2;
        }
        const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
        if (!strcmp(a, "--out") && v) {
            out_dir = argv[++i];
        } else if (!strcmp(a, "--corr") && v) {
            corr_arg = argv[++i];
        } else if (!strcmp(a, "--lut-bits") && v) {
            lut_bits = atoi(argv[++i]);
        } else if (!strcmp(a, "--lut-amp") && v) {
            lut_amp = atof(argv[++i]);
        } else if (!strcmp(a, "--vectors") && v) {
            vec_dir = argv[++i];
        } else if (!strcmp(a, "--vectors-ms") && v) {
            vec_ms = atof(argv[++i]);
        } else if (!strcmp(a, "--p4-latency-us") && v) {
            p4_latency_us = atof(argv[++i]);
        } else if (!strcmp(a, "--meas-hz") && v) {
            meas_hz = atof(argv[++i]);
        } else if (!strcmp(a, "--acq-threshold") && v) {
            acq_thr = (float)atof(argv[++i]);
        } else if (!strcmp(a, "--acq-interval") && v) {
            acq_interval = (float)atof(argv[++i]);
        } else if (!strcmp(a, "--hatch") && v) {
            hatch_s = (float)atof(argv[++i]);
        } else if (!strcmp(a, "--hatch-slip") && v) {
            if (sscanf(argv[++i], "%f,%f", &slip_k, &slip_tau) < 1) {
                fprintf(stderr, "gnssrx: --hatch-slip takes K[,TAU_S]\n");
                return 2;
            }
        } else if (!strcmp(a, "--preload-gps")) {
            preload_gps = 1;
        } else if (!strcmp(a, "--prior") && v) {
            if (sscanf(argv[++i], "%lf,%lf,%lf,%lf,%lf", &seed_err_m, &seed_err_ms, &seed_sigma_ms, &seed_vel_sigma,
                       &seed_pos_sigma) < 2) {
                fprintf(stderr, "gnssrx: --prior takes POS_ERR_M,TIME_ERR_MS[,SIGMA_MS[,VEL_SIGMA[,POS_SIGMA]]]\n");
                return 2;
            }
            seed = 1;
        } else if (!strcmp(a, "--no-raim")) {
            no_raim = 1;
        } else if (!strcmp(a, "--pvt-unweighted")) {
            pvt_unweighted = 1;
        } else if (!strcmp(a, "--pvt-adapt-tau") && v) {
            pvt_adapt_tau = atof(argv[++i]);
        } else if (!strcmp(a, "--no-iono")) {
            no_iono = 1;
        } else if (!strcmp(a, "--no-tropo")) {
            no_tropo = 1;
        } else if (!strcmp(a, "--truth") && v) {
            truth_arg = argv[++i];
        } else if (!strcmp(a, "--nav") && v) {
            nav_arg = argv[++i];
        } else if (!strcmp(a, "--no-nav")) {
            no_nav = 1;
        } else if (!strcmp(a, "--pilot-dumps")) {
            pilot_dumps = 1;
        } else if (!strcmp(a, "--imu") && v) {
            imu_path = argv[++i];
        } else if (!strcmp(a, "--osc-g") && v) {
            if (sscanf(argv[++i], "%lf,%lf", &osc_gamma_ppb, &osc_comp) < 1) {
                fprintf(stderr, "gnssrx: --osc-g takes GAMMA_PPB_PER_G[,COMP]\n");
                return 2;
            }
        } else if (!strcmp(a, "--osc-vib") && v) {
            if (sscanf(argv[++i], "%lf,%lf,%lf,%lf", &vib_hz, &vib_g, &vib0, &vib1) < 2) {
                fprintf(stderr, "gnssrx: --osc-vib takes F_HZ,A_G[,S0,S1]\n");
                return 2;
            }
        } else if (!strcmp(a, "--osc-traj") && v) {
            osc_traj = argv[++i];
        } else if (!strcmp(a, "--imu-err") && v) {
            if (sscanf(argv[++i], "%lf,%lf,%lf,%lf,%lf", &imu_lag_ms, &imu_sf, &imu_bias, &imu_noise, &imu_tilt) < 4) {
                fprintf(stderr, "gnssrx: --imu-err takes LAG_MS,SF,BIAS,NOISE[,TILT_DEG]\n");
                return 2;
            }
        } else if (!strcmp(a, "--imu-at") && v) {
            if (sscanf(argv[++i], "%lf,%lf", &imu0, &imu1) != 2) {
                fprintf(stderr, "gnssrx: --imu-at takes S0,S1\n");
                return 2;
            }
        } else if (!strcmp(a, "--loops-quiet") && v) {
            loops_quiet = argv[++i];
        } else if (!strcmp(a, "--loops-boost") && v) {
            loops_boost = argv[++i];
        } else if (!strcmp(a, "--boost-at") && v) {
            if (sscanf(argv[++i], "%lf,%lf", &boost0, &boost1) != 2) {
                fprintf(stderr, "gnssrx: --boost-at takes S0,S1\n");
                return 2;
            }
        } else if (!strcmp(a, "--boost-detect") && v) {
            const char *bs = argv[++i];
            if (!strcmp(bs, "fc")) {
                boost_detect_fc(&bdc);
            } else if (strcmp(bs, "default") != 0) {
                unsigned lms = bdc.launch_ms, bms = bdc.burnout_ms, rms = bdc.rest_ms;
                if (sscanf(bs, "%f,%u,%u,%f,%f,%u", &bdc.launch_ms2, &lms, &bms, &bdc.hold_s, &bdc.rest_ms2, &rms) < 4) {
                    fprintf(stderr, "gnssrx: --boost-detect takes default, fc or "
                                    "LAUNCH_MS2,LAUNCH_MS,BURNOUT_MS,HOLD_S[,REST_MS2,REST_MS]\n");
                    return 2;
                }
                bdc.launch_ms = (uint16_t)lms;
                bdc.burnout_ms = (uint16_t)bms;
                bdc.rest_ms = (uint16_t)rms;
            }
            boost_detect = 1;
        } else if (!strcmp(a, "--boost-gate") && v) {
            if (sscanf(argv[++i], "%lf,%lf,%lf", &gate_ign, &gate_frac, &gate_ms) != 3) {
                fprintf(stderr, "gnssrx: --boost-gate takes IGN_S,TAIL_FRAC,TAIL_MS\n");
                return 2;
            }
        } else if (!strcmp(a, "--imu-no-aid")) {
            imu_no_aid = 1;
        } else if (!strcmp(a, "--iono") && v) {
            double *al = iono_aid.alpha, *be = iono_aid.beta;
            if (sscanf(argv[++i], "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &al[0], &al[1], &al[2], &al[3], &be[0], &be[1],
                       &be[2], &be[3]) != 8) {
                fprintf(stderr, "gnssrx: --iono takes eight comma-separated values\n");
                return 2;
            }
            iono_aid.valid = 1;
        } else {
            fprintf(stderr, "gnssrx: unknown option %s\n", a);
            usage();
            return 2;
        }
    }
    if (!so.file) {
        usage();
        return 2;
    }
    /* The oscillator's g-sensitivity: the front end turns everything by its phase. */
    osc_model_t osc;
    memset(&osc, 0, sizeof(osc));
    /* COMP -1: the P4's estimate, by least squares of the fixes' clock drift on the IMU's specific
     * force, both less their pad values (averaged until ignition shows as 2 g over them). */
    double est_pad_f = 0.0, est_pad_d = 0.0, est_sff = 0.0, est_sfd = 0.0, est_gamma = 0.0;
    long est_npad = 0, est_n = 0;
    if (osc_gamma_ppb != 0.0 || vib_g != 0.0) {
        const char *tp = osc_traj ? osc_traj : imu_path;
        imu_traj_t ot;
        memset(&ot, 0, sizeof(ot));
        if (!tp || imu_load(tp, &ot) != 0) {
            fprintf(stderr, "gnssrx: --osc-g and --osc-vib need a trajectory (--osc-traj or --imu)\n");
            return 2;
        }
        osc_build(&osc, &ot);
        free(ot.t);
        free(ot.acc);
        osc.gamma = osc_gamma_ppb * 1e-9;
        osc.vib_hz = vib_hz;
        osc.vib_g = vib_g;
        osc.vib0 = vib0 < vib1 ? vib0 : (boost0 < boost1 ? boost0 : -INFINITY);
        osc.vib1 = vib0 < vib1 ? vib1 : (boost0 < boost1 ? boost1 : INFINITY);
        so.fe.osc_eps = osc_eps;
        so.fe.osc_ctx = &osc;
    }
    src_t src;
    if (src_open(&src, &so) != 0) {
        return 1;
    }
    const int two_bit = so.fe.mode != FE_MODE_NATIVE;
    bank_t bank;
    memset(&bank, 0, sizeof(bank));
    bank.golden = two_bit && !(corr_arg && !strcmp(corr_arg, "float"));
    if (corr_arg && !strcmp(corr_arg, "golden") && !two_bit) {
        fprintf(stderr, "gnssrx: the golden model takes 2-bit streams only\n");
        return 2;
    }
    const double fs = src.fs_out;
    const uint64_t spms = (uint64_t)llround(fs / 1000.0);
    if (fabs(fs / 1000.0 - (double)spms) > 1e-6) {
        fprintf(stderr, "gnssrx: the stream rate must be a whole number of samples per ms\n");
        return 1;
    }
    if (gate_ign >= 0.0) {
        if (!boost_detect) {
            fprintf(stderr, "gnssrx: --boost-gate needs --boost-detect\n");
            return 2;
        }
        boost_detect_gate(&bdc, (float)gate_ign, (float)gate_frac, (uint16_t)gate_ms);
    }
    if (boost_detect && !imu_path) {
        fprintf(stderr, "gnssrx: --boost-detect needs --imu, the trajectory the IMU is emulated from "
                        "(with --imu-no-aid for loops without the feed-forward)\n");
        return 2;
    }
    if (boost_detect && boost0 < boost1) {
        fprintf(stderr, "gnssrx: --boost-detect and --boost-at are alternatives\n");
        return 2;
    }
    if (mkdir(out_dir, 0755) != 0 && errno != EEXIST) {
        fprintf(stderr, "gnssrx: cannot create %s\n", out_dir);
        return 1;
    }

    rx_t *rx = (rx_t *)malloc(sizeof(rx_t));
    if (!rx) {
        return 1;
    }
    corr_model_cfg_t mcfg;
    corr_model_default_cfg(&mcfg);
    if (bank.golden) {
        if (lut_bits > 0 && corr_model_lut(&mcfg, lut_bits, lut_amp > 0.0 ? lut_amp : 1.0) != 0) {
            fprintf(stderr, "gnssrx: --lut-bits must be 1..%d\n", CM_MAX_LUT_BITS);
            return 2;
        }
        bank.cm = (corr_model_t *)malloc(sizeof(corr_model_t));
        if (!bank.cm) {
            return 1;
        }
        corr_model_init(bank.cm, &mcfg);
    } else {
        bank.cf = (corr_float_t *)malloc(sizeof(corr_float_t));
        if (!bank.cf) {
            return 1;
        }
        corr_float_init(bank.cf, fs);
    }
    if (vec_dir) {
        if (!bank.golden) {
            fprintf(stderr, "gnssrx: test vectors come from the golden model only\n");
            return 2;
        }
        if (mkdir(vec_dir, 0755) != 0 && errno != EEXIST) {
            fprintf(stderr, "gnssrx: cannot create %s\n", vec_dir);
            return 1;
        }
        char p[4200];
        snprintf(p, sizeof(p), "%s/samples.u2", vec_dir);
        bank.v_samples = fopen(p, "wb");
        bank.v_cmds = open_csv(vec_dir, "commands.csv",
                               "t_sample,type,ch,prn,t_start,code_phase,tap_offset,carr_word,code_word,apply_seq,sig,"
                               "tap_offset2");
        bank.v_dumps = open_csv(vec_dir, "dumps.csv",
                                "ch,seq,t_samp,code_phase,carr_phase,carr_cycles,carr_word,code_word,flags,ie,qe,ip,qp,"
                                "il,ql,ive,qve,ivl,qvl,id,qd");
        bank.v_end = (uint64_t)llround(vec_ms * 1e-3 * fs);
        if (!bank.v_samples || !bank.v_cmds || !bank.v_dumps) {
            fprintf(stderr, "gnssrx: cannot write vectors into %s\n", vec_dir);
            return 1;
        }
        /* With the notch stage, its own vectors too (the correlators' samples.u2 is its output). */
        if (so.fe.mit.type == MIT_NOTCH &&
            mit_set_vectors(&src.fe.mit, vec_dir, bank.v_end, (uint64_t)llround(1e-3 * fs)) != 0) {
            fprintf(stderr, "gnssrx: cannot write the notch's vectors into %s\n", vec_dir);
            return 1;
        }
    }

    rx_cfg_t rc;
    rx_default_cfg(&rc, fs, src.if_out);
    if (acq_thr > 0.0f) {
        rc.acq_threshold = acq_thr;
    }
    if (hatch_s >= 0.0f) {
        rc.hatch_s = hatch_s;
    }
    if (slip_k >= 0.0f) {
        rc.slip_k = slip_k;
    }
    if (slip_tau > 0.0f) {
        rc.slip_tau_s = slip_tau;
    }
    if (acq_interval > 0.0f) {
        rc.acq_interval_s = acq_interval;
    }
    if ((loops_quiet && loops_arg_parse(loops_quiet, &rc.quiet) != 0) ||
        (loops_boost && loops_arg_parse(loops_boost, &rc.boost) != 0)) {
        fprintf(stderr, "gnssrx: a loop profile is PF,PP,PD/LF,LP,LD[:MS]\n");
        return 2;
    }
    rc.pvt_weights = !pvt_unweighted;
    if (pvt_adapt_tau >= 0.0) {
        rc.adapt_tau_s = (float)pvt_adapt_tau;
    }
    rx_init(rx, &rc);
    /* The generator's own ionosphere and troposphere, from the manifest (page 18 repeats only every
     * 12.5 min, so short files never deliver it); the options override. */
    if (!iono_aid.valid && src.meta.have_iono) {
        memcpy(iono_aid.alpha, src.meta.iono, sizeof(iono_aid.alpha));
        memcpy(iono_aid.beta, src.meta.iono + 4, sizeof(iono_aid.beta));
        iono_aid.valid = 1;
    }
    if (iono_aid.valid) {
        rx->iono = iono_aid;
    }
    rx->pvt_opt.use_iono = !no_iono;
    rx->pvt_opt.raim = !no_raim;
    {
        /* The file's date settles the navigation message's 10-bit week (a 2015 recording reads
         * 1024 weeks late otherwise). */
        double t0;
        if (gpst_parse(src.meta.start_gpst, &t0) == 0) {
            rx_set_week_ref(rx, (int)floor(t0 / 604800.0));
        }
    }
    if (seed) {
        double t0, pos[3], la, lo;
        if (gpst_parse(src.meta.start_gpst, &t0) != 0 || !static_truth(src.meta.truth, pos, &la, &lo)) {
            fprintf(stderr, "gnssrx: --prior needs the manifest's start_gpst and a static truth\n");
            return 2;
        }
        pos[0] += -sin(lo) * seed_err_m;  /* east */
        pos[1] += cos(lo) * seed_err_m;
        const double t = t0 + so.start_s + 1e-3 * seed_err_ms;
        const int wk = (int)floor(t / 604800.0);
        if (seed_sigma_ms < 0.0) {
            seed_sigma_ms = fabs(seed_err_ms) > 0.001 ? fabs(seed_err_ms) : 0.001;
        }
        if (seed_pos_sigma < 0.0) {
            seed_pos_sigma = fabs(seed_err_m) > 100.0 ? fabs(seed_err_m) : 100.0;
        }
        rx_set_seed(rx, pos, seed_pos_sigma, wk, t - wk * 604800.0, 1e-3 * seed_sigma_ms, 0);
        const double v0[3] = {0.0, 0.0, 0.0};
        rx_set_seed_vel(rx, v0, seed_vel_sigma);
        printf("gnssrx: seeded with the truth %.0f m east (claimed good to %.0f m, at rest to %.0f m/s) and the time "
               "%+.3f ms out (claimed good to %.3f ms%s)\n",
               seed_err_m, seed_pos_sigma, seed_vel_sigma, seed_err_ms, seed_sigma_ms,
               rx->time_coarse ? ": coarse time" : "");
    }
    rx->pvt_opt.use_tropo = !no_tropo && !src.meta.tropo_none;
    /* Galileo and BeiDou ephemerides from the generator's RINEX, as the flight computer could
     * preload them: the records nearest the run's middle, from the manifest's start_gpst. */
    const char *nav = no_nav ? NULL : (nav_arg ? nav_arg : (src.meta.nav[0] ? src.meta.nav : NULL));
    if (nav) {
        char nav_path[4096];
        double t0;
        if (manifest_resolve_iq(nav, nav_path, sizeof(nav_path)) != 0 || gpst_parse(src.meta.start_gpst, &t0) != 0) {
            fprintf(stderr, "gnssrx: cannot preload %s (needs the file and the manifest's start_gpst)\n", nav);
            return 1;
        }
        /* GPS week and second of the run's middle. */
        double t_mid = t0 + so.start_s + (so.dur_s > 0 ? 0.5 * so.dur_s : 0.0);
        int week = (int)floor(t_mid / 604800.0);
        const unsigned mask = (1u << GNSS_SYS_GAL) | (1u << GNSS_SYS_BDS) | (preload_gps ? 1u << GNSS_SYS_GPS : 0u);
        int n_eph = rinex_nav_load(nav_path, mask, week, t_mid - week * 604800.0, rx->eph, NULL);
        printf("gnssrx: preloaded %d %s ephemerides from %s\n", n_eph,
               preload_gps ? "GPS/Galileo/BeiDou" : "Galileo/BeiDou", nav);
    }

    const int acq_ms = rc.acq_ms;
    float *ring = (float *)malloc(sizeof(float) * 2 * spms * (size_t)acq_ms);
    float *snap = (float *)malloc(sizeof(float) * 2 * spms * (size_t)acq_ms);
    float *work = (float *)malloc(sizeof(float) * acq_work_floats(2048, acq_ms));
    float *blk = (float *)malloc(sizeof(float) * 2 * spms);
    uint8_t *cblk = (uint8_t *)malloc(spms);
    if (!ring || !snap || !work || !blk || !cblk) {
        return 1;
    }
    int ring_head = 0, ring_count = 0;

    FILE *ftrk = open_csv(out_dir, "trk.csv", "t_s,ch,prn,state,cn0,dop_hz,pll_lock,bit_sync,frame_sync,subframes,parity_fail");
    FILE *fobs = open_csv(out_dir, "obs.csv",
                          "t_s,rx_tow,prn,pr_m,adr_cyc,dop_hz,cn0,lock_s,el_deg,resid_m,pr_raw_m,excl");
    FILE *fpvt = open_csv(out_dir, "pvt.csv",
                          "t_s,rx_tow,week,lat_deg,lon_deg,h_m,x,y,z,vx,vy,vz,clk_bias_m,clk_drift_mps,nsat,pdop,"
                          "resid_rms,e_m,n_m,u_m,nexcl,chi2,chi2_lim,vel_valid,coarse,time_off_ms,dof,vdof,jam");
    FILE *feph = open_csv(out_dir, "eph.csv",
                          "t_s,prn,week,toe,toc,iode,iodc,health,sqrt_a,e,i0,omega0,omega,m0,delta_n,idot,"
                          "omega_dot,cuc,cus,crc,crs,cic,cis,af0,af1,af2,tgd");
    FILE *fpd = pilot_dumps ? open_csv(out_dir, "pilot_dumps.csv", "ch,sat,seq,t_samp,n1,sec_len,ip,qp,id,qd") : NULL;
    FILE *fimu = NULL;  /* opened with the trajectory below */
    imu_traj_t imu;
    memset(&imu, 0, sizeof(imu));
    rng_t imu_rng;
    rng_seed(&imu_rng, 77);
    boost_detect_t bd;
    boost_detect_init(&bd, &bdc);
    double bd_on[4], bd_burnout[4], bd_off[4];  /* what it detected, file seconds */
    int bd_n = 0, bd_prev = 0;
    boost_phase_t bd_phase_prev = BOOST_PAD;
    if (imu_path && imu_load(imu_path, &imu) != 0) {
        fprintf(stderr, "gnssrx: cannot read the trajectory %s\n", imu_path);
        return 1;
    }
    if (imu.n > 0) {
        /* What the aiding was given, at 10 Hz: the true and the emulated acceleration along local up. */
        fimu = open_csv(out_dir, "imu.csv", "t_s,acc_up_true,acc_up_imu,aiding,boost");
    }
    if (!ftrk || !fobs || !fpvt || !feph) {
        fprintf(stderr, "gnssrx: cannot write into %s\n", out_dir);
        return 1;
    }
    double ref[3], rlat = 0.0, rlon = 0.0;
    int have_truth = static_truth(truth_arg ? truth_arg : src.meta.truth, ref, &rlat, &rlon);

    printf("gnssrx: %s from %.1f s, %.3f MS/s, IF %+.3f MHz, %s, %s correlator", src.path, so.start_s, fs / 1e6,
           src.if_out / 1e6, two_bit ? "2-bit" : "native", bank.golden ? "golden" : "float");
    if (bank.golden) {
        printf(" (%d carrier sectors, cos levels", 1 << mcfg.lut_bits);
        for (int k = 0; k < (1 << mcfg.lut_bits) / 4 + ((1 << mcfg.lut_bits) < 4); k++) {
            printf(" %d", mcfg.cos_lut[k]);
        }
        printf(")");
    }
    printf(" -> %s\n", out_dir);
    FILE *fini = open_csv(out_dir, "run.ini", NULL);
    if (fini) {
        /* What made this run, for the analysis scripts (py/boost_track.py) and for the record. */
        char q[96], b[96];
        loops_arg_format(&rc.quiet, q, sizeof(q));
        loops_arg_format(&rc.boost, b, sizeof(b));
        fprintf(fini, "[run]\nsource = %s\nstart_s = %.6f\ndur_s = %.6f\nfs = %.3f\nif_hz = %.3f\nmode = %s\n"
                      "corr = %s\n",
                src.path, so.start_s, so.dur_s, fs, src.if_out,
                so.fe.mode == FE_MODE_ADC27 ? "adc27" : (so.fe.mode == FE_MODE_NATIVE ? "native" : "direct"),
                bank.golden ? "golden" : "float");
        if (so.have_cn0) {
            fprintf(fini, "cn0 = %.2f\n", so.cn0);
        }
        for (int k = 0; k < so.nsteps; k++) {
            fprintf(fini, "cn0_at = %.3f:%.2f\n", so.step_t[k], so.step_cn0[k]);
        }
        fprintf(fini, "p4_latency_us = %.1f\ncmd_lead = %u\nloops_quiet = %s\nloops_boost = %s\nhatch_s = %g\n",
                p4_latency_us, rc.cmd_lead, q, b, (double)rc.hatch_s);
        fprintf(fini, "hatch_slip = %g,%g\n", (double)rc.slip_k, (double)rc.slip_tau_s);
        if (seed) {
            fprintf(fini, "prior = %g,%g,%g,%g,%g\n", seed_err_m, seed_err_ms, seed_sigma_ms, seed_vel_sigma,
                    seed_pos_sigma);
        }
        if (boost0 < boost1) {
            fprintf(fini, "boost_at = %.3f,%.3f\n", boost0, boost1);
        }
        if (imu_path) {
            fprintf(fini, "imu = %s\nimu_err = %g,%g,%g,%g,%g\n", imu_path, imu_lag_ms, imu_sf, imu_bias, imu_noise,
                    imu_tilt);
        }
        if (imu_no_aid) {
            fprintf(fini, "imu_aid = 0\n");
        }
        if (boost_detect) {
            fprintf(fini, "boost_detect = %g,%u,%u,%u,%g,%g,%u\n", (double)bdc.launch_ms2, bdc.launch_ms,
                    bdc.lockout_ms, bdc.burnout_ms, (double)bdc.hold_s, (double)bdc.rest_ms2, bdc.rest_ms);
            if (bdc.gate) {
                fprintf(fini, "boost_gate = %g,%g,%u\n", (double)bdc.ign_s, (double)bdc.tail_frac, bdc.tail_ms);
            }
        }
        if (osc.n > 0) {
            fprintf(fini, "osc_g = %g,%g\nosc_vib = %g,%g,%g,%g\n", osc_gamma_ppb, osc_comp, vib_hz, vib_g, osc.vib0,
                    osc.vib1);
        }
        for (int k = 0; k < so.fe.n_jam; k++) {
            const jam_cfg_t *j = &so.fe.jam[k];
            fprintf(fini, "jam = %s,%.1f,%.1f,%g,%g,%g\n", j->type == JAM_CW ? "cw" : (j->type == JAM_NB ? "nb" : "chirp"),
                    j->f_hz, j->jnr_db, j->bw_hz, j->period_s, so.fe.jam_t0_s);
        }
        if (so.fe.mit.type != MIT_NONE) {
            fprintf(fini, "mitig = %s,%d,%g,%g,%d,%g,%g\n", so.fe.mit.type == MIT_ANF ? "anf" : (so.fe.mit.type == MIT_ANFQ ? "anfq" : (so.fe.mit.type == MIT_NOTCH ? "notch" : "fde")), so.fe.mit.n_notch,
                    so.fe.mit.anf_k, so.fe.mit.anf_mu, so.fe.mit.fde_n, so.fe.mit.fde_k, so.fe.mit.fde_tau_s);
        }
        fclose(fini);
    }

    corr_dump_t dumps[MAX_DUMPS];
    corr_cmd_t cmds[MAX_CMDS], held[2 * MAX_CMDS];
    int nheld = 0;
    const uint64_t lat = (uint64_t)llround(p4_latency_us * 1e-6 * fs);
    if (lat >= spms) {
        fprintf(stderr, "gnssrx: --p4-latency-us must be under one tick (1000 us)\n");
        return 2;
    }
    rx_obs_t obs[CORR_MAX_CH];
    uint64_t t0 = 0;
    const uint64_t meas_step = (uint64_t)llround(fs / meas_hz);
    double eph_toe[GPS_MAX_PRN + 1];
    for (int p = 0; p <= GPS_MAX_PRN; p++) {
        eph_toe[p] = -1.0;
    }
    double wall0 = now_s();
    double sum_e[3] = {0}, sum_e2[3] = {0};
    long n_fix = 0, n_withheld = 0, n_excl_fix = 0, n_vel_fail = 0, n_coarse_fix = 0, n_jam_epochs = 0;
    double sup_max = 0.0;
    double t_first_fix = 0.0;
    int t_first_fix_set = 0;
    for (;;) {
        if (two_bit) {
            if (src_read_codes(&src, cblk, spms) < spms) {
                break;
            }
            for (uint64_t k = 0; k < spms; k++) {
                unsigned x = cblk[k];
                float wi = (x & FE_CODE_I_MAG) ? (float)FE_WEIGHT_LARGE : (float)FE_WEIGHT_SMALL;
                float wq = (x & FE_CODE_Q_MAG) ? (float)FE_WEIGHT_LARGE : (float)FE_WEIGHT_SMALL;
                blk[2 * k] = (x & FE_CODE_I_SIGN) ? -wi : wi;
                blk[2 * k + 1] = (x & FE_CODE_Q_SIGN) ? -wq : wq;
            }
        } else if (src_read(&src, blk, spms) < spms) {
            break;
        }
        memcpy(ring + 2 * spms * (size_t)ring_head, blk, sizeof(float) * 2 * spms);
        ring_head = (ring_head + 1) % acq_ms;
        ring_count = ring_count < acq_ms ? ring_count + 1 : acq_ms;

        /* Commands from the last tick reach the correlator `lat` samples into this block. */
        int nd = 0;
        uint64_t split = nheld > 0 ? lat : 0;
        if (split > 0) {
            nd += bank.golden ? corr_model_process(bank.cm, t0, cblk, (size_t)split, dumps, MAX_DUMPS)
                              : corr_float_process(bank.cf, t0, blk, (size_t)split, dumps, MAX_DUMPS);
        }
        for (int k = 0; k < nheld; k++) {
            apply(&bank, t0 + split, &held[k]);
        }
        nheld = 0;
        nd += bank.golden
                  ? corr_model_process(bank.cm, t0 + split, cblk + split, (size_t)(spms - split), dumps + nd,
                                       MAX_DUMPS - nd)
                  : corr_float_process(bank.cf, t0 + split, blk + 2 * split, (size_t)(spms - split), dumps + nd,
                                       MAX_DUMPS - nd);
        if (bank.v_samples && t0 < bank.v_end) {
            uint64_t m = bank.v_end - t0 < spms ? bank.v_end - t0 : spms;
            vec_samples(&bank, cblk, (size_t)m);
            vec_dumps(&bank, dumps, nd);
        }
        uint64_t t_now = t0 + spms;
        if (fpd) {
            for (int k = 0; k < nd; k++) {
                const rx_nco_t *pn = &rx->nco[dumps[k].ch];
                if (pn->sec_len > 0) {
                    fprintf(fpd, "%d,%d,%u,%llu,%lld,%u,%.0f,%.0f,%.0f,%.0f\n", dumps[k].ch, rx->ch[dumps[k].ch].prn +
                            100 * (rx->ch[dumps[k].ch].sig == GNSS_SIG_GAL_E1C ? 1 : 2), dumps[k].seq,
                            (unsigned long long)dumps[k].t_samp, (long long)pn->n1, pn->sec_len, (double)dumps[k].ip,
                            (double)dumps[k].qp, (double)dumps[k].id, (double)dumps[k].qd);
                }
            }
        }
        const double t_file = so.start_s + (double)t_now / fs;
        int boost_on = t_file >= boost0 && t_file < boost1;
        if (imu.n > 0) {
            /* The IMU as the P4 would see it: imu_lag_ms late, scaled, biased along local up, noisy. */
            int on = t_file >= imu0 && t_file < imu1;
            double a[3], nz[4];
            imu_at(&imu, t_file - 1e-3 * imu_lag_ms, a);
            rng_gauss2(&imu_rng, &nz[0], &nz[1]);
            rng_gauss2(&imu_rng, &nz[2], &nz[3]);
            if (imu_tilt != 0.0) {
                /* An attitude error: the acceleration turned by imu_tilt in the up-east plane. */
                const double east[3] = {-imu.up[1] / hypot(imu.up[0], imu.up[1]), imu.up[0] / hypot(imu.up[0], imu.up[1]),
                                        0.0};
                const double au = a[0] * imu.up[0] + a[1] * imu.up[1] + a[2] * imu.up[2];
                const double ae = a[0] * east[0] + a[1] * east[1];
                const double c = cos(imu_tilt * PI / 180.0), sn = sin(imu_tilt * PI / 180.0);
                const double du = au * c - ae * sn - au, de = au * sn + ae * c - ae;
                for (int j = 0; j < 3; j++) {
                    a[j] += du * imu.up[j] + de * east[j];
                }
            }
            for (int j = 0; j < 3; j++) {
                a[j] = a[j] * (1.0 + imu_sf) + imu_bias * imu.up[j] + imu_noise * nz[j];
            }
            if (boost_detect) {
                /* The P4's launch and burnout detection on the same samples: the specific force along the
                 * thrust axis (these flights go straight up, so the pad's up). */
                const double f_ax = a[0] * imu.up[0] + a[1] * imu.up[1] + a[2] * imu.up[2] + G0;
                boost_on = boost_detect_step(&bd, (float)f_ax);
                if (boost_on && !bd_prev && bd_n < 4) {
                    bd_on[bd_n] = t_file;
                    bd_burnout[bd_n] = bd_off[bd_n] = NAN;
                    bd_n++;
                }
                if (bd.phase == BOOST_HOLD && bd_phase_prev == BOOST_BURN && bd_n > 0) {
                    bd_burnout[bd_n - 1] = t_file;
                }
                if (!boost_on && bd_prev && bd_n > 0) {
                    bd_off[bd_n - 1] = t_file;
                }
                bd_prev = boost_on;
                bd_phase_prev = bd.phase;
            }
            rx_set_accel(rx, a, on && !imu_no_aid);
            if (fimu && (t_now / spms) % 100 == 0) {
                double at[3];
                imu_at(&imu, t_file, at);
                fprintf(fimu, "%.3f,%.4f,%.4f,%d,%d\n", (double)t_now / fs,
                        at[0] * imu.up[0] + at[1] * imu.up[1] + at[2] * imu.up[2],
                        a[0] * imu.up[0] + a[1] * imu.up[1] + a[2] * imu.up[2], on && !imu_no_aid, boost_on);
            }
        }
        rx_set_boost(rx, boost_on);
        if (osc.n > 0 && osc_comp != 0.0) {
            /* The P4 feeds the oscillator forward from the IMU's specific force (as late and as
             * scaled as the IMU) through its sensitivity: osc_comp of the true one, or its own
             * estimate once it has one (osc_comp -1). */
            double rate;
            osc_force(&osc, t_file - 1e-3 * imu_lag_ms, &rate);
            const double gam = osc_comp > 0.0 ? osc_comp * osc.gamma : est_gamma;
            rx_set_clock_rate(rx, -GNSS_FREQ_L1_HZ * gam * rate * (1.0 + imu_sf) / G0, gam != 0.0);
        }
        int nc = rx_tick(rx, t_now, dumps, nd, cmds, MAX_CMDS);
        for (int k = 0; k < nc && nheld < 2 * MAX_CMDS; k++) {
            held[nheld++] = cmds[k];
        }
        /* Galileo and BeiDou channels from the GPS fix and the preloaded ephemerides. */
        nc = rx_aid(rx, t_now, cmds, MAX_CMDS);
        for (int k = 0; k < nc; k++) {
            if (nheld < 2 * MAX_CMDS) {
                held[nheld++] = cmds[k];
            }
            if (cmds[k].type == CORR_CMD_START) {
                printf("  %7.3f s  ch %2d  %c%02d    aided start, Doppler %+7.1f Hz\n", (double)t_now / fs, cmds[k].ch,
                       cmds[k].sig == GNSS_SIG_GAL_E1C ? 'E' : 'C', cmds[k].prn, (double)rx->ch[cmds[k].ch].dop_hz);
            }
        }
        int ms;
        if (rx_wants_snapshot(rx, t_now, &ms) && ring_count >= ms) {
            for (int b = 0; b < ms; b++) {
                int slot = (ring_head + acq_ms - ms + b) % acq_ms;
                memcpy(snap + 2 * spms * (size_t)b, ring + 2 * spms * (size_t)slot, sizeof(float) * 2 * spms);
            }
            uint64_t t_snap = t_now - spms * (uint64_t)ms;
            nc = rx_acquire(rx, t_now, t_snap, snap, spms * (size_t)ms, work, cmds, MAX_CMDS);
            for (int k = 0; k < nc; k++) {
                if (nheld < 2 * MAX_CMDS) {
                    held[nheld++] = cmds[k];
                }
                if (cmds[k].type == CORR_CMD_START) {
                    printf("  %7.3f s  ch %2d  PRN %2d  acquired, Doppler %+7.1f Hz, metric %.1f\n", (double)t_now / fs,
                           cmds[k].ch, cmds[k].prn, (double)rx->ch[cmds[k].ch].dop_hz,
                           (double)rx->ch[cmds[k].ch].acq_metric);
                }
            }
        }
        if (t_now % meas_step == 0) {
            double ts = (double)t_now / fs;
            pvt_sol_t sol;
            if (so.fe.mit.type != MIT_NONE) {
                /* The stage's power in over power out is the interference flag (0.5 dB: a tone ~10 dB
                 * under the noise, where unmitigated channels start to go false). */
                const double sup = mit_take_suppression_db(&src.fe.mit);
                rx_set_interference(rx, sup > 0.5);
                if (sup > sup_max) {
                    sup_max = sup;
                }
                n_jam_epochs += rx->interference;
            }
            int no = rx_measure(rx, t_now, obs, CORR_MAX_CH, &sol);
            const int coarse_fix = rx->time_coarse;  /* as the solve had it (an anchor comes first) */
            if (osc.n > 0 && osc_comp < 0.0 && sol.valid && sol.vel_valid) {
                double r;
                const double tfx = so.start_s + ts;
                const double f = osc_force(&osc, tfx - 1e-3 * imu_lag_ms, &r) * (1.0 + imu_sf);
                if (est_n == 0 && (est_npad == 0 || f - est_pad_f / (double)est_npad < 2.0 * G0)) {
                    est_pad_f += f;  /* on the pad */
                    est_pad_d += sol.clk_drift;
                    est_npad++;
                } else if (est_npad > 0) {
                    const double df = f - est_pad_f / (double)est_npad, dd = sol.clk_drift - est_pad_d / (double)est_npad;
                    est_sff += df * df;
                    est_sfd += df * dd;
                    est_n++;
                    if (est_n >= 5 && est_sff > 5.0 * (3.0 * G0) * (3.0 * G0)) {
                        est_gamma = est_sfd / est_sff * G0 / GNSS_C;  /* per g */
                    }
                }
            }
            double rtow = rx->clk_valid ? rx_time(rx, t_now) : 0.0;
            for (int k = 0; k < no && rx->clk_valid; k++) {
                /* prn: 100 x system + PRN (GPS 0, Galileo 1, BeiDou 2), so GPS rows read as before. */
                fprintf(fobs, "%.3f,%.9f,%d,%.4f,%.4f,%.4f,%.2f,%.2f,%.3f,%.4f,%.4f,%d\n", ts, rtow,
                        100 * obs[k].sys + obs[k].prn, obs[k].pr, obs[k].adr, obs[k].dop, (double)obs[k].cn0,
                        (double)obs[k].lock_s, sol.valid ? sol.el[k] * 180.0 / PI : 0.0,
                        sol.valid && sol.used[k] ? sol.resid[k] : 0.0, obs[k].pr_raw, sol.excluded[k]);
            }
            if (sol.valid) {
                double e[3] = {0, 0, 0};
                if (have_truth) {
                    enu(sol.pos, ref, rlat, rlon, e);
                    for (int j = 0; j < 3; j++) {
                        sum_e[j] += e[j];
                        sum_e2[j] += e[j] * e[j];
                    }
                    n_fix++;
                }
                fprintf(fpvt, "%.3f,%.9f,%d,%.9f,%.9f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.3f,%.4f,%.4f,%.4f,%.4f,"
                              "%d,%.2f,%.2f,%d,%d,%.4f,%d,%d,%d\n",
                        ts, rx_time(rx, t_now), rx->week, sol.lat * 180.0 / PI, sol.lon * 180.0 / PI, sol.h,
                        sol.pos[0], sol.pos[1], sol.pos[2], sol.vel[0], sol.vel[1], sol.vel[2], sol.clk_bias,
                        sol.clk_drift, sol.nsat, sol.pdop, sol.resid_rms, e[0], e[1], e[2], sol.nexcl, sol.chi2,
                        sol.chi2_lim, sol.vel_valid, coarse_fix, 1e3 * sol.time_offset, sol.dof, sol.vdof,
                        rx->interference);
                n_coarse_fix += coarse_fix;
                if (!t_first_fix_set) {
                    t_first_fix = ts;
                    t_first_fix_set = 1;
                }
                n_excl_fix += sol.nexcl > 0;
                n_vel_fail += !sol.vel_valid;
            } else if (sol.chi2_lim > 0.0 && sol.chi2 > sol.chi2_lim) {
                n_withheld++;  /* the residual test failed and nothing could be left out to mend it */
            }
            for (int ch = 0; ch < rc.max_ch; ch++) {
                const trk_ch_t *c = &rx->ch[ch];
                if (c->prn == 0) {
                    continue;
                }
                const int sys = c->sig == GNSS_SIG_GAL_E1C ? 1 : (c->sig == GNSS_SIG_BDS_B1CP ? 2 : 0);
                fprintf(ftrk, "%.3f,%d,%d,%d,%.2f,%.3f,%.3f,%d,%d,%u,%u\n", ts, ch, 100 * sys + c->prn, c->state,
                        (double)c->cn0,
                        (double)c->dop_hz, (double)c->pll_lock, c->bit_sync, rx->nav[ch].synced,
                        rx->nav[ch].n_subframes, rx->nav[ch].n_parity_fail);
            }
            for (int p = 1; p <= GPS_MAX_PRN; p++) {
                const gps_eph_t *e = &rx->eph[GNSS_SYS_GPS][p];
                if (e->valid && e->toe != eph_toe[p]) {
                    eph_toe[p] = e->toe;
                    fprintf(feph,
                            "%.3f,%d,%d,%.0f,%.0f,%d,%d,%d,%.10f,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e,"
                            "%.12e,%.12e,%.6f,%.6f,%.12e,%.12e,%.12e,%.12e,%.12e,%.12e\n",
                            ts, p, e->week, e->toe, e->toc, e->iode, e->iodc, e->health, e->sqrt_a, e->e, e->i0,
                            e->omega0, e->omega, e->m0, e->delta_n, e->idot, e->omega_dot, e->cuc, e->cus, e->crc,
                            e->crs, e->cic, e->cis, e->af0, e->af1, e->af2, e->tgd);
                }
            }
            if (t_now % (uint64_t)(10.0 * fs) == 0) {
                int locked = 0, sync = 0;
                for (int ch = 0; ch < rc.max_ch; ch++) {
                    locked += rx->ch[ch].state == TRK_LOCKED;
                    sync += rx->nav[ch].synced;
                }
                printf("  %7.1f s  locked %2d  frame-synced %2d  obs %2d", ts, locked, sync, no);
                if (sol.valid) {
                    printf("  fix %d sats, pdop %.1f, resid %.2f m", sol.nsat, sol.pdop, sol.resid_rms);
                    if (sol.isb[GNSS_SYS_GAL] != 0.0 || sol.isb[GNSS_SYS_BDS] != 0.0) {
                        printf(", GAL/BDS time %+.1f/%+.1f m", sol.isb[GNSS_SYS_GAL], sol.isb[GNSS_SYS_BDS]);
                    }
                    if (have_truth) {
                        double e[3];
                        enu(sol.pos, ref, rlat, rlon, e);
                        printf(", error E %+.2f N %+.2f U %+.2f m", e[0], e[1], e[2]);
                    }
                }
                printf("  (%.1fx real time)\n", ts / (now_s() - wall0));
                fflush(stdout);
            }
        }
        t0 = t_now;
    }
    double dur = (double)t0 / fs;
    printf("gnssrx: %.1f s processed in %.1f s\n", dur, now_s() - wall0);
    if (bank.golden) {
        printf("  golden model: largest |accumulator| %d (%d-bit range +-%d), overflows %u\n", bank.cm->acc_peak,
               mcfg.acc_bits, 1 << (mcfg.acc_bits - 1), bank.cm->acc_overflows);
    }
    double cn0_sum = 0.0;
    int cn0_n = 0;
    for (int ch = 0; ch < rc.max_ch; ch++) {
        if (rx->ch[ch].state == TRK_LOCKED && rx->ch[ch].cn0 > 0.0f) {
            cn0_sum += (double)rx->ch[ch].cn0;
            cn0_n++;
        }
    }
    if (cn0_n) {
        printf("  final C/N0 over %d locked channels: mean %.2f dB-Hz\n", cn0_n, cn0_sum / cn0_n);
    }
    if (n_fix > 0) {
        printf("  %ld fixes vs truth: mean E %+.3f N %+.3f U %+.3f m, sd %.3f %.3f %.3f m\n", n_fix,
               sum_e[0] / n_fix, sum_e[1] / n_fix, sum_e[2] / n_fix,
               sqrt(fmax(sum_e2[0] / n_fix - pow(sum_e[0] / n_fix, 2), 0)),
               sqrt(fmax(sum_e2[1] / n_fix - pow(sum_e[1] / n_fix, 2), 0)),
               sqrt(fmax(sum_e2[2] / n_fix - pow(sum_e[2] / n_fix, 2), 0)));
    }
    if (bank.v_samples) {
        if (bank.half) {
            uint8_t byte = (uint8_t)(bank.half_val << 4);
            fwrite(&byte, 1, 1, bank.v_samples);
        }
        fclose(bank.v_samples);
        fclose(bank.v_cmds);
        fclose(bank.v_dumps);
        printf("  test vectors: %s (%.0f ms)\n", vec_dir, vec_ms);
    }
    printf("  residual test: %ld fixes left a measurement out, %ld withheld; velocity failed it on %ld\n",
           n_excl_fix, n_withheld, n_vel_fail);
    printf("  carrier smoothing: restarted %u times where the code walked away from the carrier\n", rx->n_slip);
    if (so.fe.n_jam > 0 || so.fe.mit.type != MIT_NONE) {
        char line[512];
        mit_report(&src.fe.mit, line, sizeof(line));
        printf("  front end: %d interferer(s); magnitude density %.3f; mitigation %s\n", so.fe.n_jam,
               quant2_density(&src.fe.q), line);
        if (so.fe.mit.type != MIT_NONE) {
            printf("  interference flag on %ld epochs (the stage took out up to %.2f dB)\n", n_jam_epochs, sup_max);
        }
    }
    if (osc.n > 0 && osc_comp < 0.0) {
        printf("  oscillator: learnt %.3f ppb/g in flight (true %.3f) from %ld fixes over the pad's %ld\n",
               est_gamma * 1e9, osc.gamma * 1e9, est_n, est_npad);
    }
    if (boost_detect) {
        char ip[1024];
        snprintf(ip, sizeof(ip), "%s/run.ini", out_dir);
        FILE *fa = fopen(ip, "a");
        if (bd_n == 0) {
            printf("  boost detection: no launch detected\n");
        }
        for (int k = 0; k < bd_n; k++) {
            printf("  boost detected: on at %.3f s, burnout at %.3f s, off at %.3f s (file seconds)\n", bd_on[k],
                   bd_burnout[k], bd_off[k]);
            if (fa) {
                fprintf(fa, "boost_detected = %.3f,%.3f,%.3f\n", bd_on[k], bd_burnout[k], bd_off[k]);
            }
        }
        if (fa) {
            fclose(fa);
        }
    }
    printf("  integrity gate: %u channels dropped (under the horizon, or out of agreement), %u fixes withheld for "
           "want of redundancy\n", rx->n_gate_drop, rx->n_gate_withheld);
    if (t_first_fix_set) {
        printf("  first fix at %.3f s", t_first_fix);
        if (seed) {
            printf("; %ld coarse-time fixes; ", n_coarse_fix);
            if (rx->time_coarse) {
                printf("the navigation messages never settled the time");
            } else if (rx->t_anchor) {
                printf("time settled at %.3f s by the navigation messages, moved %+lld ms", (double)rx->t_anchor / fs,
                       (long long)rx->anchor_ms);
            } else {
                printf("the seed's time was good to start with");
            }
            printf("; %u milliseconds resolved, %u messages restarted", rx->n_ms_fixed, rx->n_nav_reset);
        }
        printf("\n");
    }
    fclose(ftrk);
    if (fimu) {
        fclose(fimu);
    }
    fclose(fobs);
    fclose(fpvt);
    fclose(feph);
    src_close(&src);
    free(bank.cm);
    free(bank.cf);
    free(rx);
    free(ring);
    free(snap);
    free(work);
    free(blk);
    free(cblk);
    return 0;
}
