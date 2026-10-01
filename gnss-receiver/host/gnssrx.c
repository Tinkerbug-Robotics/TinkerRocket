/*
 * gnssrx: the stage-0 receiver on a recorded IQ file.
 *
 *   gnssrx FILE [source options] [--out DIR] [--meas-hz N] [--acq-threshold M]
 *
 * The file runs through the front-end emulation (host/source.c) into the float
 * correlator bank (host/corr_float.c); the receiver core (core/) gets a 1 ms
 * tick with the dumps, exactly as it will from the FPGA's interrupt on the P4.
 * Writes trk.csv, obs.csv, pvt.csv and eph.csv into the output directory.
 */
#define _POSIX_C_SOURCE 200809L

#include "corr_float.h"
#include "gnss/rx.h"
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
    fprintf(stderr, "usage: gnssrx FILE [source options] [--out DIR] [--meas-hz N] [--acq-threshold M]\n"
                    "                    [--no-iono] [--no-tropo] [--iono A0,A1,A2,A3,B0,B1,B2,B3] [--truth static:LAT,LON,H]\n");
    src_usage();
}

static FILE *open_csv(const char *dir, const char *name, const char *header)
{
    char p[4200];
    snprintf(p, sizeof(p), "%s/%s", dir, name);
    FILE *f = fopen(p, "w");
    if (f) {
        fprintf(f, "%s\n", header);
    }
    return f;
}

/* Truth "static:LAT,LON,H" from the manifest, as ECEF; returns 1 if there is one. */
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

static void enu(const double x[3], const double ref[3], double lat, double lon, double out[3])
{
    double d[3] = {x[0] - ref[0], x[1] - ref[1], x[2] - ref[2]};
    double sl = sin(lat), cl = cos(lat), so = sin(lon), co = cos(lon);
    out[0] = -so * d[0] + co * d[1];
    out[1] = -sl * co * d[0] - sl * so * d[1] + cl * d[2];
    out[2] = cl * co * d[0] + cl * so * d[1] + sl * d[2];
}

int main(int argc, char **argv)
{
    src_opts_t so;
    src_default_opts(&so);
    const char *out_dir = "runs/gnssrx";
    double meas_hz = 10.0;
    float acq_thr = -1.0f;
    int no_iono = 0, no_tropo = 0;
    const char *truth_arg = NULL;
    gps_iono_t iono_aid;
    memset(&iono_aid, 0, sizeof(iono_aid));
    for (int i = 1; i < argc; i++) {
        int r = src_parse_opt(&so, argc, argv, &i);
        if (r == 1) {
            continue;
        }
        if (r < 0) {
            usage();
            return 2;
        }
        if (!strcmp(argv[i], "--out") && i + 1 < argc) {
            out_dir = argv[++i];
        } else if (!strcmp(argv[i], "--meas-hz") && i + 1 < argc) {
            meas_hz = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--acq-threshold") && i + 1 < argc) {
            acq_thr = (float)atof(argv[++i]);
        } else if (!strcmp(argv[i], "--no-iono")) {
            no_iono = 1;
        } else if (!strcmp(argv[i], "--no-tropo")) {
            no_tropo = 1;
        } else if (!strcmp(argv[i], "--truth") && i + 1 < argc) {
            truth_arg = argv[++i];
        } else if (!strcmp(argv[i], "--iono") && i + 1 < argc) {
            /* Klobuchar alpha0..3, beta0..3: assistance, as the flight computer could preload. */
            double *a = iono_aid.alpha, *b = iono_aid.beta;
            if (sscanf(argv[++i], "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &a[0], &a[1], &a[2], &a[3], &b[0], &b[1], &b[2],
                       &b[3]) != 8) {
                fprintf(stderr, "gnssrx: --iono takes eight comma-separated values\n");
                return 2;
            }
            iono_aid.valid = 1;
        } else {
            fprintf(stderr, "gnssrx: unknown option %s\n", argv[i]);
            usage();
            return 2;
        }
    }
    if (!so.file) {
        usage();
        return 2;
    }
    src_t src;
    if (src_open(&src, &so) != 0) {
        return 1;
    }
    const double fs = src.fs_out;
    const uint64_t spms = (uint64_t)llround(fs / 1000.0);
    if (fabs(fs / 1000.0 - (double)spms) > 1e-6) {
        fprintf(stderr, "gnssrx: the stream rate must be a whole number of samples per ms\n");
        return 1;
    }
    if (mkdir(out_dir, 0755) != 0 && errno != EEXIST) {
        fprintf(stderr, "gnssrx: cannot create %s\n", out_dir);
        return 1;
    }

    corr_float_t *cf = (corr_float_t *)malloc(sizeof(corr_float_t));
    rx_t *rx = (rx_t *)malloc(sizeof(rx_t));
    if (!cf || !rx) {
        return 1;
    }
    corr_float_init(cf, fs);
    rx_cfg_t rc;
    rx_default_cfg(&rc, fs, src.if_out);
    if (acq_thr > 0.0f) {
        rc.acq_threshold = acq_thr;
    }
    rx_init(rx, &rc);
    rx->pvt_opt.use_iono = !no_iono;
    /* The generator's own ionosphere and troposphere, from the manifest (page 18 repeats only every
     * 12.5 min, so short files never deliver it); --iono and --no-tropo override. */
    if (!iono_aid.valid && src.meta.have_iono) {
        memcpy(iono_aid.alpha, src.meta.iono, sizeof(iono_aid.alpha));
        memcpy(iono_aid.beta, src.meta.iono + 4, sizeof(iono_aid.beta));
        iono_aid.valid = 1;
    }
    if (iono_aid.valid) {
        rx->iono = iono_aid;
    }
    rx->pvt_opt.use_tropo = !no_tropo && !src.meta.tropo_none;

    const int acq_ms = rc.acq_ms;
    float *ring = (float *)malloc(sizeof(float) * 2 * spms * (size_t)acq_ms);
    float *snap = (float *)malloc(sizeof(float) * 2 * spms * (size_t)acq_ms);
    float *work = (float *)malloc(sizeof(float) * acq_work_floats(2048, acq_ms));
    float *blk = (float *)malloc(sizeof(float) * 2 * spms);
    if (!ring || !snap || !work || !blk) {
        return 1;
    }
    int ring_head = 0, ring_count = 0;

    FILE *ftrk = open_csv(out_dir, "trk.csv", "t_s,ch,prn,state,cn0,dop_hz,pll_lock,bit_sync,frame_sync,subframes,parity_fail");
    FILE *fobs = open_csv(out_dir, "obs.csv", "t_s,rx_tow,prn,pr_m,adr_cyc,dop_hz,cn0,lock_s,el_deg,resid_m");
    FILE *fpvt = open_csv(out_dir, "pvt.csv",
                          "t_s,rx_tow,week,lat_deg,lon_deg,h_m,x,y,z,vx,vy,vz,clk_bias_m,clk_drift_mps,nsat,pdop,"
                          "resid_rms,e_m,n_m,u_m");
    FILE *feph = open_csv(out_dir, "eph.csv",
                          "t_s,prn,week,toe,toc,iode,iodc,health,sqrt_a,e,i0,omega0,omega,m0,delta_n,idot,"
                          "omega_dot,cuc,cus,crc,crs,cic,cis,af0,af1,af2,tgd");
    if (!ftrk || !fobs || !fpvt || !feph) {
        fprintf(stderr, "gnssrx: cannot write into %s\n", out_dir);
        return 1;
    }
    double ref[3], rlat = 0.0, rlon = 0.0;
    int have_truth = static_truth(truth_arg ? truth_arg : src.meta.truth, ref, &rlat, &rlon);

    printf("gnssrx: %s from %.1f s, %.3f MS/s, IF %+.3f MHz, %s -> %s\n", src.path, so.start_s, fs / 1e6,
           src.if_out / 1e6, so.fe.mode == FE_MODE_NATIVE ? "native" : "2-bit", out_dir);

    corr_dump_t dumps[MAX_DUMPS];
    corr_cmd_t cmds[MAX_CMDS];
    rx_obs_t obs[CORR_MAX_CH];
    uint64_t t0 = 0;
    const uint64_t meas_step = (uint64_t)llround(fs / meas_hz);
    double eph_toe[GPS_MAX_PRN + 1];
    for (int p = 0; p <= GPS_MAX_PRN; p++) {
        eph_toe[p] = -1.0;
    }
    double wall0 = now_s();
    double sum_e[3] = {0}, sum_e2[3] = {0};
    long n_fix = 0;
    for (;;) {
        if (src_read(&src, blk, spms) < spms) {
            break;
        }
        memcpy(ring + 2 * spms * (size_t)ring_head, blk, sizeof(float) * 2 * spms);
        ring_head = (ring_head + 1) % acq_ms;
        ring_count = ring_count < acq_ms ? ring_count + 1 : acq_ms;

        int nd = corr_float_process(cf, t0, blk, spms, dumps, MAX_DUMPS);
        uint64_t t_now = t0 + spms;
        int nc = rx_tick(rx, t_now, dumps, nd, cmds, MAX_CMDS);
        for (int k = 0; k < nc; k++) {
            corr_float_command(cf, &cmds[k]);
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
                corr_float_command(cf, &cmds[k]);
                if (cmds[k].type == CORR_CMD_START) {
                    printf("  %7.3f s  ch %2d  PRN %2d  acquired, Doppler %+7.1f Hz, metric %.1f\n",
                           (double)t_now / fs, cmds[k].ch, cmds[k].prn, rx->ch[cmds[k].ch].dop_hz,
                           rx->ch[cmds[k].ch].acq_metric);
                }
            }
        }
        if (t_now % meas_step == 0) {
            double ts = (double)t_now / fs;
            pvt_sol_t sol;
            int no = rx_measure(rx, t_now, obs, CORR_MAX_CH, &sol);
            double rtow = rx->clk_valid ? rx_time(rx, t_now) : 0.0;
            for (int k = 0; k < no && rx->clk_valid; k++) {
                fprintf(fobs, "%.3f,%.9f,%d,%.4f,%.4f,%.4f,%.2f,%.2f,%.3f,%.4f\n", ts, rtow, obs[k].prn, obs[k].pr,
                        obs[k].adr, obs[k].dop, obs[k].cn0, obs[k].lock_s, sol.valid ? sol.el[k] * 180.0 / PI : 0.0,
                        sol.valid && sol.used[k] ? sol.resid[k] : 0.0);
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
                fprintf(fpvt, "%.3f,%.9f,%d,%.9f,%.9f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.3f,%.4f,%.4f,%.4f,%.4f\n",
                        ts, rx_time(rx, t_now), rx->week, sol.lat * 180.0 / PI, sol.lon * 180.0 / PI, sol.h,
                        sol.pos[0], sol.pos[1], sol.pos[2], sol.vel[0], sol.vel[1], sol.vel[2], sol.clk_bias,
                        sol.clk_drift, sol.nsat, sol.pdop, sol.resid_rms, e[0], e[1], e[2]);
            }
            for (int ch = 0; ch < rc.max_ch; ch++) {
                const trk_ch_t *c = &rx->ch[ch];
                if (c->prn == 0) {
                    continue;
                }
                fprintf(ftrk, "%.3f,%d,%d,%d,%.2f,%.3f,%.3f,%d,%d,%u,%u\n", ts, ch, c->prn, c->state, c->cn0, c->dop_hz,
                        c->pll_lock, c->bit_sync, rx->nav[ch].synced, rx->nav[ch].n_subframes,
                        rx->nav[ch].n_parity_fail);
            }
            for (int p = 1; p <= GPS_MAX_PRN; p++) {
                const gps_eph_t *e = &rx->eph[p];
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
    if (n_fix > 0) {
        printf("  %ld fixes vs truth: mean E %+.3f N %+.3f U %+.3f m, sd %.3f %.3f %.3f m\n", n_fix,
               sum_e[0] / n_fix, sum_e[1] / n_fix, sum_e[2] / n_fix,
               sqrt(fmax(sum_e2[0] / n_fix - pow(sum_e[0] / n_fix, 2), 0)),
               sqrt(fmax(sum_e2[1] / n_fix - pow(sum_e[1] / n_fix, 2), 0)),
               sqrt(fmax(sum_e2[2] / n_fix - pow(sum_e[2] / n_fix, 2), 0)));
    }
    fclose(ftrk);
    fclose(fobs);
    fclose(fpvt);
    fclose(feph);
    src_close(&src);
    free(cf);
    free(rx);
    free(ring);
    free(snap);
    free(work);
    free(blk);
    return 0;
}
