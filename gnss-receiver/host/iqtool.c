/*
 * iqtool: inspect a recorded IQ file, or run it through the front-end
 * emulation and write the stream the correlators will see.
 *
 *   iqtool info FILE [--start S] [--dur S] [--fs HZ]
 *   iqtool emul FILE -o OUT [options]      (iqtool emul -h for the options)
 *
 * FILE is a path, or a name looked up in $GNSS_IQ_DIR. Rates, centre
 * frequencies and per-file fixes come from data/iq_files.ini (override with
 * --manifest or $GNSS_MANIFEST), then the .TXT sidecar, then the options.
 */
#define _POSIX_C_SOURCE 200809L

#include "fe_emul.h"
#include "fe_format.h"
#include "gnss/types.h"
#include "iq_file.h"
#include "manifest.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef enum { OUT_U2, OUT_CS8, OUT_CF32 } out_fmt_t;

typedef struct {
    const char *file, *out, *manifest;
    double fs, fc, start_s, dur_s;
    int have_fs, have_fc;
    fe_cfg_t fe;
    const char *dc_arg, *carrier_arg;
    double cn0;
    double noise_sigma;
    int have_cn0, have_noise;
    out_fmt_t fmt;
    int have_fmt;
    double native_scale;
} opts_t;

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

static const char *base_name(const char *path)
{
    const char *s = strrchr(path, '/');
    return s ? s + 1 : path;
}

static void usage(void)
{
    fprintf(stderr,
        "usage: iqtool info FILE [--start S] [--dur S] [--fs HZ]\n"
        "       iqtool emul FILE -o OUT [options]\n"
        "\n"
        "emul options:\n"
        "  --mode direct|adc27|native  direct: 2-bit at 6.75 MS/s (default); adc27: 2-bit at 27 MS/s\n"
        "                              then the FPGA decimator model; native: float at the file's rate\n"
        "  --start S --dur S           segment of the file, seconds (default: all)\n"
        "  --if HZ                     where L1 sits (default: the plan's, fe_format.h)\n"
        "  --fs-out HZ                 direct mode's output rate (default 6.75e6)\n"
        "  --cn0 DBHZ                  add noise so each satellite sits at this C/N0 (manifest sig_power)\n"
        "  --noise-sigma X             or add this per-component noise, file LSB\n"
        "  --seed N                    noise seed (default 1)\n"
        "  --if-order N --if-bw HZ     IF filter (default 5th order, 4.2e6 two-sided; order 0 = off)\n"
        "  --mag-density D             AGC target (default 0.33)\n"
        "  --decim subsample|sum4      adc27 decimator (provisional)\n"
        "  --format u2|cs8|cf32        u2: packed nibbles (default for 2-bit); cs8: int8 I,Q (2-bit as\n"
        "                              +-1/+-3, readable by Pocket SDR -fmt CS8); cf32: float (native)\n"
        "  --native-scale K            cs8 from native mode: multiply by K before rounding (default 1)\n"
        "  --dc auto|none|I,Q          DC to remove (default: manifest)\n"
        "  --carrier-fix HZ            carrier-only shift (default: manifest; +22 undoes _cofs)\n"
        "  --fs HZ --fc HZ             override the file's rate / centre\n"
        "  --manifest PATH\n");
}

static int parse(int argc, char **argv, opts_t *o)
{
    memset(o, 0, sizeof(*o));
    fe_cfg_default(&o->fe);
    o->manifest = manifest_default_path();
    o->dur_s = -1.0;
    o->native_scale = 1.0;
    for (int i = 2; i < argc; i++) {
        const char *a = argv[i];
        const char *v = (i + 1 < argc) ? argv[i + 1] : NULL;
#define TAKE() (i++, v)
        if (a[0] != '-' && !o->file) {
            o->file = a;
        } else if (!strcmp(a, "-o") && v) {
            o->out = TAKE();
        } else if (!strcmp(a, "--manifest") && v) {
            o->manifest = TAKE();
        } else if (!strcmp(a, "--fs") && v) {
            o->fs = atof(TAKE());
            o->have_fs = 1;
        } else if (!strcmp(a, "--fc") && v) {
            o->fc = atof(TAKE());
            o->have_fc = 1;
        } else if (!strcmp(a, "--start") && v) {
            o->start_s = atof(TAKE());
        } else if (!strcmp(a, "--dur") && v) {
            o->dur_s = atof(TAKE());
        } else if (!strcmp(a, "--mode") && v) {
            const char *m = TAKE();
            if (!strcmp(m, "direct")) {
                o->fe.mode = FE_MODE_DIRECT;
            } else if (!strcmp(m, "adc27")) {
                o->fe.mode = FE_MODE_ADC27;
            } else if (!strcmp(m, "native")) {
                o->fe.mode = FE_MODE_NATIVE;
            } else {
                return -1;
            }
        } else if (!strcmp(a, "--if") && v) {
            o->fe.if_hz = atof(TAKE());
        } else if (!strcmp(a, "--fs-out") && v) {
            o->fe.fs_out = atof(TAKE());
        } else if (!strcmp(a, "--cn0") && v) {
            o->cn0 = atof(TAKE());
            o->have_cn0 = 1;
        } else if (!strcmp(a, "--noise-sigma") && v) {
            o->noise_sigma = atof(TAKE());
            o->have_noise = 1;
        } else if (!strcmp(a, "--seed") && v) {
            o->fe.seed = strtoull(TAKE(), NULL, 0);
        } else if (!strcmp(a, "--if-order") && v) {
            o->fe.if_order = atoi(TAKE());
        } else if (!strcmp(a, "--if-bw") && v) {
            o->fe.if_bw_hz = atof(TAKE());
        } else if (!strcmp(a, "--mag-density") && v) {
            o->fe.mag_density = atof(TAKE());
        } else if (!strcmp(a, "--decim") && v) {
            const char *m = TAKE();
            o->fe.decim_mode = !strcmp(m, "sum4") ? DECIM_SUM4 : DECIM_SUBSAMPLE;
        } else if (!strcmp(a, "--format") && v) {
            const char *m = TAKE();
            o->fmt = !strcmp(m, "cs8") ? OUT_CS8 : !strcmp(m, "cf32") ? OUT_CF32 : OUT_U2;
            o->have_fmt = 1;
        } else if (!strcmp(a, "--native-scale") && v) {
            o->native_scale = atof(TAKE());
        } else if (!strcmp(a, "--dc") && v) {
            o->dc_arg = TAKE();
        } else if (!strcmp(a, "--carrier-fix") && v) {
            o->carrier_arg = TAKE();
        } else {
            fprintf(stderr, "iqtool: unknown or incomplete option %s\n", a);
            return -1;
        }
#undef TAKE
    }
    return o->file ? 0 : -1;
}

/* Opens the file with its metadata resolved. */
static int open_input(opts_t *o, iqf_t *f, iq_meta_t *meta, char *path, size_t path_size)
{
    if (manifest_resolve_iq(o->file, path, path_size) != 0) {
        fprintf(stderr, "iqtool: %s not found (set GNSS_IQ_DIR for bare names)\n", o->file);
        return -1;
    }
    manifest_lookup(o->manifest, base_name(path), meta);
    double fs = meta->fs, fc = meta->fc;
    double sfs = 0.0, sfc = 0.0;
    if (iqf_read_sidecar(path, &sfs, &sfc) == 0) {
        if (fs == 0.0 && sfs > 0.0) {
            fs = sfs;
        }
        if (fc == 0.0 && sfc > 0.0) {
            fc = sfc;
        }
    }
    if (o->have_fs) {
        fs = o->fs;
    }
    if (o->have_fc) {
        fc = o->fc;
    }
    if (fc == 0.0) {
        fc = GNSS_FREQ_L1_HZ;
    }
    if (!(fs > 0.0)) {
        fprintf(stderr, "iqtool: no sample rate for %s: add it to the manifest or pass --fs\n", path);
        return -1;
    }
    if (iqf_open(f, path, IQF_CS8, fs, fc) != 0) {
        fprintf(stderr, "iqtool: cannot open %s\n", path);
        return -1;
    }
    return 0;
}

/* Mean and RMS per component over n samples from the current position; restores the position. */
static void measure(iqf_t *f, size_t n, double *mi, double *mq, double *ri, double *rq, double *clip,
                    int *levels)
{
    int64_t pos = f->pos;
    float *buf = (float *)malloc(sizeof(float) * 2 * n);
    size_t got = buf ? iqf_read(f, buf, n) : 0;
    double si = 0, sq = 0, ssi = 0, ssq = 0;
    size_t nclip = 0;
    int seen[256] = {0};
    for (size_t k = 0; k < got; k++) {
        double i = buf[2 * k], q = buf[2 * k + 1];
        si += i;
        sq += q;
        ssi += i * i;
        ssq += q * q;
        nclip += (fabs(i) >= 127.0) + (fabs(q) >= 127.0);
        seen[(int)i + 128] = 1;
        seen[(int)q + 128] = 1;
    }
    double nn = got ? (double)got : 1.0;
    *mi = si / nn;
    *mq = sq / nn;
    *ri = sqrt(ssi / nn - (*mi) * (*mi));
    *rq = sqrt(ssq / nn - (*mq) * (*mq));
    *clip = (double)nclip / (2.0 * nn);
    *levels = 0;
    for (int k = 0; k < 256; k++) {
        *levels += seen[k];
    }
    free(buf);
    iqf_seek(f, pos);
}

static int cmd_info(int argc, char **argv)
{
    opts_t o;
    if (parse(argc, argv, &o) != 0) {
        usage();
        return 2;
    }
    iqf_t f;
    iq_meta_t meta;
    char path[4096];
    if (open_input(&o, &f, &meta, path, sizeof(path)) != 0) {
        return 1;
    }
    double dur = (double)f.nsamp / f.fs;
    printf("file      %s\n", path);
    printf("manifest  %s\n", meta.found ? "entry found" : "no entry");
    printf("rate      %.6f MS/s, centre %.6f MHz (L1 at %+.6f MHz)\n", f.fs / 1e6, f.fc / 1e6,
           (GNSS_FREQ_L1_HZ - f.fc) / 1e6);
    printf("length    %lld samples = %.3f s\n", (long long)f.nsamp, dur);
    iqf_seek(&f, (int64_t)(o.start_s * f.fs));
    double win = o.dur_s > 0 ? o.dur_s : 1.0;
    double mi, mq, ri, rq, clip;
    int levels;
    measure(&f, (size_t)(win * f.fs), &mi, &mq, &ri, &rq, &clip, &levels);
    printf("at %.1f s over %.3f s: mean %+.3f %+.3f  rms %.2f %.2f  clipped %.4f %%  levels %d\n",
           o.start_s, win, mi, mq, ri, rq, 100.0 * clip, levels);
    if (meta.found) {
        printf("generator %s, noise %s, dc %s, carrier fix %+.1f Hz, sig_power %.4g, N0 %.4g\n",
               meta.generator, meta.has_noise ? "in file" : "none",
               meta.dc_auto ? "auto" : "fixed", meta.carrier_fix_hz, meta.sig_power, meta.noise_density);
        if (meta.start_gpst[0]) {
            printf("start     %s GPST\n", meta.start_gpst);
        }
        if (meta.truth[0]) {
            printf("truth     %s\n", meta.truth);
        }
    }
    iqf_close(&f);
    return 0;
}

typedef struct {
    FILE *fp;
    out_fmt_t fmt;
    int have_half;
    uint8_t half;
    double native_scale;
    uint64_t n;
} writer_t;

static const int8_t kWeight[2][2] = {
    {FE_WEIGHT_SMALL, FE_WEIGHT_LARGE},     /* positive: small, large */
    {-FE_WEIGHT_SMALL, -FE_WEIGHT_LARGE},   /* negative */
};

static void write_codes(writer_t *w, const uint8_t *c, size_t n)
{
    if (w->fmt == OUT_U2) {
        size_t k = 0;
        uint8_t buf[8192];
        size_t nb = 0;
        if (w->have_half && n > 0) {
            buf[nb++] = (uint8_t)((w->half << 4) | (c[k++] & 0xF));
            w->have_half = 0;
        }
        for (; k + 1 < n; k += 2) {
            buf[nb++] = (uint8_t)(((c[k] & 0xF) << 4) | (c[k + 1] & 0xF));
            if (nb == sizeof(buf)) {
                fwrite(buf, 1, nb, w->fp);
                nb = 0;
            }
        }
        if (k < n) {
            w->half = c[k] & 0xF;
            w->have_half = 1;
        }
        fwrite(buf, 1, nb, w->fp);
    } else {
        int8_t buf[8192];
        size_t nb = 0;
        for (size_t k = 0; k < n; k++) {
            unsigned x = c[k];
            buf[nb++] = kWeight[(x & FE_CODE_I_SIGN) != 0][(x & FE_CODE_I_MAG) != 0];
            buf[nb++] = kWeight[(x & FE_CODE_Q_SIGN) != 0][(x & FE_CODE_Q_MAG) != 0];
            if (nb == sizeof(buf)) {
                fwrite(buf, 1, nb, w->fp);
                nb = 0;
            }
        }
        fwrite(buf, 1, nb, w->fp);
    }
    w->n += n;
}

static void write_float(writer_t *w, const float *iq, size_t n)
{
    if (w->fmt == OUT_CF32) {
        fwrite(iq, sizeof(float), 2 * n, w->fp);
    } else {
        int8_t buf[8192];
        size_t nb = 0;
        for (size_t k = 0; k < 2 * n; k++) {
            double v = floor(iq[k] * w->native_scale + 0.5);
            buf[nb++] = (int8_t)(v > 127 ? 127 : v < -127 ? -127 : v);
            if (nb == sizeof(buf)) {
                fwrite(buf, 1, nb, w->fp);
                nb = 0;
            }
        }
        fwrite(buf, 1, nb, w->fp);
    }
    w->n += n;
}

static void writer_close(writer_t *w)
{
    if (w->fmt == OUT_U2 && w->have_half) {
        uint8_t b = (uint8_t)(w->half << 4);
        fwrite(&b, 1, 1, w->fp);
    }
    fclose(w->fp);
}

static const char *mode_name(fe_mode_t m)
{
    return m == FE_MODE_NATIVE ? "native" : m == FE_MODE_ADC27 ? "adc27" : "direct";
}

static const char *fmt_name(out_fmt_t f)
{
    return f == OUT_U2 ? "u2" : f == OUT_CS8 ? "cs8" : "cf32";
}

static int cmd_emul(int argc, char **argv)
{
    opts_t o;
    if (parse(argc, argv, &o) != 0 || !o.out) {
        usage();
        return 2;
    }
    iqf_t f;
    iq_meta_t meta;
    char path[4096];
    if (open_input(&o, &f, &meta, path, sizeof(path)) != 0) {
        return 1;
    }
    fe_cfg_t *c = &o.fe;
    c->fs_in = f.fs;
    c->fc_in = f.fc;
    int64_t start = (int64_t)llround(o.start_s * f.fs);
    int64_t end = f.nsamp;
    if (o.dur_s > 0) {
        int64_t e = start + (int64_t)llround(o.dur_s * f.fs);
        end = e < end ? e : end;
    }
    if (start >= end || iqf_seek(&f, start) != 0) {
        fprintf(stderr, "iqtool: segment outside the file\n");
        return 1;
    }
    c->start = start;

    /* DC: option, else manifest (auto = measured over the first second of the segment). */
    int dc_auto = meta.dc_auto;
    c->dc_i = meta.dc_i;
    c->dc_q = meta.dc_q;
    if (o.dc_arg) {
        dc_auto = !strcmp(o.dc_arg, "auto");
        if (!strcmp(o.dc_arg, "none")) {
            c->dc_i = c->dc_q = 0.0;
        } else if (!dc_auto && sscanf(o.dc_arg, "%lf,%lf", &c->dc_i, &c->dc_q) == 1) {
            c->dc_q = c->dc_i;
        }
    }
    if (dc_auto) {
        double ri, rq, clip;
        int levels;
        size_t win = (size_t)((end - start) < (int64_t)f.fs ? (end - start) : (int64_t)f.fs);
        measure(&f, win, &c->dc_i, &c->dc_q, &ri, &rq, &clip, &levels);
    }
    c->carrier_fix_hz = o.carrier_arg ? atof(o.carrier_arg) : meta.carrier_fix_hz;

    /* Noise. */
    double fs_add = (c->mode == FE_MODE_NATIVE) ? f.fs : (c->mode == FE_MODE_ADC27 ? FE_FS_ADC_HZ : c->fs_out);
    if (o.have_cn0) {
        if (!(meta.sig_power > 0.0)) {
            fprintf(stderr, "iqtool: --cn0 needs sig_power for %s in the manifest\n", base_name(path));
            return 1;
        }
        double s = fe_sigma_for_cn0(o.cn0, meta.sig_power, meta.noise_density, fs_add);
        if (s < 0.0) {
            fprintf(stderr, "iqtool: the file is already below %.1f dB-Hz\n", o.cn0);
            return 1;
        }
        c->noise_sigma = s;
    } else if (o.have_noise) {
        c->noise_sigma = o.noise_sigma;
    }
    if (!o.have_fmt) {
        o.fmt = (c->mode == FE_MODE_NATIVE) ? OUT_CF32 : OUT_U2;
    }
    if (c->mode == FE_MODE_NATIVE && o.fmt == OUT_U2) {
        fprintf(stderr, "iqtool: native mode writes cf32 or cs8\n");
        return 2;
    }
    if (c->mode != FE_MODE_NATIVE && o.fmt == OUT_CF32) {
        fprintf(stderr, "iqtool: 2-bit modes write u2 or cs8\n");
        return 2;
    }

    if (c->mode != FE_MODE_NATIVE && c->if_order > 0) {
        double rate = (c->mode == FE_MODE_ADC27) ? FE_FS_ADC_HZ : c->fs_out;
        if (0.5 * c->if_bw_hz >= 0.5 * rate) {
            fprintf(stderr, "iqtool: a %.2f MHz IF filter does not fit in a %.3f MS/s stream; "
                    "narrow it (--if-bw 2.5e6) or turn it off (--if-order 0)\n", c->if_bw_hz / 1e6, rate / 1e6);
            return 2;
        }
    }
    fe_t fe;
    if (fe_init(&fe, c) != 0) {
        fprintf(stderr, "iqtool: front-end emulation setup failed\n");
        return 1;
    }
    writer_t w = {0};
    w.fp = fopen(o.out, "wb");
    if (!w.fp) {
        fprintf(stderr, "iqtool: cannot write %s\n", o.out);
        return 1;
    }
    w.fmt = o.fmt;
    w.native_scale = o.native_scale;

    size_t blk = (size_t)(0.01 * f.fs);  /* 10 ms of input per step */
    float *in = (float *)malloc(sizeof(float) * 2 * blk);
    size_t max_out = fe_max_out(&fe, blk);
    float *out_f = (float *)malloc(sizeof(float) * 2 * max_out);
    uint8_t *out_c = (uint8_t *)malloc(max_out);
    double t0 = now_s();
    int64_t pos = start;
    while (pos < end) {
        size_t want = (size_t)((end - pos) < (int64_t)blk ? (end - pos) : (int64_t)blk);
        size_t got = iqf_read(&f, in, want);
        if (got == 0) {
            break;
        }
        pos += (int64_t)got;
        size_t n = fe_process(&fe, in, got, out_f, out_c, max_out);
        if (c->mode == FE_MODE_NATIVE) {
            write_float(&w, out_f, n);
        } else {
            write_codes(&w, out_c, n);
        }
    }
    double elapsed = now_s() - t0;
    writer_close(&w);

    double seg_s = (double)(pos - start) / f.fs;
    double rms = fe.n_sumsq ? sqrt(fe.sumsq / (double)fe.n_sumsq) : 0.0;
    double dens = (c->mode == FE_MODE_NATIVE) ? 0.0 : quant2_density(&fe.q);
    printf("%s: %.3f s of %s -> %llu samples at %.6f MS/s, IF %+.6f MHz (%s, %s)\n", o.out, seg_s,
           base_name(path), (unsigned long long)w.n, fe_out_rate(&fe) / 1e6, fe_out_if(&fe) / 1e6,
           mode_name(c->mode), fmt_name(o.fmt));
    printf("  dc %+.3f %+.3f, carrier fix %+.1f Hz, noise sigma %.3f LSB%s\n", c->dc_i, c->dc_q,
           c->carrier_fix_hz, c->noise_sigma, o.have_cn0 ? " (from --cn0)" : "");
    if (c->mode != FE_MODE_NATIVE) {
        printf("  into the quantizer: rms %.3f per component; magnitude density %.4f; AGC threshold %.3f\n",
               rms, dens, fe.q.thr);
    }
    printf("  %.2f s elapsed, %.1fx real time\n", elapsed, elapsed > 0 ? seg_s / elapsed : 0.0);

    /* Metadata beside the output. */
    char meta_path[4200];
    snprintf(meta_path, sizeof(meta_path), "%s.ini", o.out);
    FILE *mp = fopen(meta_path, "w");
    if (mp) {
        fprintf(mp, "; written by iqtool emul\n[stream]\n");
        fprintf(mp, "source = %s\nsource_fs = %.3f\nsource_fc = %.3f\n", base_name(path), f.fs, f.fc);
        fprintf(mp, "start_s = %.9f\nstart_sample = %lld\nduration_s = %.9f\n", o.start_s, (long long)start, seg_s);
        fprintf(mp, "mode = %s\nformat = %s\nfs = %.3f\nif_hz = %.3f\nsamples = %llu\n", mode_name(c->mode),
                fmt_name(o.fmt), fe_out_rate(&fe), fe_out_if(&fe), (unsigned long long)w.n);
        fprintf(mp, "; output sample k is the source at sample start_sample + k * step (+ offset)\n");
        fprintf(mp, "step = %.15f\noffset = %.6f\n",
                c->mode == FE_MODE_ADC27 ? fe.step * FE_DECIM : fe.step,
                fe_out_position(&fe, 0) - (double)start);
        fprintf(mp, "dc_i = %.4f\ndc_q = %.4f\ncarrier_fix_hz = %.3f\n", c->dc_i, c->dc_q, c->carrier_fix_hz);
        fprintf(mp, "noise_sigma = %.6f\nseed = %llu\n", c->noise_sigma, (unsigned long long)c->seed);
        if (o.have_cn0) {
            fprintf(mp, "cn0_target_dbhz = %.2f\n", o.cn0);
        }
        if (c->mode != FE_MODE_NATIVE) {
            fprintf(mp, "if_order = %d\nif_bw_hz = %.1f\nmag_density = %.5f\nagc_threshold = %.5f\n",
                    c->if_order, c->if_bw_hz, dens, fe.q.thr);
            fprintf(mp, "; u2: two samples per byte, earlier in the high nibble; nibble = I sign, I mag, Q sign, Q mag\n");
            fprintf(mp, "; sign 1 = negative, mag 1 = large; cs8 holds the weights +-%d / +-%d\n",
                    FE_WEIGHT_SMALL, FE_WEIGHT_LARGE);
            if (c->mode == FE_MODE_ADC27) {
                fprintf(mp, "decim = %s\n", c->decim_mode == DECIM_SUM4 ? "sum4" : "subsample");
            }
        }
        fclose(mp);
    }
    free(in);
    free(out_f);
    free(out_c);
    fe_free(&fe);
    iqf_close(&f);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        usage();
        return 2;
    }
    if (!strcmp(argv[1], "info")) {
        return cmd_info(argc, argv);
    }
    if (!strcmp(argv[1], "emul")) {
        return cmd_emul(argc, argv);
    }
    usage();
    return 2;
}
