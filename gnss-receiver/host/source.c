#define _POSIX_C_SOURCE 200809L

#include "source.h"

#include "fe_format.h"
#include "gnss/types.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void src_default_opts(src_opts_t *o)
{
    memset(o, 0, sizeof(*o));
    fe_cfg_default(&o->fe);
    o->manifest = manifest_default_path();
    o->dur_s = -1.0;
}

void src_usage(void)
{
    fprintf(stderr,
            "source options:\n"
            "  --start S --dur S           segment of the file, seconds (default: all)\n"
            "  --mode direct|adc27|native  front-end emulation (default direct: 2-bit at 6.75 MS/s)\n"
            "  --fs-out HZ                 direct mode's output rate (default 6.75e6)\n"
            "  --if HZ                     where L1 sits (default: the plan's, fe_format.h)\n"
            "  --cn0 DBHZ | --noise-sigma X  added noise (C/N0 needs the manifest's sig_power)\n"
            "  --seed N --if-order N --if-bw HZ --mag-density D --decim subsample|sum4\n"
            "  --dc auto|none|I,Q --carrier-fix HZ --fs HZ --fc HZ --manifest PATH\n");
}

int src_parse_opt(src_opts_t *o, int argc, char **argv, int *i)
{
    const char *a = argv[*i];
    const char *v = (*i + 1 < argc) ? argv[*i + 1] : NULL;
#define VAL() (v ? ((*i)++, v) : NULL)
    if (a[0] != '-') {
        if (!o->file) {
            o->file = a;
            return 1;
        }
        return 0;
    }
    if (!v) {
        return 0;
    }
    if (!strcmp(a, "--manifest")) {
        o->manifest = VAL();
    } else if (!strcmp(a, "--fs")) {
        o->fs = atof(VAL());
        o->have_fs = 1;
    } else if (!strcmp(a, "--fc")) {
        o->fc = atof(VAL());
        o->have_fc = 1;
    } else if (!strcmp(a, "--start")) {
        o->start_s = atof(VAL());
    } else if (!strcmp(a, "--dur")) {
        o->dur_s = atof(VAL());
    } else if (!strcmp(a, "--mode")) {
        const char *m = VAL();
        if (!strcmp(m, "direct")) {
            o->fe.mode = FE_MODE_DIRECT;
        } else if (!strcmp(m, "adc27")) {
            o->fe.mode = FE_MODE_ADC27;
        } else if (!strcmp(m, "native")) {
            o->fe.mode = FE_MODE_NATIVE;
        } else {
            return -1;
        }
    } else if (!strcmp(a, "--if")) {
        o->fe.if_hz = atof(VAL());
    } else if (!strcmp(a, "--fs-out")) {
        o->fe.fs_out = atof(VAL());
    } else if (!strcmp(a, "--cn0")) {
        o->cn0 = atof(VAL());
        o->have_cn0 = 1;
    } else if (!strcmp(a, "--noise-sigma")) {
        o->noise_sigma = atof(VAL());
        o->have_noise = 1;
    } else if (!strcmp(a, "--seed")) {
        o->fe.seed = strtoull(VAL(), NULL, 0);
    } else if (!strcmp(a, "--if-order")) {
        o->fe.if_order = atoi(VAL());
    } else if (!strcmp(a, "--if-bw")) {
        o->fe.if_bw_hz = atof(VAL());
    } else if (!strcmp(a, "--mag-density")) {
        o->fe.mag_density = atof(VAL());
    } else if (!strcmp(a, "--decim")) {
        o->fe.decim_mode = !strcmp(VAL(), "sum4") ? DECIM_SUM4 : DECIM_SUBSAMPLE;
    } else if (!strcmp(a, "--dc")) {
        o->dc_arg = VAL();
    } else if (!strcmp(a, "--carrier-fix")) {
        o->carrier_arg = VAL();
    } else {
        return 0;
    }
#undef VAL
    return 1;
}

static const char *base_name(const char *path)
{
    const char *s = strrchr(path, '/');
    return s ? s + 1 : path;
}

int src_open(src_t *s, src_opts_t *o)
{
    memset(s, 0, sizeof(*s));
    if (!o->file || manifest_resolve_iq(o->file, s->path, sizeof(s->path)) != 0) {
        fprintf(stderr, "source: %s not found (set GNSS_IQ_DIR for bare names)\n", o->file ? o->file : "(none)");
        return -1;
    }
    manifest_lookup(o->manifest, base_name(s->path), &s->meta);
    double fs = s->meta.fs, fc = s->meta.fc, sfs = 0.0, sfc = 0.0;
    if (iqf_read_sidecar(s->path, &sfs, &sfc) == 0) {
        fs = fs > 0.0 ? fs : sfs;
        fc = fc > 0.0 ? fc : sfc;
    }
    fs = o->have_fs ? o->fs : fs;
    fc = o->have_fc ? o->fc : (fc > 0.0 ? fc : GNSS_FREQ_L1_HZ);
    if (!(fs > 0.0) || iqf_open(&s->f, s->path, IQF_CS8, fs, fc) != 0) {
        fprintf(stderr, "source: cannot open %s (rate unknown? add it to the manifest or pass --fs)\n", s->path);
        return -1;
    }
    fe_cfg_t *c = &o->fe;
    c->fs_in = fs;
    c->fc_in = fc;
    int64_t start = (int64_t)llround(o->start_s * fs);
    s->end = s->f.nsamp;
    if (o->dur_s > 0) {
        int64_t e = start + (int64_t)llround(o->dur_s * fs);
        s->end = e < s->end ? e : s->end;
    }
    if (start >= s->end || iqf_seek(&s->f, start) != 0) {
        fprintf(stderr, "source: segment outside the file\n");
        return -1;
    }
    c->start = start;

    /* DC: option, else manifest; auto = the mean of the segment's first second. */
    int dc_auto = s->meta.dc_auto;
    c->dc_i = s->meta.dc_i;
    c->dc_q = s->meta.dc_q;
    if (o->dc_arg) {
        dc_auto = !strcmp(o->dc_arg, "auto");
        if (!strcmp(o->dc_arg, "none")) {
            c->dc_i = c->dc_q = 0.0;
        } else if (!dc_auto && sscanf(o->dc_arg, "%lf,%lf", &c->dc_i, &c->dc_q) == 1) {
            c->dc_q = c->dc_i;
        }
    }
    if (dc_auto) {
        size_t win = (size_t)((s->end - start) < (int64_t)fs ? (s->end - start) : (int64_t)fs);
        float *b = (float *)malloc(sizeof(float) * 2 * win);
        size_t got = b ? iqf_read(&s->f, b, win) : 0;
        double si = 0.0, sq = 0.0;
        for (size_t k = 0; k < got; k++) {
            si += b[2 * k];
            sq += b[2 * k + 1];
        }
        c->dc_i = got ? si / (double)got : 0.0;
        c->dc_q = got ? sq / (double)got : 0.0;
        free(b);
        iqf_seek(&s->f, start);
    }
    c->carrier_fix_hz = o->carrier_arg ? atof(o->carrier_arg) : s->meta.carrier_fix_hz;

    double fs_add = (c->mode == FE_MODE_NATIVE) ? fs : (c->mode == FE_MODE_ADC27 ? FE_FS_ADC_HZ : c->fs_out);
    if (o->have_cn0) {
        if (!(s->meta.sig_power > 0.0)) {
            fprintf(stderr, "source: --cn0 needs sig_power for %s in the manifest\n", base_name(s->path));
            return -1;
        }
        c->noise_sigma = fe_sigma_for_cn0(o->cn0, s->meta.sig_power, s->meta.noise_density, fs_add);
        if (c->noise_sigma < 0.0) {
            fprintf(stderr, "source: the file is already below %.1f dB-Hz\n", o->cn0);
            return -1;
        }
    } else if (o->have_noise) {
        c->noise_sigma = o->noise_sigma;
    }
    if (c->mode != FE_MODE_NATIVE && c->if_order > 0) {
        double rate = (c->mode == FE_MODE_ADC27) ? FE_FS_ADC_HZ : c->fs_out;
        if (c->if_bw_hz >= rate) {
            fprintf(stderr, "source: a %.2f MHz IF filter does not fit in a %.3f MS/s stream\n", c->if_bw_hz / 1e6,
                    rate / 1e6);
            return -1;
        }
    }
    if (fe_init(&s->fe, c) != 0) {
        fprintf(stderr, "source: front-end emulation setup failed\n");
        return -1;
    }
    s->fs_out = fe_out_rate(&s->fe);
    s->if_out = fe_out_if(&s->fe);
    return 0;
}

static int grow(void **p, size_t *cap, size_t need, size_t elem)
{
    if (need <= *cap) {
        return 0;
    }
    size_t n = *cap ? *cap : 4096;
    while (n < need) {
        n *= 2;
    }
    void *q = realloc(*p, n * elem);
    if (!q) {
        return -1;
    }
    *p = q;
    *cap = n;
    return 0;
}

/* Produces more output into the buffer (codes, or floats in native mode); returns 0 at the end. */
static int refill(src_t *s)
{
    if (s->eof) {
        return 0;
    }
    size_t blk = (size_t)(0.01 * s->f.fs);
    int64_t left = s->end - s->f.pos;
    if (left <= 0) {
        s->eof = 1;
        return 0;
    }
    if ((int64_t)blk > left) {
        blk = (size_t)left;
    }
    if (grow((void **)&s->in, &s->in_cap, 2 * blk, sizeof(float)) != 0) {
        return 0;
    }
    size_t got = iqf_read(&s->f, s->in, blk);
    if (got == 0) {
        s->eof = 1;
        return 0;
    }
    size_t mo = fe_max_out(&s->fe, got);
    if (s->fe.cfg.mode == FE_MODE_NATIVE) {
        if (s->head > 0) {
            memmove(s->buf, s->buf + 2 * s->head, sizeof(float) * 2 * (s->nbuf - s->head));
            s->nbuf -= s->head;
            s->head = 0;
        }
        if (grow((void **)&s->buf, &s->buf_cap, 2 * (s->nbuf + mo), sizeof(float)) != 0) {
            return 0;
        }
        s->nbuf += fe_process(&s->fe, s->in, got, s->buf + 2 * s->nbuf, NULL, mo);
    } else {
        if (s->chead > 0) {
            memmove(s->codes, s->codes + s->chead, s->ncodes - s->chead);
            s->ncodes -= s->chead;
            s->chead = 0;
        }
        if (grow((void **)&s->codes, &s->codes_cap, s->ncodes + mo, 1) != 0) {
            return 0;
        }
        s->ncodes += fe_process(&s->fe, s->in, got, NULL, s->codes + s->ncodes, mo);
    }
    return 1;
}

size_t src_read_codes(src_t *s, uint8_t *codes, size_t n)
{
    if (s->fe.cfg.mode == FE_MODE_NATIVE) {
        return 0;
    }
    size_t got = 0;
    while (got < n) {
        size_t avail = s->ncodes - s->chead;
        if (avail == 0) {
            if (!refill(s)) {
                break;
            }
            continue;
        }
        size_t take = n - got < avail ? n - got : avail;
        memcpy(codes + got, s->codes + s->chead, take);
        s->chead += take;
        got += take;
    }
    return got;
}

size_t src_read(src_t *s, float *iq, size_t n)
{
    size_t got = 0;
    if (s->fe.cfg.mode != FE_MODE_NATIVE) {
        uint8_t tmp[4096];
        while (got < n) {
            size_t want = n - got < sizeof(tmp) ? n - got : sizeof(tmp);
            size_t m = src_read_codes(s, tmp, want);
            for (size_t k = 0; k < m; k++) {
                unsigned x = tmp[k];
                float i = (x & FE_CODE_I_MAG) ? (float)FE_WEIGHT_LARGE : (float)FE_WEIGHT_SMALL;
                float q = (x & FE_CODE_Q_MAG) ? (float)FE_WEIGHT_LARGE : (float)FE_WEIGHT_SMALL;
                iq[2 * (got + k)] = (x & FE_CODE_I_SIGN) ? -i : i;
                iq[2 * (got + k) + 1] = (x & FE_CODE_Q_SIGN) ? -q : q;
            }
            got += m;
            if (m < want) {
                break;
            }
        }
        return got;
    }
    while (got < n) {
        size_t avail = s->nbuf - s->head;
        if (avail == 0) {
            if (!refill(s)) {
                break;
            }
            continue;
        }
        size_t take = n - got < avail ? n - got : avail;
        memcpy(iq + 2 * got, s->buf + 2 * s->head, sizeof(float) * 2 * take);
        s->head += take;
        got += take;
    }
    return got;
}

void src_close(src_t *s)
{
    fe_free(&s->fe);
    iqf_close(&s->f);
    free(s->in);
    free(s->buf);
    free(s->codes);
    memset(s, 0, sizeof(*s));
}
