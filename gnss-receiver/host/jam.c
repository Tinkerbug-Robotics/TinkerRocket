#include "jam.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TWO_PI 6.283185307179586
#define ROT_BLOCK 4096   /* the rotator restarts from the exact phase this often */

int jam_init(jam_t *j, const jam_cfg_t *c, double fs, double n0, double bw_ref, uint64_t seed)
{
    memset(j, 0, sizeof(*j));
    j->cfg = *c;
    j->fs = fs;
    if (!(fs > 0.0) || !(n0 > 0.0) || !(bw_ref > 0.0)) {
        return -1;
    }
    const double p = pow(10.0, c->jnr_db / 10.0) * n0 * bw_ref;
    rng_seed(&j->rng, seed);
    j->dph = c->f_hz / fs;
    j->amp = sqrt(p);
    if (c->type == JAM_NB) {
        if (!(c->bw_hz > 0.0) || 0.5 * c->bw_hz >= 0.5 * fs ||
            iir_butter_lowpass(&j->lpf, 4, 0.5 * c->bw_hz, fs) != 0) {
            return -1;
        }
        /* The filter's power gain on unit complex noise, measured on a copy with its own stream. */
        iir_t f = j->lpf;
        rng_t r;
        rng_seed(&r, seed ^ 0x9e3779b97f4a7c15ull);
        const size_t nc = 1u << 17, skip = 4096;
        float *b = (float *)calloc(2 * nc, sizeof(float));
        if (!b) {
            return -1;
        }
        rng_add_noise(&r, b, nc, sqrt(0.5));
        iir_apply(&f, b, nc);
        double s = 0.0;
        for (size_t k = skip; k < nc; k++) {
            s += (double)b[2 * k] * b[2 * k] + (double)b[2 * k + 1] * b[2 * k + 1];
        }
        free(b);
        const double g = s / (double)(nc - skip);
        if (!(g > 0.0)) {
            return -1;
        }
        j->amp = sqrt(p / g);
    } else if (c->type == JAM_CHIRP) {
        if (!(c->bw_hz > 0.0) || !(c->period_s > 0.0)) {
            return -1;
        }
        j->dph_lo = (c->f_hz - 0.5 * c->bw_hz) / fs;
        j->dph_hi = (c->f_hz + 0.5 * c->bw_hz) / fs;
        j->dph = j->dph_lo;
        j->ddph = (j->dph_hi - j->dph_lo) / (c->period_s * fs);
    }
    return 0;
}

void jam_add(jam_t *j, float *iq, size_t n)
{
    if (j->cfg.type == JAM_CHIRP) {
        for (size_t k = 0; k < n; k++) {
            iq[2 * k] += (float)(j->amp * cos(TWO_PI * j->ph));
            iq[2 * k + 1] += (float)(j->amp * sin(TWO_PI * j->ph));
            j->ph += j->dph;
            j->ph -= floor(j->ph);
            j->dph += j->ddph;
            if (j->dph > j->dph_hi) {
                j->dph = j->dph_lo;  /* sawtooth */
            }
        }
        return;
    }
    const float *nb = NULL;
    if (j->cfg.type == JAM_NB) {
        if (n > j->buf_cap) {
            float *b = (float *)realloc(j->buf, 2 * n * sizeof(float));
            if (!b) {
                return;
            }
            j->buf = b;
            j->buf_cap = n;
        }
        memset(j->buf, 0, 2 * n * sizeof(float));
        rng_add_noise(&j->rng, j->buf, n, sqrt(0.5));
        iir_apply(&j->lpf, j->buf, n);
        nb = j->buf;
    }
    /* A rotator, restarted from the exact phase every block. */
    for (size_t k0 = 0; k0 < n; k0 += ROT_BLOCK) {
        const size_t m = n - k0 < ROT_BLOCK ? n - k0 : ROT_BLOCK;
        double zr = cos(TWO_PI * j->ph), zi = sin(TWO_PI * j->ph);
        const double wr = cos(TWO_PI * j->dph), wi = sin(TWO_PI * j->dph);
        for (size_t k = k0; k < k0 + m; k++) {
            double ar = j->amp, ai = 0.0;
            if (nb) {
                ar = j->amp * nb[2 * k];
                ai = j->amp * nb[2 * k + 1];
            }
            iq[2 * k] += (float)(ar * zr - ai * zi);
            iq[2 * k + 1] += (float)(ar * zi + ai * zr);
            const double t = zr * wr - zi * wi;
            zi = zr * wi + zi * wr;
            zr = t;
        }
        j->ph += (double)m * j->dph;
        j->ph -= floor(j->ph);
    }
}

void jam_free(jam_t *j)
{
    free(j->buf);
    memset(j, 0, sizeof(*j));
}

int jam_parse(const char *s, jam_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    char t[16];
    double a = 0.0, b = 0.0, d = 0.0, e = 0.0;
    const int n = sscanf(s, "%15[a-z]:%lf:%lf:%lf:%lf", t, &a, &b, &d, &e);
    if (n < 3) {
        return -1;
    }
    c->f_hz = a;
    c->jnr_db = b;
    if (!strcmp(t, "cw") && n == 3) {
        c->type = JAM_CW;
    } else if (!strcmp(t, "nb") && n == 4) {
        c->type = JAM_NB;
        c->bw_hz = d;
    } else if (!strcmp(t, "chirp") && n == 5) {
        c->type = JAM_CHIRP;
        c->bw_hz = d;
        c->period_s = e;
    } else {
        return -1;
    }
    return 0;
}
