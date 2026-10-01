#include "mitig.h"

#include "fe_format.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TWO_PI 6.283185307179586
#define AGC_CHUNK 256          /* as the front end's quantizer */
#define AGC_TAU_S 0.01
#define ANF_P_AVG (1.0 / 256)  /* the pole section's power average, per sample */

void mit_cfg_default(mit_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->type = MIT_NONE;
    c->n_notch = 1;
    c->anf_k = 0.99;
    c->anf_mu = 0.002;
    c->fde_n = 1024;
    c->fde_k = 8.0;
    c->fde_tau_s = 0.05;
    c->mag_density = 0.33;
    c->q_k = 7;
    c->q_m = 10;
    c->q_fz = 16;
    c->q_fa = 4;
    c->q_ia = 11;
    c->q_g = 12;
}

int mit_parse(const char *s, mit_cfg_t *c)
{
    mit_cfg_default(c);
    char t[8];
    double a = 0.0, b = 0.0, d = 0.0, e = 0.0, f = 0.0, g = 0.0, h = 0.0;
    const int n = sscanf(s, "%7[a-z]:%lf:%lf:%lf:%lf:%lf:%lf:%lf", t, &a, &b, &d, &e, &f, &g, &h);
    if (n < 1) {
        return -1;
    }
    if (!strcmp(t, "notch")) {
        c->type = MIT_NOTCH;
        return 0;
    }
    if (!strcmp(t, "anfq")) {
        c->type = MIT_ANFQ;
        if (n >= 2) {
            c->n_notch = (int)a;
        }
        if (n >= 3) {
            c->q_k = (int)b;
        }
        if (n >= 4) {
            c->q_m = (int)d;
        }
        if (n >= 5) {
            c->q_fz = (int)e;
        }
        if (n >= 6) {
            c->q_fa = (int)f;
        }
        if (n >= 7) {
            c->q_ia = (int)g;
        }
        if (n >= 8) {
            c->q_g = (int)h;
        }
        return c->n_notch < 1 || c->n_notch > MIT_MAX_NOTCH || c->q_k < 2 || c->q_k > 12 || c->q_fz < 8 ||
                       c->q_fz > 24 || c->q_fa < 0 || c->q_fa > 12 || c->q_ia < 6 || c->q_ia > 16
                   ? -1
                   : 0;
    }
    if (!strcmp(t, "none")) {
        c->type = MIT_NONE;
    } else if (!strcmp(t, "anf")) {
        c->type = MIT_ANF;
        if (n >= 2) {
            c->n_notch = (int)a;
        }
        if (n >= 3) {
            c->anf_k = b;
        }
        if (n >= 4) {
            c->anf_mu = d;
        }
        if (c->n_notch < 1 || c->n_notch > MIT_MAX_NOTCH || !(c->anf_k > 0.0 && c->anf_k < 1.0)) {
            return -1;
        }
    } else if (!strcmp(t, "fde")) {
        c->type = MIT_FDE;
        if (n >= 2) {
            c->fde_n = (int)a;
        }
        if (n >= 3) {
            c->fde_k = b;
        }
        if (n >= 4) {
            c->fde_tau_s = d;
        }
        if (c->fde_n < 16 || (c->fde_n & (c->fde_n - 1)) || !(c->fde_k > 1.0)) {
            return -1;
        }
    } else {
        return -1;
    }
    return 0;
}

int mit_init(mit_t *m, const mit_cfg_t *c, double fs)
{
    memset(m, 0, sizeof(*m));
    m->cfg = *c;
    m->fs = fs;
    quant2_init(&m->q, c->mag_density, AGC_TAU_S * fs, AGC_CHUNK);
    notch_init(&m->notch_fx);
    if (c->type != MIT_FDE) {
        return 0;
    }
    const size_t n = (size_t)c->fde_n;
    m->hop = n / 2;
    m->tw = (float *)calloc(n, sizeof(float));
    m->win = (float *)calloc(n, sizeof(float));
    m->hist = (float *)calloc(2 * n, sizeof(float));
    m->frame = (float *)calloc(2 * n, sizeof(float));
    m->ola = (float *)calloc(2 * n, sizeof(float));
    m->pavg = (float *)calloc(n, sizeof(float));
    m->sel = (float *)calloc(n, sizeof(float));
    m->fifo_cap = 4 * n;
    m->fifo = (float *)calloc(2 * m->fifo_cap, sizeof(float));
    if (!m->tw || !m->win || !m->hist || !m->frame || !m->ola || !m->pavg || !m->sel || !m->fifo ||
        fft_plan_init(&m->plan, (int)n, m->tw) != 0) {
        mit_free(m);
        return -1;
    }
    for (size_t k = 0; k < n; k++) {
        m->win[k] = (float)sqrt(0.5 * (1.0 - cos(TWO_PI * (double)k / (double)n)));  /* periodic: w^2 sums to 1 */
    }
    m->avg_a = (double)m->hop / (c->fde_tau_s * fs);
    if (m->avg_a > 1.0) {
        m->avg_a = 1.0;
    }
    m->fifo_n = m->hop;  /* the overlap-add's latency, as zeros up front */
    return 0;
}

static float kth_smallest(float *a, size_t n, size_t k)
{
    size_t lo = 0, hi = n - 1;
    while (lo < hi) {
        const float piv = a[(lo + hi) / 2];
        size_t i = lo, j = hi;
        while (i <= j) {
            while (a[i] < piv) {
                i++;
            }
            while (a[j] > piv) {
                j--;
            }
            if (i <= j) {
                const float t = a[i];
                a[i] = a[j];
                a[j] = t;
                i++;
                if (j == 0) {
                    break;
                }
                j--;
            }
        }
        if (k <= j) {
            hi = j;
        } else if (k >= i) {
            lo = i;
        } else {
            break;
        }
    }
    return a[k];
}

/* One frame from the history: window, transform, excise, back, window, overlap-add; then the
 * finished half goes to the FIFO. */
static void fde_frame(mit_t *m)
{
    const size_t n = (size_t)m->cfg.fde_n, hop = m->hop;
    for (size_t k = 0; k < n; k++) {
        m->frame[2 * k] = m->hist[2 * k] * m->win[k];
        m->frame[2 * k + 1] = m->hist[2 * k + 1] * m->win[k];
    }
    fft_run(&m->plan, m->frame, 0);
    for (size_t k = 0; k < n; k++) {
        const float p = m->frame[2 * k] * m->frame[2 * k] + m->frame[2 * k + 1] * m->frame[2 * k + 1];
        m->pavg[k] = m->frames ? m->pavg[k] + (float)m->avg_a * (p - m->pavg[k]) : p;
        m->sel[k] = m->pavg[k];
    }
    const float thr = (float)m->cfg.fde_k * kth_smallest(m->sel, n, n / 2);
    for (size_t k = 0; k < n; k++) {
        if (m->pavg[k] > thr) {
            m->frame[2 * k] = 0.0f;
            m->frame[2 * k + 1] = 0.0f;
            m->bins_cut++;
        }
    }
    m->frames++;
    fft_run(&m->plan, m->frame, 1);
    const float s = 1.0f / (float)n;
    for (size_t k = 0; k < n; k++) {
        m->ola[2 * k] += m->frame[2 * k] * m->win[k] * s;
        m->ola[2 * k + 1] += m->frame[2 * k + 1] * m->win[k] * s;
    }
    /* The first half is complete: out to the FIFO, and the accumulator moves up. */
    for (size_t k = 0; k < hop; k++) {
        const size_t w = (m->fifo_head + m->fifo_n) % m->fifo_cap;
        m->fifo[2 * w] = m->ola[2 * k];
        m->fifo[2 * w + 1] = m->ola[2 * k + 1];
        m->fifo_n++;
    }
    memmove(m->ola, m->ola + 2 * hop, 2 * (n - hop) * sizeof(float));
    memset(m->ola + 2 * (n - hop), 0, 2 * hop * sizeof(float));
}

/* Arithmetic shift right by s with rounding (s >= 0), or left. */
static inline int64_t q_shr(int64_t x, int s)
{
    if (s <= 0) {
        return x * ((int64_t)1 << -s);
    }
    return (x + ((int64_t)1 << (s - 1))) >> s;
}

static inline int64_t q_sat(int64_t x, int64_t lim)
{
    return x > lim ? lim : (x < -lim ? -lim : x);
}

static inline int q_msb(uint64_t x)
{
    int b = -1;
    while (x) {
        x >>= 1;
        b++;
    }
    return b;
}

/*
 * One sample through a fixed-point notch: x (the sample weight, integer) in, y out with q_fa
 * fraction bits. Integers throughout; every product fits an 18 x 18 multiplier for the default
 * widths (z: 1 + 1 + 16 bits; the pole state: 1 + 11 + 4).
 *   pole:  p = z ar[-1] (>> fz);  ar = x << fa + p - (p >> k);  y = ar - p
 *   zero:  z += y conj(ar[-1]) >> (m + msb(P) - fz), P the pole state's power, leaky (2^-8)
 *   |z| held under 1 + 2^-(k+1), so k |z| < 1 and the pole stays inside the circle
 */
static void anfq_step(const mit_cfg_t *c, mit_notch_t *a, int64_t *xr, int64_t *xi, int in_frac)
{
    const int fz = c->q_fz, fa = c->q_fa, g = c->q_g;
    const int64_t lim = ((int64_t)1 << (c->q_ia + fa)) - 1;
    const int64_t zr = q_shr(a->qzr, g), zi = q_shr(a->qzi, g);  /* the multiplier's z: Q1.fz */
    const int64_t pr = q_shr(zr * a->qar - zi * a->qai, fz);
    const int64_t pi = q_shr(zr * a->qai + zi * a->qar, fz);
    const int64_t xin_r = q_shr(*xr, in_frac - fa), xin_i = q_shr(*xi, in_frac - fa);
    const int64_t ar = q_sat(xin_r + pr - q_shr(pr, c->q_k), lim);
    const int64_t ai = q_sat(xin_i + pi - q_shr(pi, c->q_k), lim);
    const int64_t yr = q_sat(ar - pr, lim), yi = q_sat(ai - pi, lim);
    /* The pole state's power (2 fa fraction bits), leaky, for the step's normalization. */
    const int64_t pw = a->qar * a->qar + a->qai * a->qai;
    a->qp += q_shr(pw - a->qp, 8);
    if (a->qp < 1) {
        a->qp = 1;
    }
    const int sh = c->q_m + q_msb((uint64_t)a->qp) - fz - g;
    a->qzr += q_shr(yr * a->qar + yi * a->qai, sh);
    a->qzi += q_shr(yi * a->qar - yr * a->qai, sh);
    /* Hold |z| under 1 + 2^-(k+1): shrink it by 2^-8 when over (tested on the multiplier's z). */
    const int64_t one = (int64_t)1 << fz, m2 = zr * zr + zi * zi;
    const int64_t cap = one + (one >> (c->q_k + 1));
    if (m2 > cap * cap) {
        a->qzr -= q_shr(a->qzr, 8);
        a->qzi -= q_shr(a->qzi, 8);
    }
    a->qar = ar;
    a->qai = ai;
    *xr = yr;
    *xi = yi;
}

static void vec_nibble(FILE *f, uint8_t *hi, uint64_t k, uint8_t code)
{
    if ((k & 1) == 0) {
        *hi = (uint8_t)(code << 4);
    } else {
        const uint8_t b = (uint8_t)(*hi | (code & 0xF));
        fwrite(&b, 1, 1, f);
    }
}

int mit_set_vectors(mit_t *m, const char *dir, uint64_t n_samples, uint64_t ms_len)
{
    if (m->cfg.type != MIT_NOTCH) {
        return -1;
    }
    char p[4096];
    snprintf(p, sizeof(p), "%s/notch_in.u2", dir);
    m->vec_in = fopen(p, "wb");
    snprintf(p, sizeof(p), "%s/notch_out.u2", dir);
    m->vec_out = fopen(p, "wb");
    snprintf(p, sizeof(p), "%s/notch_power.csv", dir);
    m->vec_pow = fopen(p, "w");
    if (!m->vec_in || !m->vec_out || !m->vec_pow) {
        return -1;
    }
    snprintf(p, sizeof(p), "%s/notch.ini", dir);
    FILE *fi = fopen(p, "w");
    if (fi) {
        fprintf(fi, "[notch]\nms_samples = %llu\nn = %d\nk = %d\nm = %d\nfz = %d\ng = %d\nfa = %d\nia = %d\n"
                    "pleak = %d\ntf = %d\nchunk = %d\ntarget = %d\nagc_shift = %d\nt0 = %d\n",
                (unsigned long long)ms_len, NOTCH_N, NOTCH_K, NOTCH_M, NOTCH_FZ, NOTCH_G, NOTCH_FA, NOTCH_IA, NOTCH_PLEAK,
                NOTCH_TF, NOTCH_CHUNK, NOTCH_TARGET, NOTCH_AGC_SHIFT, NOTCH_T0);
        fclose(fi);
    }
    fprintf(m->vec_pow, "ms,pin,pout,t");
    for (int s = 0; s < NOTCH_N; s++) {
        fprintf(m->vec_pow, ",zr%d,zi%d", s, s);
    }
    fprintf(m->vec_pow, "\n");
    m->vec_end = n_samples & ~(uint64_t)1;
    m->vec_ms_len = ms_len;
    m->vec_n = 0;
    return 0;
}

static void vec_close(mit_t *m)
{
    if (m->vec_in) {
        fclose(m->vec_in);
        fclose(m->vec_out);
        fclose(m->vec_pow);
        m->vec_in = m->vec_out = NULL;
        m->vec_pow = NULL;
    }
}

void mit_apply(mit_t *m, const uint8_t *in, uint8_t *out, size_t n)
{
    if (m->cfg.type == MIT_NONE) {
        if (out != in) {
            memcpy(out, in, n);
        }
        return;
    }
    if (m->cfg.type == MIT_NOTCH) {
        /* The FPGA's stage itself: integer in, integer out. The host's power sums follow its
         * counters (weights in, y out), for the interference flag. */
        for (size_t k = 0; k < n; k++) {
            const uint8_t c = in[k];
            const uint8_t o = notch_sample(&m->notch_fx, c);
            out[k] = o;
            if (m->vec_in && m->vec_n < m->vec_end) {
                vec_nibble(m->vec_in, &m->vec_hi_in, m->vec_n, c);
                vec_nibble(m->vec_out, &m->vec_hi_out, m->vec_n, o);
                if ((m->vec_n + 1) % m->vec_ms_len == 0) {
                    uint64_t pi, po;
                    notch_take_power(&m->notch_fx, &pi, &po);
                    m->pin += (double)pi;
                    m->pout += (double)po;
                    fprintf(m->vec_pow, "%llu,%llu,%llu,%lld", (unsigned long long)((m->vec_n + 1) / m->vec_ms_len),
                            (unsigned long long)pi, (unsigned long long)po, (long long)m->notch_fx.t);
                    for (int s = 0; s < NOTCH_N; s++) {
                        fprintf(m->vec_pow, ",%lld,%lld", (long long)m->notch_fx.s[s].zr, (long long)m->notch_fx.s[s].zi);
                    }
                    fprintf(m->vec_pow, "\n");
                }
                if (++m->vec_n == m->vec_end) {
                    vec_close(m);
                }
            }
        }
        m->n += n;
        return;
    }
    if (n > m->y_cap) {
        float *y = (float *)realloc(m->y, 2 * n * sizeof(float));
        if (!y) {
            return;
        }
        m->y = y;
        m->y_cap = n;
    }
    for (size_t k = 0; k < n; k++) {
        const unsigned c = in[k];
        const double xi0 = (c & FE_CODE_I_MAG) ? FE_WEIGHT_LARGE : FE_WEIGHT_SMALL;
        const double xq0 = (c & FE_CODE_Q_MAG) ? FE_WEIGHT_LARGE : FE_WEIGHT_SMALL;
        double xr = (c & FE_CODE_I_SIGN) ? -xi0 : xi0, xi = (c & FE_CODE_Q_SIGN) ? -xq0 : xq0;
        m->pin += xr * xr + xi * xi;
        if (m->cfg.type == MIT_ANFQ) {
            int64_t qr = (int64_t)xr, qi = (int64_t)xi;
            int frac = 0;
            for (int s2 = 0; s2 < m->cfg.n_notch; s2++) {
                anfq_step(&m->cfg, &m->notch[s2], &qr, &qi, frac);
                frac = m->cfg.q_fa;
            }
            const double sc = 1.0 / (double)((int64_t)1 << m->cfg.q_fa);
            m->y[2 * k] = (float)((double)qr * sc);
            m->y[2 * k + 1] = (float)((double)qi * sc);
        } else if (m->cfg.type == MIT_ANF) {
            for (int s = 0; s < m->cfg.n_notch; s++) {
                mit_notch_t *a = &m->notch[s];
                /* Pole section, then the zero: y = x_ar - z x_ar[-1], x_ar = x + k z x_ar[-1]. */
                const double pr = a->zr * a->ar - a->zi * a->ai, pi = a->zr * a->ai + a->zi * a->ar;
                const double ar = xr + m->cfg.anf_k * pr, ai = xi + m->cfg.anf_k * pi;
                const double yr = ar - pr, yi = ai - pi;
                /* Normalized LMS on |y|^2: z += mu y conj(x_ar[-1]) / P. */
                const double pa = a->ar * a->ar + a->ai * a->ai;
                a->p = a->p > 0.0 ? a->p + ANF_P_AVG * (pa - a->p) : pa;
                const double g = m->cfg.anf_mu / (a->p + 1e-9);
                a->zr += g * (yr * a->ar + yi * a->ai);
                a->zi += g * (yi * a->ar - yr * a->ai);
                const double z2 = a->zr * a->zr + a->zi * a->zi;
                if (z2 > 1.0) {
                    const double r = 1.0 / sqrt(z2);
                    a->zr *= r;
                    a->zi *= r;
                }
                a->ar = ar;
                a->ai = ai;
                xr = yr;
                xi = yi;
            }
            m->y[2 * k] = (float)xr;
            m->y[2 * k + 1] = (float)xi;
        } else {
            /* FDE: into the history; a frame every hop samples; out of the FIFO. */
            const size_t nn = (size_t)m->cfg.fde_n;
            m->hist[2 * (nn - m->hop + m->fill)] = (float)xr;
            m->hist[2 * (nn - m->hop + m->fill) + 1] = (float)xi;
            if (++m->fill == m->hop) {
                fde_frame(m);
                memmove(m->hist, m->hist + 2 * m->hop, 2 * (nn - m->hop) * sizeof(float));
                m->fill = 0;
            }
            m->y[2 * k] = m->fifo[2 * m->fifo_head];
            m->y[2 * k + 1] = m->fifo[2 * m->fifo_head + 1];
            m->fifo_head = (m->fifo_head + 1) % m->fifo_cap;
            m->fifo_n--;
        }
    }
    for (size_t k = 0; k < n; k++) {
        m->pout += (double)m->y[2 * k] * m->y[2 * k] + (double)m->y[2 * k + 1] * m->y[2 * k + 1];
    }
    quant2_apply(&m->q, m->y, n, out);
    m->n += n;
}

double mit_take_suppression_db(mit_t *m)
{
    if (m->cfg.type == MIT_NOTCH && !m->vec_in) {
        uint64_t pi, po;
        notch_take_power(&m->notch_fx, &pi, &po);
        m->pin += (double)pi;
        m->pout += (double)po;
    }
    const double r = m->pin > 0.0 && m->pout > 0.0 ? 10.0 * log10(m->pin / m->pout) : 0.0;
    m->pin = m->pout = 0.0;
    return r;
}

void mit_report(const mit_t *m, char *buf, size_t len)
{
    if (m->cfg.type == MIT_ANF) {
        int w = snprintf(buf, len, "ANF k %.4f mu %g:", m->cfg.anf_k, m->cfg.anf_mu);
        for (int s = 0; s < m->cfg.n_notch && w > 0 && (size_t)w < len; s++) {
            const mit_notch_t *a = &m->notch[s];
            w += snprintf(buf + w, len - (size_t)w, " notch %d at %+.1f kHz, |z| %.3f;", s,
                          atan2(a->zi, a->zr) / TWO_PI * m->fs / 1e3, sqrt(a->zr * a->zr + a->zi * a->zi));
        }
    } else if (m->cfg.type == MIT_ANFQ) {
        int w = snprintf(buf, len, "ANFQ k 1-2^-%d step 2^-%d z Q1.%d state %d.%d:", m->cfg.q_k, m->cfg.q_m, m->cfg.q_fz,
                         m->cfg.q_ia, m->cfg.q_fa);
        for (int s = 0; s < m->cfg.n_notch && w > 0 && (size_t)w < len; s++) {
            const mit_notch_t *a = &m->notch[s];
            const double zr = (double)a->qzr, zi = (double)a->qzi;
            w += snprintf(buf + w, len - (size_t)w, " notch %d at %+.1f kHz, |z| %.3f;", s,
                          atan2(zi, zr) / TWO_PI * m->fs / 1e3,
                          sqrt(zr * zr + zi * zi) / (double)((int64_t)1 << (m->cfg.q_fz + m->cfg.q_g)));
        }
    } else if (m->cfg.type == MIT_NOTCH) {
        int w = snprintf(buf, len, "NOTCH (bit exact, %d in cascade):", NOTCH_N);
        for (int s = 0; s < NOTCH_N && w > 0 && (size_t)w < len; s++) {
            double f, d;
            notch_zero(&m->notch_fx, s, &f, &d);
            w += snprintf(buf + w, len - (size_t)w, " notch %d at %+.1f kHz, |z| %.3f;", s, f * m->fs / 1e3, d);
        }
    } else if (m->cfg.type == MIT_FDE) {
        snprintf(buf, len, "FDE %d points (%.2f kHz bins), k %g, tau %g s: %.2f bins excised per frame", m->cfg.fde_n,
                 m->fs / m->cfg.fde_n / 1e3, m->cfg.fde_k, m->cfg.fde_tau_s,
                 m->frames ? (double)m->bins_cut / (double)m->frames : 0.0);
    } else {
        snprintf(buf, len, "none");
    }
}

void mit_free(mit_t *m)
{
    vec_close(m);
    free(m->tw);
    free(m->win);
    free(m->hist);
    free(m->frame);
    free(m->ola);
    free(m->pavg);
    free(m->sel);
    free(m->fifo);
    free(m->y);
    memset(m, 0, sizeof(*m));
}
