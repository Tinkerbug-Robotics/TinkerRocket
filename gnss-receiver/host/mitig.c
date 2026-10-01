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
}

int mit_parse(const char *s, mit_cfg_t *c)
{
    mit_cfg_default(c);
    char t[8];
    double a = 0.0, b = 0.0, d = 0.0;
    const int n = sscanf(s, "%7[a-z]:%lf:%lf:%lf", t, &a, &b, &d);
    if (n < 1) {
        return -1;
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

void mit_apply(mit_t *m, const uint8_t *in, uint8_t *out, size_t n)
{
    if (m->cfg.type == MIT_NONE) {
        if (out != in) {
            memcpy(out, in, n);
        }
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
        if (m->cfg.type == MIT_ANF) {
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
