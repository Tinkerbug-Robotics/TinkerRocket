#include "resamp.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846264338328

/* Modified Bessel function of the first kind, order 0 (series). */
static double bessel_i0(double x)
{
    double sum = 1.0, term = 1.0, q = x * x / 4.0;
    for (int k = 1; k < 64; k++) {
        term *= q / ((double)k * (double)k);
        sum += term;
        if (term < sum * 1e-17) {
            break;
        }
    }
    return sum;
}

static double kaiser_beta(double atten_db)
{
    if (atten_db > 50.0) {
        return 0.1102 * (atten_db - 8.7);
    }
    if (atten_db >= 21.0) {
        return 0.5842 * pow(atten_db - 21.0, 0.4) + 0.07886 * (atten_db - 21.0);
    }
    return 0.0;
}

int resamp_init(resamp_t *r, double fs_in, double fs_out, double f_pass, double f_stop,
                double atten_db, int64_t start)
{
    memset(r, 0, sizeof(*r));
    if (!(fs_in > 0.0) || !(fs_out > 0.0) || !(f_stop > f_pass) || !(f_pass > 0.0)) {
        return -1;
    }
    double dw = 2.0 * PI * (f_stop - f_pass) / fs_in;
    int n = (int)ceil((atten_db - 7.95) / (2.285 * dw)) + 1;
    if (n < 8) {
        n = 8;
    }
    n += n & 1;
    r->ntaps = n;
    r->half = n / 2;
    r->step = fs_in / fs_out;
    r->fs_out = fs_out;

    double fc = 0.5 * (f_pass + f_stop) / fs_in;  /* cycles per input sample */
    double beta = kaiser_beta(atten_db);
    double i0b = bessel_i0(beta);
    r->h = (float *)malloc(sizeof(float) * (size_t)n * (RESAMP_PHASES + 1));
    r->taps = (float *)malloc(sizeof(float) * (size_t)n);
    if (!r->h || !r->taps) {
        resamp_free(r);
        return -1;
    }
    for (int p = 0; p <= RESAMP_PHASES; p++) {
        double mu = (double)p / RESAMP_PHASES;
        double row[1024];
        double sum = 0.0;
        if (n > 1024) {
            resamp_free(r);
            return -1;
        }
        for (int k = 0; k < n; k++) {
            /* Row stored time-reversed: tap k multiplies x[m - half + 1 + k]. */
            double t = mu + r->half - 1 - k;
            double u = t / r->half;
            double w = (fabs(u) <= 1.0) ? bessel_i0(beta * sqrt(1.0 - u * u)) / i0b : 0.0;
            double x = 2.0 * fc * t;
            double sinc = (fabs(x) < 1e-12) ? 1.0 : sin(PI * x) / (PI * x);
            row[k] = 2.0 * fc * sinc * w;
            sum += row[k];
        }
        for (int k = 0; k < n; k++) {
            r->h[(size_t)p * n + k] = (float)(row[k] / sum);  /* unit gain at DC for every phase */
        }
    }

    r->cap = 1 << 16;
    r->bi = (float *)malloc(sizeof(float) * r->cap);
    r->bq = (float *)malloc(sizeof(float) * r->cap);
    if (!r->bi || !r->bq) {
        resamp_free(r);
        return -1;
    }
    /* Zeros stand in for the samples before start. */
    r->len = (size_t)(r->half - 1);
    memset(r->bi, 0, sizeof(float) * r->len);
    memset(r->bq, 0, sizeof(float) * r->len);
    r->buf_start = start - (r->half - 1);
    r->tau_int = start;
    r->tau_frac = 0.0;
    return 0;
}

static int reserve(resamp_t *r, size_t n)
{
    if (r->len + n <= r->cap) {
        return 0;
    }
    /* Drop what the next output no longer needs. */
    int64_t keep_from = r->tau_int - r->half + 1;
    if (keep_from > r->buf_start) {
        size_t drop = (size_t)(keep_from - r->buf_start);
        if (drop > r->len) {
            drop = r->len;
        }
        memmove(r->bi, r->bi + drop, sizeof(float) * (r->len - drop));
        memmove(r->bq, r->bq + drop, sizeof(float) * (r->len - drop));
        r->len -= drop;
        r->buf_start += (int64_t)drop;
    }
    if (r->len + n > r->cap) {
        size_t cap = r->cap;
        while (cap < r->len + n) {
            cap *= 2;
        }
        float *bi = (float *)realloc(r->bi, sizeof(float) * cap);
        if (!bi) {
            return -1;
        }
        r->bi = bi;
        float *bq = (float *)realloc(r->bq, sizeof(float) * cap);
        if (!bq) {
            return -1;
        }
        r->bq = bq;
        r->cap = cap;
    }
    return 0;
}

size_t resamp_process(resamp_t *r, const float *in, size_t n, float *out, size_t max_out)
{
    if (n > 0) {
        if (reserve(r, n) != 0) {
            return 0;
        }
        float *bi = r->bi + r->len, *bq = r->bq + r->len;
        for (size_t k = 0; k < n; k++) {
            bi[k] = in[2 * k];
            bq[k] = in[2 * k + 1];
        }
        r->len += n;
    }
    const int nt = r->ntaps;
    const int64_t end = r->buf_start + (int64_t)r->len;  /* one past the newest sample */
    size_t nout = 0;
    while (nout < max_out) {
        int64_t m = r->tau_int;
        if (m + r->half >= end) {
            break;
        }
        double pos = r->tau_frac * RESAMP_PHASES;
        int p = (int)pos;
        if (p >= RESAMP_PHASES) {
            p = RESAMP_PHASES - 1;
        }
        float b = (float)(pos - p);
        const float *h0 = r->h + (size_t)p * nt;
        const float *h1 = h0 + nt;
        float *t = r->taps;
        for (int k = 0; k < nt; k++) {
            t[k] = h0[k] + b * (h1[k] - h0[k]);
        }
        size_t off = (size_t)(m - r->half + 1 - r->buf_start);
        const float *xi = r->bi + off, *xq = r->bq + off;
        float si = 0.0f, sq = 0.0f;
        for (int k = 0; k < nt; k++) {
            si += t[k] * xi[k];
            sq += t[k] * xq[k];
        }
        out[2 * nout] = si;
        out[2 * nout + 1] = sq;
        nout++;

        double s = r->step;
        if (r->warp) {
            s *= 1.0 + r->warp(r->warp_ctx, (double)r->out_count / r->fs_out);
        }
        r->out_count++;
        r->tau_frac += s;
        double whole = floor(r->tau_frac);
        r->tau_int += (int64_t)whole;
        r->tau_frac -= whole;
    }
    return nout;
}

double resamp_position(const resamp_t *r)
{
    return (double)r->tau_int + r->tau_frac;
}

void resamp_free(resamp_t *r)
{
    free(r->h);
    free(r->taps);
    free(r->bi);
    free(r->bq);
    memset(r, 0, sizeof(*r));
}
