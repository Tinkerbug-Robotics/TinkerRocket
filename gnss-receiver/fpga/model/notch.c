#include "notch.h"

#include "fe_format.h"

#include <math.h>
#include <string.h>

static inline int64_t rnd(int64_t v, int s)
{
    if (s <= 0) {
        return v * ((int64_t)1 << -s);
    }
    return (v + ((int64_t)1 << (s - 1))) >> s;
}

static inline int64_t sat(int64_t v)
{
    const int64_t lim = ((int64_t)1 << (NOTCH_IA + NOTCH_FA)) - 1;
    return v > lim ? lim : (v < -lim ? -lim : v);
}

static inline int msb(uint64_t v)
{
    int b = -1;
    while (v) {
        v >>= 1;
        b++;
    }
    return b;
}

void notch_init(notch_t *n)
{
    memset(n, 0, sizeof(*n));
    for (int s = 0; s < NOTCH_N; s++) {
        n->s[s].p = 1;
    }
    n->t = NOTCH_T0;
}

/* One notch: x in (NOTCH_FA fraction bits), y out. */
static void section(notch_sec_t *a, int64_t xr, int64_t xi, int64_t *yr, int64_t *yi)
{
    const int64_t zr = rnd(a->zr, NOTCH_G), zi = rnd(a->zi, NOTCH_G);
    const int64_t pr = rnd(zr * a->ar - zi * a->ai, NOTCH_FZ);
    const int64_t pi = rnd(zr * a->ai + zi * a->ar, NOTCH_FZ);
    const int64_t ar = sat(xr + pr - rnd(pr, NOTCH_K)), ai = sat(xi + pi - rnd(pi, NOTCH_K));
    *yr = sat(ar - pr);
    *yi = sat(ai - pi);
    a->p += rnd(a->ar * a->ar + a->ai * a->ai - a->p, NOTCH_PLEAK);
    if (a->p < 1) {
        a->p = 1;
    }
    const int sh = NOTCH_M + msb((uint64_t)a->p) - NOTCH_FZ - NOTCH_G;
    a->zr += rnd(*yr * a->ar + *yi * a->ai, sh);
    a->zi += rnd(*yi * a->ar - *yr * a->ai, sh);
    const int64_t one = (int64_t)1 << NOTCH_FZ, cap = one + (one >> (NOTCH_K + 1));
    if (zr * zr + zi * zi > cap * cap) {
        a->zr -= rnd(a->zr, 8);
        a->zi -= rnd(a->zi, 8);
    }
    a->ar = ar;
    a->ai = ai;
}

uint8_t notch_sample(notch_t *n, uint8_t code)
{
    const int64_t wi = ((code & FE_CODE_I_MAG) ? FE_WEIGHT_LARGE : FE_WEIGHT_SMALL) * ((code & FE_CODE_I_SIGN) ? -1 : 1);
    const int64_t wq = ((code & FE_CODE_Q_MAG) ? FE_WEIGHT_LARGE : FE_WEIGHT_SMALL) * ((code & FE_CODE_Q_SIGN) ? -1 : 1);
    n->pin += (uint64_t)(wi * wi + wq * wq);
    int64_t xr = wi * ((int64_t)1 << NOTCH_FA), xi = wq * ((int64_t)1 << NOTCH_FA), yr = 0, yi = 0;
    for (int s = 0; s < NOTCH_N; s++) {
        section(&n->s[s], xr, xi, &yr, &yi);
        xr = yr;
        xi = yi;
    }
    n->yr = yr;
    n->yi = yi;
    n->pout += (uint64_t)((yr * yr + yi * yi) >> (2 * NOTCH_FA));
    /* The requantizer. */
    unsigned c = 0;
    const int64_t ar = yr < 0 ? -yr : yr, ai = yi < 0 ? -yi : yi;
    if (yr < 0) {
        c |= FE_CODE_I_SIGN;
    }
    if (ar * ((int64_t)1 << NOTCH_TF) > n->t) {
        c |= FE_CODE_I_MAG;
        n->mags++;
    }
    if (yi < 0) {
        c |= FE_CODE_Q_SIGN;
    }
    if (ai * ((int64_t)1 << NOTCH_TF) > n->t) {
        c |= FE_CODE_Q_MAG;
        n->mags++;
    }
    if (++n->fill == NOTCH_CHUNK) {
        n->t += (n->t * (int64_t)(n->mags - NOTCH_TARGET)) >> NOTCH_AGC_SHIFT;
        if (n->t < 1) {
            n->t = 1;
        }
        n->fill = 0;
        n->mags = 0;
    }
    return (uint8_t)c;
}

void notch_process(notch_t *n, const uint8_t *in, uint8_t *out, size_t count)
{
    for (size_t k = 0; k < count; k++) {
        out[k] = notch_sample(n, in[k]);
    }
}

void notch_take_power(notch_t *n, uint64_t *pin, uint64_t *pout)
{
    *pin = n->pin;
    *pout = n->pout;
    n->pin = n->pout = 0;
}

void notch_zero(const notch_t *n, int s, double *f_cyc, double *depth)
{
    const double zr = (double)n->s[s].zr, zi = (double)n->s[s].zi;
    *f_cyc = atan2(zi, zr) / 6.283185307179586;
    *depth = sqrt(zr * zr + zi * zi) / (double)((int64_t)1 << (NOTCH_FZ + NOTCH_G));
}
