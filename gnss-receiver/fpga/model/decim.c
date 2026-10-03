#include "decim.h"
#include "fe_format.h"

/*
 * SUM4 threshold: for Gaussian input quantized at 1/3 magnitude density, one
 * sample's weighted value has variance 0.67*1 + 0.33*9 = 3.6, so the sum of
 * four has sigma ~3.8; a threshold of 4 keeps the magnitude density near 1/3.
 */
#define SUM4_DEFAULT_THR 4

static int code_value(unsigned sign_bit, unsigned mag_bit)
{
    int v = mag_bit ? FE_WEIGHT_LARGE : FE_WEIGHT_SMALL;
    return sign_bit ? -v : v;
}

static unsigned requant(int v, int thr, unsigned sign_mask, unsigned mag_mask)
{
    unsigned c = 0;
    if (v < 0) {
        c |= sign_mask;
        v = -v;
    }
    if (v >= thr) {
        c |= mag_mask;
    }
    return c;
}

void decim_init(decim_t *d, decim_mode_t mode, int phase)
{
    d->mode = mode;
    d->phase = phase & (FE_DECIM - 1);
    d->sum4_thr = SUM4_DEFAULT_THR;
    d->count = 0;
    d->acc_i = 0;
    d->acc_q = 0;
}

size_t decim_process(decim_t *d, const uint8_t *in, size_t n, uint8_t *out)
{
    size_t nout = 0;
    for (size_t k = 0; k < n; k++) {
        unsigned c = in[k];
        if (d->mode == DECIM_SUBSAMPLE) {
            if (d->count == d->phase) {
                out[nout++] = (uint8_t)c;
            }
        } else {
            d->acc_i += code_value(c & FE_CODE_I_SIGN, c & FE_CODE_I_MAG);
            d->acc_q += code_value(c & FE_CODE_Q_SIGN, c & FE_CODE_Q_MAG);
            if (d->count == FE_DECIM - 1) {
                out[nout++] = (uint8_t)(requant(d->acc_i, d->sum4_thr, FE_CODE_I_SIGN, FE_CODE_I_MAG) |
                                        requant(d->acc_q, d->sum4_thr, FE_CODE_Q_SIGN, FE_CODE_Q_MAG));
                d->acc_i = 0;
                d->acc_q = 0;
            }
        }
        d->count = (d->count + 1) & (FE_DECIM - 1);
    }
    return nout;
}
