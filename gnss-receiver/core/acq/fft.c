#include "gnss/fft.h"

#include <math.h>

#define TWO_PI 6.283185307179586476925286766559

int fft_plan_init(fft_plan_t *p, int n, float *tw_storage)
{
    if (n < 2 || (n & (n - 1)) != 0) {
        return -1;
    }
    p->n = n;
    p->log2n = 0;
    while ((1 << p->log2n) < n) {
        p->log2n++;
    }
    p->tw = tw_storage;
    for (int k = 0; k < n / 2; k++) {
        double a = -TWO_PI * (double)k / (double)n;
        p->tw[2 * k] = (float)cos(a);
        p->tw[2 * k + 1] = (float)sin(a);
    }
    return 0;
}

void fft_run(const fft_plan_t *p, float *x, int inverse)
{
    const int n = p->n;
    /* Bit-reversal permutation. */
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            float tr = x[2 * i], ti = x[2 * i + 1];
            x[2 * i] = x[2 * j];
            x[2 * i + 1] = x[2 * j + 1];
            x[2 * j] = tr;
            x[2 * j + 1] = ti;
        }
    }
    /* Butterflies. */
    const float sgn = inverse ? -1.0f : 1.0f;
    for (int len = 2; len <= n; len <<= 1) {
        int half = len >> 1, step = n / len;
        for (int i = 0; i < n; i += len) {
            for (int k = 0; k < half; k++) {
                float wr = p->tw[2 * k * step], wi = sgn * p->tw[2 * k * step + 1];
                float *a = x + 2 * (i + k), *b = x + 2 * (i + k + half);
                float br = b[0] * wr - b[1] * wi;
                float bi = b[0] * wi + b[1] * wr;
                b[0] = a[0] - br;
                b[1] = a[1] - bi;
                a[0] += br;
                a[1] += bi;
            }
        }
    }
}
