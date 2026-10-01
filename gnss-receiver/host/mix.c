#include "mix.h"

#include <math.h>

#define TWO_PI 6.283185307179586476925286766559
#define TWO_POW_64 18446744073709551616.0

/* Re-anchors the rotator to the exact integer phase every this many samples. */
#define MIX_ANCHOR 1024

void mix_init(mix_t *m, double f_hz, double fs_hz, int64_t start)
{
    double r = f_hz / fs_hz;
    r -= floor(r + 0.5);                        /* [-0.5, 0.5) cycles per sample */
    double s = ldexp(r, 64);                    /* cycles * 2^64 */
    int64_t si = (s >= 9.2233720368547748e18) ? INT64_MAX : (int64_t)llround(s);
    m->step = (uint64_t)si;
    m->phase = m->step * (uint64_t)start;       /* wraps mod 2^64: exact */
    m->f_hz = (double)si / TWO_POW_64 * fs_hz;
}

static double phase_rad(uint64_t p)
{
    return (double)(int64_t)p / TWO_POW_64 * TWO_PI;
}

void mix_apply(mix_t *m, float *iq, size_t n)
{
    if (m->step == 0) {
        return;  /* phase stays 0: identity */
    }
    double w = phase_rad(m->step);
    double dc = cos(w), ds = sin(w);
    size_t k = 0;
    while (k < n) {
        size_t blk = n - k < MIX_ANCHOR ? n - k : MIX_ANCHOR;
        double ph = phase_rad(m->phase);
        double c = cos(ph), s = sin(ph);
        for (size_t j = 0; j < blk; j++) {
            double x = iq[2 * (k + j)], y = iq[2 * (k + j) + 1];
            iq[2 * (k + j)]     = (float)(x * c - y * s);
            iq[2 * (k + j) + 1] = (float)(x * s + y * c);
            double c2 = c * dc - s * ds;
            s = s * dc + c * ds;
            c = c2;
        }
        m->phase += m->step * (uint64_t)blk;
        k += blk;
    }
}
