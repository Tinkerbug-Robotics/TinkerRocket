#include "rng.h"

#include <math.h>

static uint64_t rotl(uint64_t x, int k)
{
    return (x << k) | (x >> (64 - k));
}

static uint64_t splitmix64(uint64_t *x)
{
    uint64_t z = (*x += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

void rng_seed(rng_t *r, uint64_t seed)
{
    uint64_t x = seed;
    for (int i = 0; i < 4; i++) {
        r->s[i] = splitmix64(&x);
    }
}

uint64_t rng_next(rng_t *r)
{
    uint64_t *s = r->s;
    uint64_t result = rotl(s[1] * 5, 7) * 9;
    uint64_t t = s[1] << 17;
    s[2] ^= s[0];
    s[3] ^= s[1];
    s[1] ^= s[2];
    s[0] ^= s[3];
    s[2] ^= t;
    s[3] = rotl(s[3], 45);
    return result;
}

double rng_uniform(rng_t *r)
{
    return (double)(rng_next(r) >> 11) * (1.0 / 9007199254740992.0);
}

void rng_gauss2(rng_t *r, double *a, double *b)
{
    double u, v, s;
    do {
        u = 2.0 * rng_uniform(r) - 1.0;
        v = 2.0 * rng_uniform(r) - 1.0;
        s = u * u + v * v;
    } while (s >= 1.0 || s == 0.0);
    double m = sqrt(-2.0 * log(s) / s);
    *a = u * m;
    *b = v * m;
}

void rng_add_noise(rng_t *r, float *iq, size_t n, double sigma)
{
    for (size_t k = 0; k < n; k++) {
        double a, b;
        rng_gauss2(r, &a, &b);
        iq[2 * k] += (float)(sigma * a);
        iq[2 * k + 1] += (float)(sigma * b);
    }
}
