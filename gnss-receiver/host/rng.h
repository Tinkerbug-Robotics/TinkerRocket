/*
 * Reproducible noise for the front-end emulation: xoshiro256** (Blackman and
 * Vigna's public-domain generator, written here from its published
 * description) seeded by splitmix64, and Gaussian pairs by Marsaglia's polar
 * method. Host only.
 */
#ifndef GNSS_HOST_RNG_H
#define GNSS_HOST_RNG_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t s[4];
} rng_t;

void rng_seed(rng_t *r, uint64_t seed);
uint64_t rng_next(rng_t *r);

/* Uniform in [0, 1) with 53 random bits. */
double rng_uniform(rng_t *r);

/* Two independent N(0,1) values. */
void rng_gauss2(rng_t *r, double *a, double *b);

/* Adds complex white Gaussian noise, sigma per component, to n interleaved I,Q samples. */
void rng_add_noise(rng_t *r, float *iq, size_t n, double sigma);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_RNG_H */
