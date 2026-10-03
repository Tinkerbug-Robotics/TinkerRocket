/*
 * The MAX2769B's 2-bit sign/magnitude ADC and its AGC. The chip's AGC servos
 * the density of magnitude bits to a target (about 1/3 by default); this
 * model moves the magnitude threshold the same way, once per chunk, in the log
 * domain. Output codes follow fpga/model/fe_format.h. Host only.
 */
#ifndef GNSS_HOST_QUANT_H
#define GNSS_HOST_QUANT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double thr;          /* magnitude threshold, input units (0 until the first chunk sets it) */
    double target;       /* magnitude-bit density the AGC holds */
    double gain;         /* log-threshold step per unit of density error, per chunk */
    size_t chunk;        /* samples per AGC update */
    size_t fill;         /* samples into the current chunk */
    size_t mag_count;    /* magnitude bits set in the current chunk (I and Q) */
    double sumsq;        /* for the first threshold */
    uint64_t total_mag, total_bits;
} quant2_t;

/* target e.g. 0.33; tau_samples = AGC time constant; chunk = samples per update. */
void quant2_init(quant2_t *q, double target, double tau_samples, size_t chunk);

/* One code per complex sample (FE_CODE_* bits). */
void quant2_apply(quant2_t *q, const float *iq, size_t n, uint8_t *codes);

/* Magnitude-bit density over everything quantized so far. */
double quant2_density(const quant2_t *q);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_QUANT_H */
