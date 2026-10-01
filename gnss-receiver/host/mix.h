/*
 * Exact frequency shift of a complex stream. The phase is a 64-bit integer
 * count of cycles/2^64 tied to the absolute sample index, so a run that starts
 * mid-file mixes with the same phase as a run from the beginning. Host only.
 */
#ifndef GNSS_HOST_MIX_H
#define GNSS_HOST_MIX_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t phase;  /* cycles * 2^64 at the next sample */
    uint64_t step;   /* per sample */
    double f_hz;     /* the frequency actually applied (step quantized) */
} mix_t;

/* Shift by f_hz at sample rate fs_hz; the first sample processed has absolute index start. */
void mix_init(mix_t *m, double f_hz, double fs_hz, int64_t start);

/* Multiplies n interleaved I,Q samples by exp(j*2*pi*f*t), in place. */
void mix_apply(mix_t *m, float *iq, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_MIX_H */
