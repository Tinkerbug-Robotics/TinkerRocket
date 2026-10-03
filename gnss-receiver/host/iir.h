/*
 * Butterworth low-pass by the bilinear transform, as a cascade of second-order
 * sections run on I and Q with the same real coefficients. Used at baseband to
 * model the MAX2769B's complex band-pass IF filter: a low-pass followed by a
 * shift to the IF is the same filter as a band-pass centred on the IF. Host only.
 */
#ifndef GNSS_HOST_IIR_H
#define GNSS_HOST_IIR_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IIR_MAX_SECTIONS 4

typedef struct {
    double b0, b1, b2, a1, a2;   /* a0 = 1 */
    double si[2], sq[2];         /* transposed direct form II state, per component */
} iir_sos_t;

typedef struct {
    int nsec;
    iir_sos_t sec[IIR_MAX_SECTIONS];
} iir_t;

/* order 1..8, -3 dB at fc_hz. Returns 0 on success. */
int iir_butter_lowpass(iir_t *f, int order, double fc_hz, double fs_hz);

/* Filters n interleaved I,Q samples in place. */
void iir_apply(iir_t *f, float *iq, size_t n);

/* |H(f)| of the designed filter, for tests. */
double iir_mag(const iir_t *f, double f_hz, double fs_hz);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_IIR_H */
