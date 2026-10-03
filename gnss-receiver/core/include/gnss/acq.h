/*
 * FFT acquisition of GPS L1 C/A on a snapshot of the correlator's input stream.
 *
 * The snapshot is mixed to baseband and block-averaged down to n_fft samples
 * per ms (2.048 MS/s by default), so every FFT is a power of two. Each 1 ms
 * block is transformed once per 250 Hz sub-bin; whole-kHz Doppler steps are
 * circular shifts of those spectra. Blocks are combined non-coherently.
 *
 * The caller provides the workspace (no malloc).
 */
#ifndef GNSS_ACQ_H
#define GNSS_ACQ_H

#include <stddef.h>
#include <stdint.h>

#include "gnss/fft.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ACQ_MAX_MS     20
#define ACQ_SUBBINS    4        /* Doppler step = 1000 / ACQ_SUBBINS Hz */
#define ACQ_MAX_FFT    4096

typedef struct {
    double fs;          /* snapshot rate; fs / 1000 must be a whole number */
    double if_hz;       /* where L1 sits in the snapshot */
    int n_fft;          /* samples per ms after decimation: 2048 or 4096 */
    int n_ms;           /* 1 ms blocks combined non-coherently, <= ACQ_MAX_MS */
    double dop_center;  /* centre of the Doppler search, Hz */
    double dop_max;     /* half-width of the search, Hz */
} acq_cfg_t;

typedef struct {
    int prn;
    float metric;       /* peak / mean of the search grid */
    double dop_hz;      /* Doppler: carrier = IF + dop */
    double code_phase;  /* chips in [0, 1023): the signal's code phase at the snapshot's first sample */
} acq_result_t;

typedef struct {
    acq_cfg_t cfg;
    fft_plan_t plan;
    const float *iq;    /* the full-rate snapshot (kept by the caller) */
    size_t n;
    uint64_t spms;      /* samples per ms */
    float *dec, *spec, *code_f, *buf, *acc;
} acq_t;

/* Floats of workspace for a configuration. */
size_t acq_work_floats(int n_fft, int n_ms);

/* Decimates the snapshot (n samples, interleaved I,Q, at least n_ms ms) and takes its spectra. */
int acq_prepare(acq_t *a, const acq_cfg_t *cfg, const float *iq, size_t n, float *work);

/* Searches one PRN in the prepared snapshot (decimated: code phase to ~0.3 chip, Doppler to 125 Hz). */
void acq_search(acq_t *a, int prn, acq_result_t *res);

/* Refines a detection on the full-rate snapshot: code phase to a few hundredths of a chip,
 * Doppler to ~20 Hz at 40 dB-Hz. The snapshot passed to acq_prepare must still exist. */
void acq_refine(acq_t *a, acq_result_t *res);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_ACQ_H */
