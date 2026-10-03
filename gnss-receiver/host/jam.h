/*
 * Interference for the front-end emulation: a continuous wave, band-limited noise, or a swept
 * carrier, added at complex baseband (L1 at 0 Hz) with the thermal noise, ahead of the IF filter
 * and the MAX2769B's quantizer, where a real one would arrive. Its power is set against the
 * noise in the IF filter's bandwidth (JNR). Host only.
 */
#ifndef GNSS_HOST_JAM_H
#define GNSS_HOST_JAM_H

#include <stddef.h>
#include <stdint.h>

#include "iir.h"
#include "rng.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    JAM_CW = 0,     /* a carrier at f_hz */
    JAM_NB = 1,     /* Gaussian noise bw_hz wide (two-sided, 4th-order Butterworth) around f_hz */
    JAM_CHIRP = 2   /* a carrier swept linearly over f_hz +- bw_hz / 2, once per period_s, sawtooth */
} jam_type_t;

typedef struct {
    jam_type_t type;
    double f_hz;      /* relative to L1 */
    double jnr_db;    /* power against the noise in the IF filter's (two-sided) bandwidth */
    double bw_hz;
    double period_s;
} jam_cfg_t;

typedef struct {
    jam_cfg_t cfg;
    double fs;
    double amp;       /* CW, chirp: amplitude; NB: the scale on the filtered unit noise */
    double ph, dph;   /* cycles, cycles per sample (CW, chirp: dph moves) */
    double ddph;      /* chirp: dph per sample */
    double dph_lo, dph_hi;
    iir_t lpf;        /* NB */
    rng_t rng;
    float *buf;       /* NB: scratch */
    size_t buf_cap;
} jam_t;

/*
 * Power J = 10^(jnr_db/10) * n0 * bw_ref, with n0 the complex noise density (LSB^2/Hz) and
 * bw_ref the IF filter's bandwidth. Returns 0 on success.
 */
int jam_init(jam_t *j, const jam_cfg_t *c, double fs, double n0, double bw_ref, uint64_t seed);

/* Adds the interference to n interleaved I,Q samples. */
void jam_add(jam_t *j, float *iq, size_t n);

void jam_free(jam_t *j);

/* "cw:F_HZ:JNR_DB", "nb:F_HZ:JNR_DB:BW_HZ" or "chirp:F_HZ:JNR_DB:SPAN_HZ:PERIOD_S"; 0 on success. */
int jam_parse(const char *s, jam_cfg_t *c);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_JAM_H */
