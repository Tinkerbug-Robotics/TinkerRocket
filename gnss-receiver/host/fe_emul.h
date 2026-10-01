/*
 * Front-end emulation: turns a recorded 8-bit I/Q file into the sample stream
 * the FPGA's correlators will see.
 *
 *   DIRECT (default)  DC and carrier fixes, L1 to 0 Hz, resample to 6.75 MS/s,
 *                     add noise to a set C/N0, the MAX2769B's IF filter,
 *                     L1 to the IF, 2-bit sign/magnitude with AGC.
 *   ADC27             the same at the MAX2769B's own 27 MS/s, then the FPGA's
 *                     decimator model (provisional until the hardware session
 *                     settles it).
 *   NATIVE            float at the file's own rate: the fixes and optional
 *                     noise only, no filter, no quantization.
 *
 * Host only.
 */
#ifndef GNSS_HOST_FE_EMUL_H
#define GNSS_HOST_FE_EMUL_H

#include <stddef.h>
#include <stdint.h>

#include "decim.h"
#include "iir.h"
#include "mix.h"
#include "quant.h"
#include "resamp.h"
#include "rng.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FE_MODE_NATIVE = 0,
    FE_MODE_DIRECT = 1,
    FE_MODE_ADC27  = 2
} fe_mode_t;

typedef struct {
    fe_mode_t mode;
    double fs_in, fc_in;     /* the file */
    int64_t start;           /* file sample index of the first input */
    double dc_i, dc_q;       /* offsets removed first, LSB */
    double carrier_fix_hz;   /* carrier-only shift (+22.0 for the rig's _cofs files) */
    double fs_out;           /* DIRECT: the correlator rate */
    double if_hz;            /* where L1 sits: in the output (DIRECT, NATIVE) or at the ADC (ADC27);
                                NAN (the default) = the frequency plan's, fe_format.h (NATIVE: 0) */
    double noise_sigma;      /* per-component noise added after resampling, file LSB; 0 = none */
    uint64_t seed;
    int if_order;            /* IF filter order, 0 = none */
    double if_bw_hz;         /* two-sided */
    double mag_density;      /* AGC target */
    double agc_tau_s;
    decim_mode_t decim_mode; /* ADC27 */
    int decim_phase;
} fe_cfg_t;

typedef struct {
    fe_cfg_t cfg;
    double fs_int;           /* rate after resampling: fs_out, 27 MS/s, or fs_in */
    double step;             /* input samples per internal sample */
    mix_t mix1, mix2;
    resamp_t rs;
    iir_t lpf;
    rng_t rng;
    quant2_t q;
    decim_t dec;
    float *work;
    size_t work_cap;
    uint8_t *codes;
    size_t codes_cap;
    int64_t n_in, n_out;
    double sumsq;            /* power into the quantizer, per component */
    uint64_t n_sumsq;
} fe_t;

void fe_cfg_default(fe_cfg_t *c);
int fe_init(fe_t *fe, const fe_cfg_t *cfg);

/*
 * Pushes n input samples (interleaved float I,Q in file units; modified in
 * place). NATIVE writes float samples to out_iq; the others write one code per
 * sample to out_codes. Returns the number written; size the output for
 * fe_max_out(fe, n).
 */
size_t fe_process(fe_t *fe, float *in, size_t n, float *out_iq, uint8_t *out_codes, size_t max_out);
size_t fe_max_out(const fe_t *fe, size_t n);

double fe_out_rate(const fe_t *fe);
/* IF of the output stream; for ADC27 the ADC's IF after aliasing by the decimation. */
double fe_out_if(const fe_t *fe);
/* Input position (file samples) of output sample k, without clock warp. */
double fe_out_position(const fe_t *fe, int64_t k);

/*
 * Per-component noise sigma to add at rate fs_add so that a satellite of
 * power sig_power (LSB^2) sits at cn0_dbhz, given the noise density already in
 * the file. Returns a negative value if the file is already below the target.
 */
double fe_sigma_for_cn0(double cn0_dbhz, double sig_power, double noise_density, double fs_add);

void fe_free(fe_t *fe);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_FE_EMUL_H */
