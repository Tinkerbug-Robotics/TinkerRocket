/*
 * The FPGA correlator bank, bit exact: the reference the HDL is verified
 * against. It implements core/include/gnss/corr_if.h on 2-bit sign/magnitude
 * sample codes (fe_format.h), with every width from corr_params.h.
 *
 * Per channel and per sample, exactly:
 *   sector  = carr_phase >> (32 - LUT bits)
 *   (c, s)  = cos_lut[sector], sin_lut[sector]
 *   (wi,wq) = sample weights (+-1, +-3)
 *   mi = wi * c + wq * s;  mq = wq * c - wi * s     (sample times exp(-j * phase))
 *   chips   = code[(code_phase + tap) >> F], code[code_phase >> F], code[(code_phase - tap) >> F]
 *             (wrapped modulo the code length; F = CORR_CODE_FRAC_BITS)
 *   acc[e|p|l][i|q] += chip ? -m : m
 *   carr_phase += carr_word (mod 2^32; the cycle counter follows wraps)
 *   code_phase += code_word; at or past code length: wrap, dump, apply latched commands
 *
 * The NCOs and the dump timing are the same integer arithmetic as the float
 * correlator (host/corr_float.c); tests hold the two to identical dump times
 * and phases.
 */
#ifndef GNSS_FPGA_CORR_MODEL_H
#define GNSS_FPGA_CORR_MODEL_H

#include <stddef.h>
#include <stdint.h>

#include "cmd_queue.h"
#include "gnss/corr_if.h"
#include "gnss/sig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CM_MAX_LUT_BITS 5

typedef struct {
    int lut_bits;                          /* carrier sectors = 2^lut_bits */
    int8_t cos_lut[1 << CM_MAX_LUT_BITS];
    int8_t sin_lut[1 << CM_MAX_LUT_BITS];
    int acc_bits;
} corr_model_cfg_t;

/* The configuration in corr_params.h. */
void corr_model_default_cfg(corr_model_cfg_t *c);

/* A table of 2^bits sectors with levels round(amp * cos/sin(centre angle)), for trade studies. */
int corr_model_lut(corr_model_cfg_t *c, int bits, double amp);

typedef struct {
    int active;
    int start_pending;
    corr_cmd_t start;
    cmdq_t q;                              /* tagged NCO commands */
    const uint8_t *code;                   /* chips, 0/1 */
    uint64_t code_mod;
    uint32_t carr_phase, carr_cycles;
    int32_t carr_word;
    uint64_t code_phase, code_word, tap;
    int32_t acc[6];                        /* ie qe ip qp il ql */
    uint32_t seq;
} cm_ch_t;

typedef struct {
    corr_model_cfg_t cfg;
    cm_ch_t ch[CORR_MAX_CH];
    uint8_t ca[GPS_MAX_PRN][GPS_CA_LEN];
    int8_t mix_i[16][1 << CM_MAX_LUT_BITS];  /* [sample nibble][sector] */
    int8_t mix_q[16][1 << CM_MAX_LUT_BITS];
    int32_t acc_peak;                      /* largest |accumulator| in any dump */
    uint32_t acc_overflows;                /* dumps with a value outside acc_bits */
} corr_model_t;

void corr_model_init(corr_model_t *m, const corr_model_cfg_t *cfg);
int corr_model_command(corr_model_t *m, const corr_cmd_t *cmd);

/* Correlates n sample codes whose first is sample t0; appends up to max dumps, returns how many. */
int corr_model_process(corr_model_t *m, uint64_t t0, const uint8_t *codes, size_t n, corr_dump_t *dumps, int max);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_FPGA_CORR_MODEL_H */
