/*
 * Floating-point correlator bank behind core/include/gnss/corr_if.h. It keeps
 * the contract's integer NCOs and timing exactly (the same phases, words, epochs
 * and command latching as the FPGA will have) and differs from the bit-exact
 * model only in the arithmetic of the products and sums: float samples, a fine
 * sine table, float accumulators. It runs at any sample rate, so it also serves
 * native-rate runs. Host only.
 */
#ifndef GNSS_HOST_CORR_FLOAT_H
#define GNSS_HOST_CORR_FLOAT_H

#include <stddef.h>
#include <stdint.h>

#include "cmd_queue.h"
#include "gnss/corr_if.h"
#include "gnss/sig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CF_LUT_BITS 12

typedef struct {
    int active;
    int start_pending;
    corr_cmd_t start;
    cmdq_t q;                              /* tagged NCO commands */
    const float *code;      /* +-1 per chip */
    uint64_t code_mod;      /* code length << CORR_CODE_FRAC_BITS */
    uint32_t carr_phase;
    uint32_t carr_cycles;
    int32_t carr_word;
    uint64_t code_phase, code_word, tap;
    float acc[6];           /* ie qe ip qp il ql */
    uint32_t seq;
} cf_ch_t;

typedef struct {
    double fs;
    cf_ch_t ch[CORR_MAX_CH];
    float ca[GPS_MAX_PRN][GPS_CA_LEN];
    float cos_lut[1 << CF_LUT_BITS], sin_lut[1 << CF_LUT_BITS];
} corr_float_t;

void corr_float_init(corr_float_t *c, double fs);

/* Queues a command; returns -1 if it is malformed. */
int corr_float_command(corr_float_t *c, const corr_cmd_t *cmd);

/*
 * Correlates n samples (interleaved float I,Q) whose first sample is t0 on the
 * stream's sample count. Appends up to max dumps and returns how many.
 */
int corr_float_process(corr_float_t *c, uint64_t t0, const float *iq, size_t n, corr_dump_t *dumps, int max);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_CORR_FLOAT_H */
