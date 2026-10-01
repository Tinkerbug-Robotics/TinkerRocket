/*
 * The FPGA's 27 -> 6.75 MS/s step, bit-exact on sample codes. PROVISIONAL: the
 * hardware session has not chosen the decimator yet, so the variants are
 * swappable and the stage-0 default path (host/fe_emul.c, FE_MODE_DIRECT)
 * does not use this at all.
 */
#ifndef GNSS_FPGA_DECIM_H
#define GNSS_FPGA_DECIM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DECIM_SUBSAMPLE = 0,  /* keep every 4th sample: no filter, the noise between bands folds in */
    DECIM_SUM4      = 1   /* sum 4 weighted samples, requantize to 2-bit sign/magnitude */
} decim_mode_t;

typedef struct {
    decim_mode_t mode;
    int phase;       /* SUBSAMPLE: which of the 4 samples is kept */
    int sum4_thr;    /* SUM4: |sum| >= this sets the magnitude bit */
    int count;       /* samples since the last output */
    int acc_i, acc_q;
} decim_t;

void decim_init(decim_t *d, decim_mode_t mode, int phase);

/* Consumes n codes at 27 MS/s; writes one code per 4 inputs; returns outputs written. */
size_t decim_process(decim_t *d, const uint8_t *in, size_t n, uint8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_FPGA_DECIM_H */
