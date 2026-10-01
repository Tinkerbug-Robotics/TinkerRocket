/*
 * Float models of two candidate FPGA stages against narrowband interference, for the interference
 * study (milestone 7). Each runs on the 2-bit samples the correlators would get and requantizes
 * its output to 2 bits with its own AGC, so the correlators stay as they are. A bit-exact model
 * follows once one is chosen. Host only.
 *   ANF  adaptive notches in cascade, the single-pole complex form: a zero adapted by normalized
 *        LMS onto the interferer and a pole at k times it. With nothing to remove the zero
 *        shrinks toward 0 and the stage passes the signal unchanged.
 *   FDE  frequency-domain excision: N-point sqrt-Hann frames at 50 % overlap; the bins whose
 *        power, averaged over tau, stands k times over the median are zeroed. N samples late.
 */
#ifndef GNSS_HOST_MITIG_H
#define GNSS_HOST_MITIG_H

#include <stddef.h>
#include <stdint.h>

#include "gnss/fft.h"
#include "quant.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MIT_NONE = 0,
    MIT_ANF = 1,
    MIT_FDE = 2
} mit_type_t;

#define MIT_MAX_NOTCH 4

typedef struct {
    mit_type_t type;
    int n_notch;          /* ANF: notches in cascade, 1..MIT_MAX_NOTCH */
    double anf_k;         /* ANF: pole radius over the zero's (0.99: ~20 kHz wide at 6.75 MS/s) */
    double anf_mu;        /* ANF: normalized step */
    int fde_n;            /* FDE: points, a power of two */
    double fde_k;         /* FDE: threshold over the median bin power */
    double fde_tau_s;     /* FDE: bin power averaging */
    double mag_density;   /* the requantizer's AGC target */
} mit_cfg_t;

typedef struct {
    double zr, zi;        /* the zero */
    double ar, ai;        /* the pole section's last output */
    double p;             /* its power, averaged */
} mit_notch_t;

typedef struct {
    mit_cfg_t cfg;
    double fs;
    mit_notch_t notch[MIT_MAX_NOTCH];
    /* FDE */
    fft_plan_t plan;
    float *tw, *win, *hist, *frame, *ola, *pavg, *sel, *fifo;
    size_t hop, fill, fifo_n, fifo_cap, fifo_head;
    double avg_a;
    uint64_t frames, bins_cut;
    /* both */
    quant2_t q;
    float *y;
    size_t y_cap;
    uint64_t n;
} mit_t;

void mit_cfg_default(mit_cfg_t *c);

/* "none", "anf[:N[:K[:MU]]]" or "fde[:N[:K[:TAU_S]]]"; 0 on success. */
int mit_parse(const char *s, mit_cfg_t *c);

int mit_init(mit_t *m, const mit_cfg_t *c, double fs);

/* n 2-bit codes (fe_format.h) in, n out; in and out may be the same buffer. */
void mit_apply(mit_t *m, const uint8_t *in, uint8_t *out, size_t n);

/* One line on what it did: the notches' frequencies and depths, or the bins excised. */
void mit_report(const mit_t *m, char *buf, size_t len);

void mit_free(mit_t *m);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_HOST_MITIG_H */
