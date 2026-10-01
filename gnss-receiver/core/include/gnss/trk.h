/*
 * One tracking channel: carrier and code loops on 1 ms correlator dumps, lock
 * detection, C/N0, bit synchronization and bit decisions. GPS L1 C/A for now.
 *
 * Loops run in float32, always relative to exact integer nominal NCO words (the
 * IF and the nominal chip rate), so float precision never touches the
 * absolute frequencies.
 *
 * Carrier: Costas PLL (3rd order), assisted by a 2nd-order FLL during pull-in;
 * the structure and coefficients of Kaplan & Hegarty, "Understanding GPS/GNSS"
 * (3rd-order PLL: w0 = Bn/0.7845, a3 = 1.1, b3 = 2.4; 2nd-order FLL:
 * w0 = Bn/0.53, a2 = 1.414). Code: a 1st-order DLL on the normalized early-minus-late
 * envelope, carrier aided. These are static-receiver settings; the boost loops are
 * milestone 5.
 */
#ifndef GNSS_TRK_H
#define GNSS_TRK_H

#include <stdint.h>

#include "gnss/corr_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TRK_OFF = 0,
    TRK_PULLIN,   /* FLL-assisted PLL, wide; ends when the PLL locks */
    TRK_LOCKED    /* PLL alone, narrower; bit sync and data */
} trk_state_t;

typedef struct {
    float fll_bw, pll_bw, dll_bw;  /* noise bandwidths, Hz (fll 0 = off) */
} trk_bw_t;

/* Loss of lock: C/N0 below this (dB-Hz) for TRK_LOSS_S seconds. */
#define TRK_LOSS_CN0  25.0f
#define TRK_LOSS_S    1.0f
/* C/N0 is estimated over this many dumps. */
#define TRK_CN0_N     200

typedef struct {
    trk_state_t state;
    uint8_t prn;
    float acq_metric;           /* the detection's peak / grid mean, for logs */
    float tap_chips;            /* early/late offset from prompt */
    float t_state;              /* seconds in the current state */
    uint32_t period;            /* dump seq of the last period processed */

    /* Nominal words (exact) and conversions. */
    int32_t if_word;
    uint64_t code_word0;
    float carr_k;               /* carrier words per Hz */
    float code_k;               /* code words per chip/s */

    /* Loops. */
    float dop_hz;               /* carrier frequency relative to the IF */
    float x1, x2;               /* carrier filter: frequency rate (rad/s^2), frequency (rad/s) */
    float dll_rate;             /* DLL correction, chips/s */
    float prev_ip, prev_qp;
    int have_prev;

    /* Lock and C/N0. */
    float pll_lock;             /* low-passed cos(2 * phase error) */
    float cn0;                  /* dB-Hz; 0 until the first estimate */
    float m2, m4;
    int nm;
    float t_weak;

    /* Bit sync and bits. */
    uint16_t hist[20];
    uint16_t n_trans;
    int bit_sync;
    uint8_t bit_phase;          /* periods p with p % 20 == bit_phase open a bit */
    float prev_sign;
    float bit_sum;
    uint8_t bit_n;
    uint32_t bit_first;         /* first period of the bit being summed */
} trk_ch_t;

/* Starts a channel from an acquisition: Doppler (Hz), nominal words for this stream. */
void trk_start(trk_ch_t *c, int prn, float dop_hz, float tap_chips, int32_t if_word, uint64_t code_word0,
               float carr_k, float code_k);

/*
 * Processes one dump (period d->seq, lasting T seconds). Returns 1 and fills
 * *bit (+1/-1) and *bit_period (the bit's first period) when a data bit
 * completes, else 0.
 */
int trk_update(trk_ch_t *c, const corr_dump_t *d, float T, int *bit, uint32_t *bit_period);

/* The NCO words the loops want next. */
void trk_words(const trk_ch_t *c, int32_t *carr_word, uint64_t *code_word);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_TRK_H */
