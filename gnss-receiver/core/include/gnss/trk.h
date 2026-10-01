/*
 * One tracking channel: carrier and code loops on 1 ms correlator dumps, lock
 * detection, C/N0, bit synchronization and bit decisions. GPS L1 C/A for now.
 *
 * Loops run in float32, always relative to exact integer nominal NCO words (the
 * IF and the nominal chip rate), so float precision never touches the
 * absolute frequencies.
 *
 * Carrier: Costas PLL (3rd order), assisted by a 2nd-order FLL; the structure
 * and coefficients of Kaplan & Hegarty, "Understanding GPS/GNSS" (3rd-order PLL:
 * w0 = Bn/0.7845, a3 = 1.1, b3 = 2.4; 2nd-order FLL: w0 = Bn/0.53, a2 = 1.414).
 * Code: a 1st-order DLL on the normalized early-minus-late envelope, carrier
 * aided. The bandwidths come from a profile (trk_profile_t) the receiver picks
 * for its dynamics: quiet loops at rest, wide ones through a boost (milestone 5,
 * README "Boost dynamics").
 *
 * A channel whose PLL lets go keeps its code, frequency and bit sync under the
 * FLL, and its measurements stay valid but for the carrier phase. One whose
 * C/N0 falls below TRK_LOSS_CN0 coasts on its last frequency, and is dropped
 * after TRK_LOSS_S.
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

/* The loops' bandwidths in each state. The receiver picks a profile for its dynamics (rx.h). */
typedef struct {
    trk_bw_t pullin;            /* until the PLL locks, and after it loses lock */
    trk_bw_t locked;
    /*
     * FLL integration, ms (1, 2, 4, 5 or 10): the FLL compares consecutive blocks of this many
     * dumps, so its noise falls with the block length. Once bits are synchronized the blocks sit
     * inside a data bit and the discriminator is a full atan2, +-1 / (2 fll_ms ms); before that
     * it is folded for data bits, +-1 / (4 fll_ms ms).
     */
    uint8_t fll_ms;
} trk_profile_t;

/* A receiver at rest: the stage-0 settings, with a 2 ms FLL block. */
extern const trk_profile_t trk_profile_quiet;
/* Under a boost (PROPOSED, milestone 5): a 50 Hz PLL rides a burnout's step in Doppler rate. */
extern const trk_profile_t trk_profile_boost;

/*
 * Moves the profile in force, *cur, toward *target over dt seconds: wider bandwidths at once,
 * narrower ones with time constant TRK_NARROW_TAU. Stepping a wide loop's state straight into a
 * narrow loop hands it the wide loop's frequency noise, and the narrow PLL slips on it.
 */
#define TRK_NARROW_TAU 0.5f
void trk_profile_step(trk_profile_t *cur, const trk_profile_t *target, float dt);

/* Loss of lock: C/N0 below this (dB-Hz) for TRK_LOSS_S seconds; the loops coast meanwhile. */
#define TRK_LOSS_CN0  25.0f
#define TRK_LOSS_S    1.0f
/* Before bit sync, C/N0 is estimated over this many dumps (moments); after it, over 10 bits. */
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
    float ff_rate;              /* aiding: the line of sight's predicted Doppler rate, Hz/s (0 = none). It
                                   drives the frequency directly, so the loop tracks only what it misses */
    float x1, x2;               /* carrier filter: frequency rate (rad/s^2), frequency (rad/s) */
    float dll_rate;             /* DLL correction, chips/s */
    float prev_ip, prev_qp;
    int have_prev;
    float blk_i, blk_q;         /* the FLL's block being summed, and the one before */
    float blk_prev_i, blk_prev_q;
    uint32_t blk_n;             /* dumps in the block so far */
    int blk_have_prev;
    int blk_aligned;            /* blocks are aligned to data bits */

    /* Lock and C/N0. */
    float pll_lock;             /* cos(2 * phase error), low-passed and noise-corrected */
    float nbd, nbp;             /* low-passed I^2 - Q^2 and I^2 + Q^2 */
    float pn;                   /* noise power per dump, from the last C/N0 estimate (0 before it) */
    float nw_i, nw_q, nw_wp;    /* narrowband/wideband C/N0 (after bit sync): this bit's sums */
    uint32_t nw_n;
    float nw_mu, nw_wsum;       /* and the bits averaged so far */
    int nw_k;
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
 * Processes one dump (period d->seq, lasting T seconds) with the loops of
 * profile p. Returns 1 and fills *bit (+1/-1) and *bit_period (the bit's first
 * period) when a data bit completes, else 0.
 */
int trk_update(trk_ch_t *c, const trk_profile_t *p, const corr_dump_t *d, float T, int *bit, uint32_t *bit_period);

/* The NCO words the loops want next. */
void trk_words(const trk_ch_t *c, int32_t *carr_word, uint64_t *code_word);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_TRK_H */
