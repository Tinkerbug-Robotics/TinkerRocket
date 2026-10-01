/*
 * The adaptive notch stage, bit exact: the reference the HDL is verified against. Proposed in
 * milestone 7's interference study; the owner approved the formats on 2026-10-01. It sits between
 * the L1 decimator and the correlators (and the acquisition snapshot path), on 2-bit
 * sign/magnitude codes (fe_format.h), and hands 2-bit codes on, so the correlators are unchanged.
 *
 * Notation: integers throughout. rnd(v, s) = (v + 2^(s-1)) >> s for s > 0 (an arithmetic shift,
 * so halves round up), v << -s for s <= 0. sat(v) clips to +-(2^(NOTCH_IA + NOTCH_FA) - 1).
 * Complex products: (a + jb)(c + jd) = (ac - bd) + j(ad + bc); conj(c + jd) = c - jd.
 *
 * Per sample, the notches in cascade (NOTCH_N of them), each with state zacc (complex), ar
 * (complex, the pole section's last output) and P (its power):
 *   x    = the sample's weights (+-1, +-3) << NOTCH_FA for the first notch, the previous notch's
 *          y after it (NOTCH_FA fraction bits)
 *   z    = rnd(zacc, NOTCH_G)                                   Q1.16: 18 bits with the sign
 *   p    = rnd(z * ar, NOTCH_FZ)                                the pole's prediction
 *   arn  = sat(x + p - rnd(p, NOTCH_K))                         the pole section: k = 1 - 2^-K
 *   y    = sat(arn - p)                                         the zero: the notch's output
 *   P   += rnd(|ar|^2 - P, NOTCH_PLEAK); P = max(P, 1)          |ar|^2 = re^2 + im^2
 *   zacc += rnd(y * conj(ar), NOTCH_M + msb(P) - NOTCH_FZ - NOTCH_G)
 *          (msb(P): the index of P's highest set bit; the step is 2^-M over P's power of two)
 *   if |z|^2 > (2^FZ + 2^(FZ - K - 1))^2: zacc -= rnd(zacc, 8)  (keeps k |z| < 1)
 *   ar   = arn
 * Then the requantizer, on the last notch's y, per component (I and Q alike):
 *   sign bit = y < 0;  magnitude bit = (|y| << NOTCH_TF) > T
 *   after every NOTCH_CHUNK samples, with m the magnitude bits set in them (of 2 * CHUNK):
 *   T += (T * (m - NOTCH_TARGET)) >> NOTCH_AGC_SHIFT  (an arithmetic shift; T >= 1)
 * holding the magnitude bits at a third, as the MAX2769B's own AGC does (a 10 ms time constant).
 * And the power counters the P4 reads each millisecond, the interference detector:
 *   pin  += |w|^2 of the input weights;  pout += |y|^2 >> (2 NOTCH_FA)
 *
 * Resources (estimate): per notch per sample 12 real products, all within 18 x 18; at a 108 MHz
 * fabric clock, 16 cycles a sample, one or two multipliers a notch. State: under 200 bits a notch.
 */
#ifndef GNSS_FPGA_NOTCH_H
#define GNSS_FPGA_NOTCH_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NOTCH_N         2          /* notches in cascade */
#define NOTCH_K         7          /* pole factor 1 - 2^-7: about 17 kHz wide at 6.75 MS/s */
#define NOTCH_M         10         /* step 2^-10 over the power's leading bit */
#define NOTCH_FZ        16         /* z at the multipliers: Q1.16 */
#define NOTCH_G         12         /* zacc's guard bits: Q1.28 */
#define NOTCH_FA        4          /* fraction bits of the pole state and the output */
#define NOTCH_IA        11         /* integer bits of the pole state and the output (16 with sign) */
#define NOTCH_PLEAK     8          /* the power's average: 2^-8 a sample */
#define NOTCH_TF        16         /* the requantizer threshold's extra fraction bits */
#define NOTCH_CHUNK     256        /* samples per AGC step */
#define NOTCH_TARGET    169        /* magnitude bits a chunk: a third of 512 */
#define NOTCH_AGC_SHIFT 16         /* the AGC's gain: 2^-16 per bit of error */
/* The threshold at reset: 0.9674 times the rms of a component's weights (1.908), the level for a
 * third, in NOTCH_FA + NOTCH_TF fraction bits. */
#define NOTCH_T0        1935653

typedef struct {
    int64_t zr, zi;      /* zacc */
    int64_t ar, ai;      /* the pole section's last output */
    int64_t p;           /* its power */
} notch_sec_t;

typedef struct {
    notch_sec_t s[NOTCH_N];
    int64_t t;           /* the requantizer's threshold */
    int fill, mags;      /* samples into the AGC chunk, magnitude bits in it */
    uint64_t pin, pout;  /* the power counters, since the last take */
    int64_t yr, yi;      /* the last sample's y before the requantizer (for tests and vectors) */
} notch_t;

void notch_init(notch_t *n);

/* One sample: an input code (fe_format.h) in, an output code out. */
uint8_t notch_sample(notch_t *n, uint8_t code);

/* n codes in and out; in and out may be the same buffer. */
void notch_process(notch_t *n, const uint8_t *in, uint8_t *out, size_t count);

/* The power counters since the last take, then cleared (the P4's read each millisecond). */
void notch_take_power(notch_t *n, uint64_t *pin, uint64_t *pout);

/* A notch's centre (cycles per sample, -0.5..0.5) and depth |z| (1 = full), for reports. */
void notch_zero(const notch_t *n, int s, double *f_cyc, double *depth);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_FPGA_NOTCH_H */
